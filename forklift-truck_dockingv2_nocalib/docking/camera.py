"""Single RealSense owner: stereo+ordered IMU to OpenVINS, colour tags separately."""
import copy
import gc
import json
import queue
import signal
import threading
import time
import traceback
from collections import deque
from pathlib import Path
from .stream import Latest,ImuSynchronizer,CaptureClock,StereoSequence,OutputWatchdog,VideoDeadline,publish_latest,ClockUnwrap

IMU_SHORT_GAP_S=.1   # one gyro/accel gap up to this, once per second, is tolerated (Python and native bridge use the same rule)
from .recording import Recorder


def worker(cfg,stop,results,previews,output_dir,view=False,motion_hint=None,fast_imu=None,setup=None):
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    if hasattr(signal,'SIGHUP'):signal.signal(signal.SIGHUP,signal.SIG_IGN)
    pipes=[];threads=[];bridge=None;mapping=None;rec=Recorder(output_dir,cfg['logging']);errors=queue.Queue()
    frames=Latest();tags=Latest();tag_results=Latest();preview_tasks=Latest()
    enabled=threading.Event();started=time.monotonic();clock=CaptureClock();epoch=time.monotonic_ns() & ((1<<63)-1)
    stereo_sequence=StereoSequence()
    received=0;raw_received=[0];sdk_gaps=[0];processed=0;last_obs=None;latest_tag=dict(ok=False,reason='not_seen');imu=None
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
        from .sensor_calibration import prepare,device_profiles
        from .observer import Observer
        from .tag import TagObserver
        cv2.setNumThreads(cfg['opencv_threads'])
        c=cfg['camera'];context=rs.context();ipipe=rs.pipeline(context);ic=rs.config()
        if c['serial']:ic.enable_device(c['serial'])
        ic.enable_stream(rs.stream.gyro,rs.format.motion_xyz32f,c['gyro_hz']);ic.enable_stream(rs.stream.accel,rs.format.motion_xyz32f,c['accel_hz'])
        ip=ic.resolve(ipipe);serial=ip.get_device().get_info(rs.camera_info.serial_number)
        vpipe=rs.pipeline(context);vc=rs.config();vc.enable_device(serial)
        for index in (1,2):vc.enable_stream(rs.stream.infrared,index,c['ir_width'],c['ir_height'],rs.format.y8,c['fps'])
        vc.enable_stream(rs.stream.color,c['color_width'],c['color_height'],rs.format.bgr8,c['fps'])
        if c['depth_enabled']:vc.enable_stream(rs.stream.depth,c['ir_width'],c['ir_height'],rs.format.z16,c['fps'])
        vp=vc.resolve(vpipe)
        for sensor in vp.get_device().query_sensors():
            if sensor.supports(rs.option.global_time_enabled):sensor.set_option(rs.option.global_time_enabled,0.)
            if sensor.supports(rs.option.enable_auto_exposure):sensor.set_option(rs.option.enable_auto_exposure,float(c['auto_exposure']))
            if sensor.supports(rs.option.emitter_enabled):sensor.set_option(rs.option.emitter_enabled,float(c['emitter_enabled']))
        profiles=(vp.get_stream(rs.stream.infrared,1),vp.get_stream(rs.stream.infrared,2),vp.get_stream(rs.stream.color),ip.get_stream(rs.stream.gyro),ip.get_stream(rs.stream.accel))
        # Keep the common factory device alive while resolving inter-sensor calibration.
        calibration_device=vp.get_device()
        profiles=device_profiles(calibration_device,profiles)
        bundle,maps,Rga,estimator_file=prepare(cfg,profiles,serial,Path(output_dir)/'calibration')
        rec.put(bundle,'device.json');observer=Observer(cfg)
        if not cfg.get('camera_only'):
            from .ros_adapter import Bridge
            bridge=Bridge(estimator_file,output_dir,epoch,cfg['openvins']['max_input_age_s'],cancel=stop,imu_max_gap=c['max_imu_gap_s'],imu_short_gap=IMU_SHORT_GAP_S)
            if cfg.get('mapping',{}).get('enabled'):
                from .mapping import Mapping
                mapping=Mapping(bridge,bundle,cfg,output_dir)
        if setup is not None:setup.put(dict(calibration=cfg['calibration'],device=bundle),timeout=2)
        imu=ImuSynchronizer(c['imu_capacity'],c['max_imu_gap_s'],startup_window_s=.5,short_gap_s=IMU_SHORT_GAP_S)
        unwrap=ClockUnwrap()   # 32-bit device clock wraps every 71.6 min; without this every stream reports a clock reversal
        def timestamp(frame):
            if frame.get_frame_timestamp_domain()!=rs.timestamp_domain.hardware_clock:raise ValueError('RealSense hardware clock required')
            return unwrap(str(frame.get_profile().stream_type()),frame.get_timestamp()/1000.)
        def motion(frame):
            if not enabled.is_set() or not frame.is_motion_frame():return
            try:
                f=frame.as_motion_frame();kind='gyro' if f.get_profile().stream_type()==rs.stream.gyro else 'accel'
                v=f.get_motion_data();v=np.array([v.x,v.y,v.z]);arrival=time.monotonic()
                if kind=='accel':v=Rga@v
                imu.add(kind,timestamp(f),v,arrival)
                if fast_imu is not None and kind=='gyro':
                    with fast_imu.get_lock():fast_imu[1]=arrival;fast_imu[6]=float(bool(imu.fault))
            except Exception as exc:errors.put(str(exc))
        ipipe.start(ic,motion);pipes.append(ipipe);vpipe.start(vc);pipes.append(vpipe)
        depth_scale=vp.get_device().first_depth_sensor().get_depth_scale() if c['depth_enabled'] else 1.
        deadline=VideoDeadline(time.monotonic(),c['startup_timeout_s']);delivery=deque(maxlen=12)
        def capture():
            # 2026-10-19 hand-held runs: 1-2 s holes in the VIO output with no Python stall and no frame backlog ->
            # the SDK delivered nothing. Record every delivery gap > 0.15 s with frame-number and device-clock deltas
            # so the next run can tell "sensor paused" (numbers continuous) from "frames lost on USB" (numbers jump).
            previous=None;prev_host=None;prev_stamp=None
            while not stop.is_set():
                try:
                    deadline.check(time.monotonic())
                    fs=vpipe.poll_for_frames()
                    if not fs:
                        stop.wait(.005);continue
                    l,r,cf=fs.get_infrared_frame(1),fs.get_infrared_frame(2),fs.get_color_frame()
                    if not l or not r or not cf:
                        deadline.incomplete+=1;stop.wait(.005);continue
                    number=l.get_frame_number()
                    if not stereo_sequence.accept(number,r.get_frame_number(),timestamp(l),timestamp(r)):
                        # A repeated frameset is not a fresh camera observation.
                        stop.wait(.005);continue
                    last_frame=time.monotonic();deadline.accepted(last_frame);raw_received[0]+=1
                    if previous is not None and not stereo_sequence.counter_anomalies:sdk_gaps[0]+=max(0,number-previous-1)
                    ts=timestamp(l)
                    if prev_host is not None and last_frame-prev_host>.15:
                        delivery.append(dict(t=round(ts,3),host_gap_s=round(last_frame-prev_host,3),stamp_gap_s=round(ts-prev_stamp,3),
                                             frame_delta=number-previous,incomplete_so_far=deadline.incomplete))
                    prev_host=last_frame;prev_stamp=ts
                    previous=number;frames.put((fs,last_frame))
                except Exception as exc:
                    if not stop.is_set():errors.put('video: '+str(exc))
                    return
        def detect():
            try:
                detector=TagObserver(cfg) if cfg['tag']['enabled'] else None
                align=rs.align(rs.stream.color)
                while not stop.is_set():
                    item=tags.get()
                    if item is None:stop.wait(.005);continue
                    fs,stamp=item;cf=fs.get_color_frame();depth=None
                    if c['depth_enabled']:
                        df=fs.get_depth_frame()
                        if df and abs(timestamp(df)-stamp)<=c['depth_color_skew_s']:
                            aligned=align.process(fs).get_depth_frame()
                            if aligned:depth=np.asanyarray(aligned.get_data()).astype(np.float32)*depth_scale
                    tag=detector.process(np.asanyarray(cf.get_data()),bundle['raw_models']['color'],stamp,epoch,depth) if detector else dict(ok=False,reason='disabled',stamp=stamp,sensor_epoch=epoch)
                    tag['capture_key']=str(cf.get_frame_number());tag_results.put(tag)
            except Exception as exc:errors.put('tag: '+str(exc))
        def preview():
            try:
                from .webview import encode_preview
                while not stop.is_set():
                    item=preview_tasks.get()
                    if item is None:stop.wait(.01);continue
                    image,obs=item
                    publish_latest(previews,(obs['capture_id'],encode_preview(image,obs,cfg['view'])))
            except Exception as exc:errors.put('preview: '+str(exc))
        enabled.set()
        acquisition_started=time.monotonic()
        for fn in ((capture,detect,preview) if view else (capture,detect)):
            th=threading.Thread(target=fn,daemon=True);threads.append(th);th.start()
        last_color=-1.;last_tag=-1.;last_view=-1.;pending=deque(maxlen=32);latest_image=None;output_watchdog=OutputWatchdog();have_vio_output=False;last_diagnostic=0.;ir_mean=None;last_frames=None;exposure=None
        # Stall diagnostics (2026-10-19): the whole worker froze 0.3-1.1 s every few seconds with no backlog anywhere.
        # Record garbage-collector pauses (every thread stops for them) and the worst time of each main-loop stage
        # per second. Record only; nothing here changes behaviour.
        gc_stat=dict(count=0,max_ms=0.,total_ms=0.,gen2=0,_t0=None)
        def gc_callback(phase,info):
            if phase=='start':gc_stat['_t0']=time.monotonic()
            elif gc_stat['_t0'] is not None:
                ms=(time.monotonic()-gc_stat['_t0'])*1000.;gc_stat['count']+=1;gc_stat['total_ms']+=ms;gc_stat['max_ms']=max(gc_stat['max_ms'],ms)
                if info.get('generation')==2:gc_stat['gen2']+=1
        gc.callbacks.append(gc_callback)
        stage_max={};stage_t=[time.monotonic()]
        def stage(name):
            now=time.monotonic();ms=(now-stage_t[0])*1000.;stage_t[0]=now
            if ms>stage_max.get(name,0.):stage_max[name]=ms
        def metadata(frame):
            out={}
            for name in ('actual_exposure','actual_fps','gain_level','auto_exposure'):
                key=getattr(rs.frame_metadata_value,name,None)
                try:
                    if key is not None and frame.supports_frame_metadata(key):out[name]=int(frame.get_frame_metadata(key))
                except Exception:pass
            return out
        while not stop.is_set():
            if not errors.empty():raise RuntimeError(errors.get_nowait())
            if imu.fault:raise RuntimeError(imu.fault)
            if not imu.startup_ready and time.monotonic()-acquisition_started>c['startup_timeout_s']:
                raise RuntimeError('IMU/video startup continuity timeout: '+json.dumps(imu.status()))
            stage('idle')
            for sample in imu.drain():
                if bridge:bridge.imu(sample)
                observer.add_gyro(sample['stamp'],sample['gyro'])   # stationary judgment by raw gyro (2026-10-19)
            stage('imu_publish')
            item=frames.get()
            if item is not None:
                fs,arrival=item;l,r,cf=fs.get_infrared_frame(1),fs.get_infrared_frame(2),fs.get_color_frame()
                if not l or not r or not cf:raise RuntimeError('missing IR stereo/color frames')
                stamp=timestamp(l);right_stamp=timestamp(r);color_stamp=timestamp(cf)
                if abs(stamp-right_stamp)>c['stereo_skew_s']:raise RuntimeError('unsynchronized_stereo')
                clock.observe('left',stamp,arrival)
                left=cv2.remap(np.asanyarray(l.get_data()),*maps[0],cv2.INTER_LINEAR)
                right=cv2.remap(np.asanyarray(r.get_data()),*maps[1],cv2.INTER_LINEAR)
                ir_mean=float(left.mean());last_frames=(l,cf)   # scene brightness for the log: dark IR = few features
                # Preview: IR left with OpenVINS points normally; the colour frame with the tag outline in camera-only
                # mode, where there is no VIO and the colour image is what the tag detector sees (2026-10-19).
                latest_image=(color_stamp,np.asanyarray(cf.get_data())) if cfg.get('camera_only') else (stamp,left)
                stage('remap')
                if imu.begin(stamp,time.monotonic()):
                    # The first image cannot enter OpenVINS until its clean
                    # continuous IMU history is sent. This is acquisition
                    # warmup, not inertial/stationary calibration readiness.
                    for sample in imu.drain():
                        if bridge:bridge.imu(sample)
                        observer.add_gyro(sample['stamp'],sample['gyro'])
                if mapping and imu.startup_ready:mapping.image(stamp,left,right)
                if bridge and imu.startup_ready:
                    bridge.stereo(left,right,stamp)
                    received+=1;stage('stereo_publish')
                    # Native output cannot exist before its first video input.
                    # The independent camera startup timeout covers this phase.
                    output_watchdog.input(time.monotonic())
                if color_stamp>last_color and color_stamp-last_tag>=cfg['tag']['period_s']:
                    tags.put((fs,color_stamp));last_tag=color_stamp
                last_color=max(last_color,color_stamp)
                if cfg.get('camera_only') or not have_vio_output:
                    last_obs=dict(preview_only=True,capture_id=stamp,arrival=arrival,sensor_epoch=epoch,sensor_ready=False,local_valid=False,
                        local_pose=[0.,0.,0.],pose=None,usable=False,stationary=False,clock_fault=False,imu_bad=False,
                        transport_lag_s=max(0.,time.monotonic()-clock.host_time(stamp)),tag=copy.deepcopy(latest_tag),
                        odom=dict(ok=False,reason='camera_only_no_vio' if cfg.get('camera_only') else
                            'waiting_for_openvins_output' if imu.startup_ready else 'imu_startup_stabilizing'),
                        imu=dict(ready=False,calibration=dict(state='DISABLED' if cfg.get('camera_only') else 'COLLECTING',
                            reason='camera_only' if cfg.get('camera_only') else 'waiting_for_openvins_output')))
            tag=tag_results.get()
            if tag is not None:
                latest_tag=tag;rec.put(dict(kind='tag',value=tag))
                if len(pending)==pending.maxlen:pending.popleft()
                pending.append(tag)
            if bridge:
                packets=bridge.drain();stage('vio_drain')
                for packet in packets:
                    if packet['fault']:raise RuntimeError(packet['fault'])
                    output_watchdog.output(time.monotonic());have_vio_output=True;packet['imu_arrival']=min(imu.arrival,packet['arrival'])
                    packet['transport_lag_s']=max(0.,packet['arrival']-clock.host_time(packet['stamp']))
                    packet['imu_gaps']=imu.status()['gaps']
                    neutral=True
                    if motion_hint is not None:
                        with motion_hint.get_lock():
                            neutral=bool(motion_hint[1]);packet['neutral_since']=motion_hint[2];packet['baseline_lock']=bool(motion_hint[3]) if len(motion_hint)>3 else False
                        packet['captured_after_neutral']=clock.host_time(packet['stamp'])>=packet['neutral_since']
                        packet['since_neutral_s']=clock.host_time(packet['stamp'])-packet['neutral_since'] if neutral else None
                    # Insert current VIO before delayed colour measurements.
                    obs=observer.add_vio(packet,neutral);stage('add_vio')
                    while pending and observer.last_state and pending[0]['stamp']<=observer.last_state.stamp:
                        observer.add_tag(pending.popleft())
                    stage('add_tag')
                    if fast_imu is not None:
                        with fast_imu.get_lock():fast_imu[7]=float(packet['initialized'])
                    if obs is None:
                        obs=dict(capture_id=packet['stamp'],arrival=packet['arrival'],pose=None,local_pose=[0.,0.,0.],
                            local_valid=False,usable=False,sensor_ready=False,imu_bad=False,clock_fault=False,stationary=False,
                            tag=copy.deepcopy(latest_tag),odom=dict(ok=False,reason='openvins_initializing'),
                            imu=dict(ready=False,calibration=dict(state='COLLECTING',reason='openvins_stationary_initialization')))
                    else:processed+=1
                    obs['device_model']=bundle['raw_models']['color'];obs['vio_gyro_norm']=packet.get('gyro_norm')
                    # Record diet (2026-10-19): the 250 tracked-point coordinates are for the live preview only;
                    # the log keeps their count (3 KB per packet and 3.5 KB per observation saved, half the file).
                    rec.put(dict(kind='vio',value=dict({k:v for k,v in packet.items() if k!='points'},points_n=len(packet.get('points') or []))));last_obs=obs
                    if mapping:mapping.state(packet)
                diagnostic=dict(camera_received=raw_received[0],stereo_sent=received,imu_paired=imu.sequence,
                    last_imu_device_stamp=imu.last.get('gyro'),last_video_device_stamp=clock.last.get('left'),
                    vio_output_seen=have_vio_output,imu=imu.status(),transport=bridge.status())
                if time.monotonic()-last_diagnostic>=1:
                    if last_frames is not None:exposure=dict(ir=metadata(last_frames[0]),color=metadata(last_frames[1]))
                    diagnostic.update(delivery_gaps=list(delivery),incomplete_framesets=deadline.incomplete,exposure=exposure,
                        gc=dict((k,v) for k,v in gc_stat.items() if not k.startswith('_')),gc_counts=gc.get_count(),loop_stage_max_ms={k:round(v,1) for k,v in stage_max.items()})
                    rec.put(diagnostic,'pipeline_status.json');rec.put(dict(kind='health',value=dict(t=time.monotonic(),gc=diagnostic['gc'],gc_counts=diagnostic['gc_counts'],loop_stage_max_ms=diagnostic['loop_stage_max_ms'],delivery_gaps=diagnostic['delivery_gaps'][-3:])))
                    gc_stat.update(count=0,max_ms=0.,total_ms=0.,gen2=0);stage_max.clear();last_diagnostic=time.monotonic()
                if output_watchdog.expired(time.monotonic()):
                    log=Path(output_dir)/'openvins.log'
                    tail=log.read_text(errors='replace')[-6000:] if log.exists() else '(no openvins.log)'
                    raise RuntimeError('OpenVINS output timeout; pipeline='+json.dumps(diagnostic)+'\nOpenVINS log:\n'+tail)
            if last_obs is not None:
                elapsed=max(.001,time.monotonic()-started)
                last_obs['performance']=dict(camera_fps=raw_received[0]/elapsed,stereo_sent_fps=received/elapsed,sdk_frame_gaps=sdk_gaps[0],processed_fps=processed/elapsed,ir_mean=ir_mean,delivery_gaps=list(delivery),incomplete_framesets=deadline.incomplete,exposure=exposure,
                    stereo_counter_anomalies=stereo_sequence.counter_anomalies,sdk_gap_count_reliable=not bool(stereo_sequence.counter_anomalies),stereo_duplicates=stereo_sequence.duplicates,latest_replaced=frames.dropped,tag_replaced=tags.dropped,ros_state_dropped=bridge.dropped if bridge else 0)
                last_obs['recording']=rec.status()
                if mapping:last_obs['mapping']=mapping.status()
                record=dict(last_obs,odom=dict({k:v for k,v in last_obs['odom'].items() if k!='tracked_points'},tracked_points_n=len(last_obs['odom'].get('tracked_points') or [])))
                rec.put(record);publish_latest(results,last_obs);stage('record_publish')
                if view and latest_image and time.monotonic()-last_view>=1/cfg['view']['fps']:
                    # Only overlay features from a sufficiently close stereo frame.
                    picture=copy.deepcopy(last_obs);picture['preview_stream']='color' if cfg.get('camera_only') else 'infrared'
                    if abs(latest_image[0]-picture['capture_id'])>.05:picture['odom']['tracked_points']=[]
                    preview_tasks.put((latest_image[1].copy(),picture));last_view=time.monotonic();stage('preview_put')
                last_obs=None
            if rec.status()['error']:raise RuntimeError(rec.status()['error'])
            stop.wait(.002)
    except Exception as exc:
        # Persist synchronously BEFORE setting stop: multiprocessing.Queue uses
        # an asynchronous feeder, so stop can reach the parent before its error.
        try:
            (Path(output_dir)/'camera_error.txt').write_text(traceback.format_exc())
        except OSError:
            traceback.print_exc()
        # A fault notification must reach the supervisor; stale data is also
        # rejected independently by its age gate and the CAN process lease.
        try:results.put(dict(worker_error=type(exc).__name__+': '+str(exc)),timeout=.5)
        except queue.Full:publish_latest(results,dict(worker_error=str(exc)))
    finally:
        enabled.clear();stop.set()
        # Finish nonblocking SDK polling/alignment before closing its streams.
        for th in threads:th.join(1)
        for pipe in reversed(pipes):
            try:pipe.stop()
            except Exception:pass
        for resource in (mapping,bridge):
            if resource:
                try:resource.close()
                except Exception:traceback.print_exc()
        rec.put(dict(camera_received=raw_received[0],stereo_sent=received,sdk_frame_gaps=sdk_gaps[0],processed=processed,imu_samples=imu.sequence if imu else 0,
            imu_fault=imu.fault if imu else None,imu=imu.status() if imu else None,stereo_duplicates=stereo_sequence.duplicates,stereo_counter_anomalies=stereo_sequence.counter_anomalies,sdk_gap_count_reliable=not bool(stereo_sequence.counter_anomalies)),'metrics.json');rec.close()
