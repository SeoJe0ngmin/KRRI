"""Single RealSense owner: stereo+ordered IMU to OpenVINS, colour tags separately."""
import copy
import json
import queue
import signal
import threading
import time
from collections import deque
from pathlib import Path
from .stream import Latest,ImuSynchronizer,CaptureClock,publish_latest
from .recording import Recorder


def worker(cfg,stop,results,previews,output_dir,view=False,motion_hint=None,fast_imu=None,setup=None):
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    if hasattr(signal,'SIGHUP'):signal.signal(signal.SIGHUP,signal.SIG_IGN)
    pipes=[];threads=[];bridge=None;mapping=None;rec=Recorder(output_dir,cfg['logging']);errors=queue.Queue()
    frames=Latest();tags=Latest();tag_results=Latest();preview_tasks=Latest()
    enabled=threading.Event();started=time.monotonic();clock=CaptureClock();epoch=time.monotonic_ns() & ((1<<63)-1)
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
        bundle,maps,Rga,estimator_file=prepare(cfg,profiles,serial,Path(output_dir)/'calibration',cfg.get('sensor_calibration'))
        rec.put(bundle,'device.json');observer=Observer(cfg)
        if not cfg.get('camera_only'):
            from .ros_adapter import Bridge
            bridge=Bridge(estimator_file,output_dir,epoch,cfg['openvins']['max_input_age_s'])
            if cfg.get('mapping',{}).get('enabled'):
                from .mapping import Mapping
                mapping=Mapping(bridge,bundle,cfg,output_dir)
        if setup is not None:setup.put(dict(calibration=cfg['calibration'],device=bundle),timeout=2)
        imu=ImuSynchronizer(c['imu_capacity'],c['max_imu_gap_s'])
        def timestamp(frame):
            if frame.get_frame_timestamp_domain()!=rs.timestamp_domain.hardware_clock:raise ValueError('RealSense hardware clock required')
            return frame.get_timestamp()/1000.
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
        def capture():
            previous=None
            while not stop.is_set():
                try:
                    fs=vpipe.wait_for_frames(1000);number=fs.get_infrared_frame(1).get_frame_number()
                    raw_received[0]+=1
                    if previous is not None:sdk_gaps[0]+=max(0,number-previous-1)
                    previous=number;frames.put((fs,time.monotonic()))
                except RuntimeError as exc:
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
            from .webview import encode_preview
            while not stop.is_set():
                item=preview_tasks.get()
                if item is None:stop.wait(.01);continue
                image,obs=item
                try:publish_latest(previews,(obs['capture_id'],encode_preview(image,obs,cfg['view'])))
                except Exception as exc:errors.put('preview: '+str(exc));return
        enabled.set()
        for fn in (capture,detect,preview):
            th=threading.Thread(target=fn,daemon=True);threads.append(th);th.start()
        last_color=-1.;last_tag=-1.;last_view=-1.;pending=deque(maxlen=32);latest_image=None;last_state_at=time.monotonic()
        while not stop.is_set():
            if not errors.empty():raise RuntimeError(errors.get_nowait())
            if imu.fault:raise RuntimeError(imu.fault)
            for sample in imu.drain():
                if bridge:bridge.imu(sample)
            item=frames.get()
            if item is not None:
                fs,arrival=item;l,r,cf=fs.get_infrared_frame(1),fs.get_infrared_frame(2),fs.get_color_frame()
                if not l or not r or not cf:raise RuntimeError('missing IR stereo/color frames')
                stamp=timestamp(l);right_stamp=timestamp(r);color_stamp=timestamp(cf)
                if abs(stamp-right_stamp)>c['stereo_skew_s']:raise RuntimeError('unsynchronized_stereo')
                clock.observe('left',stamp,arrival);received+=1
                left=cv2.remap(np.asanyarray(l.get_data()),*maps[0],cv2.INTER_LINEAR)
                right=cv2.remap(np.asanyarray(r.get_data()),*maps[1],cv2.INTER_LINEAR)
                latest_image=(stamp,left)
                if mapping:mapping.image(stamp,left,right)
                if bridge:bridge.stereo(left,right,stamp)
                if color_stamp>last_color and color_stamp-last_tag>=cfg['tag']['period_s']:
                    tags.put((fs,color_stamp));last_tag=color_stamp
                last_color=max(last_color,color_stamp)
                if cfg.get('camera_only'):
                    last_obs=dict(capture_id=stamp,arrival=arrival,sensor_epoch=epoch,sensor_ready=False,local_valid=False,
                        local_pose=[0.,0.,0.],pose=None,usable=False,stationary=False,clock_fault=False,imu_bad=False,
                        transport_lag_s=max(0.,time.monotonic()-clock.host_time(stamp)),tag=copy.deepcopy(latest_tag),
                        odom=dict(ok=False,reason='camera_only_no_vio'),imu=dict(ready=False,calibration=dict(state='DISABLED',reason='camera_only')))
            tag=tag_results.get()
            if tag is not None:
                latest_tag=tag;rec.put(dict(kind='tag',value=tag))
                if len(pending)==pending.maxlen:pending.popleft()
                pending.append(tag)
            if bridge:
                packets=bridge.drain()
                for packet in packets:
                    if packet['fault']:raise RuntimeError(packet['fault'])
                    last_state_at=time.monotonic();packet['imu_arrival']=min(imu.arrival,packet['arrival'])
                    packet['transport_lag_s']=max(0.,packet['arrival']-clock.host_time(packet['stamp']))
                    neutral=True
                    if motion_hint is not None:
                        with motion_hint.get_lock():
                            neutral=bool(motion_hint[1]);packet['neutral_since']=motion_hint[2]
                        packet['captured_after_neutral']=clock.host_time(packet['stamp'])>=packet['neutral_since']
                    # Insert current VIO before delayed colour measurements.
                    obs=observer.add_vio(packet,neutral)
                    while pending and observer.last_state and pending[0]['stamp']<=observer.last_state.stamp:
                        observer.add_tag(pending.popleft())
                    if fast_imu is not None:
                        with fast_imu.get_lock():fast_imu[7]=float(packet['initialized'])
                    if obs is None:
                        obs=dict(capture_id=packet['stamp'],arrival=packet['arrival'],pose=None,local_pose=[0.,0.,0.],
                            local_valid=False,usable=False,sensor_ready=False,imu_bad=False,clock_fault=False,stationary=False,
                            tag=copy.deepcopy(latest_tag),odom=dict(ok=False,reason='openvins_initializing'),
                            imu=dict(ready=False,calibration=dict(state='COLLECTING',reason='openvins_stationary_initialization')))
                    else:processed+=1
                    obs['device_model']=bundle['raw_models']['color'];obs['vio_gyro_norm']=packet.get('gyro_norm')
                    rec.put(dict(kind='vio',value=packet));last_obs=obs
                    if mapping:mapping.state(packet)
                if time.monotonic()-last_state_at>10:raise RuntimeError('OpenVINS output timeout')
            if last_obs is not None:
                elapsed=max(.001,time.monotonic()-started)
                last_obs['performance']=dict(camera_fps=raw_received[0]/elapsed,stereo_sent_fps=received/elapsed,sdk_frame_gaps=sdk_gaps[0],processed_fps=processed/elapsed,
                    latest_replaced=frames.dropped,tag_replaced=tags.dropped,ros_state_dropped=bridge.dropped if bridge else 0)
                last_obs['recording']=rec.status()
                if mapping:last_obs['mapping']=mapping.status()
                rec.put(last_obs);publish_latest(results,last_obs)
                if view and latest_image and time.monotonic()-last_view>=1/cfg['view']['fps']:
                    # Only overlay features from a sufficiently close stereo frame.
                    picture=copy.deepcopy(last_obs)
                    if abs(latest_image[0]-picture['capture_id'])>.05:picture['odom']['tracked_points']=[]
                    preview_tasks.put((latest_image[1].copy(),picture));last_view=time.monotonic()
                last_obs=None
            if rec.status()['error']:raise RuntimeError(rec.status()['error'])
            stop.wait(.002)
    except Exception as exc:
        # A fault notification must reach the supervisor; stale data is also
        # rejected independently by its age gate and the CAN process lease.
        try:results.put(dict(worker_error=type(exc).__name__+': '+str(exc)),timeout=.5)
        except queue.Full:publish_latest(results,dict(worker_error=str(exc)))
    finally:
        enabled.clear();stop.set()
        for pipe in reversed(pipes):
            try:pipe.stop()
            except Exception:pass
        for th in threads:th.join(1)
        if mapping:mapping.close()
        if bridge:bridge.close()
        rec.put(dict(camera_received=raw_received[0],stereo_sent=received,sdk_frame_gaps=sdk_gaps[0],processed=processed,imu_samples=imu.sequence if imu else 0,
            imu_fault=imu.fault if imu else None),'metrics.json');rec.close()
