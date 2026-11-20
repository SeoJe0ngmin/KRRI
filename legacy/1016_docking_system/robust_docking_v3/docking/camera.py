"""RealSense capture/vision process; latest-only video, every-sample gyro."""
import queue
import threading
import time
import signal
import resource
import sys

from .stream import Latest, Imu, Metrics
from .exposure import configure_exposure, frame_exposure, recovery_camera_settings
from .recording import Recorder
from .diagnostics import StillnessReport
from .calibration import stationary_evidence


def publish_latest(q, item):
    try:
        q.put_nowait(item)
        return
    except queue.Full:
        pass
    try:
        q.get_nowait()
    except queue.Empty:
        pass
    try:
        q.put_nowait(item)
    except queue.Full:
        pass  # multiprocessing feeder may still be moving the previous item


def configure_device_clock(profile, rs):
    """Select device time on the resolved sensors BEFORE starting callbacks.

    SDK global-time conversion can become ready between gyro/accel frames.
    All streams here belong to one device; host monotonic arrival time is
    recorded separately and is never substituted for an exposure timestamp.
    """
    active = {p.stream_type() for p in profile.get_streams()}
    settings = []
    for sensor in profile.get_device().query_sensors():
        if not any(p.stream_type() in active for p in sensor.get_stream_profiles()):
            continue
        supported = sensor.supports(rs.option.global_time_enabled)
        value = None
        if supported:
            sensor.set_option(rs.option.global_time_enabled, 0.)
            value = sensor.get_option(rs.option.global_time_enabled)
            if value != 0.:
                raise RuntimeError('could not disable RealSense global timestamp conversion')
        settings.append({'sensor': sensor.get_info(rs.camera_info.name),
                         'global_time_supported': supported,
                         'global_time_enabled': value})
    # A sensor without this option is still checked on every actual frame.
    return settings


def receive_motion(frame, rs, imu, domains, fast_imu=None):
    """Reject incompatible timestamps BEFORE calibration or integration."""
    if not frame.is_motion_frame():
        return
    f = frame.as_motion_frame()
    stream = f.get_profile().stream_type()
    if stream not in (rs.stream.gyro, rs.stream.accel):
        return
    kind = 'gyro' if stream == rs.stream.gyro else 'accel'
    domain = str(f.get_frame_timestamp_domain())
    domains[kind] = domain
    if domains['fault']:
        return
    if domain != str(rs.timestamp_domain.hardware_clock):
        reason = 'unexpected_imu_clock_domain: %s=%s' % (kind, domain)
        domains['fault'] = reason
        imu.calibration_error(reason)
        if fast_imu is not None:
            with fast_imu.get_lock():
                fast_imu[7] = 0.
        return
    v = f.get_motion_data()
    stamp = f.get_timestamp()/1000.
    if kind == 'gyro':
        imu.add(stamp, (v.x, v.y, v.z))
        if fast_imu is not None:
            imu.publish_fast(fast_imu)
    else:
        imu.add_accel(stamp, (v.x, v.y, v.z))


def worker(cfg, stop, results, previews, output_dir, view=False, bag=None, motion_hint=None, fast_imu=None, sensor_request=None):
    """Never imports the CAN backend. Exceptions are sent to supervisor."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    if hasattr(signal, 'SIGHUP'):
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
    pipes, capture_thread = [], None
    metrics, latest = Metrics(), Latest()
    started = time.monotonic()
    received = [0]
    sdk_drops = [0]
    clock_errors = [0]
    last_num = [None]
    latest_performance = None
    usage0 = resource.getrusage(resource.RUSAGE_SELF)
    recorder = Recorder(output_dir, cfg.get('logging', {}))
    jitter = StillnessReport(cfg.get('diagnostics', {}))
    jitter_report = None
    last_report = 0.
    saved_calibration = saved_camera_model = False
    saved_stationary_updates = 0
    last_exposure_request = 0
    imu = None
    motion_domains = {'gyro': None, 'accel': None, 'fault': None}
    timestamp_domains = None
    reported_calibration_state = None
    try:
        import numpy as np
        import cv2
        import pyrealsense2 as rs
        from .vision import Observer
        imu = Imu(cfg['imu'])
        device_serial = cfg['camera'].get('serial')
        if bag:
            raise ValueError('SDK bag replay is not enabled here; use replay for observations.jsonl')
        # Motion module first, matching the working D435i/RSUSB arrangement.
        context = rs.context()
        imupipe, ic = rs.pipeline(context), rs.config()
        if device_serial:
            ic.enable_device(device_serial)
        ic.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, cfg['imu']['gyro_hz'])
        ic.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, cfg['imu']['accel_hz'])
        ip = ic.resolve(imupipe)
        serial = ip.get_device().get_info(rs.camera_info.serial_number)
        ic.enable_device(serial)
        ip = ic.resolve(imupipe)
        clock_settings = {'expected_domain': str(rs.timestamp_domain.hardware_clock),
                          'motion': configure_device_clock(ip, rs)}
        recorder.put(clock_settings, 'timestamp_settings.json')
        def motion(frame):
            receive_motion(frame, rs, imu, motion_domains, fast_imu)
        ip = imupipe.start(ic, motion)
        pipes.append(imupipe)
        if ip.get_device().get_info(rs.camera_info.serial_number) != serial:
            raise RuntimeError('IMU device changed after clock configuration')
        # Bind video to the SAME physical device if serial was not provided.
        cam, cc = rs.pipeline(context), rs.config()
        cc.enable_device(serial)
        cs = cfg['camera']
        use_depth = cs.get('depth_enabled', True)
        target = rs.stream.infrared if cs['stream'] == 'infrared' else rs.stream.color
        if cs['stream'] == 'infrared':
            cc.enable_stream(target, 1, cs['width'], cs['height'], rs.format.y8, cs['fps'])
        else:
            cc.enable_stream(target, cs['width'], cs['height'], rs.format.bgr8, cs['fps'])
        if use_depth:
            cc.enable_stream(rs.stream.depth, cs['width'], cs['height'], rs.format.z16, cs['fps'])
        video_clock_settings = configure_device_clock(cc.resolve(cam), rs)
        clock_settings = dict(clock_settings, video=video_clock_settings)
        recorder.put(clock_settings, 'timestamp_settings.json')
        profile = cam.start(cc)
        pipes.append(cam)
        for sensor in profile.get_device().query_sensors():
            if sensor.supports(rs.option.frames_queue_size):
                sensor.set_option(rs.option.frames_queue_size, 1)
        depth_sensor = profile.get_device().first_depth_sensor() if use_depth else None
        if use_depth and cs.get('emitter_enabled') is not None and depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, int(cs['emitter_enabled']))
        exposure = configure_exposure(profile.get_device(), cs, rs)
        recorder.put(exposure, 'camera_settings.json')
        print('카메라 노출: %s / %s / 시작 SDK 읽기 %s us (프레임별 노출과 구분)' %
              (cs['stream'], '자동' if cs['auto_exposure'] else '수동',
               exposure['readback_exposure_us_at_start']), flush=True)
        scale = depth_sensor.get_depth_scale() if use_depth else None
        align = rs.align(target) if use_depth else None
        observer = Observer(cfg)
        gyro_prediction = None
        if observer.odom is not None and cfg['odom'].get('gyro_aided_tracking'):
            from .gyro_vision import GyroPrediction
            video_profile = profile.get_stream(target, 1) if cs['stream'] == 'infrared' else profile.get_stream(target)
            gyro_prediction = GyroPrediction(imu, ip.get_stream(rs.stream.gyro), video_profile)
            observer.odom.rotation_predictor = gyro_prediction
            recorder.put(gyro_prediction.settings, 'gyro_vision_settings.json')
            print('자이로 영상 보조: %s' % ('장치 축 변환 읽음 / IMU 보정 대기' if gyro_prediction.settings['available']
                                            else gyro_prediction.settings['reason']), flush=True)
        def capture():
            try:
                while not stop.is_set():
                    frames = cam.wait_for_frames(1000)
                    now = time.monotonic()
                    f = frames.get_infrared_frame(1) if cs['stream'] == 'infrared' else frames.get_color_frame()
                    if not f:
                        continue
                    n = f.get_frame_number()
                    if last_num[0] is not None:
                        if n <= last_num[0]:
                            clock_errors[0] += 1
                        else:
                            sdk_drops[0] += max(0, n-last_num[0]-1)
                    last_num[0] = n
                    received[0] += 1
                    latest.put((frames, now, n))
            except Exception as e:
                if not stop.is_set():
                    latest.put({'error': 'capture: '+str(e)})
        capture_thread = threading.Thread(target=capture, daemon=True)
        capture_thread.start()
        previous_stamp, last_preview = None, 0.
        while not stop.is_set():
            item = latest.get(.2)
            if item is None:
                continue
            if isinstance(item, dict):
                raise RuntimeError(item['error'])
            frames, arrival, frame_number = item
            t0 = time.monotonic()
            raw_video = frames.get_infrared_frame(1) if cs['stream'] == 'infrared' else frames.get_color_frame()
            camera_metadata = frame_exposure(raw_video, rs)
            aligned = align.process(frames) if use_depth else frames
            f = aligned.get_infrared_frame(1) if cs['stream'] == 'infrared' else aligned.get_color_frame()
            d = aligned.get_depth_frame() if use_depth else None
            if not f or (use_depth and not d):
                metrics.counts['missing_aligned_depth'] += 1
                continue
            expected_domain = str(rs.timestamp_domain.hardware_clock)
            video_domain = str(f.get_frame_timestamp_domain())
            depth_domain = str(d.get_frame_timestamp_domain()) if use_depth else None
            timestamp_domains = dict(motion_domains, video=video_domain, depth=depth_domain,
                                     expected=expected_domain)
            if video_domain != expected_domain or (use_depth and depth_domain != expected_domain):
                raise RuntimeError('unexpected_video_clock_domain: '+str(timestamp_domains))
            stamp = f.get_timestamp()/1000.
            intr = f.profile.as_video_stream_profile().intrinsics
            model = str(intr.model)
            if not saved_camera_model:
                actual_model = dict(width=intr.width, height=intr.height, fx=intr.fx, fy=intr.fy,
                                    cx=intr.ppx, cy=intr.ppy, distortion=model, coeffs=list(intr.coeffs),
                                    stream=cs['stream'], serial=serial)
                recorder.put(actual_model, 'camera_model.json')
                expected = cfg['calibration'].get('camera_model')
                if expected:
                    for name in ('width', 'height', 'fx', 'fy', 'cx', 'cy'):
                        if abs(expected[name]-actual_model[name]) > 1e-3:
                            raise RuntimeError('configured camera model differs from active stream: '+name)
                    if expected.get('stream') != cs['stream'] or expected.get('distortion') != model or any(
                            abs(a-b) > 1e-6 for a, b in zip(expected['coeffs'], intr.coeffs)):
                        raise RuntimeError('configured camera distortion/stream differs from active stream')
                saved_camera_model = True
            # Inverse Brown coefficients cannot be passed directly to
            # OpenCV. Fail rather than silently applying the wrong model.
            if intr.model not in (rs.distortion.none, rs.distortion.brown_conrady, rs.distortion.modified_brown_conrady):
                raise RuntimeError('unsupported OpenCV distortion model: '+model)
            K = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], float)
            dist = np.array(intr.coeffs, float)
            image = np.asanyarray(f.get_data())
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
            depth = np.asanyarray(d.get_data()).astype(np.float32)*scale if use_depth else None
            before_imu = imu.snapshot(t0)
            exposure_us = camera_metadata['actual_exposure_us']
            rotational_blur = None if exposure_us is None else max(intr.fx, intr.fy)*before_imu['vector_rate_rps']*exposure_us/1e6
            sample = gray[::8, ::8]
            dsample = depth[::8, ::8] if use_depth else None
            blur_limit = cfg.get('diagnostics', {}).get('reject_rotational_blur_px')
            quality = {'saturated_fraction': float(np.mean(sample >= 250)),
                       'dark_fraction': float(np.mean(sample <= 5)),
                       'valid_depth_fraction': float(np.mean((dsample > cfg['odom']['min_depth_m']) &
                                                            (dsample < cfg['odom']['max_depth_m']))) if use_depth else None,
                       'rotational_blur_estimate_px': rotational_blur,
                       'reject_motion_blur': bool(blur_limit is not None and rotational_blur is not None and rotational_blur > blur_limit)}
            domains_match = (not motion_domains['fault'] and
                             motion_domains['gyro'] == motion_domains['accel'] == video_domain)
            dyaw = imu.between(previous_stamp, stamp) if previous_stamp is not None and domains_match and before_imu['ready'] else None
            capture_imu = imu.at(stamp) if domains_match and before_imu['ready'] else None
            if gyro_prediction is not None:
                gyro_prediction.clock_valid = domains_match
            t1 = time.monotonic()
            obs = observer.process(gray, depth, K, dist, stamp, dyaw, quality, capture_imu=capture_imu)
            obs['camera'] = {'stream': cs['stream'], 'depth_enabled': use_depth, 'requested_auto_exposure': cs['auto_exposure'],
                             'frame_metadata': camera_metadata, 'quality': quality}
            t2 = time.monotonic()
            imu_state = imu.snapshot(t2)
            calibration_state = imu_state['calibration']['state']
            if calibration_state != reported_calibration_state:
                print('IMU 초기 보정: %s / %s' %
                      (calibration_state, imu_state['calibration'].get('reason', '')), flush=True)
                reported_calibration_state = calibration_state
            L = cfg['bounds']['capture_to_callback_max_s']
            lower = None if L is None else arrival-L
            obs.update(capture_id=stamp, arrival=arrival, capture_lower=lower,
                       capture_imu=capture_imu,
                       frame_number=frame_number, published=t2,
                       imu=imu_state, imu_domains_match=domains_match,
                       timestamp_domains=timestamp_domains,
                       clock_fault=bool(imu_state['clock_reversals'] or clock_errors[0] or motion_domains['fault']),
                       sensor_ready=imu_state['ready'] and domains_match and dyaw is not None,
                       imu_bad=(not domains_match or dyaw is None or not imu_state['ready'] or
                                imu_state['clock_reversals'] > 0 or clock_errors[0] > 0 or
                                imu_state['age_s'] is None or cfg['imu']['max_age_s'] is None or
                                imu_state['age_s'] > cfg['imu']['max_age_s']))
            yaw_err = cfg['bounds']['gyro_rate_error_rps']
            if yaw_err is not None:
                obs['yaw_rate_upper'] = abs(imu_state['rate_rps'])+yaw_err
            obs['bounded'] = bool(obs['bounded'] and L is not None and cfg['bounds']['verified'] and imu_state['ready'])
            obs['motion_bounded'] = bool(obs['motion_bounded'] and L is not None and
                                         obs['sensor_ready'] and not obs['imu_bad'])
            if 'stop_motion' in obs:
                obs['stop_motion']['bounded'] &= bool(L is not None and obs['sensor_ready'] and not obs['imu_bad'])
            hint = None
            if motion_hint is not None:
                with motion_hint.get_lock():
                    hint = motion_hint[:]
            if sensor_request is not None:
                with sensor_request.get_lock():
                    request = sensor_request[:]
                revised = recovery_camera_settings(cs, request, hint, t2, last_exposure_request)
                if revised is not None:
                    if not cs['auto_exposure']:
                        applied = configure_exposure(profile.get_device(), revised, rs)
                        cs.update(revised)
                        recorder.put(applied, 'exposure_recovery_%d.json' % request[0])
                    last_exposure_request = request[0]
            obs['camera']['exposure_recovery_request_applied'] = last_exposure_request
            imu.observe_stationary(stamp, stationary_evidence(obs, hint, t2, cfg['imu']))
            recal = imu.snapshot(t2)['stationary_recalibration']
            obs['imu']['stationary_recalibration'] = recal
            if recal['updates'] != saved_stationary_updates:
                recorder.put(recal, 'stationary_imu_%04d.json' % recal['updates'])
                saved_stationary_updates = recal['updates']
            if imu_state['calibration']['state'] == 'READY' and not saved_calibration:
                recorder.put(imu_state['calibration'], 'imu_calibration.json')
                saved_calibration = True
            if obs['tag']['ok'] and obs['tag'].get('camera_translation_m'):
                jitter.add(stamp, obs['tag']['camera_translation_m'])
            if t2-last_report >= cfg.get('logging', {}).get('flush_s', 1.):
                jitter_report = jitter.report()
            obs['tag_position_window'] = jitter_report
            obs['recording'] = recorder.status()
            metrics.counts['processed'] += 1
            if obs['tag']['reason'] != 'not_scheduled':
                metrics.counts['tag_attempts'] += 1
                metrics.counts['tag_accepted' if obs['tag']['ok'] else 'tag_rejected'] += 1
            metrics.counts['odom_accepted' if obs['odom']['ok'] else 'odom_rejected'] += 1
            metrics.counts['imu_pair_unavailable'] += int(dyaw is None)
            aid = obs['odom'].get('gyro_aid', {})
            metrics.counts['gyro_aided_frames'] += int(aid.get('used', False))
            metrics.counts['gyro_rotation_rejected'] += int(obs['odom']['reason'] == 'gyro_rotation_disagree')
            metrics.add('align_extract_ms', (t1-t0)*1000)
            metrics.add('vision_ms', (t2-t1)*1000)
            for stage, milliseconds in obs['stage_ms'].items():
                metrics.add(stage+'_ms', milliseconds)
            metrics.add('arrival_to_result_ms', (t2-arrival)*1000)
            metrics.add('capture_age_upper_ms', None if lower is None else (t2-lower)*1000)
            if previous_stamp is not None:
                metrics.add('processed_capture_gap_ms', (stamp-previous_stamp)*1000)
            previous_stamp = stamp
            elapsed = t2-started
            obs['performance'] = {'elapsed_s': elapsed, 'camera_received': received[0],
                'sdk_frame_gaps': sdk_drops[0], 'latest_replaced': latest.replaced,
                'camera_fps': received[0]/elapsed, 'processed_fps': metrics.counts['processed']/elapsed,
                'accepted_tag_fps': metrics.counts['tag_accepted']/elapsed,
                'arrival_to_result_ms': (t2-arrival)*1000}
            latest_performance = obs['performance']
            usage = resource.getrusage(resource.RUSAGE_SELF)
            obs['performance']['vision_process_cpu_percent'] = 100*(usage.ru_utime+usage.ru_stime-usage0.ru_utime-usage0.ru_stime)/elapsed
            obs['performance']['peak_rss_mb'] = usage.ru_maxrss/(1024*1024 if sys.platform == 'darwin' else 1024)
            # Results reach the control process BEFORE logs/JPEG work.
            publish_latest(results, obs)
            compact = dict(obs)
            compact['odom'] = {k: v for k, v in obs['odom'].items() if k != 'tracked_points'}
            recorder.put(compact)
            if view and t2-last_preview >= 1/cfg['view']['fps']:
                from .webview import encode_preview
                tj = time.monotonic()
                jpeg = encode_preview(image, obs, cfg['view'])
                publish_latest(previews, (t2, jpeg))
                metrics.add('preview_encode_ms', (time.monotonic()-tj)*1000)
                last_preview = t2
            if t2-last_report >= cfg.get('logging', {}).get('flush_s', 1.):
                report = metrics.report()
                report['performance'] = obs['performance']
                report['imu'] = imu_state
                report['timestamp_domains'] = timestamp_domains
                report['tag_position_window'] = jitter_report
                report['recording'] = recorder.status()
                recorder.put(report, 'metrics.json')
                last_report = t2
    except Exception as e:
        # Observation 'error' is a pose uncertainty bound, not an exception.
        publish_latest(results, {'worker_error': type(e).__name__+': '+str(e)})
    finally:
        stop.set()
        latest.close()
        for pipe in reversed(pipes):
            try:
                pipe.stop()
            except Exception:
                pass
        if capture_thread:
            capture_thread.join(timeout=2)
        report = metrics.report()
        report['camera_received'] = received[0]
        report['sdk_frame_gaps'] = sdk_drops[0]
        report['latest_replaced'] = latest.replaced
        report['performance'] = latest_performance
        if imu is not None:
            report['imu'] = imu.snapshot()
        report['timestamp_domains'] = dict(timestamp_domains or {}, **motion_domains)
        report['tag_position_window'] = jitter.report()
        report['recording'] = recorder.status()
        recorder.put(report, 'metrics.json')
        recording_status = recorder.close()
        if recording_status['error']:
            print('기록 상태: '+str(recording_status), flush=True)
