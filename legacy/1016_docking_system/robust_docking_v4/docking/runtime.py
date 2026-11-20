"""Parent supervisor: CLI/SSH, policy, CAN leases, read-only dashboard."""
import json
import multiprocessing as mp
import os
import queue
import select
import signal
import sys
import time

from .core import Policy, validate
from .controller import Controller
from .configuration import load_config, rig_dimensions
from .exposure import validate_exposure
from .diagnostics import validate_sensor_options
from .mission import load_policy
from .recording import Recorder
from .calibration import validate_stationary_recalibration
from .feedback import read_imu
from .paths import WORK_ROOT
from .motion import validate_motion
from .canhealth import validate_diagnostics
from .estimation import validate_estimation


def check_sensor(cfg, live=False):
    if cfg.get('schema') != 1:
        raise ValueError('unsupported sensor config')
    if cfg['camera']['stream'] not in ('infrared', 'color'):
        raise ValueError('camera stream must be infrared or color')
    validate_exposure(cfg['camera'])
    validate_sensor_options(cfg)
    validate_estimation(cfg, live)
    validate_stationary_recalibration(cfg['imu'], live)
    validate_motion(cfg, live=live)
    if not isinstance(cfg['camera'].get('depth_enabled', True), bool):
        raise ValueError('camera.depth_enabled must be boolean')
    if cfg['odom']['enabled'] and not cfg['camera'].get('depth_enabled', True):
        raise ValueError('sparse odometry requires depth')
    if not live:
        return
    if not cfg['imu'].get('startup_calibration', {}).get('enabled'):
        raise ValueError('drive requires stationary startup IMU calibration')
    if not cfg['imu'].get('yaw_sign_verified'):
        raise ValueError('drive requires IMU yaw sign/axis verification on this mounting')
    from .core import finite
    if not finite([cfg['imu'].get('max_age_s')]) or cfg['imu']['max_age_s'] <= 0:
        raise ValueError('drive requires measured IMU max_age_s on this host')
    b, cal = cfg['bounds'], cfg['calibration']
    if not b.get('verified') or not cal.get('verified'):
        raise ValueError('sensor calibration/error/timing bounds are unverified')
    for name in ('T_body_camera', 'T_dock_tag'):
        T = cal.get(name)
        if T is None or len(T) != 4 or any(len(row) != 4 for row in T):
            raise ValueError('missing 4x4 transform: '+name)
        import math
        if not all(math.isfinite(x) for row in T for x in row) or T[3] != [0, 0, 0, 1]:
            raise ValueError('invalid rigid transform '+name)
        R = [row[:3] for row in T[:3]]
        for i in range(3):
            for j in range(3):
                if abs(sum(R[k][i]*R[k][j] for k in range(3))-(1 if i == j else 0)) > 1e-5:
                    raise ValueError('rotation not orthonormal')
        det = sum(R[0][i]*(R[1][(i+1)%3]*R[2][(i+2)%3]-R[1][(i+2)%3]*R[2][(i+1)%3]) for i in range(3))
        if abs(det-1) > 1e-5:
            raise ValueError('rotation has reflection')
    vector_bounds = ('tag_pose_error', 'odom_error_per_m', 'odom_error_per_frame', 'odom_error_per_rad')
    scalar_bounds = ('capture_to_callback_max_s', 'relative_translation_error_m', 'max_acceleration_mps2', 'gyro_rate_error_rps')
    for key in vector_bounds+scalar_bounds:
        v = b.get(key)
        values = v if isinstance(v, list) else [v]
        if (not values or not finite(values) or min(values) < 0 or
                (key in vector_bounds and len(values) != 3) or
                (key in scalar_bounds and not isinstance(v, (int, float)))):
            raise ValueError('missing/invalid sensor bound: '+key)
    if not cfg['tag']['enabled'] or not cfg['odom']['enabled']:
        raise ValueError('drive needs tag anchoring and local odometry')
    axis = cfg['imu']['yaw_axis']
    if abs(sum(x*x for x in axis)-1) > 1e-5:
        raise ValueError('IMU yaw axis must be a calibrated unit vector')


def drain(q):
    last = None
    while True:
        try:
            last = q.get_nowait()
        except queue.Empty:
            return last


def run(args):
    cfg = load_config(args.config)
    mode = args.command
    experiment = cfg.get('experiment', {})
    if experiment:
        print('실행 구성: '+experiment.get('folder', experiment.get('variant', 'custom'))+
              ' / '+experiment.get('description', ''), flush=True)
    control = policy = None
    if mode != 'measure':
        validate(cfg, live=(mode == 'drive'))
        if mode == 'drive' and args.platform == 'vm':
            deployment = cfg.get('deployment', {})
            if deployment.get('platform') != 'ubuntu_vm' or not deployment.get('timing_verified_on_this_host'):
                raise ValueError('VM requires its own verified timing/motion profile; do not reuse Nano timing bounds')
        if not args.policy:
            raise ValueError('--policy is required')
        policy = load_policy(cfg, json.load(open(args.policy)))
        control = Controller(cfg, policy, fast_imu_required=True)
        sensor = cfg['sensor']
    else:
        sensor = cfg.get('sensor', cfg)
    if mode != 'measure' and (args.no_tag or args.no_odom or args.no_depth):
        raise ValueError('sensor override flags are measure-only; regenerate driving policy after changes')
    if mode == 'measure':
        sensor['tag']['enabled'] = sensor['tag']['enabled'] and not args.no_tag
        sensor['odom']['enabled'] = sensor['odom']['enabled'] and not (args.no_odom or args.no_depth)
        if args.no_depth:
            sensor['camera']['depth_enabled'] = False
            sensor['camera']['stream'] = 'color'
            # An IR camera model/extrinsic must not be silently reused for RGB.
            sensor['calibration'].update(verified=False, T_body_camera=None,
                                          R_body_camera=None, camera_model=None)
    check_sensor(sensor, live=(mode == 'drive'))
    if mode != 'measure':
        validate_motion(sensor, cfg['limits'], live=(mode == 'drive'))
    if mode == 'drive':
        validate_diagnostics(cfg['can'], live=True)
    rig = rig_dimensions(sensor)
    if rig is not None:
        print('장착 치수: 태그 높이 %.2f m / 카메라 높이 %.2f m / 카메라→앞끝 %.2f m / 높이차 %.2f m' %
              (rig['tag_height_m'], rig['camera_height_m'],
               rig['camera_to_fork_tip_m'], rig['height_difference_m']), flush=True)
    mount = sensor['calibration'].get('mount_measurement')
    if mount and sensor['calibration'].get('R_body_camera') is not None:
        print('카메라 장착각: 좌향 %.3f도 / 측정 %s (카메라 탈부착 후 재측정 필요)' %
              (mount['yaw_left_deg'], mount['id']), flush=True)
    if mode == 'drive' and (not args.execute or not sys.stdin.isatty()):
        raise ValueError('drive requires --execute and an interactive SSH terminal (ssh -t)')
    # In drive mode the policy hash includes sensor settings; do not silently
    # change them with runtime overrides.
    output = args.output or os.path.join(WORK_ROOT, time.strftime('%Y%m%d_%H%M%S')+'_'+mode)
    os.makedirs(output, exist_ok=False)
    with open(os.path.join(output, 'config_used.json'), 'w') as f:
        json.dump(cfg, f, indent=2)
    ctx = mp.get_context('spawn')
    stop, results, previews = ctx.Event(), ctx.Queue(1), ctx.Queue(1)
    # Camera receives only a time-limited neutral-command observation. It
    # cannot issue CAN commands or unlock the independent CAN watchdog.
    motion_hint = ctx.Array('d', [0., 0., 0.])
    fast_imu = ctx.Array('d', [0.]*8)
    sensor_request = ctx.Array('d', [0., 0.])
    from .camera import worker
    process = ctx.Process(target=worker, args=(sensor, stop, results, previews, output, args.view),
                          kwargs={'motion_hint': motion_hint, 'fast_imu': fast_imu,
                                  'sensor_request': sensor_request}, daemon=True)
    can = dashboard = None
    observation = None
    recorder = Recorder(output, sensor.get('logging', {}), stream='control')
    armed = mode == 'shadow'
    status = {'state': 'WAIT_ENTER' if mode == 'drive' else 'MEASURE', 'command': 'stop'}
    if control:
        status['startup_measurement'] = control.preparation.status()
    process.start()
    previous_handlers = {}
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    for name in ('SIGHUP', 'SIGTERM'):
        if hasattr(signal, name):
            sig = getattr(signal, name)
            previous_handlers[sig] = signal.signal(sig, interrupted)
    t0, last_print, seen_observation = time.monotonic(), 0., False
    interrupted_by_operator = False
    preparation_saved = False
    try:
        if args.view:
            from .webview import Dashboard
            dashboard = Dashboard(args.port)
            print('화면: 맥북 SSH 터널 -> http://127.0.0.1:%d' % args.port, flush=True)
        if mode == 'drive':
            from .canbus import Can
            can = Can(cfg['can'], cfg['limits']['lease_s'])
            if control.preparation.enabled:
                print('CAN 중립 연결 완료. 차량을 정지시키고 Enter: IMU 보정 후 정지 측정 %.0f초, '
                      '완료 시 태그 확보/주행 절차로 자동 진행. Ctrl+C 또는 s+Enter 정지.' %
                      control.preparation.c['duration_s'], flush=True)
            else:
                print('CAN 중립 연결 완료. 주변/차량 정지 확인 후 Enter로 시작. Ctrl+C 또는 s+Enter 정지.', flush=True)
        else:
            print('%s 모드: CAN을 열지 않습니다. Ctrl+C 종료. 출력: %s' % (mode, output), flush=True)
        while True:
            now = time.monotonic()
            update = drain(results)
            if update is not None:
                if 'worker_error' in update:
                    raise RuntimeError(update['worker_error'])
                observation, seen_observation = update, True
            if mode == 'drive':
                if select.select([sys.stdin], [], [], 0)[0]:
                    line = sys.stdin.readline()
                    if line == '' or line.strip().lower() in ('s', 'q', 'stop'):
                        can.abort()
                        control.fault('operator_stop_or_ssh_eof')
                        break
                    if not armed and line == '\n':
                        armed = True
                ch = can.status()
                if ch['latched'] or not ch['alive'] or ch['tx_worker_age_s'] > cfg['limits']['lease_s']:
                    raise RuntimeError(ch.get('diagnostics', {}).get('fault') or 'CAN worker stopped/stale or abort latched')
            if armed and control:
                imu_feedback = read_imu(fast_imu)
                now = time.monotonic()
                status = control.step(now, observation, imu_feedback, can_status=ch if can else None)
                if status['command'] == 'stop' and status['state'].startswith('INITIAL'):
                    with sensor_request.get_lock():
                        sensor_request[:] = [float(control.initial.exposure_request), now]
                with motion_hint.get_lock():
                    neutral = status['command'] == 'stop'
                    since = motion_hint[2] if neutral and motion_hint[1] == 1 else now
                    motion_hint[:] = [now, float(neutral), since]
                if can:
                    can.set(status['command'])
                if control.preparation.enabled and control.preparation.complete and not preparation_saved:
                    preparation_saved = recorder.put(control.preparation.report(), 'startup_measurement.json')
                    if preparation_saved:
                        print('정지 준비 측정 완료. 결과: startup_measurement.json. 태그 확보/주행 조건을 확인합니다.', flush=True)
                if update is not None:
                    recorder.put({'now': now, 'status': status,
                                  'observation_id': observation['capture_id'],
                                  'can': can.status() if can else {'connected': False}})
            else:
                with motion_hint.get_lock():
                    since = motion_hint[2] if motion_hint[1] == 1 else now
                    motion_hint[:] = [now, 1., since]
            if dashboard:
                geometry = {k: cfg[k] for k in ('body', 'scene', 'goal')} if policy else None
                dashboard.update({'mode': mode, 'experiment': experiment, 'controller': status, 'observation': observation,
                                  'next_stop': control.expected.to_json() if control and control.expected else None,
                                  'geometry': geometry, 'control_recording': recorder.status(),
                                  'can': can.status() if can else {'connected': False}}, drain(previews))
            if now-last_print >= 2.:
                perf = (observation or {}).get('performance', {})
                prep = status.get('startup_measurement', {})
                if prep.get('enabled') and not prep.get('complete'):
                    print('출발 전 정지 측정: %.1f / %.1f초 | %s' %
                          (prep['stationary_s'], prep['required_s'], prep['reason']), flush=True)
                print('%s | camera %.1f / process %.1f / accepted tag %.1f fps | %s' %
                      (status['state'], perf.get('camera_fps', 0), perf.get('processed_fps', 0),
                       perf.get('accepted_tag_fps', 0), status.get('reason', '')), flush=True)
                last_print = now
                recording = (observation or {}).get('recording', {})
                if recording.get('error') or recorder.status().get('error'):
                    print('기록 중단/누락: '+str({'vision': recording, 'control': recorder.status()}), flush=True)
            if status['state'] in ('DONE', 'FAULT'):
                print(json.dumps(status, ensure_ascii=False), flush=True)
                break
            if stop.is_set() or not process.is_alive():
                # Give the queue feeder a bounded opportunity to deliver
                # the actual camera error before reporting worker exit.
                try:
                    event = results.get(timeout=.2)
                    if 'worker_error' in event:
                        raise RuntimeError(event['worker_error'])
                except queue.Empty:
                    pass
                raise RuntimeError('camera process exited')
            if not seen_observation and now-t0 > 30:
                raise RuntimeError('camera startup timeout')
            if args.seconds and now-t0 >= args.seconds:
                break
            time.sleep(.01)
    except KeyboardInterrupt:
        interrupted_by_operator = True
        if control:
            control.fault('operator_interrupt')
        print('\n정지 후 종료합니다.', flush=True)
    except Exception as exc:
        if control:
            control.fault(str(exc))
        raise
    finally:
        # STOP precedes sensor shutdown, HTTP shutdown and disk finalization.
        if can:
            can.abort()
            can.close()
        stop.set()
        process.join(timeout=4)
        if process.is_alive():
            process.terminate()  # Camera process has no CAN access.
            process.join(timeout=1)
        if dashboard:
            dashboard.close()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        if control and control.preparation.enabled and not preparation_saved:
            recorder.put(control.preparation.report(), 'startup_measurement.json')
        recording_status = recorder.close()
        with open(os.path.join(output, 'session.json'), 'w') as f:
            json.dump({'mode': mode, 'experiment': experiment, 'hardware_motion_enabled': can is not None, 'control_recording': recording_status,
                       'last_status': control.status() if control else status,
                       'can': can.status() if can else {'connected': False},
                       'elapsed_s': time.monotonic()-t0, 'interrupted': interrupted_by_operator}, f, indent=2)
    return output
