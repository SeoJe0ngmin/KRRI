"""Connection probes only: no controller, CAN sender, or movement frames.

SDK calls run in isolated processes so an absent/hung device cannot hide the
other probe's result. CAN requires exclusive init access and verified SILENT
mode before busOn; there is deliberately no fallback to NORMAL mode.
"""
import json
import math
import multiprocessing as mp
import os
import signal
import threading
import time

from .paths import WORK_ROOT


def result(status, reason, **details):
    return dict(status=status, reason=reason, **details)


def worst(items):
    statuses = [item['status'] for item in items]
    return 'FAIL' if 'FAIL' in statuses else ('WARN' if 'WARN' in statuses else 'PASS')


class Samples:
    """Count progressing device timestamps, not repeated callback deliveries."""
    def __init__(self):
        self.count = self.duplicates = self.reversals = self.invalid = 0
        self.first = self.last = self.arrival = None
        self.max_gap_s = 0.
        self.domain = None
        self.domain_changes = 0

    def add(self, stamp, arrival, domain, values=()):
        if not all(math.isfinite(x) for x in (stamp, arrival)+tuple(values)):
            self.invalid += 1
            return
        if self.domain is not None and domain != self.domain:
            self.domain_changes += 1
        self.domain = domain
        if self.last is not None:
            if stamp <= self.last:
                self.duplicates += int(stamp == self.last)
                self.reversals += int(stamp < self.last)
                return
            self.max_gap_s = max(self.max_gap_s, stamp-self.last)
        if self.first is None:
            self.first = stamp
        self.last, self.arrival = stamp, arrival
        self.count += 1

    def report(self, now, duration, requested_hz):
        age = None if self.arrival is None else max(0., now-self.arrival)
        hz = self.count/max(duration, 1e-9)
        status, reason = 'PASS', 'fresh_samples_received'
        if self.count < 2 or self.reversals or self.invalid or self.domain_changes:
            status, reason = 'FAIL', 'missing_or_invalid_samples'
        elif age > max(.5, 3./requested_hz):
            status, reason = 'FAIL', 'stream_stalled'
        elif hz < requested_hz*.5 or self.duplicates or self.max_gap_s > max(.1, 3./requested_hz):
            status, reason = 'WARN', 'low_rate_or_gaps'
        return result(status, reason, samples=self.count, received_hz=hz,
                      requested_hz=requested_hz, last_age_s=age, max_gap_s=self.max_gap_s,
                      duplicates=self.duplicates, clock_reversals=self.reversals,
                      invalid_samples=self.invalid, timestamp_domain=self.domain,
                      domain_changes=self.domain_changes)


def probe_realsense(sensor, seconds, stop, rs=None):
    """Open IMU first, then video on that same D435i; test actual reception.

    This does not run pose estimation or certify IMU calibration/accuracy.
    A failed IMU start does not prevent an independent camera diagnosis.
    """
    if rs is None:
        import pyrealsense2 as rs
    cs, icfg = sensor['camera'], sensor['imu']
    devices = list(rs.context().query_devices())
    serials = [d.get_info(rs.camera_info.serial_number) for d in devices]
    serial = cs.get('serial')
    if serial is None:
        if len(serials) != 1:
            why = 'no_realsense_device' if not serials else 'multiple_devices_set_camera_serial'
            return {name: result('FAIL', why, devices=serials) for name in ('camera', 'imu')}
        serial = serials[0]
    if serial not in serials:
        return {name: result('FAIL', 'configured_serial_not_found', devices=serials)
                for name in ('camera', 'imu')}
    lock = threading.Lock()
    samples = {name: Samples() for name in ('image', 'depth', 'gyro', 'accel')}
    errors, started_pipes, cleanup = {}, [], []
    collecting = [False]
    cam = None

    def motion(frame):
        try:
            if not frame.is_motion_frame():
                return
            f = frame.as_motion_frame()
            kind = f.get_profile().stream_type()
            name = 'gyro' if kind == rs.stream.gyro else 'accel' if kind == rs.stream.accel else None
            if name is None:
                return
            v = f.get_motion_data()
            with lock:
                if collecting[0]:
                    samples[name].add(f.get_timestamp()/1000., time.monotonic(),
                                      str(f.get_frame_timestamp_domain()), (v.x, v.y, v.z))
        except Exception as exc:
            with lock:
                errors['imu'] = str(exc)

    try:
        pipe = None
        try:
            pipe, cfg = rs.pipeline(), rs.config()
            cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, icfg['gyro_hz'])
            cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, icfg['accel_hz'])
            pipe.start(cfg, motion)
            started_pipes.append(pipe)
        except Exception as exc:
            errors['imu'] = str(exc)
            if pipe is not None:
                try:
                    pipe.stop()
                except Exception:
                    pass  # start may have failed before allocating streams.
        pipe = None
        try:
            pipe, cfg = rs.pipeline(), rs.config()
            cfg.enable_device(serial)
            if cs['stream'] == 'infrared':
                cfg.enable_stream(rs.stream.infrared, 1, cs['width'], cs['height'], rs.format.y8, cs['fps'])
            else:
                cfg.enable_stream(rs.stream.color, cs['width'], cs['height'], rs.format.bgr8, cs['fps'])
            if cs.get('depth_enabled', True):
                cfg.enable_stream(rs.stream.depth, cs['width'], cs['height'], rs.format.z16, cs['fps'])
            pipe.start(cfg)
            started_pipes.append(pipe)
            cam = pipe
        except Exception as exc:
            errors['camera'] = str(exc)
            if pipe is not None:
                try:
                    pipe.stop()
                except Exception:
                    pass
        began = time.monotonic()
        with lock:
            collecting[0] = True
        while not stop.is_set() and time.monotonic()-began < seconds:
            if cam is None:
                stop.wait(.02)
                continue
            # Nonblocking SDK poll: a missing video frame cannot starve IMU or
            # keep the probe beyond its sampling interval.
            try:
                frames = cam.poll_for_frames()
                if frames:
                    image = frames.get_infrared_frame(1) if cs['stream'] == 'infrared' else frames.get_color_frame()
                    for name, f in (('image', image), ('depth', frames.get_depth_frame() if cs.get('depth_enabled', True) else None)):
                        if f:
                            samples[name].add(f.get_timestamp()/1000., time.monotonic(), str(f.get_frame_timestamp_domain()))
            except Exception as exc:
                errors['camera'] = str(exc)
                cam = None
            stop.wait(.002)
        now = time.monotonic()
        with lock:
            collecting[0] = False
            data = {name: s.report(now, now-began, icfg[name+'_hz'] if name in ('gyro', 'accel') else cs['fps'])
                    for name, s in samples.items()}
        video = {name: data[name] for name in ('image', 'depth') if name == 'image' or cs.get('depth_enabled', True)}
        imu = {name: data[name] for name in ('gyro', 'accel')}
        out = {}
        for name, streams in (('camera', video), ('imu', imu)):
            status = 'FAIL' if name in errors or stop.is_set() else worst(streams.values())
            out[name] = result(status, errors.get(name, 'interrupted' if stop.is_set() else 'stream_reception_check'),
                               serial=serial, streams=streams, sample_duration_s=now-began)
    finally:
        with lock:
            collecting[0] = False
        for pipe in reversed(started_pipes):
            try:
                pipe.stop()
            except Exception as exc:
                cleanup.append(str(exc))
    if cleanup:
        for item in out.values():
            item.update(status='FAIL', cleanup_errors=cleanup)
    return out


def probe_can(cfg, seconds, stop, api=None):
    """Observe physical CAN traffic without write(), heartbeats, or ACKs."""
    if api is None:
        from canlib import canlib as api
    out = result('FAIL', 'not_checked', channel=cfg['channel'], bitrate=cfg['bitrate'],
                 tx_calls=0, receive_only=True, rx_frames=0, error_frames=0, received_ids=[])
    ch, on_bus, old_mode, old_params = None, False, None, None
    cleanup = []
    try:
        count = api.getNumberOfChannels()
        out['channel_count'] = count
        if not 0 <= cfg['channel'] < count:
            raise RuntimeError('configured_CAN_channel_not_found')
        info = api.ChannelData(cfg['channel'])
        cap = info.channel_cap
        out['device'] = str(info.channel_name)
        if cap & (api.ChannelCap.VIRTUAL | api.ChannelCap.SIMULATED):
            raise RuntimeError('physical_CAN_required_virtual_channel_rejected')
        if not cap & api.ChannelCap.SILENT_MODE:
            raise RuntimeError('CAN_silent_mode_not_supported_no_busOn')
        # Exclusive + init access prevents changing an active drive channel
        # and avoids silently ignored mode changes on a shared handle.
        ch = api.openChannel(cfg['channel'], flags=api.Open.EXCLUSIVE | api.Open.REQUIRE_INIT_ACCESS)
        old_mode, old_params = ch.getBusOutputControl(), ch.getBusParams()
        rate = {125000: api.Bitrate.BITRATE_125K, 250000: api.Bitrate.BITRATE_250K,
                500000: api.Bitrate.BITRATE_500K, 1000000: api.Bitrate.BITRATE_1M}[cfg['bitrate']]
        ch.setBusParams(rate)
        ch.setBusOutputControl(api.Driver.SILENT)
        if ch.getBusOutputControl() != api.Driver.SILENT:
            raise RuntimeError('CAN_silent_mode_readback_failed_no_busOn')
        # Mark before the call: even a partial busOn failure must try busOff.
        on_bus = True
        ch.busOn()
        began = time.monotonic()
        ids = set()
        while not stop.is_set() and time.monotonic()-began < seconds:
            try:
                frame = ch.read(timeout=20)
            except api.CanNoMsg:
                stop.wait(.002)
                continue
            if frame.flags & (api.MessageFlag.ERROR_FRAME | api.MessageFlag.OVERRUN):
                out['error_frames'] += 1
            elif not frame.flags & (api.MessageFlag.TXACK | api.MessageFlag.TXRQ):
                out['rx_frames'] += 1
                if len(ids) < 16:
                    ids.add(int(frame.id))
        out['received_ids'] = ['0x%X' % cid for cid in sorted(ids)]
        state = ch.readStatus()
        out['bus_status'] = int(state)
        out['sample_duration_s'] = time.monotonic()-began
        if stop.is_set():
            out.update(status='FAIL', reason='interrupted')
        elif out['error_frames'] or state & (api.Stat.BUS_OFF | api.Stat.ERROR_PASSIVE):
            out.update(status='FAIL', reason='CAN_receive_errors')
        elif out['rx_frames']:
            out.update(status='PASS', reason='physical_bus_frames_received_actuation_not_tested')
        else:
            out.update(status='WARN', reason='adapter_opened_but_no_traffic_wiring_not_confirmed')
    except Exception as exc:
        out.update(status='FAIL', reason=str(exc))
    finally:
        if ch is not None:
            if on_bus:
                try:
                    ch.busOff()
                    on_bus = False
                except Exception as exc:
                    cleanup.append('busOff: '+str(exc))
            # Never restore NORMAL while on the bus. Restore the prior setup
            # off-bus so this check does not leave the later drive sender silent.
            if not on_bus:
                for name, value in (('setBusParams', old_params), ('setBusOutputControl', old_mode)):
                    if value is None:
                        continue
                    try:
                        getattr(ch, name)(*value) if name == 'setBusParams' else getattr(ch, name)(value)
                    except Exception as exc:
                        cleanup.append(name+': '+str(exc))
            try:
                ch.close()
            except Exception as exc:
                cleanup.append('close: '+str(exc))
        if cleanup:
            out.update(status='FAIL', cleanup_errors=cleanup)
    return {'can': out}


def _worker(kind, cfg, seconds, stop, connection):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    names = ('camera', 'imu') if kind == 'sensor' else ('can',)
    try:
        probe = probe_realsense if kind == 'sensor' else probe_can
        value = probe(cfg, seconds, stop)
    except Exception as exc:
        value = {name: result('FAIL', type(exc).__name__+': '+str(exc)) for name in names}
    try:
        connection.send(value)
    finally:
        connection.close()


def collect_jobs(jobs, timeout, stop):
    """Keep the other result even if one native SDK never returns."""
    pending, out = dict(jobs), {}
    deadline = time.monotonic()+timeout
    while pending and time.monotonic() < deadline:
        for kind, (process, connection) in list(pending.items()):
            names = ('camera', 'imu') if kind == 'sensor' else ('can',)
            if connection.poll():
                try:
                    out.update(connection.recv())
                except EOFError:
                    out.update({n: result('FAIL', 'probe_exited_without_report') for n in names})
                del pending[kind]
            elif not process.is_alive():
                out.update({n: result('FAIL', 'probe_exited_without_report') for n in names})
                del pending[kind]
        if pending:
            stop.wait(.02)
    if pending:
        stop.set()
        for kind in pending:
            for name in (('camera', 'imu') if kind == 'sensor' else ('can',)):
                out[name] = result('FAIL', 'device_probe_timeout')
    return out


def check_devices(cfg, seconds=5., timeout=None):
    if not math.isfinite(seconds) or not 1 <= seconds <= 60:
        raise ValueError('--seconds must be between 1 and 60')
    timeout = seconds+20. if timeout is None else timeout
    if not math.isfinite(timeout) or not seconds < timeout <= 120:
        raise ValueError('--timeout must exceed --seconds and be at most 120')
    sensor = cfg.get('sensor', cfg)
    camera, imu, can = sensor['camera'], sensor['imu'], cfg['can']
    if camera['stream'] not in ('infrared', 'color'):
        raise ValueError('invalid camera stream')
    for name, value in [('width', camera['width']), ('height', camera['height']), ('fps', camera['fps']),
                        ('gyro_hz', imu['gyro_hz']), ('accel_hz', imu['accel_hz'])]:
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError('invalid '+name)
    if isinstance(can['channel'], bool) or not isinstance(can['channel'], int) or can['channel'] < 0:
        raise ValueError('invalid CAN channel')
    if can['bitrate'] not in (125000, 250000, 500000, 1000000):
        raise ValueError('unsupported CAN bitrate')
    ctx = mp.get_context('spawn')
    stop, jobs = ctx.Event(), {}
    previous_handlers = {}
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    try:
        if threading.current_thread() is threading.main_thread():
            for name in ('SIGTERM', 'SIGHUP'):
                if hasattr(signal, name):
                    sig = getattr(signal, name)
                    previous_handlers[sig] = signal.signal(sig, interrupted)
        for kind, config in (('sensor', sensor), ('can', can)):
            receive, send = ctx.Pipe(duplex=False)
            process = ctx.Process(target=_worker, args=(kind, config, seconds, stop, send), daemon=True)
            try:
                process.start()
            except BaseException:
                receive.close()
                raise
            finally:
                send.close()
            jobs[kind] = process, receive
        checks = collect_jobs(jobs, timeout, stop)
    finally:
        stop.set()
        for process, connection in jobs.values():
            process.join(timeout=1.)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.)
            if process.is_alive() and hasattr(signal, 'SIGKILL'):
                os.kill(process.pid, signal.SIGKILL)
                process.join(timeout=1.)
            connection.close()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
    return dict(schema=1, status=worst(checks.values()), checks=checks,
                command_frames_sent=False, purpose='connection_check_not_drive_validation')


def run(args):
    from .configuration import load_config
    cfg = load_config(args.config)
    if args.can_channel is not None:
        cfg['can']['channel'] = args.can_channel
    if args.bitrate is not None:
        cfg['can']['bitrate'] = args.bitrate
    print('장치 점검: 카메라·IMU 수신 / CAN SILENT 수신 전용. 주행·중립·heartbeat 송신 없음.', flush=True)
    print('측정 %.1f초 + 장치 시작 시간. 다른 카메라/주행 프로그램은 종료한 상태에서 실행하세요.' % args.seconds, flush=True)
    report = check_devices(cfg, args.seconds, args.timeout)
    report.update(platform=args.platform, config=os.path.abspath(args.config))
    output = args.output or os.path.join(WORK_ROOT, 'device_check_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid())+'.json')
    folder = os.path.dirname(output)
    if folder:
        os.makedirs(folder, exist_ok=True)
    with open(output, 'x') as f:
        json.dump(report, f, ensure_ascii=False, indent=2, allow_nan=False)
    for name in ('camera', 'imu', 'can'):
        item = report['checks'][name]
        print('[%s] %s: %s' % (item['status'], name, item['reason']))
        for stream, info in item.get('streams', {}).items():
            print('  %s: %s, %d samples, %.1f Hz (요청 %d)' %
                  (stream, info['status'], info['samples'], info['received_hz'], info['requested_hz']))
        if name == 'can':
            print('  CAN 수신 %d개 / 이 점검의 송신 0개' % item.get('rx_frames', 0))
        if item.get('cleanup_errors'):
            print('  정리 오류: '+str(item['cleanup_errors']))
    print('결과: %s / 보고서: %s' % (report['status'], output))
    print('CAN 무수신은 배선 불량 확정이 아닙니다. 이 점검은 위치 정확도·정지 성능·주행 준비를 승인하지 않습니다.')
    return {'PASS': 0, 'WARN': 1, 'FAIL': 2}[report['status']]
