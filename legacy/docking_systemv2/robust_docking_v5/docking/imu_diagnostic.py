"""Stationary raw-IMU diagnosis only; never opens CAN or updates a bias/config."""
import argparse
import collections
import datetime
import json
import math
from pathlib import Path
import statistics
import threading
import time

from .calibration import StationaryCalibration
from .paths import PROJECT_ROOT, WORK_ROOT
from .config import COMMON, load


def summarize(rows, cfg):
    """Rows preserve callback order and include both device and host time."""
    streams = {'gyro': [], 'accel': []}
    last = {}
    faults = collections.Counter()
    resets = collections.Counter()
    calibration = StationaryCalibration(cfg)
    for row in rows:
        kind, stamp, xyz = row['kind'], row['device_s'], row['xyz']
        if row['domain'] != 'timestamp_domain.hardware_clock':
            faults[kind + '_wrong_clock_domain'] += 1
            continue
        if not all(math.isfinite(x) for x in [stamp, *xyz]):
            faults[kind + '_nonfinite'] += 1
            continue
        if kind in last and stamp <= last[kind]:
            faults[kind + ('_duplicate' if stamp == last[kind] else '_clock_reversal')] += 1
            continue
        last[kind] = stamp
        streams[kind].append(row)
        before = calibration.restarts
        calibration.add(kind, stamp, xyz)
        if calibration.restarts > before:
            resets[calibration.reason] += calibration.restarts - before

    report = {'diagnostic_only': True, 'config_changed': False,
              'sample_faults': dict(faults), 'calibration_reset_events': dict(resets),
              'calibration_replay': calibration.status()}
    for kind, samples in streams.items():
        if not samples:
            report[kind] = {'samples': 0}
            continue
        scale = 180. / math.pi if kind == 'gyro' else 1.
        values = [[v * scale for v in row['xyz']] for row in samples]
        axes = list(zip(*values))
        gaps = [b['device_s'] - a['device_s'] for a, b in zip(samples, samples[1:])]
        span = samples[-1]['device_s'] - samples[0]['device_s']
        data = {'samples': len(samples), 'units': 'deg/s' if kind == 'gyro' else 'm/s^2',
                'span_s': span, 'hz': (len(samples) - 1) / span if span > 0 else None,
                'mean_xyz': [statistics.mean(a) for a in axes],
                'std_xyz': [statistics.stdev(a) if len(a) > 1 else 0. for a in axes],
                'max_abs_xyz': [max(abs(v) for v in a) for a in axes],
                'max_gap_s': max(gaps, default=0.),
                'gaps_over_limit': sum(g > cfg['startup_calibration']['max_gap_s'] for g in gaps)}
        if kind == 'gyro':
            residual = [math.sqrt(sum((v - b * scale)**2 for v, b in zip(xyz, cfg['gyro_bias_rps'])))
                        for xyz in values]
            threshold = cfg['startup_calibration']['moving_dps']
            ranked = sorted(residual)
            data.update(configured_bias_dps=[b * scale for b in cfg['gyro_bias_rps']],
                        movement_threshold_dps=threshold,
                        movement_gate=('window_mean' if cfg['startup_calibration'].get('motion_window_s') else 'instantaneous'),
                        motion_window_s=cfg['startup_calibration'].get('motion_window_s', 0.),
                        residual_norm_p99_dps=ranked[math.ceil(.99 * len(ranked)) - 1],
                        residual_norm_max_dps=max(residual),
                        samples_over_movement_threshold=sum(v > threshold for v in residual))
            bins = collections.defaultdict(list)
            for row, xyz in zip(samples, values):
                bins[int(row['host_s'])].append(xyz)
            data['one_second_mean_xyz_dps'] = {
                str(k): [statistics.mean(a) for a in zip(*v)] for k, v in sorted(bins.items())}
        report[kind] = data
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20.)
    parser.add_argument('--config', default=str(COMMON))
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 10 <= args.seconds <= 60:
        parser.error('--seconds must be between 10 and 60')
    cfg = load(args.config)
    sensor = cfg.get('sensor', cfg)
    import pyrealsense2 as rs
    from .camera import configure_device_clock

    context = rs.context()
    devices = list(context.query_devices())
    serials = [d.get_info(rs.camera_info.serial_number) for d in devices]
    serial = sensor['camera'].get('serial')
    if serial is None:
        if len(serials) != 1:
            raise RuntimeError('Exactly one RealSense must be connected, or set camera.serial')
        serial = serials[0]
    if serial not in serials:
        raise RuntimeError('Configured camera not found')
    pipe, config = rs.pipeline(context), rs.config()
    config.enable_device(serial)
    for kind in ('gyro', 'accel'):
        config.enable_stream(getattr(rs.stream, kind), rs.format.motion_xyz32f, sensor['imu'][kind + '_hz'])
    clocks = configure_device_clock(config.resolve(pipe), rs)
    folder = Path(WORK_ROOT) / datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f_imu_diagnostic')
    folder.mkdir(parents=True, exist_ok=False)
    (folder / 'config_used.json').write_text(json.dumps(cfg, indent=2))
    rows, errors = [], []
    lock = threading.Lock()
    started = time.monotonic()
    collecting = False

    def receive(frame):
        try:
            if not frame.is_motion_frame():
                return
            f = frame.as_motion_frame()
            stream = f.get_profile().stream_type()
            kind = 'gyro' if stream == rs.stream.gyro else 'accel' if stream == rs.stream.accel else None
            if kind is None:
                return
            v = f.get_motion_data()
            row = {'kind': kind, 'device_s': f.get_timestamp() / 1000.,
                   'domain': str(f.get_frame_timestamp_domain()), 'xyz': [v.x, v.y, v.z]}
            with lock:
                if collecting:
                    row['host_s'] = time.monotonic() - started
                    rows.append(row)
        except Exception as exc:
            with lock:
                if not errors:
                    errors.append(str(exc))

    print('IMU 원시값 진단: CAN/영상은 열지 않고 설정도 변경하지 않습니다.', flush=True)
    print('카메라를 고정하세요. 3초 준비 후 %.0f초 수집합니다.' % args.seconds, flush=True)
    running = False
    interrupted = False
    try:
        pipe.start(config, receive)
        running = True
        time.sleep(3.)
        with lock:
            started = time.monotonic()
            collecting = True
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        with lock:
            collecting = False
        if running:
            try:
                pipe.stop()
            except Exception as exc:
                errors.append('stop: ' + str(exc))
        with (folder / 'raw_imu.jsonl').open('w') as f:
            for row in rows:
                f.write(json.dumps(row) + '\n')
    report = summarize(rows, sensor['imu'])
    report.update(callback_errors=errors, interrupted=interrupted, clock_settings=clocks)
    (folder / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    # Per-second values remain in the saved report; keep the terminal concise.
    terminal = json.loads(json.dumps(report))
    terminal['gyro'].pop('one_second_mean_xyz_dps', None)
    print(json.dumps(terminal, ensure_ascii=False, indent=2))
    print('저장 위치:', folder)
    print('보정값을 적용하지 않았습니다. 평균값만으로 실제 움직임과 bias를 구별할 수는 없습니다.')
    return int(interrupted or bool(errors) or bool(report['sample_faults']) or
               any(report[k]['samples'] < 2 for k in ('gyro', 'accel')))


if __name__ == '__main__':
    raise SystemExit(main())
