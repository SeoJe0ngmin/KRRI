"""Camera mounting orientation from an operator-established reference pose.

The vehicle must be level and normal to an upright, level tag plane. This
assumption is not observable from a stationary camera. No CAN or old controller
is used. Repeatability checks below are NOT absolute pose-error certificates.
"""
import copy
import datetime
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import uuid

import numpy as np
import cv2

from .configuration import BODY_REFERENCE, _rotation, resolve_config
from .paths import PROJECT_ROOT, WORK_ROOT


# Current Tag.update/IPPE corner convention, not the old apriltag_v4 frame.
R_OPTICAL_TO_BODY = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
R_REFERENCE_TAG = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
SCHEMA = 'camera_mount_reference_v1'
LIMITS = dict(min_accepted_hz=2., min_coverage=.8, max_tag_gap_s=2.,
              block_spread_p95_deg=1., half_change_deg=.5,
              translation_spread_p95_m=.05, max_mount_angle_deg=30.)


def _project_rotation(matrix):
    u, _, vt = np.linalg.svd(matrix)
    return u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt


def _angle(a, b):
    return math.degrees(math.acos(float(np.clip((np.trace(a @ b.T)-1.)/2., -1., 1.))))


def _centre(rotations):
    return _project_rotation(np.median(np.asarray(rotations), axis=0))


def estimate(rows, sensor, camera_model, seconds=60.):
    """Analyse the last requested interval; old mounting transforms are unused."""
    if not math.isfinite(seconds) or not 30 <= seconds <= 180:
        raise ValueError('measurement interval must be 30..180 seconds')
    report = dict(schema=SCHEMA, id=uuid.uuid4().hex, usable=False,
                  method='upright_tag_vehicle_level_and_normal_to_plane',
                  reference_alignment_confirmed=True, certified_error_bound=False,
                  limits=LIMITS.copy(), reasons=[], seconds=seconds,
                  camera_model=copy.deepcopy(camera_model),
                  tag={k: sensor['tag'][k] for k in ('id', 'size_m')})
    reasons = report['reasons']
    if not rows:
        reasons.append('no_observations')
        return report
    times = np.array([r['arrival'] for r in rows], float)
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        reasons.append('invalid_observation_clock')
        return report
    end, start = times[-1], times[-1]-seconds
    if times[-1]-times[0] < seconds:
        reasons.append('measurement_too_short')
    window = [r for r in rows if r['arrival'] >= start]
    if any(r.get('clock_fault') for r in window):
        reasons.append('sensor_clock_fault')
    if any(r.get('imu', {}).get('calibration', {}).get('state') != 'READY' for r in window):
        reasons.append('imu_not_ready_throughout_measurement')
    imus = [r.get('imu', {}) for r in window]
    if any(i.get('age_s') is None or i['age_s'] > .5 for i in imus):
        reasons.append('imu_stale')
    if len({i.get('gaps') for i in imus}) != 1 or any(i.get('clock_reversals', 0) for i in imus):
        reasons.append('imu_gap_or_clock_reversal')
    if any(r.get('recording', {}).get('error') or r.get('recording', {}).get('dropped', 0) for r in window):
        reasons.append('incomplete_recording')
    # Binned integrated yaw uses every SDK gyro sample, not a single noisy rate.
    gyro_bins = {}
    for r in window:
        i = r['imu']
        if i.get('device_stamp_s') is not None and i.get('yaw_rad') is not None:
            gyro_bins[int(r['arrival']-start)] = (i['device_stamp_s'], i['yaw_rad'])
    rates = []
    for a, b in zip(list(gyro_bins.values()), list(gyro_bins.values())[1:]):
        if b[0] > a[0]:
            rates.append(abs(math.degrees((b[1]-a[1])/(b[0]-a[0]))))
    report['max_one_second_yaw_rate_dps'] = max(rates) if rates else None
    if not rates or max(rates) > sensor['imu']['startup_calibration']['moving_dps']:
        reasons.append('rotation_during_measurement')

    bins, accepted, stamps, positions = {}, [], [], []
    attempts = sum(r.get('tag', {}).get('reason') not in ('not_scheduled', None) for r in window)
    for row in window:
        tag = row.get('tag', {})
        if not tag.get('ok') or tag.get('reason') != 'accepted':
            continue
        rv, xyz = np.asarray(tag.get('camera_rotation_vector_rad'), float), np.asarray(tag.get('camera_translation_m'), float)
        if rv.shape != (3,) or xyz.shape != (3,) or not np.isfinite([rv, xyz]).all():
            if 'invalid_tag_pose' not in reasons:
                reasons.append('invalid_tag_pose')
            continue
        R = R_REFERENCE_TAG @ cv2.Rodrigues(rv)[0].T
        accepted.append(R)
        positions.append(xyz)
        stamps.append(row['arrival'])
        bins.setdefault(int(row['arrival']-start), []).append((R, xyz))
    report.update(tag_attempts=attempts, accepted=len(accepted))
    if len(accepted) < seconds*LIMITS['min_accepted_hz']:
        reasons.append('too_few_accepted_tags')
    if not accepted:
        return report
    max_gap = max(np.diff([start, *stamps, end]))
    blocks = [v for v in bins.values() if len(v) >= 2]
    report.update(covered_seconds=len(blocks), max_tag_gap_s=float(max_gap))
    if len(blocks) < seconds*LIMITS['min_coverage'] or max_gap > LIMITS['max_tag_gap_s']:
        reasons.append('insufficient_tag_time_coverage')
    if len(blocks) < 4:
        return report
    # Each one-second block has one vote; a burst of frames cannot dominate.
    rotations = [_centre([x[0] for x in block]) for block in blocks]
    R = _centre(rotations)
    spread = [_angle(r, R) for r in rotations]
    delta = R @ R_OPTICAL_TO_BODY.T
    yaw = math.degrees(math.atan2(delta[1, 0], delta[0, 0]))
    pitch = math.degrees(math.atan2(-delta[2, 0], math.hypot(delta[0, 0], delta[1, 0])))
    roll = math.degrees(math.atan2(delta[2, 1], delta[2, 2]))
    half = len(rotations)//2
    change = _angle(_centre(rotations[:half]), _centre(rotations[half:]))
    centres = np.array([np.median([x[1] for x in block], axis=0) for block in blocks])
    position_spread = float(np.percentile(np.linalg.norm(centres-np.median(centres, axis=0), axis=1), 95))
    report.update(R_body_camera=R.tolist(), yaw_left_deg=yaw, pitch_down_deg=pitch,
                  roll_deg=roll, block_spread_p95_deg=float(np.percentile(spread, 95)),
                  first_last_half_change_deg=change, translation_spread_p95_m=position_spread)
    if report['block_spread_p95_deg'] > LIMITS['block_spread_p95_deg'] or change > LIMITS['half_change_deg']:
        reasons.append('orientation_unstable')
    if position_spread > LIMITS['translation_spread_p95_m']:
        reasons.append('tag_translation_unstable')
    if _angle(R, R_OPTICAL_TO_BODY) > LIMITS['max_mount_angle_deg']:
        reasons.append('check_reference_pose_or_tag_orientation')
    report['usable'] = not reasons
    return report


def calibration_sensor(config):
    sensor = copy.deepcopy(config.get('sensor', config))
    if sensor.get('rig', {}).get('body_reference') != BODY_REFERENCE:
        raise ValueError('mount calibration requires the ground-centreline body frame')
    if not sensor['imu'].get('startup_calibration', {}).get('enabled'):
        raise ValueError('mount calibration requires startup IMU calibration')
    sensor['tag']['enabled'] = True
    sensor['odom']['enabled'] = False
    sensor['imu']['stationary_recalibration']['enabled'] = False
    # No old pose/anchor may gate raw tag measurements. Keep stream and depth
    # unchanged, including IR emitter settings from the normal camera worker.
    sensor['calibration'].update(verified=False, R_body_camera=None, T_body_camera=None,
                                 R_dock_tag=None, T_dock_tag=None, camera_model=None)
    sensor['calibration'].pop('mount_measurement', None)
    return sensor


def _updated_config(config, report, lateral):
    c = copy.deepcopy(config)
    s = c.get('sensor', c)
    model = report['camera_model']
    if s.get('rig', {}).get('body_reference') != BODY_REFERENCE:
        raise ValueError('different body reference')
    for key in ('stream', 'width', 'height'):
        if s['camera'][key] != model[key]:
            raise ValueError('camera profile differs: '+key)
    if s['camera'].get('serial') not in (None, model['serial']):
        raise ValueError('camera serial differs')
    if any(s['tag'][k] != report['tag'][k] for k in ('id', 'size_m')):
        raise ValueError('tag profile differs')
    s['camera']['serial'] = model['serial']
    s['calibration'].update(R_body_camera=report['R_body_camera'], T_body_camera=None,
                            camera_lateral_m=lateral, camera_model=model, verified=False,
                            mount_measurement={'id': report['id'], 'method': report['method'],
                                               'yaw_left_deg': report['yaw_left_deg'],
                                               'certified_error_bound': False})
    s['imu']['yaw_sign_verified'] = False
    if 'validation' in c:
        c['validation'].update(approved=False, extrinsics=None)
    # Lateral translation is a separate tape measurement, never a pivot offset.
    return resolve_config(c)


def apply_report(report, paths, lateral=None, expected=None):
    """Validate all targets before writing; restore originals on write failure."""
    if (report.get('schema') != SCHEMA or not report.get('usable') or report.get('reasons') or
            not report.get('reference_alignment_confirmed')):
        raise ValueError('a successful reference-pose measurement is required')
    _rotation(report.get('R_body_camera'), 'mount R_body_camera')
    if not report.get('camera_model', {}).get('serial'):
        raise ValueError('measured camera serial is missing')
    if lateral is not None and (isinstance(lateral, bool) or not math.isfinite(lateral)):
        raise ValueError('camera lateral offset must be a measured finite number')
    if len(paths) == 1:
        path = Path(paths[0])
        from .shared import manager
        if manager().decode(path.read_text()).get('schema') == 'docking_system_common_v1':
            return manager().apply_mount(path, report, lateral, expected)
    pending = []
    for path in sorted(set(Path(p).resolve() for p in paths)):
        before = path.read_bytes()
        cfg = _updated_config(json.loads(before), report, lateral)
        pending.append((path, before, json.dumps(cfg, ensure_ascii=False, indent=2, allow_nan=False).encode()+b'\n'))
    if not pending:
        raise ValueError('no matching full sensor profiles to update')
    written = []
    try:
        for path, before, after in pending:
            # Catch changes since preflight instead of overwriting concurrent edits.
            if path.read_bytes() != before:
                raise ValueError('config changed during apply: '+str(path))
            _atomic(path, after)
            written.append((path, before))
    except BaseException:
        for path, before in reversed(written):
            _atomic(path, before)
        raise
    return [str(p) for p, _, _ in pending]


def _atomic(path, data):
    fd, name = tempfile.mkstemp(prefix='.'+path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as f:
            os.fchmod(f.fileno(), path.stat().st_mode & 0o777)
            f.write(data)
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def profile_paths(project, all_versions):
    common = Path(project).parent/'configs/common.json'
    if not common.is_file():
        raise ValueError('docking_system/configs/common.json is missing')
    # The flag remains an alias; both apply paths target the same shared source.
    return [common]


def run(args):
    if not math.isfinite(args.seconds) or not 30 <= args.seconds <= 180:
        raise ValueError('--seconds must be 30..180 (default 60)')
    if args.camera_lateral_m is not None and not math.isfinite(args.camera_lateral_m):
        raise ValueError('--camera-lateral-m must be finite')
    # Preflight folders/configs before spending time on capture.
    targets = profile_paths(PROJECT_ROOT, args.apply_versions) if args.apply or args.apply_versions else []
    expected = targets[0].read_bytes() if len(targets) == 1 else None
    from .configuration import load_config
    sensor = calibration_sensor(load_config(args.config))
    print('카메라 장착각 측정: CAN 연결 없음. 기존 장착각은 측정에 사용하지 않습니다.')
    print('조건: 수평 바닥에서 차체 정면을 태그 면에 수직으로 정렬. 태그는 수직, 윗변은 수평.')
    print('영상 중앙에 태그를 놓는 것으로 차체 정렬을 대신할 수 없습니다. 차체 오정렬/태그 기울기는 분리할 수 없습니다.')
    if not args.aligned_reference:
        if not sys.stdin.isatty():
            raise ValueError('reference pose must be confirmed in an interactive terminal or with --aligned-reference')
        if input('실제 차체/태그 정렬을 확인했으면 Enter, 취소는 q + Enter: ').strip():
            return 1
    token = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:8]
    folder = Path(WORK_ROOT)/(token+'_mount_calibration')
    folder.mkdir(parents=True, exist_ok=False)
    config_path = folder/'capture_config.json'
    config_path.write_text(json.dumps(sensor, ensure_ascii=False, indent=2))
    print('준비 여유 15초를 포함해 %.0f초 기록합니다. 차량/카메라를 계속 고정하세요.' % (args.seconds+15), flush=True)
    from .runtime import run as measure
    options = SimpleNamespace(command='measure', platform=args.platform, config=str(config_path),
                              policy=None, output=str(folder/'capture'), seconds=args.seconds+15,
                              view=args.view, port=args.port, execute=False, no_tag=False, no_odom=True, no_depth=False)
    capture = Path(measure(options))
    session = json.loads((capture/'session.json').read_text())
    rows = [json.loads(line) for line in (capture/'observations.jsonl').read_text().splitlines() if line.strip()]
    model = json.loads((capture/'camera_model.json').read_text())
    report = estimate(rows, sensor, model, args.seconds)
    if session.get('interrupted') or session['elapsed_s'] < args.seconds+15:
        report['reasons'].append('capture_interrupted')
    if session.get('hardware_motion_enabled'):
        report['reasons'].append('unexpected_CAN_session')
    report.update(usable=not report['reasons'], created_local=datetime.datetime.now().isoformat(),
                  config_source=str(Path(args.config).resolve()), capture=str(capture), applied=[])
    path = folder/'mount_calibration.json'
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    print('장착각: 좌향 yaw %s° / pitch(아래+) %s° / roll %s°' % tuple(
        round(report[k], 4) if k in report else '-' for k in ('yaw_left_deg', 'pitch_down_deg', 'roll_deg')))
    print('통과 태그 %d장 / 유효 시간 %s초 / 블록 각도 편차 p95 %s° / 전후반 변화 %s°' %
          (report.get('accepted', 0), report.get('covered_seconds', 0),
           report.get('block_spread_p95_deg', '-'), report.get('first_last_half_change_deg', '-')))
    print('상태:', '측정 품질 통과 (절대 정확도 보증 아님)' if report['usable'] else ', '.join(report['reasons']))
    print('결과:', path, flush=True)
    if not report['usable']:
        return 2
    if targets:
        report['applied'] = apply_report(report, targets, args.camera_lateral_m, expected=expected)
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        print('장착각 적용: %d개 설정 원본 갱신 (공통 파일이면 v1~v4 모두 다음 실행에 적용). 주행 승인은 다시 검증하고 정책을 다시 생성하세요.' % len(report['applied']))
        if args.camera_lateral_m is None:
            print('카메라 좌우 위치는 미측정으로 유지합니다. 차체 중심선 기준 좌우 거리는 별도 실측값입니다.')
    return 0
