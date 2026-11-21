"""check (environment and device checks, no CAN) and mount (camera rotation from a recorded measure run at rest,
camyaw style, no CAN). 2026-10-19: the CAN forward-pulse mount and the manual straight-drive mount were removed."""

import argparse
import importlib
import json
from pathlib import Path
import numpy as np
from .config import COMMON, load, save_mount


def check(args):
    modules = ['numpy', 'cv2', 'scipy', 'yaml', 'pupil_apriltags', 'pyrealsense2']
    if not args.camera_only:
        modules += ['rclpy', 'docking_msgs.msg']
    if not args.sensor_only:
        modules += ['canlib.canlib']
    if args.profile == 'mapped':
        modules += ['rtabmap_msgs.msg', 'tf2_ros']
    failed = []
    for name in modules:
        try:
            mod = importlib.import_module(name)
            if name == 'cv2' and not mod.__version__.startswith('4.'):
                raise ImportError('OpenCV 4.x required; use the Ubuntu system OpenCV from install_jetson.sh')
            print(name + ': OK ' + str(getattr(mod, '__version__', '')))
        except (ImportError, OSError) as exc:
            failed.append(name)
            print(name + ': MISSING ' + str(exc))
    if not args.camera_only:
        try:
            from .vio_quality import check_message_schema
            from docking_msgs.msg import VioState

            check_message_schema(VioState)
            from ament_index_python.packages import get_package_prefix

            prefix = Path(get_package_prefix('docking_openvins'))
            if not (prefix / 'lib/docking_openvins/bridge').is_file():
                raise ValueError('native bridge binary missing')
        except Exception as exc:
            failed.append('native OpenVINS bridge')
            print(str(exc))
    cfg = load(args.common, args.mode, require_mount=not args.sensor_only)
    print('설정: OK; CAN 미사용. 장치 입력은 measure에서 확인합니다.')
    for line in inner_tag_lines(cfg):
        print(line)
    if args.sensor_only:
        print('sensor-only: 현장 치수·장착 보정 검사는 생략했습니다. 주행 준비 완료 판정이 아닙니다.')
    if failed:
        raise SystemExit('설치/환경 필요: ' + ', '.join(failed) + '; scripts/install_jetson.sh 참고')
    return cfg


def inner_tag_lines(cfg):
    """실험판 D: 적은 안쪽 태그 자리와 장착각 · 렌즈 모델로, 들어가는 동안 어디까지 두 태그가 화면에 온전히 남는지 알려 준다(기록 · 판단에는 안 씀)."""
    from .geometry import inner_view

    if not cfg['sensor']['calibration'].get('inner'):
        return []
    try:
        v = inner_view(cfg)
    except Exception as exc:
        return ['안쪽 태그 자리 계산 불가: ' + str(exc)]
    if v is None:
        return ['안쪽 태그: 장착각(mount.json)이 없어 화면에서 잘리는 자리는 계산하지 않았습니다.']
    tip = cfg['site']['camera_to_fork_tip_m']
    side = dict(left='왼쪽', right='오른쪽', top='위쪽', bottom='아래쪽').get(v['cut_by'], '')
    out = ['안쪽 태그 면: 입구에서 %.2f m 안쪽.' % v['plane_m']]
    if v['last_camera_x_m'] is None:
        return out + ['  ※ 차로 중심선 어디에서도 두 태그가 화면에 다 들어오지 않습니다: 높이 · 간격을 확인하세요.']
    last_tip = v['last_camera_x_m'] + tip
    where = '입구 %.2f m 앞' % -last_tip if last_tip < 0 else '입구에서 %.2f m 안' % last_tip
    target_tip = v['target_camera_x_m'] + tip
    if v['last_camera_x_m'] >= v['target_camera_x_m']:
        out.append('  → 목표 %.2f m 까지 두 태그가 화면에 남습니다.' % target_tip)
    else:
        out.append(
            '  → 두 태그가 화면에 온전히 남는 것은 포크 끝이 %s에 올 때까지입니다(그 뒤 %s으로 잘림). 남은 %.2f m 는 태그 없이(VIO · depth 로) 갑니다.'
            % (where, side, target_tip - last_tip)
        )
    if v['check_margin_px'] is not None:
        edge = float(cfg['sensor']['tag'].get('edge_px', 15))
        out.append(
            '  → 최종 확인 지점(입구 %.2f m 앞): 화면 가장자리까지 %.0f 화소 %s'
            % (
                cfg['tuning']['final_check_distance_m'],
                v['check_margin_px'],
                '(보임)' if v['check_margin_px'] >= edge else '※ 잘립니다: 태그를 낮추거나 더 안쪽에 두세요',
            )
        )
    return out


def mount(args):
    """Mounting rotation from a recorded measure run: parked facing the tag normal, at rest. No CAN."""
    from .mount import estimate

    folder = Path(args.folder)
    before = Path(args.common).read_bytes()
    cfg = load(args.common, args.mode, False)
    lines = [json.loads(line) for line in (folder / 'observations.jsonl').read_text().splitlines()]
    rows = [r for r in lines if 'kind' not in r and 'capture_id' in r and not r.get('preview_only')]
    vio = [r['value'] for r in lines if r.get('kind') == 'vio']
    bundle = json.loads((folder / 'device.json').read_text())
    report = estimate(
        rows, cfg['sensor'], bundle['raw_models']['color'], vio_rows=vio, T_left_imu=bundle.get('T_left_imu')
    )
    report['source_folder'] = str(folder)
    if report.get('R_body_camera'):
        report['R_body_camera'] = (
            np.array(report['R_body_camera']) @ np.array(bundle['T_left_color'])[:3, :3].T
        ).tolist()
        report['camera_model'] = bundle['raw_models']['left']
        report['measured_stream'] = 'color_converted_to_raw_left_ir'
    (folder / 'mount_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    if not report['usable']:
        raise SystemExit('계산 불가: ' + ', '.join(report['reasons']) + '; ' + str(folder / 'mount_report.json'))
    print(
        'yaw %.2f°(태그) pitch %.2f° roll %.2f°(%s) · 방향 σ %.2f° · 정지 %.0f s · 블록 %d (차체가 태그 법선과 평행하다고 가정)'
        % (
            report['yaw_left_deg'],
            report['pitch_down_deg'],
            report['roll_deg'],
            report['pitch_roll_source'],
            report['rotation_std_deg'],
            report['rest_s'],
            report['blocks'],
        )
    )
    if len(report['warnings']) > 1:
        print('경고(기록만): ' + ', '.join(report['warnings'][1:]))
    if args.write:
        out = save_mount(args.common, report, before)
        print('장착각 저장: ' + str(out) + ' (common.json 은 그대로)')
    else:
        print('저장 안 함 (--write 를 주면 configs/mount.json 에 저장). 보고서: ' + str(folder / 'mount_report.json'))


def parser():
    p = argparse.ArgumentParser(
        description='check(환경·장치 점검, CAN 안 엶) / mount(태그 정면에 세우고 가만히 찍은 measure 로그로 장착각 계산, camyaw 방식, CAN 안 엶)'
    )
    p.add_argument('command', choices=['check', 'mount'])
    p.add_argument('--common', default=str(COMMON))
    p.add_argument(
        '--mode', choices=['left'], default='left', help='left only (the central mode was removed 2026-10-30)'
    )
    p.add_argument('--sensor-only', action='store_true')
    p.add_argument('--camera-only', action='store_true')
    p.add_argument('--profile', choices=['core', 'mapped'], default='core')
    p.add_argument('--folder', help='mount: 태그 정면에 세우고 30초 이상 가만히 찍은 measure 폴더')
    p.add_argument(
        '--write', action='store_true', help='mount: 결과를 configs/mount.json 에 저장 (common.json 은 그대로)'
    )
    return p


if __name__ == '__main__':
    a = parser().parse_args()
    {'check': check, 'mount': mount}[a.command](a)
