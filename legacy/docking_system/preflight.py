#!/usr/bin/env python3
"""Field setup entry point. No command here sends vehicle motion commands."""
import argparse
import datetime
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import uuid

ROOT = Path(__file__).resolve().parent


def token():
    # Jetsons may start with a 1970 clock; UUID prevents reused result names.
    return datetime.datetime.now().strftime('%Y%m%d_%H%M%S_')+uuid.uuid4().hex[:8]


def save_report(kind, report):
    folder = ROOT/'calibration'
    folder.mkdir(exist_ok=True)
    path = folder/(token()+'_'+kind+'.json')
    with path.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print('공통 점검 기록:', path)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description='v1~v4 공통 설정·주행 전 점검 (차량 구동 명령 없음)')
    parser.add_argument('--version', choices=('v1', 'v2', 'v3', 'v4'), default='v1')
    parser.add_argument('--platform', choices=('nano', 'vm'), default='nano')
    parser.add_argument('--task', choices=('kw_wall_stop', 'kw_park_left_tag'), default='kw_wall_stop')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('check', help='네 버전의 미입력 값·주행 준비 상태 확인; 장치 연결 없음')
    sub.add_parser('sync', help='공통 설정으로 각 버전의 읽기용 설정 사본 갱신; 실행에는 원래 즉시 반영')
    mount = sub.add_parser('mount', help='카메라 재장착 후 각도 측정, 성공 시 공통 설정 자동 저장; CAN 없음')
    mount.add_argument('--seconds', type=float, default=60.)
    mount.add_argument('--camera-lateral-m', type=float)
    mount.add_argument('--aligned-reference', action='store_true')
    mount.add_argument('--view', action='store_true')
    mount.add_argument('--port', type=int, default=8765)
    devices = sub.add_parser('devices', help='카메라·IMU·CAN 수신만 점검; CAN 송신 없음')
    devices.add_argument('--seconds', type=float, default=5.)
    devices.add_argument('--timeout', type=float)
    inspect = sub.add_parser('inspect', help='영상·태그·오도메트리·실행별 IMU 초기 보정 측정; CAN 없음')
    inspect.add_argument('--seconds', type=float, default=60.)
    inspect.add_argument('--view', action='store_true')
    inspect.add_argument('--port', type=int, default=8765)
    args = parser.parse_args(argv)
    project = ROOT/('robust_docking_'+args.version)
    sys.path.insert(0, str(project))
    from docking.configuration import load_config
    from docking.shared import manager
    shared = manager()
    source = ROOT/'configs/common.json'
    before = source.read_bytes()
    doc = shared.read(source)
    shared.check_document(ROOT, doc)
    base = project/'configs'/('hardware_vm.json' if args.platform == 'vm' else 'hardware.json')
    measure = project/'configs'/(args.platform+'_measure.json')

    if args.command == 'sync':
        print('공통 설정으로 %d개 설정 사본 갱신.' % len(shared.sync(ROOT)))
        return 0
    if args.command == 'check':
        from docking.profiles import readiness
        report = dict(task=args.task, platform=args.platform, hardware_opened=False, versions={})
        for n in range(1, 5):
            version = json.loads((ROOT/f'robust_docking_v{n}/version.json').read_text())
            cfg = shared.expand(doc, dict(mode='hardware', platform=args.platform,
                                         site=args.task, variant=version['variant']))
            status = readiness(cfg)
            report['versions']['v'+str(n)] = status
            print('v%d: drive_ready=%s / 미입력 %d / 미검증 %d' %
                  (n, status['drive_ready'], len(status['missing_inputs']), len(status['unverified_inputs'])))
        first = report['versions']['v1']
        print('공통 미입력:', ', '.join(first['missing_inputs']) or '없음')
        print('공통 미검증:', ', '.join(first['unverified_inputs']) or '없음')
        save_report('config_check', report)
        return 0 if all(v['drive_ready'] for v in report['versions'].values()) else 2
    if args.command == 'mount':
        from docking.mount_calibration import run
        options = SimpleNamespace(**vars(args), config=str(measure), apply=True, apply_versions=False)
        result = run(options)
        if result == 0:
            report = shared.read(source)['automatic']['mount']
            save_report('mount', report)
            shared.sync(ROOT)
        return result
    if args.command == 'devices':
        from docking.device_check import check_devices
        print('카메라·IMU 수신, CAN SILENT 수신 점검. 구동·중립·heartbeat 송신 없음.', flush=True)
        report = check_devices(load_config(base), args.seconds, args.timeout)
        report.update(version=args.version, platform=args.platform)
        save_report('devices', report)
        shared.record_check(source, 'devices_'+args.platform, report, before)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return {'PASS': 0, 'WARN': 1, 'FAIL': 2}[report['status']]
    if args.command == 'inspect':
        import math
        if not math.isfinite(args.seconds) or args.seconds < 30:
            raise ValueError('--seconds must be finite and at least 30')
        from docking.runtime import run
        from docking.paths import WORK_ROOT
        output = Path(WORK_ROOT)/(token()+'_preflight_measure')
        options = SimpleNamespace(command='measure', platform=args.platform, config=str(measure),
                                  seconds=args.seconds, output=str(output), policy=None, execute=False,
                                  no_tag=False, no_odom=False, no_depth=False, view=args.view, port=args.port)
        run(options)
        metrics = json.loads((output/'metrics.json').read_text())
        session = json.loads((output/'session.json').read_text())
        report = dict(version=args.version, platform=args.platform, capture=str(output),
                      metrics=metrics, session=session, certified_error_bound=False,
                      applied_startup_bias=False)
        save_report('inspection', report)
        shared.record_check(source, 'inspection_'+args.platform+'_'+args.version, report, before)
        imu = metrics.get('imu', {})
        print('IMU 준비:', imu.get('ready'), '/', imu.get('calibration', {}).get('reason'))
        print('FPS:', metrics.get('performance', {}))
        print('점검 결과를 공통 기록에 저장했습니다. IMU 영점은 다음 실행에서 새로 측정합니다.')
        return 0 if imu.get('ready') and not session.get('interrupted') else 2


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, OSError) as exc:
        print('설정/점검 오류:', exc, file=sys.stderr)
        raise SystemExit(2)
