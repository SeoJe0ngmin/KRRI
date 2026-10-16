"""Checks and mounting calibration. mount-straight --execute uses CAN."""
import argparse
import importlib
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from .config import COMMON,load,save_mount


def check(args):
    modules=['numpy','cv2','scipy','yaml','pupil_apriltags','pyrealsense2']
    if not args.camera_only:modules+=['rclpy','docking_v6_msgs.msg']
    if not args.sensor_only:modules+=['canlib.canlib']
    if args.profile=='mapped':modules+=['rtabmap_msgs.msg','tf2_ros']
    failed=[]
    for name in modules:
        try:
            mod=importlib.import_module(name)
            if name=='cv2' and not mod.__version__.startswith('4.'):
                raise ImportError('OpenCV 4.x required; use the Ubuntu system OpenCV from install_jetson.sh')
            print(name+': OK '+str(getattr(mod,'__version__','')))
        except (ImportError,OSError) as exc:failed.append(name);print(name+': MISSING '+str(exc))
    if not args.camera_only:
        try:
            from .vio_quality import check_message_schema
            from docking_v6_msgs.msg import VioState
            check_message_schema(VioState)
            from ament_index_python.packages import get_package_prefix
            prefix=Path(get_package_prefix('docking_openvins'))
            if not (prefix/'lib/docking_openvins/bridge').is_file():raise ValueError('native bridge binary missing')
        except Exception as exc:failed.append('native OpenVINS bridge');print(str(exc))
    cfg=load(args.common,args.mode,require_mount=not args.sensor_only)
    print('설정: OK; CAN 미사용. 장치 입력은 measure에서 확인합니다.')
    if args.sensor_only:print('sensor-only: 현장 치수·장착 보정 검사는 생략했습니다. 주행 준비 완료 판정이 아닙니다.')
    if failed:raise SystemExit('설치/환경 필요: '+', '.join(failed)+'; scripts/install_jetson.sh 참고')
    return cfg


def mount(args):
    if not np.isfinite(args.seconds) or not 30<=args.seconds<=180:
        raise ValueError('mount 측정 시간은 30..180초입니다')
    if not args.reference_aligned:raise ValueError('--reference-aligned 필요: 차량을 수평으로 두고 입구면 법선과 독립적으로 정렬한 뒤 측정하세요.')
    from .runtime import run
    from .mount_calibration import estimate
    before=Path(args.common).read_bytes();cfg=load(args.common,args.mode,False)
    observations=[];last_check=[-float('inf')]
    def complete(o):
        if 'capture_id' not in o:return False
        observations.append(o)
        if o['arrival']-last_check[0]<2:return False
        last_check[0]=o['arrival']
        if observations[-1]['arrival']-observations[0]['arrival']<args.seconds:return False
        return estimate(observations,cfg['sensor'],{},args.seconds)['usable']
    def reset_collection():
        observations.clear();last_check[0]=-float('inf')
    complete.reset=reset_collection
    runargs=SimpleNamespace(common=args.common,mode=args.mode,command='measure',seconds=args.seconds+120,
        output=args.output,view=args.view,port=8765,execute=False,no_tag=False,no_odom=False,camera_only=False,profile='core',sensor_calibration=args.sensor_calibration)
    folder=run(runargs,cfg,complete)
    rows=[]
    for line in (folder/'observations.jsonl').read_text().splitlines():
        r=json.loads(line)
        if 'capture_id' not in r:continue
        rows.append(r)
    bundle=json.loads((folder/'device.json').read_text())
    report=estimate(rows,cfg['sensor'],bundle['raw_models']['color'],args.seconds)
    # Current colour fit -> raw left IR mounting in common, shared with v5.
    if report.get('R_body_camera'):
        Rbc=np.array(report['R_body_camera']);Rlc=np.array(bundle['T_left_color'])[:3,:3]
        report['R_body_camera']=(Rbc@Rlc.T).tolist();report['camera_model']=bundle['raw_models']['left']
        report['measured_stream']='color_converted_to_raw_left_ir'
        u=cfg['sensor']['uncertainty'];r=max(u['mount_rotation_std_rad'],np.deg2rad(report['block_spread_p95_deg']))
        report['covariance']=np.diag([u['mount_position_std_m']**2]*3+[r*r]*3).tolist()
    (folder/'mount_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    if report['usable']:
        save_mount(args.common,report,before);print('공통 장착 보정 저장 완료: '+args.common)
    else:raise SystemExit('저장하지 않음: '+', '.join(report['reasons'])+'; '+str(folder/'mount_report.json'))


def mount_straight(args):
    """Thirty seconds at rest, one three-second CAN pulse, then settle and save."""
    from .runtime import run
    from .mount_pulse import estimate,Pulse
    import sys
    if not args.execute or not sys.stdin.isatty():
        raise ValueError('mount-straight는 CAN 3초 전진합니다. 대화형 SSH 터미널에서 --execute 필요')
    if not np.isfinite(args.seconds) or not 90<=args.seconds<=300:
        raise ValueError('직진 보정 전체 제한 시간은 90..300초 (권장 180초)')
    before=Path(args.common).read_bytes();cfg=load(args.common,args.mode,False)
    pulse=Pulse()
    print('태그 수직·평평한 바닥·조향 고정. heading 0 불필요. 종료 후 수동으로 출발 위치로 복귀하세요.',flush=True)
    runargs=SimpleNamespace(common=args.common,mode=args.mode,command='measure',seconds=args.seconds,
        output=args.output,view=args.view,port=8765,execute=True,no_tag=False,no_odom=False,
        camera_only=False,profile='core',sensor_calibration=args.sensor_calibration,calibration_forward=True)
    folder=run(runargs,cfg,pulse)
    bundle=json.loads((folder/'device.json').read_text())
    # Select durable full-rate observations using the supervisor's capture boundaries.
    saved=[json.loads(line) for line in (folder/'observations.jsonl').read_text().splitlines()]
    saved=[r for r in saved if 'capture_id' in r and not r.get('preview_only')]
    def selected(part):
        if not part:return []
        return [r for r in saved if part[0]['capture_id']<=r['capture_id']<=part[-1]['capture_id']]
    report=estimate(selected(pulse.rows),selected(pulse.start),selected(pulse.end),cfg['sensor'],bundle['raw_models']['color'],stop_capture=pulse.stop_capture)
    if not pulse.done:
        report['usable']=False;report['reasons'].append('보정 절차 미완료 또는 사용자 중단')
    if report.get('R_body_camera'):
        report['R_body_camera']=(np.array(report['R_body_camera'])@np.array(bundle['T_left_color'])[:3,:3].T).tolist()
        report['camera_model']=bundle['raw_models']['left']
        report['measured_stream']='color_converted_to_raw_left_ir'
    (folder/'mount_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    if not report['usable']:raise SystemExit('저장하지 않음: '+', '.join(report['reasons'])+'; '+str(folder/'mount_report.json'))
    save_mount(args.common,report,before)
    print('공통 직진 장착 보정 저장 완료: '+args.common,flush=True)
    print('CAN 종료. 수동 모드로 실험 위치로 이동한 뒤 CAN 모드로 전환하고 run을 실행하세요.',flush=True)


def parser():
    p=argparse.ArgumentParser(description='v6 check / mount. CAN을 열지 않습니다.')
    p.add_argument('command',choices=['check','mount','mount-straight']);p.add_argument('--common',default=str(COMMON))
    p.add_argument('--mode',choices=['central','left'],default='central');p.add_argument('--sensor-only',action='store_true')
    p.add_argument('--camera-only',action='store_true');p.add_argument('--profile',choices=['core','mapped'],default='core')
    p.add_argument('--execute',action='store_true',help='mount-straight CAN 3초 전진 허용');p.add_argument('--reference-aligned',action='store_true');p.add_argument('--seconds',type=float,default=60)
    p.add_argument('--output');p.add_argument('--view',action='store_true');p.add_argument('--sensor-calibration')
    return p

if __name__=='__main__':
    a=parser().parse_args();{'check':check,'mount':mount,'mount-straight':mount_straight}[a.command](a)
