"""Offline checks and observed mounting calibration; this module never opens CAN."""
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
    begin=[None]
    def complete(o):
        if not o.get('stationary') or not o.get('sensor_ready'):begin[0]=None
        elif begin[0] is None:begin[0]=o['capture_id']
        return begin[0] is not None and o['capture_id']-begin[0]>=args.seconds+1
    runargs=SimpleNamespace(common=args.common,mode=args.mode,command='measure',seconds=args.seconds+120,
        output=args.output,view=args.view,port=8765,execute=False,no_tag=False,no_odom=False,camera_only=False,profile='core',sensor_calibration=args.sensor_calibration)
    folder=run(runargs,cfg,complete)
    rows=[];seen=set()
    for line in (folder/'observations.jsonl').read_text().splitlines():
        r=json.loads(line)
        if 'capture_id' not in r:continue
        tag=r.get('tag',{});key=tag.get('capture_key')
        if key is None or key in seen:continue
        seen.add(key);rows.append(r)
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


def parser():
    p=argparse.ArgumentParser(description='v6 check / mount. CAN을 열지 않습니다.')
    p.add_argument('command',choices=['check','mount']);p.add_argument('--common',default=str(COMMON))
    p.add_argument('--mode',choices=['central','left'],default='central');p.add_argument('--sensor-only',action='store_true')
    p.add_argument('--camera-only',action='store_true');p.add_argument('--profile',choices=['core','mapped'],default='core')
    p.add_argument('--reference-aligned',action='store_true');p.add_argument('--seconds',type=float,default=60)
    p.add_argument('--output');p.add_argument('--view',action='store_true');p.add_argument('--sensor-calibration')
    return p

if __name__=='__main__':
    a=parser().parse_args();(check if a.command=='check' else mount)(a)
