"""check (environment and device checks, no CAN) and mount (camera rotation from a recorded measure run at rest,
camyaw style, no CAN). 2026-10-19: the CAN forward-pulse mount and the manual straight-drive mount were removed."""
import argparse
import importlib
import json
from pathlib import Path
import numpy as np
from .config import COMMON,load,save_mount


def check(args):
    modules=['numpy','cv2','scipy','yaml','pupil_apriltags','pyrealsense2']
    if not args.camera_only:modules+=['rclpy','docking_msgs.msg']
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
            from docking_msgs.msg import VioState
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
    """Mounting rotation from a recorded measure run: parked facing the tag normal, at rest. No CAN."""
    from .mount import estimate
    folder=Path(args.folder)
    before=Path(args.common).read_bytes();cfg=load(args.common,args.mode,False)
    lines=[json.loads(line) for line in (folder/'observations.jsonl').read_text().splitlines()]
    rows=[r for r in lines if 'kind' not in r and 'capture_id' in r and not r.get('preview_only')]
    vio=[r['value'] for r in lines if r.get('kind')=='vio']
    bundle=json.loads((folder/'device.json').read_text())
    report=estimate(rows,cfg['sensor'],bundle['raw_models']['color'],vio_rows=vio,T_left_imu=bundle.get('T_left_imu'));report['source_folder']=str(folder)
    if report.get('R_body_camera'):
        report['R_body_camera']=(np.array(report['R_body_camera'])@np.array(bundle['T_left_color'])[:3,:3].T).tolist()
        report['camera_model']=bundle['raw_models']['left'];report['measured_stream']='color_converted_to_raw_left_ir'
    (folder/'mount_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    if not report['usable']:raise SystemExit('계산 불가: '+', '.join(report['reasons'])+'; '+str(folder/'mount_report.json'))
    print('yaw %.2f°(태그) pitch %.2f° roll %.2f°(%s) · 방향 σ %.2f° · 정지 %.0f s · 블록 %d (차체가 태그 법선과 평행하다고 가정)'%(
        report['yaw_left_deg'],report['pitch_down_deg'],report['roll_deg'],report['pitch_roll_source'],report['rotation_std_deg'],report['rest_s'],report['blocks']))
    if len(report['warnings'])>1:print('경고(기록만): '+', '.join(report['warnings'][1:]))
    if args.write:
        out=save_mount(args.common,report,before);print('장착각 저장: '+str(out)+' (common.json 은 그대로)')
    else:print('저장 안 함 (--write 를 주면 configs/mount.json 에 저장). 보고서: '+str(folder/'mount_report.json'))


def parser():
    p=argparse.ArgumentParser(description='check(환경·장치 점검, CAN 안 엶) / mount(태그 정면에 세우고 가만히 찍은 measure 로그로 장착각 계산, camyaw 방식, CAN 안 엶)')
    p.add_argument('command',choices=['check','mount']);p.add_argument('--common',default=str(COMMON))
    p.add_argument('--mode',choices=['central','left'],default='central');p.add_argument('--sensor-only',action='store_true')
    p.add_argument('--camera-only',action='store_true');p.add_argument('--profile',choices=['core','mapped'],default='core')
    p.add_argument('--folder',help='mount: 태그 정면에 세우고 30초 이상 가만히 찍은 measure 폴더');p.add_argument('--write',action='store_true',help='mount: 결과를 configs/mount.json 에 저장 (common.json 은 그대로)')
    return p

if __name__=='__main__':
    a=parser().parse_args();{'check':check,'mount':mount}[a.command](a)
