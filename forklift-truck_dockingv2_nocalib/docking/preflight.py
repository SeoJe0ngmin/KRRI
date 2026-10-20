"""check: environment and device checks, no CAN. v2_nocalib: there is no mounting calibration; the camera is assumed
mounted straight and the tag upright (2026-10-19)."""
import argparse
import importlib
import json
from pathlib import Path
import numpy as np
from .config import COMMON,load


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
    print('설정: OK; CAN 미사용. 장착각 보정 없음(카메라 정면 장착·태그 수직 전제). 장치 입력은 measure에서 확인합니다.')
    if args.sensor_only:print('sensor-only: 현장 치수 검사는 생략했습니다. 주행 준비 완료 판정이 아닙니다.')
    if failed:raise SystemExit('설치/환경 필요: '+', '.join(failed)+'; scripts/install_jetson.sh 참고')
    return cfg


def parser():
    p=argparse.ArgumentParser(description='check: 환경·장치 점검 (CAN 안 엶). v2_nocalib 에는 장착각 보정이 없다(카메라 정면 장착·태그 수직 전제)')
    p.add_argument('command',choices=['check']);p.add_argument('--common',default=str(COMMON))
    p.add_argument('--mode',choices=['central','left'],default='central');p.add_argument('--sensor-only',action='store_true')
    p.add_argument('--camera-only',action='store_true');p.add_argument('--profile',choices=['core','mapped'],default='core')
    return p

if __name__=='__main__':
    a=parser().parse_args();{'check':check}[a.command](a)
