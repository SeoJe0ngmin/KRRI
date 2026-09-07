import importlib
import os
import sys
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
COMMON = [('numpy', 'numpy'), ('cv2', 'opencv-python'), ('pupil_apriltags', 'pupil-apriltags'), ('pyrealsense2', 'pyrealsense2'), ('matplotlib', 'matplotlib'), ('scipy', 'scipy'), ('shapely', 'shapely')]
DRIVE = [('canlib', 'canlib  (pip 외에 Kvaser 드라이버도 OS 에 설치해야 한다)'), ('keyboard', 'keyboard')]
MAC = sys.platform == 'darwin'

def try_imports(pairs):
    missing = []
    for module, pip_name in pairs:
        try:
            importlib.import_module(module)
            print('   OK    %s' % module)
        except Exception as exc:
            print('   없음  %-16s -> pip install %s   (%s)' % (module, pip_name.split()[0], type(exc).__name__))
            missing.append(pip_name)
    return missing

def main():
    print('── 공통 (검출·시뮬·dry-run) ' + '─' * 38)
    missing_common = try_imports(COMMON)
    print('── 저장소 자체 점검 ' + '─' * 46)
    try:
        import config.main
        from src.models.control.control_from_pose import plan_step
        from src.models.detection.detection_pose import measure
        print('   OK    config + 검출 + 제어 모듈이 전부 import 된다')
        repo_ok = True
    except Exception as exc:
        print('   깨짐  %s: %s' % (type(exc).__name__, exc))
        print('         (clone 이 불완전하거나 폴더 위치가 다르다)')
        repo_ok = False
    print('── 실주행 (CAN) ' + '─' * 50)
    if MAC:
        missing_drive = []
        print('   해당 없음  macOS 는 Kvaser CANlib 이 없어 실주행을 못 한다.')
        print('              dry-run / 시뮬 / 기록분석은 된다. 카메라는 컬러만')
        print('              (live_pose.py --source webcam). depth/IR/IMU 는 안 된다.')
    else:
        missing_drive = try_imports(DRIVE)
    print('── 장비 (지금 꽂혀 있는 것) ' + '─' * 38)
    try:
        import pyrealsense2 as rs
        n = len(rs.context().query_devices())
        print('   카메라        %d대 %s' % (n, '' if n else '(현장에서 꽂으면 됨)'))
    except Exception:
        print('   카메라        확인 불가 (pyrealsense2 없음)')
    try:
        from canlib import canlib as _cl
        n = _cl.getNumberOfChannels()
        print('   Kvaser 채널   %d개 %s' % (n, '' if n else '(현장에서 꽂으면 됨)'))
        for i in range(n):
            print('                 ch%d: %s' % (i, _cl.ChannelData(i).channel_name))
    except ImportError:
        print('   Kvaser 채널   확인 불가 (canlib 없음)')
    except Exception as exc:
        print('   Kvaser 채널   드라이버 문제일 수 있다: %s' % exc)
        print("                 -> kvaser.com 의 'Kvaser Drivers for Windows' 를 설치했나?")
    print('─' * 66)
    if not missing_common and (not missing_drive) and repo_ok:
        if MAC:
            print('판정: 개발 준비 완료 (macOS — 실주행·RealSense SDK 불가, 컬러는 webcam 소스).')
        else:
            print('판정: 준비 완료. 현장에서는 장비만 꽂고 run.py 를 켜면 된다.')
        return 0
    if not missing_common and repo_ok:
        print('판정: 개발·dry-run 은 된다. 실주행 전에 위 [실주행] 항목을 설치할 것.')
        return 1
    print("판정: 아직 안 된다. 위 '없음' 항목부터.")
    return 1
if __name__ == '__main__':
    sys.exit(main())
