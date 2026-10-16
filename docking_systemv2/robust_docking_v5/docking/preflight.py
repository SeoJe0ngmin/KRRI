"""Local checks and stationary mount measurement. Never opens CAN."""
import argparse
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
from .config import COMMON,load,save_mount

def check(args):
    cfg=load(args.common,args.mode,require_mount=not args.sensor_only)
    missing=[m for m in ('numpy','cv2','pyrealsense2','pupil_apriltags') if importlib.util.find_spec(m) is None]
    if not args.sensor_only and importlib.util.find_spec('canlib') is None:missing.append('canlib')
    if missing:raise ValueError('현재 Python에 없는 모듈: '+', '.join(missing))
    if not args.sensor_only:
        from .geometry import cut_position
        print('예상 태그 가시 전환 카메라 x: %.3f m'%cut_position(cfg))
        print('포크 앞끝 목표 x: %.2f m (입구면 x=0)'%cfg['target_front_x'])
    from .canbus import templates
    from .runtime import CAN_CONFIG
    frames=templates(CAN_CONFIG)
    print('CAN 프레임 설정 검사 통과: '+str(frames))
    print('CAN 장치를 열거나 송신하지 않았습니다. 실제 통신/제동 확인을 대신하지 않습니다.')
    if args.devices:
        import pyrealsense2 as rs
        devices=list(rs.context().query_devices())
        for d in devices:
            print(d.get_info(rs.camera_info.name),d.get_info(rs.camera_info.serial_number))
            print('센서:',[s.get_info(rs.camera_info.name) for s in d.query_sensors()])
        if len(devices)!=1:raise ValueError('RealSense 한 대 연결을 확인하세요')
    print('CHECK 통과'+(' (센서만; 현장 치수/장착 기준은 검사 제외)' if args.sensor_only else ''))

def mount(args):
    if not args.reference_aligned:
        raise ValueError('차량을 태그면에 수직·수평으로 정렬하고 태그를 수직으로 고정한 뒤 --reference-aligned를 붙이세요. 차량 방향 오차와 카메라 장착 오차는 정지 영상만으로 분리할 수 없습니다.')
    if not 30<=args.seconds<=120:raise ValueError('--seconds는 30..120초')
    original=Path(args.common).read_bytes();cfg=load(args.common,args.mode)
    cfg['sensor']['calibration'].update(T_body_camera=None,T_dock_tag=None,camera_model=None)
    cfg['sensor']['camera']['serial']=None
    cfg['sensor']['odom']['enabled']=False
    cfg['sensor']['tag']['enabled']=True
    from .runtime import run
    started=[None]
    def complete(o):
        if started[0] is None and o.get('sensor_ready') and not o.get('imu_bad'):
            started[0]=o['arrival']
        return started[0] is not None and o['arrival']-started[0]>=args.seconds+1
    options=SimpleNamespace(common=args.common,mode=args.mode,command='measure',execute=False,
                            seconds=cfg['tuning']['preparation_timeout_s'],view=args.view,port=args.port,output=None)
    print('장착각 측정: IMU 준비 후 %.0f초. 차량·카메라·태그를 고정하세요.'%args.seconds,flush=True)
    folder=run(options,cfg_override=cfg,stop_when=complete)
    rows=[json.loads(line) for line in (folder/'observations.jsonl').read_text().splitlines()]
    from .mount_calibration import estimate
    report=estimate(rows,cfg['sensor'],json.loads((folder/'camera_model.json').read_text()),args.seconds)
    (folder/'mount_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    if not report['usable']:raise ValueError('장착 기준 저장 안 함: '+', '.join(report['reasons'])+'; '+str(folder))
    save_mount(args.common,report,original)
    print('장착 기준 저장 완료: '+str(args.common))
    print('좌우 %.3f° / 상하 %.3f° / 롤 %.3f°'%(report['yaw_left_deg'],report['pitch_down_deg'],report['roll_deg']))
    print('다음 drive 시작 시 IMU 정지 보정은 다시 합니다. 장착 기준은 재장착할 때 다시 측정하세요.')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=['check','mount']);p.add_argument('--common',default=str(COMMON))
    p.add_argument('--mode',choices=['central','left'],default='central')
    p.add_argument('--sensor-only',action='store_true');p.add_argument('--devices',action='store_true')
    p.add_argument('--reference-aligned',action='store_true');p.add_argument('--seconds',type=float,default=60.)
    p.add_argument('--view',action='store_true');p.add_argument('--port',type=int,default=8765)
    args=p.parse_args()
    try:(check if args.command=='check' else mount)(args)
    except (ValueError,RuntimeError,OSError) as exc:
        print('CHECK 실패:',exc);return 2
    return 0

if __name__=='__main__':raise SystemExit(main())
