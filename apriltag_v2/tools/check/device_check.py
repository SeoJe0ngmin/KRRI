import argparse
import sys
import time
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import realsense_check as rc
from config.imu import IMU_BIAS_SEC, IMU_GYRO_HZ, IMU_MOVING_DPS
from config.detection import TAG_ID, TAG_SIZE_M

def step_imu(args):
    rc.section(5, 'IMU (자이로 드리프트 / 부호)')
    try:
        from src.utils.imu_yaw import GyroYaw
    except Exception as exc:
        return rc.mark('WARN', 'imu_yaw 임포트', rc.short(exc))
    try:
        yaw = GyroYaw(hz=args.hz).start()
    except Exception as exc:
        rc.mark('FAIL', 'gyro 스트림 열기', rc.short(exc))
        rc.info('→ RSUSB 는 카메라를 먼저 열면 자이로가 죽는다. 다른 프로세스가 카메라를 잡고 있나 확인')
        return
    try:
        rep = yaw.calibrate(args.bias_sec)
        rc.mark('PASS' if not rep['moving'] else 'WARN', '정지 보정', '%d 샘플/%.1fs, 축=%s, 잡음 %.3f도/s, 예상 드리프트 %.2f도/분' % (rep['n'], rep['sec'], rep['axis_src'], rep['noise_dps'], rep['drift_dpm']))
        if '가정' in rep['axis_src']:
            rc.info('→ 중력축을 못 잡았다(accel 없음). 회전각이 cos(기울기)만큼 적게 세진다')
        if rep['moving']:
            rc.info('→ 보정 중 %.1f도/s 초과 움직임 — 세워 두고 다시' % IMU_MOVING_DPS)
        print('\n         %.0f초 동안 0.5초마다 yaw. 가만히 두면 끝값이 드리프트, 왼쪽(반시계)으로 ~90도 돌리면 +90 근처여야 한다\n' % args.imu_sec)
        t0 = time.monotonic()
        nxt = t0
        while time.monotonic() - t0 < args.imu_sec:
            now = time.monotonic()
            if now >= nxt:
                nxt += 0.5
                warn = '' if yaw.alive else '   !! gyro 끊김 %.1fs' % yaw.age_sec
                rc.info('t %5.1fs   yaw %+8.2f도   rate %+7.2f도/s%s' % (now - t0, yaw.angle_deg, yaw.rate_dps, warn))
            time.sleep(0.05)
        st = yaw.stats()
        drift_dpm = abs(yaw.angle_deg) / (args.imu_sec / 60.0)
        rc.mark('PASS' if st['gaps'] == 0 else 'WARN', 'yaw 적분', '끝 yaw %+.2f도, %.1fHz, 적분 건너뜀 %d회 (가만히 뒀다면 %.2f도/분 드리프트)' % (yaw.angle_deg, st['hz'], st['gaps'], drift_dpm))
        rc.info('→ 부호가 반대면(반시계인데 -) config.IMU_YAW_SIGN 을 뒤집어라')
    finally:
        yaw.close()

def step_imu_tag(args):
    rc.section(6, 'IMU vs 태그 heading (부호·배율 교차검증)')
    from src.utils.imu_yaw import GyroYaw
    from src.models import TagPipeline
    yaw = GyroYaw(hz=args.hz).start()
    try:
        yaw.calibrate(args.bias_sec)
        pipe = TagPipeline.from_realsense(args.tag_size, width=args.width, height=args.height, fps=30, label='device_check')
        rc.info('태그 %d 를 화면에 두고 ±30도 안에서 좌우로 (%.0f초). 반시계면 둘 다 +' % (args.tag_id, args.imu_sec))
        print('%9s %12s %12s %10s' % ('t[s]', 'IMU yaw', '태그 Δhead', '차이'))
        h0 = None
        diffs = []
        t0 = time.monotonic()
        nxt = 0.0
        try:
            for i, ts, frame in pipe.frames:
                if time.monotonic() - t0 >= args.imu_sec:
                    break
                res = pipe.process(frame, index=i, timestamp=ts)
                d = res.docking.get(args.tag_id)
                now = time.monotonic() - t0
                if now < nxt:
                    continue
                nxt = now + 0.3
                iy = yaw.angle_deg
                if d is None:
                    print('%9.1f %+11.2f도 %12s' % (now, iy, '태그 안 보임'))
                    continue
                h = d['heading_deg']
                if h0 is None:
                    h0 = h
                    yaw.zero()
                    iy = 0.0
                dh = h - h0
                diffs.append(iy - dh)
                print('%9.1f %+11.2f도 %+11.2f도 %+9.2f도' % (now, iy, dh, iy - dh))
        finally:
            pipe.close()
        if diffs:
            import statistics
            rc.mark('PASS' if abs(statistics.mean(diffs)) < 5 else 'WARN', 'IMU−태그 차이', '평균 %+.2f도, 표준편차 %.2f도 (n=%d)' % (statistics.mean(diffs), statistics.pstdev(diffs), len(diffs)))
            rc.info('→ 부호가 반대로 움직였으면 config.IMU_YAW_SIGN 을 뒤집어라')
        else:
            rc.mark('WARN', 'IMU−태그 차이', '태그가 잡힌 프레임이 없다')
    finally:
        yaw.close()

def main():
    ap = argparse.ArgumentParser(description='장치 점검 — RealSense 카메라 + IMU 한 번에')
    ap.add_argument('--no-gui', dest='gui', action='store_false', help='cv2 창 검사 생략')
    ap.add_argument('--no-imu', dest='imu', action='store_false', help='IMU 검사 생략')
    ap.add_argument('--imu-sec', type=float, default=15.0, help='IMU 관찰 시간 [s]')
    ap.add_argument('--bias-sec', type=float, default=IMU_BIAS_SEC, help='정지 보정 [s]')
    ap.add_argument('--hz', type=int, default=IMU_GYRO_HZ, choices=[200, 400])
    ap.add_argument('--tag', action='store_true', help='IMU 를 태그 heading 과 나란히 (부호·배율)')
    ap.add_argument('--tag-id', type=int, default=TAG_ID)
    ap.add_argument('--tag-size', type=float, default=TAG_SIZE_M)
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--force-mac', action='store_true', help='macOS 에서도 장치를 연다(보통 죽는다)')
    args = ap.parse_args()
    print('=' * 74)
    print(' 장치 점검 (device_check)  |  %s' % time.strftime('%Y-%m-%d %H:%M:%S'))
    print(' 카메라 = realsense_check, IMU = imu_check 를 한 실행으로. 설치만 = check_setup')
    print('=' * 74)
    rs = rc.step_import()
    if rs is None:
        rc.section(2, '장치 유무')
        rc.mark('SKIP', '장치 조회', 'pyrealsense2 없음')
        return _summary()
    if sys.platform == 'darwin' and (not args.force_mac):
        rc.section(2, '장치 유무')
        rc.mark('SKIP', '장치 조회', 'macOS 는 SDK 로 카메라를 못 연다 (--force-mac 으로 강행)')
        rc.step_gui(enabled=args.gui)
        return _summary()
    ndev = rc.count_devices(rs)
    rc.step_gui(enabled=args.gui)
    if ndev == 0:
        rc.step_no_device_help()
        return _summary()
    if args.imu and (not args.tag):
        step_imu(args)
    rc.step_frame_metadata(rs)
    if args.imu and args.tag:
        step_imu_tag(args)
    _summary()

def _summary():
    print('\n' + '=' * 74)
    t = rc.TALLY
    print(' 결과: PASS %d  FAIL %d  WARN %d  SKIP %d' % (t['PASS'], t['FAIL'], t['WARN'], t['SKIP']))
    print('=' * 74)
    return 1 if t['FAIL'] else 0
if __name__ == '__main__':
    try:
        sys.exit(main() or 0)
    except KeyboardInterrupt:
        print('\n중단')
        sys.exit(130)
