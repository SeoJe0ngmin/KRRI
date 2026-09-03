"""IMU yaw 가 제대로 나오는지 확인한다 — 드리프트 측정 + 부호 확정 + 태그 교차검증.

    python tools/imu_check.py                    정지 드리프트 + 손 회전 확인 (60초)
    python tools/imu_check.py --duration 20
    python tools/imu_check.py --tag              태그 heading 과 나란히 (부호·배율 검증)

절차:
    1) 시작하면 2초 정지 보정 — 그동안 카메라를 절대 만지지 말 것
    2) 0.5초마다 yaw 가 찍힌다
       - 가만히 두면: 끝날 때 |yaw| 가 드리프트다. 실측 기준 1도/분 근처여야 한다
       - 카메라를 **왼쪽(반시계)** 으로 ~90도 돌리면: yaw 가 **+90 근처**여야 한다
         반대 부호로 나오면 config.IMU_YAW_SIGN 을 뒤집고 다시 확인할 것
    3) --tag: 태그를 화면에 두고 ±30도 안에서 좌우로 돌린다.
       IMU yaw 와 태그 Δheading 이 같이 움직이고 차이가 작아야 한다.
       (반시계로 돌리면 둘 다 + 로 — handspin 실측과 같은 방향)
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from config.system import (IMU_BIAS_SEC, IMU_GYRO_HZ, IMU_MOVING_DPS,
                        TAG_ID, TAG_SIZE_M)   # noqa: E402
from src.utils.imu_yaw import GyroYaw                                   # noqa: E402


def print_calib(rep):
    print("보정 완료: %d 샘플 / %.1fs" % (rep["n"], rep["sec"]))
    print("  바이어스 [도/s]  x %+6.3f  y %+6.3f  z %+6.3f" % rep["bias_dps"])
    print("  회전축          (%+.3f, %+.3f, %+.3f)  <- %s" % (rep["axis"] + (rep["axis_src"],)))
    print("  회전축 잡음     %.3f 도/s   예상 드리프트 %.2f 도/분"
          % (rep["noise_dps"], rep["drift_dpm"]))
    if "가정" in rep["axis_src"]:
        print("  !! 중력축을 못 잡았다 — 카메라가 기울어 달렸으면 회전각이 cos(기울기)만큼 적게 세진다")
    if rep["moving"]:
        print("  !! 보정 중 움직인 것 같다 (%.1f도/s 초과) — 세워 두고 다시 돌릴 것"
              % IMU_MOVING_DPS)


def run_plain(yaw, duration):
    """정지 드리프트 + 손 회전 확인. 화면 없이 숫자만."""
    print("\n지금부터 %d초 — 가만히 두거나, 왼쪽(반시계)으로 돌려 부호를 확인할 것\n"
          % duration)
    t0 = time.monotonic()
    next_p = t0
    while True:
        now = time.monotonic()
        if now - t0 >= duration:
            break
        if now >= next_p:
            next_p += 0.5
            warn = "" if yaw.alive else "   !! gyro 끊김 %.1fs" % yaw.age_sec
            print("t %5.1fs   yaw %+8.2f도   rate %+7.2f도/s%s"
                  % (now - t0, yaw.angle_deg, yaw.rate_dps, warn))
        time.sleep(0.05)
    st = yaw.stats()
    print("\n끝. 최종 yaw %+.2f도 / %.1f초  (gyro %d개, %.1fHz, 적분 건너뜀 %d회)"
          % (yaw.angle_deg, duration, st["n"], st["hz"], st["gaps"]))
    print("가만히 뒀다면 |최종 yaw| 가 곧 드리프트다 — %.1f분 환산 %.2f도/분"
          % (duration / 60.0, abs(yaw.angle_deg) / (duration / 60.0)))


def run_tag(yaw, args):
    """태그 heading 변화와 IMU yaw 를 나란히 — 부호와 배율을 한 번에 검증."""
    from src.models import TagPipeline
    pipe = TagPipeline.from_realsense(args.tag_size, width=args.width,
                                      height=args.height, fps=30, label="imu_check")
    print("\n카메라 열림. 태그 %d 를 화면에 두고 ±30도 안에서 좌우로 돌릴 것 (%d초)"
          % (args.tag_id, args.duration))
    print("반시계로 돌리면 IMU 와 Δheading 둘 다 + 여야 한다\n")
    print("%7s %12s %12s %10s" % ("t[s]", "IMU yaw", "태그 Δhead", "차이"))

    h0 = None
    diffs = []
    t0 = time.monotonic()
    next_p = 0.0
    try:
        for i, ts, frame in pipe.frames:
            if time.monotonic() - t0 >= args.duration:
                break
            res = pipe.process(frame, index=i, timestamp=ts)
            d = res.docking.get(args.tag_id)
            now = time.monotonic() - t0
            if now < next_p:
                continue
            next_p = now + 0.3
            iy = yaw.angle_deg
            if d is None:
                print("%7.1f %+11.2f도 %12s" % (now, iy, "태그 안 보임"))
                continue
            h = d["heading_deg"]
            if h0 is None:
                h0 = h
                yaw.zero()                 # 태그 기준과 IMU 영점을 같은 순간에 맞춤
                iy = 0.0
            dh = h - h0
            diffs.append(iy - dh)
            flag = "" if d.get("tilt_deg", 99) >= 10 else "  (정면이라 heading 잡음 큼)"
            print("%7.1f %+11.2f도 %+11.2f도 %+9.2f도%s" % (now, iy, dh, iy - dh, flag))
    finally:
        pipe.close()

    if diffs:
        import statistics
        print("\nIMU − 태그 차이: 평균 %+.2f도, 표준편차 %.2f도 (n=%d)"
              % (statistics.mean(diffs), statistics.pstdev(diffs), len(diffs)))
        print("부호가 반대로 움직였다면 config.IMU_YAW_SIGN 을 뒤집을 것.")
    else:
        print("\n태그가 잡힌 프레임이 없어 비교를 못 했다.")


def main():
    ap = argparse.ArgumentParser(description="IMU yaw 확인 (드리프트/부호/태그 교차검증)")
    ap.add_argument("--duration", type=float, default=60.0, help="확인 시간 [s]")
    ap.add_argument("--bias-sec", type=float, default=IMU_BIAS_SEC,
                    help="정지 보정 시간 [s]")
    ap.add_argument("--hz", type=int, default=IMU_GYRO_HZ, choices=[200, 400])
    ap.add_argument("--tag", action="store_true",
                    help="컬러 스트림도 열어 태그 heading 과 나란히 찍는다")
    ap.add_argument("--tag-id", type=int, default=TAG_ID)
    ap.add_argument("--tag-size", type=float, default=TAG_SIZE_M)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    yaw = GyroYaw(hz=args.hz).start()
    try:
        print("gyro %dHz 열림. %.1f초 정지 보정 — 만지지 말 것..."
              % (args.hz, args.bias_sec))
        print_calib(yaw.calibrate(args.bias_sec))
        if args.tag:
            run_tag(yaw, args)
        else:
            run_plain(yaw, args.duration)
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        yaw.close()


if __name__ == "__main__":
    main()
