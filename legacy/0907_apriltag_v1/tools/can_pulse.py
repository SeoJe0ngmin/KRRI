"""CAN 명령 펄스 시험 — 명령 하나를 잠깐만 보내고 IMU 로 반응을 잰다.

    python tools/can_pulse.py rotate_ccw              # 0.5초 보내고 정지
    python tools/can_pulse.py rotate_cw 0.3
    python tools/can_pulse.py forward 0.5 --no-imu
    python tools/can_pulse.py rotate_ccw --dry-run    # CAN 없이, 보낼 바이트만 찍는다

도킹 루프(run.py)는 목표각에 닿을 때까지 계속 보내지만, 이건 정해진 시간만
보내고 반드시 정지한다. 새 매핑을 실차에 처음 붙일 때 "무엇이 움직이나 /
IMU 부호 / 각속도" 를 확인하는 용도다(2026-09-07 byte4 리프트 사고 뒤에 만듦).
포크를 내리고 주변을 비우고, 비상정지를 잡은 사람과 함께 돌릴 것. Ctrl+C 는
즉시 정지 프레임으로 넘어간다.
"""
import argparse
import asyncio
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from src.models.control.control_forklift_v2 import (      # noqa: E402
    MOVEMENT_TEMPLATES, CONTROL_TEMPLATES, CAN_MOVEMENT_ID, CAN_CONTROL_ID)

MAX_SEC = 2.0          # 펄스 상한. 그 이상은 run.py 의 몫이다


def _bytes(seq):
    return " ".join("%3d" % b for b in seq)


async def main_async(args):
    if args.movement not in MOVEMENT_TEMPLATES:
        raise SystemExit("모르는 명령: %s  (가능: %s)" % (args.movement, ", ".join(MOVEMENT_TEMPLATES)))
    sec = max(0.05, min(MAX_SEC, args.sec))
    print("명령       : %s  %.2f초" % (args.movement, sec))
    print("0x%03X 주행 : %s   (byte0..7, 중립 127)" % (CAN_MOVEMENT_ID, _bytes(MOVEMENT_TEMPLATES[args.movement])))
    print("0x%03X 모드 : %s   (byte4 는 카운터)" % (CAN_CONTROL_ID, " ".join("%02X" % b for b in CONTROL_TEMPLATES["driving_mode"])))

    # IMU 를 카메라/CAN 보다 먼저 — RSUSB 백엔드는 먼저 연 쪽이 IMU 를 갖는다
    yaw = None
    if not args.no_imu:
        try:
            from src.utils.imu_yaw import GyroYaw
            yaw = GyroYaw().start()
            print("자이로 열림. 2초 정지 보정...")
            rep = await asyncio.to_thread(yaw.calibrate)
            print("  축 %s / 잡음 %.3f도/s%s" % (rep["axis_src"], rep["noise_dps"],
                                             "  !! 보정 중 움직임" if rep["moving"] else ""))
        except Exception as exc:
            print("!! 자이로를 못 열었다 (%s) — 각도 없이 진행" % exc)
            yaw = None

    if args.dry_run:
        print("DRY-RUN — CAN 으로 아무것도 안 보낸다")
        if yaw is not None:
            yaw.close()
        return

    from src.models.control.control_forklift_v2 import DirectFrameForkliftController
    ctrl = DirectFrameForkliftController()
    if not ctrl.connect_can():
        if yaw is not None:
            yaw.close()
        raise SystemExit("CAN 연결 실패")
    ctrl.is_running = True
    tasks = [asyncio.create_task(ctrl.control_tx_loop()),
             asyncio.create_task(ctrl.movement_tx_loop()),
             asyncio.create_task(ctrl.heartbeat_loop())]
    await asyncio.sleep(0.5)                       # 시작 버스트가 나갈 시간
    samples = []
    try:
        if not args.yes:
            await asyncio.to_thread(input, "  준비되면 엔터 — %s 를 %.2f초 보낸다 (Ctrl+C 취소) > " % (args.movement, sec))
        if yaw is not None:
            yaw.zero()
        t0 = time.perf_counter()
        ctrl.current_movement = args.movement
        phase = "send"
        while True:
            t = time.perf_counter() - t0
            if phase == "send" and t >= sec:
                ctrl.current_movement = "stop"
                phase = "settle"
            if t >= sec + 1.5:
                break
            a = yaw.angle_deg if yaw is not None else float("nan")
            r = yaw.rate_dps if yaw is not None else float("nan")
            samples.append((t, phase, a, r))
            await asyncio.sleep(0.05)
    except KeyboardInterrupt:
        print("\n중단 — 정지 프레임으로 넘어간다")
    finally:
        ctrl.current_movement = "stop"
        await asyncio.sleep(0.3)                   # 정지 프레임이 몇 번 나가게
        ctrl.is_running = False
        for t in tasks:
            t.cancel()
        ctrl.disconnect_can()
        if yaw is not None:
            yaw.close()

    if not samples:
        return
    print("\n  t[s]   구간    yaw[도]  rate[도/s]")
    for t, ph, a, r in samples[::2]:
        print("  %5.2f  %-6s %8.2f  %8.2f" % (t, ph, a, r))
    send = [x for x in samples if x[1] == "send"]
    end_yaw = send[-1][2] if send else float("nan")
    final_yaw = samples[-1][2]
    peak = max((abs(x[3]) for x in samples), default=float("nan"))
    print("\n결과: 보내는 동안 yaw %+.2f도, 정지 후 최종 %+.2f도, 최대 각속도 %.1f도/s" % (end_yaw, final_yaw, peak))
    if yaw is not None and abs(final_yaw) >= 0.5:
        print("      부호: %s  (rotate_ccw 는 + 가 정상, 반대면 템플릿 좌/우를 바꿔라)" % ("+" if final_yaw > 0 else "-"))
    elif yaw is not None:
        print("      각도 변화 없음 — 차체가 안 돌았다. 눈으로 무엇이 움직였는지 확인할 것")


def main():
    ap = argparse.ArgumentParser(description="CAN 명령 펄스 시험 (짧게 보내고 IMU 로 잰다)")
    ap.add_argument("movement", help="MOVEMENT_TEMPLATES 이름 (rotate_ccw, rotate_cw, forward, ...)")
    ap.add_argument("sec", nargs="?", type=float, default=0.5, help="보내는 시간 [s], 최대 %.1f" % MAX_SEC)
    ap.add_argument("--no-imu", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="CAN 없이 바이트만 찍는다")
    ap.add_argument("--yes", action="store_true", help="엔터 확인 없이 바로 보낸다")
    asyncio.run(main_async(ap.parse_args()))


if __name__ == "__main__":
    main()
