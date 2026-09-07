"""CAN 명령 펄스 시험 — 명령 하나를 잠깐만 보내고 IMU 로 반응을 잰다.

    python tools/can_pulse.py rotate_ccw              # 0.5초 보내고 정지
    python tools/can_pulse.py rotate_cw 0.3
    python tools/can_pulse.py forward 0.5 --no-imu
    python tools/can_pulse.py rotate_ccw --dry-run    # CAN 없이, 보낼 바이트만 찍는다
    python tools/can_pulse.py rotate_ccw 1.0 --camera # 펄스 전후를 카메라(30프레임)로도 잰다
                                                     #   -> 회전 팔 길이 A = Δlateral / sin(Δheading), 직진 속도

도킹 루프(run.py)는 목표각에 닿을 때까지 계속 보내지만, 이건 정해진 시간만
보내고 반드시 정지한다. 새 매핑을 실차에 처음 붙일 때 "무엇이 움직이나 /
IMU 부호 / 각속도" 를 확인하는 용도다(2026-09-07 byte4 리프트 사고 뒤에 만듦).
포크를 내리고 주변을 비우고, 비상정지를 잡은 사람과 함께 돌릴 것. Ctrl+C 는
즉시 정지 프레임으로 넘어간다.
"""
import argparse
import asyncio
import math
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from src.models.control.control_forklift_v2 import (      # noqa: E402
    MOVEMENT_TEMPLATES, CONTROL_TEMPLATES, CAN_MOVEMENT_ID, CAN_CONTROL_ID)

MAX_SEC = 5.0          # 펄스 상한. 회전은 지연 0.85s 뒤에야 돌기 시작해 2~3s 는 줘야 5~15도, 직진은 3~5s 는 되어야 정속 구간이 들어온다


def _measure_now(pipe, tag_id, n=30, max_frames=150):
    """지금 카메라로 30프레임 중앙값 하나. 태그가 없으면 None."""
    from src.models.detection.detection_pose import measure
    buf = []
    for i, (idx, ts, frame) in enumerate(pipe.frames):
        res = pipe.process(frame, index=idx, timestamp=ts)
        if tag_id in res.docking:
            res.image = None
            buf.append(res)
        if len(buf) >= n or i >= max_frames:
            break
    return measure(buf, tag_id=tag_id, n=n) if buf else None


def _fmt_m(m):
    if m is None:
        return "(태그 안 보임)"
    return "lateral %+.3fm  forward %.3fm  heading %+.2f도" % (m["lateral"], m["forward"], m["heading_deg"])


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

    pipe = None
    m0 = None
    from config.detection import TAG_ID, TAG_SIZE_M
    tag_id = TAG_ID
    if args.camera:
        from src.models import TagPipeline
        for attempt in (1, 2):
            pipe = TagPipeline.from_realsense(TAG_SIZE_M, label="pulse")
            try:
                m0 = _measure_now(pipe, tag_id)
                break
            except RuntimeError as exc:
                # 직전 실행을 Ctrl+C 로 끊은 직후엔 첫 프레임이 5초 안에 안 오기도 한다. 닫고 한 번 더.
                print("!! 카메라 첫 프레임 실패 (%s) — %s" % (str(exc)[:40], "2초 뒤 다시 연다" if attempt == 1 else "포기"))
                pipe.close(); pipe = None
                if attempt == 2:
                    raise SystemExit("카메라가 응답하지 않는다. UTM USB 메뉴에서 RealSense 를 뺐다 다시 넣고 재시도")
                await asyncio.sleep(2.0)
        print("카메라 전 : %s" % _fmt_m(m0))

    if args.dry_run:
        print("DRY-RUN — CAN 으로 아무것도 안 보낸다")
        if pipe is not None:
            pipe.close()
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
            cf = cl = ch = float("nan")
            if pipe is not None:
                # 카메라 한 프레임 (약 30ms). 정지 명령이 그만큼 늦을 수 있지만 시험 도구라 감수한다
                try:
                    idx, ts, frame = next(pipe.frames)
                    d = pipe.process(frame, index=idx, timestamp=ts).docking.get(tag_id)
                    if d is not None:
                        cf, cl, ch = d["forward"], d["lateral"], d["heading_deg"]
                except StopIteration:
                    pass
            samples.append((t, phase, a, r, cf, cl, ch))
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

    m1 = None
    if pipe is not None:
        await asyncio.sleep(1.0)                   # 완전히 멎은 뒤에 잰다
        m1 = _measure_now(pipe, tag_id)
        print("카메라 후 : %s" % _fmt_m(m1))
        pipe.close()
    if yaw is not None:
        yaw.close()

    if not samples:
        return
    have_cam = any(not math.isnan(x[4]) for x in samples)
    if have_cam:
        print("\n  t[s]   구간    yaw[도]  rate[도/s]   cam fwd[m]  cam lat[m]  cam head[도]")
        for t, ph, a, r, cf, cl, ch in samples[::2]:
            print("  %5.2f  %-6s %8.2f  %8.2f   %9.3f  %9.3f  %10.2f" % (t, ph, a, r, cf, cl, ch))
    else:
        print("\n  t[s]   구간    yaw[도]  rate[도/s]")
        for t, ph, a, r, *_ in samples[::2]:
            print("  %5.2f  %-6s %8.2f  %8.2f" % (t, ph, a, r))
    send = [x for x in samples if x[1] == "send"]
    end_yaw = send[-1][2] if send else float("nan")
    final_yaw = samples[-1][2]
    peak = max((abs(x[3]) for x in samples), default=float("nan"))
    print("\n결과: 보내는 동안 yaw %+.2f도, 정지 후 최종 %+.2f도, 최대 각속도 %.1f도/s" % (end_yaw, final_yaw, peak))
    if have_cam and args.movement in ("forward", "backward"):
        cam = [(t, ph, cf) for t, ph, a, r, cf, cl, ch in samples if not math.isnan(cf)]
        if len(cam) >= 6:
            f0 = cam[0][2]
            onset = next((t for t, ph, cf in cam if abs(cf - f0) > 0.03), None)      # 3cm 움직인 시각 = 지연
            send_c = [(t, cf) for t, ph, cf in cam if ph == "send" and onset is not None and t >= onset + 0.5]
            v = None
            if len(send_c) >= 4:
                ts_ = [t for t, _ in send_c]; fs_ = [f for _, f in send_c]
                tm, fm = sum(ts_) / len(ts_), sum(fs_) / len(fs_)
                den = sum((t - tm) ** 2 for t in ts_)
                v = -sum((t - tm) * (f - fm) for t, f in zip(ts_, fs_)) / den if den else None   # forward 는 줄어드니 부호 반전
            at_stop = next((cf for t, ph, cf in cam if ph == "settle"), None)
            final_f = cam[-1][2]
            print("카메라 시간축: 움직이기 시작 %s  정속 %s  정지 명령 뒤 더 간 거리(관성) %s"
                  % ("%.2fs 뒤" % onset if onset is not None else "(3cm 도 안 움직임)",
                     "%.3fm/s" % v if v is not None else "(정속 구간 부족 — 더 길게)",
                     "%.3fm" % (at_stop - final_f) if at_stop is not None else "?"))
    if m0 is not None and m1 is not None:
        dl, df, dh = m1["lateral"] - m0["lateral"], m1["forward"] - m0["forward"], m1["heading_deg"] - m0["heading_deg"]
        print("카메라 변화: lateral %+.3fm  forward %+.3fm  heading %+.2f도" % (dl, df, dh))
        if args.movement.startswith("rotate") and abs(dh) >= 1.0:
            print("      회전 팔 길이 A = Δlateral/sin(Δheading) = %.2fm  (config CAM_TO_PIVOT_M 후보)" % abs(dl / math.sin(math.radians(dh))))
        elif args.movement in ("forward", "backward"):
            moved = math.hypot(dl, df)
            print("      이동 %.3fm / %.2fs 명령 -> 평균 %.3fm/s (지연 포함)" % (moved, sec, moved / sec))
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
    ap.add_argument("--camera", action="store_true", help="펄스 전후를 카메라 30프레임 중앙값으로 잰다 (태그가 보여야 한다)")
    asyncio.run(main_async(ap.parse_args()))


if __name__ == "__main__":
    main()
