"""도킹 자동 실행 (v2: 조준-전진) — 시작만 키보드, 그 뒤는 카메라가 정한다.

    python tools/run.py --dry-run --show    화면 보며 순서 확인 (CAN 안 씀)
    python tools/run.py --show              실제 주행 + 화면
    python tools/run.py                     실제 주행 (화면 없음)

**화면과 제어가 한 프로세스여야 한다.** 카메라는 한 프로세스만 열 수 있어서,
live_pose.py 를 따로 띄우면 이쪽이 카메라를 못 연다. 그래서 --show 로 여기서 그린다.

키보드는 시작과 비상정지, 그리고 카메라 노출 조절에만 쓴다. 주행 방향은 카메라가 정한다.
    SPACE  시작        ESC  비상정지 후 종료
    (시작 키는 터미널에서 직접 읽는다 — SSH 로 들어온 VS Code 터미널, Jetson 에서도 된다.
     keyboard 라이브러리는 물리 키보드 장치를 읽어서 SSH 키를 못 보므로 터미널이 없을 때만 쓴다)
    e      자동노출 켜기/끄기      [ ]  노출 -/+       - =  게인 -/+

control_forklift_v2.py 는 한 줄도 안 고친다. 그쪽 TX 루프(movement 10ms /
control 5ms / heartbeat 200ms)를 그대로 띄워 두고, current_movement 만 바꾼다.
"""
import argparse
import asyncio
import os
import sys
from datetime import datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# --record-events. 실행마다 새 파일 — live_pose.py 의 --log 와 같은 방식
# (시각으로 이름 지음). 한 파일에 계속 이어붙이면 오래된 실행과 섞여서
# "오늘 이 세션에서 뭐가 있었나"를 보기 번거로워진다.
EVENT_LOG_DIR = os.path.join(ROOT, "work_dirs", "docking_log")


def _new_event_log_dir():
    return os.path.join(EVENT_LOG_DIR, datetime.now().strftime("%Y%m%d_%H%M%S"))

from config.main import TAG_ID, TAG_SIZE_M                                # noqa: E402
from src.models import TagPipeline                                       # noqa: E402
from src.models.control.control_from_pose import (CanDriver,             # noqa: E402
                                                  DryRunDriver, dock_live)
from config.main import MAX_STEPS                                  # noqa: E402
from src.models.detection.image import intrinsics_from_ref        # noqa: E402                        # noqa: E402
from src.utils.imu_yaw import GyroYaw                                    # noqa: E402

PHASE_KO = {"measure": "측정 중", "command": "명령 실행 중", "search": "Set3: 태그 찾는 중",
           "final": "마지막 접근 중", "done": "끝", "manual": "!! 수동전환 필요 !!"}


from src.utils.imu_yaw import imu_panel_lines as imu_lines   # noqa: E402  (live_pose 와 공용)


def make_view(args, yaw=None):
    """--show 일 때 프레임마다 그리는 함수를 만든다. 아니면 None."""
    if not args.show:
        return None, None
    import cv2
    import numpy as np
    import live_pose as L

    win = "docking"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    state = {"stop": False}

    def on_frame(res, info):
        vis = L.draw_overlay(res, args.tag_size, "cube")
        det = L.pick_primary(res, args.tag_id)
        ctx = {"source": "run", "w": vis.shape[1], "h": vis.shape[0],
               "frame": res.index, "fps": 0.0, "intr": res.intrinsics,
               "intr_origin": "factory", "intr_assumed": False,
               "tag_size": args.tag_size, "size_assumed": False,
               "res": res, "primary": det, "paused": False, "draw": "cube",
               "family": "tag36h11", "method": "auto", "quad_blur": 0.0,
               "stats": None}
        items = L.build_lines(ctx)
        # 맨 위에 지금 무슨 단계인지 끼운다 — 화면만 보고도 진행이 보이게
        head = [("sec", "%d단계   %s" % (info["step"], PHASE_KO.get(info["phase"], "")))]
        if info["phase"] == "measure":
            head.append(("kv", "모으는 중", "%d / %d 프레임" % (info["n"], info["need"]), "ok"))
        elif info["phase"] in ("command", "search", "final"):
            head.append(("kv", info.get("action", ""), "남은 %.1fs" % info["left"], "warn"))
            head.append(("kv", "", info.get("why", "")[:44], "dim"))
        elif info["phase"] == "manual":
            head.append(("kv", "", info.get("why", "")[:44], "bad"))
        head += imu_lines(yaw)
        items = items[:2] + head + [("rule",)] + items[2:]

        panel = L.render_panel(items, 470, vis.shape[0])
        h = max(vis.shape[0], panel.shape[0])
        canvas = np.full((h, vis.shape[1] + panel.shape[1], 3), L.PANEL_BG, np.uint8)
        canvas[:vis.shape[0], :vis.shape[1]] = vis
        canvas[:panel.shape[0], vis.shape[1]:] = panel
        cv2.imshow(win, canvas)
        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord('q')):
            state["stop"] = True
    return on_frame, state


def _read_key_tty():
    """터미널에서 키 하나를 읽는다. 줄 단위·에코 없이. ESC 는 '\x1b' 로 온다."""
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


async def _wait_start(show):
    """SPACE 를 기다린다.

    터미널이 붙어 있으면 stdin 을 키 단위로 읽는다 — SSH(VS Code Remote-SSH,
    Jetson)에서도 된다. keyboard 라이브러리는 리눅스의 물리 키보드 장치를 직접
    읽어서 SSH 로 친 키를 못 보고 root 까지 필요하므로, 터미널이 없을 때만 쓴다.
    그것도 없으면 엔터. 기다리는 동안 CAN heartbeat 루프는 계속 돈다(스레드로 읽음).
    """
    if os.name != "nt" and sys.stdin.isatty():
        print("  SPACE 를 누르면 시작. ESC 면 중단.", flush=True)
        while True:
            k = await asyncio.to_thread(_read_key_tty)
            if k == " ":
                return
            if k in ("\x1b", "q"):       # 화살표 키도 \x1b 로 시작하니 누르지 말 것
                raise KeyboardInterrupt
    try:
        import keyboard
    except ImportError:
        await asyncio.to_thread(input, "  시작하려면 엔터 (keyboard 미설치) > ")
        return
    print("  SPACE 를 누르면 시작. ESC 면 중단.")
    while True:
        if keyboard.is_pressed("space"):
            return
        if keyboard.is_pressed("esc"):
            raise KeyboardInterrupt
        await asyncio.sleep(0.02)



def _check_can_templates():
    """보내기 전에 CAN 템플릿을 눈으로 확인한다. 2026-09-07 실차에서 byte4 가 포크
    리프트였다(회전 명령이 포크를 올림). 우리가 쓰는 다섯 동작은 byte1(조향)·byte2(주행)만
    쓰고 나머지 바이트는 중립이어야 한다. 아니면 출발 자체를 막는다."""
    from src.models.control.control_forklift_v2 import MOVEMENT_TEMPLATES as M, AN_NEUTRAL
    for name in ("stop", "forward", "backward", "rotate_ccw", "rotate_cw"):
        bad = [i for i in (0, 3, 4, 5, 6, 7) if M[name][i] != AN_NEUTRAL]
        if bad:
            raise SystemExit("!! CAN 템플릿 %s 의 byte%s 가 중립(%d)이 아니다 — byte4 는 이 지게차에서 "
                             "포크 리프트다. 출발하지 않는다" % (name, bad, AN_NEUTRAL))
    print("  CAN 템플릿 확인: rotate_ccw byte1=%d  rotate_cw byte1=%d  forward byte2=%d  "
          "backward byte2=%d  (그 외 바이트 전부 중립)"
          % (M["rotate_ccw"][1], M["rotate_cw"][1], M["forward"][2], M["backward"][2]))


async def main_async(args):
    # 카메라보다 IMU 를 먼저 연다 — RSUSB 백엔드(맥북 VM, Jetson 소스 빌드)는
    # 먼저 연 device 객체가 IMU(HID) 인터페이스를 갖는다. 컬러 파이프라인이 먼저면
    # 뒤에 여는 자이로가 "failed to set power state" 로 죽는다(2026-09-07 실측).
    # IMU — 회전 폐루프의 눈. 없으면 회전이 미측정 시간모델 개루프로 떨어진다.
    yaw = None
    if not args.no_imu:
        try:
            yaw = GyroYaw().start()
            print("  자이로 열림. %.1f초 정지 보정 — 지게차를 세워 둘 것..." % 2.0)
            # calibrate 는 블로킹(time.sleep)이라 그냥 부르면 CAN heartbeat 가 끊긴다.
            rep = await asyncio.to_thread(yaw.calibrate)
            print("     회전축 %s,  잡음 %.3f 도/s,  드리프트 %.2f 도/분"
                  % (rep["axis_src"], rep["noise_dps"], rep["drift_dpm"]))
            if rep["moving"]:
                print("     !! 보정 중 움직인 것 같다 — 세워 두고 다시 돌리는 게 좋다")
        except Exception as exc:
            print("  !! 자이로를 못 열었다 (%s) — 회전이 시간모델 개루프가 된다" % exc)
            yaw = None

    try:
        pipe = TagPipeline.from_realsense(args.tag_size, label="run")
    except Exception:
        if yaw is not None:
            yaw.close()
        raise
    print("  카메라 열림. 태그 %d, 크기 %.3fm" % (args.tag_id, args.tag_size))

    # 기록 폴더는 여기서 만들지 않는다 — SPACE 를 눌러 실제로 주행이 시작되는
    # 순간의 시각으로 이름을 지어야, 폴더 이름과 "몇 시에 주행했다"가 맞는다.
    record_dir = None

    # CAN 템플릿 안전장치는 dry-run 에서도 찍는다 (canlib 없는 맥에서는 건너뜀)
    try:
        _check_can_templates()
    except ImportError as exc:
        if not args.dry_run:
            raise
        print("  (canlib 없음 — 템플릿 확인 생략: %s)" % exc)

    ctrl, tasks = None, []
    if args.dry_run:
        driver = DryRunDriver(realtime=True)
        print("  DRY-RUN — CAN 으로 아무것도 안 보낸다")
    else:
        from src.models.control.control_forklift_v2 import DirectFrameForkliftController
        ctrl = DirectFrameForkliftController()
        if not ctrl.connect_can():
            raise SystemExit("CAN 연결 실패")
        ctrl.is_running = True
        tasks = [asyncio.create_task(ctrl.control_tx_loop()),
                 asyncio.create_task(ctrl.movement_tx_loop()),
                 asyncio.create_task(ctrl.heartbeat_loop())]
        driver = CanDriver(ctrl, yaw=yaw, log=print)   # record_dir 는 주행 시작 때 넣는다
        await asyncio.sleep(0.5)
        print("  CAN 연결됨. TX 루프 3개 가동")

    on_frame, view = make_view(args, yaw)
    try:
        await _wait_start(args.show)
        if args.record_events:
            record_dir = _new_event_log_dir()          # <- 주행 시작 시각으로 이름 짓는다
            if not args.dry_run:
                driver.record_dir = record_dir         # CanDriver 는 기록 때마다 이 속성을 읽는다
            else:
                # dry-run 에도 카메라 쪽 기록(measure/decision/result)은 진짜다.
                # rotation/drive 는 하드웨어가 안 움직이므로 안 남는다(DryRunDriver 는 기록 안 함).
                print("  (dry-run: measure/decision/result 만 기록. rotation/drive 는 실주행에서만)")
            print("  운행 기록 -> %s/" % record_dir)
        print("  시작\n")
        await dock_live(pipe, driver, tag_id=args.tag_id,
                        max_steps=args.max_steps, on_frame=on_frame,
                        record_dir=record_dir)
    except KeyboardInterrupt:
        print("\n  중단")
    finally:
        await driver.stop()
        for t in tasks:
            t.cancel()
        if yaw is not None:
            yaw.close()
        if ctrl is not None:
            ctrl.is_running = False
            ctrl.disconnect_can()
        pipe.close()
        if args.show:
            import cv2
            cv2.destroyAllWindows()
        print("  정리 완료")


def main():
    ap = argparse.ArgumentParser(description="AprilTag 도킹 자동 실행")
    ap.add_argument("--dry-run", action="store_true",
                    help="CAN 없이 순서만 본다. 카메라는 실제로 쓴다")
    ap.add_argument("--show", action="store_true",
                    help="주행하면서 화면도 띄운다 (같은 프로세스라 카메라 충돌 없음)")
    ap.add_argument("--tag-id", type=int, default=TAG_ID)
    ap.add_argument("--tag-size", type=float, default=TAG_SIZE_M,
                    help="태그 한 변 [m], 검은 테두리 바깥까지")
    ap.add_argument("--max-steps", type=int, default=MAX_STEPS)
    ap.add_argument("--no-imu", action="store_true",
                    help="자이로를 안 연다. 회전이 미측정 시간모델 개루프가 된다")
    ap.add_argument("--record-events", action="store_true",
                    help="운행 기록을 %s/시각/ 폴더에 남긴다 (실행마다 새 폴더, "
                         "종류별 .jsonl, ts 로 병합 가능). 분석은 tools/analyze_run.py. "
                         "dry-run 에선 카메라 쪽(measure/decision/result)만 남는다" % EVENT_LOG_DIR)
    asyncio.run(main_async(ap.parse_args()))


if __name__ == "__main__":
    main()
