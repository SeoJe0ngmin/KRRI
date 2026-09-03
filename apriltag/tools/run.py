"""도킹 자동 실행 — 시작만 키보드, 그 뒤는 카메라가 정한다.

    python tools/run.py --dry-run --show    화면 보며 순서 확인 (CAN 안 씀)
    python tools/run.py --show              실제 주행 + 화면
    python tools/run.py                     실제 주행 (화면 없음)

**화면과 제어가 한 프로세스여야 한다.** 카메라는 한 프로세스만 열 수 있어서,
live_pose.py 를 따로 띄우면 이쪽이 카메라를 못 연다. 그래서 --show 로 여기서 그린다.

키보드는 시작과 비상정지, 그리고 카메라 노출 조절에만 쓴다. 주행 방향은 카메라가 정한다.
    SPACE  시작        ESC  비상정지 후 종료
    e      자동노출 켜기/끄기      [ ]  노출 -/+       - =  게인 -/+

control_forklift_v2.py 는 한 줄도 안 고친다. 그쪽 TX 루프(movement 10ms /
control 5ms / heartbeat 200ms)를 그대로 띄워 두고, current_movement 만 바꾼다.
"""
import argparse
import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config.system import TAG_ID, TAG_SIZE_M                                # noqa: E402
from src.models import TagPipeline                                       # noqa: E402
from src.models.control.control_from_pose import (CanDriver,             # noqa: E402
                                                  DryRunDriver, dock_live)
from config.system import MAX_STEPS                                  # noqa: E402
from src.models.detection.image import intrinsics_from_ref        # noqa: E402                        # noqa: E402
from src.utils.imu_yaw import GyroYaw                                    # noqa: E402

PHASE_KO = {"measure": "측정 중", "command": "명령 실행 중", "done": "끝"}


def make_view(args):
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
        elif info["phase"] == "command":
            head.append(("kv", info.get("action", ""), "남은 %.1fs" % info["left"], "warn"))
            head.append(("kv", "", info.get("why", "")[:44], "dim"))
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


async def _wait_start(show):
    """SPACE 를 기다린다. keyboard 가 없으면 엔터로 대신한다."""
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


async def main_async(args):
    pipe = TagPipeline.from_realsense(args.tag_size, label="run")
    print("  카메라 열림. 태그 %d, 크기 %.3fm" % (args.tag_id, args.tag_size))

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
        driver = CanDriver(ctrl, yaw=yaw, log=print)
        await asyncio.sleep(0.5)
        print("  CAN 연결됨. TX 루프 3개 가동")

    on_frame, view = make_view(args)
    try:
        await _wait_start(args.show)
        print("  시작\n")
        await dock_live(pipe, driver, tag_id=args.tag_id,
                        max_steps=args.max_steps, on_frame=on_frame)
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
    asyncio.run(main_async(ap.parse_args()))


if __name__ == "__main__":
    main()
