"""도킹 자동 실행 — 시작만 키보드, 그 뒤는 알아서 간다.

    python tools/run.py --dry-run      CAN 없이 순서만 본다 (카메라는 씀)
    python tools/run.py                실제 주행. canlib + keyboard 필요

키보드는 **시작과 비상정지에만** 쓴다. 주행 방향은 카메라가 정한다.
    SPACE  시작
    ESC    비상정지 후 종료

control_forklift_v2.py 는 한 줄도 안 고친다. 그쪽 TX 루프(movement 10ms /
control 5ms / heartbeat 200ms)를 그대로 띄워 두고, 우리는 current_movement
변수만 바꾼다 — 키보드 루프가 하던 일과 같다. 대신 key_monitor_loop 은 안 띄운다.
"""
import argparse
import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.config import TAG_ID, TAG_SIZE_M                                # noqa: E402
from src.models import TagPipeline                                       # noqa: E402
from src.models.control.control_from_pose import (CanDriver, DryRunDriver,  # noqa: E402
                                                  dock)


async def _wait_start():
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


async def _watch_stop(driver, stop_evt):
    """ESC 를 누르면 즉시 정지."""
    try:
        import keyboard
    except ImportError:
        return
    while not stop_evt.is_set():
        if keyboard.is_pressed("esc"):
            await driver.stop()
            stop_evt.set()
            print("\n  !! ESC — 비상정지")
            return
        await asyncio.sleep(0.02)


async def main_async(args):
    pipe = TagPipeline.from_realsense(args.tag_size, label="run")
    print("  카메라 열림. 태그 %d, 크기 %.3fm" % (args.tag_id, args.tag_size))

    ctrl = None
    tasks = []
    if args.dry_run:
        driver = DryRunDriver(realtime=args.realtime)
        print("  DRY-RUN — CAN 으로 아무것도 안 보낸다")
    else:
        from src.models.control.control_forklift_v2 import DirectFrameForkliftController
        ctrl = DirectFrameForkliftController()
        if not ctrl.connect_can():
            raise SystemExit("CAN 연결 실패")
        ctrl.is_running = True
        # 키보드 루프는 안 띄운다. 나머지 TX 루프만 띄운다.
        tasks = [asyncio.create_task(ctrl.control_tx_loop()),
                 asyncio.create_task(ctrl.movement_tx_loop()),
                 asyncio.create_task(ctrl.heartbeat_loop())]
        driver = CanDriver(ctrl, log=print)
        await asyncio.sleep(0.5)          # 하트비트가 자리잡을 시간
        print("  CAN 연결됨. TX 루프 3개 가동")

    stop_evt = asyncio.Event()
    try:
        await _wait_start()
        watcher = asyncio.create_task(_watch_stop(driver, stop_evt))
        print("  시작\n")
        await dock(pipe, driver, tag_id=args.tag_id, max_steps=args.max_steps)
        stop_evt.set()
        watcher.cancel()
    except KeyboardInterrupt:
        print("\n  중단")
    finally:
        await driver.stop()
        for t in tasks:
            t.cancel()
        if ctrl is not None:
            ctrl.is_running = False
            ctrl.disconnect_can()
        pipe.close()
        print("  정리 완료")


def main():
    ap = argparse.ArgumentParser(description="AprilTag 도킹 자동 실행")
    ap.add_argument("--dry-run", action="store_true",
                    help="CAN 없이 순서만 본다. 카메라는 실제로 쓴다")
    ap.add_argument("--realtime", action="store_true",
                    help="--dry-run 에서 명령 시간만큼 실제로 기다린다")
    ap.add_argument("--tag-id", type=int, default=TAG_ID)
    ap.add_argument("--tag-size", type=float, default=TAG_SIZE_M,
                    help="태그 한 변 [m], 검은 테두리 바깥까지")
    ap.add_argument("--max-steps", type=int, default=30)
    asyncio.run(main_async(ap.parse_args()))


if __name__ == "__main__":
    main()
