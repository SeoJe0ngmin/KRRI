"""도킹 루프 무하드웨어 스모크 — **진짜 TagPipeline** 으로 dock_live 를 돌린다.

    python tools/smoke_dock.py

카메라도 CAN 도 없이, 합성 태그 영상을 진짜 파이프라인에 넣어 dock_live 를
몇 사이클 돌린다. 목적은 도킹이 잘 되나가 아니라 **부품 사이 계약이 맞나**다.

이게 왜 필요한가 — 예전 시뮬레이션은 파이프라인을 흉내 낸 가짜 객체를 썼고,
그 가짜가 진짜와 다른 계약(3-튜플 vs Result)을 갖고 있었다. 그래서
"TagPipeline 을 순회하면 무엇이 나오나"를 아무도 검사하지 않았고,
첫 프레임에서 죽는 버그가 실차 전날까지 살아남았다. 가짜를 아무리 정교하게
만들어도 그 계약은 못 잡는다 — 진짜 클래스를 통과시켜야만 잡힌다.

검사하는 계약:
    TagPipeline 순회 -> Result (index/timestamp/image/docking...)
    measure() 가 그 Result 리스트를 먹는다
    plan_step -> _execute -> driver 호출이 이어진다
    record_event 가 기록 폴더에 파일을 만든다
"""
import asyncio
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

from config.main import COLOR_SIZE                                 # noqa: E402
from src.models import TagPipeline, intrinsics_from_ref              # noqa: E402
from src.models.control.control_from_pose import (DryRunDriver,      # noqa: E402
                                                  dock_live)
from src.utils.event_log import read_events                          # noqa: E402
from verify import synth_source                                      # noqa: E402

TAG_SIZE = 0.20
TAG_ID = 0          # synth_source 가 그리는 태그 번호


def main():
    w, h = COLOR_SIZE
    intr = intrinsics_from_ref((h, w))
    frames, _ = synth_source(intr, TAG_SIZE, tilt_deg=6.0, distance=3.0,
                             lateral=0.15, n=120, tag_id=TAG_ID)
    pipe = TagPipeline(frames=frames, intrinsics=intr, tag_size=TAG_SIZE)

    record_dir = os.path.join(tempfile.mkdtemp(prefix="smoke_dock_"), "run")
    lines = []
    try:
        asyncio.run(dock_live(pipe, DryRunDriver(realtime=False), tag_id=TAG_ID,
                              max_steps=3, log=lines.append, n_frames=5,
                              record_dir=record_dir))
    except Exception as exc:
        print("실패: dock_live 가 %s 로 죽었다" % type(exc).__name__)
        print("   %s" % exc)
        return 1

    events = read_events(record_dir) if os.path.isdir(record_dir) else []
    kinds = {}
    for e in events:
        kinds[e["kind"]] = kinds.get(e["kind"], 0) + 1
    shutil.rmtree(os.path.dirname(record_dir), ignore_errors=True)

    print("파이프라인 순회 + dock_live 완주 OK")
    for line in lines[:6]:
        print("   %s" % line)
    print("기록: %s" % (", ".join("%s %d줄" % (k, n) for k, n in sorted(kinds.items()))
                        or "(없음)"))

    if not any(e["kind"] == "measure" for e in events):
        print("실패: measure 가 한 줄도 안 남았다 — 검출/기록 사슬이 끊겼다")
        return 1
    print("판정: 계약 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
