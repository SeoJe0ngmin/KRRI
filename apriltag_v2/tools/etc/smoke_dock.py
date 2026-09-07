import asyncio
import os
import shutil
import sys
import tempfile
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tools', 'check'))
from config.main import COLOR_SIZE
from src.models import TagPipeline, intrinsics_from_ref
from src.models.control.control_from_pose import DryRunDriver, dock_live
from src.utils.event_log import read_events
from verify import synth_source
TAG_SIZE = 0.2
TAG_ID = 0

def main():
    w, h = COLOR_SIZE
    intr = intrinsics_from_ref((h, w))
    frames, _ = synth_source(intr, TAG_SIZE, tilt_deg=6.0, distance=3.0, lateral=0.15, n=120, tag_id=TAG_ID)
    pipe = TagPipeline(frames=frames, intrinsics=intr, tag_size=TAG_SIZE)
    record_dir = os.path.join(tempfile.mkdtemp(prefix='smoke_dock_'), 'run')
    lines = []
    try:
        asyncio.run(dock_live(pipe, DryRunDriver(realtime=False), tag_id=TAG_ID, max_steps=3, log=lines.append, n_frames=5, record_dir=record_dir))
    except Exception as exc:
        print('실패: dock_live 가 %s 로 죽었다' % type(exc).__name__)
        print('   %s' % exc)
        return 1
    events = read_events(record_dir) if os.path.isdir(record_dir) else []
    kinds = {}
    for e in events:
        kinds[e['kind']] = kinds.get(e['kind'], 0) + 1
    shutil.rmtree(os.path.dirname(record_dir), ignore_errors=True)
    print('파이프라인 순회 + dock_live 완주 OK')
    for line in lines[:6]:
        print('   %s' % line)
    print('기록: %s' % (', '.join(('%s %d줄' % (k, n) for k, n in sorted(kinds.items()))) or '(없음)'))
    if not any((e['kind'] == 'measure' for e in events)):
        print('실패: measure 가 한 줄도 안 남았다 — 검출/기록 사슬이 끊겼다')
        return 1
    print('판정: 계약 통과')
    return 0
if __name__ == '__main__':
    sys.exit(main())
