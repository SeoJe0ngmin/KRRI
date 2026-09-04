"""도킹 중 벌어지는 일을 JSON Lines 로 기록하는 곳.

record_dir(실행마다 새 폴더) 아래에 **종류별로 파일을 나눠** 쌓는다 —
종류마다 열(스키마)이 달라서 한 파일에 섞으면 표로 펴 보기 불편하다.

    measure.jsonl    lateral/forward/heading 등 측정값  (dock/dock_live 가 기록)
                     30프레임을 중앙값 하나로 누른 뒤라 사이클당 한 줄이다
    rotation.jsonl   목표각/실제각/오버슈트/걸린시간     (rot_control.rotate_to 가 기록)
    drive.jsonl      전진/후진 방향/명령시간             (CanDriver._hold 가 기록)

모든 줄에 ts(초, epoch)가 붙으므로 시간순으로 다시 합쳐 볼 수 있다 —
read_events() 가 그 일을 한다. run.py 의 --record-events 로 켠다.
"""
import glob
import json
import os
import time


def record_event(record_dir, kind, **fields):
    """record_dir/<kind>.jsonl 에 한 줄 추가. record_dir 가 None 이면 아무 일도 안 한다.

    실패해도 예외를 던지지 않는다 — 기록 때문에 도킹이 멈추면 안 된다.
    """
    if not record_dir:
        return
    entry = dict(fields, ts=time.time())
    try:
        os.makedirs(record_dir, exist_ok=True)
        path = os.path.join(record_dir, "%s.jsonl" % kind)
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")
    except OSError:
        pass


def read_events(record_dir, kind=None):
    """기록을 읽는다. kind 를 주면 그 파일만, 안 주면 전부 시간순으로 합쳐서.

    분석할 때 쓴다:
        rotations = read_events(d, "rotation")          # 회전만 표로
        timeline  = read_events(d)                       # 사건 전체를 시간순으로
    합칠 때는 어느 파일에서 왔는지 "kind" 를 붙여 준다.
    """
    events = []
    pattern = "%s.jsonl" % (kind or "*")
    for path in glob.glob(os.path.join(record_dir, pattern)):
        name = os.path.splitext(os.path.basename(path))[0]
        with open(path, encoding="utf-8") as f:
            for line in f:
                if line.strip():
                    events.append(dict(json.loads(line), kind=name))
    events.sort(key=lambda e: e.get("ts", 0.0))
    return events
