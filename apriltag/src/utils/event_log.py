"""도킹 중 벌어지는 일을 JSON Lines 로 기록하는 곳. 함수 하나뿐이다.

회전·직진·검출이 전부 같은 파일에 시간순으로 쌓인다. 한 줄 = 사건 하나,
"kind" 로 구분한다.

    kind="rotation"   목표각/실제각/오버슈트/걸린시간   (rot_control.rotate_to 가 기록)
    kind="drive"      전진/후진 방향/명령시간            (CanDriver._hold 가 기록)
    kind="measure"    lateral/forward/heading 등 측정값  (dock/dock_live 가 기록)

한 파일에 모으는 이유: 회전 하나의 오버슈트를 그 앞뒤 measure 값과 같이
보면 실제로 몇 도/몇 mm 움직였는지 재구성할 수 있다. run.py 의
--record-events 로 켠다.
"""
import json
import os
import time


def record_event(path, kind, **fields):
    """한 줄 기록. path 가 None 이면 아무 일도 안 한다.

    실패해도 예외를 던지지 않는다 — 기록 때문에 도킹이 멈추면 안 된다.
    """
    if not path:
        return
    entry = dict(fields, kind=kind, ts=time.time())
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")
    except OSError:
        pass
