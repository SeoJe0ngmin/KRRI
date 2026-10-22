"""동역학 — **명령을 주면 어디서 멈추나** 하나만 답하는 층. (plan 2-1~2-7, contracts_B §3)

    predict.Dynamics       캘리브(또는 9/7 가정값)로 정지거리·정지각·명령시간·최소 증분
    predict.Prediction     그 답 하나의 꼴 (value·sigma·q95·source·provisional)
    predict.CommandBuffer  언제 무슨 명령을 보냈나 → 임의 시각의 예상 v̂·ω̂
                           (추정기가 prior·Q 스케줄 창으로 쓴다, plan 2-2)

경계
    · CAN 을 직접 건드리지 않는다 — 송신은 `control.can_tx.SafeCanTx` 하나뿐이다.
    · 추정(estimate)·상태기계(control)를 import 하지 않는다. 의존은 한 방향:
      utils → estimate → **dynamics** → control.dock_fsm → tools/dock.py.
    · 여기 숫자는 사람이 정하는 값이 아니라 **측정 산출물**이다. 그래서 config 가
      아니라 `dynamics_calib.json` 에서 읽는다(CLAUDE.md 상수 최소화).
"""
from .predict import (
    Dynamics, Prediction, CommandBuffer, CommandRecord, CommandState,
    ASSUMED, LEVELS, SIDES, cell_name, side_of, movement_for,
)

__all__ = ["Dynamics", "Prediction", "CommandBuffer", "CommandRecord", "CommandState",
           "ASSUMED", "LEVELS", "SIDES", "cell_name", "side_of", "movement_for"]
