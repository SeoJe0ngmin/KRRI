"""회전 명령 — rotate_to() 하나. IMU 를 보며 목표각에서 멈추는 폐루프.

시간 모델(rot_sec_from_deg)은 실행에 안 쓴다 — 표시·워치독·IMU 없을 때 폴백 전용.
설계 이유는 ../../../../docs/CODE_NOTES.md 의 rot_control 절.
"""
import asyncio
from dataclasses import dataclass
from typing import Optional

from config import control as C

ROT_T0 = 0.85            # 명령→실회전 지연 [s]. 2026-09-07 can_pulse 실측 (IMU 가 0.8~0.9s 뒤 움직임)
ROT_DEG_PER_SEC = 8.0    # 강도 20 에서 1.5s 펄스 끝에 7.3도/s, 아직 가속 중 — 보수적으로 8. 표시·워치독 전용


ROT_MIN_SEC = 1.0
ROT_MAX_SEC = 30.0


@dataclass(frozen=True)
class RotTimeParams:
    t0: float = ROT_T0
    deg_per_sec: float = ROT_DEG_PER_SEC
    min_sec: float = ROT_MIN_SEC
    max_sec: float = ROT_MAX_SEC


def rot_sec_from_deg(deg, *, t0: Optional[float] = None,
                     deg_per_sec: Optional[float] = None,
                     min_sec: Optional[float] = None,
                     max_sec: Optional[float] = None) -> float:
    """돌 각도[도] -> 예상 시간[s]. 실행에는 안 쓴다."""
    params = RotTimeParams(
        t0=ROT_T0 if t0 is None else t0,
        deg_per_sec=ROT_DEG_PER_SEC if deg_per_sec is None else deg_per_sec,
        min_sec=ROT_MIN_SEC if min_sec is None else min_sec,
        max_sec=ROT_MAX_SEC if max_sec is None else max_sec,
    )
    angle = abs(float(deg))
    if angle == 0.0:
        return 0.0
    seconds = params.t0 + angle / params.deg_per_sec
    return max(params.min_sec, min(params.max_sec, seconds))


def rot_timeout_sec(deg) -> float:
    """회전 워치독 상한 [s]. 각도에 비례해서 늘어난다 — 각속도가 가정보다"""
    return max(ROT_MAX_SEC, rot_sec_from_deg(deg) * C.ROT_WATCHDOG_GAIN)


def _settle_threshold_dps(yaw):
    """"회전이 멎었다" 판정 문턱 [도/s]. 보정 때 잰 잡음의 배수로 잡되, 위아래로 막는다."""
    noise = getattr(yaw, "noise_dps", None) if yaw is not None else None
    if not noise:
        return C.ROT_SETTLE_RATE_FLOOR
    return min(C.ROT_SETTLE_RATE_CEIL, max(C.ROT_SETTLE_RATE_FLOOR, noise * C.ROT_SETTLE_RATE_K))


async def rotate_to(controller, yaw, deg, log=None, timeout=None, record=None):
    """제자리로 deg 만큼 돈다. +가 반시계. **폐루프** — IMU 를 보다가 목표각에서 멈춘다."""
    deg = float(deg)
    if abs(deg) < 1e-9:
        result = {"target": 0.0, "turned": 0.0, "overshoot": 0.0,
                  "ok": True, "reason": "", "elapsed_sec": 0.0}
        if record:
            record(result)
        return result

    if yaw is None:
        seconds = rot_sec_from_deg(deg)
        if log:
            log("       -> rotate_to    %+7.1f도  (IMU 없음 — 시간모델 %.2fs 개루프)"
                % (deg, seconds))
        controller.current_movement = "rotate_ccw" if deg > 0 else "rotate_cw"
        try:
            if seconds:
                await asyncio.sleep(seconds)
        finally:
            controller.current_movement = "stop"
        await asyncio.sleep(C.SETTLE_SEC)
        result = {"target": deg, "turned": None, "overshoot": None,
                  "ok": True, "reason": "time-fallback", "elapsed_sec": seconds}
        if record:
            record(result)
        return result

    if not yaw.alive:
        if log:
            log("       !! rotate_to 거부 — 자이로가 %.1fs 째 안 온다. 안 돈다" % yaw.age_sec)
        controller.current_movement = "stop"
        result = {"target": deg, "turned": 0.0, "overshoot": None,
                  "ok": False, "reason": "imu-stale", "elapsed_sec": 0.0}
        if record:
            record(result)
        return result

    loop = asyncio.get_event_loop()
    timeout = rot_timeout_sec(deg) if timeout is None else float(timeout)
    start_angle = yaw.angle_deg
    gaps_before = yaw.stats().get("gaps", 0)
    direction = 1.0 if deg > 0 else -1.0
    goal = abs(deg) - min(C.ROT_LEAD_DEG, abs(deg) / 2.0)
    if log:
        log("       -> rotate_to    %+7.1f도  (IMU 폐루프, 워치독 %.0fs)" % (deg, timeout))

    ok, reason = True, ""
    t_start = loop.time()
    controller.current_movement = "rotate_ccw" if deg > 0 else "rotate_cw"
    try:
        while True:
            await asyncio.sleep(C.ROT_POLL_SEC)
            progress = direction * (yaw.angle_deg - start_angle)
            if progress >= goal:
                break
            if progress <= -C.ROT_WRONG_WAY_DEG:
                ok, reason = False, "wrong-way"
                break
            if not yaw.alive:
                ok, reason = False, "imu-stale"
                break
            if loop.time() - t_start > timeout:
                ok, reason = False, "timeout"
                break
    finally:
        elapsed_sec = loop.time() - t_start
        controller.current_movement = "stop"

    threshold = _settle_threshold_dps(yaw)
    t_settle = loop.time()
    while loop.time() - t_settle < C.ROT_SETTLE_MAX_SEC:
        await asyncio.sleep(C.ROT_SETTLE_POLL_SEC)
        if (loop.time() - t_settle >= C.ROT_SETTLE_MIN_SEC
                and abs(yaw.rate_dps) < threshold):
            break

    turned = yaw.angle_deg - start_angle
    overshoot = direction * turned - abs(deg)
    gaps = yaw.stats().get("gaps", 0) - gaps_before
    if ok and gaps > 0:
        ok, reason = False, "gyro-gaps"

    if log:
        if ok:
            log("          회전 끝: 목표 %+.1f도 -> 실제 %+.1f도 (오버슈트 %+.1f도, %.2fs)"
                % (deg, turned, overshoot, elapsed_sec))
        elif reason == "wrong-way":
            log("          !! 반대로 돌았다(%+.1f도) — CAN 회전 부호가 뒤집혔다. "
                "CanDriver.rotate_by 안의 movement 매핑 두 곳만 맞바꿀 것" % turned)
        elif reason == "gyro-gaps":
            log("          !! 회전 중 자이로 %d구간 유실 — 정지하고 재측정" % gaps)
        else:
            log("          !! 회전 중단(%s): 목표 %+.1f도 중 %+.1f도에서 정지"
                % (reason, deg, turned))

    result = {"target": deg, "turned": turned, "overshoot": overshoot,
              "ok": ok, "reason": reason, "elapsed_sec": elapsed_sec}
    if record:
        record(result)
    return result
