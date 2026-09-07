import asyncio
from dataclasses import dataclass
from typing import Optional
from config import control as C
ROT_T0 = 0.85
ROT_DEG_PER_SEC = 8.0
ROT_MIN_SEC = 1.0
ROT_MAX_SEC = 30.0

@dataclass(frozen=True)
class RotTimeParams:
    t0: float = ROT_T0
    deg_per_sec: float = ROT_DEG_PER_SEC
    min_sec: float = ROT_MIN_SEC
    max_sec: float = ROT_MAX_SEC

def rot_sec_from_deg(deg, *, t0: Optional[float]=None, deg_per_sec: Optional[float]=None, min_sec: Optional[float]=None, max_sec: Optional[float]=None) -> float:
    params = RotTimeParams(t0=ROT_T0 if t0 is None else t0, deg_per_sec=ROT_DEG_PER_SEC if deg_per_sec is None else deg_per_sec, min_sec=ROT_MIN_SEC if min_sec is None else min_sec, max_sec=ROT_MAX_SEC if max_sec is None else max_sec)
    angle = abs(float(deg))
    if angle == 0.0:
        return 0.0
    seconds = params.t0 + angle / params.deg_per_sec
    return max(params.min_sec, min(params.max_sec, seconds))

def rot_timeout_sec(deg) -> float:
    return max(ROT_MAX_SEC, rot_sec_from_deg(deg) * C.ROT_WATCHDOG_GAIN)

def _settle_threshold_dps(yaw):
    noise = getattr(yaw, 'noise_dps', None) if yaw is not None else None
    if not noise:
        return C.ROT_SETTLE_RATE_FLOOR
    return min(C.ROT_SETTLE_RATE_CEIL, max(C.ROT_SETTLE_RATE_FLOOR, noise * C.ROT_SETTLE_RATE_K))

async def rotate_to(controller, yaw, deg, log=None, timeout=None, record=None):
    deg = float(deg)
    if abs(deg) < 1e-09:
        result = {'target': 0.0, 'turned': 0.0, 'overshoot': 0.0, 'ok': True, 'reason': '', 'elapsed_sec': 0.0}
        if record:
            record(result)
        return result
    if yaw is None:
        seconds = rot_sec_from_deg(deg)
        if log:
            log('       -> rotate_to    %+7.1f도  (IMU 없음 — 시간모델 %.2fs 개루프)' % (deg, seconds))
        controller.current_movement = 'rotate_ccw' if deg > 0 else 'rotate_cw'
        try:
            if seconds:
                await asyncio.sleep(seconds)
        finally:
            controller.current_movement = 'stop'
        await asyncio.sleep(C.SETTLE_SEC)
        result = {'target': deg, 'turned': None, 'overshoot': None, 'ok': True, 'reason': 'time-fallback', 'elapsed_sec': seconds}
        if record:
            record(result)
        return result
    if not yaw.alive:
        if log:
            log('       !! rotate_to 거부 — 자이로가 %.1fs 째 안 온다. 안 돈다' % yaw.age_sec)
        controller.current_movement = 'stop'
        result = {'target': deg, 'turned': 0.0, 'overshoot': None, 'ok': False, 'reason': 'imu-stale', 'elapsed_sec': 0.0}
        if record:
            record(result)
        return result
    loop = asyncio.get_event_loop()
    timeout = rot_timeout_sec(deg) if timeout is None else float(timeout)
    start_angle = yaw.angle_deg
    gaps_before = yaw.stats().get('gaps', 0)
    direction = 1.0 if deg > 0 else -1.0
    goal = abs(deg) - min(C.ROT_LEAD_DEG, abs(deg) / 2.0)
    if log:
        log('       -> rotate_to    %+7.1f도  (IMU 폐루프, 워치독 %.0fs)' % (deg, timeout))
    ok, reason = (True, '')
    t_start = loop.time()
    controller.current_movement = 'rotate_ccw' if deg > 0 else 'rotate_cw'
    try:
        while True:
            await asyncio.sleep(C.ROT_POLL_SEC)
            progress = direction * (yaw.angle_deg - start_angle)
            if progress >= goal:
                break
            if progress <= -C.ROT_WRONG_WAY_DEG:
                ok, reason = (False, 'wrong-way')
                break
            if not yaw.alive:
                ok, reason = (False, 'imu-stale')
                break
            if loop.time() - t_start > timeout:
                ok, reason = (False, 'timeout')
                break
    finally:
        elapsed_sec = loop.time() - t_start
        controller.current_movement = 'stop'
    threshold = _settle_threshold_dps(yaw)
    t_settle = loop.time()
    while loop.time() - t_settle < C.ROT_SETTLE_MAX_SEC:
        await asyncio.sleep(C.ROT_SETTLE_POLL_SEC)
        if loop.time() - t_settle >= C.ROT_SETTLE_MIN_SEC and abs(yaw.rate_dps) < threshold:
            break
    turned = yaw.angle_deg - start_angle
    overshoot = direction * turned - abs(deg)
    gaps = yaw.stats().get('gaps', 0) - gaps_before
    if ok and gaps > 0:
        ok, reason = (False, 'gyro-gaps')
    if log:
        if ok:
            log('          회전 끝: 목표 %+.1f도 -> 실제 %+.1f도 (오버슈트 %+.1f도, %.2fs)' % (deg, turned, overshoot, elapsed_sec))
        elif reason == 'wrong-way':
            log('          !! 반대로 돌았다(%+.1f도) — CAN 회전 부호가 뒤집혔다. CanDriver.rotate_by 안의 movement 매핑 두 곳만 맞바꿀 것' % turned)
        elif reason == 'gyro-gaps':
            log('          !! 회전 중 자이로 %d구간 유실 — 정지하고 재측정' % gaps)
        else:
            log('          !! 회전 중단(%s): 목표 %+.1f도 중 %+.1f도에서 정지' % (reason, deg, turned))
    result = {'target': deg, 'turned': turned, 'overshoot': overshoot, 'ok': ok, 'reason': reason, 'elapsed_sec': elapsed_sec}
    if record:
        record(result)
    return result
