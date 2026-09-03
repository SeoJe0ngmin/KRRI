"""회전 명령 — 목표각까지 도는 함수 하나.

직진(fwd_time_model.py)은 속도를 모르니 "시간"으로 명령한다. 회전은 IMU 가
있어서 다르게 짤 수 있다 — 목표각에 닿을 때까지 보면서 돌리면 되므로,
각속도를 몰라도 정확하다. rotate_to() 가 그 전부다: 각도를 주면 알아서
돌고 멈춘다.

    result = await rotate_to(controller, yaw, 30.0)   # 30도 반시계
    result["ok"]       성공했나
    result["turned"]   실제로 돈 각도 [도]
    result["overshoot"]  목표를 얼마나 지나쳤나 [도]

rot_sec_from_deg() 는 실행에 안 쓴다. 화면 표시·워치독 상한·IMU 없을 때
폴백, 이 셋에만 쓰는 "예상 시간"이다. ROT_T0 는 미측정이라 0 — 회전 명령을
주고 실제로 움직이기 시작하는 시점까지의 지연을 ms 단위로 재서 넣을 것.
"""
import asyncio
from dataclasses import dataclass
from typing import Optional

from config.control import (ROT_LEAD_DEG, ROT_POLL_SEC, ROT_SETTLE_MAX_SEC,
                            ROT_SETTLE_MIN_SEC, ROT_SETTLE_POLL_SEC,
                            ROT_SETTLE_RATE_CEIL, ROT_SETTLE_RATE_FLOOR,
                            ROT_SETTLE_RATE_K, ROT_WATCHDOG_GAIN,
                            ROT_WRONG_WAY_DEG, SETTLE_SEC)

ROT_T0 = 0.0             # s. 명령 후 실회전 시작까지 지연. 미측정
ROT_DEG_PER_SEC = 15.0   # deg/s. 미측정 가정값
                         # 이 둘은 정지 판단(rotate_to)엔 안 쓴다. 다만 IMU 없을 때
                         # 개루프 폴백엔 그대로 실행 시간이 되므로, 여기서 "안전하게
                         # 크게" 키우면 그 폴백이 실제로 더 오래/많이 돈다 — 워치독과
                         # 반대 방향이라 여기는 안 건드린다. 워치독 여유는 아래 둘로.
ROT_MIN_SEC = 1.0
ROT_MAX_SEC = 30.0       # 워치독 하한 [s]. 최대 회전각은 90도(Set2 가 작은 쪽만 고름).
                         # 미실측이라 실제 속도가 가정의 1/5(3도/s)까지 느려도
                         # 90/3=30s 안에는 들어오게 넉넉히 잡았다. 실측 후 낮출 것.


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
    """회전 워치독 상한 [s]. 각도에 비례해서 늘어난다 — 각속도가 가정보다
    느려도(1/ROT_WATCHDOG_GAIN 배까지) 워치독에 안 걸린다."""
    return max(ROT_MAX_SEC, rot_sec_from_deg(deg) * ROT_WATCHDOG_GAIN)


def _settle_threshold_dps(yaw):
    """"회전이 멎었다" 판정 문턱 [도/s]. 보정 때 잰 잡음의 배수로 잡되, 위아래로 막는다.

    하한(FLOOR) 없이 노이즈가 아주 작으면 문턱도 너무 빡빡해져 "멎었다"가 영영
    안 나온다. 상한(CEIL) 없이 실제 현장 진동(엔진·유압)으로 noise_dps 가
    조용한 벤치 보정보다 훨씬 크게 나오면, 그만큼 문턱도 같이 커져서 아직
    도는 중인데 "멎었다"고 오판할 수 있다 — 그러면 그때 잰 overshoot 가
    과소평가돼서 ROT_LEAD_DEG 를 잘못 추정하게 된다. 아직 실차로 못 재봤으니
    상한을 걸어 둔다.
    """
    noise = getattr(yaw, "noise_dps", None) if yaw is not None else None
    if not noise:
        return ROT_SETTLE_RATE_FLOOR
    return min(ROT_SETTLE_RATE_CEIL, max(ROT_SETTLE_RATE_FLOOR, noise * ROT_SETTLE_RATE_K))


async def rotate_to(controller, yaw, deg, log=None, timeout=None, record=None):
    """제자리로 deg 만큼 돈다. +가 반시계. **폐루프** — IMU 를 보다가 목표각에서 멈춘다.

    controller 는 current_movement 를 "rotate_ccw"/"rotate_cw"/"stop" 로
    바꿀 수 있는 객체(control_forklift_v2 의 컨트롤러). yaw 는 GyroYaw —
    None 이면 시간 모델로 개루프 폴백한다.

    지키는 안전장치: 반대로 ROT_WRONG_WAY_DEG 넘게 돌면 즉시 정지(CAN 부호
    뒤집힘), 자이로가 끊기면 즉시 정지, 워치독 시간 초과면 정지, 회전 중
    자이로 샘플이 유실됐으면(gaps) 각도 과소집계라 실패로 강등한다.
    어느 경우든 멈추기만 하면 호출하는 쪽이 다시 계획한다.

    record 는 매 회전 결과를 넘겨주는 콜백(선택) — CanDriver 가 여기에 파일
    기록 함수를 꽂아서 실측 이력을 남긴다(work_dirs/rotations/log.jsonl).
    elapsed_sec 은 명령부터 목표각 도달까지 걸린 시간 — 나중에 ROT_T0 /
    ROT_DEG_PER_SEC 를 (각도, elapsed_sec) 로 실측 적합할 때 그대로 쓸 데이터다.

    돌려주는 것: {"target", "turned", "overshoot", "ok", "reason", "elapsed_sec"}
    """
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
        await asyncio.sleep(SETTLE_SEC)
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
    start_angle = yaw.angle_deg               # 절대각이 아니라 여기서부터 잰다
    gaps_before = yaw.stats().get("gaps", 0)
    direction = 1.0 if deg > 0 else -1.0
    goal = abs(deg) - min(ROT_LEAD_DEG, abs(deg) / 2.0)   # 관성만큼 미리 끊는다
    if log:
        log("       -> rotate_to    %+7.1f도  (IMU 폐루프, 워치독 %.0fs)" % (deg, timeout))

    ok, reason = True, ""
    t_start = loop.time()
    controller.current_movement = "rotate_ccw" if deg > 0 else "rotate_cw"
    try:
        while True:
            await asyncio.sleep(ROT_POLL_SEC)
            progress = direction * (yaw.angle_deg - start_angle)
            if progress >= goal:
                break
            if progress <= -ROT_WRONG_WAY_DEG:
                ok, reason = False, "wrong-way"
                break
            if not yaw.alive:
                ok, reason = False, "imu-stale"
                break
            if loop.time() - t_start > timeout:
                ok, reason = False, "timeout"
                break
    finally:
        elapsed_sec = loop.time() - t_start    # 명령 -> 목표 도달까지 (정착 대기 전)
        controller.current_movement = "stop"

    threshold = _settle_threshold_dps(yaw)
    t_settle = loop.time()
    while loop.time() - t_settle < ROT_SETTLE_MAX_SEC:
        await asyncio.sleep(ROT_SETTLE_POLL_SEC)
        if (loop.time() - t_settle >= ROT_SETTLE_MIN_SEC
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
