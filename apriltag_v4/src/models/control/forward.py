"""직진 한 걸음. 카메라를 보며 **미리** 끊는다 (plan 4-2).

회전과 다른 점 하나 — 직진엔 자이로 같은 콜백이 없다. 거리는 카메라에서만 오고,
카메라는 "조금 전"을 알려준다. 그래서 끊는 시점을 이렇게 잡는다:

    남은거리 <= 속도 x (검출지연 + 관성시간 + 판정주기/2) + 잔여보정

속도는 **창을 잡고 직선을 맞춰서** 낸다. 두 프레임 차로 내면 0.28 m/s 에서 한 프레임이
9 mm 인데 거리 잡음이 +-39 mm(9/21 실측, 3.7 m) 라 그 차가 잡음에 묻힌다. 창이 안 차면 배운 정속을 쓴다.
창이 찼다고 보는 기준은 상수가 아니라 `3 x 거리잡음 / 정속` — 그만큼은 움직여야 기울기가 잡음 위로 나온다.

두 가지가 있다:
  forward()        카메라를 보며 간다. 정상 걸음. 여기서 배운다
  forward_timed()  눈 감고 시간으로 간다. 사이드스텝 전용. 배우지 않는다 (2026-09-30 결정 ⑤③)

후진(movement="backward")도 같은 함수를 쓴다 — 단 learner 에 후진 속도가 있을 때만(없으면 no_model).
그때 look.remaining_m 은 **가는 방향으로** 남은 거리여야 한다(부르는 쪽이 맞춘다).

안전망 두 겹 — 회전보다 하나 적다(CAN 쪽 타이머는 회전 전용이다):
  1) 여기서 재는 워치독
  2) 데드맨 lease — 우리가 죽으면 CAN 스레드가 LEASE_S 안에 세운다

시각은 전부 clock.now()(monotonic). Look.t_capture 도 CameraClock.see() 로 바꾼 우리 시계 값이다.
"""
import time
from collections import deque
from dataclasses import dataclass, field

from config import control as C
from ...utils import clock

POLL_S = 0.005         # 측정을 확인하는 주기. 새 프레임이 없으면 그냥 지나간다
LEASE_S = 0.30         # 이 안에 명령이 안 갱신되면 CAN 스레드가 세운다.
                       # 광운대 COARSE_COMMAND_LEASE_SEC
ABORT_FACTOR = 2.0     # 예상의 이 배를 넘으면 무조건 끊는다. 중단 규칙이지 측정값이 아니다
FIT_WINDOW_S = 0.6     # 속도를 맞출 창. 0.29 m/s 면 174 mm 움직인다 (잡음 39 mm 보다 크다)
FIT_MIN_N = 5
FIT_SPAN_K = 3.0       # 창이 찼다 = 그동안 간 거리가 잡음의 이 배 (3σ). 2026-09-30 결정 ⑨
ONSET_K = 3.0          # 흔들림의 이 배를 넘게 움직이면 "출발했다"
ONSET_N = 3            # 연속 이만큼 넘어야 인정. 한 프레임만 보면 잡음이 출발로 둔갑한다
SETTLE_SPAN_S = 1.0    # 멎음은 **이 간격 전과** 비교해 본다. 옆 프레임끼리 보면 미끄러지는 중에 속는다.
                       # 광운대 정지지연 실측 0.352 s 를 넉넉히 덮는 값 (2026-09-30 결정)
BLIND_MAX_S = 1.0      # 태그를 이만큼 못 보면 선다. 0.3 m/s 면 30 cm — 그 안에선 더듬어도 된다
SETTLE_MIN_S = 0.3
SETTLE_QUIET_K = 2.0   # 멎음 판정 = 흔들림 x 이 값 (거리 변화 기준)


@dataclass
class Look:
    """지금 카메라가 말하는 것. 부르는 쪽(estimate/plan)이 채운다."""
    remaining_m: float = 0.0    # 서야 할 곳까지 남은 거리. 지나쳤으면 음수
    t_capture: float = 0.0      # **노출 시각** (clock.now 기준 — CameraClock.see 가 바꿔준 값). 지금이 아니다
    ok: bool = False            # 태그를 믿을 만하게 봤나
    sigma_m: float = 0.0        # 이 값의 흔들림


@dataclass
class LegResult:
    target_m: float = 0.0
    strength: int = 0
    travelled_m: float = 0.0
    travelled_at_stop_m: float = 0.0
    d_at_stop_cmd: float = 0.0     # 정지명령 때 남아 있던 거리
    d_final: float = 0.0
    speed_mps: float = 0.0
    detect_age_s: float = 0.0
    blind_s: float = 0.0           # 태그를 못 본 채 간 시간
    reason: str = ""
    t_cmd: float = 0.0
    t_onset: float = 0.0
    t_stop_cmd: float = 0.0
    t_settled: float = 0.0
    hold_s: float = 0.0            # 시간 직진일 때 명령을 쥔 시간
    track: list = field(default_factory=list)   # (t, 남은거리) — 분석용

    @property
    def done(self):
        return self.t_settled > 0.0

    @property
    def ok(self):
        """카메라를 보고 제대로 끝난 걸음. 배우는 것도 이것만 믿는다."""
        return self.done and self.reason == "predicted"

    @property
    def timed_ok(self):
        """시간 직진이 끝까지 간 것. 실제로 얼마나 갔는지는 모른다 — 배우지 않는다."""
        return self.done and self.reason == "timed"


class Speed:
    """창 안의 (시각, 남은거리) 에 직선을 맞춰 속도를 낸다. 최소제곱 기울기."""

    def __init__(self, window_s=FIT_WINDOW_S):
        self.window_s = window_s
        self.pts = deque()

    def add(self, t, d):
        self.pts.append((t, d))
        while self.pts and t - self.pts[0][0] > self.window_s:
            self.pts.popleft()

    def value(self, min_span_s):
        """전진 속도 [m/s]. 남은거리가 줄어드는 기울기의 부호를 뒤집은 것. 못 믿으면 None."""
        n = len(self.pts)
        if n < FIT_MIN_N or self.pts[-1][0] - self.pts[0][0] < min_span_s:
            return None
        mt = sum(p[0] for p in self.pts) / n
        md = sum(p[1] for p in self.pts) / n
        num = sum((t - mt) * (d - md) for t, d in self.pts)
        den = sum((t - mt) ** 2 for t, _ in self.pts)
        return -num / den if den > 0 else None


def fit_span_s(sigma_m, speed_mps, window_s=FIT_WINDOW_S):
    """기울기를 믿으려면 창이 이만큼은 차야 한다 [s] = 잡음 x FIT_SPAN_K / 정속.

    정속을 모르면 창 전체를 요구한다(더 보수적인 쪽). 결과가 창보다 길면 이 잡음으로는
    속도를 못 재는 것이고, 그때는 부르는 쪽이 배운 정속으로 간다.
    """
    if not speed_mps or speed_mps <= 0:
        return window_s
    return FIT_SPAN_K * max(0.0, float(sigma_m)) / float(speed_mps)


def min_step_m(learner, strength):
    """이보다 짧게는 못 간다 [m] = 정속 x 관성시간. 끊어도 그만큼은 더 간다."""
    v = learner.fwd_speed_mps(strength)
    e = learner.fwd_tau.get(str(int(strength)))
    tau = e.value if e else 0.0
    return v * tau if (v and tau) else None


def strength_of(movement):
    """동작 이름 -> CAN byte2. 배운 값을 담는 열쇠이기도 하다 (중립 127 에서 강도만큼)."""
    if movement == "backward":
        return 127 + C.BACKWARD_JOYSTICK_DEFLECTION
    return 127 - (C.FORWARD_SLOW_JOYSTICK_DEFLECTION if movement == "forward_slow"
                  else C.FORWARD_JOYSTICK_DEFLECTION)


def forward(driver, learner, look_fn, target_m, movement="forward",
            allow_blind=False, rec=None, log=print):
    """target_m 만큼 앞으로 간다. look_fn() 은 지금 측정(Look)을 돌려준다.

    allow_blind=True 는 **마지막 진입 전용** — 태그가 화면 밖으로 나가는 걸 알고 가는 구간이다.
    그때는 최종 거리를 못 재니 아무것도 배우지 않는다(reason='blind').
    """
    target_m = float(target_m)
    strength = strength_of(movement)
    res = LegResult(target_m=target_m, strength=strength)

    refused = _refuse(learner, target_m, strength, movement, look_fn)
    if refused:
        log("       !! 직진 거부(%s): %.3f m" % (refused, target_m))
        res.reason = refused
        _record(rec, res, movement, log)
        return res

    start = look_fn()
    hold_s = _hold_s(learner, target_m, strength)
    log("       -> 직진 %.3f m  (강도 %d, 안전망 %.1fs)" % (target_m, strength, hold_s))

    res.t_cmd = clock.now()
    driver.lease(LEASE_S)
    try:
        driver.set(movement, why="%s %.3f m" % (movement, target_m))
    except BaseException:
        driver.stop("직진 명령 실패")
        driver.clear_lease()
        raise

    try:
        _drive(driver, learner, look_fn, res, start, hold_s, allow_blind)
        _settle(driver, look_fn, res, start)
    finally:
        driver.stop("직진 끝 " + (res.reason or "?"))
        driver.clear_lease()

    learned = learner.forward(res)
    _record(rec, res, movement, log, learned)
    if res.ok:
        log("          끝: %.3f m 요청 -> %.3f m (관성 %.3f m, 속도 %.3f m/s%s)"
            % (target_m, res.travelled_m, res.travelled_m - res.travelled_at_stop_m,
               res.speed_mps, ", 눈감음 %.1fs" % res.blind_s if res.blind_s > 0.05 else ""))
    else:
        log("          !! 직진 실패(%s): %.3f m 중 %.3f m" % (res.reason, target_m, res.travelled_m))
    return res


def forward_timed(driver, learner, distance_m, movement="forward", rec=None, log=print):
    """눈 감고 **시간으로** distance_m 을 간다. 사이드스텝 전용 (2026-09-30 결정 ⑤③).

    유지시간 = 출발지연 + 거리/정속 — 광운대 delayed_linear 적합(거리 = v x 유지 − v x 죽은시간)을
    뒤집은 것. 거리 하드 상한(STEP_FORWARD_HARD_MAX_M)은 안 건다. 시간 상한(FWD_SAFETY_MAX_S)은 건다 —
    그보다 길면 거부(too_long). 실제로 얼마나 갔는지 못 재므로 **아무것도 배우지 않는다.**
    끝나면 C.SETTLE_S 만큼 기다려 관성이 죽은 뒤 돌아온다 (회전은 정지 후에만 — 결정 ⑦).
    """
    distance_m = float(distance_m)
    strength = strength_of(movement)
    res = LegResult(target_m=distance_m, strength=strength)
    v, startup = learner.fwd_speed_mps(strength), learner.fwd_startup_s(strength)

    refused = ("zero" if distance_m == 0 else "negative" if distance_m < 0
               else "no_model" if (not v or startup is None) else "")
    if not refused:
        res.hold_s = startup + distance_m / v
        if res.hold_s > C.FWD_SAFETY_MAX_S:
            refused = "too_long"
    if refused:
        log("       !! 시간 직진 거부(%s): %s %.3f m" % (refused, movement, distance_m))
        res.reason = refused
        _record(rec, res, movement, log, timed=True)
        return res

    log("       -> 시간 직진 %s %.3f m  (강도 %d, 유지 %.2fs = 출발 %.2f + %.3f/%.3f)"
        % (movement, distance_m, strength, res.hold_s, startup, distance_m, v))
    res.t_cmd = clock.now()
    driver.lease(LEASE_S)
    try:
        driver.set(movement, why="timed %s %.3f m" % (movement, distance_m))
    except BaseException:
        driver.stop("시간 직진 명령 실패")
        driver.clear_lease()
        raise

    reason = "timed"
    try:
        while clock.now() - res.t_cmd < res.hold_s:
            if driver.lease_expired():
                reason = "lease"
                break
            _tick(driver)
    finally:
        driver.stop("시간 직진 " + reason)          # 정지가 먼저 (plan 6-6 ④)
        driver.clear_lease()
    res.t_stop_cmd, res.reason = clock.now(), reason
    # 얼마나 갔는지는 모델 추정치뿐이다. 학습엔 안 쓴다(ok=False) — 보고용
    res.travelled_m = max(0.0, (res.t_stop_cmd - res.t_cmd - startup)) * v
    res.speed_mps = v
    time.sleep(C.SETTLE_S)                          # 카메라가 없으니 시간으로 가라앉힌다
    res.t_settled = clock.now()
    _record(rec, res, movement, log, timed=True)
    if res.timed_ok:
        log("          끝: %.3f m 을 %.2fs 유지 (모델상 %.3f m)"
            % (distance_m, res.hold_s, res.travelled_m))
    else:
        log("          !! 시간 직진 실패(%s): %.2fs 중 %.2fs"
            % (reason, res.hold_s, res.t_stop_cmd - res.t_cmd))
    return res


# ── 속 ──────────────────────────────────────────────────────────────
def _refuse(learner, target_m, strength, movement, look_fn):
    if target_m == 0:
        return "zero"
    if target_m < 0:
        return "negative"                   # 뒤로 가는 건 movement="backward" 다
    if target_m > C.STEP_FORWARD_HARD_MAX_M:
        return "too_big"
    if movement == "backward" and not learner.fwd_speed_mps(strength):
        return "no_model"                   # 후진 속도를 모르면 안 간다 (2026-09-30 결정)
    floor = min_step_m(learner, strength)
    if floor and target_m < floor:
        return "too_small"                  # 관성만으로 지나친다. 다른 방법을 찾아라 (plan 3-6)
    if not look_fn().ok:
        return "no_tag"                     # 안 보이는 채로 출발하지 않는다
    return ""


def _hold_s(learner, target_m, strength):
    v = learner.fwd_speed_mps(strength)
    startup = learner.fwd_startup_s(strength)
    if not v or startup is None:
        return C.FWD_SAFETY_MAX_S
    return min(C.FWD_SAFETY_MAX_S, (target_m / v + startup) * ABORT_FACTOR)


def _drive(driver, learner, look_fn, res, start, hold_s, allow_blind):
    """갈 만큼 가고 **미리** 끊는다.

    판단 기준은 늘 "마지막으로 **본** 거리" 와 "그게 얼마나 낡았나" 다. 낡은 걸 속도로 메워
    현재 위치를 지어내지 않는다 — 그 지어낸 값으로 tau 를 배우면 tau 가 지연을 두 번 세게 된다.
    대신 낡은 만큼을 lead 에 그대로 넣는다. 안 보일수록 lead 가 커져 **일찍** 선다 (plan 4-10).
    """
    speed, periods = Speed(), deque(maxlen=20)
    nominal = learner.fwd_speed_mps(res.strength)
    stop_at = start.remaining_m - res.target_m      # 남은거리가 여기까지 줄면 도착
    last_t = last_d = onset_t = None
    onset_n = 0
    span_min = fit_span_s(start.sigma_m, nominal, speed.window_s)

    while True:
        now = clock.now()
        if now - res.t_cmd > hold_s:
            return _cut(driver, res, now, "watchdog", last_d, 0.0, 0.0)
        if driver.lease_expired():
            return _cut(driver, res, now, "lease", last_d, 0.0, 0.0)

        look = look_fn()
        if look.ok and (last_t is None or look.t_capture > last_t):
            if last_t is not None:
                periods.append(look.t_capture - last_t)
            last_t, last_d = look.t_capture, look.remaining_m
            speed.add(look.t_capture, look.remaining_m)
            span_min = fit_span_s(look.sigma_m, nominal, speed.window_s)
            res.track.append((round(look.t_capture - res.t_cmd, 3), round(look.remaining_m, 4)))
            del res.track[:-200]                    # 끝쪽을 남긴다. 거기가 중요하다
            moved = start.remaining_m - look.remaining_m
            if not res.t_onset:
                if abs(moved) > ONSET_K * max(look.sigma_m, 1e-4):
                    onset_t = onset_t or look.t_capture
                    onset_n += 1
                    if onset_n >= ONSET_N:
                        res.t_onset = onset_t
                else:
                    onset_t, onset_n = None, 0

        if last_t is None:                          # 출발 후 아직 한 번도 못 봤다
            if now - res.t_cmd > BLIND_MAX_S and not allow_blind:
                return _cut(driver, res, now, "tag_lost", start.remaining_m, 0.0, now - res.t_cmd)
            _tick(driver)
            continue

        age = now - last_t
        res.blind_s = max(res.blind_s, age)
        v = speed.value(span_min) or nominal
        if v is None:
            # 속도를 아직 모른다. 미리 못 끊으니 지나칠 때까지 가면서 배운다
            if last_d <= stop_at:
                return _cut(driver, res, now, "predicted", last_d, 0.0, age)
            _tick(driver)
            continue

        if age > BLIND_MAX_S and not allow_blind:
            return _cut(driver, res, now, "tag_lost", last_d, v, age)
        if last_d - stop_at <= 0:                    # 이미 지났다 — 무조건 끊는다
            return _cut(driver, res, now, "predicted", last_d, v, age)
        if not res.t_onset and not allow_blind:
            # 아직 안 움직였다. 정속으로 미리 끊으면 출발도 전에 끊긴다 — 출발을 본 뒤 예측한다.
            # (예전 target/2 클램프는 이 문제를 가리느라 짧은 걸음의 예측을 절반으로 잘랐다 — 검토 지적)
            _tick(driver)
            continue
        # lead = 눈감은 거리(v x 나이) + 예측분(v x (tau + 판정주기/2) + 잔여)
        blind_m = v * age
        anticipate = learner.fwd_lead_m(res.strength, v, 0.0, _median(periods))
        lead = blind_m + anticipate
        if last_d - stop_at <= lead:
            return _cut(driver, res, now, "blind" if age > BLIND_MAX_S else "predicted",
                        last_d, v, age)
        _tick(driver)


def _tick(driver):
    time.sleep(POLL_S)
    driver.lease(LEASE_S)                           # 우리가 살아있다는 신호


def _median(xs):
    if not xs:
        return POLL_S
    ys = sorted(xs)
    return ys[len(ys) // 2]


def _cut(driver, res, now, reason, d_here, v, age):
    """**정지가 먼저.** 기록·계산은 그 다음이다 (plan 6-6 ④)."""
    driver.stop("직진 " + reason)
    res.t_stop_cmd, res.reason = now, reason
    res.d_at_stop_cmd = d_here if d_here is not None else 0.0
    res.speed_mps, res.detect_age_s = v or 0.0, age
    return reason


def _settle(driver, look_fn, res, start):
    """멈출 때까지 **본다**. 시간으로 때려맞히지 않는다.

    옆 프레임끼리 비교하면 안 된다 — 0.29 m/s 로 미끄러지는 중에도 한 프레임 변화는
    10 mm 라 잡음 문턱을 금방 밑돈다. **SETTLE_SPAN_S 전의 값과** 비교해야 진짜 정지가 보인다.
    """
    t0 = clock.now()
    pts, last = deque(), None
    settled = False
    while clock.now() - t0 < C.SETTLE_S:
        look = look_fn()
        if look.ok and (not pts or look.t_capture > pts[-1][0]):
            pts.append((look.t_capture, look.remaining_m))
            last = look.remaining_m
            # pts[0] 을 "SETTLE_SPAN_S 보다 오래된 것 중 가장 최근" 으로 남긴다.
            # 창을 SPAN 이하로 잘라 버리면 span >= SPAN 이 영영 안 참이 된다
            while len(pts) > 2 and look.t_capture - pts[1][0] >= SETTLE_SPAN_S:
                pts.popleft()
            span = look.t_capture - pts[0][0]
            quiet = SETTLE_QUIET_K * max(look.sigma_m, 1e-4)
            if (span >= SETTLE_SPAN_S and clock.now() - t0 > SETTLE_MIN_S
                    and abs(look.remaining_m - pts[0][1]) < quiet):
                settled = True
                break
        time.sleep(POLL_S)
    res.t_settled = clock.now()
    final = look_fn()
    if final.ok:
        res.d_final = final.remaining_m
    elif last is not None:
        res.d_final = last
    else:
        res.d_final = res.d_at_stop_cmd
    res.travelled_m = start.remaining_m - res.d_final
    res.travelled_at_stop_m = start.remaining_m - res.d_at_stop_cmd
    if res.reason == "predicted" and not settled:
        # 아직 미끄러지는 중이다. 이 값으로 배우면 관성을 실제보다 작게 배운다
        res.reason = "settle_blind" if not final.ok else "settle_timeout"


def _record(rec, res, movement, log, learned=None, **extra):
    if rec is None:
        return
    try:
        rec.event("forward_end", movement=movement, ok=res.ok, reason=res.reason,
                  strength=res.strength, target_m=res.target_m,
                  travelled_m=res.travelled_m, travelled_at_stop_m=res.travelled_at_stop_m,
                  d_at_stop_cmd=res.d_at_stop_cmd, d_final=res.d_final,
                  speed_mps=res.speed_mps, detect_age_s=res.detect_age_s,
                  blind_s=res.blind_s, hold_s=res.hold_s, t_cmd=res.t_cmd, t_onset=res.t_onset,
                  t_stop_cmd=res.t_stop_cmd, t_settled=res.t_settled,
                  track=res.track, learned=learned or {}, **extra)
    except Exception as e:
        log("          !! 직진 기록 실패: %s (주행은 계속한다)" % e)


if __name__ == "__main__":
    # 자체 시험 — 하드웨어 없이. 가상 시계: time.sleep 이 차 모델을 5 ms 씩 밀고 30 fps 프레임을 만든다.
    # 차 모델은 광운대 씨앗(죽은시간 1.498 s · 0.2897 m/s)에 정지지연 0.352 s(그쪽 실측)를 1차 지연으로.
    import math
    import random
    from src.models.control import learn as M
    from .learn import Learner

    random.seed(1)
    DT, FPS, DETECT_S, NOISE_M = 0.005, 30.0, 0.022, 0.010

    class VClock:
        t = 1000.0

    class Truck:
        def __init__(self, drv, v_mps=0.28973, startup_s=1.4977, tau_s=0.352):
            self.drv, self.v_cmd, self.startup, self.tau = drv, v_mps, startup_s, tau_s
            self.pos, self.v, self._m, self._t_set = 0.0, 0.0, "stop", None
            self.hist = deque(maxlen=4000)                     # (t, pos) — 프레임이 과거를 읽는다

        def step(self):
            m = self.drv.movement
            if m != self._m:
                self._m, self._t_set = m, VClock.t
            want = 0.0
            if m in ("forward", "forward_slow", "backward") and VClock.t - self._t_set >= self.startup:
                want = self.v_cmd
            self.v += (want - self.v) * (1 - math.exp(-DT / self.tau))
            self.pos += self.v * DT
            self.hist.append((VClock.t, self.pos))

        def pos_at(self, t):
            best = self.hist[0][1] if self.hist else self.pos
            for tt, p in self.hist:
                if tt <= t:
                    best = p
                else:
                    break
            return best

    class FakeDriver:
        def __init__(self):
            self.movement, self.deadline, self.log = "stop", None, []

        def set(self, m, why=""):
            assert self.deadline is not None, "set 전에 lease 가 걸려 있어야 한다"
            self.movement = m
            self.log.append((VClock.t, m))

        def stop(self, why=""):
            self.movement = "stop"
            self.log.append((VClock.t, "stop:" + why))

        def lease(self, s):
            self.deadline = VClock.t + s

        def lease_expired(self):
            return self.deadline is not None and VClock.t >= self.deadline

        def clear_lease(self):
            self.deadline = None

    class Camera:
        """30 fps. 노출 t 의 값은 검출 DETECT_S 뒤에야 보인다. lose_after_m 를 지나면 안 보인다."""
        def __init__(self, truck, stop_pos, lose_after_m=None):
            self.truck, self.stop_pos, self.lose = truck, stop_pos, lose_after_m
            self.k0 = math.floor(VClock.t * FPS)

        def look(self):
            k = math.floor((VClock.t - DETECT_S) * FPS)
            t_cap = k / FPS
            pos = self.truck.pos_at(t_cap)
            ok = self.lose is None or pos < self.lose
            return Look(remaining_m=self.stop_pos - pos + random.gauss(0, NOISE_M),
                        t_capture=t_cap, ok=ok, sigma_m=NOISE_M)

    drv = FakeDriver()
    truck = Truck(drv)

    def vsleep(s):
        for _ in range(max(1, int(round(s / DT)))):
            VClock.t += DT
            truck.step()
    clock.now = lambda: VClock.t
    time.sleep = vsleep
    quiet = lambda *_: None

    # 0) 창 기준 식
    assert abs(fit_span_s(0.039, 0.28973) - 3 * 0.039 / 0.28973) < 1e-12
    assert fit_span_s(0.039, None) == FIT_WINDOW_S and fit_span_s(0.0, 0.29) == 0.0
    assert strength_of("forward") == 67 and strength_of("forward_slow") == 97 and strength_of("backward") == 187

    # 1) 카메라 보며 1.0 m 세 번 — tau 를 배워 오차가 준다
    lrn = Learner(seeds=M.seeds())
    errs = []
    for i in range(3):
        cam = Camera(truck, stop_pos=truck.pos + 1.0)
        p0 = truck.pos
        r = forward(drv, lrn, cam.look, 1.0, log=quiet)
        assert r.ok, r.reason
        errs.append(truck.pos - p0 - 1.0)
        print("  1.000 m 요청 -> 실제 %.3f m (예측정지 %.3f, 배운 tau %.3f s, 속도 %.3f)"
              % (truck.pos - p0, r.travelled_m, lrn.fwd_tau["67"].value, r.speed_mps))
    assert abs(errs[-1]) < abs(errs[0]) or abs(errs[-1]) < 0.05, errs
    assert abs(errs[-1]) < 0.10, errs
    assert drv.deadline is None

    # 2) 거부들
    assert forward(drv, lrn, cam.look, 0.0, log=quiet).reason == "zero"
    assert forward(drv, lrn, cam.look, -1.0, log=quiet).reason == "negative"
    assert forward(drv, lrn, cam.look, C.STEP_FORWARD_HARD_MAX_M + 0.1, log=quiet).reason == "too_big"
    # 후진 모델이 없으면 거부. 광운대 씨앗은 187 을 직진값으로 심어 두므로(2026-10-02 결정) 씨앗을 빼고 만든 학습기로 본다
    lrn_nb = Learner(seeds={k: ({kk: vv for kk, vv in v.items() if kk != "187"} if isinstance(v, dict) else v)
                            for k, v in M.seeds().items()})
    assert forward(drv, lrn_nb, cam.look, 0.5, movement="backward", log=quiet).reason == "no_model"
    assert lrn.fwd_speed_mps(187) and lrn.fwd_startup_s(187) is not None      # 씨앗이 있으면 후진도 모델이 있다
    assert forward(drv, lrn, Camera(truck, truck.pos + 1.0, lose_after_m=-1).look, 0.5,
                   log=quiet).reason == "no_tag"

    # 3) 눈감고 가는 마지막 구간: 태그가 0.3 m 뒤에 사라지는데 1.4 m 가야 한다.
    #    예전 클램프(lead <= 걸음/2)면 눈감은 거리가 0.7 m 를 넘는 순간 정지 조건이 영영 안 참 → 워치독.
    cam = Camera(truck, stop_pos=truck.pos + 1.4, lose_after_m=truck.pos + 0.3)
    p0 = truck.pos
    r = forward(drv, lrn, cam.look, 1.4, allow_blind=True, log=quiet)
    gone = truck.pos - p0
    print("  눈감고 1.400 m -> 실제 %.3f m (reason %s, 눈감음 %.1fs)" % (gone, r.reason, r.blind_s))
    assert r.reason == "blind", r.reason
    assert abs(gone - 1.4) < 0.15, gone

    # 4) 시간 직진 — 유지시간 = 출발 + 거리/속도, 배우지 않는다
    n_before = lrn.fwd_tau["67"].n
    p0, t0 = truck.pos, VClock.t
    r = forward_timed(drv, lrn, 0.8, "forward", log=quiet)
    assert r.timed_ok and not r.ok, r.reason
    assert abs(r.hold_s - (lrn.fwd_startup_s(67) + 0.8 / lrn.fwd_speed_mps(67))) < 1e-9
    t_set = next(t for t, m in drv.log if t >= t0 and m == "forward")
    t_stop = next(t for t, m in drv.log if t > t_set and m.startswith("stop"))
    assert abs((t_stop - t_set) - r.hold_s) < 2 * DT, (t_stop - t_set, r.hold_s)
    assert abs(VClock.t - t_stop - C.SETTLE_S) < 2 * DT                   # 가라앉힌 뒤 돌아온다
    assert lrn.fwd_tau["67"].n == n_before                                  # 학습 없음
    print("  시간 직진 0.800 m -> 유지 %.2fs, 실제 %.3f m (모델 정지지연 만큼 더 간다)"
          % (r.hold_s, truck.pos - p0))
    assert forward_timed(drv, lrn_nb, 0.5, "backward", log=quiet).reason == "no_model"
    assert forward_timed(drv, lrn, 0.0, log=quiet).reason == "zero"
    assert forward_timed(drv, lrn, 10.0, log=quiet).reason == "too_long"
    print("forward 자체 시험 통과")
