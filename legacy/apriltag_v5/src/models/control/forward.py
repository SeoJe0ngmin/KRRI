"""직진 한 걸음. 카메라를 보며 **미리** 끊는다 (plan 4-2).

회전과 다른 점 하나 — 직진엔 자이로 같은 콜백이 없다. 거리는 카메라에서만 오고,
카메라는 "조금 전"을 알려준다. 그래서 끊는 시점을 이렇게 잡는다:

    남은거리 <= 속도 x (검출지연 + 관성시간 + 판정주기/2) + 잔여보정

속도는 **창을 잡고 직선을 맞춰서** 낸다. 두 프레임 차로 내면 0.28 m/s 에서 한 프레임이
9 mm 인데 거리 잡음이 +-39 mm(9/21 실측, 3.7 m) 라 그 차가 잡음에 묻힌다. 창이 안 차면 배운 정속을 쓴다.
창이 찼다고 보는 기준은 상수가 아니라 `3 x 거리잡음 / 정속` — 그만큼은 움직여야 기울기가 잡음 위로 나온다.

두 가지가 있다:
  forward()        카메라를 보며 간다. 정상 걸음
  forward_timed()  눈 감고 시간으로 간다. 사이드스텝 옆 이동 · 눈 감고 진입 (2026-09-30 결정 ⑤③)
둘 다 **여기서 배우지 않는다** (v5, 2026-10-02 결정 F1). 학습 파일(learnfile)이 다음 주행을 시작할 때 forward_end 기록에서
카메라 직진만 걸러 배운다 — 그래서 끊을 때 쓴 값(τ·잔여·정속·죽은시간)과 시작 측정을 forward_end 에 남긴다.

후진(movement="backward")도 같은 함수를 쓴다 — 단 learner 에 후진 속도가 있을 때만(없으면 no_model).
그때 look.remaining_m 은 **가는 방향으로** 남은 거리여야 한다(부르는 쪽이 맞춘다).

출발·멈춤 판정 (2026-10-03 결정 F5 — 예전엔 둘 다 "남은 거리" 를 3D 거리의 σ 와 견줬는데, 남은 거리는 PnP 각도·매번 다시 재는
태그컷이 섞여 서 있어도 흔들렸다: 2026-10-01 카메라 직진 28번 중 23번이 서 있는데 "출발", 12번이 다 섰는데 "멈춤 확인 실패"):
  출발 = 가는 쪽으로 태그까지 **3D 거리가 줄어드는 속도**(추정기 0.5초 창 기울기 — Look.moving_mps)가 정속의 ONSET_FRACTION 을
         넘는 화면이 ONSET_N 장 연속. 명령 뒤에 찍힌 화면만. 카메라가 못 보면 가속도계(더한 속도 > 정속의 ONSET_IMU_FRACTION),
         명령 뒤 배운 출발 시간 × ONSET_TIMEOUT_K 가 지나도 못 보면 출발로 친다(놓치면 목표를 지난 뒤에야 끊어 약 14 cm 더 간다).
         출발 판정이 끊는 데 필요한 건 짧은 직진뿐이다 — 긴 직진은 미리 끊기가 일찍 켜져도 끊을 조건이 안 된다.
  멈춤 = 가속도계. 정지 명령 뒤 제동을 본 다음 조용함이 IMU_QUIET_S 이어지면 (imu_settled_at). 가속도계가 없으면 STILL_AFTER_S.

안전망 두 겹 — 회전보다 하나 적다(CAN 쪽 타이머는 회전 전용이다):
  1) 여기서 재는 워치독
  2) 데드맨 lease — 우리가 죽으면 CAN 스레드가 LEASE_S 안에 세운다

시각은 전부 clock.now()(monotonic). Look.t_capture 도 CameraClock.see() 로 바꾼 우리 시계 값이다.
"""
import bisect
import math
import time
from collections import deque
from dataclasses import dataclass, field

from config import control as C
from ...utils import clock
from ...utils.gyro import Gyro

POLL_S = 0.005         # 측정을 확인하는 주기. 새 프레임이 없으면 그냥 지나간다
LEASE_S = C.DEADMAN_S  # 이 안에 명령이 안 갱신되면 CAN 스레드가 세운다. 값은 config (플랫폼을 탄다 — VM 1.0 / 직결 0.30)
ABORT_FACTOR = 2.0     # 예상의 이 배를 넘으면 무조건 끊는다. 중단 규칙이지 측정값이 아니다
FIT_WINDOW_S = 0.6     # 속도를 맞출 창. 0.29 m/s 면 174 mm 움직인다 (잡음 39 mm 보다 크다)
FIT_MIN_N = 5
FIT_SPAN_K = 3.0       # 창이 찼다 = 그동안 간 거리가 잡음의 이 배 (3σ). 2026-09-30 결정 ⑨
BLIND_MAX_S = 1.0      # 태그를 이만큼 못 보면 선다. 0.3 m/s 면 30 cm — 그 안에선 더듬어도 된다
# 출발 (2026-10-03 결정 F5)
ONSET_FRACTION = 1.0 / 3.0   # 가는 쪽 3D 거리 속도가 정속(학습 파일)의 이만큼을 넘으면 — 그날 기준선은 안 쓴다. 근거 2026-10-01 28번:
                             # 서 있을 때 3장 연속 넘은 최대 속도 5 m 안 0.014 · 5~7 m 0.035 · 7~10 m 0.08 m/s, 정속 0.27~0.29.
                             # 문턱 0.08~0.10 에서 너무 이름 0 · 놓침 0. 짧은 직진은 태그 가까이(10/1 전부 3.3~4.7 m)라 여유가 크다
ONSET_N = 3                  # 연속 이만큼 넘어야 인정. 한 프레임만 보면 잡음이 출발로 둔갑한다
ONSET_IMU_FRACTION = 0.5     # 예비 — 카메라가 못 볼 때 가속도계: 명령 뒤 (앞 가속도 − 직전 기준)을 더한 속도가 정속의 이만큼을 넘으면.
                             # 가속도는 출발에 약하다(출발 +0.2~0.4 m/s² 가 진동 ±0.3 에 묻히고, 더하면 오차가 쌓인다) — 그래서 예비.
                             # 10/1: 서 있는 동안 더한 속도(가짜) 최대 0.056 m/s, 28번 중 25번 잡음
ONSET_TIMEOUT_K = 2.0        # 안전망 — 명령 뒤 배운 출발 시간(학습 파일 dead_s, 10/1 1.20 s) × 이만큼 지나도 못 보면 출발로 친다
# 멈춤 (2026-10-03 결정 F5) — 가속도계
IMU_REF_S = 1.0              # 기준 = 직진 명령 직전 이만큼(차가 서 있다)의 앞 가속도. 10/1 비교: 명령 직후 서 있는 구간·둘 합침과 결과가 같았다
IMU_BIN_S = 0.1              # 가속도는 0.1초 평균으로 본다 (한 번 잴 때 진동 ±0.3 m/s²)
IMU_STEP_S = 0.05            # 0.1초 평균을 이 간격으로 · 멈춤 확인 주기
IMU_BIN_MIN_N = 4            # 0.1초 칸에 샘플이 이만큼은 있어야 한다 (100 Hz 면 10개)
IMU_K = 3.0                  # 제동 = 기준 − 3 × 흔들림 아래 · 조용함 = 기준 ± 3 × 흔들림 안 (흔들림 = 기준 구간 0.1초 평균의 sd)
IMU_QUIET_S = 0.3            # 조용함이 이만큼 이어지면 멈춤. 되튐으로 다시 넘으면 처음부터 다시 센다 (10/1 되튐 0.13~0.55 m/s² — 28번 중 13번 다시 셈)
STILL_AFTER_S = Gyro.STILL_AFTER_S   # 1.0 s — 제동을 못 봤어도 정지 명령 뒤 이만큼 지났으면 조용함만 본다. 가속도계가 없으면 이 시간에 멈춤으로 친다


@dataclass
class Look:
    """지금 카메라가 말하는 것. 부르는 쪽(estimate/plan)이 채운다."""
    remaining_m: float = 0.0    # 서야 할 곳까지 남은 거리. 지나쳤으면 음수
    t_capture: float = 0.0      # **노출 시각** (clock.now 기준 — CameraClock.see 가 바꿔준 값). 지금이 아니다
    ok: bool = False            # 태그를 믿을 만하게 봤나
    sigma_m: float = 0.0        # 이 값의 흔들림
    moving_mps: float | None = None   # 가는 쪽으로 태그까지 3D 거리가 줄어드는 속도 [m/s] — 추정기 0.5초 창 기울기(Fix.closing_mps × 방향).
                                      # 출발 판정에 쓴다 (F5). 남은 거리와 달리 PnP 각도·태그컷이 안 섞인다. 모르면 None


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
    t_onset_ok: float = 0.0        # 출발을 인정한 때(우리 시계) — 정지 명령을 보낼 수 있는 첫 순간. 최소걸음 재료 (F1-3)
    cut_at_onset: bool = False     # 출발을 인정하자마자 끊었다(목표가 최소걸음보다 짧았다) — 간 거리 = 최소걸음. τ·잔여 학습엔 안 넣는다
    t_stop_cmd: float = 0.0
    t_settled: float = 0.0
    hold_s: float = 0.0            # 시간 직진일 때 명령을 쥔 시간
    onset_by: str = ""             # 출발을 무엇으로 봤나 — camera / imu / time (F5)
    settle_by: str = ""            # 멈춤을 무엇으로 봤나 — imu / time. 비었으면 SETTLE_S 안에 못 봤다 (F5)
    track: list = field(default_factory=list)   # (t, 남은거리) — 분석용

    @property
    def done(self):
        return self.t_settled > 0.0

    @property
    def ok(self):
        """카메라를 보고 제대로 끝난 걸음 (학습 파일은 이것과 멈춤 확인 실패 settle_timeout 을 넣는다)."""
        return self.done and self.reason == "predicted"

    @property
    def timed_ok(self):
        """시간 직진이 끝까지 간 것. 실제로 얼마나 갔는지는 모른다 — 학습 재료가 못 된다."""
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
    """이보다 짧게는 못 간다 [m]. 학습 파일 값 = **가장 빨리 끊었을 때 실제로 가는 거리**(출발 확인까지 간 거리 + 그때 속도로 미끄러진 거리,
    learnfile — F1-3, 2026-10-11). 그게 없으면 정속 x 관성시간 — 출발 확인까지 가는 거리를 빼먹어 작게 나온다(10/1: 0.085 대 실제 약 0.16 m)."""
    m = learner.fwd_min_step_m(strength)
    if m:
        return m
    v = learner.fwd_speed_mps(strength)
    tau = learner.fwd_tau_s(strength)
    return v * tau if (v and tau) else None


def strength_of(movement):
    """동작 이름 -> CAN byte2. 배운 값을 담는 열쇠이기도 하다 (중립 127 에서 강도만큼)."""
    if movement == "backward":
        return 127 + C.BACKWARD_JOYSTICK_DEFLECTION
    return 127 - (C.FORWARD_SLOW_JOYSTICK_DEFLECTION if movement == "forward_slow"
                  else C.FORWARD_JOYSTICK_DEFLECTION)


def forward(driver, learner, look_fn, target_m, movement="forward",
            allow_blind=False, rec=None, log=print, lost_s=None, imu=None):
    """target_m 만큼 앞으로 간다. look_fn() 은 지금 측정(Look)을 돌려준다.
    imu = 앞 가속도를 주는 것(Gyro.accel_fwd) — 멈춤 판정·출발 예비에 쓴다. 없으면 멈춤은 정지 명령 뒤 STILL_AFTER_S 로 친다.

    allow_blind=True 는 **마지막 진입 전용** — 태그가 화면 밖으로 나가는 걸 알고 가는 구간이다.
    그때는 최종 거리를 못 재니 아무것도 배우지 않는다(reason='blind').
    lost_s = 태그를 이만큼 못 보면 선다(기본 BLIND_MAX_S). 마지막 구간의 "태그컷까지 곧장" 다리는 측정이 낡는 순간
    (clock.MAX_AGE_S) 선다 — 그 다리의 목적이 태그컷 도착이라 더 갈 이유가 없다 (v5 결정 R4 ②).
    """
    target_m = float(target_m)
    strength = strength_of(movement)
    res = LegResult(target_m=target_m, strength=strength)

    # 시작 측정 하나로 "보이나" 확인과 출발 기준을 같이 한다. 예전엔 따로 두 번 읽어, 확인 땐 보였는데 출발 기준은 못 본 측정
    # (남은거리 0.0)으로 잡힌 적이 있다 — 2026-10-01 16:55 의 0.67 m 직진은 실제 1.01 m 를 갔는데 0.63 m 로 남았다 (F2)
    start = look_fn()
    used = {"tau_used": learner.fwd_tau_s(strength), "residual_used": learner.fwd_residual_m(strength),
            "speed_used": learner.fwd_speed_mps(strength), "dead_used": learner.fwd_startup_s(strength),
            "start_ok": bool(start.ok), "start_remaining_m": start.remaining_m if start.ok else None}
    refused = _refuse(learner, target_m, strength, movement, start)
    if refused:
        log("       !! 직진 거부(%s): %.3f m" % (refused, target_m))
        res.reason = refused
        _record(rec, res, movement, log, **used)
        return res

    hold_s = _hold_s(learner, target_m, strength)
    log("       -> 직진 %.3f m  (강도 %d, 안전망 %.1fs)" % (target_m, strength, hold_s))
    sgn = -1.0 if movement == "backward" else 1.0
    ref = imu_reference(imu, clock.now())           # 직진 명령 직전(서 있다)의 앞 가속도 — 멈춤 판정의 기준
    used.update(imu_base=ref[0] if ref else None, imu_sd=ref[1] if ref else None)

    res.t_cmd = clock.now()
    driver.lease(LEASE_S)
    try:
        driver.set(movement, why="%s %.3f m" % (movement, target_m))
    except BaseException:
        driver.stop("직진 명령 실패")
        driver.clear_lease()
        raise

    try:
        _drive(driver, learner, look_fn, res, start, hold_s, allow_blind, BLIND_MAX_S if lost_s is None else float(lost_s),
               imu=imu, ref=ref, sgn=sgn)
        _settle(driver, look_fn, res, start, imu=imu, ref=ref, sgn=sgn)
    finally:
        driver.stop("직진 끝 " + (res.reason or "?"))
        driver.clear_lease()

    _record(rec, res, movement, log, **used)
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
    used = {"speed_used": v, "dead_used": startup}
    if refused:
        log("       !! 시간 직진 거부(%s): %s %.3f m" % (refused, movement, distance_m))
        res.reason = refused
        _record(rec, res, movement, log, timed=True, **used)
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
    _record(rec, res, movement, log, timed=True, **used)
    if res.timed_ok:
        log("          끝: %.3f m 을 %.2fs 유지 (모델상 %.3f m)"
            % (distance_m, res.hold_s, res.travelled_m))
    else:
        log("          !! 시간 직진 실패(%s): %.2fs 중 %.2fs"
            % (reason, res.hold_s, res.t_stop_cmd - res.t_cmd))
    return res


# ── 속 ──────────────────────────────────────────────────────────────
def _refuse(learner, target_m, strength, movement, start):
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
    if not start.ok:
        return "no_tag"                     # 안 보이는 채로 출발하지 않는다 — 출발 기준도 이 측정이다
    return ""


def _hold_s(learner, target_m, strength):
    v = learner.fwd_speed_mps(strength)
    startup = learner.fwd_startup_s(strength)
    if not v or startup is None:
        return C.FWD_SAFETY_MAX_S
    return min(C.FWD_SAFETY_MAX_S, (target_m / v + startup) * ABORT_FACTOR)


def _drive(driver, learner, look_fn, res, start, hold_s, allow_blind, lost_s=BLIND_MAX_S, imu=None, ref=None, sgn=1.0):
    """갈 만큼 가고 **미리** 끊는다.

    판단 기준은 늘 "마지막으로 **본** 거리" 와 "그게 얼마나 낡았나" 다. 낡은 걸 속도로 메워
    현재 위치를 지어내지 않는다 — 그 지어낸 값으로 tau 를 배우면 tau 가 지연을 두 번 세게 된다.
    대신 낡은 만큼을 lead 에 그대로 넣는다. 안 보일수록 lead 가 커져 **일찍** 선다 (plan 4-10).
    미리 끊기는 출발을 본 뒤에만 켠다 — 출발은 3D 거리 속도로 보고, 카메라가 못 보면 가속도, 그래도 못 보면 시간 (F5).
    """
    speed, periods = Speed(), deque(maxlen=20)
    nominal = learner.fwd_speed_mps(res.strength)
    stop_at = start.remaining_m - res.target_m      # 남은거리가 여기까지 줄면 도착
    last_t = last_d = onset_t = None
    onset_n = 0
    checked = False                                 # 출발 인정 뒤 끊을지 한 번이라도 봤나 (cut_at_onset)
    span_min = fit_span_s(start.sigma_m, nominal, speed.window_s)
    onset_cam = ONSET_FRACTION * nominal if nominal else None
    onset_imu = ONSET_IMU_FRACTION * nominal if nominal else None
    dead = learner.fwd_startup_s(res.strength)
    onset_deadline = res.t_cmd + ONSET_TIMEOUT_K * dead if dead else None
    imu_next = 0.0

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
            # 출발 — 명령 뒤에 찍힌 화면의 3D 거리 속도가 정속의 ONSET_FRACTION 을 ONSET_N 장 연속 넘으면
            if not res.t_onset and onset_cam and look.moving_mps is not None and look.t_capture > res.t_cmd:
                if look.moving_mps > onset_cam:
                    onset_t = onset_t or look.t_capture
                    onset_n += 1
                    if onset_n >= ONSET_N:
                        res.t_onset, res.t_onset_ok, res.onset_by = onset_t, now, "camera"
                else:
                    onset_t, onset_n = None, 0
        if not res.t_onset:
            if (not look.ok or look.moving_mps is None) and ref is not None and onset_imu and now >= imu_next:
                imu_next = now + IMU_STEP_S          # 카메라가 못 본다 — 가속도계로 (예비)
                try:
                    hit = next((t for t, v in imu_speed(imu.accel_fwd(res.t_cmd), res.t_cmd, ref[0], sgn) if v > onset_imu), None)
                except Exception:
                    hit = None
                if hit is not None:
                    res.t_onset, res.t_onset_ok, res.onset_by = hit, now, "imu"
            if not res.t_onset and onset_deadline is not None and now >= onset_deadline:
                res.t_onset, res.t_onset_ok, res.onset_by = now, now, "time"   # 끝내 못 봤다 — 출발로 친다 (놓치면 지난 뒤에야 끊는다)

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

        if age > lost_s and not allow_blind:
            return _cut(driver, res, now, "tag_lost", last_d, v, age)
        if last_d - stop_at <= 0:                    # 이미 지났다 — 무조건 끊는다
            res.cut_at_onset = bool(res.t_onset) and not checked
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
            res.cut_at_onset = bool(res.t_onset) and not checked   # 출발 인정 뒤 첫 판단에서 벌써 끊을 때 — 더 일찍은 못 끊는다
            return _cut(driver, res, now, "blind" if age > BLIND_MAX_S else "predicted",
                        last_d, v, age)
        checked = bool(res.t_onset)
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


def _settle(driver, look_fn, res, start, imu=None, ref=None, sgn=1.0):
    """멈출 때까지 기다린다 — 가속도계로 본다 (2026-10-03 결정 F5). 끝 위치는 그 뒤 카메라로 잰다.

    정지 명령 뒤 제동(앞 가속도가 직진 명령 직전 기준보다 IMU_K × 흔들림 아래)을 본 다음, 기준 ± IMU_K × 흔들림 안이
    IMU_QUIET_S 이어지면 멈춤 (imu_settled_at). 예전엔 카메라의 남은 거리를 1초 전과 견줬는데, 그 값이 서 있어도 흔들려
    2026-10-01 카메라 직진 28번 중 12번이 2초를 다 기다리고 "멈춤 확인 실패" 였다. 가속도계는 카메라 거리·프레임과 상관없다
    (10/1 다시 돌림: 28번 모두 정지 명령 뒤 0.45~1.25초에 조용해짐, '섰다' 뒤 카메라로 더 간 거리 5.5 m 안 ±1.6 mm).
    가속도계가 없거나 기준을 못 냈으면 정지 명령 뒤 STILL_AFTER_S(1초 — 자이로 영점과 같은 근거).
    """
    t_stop = res.t_stop_cmd or clock.now()
    last, settled_by = None, ""
    while clock.now() - t_stop < C.SETTLE_S:
        look = look_fn()
        if look.ok:
            last = look
        if ref is not None and imu is not None:
            try:
                pts = imu.accel_fwd(t_stop)
            except Exception:
                pts = []
            if imu_settled_at(pts, t_stop, ref, sgn) is not None:
                settled_by = "imu"
                break
        elif clock.now() - t_stop >= STILL_AFTER_S:
            settled_by = "time"
            break
        time.sleep(IMU_STEP_S)
    res.t_settled = clock.now()
    res.settle_by = settled_by
    final = look_fn()
    if final.ok:
        res.d_final = final.remaining_m
    elif last is not None:
        res.d_final = last.remaining_m
    else:
        res.d_final = res.d_at_stop_cmd
    res.travelled_m = start.remaining_m - res.d_final
    res.travelled_at_stop_m = start.remaining_m - res.d_at_stop_cmd
    if res.reason == "predicted":
        if not final.ok:
            res.reason = "settle_blind"          # 선 뒤 태그를 못 봤다 — 끝 위치가 옛 값이다
        elif not settled_by:
            res.reason = "settle_timeout"        # SETTLE_S 안에 조용해지지 않았다 — 끝 위치는 그때 잰 값


# ── 가속도계 (F5) ────────────────────────────────────────────────────
def _bins(pts, t0, t1, width=IMU_BIN_S, step=IMU_STEP_S):
    """[(t, a)] (시각 순) → [(칸 끝 시각, 평균)] — [t, t + width) 칸을 t0 부터 step 마다, 칸 끝이 t1 을 안 넘는 것만.
    샘플이 IMU_BIN_MIN_N 미만인 칸은 뺀다(가속도계가 끊긴 구간)."""
    ts = [t for t, _ in pts]
    out, t = [], t0
    while t + width <= t1 + 1e-9:
        i, j = bisect.bisect_left(ts, t), bisect.bisect_left(ts, t + width)
        if j - i >= IMU_BIN_MIN_N:
            out.append((t + width, sum(a for _, a in pts[i:j]) / (j - i)))
        t += step
    return out


def imu_reference(imu, t_end, span_s=IMU_REF_S):
    """직진 명령 직전 span_s 초(차가 서 있다)의 앞 가속도 → (기준, 흔들림). 모자라면 None.
    기준 = 0.1초 평균들의 평균. 흔들림 = 0.1초 평균들의 sd 인데, **샘플 하나의 sd ÷ √(칸당 샘플 수)** 보다 작게는 안 잡는다 —
    평균 열 개의 sd 는 우연히 작게 나올 수 있고(그러면 조용함 띠가 너무 좁아 멈춤을 영영 못 본다), 진동이 서로 닮아 있으면 칸 평균의
    흔들림은 그 값보다 크면 컸지 작을 수 없다. (2026-10-01 28번 다시 돌림: 넣기 전과 같이 28번 모두 확인 — 0.45~1.05 s, 넣기 전 1.25 s ·
    '섰다' 뒤 카메라로 더 간 거리 5.5 m 안 ±1.6 mm 그대로)"""
    if imu is None:
        return None
    try:
        pts = imu.accel_fwd(t_end - span_s, t_end)
    except Exception:
        return None
    bins = _bins(pts, t_end - span_s, t_end, IMU_BIN_S, IMU_BIN_S)
    if len(bins) < 3:
        return None
    means = [v for _, v in bins]
    m = sum(means) / len(means)
    sd_bins = math.sqrt(sum((v - m) ** 2 for v in means) / len(means))
    raw = [a for t, a in pts if t_end - span_s <= t < t_end]
    mr = sum(raw) / len(raw)
    per_bin = len(raw) / len(bins)
    sd_floor = math.sqrt(sum((a - mr) ** 2 for a in raw) / len(raw)) / math.sqrt(per_bin)
    return m, max(sd_bins, sd_floor)


def imu_settled_at(pts, t_stop, ref, sgn=1.0, still_after=STILL_AFTER_S):
    """정지 명령(t_stop) 뒤 앞 가속도 [(t, a)] → 멈춘 시각(조용함이 시작된 칸의 끝) 또는 None(아직).
    제동(기준 − IMU_K × 흔들림 아래)을 본 다음 — 못 봤어도 정지 명령 뒤 still_after 가 지났으면 — 기준 ± IMU_K × 흔들림 안이
    IMU_QUIET_S 이어지면 멈춤. 되튐으로 다시 넘으면 처음부터 다시 센다. sgn = 가는 쪽 (후진 −1)."""
    if not pts or ref is None:
        return None
    base, sd = ref
    lim = IMU_K * max(sd, 1e-6)
    seen, quiet = False, None
    for te, mean in _bins(pts, t_stop, pts[-1][0]):
        v = sgn * (mean - base)
        if not seen:
            braking = v < -lim
            seen = braking or te - t_stop >= still_after
            if braking or not seen:
                continue                             # 제동 칸 자체는 조용함이 아니다
        if abs(v) <= lim:
            quiet = te if quiet is None else quiet
            if te - quiet >= IMU_QUIET_S:
                return quiet
        else:
            quiet = None
    return None


def imu_speed(pts, t0, base, sgn=1.0):
    """명령(t0) 뒤 (앞 가속도 − 기준)을 더한 속도 [(t, m/s)] — 출발 예비용. 샘플이 끊긴 구간은 IMU_STEP_S 까지만 더한다
    (몰아서 더하면 없는 속도가 생긴다). 더할수록 오차가 쌓이니 출발 직후 몇 초만 믿는다."""
    v, out, t_prev = 0.0, [], t0
    for t, a in pts:
        if t <= t0:
            continue
        v += sgn * (a - base) * min(t - t_prev, IMU_STEP_S)
        t_prev = t
        out.append((t, v))
    return out


def _record(rec, res, movement, log, **extra):
    """forward_end — 학습 파일(learnfile)이 다음 주행 때 읽는 재료. extra = 그때 쓴 값(tau_used·residual_used·speed_used·
    dead_used)과 시작 측정(start_ok·start_remaining_m), 시간 직진이면 timed=True."""
    if rec is None:
        return
    try:
        rec.event("forward_end", movement=movement, ok=res.ok, reason=res.reason,
                  strength=res.strength, target_m=res.target_m,
                  travelled_m=res.travelled_m, travelled_at_stop_m=res.travelled_at_stop_m,
                  d_at_stop_cmd=res.d_at_stop_cmd, d_final=res.d_final,
                  speed_mps=res.speed_mps, detect_age_s=res.detect_age_s,
                  blind_s=res.blind_s, hold_s=res.hold_s, t_cmd=res.t_cmd, t_onset=res.t_onset,
                  t_onset_ok=res.t_onset_ok, cut_at_onset=res.cut_at_onset, t_stop_cmd=res.t_stop_cmd, t_settled=res.t_settled, onset_by=res.onset_by, settle_by=res.settle_by,
                  track=res.track, **extra)
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
    VIB, TILT = 0.3, 0.2          # 가짜 가속도계: 한 번 잴 때 진동 ±0.3 m/s²(2026-10-01 서 있을 때와 같은 크기) · 카메라가 숙인 만큼 섞인 중력
    rng_imu, rng_mv = random.Random(2), random.Random(3)    # 가속도·속도 잡음은 따로 — 카메라 거리 잡음의 순서를 안 바꾸게

    class VClock:
        t = 1000.0

    class Truck:
        def __init__(self, drv, v_mps=0.28973, startup_s=1.4977, tau_s=0.352):
            self.drv, self.v_cmd, self.startup, self.tau = drv, v_mps, startup_s, tau_s
            self.pos, self.v, self._m, self._t_set = 0.0, 0.0, "stop", None
            self.hist = deque(maxlen=4000)                     # (t, pos) — 프레임이 과거를 읽는다
            self.acc = deque(maxlen=4000)                      # (t, 앞 가속도 + 진동 + 중력 몫) — 가짜 가속도계

        def step(self):
            m = self.drv.movement
            if m != self._m:
                self._m, self._t_set = m, VClock.t
            want = 0.0
            if m in ("forward", "forward_slow", "backward") and VClock.t - self._t_set >= self.startup:
                want = self.v_cmd
            dv = (want - self.v) * (1 - math.exp(-DT / self.tau))
            self.v += dv
            self.pos += self.v * DT
            self.hist.append((VClock.t, self.pos))
            self.acc.append((VClock.t, dv / DT + TILT + rng_imu.gauss(0, VIB)))

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
            mv = (pos - self.truck.pos_at(t_cap - 0.5)) / 0.5 + rng_mv.gauss(0, 0.005)   # 0.5초 창 기울기처럼 늦게 따라온다
            return Look(remaining_m=self.stop_pos - pos + random.gauss(0, NOISE_M),
                        t_capture=t_cap, ok=ok, sigma_m=NOISE_M, moving_mps=mv)

    class FakeImu:
        def __init__(self, truck):
            self.truck = truck

        def accel_fwd(self, t0, t1=None):
            return [(t, a) for t, a in list(self.truck.acc) if t >= t0 and (t1 is None or t <= t1)]

    drv = FakeDriver()
    truck = Truck(drv)
    imu = FakeImu(truck)

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

    # 1) 카메라 보며 1.0 m 세 번 — 끊고 서는지, 주행 중엔 값이 안 바뀌는지 (v5: 배우는 건 학습 파일). 정확도 주장이 아니다
    lrn = Learner(seeds=M.seeds())
    tau0, res0 = lrn.fwd_tau_s(67), lrn.fwd_residual_m(67)

    class Rec:
        def __init__(self):
            self.rows = []

        def event(self, kind, **kw):
            self.rows.append(dict(kw, event=kind))
    rec = Rec()
    for i in range(3):
        time.sleep(1.0)                                      # 판단하는 동안 서 있다 — 직전 1초가 가속도 기준이 된다
        cam = Camera(truck, stop_pos=truck.pos + 1.0)
        p0 = truck.pos
        r = forward(drv, lrn, cam.look, 1.0, rec=rec, log=quiet, imu=imu)
        assert r.ok, r.reason
        assert abs(truck.pos - p0 - 1.0) < 0.10, truck.pos - p0
        # 출발은 카메라 속도로, 실제 출발(모델 죽은시간) 뒤에 / 멈춤은 가속도계로, 차가 거의 섰을 때
        assert r.onset_by == "camera" and r.t_onset > r.t_cmd + truck.startup, (r.onset_by, r.t_onset - r.t_cmd)
        assert r.settle_by == "imu" and 0.5 < r.t_settled - r.t_stop_cmd < C.SETTLE_S, (r.settle_by, r.t_settled - r.t_stop_cmd)
        # 가짜 차는 1차 지연이라 꼬리가 길다(실차는 0.3~0.4초 제동 뒤 멎었다) — 진동보다 작은 감속은 가속도계가 못 본다.
        # 여기선 "제동 뒤에, 거의 섰을 때" 만 본다 (남은 미끄러짐 = 속도 × τ)
        assert abs(truck.v) < 0.2 * truck.v_cmd, truck.v
        print("  1.000 m 요청 -> 실제 %.3f m (예측정지 %.3f, 쓴 tau %.3f s, 속도 %.3f · 출발 명령 뒤 %.2f s · 멈춤 정지 명령 뒤 %.2f s)"
              % (truck.pos - p0, r.travelled_m, lrn.fwd_tau_s(67), r.speed_mps, r.t_onset - r.t_cmd, r.t_settled - r.t_stop_cmd))
    assert lrn.fwd_tau_s(67) == tau0 and lrn.fwd_residual_m(67) == res0          # 주행 중엔 안 배운다
    e = rec.rows[-1]
    assert e["tau_used"] == tau0 and e["residual_used"] == res0 and e["start_ok"] and e["start_remaining_m"] is not None, e
    assert abs(e["travelled_m"] + e["d_final"] - e["start_remaining_m"]) < 1e-9     # 간 거리 = 시작 − 끝 (같은 시작 측정)
    assert drv.deadline is None
    assert not e["cut_at_onset"] and e["t_onset_ok"] >= e["t_onset"] > e["t_cmd"], e   # 1 m 는 출발 인정 뒤에도 한참 간다

    # 1-b) 최소걸음 (F1-3) — 목표가 출발 인정 때 이미 끊을 거리 안이면 인정하자마자 끊고 cut_at_onset 으로 남긴다.
    #      학습 파일 값이 있으면 그게 최소걸음이고, 그보다 짧은 요청은 거부한다
    time.sleep(1.0)
    cam = Camera(truck, stop_pos=truck.pos + 0.09)
    r = forward(drv, lrn, cam.look, 0.09, rec=rec, log=quiet, imu=imu)
    assert r.cut_at_onset and r.onset_by == "camera" and r.t_stop_cmd - r.t_onset_ok < 2 * DT, (r.cut_at_onset, r.t_stop_cmd - r.t_onset_ok)
    assert rec.rows[-1]["cut_at_onset"] and rec.rows[-1]["t_onset_ok"] == r.t_onset_ok
    assert abs(min_step_m(lrn, 67) - lrn.fwd_speed_mps(67) * lrn.fwd_tau_s(67)) < 1e-12        # 학습 파일 값이 없으면 식
    lrn_ms = Learner(seeds=M.seeds())
    lrn_ms.set_forward({"67": {"min_step_m": 0.16}})
    assert min_step_m(lrn_ms, 67) == 0.16 and min_step_m(lrn_ms, 187) == lrn_ms.fwd_speed_mps(187) * lrn_ms.fwd_tau_s(187)
    assert forward(drv, lrn_ms, cam.look, 0.12, log=quiet).reason == "too_small"
    print("  0.090 m 요청 -> 출발 인정하자마자 끊음, 실제 %.3f m (가짜 차 — 숫자는 근거가 아니다)" % (r.travelled_m))

    # 2) 거부들. 시작 측정이 안 보이면(F2) 그 측정으로 거부한다 — 확인과 출발 기준이 같은 측정
    flip = iter([Look(ok=False), Look(remaining_m=1.0, t_capture=VClock.t, ok=True, sigma_m=NOISE_M)])
    assert forward(drv, lrn, lambda: next(flip), 0.5, rec=rec, log=quiet).reason == "no_tag"
    assert rec.rows[-1]["start_ok"] is False and rec.rows[-1]["start_remaining_m"] is None
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

    # 5) 출발 예비 — 카메라가 속도를 못 주면 가속도계, 그것도 없으면 배운 출발 시간 × 2 에 출발로 친다
    def no_speed(look_fn):
        def f():
            lk = look_fn()
            lk.moving_mps = None
            return lk
        return f
    time.sleep(1.0)
    cam = Camera(truck, stop_pos=truck.pos + 1.0)
    r = forward(drv, lrn, no_speed(cam.look), 1.0, log=quiet, imu=imu)
    assert r.onset_by == "imu" and r.t_onset > r.t_cmd + truck.startup, (r.onset_by, r.t_onset - r.t_cmd)
    time.sleep(1.0)
    cam = Camera(truck, stop_pos=truck.pos + 1.0)
    r = forward(drv, lrn, no_speed(cam.look), 1.0, log=quiet)
    assert r.onset_by == "time" and abs((r.t_onset - r.t_cmd) - ONSET_TIMEOUT_K * lrn.fwd_startup_s(67)) < 2 * DT, r.t_onset - r.t_cmd
    assert r.settle_by == "time" and abs((r.t_settled - r.t_stop_cmd) - STILL_AFTER_S) < IMU_STEP_S + 2 * DT, r.t_settled - r.t_stop_cmd
    print("  출발 예비: 카메라 속도 없음 → 가속도계 / 둘 다 없음 → 명령 뒤 %.2f s 에 출발로 침 · 멈춤은 정지 명령 뒤 %.2f s"
          % (r.t_onset - r.t_cmd, r.t_settled - r.t_stop_cmd))

    # 6) 멈춤 규칙 — 합성 가속도: 제동 → 되튐 → 조용. 되튐이 기준 ± 3σ 를 넘으면 0.3초를 다시 센다. 후진은 부호가 반대
    def trace(parts, t0=0.0, hz=100.0):
        out, t = [], t0
        for dur, a in parts:
            for _ in range(int(round(dur * hz))):
                out.append((t, a))
                t += 1.0 / hz
        return out
    ref = (0.2, 0.05)                                         # 기준 0.2 (중력 몫) · 흔들림 0.05 → 문턱 ±0.15
    plain = trace([(0.2, 0.2), (0.3, -0.8), (1.5, 0.2)])
    rebound = trace([(0.2, 0.2), (0.3, -0.8), (0.1, 0.6), (1.5, 0.2)])
    assert abs(imu_settled_at(plain, 0.0, ref) - 0.6) < 1e-6, imu_settled_at(plain, 0.0, ref)
    assert abs(imu_settled_at(rebound, 0.0, ref) - 0.7) < 1e-6, imu_settled_at(rebound, 0.0, ref)
    assert imu_settled_at(trace([(0.2, 0.2), (0.3, -0.8), (0.4, 0.2)]), 0.0, ref) is None     # 아직 0.3초 안 됐다
    assert abs(imu_settled_at(trace([(2.0, 0.2)]), 0.0, ref) - 1.0) < 1e-6                     # 제동을 못 봤으면 1초 뒤부터 조용함만
    back = [(t, 0.4 - a) for t, a in rebound]                 # 후진: 제동이 + 쪽 (기준 0.2 를 중심으로 뒤집음)
    assert abs(imu_settled_at(back, 0.0, ref, sgn=-1.0) - 0.7) < 1e-6
    assert imu_reference(FakeImu(truck), 0.0) is None                                          # 그때 샘플이 없으면 기준을 못 낸다
    print("  멈춤 규칙: 제동 뒤 0.3초 조용 → 0.60 s · 되튐 있으면 → 0.70 s (다시 셈) · 제동 못 봄 → 1.00 s")
    print("forward 자체 시험 통과")
