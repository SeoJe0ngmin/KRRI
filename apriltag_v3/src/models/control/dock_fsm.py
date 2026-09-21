"""수렴 제어 상태기계 — 관측 → 결정 → 실행 → 정지 → 확인.  (plan 3-0~3-7·4-4, 계약 §4)

도킹 = **접힌 포크로 탑재부(차고형 공간)에 들어가 주차**. 팔레트 삽입이 아니다.
기준점 x_ref = 포크 끝(카메라 앞 `CAM_TO_REF_M`), 좌우 여유 `LAT_TOL_M` 0.030 은 실물값.

골격 (바꾸지 않는다)
────────────────────────────────────────────────────────────────────────────
· 사이드스텝·곡선주행 없음 → **태그를 화면 중앙 가까이 두고 소각 대각 접근**
· 가이드 = **대각 다리 → 태그 겨냥 회전 → 블라인드 직진**. 비용함수·N-step 롤아웃 없음
· 구역 FAR → NEAR → FINAL **단조**(역행 금지). 전환 판정은 VERIFY(정지)에서만
· 명령 래치는 **DECIDE→EXECUTE 전이 한 곳**. EXECUTE 중에 낼 수 있는 명령은 stop 뿐
· 모든 CAN 송신은 주입받은 `SafeCanTx` 하나를 통해서만. byte4(포크)는 건드리지 않는다

이 파일이 지키는 한 줄
────────────────────────────────────────────────────────────────────────────
**모르면 움직이지 않고, 못 미더우면 성공이라고 하지 않는다.**
9/7 의 좌우 왕복은 σ 가 큰 lateral 을 그대로 따라간 결과였다 — 그래서 모든 판정은
3분할(행동 / 수용 / 판정불가)로 하고, `k·σ > T/2` 면 **움직이지 않고** Tier 1 로 간다.
성공(DONE)은 수용식을 관측으로 통과했을 때만. 애매하면 DONE_UNVERIFIED 아니면 ABORT.

쓰는 법 (통합 `tools/dock.py`)
────────────────────────────────────────────────────────────────────────────
    fsm = DockFSM(calibs=calib.load_all(), dyn=dynamics, tx=tx, logger=logger,
                  assume_calib=args.assume_calib, final_anyway=args.final_anyway)
    fsm.arm(gate_ok=decision.allow, why="SPACE")       # 캘리브 게이트 + 템플릿 검사 결과
    for ...:
        est = estimator.update(obs)
        cmd = fsm.step(est, dyn, gyro_deg=gyro.angle_deg)
        ...                                            # tx 를 주입했으면 **여기서 보낼 것 없다**
        logger.frame(..., phase=fsm.phase.value, zone=int(fsm.zone))

`tx` 를 주입하면 이 파일이 직접 `set_movement`/`stop` 을 부른다(송신 창구를 하나로 두려고).
그때 계약 §6 실행루프의 `tx.set_movement(...)`·`tx.stop(...)` 두 줄은 **빼야 한다** —
안 빼면 같은 명령이 두 번 래치된다. `tx=None` 이면 반환된 Command 를 보고 통합이 보낸다.
"""
from __future__ import annotations

import math
import statistics
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum, IntEnum

from config import control as C
from config import detection as D
from .failure import (AbortKeeper, AbortReason, FailureCode, FailureMonitor,
                      FAILURE_MODES, Tier, can_health, rotation_consistency,
                      straight_consistency)

# ── config 다리 ─────────────────────────────────────────────────────────────
#: 계약 §8 이 "통합 담당이 넣는다" 고 한 두 값. 아직 없으면 여기 기본값으로 돌고
#: 출발 로그에 "config 에 없음" 을 남긴다 (남의 파일을 고치지 않으려고 이렇게 한다).
_CFG_PENDING = {}


def _cfg(name, default, why):
    v = getattr(C, name, None)
    if v is None:
        _CFG_PENDING[name] = "%s (임시 %s)" % (why, default)
        return default
    return v


# ── 코드 내부 상수 ──────────────────────────────────────────────────────────
# 전부 "차량·현장이 바뀌어도 사람이 고칠 값이 아니다" — 그래서 config 가 아니다.
K_SIGMA = 2.0                 # 수용·행동 판정의 k (계약 §3.4 "k=2, config 아님")
MIN_OBS_FRAMES = 5            # OBSERVE 탈출 최소 프레임 (plan 3-1 "최소 프레임 코드 내부")
VERIFY_FRAMES = 30            # 정지 후 확인 프레임 (v2 MEASURE_FRAMES 를 코드 내부로)
VERIFY_CAP_S = 3.0            # 30 프레임을 못 모아도 넘어가는 시간 캡
WAIT_CAP_S = 4.0              # Tier 0 WAIT 상한. 넘으면 Tier 1 (FM6)
TIER1_MAX = 3                 # Tier 1 재배치 횟수 상한. 넘으면 Tier 2 → Tier 3
REACQUIRE_AFTER_S = 1.0       # 태그 실종이 이만큼 이어지면 REACQUIRE
AMBIGUOUS_CAP_S = 10.0        # AMBIGUOUS 지속 → FALLBACK (plan 4-4 고장주입 10 s)
SIGN_RESID_DEG = 0.2          # (I1) 잔차 상한 [°] (계약 §2.4)
SIGN_RESID_N = 3              # 그 초과가 이만큼 **연속**이어야 ABORT (프레임 잡음 지속성)
CAN_BAD_FRAMES = 3            # CAN 건강 불량이 이만큼 **연속**이어야 EXECUTE 를 끊는다
ROT_MAX_DEG = 30.0            # 한 번에 도는 최대 각 [°]
ROT_MAX_DEG_DEGRADED = 15.0   # 가정값으로 돌 때 (계약 §5.4)
ROT_WRONG_WAY_DEG = 5.0       # 반대로 이만큼 돌면 부호 사고 → 즉시 정지
LEG_MAX_M = 2.0               # 한 다리 최대 [m]
LEG_MAX_M_DEGRADED = 1.0      # 가정값으로 돌 때 (계약 §5.4)
BLIND_CHUNK_M = 1.0           # S(T) 가 가정값이면 블라인드 다리를 이만큼씩 쪼갠다 (계약 §5.1)
BLIND_CHUNK_M_DEGRADED = 0.30 # v97 의 m/s 가 미측정인 동안의 한 다리 상한 [m].
                              # 근거: 누적 예산이 v_upper(=v67 0.28 m/s)로 걸리므로 한 다리가
                              # 길수록 '실제로 간 거리' 의 불확실 구간이 커진다.
                              # first_run creep 이 v97 을 재면 BLIND_CHUNK_M 으로 돌아간다.
COLLISION_GUARD_M = 0.15      # 블라인드 주행 예산을 **충돌점 기준**으로 잡을 때의 여유 [m].
                              # 포크 끝이 태그면에 닿는 자리는 카메라 x = CAM_TO_REF_M 이다.
                              # 예산을 (진입 x̂ − CAM_TO_REF_M − 이 값) 으로 잡으면 v97 이
                              # 상한만큼 빨라도 포크 끝이 태그면 앞 이 거리에서 반드시 선다.
                              # 예산을 '목표까지' 로 잡으면 v 불확실도만큼 늘 못 가고,
                              # '충돌점까지' 로 잡으면 목표에는 닿으면서 벽은 안 닿는다.
BACK_MAX_M = 0.30             # 후진 다리 상한 [m] — 후진 캘리브가 없다 (계약 §5.1)
TAG_CUT_GUARD_M = 0.20        # T 를 태그 컷보다 이만큼 앞에 둔다 [m]
J_MIN_RUN_M = 1.00            # T 까지 남은 축 거리가 이보다 짧으면 J 겨냥을 접고 **태그를 겨냥**한다.
                              # 근거: J 겨냥각의 이득이 dβ*/dℓ ≈ run/(run²+ℓ²) 라 run 이
                              # 짧아지면 ℓ 잡음이 각도로 증폭된다 — run 0.6 m 에서 ℓ 5 cm
                              # 흔들림이 β* 를 4.6° 움직여 최소 회전각 2° 를 넘고, 그래서
                              # 좌우로 되튄다(FM3). run 1 m 면 그 증폭이 데드존 아래다.
                              # 200회 몬테카를로(2026-09-21): 0.5 → 1.0 으로 올리니
                              # abort 56→38, envelope 15→2, 반전 56→43, |e_l| 137→127 mm,
                              # 고장주입 필수 4종이 전부 통과(2 실패 → 0). 1.5 는 반전이
                              # 다시 51 로 늘어 1.0 을 골랐다.
STOP_CONFIRM_CAP_S = 0.30     # stop payload 가 나갔는지 기다리는 상한 [s]
NO_RESP_EXTRA_S = 0.50        # 죽은시간 + 이 값 안에 안 움직이면 FM4 무응답 (하한)
MOVE_EVIDENCE_M = 0.03        # "움직였다" 로 인정할 x̂ 변화 [m]. 관측 잡음(σ_x ~1 cm) 위.
                              # 무응답 창은 이 증거가 **실제로 쌓일 만큼** 길어야 한다 —
                              # 97(0.12 m/s)에서 0.5 s 면 2 cm 밖에 안 움직여 멀쩡한 차를
                              # 무응답으로 잡는다. 그래서 창을 v 로부터 유도한다.
START_DELAY_GAIN = 2.0        # 무응답 판정에 쓰는 죽은시간 배수. 97 은 데드밴드 바로 위라
                              # 한 번에 안 붙는 일(stiction)이 있고 그 추가 지연은 ★미측정
                              # (시뮬 모델 0.3~1.0 s). 늦게 붙는 차를 '무응답' 으로 잡으면
                              # 내일 실험이 첫 다리에서 죽는다 — 배수로 덮고 창은 길게 둔다.
SIGMA_ROLL_M = 0.019          # 롤 1° = 19 mm (계약 §5.2). roll 미측정이면 σ 에 더한다
BETA_VIS_USE = 0.80           # 가시 한계 β 를 이 비율까지만 쓴다(여유)
SETTLE_FRAMES = 9             # 정지 판정에 보는 프레임 수 (0.3 s @30 fps = SETTLE_HOLD_S)
SETTLE_V_FLOOR = 0.02         # 정지 판정 속도 바닥 [m/s] (동역학이 없을 때만)
SETTLE_W_FLOOR = 1.0          # 정지 판정 각속도 바닥 [°/s] (동역학이 없을 때만)

# 동역학(B팀)이 없을 때 쓰는 가정값. 출처는 계약 §5.1 "없을 때 가정값" 열 = 9/7 실측.
_ASSUMED = {"tau_fwd_s": 0.50, "tau_rot_s": 0.18, "v67": 0.28, "v97": 0.12,
            "t_dead_fwd": 1.00, "t_dead_rot": 0.85,
            "min_fwd_67": 0.30, "min_fwd_97": 0.12, "min_turn_deg": 2.0,
            "degrade_gain": 1.5}       # 가정값이면 정지거리를 이만큼 부풀려 일찍 선다


# ── enum (계약 §4.1) ────────────────────────────────────────────────────────

class Phase(Enum):
    IDLE = "idle"                 # 아직 출발 안 함(SPACE 대기·게이트 검사)
    OBSERVE = "observe"           # 프레임을 모아 판단 가능 상태를 만든다
    DECIDE = "decide"             # 명령을 고르고 **래치**한다 (명령 결정은 여기서만)
    EXECUTE = "execute"           # 명령 유지. 여기서 낼 수 있는 새 명령은 stop 뿐
    STOPPING = "stopping"         # stop 프레임 송신, 코스팅 중
    SETTLE = "settle"             # 실제로 멎었는지 **판정**(가시 v̂ / 블라인드 IMU)
    VERIFY = "verify"             # 정지 상태에서 30프레임 확인 + 구역 전환 판정
    REACQUIRE = "reacquire"       # 태그 실종 — 예측 β 방향으로 1회 회전(DECIDE 경유)
    FALLBACK = "fallback"         # AMBIGUOUS 지속 — oblique 로 재획득(DECIDE 경유)
    DONE = "done"                 # 관측으로 확인된 성공
    DONE_UNVERIFIED = "done_dr"   # 정지했으나 카메라로 확인 못 함(추측항법) — 기본값
    ABORT = "abort"               # stop 프레임 + heartbeat 지속 송신 + 사람 호출


class Zone(IntEnum):              # 단조. 역행 금지(FM7)
    FAR = 0
    NEAR = 1
    FINAL = 2


class Primitive(Enum):
    NONE = "none"
    ROTATE = "rotate"                 # 제자리 회전. magnitude [°] + = 반시계
    FORWARD = "forward"               # 태그 보며 전진(67 또는 97)
    FORWARD_BLIND = "forward_blind"   # 태그 컷 뒤 마지막 직진(97 고정)
    BACKWARD = "backward"             # 재접근 후진(97 고정)
    DOGLEG = "dogleg"                 # 포락선 밖 FAR 한정 1회(97, ≤10 s)
    HOLD = "hold"                     # 아무것도 안 하고 관측만


class Movement(str, Enum):            # 값은 can_tx.SAFE_MOVEMENTS 문자열과 **완전히 같다**
    STOP = "stop"
    FORWARD = "forward"               # byte2 = 67
    FORWARD_SLOW = "forward_slow"     # byte2 = 97
    BACKWARD = "backward"             # byte2 = 187
    ROTATE_CCW = "rotate_ccw"         # byte1 = 147  (좌, +)
    ROTATE_CW = "rotate_cw"           # byte1 = 107  (우, −)


class StopReason(Enum):               # EXECUTE 를 끝낸 이유 (전부 can.jsonl 에 남는다)
    TARGET = "target"                 # 종료식 충족(정상)
    STALE = "stale"                   # 프레임이 150 ms 이상 묵음
    GYRO_DEAD = "gyro_dead"
    TAG_CUT = "tag_cut"               # 가시성 픽셀 여유 술어
    HARD_CAP = "hard_cap"             # 예측 정지점이 안전선을 넘음(횡 드리프트 포함)
    WRONG_WAY = "wrong_way"           # 회전이 반대로
    TIMEOUT = "timeout"
    DEADMAN = "deadman"               # SafeCanTx 가 먼저 끊음
    NO_RESPONSE = "no_response"       # 명령 창 안에 움직임이 없음(FM4)
    OPERATOR = "operator"


TERMINAL = (Phase.DONE, Phase.DONE_UNVERIFIED, Phase.ABORT)

ALLOWED = {
    Phase.IDLE:      {Phase.OBSERVE, Phase.ABORT},
    Phase.OBSERVE:   {Phase.DECIDE, Phase.REACQUIRE, Phase.FALLBACK, Phase.ABORT, Phase.OBSERVE},
    Phase.DECIDE:    {Phase.EXECUTE, Phase.VERIFY, Phase.OBSERVE, Phase.ABORT},
    Phase.EXECUTE:   {Phase.STOPPING, Phase.ABORT},
    Phase.STOPPING:  {Phase.SETTLE, Phase.ABORT},
    Phase.SETTLE:    {Phase.VERIFY, Phase.ABORT},
    Phase.VERIFY:    {Phase.OBSERVE, Phase.DONE, Phase.DONE_UNVERIFIED, Phase.ABORT},
    Phase.REACQUIRE: {Phase.DECIDE, Phase.OBSERVE, Phase.ABORT},
    Phase.FALLBACK:  {Phase.DECIDE, Phase.OBSERVE, Phase.ABORT},
    Phase.DONE: set(), Phase.DONE_UNVERIFIED: set(), Phase.ABORT: set(),
}


@dataclass(frozen=True)
class Command:
    """DECIDE 가 래치하는 명령 하나 (계약 §4.2)."""
    primitive: Primitive
    movement: str            # Movement 값. SafeCanTx.set_movement 에 그대로 들어간다
    magnitude: float         # ROTATE [°] (+반시계) / 직진 [m] (+전진, 후진도 + 로)
    end_rule: str            # "camera_x" | "gyro_angle" | "time"
    end_value: float         # 목표 x_m [m] / 목표 Δψ [°] / 명령시간 [s]
    level: int               # 67 | 97 (회전은 byte1=±20 고정이라 그때의 직진 단만 적는다)
    expect: object           # Prediction (정지거리·정지각). 사후 잔차 계산에 쓴다
    timeout_s: float
    zone: Zone
    tier: Tier
    est_seq: int
    t_decide: float
    why: str

    def as_row(self):
        return {"primitive": self.primitive.value, "movement": self.movement,
                "magnitude": self.magnitude, "end_rule": self.end_rule,
                "end_value": self.end_value, "level": self.level,
                "timeout_s": self.timeout_s, "zone": int(self.zone),
                "tier": int(self.tier), "est_seq": self.est_seq,
                "t_decide": self.t_decide, "why": self.why}


@dataclass(frozen=True)
class Judgment:
    """3분할 판정 (plan 3-5 ②). 겹치면 수용 우선."""
    e: float
    sigma: float
    tol: float
    dq: float
    act: bool
    accept: bool
    undecidable: bool

    def as_row(self):
        return {"e": self.e, "sigma": self.sigma, "tol": self.tol, "dq": self.dq,
                "act": self.act, "accept": self.accept, "undecidable": self.undecidable}


def judge(e, sigma, tol, dq, k=K_SIGMA):
    """|e| > max(kσ, δ_q) 행동 / |e| + kσ ≤ T 수용 / kσ > T/2 판정불가.

    **긍정형으로만 쓴다** — NaN 이면 모든 비교가 False 라 '수용' 이 절대 안 난다.
    값이 없으면 판정불가로 떨어뜨린다(조용히 통과시키지 않는다).
    """
    if not (_fin(e) and _fin(sigma) and _fin(tol)):
        return Judgment(e, sigma, tol, dq, False, False, True)
    dq = dq if _fin(dq) else 0.0
    accept = (abs(e) + k * sigma <= tol)
    if accept:
        return Judgment(e, sigma, tol, dq, False, True, False)
    return Judgment(e, sigma, tol, dq,
                    abs(e) > max(k * sigma, dq), False, k * sigma > tol / 2.0)


# ── 작은 도구 ───────────────────────────────────────────────────────────────

def _num(v, default=float("nan")):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return default
    return f if math.isfinite(f) else default


def _fin(v):
    return isinstance(v, (int, float)) and math.isfinite(v)


def _flag(obj, name, default=False):
    return bool(getattr(obj, name, default))


def _med(vals, default=float("nan")):
    vals = [v for v in vals if _fin(v)]
    return statistics.median(vals) if vals else default


def parallelogram_margin(e_l, e_h_deg, dock_depth_m=None, tol_m=None):
    """성공 판정 여유 m = c − max(|e_l|, |e_l − L·tan e_h|)  (plan 4-3).

    m ≥ 0 이면 허용영역 안. 다이아몬드(|e_l| + L·tan|e_h| ≤ c)는 절반짜리 진부분집합이라
    쓰지 않는다 — 그걸 쓰면 e_l = +c 에서 진짜 허용각 1.7° 를 0° 로 과보수한다.
    """
    L = C.DOCK_DEPTH_M if dock_depth_m is None else dock_depth_m
    c = C.LAT_TOL_M if tol_m is None else tol_m
    if not (_fin(e_l) and _fin(e_h_deg)):
        return float("nan")
    far = e_l - L * math.tan(math.radians(e_h_deg))
    return c - max(abs(e_l), abs(far))


# ── B팀 동역학 안전 래퍼 ────────────────────────────────────────────────────

@dataclass
class _Pred:
    """계약 §3.1 Prediction 과 같은 얼굴. B팀 객체가 없을 때만 쓴다."""
    kind: str
    value: float
    sigma: float = float("nan")
    q95: float = float("nan")
    level: int = 97
    cell: str = ""
    source: str = "assumed"
    provisional: bool = True
    note: str = ""


class _DynView:
    """`Dynamics`(B팀) 를 감싸 **없거나 모자라도 돌게** 한다.

    내일 아침엔 `src/models/dynamics/` 자체가 없을 수 있다. 그때 상태기계가 import 에서
    죽으면 실험이 못 돈다 — 그래서 계약 §3.2 API 를 그대로 부르되, 없으면 계약 §5.1
    "없을 때 가정값" 으로 떨어지고 **그 사실을 degraded 로 올린다**(= 97 강제).
    """

    def __init__(self, dyn, calib_dyn=None, log=None):
        self.d = dyn
        self.cal = calib_dyn
        self.log = log
        self.missing = set()
        self._degraded = dyn is None
        if dyn is None and log:
            log("  !! 동역학(B팀) 없음 — 계약 §5.1 가정값으로 돈다. 전진은 97 만.")

    # -- 내부 -------------------------------------------------------------
    def _call(self, name, *a, **kw):
        fn = getattr(self.d, name, None)
        if fn is None:
            if name not in self.missing:
                self.missing.add(name)
                if self.log:
                    self.log("  !! 동역학에 %s() 가 없다 — 가정값으로 대신한다" % name)
            return None
        try:
            return fn(*a, **kw)
        except Exception as exc:
            if name not in self.missing:
                self.missing.add(name)
                if self.log:
                    self.log("  !! 동역학 %s() 실패(%s) — 가정값으로 대신한다"
                             % (name, type(exc).__name__))
            return None

    def _get(self, dotted, default):
        if self.cal is None:
            return default
        v = self.cal.get(dotted, None)
        return default if v is None else v

    # -- 상태 -------------------------------------------------------------
    @property
    def degraded(self):
        v = getattr(self.d, "degraded", None)
        return self._degraded or bool(v) or bool(self.missing)

    @property
    def force_slow(self):
        """True 면 전진은 forward_slow(97)만. 가정값으로 도는 동안은 **무조건 True**."""
        v = getattr(self.d, "force_slow", None)
        return True if self.degraded else bool(v)

    def v_of(self, level):
        return self._get("v_mps.%d.value" % level,
                         _ASSUMED["v67"] if level == 67 else _ASSUMED["v97"])

    def tau_of(self, level, direction="fwd"):
        return self._get("tau_eff_s.%d_%s.value" % (level, direction), _ASSUMED["tau_fwd_s"])

    # -- 직진 -------------------------------------------------------------
    def stop_distance(self, v, level=67, direction="fwd", dt_ctrl_s=0.0):
        p = self._call("stop_distance", v, level=level, direction=direction,
                       dt_ctrl_s=dt_ctrl_s)
        if p is not None:
            return p
        v = abs(_num(v, 0.0))
        d = self.tau_of(level, direction) * v + v * max(0.0, dt_ctrl_s) / 2.0
        if self.degraded:
            d *= _ASSUMED["degrade_gain"]          # 가정값이면 일찍 선다 (계약 §5.4)
        return _Pred("stop_forward", d, q95=d * 1.3, level=level,
                     cell="%d_%s" % (level, direction))

    def should_stop_forward(self, x_rem, v, level, dt_ctrl_s):
        r = self._call("should_stop_forward", x_rem, v, level, dt_ctrl_s)
        if r is not None:
            return bool(r)
        return _fin(x_rem) and x_rem <= self.stop_distance(v, level, "fwd", dt_ctrl_s).value

    def command_time_for(self, dist_m, level):
        p = self._call("command_time_for", dist_m, level)
        if p is not None:
            return p
        v = max(self.v_of(level), 0.02)
        t_dead = self._get("tau_start_s.fwd.value", _ASSUMED["t_dead_fwd"])
        coast = self.stop_distance(v, level, "fwd").value      # 명령 끊은 뒤 더 가는 거리
        t = t_dead + max(0.0, _num(dist_m, 0.0) - coast) / v
        return _Pred("leg_time", t, level=level, note="fwd 가정값(S(T) 없음, 코스팅 뺌)")

    def distance_for_command(self, sec, level=97, coasted=True):
        """명령시간 → 예상 거리 [m]. 블라인드 다리 **사후 정산**에 쓴다."""
        p = self._call("distance_for_command", sec, level)
        if p is not None:
            v = _num(getattr(p, "value", p))
            if _fin(v):
                return max(0.0, v)
        v = max(self.v_of(level), 0.02)
        t_dead = self._get("tau_start_s.fwd.value", _ASSUMED["t_dead_fwd"])
        d = max(0.0, _num(sec, 0.0) - t_dead) * v
        if coasted:
            d += self.stop_distance(v, level, "fwd").value
        return d

    #: 실측 v 에 얹는 상한 여유 (게인 CV 를 감싸는 값. 코드 내부 상수)
    V_UPPER_MARGIN = 1.3

    def v_upper(self, level=97):
        """그 단의 속도 **상한** [m/s] — 블라인드 거리 안전장치가 쓰는 보수값.

        실측이 있으면 실측×1.3. 없으면 **v67** 을 쓴다 — byte2=97 은 67 보다 편향이
        작으니 97 이 67 보다 빠를 수 없다(물리 상한). 미측정 동안은 이 상한 때문에
        블라인드를 **덜 가고 멈춘다**. 그게 벽을 뚫는 것보다 낫고, first_run creep 이
        v97 을 재면 그대로 풀린다.
        """
        m = self._get("v_mps.%d.value" % int(level), None)
        if m is not None and _fin(_num(m)) and float(m) > 0.0:
            return float(m) * self.V_UPPER_MARGIN
        return max(self.v_of(67), self.v_of(level))

    # -- 회전 -------------------------------------------------------------
    def should_stop_rotate(self, theta_rem_deg, omega_dps, side, accelerating):
        r = self._call("should_stop_rotate", theta_rem_deg, omega_dps, side, accelerating)
        if r is not None:
            return bool(r)
        tau = self._get("tau_r_s.%s.value" % side, _ASSUMED["tau_rot_s"])
        if self.degraded:
            tau *= _ASSUMED["degrade_gain"]
        return _fin(theta_rem_deg) and theta_rem_deg <= abs(_num(omega_dps, 0.0)) * tau

    def rotate_timeout_s(self, deg, omega_dps):
        r = self._call("rotate_timeout_s", deg, omega_dps)
        if r is not None:
            return float(r)
        w = max(abs(_num(omega_dps, 0.0)), 4.0)
        return _ASSUMED["t_dead_rot"] + 2.0 * abs(_num(deg, 0.0)) / w + 2.0

    # -- 최소 증분 --------------------------------------------------------
    def min_forward_m(self, level):
        r = self._call("min_forward_m", level)
        if r is not None:
            return float(r)
        return self._get("min_inc.fwd_%d_m" % level,
                         _ASSUMED["min_fwd_67"] if level == 67 else _ASSUMED["min_fwd_97"])

    def min_turn_deg(self, side="L"):
        r = self._call("min_turn_deg", side)
        if r is not None:
            return float(r)
        sig = self._get("rot_closed.%s.sigma_deg" % side, 1.0)
        return max(2.0 * sig, self._get("min_inc.turn_deg", _ASSUMED["min_turn_deg"]))

    # -- 정지 판별 --------------------------------------------------------
    def settle_visible(self, v_hist, omega_hist):
        r = self._call("settle_visible", v_hist, omega_hist)
        if r is not None:
            return bool(r)
        v = [abs(x) for x in v_hist if _fin(x)][-5:]
        w = [abs(x) for x in omega_hist if _fin(x)][-5:]
        return bool(v) and max(v) <= SETTLE_V_FLOOR and (not w or max(w) <= SETTLE_W_FLOOR)

    def settle_blind(self, imu_window):
        """IMU 원시행(dict) 창이면 B팀 판정을, ω 실수 창이면 **폴백**을 쓴다.

        예전에는 실수 창을 그대로 B팀에 넘겨 `isinstance(dict)` 필터에 전부 걸러
        False 가 돌아왔고, `r is not None` 때문에 폴백이 죽은 코드였다.
        """
        rows = list(imu_window or [])
        dicts = [r for r in rows if isinstance(r, dict)]
        if dicts:
            r = self._call("settle_blind", dicts)
            if r is not None:
                return bool(r)
        w = [abs(x) for x in rows if _fin(x)][-10:]
        return bool(w) and max(w) <= SETTLE_W_FLOOR

    def settle_timeout_s(self, v, level):
        r = self._call("settle_timeout_s", v, level)
        if r is not None:
            return float(r)
        return self.tau_of(level) * 3.0 + abs(_num(v, 0.0)) / 0.3 + 0.5

    def report(self, log=print):
        log("  동역학: %s%s" % ("B팀 객체" if self.d is not None else "**없음(가정값)**",
                              "  [강등]" if self.degraded else ""))
        if self.missing:
            log("     없는 API: %s" % ", ".join(sorted(self.missing)))


# ── 상태기계 ────────────────────────────────────────────────────────────────

class DockFSM:
    """수렴 제어 상태기계 하나. 하드웨어는 **주입받는다**(테스트에 가짜를 넣을 수 있게)."""

    def __init__(self, calibs=None, dyn=None, tx=None, logger=None, log=print,
                 assume_calib=False, final_anyway=False, clock=None):
        self.calibs = calibs or {}
        self.tx = tx
        self.logger = logger
        self.log = log or (lambda *a, **k: None)
        self.assume_calib = bool(assume_calib)
        self.final_anyway = bool(final_anyway)
        self.clock = clock or time.time
        self.monitor = FailureMonitor(log=self.log, logger=logger)
        self.keeper = AbortKeeper(tx, log=self.log, logger=logger)
        self.dyn = _DynView(dyn, self.calibs.get("dynamics"), log=self.log)

        # -- 캘리브에서 읽는 값 (없으면 계약 §5 의 "없을 때 가정값") --------
        per = self.calibs.get("perception")
        dyc = self.calibs.get("dynamics")
        g = (lambda c, k, d: d if c is None else (c.get(k, d) if c.get(k, None) is not None else d))
        self.delta_deg = g(per, "dynamic.cam_yaw_offset_deg.value", getattr(D, "CAM_YAW_OFFSET_DEG", 0.0))
        self.sigma_delta_deg = g(per, "dynamic.cam_yaw_offset_deg.sigma", 1.0)
        self.x_off_m = g(per, "dynamic.x_off_m.value", 0.0)
        self.A_m = g(per, "dynamic.A_m.value", 0.0)
        self.sigma_A_m = g(per, "dynamic.A_m.sigma", 0.5)
        self.theta_min_deg = g(per, "static.theta_min_deg.value", 15.0)
        # 가시 한계 β 는 좌/우가 다를 수 있다(비대칭 장착). 한쪽 값을 양쪽에 쓰면
        # 한쪽에서 태그를 잃고(REACQUIRE→ABORT) 다른 쪽은 갈 수 있는 자세를 거부한다.
        # 부호 규약: rotate_ccw(L) 로 돌면 β 가 **줄고**(음수 쪽), rotate_cw(R) 면 커진다.
        self.beta_vis_L = abs(g(per, "static.beta_vis_deg.L", 28.0))   # β < 0 쪽 한계
        self.beta_vis_R = abs(g(per, "static.beta_vis_deg.R", 28.0))   # β > 0 쪽 한계
        self.beta_vis_deg = min(self.beta_vis_L, self.beta_vis_R)      # 방향을 모를 때의 보수값
        self.pitch_px = g(per, "static.pitch_px.value", g(per, "static.pitch_px", 24.0))
        # 롤: **보정이 코드 어디에도 없다**(plan 1-2 미구현). 그래서 roll 이 캘리브에
        # 있다고 σ 바닥을 그냥 치우면 h·φ(≈19 mm/°) 계통오차는 그대로 남고 자신감만
        # 올라 wrong-DONE 이 난다. 규칙은 하나 — **안 고치는 바이어스는 통째로 σ 다**:
        #   미측정      → 1° 분(19 mm)
        #   측정 φ      → h·|sin φ| (φ=0 이면 고칠 것이 없으니 0, φ=2° 면 38 mm 라
        #                 kσ > T/2 가 되어 자동으로 '판정불가' → Tier 1 이 된다)
        roll = None if per is None else per.get("static.roll_deg.value", per.get("static.roll_deg", None))
        self.roll_deg = _num(roll) if roll is not None else float("nan")
        h_tag = _num(g(per, "dynamic.h_tag_cam_m.value", 1.10), 1.10)
        if _fin(self.roll_deg):
            self.sigma_roll_m = abs(h_tag * math.sin(math.radians(self.roll_deg)))
        else:
            self.sigma_roll_m = SIGMA_ROLL_M
        self.sigma_theta_deg = max(g(dyc, "rot_closed.L.sigma_deg", 1.0),
                                   g(dyc, "rot_closed.R.sigma_deg", 1.0))
        self.tag_cut_m = g(dyc, "tag_cut.forward_m", 3.3)
        self.tag_cut_px = g(dyc, "tag_cut.margin_px", 60.0)

        # -- 기하 (파생값. 상수가 아니다) -----------------------------------
        self.x_stop_cam = C.CAM_TO_REF_M + C.STANDOFF_M          # 계약 §8 X_STOP_CAM_M
        self.x_T = max(self.x_stop_cam + self.dyn.min_forward_m(97),
                       self.tag_cut_m + TAG_CUT_GUARD_M)         # 마지막 관측점 T
        self.budget_s = _cfg("DOCK_TIME_BUDGET_S", 180.0, "도킹 시간 상한 [s] (plan 3-7)")
        self.fwd_tol_m = _cfg("FWD_TOL_M", 0.10, "종방향 허용오차 [m] (plan 4-3)")

        # -- 상태 -----------------------------------------------------------
        #: 사람이 정한 전진 단 상한(67 = 제한 없음, 97 = 저속만). 통합이 덮어쓴다
        self.level_cap = 67
        self.phase = Phase.IDLE
        self.zone = Zone.FAR
        self.tier = Tier.NONE
        self.stop_reason = None
        self.abort_reason = None
        self.command = None
        self.armed = False
        self.gate_ok = None
        self.result_note = ""
        self.transitions = []         # (t, from, to, why)
        self.illegal = 0
        self.n_steps = 0
        self.t_start = None
        self.t_phase = None
        self.last_error = {}          # 마지막 판정값 (로그·자기검증)

        self._obs_n = 0
        self._vbuf = []
        self._t_verify0 = None
        self._wait_t0 = None
        self._wait_sigma0 = None
        self._tier1_n = 0
        self._macro = []              # 다음에 낼 프리미티브 예약(dog-leg·블라인드 쪼개기)
        self._forced = None
        self._pre = None              # 프리미티브 직전 스냅샷 (사후 일관성 ⓪)
        self._consistency_ok = True
        self._reanchor_ok = True
        self._aimed = False
        self._final_remain_m = None
        self._final_budget_m = None   # FINAL 전체의 **절대 주행 예산** [m]
        self._final_done = False      # 더 낼 블라인드 명령이 없다
        self._final_gate = None       # "accept" | "operator"
        self._reacq_n = 0
        self._fallback_n = 0
        self._dogleg_n = 0
        self._lost_since = None
        self._amb_since = None
        self._sign_bad_n = 0
        self._i1_obs = float("nan")   # 원관측 (I1) 잔차. step(i1_obs_deg=) 로 들어온다
        self._last_corr_sign = 0
        self._alt_n = 0
        self._rot_denied = False
        self._gyro_deg = 0.0
        self._gyro_ext = None
        self._t_prev = None
        self._v_hist = deque(maxlen=30)
        self._w_hist = deque(maxlen=60)
        self._margin_hist = deque(maxlen=10)
        self._moved = False
        self._moved_x0 = float("nan")     # 움직임 판정 기준 x̂ (명령 직후)
        self._moved_g0 = float("nan")     # 같은 목적의 자이로 각
        self._x_win = deque(maxlen=5)     # 움직임 판정용 x̂ 창(중앙값)
        self._base_trips = 0
        self._can_bad_n = 0
        self._stop_req_t = None
        self._imu_win = []                # SETTLE 에 넘길 IMU 원시행(dict)
        self._settled_confirmed = True    # 마지막 정지를 관측으로 확인했나
        self._fm11_seen = 0               # 남이 보낸 0x1E3 프레임 수(FM11)
        self._blind_done_m = 0.0          # 블라인드 다리에서 **실제로 간** 거리 [m]

    # ── 출발 ────────────────────────────────────────────────────────────
    def arm(self, gate_ok=True, why="operator"):
        """출발 허가. gate_ok = 캘리브 게이트 ∧ CAN 템플릿 검사 ∧ 도메인 GLOBAL (통합이 판정)."""
        self.gate_ok = bool(gate_ok)
        self.armed = True
        self.t_start = self.clock()
        self.monitor.unmitigated_notice(t=self.t_start)          # FM8b 미완화 리스크
        if _CFG_PENDING:
            for k, v in _CFG_PENDING.items():
                self.log("  !! config.control 에 %s 가 없다 — %s. 통합 담당이 넣을 것(계약 §8)" % (k, v))
        if self.dyn.degraded or not self.gate_ok:
            self.log("  !! **가정값 사용 중** — 전진 97 한정, 다리 ≤%.1f m, 회전 ≤%.0f°, "
                     "FINAL 은 --final-anyway 없이는 못 들어간다"
                     % (LEG_MAX_M_DEGRADED, ROT_MAX_DEG_DEGRADED))
            self.log("     예외 하나: **Tier 1 재배치 후진은 byte2=187(전속)** 이다 — "
                     "저속 후진 단이 없다. 다리 ≤%.2f m 로 묶어 둔다" % BACK_MAX_M)
        if self.final_anyway:
            self.log("  !! --final-anyway : 수용식을 못 넘어도 FINAL 로 들어간다. "
                     "결과는 **무조건 DONE_UNVERIFIED** (줄자로 재야 한다)")
        if self.logger is not None:
            self.logger.event("arm", gate_ok=self.gate_ok, why=why,
                              assume_calib=self.assume_calib, final_anyway=self.final_anyway,
                              degraded=self.dyn.degraded, x_T=self.x_T,
                              x_stop_cam=self.x_stop_cam, budget_s=self.budget_s,
                              cfg_pending=sorted(_CFG_PENDING))
        return self

    def release(self, why="조이스틱 개입"):
        """FM11 — 사람이 잡았다. movement 송신을 놓고 heartbeat 만 남긴다."""
        self.monitor.note(FailureCode.FM11_DUAL_SOURCE, why, t=self.clock())
        self._abort(AbortReason.OPERATOR, why)
        self.keeper.release(why, t=self.clock())

    # ── 한 스텝 ─────────────────────────────────────────────────────────
    def step(self, est, dyn=None, gyro_deg=None, i1_obs_deg=None, imu_rows=None):
        """프레임 하나 분의 상태기계. 새 명령을 **래치한 그 스텝에만** Command 를 돌려준다.

        gyro_deg    자이로 누적각 [°]. 안 주면 est.gyro_deg → est.omega_dps 적분 순으로 쓴다.
        i1_obs_deg  **원관측** (I1) 잔차 [°] = `Estimator.identity_obs_deg`
                    (β 픽셀 경로 vs ℓ·x·ψ PnP 경로). 발행 상태의 (I1) 은 ℓ 을 β 로 세우는
                    구성상 건강할 때 늘 ~0 이라, 이 둘을 같이 봐야 "배선 부호가 틀렸다"
                    와 "PnP 2중해가 거짓 해로 잠겼다" 가 갈린다. 안 주면 옛 동작 그대로.
        """
        self.n_steps += 1
        self._i1_obs = _num(i1_obs_deg) if i1_obs_deg is not None else float("nan")
        if imu_rows:
            # 블라인드 정지 판정(plan 2-3)은 IMU 원시행이 있어야 돈다. 통합이 넘겨 주면
            # 창을 들고 있다가 SETTLE 에서 쓴다(최근 2 s 면 충분 — SETTLE_HOLD_S 0.3 s).
            self._imu_win.extend(r for r in imu_rows if isinstance(r, dict))
            if len(self._imu_win) > 1200:
                self._imu_win = self._imu_win[-1200:]
        if dyn is not None and dyn is not self.dyn.d:
            self.dyn = _DynView(dyn, self.calibs.get("dynamics"), log=self.log)
        try:
            return self._step(est, gyro_deg)
        except Exception as exc:                 # 상태기계가 죽어도 차는 서 있어야 한다
            self.log("  !! 상태기계 내부 예외: %s: %s" % (type(exc).__name__, exc))
            self._abort(AbortReason.INTERNAL, "%s: %s" % (type(exc).__name__, exc))
            return None

    # -- 본체 -------------------------------------------------------------
    def _step(self, est, gyro_deg):
        now = _num(getattr(est, "t_pub", None), self.clock())
        if not _fin(now):
            now = self.clock()
        self._track_gyro(est, gyro_deg, now)
        self._v_hist.append(_num(getattr(est, "v_mps", None)))
        self._w_hist.append(_num(getattr(est, "omega_dps", None)))
        self._margin_hist.append((now, _num(getattr(est, "margin_px", None))))

        if self.phase in TERMINAL:
            self.keeper.pump(now)                 # ABORT 도 버스를 조용히 두지 않는다
            return None
        if self.tx is not None:
            self.tx.feed()

        if self.phase is Phase.IDLE:
            return self._idle(est, now)
        if not self._guards(est, now):
            return None

        if self.phase is Phase.OBSERVE:
            return self._observe(est, now)
        if self.phase is Phase.DECIDE:
            return self._decide(est, now)
        if self.phase is Phase.EXECUTE:
            return self._execute(est, now)
        if self.phase is Phase.STOPPING:
            return self._stopping(est, now)
        if self.phase is Phase.SETTLE:
            return self._settle(est, now)
        if self.phase is Phase.VERIFY:
            return self._verify(est, now)
        if self.phase is Phase.REACQUIRE:
            return self._reacquire(est, now)
        if self.phase is Phase.FALLBACK:
            return self._fallback(est, now)
        return None

    # ── 전이 ────────────────────────────────────────────────────────────
    def _go(self, phase, why="", t=None):
        t = self.clock() if t is None else t
        if phase not in ALLOWED.get(self.phase, set()) and phase is not self.phase:
            self.illegal += 1
            self.monitor.note(FailureCode.FM3_PING_PONG,
                              "허용되지 않은 전이 %s → %s (%s)" % (self.phase.value, phase.value, why),
                              t=t)
            self._abort(AbortReason.INTERNAL,
                        "전이표 위반 %s → %s" % (self.phase.value, phase.value))
            return False
        if phase is not self.phase:
            self.transitions.append((t, self.phase, phase, why))
            if self.logger is not None:
                self.logger.event("phase", **{"from": self.phase.value, "to": phase.value,
                                              "why": why, "zone": int(self.zone),
                                              "tier": int(self.tier), "t": t})
            self.phase = phase
            self.t_phase = t
            if phase is Phase.OBSERVE:
                self._obs_n = 0          # 새 관측 구간 — 최소 프레임을 다시 모은다
        return True

    def _set_zone(self, zone, why, t):
        if int(zone) < int(self.zone):
            self.monitor.note(FailureCode.FM7_ZONE_CHATTER,
                              "구역 역행 요구 %s → %s (%s) — 안 한다"
                              % (self.zone.name, zone.name, why), t=t)
            self._tier(Tier.REPOSITION, "구역 역행 요구", t)
            return False
        if zone is not self.zone:
            self.log("  구역 %s → %s : %s" % (self.zone.name, zone.name, why))
            if self.logger is not None:
                self.logger.event("zone", **{"from": self.zone.name, "to": zone.name,
                                             "why": why, "t": t})
            self.zone = zone
            self._last_corr_sign = 0
            self._alt_n = 0
        return True

    # ── 공통 가드 (어느 상태에서나 먼저 본다) ───────────────────────────
    def _guards(self, est, now):
        # 1) 시간 예산
        if self.t_start is not None and now - self.t_start > self.budget_s:
            self._abort(AbortReason.TIME_BUDGET,
                        "시간 예산 %.0f s 초과" % self.budget_s)
            return False
        # 2) (I1) 부호 항등식 — 여기가 부호 사고 현장 검출기다
        r = self._sign_residual(est)
        ro = getattr(self, "_i1_obs", float("nan"))       # 원관측 잔차(있으면)
        if _fin(r) and abs(r) > SIGN_RESID_DEG:
            self._sign_bad_n += 1
            if self._sign_bad_n >= SIGN_RESID_N:
                # **배선 부호**인가 **PnP 오브랜치 락**인가. 원관측(픽셀 vs PnP)이 같이
                # 틀어졌으면 배선, 원관측만 멀쩡하면 거짓 해에 잠긴 것이다(FM1).
                # 대응은 똑같이 즉시 정지 + Tier 3 — 이름만 진실을 말한다.
                wiring = (not _fin(ro)) or abs(ro) > SIGN_RESID_DEG
                if wiring:
                    self.monitor.note(
                        FailureCode.FM2_FALSE_CONF,
                        "(I1) 잔차 %.3f° > %.2f° 가 %d 프레임 연속 — 부호·보정 배선이 "
                        "어긋났다 (원관측 %s)"
                        % (r, SIGN_RESID_DEG, self._sign_bad_n,
                           ("%.3f°" % ro) if _fin(ro) else "없음"), t=now)
                    self._abort(AbortReason.SIGN_CHECK,
                                "(I1) 잔차 %.3f° — 코너 순서·왜곡 모델·δ 배선을 볼 것" % r)
                else:
                    self.monitor.note(
                        FailureCode.FM1_WRONG_BRANCH,
                        "(I1) 발행 잔차 %.3f° 인데 원관측은 %.3f° 로 멀쩡하다 — 배선이 "
                        "아니라 PnP 2중해가 거짓 해로 잠겼다" % (r, ro), t=now)
                    self._abort(AbortReason.WRONG_BRANCH,
                                "(I1) 발행 %.3f° / 원관측 %.3f° — 거짓 해 락. **배선을 "
                                "고치지 마라**. 비껴 서서(oblique) 다시 잡아야 한다" % (r, ro))
                return False
        elif _fin(r):
            self._sign_bad_n = 0
        # 3) 추정기 재초기화 2회 (plan 3-7)
        if int(getattr(est, "reinit_count", 0) or 0) >= 2:
            self._abort(AbortReason.REINIT_TWICE, "추정기 재초기화 2회")
            return False
        # 4) 자이로
        if not _flag(est, "gyro_alive", True):
            if self._blind_now():
                self._abort(AbortReason.GYRO_DEAD_BLIND, "블라인드 중 자이로 사망")
                return False
            if self.phase is Phase.EXECUTE:
                self._stop(StopReason.GYRO_DEAD, "자이로 사망", now)
                return False
        # 5) FM11 이중 송신원 — 우리가 안 보낸 0x1E3 이 버스에 있으면 사람이 잡은 것이다
        if self.tx is not None:
            foreign = int(getattr(self.tx, "foreign_movement_frames", 0) or 0)
            if foreign > self._fm11_seen:
                self._fm11_seen = foreign
                self.release("버스에 우리가 안 보낸 0x1E3 이 %d 건 — 조이스틱·리모컨 개입"
                             % foreign)
                return False
        # 6) CAN 건강 (FM9 · FM8a)
        if self.tx is not None:
            ok, det = can_health(self.tx.stats(), base_trips=self._base_trips)
            if not ok:
                self._base_trips = int(self.tx.stats().get("deadman_trips") or 0)
                self._can_bad_n += 1
                # 한 프레임짜리 튐으로 EXECUTE 를 끊지 않는다 — 연속 k 프레임일 때만.
                # 기록도 프레임마다 찍지 않는다(로그 폭주가 다시 간격을 키운다).
                if self._can_bad_n % 30 == 1:
                    self.monitor.note(FailureCode.FM9_CAN, det["why"], t=now)
                if self.phase is Phase.EXECUTE and self._can_bad_n >= CAN_BAD_FRAMES:
                    self._stop(StopReason.DEADMAN, det["why"], now)
                    return False
            else:
                self._can_bad_n = 0
        # 7) stale — 판단·정지에 쓰지 않는다
        if _flag(est, "stale") and self.phase is Phase.EXECUTE:
            self.monitor.note(FailureCode.FM8A_THREAD_LAG, "프레임 stale — 정지", t=now)
            self._stop(StopReason.STALE, "stale", now)
            return False
        return True

    def _sign_residual(self, est):
        """(I1) β + atan2(ℓ,x)·180/π + ψ + δ. 0 이어야 한다 (계약 §1.3·§2.4)."""
        fn = getattr(est, "bearing_identity_residual", None)
        if callable(fn):
            try:
                return _num(fn(self.delta_deg))
            except Exception:
                pass
        if not _flag(est, "tag_seen"):
            return float("nan")
        b, lat, x, psi = (_num(getattr(est, "beta_deg", None)), _num(getattr(est, "lat_m", None)),
                          _num(getattr(est, "x_m", None)), _num(getattr(est, "psi_deg", None)))
        if not (_fin(b) and _fin(lat) and _fin(x) and _fin(psi)) or x <= 0.0:
            return float("nan")
        return b + math.degrees(math.atan2(lat, x)) + psi + self.delta_deg

    def _track_gyro(self, est, gyro_deg, now):
        ext = gyro_deg if gyro_deg is not None else getattr(est, "gyro_deg", None)
        if _fin(_num(ext)):
            self._gyro_ext = _num(ext)
            self._gyro_deg = self._gyro_ext
        else:                                     # 없으면 ω 를 적분해 쓴다(계약에 gyro_deg 가 없다)
            w = _num(getattr(est, "omega_dps", None), 0.0)
            if self._t_prev is not None and _fin(w):
                self._gyro_deg += w * max(0.0, now - self._t_prev)
        self._t_prev = now

    def _beta_vis(self, beta):
        """그 **방향**의 가시 한계 |β| [°]. β<0 이면 L(rotate_ccw 쪽), β>0 이면 R."""
        if not _fin(_num(beta)):
            return self.beta_vis_deg
        return self.beta_vis_L if beta < 0 else self.beta_vis_R

    def _blind_now(self):
        c = self.command
        return (self.zone is Zone.FINAL
                or (c is not None and c.primitive in (Primitive.FORWARD_BLIND, Primitive.BACKWARD)))

    # ── IDLE ────────────────────────────────────────────────────────────
    def _idle(self, est, now):
        if not self.armed:
            return None
        if not self.gate_ok and not self.assume_calib:
            self._abort(AbortReason.CALIB,
                        "캘리브 게이트 미통과 — 실주행 거부(dry-run 만). "
                        "사람이 --assume-calib 를 직접 줘야 가정값으로 돈다")
            return None
        if not self.gate_ok:
            self.log("  !! --assume-calib : 캘리브 없이 가정값으로 출발한다")
        self._go(Phase.OBSERVE, "출발", now)
        self._obs_n = 0
        return None

    # ── OBSERVE ─────────────────────────────────────────────────────────
    def _observe(self, est, now):
        self._obs_n += 1
        seen = _flag(est, "tag_seen")
        blind_ok = self.zone is Zone.FINAL or bool(self._macro)

        if not seen and not blind_ok:
            self._lost_since = self._lost_since or now
            if now - self._lost_since > REACQUIRE_AFTER_S:
                self._go(Phase.REACQUIRE, "태그 실종 %.1f s" % (now - self._lost_since), now)
                return None
        else:
            self._lost_since = None

        if _flag(est, "ambiguous"):
            self._amb_since = self._amb_since or now
            if now - self._amb_since > AMBIGUOUS_CAP_S:
                self.monitor.note(FailureCode.FM1_WRONG_BRANCH,
                                  "AMBIGUOUS %.1f s 지속" % (now - self._amb_since), t=now)
                self._go(Phase.FALLBACK, "AMBIGUOUS 지속", now)
                return None
            return None                   # 이 블록은 AMBIGUOUS 타임아웃이 임자다(WAIT 캡 아님)
        else:
            self._amb_since = None

        # 값이 서 있어야 결정한다. **계약 §2.3 의 긍정형 게이트** — 없으면 추정기가
        # 아직 안 선 첫 몇 초(ψ̂ = NaN, σ_ψ = NaN, x̂ 이 진값의 1.7배)에도 DECIDE 가
        # 명령을 낸다(D팀 보고 B: replay 34306 의 0.03~2.83 s 구간에서 실제로 났다).
        finite = (_fin(_num(getattr(est, "psi_deg", None)))
                  and _fin(_num(getattr(est, "sigma_psi_deg", None)))
                  and (_fin(_num(getattr(est, "x_m", None))) or blind_ok))
        ready = (not _flag(est, "stale") and _flag(est, "gyro_alive", True)
                 and not _flag(est, "ambiguous") and self._obs_n >= MIN_OBS_FRAMES
                 and finite and self._consistency_ok and (seen or blind_ok))
        if not ready:
            if self._wait_t0 is not None and now - self._wait_t0 > WAIT_CAP_S:
                self.monitor.note(FailureCode.FM6_WAIT_DEADLOCK,
                                  "WAIT %.1f s 지나도 판단 가능 상태가 안 된다" % (now - self._wait_t0),
                                  t=now)
                self._tier(Tier.REPOSITION, "WAIT 캡 초과", now)
                self._wait_t0 = None          # 시계 다시 잡기
                self._forced = "reposition"
                self._go(Phase.DECIDE, "Tier 1", now)
            elif self._wait_t0 is None:
                self._wait_t0 = now
            return None
        self._wait_t0 = None
        self._go(Phase.DECIDE, "관측 준비됨(%d 프레임)" % self._obs_n, now)
        return None

    # ── DECIDE — 명령은 여기서만 고른다 ──────────────────────────────────
    def _decide(self, est, now):
        o = self._snap(est)
        forced, self._forced = self._forced, None

        if self._macro:                                   # 예약된 프리미티브(블라인드 쪼개기 등)
            return self._issue_macro(o, now)
        if forced == "reposition":
            return self._plan_reposition(o, now)
        if forced == "reacquire":
            return self._plan_reacquire(o, now)
        if forced == "fallback":
            return self._plan_fallback(o, now)
        if self.zone is Zone.FINAL:
            return self._plan_blind(o, now)

        d, lat = o["x"], o["lat"]
        if not (_fin(d) and _fin(lat)):
            self._go(Phase.OBSERVE, "값 없음", now)
            return None

        # T 도착? → 태그 겨냥 회전 / 아니면 대각 다리
        at_T = d <= self.x_T + self.dyn.min_forward_m(97)
        run = d - self.x_T                       # T 까지 남은 축 방향 거리
        if at_T:
            beta_star = -self.delta_deg                                   # (I5) E1
            kind = "태그 겨냥"
        elif run < J_MIN_RUN_M:
            # **T 코앞에서는 J 겨냥이 발산한다.** α* = atan(ℓ/(d−x_J)) 라 분모가 0 으로
            # 가면 필요한 β 가 가시 한계를 넘어 "포락선 밖" 으로 오판된다 — 2026-09-21
            # 통합 시뮬에서 d−x_T = 0.16 m·ℓ 0.4 m 가 β 31.6° 를 요구해 ABORT 했다.
            # 거기서 할 일은 애초에 **태그 겨냥**이다(가이드 순서 그대로). 남은 다리가
            # 최소 증분 두 배도 안 되면 대각으로 고칠 수 있는 게 없다.
            beta_star = -self.delta_deg
            kind = "태그 겨냥(T 코앞 %.2f m)" % run
        else:
            psi_J = -math.degrees(math.atan2(lat, run))
            beta_star = -math.degrees(math.atan2(lat, d)) - psi_J - self.delta_deg
            kind = "대각 다리(J)"
            if abs(beta_star) > self._beta_vis(beta_star):   # 진짜 포락선 밖 (plan 3-4)
                return self._plan_envelope(o, beta_star, now)
            lim = BETA_VIS_USE * self._beta_vis(beta_star)   # 계획 여유 — 넘으면 부분 조준만
            beta_star = max(-lim, min(lim, beta_star))

        dpsi = o["beta"] - beta_star                     # Δβ = −Δψ (계약 §1.3·§7.3-2)
        if not _fin(dpsi):
            dpsi = -o["psi"]                              # 태그가 없으면 축 정렬로라도
        lever = max(d - C.CAM_TO_REF_M, 0.3)
        dead = max(math.degrees(math.atan2(C.LAT_TOL_M, 2.0 * lever)),
                   self.dyn.min_turn_deg("L" if dpsi > 0 else "R"),
                   K_SIGMA * o["sig_beta"] if _fin(o["sig_beta"]) else 0.0)
        far_dir_only = (self.zone is Zone.FAR
                        and _fin(o["sig_lat"]) and abs(lat) <= K_SIGMA * o["sig_lat"])
        lim_c = BETA_VIS_USE * self._beta_vis(o["beta"])
        if far_dir_only and not at_T and _fin(o["beta"]) and abs(o["beta"]) < 0.5 * lim_c:
            # FAR 에서는 방향만 유의하다 — 30 mm 급 lateral 판정 금지(plan 3-6).
            # 단 태그가 화면 가장자리로 가면 가시성이 먼저라 재중심 회전은 한다(3-4 하드 제약)
            dead = max(dead, abs(dpsi) + 1.0)

        if abs(dpsi) > dead:
            self._rot_denied = False
            cmd = self._plan_rotate(o, dpsi, now, why="%s β*=%.1f°" % (kind, beta_star))
            if cmd is not None or not self._rot_denied:
                return cmd
            if at_T:                       # 돌 수도 없고 갈 수도 없다 → Tier 1
                self._tier(Tier.REPOSITION, "반전 금지 상태에서 T 도착", now)
                self._forced = "reposition"
                self._go(Phase.OBSERVE, "Tier 1", now)
                return None
        # 반전 가드(_reversal_ok)는 **T 도착 뒤 미세 보정에만** 건다. 'T 코앞 태그
        # 겨냥' 까지 넓혀 보니 필요한 재조준이 막혀 |e_l| 중앙이 137 → 227 mm 로
        # 나빠졌다(200회 몬테카를로, 2026-09-21). 넓히지 않는다.
        self._aimed = at_T
        if at_T:
            self._go(Phase.VERIFY, "T 도착 + 겨냥 완료", now)
            self._vbuf, self._t_verify0 = [], now
            return None
        return self._plan_forward(o, now)

    # -- 계획: 회전 -------------------------------------------------------
    def _plan_rotate(self, o, dpsi, now, why=""):
        cap = ROT_MAX_DEG_DEGRADED if self.dyn.degraded else ROT_MAX_DEG
        mag = max(-cap, min(cap, dpsi))
        sign = 1.0 if mag > 0 else -1.0
        if self._last_corr_sign and sign != self._last_corr_sign:
            self._alt_n += 1
            fine = self._aimed or self.zone is Zone.FINAL
            if fine:
                # 미세 보정 구간 — 9/7 의 좌우 왕복이 났던 자리다. 반전 가드를 건다
                if not self._reversal_ok(o, now):
                    self._rot_denied = True   # 회전 포기, 다리로 진행(제자리 맴돌기 금지)
                    return None
            elif self._alt_n > 3:
                # FAR 접근 조준은 번갈아 도는 게 정상이지만, 너무 자주면 유령 오차 추종이다
                # plan 4-4 FM3: "같은 구역에서 부호 반전 2회면 Tier 1"
                self.monitor.note(FailureCode.FM3_PING_PONG,
                                  "%s 에서 조준 부호가 %d 번 바뀐다 — 유령 오차 추종 의심"
                                  % (self.zone.name, self._alt_n), t=now)
                self._tier(Tier.REPOSITION, "조준 왕복", now)
                self._alt_n = 0          # 이 에피소드는 닫는다(다음 왕복부터 다시 센다)
                self._rot_denied = True
                return None
        side = "L" if mag > 0 else "R"
        expect = _Pred("stop_rotate", self.dyn.min_turn_deg(side), level=97, cell=side)
        cmd = Command(Primitive.ROTATE,
                      Movement.ROTATE_CCW.value if mag > 0 else Movement.ROTATE_CW.value,
                      mag, "gyro_angle", mag, self._level(), expect,
                      self.dyn.rotate_timeout_s(mag, 8.0), self.zone, self.tier,
                      int(getattr(o["est"], "seq", -1) or -1), now,
                      "%s Δψ=%+.1f° %s" % (why, mag, "(강등 캡)" if abs(dpsi) > cap else ""))
        self._last_corr_sign = sign
        return self._issue(cmd, o, now)

    # -- 계획: 전진 -------------------------------------------------------
    def _plan_forward(self, o, now):
        if not _fin(o["x"]):
            self._go(Phase.OBSERVE, "x 를 모른다 — 더 본다", now)
            return None
        level = self._level()
        remain = o["x"] - self.x_T
        leg_cap = LEG_MAX_M_DEGRADED if self.dyn.degraded else LEG_MAX_M
        # 남은 거리의 60% 까지만 — J 에 붙을수록 α* = atan(ℓ/(d−x_J)) 가 발산하므로
        # 매번 재조준할 자리를 남긴다(소각 대각 접근)
        leg = min(remain, leg_cap, max(0.6 * remain, self.dyn.min_forward_m(level)))
        if leg < self.dyn.min_forward_m(level):
            self._aimed = True
            self._go(Phase.VERIFY, "남은 %.2f m 가 최소 증분보다 작다 — T 도착으로 본다" % remain, now)
            self._vbuf, self._t_verify0 = [], now
            return None
        # **여기서 _alt_n 을 지우지 않는다.** rotate→forward→rotate→forward 가 정상
        # 패턴이라 전진마다 지우면 카운터가 1 을 못 넘어 FM3 가드가 죽는다(9/7 의
        # 좌우 왕복이 났던 바로 그 구역이 무방비였다). 초기화는 **구역이 바뀔 때만**.
        target_x = max(o["x"] - leg, self.x_stop_cam)
        expect = self.dyn.stop_distance(self.dyn.v_of(level), level, "fwd")
        cmd = Command(Primitive.FORWARD, self._fwd_movement(level), leg,
                      "camera_x", target_x, level, expect,
                      self.dyn.command_time_for(leg, level).value * 2.0 + 2.0,
                      self.zone, self.tier, int(getattr(o["est"], "seq", -1) or -1), now,
                      "대각 다리 전진 %.2f m → x %.2f m" % (leg, target_x))
        return self._issue(cmd, o, now)

    # -- 계획: 블라인드 (FINAL) -------------------------------------------
    def _plan_blind(self, o, now):
        """FINAL 블라인드 다리.

        v97 의 m/s 는 **완전 미측정**이다. 그래서 시간만 보내고 계획값을 차감하면
        v97 이 가정보다 크기만 해도 포크 끝이 태그면(벽)을 지나간다. 여기서는 셋으로 막는다:
          1) 한 다리의 **누적 주행 상한** = v_upper × 움직인 시간 ≤ 다리 길이 (EXECUTE 감시)
          2) FINAL 전체의 **절대 주행 예산** `_final_budget_m` = (진입 x̂ − 정지점) + 허용오차
             — v97 이 얼마든 실제 이동은 이 값을 못 넘는다
          3) 남은 거리는 **간 뒤에** 실경과로 차감한다(선차감 금지 — 다리가 끊기면
             안 갔는데 다 간 것으로 치고 done_dr 로 끝났다)
        """
        if self._final_remain_m is None:
            x0 = o["x"] if _fin(o["x"]) else self.x_T
            self._final_remain_m = max(0.0, x0 - self.x_stop_cam)
            # 예산은 **충돌점 기준**이다 — 포크 끝이 태그면에 닿는 자리(카메라 x =
            # CAM_TO_REF_M)에서 COLLISION_GUARD_M 앞까지. v97 이 상한만큼 빨라도
            # 실제 주행은 이 값을 못 넘으므로 포크 끝은 태그면 앞에서 반드시 선다.
            self._final_budget_m = max(self.fwd_tol_m,
                                       x0 - C.CAM_TO_REF_M - COLLISION_GUARD_M)
            self._blind_done_m = 0.0
            self.log("  FINAL 블라인드: 남은 %.2f m, 주행 예산 %.2f m (충돌점 %.2f m 앞 "
                     "%.2f m, v97 상한 %.2f m/s)"
                     % (self._final_remain_m, self._final_budget_m, C.CAM_TO_REF_M,
                        COLLISION_GUARD_M, self.dyn.v_upper(97)))
        if self._final_remain_m <= self.fwd_tol_m or self._budget_spent() or self._final_done:
            # 종료 판정은 VERIFY 에서만 한다 (계약 §4.3: DECIDE 에서 DONE 으로 못 간다)
            self._go(Phase.VERIFY, "블라인드 종료 판정", now)
            self._vbuf, self._t_verify0 = [], now
            return None
        cap = BLIND_CHUNK_M if not self.dyn.degraded else BLIND_CHUNK_M_DEGRADED
        left = max(0.0, self._final_budget_m - self._blind_done_m)
        # **단위를 섞지 않는다**: chunk 는 공칭 거리, left 는 v 상한 화폐다.
        # left 로 chunk 를 자르면 공칭 v 에서도 다리가 계속 짧아져 목표에 못 닿는다.
        # 예산은 아래 명령시간(t_allow)과 EXECUTE 의 누적 감시로만 건다.
        chunk = min(self._final_remain_m, cap)
        if chunk < self.dyn.min_forward_m(97) or left <= 0.0:
            # 더 낼 수 있는 명령이 없다. 여기서 끝내지 않으면 VERIFY↔DECIDE 를 예산까지 돈다
            self._final_done = True
            why = ("주행 예산이 %.2f m 밖에 안 남았다 (v97 상한 %.2f m/s 로 셌다 — "
                   "v97 실측 전에는 **덜 가고 선다**)" % (left, self.dyn.v_upper(97))
                   if left <= self._final_remain_m else
                   "남은 %.2f m 가 최소 증분보다 작다" % self._final_remain_m)
            self.log("  블라인드 종료: %s. 장부상 %.2f m 남음 — 줄자로 잴 것"
                     % (why, self._final_remain_m))
            self._go(Phase.VERIFY, "남은 블라인드가 최소 증분보다 작다", now)
            self._vbuf, self._t_verify0 = [], now
            return None
        p = self.dyn.command_time_for(chunk, 97)
        # 명령시간도 예산으로 깎는다 — v97 이 상한이어도 이 다리가 예산을 못 넘게.
        t_dead = self.dyn._get("tau_start_s.fwd.value", _ASSUMED["t_dead_fwd"])
        t_allow = t_dead + left / max(self.dyn.v_upper(97), 0.02)
        p = _Pred("leg_time", min(p.value, t_allow), level=97, note=p.note)
        expect = self.dyn.stop_distance(self.dyn.v_of(97), 97, "fwd")
        cmd = Command(Primitive.FORWARD_BLIND, Movement.FORWARD_SLOW.value, chunk,
                      "time", p.value, 97, expect, p.value * 1.5 + 2.0,
                      self.zone, self.tier, int(getattr(o["est"], "seq", -1) or -1), now,
                      "블라인드 직진 %.2f m (%.1f s, 남은 %.2f m, 예산 %.2f/%.2f)"
                      % (chunk, p.value, self._final_remain_m,
                         self._blind_done_m, self._final_budget_m))
        return self._issue(cmd, o, now)

    def _budget_spent(self):
        """FINAL 주행 예산을 다 썼나 (v97 이 얼마든 여기서 멈춘다)."""
        return (self._final_budget_m is not None
                and self._blind_done_m >= self._final_budget_m - 1e-9)

    # -- 계획: 포락선 밖 dog-leg (FAR 한정 1회) ---------------------------
    def _plan_envelope(self, o, beta_want, now):
        if self.zone is not Zone.FAR or self._dogleg_n >= 1:
            self._abort(AbortReason.ENVELOPE,
                        "포락선 밖: 태그를 보며 축에 합류하려면 β %.1f° 가 필요한데 가시 한계는 "
                        "%.1f° 다. dog-leg 는 FAR 에서 1회뿐(지금 구역 %s, 쓴 횟수 %d)"
                        % (beta_want, self._beta_vis(beta_want), self.zone.name,
                           self._dogleg_n))
            return None
        self._dogleg_n += 1
        lim = BETA_VIS_USE * self._beta_vis(beta_want)
        turn = math.copysign(lim, beta_want) - o["beta"]
        turn = max(-ROT_MAX_DEG, min(ROT_MAX_DEG, turn))
        self.monitor.note(FailureCode.FM7_ZONE_CHATTER,
                          "포락선 밖 — FAR dog-leg 1회 (필요 β %.1f°)" % beta_want, t=now)
        self._macro = [{"kind": "blind_leg", "m": min(2.0, max(0.5, abs(o["lat"]) * 0.5)),
                        "why": "dog-leg 블라인드 다리"}]
        return self._plan_rotate(o, turn, now, why="dog-leg 회전")

    # -- 계획: Tier 1 재배치 ----------------------------------------------
    def _plan_reposition(self, o, now):
        self._tier1_n += 1
        if self._tier1_n > TIER1_MAX:
            self._abort(AbortReason.NO_CONVERGENCE,
                        "Tier 1 재배치를 %d 번 했는데 판정이 안 선다 — 관측성이 안 는다 "
                        "(σ_θ·σ_δ 실측 또는 레버 단축이 필요하다)" % self._tier1_n)
            return None
        if _fin(o["x"]) and o["x"] - self.x_T > self.dyn.min_forward_m(97):
            return self._plan_forward(o, now)         # 가까워지면 σ 가 준다 = 관측성 개선
        b = min(BACK_MAX_M, max(self.dyn.min_forward_m(97), 0.2))
        # ⚠ **저속 후진 단이 없다.** MOVEMENT_TEMPLATES["backward"] 는 byte2=187(전속)
        # 하나뿐이고 새 byte2 값은 실차에서 확인된 적이 없다 → 만들지 않는다.
        # 대신 (1) 다리를 ≤0.3 m 로 묶고 (2) 시간을 **67 속도**로 잡고(코스팅을 빼므로
        # 실제로는 관성만큼만 간다) (3) level 을 67 로 **정직하게** 적는다.
        # "가정값이면 97 한정" 이라는 화면 문구는 이 한 동작만 예외라고 arm() 이 말한다.
        p = self.dyn.command_time_for(b, 67)
        expect = self.dyn.stop_distance(self.dyn.v_of(67), 67, "bwd")
        if self.logger is not None:
            self.logger.event("backward_full_speed", leg_m=b, byte2=187, t=now,
                              why="저속 후진 단이 없다 — 새 byte2 값은 실차 미검증이라 만들지 않는다")
        self.log("  !! Tier 1 후진은 byte2=187(전속)이다 — 저속 단이 없다. 다리 %.2f m" % b)
        cmd = Command(Primitive.BACKWARD, Movement.BACKWARD.value, b, "time", p.value,
                      67, expect, p.value * 1.5 + 2.0, self.zone, Tier.REPOSITION,
                      int(getattr(o["est"], "seq", -1) or -1), now,
                      "Tier 1 재배치 — 후진 %.2f m(187 전속, 저속 단 없음) 뒤 재관측" % b)
        return self._issue(cmd, o, now)

    def _plan_reacquire(self, o, now):
        b = _num(getattr(o["est"], "beta_pred_deg", None), o["beta"])
        if not _fin(b):
            b = -o["psi"] if _fin(o["psi"]) else 0.0
        turn = max(-self.beta_vis_L, min(self.beta_vis_R, b))
        return self._plan_rotate(o, turn, now, why="REACQUIRE(예측 β %.1f°)" % b)

    def _plan_fallback(self, o, now):
        """AMBIGUOUS — 정면 원뿔을 벗어나려고 축에서 더 비껴 선다(plan 3-4 oblique 재획득)."""
        away = math.copysign(8.0, o["lat"] if _fin(o["lat"]) and o["lat"] != 0 else 1.0)
        self._macro = [{"kind": "leg", "m": min(1.0, max(self.dyn.min_forward_m(97), 0.3)),
                        "why": "oblique 재획득 다리"}]
        return self._plan_rotate(o, away, now, why="FALLBACK oblique")

    def _issue_macro(self, o, now):
        if not _fin(o["x"]) and self._macro and self._macro[0]["kind"] == "leg":
            self._go(Phase.OBSERVE, "x 를 모른다 — 매크로 보류", now)
            return None
        m = self._macro.pop(0)
        level = 97
        dist = float(m["m"])
        if m["kind"] == "blind_leg":
            p = self.dyn.command_time_for(dist, level)
            cmd = Command(Primitive.DOGLEG, Movement.FORWARD_SLOW.value, dist, "time",
                          min(p.value, 10.0), level,
                          self.dyn.stop_distance(self.dyn.v_of(level), level, "fwd"),
                          min(p.value, 10.0) * 1.5 + 2.0, self.zone, self.tier,
                          int(getattr(o["est"], "seq", -1) or -1), now,
                          "%s %.2f m" % (m["why"], dist))
        else:
            target_x = (o["x"] - dist) if _fin(o["x"]) else self.x_T
            cmd = Command(Primitive.FORWARD, self._fwd_movement(level), dist,
                          "camera_x", target_x, level,
                          self.dyn.stop_distance(self.dyn.v_of(level), level, "fwd"),
                          self.dyn.command_time_for(dist, level).value * 2.0 + 2.0,
                          self.zone, self.tier,
                          int(getattr(o["est"], "seq", -1) or -1), now,
                          "%s %.2f m" % (m["why"], dist))
        return self._issue(cmd, o, now)

    # -- 래치 -------------------------------------------------------------
    def _issue(self, cmd, o, now):
        """**여기가 set_movement 를 부르는 유일한 자리다** (DECIDE → EXECUTE)."""
        self._pre = {"t": now, "psi": o["psi"], "lat": o["lat"], "x": o["x"],
                     "beta": o["beta"], "gyro": self._gyro_deg, "cmd": cmd}
        self.command = cmd
        self.stop_reason = None
        self._moved = False
        self._moved_x0 = _num(o["x"])
        self._moved_g0 = self._gyro_deg
        self._moved_blind_unknown = False
        self._x_win.clear()
        if not self._go(Phase.EXECUTE, cmd.why, now):
            return None
        if self.tx is not None:
            self.tx.set_movement(cmd.movement, why=cmd.why)
        if self.logger is not None:
            self.logger.event("command", **cmd.as_row())
        self.log("  → %s" % cmd.why)
        return cmd

    # ── EXECUTE — 여기서 낼 수 있는 명령은 stop 뿐 ───────────────────────
    def _execute(self, est, now):
        cmd = self.command
        if cmd is None:
            self._stop(StopReason.OPERATOR, "명령 없음", now)
            return None
        age = now - cmd.t_decide
        if age > cmd.timeout_s:
            self._stop(StopReason.TIMEOUT, "명령 %.1f s 초과" % age, now)
            return None
        if self._pre is None:
            self._pre = {"t": cmd.t_decide, "psi": float("nan"), "lat": float("nan"),
                         "x": float("nan"), "beta": float("nan"),
                         "gyro": self._gyro_deg, "cmd": cmd}
        v, w = _num(getattr(est, "v_mps", None), 0.0), _num(getattr(est, "omega_dps", None), 0.0)
        # **v̂ 로 '움직였다' 를 판정하지 않는다.** v̂ 는 명령버퍼 prior(= 가정 v97)를
        # 되풀이한 값이라, 차가 1 mm 도 안 움직여도 죽은시간 직후 +0.12 m/s 가 된다
        # → FM4(데드존·무응답) 검출기가 원리적으로 못 뜬다. 내일 1순위 미지수인
        # "97 이 데드밴드 위인가" 를 잡을 장치가 이것뿐이라 관측으로만 센다.
        self._note_moved(est, w)
        dead = _ASSUMED["t_dead_rot"] if cmd.primitive is Primitive.ROTATE else _ASSUMED["t_dead_fwd"]
        dead = dead * START_DELAY_GAIN + self._no_resp_window(cmd)
        if getattr(self, "_moved_blind_unknown", False) and age > dead:
            # 태그도 안 보이고 회전도 아니다 = 움직임을 **관측할 방법이 없다**.
            # '무응답' 이라고 단정하지 않고 그 사실만 남긴다(거짓 진단 금지).
            if self.monitor.count(FailureCode.FM4_DEADZONE) == 0:
                self.monitor.note(FailureCode.FM4_DEADZONE,
                                  "블라인드 %s 중에는 움직임을 관측할 수단이 없다 — "
                                  "무응답 판정 불가(거리 상한·예산으로만 막는다)" % cmd.movement,
                                  t=now)
            self._moved_blind_unknown = False
        if not self._moved and not _flag(est, "tag_seen") \
                and cmd.primitive in (Primitive.FORWARD_BLIND, Primitive.DOGLEG):
            pass                                   # 판정 불가 — 아래 FM4 로 안 내려간다
        elif not self._moved and age > dead:
            self.monitor.note(FailureCode.FM4_DEADZONE,
                              "명령 %s 후 %.1f s 동안 움직임 없음" % (cmd.movement, age), t=now)
            if self.monitor.cause_once("no_response"):
                self._stop(StopReason.NO_RESPONSE, "무응답 — 1회 재시도", now)
            else:
                self._abort(AbortReason.NO_RESPONSE, "명령에 차가 반응하지 않는다")
            return None

        if cmd.primitive is Primitive.ROTATE:
            dth = self._gyro_deg - self._pre["gyro"]
            if dth * cmd.end_value < 0 and abs(dth) > ROT_WRONG_WAY_DEG:
                self._stop(StopReason.WRONG_WAY, "반대로 %.1f° 돌았다" % dth, now)
                return None
            rem = abs(cmd.end_value) - abs(dth) * (1.0 if dth * cmd.end_value >= 0 else -1.0)
            if rem <= 0 or self.dyn.should_stop_rotate(max(rem, 0.0), w,
                                                       "L" if cmd.end_value > 0 else "R",
                                                       age < dead + 0.5):
                self._stop(StopReason.TARGET, "회전 %.1f/%.1f°" % (dth, cmd.end_value), now)
            return None

        if cmd.primitive is Primitive.FORWARD:
            x = _num(getattr(est, "x_m", None))
            if self._tag_cut_soon(est, now, v):
                self._stop(StopReason.TAG_CUT, "태그가 곧 잘린다", now)
                return None
            if _fin(x) and x - self._stop_dist_q95(v, cmd.level) < self.x_stop_cam:
                # 허용오차는 '성공 판정' 에 쓰는 값이지 **안전선을 미는 값이 아니다**.
                # 예전엔 x_stop_cam − fwd_tol_m 이라 10 cm 를 벽 쪽으로 내줬고,
                # 중앙값 정지거리를 써서 절반은 예측보다 더 갔다.
                self._stop(StopReason.HARD_CAP, "예측 정지점(q95)이 안전선(%.2f m)을 넘는다"
                           % self.x_stop_cam, now)
                return None
            if _fin(x) and self.dyn.should_stop_forward(x - cmd.end_value, v, cmd.level, 0.0):
                self._stop(StopReason.TARGET, "x %.2f → 목표 %.2f m" % (x, cmd.end_value), now)
            return None

        # FORWARD_BLIND / BACKWARD / DOGLEG — 시간 종료 + 자이로 드리프트 + **거리 상한**
        dth = self._gyro_deg - self._pre["gyro"]
        lim = self._fwd_abort_deg(est)
        if _fin(dth) and abs(dth) > lim:
            self.monitor.note(FailureCode.FM8A_THREAD_LAG,
                              "직진 중 heading 이 %.1f° 흘렀다(한도 %.1f°)" % (dth, lim), t=now)
            self._stop(StopReason.HARD_CAP, "직진 중 heading 드리프트 %.1f°" % dth, now)
            return None
        if cmd.primitive in (Primitive.FORWARD_BLIND, Primitive.DOGLEG):
            # ① 태그가 아직 보이면 가시 구간과 **같은 안전선**을 건다
            x = _num(getattr(est, "x_m", None))
            if _flag(est, "tag_seen") and _fin(x) \
                    and x - self._stop_dist_q95(v, cmd.level) < self.x_stop_cam:
                self._stop(StopReason.HARD_CAP,
                           "블라인드 중 예측 정지점(q95)이 안전선(%.2f m)을 넘는다"
                           % self.x_stop_cam, now)
                return None
            # ② FINAL 전체 **주행 예산** — v 와 무관한 상한이다.
            #    한 다리만 묶으면(예전) 공칭 v 에서 매 다리가 v_nom/v_upper 만큼만 가서
            #    구조적으로 못 닿는다. 누적으로 묶으면 공칭에서는 닿고 상한에서만 선다.
            d_max = self._blind_travel_max(age)
            if (cmd.primitive is Primitive.FORWARD_BLIND
                    and self._final_budget_m is not None
                    and self._blind_done_m + d_max >= self._final_budget_m):
                self._stop(StopReason.HARD_CAP,
                           "FINAL 주행 예산 %.2f m 소진 (v 상한 %.2f m/s 기준)"
                           % (self._final_budget_m, self.dyn.v_upper(cmd.level)), now)
                return None
            if cmd.primitive is Primitive.DOGLEG and d_max >= cmd.magnitude:
                self._stop(StopReason.TARGET, "dog-leg 거리 상한 %.2f m" % cmd.magnitude, now)
                return None
        if age >= cmd.end_value:
            self._stop(StopReason.TARGET, "명령시간 %.1f s 끝" % cmd.end_value, now)
        return None

    def _blind_travel_max(self, age):
        """명령 뒤 age [s] 동안 **최대로** 갔을 수 있는 거리 [m] = v_upper × 움직인 시간."""
        lvl = self.command.level if self.command else 97
        t_dead = self.dyn._get("tau_start_s.fwd.value", _ASSUMED["t_dead_fwd"])
        return max(0.0, _num(age, 0.0) - t_dead) * self.dyn.v_upper(lvl)

    def _no_resp_window(self, cmd):
        """무응답 판정까지 기다릴 시간 [s] (죽은시간 **뒤**로).

        회전은 ω̂ 이 바로 보이니 하한이면 충분하고, 직진은 x̂ 이 잡음 위로 올라올
        만큼(= 2·MOVE_EVIDENCE_M) 가야 한다. 안 그러면 멀쩡한 차를 무응답으로 잡는다.
        """
        if cmd.primitive is Primitive.ROTATE:
            return NO_RESP_EXTRA_S
        v = max(self.dyn.v_of(cmd.level), 0.02)
        return max(NO_RESP_EXTRA_S, 2.0 * MOVE_EVIDENCE_M / v + self.dyn.tau_of(cmd.level))

    def _stop_dist_q95(self, v, level):
        """정지거리의 **보수 상단**(q95). 안전선 판정은 중앙값이 아니라 이걸 쓴다."""
        p = self.dyn.stop_distance(v, level, "fwd")
        q = _num(getattr(p, "q95", None))
        val = _num(getattr(p, "value", None), 0.0)
        return q if _fin(q) and q >= val else val * 1.3

    def _note_moved(self, est, w):
        """명령과 **독립인 증거**로만 `_moved` 를 세운다.

        쓰는 것: ① 자이로 각속도 ω̂(회전) ② 카메라 x̂ 의 실제 변화 > 2σ_x(전진).
        안 쓰는 것: v̂ — 명령버퍼 prior 의 메아리라 차가 멎어 있어도 +v97 이 뜬다.
        자이로 **누적각**도 안 쓴다 — 바이어스가 수십 초에 1° 넘게 쌓여 거짓 양성이 된다.
        """
        if _fin(w) and abs(w) > SETTLE_W_FLOOR:
            self._moved = True
        x = _num(getattr(est, "x_m", None))
        sx = _num(getattr(est, "sigma_x_m", None), 0.02)
        if not _fin(self._moved_x0) and _fin(x):
            self._moved_x0 = x
        seen = _flag(est, "tag_seen")
        if seen and _fin(x):
            self._x_win.append(x)
        # **중앙값**을 본다 — 한 프레임의 2σ 튐(σ_x 1 cm 면 20 회에 한 번)으로
        # "움직였다" 가 서면 FM4 가 또 못 뜬다. 5 프레임이 같이 밀려야 인정한다.
        if seen and _fin(self._moved_x0) and len(self._x_win) >= 5:
            if abs(_med(list(self._x_win)) - self._moved_x0) \
                    > max(2.0 * (sx if _fin(sx) else 0.02), MOVE_EVIDENCE_M):
                self._moved = True
        # 블라인드 전진은 카메라도 자이로도 종방향을 못 본다 → **판정 불가**를 남긴다
        if not seen and self.command is not None \
                and self.command.primitive in (Primitive.FORWARD_BLIND, Primitive.DOGLEG) \
                and not self._moved:
            self._moved_blind_unknown = True

    def _tag_cut_soon(self, est, now, v):
        """관측값 중단 술어 (plan 3-4). margin 의 실제 시간변화로 정지지연 뒤를 내다본다."""
        m = _num(getattr(est, "margin_px", None))
        if not _fin(m):
            return False
        hist = [(t, x) for t, x in self._margin_hist if _fin(x)]
        slope = 0.0
        if len(hist) >= 3 and hist[-1][0] > hist[0][0]:
            slope = (hist[-1][1] - hist[0][1]) / (hist[-1][0] - hist[0][0])
        ahead = self.dyn.tau_of(self.command.level if self.command else 97)
        lat = _num(getattr(getattr(est, "stamps", None), "latency_s", None), 0.0)
        pred = m + slope * (ahead + (lat if _fin(lat) else 0.0))
        return pred - self.pitch_px <= 0.0

    def _fwd_abort_deg(self, est):
        """직진 중 heading 중단 술어 하나 (plan 3-5). 레버 = x_cam_now − x_ref."""
        x = _num(getattr(est, "x_m", None), self.x_T)
        lever = max((x if _fin(x) else self.x_T) - C.CAM_TO_REF_M, 0.3)
        geo = math.degrees(math.atan2(C.LAT_TOL_M, lever))
        noise = K_SIGMA * _num(getattr(est, "sigma_psi_deg", None), 0.5)
        return max(geo, noise)

    # ── STOPPING / SETTLE ───────────────────────────────────────────────
    def _stop(self, reason, why, now):
        self.stop_reason = reason
        if self.tx is not None:
            self.tx.stop(why="%s:%s" % (reason.value, why))
        self._stop_req_t = now
        # 정지 판정 창에 "아직 움직이던" 표본을 남기지 않는다 — 8°/s 로 15° 를 돌면
        # ω 창 2 s 가 전부 문턱 위라 실제로 0.3 s 만에 멎어도 가시 판정이 구조적으로
        # 통과 못 하고 매번 시간 캡(1.75 s)을 태운다(도킹당 10~15 s 낭비).
        self._v_hist.clear()
        self._w_hist.clear()
        self._imu_win = []
        self._settle_blind_ledger(now)
        if self.logger is not None:
            self.logger.event("stop", reason=reason.value, why=why, t=now,
                              primitive=(self.command.primitive.value if self.command else None))
        self._go(Phase.STOPPING, "%s (%s)" % (reason.value, why), now)

    def _settle_blind_ledger(self, now):
        """블라인드 다리가 끝났다 — **실경과**로 장부를 맞춘다.

        예전에는 명령을 내기 전에 계획값을 통째로 차감했다. 그래서 stale·데드맨·
        heading 드리프트로 다리가 1.3 s 만에 끊겨도 '다 갔다' 가 되어 종방향 1.5 m 를
        안 가고 done_dr 로 끝났다(종료코드 0).
        """
        c = self.command
        if c is None or c.primitive is not Primitive.FORWARD_BLIND:
            return
        age = max(0.0, _num(now, 0.0) - c.t_decide)
        age = min(age, c.end_value if _fin(c.end_value) else age)
        # 장부(수렴용)는 공칭 모델, 예산(안전용)은 상한 — 서로 반대쪽으로 틀리게 둔다
        done = min(self.dyn.distance_for_command(age, c.level), c.magnitude)
        self._blind_done_m += self._blind_travel_max(age)
        if self._final_remain_m is not None:
            self._final_remain_m = max(0.0, self._final_remain_m - done)
        if self.logger is not None:
            self.logger.event("blind_leg", planned_m=c.magnitude, done_m=done,
                              age_s=age, remain_m=self._final_remain_m,
                              spent_m=self._blind_done_m, budget_m=self._final_budget_m,
                              stop_reason=(self.stop_reason.value if self.stop_reason else None),
                              t=now)

    def _stopping(self, est, now):
        sent = True
        if self.tx is not None:
            sent = (self.tx.movement == Movement.STOP.value)
        if sent or (self._stop_req_t is not None and now - self._stop_req_t > STOP_CONFIRM_CAP_S):
            if not sent:
                self.monitor.note(FailureCode.FM9_CAN, "stop payload 확인을 못 했다", t=now)
            self._go(Phase.SETTLE, "stop 송신됨", now)
            self.t_phase = now
        return None

    def _settle(self, est, now):
        cap = self.dyn.settle_timeout_s(_num(getattr(est, "v_mps", None), 0.0),
                                        self.command.level if self.command else 97)
        blind = not _flag(est, "tag_seen")
        # 블라인드 판정은 **IMU 원시행**(dict)을 먹는다. 예전에는 ω 실수 리스트를
        # 넘겨 Dynamics.settle_blind 가 isinstance(dict) 로 전부 걸러 **항상 False**
        # 였고, 폴백도 `r is not None` 때문에 안 돌아 모든 블라인드 정지가 시간 캡으로
        # 빠졌다(= 아직 미끄러지는 중이어도 '멎었다' 로 봤다).
        # 창은 **SETTLE_HOLD_S(0.3 s ≈ 9 프레임)** 만큼만 본다. 1 s 창을 그대로 쓰면
        # 정지 직후 아직 빠르던 표본이 남아 실제로 멎어도 통과가 안 된다.
        done = (self.dyn.settle_blind(self._imu_win or list(self._w_hist)[-SETTLE_FRAMES:])
                if blind
                else self.dyn.settle_visible(list(self._v_hist)[-SETTLE_FRAMES:],
                                             list(self._w_hist)[-SETTLE_FRAMES:]))
        if done:
            self._settled_confirmed = True
            self._go(Phase.VERIFY, "멎음(%s)" % ("블라인드" if blind else "가시"), now)
            self._vbuf, self._t_verify0 = [], now
            return None
        # **NO_RESPONSE 를 캡보다 먼저** 본다 — 순서가 반대면 시간상 캡이 늘 먼저 와서
        # "stop 을 보냈는데 감속이 안 보인다" 가 영영 안 뜬다
        if now - self.t_phase > cap * 2.0:
            self._abort(AbortReason.NO_RESPONSE,
                        "stop 뒤 %.1f s 가 지나도 감속이 안 보인다" % (now - self.t_phase))
            return None
        if now - self.t_phase > cap:
            self._settled_confirmed = False     # 캡으로 빠졌다 = 정지를 **확인 못 했다**
            self.monitor.note(FailureCode.FM8A_THREAD_LAG,
                              "정지 시간 캡 %.1f s — IMU/카메라로 정지를 확인 못 했다" % cap,
                              t=now)
            self._go(Phase.VERIFY, "정지 시간 캡(미확인)", now)
            self._vbuf, self._t_verify0 = [], now
        return None

    # ── VERIFY — 구역 전환 판정은 여기서만 ──────────────────────────────
    def _verify(self, est, now):
        self._vbuf.append(self._snap(est))
        if len(self._vbuf) < VERIFY_FRAMES and now - self._t_verify0 < VERIFY_CAP_S:
            return None
        m = {k: _med([b[k] for b in self._vbuf])
             for k in ("x", "lat", "psi", "beta", "sig_lat", "sig_psi", "sig_beta", "tilt")}
        seen = sum(1 for b in self._vbuf if b["seen"]) > len(self._vbuf) / 2
        hv = sum(1 for b in self._vbuf if b["hv"]) > len(self._vbuf) / 2

        self._post_consistency(m, now)
        acc = self._accept(m)
        self.last_error = dict(acc, seen=seen, heading_valid=hv, n=len(self._vbuf))
        if self.logger is not None:
            self.logger.event("verify", zone=self.zone.name, t=now, seen=seen,
                              heading_valid=hv, n=len(self._vbuf), **acc_row(acc))

        if self.zone is Zone.FINAL:
            if (self._final_remain_m is not None and self._final_remain_m > self.fwd_tol_m
                    and not self._budget_spent() and not self._final_done):
                self._go(Phase.OBSERVE, "블라인드 %.2f m 남음" % self._final_remain_m, now)
                return None
            if self._budget_spent() and self._final_remain_m and self._final_remain_m > self.fwd_tol_m:
                self.monitor.note(FailureCode.FM4_DEADZONE,
                                  "FINAL 주행 예산 %.2f m 소진 — 장부로는 %.2f m 남았다. "
                                  "예산은 v97 **상한** %.2f m/s 로 세는데 공칭은 %.2f m/s 라, "
                                  "v97 이 미측정인 동안은 구조적으로 덜 가고 선다(벽 안전 우선). "
                                  "줄자로 재고 first_run creep 으로 v97 을 실측할 것"
                                  % (self._final_budget_m, self._final_remain_m,
                                     self.dyn.v_upper(97), self.dyn.v_of(97)), t=now)
            return self._finish({"est": est, **m, "seen": seen}, now)

        # 구역 전환 (단조)
        if (self.zone is Zone.FAR and hv and _fin(acc["sigma_e_l"])
                and acc["sigma_e_l"] <= C.LAT_TOL_M / 2.0):
            self._set_zone(Zone.NEAR, "σ(e_l) %.3f m ≤ T/2" % acc["sigma_e_l"], now)

        if self._aimed:
            if acc["accept_l"] and acc["accept_h"]:
                self._final_gate = "accept"
                self._set_zone(Zone.FINAL, "수용식 통과 + 태그 겨냥 완료", now)
                self._go(Phase.OBSERVE, "FINAL 진입", now)
                self._obs_n = 0
                return None
            self._verify_fail(m, acc, now)
            if self.phase is not Phase.VERIFY:
                return None               # 이미 Tier·FINAL·ABORT 로 갔다
        self._go(Phase.OBSERVE, "다음 다리", now)
        self._obs_n = 0
        return None

    def _verify_fail(self, m, acc, now):
        """T 에서 수용식을 못 넘었을 때 — 3분할 순서대로 (판정불가 → 양자화 공백 → 재접근)."""
        jl = acc["judge_l"]
        if jl.undecidable or acc["judge_h"].undecidable:
            if self.final_anyway:
                return self._final_by_operator(acc, now, "판정불가(kσ %.3f m > T/2)"
                                               % (K_SIGMA * acc["sigma_e_l"]))
            self._tier(Tier.REPOSITION, "판정불가 — σ 가 너무 크다", now)
            self._forced = "reposition"
            self._go(Phase.OBSERVE, "Tier 1", now)
            self._obs_n = 0
            return None
        dq = acc["dq_l"]
        # 양자화 공백 = **수용도 못 하는데 낼 수 있는 최소 명령이 오차보다 크다**.
        # 예전 조건(|e_l| > T ∧ |e_l| < δ_q)에는 구멍이 있었다 — e_l 0.020·σ 0.006·
        # T 0.030·δ_q 0.020 이면 accept·act·undecidable 이 전부 False 인데 어디에도
        # 안 들어가 Tier 2 를 한 번 태우고 사유를 hard_cap 으로 잘못 찍었다.
        if _fin(acc["e_l"]) and not jl.accept and not jl.act:
            # 양자화 공백: 낼 수 있는 가장 작은 명령이 오차보다 크다 (plan 3-5 δ_q, 3-7)
            self.monitor.note(FailureCode.FM4_DEADZONE,
                              "양자화 공백 |e_l| %.3f m ∈ (T %.3f, δ_q %.3f) — "
                              "**저속 회전 단 필요**(plan 2-4 조건부 항목을 필수로)"
                              % (abs(acc["e_l"]), C.LAT_TOL_M, dq), t=now)
            if self.final_anyway:
                return self._final_by_operator(acc, now, "양자화 공백")
            self._abort(AbortReason.QUANT_GAP, "양자화 공백 — 저속 회전 단 필요")
            return None
        if self.final_anyway:
            return self._final_by_operator(acc, now, "|e_l| %.3f m" % abs(acc["e_l"]))
        if self.monitor.cause_once("accept_fail"):
            self._tier(Tier.CAUSE, "수용식 초과 — 97 재접근 1회", now)
            self._aimed = False
            self._forced = "reposition"
            self._go(Phase.OBSERVE, "Tier 2 재접근", now)
            self._obs_n = 0
            return None
        self._abort(AbortReason.HARD_CAP,
                    "재접근을 했는데도 수용식을 못 넘는다 (|e_l| %.3f m, σ %.3f m)"
                    % (abs(acc["e_l"]), acc["sigma_e_l"]))
        return None

    def _final_by_operator(self, acc, now, why):
        """E5 두 번째 문 — 사람이 켠 `--final-anyway`. 결과는 **무조건 DONE_UNVERIFIED**."""
        self._final_gate = "operator"
        self.monitor.note(FailureCode.FM10_WRONG_DONE,
                          "운영자 승인으로 FINAL 진입 (%s) — 결과는 DONE_UNVERIFIED 고정" % why,
                          t=now)
        if self.logger is not None:
            self.logger.event("final_unproven", why=why, t=now, **acc_row(acc))
        self.log("  !! final_unproven: %s — 수용식을 못 넘었다. 줄자로 재야 한다." % why)
        self._set_zone(Zone.FINAL, "운영자 승인(--final-anyway)", now)
        self._go(Phase.OBSERVE, "FINAL 진입(승인)", now)
        self._obs_n = 0
        return None

    def _post_consistency(self, m, now):
        """ⓠ 프리미티브 사후 일관성 (plan 3-5 ⓪). 어긋나면 재앵커·반전 금지."""
        self._consistency_ok = True
        self._reanchor_ok = True
        pre = self._pre
        if pre is None or pre["cmd"] is None:
            return
        dcam = m["psi"] - pre["psi"] if _fin(m["psi"]) and _fin(pre["psi"]) else float("nan")
        dimu = self._gyro_deg - pre["gyro"]
        sig = m["sig_psi"] if _fin(m["sig_psi"]) else 1.0
        if pre["cmd"].primitive is Primitive.ROTATE:
            dlat = m["lat"] - pre["lat"] if _fin(m["lat"]) and _fin(pre["lat"]) else float("nan")
            ok, det = rotation_consistency(dcam, dimu, sig, dlat,
                                           abs(self.A_m) + K_SIGMA * self.sigma_A_m,
                                           m["sig_lat"])
            if not ok:
                self._reanchor_ok = False
                self.monitor.note(FailureCode.FM2_FALSE_CONF,
                                  "회전 사후 불일치 Δψ_cam %.2f vs Δψ_IMU %.2f (한도 %.2f°) — "
                                  "카메라 재앵커·반전 금지, ψ̂ 는 자이로 전파"
                                  % (_num(dcam, 0.0), dimu, det.get("thr_deg", 0.0)), t=now)
        elif pre["cmd"].primitive in (Primitive.FORWARD, Primitive.FORWARD_BLIND):
            leg = abs(pre["x"] - m["x"]) if _fin(pre["x"]) and _fin(m["x"]) else float("nan")
            ok, det = straight_consistency(dcam, dimu, leg, sig)
            if not ok:
                self._consistency_ok = False
                self.monitor.note(FailureCode.FM1_WRONG_BRANCH,
                                  "직진 %.2f m 뒤 Δψ_cam %.2f vs Δψ_IMU %.2f (한도 %.2f°) — "
                                  "2중해 오선택 의심" % (_num(leg, 0.0), _num(dcam, 0.0), dimu,
                                                    det.get("thr_deg", 0.0)), t=now)
        self._pre = None                  # 같은 다리를 두 번 채점하지 않는다

    # ── 수용식 (계약 §3.4) ──────────────────────────────────────────────
    def _sigma_e_l(self, m, lever, sig_psi_aim):
        """σ(e_l) — ψ 오차를 **한 번만** 센다 (E7).

            σ(e_l)² = (x̂·σ_β)² + ((lever − x̂)·σ_ψ)² + (lever·σ_aim,여분)² + σ_roll²

        σ_lat 안에 이미 깔려 있는 d·σ_ψ 바닥은 ℓ̂ 단독 보고용이라 여기 넣지 않는다.
        """
        if not _fin(sig_psi_aim):
            return float("nan")
        d = m["x"] if _fin(m["x"]) else self.x_T
        sp = m["sig_psi"] if _fin(m["sig_psi"]) else float("nan")
        if not _fin(sp):
            return float("nan")
        # 겨냥 잡음 중 σ_ψ 로 이미 센 몫을 뺀 나머지(회전 σ_θ·δ 배선)
        extra = math.sqrt(max(0.0, sig_psi_aim ** 2 - sp ** 2))
        from ..estimate import sigma_e_l as _sig_el
        return _sig_el(d, lever, m["sig_beta"], sp, extra, self.sigma_roll_m,
                       m["sig_lat"])

    def _accept(self, m):
        d = m["x"]
        s = max(0.0, (d if _fin(d) else self.x_T) - self.x_stop_cam)      # 남은 블라인드 다리
        lever = C.CAM_TO_REF_M + s                                        # E3: x_ref + s
        # **ψ̂ 나 σ_ψ 를 모르면 0 으로 치우지 않는다.** 예전에는 ψ=NaN 을 0 으로 바꾸고
        # σ 에 추정기의 σ_ψ 를 한 번도 안 써서, heading 을 전혀 모르는 상태에서
        # accept_l·accept_h 가 동시에 참이 되어 정식 DONE 이 났다(wrong-DONE 유일 경로).
        psi_ok = _fin(m["psi"]) and _fin(m["sig_psi"])
        psi = math.radians(m["psi"]) if psi_ok else float("nan")
        e_l = (m["lat"] + lever * math.sin(psi) + self.x_off_m * math.cos(psi)) \
            if (_fin(m["lat"]) and psi_ok) else float("nan")              # (F1) E2
        sig_psi_aim = math.sqrt(max(0.0, (m["sig_beta"] if _fin(m["sig_beta"]) else 0.3) ** 2
                                    + self.sigma_theta_deg ** 2 + self.sigma_delta_deg ** 2)) \
            if psi_ok else float("nan")
        # (F2) ψ 감도는 **한 번만**, 부호까지 살려 센다. ℓ̂ 은 β 로 풀리므로 ∂ℓ̂/∂ψ ≈ −x̂ 다
        #      → e_l = ℓ̂ + lever·sinψ 의 실효 레버는 (lever − x̂) 이지 lever 가 아니다.
        #      σ_lat 안의 d·σ_ψ 바닥을 여기서 또 더하면 같은 오차를 직교합으로 두 번 센다
        #      (실측 감도 8.7 mm/° 인데 63 mm/° 로 7.2배 과대 → NEAR 가 영영 안 난다).
        sig_l = self._sigma_e_l(m, lever, sig_psi_aim)
        e_h = -math.degrees(math.atan2(m["lat"], d)) if (_fin(m["lat"]) and _fin(d) and d > 0) \
            else float("nan")
        sig_h = math.sqrt((math.degrees((m["sig_lat"] if _fin(m["sig_lat"]) else 0.1)
                                        / max(d if _fin(d) else 1.0, 0.5))) ** 2
                          + sig_psi_aim ** 2) if psi_ok else float("nan")
        dq_l = math.radians(self.dyn.min_turn_deg("L")) * max((d if _fin(d) else self.x_T)
                                                              - C.CAM_TO_REF_M, 0.3)
        jl = judge(e_l, sig_l, C.LAT_TOL_M, dq_l)
        jh = judge(e_h, sig_h, C.HEAD_TOL_DEG, self.dyn.min_turn_deg("L"))
        # 참고용: e_l 을 0 으로 만드는 겨냥각. 가이드는 태그 겨냥 고정이라 **쓰지 않고 로그만** 한다
        beta_zero = float("nan")
        if _fin(m["lat"]) and lever > 0:
            beta_zero = -math.degrees(math.atan2(m["lat"], d if _fin(d) else self.x_T)) \
                + math.degrees(math.asin(max(-1.0, min(1.0, -m["lat"] / lever)))) - self.delta_deg
        return {"e_l": e_l, "sigma_e_l": sig_l, "e_h_deg": e_h, "sigma_e_h_deg": sig_h,
                "lever_m": lever, "blind_m": s, "dq_l": dq_l,
                "sigma_psi_aim_deg": sig_psi_aim,
                "accept_l": jl.accept, "accept_h": jh.accept,
                "judge_l": jl, "judge_h": jh,
                "margin_m": parallelogram_margin(e_l, e_h),
                "beta_zero_el_deg": beta_zero,
                "F_plan": (m["lat"] * C.CAM_TO_REF_M / d) if (_fin(m["lat"]) and _fin(d) and d > 0)
                else float("nan")}

    # ── 종단 ────────────────────────────────────────────────────────────
    def _finish(self, o, now):
        acc = self.last_error or {}
        seen = bool(o.get("seen"))
        ok = bool(acc.get("accept_l")) and bool(acc.get("accept_h"))
        if self._final_gate == "operator" or self.final_anyway:
            ok = False                                    # 승인 모드에서 DONE 은 계약 위반(FM10)
        if not self._settled_confirmed:
            # 정지 판정이 시간 캡으로 빠졌다 = 멎은 걸 **확인 못 했다**. 그 위에 DONE 을
            # 얹으면 아직 미끄러지는 중인 자세를 성공이라고 말하게 된다.
            ok = False
        if self._final_remain_m is not None and self._final_remain_m > self.fwd_tol_m:
            ok = False                                    # 종방향으로 다 안 갔다
        if ok and seen:
            self.result_note = "수용식 통과 + 관측 확인"
            self._go(Phase.DONE, self.result_note, now)
        else:
            self.result_note = ("관측 확인 못 함(태그 컷) — 줄자 검증 필요" if not seen
                                else "수용식 미통과 — 성공이라고 하지 않는다")
            if not seen and self.logger is not None:
                self.logger.event("measure_request",
                                  why="DONE_UNVERIFIED — 포크 끝 lateral·heading 을 줄자로 재 주세요",
                                  t=now, **acc_row(acc))
            self._go(Phase.DONE_UNVERIFIED, self.result_note, now)
        if self.tx is not None:
            self.tx.stop(why="finish")
        self.log("  == %s : %s" % (self.phase.value, self.result_note))
        return None

    def _abort(self, reason, why):
        now = self.clock()
        self.abort_reason = reason
        self.tier = Tier.ABORT
        if self.phase in TERMINAL:
            return
        self.transitions.append((now, self.phase, Phase.ABORT, why))
        self.phase = Phase.ABORT
        self.t_phase = now
        self.result_note = why
        self.keeper.start(reason, why, t=now)

    def _tier(self, tier, why, now):
        self.tier = tier
        if self.logger is not None:
            self.logger.event("tier", tier=int(tier), name=tier.name, why=why, t=now,
                              zone=self.zone.name)
        self.log("  Tier %d (%s): %s" % (int(tier) - 1 if tier != Tier.NONE else 0,
                                         tier.name, why))

    def _reversal_ok(self, o, now):
        """반전 가드 — 정지 후 오차가 T+kσ 를 넘어 0 을 지났을 때만, 구역당 1회 (plan 3-5 ④)."""
        if not self._reanchor_ok:
            self.monitor.note(FailureCode.FM2_FALSE_CONF,
                              "사후 일관성 불합격 뒤의 반전 요구 — 금지", t=now)
            return False
        acc = self.last_error or {}
        e, sig = _num(acc.get("e_l")), _num(acc.get("sigma_e_l"), 0.0)
        if not (_fin(e) and abs(e) > C.LAT_TOL_M + K_SIGMA * sig):
            self.monitor.note(FailureCode.FM3_PING_PONG,
                              "반전 요구인데 오차가 T+kσ 를 못 넘는다 — 금지", t=now)
            return False
        if not self.monitor.reversal_allowed(self.zone):
            self.monitor.note(FailureCode.FM3_PING_PONG,
                              "%s 구역에서 반전은 이미 1회 썼다 — Tier 1" % self.zone.name, t=now)
            self._tier(Tier.REPOSITION, "반전 한도", now)
            return False
        self.monitor.mark_reversal(self.zone)
        return True

    # ── REACQUIRE / FALLBACK ────────────────────────────────────────────
    def _reacquire(self, est, now):
        self._reacq_n += 1
        # 재획득 기동이 끝나고 OBSERVE 로 돌아온 **첫 프레임**에 태그가 아직 안 보이면
        # 곧바로 2회째가 되어 ABORT 했다(유예 1프레임). 회전 관성 1.3~1.5°·폐루프 ±2°
        # 를 생각하면 몇 프레임 뒤에 살아나는 게 보통이다 — 시계를 여기서 다시 잡는다.
        self._lost_since = None
        if self._reacq_n > 1:
            self._abort(AbortReason.ENVELOPE,
                        "REACQUIRE 2회째 — 여기서는 태그를 못 본다(관측 가능 영역 밖)")
            return None
        self._forced = "reacquire"
        self._go(Phase.DECIDE, "REACQUIRE 계획", now)
        return None

    def _fallback(self, est, now):
        self._fallback_n += 1
        self._amb_since = None            # oblique 기동에 **온전한 10 s 창**을 준다
        if self._fallback_n > 1:
            self._abort(AbortReason.ENVELOPE,
                        "FALLBACK 2회째 — oblique 로도 AMBIGUOUS 가 안 풀린다(쓸 관측이 없다)")
            return None
        self._forced = "fallback"
        self._go(Phase.DECIDE, "FALLBACK 계획", now)
        return None

    # ── 잡일 ────────────────────────────────────────────────────────────
    def _level(self):
        """쓸 전진 단. **내려가는 건 언제나 허용, 올라가는 건 절대 강제 안 함.**

        level_cap 은 사람이 `--level 97` 로 "느리게만 가라" 고 말한 자리다(통합이 넣는다).
        """
        if self.dyn.force_slow or self.zone is not Zone.FAR or int(self.level_cap) == 97:
            return 97
        return 67

    def _fwd_movement(self, level):
        return Movement.FORWARD_SLOW.value if level == 97 else Movement.FORWARD.value

    def _snap(self, est):
        return {"est": est,
                "x": _num(getattr(est, "x_m", None)), "lat": _num(getattr(est, "lat_m", None)),
                "psi": _num(getattr(est, "psi_deg", None)), "beta": _num(getattr(est, "beta_deg", None)),
                "sig_lat": _num(getattr(est, "sigma_lat_m", None)),
                "sig_psi": _num(getattr(est, "sigma_psi_deg", None)),
                "sig_beta": _num(getattr(est, "sigma_beta_deg", None)),
                "tilt": _num(getattr(est, "tilt_deg", None)),
                "seen": _flag(est, "tag_seen"), "hv": _flag(est, "heading_valid")}

    def summary(self):
        return {"phase": self.phase.value, "zone": self.zone.name, "tier": int(self.tier),
                "abort": getattr(self.abort_reason, "value", None),
                "note": self.result_note, "steps": self.n_steps,
                "transitions": len(self.transitions), "illegal": self.illegal,
                "final_gate": self._final_gate, "dogleg": self._dogleg_n,
                "reacquire": self._reacq_n, "fallback": self._fallback_n,
                "tier1": self._tier1_n, "failures": self.monitor.summary(),
                "abort_keeper": self.keeper.stats(),
                "error": acc_row(self.last_error)}

    def report(self, log=None):
        log = log or self.log
        s = self.summary()
        log("  결과 %s / 구역 %s / %s" % (s["phase"], s["zone"], s["note"]))
        log("     스텝 %d, 전이 %d, 전이표 위반 %d, 고장기록 %s"
            % (s["steps"], s["transitions"], s["illegal"], s["failures"]["counts"] or "없음"))
        e = s["error"]
        if e:
            log("     e_l %s m (σ %s) · e_h %s° (σ %s) · 여유 %s m"
                % (_r(e.get("e_l")), _r(e.get("sigma_e_l")), _r(e.get("e_h_deg")),
                   _r(e.get("sigma_e_h_deg")), _r(e.get("margin_m"))))
        return s


def acc_row(acc):
    """Judgment 를 뺀 직렬화 가능한 수용식 한 줄."""
    out = {}
    for k, v in (acc or {}).items():
        if isinstance(v, Judgment):
            out[k] = v.as_row()
        elif isinstance(v, (int, float, str, bool)) or v is None:
            out[k] = v
    return out


def _r(v, n=4):
    return "?" if not _fin(_num(v)) else ("%.*f" % (n, v))


__all__ = ["Phase", "Zone", "Primitive", "Movement", "StopReason", "Tier",
           "FailureCode", "AbortReason", "ALLOWED", "TERMINAL", "Command",
           "Judgment", "judge", "parallelogram_margin", "DockFSM", "acc_row",
           "FAILURE_MODES", "K_SIGMA", "VERIFY_FRAMES"]


# ════════════════════════════════════════════════════════════════════════════
# 자기 검증 — 가짜 추정치 시퀀스를 먹여 계약대로 도는지 본다
#
#     python -m src.models.control.dock_fsm            (apriltag_v3 에서)
#
# 카메라·CAN 없이 돈다. **진짜 SafeCanTx 를 쓴다**(송신 창구 계약까지 같이 검사).
# 플랜트 숫자는 tools/etc/fake_rig.py 의 FakePlant 와 같다(9/7 실측: 죽은시간 1.0/0.85 s,
# 정속 0.28/0.12 m/s, 코스팅 τ 0.5/0.18 s, 회전팔 A 0.3 m). fake_rig 를 그대로 안 쓰는 이유 둘:
#   1) 이 맥엔 numpy·cv2 가 없다(fake_rig 는 모듈 상단에서 둘 다 import 한다).
#   2) 계약 §7.4 — fake_rig 의 **렌더가 화면에서 180° 뒤집혀 있다**. 픽셀에서 나오는
#      β 의 부호가 현실과 반대라, 고치기 전에 그걸로 β 판단을 검증하면 "시뮬은 되는데
#      현장에서 반대로 가는" 사고가 난다. 그래서 여기서는 렌더를 거치지 않고
#      **항등식 (I1) 로 β 를 만든다**(부호는 계약 §1.4 실측표 그대로).
# ════════════════════════════════════════════════════════════════════════════

class _MiniPlant:
    """이산 명령 → 죽은시간 + 정속 + 코스팅. 가상 시계(실시간 대기 없음)."""
    V = {"forward": 0.28, "forward_slow": 0.12, "backward": -0.28}
    OMEGA = {"rotate_ccw": +8.0, "rotate_cw": -8.0}
    T_DEAD_FWD, T_DEAD_ROT = 1.00, 0.85
    TAU_FWD, TAU_ROT = 0.50, 0.18
    A_M = 0.30

    def __init__(self, forward=6.0, lateral=0.6, heading_deg=0.0, t0=1000.0, seed=0):
        import random
        self.forward, self.lateral, self.heading = float(forward), float(lateral), float(heading_deg)
        self.v = self.omega = 0.0
        self.t = float(t0)
        self.cmd, self.t_cmd = "stop", float(t0)
        self.rng = random.Random(seed)
        self.gyro_bias_dps = 0.02

    def set_cmd(self, name):
        if name != self.cmd:
            self.cmd, self.t_cmd = name, self.t

    def _target(self):
        age = self.t - self.t_cmd
        if self.cmd in self.V:
            return (self.V[self.cmd] if age >= self.T_DEAD_FWD else 0.0), 0.0
        if self.cmd in self.OMEGA:
            return 0.0, (self.OMEGA[self.cmd] if age >= self.T_DEAD_ROT else 0.0)
        return 0.0, 0.0

    def step(self, dt):
        self.t += dt
        v_t, w_t = self._target()
        self.v += (v_t - self.v) * min(1.0, dt / self.TAU_FWD)
        self.omega += (w_t - self.omega) * min(1.0, dt / self.TAU_ROT)
        h = math.radians(self.heading)
        dpsi = self.omega * dt
        self.forward -= self.v * math.cos(h) * dt
        self.lateral += self.v * math.sin(h) * dt
        if dpsi:
            # 회전중심을 고정하고 돈다 — 카메라가 팔 A 로 호를 그린다 (계약 §1.2 dℓ/dψ=+A).
            # 2026-09-21: 여기만 부호가 반대여서 자기검증이 **거울상 플랜트** 위에서
            # 돌았다(10° 회전에 ±52 mm = 허용치의 1.7배). fake_rig·sim_plant 와 맞춘다.
            h2 = math.radians(self.heading + dpsi)
            self.forward -= self.A_M * (math.cos(h2) - math.cos(h))
            self.lateral += self.A_M * (math.sin(h2) - math.sin(h))
        self.heading += dpsi


class _MiniController:
    """DirectFrameForkliftController 의 얼굴만. SafeCanTx 가 여기까지만 닿는다."""

    def __init__(self, plant, ignore=False):
        self.plant, self.ignore = plant, ignore
        self.ch_a = None
        self._movement = "stop"
        self.sent = []

    @property
    def current_movement(self):
        return self._movement

    @current_movement.setter
    def current_movement(self, name):
        if name != self._movement:
            self.sent.append((self.plant.t, name))
        self._movement = name
        if not self.ignore:
            self.plant.set_cmd(name)


class _Est:
    """계약 §2.2 Estimate 의 얼굴만 한 가짜."""

    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


_TAG_CUT_M = 3.3          # 9/7 18:14 실측(카메라가 태그 중심보다 1.1 m 아래)
_BETA_VIS = 28.0          # HFOV 69°/2 − 태그 반각


def _make_est(plant, seq, rng, delta_deg=0.0, lost=False, ambiguous=False,
              gyro_alive=True, sign_fault=False, gyro_deg=0.0, low_noise=False,
              cut_m=None):
    d, lat, psi = plant.forward, plant.lateral, plant.heading
    sig_lat = 0.012 * max(d, 0.5) + 0.005          # 9/7: 7~10 m 에서 60~145 mm
    tilt = abs(math.degrees(math.atan2(lat, max(d, 0.1))))
    sig_psi = 1.0 if tilt >= 15.0 else 1.5         # 정면은 더 못 믿는다
    sig_beta = 0.3
    if low_noise:                                  # 근거리 태그·격자 캘리브 뒤를 흉내
        sig_lat, sig_psi, sig_beta = 0.002, 0.15, 0.05
    lat_m = lat + rng.gauss(0.0, sig_lat)
    x_m = max(0.05, d + rng.gauss(0.0, 0.01))
    psi_deg = psi + rng.gauss(0.0, sig_psi)
    # β 는 (I1) 로 만든다 — 계약 §1.4 실측 부호 그대로 (태그 오른쪽 ⇒ ℓ>0, β<0)
    beta = -math.degrees(math.atan2(lat_m, x_m)) - psi_deg - delta_deg
    if sign_fault:
        beta = -beta                                # 부호 사고 주입
    margin = max(0.0, (x_m - (_TAG_CUT_M if cut_m is None else cut_m)) * 250.0)
    seen = (not lost) and margin > 0.0 and abs(beta) <= _BETA_VIS
    return _Est(t=plant.t, t_pub=plant.t, seq=seq,
                x_m=x_m, lat_m=lat_m, psi_deg=psi_deg,
                v_mps=plant.v, omega_dps=plant.omega,
                beta_deg=beta if seen else float("nan"),
                beta_pred_deg=beta,
                sigma_x_m=0.01, sigma_lat_m=sig_lat, sigma_psi_deg=sig_psi,
                sigma_beta_deg=sig_beta, sigma_v_mps=0.01,
                A_m=0.0, sigma_A_m=0.5,
                tag_seen=seen, margin_px=margin if seen else float("nan"),
                tag_px=40.0, tilt_deg=tilt, reproj_px=0.3, err_ratio=0.5,
                heading_valid=(seen and tilt >= 15.0), lateral_valid=False,
                gyro_alive=gyro_alive, ambiguous=ambiguous, stale=False,
                anchor_age_s=0.0, gyro_gaps=0, reinit_count=0, n_frames=seq,
                degraded="", note="", gyro_deg=gyro_deg)


class _FakeCalib:
    """utils.calib.Calib 의 읽기 얼굴만 (자기검증에서 캘리브 경로까지 태우려고)."""

    def __init__(self, flat):
        self.flat = dict(flat)

    def get(self, dotted, default=None):
        v = self.flat.get(dotted, None)
        return default if v is None else v


def _tight_calib(tag_cut_m=3.3):
    """"σ_θ·σ_δ 를 실측해서 줄였을 때" 의 캘리브. 계약 §3.4 가 말한 유일한 탈출구다."""
    return {"perception": _FakeCalib({"dynamic.cam_yaw_offset_deg.value": 0.0,
                                      "dynamic.cam_yaw_offset_deg.sigma": 0.05,
                                      "dynamic.x_off_m.value": 0.0,
                                      "dynamic.A_m.value": 0.0, "dynamic.A_m.sigma": 0.2,
                                      "static.roll_deg.value": 0.0,
                                      "static.theta_min_deg.value": 15.0,
                                      "static.beta_vis_deg.L": 28.0,
                                      "static.pitch_px": 24.0}),
            "dynamics": _FakeCalib({"rot_closed.L.sigma_deg": 0.05,
                                    "rot_closed.R.sigma_deg": 0.05,
                                    "tag_cut.forward_m": tag_cut_m,
                                    "tag_cut.margin_px": 60.0,
                                    "min_inc.turn_deg": 2.0, "min_inc.fwd_97_m": 0.12,
                                    "v_mps.67.value": 0.28, "v_mps.97.value": 0.12})}


def _run(name, start=(6.0, 0.6, 0.0), final_anyway=True, cap_s=300.0, budget_s=None,
         lost_after=None, ambiguous_after=None, gyro_dead_in_final=False,
         sign_fault=False, no_response=False, low_noise=False, tight_calib=False,
         tag_cut_m=None, seed=3, verbose=False):
    """시나리오 하나. 계약 불변식을 돌면서 같이 감시한다."""
    import random
    from .can_tx import SafeCanTx
    rng = random.Random(seed)
    plant = _MiniPlant(*start, seed=seed)
    ctl = _MiniController(plant, ignore=no_response)
    tx = SafeCanTx(ctl, log=None, clock=lambda: plant.t)
    fsm = DockFSM(calibs=(_tight_calib(tag_cut_m if tag_cut_m else _TAG_CUT_M)
                          if tight_calib else None),
                  tx=tx, log=(print if verbose else (lambda *a, **k: None)),
                  final_anyway=final_anyway, clock=lambda: plant.t)
    if budget_s is not None:
        fsm.budget_s = budget_s
    fsm.arm(gate_ok=False, why="selftest")       # 캘리브 없음 = 내일 아침 상태
    fsm.assume_calib = True                      # 사람이 --assume-calib 를 줬다고 본다

    dt, t0 = 1.0 / 30.0, plant.t
    viol, zone_prev, bad_cmds = [], 0, []
    gyro0 = plant.heading
    while plant.t - t0 < cap_s and fsm.phase not in TERMINAL:
        plant.step(dt)
        seq = int((plant.t - t0) / dt)
        el = plant.t - t0
        est = _make_est(plant, seq, rng, low_noise=low_noise, cut_m=tag_cut_m,
                        lost=(lost_after is not None and el > lost_after),
                        ambiguous=(ambiguous_after is not None and el > ambiguous_after),
                        gyro_alive=not (gyro_dead_in_final and fsm.zone is Zone.FINAL),
                        sign_fault=sign_fault,
                        gyro_deg=plant.heading - gyro0 + plant.gyro_bias_dps * (plant.t - t0))
        before, mv_before = fsm.phase, tx.movement
        cmd = fsm.step(est)
        tx.check()
        # 불변식 1: 명령 래치는 DECIDE→EXECUTE 한 곳. 그 밖의 변화는 stop 뿐
        if tx.movement != mv_before and tx.movement != Movement.STOP.value:
            if not (before is Phase.DECIDE and cmd is not None
                    and cmd.movement == tx.movement):
                bad_cmds.append((plant.t, before.value, fsm.phase.value, tx.movement))
        # 불변식 2: 구역 단조
        if int(fsm.zone) < zone_prev:
            viol.append("구역 역행 %d → %d" % (zone_prev, int(fsm.zone)))
        zone_prev = int(fsm.zone)
    # 불변식 3: 전이표
    for _, a, b, _w in fsm.transitions:
        if b is not Phase.ABORT and b not in ALLOWED.get(a, set()):
            viol.append("전이표 밖 %s → %s" % (a.value, b.value))
    # ABORT 면 버스를 조용히 두지 않는지 본다
    resends0 = fsm.keeper.resends
    if fsm.phase is Phase.ABORT:
        for _ in range(60):
            plant.step(dt)
            fsm.step(_make_est(plant, 0, rng))
    return {"name": name, "phase": fsm.phase, "zone": fsm.zone, "fsm": fsm, "tx": tx,
            "ctl": ctl, "plant": plant, "t_s": plant.t - t0,
            "viol": viol, "bad_cmds": bad_cmds,
            "abort": getattr(fsm.abort_reason, "value", None),
            "zone_max": max(int(z) for z in (fsm.zone, Zone.FAR)),
            "resend_grew": fsm.keeper.resends > resends0,
            "summary": fsm.summary()}


def _selftest(verbose=False):
    ok_all, out = True, []

    def chk(label, cond, detail=""):
        nonlocal ok_all
        ok_all = ok_all and bool(cond)
        out.append("  %s %-58s %s" % ("OK  " if cond else "FAIL", label, detail))

    print("═══ dock_fsm 자기검증 ═══")

    # ── 1. 계약 대조 ────────────────────────────────────────────────────
    try:
        from .can_tx import SAFE_MOVEMENTS
        chk("Movement 값 == can_tx.SAFE_MOVEMENTS",
            set(m.value for m in Movement) == set(SAFE_MOVEMENTS),
            str(sorted(SAFE_MOVEMENTS)))
    except Exception as exc:
        chk("can_tx import", False, str(exc))
    chk("종단 상태에서 나가는 전이 없음",
        all(not ALLOWED[p] for p in TERMINAL))
    chk("EXECUTE 에서 갈 수 있는 곳은 STOPPING·ABORT 뿐",
        ALLOWED[Phase.EXECUTE] == {Phase.STOPPING, Phase.ABORT})
    chk("구역은 FAR<NEAR<FINAL", int(Zone.FAR) < int(Zone.NEAR) < int(Zone.FINAL))
    chk("FM 대장 11종 + FM8b 만 미완화",
        len(FAILURE_MODES) == 12
        and [c.value for c, m in FAILURE_MODES.items() if not m.mitigated] == ["FM8b"],
        "%d 종" % len(FAILURE_MODES))

    # ── 2. 3분할 판정 (plan 3-5 ②) ──────────────────────────────────────
    j = judge(0.005, 0.005, 0.030, 0.010)
    chk("수용: |e|+kσ ≤ T", j.accept and not j.act and not j.undecidable, str(j.as_row()))
    j = judge(0.100, 0.005, 0.030, 0.010)
    chk("행동: |e| > max(kσ, δ_q)", j.act and not j.accept)
    j = judge(0.005, 0.030, 0.030, 0.010)
    chk("판정불가: kσ > T/2", j.undecidable and not j.accept)
    j = judge(float("nan"), 0.005, 0.030, 0.010)
    chk("NaN 은 조용히 통과하지 않는다", j.undecidable and not j.accept and not j.act)

    # ── 3. 성공 판정 평행사변형 (plan 4-3) ──────────────────────────────
    m1 = parallelogram_margin(0.030, 1.7)
    m2 = parallelogram_margin(0.030, 0.0)
    m3 = parallelogram_margin(0.030, -0.5)
    chk("e_l=+c 에서 e_h=1.7° 는 허용", m1 >= -1e-4, "여유 %.4f m" % m1)
    chk("e_l=+c 에서 e_h=0° 도 허용(다이아몬드였다면 탈락)", m2 >= -1e-9, "여유 %.4f m" % m2)
    chk("e_l=+c 에서 반대쪽으로 기울면 탈락", m3 < 0, "여유 %.4f m" % m3)

    # ── 4. 부호 4항 (계약 §7.3) ─────────────────────────────────────────
    import random
    rng = random.Random(0)
    p = _MiniPlant(forward=5.0, lateral=+0.5, heading_deg=0.0)
    e = _make_est(p, 0, random.Random(1))
    chk("① 태그가 오른쪽 → ℓ>0 ∧ β<0", e.lat_m > 0 and e.beta_deg < 0,
        "ℓ %+.3f m, β %+.2f°" % (e.lat_m, e.beta_deg))
    p2 = _MiniPlant(forward=5.0, lateral=+0.5, heading_deg=0.0)
    b0 = _make_est(p2, 0, random.Random(1)).beta_deg
    h0 = p2.heading
    p2.set_cmd("rotate_ccw")
    for _ in range(80):            # 태그가 가시 범위(±28°) 안에 남을 만큼만 돈다
        p2.step(1 / 30.0)
    b1 = _make_est(p2, 1, random.Random(1)).beta_deg
    chk("② rotate_ccw(147) → ψ 증가 ∧ β 감소",
        p2.heading > h0 and b1 < b0, "Δψ %+.2f°, Δβ %+.2f°" % (p2.heading - h0, b1 - b0))
    p3 = _MiniPlant(forward=5.0, lateral=0.0, heading_deg=+5.0)
    l0, x0 = p3.lateral, p3.forward
    p3.set_cmd("forward")
    for _ in range(120):
        p3.step(1 / 30.0)
    chk("③ ψ>0 인 채 전진 → ℓ 증가 ∧ x 감소",
        p3.lateral > l0 and p3.forward < x0,
        "Δℓ %+.3f m, Δx %+.3f m" % (p3.lateral - l0, p3.forward - x0))
    resid = max(abs(_make_est(_MiniPlant(forward=f, lateral=l, heading_deg=h), 0,
                              random.Random(2), delta_deg=dd).beta_deg
                    + math.degrees(math.atan2(
                        _make_est(_MiniPlant(forward=f, lateral=l, heading_deg=h), 0,
                                  random.Random(2), delta_deg=dd).lat_m,
                        _make_est(_MiniPlant(forward=f, lateral=l, heading_deg=h), 0,
                                  random.Random(2), delta_deg=dd).x_m))
                    + _make_est(_MiniPlant(forward=f, lateral=l, heading_deg=h), 0,
                                random.Random(2), delta_deg=dd).psi_deg + dd)
                for f, l, h in ((5.0, 0.5, 0.0), (8.0, -1.5, 3.0), (3.5, 0.1, -2.0))
                for dd in (0.0, 3.0, -3.0))
    chk("④ (I1) 잔차 ≤ 0.2°", resid <= 0.2, "최대 %.4f°" % resid)

    # ── 5. 시나리오 ─────────────────────────────────────────────────────
    runs = []
    r = _run("명목 6 m (운영자 승인)", start=(6.0, 0.6, 0.0), final_anyway=True, verbose=verbose)
    runs.append(r)
    chk("명목: 종단에 든다", r["phase"] in TERMINAL, r["phase"].value)
    chk("명목: **DONE 이 아니라 DONE_UNVERIFIED** (승인 모드)",
        r["phase"] is Phase.DONE_UNVERIFIED, "%.0f s, %s" % (r["t_s"], r["phase"].value))
    chk("명목: 카메라 정지점이 안전선 뒤",
        r["plant"].forward >= 1.9, "x %.2f m (안전선 2.02)" % r["plant"].forward)

    r2 = _run("명목 8 m", start=(8.0, 1.5, 0.0), final_anyway=True)
    runs.append(r2)
    chk("8 m/ℓ1.5 m: 종단에 든다", r2["phase"] in TERMINAL,
        "%s %.0f s" % (r2["phase"].value, r2["t_s"]))

    r3 = _run("승인 없음 — wrong-DONE 금지", start=(6.0, 0.6, 0.0), final_anyway=False)
    runs.append(r3)
    chk("승인 없으면 **DONE 을 내지 않는다**(FM10 하드 KPI)",
        r3["phase"] is not Phase.DONE, "%s / %s" % (r3["phase"].value, r3["abort"]))

    r4 = _run("태그 실종", start=(6.0, 0.6, 0.0), lost_after=3.0, cap_s=120.0)
    runs.append(r4)
    chk("태그 실종 → REACQUIRE 뒤 ABORT(envelope)",
        r4["phase"] is Phase.ABORT and r4["abort"] == "envelope"
        and r4["summary"]["reacquire"] >= 1,
        "reacquire %d, %s" % (r4["summary"]["reacquire"], r4["abort"]))

    r5 = _run("AMBIGUOUS 지속", start=(6.0, 0.6, 0.0), ambiguous_after=2.0, cap_s=120.0)
    runs.append(r5)
    chk("AMBIGUOUS 10 s → FALLBACK 경유 ABORT(envelope)",
        r5["phase"] is Phase.ABORT and r5["abort"] == "envelope"
        and r5["summary"]["fallback"] >= 1,
        "fallback %d, %s/%s" % (r5["summary"]["fallback"], r5["phase"].value, r5["abort"]))

    r6 = _run("블라인드 중 자이로 사망", start=(4.0, 0.05, 0.0), gyro_dead_in_final=True)
    runs.append(r6)
    chk("FINAL 에서 자이로 죽으면 즉시 ABORT",
        r6["phase"] is Phase.ABORT and r6["abort"] == "gyro_dead_blind", str(r6["abort"]))
    chk("ABORT 뒤에도 stop 을 계속 보낸다(버스 침묵 금지)", r6["resend_grew"],
        "stop 재송신 %d" % r6["summary"]["abort_keeper"]["stop_resends"])

    r7 = _run("부호 사고 주입", start=(6.0, 0.6, 0.0), sign_fault=True, cap_s=30.0)
    runs.append(r7)
    chk("(I1) 잔차 > 0.2° → ABORT(sign_check)",
        r7["phase"] is Phase.ABORT and r7["abort"] == "sign_check", str(r7["abort"]))

    r8 = _run("시간 예산", start=(8.0, 1.5, 0.0), budget_s=25.0, cap_s=120.0)
    runs.append(r8)
    chk("시간 예산 초과 → ABORT(time_budget)",
        r8["phase"] is Phase.ABORT and r8["abort"] == "time_budget", str(r8["abort"]))

    r9 = _run("무응답(데드존)", start=(6.0, 0.6, 0.0), no_response=True, cap_s=120.0)
    runs.append(r9)
    chk("명령에 반응 없음 → FM4 뒤 ABORT(no_response)",
        r9["phase"] is Phase.ABORT and r9["abort"] == "no_response",
        "FM4 %d회" % r9["summary"]["failures"]["counts"].get("FM4", 0))

    r10 = _run("양자화 공백(σ 실측 뒤)", start=(3.55, 0.05, 0.0), final_anyway=False,
               low_noise=True, tight_calib=True, cap_s=120.0)
    runs.append(r10)
    chk("T 에서 낼 명령이 없으면 Tier 3 + '저속 회전 단 필요'",
        r10["phase"] is Phase.ABORT and r10["abort"] == "quant_gap",
        "%s / FM4 %d회" % (r10["abort"], r10["summary"]["failures"]["counts"].get("FM4", 0)))

    r11 = _run("σ 가 크면 판정불가 → Tier 사다리", start=(3.6, 0.05, 0.0), final_anyway=False,
               cap_s=200.0)
    runs.append(r11)
    chk("σ 가 크면 DONE 대신 no_convergence 로 선다",
        r11["phase"] is Phase.ABORT and r11["abort"] in ("no_convergence", "time_budget"),
        str(r11["abort"]))

    # 근거리 태그(plan 4-5) + σ 실측 구성 — **정식 DONE 이 나와야 한다**.
    # 이게 없으면 "wrong-DONE 0" 은 '아무것도 DONE 하지 않는 컨트롤러' 로도 만족된다
    r12 = _run("근거리 태그 + σ 실측 → 정식 DONE", start=(2.30, 0.004, 0.0),
               final_anyway=False, low_noise=True, tight_calib=True, tag_cut_m=0.5,
               cap_s=120.0)
    runs.append(r12)
    chk("수용식을 관측으로 통과하면 **DONE 을 낸다**",
        r12["phase"] is Phase.DONE, "%s / %s" % (r12["phase"].value, r12["abort"]))

    r13 = _run("FAR → NEAR 전환", start=(6.0, 2.0, 0.0), final_anyway=True,
               low_noise=True, tight_calib=True, tag_cut_m=0.5, cap_s=300.0)
    runs.append(r13)
    chk("σ(e_l) ≤ T/2 이고 oblique 면 NEAR 로 올라간다",
        r13["zone_max"] >= int(Zone.NEAR),
        "도달 구역 %s" % Zone(r13["zone_max"]).name)

    # ── 6. 불변식 (모든 시나리오 공통) ──────────────────────────────────
    chk("전이표 밖 전이 0", all(not r["viol"] for r in runs),
        str([v for r in runs for v in r["viol"]][:2]))
    chk("EXECUTE 중 stop 아닌 명령 0", all(not r["bad_cmds"] for r in runs),
        str([b for r in runs for b in r["bad_cmds"]][:2]))
    chk("내부 계약 위반(illegal) 0", all(r["summary"]["illegal"] == 0 for r in runs))
    chk("교착 없음 — 전부 캡 안에 종단",
        all(r["phase"] in TERMINAL for r in runs),
        str([r["name"] for r in runs if r["phase"] not in TERMINAL]))
    chk("SafeCanTx 밖 송신 0 (컨트롤러가 받은 명령 ⊆ 허용 동작)",
        all(all(n in set(m.value for m in Movement) for _t, n in r["ctl"].sent) for r in runs))
    dones = [r for r in runs if r["phase"] is Phase.DONE]
    chk("**wrong-DONE 0** — DONE 은 '수용식 통과 ∧ 관측 확인' 인 그 한 판에서만",
        [r["name"] for r in dones] == ["근거리 태그 + σ 실측 → 정식 DONE"],
        "DONE %d건 %s" % (len(dones), [r["name"] for r in dones]))
    chk("DONE 은 여유가 실제로 양수였다(평행사변형)",
        all((r["summary"]["error"] or {}).get("margin_m", -1) > 0 for r in dones),
        str([_r((r["summary"]["error"] or {}).get("margin_m"), 4) for r in dones]))

    print("\n".join(out))
    print("── 시나리오 요약 ──")
    for r in runs:
        s = r["summary"]
        e = s["error"] or {}
        print("  %-26s %-14s zone %-5s %5.1f s  전이 %3d  e_l %s σ %s  %s"
              % (r["name"], r["phase"].value, s["zone"], r["t_s"], s["transitions"],
                 _r(e.get("e_l"), 3), _r(e.get("sigma_e_l"), 3),
                 (s["failures"]["counts"] or "")))
    print("═══ %s ═══" % ("전부 통과" if ok_all else "실패 있음"))
    return 0 if ok_all else 1


if __name__ == "__main__":
    import sys
    sys.exit(_selftest(verbose="-v" in sys.argv))
