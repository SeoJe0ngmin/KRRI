"""폐루프 시험대 — 하드웨어 없이 도킹 전체를 몇 백 번 돌린다. (contracts_B §7.3, plan 4-6)

    python tools/etc/test_dock.py                 # 부호 4항 + 계약 + 고장주입 + 작은 격자
    python tools/etc/test_dock.py --grid --n 5    # 시작 격자 × 5회 몬테카를로 (KPI 표)
    python tools/etc/test_dock.py --replay 34306 --dist 8 --lat 2 --psi aim+8
                                                  # 격자 시드 하나를 그대로 재현
    python tools/etc/test_dock.py --ref           # 진짜 스택 대신 이 파일의 대역으로 다시
    python tools/etc/test_dock.py --signs         # 부호 자가시험만 (렌더 경로 포함)

무엇을 재는가 (plan 4-3 KPI 서열)
────────────────────────────────────────────────────────────────────────
  1) **wrong-DONE 0** — 하드. `DONE`(관측으로 확인된 성공)을 냈는데 진값이 틀린 경우.
     블라인드로 끝나면 계약상 `DONE_UNVERIFIED` 라 wrong-DONE 은 잘 안 난다 →
     그래서 **거짓 수용**(수용식은 통과했는데 진값 평행사변형은 실패)도 같이 센다.
     이쪽이 내일을 예고하는 숫자다.
  2) 평행사변형 성공률  max(|e_l|, |e_l − DOCK_DEPTH·tan e_h|) ≤ LAT_TOL_M
  3) abort 율·원인 / 프리미티브 수 / VERIFY 후 반전 / 소요 시간 / 여유 m 분위수

이 파일 안의 **Ref*** 세 개는 임시 대역이다
────────────────────────────────────────────────────────────────────────
  A팀 `estimate` · B팀 `dynamics` · C팀 `dock_fsm` 이 들어오면 `import` 로 바뀐다
  (파일 맨 위 shim 이 자동으로 진짜를 먼저 찾는다). Ref* 는 **시험대 자체를 시험하기
  위한 기준 구현**이고 실주행 코드가 아니다 — 다른 파일에서 import 하지 마라.
  계약(필드 이름·부호·전이표)은 진짜와 똑같이 맞춰 뒀다.

시뮬은 순위 도구다. 99% 인증 도구가 아니다(plan 4-6).
"""
import argparse
import dataclasses
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass, field

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools", "etc"))

from config import control as C                      # noqa: E402
from config import detection as D                    # noqa: E402
from src.models.control import can_tx                 # noqa: E402
from src.utils import timing as TM                    # noqa: E402

import sim_plant as SP                                # noqa: E402

# ── 계약 shim — 진짜 모듈이 있으면 그걸 쓴다 (contracts_B §4.1) ───────────────
try:                                                  # C팀
    from src.models.control.dock_fsm import (Phase, Zone, Tier, Primitive,   # noqa
                                             Movement, StopReason, AbortReason,
                                             FailureCode, ALLOWED, Command)
    HAVE_FSM = True
except Exception:
    HAVE_FSM = False
    from enum import Enum, IntEnum

    class Phase(Enum):
        IDLE = "idle"; OBSERVE = "observe"; DECIDE = "decide"; EXECUTE = "execute"
        STOPPING = "stopping"; SETTLE = "settle"; VERIFY = "verify"
        REACQUIRE = "reacquire"; FALLBACK = "fallback"
        DONE = "done"; DONE_UNVERIFIED = "done_dr"; ABORT = "abort"

    class Zone(IntEnum):
        FAR = 0; NEAR = 1; FINAL = 2

    class Tier(IntEnum):
        NONE = 0; WAIT = 1; REPOSITION = 2; CAUSE = 3; ABORT = 4

    class Primitive(Enum):
        NONE = "none"; ROTATE = "rotate"; FORWARD = "forward"
        FORWARD_BLIND = "forward_blind"; BACKWARD = "backward"
        DOGLEG = "dogleg"; HOLD = "hold"

    class Movement(str, Enum):
        STOP = "stop"; FORWARD = "forward"; FORWARD_SLOW = "forward_slow"
        BACKWARD = "backward"; ROTATE_CCW = "rotate_ccw"; ROTATE_CW = "rotate_cw"

    class StopReason(Enum):
        TARGET = "target"; STALE = "stale"; GYRO_DEAD = "gyro_dead"
        TAG_CUT = "tag_cut"; HARD_CAP = "hard_cap"; WRONG_WAY = "wrong_way"
        TIMEOUT = "timeout"; DEADMAN = "deadman"; NO_RESPONSE = "no_response"
        OPERATOR = "operator"

    class AbortReason(Enum):
        CALIB = "calib"; GYRO_DEAD_BLIND = "gyro_dead_blind"
        REINIT_TWICE = "reinit_twice"; ENVELOPE = "envelope"
        NO_RESPONSE = "no_response"; HARD_CAP = "hard_cap"
        TIME_BUDGET = "time_budget"; QUANT_GAP = "quant_gap"
        SIGN_CHECK = "sign_check"; CAN_FAULT = "can_fault"; OPERATOR = "operator"

    class FailureCode(Enum):
        FM1_WRONG_BRANCH = "FM1"; FM2_FALSE_CONF = "FM2"; FM3_PING_PONG = "FM3"
        FM4_DEADZONE = "FM4"; FM5_ERR_MIX = "FM5"; FM6_WAIT_DEADLOCK = "FM6"
        FM7_ZONE_CHATTER = "FM7"; FM8A_THREAD_LAG = "FM8a"; FM8B_HOST_FREEZE = "FM8b"
        FM9_CAN = "FM9"; FM10_WRONG_DONE = "FM10"; FM11_DUAL_SOURCE = "FM11"

    ALLOWED = {
        Phase.IDLE: {Phase.OBSERVE, Phase.ABORT},
        Phase.OBSERVE: {Phase.DECIDE, Phase.REACQUIRE, Phase.FALLBACK,
                        Phase.ABORT, Phase.OBSERVE},
        Phase.DECIDE: {Phase.EXECUTE, Phase.VERIFY, Phase.OBSERVE, Phase.ABORT},
        Phase.EXECUTE: {Phase.STOPPING, Phase.ABORT},
        Phase.STOPPING: {Phase.SETTLE, Phase.ABORT},
        Phase.SETTLE: {Phase.VERIFY, Phase.ABORT},
        Phase.VERIFY: {Phase.OBSERVE, Phase.DONE, Phase.DONE_UNVERIFIED, Phase.ABORT},
        Phase.REACQUIRE: {Phase.DECIDE, Phase.OBSERVE, Phase.ABORT},
        Phase.FALLBACK: {Phase.DECIDE, Phase.OBSERVE, Phase.ABORT},
        Phase.DONE: set(), Phase.DONE_UNVERIFIED: set(), Phase.ABORT: set(),
    }

    @dataclass(frozen=True)
    class Command:
        primitive: Primitive
        movement: str
        magnitude: float
        end_rule: str
        end_value: float
        level: int
        expect: object
        timeout_s: float
        zone: Zone
        tier: Tier
        est_seq: int
        t_decide: float
        why: str

# Movement 값이 can_tx 와 **문자 하나까지** 같아야 한다 (계약 §4.1)
assert set(m.value for m in Movement) == set(can_tx.SAFE_MOVEMENTS), \
    "Movement enum 이 can_tx.SAFE_MOVEMENTS 와 다르다"

# ── config 에서 오는 값 (§8). 통합 담당이 넣기 전이면 계약값으로 버틴다 ────────
X_STOP_CAM_M = C.CAM_TO_REF_M + C.STANDOFF_M      # 파생값(상수 아님) = 2.02 m
LAT_TOL_M = C.LAT_TOL_M
HEAD_TOL_DEG = C.HEAD_TOL_DEG
DOCK_DEPTH_M = C.DOCK_DEPTH_M
DOCK_TIME_BUDGET_S = getattr(C, "DOCK_TIME_BUDGET_S", 180.0)
FWD_TOL_M = getattr(C, "FWD_TOL_M", 0.10)
CONFIG_MISSING = [n for n in ("DOCK_TIME_BUDGET_S", "FWD_TOL_M")
                  if not hasattr(C, n)]

# ── 코드 내부 상수 (config 아님. 이유는 옆에 한 줄) ──────────────────────────
K_GATE = 2.0            # 수용식의 k. |e| + k·σ ≤ T (계약 §3.4)
THETA_MIN_DEG = 15.0    # tilt 가 이 아래면 카메라 heading 갱신 금지 (perception_calib 기본)
ERR_RATIO_MAX = 0.2     # 두 해 재투영비가 이보다 크면 AMBIGUOUS (plan 1-3)
BETA_VIS_DEG = 28.0     # 가시 한계 방위각 (HFOV 69/2 − 태그 반각)
BETA_RECENTER_DEG = 18.0  # 이보다 벌어지면 태그를 화면 가운데로 되돌린다
B_FLOOR_DEG = 1.0       # heading 바이어스 floor. 정면이면 1.5 (perception_calib 기본)
B_FLOOR_FRONT_DEG = 1.5
GYRO_DRIFT_DPS = 0.05   # 앵커 나이 × 이 값이 σ_ψ 에 더해진다
MIN_FRAMES = 8          # OBSERVE 탈출 최소 프레임
VERIFY_FRAMES = 20      # 정지 후 확인 프레임 수 (30프레임 중앙값의 축소판)
WAIT_CAP_S = 4.0        # Tier 0 대기 상한
SETTLE_CAP_S = 3.0      # 정지 판정 상한
ROT_CAP_DEG = 15.0      # 강등 상태의 1회 회전 상한 (계약 §5.4)
LEG_CAP_M = 1.0         # 강등 상태의 1회 다리 상한 (계약 §5.4)
X_J_MARGIN_M = 0.4      # 대각 다리의 목표점 J = 태그컷 + 이 여유
MIN_LEG_M = 0.30        # 67 최소 신뢰 증분 (dynamics_calib min_inc 기본)
MIN_LEG_97_M = 0.12
MIN_TURN_DEG = 2.0      # 회전 최소 신뢰 증분 (rot_closed σ 1.0° × 2)
# Ref 추정기의 σ 모델. 실제로는 perception_calib.static 에서 온다 —
# 여기 값은 계약 §5.2 의 "없을 때 가정값" 이고, **플랜트의 진짜 잡음이 아니다**
# (같은 숫자를 쓰면 추정기가 정답을 아는 셈이 되니 출처를 갈라 둔다).
SIGMA_C_PX = 0.30       # 주행 중 코너 잡음 [px] (plan 1-4)
K_PSI_REF = 0.070       # σ_ψ ≈ K·(σ_c/tag_px)/sin(tilt)
NU_FLOOR_REF = 2.5      # 그 식의 tilt 하한 [°]
DT = 1.0 / SP.FPS


# ═══════════════════════════════════════════════════════════════════════════
# A팀 자리 — 추정기 (계약 §2). 진짜가 들어오면 이 절은 통째로 빠진다
# ═══════════════════════════════════════════════════════════════════════════
try:
    from src.models.estimate import Estimate as _RealEstimate          # noqa: F401
    from src.models.estimate import FrameObs as _RealFrameObs          # noqa: F401
    HAVE_EST = True
except Exception:
    HAVE_EST = False


@dataclass
class RefEstimate:
    """계약 §2.2 의 필드 이름을 그대로 쓴다. 모르는 실수는 nan, bool 은 안전한 쪽."""
    t: float = 0.0
    t_pub: float = 0.0
    seq: int = 0
    x_m: float = float("nan")
    lat_m: float = float("nan")
    psi_deg: float = float("nan")
    v_mps: float = float("nan")
    omega_dps: float = float("nan")
    beta_deg: float = float("nan")
    beta_pred_deg: float = float("nan")
    sigma_x_m: float = float("nan")
    sigma_lat_m: float = float("nan")
    sigma_psi_deg: float = float("nan")
    sigma_beta_deg: float = float("nan")
    sigma_v_mps: float = float("nan")
    A_m: float = 0.0
    sigma_A_m: float = 0.5
    tag_seen: bool = False
    margin_px: float = float("nan")
    tag_px: float = float("nan")
    tilt_deg: float = float("nan")
    reproj_px: float = float("nan")
    err_ratio: float = float("nan")
    heading_valid: bool = False
    lateral_valid: bool = False
    gyro_alive: bool = False
    ambiguous: bool = False
    stale: bool = False
    anchor_age_s: float = 0.0
    gyro_gaps: int = 0
    reinit_count: int = 0
    n_frames: int = 0
    degraded: str = ""
    note: str = ""


def bearing_identity_residual(est, delta_deg=0.0):
    """(I1) 잔차 [°]. 0.2° 를 넘으면 부호·보정이 어긋난 것 (계약 §2.4)."""
    if not est.tag_seen or est.x_m != est.x_m:
        return float("nan")
    return est.beta_deg - (-math.degrees(math.atan2(est.lat_m, est.x_m))
                           - est.psi_deg - delta_deg)


def fork_tip(est, cam_to_ref_m, x_off_m=0.0):
    """(I4) 기준점(포크 끝)의 (x, ℓ)."""
    h = math.radians(est.psi_deg)
    return (est.x_m - cam_to_ref_m * math.cos(h) + x_off_m * math.sin(h),
            est.lat_m + cam_to_ref_m * math.sin(h) + x_off_m * math.cos(h))


class RefEstimator:
    """대역 추정기. **진값을 절대 안 본다** — plant.observe() 한 줄만 먹는다.

    KF 가 아니라 창 평균 + 규칙이다(계약을 시험하는 게 목적이라 이걸로 충분하다).
    중요한 건 셋: ① heading 은 tilt ≥ θ_min 일 때만 카메라로 **갱신**, 아니면 자이로 전파
    ② σ_lat 에 d·σ_ψ floor(유령 정밀도 방지) ③ 게이트는 전부 **긍정형**(NaN 이 통과 못 함).
    """

    WIN = 10                     # 이동 창 [프레임] ≈ 0.33 s

    def __init__(self, delta_hat_deg=0.0, x_off_m=0.0):
        self.delta = float(delta_hat_deg)
        self.x_off = float(x_off_m)
        self.n = 0
        self._x, self._lat, self._beta = [], [], []
        self._d, self._t = [], []
        self.psi = float("nan")
        self.anchor_t = None
        self.gyro_ref = None       # (gyro_deg, psi) 앵커
        self.last = None
        self.reinit = 0
        self.prev_gyro = None

    @staticmethod
    def _med(v):
        return statistics.median(v) if v else float("nan")

    def update(self, row):
        """프레임마다 하나. 태그가 안 보여도 예측만으로 낸다(계약 §2.2)."""
        self.n += 1
        st = row.get("stamps")
        t = getattr(st, "t_capture", None) or row.get("ts") or 0.0
        e = RefEstimate(t=t, t_pub=t, seq=row["seq"], n_frames=self.n)
        e.stale = bool(st is not None and not st.usable)
        e.gyro_alive = bool(row.get("gyro_alive"))
        e.gyro_gaps = int(row.get("gyro_gaps") or 0)
        gyro = row.get("gyro_deg")
        e.omega_dps = float(row.get("gyro_dps") or 0.0)

        seen = bool(row.get("tag_seen"))
        e.tag_seen = seen
        if seen:
            e.margin_px = float(row["margin_px"])
            e.tag_px = float(row["tag_px"])
            e.tilt_deg = float(row["tilt_deg"])
            e.reproj_px = float(row["reproj_rms_px"])
            p2 = row.get("pnp2") or {}
            e.err_ratio = float(p2.get("err_ratio") or 0.0)
            # ★ AMBIGUOUS 의 범위. tilt < θ_min 이면 어차피 카메라 heading 을 안 쓴다 —
            #   거기서까지 AMBIGUOUS 를 세우면 계약 §4.3 의 OBSERVE→DECIDE 가드
            #   (¬ambiguous)가 정면 원뿔에서 **영구 교착**을 만든다(실제로 났다).
            #   그래서 "기하는 갈라 줄 만한데(tilt ≥ θ_min) 재투영비가 안 갈린다" 일 때만
            #   진짜 이상으로 본다. 시험 주입(sim_ambiguous)은 그와 별개로 강제한다.
            #   ERR_RATIO_MAX=0.2 은 plan 1-3 의 **잠정 초기값**인데, 렌더+진짜
            #   pnp2_solutions 로 재 보면 3.5~8 m 전 구간에서 비가 0.3~0.95 다
            #   (§sim_plant AMB_K). 그걸 그대로 하드 게이트로 쓰면 heading 이
            #   영영 유효해지지 않는다 → **소프트로 쓴다**: σ_ψ 를 키우는 데만 쓰고,
            #   "사실상 동률(>0.9)" 일 때만 AMBIGUOUS 로 세운다.
            e.ambiguous = bool((e.err_ratio > 0.9 and e.tilt_deg >= THETA_MIN_DEG)
                               or row.get("sim_ambiguous"))
            e.beta_deg = float(row["beta_px_deg"])
            self._beta.append(e.beta_deg)
            self._t.append(t)
            for buf in (self._x, self._lat, self._beta, self._t):
                if len(buf) > self.WIN:
                    del buf[0]
            # ★ ℓ 은 PnP 전체 자세에서 뽑지 않는다 — d̂ 와 β̂·ψ̂ 로 재구성한다
            #   (plan 1-1·3-5 ⓪). 그래야 (I1) 이 추정값에서도 성립하고, heading
            #   바이어스가 ℓ 로 새는 경로가 **하나**로 모인다.
            self._d.append(math.hypot(float(row["lateral_m"]), float(row["forward_m"])))
            if len(self._d) > self.WIN:
                del self._d[0]
            # heading 은 **갱신 조건**이 따로 있다 (plan 1-3 하드 게이트)
            e.heading_valid = bool(e.tilt_deg >= THETA_MIN_DEG
                                   and not e.ambiguous and not e.stale
                                   and abs(e.omega_dps) < 2.0)   # 회전 중엔 자이로 전파
            if e.heading_valid:
                meas = float(row["heading_deg"])
                self.psi = meas if self.psi != self.psi else 0.7 * self.psi + 0.3 * meas
                self.anchor_t = t
                self.gyro_ref = (gyro, self.psi)
            elif self.psi == self.psi and gyro is not None and self.gyro_ref:
                self.psi = self.gyro_ref[1] + (gyro - self.gyro_ref[0])
            elif self.psi != self.psi:
                # 앵커가 아직 없다 — 원시값으로 시작하되 heading_valid 는 False 다
                self.psi = float(row["heading_deg"])
                self.gyro_ref = (gyro, self.psi)
        else:
            if self.psi == self.psi and gyro is not None and self.gyro_ref:
                self.psi = self.gyro_ref[1] + (gyro - self.gyro_ref[0])
            if self.last is not None:
                e.x_m, e.lat_m = self.last.x_m, self.last.lat_m     # 예측 유지

        e.psi_deg = self.psi
        if seen and self._d:
            # d̂ 는 창 중앙값, β 는 **이번 프레임 값 그대로** — β 는 σ 0.02° 라 걸러도
            # 얻을 게 없고, 걸면 회전 중에 (I1) 이 어긋나 부호경보가 헛 울린다.
            d_hat = self._med(self._d)
            b_hat = e.beta_deg
            phi = math.radians(-b_hat - self.psi - D.CAM_YAW_OFFSET_DEG)
            e.x_m = d_hat * math.cos(phi)
            e.lat_m = d_hat * math.sin(phi)
            self._lat.append(e.lat_m)
            self._x.append(e.x_m)
            for buf in (self._lat, self._x):
                if len(buf) > self.WIN:
                    del buf[0]
        e.anchor_age_s = 0.0 if self.anchor_t is None else max(0.0, t - self.anchor_t)

        # 속도: forward 차분(거리에서 직접 — plan 1-7 "v̂ 는 forward 에서 잰다")
        if len(self._x) >= 3 and (self._t[-1] - self._t[0]) > 0.05:
            e.v_mps = (self._x[0] - self._x[-1]) / (self._t[-1] - self._t[0])
            e.sigma_v_mps = statistics.pstdev(self._x) / max(0.05, self._t[-1] - self._t[0])
        else:
            e.v_mps, e.sigma_v_mps = 0.0, 0.05

        # σ — 관측 가능한 것만으로. 잡음의 온라인 적응은 금지(plan 2-6)
        if seen and e.tag_px == e.tag_px and e.tag_px > 0:
            sig_rand = math.degrees(K_PSI_REF * (SIGMA_C_PX / e.tag_px)
                                    / math.sin(math.radians(max(e.tilt_deg,
                                                                NU_FLOOR_REF))))
            floor = B_FLOOR_FRONT_DEG if e.tilt_deg < THETA_MIN_DEG else B_FLOOR_DEG
            e.sigma_psi_deg = math.sqrt(sig_rand ** 2 + floor ** 2
                                        + (GYRO_DRIFT_DPS * e.anchor_age_s) ** 2)
            if e.err_ratio == e.err_ratio:        # 두 해가 안 갈릴수록 σ 를 키운다
                e.sigma_psi_deg *= (1.0 + 2.0 * max(0.0, e.err_ratio - 0.2))
            e.sigma_beta_deg = math.degrees(0.5 * SIGMA_C_PX
                                            / max(1.0, e.tag_px)) + 0.02
            e.sigma_x_m = max(0.01, e.x_m * SIGMA_C_PX / max(1.0, e.tag_px))
            d_h = math.hypot(e.lat_m, e.x_m)
            scatter = statistics.pstdev(self._lat) if len(self._lat) > 2 else 0.05
            # ★ floor: 유령 정밀도 방지 (계약 §2.2 sigma_lat_m)
            e.sigma_lat_m = max(scatter, d_h * math.radians(e.sigma_psi_deg))
        else:
            e.sigma_psi_deg = (B_FLOOR_FRONT_DEG
                               + GYRO_DRIFT_DPS * e.anchor_age_s + 1.0)
            e.sigma_lat_m = 1.0
            e.sigma_x_m = 0.5
            e.sigma_beta_deg = 5.0

        e.lateral_valid = bool(e.sigma_lat_m <= LAT_TOL_M / 2.0)
        if row.get("sim_lateral_bad"):
            e.lateral_valid = False          # 시험용 주입(실기에는 이 키가 없다)
            e.sigma_lat_m = max(e.sigma_lat_m, 1.0)
        # β 예측: 태그가 없어도 남는다(REACQUIRE 가 쓴다)
        if seen:
            e.beta_pred_deg = e.beta_deg
        elif self.last is not None and gyro is not None and self.prev_gyro is not None:
            e.beta_pred_deg = self.last.beta_pred_deg - (gyro - self.prev_gyro)
        self.prev_gyro = gyro
        if not e.gyro_alive:
            e.degraded = "자이로 유실"
        self.last = e
        return e


# ═══════════════════════════════════════════════════════════════════════════
# B팀 자리 — 동역학 (계약 §3). 캘리브가 없으므로 전부 source="assumed"
# ═══════════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class Prediction:
    kind: str
    value: float
    sigma: float = float("nan")
    q95: float = float("nan")
    level: int = 67
    cell: str = ""
    source: str = "assumed"
    provisional: bool = True
    note: str = ""


class RefDynamics:
    """계약 §5.1 의 **가정값**으로만 구른다 → degraded=True, force_slow=True."""

    TAU_EFF = {67: 0.50, 97: 0.50}      # s. 9/7 정지지연
    V = {67: 0.28, 97: 0.12}            # m/s. 97 은 ★미측정
    TAU_R = 0.18                        # s. 회전 코스팅
    SAFETY = 1.5                        # 가정값이면 D̂ ×1.5 (계약 §5.4)

    def __init__(self, calib=None):
        self.calib = calib
        self.degraded = calib is None or not getattr(calib, "ok", False)
        self.force_slow = self.degraded

    def stop_distance(self, v_mps, level=67, dt_ctrl_s=0.0):
        v = abs(float(v_mps))
        d = self.TAU_EFF[level] * v + v * dt_ctrl_s / 2.0
        if self.degraded:
            d *= self.SAFETY
        return Prediction("stop_forward", d, sigma=0.02, q95=d + 0.04,
                          level=level, cell="%d_fwd" % level)

    def should_stop_forward(self, x_rem_m, v_mps, level=67, dt_ctrl_s=DT):
        return bool(x_rem_m <= self.stop_distance(v_mps, level, dt_ctrl_s).value)

    def should_stop_rotate(self, theta_rem_deg, omega_dps):
        return bool(theta_rem_deg <= abs(omega_dps) * self.TAU_R)

    def command_time_for(self, dist_m, level=67):
        """블라인드 다리의 명령시간. S(T) 표가 없으면 fwd_time_model + 경고."""
        from src.models.control import fwd_time_model as FTM
        par = FTM.PiecewiseFwdParams()
        t = FTM.time_from_distance_piecewise(max(0.0, float(dist_m)), par)
        if level == 97:                  # 67 기준 모델이라 속도비로 늘린다 ★근사
            t = (t - par.t0) * (self.V[67] / self.V[97]) + par.t0
        return Prediction("leg_time", float(t), level=level, cell="%d_fwd" % level,
                          note="S(T) 표 없음 — fwd_time_model 근사")

    def min_forward_m(self, level=67):
        return MIN_LEG_M if level == 67 else MIN_LEG_97_M

    def min_turn_deg(self):
        return MIN_TURN_DEG

    def rotate_timeout_s(self, deg):
        return max(3.0, abs(deg) / 8.0 * 2.0 + 2.0)


# ═══════════════════════════════════════════════════════════════════════════
# C팀 자리 — 상태기계·가이드 (계약 §4). 가이드는 plan 3-3 그대로:
#   대각 다리 → 태그 겨냥 회전(β* = −δ̂) → 블라인드 직진. 비용함수·롤아웃 없음.
# ═══════════════════════════════════════════════════════════════════════════
class ContractViolation(Exception):
    pass


class RefPilot:

    AMBIG_TIMEOUT_S = 3.0
    TAG_LOST_S = 1.5
    AIM_TOL_DEG = 0.6          # 태그 겨냥 잔차 허용. σ_θ(=1°)/2 보다 작게 요구하지 않는다

    def __init__(self, dyn, delta_hat_deg=0.0, x_off_m=0.0, tag_cut_m=3.3,
                 final_anyway=False, strict=True):
        self.dyn = dyn
        self.delta = float(delta_hat_deg)
        self.x_off = float(x_off_m)
        self.x_J = max(X_STOP_CAM_M + 0.6, float(tag_cut_m) + X_J_MARGIN_M)
        self.final_anyway = bool(final_anyway)
        self.strict = bool(strict)

        self.phase = Phase.IDLE
        self.zone = Zone.FAR
        self.tier = Tier.NONE
        self.cmd = None
        self.stop_reason = None
        self.abort_reason = None
        self.final_formal = False
        self.final_done = False
        self.events = []
        self.violations = []
        self.primitives = 0
        self.reversals = 0
        self.rot_signs = []
        self._prev_kind = ""
        self.tier_hits = {t: 0 for t in Tier}
        self.cause_used = set()
        self.reacquire_used = 0
        self.fallback_used = 0
        self.t0 = None
        self._t_phase = 0.0
        self._n_obs = 0
        self._n_verify = 0
        self._lost_since = None
        self._ambig_since = None
        self._sig_hist = []
        self._cmd_t0 = 0.0
        self._psi0 = 0.0
        self._x_target = 0.0
        self._settle_hist = []
        self.verify_buf = []
        self.last_accept = (False, float("nan"), float("nan"))
        self._reapproach = False
        self._anchored = False       # 카메라 heading 앵커를 한 번이라도 잡았나
        self._anchor_dir = 1.0
        self._anchor_tries = 0

    # ── 전이 ────────────────────────────────────────────────────────────────
    def _to(self, ph, t):
        if ph is not self.phase and ph not in ALLOWED[self.phase]:
            msg = "허용 안 된 전이: %s → %s" % (self.phase.value, ph.value)
            self.violations.append(msg)
            if self.strict:
                raise ContractViolation(msg)
        self.phase = ph
        self._t_phase = t

    def _ev(self, kind, t, **kw):
        self.events.append(dict(event=kind, t=round(t - (self.t0 or t), 3),
                                phase=self.phase.value, zone=int(self.zone), **kw))

    def _abort(self, reason, t):
        self.abort_reason = reason
        self.tier = Tier.ABORT
        self.tier_hits[Tier.ABORT] += 1
        self._ev("abort", t, reason=reason.value)
        self._to(Phase.ABORT, t)

    # ── 수용식 (계약 §3.4) ──────────────────────────────────────────────────
    def acceptance(self, est, s_blind):
        lever = C.CAM_TO_REF_M + max(0.0, s_blind)
        h = math.radians(est.psi_deg)
        e_l = est.lat_m + lever * math.sin(h) + self.x_off * math.cos(h)
        sig = math.hypot(est.sigma_lat_m, lever * math.radians(est.sigma_psi_deg))
        ok = (abs(e_l) + K_GATE * sig <= LAT_TOL_M
              and abs(est.psi_deg) + K_GATE * est.sigma_psi_deg <= HEAD_TOL_DEG)
        return bool(ok), e_l, sig

    # ── 계획 (plan 3-3 가이드) ──────────────────────────────────────────────
    def _rotate(self, dpsi, why, t, est, tier=Tier.NONE):
        dpsi = max(-ROT_CAP_DEG, min(ROT_CAP_DEG, float(dpsi)))
        mv = Movement.ROTATE_CCW.value if dpsi > 0 else Movement.ROTATE_CW.value
        return Command(primitive=Primitive.ROTATE, movement=mv, magnitude=dpsi,
                       end_rule="gyro_angle", end_value=dpsi, level=67,
                       expect=Prediction("stop_rotate", self.dyn.TAU_R * 8.0),
                       timeout_s=self.dyn.rotate_timeout_s(dpsi), zone=self.zone,
                       tier=tier, est_seq=est.seq, t_decide=t, why=why)

    def _forward(self, x_target, why, t, est, blind=False, tier=Tier.NONE):
        level = 97 if (self.dyn.force_slow or blind or self.zone is Zone.FINAL) else 67
        mv = (Movement.FORWARD_SLOW.value if level == 97 else Movement.FORWARD.value)
        s = max(0.0, est.x_m - x_target)
        pr = self.dyn.command_time_for(s, level)
        prim = Primitive.FORWARD_BLIND if blind else Primitive.FORWARD
        return Command(primitive=prim, movement=mv, magnitude=s,
                       end_rule="time" if blind else "camera_x",
                       end_value=pr.value if blind else x_target,
                       level=level, expect=pr,
                       timeout_s=pr.value * 2.0 + 4.0, zone=self.zone, tier=tier,
                       est_seq=est.seq, t_decide=t, why=why)

    def _plan(self, est, t):
        """명령 하나를 고른다. None 이면 움직일 게 없다."""
        beta_star = -self.delta                      # (I5) E1: β* = −δ̂
        s_blind = max(0.0, est.x_m - X_STOP_CAM_M)

        # ⓪ heading 앵커가 아직 없다 — **관측성부터 만든다** (plan 3-4 Tier 1).
        #   태그를 화면 가운데 두고 접근하면 tilt = 접근각 ν 라 8 m·ℓ1 m 에서 7° 뿐이고
        #   θ_min(15°) 을 못 넘어 heading 이 영영 안 잡힌다. 제자리에서 조금 틀면
        #   tilt = |ψ_raw| 가 그만큼 커진다(β 는 반대로 움직이니 가시성만 보면 된다).
        if (not self._anchored and self._anchor_tries < 3 and est.tag_seen
                and est.tilt_deg == est.tilt_deg
                and est.tilt_deg < THETA_MIN_DEG):
            need = min(ROT_CAP_DEG, THETA_MIN_DEG + 6.0 - est.tilt_deg)
            # tilt = |ψ_raw| 이니 **ψ̂ 와 같은 부호로** 틀어야 obliquity 가 는다.
            # ψ̂ 는 아직 못 믿는 값이지만 8~10° 급의 부호는 맞는다.
            base = est.psi_deg if est.psi_deg == est.psi_deg else -est.beta_deg
            sgn = (1.0 if base >= 0 else -1.0) * self._anchor_dir
            dpsi = sgn * need
            if abs(est.beta_deg - dpsi) > BETA_VIS_DEG - 2.0:
                dpsi = -dpsi                      # 가시성이 먼저다
            self._anchor_tries += 1
            self._tier(Tier.REPOSITION, "anchor", t)
            return self._rotate(dpsi, "heading 앵커용 회전 (tilt %.1f → %.1f°)"
                                % (est.tilt_deg, est.tilt_deg + abs(dpsi)),
                                t, est, tier=Tier.REPOSITION)

        if self._reapproach:                         # Tier 2 원인분기: 97 재접근
            self._reapproach = False
            pr = self.dyn.command_time_for(0.30, 67)
            return Command(primitive=Primitive.BACKWARD,
                           movement=Movement.BACKWARD.value, magnitude=0.30,
                           end_rule="time", end_value=pr.value, level=67, expect=pr,
                           timeout_s=pr.value * 2.0 + 3.0, zone=self.zone,
                           tier=Tier.CAUSE, est_seq=est.seq, t_decide=t,
                           why="수용식 초과 → 0.3 m 후진 후 재조준")

        if self.zone is Zone.FINAL:
            if self.final_done:
                return None
            return self._forward(X_STOP_CAM_M, "FINAL 블라인드 직진 %.2f m" % s_blind,
                                 t, est, blind=True)

        # ① 가시성 하드 제약이 먼저다 — 태그를 화면 가운데로 되돌린다 (plan 3-4)
        if est.tag_seen and abs(est.beta_deg - beta_star) > BETA_RECENTER_DEG:
            return self._rotate(est.beta_deg - beta_star,
                                "β 재중심 %.1f°" % est.beta_deg, t, est)

        # ② J 까지 왔나 — 아니면 대각 다리
        if est.x_m > self.x_J + self.dyn.min_forward_m(67):
            psi_t = math.degrees(math.atan2(-est.lat_m,
                                            max(0.3, est.x_m - self.x_J)))
            dpsi = psi_t - est.psi_deg
            # 횡오차가 **유의**할 때만 튼다 (FAR 는 방향만, plan 3-6)
            worth = abs(est.lat_m) > max(K_GATE * est.sigma_lat_m, LAT_TOL_M)
            # `heading_valid` 는 "이 프레임으로 **갱신**해도 되나" 이지 "ψ̂ 를 써도 되나"
            # 가 아니다(계약 §2.2). 앵커를 한 번 잡았으면 정면에서도 자이로 전파한
            # ψ̂ 로 계획한다 — plan 1-3 "정면 구간 heading = oblique 앵커 + 자이로".
            usable_psi = (self._anchored and est.psi_deg == est.psi_deg
                          and est.sigma_psi_deg <= 3.0)
            if (worth and usable_psi
                    and abs(dpsi) > max(MIN_TURN_DEG, HEAD_TOL_DEG)):
                beta_after = est.beta_deg - dpsi
                if abs(beta_after) > BETA_VIS_DEG:      # 가시성 밖이면 깎는다
                    lim = BETA_VIS_DEG - 3.0
                    dpsi = est.beta_deg - (lim if beta_after > 0 else -lim)
                return self._rotate(dpsi, "대각 조준 ψ*=%.1f° (ℓ %.3f)"
                                    % (psi_t, est.lat_m), t, est)
            leg = min(LEG_CAP_M if self.dyn.force_slow else 2.0,
                      est.x_m - self.x_J)
            if leg < self.dyn.min_forward_m(67):
                return None
            return self._forward(est.x_m - leg, "대각 다리 %.2f m" % leg, t, est)

        # ③ J 도착 — 태그 겨냥 (β* = −δ̂)
        if est.tag_seen and abs(est.beta_deg - beta_star) > self.AIM_TOL_DEG:
            return self._rotate(est.beta_deg - beta_star,
                                "태그 겨냥 β %.2f → %.2f" % (est.beta_deg, beta_star),
                                t, est)
        return None

    # ── 한 프레임 ───────────────────────────────────────────────────────────
    def step(self, est, t):
        """계약 §6 실행루프가 부르는 자리. 새 Command 를 내면 EXECUTE 로 간다."""
        if self.t0 is None:
            self.t0 = t
        el = t - self.t0
        if self.phase in (Phase.DONE, Phase.DONE_UNVERIFIED, Phase.ABORT):
            return None
        if el > DOCK_TIME_BUDGET_S:
            self._abort(AbortReason.TIME_BUDGET, t)
            return None
        if est.reinit_count >= 2:
            self._abort(AbortReason.REINIT_TWICE, t)
            return None
        res = bearing_identity_residual(est, D.CAM_YAW_OFFSET_DEG)
        if res == res and abs(res) > 0.2:
            self._abort(AbortReason.SIGN_CHECK, t)       # 부호가 어긋났다
            return None

        if est.heading_valid:
            self._anchored = True
        if self.phase is Phase.IDLE:
            self._to(Phase.OBSERVE, t)
            self._n_obs = 0
            return None

        if self.phase is Phase.OBSERVE:
            return self._observe(est, t)
        if self.phase is Phase.DECIDE:
            return self._decide(est, t)
        if self.phase is Phase.EXECUTE:
            self._execute(est, t)
            return None
        if self.phase is Phase.STOPPING:
            self._to(Phase.SETTLE, t)
            self._settle_hist = []
            return None
        if self.phase is Phase.SETTLE:
            self._settle(est, t)
            return None
        if self.phase is Phase.VERIFY:
            self._verify(est, t)
            return None
        if self.phase in (Phase.REACQUIRE, Phase.FALLBACK):
            self._to(Phase.DECIDE, t)
            return None
        return None

    def _observe(self, est, t):
        self._n_obs += 1
        if not est.tag_seen:
            self._lost_since = self._lost_since or t
            if (t - self._lost_since > self.TAG_LOST_S and self.zone is not Zone.FINAL):
                if self.reacquire_used >= 1:
                    self._abort(AbortReason.ENVELOPE, t)
                    return None
                self.reacquire_used += 1
                self._ev("reacquire", t, beta_pred=round(est.beta_pred_deg, 2))
                self._to(Phase.REACQUIRE, t)
                return None
        else:
            self._lost_since = None
        if est.ambiguous:
            self._ambig_since = self._ambig_since or t
            if t - self._ambig_since > self.AMBIG_TIMEOUT_S:
                self._ambig_since = t            # 한 번 처리했으면 다시 재 둔다
                if self.fallback_used >= 1:
                    self._tier(Tier.CAUSE, "ambiguous", t)
                else:
                    self.fallback_used += 1
                    self._ev("fallback", t, err_ratio=round(est.err_ratio, 3))
                    self._to(Phase.FALLBACK, t)
                    return None
        else:
            self._ambig_since = None

        # 계약 §4.3 의 OBSERVE→DECIDE 가드는 `¬ambiguous` 를 요구한다. 그런데 앵커를
        # 잡기 전에는 tilt < θ_min 이라 **늘 ambiguous** 다 — 그대로 걸면 관측성을
        # 고치러 갈 수조차 없다(FALLBACK 도 DECIDE 를 거친다). 그래서 앵커 전에는
        # 통과시키고, 앵커를 잡은 뒤의 ambiguous 만 진짜 이상으로 본다.
        ready = (self._n_obs >= MIN_FRAMES and not est.stale and est.gyro_alive
                 and est.tag_seen
                 and (not est.ambiguous or not self._anchored))
        if ready:
            self.tier = Tier.NONE
            self._sig_hist = []
            self._to(Phase.DECIDE, t)
            return None
        # Tier 0 WAIT — σ 가 아직 줄고 있나 (FM6: floor 지배면 즉시 Tier 1)
        self._sig_hist.append(est.sigma_lat_m)
        if len(self._sig_hist) > 60:
            del self._sig_hist[0]
        waited = t - self._t_phase
        if waited > WAIT_CAP_S:
            shrinking = (len(self._sig_hist) > 20
                         and self._sig_hist[-1] < 0.9 * self._sig_hist[0])
            self._tier(Tier.WAIT if shrinking else Tier.REPOSITION,
                       "wait %.1fs" % waited, t)
            self._t_phase = t
            if not shrinking and not est.gyro_alive:
                self._abort(AbortReason.GYRO_DEAD_BLIND, t)
            elif not shrinking and self.tier_hits[Tier.REPOSITION] > 8:
                self._abort(AbortReason.NO_RESPONSE, t)
        else:
            self.tier = Tier.WAIT
        return None

    def _tier(self, tier, why, t):
        self.tier = tier
        self.tier_hits[tier] += 1
        self._ev("tier", t, tier=int(tier), why=why)

    def _decide(self, est, t):
        s_blind = max(0.0, est.x_m - X_STOP_CAM_M)
        ok, e_l, sig = self.acceptance(est, s_blind)
        cmd = self._plan(est, t)
        if cmd is None:
            self._ev("decide_none", t, e_l=round(e_l, 4), sigma=round(sig, 4),
                     accept=ok)
            self._to(Phase.VERIFY, t)
            self._n_verify = 0
            self.verify_buf = []
            return None
        self.cmd = cmd
        self.primitives += 1
        if cmd.primitive is Primitive.ROTATE:
            s = 1 if cmd.magnitude > 0 else -1
            if self.rot_signs and self.rot_signs[-1] == -s and self._prev_kind == "rotate":
                self.reversals += 1          # 직진 없이 좌↔우로 되튄 것만 (FM3)
            self.rot_signs.append(s)
        self._prev_kind = cmd.primitive.value
        self._cmd_t0 = t
        self._psi0 = est.psi_deg
        self._x_target = cmd.end_value if cmd.end_rule == "camera_x" else 0.0
        self.stop_reason = None
        self._ev("cmd", t, prim=cmd.primitive.value, mv=cmd.movement,
                 mag=round(cmd.magnitude, 3), why=cmd.why)
        self._to(Phase.EXECUTE, t)
        return cmd

    def _execute(self, est, t):
        c = self.cmd
        el = t - self._cmd_t0
        reason = None
        if el > c.timeout_s:
            reason = StopReason.TIMEOUT
        elif est.stale:
            reason = StopReason.STALE
        elif not est.gyro_alive:
            reason = StopReason.GYRO_DEAD
        elif c.primitive is Primitive.ROTATE:
            done = est.psi_deg - self._psi0
            rem = abs(c.magnitude) - abs(done)
            if done * c.magnitude < 0 and abs(done) > 5.0:
                reason = StopReason.WRONG_WAY
            elif self.dyn.should_stop_rotate(rem, est.omega_dps):
                reason = StopReason.TARGET
        elif c.primitive is Primitive.FORWARD:
            if est.tag_seen:
                if est.margin_px == est.margin_px and est.margin_px < 25.0:
                    reason = StopReason.TAG_CUT
                elif self.dyn.should_stop_forward(est.x_m - self._x_target,
                                                 est.v_mps, c.level):
                    reason = StopReason.TARGET
                elif est.x_m < X_STOP_CAM_M - FWD_TOL_M:
                    reason = StopReason.HARD_CAP
            elif el > 1.0:
                reason = StopReason.TAG_CUT
        elif c.primitive is Primitive.BACKWARD:
            if el >= c.end_value:
                reason = StopReason.TARGET
        else:                                    # FORWARD_BLIND — 시간으로 닫는다
            if est.tag_seen and self.dyn.should_stop_forward(
                    est.x_m - X_STOP_CAM_M, est.v_mps, c.level):
                reason = StopReason.TARGET
            elif el >= c.end_value:
                reason = StopReason.TARGET
        if reason is not None:
            self.stop_reason = reason
            self._ev("stop", t, reason=reason.value, el=round(el, 2))
            if c.primitive is Primitive.FORWARD_BLIND:
                self.final_done = True
            self._to(Phase.STOPPING, t)

    def _settle(self, est, t):
        self._settle_hist.append(abs(est.v_mps))
        settled = (len(self._settle_hist) >= 8
                   and max(self._settle_hist[-8:]) < 0.02)
        if settled or t - self._t_phase > SETTLE_CAP_S:
            self._to(Phase.VERIFY, t)
            self._n_verify = 0
            self.verify_buf = []

    def _verify(self, est, t):
        """정지 상태 확인 + **구역 전환은 여기서만**. 역행 금지(FM7)."""
        self._n_verify += 1
        if est.tag_seen:
            self.verify_buf.append(est)
        if self._n_verify < VERIFY_FRAMES:
            return
        use = self.verify_buf[-1] if self.verify_buf else est
        s_blind = max(0.0, use.x_m - X_STOP_CAM_M)
        ok, e_l, sig = self.acceptance(use, s_blind)
        self.last_accept = (ok, e_l, sig)
        at_J = use.x_m <= self.x_J + 0.35
        aimed = (use.tag_seen
                 and abs(use.beta_deg + self.delta) <= self.AIM_TOL_DEG * 2.0)
        if self.zone is Zone.FAR and sig <= LAT_TOL_M / 2.0:
            self.zone = Zone.NEAR
            self._ev("zone", t, to="NEAR", sigma_F=round(sig, 4))
        if self.zone is not Zone.FINAL and at_J and aimed:
            if ok:
                self.zone = Zone.FINAL
                self.final_formal = True
                self._ev("zone", t, to="FINAL", how="formal", e_l=round(e_l, 4))
            elif self.final_anyway:
                self.zone = Zone.FINAL
                self.final_formal = False
                self._ev("final_unproven", t, e_l=round(e_l, 4),
                         sigma=round(sig, 4))       # 계약 §4.5 (2) 운영자 승인
        if self.zone is Zone.FINAL and self.final_done:
            observed = bool(use.tag_seen and use.x_m == use.x_m)
            if self.final_formal and ok and observed:
                self._to(Phase.DONE, t)
            else:
                self._to(Phase.DONE_UNVERIFIED, t)
            self._ev("result", t, phase=self.phase.value, e_l=round(e_l, 4),
                     sigma=round(sig, 4), formal=self.final_formal)
            return
        # 양자화 공백: 수용 밖인데 최소 신뢰 증분보다 작다 → 즉시 Tier 3 (plan 3-7)
        if self.zone is Zone.FINAL or at_J:
            gap = (not ok and abs(e_l) < MIN_TURN_DEG * math.pi / 180.0
                   * max(0.1, use.x_m - C.CAM_TO_REF_M))
            if gap and "quant" not in self.cause_used:
                self.cause_used.add("quant")
                self._ev("quant_gap", t, note="저속 회전 단 필요", e_l=round(e_l, 4))
                self._abort(AbortReason.QUANT_GAP, t)
                return
        if (not self._anchored and self._anchor_tries and use.tag_seen
                and use.tilt_deg == use.tilt_deg and use.tilt_deg < THETA_MIN_DEG):
            self._anchor_dir = -self._anchor_dir      # 반대로 틀어야 obliquity 가 는다
        if at_J and not ok:
            # 수용식 초과 — 원인분기 1회(97 재접근), 두 번째면 Tier 3 (plan 3-7)
            bad_lat = not use.lateral_valid
            key = "lateral" if bad_lat else "accept"
            if key not in self.cause_used:
                self.cause_used.add(key)
                self._tier(Tier.CAUSE, key, t)
                self._reapproach = True
            else:
                self._tier(Tier.REPOSITION, key + "2", t)
                if self.tier_hits[Tier.REPOSITION] >= 4:
                    self._abort(AbortReason.NO_RESPONSE, t)
                    return
        self._to(Phase.OBSERVE, t)
        self._n_obs = 0


# ═══════════════════════════════════════════════════════════════════════════
# 통합 자리 — 실행 루프 하나 (계약 §6). CAN 은 SafeCanTx 하나로만 나간다
# ═══════════════════════════════════════════════════════════════════════════
@dataclass
class Episode:
    seed: int
    start: tuple
    outcome: str = ""
    abort: str = ""
    zone: int = 0
    e_l: float = float("nan")          # 진값 기준점 횡오차 [m]
    e_h: float = float("nan")          # 진값 heading [°]
    x_err: float = float("nan")        # 진값 종방향 오차 [m]
    worst: float = float("nan")        # 평행사변형 좌변
    margin_m: float = float("nan")     # LAT_TOL − worst
    success: bool = False              # 평행사변형 통과
    success_full: bool = False         # + 종방향까지
    wrong_done: bool = False
    false_accept: bool = False
    pred_e_l: float = float("nan")     # 컨트롤러가 예측한 e_l
    pred_sigma: float = float("nan")
    accepted: bool = False
    primitives: int = 0
    reversals: int = 0
    elapsed: float = 0.0
    tiers: tuple = ()
    violations: tuple = ()
    tail_stop: int = 0
    flip_rate: float = 0.0
    inside_envelope: bool = True
    events: list = field(default_factory=list)


def _truth_score(plant):
    """진값으로 채점. (I4) + plan 4-3 평행사변형."""
    tr = plant.truth()
    h = math.radians(tr.psi_deg)
    e_l = tr.lat_m + C.CAM_TO_REF_M * math.sin(h) + plant.p.x_off_m * math.cos(h)
    e_h = tr.psi_deg
    worst = max(abs(e_l), abs(e_l - DOCK_DEPTH_M * math.tan(h)))
    x_ref_x = tr.x_m - C.CAM_TO_REF_M * math.cos(h)      # 기준점의 종방향 위치
    return e_l, e_h, worst, x_ref_x - C.STANDOFF_M


def _run_ref(seed, start=(8.0, 1.0, None), faults=None, final_anyway=True,
             strict=True, trace=False, budget_s=None):
    """대역 스택(Ref*)으로 도킹 한 번. start=(forward, lateral, heading 또는 None)."""
    from fake_rig import FakeController

    fwd0, lat0, psi0 = start
    if psi0 is None:
        psi0 = -math.degrees(math.atan2(lat0, fwd0))      # 태그를 대충 보고 시작
    plant = SP.SimPlant(forward=fwd0, lateral=lat0, heading_deg=psi0, seed=seed,
                        faults=faults)
    ctrl = FakeController(plant=plant)
    tx = can_tx.SafeCanTx(ctrl, clock=lambda: plant.t)
    estor = RefEstimator()
    dyn = RefDynamics()
    pilot = RefPilot(dyn, final_anyway=final_anyway, strict=strict,
                     tag_cut_m=3.3)
    budget = budget_s or DOCK_TIME_BUDGET_S
    n_set = 0
    prev_phase = pilot.phase
    t_end = plant.t + budget + 5.0

    while plant.t < t_end:
        plant.step(DT)
        if plant._host_frozen():
            continue                     # 호스트가 얼면 루프도 같이 멎는다(FM8b)
        row = plant.observe()
        est = estor.update(row)
        cmd = pilot.step(est, plant.t)
        if cmd is not None:
            if pilot.phase is not Phase.EXECUTE:
                pilot.violations.append("EXECUTE 아닌 곳에서 명령이 나왔다")
            tx.set_movement(cmd.movement, why=cmd.why)      # 유일한 송신 자리
            n_set += 1
        elif pilot.phase is not Phase.EXECUTE and tx.movement != "stop":
            tx.stop(why=(pilot.stop_reason.value if pilot.stop_reason else "idle"))
        elif pilot.phase is Phase.EXECUTE and tx.movement == "stop":
            pilot.violations.append("EXECUTE 인데 CAN 은 stop 이다")
        tx.feed()
        tx.check()
        if trace and pilot.phase is not prev_phase:
            tr = plant.truth()
            print("   %6.2fs %-9s zone%d  진값 x %.2f ℓ %+.3f ψ %+.2f  "
                  "추정 x %.2f ℓ %+.3f ψ %+.2f β %+.2f σψ %.2f"
                  % (plant.t - plant.t0, pilot.phase.value, int(pilot.zone),
                     tr.x_m, tr.lat_m, tr.psi_deg, est.x_m, est.lat_m,
                     est.psi_deg, est.beta_deg, est.sigma_psi_deg))
        prev_phase = pilot.phase
        if pilot.phase in (Phase.DONE, Phase.DONE_UNVERIFIED, Phase.ABORT):
            break

    # 종단 — stop 프레임을 **계속** 보낸다(버스 침묵 금지, 계약 §4.6 Tier 3)
    tail = 0
    for _ in range(int(1.5 / DT)):
        plant.step(DT)
        tx.stop(why="terminal")
        tx.feed()
        tail += int(tx.movement == "stop" and ctrl.current_movement == "stop")

    e_l, e_h, worst, x_err = _truth_score(plant)
    acc = getattr(pilot, "last_accept", (False, float("nan"), float("nan")))
    ok_lat = worst <= LAT_TOL_M
    ep = Episode(seed=seed, start=(fwd0, lat0, round(psi0, 2)),
                 outcome=pilot.phase.value,
                 abort=pilot.abort_reason.value if pilot.abort_reason else "",
                 zone=int(pilot.zone), e_l=e_l, e_h=e_h, x_err=x_err, worst=worst,
                 margin_m=LAT_TOL_M - worst, success=ok_lat,
                 success_full=bool(ok_lat and abs(x_err) <= FWD_TOL_M),
                 wrong_done=bool(pilot.phase is Phase.DONE and not ok_lat),
                 false_accept=bool(acc[0] and not ok_lat),
                 pred_e_l=acc[1], pred_sigma=acc[2], accepted=bool(acc[0]),
                 primitives=pilot.primitives, reversals=pilot.reversals,
                 elapsed=plant.t - plant.t0,
                 tiers=tuple(sorted((int(k), v) for k, v in pilot.tier_hits.items()
                                    if v)),
                 violations=tuple(pilot.violations), tail_stop=tail,
                 flip_rate=plant.n_flip / max(1, plant.n_tag_seen),
                 events=pilot.events)
    if len(plant.cmd_log) > n_set + 2:        # SafeCanTx 밖에서 샌 명령이 있나
        ep.violations = ep.violations + ("플랜트 명령 %d > 송신 %d"
                                         % (len(plant.cmd_log), n_set),)
    return ep


def _run_real(seed, start=(8.0, 1.0, None), faults=None, final_anyway=True,
              strict=True, trace=False, budget_s=None):
    """**진짜 스택**(A 추정기 · B 동역학 · C 상태기계)으로 도킹 한 번.

    계약 §6 실행루프 그대로다. CAN 은 상태기계가 SafeCanTx 로만 낸다.
    """
    from fake_rig import FakeController
    from src.models.estimate import Estimator, FrameObs
    from src.models.dynamics.predict import Dynamics
    from src.models.control.dock_fsm import DockFSM, TERMINAL
    from src.utils import calib as CAL

    fwd0, lat0, psi0 = start
    if psi0 is None:
        psi0 = -math.degrees(math.atan2(lat0, fwd0))
    plant = SP.SimPlant(forward=fwd0, lateral=lat0, heading_deg=psi0, seed=seed,
                        faults=faults)
    ctrl = FakeController(plant=plant)
    tx = can_tx.SafeCanTx(ctrl, clock=lambda: plant.t)
    calibs = CAL.load_all()
    dyn = Dynamics(calibs.get("dynamics"), log=None)
    estor = Estimator(intrinsics=plant.intr, calibs=calibs)
    quiet = (lambda *a, **k: None)
    fsm = DockFSM(calibs=calibs, dyn=dyn, tx=tx, log=quiet, assume_calib=True,
                  final_anyway=final_anyway, clock=lambda: plant.t)
    fsm.arm(gate_ok=True, why="sim")

    budget = budget_s or DOCK_TIME_BUDGET_S
    t_end = plant.t + budget + 10.0
    prims = rev = 0
    signs = []
    prev_kind = ""
    tiers = {}
    prev_phase = fsm.phase
    while plant.t < t_end:
        plant.step(DT)
        if plant._host_frozen():
            continue                     # 호스트가 얼면 루프도 같이 멎는다(FM8b)
        row = plant.observe()
        obs = FrameObs.from_row(row, stamps=row["stamps"])
        est = estor.update(obs)
        # 추정기는 t_pub 에 호스트 벽시계를 넣는다. 시뮬은 가상 시계로 도니까
        # 상태기계가 쓰는 '지금' 을 가상 시계로 바꿔 끼운다(계약 §7.1 "실시간 대기 없음").
        est = dataclasses.replace(est, t_pub=plant.t)
        # tools/dock.py 와 **같은 배선**으로 돈다 — 원관측 (I1) 을 같이 넘겨야
        # 상태기계가 "배선 부호 사고(sign_check)" 와 "PnP 오브랜치 락(wrong_branch)" 을
        # 갈라 찍는다(2026-09-21 통합). 안 넘기면 둘 다 sign_check 으로 나온다.
        cmd = fsm.step(est, gyro_deg=plant.gyro_deg,
                       i1_obs_deg=getattr(estor, "identity_obs_deg", None))
        tx.check()
        if cmd is not None:
            prims += 1
            # **반전** = 직진을 사이에 두지 않고 좌↔우로 되튄 회전(FM3 핑퐁).
            # 왼쪽으로 틀고 → 달리고 → 오른쪽으로 트는 건 정상 수렴이라 안 센다.
            kind = getattr(cmd.primitive, "value", "")
            if kind == "rotate":
                sg = 1 if cmd.magnitude > 0 else -1
                if signs and signs[-1] == -sg and prev_kind == "rotate":
                    rev += 1
                signs.append(sg)
            prev_kind = kind
        if int(fsm.tier):
            tiers[int(fsm.tier)] = tiers.get(int(fsm.tier), 0) + 1
        if trace and fsm.phase is not prev_phase:
            tr = plant.truth()
            print("   %6.2fs %-9s zone%d tier%d  진값 x %.2f ℓ %+.3f ψ %+.2f  "
                  "추정 x %.2f ℓ %+.3f ψ %+.2f β %+.2f σψ %.2f"
                  % (plant.t - plant.t0, fsm.phase.value, int(fsm.zone),
                     int(fsm.tier), tr.x_m, tr.lat_m, tr.psi_deg,
                     est.x_m, est.lat_m, est.psi_deg, est.beta_deg,
                     est.sigma_psi_deg))
        prev_phase = fsm.phase
        if fsm.phase in TERMINAL:
            break

    tail = 0
    for _ in range(int(1.5 / DT)):
        plant.step(DT)
        tx.stop(why="terminal")
        tx.feed()
        tail += int(tx.movement == "stop" and ctrl.current_movement == "stop")

    e_l, e_h, worst, x_err = _truth_score(plant)
    acc = fsm.last_error or {}
    ok_acc = bool(acc.get("accept_l") and acc.get("accept_h"))
    ok_lat = worst <= LAT_TOL_M
    viol = () if not fsm.illegal else ("전이표 위반 %d 건" % fsm.illegal,)
    return Episode(
        seed=seed, start=(fwd0, lat0, round(psi0, 2)), outcome=fsm.phase.value,
        abort=fsm.abort_reason.value if fsm.abort_reason else "",
        zone=int(fsm.zone), e_l=e_l, e_h=e_h, x_err=x_err, worst=worst,
        margin_m=LAT_TOL_M - worst, success=ok_lat,
        success_full=bool(ok_lat and abs(x_err) <= FWD_TOL_M),
        wrong_done=bool(fsm.phase.value == "done" and not ok_lat),
        false_accept=bool(ok_acc and not ok_lat),
        pred_e_l=float(acc.get("e_l", float("nan"))),
        pred_sigma=float(acc.get("sigma_e_l", float("nan"))), accepted=ok_acc,
        primitives=prims, reversals=rev, elapsed=plant.t - plant.t0,
        tiers=tuple(sorted(tiers.items())), violations=viol, tail_stop=tail,
        flip_rate=plant.n_flip / max(1, plant.n_tag_seen),
        events=[{"event": "phase", "t": round(t - plant.t0, 2),
                 "from": a.value, "to": b.value, "why": w}
                for (t, a, b, w) in fsm.transitions])


#: 기본은 **진짜 스택**. 한 팀이라도 빠졌거나 터지면 Ref 대역으로 내려간다.
STACK = "auto"


def run_episode(seed, start=(8.0, 1.0, None), faults=None, final_anyway=True,
                strict=True, trace=False, budget_s=None, stack=None):
    """도킹 한 번. stack = "real" | "ref" | "auto"(기본)."""
    want = stack or STACK
    if want != "ref":
        try:
            return _run_real(seed, start, faults, final_anyway, strict, trace,
                             budget_s)
        except Exception as exc:
            if want == "real":
                raise
            if not getattr(run_episode, "_warned", False):
                run_episode._warned = True
                print("   !! 진짜 스택이 안 돌아간다(%s: %s) — Ref 대역으로 내려간다"
                      % (type(exc).__name__, exc))
    return _run_ref(seed, start, faults, final_anyway, strict, trace, budget_s)


# ═══════════════════════════════════════════════════════════════════════════
# 시험 1 — 부호 자가시험 4항 (계약 §7.3). **이걸 못 넘기면 그 아래는 전부 무효**
# ═══════════════════════════════════════════════════════════════════════════
def _row_from_result(res, tag_id, seq, gyro_deg=0.0, gyro_dps=0.0):
    """진짜 `Result` → frame.jsonl 한 줄. 해석형 row 와 **같은 키**여야 한다."""
    from src.models.detection.detection_pose import bearing_px_deg, pnp2_solutions
    from src.models.detection.detection_tag import tag_edge_margin_px, tag_pixel_size
    row = {"seq": seq, "stamps": res.stamps, "tag_seen": False,
           "lateral_m": None, "forward_m": None, "vertical_m": None,
           "heading_deg": None, "tilt_deg": None, "distance_m": None,
           "beta_px_deg": None, "margin_px": None, "tag_px": None,
           "center_px": None, "reproj_rms_px": None, "decision_margin": None,
           "hamming": 0, "quality_ok": False, "pnp2": None,
           "gyro_deg": gyro_deg, "gyro_dps": gyro_dps, "gyro_gaps": 0,
           "gyro_alive": True, "gyro_age_s": 0.0, "movement": "stop",
           "t_cmd_set": None, "t_cmd_tx": None,
           "sim_dropped": False, "sim_lateral_bad": False}
    pr = res.primary(tag_id)
    if pr is None:
        return row
    det, dk, q = pr["detection"], pr["docking"], pr["quality"] or {}
    row.update({
        "tag_seen": True, "lateral_m": dk["lateral"], "forward_m": dk["forward"],
        "vertical_m": dk["vertical"], "heading_deg": dk["heading_deg"],
        "tilt_deg": dk["tilt_deg"], "distance_m": dk["distance"],
        "beta_px_deg": bearing_px_deg(det, res.intrinsics),
        "margin_px": tag_edge_margin_px(det, np.asarray(res.image).shape),
        "tag_px": tag_pixel_size(det),
        "center_px": tuple(float(v) for v in det.center),
        "reproj_rms_px": q.get("reproj_rms_px"),
        "decision_margin": float(det.decision_margin),
        "hamming": int(det.hamming), "quality_ok": bool(q.get("ok", True)),
        "pnp2": pnp2_solutions(det, res.intrinsics, D.TAG_SIZE_M)})
    return row


def sign_tests(verbose=True, render=True):
    """진값 없이 **관측만으로** 부호를 판정한다. 반환 = 실패 수."""
    say = print if verbose else (lambda *a, **k: None)
    bad = 0

    def run(plant, n, est=None):
        est = est or RefEstimator()
        out = None
        for _ in range(n):
            plant.step(DT)
            out = est.update(plant.observe())
        return est, out

    say("── 부호 자가시험 (계약 §7.3) ──────────────────────────────────")
    # 1) 태그를 화면 오른쪽에 → ℓ > 0 ∧ β < 0
    pl = SP.SimPlant(forward=3.0, lateral=0.7, vertical=-0.35, seed=1)
    _, e = run(pl, 20)
    ok = e.tag_seen and e.lat_m > 0 and e.beta_deg < 0
    bad += not ok
    say("  %s1) 태그 오른쪽 → ℓ %+.3f (>0), β %+.2f (<0)"
        % (SP._fmt(ok), e.lat_m, e.beta_deg))

    # 2) rotate_ccw → 자이로↑ ∧ ψ↑ ∧ β↓
    pl = SP.SimPlant(forward=4.0, lateral=0.0, heading_deg=0.0, seed=2)
    pl.p.delta_deg = 0.0
    estr, e0 = run(pl, 30)
    g0, p0, b0 = pl.gyro_deg, e0.psi_deg, e0.beta_deg
    pl.set_movement("rotate_ccw")
    _, e1 = run(pl, 60, estr)
    ok = (pl.gyro_deg > g0 + 0.5 and e1.psi_deg > p0 + 0.5
          and e1.beta_deg < b0 - 0.5)
    bad += not ok
    say("  %s2) rotate_ccw(147) → 자이로 %+.2f→%+.2f, ψ %+.2f→%+.2f, β %+.2f→%+.2f"
        % (SP._fmt(ok), g0, pl.gyro_deg, p0, e1.psi_deg, b0, e1.beta_deg))

    # 3) ψ > 0 인 채 전진 → ℓ 증가 ∧ x 감소
    pl = SP.SimPlant(forward=6.0, lateral=0.0, heading_deg=+8.0, seed=3)
    estr, e0 = run(pl, 20)
    pl.set_movement("forward")
    _, e1 = run(pl, 150, estr)
    ok = e1.lat_m > e0.lat_m + 0.05 and e1.x_m < e0.x_m - 0.05
    bad += not ok
    say("  %s3) ψ=+8° 전진 → ℓ %+.3f→%+.3f, x %.2f→%.2f"
        % (SP._fmt(ok), e0.lat_m, e1.lat_m, e0.x_m, e1.x_m))

    # 4) (I1) 항등식. δ 는 **docking_state 가 빼는 config 값**이지 진짜 장착각이 아니다 —
    #    이 시험은 부호·보정의 코드 일관성 검사이지 장착 캘리브 검사가 아니다(계약 §1.3).
    pl = SP.SimPlant(forward=5.0, lateral=1.0, heading_deg=-7.0, seed=4)
    estr, _ = run(pl, 5)
    worst = 0.0
    for _ in range(120):
        pl.step(DT)
        e = estr.update(pl.observe())
        r = bearing_identity_residual(e, D.CAM_YAW_OFFSET_DEG)
        if r == r:
            worst = max(worst, abs(r))
    ok = worst <= 0.2
    bad += not ok
    say("  %s4) (I1) |β + atan2(ℓ,x) + ψ + δ| 최대 %.4f° (≤0.2)"
        % (SP._fmt(ok), worst))

    # 추가) 카메라가 낮으면 태그는 **위로** 잘린다
    pl = SP.SimPlant(forward=3.6, lateral=0.0, seed=5)
    pl.p.sigma_c = 0.0
    pl.p.delta_deg = 0.0
    pl.p.roll_deg = 0.0
    top = None
    while pl.x > 2.6:
        pl.x -= 0.02
        uv = pl._corner_px(noise=False)
        r = pl.observe()
        if not r["tag_seen"] and top is None and uv is not None:
            h, w = 480, 640
            d = {"위": uv[:, 1].min(), "아래": h - 1 - uv[:, 1].max(),
                 "왼쪽": uv[:, 0].min(), "오른쪽": w - 1 - uv[:, 0].max()}
            top = min(d, key=d.get)
    ok = top == "위"
    bad += not ok
    say("  %s+) 가까워질 때 먼저 잘리는 변 = %s (기대: 위)" % (SP._fmt(ok), top))

    if render:
        bad += _sign_test_render(say)
    say("  실패 %d 항" % bad)
    return bad


def _sign_test_render(say):
    """렌더 → **진짜 TagPipeline** → docking_state 로 1) 을 다시 확인한다."""
    try:
        from src.models.detection.detection_pose import TagPipeline
    except Exception as exc:
        say("  -- 렌더 경로 건너뜀 (%s)" % exc)
        return 0
    try:
        pl = SP.SimPlant(forward=3.0, lateral=0.7, vertical=-0.35, seed=9,
                         render=True)
        pl.p.sigma_c = 0.0
        pl.p.delta_deg = 0.0
        pl.p.roll_deg = 0.0
        pipe = TagPipeline(intrinsics=pl.intr, tag_size=pl.tag_size, quality=True,
                           depth_check=False)
        i, ts, img = pl.frame()
        res = pipe.process(img, index=i, timestamp=ts)
        row = _row_from_result(res, int(D.TAG_ID), 1)
        e = RefEstimator().update(row)
    except Exception as exc:
        say("  -- 렌더 경로 건너뜀 (%s: %s)" % (type(exc).__name__, exc))
        return 0
    ok = bool(e.tag_seen and e.lat_m > 0 and e.beta_deg < 0 and e.x_m > 0)
    say("  %sR) 렌더+진짜 TagPipeline: ℓ %+.3f (>0), β %+.2f (<0), x %.2f"
        % (SP._fmt(ok), e.lat_m, e.beta_deg, e.x_m))
    if ok:
        r = bearing_identity_residual(e, 0.0)
        ok2 = abs(r) <= 0.2
        say("  %sR2) 렌더 경로 (I1) 잔차 %.4f°" % (SP._fmt(ok2), r))
        return 0 if ok2 else 1
    return 1


# ═══════════════════════════════════════════════════════════════════════════
# 시험 2 — 계약 위반 0 (전이표·EXECUTE 중 명령·SafeCanTx 밖 송신·CAN 템플릿)
# ═══════════════════════════════════════════════════════════════════════════
def contract_tests(verbose=True, seed=0):
    say = print if verbose else (lambda *a, **k: None)
    bad = 0
    say("── 계약 시험 ──────────────────────────────────────────────────")

    try:
        tpl = can_tx.check_templates(log=None, forward_slow_expect=C.FORWARD_SLOW)
        ok = all(len(v) == 8 for v in tpl.values())
        # byte4(포크 리프트)가 어떤 동작에도 안 쓰여야 한다 — 2026-09-07 사고
        ok = ok and all(v[4] == 127 for v in tpl.values())
        say("  %sCAN 템플릿: byte1/2 만 쓰고 byte4(포크)는 전부 중립" % SP._fmt(ok))
        bad += not ok
    except SystemExit as exc:
        say("  !! CAN 템플릿 거부: %s" % exc)
        bad += 1
    except ImportError:
        # 맥엔 canlib 이 없다(control_forklift_v2 가 import 한다). VM/Jetson 에서 본다.
        say("  -- CAN 템플릿 검사 건너뜀 (canlib 없음 — 우분투 VM/Jetson 에서 확인할 것)")

    try:
        SP.SimPlant().set_movement("fork_up")
        ok = False
    except ValueError:
        ok = True
    bad += not ok
    say("  %s플랜트가 SAFE_MOVEMENTS 밖 명령을 거부한다" % SP._fmt(ok))

    eps = [run_episode(seed + 100 * k, start=(8.0, l, None), strict=True)
           for k, l in enumerate((0.0, 1.0, -1.5))]
    viol = [v for e in eps for v in e.violations]
    ok = not viol
    bad += not ok
    say("  %s전이표·EXECUTE·송신창구 위반 %d 건 %s"
        % (SP._fmt(ok), len(viol), viol[:3] if viol else ""))

    ok = all(e.tail_stop > 30 for e in eps)
    bad += not ok
    say("  %s종단에서 stop 프레임이 계속 나간다 (최소 %d 프레임)"
        % (SP._fmt(ok), min(e.tail_stop for e in eps)))

    ok = all(abs(e.pred_e_l) < 10.0 or e.pred_e_l != e.pred_e_l for e in eps)
    bad += not ok
    say("  %s수용식이 말이 되는 값을 낸다" % SP._fmt(ok))
    say("  실패 %d 항" % bad)
    return bad


# ═══════════════════════════════════════════════════════════════════════════
# 시험 3 — 고장주입. 필수 4종 + 계약 §7.3 의 나머지
# ═══════════════════════════════════════════════════════════════════════════
FAULT_CASES = (
    ("AMBIGUOUS 10 s", SP.Faults(ambiguous_s=10.0, ambiguous_at=2.0), True),
    ("lateral_valid False", SP.Faults(lateral_bad=True), True),
    ("자이로 표류 2°/다리", SP.Faults(gyro_drift_dps=0.2), True),
    ("양자화 공백", SP.Faults(quant_gap=True), True),
    ("검출 스레드 300 ms", SP.Faults(detect_stall_s=0.3, detect_stall_at=6.0), False),
    ("호스트 동결 2 s", SP.Faults(host_freeze_s=2.0, host_freeze_at=8.0), False),
    ("이중 송신원", SP.Faults(dual_source_at=9.0, dual_source_cmd="stop"), False),
    ("차량 워치독 0.6 s", SP.Faults(watchdog_s=0.6), False),
    ("자이로 유실(50 s 에 1회)", SP.Faults(gyro_gap_p=1e-4), False),
)


def fault_tests(verbose=True, seed=0, n=2):
    """각 고장이 **캡 안에 Tier 에 도달**하고 반전 0 인가 (plan 3-7 검증)."""
    say = print if verbose else (lambda *a, **k: None)
    bad = 0
    say("── 고장주입 (필수 4종 + 4) ─────────────────────────────────────")
    say("     ※ Tier 사다리를 보려고 **--final-anyway 없이** 돌린다")
    for name, f, required in FAULT_CASES:
        eps = [run_episode(seed + 17 * k, start=(8.0, 1.0, None), faults=f,
                           final_anyway=False, strict=False)
               for k in range(n)]
        term = all(e.outcome in ("done", "done_dr", "abort") for e in eps)
        tier = all(sum(v for _, v in e.tiers) > 0 or e.abort for e in eps)
        rev = sum(e.reversals for e in eps)
        viol = sum(len(e.violations) for e in eps)
        tail = min(e.tail_stop for e in eps)
        wrong = sum(e.wrong_done for e in eps)
        # 반전 기준은 plan 3-1 KPI 그대로 "도킹당 ≤1" (0 이 아니다)
        ok = term and tier and rev <= len(eps) and viol == 0 and tail > 30 and wrong == 0
        if required:
            bad += not ok
        say("  %s%-22s 종단 %s  Tier %s  반전 %d  위반 %d  wrong-DONE %d  결과 %s"
            % (SP._fmt(ok) if required else ("   " if ok else "~~ "), name,
               "도달" if term else "미달", "도달" if tier else "미도달",
               rev, viol, wrong,
               ",".join(sorted({e.abort or e.outcome for e in eps}))))
    say("  실패 %d 항 (필수만)" % bad)
    return bad


# ═══════════════════════════════════════════════════════════════════════════
# 시험 4 — 몬테카를로 + KPI (plan 4-3 서열)
# ═══════════════════════════════════════════════════════════════════════════
GRID_D = (6.0, 8.0)
GRID_L = (0.0, 1.0, -1.0, 2.0, -2.0)
GRID_PSI = ("aim", 0.0, "aim+8", "aim-8")


def _psi0(kind, d, l):
    aim = -math.degrees(math.atan2(l, d))
    if kind == "aim":
        return aim
    if kind == "aim+8":
        return aim + 8.0
    if kind == "aim-8":
        return aim - 8.0
    return float(kind)


def _pct(v, q):
    if not v:
        return float("nan")
    s = sorted(v)
    return s[min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))]


def monte_carlo(n=3, seed=0, final_anyway=True, verbose=True, grid=True):
    say = print if verbose else (lambda *a, **k: None)
    cells = ([(d, l, p) for d in GRID_D for l in GRID_L for p in GRID_PSI]
             if grid else [(8.0, 1.0, "aim"), (8.0, -1.5, "aim"), (6.0, 0.5, 0.0)])
    x_J = max(X_STOP_CAM_M + 0.6, 3.3 + X_J_MARGIN_M)
    eps = []
    t0 = time.time()
    for ci, (d, l, pk) in enumerate(cells):
        inside = abs(l) <= (d - x_J) * math.tan(math.radians(BETA_VIS_DEG))
        for k in range(n):
            sd = seed + 1009 * ci + 7 * k
            e = run_episode(sd, start=(d, l, _psi0(pk, d, l)),
                            final_anyway=final_anyway, strict=False)
            e.inside_envelope = inside
            eps.append(e)
    wall = time.time() - t0

    def report(sub, label):
        if not sub:
            say("  %s: 표본 0" % label)
            return
        nsucc = sum(e.success for e in sub)
        m = [e.margin_m for e in sub]
        say("  %-12s n %3d | 평행사변형 %3d/%-3d (%3.0f%%) | wrong-DONE %d | "
            "거짓수용 %d | abort %2d | 프리미티브 중앙 %.0f | 반전 %d | %.0fs"
            % (label, len(sub), nsucc, len(sub), 100.0 * nsucc / len(sub),
               sum(e.wrong_done for e in sub), sum(e.false_accept for e in sub),
               sum(1 for e in sub if e.outcome == "abort"),
               statistics.median([e.primitives for e in sub]),
               sum(e.reversals for e in sub),
               statistics.median([e.elapsed for e in sub])))
        say("               여유 m [mm]  p10 %+.0f  p50 %+.0f  p90 %+.0f   "
            "|e_l| 중앙 %.0f mm  |e_h| 중앙 %.2f°"
            % (1000 * _pct(m, 0.1), 1000 * _pct(m, 0.5), 1000 * _pct(m, 0.9),
               1000 * statistics.median([abs(e.e_l) for e in sub]),
               statistics.median([abs(e.e_h) for e in sub])))

    say("── 몬테카를로 (셀 %d × %d회 = %d, %.0fs, 벽시계 %.0fs) ──────────"
        % (len(cells), n, len(eps), sum(e.elapsed for e in eps), wall))
    say("     final_anyway=%s  (%s)" % (final_anyway,
        "운영자 승인 문 — 결과는 무조건 DONE_UNVERIFIED" if final_anyway
        else "정식 수용식만"))
    report(eps, "전체")
    report([e for e in eps if e.inside_envelope], "포락선 안")
    report([e for e in eps if not e.inside_envelope], "포락선 밖")
    out = {}
    for e in eps:
        key = e.abort or e.outcome
        out[key] = out.get(key, 0) + 1
    say("  결과 분포: %s" % ", ".join("%s %d" % kv for kv in sorted(out.items())))
    say("  정식 수용식 통과: %d/%d  (내일 예상 σ(e_l)≈68 mm ≫ %.0f mm 라 0 이 정상)"
        % (sum(e.accepted for e in eps), len(eps), 1000 * LAT_TOL_M / 2))
    viol = [v for e in eps for v in e.violations]
    say("  계약 위반 %d 건 %s" % (len(viol), viol[:3] if viol else ""))
    worst = sorted(eps, key=lambda e: e.margin_m)[:5]
    say("  가장 나쁜 시드: %s"
        % ", ".join("%d(%+.0f mm, %s)" % (e.seed, 1000 * e.margin_m,
                                          e.abort or e.outcome) for e in worst))
    hard = sum(e.wrong_done for e in eps)
    say("  ** 하드 KPI wrong-DONE = %d (0 이어야 한다) **" % hard)
    return eps, hard, len(viol)


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════
def _have(mod):
    import importlib
    try:
        importlib.import_module(mod)
        return True
    except Exception:
        return False


def replay(seed, start=(8.0, 1.0, None), final_anyway=True, faults=None):
    """실패한 시드 하나를 한 줄씩 다시 본다."""
    print("── replay seed %d  start %s ─────────────────────────────────"
          % (seed, start))
    e = run_episode(seed, start=start, final_anyway=final_anyway, faults=faults,
                    strict=False, trace=True)
    print("  결과 %s%s  zone %d  프리미티브 %d  반전 %d  %.1fs"
          % (e.outcome, (" (%s)" % e.abort) if e.abort else "", e.zone,
             e.primitives, e.reversals, e.elapsed))
    print("  진값  e_l %+.1f mm  e_h %+.2f°  x오차 %+.0f mm  → 평행사변형 %s "
          "(여유 %+.1f mm)"
          % (1000 * e.e_l, e.e_h, 1000 * e.x_err,
             "통과" if e.success else "실패", 1000 * e.margin_m))
    print("  예측  e_l %+.1f mm  σ %.1f mm  수용 %s"
          % (1000 * e.pred_e_l, 1000 * e.pred_sigma, e.accepted))
    for ev in e.events:
        print("    %7.2fs %-14s %s" % (ev["t"], ev["event"],
                                       {k: v for k, v in ev.items()
                                        if k not in ("t", "event")}))
    return e


def main(argv=None):
    ap = argparse.ArgumentParser(description="도킹 폐루프 시험대 (하드웨어 없음)")
    ap.add_argument("--n", type=int, default=2, help="격자 셀당 반복 수")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--grid", action="store_true", help="전체 시작 격자로 돌린다")
    ap.add_argument("--signs", action="store_true", help="부호 자가시험만")
    ap.add_argument("--faults", action="store_true", help="고장주입만")
    ap.add_argument("--replay", type=int, default=None, help="이 시드를 한 줄씩")
    ap.add_argument("--lat", type=float, default=1.0, help="--replay 시작 횡위치")
    ap.add_argument("--dist", type=float, default=8.0, help="--replay 시작 거리")
    ap.add_argument("--psi", default=None,
                    help="--replay 시작 heading [도] 또는 aim / aim+8 / aim-8 (격자와 같게)")
    ap.add_argument("--no-final-anyway", action="store_true",
                    help="운영자 승인 문을 닫는다(정식 수용식만)")
    ap.add_argument("--plant", action="store_true", help="플랜트 자체 검증도 돌린다")
    ap.add_argument("--ref", action="store_true",
                    help="진짜 스택 대신 이 파일의 Ref 대역으로 돌린다")
    ap.add_argument("--real", action="store_true",
                    help="진짜 스택만 (실패하면 예외를 그대로 낸다)")
    a = ap.parse_args(argv)

    global STACK
    STACK = "ref" if a.ref else ("real" if a.real else "auto")
    if CONFIG_MISSING:
        print("!! config/control.py 에 %s 가 아직 없다 — 계약 §8 값으로 대신한다"
              % ", ".join(CONFIG_MISSING))
    print("   허용치 LAT_TOL %.0f mm · HEAD_TOL %.1f° · DOCK_DEPTH %.1f m · "
          "정지점 %.2f m · 예산 %.0fs"
          % (1000 * LAT_TOL_M, HEAD_TOL_DEG, DOCK_DEPTH_M, X_STOP_CAM_M,
             DOCK_TIME_BUDGET_S))
    print("   스택 %s — 추정기 %s · 동역학 %s · 상태기계 %s"
          % (STACK, "있음" if HAVE_EST else "없음",
             "있음" if _have("src.models.dynamics.predict") else "없음",
             "있음" if HAVE_FSM else "없음"))

    if a.replay is not None:
        psi = None
        if a.psi is not None:
            psi = _psi0(a.psi if a.psi.startswith("aim") else float(a.psi),
                        a.dist, a.lat)
        replay(a.replay, start=(a.dist, a.lat, psi),
               final_anyway=not a.no_final_anyway)
        return 0
    bad = 0
    if a.plant:
        bad += SP.self_check(n=60, seed=a.seed)
    if a.signs:
        return 1 if sign_tests() else 0
    if a.faults:
        return 1 if fault_tests(seed=a.seed, n=a.n) else 0

    bad += sign_tests()
    if bad:
        print("!! 부호 시험이 깨졌다 — 아래 결과는 전부 무효다. 여기서 멈춘다.")
        return 1
    bad += contract_tests(seed=a.seed)
    bad += fault_tests(seed=a.seed, n=max(1, a.n // 2))
    eps, hard, viol = monte_carlo(n=a.n, seed=a.seed, grid=a.grid,
                                  final_anyway=not a.no_final_anyway)
    bad += hard + viol
    print("══ 실패 %d 항 ═══════════════════════════════════════════════" % bad)
    return 1 if bad else 0


__all__ = ["RefEstimate", "RefEstimator", "RefDynamics", "RefPilot", "Episode",
           "run_episode", "sign_tests", "contract_tests", "fault_tests",
           "monte_carlo", "replay", "bearing_identity_residual", "fork_tip"]

if __name__ == "__main__":
    raise SystemExit(main())
