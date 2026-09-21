"""연속 추정기 — 프레임 한 줄을 먹고 **매 프레임 하나씩** 상태를 내놓는다.

    FrameObs  (frame.jsonl 한 줄)  →  Estimator.update()  →  Estimate

stop-and-go 를 버린 자리가 여기다. v2 는 멈춰서 30 프레임 중앙값을 내고 눈 감고
한 동작을 했다 — 프레임 사이에 기억이 없으니 유령 오차를 쫓아 좌우로 왕복했다.
여기서는 멈추지 않고 계속 추정하고, 30 프레임 중앙값은 **정지 후 확인(VERIFY)**
에만 쓴다.

계약은 `Agent/contracts_B.md` §1·§2. 부호는 §1 하나로 통일돼 있고
**항등식 (I1) 은 매 프레임 로그에 남긴다** — 부호 사고를 현장에서 즉시 잡는
유일한 장치다.

구성
────────────────────────────────────────────────────────────────────────
    bearing.py   태그 중심 픽셀 → β 와 σ_β (거리 무관한 1차 관측량)
    heading.py   heading KF [ψ, b_g] + 2중해 branch + 정면 불신 규칙
    track.py     위치 KF(유니사이클) + 명령버퍼 prior + 회전팔 A 실시간 도출

의존 방향 (역류 금지, contracts_B §6)
    utils.{timing,calib,run_log,imu_yaw} → **estimate** → dynamics → control → tools/dock.py
    estimate 는 dynamics·control 을 import 하지 않는다. detection 도 마찬가지다 —
    입력은 오직 `FrameObs` 라서 실주행·시뮬·로그 재생이 한 코드로 돈다.
"""
import math
from dataclasses import dataclass

from config import detection as D        # CAM_YAW_OFFSET_DEG 기본값 하나만 쓴다

from .bearing import wrap180

__version__ = "1.0"

#: 수용식·게이트에 쓰는 배수. 코드 내부 상수다 (contracts_B §3.4 "k = 2. config 아님")
K_SIGMA = 2.0
#: (I1) 잔차가 이 값을 넘으면 부호가 어긋난 것 → 즉시 정지 + Tier 3 (contracts_B §2.4)
I1_TOL_DEG = 0.2


# ===========================================================================
# 입력 — frame.jsonl 한 줄
# ===========================================================================

@dataclass(frozen=True)
class FrameObs:
    """frame.jsonl 한 줄. 없는 값은 None (bool 은 False).

    필드 이름은 `tools/first_run.py::_log_frame` 이 쓰는 것과 맞춰 뒀다 —
    그래야 내일 로그로 오프라인 재생·시뮬·실주행이 한 코드로 돈다.
    """
    seq: int = 0
    stamps: object = None
    tag_seen: bool = False
    # docking_state (태그 좌표계, contracts_B §1)
    lateral_m: float = None       # ℓ  + = 태그 오른쪽 = 차가 축의 왼쪽
    forward_m: float = None       # x  카메라 → 태그면
    vertical_m: float = None      # 카메라높이 − 태그높이 (카메라가 낮으면 −)
    heading_deg: float = None     # ψ  δ 보정 완료본
    tilt_deg: float = None
    distance_m: float = None
    # 픽셀 관측
    beta_px_deg: float = None     # β  + = 태그 화면 왼쪽. **δ 미보정 생값**
    margin_px: float = None       # 0 이 되는 순간 검출이 끊긴다
    tag_px: float = None
    center_px: tuple = None
    reproj_rms_px: float = None
    decision_margin: float = None
    hamming: int = None
    quality_ok: bool = False
    pnp2: dict = None             # pnp2_solutions() 그대로
    # 자이로
    gyro_deg: float = None        # 누적각 + = 반시계
    gyro_dps: float = None
    gyro_gaps: int = 0
    gyro_alive: bool = False
    gyro_age_s: float = None
    # 명령 쪽 (전파 Q 스케줄·v prior 용, plan 2-2)
    movement: str = None
    t_cmd_set: float = None
    t_cmd_tx: float = None

    #: _log_frame 의 이름 → FrameObs 의 이름. 로그를 그대로 먹기 위한 표 하나.
    ROW_MAP = {"i": "seq", "seen": "tag_seen", "lateral": "lateral_m",
               "forward": "forward_m", "vertical": "vertical_m",
               "distance": "distance_m"}

    @classmethod
    def from_row(cls, row, stamps=None):
        """frame.jsonl 한 줄(dict) → FrameObs. 모르는 키는 조용히 버린다."""
        row = dict(row or {})
        if stamps is None:
            stamps = _stamps_from_row(row)
        kw = {}
        names = {f for f in cls.__dataclass_fields__ if f != "ROW_MAP"}
        for k, v in row.items():
            name = cls.ROW_MAP.get(k, k)
            if name in names and name != "stamps":
                kw[name] = v
        if "gyro_alive" not in kw:
            # 로그에는 alive 열이 없다. 나이가 있으면 그걸로, 없으면 샘플 유무로 본다.
            age = kw.get("gyro_age_s")
            kw["gyro_alive"] = (float(age) <= 0.2) if age is not None else (
                kw.get("gyro_deg") is not None and kw.get("gyro_dps") is not None)
        if kw.get("center_px") is not None:
            kw["center_px"] = tuple(kw["center_px"])
        kw["tag_seen"] = bool(kw.get("tag_seen", False))
        kw["quality_ok"] = bool(kw.get("quality_ok", False))
        kw["gyro_gaps"] = int(kw.get("gyro_gaps") or 0)
        kw["seq"] = int(kw.get("seq") or 0)
        return cls(stamps=stamps, **kw)


def _stamps_from_row(row):
    """로그 줄에 섞여 있는 시각들로 FrameStamps 를 되살린다(오프라인 재생용)."""
    if row.get("t_capture") is None:
        return None
    try:
        from ...utils.timing import FrameStamps
    except Exception:
        return None
    keys = ("t_capture", "t_arrival", "t_detect_done", "t_decide", "t_cmd_set",
            "t_cmd_tx", "t_raw", "domain", "delta_fs_ms", "row_px", "row_corr_ms",
            "stale", "skew", "backward", "dt_s", "frame_number", "dropped_before",
            "exposure_us")
    return FrameStamps(**{k: row.get(k) for k in keys})


# ===========================================================================
# 출력 — contracts_B §2.2 그대로
# ===========================================================================

@dataclass(frozen=True)
class Estimate:
    """추정기 출력. **프레임마다 무조건 하나** 나온다(태그가 안 보여도 예측만으로 낸다).

    부호는 contracts_B §1. 모르는 실수는 float('nan'), 모르는 bool 은 안전한 쪽(False).
    게이트는 반드시 긍정형(`abs(e) + k*sigma <= T`)으로 쓴다 — NaN 은 모든 비교가
    False 라 자동으로 "통과 안 함" 이 된다. `if e > T: 거부` 꼴은 NaN 이 조용히 통과한다.
    """
    # ── 시각 ────────────────────────────────────────────────────────────
    t: float = float("nan")          # 이 추정이 말하는 시각 = 마지막 반영 프레임의 t_capture
    t_pub: float = float("nan")      # 만든 시각. t_pub − t 가 추정의 나이
    seq: int = -1
    # ── 상태 (카메라 기준, 태그축 좌표) ───────────────────────────────────
    x_m: float = float("nan")
    lat_m: float = float("nan")
    psi_deg: float = float("nan")
    v_mps: float = float("nan")
    omega_dps: float = float("nan")
    beta_deg: float = float("nan")
    beta_pred_deg: float = float("nan")
    # ── 불확실도 (1σ, floor 적용 후) ─────────────────────────────────────
    sigma_x_m: float = float("nan")
    sigma_lat_m: float = float("nan")
    sigma_psi_deg: float = float("nan")
    sigma_beta_deg: float = float("nan")
    sigma_v_mps: float = float("nan")
    # ── 회전팔 (실시간 도출) ─────────────────────────────────────────────
    A_m: float = float("nan")
    sigma_A_m: float = float("nan")
    # ── 원관측 (게이트·로그) ─────────────────────────────────────────────
    tag_seen: bool = False
    margin_px: float = float("nan")
    tag_px: float = float("nan")
    tilt_deg: float = float("nan")
    reproj_px: float = float("nan")
    err_ratio: float = float("nan")
    # ── 하드 플래그 ──────────────────────────────────────────────────────
    heading_valid: bool = False
    lateral_valid: bool = False
    gyro_alive: bool = False
    ambiguous: bool = False
    stale: bool = False
    # ── 앵커·건강 ────────────────────────────────────────────────────────
    anchor_age_s: float = float("nan")
    gyro_gaps: int = 0
    reinit_count: int = 0
    n_frames: int = 0
    degraded: str = ""
    note: str = ""

    # 필드는 계약 그대로고 아래는 편의 메서드 하나다. C팀 dock_fsm._sign_residual 이
    # getattr(est, "bearing_identity_residual") 로 먼저 찾아보므로 붙여 둔다 —
    # 없으면 그쪽이 같은 식을 직접 계산한다(값은 동일).
    def bearing_identity_residual(self, delta_deg=0.0):
        return bearing_identity_residual(self, delta_deg)


# ===========================================================================
# 고정 헬퍼 — 다른 팀은 **호출만** 한다 (contracts_B §2.4)
# ===========================================================================

def at_offset(est, ahead_m=0.0, left_m=0.0):
    """(x, ℓ) 을 차체 어느 점 기준으로 옮긴다. contracts_B §1.3 (I2)(I3).

        ℓ_p = ℓ + a·sin ψ + b·cos ψ
        x_p = x − a·cos ψ + b·sin ψ        (a = 앞으로, b = 왼쪽으로)
    """
    psi = math.radians(est.psi_deg)
    s, c = math.sin(psi), math.cos(psi)
    return (est.x_m - ahead_m * c + left_m * s,
            est.lat_m + ahead_m * s + left_m * c)


def pivot(est):
    """회전중심 기준 (x, ℓ). a = −A (A > 0 이면 회전중심이 카메라 뒤)."""
    return at_offset(est, ahead_m=-est.A_m, left_m=0.0)


def centerline(est, x_off_m):
    """차체 중심선 기준. b = +x_off — **부호 주의**(§1.3 이 이렇게 못 박았다)."""
    return at_offset(est, ahead_m=0.0, left_m=float(x_off_m))


def fork_tip(est, cam_to_ref_m, x_off_m):
    """기준점(접힌 포크 끝) 기준. (I4) e_l = ℓ + CAM_TO_REF·sinψ + x_off·cosψ."""
    return at_offset(est, ahead_m=float(cam_to_ref_m), left_m=float(x_off_m))


def bearing_identity_residual(est, delta_deg=0.0):
    """(I1) 잔차 [°]. |잔차| > 0.2° 면 부호·보정이 어긋난 것 — 즉시 정지 + Tier 3.

        β = −atan2(ℓ, x)·180/π − ψ − δ          … (I1)  실측 잔차 0.0003°

    **주의**: 이 값은 *발행된 상태*의 내부 정합만 본다(ℓ 을 β 로 갱신하므로
    건강할 때 ~0 이다). 픽셀 경로와 PnP 경로를 맞대는 진짜 교차검사는
    `Estimator.identity_obs_deg` 다 — 둘 다 frame.jsonl 에 남긴다.
    """
    if not est.tag_seen or est.beta_deg != est.beta_deg:
        return float("nan")
    pred = -math.degrees(math.atan2(est.lat_m, est.x_m)) - est.psi_deg - float(delta_deg)
    return wrap180(est.beta_deg - pred)


def sigma_e_l(x_m, lever_m, sigma_beta_deg, sigma_psi_deg,
              sigma_extra_deg=0.0, sigma_roll_m=0.0, sigma_lat_m=None):
    """(F2) σ(e_l) — **ψ 오차를 한 번만, 부호까지 살려** 센다. contracts_B §3.4 + E7.

        σ(e_l)² = (x̂·σ_β)² + ((lever − x̂)·σ_ψ)² + (lever·σ_여분)² + σ_roll²

    왜 σ_ℓ 을 그대로 안 더하나: ℓ̂ 은 β 로 풀린다(ℓ̂ ≈ −x̂·tan(β+ψ+δ)) → ∂ℓ̂/∂ψ ≈ −x̂ 다.
    e_l = ℓ̂ + lever·sinψ 의 ψ 실효 감도는 (lever − x̂) 이지 lever 가 아니다. σ_ℓ 안에
    이미 깔린 d·σ_ψ 바닥을 또 더하면 **같은 오차를 직교합으로 두 번** 세어(실측 감도
    8.7 mm/° 를 63 mm/° 로) NEAR 진입식 kσ ≤ T/2 가 영영 안 선다.
    σ_β 를 못 쓰는 프레임(태그 미검출)에서만 σ_ℓ 로 떨어진다.
    """
    def f(v, d=float("nan")):
        try:
            v = float(v)
        except (TypeError, ValueError):
            return d
        return v if v == v else d

    x, lever = f(x_m), float(lever_m)
    sb, sp = f(sigma_beta_deg), f(sigma_psi_deg)
    if sp != sp:
        return float("nan")                 # ψ 를 모르면 σ 는 NaN 이지 작은 수가 아니다
    if sb != sb or x != x:
        # σ_β 를 못 쓰는 프레임(태그 미검출) — 분해가 안 되니 σ_ℓ 을 그대로 쓰고
        # 레버 항을 **더한다**(보수 쪽). 블라인드에서 자신감이 올라가면 안 된다.
        base = f(sigma_lat_m, float("nan"))
        if base != base:
            return float("nan")
        term_beta = base
        term_psi = (lever - x) * math.radians(sp) if x == x else 0.0
    else:
        term_beta = x * math.radians(sb)
        term_psi = (lever - x) * math.radians(sp)
    ex = f(sigma_extra_deg, 0.0)
    return math.sqrt(term_beta ** 2 + term_psi ** 2
                     + (lever * math.radians(ex)) ** 2 + f(sigma_roll_m, 0.0) ** 2)


def fork_tip_error(est, cam_to_ref_m, x_off_m, blind_s_m=0.0,
                   sigma_extra_deg=0.0, sigma_roll_m=0.0):
    """(F1) 블라인드 s [m] 뒤 기준점 횡오차와 그 σ. contracts_B §3.4.

        e_l = ℓ̂ + (CAM_TO_REF + s)·sin ψ̂ + x_off·cos ψ̂     ← 레버는 x_ref + s 다
        σ   = sigma_e_l(...)  (같은 식을 화면·로그·상태기계가 공유한다)

    **ψ̂ 를 모르면 e_l 도 NaN 이다** — 0 으로 치면 heading 을 전혀 모르는 상태에서
    "오차 0" 이라고 말하게 된다.
    """
    lever = float(cam_to_ref_m) + float(blind_s_m)
    if est.psi_deg != est.psi_deg or est.lat_m != est.lat_m:
        return float("nan"), sigma_e_l(est.x_m, lever, est.sigma_beta_deg,
                                       est.sigma_psi_deg, sigma_extra_deg, sigma_roll_m,
                                       est.sigma_lat_m)
    psi = math.radians(est.psi_deg)
    e_l = est.lat_m + lever * math.sin(psi) + float(x_off_m) * math.cos(psi)
    sig = sigma_e_l(est.x_m, lever, est.sigma_beta_deg, est.sigma_psi_deg,
                    sigma_extra_deg, sigma_roll_m, est.sigma_lat_m)
    return e_l, sig


def est_row(est, delta_deg=0.0):
    """frame.jsonl 에 넣을 한 줄. (I1) 잔차가 여기 들어간다."""
    r = {k: getattr(est, k) for k in est.__dataclass_fields__}
    for k, v in list(r.items()):
        if isinstance(v, float) and v != v:
            r[k] = None
    i1 = bearing_identity_residual(est, delta_deg)
    r["i1_deg"] = None if i1 != i1 else round(i1, 5)
    return r


# ===========================================================================
# 캘리브 한 벌 — 없으면 가정값 + "가정값 사용 중" 문자열
# ===========================================================================

#: (경로, 기본값) — contracts_B §5.1·§5.2 의 "없을 때 가정값" 열 그대로.
_DEFAULTS = {
    "theta_min_deg": ("perception", "static.theta_min_deg.value", 15.0),
    "sigma_c_px": ("perception", "static.sigma_c_px.value", 0.30),
    "roll_deg": ("perception", "static.roll_deg.value", 0.0),
    "A_m": ("perception", "dynamic.A_m.value", 0.0),
    "sigma_A_m": ("perception", "dynamic.A_m.sigma", 0.5),
    "gyro_scale": ("perception", "dynamic.gyro_scale.value", 1.0),
    "x_off_m": ("perception", "dynamic.x_off_m.value", 0.0),
    "cam_yaw_offset_deg": ("perception", "dynamic.cam_yaw_offset_deg.value", None),
    "sigma_delta_deg": ("perception", "dynamic.cam_yaw_offset_deg.sigma", 1.0),
    "h_tag_cam_m": ("perception", "dynamic.h_tag_cam_m.value", 1.10),
    "beta_vis_L_deg": ("perception", "static.beta_vis_deg.L", 28.0),
    "beta_vis_R_deg": ("perception", "static.beta_vis_deg.R", 28.0),
    "v67_mps": ("dynamics", "v_mps.67.value", 0.28),
    "v97_mps": ("dynamics", "v_mps.97.value", 0.12),
    "tau_start_fwd_s": ("dynamics", "tau_start_s.fwd.value", 1.00),
    "tau_start_rot_s": ("dynamics", "tau_start_s.rot.value", 0.85),
    "tau_eff_s": ("dynamics", "tau_eff_s.67_fwd.value", 0.50),
    "cut_margin_px": ("dynamics", "tag_cut.margin_px", 60.0),
    "cut_forward_m": ("dynamics", "tag_cut.forward_m", 3.3),
}


class EstimatorCalib:
    """`utils.calib.load()` 한 벌을 읽고, **무엇이 가정값인지** 들고 있는다.

    캘리브 파일은 내일 아침까지 없다. 없으면 가정값으로 돌되 **조용히 넘어가지
    않는다** — 쓴 항목 이름이 그대로 Estimate.degraded 에 실려 화면·로그로 나간다.
    """

    def __init__(self, calibs=None, directory=None):
        if calibs is None:
            try:
                from ...utils import calib as CAL
                calibs = CAL.load_all(directory)
            except Exception:
                calibs = {}
        self.calibs = calibs or {}
        self.assumed = []
        self._v = {}
        for name, (kind, path, dflt) in _DEFAULTS.items():
            if dflt is None and name == "cam_yaw_offset_deg":
                dflt = float(getattr(D, "CAM_YAW_OFFSET_DEG", 0.0))
            c = self.calibs.get(kind)
            got = None
            if c is not None and getattr(c, "ok", False):
                got = c.get(path)
            if got is None:
                self.assumed.append(name)
                got = dflt
            self._v[name] = float(got)

    def __getattr__(self, name):
        try:
            return self.__dict__["_v"][name]
        except KeyError:
            raise AttributeError(name)

    @property
    def degraded(self):
        """화면 상단·매 결정 로그에 그대로 들어가는 한 줄. 비면 전부 실측이다."""
        if not self.assumed:
            return ""
        return "가정값 사용 중: " + ", ".join(self.assumed)

    def summary(self):
        return {k: self._v[k] for k in sorted(self._v)} | {"assumed": list(self.assumed)}


# track 은 이 파일의 정의를 쓴다(순환). 그래서 **늦게** 붙인다 — PEP 562 게으른 참조.
# 덤: `python -m src.models.estimate.track` 가 이중 import 경고 없이 돈다.
_LAZY = {"Estimator": "track", "PositionKF": "track",
         "CommandBuffer": "track", "ArmEstimator": "track"}


def __getattr__(name):
    mod = _LAZY.get(name)
    if mod is None:
        raise AttributeError("module %r has no attribute %r" % (__name__, name))
    import importlib
    return getattr(importlib.import_module("." + mod, __name__), name)


__all__ = ["FrameObs", "Estimate", "Estimator", "EstimatorCalib",
           "PositionKF", "CommandBuffer", "ArmEstimator",
           "at_offset", "pivot", "centerline", "fork_tip",
           "bearing_identity_residual", "fork_tip_error", "sigma_e_l", "est_row",
           "K_SIGMA", "I1_TOL_DEG", "wrap180"]
