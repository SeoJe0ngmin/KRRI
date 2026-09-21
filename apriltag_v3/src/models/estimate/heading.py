"""ψ — heading KF [ψ, b_g] 와 PnP 2중해 branch 판정. (plan 1-3·1-4·1-7)

세 가지를 한다
────────────────────────────────────────────────────────────────────────
1) **자이로 바이어스를 실시간 추정한다.** 상수로 박으면 온도 따라 흘러간다.
   상태에 b_g 를 넣고, 차가 멎을 때마다(|ω| < 0.3 °/s) 자이로 출력 자체가
   b_g 의 직접 관측이 된다.
2) **정면에서는 카메라 heading 을 안 믿는다.** 9/7 정면쌍은 spread 0.016~0.064°
   인데 진값에서 +1.1~1.8° 치우쳐 있었다 — 분산으로는 절대 안 잡히는 **매끈한
   바이어스**다. 중앙값·NIS·err비·pitch-cue 를 전부 통과한다. 그래서 표가 아니라
   **규칙 하나**로 막는다: `tilt < θ_min(d)` 이면 그 프레임으로 heading 을 **갱신하지
   않는다**(전파는 자이로로 계속한다). 이게 유일한 방어선이다.
3) **2중해는 골라 주거나, 못 고르면 ambiguous 라고 말한다.** 고르는 근거는
   (a) 두 해의 재투영비(≤0.2 면 그것만으로 갈린다) (b) pitch-cue Wald SPRT
   (c) 직진 다리 ≥1 m 일관성(거짓 해는 Δψ_cam ≈ −2Δβ 로 흐르고 자이로는 0).
   "N 프레임 연속" 규칙은 쓰지 않는다 — 8 m 에서 탈출 확률 0.2% 라 영영 못 빠져나온다.

σ 를 어디에 넣나 (이 갈래를 틀리면 false confidence 가 난다)
────────────────────────────────────────────────────────────────────────
    R(카메라 갱신 잡음)      = σ_random²          ← 진짜 무작위. 평균내면 줄어드는 게 맞다
    출력 σ_psi               = √(P00 + b²)        ← b 는 **필터 밖**에서 바닥으로 깐다
    innovation 게이트        = k·√(P00 + R + b²)  ← b 없이 재포착하면 1~2° 를 매번 기각한다
b 를 R 에 넣으면 KF 가 √N 으로 줄여 버린다. 바이어스는 관측 시간으로 안 줄어든다
(plan 1-4 "√N 축소 금지"). contracts_B §2.2 의 σ_psi 3항식이 이 구조를 말한 것이다.

σ_random 모델 (해석식 + 손검산)
────────────────────────────────────────────────────────────────────────
    한 변 S 인 정사각 태그를 거리 d, yaw ψ 로 보면 좌우 세로변 길이가 어긋난다
        ΔL ≈ f·S²·sin ψ / d²,      d(ΔL)/dψ ≈ f·S²·cos ψ / d²
        σ_ΔL ≈ 2·σ_c (두 변 × 두 코너)
    → σ_ψ ≈ 2·σ_c·d² / (f·S²·cos ψ) = 2·σ_c·f / (tag_px²·cos ψ)   [rad]
      (tag_px = f·S/d 를 넣으면 d·S 가 사라진다 — 그래서 tag_px 하나로 쓴다)
    검산: 1.4 m·0.30 m 태그·fx 1359 → tag_px 291, σ_c 0.35 px
          계수 √2 로 0.455° — plan 1-4 의 실측 0.45° 와 맞는다. 그래서 a 기본값 √2.
    계수 a·지수 p·g(tilt) 는 Day 0 정적 격자 피팅값으로 갈아끼운다
    (perception_calib.static). 없으면 위 기본값 — **낙관 쪽으로 틀리지 않는다.**

자기검증: `python -m src.models.estimate.heading`
"""
import math
from dataclasses import dataclass

from .bearing import wrap180

# ── 코드 내부 상수 (config 아님) ─────────────────────────────────────────────
GYRO_STILL_DPS = 0.30        #: 이 아래면 "멎었다" 로 보고 b_g 를 재추정한다 (plan 1-7)
GYRO_HZ = 200.0              #: 자이로 표본율 [Hz]. 각증분 잡음 유도에만 쓴다
SIGMA_OMEGA_DEFAULT_DPS = 0.30   #: 정지 관측 전 쓸 자이로 1표본 잡음 [°/s]
BIAS_RW_DPS_RTS = 0.002      #: b_g 랜덤워크 [°/s/√s]. 9/7 드리프트 0.2~0.4°/12~23 s 규모
SIGMA_PSI_A = math.sqrt(2.0) #: σ_random 계수 a (plan 1-4 실측 1점에 맞춘 기본값)
SIGMA_PSI_P = 2.0            #: tag_px 지수 p (해석식 유도값)
COS_TILT_FLOOR = 0.30        #: g(tilt) = 1/max(cos tilt, 이 값). 큰 tilt 에서 발산 방지
THETA_MIN_DEFAULT_DEG = 15.0 #: θ_min 기본값 [°] (perception_calib.static.theta_min_deg)
BIAS_FLOOR_OBLIQUE_DEG = 1.0 #: b 기본 바닥 [°] (contracts_B §5.2)
BIAS_FLOOR_FRONTAL_DEG = 1.5 #: 정면 원뿔 안에서의 b 바닥 [°]
INNOV_GATE_K = 3.0           #: innovation 게이트 배수
REINIT_AFTER_REJECT = 12     #: 게이트 연속 기각이 이만큼이면 재앵커 (12 프레임 ≈ 0.4 s)
ANCHOR_FRAMES = 30           #: 초기화·재초기화 다수결 프레임 수 (plan 1-3 "단일 프레임 금지")
ANCHOR_SPREAD_MAX_DEG = 5.0  #: 다수결 창의 산포가 이보다 크면 앵커로 안 쓴다
SIGMA_PSI_FALLBACK_DEG = 2.0 #: fx 를 모르면 해석식을 못 쓴다. 9/7 정면 바이어스 1.1~1.8° 위의 보수값
PROV_PENALTY = 1.5           #: 앵커 못 선 **잠정** ψ 의 σ 벌점. 중앙값이라 무작위는 √n 로 줄지만
                             #: 그 창이 진짜 독립인지 모른다 — 반쯤만 믿는다


def theta_min_deg(d_m, calib=None):
    """정면 원뿔 경계 [°]. tilt 가 이보다 작으면 카메라 heading 을 **갱신하지 않는다**.

    plan 은 θ_min(d) 라 썼지만 측정 전에는 거리 의존을 만들 근거가 없다 —
    스칼라 하나(기본 15°)로 두고, 격자가 (d, θ) 표를 채우면 그 표를 쓴다.
    """
    if calib is None:
        return THETA_MIN_DEFAULT_DEG
    tbl = calib.get("static.theta_min_deg.by_d") if hasattr(calib, "get") else None
    if isinstance(tbl, list) and tbl and d_m == d_m:
        best = min(tbl, key=lambda e: abs(float(e.get("d", 0.0)) - d_m))
        v = best.get("theta_min_deg")
        if v is not None:
            return float(v)
    v = calib.get("static.theta_min_deg.value") if hasattr(calib, "get") else None
    return THETA_MIN_DEFAULT_DEG if v is None else float(v)


def bias_floor_deg(d_m, tilt_deg, calib=None, th_min=None):
    """b(d, tilt) — 관측 시간으로 **안 줄어드는** heading 바닥 [°].

    격자 표(static.heading_bias_deg)가 있으면 (d, tilt) 가 가장 가까운 칸의 |bias|,
    없으면 정면 1.5° / 경사 1.0° (contracts_B §5.2 기본값).
    """
    th = THETA_MIN_DEFAULT_DEG if th_min is None else th_min
    frontal = not (tilt_deg == tilt_deg) or tilt_deg < th
    base = BIAS_FLOOR_FRONTAL_DEG if frontal else BIAS_FLOOR_OBLIQUE_DEG
    tbl = calib.get("static.heading_bias_deg") if (calib is not None and hasattr(calib, "get")) else None
    if isinstance(tbl, list) and tbl:
        def dist(e):
            dd = float(e.get("d", 0.0)) - (d_m if d_m == d_m else 0.0)
            dt = float(e.get("tilt", 0.0)) - (tilt_deg if tilt_deg == tilt_deg else 0.0)
            return (dd / 2.0) ** 2 + (dt / 10.0) ** 2      # 거리 2 m ≈ tilt 10° 로 정규화
        e = min(tbl, key=dist)
        b = e.get("bias")
        if b is not None:
            return max(abs(float(b)), 0.1)
    return base


def sigma_psi_random_deg(sigma_c_px, fx, tag_px, tilt_deg, calib=None):
    """위 해석식. 프레임마다 σ̂_c 를 먹여 실시간으로 낸다."""
    if not tag_px or tag_px != tag_px or tag_px <= 0 or not fx:
        return float("nan")
    a, p = SIGMA_PSI_A, SIGMA_PSI_P
    if calib is not None and hasattr(calib, "get"):
        a = float(calib.get("static.sigma_psi.a", a))
        p = float(calib.get("static.sigma_psi.p", p))
    c = math.cos(math.radians(tilt_deg if tilt_deg == tilt_deg else 0.0))
    g = 1.0 / max(abs(c), COS_TILT_FLOOR)
    return math.degrees(a * sigma_c_px * fx / (tag_px ** p) * g)


class HeadingKF:
    """상태 [ψ(°), b_g(°/s)]. 예측은 자이로, 갱신은 카메라(정면 아닐 때만)."""

    def __init__(self, sigma_psi0_deg=5.0, sigma_bias0_dps=0.10):
        self.psi = float("nan")
        self.bias = 0.0
        self.P = [[sigma_psi0_deg ** 2, 0.0], [0.0, sigma_bias0_dps ** 2]]
        self.ready = False
        self.sigma_omega_dps = SIGMA_OMEGA_DEFAULT_DPS
        self._still = []              # 정지 중 gyro_dps 표본 (σ_ω 실측용)

    # -- 예측 ---------------------------------------------------------------
    def predict(self, dt, dpsi_gyro_deg, gyro_scale=1.0):
        """ψ ← ψ + s·Δψ_gyro − b_g·dt.  (Δψ_gyro 는 **이미 적분된 각증분**)

        Q_ψ: 200 Hz 표본 잡음 σ_ω 가 dt 동안 적분되면
             var(Δψ) = N·(σ_ω·dt_s)² = σ_ω²·dt/GYRO_HZ   (N = dt·GYRO_HZ, dt_s = 1/GYRO_HZ)
             손검산: σ_ω 0.3 °/s, dt 1/30 s → σ = 0.0037° / 프레임, 10 s 쌓여 0.064°
        """
        if dt is None or dt <= 0.0 or dt != dt:
            return
        if self.ready and dpsi_gyro_deg == dpsi_gyro_deg:
            self.psi = wrap180(self.psi + gyro_scale * dpsi_gyro_deg - self.bias * dt)
        q_psi = (self.sigma_omega_dps ** 2) * dt / GYRO_HZ
        q_b = (BIAS_RW_DPS_RTS ** 2) * dt
        # F = [[1, −dt], [0, 1]]
        p00, p01, p10, p11 = self.P[0][0], self.P[0][1], self.P[1][0], self.P[1][1]
        n00 = p00 - dt * (p01 + p10) + dt * dt * p11 + q_psi
        n01 = p01 - dt * p11
        n10 = p10 - dt * p11
        n11 = p11 + q_b
        self.P = [[n00, n01], [n10, n11]]

    # -- 갱신 ---------------------------------------------------------------
    def update_psi(self, psi_meas_deg, r_deg2):
        """카메라 heading 한 개. innovation 과 게이트 판정은 호출자가 한다."""
        if not self.ready:
            self.psi = wrap180(psi_meas_deg)
            self.P[0][0] = max(r_deg2, 1e-6)
            self.P[0][1] = self.P[1][0] = 0.0
            self.ready = True
            return 0.0
        nu = wrap180(psi_meas_deg - self.psi)
        s = self.P[0][0] + r_deg2
        k0, k1 = self.P[0][0] / s, self.P[1][0] / s
        self.psi = wrap180(self.psi + k0 * nu)
        self.bias = self.bias + k1 * nu
        p = self.P
        self.P = [[(1 - k0) * p[0][0], (1 - k0) * p[0][1]],
                  [p[1][0] - k1 * p[0][0], p[1][1] - k1 * p[0][1]]]
        return nu

    def update_bias(self, omega_meas_dps, r_dps2):
        """차가 멎었을 때: 자이로 출력 자체가 b_g 의 직접 관측이다. H = [0, 1]."""
        s = self.P[1][1] + r_dps2
        k0, k1 = self.P[0][1] / s, self.P[1][1] / s
        nu = omega_meas_dps - self.bias
        self.psi = wrap180(self.psi + k0 * nu) if self.ready else self.psi
        self.bias = self.bias + k1 * nu
        p = self.P
        self.P = [[p[0][0] - k0 * p[1][0], p[0][1] - k0 * p[1][1]],
                  [(1 - k1) * p[1][0], (1 - k1) * p[1][1]]]

    def note_still(self, omega_dps):
        """정지 표본을 모아 σ_ω 를 실측한다(고정 상수보다 장비·온도에 잘 맞는다)."""
        self._still.append(float(omega_dps))
        if len(self._still) > 200:
            del self._still[0]
        if len(self._still) >= 20:
            m = sum(self._still) / len(self._still)
            var = sum((v - m) ** 2 for v in self._still) / (len(self._still) - 1)
            self.sigma_omega_dps = max(math.sqrt(var), 0.01)

    def inflate(self, add_sigma_psi_deg):
        self.P[0][0] += float(add_sigma_psi_deg) ** 2

    @property
    def sigma_psi_deg(self):
        return math.sqrt(max(self.P[0][0], 0.0))

    @property
    def sigma_bias_dps(self):
        return math.sqrt(max(self.P[1][1], 0.0))


class BranchTest:
    """PnP 두 해 중 어느 쪽인가. (plan 1-3 pitch-cue SPRT + plan 4-4 FM1 직진 일관성)

    pitch-cue: 태그가 카메라보다 h 만큼 **위**에 있으면 틀린 해의 pitch 는
    ≈ 2·atan(h/d) 로 커지고 옳은 해는 장착 pitch(≈0) 근처에 남는다.
    부호 규약 사고를 피하려고 **|pitch| 크기**로만 비교한다.

    왜 SPRT 인가: "15 프레임 연속" 류는 8 m 에서 σ_pitch 6.3° 라 탈출 확률
    0.66^15 ≈ 0.2% — 한 번 잘못 잠기면 영영 못 나온다(plan 1-3 critic).
    Wald 는 증거가 쌓인 만큼만 결론을 내고 반대 증거로 되돌아온다.

    **override 는 기본 꺼 둔다.** pitch 분포의 양봉·σ 는 Day 0 정적 격자에서
    확인한 뒤 승격하기로 했다(plan 1-3 "1단계에선 로그만"). 그때까지 SPRT 의
    역할은 "이 프레임으로 heading 을 갱신해도 되나" 를 막는 것까지다.
    """

    LLR_THRESH = 4.6            #: Wald α=β=0.01 → ln(0.99/0.01) ≈ 4.6
    ERR_RATIO_MAX = 0.2         #: 이보다 작으면 재투영만으로 두 해가 갈린다 (plan 1-3)
    LEG_MIN_M = 1.0             #: 직진 일관성은 이만큼 가야 판별력이 생긴다 (plan 4-4)
    LEG_MIN_DBETA_DEG = 2.0     #: Δβ 가 이보다 작으면 두 가설이 안 갈린다
    PITCH_SEP_K = 2.0           #: |w − m| 이 이 배 σ_p 보다 작으면 그 프레임은 무력
    MIN_SPRT_FRAMES = 5         #: 한 프레임 튐으로 branch 가 정해지면 안 된다 (plan 1-3)

    def __init__(self, allow_override=False):
        self.allow_override = bool(allow_override)
        self.llr = 0.0
        self.n = 0
        self.n_informative = 0
        self.choice = None          # 0 / 1 / None(아직)
        self.by_err = 0             # 재투영비로 갈린 프레임 수
        self.leg_verdict = ""
        self.flip_suspect = False
        self.n_no_pnp2 = 0

    def reset(self, why=""):
        self.llr = 0.0
        self.n = 0
        self.n_informative = 0
        self.choice = None
        self.flip_suspect = False
        self.leg_verdict = why

    def frame(self, pnp2, vertical_m, x_m, sigma_pitch_deg, pitch_mount_deg=0.0):
        """한 프레임의 증거. (choice, ambiguous, note) 를 돌려준다."""
        if not pnp2:
            # pnp2 가 아예 없다(solvePnPGeneric 실패·옛 로그). 이걸 "모호" 로 잠그면
            # heading 이 영영 안 선다 — 2중해 시험은 **덤 게이트**이고 하드 게이트는
            # tilt ≥ θ_min(기하)이다. 그래서 통과시키되 센다(계약 밖 판단, 보고서에 기록).
            self.n_no_pnp2 += 1
            self.choice = 0
            return 0, False, "pnp2 없음 — tilt 게이트만"
        pit = pnp2.get("pitch_deg") or []
        err = pnp2.get("reproj_px") or []
        if len(pit) < 2 or len(err) < 2:
            # 해가 하나면 모호하지 않다(IPPE 가 한 해만 냈다)
            self.n += 1
            return 0, False, "해 1개"
        self.n += 1
        ratio = pnp2.get("err_ratio")
        if ratio is not None and float(ratio) <= self.ERR_RATIO_MAX:
            self.by_err += 1
            self.choice = 0 if err[0] <= err[1] else 1
            return self.choice, False, "재투영비 %.2f" % float(ratio)

        h = abs(vertical_m) if (vertical_m is not None and vertical_m == vertical_m) else float("nan")
        d = x_m if (x_m is not None and x_m == x_m and x_m > 0.05) else float("nan")
        if h != h or d != d:
            return self.choice, self.choice is None, "h·d 없음"
        w = abs(math.degrees(2.0 * math.atan2(h, d)))       # 틀린 해의 |pitch|
        m = abs(pitch_mount_deg)
        sp = max(sigma_pitch_deg if sigma_pitch_deg == sigma_pitch_deg else 3.0, 0.3)
        if abs(w - m) < self.PITCH_SEP_K * sp:
            return self.choice, self.choice is None, "pitch 분리 부족"
        p0, p1 = abs(float(pit[0])), abs(float(pit[1]))
        if abs(p0 - p1) < self.PITCH_SEP_K * sp:
            # 두 해의 pitch 가 붙어 있으면 pitch 는 branch 정보를 안 갖는다.
            return self.choice, self.choice is None, "두 해 pitch 붙음"
        self.llr += (((p1 - m) ** 2 - (p0 - m) ** 2) + ((p0 - w) ** 2 - (p1 - w) ** 2)) / (2.0 * sp * sp)
        self.n_informative += 1
        enough = self.n_informative >= self.MIN_SPRT_FRAMES
        if enough and self.llr >= self.LLR_THRESH:
            self.choice = 0
        elif enough and self.llr <= -self.LLR_THRESH:
            self.choice = 1
        else:
            self.choice = None
        return self.choice, self.choice is None, "LLR %+.1f (n %d)" % (self.llr, self.n_informative)

    def leg(self, dpsi_cam_deg, dpsi_gyro_deg, dbeta_deg, ds_m):
        """직진 다리 하나가 끝났을 때. 거짓 해는 Δψ_cam ≈ −Δψ_gyro − 2Δβ 로 흐른다.

        **회전 뒤의 |Δψ_cam − Δψ_IMU| 는 여기 쓰지 않는다** — 그건 재앵커
        바이어스(FM2) 신호이지 wrong-branch(FM1) 신호가 아니다(plan 정오표).
        """
        if ds_m is None or ds_m < self.LEG_MIN_M or abs(dbeta_deg) < self.LEG_MIN_DBETA_DEG:
            self.leg_verdict = "판별력 부족(Δs %.2f m, Δβ %.1f°)" % (ds_m or 0.0, dbeta_deg)
            return self.leg_verdict
        r_right = abs(dpsi_cam_deg - dpsi_gyro_deg)
        r_wrong = abs(dpsi_cam_deg - (-dpsi_gyro_deg - 2.0 * dbeta_deg))
        if r_wrong < r_right and (r_right - r_wrong) > self.LEG_MIN_DBETA_DEG:
            self.flip_suspect = True
            self.leg_verdict = "wrong_branch (옳음잔차 %.1f° > 거짓잔차 %.1f°)" % (r_right, r_wrong)
        else:
            self.flip_suspect = False
            self.leg_verdict = "ok (잔차 %.1f° / %.1f°)" % (r_right, r_wrong)
        return self.leg_verdict


@dataclass(frozen=True)
class HeadingOut:
    psi_deg: float
    sigma_psi_deg: float        # √(P00 + b²) — 바이어스 바닥 포함
    sigma_random_deg: float     # 이 프레임의 카메라 잡음 1σ (R 에 들어가는 값)
    bias_floor_deg: float
    bias_dps: float
    sigma_bias_dps: float
    omega_dps: float
    heading_valid: bool         # 이 프레임으로 카메라 heading 을 **갱신**했나/해도 되나
    ambiguous: bool
    gyro_alive: bool
    anchor_age_s: float
    theta_min_deg: float
    reinit: bool
    ready: bool
    note: str = ""


class Heading:
    """프레임마다 ψ̂ 를 낸다. 카메라가 절대 기준, 자이로는 그 사이를 잇는다."""

    def __init__(self, calib=None, gyro_scale=1.0, allow_branch_override=False):
        self.calib = calib
        self.gyro_scale = float(gyro_scale)
        self.kf = HeadingKF()
        self.branch = BranchTest(allow_override=allow_branch_override)
        self.t_anchor = None
        self.t_prev = None
        self.gyro_prev = None
        self.n_reject = 0
        self.reinit_count = 0
        self._anchor_buf = []       # 초기화·재초기화 다수결 창
        # 정면 원뿔 전용 **잠정** heading: (카메라 heading − 자이로 누적각) 의 창.
        # 중앙값 + 지금 자이로각 = 자이로로 끌고 간 중앙값이다. 앵커(kf)와는 별개이고
        # heading_valid 를 True 로 만들지 않는다. 아래 update() 끝의 elif 가 쓴다.
        self._prov_buf = []
        self.last = None

    # -- 내부 -------------------------------------------------------------
    def _anchor_try(self, psi_meas, t):
        """단일 프레임으로는 앵커를 잡지 않는다 (plan 1-3 30프레임 다수결)."""
        self._anchor_buf.append(psi_meas)
        if len(self._anchor_buf) > ANCHOR_FRAMES:
            del self._anchor_buf[0]
        if len(self._anchor_buf) < ANCHOR_FRAMES:
            return False, "앵커 %d/%d" % (len(self._anchor_buf), ANCHOR_FRAMES)
        s = sorted(self._anchor_buf)
        med = s[len(s) // 2]
        spread = s[-1] - s[0]
        if spread > ANCHOR_SPREAD_MAX_DEG:
            del self._anchor_buf[0]
            return False, "앵커 산포 %.1f° 과다" % spread
        self.kf.ready = False
        self.kf.update_psi(med, max(self.kf.sigma_psi_deg ** 2, 1.0))
        self._anchor_buf = []
        self.t_anchor = t
        return True, "앵커 %.2f°" % med

    # -- 본체 -------------------------------------------------------------
    def update(self, obs, bobs, sigma_c_px, fx, dt=None, still=False):
        """FrameObs + BearingObs → HeadingOut.

        still 은 "차가 멎어 있다"(명령·속도로 상위가 판단) — b_g 재추정 문이다.
        """
        t = getattr(obs.stamps, "t_capture", None) if obs.stamps is not None else None
        if t is None:
            t = self.t_prev if self.t_prev is not None else 0.0
        if dt is None:
            dt = 0.0 if self.t_prev is None else max(0.0, t - self.t_prev)

        gyro_alive = bool(obs.gyro_alive)
        dpsi = float("nan")
        if obs.gyro_deg is not None and self.gyro_prev is not None and gyro_alive:
            dpsi = wrap180(float(obs.gyro_deg) - self.gyro_prev)
        if obs.gyro_deg is not None:
            self.gyro_prev = float(obs.gyro_deg)
        self.t_prev = t

        self.kf.predict(dt, dpsi, self.gyro_scale)

        omega = float("nan")
        if obs.gyro_dps is not None:
            omega = float(obs.gyro_dps) * self.gyro_scale - self.kf.bias
        if still and gyro_alive and obs.gyro_dps is not None and abs(float(obs.gyro_dps)) < GYRO_STILL_DPS:
            self.kf.note_still(float(obs.gyro_dps))
            self.kf.update_bias(float(obs.gyro_dps), max(self.kf.sigma_omega_dps ** 2, 1e-4))

        tilt = float(obs.tilt_deg) if obs.tilt_deg is not None else float("nan")
        d = float(obs.forward_m) if obs.forward_m is not None else float("nan")
        th = theta_min_deg(d, self.calib)
        b_floor = bias_floor_deg(d, tilt, self.calib, th)
        s_rand = sigma_psi_random_deg(sigma_c_px, fx, bobs.tag_px, tilt, self.calib)
        s_fallback = False
        if s_rand != s_rand and obs.tag_seen:
            s_rand, s_fallback = SIGMA_PSI_FALLBACK_DEG, True   # fx·tag_px 미상 → 보수값

        # 2중해 — 갱신 전에 먼저 본다
        choice, ambiguous, bnote = self.branch.frame(
            obs.pnp2, obs.vertical_m, obs.forward_m,
            s_rand if s_rand == s_rand else 3.0)
        if self.branch.flip_suspect:
            ambiguous = True

        # **파이프라인이 쓴 해가 SPRT 가 고른 해와 다르면 그 heading 은 못 쓴다.**
        # 이게 없으면 정면 근처에서 pose 가 거짓 해로 넘어가는 순간 tilt 가 커 보여
        # 기하 게이트를 통과하고(거짓 해는 tilt ≈ 2·elev), 그 값으로 재앵커가 나서
        # ψ̂ 가 통째로 뒤집힌다 — 시뮬에서 실제로 재현된 FM1 경로다.
        mismatch = False
        if choice is not None and obs.pnp2 and obs.heading_deg is not None:
            ys = obs.pnp2.get("yaw_deg") or []
            if len(ys) > 1 and abs(wrap180(float(ys[0]) - float(ys[1]))) > max(2.0 * (s_rand if s_rand == s_rand else 1.0), 1.0):
                used = _used_index(obs.pnp2, float(obs.heading_deg))
                if used is not None and used != choice and choice < len(ys):
                    mismatch = True
                    bnote += " / 사용해(%d)≠SPRT(%d)" % (used, choice)
        if mismatch:
            ambiguous = True

        stale = bool(obs.stamps is not None and not getattr(obs.stamps, "usable", True))
        frontal = not (tilt == tilt) or tilt < th
        can_update = bool(obs.tag_seen and obs.heading_deg is not None and obs.quality_ok
                          and not stale and not frontal and not ambiguous
                          and s_rand == s_rand)

        psi_meas = None
        if can_update:
            psi_meas = float(obs.heading_deg)
        elif mismatch and self.branch.allow_override and not frontal and not stale \
                and obs.tag_seen and obs.quality_ok and s_rand == s_rand:
            # 격자로 pitch-cue 를 승격한 뒤에만 열리는 문: 옳은 해로 **갈아끼워** 쓴다.
            psi_meas = float(obs.pnp2["yaw_deg"][choice])
            can_update, ambiguous = True, False
            bnote += " / branch 교체"

        # 잠정 창: 앵커가 못 서는 정면에서도 ψ 를 내려면 여기서 모아 둔다.
        # 거짓 해(ambiguous) 프레임은 넣지 않는다 — 중앙값이 그쪽으로 끌려간다.
        if (obs.tag_seen and obs.heading_deg is not None and obs.quality_ok
                and not stale and not ambiguous):
            self._prov_buf.append(wrap180(float(obs.heading_deg)
                                          - (float(obs.gyro_deg)
                                             if obs.gyro_deg is not None else 0.0)))
            if len(self._prov_buf) > ANCHOR_FRAMES:
                del self._prov_buf[0]

        reinit = False
        note = bnote
        if psi_meas is not None:
            if not self.kf.ready:
                ok, why = self._anchor_try(psi_meas, t)
                note = why if not ok else (why + " " + bnote)
                if not ok:
                    can_update = False
            else:
                gate = INNOV_GATE_K * math.sqrt(self.kf.P[0][0] + s_rand ** 2 + b_floor ** 2)
                nu = wrap180(psi_meas - self.kf.psi)
                if abs(nu) > gate:
                    self.n_reject += 1
                    can_update = False
                    note += " / 기각 ν %+.2f° > %.2f°" % (nu, gate)
                    if self.n_reject >= REINIT_AFTER_REJECT:
                        ok, why = self._anchor_try(psi_meas, t)
                        if ok:
                            self.reinit_count += 1
                            self.n_reject = 0
                            reinit = True
                            note += " / 재앵커"
                else:
                    self.n_reject = 0
                    self.kf.update_psi(psi_meas, max(s_rand ** 2, 1e-6))
                    self.t_anchor = t

        age = float("nan") if self.t_anchor is None else max(0.0, t - self.t_anchor)
        if self.kf.ready:
            psi_out = self.kf.psi
            sigma_out = math.sqrt(self.kf.P[0][0] + b_floor ** 2)
        elif psi_meas is not None or self._anchor_buf:
            # 앵커 전에도 **잠정값**은 낸다 — 아무것도 안 주면 위치 KF 가 ψ=0 으로
            # 돌아 ℓ 이 x·ψ 만큼 틀어진다. heading_valid 는 False 라 판단엔 안 쓰인다.
            buf = self._anchor_buf or [psi_meas]
            psi_out = sorted(buf)[len(buf) // 2]
            base = s_rand if s_rand == s_rand else 3.0
            sigma_out = 2.0 * math.sqrt(base ** 2 + b_floor ** 2)   # 앵커 전 벌점 ×2
        elif self._prov_buf:
            # **정면 원뿔에서도 값은 낸다.** tilt < θ_min 이면 앵커(kf.ready)는 영영 안
            # 서는데 — 태그를 화면 가운데 두고 소각으로 접근하면 tilt ≈ 접근각이라
            # 6 m·ℓ 0.8 m 에서 9° 뿐이다 — 여기서 NaN 을 내보내면 ψ 가 **한 번도** 안
            # 서서 위치 KF 가 ψ=0 으로 돌고(ℓ 이 x·ψ 만큼 통째로 틀어진다) 상태기계는
            # 판단 가능 상태에 못 든다(2026-09-21 통합 시뮬에서 실제로 교착했다).
            #
            # 규칙은 그대로 지킨다: **KF 를 이 값으로 갱신하지 않고**(kf.ready 유지),
            # heading_valid=False 다. 원시값을 그대로 내보내면 안 된다 — 6 m 에서
            # 태그가 30 px 라 σ_random 이 5~7° 고 가끔 다른 해로 튄다. 그래서
            # **자이로로 끌고 간 30프레임 중앙값**을 낸다: 창에는 (ψ_cam − 자이로각) 을
            # 넣고 꺼낼 때 지금 자이로각을 더한다 — 회전 중에도 안 뒤처진다.
            # 중앙값이 지우는 건 **무작위 몫뿐**이다. 정면 바이어스는 매끈해서 안 지워지므로
            # σ 에 b(d,tilt) 를 바닥으로 그대로 싣는다(plan 1-3). 그래서 이 ψ 로는
            # 30 mm 급 판정이 통과하지 못한다 — 쓰되 믿지는 않는다.
            offs = sorted(self._prov_buf)
            n_p = len(offs)
            psi_out = wrap180(offs[n_p // 2]
                              + (float(obs.gyro_deg) if obs.gyro_deg is not None else 0.0))
            base = s_rand if s_rand == s_rand else SIGMA_PSI_FALLBACK_DEG
            sigma_out = PROV_PENALTY * math.sqrt((base / math.sqrt(n_p)) ** 2
                                                 + b_floor ** 2)
        else:
            psi_out, sigma_out = float("nan"), float("nan")
        out = HeadingOut(
            psi_deg=psi_out,
            sigma_psi_deg=sigma_out, sigma_random_deg=s_rand, bias_floor_deg=b_floor,
            bias_dps=self.kf.bias, sigma_bias_dps=self.kf.sigma_bias_dps,
            omega_dps=omega, heading_valid=bool(can_update and self.kf.ready),
            ambiguous=bool(ambiguous), gyro_alive=gyro_alive, anchor_age_s=age,
            theta_min_deg=th, reinit=reinit, ready=self.kf.ready,
            note=("정면(tilt %.1f < %.1f) — heading 갱신 안 함; " % (tilt, th) if frontal else "")
                 + ("σ_ψ 보수값(fx 미상); " if s_fallback else "") + note)
        self.last = out
        return out


def _used_index(pnp2, psi_used_deg):
    """docking_state 가 쓴 heading 이 pnp2 의 몇 번째 해인가."""
    ys = pnp2.get("yaw_deg") or []
    if not ys:
        return None
    return min(range(len(ys)), key=lambda i: abs(wrap180(float(ys[i]) - psi_used_deg)))


__all__ = ["Heading", "HeadingKF", "HeadingOut", "BranchTest",
           "theta_min_deg", "bias_floor_deg", "sigma_psi_random_deg",
           "GYRO_STILL_DPS", "THETA_MIN_DEFAULT_DEG"]


# ===========================================================================
# 자기검증 — `python -m src.models.estimate.heading` (리포 루트에서)
# 합성 자이로·카메라로 (1) 바이어스 추정 (2) 융합 수렴 (3) 정면 불신
# (4) σ 바닥이 √N 으로 안 줄어드는지 (5) branch 판정을 본다.
# ===========================================================================
if __name__ == "__main__":
    import random
    import sys

    from . import FrameObs
    from .bearing import BearingObs

    fails = []

    def check(name, cond, detail=""):
        print("  %-52s %s %s" % (name, "OK " if cond else "실패", detail))
        if not cond:
            fails.append(name)

    class _St:
        def __init__(self, t, usable=True):
            self.t_capture = t
            self.usable = usable

    def bobs(tag_px=120.0, seen=True):
        return BearingObs(beta_deg=0.0, sigma_deg=0.01, sigma_c_px=0.30, x_n=0.0,
                          margin_px=100.0, tag_px=tag_px, seen=seen,
                          visible_ok=True, cut_soon=False, sigma_assumed=False)

    def row(t, psi_cam=None, tilt=20.0, gyro=0.0, dps=0.0, pnp2=None, seen=True):
        return FrameObs(seq=0, stamps=_St(t), tag_seen=seen,
                        forward_m=3.5, lateral_m=0.2, vertical_m=-1.10,
                        heading_deg=psi_cam, tilt_deg=tilt, beta_px_deg=0.0,
                        tag_px=120.0, reproj_rms_px=0.30, quality_ok=seen,
                        pnp2=pnp2, gyro_deg=gyro, gyro_dps=dps, gyro_alive=True)

    FX, DT = 604.089, 1.0 / 30.0
    random.seed(1)

    print("[1] σ_ψ 해석식 손검산 (plan 1-4 의 1.4 m 실측점)")
    v = sigma_psi_random_deg(0.35, 1359.2, 1359.2 * 0.30 / 1.4, 0.0)
    check("1.4 m·0.35 px·fx1359 → 0.45° 근처", abs(v - 0.455) < 0.02, "%.3f°" % v)
    v2 = sigma_psi_random_deg(0.35, 1359.2, 1359.2 * 0.30 / 2.8, 0.0)
    check("거리 2배 → tag_px 절반 → σ 4배", abs(v2 / v - 4.0) < 0.05, "%.3f°" % v2)

    print("[2] 자이로 바이어스 실시간 추정 (정지 중)")
    h = Heading()
    b_true, g = 0.08, 0.0
    for i in range(1200):
        t = i * DT
        g += b_true * DT
        h.update(row(t, psi_cam=3.0, gyro=g, dps=b_true + random.gauss(0, 0.05)),
                 bobs(), 0.30, FX, still=True)
    check("b̂ → 0.08 °/s", abs(h.kf.bias - b_true) < 0.02, "b̂ = %+.4f" % h.kf.bias)
    check("σ_ω 를 정지 표본에서 실측", 0.02 < h.kf.sigma_omega_dps < 0.12,
          "σ_ω = %.3f °/s" % h.kf.sigma_omega_dps)

    print("[3] 융합 수렴 — 카메라가 절대 기준")
    h = Heading()
    psi_t, g, cam_bias = 0.0, 0.0, 1.2
    p2 = {"yaw_deg": [0.0, -3.0], "pitch_deg": [0.5, -34.0],
          "reproj_px": [0.30, 0.45], "err_ratio": 0.66, "n_sol": 2}
    for i in range(400):
        t = i * DT
        w = 6.0 if 60 <= i < 120 else 0.0        # 중간에 12° 돌린다
        psi_t += w * DT
        g += (w + 0.05) * DT
        o = h.update(row(t, psi_cam=psi_t + cam_bias + random.gauss(0, 0.3),
                         gyro=g, dps=w + 0.05, pnp2=p2), bobs(), 0.30, FX)
    check("ψ̂ 가 진값+카메라바이어스로 수렴", abs(o.psi_deg - (psi_t + cam_bias)) < 0.4,
          "ψ̂ %.2f / 진값 %.2f (+bias %.1f)" % (o.psi_deg, psi_t, cam_bias))
    check("heading_valid (tilt 20 ≥ θ_min 15)", o.heading_valid)
    check("branch 안 모호", not o.ambiguous, "LLR %+.0f" % h.branch.llr)

    print("[4] 바이어스 바닥은 √N 으로 안 줄어든다 (plan 1-4)")
    check("σ_ψ ≥ b floor 1.0° (400 프레임 뒤에도)", o.sigma_psi_deg >= 1.0,
          "σ_ψ = %.3f°, b = %.1f°" % (o.sigma_psi_deg, o.bias_floor_deg))
    check("필터 내부 P 는 작아졌다(갱신은 되고 있다)", h.kf.sigma_psi_deg < 0.5,
          "√P00 = %.3f°" % h.kf.sigma_psi_deg)

    print("[5] 정면 불신 — 규칙 하나 (tilt < θ_min 이면 갱신 금지)")
    h2 = Heading()
    g = 0.0
    for i in range(60):        # 먼저 경사에서 앵커를 잡는다
        h2.update(row(i * DT, psi_cam=0.0, tilt=20.0, gyro=g, pnp2=p2), bobs(), 0.30, FX)
    psi_anchor = h2.kf.psi
    for i in range(60, 400):   # 정면으로 들어가고, 카메라가 +5° 틀린 값을 준다
        o = h2.update(row(i * DT, psi_cam=5.0, tilt=4.0, gyro=g, pnp2=p2), bobs(), 0.30, FX)
    check("정면에서 heading_valid False", not o.heading_valid)
    check("ψ̂ 가 +5° 유혹에 안 끌림", abs(o.psi_deg - psi_anchor) < 0.2,
          "ψ̂ %.3f / 앵커 %.3f" % (o.psi_deg, psi_anchor))
    check("정면 b floor 는 1.5°", abs(o.bias_floor_deg - 1.5) < 1e-9)
    check("앵커 나이가 쌓인다", o.anchor_age_s > 10.0, "%.1f s" % o.anchor_age_s)

    print("[6] PnP 2중해 branch")
    bt = BranchTest()
    for _ in range(10):
        c, amb, _n = bt.frame({"pitch_deg": [0.4, -34.0], "reproj_px": [0.3, 0.45],
                               "err_ratio": 0.66}, -1.10, 3.5, 1.0)
    check("pitch 가 갈리면 옳은 해(0)를 고른다", c == 0 and not amb, "LLR %+.1f" % bt.llr)
    bt2 = BranchTest()
    for _ in range(50):
        c, amb, note = bt2.frame({"pitch_deg": [0.4, 0.6], "reproj_px": [0.30, 0.31],
                                  "err_ratio": 0.97}, -1.10, 3.5, 1.0)
    check("두 해 pitch 가 붙어 있으면 ambiguous", amb and c is None, note)
    c, amb, note = bt2.frame({"pitch_deg": [0.4, -34.0], "reproj_px": [0.10, 0.90],
                              "err_ratio": 0.11}, -1.10, 3.5, 1.0)
    check("재투영비 ≤ 0.2 면 그것만으로 갈린다", (not amb) and c == 0, note)
    check("한 프레임으로는 CONFIDENT 안 낸다", BranchTest().frame(
        {"pitch_deg": [0.4, -34.0], "reproj_px": [0.3, 0.45], "err_ratio": 0.66},
        -1.10, 3.5, 1.0)[0] is None)

    print("[7] 직진 다리 일관성 (FM1) — 거짓 해는 Δψ_cam ≈ −2Δβ")
    bt3 = BranchTest()
    check("정상 다리", "ok" in bt3.leg(0.3, 0.2, 5.0, 2.0) and not bt3.flip_suspect,
          bt3.leg_verdict)
    bt4 = BranchTest()
    bt4.leg(-10.0, 0.0, 5.0, 2.0)
    check("거짓 해 다리 → wrong_branch", bt4.flip_suspect, bt4.leg_verdict)
    bt5 = BranchTest()
    bt5.leg(-10.0, 0.0, 5.0, 0.3)
    check("짧은 다리는 판정 안 한다", not bt5.flip_suspect, bt5.leg_verdict)

    print("[8] 파이프라인이 **거짓 해**를 썼을 때 — heading 을 안 믿는다 (FM1)")
    # 정면 근처에서 pose 가 거짓 해로 넘어가면 tilt 가 커 보여 기하 게이트를 통과한다
    # (거짓 해 tilt ≈ 2·elev). 그때 그 값으로 재앵커가 나면 ψ̂ 가 통째로 뒤집힌다 —
    # fake_rig 폐루프에서 실제로 재현된 경로라 회귀시험으로 박아 둔다.
    h3 = Heading()
    good = {"yaw_deg": [-18.0, +7.0], "pitch_deg": [0.5, -34.0],
            "reproj_px": [0.30, 0.45], "err_ratio": 0.66, "n_sol": 2}
    g = 0.0
    for i in range(80):
        o = h3.update(row(i * DT, psi_cam=-18.0, tilt=18.0, gyro=g, pnp2=good), bobs(), 0.30, FX)
    psi_ok = o.psi_deg
    check("경사에서 옳은 해로 앵커", o.heading_valid and abs(psi_ok + 18.0) < 0.5,
          "ψ̂ %.2f" % psi_ok)
    for i in range(80, 260):        # 이제 파이프라인이 해 1(거짓)을 쓴다
        o = h3.update(row(i * DT, psi_cam=+7.0, tilt=24.8, gyro=g, pnp2=good), bobs(), 0.30, FX)
    check("거짓 해를 쓰면 heading_valid False", not o.heading_valid)
    check("ambiguous 로 올려 FSM 이 FALLBACK 하게 한다", o.ambiguous, o.note[:60])
    check("ψ̂ 가 +7° 로 안 끌려간다", abs(o.psi_deg - psi_ok) < 0.5, "ψ̂ %.2f" % o.psi_deg)
    check("재앵커도 안 난다", h3.reinit_count == 0)

    print("\n%s  (%d 실패)" % ("전부 통과" if not fails else "실패: " + ", ".join(fails),
                              len(fails)))
    sys.exit(1 if fails else 0)
