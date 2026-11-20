"""위치 추정 — 유니사이클 KF + 명령버퍼 prior + 회전팔 A 실시간 도출. (plan 1-1·1-7)

상태 [x, ℓ, v] — 카메라 점 기준, 태그축 좌표 (contracts_B §1)
────────────────────────────────────────────────────────────────────────
    ẋ = −v·cos ψ + A·sin ψ·ω          (직진은 −cosψ, 회전은 팔 A 로 호를 그린다)
    ℓ̇ = +v·sin ψ + A·cos ψ·ω
    v̇ = 0  (Q 로만 흔든다. 구동기 모델은 안 넣는다 — plan 1-7)

  이 두 줄은 pivot 좌표의 유니사이클과 **같은 식**이다. (I2)(I3) 의 역변환
  ℓ = ℓ_p + A·sin ψ, x = x_p − A·cos ψ 를 ψ 로 미분하면 dℓ/dψ = A·cos ψ,
  dx/dψ = A·sin ψ 가 나온다. 출력이 카메라 점이라 여기서 바로 굴린다.

왜 ℓ 은 β 로만 갱신하나
────────────────────────────────────────────────────────────────────────
PnP 의 lateral 을 쓰면 yaw 오차가 거리로 증폭돼 들어온다(9/7 7~10 m 에서 60~145 mm).
β 는 중심 픽셀 하나라 σ 가 0.01° 급(3.5 m 에서 0.6 mm)이고 2중해·정면 바이어스를
안 탄다. 그래서 **ℓ 은 β 관측으로 세우고, PnP lateral 은 교차검사에만 쓴다**
(plan 1-1 "명시적 변환", plan 1-2).

σ 를 어디에 넣나 — 유령 정밀도 금지
────────────────────────────────────────────────────────────────────────
β 의 R 에는 **σ_β 만** 넣는다. ψ 의 오차는 프레임마다 독립이 아니라 **공통모드
바이어스**라 필터로 평균내면 √N 으로 잘못 줄어든다. 그래서 ψ 오차는 출력에서
바닥으로 깐다:  σ_lat = max(σ̂_KF, d_h·σ_ψ[rad])  (contracts_B §2.2 가 이 floor 를
요구하는 이유가 이것이다). 롤이 미측정이면 h·1° ≈ 19 mm 도 같이 더한다.

v̂ 는 저속에서 잡음에 묻힌다 → 명령버퍼 prior
────────────────────────────────────────────────────────────────────────
0.15~0.2 m/s 면 프레임당 5~7 mm 인데 σ_x 가 ±5~7 mm 다. 거리 차분만으로는
v̂ 가 안 선다. 그래서 **무슨 movement 를 언제 보냈는지**를 약한 관측으로 넣는다
(죽은시간 지나면 정속, run 간 게인 ±30%). 상태입력이 아니라 prior 다 —
차가 실제로 안 움직이면 카메라가 이긴다.

회전팔 A — 상수가 아니라 매 회전마다 잰다
────────────────────────────────────────────────────────────────────────
config 의 `CAM_TO_PIVOT_M = 1.46` 은 철회됐다(9/7 13/13 회전과 불일치).
제자리 회전 중 β 와 자이로만 있으면 A 가 나온다:

    γ = atan2(ℓ, x),  β = −γ·180/π − ψ − δ
    회전 중 ℓ' = A·cos ψ, x' = A·sin ψ  (ψ 로 미분)
    dγ/dψ = (x·ℓ' − ℓ·x')/(x²+ℓ²) = A·cos(γ+ψ)/d_h = A·cos(β+δ)/d_h
    → **k ≡ −Δβ/Δψ_gyro = s·(1 + A·cos(β+δ)/d_h)**            … (A1)

  부호 주의: plan 의 `Δβ = s(1+A cosβ/d)Δψ` 는 β 를 카메라축 방위각
  atan2(t_x,t_z) 로 쓴 것이라 **우리 β(=−그것)와 부호가 반대**다. 위 식이 우리 규약.
  검산: 9/7 18:14 13회 회전 k = 0.77~0.95(평균 0.88), d_h ≈ 3.6 m, β 작음 → u ≈ 0.278,
        s = 1 이면 A = (0.88 − 1)/0.278 = **−0.43 m**. judge 의 |A| ≲ 0.4 와 맞는다.
  한 거리에서만 돌리면 s 와 A 가 안 갈린다(u 가 상수) — 그래서 두 거리가 필요하다.

자기검증: `python -m src.models.estimate.track`
"""
import math
import time
from dataclasses import dataclass

import numpy as np

from config import control as C
from config import detection as D

from . import Estimate, EstimatorCalib
from .bearing import Bearing, wrap180
from .heading import Heading

# ── 코드 내부 상수 (config 아님 — 필터·창 같은 구현 세부) ────────────────────
SIGMA_X_FLOOR_FRAC = 0.005   #: σ_x 하한 = 거리의 0.5%. 인쇄 태그 한 변 오차(스케일 바이어스)
SIGMA_X_BIAS_FRAC = 0.5      #: 한 프레임 σ_x 중 **평균으로 안 주는** 몫. 격자 전까지 보수값
SIGMA_X_FLOOR_M = 0.002      #: σ_x 절대 하한 [m]
SIGMA_V_INIT = 0.30          #: v 초기 1σ [m/s]
Q_V_TRANSIENT = 0.60         #: 명령이 바뀐 직후 v 프로세스잡음 [m/s/√s] (plan 1-7 스케줄)
Q_V_CRUISE = 0.05            #: 같은 명령이 이어질 때
Q_CRUISE_AFTER_S = 2.0       #: 명령이 이만큼 이어지면 정속으로 본다 [s]
Q_POS_BASE_M_RTS = 0.01      #: 위치 기본 프로세스잡음 [m/√s]. 노면·미끄러짐 몫
GAIN_CV = 0.30               #: run 간 정속 게인 산포 (plan 4-6 G2 "±10~30%")
DEADTIME_BLUR_S = 0.30       #: 죽은시간 전후 이 구간은 "움직이는지 모른다" 로 σ 를 키운다
V_STOP_SIGMA = 0.02          #: 확실히 멎었을 때 v prior 의 σ [m/s]
ROT_MIN_DEG = 5.0            #: 회전 구간이 이보다 작으면 A 회귀에 안 쓴다
ARM_U_SPREAD_MIN = 0.05      #: s 와 A 를 가르려면 u = cos(β+δ)/d 가 이만큼은 벌어져야 한다
ARM_PRIOR_RANGE_M = 0.5      #: A 사전 [−0.5, +0.5] (plan 1-1)
BLIND_SIGMA_GROW = 3.0       #: 태그를 못 볼 때 위치 σ 성장 배수 (프로세스잡음에 곱)


def _f(v):
    """None → NaN. 계약 §2.3 "모르는 실수는 nan"."""
    return float("nan") if v is None else float(v)


# ===========================================================================
# 명령 버퍼
# ===========================================================================

class CommandBuffer:
    """무슨 movement 를 언제 보냈나. v prior 와 Q 스케줄의 근거. (plan 2-2)"""

    FORWARD = {"forward": "67", "forward_slow": "97"}

    def __init__(self, cal):
        self.cal = cal
        self.movement = None
        self.t_set = None
        self.prev_movement = None
        self.v_at_change = 0.0

    def note(self, movement, t_set, t_now, v_now=0.0):
        """프레임마다 부른다. movement 가 바뀐 순간을 잡는다."""
        m = movement or "stop"
        if m != (self.movement or "stop"):
            self.prev_movement = self.movement
            self.movement = m
            self.t_set = t_set if t_set is not None else t_now
            self.v_at_change = float(v_now)
        elif self.t_set is None:
            self.movement = m
            self.t_set = t_set if t_set is not None else t_now

    def age(self, t):
        if self.t_set is None or t != t:
            return float("nan")
        return max(0.0, t - self.t_set)

    @property
    def rotating(self):
        return (self.movement or "") in ("rotate_ccw", "rotate_cw")

    def q_v(self, t):
        """v 프로세스잡음 [m/s/√s] — 명령을 **분산 스케줄로만** 쓴다(plan 1-7)."""
        a = self.age(t)
        if a != a or a < Q_CRUISE_AFTER_S:
            return Q_V_TRANSIENT
        return Q_V_CRUISE

    def prior(self, t):
        """(v_nominal, sigma, note). sigma 가 크면 카메라가 이긴다."""
        m = self.movement or "stop"
        a = self.age(t)
        cal = self.cal
        if m in ("stop", "", None) or m in ("rotate_ccw", "rotate_cw"):
            tau = cal.tau_eff_s
            if a != a:
                return 0.0, SIGMA_V_INIT, "명령 이력 없음"
            if a < 3.0 * tau:
                v = self.v_at_change * math.exp(-a / max(tau, 1e-3))
                return v, max(0.5 * abs(self.v_at_change), V_STOP_SIGMA), "코스팅 %.2fs" % a
            return 0.0, V_STOP_SIGMA, "정지"
        if m == "backward":
            vn, t0, why = -cal.v67_mps, cal.tau_start_fwd_s, "후진"
        elif m == "forward_slow":
            vn, t0, why = cal.v97_mps, cal.tau_start_fwd_s, "97"
        elif m == "forward":
            vn, t0, why = cal.v67_mps, cal.tau_start_fwd_s, "67"
        else:
            return 0.0, SIGMA_V_INIT, "모르는 명령 %s" % m
        if a != a:
            return 0.0, SIGMA_V_INIT, why + " (시각 없음)"
        if a < t0 - DEADTIME_BLUR_S:
            return 0.0, V_STOP_SIGMA, why + " 죽은시간 %.2fs" % a
        if a < t0 + DEADTIME_BLUR_S:
            # 출발했는지 아닌지 모르는 구간 — 절반을 주고 σ 를 통째로 키운다
            return 0.5 * vn, max(0.5 * abs(vn), 0.05), why + " 출발 전후"
        return vn, max(GAIN_CV * abs(vn), 0.02), why + " 정속"


# ===========================================================================
# 회전팔 A
# ===========================================================================

@dataclass
class ArmSample:
    u: float          # cos(β+δ)/d_h  — 회전 구간 평균
    k: float          # −Δβ/Δψ_gyro
    sigma_k: float
    dpsi: float
    d_h: float


class ArmEstimator:
    """(A1) 로 A 를 잰다. 회전 하나가 끝날 때마다 갱신한다."""

    def __init__(self, cal):
        self.cal = cal
        self.A_m = float(cal.A_m)
        self.sigma_A_m = float(cal.sigma_A_m)
        self.gyro_scale = float(cal.gyro_scale)
        self.samples = []
        self._seg = None
        self.note = "사전값" if "A_m" in cal.assumed else "캘리브"

    # -- 구간 --------------------------------------------------------------
    def open_segment(self, beta_deg, gyro_deg, d_h, delta_deg):
        if beta_deg != beta_deg or gyro_deg is None:
            return
        self._seg = {"b0": beta_deg, "g0": float(gyro_deg), "u": [], "n": 0,
                     "b1": beta_deg, "g1": float(gyro_deg), "d": d_h}

    def feed(self, beta_deg, gyro_deg, d_h, delta_deg):
        s = self._seg
        if s is None or beta_deg != beta_deg or gyro_deg is None or d_h != d_h or d_h <= 0.1:
            return
        s["b1"], s["g1"], s["d"] = beta_deg, float(gyro_deg), d_h
        s["u"].append(math.cos(math.radians(beta_deg + delta_deg)) / d_h)
        s["n"] += 1

    def close_segment(self, sigma_beta_deg=0.02):
        """구간을 닫고 한 표본을 만든다. 표본이 늘면 s 와 A 를 같이 푼다."""
        s, self._seg = self._seg, None
        if s is None or s["n"] < 5:
            return None
        dbeta = wrap180(s["b1"] - s["b0"])
        dpsi = s["g1"] - s["g0"]
        if abs(dpsi) < ROT_MIN_DEG:
            return None
        k = -dbeta / dpsi
        sk = abs(math.sqrt(2.0) * sigma_beta_deg / dpsi) + 0.01   # β 잡음 + 자이로 몫
        u = sum(s["u"]) / len(s["u"])
        smp = ArmSample(u=u, k=k, sigma_k=sk, dpsi=dpsi, d_h=s["d"])
        self.samples.append(smp)
        self._solve()
        return smp

    # -- 풀기 --------------------------------------------------------------
    def _solve(self):
        us = [x.u for x in self.samples]
        if len(self.samples) >= 2 and (max(us) - min(us)) >= ARM_U_SPREAD_MIN:
            # k = c0 + c1·u  (c0 = s, c1 = s·A) — 두 거리가 있어야 갈린다
            X = np.array([[1.0, x.u] for x in self.samples])
            y = np.array([x.k for x in self.samples])
            w = np.array([1.0 / max(x.sigma_k, 1e-3) ** 2 for x in self.samples])
            W = np.diag(w)
            try:
                cov = np.linalg.inv(X.T @ W @ X)
                c = cov @ (X.T @ W @ y)
            except np.linalg.LinAlgError:
                return
            s_g, c1 = float(c[0]), float(c[1])
            if abs(s_g) < 0.5:
                return
            A = c1 / s_g
            sA = math.sqrt(max(cov[1, 1], 0.0)) / abs(s_g)
            self.gyro_scale = s_g
            self._blend(A, sA, "회귀 n=%d" % len(self.samples))
        elif self.samples:
            # 한 거리뿐 — s 를 캘리브값으로 고정하고 A 만 본다
            x = self.samples[-1]
            if abs(x.u) < 1e-6:
                return
            A = (x.k / max(self.gyro_scale, 1e-6) - 1.0) / x.u
            sA = abs(x.sigma_k / (self.gyro_scale * x.u))
            self._blend(A, sA, "단일거리 (s 고정 %.3f)" % self.gyro_scale)

    def _blend(self, A, sA, why):
        """사전 [−0.5, +0.5] 과 가우시안 결합. 측정이 나쁘면 사전이 이긴다."""
        A = max(-ARM_PRIOR_RANGE_M, min(ARM_PRIOR_RANGE_M, A))
        sA = max(sA, 0.02)
        p0, s0 = float(self.cal.A_m), float(self.cal.sigma_A_m)
        w0, w1 = 1.0 / s0 ** 2, 1.0 / sA ** 2
        self.A_m = (w0 * p0 + w1 * A) / (w0 + w1)
        self.sigma_A_m = math.sqrt(1.0 / (w0 + w1))
        self.note = "%s A_raw %+.3f±%.3f" % (why, A, sA)


# ===========================================================================
# 위치 KF
# ===========================================================================

class PositionKF:
    """상태 [x, ℓ, v]. 예측은 유니사이클 + 회전팔, 갱신은 forward 와 β."""

    def __init__(self):
        self.x = np.array([float("nan"), float("nan"), 0.0])
        self.P = np.diag([1.0, 1.0, SIGMA_V_INIT ** 2])
        self.ready = False

    def init(self, x_m, lat_m, sigma_x, sigma_lat):
        self.x = np.array([float(x_m), float(lat_m), 0.0])
        self.P = np.diag([max(sigma_x, 1e-3) ** 2, max(sigma_lat, 1e-3) ** 2,
                          SIGMA_V_INIT ** 2])
        self.ready = True

    def predict(self, dt, psi_deg, omega_dps, A_m, sigma_A_m, sigma_psi_deg,
                q_v, blind=False):
        if not self.ready or dt is None or dt <= 0.0 or dt != dt:
            return
        psi = math.radians(psi_deg if psi_deg == psi_deg else 0.0)
        w = math.radians(omega_dps if omega_dps == omega_dps else 0.0)
        A = A_m if A_m == A_m else 0.0
        s, c = math.sin(psi), math.cos(psi)
        v = self.x[2]
        self.x[0] += (-v * c + A * s * w) * dt
        self.x[1] += (+v * s + A * c * w) * dt

        F = np.eye(3)
        F[0, 2] = -c * dt
        F[1, 2] = +s * dt

        ds = abs(v) * dt                         # 이 프레임에 간 거리
        dpsi = abs(w) * dt                       # 이 프레임에 돈 각 [rad]
        sp = math.radians(sigma_psi_deg if sigma_psi_deg == sigma_psi_deg else 2.0)
        sA = sigma_A_m if sigma_A_m == sigma_A_m else 0.5
        base = (Q_POS_BASE_M_RTS ** 2) * dt * (BLIND_SIGMA_GROW if blind else 1.0)
        # (a) ψ 오차가 주행거리에 실려 횡으로 샌다  (b) 회전 중 팔 A 의 불확실도
        q_lat = base + (ds * sp) ** 2 + (sA * dpsi * abs(c)) ** 2
        q_x = base + (ds * sp * abs(s)) ** 2 + (sA * dpsi * abs(s)) ** 2
        Q = np.diag([q_x, q_lat, (q_v ** 2) * dt])
        self.P = F @ self.P @ F.T + Q

    def _update(self, H, nu, r):
        H = np.asarray(H, dtype=float).reshape(1, 3)
        S = float((H @ self.P @ H.T).item()) + float(r)
        if S <= 0.0:
            return 0.0
        K = (self.P @ H.T) / S
        self.x = self.x + (K * nu).ravel()
        I_KH = np.eye(3) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ K.T * float(r)   # Joseph — 대칭·양정 유지
        return nu / math.sqrt(S)                                # 정규화 innovation

    def update_forward(self, z_x, sigma_x):
        if not self.ready or z_x is None or z_x != z_x:
            return float("nan")
        return self._update([1.0, 0.0, 0.0], float(z_x) - self.x[0], max(sigma_x, 1e-4) ** 2)

    def update_bearing(self, z_beta_deg, psi_deg, delta_deg, sigma_beta_deg):
        """β 관측. h = −atan2(ℓ, x)·180/π − ψ − δ.

        ∂h/∂x = +ℓ/(x²+ℓ²)·180/π,  ∂h/∂ℓ = −x/(x²+ℓ²)·180/π
        R 에는 **σ_β 만** 넣는다 — ψ 오차는 공통모드라 출력 floor 로 깐다.
        """
        if not self.ready or z_beta_deg != z_beta_deg:
            return float("nan")
        if psi_deg != psi_deg:
            # **ψ 를 0 으로 치지 않는다.** β = −atan2(ℓ,x) − ψ − δ 라 ψ 를 0 으로 놓고
            # 풀면 ℓ 이 x·tanψ 만큼(2.1 m·5° = 184 mm) 통째로 틀어진다. 모르면 예측만 한다.
            return float("nan")
        x, l = float(self.x[0]), float(self.x[1])
        d2 = x * x + l * l
        if d2 < 1e-6:
            return float("nan")
        k = 180.0 / math.pi
        h = -math.degrees(math.atan2(l, x)) - psi_deg - delta_deg
        H = [l / d2 * k, -x / d2 * k, 0.0]
        return self._update(H, wrap180(float(z_beta_deg) - h), max(sigma_beta_deg, 1e-4) ** 2)

    def update_v(self, z_v, sigma_v):
        if not self.ready or z_v != z_v:
            return float("nan")
        return self._update([0.0, 0.0, 1.0], float(z_v) - self.x[2], max(sigma_v, 1e-4) ** 2)

    def predicted_beta(self, psi_deg, delta_deg):
        if not self.ready or psi_deg != psi_deg:
            return float("nan")         # ψ 를 모르면 β 예측도 없다 (0 으로 치지 않는다)
        x, l = float(self.x[0]), float(self.x[1])
        if x * x + l * l < 1e-9:
            return float("nan")
        return wrap180(-math.degrees(math.atan2(l, x)) - psi_deg - delta_deg)

    def inflate(self, add_x, add_lat):
        self.P[0, 0] += float(add_x) ** 2
        self.P[1, 1] += float(add_lat) ** 2

    @property
    def sigma(self):
        d = np.clip(np.diag(self.P), 0.0, None)
        return float(math.sqrt(d[0])), float(math.sqrt(d[1])), float(math.sqrt(d[2]))


# ===========================================================================
# 추정기 본체
# ===========================================================================

class Estimator:
    """FrameObs → Estimate. 프레임마다 **무조건 하나** 낸다.

        est = Estimator(intrinsics=res.intrinsics)     # 캘리브는 config/ 에서 자동
        for obs in frames:
            e = est.update(obs)
    """

    #: 재앵커(heading 재초기화)가 나면 그 각만큼 횡 레버가 통째로 흔들린다 — 원자 이벤트
    REANCHOR_INFLATE = True

    def __init__(self, intrinsics=None, calibs=None, directory=None, calib=None,
                 delta_deg=None, allow_branch_override=False, tag_size_m=None):
        self.cal = calib if calib is not None else EstimatorCalib(calibs, directory)
        self.intr = intrinsics
        self.tag_size_m = float(tag_size_m if tag_size_m is not None else D.TAG_SIZE_M)
        self.delta_deg = float(self.cal.cam_yaw_offset_deg if delta_deg is None else delta_deg)
        self.bearing = Bearing(intrinsics=intrinsics,
                               sigma_c_fallback=self.cal.sigma_c_px,
                               cut_margin_px=self.cal.cut_margin_px,
                               beta_vis_deg=(self.cal.beta_vis_L_deg, self.cal.beta_vis_R_deg))
        self.heading = Heading(calib=self.cal.calibs.get("perception"),
                               gyro_scale=self.cal.gyro_scale,
                               allow_branch_override=allow_branch_override)
        self.arm = ArmEstimator(self.cal)
        self.cmd = CommandBuffer(self.cal)
        self.kf = PositionKF()
        self.n_frames = 0
        self.reinit_count = 0
        self.t_prev = None
        self.identity_obs_deg = float("nan")     # 픽셀 경로 vs PnP 경로 교차검사
        self.last = None
        self._leg = None                          # 직진 다리 누적 (branch 일관성용)
        self._psi_prev = float("nan")
        self._prior_n = 0                         # 같은 명령 안에서 prior 를 쓴 횟수
        self._prior_mv = None

    # -- 보조 -------------------------------------------------------------
    def _sigma_x(self, x_m, sigma_c_px):
        """σ_x ≈ σ_c·x²/(fx·S)  (plan 1-7: 3 m 에서 3~8 mm).

        손검산: σ_c 0.3 px, x 3 m, fx 1359, S 0.30 m → 0.3·9/(1359·0.3) = 6.6 mm.
        바닥은 거리의 0.5%(인쇄 태그 한 변 오차 = 스케일 바이어스, 평균으로 안 준다).
        """
        fx = getattr(self.intr, "fx", None)
        if not fx or x_m != x_m or x_m <= 0.0:
            return 0.05
        s = sigma_c_px * x_m * x_m / (fx * self.tag_size_m)
        return max(s, SIGMA_X_FLOOR_FRAC * x_m, SIGMA_X_FLOOR_M)

    def _identity_obs(self, obs):
        """(I1) 을 **원관측**으로 본다 — 이게 진짜 부호 검출기다.

        β(픽셀 경로)와 ℓ·x·ψ(PnP 경로)가 같은 자세에서 나왔으면 잔차는 0.0003° 다
        (contracts_B §1.4 실측). 0.2° 를 넘으면 코너 순서·왜곡 모델·δ 배선 중 하나가
        어긋난 것이다 — 왜곡계수가 0 이 아닌데 튀면 plan 1-2 의 "두 모델 혼용" 이 1순위.
        """
        if (not obs.tag_seen or obs.beta_px_deg is None or obs.lateral_m is None
                or obs.forward_m is None or obs.heading_deg is None):
            return float("nan")
        pred = (-math.degrees(math.atan2(float(obs.lateral_m), float(obs.forward_m)))
                - float(obs.heading_deg) - self.delta_deg)
        return wrap180(float(obs.beta_px_deg) - pred)

    # -- 본체 -------------------------------------------------------------
    def update(self, obs):
        t = getattr(obs.stamps, "t_capture", None) if obs.stamps is not None else None
        if t is None or t != t:
            t = (self.t_prev + 1.0 / 30.0) if self.t_prev is not None else time.time()
        dt = 0.0 if self.t_prev is None else max(0.0, t - self.t_prev)
        self.t_prev = t
        self.n_frames += 1
        stale = bool(obs.stamps is not None and not getattr(obs.stamps, "usable", True))

        bobs = self.bearing.update(obs)
        self.identity_obs_deg = self._identity_obs(obs)

        v_now = float(self.kf.x[2]) if self.kf.ready else 0.0
        self.cmd.note(obs.movement, obs.t_cmd_set, t, v_now)
        still = (not self.cmd.rotating and (self.cmd.movement or "stop") == "stop"
                 and (self.cmd.age(t) != self.cmd.age(t) or self.cmd.age(t) > self.cal.tau_eff_s))

        hout = self.heading.update(obs, bobs, bobs.sigma_c_px,
                                   getattr(self.intr, "fx", None), dt=dt, still=still)
        # 앵커 전에는 hout 의 잠정 ψ, 그것도 없으면 이 프레임의 카메라 heading 을 쓴다.
        # ψ=0 으로 돌면 ℓ 이 x·ψ 만큼(3.5 m·2° = 12 cm) 통째로 틀어진다.
        psi = hout.psi_deg
        if psi != psi and obs.heading_deg is not None:
            psi = float(obs.heading_deg)
        d_h = float("nan")
        if self.kf.ready:
            d_h = math.hypot(float(self.kf.x[0]), float(self.kf.x[1]))
        elif obs.forward_m is not None:
            d_h = math.hypot(float(obs.forward_m), float(obs.lateral_m or 0.0))

        # 회전팔 A — 회전 구간의 열고·먹이고·닫기
        if self.cmd.rotating and self.arm._seg is None:
            self.arm.open_segment(bobs.beta_deg, obs.gyro_deg, d_h, self.delta_deg)
        elif self.cmd.rotating:
            self.arm.feed(bobs.beta_deg, obs.gyro_deg, d_h, self.delta_deg)
        elif self.arm._seg is not None:
            self.arm.close_segment(bobs.sigma_deg if bobs.sigma_deg == bobs.sigma_deg else 0.02)

        # 직진 다리 누적 (wrong-branch 일관성, plan 4-4 FM1)
        self._accumulate_leg(t, obs, bobs, hout)

        # 예측
        self.kf.predict(dt, psi, hout.omega_dps, self.arm.A_m, self.arm.sigma_A_m,
                        hout.sigma_psi_deg, self.cmd.q_v(t), blind=not obs.tag_seen)
        if hout.reinit:
            # heading 재앵커 = 횡 레버가 통째로 흔들린 것. 위치 P 를 같이 키운다(원자 이벤트).
            # **한 번의 재앵커는 1 로 센다.** 예전에는 여기서 한 번 세고 발행할 때
            # heading.reinit_count 를 또 더해서 재앵커 한 번에 2 가 나갔고, 상태기계가
            # 첫 재앵커에서 곧바로 REINIT_TWICE 로 중단했다(2026-09-21 통합 시뮬 14/200).
            self.reinit_count += 1
            if self.REANCHOR_INFLATE and self.kf.ready:
                lever = d_h if d_h == d_h else 3.0
                self.kf.inflate(0.0, lever * math.radians(max(hout.sigma_psi_deg, 1.0)))

        usable_meas = bool(obs.tag_seen and obs.quality_ok and not stale)
        if usable_meas and obs.forward_m is not None:
            sx = self._sigma_x(float(obs.forward_m), bobs.sigma_c_px)
            if not self.kf.ready:
                lat0 = float(obs.lateral_m) if obs.lateral_m is not None else 0.0
                self.kf.init(float(obs.forward_m), lat0, sx, max(0.10, abs(lat0) * 0.2))
            else:
                self.kf.update_forward(float(obs.forward_m), sx)
        if usable_meas and self.kf.ready and bobs.seen:
            self.kf.update_bearing(bobs.beta_deg, psi, self.delta_deg, bobs.sigma_deg)

        v_nom, v_sig, v_why = self.cmd.prior(t)
        if self.kf.ready:
            # prior 는 **명령 하나당 관측 하나**다. 30 Hz 로 같은 값을 독립 관측처럼
            # 넣으면 v̂ 가 가정 공칭값에 못박혀(차가 1 mm 도 안 움직여도 +v97) FM4
            # 무응답 감시가 죽고, 반대로 진짜 v 가 커도 v̂ 이 안 따라가 정지거리를
            # 과소예측한다. σ 를 √N 으로 부풀려 두 번째 프레임부터 힘을 뺀다.
            self._prior_n = self._prior_n + 1 if self.cmd.movement == self._prior_mv else 1
            self._prior_mv = self.cmd.movement
            self.kf.update_v(v_nom, v_sig * math.sqrt(self._prior_n))

        return self._publish(t, obs, bobs, hout, stale, v_why)

    def _accumulate_leg(self, t, obs, bobs, hout):
        """직진 다리 하나의 (Δs, Δβ, Δψ_cam, Δψ_gyro) 를 모아 다리 끝에 판정한다."""
        m = self.cmd.movement or "stop"
        forward = m in ("forward", "forward_slow")
        if forward and self._leg is None and bobs.seen and obs.heading_deg is not None:
            self._leg = {"b0": bobs.beta_deg, "psi0": float(obs.heading_deg),
                         "g0": obs.gyro_deg, "x0": float(self.kf.x[0]) if self.kf.ready else float("nan")}
        elif not forward and self._leg is not None:
            L = self._leg
            self._leg = None
            if bobs.seen and obs.heading_deg is not None and L["g0"] is not None and obs.gyro_deg is not None:
                ds = abs(L["x0"] - float(self.kf.x[0])) if (self.kf.ready and L["x0"] == L["x0"]) else 0.0
                self.heading.branch.leg(wrap180(float(obs.heading_deg) - L["psi0"]),
                                        wrap180(float(obs.gyro_deg) - L["g0"]),
                                        wrap180(bobs.beta_deg - L["b0"]), ds)

    def _publish(self, t, obs, bobs, hout, stale, v_why):
        sx, sl, sv = self.kf.sigma if self.kf.ready else (float("nan"),) * 3
        x = float(self.kf.x[0]) if self.kf.ready else float("nan")
        lat = float(self.kf.x[1]) if self.kf.ready else float("nan")
        v = float(self.kf.x[2]) if self.kf.ready else float("nan")
        d_h = math.hypot(x, lat) if x == x else float("nan")

        # σ_lat floor — 유령 정밀도 방지 (contracts_B §2.2).
        # **긍정형으로 쓴다**: σ_ψ 를 모르면 바닥을 못 까는 게 아니라 σ_lat 자체가
        # NaN 이다. 예전엔 조건이 거짓이 되며 바닥이 통째로 건너뛰어져, ψ 를 전혀
        # 모르는 프레임이 σ_lat 0.5 mm·lateral_valid=True 로 발행됐다(유령 정밀도).
        if sl == sl:
            if hout.sigma_psi_deg == hout.sigma_psi_deg and d_h == d_h:
                sl = max(sl, d_h * math.radians(hout.sigma_psi_deg))
            else:
                sl = float("nan")
        # σ_x 에도 같은 이유의 바닥을 깐다(계약엔 없다 — 아래 근거로 A팀이 더한 것).
        # forward 오차에는 태그 한 변 실측오차·코너 계통오차처럼 **N 프레임 평균으로
        # 안 줄어드는 몫**이 섞여 있다. D팀 sim_plant 6 m 에서 원시 PnP 가 진값보다
        # +17 cm(2.9%) 나온 것이 그 증거다. 무작위/계통 비는 Day 0 격자 전까지 미측정이라
        # "절반은 계통" 으로 보수적으로 잡는다.
        if sx == sx and x == x and x > 0.0:
            sx = max(sx, SIGMA_X_BIAS_FRAC * self._sigma_x(x, bobs.sigma_c_px),
                     SIGMA_X_FLOOR_FRAC * x)
        if "roll_deg" in self.cal.assumed and sl == sl:
            # 롤 미측정: h·1° ≈ 19 mm 가 거리와 무관하게 실린다 (plan 1-2)
            sl = math.hypot(sl, self.cal.h_tag_cam_m * math.radians(1.0))

        degraded = self.cal.degraded
        extra = []
        if bobs.sigma_assumed:
            extra.append("σ_c 가정")
        if not obs.gyro_alive:
            extra.append("자이로 죽음")
        if extra:
            degraded = (degraded + "; " if degraded else "") + ", ".join(extra)

        note = "; ".join(s for s in (hout.note, bobs.note, "v:" + v_why, self.arm.note) if s)
        i1o = self.identity_obs_deg
        if i1o == i1o and abs(i1o) > 0.2:
            note = "**부호경보 (I1)관측 %+.3f°**; " % i1o + note

        est = Estimate(
            t=t, t_pub=time.time(), seq=int(obs.seq),
            x_m=x, lat_m=lat, psi_deg=hout.psi_deg, v_mps=v, omega_dps=hout.omega_dps,
            beta_deg=bobs.beta_deg if bobs.seen else float("nan"),
            beta_pred_deg=self.kf.predicted_beta(hout.psi_deg, self.delta_deg),
            sigma_x_m=sx, sigma_lat_m=sl, sigma_psi_deg=hout.sigma_psi_deg,
            sigma_beta_deg=bobs.sigma_deg, sigma_v_mps=sv,
            A_m=self.arm.A_m, sigma_A_m=self.arm.sigma_A_m,
            tag_seen=bool(obs.tag_seen), margin_px=bobs.margin_px, tag_px=bobs.tag_px,
            tilt_deg=float(obs.tilt_deg) if obs.tilt_deg is not None else float("nan"),
            reproj_px=float(obs.reproj_rms_px) if obs.reproj_rms_px is not None else float("nan"),
            err_ratio=_f((obs.pnp2 or {}).get("err_ratio")),
            heading_valid=bool(hout.heading_valid),
            lateral_valid=bool(sl == sl and sl <= C.LAT_TOL_M / 2.0),
            gyro_alive=bool(obs.gyro_alive), ambiguous=bool(hout.ambiguous), stale=stale,
            anchor_age_s=hout.anchor_age_s, gyro_gaps=int(obs.gyro_gaps or 0),
            reinit_count=self.reinit_count,          # heading.reinit_count 와 같은 사건이다
            n_frames=self.n_frames, degraded=degraded, note=note)
        self.last = est
        return est

    # -- 로그 -------------------------------------------------------------
    def row(self, est=None):
        """frame.jsonl 에 넣을 한 줄. (I1) 두 개를 **매 프레임** 남긴다."""
        from . import est_row
        e = est if est is not None else self.last
        if e is None:
            return {}
        r = est_row(e, self.delta_deg)
        r["i1_obs_deg"] = (None if self.identity_obs_deg != self.identity_obs_deg
                           else round(self.identity_obs_deg, 5))
        r["A_note"] = self.arm.note
        r["beta_sigma_c_px"] = round(self.bearing.noise.sigma_c_px, 4)
        return r

    def summary(self):
        return {"n_frames": self.n_frames, "reinit": self.reinit_count,
                "A_m": self.arm.A_m, "sigma_A_m": self.arm.sigma_A_m,
                "gyro_scale": self.arm.gyro_scale,
                "arm_samples": [(round(s.u, 4), round(s.k, 4)) for s in self.arm.samples],
                "heading_reject": self.heading.n_reject,
                "heading_reinit": self.heading.reinit_count,
                "branch_llr": self.heading.branch.llr,
                "branch_leg": self.heading.branch.leg_verdict,
                "beta_recompute_mismatch": self.bearing.n_recompute_mismatch,
                "degraded": self.cal.degraded}


__all__ = ["Estimator", "PositionKF", "CommandBuffer", "ArmEstimator", "ArmSample"]


# ===========================================================================
# 자기검증 — `python -m src.models.estimate.track` (리포 루트에서)
#
# fake_rig 는 **쓰지 않는다**(§7.4 E6: 렌더가 화면에서 180° 뒤집혀 있어 β 부호가
# 반대로 나온다). 대신 여기서 §1 규약 그대로인 진값 플랜트를 굴리고, 관측은
# **한 자세에서 나온 것처럼** 만든다 — 즉 (ℓ, x, ψ, β) 가 항등식 (I1) 을 정확히
# 만족하도록 생성한다. 실제 PnP 가 그렇기 때문이다(실측 잔차 0.0003°).
# fake_rig 는 맨 끝에서 **배관 점검 + E6 확인**에만 쓴다.
# ===========================================================================
if __name__ == "__main__":
    import random
    import sys

    from . import FrameObs, at_offset, bearing_identity_residual, centerline
    from . import fork_tip, fork_tip_error, pivot

    fails = []

    def check(name, cond, detail=""):
        print("  %-54s %s %s" % (name, "OK " if cond else "실패", detail))
        if not cond:
            fails.append(name)

    class _Intr:
        """D435i 컬러 1920x1080 공장값 (config/detection.D435I_COLOR_REF)."""
        fx, fy = 1359.2, 1359.0
        cx, cy = 956.9, 571.3
        width, height = 1920, 1080
        distortion = ()

    class _St:
        def __init__(self, t, usable=True):
            self.t_capture, self.usable = t, usable

    class _Plant:
        """진값. §1 부호 그대로. 죽은시간 + 램프 + 코스팅까지 넣어 등속 가정을 흔든다."""
        V = {"forward": 0.28, "forward_slow": 0.12, "backward": -0.28}
        W = {"rotate_ccw": +8.0, "rotate_cw": -8.0}

        def __init__(self, x=3.5, lat=0.0, psi=0.0, A=0.0, h=1.10, seed=0):
            self.x, self.lat, self.psi, self.A, self.h = x, lat, psi, A, h
            self.v = self.w = 0.0
            self.t = 0.0
            self.cmd_name, self.t_cmd = "stop", 0.0
            self.gyro = 0.0
            self.gyro_bias = 0.05
            self.rng = random.Random(seed)

        def cmd(self, name):
            if name != self.cmd_name:
                self.cmd_name, self.t_cmd = name, self.t

        def step(self, dt):
            self.t += dt
            age = self.t - self.t_cmd
            vt = self.V.get(self.cmd_name, 0.0) if age >= 1.00 else 0.0
            wt = self.W.get(self.cmd_name, 0.0) if age >= 0.85 else 0.0
            self.v += (vt - self.v) * (1.0 - math.exp(-dt / 0.30))
            self.w += (wt - self.w) * (1.0 - math.exp(-dt / 0.18))
            p = math.radians(self.psi)
            wr = math.radians(self.w)
            self.x += (-self.v * math.cos(p) + self.A * math.sin(p) * wr) * dt
            self.lat += (+self.v * math.sin(p) + self.A * math.cos(p) * wr) * dt
            self.psi += self.w * dt
            self.gyro += self.w * dt + self.gyro_bias * dt

        @property
        def beta_true(self):
            return wrap180(-math.degrees(math.atan2(self.lat, self.x)) - self.psi)

        def obs(self, i, sigma_c=0.30, delta=0.0, cam_bias=0.0, usable=True,
                seen=True, tag_size=0.30, noise=True):
            """한 자세에서 나온 것처럼 관측을 만든다 — (I1) 이 정확히 성립한다."""
            fx = _Intr.fx
            d = math.hypot(self.x, self.lat)
            tag_px = fx * tag_size / max(d, 0.2)
            sx = sigma_c * self.x ** 2 / (fx * tag_size)      # plan 1-7: 3 m 에서 3~8 mm
            sb = math.degrees(sigma_c / (2.0 * fx))
            # heading 프레임 잡음은 **실측값**으로 흔든다(9/7 3.3~3.9 m 프레임 σ 0.09~0.35°).
            # 추정기 쪽 σ 모델은 보수적이라, 이렇게 해야 "σ 가 오차를 덮나" 가 의미 있는 시험이 된다.
            sp = 0.30
            g = (lambda s: self.rng.gauss(0.0, s)) if noise else (lambda s: 0.0)
            x_o = self.x + g(sx)
            psi_o = self.psi + cam_bias + g(sp)
            b_o = self.beta_true - delta + g(sb)
            lat_o = -x_o * math.tan(math.radians(b_o + psi_o + delta))
            tilt = abs(self.psi)
            p2 = {"yaw_deg": [psi_o, -psi_o - 2.0],
                  "pitch_deg": [0.5, math.degrees(2.0 * math.atan2(self.h, max(self.x, 0.3)))],
                  "reproj_px": [sigma_c, sigma_c * 1.6],
                  "err_ratio": 0.62, "n_sol": 2}
            return FrameObs(
                seq=i, stamps=_St(self.t, usable), tag_seen=seen,
                lateral_m=lat_o if seen else None, forward_m=x_o if seen else None,
                vertical_m=-self.h if seen else None,
                heading_deg=psi_o if seen else None, tilt_deg=tilt if seen else None,
                distance_m=math.hypot(x_o, lat_o) if seen else None,
                beta_px_deg=b_o if seen else None, margin_px=120.0 if seen else None,
                tag_px=tag_px if seen else None, reproj_rms_px=sigma_c if seen else None,
                quality_ok=seen, pnp2=p2 if seen else None,
                gyro_deg=self.gyro, gyro_dps=self.w + self.gyro_bias, gyro_alive=True,
                movement=self.cmd_name, t_cmd_set=self.t_cmd)

    DT = 1.0 / 30.0

    def run(plant, est, n, cmd=None, **kw):
        out = None
        for i in range(n):
            if cmd is not None:
                plant.cmd(cmd)
            plant.step(DT)
            out = est.update(plant.obs(i, **kw))
        return out

    print("[1] contracts_B §7.3 부호 자가시험 4항 (진값 없이 관측만으로)")
    # ψ = +18° 에서 시작한다 — 정면(tilt < θ_min 15°)에서는 설계상 heading 을
    # 앵커하지 않으므로 ψ̂ 가 NaN 이고, 그러면 2) 를 시험할 수 없다.
    p = _Plant(x=3.5, lat=+0.60, psi=+18.0)
    e = Estimator(intrinsics=_Intr())
    o = run(p, e, 60, cmd="stop", noise=False)
    check("1) 태그 오른쪽 → lat_m > 0 ∧ beta_deg < 0",
          o.lat_m > 0 and o.beta_deg < 0, "lat %+.3f  β %+.2f°" % (o.lat_m, o.beta_deg))

    b0, psi0, g0 = o.beta_deg, o.psi_deg, p.gyro
    o2 = run(p, e, 120, cmd="rotate_ccw", noise=False)
    check("2) rotate_ccw → 자이로↑ ∧ ψ↑ ∧ β↓",
          p.gyro > g0 and o2.psi_deg > psi0 and o2.beta_deg < b0,
          "Δgyro %+.1f°  Δψ %+.1f°  Δβ %+.1f°" % (p.gyro - g0, o2.psi_deg - psi0,
                                                  o2.beta_deg - b0))

    p3 = _Plant(x=4.0, lat=0.0, psi=+12.0)
    e3 = Estimator(intrinsics=_Intr())
    a = run(p3, e3, 60, cmd="stop", noise=False)
    b = run(p3, e3, 150, cmd="forward", noise=False)
    check("3) ψ>0 로 전진 → lat↑ ∧ x↓",
          b.lat_m > a.lat_m and b.x_m < a.x_m,
          "Δlat %+.3f m  Δx %+.3f m" % (b.lat_m - a.lat_m, b.x_m - a.x_m))

    worst = 0.0
    p4 = _Plant(x=5.0, lat=-0.4, psi=+8.0)
    e4 = Estimator(intrinsics=_Intr(), delta_deg=+2.0)
    for i in range(200):
        p4.cmd("forward" if i > 40 else "stop")
        p4.step(DT)
        o = e4.update(p4.obs(i, delta=+2.0))
        r = bearing_identity_residual(o, e4.delta_deg)
        if i > 60 and r == r:
            worst = max(worst, abs(r))
    check("4) |(I1) 잔차| ≤ 0.2° (δ=+2° 에서도)", worst <= 0.2, "최대 %.4f°" % worst)
    check("   원관측 (I1) 도 0 (픽셀 경로 vs PnP 경로)",
          abs(e4.identity_obs_deg) < 1e-6, "%.2e°" % e4.identity_obs_deg)

    print("[2] 수렴 — 대각 접근 5 m → 3.5 m, 잡음 있음")
    p = _Plant(x=5.0, lat=+0.9, psi=-20.0)
    e = Estimator(intrinsics=_Intr())
    o = run(p, e, 60, cmd="stop")
    o = run(p, e, 260, cmd="forward")
    ex, el, ep = abs(o.x_m - p.x), abs(o.lat_m - p.lat), abs(o.psi_deg - p.psi)
    check("x̂ 오차 ≤ 2 cm", ex <= 0.02, "%.4f m (σ %.4f)" % (ex, o.sigma_x_m))
    check("ℓ̂ 오차 ≤ 3 cm", el <= 0.03, "%.4f m (σ %.4f)" % (el, o.sigma_lat_m))
    check("ψ̂ 오차 ≤ 0.5°", ep <= 0.5, "%.3f° (σ %.3f)" % (ep, o.sigma_psi_deg))
    check("σ 가 실제 오차를 덮는다 (|e| ≤ 2σ)",
          ex <= 2 * o.sigma_x_m and el <= 2 * o.sigma_lat_m and ep <= 2 * o.sigma_psi_deg)
    check("진행 중 태그를 계속 봤다 → x 가 실제로 줄었다", p.x < 4.0, "x %.2f m" % p.x)

    print("[3] v̂ — 저속 97 에서 명령버퍼 prior 가 잡아 준다")
    p = _Plant(x=4.0, lat=0.0, psi=0.0)
    e = Estimator(intrinsics=_Intr())
    run(p, e, 30, cmd="stop")
    o = run(p, e, 180, cmd="forward_slow")
    check("v̂ → 0.12 m/s (±0.03)", abs(o.v_mps - p.v) < 0.03,
          "v̂ %.4f / 진값 %.4f (σ %.4f)" % (o.v_mps, p.v, o.sigma_v_mps))
    o = run(p, e, 60, cmd="stop")
    check("정지 명령 뒤 v̂ → 0", abs(o.v_mps) < 0.03, "v̂ %.4f / 진값 %.4f" % (o.v_mps, p.v))

    print("[4] 회전팔 A — 매 회전마다 프레임 회귀 (상수 아님)")
    A_TRUE = +0.35
    e = Estimator(intrinsics=_Intr())
    for d0 in (3.5, 6.0):
        p = _Plant(x=d0, lat=0.0, psi=0.0, A=A_TRUE)
        run(p, e, 30, cmd="stop", noise=False)
        run(p, e, 150, cmd="rotate_ccw", noise=False)     # ≈ +30°
        run(p, e, 40, cmd="stop", noise=False)
    check("표본 2개(두 거리)", len(e.arm.samples) == 2,
          str([(round(s.u, 3), round(s.k, 3)) for s in e.arm.samples]))
    check("Â → +0.35 (±0.12)", abs(e.arm.A_m - A_TRUE) < 0.12,
          "Â %+.3f ± %.3f  (%s)" % (e.arm.A_m, e.arm.sigma_A_m, e.arm.note))
    check("자이로 스케일 ŝ ≈ 1", abs(e.arm.gyro_scale - 1.0) < 0.1, "%.4f" % e.arm.gyro_scale)

    e2 = Estimator(intrinsics=_Intr())
    p = _Plant(x=3.6, lat=0.0, psi=0.0, A=-0.43)          # 9/7 재현: k ≈ 0.88
    run(p, e2, 30, cmd="stop", noise=False)
    run(p, e2, 150, cmd="rotate_ccw", noise=False)
    run(p, e2, 40, cmd="stop", noise=False)
    k = e2.arm.samples[0].k if e2.arm.samples else float("nan")
    check("한 거리만: 9/7 의 k = −Δβ/Δψ ≈ 0.88 이 재현된다", 0.83 < k < 0.93, "k %.3f" % k)
    check("   그때 Â ≈ −0.43", abs(e2.arm.A_m - (-0.43)) < 0.12, "Â %+.3f" % e2.arm.A_m)

    print("[5] stale 프레임으로는 갱신하지 않는다 (STALE_S 초과)")
    p = _Plant(x=3.5, lat=0.0, psi=0.0)
    e = Estimator(intrinsics=_Intr())
    o = run(p, e, 60, cmd="stop", noise=False)
    x_before, s_before = o.x_m, e.kf.sigma[0]        # 출력 σ_x 는 바닥에 눌려 있다
    for i in range(60, 120):                     # 묵은 프레임이 엉뚱한 값을 들고 온다
        p.step(DT)
        bad = p.obs(i, usable=False, noise=False)
        bad = FrameObs(**{**{k: getattr(bad, k) for k in bad.__dataclass_fields__},
                          "forward_m": 1.0, "lateral_m": 0.9})
        o = e.update(bad)
    check("x̂ 가 묵은 관측(1.0 m)으로 안 끌려간다", abs(o.x_m - x_before) < 0.05,
          "x̂ %.3f (묵은 관측 1.000)" % o.x_m)
    check("대신 σ 가 커진다 (필터 내부 P — 출력은 바닥에 눌린다)",
          e.kf.sigma[0] > s_before, "√P_xx %.4f → %.4f (출력 σ_x %.4f, 바닥 %.4f)"
          % (s_before, e.kf.sigma[0], o.sigma_x_m, SIGMA_X_FLOOR_FRAC * o.x_m))
    check("stale 플래그가 선다", o.stale)

    print("[6] 헬퍼 — (I2)(I3)(I4) 손검산")
    e_ = Estimate(x_m=3.0, lat_m=0.5, psi_deg=10.0, A_m=0.4, tag_seen=True,
                  beta_deg=0.0, sigma_lat_m=0.05, sigma_psi_deg=1.0)
    x_p, l_p = pivot(e_)
    check("pivot: ℓ−A·sinψ, x+A·cosψ",
          abs(l_p - (0.5 - 0.4 * math.sin(math.radians(10)))) < 1e-12
          and abs(x_p - (3.0 + 0.4 * math.cos(math.radians(10)))) < 1e-12,
          "(%.4f, %.4f)" % (x_p, l_p))
    _x, l_c = centerline(e_, 0.12)
    check("centerline: ℓ + x_off·cosψ",
          abs(l_c - (0.5 + 0.12 * math.cos(math.radians(10)))) < 1e-12)
    _x, l_f = fork_tip(e_, 1.52, 0.12)
    check("fork_tip (I4): ℓ + 1.52·sinψ + x_off·cosψ",
          abs(l_f - (0.5 + 1.52 * math.sin(math.radians(10))
                     + 0.12 * math.cos(math.radians(10)))) < 1e-12, "%.4f m" % l_f)
    el, sg = fork_tip_error(e_, 1.52, 0.12, blind_s_m=0.5)
    lever = 1.52 + 0.5
    # σ 는 (E7) — ψ 감도를 **한 번만**, 부호까지 살려 센다.
    #   σ² = (x̂·σ_β)² + ((lever − x̂)·σ_ψ)².  옛 식(σ_ℓ ⊕ lever·σ_ψ)은 σ_ℓ 안에 이미
    #   깔린 d·σ_ψ 바닥을 또 더해 같은 오차를 두 번 셌다(실측 8.7 mm/° → 63 mm/°).
    # 이 가짜 Estimate 는 σ_β 가 없다(태그 미검출 갈래) → σ_ℓ ⊕ (lever − x̂)·σ_ψ
    want_sg = math.hypot(0.05, (lever - 3.0) * math.radians(1.0))
    check("(F1)(F2) 레버는 x_ref + s (E3) · σ 는 (lever − x̂)·σ_ψ (E7)",
          abs(el - (0.5 + lever * math.sin(math.radians(10))
                    + 0.12 * math.cos(math.radians(10)))) < 1e-12
          and abs(sg - want_sg) < 1e-12,
          "e_l %.4f  σ %.4f (기대 %.4f)" % (el, sg, want_sg))
    check("at_offset(0,0) 는 그대로", at_offset(e_) == (3.0, 0.5))

    print("[7] 태그 실종 — 예측만으로 계속 내고, β 예측이 남는다 (REACQUIRE 용)")
    p = _Plant(x=4.0, lat=+0.5, psi=0.0)
    e = Estimator(intrinsics=_Intr())
    o = run(p, e, 60, cmd="stop", noise=False)
    b_pred0 = o.beta_pred_deg
    for i in range(60, 150):
        p.cmd("forward")
        p.step(DT)
        o = e.update(p.obs(i, seen=False, noise=False))
    check("태그가 없어도 매 프레임 추정이 나온다", o.x_m == o.x_m and o.n_frames == 150)
    check("β 예측이 남아 있다", o.beta_pred_deg == o.beta_pred_deg,
          "β_pred %+.2f° (실종 전 %+.2f°)" % (o.beta_pred_deg, b_pred0))
    check("tag_seen False ∧ β 관측은 NaN", (not o.tag_seen) and o.beta_deg != o.beta_deg)
    check("블라인드에서 σ_lat 가 자란다", o.sigma_lat_m > 0.02, "%.4f m" % o.sigma_lat_m)

    print("[8] 캘리브 부재 — 조용히 넘어가지 않는다")
    e = Estimator(intrinsics=_Intr())
    p = _Plant()
    o = run(p, e, 10, cmd="stop", noise=False)
    check("degraded 문자열이 비어 있지 않다", bool(o.degraded), o.degraded[:60] + "...")
    check("A_m·x_off·δ 가 가정값 목록에 있다",
          all(k in e.cal.assumed for k in ("A_m", "x_off_m", "cam_yaw_offset_deg")))

    print("[9] fake_rig 배관 점검 + E6(§7.4) 상태 확인")
    try:
        import os
        sys.path.insert(0, os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))))), "tools", "etc"))
        from fake_rig import FakeCamera, FakePlant                 # noqa: E402
        from src.models.detection.detection_pose import (TagPipeline, bearing_px_deg,
                                                         pnp2_solutions)
        from src.models.detection.detection_tag import tag_edge_margin_px
        from src.models.detection.image import intrinsics_from_ref

        intr = intrinsics_from_ref((480, 640))
        fp = FakePlant(forward=3.5, lateral=+0.5, heading_deg=0.0, vertical=-1.10)
        cam = FakeCamera(fp, intr, 0.30)
        pipe = TagPipeline(intrinsics=intr, tag_size=0.30, depth_check=False)
        est = Estimator(intrinsics=intr)
        last = None
        for j in range(40):
            i, ts, img = cam.next_frame(advance=False)
            res = pipe.process(img, index=i, timestamp=ts)
            pr = res.primary(1)
            if pr is None:
                continue
            det, doc, q = pr["detection"], pr["docking"], pr["quality"]
            row = {"i": i, "seen": True, "lateral": doc["lateral"], "forward": doc["forward"],
                   "vertical": doc["vertical"], "heading_deg": doc["heading_deg"],
                   "tilt_deg": doc["tilt_deg"], "distance": doc["distance"],
                   "beta_px_deg": bearing_px_deg(det, res.intrinsics),
                   "margin_px": tag_edge_margin_px(det, img.shape),
                   "tag_px": q.get("tag_px"), "center_px": [float(v) for v in det.center],
                   "reproj_rms_px": q.get("reproj_rms_px"), "quality_ok": q.get("ok"),
                   "pnp2": pnp2_solutions(det, res.intrinsics, 0.30),
                   "gyro_deg": 0.0, "gyro_dps": 0.0, "gyro_alive": True, "movement": "stop"}
            last = est.update(FrameObs.from_row(row))
        check("배관: 진짜 파이프라인 → FrameObs → Estimate 가 돈다", last is not None)
        if last is not None:
            check("   PnP 경로는 맞다 (lateral ≈ +0.5)", abs(last.x_m - 3.5) < 0.1,
                  "x %.3f lat(PnP 원값 +0.5)" % last.x_m)
            i1 = est.identity_obs_deg
            if abs(i1) <= 0.2:
                print("  §7.4 E6: 고쳐졌다 — (I1) 원관측 잔차 %.4f° (β 부호 정상)" % i1)
                check("   E6 수정 뒤에는 lat_m > 0 ∧ beta_deg < 0",
                      last.lat_m > 0 and last.beta_deg < 0)
            else:
                print("  §7.4 E6: **아직 미수정** — (I1) 원관측 잔차 %+.3f° "
                      "(β 부호가 반대라 정확히 −2β 만큼 어긋난다)" % i1)
                print("       렌더 확인: β_px %+.2f° 인데 PnP lateral 은 %+.3f m "
                      "→ 부호가 같다(반대여야 한다)" % (last.beta_deg, 0.5))
                print("       고침: FakeCamera.T_camera_tag 의 R_tag_cam = Ry(ψ) @ diag(-1,-1,+1)")
                check("   미수정 상태를 추정기가 **부호경보로 잡아낸다**",
                      "부호경보" in last.note, last.note[:40])
        # --- E6 를 **메모리에서만** 고쳐 폐루프로 돌려 본다 -------------------
        # tools/etc/fake_rig.py 는 D팀 소유라 파일은 건드리지 않는다.
        import fake_rig as _fr

        def _T_fixed(self):
            a = math.radians(self.plant.heading)
            R = (np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0],
                           [-math.sin(a), 0.0, math.cos(a)]])
                 @ np.diag([-1.0, -1.0, 1.0]))
            t = np.array([self.plant.lateral, self.plant.vertical, -self.plant.forward])
            return R.T, -R.T @ t
        _orig = _fr.FakeCamera.T_camera_tag
        _fr.FakeCamera.T_camera_tag = _T_fixed
        try:
            fp = FakePlant(forward=5.0, lateral=+0.60, heading_deg=-18.0, vertical=-1.10)
            cam = FakeCamera(fp, intr, 0.30)
            pipe = TagPipeline(intrinsics=intr, tag_size=0.30, depth_check=False)
            est = Estimator(intrinsics=intr)
            gy, last = 0.0, None
            for name, nfr in (("stop", 30), ("rotate_ccw", 90), ("stop", 30),
                              ("forward", 150), ("stop", 40)):
                for _ in range(nfr):
                    fp.set_cmd(name)
                    i, ts_, img = cam.next_frame()
                    gy = fp.heading                      # 가짜 자이로 = 진값 적분
                    res = pipe.process(img, index=i, timestamp=ts_)
                    pr2 = res.primary(1)
                    row = {"i": i, "seen": pr2 is not None, "movement": name,
                           "gyro_deg": gy, "gyro_dps": fp.omega, "gyro_alive": True,
                           "gyro_gaps": 0, "t_cmd_set": fp.t_cmd}
                    if pr2 is not None:
                        d2, doc2, q2 = pr2["detection"], pr2["docking"], pr2["quality"]
                        row.update({"lateral": doc2["lateral"], "forward": doc2["forward"],
                                    "vertical": doc2["vertical"], "heading_deg": doc2["heading_deg"],
                                    "tilt_deg": doc2["tilt_deg"], "distance": doc2["distance"],
                                    "beta_px_deg": bearing_px_deg(d2, res.intrinsics),
                                    "margin_px": tag_edge_margin_px(d2, img.shape),
                                    "tag_px": q2.get("tag_px"),
                                    "center_px": [float(v) for v in d2.center],
                                    "reproj_rms_px": q2.get("reproj_rms_px"),
                                    "quality_ok": q2.get("ok"),
                                    "pnp2": pnp2_solutions(d2, res.intrinsics, 0.30)})
                    last = est.update(FrameObs.from_row(row))
            print("  E6 를 메모리에서 고치고 폐루프 340 프레임:")
            check("   x̂ 오차 ≤ 5 cm", abs(last.x_m - fp.forward) < 0.05,
                  "x̂ %.3f / 진값 %.3f" % (last.x_m, fp.forward))
            e_lat = last.lat_m - fp.lateral
            e_psi = last.psi_deg - fp.heading
            # 절대 문턱 대신 **추정기가 스스로 말한 σ** 로 재는 게 맞다. 앵커는 18° tilt
            # 렌더 한 장에서 서고 640x480 에서 태그가 ~40 px 라 σ_ψ 가 1° 급이다 —
            # 1.5° 라는 고정 숫자는 그 사실을 모르는 문턱이었다(2026-09-21 통합).
            check("   ψ̂ 오차가 σ 안에 든다 (|e_ψ| ≤ 2σ_ψ, 정면 구간은 자이로가 끌고 간다)",
                  abs(e_psi) <= 2.0 * last.sigma_psi_deg,
                  "ψ̂ %.2f / 진값 %.2f (σ %.2f)" % (last.psi_deg, fp.heading,
                                                  last.sigma_psi_deg))
            check("   ℓ̂ 오차가 σ 안에 든다 (|e| ≤ 2σ)", abs(e_lat) <= 2 * last.sigma_lat_m,
                  "e %.3f m / σ %.3f m" % (e_lat, last.sigma_lat_m))
            # ℓ = −x·tan(β+ψ+δ) 라 ψ̂ 가 +1° 틀리면 ℓ̂ 은 −x·1° 만큼 틀린다.
            # 3.9 m 에서 68 mm — §3.4 의 σ(e_l) ≈ 레버 × σ_ψ 가 바로 이것이다.
            check("   그 오차는 전부 −x·e_ψ 로 설명된다 (β 는 멀쩡하다)",
                  abs(e_lat + last.x_m * math.radians(e_psi)) < 0.02,
                  "−x·e_ψ = %+.3f m / e_ℓ = %+.3f m" % (-last.x_m * math.radians(e_psi), e_lat))
            check("   그래서 lateral_valid 는 False — 추정기가 스스로 못 쓴다고 말한다",
                  not last.lateral_valid, "σ_lat %.3f m > LAT_TOL/2 %.3f" % (
                      last.sigma_lat_m, C.LAT_TOL_M / 2.0))
            check("   (I1) 원관측 잔차 ≤ 0.2°", abs(est.identity_obs_deg) <= 0.2,
                  "%.4f°" % est.identity_obs_deg)
            check("   거짓 해로 넘어간 구간은 재앵커 없이 넘겼다", est.reinit_count == 0)
            if est.arm.samples:
                A_hat, u, k = est.arm.A_m, est.arm.samples[0].u, est.arm.samples[0].k
                print("  회전팔: k = −Δβ/Δψ = %.3f, u = %.3f → Â %+.3f "
                      "(fake_rig FakePlant.A_M = %+.2f)" % (k, u, A_hat, FakePlant.A_M))
                # 2026-09-21 통합: fake_rig 의 A_M 부호를 §1 의 A 에 맞췄다
                # (FakePlant.step: ℓ += A·(sin(h+dψ)−sin h) → dℓ/dψ = +A).
                # 그래서 이제 **크기뿐 아니라 부호까지** 맞아야 한다.
                check("   Â 가 크기·부호 모두 맞는다",
                      abs(A_hat - FakePlant.A_M) < 0.12,
                      "Â %+.3f / 플랜트 %+.2f" % (A_hat, FakePlant.A_M))
        finally:
            _fr.FakeCamera.T_camera_tag = _orig
    except Exception as exc:
        print("  건너뜀 (%s: %s)" % (type(exc).__name__, exc))

    print("\n%s  (%d 실패)" % ("전부 통과" if not fails else "실패: " + ", ".join(fails),
                              len(fails)))
    sys.exit(1 if fails else 0)
