"""β — 태그 중심 픽셀 하나에서 나오는 방위각. (plan 1-2, contracts_B §1)

왜 픽셀인가
────────────────────────────────────────────────────────────────────────
lateral 을 PnP 전체 자세에서 뽑으면 yaw 오차가 **거리로 증폭**된다(8 m·1° = 140 mm).
9/7 의 7~10 m lateral 잡음 60~145 mm 가 그것이다. β 는 중심 픽셀 하나에서 나와
**거리와 무관**하고, PnP 2중해·정면 heading 바이어스의 영향을 받지 않는다 —
그래서 이게 1차 관측량이고 lateral 은 β 에서 재구성한다.

부호 (contracts_B §1, 2026-09-21 맥 실측)
────────────────────────────────────────────────────────────────────────
태그가 화면 **오른쪽**에 보이면 β < 0 (그때 ℓ > 0 — 둘은 부호가 반대다).
δ(카메라 장착 yaw)는 **β 에 안 들어 있다** — β 는 광축 기준 생값이다.

σ_β 는 프레임마다 실시간으로 낸다 (고정 표 금지)
────────────────────────────────────────────────────────────────────────
    β  = −atan2(x_n, 1),  x_n = (u − cx)/fx            (왜곡 보정 뒤)
    dβ/du = −1 / (fx·(1 + x_n²))                        [rad/px]
    중심 u 는 코너 4개에서 나오므로 σ_u ≈ σ_c/2 (코너 독립 가정)
    → **σ_β = σ_c / (2·fx·(1 + x_n²))  [rad]**

    손검산: fx 604(640x480), σ_c 0.3 px, 화면 중앙 → 0.3/(2·604) = 2.5e-4 rad = 0.014°
            1080p(fx 1359) 이면 0.0063°. lateral 로는 3.5 m 에서 0.9 mm.
            같은 자리의 σ_ψ 는 1°(= 61 mm) 다 — β 가 왜 1차인지가 이 두 줄에 있다.

σ_c 는 상수로 박지 않는다 — **옳은 해의 픽셀 재투영 잔차**(quality.reproj_rms_px)를
창 하나로 모아 쓴다(plan 1-4: 정지 0.07~0.13 px, 주행 ≈0.3 px. config 의
CORNER_NOISE_PX=0.07 은 주행에서 낙관이다). 창 통계이지 온라인 적응이 아니다 —
잡음 모델 파라미터를 주행 중에 학습·미분하지 않는다(plan 2-6).

왜곡: v2 는 `detect()` 에 왜곡계수를 **안 넘겼다**(라이브러리 자세는 미보정,
pose_by_pnp 만 보정 — 두 모델 혼용). 여기서는 중심 픽셀을 직접 편다.

자기검증: `python -m src.models.estimate.bearing`
"""
import math
from dataclasses import dataclass

# ── 코드 내부 상수 (config 아님 — 현장에서 사람이 고칠 값이 아니다) ───────────
SIGMA_C_WINDOW = 30          #: σ̂_c 를 모으는 프레임 수 (plan 1-4 "30 프레임 평균")
SIGMA_C_FLOOR_PX = 0.07      #: σ̂_c 하한 [px]. 9/7 정지 실측 최솟값 — 이보다 낙관하지 않는다
SIGMA_C_CEIL_PX = 2.0        #: σ̂_c 상한 [px]. 검출이 이 정도로 나빠지면 quality 가 먼저 막는다
SIGMA_BETA_FLOOR_DEG = 0.010 #: σ_β 하한 [°]. 3.5 m 에서 0.6 mm — 잡음 0 인 관측은 없다
UNDISTORT_ITERS = 5          #: 왜곡 역산 반복 수. cv2.undistortPoints 기본값과 같게 맞췄다
CUT_MARGIN_PX_DEFAULT = 60.0 #: 태그 컷 여유 기본값 [px] (dynamics_calib.tag_cut.margin_px 없을 때)
BETA_VIS_DEG_DEFAULT = 28.0  #: 가시 한계 |β| 기본값 [°]. HFOV 69°/2 − 태그 반각


def wrap180(deg):
    """각을 (−180, 180] 으로."""
    d = float(deg)
    if d != d:                       # NaN
        return d
    d = (d + 180.0) % 360.0 - 180.0
    return 180.0 if d == -180.0 else d


def undistort_normalized(u, v, fx, fy, cx, cy, coeffs=(), model=None):
    """픽셀 → 왜곡 편 정규화 좌표 (x_n, y_n).

    coeffs 는 radtan (k1, k2, p1, p2, k3). 비어 있거나 전부 0 이면 그냥 나눈다.
    model 에 "inverse" 가 들어 있으면(RealSense inverse_brown_conrady) 계수가
    **역방향**을 기술하므로 역산이 아니라 다항식을 그대로 먹인다 — plan 1-2 가
    `i.model` 을 버리지 말라고 한 이유가 이것이다.

    cv2.undistortPoints 와 같은 반복식이다(자기검증 루틴에서 1e-9 안으로 일치 확인).
    """
    x = (float(u) - cx) / fx
    y = (float(v) - cy) / fy
    k = list(coeffs or ())[:5] + [0.0] * (5 - len(list(coeffs or ())[:5]))
    if not any(k):
        return x, y
    k1, k2, p1, p2, k3 = k
    if model is not None and "inverse" in str(model).lower():
        # 계수가 이미 "정규화 → 왜곡" 방향. 한 번 먹이면 편 좌표가 된다.
        r2 = x * x + y * y
        rad = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        return x * rad + dx, y * rad + dy
    x0, y0 = x, y
    for _ in range(UNDISTORT_ITERS):
        r2 = x * x + y * y
        icd = 1.0 / (1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2)
        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (x0 - dx) * icd
        y = (y0 - dy) * icd
    return x, y


def beta_deg(center_px, intr):
    """태그 중심 픽셀 → β [°]. + 가 반시계(태그가 화면 **왼쪽**).

    detection_pose.bearing_px_deg 와 같은 값이어야 한다(자기검증에서 대조한다).
    """
    if center_px is None or intr is None:
        return float("nan")
    try:
        u, v = float(center_px[0]), float(center_px[1])
    except (TypeError, IndexError, ValueError):
        return float("nan")
    xn, _ = undistort_normalized(u, v, intr.fx, intr.fy, intr.cx, intr.cy,
                                 getattr(intr, "distortion", ()) or (),
                                 getattr(intr, "model", None))
    return -math.degrees(math.atan2(xn, 1.0))


def sigma_beta_deg(sigma_c_px, fx, x_n=0.0):
    """위 유도 그대로. σ_c [px] → σ_β [°]."""
    if not fx or fx <= 0:
        return float("nan")
    s = sigma_c_px / (2.0 * fx * (1.0 + x_n * x_n))
    return max(math.degrees(s), SIGMA_BETA_FLOOR_DEG)


class CornerNoise:
    """σ̂_c — 재투영 잔차 창 하나. (plan 1-4)

    평균 대신 **중앙값**을 쓴다: 한 프레임 blur·반사로 잔차가 튀어도 σ 가 통째로
    커지면 그 프레임 이후 판단이 전부 굳는다. 창 통계일 뿐 적응이 아니다.
    """

    def __init__(self, window=SIGMA_C_WINDOW, fallback_px=None):
        self.window = int(window)
        self.fallback = SIGMA_C_FLOOR_PX if fallback_px is None else float(fallback_px)
        self._buf = []
        self.assumed = True          # 아직 관측이 없어 fallback 을 쓰는 중인가

    def push(self, reproj_rms_px):
        if reproj_rms_px is None:
            return
        try:
            r = float(reproj_rms_px)
        except (TypeError, ValueError):
            return
        if not (r == r) or r < 0.0:
            return
        self._buf.append(min(r, SIGMA_C_CEIL_PX))
        if len(self._buf) > self.window:
            del self._buf[0]

    @property
    def n(self):
        return len(self._buf)

    @property
    def sigma_c_px(self):
        if not self._buf:
            self.assumed = True
            return self.fallback
        self.assumed = False
        s = sorted(self._buf)
        m = len(s) // 2
        med = s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])
        return min(max(med, SIGMA_C_FLOOR_PX), SIGMA_C_CEIL_PX)


@dataclass(frozen=True)
class BearingObs:
    """한 프레임의 β 와 그 신뢰도·가시성."""
    beta_deg: float          # + = 태그가 화면 왼쪽. δ 미보정 생값
    sigma_deg: float         # 1σ [°]
    sigma_c_px: float        # 이 프레임에 쓴 σ̂_c
    x_n: float               # 정규화 가로좌표 (σ 유도에 쓴 값)
    margin_px: float         # 태그가 화면 가장자리에서 떨어진 px. 0 이면 검출이 끊긴다
    tag_px: float
    seen: bool
    visible_ok: bool         # |β| 가 가시 한계 안인가
    cut_soon: bool           # margin_px 가 컷 문턱 아래인가 — 이 다리는 블라인드로 끝난다
    sigma_assumed: bool      # σ̂_c 가 관측이 아니라 가정값인가
    note: str = ""


class Bearing:
    """프레임마다 β 를 내고 σ 를 실시간으로 붙인다.

    intr 이 있으면 중심 픽셀에서 직접 β 를 다시 계산하고(왜곡 보정 포함), 로그의
    beta_px_deg 와 대조한다. 오프라인 재생처럼 intr 이 없으면 로그값을 쓴다.
    """

    #: 로그의 β 와 다시 계산한 β 가 이보다 벌어지면 왜곡·내부파라미터가 어긋난 것 [°]
    RECOMPUTE_TOL_DEG = 0.05

    def __init__(self, intrinsics=None, sigma_c_fallback=None,
                 cut_margin_px=None, beta_vis_deg=None):
        self.intr = intrinsics
        self.noise = CornerNoise(fallback_px=sigma_c_fallback)
        self.cut_margin_px = (CUT_MARGIN_PX_DEFAULT if cut_margin_px is None
                              else float(cut_margin_px))
        vis = BETA_VIS_DEG_DEFAULT if beta_vis_deg is None else beta_vis_deg
        if isinstance(vis, (tuple, list)) and len(vis) == 2:
            self.vis_left_deg, self.vis_right_deg = abs(float(vis[0])), abs(float(vis[1]))
        else:
            self.vis_left_deg = self.vis_right_deg = abs(float(vis))
        self.n_recompute_mismatch = 0

    def update(self, obs):
        """FrameObs → BearingObs. 태그가 없으면 seen=False + NaN 으로 돌려준다."""
        if not obs.tag_seen or obs.beta_px_deg is None:
            return BearingObs(beta_deg=float("nan"), sigma_deg=float("nan"),
                              sigma_c_px=self.noise.sigma_c_px, x_n=float("nan"),
                              margin_px=float("nan"), tag_px=float("nan"),
                              seen=False, visible_ok=False, cut_soon=False,
                              sigma_assumed=self.noise.assumed, note="태그 없음")

        self.noise.push(obs.reproj_rms_px)
        sc = self.noise.sigma_c_px

        b_log = float(obs.beta_px_deg)
        note = ""
        b = b_log
        xn = -math.tan(math.radians(b))          # intr 이 없을 때의 x_n (β 자체에서 역산)
        if self.intr is not None and obs.center_px is not None:
            b_calc = beta_deg(obs.center_px, self.intr)
            if b_calc == b_calc:
                xn, _ = undistort_normalized(float(obs.center_px[0]), float(obs.center_px[1]),
                                             self.intr.fx, self.intr.fy,
                                             self.intr.cx, self.intr.cy,
                                             getattr(self.intr, "distortion", ()) or (),
                                             getattr(self.intr, "model", None))
                if abs(wrap180(b_calc - b_log)) > self.RECOMPUTE_TOL_DEG:
                    # 로그의 β 와 다시 계산한 β 가 다르다 = 왜곡 모델이나 내부파라미터가
                    # 파이프라인과 다르다. 조용히 덮지 말고 남긴다.
                    self.n_recompute_mismatch += 1
                    note = "β 재계산 불일치 %.3f°" % wrap180(b_calc - b_log)
                b = b_calc

        fx = getattr(self.intr, "fx", None)
        sig = sigma_beta_deg(sc, fx, xn) if fx else max(sc * 0.02, SIGMA_BETA_FLOOR_DEG)

        margin = float("nan") if obs.margin_px is None else float(obs.margin_px)
        tagpx = float("nan") if obs.tag_px is None else float(obs.tag_px)
        lim = self.vis_left_deg if b > 0 else self.vis_right_deg
        return BearingObs(beta_deg=b, sigma_deg=sig, sigma_c_px=sc, x_n=xn,
                          margin_px=margin, tag_px=tagpx, seen=True,
                          visible_ok=bool(abs(b) <= lim),
                          cut_soon=bool(margin == margin and margin <= self.cut_margin_px),
                          sigma_assumed=self.noise.assumed, note=note)


__all__ = ["Bearing", "BearingObs", "CornerNoise", "beta_deg", "sigma_beta_deg",
           "undistort_normalized", "wrap180",
           "SIGMA_C_WINDOW", "SIGMA_C_FLOOR_PX", "SIGMA_BETA_FLOOR_DEG"]


# ===========================================================================
# 자기검증 — `python -m src.models.estimate.bearing` (리포 루트에서)
# 합성 데이터로 (1) 부호 (2) 왜곡 역산 (3) σ_β 예측이 실제 흔들림과 맞는지 본다.
# ===========================================================================
if __name__ == "__main__":
    import os
    import random
    import sys

    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))))

    class _Intr:
        def __init__(self, fx, fy, cx, cy, distortion=(), model=None):
            self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
            self.distortion, self.model = distortion, model

        @property
        def K(self):
            import numpy as np
            return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    fails = []

    def check(name, cond, detail=""):
        print("  %-46s %s %s" % (name, "OK " if cond else "실패", detail))
        if not cond:
            fails.append(name)

    intr = _Intr(604.089, 604.0, 318.622, 253.911)
    print("[1] 부호 — contracts_B §1.4 실측표와 같은가")
    # ℓ>0(태그 오른쪽), ψ=δ=0 → 항등식 (I1) 으로 β 를 만들고 픽셀을 되짚는다
    for lat, x, psi, delta in ((+0.707, 3.0, 0.0, 0.0), (-0.696, 3.0, 0.0, 0.0),
                               (+0.5, 3.5, +3.0, -2.0), (-0.2, 8.0, -7.0, +1.5)):
        b_true = wrap180(-math.degrees(math.atan2(lat, x)) - psi - delta)
        u = intr.cx + intr.fx * (-math.tan(math.radians(b_true)))
        got = beta_deg((u, intr.cy), intr)
        check("ℓ=%+.3f x=%.1f ψ=%+.1f δ=%+.1f → β" % (lat, x, psi, delta),
              abs(wrap180(got - b_true)) < 1e-9, "β=%+.4f°" % got)
    u_right = intr.cx + 60.0
    check("태그가 화면 오른쪽(u>cx) → β<0", beta_deg((u_right, intr.cy), intr) < 0,
          "β=%+.3f°" % beta_deg((u_right, intr.cy), intr))
    check("태그가 화면 왼쪽(u<cx) → β>0", beta_deg((intr.cx - 60.0, intr.cy), intr) > 0)

    print("[2] 왜곡 역산이 cv2.undistortPoints 와 같은가")
    try:
        import cv2
        import numpy as np
        d = (-0.12, 0.05, 0.001, -0.002, 0.01)
        it = _Intr(604.089, 604.0, 318.622, 253.911, distortion=d)
        worst = 0.0
        for u, v in ((100.0, 90.0), (318.6, 253.9), (600.0, 460.0), (20.0, 440.0)):
            ref = cv2.undistortPoints(np.array([[[u, v]]], dtype=np.float64), it.K,
                                      np.array(d, dtype=np.float64))
            mine = undistort_normalized(u, v, it.fx, it.fy, it.cx, it.cy, d)
            worst = max(worst, abs(ref[0, 0, 0] - mine[0]), abs(ref[0, 0, 1] - mine[1]))
        check("cv2 와 최대 차", worst < 1e-9, "%.2e" % worst)
        check("왜곡계수를 먹이면 β 가 실제로 달라진다",
              abs(beta_deg((100.0, 90.0), it) - beta_deg((100.0, 90.0), intr)) > 0.1,
              "Δ=%.3f°" % (beta_deg((100.0, 90.0), it) - beta_deg((100.0, 90.0), intr)))
    except ImportError:
        print("  cv2 없음 — 건너뜀")

    print("[3] σ_β 예측 vs 몬테카를로 (코너잡음 → 중심잡음 σ_c/2)")
    random.seed(0)
    for sc, u0 in ((0.30, intr.cx), (0.30, intr.cx + 180.0), (0.80, intr.cx)):
        xn0, _ = undistort_normalized(u0, intr.cy, intr.fx, intr.fy, intr.cx, intr.cy)
        pred = sigma_beta_deg(sc, intr.fx, xn0)
        s = [beta_deg((u0 + random.gauss(0.0, sc / 2.0), intr.cy), intr) for _ in range(20000)]
        m = sum(s) / len(s)
        emp = math.sqrt(sum((v - m) ** 2 for v in s) / (len(s) - 1))
        check("σ_c=%.2f u0−cx=%+4.0f  예측 %.4f° / 실측 %.4f°" % (sc, u0 - intr.cx, pred, emp),
              abs(pred - emp) / emp < 0.05)

    print("[4] σ̂_c 창 — 중앙값이라 한 프레임 튐에 안 끌려간다")
    cn = CornerNoise(window=30)
    check("관측 전에는 가정값", cn.assumed and abs(cn.sigma_c_px - SIGMA_C_FLOOR_PX) < 1e-12)
    for _ in range(29):
        cn.push(0.30)
    cn.push(1.9)
    check("29x0.30 + 1x1.9 → 중앙값 0.30", abs(cn.sigma_c_px - 0.30) < 1e-9,
          "%.3f px" % cn.sigma_c_px)
    check("창을 넘으면 옛 표본이 빠진다", cn.n == 30)
    cn2 = CornerNoise(window=5)
    for _ in range(5):
        cn2.push(0.01)
    check("하한 아래는 하한으로", abs(cn2.sigma_c_px - SIGMA_C_FLOOR_PX) < 1e-12)

    print("[5] detection_pose.bearing_px_deg 와 같은 값인가 (파이프라인 대조)")
    try:
        from src.models.detection.detection_pose import bearing_px_deg
        from src.models.detection.image import CameraIntrinsics

        class _Det:
            def __init__(self, c):
                self.center = c
        worst = 0.0
        for dist in ((), (-0.12, 0.05, 0.001, -0.002, 0.01)):
            ci = CameraIntrinsics(fx=604.089, fy=604.0, cx=318.622, cy=253.911,
                                  width=640, height=480, distortion=dist)
            for u, v in ((100.0, 90.0), (318.6, 253.9), (600.0, 460.0)):
                a = bearing_px_deg(_Det((u, v)), ci)
                b = beta_deg((u, v), ci)
                worst = max(worst, abs(wrap180(a - b)))
        check("최대 차", worst < 1e-9, "%.2e°" % worst)
    except Exception as exc:
        print("  검출 모듈 import 실패(%s) — 건너뜀" % exc)

    print("[6] Bearing.update — 가시성·컷 판정")

    class _Obs:
        tag_seen = True
        beta_px_deg = -13.08
        center_px = None
        reproj_rms_px = 0.31
        margin_px = 40.0
        tag_px = 120.0
    b = Bearing(intrinsics=None, cut_margin_px=60.0, beta_vis_deg=28.0)
    o = b.update(_Obs())
    check("margin 40 < 컷 60 → cut_soon", o.cut_soon and o.seen)
    check("|β| 13 < 28 → visible_ok", o.visible_ok)
    _Obs.beta_px_deg = -31.0
    check("|β| 31 > 28 → visible_ok False", not b.update(_Obs()).visible_ok)
    _Obs.tag_seen = False
    o = b.update(_Obs())
    check("태그 없음 → seen False·β NaN", (not o.seen) and o.beta_deg != o.beta_deg)

    print("\n%s  (%d 실패)" % ("전부 통과" if not fails else "실패: " + ", ".join(fails),
                              len(fails)))
    sys.exit(1 if fails else 0)
