"""검출 -> 믿을 만한 값. **멈춰서 30 프레임 모으지 않는다** (plan 3절).

v2 는 정지한 뒤 30 프레임 중앙값을 썼다. v4 는 가면서 계속 추정한다. 그래서
"몇 개 모았나" 가 아니라 "**지금 값이 얼마고 얼마나 흔들리나**" 를 답한다.

세 갈래로만 답한다 (plan 3-6):
    ok        믿을 만하다. 그 값으로 움직여라
    uncertain 봤지만 못 믿겠다 — **고치지 말고 다른 걸 해라** (더 보기·전진·문턱 낮추기)
    no_tag    못 봤다

창 안에 직선을 맞춰 **가장 최근 노출 시각의 값**을 낸다. 움직이는 중에는 창 평균이
그 자체로 과거값이라 틀린다. 기울기는 덤으로 나온다.

σ = √(밀림² + 떨림²)  (2026-09-30 결정 8, plan 3-6)
    밀림  before_run 이 정지 60초에서 잰 "0.5초 창 평균의 흔들림"(Measured.sigma_drift_*).
          창 안에서 평균을 내도 안 줄어드는 느린 성분이다. 9/21 3.68 m 정지 1459 프레임:
          잡음의 60 % 가 자기상관 0.6 인 느린 성분이라 30장 평균 sd 가 30.9 mm 였다
          (독립이면 7.2 mm). 잔차만 보던 예전 Track 은 같은 자료에서 σ 를 3 배 작게 냈다
          (plan 문제 5-1 ③). 기준선 계산식은 drift_baseline() — before_run 도 이걸 쓴다.
    떨림  지금 창의 직선 잔차 sd ÷ √n, 창 끝 지렛대 포함. 프레임마다 새로 나온다.
밀림은 잰 거리에서 지금 거리로 옮긴다: 방향 ∝ 거리, 좌우 = 거리 × 방향 ∝ 거리² (plan 문제 1).
직진 중(fix(moving=True))엔 직진 60초 기준선(sigma_drive_*)을 쓴다. 없으면 정지 것 (plan 3-4).
기준선이 없으면 σ 를 못 내므로 **조용히 기본값을 쓰지 않고** why="no_sigma" 로 답한다.

두 PnP 해가 헷갈린 프레임(angle_ok=False)은 **한 장씩** 각도 계열(좌우·방향·전방)에서 뺀다.
거리·화면위치는 회전행렬과 무관하니 그대로 넣는다. 깨끗한 장이 MIN_N 미만이면 "few".
측정 나이는 clock.age_ok 로 본다 — 음수(−1 ms 아래)면 시계가 섞인 것("clock"), 0.30 s 넘으면 "stale".
회전 중엔 부르지 않는다 — 자세를 안 믿는다(결정 7). 회전이 끝나면 reset().
"""
if __name__ == "__main__":      # 스크립트로 돌릴 때만 경로를 잡는다. bootstrap.repo_root 와 같은 규칙
    import sys
    from pathlib import Path
    sys.path.insert(0, str(next(d for d in Path(__file__).resolve().parents
                               if (d / "config").is_dir() and (d / "src").is_dir())))

import math
from collections import deque
from dataclasses import dataclass
from typing import NamedTuple

from config import control as C
from config import detection as D
from src.models.detection import pose as P
from src.models.detection import tag as T
from src.utils import clock

WINDOW_S = 0.5         # 이 시간 안의 검출만 쓴다. 밀림 기준선의 "0.5초 블록" 과 같은 값이어야 한다
MIN_N = 3              # 직선을 맞추려면 최소 세 점
MAD_K = 1.4826         # MAD -> 표준편차 환산
INLIER_K = 3.0         # 중앙값에서 이 배 밖이면 버린다


@dataclass
class Fix:
    """지금 아는 것. sigma 는 **이 값 자체의** 흔들림이지 한 프레임의 잡음이 아니다.

    *_sigma = √(밀림² + 떨림²). 두 조각(*_drift_sigma · *_fast_sigma)은 진단으로 따로 남긴다.
    모르는 σ 는 무한대다 — 0 이면 "확실하다" 로 읽혀 위험하다.
    거리의 밀림 기준선(sigma_drift_distance_m)은 ★Measured 에 아직 없다. 그때까지 distance_sigma_m
    은 떨림만이라 1초 사이의 변화도 3 배쯤 작게 본다 (9/21 정지: 떨림 1.3 mm, 1초 변화 sd 3.5 mm,
    0.5초 블록 밀림 11.5 mm). 출발·멎음 문턱(forward.py)에 쓰려면 그 기준선이 있어야 한다.
    """
    ok: bool = False
    why: str = "no_tag"                 # ok / few / stale / clock / no_sigma / no_tag
    lateral_m: float = 0.0
    heading_deg: float = 0.0
    forward_m: float = 0.0
    distance_m: float = 0.0
    beta_deg: float = 0.0
    lateral_sigma_m: float = float("inf")
    heading_sigma_deg: float = float("inf")
    lateral_fast_sigma_m: float = float("inf")      # 떨림 (이 창의 잔차)
    heading_fast_sigma_deg: float = float("inf")
    lateral_drift_sigma_m: float | None = None      # 밀림 (지금 거리로 옮긴 것). 기준선 없으면 None
    heading_drift_sigma_deg: float | None = None
    drift_from: str = ""                            # 밀림 출처 — "still"(정지 60초) / "drive"(직진 60초)
    distance_sigma_m: float = float("inf")
    distance_fast_sigma_m: float = float("inf")
    distance_drift_sigma_m: float | None = None
    beta_sigma_deg: float = float("inf")            # 떨림만. 화면위치는 계산이 안 들어가 밀림도 작다 (9/21: 0.006도)
    closing_mps: float = 0.0            # 태그에 다가가는 속도. 기울기에서 덤으로 나온다
    edge_px: float = float("nan")       # 화면 가장자리까지 (네 변 중 제일 가까운 쪽). 0 이 되는 순간 검출이 끊긴다
    top_px: float = float("nan")        # 윗변 행 [px, 화면 위에서]. 태그컷은 이게 TAG_EDGE_MARGIN_PX 에 닿는 순간 — limits.tag_cut_live_m
    vertical_m: float = float("nan")    # 태그 기준 카메라 높이 (아래가 −). 위치라 피치와 무관. −이것 = 지금 자리의 높이차
    vertical_fast_sigma_m: float = float("inf")
    tilt_deg: float = 0.0
    tag_roll_deg: float | None = None
    n: int = 0                          # 창 안 프레임 (게이트 통과분)
    ambiguous: int = 0                  # 그중 두 해가 헷갈려 각도 계열에서 뺀 장 수
    n_fit: int = 0                      # 좌우 직선에 실제로 쓴 점 (튀는 점 뺀 뒤)
    t_capture: float = 0.0
    age_s: float = 0.0

    @property
    def uncertain(self):
        return not self.ok and self.why != "no_tag"


class LineFit(NamedTuple):
    """창 안 직선을 한 시각에서 읽은 것."""
    value: float
    sigma_fast: float      # 떨림만 — 잔차 sd × √(1/n + 지렛대). 밀림은 Estimator 가 얹는다
    slope: float           # 단위/s
    n: int                 # 실제로 쓴 점


def line_fit(pts, t_eval):
    """(시각, 값) 점들에 직선을 맞춰 t_eval 에서 읽는다. 점은 셋 이상."""
    n = len(pts)
    mt = sum(t for t, _ in pts) / n
    mv = sum(v for _, v in pts) / n
    sxx = sum((t - mt) ** 2 for t, _ in pts)
    slope = (sum((t - mt) * (v - mv) for t, v in pts) / sxx) if sxx > 0 else 0.0
    te = t_eval - mt
    resid2 = sum((v - mv - slope * (t - mt)) ** 2 for t, v in pts)
    s_res = math.sqrt(resid2 / (n - 2)) if n > 2 else 0.0
    # 창 끝에서 직선을 읽으면 가운데보다 덜 확실하다. 그 지렛대를 같이 센다
    lev = 1.0 / n + (te * te / sxx if sxx > 0 else 0.0)
    return LineFit(mv + slope * te, s_res * math.sqrt(lev), slope, n)


class Track:
    """창 안의 (시각, 값) 에 직선을 맞춘다. 튀는 점은 버린다."""

    def __init__(self, window_s=WINDOW_S):
        self.window_s = window_s
        self.pts = deque()

    def add(self, t, v):
        self.pts.append((t, float(v)))

    def prune(self, t_now):
        """점이 안 들어온 프레임에도 부른다 — 헷갈린 장을 건너뛴 트랙이 옛 점을 끌어안으면 안 된다."""
        while self.pts and t_now - self.pts[0][0] > self.window_s:
            self.pts.popleft()

    def clear(self):
        self.pts.clear()

    def at_last(self, t_eval=None):
        """t_eval(없으면 마지막 점) 시각의 LineFit. 쓸 점이 MIN_N 미만이면 None."""
        pts = self._inliers()
        if len(pts) < MIN_N:
            return None
        return line_fit(pts, pts[-1][0] if t_eval is None else t_eval)

    def _inliers(self):
        pts = list(self.pts)
        if len(pts) < MIN_N:
            return pts
        vs = sorted(p[1] for p in pts)
        med = vs[len(vs) // 2]
        mad = sorted(abs(v - med) for v in vs)[len(vs) // 2] * MAD_K
        if mad <= 0:
            return pts
        keep = [p for p in pts if abs(p[1] - med) <= INLIER_K * mad]
        # 빼고도 직선을 맞출 만큼 남을 때만 뺀다. 세 점의 MAD 는 우연이라 그걸로 한 점을 버리면 안 된다
        return keep if len(keep) >= MIN_N else pts


class Drift:
    """밀림 기준선(before_run 이 잰 Measured 값) -> 지금 거리의 밀림 σ."""

    def __init__(self, noise):
        def g(k):
            return getattr(noise, k, None)
        self.ref_m = g("sigma_ref_distance_m")
        self.still = (g("sigma_drift_lateral_m"), g("sigma_drift_heading_deg"))
        self.drive = (g("sigma_drive_lateral_m"), g("sigma_drive_heading_deg"))   # 직진 60초. 없으면 정지 것
        self.distance_m = g("sigma_drift_distance_m")   # ★Measured 에 아직 없다. 생기면 자동으로 쓴다

    @property
    def ok(self):
        return (self.ref_m is not None and self.ref_m > 0
                and self.still[0] is not None and self.still[1] is not None)

    def at(self, distance_m, moving=False):
        """(좌우, 방향, 거리, 출처) 밀림 σ. 방향 잡음은 태그 픽셀 크기에 반비례 -> ∝ 거리.
        좌우 = 거리 × 방향 -> ∝ 거리² (9/7 38개·9/21 1459프레임 비율 1.00, plan 문제 1).
        거리도 픽셀 크기에서 오니 ∝ 거리²."""
        (lat, head), src = self.still, "still"
        if moving and self.drive[0] is not None and self.drive[1] is not None:
            (lat, head), src = self.drive, "drive"
        r = distance_m / self.ref_m
        return (lat * r * r, head * r,
                None if self.distance_m is None else self.distance_m * r * r, src)


class Estimator:
    """검출을 받아 쌓고, 물으면 Fix 를 준다. 게이트도 여기서 건다. 한 스레드(제어)에서만 부른다 (plan 4-2-2)."""

    ANGLE_KEYS = ("lateral", "heading_deg", "forward", "vertical")   # 회전행렬에서 나온다. 두 해가 헷갈리면 다 못 믿는다
    ALWAYS_KEYS = ("distance", "beta_deg", "top_px")                 # |t| 와 화면위치. 어느 해든 같다
    KEYS = ANGLE_KEYS + ALWAYS_KEYS

    def __init__(self, intrinsics, tag_size=None, cam_yaw_offset_deg=0.0,
                 tag_roll_correction_deg=0.0, noise=None, window_s=WINDOW_S):
        """noise 는 config.measured.Measured (sigma_ref_distance_m · sigma_drift_* · sigma_drive_*).
        None 이면 값은 내되 ok 가 안 된다(no_sigma). window_s 를 바꾸면 밀림 기준선도 같은 창으로 다시 재야 한다."""
        self.intr = intrinsics
        self.tag_size = tag_size or D.TAG_SIZE_M
        self.cam_yaw_offset_deg = float(cam_yaw_offset_deg)
        self.tag_roll_correction_deg = float(tag_roll_correction_deg)
        self.drift = Drift(noise)
        self.tracks = {k: Track(window_s) for k in self.KEYS}
        self.window_s = window_s
        self.flags = deque()            # (t, 두해모호?, 가장자리px, tilt, roll)
        self.last_t = 0.0
        self.rejected = {}

    def reset(self):
        """회전처럼 값이 뚝 끊기는 동작 **뒤에** 부른다. 옛 점이 섞이면 기울기가 거짓말한다."""
        for tr in self.tracks.values():
            tr.clear()
        self.flags.clear()

    def observe(self, t_capture, detection, shape, accel=None):
        """한 프레임. t_capture 는 CameraClock.see() 를 거친 우리 시계. 게이트를 통과하면 쌓고 state 를 돌려준다."""
        if detection is None:
            return None
        why = self._gate(detection)
        if why:
            self._reject(why)
            return None
        Tm, rms = P.pose_by_pnp(detection, self.intr, self.tag_size)
        if not (rms <= D.MAX_REPROJ_RMS_PX):
            self._reject("reproj")
            return None
        st = P.state(Tm, detection=detection, intrinsics=self.intr,
                     tag_size=self.tag_size,
                     cam_yaw_offset_deg=self.cam_yaw_offset_deg,
                     tag_roll_correction_deg=self.tag_roll_correction_deg,
                     accel=accel)
        st["reproj_px"] = rms
        st["edge_px"] = T.tag_edge_margin_px(detection, shape)
        st["top_px"] = float(min(c[1] for c in detection.corners))   # 윗변 행 — 태그컷을 프레임마다 다시 재는 입력
        return st if self.push(t_capture, st) else None

    def push(self, t_capture, st):
        """자세 하나를 창에 넣는다. observe 의 뒷부분 — 기록 재생·시험에서 바로 부른다.

        시각이 안 늘면 안 넣는다(같은 프레임 두 번·순서 뒤집힘). 창의 "마지막 점" 이 최신이어야 한다.
        """
        if not (t_capture > self.last_t):
            self._reject("order")
            return False
        clean = bool(st.get("angle_ok", True))
        for k in self.KEYS:
            v = st.get(k)
            if v is not None and math.isfinite(v) and (clean or k in self.ALWAYS_KEYS):
                self.tracks[k].add(t_capture, v)
            self.tracks[k].prune(t_capture)
        self.flags.append((t_capture, not clean, st.get("edge_px", float("nan")),
                           st.get("tilt_deg", 0.0), st.get("tag_roll_deg")))
        while self.flags and t_capture - self.flags[0][0] > self.window_s:
            self.flags.popleft()
        self.last_t = t_capture
        return True

    def fix(self, now=None, moving=False):
        """지금 아는 것. 세 갈래 중 하나로 답한다. now 는 우리 시계(clock.now) 여야 한다.
        moving=True 면 밀림을 직진 기준선에서 (plan 3-4). 회전 중엔 부르지 않는다 (결정 7)."""
        now = clock.now() if now is None else now
        f = Fix(t_capture=self.last_t,
                age_s=(now - self.last_t) if self.last_t else float("inf"))
        if not self.flags:
            return f
        f.n = len(self.flags)
        f.ambiguous = sum(1 for x in self.flags if x[1])
        edges = [x[2] for x in self.flags if x[2] == x[2]]
        f.edge_px = min(edges) if edges else float("nan")
        f.tilt_deg = self.flags[-1][3]
        f.tag_roll_deg = self.flags[-1][4]
        if not clock.age_ok(f.age_s):
            f.why = "clock" if f.age_s < 0 else "stale"   # 음수 = 카메라 스탬프를 우리 시계로 안 옮겼다
            return f
        if f.n < MIN_N:
            f.why = "few"
            return f

        # 모든 값을 **마지막 노출 시각**에서 읽는다. 마지막 장이 헷갈려 각도 트랙에 안 들어갔어도
        # 각도는 그 시각까지 직선을 늘려 읽는다 — 지렛대가 그만큼 σ 를 키운다
        got = {k: self.tracks[k].at_last(self.last_t) for k in self.KEYS}
        beta, dist = got["beta_deg"], got["distance"]
        if beta is None or dist is None:
            f.why = "few"
            return f
        f.beta_deg, f.beta_sigma_deg = beta.value, beta.sigma_fast
        f.distance_m, f.distance_fast_sigma_m = dist.value, dist.sigma_fast
        f.distance_sigma_m = dist.sigma_fast
        f.closing_mps = -dist.slope
        if got["forward"]:
            f.forward_m = got["forward"].value
        if got["top_px"]:
            f.top_px = got["top_px"].value
        if got["vertical"]:
            f.vertical_m, f.vertical_fast_sigma_m = got["vertical"].value, got["vertical"].sigma_fast
        lat, head = got["lateral"], got["heading_deg"]
        if lat is None or head is None:
            f.why = "few"                   # 깨끗한 장이 모자란다 (헷갈린 장은 ambiguous 에 세어 둔다)
            return f
        f.lateral_m, f.lateral_fast_sigma_m = lat.value, lat.sigma_fast
        f.heading_deg, f.heading_fast_sigma_deg = head.value, head.sigma_fast
        f.n_fit = lat.n
        if not self.drift.ok:
            f.why = "no_sigma"              # 밀림 기준선이 없다. 떨림만으로는 3 배 과신한다 — 못 믿는다
            return f
        dl, dh, dd, f.drift_from = self.drift.at(f.distance_m, moving)
        f.lateral_drift_sigma_m, f.heading_drift_sigma_deg = dl, dh
        f.lateral_sigma_m = math.hypot(dl, f.lateral_fast_sigma_m)
        f.heading_sigma_deg = math.hypot(dh, f.heading_fast_sigma_deg)
        if dd is not None:
            f.distance_drift_sigma_m = dd
            f.distance_sigma_m = math.hypot(dd, f.distance_fast_sigma_m)
        else:
            # 거리 밀림 기준선이 없으면 **한 장 흔들림**(잔차 sd)을 쓴다. /√n 한 값(1 mm 급)은 죽은시간의
            # 프레임 잡음(8 mm)보다 작아 3 장 연속 '출발' 오검출 → 출발지연이 짧게 학습된다 (검토 지적)
            f.distance_sigma_m = f.distance_fast_sigma_m * math.sqrt(max(1, dist.n))
        f.ok, f.why = True, "ok"
        return f

    # ── 속 ──────────────────────────────────────────────────────────
    def _gate(self, det):
        if T.tag_pixel_size(det) < D.MIN_TAG_PX:
            return "small"
        if getattr(det, "decision_margin", 1e9) < D.MIN_DECISION_MARGIN:
            return "dim"
        return ""

    def _reject(self, why):
        self.rejected[why] = self.rejected.get(why, 0) + 1


def worth_fixing(value, sigma, tol):
    """고칠 값인가. **흔들림보다 작은 오차는 유령이다** — 쫓으면 왕복한다 (9/7 실패 원인).

    v2 는 이걸 안 봐서 7~10 m 에서 60~145 mm 잡음을 진짜 오차로 알고 좌우로 왔다갔다 했다.
    σ 를 모르면(무한대) 절대 고치지 않는다.
    """
    return abs(value) > max(tol, C.UNCERTAIN_FACTOR * sigma)


# ── 밀림 기준선 (before_run 이 쓴다) ────────────────────────────────────
class Baseline(NamedTuple):
    """drift_baseline() 의 답. Measured 에 적는 건 drift 다. 나머지는 진단."""
    drift: float          # 밀림 σ — 블록 평균의 흔들림에서 떨림 몫을 뺀 것
    block_sd: float       # 블록 평균의 sd 그대로
    resid_sd: float       # 블록 안 직선 잔차 sd = 한 장의 떨림
    n_blocks: int
    n_per_block: float    # 블록당 평균 점 수


def drift_baseline(ts, xs, window_s=WINDOW_S):
    """정지(또는 직진) 60초 기록 -> 밀림 기준선. before_run 이 좌우·방향(·거리)에 각각 돌린다.

    기록을 window_s 블록으로 잘라 블록마다 직선을 맞춘다. 블록 평균의 분산에는 떨림 몫(잔차²/n)이
    섞여 있으니 빼야 밀림만 남는다 — 안 빼면 Estimator 가 떨림을 두 번 센다.
    ts 는 우리 시계 [s]. 기준 거리(sigma_ref_distance_m)는 같은 기록의 Fix.distance_m 중앙값 —
    3D 거리(높이차 포함)다. 쓸 만한 블록이 둘 미만이면 None.
    """
    pts = sorted(zip((float(t) for t in ts), (float(x) for x in xs)))
    blocks, cur, t0 = [], [], None
    for t, x in pts:
        if t0 is None or t - t0 >= window_s:
            if cur:
                blocks.append(cur)
            cur, t0 = [], t
        cur.append((t, x))
    if cur:
        blocks.append(cur)
    fits = [line_fit(b, sum(t for t, _ in b) / len(b)) for b in blocks if len(b) >= MIN_N]
    if len(fits) < 2:
        return None
    m = sum(f.value for f in fits) / len(fits)
    block_var = sum((f.value - m) ** 2 for f in fits) / len(fits)
    jitter = sum(f.sigma_fast ** 2 for f in fits) / len(fits)           # 블록 가운데서 읽어 = 잔차²/n
    resid_var = sum(f.sigma_fast ** 2 * f.n for f in fits) / len(fits)
    return Baseline(math.sqrt(max(0.0, block_var - jitter)), math.sqrt(block_var),
                    math.sqrt(resid_var), len(fits), sum(f.n for f in fits) / len(fits))


# ── 자체 시험 ────────────────────────────────────────────────────────
# python src/models/detection/estimate.py   (카메라 없이. 9/21 기록이 옆 리포에 있으면 실자료도 돈다)
def _selftest():
    import random
    import statistics as S
    from types import SimpleNamespace

    def sd(xs):
        return S.pstdev(xs)

    def push_series(est, t0, fps, lat, head, dist, amb=None, beta=None, moving=False):
        """값 배열을 프레임처럼 밀어 넣고 매 프레임의 Fix 를 모은다."""
        out = []
        for i in range(len(lat)):
            t = t0 + i / fps
            st = {"lateral": lat[i], "heading_deg": head[i], "forward": dist[i],
                  "distance": dist[i], "beta_deg": 0.0 if beta is None else beta[i],
                  "angle_ok": not (amb and amb[i]), "edge_px": 100.0, "tilt_deg": 12.0}
            est.push(t, st)
            out.append(est.fix(now=t + 0.02, moving=moving))
        return out

    def ar1_plus_white(rng, n, sig_slow, sig_white, phi=0.998):
        """9/21 잡음 모양: 자기상관이 lag 30 까지 0.6 으로 버티는 느린 성분 + 흰 잡음."""
        s, out = 0.0, []
        k = math.sqrt(1 - phi * phi)
        for _ in range(n):
            s = phi * s + k * sig_slow * rng.gauss(0, 1)
            out.append(s + sig_white * rng.gauss(0, 1))
        return out

    failures = []

    def check(cond, msg):
        print(("  ok   " if cond else "  FAIL ") + msg)
        if not cond:
            failures.append(msg)

    # 9/21 3.68 m 정지 1459 프레임에서 뽑은 잡음 모양 (plan 문제 5-1, 이 파일 머리말)
    REF_M, LAT_SLOW, LAT_WHITE = 3.68, 0.0307, 0.0245     # 밀림 30.7 mm · 떨림 24.5 mm
    HEAD_SLOW, HEAD_WHITE = 0.456, 0.366                  # 같은 기록의 방향: 밀림 0.456도 · 잔차 0.366도
    noise = SimpleNamespace(sigma_ref_distance_m=REF_M, sigma_drift_lateral_m=LAT_SLOW,
                            sigma_drift_heading_deg=HEAD_SLOW)

    # ── 1. 합성: 정지 + 밀림 + 떨림. σ 가 밀림을 놓치지 않나 ────────────
    print("\n[1] 합성 정지 240 s @30fps — 밀림 %.1f mm + 떨림 %.1f mm (참값 0)" % (LAT_SLOW * 1e3, LAT_WHITE * 1e3))
    rng = random.Random(1)
    fps, n = 30.0, 240 * 30
    ts = [1000.0 + i / fps for i in range(n)]
    lat = ar1_plus_white(rng, n, LAT_SLOW, LAT_WHITE)
    head = ar1_plus_white(rng, n, HEAD_SLOW, HEAD_WHITE)
    dist = [REF_M + 0.003 * rng.gauss(0, 1) for _ in range(n)]
    est = Estimator(intrinsics=None, noise=noise)
    fixes = [f for f in push_series(est, ts[0], fps, lat, head, dist) if f.ok]
    err = sd([f.lateral_m for f in fixes])                       # 보고값이 참값(0)에서 실제로 얼마나 벗어나나
    tot = S.mean(f.lateral_sigma_m for f in fixes)
    fast = S.mean(f.lateral_fast_sigma_m for f in fixes)
    err_h = sd([f.heading_deg for f in fixes])
    tot_h = S.mean(f.heading_sigma_deg for f in fixes)
    fast_h = S.mean(f.heading_fast_sigma_deg for f in fixes)
    print("     한 장 sd %.1f mm | 보고값의 실제 오차 sd %.1f mm | 새 σ %.1f mm | 떨림만(옛 Track) %.1f mm"
          % (sd(lat) * 1e3, err * 1e3, tot * 1e3, fast * 1e3))
    print("     방향: 실제 오차 sd %.3f° | 새 σ %.3f° | 떨림만 %.3f°" % (err_h, tot_h, fast_h))
    check(0.8 <= tot / err <= 1.25, "좌우 σ 가 실제 오차를 맞춘다 (비 %.2f)" % (tot / err))
    check(0.8 <= tot_h / err_h <= 1.25, "방향 σ 가 실제 오차를 맞춘다 (비 %.2f)" % (tot_h / err_h))
    check(fast / err < 0.5, "떨림만 쓰면 %.1f 배 과신 — 밀림을 얹어야 한다" % (err / fast))
    check(len(fixes) == n - MIN_N + 1 and all(f.drift_from == "still" for f in fixes),
          "세 장째부터 전부 ok (%d / %d 장), 정지 기준선(drift_from=still)" % (len(fixes), n - MIN_N + 1))
    b = drift_baseline(ts, lat)
    print("     drift_baseline(before_run 식): 밀림 %.1f mm (넣은 값 %.1f) · 블록평균 sd %.1f · 잔차 %.1f mm (넣은 값 %.1f) · 블록 %d 개 x %.0f 장"
          % (b.drift * 1e3, LAT_SLOW * 1e3, b.block_sd * 1e3, b.resid_sd * 1e3, LAT_WHITE * 1e3, b.n_blocks, b.n_per_block))
    check(abs(b.drift / LAT_SLOW - 1) < 0.15 and abs(b.resid_sd / LAT_WHITE - 1) < 0.1,
          "drift_baseline 이 넣은 밀림·떨림을 되찾는다")
    check(drift_baseline(ts[:4], lat[:4]) is None, "블록이 둘이 안 되면 None")
    dl8, dh8, _, _ = est.drift.at(8.0)
    print("     밀림을 8 m 로 옮기면 좌우 %.0f mm · 방향 %.2f° (9/7 실주행 7~10 m 좌우 60~145 mm)" % (dl8 * 1e3, dh8))
    check(0.10 < dl8 < 0.20, "거리² 비례가 9/7 관측 범위와 맞는다")
    # 직진 기준선이 있으면 moving=True 때만 그걸 쓴다. 없으면 정지 것으로 (plan 3-4)
    both = SimpleNamespace(sigma_ref_distance_m=REF_M, sigma_drift_lateral_m=LAT_SLOW, sigma_drift_heading_deg=HEAD_SLOW,
                           sigma_drive_lateral_m=LAT_SLOW / 2, sigma_drive_heading_deg=HEAD_SLOW / 2)
    est2 = Estimator(intrinsics=None, noise=both)
    push_series(est2, 2000.0, fps, lat[:15], head[:15], dist[:15])
    f_still, f_drive = est2.fix(now=2000.0 + 14 / fps + 0.02), est2.fix(now=2000.0 + 14 / fps + 0.02, moving=True)
    check(f_still.drift_from == "still" and f_drive.drift_from == "drive"
          and abs(f_drive.lateral_drift_sigma_m * 2 - f_still.lateral_drift_sigma_m) < 1e-9
          and abs(f_drive.heading_drift_sigma_deg * 2 - f_still.heading_drift_sigma_deg) < 1e-9,
          "moving=True 면 직진 기준선 (좌우 %.1f -> %.1f mm)" % (f_still.lateral_drift_sigma_m * 1e3, f_drive.lateral_drift_sigma_m * 1e3))
    check(est.fix(now=ts[-1] + 0.02, moving=True).drift_from == "still", "직진 기준선이 없으면 정지 것")

    # ── 2. 실자료: 9/21 3.68 m 정지 (옆 리포에 있을 때만) ─────────────
    from pathlib import Path
    rec = None
    for d in Path(__file__).resolve().parents:
        cand = d / "apriltag_v3" / "work_dirs" / "20260921_측정기록.json"
        if cand.is_file():
            rec = cand
            break
    if rec is None:
        print("\n[2] 실자료 없음 — 건너뜀 (apriltag_v3/work_dirs/20260921_측정기록.json)")
    else:
        import json
        print("\n[2] 실자료 %s" % rec)
        run = json.loads(rec.read_text())["실행"][0]
        fr = [x for x in run["프레임"] if x.get("lateral") is not None]
        ts = [x["t_capture"] for x in fr]
        lat = [x["lateral"] for x in fr]
        head = [x["heading_deg"] for x in fr]
        dist = [x["distance"] for x in fr]
        amb = [bool(x.get("pnp2") and (x["pnp2"].get("err_ratio") or 0) > D.AMBIGUITY_RATIO) for x in fr]
        fps_real = (len(fr) - 1) / (ts[-1] - ts[0])
        b_lat, b_head, b_dist = drift_baseline(ts, lat), drift_baseline(ts, head), drift_baseline(ts, dist)
        real = SimpleNamespace(sigma_ref_distance_m=S.median(dist), sigma_drift_lateral_m=b_lat.drift,
                               sigma_drift_heading_deg=b_head.drift)
        m30 = sd([S.mean(lat[i:i + 30]) for i in range(0, len(lat) - 29, 30)])
        print("     %d 프레임 %.1f fps, 헷갈린 장 %d | 한 장 sd %.1f mm | 30장 평균 sd %.1f mm (plan: 30.9)"
              % (len(fr), fps_real, sum(amb), sd(lat) * 1e3, m30 * 1e3))
        print("     기준선(drift_baseline): 좌우 밀림 %.1f mm (블록평균 sd %.1f, 잔차 %.1f) · 방향 %.3f° (%.3f, %.3f) · 거리 %.1f mm (%.1f, %.1f) · 블록 %d 개 x %.1f 장"
              % (b_lat.drift * 1e3, b_lat.block_sd * 1e3, b_lat.resid_sd * 1e3, b_head.drift, b_head.block_sd, b_head.resid_sd,
                 b_dist.drift * 1e3, b_dist.block_sd * 1e3, b_dist.resid_sd * 1e3, b_lat.n_blocks, b_lat.n_per_block))
        formula = math.hypot(b_lat.drift, b_lat.resid_sd / math.sqrt(b_lat.n_per_block))
        est = Estimator(intrinsics=None, noise=real)
        fixes = []
        for i in range(len(fr)):
            est.push(ts[i], {"lateral": lat[i], "heading_deg": head[i], "forward": fr[i]["forward"],
                             "distance": dist[i], "beta_deg": fr[i]["beta_px_deg"],
                             "angle_ok": not amb[i], "edge_px": fr[i]["margin_px"],
                             "tilt_deg": fr[i]["tilt_deg"]})
            f = est.fix(now=ts[i] + 0.02)
            if f.ok:
                fixes.append(f)
        err = sd([f.lateral_m for f in fixes])
        tot = S.mean(f.lateral_sigma_m for f in fixes)
        fast = S.mean(f.lateral_fast_sigma_m for f in fixes)
        err_h = sd([f.heading_deg for f in fixes])
        tot_h = S.mean(f.heading_sigma_deg for f in fixes)
        print("     좌우: 보고값 실제 흔들림 %.1f mm | 새 σ %.1f mm | 식 √(밀림²+(잔차/√n)²) %.1f mm | 떨림만(옛 Track) %.1f mm (%.1f 배 과신)"
              % (err * 1e3, tot * 1e3, formula * 1e3, fast * 1e3, err / fast))
        print("     방향: 보고값 실제 흔들림 %.3f° | 새 σ %.3f° | 떨림만 %.3f°"
              % (err_h, tot_h, S.mean(f.heading_fast_sigma_deg for f in fixes)))
        check(0.8 <= tot / err <= 1.25, "실자료 좌우 σ 가 보고값의 실제 흔들림을 재현한다 (비 %.2f)" % (tot / err))
        check(0.85 <= tot / m30 <= 1.2, "새 σ %.1f mm 가 30장 평균 sd %.1f mm 를 재현한다 (비 %.2f)" % (tot * 1e3, m30 * 1e3, tot / m30))
        check(0.8 <= tot_h / err_h <= 1.25, "실자료 방향 σ 도 재현한다 (비 %.2f)" % (tot_h / err_h))
        check(fast / err < 0.5, "옛 방식은 %.1f 배 과신했다" % (err / fast))
        check(est.rejected.get("order", 0) == 0, "실자료 스탬프가 전부 단조 증가")
        # 거리: 기준선이 없으면 떨림만 -> 1초 변화도 못 덮는다. Measured 에 sigma_drift_distance_m 이 생기면 덮인다
        d_fast = S.mean(f.distance_sigma_m for f in fixes)
        d_1s = sd([dist[i + int(fps_real)] - dist[i] for i in range(len(dist) - int(fps_real))]) / math.sqrt(2)
        with_d = SimpleNamespace(sigma_ref_distance_m=S.median(dist), sigma_drift_lateral_m=b_lat.drift,
                                 sigma_drift_heading_deg=b_head.drift, sigma_drift_distance_m=b_dist.drift)
        est_d = Estimator(intrinsics=None, noise=with_d)
        d_tot = []
        for i in range(len(fr)):
            est_d.push(ts[i], {"lateral": lat[i], "heading_deg": head[i], "forward": fr[i]["forward"],
                               "distance": dist[i], "beta_deg": fr[i]["beta_px_deg"], "angle_ok": not amb[i]})
            f = est_d.fix(now=ts[i] + 0.02)
            if f.ok:
                d_tot.append(f.distance_sigma_m)
        print("     거리: 한 장 sd %.1f mm | 1초 간격 변화의 sd/√2 %.1f mm | 떨림만인 σ %.1f mm (기준선 없을 때) | 기준선 주면 %.1f mm"
              % (sd(dist) * 1e3, d_1s * 1e3, d_fast * 1e3, S.mean(d_tot) * 1e3))
        # 기준선이 없으면 한 장 떨림 급(잔차 sd — /√n 아님, 가짜 출발을 막으려고) → 1초 변화는 덮지만 밀림(기준선 주면)보다는 작다
        check(d_1s <= d_fast < S.mean(d_tot), "거리 σ: 기준선 없으면 한 장 떨림 급, 기준선 주면 더 크다 — Measured 에 sigma_drift_distance_m 이 필요하다")

    # ── 3. 헷갈린 장을 한 장씩 뺀다 ───────────────────────────────────
    print("\n[3] 12 장 중 4 장이 헷갈린 해(좌우 부호 뒤집힘)")
    rng = random.Random(2)
    n = 12
    amb = [i % 3 == 2 for i in range(n)]
    lat = [(-0.100 if amb[i] else 0.100) + 0.001 * rng.gauss(0, 1) for i in range(n)]
    head = [(-2.0 if amb[i] else 2.0) + 0.02 * rng.gauss(0, 1) for i in range(n)]
    dist = [REF_M] * n
    est = Estimator(intrinsics=None, noise=noise)
    f = push_series(est, 2000.0, 30.0, lat, head, dist, amb=amb)[-1]
    print("     Fix: %s lateral %.3f (전부 평균이면 %.3f) heading %.2f n=%d ambiguous=%d n_fit=%d"
          % (f.why, f.lateral_m, S.mean(lat), f.heading_deg, f.n, f.ambiguous, f.n_fit))
    check(f.ok and abs(f.lateral_m - 0.100) < 0.005, "헷갈린 장을 빼서 좌우가 +0.100 으로 남는다")
    check(abs(f.heading_deg - 2.0) < 0.1, "방향도 뒤집힌 장에 안 끌린다")
    check(f.n == 12 and f.ambiguous == 4 and MIN_N <= f.n_fit <= 8,
          "n/ambiguous/n_fit 집계 (n_fit 은 튀는 점 규칙이 더 뺄 수 있다)")
    est.reset()
    f = push_series(est, 3000.0, 30.0, lat, head, dist, amb=[True] * n)[-1]
    print("     전부 헷갈리면: %s ambiguous=%d distance %.2f beta 있음=%s" % (f.why, f.ambiguous, f.distance_m, f.beta_sigma_deg < 1))
    check(f.why == "few" and not f.ok and f.ambiguous == n, "깨끗한 장 < MIN_N → few")
    check(abs(f.distance_m - REF_M) < 1e-6 and math.isinf(f.lateral_sigma_m), "거리·화면위치는 남고 좌우 σ 는 무한대")
    check(not worth_fixing(0.5, f.lateral_sigma_m, C.SIDE_GAP_M), "σ 무한대면 절대 안 고친다")
    est.reset()
    check(est.fix(now=3001.0).why == "no_tag" and est.fix(now=3001.0).n == 0, "reset 뒤엔 no_tag")
    est.push(3001.0, {"lateral": float("nan"), "heading_deg": 2.0, "distance": REF_M, "beta_deg": 0.0})
    check(len(est.tracks["lateral"].pts) == 0 and len(est.tracks["heading_deg"].pts) == 1, "NaN 은 트랙에 안 들어간다")

    # ── 4. 실제 경로: 가짜 검출 → PnP → observe → fix (게이트·시계·기준선 없음) ──
    print("\n[4] 가짜 검출로 observe 끝까지 (3.68 m, 카메라가 태그보다 0.81 m 아래, 모서리 잡음 0.20 px)")
    import numpy as np
    import cv2
    from src.models.detection.image import CameraIntrinsics
    w, h, fx, fy, cx, cy = D.D435I_COLOR_REF
    intr = CameraIntrinsics(fx, fy, cx, cy, w, h)

    def fake_detection(lat_m, vert_m, fwd_m, head_deg, px_noise, rng):
        """카메라를 태그 좌표계에 놓고 네 모서리를 투영한다. 모서리 순서는 pose._object_points
        (= tag.detect 가 corners[::-1] 한 뒤의 순서). 카메라 축 = diag(-1,-1,1)·Ry(방향): 태그 x 가
        화면 왼쪽, y 가 위, z 가 벽 안쪽 — 9/21 실프레임(vertical -0.8, 태그가 화면 위쪽)과 같은 배치."""
        a = math.radians(head_deg)
        R = np.diag([-1.0, -1.0, 1.0]) @ np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0],
                                                    [-math.sin(a), 0, math.cos(a)]])
        T_tc = np.eye(4)
        T_tc[:3, :3], T_tc[:3, 3] = R, [lat_m, vert_m, -fwd_m]
        T_ct = P.invert_T(T_tc)
        rvec, _ = cv2.Rodrigues(T_ct[:3, :3])
        img, _ = cv2.projectPoints(P._object_points(D.TAG_SIZE_M), rvec, T_ct[:3, 3], intr.K, np.zeros(5))
        img = img.reshape(-1, 2) + px_noise * np.array([[rng.gauss(0, 1) for _ in range(2)] for _ in range(4)])
        det = SimpleNamespace(corners=img, center=img.mean(axis=0), decision_margin=50.0, tag_id=D.TAG_ID, hamming=0)
        return det, P.state(T_ct)

    rng = random.Random(3)
    est = Estimator(intr, noise=noise)
    t0, truth, n_amb = 5000.0, None, 0
    n_seen = 0
    for i in range(15):
        det, truth = fake_detection(0.15, -0.81, 3.68, 2.0, 0.20, rng)
        st = est.observe(t0 + i / 30.0, det, (h, w))
        n_seen += int(st is not None)
        n_amb += int(st is not None and not st["angle_ok"])
    check(n_seen == 15, "15 장 전부 게이트·재투영 통과 (거부: %s)" % est.rejected)
    t_last = t0 + 14 / 30.0
    f = est.fix(now=t_last + 0.02)
    print("     참값 lateral %.3f heading %.2f | Fix %s lateral %.3f±%.3f heading %.2f±%.2f dist %.3f±%.4f beta %.2f edge %.0f px n=%d amb=%d"
          % (truth["lateral"], truth["heading_deg"], f.why, f.lateral_m, f.lateral_sigma_m, f.heading_deg,
             f.heading_sigma_deg, f.distance_m, f.distance_sigma_m, f.beta_deg, f.edge_px, f.n, f.ambiguous))
    check(f.ok and n_amb == 0, "올려다보는 배치라 두 해가 안 헷갈린다")
    check(abs(f.lateral_m - truth["lateral"]) < 3 * f.lateral_sigma_m, "좌우가 3σ 안")
    from src import limits
    live = limits.tag_cut_live_m(intr, -f.vertical_m, f.forward_m, f.top_px)
    fixed = limits.tag_cut_m(intr, height_diff_m=0.81)
    print("     vertical %.3f (참 −0.81) · 윗변 행 %.1f px · 태그컷 실시간 %.3f m / 고정식 %.3f m" % (f.vertical_m, f.top_px, live, fixed))
    check(abs(f.vertical_m - truth["vertical"]) < 0.05 and math.isfinite(f.top_px),
          "vertical·윗변 행이 observe 를 거쳐 나온다 (vertical 오차 %.0f mm — 올려다보는 배치라 좌우만큼 흔들린다)"
          % (abs(f.vertical_m - truth["vertical"]) * 1e3))
    check(abs(live - fixed) < 0.03, "피치 없는 카메라면 실시간 태그컷 = 고정식 (모서리 잡음만큼 차이)")
    check(abs(f.heading_deg - truth["heading_deg"]) < 3 * f.heading_sigma_deg, "방향이 3σ 안")
    # distance 는 3D 거리(높이차 포함)라 3.77 m — before_run 도 같은 정의(Fix.distance_m)로 기준 거리를 적어야 한다
    check(abs(f.lateral_drift_sigma_m - LAT_SLOW * (f.distance_m / REF_M) ** 2) < 1e-6,
          "밀림이 (지금 3D 거리 / 기준 거리)² 로 옮겨진다")
    check(est.fix(now=t_last - 0.01).why == "clock", "나이가 음수면 clock")
    check(est.fix(now=t_last + clock.MAX_AGE_S - 0.01).ok, "MAX_AGE_S 안이면 ok")
    check(est.fix(now=t_last + clock.MAX_AGE_S + 0.01).why == "stale", "MAX_AGE_S(%.2f s) 넘으면 stale" % clock.MAX_AGE_S)
    bare = Estimator(intr, noise=None)
    for i in range(5):
        det, _ = fake_detection(0.15, -0.81, 3.68, 2.0, 0.20, rng)
        bare.observe(t0 + i / 30.0, det, (h, w))
    f = bare.fix(now=t0 + 4 / 30.0 + 0.02)
    check(f.why == "no_sigma" and not f.ok and math.isinf(f.lateral_sigma_m) and abs(f.lateral_m - 0.15) < 0.05,
          "기준선 없으면 no_sigma (값은 채우되 σ 무한대)")
    det, _ = fake_detection(0.15, -0.81, 3.68, 2.0, 0.20, rng)
    det.corners = det.center + (det.corners - det.center) * (15.0 / T.tag_pixel_size(det))   # 15 px 짜리
    check(est.observe(t0 + 1.0, det, (h, w)) is None and est.rejected.get("small") == 1, "작은 태그는 게이트에서 막힌다")
    det, _ = fake_detection(0.15, -0.81, 3.68, 2.0, 0.20, rng)
    check(est.observe(t0 + 0.1, det, (h, w)) is None and est.rejected.get("order") == 1, "시각이 안 늘면 안 넣는다")

    print("\n%s" % ("전부 통과" if not failures else "실패 %d: %s" % (len(failures), failures)))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
