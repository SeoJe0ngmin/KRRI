"""마지막 직진 구간의 조준 — PnP 의 방향·좌우 없이 **화면위치 β 와 거리**만으로 "가는 직선이 목표를 얼마나 빗나가나".

2026-10-02 실차: PnP 방향은 자세마다 ±3~4도(좌우 ±0.2~0.3 m) 치우쳤고 정지 σ 에는 안 보였다. 믿을 수 있는 건 β(0.004도)·거리·자이로.

차가 **곧장** 가면 태그는 차 좌표계에서 뒤로만 온다 — 카메라 좌표계(앞 X · 오른쪽 Yr)에서 한 직선 위를 움직인다:
    X = ρ·cos β ,  Yr = ρ·sin β                       ρ 수평거리 · β 화면위치(+오른쪽)
    Yr = tan(c)·X + b0                                 c = 카메라 축이 **실제 진행 방향**에서 틀어진 각(+반시계). 쏠림까지 들어간다
그 직선이 카메라(원점)를 얼마나 비켜 가나가 곧 빗나감이다:  태그가 카메라 진행선의 왼쪽으로 y = −b0·cos c = ρ·sin(c − β).

    fit_leg()   직진 다리 하나의 (ρ, β) 들에 직선을 맞춰 c 를 낸다. ρ 가 변해야 풀린다 — 서 있는 프레임만으론 안 나온다.
                다리 중 자이로가 돈 만큼은 β 에서 뺀다(차가 살짝 틀어지면 태그가 화면에서 그만큼 밀린다).
    Aim         다리마다 나온 c 를 모은다(가중 평균). c 는 카메라 장착(+쏠림)이라 다리가 바뀌어도 같은 값이어야 한다.
    Aim.miss()  지금 한 장의 (ρ, β) 와 c 로 빗나감을 낸다 — 서서 판단할 때 쓴다.
                차 맨 앞 가운데가 목표에서 벗어나는 양 = y + CAM_LATERAL_OFFSET_M(카메라가 차 중심선에서 왼쪽으로) + TAG_LATERAL_OFFSET_M.
                + 면 목표가 진행선 **왼쪽** — 왼쪽(반시계, +)으로 돌아야 한다.
"""
if __name__ == "__main__":      # 스크립트로 돌릴 때만 경로를 잡는다. bootstrap.repo_root 와 같은 규칙
    import sys
    from pathlib import Path
    sys.path.insert(0, str(next(d for d in Path(__file__).resolve().parents
                               if (d / "config").is_dir() and (d / "src").is_dir())))

import math

from config import control as C

MIN_N = 3              # 직선을 맞추려면 최소 세 점 (estimate.MIN_N 과 같은 이유)


def _finite(*xs):
    return all(x is not None and math.isfinite(x) for x in xs)


def fit_leg(rows, height_diff_m):
    """직진 다리의 프레임들 → {"c_deg", "c_sigma_deg", "y_m", "n", "span_m", "resid_mm"}. 못 풀면 None.

    rows: dict 들 — distance(3D) · beta_deg · gyro_deg(없으면 보정 없이). height_diff_m 로 수평거리를 낸다.
    y_m 은 이 다리에서 태그가 카메라 진행선 왼쪽으로 떨어진 거리(진단). σ 는 직선 잔차에서 — 차가 휘어 간 몫은 안 들어 있다.
    """
    pts, g0 = [], None
    for r in rows:
        d, b, g = r.get("distance"), r.get("beta_deg"), r.get("gyro_deg")
        if not _finite(d, b):
            continue
        if _finite(g) and g0 is None:
            g0 = g
        yaw = (g - g0) if (_finite(g) and g0 is not None) else 0.0
        rho = math.sqrt(max(0.0, d * d - height_diff_m * height_diff_m))
        bc = math.radians(b - yaw)                       # 차가 반시계로 yaw 만큼 틀어지면 태그는 화면 오른쪽으로 그만큼 간다
        pts.append((rho * math.cos(bc), rho * math.sin(bc)))
    n = len(pts)
    if n < MIN_N:
        return None
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    if sxx <= 0:
        return None
    a = sum((p[0] - mx) * (p[1] - my) for p in pts) / sxx
    b0 = my - a * mx
    resid2 = sum((p[1] - (a * p[0] + b0)) ** 2 for p in pts)
    s = math.sqrt(resid2 / (n - 2)) if n > 2 else 0.0
    c = math.atan(a)
    sig_c = (s / math.sqrt(sxx)) * math.cos(c) ** 2      # d(atan a) = cos²c · da
    xs = [p[0] for p in pts]
    return {"c_deg": math.degrees(c), "c_sigma_deg": math.degrees(sig_c), "y_m": -b0 * math.cos(c),
            "n": n, "span_m": max(xs) - min(xs), "resid_mm": s * 1e3}


class Aim:
    """직진 다리들에서 조준각 c 를 모으고, 지금 프레임의 빗나감을 낸다. 제어 스레드에서만 쓴다."""

    def __init__(self):
        self.fits = []

    def add_leg(self, rows, height_diff_m):
        f = fit_leg(rows, height_diff_m)
        if f is not None and f["c_sigma_deg"] > 0:       # σ 0 = 잔차 0 (점 셋 미만과 같다) — 무게를 못 준다
            self.fits.append(f)
        return f

    @property
    def known(self):
        return bool(self.fits)

    def c(self):
        """(c_deg, σ_deg). 다리별 σ 로 가중 평균. σ 는 '다리끼리 흩어진 정도(가중 sd)' 와 식의 σ 중 큰 쪽 —
        직선 잔차는 차가 휘어 간 몫·화면 가장자리의 치우침을 모르고, 그런 건 다리를 늘려도 √n 으로 줄지 않는다.
        2026-10-02 실주행 6 다리: +0.85 · +0.66 · +0.42 · +0.15 · +0.56 · +1.13 → +0.79 ±0.16도 (3 m 에서 0.9 cm)."""
        if not self.fits:
            return None, None
        w = [1.0 / f["c_sigma_deg"] ** 2 for f in self.fits]
        c = sum(wi * f["c_deg"] for wi, f in zip(w, self.fits)) / sum(w)
        sig = math.sqrt(1.0 / sum(w))
        if len(self.fits) > 1:
            sig = max(sig, math.sqrt(sum(wi * (f["c_deg"] - c) ** 2 for wi, f in zip(w, self.fits)) / sum(w)))
        return c, sig

    def miss(self, fix, height_diff_m):
        """(빗나감 [m], σ [m]). + = 목표가 진행선 왼쪽. c 를 모르면 (None, None)."""
        c, sig_c = self.c()
        d, b = getattr(fix, "distance_m", None), getattr(fix, "beta_deg", None)
        if c is None or not _finite(d, b) or d <= 0:
            return None, None
        rho = math.sqrt(max(0.0, d * d - height_diff_m * height_diff_m))
        y = rho * math.sin(math.radians(c - b))
        sig_b = getattr(fix, "beta_sigma_deg", 0.0)
        sig_b = sig_b if _finite(sig_b) else 0.0
        sig = rho * math.radians(math.hypot(sig_c, sig_b))
        return y + C.CAM_LATERAL_OFFSET_M + C.TAG_LATERAL_OFFSET_M, sig


def turn_for(miss_m, forward_m, center_m):
    """빗나감을 0 으로 만드는 제자리 회전각 [도]. 돌면 진행선이 회전중심 둘레로 돌아 목표 자리에서 (앞거리 + 중심)·sin Δ 만큼 옮겨진다.
    center_m = CAM_TO_ROT_CENTER_M (음수 = 카메라 앞). + 면 반시계."""
    lever = max(1e-3, forward_m + center_m)
    return math.degrees(math.asin(max(-1.0, min(1.0, miss_m / lever))))


# ── 자체 시험 ────────────────────────────────────────────────────────
# python src/models/planning/aim.py
if __name__ == "__main__":
    import random
    from types import SimpleNamespace

    def leg(c_deg, y_m, x0, x1, n, rng, dz=0.96, sb=0.02, sd=0.01, yaw_drift=0.0):
        """카메라가 차 진행 방향에서 c 만큼 틀어져 달리고, 태그가 진행선 왼쪽 y 에 있을 때 (x0 → x1 직진) 프레임들."""
        out = []
        for i in range(n):
            x = x0 + (x1 - x0) * i / (n - 1)                     # 차 좌표계에서 태그까지 앞거리
            yaw = yaw_drift * i / (n - 1)                        # 다리 중 차가 반시계로 틀어진 양 [도]
            phi = math.degrees(math.atan2(y_m, x)) - yaw         # 진행 방향 기준 태그 방향 (반시계 +)
            beta = c_deg - phi                                   # 카메라 축 기준, 오른쪽 +
            rho = math.hypot(x, y_m)
            out.append({"distance": math.sqrt(rho * rho + dz * dz) + rng.gauss(0, sd), "beta_deg": beta + rng.gauss(0, sb),
                        "gyro_deg": 10.0 + yaw})
        return out

    rng = random.Random(4)
    # 1) 한 다리: c = +1.5도, 태그가 진행선 왼쪽 8 cm, 4.6 → 3.1 m, 프레임 30장 (β 잡음 0.02도 · 거리 잡음 1 cm)
    f = fit_leg(leg(1.5, 0.08, 4.6, 3.1, 30, rng), 0.96)
    print("한 다리: c %.3f ±%.3f도 (참 1.5) · y %+.1f mm (참 +80) · 잔차 %.1f mm · 폭 %.2f m"
          % (f["c_deg"], f["c_sigma_deg"], f["y_m"] * 1e3, f["resid_mm"], f["span_m"]))
    assert abs(f["c_deg"] - 1.5) < 4 * f["c_sigma_deg"] + 0.02 and abs(f["y_m"] - 0.08) < 0.01
    assert fit_leg(leg(1.5, 0.08, 4.0, 4.0, 2, rng), 0.96) is None                       # 점이 모자라면 못 푼다
    # 2) 차가 다리 중 0.5도 틀어졌는데 자이로로 빼 주면 같은 답 (안 빼면 y 가 수 cm 틀린다)
    drift = leg(1.5, 0.08, 4.6, 3.1, 30, rng, yaw_drift=0.5)
    f2 = fit_leg(drift, 0.96)
    raw = fit_leg([dict(r, gyro_deg=None) for r in drift], 0.96)
    print("휘어 간 다리: 자이로 보정 y %+.1f mm · 보정 없으면 %+.1f mm" % (f2["y_m"] * 1e3, raw["y_m"] * 1e3))
    assert abs(f2["y_m"] - 0.08) < 0.01 and abs(raw["y_m"] - 0.08) > 0.02
    # 3) 여러 다리 → c 가중 평균, 그 c 로 한 장에서 빗나감
    aim = Aim()
    assert aim.miss(SimpleNamespace(distance_m=3.3, beta_deg=0.0, beta_sigma_deg=0.01), 0.96) == (None, None)
    for x0, x1, y in ((8.0, 6.5, -0.9), (6.5, 5.0, -0.4), (4.6, 3.85, 0.05)):            # 다리마다 y 는 달라도 c 는 같다
        aim.add_leg(leg(1.5, y, x0, x1, 25, rng), 0.96)
    c, sc = aim.c()
    print("세 다리: c %.3f ±%.3f도" % (c, sc))
    assert abs(c - 1.5) < 4 * sc + 0.02
    x, y = 3.2, 0.04                                                                     # 지금: 태그 앞 3.2 m, 왼쪽 4 cm
    rho = math.hypot(x, y)
    fix = SimpleNamespace(distance_m=math.hypot(rho, 0.96), beta_deg=1.5 - math.degrees(math.atan2(y, x)), beta_sigma_deg=0.01)
    m, s = aim.miss(fix, 0.96)
    print("빗나감 %+.1f ±%.1f mm (참 +40) → 필요 회전 %+.2f도 (회전중심 앞 0.465 m)" % (m * 1e3, s * 1e3, turn_for(m, x, -0.465)))
    assert abs(m - 0.04 - C.CAM_LATERAL_OFFSET_M - C.TAG_LATERAL_OFFSET_M) < 3 * s + 0.003 and 0 < s < 0.02
    assert abs(turn_for(0.04, 3.2, -0.465) - math.degrees(math.asin(0.04 / (3.2 - 0.465)))) < 1e-9 and turn_for(-0.04, 3.2, -0.465) < 0
    print("aim 자체 시험 통과")
