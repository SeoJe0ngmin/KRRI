"""before_run 기록에서 측정값을 **다시** 낸다 — 판정을 기록 시점에 박지 않기 위해 (plan 6-3 ⑤).

    python tools/analyze_before_run.py                              # 최신 폴더
    python tools/analyze_before_run.py work_dirs/before_run/<시각>   # 그 폴더
    python tools/analyze_before_run.py --selftest                   # 하드웨어 없이 계산식만

before_run.py 는 현장에서 **여기 있는 함수**로 값을 내어 measured.json 에 적는다. 이 도구는 같은 폴더의
event.jsonl · frame.jsonl 에서 같은 함수로 다시 내어 나란히 보인다. 둘이 다르면 기록이 샜거나 함수가 바뀐 것이다.

    σ 밀림      estimate.drift_baseline — 추정기가 쓰는 식 그대로 (결정 8)
    회전중심    두 가지로 교차검증 (plan 4-7)
                  위치+방향   p_i + M(ψ_i)·o = c 를 최소제곱으로. 연속 스텝 쌍의 독립 추정으로 흩어짐도 본다
                  위치만      동심원 적합 → RMS. 광운대 rotation_fit/icr_fit.py 의 생각(반지름이 일정한 중심)만 빌렸다
    쏠림        (좌우 변화 ÷ 간 거리) 의 각 − 그동안의 카메라 방향. 자이로가 돈 시행은 뺀다 (plan 5-4)
    속도        (시각, 거리) 트랙에서 출발 시점(연속 ONSET_N 장이 ONSET_K·σ 를 넘는 첫 장)과 그 뒤 기울기
    태그컷      (거리, 가장자리 px) 트랙에서 가장자리가 TAG_EDGE_MARGIN_PX 를 지나는 거리를 보간

좌표 약속은 pose.state · sidestep.py 와 같다: 좌우 +왼쪽, 방향 +반시계, 세계 x = −forward(태그 쪽), y = lateral.
"""
import argparse
import json
import math
import statistics as S
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))       # tools/ 의 부모. bootstrap 이 다시 확인한다
from src.bootstrap import setup, work_root                          # noqa: E402

setup()

import numpy as np                                                  # noqa: E402

from config import detection as D                                   # noqa: E402
from src.models.control.forward import ONSET_K, ONSET_N             # noqa: E402  출발 판정은 forward 와 같은 규칙
from src.models.control.sidestep import normalize_deg               # noqa: E402
from src.models.detection.estimate import MIN_N, WINDOW_S, drift_baseline   # noqa: E402

#: 회전중심 적합에 최소로 필요한 정지점 수. 미지수 4 개(중심 오프셋 2 + 세계 중심 2)에 점당 식 2 개
FIT_MIN_STOPS = 4
#: 연속 쌍 독립 추정에서 이보다 적게 돈 쌍은 뺀다 [도] — (M_i − M_j) 가 특이해진다
PAIR_MIN_DEG = 1.0


# ── 작은 도구 ─────────────────────────────────────────────────────────
def _intr6(intr):
    """dict(measured.intrinsics) · CameraIntrinsics · (w,h,fx,fy,cx,cy) → (w, h, fx, fy, cx, cy). limits._intr 과 같은 규칙."""
    if intr is None or (isinstance(intr, (dict, tuple, list)) and not intr):
        return D.D435I_COLOR_REF
    if isinstance(intr, dict):
        g = intr.get
        w, h = g("w", g("width")), g("h", g("height"))
        fx, fy, cx, cy = g("fx"), g("fy"), g("cx"), g("cy")
    elif hasattr(intr, "fx"):
        fx, fy, cx, cy = intr.fx, intr.fy, intr.cx, intr.cy
        w = getattr(intr, "width", None) or getattr(intr, "w", None)
        h = getattr(intr, "height", None) or getattr(intr, "h", None)
    else:
        w, h, fx, fy, cx, cy = intr
    w = int(round(cx * 2)) if not w else int(w)
    h = int(round(cy * 2)) if not h else int(h)
    return w, h, float(fx), float(fy), float(cx), float(cy)


def _line(ts, xs):
    """최소제곱 직선. (기울기, 절편, 잔차 리스트). 점이 둘 미만이면 기울기 0."""
    n = len(ts)
    if n < 2:
        return 0.0, (xs[0] if xs else 0.0), [0.0] * n
    mt, mx = sum(ts) / n, sum(xs) / n
    sxx = sum((t - mt) ** 2 for t in ts)
    slope = (sum((t - mt) * (x - mx) for t, x in zip(ts, xs)) / sxx) if sxx > 0 else 0.0
    icpt = mx - slope * mt
    return slope, icpt, [x - (icpt + slope * t) for t, x in zip(ts, xs)]


def _rot2(psi):
    """세계 ← 카메라 2x2. 열 = (앞 방향, 왼쪽 방향)."""
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s], [s, c]])


def _med(xs):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    return S.median(xs) if xs else None


def _sd(xs):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    return S.pstdev(xs) if len(xs) > 1 else None


# ── σ 기준선 ─────────────────────────────────────────────────────────
def sigma_of(ts, xs, window_s=WINDOW_S):
    """drift_baseline 을 JSON 으로 적을 수 있는 dict 로. 못 내면 None."""
    b = drift_baseline(ts, xs, window_s)
    if b is None:
        return None
    return {"drift": b.drift, "block_sd": b.block_sd, "resid_sd": b.resid_sd,
            "n_blocks": b.n_blocks, "n_per_block": b.n_per_block,
            "frame_sd": _sd(xs), "n": len(xs)}


def still_sigma(rows):
    """정지 기록(프레임 dict 리스트) → 좌우·방향 밀림 + 기준 거리. estimate 의 Drift 가 읽는 이름으로."""
    clean = [r for r in rows if r.get("angle_ok") and r.get("lateral") is not None
             and r.get("heading_deg") is not None]
    if len(clean) < 2 * MIN_N:
        return None
    ts = [r["t"] for r in clean]
    lat = sigma_of(ts, [r["lateral"] for r in clean])
    head = sigma_of(ts, [r["heading_deg"] for r in clean])
    dist_rows = [r for r in rows if r.get("distance") is not None]
    dist = sigma_of([r["t"] for r in dist_rows], [r["distance"] for r in dist_rows]) if dist_rows else None
    if lat is None or head is None:
        return None
    return {"sigma_ref_distance_m": _med([r["distance"] for r in dist_rows]),   # 3D 거리 — Fix.distance_m 과 같은 정의
            "sigma_drift_lateral_m": lat["drift"], "sigma_drift_heading_deg": head["drift"],
            "lateral": lat, "heading": head, "distance": dist,
            "n": len(rows), "n_clean": len(clean), "ambiguous_rate": 1.0 - len(clean) / len(rows)}


def drive_sigma(rows, ref_m=None):
    """직진 한 다리의 프레임 → 같은 계산. 진짜 움직임(직선 추세)은 빼고, 밀림은 기준 거리로 옮긴다.

    좌우 ∝ 거리², 방향 ∝ 거리 (estimate.Drift.at 과 같은 비례). ref_m 이 없으면 이 다리의 중앙 거리 그대로.
    """
    clean = [r for r in rows if r.get("angle_ok") and r.get("lateral") is not None
             and r.get("heading_deg") is not None]
    if len(clean) < 2 * MIN_N:
        return None
    ts = [r["t"] for r in clean]
    _, _, rl = _line(ts, [r["lateral"] for r in clean])
    _, _, rh = _line(ts, [r["heading_deg"] for r in clean])
    lat, head = drift_baseline(ts, rl), drift_baseline(ts, rh)
    if lat is None or head is None:
        return None
    d = _med([r["distance"] for r in clean if r.get("distance") is not None]) or 0.0
    s = (ref_m / d) if (ref_m and d > 0) else 1.0
    return {"lateral_drift_m": lat.drift * s * s, "heading_drift_deg": head.drift * s,
            "lateral_block_sd_m": lat.block_sd, "heading_block_sd_deg": head.block_sd,
            "lateral_resid_sd_m": lat.resid_sd, "heading_resid_sd_deg": head.resid_sd,
            "distance_med_m": d, "scale": s, "n": len(clean), "n_blocks": lat.n_blocks}


# ── 회전중심 (plan 4-7) ───────────────────────────────────────────────
def circle_fit(xs, ys):
    """동심원 적합. (cx, cy, r, rms). 대수 해(Kåsa)로 시작해 기하 잔차로 다듬는다."""
    x, y = np.asarray(xs, float), np.asarray(ys, float)
    a, b, c = np.linalg.lstsq(np.c_[x, y, np.ones_like(x)], -(x * x + y * y), rcond=None)[0]
    cx, cy = -a / 2.0, -b / 2.0
    r = math.sqrt(max(0.0, cx * cx + cy * cy - c))
    for _ in range(30):
        dx, dy = x - cx, y - cy
        d = np.hypot(dx, dy)
        d[d == 0] = 1e-12
        step = np.linalg.lstsq(np.c_[-dx / d, -dy / d, -np.ones_like(d)], -(d - r), rcond=None)[0]
        cx, cy, r = cx + step[0], cy + step[1], r + step[2]
        if np.abs(step).max() < 1e-9:
            break
    rms = float(np.sqrt(np.mean((np.hypot(x - cx, y - cy) - r) ** 2)))
    return float(cx), float(cy), float(r), rms


def gain_of(stops):
    """태그가 화면에서 몇 배로 움직이나 = dβ/dψ. ψ 는 자이로 상대각(psi_rel_deg), 없으면 카메라 방향."""
    pts = [(s.get("psi_rel_deg", s.get("heading_deg")), s.get("beta_deg")) for s in stops]
    pts = [(p, b) for p, b in pts if p is not None and b is not None]
    if len(pts) < 2:
        return None
    slope, _, _ = _line([p for p, _ in pts], [b for _, b in pts])
    return slope


def _ls_center(pts):
    """위치+방향 최소제곱. 돌려주는 것: (o_fwd, o_lat, cx, cy, 점당 잔차 RMS[m])."""
    n = len(pts)
    P = np.array([[-s["forward"], s["lateral"]] for s in pts])
    psi = np.radians([s["heading_deg"] for s in pts])
    A = np.zeros((2 * n, 4))
    b = np.zeros(2 * n)
    for i, p in enumerate(psi):
        c, s_ = math.cos(p), math.sin(p)
        A[2 * i] = (c, -s_, -1.0, 0.0)
        A[2 * i + 1] = (s_, c, 0.0, -1.0)
        b[2 * i:2 * i + 2] = -P[i]
    sol = np.linalg.lstsq(A, b, rcond=None)[0]
    res = (A @ sol - b).reshape(-1, 2)
    return float(sol[0]), float(sol[1]), float(sol[2]), float(sol[3]), float(np.sqrt((res ** 2).sum(axis=1).mean()))


def rot_center_fit(stops):
    """정지점들 → 회전중심. cam_to_rot_center_m 은 Measured 규약(음수 = 카메라 앞).

    stops 의 각 항: lateral · forward · heading_deg (카메라) · beta_deg · dir(그 점에 이른 스텝 방향 ±1) · psi_rel_deg.
    명령한 각도와 자이로는 계산에 안 들어간다 (plan 4-7) — gain(진단)에만 자이로 상대각을 쓴다.
    """
    pts = [s for s in stops if all(s.get(k) is not None for k in ("lateral", "forward", "heading_deg"))]
    n = len(pts)
    out = {"n": n}
    if n < FIT_MIN_STOPS:
        out["why"] = "few"
        return out
    o_f, o_l, cx, cy, ls_rms = _ls_center(pts)
    P = np.array([[-s["forward"], s["lateral"]] for s in pts])
    ccx, ccy, r, crms = circle_fit(P[:, 0], P[:, 1])
    # 연속 쌍 독립 추정 — 스텝마다 따로 나오니 흩어짐이 곧 신뢰도다
    psi = np.radians([s["heading_deg"] for s in pts])
    pairs = []
    for i in range(n - 1):
        if abs(psi[i + 1] - psi[i]) < math.radians(PAIR_MIN_DEG):
            continue
        try:
            o = np.linalg.solve(_rot2(psi[i]) - _rot2(psi[i + 1]), P[i + 1] - P[i])
        except np.linalg.LinAlgError:
            continue
        pairs.append((float(o[0]), float(o[1])))
    # 좌·우 스텝을 따로 (광운대 icr_fit 의 mirror 검사와 같은 뜻)
    sides = {}
    for d, name in ((+1, "L"), (-1, "R")):
        sub = [s for s in pts if s.get("dir") == d]
        if len(sub) >= FIT_MIN_STOPS:
            sf, sl, _, _, srms = _ls_center(sub)
            sides[name] = {"cam_to_rot_center_m": -sf, "rot_center_lateral_m": sl, "rms_mm": srms * 1e3, "n": len(sub)}
    out.update({
        "cam_to_rot_center_m": -o_f, "rot_center_lateral_m": o_l, "radius_m": math.hypot(o_f, o_l),
        "ls_rms_mm": ls_rms * 1e3, "world_center": (cx, cy),
        "circle_center": (ccx, ccy), "circle_radius_m": r, "circle_rms_mm": crms * 1e3,
        "pairs_n": len(pairs),
        "pairs_cam_to_rot_center_m": (-_med([p[0] for p in pairs])) if pairs else None,
        "pairs_spread_m": _sd([p[0] for p in pairs]) if len(pairs) > 1 else None,
        "gain": gain_of(pts), "sides": sides,
        "swing_deg": (max(s["heading_deg"] for s in pts) - min(s["heading_deg"] for s in pts)),
    })
    return out


# ── 쏠림 (plan 5-4) ───────────────────────────────────────────────────
def veer_from_trials(trials, gyro_tol_deg):
    """시행: lat0 fwd0 lat1 fwd1 heading_deg(달리는 동안 카메라 방향 중앙값) gyro_delta_deg legs_ok."""
    rows, used = [], []
    for t in trials:
        if any(t.get(k) is None for k in ("lat0", "fwd0", "lat1", "fwd1", "heading_deg")):
            rows.append({"travel_deg": None, "veer_deg": None, "run_m": None, "used": False,
                         "gyro_delta_deg": t.get("gyro_delta_deg")})
            continue
        run = t["fwd0"] - t["fwd1"]
        travel = math.degrees(math.atan2(t["lat1"] - t["lat0"], run)) if run > 0 else None
        veer = None if travel is None else normalize_deg(travel - t["heading_deg"])
        ok = (veer is not None and bool(t.get("legs_ok", True))
              and abs(t.get("gyro_delta_deg") or 0.0) <= gyro_tol_deg)
        rows.append({"travel_deg": travel, "veer_deg": veer, "run_m": run, "used": ok,
                     "gyro_delta_deg": t.get("gyro_delta_deg")})
        if ok:
            used.append(veer)
    return {"veer_deg": _med(used), "spread_deg": _sd(used), "n_used": len(used), "n": len(trials), "rows": rows}


# ── 회전 하한 (plan 4-5) ──────────────────────────────────────────────
def rot_floor(turned_deg):
    """움직이자마자 끊은 회전들의 |각| → 최대가 하한. 평균이 아니라 최대 — 지나치는 쪽이 비싸다."""
    xs = [abs(x) for x in turned_deg if x is not None]
    return {"rot_floor_deg": max(xs) if xs else None, "median_deg": _med(xs), "sd_deg": _sd(xs), "n": len(xs)}


# ── 속도 (시간 유지 명령 + 카메라) ────────────────────────────────────
def speed_from_track(track, sigma_m, t_stop_rel, base_s=WINDOW_S):
    """track = [(t_rel, d, ...)]. 출발 = 연속 ONSET_N 장이 출발 전 값에서 ONSET_K·σ 를 넘는 첫 장. 속도 = 그 뒤 기울기.

    σ 는 준 값과 **처음 base_s 동안의 프레임 sd** 중 큰 쪽 — 출발지연(≥1 s)이 있으니 그 구간은 정지다.
    """
    pts = [(float(p[0]), float(p[1])) for p in track if p[1] is not None]
    if len(pts) < ONSET_N + MIN_N:
        return {"n": len(pts), "why": "few", "speed_mps": None, "onset_s": None}
    base = [d for t, d in pts if t <= pts[0][0] + base_s]
    d0 = S.median(base) if base else pts[0][1]
    thr = ONSET_K * max(float(sigma_m or 0.0), _sd(base) or 0.0, 1e-4)
    onset, first, run = None, None, 0
    for t, d in pts:
        if abs(d - d0) > thr:
            first = t if first is None else first
            run += 1
            if run >= ONSET_N:
                onset = first
                break
        else:
            first, run = None, 0
    if onset is None:
        return {"n": len(pts), "why": "no_onset", "speed_mps": None, "onset_s": None,
                "moved_m": abs(pts[-1][1] - d0)}
    fit = [(t, d) for t, d in pts if onset <= t <= t_stop_rel]
    if len(fit) < MIN_N:
        return {"n": len(pts), "why": "few_moving", "speed_mps": None, "onset_s": onset}
    slope, _, resid = _line([t for t, _ in fit], [d for _, d in fit])
    return {"onset_s": onset, "speed_mps": abs(slope), "n_fit": len(fit), "n": len(pts),
            "moved_m": abs(pts[-1][1] - d0), "resid_sd_m": _sd(resid), "d_start": d0, "d_end": pts[-1][1]}


# ── 태그컷 (plan 문제 4) ──────────────────────────────────────────────
def tag_cut_from_track(track, margin_px=D.TAG_EDGE_MARGIN_PX):
    """track = [(t_rel, forward_m, edge_px)]. 가장자리가 margin 을 지나는 거리(마지막 교차)를 보간한다."""
    pts = [(float(p[0]), float(p[1]), float(p[2])) for p in track if p[1] is not None and p[2] is not None]
    if not pts:
        return {"n": 0, "tag_cut_m": None, "last_seen_m": None}
    cut = None
    for (_, f0, e0), (_, f1, e1) in zip(pts, pts[1:]):
        if e0 >= margin_px > e1 and e0 != e1:
            cut = f0 + (f1 - f0) * (e0 - margin_px) / (e0 - e1)
    return {"n": len(pts), "tag_cut_m": cut, "last_seen_m": min(f for _, f, _ in pts),
            "edge_min_px": min(e for _, _, e in pts), "edge_start_px": pts[0][2]}


# ── 카메라 대조 3종 (결정 11) ────────────────────────────────────────
def tag_cut_live_check(track, intr, cut_measured_m=None):
    """실행이 쓰는 실시간 태그컷 식(limits.tag_cut_live_m)을 접근 다리의 프레임마다 돌려 실측 태그컷과 견준다.
    track = [(t_rel, forward_m, edge_px, vertical, top_px)] (hold 의 5열). 실측 태그컷을 주면 그보다 먼 프레임만 쓴다
    (지난 뒤의 프레임은 윗변이 이미 띠 안이라 자기 자신을 맞춘다). err_m = 예측 중앙값 − 실측."""
    from src import limits
    pts = []
    for p in track:
        if len(p) < 5 or p[1] is None or p[3] is None or p[4] is None:
            continue
        t, fwd, _, vert, top = float(p[0]), float(p[1]), p[2], float(p[3]), float(p[4])
        if cut_measured_m is not None and fwd <= cut_measured_m:
            continue
        if fwd > 0 and math.isfinite(vert) and math.isfinite(top):
            pts.append((t, fwd, limits.tag_cut_live_m(intr, -vert, fwd, top)))
    if not pts:
        return {"n": 0}
    preds = [c for _, _, c in pts]
    far = max(pts, key=lambda x: x[1])
    near = min(pts, key=lambda x: x[1])
    med = S.median(preds)
    return {"n": len(pts), "pred_median_m": med, "pred_sd_m": _sd(preds) or 0.0,
            "far_m": far[1], "pred_far_m": far[2], "near_m": near[1], "pred_near_m": near[2],
            "err_m": (med - cut_measured_m) if cut_measured_m is not None else None}


def camcheck(intr, tag_px, forward_m, row_px, tape_m, height_diff_m,
             tag_size_m=D.TAG_SIZE_M, margin_px=D.TAG_EDGE_MARGIN_PX):
    """줄자 tape_m 에서: (a) 태그 px 예측 fx·태그/줄자 vs 실제 (b) 자세 거리 vs 줄자 (c) 태그 중심 행 예측 cy − fy·높이차/줄자 vs 실제.

    셋 다 '태그컷이 얼마나 틀어지나'[m] 로 바꿔 한 자로 잰다 — 통로·태그컷 입력이 전부 여기서 흔들린다.
    """
    _, _, fx, fy, cx, cy = _intr6(intr)

    def cut_of(hd):                         # limits.tag_cut_m 과 같은 식. 높이차만 입력으로 받는다
        return (hd + tag_size_m / 2.0) * fy / (cy - margin_px)
    px_pred = fx * tag_size_m / tape_m
    dist_from_px = fx * tag_size_m / tag_px if tag_px else None
    row_pred = cy - fy * height_diff_m / tape_m
    hd_from_row = (cy - row_px) / fy * tape_m
    return {"tape_m": tape_m,
            "px_pred": px_pred, "px_meas": tag_px, "dist_from_px_m": dist_from_px,
            "err_px_m": (dist_from_px - tape_m) if dist_from_px else None,
            "forward_meas_m": forward_m, "err_dist_m": forward_m - tape_m,
            "row_pred": row_pred, "row_meas": row_px,
            "height_diff_in_m": height_diff_m, "height_diff_from_row_m": hd_from_row,
            "cut_from_input_m": cut_of(height_diff_m), "cut_from_row_m": cut_of(hd_from_row),
            "err_cut_m": cut_of(hd_from_row) - cut_of(height_diff_m)}


# ── 폴더에서 다시 내기 ───────────────────────────────────────────────
def load_jsonl(path):
    out = []
    p = Path(path)
    if not p.is_file():
        return out
    with p.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except json.JSONDecodeError:
                    pass                    # 죽으면서 반만 적힌 줄
    return out


def latest_run(base):
    runs = sorted(d for d in Path(base).glob("*") if d.is_dir() and (d / "event.jsonl").exists())
    return runs[-1] if runs else None


def reanalyze(run_dir, gyro_tol_deg=0.3):
    """폴더 하나 → {이름: 재계산값}. before_run 이 적은 measured.json 과 비교하려고."""
    run_dir = Path(run_dir)
    ev = load_jsonl(run_dir / "event.jsonl")
    fr = load_jsonl(run_dir / "frame.jsonl")
    out = {}
    still = [r for r in fr if r.get("stage") == "sigma_still" and r.get("seen")]
    s = still_sigma(still)
    if s:
        out.update({k: s[k] for k in ("sigma_ref_distance_m", "sigma_drift_lateral_m", "sigma_drift_heading_deg")})
    legs = [e for e in ev if e.get("event") == "sigma_drive_leg" and e.get("t_onset") and e.get("t_stop_cmd")]
    ds = []
    for e in legs:
        rows = [r for r in fr if r.get("stage") == "sigma_drive" and e["t_onset"] <= r.get("t", -1) <= e["t_stop_cmd"]]
        d = drive_sigma(rows, out.get("sigma_ref_distance_m"))
        if d:
            ds.append(d)
    if ds:
        out["sigma_drive_lateral_m"] = _med([d["lateral_drift_m"] for d in ds])
        out["sigma_drive_heading_deg"] = _med([d["heading_drift_deg"] for d in ds])
    stops = [e for e in ev if e.get("event") == "rotcenter_stop"]
    if stops:
        f = rot_center_fit(stops)
        for k in ("cam_to_rot_center_m", "rot_center_lateral_m", "circle_rms_mm"):
            out[k] = f.get(k)
    trials = [e for e in ev if e.get("event") == "veer_trial"]
    if trials:
        out["veer_deg"] = veer_from_trials(trials, gyro_tol_deg)["veer_deg"]
    floors = [e.get("turned_deg") for e in ev if e.get("event") == "rotfloor" and e.get("ok")]
    if floors:
        out["rot_floor_deg"] = rot_floor(floors)["rot_floor_deg"]
    backs = [e for e in ev if e.get("event") == "hold" and e.get("why") == "backspeed"]
    sp = [speed_from_track(e["track"], e.get("sigma_m"), e["t_stop_cmd"] - e["t_cmd"]) for e in backs]
    sp = [x for x in sp if x.get("speed_mps")]
    if sp:
        out["back_speed_mps"] = _med([x["speed_mps"] for x in sp])
        out["back_startup_s"] = _med([x["onset_s"] for x in sp])
    cuts = [e for e in ev if e.get("event") == "hold" and e.get("why") == "tagcut"]
    if cuts:
        out["tag_cut_m"] = tag_cut_from_track(cuts[-1]["track"])["tag_cut_m"]
    return out


def main():
    ap = argparse.ArgumentParser(description="before_run 기록에서 측정값 다시 내기")
    ap.add_argument("run_dir", nargs="?", help="work_dirs/before_run/<시각>. 없으면 최신")
    ap.add_argument("--out", default=None, help="기록 루트 (bootstrap.work_root 와 같은 규칙)")
    ap.add_argument("--selftest", action="store_true", help="하드웨어 없이 계산식 검사")
    args = ap.parse_args()
    if args.selftest:
        return _selftest()
    run_dir = Path(args.run_dir) if args.run_dir else latest_run(work_root(args.out) / "before_run")
    if run_dir is None or not run_dir.is_dir():
        print("!! 폴더가 없다: %s" % run_dir)
        return 1
    print("== %s" % run_dir)
    mpath = run_dir / "measured.json"
    measured = json.loads(mpath.read_text()) if mpath.is_file() else {}
    if measured.get("fake_source"):
        print("   (fake_source — 가짜 장비로 만든 기록이다. 값은 실측이 아니다)")
    again = reanalyze(run_dir)
    print("%-26s %14s %14s" % ("이름", "measured.json", "재계산"))
    keys = sorted(set(again) | {k for k in measured if k in again})
    for k in keys:
        a, b = measured.get(k), again.get(k)
        fa = "-" if a is None else ("%.4f" % a if isinstance(a, (int, float)) else str(a))
        fb = "-" if b is None else ("%.4f" % b if isinstance(b, (int, float)) else str(b))
        mark = ""
        if isinstance(a, (int, float)) and isinstance(b, (int, float)) and abs(a - b) > 1e-3 * max(1.0, abs(a)):
            mark = "  *"
        print("%-26s %14s %14s%s" % (k, fa, fb, mark))
    return 0


# ── 자체 시험 ────────────────────────────────────────────────────────
def _selftest():
    import random
    rng = random.Random(7)
    fails = []

    def check(cond, msg):
        print(("  ok   " if cond else "  FAIL ") + msg)
        if not cond:
            fails.append(msg)

    # 1) 회전중심: 카메라 앞 1.47 m · 왼쪽 0.03 m 둘레를 5도씩 ±40도 스윙. 위치 잡음 30 mm · 방향 0.5도 (9/21 급)
    o_f, o_l = 1.47, 0.03
    c = np.array([-4.0 + o_f, 0.10 + o_l])                                   # 세계 중심 = p0 + M(0)·o
    stops = []
    seq = [(+1, k) for k in range(8)] + [(-1, k) for k in range(16)] + [(+1, k) for k in range(8)]
    psi = 0.0
    for d, _ in [(0, 0)] + seq:
        psi += d * 5.0
        p = c - _rot2(math.radians(psi)) @ np.array([o_f, o_l])
        stops.append({"lateral": p[1] + rng.gauss(0, 0.03), "forward": -p[0] + rng.gauss(0, 0.03),
                      "heading_deg": psi + rng.gauss(0, 0.5), "beta_deg": 0.63 * psi, "psi_rel_deg": psi, "dir": d})
    f = rot_center_fit(stops)
    print("     회전중심 %.3f (참 %.3f) 좌우 %.3f (참 %.3f) LS rms %.0f mm · 동심원 r %.3f rms %.0f mm · 쌍 %d개 흩어짐 %s · gain %.2f"
          % (f["cam_to_rot_center_m"], -o_f, f["rot_center_lateral_m"], o_l, f["ls_rms_mm"],
             f["circle_radius_m"], f["circle_rms_mm"], f["pairs_n"],
             "%.2f" % f["pairs_spread_m"] if f["pairs_spread_m"] else "-", f["gain"]))
    check(abs(f["cam_to_rot_center_m"] + o_f) < 0.07, "위치+방향 적합이 ±0.07 m 안 (plan 4-7 필요 정확도)")
    check(abs(f["circle_radius_m"] - math.hypot(o_f, o_l)) < 0.10, "동심원 반지름이 맞는다")
    check(f["circle_rms_mm"] < 60 and abs(f["gain"] - 0.63) < 0.05, "RMS 가 잡음 급이고 gain 이 맞는다")
    check("L" in f["sides"] and "R" in f["sides"], "좌·우 따로도 나온다")
    check(rot_center_fit(stops[:2]).get("why") == "few", "점이 모자라면 few")

    # 2) 쏠림: 방향 -1.0도로 서서 3 m 갔는데 좌우가 +0.3도 만큼 더 샜다 → veer +0.3
    tr = []
    for k in range(4):
        h = -1.0 + rng.gauss(0, 0.1)
        run = 3.0
        lat1 = 0.2 + run * math.sin(math.radians(h + 0.3))
        tr.append({"lat0": 0.2, "fwd0": 6.8, "lat1": lat1, "fwd1": 6.8 - run, "heading_deg": h,
                   "gyro_delta_deg": 0.05 if k < 3 else 2.0, "legs_ok": True})
    v = veer_from_trials(tr, 0.3)
    check(v["n_used"] == 3 and abs(v["veer_deg"] - 0.3) < 0.02, "쏠림 %.3f도, 자이로가 돈 시행은 뺐다" % v["veer_deg"])

    # 3) 속도: 죽은시간 1.5 s 뒤 0.29 m/s 로 멀어진다 (후진). 잡음 10 mm
    track = []
    for i in range(150):
        t = i / 30.0
        d = 4.0 + max(0.0, t - 1.5) * 0.29 + rng.gauss(0, 0.01)
        track.append((t, d, 100.0))
    sp = speed_from_track(track, 0.01, 4.5)
    check(sp["speed_mps"] and abs(sp["speed_mps"] - 0.29) < 0.02 and 1.4 < sp["onset_s"] < 1.8,
          "속도 %.3f m/s, 출발 %.2f s" % (sp["speed_mps"], sp["onset_s"]))
    check(speed_from_track(track[:5], 0.01, 4.5).get("why") == "few", "점이 모자라면 few")

    # 4) 태그컷: 가장자리 px 가 거리에 따라 직선으로 줄어 3.30 m 에서 60 px
    tc = [(i / 30.0, 4.0 - i * 0.01, 60.0 + (4.0 - i * 0.01 - 3.30) * 400.0) for i in range(90)]
    r = tag_cut_from_track(tc, 60.0)
    check(abs(r["tag_cut_m"] - 3.30) < 1e-6 and r["last_seen_m"] < 3.30, "태그컷 보간 %.3f m" % r["tag_cut_m"])
    # 실시간 식 대조: 피치 0 인 합성 접근 (윗변 행 = 피치 없는 모델) → 모든 프레임이 고정식 태그컷을 맞춘다
    from src import limits as _L
    _ref = D.D435I_COLOR_REF
    _dz = D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M
    _cut = _L.tag_cut_m(_ref, _dz)
    tc5 = [(i * 0.1, f, 100.0, -_dz, _ref[5] - _ref[3] * (_dz + D.TAG_SIZE_M / 2) / f)
           for i, f in enumerate([6.0, 5.5, 5.0, 4.5, 4.0, 3.6, 3.2])]
    lc = tag_cut_live_check(tc5, _ref, _cut)
    check(lc["n"] == 6 and abs(lc["err_m"]) < 1e-9 and lc["pred_sd_m"] < 1e-9,
          "실시간 식 대조: 피치 0 합성 %d 장 → %.3f m (고정식 %.3f)" % (lc["n"], lc["pred_median_m"], _cut))
    check(tag_cut_live_check(tc, _ref)["n"] == 0, "3열 track(옛 기록)이면 n=0")

    # 5) 카메라 대조: 자기 자신과 맞으면 오차 0
    w, h, fx, fy, cx, cy = D.D435I_COLOR_REF
    cc = camcheck(D.D435I_COLOR_REF, fx * 0.3 / 4.0, 4.0, cy - fy * 1.10 / 4.0, 4.0, 1.10)
    check(abs(cc["err_px_m"]) < 1e-9 and abs(cc["err_dist_m"]) < 1e-9 and abs(cc["err_cut_m"]) < 1e-9,
          "대조 3종 오차 0 (태그컷 %.2f m)" % cc["cut_from_input_m"])
    cc = camcheck(D.D435I_COLOR_REF, fx * 0.3 / 4.2, 4.0, cy - fy * 1.10 / 4.0, 4.0, 1.10)
    check(abs(cc["err_px_m"] - 0.2) < 1e-9, "태그가 작게 보이면 거리 오차 +0.2 m 로 나온다")

    # 6) σ: 느린 성분 + 흰 잡음. 밀림이 느린 성분 급으로 나온다
    n, fps, phi = 1800, 30.0, 0.99            # 상관시간 3.3 s — 60 초 안에 여러 번 풀려야 블록 sd 가 선다
    s, lat, head = 0.0, [], []
    kk = math.sqrt(1 - phi * phi)
    for _ in range(n):
        s = phi * s + kk * 0.030 * rng.gauss(0, 1)
        lat.append(s + 0.025 * rng.gauss(0, 1))
        head.append(s * 15 + 0.37 * rng.gauss(0, 1))
    rows = [{"t": 1000 + i / fps, "lateral": lat[i], "heading_deg": head[i], "distance": 3.7, "angle_ok": i % 9 != 4}
            for i in range(n)]
    ss = still_sigma(rows)
    print("     정지 σ: 좌우 밀림 %.1f mm (한 장 sd %.1f, 블록 sd %.1f, 떨림 %.1f) · 방향 %.3f도 · 기준 %.2f m · 모호 %.0f %%"
          % (ss["sigma_drift_lateral_m"] * 1e3, ss["lateral"]["frame_sd"] * 1e3, ss["lateral"]["block_sd"] * 1e3,
             ss["lateral"]["resid_sd"] * 1e3, ss["sigma_drift_heading_deg"], ss["sigma_ref_distance_m"],
             ss["ambiguous_rate"] * 100))
    check(0.012 < ss["sigma_drift_lateral_m"] < 0.060 and ss["sigma_ref_distance_m"] == 3.7, "밀림이 느린 성분 급이다")
    dr = drive_sigma([dict(r, lateral=r["lateral"] + 0.1 * (r["t"] - 1000)) for r in rows], 3.7)
    check(dr and abs(dr["lateral_drift_m"] - ss["sigma_drift_lateral_m"]) < 0.5 * ss["sigma_drift_lateral_m"],
          "직진 σ: 추세를 빼면 같은 밀림이 나온다 (%.1f mm)" % (dr["lateral_drift_m"] * 1e3))
    dr2 = drive_sigma([dict(r, distance=7.4) for r in rows], 3.7)
    check(dr2 and abs(dr2["lateral_drift_m"] - dr["lateral_drift_m"] * 0.25) < 0.3 * dr["lateral_drift_m"] * 0.25,
          "7.4 m 에서 잰 밀림은 기준 3.7 m 로 1/4 이 된다")
    check(rot_floor([0.9, -1.2, 0.7])["rot_floor_deg"] == 1.2, "하한 = 최대각")
    print("\n%s" % ("analyze_before_run 자체 시험 통과" if not fails else "실패 %d: %s" % (len(fails), fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
