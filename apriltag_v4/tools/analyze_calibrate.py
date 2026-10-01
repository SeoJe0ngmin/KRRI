"""calibrate 기록에서 값을 **다시** 낸다 — 판정을 기록 시점에 박지 않기 위해 (plan 6-3 ⑤).

    python tools/analyze_calibrate.py                             # 최신 폴더
    python tools/analyze_calibrate.py work_dirs/calibrate/<시각>   # 그 폴더
    python tools/analyze_calibrate.py --selftest                  # 하드웨어 없이 계산식만

calibrate.py 는 현장에서 **여기 있는 함수**로 값을 내어 results.json 에 적는다. 이 도구는 같은 폴더의 event.jsonl 에서
같은 함수로 다시 내어 나란히 보인다. 둘이 다르면 기록이 샜거나 함수가 바뀐 것이다.

    회전중심    두 가지로 교차검증 (plan 4-7)
                  위치+방향   p_i + M(ψ_i)·o = c 를 최소제곱으로. 연속 스텝 쌍의 독립 추정으로 흩어짐도 본다
                  위치만      동심원 적합 → RMS. 광운대 rotation_fit/icr_fit.py 의 생각(반지름이 일정한 중심)만 빌렸다
    회전 하한    움직이자마자 끊은 회전들의 |각| 의 최대 (plan 4-5)

좌표 약속은 pose.state · sidestep.py 와 같다: 좌우 +왼쪽, 방향 +반시계, 세계 x = −forward(태그 쪽), y = lateral.
σ 기준선은 여기 없다 — run.py 가 출발 뒤 estimate.baseline_from_rows 로 잰다 (2026-10-02).
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

#: 회전중심 적합에 최소로 필요한 정지점 수. 미지수 4 개(중심 오프셋 2 + 세계 중심 2)에 점당 식 2 개
FIT_MIN_STOPS = 4
#: 연속 쌍 독립 추정에서 이보다 적게 돈 쌍은 뺀다 [도] — (M_i − M_j) 가 특이해진다
PAIR_MIN_DEG = 1.0

#: results.json 의 이름 ↔ config/control.py 의 이름 (calibrate 가 찍어 주는 줄)
CONFIG_KEYS = ("CAM_YAW_OFFSET_DEG", "CAM_TO_ROT_CENTER_M", "ROT_CENTER_LATERAL_M", "ROT_CENTER_RMS_MM", "ROT_FLOOR_DEG")


# ── 작은 도구 ─────────────────────────────────────────────────────────
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
    """정지점들 → 회전중심. cam_to_rot_center_m 은 config 규약(음수 = 카메라 앞).

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


def rot_center_from_bearing(stops, height_diff_m):
    """정지점들의 **거리와 화면위치(β)만**으로 회전중심 (2026-10-02 실차 스윙을 보고 이쪽을 주로 쓴다).

    차가 제자리에서 돌면, 카메라 좌표계에서 본 태그는 회전중심 둘레로 원을 그린다:
        q_i = ρ_i · (cos β_i, −sin β_i)      (앞, 왼쪽)      ρ = √(3D 거리² − 높이차²)
    원의 중심 = 카메라 좌표계의 회전중심 → A = −중심_앞 · b = 중심_왼쪽.  PnP 의 방향·좌우가 안 들어간다.
    4.2 m ±30도 스윙(calibrate 20261001_141357)에서 PnP 방향은 자세마다 +4 ~ −3도(좌우 ±0.2~0.3 m)씩 틀렸고,
    같은 자리를 갈 때·올 때 같은 쪽으로 틀렸다(잡음이 아니라 치우침) — 그걸로 맞춘 원은 잔차 162 mm 였다.
    불확실도(se_*)는 한 점씩 빼고 다시 맞춘 값의 흩어짐(잭나이프). 높이차는 ±0.1 m 틀려도 2 mm 만 바뀐다.
    """
    pts = [s for s in stops if s.get("distance") is not None and s.get("beta_deg") is not None]
    n = len(pts)
    out = {"n": n}
    if n < FIT_MIN_STOPS:
        out["why"] = "few"
        return out
    q = []
    for s in pts:
        rho = math.sqrt(max(0.0, s["distance"] ** 2 - height_diff_m ** 2))
        b = math.radians(s["beta_deg"])
        q.append((rho * math.cos(b), -rho * math.sin(b)))
    q = np.array(q)
    cx, cy, r, rms = circle_fit(q[:, 0], q[:, 1])
    se_f = se_l = None
    if n > FIT_MIN_STOPS:
        loo = np.array([circle_fit(np.delete(q[:, 0], i), np.delete(q[:, 1], i))[:2] for i in range(n)])
        k = math.sqrt((n - 1) / n)
        se_f = k * float(np.sqrt(((loo[:, 0] - loo[:, 0].mean()) ** 2).sum()))
        se_l = k * float(np.sqrt(((loo[:, 1] - loo[:, 1].mean()) ** 2).sum()))
    betas = [s["beta_deg"] for s in pts]
    out.update({"cam_to_rot_center_m": -cx, "rot_center_lateral_m": cy, "circle_radius_m": r, "circle_rms_mm": rms * 1e3,
                "se_forward_m": se_f, "se_lateral_m": se_l, "beta_span_deg": max(betas) - min(betas),
                "height_diff_m": height_diff_m})
    return out


def to_vehicle_frame(cam_to_rot_center_m, rot_center_lateral_m, cam_yaw_offset_deg):
    """적합은 카메라가 낸 **날것** heading 축에서 나온다(calibrate 는 cam_yaw 0 으로 연다). 계획기는 cam_yaw 를 뺀 차체
    heading 을 쓰므로 중심 오프셋을 그 각만큼 돌려야 같은 점이 된다: R(ψ_날것) = R(ψ_차체)·R(yaw) → o_차체 = R(yaw)·o_카메라.
    (A = −o_fwd, b = o_lat.) 안 돌리면 중심이 |A|·sin(yaw) 만큼 옆으로 틀린다 — 1.5 m · 1.14도 = 30 mm."""
    o = _rot2(math.radians(cam_yaw_offset_deg)) @ np.array([-cam_to_rot_center_m, rot_center_lateral_m])
    return -float(o[0]), float(o[1])


# ── 회전 하한 (plan 4-5) ──────────────────────────────────────────────
def rot_floor(turned_deg):
    """움직이자마자 끊은 회전들의 |각| → 최대가 하한. 평균이 아니라 최대 — 지나치는 쪽이 비싸다."""
    xs = [abs(x) for x in turned_deg if x is not None]
    return {"rot_floor_deg": max(xs) if xs else None, "median_deg": _med(xs), "sd_deg": _sd(xs), "n": len(xs)}


# ── 폴더 ─────────────────────────────────────────────────────────────
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


def reanalyze(run_dir):
    """폴더 하나 → {config 이름: 재계산값}. calibrate 가 적은 results.json 과 비교하려고."""
    ev = load_jsonl(Path(run_dir) / "event.jsonl")
    out = {}
    cy = [e for e in ev if e.get("event") == "camyaw"]
    if cy:
        out["CAM_YAW_OFFSET_DEG"] = cy[-1].get("cam_yaw_offset_deg")
    stops = [e for e in ev if e.get("event") == "rotcenter_stop"]
    if stops:
        rc = [e for e in ev if e.get("event") == "rotcenter"]
        yaw = rc[-1].get("cam_yaw_used_deg") if rc else None
        dz = rc[-1].get("height_diff_m") if rc else None
        # 거리+화면위치 원 맞춤이 주 (높이차가 기록에 있을 때). 옛 기록이면 PnP 위치+방향 적합
        f = rot_center_from_bearing(stops, dz) if dz is not None else rot_center_fit(stops)
        A, b = f.get("cam_to_rot_center_m"), f.get("rot_center_lateral_m")
        if A is not None and yaw is not None:
            A, b = to_vehicle_frame(A, b, yaw)                  # calibrate 가 쓴 것과 같은 각으로 차체 축에
        out["CAM_TO_ROT_CENTER_M"], out["ROT_CENTER_LATERAL_M"] = A, b
        out["ROT_CENTER_RMS_MM"] = f.get("circle_rms_mm")
    floors = [e.get("turned_deg") for e in ev if e.get("event") == "rotfloor" and e.get("ok")]
    if floors:
        out["ROT_FLOOR_DEG"] = rot_floor(floors)["rot_floor_deg"]
    return out


def main():
    ap = argparse.ArgumentParser(description="calibrate 기록에서 값 다시 내기")
    ap.add_argument("run_dir", nargs="?", help="work_dirs/calibrate/<시각>. 없으면 최신")
    ap.add_argument("--out", default=None, help="기록 루트 (bootstrap.work_root 와 같은 규칙)")
    ap.add_argument("--selftest", action="store_true", help="하드웨어 없이 계산식 검사")
    args = ap.parse_args()
    if args.selftest:
        return _selftest()
    run_dir = Path(args.run_dir) if args.run_dir else latest_run(work_root(args.out) / "calibrate")
    if run_dir is None or not run_dir.is_dir():
        print("!! 폴더가 없다: %s" % run_dir)
        return 1
    print("== %s" % run_dir)
    rpath = run_dir / "results.json"
    results = json.loads(rpath.read_text(encoding="utf-8")).get("config", {}) if rpath.is_file() else {}
    again = reanalyze(run_dir)
    print("%-22s %14s %14s" % ("config 이름", "results.json", "재계산"))
    for k in CONFIG_KEYS:
        a, b = results.get(k), again.get(k)
        fa = "-" if a is None else "%.4f" % a
        fb = "-" if b is None else "%.4f" % b
        mark = "  *" if (a is not None and b is not None and abs(a - b) > 1e-3 * max(1.0, abs(a))) else ""
        print("%-22s %14s %14s%s" % (k, fa, fb, mark))
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
    # 카메라가 차체와 2도 어긋나 있으면 날것 heading 이 전부 +2도 — 적합은 카메라 축 값을 내고, 돌려 주면 차체 축 참값이 된다
    yaw = 2.0
    clean = []
    psi = 0.0
    for d, _ in [(0, 0)] + seq:
        psi += d * 5.0
        p = c - _rot2(math.radians(psi)) @ np.array([o_f, o_l])
        clean.append({"lateral": p[1], "forward": -p[0], "heading_deg": psi + yaw, "beta_deg": 0.0, "psi_rel_deg": psi, "dir": d})
    fr = rot_center_fit(clean)
    Av, bv = to_vehicle_frame(fr["cam_to_rot_center_m"], fr["rot_center_lateral_m"], yaw)
    check(abs(fr["rot_center_lateral_m"] - o_l) > 0.04 and abs(Av + o_f) < 1e-6 and abs(bv - o_l) < 1e-6,
          "cam_yaw %.0f도: 카메라 축 좌우 %+.3f (틀림) → 차체 축 %+.3f · 앞뒤 %+.3f (참 %+.3f · %+.3f)"
          % (yaw, fr["rot_center_lateral_m"], bv, Av, o_l, -o_f))
    # 1-b) 거리+화면위치 원 맞춤: 같은 스윙을 (3D 거리, β) 로 만들어 넣으면 중심이 나온다. PnP 방향을 +4도 틀리게 줘도 그대로다
    dz0 = 0.96
    bs = []
    psi = 0.0
    for d, _ in [(0, 0)] + seq:
        psi += d * 5.0
        p = c - _rot2(math.radians(psi)) @ np.array([o_f, o_l])              # 카메라 위치 (태그가 원점)
        theta = math.degrees(math.atan2(-p[1], -p[0]))                        # 카메라에서 태그를 보는 세계 방향
        bs.append({"distance": math.sqrt(p @ p + dz0 * dz0) + rng.gauss(0, 0.02), "beta_deg": psi - theta,
                   "heading_deg": psi + 4.0, "lateral": p[1] + 0.3, "forward": -p[0]})
    fb = rot_center_from_bearing(bs, dz0)
    print("     거리+화면위치: 앞뒤 %.3f (참 %.3f) ±%.3f · 좌우 %+.3f (참 %+.3f) ±%.3f · RMS %.0f mm"
          % (fb["cam_to_rot_center_m"], -o_f, fb["se_forward_m"], fb["rot_center_lateral_m"], o_l, fb["se_lateral_m"], fb["circle_rms_mm"]))
    check(abs(fb["cam_to_rot_center_m"] + o_f) < 3 * fb["se_forward_m"] and abs(fb["rot_center_lateral_m"] - o_l) < 3 * fb["se_lateral_m"],
          "거리+화면위치 원 맞춤이 참값을 3·se 안에서 맞춘다 (PnP 방향이 틀려도)")
    check(rot_center_from_bearing(bs[:3], dz0).get("why") == "few", "점이 모자라면 few")
    # 2) 회전 하한 = 최대각
    check(rot_floor([0.9, -1.2, 0.7])["rot_floor_deg"] == 1.2, "하한 = 최대각")
    # 3) reanalyze 가 event 이름을 제대로 읽는다 (임시 폴더)
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        ev = [dict(s, event="rotcenter_stop") for s in stops] + [{"event": "rotfloor", "ok": True, "turned_deg": 0.95},
                                                                 {"event": "camyaw", "cam_yaw_offset_deg": 1.25}]
        (Path(td) / "event.jsonl").write_text("\n".join(json.dumps(e) for e in ev))
        r = reanalyze(td)
        check(abs(r["CAM_TO_ROT_CENTER_M"] - f["cam_to_rot_center_m"]) < 1e-9 and r["ROT_FLOOR_DEG"] == 0.95
              and r["CAM_YAW_OFFSET_DEG"] == 1.25, "reanalyze: event.jsonl → config 이름 5개")
    print("\n%s" % ("analyze_calibrate 자체 시험 통과" if not fails else "실패 %d: %s" % (len(fails), fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
