"""first_run 기록 → 캘리브 3종 + report.md.  **하드웨어 없이, numpy 만.**

    python tools/analyze_first_run.py work_dirs/first_run/20260922_0830_all
    python tools/analyze_first_run.py <폴더> --install     # config/ 에도 설치
    python tools/analyze_first_run.py --selftest           # 합성 데이터 자기 검사

무엇을 뽑나 (plan 근거)
────────────────────────────────────────────────────────────────────────
    forward → 셀별 τ_eff = median(D_obs/v̂), D_obs 는 **경로길이**(Δforward/cos ψ̂),
              σ_e·q95·n                                            (plan 2-1)
    rotate  → 개루프 ω(t) 에서 τ_r(L/R) = 코스팅 각/ω0              (plan 2-4)
              --camera 회전 → Δβ_px vs Δψ_gyro 회귀로 Â·s 분리      (plan 1-1·3-2)
    grid    → heading 바이어스 vs tilt → θ_min 제안                  (plan 1-3)
    creep   → v97·stiction_ok·데드존                                 (plan 2-5)
    timing  → L 통계·ε·Δ_FS·T_line·TXACK 오프셋 → timing_calib       (plan 4-1)
    static  → σ_c·flip 빈도 → perception_calib.static                (plan 1-4)

원칙
────────────────────────────────────────────────────────────────────────
· **판정은 전부 여기서** 한다. 기록 쪽에는 판정이 없다(plan 2-7).
· 표본 n < 10 인 항목은 provisional 을 유지한다 — 값은 쓰되 "잠정" 딱지를 뗄 수 없다.
· 타이밍 게이트 미통과(--gate-failed) 면 **캘리브 파일을 만들지 않는다**(plan 4-2).
· 가짜 소스(dry-run)로 만든 기록은 config.json 의 fake=true 로 알아채고 provisional 고정.
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from config import control as C                     # noqa: E402
from src.utils import calib as CAL                  # noqa: E402

#: 표본이 이보다 적으면 provisional 을 못 뗀다 (plan 2-1 분위수 규칙)
N_FIRM = 10
#: 움직였다고 볼 최소 속도 [m/s] — 이보다 느리면 그 구간은 버린다
V_MIN = 0.02
#: 회전이 돌고 있다고 볼 최소 각속도 [도/s]
W_MIN = 1.0
#: 정지 판정 각속도 [도/s]
W_STILL = 0.3
#: ε 탐색 범위·간격 [s]
EPS_RANGE, EPS_STEP = 0.25, 0.005

#: numpy 1.x 는 trapz, 2.x 는 trapezoid — VM/맥 어느 쪽에서도 돌아야 한다
_trapz = getattr(np, "trapezoid", None) or np.trapz


# ═══════════════════════════════════════════════════════════════════════════
# 읽기
# ═══════════════════════════════════════════════════════════════════════════

def read_jsonl(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except Exception:
                pass
    return out


class Stage:
    """단계 폴더 하나."""

    def __init__(self, d):
        self.dir = d
        self.name = os.path.basename(d).split("_")[-1]
        self.frames = read_jsonl(os.path.join(d, "frame.jsonl"))
        self.events = read_jsonl(os.path.join(d, "events.jsonl"))
        self.can = read_jsonl(os.path.join(d, "can.jsonl"))
        self.imu = read_jsonl(os.path.join(d, "imu.jsonl"))
        cfg = os.path.join(d, "config.json")
        self.config = {}
        if os.path.exists(cfg):
            try:
                with open(cfg, encoding="utf-8") as f:
                    self.config = json.load(f)
            except Exception:
                pass
        self.fake = bool((self.config.get("first_run") or {}).get("fake"))

    def __repr__(self):
        return "<Stage %s frames=%d events=%d>" % (self.name, len(self.frames),
                                                   len(self.events))


def load_stages(root):
    dirs = []
    if os.path.exists(os.path.join(root, "frame.jsonl")):
        dirs = [root]
    else:
        dirs = sorted(d for d in glob.glob(os.path.join(root, "*"))
                      if os.path.isdir(d) and os.path.exists(os.path.join(d, "frame.jsonl")))
    return [Stage(d) for d in dirs]


def stages_named(stages, name):
    return [s for s in stages if s.name == name]


# ═══════════════════════════════════════════════════════════════════════════
# 공통 수치
# ═══════════════════════════════════════════════════════════════════════════

def _arr(rows, key):
    return np.array([r.get(key) if r.get(key) is not None else np.nan
                     for r in rows], dtype=float)


def slope(t, x):
    """최소제곱 기울기. 점이 3개 미만이면 None."""
    t = np.asarray(t, float)
    x = np.asarray(x, float)
    ok = np.isfinite(t) & np.isfinite(x)
    if ok.sum() < 3:
        return None
    t, x = t[ok], x[ok]
    t = t - t.mean()
    den = (t * t).sum()
    if den <= 0:
        return None
    return float((t * (x - x.mean())).sum() / den)


def linfit(t, x):
    """(기울기, 절편). 점이 3개 미만이면 None. 정지 시각으로 **외삽**하려고 절편도 쓴다."""
    t = np.asarray(t, float)
    x = np.asarray(x, float)
    ok = np.isfinite(t) & np.isfinite(x)
    if ok.sum() < 3:
        return None
    t, x = t[ok], x[ok]
    tm, xm = t.mean(), x.mean()
    den = ((t - tm) ** 2).sum()
    if den <= 0:
        return None
    k = float(((t - tm) * (x - xm)).sum() / den)
    return k, float(xm - k * tm)


def median(v):
    v = [x for x in v if x is not None and np.isfinite(x)]
    return float(np.median(v)) if v else None


def q95(v):
    v = [x for x in v if x is not None and np.isfinite(x)]
    return float(np.percentile(v, 95)) if v else None


def frame_segments(frames, moving_only=True):
    """movement 가 같은 연속 구간으로 자른다. [{movement, i0, i1, rows}]"""
    segs = []
    cur = None
    for r in frames:
        mv = r.get("movement")
        if cur is None or mv != cur["movement"]:
            cur = {"movement": mv, "rows": []}
            segs.append(cur)
        cur["rows"].append(r)
    if moving_only:
        segs = [s for s in segs if s["movement"] not in (None, "stop")]
    return segs


def gyro_series(stage):
    """imu.jsonl → (t, ω[도/s]) 우리 부호규약. 없으면 frame.jsonl 의 gyro_dps 로 떨어진다."""
    if stage.imu:
        g = [r for r in stage.imu if r.get("s") == "gyro"]
        if len(g) > 10:
            t = np.array([r["t"] for r in g], float)
            # 기본 축 (0,-1,0) · IMU_YAW_SIGN=+1 → ω = -y [rad/s]
            w = np.degrees(-np.array([r.get("y", 0.0) for r in g], float))
            return t, w
    f = [r for r in stage.frames if r.get("gyro_dps") is not None
         and r.get("t_capture") is not None]
    if len(f) > 5:
        return (np.array([r["t_capture"] for r in f], float),
                np.array([r["gyro_dps"] for r in f], float))
    return np.array([]), np.array([])


def gyro_debiased(stage):
    t, w = gyro_series(stage)
    if t.size == 0:
        return t, w, 0.0
    still = np.abs(w) < 1.0
    bias = float(np.median(w[still])) if still.sum() > 20 else 0.0
    return t, w - bias, bias


def cmd_pairs(stage):
    """can.jsonl → [{movement, t_set, t_tx}] 시간순. **두 시각을 짝지어** 돌려준다.

    t_set = 우리가 명령을 바꾼 시각(결정 시각, dir="set").
    t_tx  = 그 movement 가 **CAN 버스에 처음 나간** 시각(dir="tx"/"fake_tx").
    둘의 차가 **명령 지연**(소프트웨어+드라이버)이고, **출발 지연은 t_tx 를 원점으로**
    재야 한다. 옛 코드는 t_set 만 썼고 전진은 "라벨이 바뀐 첫 프레임" 을 원점으로 삼아,
    프레임 양자화(최대 33 ms)와 송신 지연이 통째로 출발 지연에 섞여 들어갔다.

    tx 줄은 매 주기 나오고 set 줄과 순서가 엇갈릴 수 있으므로, **set 마다 그 시각
    이후 같은 movement 의 첫 tx** 를 찾는 방식으로 짝짓는다(순차 매칭 금지).
    """
    sets, txs = [], []
    for r in stage.can:
        mv = r.get("movement")
        if not mv:
            continue
        if r.get("dir") == "set":
            t = r.get("t_cmd_set") or r.get("ts")
            if t is not None:
                sets.append((float(t), mv))
        elif r.get("dir") in ("tx", "fake_tx"):
            t = r.get("t_cmd_tx") or r.get("ts")
            if t is not None:
                txs.append((float(t), mv))
    sets.sort()
    txs.sort()
    out = []
    for t_set, mv in sets:
        t_tx = None
        for t, m in txs:                       # 그 시각 이후 같은 movement 의 첫 tx
            if m == mv and t >= t_set - 0.020:  # 20 ms 앞까지는 같은 명령으로 본다
                t_tx = t
                break
        out.append({"movement": mv, "t_set": t_set, "t_tx": t_tx})
    return out


def command_timeline(stage):
    """옛 형태 [(t, movement)] — **t 는 버스에 나간 시각**(없으면 결정 시각)."""
    return sorted(((c["t_tx"] if c["t_tx"] is not None else c["t_set"]), c["movement"])
                  for c in cmd_pairs(stage))


def cmd_latency_ms(stages):
    """명령 지연 = t_tx − t_set [ms]. 사람이 물어본 '명령 지연시간' 이 이것이다."""
    v = [(c["t_tx"] - c["t_set"]) * 1000.0 for st in stages for c in cmd_pairs(st)
         if c["t_tx"] is not None and c["t_set"] is not None
         and -50.0 < (c["t_tx"] - c["t_set"]) * 1000.0 < 2000.0]
    if not v:
        return {"value": None, "p99": None, "n": 0}
    return {"value": median(v), "p99": q95(v), "n": len(v)}


def _tx_near(cmds_tx, t_ref, movement, back_s=2.0, fwd_s=0.5):
    """t_ref 근처에서 같은 movement 가 버스에 나간 시각. 없으면 None.

    앞뒤 양쪽을 본다 — ε 보정이 조금 틀려 t_ref 가 tx 보다 앞으로 밀려도 놓치지 않게.
    (한쪽만 보면 ε 가 나쁠 때 측정이 **조용히 사라진다**. 실제로 그랬다.)
    """
    best = None
    for t, mv in cmds_tx:
        if mv != movement:
            continue
        d = t_ref - t
        if -fwd_s <= d <= back_s:
            if best is None or abs(d) < abs(t_ref - best):
                best = t
    return best


def eps_usable(T):
    """ε 를 출발 지연의 시계 보정에 써도 되나. 못 믿으면 0 으로 두고 그 사실을 남긴다."""
    e = (T or {}).get("epsilon_ms") or {}
    v, sg, n = e.get("value"), e.get("sigma"), e.get("n") or 0
    if v is None or n < 3:
        return 0.0, "ε 표본 부족 — 시계 보정 없이 쟀다"
    if sg is not None and sg > 50.0:
        return 0.0, "ε 산포 %.0f ms 로 커서 안 썼다" % sg
    if abs(v) > 250.0:
        return 0.0, "ε %.0f ms 가 비정상이라 안 썼다" % v
    return v / 1000.0, None


def cmd_latency_ms(stages):
    """명령 지연 = t_tx − t_set [ms]. 사람이 물어본 '명령 지연시간' 이 이것이다."""
    v = [(c["t_tx"] - c["t_set"]) * 1000.0 for st in stages for c in cmd_pairs(st)
         if c["t_tx"] is not None and c["t_set"] is not None
         and -50.0 < (c["t_tx"] - c["t_set"]) * 1000.0 < 2000.0]
    if not v:
        return {"value": None, "p99": None, "n": 0}
    return {"value": median(v), "p99": q95(v), "n": len(v)}


# ═══════════════════════════════════════════════════════════════════════════
# 1) 직진 — 셀별 τ_eff
# ═══════════════════════════════════════════════════════════════════════════

LEVEL_OF = {"forward": 67, "backward": 67, "forward_slow": 97}
DIR_OF = {"forward": "fwd", "forward_slow": "fwd", "backward": "bwd"}


def s_of_t(stages):
    """S(T) 표 — 명령길이 버킷별 이동거리 중앙값·CV. (plan 2-1②, 계약 §5.1)

    predict.Dynamics 가 `S_of_T.{67,97}` 을 [{T, median, cv, n}] 로 읽는다. 버킷당
    3회 미만이면 CV 가 뜻이 없어 버린다 — 표가 비면 Dynamics 가 다리 제한으로 떨어진다.
    """
    per = {67: {}, 97: {}}
    for st in stages:
        frames = [r for r in st.frames if r.get("t_capture") is not None]
        for seg in frame_segments(frames):
            mv = seg["movement"]
            lvl = LEVEL_OF.get(mv)
            if lvl is None or DIR_OF.get(mv) != "fwd":
                continue
            rows = [r for r in seg["rows"] if r.get("forward") is not None]
            if len(rows) < 4:
                continue
            sec = float(rows[-1]["t_capture"]) - float(rows[0]["t_capture"])
            dist = abs(float(rows[0]["forward"]) - float(rows[-1]["forward"]))
            if sec <= 0.2 or dist <= 0.0:
                continue
            per[lvl].setdefault(round(sec * 2.0) / 2.0, []).append(dist)
    out = {}
    for lvl, buckets in per.items():
        tab = []
        for T in sorted(buckets):
            v = buckets[T]
            if len(v) < 3:
                continue
            m = float(np.median(v))
            cv = float(np.std(v, ddof=1)) / m if m > 1e-6 else None
            tab.append({"T": float(T), "median": m,
                        "cv": (None if cv is None else float(cv)), "n": len(v)})
        out[str(lvl)] = tab
    return out


def analyze_forward(stages, eps_s=0.0):
    """정속정지 구간마다 τ_eff = D_obs/v̂. 셀별로 모아 표로.

    같은 구간에서 **전진 출발 죽은시간**(명령 → 3 cm 움직임)도 같이 뽑는다.
    예전엔 회전 지연 하나만 재서 `tau_start_s` 가 전진에도 그대로 쓰였다.
    """
    cells = {}
    rows = []
    onset_fwd = []
    for st in stages:
        cmds_tx = [(c["t_tx"], c["movement"]) for c in cmd_pairs(st)
                   if c["t_tx"] is not None]
        frames = [r for r in st.frames if r.get("t_capture") is not None]
        for k, seg in enumerate(frame_segments(frames)):
            mv = seg["movement"]
            if mv not in LEVEL_OF:
                continue
            body = [r for r in seg["rows"] if r.get("forward") is not None]
            if len(body) < 6:
                continue
            t = _arr(body, "t_capture")
            x = _arr(body, "forward")
            head = _arr(body, "heading_deg")
            t_stop = float(t[-1])
            # v̂ = 정지 직전 1 s 의 회귀 기울기 (forward 는 전진하면 줄어든다).
            # x_at_stop 은 그 직선을 **정지 명령 시각으로 외삽**한 값 — 마지막 몇 프레임의
            # 중앙값을 쓰면 프레임 간격의 절반만큼 늦은 위치가 되어 τ 가 부풀려진다.
            win = t >= (t_stop - 1.0)
            if win.sum() < 3:
                win = np.ones_like(t, bool)
            fit = linfit(t[win], x[win])
            if fit is None:
                continue
            sl, b0 = fit
            v = -sl
            if abs(v) < V_MIN:
                rows.append({"cell": None, "why": "안 움직임", "v": v, "dir": st.dir})
                continue
            # 출발 죽은시간 = **CAN 에 나간 시각 → 3 cm 움직인 시각**.
            # t 는 카메라 시계(t_capture)라 ε 를 더해 시스템 시계로 옮긴 뒤 t_tx 를 뺀다.
            # (옛 코드는 t[0] = "라벨 바뀐 첫 프레임" 을 원점으로 써서 프레임 양자화
            #  최대 33 ms + 송신 지연이 통째로 섞였다.)
            moved = np.abs(x - x[0]) > 0.03
            if DIR_OF.get(mv) == "fwd" and moved.any():
                t_move = float(t[np.argmax(moved)]) + eps_s
                t_org = _tx_near(cmds_tx, float(t[0]) + eps_s, mv)
                if t_org is not None and 0.0 < t_move - t_org < 5.0:
                    onset_fwd.append(t_move - t_org)
            x_at_stop = sl * t_stop + b0
            # 정지 후: 같은 정지 구간 안에서, 명령 뒤 1 s 지나고부터 30 프레임 중앙값.
            # **다음 명령이 시작되면 거기서 끊는다** (복귀 주행이 섞이면 τ 가 망가진다)
            after = []
            for r in frames:
                tc = r.get("t_capture")
                if tc is None or tc <= t_stop:
                    continue
                if r.get("movement") not in (None, "stop"):
                    break
                if tc > t_stop + 1.0 and r.get("forward") is not None:
                    after.append(r)
                if len(after) >= 30:
                    break
            if len(after) < 3:
                continue
            x_final = float(np.nanmedian(_arr(after, "forward")))
            psi = float(np.nanmedian(head)) if np.isfinite(head).any() else 0.0
            d_obs = (x_at_stop - x_final) / max(0.2, math.cos(math.radians(psi)))
            if mv == "backward":
                d_obs = -d_obs
            tau = d_obs / abs(v)
            cell = "%d_%s" % (LEVEL_OF[mv], DIR_OF[mv])
            rec = {"cell": cell, "tau": tau, "v": abs(v), "d_obs": d_obs,
                   "heading": psi, "t_stop": t_stop, "dir": st.dir, "seg": k}
            rows.append(rec)
            if 0.0 < tau < 3.0 and abs(d_obs) < 2.0:
                cells.setdefault(cell, []).append(rec)
    out = {}
    for cell, recs in cells.items():
        taus = [r["tau"] for r in recs]
        tau = median(taus)
        e = [r["d_obs"] - tau * r["v"] for r in recs]
        out[cell] = {"value": tau, "sigma_e_m": (float(np.std(e, ddof=1))
                                                 if len(e) > 1 else None),
                     "q95_m": q95([abs(x) for x in e]), "n": len(recs),
                     "v_mps": median([r["v"] for r in recs])}
    return {"cells": out, "rows": rows,
            "tau_start_fwd": {"value": median(onset_fwd), "n": len(onset_fwd)},
            "S_of_T": s_of_t(stages)}


# ═══════════════════════════════════════════════════════════════════════════
# 2) 회전 — τ_r(L/R), Â·s
# ═══════════════════════════════════════════════════════════════════════════

def analyze_rotate(stages):
    """회전 τ_r(L/R) · 출발 지연 · **α_up/α_r** (plan 2-4 ω 규칙의 가속·감속 항).

    α 는 200 Hz ω(t) 의 램프 기울기에서 바로 나온다 — 개루프 펄스가 여러 길이일
    필요가 없다(예전 주석이 그렇게 적혀 있어 두 항이 영영 None 이었다).
      α_up = 출발 직후 ω 가 0 → 0.9·ω_ss 로 오르는 구간의 기울기 [°/s²]
      α_r  = stop 뒤 ω 가 ω0 → 정지 문턱으로 떨어지는 구간의 기울기 [°/s²]
    """
    out = {"L": [], "R": [], "start": [], "rows": [],
           "alpha_up": {"L": [], "R": []}, "alpha_r": {"L": [], "R": []}}
    for st in stages:
        t, w, bias = gyro_debiased(st)
        if t.size < 50:
            continue
        cmds = command_timeline(st)
        for i, (t0, mv) in enumerate(cmds):
            if mv not in ("rotate_ccw", "rotate_cw"):
                continue
            t1 = cmds[i + 1][0] if i + 1 < len(cmds) else t[-1]
            side = "L" if mv == "rotate_ccw" else "R"
            sgn = 1.0 if side == "L" else -1.0
            # 정지 명령 직전 0.3 s 의 ω0
            pre = (t >= t1 - 0.3) & (t <= t1)
            if pre.sum() < 3:
                continue
            w0 = float(np.mean(w[pre]) * sgn)
            if w0 < W_MIN:
                continue
            # 코스팅: t1 이후 |ω| 가 정지 문턱 밑으로 갈 때까지 적분
            post = (t > t1) & (t <= t1 + 3.0)
            tp, wp = t[post], w[post] * sgn
            if tp.size < 3:
                continue
            stop_i = np.argmax(wp < W_STILL) if (wp < W_STILL).any() else wp.size - 1
            stop_i = max(1, int(stop_i))
            coast = float(_trapz(wp[:stop_i + 1], tp[:stop_i + 1]))
            tau_r = coast / w0
            # 출발 지연: 명령이 **버스에 나간 시각** → |ω| > 0.5 도/s.
            # 차가 명령 시점에 이미 돌고 있으면(직전 회전의 코스팅) 첫 샘플이 바로
            # 문턱을 넘어 0 에 가까운 값이 나온다 — 실제로 그래서 0.85 s 대신 0.01 s 가
            # 나왔다. **명령 직전에 멎어 있던 시행만** 센다.
            seg = (t >= t0) & (t <= t1)
            ts, ws = t[seg], np.abs(w[seg])
            t_start = None
            pre_still = (t >= t0 - 0.30) & (t < t0)
            was_still = bool(pre_still.any() and np.all(np.abs(w[pre_still]) < W_STILL))
            if was_still and ts.size:
                # 문턱은 **그 시행의 정지 잡음에서** 만든다(고정 0.5 도/s 는 잡음이 크면
                # 한 샘플만 튀어도 걸린다 — 실제로 0.85 s 를 0.67 s 로 이르게 봤다).
                noise = float(np.std(np.abs(w[pre_still]))) if pre_still.sum() > 5 else 0.0
                thr = max(0.5, 5.0 * noise)
                hit = ws > thr
                # 3 샘플 연속이라야 진짜 움직임으로 본다(200 Hz 에서 15 ms)
                run = hit & np.roll(hit, -1) & np.roll(hit, -2)
                run[-2:] = False
                if run.any():
                    t_start = float(ts[np.argmax(run)] - t0)
                    if not 0.05 < t_start < 3.0:   # 말이 안 되는 값은 버린다
                        t_start = None
            # α_r — 코스팅 구간의 평균 감속 [°/s²]
            a_r = None
            if stop_i >= 1 and (tp[stop_i] - tp[0]) > 1e-3:
                a_r = float((w0 - wp[stop_i]) / (tp[stop_i] - tp[0]))
                if a_r > 0:
                    out["alpha_r"][side].append(a_r)
            # α_up — 출발 램프 기울기 [°/s²]
            a_up = None
            if t_start is not None:
                up = (t >= t0 + t_start) & (t <= t1)
                tu, wu = t[up], w[up] * sgn
                hit = wu >= 0.9 * w0
                if tu.size >= 3 and hit.any():
                    j = max(1, int(np.argmax(hit)))
                    if (tu[j] - tu[0]) > 1e-3:
                        a_up = float((wu[j] - wu[0]) / (tu[j] - tu[0]))
                        if a_up > 0:
                            out["alpha_up"][side].append(a_up)
            rec = {"side": side, "tau_r": tau_r, "w0": w0, "coast_deg": coast,
                   "t_start": t_start, "t_cmd": t0, "dir": st.dir,
                   "alpha_r": a_r, "alpha_up": a_up}
            out["rows"].append(rec)
            if 0.0 < tau_r < 1.5:
                out[side].append(tau_r)
            if t_start is not None:
                out["start"].append(t_start)
    res = {}
    for side in ("L", "R"):
        v = out[side]
        res[side] = {"value": median(v),
                     "sigma": float(np.std(v, ddof=1)) if len(v) > 1 else None,
                     "n": len(v),
                     "alpha_r_dps2": median(out["alpha_r"][side]),
                     "alpha_up_dps2": median(out["alpha_up"][side])}
    res["tau_start_s"] = median(out["start"])
    res["tau_start_n"] = len(out["start"])
    res["rows"] = out["rows"]
    return res


#: 회전팔 A 의 물리 사전범위 [m]. 이 밖이면 적합이 깨진 것이다 (plan 1-1: |A| ≲ 0.4)
ARM_PRIOR_RANGE_M = 0.5


def analyze_arm(stages):
    """rotate_camera 이벤트 → Δβ/Δψ = s(1 + A cosβ/d) 회귀. (A 부호 포함)

    **사전범위 밖이거나 σ 를 못 내면 value=None 을 낸다.** −7.55 m 같은 값이
    "측정됨" 으로 나가면 위치 KF 가 회전 때마다 초당 1 m 의 가짜 횡이동을 적분한다.
    """
    pts = []
    for st in stages:
        for e in st.events:
            if e.get("event") != "rotate_camera":
                continue
            b, a, r = e.get("before"), e.get("after"), e.get("rotate")
            if not (b and a and r):
                continue
            db = (a.get("beta_px_deg") or np.nan) - (b.get("beta_px_deg") or np.nan)
            dpsi = r.get("turned_deg")
            if dpsi is None or not np.isfinite(db) or abs(dpsi) < 2.0:
                continue
            beta = 0.5 * ((a.get("beta_px_deg") or 0.0) + (b.get("beta_px_deg") or 0.0))
            d = 0.5 * ((a.get("forward") or np.nan) + (b.get("forward") or np.nan))
            if not np.isfinite(d) or d <= 0.3:
                continue
            pts.append({"y": db / dpsi, "x": math.cos(math.radians(beta)) / d,
                        "dpsi": dpsi, "d": d, "beta": beta})
    if not pts:
        return {"A_m": {"value": None, "sigma": None, "n": 0},
                "gyro_scale": {"value": None, "sigma": None, "n": 0}, "pts": []}
    x = np.array([p["x"] for p in pts])
    y = np.array([p["y"] for p in pts])
    if len(pts) >= 3 and np.ptp(x) > 1e-6:
        k, b0 = np.polyfit(x, y, 1)          # y = k x + b0 → s = b0, A = k/s
        resid = y - (k * x + b0)
        sig = float(np.std(resid, ddof=1)) if len(y) > 2 else None
        s_hat = float(b0)
        a_hat = float(k / b0) if abs(b0) > 1e-6 else None
        sig_a = (abs(sig / b0 / max(np.std(x), 1e-6)) if (sig is not None and b0) else None)
    else:
        s_hat = 1.0                           # 표본이 모자라면 자이로 스케일 1 로 두고 A 만
        a_hat = float(np.median((y - 1.0) / np.maximum(x, 1e-6)))
        sig_a = None
    why = None
    if a_hat is None or not np.isfinite(a_hat):
        why = "적합 실패"
    elif abs(a_hat) > ARM_PRIOR_RANGE_M:
        why = "|A| %.2f m 가 사전범위 ±%.1f m 밖 — 적합이 깨졌다" % (a_hat, ARM_PRIOR_RANGE_M)
    elif sig_a is None:
        why = "σ 를 못 냈다(표본 %d) — 측정으로 안 친다" % len(pts)
    else:
        # plan 1-1 통과기준: 5회 중 4회 같은 부호 ∧ 산포 ≤ 0.1 m
        ratios = [(p["y"] - s_hat) / max(p["x"], 1e-6) / max(s_hat, 1e-6) for p in pts]
        same = max(sum(1 for r in ratios if r > 0), sum(1 for r in ratios if r < 0))
        if len(ratios) >= 5 and same < 0.8 * len(ratios):
            why = "부호가 %d/%d 만 같다 — 통과기준(4/5) 미달" % (same, len(ratios))
        elif len(ratios) > 1 and float(np.std(ratios, ddof=1)) > 0.1:
            why = "산포 %.2f m > 0.1 m — 통과기준 미달" % float(np.std(ratios, ddof=1))
    if why:
        return {"A_m": {"value": None, "sigma": None, "n": len(pts), "why": why,
                        "raw_value": a_hat},
                "gyro_scale": {"value": None, "sigma": None, "n": len(pts), "why": why},
                "pts": pts}
    return {"A_m": {"value": a_hat, "sigma": sig_a, "n": len(pts)},
            "gyro_scale": {"value": s_hat, "sigma": None, "n": len(pts)},
            "pts": pts}


# ═══════════════════════════════════════════════════════════════════════════
# 3) 저속 97 — v97·stiction·데드존
# ═══════════════════════════════════════════════════════════════════════════

def analyze_creep(stages):
    v97, onsets, fails = [], [], 0
    dead = {}
    for st in stages:
        frames = [r for r in st.frames if r.get("t_capture") is not None]
        for seg in frame_segments(frames):
            if seg["movement"] != "forward_slow":
                continue
            body = [r for r in seg["rows"] if r.get("forward") is not None]
            if len(body) < 6:
                continue
            t = _arr(body, "t_capture")
            x = _arr(body, "forward")
            t0 = float(t[0])
            late = t >= t0 + 1.0                    # 죽은시간 뒤
            sl = slope(t[late], x[late]) if late.sum() >= 3 else None
            v = None if sl is None else -sl
            if v is not None and v > V_MIN:
                v97.append(v)
                moved = np.abs(x - x[0]) > 0.03
                if moved.any():
                    onsets.append(float(t[np.argmax(moved)] - t0))
                else:
                    fails += 1
            else:
                fails += 1
        for e in st.events:
            if e.get("event") == "creep_sweep" and e.get("phase") == "end":
                cmd = e.get("cmd") or {}
                moved = None
                if cmd.get("forward_at_stop") is not None and cmd.get("forward_after") is not None:
                    moved = abs(cmd["forward_at_stop"] - cmd["forward_after"]) > 0.02
                dead.setdefault(e.get("bias"), []).append(bool(moved))
    deadzone = None
    if dead:
        moved_levels = sorted(b for b, v in dead.items() if b is not None and any(v))
        deadzone = moved_levels[0] if moved_levels else None
    n = len(v97) + fails
    return {"v97": {"value": median(v97), "n": len(v97)},
            "onset_s": median(onsets),
            "stiction_ok": (None if n == 0 else (fails == 0 and
                                                 (max(onsets) < 2.0 if onsets else False))),
            "fails": fails, "n": n, "deadzone_level": deadzone, "dead": dead}


# ═══════════════════════════════════════════════════════════════════════════
# 4) 격자 — heading 바이어스 vs tilt → θ_min
# ═══════════════════════════════════════════════════════════════════════════

def analyze_grid(stages):
    cells = []
    for st in stages:
        pend = {}
        for e in st.events:
            if e.get("event") != "grid_cell":
                continue
            if e.get("phase") == "start":
                pend = e
            elif e.get("phase") == "end":
                # 진값은 **실측**(줄자 기준 ψ_0 + 자이로 누적)이다. 명령값(heading_cmd,
                # 0/±5/±15)은 사람이 눈으로 맞춘 값이라 오차가 재려는 바이어스(1~2°)보다
                # 크다 → 폴백으로만 쓰고 그 사실을 셀에 박아 둔다.
                truth = e.get("truth_deg")
                if truth is None:
                    truth = pend.get("truth_deg")
                src = "measured"
                if truth is None:
                    truth = pend.get("heading_cmd")
                    src = "commanded"        # 믿을 수 없는 진값
                if truth is None or e.get("heading_deg") is None:
                    continue
                cells.append({"truth_deg": float(truth),
                              "truth_source": pend.get("truth_source") or src,
                              "tape_check_deg": pend.get("tape_check_deg"),
                              "measured_deg": float(e["heading_deg"]),
                              "bias_deg": float(e["heading_deg"]) - float(truth),
                              "tilt_deg": e.get("tilt_deg"),
                              "d": e.get("forward"), "n": e.get("n")})
    theta_min = None
    shaky = [c for c in cells if c.get("truth_source") == "commanded"]
    warn = None
    if shaky:
        warn = ("grid 진값이 **명령값(눈대중)** 인 셀 %d 개 — 재려는 바이어스가 1~2°인데 "
                "사람의 각도 오차가 그보다 크다. heading_bias·θ_min 을 믿으면 안 된다. "
                "grid 를 다시 돌려라(줄자 d_L·d_R 기준 1회 + 차가 스스로 회전)." % len(shaky))
    if cells and not shaky:
        # θ_min = "그 위에서는 바이어스가 허용치 아래" 인 경계.
        # bad 셀의 최대 tilt **위의 첫 good 셀** 을 쓴다. bad 가 하나도 없으면
        # 경계를 못 잰 것이므로 **None**(소비자가 기본 15°를 쓴다) — 0.0 을 유효값으로
        # 저장하면 정면 원뿔 하드 게이트가 통째로 꺼진다(heading.py 가 0.0 을 그대로 받는다).
        bad, good = [], []
        for c in cells:
            d = c.get("d") or 3.5
            lim = max(1.0, math.degrees(math.atan2(C.LAT_TOL_M, d)))
            tilt = c.get("tilt_deg")
            if tilt is None:
                continue
            (bad if abs(c["bias_deg"]) > lim else good).append(float(tilt))
        if bad:
            above = sorted(t for t in good if t > max(bad))
            theta_min = above[0] if above else max(bad)
    return {"cells": cells, "theta_min_deg": theta_min, "n": len(cells),
            "truth_warning": warn}


# ═══════════════════════════════════════════════════════════════════════════
# 5) 타이밍 — L·ε·Δ_FS·T_line·TXACK
# ═══════════════════════════════════════════════════════════════════════════

def _epsilon_for(frames, t_g, psi_g):
    """ψ_cam(t_capture) ≈ ψ_gyro(t_capture + ε) 를 맞추는 ε [s]. 못 재면 None."""
    rows = [r for r in frames
            if r.get("heading_deg") is not None and r.get("t_capture") is not None]
    if len(rows) < 10 or t_g.size < 10:
        return None, None
    tc = np.array([r["t_capture"] for r in rows], float)
    pc = np.array([r["heading_deg"] for r in rows], float)
    if np.ptp(pc) < 2.0:                 # 안 돌았으면 지연을 못 잰다
        return None, None
    best, best_rms = None, None
    for eps in np.arange(-EPS_RANGE, EPS_RANGE + 1e-9, EPS_STEP):
        pg = np.interp(tc + eps, t_g, psi_g)
        d = (pc - pc.mean()) - (pg - pg.mean())
        rms = float(np.sqrt(np.mean(d * d)))
        if best_rms is None or rms < best_rms:
            best, best_rms = float(eps), rms
    return best, best_rms


def analyze_timing(stages, all_stages=None):
    L, dfs, rows_used = [], [], 0
    eps_by_row = {}
    txack = []
    loop_p99 = []
    gaps = 0
    src = stages or all_stages or []
    for st in src:
        for r in st.frames:
            if r.get("L_ms") is not None:
                L.append(r["L_ms"])
            if r.get("delta_fs_ms") is not None:
                dfs.append(r["delta_fs_ms"])
            if r.get("gyro_gaps"):
                gaps = max(gaps, int(r["gyro_gaps"]))
            rows_used += 1
        for r in st.can:
            if r.get("dir") == "txack" and r.get("resid_ms") is not None:
                txack.append((r.get("t_rx") or r.get("ts"), r["resid_ms"]))
        for e in st.events:
            if e.get("event") == "session_close":
                lt = (e.get("loop_tick") or {}).get("p99")
                if lt:
                    loop_p99.append(lt)
        # ε: 회전 블록별로. 행(center/upper)마다 따로 봐야 T_line 이 나온다
        t_g, w_g, _bias = gyro_debiased(st)
        if t_g.size > 20:
            psi_g = np.concatenate([[0.0], np.cumsum(np.diff(t_g) * w_g[:-1])])
            blocks = {}
            for r in st.frames:
                b = r.get("block")
                if b and str(b).startswith("rot_"):
                    blocks.setdefault(b, []).append(r)
            for b, fr in blocks.items():
                eps, rms = _epsilon_for(fr, t_g, psi_g)
                if eps is None:
                    continue
                row = "upper" if "upper" in b else "center"
                rowpx = median([r.get("row_px") for r in fr])
                eps_by_row.setdefault(row, []).append((eps * 1000.0, rowpx, rms))
    eps_all = [e for v in eps_by_row.values() for (e, _r, _m) in v]
    t_line_us = None
    if "center" in eps_by_row and "upper" in eps_by_row:
        ec = median([e for e, _r, _m in eps_by_row["center"]])
        eu = median([e for e, _r, _m in eps_by_row["upper"]])
        rc = median([r for _e, r, _m in eps_by_row["center"] if r])
        ru = median([r for _e, r, _m in eps_by_row["upper"] if r])
        if None not in (ec, eu, rc, ru) and abs(rc - ru) > 20:
            t_line_us = abs((ec - eu) * 1000.0 / (rc - ru))      # ms/행 → µs/행
    txack.sort(key=lambda x: (x[0] or 0))
    first10 = [v for _t, v in txack[:10]]
    return {"L_ms": {"median": median(L), "p99": q95(L) if len(L) < 100 else
                     float(np.percentile(L, 99)), "n": len(L)},
            "epsilon_ms": {"value": median(eps_all),
                           "sigma": (float(np.std(eps_all, ddof=1))
                                     if len(eps_all) > 1 else None),
                           "n": len(eps_all)},
            "delta_fs_ms": {"value": median(dfs), "n": len(dfs)},
            "t_line_us": {"value": t_line_us, "n": len(eps_all)},
            "txack_offset_ms": {"value": median([v for _t, v in txack]),
                                "p99": q95([v for _t, v in txack]), "n": len(txack)},
            "txack_first10_max_ms": max(first10) if first10 else None,
            "loop_tick_ms": {"p99": median(loop_p99), "max": (max(loop_p99)
                                                              if loop_p99 else None),
                             "n": len(loop_p99)},
            "gyro_gaps": gaps, "frames": rows_used, "eps_by_row": eps_by_row}


# ═══════════════════════════════════════════════════════════════════════════
# 6) 정적 perception — σ_c·flip
# ═══════════════════════════════════════════════════════════════════════════

def analyze_static(stages):
    reproj, flips, n = [], 0, 0
    for st in stages:
        for r in st.frames:
            if r.get("reproj_rms_px") is not None:
                reproj.append(r["reproj_rms_px"])
            p = r.get("pnp2")
            if p and p.get("err_ratio") is not None:
                n += 1
                if p["err_ratio"] > 0.8:        # 두 해의 재투영오차가 비슷 = 구분 불가
                    flips += 1
    return {"sigma_c_px": {"value": median(reproj), "n": len(reproj)},
            "flip_rate": {"value": (flips / n) if n else None, "n": n}}


def analyze_rot_closed(stages):
    """폐루프 회전 잔차 σ_θ (L/R) + 최소 신뢰 회전각. (계약 §5.1 신설 rot_closed·min_inc.turn_deg)

    `rotate_closed()` 가 리드 0 으로 돌고 `rotate_end` 에 target/turned 를 남긴다 —
    잔차 = turned − target 이 그대로 폐루프 σ_θ 다. 9/7 은 ±2° 였고 plan 2-4 는
    **σ_θ ≤ 0.5° 를 요건**으로 못 박았다(블라인드 1.28 m × sin 2° = 4.5 cm > 3 cm).
    """
    out = {"L": [], "R": [], "small": []}
    for st in stages:
        for e in st.events:
            if e.get("event") != "rotate_end":
                continue
            if e.get("reason") != "goal":       # cap·자이로 사망은 잔차가 아니다
                continue
            tgt, got = e.get("target_deg"), e.get("turned_deg")
            if tgt is None or got is None or abs(float(tgt)) < 0.5:
                continue
            side = e.get("dir") or ("L" if float(tgt) > 0 else "R")
            resid = float(got) - float(tgt)
            out.setdefault(side, []).append(resid)
            if abs(float(tgt)) <= 6.0:          # 소각(5°) 일관성 — 최소 증분 판정용
                out["small"].append(resid)
    res = {}
    for side in ("L", "R"):
        v = out.get(side) or []
        res[side] = {"sigma_deg": (float(np.std(v, ddof=1)) if len(v) > 1 else None),
                     "mean_deg": median(v), "n": len(v)}
    sig = [res[s2]["sigma_deg"] for s2 in ("L", "R") if res[s2]["sigma_deg"] is not None]
    # 최소 신뢰 회전각 = 2σ (그 밑은 잔차에 묻힌다). 못 재면 None — 지어내지 않는다
    res["turn_deg"] = (2.0 * max(sig)) if sig else None
    res["small_n"] = len(out["small"])
    return res


def analyze_min_inc(stages):
    """최소 신뢰 직진 증분 δ_x (67/97). plan 2-1 "CV < 30% 인 가장 짧은 명령".

    명령 길이별로 실제 이동거리를 모아 CV 를 본다. 버킷이 모자라면 **None** 을
    돌려준다 — Dynamics 가 가정값(0.30/0.12)을 쓰고 그 사실을 화면에 띄운다.
    숫자를 지어내면 그게 실측처럼 캘리브에 박힌다.
    """
    per = {67: {}, 97: {}}
    for st in stages:
        frames = [r for r in st.frames if r.get("t_capture") is not None]
        for seg in frame_segments(frames):
            mv = seg["movement"]
            lvl = LEVEL_OF.get(mv)
            if lvl is None or DIR_OF.get(mv) != "fwd":
                continue
            rows = [r for r in seg["rows"] if r.get("forward") is not None]
            if len(rows) < 4:
                continue
            sec = float(rows[-1]["t_capture"]) - float(rows[0]["t_capture"])
            dist = abs(float(rows[0]["forward"]) - float(rows[-1]["forward"]))
            if sec <= 0.2 or dist <= 0.0:
                continue
            per[lvl].setdefault(round(sec * 2.0) / 2.0, []).append(dist)
    out = {}
    for lvl, buckets in per.items():
        best, rejected = None, False
        for T in sorted(buckets):
            v = buckets[T]
            if len(v) < 3:
                continue
            m = float(np.median(v))
            cv = float(np.std(v, ddof=1)) / m if m > 1e-6 else 9.9
            if cv < 0.30:
                best = {"value": m, "cv": cv, "T": T, "n": len(v)}
                break
            rejected = True          # 더 짧은 명령을 실제로 해 봤고 CV 가 컸다
        # **더 짧은 걸 해 보지 않았으면 "최소" 라고 말하지 않는다.** 처음 본 길이가
        # 통과했다면 그건 상한일 뿐이고, 그 값을 최소 증분으로 쓰면 상태기계가 그보다
        # 작은 보정을 영영 거부해 양자화 공백(Tier 3)으로 떨어진다. 그럴 땐 None 을
        # 내보내고 Dynamics 의 가정값(0.30 / 0.12 m)이 이기게 둔다.
        if best is not None and not rejected:
            best = {"value": None, "n": best["n"], "upper_bound_m": best["value"],
                    "why": "더 짧은 명령을 안 해 봤다 — 상한일 뿐이라 최소로 안 쓴다"}
        out["fwd_%d_m" % lvl] = best or {"value": None, "n": 0}
    return out


def analyze_beta_vis(stages):
    """가시 한계 |β| (L/R). `rotate_beta_limit` start~end 창에서 **마지막으로 보인** β.

    (계약 §5.2 신설 static.beta_vis_deg. 없으면 ±28° 가정 — HFOV 69/2 − 태그 반각)
    """
    out = {"L": [], "R": []}
    for st in stages:
        frames = [r for r in st.frames if r.get("t_capture") is not None]
        opens = {}
        for e in st.events:
            if e.get("event") != "rotate_beta_limit":
                continue
            key = (e.get("side"), e.get("trial"))
            t = e.get("ts")
            if t is None:
                continue
            if e.get("phase") == "start":
                opens[key] = float(t)
            elif e.get("phase") == "end" and key in opens:
                t0, t1 = opens.pop(key), float(t)
                seen = [abs(float(r["beta_px_deg"])) for r in frames
                        if r.get("beta_px_deg") is not None and r.get("seen")
                        and t0 <= float(r["t_capture"]) <= t1]
                if seen:
                    out.setdefault(e.get("side") or "L", []).append(max(seen))
    return {"L": {"value": median(out["L"]), "n": len(out["L"])},
            "R": {"value": median(out["R"]), "n": len(out["R"])}}


def analyze_pitch_px(stages):
    """제동 노즈다운으로 태그 상단이 **위로** 몇 px 올라가나. (계약 §5.2 신설 static.pitch_px)

    plan 3-4 의 태그 컷 중단 술어가 이 값을 뺀다. 태그 상단 행 = center_v − tag_px/2
    (행은 아래로 +) 이므로 "올라간다" = 그 값이 **줄어든다**.
    """
    rises = []
    for st in stages:
        frames = [r for r in st.frames if r.get("t_capture") is not None]
        for t_stop in [t for t, mv in command_timeline(st) if mv == "stop"]:
            win = [r for r in frames
                   if r.get("center_px") and r.get("tag_px")
                   and t_stop - 0.4 <= float(r["t_capture"]) <= t_stop + 1.5]
            if len(win) < 5:
                continue
            top = [float(r["center_px"][1]) - 0.5 * float(r["tag_px"]) for r in win]
            base = float(np.median(top[:3]))          # 제동 직전
            rises.append(max(0.0, base - min(top)))
    if not rises:
        return {"value": None, "n": 0}
    return {"value": float(q95(rises)), "median": median(rises), "n": len(rises)}


def analyze_mount(stages):
    """줄자 입력(`mount_measured`) → 캘리브로 **옮긴다**. 여태 아무도 안 옮기고 있었다.

    x_off_m(오른쪽 +)·roll·h_tag_cam(= 태그높이 − 카메라높이)·δ̂ 넷이 여기서 나온다.
    """
    out = {}
    for st in stages:
        for e in st.events:
            if e.get("event") == "mount_measured":
                out.update({k: v for k, v in e.items()
                            if k not in ("event", "ts", "time", "stage") and v is not None})
    if not out:
        return {}
    h = None
    if out.get("tag_height_m") is not None and out.get("cam_height_m") is not None:
        h = float(out["tag_height_m"]) - float(out["cam_height_m"])
    return {"raw": out,
            "x_off_m": out.get("x_off_m"),
            "roll_deg": out.get("cam_roll_deg"),
            "pitch_deg": out.get("cam_pitch_deg"),
            "cam_yaw_offset_deg": out.get("cam_yaw_deg"),
            "h_tag_cam_m": h,
            "cam_to_ref_m": out.get("cam_to_ref_m")}


def analyze_tagcut(stages):
    cuts = []
    for st in stages:
        for e in st.events:
            if e.get("event") == "tagcut" and e.get("cut"):
                cuts.append(e["cut"])
    if not cuts:
        return {"forward_m": None, "margin_px": None, "n": 0}
    return {"forward_m": median([c.get("forward") for c in cuts]),
            "margin_px": median([c.get("margin_px") for c in cuts]),
            "n": len(cuts)}


# ═══════════════════════════════════════════════════════════════════════════
# 캘리브 만들기
# ═══════════════════════════════════════════════════════════════════════════

def build_calibs(root, stages, gate_failed=False):
    fake = any(s.fake for s in stages)
    src = "first_run %s" % os.path.basename(os.path.abspath(root))
    timing = CAL.blank("timing", source=src)
    percep = CAL.blank("perception", source=src)
    dyn = CAL.blank("dynamics", source=src)

    T = analyze_timing(stages_named(stages, "timing"), stages)
    _eps, _eps_why = eps_usable(T)
    F = analyze_forward(stages_named(stages, "forward") + stages_named(stages, "creep")
                        + stages_named(stages, "oblique"), eps_s=_eps)
    R = analyze_rotate(stages_named(stages, "rotate") + stages_named(stages, "timing"))
    A = analyze_arm(stages_named(stages, "rotate"))
    G = analyze_grid(stages_named(stages, "grid") + stages_named(stages, "mount"))
    K = analyze_creep(stages_named(stages, "creep"))
    S = analyze_static(stages_named(stages, "grid") + stages_named(stages, "mount")
                       + stages_named(stages, "timing"))
    X = analyze_tagcut(stages_named(stages, "tagcut"))
    RC = analyze_rot_closed(stages_named(stages, "rotate") + stages_named(stages, "timing"))
    MI = analyze_min_inc(stages_named(stages, "forward") + stages_named(stages, "creep"))
    BV = analyze_beta_vis(stages_named(stages, "rotate"))
    PP = analyze_pitch_px(stages_named(stages, "forward") + stages_named(stages, "creep"))
    MT = analyze_mount(stages_named(stages, "mount"))

    # -- timing --
    timing["L_ms"] = dict(T["L_ms"])
    timing["epsilon_ms"] = T["epsilon_ms"]
    # 명령 지연 = 우리가 명령을 바꾼 시각 → CAN 버스에 실제로 나간 시각.
    # 출발 지연(tau_start_s)과 **다른 것**이다: 이건 소프트웨어+드라이버, 저건 차량 기계.
    timing["cmd_latency_ms"] = cmd_latency_ms(stages)
    T["cmd_latency"] = timing["cmd_latency_ms"]
    timing["delta_fs_ms"] = T["delta_fs_ms"]
    timing["t_line_us"] = T["t_line_us"]
    timing["txack_offset_ms"] = T["txack_offset_ms"]
    timing["loop_tick_ms"] = T["loop_tick_ms"]
    timing["gyro"] = {"gaps": T["gyro_gaps"], "hz": None, "n": T["frames"]}
    timing["gate"] = {"txack_first10_max_ms": T["txack_first10_max_ms"],
                      "gyro_gaps": T["gyro_gaps"],
                      "passed": bool(not gate_failed
                                     and T["gyro_gaps"] == 0
                                     and T["txack_first10_max_ms"] is not None
                                     and T["txack_first10_max_ms"] < 20.0)}

    # -- perception --
    if G["theta_min_deg"] is not None and G["n"]:
        percep["static"]["theta_min_deg"] = {"value": float(G["theta_min_deg"]),
                                             "n": G["n"]}
    percep["static"]["heading_bias_deg"] = G["cells"]
    percep["static"]["sigma_c_px"] = S["sigma_c_px"]
    percep["static"]["flip_rate"] = S["flip_rate"]
    percep["dynamic"]["A_m"] = A["A_m"]
    percep["dynamic"]["gyro_scale"] = A["gyro_scale"]
    percep["dynamic"]["epsilon_ms"] = T["epsilon_ms"]
    # -- 신설 (계약 §5.2). 없으면 키를 안 만든다 → 소비자가 "없을 때 가정값" 을 쓴다
    if BV["L"]["value"] is not None or BV["R"]["value"] is not None:
        percep["static"]["beta_vis_deg"] = {
            "L": BV["L"]["value"], "R": BV["R"]["value"],
            "n": BV["L"]["n"] + BV["R"]["n"]}
    if PP["value"] is not None:
        percep["static"]["pitch_px"] = PP
    if MT:
        if MT.get("roll_deg") is not None:
            percep["static"]["roll_deg"] = {"value": float(MT["roll_deg"]),
                                            "source": "줄자(mount)", "n": 1}
        for key, path in (("x_off_m", "x_off_m"), ("h_tag_cam_m", "h_tag_cam_m"),
                          ("cam_yaw_offset_deg", "cam_yaw_offset_deg")):
            if MT.get(key) is not None:
                percep["dynamic"][path] = {"value": float(MT[key]),
                                           "source": "줄자(mount)", "n": 1}
        percep["mount_raw"] = MT.get("raw")

    # -- dynamics --
    for cell, v in F["cells"].items():
        dyn["tau_eff_s"][cell] = {"value": v["value"], "sigma_e_m": v["sigma_e_m"],
                                  "q95_m": v["q95_m"], "n": v["n"]}
        lvl = cell.split("_")[0]
        if v.get("v_mps"):
            cur = dyn["v_mps"].get(lvl) or {"value": None, "n": 0}
            if cell.endswith("fwd"):
                dyn["v_mps"][lvl] = {"value": v["v_mps"], "n": v["n"]}
            elif cur.get("value") is None:
                dyn["v_mps"][lvl] = {"value": v["v_mps"], "n": v["n"]}
    dyn["tau_r_s"] = {"L": R["L"], "R": R["R"]}
    # **스키마 한 곳**(calib.blank) 과 같은 꼴로 쓴다. 예전엔 스칼라라 소비자 넷 중
    # 셋(predict 전진·estimate·dock_fsm)이 실측값을 못 읽고 가정값으로 돌았다.
    dyn["tau_start_s"] = {
        "fwd": {"value": (F.get("tau_start_fwd") or {}).get("value")
                if (F.get("tau_start_fwd") or {}).get("value") is not None
                else K.get("onset_s"),
                "n": (F.get("tau_start_fwd") or {}).get("n", 0)},
        "rot": {"value": R.get("tau_start_s"), "n": R.get("tau_start_n", 0)}}
    dyn["S_of_T"] = F.get("S_of_T") or {"67": [], "97": []}
    if K["v97"]["value"]:
        dyn["v_mps"]["97"] = K["v97"]
    dyn["stiction_ok"] = K["stiction_ok"]
    dyn["deadzone_level"] = K["deadzone_level"]
    dyn["tag_cut"] = X
    # -- 신설 (계약 §5.1). 못 잰 항목은 None 으로 남긴다 — 지어내지 않는다
    dyn["rot_closed"] = {"L": RC["L"], "R": RC["R"]}
    dyn["min_inc"] = {"fwd_67_m": MI.get("fwd_67_m", {}).get("value"),
                      "fwd_97_m": MI.get("fwd_97_m", {}).get("value"),
                      "turn_deg": RC["turn_deg"],
                      "detail": {"fwd": MI, "turn_n": RC["small_n"]}}

    # -- provisional 판정 --
    def firm(*items):
        return all(it and it.get("n", 0) >= N_FIRM and it.get("value") is not None
                   for it in items)

    notes = []
    if fake:
        notes.append("가짜 소스(dry-run)로 만든 기록이다 — 실주행 캘리브로 쓰면 안 된다")
    if gate_failed:
        notes.append("타이밍 게이트 미통과 — 값은 참고용")
    if G.get("truth_warning"):
        notes.append(G["truth_warning"])
    if _eps_why:
        notes.append("출발 지연: %s" % _eps_why)
    timing["provisional"] = bool(fake or gate_failed
                                 or not timing["gate"]["passed"]
                                 or T["epsilon_ms"]["n"] < 3)
    percep["provisional"] = bool(fake or gate_failed
                                 or not firm(percep["dynamic"]["A_m"]))
    dyn_firm = all(dyn["tau_eff_s"][c]["n"] >= N_FIRM for c in ("67_fwd",)) \
        and R["L"]["n"] >= 3 and R["R"]["n"] >= 3
    dyn["provisional"] = bool(fake or gate_failed or not dyn_firm)
    for c in (timing, percep, dyn):
        c["notes"] = list(notes)
        c["fake_source"] = fake
    return {"timing": timing, "perception": percep, "dynamics": dyn}, \
           {"T": T, "F": F, "R": R, "A": A, "G": G, "K": K, "S": S, "X": X,
            "RC": RC, "MI": MI, "BV": BV, "PP": PP, "MT": MT,
            "fake": fake, "gate_failed": gate_failed}


# ═══════════════════════════════════════════════════════════════════════════
# 보고서
# ═══════════════════════════════════════════════════════════════════════════

def _fmt(v, f="%.3f"):
    return "측정 부족" if v is None else (f % v if isinstance(v, (int, float)) else str(v))


def write_report(path, root, stages, calibs, an):
    L = []
    L.append("# first_run 분석 — %s" % os.path.basename(os.path.abspath(root)))
    L.append("")
    if an["fake"]:
        L.append("> **가짜 소스(dry-run)** 기록이다. 숫자는 코드 경로 검증용이지 캘리브가 아니다.")
        L.append("")
    if an["gate_failed"]:
        L.append("> **타이밍 게이트 미통과** — plan 4-2 규칙대로 값은 참고용이다.")
        L.append("")
    L.append("| 단계 | 폴더 | frame | imu | can | events |")
    L.append("|---|---|---|---|---|---|")
    for s in stages:
        L.append("| %s | %s | %d | %d | %d | %d |"
                 % (s.name, os.path.basename(s.dir), len(s.frames), len(s.imu),
                    len(s.can), len(s.events)))
    T, F, R, A, G, K, S, X = (an[k] for k in ("T", "F", "R", "A", "G", "K", "S", "X"))
    RC, MI, BV, PP, MT = (an.get(k) or {} for k in ("RC", "MI", "BV", "PP", "MT"))

    L += ["", "## 타이밍 (plan 4-1)", "",
          "| 항목 | 값 | n |", "|---|---|---|",
          "| L 중앙값 [ms] | %s | %d |" % (_fmt(T["L_ms"]["median"], "%.1f"), T["L_ms"]["n"]),
          "| L p99 [ms] | %s | |" % _fmt(T["L_ms"]["p99"], "%.1f"),
          "| ε [ms] | %s | %d |" % (_fmt(T["epsilon_ms"]["value"], "%.1f"),
                                    T["epsilon_ms"]["n"]),
          "| Δ_FS [ms] | %s | %d |" % (_fmt(T["delta_fs_ms"]["value"], "%.1f"),
                                       T["delta_fs_ms"]["n"]),
          "| T_line [µs/행] | %s | |" % _fmt(T["t_line_us"]["value"], "%.1f"),
          "| TXACK 첫10 max [ms] | %s | %d |" % (_fmt(T["txack_first10_max_ms"], "%.1f"),
                                                 T["txack_offset_ms"]["n"]),
          "| 자이로 gaps | %s | |" % T["gyro_gaps"],
          "", "통과 규칙: L ∈ [50, 150] ms ∧ |ε| ≤ 15 ms ∧ TXACK 첫10 max < 20 ms ∧ gaps = 0"]

    L += ["", "## 정지거리 τ_eff (plan 2-1)", "",
          "| 셀 | τ_eff [s] | σ_e [m] | q95 [m] | v̂ [m/s] | n |", "|---|---|---|---|---|---|"]
    for cell in ("67_fwd", "97_fwd", "67_bwd", "97_bwd"):
        v = F["cells"].get(cell)
        if v is None:
            L.append("| %s | 측정 부족 | | | | 0 |" % cell)
        else:
            L.append("| %s | %s | %s | %s | %s | %d |"
                     % (cell, _fmt(v["value"]), _fmt(v["sigma_e_m"]),
                        _fmt(v["q95_m"]), _fmt(v["v_mps"]), v["n"]))
    L.append("")
    L.append("n < %d 인 셀은 provisional 을 못 뗀다." % N_FIRM)

    L += ["", "## 회전 (plan 2-4)", "",
          "| 방향 | τ_r [s] | σ | n |", "|---|---|---|---|"]
    for side in ("L", "R"):
        v = R[side]
        L.append("| %s | %s | %s | %d |" % (side, _fmt(v["value"]),
                                            _fmt(v["sigma"]), v["n"]))
    L.append("")
    _tc = (calibs or {}).get("timing") or {}
    _dc = (calibs or {}).get("dynamics") or {}
    CL = _tc.get("cmd_latency_ms") or {}
    L += ["", "### 명령 → 움직임까지 (두 단계로 나눠 잰다)", "",
          "| 무엇 | 값 | n | 어디서 나온 것 |", "|---|---|---|---|",
          "| ① 명령 지연 (결정 → CAN 버스) | %s ms (p99 %s) | %d | 소프트웨어·드라이버 |"
          % (_fmt(CL.get("value"), "%.1f"), _fmt(CL.get("p99"), "%.1f"), CL.get("n", 0)),
          "| ② 출발 지연 전진 (버스 → 3 cm 움직임) | %s s | %d | 차량 기계 |"
          % (_fmt((_dc.get("tau_start_s") or {}).get("fwd", {}).get("value")),
             (_dc.get("tau_start_s") or {}).get("fwd", {}).get("n", 0)),
          "| ② 출발 지연 회전 (버스 → 자이로 반응) | %s s | %d | 차량 기계 |"
          % (_fmt((_dc.get("tau_start_s") or {}).get("rot", {}).get("value")),
             (_dc.get("tau_start_s") or {}).get("rot", {}).get("n", 0)),
          "",
          "①이 크면 호스트·드라이버 문제(고칠 수 있다), ②가 크면 차량 자체다(모델로 보정한다).",
          "9/7 실측은 ② 전진 ~1.0 s · 회전 ~0.85 s 였고 ①은 안 쟀다.", ""]
    L.append("출발 지연 τ_start 중앙값: %s s" % _fmt(R.get("tau_start_s")))

    L += ["", "## 회전 팔 A·자이로 스케일 s (plan 1-1)", "",
          "| 항목 | 값 | σ | n |", "|---|---|---|---|",
          "| A [m] | %s | %s | %d |" % (_fmt(A["A_m"]["value"]), _fmt(A["A_m"]["sigma"]),
                                        A["A_m"]["n"]),
          "| s | %s | | %d |" % (_fmt(A["gyro_scale"]["value"]), A["gyro_scale"]["n"]),
          "", "Δβ_px = s·(1 + A·cosβ/d)·Δψ_gyro 회귀. 정면 원뿔에서 재면 바이어스가 A 로 둔갑한다."]

    L += ["", "## heading 바이어스 vs tilt → θ_min (plan 1-3)", "",
          "| 진값 heading | 측정 | 바이어스 | tilt | d | n |", "|---|---|---|---|---|---|"]
    for c in G["cells"]:
        L.append("| %s | %s | %s | %s | %s | %s |"
                 % (_fmt(c["truth_deg"], "%.1f"), _fmt(c["measured_deg"], "%.2f"),
                    _fmt(c["bias_deg"], "%.2f"), _fmt(c["tilt_deg"], "%.1f"),
                    _fmt(c["d"], "%.2f"), c["n"]))
    L.append("")
    L.append("θ_min 제안: %s 도" % _fmt(G["theta_min_deg"], "%.1f"))

    L += ["", "## 저속 97 (plan 2-5)", "",
          "| 항목 | 값 |", "|---|---|",
          "| v97 [m/s] | %s (n=%d) |" % (_fmt(K["v97"]["value"]), K["v97"]["n"]),
          "| 온셋 [s] | %s |" % _fmt(K["onset_s"]),
          "| stiction_ok | %s |" % K["stiction_ok"],
          "| 무이동 횟수 | %d / %d |" % (K["fails"], K["n"]),
          "| 데드존(움직인 최저 편향) | %s |" % _fmt(K["deadzone_level"], "%.0f")]

    L += ["", "## 정적 perception / 태그 컷", "",
          "| 항목 | 값 | n |", "|---|---|---|",
          "| 재투영 RMS [px] | %s | %d |" % (_fmt(S["sigma_c_px"]["value"], "%.3f"),
                                             S["sigma_c_px"]["n"]),
          "| 두 해 구분 불가 비율 | %s | %d |" % (_fmt(S["flip_rate"]["value"], "%.3f"),
                                              S["flip_rate"]["n"]),
          "| 태그 컷 forward [m] | %s | %d |" % (_fmt(X["forward_m"]), X["n"])]

    L += ["", "## 신설 필드 (계약 §5.1·§5.2 — 2026-09-21 통합이 추가)", "",
          "| 경로 | 값 | n | 못 재면 쓰는 가정값 |", "|---|---|---|---|"]
    for side in ("L", "R"):
        rc = (RC.get(side) or {})
        L.append("| dynamics.rot_closed.%s.sigma_deg | %s | %s | 1.0 |"
                 % (side, _fmt(rc.get("sigma_deg"), "%.2f"), rc.get("n", 0)))
    L.append("| dynamics.min_inc.turn_deg | %s | %s | 2.0 |"
             % (_fmt(RC.get("turn_deg"), "%.2f"), RC.get("small_n", 0)))
    for lvl in (67, 97):
        mi = (MI.get("fwd_%d_m" % lvl) or {})
        L.append("| dynamics.min_inc.fwd_%d_m | %s | %s | %s |"
                 % (lvl, _fmt(mi.get("value")), mi.get("n", 0),
                    "0.30" if lvl == 67 else "0.12"))
    for side in ("L", "R"):
        bv = (BV.get(side) or {})
        L.append("| perception.static.beta_vis_deg.%s | %s | %s | 28.0 |"
                 % (side, _fmt(bv.get("value"), "%.1f"), bv.get("n", 0)))
    L.append("| perception.static.pitch_px | %s | %s | 24.0 |"
             % (_fmt(PP.get("value"), "%.1f"), PP.get("n", 0)))
    # 루프 변수를 path 로 두면 이 함수의 인자 path(= report.md 경로)를 덮어쓴다 —
    # 실제로 report 가 "static.roll_deg" 라는 파일로 나갔다(2026-09-21에 잡음).
    for key, dotted, dflt in (("x_off_m", "dynamic.x_off_m", "0.0"),
                              ("cam_yaw_offset_deg", "dynamic.cam_yaw_offset_deg", "0.0"),
                              ("h_tag_cam_m", "dynamic.h_tag_cam_m", "1.10"),
                              ("roll_deg", "static.roll_deg", "0.0 (σ_lat 에 19 mm 추가)")):
        L.append("| perception.%s | %s | %s | %s |"
                 % (dotted, _fmt(MT.get(key)), 1 if MT.get(key) is not None else 0, dflt))
    L.append("")
    L.append("**None 은 지어내지 않은 것이다** — 그 항목은 코드가 오른쪽 가정값으로 돌고 "
             "화면 상단에 \"가정값 사용 중\" 이 뜬다.")

    L += ["", "## 산출 캘리브", ""]
    for k, c in calibs.items():
        L.append("- **%s**: provisional=%s  (%s)" % (k, c["provisional"], c["source"]))
    L.append("")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L) + "\n")
    return path


# ═══════════════════════════════════════════════════════════════════════════
# 자기 검사 (합성 데이터)
# ═══════════════════════════════════════════════════════════════════════════

def _synth_session(dirpath, tau_eff=0.50, v=0.28, tau_r=0.18, eps_s=0.0,
                   n_runs=12, v97=0.12, seed=0,
                   t_dead_fwd=1.00, t_dead_rot=0.85, cmd_lat_s=0.008):
    """알고 있는 값으로 기록을 지어낸다. analyze 가 그 값을 되찾아야 한다."""
    rng = np.random.default_rng(seed)
    os.makedirs(dirpath, exist_ok=True)
    fw = os.path.join(dirpath, "frame.jsonl")
    ev = os.path.join(dirpath, "events.jsonl")
    cn = os.path.join(dirpath, "can.jsonl")
    im = os.path.join(dirpath, "imu.jsonl")
    F, E, N, I = [], [], [], []
    t = 1000.0
    x = 3.5
    dt = 1.0 / 30.0

    def setcmd(tt, movement):
        """명령 한 번 = set(결정) + tx(버스에 나감). 둘의 차가 **명령 지연** 진값이다."""
        N.append({"dir": "set", "movement": movement, "t_cmd_set": tt, "ts": tt})
        N.append({"dir": "tx", "movement": movement,
                  "t_cmd_tx": tt + cmd_lat_s, "ts": tt + cmd_lat_s})

    def frame(t_true, movement, forward=None, heading=None, block=None, gyro=0.0,
              tilt=20.0):
        # ε = 카메라 시계가 시스템 시계보다 얼마나 어긋났나. **모든 프레임에 똑같이**
        # 적용된다(예전 픽스처는 회전 프레임에만 넣어서, 전진 출발 지연 검사가
        # ε 만큼 통째로 틀린 것을 "정상" 으로 통과시켰다).
        t_cap = t_true - eps_s
        F.append({"t_capture": t_cap, "t_arrival": t_cap + 0.055,
                  "L_ms": 55.0, "delta_fs_ms": 20.0, "row_px": 300.0,
                  "movement": movement, "forward": forward, "heading_deg": heading,
                  "tilt_deg": tilt, "block": block, "gyro_dps": gyro,
                  "gyro_gaps": 0, "reproj_rms_px": 0.09,
                  "pnp2": {"err_ratio": 0.12}})

    # ── 직진 정속정지 ──
    for k in range(n_runs):
        setcmd(t, "forward")
        x0 = x
        for i in range(int(4.0 / dt)):
            moving = (i * dt) > t_dead_fwd
            x -= (v * dt if moving else 0.0)
            frame(t, "forward", forward=x + rng.normal(0, 0.002), heading=0.0,
                  block="forward_%d" % k)
            t += dt
        setcmd(t, "stop")
        x -= v * tau_eff                              # 코스팅
        for i in range(60):
            frame(t, "stop", forward=x + rng.normal(0, 0.002), heading=0.0,
                  block="forward_%d" % k)
            t += dt
        E.append({"event": "forward_run", "phase": "end", "trial": k,
                  "start_forward": x0})
        # 원위치 복귀(후진)
        setcmd(t, "backward")
        for i in range(int(4.0 / dt)):
            moving = (i * dt) > t_dead_fwd
            x += (v * dt if moving else 0.0)
            frame(t, "backward", forward=x, heading=0.0, block="back_%d" % k)
            t += dt
        setcmd(t, "stop")
        x += v * tau_eff
        for i in range(60):
            frame(t, "stop", forward=x, heading=0.0, block="back_%d" % k)
            t += dt

    # ── 저속 97 ──
    for k in range(10):
        setcmd(t, "forward_slow")
        for i in range(int(4.0 / dt)):
            moving = (i * dt) > t_dead_fwd
            x -= (v97 * dt if moving else 0.0)
            frame(t, "forward_slow", forward=x, heading=0.0, block="creep_%d" % k)
            t += dt
        setcmd(t, "stop")
        x -= v97 * tau_eff
        for i in range(40):
            frame(t, "stop", forward=x, heading=0.0, block="creep_%d" % k)
            t += dt
        setcmd(t, "backward")
        for i in range(int(2.0 / dt)):                # 후진 복귀 (실제 도구와 같게)
            moving = (i * dt) > t_dead_fwd
            x += (v * dt if moving else 0.0)
            frame(t, "backward", forward=x, heading=0.0, block="creep_%d" % k)
            t += dt
        setcmd(t, "stop")
        x += v * tau_eff
        for i in range(40):
            frame(t, "stop", forward=x, heading=0.0, block="creep_%d" % k)
            t += dt

    # ── 회전 (200 Hz 자이로 + 카메라 heading, ε 주입) ──
    psi = 0.0
    for k in range(6):
        side = "ccw" if k % 2 == 0 else "cw"
        sgn = 1.0 if side == "ccw" else -1.0
        setcmd(t, "rotate_%s" % side)
        w = 0.0
        t0_rot = t
        t_end = t + 3.0 + t_dead_rot
        while t < t_end:
            if (t - t0_rot) >= t_dead_rot:            # **출발 죽은시간** 뒤에 램프 시작
                w += (8.0 - w) * min(1.0, 0.005 / 0.3)
            psi += sgn * w * 0.005
            I.append({"s": "gyro", "t": t, "x": 0.0,
                      "y": -math.radians(sgn * w), "z": 0.0})
            if abs((t / dt) % 1.0) < 0.15:
                frame(t, "rotate_%s" % side, forward=3.5, heading=psi,
                      block="rot_center_%d" % k, gyro=sgn * w)
            t += 0.005
        setcmd(t, "stop")
        t_end = t + 2.0
        while t < t_end:                               # 코스팅: τ_r 로 지수 감쇠
            w *= math.exp(-0.005 / tau_r)
            psi += sgn * w * 0.005
            I.append({"s": "gyro", "t": t, "x": 0.0,
                      "y": -math.radians(sgn * w), "z": 0.0})
            if abs((t / dt) % 1.0) < 0.15:
                frame(t, "stop", forward=3.5, heading=psi,
                      block="rot_center_%d" % k, gyro=sgn * w)
            t += 0.005

    # ── 격자 (heading 바이어스: tilt 가 작을수록 크다) ──
    for truth in (0.0, 5.0, -5.0, 15.0, -15.0):
        tilt = abs(truth)
        bias = 2.5 if tilt < 10 else 0.2
        # 실제 도구와 같은 형식 — 진값은 **실측**(카메라 앵커 + 자이로 누적)이다.
        # heading_cmd 는 "가려고 한 자세" 일 뿐 진값이 아니다(analyze 가 거부한다).
        E.append({"event": "grid_cell", "phase": "start", "heading_cmd": truth,
                  "truth_deg": truth, "truth_source": "camera-pose+gyro-truth"})
        E.append({"event": "grid_cell", "phase": "end", "heading_deg": truth + bias,
                  "truth_deg": truth, "truth_source": "camera-pose+gyro-truth",
                  "tilt_deg": tilt, "forward": 3.5, "n": 30})

    # ── 회전 팔 A (Δβ/Δψ = s(1 + A cosβ/d)) ──
    A_true, s_true = 0.35, 1.0
    for k, (d, beta0) in enumerate([(3.5, 5.0), (3.5, -5.0), (6.0, 4.0),
                                    (6.0, -6.0), (3.5, 10.0)]):
        dpsi = 20.0 if k % 2 == 0 else -20.0
        ratio = s_true * (1.0 + A_true * math.cos(math.radians(beta0)) / d)
        E.append({"event": "rotate_camera", "dist": d, "trial": k,
                  "before": {"beta_px_deg": beta0, "forward": d, "heading_deg": beta0,
                             "n": 30},
                  "after": {"beta_px_deg": beta0 + ratio * dpsi, "forward": d,
                            "heading_deg": beta0 + dpsi, "n": 30},
                  "rotate": {"turned_deg": dpsi}})

    E.append({"event": "session_close", "loop_tick": {"p99": 12.0, "max": 40.0}})
    for t_ack in range(10):
        N.append({"dir": "txack", "can_id": 0x1E3, "resid_ms": 1.5 + 0.1 * t_ack,
                  "t_rx": 1000.0 + t_ack})
    for path, rows in ((fw, F), (ev, E), (cn, N), (im, I)):
        with open(path, "w", encoding="utf-8") as f:
            for r in rows:
                f.write(json.dumps(r, ensure_ascii=False) + "\n")
    with open(os.path.join(dirpath, "config.json"), "w", encoding="utf-8") as f:
        json.dump({"first_run": {"fake": False, "mode": "selftest"}}, f)
    return {"tau_eff": tau_eff, "v": v, "tau_r": tau_r, "eps_s": eps_s,
            "v97": v97, "A": A_true, "s": s_true}


def selftest():
    import shutil
    import tempfile
    ok = True
    tmp = tempfile.mkdtemp(prefix="afr_selftest_")
    try:
        for eps_inject in (0.0, 0.100):
            root = os.path.join(tmp, "s_%03d" % int(eps_inject * 1000))
            for stage in ("timing", "forward", "creep", "rotate", "grid"):
                truth = _synth_session(os.path.join(root, "20260922_0800_%s" % stage),
                                       eps_s=eps_inject, seed=1)
            stages = load_stages(root)
            calibs, an = build_calibs(root, stages)

            def chk(name, got, want, tol):
                nonlocal ok
                good = got is not None and abs(got - want) <= tol
                ok = ok and good
                print("  %-22s got %-10s want %-8s tol %-6s %s"
                      % (name, _fmt(got, "%.4f"), _fmt(want, "%.4f"),
                         _fmt(tol, "%.4f"), "OK" if good else "FAIL"))

            print("[ε 주입 %.0f ms]" % (eps_inject * 1000))
            chk("tau_eff 67_fwd", an["F"]["cells"].get("67_fwd", {}).get("value"),
                truth["tau_eff"], 0.06)
            chk("tau_eff 67_bwd", an["F"]["cells"].get("67_bwd", {}).get("value"),
                truth["tau_eff"], 0.08)
            chk("tau_eff 97_fwd", an["F"]["cells"].get("97_fwd", {}).get("value"),
                truth["tau_eff"], 0.10)
            chk("v97", an["K"]["v97"]["value"], truth["v97"], 0.01)
            chk("tau_r L", an["R"]["L"]["value"], truth["tau_r"], 0.05)
            chk("tau_r R", an["R"]["R"]["value"], truth["tau_r"], 0.05)
            chk("A", an["A"]["A_m"]["value"], truth["A"], 0.12)
            chk("gyro_scale", an["A"]["gyro_scale"]["value"], truth["s"], 0.10)
            # θ_min 은 "**그 위에서** 바이어스가 허용치 아래" 인 경계다. 합성 격자는
            # tilt 0·5 가 나쁘고(2.5°) 15 가 좋으니(0.2°) 경계는 **15** — 5 를 쓰면
            # 증거가 없는 5.1° 구간까지 heading 을 믿게 된다(예전 정의).
            chk("theta_min", an["G"]["theta_min_deg"], 15.0, 0.6)
            chk("epsilon_ms", an["T"]["epsilon_ms"]["value"], eps_inject * 1000.0, 12.0)
            chk("L median", an["T"]["L_ms"]["median"], 55.0, 1.0)
            # ── 사용자가 요구한 두 값: 명령 지연 · 실제 움직이기 시작한 시각 ──
            # 진값: 명령 지연 8 ms · 전진 죽은시간 1.00 s(+3 cm 가는 시간) · 회전 0.85 s
            chk("명령 지연 [ms]", (an["T"].get("cmd_latency") or {}).get("value"),
                8.0, 3.0)
            chk("출발 지연 전진 [s]",
                (an["F"].get("tau_start_fwd") or {}).get("value"),
                1.00 + 0.03 / truth["v"], 0.05)
            chk("출발 지연 회전 [s]", an["R"].get("tau_start_s"), 0.85, 0.06)
            if an["K"]["stiction_ok"] is not True:
                print("  stiction_ok FAIL (%s)" % an["K"]["stiction_ok"])
                ok = False
            if calibs["dynamics"]["provisional"]:
                print("  dynamics provisional=True (n 부족 판정) — 표본 %s"
                      % {k: v["n"] for k, v in an["F"]["cells"].items()})
            rp = write_report(os.path.join(root, "report.md"), root, stages, calibs, an)
            print("  report: %s" % rp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print("자기검사: %s" % ("통과" if ok else "실패"))
    return 0 if ok else 1


# ═══════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="first_run 기록 → 캘리브 3종 + report.md")
    ap.add_argument("root", nargs="?", help="세션 폴더(또는 단계 폴더)")
    ap.add_argument("--install", action="store_true",
                    help="config/ 에도 설치한다 (run.py 가 읽는 자리)")
    ap.add_argument("--out-dir", default=None, help="캘리브를 쓸 폴더(기본 = 세션 폴더)")
    ap.add_argument("--gate-failed", action="store_true",
                    help="타이밍 게이트 미통과 — 캘리브를 만들지 않는다")
    ap.add_argument("--selftest", action="store_true", help="합성 데이터 자기 검사")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.root:
        ap.error("세션 폴더를 주거나 --selftest")

    stages = load_stages(args.root)
    if not stages:
        print("!! %s 아래에 frame.jsonl 이 없다" % args.root)
        return 1
    print("  단계 %d개: %s" % (len(stages), ", ".join(s.name for s in stages)))
    calibs, an = build_calibs(args.root, stages, gate_failed=args.gate_failed)
    out_dir = args.out_dir or args.root
    rp = write_report(os.path.join(args.root, "report.md"), args.root, stages, calibs, an)
    print("  보고서: %s" % rp)

    if args.gate_failed and not an["fake"]:
        print("  !! 타이밍 게이트 미통과 — 캘리브 파일을 만들지 않는다 (plan 4-2)")
        print("     원시 기록과 report.md 만 남는다")
        return 2
    for kind, data in calibs.items():
        p = CAL.save(data, directory=out_dir)
        print("  %s → %s  (provisional=%s)" % (kind, p, data["provisional"]))
        if args.install and an["fake"]:
            # dry-run 산출물은 `fake_source: true` 라 calib.load() 가 **거부**한다.
            # 그걸 config/ 에 깔아 두면 다음날 아침 dock.py 가 "캘리브 없음" 으로
            # 출발을 거부하는데 파일은 있어서 원인을 못 찾는다.
            print("      설치 생략 — 가짜 소스(dry-run) 캘리브는 config/ 에 깔지 않는다")
        elif args.install:
            p2 = CAL.save(data, directory=CAL.CALIB_DIR)
            print("      설치 → %s" % p2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
