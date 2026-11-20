"""v5 실주행 기록에 작은 칼만 필터(src/ekf.py)를 다시 돌려 본다 — 오프라인, 기록만 읽는다(v5 파일은 안 바꾼다).

무엇을 보나 (참값이 없어서 정확도가 아니라 **일관성**만):
  ① NIS — 측정이 필터 예측에서 벗어난 양 ÷ 필터가 말한 흔들림. 평균 1 근처면 흔들림 값이 정직하다.
     PnP 방향의 NIS 가 크면 "치우침" 이 흔들림으로 설명이 안 된다는 뜻.
  ② PnP 방향 혁신 — 자이로로 이어 온 방향과 PnP 방향의 차이 [도].
  ③ 좌우 차 — 필터 좌우 대 PnP 좌우 (태그 3~5 m 판단).
  ④ 제자리 회전 뒤 좌우 변화가 회전중심 기하와 어긋난 양 — PnP 대 필터. 필터는 예측에 같은 기하를 써서
     작게 나오는 게 당연한 부분이 있다(참고용). 회전중심은 기본으로 config(calibrate 스윙 — 이 run 들과 다른 자료).

    python tools/ekf_replay.py                      # 2026-10-01 kw run 전부
    python tools/ekf_replay.py --psi first          # PnP 방향은 첫 판단에서만(기준만 잡기) — 나머지는 자이로로
    python tools/ekf_replay.py --center learned     # 회전중심을 학습 파일 값으로 (이 run 들로 맞춘 값 — 순환)
    python tools/ekf_replay.py --root <v5 run 폴더들의 부모>

숫자 출처 (새로 찍은 값 없음):
  R_β     그 run 의 출발 60초 정지(BASELINE 프레임) — 0.5초 묶음 평균의 sd
  R_d     그 run 의 baseline.json sigma_drift_distance_m, 거리² 에 비례 (v5 σ 모델: 거리 오차 ∝ 거리²)
  R_ψ     **다른** run 들에서 판단마다 (PnP 방향 − 자이로) 의 sd 를 모은 것 (자기 run 은 안 씀)
  회전    자이로 눈금 오차 √(0.0091² + 0.0047²) — 학습 파일 눈금 맞춤 0.9909 ±0.0047 (2026-10-01, CLAUDE.md)
          자이로 흐름 0.6°/60초 (2026-10-01 VM, CLAUDE.md) × 움직인 시간
          회전중심 흩어짐 ROT_CENTER_RMS_MM (그 run 의 config) × 현의 길이 2|sin(Δ/2)|
  직진    거리 = 학습 파일 정속 × (명령 쥔 시간 − 출발지연). 앞뒤 흔들림 = **다른** run 카메라 직진들에서 이 식의 잔차 RMS
          (정속·출발지연 자체는 이 날 기록으로 맞춘 값 — 같은 자료를 쓴다는 한계)
"""
import argparse
import csv
import glob
import json
import math
import os
import statistics as S
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from src import ekf as E  # noqa: E402

V5 = HERE.parent / "apriltag_v5"
DEFAULT_ROOT = V5 / "work_dirs" / "kw" / "20261001" / "run"
LEARNED = V5 / "config" / "learned" / "kw.json"

GYRO_SCALE_FIT, GYRO_SCALE_SE = 0.9909, 0.0047      # 학습 파일 눈금 맞춤 (2026-10-01) — v5 는 1.0 을 쓴다
GYRO_DRIFT_DPS = 0.6 / 60.0                          # 자이로 흐름 60초에 0.6° (2026-10-01 VM)
GYRO_WINDOW_S = 0.5                                  # 판단 시각의 자이로 = 직전 이만큼 프레임 중앙값 (v5 추정 창 E.WINDOW_S 와 같은 길이)
BLOCK_S = 0.5                                        # β 정지 흔들림을 재는 묶음 (v5 σ 기준선과 같은 0.5초 창)


def jl(p):
    out = []
    try:
        with open(p) as fh:
            for line in fh:
                try:
                    out.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    except FileNotFoundError:
        pass
    return out


class Run:
    def __init__(self, d):
        self.dir, self.name = Path(d), Path(d).name
        self.cfg = json.loads((self.dir / "config.json").read_text())
        ctl, det = self.cfg["control"], self.cfg["detection"]
        self.yaw_off = math.radians(ctl["CAM_YAW_OFFSET_DEG"] or 0.0)
        self.A, self.b = ctl["CAM_TO_ROT_CENTER_M"], ctl["ROT_CENTER_LATERAL_M"]
        self.center_rms = (ctl.get("ROT_CENTER_RMS_MM") or 0.0) / 1000.0
        self.dz = det["TAG_HEIGHT_M"] - det["CAMERA_HEIGHT_M"]
        self.frames = [f for f in jl(self.dir / "frame.jsonl") if f.get("seen") and f.get("t") is not None]
        self.ft = np.array([f["t"] for f in self.frames])
        self.events = jl(self.dir / "event.jsonl")
        bl = self.dir / "baseline.json"
        self.baseline = json.loads(bl.read_text()) if bl.is_file() else None

    def gyro_at(self, t):
        g = [f["gyro_deg"] for f in self.frames if t - GYRO_WINDOW_S <= f["t"] <= t and f.get("gyro_deg") is not None]
        return S.median(g) if g else None

    def decisions(self):
        out = []
        for e in self.events:
            fx = e.get("fix") or {}
            if e.get("event") != "decision" or not fx.get("forward_m") or fx.get("beta_deg") is None or not fx.get("distance_m"):
                continue
            g = self.gyro_at(e["t"])
            if g is None:
                continue
            psi_ok = fx.get("heading_deg") is not None and fx.get("why") == "ok"
            out.append(dict(t=e["t"], fx=fx, gyro=g, psi_ok=psi_ok, why=e.get("why") or ""))
        return out

    def motions(self, t0, t1):
        """두 판단 사이의 회전·직진 — 명령 순서대로."""
        ms = []
        for e in self.events:
            if e.get("event") not in ("rotate_end", "forward_end"):
                continue
            tc = e.get("t_cmd") or 0.0
            if not (t0 < tc < t1) or not e.get("t_settled"):
                continue
            ms.append(e)
        return sorted(ms, key=lambda e: e["t_cmd"])

    def beta_block_sd(self):
        """출발 60초 정지의 β — 0.5초 묶음 평균의 sd [도]. 정지 프레임이 모자라면 None."""
        base = [f for f in self.frames if f.get("state") == "BASELINE" and f.get("beta_deg") is not None]
        if len(base) < 20:
            return None
        t0 = base[0]["t"]
        blocks = {}
        for f in base:
            blocks.setdefault(int((f["t"] - t0) / BLOCK_S), []).append(f["beta_deg"])
        means = [S.mean(v) for v in blocks.values() if len(v) >= 3]
        return S.stdev(means) if len(means) >= 3 else None


def drive_model():
    m = json.loads(LEARNED.read_text())["straight"]["forward"]
    return m["speed_mps"], m["dead_s"]


def drive_length(e, speed, dead):
    hold = (e.get("t_stop_cmd") or 0.0) - e["t_cmd"]
    s = speed * max(0.0, hold - dead)
    return -s if e.get("movement") == "backward" else s


def drive_residuals(runs, speed, dead):
    """run 이름 → [카메라 직진의 (실제 − 예측) m]. 시간 직진은 실제를 몰라 뺀다."""
    out = {}
    for r in runs:
        res = []
        for e in r.events:
            if e.get("event") == "forward_end" and not e.get("timed") and e.get("reason") in ("predicted", "settle_timeout") \
                    and e.get("t_cmd") and e.get("t_stop_cmd"):
                res.append(abs(e["travelled_m"]) - abs(drive_length(e, speed, dead)))
        out[r.name] = res
    return out


def psi_spreads(runs):
    """run 이름 → 판단마다 (PnP 방향 − 자이로) 의 sd [도]."""
    out = {}
    for r in runs:
        v = [d["fx"]["heading_deg"] - d["gyro"] for d in r.decisions() if d["psi_ok"]]
        out[r.name] = (S.stdev(v), len(v)) if len(v) >= 3 else (None, len(v))
    return out


def pooled(others):
    num = sum((sd ** 2) * (n - 1) for sd, n in others if sd is not None and n > 1)
    den = sum(n - 1 for sd, n in others if sd is not None and n > 1)
    return math.sqrt(num / den) if den else None


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


def replay(r, args, speed, dead, sigma_s, sigma_psi_deg):
    A, b = r.A, r.b
    if args.center == "learned":
        g = json.loads(LEARNED.read_text())["gyro"]
        A, b = g["cam_to_rot_center_m"], g["rot_center_lateral_m"]
    sd_beta = r.beta_block_sd()
    if sd_beta is None or r.baseline is None:
        return None, "출발 정지(기준선) 기록 없음"
    noise = r.baseline["noise"]
    d_ref, sd_d_ref = noise["sigma_ref_distance_m"], noise["sigma_drift_distance_m"]
    R_psi = math.radians(sigma_psi_deg) ** 2
    scale_sd = math.hypot(1.0 - GYRO_SCALE_FIT, GYRO_SCALE_SE)

    def R_for(d):
        return [math.radians(sd_beta) ** 2, (sd_d_ref * (d / d_ref) ** 2) ** 2, R_psi]

    rows, f, prev = [], None, None
    for k, dec in enumerate(r.decisions()):
        fx = dec["fx"]
        z = np.array([math.radians(fx["beta_deg"]), fx["distance_m"],
                      math.radians((fx["heading_deg"] or 0.0)) + r.yaw_off])
        Rz = R_for(fx["distance_m"])
        row = dict(run=r.name, k=k, t=dec["t"], why=dec["why"][:40], pnp_L=fx["lateral_m"], pnp_F=fx["forward_m"],
                   pnp_h=fx["heading_deg"] if dec["psi_ok"] else None, gyro=dec["gyro"], motion="")
        if f is None:
            if not dec["psi_ok"]:
                continue                                   # 첫 판단은 PnP 방향이 있어야 연다
            x0 = E.from_measurement(*z, r.dz)
            J = E.from_measurement_jacobian(*z, r.dz)
            f = E.Filter(x0, J @ np.diag(Rz) @ J.T)
            row.update(init=True)
        else:
            # 두 판단 사이의 움직임 — 자이로 전체 변화를 회전들에 나눠 준다(되튐 포함), 회전이 없으면 끝에 작은 회전으로
            ms = r.motions(prev["t"], dec["t"])
            total = math.radians(dec["gyro"] - prev["gyro"])
            turned = [math.radians(e.get("turned_deg") or 0.0) for e in ms if e["event"] == "rotate_end"]
            k_scale = total / sum(turned) if turned and abs(sum(turned)) > 1e-6 else None
            parts, rest = [], total
            for e in ms:
                T = max(0.0, e["t_settled"] - e["t_cmd"])
                if e["event"] == "rotate_end":
                    dlt = math.radians(e.get("turned_deg") or 0.0) * (k_scale if k_scale is not None else 1.0)
                    rest -= dlt
                    q_psi = (scale_sd * abs(dlt)) ** 2 + math.radians(GYRO_DRIFT_DPS * T) ** 2
                    q_pos = (r.center_rms * 2.0 * abs(math.sin(dlt / 2.0))) ** 2
                    f.predict(lambda x, d_=dlt: E.rotate(x, d_, A, b, r.yaw_off), np.diag([q_pos, q_pos, q_psi]))
                    parts.append("R%+.1f" % math.degrees(dlt))
                else:
                    s = drive_length(e, speed, dead)
                    if s == 0.0:
                        continue
                    h = f.x[2] - r.yaw_off
                    u = np.array([math.sin(h), -math.cos(h)])          # 앞뒤 방향 (L, F)
                    Qp = sigma_s ** 2 * np.outer(u, u)
                    Q = np.zeros((3, 3))
                    Q[:2, :2] = Qp
                    Q[2, 2] = math.radians(GYRO_DRIFT_DPS * T) ** 2
                    f.predict(lambda x, s_=s: E.drive(x, s_, 0.0, r.yaw_off), Q)
                    parts.append("D%+.2f%s" % (s, "t" if e.get("timed") else ""))
            if abs(rest) > 1e-9:                                # 회전 명령 없이 자이로가 움직인 몫(되튐·흐름)
                f.predict(lambda x, d_=rest: E.rotate(x, d_, A, b, r.yaw_off),
                          np.diag([0.0, 0.0, (scale_sd * abs(rest)) ** 2]))
            row["motion"] = " ".join(parts)
            prior_x, prior_P = f.x.copy(), f.P.copy()
            use_psi = dec["psi_ok"] and args.psi == "all"
            out = f.update(z, Rz, r.dz, use=(True, True, use_psi))
            row.update(prior_L=prior_x[0], prior_F=prior_x[1], prior_h=math.degrees(prior_x[2] - r.yaw_off),
                       nu_beta_deg=math.degrees(out["nu"][0]), nu_d_mm=out["nu"][1] * 1e3,
                       nu_psi_deg=math.degrees(out["nu"][2]) if dec["psi_ok"] else None,
                       nis_beta=out["nis"][0], nis_d=out["nis"][1], nis_psi=out["nis"][2] if dec["psi_ok"] else None,
                       psi_used=use_psi)
        row.update(L=f.x[0], F=f.x[1], h=math.degrees(f.x[2] - r.yaw_off),
                   sL=math.sqrt(f.P[0, 0]), sF=math.sqrt(f.P[1, 1]), sh=math.degrees(math.sqrt(f.P[2, 2])))
        rows.append(row)
        prev = dec
    info = dict(sd_beta_deg=sd_beta, sd_d_ref_m=sd_d_ref, d_ref_m=d_ref, R_psi_deg=sigma_psi_deg, sigma_s_m=sigma_s,
                center=dict(A=A, b=b, rms_m=r.center_rms, source=args.center), yaw_off_deg=math.degrees(r.yaw_off), dz_m=r.dz)
    return rows, info


def rotation_consistency(rows, run):
    """제자리 회전만 있었던 판단 쌍 — 좌우 변화가 회전중심 기하(config 값)와 어긋난 양 [m]: PnP · 필터."""
    out = []
    for a, b in zip(rows, rows[1:]):
        if not b["motion"] or any(p[0] != "R" for p in b["motion"].split()):
            continue
        if a.get("pnp_h") is None or b.get("pnp_h") is None:
            continue
        turn = math.radians(sum(float(p[1:]) for p in b["motion"].split()))
        h = math.radians(a["h"])
        x = np.array([a["L"], a["F"], h + run.yaw_off])
        exp = E.rotate(x, turn, run.A, run.b, run.yaw_off)[0] - a["L"]
        out.append(((b["pnp_L"] - a["pnp_L"]) - exp, (b["L"] - a["L"]) - exp))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", default=str(DEFAULT_ROOT), help="v5 run 폴더들의 부모 (기본 2026-10-01 kw)")
    ap.add_argument("--psi", choices=("all", "first"), default="all", help="PnP 방향을 판단마다 넣나(all), 첫 판단에서만(first)")
    ap.add_argument("--center", choices=("config", "learned"), default="config")
    ap.add_argument("--out", default=None, help="결과 폴더 (기본 apriltag_v6/work_dirs/ekf_replay/<시각>)")
    args = ap.parse_args()

    runs = []
    for d in sorted(glob.glob(os.path.join(args.root, "*"))):
        if os.path.isfile(os.path.join(d, "config.json")) and os.path.isfile(os.path.join(d, "event.jsonl")):
            runs.append(Run(d))
    if not runs:
        raise SystemExit("run 기록이 없다: %s" % args.root)
    speed, dead = drive_model()
    resid = drive_residuals(runs, speed, dead)
    spreads = psi_spreads(runs)

    # 자이로 부호 확인 — 판단 사이 PnP 방향 변화와 자이로 변화가 같은 쪽이어야 이 필터의 ψ += Δ 가 맞다
    dh, dg = [], []
    for r in runs:
        ds = [d for d in r.decisions() if d["psi_ok"]]
        for a, b in zip(ds, ds[1:]):
            dh.append(b["fx"]["heading_deg"] - a["fx"]["heading_deg"])
            dg.append(b["gyro"] - a["gyro"])
    slope = sum(x * y for x, y in zip(dh, dg)) / sum(y * y for y in dg)
    print("확인: 판단 사이 PnP 방향 변화 ÷ 자이로 변화 = %.3f (%d쌍) — +1 근처여야 부호가 맞다" % (slope, len(dh)))
    if slope < 0.5:
        raise SystemExit("자이로 부호가 기록과 안 맞는다 — 중단")

    out_dir = Path(args.out) if args.out else HERE / "work_dirs" / "ekf_replay" / time.strftime("%Y%m%d_%H%M%S")
    out_dir.mkdir(parents=True, exist_ok=True)
    all_rows, summary = [], {"args": vars(args), "drive_model": dict(speed_mps=speed, dead_s=dead), "runs": {}}
    print("직진 거리 모델: 정속 %.4f m/s · 출발지연 %.3f s (학습 파일)  |  PnP 방향 흔들림(다른 run 들): %s"
          % (speed, dead, ", ".join("%s %.2f°" % (k, v[0]) for k, v in spreads.items() if v[0] is not None)))
    for r in runs:
        others_res = [x for k, v in resid.items() if k != r.name for x in v]
        sigma_s = rms(others_res)
        sigma_psi = pooled([v for k, v in spreads.items() if k != r.name])
        if sigma_psi is None or not others_res:
            print("%s: 다른 run 자료가 모자라 건너뜀" % r.name)
            continue
        rows, info = replay(r, args, speed, dead, sigma_s, sigma_psi)
        if rows is None:
            print("%s: 건너뜀 — %s" % (r.name, info))
            continue
        all_rows += rows
        upd = [x for x in rows if "nis_beta" in x]
        rc = rotation_consistency(rows, r)
        near = [x for x in rows if 3.0 <= x["F"] <= 5.0 and x.get("pnp_L") is not None]
        s = dict(info=info, decisions=len(rows), updates=len(upd),
                 nis_mean=dict(beta=S.mean([x["nis_beta"] for x in upd]) if upd else None,
                               d=S.mean([x["nis_d"] for x in upd]) if upd else None,
                               psi=S.mean([x["nis_psi"] for x in upd if x["nis_psi"] is not None]) if any(x["nis_psi"] is not None for x in upd) else None),
                 psi_innovation_rms_deg=rms([x["nu_psi_deg"] for x in upd if x["nu_psi_deg"] is not None]),
                 lateral_vs_pnp_rms_m_3to5=rms([x["L"] - x["pnp_L"] for x in near]),
                 rotation_consistency_rms_m=dict(pnp=rms([p for p, _ in rc]), filt=rms([q for _, q in rc]), n=len(rc)),
                 last=dict(F=rows[-1]["F"], sL=rows[-1]["sL"], sh=rows[-1]["sh"]))
        summary["runs"][r.name] = s
        print("\n== %s  판단 %d번 (갱신 %d) · R: β %.4f° · 거리 %.1f mm@%.1fm · PnP방향 %.2f° · 직진 앞뒤 %.3f m"
              % (r.name, len(rows), len(upd), info["sd_beta_deg"], info["sd_d_ref_m"] * 1e3, info["d_ref_m"],
                 info["R_psi_deg"], sigma_s))
        print("   NIS 평균 (1 근처면 흔들림 값이 정직): β %.2f · 거리 %.2f · PnP 방향 %s"
              % (s["nis_mean"]["beta"] or float("nan"), s["nis_mean"]["d"] or float("nan"),
                 ("%.2f" % s["nis_mean"]["psi"]) if s["nis_mean"]["psi"] is not None else "-"))
        print("   PnP 방향 − 자이로로 이어 온 방향: RMS %.2f°  |  필터 좌우 − PnP 좌우 (3~5 m): RMS %.3f m"
              % (s["psi_innovation_rms_deg"], s["lateral_vs_pnp_rms_m_3to5"]))
        if rc:
            print("   제자리 회전 %d번 뒤 좌우 변화 − 기하: PnP RMS %.3f m · 필터 RMS %.3f m (필터는 예측에 같은 기하 — 참고용)"
                  % (len(rc), s["rotation_consistency_rms_m"]["pnp"], s["rotation_consistency_rms_m"]["filt"]))
        print("   마지막 판단: 앞 %.2f m · 필터가 말하는 흔들림 좌우 ±%.3f m · 방향 ±%.2f°" % (s["last"]["F"], s["last"]["sL"], s["last"]["sh"]))

    keys = ["run", "k", "t", "motion", "why", "pnp_L", "pnp_F", "pnp_h", "gyro", "prior_L", "prior_F", "prior_h",
            "nu_beta_deg", "nu_d_mm", "nu_psi_deg", "nis_beta", "nis_d", "nis_psi", "psi_used", "L", "F", "h", "sL", "sF", "sh", "init"]
    with open(out_dir / "decisions.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for row in all_rows:
            w.writerow({k: (round(v, 6) if isinstance(v, float) else v) for k, v in row.items()})
    (out_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1, default=float))
    print("\n결과: %s (decisions.csv · summary.json)" % out_dir)


if __name__ == "__main__":
    main()
