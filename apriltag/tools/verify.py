"""자세 숫자가 **맞는지** 잰다. 보이는지가 아니라."""
import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src.config import TAG_SIZE_M as DEFAULT_TAG_SIZE  # noqa: E402
from src.models import (CameraIntrinsics, TagPipeline,      # noqa: E402
                                 intrinsics_from_hfov, pose_to_xyzrpy,
                                 tag_tilt_deg, docking_state,
                                 ASSUMED_HFOV_DEG, DEFAULT_QUAD_BLUR,
                                 RELIABLE_TILT_DEG)



# ===========================================================================
ROWS = [
    ("z",        "z",        "m",   "mm",  1000.0),
    ("distance", "distance", "m",   "mm",  1000.0),
    ("forward",  "forward",  "m",   "mm",  1000.0),
    ("lateral",  "lateral",  "m",   "mm",  1000.0),
    ("vertical", "vertical", "m",   "mm",  1000.0),
    ("heading",  "heading",  "deg", "deg", 1.0),
    ("approach", "approach", "deg", "deg", 1.0),
    ("tilt",     "tag tilt", "deg", "deg", 1.0),
    ("tag_px",   "tag px",   "px",  "px",  1.0),
    ("reproj",   "reproj rms", "px", "px", 1.0),
    ("margin",   "margin",   "-",   "-",   1.0),
]

#: --truth-* 로 받을 수 있는 것. 실측 정답은 사람이 잴 수 있는 것만 받는다.
TRUTH_FLAGS = {"z": "truth_z", "lateral": "truth_lateral", "heading": "truth_heading"}

#: 각도라서 reliable_angle 이 False 면 의미가 없는 행
ANGLE_ROWS = ("heading", "approach")


def sample_of(det, T, qual):
    """한 태그의 결과를 표의 행 키에 맞춘 평평한 dict 로 편다."""
    v = pose_to_xyzrpy(T)
    st = docking_state(T)
    return {"z": v["z"], "distance": v["distance"],
            "forward": st["forward"], "lateral": st["lateral"],
            "vertical": st["vertical"],
            "heading": st["heading_deg"], "approach": st["approach_deg"],
            "tilt": qual.get("tilt_deg", tag_tilt_deg(T)),
            "tag_px": qual.get("tag_px", float("nan")),
            "reproj": qual.get("reproj_rms_px", float("nan")),
            "margin": float(getattr(det, "decision_margin", float("nan"))),
            # 표에는 안 나가지만 신뢰도 회계에 쓴다. docking_state 와 pose_quality
            # 가 각자 판정한 값이다. 둘 다 tilt >= RELIABLE_TILT_DEG 라 보통 같다.
            "_rel_approach": bool(st["reliable_angle"]),
            "_rel_tilt": bool(qual.get("reliable_angle",
                                       tag_tilt_deg(T) >= RELIABLE_TILT_DEG)),
            "_quality_ok": bool(qual.get("ok", False)),
            "_reasons": tuple(qual.get("reasons", ()))}


def stats(vals):
    """n/mean/sd/min/max. sd 는 표본표준편차(ddof=1)다."""
    a = np.asarray([v for v in vals if v is not None and np.isfinite(v)], dtype=float)
    if a.size == 0:
        return None
    return {"n": int(a.size), "mean": float(a.mean()),
            "sd": float(a.std(ddof=1)) if a.size > 1 else 0.0,
            "min": float(a.min()), "max": float(a.max())}


# ===========================================================================

def synth_source(intr, tag_size, tilt_deg=15.0, distance=2.0, lateral=0.0,
                 vertical=0.0, roll_deg=0.0, noise=2.0, n=30, seed=0,
                 tag_id=0, px=600, pad=150):
    """정해진 자세로 태그를 합성해 (frames, T_truth) 를 돌려준다."""
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    core = cv2.aruco.generateImageMarker(dic, int(tag_id), px)
    # 흰 여백이 없으면 검출기가 태그 경계를 못 잡는다(quad 를 못 닫는다).
    full = cv2.copyMakeBorder(core, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    src_quad = np.float32([[pad, pad], [pad + px, pad],
                           [pad + px, pad + px], [pad, pad + px]])

    s = tag_size / 2.0
    obj = np.float32([[s, s, 0], [-s, s, 0], [-s, -s, 0], [s, -s, 0]])   # AT2 좌표계
    a, r = np.deg2rad(tilt_deg), np.deg2rad(roll_deg)
    Ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    Rz = np.array([[np.cos(r), -np.sin(r), 0], [np.sin(r), np.cos(r), 0], [0, 0, 1]])
    R = Ry @ Rz
    t = np.array([float(lateral), float(vertical), float(distance)])

    pts = (R @ obj.T).T + t
    if (pts[:, 2] <= 0).any():
        raise SystemExit("합성 자세가 카메라 뒤로 갔다 — --synth-distance 를 키울 것")
    uv = (intr.K @ pts.T).T
    uv = (uv[:, :2] / uv[:, 2:]).astype(np.float32)
    M = cv2.getPerspectiveTransform(src_quad, uv)
    base = cv2.warpPerspective(full, M, (intr.width, intr.height), borderValue=255)

    T_truth = np.eye(4)
    T_truth[:3, :3] = R
    T_truth[:3, 3] = t

    rng = np.random.default_rng(seed)

    def frames():
        for i in range(int(n)):
            if noise > 0:
                img = base.astype(np.float32) + rng.normal(0.0, float(noise), base.shape)
                img = np.clip(img, 0, 255).astype(np.uint8)
            else:
                img = base
            yield i, float(i) / 30.0, img
    return frames(), T_truth


# ===========================================================================

def open_pipeline(args, tag_size):
    """--source 에 맞는 (TagPipeline, truth dict) 를 연다."""
    common = dict(families=args.family, quad_blur=args.quad_blur,
                  method=args.method, min_margin=args.min_margin,
                  max_hamming=args.max_hamming,
                  )
    truth = {k: getattr(args, f) for k, f in TRUTH_FLAGS.items()
             if getattr(args, f) is not None}

    if args.source == "synth":
        intr = intrinsics_from_hfov((args.height, args.width), args.hfov)
        frames, T_truth = synth_source(
            intr, tag_size, tilt_deg=args.synth_tilt, distance=args.synth_distance,
            lateral=args.synth_lateral, vertical=args.synth_vertical,
            roll_deg=args.synth_roll, noise=args.synth_noise, n=args.frames,
            seed=args.seed, tag_id=(args.tag_id or 0))
        # 합성은 **모든 칸의 정답을 안다.** 추정과 똑같은 함수를 통과시켜
        t_all = sample_of(_NoDet(), T_truth, {})
        truth = {k: t_all[k] for k, *_ in ROWS if k in t_all and not k.startswith("_")}
        truth.pop("tag_px", None)          # 픽셀 크기는 "정답"이랄 게 없다
        truth.pop("reproj", None)
        truth.pop("margin", None)
        origin = ("SYNTHETIC (%s, 정답과 같은 값)"
                  % ("hfov=%.0fdeg" % args.hfov if args.hfov else "D435i ref"))
        pipe = TagPipeline(frames, intrinsics=intr, tag_size=tag_size,
                           label="synth (tilt=%.1fdeg d=%.2fm noise=%.1f)"
                                 % (args.synth_tilt, args.synth_distance, args.synth_noise),
                           origin=origin, **common)
        return pipe, truth

    if args.source in ("realsense", "ir"):
        stream = "color" if args.source == "realsense" else "infrared"
        pipe = TagPipeline.from_realsense(
            tag_size, stream=stream, width=args.width, height=args.height,
            fps=args.fps, ir_index=args.ir_index, **common)
        return pipe, truth

    if args.source == "bag":
        if not args.path:
            raise SystemExit("--source bag 은 --path 가 필요하다")
        if not Path(args.path).exists():
            raise SystemExit("bag 이 없다: %s" % args.path)
        return TagPipeline.from_bag(args.path, tag_size, **common), truth

    if args.source == "video":
        if not args.path:
            raise SystemExit("--source video 는 --path 가 필요하다")
        if not Path(args.path).exists():
            raise SystemExit("영상이 없다: %s" % args.path)
        return TagPipeline.from_video(args.path, tag_size, hfov=args.hfov,
                                      **common), truth

    raise SystemExit("모르는 소스: %s" % args.source)


class _NoDet:
    """합성 정답을 sample_of() 에 태우기 위한 빈 검출 자리표."""
    decision_margin = float("nan")


# ===========================================================================

def print_table(rows_stat, truth):
    """metric / n / mean / sd / min / max / truth / err / err% 한 판."""
    head = ("%-16s %4s %10s %9s %10s %10s │ %10s %11s %8s"
            % ("metric", "n", "mean", "sd", "min", "max", "truth", "err", "err%"))
    print(head)
    print("─" * len(head))
    for key, label, unit, eunit, escale in ROWS:
        st = rows_stat.get(key)
        name = "%s [%s]" % (label, unit)
        if st is None:
            print("%-16s %4d %10s" % (name, 0, "-- no sample --"))
            continue
        line = ("%-16s %4d %10.4f %9.4f %10.4f %10.4f │"
                % (name, st["n"], st["mean"], st["sd"], st["min"], st["max"]))
        if key in truth and truth[key] is not None and np.isfinite(truth[key]):
            tv = float(truth[key])
            err = st["mean"] - tv
            pct = (100.0 * err / abs(tv)) if abs(tv) > 1e-9 else float("nan")
            line += (" %10.4f %+8.2f %-2s %7s"
                     % (tv, err * escale, eunit,
                        "-" if not np.isfinite(pct) else "%+.2f%%" % pct))
        else:
            line += " %10s %11s %8s" % ("-", "-", "-")
        print(line)


def print_reliability(samples, n_seen, n_det, angle_stats):
    """각도를 믿어도 되는 프레임이 몇 %인지. 이걸 빼면 표가 거짓말을 한다."""
    n = len(samples)
    ra = sum(1 for s in samples if s["_rel_approach"])
    rt = sum(1 for s in samples if s["_rel_tilt"])
    qok = sum(1 for s in samples if s["_quality_ok"])
    pc = lambda k, d: (100.0 * k / d) if d else 0.0

    print("\nreliability")
    print("  frames seen / with detection / with pose : %d / %d / %d  (%.0f%% pose)"
          % (n_seen, n_det, n, pc(n, n_seen)))
    print("  pose_quality.ok                          : %d/%d (%.0f%%)"
          % (qok, n, pc(qok, n)))
    if qok < n:
        why = {}
        for s in samples:
            for r in s["_reasons"]:
                why[r] = why.get(r, 0) + 1
        print("      reasons: %s"
              % ", ".join("%s x%d" % (k, v) for k, v in sorted(why.items(),
                                                               key=lambda kv: -kv[1])))
    print("  reliable_angle (docking_state, tilt>=%.0fdeg) : %d/%d (%.0f%%)"
          % (RELIABLE_TILT_DEG, ra, n, pc(ra, n)))
    print("  reliable_angle (pose_quality, tilt>=%.0fdeg)  : %d/%d (%.0f%%)"
          % (RELIABLE_TILT_DEG, rt, n, pc(rt, n)))

    # 여기가 이 도구의 존재 이유 절반이다. reliable 이 아닌 프레임을 섞어
    if ra == n:
        return
    if ra == 0:
        print("  ! reliable 프레임이 하나도 없다. heading/approach 는 이 자세에서")
        print("    측정 불가다(태그가 정면에 가까워 원근 왜곡이 픽셀 이하).")
        print("    위 표의 heading/approach 평균은 **쓰지 마라** — lateral 로 조종할 것.")
        return
    print("  ! 위 표의 heading/approach 는 unreliable 프레임까지 섞은 값이다.")
    print("    reliable(tilt>=%.0fdeg) 프레임만 골라 다시 재면:" % RELIABLE_TILT_DEG)
    for key in ANGLE_ROWS:
        st = angle_stats.get(key)
        if st is None:
            print("      %-9s : (표본 없음)" % key)
        else:
            print("      %-9s : mean %+9.4f  sd %8.4f  (n=%d)"
                  % (key, st["mean"], st["sd"], st["n"]))


def append_log(path, args, meta, rows_stat, truth, rel):
    """--log CSV 에 한 줄 덧붙인다. 파일이 없으면 헤더부터 쓴다."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    row = {"time": datetime.now().isoformat(timespec="seconds"),
           "note": args.note or "",
           "source": args.source, "path": args.path or "",
           "tag_size_m": "%.4f" % meta["tag_size"],
           "tag_size_assumed": int(meta["size_assumed"]),
           "intr_origin": meta["origin"],
           "intr_assumed": int(meta["intr_assumed"]),
           "method": args.method, "quad_blur": args.quad_blur,
           "frames_seen": meta["n_seen"], "frames_detected": meta["n_det"],
           "frames_pose": meta["n_pose"]}
    for key, *_ in ROWS:
        st = rows_stat.get(key)
        for f in ("mean", "sd", "min", "max"):
            row["%s_%s" % (key, f)] = "" if st is None else "%.6f" % st[f]
    for key, *_rest in ROWS:
        tv = truth.get(key)
        st = rows_stat.get(key)
        row["%s_truth" % key] = "" if tv is None else "%.6f" % tv
        row["%s_err" % key] = ("" if (tv is None or st is None)
                               else "%.6f" % (st["mean"] - float(tv)))
    row.update({"rel_approach_frac": "%.4f" % rel["approach"],
                "rel_tilt_frac": "%.4f" % rel["tilt"],
                "quality_ok_frac": "%.4f" % rel["ok"]})

    new = not p.exists()
    with p.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if new:
            w.writeheader()
        w.writerow(row)
    print("\nlog: %s 에 1줄 덧붙임%s" % (p, " (헤더 새로 씀)" if new else ""))


# ===========================================================================

def main():
    ap = argparse.ArgumentParser(
        description="AprilTag docking pose - accuracy measurement (headless)")
    ap.add_argument("--source", default="synth",
                    choices=["synth", "realsense", "ir", "bag", "video"])
    ap.add_argument("--path", default=None, help="--source video/bag 일 때 파일 경로")
    ap.add_argument("--frames", type=int, default=30, metavar="N",
                    help="모을 프레임 수. 분포를 보려면 최소 20~30 은 있어야 한다")
    ap.add_argument("--tag-id", type=int, default=None,
                    help="이 태그만 잰다. 안 주면 프레임마다 제일 큰 태그")
    # 정답 (실측). 없으면 분포만 낸다.
    ap.add_argument("--truth-z", type=float, default=None, metavar="M",
                    help="줄자로 잰 광축방향 거리 [m]. 태그면 수직거리라면 forward 와 비교할 것")
    ap.add_argument("--truth-lateral", type=float, default=None, metavar="M",
                    help="태그 정면축에서 좌우로 벗어난 거리 [m]. 태그 왼쪽이 +")
    ap.add_argument("--truth-heading", type=float, default=None, metavar="DEG",
                    help="진입방향과 태그축이 이루는 각 [deg]")
    ap.add_argument("--max-err-mm", type=float, default=None, metavar="MM",
                    help="길이 오차가 이걸 넘으면 exit 1. 회귀 게이트로 쓸 때")
    ap.add_argument("--max-err-deg", type=float, default=None, metavar="DEG",
                    help="각도 오차가 이걸 넘으면 exit 1")
    ap.add_argument("--log", default=None, metavar="PATH", help="CSV 한 줄 덧붙일 파일")
    ap.add_argument("--note", default=None, help="--log 줄에 달아 둘 메모 (예: '3m 역광')")
    # 합성 소스
    ap.add_argument("--synth-tilt", type=float, default=15.0, metavar="DEG",
                    help="합성 태그 기울기. 10도 미만이면 각도는 원래 못 잰다")
    ap.add_argument("--synth-distance", type=float, default=2.0, metavar="M")
    ap.add_argument("--synth-lateral", type=float, default=0.0, metavar="M")
    ap.add_argument("--synth-vertical", type=float, default=0.0, metavar="M")
    ap.add_argument("--synth-roll", type=float, default=0.0, metavar="DEG")
    ap.add_argument("--synth-noise", type=float, default=2.0, metavar="LEVELS",
                    help="가우시안 잡음 sd [그레이레벨]. 0 이면 N 장이 전부 같아 sd=0 이다")
    ap.add_argument("--seed", type=int, default=0)
    # 카메라 값 / 검출기
    ap.add_argument("--tag-size", type=float, default=None,
                    help="태그 한 변 [m]. 안 주면 %.2f 로 가정한다 — 거리가 여기 정비례한다"
                         % DEFAULT_TAG_SIZE)
    ap.add_argument("--hfov", type=float, default=None)
    # 실카메라는 지원 해상도만 받는다(D435i 컬러에 1280x960 은 없다). config 를 따른다.
    ap.add_argument("--width", type=int, default=None, help="가로. 안 주면 config.COLOR_SIZE")
    ap.add_argument("--height", type=int, default=None, help="세로. 안 주면 config.COLOR_SIZE")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--ir-index", type=int, default=1)
    ap.add_argument("--family", default="tag36h11")
    ap.add_argument("--quad-blur", type=float, default=DEFAULT_QUAD_BLUR)
    ap.add_argument("--min-margin", type=float, default=0.0)
    ap.add_argument("--max-hamming", type=int, default=0)
    ap.add_argument("--method", default="auto", choices=["auto", "tag", "pnp"])
    args = ap.parse_args()
    if args.width is None or args.height is None:
        from src.config import COLOR_SIZE
        args.width, args.height = COLOR_SIZE

    tag_size = args.tag_size if args.tag_size else DEFAULT_TAG_SIZE
    size_assumed = args.tag_size is None

    try:
        pipe, truth = open_pipeline(args, tag_size)
    except SystemExit:
        raise
    except Exception as exc:
        raise SystemExit("소스를 열지 못했다 (%s): %s: %s"
                         % (args.source, type(exc).__name__, exc))

    samples = []
    n_seen = n_det = 0
    t0 = time.perf_counter()
    try:
        for i, _ts, img in pipe.frames:
            if img is None or img.size == 0:
                continue
            n_seen += 1
            res = pipe.process(img, index=i)
            if res.detections:
                n_det += 1
            p = res.primary(args.tag_id)
            if p is not None:
                samples.append(sample_of(p["detection"], p["T"], p["quality"] or {}))
            if n_seen >= args.frames:
                break
    except KeyboardInterrupt:
        print("(중단됨 — 여기까지 모은 것으로 낸다)")
    finally:
        pipe.close()
    elapsed = time.perf_counter() - t0

    # ----------------------------------------------------------------- 머리말
    print("source     : %s" % (pipe.label or args.source))
    print("intrinsics : %s%s" % (pipe.origin or "-",
                                 "   ** ASSUMED - 거리 전체가 의심스럽다 **"
                                 if pipe.intrinsics_assumed else ""))
    if pipe.intr is not None:
        print("             fx=%.1f fy=%.1f cx=%.1f cy=%.1f  (%dx%d)"
              % (pipe.intr.fx, pipe.intr.fy, pipe.intr.cx, pipe.intr.cy,
                 pipe.intr.width, pipe.intr.height))
    print("tag size   : %.3f m%s" % (tag_size,
                                     "   ** ASSUMED - 거리가 여기 정비례한다 **"
                                     if size_assumed else "   (user --tag-size)"))
    print("detector   : %s  quad_blur=%.1f  method=%s"
          % (args.family, args.quad_blur, args.method))
    print("frames     : %d 요청 / %d 처리 / %d 검출 / %d 자세  (%.1fs)"
          % (args.frames, n_seen, n_det, len(samples), elapsed))
    print("")

    if not samples:
        print("자세가 나온 프레임이 하나도 없다. 잴 것이 없다.")
        print("  - 태그가 화면에 있나 (tools/live_pose.py 로 눈으로 먼저 볼 것)")
        return 1

    rows_stat = {k: stats([s[k] for s in samples]) for k, *_ in ROWS}
    print_table(rows_stat, truth)

    rel_samples = [s for s in samples if s["_rel_approach"]]
    angle_stats = {k: stats([s[k] for s in rel_samples]) for k in ANGLE_ROWS}
    n = len(samples)
    rel = {"approach": len(rel_samples) / n,
           "tilt": sum(1 for s in samples if s["_rel_tilt"]) / n,
           "ok": sum(1 for s in samples if s["_quality_ok"]) / n}
    print_reliability(samples, n_seen, n_det, angle_stats)

    if not truth:
        print("\n정답이 없어 분포만 냈다. --truth-z/--truth-lateral/--truth-heading 을 주거나")
        print("--source synth 로 돌리면 오차까지 나온다.")

    if args.log:
        append_log(args.log, args,
                   {"tag_size": tag_size, "size_assumed": size_assumed,
                    "origin": pipe.origin or "", "intr_assumed": pipe.intrinsics_assumed,
                    "n_seen": n_seen, "n_det": n_det, "n_pose": n},
                   rows_stat, truth, rel)

    # ----------------------------------------------------------------- 게이트
    bad = []
    for key, label, unit, eunit, escale in ROWS:
        st, tv = rows_stat.get(key), truth.get(key)
        if st is None or tv is None or not np.isfinite(tv):
            continue
        err = abs(st["mean"] - float(tv)) * escale
        lim = (args.max_err_mm if eunit == "mm" else
               args.max_err_deg if eunit == "deg" else None)
        if lim is not None and err > lim:
            bad.append("%s: |err| %.2f %s > %.2f %s" % (label, err, eunit, lim, eunit))
    if bad:
        print("\nFAIL — 오차가 한계를 넘었다")
        for b in bad:
            print("  %s" % b)
        return 1
    if (args.max_err_mm is not None or args.max_err_deg is not None) and truth:
        print("\nPASS — 정답과 비교한 모든 값이 한계 안이다")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
