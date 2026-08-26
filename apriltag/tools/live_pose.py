"""도킹용 AprilTag 자세를 실시간 화면으로 본다.

**지금 카메라가 무엇을 보고 있는지** 한 화면에서 본다.
왼쪽은 검출/자세를 그린 영상, 오른쪽은 숫자 패널이다.

RealSense 가 아직 WSL2 로 넘어오지 않았으므로 카메라 없이도 돌아가야 한다.
그래서 입력을 네 갈래로 열어둔다.

    --source realsense          RealSense 컬러 (기본)
    --source ir                 RealSense 적외선. 글로벌 셔터, 이미터 자동 off
    --source bag --path X.bag   녹화한 .bag 재생 — 매번 완전히 같은 입력
    --source video --path X     영상 파일 — 오늘 검증은 이걸로 한다

(예전에 있던 `--source cam`(일반 웹캠)은 뺐다. 내부 파라미터가 없는 입력이라
숫자가 전부 ASSUMED 로 나오는데, 그 경로는 --source video 로 이미 덮인다.

이 도구는 **보여주기만 한다.** 숫자를 만드는 것은 src/models/tag_pose.py 의
TagPipeline 이고, 여기서는 프레임을 넣고 나온 Result 를 그리거나 찍을 뿐이다.
정확도를 재려면 이 도구가 아니라 tools/verify.py 다 — 눈으로 보는 것과
숫자가 맞는지 재는 것은 다른 일이라 도구를 갈랐다.

패널 글자는 전부 영어다. cv2 의 Hershey 폰트에 한글 글리프가 없어서
한글을 쓰면 네모로 깨진다. 주석/독스트링만 한글로 둔다.

믿으면 안 되는 값은 눈에 띄게 표시한다.
    - 내부 파라미터를 화각 가정으로 만들었을 때        -> ASSUMED 경고 + 거리값에 '?'
    - 태그 크기가 사용자 입력이 아니라 기본값일 때     -> ASSUMED 경고 + 거리값에 '?'
    - docking_state()['reliable_angle'] 이 False 일 때 -> 각도값에 '?' + 색 변경
거리는 fx 와 태그 크기에 그대로 비례하므로, 둘 중 하나가 가정이면 거리도 가정이다.

키
    q / ESC   종료        a  좌표축<->큐브
    s         현재 화면 저장 (work_dirs/run/)
    r         fps 카운터 초기화
    SPACE     일시정지 / 재개

사용법
    python tools/live_pose.py                                   # RealSense 컬러
    python tools/live_pose.py --source ir
    python tools/live_pose.py --source video --path a.mp4 --tag-size 0.20
    python tools/live_pose.py --source video --path a.mp4 --headless 5   # 창 없이 검증

합성 테스트 영상(Testing_apriltag*.mp4)은 디버그 그래픽이 태그를 덮고 있어서
실촬영에는 쓰지 마라.
"""
import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src.models.tag_pose import (CameraIntrinsics, TagPipeline,      # noqa: E402
                                 pose_to_xyzrpy, tag_pixel_size,
                                 ASSUMED_HFOV_DEG, DEFAULT_QUAD_BLUR,
                                 MAX_REPROJ_RMS_PX)
from src.utils.drawing import draw_cube, draw_axes, draw_corners     # noqa: E402
from src.utils.rs_tuning import diagnose_frame                       # noqa: E402

DEFAULT_TAG_SIZE = 0.20      # m. 태그1 기준(20x20cm). 인쇄 후 자로 재서 --tag-size 로 덮어쓸 것
OUTDIR = ROOT / "work_dirs" / "run"

PANEL_BG = (30, 30, 32)
FONT = cv2.FONT_HERSHEY_PLAIN      # Hershey 중에서 제일 고정폭에 가깝다
COLORS = {
    "head": (150, 220, 255),
    "lab": (200, 200, 200),
    "ok": (140, 255, 170),
    "warn": (60, 200, 255),
    "bad": (90, 90, 255),
    "dim": (130, 130, 130),
    "rule": (80, 80, 80),
}
KEYMAP = "q quit | a axes/cube | s save | r reset fps | SPACE pause"
KEYMAP_PANEL = ["keys: q quit | a cube/axes/both | s save",
                "      r reset fps | SPACE pause"]


# ----------------------------------------------------------------- 입력 소스
def open_pipeline(args, tag_size):
    """--source 에 맞는 TagPipeline 을 연다. 실패는 여기서 즉시 터뜨린다.

    예전에는 이 도구가 Source 클래스를 따로 들고 (프레임, 내부파라미터, origin,
    ASSUMED 여부) 를 손으로 엮었는데, 그 조립이 TagPipeline 으로 통째로
    들어갔다. 여기 남은 것은 **CLI 인자를 소스별 인자로 옮겨 적는 일**뿐이다.
    """
    common = dict(families=args.family, quad_blur=args.quad_blur,
                  method=args.method, min_margin=args.min_margin,
                  max_hamming=args.max_hamming,
                  )

    if args.source in ("realsense", "ir"):
        stream = "color" if args.source == "realsense" else "infrared"
        # emitter=None 이 자동이다 — IR 이면 끄고 컬러면 켠다.
        # 점 패턴이 태그 위에 찍히면 검출이 죽으므로 IR 에서는 반드시 꺼야 한다.
        label = "realsense/%s" % stream
        if args.exposure_ms is not None:
            label += " (exp %.1fms 고정)" % args.exposure_ms
        if args.ae_roi:
            label += " (AE ROI)"
        if stream == "infrared":
            label += " (ir%d, emitter auto-off)" % args.ir_index
        return TagPipeline.from_realsense(
            tag_size, stream=stream, width=args.width, height=args.height,
            fps=args.fps, ir_index=args.ir_index, ae_roi=args.ae_roi,
            exposure_us=(None if args.exposure_ms is None else args.exposure_ms * 1000.0),
            ae_priority=args.ae_priority, label=label, **common)

    if args.source == "bag":
        if not args.path:
            raise SystemExit("--source bag 은 --path 가 필요하다")
        if not Path(args.path).exists():
            raise SystemExit("bag 이 없다: %s" % args.path)
        # realtime=False 가 기본이다 — 한 프레임도 안 버리고 우리 속도에 맞춰 준다.
        # 임계값을 만질 때는 이쪽이어야 같은 입력으로 숫자를 비교할 수 있다.
        return TagPipeline.from_bag(args.path, tag_size, loop=args.loop, **common)

    if args.source == "video":
        if not args.path:
            raise SystemExit("--source video 는 --path 가 필요하다")
        if not Path(args.path).exists():
            raise SystemExit("영상이 없다: %s" % args.path)
        return TagPipeline.from_video(args.path, tag_size, loop=args.loop,
                                      hfov=args.hfov, **common)

    raise SystemExit("모르는 소스: %s" % args.source)


# ----------------------------------------------------------------- fps
class FpsMeter:
    """측정 fps. 프레임 간격을 지수이동평균으로 눌러 숫자가 튀지 않게 한다."""

    def __init__(self, alpha=0.15):
        self.alpha = alpha
        self.reset()

    def reset(self):
        self._dt = None
        self._last = None
        self.n = 0

    def tick(self):
        now = time.perf_counter()
        if self._last is not None:
            dt = now - self._last
            if dt > 0:
                self._dt = dt if self._dt is None else self.alpha * dt + (1 - self.alpha) * self._dt
        self._last = now
        self.n += 1

    @property
    def fps(self):
        return 0.0 if not self._dt else 1.0 / self._dt


# ----------------------------------------------------------------- 그리기
DRAW_MODES = ("cube", "axes", "both")   # a 키가 이 순서로 돈다


def draw_overlay(res, tag_size, mode="cube"):
    """Result 위에 검출/자세를 그려 화면용 BGR 한 장을 만든다.

    원본에 그리면 검출 좌표와 픽셀이 어긋나 상자가 태그에서 떠 보인다.

    그리기 실패로 뷰어가 죽지는 않게 전부 삼킨다. 여기서 나는 예외는 화면
    문제일 뿐이고, 숫자는 이미 res 안에 다 들어 있다.
    """
    img = np.asarray(res.image)
    vis = img.copy() if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    for d in res.detections:
        try:
            draw_corners(vis, d)               # 세 draw_* 는 전부 vis 를 제자리에서 고친다
            T = res.poses.get(int(d.tag_id))
            if T is not None:
                # both 는 선이 15개라 태그가 작을 때(5m 에서 54px) 지저분하다.
                # 그래서 기본은 큐브 하나고, 필요할 때만 겹쳐 본다.
                if mode in ("cube", "both"):
                    draw_cube(vis, res.intrinsics.params, tag_size, T)
                if mode in ("axes", "both"):
                    draw_axes(vis, res.intrinsics.params, tag_size, T)
        except Exception:
            pass
    return vis


def pick_primary(res, want_id=None):
    """패널에 띄울 주 태그의 detection. --tag-id 가 있으면 그것, 없으면 제일 큰 태그.

    Result.primary() 와 두 가지가 다르다.
      - 자세 계산에 **실패한 태그도 고른다.** 패널이 "POSE FAILED" 를 띄우려면
        그 태그를 잡고 있어야 한다. 실패를 조용히 건너뛰면 화면에는 태그가
        보이는데 패널만 "no detection" 이 되어 사람을 헷갈리게 한다.
      - 그래서 반환이 dict 가 아니라 detection 하나다.
    고르는 기준은 Result.primary() 와 같은 tag_pixel_size(네 변 길이 평균)다 —
    면적(contourArea)으로 재면 태그가 기울 때 원근으로 납작해져서
    "멀어졌다"와 "돌아갔다"를 구분하지 못한다.
    """
    if not res.detections:
        return None
    if want_id is not None:
        for d in res.detections:
            if int(d.tag_id) == int(want_id):
                return d
    return max(res.detections, key=tag_pixel_size)


# ----------------------------------------------------------------- 패널
def build_lines(ctx):
    """패널 한 장을 (문자열, 색키) 목록으로 만든다. 영어만 쓴다."""
    L = []
    add = lambda t, c="lab": L.append((t, c))
    rule = lambda: L.append(("-" * 44, "rule"))

    intr, tag_size = ctx["intr"], ctx["tag_size"]
    scale_bad = ctx["intr_assumed"] or ctx["size_assumed"]
    q = "?" if scale_bad else ""               # 거리 계열에 붙이는 의심 표시
    cd = "warn" if scale_bad else "ok"

    add("APRILTAG DOCKING  -  LIVE POSE", "head")
    add("source     : %s" % ctx["source"])
    add("resolution : %dx%d   frame %d%s" % (ctx["w"], ctx["h"], ctx["frame"],
                                             "  [PAUSED]" if ctx["paused"] else ""))
    add("fps        : %5.1f  (measured)" % ctx["fps"])
    if ctx["strip"]:
        add("preproc    : color overlay stripped (%s)" % ctx["strip"], "warn")
    add("intrinsics : fx=%.1f fy=%.1f" % (intr.fx, intr.fy))
    add("             cx=%.1f cy=%.1f" % (intr.cx, intr.cy))
    add("  origin   : %s" % ctx["intr_origin"], "warn" if ctx["intr_assumed"] else "ok")
    if ctx["intr_assumed"]:
        add("             ** NOT CALIBRATED - scale suspect **", "bad")
    add("tag size   : %.3f m  (%s)" % (tag_size, "ASSUMED default" if ctx["size_assumed"]
                                       else "user --tag-size"),
        "warn" if ctx["size_assumed"] else "ok")
    rule()

    res = ctx["res"]
    det = ctx["primary"]
    if det is None:
        add("[raw] camera -> tag", "head")
        add("  no detection", "bad")
        if res.errors.get("detect"):
            add("  detector failed: %s" % res.errors["detect"], "bad")
        rule()
        add("[control] forklift -> dock", "head")
        add("  -", "dim")
    else:
        tid = int(det.tag_id)
        T = res.poses.get(tid)
        add("[raw] camera -> tag   id=%d  margin=%.0f" % (tid, det.decision_margin), "head")
        if T is None:
            add("  POSE FAILED: %s" % res.errors.get(tid, "unknown"), "bad")
            rule()
            add("[control] forklift -> dock", "head")
            add("  unavailable", "bad")
        else:
            v = pose_to_xyzrpy(T)
            st = res.docking[tid]
            qa = res.quality.get(tid, {})
            ok_ang = bool(st["reliable_angle"])
            a = "" if ok_ang else "?"
            ca = "ok" if ok_ang else "warn"

            add("  x %+8.3f  y %+8.3f  z %+8.3f  [m]%s"
                % (v["x"], v["y"], v["z"], q), cd)
            add("  roll %+7.1f  pitch %+7.1f  yaw %+7.1f  [deg]"
                % (v["roll"], v["pitch"], v["yaw"]))
            add("  distance %.3f m%s" % (v["distance"], q), cd)
            rule()
            add("[control] forklift -> dock", "head")
            side = "left" if st["lateral"] > 0 else "right"
            add("  lateral  %+8.3f m%s  (%s of tag)" % (st["lateral"], q, side), cd)
            add("  forward  %+8.3f m%s" % (st["forward"], q), cd)
            add("  vertical %+8.3f m%s" % (st["vertical"], q), cd)
            add("  heading  %+8.1f deg%s" % (st["heading_deg"], a), ca)
            add("  approach %+8.1f deg%s  (unsigned)" % (st["approach_deg"], a), ca)
            add("  tag tilt %8.1f deg   reliable: %s"
                % (qa.get("tilt_deg", float("nan")), "yes" if ok_ang else "NO"),
                "ok" if ok_ang else "warn")
            # 재투영은 반드시 rms_px 로 찍는다. estimate_pose 가 주는 e1 은
            # detection_pose 경로에서 **px^2** 이라 px 인 척 찍으면 한 자릿수가
            # 통째로 틀린다(pose_quality docstring 의 단위 함정).
            rms = qa.get("reproj_rms_px", float("nan"))
            add("  reproj rms %5.2f px" % rms,
                "ok" if rms <= MAX_REPROJ_RMS_PX else "warn")
            add("  tag px   %8.1f     quality: %s"
                % (qa.get("tag_px", float("nan")),
                   "ok" if qa.get("ok") else ",".join(qa.get("reasons", [])) or "-"),
                "ok" if qa.get("ok") else "warn")
            if not ok_ang:
                add("  ! angle < 10deg: steer on lateral", "warn")
    rule()
    ids = res.tag_ids
    add("detections : %d   ids: %s" % (len(ids), ids if ids else "[]"),
        "ok" if ids else "bad")
    add("draw: %s" % ctx["draw"], "dim")
    for k in KEYMAP_PANEL:
        add(k, "dim")
    return L


def render_panel(lines, width, height, scale=1.15, step=21, margin=12):
    """패널 줄을 이미지로 그린다. 제일 긴 줄이 폭을 넘으면 글자를 줄여서 맞춘다."""
    inner = width - 2 * margin
    widest = max([cv2.getTextSize(t, FONT, scale, 1)[0][0] for t, _ in lines] or [1])
    if widest > inner:
        scale *= inner / float(widest)
    need = margin * 2 + step * len(lines)
    panel = np.full((max(height, need), width, 3), PANEL_BG, np.uint8)
    y = margin + step
    for text, key in lines:
        cv2.putText(panel, text, (margin, y), FONT, scale, COLORS.get(key, COLORS["lab"]),
                    1, cv2.LINE_AA)
        y += step
    return panel


def compose_canvas(vis, lines, args):
    """왼쪽 영상 + 오른쪽 패널을 한 캔버스로 붙인다.

    이름이 compose_canvas 인 이유: src/utils/util.py 에 동차변환을 곱하는
    compose(*Ts) 가 이미 있다. 같은 이름을 쓰면 둘 중 하나가 조용히 가려지고,
    터지는 자리는 여기가 아니라 저 아래 좌표 계산 한복판이 된다.
    """
    h, w = vis.shape[:2]
    if h > args.view_height:
        s = args.view_height / float(h)
        vis = cv2.resize(vis, (int(round(w * s)), args.view_height), interpolation=cv2.INTER_AREA)
        h, w = vis.shape[:2]
    panel = render_panel(lines, args.panel_width, h)
    H = max(h, panel.shape[0])
    canvas = np.full((H, w + panel.shape[1], 3), PANEL_BG, np.uint8)
    canvas[:h, :w] = vis
    canvas[:panel.shape[0], w:] = panel
    return canvas


# ----------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="AprilTag docking pose - live viewer")
    ap.add_argument("--source", default="realsense", choices=["realsense", "ir", "bag", "video"])
    ap.add_argument("--path", default=None, help="--source video/bag 일 때 파일 경로")
    ap.add_argument("--loop", action="store_true", help="영상/bag 끝에서 되감는다")
    ap.add_argument("--width", type=int, default=None, help="RealSense 가로")
    ap.add_argument("--height", type=int, default=None, help="RealSense 세로")
    ap.add_argument("--fps", type=int, default=30, help="RealSense fps")
    ap.add_argument("--ir-index", type=int, default=1, help="적외선 1=왼쪽 2=오른쪽")
    ap.add_argument("--intrinsics", default=None, help="cameras.yaml 로 내부 파라미터 덮어쓰기")
    ap.add_argument("--yaml-cam", default="cam0", help="yaml 안의 카메라 이름")
    ap.add_argument("--hfov", type=float, default=ASSUMED_HFOV_DEG,
                    help="보정값이 없을 때 가정할 수평화각 [deg]")
    ap.add_argument("--tag-size", type=float, default=None,
                    help="태그 한 변 [m], 검은 테두리 포함. 안 주면 %.2f 로 가정한다"
                         % DEFAULT_TAG_SIZE)
    ap.add_argument("--tag-id", type=int, default=None, help="패널에 고정으로 띄울 태그")
    ap.add_argument("--family", default="tag36h11")
    ap.add_argument("--quad-blur", type=float, default=DEFAULT_QUAD_BLUR,
                    help="노이즈 심한 실촬영은 2~4")
    ap.add_argument("--min-margin", type=float, default=0.0)
    ap.add_argument("--max-hamming", type=int, default=0)
    ap.add_argument("--method", default="auto", choices=["auto", "tag", "pnp"])
    # RealSense 전용 — 근거는 src/utils/rs_tuning.py 와 open_realsense docstring 에 있다
    ap.add_argument("--ae-roi", action="store_true",
                    help="자동노출을 태그에만 건다. 역광 도크에서 검출률을 가른다")
    ap.add_argument("--exposure-ms", type=float, default=None, metavar="MS",
                    help="컬러 수동노출 [ms]. 모션블러를 끊는 유일한 수단이다. "
                         "실측: 블러 10px 까지 100%%, 32px 에서 0%%. "
                         "1.0m/s·z=1m 이면 7.4ms 로 끊어야 10px 이다. "
                         "(주의: 고정하면 자동노출이 꺼진다)")
    ap.add_argument("--ae-priority", type=int, default=None, choices=[0, 1],
                    help="0=fps 사수(노출이 프레임시간을 못 넘음), 1=어두우면 fps 를 떨굼")
    ap.add_argument("--diagnose", action="store_true",
                    help="검출 실패 프레임마다 원인(노출/게인/드롭)을 한 줄 찍는다")
    ap.add_argument("--speed", type=float, default=0.5, metavar="MPS",
                    help="--diagnose 의 블러 환산에 쓸 가정 주행속도 [m/s]")
    ap.add_argument("--draw", default="cube", choices=DRAW_MODES,
                    help="시작 표시. a 키로 cube -> axes -> both 순환")
    ap.add_argument("--view-height", type=int, default=720, help="화면에 띄울 영상 높이")
    ap.add_argument("--panel-width", type=int, default=500)
    ap.add_argument("--headless", type=int, default=0, metavar="N",
                    help="창 없이 N 프레임만 처리하고 패널을 stdout 으로 찍는다")
    args = ap.parse_args()

    tag_size = args.tag_size if args.tag_size else DEFAULT_TAG_SIZE
    size_assumed = args.tag_size is None

    try:
        pipe = open_pipeline(args, tag_size)
    except SystemExit:
        raise
    except Exception as exc:
        raise SystemExit("소스를 열지 못했다 (%s): %s: %s"
                         % (args.source, type(exc).__name__, exc))
    if args.intrinsics:
        pipe.intr = CameraIntrinsics.from_yaml(args.intrinsics, args.yaml_cam)
        pipe.intrinsics_assumed = False
        pipe.origin = "yaml (%s:%s)" % (Path(args.intrinsics).name, args.yaml_cam)

    fps = FpsMeter()

    print("source     : %s" % pipe.label)
    print("tag size   : %.3f m%s" % (tag_size, "  (ASSUMED default)" if size_assumed else ""))
    print("intrinsics : %s" % (pipe.origin or "pending (first frame)"))
    print("keys       : %s" % KEYMAP)
    if args.headless:
        print("headless   : %d frames, no window\n" % args.headless)
    else:
        print("")

    win = "AprilTag live pose"
    if not args.headless:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    paused = False
    canvas = None
    lines = []
    n_seen = n_hit = 0
    dists = []
    last_z = None                  # 직전에 성공한 거리. 실패 프레임 블러 환산에 쓴다
    try:
        # pipe 를 그냥 `for res in pipe:` 로 돌지 않는 이유가 두 개 있다.
        #   - 빈 프레임을 세기 전에 걸러야 검출률(n_seen)이 오염되지 않는다.
        #   - 자동노출 ROI 와 --diagnose 가 **원본 프레임**을 봐야 한다
        #     (res.image 는 오버레이를 지운 쪽이라 depth/luma 가 떨어져 나간다).
        for i, ts, frame in pipe.frames:
            if frame is None or frame.size == 0:
                continue
            fps.tick()
            n_seen += 1

            res = pipe.process(frame, index=i, timestamp=ts)
            intr = res.intrinsics
            det = pick_primary(res, args.tag_id)
            if res.detections:
                n_hit += 1
            T = None if det is None else res.poses.get(int(det.tag_id))
            if T is not None:
                last_z = float(pose_to_xyzrpy(T)["distance"])
                dists.append(last_z)

            # 자동노출을 태그로 끌고 간다. 놓친 프레임에는 직전 ROI 를 유지한다
            # (매번 전체화면으로 되돌리면 노출이 펄떡거려 재검출을 방해한다).
            roi = pipe.ae_roi
            if roi is not None and res.detections:
                roi.follow(res.detections, frame.shape)

            # 못 찾은 프레임은 왜 못 찾았는지 그 자리에서 남긴다. 지나가면 못 캔다.
            if args.diagnose and not res.detections:
                print("frame %d 검출실패: %s" % (
                    i, diagnose_frame(frame, fx=intr.fx, z_m=last_z,
                                      speed_mps=args.speed)))

            ctx = {"source": pipe.label, "w": frame.shape[1], "h": frame.shape[0],
                   "frame": i, "fps": fps.fps, "intr": intr,
                   "intr_origin": pipe.origin, "intr_assumed": pipe.intrinsics_assumed,
                   "tag_size": tag_size, "size_assumed": size_assumed,
                   "res": res, "primary": det,
                   "paused": paused, "draw": args.draw,
                   }
            lines = build_lines(ctx)

            if args.headless:
                print("===== frame %d =====" % i)
                for text, _c in lines:
                    print(text)
                print("")
                if n_seen >= args.headless:
                    break
                continue

            canvas = compose_canvas(draw_overlay(res, tag_size, args.draw), lines, args)
            cv2.imshow(win, canvas)

            # 일시정지 중에는 같은 화면을 계속 다시 그리며 키만 받는다
            while True:
                key = cv2.waitKey(1 if not paused else 30) & 0xFF
                if key in (ord('q'), 27):
                    raise KeyboardInterrupt
                if key == ord('a'):
                    args.draw = DRAW_MODES[(DRAW_MODES.index(args.draw) + 1)
                                           % len(DRAW_MODES)]
                if key == ord('r'):
                    fps.reset()
                if key == ord('s'):
                    OUTDIR.mkdir(parents=True, exist_ok=True)
                    name = "%s_f%06d.png" % (datetime.now().strftime("%Y%m%d_%H%M%S"), i)
                    cv2.imwrite(str(OUTDIR / name), canvas)
                    print("saved: %s" % (OUTDIR / name))
                if key == ord(' '):
                    paused = not paused
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    raise KeyboardInterrupt
                if not paused:
                    break
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:               # 소스가 첫 프레임에서야 터지는 경우
        print("capture failed: %s" % exc)
    finally:
        pipe.close()                   # 제너레이터를 닫아야 파이프라인/캡처가 풀린다
        cv2.destroyAllWindows()

    hit = 100.0 * n_hit / max(1, n_seen)
    print("frames %d, detected %d (%.0f%%)" % (n_seen, n_hit, hit))
    # 프레임 드롭 회계. RealSense/bag 소스일 때만 있다.
    # 이걸 봐야 fps 가 30 을 밑돈 게 "카메라가 못 냈다"인지
    # "우리가 늦어 버려졌다"인지 갈린다 — 대책이 정반대다.
    st = pipe.stats
    if st is not None and st.received:
        print("frames(SDK): %s" % st.summary())
    if dists:
        print("distance: min %.3f  max %.3f  mean %.3f m%s"
              % (min(dists), max(dists), sum(dists) / len(dists),
                 "   (ASSUMED intrinsics/tag size)"
                 if pipe.intrinsics_assumed or size_assumed else ""))


if __name__ == "__main__":
    main()
