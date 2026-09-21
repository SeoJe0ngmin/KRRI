"""도킹용 AprilTag 자세를 실시간 화면으로 봄.

    python tools/live_pose.py                        # RealSense (Windows/Linux/Jetson)
    python tools/live_pose.py --source webcam        # macOS: RealSense 컬러를 UVC 웹캠으로
                                                     #   (depth/IR/IMU/녹화 없음. 이유는 open_webcam)
    python tools/live_pose.py --source bag --path x.db3
"""
import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from config.main import TAG_SIZE_M as DEFAULT_TAG_SIZE  # noqa: E402
from config.main import (CAM_YAW_OFFSET_DEG, MIN_DECISION_MARGIN,        # noqa: E402
                        MIN_TAG_PX, STABLE_TAG_PX)
from src.models import (CameraIntrinsics, TagPipeline, camera_index,   # noqa: E402
                                 pose_to_xyzrpy, pose_to_forklift,
                                 tag_pixel_size,
                                 DEFAULT_QUAD_BLUR,
                                 MAX_REPROJ_RMS_PX)
from src.utils.drawing import draw_cube, draw_axes, draw_corners     # noqa: E402
from src.utils.camera import diagnose_frame                       # noqa: E402

OUTDIR = ROOT / "work_dirs" / "live_pose"       # 스크린샷(s 키)
LOGDIR = ROOT / "work_dirs" / "live_pose" / "log"   # --log
BAGDIR = ROOT / "work_dirs" / "live_pose" / "bag"   # --record

PANEL_BG = (30, 30, 32)
FONT = cv2.FONT_HERSHEY_PLAIN      # Hershey 중에서 제일 고정폭에 가까움
# 패널은 DUPLEX 를 쓴다. PLAIN 은 같은 크기로도 글자 높이가 절반이라 읽기 나쁨.
PFONT = cv2.FONT_HERSHEY_DUPLEX
COLORS = {
    "head": (150, 220, 255),
    "big": (255, 255, 255),
    "bigwarn": (60, 200, 255),
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
def _resolve(val, outdir, ext):
    """이름만 주면 outdir 아래로, 경로가 들어오면 그대로. 안 주면 시각으로 지음."""
    if not val:
        return outdir / ("%s%s" % (datetime.now().strftime("%Y%m%d_%H%M%S"), ext))
    p = Path(val)
    if not (p.is_absolute() or "/" in val):
        p = outdir / val
    return p if p.suffix == ext else p.with_suffix(ext)


def open_pipeline(args, tag_size):
    """--source 에 맞는 TagPipeline 을 엶. 실패는 여기서 즉시 터뜨림."""
    common = dict(families=args.family, quad_blur=args.quad_blur,
                  method=args.method, min_margin=args.min_margin,
                  max_hamming=args.max_hamming,
                  )

    if args.source in ("realsense", "ir"):
        stream = "color" if args.source == "realsense" else "infrared"
        # emitter=None 이 자동임 — IR 이면 끄고 컬러면 켬.
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
            ae_priority=args.ae_priority, label=label,
            record=(None if args.record is None
                    else str(_resolve(args.record, BAGDIR, ".db3"))), **common)

    if args.source == "bag":
        if not args.path:
            raise SystemExit("--source bag 은 --path 가 필요하다")
        if not Path(args.path).exists():
            raise SystemExit("bag 이 없다: %s" % args.path)
        # realtime=False 가 기본임 — 한 프레임도 안 버리고 우리 속도에 맞춰 줌.
        return TagPipeline.from_bag(args.path, tag_size, loop=args.loop, **common)

    if args.source == "video":
        if not args.path:
            raise SystemExit("--source video 는 --path 가 필요하다")
        if not Path(args.path).exists():
            raise SystemExit("영상이 없다: %s" % args.path)
        return TagPipeline.from_video(args.path, tag_size, loop=args.loop,
                                      hfov=args.hfov, **common)

    if args.source == "webcam":
        # macOS 에서 RealSense 컬러를 보는 길 — librealsense 없이 OpenCV 로 연다.
        # 이름 조각(기본 realsense)이나 번호. 번호는 OpenCV 순서라 이름이 안전하다.
        if args.record is not None:
            raise SystemExit("--record 는 --source realsense/ir 에서만 된다")
        try:
            idx, name = camera_index(args.path or "realsense")
        except RuntimeError as exc:
            raise SystemExit(str(exc))
        label = "webcam #%d%s" % (idx, (" (%s)" % name) if name else "")
        if args.hfov is None and "realsense" not in name.lower():
            label += " [intrinsics: D435i 가정 — 다른 카메라면 --hfov 를 줘라]"
        return TagPipeline.from_webcam(idx, tag_size, width=args.width, height=args.height,
                                       fps=args.fps, hfov=args.hfov, label=label, **common)

    raise SystemExit("모르는 소스: %s" % args.source)


# ----------------------------------------------------------------- fps
class FpsMeter:
    """측정 fps. 프레임 간격을 지수이동평균으로 눌러 숫자가 튀지 않게 함."""

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
DRAW_MODES = ("cube", "axes", "both")   # a 키가 이 순서로 돎


def draw_overlay(res, tag_size, mode="cube"):
    """Result 위에 검출/자세를 그려 화면용 BGR 한 장을 만듦."""
    img = np.asarray(res.image)
    vis = img.copy() if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    for d in res.detections:
        try:
            draw_corners(vis, d)               # 세 draw_* 는 전부 vis 를 제자리에서 고침
            T = res.poses.get(int(d.tag_id))
            if T is not None:
                # both 는 선이 15개라 태그가 작을 때(5m 에서 54px) 지저분함.
                if mode in ("cube", "both"):
                    draw_cube(vis, res.intrinsics.params, tag_size, T)
                if mode in ("axes", "both"):
                    draw_axes(vis, res.intrinsics.params, tag_size, T)
        except Exception:
            pass
    return vis


def pick_primary(res, want_id=None):
    """패널에 띄울 주 태그의 detection. --tag-id 가 있으면 그것, 없으면 제일 큰 태그."""
    if not res.detections:
        return None
    if want_id is not None:
        for d in res.detections:
            if int(d.tag_id) == int(want_id):
                return d
    return max(res.detections, key=tag_pixel_size)


# ----------------------------------------------------------------- 패널
def build_lines(ctx):
    """패널 한 장을 항목 목록으로 만듦.

    항목은 종류별 튜플이다. render_panel 이 그리고, --headless 는 글로 찍는다.
        ("title", 글)                   맨 위 제목
        ("rule",)                       구분선
        ("sec", 이름)                   구역 이름 (QUALITY, SETUP ...)
        ("big", 라벨, 값, 색, 덧말)      크게 보여줄 값 (제어에 쓰는 셋)
        ("kv", 키, 값, 색)              작은 두 칸 줄
        ("msg", 글, 색)                 한 줄 알림
    """
    L = []
    intr, tag_size = ctx["intr"], ctx["tag_size"]
    scale_bad = ctx["intr_assumed"] or ctx["size_assumed"]
    q = "?" if scale_bad else ""               # 거리 계열에 붙이는 의심 표시
    big_c = "warn" if scale_bad else "val"

    L.append(("title", "APRILTAG DOCKING"))
    L.append(("rule",))

    res, det = ctx["res"], ctx["primary"]
    T = None if det is None else res.poses.get(int(det.tag_id))

    if det is None:
        L.append(("msg", "  no detection", "bad"))
        if res.errors.get("detect"):
            L.append(("msg", "  detector failed: %s" % res.errors["detect"], "bad"))
        L.append(("rule",))
    elif T is None:
        L.append(("msg", "  POSE FAILED: %s" % res.errors.get(int(det.tag_id), "unknown"),
                  "bad"))
        L.append(("rule",))
    else:
        tid = int(det.tag_id)
        v, f = pose_to_xyzrpy(T), pose_to_forklift(T)
        st = res.docking[tid]
        qa = res.quality.get(tid, {})
        ok_ang = bool(st["reliable_angle"])
        a = "" if ok_ang else "?"

        # 제어에 쓰는 셋. 크게.
        L.append(("big", "LATERAL", "%+.3f m%s" % (st["lateral"], q), big_c, ""))
        L.append(("big", "FORWARD", "%+.3f m%s" % (st["forward"], q), big_c, ""))
        L.append(("big", "HEADING", "%+.1f deg%s" % (st["heading_deg"], a),
                  big_c if ok_ang else "warn", "" if ok_ang else "못 믿음"))
        L.append(("rule",))

        # 이 값을 믿어도 되나
        rms = qa.get("reproj_rms_px", float("nan"))
        tpx = qa.get("tag_px", float("nan"))
        L.append(("sec", "QUALITY"))
        L.append(("kv", "tag size", "%.0f px   (>=%.0f)" % (tpx, MIN_TAG_PX),
                  "ok" if tpx >= STABLE_TAG_PX else "warn"))
        L.append(("kv", "tilt", "%.1f deg   %s" % (qa.get("tilt_deg", float("nan")),
                  "각도 OK" if ok_ang else "각도 못 믿음"), "ok" if ok_ang else "warn"))
        L.append(("kv", "reproj", "%.2f px" % rms,
                  "ok" if rms <= MAX_REPROJ_RMS_PX else "warn"))
        L.append(("kv", "margin", "%.0f" % det.decision_margin, "ok"))
        if not qa.get("ok", True):
            L.append(("msg", "  ! %s" % (",".join(qa.get("reasons", [])) or "quality"), "warn"))
        if not ok_ang:
            L.append(("msg", "  ! 각도를 못 믿는다 — lateral 로 조종할 것", "warn"))
        L.append(("rule",))

        # 이상할 때만 보는 값
        side = "left" if st["lateral"] > 0 else "right"
        L.append(("sec", "REFERENCE"))
        L.append(("kv", "forklift", "lat %+.3f  vert %+.3f  fwd %+.3f"
                  % (f["lateral"], f["vertical"], f["forward"]), "dim"))
        L.append(("kv", "", "roll %+.1f  pitch %+.1f  yaw %+.1f"
                  % (f["roll"], f["pitch"], f["yaw"]), "dim"))
        L.append(("kv", "raw cam", "x %+.3f  y %+.3f  z %+.3f"
                  % (v["x"], v["y"], v["z"]), "dim"))
        L.append(("kv", "", "roll %+.1f  pitch %+.1f  yaw %+.1f"
                  % (v["roll"], v["pitch"], v["yaw"]), "dim"))
        L.append(("kv", "approach", "%+.1f deg%s   (태그 %s 쪽)"
                  % (st["approach_deg"], a, side), "dim"))
        L.append(("rule",))

    # 한 번 확인하고 잊는 값
    ids = res.tag_ids
    L.append(("sec", "SETUP"))
    L.append(("kv", "source", "%s   %dx%d   %.1f fps%s"
              % (ctx["source"], ctx["w"], ctx["h"], ctx["fps"],
                 "  [PAUSED]" if ctx["paused"] else ""), "dim"))
    L.append(("kv", "intrinsics", "fx %.1f  fy %.1f" % (intr.fx, intr.fy), "dim"))
    L.append(("kv", "", "cx %.1f  cy %.1f    %s" % (intr.cx, intr.cy, ctx["intr_origin"]),
              "warn" if ctx["intr_assumed"] else "dim"))
    L.append(("kv", "tag", "%.0f mm   %s   ids %s"
              % (tag_size * 1000, ctx["family"], ids if ids else "[]"),
              "warn" if ctx["size_assumed"] else "dim"))
    L.append(("kv", "detector", "method %s  blur %.1f  margin>=%.0f"
              % (ctx["method"], ctx["quad_blur"], MIN_DECISION_MARGIN), "dim"))
    if ctx.get("yaw_dev") is not None:
        from src.utils.imu_yaw import imu_panel_lines
        L.append(("sec", "IMU (자이로)"))
        L.extend(imu_panel_lines(ctx["yaw_dev"]))
    L.append(("kv", "offset", "cam yaw %+.1f deg" % CAM_YAW_OFFSET_DEG, "dim"))
    st_ = ctx.get("stats")
    L.append(("kv", "frames", "%d%s" % (ctx["frame"],
              "   drop %s" % st_.summary() if st_ is not None and st_.received else ""), "dim"))
    L.append(("kv", "draw", ctx["draw"], "dim"))
    L.append(("kv", "keys", "q quit   a cube/axes/both   s save", "dim"))
    L.append(("kv", "", "r reset fps   SPACE pause", "dim"))
    return L


def line_text(item):
    """항목 하나를 --headless 용 한 줄 글로."""
    k = item[0]
    if k == "title":
        return item[1]
    if k == "rule":
        return "-" * 52
    if k == "sec":
        return "[%s]" % item[1]
    if k == "big":
        return "  %-9s %s%s" % (item[1], item[2], ("   " + item[4]) if item[4] else "")
    if k == "kv":
        return "  %-11s %s" % (item[1], item[2])
    return item[1]


#: 패널 색
PC = {"title": (150, 220, 255), "sec": (150, 150, 150), "lab": (140, 140, 140),
      "val": (245, 245, 245), "ok": (140, 255, 170), "warn": (60, 200, 255),
      "bad": (90, 90, 255), "dim": (105, 105, 105), "rule": (58, 58, 58)}

#: 항목 종류별 줄 높이 [px]
ROW_H = {"title": 25, "rule": 18, "sec": 24, "big": 69, "kv": 20}



# ── 한글 그리기 ─────────────────────────────────────────────────────────────
# cv2.putText 의 Hershey 폰트는 **ASCII 만** 된다 — 한글이 전부 '?' 로 찍힌다.
# (2026-09-21 현장 화면에서 경고 줄이 통째로 '???? ???' 로 나와 못 읽었다.)
# 그래서 글자에 한글이 섞여 있으면 Pillow + 시스템 CJK 폰트로 그린다.
_CJK_FONT_PATHS = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc",
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
    "/System/Library/Fonts/AppleSDGothicNeo.ttc",          # macOS
)
_font_cache = {}
_font_warned = [False]


def _cjk_font(px):
    """크기별 CJK 폰트. 없으면 None(그러면 ASCII 로 옮겨 적는다)."""
    px = max(9, int(px))
    if px in _font_cache:
        return _font_cache[px]
    f = None
    try:
        from PIL import ImageFont
        import os
        for path in _CJK_FONT_PATHS:
            if os.path.exists(path):
                try:
                    f = ImageFont.truetype(path, px)
                    break
                except Exception:
                    continue
    except Exception:
        f = None
    _font_cache[px] = f
    return f


def _put(img, text, org, font, scale, color, thickness=1, lineType=cv2.LINE_AA):
    """cv2.putText 와 같은 인자. 한글이 섞이면 Pillow 로 그린다."""
    text = str(text)
    if all(ord(c) < 128 for c in text):
        cv2.putText(img, text, org, font, scale, color, thickness, lineType)
        return
    f = _cjk_font(scale * 26)
    if f is None:
        if not _font_warned[0]:
            print("  !! CJK 폰트가 없다 — 한글이 '?' 로 나온다. "
                  "우분투: sudo apt install fonts-noto-cjk")
            _font_warned[0] = True
        cv2.putText(img, text.encode("ascii", "replace").decode(), org,
                    font, scale, color, thickness, lineType)
        return
    try:
        from PIL import Image, ImageDraw
        import numpy as _np
        pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        d = ImageDraw.Draw(pil)
        # cv2 의 org 는 **글자 아래쪽 기준선**, PIL 은 왼쪽 위 → 대략 폰트 높이만큼 올린다
        d.text((org[0], org[1] - int(scale * 26)), text,
               font=f, fill=(int(color[2]), int(color[1]), int(color[0])))
        img[:, :, :] = cv2.cvtColor(_np.asarray(pil), cv2.COLOR_RGB2BGR)
    except Exception:
        cv2.putText(img, text.encode("ascii", "replace").decode(), org,
                    font, scale, color, thickness, lineType)


def render_panel(items, width, height, scale=1.0, margin=18):
    """항목 목록을 패널 이미지로.

    큰 값은 **라벨을 위, 숫자를 아래** 두 줄로 그린다 — 한 줄에 몰아넣으면
    라벨과 숫자가 자리를 다투어 둘 다 작아진다.
    작은 두 칸 줄(kv)은 키 열을 맞춰 값만 세로로 훑을 수 있게 한다.
    """
    need = margin * 2 + sum(ROW_H.get(i[0], 22) for i in items)
    p = np.full((max(height, need), width, 3), PANEL_BG, np.uint8)
    kx = margin + int(108 * scale)          # kv 의 값이 시작하는 x
    y = margin + 14

    for it in items:
        k = it[0]
        if k == "rule":
            cv2.line(p, (margin, y - 7), (width - margin, y - 7), PC["rule"], 1)
        elif k == "title":
            _put(p, it[1], (margin, y), PFONT, 0.60 * scale, PC["title"], 1, cv2.LINE_AA)
        elif k == "sec":
            _put(p, it[1], (margin, y), PFONT, 0.50 * scale, PC["sec"], 1, cv2.LINE_AA)
        elif k == "big":
            _, lab, val, col, note = it
            _put(p, lab, (margin, y), PFONT, 0.50 * scale, PC["lab"], 1, cv2.LINE_AA)
            c = PC.get(col, PC["val"])
            _put(p, val, (margin, y + 29), PFONT, 1.12 * scale, c, 2, cv2.LINE_AA)
            if note:
                w = cv2.getTextSize(val, PFONT, 1.12 * scale, 2)[0][0]
                _put(p, note, (margin + w + 14, y + 29), PFONT, 0.48 * scale,
                            c, 1, cv2.LINE_AA)
        elif k == "kv":
            _, key, val, col = it
            if key:
                _put(p, key, (margin + 6, y), PFONT, 0.46 * scale, PC["dim"],
                            1, cv2.LINE_AA)
            _put(p, val, (kx, y), PFONT, 0.46 * scale, PC.get(col, PC["dim"]),
                        1, cv2.LINE_AA)
        else:                                   # msg
            _put(p, it[1], (margin, y), PFONT, 0.50 * scale,
                        PC.get(it[2], PC["lab"]), 1, cv2.LINE_AA)
        y += ROW_H.get(k, 22)
    return p


def compose_canvas(vis, lines, args):
    """왼쪽 영상 + 오른쪽 패널을 한 캔버스로 붙임."""
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
    ap.add_argument("--source", default="realsense", choices=["realsense", "ir", "bag", "video", "webcam"])
    ap.add_argument("--path", default=None, help="video/bag 은 파일 경로. webcam 은 번호나 이름 조각(기본 realsense)")
    ap.add_argument("--loop", action="store_true", help="영상/bag 끝에서 되감는다")
    ap.add_argument("--width", type=int, default=None, help="RealSense/webcam 가로")
    ap.add_argument("--height", type=int, default=None, help="RealSense/webcam 세로")
    ap.add_argument("--fps", type=int, default=30, help="RealSense/webcam fps")
    ap.add_argument("--ir-index", type=int, default=1, help="적외선 1=왼쪽 2=오른쪽")
    ap.add_argument("--hfov", type=float, default=None,
                    help="보정값이 없을 때 가정할 수평화각 [deg]")
    ap.add_argument("--tag-size", type=float, default=None,
                    help="태그 한 변 [m], 검은 테두리 포함. 안 주면 %.2f 로 가정한다"
                         % DEFAULT_TAG_SIZE)
    ap.add_argument("--tag-id", type=int, default=None, help="패널에 고정으로 띄울 태그")
    ap.add_argument("--family", default="tag36h11")
    ap.add_argument("--no-imu", action="store_true",
                    help="자이로 계기판을 끈다 (realsense 소스에서만 켜짐)")
    ap.add_argument("--quad-blur", type=float, default=DEFAULT_QUAD_BLUR,
                    help="노이즈 심한 실촬영은 2~4")
    ap.add_argument("--min-margin", type=float, default=0.0)
    ap.add_argument("--max-hamming", type=int, default=0)
    ap.add_argument("--method", default="auto", choices=["auto", "tag", "pnp"])
    # RealSense 전용 — 근거는 src/utils/camera.py 와 open_realsense docstring 에 있음
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
    ap.add_argument("--record", nargs="?", const="", default=None, metavar="PATH",
                    help="RealSense 스트림을 .db3 로 녹화한다(영상+depth+내부파라미터). "
                         "나중에 --source bag 으로 똑같이 재생된다. "
                         "이름만 주면 work_dirs/live_pose/bag/ 아래. "
                         "무압축이라 1080p 는 11GB/분이다 — 길게 찍으려면 해상도를 낮춰라")
    ap.add_argument("--log", nargs="?", const="", default=None, metavar="PATH",
                    help="프레임별 도킹값을 JSON 으로 저장한다. 값이 얼마나 흔들리는지 보려고. "
                         "이름만 주면 work_dirs/live_pose/log/ 아래, 안 주면 시각으로 짓는다")
    ap.add_argument("--headless", type=int, default=0, metavar="N",
                    help="창 없이 N 프레임만 처리하고 패널을 stdout 으로 찍는다")
    args = ap.parse_args()

    tag_size = args.tag_size if args.tag_size else DEFAULT_TAG_SIZE
    size_assumed = args.tag_size is None

    # 자이로를 카메라보다 먼저 연다 — RSUSB 백엔드는 먼저 연 device 객체가 IMU 를
    # 갖는다. 컬러가 먼저면 자이로가 "failed to set power state" 로 못 열린다.
    # 자이로 계기판 — 실카메라일 때만. 부호·드리프트를 눈으로 확인하는 용도라
    # run.py 와 같은 공용 표시(imu_panel_lines)를 쓴다.
    yaw_dev = None
    if args.source == "realsense" and not args.no_imu:
        try:
            from src.utils.imu_yaw import GyroYaw
            yaw_dev = GyroYaw().start()
            print("gyro       : 열림. 2.0초 정지 보정 — 카메라를 가만히 둘 것...")
            rep = yaw_dev.calibrate()
            print("             축 %s / 잡음 %.3f도/s%s"
                  % (rep["axis_src"], rep["noise_dps"],
                     "  !! 움직임 의심 — 커밋 안 됨" if rep["moving"] else ""))
        except Exception as exc:
            print("gyro       : 못 엶 (%s) — IMU 줄 없이 진행" % exc)
            yaw_dev = None

    try:
        pipe = open_pipeline(args, tag_size)
    except BaseException as exc:                      # SystemExit 포함
        if yaw_dev is not None:
            yaw_dev.close()
        if isinstance(exc, SystemExit):
            raise
        raise SystemExit("소스를 열지 못했다 (%s): %s: %s"
                         % (args.source, type(exc).__name__, exc))

    fps = FpsMeter()

    print("source     : %s" % pipe.label)
    print("tag size   : %.3f m%s" % (tag_size, "  (ASSUMED default)" if size_assumed else ""))
    print("intrinsics : %s" % (pipe.origin or "pending (first frame)"))
    print("keys       : %s" % KEYMAP)
    if args.record is not None:
        _bag = _resolve(args.record, BAGDIR, ".db3")
        print("record     : %s  (무압축 %s)"
              % (_bag, "약 11GB/분 @1080p" if (args.height or 1080) >= 1080
                 else "약 5GB/분 @720p"))
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
    log_rows = []                  # --log 용. 프레임마다 한 줄
    last_z = None                  # 직전에 성공한 거리. 실패 프레임 블러 환산에 씀
    try:
        # pipe 를 그냥 `for res in pipe:` 로 돌지 않는 이유가 두 개 있음.
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

            # 자동노출을 태그로 끌고 감. 놓친 프레임에는 직전 ROI 를 유지함
            roi = pipe.ae_roi
            if roi is not None and res.detections:
                roi.follow(res.detections, frame.shape)

            # 못 찾은 프레임은 왜 못 찾았는지 그 자리에서 남김. 지나가면 못 캠.
            if args.diagnose and not res.detections:
                print("frame %d 검출실패: %s" % (
                    i, diagnose_frame(frame, fx=intr.fx, z_m=last_z,
                                      speed_mps=args.speed)))

            if args.log is not None and det is not None:
                tid = int(det.tag_id)
                d = res.docking.get(tid)
                q = res.quality.get(tid, {})
                if d is not None:
                    log_rows.append({"frame": int(i), "t": float(ts), "tag_id": tid,
                                     **{k: float(v) if isinstance(v, (int, float)) else v
                                        for k, v in d.items()},
                                     "z_optical": float(T[2, 3]),
                                     "ok": bool(q.get("ok", False)),
                                     "tag_px": float(q.get("tag_px", 0.0)),
                                     "reproj_rms_px": float(q.get("reproj_rms_px", 0.0)),
                                     "decision_margin": float(q.get("decision_margin", 0.0))})

            ctx = {"source": pipe.label, "w": frame.shape[1], "h": frame.shape[0],
                   "frame": i, "fps": fps.fps, "intr": intr,
                   "intr_origin": pipe.origin, "intr_assumed": pipe.intrinsics_assumed,
                   "tag_size": tag_size, "size_assumed": size_assumed,
                   "res": res, "primary": det,
                   "paused": paused, "draw": args.draw,
                   "family": args.family, "method": args.method,
                   "quad_blur": args.quad_blur, "stats": pipe.stats,
                   "yaw_dev": yaw_dev,
                   }
            lines = build_lines(ctx)

            if args.headless:
                print("===== frame %d =====" % i)
                for it in lines:
                    print(line_text(it))
                print("")
                if n_seen >= args.headless:
                    break
                continue

            canvas = compose_canvas(draw_overlay(res, tag_size, args.draw), lines, args)
            cv2.imshow(win, canvas)

            # 일시정지 중에는 같은 화면을 계속 다시 그리며 키만 받음
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
        if yaw_dev is not None:
            yaw_dev.close()
        pipe.close()                   # 제너레이터를 닫아야 파이프라인/캡처가 풀림
        cv2.destroyAllWindows()

    hit = 100.0 * n_hit / max(1, n_seen)
    print("frames %d, detected %d (%.0f%%)" % (n_seen, n_hit, hit))
    # 프레임 드롭 회계. RealSense/bag 소스일 때만 있음.
    st = pipe.stats
    if st is not None and st.received:
        print("frames(SDK): %s" % st.summary())
    if args.record is not None:
        _b = _resolve(args.record, BAGDIR, ".db3")
        if _b.exists():
            print("record: %s (%.0f MB)" % (_b, _b.stat().st_size / 1e6))

    if args.log is not None:
        import json, statistics as st_
        # 이름만 주면 LOGDIR 아래로. 절대경로나 / 가 든 경로는 그대로 씀.
        out = _resolve(args.log, LOGDIR, ".json")
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps({"source": pipe.label, "tag_size": tag_size,
                                   "intrinsics_assumed": bool(pipe.intrinsics_assumed),
                                   "n": len(log_rows), "rows": log_rows},
                                  ensure_ascii=False, indent=1))
        print("log: %s (%d rows)" % (out, len(log_rows)))
        keys = ("lateral", "forward", "heading_deg", "tilt_deg", "z_optical")
        if len(log_rows) >= 2:
            print("  %-14s%10s%10s%10s" % ("", "평균", "표준편차", "최대-최소"))
            for k in keys:
                v = [r[k] for r in log_rows]
                u, f = ("mm", 1000.0) if k in ("lateral", "forward", "z_optical") else ("도", 1.0)
                v = [x * f for x in v]
                print("  %-14s%9.2f%s%9.2f%10.2f"
                      % (k, st_.mean(v), u, st_.pstdev(v), max(v) - min(v)))

    if dists:
        print("distance: min %.3f  max %.3f  mean %.3f m%s"
              % (min(dists), max(dists), sum(dists) / len(dists),
                 "   (ASSUMED intrinsics/tag size)"
                 if pipe.intrinsics_assumed or size_assumed else ""))


if __name__ == "__main__":
    main()
