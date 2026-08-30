"""정답을 **지어낸** 장면으로 자세 추정을 잰다. 카메라도 줄자도 없이."""
import argparse
import csv
import json
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import apriltag                                                      # noqa: E402
from src.config import TAG_SIZE_M as DEFAULT_TAG_SIZE  # noqa: E402
from src.config import TAG_CELLS as CELLS               # noqa: E402
from src.config import CAM_HEIGHT_M, TAG_HEIGHT_M       # noqa: E402
from src.models import (CameraIntrinsics, make_detector,    # noqa: E402
                                 detect, to_gray, estimate_pose,
                                 pose_to_xyzrpy, docking_state, tag_tilt_deg,
                                 pose_quality, tag_pixel_size, MIN_TAG_PX, STABLE_TAG_PX, RELIABLE_TILT_DEG,
                                 DEFAULT_QUAD_BLUR)
from src.models.detection.detection_pose import _object_points  # 정답 생성용 (private)
from src.utils.util import pr2t, invert_T                            # noqa: E402


# ===========================================================================
PRESETS = {
    "1920x1080": CameraIntrinsics(1359.2, 1359.0, 956.9, 571.3, 1920, 1080),
    "1280x720":  CameraIntrinsics(906.1,  906.0,  637.9, 380.9, 1280, 720),
    "640x480":   CameraIntrinsics(604.1,  604.0,  318.6, 253.9, 640,  480),
}
#: 기본값. 1920x1080 과 화각이 같으면서 렌더가 2.25배 빠르다.
DEFAULT_RES = "1280x720"

#: 사용자 배치 기본값 (m, m, m, m, deg)
DEFAULT_PLACEMENT = dict(tag_height=TAG_HEIGHT_M, cam_height=CAM_HEIGHT_M,
                         distance=3.00, lateral=-1.03, heading_deg=20.0)


#: AT2 검출기가 돌려주는 모서리는 OpenCV 픽셀 규약보다 **정확히 +0.5 px** 크다.
DETECTOR_CORNER_OFFSET_PX = 0.5

#: 한 출력 픽셀당 한 변 몇 번을 샘플링할지. 렌더러의 **핵심 파라미터**다.
SUPERSAMPLE = 4

#: 슈퍼샘플 버퍼 상한 [픽셀]. 넘으면 배율을 자동으로 낮춘다(품질보다 안 죽는 게 낫다).
_MAX_SS_PIXELS = 24_000_000

#: 이 잡음 수준까지는 quad_blur=0 으로도 검출기가 산다. 넘으면 SIGSEGV 다(함정 2).
NOISE_SAFE_LEVELS = 3.0
#: 잡음이 그 위일 때 올려 쓸 quad_blur.
NOISE_QUAD_BLUR = 2.0


# ===========================================================================

def placement_to_T(tag_height=TAG_HEIGHT_M, cam_height=CAM_HEIGHT_M, distance=3.00,
                   lateral=-1.03, heading_deg=20.0, check=True):
    """바닥 기준 배치를 T_camera_tag (4x4) 로 바꾼다."""
    h = np.deg2rad(float(heading_deg))
    c, s = np.cos(h), np.sin(h)
    # 열 = 카메라 축을 **태그 축으로** 적은 것
    R_tag_cam = np.array([[-c,  0.0,   s],      # 카메라 +x (오른쪽)
                          [0.0, -1.0, 0.0],     # 카메라 +y (아래)  -> 수평 카메라
                          [  s, 0.0,   c]])     # 카메라 +z (광축)  -> heading
    p_tag_cam = np.array([float(lateral),                          # -> docking lateral
                          float(cam_height) - float(tag_height),   # -> docking vertical (부호 주의)
                          -float(distance)])                       # -> forward = -p[2]
    T = invert_T(pr2t(p_tag_cam, R_tag_cam))
    if check:
        assert_roundtrip(T, tag_height, cam_height, distance, lateral, heading_deg)
    return T


def assert_roundtrip(T, tag_height, cam_height, distance, lateral, heading_deg,
                     tol=1e-9):
    """docking_state(T) 가 우리가 넣은 배치를 그대로 돌려주는지 본다."""
    st = docking_state(T)
    want = {"lateral": float(lateral),
            "vertical": float(cam_height) - float(tag_height),
            "forward": float(distance)}
    bad = {k: (st[k], v) for k, v in want.items() if abs(st[k] - v) > tol}
    # heading 은 atan2 라 (-180, 180] 로 접혀 나온다. 접은 뒤에 비교해야 한다.
    dh = (st["heading_deg"] - float(heading_deg) + 180.0) % 360.0 - 180.0
    if abs(dh) > tol:
        bad["heading_deg"] = (st["heading_deg"], float(heading_deg))
    # tilt 는 **|heading| 이 아니다.** tag_tilt_deg 는 arccos(|cos h|) 라 0..90 로
    want_tilt = np.degrees(np.arccos(abs(np.cos(np.radians(float(heading_deg))))))
    if abs(tag_tilt_deg(T) - want_tilt) > 1e-6:
        bad["tilt=arccos|cos h|"] = (tag_tilt_deg(T), want_tilt)
    if bad:
        raise AssertionError("배치 구성이 docking_state 와 안 맞는다: %s" % bad)
    return st


def truth_of(T):
    """정답 T 에서 표에 올릴 값들을 뽑는다. 추정쪽과 **같은 함수**로 뽑는다."""
    st = docking_state(T)
    v = pose_to_xyzrpy(T)
    return {"distance": st["distance"], "lateral": st["lateral"],
            "forward": st["forward"], "vertical": st["vertical"],
            "heading": st["heading_deg"], "tilt": tag_tilt_deg(T),
            "z": v["z"], "approach": st["approach_deg"],
            "rel_approach": bool(st["reliable_angle"]),
            "rel_tilt": bool(tag_tilt_deg(T) >= RELIABLE_TILT_DEG)}


# ===========================================================================

class OutOfView(Exception):
    """이 배치에서는 태그가 화면에 안 잡힌다. 렌더를 거부한다."""


class _Corners:
    """tag_pixel_size() 에 넘길 최소 껍데기. 그 함수는 .corners 만 본다."""

    def __init__(self, corners):
        self.corners = np.asarray(corners, dtype=np.float64)


def project_corners(T, intr, tag_size):
    """태그 네 모서리를 픽셀로 투영한다. detection.corners 와 **같은 순서**다."""
    T = np.asarray(T, dtype=np.float64)
    # 이 투영은 **순수 핀홀**이다. 왜곡계수가 실린 intrinsics 로 부르면 그림은 왜곡
    if getattr(intr, "distortion", ()):
        raise SystemExit("이 렌더러는 왜곡을 못 그린다 (distortion=%s). 왜곡 있는 렌즈를 "
                         "재려면 렌더러부터 고쳐야 한다 — 지금 돌리면 렌더는 핀홀인데 "
                         "추정만 왜곡을 되풀어 계통오차가 생긴다." % (intr.distortion,))
    obj = _object_points(float(tag_size))
    cam = (T[:3, :3] @ obj.T).T + T[:3, 3]
    z = np.where(np.abs(cam[:, 2]) < 1e-12, 1e-12, cam[:, 2])
    uv = (intr.K @ (cam / z[:, None]).T).T[:, :2]
    return uv, cam


def visibility(T, intr, tag_size, margin_px=0.0):
    """태그가 정말 찍히는가. **렌더 전에** 반드시 통과시킨다."""
    uv, cam = project_corners(T, intr, tag_size)
    w = intr.width or PRESETS[DEFAULT_RES].width
    h = intr.height or PRESETS[DEFAULT_RES].height
    behind = int((cam[:, 2] <= 0).sum())
    lo, hi_w, hi_h = margin_px, w - margin_px, h - margin_px
    outside = int(((uv[:, 0] < lo) | (uv[:, 0] >= hi_w) |
                   (uv[:, 1] < lo) | (uv[:, 1] >= hi_h)).sum())
    # 화면 크기는 **검출 결과와 같은 정의**로 재야 비교가 된다. tag_pixel_size 는
    tag_px = float(tag_pixel_size(_Corners(uv))) if behind == 0 else float("nan")

    reason = ""
    if behind:
        reason = "모서리 %d개가 카메라 뒤에 있다" % behind
    elif outside:
        reason = ("모서리 %d개가 화면(%dx%d) 밖이다 — 태그 중심이 화면에서 "
                  "(%.0f, %.0f)" % (outside, w, h, uv[:, 0].mean(), uv[:, 1].mean()))
    return {"ok": not reason, "reason": reason, "uv": uv, "cam": cam,
            "tag_px": tag_px}


# ===========================================================================

def _warp_area(canvas, texture, M, dst_uv, background, supersample):
    """질감을 canvas 에 **면적평균**으로 굽는다. 진짜 카메라가 하는 일이 이것이다."""
    h, w = canvas.shape[:2]
    ss = int(max(1, supersample))
    if ss == 1:
        cv2.warpPerspective(texture, M, (w, h), dst=canvas,
                            borderMode=cv2.BORDER_TRANSPARENT)
        return canvas
    x0 = max(0, int(np.floor(dst_uv[:, 0].min())) - 1)
    y0 = max(0, int(np.floor(dst_uv[:, 1].min())) - 1)
    x1 = min(w, int(np.ceil(dst_uv[:, 0].max())) + 2)
    y1 = min(h, int(np.ceil(dst_uv[:, 1].max())) + 2)
    if x1 <= x0 or y1 <= y0:
        return canvas                       # 태그가 화면과 안 겹친다
    bw, bh = x1 - x0, y1 - y0
    while ss > 1 and bw * bh * ss * ss > _MAX_SS_PIXELS:
        ss -= 1
    if ss == 1:
        cv2.warpPerspective(texture, M, (w, h), dst=canvas,
                            borderMode=cv2.BORDER_TRANSPARENT)
        return canvas
    A = np.array([[ss, 0.0, (ss - 1) / 2.0 - ss * x0],
                  [0.0, ss, (ss - 1) / 2.0 - ss * y0],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    buf = np.full((bh * ss, bw * ss, 3), int(background), dtype=np.uint8)
    cv2.warpPerspective(texture, A @ M, (bw * ss, bh * ss), dst=buf,
                        borderMode=cv2.BORDER_TRANSPARENT)
    canvas[y0:y1, x0:x1] = cv2.resize(buf, (bw, bh), interpolation=cv2.INTER_AREA)
    return canvas


def render(T, intrinsics, tag_size, tag_id=0, width=None, height=None,
           quiet_cells=2.0, texture_px=640, background=170,
           blur_px=0.0, noise_sigma=0.0, seed=0, supersample=SUPERSAMPLE):
    """배치 T 로 본 장면을 합성한다. uint8 BGR 을 돌려준다."""
    intr = intrinsics
    w = int(width or intr.width)
    h = int(height or intr.height)

    core_px = int(round(texture_px / CELLS)) * CELLS      # 칸 수로 나누어떨어지게
    pad_px = int(round(quiet_cells * core_px / CELLS))
    e = (float(tag_size) / 2.0) * (core_px + 2.0 * pad_px) / core_px

    # 두 단계로 거른다. 이유가 다르기 때문이다.
    vis = visibility(T, intr, tag_size, margin_px=0.0)
    if not vis["ok"]:
        raise OutOfView(vis["reason"])
    need = vis["tag_px"] / CELLS                     # 한 칸이 화면에서 몇 px 인가
    vis_q = visibility(T, intr, tag_size, margin_px=need)
    if not vis_q["ok"]:
        raise OutOfView("태그가 화면 가장자리에 붙어 흰 여백 한 칸(%.0fpx)이 안 남는다 "
                        "— 검출기가 태그 경계를 못 닫는다: %s" % (need, vis_q["reason"]))
    # 여백까지 포함한 사각형은 화면 밖으로 삐져나가도 된다(warp 가 잘라 준다).
    vis_pad = visibility(T, intr, 2.0 * e, margin_px=0.0)
    if (vis_pad["cam"][:, 2] <= 0).any():
        raise OutOfView("흰 여백 모서리가 카메라 뒤로 넘어간다 — 태그가 렌즈에 너무 가깝다")

    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    core = cv2.aruco.generateImageMarker(dic, int(tag_id), core_px)
    full = cv2.copyMakeBorder(core, pad_px, pad_px, pad_px, pad_px,
                              cv2.BORDER_CONSTANT, value=255)
    fh, fw = full.shape[:2]
    # **픽셀중심 규약**. warpPerspective 는 정수좌표를 픽셀 **중심**으로 읽는다.
    src = np.float32([[-0.5, -0.5], [fw - 0.5, -0.5],
                      [fw - 0.5, fh - 0.5], [-0.5, fh - 0.5]])   # TL, TR, BR, BL

    dst = np.float32(vis_pad["uv"])          # _object_points 순서 = (-e,-e),(e,-e),(e,e),(-e,e)
    # 질감 코너 -> 태그점 대응을 위 docstring 표대로 다시 엮는다.
    dst = np.float32([dst[2], dst[3], dst[0], dst[1]])

    M = cv2.getPerspectiveTransform(src, dst)
    canvas = np.full((h, w, 3), int(background), dtype=np.uint8)
    _warp_area(canvas, cv2.cvtColor(full, cv2.COLOR_GRAY2BGR), M, dst,
               background, supersample)

    if blur_px and blur_px > 1:
        # 수평 1차원 평균 커널 = 등속 수평 이동으로 생기는 모션블러 그 자체다.
        n = int(round(float(blur_px)))
        k = np.ones((1, n), dtype=np.float32) / float(n)
        canvas = cv2.filter2D(canvas, -1, k, borderType=cv2.BORDER_REPLICATE)
    if noise_sigma and noise_sigma > 0:
        rng = np.random.default_rng(int(seed))
        f = canvas.astype(np.float32) + rng.normal(0.0, float(noise_sigma), canvas.shape)
        canvas = np.clip(f, 0, 255).astype(np.uint8)
    return canvas


# ===========================================================================

_QB_WARNED = [False]


def quad_blur_for(noise, quad_blur=None):
    """잡음 수준에 맞는 검출기 quad_blur 를 고른다."""
    noise = float(noise)
    if quad_blur is None:
        return DEFAULT_QUAD_BLUR if noise <= NOISE_SAFE_LEVELS else NOISE_QUAD_BLUR
    quad_blur = float(quad_blur)
    if noise > NOISE_SAFE_LEVELS and quad_blur < NOISE_QUAD_BLUR:
        if not _QB_WARNED[0]:
            _QB_WARNED[0] = True
            print("잡음 %.1f 에 quad_blur=%.1f 은 검출기가 SIGSEGV 로 죽는 조합이다 — "
                  "%.1f 로 올려서 돈다.\n" % (noise, quad_blur, NOISE_QUAD_BLUR))
        return NOISE_QUAD_BLUR
    return quad_blur


def detect_in_process(gray, cfg):
    """이 프로세스에서 검출한다. 노이즈 없는 그림에서만 쓸 것."""
    det = make_detector(cfg["family"], quad_blur=cfg["quad_blur"])
    res = detect(det, gray, min_margin=cfg["min_margin"],
                 max_hamming=cfg["max_hamming"])
    return [{"tag_id": int(r.tag_id), "hamming": int(r.hamming),
             "margin": float(r.decision_margin),
             "corners": np.asarray(r.corners, dtype=float).tolist()} for r in res]


def detect_in_subprocess(gray, cfg, timeout=60.0):
    """검출을 자식 프로세스에서 돌린다. 죽으면 예외 대신 None 을 돌려준다."""
    with tempfile.NamedTemporaryFile(suffix=".npy", delete=False) as f:
        tmp = Path(f.name)
    try:
        np.save(tmp, gray)
        p = subprocess.run([sys.executable, str(Path(__file__).resolve()),
                            "--_worker", str(tmp), json.dumps(cfg)],
                           capture_output=True, timeout=timeout)
        if p.returncode != 0:
            sig = -p.returncode if p.returncode < 0 else 0
            note = ("detector crashed (%s)"
                    % ("SIG%d%s" % (sig, " = SIGSEGV" if sig == 11 else "")
                       if sig else "rc=%d" % p.returncode))
            return None, note
        return json.loads(p.stdout.decode() or "[]"), ""
    except subprocess.TimeoutExpired:
        return None, "detector timeout (%.0fs)" % timeout
    finally:
        try:
            tmp.unlink()
        except OSError:
            pass


def _worker_main(argv):
    """--_worker 로 불려 오는 자식. 검출 결과만 JSON 으로 뱉고 끝난다."""
    gray = np.load(argv[0])
    sys.stdout.write(json.dumps(detect_in_process(gray, json.loads(argv[1]))))
    return 0


def detection_from_corners(corners, tag_id=0, margin=72.0, hamming=0,
                           family=b"tag36h11"):
    """모서리 네 개로 apriltag.Detection 을 짓는다."""
    c = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    unit = np.float32([[-1, -1], [1, -1], [1, 1], [-1, 1]])   # _object_points 와 같은 순서
    H = cv2.getPerspectiveTransform(unit, c)
    return apriltag.Detection(family, int(tag_id), int(hamming), 0.0,
                              float(margin), H, c.mean(axis=0), c.astype(np.float64))


# ===========================================================================

@dataclass
class SimResult:
    """배치 하나의 결과 전부. 시각화 도구는 이걸 그대로 받아 쓰면 된다."""
    status: str
    placement: dict
    tag_size: float
    est_tag_size: float
    blur_px: float = 0.0
    noise_sigma: float = 0.0
    supersample: int = SUPERSAMPLE
    truth: dict = field(default_factory=dict)
    meas: dict = field(default_factory=dict)
    err: dict = field(default_factory=dict)
    tag_px: float = float("nan")
    tag_px_truth: float = float("nan")
    reproj_rms_px: float = float("nan")
    margin: float = float("nan")
    corner_res_px: float = float("nan")     # 검출 모서리 - 투영 모서리. 대부분 검출기의
    #                                       +0.5px 규약 오프셋이다(위 함정 1). 렌더러 탓이 아니다.
    render_res_px: float = float("nan")     # 그 +0.5px 를 뺀 나머지 = **렌더러의 성적표**.
    #                                       supersample 을 올리면 줄어드는 쪽이 이것이다.
    rel_approach: bool = False
    rel_tilt: bool = False
    quality_ok: bool = False
    reasons: tuple = ()
    note: str = ""
    T_truth: np.ndarray = None
    T_meas: np.ndarray = None
    image: np.ndarray = None                # --analytic 이면 None

    def flat(self):
        """CSV 한 줄용 평평한 dict."""
        r = dict(self.placement)
        r.update({"tag_size": self.tag_size, "est_tag_size": self.est_tag_size,
                  "blur_px": self.blur_px, "noise_sigma": self.noise_sigma,
                  "supersample": self.supersample, "status": self.status})
        for k in KEYS:
            r["truth_" + k] = self.truth.get(k, float("nan"))
            r["meas_" + k] = self.meas.get(k, float("nan"))
            r["err_" + k] = self.err.get(k, float("nan"))
        r.update({"tag_px": self.tag_px, "tag_px_truth": self.tag_px_truth,
                  "reproj_rms_px": self.reproj_rms_px, "margin": self.margin,
                  "corner_res_px": self.corner_res_px,
                  "render_res_px": self.render_res_px,
                  "rel_approach": int(self.rel_approach),
                  "rel_tilt": int(self.rel_tilt),
                  "quality_ok": int(self.quality_ok),
                  "reasons": "|".join(self.reasons), "note": self.note})
        return r


#: 표/CSV 에 올리는 값들. 순서가 곧 표의 순서다.
KEYS = ("distance", "lateral", "forward", "vertical", "heading", "tilt", "z")
#: (키, 표시이름, 단위, 오차단위, 오차환산) — 오차는 mm/도로 찍는다. 현장 단위다.
ROWS = [("distance", "distance", "m", "mm", 1000.0),
        ("lateral",  "lateral",  "m", "mm", 1000.0),
        ("forward",  "forward",  "m", "mm", 1000.0),
        ("vertical", "vertical", "m", "mm", 1000.0),
        ("z",        "z(optical)", "m", "mm", 1000.0),
        ("heading",  "heading",  "deg", "deg", 1.0),
        ("tilt",     "tag tilt", "deg", "deg", 1.0)]


def simulate_one(placement, intr, tag_size, est_tag_size=None, tag_id=0,
                 blur_px=0.0, noise_sigma=0.0, seed=0, method="auto",
                 analytic=False, cfg=None, keep_image=False,
                 supersample=SUPERSAMPLE):
    """배치 하나를 그려서 다시 재고 SimResult 를 돌려준다."""
    cfg = cfg or dict(family="tag36h11", quad_blur=DEFAULT_QUAD_BLUR,
                      min_margin=0.0, max_hamming=0)
    est = float(est_tag_size or tag_size)
    T = placement_to_T(**placement)
    tr = truth_of(T)
    base = dict(placement=dict(placement), tag_size=float(tag_size),
                est_tag_size=est, blur_px=float(blur_px),
                noise_sigma=float(noise_sigma),
                supersample=(0 if analytic else int(supersample)),
                truth=tr, T_truth=T,
                rel_approach=tr["rel_approach"], rel_tilt=tr["rel_tilt"])

    vis = visibility(T, intr, tag_size)
    base["tag_px_truth"] = vis["tag_px"]

    img = None
    if analytic:
        if blur_px or noise_sigma:
            raise SystemExit("--analytic 은 블러/노이즈를 못 넣는다 — 그건 그림이 "
                             "있어야 하는 이야기다. --analytic 을 빼고 돌릴 것")
        if not vis["ok"]:
            return SimResult(status="out_of_view", note=vis["reason"], **base)
        dets = [{"tag_id": int(tag_id), "hamming": 0, "margin": 72.0,
                 "corners": vis["uv"].tolist()}]
    else:
        try:
            img = render(T, intr, tag_size, tag_id=tag_id, blur_px=blur_px,
                         noise_sigma=noise_sigma, seed=seed,
                         supersample=supersample)
        except OutOfView as exc:
            return SimResult(status="out_of_view", note=str(exc), **base)
        gray = to_gray(img)
        if noise_sigma > 0:
            dets, note = detect_in_subprocess(gray, cfg)
            if dets is None:
                r = SimResult(status="crashed", note=note, **base)
                r.image = img if keep_image else None
                return r
        else:
            dets = detect_in_process(gray, cfg)

    if keep_image:
        base_img = img
    else:
        base_img = None

    want = [d for d in dets if d["tag_id"] == int(tag_id)] or dets
    if not want:
        r = SimResult(status="no_detection",
                      note="검출 0개 (blur=%.0fpx noise=%.0f, 태그 %.0fpx)"
                           % (blur_px, noise_sigma, vis["tag_px"]), **base)
        r.image = base_img
        return r

    d = want[0]
    det = detection_from_corners(d["corners"], tag_id=d["tag_id"],
                                 margin=d["margin"], hamming=d["hamming"],
                                 family=cfg["family"].encode())
    detector = make_detector(cfg["family"], quad_blur=cfg["quad_blur"])
    T_meas, _e0, _e1 = estimate_pose(detector, det, intr, est, method=method)
    q = pose_quality(detector, det, intr, est, T_meas, method=method)
    st = docking_state(T_meas)
    v = pose_to_xyzrpy(T_meas)
    meas = {"distance": st["distance"], "lateral": st["lateral"],
            "forward": st["forward"], "vertical": st["vertical"],
            "heading": st["heading_deg"], "tilt": tag_tilt_deg(T_meas),
            "z": v["z"]}
    err = {k: meas[k] - tr[k] for k in KEYS}

    r = SimResult(status="ok", meas=meas, err=err,
                  tag_px=q["tag_px"], reproj_rms_px=q["reproj_rms_px"],
                  margin=q["decision_margin"],
                  corner_res_px=float(np.linalg.norm(
                      np.asarray(d["corners"]) - vis["uv"], axis=1).mean()),
                  render_res_px=float(np.linalg.norm(
                      np.asarray(d["corners"]) - DETECTOR_CORNER_OFFSET_PX
                      - vis["uv"], axis=1).mean()),
                  quality_ok=bool(q["ok"]), reasons=tuple(q["reasons"]),
                  T_meas=T_meas, **base)
    # reliable 은 **추정값 기준**으로 덮어쓴다 — 실제 제어가 보는 것은 이쪽이다.
    r.rel_approach = bool(st["reliable_angle"])
    r.rel_tilt = bool(q["reliable_angle"])
    r.image = base_img
    return r


# ===========================================================================

def print_single(r, intr, res_name):
    """단일 배치: 정답 vs 추정 vs 오차."""
    p = r.placement
    print("배치 (바닥 기준)  태그높이 %.2fm / 카메라높이 %.2fm / 거리 %.2fm / "
          "좌우 %+.2fm / heading %+.1f도"
          % (p["tag_height"], p["cam_height"], p["distance"],
             p["lateral"], p["heading_deg"]))
    print("카메라            %s  fx=%.1f fy=%.1f cx=%.1f cy=%.1f  왜곡 없음 (D435i 컬러 실측)"
          % (res_name, intr.fx, intr.fy, intr.cx, intr.cy))
    print("태그              %.3f m (추정 %.3f m%s), 정답 화면크기 %.1f px"
          % (r.tag_size, r.est_tag_size,
             "  ** 일부러 어긋냄 = 순수 배율오차 **"
             if abs(r.tag_size - r.est_tag_size) > 1e-9 else "", r.tag_px_truth))
    if r.blur_px or r.noise_sigma:
        print("열화              블러 %.0f px / 잡음 sd %.1f" % (r.blur_px, r.noise_sigma))
    print()

    if r.status != "ok":
        print("!! %s — %s" % (r.status, r.note))
        print("   (정답은 여전히 안다: forward %.3f m, lateral %+.3f m, heading %+.2f도)"
              % (r.truth["forward"], r.truth["lateral"], r.truth["heading"]))
        return

    print("%-10s %12s %12s %12s" % ("", "truth", "recovered", "error"))
    print("-" * 50)
    for k, name, unit, eunit, scale in ROWS:
        print("%-10s %12.4f %12.4f %9.3f %s"
              % (name, r.truth[k], r.meas[k], r.err[k] * scale, eunit))
    print("-" * 50)
    # 재투영오차는 자세가 NaN 이면 10^155 같은 값으로 나온다(detection_pose 가
    e = r.reproj_rms_px
    es = "%.3f" % e if np.isfinite(e) and e < 1e6 else "발산(%.1e)" % e
    print("tag_px          %.1f   (MIN %.0f / STABLE %.0f)   재투영 RMS %s px   margin %.1f"
          % (r.tag_px, MIN_TAG_PX, STABLE_TAG_PX, es, r.margin))
    if not np.isfinite(np.asarray(r.T_meas, dtype=float)).all():
        print("                자세가 NaN 이다 — method=tag 는 **정확히 정면**(tilt 0.00도)에서"
              " 퇴화한다\n"
              "                (라이브러리의 \"had ta normalize!\"). auto/pnp 로 돌리면 나온다.\n"
              "                위 tag tilt 0.00 은 진짜가 아니다: tag_tilt_deg 의"
              " min(1.0, nan) 이 1.0 이라 arccos(1)=0 이 된 것뿐이다.")
    print("모서리 잔차     %.3f px  (검출 - 투영. 대부분 검출기의 +0.5px 규약 "
          "오프셋이다 — 실촬영에도 있다)" % r.corner_res_px)
    print("                %.3f px  그 +0.5 를 뺀 나머지 = **렌더러의 몫**"
          " (supersample %d)" % (r.render_res_px, r.supersample))
    print("reliable_angle  approach %s (approach %.2f도, 임계 10) / "
          "tilt %s (tilt %.2f도, 임계 %.0f)"
          % ("O" if r.rel_approach else "X", r.truth["approach"],
             "O" if r.rel_tilt else "X", r.meas["tilt"], RELIABLE_TILT_DEG))
    print("pose_quality    %s%s"
          % ("ok" if r.quality_ok else "NG", ""
             if r.quality_ok else "  <- " + ", ".join(r.reasons)))


SWEEP_AXES = {
    "distance": ("placement", "distance", "m"),
    "lateral":  ("placement", "lateral", "m"),
    "heading":  ("placement", "heading_deg", "도"),
    "blur":     ("degrade", "blur_px", "px"),
    "noise":    ("degrade", "noise_sigma", "레벨"),
    "tag-size": ("tag", "tag_size", "m"),
}


def fix_sweep_argv(argv):
    """'--sweep lateral -1.5:1.5:0.5' 를 argparse 가 삼킬 수 있게 고친다."""
    out, i = [], 0
    while i < len(argv):
        a = argv[i]
        if (a == "--sweep" and i + 2 < len(argv)
                and argv[i + 2].startswith("-") and ":" in argv[i + 2]):
            out += [a, argv[i + 1], " " + argv[i + 2]]
            i += 3
            continue
        out.append(a)
        i += 1
    return out


def parse_spec(spec):
    """'start:stop:step' -> 값 배열. stop 포함이다."""
    try:
        a, b, s = (float(x) for x in spec.split(":"))
    except ValueError:
        raise SystemExit("--sweep 두 번째 인자는 start:stop:step 이다 (예: 0:40:2)")
    if s == 0:
        raise SystemExit("step 이 0 이다")
    if (b - a) * s < 0:
        raise SystemExit("범위와 step 의 방향이 반대다: %s -> %s 인데 step 이 %+g 다. "
                         "거꾸로 쓸려면 step 을 음수로 줄 것 (예: %g:%g:%g)"
                         % (a, b, s, a, b, -abs(s) if b < a else abs(s)))
    n = int(np.floor((b - a) / s + 1e-9)) + 1
    return [a + i * s for i in range(max(n, 1))]


def run_sweep(axis, values, args, intr, cfg, repeat=1):
    """축 하나를 쓸며 배치마다 SimResult 를 모은다."""
    kind, name, _unit = SWEEP_AXES[axis]
    out = []
    for v in values:
        place = dict(tag_height=args.tag_height, cam_height=args.cam_height,
                     distance=args.distance, lateral=args.lateral,
                     heading_deg=args.heading)
        tag_size, blur, noise = args.tag_size, args.blur, args.noise
        if kind == "placement":
            place[name] = v
        elif kind == "degrade":
            if name == "blur_px":
                blur = v
            else:
                noise = v
        else:
            tag_size = v
        # repeat 0/음수를 그대로 쓰면 결과가 한 개도 안 쌓여 print_sweep 이
        n = max(1, int(repeat)) if (noise > 0) else 1
        for k in range(n):
            out.append(simulate_one(
                place, intr, tag_size,
                est_tag_size=(args.est_tag_size if args.est_tag_size else
                              (tag_size if kind == "tag" else args.tag_size)),
                tag_id=args.tag_id, blur_px=blur, noise_sigma=noise,
                seed=args.seed + k, method=args.method, analytic=args.analytic,
                # 쓸기 점마다 잡음이 다르므로 quad_blur 도 점마다 고른다.
                cfg=dict(cfg, quad_blur=quad_blur_for(noise, args.quad_blur)),
                supersample=args.supersample))
            out[-1].placement["_sweep"] = v
    return out


def print_sweep(axis, results, values):
    """쓸기 표. 값마다 한 줄로 접는다(반복이 있으면 평균 + 검출률)."""
    _kind, _name, unit = SWEEP_AXES[axis]
    print("%-9s %7s %6s %8s %8s %8s %8s %7s %7s  %s"
          % (axis + "[" + unit + "]", "tag_px", "det", "d[mm]", "lat[mm]",
             "fwd[mm]", "hdg[deg]", "tilt", "reproj", "상태"))
    print("-" * 96)
    for v in values:
        g = [r for r in results if r.placement.get("_sweep") == v]
        ok = [r for r in g if r.status == "ok"]
        rate = "%d/%d" % (len(ok), len(g))
        bad = sorted({r.status for r in g if r.status != "ok"})
        note = ""
        if not ok:
            note = "; ".join(sorted({r.note for r in g}))[:38]
            print("%-9.3f %7.1f %6s %8s %8s %8s %8s %7s %7s  %s"
                  % (v, g[0].tag_px_truth, rate, "-", "-", "-", "-", "-", "-",
                     (",".join(bad) + " " + note).strip()))
            continue
        m = lambda k, s=1.0: float(np.mean([r.err[k] for r in ok])) * s
        flags = ("rel:%s%s" % ("A" if ok[0].rel_approach else "-",
                               "T" if ok[0].rel_tilt else "-"))
        if bad:
            flags += " " + ",".join(bad)
        if not ok[0].quality_ok:
            flags += " q:" + ",".join(ok[0].reasons)
        print("%-9.3f %7.1f %6s %8.2f %8.2f %8.2f %8.3f %7.3f %7.3f  %s"
              % (v, float(np.mean([r.tag_px for r in ok])), rate,
                 m("distance", 1000), m("lateral", 1000), m("forward", 1000),
                 m("heading"), m("tilt"), 
                 float(np.mean([r.reproj_rms_px for r in ok])), flags))
    print("-" * 96)
    print("오차 = 추정 - 정답. det = 검출 성공/시도. rel:A=docking approach>=10도, "
          "T=tilt>=%.0f도" % RELIABLE_TILT_DEG)


def write_csv(path, results):
    if not results:
        print("\ncsv: 쓸 게 없다 (결과 0줄)")
        return
    rows = [r.flat() for r in results]
    for r, row in zip(results, rows):
        row["sweep_value"] = r.placement.get("_sweep", "")
        row.pop("_sweep", None)
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print("\ncsv: %s (%d줄)" % (p, len(rows)))


# ===========================================================================

SELF_CHECK_CASES = [
    (1.60, 1.20, 3.00, -1.03, 20.0), (1.60, 1.20, 3.00, 1.03, -20.0),
    (1.60, 1.20, 2.00, 0.00, 0.0),   (1.60, 1.20, 2.00, 0.00, 15.0),
    (1.60, 1.20, 2.00, 0.60, 0.0),   (1.20, 1.20, 1.50, -0.30, 35.0),
    (0.80, 1.60, 4.00, 0.90, -45.0), (2.40, 1.20, 5.00, -2.00, 30.0),
    (1.60, 1.20, 0.80, 0.10, 5.0),   (1.60, 1.20, 6.00, -0.05, -3.0),
    (1.75, 1.05, 2.50, 1.50, 60.0),  (1.60, 1.20, 3.00, 0.00, -89.0),
    (1.60, 1.20, 10.0, 3.00, -16.0),
]


def self_check(intr, tag_size):
    """정답 구성 -> docking_state 왕복이 정확한지 전부 확인한다."""
    print("%5s %5s %5s %7s %8s | %9s %7s %7s %6s %7s %s"
          % ("tag_h", "cam_h", "dist", "lat", "hdg", "max|err|", "vert",
             "appr", "tilt", "tag_px", "가시성"))
    worst = 0.0
    for (th, ch, d, lat, hd) in SELF_CHECK_CASES:
        T = placement_to_T(th, ch, d, lat, hd, check=False)
        st = assert_roundtrip(T, th, ch, d, lat, hd)
        V = ch - th
        e = {"lateral": st["lateral"] - lat, "vertical": st["vertical"] - V,
             "forward": st["forward"] - d, "heading": st["heading_deg"] - hd,
             "distance": st["distance"] - float(np.linalg.norm([lat, V, d])),
             "approach": st["approach_deg"] - float(np.degrees(np.arctan2(abs(lat), abs(d)))),
             "tilt-|hdg|": tag_tilt_deg(T) - abs(hd)}
        m = max(abs(x) for x in e.values())
        worst = max(worst, m)
        vis = visibility(T, intr, tag_size)
        print("%5.2f %5.2f %5.2f %+7.2f %+8.2f | %9.2e %+7.3f %7.3f %6.2f %7.1f %s"
              % (th, ch, d, lat, hd, m, st["vertical"], st["approach_deg"],
                 tag_tilt_deg(T), vis["tag_px"],
                 "O" if vis["ok"] else "X " + vis["reason"][:34]))
    print("\n모든 항목/모든 배치 최악 오차: %.3e  (1e-12 미만: %s)"
          % (worst, worst < 1e-12))
    print("가시성 X 는 버그가 아니다 — 그 배치에서 태그는 정말 안 찍힌다. "
          "수평 카메라 + 높이차라 근거리(0.80m)에서 태그가 위로 벗어나고, "
          "heading 이 커지면(60/89도) 태그면이 카메라 뒤로 돈다.")
    return worst


# ===========================================================================

def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--_worker":
        return _worker_main(sys.argv[2:])

    ap = argparse.ArgumentParser(
        description="AprilTag 도킹 — 정답을 구성한 합성 장면으로 자세 정확도를 잰다 "
                    "(카메라 불필요)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="예)  simulate.py --sweep heading 0:40:2\n"
               "     simulate.py --sweep blur 0:40:4\n"
               "     simulate.py --self-check")
    d = DEFAULT_PLACEMENT
    ap.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE, metavar="M",
                    help="태그 한 변 [m], 검은 테두리 바깥까지 (기본 %.2f)" % DEFAULT_TAG_SIZE)
    ap.add_argument("--tag-height", type=float, default=d["tag_height"], metavar="M",
                    help="바닥에서 태그 중심까지 [m]")
    ap.add_argument("--cam-height", type=float, default=d["cam_height"], metavar="M",
                    help="바닥에서 카메라 렌즈까지 [m]")
    ap.add_argument("--distance", type=float, default=d["distance"], metavar="M",
                    help="태그 면까지의 수직 거리 [m] (광축 z 도 직선거리도 아니다)")
    ap.add_argument("--lateral", type=float, default=d["lateral"], metavar="M",
                    help="정면축에서 좌우 [m]. 양수 = 태그를 마주본 사람 기준 왼쪽")
    ap.add_argument("--heading", type=float, default=d["heading_deg"], metavar="DEG",
                    help="광축이 태그 축과 이루는 각 [도]. 이 구성에서 tag_tilt 와 같다")
    ap.add_argument("--res", default=DEFAULT_RES, choices=sorted(PRESETS),
                    help="카메라 내부파라미터 (D435i 컬러 공장값 실측, 왜곡 0)")
    ap.add_argument("--tag-id", type=int, default=0)
    ap.add_argument("--est-tag-size", type=float, default=None, metavar="M",
                    help="추정에만 쓸 태그 크기. 렌더와 다르게 주면 순수 배율오차 시험이 "
                         "된다 (거리만 그 비율로 틀어지고 각도는 안 변한다)")
    ap.add_argument("--blur", type=float, default=0.0, metavar="PX",
                    help="수평 모션블러 폭 [px]. 실측: 10px 까지 검출 100%%(28/28), "
                         "32px 에서 0%%(0/28). 한계는 **tag_px 대비**다 — 이 도구로 "
                         "tag_px 56 이면 16px, tag_px 136 이면 28px 에서 끊겼다 "
                         "(둘 다 태그 한 변의 약 1/5)")
    ap.add_argument("--noise", type=float, default=0.0, metavar="SIGMA",
                    help="가우시안 잡음 sd [그레이레벨]. **검출기가 SIGSEGV 로 죽는다** — "
                         "그래서 이 옵션을 쓰면 검출만 자식 프로세스에서 돌리고 "
                         "죽은 배치는 'detector crashed' 로 적는다")
    ap.add_argument("--repeat", type=int, default=1, metavar="N",
                    help="--noise 가 켜졌을 때 배치마다 다른 씨앗으로 N 번. 검출률을 "
                         "분수로 보려면 필요하다 (블러는 결정적이라 1 로 충분)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--method", default="auto", choices=["auto", "tag", "pnp"],
                    help="auto=detection_pose 쓰고 NaN 이면 PnP, tag=detection_pose 만, "
                         "pnp=solvePnP 만. **정확히 정면(tilt 0.00도)에서는 tag 가 NaN 이다**")
    ap.add_argument("--supersample", type=int, default=SUPERSAMPLE, metavar="N",
                    help="렌더러가 출력 픽셀 하나를 한 변 N 등분해 굽고 면적평균한다 "
                         "(기본 %d). 진짜 센서가 픽셀 면적으로 빛을 적분하는 것을 "
                         "흉내내는 것이라 **경계에 회색이 생기고 부화소 모서리가 산다**. "
                         "1 로 내리면 옛 점샘플 동작 — 태그 36px 배치에서 lateral 오차가 "
                         "59mm 에서 2201mm 로 뛴다. 비교해 보라고만 남겨 둔 값이다"
                    % SUPERSAMPLE)
    ap.add_argument("--analytic", action="store_true",
                    help="렌더링 없이 투영 모서리를 바로 쓴다. 약 100배 빠르고 렌더러 "
                         "오차 바닥(0.5px)이 없다. 블러/노이즈는 못 쓴다")
    ap.add_argument("--sweep", nargs=2, metavar=("AXIS", "START:STOP:STEP"),
                    help="축 하나를 쓸며 표 + CSV. 축: " + ", ".join(sorted(SWEEP_AXES)))
    ap.add_argument("--csv", default=None, metavar="PATH",
                    help="쓸기 결과 CSV 경로 (기본 work_dirs/sim_<축>.csv)")
    ap.add_argument("--save", default=None, metavar="PATH",
                    help="단일 배치의 합성 이미지를 PNG 로 저장")
    ap.add_argument("--self-check", action="store_true",
                    help="정답 구성이 docking_state 와 맞물리는지 13개 배치로 확인")
    ap.add_argument("--family", default="tag36h11")
    ap.add_argument("--quad-blur", type=float, default=None, metavar="SIGMA",
                    help="검출기 quad_blur. 안 주면 잡음에 맞춰 고른다 (잡음 %.0f 이하면 "
                         "%.1f, 넘으면 %.1f — 0 인 채로 잡음을 주면 검출기가 SIGSEGV 로 "
                         "죽어서 그 배치가 통째로 빈다)"
                    % (NOISE_SAFE_LEVELS, DEFAULT_QUAD_BLUR, NOISE_QUAD_BLUR))
    ap.add_argument("--min-margin", type=float, default=0.0)
    ap.add_argument("--max-hamming", type=int, default=0)
    args = ap.parse_args(fix_sweep_argv(sys.argv[1:]))

    intr = PRESETS[args.res]
    cfg = dict(family=args.family, quad_blur=quad_blur_for(args.noise, args.quad_blur),
               min_margin=args.min_margin, max_hamming=args.max_hamming)

    if args.self_check:
        worst = self_check(intr, args.tag_size)
        return 0 if worst < 1e-12 else 1

    if args.sweep:
        axis, spec = args.sweep
        if axis not in SWEEP_AXES:
            raise SystemExit("모르는 축: %s (가능: %s)"
                             % (axis, ", ".join(sorted(SWEEP_AXES))))
        values = parse_spec(spec)
        print("쓸기: %s = %s ... %s (%d점)   기준 배치(쓸리는 축은 덮인다): "
              "태그 %.2fm / 높이 %.2f-%.2f / 거리 %.2fm / 좌우 %+.2fm / heading %+.1f도"
              % (axis, values[0], values[-1], len(values), args.tag_size,
                 args.tag_height, args.cam_height, args.distance,
                 args.lateral, args.heading))
        print("카메라 %s  경로 %s\n" % (args.res, "analytic" if args.analytic
                                       else "render+detect"))
        res = run_sweep(axis, values, args, intr, cfg, repeat=args.repeat)
        print_sweep(axis, res, values)
        write_csv(args.csv or (ROOT / "work_dirs" / ("sim_%s.csv" % axis)), res)
        return 0

    place = dict(tag_height=args.tag_height, cam_height=args.cam_height,
                 distance=args.distance, lateral=args.lateral,
                 heading_deg=args.heading)
    r = simulate_one(place, intr, args.tag_size, est_tag_size=args.est_tag_size,
                     tag_id=args.tag_id, blur_px=args.blur,
                     noise_sigma=args.noise, seed=args.seed, method=args.method,
                     analytic=args.analytic, cfg=cfg,
                     supersample=args.supersample, keep_image=bool(args.save))
    print_single(r, intr, args.res)
    if args.save and r.image is not None:
        p = Path(args.save)
        p.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(p), r.image)
        print("\nimage: %s (%dx%d)" % (p, r.image.shape[1], r.image.shape[0]))
    return 0 if r.status == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
