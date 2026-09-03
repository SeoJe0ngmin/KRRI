"""시뮬레이터 엔진 — 배치를 정해 그림을 그리고 정답을 냄.

    placement_to_T   바닥 기준 배치 -> 4x4 자세 (정답)
    truth_of         그 자세의 도킹값 (정답)
    render           태그가 그렇게 보이는 그림을 만듦
    visibility       화면 안에 들어오나

정답을 **우리가 구성하기 때문에** 소수점 열두 자리까지 정확함.
카메라도 줄자도 필요 없음. 실카메라 검증은 tools/verify.py 가 함.

이 파일은 손으로 쓰는 게 아니라 tools/sim.py 가 부름.
"""
import os
import sys
from dataclasses import dataclass, field

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config.sim import CAM_HEIGHT_M, TAG_HEIGHT_M                    # noqa: E402
from config.system import TAG_CELLS as CELLS                            # noqa: E402
from config.system import TAG_SIZE_M as DEFAULT_TAG_SIZE                # noqa: E402
from config.system import RELIABLE_TILT_DEG                             # noqa: E402
from src.models import (CameraIntrinsics, docking_state,             # noqa: E402
                        pose_to_xyzrpy, tag_pixel_size, tag_tilt_deg)
from src.models.detection.detection_pose import _object_points       # noqa: E402
from src.utils.util import invert_T, pr2t                            # noqa: E402


#: 해상도별 카메라 값. D435i 공장 보정값.
PRESETS = {
    "1920x1080": CameraIntrinsics(1359.2, 1359.0, 956.9, 571.3, 1920, 1080),
    "1280x720":  CameraIntrinsics(906.1,  906.0,  637.9, 380.9, 1280, 720),
    "640x480":   CameraIntrinsics(604.1,  604.0,  318.6, 253.9, 640,  480),
}
DEFAULT_RES = "1280x720"

#: 표에 올리는 값들. 순서가 곧 표의 순서.
KEYS = ("distance", "lateral", "forward", "vertical", "heading", "tilt", "z")

#: 한 출력 픽셀당 한 변 몇 번을 샘플링할지. 렌더러의 **핵심 파라미터**다.
SUPERSAMPLE = 4


#: 슈퍼샘플 버퍼 상한 [픽셀]. 넘으면 배율을 자동으로 낮춤(품질보다 안 죽는 게 나음).
_MAX_SS_PIXELS = 24_000_000


def placement_to_T(tag_height=TAG_HEIGHT_M, cam_height=CAM_HEIGHT_M, distance=3.00,
                   lateral=-1.03, heading_deg=20.0, check=True):
    """바닥 기준 배치를 T_camera_tag (4x4) 로 바꿈."""
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
    """docking_state(T) 가 우리가 넣은 배치를 그대로 돌려주는지 봄."""
    st = docking_state(T)
    want = {"lateral": float(lateral),
            "vertical": float(cam_height) - float(tag_height),
            "forward": float(distance)}
    bad = {k: (st[k], v) for k, v in want.items() if abs(st[k] - v) > tol}
    # heading 은 atan2 라 (-180, 180] 로 접혀 나옴. 접은 뒤에 비교해야 함.
    dh = (st["heading_deg"] - float(heading_deg) + 180.0) % 360.0 - 180.0
    if abs(dh) > tol:
        bad["heading_deg"] = (st["heading_deg"], float(heading_deg))
    # tilt 는 **|heading| 이 아님.** tag_tilt_deg 는 arccos(|cos h|) 라 0..90 로
    want_tilt = np.degrees(np.arccos(abs(np.cos(np.radians(float(heading_deg))))))
    if abs(tag_tilt_deg(T) - want_tilt) > 1e-6:
        bad["tilt=arccos|cos h|"] = (tag_tilt_deg(T), want_tilt)
    if bad:
        raise AssertionError("배치 구성이 docking_state 와 안 맞는다: %s" % bad)
    return st


def truth_of(T):
    """정답 T 에서 표에 올릴 값들을 뽑음. 추정쪽과 **같은 함수**로 뽑음."""
    st = docking_state(T)
    v = pose_to_xyzrpy(T)
    return {"distance": st["distance"], "lateral": st["lateral"],
            "forward": st["forward"], "vertical": st["vertical"],
            "heading": st["heading_deg"], "tilt": tag_tilt_deg(T),
            "z": v["z"], "approach": st["approach_deg"],
            "rel_approach": bool(st["reliable_angle"]),
            "rel_tilt": bool(tag_tilt_deg(T) >= RELIABLE_TILT_DEG)}


class OutOfView(Exception):
    """이 배치에서는 태그가 화면에 안 잡힘. 렌더를 거부함."""


class _Corners:
    """tag_pixel_size() 에 넘길 최소 껍데기. 그 함수는 .corners 만 봄."""

    def __init__(self, corners):
        self.corners = np.asarray(corners, dtype=np.float64)


def project_corners(T, intr, tag_size):
    """태그 네 모서리를 픽셀로 투영함. detection.corners 와 **같은 순서**다."""
    T = np.asarray(T, dtype=np.float64)
    # 이 투영은 **순수 핀홀**임. 왜곡계수가 실린 intrinsics 로 부르면 그림은 왜곡
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
    """태그가 정말 찍히는가. **렌더 전에** 반드시 통과시킴."""
    uv, cam = project_corners(T, intr, tag_size)
    w = intr.width or PRESETS[DEFAULT_RES].width
    h = intr.height or PRESETS[DEFAULT_RES].height
    behind = int((cam[:, 2] <= 0).sum())
    lo, hi_w, hi_h = margin_px, w - margin_px, h - margin_px
    outside = int(((uv[:, 0] < lo) | (uv[:, 0] >= hi_w) |
                   (uv[:, 1] < lo) | (uv[:, 1] >= hi_h)).sum())
    # 화면 크기는 **검출 결과와 같은 정의**로 재야 비교가 됨. tag_pixel_size 는
    tag_px = float(tag_pixel_size(_Corners(uv))) if behind == 0 else float("nan")

    reason = ""
    if behind:
        reason = "모서리 %d개가 카메라 뒤에 있다" % behind
    elif outside:
        reason = ("모서리 %d개가 화면(%dx%d) 밖이다 — 태그 중심이 화면에서 "
                  "(%.0f, %.0f)" % (outside, w, h, uv[:, 0].mean(), uv[:, 1].mean()))
    return {"ok": not reason, "reason": reason, "uv": uv, "cam": cam,
            "tag_px": tag_px}


def _warp_area(canvas, texture, M, dst_uv, background, supersample):
    """질감을 canvas 에 **면적평균**으로 구움. 진짜 카메라가 하는 일이 이것."""
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
        return canvas                       # 태그가 화면과 안 겹침
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
    """배치 T 로 본 장면을 합성함. uint8 BGR 을 돌려줌."""
    intr = intrinsics
    w = int(width or intr.width)
    h = int(height or intr.height)

    core_px = int(round(texture_px / CELLS)) * CELLS      # 칸 수로 나누어떨어지게
    pad_px = int(round(quiet_cells * core_px / CELLS))
    e = (float(tag_size) / 2.0) * (core_px + 2.0 * pad_px) / core_px

    # 두 단계로 거름. 이유가 다르기 때문.
    vis = visibility(T, intr, tag_size, margin_px=0.0)
    if not vis["ok"]:
        raise OutOfView(vis["reason"])
    need = vis["tag_px"] / CELLS                     # 한 칸이 화면에서 몇 px 인가
    vis_q = visibility(T, intr, tag_size, margin_px=need)
    if not vis_q["ok"]:
        raise OutOfView("태그가 화면 가장자리에 붙어 흰 여백 한 칸(%.0fpx)이 안 남는다 "
                        "— 검출기가 태그 경계를 못 닫는다: %s" % (need, vis_q["reason"]))
    # 여백까지 포함한 사각형은 화면 밖으로 삐져나가도 됨(warp 가 잘라 줌).
    vis_pad = visibility(T, intr, 2.0 * e, margin_px=0.0)
    if (vis_pad["cam"][:, 2] <= 0).any():
        raise OutOfView("흰 여백 모서리가 카메라 뒤로 넘어간다 — 태그가 렌즈에 너무 가깝다")

    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    core = cv2.aruco.generateImageMarker(dic, int(tag_id), core_px)
    full = cv2.copyMakeBorder(core, pad_px, pad_px, pad_px, pad_px,
                              cv2.BORDER_CONSTANT, value=255)
    fh, fw = full.shape[:2]
    # **픽셀중심 규약**. warpPerspective 는 정수좌표를 픽셀 **중심**으로 읽음.
    src = np.float32([[-0.5, -0.5], [fw - 0.5, -0.5],
                      [fw - 0.5, fh - 0.5], [-0.5, fh - 0.5]])   # TL, TR, BR, BL

    dst = np.float32(vis_pad["uv"])          # _object_points 순서 = (-e,-e),(e,-e),(e,e),(-e,e)
    # 질감 코너 -> 태그점 대응을 위 docstring 표대로 다시 엮음.
    dst = np.float32([dst[2], dst[3], dst[0], dst[1]])

    M = cv2.getPerspectiveTransform(src, dst)
    canvas = np.full((h, w, 3), int(background), dtype=np.uint8)
    _warp_area(canvas, cv2.cvtColor(full, cv2.COLOR_GRAY2BGR), M, dst,
               background, supersample)

    if blur_px and blur_px > 1:
        # 수평 1차원 평균 커널 = 등속 수평 이동으로 생기는 모션블러 그 자체.
        n = int(round(float(blur_px)))
        k = np.ones((1, n), dtype=np.float32) / float(n)
        canvas = cv2.filter2D(canvas, -1, k, borderType=cv2.BORDER_REPLICATE)
    if noise_sigma and noise_sigma > 0:
        rng = np.random.default_rng(int(seed))
        f = canvas.astype(np.float32) + rng.normal(0.0, float(noise_sigma), canvas.shape)
        canvas = np.clip(f, 0, 255).astype(np.uint8)
    return canvas

