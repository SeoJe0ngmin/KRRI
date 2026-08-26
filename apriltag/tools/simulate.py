"""정답을 **지어낸** 장면으로 자세 추정을 잰다. 카메라도 줄자도 없이.

tools/verify.py 는 줄자로 잰 정답과 비교한다. 이 도구는 정답을 우리가
**구성**한다 — 그래서 오차가 소수점 열두 자리까지 정확하고, 하드웨어가 한 개도
필요 없다. 줄자로는 못 묻는 질문에 답하려고 만들었다.

    * 각도를 0.5도씩 쓸어 가며 언제부터 heading 을 믿을 수 있나 (reliable 임계)
    * 모션블러 몇 px 부터 검출이 끊기나
    * 센서 노이즈 몇 레벨부터 자세가 무너지나
    * 태그가 화면 밖으로 나가는 배치는 어디부터인가

**카메라는 뽑혀 있다(rs.context().query_devices() == 0).** 이 파일은 pyrealsense2
경로를 아예 import 하지 않는다 — 실수로도 장치를 잡을 수 없게 하려는 것이다.
여기서 나온 숫자는 전부 합성이며, 실촬영 결과라고 보고해서는 안 된다.

── 배치를 적는 법: 바닥이 기준이다 ────────────────────────────────────────
실제 창고에서 줄자로 재는 순서 그대로 받는다.

    --tag-size    0.20   태그 한 변 [m], 검은 테두리 바깥까지
    --tag-height  1.60   바닥에서 태그 중심까지 [m]
    --cam-height  1.20   바닥에서 카메라 렌즈까지 [m]
    --distance    3.00   태그 정면축을 따라 태그면까지 [m]
    --lateral    -1.03   정면축에서 좌우 [m]  (부호가 좌우를 가른다)
    --heading     20     지게차가 축과 이루는 각 [도]

카메라는 **수평**, 태그는 **수직**으로 못박혀 있다(사용자 결정).
--cam-pitch / --cam-roll / --tag-roll 은 없다. 자유도를 늘리면 "부호가 어디서
뒤집혔나"를 다시 못 찾는다.

── 이 도구가 답하는 세 개의 거리는 서로 다른 숫자다 ────────────────────────
기본 배치(1.60/1.20/3.00/-1.03/20도)에서

    docking.forward       3.000   태그면까지 **수직** 거리   <- --distance 가 이것
    pose_to_xyzrpy z      3.171   **광축 방향** Z (=T[2,3])  <- depth 가 재는 것
    docking.distance      3.197   직선 거리 (norm)

셋을 서로 비교하면 안 된다. 표에는 셋 다 찍는다.

── 쓰는 법 ────────────────────────────────────────────────────────────────
    # 한 배치 (기본값이 곧 사용자 배치다)
    python tools/simulate.py

    # 각도 쓸기 — reliable 임계가 어디서 켜지는지
    python tools/simulate.py --sweep heading 0:40:2

    # 모션블러가 검출을 어디서 끊는지 (실측: 10px 100%, 32px 0%)
    python tools/simulate.py --sweep blur 0:40:4

    # 센서 노이즈 (검출기가 SIGSEGV 로 죽는다 — 아래 참고)
    python tools/simulate.py --sweep noise 0:40:5 --repeat 5

    # 렌더링 없이 정확한 모서리로만 (약 100배 빠르다. 블러/노이즈는 못 쓴다)
    python tools/simulate.py --sweep heading 0:60:0.5 --analytic

    # 구성이 docking_state 와 정말 맞물리는지 (14개 배치 왕복검사)
    python tools/simulate.py --self-check

── 알아 둘 함정 (전부 실측) ───────────────────────────────────────────────
1. **렌더러 자체에 오차 바닥이 있다.** AT2 검출기가 돌려주는 모서리는
   cv2.projectPoints 대비 x 로 +0.50px, y 로 +0.41px 치우쳐 있다(28배치 112모서리).
   라이브러리의 픽셀중심 규약 차이다. 1.5~3m 에서 lateral 약 1cm, heading
   0.2~0.8도 값이다. 그래서 이 도구는 **블러/노이즈 0 인 줄을 먼저 찍고**
   그것을 바닥(noise floor)이라고 부른다. 그보다 작은 변화는 보고하면 안 된다.
   (cx-0.5 로 렌더해 상쇄해 봤지만 안 됐다 — 잔차가 균일한 이동이 아니다.
    4배 슈퍼샘플링도 안 줄었고, tag_px≈44 에서는 오히려 자세를 모호한 가지로
    넘겨 heading 이 25도 틀어졌다.)
2. **노이즈를 주면 AT2 검출기가 죽는다.** "too many borders in contour_detect"
   를 뱉고 SIGSEGV 로 프로세스가 통째로 내려간다. 파이썬에서 못 잡는다.
   그래서 --noise 를 쓰면 검출만 **자식 프로세스**에서 돌리고, 죽으면 그 배치를
   "detector crashed" 로 적고 쓸기를 계속한다.
3. **화면 밖은 조용히 일어난다.** cv2.projectPoints 는 카메라 뒤 점도 아무 말 없이
   픽셀 좌표로 돌려준다. 실제로 lateral=+1.50, distance=2.50, heading=+60 배치는
   광축이 태그에서 91도 떨어져 있는데 PnP 가 재투영오차 1.2e-3px 로 "수렴"했다
   (정답에서 120 단위 떨어진 자세로). 그래서 네 모서리가 전부 z>0 이고 화면 안에
   들어오는지 **먼저** 검사하고, 아니면 렌더 자체를 거부한다.
4. **heading 이 ±160~180 으로 나오면 거의 항상 모서리 순서 버그다.** 배치가
   이상한 게 아니다. 그 증상은 corner winding 이 뒤집혔을 때 정확히 나온다.
5. **reliable_angle 이 두 개다.** docking_state 쪽은 approach_deg>=10 (위치),
   pose_quality 쪽은 tag_tilt_deg>=10 (기하). 이 구성에서는
   tag_tilt_deg == |heading| 이 **정확히** 성립하므로(1e-14) 각도 쓸기는 곧
   heading 쓸기다. 둘은 서로 다른 배치에서 켜진다 — 표에 둘 다 찍는다.
6. **tag_size 가 어긋나면 거리만 그 비율로 통째로 틀어지고 그림은 멀쩡하다.**
   일부러 시험하려면 --est-tag-size 로 추정쪽 값만 다르게 준다.

계산/검출/자세는 전부 src/models/tag_pose.py 것을 그대로 쓴다. 여기서 다시
구현한 것은 하나도 없다 — 이 도구가 재는 대상이 바로 그 모듈이기 때문이다.
"""
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
from src.models.tag_pose import (CameraIntrinsics, make_detector,    # noqa: E402
                                 detect, to_gray, estimate_pose,
                                 pose_to_xyzrpy, docking_state, tag_tilt_deg,
                                 pose_quality, tag_pixel_size, _object_points,
                                 MIN_TAG_PX, STABLE_TAG_PX, RELIABLE_TILT_DEG,
                                 DEFAULT_QUAD_BLUR)
from src.utils.util import pr2t, invert_T                            # noqa: E402


# ===========================================================================
# 카메라 값 — 이 D435i 개체의 공장 캘리브레이션 실측치
# ===========================================================================
#
# 왜 하드코딩인가: 카메라가 뽑혀 있어 from_realsense() 로 읽을 수가 없다.
# 아래 숫자는 이 개체에서 실제로 읽어 적어 둔 것이다. 다른 개체를 쓰면 다르다.
#
# **왜곡계수는 0 이다(사용자 검증).** 그래서 distortion=() 로 두고 순수 핀홀로
# 렌더한다 — pose_by_pnp 가 np.zeros(5) 를 넘기므로 우리가 그린 그림이 곧
# 추정기가 역산하는 모델이 된다. 왜곡이 있는 렌즈였다면 이 도구의 숫자는
# 낙관적으로 치우쳤을 것이다.
#
# 1920x1080 과 1280x720 은 같은 센서의 같은 화각(16:9)이고 fx 만 비례한다.
# 640x480 은 4:3 이라 **세로 화각이 다르다** — 태그가 화면 밖으로 나가는
# 지점이 달라지므로 해상도를 바꿔 가며 봐야 한다.
PRESETS = {
    "1920x1080": CameraIntrinsics(1359.2, 1359.0, 956.9, 571.3, 1920, 1080),
    "1280x720":  CameraIntrinsics(906.1,  906.0,  637.9, 380.9, 1280, 720),
    "640x480":   CameraIntrinsics(604.1,  604.0,  318.6, 253.9, 640,  480),
}
#: 기본값. 1920x1080 과 화각이 같으면서 렌더가 2.25배 빠르다.
DEFAULT_RES = "1280x720"

#: 사용자 배치 기본값 (m, m, m, m, deg)
DEFAULT_PLACEMENT = dict(tag_height=1.60, cam_height=1.20,
                         distance=3.00, lateral=-1.03, heading_deg=20.0)
DEFAULT_TAG_SIZE = 0.20

#: tag36h11 한 변의 칸 수 (6x6 데이터 + 검은 테두리 1칸). make_tag_pdf.CELLS 와 같다.
#: 한 칸 = 태그 한 변의 1/8 이다. "블러 32px" 이 왜 치명적인지는 이 환산에서 나온다 —
#: tag_px=320 이면 한 칸이 40px 이고, 32px 블러는 칸 하나를 거의 통째로 뭉갠다.
CELLS = 8


# ===========================================================================
# 1) 기하 — 바닥 기준 배치 -> T_camera_tag
# ===========================================================================
#
# 이 함수가 이 도구의 전부다. 나머지는 이걸 그림으로 만들고 다시 재는 껍데기다.
#
# 좌표계 규약 (실측으로 못박은 것. 다시 유도하지 말 것):
#   태그   +x = 태그를 마주본 사람의 **왼쪽**, +y = **위**, +z = 태그 **뒤쪽**(벽 속)
#   카메라 +x = 오른쪽, +y = **아래**, +z = 광축 앞  (OpenCV 표준)
#   정면·수평이면 R_camera_tag = diag(-1, -1, +1)
#
# 세 줄만 기억하면 부호를 다시 안 틀린다:
#   forward  = -p[2]  이므로 정답은 p[2] = **-distance**        (docking_state 본문)
#   vertical =  p[1]  이고 태그 +y 가 위이므로 **cam_height - tag_height**
#   lateral  =  p[0]  그대로. --lateral 부호가 곧 태그 +x 좌표다.

def placement_to_T(tag_height=1.60, cam_height=1.20, distance=3.00,
                   lateral=-1.03, heading_deg=20.0, check=True):
    """바닥 기준 배치를 T_camera_tag (4x4) 로 바꾼다.

    Args:
        tag_height: 바닥에서 태그 중심까지 [m]
        cam_height: 바닥에서 카메라 렌즈까지 [m]
        distance:   태그 **면**까지의 수직 거리 [m] (광축 거리도, 직선 거리도 아니다)
        lateral:    정면축에서 좌우 [m]. 양수면 태그 +x 쪽 = 마주본 사람 기준 왼쪽
        heading_deg: 광축이 태그 축과 이루는 각 [도]. 양수면 운전자가 제 왼쪽으로 튼 것
        check:      True 면 docking_state(T) 가 입력을 그대로 돌려주는지 확인한다

    Returns:
        T_camera_tag 4x4 — 태그 좌표의 점을 카메라 좌표로 옮기는 행렬.
        estimate_pose() 가 돌려주는 것과 **같은 규약**이라 그대로 비교하면 된다.

    닫힌 형태 (L=lateral, V=cam_height-tag_height, D=distance, c=cos h, s=sin h):
        R_camera_tag = [[-c, 0, s], [0, -1, 0], [s, 0, c]]      (R 이 대칭이다)
        t_camera_tag = [L*c + D*s,  V,  D*c - L*s]
    아래 구현과 비트 단위로 같다(14개 배치에서 최대 차 0.00e+00).
    """
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
    """docking_state(T) 가 우리가 넣은 배치를 그대로 돌려주는지 본다.

    **이게 이 도구의 유일한 안전장치다.** 정답을 잘못 구성하면 오차표가 0 에
    가깝게 나오면서도 통째로 틀린다(대표적으로 lateral/heading 이 함께 미러링되는
    corner shift 2 실패 — 그럴듯하게 생겨서 제일 위험하다).
    """
    st = docking_state(T)
    want = {"lateral": float(lateral),
            "vertical": float(cam_height) - float(tag_height),
            "forward": float(distance),
            "heading_deg": float(heading_deg)}
    bad = {k: (st[k], v) for k, v in want.items() if abs(st[k] - v) > tol}
    # tilt == |heading| 은 이 구성에서 항등식이다(실측 1e-14). 깨지면 R 이 틀린 것이다.
    if abs(tag_tilt_deg(T) - abs(float(heading_deg))) > 1e-6:
        bad["tilt=|heading|"] = (tag_tilt_deg(T), abs(float(heading_deg)))
    if bad:
        raise AssertionError("배치 구성이 docking_state 와 안 맞는다: %s" % bad)
    return st


def truth_of(T):
    """정답 T 에서 표에 올릴 값들을 뽑는다. 추정쪽과 **같은 함수**로 뽑는다.

    같은 함수를 쓰는 게 중요하다 — 정답만 손으로 계산하면 규약 차이가 오차로
    둔갑한다. 여기 값들은 전부 해석적으로도 확인돼 있다(1e-14).
    """
    st = docking_state(T)
    v = pose_to_xyzrpy(T)
    return {"distance": st["distance"], "lateral": st["lateral"],
            "forward": st["forward"], "vertical": st["vertical"],
            "heading": st["heading_deg"], "tilt": tag_tilt_deg(T),
            "z": v["z"], "approach": st["approach_deg"],
            "rel_approach": bool(st["reliable_angle"]),
            "rel_tilt": bool(tag_tilt_deg(T) >= RELIABLE_TILT_DEG)}


# ===========================================================================
# 2) 가시성 — 화면 안에 들어오는가
# ===========================================================================

class OutOfView(Exception):
    """이 배치에서는 태그가 화면에 안 잡힌다. 렌더를 거부한다."""


def project_corners(T, intr, tag_size):
    """태그 네 모서리를 픽셀로 투영한다. detection.corners 와 **같은 순서**다.

    순서는 _object_points() 가 정한다: (-s,-s), (s,-s), (s,s), (-s,s).
    정면 태그에서 화면 오른쪽아래부터 반시계다. 이 순서가 어긋나면 lateral 과
    heading 이 통째로 미러링된다(shift 2) — 눈으로는 안 보이는 실패다.

    Returns:
        (uv [4x2] float64, cam [4x3] 카메라좌표)
        카메라 뒤 점(z<=0)의 uv 는 **의미 없는 값**이다. visibility() 로 거를 것.
    """
    T = np.asarray(T, dtype=np.float64)
    obj = _object_points(float(tag_size))
    cam = (T[:3, :3] @ obj.T).T + T[:3, 3]
    z = np.where(np.abs(cam[:, 2]) < 1e-12, 1e-12, cam[:, 2])
    uv = (intr.K @ (cam / z[:, None]).T).T[:, :2]
    return uv, cam


def visibility(T, intr, tag_size, margin_px=0.0):
    """태그가 정말 찍히는가. **렌더 전에** 반드시 통과시킨다.

    cv2.projectPoints 는 카메라 **뒤**의 점도 아무 경고 없이 픽셀 좌표를 준다.
    그 좌표로 warp 하면 그럴듯한 쓰레기 그림이 나오고, PnP 는 그걸 재투영오차
    1e-3 px 로 "수렴"시킨다(실측: 정답에서 120 단위 떨어진 자세).
    조용히 틀린 숫자보다 "안 보인다"가 훨씬 낫다.

    Args:
        margin_px: 이만큼 안쪽까지 들어와야 통과. 흰 여백(quiet zone)이 잘리면
            검출기가 quad 를 못 닫으므로, 렌더러는 여백 폭을 여기 넣는다.

    Returns:
        dict(ok, reason, uv, cam, tag_px)
    """
    uv, cam = project_corners(T, intr, tag_size)
    w = intr.width or PRESETS[DEFAULT_RES].width
    h = intr.height or PRESETS[DEFAULT_RES].height
    behind = int((cam[:, 2] <= 0).sum())
    lo, hi_w, hi_h = margin_px, w - margin_px, h - margin_px
    outside = int(((uv[:, 0] < lo) | (uv[:, 0] >= hi_w) |
                   (uv[:, 1] < lo) | (uv[:, 1] >= hi_h)).sum())
    edges = np.linalg.norm(uv - np.roll(uv, -1, axis=0), axis=1)
    tag_px = float(edges.mean()) if behind == 0 else float("nan")

    reason = ""
    if behind:
        reason = "모서리 %d개가 카메라 뒤에 있다" % behind
    elif outside:
        reason = ("모서리 %d개가 화면(%dx%d) 밖이다 — 태그 중심이 화면에서 "
                  "(%.0f, %.0f)" % (outside, w, h, uv[:, 0].mean(), uv[:, 1].mean()))
    return {"ok": not reason, "reason": reason, "uv": uv, "cam": cam,
            "tag_px": tag_px}


# ===========================================================================
# 3) 렌더링 — 태그 하나를 핀홀로 그린다
# ===========================================================================

def render(T, intrinsics, tag_size, tag_id=0, width=None, height=None,
           quiet_cells=2.0, texture_px=640, background=170,
           blur_px=0.0, noise_sigma=0.0, seed=0):
    """배치 T 로 본 장면을 합성한다. uint8 BGR 을 돌려준다.

    시각화 도구가 쓸 수 있게 **이미지를 그대로 돌려준다** — 이 파일은 창을 띄우지
    않는다(측정 도구다).

    질감 -> 태그좌표 대응이 함정이다. 검증된 대응은 이것뿐이다:
        질감 TL (0,0)   -> ( +e, +e, 0 )
        질감 TR (W,0)   -> ( -e, +e, 0 )
        질감 BR (W,H)   -> ( -e, -e, 0 )
        질감 BL (0,H)   -> ( +e, -e, 0 )
    (태그 +x 가 마주본 사람의 왼쪽이라 좌우가 뒤집힌다. 순진한 TL->(-e,-e) 로
     쓰면 검출 모서리가 shift 2 로 나오고 lateral/heading 이 함께 미러링된다.)
    e 는 흰 여백까지 포함한 반변 길이다 — 여백도 같은 평면에 있으므로 같이 warp 해야
    원근이 맞는다.

    Args:
        tag_size: 태그 실제 한 변 [m]. 검은 테두리 바깥까지.
        quiet_cells: 태그 둘레 흰 여백을 **칸 수**로. 1칸이 권장 하한이고
            (make_tag_pdf.py 도 size/8 을 권한다) 기본 2칸은 안전쪽이다.
            여백이 없으면 검출기가 태그 경계 quad 를 못 닫아 아예 못 찾는다.
        texture_px: 질감 한 변 [px]. 화면 태그 크기의 2배쯤이면 충분하다.
            **4배 슈퍼샘플링은 모서리 잔차를 줄여주지 않았다**(실측) — 잔차의
            정체가 해상도가 아니라 라이브러리의 픽셀중심 규약이라서다.
        background: 여백 바깥 배경 밝기. 검출에는 영향이 없다(흰 여백이 태그를
            둘러싸고 있으므로). 회색이라 여백이 눈에 보인다.
        blur_px: 수평 모션블러 폭 [px]. 실측 기준선: 10px 까지 검출 28/28(100%),
            32px 에서 0/28. tag36h11 한 칸이 태그 한 변의 1/10(테두리 포함 10칸
            기준)이라 그쯤에서 칸이 뭉개진다. 롤링셔터 D435i 가 z=1m 에서
            1.0m/s 로 접근하며 노출 33ms 면 약 45px 이다 — 즉 실제로 일어난다.
        noise_sigma: 가우시안 잡음 sd [그레이레벨]. 노출이 제대로 잡힌 D435i
            컬러가 대략 2 수준이다.
            **주의: 검출기가 노이즈에 SIGSEGV 로 죽는다.** 그림을 만드는 여기는
            안전하지만, 이 그림을 detect() 에 넣는 쪽은 자식 프로세스여야 한다.
        seed: 잡음 난수 씨앗. 같은 씨앗이면 같은 그림이다.

    Raises:
        OutOfView: 모서리가 카메라 뒤로 가거나 화면 밖으로 나갈 때.
            **쓰레기 프레임을 돌려주지 않는다.** 수평 카메라 + 높이차 배치에서는
            근거리(태그가 위로 벗어난다)에 실제로 일어난다 — 그걸 사용자가 봐야 한다.
    """
    intr = intrinsics
    w = int(width or intr.width)
    h = int(height or intr.height)

    core_px = int(round(texture_px / CELLS)) * CELLS      # 칸 수로 나누어떨어지게
    pad_px = int(round(quiet_cells * core_px / CELLS))
    e = (float(tag_size) / 2.0) * (core_px + 2.0 * pad_px) / core_px

    # 두 단계로 거른다. 이유가 다르기 때문이다.
    #   (1) 태그 자체가 화면 밖/카메라 뒤   -> 아예 못 찍는다
    #   (2) 태그는 들어왔는데 화면 가장자리에 붙어 흰 여백 **한 칸**이 안 남는다
    #       -> 검출기가 태그 경계 quad 를 못 닫아 못 찾는다(그림은 멀쩡해 보인다)
    # 여백 2칸이 잘리는 것 자체는 괜찮다. 한 칸만 남으면 검출은 된다
    # (make_tag_pdf.py 도 권장 여백을 size/8 = 한 칸으로 잡는다).
    vis = visibility(T, intr, tag_size, margin_px=0.0)
    if not vis["ok"]:
        raise OutOfView(vis["reason"])
    need = vis["tag_px"] / CELLS                     # 한 칸이 화면에서 몇 px 인가
    vis_q = visibility(T, intr, tag_size, margin_px=need)
    if not vis_q["ok"]:
        raise OutOfView("태그가 화면 가장자리에 붙어 흰 여백 한 칸(%.0fpx)이 안 남는다 "
                        "— 검출기가 태그 경계를 못 닫는다: %s" % (need, vis_q["reason"]))
    # 여백까지 포함한 사각형은 화면 밖으로 삐져나가도 된다(warp 가 잘라 준다).
    # 다만 **카메라 뒤로 넘어가면** 원근변환 자체가 무의미해진다.
    vis_pad = visibility(T, intr, 2.0 * e, margin_px=0.0)
    if (vis_pad["cam"][:, 2] <= 0).any():
        raise OutOfView("흰 여백 모서리가 카메라 뒤로 넘어간다 — 태그가 렌즈에 너무 가깝다")

    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    core = cv2.aruco.generateImageMarker(dic, int(tag_id), core_px)
    full = cv2.copyMakeBorder(core, pad_px, pad_px, pad_px, pad_px,
                              cv2.BORDER_CONSTANT, value=255)
    fh, fw = full.shape[:2]
    src = np.float32([[0, 0], [fw, 0], [fw, fh], [0, fh]])   # TL, TR, BR, BL

    dst = np.float32(vis_pad["uv"])          # _object_points 순서 = (-e,-e),(e,-e),(e,e),(-e,e)
    # 질감 코너 -> 태그점 대응을 위 docstring 표대로 다시 엮는다.
    #   _object_points 인덱스:  0=(-e,-e)  1=(+e,-e)  2=(+e,+e)  3=(-e,+e)
    #   질감          TL=(+e,+e)=idx2  TR=(-e,+e)=idx3  BR=(-e,-e)=idx0  BL=(+e,-e)=idx1
    dst = np.float32([dst[2], dst[3], dst[0], dst[1]])

    M = cv2.getPerspectiveTransform(src, dst)
    canvas = np.full((h, w, 3), int(background), dtype=np.uint8)
    cv2.warpPerspective(cv2.cvtColor(full, cv2.COLOR_GRAY2BGR), M, (w, h),
                        dst=canvas, borderMode=cv2.BORDER_TRANSPARENT)

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
# 4) 검출 — 그림에서 모서리를 되찾는다 (노이즈면 자식 프로세스에서)
# ===========================================================================
#
# 왜 자식 프로세스인가 (실측):
#   노이즈를 섞은 그림을 AT2 검출기에 넣으면 "too many borders in contour_detect"
#   를 뱉고 **SIGSEGV** 로 인터프리터가 통째로 내려간다. C 라이브러리 안에서
#   나는 신호라 try/except 로 못 잡는다. 40포인트짜리 쓸기가 한 점 때문에
#   통째로 날아가는 걸 막으려면 프로세스를 분리하는 수밖에 없다.
#   비용은 배치당 파이썬 기동 약 0.3초다. --noise 0 이면 이 경로를 아예 안 탄다.

_DETECT_KEYS = ("family", "quad_blur", "min_margin", "max_hamming")


def detect_in_process(gray, cfg):
    """이 프로세스에서 검출한다. 노이즈 없는 그림에서만 쓸 것."""
    det = make_detector(cfg["family"], quad_blur=cfg["quad_blur"])
    res = detect(det, gray, min_margin=cfg["min_margin"],
                 max_hamming=cfg["max_hamming"])
    return [{"tag_id": int(r.tag_id), "hamming": int(r.hamming),
             "margin": float(r.decision_margin),
             "corners": np.asarray(r.corners, dtype=float).tolist()} for r in res]


def detect_in_subprocess(gray, cfg, timeout=60.0):
    """검출을 자식 프로세스에서 돌린다. 죽으면 예외 대신 None 을 돌려준다.

    Returns:
        (검출목록 or None, 메모). None 이면 자식이 죽은 것이다 — 배치를
        "detector crashed" 로 기록하고 쓸기를 계속하면 된다.
    """
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
    """--_worker 로 불려 오는 자식. 검출 결과만 JSON 으로 뱉고 끝난다.

    **자세 계산은 여기서 하지 않는다.** 죽는 것은 detect() 뿐이고, 모서리만
    돌려받으면 부모가 안전하게 나머지를 계산할 수 있다.
    """
    gray = np.load(argv[0])
    sys.stdout.write(json.dumps(detect_in_process(gray, json.loads(argv[1]))))
    return 0


def detection_from_corners(corners, tag_id=0, margin=72.0, hamming=0,
                           family=b"tag36h11"):
    """모서리 네 개로 apriltag.Detection 을 짓는다.

    Detection 은 그냥 namedtuple 이라 이렇게 만들어도 estimate_pose /
    docking_state / pose_quality / tag_pixel_size 를 그대로 통과한다(실측:
    quality ok=True, reasons=[], reproj_rms_px=0.0).

    두 군데서 쓴다:
      * --analytic (렌더 없이 정확한 투영 모서리로만 재는 경로)
      * 자식 프로세스에서 받아온 모서리를 부모에서 되살릴 때
    margin 기본 72 는 실측 정상값(71.9~73.6)이다. MIN_DECISION_MARGIN(20) 아래로
    두면 pose_quality 가 근거 없이 실패로 찍는다.
    """
    c = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    unit = np.float32([[-1, -1], [1, -1], [1, 1], [-1, 1]])   # _object_points 와 같은 순서
    H = cv2.getPerspectiveTransform(unit, c)
    return apriltag.Detection(family, int(tag_id), int(hamming), 0.0,
                              float(margin), H, c.mean(axis=0), c.astype(np.float64))


# ===========================================================================
# 5) 한 배치 재기
# ===========================================================================

@dataclass
class SimResult:
    """배치 하나의 결과 전부. 시각화 도구는 이걸 그대로 받아 쓰면 된다.

    status:
        "ok"            잼
        "out_of_view"   태그가 화면 밖 — 렌더 자체를 거부했다
        "no_detection"  그림은 만들었는데 검출기가 못 찾았다 (블러/노이즈)
        "crashed"       검출기가 자식 프로세스에서 죽었다 (SIGSEGV)
    truth/meas/err 의 키는 같다: distance/lateral/forward/vertical/heading/tilt/z
    err 은 meas - truth 다 (길이 m, 각도 도).
    """
    status: str
    placement: dict
    tag_size: float
    est_tag_size: float
    blur_px: float = 0.0
    noise_sigma: float = 0.0
    truth: dict = field(default_factory=dict)
    meas: dict = field(default_factory=dict)
    err: dict = field(default_factory=dict)
    tag_px: float = float("nan")
    tag_px_truth: float = float("nan")
    reproj_rms_px: float = float("nan")
    margin: float = float("nan")
    corner_res_px: float = float("nan")     # 검출 모서리 - 투영 모서리 (렌더러 오차 바닥)
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
                  "status": self.status})
        for k in KEYS:
            r["truth_" + k] = self.truth.get(k, float("nan"))
            r["meas_" + k] = self.meas.get(k, float("nan"))
            r["err_" + k] = self.err.get(k, float("nan"))
        r.update({"tag_px": self.tag_px, "tag_px_truth": self.tag_px_truth,
                  "reproj_rms_px": self.reproj_rms_px, "margin": self.margin,
                  "corner_res_px": self.corner_res_px,
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
                 analytic=False, cfg=None, keep_image=False):
    """배치 하나를 그려서 다시 재고 SimResult 를 돌려준다.

    Args:
        placement: dict(tag_height, cam_height, distance, lateral, heading_deg)
        est_tag_size: 추정할 때 쓸 태그 크기 [m]. None 이면 tag_size 와 같다.
            **일부러 다르게 주면 순수한 배율 오차가 된다** — lateral/vertical/
            forward/distance 가 그 비율로 곱해지고 heading/tilt 는 안 변한다.
            그림으로는 절대 안 보이는 실패라 시험해 볼 값어치가 있다.
        analytic: True 면 렌더링 없이 cv2.projectPoints 모서리를 바로 쓴다.
            약 100배 빠르고 렌더러 오차 바닥(0.5px)이 없다. 대신 블러/노이즈를
            못 넣는다 — 그건 그림이 있어야 하는 이야기라서다.
        cfg: 검출기 설정 dict(family, quad_blur, min_margin, max_hamming)
    """
    cfg = cfg or dict(family="tag36h11", quad_blur=DEFAULT_QUAD_BLUR,
                      min_margin=0.0, max_hamming=0)
    est = float(est_tag_size or tag_size)
    T = placement_to_T(**placement)
    tr = truth_of(T)
    base = dict(placement=dict(placement), tag_size=float(tag_size),
                est_tag_size=est, blur_px=float(blur_px),
                noise_sigma=float(noise_sigma), truth=tr, T_truth=T,
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
                         noise_sigma=noise_sigma, seed=seed)
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
                  quality_ok=bool(q["ok"]), reasons=tuple(q["reasons"]),
                  T_meas=T_meas, **base)
    # reliable 은 **추정값 기준**으로 덮어쓴다 — 실제 제어가 보는 것은 이쪽이다.
    r.rel_approach = bool(st["reliable_angle"])
    r.rel_tilt = bool(q["reliable_angle"])
    r.image = base_img
    return r


# ===========================================================================
# 6) 출력 — 단일 배치 표 / 쓸기 표 / CSV
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
    print("tag_px          %.1f   (MIN %.0f / STABLE %.0f)   재투영 RMS %.3f px   margin %.1f"
          % (r.tag_px, MIN_TAG_PX, STABLE_TAG_PX, r.reproj_rms_px, r.margin))
    print("모서리 잔차     %.3f px  (검출 - 투영. 렌더러의 오차 바닥이다 — 실측 평균 "
          "+0.50/+0.41 px)" % r.corner_res_px)
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


def parse_spec(spec):
    """'start:stop:step' -> 값 배열. stop 포함이다."""
    try:
        a, b, s = (float(x) for x in spec.split(":"))
    except ValueError:
        raise SystemExit("--sweep 두 번째 인자는 start:stop:step 이다 (예: 0:40:2)")
    if s == 0:
        raise SystemExit("step 이 0 이다")
    n = int(np.floor((b - a) / s + 1e-9)) + 1
    return [a + i * s for i in range(max(n, 1))]


def run_sweep(axis, values, args, intr, cfg, repeat=1):
    """축 하나를 쓸며 배치마다 SimResult 를 모은다.

    repeat: 잡음처럼 매번 그림이 달라지는 축에서만 의미가 있다. 검출률을
        분수로 내려면 여러 번 뽑아야 한다(1번이면 0 아니면 1 뿐이다).
        블러는 결정적이라 1 로 충분하다.
    """
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
        n = int(repeat) if (noise > 0) else 1
        for k in range(n):
            out.append(simulate_one(
                place, intr, tag_size,
                est_tag_size=(args.est_tag_size if args.est_tag_size else
                              (tag_size if kind == "tag" else args.tag_size)),
                tag_id=args.tag_id, blur_px=blur, noise_sigma=noise,
                seed=args.seed + k, method=args.method, analytic=args.analytic,
                cfg=cfg))
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
# 7) 자체검사 — 구성이 docking_state 와 정말 맞물리는가
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
    """정답 구성 -> docking_state 왕복이 정확한지 전부 확인한다.

    오차표가 0 에 가깝다고 해서 구성이 맞는 게 아니다 — 정답과 추정이 **같은
    방식으로** 틀리면 오차는 0 이 된다. 그래서 정답 쪽을 따로 못박는다.
    """
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
# main
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
                         "32px 에서 0%%(0/28). tag36h11 한 칸이 태그 한 변의 1/10 이라 "
                         "그 부근에서 칸이 뭉개진다")
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
    ap.add_argument("--quad-blur", type=float, default=DEFAULT_QUAD_BLUR)
    ap.add_argument("--min-margin", type=float, default=0.0)
    ap.add_argument("--max-hamming", type=int, default=0)
    args = ap.parse_args()

    intr = PRESETS[args.res]
    cfg = dict(family=args.family, quad_blur=args.quad_blur,
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
                     keep_image=bool(args.save))
    print_single(r, intr, args.res)
    if args.save and r.image is not None:
        p = Path(args.save)
        p.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(p), r.image)
        print("\nimage: %s (%dx%d)" % (p, r.image.shape[1], r.image.shape[0]))
    return 0 if r.status == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
