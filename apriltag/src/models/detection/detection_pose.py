"""3. 자세 구하기 — 검출된 태그에서 도킹 값까지.

lateral / forward / heading 이 제어에 쓰는 값.
"""
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from config.system import (CAM_YAW_OFFSET_DEG, DEPTH_CHECK_MAX_Z,
                       DEPTH_TOL_COEF, DEPTH_TOL_FLOOR_M,
                      MAX_REPROJ_RMS_PX, MIN_DECISION_MARGIN, MIN_TAG_PX,
                      RELIABLE_TILT_DEG)
from ...utils.util import r2rpy, invert_T, t2pr
from .image import (ASSUMED_HFOV_DEG, depth_at, from_video, intrinsics_from_hfov,
                    intrinsics_from_ref,
                    open_bag, open_realsense, to_gray)
from .detection_tag import (DEFAULT_QUAD_BLUR, STABLE_TAG_PX, detect, make_detector,
                     tag_pixel_size)


# 태그 네 모서리의 3D 좌표. detection.corners 와 같은 순서.
def _object_points(tag_size):
    s = tag_size / 2.0
    return np.array([[-s, -s, 0.], [s, -s, 0.], [s, s, 0.], [-s, s, 0.]], dtype=np.float64)


def pose_by_pnp(detection, intrinsics, tag_size):
    """OpenCV solvePnP 로 자세를 구함. detection_pose 의 대안."""
    obj = _object_points(tag_size)
    img = np.asarray(detection.corners, dtype=np.float64).reshape(-1, 1, 2)
    dist = np.array(intrinsics.distortion, dtype=np.float64) if intrinsics.distortion else np.zeros(5)

    ok, rvec, tvec = cv2.solvePnP(obj, img, intrinsics.K, dist,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return np.full((4, 4), np.nan), float("inf")

    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.ravel()

    proj, _ = cv2.projectPoints(obj, rvec, tvec, intrinsics.K, dist)
    err = float(np.sqrt(((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2).sum(axis=1)).mean())
    return T, err


def estimate_pose(detector, detection, intrinsics, tag_size, method="auto"):
    """태그 하나의 자세를 구함. (T, e0, e1) 3-튜플.

    detector.detection_pose() 와 같은 반환 형식. 실측 예:

        T  = [[-0.999  0.009  0.032  0.151]   좌상 3x3 = 회전, 우측 3x1 = 위치 [m]
              [ 0.005 -0.920  0.391  0.322]   카메라 기준 태그
              [ 0.033  0.391  0.920  1.387]
              [ 0.     0.     0.     1.   ]]
        e0 = e1 = 1.38e-06                    재투영 잔차

    AT2 는 다듬기 전/후 오차 둘을 줬지만 AT3 는 하나뿐이라 같은 값을 두 번 넣음.
    """
    if method == "pnp":
        T, err = pose_by_pnp(detection, intrinsics, tag_size)
        return T, err, err

    R = getattr(detection, "pose_R", None)
    tvec = getattr(detection, "pose_t", None)
    if R is None or tvec is None:
        if method == "tag":
            raise ValueError("AT3 자세가 없다 — detect(intrinsics=..., tag_size=...) 로 부를 것")
        T, err = pose_by_pnp(detection, intrinsics, tag_size)
        return T, err, err

    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = np.asarray(tvec, dtype=float).ravel()
    # AT3 의 pose_err 는 재투영 잔차. AT2 의 e0/e1(다듬기 전/후)과 달리 하나뿐이라
    err = float(getattr(detection, "pose_err", 0.0) or 0.0)

    if method == "auto" and not np.isfinite(T).all():
        T, e = pose_by_pnp(detection, intrinsics, tag_size)
        return T, e, e
    return T, err, err


def pose_to_xyzrpy(T, unit='deg'):
    """4x4 -> 읽을 수 있는 형태. 사람이 보기 위한 것이지 제어용이 아님.

        {'x': 0.151, 'y': 0.322, 'z': 1.387,        카메라 기준 태그 위치 [m]
         'roll': 23.02, 'pitch': -1.87, 'yaw': 179.73,   [도]
         'distance': 1.432}                          원점 거리 [m]

    roll/pitch/yaw 는 분해 순서에 따라 값이 달라지고 짐벌락에서 튐.
    제어에는 docking_state() 를 쓸 것. yaw 는 지게차 방향이 아니라
    카메라 장착 기울기를 잼(카메라 좌표계는 y 가 아래라 축 이름이 다름).
    """
    p, R = t2pr(T)
    rpy = r2rpy(R, unit=unit)
    return {"x": p[0], "y": p[1], "z": p[2],
            "roll": rpy[0], "pitch": rpy[1], "yaw": rpy[2],
            "distance": float(np.linalg.norm(p))}


def pose_to_forklift(T_camera_tag, unit='deg'):
    """4x4 -> **지게차(항공기) 기준**. 사람이 읽기 쉬운 쪽. 제어는 docking_state 를 쓸 것.

        {'lateral': 1.000, 'vertical': -0.400, 'forward': 3.000,   위치 [m]
         'roll': 0.0, 'pitch': 0.0, 'yaw': 20.0,                   자세 [도]
         'distance': 3.187}

    위치 세 개는 docking_state 와 **이름도 값도 같다** — 같은 태그 기준이라서다.
    새로 얹는 것은 roll/pitch/yaw 뿐이다(그중 yaw 는 heading 과 같은 값).

    pose_to_xyzrpy 와 뭐가 다른가:
        저쪽은 **카메라 원점 + 카메라 축**이라 좌우 회전이 pitch 자리에 오고
        yaw 는 180도 근처에 붙박인다. 축 이름이 항공기와 어긋나 읽기 나쁘다.
        여기는 **태그 원점 + 항공기 축**이라 이름이 직관대로다.

    yaw 는 docking_state 의 heading_deg 와 같은 값이다(둘 다 광축 방향에서 뽑는다).
    roll/pitch 는 평평한 바닥에 카메라를 똑바로 달았으면 0 이다.
    **0 이 아니면 장착이 기울었거나 노면이 기운 것** — 그 점검에 쓴다.

    오일러 분해(r2rpy)를 안 쓴다. 축 방향을 하나씩 읽으므로 분해 순서에
    좌우되지 않고, 카메라가 기울어도 yaw 가 오염되지 않는다.
    """
    T_tag_cam = invert_T(np.asarray(T_camera_tag))
    p = T_tag_cam[:3, 3]
    R = T_tag_cam[:3, :3]

    fwd = R @ np.array([0.0, 0.0, 1.0])          # 카메라 광축이 향하는 곳
    right = R @ np.array([1.0, 0.0, 0.0])        # 카메라의 오른쪽

    yaw = np.arctan2(fwd[0], fwd[2])                       # 좌우 (= heading)
    pitch = np.arcsin(np.clip(-fwd[1], -1.0, 1.0))         # 위아래 끄덕
    roll = np.arctan2(-right[1], np.hypot(right[0], right[2]))   # 갸우뚱
    if unit == 'deg':
        roll, pitch, yaw = (np.degrees(v) for v in (roll, pitch, yaw))
        yaw -= CAM_YAW_OFFSET_DEG          # heading 과 같은 보정
    else:
        yaw -= np.radians(CAM_YAW_OFFSET_DEG)

    return {"lateral": float(p[0]), "vertical": float(p[1]), "forward": float(-p[2]),
            "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw),
            "distance": float(np.linalg.norm(p))}


def docking_state(T_camera_tag, intrinsics=None, tag_size=None):
    """카메라 기준 태그 자세를 **도킹 제어가 쓸 형태**로 바꿈. 실측 예:

        {'lateral':  0.104,      태그 축에서 좌우로 벗어난 거리 [m]. + 가 오른쪽
         'vertical': -0.248,     카메라높이 - 태그높이 [m]
         'forward':  1.407,      태그면까지 남은 거리 [m]
         'distance': 1.432,      직선 거리 [m]
         'approach_deg':  4.22,  태그에서 봤을 때 축에서 몇 도 비켜 있나 (위치)
         'heading_deg':   2.04,  지게차가 태그 축과 몇 도 틀어져 있나 (자세)
         'tilt_deg':     23.10,  태그가 화면에서 찌그러진 정도
         'reliable_angle': True} tilt >= 10도 라 각도를 믿어도 됨

    제어에 쓰는 것은 lateral / forward / heading_deg 셋.
    x,y,z 는 지게차가 고개만 돌려도 부호까지 바뀌지만 이 셋은 안 변함.
    approach 와 heading 은 독립임 — 축 위에 서서 고개를 돌리면
    approach=0 인데 heading!=0 임. 진입하려면 둘 다 0 이어야 함.

    heading 은 config.CAM_YAW_OFFSET_DEG 를 뺀 값이다. 카메라가 지게차 정면과
    어긋나게 달리면 heading 이 통째로 그만큼 밀리는데, 자리·거리·각도와 무관하게
    늘 같은 양이라 상수로 뺄 수 있다(실측 확인).
    **재는 법** 둘 중 하나:
        (1) 줄자로 지게차를 태그 정면축 위에 세우고 heading 을 읽는다. 그 값이 오차.
        (2) 태그를 보며 N m 직진한다. 똑바로 갔다면 lateral 이 안 변해야 한다.
            변했으면 atan(변화량 / N) 이 오차.
    tilt_deg 는 보정하지 않는다 — 화면에서 실제로 찌그러진 정도라
    각도 신뢰도(reliable_angle) 판단에는 있는 그대로가 맞다.
    """
    T_tag_cam = invert_T(np.asarray(T_camera_tag))
    p = T_tag_cam[:3, 3]
    R = T_tag_cam[:3, :3]

    lateral = float(p[0])
    vertical = float(p[1])
    forward = float(-p[2])                      # 태그 앞쪽이 양수가 되게 뒤집음
    distance = float(np.linalg.norm(p))

    # 내가 태그 정면축에서 몇 도 벗어난 위치에 있나
    approach = float(np.degrees(np.arctan2(abs(lateral), abs(forward))))

    # 카메라 광축(+z)이 태그 좌표계에서 어디를 향하나.
    fwd = R @ np.array([0.0, 0.0, 1.0])
    # 카메라가 지게차 정면과 어긋나게 달렸으면 그만큼 통째로 밀려 읽힌다.
    # 어느 자리에서든 같은 양이라 상수 하나로 뺀다(실측 확인).
    heading = float(np.degrees(np.arctan2(fwd[0], fwd[2])) - CAM_YAW_OFFSET_DEG)

    # 각도를 믿어도 되는지는 **태그가 화면에서 얼마나 찌그러져 보이나(tilt)** 로 정함.
    tilt = tag_tilt_deg(T_camera_tag)
    # intrinsics 를 주면 프록시(tilt) 대신 heading 흔들림을 직접 예측해서 판정한다.
    # tilt 은 태그가 화면 중심을 벗어나 생기는 원근 정보를 못 보기 때문이다.
    sigma = float("nan")
    reliable = tilt >= RELIABLE_TILT_DEG
    if intrinsics is not None and tag_size:
        from config.system import MAX_HEADING_SIGMA_DEG
        sigma = heading_sigma_deg(T_camera_tag, intrinsics, tag_size)
        reliable = sigma <= MAX_HEADING_SIGMA_DEG
    return {"lateral": lateral, "vertical": vertical, "forward": forward,
            "distance": distance,
            "approach_deg": approach, "heading_deg": heading,
            "tilt_deg": tilt, "heading_sigma_deg": sigma,
            "reliable_angle": bool(reliable)}


def heading_sigma_deg(T_camera_tag, intrinsics, tag_size,
                      corner_px=None, n=None, seed=0):
    """이 배치에서 heading 이 코너 잡음에 얼마나 흔들리나 [도]. float 하나.

    tilt 로는 못 잡는 게 있다 — 태그가 화면 중심을 벗어나기만 해도 원근 때문에
    사다리꼴이 생겨 각도가 정확해지는데, tilt(태그 법선 vs 광축)는 그걸 0 으로 읽는다.
    우리 배치(태그가 0.4m 위, 카메라 수평)가 정확히 그 경우라 tilt 이 항상 0 이다.
    그래서 프록시를 쓰지 말고 직접 흘려본다: 이상적인 코너를 만들어
    corner_px 만큼 흔들고 다시 풀기를 n 번, heading 의 표준편차를 낸다.

    거리·태그크기·기울기·화면상 위치가 전부 자동으로 반영된다.
    corner_px 는 실측으로 보정할 값이다(기본 0.2px).
    """
    from config.system import CORNER_NOISE_PX, SIGMA_SAMPLES
    corner_px = CORNER_NOISE_PX if corner_px is None else float(corner_px)
    n = int(SIGMA_SAMPLES if n is None else n)

    T = np.asarray(T_camera_tag, dtype=np.float64)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        return float("inf")

    obj = _object_points(tag_size)
    K = np.asarray(intrinsics.K, dtype=np.float64)
    dist = (np.asarray(intrinsics.distortion, dtype=np.float64)
            if getattr(intrinsics, "distortion", None) else np.zeros(5))
    R, t = T[:3, :3], T[:3, 3]

    cam = R @ obj.T + t[:, None]                  # 이상적인 코너를 만든다
    if (cam[2] <= 1e-6).any():
        return float("inf")
    px = (K @ cam)
    px = (px[:2] / px[2]).T

    h0 = docking_state(T)["heading_deg"]
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        ok, rv, tv = cv2.solvePnP(obj, (px + rng.normal(0.0, corner_px, px.shape)
                                        ).reshape(-1, 1, 2), K, dist,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        M = np.eye(4)
        M[:3, :3] = cv2.Rodrigues(rv)[0]
        M[:3, 3] = tv.ravel()
        out.append(docking_state(M)["heading_deg"] - h0)
    if len(out) < 3:
        return float("inf")
    return float(np.std(out))


def tag_tilt_deg(T_camera_tag):
    """태그면이 카메라를 정면으로 마주보는 정도 [도]. float 하나.

    0 이면 정면(각도를 못 믿음), 클수록 비스듬(각도가 정확). 실측 예 23.10.
    3m/25cm 태그에서 tilt 2도면 좌우 변 길이차가 0.3px 라 노이즈에 묻힘.
    그래서 RELIABLE_TILT_DEG = 10.
    """
    n = np.asarray(T_camera_tag)[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(min(1.0, abs(n[2])))))


# ===========================================================================


# ---------------------------------------------------------------------------

# 재투영 RMS 상한 [px].

# decision_margin 하한.

# 각도를 믿을 수 있는 최소 기울기 [도]. tag_tilt_deg 의 docstring 과 같은 값이고

# depth 교차검증 허용 오차 [m].


def _sample_depth_m(detection, depth, patch, depth_scale):
    """태그 중심 주변 patch x patch 의 depth 중앙값 [m]. 못 재면 None."""
    if depth is None:
        return None
    u, v = np.asarray(detection.corners, dtype=np.float64).mean(axis=0)
    return depth_at(depth, u, v, patch=patch, scale=depth_scale)


def depth_cross_check(detection, depth, T_camera_tag, patch=5, depth_scale=None):
    """태그 자세의 z 와 depth 센서의 z 를 대조함. 원리가 다른 두 측정이라
    어긋나면 tag_size 나 내부파라미터가 틀렸다는 신호.

        {'z_pose':   1.388,   자세로 잰 거리 [m]
         'z_depth':  1.332,   depth 센서로 잰 거리 [m]
         'diff_m':  -0.056,   차이 [m]
         'diff_pct': -4.06,   차이 [%]
         'tol_m':    0.096,   허용치. max(0.02, 0.05*z^2) 로 거리에 따라 커짐
         'in_range': True,    z 가 판정 가능 거리(1.5m) 안인가
         'agree':    True}    |diff_m| <= tol_m

    z 가 DEPTH_CHECK_MAX_Z(1.5m) 를 넘으면 허용치가 너무 헐거워져 판정을 건너뜀.
    """
    z_pose = float(np.asarray(T_camera_tag)[2, 3])
    tol = max(DEPTH_TOL_FLOOR_M, DEPTH_TOL_COEF * z_pose * z_pose)
    out = {"z_pose": z_pose, "z_depth": None, "diff_m": None, "diff_pct": None,
           "tol_m": float(tol), "in_range": bool(0.0 < z_pose <= DEPTH_CHECK_MAX_Z),
           "agree": False}

    z_depth = _sample_depth_m(detection, depth, patch, depth_scale)
    if z_depth is None or not np.isfinite(z_pose) or z_pose <= 0:
        return out

    diff = z_depth - z_pose
    out["z_depth"] = float(z_depth)
    out["diff_m"] = float(diff)
    out["diff_pct"] = float(100.0 * diff / z_pose)
    out["agree"] = bool(abs(diff) <= tol)
    return out


def pose_quality(detector, detection, intrinsics, tag_size, T_camera_tag, method="auto"):
    """이 프레임의 자세를 믿어도 되는지 한 번에 판정함. 실측 예:

        {'reproj_rms_px': 1.4e-06,   구한 자세로 모서리를 되찍어 본 오차 [px]
         'reproj_err':    1.4e-06,   같은 값 (단위는 reproj_units)
         'reproj_units': 'px',
         'tag_px':      131.71,      태그 한 변의 화면상 길이 [px]
         'tilt_deg':     23.10,      찌그러진 정도
         'reliable_angle': True,     각도를 믿어도 되나
         'decision_margin': 43.44,   검출기 판정 여유
         'hamming': 0,               고쳐낸 비트 수
         'ok': True,                 아래 reasons 가 비었나
         'reasons': []}              걸린 항목들. ok=False 면 여기에 이유가 옴

    depth 를 넘기면 'depth' 키가 더 붙음(depth_cross_check 결과).
    거르는 기준: tag_px >= 20, reproj <= 2.0px, margin >= 20, hamming == 0.
    """
    T = np.asarray(T_camera_tag, dtype=np.float64)

    if method == "pnp":
        _, err = pose_by_pnp(detection, intrinsics, tag_size)
        e1, squared = err, False
    else:
        pe = getattr(detection, "pose_err", None)
        if pe is None:
            _, err = pose_by_pnp(detection, intrinsics, tag_size)
            e1, squared = err, False
        else:
            e1, squared = float(pe), False

    e1 = float(e1)
    rms = float(np.sqrt(e1)) if (squared and e1 >= 0) else e1

    tag_px = tag_pixel_size(detection)
    tilt = tag_tilt_deg(T)
    margin = float(getattr(detection, "decision_margin", 0.0))
    hamming = int(getattr(detection, "hamming", 0))

    reasons = []
    if not np.isfinite(T).all():
        reasons.append("pose_nan")
    if tag_px < STABLE_TAG_PX:
        reasons.append(f"tag_px<{STABLE_TAG_PX:g}")
    if not np.isfinite(rms) or rms > MAX_REPROJ_RMS_PX:
        reasons.append(f"reproj>{MAX_REPROJ_RMS_PX:g}px")
    if margin < MIN_DECISION_MARGIN:
        reasons.append(f"margin<{MIN_DECISION_MARGIN:g}")
    if hamming != 0:
        reasons.append("hamming!=0")

    return {"reproj_err": e1,
            "reproj_units": "px^2" if squared else "px",
            "reproj_rms_px": rms,
            "tag_px": tag_px,
            "tilt_deg": tilt,
            "reliable_angle": bool(tilt >= RELIABLE_TILT_DEG),
            "decision_margin": margin,
            "hamming": hamming,
            "ok": not reasons,
            "reasons": reasons}

@dataclass
class Result:
    """한 프레임에서 나온 것 전부. TagPipeline 이 프레임마다 하나씩 만듦."""
    index: int
    timestamp: float
    image: object
    detections: list = field(default_factory=list)
    poses: dict = field(default_factory=dict)
    docking: dict = field(default_factory=dict)
    quality: dict = field(default_factory=dict)
    intrinsics: object = None
    errors: dict = field(default_factory=dict)

    @property
    def tag_ids(self):
        return [int(d.tag_id) for d in self.detections]

    def __len__(self):
        return len(self.detections)

    def primary(self, tag_id=None):
        """이 프레임에서 **믿고 쓸 태그 하나**를 골라 한 묶음으로 돌려줌."""
        usable = [d for d in self.detections if int(d.tag_id) in self.poses]
        if not usable:
            return None
        if tag_id is not None:
            usable = [d for d in usable if int(d.tag_id) == int(tag_id)]
            if not usable:
                return None
            det = usable[0]
        else:
            det = max(usable, key=tag_pixel_size)
        tid = int(det.tag_id)
        return {"tag_id": tid, "detection": det, "T": self.poses[tid],
                "docking": self.docking.get(tid), "quality": self.quality.get(tid)}

_PIPE_KEYS = ("detector", "families", "quad_blur", "method", "min_margin",
              "max_hamming", "gray_channel",
              "hfov", "quality", "depth_check", "label", "origin")

class TagPipeline:
    """소스 한 개 + 검출기 한 개를 들고, 프레임마다 Result 를 뱉음."""

    def __init__(self, frames=None, intrinsics=None, tag_size=None, detector=None,
                 families="tag36h11", quad_blur=DEFAULT_QUAD_BLUR, method="auto",
                 min_margin=0.0, max_hamming=0, gray_channel=None,
                 hfov=None, quality=True, depth_check=True,
                 label="", origin="", close=None):
        """Args:
        tag_size: **필수. 기본값을 두지 않았음.**
            이 값이 틀리면 거리 전체가 그 비율만큼 조용히 틀어짐(화면은
        """
        if tag_size is None:
            raise ValueError(
                "tag_size 를 반드시 줘야 한다 [m]. 거리가 이 값에 정비례하므로 "
                "기본값을 두면 틀린 거리가 조용히 나온다. 검은 테두리 바깥까지 잰 한 변.")
        self.frames = frames
        self.intr = intrinsics
        self.tag_size = float(tag_size)
        self.detector = detector if detector is not None else make_detector(
            families=families, quad_blur=quad_blur)
        self.method = method
        self.min_margin = float(min_margin)
        self.max_hamming = int(max_hamming)
        self.gray_channel = gray_channel
        self.hfov = None if hfov is None else float(hfov)
        self.quality = bool(quality)
        self.depth_check = bool(depth_check)
        self.label = label
        self.origin = origin or ("given" if intrinsics is not None else "")
        self.intrinsics_assumed = False
        self._close = close
        self._n = 0

    @staticmethod
    def _split_kw(kw):
        """kw 를 (파이프라인용, 소스용) 으로 가름. 이름이 겹치는 인자는 없음."""
        pipe = {k: kw.pop(k) for k in list(kw) if k in _PIPE_KEYS}
        return pipe, kw

    @classmethod
    def from_realsense(cls, tag_size, stream="color", **kw):
        """실물 D435i 를 열어 파이프라인을 만듦. 남는 인자는 open_realsense 로 감."""
        pipe_kw, open_kw = cls._split_kw(kw)
        frames, intr = open_realsense(stream=stream, **open_kw)
        pipe_kw.setdefault("label", "realsense/%s" % stream)
        pipe_kw.setdefault("origin", "factory (RealSense %s)" % stream)
        return cls(frames, intrinsics=intr, tag_size=tag_size,
                   close=frames.close, **pipe_kw)

    @classmethod
    def from_bag(cls, path, tag_size, **kw):
        """녹화한 .bag 을 재생함. 남는 인자는 open_bag 으로 감."""
        pipe_kw, open_kw = cls._split_kw(kw)
        frames, intr = open_bag(str(path), **open_kw)
        pipe_kw.setdefault("label", "bag (%s)" % Path(path).name)
        pipe_kw.setdefault("origin", "bag stream profile")
        return cls(frames, intrinsics=intr, tag_size=tag_size,
                   close=frames.close, **pipe_kw)

    @classmethod
    def from_video(cls, path, tag_size, intrinsics=None, hfov=None,
                   loop=False, **kw):
        """영상 파일. 카메라 값이 없으므로 intrinsics 를 주거나 화각을 가정함."""
        pipe_kw, _rest = cls._split_kw(kw)
        if _rest:
            raise TypeError("모르는 인자: %s" % ", ".join(sorted(_rest)))
        frames, _none = from_video(path, loop=loop)
        pipe_kw.setdefault("label", "video (%s)" % Path(path).name)
        pipe_kw.setdefault("origin", "given" if intrinsics is not None else "")
        return cls(frames, intrinsics=intrinsics, tag_size=tag_size, hfov=hfov,
                   close=frames.close, **pipe_kw)

    def intrinsics_for(self, shape):
        """카메라 값을 확정함. 없으면 화면 크기 + 화각 가정으로 지어냄."""
        if self.intr is None:
            self.intr = intrinsics_from_hfov(shape, self.hfov)
            self.intrinsics_assumed = True
            self.origin = ("ASSUMED from D435i ref" if self.hfov is None
                           else "ASSUMED from hfov=%.0fdeg" % self.hfov)
        return self.intr

    def process(self, img, index=None, timestamp=None):
        """이미지 한 장 -> Result. 어떤 실패도 밖으로 새지 않음."""
        src = img                 
        gray = to_gray(img, channel=self.gray_channel)
        intr = self.intrinsics_for(np.asarray(img).shape)

        if index is None:
            index = self._n
        self._n = index + 1

        res = Result(index=int(index),
                     timestamp=float(timestamp) if timestamp is not None else 0.0,
                     image=img, intrinsics=intr)

        try:
            res.detections = detect(self.detector, gray,
                                    intrinsics=intr, tag_size=self.tag_size,
                                    min_margin=self.min_margin,
                                    max_hamming=self.max_hamming)
        except Exception as exc:

            res.errors["detect"] = "%s: %s" % (type(exc).__name__, exc)
            return res

        depth = getattr(src, "depth", None)
        depth_scale = getattr(src, "depth_scale", 0.0) or None

        for d in res.detections:
            tid = int(d.tag_id)
            try:
                T, _e0, _e1 = estimate_pose(self.detector, d, intr, self.tag_size,
                                            method=self.method)
            except Exception as exc:
                res.errors[tid] = "estimate_pose %s: %s" % (type(exc).__name__, exc)
                continue
            T = np.asarray(T, dtype=np.float64)
            if T.shape != (4, 4) or not np.isfinite(T).all():
                res.errors[tid] = "non-finite pose (NaN)"
                continue

            res.poses[tid] = T
            res.docking[tid] = docking_state(T, intr, self.tag_size)
            if self.quality:
                q = pose_quality(self.detector, d, intr, self.tag_size, T,
                                 method=self.method)
                if self.depth_check and depth is not None:
                    q["depth"] = depth_cross_check(d, src, T, depth_scale=depth_scale)
                res.quality[tid] = q
        return res

    def __iter__(self):
        """소스를 끝까지 돌며 Result 를 뱉음. 3-튜플 계약을 그대로 소비함."""
        if self.frames is None:
            raise RuntimeError("소스 없이 만든 파이프라인이다 — process(img) 로 한 장씩 넣어라.")
        for i, ts, img in self.frames:
            yield self.process(img, index=i, timestamp=ts)

    @property
    def stats(self):
        """소스의 프레임 드롭 회계(FrameStats). 없는 소스면 None."""
        return getattr(self.frames, "stats", None)

    @property
    def ae_roi(self):
        """자동노출 ROI 객체(open_realsense(ae_roi=True) 일 때만). 없으면 None."""
        return getattr(self.frames, "ae_roi", None)

    def close(self):
        """소스를 닫고 검출기를 풂. 몇 번 불러도 안전함.

        검출기 참조만 놓음. 실제 해제는 make_detector 가 막아 둠 —
        pupil_apriltags 의 apriltag_detector_destroy 가 세그폴트를 냄.
        """
        c, self._close = self._close, None
        if c is not None:
            try:
                c()
            except Exception:
                pass
        self.detector = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


# ===========================================================================


def measure(results, tag_id=None, n=None, max_frames=None, require_ok=True):
    """정지 상태에서 여러 프레임을 모아 **대표값 하나**를 냄.

    한 프레임 값은 못 씀 — 태그·카메라를 고정해 둬도 실측으로 이만큼 흔들림:
        lateral +-9.0mm / heading +-0.45도 (1.4m, 1920x1080)
    코너 검출이 0.35px 떨리고, 그게 heading 을 흔들고, 다시 거리에 곱해져
    lateral 을 흔듦. n 프레임을 모으면 sqrt(n) 로 줌(30이면 1/5.5).

    중앙값을 씀. 평균과 달리 가끔 튀는 프레임에 안 끌려감.
    spread 는 **대표값의 표준오차**(표준편차/sqrt(n))다 — "이 값을 얼마나
    믿을 수 있나"이지 장면이 얼마나 흔들리나가 아님.

    반환:
        {'lateral': 0.104, 'forward': 1.407, 'heading_deg': 2.04,   대표값
         'tilt_deg': 23.1, 'z_optical': 1.388, 'distance': 1.432,
         'n': 30,                                                    쓴 프레임 수
         'spread': {'lateral': 0.0014, 'forward': 0.0003, ...},      표준오차
         'reliable_angle': True,        tilt >= RELIABLE_TILT_DEG
         'stable': True,                spread 가 임계 안
         'reasons': []}                 stable=False 면 왜인지
        태그를 못 봤으면 None.

    results 는 Result 를 흘리는 무엇이든 된다 — TagPipeline 이면 프레임을 직접
    읽고, **이미 모아둔 리스트**면 그것만 계산한다. 화면을 그리느라 프레임 루프를
    내줄 수 없는 쪽(dock_live)은 스스로 모은 리스트를 넘긴다.

    stable 이 False 면 **명령을 내지 말고 다시 재라.** 누가 지나갔거나
    조명이 깜빡였거나 아직 안 멈춘 것.
    """
    from config.system import (MEASURE_FRAMES, MEASURE_MAX_FRAMES,
                           STABLE_HEADING_DEG, STABLE_LATERAL_M)
    n = int(n or MEASURE_FRAMES)
    max_frames = int(max_frames or MEASURE_MAX_FRAMES)

    keys = ("lateral", "vertical", "forward", "distance",
            "approach_deg", "heading_deg", "tilt_deg",
            "heading_sigma_deg")
    got = {k: [] for k in keys}
    got["z_optical"] = []
    seen = 0

    for res in results:
        seen += 1
        tid = tag_id
        if tid is None:
            tid = next(iter(res.docking), None)   
        if tid is None or tid not in res.docking:
            if seen >= max_frames:
                break
            continue
        if require_ok and not res.quality.get(tid, {}).get("ok", True):
            if seen >= max_frames:
                break
            continue
        d = res.docking[tid]
        for k in keys:
            got[k].append(float(d[k]))
        got["z_optical"].append(float(res.poses[tid][2, 3]))
        if len(got["lateral"]) >= n or seen >= max_frames:
            break

    m = len(got["lateral"])
    if m == 0:
        return None

    def med(v):
        s = sorted(v)
        return s[m // 2] if m % 2 else 0.5 * (s[m // 2 - 1] + s[m // 2])

    def stderr(v):
        if m < 2:
            return float("inf")
        mu = sum(v) / m
        var = sum((x - mu) ** 2 for x in v) / (m - 1)
        return (var ** 0.5) / (m ** 0.5)

    out = {k: med(v) for k, v in got.items()}
    out["n"] = m
    out["spread"] = {k: stderr(v) for k, v in got.items()}
    from config.system import MAX_HEADING_SIGMA_DEG
    sig = out.get("heading_sigma_deg", float("nan"))
    out["reliable_angle"] = bool(sig <= MAX_HEADING_SIGMA_DEG) if np.isfinite(sig) \
        else bool(out["tilt_deg"] >= RELIABLE_TILT_DEG)

    # 흔들림 판정은 **두 조건을 같이** 본다.
    #
    #   ① 예측보다 훨씬 흔들리나   기하로 예측한 흔들림(heading_sigma_deg)과 대조한다.
    #      진동·부분 가림·모션블러·태그가 흔들림 처럼 **기하가 모르는 일**은
    #      여기서만 잡힌다. 예측 자체는 그런 걸 모른다.
    #   ② 그게 실제로 문제가 되나  아무리 예측 대비 커도 허용치의 1/3 밑이면
    #      명령에 영향이 없다. 가까이서 예측이 0.02도인데 3배 흔들린다고
    #      멈추면 멀쩡한 측정을 버리는 꼴이다.
    #
    # 예전에는 ②만 봤는데, 그 문턱이 reliable_angle 보다 늘 느슨해서
    # (같은 눈금으로 환산하면 3.65도 vs 0.50도) **한 번도 걸릴 수 없었다.**
    from config.system import STABLE_SPREAD_K
    reasons = []
    if m < n:
        reasons.append("프레임 부족 %d/%d" % (m, n))
    pred_h = out.get("heading_sigma_deg")
    pred_h = (pred_h / np.sqrt(m)) if (pred_h and np.isfinite(pred_h)) else None
    obs_l, obs_h = out["spread"]["lateral"], out["spread"]["heading_deg"]
    if obs_l > STABLE_LATERAL_M:
        reasons.append("lateral 흔들림 %.1fmm" % (obs_l * 1000))
    if obs_h > STABLE_HEADING_DEG:
        reasons.append("heading 흔들림 %.2f도" % obs_h)
    if pred_h and obs_h > max(STABLE_SPREAD_K * pred_h, STABLE_HEADING_DEG / 3.0):
        reasons.append("예측보다 %.1f배 흔들림 (%.3f -> %.3f도)"
                       % (obs_h / pred_h, pred_h, obs_h))
    out["stable"] = not reasons
    out["reasons"] = reasons
    return out
