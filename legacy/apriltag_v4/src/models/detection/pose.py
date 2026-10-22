"""자세 구하기 — 태그에서 lateral / forward / heading / 화면위치(beta) 까지.

수식은 apriltag_v2/src/models/detection/detection_pose.py 의 docking_state·pose_to_forklift
그대로다. 두 해 뽑기는 apriltag_v3 의 pnp2_solutions 그대로.
v4 가 더한 것: 화면위치 beta · 태그 기울기 보정(중력) · 모호율로 각도 버리기.
"""
import math

import cv2
import numpy as np

from config import detection as D

_IPPE_ORDER = (3, 2, 1, 0)


def _object_points(tag_size):
    """태그 네 모서리의 3D 좌표. detection.corners 와 같은 순서."""
    s = tag_size / 2.0
    return np.array([[-s, -s, 0.], [s, -s, 0.], [s, s, 0.], [-s, s, 0.]],
                    dtype=np.float64)


def invert_T(T):
    """4x4 역변환 (T_A_B -> T_B_A). apriltag_v2/src/utils/util.py 와 같은 식.

    util.py 를 통째로 안 가져오는 이유: 그 파일이 matplotlib 을 import 해서
    제어 루프에 무거운 의존이 붙는다. 우리가 쓰는 건 이 함수 하나뿐이다.
    """
    R, t = np.asarray(T)[:3, :3], np.asarray(T)[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def pose_by_pnp(detection, intrinsics, tag_size):
    """solvePnP 로 자세 하나. (T, 재투영 RMS px)."""
    obj = _object_points(tag_size)
    img = np.asarray(detection.corners, dtype=np.float64).reshape(-1, 1, 2)
    dist = (np.array(intrinsics.distortion, dtype=np.float64)
            if getattr(intrinsics, "distortion", None) else np.zeros(5))
    ok, rvec, tvec = cv2.solvePnP(obj, img, intrinsics.K, dist,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return np.full((4, 4), np.nan), float("inf")
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    proj, _ = cv2.projectPoints(obj, rvec, tvec, intrinsics.K, dist)
    err = float(np.sqrt(((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2)
                        .sum(axis=1)).mean())
    return T, err


def pnp2(detection, intrinsics, tag_size):
    """평면 PnP **두 해**. err_ratio 가 1 에 가까우면 어느 쪽인지 구분이 안 된다.

    9/21 실측 모호율 14 % (3.5 m). 여러 프레임의 중앙값으로는 안 잡힌다 —
    치우침이지 흔들림이 아니라서 (plan 3-6).
    """
    try:
        obj = _object_points(tag_size)[list(_IPPE_ORDER)].reshape(-1, 1, 3)
        img = np.asarray(detection.corners, dtype=np.float64)[list(_IPPE_ORDER)]
        img = img.reshape(-1, 1, 2)
        dist = (np.array(intrinsics.distortion, dtype=np.float64)
                if getattr(intrinsics, "distortion", None) else np.zeros(5))
        out = cv2.solvePnPGeneric(obj, img, intrinsics.K, dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except Exception:
        return None
    rvecs, tvecs = out[1], out[2]
    if not rvecs:
        return None
    err = []
    for rv, tv in zip(rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rv, tv, intrinsics.K, dist)
        d = proj.reshape(-1, 2) - img.reshape(-1, 2)
        err.append(float(np.sqrt((d ** 2).sum(axis=1).mean())))
    ratio = (float(min(err) / max(err)) if len(err) == 2 and max(err) > 0 else None)
    return {"reproj_px": err, "err_ratio": ratio, "n_sol": len(rvecs)}


def bearing_deg(detection, intrinsics):
    """화면에서 태그 중심이 광축에서 몇 도 벗어났나. **계산이 안 들어가 뒤집힘이 없다.**

    0.3 px = 0.013도. 9/21 정지 실측 흔들림 0.008도 (plan 3-7).
    """
    cx_px = float(np.asarray(detection.center)[0])
    return math.degrees(math.atan2(cx_px - intrinsics.cx, intrinsics.fx))


def tag_roll_deg(T_camera_tag, accel):
    """태그가 액자처럼 기울어진 각 [도]. 중력이 절대 기준이라 카메라 기울기까지 같이 잡는다.

    태그 가로축이 수평이면 중력과 직각이다. accel 은 (x,y,z) 원시값 — 정지 중에만 쓴다
    (움직이면 가속도가 섞인다). plan 5-3.
    """
    if accel is None:
        return None
    g = np.asarray(accel, dtype=float)
    n = np.linalg.norm(g)
    if n < 1e-6:
        return None
    x_tag = np.asarray(T_camera_tag)[:3, 0]      # 태그 가로축 (카메라 좌표계)
    nx = np.linalg.norm(x_tag)
    if nx < 1e-9:
        return None
    return math.degrees(math.asin(np.clip(float((x_tag / nx) @ (g / n)), -1.0, 1.0)))


def state(T_camera_tag, detection=None, intrinsics=None, tag_size=None,
          cam_yaw_offset_deg=0.0, tag_roll_correction_deg=0.0, accel=None):
    """제어가 쓰는 값. lateral / forward / heading 셋이 핵심이다.

    cam_yaw_offset_deg 는 config CAM_YAW_OFFSET_DEG (calibrate camyaw 실측) — 카메라가 차체 정면과 어긋나게 달린 각.
    tag_roll_correction_deg 는 태그 액자 기울기. 높이차가 좌우로 새는 걸 되돌린다.
    """
    T_tag_cam = invert_T(np.asarray(T_camera_tag))
    p, R = T_tag_cam[:3, 3], T_tag_cam[:3, :3]

    lateral, vertical, forward = float(p[0]), float(p[1]), float(-p[2])
    # 태그가 액자처럼 기울면 높이차가 좌우로 샌다. 되돌린다 (plan 5-3)
    if tag_roll_correction_deg:
        g = math.radians(tag_roll_correction_deg)
        lateral = lateral * math.cos(g) + vertical * math.sin(g)

    fwd = R @ np.array([0.0, 0.0, 1.0])          # 카메라 광축이 태그 좌표계에서 향하는 곳
    heading = math.degrees(math.atan2(fwd[0], fwd[2])) - cam_yaw_offset_deg

    out = {"lateral": lateral, "vertical": vertical, "forward": forward,
           "distance": float(np.linalg.norm(p)), "heading_deg": heading,
           "tilt_deg": _tilt_deg(T_camera_tag),
           "beta_deg": None, "err_ratio": None, "tag_roll_deg": None,
           "angle_ok": True}
    if detection is not None and intrinsics is not None:
        out["beta_deg"] = bearing_deg(detection, intrinsics)
        if tag_size:
            two = pnp2(detection, intrinsics, tag_size)
            if two:
                out["err_ratio"] = two["err_ratio"]
    if accel is not None:
        out["tag_roll_deg"] = tag_roll_deg(T_camera_tag, accel)
    # 두 해가 구분이 안 되면 **각도를 통째로 버린다.** 화면위치(beta)만 쓴다
    r = out["err_ratio"]
    out["angle_ok"] = not (r is not None and r > D.AMBIGUITY_RATIO)
    return out


def _tilt_deg(T_camera_tag):
    """태그면이 카메라를 정면으로 마주보는 정도. 0 이면 정면(두 해가 제일 헷갈린다)."""
    n = np.asarray(T_camera_tag)[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return math.degrees(math.acos(min(1.0, abs(float(n[2])))))
