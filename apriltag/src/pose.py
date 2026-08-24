"""3단계 — 자세 구하기.

블로그의 이 부분에 해당한다.

    pose, e0, e1 = detector.detection_pose(
        detection=r, camera_params=cam_params_rgb, tag_size=tag_size)

원리: 정사각형 태그가 화면에서 얼마나 찌그러졌는지(homography)와
      카메라 값(fx,fy,cx,cy), 그리고 태그의 실제 크기를 알면
      거리와 각도가 역산된다. depth 센서는 필요 없다.
      라이브러리 내부에서 pose_from_homography() 가 이 일을 한다.

주는 값은 T_camera_tag — "카메라 기준으로 본 태그의 자세"다.
반대로 "태그 기준으로 본 카메라" 가 필요하면 utils.invert_T 로 뒤집는다.
"""
import cv2
import numpy as np

from utils.util import r2rpy, invert_T, t2pr


def estimate_pose(detector, detection, intrinsics, tag_size):
    """태그 하나의 자세를 구한다.

    Args:
        intrinsics: CameraIntrinsics
        tag_size: 태그 실제 한 변 길이 [m]. 검은 테두리까지 포함한 값이다.
                  이 값이 틀리면 거리가 그 비율만큼 통째로 틀어진다.

    Returns:
        (T_camera_tag 4x4, init_error, final_error)
        두 오차는 다듬기 전/후 재투영 오차다. 둘이 비슷하게 크면 자세를 믿기 어렵다.
    """
    T, e0, e1 = detector.detection_pose(
        detection=detection,
        camera_params=intrinsics.params,
        tag_size=tag_size,
    )
    return np.asarray(T), e0, e1


def pose_to_xyzrpy(T, unit='deg'):
    """4x4 -> 읽을 수 있는 형태.

    Returns:
        dict(x, y, z, roll, pitch, yaw, distance)
    """
    p, R = t2pr(T)
    rpy = r2rpy(R, unit=unit)
    return {"x": p[0], "y": p[1], "z": p[2],
            "roll": rpy[0], "pitch": rpy[1], "yaw": rpy[2],
            "distance": float(np.linalg.norm(p))}


def camera_pose_in_tag(T_camera_tag):
    """카메라 기준 태그 -> 태그 기준 카메라.

    도킹처럼 "목표물 기준으로 내가 어디 있나" 가 필요할 때 쓴다.
    """
    return invert_T(T_camera_tag)


def draw_cube(overlay, camera_params, tag_size, pose, z_sign=1):
    """태그 위에 정육면체를 그려 자세를 눈으로 확인한다.

    태그 좌표계에서 정육면체 꼭짓점을 정의하고, 방금 구한 pose 로
    이미지에 되투영한다. 상자가 태그에 딱 붙어 보이면 자세가 맞은 것이다.

    [원본] apriltag 패키지의 _draw_pose() 를 그대로 가져왔다.
           블로그도 이걸 _draw_cube 로 이름만 바꿔 썼다.
           원본은 ref/apriltag_pkg_source.py 참고.
    """
    opoints = np.array([
        -1, -1, 0,
         1, -1, 0,
         1,  1, 0,
        -1,  1, 0,
        -1, -1, -2 * z_sign,
         1, -1, -2 * z_sign,
         1,  1, -2 * z_sign,
        -1,  1, -2 * z_sign,
    ]).reshape(-1, 1, 3) * 0.5 * tag_size

    edges = np.array([
        0, 1, 1, 2, 2, 3, 3, 0,
        0, 4, 1, 5, 2, 6, 3, 7,
        4, 5, 5, 6, 6, 7, 7, 4,
    ]).reshape(-1, 2)

    fx, fy, cx, cy = camera_params
    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    tvec = pose[:3, 3]
    dcoeffs = np.zeros(5)

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)
    ipoints = np.round(ipoints).astype(int)
    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)
    return overlay


def draw_axes(overlay, camera_params, tag_size, pose, length=None, thickness=3):
    """태그 원점에 좌표축을 그린다. X=빨강, Y=초록, Z=파랑.

    draw_cube 가 상자를 그리는 대신 축 세 개를 그리는 버전이다.
    축이 태그면에 붙어 보이고 Z축이 태그에서 수직으로 솟으면 자세가 맞은 것이다.

    Args:
        length: 축 길이 [m]. 기본은 태그 한 변의 절반.
    """
    L = length if length is not None else tag_size * 0.5
    opoints = np.float32([[0, 0, 0], [L, 0, 0], [0, L, 0], [0, 0, -L]]).reshape(-1, 3)

    fx, fy, cx, cy = camera_params
    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    tvec = pose[:3, 3]

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, np.zeros(5))
    o, x, y, z = [tuple(p) for p in np.round(ipoints).astype(int).reshape(-1, 2)]

    cv2.line(overlay, o, x, (0, 0, 255), thickness, 16)     # X 빨강
    cv2.line(overlay, o, y, (0, 255, 0), thickness, 16)     # Y 초록
    cv2.line(overlay, o, z, (255, 0, 0), thickness, 16)     # Z 파랑
    return overlay


def draw_corners(overlay, detection, color=(0, 0, 255), thickness=2):
    """검출된 태그의 네 모서리를 잇고 id 를 적는다."""
    pts = np.round(detection.corners).astype(int)
    cv2.polylines(overlay, [pts.reshape(-1, 1, 2)], True, color, thickness, 16)
    cx, cy = np.round(detection.center).astype(int)
    cv2.putText(overlay, f"id:{detection.tag_id}", (cx - 20, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, 16)
    return overlay
