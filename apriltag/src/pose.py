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


# 태그 네 모서리의 3D 좌표. detection.corners 와 같은 순서다.
# 실측 확인: 정면 태그에서 corners[0] 이 화면 오른쪽아래에 오고 (-s,-s) 에 대응한다.
def _object_points(tag_size):
    s = tag_size / 2.0
    return np.array([[-s, -s, 0.], [s, -s, 0.], [s, s, 0.], [-s, s, 0.]], dtype=np.float64)


def pose_by_pnp(detection, intrinsics, tag_size):
    """OpenCV solvePnP 로 자세를 구한다. detection_pose 의 대안.

    라이브러리의 detection_pose() 는 호모그래피를 분해하는 방식이라,
    태그가 **정확히 정면(기울기 0.00도)이고 화면축과 나란할 때** 계산이 퇴화해
    NaN 을 낸다(라이브러리 내부 homography_to_pose 의 "had ta normalize!" 경고).
    도킹은 정렬각 0 이 목표 상태라 바로 그 지점에서 값이 필요하므로 이 경로를 둔다.

    실측: 기울기 0도에서 오차 0.00도, 다른 각도에서는 detection_pose 와 소수점까지 일치.

    Returns:
        (T_camera_tag 4x4, 재투영오차[px])
    """
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
    """태그 하나의 자세를 구한다.

    Args:
        intrinsics: CameraIntrinsics
        tag_size: 태그 실제 한 변 길이 [m]. 검은 테두리까지 포함한 값이다.
                  이 값이 틀리면 거리가 그 비율만큼 통째로 틀어진다.
        method:
            "auto"  detection_pose 를 쓰되 NaN 이 나오면 solvePnP 로 넘어간다 (기본)
            "tag"   detection_pose 만 쓴다 (블로그 원본 그대로)
            "pnp"   solvePnP 만 쓴다

    Returns:
        (T_camera_tag 4x4, init_error, final_error)
        두 오차는 다듬기 전/후 재투영 오차다. 둘이 비슷하게 크면 자세를 믿기 어렵다.
    """
    if method == "pnp":
        T, err = pose_by_pnp(detection, intrinsics, tag_size)
        return T, err, err

    T, e0, e1 = detector.detection_pose(
        detection=detection,
        camera_params=intrinsics.params,
        tag_size=tag_size,
    )
    T = np.asarray(T)

    if method == "auto" and not np.isfinite(T).all():
        T, err = pose_by_pnp(detection, intrinsics, tag_size)
        return T, err, err
    return T, e0, e1


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


def docking_state(T_camera_tag):
    """도킹 제어가 바로 쓸 수 있는 형태로 바꾼다.

    detection_pose() 는 "카메라 기준 태그"를 준다. 도킹에서 알고 싶은 건 반대,
    **태그(탑재부) 기준으로 지게차가 어디에 어떻게 서 있나**다.

    그런데 이게 하나로 안 된다. 두 가지가 서로 다른 것을 말한다.

        approach_deg : 태그 정면축 기준으로 내가 **어느 방향에** 있나  (위치)
        heading_deg  : 내가 태그 축과 **나란히 서 있나**              (자세)

    둘은 독립이다. 정면축 위에 있어도(approach=0) 고개를 돌리고 있으면 heading≠0 이고,
    반대로 옆에 비켜서 태그를 똑바로 바라보면 heading=0 이지만 approach≠0 이다.
    **진입하려면 둘 다 0 이어야 한다.**

    부호 규약 주의: AprilTag 의 태그 좌표계는 **+z 가 태그 뒤쪽**을 향한다.
    그대로 쓰면 카메라가 항상 z 음수에 놓여 "앞으로 몇 m" 가 음수로 나온다.
    여기서는 사람이 읽기 편하게 뒤집어서, forward 가 **양수면 태그 앞쪽**이 되게 한다.

    Returns:
        dict
          lateral    [m]  정면축에서 좌우 벗어남
          vertical   [m]  위아래 벗어남
          forward    [m]  태그면까지 수직 거리 (양수 = 태그 앞쪽)
          distance   [m]  직선 거리
          approach_deg [도] 정면축에서 벗어난 방향각
          heading_deg  [도] 진입 방향이 축과 이루는 각 (0 이면 축과 나란함)
          reliable_angle [bool] approach_deg 를 믿어도 되는지.
              태그가 정면에 가까우면(약 10도 미만) 원근 왜곡이 픽셀 이하라
              각도를 못 잰다. 그때는 lateral 로 판단해야 한다.
    """
    T_tag_cam = invert_T(np.asarray(T_camera_tag))
    p = T_tag_cam[:3, 3]
    R = T_tag_cam[:3, :3]

    lateral = float(p[0])
    vertical = float(p[1])
    forward = float(-p[2])                      # 태그 앞쪽이 양수가 되게 뒤집는다
    distance = float(np.linalg.norm(p))

    # 내가 태그 정면축에서 몇 도 벗어난 위치에 있나
    approach = float(np.degrees(np.arctan2(abs(lateral), abs(forward))))

    # 카메라 광축(+z)이 태그 좌표계에서 어디를 향하나.
    # 태그를 정면으로 마주보고 축과 나란하면 태그의 +z 방향(뒤쪽)을 향한다.
    fwd = R @ np.array([0.0, 0.0, 1.0])
    heading = float(np.degrees(np.arctan2(fwd[0], fwd[2])))

    return {"lateral": lateral, "vertical": vertical, "forward": forward,
            "distance": distance,
            "approach_deg": approach, "heading_deg": heading,
            "reliable_angle": approach >= 10.0}


def tag_tilt_deg(T_camera_tag):
    """태그면이 카메라를 정면으로 마주보는 정도 [도].

    0 이면 태그가 화면과 완전히 나란하다(정면). 이 값이 약 10도 미만이면
    원근 왜곡이 픽셀 이하라 **각도 추정을 믿을 수 없다.**
    좌표계 규약과 무관해서 roll/pitch/yaw 보다 판정 기준으로 쓰기 좋다.
    """
    n = np.asarray(T_camera_tag)[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(min(1.0, abs(n[2])))))
