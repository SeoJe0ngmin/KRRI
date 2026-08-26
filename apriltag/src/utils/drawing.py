"""오버레이 그리기 — 보여주기 전용.

여기 있는 함수는 전부 이미지 위에 선/글자를 얹기만 한다.
숫자를 만들어내지 않고, 제어로도 넘어가지 않는다.
도킹 값(lateral/forward/heading/tilt)은 src/models/tag_pose.py 가 만든다.
이 파일을 통째로 지워도 파이프라인은 그대로 돈다 — 화면만 비어 보일 뿐이다.

그래서 여기서는 pose 를 절대 고치지 않는다. 축 방향이 이상해 보이면
그건 그리기 버그가 아니라 pose 버그다. tag_pose.py 쪽을 봐야 한다.
"""
import cv2
import numpy as np


def draw_cube(overlay, camera_params, tag_size, pose, z_sign=1):
    """태그 위에 정육면체를 그려 자세를 눈으로 확인한다.

    태그 좌표계에서 정육면체 꼭짓점을 정의하고, 방금 구한 pose 로
    이미지에 되투영한다. 상자가 태그에 딱 붙어 보이면 자세가 맞은 것이다.

    [원본] apriltag 패키지의 _draw_pose() 를 그대로 가져왔다.
           블로그도 이걸 _draw_cube 로 이름만 바꿔 썼다.
           원본은 설치된 패키지(site-packages/apriltag.py) 안에 있다.
           (예전 주석이 가리키던 ref/apriltag_pkg_source.py 는 이제 없다.)
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
