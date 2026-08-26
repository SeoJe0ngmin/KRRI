"""도킹 파이프라인 본체.

    tag_pose.py   이미지 -> 검출 -> 자세 -> 도킹 값 (1~6단계가 한 파일에 있다)
    control.py    비어 있다. 제어(주행 명령)가 들어올 자리 — 아직 아무것도 없다.

여기서는 tag_pose 의 공개 이름만 끌어올린다. **control 은 import 하지 않는다** —
0바이트 파일이라 끌어올 것이 없다.

카메라를 안 만지는 이름들(make_detector/detect/estimate_pose/docking_state ...)은
pyrealsense2 없이도 그대로 돌아간다. pyrealsense2 는 open_realsense/open_bag
안에서만 늦게 import 하므로, 카메라가 없는 PC 에서도 이 패키지는 열린다.
"""
from .tag_pose import (
    # 1) 카메라 값
    CameraIntrinsics, intrinsics_from_hfov, ASSUMED_HFOV_DEG,
    # 2) 이미지 얻기 — 어느 소스든 `for i, ts, img in frames:` 3-튜플이다
    Frame, open_realsense, open_bag, from_video,
    depth_at, to_gray,
    # 3) 태그 찾기
    make_detector, detect, tag_pixel_size,
    DEFAULT_QUAD_BLUR, MIN_TAG_PX, STABLE_TAG_PX,
    # 4) 자세 구하기 — docking_state/tag_tilt_deg 는 C++ 필터와 짝이라 본문이 얼어 있다
    estimate_pose, pose_by_pnp, pose_to_xyzrpy, docking_state, tag_tilt_deg,
    # 5) 품질 판정 — 임계값 근거는 tag_pose.py 의 각 상수 주석에 있다
    pose_quality, depth_cross_check,
    MAX_REPROJ_RMS_PX, MIN_DECISION_MARGIN, RELIABLE_TILT_DEG,
    DEPTH_TOL_COEF, DEPTH_TOL_FLOOR_M, DEPTH_CHECK_MAX_Z,
    # 6) 파이프라인 — 편의층이다. 위 낱개 함수들을 대체하지 않는다
    TagPipeline, Result,
)

__all__ = ["CameraIntrinsics", "intrinsics_from_hfov", "ASSUMED_HFOV_DEG",
           "Frame", "open_realsense", "open_bag", "from_video",
           "depth_at", "to_gray",
           "make_detector", "detect", "tag_pixel_size",
           "DEFAULT_QUAD_BLUR", "MIN_TAG_PX", "STABLE_TAG_PX",
           "estimate_pose", "pose_by_pnp", "pose_to_xyzrpy",
           "docking_state", "tag_tilt_deg",
           "pose_quality", "depth_cross_check",
           "MAX_REPROJ_RMS_PX", "MIN_DECISION_MARGIN", "RELIABLE_TILT_DEG",
           "DEPTH_TOL_COEF", "DEPTH_TOL_FLOOR_M", "DEPTH_CHECK_MAX_Z",
           "TagPipeline", "Result"]
