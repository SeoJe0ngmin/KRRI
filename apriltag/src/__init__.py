"""AprilTag 도킹 — 패키지 표면."""
from .models import (
    # 1) 카메라 값
    CameraIntrinsics, intrinsics_from_hfov, intrinsics_from_ref,
    ASSUMED_HFOV_DEG, D435I_COLOR_REF,
    # 2) 이미지 얻기 — 어느 소스든 `for i, ts, img in frames:` 3-튜플이다
    Frame, open_realsense, open_bag, from_video,
    depth_at, to_gray,
    # 3) 태그 찾기
    make_detector, detect, tag_pixel_size,
    DEFAULT_QUAD_BLUR, MIN_TAG_PX, STABLE_TAG_PX,
    # 4) 자세 구하기 — docking_state/tag_tilt_deg 는 C++ 필터와 짝이라 본문이 얼어 있다
    estimate_pose, pose_by_pnp, pose_to_xyzrpy, docking_state, tag_tilt_deg,
    # 5) 품질 판정
    pose_quality, depth_cross_check,
    MAX_REPROJ_RMS_PX, MIN_DECISION_MARGIN, RELIABLE_TILT_DEG,
    DEPTH_TOL_COEF, DEPTH_TOL_FLOOR_M, DEPTH_CHECK_MAX_Z,
    # 6) 파이프라인 — 편의층이다. 위 낱개 함수들을 대체하지 않는다
    TagPipeline, Result,
)

__all__ = ["CameraIntrinsics", "intrinsics_from_hfov", "intrinsics_from_ref",
           "ASSUMED_HFOV_DEG", "D435I_COLOR_REF",
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
