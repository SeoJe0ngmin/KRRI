"""AprilTag 도킹 — 패키지 표면."""
# 설정은 패키지 밖(저장소 루트의 config/)에 있다. 실사용값과 시뮬레이터값을
# 갈라 두려고 그렇게 했다. 그래서 어디서 import 하든 루트가 보이게 해 둔다.
import os as _os
import sys as _sys

_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _ROOT not in _sys.path:
    _sys.path.insert(0, _ROOT)

from .models import (
    # 1) 카메라 값
    CameraIntrinsics, intrinsics_from_hfov, fov_edges_deg, tag_visible_near_m, intrinsics_from_ref,
    D435I_COLOR_REF,
    # 2) 이미지 얻기 — 어느 소스든 `for i, ts, img in frames:` 3-튜플이다
    Frame, open_realsense, open_bag, from_video,
    open_webcam, list_cameras, camera_index,
    depth_at, to_gray,
    # 3) 태그 찾기
    make_detector, detect, tag_pixel_size, tag_edge_margin_px,
    DEFAULT_QUAD_BLUR, MIN_TAG_PX, STABLE_TAG_PX,
    # 4) 자세 구하기 — docking_state/tag_tilt_deg 는 C++ 필터와 짝이라 본문이 얼어 있다
    estimate_pose, pose_by_pnp, pose_to_xyzrpy, pose_to_forklift,
    docking_state, tag_tilt_deg, heading_sigma_deg,
    # 5) 품질 판정
    pose_quality, depth_cross_check,
    MAX_REPROJ_RMS_PX, MIN_DECISION_MARGIN, RELIABLE_TILT_DEG,
    # 6) 파이프라인 — 편의층이다. 위 낱개 함수들을 대체하지 않는다
    TagPipeline, Result,
)

__all__ = ["CameraIntrinsics", "intrinsics_from_hfov", "fov_edges_deg", "tag_visible_near_m", "intrinsics_from_ref",
           "D435I_COLOR_REF",
           "Frame", "open_realsense", "open_bag", "from_video",
           "open_webcam", "list_cameras", "camera_index",
           "depth_at", "to_gray",
           "make_detector", "detect", "tag_pixel_size", "tag_edge_margin_px",
           "DEFAULT_QUAD_BLUR", "MIN_TAG_PX", "STABLE_TAG_PX",
           "estimate_pose", "pose_by_pnp", "pose_to_xyzrpy", "pose_to_forklift",
           "docking_state", "tag_tilt_deg", "heading_sigma_deg",
           "pose_quality", "depth_cross_check",
           "MAX_REPROJ_RMS_PX", "MIN_DECISION_MARGIN", "RELIABLE_TILT_DEG",
           "TagPipeline", "Result"]
