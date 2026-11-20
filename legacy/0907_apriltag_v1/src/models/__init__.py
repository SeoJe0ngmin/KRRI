"""도킹 파이프라인 본체.

    detection/  image -> detection_tag -> detection_pose
    control/    control, control_forklift_v2, fwd_time_model
"""
from .detection.image import (
    CameraIntrinsics, intrinsics_from_hfov, fov_edges_deg, tag_visible_near_m, intrinsics_from_ref,
    D435I_COLOR_REF,
    Frame, open_realsense, open_bag, from_video,
    open_webcam, list_cameras, camera_index, depth_at, to_gray,
)
from .detection.detection_tag import (
    make_detector, detect, tag_pixel_size, tag_edge_margin_px,
    DEFAULT_QUAD_BLUR, MIN_TAG_PX, STABLE_TAG_PX,
)
from .detection.detection_pose import (
    estimate_pose, pose_by_pnp, pose_to_xyzrpy, pose_to_forklift,
    docking_state, tag_tilt_deg, heading_sigma_deg,
    pose_quality, depth_cross_check,
    MAX_REPROJ_RMS_PX, MIN_DECISION_MARGIN, RELIABLE_TILT_DEG,
    DEPTH_TOL_COEF, DEPTH_TOL_FLOOR_M, DEPTH_CHECK_MAX_Z,
    TagPipeline, Result, measure,
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
           "DEPTH_TOL_COEF", "DEPTH_TOL_FLOOR_M", "DEPTH_CHECK_MAX_Z",
           "TagPipeline", "Result", "measure"]
