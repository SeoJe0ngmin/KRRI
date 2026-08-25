"""AprilTag 자세 추정 파이프라인.

    image_source  이미지를 얻는다   (rosbag / 이미지파일 / 카메라)
          |
    detection     태그를 찾는다     detector.detect()
          |
    pose          자세를 구한다     detector.detection_pose()

[원본] 블로그: joonhyung-lee.github.io/blog/2023/apriltag-pose-estimation/
       라이브러리 원본은 ref/apriltag_pkg_source.py 에 보관.
"""
from .image_source import CameraIntrinsics, from_rosbag, from_files, from_camera, open_realsense, debayer, to_gray, meters2xyz, remove_color_overlay
from .detection import make_detector, detect, center_offset
from .tag_layout import (TagPlacement, TagLayout, facing_approach,
                         average_rotations, DOCK_LAYOUT)
from .pose import (estimate_pose, pose_by_pnp, docking_state, tag_tilt_deg, pose_to_xyzrpy, camera_pose_in_tag,
                   draw_cube, draw_axes, draw_corners)

__all__ = ["CameraIntrinsics", "from_rosbag", "from_files", "from_camera", "open_realsense", "debayer", "to_gray", "meters2xyz", "remove_color_overlay",
           "make_detector", "detect", "center_offset",
           "estimate_pose", "pose_by_pnp", "pose_to_xyzrpy", "camera_pose_in_tag",
           "draw_cube", "draw_axes", "draw_corners", "docking_state", "tag_tilt_deg",
           "TagPlacement", "TagLayout", "facing_approach", "average_rotations", "DOCK_LAYOUT"]
