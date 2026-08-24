"""AprilTag 자세 추정 파이프라인.

    image_source  이미지를 얻는다   (rosbag / 이미지파일 / 카메라)
          |
    detection     태그를 찾는다     detector.detect()
          |
    pose          자세를 구한다     detector.detection_pose()

[원본] 블로그: joonhyung-lee.github.io/blog/2023/apriltag-pose-estimation/
       라이브러리 원본은 ref/apriltag_pkg_source.py 에 보관.
"""
from .image_source import CameraIntrinsics, from_rosbag, from_files, debayer, to_gray, meters2xyz
from .detection import make_detector, detect
from .tag_layout import (TagPlacement, TagLayout, facing_approach,
                         average_rotations, DOCK_LAYOUT)
from .pose import (estimate_pose, pose_to_xyzrpy, camera_pose_in_tag,
                   draw_cube, draw_axes, draw_corners)

__all__ = ["CameraIntrinsics", "from_rosbag", "from_files", "debayer", "to_gray", "meters2xyz",
           "make_detector", "detect",
           "estimate_pose", "pose_to_xyzrpy", "camera_pose_in_tag",
           "draw_cube", "draw_axes", "draw_corners",
           "TagPlacement", "TagLayout", "facing_approach", "average_rotations", "DOCK_LAYOUT"]
