"""AprilTag 도킹 — 패키지 표면.

    src/models/tag_pose.py   이미지 -> 검출 -> 자세 -> 품질 -> 도킹 값 (파이프라인 본체)
    src/models/control.py    비어 있다. 제어(주행 명령)가 들어올 자리다.
    src/utils/               곁다리 — 그리기, 태그 배치, RealSense 관측/제어, 좌표 변환
    src/etc/                 아직 자리를 못 정한 것들 (점검 스크립트, PDF 생성, C++ 필터)

여기서 끌어올리는 것은 **models 의 공개 이름뿐**이다. `from src import detect` 처럼
짧게 쓰라고 둔 편의 통로다.

utils 는 일부러 안 끌어올린다. 두 가지 이유다.
    - utils 에는 이 파이프라인과 무관한 파일이 섞여 있다(mujoco_parser / rrt / grpp).
      그것들은 mujoco·networkx 를 필요로 하는데 env krri 에는 없다. 여기서 utils 를
      건드리는 순간 `import src` 한 줄이 ModuleNotFoundError 로 죽는다.
      **src/utils/__init__.py 를 만들지 마라.** 만들더라도 반드시 0바이트여야 한다.
    - 이름이 겹친다. utils/util.py 의 compose() 는 동차변환 합성이고, 도구 쪽
      compose 는 화면 합성이다. 통로를 하나로 합치면 조용히 서로를 가린다.
그래서 utils 는 항상 제 경로로 부른다:

    from src.utils.drawing import draw_axes, draw_cube, draw_corners
    from src.utils.tag_layout import DOCK_LAYOUT
    from src.utils.rs_tuning import diagnose_frame
    from src.utils.util import invert_T, r2rpy

카메라(pyrealsense2)는 open_realsense/open_bag **안에서만** 늦게 import 한다.
그래서 카메라가 없는 PC 에서도 `import src` 는 그대로 열린다.

[원본] 블로그: joonhyung-lee.github.io/blog/2023/apriltag-pose-estimation/
"""
from .models.tag_pose import (
    # 1) 카메라 값
    CameraIntrinsics, intrinsics_from_hfov, ASSUMED_HFOV_DEG,
    # 2) 이미지 얻기 — 어느 소스든 `for i, ts, img in frames:` 3-튜플이다
    Frame, open_realsense, open_bag, from_video,
    depth_at, to_gray,
    # 3) 태그 찾기
    make_detector, detect, tag_pixel_size,
    DEFAULT_QUAD_BLUR, MIN_TAG_PX, STABLE_TAG_PX,
    # 4) 자세 구하기 — docking_state/tag_tilt_deg 는 C++ 필터와 짝이라 본문이 얼어 있다
    #    (src/etc/viewer_filter/apriltag-detection.cpp 가 같은 계산을 그대로 옮겨 놨다)
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
