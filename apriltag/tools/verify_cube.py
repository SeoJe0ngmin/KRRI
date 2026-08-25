"""블로그의 큐브 검증을 우리 데이터에 돌린다.

블로그는 태그 위에 정육면체를 그려서 자세가 맞았는지 눈으로 확인한다.
상자가 태그면에 딱 붙고 기울기가 태그와 같이 움직이면 자세가 맞은 것이고,
따로 놀거나 찌그러지면 틀린 것이다. 숫자만 봐서는 알기 어려운 걸 잡아준다.

블로그는 RealSense 화면에 실시간으로 그렸지만(cv2.imshow),
우리는 카메라가 없으므로 영상/rosbag 을 읽어 파일로 저장한다.

산출물은 work_dirs/verify_cube/<source>/ 에 쌓인다.

사용법
    python tools/verify_cube.py                        # 기본: 테스트 영상
    python tools/verify_cube.py --source tagslam       # tagslam 실촬영
    python tools/verify_cube.py --axes                 # 큐브 대신 좌표축
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src import (CameraIntrinsics, from_rosbag, to_gray,          # noqa: E402
                 make_detector, detect, estimate_pose, pose_to_xyzrpy,
                 draw_cube, draw_axes, draw_corners)

DATA = Path("/home/jeongmin/work/projects/krri/data/apriltag")


def frames_from_video(path):
    cap = cv2.VideoCapture(str(path))
    i = 0
    while True:
        ok, bgr = cap.read()
        if not ok:
            break
        yield i, bgr
        i += 1
    cap.release()


def source_video():
    """합성 테스트 영상. 카메라 값을 모르므로 화각 60도로 가정한다."""
    path = DATA / "Testing_apriltag.mp4"
    cap = cv2.VideoCapture(str(path))
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()
    fx = (W / 2) / np.tan(np.deg2rad(60.0) / 2)
    return frames_from_video(path), CameraIntrinsics(fx, fx, W / 2, H / 2, W, H), 0.16


def source_tagslam():
    """tagslam 실촬영. 카메라 값과 태그 크기가 저장소에 같이 들어있다."""
    d = DATA / "tagslam"
    intr = CameraIntrinsics.from_yaml(d / "cameras.yaml")
    gen = ((i, bgr) for i, _, bgr in from_rosbag(d / "example.bag"))
    return gen, intr, 0.12


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="video", choices=["video", "tagslam"])
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--axes", action="store_true", help="큐브 대신 좌표축을 그린다")
    ap.add_argument("--tag-size", type=float, default=None, help="태그 한 변 [m] 덮어쓰기")
    args = ap.parse_args()

    frames, intr, tag_size = source_video() if args.source == "video" else source_tagslam()
    if args.tag_size:
        tag_size = args.tag_size
    outdir = Path(args.outdir or (ROOT / "work_dirs" / "verify_cube" / args.source))
    outdir.mkdir(parents=True, exist_ok=True)

    detector = make_detector()
    print(f"source     : {args.source}")
    print(f"카메라     : fx={intr.fx:.1f} fy={intr.fy:.1f} cx={intr.cx:.1f} cy={intr.cy:.1f}")
    print(f"태그 크기  : {tag_size} m")
    print(f"저장 위치  : {outdir}\n")

    n_frames = n_hit = 0
    for i, bgr in frames:
        n_frames += 1
        results = detect(detector, to_gray(bgr))
        if not results:
            continue
        n_hit += 1

        vis = bgr.copy()
        for r in results:
            T, e0, e1 = estimate_pose(detector, r, intr, tag_size)
            if not np.isfinite(T).all():
                print(f"  frame {i:>3} id={r.tag_id}: 자세 계산 실패")
                continue
            draw_corners(vis, r)
            if args.axes:
                draw_axes(vis, intr.params, tag_size, T)
            else:
                draw_cube(vis, intr.params, tag_size, T)      # ← 블로그의 검증 방식

            v = pose_to_xyzrpy(T)
            print(f"  frame {i:>3} id={r.tag_id:<3} "
                  f"dist={v['distance']:.3f}m  "
                  f"rpy=({v['roll']:+6.1f},{v['pitch']:+6.1f},{v['yaw']:+6.1f})  "
                  f"reproj={e1:.3f}")
        cv2.imwrite(str(outdir / f"{i:03d}.png"), vis)

    print(f"\n{n_frames} 프레임 중 {n_hit} 프레임에서 검출 "
          f"({100 * n_hit / max(1, n_frames):.0f}%)")
    print("상자가 태그면에 붙어 보이면 자세가 맞은 것이다.")


if __name__ == "__main__":
    main()
