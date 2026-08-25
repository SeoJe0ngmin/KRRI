"""정답을 아는 합성 이미지로 자세 추정 정확도를 잰다.

실제 영상에는 "이 태그가 몇 도 기울어져 있다"는 정답이 없다.
그래서 **우리가 각도를 정해서 그 각도로 보이는 이미지를 만들고**, 검출기가
그 각도를 되찾아내는지 본다. 정답은 우리가 넣은 값이다.

이 방법이 검증하는 것과 못 하는 것
    검증됨 : 모서리 픽셀 -> 자세 계산 경로가 정확한가
    못 함  : 렌즈 왜곡, 센서 노이즈, 모션 블러, 조명
렌더링이 수학적으로 완벽한 원근 변환이라 실제 카메라의 결함이 하나도 없다.
따라서 여기 수치는 **상한선**이다. 실물에서는 이보다 나빠진다.

산출물은 work_dirs/accuracy_test/ 에 쌓인다.

사용법
    python tools/accuracy_test.py                 # 기울기 스윕
    python tools/accuracy_test.py --save          # 합성 이미지도 저장
    python tools/accuracy_test.py --distance 5    # 5m 거리에서
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src import (CameraIntrinsics, make_detector, detect,          # noqa: E402
                 estimate_pose, pose_to_xyzrpy)

TILTS = [0, 0.25, 0.5, 1, 2, 3, 5, 8, 12, 20, 30, 45, 60]


def make_tag_image(tag_id=0, px=600, pad=150):
    """tag36h11 이미지 + 흰 여백. 여백이 없으면 검출기가 태그를 못 찾는다."""
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    core = cv2.aruco.generateImageMarker(dic, tag_id, px)
    full = cv2.copyMakeBorder(core, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    # 태그 실제 영역(여백 제외)의 네 모서리 — 여기가 물리적 태그 크기에 대응한다
    src = np.float32([[pad, pad], [pad + px, pad], [pad + px, pad + px], [pad, pad + px]])
    return full, src


def render(tag_img, src_quad, intr, tag_size, tilt_deg, distance, roll_deg=0.0):
    """태그를 tilt_deg 만큼 돌려 distance 앞에 놓고 본 화면을 합성한다.

    Returns:
        (합성 이미지, 정답 R 3x3, 정답 t 3)
    """
    s = tag_size / 2
    obj = np.float32([[-s, -s, 0], [s, -s, 0], [s, s, 0], [-s, s, 0]])   # AprilTag 모서리 순서

    a, r = np.deg2rad(tilt_deg), np.deg2rad(roll_deg)
    Ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    Rz = np.array([[np.cos(r), -np.sin(r), 0], [np.sin(r), np.cos(r), 0], [0, 0, 1]])
    R = Ry @ Rz
    t = np.array([0.0, 0.0, distance])

    pts = (R @ obj.T).T + t
    uv = (intr.K @ pts.T).T
    uv = (uv[:, :2] / uv[:, 2:]).astype(np.float32)

    M = cv2.getPerspectiveTransform(src_quad, uv)
    img = cv2.warpPerspective(tag_img, M, (intr.width, intr.height), borderValue=255)
    return img, R, t


def tilt_of(R):
    """태그 법선과 카메라 광축이 이루는 각 [도]. 좌표계 규약과 무관한 지표."""
    nz = np.asarray(R) @ np.array([0, 0, 1.0])
    return float(np.degrees(np.arccos(min(1.0, abs(nz[2])))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--distance", type=float, default=2.0, help="태그까지 거리 [m]")
    ap.add_argument("--tag-size", type=float, default=0.20, help="태그 한 변 [m]")
    ap.add_argument("--hfov", type=float, default=60.0, help="수평화각 [도]")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=960)
    ap.add_argument("--save", action="store_true", help="합성 이미지를 파일로 남긴다")
    ap.add_argument("--outdir", default=None)
    args = ap.parse_args()

    W, H = args.width, args.height
    fx = (W / 2) / np.tan(np.deg2rad(args.hfov) / 2)
    intr = CameraIntrinsics(fx, fx, W / 2, H / 2, W, H)
    detector = make_detector()
    tag_img, src_quad = make_tag_image()

    outdir = Path(args.outdir or (ROOT / "work_dirs" / "accuracy_test"))
    if args.save:
        outdir.mkdir(parents=True, exist_ok=True)

    print(f"카메라   : {W}x{H}, HFOV {args.hfov}도 -> fx={fx:.1f}")
    print(f"태그     : {args.tag_size} m, 거리 {args.distance} m")
    print(f"검출기   : tag36h11, quad_blur={0}\n")
    print(f"{'정답기울기':>10} {'추정':>8} {'오차':>8} {'거리추정':>10} {'거리오차':>10} {'재투영':>9}")

    rows = []
    for truth in TILTS:
        img, R_true, t_true = render(tag_img, src_quad, intr, args.tag_size,
                                     truth, args.distance)
        if args.save:
            cv2.imwrite(str(outdir / f"tilt_{truth:05.2f}.png"), img)

        res = detect(detector, img)
        if not res:
            print(f"{truth:>10} {'검출실패':>8}")
            rows.append((truth, None, None, None))
            continue

        T, e0, e1 = estimate_pose(detector, res[0], intr, args.tag_size)
        if not np.isfinite(T).all():
            print(f"{truth:>10} {'NaN':>8}")
            rows.append((truth, None, None, None))
            continue

        est = tilt_of(T[:3, :3])
        dist = float(np.linalg.norm(T[:3, 3]))
        print(f"{truth:>10} {est:>8.2f} {est - truth:>+8.2f} "
              f"{dist:>10.4f} {dist - args.distance:>+10.4f} {e1:>9.3f}")
        rows.append((truth, est, dist, e1))

    ok = [r for r in rows if r[1] is not None]
    near = [r for r in ok if r[0] <= 10]
    far = [r for r in ok if r[0] > 10]
    if near:
        print(f"\n정면 근처(<=10도) 각도 오차 최대 : {max(abs(r[1]-r[0]) for r in near):.2f}도")
    if far:
        print(f"기울었을 때(>10도) 각도 오차 최대 : {max(abs(r[1]-r[0]) for r in far):.2f}도")
    if ok:
        print(f"거리 오차 최대                    : {max(abs(r[2]-args.distance) for r in ok)*1000:.1f} mm")
    if args.save:
        print(f"\n합성 이미지 저장: {outdir}")


if __name__ == "__main__":
    main()
