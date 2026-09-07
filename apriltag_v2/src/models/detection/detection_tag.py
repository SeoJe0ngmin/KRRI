"""2. 태그 찾기 — 흑백 이미지에서 AprilTag 을 검출함."""
from config import detection as D
import numpy as np
import pupil_apriltags

_keep_alive = []

def make_detector(families="tag36h11", quad_blur=D.DEFAULT_QUAD_BLUR, **options):
    """검출기를 만듦."""
    # AT3 는 blur 인자 이름이 quad_sigma
    if quad_blur:
        options.setdefault("quad_sigma", quad_blur)
    options.setdefault("nthreads", 2)
    det = pupil_apriltags.Detector(families=families, **options)
    _keep_alive.append(det)    
    return det

def tag_pixel_size(detection):
    """태그 한 변의 화면상 길이 [px]."""
    c = np.asarray(detection.corners, dtype=np.float64)
    edges = np.linalg.norm(c - np.roll(c, -1, axis=0), axis=1)
    return float(edges.mean())


def tag_edge_margin_px(detection, shape):
    """태그가 화면 가장자리에서 몇 px 떨어져 있나. 가장 가까운 쪽 하나. float.

    가까이 갈수록 태그를 가파르게 올려다보게 되어 어느 지점부터 윗변이 화면
    밖으로 나간다. AprilTag 은 네 모서리가 다 있어야 인식되므로 그 순간
    검출이 끊긴다 — **회전으로는 복구가 안 된다**(태그가 계속 위에 있다).

    거리로 계산해서 판단할 수도 있지만(태그높이·카메라높이·화각), 그러면
    장착 pitch 나 실측 오차가 그대로 들어온다. **화면을 직접 보는 쪽이
    무조건 맞다** — 여기 남은 px 이 0 이 되는 순간이 곧 검출이 끊기는 순간이다.

    음수면 이미 밖으로 나갔다는 뜻.
    """
    h, w = shape[:2]
    c = np.asarray(detection.corners, dtype=np.float64)
    return float(min(c[:, 0].min(), c[:, 1].min(),
                     w - 1 - c[:, 0].max(), h - 1 - c[:, 1].max()))


def detect(detector, img_gray, min_margin=0.0, max_hamming=0,
           intrinsics=None, tag_size=None):
    """태그를 찾음. 라이브러리가 만든 Detection 리스트를 거르고 정렬해 돌려줌.

    반환값 한 개는 이렇게 생겼음 (1280x720, 20cm 태그를 1.4m 앞에서 본 실측):

        Detection(tag_family = b'tag36h11',      태그 계열
                  tag_id     = 1,                태그 번호. 결과 딕셔너리의 키
                  hamming    = 0,                고쳐낸 비트 수. 0 이 정상
                  decision_margin = 42.59,       판정 여유. 밝기에 반응, 거리엔 둔감
                  center     = array([736.04, 590.1]),        화면상 중심 [px]
                  corners    = array([[806.13, 658.99],       화면상 네 모서리 [px]
                                      [671.56, 657.77],       AT2 순서로 뒤집어 놨음
                                      [670.77, 525.95],
                                      [797.53, 525.55]]),
                  homography = array([[-60.87,  19.51, 736.04],   태그평면->화면 변환
                                      [  3.35, -48.76, 590.1 ],   h33=1 로 정규화됨
                                      [  0.01,   0.03,   1.  ]]), (AT2 는 정규화 안 함)
                  pose_R     = array([[-1.  ,  0.01,  0.05],   카메라 기준 태그 회전
                                      [ 0.01, -0.92,  0.38],
                                      [ 0.05,  0.38,  0.92]]),
                  pose_t     = array([[0.15],                 카메라 기준 태그 위치 [m]
                                      [0.32],
                                      [1.39]]),
                  pose_err   = 1.83e-06)         재투영 잔차

    pose_* 세 개는 intrinsics 와 tag_size 를 넘겼을 때만 붙음.
    AT2 에는 없어서 detector.detection_pose() 를 따로 불러야 했음.
    AT2 의 goodness 는 AT3 에 없음(항상 0.0 이라 빠졌음).
    """
    kw = {}
    if intrinsics is not None and tag_size:
        kw = dict(estimate_tag_pose=True,
                  camera_params=tuple(intrinsics.params), tag_size=float(tag_size))
    results = detector.detect(img_gray, **kw)
    keep = [r for r in results
            if r.decision_margin >= min_margin and r.hamming <= max_hamming]
    for r in keep:
        r.corners = np.asarray(r.corners, dtype=float)[::-1].copy()

    # 유리에 비치면 같은 id 태그가 2개 잡혀서 큰 쪽만 남김
    best = {}
    for r in keep:
        tid = int(r.tag_id)
        if tid not in best or tag_pixel_size(r) > tag_pixel_size(best[tid]):
            best[tid] = r
    return sorted(best.values(), key=lambda r: r.tag_id)

