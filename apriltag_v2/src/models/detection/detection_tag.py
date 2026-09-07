from config import detection as D
import numpy as np
import pupil_apriltags
_keep_alive = []

def make_detector(families='tag36h11', quad_blur=D.DEFAULT_QUAD_BLUR, **options):
    if quad_blur:
        options.setdefault('quad_sigma', quad_blur)
    options.setdefault('nthreads', 2)
    det = pupil_apriltags.Detector(families=families, **options)
    _keep_alive.append(det)
    return det

def tag_pixel_size(detection):
    c = np.asarray(detection.corners, dtype=np.float64)
    edges = np.linalg.norm(c - np.roll(c, -1, axis=0), axis=1)
    return float(edges.mean())

def tag_edge_margin_px(detection, shape):
    h, w = shape[:2]
    c = np.asarray(detection.corners, dtype=np.float64)
    return float(min(c[:, 0].min(), c[:, 1].min(), w - 1 - c[:, 0].max(), h - 1 - c[:, 1].max()))

def detect(detector, img_gray, min_margin=0.0, max_hamming=0, intrinsics=None, tag_size=None):
    kw = {}
    if intrinsics is not None and tag_size:
        kw = dict(estimate_tag_pose=True, camera_params=tuple(intrinsics.params), tag_size=float(tag_size))
    results = detector.detect(img_gray, **kw)
    keep = [r for r in results if r.decision_margin >= min_margin and r.hamming <= max_hamming]
    for r in keep:
        r.corners = np.asarray(r.corners, dtype=float)[::-1].copy()
    best = {}
    for r in keep:
        tid = int(r.tag_id)
        if tid not in best or tag_pixel_size(r) > tag_pixel_size(best[tid]):
            best[tid] = r
    return sorted(best.values(), key=lambda r: r.tag_id)
