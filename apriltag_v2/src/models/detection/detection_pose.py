from config import detection as D
from dataclasses import dataclass, field
from pathlib import Path
import cv2
import numpy as np
from ...utils.util import r2rpy, invert_T, t2pr
from .image import depth_at, from_video, intrinsics_from_hfov, intrinsics_from_ref, open_bag, open_realsense, open_webcam, to_gray
from .detection_tag import detect, make_detector, tag_pixel_size

def _object_points(tag_size):
    s = tag_size / 2.0
    return np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]], dtype=np.float64)

def pose_by_pnp(detection, intrinsics, tag_size):
    obj = _object_points(tag_size)
    img = np.asarray(detection.corners, dtype=np.float64).reshape(-1, 1, 2)
    dist = np.array(intrinsics.distortion, dtype=np.float64) if intrinsics.distortion else np.zeros(5)
    ok, rvec, tvec = cv2.solvePnP(obj, img, intrinsics.K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return (np.full((4, 4), np.nan), float('inf'))
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.ravel()
    proj, _ = cv2.projectPoints(obj, rvec, tvec, intrinsics.K, dist)
    err = float(np.sqrt(((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2).sum(axis=1)).mean())
    return (T, err)

def estimate_pose(detector, detection, intrinsics, tag_size, method='auto'):
    if method == 'pnp':
        T, err = pose_by_pnp(detection, intrinsics, tag_size)
        return (T, err, err)
    R = getattr(detection, 'pose_R', None)
    tvec = getattr(detection, 'pose_t', None)
    if R is None or tvec is None:
        if method == 'tag':
            raise ValueError('AT3 자세가 없다 — detect(intrinsics=..., tag_size=...) 로 부를 것')
        T, err = pose_by_pnp(detection, intrinsics, tag_size)
        return (T, err, err)
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = np.asarray(tvec, dtype=float).ravel()
    err = float(getattr(detection, 'pose_err', 0.0) or 0.0)
    if method == 'auto' and (not np.isfinite(T).all()):
        T, e = pose_by_pnp(detection, intrinsics, tag_size)
        return (T, e, e)
    return (T, err, err)

def pose_to_xyzrpy(T, unit='deg'):
    p, R = t2pr(T)
    rpy = r2rpy(R, unit=unit)
    return {'x': p[0], 'y': p[1], 'z': p[2], 'roll': rpy[0], 'pitch': rpy[1], 'yaw': rpy[2], 'distance': float(np.linalg.norm(p))}

def pose_to_forklift(T_camera_tag, unit='deg'):
    T_tag_cam = invert_T(np.asarray(T_camera_tag))
    p = T_tag_cam[:3, 3]
    R = T_tag_cam[:3, :3]
    fwd = R @ np.array([0.0, 0.0, 1.0])
    right = R @ np.array([1.0, 0.0, 0.0])
    yaw = np.arctan2(fwd[0], fwd[2])
    pitch = np.arcsin(np.clip(-fwd[1], -1.0, 1.0))
    roll = np.arctan2(-right[1], np.hypot(right[0], right[2]))
    if unit == 'deg':
        roll, pitch, yaw = (np.degrees(v) for v in (roll, pitch, yaw))
        yaw -= D.CAM_YAW_OFFSET_DEG
    else:
        yaw -= np.radians(D.CAM_YAW_OFFSET_DEG)
    return {'lateral': float(p[0]), 'vertical': float(p[1]), 'forward': float(-p[2]), 'roll': float(roll), 'pitch': float(pitch), 'yaw': float(yaw), 'distance': float(np.linalg.norm(p))}

def docking_state(T_camera_tag, intrinsics=None, tag_size=None):
    T_tag_cam = invert_T(np.asarray(T_camera_tag))
    p = T_tag_cam[:3, 3]
    R = T_tag_cam[:3, :3]
    lateral = float(p[0])
    vertical = float(p[1])
    forward = float(-p[2])
    distance = float(np.linalg.norm(p))
    approach = float(np.degrees(np.arctan2(abs(lateral), abs(forward))))
    fwd = R @ np.array([0.0, 0.0, 1.0])
    heading = float(np.degrees(np.arctan2(fwd[0], fwd[2])) - D.CAM_YAW_OFFSET_DEG)
    tilt = tag_tilt_deg(T_camera_tag)
    sigma = float('nan')
    reliable = tilt >= D.RELIABLE_TILT_DEG
    if intrinsics is not None and tag_size:
        sigma = heading_sigma_deg(T_camera_tag, intrinsics, tag_size)
        reliable = sigma <= D.MAX_HEADING_SIGMA_DEG
    return {'lateral': lateral, 'vertical': vertical, 'forward': forward, 'distance': distance, 'approach_deg': approach, 'heading_deg': heading, 'tilt_deg': tilt, 'heading_sigma_deg': sigma, 'reliable_angle': bool(reliable)}

def heading_sigma_deg(T_camera_tag, intrinsics, tag_size, corner_px=None, n=None, seed=0):
    corner_px = D.CORNER_NOISE_PX if corner_px is None else float(corner_px)
    n = int(D.SIGMA_SAMPLES if n is None else n)
    T = np.asarray(T_camera_tag, dtype=np.float64)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        return float('inf')
    obj = _object_points(tag_size)
    K = np.asarray(intrinsics.K, dtype=np.float64)
    dist = np.asarray(intrinsics.distortion, dtype=np.float64) if getattr(intrinsics, 'distortion', None) else np.zeros(5)
    R, t = (T[:3, :3], T[:3, 3])
    cam = R @ obj.T + t[:, None]
    if (cam[2] <= 1e-06).any():
        return float('inf')
    px = K @ cam
    px = (px[:2] / px[2]).T
    h0 = docking_state(T)['heading_deg']
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        ok, rv, tv = cv2.solvePnP(obj, (px + rng.normal(0.0, corner_px, px.shape)).reshape(-1, 1, 2), K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        M = np.eye(4)
        M[:3, :3] = cv2.Rodrigues(rv)[0]
        M[:3, 3] = tv.ravel()
        out.append(docking_state(M)['heading_deg'] - h0)
    if len(out) < 3:
        return float('inf')
    return float(np.std(out))

def tag_tilt_deg(T_camera_tag):
    n = np.asarray(T_camera_tag)[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(min(1.0, abs(n[2])))))

def _sample_depth_m(detection, depth, patch, depth_scale):
    if depth is None:
        return None
    u, v = np.asarray(detection.corners, dtype=np.float64).mean(axis=0)
    return depth_at(depth, u, v, patch=patch, scale=depth_scale)

def depth_cross_check(detection, depth, T_camera_tag, patch=5, depth_scale=None):
    z_pose = float(np.asarray(T_camera_tag)[2, 3])
    tol = max(D.DEPTH_TOL_FLOOR_M, D.DEPTH_TOL_COEF * z_pose * z_pose)
    out = {'z_pose': z_pose, 'z_depth': None, 'diff_m': None, 'diff_pct': None, 'tol_m': float(tol), 'in_range': bool(0.0 < z_pose <= D.DEPTH_CHECK_MAX_Z), 'agree': False}
    z_depth = _sample_depth_m(detection, depth, patch, depth_scale)
    if z_depth is None or not np.isfinite(z_pose) or z_pose <= 0:
        return out
    diff = z_depth - z_pose
    out['z_depth'] = float(z_depth)
    out['diff_m'] = float(diff)
    out['diff_pct'] = float(100.0 * diff / z_pose)
    out['agree'] = bool(abs(diff) <= tol)
    return out

def pose_quality(detector, detection, intrinsics, tag_size, T_camera_tag, method='auto'):
    T = np.asarray(T_camera_tag, dtype=np.float64)
    if method == 'pnp':
        _, err = pose_by_pnp(detection, intrinsics, tag_size)
        e1, squared = (err, False)
    else:
        pe = getattr(detection, 'pose_err', None)
        if pe is None:
            _, err = pose_by_pnp(detection, intrinsics, tag_size)
            e1, squared = (err, False)
        else:
            e1, squared = (float(pe), False)
    e1 = float(e1)
    rms = float(np.sqrt(e1)) if squared and e1 >= 0 else e1
    tag_px = tag_pixel_size(detection)
    tilt = tag_tilt_deg(T)
    margin = float(getattr(detection, 'decision_margin', 0.0))
    hamming = int(getattr(detection, 'hamming', 0))
    reasons = []
    if not np.isfinite(T).all():
        reasons.append('pose_nan')
    if tag_px < D.STABLE_TAG_PX:
        reasons.append(f'tag_px<{D.STABLE_TAG_PX:g}')
    if not np.isfinite(rms) or rms > D.MAX_REPROJ_RMS_PX:
        reasons.append(f'reproj>{D.MAX_REPROJ_RMS_PX:g}px')
    if margin < D.MIN_DECISION_MARGIN:
        reasons.append(f'margin<{D.MIN_DECISION_MARGIN:g}')
    if hamming != 0:
        reasons.append('hamming!=0')
    return {'reproj_err': e1, 'reproj_units': 'px^2' if squared else 'px', 'reproj_rms_px': rms, 'tag_px': tag_px, 'tilt_deg': tilt, 'reliable_angle': bool(tilt >= D.RELIABLE_TILT_DEG), 'decision_margin': margin, 'hamming': hamming, 'ok': not reasons, 'reasons': reasons}

@dataclass
class Result:
    index: int
    timestamp: float
    image: object
    detections: list = field(default_factory=list)
    poses: dict = field(default_factory=dict)
    docking: dict = field(default_factory=dict)
    quality: dict = field(default_factory=dict)
    intrinsics: object = None
    errors: dict = field(default_factory=dict)

    @property
    def tag_ids(self):
        return [int(d.tag_id) for d in self.detections]

    def __len__(self):
        return len(self.detections)

    def primary(self, tag_id=None):
        usable = [d for d in self.detections if int(d.tag_id) in self.poses]
        if not usable:
            return None
        if tag_id is not None:
            usable = [d for d in usable if int(d.tag_id) == int(tag_id)]
            if not usable:
                return None
            det = usable[0]
        else:
            det = max(usable, key=tag_pixel_size)
        tid = int(det.tag_id)
        return {'tag_id': tid, 'detection': det, 'T': self.poses[tid], 'docking': self.docking.get(tid), 'quality': self.quality.get(tid)}
_PIPE_KEYS = ('detector', 'families', 'quad_blur', 'method', 'min_margin', 'max_hamming', 'gray_channel', 'hfov', 'quality', 'depth_check', 'label', 'origin')

class TagPipeline:

    def __init__(self, frames=None, intrinsics=None, tag_size=None, detector=None, families='tag36h11', quad_blur=D.DEFAULT_QUAD_BLUR, method='auto', min_margin=0.0, max_hamming=0, gray_channel=None, hfov=None, quality=True, depth_check=True, label='', origin='', close=None):
        if tag_size is None:
            raise ValueError('tag_size 를 반드시 줘야 한다 [m]. 거리가 이 값에 정비례하므로 기본값을 두면 틀린 거리가 조용히 나온다. 검은 테두리 바깥까지 잰 한 변.')
        self.frames = frames
        self.intr = intrinsics
        self.tag_size = float(tag_size)
        self.detector = detector if detector is not None else make_detector(families=families, quad_blur=quad_blur)
        self.method = method
        self.min_margin = float(min_margin)
        self.max_hamming = int(max_hamming)
        self.gray_channel = gray_channel
        self.hfov = None if hfov is None else float(hfov)
        self.quality = bool(quality)
        self.depth_check = bool(depth_check)
        self.label = label
        self.origin = origin or ('given' if intrinsics is not None else '')
        self.intrinsics_assumed = False
        self._close = close
        self._n = 0

    @staticmethod
    def _split_kw(kw):
        pipe = {k: kw.pop(k) for k in list(kw) if k in _PIPE_KEYS}
        return (pipe, kw)

    @classmethod
    def from_realsense(cls, tag_size, stream='color', **kw):
        pipe_kw, open_kw = cls._split_kw(kw)
        frames, intr = open_realsense(stream=stream, **open_kw)
        pipe_kw.setdefault('label', 'realsense/%s' % stream)
        pipe_kw.setdefault('origin', 'factory (RealSense %s)' % stream)
        return cls(frames, intrinsics=intr, tag_size=tag_size, close=frames.close, **pipe_kw)

    @classmethod
    def from_bag(cls, path, tag_size, **kw):
        pipe_kw, open_kw = cls._split_kw(kw)
        frames, intr = open_bag(str(path), **open_kw)
        pipe_kw.setdefault('label', 'bag (%s)' % Path(path).name)
        pipe_kw.setdefault('origin', 'bag stream profile')
        return cls(frames, intrinsics=intr, tag_size=tag_size, close=frames.close, **pipe_kw)

    @classmethod
    def from_video(cls, path, tag_size, intrinsics=None, hfov=None, loop=False, **kw):
        pipe_kw, _rest = cls._split_kw(kw)
        if _rest:
            raise TypeError('모르는 인자: %s' % ', '.join(sorted(_rest)))
        frames, _none = from_video(path, loop=loop)
        pipe_kw.setdefault('label', 'video (%s)' % Path(path).name)
        pipe_kw.setdefault('origin', 'given' if intrinsics is not None else '')
        return cls(frames, intrinsics=intrinsics, tag_size=tag_size, hfov=hfov, close=frames.close, **pipe_kw)

    @classmethod
    def from_webcam(cls, index, tag_size, width=None, height=None, fps=30, intrinsics=None, hfov=None, **kw):
        pipe_kw, _rest = cls._split_kw(kw)
        if _rest:
            raise TypeError('모르는 인자: %s' % ', '.join(sorted(_rest)))
        frames, _none = open_webcam(index, width=width, height=height, fps=fps)
        pipe_kw.setdefault('label', 'webcam #%s' % index)
        pipe_kw.setdefault('origin', 'given' if intrinsics is not None else '')
        return cls(frames, intrinsics=intrinsics, tag_size=tag_size, hfov=hfov, close=frames.close, **pipe_kw)

    def intrinsics_for(self, shape):
        if self.intr is None:
            self.intr = intrinsics_from_hfov(shape, self.hfov)
            self.intrinsics_assumed = True
            self.origin = 'ASSUMED from D435i ref' if self.hfov is None else 'ASSUMED from hfov=%.0fdeg' % self.hfov
        return self.intr

    def process(self, img, index=None, timestamp=None):
        src = img
        gray = to_gray(img, channel=self.gray_channel)
        intr = self.intrinsics_for(np.asarray(img).shape)
        if index is None:
            index = self._n
        self._n = index + 1
        res = Result(index=int(index), timestamp=float(timestamp) if timestamp is not None else 0.0, image=img, intrinsics=intr)
        try:
            res.detections = detect(self.detector, gray, intrinsics=intr, tag_size=self.tag_size, min_margin=self.min_margin, max_hamming=self.max_hamming)
        except Exception as exc:
            res.errors['detect'] = '%s: %s' % (type(exc).__name__, exc)
            return res
        depth = getattr(src, 'depth', None)
        depth_scale = getattr(src, 'depth_scale', 0.0) or None
        for d in res.detections:
            tid = int(d.tag_id)
            try:
                T, _e0, _e1 = estimate_pose(self.detector, d, intr, self.tag_size, method=self.method)
            except Exception as exc:
                res.errors[tid] = 'estimate_pose %s: %s' % (type(exc).__name__, exc)
                continue
            T = np.asarray(T, dtype=np.float64)
            if T.shape != (4, 4) or not np.isfinite(T).all():
                res.errors[tid] = 'non-finite pose (NaN)'
                continue
            res.poses[tid] = T
            res.docking[tid] = docking_state(T, intr, self.tag_size)
            if self.quality:
                q = pose_quality(self.detector, d, intr, self.tag_size, T, method=self.method)
                if self.depth_check and depth is not None:
                    q['depth'] = depth_cross_check(d, src, T, depth_scale=depth_scale)
                res.quality[tid] = q
        return res

    def __iter__(self):
        if self.frames is None:
            raise RuntimeError('소스 없이 만든 파이프라인이다 — process(img) 로 한 장씩 넣어라.')
        for i, ts, img in self.frames:
            yield self.process(img, index=i, timestamp=ts)

    @property
    def stats(self):
        return getattr(self.frames, 'stats', None)

    @property
    def ae_roi(self):
        return getattr(self.frames, 'ae_roi', None)

    def close(self):
        c, self._close = (self._close, None)
        if c is not None:
            try:
                c()
            except Exception:
                pass
        self.detector = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

def measure(results, tag_id=None, n=None, max_frames=None, require_ok=True):
    n = int(n or D.MEASURE_FRAMES)
    max_frames = int(max_frames or D.MEASURE_MAX_FRAMES)
    keys = ('lateral', 'vertical', 'forward', 'distance', 'approach_deg', 'heading_deg', 'tilt_deg', 'heading_sigma_deg')
    got = {k: [] for k in keys}
    got['z_optical'] = []
    seen = 0
    for res in results:
        seen += 1
        tid = tag_id
        if tid is None:
            tid = next(iter(res.docking), None)
        if tid is None or tid not in res.docking:
            if seen >= max_frames:
                break
            continue
        if require_ok and (not res.quality.get(tid, {}).get('ok', True)):
            if seen >= max_frames:
                break
            continue
        d = res.docking[tid]
        for k in keys:
            got[k].append(float(d[k]))
        got['z_optical'].append(float(res.poses[tid][2, 3]))
        if len(got['lateral']) >= n or seen >= max_frames:
            break
    m = len(got['lateral'])
    if m == 0:
        return None

    def med(v):
        s = sorted(v)
        return s[m // 2] if m % 2 else 0.5 * (s[m // 2 - 1] + s[m // 2])

    def stderr(v):
        if m < 2:
            return float('inf')
        mu = sum(v) / m
        var = sum(((x - mu) ** 2 for x in v)) / (m - 1)
        return var ** 0.5 / m ** 0.5
    out = {k: med(v) for k, v in got.items()}
    out['n'] = m
    out['spread'] = {k: stderr(v) for k, v in got.items()}
    sig = out.get('heading_sigma_deg', float('nan'))
    out['reliable_angle'] = bool(sig <= D.MAX_HEADING_SIGMA_DEG) if np.isfinite(sig) else bool(out['tilt_deg'] >= D.RELIABLE_TILT_DEG)
    reasons = []
    if m < n:
        reasons.append('프레임 부족 %d/%d' % (m, n))
    pred_h = out.get('heading_sigma_deg')
    pred_h = pred_h / np.sqrt(m) if pred_h and np.isfinite(pred_h) else None
    obs_l, obs_h = (out['spread']['lateral'], out['spread']['heading_deg'])
    if obs_l > D.STABLE_LATERAL_M:
        reasons.append('lateral 흔들림 %.1fmm' % (obs_l * 1000))
    if obs_h > D.STABLE_HEADING_DEG:
        reasons.append('heading 흔들림 %.2f도' % obs_h)
    if pred_h and obs_h > max(D.STABLE_SPREAD_K * pred_h, D.STABLE_HEADING_DEG / 3.0):
        reasons.append('예측보다 %.1f배 흔들림 (%.3f -> %.3f도)' % (obs_h / pred_h, pred_h, obs_h))
    out['stable'] = not reasons
    out['reasons'] = reasons
    return out
