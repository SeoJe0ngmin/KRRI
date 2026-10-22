"""Sparse RGB-D/IR-D odometry and ambiguity-gated AprilTag anchors.

No learned model. Feature residuals are quality checks, NOT error certificates.
OpenCV/NumPy/AprilTag are imported only in the camera process.
"""
import math
import time
import copy
from .geometry import finite
from .gyro_vision import predict_pixels
import numpy as np
import cv2


def inverse(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3].dot(T[:3, 3])
    return out


def transform(r, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(r)[0], t.ravel()
    return T


def pose2(T):
    return [float(T[0, 3]), float(T[1, 3]), math.atan2(T[1, 0], T[0, 0])]


def unwrap(value, reference):
    return reference + (value-reference+math.pi) % (2*math.pi)-math.pi


class Tag:
    def __init__(self, cfg):
        from pupil_apriltags import Detector
        self.cfg = cfg
        self.detector = Detector(families='tag36h11', nthreads=cfg['threads'],
                                 quad_decimate=cfg['quad_decimate'], quad_sigma=cfg.get('quad_sigma', 0.), refine_edges=1)
        self.roi, self.last_full = None, -float('inf')

    def update(self, gray, K, dist, stamp):
        c, h, w = self.cfg, *gray.shape
        roi = self.roi if stamp-self.last_full < c['full_scan_period_s'] else None
        if roi is None:
            x0, y0, x1, y1 = 0, 0, w, h
            self.last_full = stamp
        else:
            x0, y0, x1, y1 = roi
        results = self.detector.detect(np.ascontiguousarray(gray[y0:y1, x0:x1]))
        matches = [r for r in results if r.tag_id == c['id'] and
                   r.decision_margin >= c['min_margin'] and r.hamming <= c['max_hamming']]
        if len(matches) != 1:
            self.roi = None
            return {'ok': False, 'reason': 'missing_or_duplicate_id', 'corners': None}
        r = matches[0]
        # Detector's native IPPE order is retained; restore crop coordinates.
        corners = np.asarray(r.corners, dtype=np.float64) + [x0, y0]
        low, high = corners.min(axis=0), corners.max(axis=0)
        pad = np.maximum(high-low, 40)*c['roi_padding']
        self.roi = (max(0, int(low[0]-pad[0])), max(0, int(low[1]-pad[1])),
                    min(w, int(high[0]+pad[0])), min(h, int(high[1]+pad[1])))
        out = {'ok': False, 'reason': 'clipped', 'corners': corners.tolist(),
               'margin': float(r.decision_margin), 'id': int(r.tag_id), 'detected': True}
        if min(low[0], low[1], w-1-high[0], h-1-high[1]) < c['edge_px']:
            return out
        edge_lengths = np.linalg.norm(corners-np.roll(corners, 1, axis=0), axis=1)
        out['min_edge_px'] = float(edge_lengths.min())
        if out['min_edge_px'] < c.get('min_tag_px', 20):
            out['reason'] = 'tag_too_small'
            return out
        s = c['size_m']/2
        obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)
        result = cv2.solvePnPGeneric(obj, corners, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        solutions = []
        for rv, tv in zip(result[1], result[2]):
            T = transform(rv, tv)
            if np.min((obj.dot(T[:3, :3].T)+T[:3, 3])[:, 2]) <= 0:
                continue
            projected = cv2.projectPoints(obj, rv, tv, K, dist)[0].reshape(-1, 2)
            err = float(np.sqrt(np.mean(np.sum((projected-corners)**2, axis=1))))
            solutions.append((err, T))
        solutions.sort(key=lambda p: p[0])
        if not solutions:
            out['reason'] = 'pnp_failed'
            return out
        err = solutions[0][0]
        ratio = max(err, 1e-12)/max(solutions[1][0], 1e-12) if len(solutions) > 1 else 0.
        out.update(reproj_px=err, ambiguity_ratio=ratio)
        out['camera_translation_m'] = solutions[0][1][:3, 3].tolist()
        out['camera_rotation_vector_rad'] = cv2.Rodrigues(solutions[0][1][:3, :3])[0].ravel().tolist()
        if err > c['max_reproj_px'] or ratio > c['max_ambiguity_ratio']:
            out['reason'] = 'reprojection_or_planar_ambiguity'
            return out
        out.update(ok=True, reason='accepted', T_camera_tag=solutions[0][1])
        return out


class SparseOdom:
    def __init__(self, cfg):
        self.cfg = cfg
        self.prev = self.points = self.xyz = self.last_stamp = None
        self.local = np.eye(4)
        self.generation = 0
        self.hold_s = cfg.get('hold_keyframe_s', 0.)
        self.rotation_predictor = None

    def seed(self, gray, depth, K, dist, stamp):
        c = self.cfg
        mask = np.full(gray.shape, 255, np.uint8)
        h, w = gray.shape
        for r in c['exclude_normalized_rects']:
            mask[int(r[1]*h):int(r[3]*h), int(r[0]*w):int(r[2]*w)] = 0
        pts = cv2.goodFeaturesToTrack(gray, c['max_features'], .01, c['feature_spacing_px'], mask=mask)
        good, xyz = np.empty((0, 1, 2), np.float32), np.empty((0, 3), float)
        if pts is not None and depth is not None:
            xy = np.round(pts[:, 0]).astype(int)
            inside = (xy[:, 0] >= 2) & (xy[:, 0] < w-2) & (xy[:, 1] >= 2) & (xy[:, 1] < h-2)
            pts, xy = pts[inside], xy[inside]
            dx, dy = np.meshgrid([-1, 0, 1], [-1, 0, 1])
            patches = depth[xy[:, 1, None]+dy.ravel(), xy[:, 0, None]+dx.ravel()]
            valid = (patches > c['min_depth_m']) & (patches < c['max_depth_m'])
            low = np.min(np.where(valid, patches, np.inf), axis=1)
            high = np.max(np.where(valid, patches, -np.inf), axis=1)
            keep = (np.sum(valid, axis=1) >= 6) & (high-low <= c['depth_patch_spread_m'])
            good = pts[keep]
            if len(good):
                z = np.nanmedian(np.where(valid[keep], patches[keep], np.nan), axis=1)
                uv = cv2.undistortPoints(good.astype(np.float64), K, dist).reshape(-1, 2)
                xyz = np.column_stack((uv*z[:, None], z))
        self.points, self.xyz = good, xyz
        self.prev, self.last_stamp = gray.copy(), stamp

    def update(self, gray, depth, K, dist, stamp):
        c = self.cfg
        result = {'ok': False, 'reason': 'initializing', 'inliers': 0,
                  'generation': self.generation, 'dt': None,
                  'reference_stamp': self.last_stamp,
                  'gyro_aid': {'used': False, 'reason': 'not_available'}}
        if self.prev is not None:
            dt = stamp-self.last_stamp
            result['dt'] = dt
            if not 0 < dt <= c['max_pair_gap_s']:
                result['reason'] = 'frame_gap'
            elif len(self.points) < c['min_inliers']:
                result['reason'] = 'too_few_depth_features'
            else:
                lk = dict(winSize=(21, 21), maxLevel=3,
                          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, .01))
                rotation, guess = None, None
                prior_valid = np.ones(len(self.points), bool)
                if c.get('gyro_aided_tracking') and self.rotation_predictor is not None:
                    rotation, info = self.rotation_predictor(self.last_stamp, stamp)
                    result['gyro_aid'] = dict(info, used=rotation is not None)
                    if rotation is not None:
                        guess, prior_valid = predict_pixels(self.points, K, dist, rotation)
                forward = dict(lk)
                if guess is not None:
                    forward['flags'] = cv2.OPTFLOW_USE_INITIAL_FLOW
                nxt, status, _ = cv2.calcOpticalFlowPyrLK(self.prev, gray, self.points, guess, **forward)
                if nxt is None or status is None or not np.all(np.isfinite(nxt)):
                    return self.lost_and_reseed(result, gray, depth, K, dist, stamp)
                back, back_status, _ = cv2.calcOpticalFlowPyrLK(gray, self.prev, nxt, None, **lk)
                if back is None or back_status is None or not np.all(np.isfinite(back)):
                    return self.lost_and_reseed(result, gray, depth, K, dist, stamp)
                good = (status.ravel() == 1) & (back_status.ravel() == 1) & prior_valid
                good &= np.linalg.norm(back[:, 0]-self.points[:, 0], axis=1) < c['fb_error_px']
                obj, img = self.xyz[good], nxt[good, 0]
                result['tracked'] = len(obj)
                if len(obj) >= c['min_inliers']:
                    ok, rv, tv, ids = cv2.solvePnPRansac(obj, img, K, dist,
                        iterationsCount=60, reprojectionError=c['max_reproj_px'],
                        confidence=.99, flags=cv2.SOLVEPNP_EPNP)
                    if ok and ids is not None:
                        ids = ids.ravel()
                        result['inliers'] = len(ids)
                        uv = img[ids]
                        bins = set((min(3, int(p[0]*4/gray.shape[1])),
                                    min(2, int(p[1]*3/gray.shape[0]))) for p in uv)
                        if len(ids) >= c['min_inliers'] and len(ids)/len(obj) >= c['min_inlier_ratio'] and len(bins) >= c['min_spatial_bins']:
                            ok, rv, tv = cv2.solvePnP(obj[ids], uv, K, dist, rv, tv, True,
                                                    flags=cv2.SOLVEPNP_ITERATIVE)
                            T = transform(rv, tv)
                            length = float(np.linalg.norm(tv))
                            angle = float(np.linalg.norm(rv))
                            if ok and length <= c['max_speed_mps']*dt and angle <= c['max_rotation_rps']*dt:
                                delta = inverse(T)
                                agreement = True
                                if rotation is not None:
                                    difference = delta[:3, :3].T.dot(rotation)
                                    residual = math.acos(float(np.clip((np.trace(difference)-1.)/2., -1., 1.)))
                                    result['gyro_aid']['visual_disagreement_rad'] = residual
                                    agreement = residual <= c['gyro_consistency_rad']
                                if agreement:
                                    self.local = self.local.dot(delta)
                                    result.update(ok=True, reason='tracked', T_previous_current=delta,
                                                  translation_m=length, rotation_rad=angle,
                                                  tracked_points=uv.tolist())
                                else:
                                    result['reason'] = 'gyro_rotation_disagree'
                if not result['ok'] and result['reason'] == 'initializing':
                    result['reason'] = 'insufficient_or_inconsistent_geometry'
            if not result['ok']:
                if self.hold_s and 0 < dt <= self.hold_s:
                    result.update(held_reference=True, generation=self.generation, T_local_camera=self.local.copy())
                    return result
                # A new relative chain is not an absolute relocalization.
                self.generation += 1
                self.local = np.eye(4)
        self.seed(gray, depth, K, dist, stamp)
        result['generation'] = self.generation
        result['T_local_camera'] = self.local.copy()
        return result

    def lost_and_reseed(self, result, gray, depth, K, dist, stamp):
        if self.hold_s and self.last_stamp is not None and 0 < stamp-self.last_stamp <= self.hold_s:
            result.update(reason='optical_flow_unavailable', held_reference=True, generation=self.generation,
                          T_local_camera=self.local.copy())
            return result
        self.generation += 1
        self.local = np.eye(4)
        self.seed(gray, depth, K, dist, stamp)
        result.update(reason='optical_flow_unavailable', generation=self.generation,
                      T_local_camera=self.local.copy())
        return result


