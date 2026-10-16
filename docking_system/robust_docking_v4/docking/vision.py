"""Sparse RGB-D/IR-D odometry and ambiguity-gated AprilTag anchors.

No learned model. Feature residuals are quality checks, NOT error certificates.
OpenCV/NumPy/AprilTag are imported only in the camera process.
"""
import math
import time
import copy
from .motion import velocity_bound
from .core import finite
from .estimation import drift_budget, fuse_tag
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


class WindowMotion:
    """One extra sparse match per processed image, against staggered keyframes.

    Direct reference/current matching avoids accumulating frame-pair error.
    There are at most 16 fixed-size references; rejected pairs supply no stop.
    """
    def __init__(self, sensor):
        self.sensor, self.c = sensor, sensor['stop_window']
        self.references, self.last = [], None

    def update(self, gray, depth, K, dist, stamp, Tbc, healthy):
        c, b = self.c, self.sensor['bounds']
        result = dict(bounded=False, speed_upper=None, visual_yaw_upper=None,
                      acceleration_bound=b.get('max_acceleration_mps2'),
                      angular_acceleration_bound=c.get('max_angular_acceleration_rps2'), reason='collecting_stop_window')
        if not healthy or (self.last is not None and not 0 < stamp-self.last <= c['max_frame_gap_s']):
            self.references = []
        self.last = stamp
        self.references = [(t, ref) for t, ref in self.references if stamp-t <= c['max_s']]
        eligible = [(t, ref) for t, ref in self.references if stamp-t >= c['min_s']]
        a, e = b.get('max_acceleration_mps2'), c.get('translation_error_m')
        if healthy and Tbc is not None and eligible and a is not None and e is not None:
            # Choose from timing/error bounds, never select the smallest noisy measurement.
            t, ref = min(eligible, key=lambda tr: velocity_bound(0., e, stamp-tr[0], a))
            pair = copy.copy(ref)  # update() reseeds only the disposable copy
            od = pair.update(gray, depth, K, dist, stamp)
            if od['ok']:
                db = Tbc.dot(od['T_previous_current']).dot(inverse(Tbc))
                dt = stamp-t
                result.update(speed_upper=velocity_bound(float(np.linalg.norm(db[:2, 3])), e, dt, a),
                              baseline_s=dt, displacement_m=float(np.linalg.norm(db[:2, 3])),
                              reason='direct_window_motion')
                ye, ya = c.get('yaw_error_rad'), c.get('max_angular_acceleration_rps2')
                if ye is not None and ya is not None:
                    result['visual_yaw_upper'] = velocity_bound(abs(pose2(db)[2]), ye, dt, ya)
                result['bounded'] = bool(c.get('verified') and b.get('verified') and
                                         self.sensor['calibration'].get('verified'))
            else:
                result['reason'] = 'stop_window_tracking_failed'
        if healthy and (not self.references or stamp-self.references[-1][0] >= c['reference_period_s']):
            cfg = dict(self.sensor['odom'], max_pair_gap_s=c['max_s'], hold_keyframe_s=0.)
            ref = SparseOdom(cfg)
            ref.seed(gray, depth, K, dist, stamp)
            self.references.append((stamp, ref))
        return result


class Observer:
    """Explicit anchor, accumulated drift, and independent IMU consistency.

    This is sparse RGB-D odometry + gyro checking, NOT a full VINS/MSCKF.
    Feature loss can propagate a qualified coarse box; only a tag reanchors it.
    """
    def __init__(self, cfg):
        self.cfg = cfg
        cv2.setNumThreads(cfg['opencv_threads'])
        self.tag = Tag(cfg['tag']) if cfg['tag']['enabled'] else None
        track = cfg.get('tracking_recovery', {})
        odcfg = dict(cfg['odom'])
        if track.get('enabled'):
            odcfg['hold_keyframe_s'] = track['hold_keyframe_s']
            odcfg['max_pair_gap_s'] = max(odcfg['max_pair_gap_s'], track['hold_keyframe_s'])
        self.odom = SparseOdom(odcfg) if cfg['odom']['enabled'] else None
        self.stop_window = WindowMotion(cfg) if cfg.get('stop_window', {}).get('enabled') and self.odom else None
        self.last_result = self.last_stamp = None
        self.episode_started = None
        self.coarse = False
        self.keyframe_imu = None
        self.anchor = None
        self.anchor_error = None
        self.anchor_stamp = None
        self.travel = self.angle = 0.
        self.steps = 0
        self.last_tag = -float('inf')
        self.generation = None
        self.last_pose = None
        self.gyro_heading = self.gyro_error = None
        self.local_travel = self.local_angle = self.local_steps = 0
        self.local_started = None
        self.fusion_report = {'reason': 'awaiting_calibrated_tag_pose'}

    def process(self, gray, depth, K, dist, stamp, imu_delta=None, image_quality=None, capture_imu=None):
        c = self.cfg
        timing_start = time.monotonic()
        previous_result = self.last_result
        frame_dt = stamp-self.last_stamp if self.last_stamp is not None else None
        self.last_stamp = stamp
        # Match gyro endpoints to the actual visual keyframe, not to the last
        # processed frame. Held LK references can span several failed frames.
        current_imu = None
        if (capture_imu is not None and
                finite([capture_imu.get('yaw_rad'), capture_imu.get('device_stamp_s'), capture_imu.get('gaps')]) and
                capture_imu['device_stamp_s'] == stamp):
            current_imu = dict(capture_imu)
        reference_imu = self.keyframe_imu
        discarded_reference = reseeded_by_tag = False
        od = self.odom.update(gray, depth, K, dist, stamp) if self.odom else {'ok': False, 'reason': 'disabled', 'generation': 0, 'T_local_camera': np.eye(4)}
        odom_done = time.monotonic()
        tag = {'ok': False, 'reason': 'not_scheduled', 'corners': None}
        if self.tag and stamp-self.last_tag+1e-9 >= c['tag']['period_s']:
            tag = self.tag.update(gray, K, dist, stamp)
            self.last_tag = stamp
        if image_quality and image_quality.get('reject_motion_blur'):
            tag['ok'], tag['reason'] = False, 'motion_blur_limit'
            od['ok'], od['reason'] = False, 'motion_blur_limit'
            if self.odom:
                # Never bridge an absolute chain through rejected imagery.
                self.odom.prev = None
                self.odom.local = np.eye(4)
                self.odom.generation += 1
                od['generation'] = self.odom.generation
                od['T_local_camera'] = self.odom.local.copy()
                discarded_reference = True
        chain_changed = od['generation'] != self.generation
        failed = not od['ok']
        if chain_changed or failed:
            self.anchor, self.anchor_error = None, None
            self.anchor_stamp = None
            self.gyro_heading = self.gyro_error = None
            if chain_changed:
                self.local_travel = self.local_angle = self.local_steps = 0
                self.local_started = stamp
        self.generation = od['generation']
        self.travel += od.get('translation_m', 0.)
        self.angle += od.get('rotation_rad', 0.)
        self.steps += 1
        ext = c['calibration']
        Tbc = np.asarray(ext['T_body_camera'], float) if ext.get('T_body_camera') is not None else None
        Tdt = np.asarray(ext['T_dock_tag'], float) if ext.get('T_dock_tag') is not None else None
        imu_consistent = None
        pair_gyro = None
        if (current_imu is not None and reference_imu is not None and
                reference_imu['generation'] == od['generation'] and
                reference_imu['device_stamp_s'] == od.get('reference_stamp') and
                reference_imu['device_stamp_s'] < stamp and
                reference_imu['gaps'] == current_imu['gaps']):
            pair_gyro = current_imu['yaw_rad']-reference_imu['yaw_rad']
        if od['ok'] and Tbc is not None and pair_gyro is not None:
            delta_body = Tbc.dot(od['T_previous_current']).dot(inverse(Tbc))
            dyaw = pose2(delta_body)[2]
            imu_consistent = abs(unwrap(dyaw, pair_gyro)-pair_gyro) <= c['odom']['gyro_consistency_rad']
            if not imu_consistent:
                self.anchor, self.anchor_error = None, None
                self.anchor_stamp = None
                od['reason'], od['ok'] = 'vision_gyro_disagree', False
                if self.odom:
                    self.odom.prev = None
                    self.odom.local = np.eye(4)
                    self.odom.generation += 1
                    od['generation'] = self.odom.generation
                    od['T_local_camera'] = self.odom.local.copy()
                    self.generation = od['generation']
                    discarded_reference = True
                self.local_travel = self.local_angle = self.local_steps = 0
                self.local_started = stamp
        bounds = c['bounds']
        # This local frame has no dock position. Preserve its drift across tag
        # updates so pre-mission rotation can monitor body displacement even
        # before the first anchor. A new generation must never bridge a gap.
        local_motion = {'bounded': False, 'xy': None, 'error_xy': None}
        if od['ok'] and Tbc is not None:
            self.local_travel += od.get('translation_m', 0.)
            self.local_angle += od.get('rotation_rad', 0.)
            self.local_steps += 1
            local_pose = pose2(Tbc.dot(od['T_local_camera']).dot(inverse(Tbc)))
            local_motion['xy'] = local_pose[:2]
            local_elapsed = max(0., stamp-self.local_started) if self.local_started is not None else 0.
            local_error, _ = drift_budget(bounds, self.local_travel, self.local_angle,
                                          self.local_steps, local_elapsed, c['camera']['fps'])
            if local_error is not None:
                lever = float(np.linalg.norm(Tbc[:3, 3]))
                extra = (self.local_travel+lever)*min(2., local_error[2])
                local_motion.update(error_xy=[local_error[0]+extra, local_error[1]+extra],
                                    bounded=bool(bounds.get('verified') and ext.get('verified')))
        if self.gyro_heading is not None:
            if imu_delta is None:
                self.gyro_heading = self.gyro_error = None
            else:
                self.gyro_heading += imu_delta
                if self.gyro_error is not None and bounds['gyro_rate_error_rps'] is not None:
                    self.gyro_error += bounds['gyro_rate_error_rps']*(frame_dt or 0.)
                else:
                    self.gyro_error = None
        recovery = c.get('tracking_recovery', {})
        prediction = None
        can_bridge = (recovery.get('enabled') and recovery.get('verified') and bounds.get('verified') and
                      ext.get('verified') and previous_result and previous_result.get('bounded') and
                      Tbc is not None and frame_dt is not None and frame_dt > 0 and imu_delta is not None and
                      od.get('reason') not in ('vision_gyro_disagree', 'gyro_rotation_disagree', 'motion_blur_limit'))
        if chain_changed or not od['ok']:
            if can_bridge:
                if self.episode_started is None:
                    self.episode_started = stamp-frame_dt
                if stamp-self.episode_started <= recovery['max_episode_s']:
                    pp, pe = previous_result['pose'][:], previous_result['error'][:]
                    pp[2] += imu_delta
                    growth = recovery['max_unobserved_speed_mps']*frame_dt
                    pe[0] += growth
                    pe[1] += growth
                    pe[2] += bounds['gyro_rate_error_rps']*frame_dt
                    prediction = (pp, pe)
                    self.gyro_heading, self.gyro_error = pp[2], pe[2]
                    if chain_changed:
                        # New origin is tied to the ENTIRE propagated box, not to a precise point.
                        Tb = np.eye(4)
                        co, si = math.cos(pp[2]), math.sin(pp[2])
                        Tb[:2, :2] = [[co, -si], [si, co]]
                        Tb[:2, 3] = pp[:2]
                        self.anchor = Tb.dot(Tbc).dot(inverse(od['T_local_camera']))
                        self.anchor_error, self.anchor_stamp = pe, previous_result['anchor_stamp']
                        self.travel = self.angle = 0.
                        self.steps = 0
                        self.coarse = True
                    else:
                        # A retained keyframe may still recover the original precise chain.
                        self.anchor = getattr(self, '_saved_anchor', None)
                        self.anchor_error = getattr(self, '_saved_anchor_error', None)
                        self.anchor_stamp = previous_result['anchor_stamp']
            if prediction is None:
                self.anchor = self.anchor_error = self.anchor_stamp = None
                self.gyro_heading = self.gyro_error = None
        if self.coarse and self.episode_started is not None and stamp-self.episode_started > recovery['max_episode_s']:
            self.anchor = self.anchor_error = self.anchor_stamp = None
            prediction = None
        self.fusion_report = {'reason': 'no_qualified_tag_update'}
        if tag['ok'] and (Tbc is None or Tdt is None):
            self.fusion_report['reason'] = 'awaiting_calibrated_tag_pose'
        if tag['ok'] and Tbc is not None and Tdt is not None:
            absolute = Tdt.dot(inverse(tag['T_camera_tag']))
            candidate = pose2(absolute.dot(inverse(Tbc)))
            candidate_error = bounds.get('tag_pose_error')
            prior = prior_error = None
            if self.anchor is not None:
                prior, prior_error = prediction if prediction is not None else self.current(od['T_local_camera'], Tbc)
                # The prediction uses the same exposure-aligned gyro heading
                # that is published, not a second independent visual yaw.
                if self.gyro_heading is not None:
                    prior[2] = unwrap(self.gyro_heading, prior[2])
                    if prior_error is not None:
                        prior_error = prior_error[:] if self.gyro_error is not None else None
                        if prior_error is not None:
                            prior_error[2] = self.gyro_error
            consistent = True
            if prior is not None and prior_error is not None and candidate_error is not None:
                candidate[2] = unwrap(candidate[2], prior[2])
                consistent = all(abs(a-b) <= ea+eb for a, b, ea, eb in zip(candidate, prior, candidate_error, prior_error))
            if consistent:
                filtered, corrected_error, self.fusion_report = fuse_tag(
                    prior, prior_error, candidate, candidate_error, tag, c)
                # Left-multiply an SE(2) correction so x/y/yaw change without
                # discarding the raw tag's height, roll or pitch geometry.
                dh = filtered[2]-candidate[2]
                correction = np.eye(4)
                co, si = math.cos(dh), math.sin(dh)
                correction[:2, :2] = [[co, -si], [si, co]]
                correction[:2, 3] = np.asarray(filtered[:2])-correction[:2, :2].dot(candidate[:2])
                absolute = correction.dot(absolute)
                if od.get('held_reference') and self.odom is not None:
                    # The tag fixes THIS exposure. Discard the old keyframe so
                    # a later long-baseline match cannot count pre-anchor motion twice.
                    self.odom.local = np.eye(4)
                    self.odom.generation += 1
                    self.odom.seed(gray, depth, K, dist, stamp)
                    od['generation'] = self.generation = self.odom.generation
                    od['T_local_camera'] = self.odom.local.copy()
                    self.local_travel = self.local_angle = self.local_steps = 0
                    self.local_started = stamp
                    reseeded_by_tag = True
                self.anchor = absolute.dot(inverse(od['T_local_camera']))
                self.anchor_stamp = stamp
                self.anchor_error = corrected_error
                self.gyro_heading = filtered[2]
                self.gyro_error = None if corrected_error is None else corrected_error[2]
                self.travel = self.angle = 0.
                self.steps = 0
                prediction = None
                self.coarse, self.episode_started = False, None
            else:
                tag['ok'], tag['reason'] = False, 'anchor_conflicts_with_prior_bound'
                self.fusion_report = {'reason': 'tag_conflicts_with_prior_bound',
                                      'raw_tag_pose': candidate, 'prior_pose': prior}
        pose = error = None
        if self.anchor is not None and Tbc is not None:
            pose, error = self.current(od['T_local_camera'], Tbc)
            if self.last_pose is not None:
                pose[2] = unwrap(pose[2], self.last_pose[2])
            self.last_pose = pose
            if self.gyro_heading is not None:
                pose[2] = unwrap(self.gyro_heading, pose[2])
                if error is not None:
                    if self.gyro_error is None:
                        error = None
                    else:
                        error[2] = self.gyro_error
        if prediction is not None:
            pose, error = prediction
        elif od['ok'] and not self.coarse:
            self.episode_started = None
        self._saved_anchor = self.anchor
        self._saved_anchor_error = self.anchor_error
        if discarded_reference:
            self.keyframe_imu = None
        elif self.odom and (reseeded_by_tag or not od.get('held_reference')):
            # SparseOdom seeds this exposure after success or a new chain.
            # A missing IMU endpoint stays missing; never reuse an older one.
            self.keyframe_imu = (dict(current_imu, generation=od['generation'])
                                 if current_imu is not None else None)
        speed_upper = yaw_upper = visual_yaw_upper = None
        delta = od.get('T_previous_current')
        if od['ok'] and delta is not None and Tbc is not None and bounds.get('relative_translation_error_m') is not None:
            db = Tbc.dot(delta).dot(inverse(Tbc))
            dt = od['dt']
            # Average displacement alone cannot prove current velocity zero.
            # Add bounded acceleration over the sampling interval.
            if bounds.get('max_acceleration_mps2') is not None:
                speed_upper = (float(np.linalg.norm(db[:2, 3]))+bounds['relative_translation_error_m'])/dt + bounds['max_acceleration_mps2']*dt/2
            visual_error = c['imu'].get('stationary_recalibration', {}).get('relative_yaw_error_rad')
            if visual_error is not None:
                visual_yaw_upper = (abs(pose2(db)[2])+visual_error)/dt
        result = {'valid': pose is not None, 'bounded': pose is not None and error is not None and ext.get('verified', False),
                  'motion_bounded': bool(speed_upper is not None and od['ok'] and imu_consistent and
                                         bounds.get('verified') and ext.get('verified')),
                  'pose': pose, 'error': error, 'speed_upper': speed_upper, 'yaw_rate_upper': yaw_upper,
                  'visual_yaw_rate_upper': visual_yaw_upper, 'anchor_stamp': self.anchor_stamp,
                  'tracking_generation': self.generation,
                  'local_motion': local_motion,
                  'camera_odometry': {
                      'valid': bool(od['ok']), 'generation': od['generation'],
                      'frame': 'first_camera_in_tracking_generation',
                      'position_m': od['T_local_camera'][:3, 3].tolist() if od['ok'] else None,
                      'rotation_vector_rad': cv2.Rodrigues(od['T_local_camera'][:3, :3])[0].ravel().tolist() if od['ok'] else None,
                      'vehicle_pose': False, 'certified_error_bound': False},
                  'tag_fusion': self.fusion_report,
                  'error_budget': self.error_budget(Tbc),
                  'imu_consistent': imu_consistent, 'tag': {k: v for k, v in tag.items() if not k.startswith('T_')},
                  'odom': {k: v for k, v in od.items() if not k.startswith('T_')},
                  'distance_since_anchor_m': self.travel, 'steps_since_anchor': self.steps}
        if self.stop_window:
            result['stop_motion'] = self.stop_window.update(gray, depth, K, dist, stamp, Tbc,
                                                           bool(od['ok'] and imu_consistent))
        gap = stamp-self.episode_started if self.episode_started is not None else 0.
        result['tracking_recovery'] = dict(coarse=self.coarse, predicting=prediction is not None,
            episode_s=gap, bridge_ok=bool(prediction is not None and gap <= recovery.get('max_bridge_s', 0.)),
            bounded=bool(result['bounded'] and recovery.get('verified')))
        if tag.get('corners') and Tbc is not None:
            centre = np.mean(tag['corners'], axis=0)
            ray = cv2.undistortPoints(np.array([[centre]], float), K, dist)[0, 0]
            ray_body = Tbc[:3, :3].dot([ray[0], ray[1], 1.])
            result['tag']['bearing_body_rad'] = math.atan2(ray_body[1], ray_body[0])
        self.last_result = result
        result['stage_ms'] = {'odom': (odom_done-timing_start)*1000,
                              'tag_and_fusion': (time.monotonic()-odom_done)*1000}
        return result

    def current(self, local, Tbc):
        pose = pose2(self.anchor.dot(local).dot(inverse(Tbc)))
        b = self.cfg['bounds']
        elapsed = max(0., self.last_stamp-self.anchor_stamp) if self.anchor_stamp is not None else 0.
        drift, _ = drift_budget(b, self.travel, self.angle, self.steps, elapsed, self.cfg['camera']['fps'])
        if self.anchor_error is None or drift is None:
            return pose, None
        error = [e+d for e, d in zip(self.anchor_error, drift)]
        # Account for heading uncertainty acting on the traversed distance and
        # the body-camera lever arm; no covariance reset at commanded stops.
        lever = float(np.linalg.norm(Tbc[:3, 3]))
        extra = (self.travel+lever)*min(2., error[2])
        error[0] += extra
        error[1] += extra
        return pose, error

    def error_budget(self, Tbc):
        elapsed = max(0., self.last_stamp-self.anchor_stamp) if self.anchor_stamp is not None else None
        _, terms = drift_budget(self.cfg['bounds'], self.travel, self.angle, self.steps,
                                elapsed or 0., self.cfg['camera']['fps'])
        return {'anchor_error': self.anchor_error, 'seconds_since_anchor': elapsed,
                'budgeted_frame_intervals': max(self.steps, (elapsed or 0.)*self.cfg['camera']['fps']),
                'visual_drift_terms': terms, 'gyro_heading_error_rad': self.gyro_error,
                'camera_lever_arm_m': None if Tbc is None else float(np.linalg.norm(Tbc[:3, 3])),
                'includes_smoothing_offset': bool(self.cfg.get('tag_fusion', {}).get('enabled')),
                'statistical_covariance': False}
