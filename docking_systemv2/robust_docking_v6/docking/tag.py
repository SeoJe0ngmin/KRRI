"""Colour AprilTag/IPPE, capture-time geometry and covariance propagation."""
import math
import numpy as np
import cv2
from .uncertainty import inverse, perturb, jacobian, covariance, transform as check_transform, se3_planar

def transform(r, t):
    out=np.eye(4);out[:3,:3]=cv2.Rodrigues(r)[0];out[:3,3]=np.asarray(t).ravel();return out

class Detector:
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


def model_matrix(model):
    return np.array([[model['fx'], 0., model['cx']], [0., model['fy'], model['cy']], [0., 0., 1.]])


def pnp_covariance(T_camera_tag, size, K, dist, pixel_std=.7, intrinsic_covariance=None):
    """Local covariance conditional on the selected IPPE branch.

    Pixel std is a declared engineering noise floor, NOT inferred accuracy from
    repeated stationary frames. Ambiguity is rejected before this function.
    """
    T = check_transform(T_camera_tag); a = size/2
    points = np.array([[-a,a,0],[a,a,0],[a,-a,0],[-a,-a,0]])
    def project(d, k=K):
        q = perturb(T, d)
        return cv2.projectPoints(points, cv2.Rodrigues(q[:3,:3])[0], q[:3,3], k, dist)[0].ravel()
    J = jacobian(project, np.zeros(6))
    info = J.T @ J / pixel_std**2
    if not np.isfinite(info).all() or np.linalg.cond(info) > 1e12:
        raise ValueError('pnp_geometry_ill_conditioned')
    P = np.linalg.inv(info)
    if intrinsic_covariance is not None:
        n=len(intrinsic_covariance)
        if n not in (4,4+len(dist)):raise ValueError('intrinsic covariance must cover fx/fy/cx/cy, optionally all distortion coefficients')
        C = covariance(intrinsic_covariance,n)
        def project_intrinsic(v):
            k = K.copy(); k[0,0], k[1,1], k[0,2], k[1,2] = v[:4]
            distortion=dist if n==4 else v[4:]
            return cv2.projectPoints(points,cv2.Rodrigues(T[:3,:3])[0],T[:3,3],k,distortion)[0].ravel()
        values=[K[0,0],K[1,1],K[0,2],K[1,2]]
        if n>4:values+=list(dist)
        Ji = jacobian(project_intrinsic,values)
        H = -np.linalg.solve(J.T @ J, J.T @ Ji)
        P += H @ C @ H.T
    return covariance(P, 6), float(np.linalg.cond(info))


def dock_measurement(T_camera_tag, Ptag, T_body_camera, Pmount, T_dock_tag,
                     position_floor_m, yaw_floor_rad):
    Tct, Tbc, Tdt = map(check_transform, (T_camera_tag, T_body_camera, T_dock_tag))
    from .geometry import pose
    def fn(d):
        return pose(Tdt @ inverse(perturb(Tct,d[:6])) @ inverse(perturb(Tbc,d[6:])))
    J = jacobian(fn, np.zeros(12), (2,))
    # Tag-derived mounting measurements and online tags can be correlated.
    P = 2*(J[:,:6] @ covariance(Ptag,6) @ J[:,:6].T + J[:,6:] @ covariance(Pmount,6) @ J[:,6:].T)
    P += np.diag([position_floor_m**2]*2+[yaw_floor_rad**2])
    return fn(np.zeros(12)), covariance(P,3)


def depth_check(depth_m, corners, expected_z, cfg):
    """Aligned colour depth is a consistency gate, never a second pose update."""
    if depth_m is None:
        return dict(available=False, reason='depth_unavailable')
    mask = np.zeros(depth_m.shape, np.uint8)
    c = np.asarray(corners); center=c.mean(axis=0)
    cv2.fillConvexPoly(mask, np.round(center+.6*(c-center)).astype(np.int32), 1)
    selected = depth_m[mask.astype(bool)]
    valid = selected[np.isfinite(selected) & (selected>cfg['depth_min_m']) & (selected<cfg['depth_max_m'])]
    fraction = len(valid)/max(1,len(selected))
    if fraction < cfg['depth_min_fraction'] or len(valid)<12:
        return dict(available=False, reason='depth_insufficient', valid_fraction=fraction)
    z=float(np.median(valid)); spread=float(np.percentile(valid,90)-np.percentile(valid,10))
    if spread>cfg['depth_spread_m']:
        return dict(available=False, reason='depth_mixed_surface', spread_m=spread)
    return dict(available=True, reason='depth_consistent' if abs(z-expected_z)<=cfg['depth_gate_m'] else 'depth_disagrees',
                consistent=abs(z-expected_z)<=cfg['depth_gate_m'], z_m=z, residual_m=z-expected_z, valid_fraction=fraction)


class TagObserver:
    def __init__(self, cfg):
        self.cfg=cfg
        self.detector=Detector(cfg['tag'])

    def process(self, color, model, stamp, epoch, depth_m=None):
        gray=cv2.cvtColor(color,cv2.COLOR_BGR2GRAY) if color.ndim==3 else color
        K=model_matrix(model); dist=np.asarray(model['coeffs'],float)
        out=self.detector.update(gray,K,dist,stamp)
        out.update(stamp=float(stamp),sensor_epoch=int(epoch),stream='color')
        # Seeing an unusably small tag must not stop the initial search.
        out['search_candidate']=bool(out.get('detected') and out['reason'] not in ('tag_too_small','clipped'))
        if out['ok']:
            T=out.pop('T_camera_tag')
            try:
                u=self.cfg['uncertainty']
                C,condition=pnp_covariance(T,self.cfg['tag']['size_m'],K,dist,
                    max(u['corner_std_px'],out['reproj_px']),model.get('intrinsic_covariance'))
                out.update(T_camera_tag=T.tolist(),covariance=C.tolist(),condition=condition)
                check=depth_check(depth_m,out['corners'],T[2,3],u)
                out['depth_check']=check
                if check.get('consistent') is False:
                    out.update(ok=False,reason='depth_disagrees')
            except ValueError as exc:
                out.update(ok=False,reason=str(exc))
        return out

