"""Colour AprilTag/IPPE, capture-time geometry and covariance propagation.

v4 (2026-11-01): besides the ENTRANCE tag (site entrance_tag_id, beside the entrance, lost at the tag cut) the detector reads the two
INNER tags on the lattice bars (site inner_tag_ids, [left, right] as seen from the forklift). Every tag is solved on its own
(IPPE_SQUARE, as before). When both inner tags are seen unclipped their eight corners are also solved together as ONE planar
target, the 'inner frame' (origin = midpoint of the two tag centres, orientation of an upright tag facing the entrance): the
1.2 m baseline makes that solution's heading strong where a single 0.2-0.3 m tag's heading is biased by degrees (the reason
fusion.py takes a single tag's PnP heading only as a weak reference). The top-level dict keeps the v3 entrance-tag contract;
the inner tags live under out['inner'] = dict(tags={id: per-tag dict}, bundle=dict(...), seen=[ids]).
"""

import numpy as np
import cv2
from .uncertainty import inverse, perturb, jacobian, covariance, transform as check_transform


def transform(r, t):
    out = np.eye(4)
    out[:3, :3] = cv2.Rodrigues(r)[0]
    out[:3, 3] = np.asarray(t).ravel()
    return out


def square_points(size):
    """Corners of one tag in its own frame, in the detector's native IPPE order."""
    s = size / 2
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)


def solve_planar(obj, corners, K, dist, flags, max_reproj_px, max_ambiguity_ratio):
    """IPPE on coplanar object points -> (ok, reason, T_camera_obj, reproj_px, ambiguity_ratio). Solutions with a point behind
    the camera are dropped; the best must reproject within max_reproj_px and be clearly better than the runner-up."""
    result = cv2.solvePnPGeneric(obj, corners, K, dist, flags=flags)
    solutions = []
    for rv, tv in zip(result[1], result[2]):
        T = transform(rv, tv)
        if np.min((obj.dot(T[:3, :3].T) + T[:3, 3])[:, 2]) <= 0:
            continue
        projected = cv2.projectPoints(obj, rv, tv, K, dist)[0].reshape(-1, 2)
        err = float(np.sqrt(np.mean(np.sum((projected - corners) ** 2, axis=1))))
        solutions.append((err, T))
    solutions.sort(key=lambda p: p[0])
    if not solutions:
        return False, 'pnp_failed', None, None, None
    err = solutions[0][0]
    ratio = max(err, 1e-12) / max(solutions[1][0], 1e-12) if len(solutions) > 1 else 0.0
    if err > max_reproj_px or ratio > max_ambiguity_ratio:
        return False, 'reprojection_or_planar_ambiguity', solutions[0][1], err, ratio
    return True, 'accepted', solutions[0][1], err, ratio


def rotation_angle(Ra, Rb):
    return float(np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1.0, 1.0))))


def solve_bundle(obj, img, K, dist):
    """All local minima of the reprojection error for the two-tag planar target, best first: [(reproj_px, T_camera_obj), ...].
    OpenCV's IPPE is inexact on two separated squares (0.3-80 px with exact corners, both 4.14 and 5.0, 2026-11-01), so the
    starts are SQPnP (globally optimal) and the two IPPE poses, each refined by Levenberg-Marquardt; duplicates are merged.
    A planar target always has a mirrored twin (the tilt sign); for a nearly frontal target the twin fits nearly as well, so the
    caller decides what to do with a second candidate (observer: the fused heading picks)."""
    cands = []
    for flags in (cv2.SOLVEPNP_SQPNP, cv2.SOLVEPNP_IPPE):
        try:
            result = cv2.solvePnPGeneric(obj, img, K, dist, flags=flags)
        except cv2.error:
            continue
        for rv, tv in zip(result[1], result[2]):
            ok, rv2, tv2 = cv2.solvePnP(
                obj,
                img,
                K,
                dist,
                np.array(rv, float).reshape(3, 1),
                np.array(tv, float).reshape(3, 1),
                useExtrinsicGuess=True,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not ok:
                continue
            T = transform(rv2, tv2)
            if not np.isfinite(T).all() or np.min((obj.dot(T[:3, :3].T) + T[:3, 3])[:, 2]) <= 0:
                continue
            projected = cv2.projectPoints(obj, rv2, tv2, K, dist)[0].reshape(-1, 2)
            err = float(np.sqrt(np.mean(np.sum((projected - img) ** 2, axis=1))))
            if any(
                rotation_angle(T[:3, :3], c[1][:3, :3]) < 0.3 and np.linalg.norm(T[:3, 3] - c[1][:3, 3]) < 0.005
                for c in cands
            ):
                continue
            cands.append((err, T))
    cands.sort(key=lambda c: c[0])
    return cands


class Detector:
    def __init__(self, cfg):
        from pupil_apriltags import Detector

        self.cfg = cfg
        self.detector = Detector(
            families='tag36h11',
            nthreads=cfg['threads'],
            quad_decimate=cfg['quad_decimate'],
            quad_sigma=cfg.get('quad_sigma', 0.0),
            refine_edges=1,
        )
        inner = cfg.get('inner') or {}
        self.inner_ids = [int(i) for i in (inner.get('ids') or [])]
        self.inner_size = float(inner.get('size_m') or cfg['size_m'])
        self.inner_offsets = {int(k): float(v) for k, v in (inner.get('offsets') or {}).items()}  # along the tag x axis
        self.entrance_id = int(cfg['id'])
        self.wanted = [self.entrance_id] + [i for i in self.inner_ids if i != self.entrance_id]
        self.rois, self.last_full = {}, -float('inf')

    def _roi(self, corners, w, h):
        low, high = corners.min(axis=0), corners.max(axis=0)
        pad = np.maximum(high - low, 40) * self.cfg['roi_padding']
        return (
            max(0, int(low[0] - pad[0])),
            max(0, int(low[1] - pad[1])),
            min(w, int(high[0] + pad[0])),
            min(h, int(high[1] + pad[1])),
        )

    def _square(self, r, size, K, dist, w, h, shift, allow_ambiguous=False):
        """One detection -> the per-tag dict (the v3 entrance-tag contract: ok/reason/corners/margin/id/detected, then the
        edge and size gates, then the IPPE_SQUARE solution with its reprojection and ambiguity gates).
        allow_ambiguous (inner tags): a tag seen head-on has two tilts that reproject alike; its bearing and range are still good,
        so the frame is accepted with ambiguous=True and the fusion uses only those (never its heading)."""
        corners = (
            np.asarray(r.corners, dtype=np.float64) + shift
        )  # detector's native order, restored to full-frame coordinates
        low, high = corners.min(axis=0), corners.max(axis=0)
        out = {
            'ok': False,
            'reason': 'clipped',
            'corners': corners.tolist(),
            'margin': float(r.decision_margin),
            'id': int(r.tag_id),
            'detected': True,
        }
        if min(low[0], low[1], w - 1 - high[0], h - 1 - high[1]) < self.cfg['edge_px']:
            return out
        edge_lengths = np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)
        out['min_edge_px'] = float(edge_lengths.min())
        if out['min_edge_px'] < self.cfg.get('min_tag_px', 20):
            out['reason'] = 'tag_too_small'
            return out
        ok, reason, T, err, ratio = solve_planar(
            square_points(size),
            corners,
            K,
            dist,
            cv2.SOLVEPNP_IPPE_SQUARE,
            self.cfg['max_reproj_px'],
            self.cfg['max_ambiguity_ratio'],
        )
        out['reason'] = reason
        if T is not None:
            out.update(
                reproj_px=err,
                ambiguity_ratio=ratio,
                camera_translation_m=T[:3, 3].tolist(),
                camera_rotation_vector_rad=cv2.Rodrigues(T[:3, :3])[0].ravel().tolist(),
            )
            if allow_ambiguous and not ok and err <= self.cfg['max_reproj_px']:
                ok = True
                out.update(reason='accepted', ambiguous=True)
        if ok:
            out.update(ok=True, T_camera_tag=T)
        return out

    def _bundle(self, tags, K, dist):
        """Both inner tags seen unclipped -> one planar PnP over their eight corners in the inner frame."""
        parts = [tags.get(i) or {} for i in self.inner_ids]
        if len(parts) != 2 or any(not p.get('detected') for p in parts):
            return dict(ok=False, reason='inner_tag_missing', n_tags=sum(1 for p in parts if p.get('detected')))
        bad = [p['reason'] for p in parts if p.get('reason') in ('clipped', 'tag_too_small')]
        if bad:
            return dict(ok=False, reason='inner_tag_' + bad[0], n_tags=2)
        obj = np.vstack(
            [square_points(self.inner_size) + [self.inner_offsets.get(i, 0.0), 0.0, 0.0] for i in self.inner_ids]
        )
        img = np.vstack([np.asarray(p['corners'], np.float64) for p in parts])
        cands = solve_bundle(obj, img, K, dist)
        if not cands:
            return dict(ok=False, reason='bundle_pnp_failed', n_tags=2, points=obj.tolist())
        err, T = cands[0]
        second = cands[1] if len(cands) > 1 else None
        ratio = max(err, 1e-12) / max(second[0], 1e-12) if second else 0.0
        # The twin counts as an ambiguity only when it fits nearly as well AND says something different about the heading.
        ambiguous = bool(
            second and ratio > self.cfg['max_ambiguity_ratio'] and rotation_angle(T[:3, :3], second[1][:3, :3]) > 1.0
        )
        ok = err <= float(self.cfg.get('bundle_max_reproj_px', 3.0))
        out = dict(
            ok=bool(ok),
            reason='accepted' if ok else 'bundle_reprojection_%.1fpx' % err,
            n_tags=2,
            reproj_px=err,
            ambiguity_ratio=ratio,
            ambiguous=ambiguous,
            points=obj.tolist(),
            camera_translation_m=T[:3, 3].tolist(),
            camera_rotation_vector_rad=cv2.Rodrigues(T[:3, :3])[0].ravel().tolist(),
            candidates=[dict(reproj_px=e, T_camera_inner=Tc) for e, Tc in cands[:2]],
        )
        if ok:
            out['T_camera_inner'] = T
        return out

    def update(self, gray, K, dist, stamp):
        c, h, w = self.cfg, *gray.shape
        # One crop around every tracked tag (the union of their boxes), a full scan every full_scan_period_s or as soon as a
        # wanted tag has no box (v3 did the same for its single tag).
        full = stamp - self.last_full >= c['full_scan_period_s'] or any(i not in self.rois for i in self.wanted)
        if full:
            x0, y0, x1, y1 = 0, 0, w, h
            self.last_full = stamp
        else:
            boxes = [self.rois[i] for i in self.wanted]
            x0, y0 = min(b[0] for b in boxes), min(b[1] for b in boxes)
            x1, y1 = max(b[2] for b in boxes), max(b[3] for b in boxes)
        results = self.detector.detect(np.ascontiguousarray(gray[y0:y1, x0:x1]))
        # Everything the detector saw, so a failed run can tell "no tag in view" from "wrong id / low margin"
        # (2026-10-19 hand-held run: 239 attempts, 0 detections, no way to tell which).
        seen = [
            dict(id=int(r.tag_id), margin=round(float(r.decision_margin), 1), hamming=int(r.hamming)) for r in results
        ][:8]
        by_id = {}
        for r in results:
            if int(r.tag_id) in self.wanted and r.decision_margin >= c['min_margin'] and r.hamming <= c['max_hamming']:
                by_id.setdefault(int(r.tag_id), []).append(r)
        shift = np.array([x0, y0], float)
        tags = {}
        for i in self.wanted:
            rs = by_id.get(i)
            if rs is not None and len(rs) == 1:
                tags[i] = self._square(
                    rs[0],
                    self.inner_size if i in self.inner_ids else c['size_m'],
                    K,
                    dist,
                    w,
                    h,
                    shift,
                    allow_ambiguous=i in self.inner_ids,
                )
                self.rois[i] = self._roi(np.asarray(tags[i]['corners']), w, h)
            else:
                tags[i] = {'ok': False, 'reason': 'missing_or_duplicate_id', 'corners': None, 'detected': False}
                self.rois.pop(i, None)
        out = dict(tags[self.entrance_id])
        if not out.get('detected'):
            others = [s for s in seen if s['id'] not in self.inner_ids]
            out['reason'] = 'seen_but_rejected' if others else 'missing_or_duplicate_id'
        out.update(seen=seen, image_mean=float(gray.mean()))
        if self.inner_ids:
            inner = {i: tags[i] for i in self.inner_ids}
            out['inner'] = dict(
                tags=inner,
                bundle=self._bundle(inner, K, dist),
                seen=[i for i in self.inner_ids if inner[i].get('detected')],
            )
        return out


def model_matrix(model):
    return np.array([[model['fx'], 0.0, model['cx']], [0.0, model['fy'], model['cy']], [0.0, 0.0, 1.0]])


def pnp_covariance(T_camera_tag, size, K, dist, pixel_std=0.7, intrinsic_covariance=None, points=None):
    """Local covariance conditional on the selected IPPE branch.

    Pixel std is a declared engineering noise floor, NOT inferred accuracy from
    repeated stationary frames. Ambiguity is rejected before this function.
    v4: points = the planar object points (default: one square tag of `size`; the inner bundle passes its eight corners).
    """
    T = check_transform(T_camera_tag)
    points = square_points(size) if points is None else np.asarray(points, float)

    def project(d, k=K):
        q = perturb(T, d)
        return cv2.projectPoints(points, cv2.Rodrigues(q[:3, :3])[0], q[:3, 3], k, dist)[0].ravel()

    J = jacobian(project, np.zeros(6))
    info = J.T @ J / pixel_std**2
    if not np.isfinite(info).all() or np.linalg.cond(info) > 1e12:
        raise ValueError('pnp_geometry_ill_conditioned')
    P = np.linalg.inv(info)
    if intrinsic_covariance is not None:
        n = len(intrinsic_covariance)
        if n not in (4, 4 + len(dist)):
            raise ValueError('intrinsic covariance must cover fx/fy/cx/cy, optionally all distortion coefficients')
        C = covariance(intrinsic_covariance, n)

        def project_intrinsic(v):
            k = K.copy()
            k[0, 0], k[1, 1], k[0, 2], k[1, 2] = v[:4]
            distortion = dist if n == 4 else v[4:]
            return cv2.projectPoints(points, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], k, distortion)[0].ravel()

        values = [K[0, 0], K[1, 1], K[0, 2], K[1, 2]]
        if n > 4:
            values += list(dist)
        Ji = jacobian(project_intrinsic, values)
        H = -np.linalg.solve(J.T @ J, J.T @ Ji)
        P += H @ C @ H.T
    return covariance(P, 6), float(np.linalg.cond(info))


def dock_measurement(T_camera_tag, Ptag, T_body_camera, Pmount, T_dock_tag, position_floor_m, yaw_floor_rad):
    Tct, Tbc, Tdt = map(check_transform, (T_camera_tag, T_body_camera, T_dock_tag))
    from .geometry import pose

    def fn(d):
        return pose(Tdt @ inverse(perturb(Tct, d[:6])) @ inverse(perturb(Tbc, d[6:])))

    J = jacobian(fn, np.zeros(12), (2,))
    # Tag-derived mounting measurements and online tags can be correlated.
    P = 2 * (J[:, :6] @ covariance(Ptag, 6) @ J[:, :6].T + J[:, 6:] @ covariance(Pmount, 6) @ J[:, 6:].T)
    P += np.diag([position_floor_m**2] * 2 + [yaw_floor_rad**2])
    return fn(np.zeros(12)), covariance(P, 3)


def depth_check(depth_m, corners, expected_z, cfg):
    """Aligned colour depth is a consistency gate, never a second pose update."""
    if depth_m is None:
        return dict(available=False, reason='depth_unavailable')
    mask = np.zeros(depth_m.shape, np.uint8)
    c = np.asarray(corners)
    center = c.mean(axis=0)
    cv2.fillConvexPoly(mask, np.round(center + 0.6 * (c - center)).astype(np.int32), 1)
    selected = depth_m[mask.astype(bool)]
    valid = selected[np.isfinite(selected) & (selected > cfg['depth_min_m']) & (selected < cfg['depth_max_m'])]
    fraction = len(valid) / max(1, len(selected))
    if fraction < cfg['depth_min_fraction'] or len(valid) < 12:
        return dict(available=False, reason='depth_insufficient', valid_fraction=fraction)
    z = float(np.median(valid))
    spread = float(np.percentile(valid, 90) - np.percentile(valid, 10))
    if spread > cfg['depth_spread_m']:
        return dict(available=False, reason='depth_mixed_surface', spread_m=spread)
    return dict(
        available=True,
        reason='depth_consistent' if abs(z - expected_z) <= cfg['depth_gate_m'] else 'depth_disagrees',
        consistent=abs(z - expected_z) <= cfg['depth_gate_m'],
        z_m=z,
        residual_m=z - expected_z,
        valid_fraction=fraction,
    )


class TagObserver:
    def __init__(self, cfg):
        self.cfg = cfg
        self.detector = Detector(cfg['tag'])

    def process(self, color, model, stamp, epoch, depth_m=None):
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY) if color.ndim == 3 else color
        K = model_matrix(model)
        dist = np.asarray(model['coeffs'], float)
        out = self.detector.update(gray, K, dist, stamp)
        out.update(stamp=float(stamp), sensor_epoch=int(epoch), stream='color')
        # Seeing an unusably small tag must not stop the initial search.
        out['search_candidate'] = bool(out.get('detected') and out['reason'] not in ('tag_too_small', 'clipped'))
        u = self.cfg['uncertainty']
        if out['ok']:
            T = out.pop('T_camera_tag')
            try:
                C, condition = pnp_covariance(
                    T,
                    self.cfg['tag']['size_m'],
                    K,
                    dist,
                    max(u['corner_std_px'], out['reproj_px']),
                    model.get('intrinsic_covariance'),
                )
                out.update(T_camera_tag=T.tolist(), covariance=C.tolist(), condition=condition)
                check = depth_check(depth_m, out['corners'], T[2, 3], u)
                out['depth_check'] = check
                if check.get('consistent') is False:
                    out.update(ok=False, reason='depth_disagrees')
            except ValueError as exc:
                out.update(ok=False, reason=str(exc))
        inner = out.get('inner')
        if inner:
            # v4: matrices become lists (the record and the queues are JSON); the bundle gets its covariance over the eight corners
            for t in inner['tags'].values():
                T = t.pop('T_camera_tag', None)
                if T is not None:
                    t['T_camera_tag'] = np.asarray(T).tolist()
            b = inner.get('bundle') or {}
            T = b.pop('T_camera_inner', None)
            for cand in b.get('candidates') or []:
                cand['T_camera_inner'] = np.asarray(cand['T_camera_inner']).tolist()
            if T is not None:
                try:
                    C, condition = pnp_covariance(
                        T,
                        None,
                        K,
                        dist,
                        max(u['corner_std_px'], float(b.get('reproj_px') or 0.0)),
                        model.get('intrinsic_covariance'),
                        points=np.asarray(b['points']),
                    )
                    b.update(T_camera_inner=np.asarray(T).tolist(), covariance=C.tolist(), condition=condition)
                except ValueError as exc:
                    b.update(ok=False, reason=str(exc))
            inner['tags'] = {str(k): v for k, v in inner['tags'].items()}
        return out
