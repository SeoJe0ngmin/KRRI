"""Conservative tag field-of-view check for planar body pose intervals.

This checks geometry, not occlusion or future texture. The alternate odometry
regions require site evidence AND remain subject to runtime tracking checks.
"""
import math


def validate_visibility(cfg):
    v = cfg.get('visibility', {})
    if not v.get('enabled'):
        return
    from .configuration import _resolve_transform
    s = cfg['sensor']
    cal = s['calibration']
    for key in ('T_body_camera', 'T_dock_tag'):
        if cal.get(key) is None:
            raise ValueError('visibility requires '+key)
        _resolve_transform(cal, key, None, [None]*3)
    m = cal.get('camera_model')
    if not m or any(not isinstance(m.get(k), (int, float)) or not math.isfinite(m[k])
                    for k in ('width', 'height', 'fx', 'fy', 'cx', 'cy')):
        raise ValueError('visibility requires recorded camera_model')
    if (min(m[k] for k in ('width', 'height', 'fx', 'fy')) <= 0 or len(m.get('coeffs', [])) != 5 or
            not all(isinstance(x, (int, float)) and math.isfinite(x) for x in m['coeffs'])):
        raise ValueError('invalid camera_model')
    if m.get('stream') != s['camera']['stream'] or m['width'] != s['camera']['width'] or m['height'] != s['camera']['height']:
        raise ValueError('visibility camera model/stream mismatch')
    margin = v.get('projection_margin_px')
    if not isinstance(margin, (int, float)) or not math.isfinite(margin) or margin < 0:
        raise ValueError('visibility requires distortion/calibration projection_margin_px')
    for r in v.get('odometry_regions', []):
        if len(r) != 4 or not all(math.isfinite(x) for x in r) or r[0] >= r[1] or r[2] >= r[3]:
            raise ValueError('invalid verified odometry region')
    for r in v.get('odometry_pose_regions', []):
        if (len(r) != 6 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in r)
                or any(r[k] >= r[k+1] for k in (0, 2, 4))):
            raise ValueError('invalid verified odometry pose region')
    if (v.get('odometry_regions') or v.get('odometry_pose_regions')) and not v.get('odometry_evidence'):
        raise ValueError('odometry regions require recorded site evidence')


def tag_in_view(box, cfg):
    from .core import trig_bounds, product, interval_sum
    s = cfg['sensor']
    bc, dt = (s['calibration'][k] for k in ('T_body_camera', 'T_dock_tag'))
    model = s['calibration']['camera_model']
    cs = trig_bounds(box.lo[2], box.hi[2], True)
    sn = trig_bounds(box.lo[2], box.hi[2])
    # IPPE tag +z points out of the printed face. Every possible camera
    # centre must remain on that side, even if the back projects in-frame.
    camera_xy = [interval_sum(product((bc[0][3],)*2, cs), product((-bc[1][3],)*2, sn)),
                 interval_sum(product((bc[0][3],)*2, sn), product((bc[1][3],)*2, cs))]
    camera_world = [(box.lo[i]+camera_xy[i][0], box.hi[i]+camera_xy[i][1]) for i in range(2)]
    camera_world.append((bc[2][3],)*2)
    facing = (0., 0.)
    for i in range(3):
        offset = (camera_world[i][0]-dt[i][3], camera_world[i][1]-dt[i][3])
        facing = interval_sum(facing, product(offset, (dt[i][2],)*2))
    if facing[0] <= 0:
        return False
    uv = []
    half = s['tag']['size_m']/2
    for tx, ty in ((-half, half), (half, half), (half, -half), (-half, -half)):
        point = [dt[i][0]*tx+dt[i][1]*ty+dt[i][3] for i in range(3)]
        dx, dy = (point[0]-box.hi[0], point[0]-box.lo[0]), (point[1]-box.hi[1], point[1]-box.lo[1])
        xb = interval_sum(product(dx, cs), product(dy, sn))
        yb = interval_sum(product(dx, (-sn[1], -sn[0])), product(dy, cs))
        body = [(xb[0]-bc[0][3], xb[1]-bc[0][3]),
                (yb[0]-bc[1][3], yb[1]-bc[1][3]), (point[2]-bc[2][3],)*2]
        camera = []
        for j in range(3):
            val = (0., 0.)
            for i in range(3):
                val = interval_sum(val, product(body[i], (bc[i][j],)*2))
            camera.append(val)
        if camera[2][0] <= 0:
            return False
        corners = []
        for axis, focal, centre in ((0, model['fx'], model['cx']), (1, model['fy'], model['cy'])):
            values = [focal*x/z+centre for x in camera[axis] for z in camera[2]]
            corners.append((min(values), max(values)))
        uv.append(corners)
    edge = s['tag']['edge_px']+cfg['visibility']['projection_margin_px']
    if not all(edge <= u[0] and u[1] <= model['width']-1-edge and
               edge <= v[0] and v[1] <= model['height']-1-edge for u, v in uv):
        return False
    # The detector also needs a resolvable tag, not just one inside the frame.
    margin = cfg['visibility']['projection_margin_px']
    for previous, current in zip(uv, uv[1:]+uv[:1]):
        squared = 0.
        for a, b in zip(previous, current):
            low, high = a[0]-b[1]-2*margin, a[1]-b[0]+2*margin
            distance = 0. if low <= 0 <= high else min(abs(low), abs(high))
            squared += distance*distance
        if math.sqrt(squared) < s['tag'].get('min_tag_px', 20):
            return False
    return True


def observable(box, cfg):
    v = cfg.get('visibility', {})
    if not v.get('enabled'):
        return True
    if tag_in_view(box, cfg):
        return True
    return (any(r[0] <= box.lo[0] and box.hi[0] <= r[1] and
                r[2] <= box.lo[1] and box.hi[1] <= r[3] for r in v.get('odometry_regions', [])) or
            any(all(r[2*k] <= box.lo[k] and box.hi[k] <= r[2*k+1] for k in range(3))
                for r in v.get('odometry_pose_regions', [])))
