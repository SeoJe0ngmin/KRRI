"""Dock frame: x into bay, y left, yaw CCW. Body origin at camera station."""

import math
import numpy as np


def finite(v):
    try:
        return all(math.isfinite(float(x)) for x in v)
    except (TypeError, ValueError):
        return False


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def matrix(p):
    x, y, h = p
    c, s = math.cos(h), math.sin(h)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def pose(T):
    return np.array([T[0, 3], T[1, 3], math.atan2(T[1, 0], T[0, 0])])


def moved(p, turn, distance, pivot):
    # Nominal geometry for candidate planning ONLY. Never used as observation.
    q = np.array(p, float).copy()
    c, s = math.cos(q[2]), math.sin(q[2])
    center = q[:2] + np.array([[c, -s], [s, c]]) @ np.array(pivot)
    q[2] = wrap(q[2] + turn)
    c, s = math.cos(q[2]), math.sin(q[2])
    q[:2] = center - np.array([[c, -s], [s, c]]) @ np.array(pivot)
    q[:2] += distance * np.array([c, s])
    return q


def front(p, cfg):
    f = cfg['site']['camera_to_fork_tip_m']
    return np.array(p[:2]) + f * np.array([math.cos(p[2]), math.sin(p[2])])


def remaining(p, cfg):
    return cfg['target_front_x'] - front(p, cfg)[0]


def pair_width(site):
    """Inner width of the entrance pair the depth camera measures: the doors' inner vertical lines, which is also the collision
    width (docking 2026-10-30: one width, truck_inner_width_m; the v3 depth_pair_width_m split for the X frames is gone).
    """
    v = site.get('truck_inner_width_m')
    return None if v is None else float(v)


def path_bias(cfg):
    """Measured drift of a straight drive, radians, left +. apriltag/KWU FORWARD_PATH_BIAS_DEG (2026-10-21): the vehicle
    does not track its heading exactly (2026-10-20 blind legs: 1-2 deg to the right, every run). Empty/0 = no compensation.
    """
    v = (cfg.get('site') or {}).get('forward_path_bias_deg')
    return math.radians(float(v)) if v is not None and math.isfinite(float(v)) else 0.0


def arrival_y(p, cfg):
    h = p[2] + path_bias(cfg)  # direction the vehicle actually travels when driven straight
    c = math.cos(h)
    if c <= 0.2:
        return math.inf
    # Forward projection to target plane, including the fork lever arm.
    return p[1] + (remaining(p, cfg) / c + cfg['site']['camera_to_fork_tip_m']) * math.sin(h)


def inside_truck(p, cfg):
    """Only the truck bed's side walls are constrained; the outside approach is operator-cleared."""
    w, bw = cfg['site']['vehicle_width_m'], cfg['site']['truck_inner_width_m']
    if w is None or bw is None:
        return False
    T = matrix(p)
    f = cfg['site']['camera_to_fork_tip_m']
    r = cfg['site']['rear_extent_m']
    corners = [(T @ [x, y, 0, 1])[:2] for x, y in [(f, w / 2), (f, -w / 2), (-r, -w / 2), (-r, w / 2)]]
    clipped = []
    for a, b in zip(corners, corners[1:] + corners[:1]):
        if a[0] >= 0:
            clipped.append(a)
        if (a[0] < 0) != (b[0] < 0):
            clipped.append(a + (b - a) * (-a[0] / (b[0] - a[0])))
    return all(abs(q[1]) <= bw / 2 for q in clipped)


def path_clear(p, turn, distance, cfg):
    # Sample nominal sweep, not a claim of a certified swept error envelope.
    n = max(1, int(abs(turn) / math.radians(2)) + 1)
    for i in range(n + 1):
        if not inside_truck(moved(p, turn * i / n, 0, cfg['pivot']), cfg):
            return False
    q = moved(p, turn, 0, cfg['pivot'])
    n = max(1, int(abs(distance) / 0.025) + 1)
    return all(inside_truck(moved(q, 0, distance * i / n, cfg['pivot']), cfg) for i in range(n + 1))


def tag_margin(p, cfg):
    """Predicted tag edge margin; distortion is evaluated using the measured model."""
    model = cfg['sensor']['calibration'].get('camera_model')
    if not model:
        return -math.inf
    import cv2

    Tcb = np.linalg.inv(np.array(cfg['sensor']['calibration']['T_body_camera']))
    Tct = Tcb @ np.linalg.inv(matrix(p)) @ np.array(cfg['sensor']['calibration']['T_dock_tag'])
    a = cfg['site']['entrance_tag_size_m'] / 2
    corners = np.array([[-a, a, 0], [a, a, 0], [a, -a, 0], [-a, -a, 0]])
    xyz = corners @ Tct[:3, :3].T + Tct[:3, 3]
    if xyz[:, 2].min() <= 0.1:
        return -math.inf
    K = np.array([[model['fx'], 0, model['cx']], [0, model['fy'], model['cy']], [0, 0, 1.0]])
    uv = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), K, np.array(model['coeffs']))[0].reshape(-1, 2)
    return float(
        min(uv[:, 0].min(), uv[:, 1].min(), model['width'] - 1 - uv[:, 0].max(), model['height'] - 1 - uv[:, 1].max())
    )


def inner_margin(p, cfg):
    """실험판 D: smallest distance [px] from any corner of the two inner tags to the colour image border at pose p, with the
    side that is nearest ('left' | 'top' | 'right' | 'bottom'). (-inf, None) without a camera model / inner tags or behind the camera."""
    cal = cfg['sensor']['calibration']
    model, inner = cal.get('camera_model'), cal.get('inner')
    if not model or not inner:
        return -math.inf, None
    import cv2

    Tcb = np.linalg.inv(np.array(cal['T_body_camera']))
    K = np.array([[model['fx'], 0, model['cx']], [0, model['fy'], model['cy']], [0, 0, 1.0]])
    a = float(inner['size_m']) / 2
    corners = np.array([[-a, a, 0], [a, a, 0], [a, -a, 0], [-a, -a, 0]])
    worst = (math.inf, None)
    for T in inner['tags'].values():
        Tct = Tcb @ np.linalg.inv(matrix(p)) @ np.array(T)
        xyz = corners @ Tct[:3, :3].T + Tct[:3, 3]
        if xyz[:, 2].min() <= 0.1:
            return -math.inf, None
        uv = cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), K, np.array(model['coeffs']))[0].reshape(-1, 2)
        m = dict(
            left=uv[:, 0].min(),
            top=uv[:, 1].min(),
            right=model['width'] - 1 - uv[:, 0].max(),
            bottom=model['height'] - 1 - uv[:, 1].max(),
        )
        side = min(m, key=m.get)
        if m[side] < worst[0]:
            worst = (float(m[side]), side)
    return worst


def inner_view(cfg):
    """실험판 D: on the lane centreline, how far in both inner tags stay whole in the colour image (the detector drops a tag
    within tag.edge_px of the border). None without a camera model / inner tags, else dict(last_camera_x_m (None = never whole),
    cut_by, plane_m, check_margin_px (at the final check, None when its distance is not set), target_camera_x_m)."""
    cal = cfg['sensor']['calibration']
    if not cal.get('camera_model') or not cal.get('inner'):
        return None
    edge = float(cfg['sensor']['tag'].get('edge_px', 15))
    tip = cfg['site']['camera_to_fork_tip_m']
    plane = float(cal['inner']['plane_x_m'])
    last, cut_by = None, None
    for x in np.arange(-8.0, plane - 0.1, 0.025):
        m, side = inner_margin([float(x), 0, 0], cfg)
        if m >= edge:
            last, cut_by = float(x), None
        elif last is not None and cut_by is None:
            cut_by = side
    check = cfg['tuning'].get('final_check_distance_m')
    return dict(
        last_camera_x_m=last,
        cut_by=cut_by,
        plane_m=plane,
        check_margin_px=None if check is None else inner_margin([-float(check) - tip, 0, 0], cfg)[0],
        target_camera_x_m=float(cfg['target_front_x'] - tip),
    )


def cut_position(cfg):
    target = cfg['target_front_x'] - cfg['site']['camera_to_fork_tip_m']
    xs = np.arange(-15.0, min(target, -0.02) + 0.01, 0.025)
    good = [x for x in xs if tag_margin([x, 0, 0], cfg) >= cfg['tuning']['cut_margin_px']]
    if not good:
        raise ValueError('중심선에서 태그 가시 구간을 찾지 못했습니다. 태그 위치/장착각을 확인하세요.')
    return min(max(good), target)
