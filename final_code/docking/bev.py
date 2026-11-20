"""Bird's-eye view from the depth stream (2026-10-22, display/test only for now).

Frames: camera = OpenCV (x right, y down, z forward); body = vehicle (x forward, y left, z up), origin on the vehicle
centreline at the camera's longitudinal position, floor at z = 0 (so the camera sits at z = camera_height_m).
The depth stream is registered to the LEFT IR imager, so its intrinsics also project the IR image.
"""

import math
import numpy as np

R_CAMERA_FORWARD = np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
)  # body <- camera, camera looking straight ahead


def mount_rotation(pitch_deg=0.0, yaw_deg=0.0):
    """Fallback when configs/mount.json is absent: camera forward, pitched DOWN by pitch_deg (left-hand rule about body y)."""
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    Rp = np.array([[math.cos(p), 0.0, -math.sin(p)], [0.0, 1.0, 0.0], [math.sin(p), 0.0, math.cos(p)]])
    Ry = np.array([[math.cos(y), -math.sin(y), 0.0], [math.sin(y), math.cos(y), 0.0], [0.0, 0.0, 1.0]])
    return Ry @ Rp @ R_CAMERA_FORWARD


_RAYS = {}


def pixel_rays(shape, intr, step):
    """(u, v, (u - cx) / fx, (v - cy) / fy) of the sub-sampled pixel grid, built once per image shape / step / intrinsics
    (np.mgrid on every depth frame was a measurable share of the 10 Hz depth work)."""
    key = (shape, step, float(intr['fx']), float(intr['fy']), float(intr['cx']), float(intr['cy']))
    hit = _RAYS.get(key)
    if hit is None:
        v, u = np.mgrid[0 : shape[0], 0 : shape[1]] * step
        hit = _RAYS[key] = (u, v, (u - intr['cx']) / intr['fx'], (v - intr['cy']) / intr['fy'])
        if len(_RAYS) > 8:
            _RAYS.pop(next(iter(_RAYS)))
    return hit


def depth_to_points(depth_m, intr, step=2, max_m=8.0):
    """Pixels -> camera-frame points (N x 3) plus their (u, v). Holes (0/NaN) and far returns are dropped."""
    d = depth_m[::step, ::step]
    u, v, xf, yf = pixel_rays(d.shape, intr, step)
    ok = np.isfinite(d) & (d > 0) & (d <= max_m)
    z = d[ok]
    x = xf[ok] * z
    y = yf[ok] * z
    return np.column_stack([x, y, z]), u[ok], v[ok]


def camera_to_body(points_cam, R, t):
    return points_cam @ R.T + np.asarray(t, float)


def body_to_pixels(points_body, R, t, intr):
    """Inverse of camera_to_body + pinhole projection; returns (u, v, z_cam)."""
    cam = (np.asarray(points_body, float) - np.asarray(t, float)) @ R
    z = cam[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = intr['cx'] + intr['fx'] * cam[:, 0] / z
        v = intr['cy'] + intr['fy'] * cam[:, 1] / z
    return u, v, z


def occupancy(points_body, forward_m=8.0, half_width_m=3.0, cell_m=0.05, z_min=0.05, z_max=1.6):
    """Top-down hit-count grid of points between z_min and z_max above the floor. grid[row, col]: row 0 = far, col 0 = left."""
    p = points_body
    keep = (
        (p[:, 2] >= z_min)
        & (p[:, 2] <= z_max)
        & (p[:, 0] >= 0)
        & (p[:, 0] < forward_m)
        & (np.abs(p[:, 1]) < half_width_m)
    )
    p = p[keep]
    rows = int(round(forward_m / cell_m))
    cols = int(round(2 * half_width_m / cell_m))
    if not len(p):
        return np.zeros((rows, cols), np.int32)
    r = rows - 1 - (p[:, 0] / cell_m).astype(int)
    c = ((half_width_m - p[:, 1]) / cell_m).astype(int)
    flat = np.clip(r, 0, rows - 1) * cols + np.clip(
        c, 0, cols - 1
    )  # bincount: the same hit counts as np.add.at, several times faster
    return np.bincount(flat, minlength=rows * cols).reshape(rows, cols).astype(np.int32)


def bev_cells(acc, forward_m=8.0, half_width_m=3.0, cell_m=0.05, threshold=1.5, limit=2500):
    """[[x, y], ...] (body metres, 2 decimals) of the accumulated grid cells at or above threshold, far rows first - the dashboard's
    bird's-eye picture and the standing-still log. Vectorised (the per-cell Python loop cost ~1 ms per frame)."""
    rows, cols = np.nonzero(acc >= threshold)
    rows = rows[:limit]
    cols = cols[:limit]
    nrows = acc.shape[0]
    return np.column_stack(
        [np.round((nrows - 1 - rows + 0.5) * cell_m, 2), np.round(half_width_m - (cols + 0.5) * cell_m, 2)]
    ).tolist()


def grid_xy(row, col, forward_m=8.0, half_width_m=3.0, cell_m=0.05):
    """Centre of a grid cell in body metres."""
    rows = int(round(forward_m / cell_m))
    return (rows - 1 - row + 0.5) * cell_m, half_width_m - (col + 0.5) * cell_m


def xy_grid(x, y, forward_m=8.0, half_width_m=3.0, cell_m=0.05):
    rows = int(round(forward_m / cell_m))
    return rows - 1 - int(x / cell_m), int((half_width_m - y) / cell_m)


def ground_line(y_body, x0, x1, R, t, intr, n=24):
    """Pixels of a straight line on the floor at lateral y_body from x0 to x1 (body metres): the parking-camera guide line."""
    xs = np.linspace(x0, x1, n)
    pts = np.column_stack([xs, np.full(n, y_body), np.zeros(n)])
    u, v, z = body_to_pixels(pts, R, t, intr)
    ok = z > 0.1
    return np.column_stack([u[ok], v[ok]])


def entrance_frame(left_xy, right_xy):
    """Entrance frame from the two inner edges (body metres): origin = their midpoint, x = normal pointing INTO the truck
    (away from the vehicle), y = along the entrance towards the left edge. Returns (centre, n, tau)."""
    l = np.asarray(left_xy, float)
    r = np.asarray(right_xy, float)
    tau = l - r
    tau = tau / max(1e-9, np.linalg.norm(tau))
    n = np.array([tau[1], -tau[0]])
    if n[0] < 0:
        n = -n
        tau = -tau  # keep x pointing forward (away from the vehicle, which is at body x <= 0 side)
    return (l + r) / 2, n, tau


def to_entrance(points_xy, frame):
    c, n, tau = frame
    d = np.asarray(points_xy, float) - c
    return np.column_stack([d @ n, d @ tau])


def vehicle_in_entrance(frame):
    """Body origin position (x, y) and heading [rad] expressed in the entrance frame (heading 0 = facing into the truck)."""
    c, n, tau = frame
    pos = to_entrance(np.zeros((1, 2)), frame)[0]
    heading = math.atan2(np.array([1.0, 0.0]) @ tau, np.array([1.0, 0.0]) @ n)
    return pos, heading
