"""Fine-stage geometry (v2, 2026-10-22): the lateral-correction bundle and depth-entrance bookkeeping.

A bundle is turn(alpha) -> drive d -> turn back to the straight heading. It moves the body origin sideways while the
heading ends where it started, which is the only admissible correction once the vehicle must keep heading 0 (the depth
reading of the entrance centre is interpreted at heading 0). Both in-place turns swing the body on the pivot arc
(pivot 0.78 m behind the fork-front camera), so the drive is solved from the full kinematics, not from d*sin(alpha).
Pure functions; nothing here touches sensors or commands.
"""
import math
import numpy as np
from .geometry import moved, wrap


def end_lateral(p, alpha, d, pivot, heading_target):
    """Body-origin y after turn(alpha), drive(d), turn to heading_target (nominal kinematics)."""
    q = moved(p, alpha, 0., pivot)
    q = moved(q, 0., d, pivot)
    q = moved(q, wrap(heading_target - q[2]), 0., pivot)
    return float(q[1])


def drive_for(p, centre_y, pivot, alpha, heading_target):
    """Drive length that lands the body origin on centre_y for a given turn alpha (linear in d); may be negative/huge."""
    y0 = end_lateral(p, alpha, 0., pivot, heading_target)
    s = math.sin(p[2] + alpha)
    if abs(s) < 1e-6:
        return math.inf if abs(centre_y - y0) > 1e-9 else 0.
    return (centre_y - y0) / s


def turn_for(p, centre_y, pivot, d, heading_target, alpha_max, iterations=40):
    """Turn (|alpha| <= alpha_max) that lands the body origin on centre_y after driving d: bisection on the end lateral,
    which is monotonic in alpha for d > 0 and small headings. Saturates at +-alpha_max (partial correction)."""
    f = lambda a: end_lateral(p, a, d, pivot, heading_target) - centre_y
    lo, hi = -alpha_max, alpha_max
    flo, fhi = f(lo), f(hi)
    if flo * fhi > 0:
        return lo if abs(flo) < abs(fhi) else hi
    for _ in range(iterations):
        mid = (lo + hi) / 2
        fm = f(mid)
        if flo * fm <= 0:
            hi, fhi = mid, fm
        else:
            lo, flo = mid, fm
    return (lo + hi) / 2


def plan_bundle(p, centre_y, pivot, room, d_min, alpha_max, heading_target):
    """Choose (alpha, d) for one correction bundle.

    room: forward distance available before the next check point; d_min: smallest real step.
    Policy (2026-10-22 user decision 'formula, not planner'): the shortest drive that fully corrects at alpha_max; if that is
    shorter than d_min, drive d_min with the turn that fits; if it does not fit in the room, use the room and leave the rest
    to the next iteration. Returns dict(alpha, d, predicted_end_y, partial) or None when nothing useful fits."""
    if room < d_min - 1e-9:
        return None
    e = float(p[1]) - float(centre_y)
    side = -1. if e > 0 else 1.              # vehicle left of the centre -> turn right (negative) to go right
    d_full = drive_for(p, centre_y, pivot, side * alpha_max, heading_target)
    if not math.isfinite(d_full) or d_full < 0:
        d_full = math.inf
    if d_full <= d_min:
        d = d_min
    elif d_full <= room:
        d = d_full
    else:
        d = room
    alpha = turn_for(p, centre_y, pivot, d, heading_target, alpha_max)
    end_y = end_lateral(p, alpha, d, pivot, heading_target)
    return dict(alpha=float(alpha), d=float(d), predicted_end_y=end_y, partial=bool(abs(end_y - centre_y) > .005))


def swing_back_y(p, pivot, heading_target):
    """Lateral shift of the body origin that the pending turn back to heading_target will add (used while driving)."""
    return float(moved(p, wrap(heading_target - p[2]), 0., pivot)[1] - p[1])


def entrance_in_dock(p, depth):
    """Depth entrance (body frame: x forward, y left of the body origin) -> dock-frame centre (x, y) and the heading of the
    body relative to the entrance normal implied by the pair. p is the fused dock pose of the body origin."""
    c, s = math.cos(p[2]), math.sin(p[2])
    xb, yb = float(depth['x_body_m']), float(depth['lateral_body_m'])
    return float(p[0] + c * xb - s * yb), float(p[1] + s * xb + c * yb)


def lateral_error(p, centre_dock):
    """Vehicle lateral relative to the entrance centreline (left +): the body origin's dock y minus the centre's dock y."""
    return float(p[1]) - float(centre_dock[1])


def distance_to_entrance(p, centre_dock):
    """Camera (body origin) distance to the entrance plane along the dock x axis, from a depth-measured centre."""
    return float(centre_dock[0]) - float(p[0])


def final_check_distance(cfg):
    """Camera distance before the entrance plane at which the last depth check happens: where the entrance edges are still
    inside the depth field of view (half width / tan(half HFOV)) plus a margin; overridable (operation.final_check_distance_m)."""
    t = cfg['tuning']
    v = t.get('final_check_distance_m')
    if v is not None and math.isfinite(float(v)) and float(v) > 0:
        return float(v)
    half = cfg['site']['truck_inner_width_m'] / 2
    hfov = math.radians(float(t.get('depth_hfov_deg', 80.)))
    margin = float(t.get('final_check_margin_m', .1))
    # ... and never with the fork tip already past the entrance plane (an in-place correction turn there swings the tip into a wall)
    return max(half / math.tan(hfov / 2) + margin, cfg['site']['camera_to_fork_tip_m'] + margin)
