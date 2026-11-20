"""The depth stream's per-frame work (2026-11-01): the bird's-eye references (depth_goal.measure_bev) and the dashboard cells in
one object, run either in a thread of the camera worker or - by default - in its own process.

Why a process: on the Orin the measurement took 68 ms median per frame at 10 Hz (2026-10-30 logs) and its Python parts held the
interpreter lock, so the capture loop's stages stalled 30-60 ms and only 20-22 of 29 camera frames reached OpenVINS. The camera
worker is a daemon process and cannot spawn children, so the supervisor (runtime) creates the shared-memory ring and the process
(`link`, `run_process`) and hands the queues to the camera worker, which only copies each depth frame into a ring slot. The
measurer itself (`DepthMeasurer.frame`) is the same code in both modes, so the results do not depend on where they are computed.
"""

import time
import numpy as np
from . import ring


def link(ctx, camera_cfg, slots=4):
    """Supervisor side: the shared ring for depth frames (z16) plus the queues (docking.ring.link)."""
    return ring.link(ctx, (int(camera_cfg['ir_height']), int(camera_cfg['ir_width'])), np.uint16, slots)


def ring_views(buf, slots, frame_shape):
    return ring.views(buf, slots, frame_shape, np.uint16)


class DepthMeasurer:
    """One depth frame -> dict(depth=<measurement>, bev=<dashboard cells or None>). settings: T_body_ir, depth_intr, depth_scale,
    site, tuning, depth_goal (runtime.json section), inner (calibration.inner or {}), view (bool)."""

    PARAM_KEYS = (
        'max_m',
        'lattice_max_m',
        'step',
        'z_min',
        'z_max',
        'lattice_z_min',
        'lattice_z_max',
        'lattice_pair_z_min',
        'lattice_pair_z_max',
        'x_min',
        'cell_m',
        'forward_m',
        'half_width_m',
    )

    def __init__(self, settings):
        from .depth_goal import measure_bev, expected_column
        from .bev import bev_cells

        self.measure_bev, self.expected_column, self.bev_cells = measure_bev, expected_column, bev_cells
        s = settings
        self.T_body_ir = np.array(s['T_body_ir'], float)
        self.intr = s['depth_intr']
        self.scale = float(s['depth_scale'])
        site = s.get('site') or {}
        tn = s['tuning']
        dg = s.get('depth_goal') or {}
        self.width = float(site.get('truck_inner_width_m') or 0.0)
        self.tol = float(tn.get('truck_width_tol_m', 0.2))
        self.half_vehicle = float(site.get('vehicle_width_m') or 1.28) / 2
        self.lattice_x = site.get('truck_depth_m')
        self.lat_w = float(site.get('lattice_pair_width_m') or 0.0)
        self.lat_off = float(site.get('lattice_centre_offset_m') or 0.0)
        self.lattice_on = self.lattice_x is not None and self.lat_w > 0
        self.lat_tol = float(tn.get('lattice_pair_tol_m', 0.1))
        self.params = {k: dg[k] for k in self.PARAM_KEYS if k in dg}
        self.params.update(
            entrance_window_m=tn.get('entrance_window_m'),
            lattice_window_m=tn.get('lattice_window_m'),
            line_min_len_m=tn.get('lattice_line_min_len_m'),
            line_max_residual_m=tn.get('lattice_line_max_residual_m'),
        )
        self.params.setdefault('cell_m', float(dg.get('bev_cell_m', 0.05)))
        self.params.setdefault('forward_m', float(dg.get('bev_forward_m', 8.0)))
        self.params.setdefault('half_width_m', float(dg.get('bev_half_width_m', 3.0)))
        inner = s.get('inner') or {}
        self.plates = [
            (float(T[0][3]), float(T[1][3]), float(inner.get('plate_half_width_m') or 0.0))
            for T in (inner.get('tags') or {}).values()
        ]  # dock x, dock y, half width
        # 실험판 D: the tags may hang in front of the lattice. Their plates are cut out of the lattice search only when they sit
        # inside the lattice depth slab (glued to a member, as in the base version); plates further forward are not in the slab,
        # and cutting their lateral bands would remove real members behind them.
        slab = float(tn.get('lattice_window_m') or 0.35)
        self.plates = [
            p for p in self.plates if self.lattice_x is not None and abs(p[0] - float(self.lattice_x)) <= slab
        ]
        self.view = bool(s.get('view'))
        self.bev_hz = max(0.5, float(dg.get('bev_hz', 2.0)))
        self.bev_decay = float(dg.get('bev_decay', 0.7))
        self.last_bev = -1.0
        self.bev_acc = None
        self.empty = dict(doors=None, lattice_pair=None, lattice_line=None, method='window')

    def frame(self, raw, stamp, pose, arrival, standing):
        t_start = time.monotonic()
        depth = raw.astype(np.float32) * self.scale
        grid = None
        exp = None
        try:
            exp = self.expected_column(pose, self.intr, self.T_body_ir) if pose is not None else None
            if self.width <= 0:
                m = dict(self.empty, ok=False, reason='no_truck_inner_width', stamp=stamp)
            elif pose is None:
                m = dict(
                    self.empty, ok=False, reason='no_pose_for_windows', stamp=stamp
                )  # the windows need the fused pose
            else:
                c_, s_ = np.cos(pose[2]), np.sin(pose[2])

                def body_of(dx, dy):  # dock point -> body frame
                    return (
                        float(c_ * (dx - pose[0]) + s_ * (dy - pose[1])),
                        float(-s_ * (dx - pose[0]) + c_ * (dy - pose[1])),
                    )

                ex, _ = body_of(0.0, 0.0)
                entrance = dict(expected_x_m=ex, width_m=self.width, width_tol_m=self.tol, heading_rad=float(pose[2]))
                lattice = None
                if self.lattice_on:
                    lx, ly = body_of(
                        float(self.lattice_x), -self.lat_off
                    )  # the 2|3 pair centre sits lat_off right of the lane centre
                    lattice = dict(expected_x_m=lx, expected_y_m=ly, pair_width_m=self.lat_w, pair_tol_m=self.lat_tol)
                    if self.plates:  # the inner-tag plates on the lattice are not members
                        lattice['plates'] = [(body_of(px, py)[1], hw) for px, py, hw in self.plates]
                m = self.measure_bev(
                    depth,
                    self.intr,
                    self.T_body_ir,
                    entrance,
                    lattice,
                    params=self.params,
                    stamp=stamp,
                    vehicle_half_width_m=self.half_vehicle,
                )
                grid = m.pop('grid', None)
        except Exception as exc:
            m = dict(self.empty, ok=False, reason='error: ' + str(exc), stamp=stamp)
        m['expected_u'] = exp if self.width > 0 else None
        m['compute_ms'] = round((time.monotonic() - t_start) * 1000.0, 1)
        m['arrival'] = arrival
        bev = None
        if (
            (self.view or standing) and grid is not None and time.monotonic() - self.last_bev >= 1 / self.bev_hz
        ):  # dashboard, and the log while standing
            try:
                g = grid.astype(np.float32)
                self.bev_acc = (
                    g if (self.bev_acc is None or self.bev_acc.shape != g.shape) else self.bev_acc * self.bev_decay + g
                )
                cells = self.bev_cells(
                    self.bev_acc, self.params['forward_m'], self.params['half_width_m'], self.params['cell_m']
                )
                bev = dict(
                    stamp=stamp,
                    cell_m=self.params['cell_m'],
                    cells=cells,
                    points=int(m.get('points') or 0),
                    standing=standing,
                )
            except Exception as exc:
                bev = dict(stamp=stamp, error=str(exc), cells=[])
            self.last_bev = time.monotonic()
        return dict(depth=m, bev=bev)


def run_process(shm_name, slots, frame_shape, frames_q, results_q, stop, settings_q=None):
    """The depth process (docking.ring.run with a DepthMeasurer built from the settings)."""
    ring.run(DepthMeasurer, shm_name, slots, frame_shape, np.uint16, frames_q, results_q, stop, settings_q)
