#!/usr/bin/env python3
"""Depth bird's-eye-view test for the docking references (2026-10-30). Standalone: opens the RealSense itself, so stop the docking app first.

  source scripts/environment.sh && "$DOCK_PYTHON" scripts/bev_test.py [--port 8082] [--emitter] [--save DIR] [--pitch DEG] [--dist M]

Open http://<jetson>:8082/ in a browser, 2x2: top-left IR with the guide lines (green = vehicle width, cyan = truck inner width,
yellow = the doors' inner lines, red = entrance centre, orange = lattice 2|3 pair), top-right depth with the same marks,
bottom-left VEHICLE view (vehicle fixed at the bottom centre; white = main band 0.45-0.72 m, blue = lattice bar band
0.33-0.43 m, orange line = the fitted bottom bar), bottom-right ENTRANCE view (the doors' line fixed at the centre, vehicle
drawn where it is; prints the lateral/heading relative to the entrance = the stop-window measurement itself).
Without the docking app there is no fused pose, so the slab windows are scanned along x (the nearest doors pair of
truck_inner_width_m +- truck_width_tol_m wins; --dist pins the expected entrance range instead). The lattice is searched
truck_depth_m behind the doors, or scanned when no doors are found. Uses configs/common.json (widths, lattice, camera) and
configs/mount.json (camera pitch) when present. Nothing here drives the vehicle or touches the docking runtime.
"""

import argparse
import math
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import numpy as np
import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from docking import config as cfgmod
from docking.bev import (
    mount_rotation,
    occupancy,
    ground_line,
    xy_grid,
    body_to_pixels,
    entrance_frame,
    to_entrance,
    vehicle_in_entrance,
)
from docking.depth_goal import (
    DEFAULT_PARAMS,
    bev_points,
    pair_in_window,
    lattice_pair_in_window,
    line_in_window,
    body_to_column,
)

LATEST = {'jpg': None, 'n': 0, 't': 0.0, 'txt': ''}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path.startswith('/snap'):
            j = LATEST['jpg'] or b''
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(j)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(j)
            return
        if not self.path.startswith('/stream'):
            # A plain page that embeds the stream (Safari does not render a bare MJPEG stream as a document) and falls back to
            # polling single JPEGs if the stream stalls.
            page = (
                b'<!doctype html><html><head><meta charset="utf-8"><title>BEV test</title><style>body{margin:0;background:#111;color:#ccc;font:14px sans-serif}'
                b'img{width:100%;max-width:1280px;display:block}p{padding:6px 10px;margin:0}</style></head><body>'
                b'<p id="s">frames: '
                + str(LATEST['n']).encode()
                + b' &nbsp; (no image = the camera is not delivering frames; check the terminal)</p>'
                b'<img id="v" src="/stream"><script>let bad=0;const v=document.getElementById("v");v.onerror=()=>{bad++;if(bad>2){setInterval(()=>{v.src="/snap?"+Date.now()},300)}};'
                b'setInterval(()=>{fetch("/stat").then(r=>r.text()).then(t=>{document.getElementById("s").textContent=t})},1000)</script></body></html>'
            )
            if self.path.startswith('/stat'):
                page = (
                    'frames: %d | last frame %.1f s ago | %s'
                    % (LATEST['n'], time.monotonic() - LATEST['t'] if LATEST['t'] else -1, LATEST['txt'])
                ).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/plain; charset=utf-8')
                self.send_header('Content-Length', str(len(page)))
                self.end_headers()
                self.wfile.write(page)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(page)))
            self.end_headers()
            self.wfile.write(page)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                j = LATEST['jpg']
                if j:
                    self.wfile.write(
                        b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                        + str(len(j)).encode()
                        + b'\r\n\r\n'
                        + j
                        + b'\r\n'
                    )
                time.sleep(0.15)
        except (BrokenPipeError, ConnectionResetError):
            pass


class Viewer:
    """Per-frame measurement + rendering, independent of the camera so it can be exercised offline and live (Jetson)."""

    def __init__(self, site, tuning, dg, R, t, intr, src, S=4):
        self.site, self.R, self.t, self.intr, self.src, self.S = site, R, t, intr, src, S
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        self.T = T
        p = dict(DEFAULT_PARAMS)
        p.update({k: v for k, v in (dg or {}).items() if k in p and v is not None})
        p.update(
            {
                k: v
                for k, v in dict(
                    entrance_window_m=tuning.get('entrance_window_m'),
                    lattice_window_m=tuning.get('lattice_window_m'),
                    line_min_len_m=tuning.get('lattice_line_min_len_m'),
                    line_max_residual_m=tuning.get('lattice_line_max_residual_m'),
                ).items()
                if v is not None
            }
        )
        self.p = p
        self.width = float(site['truck_inner_width_m'])
        self.width_tol = float(tuning.get('truck_width_tol_m', 0.2))
        D = site.get('truck_depth_m')
        W = site.get('lattice_pair_width_m')
        self.lattice = (
            None
            if (D is None or not W)
            else (
                float(D),
                float(W),
                float(site.get('lattice_centre_offset_m') or 0.0),
                float(tuning.get('lattice_pair_tol_m', 0.1)),
            )
        )
        self.expected_x = None  # --dist: pin the entrance slab instead of scanning
        self.half_vehicle = site['vehicle_width_m'] / 2
        self.FWD, self.HALF, self.CELL = float(p['forward_m']), float(p['half_width_m']), float(p['cell_m'])
        self.frame = None
        self.held = 0.0
        self.n = 0
        self.t0 = time.monotonic()
        self.pairs = deque(maxlen=10)
        self.raw = deque(maxlen=30)  # smoothing window for the drawn frame; raw jitter statistics
        self.decay = 0.7
        self.acc = None
        self.acc_e = None

    def references(self, depth):
        """The three references of one frame, as the runtime measures them at a stop, but with the slabs scanned along x."""
        p = self.p
        lat = self.lattice
        far = max(p['max_m'], p['lattice_max_m']) if lat else p['max_m']
        zlo = min(p['z_min'], p['lattice_z_min']) if lat else p['z_min']
        zhi = max(p['z_max'], p['lattice_z_max']) if lat else p['z_max']
        body, _, _ = bev_points(depth, self.intr, self.T, int(p['step']), far, zlo, zhi, p['x_min'])
        band = body[(body[:, 2] >= p['z_min']) & (body[:, 2] <= p['z_max'])]
        near = band[band[:, 0] <= p['max_m']]
        doors = dict(ok=False, reason='no_points', n=0)
        xs = (
            [self.expected_x]
            if self.expected_x
            else np.arange(p['x_min'] + p['entrance_window_m'], p['max_m'], p['entrance_window_m'])
        )
        for ex in xs:
            d = pair_in_window(
                near,
                float(ex),
                p['entrance_window_m'],
                self.width,
                self.width_tol,
                0.0,
                p['bin_m'],
                int(p['min_points']),
            )
            if d.get('ok'):
                doors = d
                break
            if d.get('n', 0) > doors.get('n', 0):
                doors = d  # keep the most populated window's reason for the status line
        lp = ln = None
        bar = np.zeros((0, 3))
        if lat:
            D, W, off, tol = lat
            latpts = band[band[:, 0] <= p['lattice_max_m']]
            bar = body[
                (body[:, 2] >= p['lattice_z_min'])
                & (body[:, 2] <= p['lattice_z_max'])
                & (body[:, 0] <= p['lattice_max_m'])
            ]
            if doors.get('ok'):
                cands = [
                    (doors['x_body_m'] + D, doors['lateral_body_m'] - off)
                ]  # the 2|3 centre sits `off` right of the lane centre
            else:
                cands = [
                    (float(ex), 0.0)
                    for ex in np.arange(p['x_min'] + p['lattice_window_m'], p['lattice_max_m'], p['lattice_window_m'])
                ]
            for ex, ey in cands:
                lp = lattice_pair_in_window(
                    latpts, ex, p['lattice_window_m'], W, tol, ey, p['bin_m'], int(p['min_points'])
                )
                ln = line_in_window(
                    bar, ex, p['lattice_window_m'], p['bin_m'], p['line_min_len_m'], 4, p['line_max_residual_m']
                )
                if lp.get('ok') or ln.get('ok'):
                    break
        return body, near, bar, doors, lp, ln

    def column(self, pt):
        u = body_to_column((float(pt[0]), float(pt[1]), float(self.t[2])), self.intr, self.T)
        return None if u is None or not np.isfinite(u) else int(u)

    def compose(self, depth, ir):
        site, R, t, intr, S, FWD, HALF, CELL = (
            self.site,
            self.R,
            self.t,
            self.intr,
            self.S,
            self.FWD,
            self.HALF,
            self.CELL,
        )
        half_truck, half_vehicle = self.width / 2, self.half_vehicle
        frame = self.frame
        held = self.held
        ir = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR) if ir.ndim == 2 else ir.copy()
        h, w = depth.shape
        body, near, bar, doors, lp, ln = self.references(depth)
        grid = occupancy(near, FWD, HALF, CELL, z_min=-math.inf, z_max=math.inf).astype(np.float32)
        self.acc = grid if self.acc is None or self.acc.shape != grid.shape else self.acc * self.decay + grid
        grid = self.acc  # accumulated hit counts (display only; the numbers below come from the per-frame depth)
        # --- guide lines on the IR image
        for yb, col in (
            (half_vehicle, (0, 220, 0)),
            (-half_vehicle, (0, 220, 0)),
            (half_truck, (255, 200, 0)),
            (-half_truck, (255, 200, 0)),
        ):
            line = ground_line(yb, 0.3, 6.0, R, t, intr)
            if len(line) > 1:
                cv2.polylines(ir, [line.astype(np.int32)], False, col, 2 if abs(yb) == half_vehicle else 1)
        for xm in (1.0, 2.0, 3.0, 4.0, 5.0):  # distance ticks across the vehicle-width lines
            seg = np.array([[xm, half_vehicle, 0.0], [xm, -half_vehicle, 0.0]])
            u, v, z = body_to_pixels(seg, R, t, intr)
            if (z > 0.1).all():
                cv2.line(ir, (int(u[0]), int(v[0])), (int(u[1]), int(v[1])), (0, 255, 0), 1)
                cv2.putText(
                    ir, '%.0f m' % xm, (int(u[1]) + 4, int(v[1]) - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1
                )
        # --- depth preview (grey) + the references as columns
        g = (255 - 215 * np.clip(depth, 0, 6.0) / 6.0).astype(np.uint8)
        g[~((depth > 0) & np.isfinite(depth))] = 0
        dv = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        if lp and lp.get('ok'):
            for pt in (lp['left_body'], lp['right_body']):
                u_ = self.column(pt)
                if u_ is not None:
                    for img in (ir, dv):
                        cv2.line(img, (u_, 0), (u_, h - 1), (0, 140, 255), 1)
        if doors.get('ok'):
            for pt in (doors['left_body'], doors['right_body']):
                u_ = self.column(pt)
                if u_ is not None:
                    for img in (ir, dv):
                        cv2.line(img, (u_, 0), (u_, h - 1), (0, 220, 255), 2)
            u_ = self.column((doors['x_body_m'], doors['lateral_body_m']))
            if u_ is not None:
                for img in (ir, dv):
                    cv2.line(img, (u_, 0), (u_, h - 1), (0, 0, 255), 2)
            line1 = 'doors %.2f+-%.2f m: centre %+.3f m (left+) width %.2f at %.2f m, line %+.1f deg' % (
                self.width,
                self.width_tol,
                doors['lateral_body_m'],
                doors['width_m'],
                doors['x_body_m'],
                doors['pair_heading_deg'],
            )
        else:
            line1 = 'doors %.2f+-%.2f m: %s' % (self.width, self.width_tol, doors.get('reason'))
        if self.lattice is None:
            line2 = 'lattice: not configured (truck_depth_m / lattice_pair_width_m)'
        else:
            line2 = 'lattice 2|3 %.2f: %s | bar: %s' % (
                self.lattice[1],
                (
                    ('centre %+.3f at %.2f m' % (lp['lateral_body_m'], lp['x_body_m']))
                    if lp and lp.get('ok')
                    else (lp or {}).get('reason')
                ),
                (
                    ('%.2f m, heading %+.1f deg, span %.1f' % (ln['x_body_m'], ln['heading_err_deg'], ln['span_m']))
                    if ln and ln.get('ok')
                    else (ln or {}).get('reason')
                ),
            )
        for img in (ir, dv):
            cv2.rectangle(img, (0, h - 40), (w - 1, h - 1), (0, 0, 0), -1)
            cv2.putText(
                img,
                line1,
                (8, h - 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 220, 255) if doors.get('ok') else (0, 165, 255),
                1,
            )
            cv2.putText(img, line2, (8, h - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 1)
        # --- bird's-eye render (vehicle view)
        bev = np.zeros(grid.shape + (3,), np.uint8)
        occ = np.clip(grid * 40, 0, 255).astype(np.uint8)
        bev[..., :] = occ[..., None]
        if len(bar):  # the lattice bar band in blue
            rr = grid.shape[0] - 1 - (bar[:, 0] / CELL).astype(int)
            cc = ((HALF - bar[:, 1]) / CELL).astype(int)
            ok_ = (rr >= 0) & (rr < grid.shape[0]) & (cc >= 0) & (cc < grid.shape[1])
            bev[rr[ok_], cc[ok_]] = np.maximum(bev[rr[ok_], cc[ok_]], (200, 120, 0))
        bev = cv2.resize(bev, (grid.shape[1] * S, grid.shape[0] * S), interpolation=cv2.INTER_NEAREST)

        def P(x, y):
            r, cc = xy_grid(x, y, FWD, HALF, CELL)
            return (int(cc * S + S / 2), int(r * S + S / 2))

        for xm in range(1, int(FWD)):  # range rings (lines)
            cv2.line(bev, P(xm, HALF - 0.01), P(xm, -HALF + 0.01), (50, 50, 50), 1)
            cv2.putText(bev, '%d' % xm, (4, P(xm, 0)[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)
        tip = site['camera_to_fork_tip_m']
        rear = -site['rear_extent_m']
        cv2.rectangle(
            bev, P(min(tip, FWD - 0.01), half_vehicle), P(max(rear, 0.0), -half_vehicle), (0, 220, 0), 1
        )  # vehicle (clipped to the grid)
        cv2.line(bev, P(tip, half_vehicle), P(FWD - 0.01, half_vehicle), (0, 120, 0), 1)
        cv2.line(bev, P(tip, -half_vehicle), P(FWD - 0.01, -half_vehicle), (0, 120, 0), 1)
        cv2.line(bev, P(tip, half_truck), P(FWD - 0.01, half_truck), (255, 200, 0), 1)
        cv2.line(bev, P(tip, -half_truck), P(FWD - 0.01, -half_truck), (255, 200, 0), 1)
        cv2.circle(bev, P(0.0, t[1]), 3, (255, 255, 255), -1)  # camera
        if doors.get('ok'):
            for pt, col in (
                (doors['left_body'], (0, 220, 255)),
                (doors['right_body'], (0, 220, 255)),
                ((doors['x_body_m'], doors['lateral_body_m']), (0, 0, 255)),
            ):
                if 0 <= pt[0] < FWD and abs(pt[1]) < HALF:
                    cv2.circle(bev, P(pt[0], pt[1]), 5, col, 2)
        if lp and lp.get('ok'):
            for pt in (lp['left_body'], lp['right_body']):
                if 0 <= pt[0] < FWD and abs(pt[1]) < HALF:
                    cv2.circle(bev, P(pt[0], pt[1]), 5, (0, 140, 255), 2)
        if ln and ln.get('ok'):
            y0, y1 = -ln['span_m'] / 2, ln['span_m'] / 2
            x0, x1 = ln['x_body_m'] + ln['slope'] * y0, ln['x_body_m'] + ln['slope'] * y1
            if 0 <= x0 < FWD and 0 <= x1 < FWD:
                cv2.line(bev, P(x0, y0), P(x1, y1), (0, 140, 255), 2)
        self.n += 1
        fps = self.n / max(1e-6, time.monotonic() - self.t0)
        cv2.rectangle(bev, (0, bev.shape[0] - 18), (bev.shape[1] - 1, bev.shape[0] - 1), (0, 0, 0), -1)
        cv2.putText(
            bev,
            'VEHICLE view | 5 cm cells | %.1f fps | %d pts (band %d, bar %d) | %s'
            % (fps, len(body), len(near), len(bar), self.src),
            (6, bev.shape[0] - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (200, 200, 200),
            1,
        )
        # --- entrance-fixed view: rotate the world so the doors' line sits on a fixed line
        if doors.get('ok'):
            raw_pair = (doors['left_body'][0], doors['left_body'][1], doors['right_body'][0], doors['right_body'][1])
            rp, rh = vehicle_in_entrance(entrance_frame(np.array(raw_pair[:2]), np.array(raw_pair[2:])))
            self.raw.append((rp[1], math.degrees(rh)))
            self.pairs.append(raw_pair)
            med = tuple(float(np.median([q[i] for q in self.pairs])) for i in range(4))  # running median (~0.7 s)
            frame = entrance_frame(np.array(med[:2]), np.array(med[2:]))
            held = time.monotonic()
        else:
            self.pairs.clear()
        if frame is not None and time.monotonic() - held < 3.0:
            EX0, EX1, EH = -6.0, 3.0, 3.0  # entrance frame: x from 6 m before the entrance to 3 m inside, +-3 m across
            pe = to_entrance(body[:, :2], frame)
            keep = (pe[:, 0] >= EX0) & (pe[:, 0] < EX1) & (np.abs(pe[:, 1]) < EH)
            rows_e = int((EX1 - EX0) / CELL)
            cols_e = int(2 * EH / CELL)
            ge = np.zeros((rows_e, cols_e), np.int32)
            if keep.any():
                rr = rows_e - 1 - ((pe[keep, 0] - EX0) / CELL).astype(int)
                cc = ((EH - pe[keep, 1]) / CELL).astype(int)
                np.add.at(ge, (np.clip(rr, 0, rows_e - 1), np.clip(cc, 0, cols_e - 1)), 1)
            ge = ge.astype(np.float32)
            self.acc_e = ge if self.acc_e is None or self.acc_e.shape != ge.shape else self.acc_e * self.decay + ge
            ge = self.acc_e
            ev = np.zeros((rows_e, cols_e, 3), np.uint8)
            ev[..., :] = np.clip(ge * 40, 0, 255).astype(np.uint8)[..., None]
            ev = cv2.resize(ev, (cols_e * S, rows_e * S), interpolation=cv2.INTER_NEAREST)

            def Q(x, y):
                return (int(((EH - y) / CELL) * S + S / 2), int((rows_e - 1 - (x - EX0) / CELL) * S + S / 2))

            cv2.line(ev, Q(0.0, EH - 0.01), Q(0.0, -EH + 0.01), (0, 220, 255), 2)  # entrance line
            cv2.line(ev, Q(0.0, half_truck), Q(EX1 - 0.01, half_truck), (255, 200, 0), 2)
            cv2.line(ev, Q(0.0, -half_truck), Q(EX1 - 0.01, -half_truck), (255, 200, 0), 2)
            cv2.line(ev, Q(EX0 + 0.01, 0.0), Q(EX1 - 0.01, 0.0), (0, 0, 255), 1)  # truck axis
            if self.lattice is not None:
                D, W, off, _ = self.lattice
                cv2.line(ev, Q(D, -half_truck), Q(D, half_truck), (0, 140, 255), 1)  # the configured lattice plane
                for y in (-off + W / 2, -off - W / 2):
                    cv2.line(ev, Q(D - 0.08, y), Q(D + 0.08, y), (0, 140, 255), 2)
            for xm in range(int(EX0), int(EX1)):
                cv2.line(ev, Q(xm, EH - 0.01), Q(xm, -EH + 0.01), (50, 50, 50), 1)
                cv2.putText(ev, '%+d' % xm, (4, Q(xm, 0)[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)
            corners = np.array([[tip, half_vehicle], [tip, -half_vehicle], [rear, -half_vehicle], [rear, half_vehicle]])
            ce = to_entrance(corners, frame)
            poly = np.array([Q(x, y) for x, y in ce], np.int32)
            cv2.polylines(ev, [poly], True, (0, 220, 0), 2)
            cv2.line(
                ev,
                Q(*to_entrance(np.array([[tip, 0.0]]), frame)[0]),
                Q(*to_entrance(np.array([[tip + 3.0, 0.0]]), frame)[0]),
                (0, 120, 0),
                1,
            )  # vehicle axis
            pos, hdg = vehicle_in_entrance(frame)
            tipe = to_entrance(np.array([[tip, 0.0]]), frame)[0]
            cv2.rectangle(ev, (0, ev.shape[0] - 18), (ev.shape[1] - 1, ev.shape[0] - 1), (0, 0, 0), -1)
            cv2.putText(
                ev,
                'ENTRANCE view (10-frame median) | lateral %+.3f m heading %+.1f deg | fork tip %.2f m before the entrance%s | raw jitter sd %s'
                % (
                    pos[1],
                    math.degrees(hdg),
                    -tipe[0],
                    '' if doors.get('ok') else ' | HELD',
                    (
                        (
                            '%.3f m / %.2f deg (n=%d)'
                            % (np.std([r[0] for r in self.raw]), np.std([r[1] for r in self.raw]), len(self.raw))
                        )
                        if len(self.raw) >= 5
                        else '-'
                    ),
                ),
                (6, ev.shape[0] - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (200, 200, 200),
                1,
            )
        else:
            ev = np.zeros((int(9.0 / CELL) * S, int(6.0 / CELL) * S, 3), np.uint8)
            cv2.putText(ev, 'ENTRANCE view: no doors pair', (6, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1)

        def fit(img):
            r = cv2.resize(img, (int(img.shape[1] * h / img.shape[0]), h))
            canvas = np.zeros((h, w, 3), np.uint8)
            x0 = (w - r.shape[1]) // 2 if r.shape[1] <= w else 0
            canvas[:, x0 : x0 + min(w, r.shape[1])] = r[:, : min(w, r.shape[1])]
            return canvas

        out = np.vstack([np.hstack([ir, dv]), np.hstack([fit(bev), fit(ev)])])
        self.frame, self.held = frame, held
        return out, doors, lp, ln


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8082)
    ap.add_argument(
        '--emitter',
        action='store_true',
        help='IR projector on (the runtime turns it on only while standing; never with the VIO running)',
    )
    ap.add_argument('--save', help='write a combined PNG here every --every seconds')
    ap.add_argument('--every', type=float, default=2.0)
    ap.add_argument('--pitch', type=float, default=None, help='camera pitch down [deg] if configs/mount.json is absent')
    ap.add_argument('--mode', default='left')
    ap.add_argument(
        '--step',
        type=int,
        default=None,
        help='pixel subsampling for the point cloud (default: runtime.json depth_goal.step)',
    )
    ap.add_argument('--fps', type=int, default=15)
    ap.add_argument(
        '--no-filter',
        action='store_true',
        help='skip the RealSense spatial+temporal post-processing (raw per-frame depth)',
    )
    ap.add_argument('--decay', type=float, default=0.7, help="bird's-eye accumulation: old*decay + new (0 = per frame)")
    ap.add_argument(
        '--width', type=float, default=None, help='doors inner width to look for [m] (default: truck_inner_width_m)'
    )
    ap.add_argument('--width-tol', type=float, default=None, help='default: truck_width_tol_m')
    ap.add_argument(
        '--dist',
        type=float,
        default=None,
        help='expected camera -> entrance line range [m]; default: scan the slabs along x',
    )
    a = ap.parse_args()
    cfg = cfgmod.load(mode=a.mode, require_mount=False)
    site = cfg['site']
    cam = cfg['sensor']['camera']
    tuning = cfg['tuning']
    dg = dict(cfg['sensor'].get('depth_goal') or {})
    if a.step:
        dg['step'] = a.step
    calib = cfg['sensor'].get('calibration') or {}
    if calib.get('T_body_camera') is not None:
        T = np.array(calib['T_body_camera'])
        R = T[:3, :3]
        src = 'mount.json'
    else:
        R = mount_rotation(a.pitch or 0.0)
        src = 'fallback pitch %.1f deg' % (a.pitch or 0.0)
    t = np.array([0.0, site['camera_lateral_m'], site['camera_height_m']])
    print(
        'rotation source:',
        src,
        '| camera height %.2f lateral %+.3f fork tip %.2f' % (t[2], t[1], site['camera_to_fork_tip_m']),
        flush=True,
    )

    import pyrealsense2 as rs

    pipe = rs.pipeline()
    c = rs.config()
    c.enable_stream(rs.stream.infrared, 1, cam['ir_width'], cam['ir_height'], rs.format.y8, a.fps)
    c.enable_stream(rs.stream.depth, cam['ir_width'], cam['ir_height'], rs.format.z16, a.fps)
    prof = pipe.start(c)
    for s in prof.get_device().query_sensors():
        if s.supports(rs.option.emitter_enabled):
            s.set_option(rs.option.emitter_enabled, 1.0 if a.emitter else 0.0)
        if s.supports(rs.option.enable_auto_exposure):
            s.set_option(rs.option.enable_auto_exposure, 1.0)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    filters = []
    if not a.no_filter:
        try:
            sp = rs.spatial_filter()
            sp.set_option(rs.option.filter_magnitude, 2)
            sp.set_option(rs.option.filter_smooth_alpha, 0.5)
            sp.set_option(rs.option.filter_smooth_delta, 20)
            tp = rs.temporal_filter()
            tp.set_option(rs.option.filter_smooth_alpha, 0.4)
            tp.set_option(rs.option.filter_smooth_delta, 20)
            filters = [sp, tp]
            print('depth post-processing: spatial + temporal (RealSense SDK). --no-filter for raw frames', flush=True)
        except Exception as exc:
            print('depth filters unavailable (%s): raw frames' % exc, flush=True)
    di = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    intr = dict(fx=di.fx, fy=di.fy, cx=di.ppx, cy=di.ppy)
    print(
        'depth intrinsics', {k: round(v, 1) for k, v in intr.items()}, 'scale', scale, 'emitter', a.emitter, flush=True
    )
    srv = ThreadingHTTPServer(('0.0.0.0', a.port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(
        '화면: SSH 포트 전달 후 http://127.0.0.1:%d   (VS Code 터미널이면 이 주소를 클릭 — 도킹 화면과 같은 방식. Ctrl+C 로 종료)'
        % a.port,
        flush=True,
    )
    viewer = Viewer(site, tuning, dg, R, t, intr, src)
    viewer.decay = max(0.0, min(0.95, a.decay))
    last_save = 0.0
    if a.width:
        viewer.width = a.width
    if a.width_tol is not None:
        viewer.width_tol = a.width_tol
    if a.dist:
        viewer.expected_x = a.dist
    print(
        'doors %.2f +- %.2f m wide, band %.2f-%.2f m, up to %.1f m | lattice %s'
        % (
            viewer.width,
            viewer.width_tol,
            viewer.p['z_min'],
            viewer.p['z_max'],
            viewer.p['max_m'],
            (
                'off'
                if viewer.lattice is None
                else '2|3 %.2f m at %.2f m inside (offset %+.3f), bar band %.2f-%.2f m, up to %.1f m'
                % (
                    viewer.lattice[1],
                    viewer.lattice[0],
                    viewer.lattice[2],
                    viewer.p['lattice_z_min'],
                    viewer.p['lattice_z_max'],
                    viewer.p['lattice_max_m'],
                )
            ),
        ),
        flush=True,
    )
    try:
        while True:
            try:
                fs = pipe.wait_for_frames(5000)
            except RuntimeError as exc:
                print(
                    'no frames for 5 s: %s (is the docking app or another process holding the camera?)' % exc,
                    flush=True,
                )
                continue
            dfr = fs.get_depth_frame()
            ifr = fs.get_infrared_frame(1)
            if not dfr or not ifr:
                continue
            for f in filters:
                try:
                    dfr = f.process(dfr)
                except Exception as exc:
                    print('filter failed (%s): disabling' % exc, flush=True)
                    filters = []
                    break
            depth = np.asanyarray(dfr.get_data()).astype(np.float32) * scale
            ir = np.asanyarray(ifr.get_data())
            out, doors, lp, ln = viewer.compose(depth, ir)
            ok, buf = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 70])
            LATEST['jpg'] = bytes(buf) if ok else None
            LATEST['n'] += 1
            LATEST['t'] = time.monotonic()
            LATEST['txt'] = (
                (
                    'doors: centre %+.3f width %.2f at %.2f'
                    % (doors['lateral_body_m'], doors['width_m'], doors['x_body_m'])
                )
                if doors.get('ok')
                else 'doors: ' + str(doors.get('reason'))
            )
            LATEST['txt'] += (' | lattice pair at %.2f' % lp['x_body_m']) if lp and lp.get('ok') else ''
            LATEST['txt'] += (
                (' | bar %.2f m %+.1f deg' % (ln['x_body_m'], ln['heading_err_deg'])) if ln and ln.get('ok') else ''
            )
            if LATEST['n'] % 75 == 1:
                print('frames %d | %s' % (LATEST['n'], LATEST['txt']), flush=True)
            if a.save and time.monotonic() - last_save >= a.every:
                Path(a.save).mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(Path(a.save) / ('bev_%s.png' % time.strftime('%H%M%S'))), out)
                last_save = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()


if __name__ == '__main__':
    main()
