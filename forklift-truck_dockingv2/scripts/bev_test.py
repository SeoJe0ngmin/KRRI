#!/usr/bin/env python3
"""Depth bird's-eye-view + parking guide-line test (2026-10-22). Standalone: opens the RealSense itself, so stop the docking app first.

  source scripts/environment.sh && "$DOCK_PYTHON" scripts/bev_test.py [--port 8082] [--emitter] [--save DIR] [--pitch DEG]

Open http://<jetson>:8082/ in a browser, 2x2: top-left IR with the guide lines (green = vehicle width, cyan = truck inner
width, yellow = entrance pair, red = entrance centre), top-right depth, bottom-left VEHICLE view (vehicle fixed at the bottom
centre, world moves), bottom-right ENTRANCE view (entrance found by depth fixed at the centre line, vehicle drawn where it is).
The entrance view prints the vehicle's lateral/heading relative to the entrance: the stage-2 measurement itself.
Uses configs/common.json (camera height / lateral / fork tip / widths) and configs/mount.json (camera pitch) when present.
Nothing here drives the vehicle or touches the docking runtime.
"""
import argparse, json, math, sys, threading, time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import numpy as np
import cv2
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from docking import config as cfgmod
from docking.bev import mount_rotation, depth_to_points, camera_to_body, occupancy, ground_line, xy_grid, body_to_pixels, entrance_frame, to_entrance, vehicle_in_entrance
from docking.depth_goal import find_goal

LATEST = {'jpg': None, 'n': 0, 't': 0., 'txt': ''}

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        if self.path.startswith('/snap'):
            j = LATEST['jpg'] or b''
            self.send_response(200); self.send_header('Content-Type', 'image/jpeg'); self.send_header('Content-Length', str(len(j))); self.send_header('Cache-Control', 'no-store'); self.end_headers(); self.wfile.write(j); return
        if not self.path.startswith('/stream'):
            # A plain page that embeds the stream (Safari does not render a bare MJPEG stream as a document) and falls back to
            # polling single JPEGs if the stream stalls.
            page = (b'<!doctype html><html><head><meta charset="utf-8"><title>BEV test</title><style>body{margin:0;background:#111;color:#ccc;font:14px sans-serif}'
                    b'img{width:100%;max-width:1280px;display:block}p{padding:6px 10px;margin:0}</style></head><body>'
                    b'<p id="s">frames: ' + str(LATEST['n']).encode() + b' &nbsp; (no image = the camera is not delivering frames; check the terminal)</p>'
                    b'<img id="v" src="/stream"><script>let bad=0;const v=document.getElementById("v");v.onerror=()=>{bad++;if(bad>2){setInterval(()=>{v.src="/snap?"+Date.now()},300)}};'
                    b'setInterval(()=>{fetch("/stat").then(r=>r.text()).then(t=>{document.getElementById("s").textContent=t})},1000)</script></body></html>')
            if self.path.startswith('/stat'):
                page = ('frames: %d | last frame %.1f s ago | %s' % (LATEST['n'], time.monotonic() - LATEST['t'] if LATEST['t'] else -1, LATEST['txt'])).encode()
                self.send_response(200); self.send_header('Content-Type', 'text/plain; charset=utf-8'); self.send_header('Content-Length', str(len(page))); self.end_headers(); self.wfile.write(page); return
            self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Content-Length', str(len(page))); self.end_headers(); self.wfile.write(page); return
        self.send_response(200); self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame'); self.end_headers()
        try:
            while True:
                j = LATEST['jpg']
                if j:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(j)).encode() + b'\r\n\r\n' + j + b'\r\n')
                time.sleep(.15)
        except (BrokenPipeError, ConnectionResetError):
            pass

class Viewer:
    """Per-frame rendering, independent of the camera so it can be exercised offline (tests) and live (Jetson)."""
    def __init__(self, site, R, t, intr, src, S=4, FWD=8., HALF=3., CELL=.05):
        self.site, self.R, self.t, self.intr, self.src, self.S, self.FWD, self.HALF, self.CELL = site, R, t, intr, src, S, FWD, HALF, CELL
        self.frame = None; self.held = 0.; self.n = 0; self.t0 = time.monotonic()
        self.pairs = deque(maxlen=10); self.raw = deque(maxlen=30)   # smoothing window for the drawn frame; raw jitter statistics
        self.width_tol = .2
        self.decay = .7; self.acc = None; self.acc_e = None
        self.half_truck = site.get('truck_inner_width_m', site.get('bay_width_m', 1.4)) / 2; self.half_vehicle = site['vehicle_width_m'] / 2

    def compose(self, depth, ir, step=2):
        site, R, t, intr, src, S, FWD, HALF, CELL = self.site, self.R, self.t, self.intr, self.src, self.S, self.FWD, self.HALF, self.CELL
        half_truck, half_vehicle = self.half_truck, self.half_vehicle; frame = self.frame; held = self.held; n = self.n; t0 = self.t0; a = argparse.Namespace(step=step)
        ir = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR) if ir.ndim == 2 else ir.copy()
        h, w = depth.shape
        # --- point cloud -> body -> bird's-eye grid
        pts, _, _ = depth_to_points(depth, intr, a.step, FWD)
        body = camera_to_body(pts, R, t)
        grid = occupancy(body, FWD, HALF, CELL).astype(np.float32)
        self.acc = grid if self.acc is None or self.acc.shape != grid.shape else self.acc * self.decay + grid
        grid = self.acc   # accumulated hit counts (display only; the entrance numbers below come from the per-frame depth)
        # --- entrance pair on the depth image (1-D band method)
        goal = find_goal(depth, intr, width_m=2 * half_truck, width_tol_m=self.width_tol)   # only a pair truck_inner_width +- tol apart
        cand = find_goal(depth, intr)   # nearest opening of ANY width: always drawn (orange) so the centre point is visible
        # --- guide lines on the IR image
        for yb, col in ((half_vehicle, (0, 220, 0)), (-half_vehicle, (0, 220, 0)), (half_truck, (255, 200, 0)), (-half_truck, (255, 200, 0))):
            line = ground_line(yb, .3, 6., R, t, intr)
            if len(line) > 1: cv2.polylines(ir, [line.astype(np.int32)], False, col, 2 if abs(yb) == half_vehicle else 1)
        for xm in (1., 2., 3., 4., 5.):   # distance ticks across the vehicle-width lines
            seg = np.array([[xm, half_vehicle, 0.], [xm, -half_vehicle, 0.]]); u, v, z = body_to_pixels(seg, R, t, intr)
            if (z > .1).all(): cv2.line(ir, (int(u[0]), int(v[0])), (int(u[1]), int(v[1])), (0, 255, 0), 1); cv2.putText(ir, '%.0f m' % xm, (int(u[1]) + 4, int(v[1]) - 2), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 0), 1)
        # --- depth preview (grey) + pair overlay
        g = (255 - 215 * np.clip(depth, 0, 6.) / 6.).astype(np.uint8); g[~((depth > 0) & np.isfinite(depth))] = 0
        dv = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
        if cand.get('ok') and not goal.get('ok'):
            for u_ in (cand['u_left'], cand['u_right']):
                for img in (ir, dv): cv2.line(img, (int(u_), 0), (int(u_), h - 1), (0, 140, 255), 1)
            for img in (ir, dv): cv2.line(img, (int(cand['u_goal']), 0), (int(cand['u_goal']), h - 1), (0, 140, 255), 2); cv2.circle(img, (int(cand['u_goal']), h // 2), 9, (0, 140, 255), 2)
        if goal.get('ok'):
            for u_ in (goal['u_left'], goal['u_right']):
                for img in (ir, dv): cv2.line(img, (int(u_), 0), (int(u_), h - 1), (0, 220, 255), 2)
            for img in (ir, dv): cv2.line(img, (int(goal['u_goal']), 0), (int(goal['u_goal']), h - 1), (0, 0, 255), 2)
            txt = 'entrance %.2f+-%.2f m: centre %+.3f m (left+, camera) width %.2f m at %.2f m' % (2 * half_truck, self.width_tol, goal['goal_lateral_m'], goal['width_m'], (goal['z_left_m'] + goal['z_right_m']) / 2)
        else:
            txt = 'entrance %.2f+-%.2f m: %s | nearest opening (orange): %s' % (2 * half_truck, self.width_tol, goal.get('reason'), ('width %.2f m centre %+.3f m at %.2f m' % (cand['width_m'], cand['goal_lateral_m'], (cand['z_left_m'] + cand['z_right_m']) / 2)) if cand.get('ok') else cand.get('reason'))
        line1, line2 = (txt.split(' | ', 1) + [''])[:2]
        for img in (ir, dv):
            cv2.rectangle(img, (0, h - 40), (w - 1, h - 1), (0, 0, 0), -1)
            cv2.putText(img, line1, (8, h - 24), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 220, 255) if goal.get('ok') else (0, 165, 255), 1)
            cv2.putText(img, line2, (8, h - 7), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 140, 255), 1)
        # --- bird's-eye render: 160 rows x 120 cols -> x4
        bev = np.zeros(grid.shape + (3,), np.uint8); occ = np.clip(grid * 40, 0, 255).astype(np.uint8); bev[..., :] = occ[..., None]
        S = 4; bev = cv2.resize(bev, (grid.shape[1] * S, grid.shape[0] * S), interpolation=cv2.INTER_NEAREST)
        def P(x, y):
            r, cc = xy_grid(x, y, FWD, HALF, CELL); return (int(cc * S + S / 2), int(r * S + S / 2))
        for xm in range(1, int(FWD)):  # range rings (lines)
            cv2.line(bev, P(xm, HALF - .01), P(xm, -HALF + .01), (50, 50, 50), 1); cv2.putText(bev, '%d' % xm, (4, P(xm, 0)[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, .4, (120, 120, 120), 1)
        tip = site['camera_to_fork_tip_m']; rear = -site['rear_extent_m']
        cv2.rectangle(bev, P(min(tip, FWD - .01), half_vehicle), P(max(rear, 0.), -half_vehicle), (0, 220, 0), 1)   # vehicle (clipped to the grid)
        cv2.line(bev, P(tip, half_vehicle), P(FWD - .01, half_vehicle), (0, 120, 0), 1); cv2.line(bev, P(tip, -half_vehicle), P(FWD - .01, -half_vehicle), (0, 120, 0), 1)
        cv2.line(bev, P(tip, half_truck), P(FWD - .01, half_truck), (255, 200, 0), 1); cv2.line(bev, P(tip, -half_truck), P(FWD - .01, -half_truck), (255, 200, 0), 1)
        cv2.circle(bev, P(0., t[1]), 3, (255, 255, 255), -1)   # camera
        if goal.get('ok'):
            for u_, z_, col in ((goal['u_left'], goal['z_left_m'], (0, 220, 255)), (goal['u_right'], goal['z_right_m'], (0, 220, 255)), (goal['u_goal'], (goal['z_left_m'] + goal['z_right_m']) / 2, (0, 0, 255))):
                pc = np.array([[(u_ - intr['cx']) / intr['fx'] * z_, 0., z_]]); pb = camera_to_body(pc, R, t)[0]
                if 0 <= pb[0] < FWD and abs(pb[1]) < HALF: cv2.circle(bev, P(pb[0], pb[1]), 5, col, 2)
        n += 1; fps = n / max(1e-6, time.monotonic() - t0)
        cv2.rectangle(bev, (0, bev.shape[0] - 18), (bev.shape[1] - 1, bev.shape[0] - 1), (0, 0, 0), -1); cv2.putText(bev, 'VEHICLE view | 5 cm cells | %.1f fps | %d pts | %s' % (fps, len(body), src), (6, bev.shape[0] - 5), cv2.FONT_HERSHEY_SIMPLEX, .4, (200, 200, 200), 1)
        # --- entrance-fixed view: rotate the world so the depth-found entrance sits on a fixed line
        src_pair = goal if goal.get('ok') else (cand if cand.get('ok') else None); confirmed = bool(goal.get('ok'))
        if src_pair is not None:
            def edges(g):
                pl = camera_to_body(np.array([[(g[0] - intr['cx']) / intr['fx'] * g[2], 0., g[2]]]), R, t)[0]
                pr = camera_to_body(np.array([[(g[1] - intr['cx']) / intr['fx'] * g[3], 0., g[3]]]), R, t)[0]
                return pl[:2], pr[:2]
            raw_pair = (src_pair['u_left'], src_pair['u_right'], src_pair['z_left_m'], src_pair['z_right_m'])
            rp, rh = vehicle_in_entrance(entrance_frame(*edges(raw_pair))); self.raw.append((rp[1], math.degrees(rh)))
            self.pairs.append(raw_pair); med = tuple(float(np.median([q[i] for q in self.pairs])) for i in range(4))   # running median (~0.7 s)
            frame = entrance_frame(*edges(med)); held = time.monotonic()
        else:
            self.pairs.clear()
        if frame is not None and time.monotonic() - held < 3.:
            EX0, EX1, EH = -6., 2., 3.    # entrance frame: x from 6 m before the entrance to 2 m inside, +-3 m across
            pe = to_entrance(body[:, :2], frame)
            keep = (body[:, 2] >= .05) & (body[:, 2] <= 1.6) & (pe[:, 0] >= EX0) & (pe[:, 0] < EX1) & (np.abs(pe[:, 1]) < EH)
            rows_e = int((EX1 - EX0) / CELL); cols_e = int(2 * EH / CELL); ge = np.zeros((rows_e, cols_e), np.int32)
            if keep.any():
                rr = rows_e - 1 - ((pe[keep, 0] - EX0) / CELL).astype(int); cc = ((EH - pe[keep, 1]) / CELL).astype(int)
                np.add.at(ge, (np.clip(rr, 0, rows_e - 1), np.clip(cc, 0, cols_e - 1)), 1)
            ge = ge.astype(np.float32); self.acc_e = ge if self.acc_e is None or self.acc_e.shape != ge.shape else self.acc_e * self.decay + ge; ge = self.acc_e
            ev = np.zeros((rows_e, cols_e, 3), np.uint8); ev[..., :] = np.clip(ge * 40, 0, 255).astype(np.uint8)[..., None]
            ev = cv2.resize(ev, (cols_e * S, rows_e * S), interpolation=cv2.INTER_NEAREST)
            def Q(x, y): return (int(((EH - y) / CELL) * S + S / 2), int((rows_e - 1 - (x - EX0) / CELL) * S + S / 2))
            cv2.line(ev, Q(0., EH - .01), Q(0., -EH + .01), (0, 220, 255), 2)                                   # entrance line
            cv2.line(ev, Q(0., half_truck), Q(EX1 - .01, half_truck), (255, 200, 0), 2); cv2.line(ev, Q(0., -half_truck), Q(EX1 - .01, -half_truck), (255, 200, 0), 2)
            cv2.line(ev, Q(EX0 + .01, 0.), Q(EX1 - .01, 0.), (0, 0, 255), 1)                                    # truck axis
            for xm in range(int(EX0), int(EX1)):
                cv2.line(ev, Q(xm, EH - .01), Q(xm, -EH + .01), (50, 50, 50), 1); cv2.putText(ev, '%+d' % xm, (4, Q(xm, 0)[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, .4, (120, 120, 120), 1)
            corners = np.array([[tip, half_vehicle], [tip, -half_vehicle], [rear, -half_vehicle], [rear, half_vehicle]]); ce = to_entrance(corners, frame)
            poly = np.array([Q(x, y) for x, y in ce], np.int32); cv2.polylines(ev, [poly], True, (0, 220, 0), 2)
            cv2.line(ev, Q(*to_entrance(np.array([[tip, 0.]]), frame)[0]), Q(*to_entrance(np.array([[tip + 3., 0.]]), frame)[0]), (0, 120, 0), 1)   # vehicle axis
            pos, hdg = vehicle_in_entrance(frame); tipe = to_entrance(np.array([[tip, 0.]]), frame)[0]
            cv2.rectangle(ev, (0, ev.shape[0] - 18), (ev.shape[1] - 1, ev.shape[0] - 1), (0, 0, 0), -1); cv2.putText(ev, 'ENTRANCE view%s (10-frame median) | lateral %+.3f m heading %+.1f deg | fork tip %.2f m before the entrance%s | raw jitter sd %s' % ('' if confirmed else ' [CANDIDATE, width not matched]', pos[1], math.degrees(hdg), -tipe[0], '' if src_pair is not None else ' | HELD', ('%.3f m / %.2f deg (n=%d)' % (np.std([r[0] for r in self.raw]), np.std([r[1] for r in self.raw]), len(self.raw))) if len(self.raw) >= 5 else '-'), (6, ev.shape[0] - 5), cv2.FONT_HERSHEY_SIMPLEX, .4, (200, 200, 200), 1)
        else:
            ev = np.zeros((int(8. / CELL) * S, int(6. / CELL) * S, 3), np.uint8); cv2.putText(ev, 'ENTRANCE view: no entrance pair', (6, 14), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 165, 255), 1)
        def fit(img):
            r = cv2.resize(img, (int(img.shape[1] * h / img.shape[0]), h)); canvas = np.zeros((h, w, 3), np.uint8); x0 = (w - r.shape[1]) // 2 if r.shape[1] <= w else 0
            canvas[:, x0:x0 + min(w, r.shape[1])] = r[:, :min(w, r.shape[1])]; return canvas
        out = np.vstack([np.hstack([ir, dv]), np.hstack([fit(bev), fit(ev)])])
        self.frame, self.held, self.n = frame, held, n
        return out, goal


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8082); ap.add_argument('--emitter', action='store_true', help='IR projector on (better depth on plain surfaces; never with the VIO running)')
    ap.add_argument('--save', help='write a combined PNG here every --every seconds'); ap.add_argument('--every', type=float, default=2.)
    ap.add_argument('--pitch', type=float, default=None, help='camera pitch down [deg] if configs/mount.json is absent'); ap.add_argument('--mode', default='left')
    ap.add_argument('--step', type=int, default=2, help='pixel subsampling for the point cloud'); ap.add_argument('--fps', type=int, default=15)
    ap.add_argument('--no-filter', action='store_true', help='skip the RealSense spatial+temporal post-processing (raw per-frame depth)'); ap.add_argument('--decay', type=float, default=.7, help="bird's-eye accumulation: old*decay + new (0 = per frame)")
    ap.add_argument('--width', type=float, default=None, help='entrance inner width to look for [m] (default: truck_inner_width_m / bay_width_m from common.json)'); ap.add_argument('--width-tol', type=float, default=.2)
    a = ap.parse_args()
    cfg = cfgmod.load(mode=a.mode, require_mount=False); site = cfg['site']; cam = cfg['sensor']['camera']
    width_in = site.get('truck_inner_width_m', site.get('bay_width_m', 1.4)); half_truck = width_in / 2; half_vehicle = site['vehicle_width_m'] / 2
    calib = cfg['sensor'].get('calibration') or {}
    if calib.get('T_body_camera') is not None:
        T = np.array(calib['T_body_camera']); R = T[:3, :3]; src = 'mount.json'
    else:
        R = mount_rotation(a.pitch or 0.); src = 'fallback pitch %.1f deg' % (a.pitch or 0.)
    t = np.array([0., site['camera_lateral_m'], site['camera_height_m']])
    print('rotation source:', src, '| camera height %.2f lateral %+.3f fork tip %.2f' % (t[2], t[1], site['camera_to_fork_tip_m']), flush=True)

    import pyrealsense2 as rs
    pipe = rs.pipeline(); c = rs.config()
    c.enable_stream(rs.stream.infrared, 1, cam['ir_width'], cam['ir_height'], rs.format.y8, a.fps)
    c.enable_stream(rs.stream.depth, cam['ir_width'], cam['ir_height'], rs.format.z16, a.fps)
    prof = pipe.start(c)
    for s in prof.get_device().query_sensors():
        if s.supports(rs.option.emitter_enabled): s.set_option(rs.option.emitter_enabled, 1. if a.emitter else 0.)
        if s.supports(rs.option.enable_auto_exposure): s.set_option(rs.option.enable_auto_exposure, 1.)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    filters = []
    if not a.no_filter:
        try:
            sp = rs.spatial_filter(); sp.set_option(rs.option.filter_magnitude, 2); sp.set_option(rs.option.filter_smooth_alpha, .5); sp.set_option(rs.option.filter_smooth_delta, 20)
            tp = rs.temporal_filter(); tp.set_option(rs.option.filter_smooth_alpha, .4); tp.set_option(rs.option.filter_smooth_delta, 20)
            filters = [sp, tp]; print('depth post-processing: spatial + temporal (RealSense SDK). --no-filter for raw frames', flush=True)
        except Exception as exc:
            print('depth filters unavailable (%s): raw frames' % exc, flush=True)
    di = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    intr = dict(fx=di.fx, fy=di.fy, cx=di.ppx, cy=di.ppy)
    print('depth intrinsics', {k: round(v, 1) for k, v in intr.items()}, 'scale', scale, 'emitter', a.emitter, flush=True)
    srv = ThreadingHTTPServer(('0.0.0.0', a.port), Handler); threading.Thread(target=srv.serve_forever, daemon=True).start()
    print('화면: SSH 포트 전달 후 http://127.0.0.1:%d   (VS Code 터미널이면 이 주소를 클릭 — 도킹 화면과 같은 방식. Ctrl+C 로 종료)' % a.port, flush=True)
    viewer = Viewer(site, R, t, intr, src); viewer.width_tol = a.width_tol; viewer.decay = max(0., min(.95, a.decay)); last_save = 0.
    if a.width: viewer.half_truck = a.width / 2
    print('looking for an entrance %.2f +- %.2f m wide' % (2 * viewer.half_truck, viewer.width_tol), flush=True)
    try:
        while True:
            try:
                fs = pipe.wait_for_frames(5000)
            except RuntimeError as exc:
                print('no frames for 5 s: %s (is the docking app or another process holding the camera?)' % exc, flush=True); continue
            dfr = fs.get_depth_frame(); ifr = fs.get_infrared_frame(1)
            if not dfr or not ifr: continue
            for f in filters:
                try: dfr = f.process(dfr)
                except Exception as exc: print('filter failed (%s): disabling' % exc, flush=True); filters = []; break
            depth = np.asanyarray(dfr.get_data()).astype(np.float32) * scale
            ir = np.asanyarray(ifr.get_data())
            out, goal = viewer.compose(depth, ir, a.step)
            ok, buf = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 70]); LATEST['jpg'] = bytes(buf) if ok else None
            LATEST['n'] += 1; LATEST['t'] = time.monotonic(); LATEST['txt'] = ('entrance ok: centre %+.3f width %.2f' % (goal['goal_lateral_m'], goal['width_m'])) if goal.get('ok') else 'entrance: ' + str(goal.get('reason'))
            if LATEST['n'] % 75 == 1: print('frames %d | %s' % (LATEST['n'], LATEST['txt']), flush=True)
            if a.save and time.monotonic() - last_save >= a.every:
                Path(a.save).mkdir(parents=True, exist_ok=True); cv2.imwrite(str(Path(a.save) / ('bev_%s.png' % time.strftime('%H%M%S'))), out); last_save = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        pipe.stop()

if __name__ == '__main__':
    main()
