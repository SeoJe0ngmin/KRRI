#!/usr/bin/env python3
"""Tag check for the v4 inner tags (2026-11-01). Standalone: opens the RealSense colour stream itself (stop the docking app first)
or reads a folder of images. Nothing here drives the vehicle.

  source scripts/environment.sh && "$DOCK_PYTHON" scripts/tag_test.py [--port 8083] [--seconds 60] [--save DIR]
  "$DOCK_PYTHON" scripts/tag_test.py --images DIR [--device-json work/<run>/device.json]

Every ~0.3 s one line: the entrance tag (range, bearing), the inner tags seen, the bundle (range to the tag plane, lateral of the
lane centre in the body frame = left +, heading vs the lane axis = left +, reprojection px, AMBIGUOUS when the head-on twin fits
as well) and what the fusion would call the landmark at that range (bundle within inner_bundle_max_range_m, else single tags).
With --dist (tape: camera -> tag plane) and --lateral (tape: camera left of the lane centre) the differences are printed too.
http://<jetson>:8083/ shows the colour image with the tags drawn. Uses configs/common.json (tag ids/sizes/positions, camera
mount) and configs/mount.json when present; the colour camera's offset from the left IR imager comes from the device (live) or is
assumed zero (images). A bundle reprojection error that grows with the distance means inner_tag_spacing_m is off.
"""

import argparse
import glob
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import numpy as np
import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from docking import config as cfgmod
from docking.tag import TagObserver
from docking.observer import inner_body
from docking.fusion import tag_bearing_range

LATEST = {'jpg': None, 'txt': '', 'n': 0}


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
        if self.path.startswith('/stat'):
            page = ('frames %d | %s' % (LATEST['n'], LATEST['txt'])).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.send_header('Content-Length', str(len(page)))
            self.end_headers()
            self.wfile.write(page)
            return
        page = (
            b'<!doctype html><html><head><meta charset="utf-8"><title>tag test</title><style>body{margin:0;background:#111;color:#ccc;font:14px sans-serif}img{width:100%;max-width:1280px;display:block}p{padding:6px 10px;margin:0}</style></head>'
            b'<body><p id="s">...</p><img id="v" src="/snap"><script>setInterval(()=>{document.getElementById("v").src="/snap?"+Date.now();fetch("/stat").then(r=>r.text()).then(t=>{document.getElementById("s").textContent=t})},300)</script></body></html>'
        )
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(page)))
        self.end_headers()
        self.wfile.write(page)


def draw(img, out, text):
    vis = img.copy() if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if out.get('corners'):
        cv2.polylines(vis, [np.array(out['corners'], int)], True, (0, 255, 0) if out['ok'] else (0, 165, 255), 2)
    for tid, t in ((out.get('inner') or {}).get('tags') or {}).items():
        if t.get('corners'):
            pts = np.array(t['corners'], int)
            cv2.polylines(vis, [pts], True, (0, 255, 0) if t.get('ok') else (0, 165, 255), 2)
            cv2.putText(
                vis,
                'id %s' % tid,
                (int(pts[:, 0].min()), int(pts[:, 1].min()) - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                1,
            )
    cv2.rectangle(vis, (0, vis.shape[0] - 30), (vis.shape[1] - 1, vis.shape[0] - 1), (0, 0, 0), -1)
    cv2.putText(vis, text[:150], (8, vis.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return vis


def describe(out, cfg, T_body_colour, dist=None, lateral=None):
    parts = []
    if out.get('ok'):
        b, r = tag_bearing_range(np.array(out['T_camera_tag']), T_body_colour)
        parts.append('entrance %.2f m %+.1f deg' % (r, math.degrees(b)))
    else:
        parts.append('entrance -' if not out.get('detected') else 'entrance %s' % out['reason'])
    inner = out.get('inner')
    if inner is None:
        parts.append('inner tags not configured')
        return ' | '.join(parts)
    s = inner_body(inner, T_body_colour, out.get('stamp', 0.0))
    bd = s.get('bundle') or {}
    parts.append('inner %s' % (','.join(str(i) for i in s['seen']) or '-'))
    if bd.get('ok'):
        kind = (
            'BUNDLE'
            if bd['x_body_m'] <= float(cfg['tuning'].get('inner_bundle_max_range_m', 3.5))
            else 'single tags (bundle too far)'
        )
        parts.append(
            'plane %.3f m | centre %+.3f m | heading %+.2f deg | reproj %.2f px%s | %s'
            % (
                bd['x_body_m'],
                bd['lateral_body_m'],
                bd['heading_err_deg'],
                bd.get('reproj_px') or 0.0,
                ' AMBIGUOUS' if bd.get('ambiguous') else '',
                kind,
            )
        )
        if dist is not None:
            parts.append('d plane %+.3f m' % (bd['x_body_m'] - dist))
        if lateral is not None:
            parts.append('d centre %+.3f m' % (bd['lateral_body_m'] - (-lateral)))
    elif s.get('single'):
        sg = s['single']
        parts.append(
            'tag %d alone: %.2f m %+.1f deg (%s)'
            % (sg['id'], sg['x_body_m'], sg['bearing_deg'], s.get('bundle_reason'))
        )
    else:
        parts.append('bundle: %s' % s.get('bundle_reason'))
    return ' | '.join(parts)


def main():
    ap = argparse.ArgumentParser(description='inner tag check (standalone, no CAN)')
    ap.add_argument('--common', default=str(cfgmod.COMMON))
    ap.add_argument('--port', type=int, default=8083)
    ap.add_argument('--seconds', type=float, default=0.0)
    ap.add_argument('--images', help='folder of colour images instead of the camera')
    ap.add_argument('--device-json', help='work/<run>/device.json for the colour intrinsics (images)')
    ap.add_argument('--save', help='folder for annotated JPEGs')
    ap.add_argument('--dist', type=float, help='tape: camera -> tag plane [m]')
    ap.add_argument('--lateral', type=float, help='tape: camera left of the lane centre [m], left +')
    a = ap.parse_args()
    cfg = cfgmod.load(a.common, require_mount=False)
    sensor = cfg['sensor']
    if not sensor['tag'].get('inner'):
        print('configs/common.json: inner_tag_ids 가 비어 있습니다 (안쪽 태그 끔)')
    T_body_ir = np.array(sensor['calibration']['T_body_camera'], float)
    T_body_colour = T_body_ir.copy()
    detector = TagObserver(sensor)
    model = None
    pipe = None
    frames = []
    if a.images:
        frames = sorted(glob.glob(os.path.join(a.images, '*.*')))
        if a.device_json:
            import json

            model = json.loads(Path(a.device_json).read_text())['raw_models']['color']
        else:
            print('[images] no --device-json: assuming the D435 colour 1280x720 nominal intrinsics (fx 924)')
    else:
        import pyrealsense2 as rs

        pipe = rs.pipeline()
        rc = rs.config()
        cam = sensor['camera']
        rc.enable_stream(rs.stream.color, cam['color_width'], cam['color_height'], rs.format.bgr8, cam['fps'])
        rc.enable_stream(rs.stream.infrared, 1, cam['ir_width'], cam['ir_height'], rs.format.y8, cam['fps'])
        prof = pipe.start(rc)
        cp = prof.get_stream(rs.stream.color).as_video_stream_profile()
        i = cp.get_intrinsics()
        model = dict(fx=i.fx, fy=i.fy, cx=i.ppx, cy=i.ppy, width=i.width, height=i.height, coeffs=list(i.coeffs))
        try:  # the colour imager sits beside the left IR imager: body <- colour = (body <- left IR) (left IR <- colour)
            e = cp.get_extrinsics_to(prof.get_stream(rs.stream.infrared, 1))
            T = np.eye(4)
            T[:3, :3] = np.array(e.rotation).reshape(3, 3, order='F')
            T[:3, 3] = e.translation
            T_body_colour = T_body_ir @ T
        except Exception as exc:
            print('colour->IR extrinsics unavailable (%s): using the IR mount for the colour camera' % exc)
    if model is None:
        model = dict(fx=924.0, fy=924.0, cx=640.0, cy=360.0, width=1280, height=720, coeffs=[0.0] * 5)
    server = ThreadingHTTPServer(('0.0.0.0', a.port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print('화면: http://127.0.0.1:%d  (Ctrl+C 로 끝)' % a.port)
    if a.save:
        os.makedirs(a.save, exist_ok=True)
    t0 = time.monotonic()
    k = 0
    last = 0.0
    try:
        while True:
            if pipe is not None:
                fs = pipe.wait_for_frames()
                cf = fs.get_color_frame()
                if not cf:
                    continue
                img = np.asanyarray(cf.get_data())
                stamp = cf.get_timestamp() / 1000.0
            else:
                if k >= len(frames):
                    break
                img = cv2.imread(frames[k])
                stamp = float(k)
                k += 1
                if img is None:
                    continue
            out = detector.process(img, model, stamp, 1)
            text = describe(out, cfg, T_body_colour, a.dist, a.lateral)
            LATEST['n'] += 1
            LATEST['txt'] = text
            vis = draw(img, out, text)
            ok, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 70])
            LATEST['jpg'] = bytes(buf) if ok else None
            if a.save:
                cv2.imwrite(os.path.join(a.save, '%06d.jpg' % LATEST['n']), vis)
            if time.monotonic() - last >= 0.3 or pipe is None:
                print(text, flush=True)
                last = time.monotonic()
            if a.seconds and time.monotonic() - t0 >= a.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if pipe is not None:
            pipe.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
