"""Read-only, localhost-only MJPEG dashboard, intended for an SSH tunnel."""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

HTML = '''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Robust docking · Nano</title><style>
body{background:#101923;color:#e5ecf5;font:16px system-ui;margin:24px}h1{font-size:24px}
.grid{display:grid;grid-template-columns:minmax(400px,2fr) minmax(300px,1fr);gap:20px}img,canvas{width:100%;background:#192734;border-radius:10px}
pre{white-space:pre-wrap;font:13px monospace}.pill{padding:8px 14px;background:#294052;border-radius:6px}#age{color:#ffbf74}
@media(max-width:850px){.grid{display:block}}</style>
<h1>Robust docking <span class="pill" id="mode">연결 중</span></h1>
<p>주행 시작: SSH 터미널 Enter · 정지: Ctrl+C / s + Enter · 화면은 읽기 전용입니다.</p>
<p id="preparation" hidden></p>
<p id="age">관측을 기다리는 중</p><div class="grid"><div><img src="/stream.mjpg" alt="Nano 카메라">
<canvas id="map" width="960" height="430"></canvas><p>평면도: 파랑=차량 / 청록=목표 영역 / 노랑=다음 정지 영역 · 미보정 위치는 표시하지 않습니다.</p></div>
<div><pre id="info"></pre></div></div><script>
const can=document.getElementById('map'),c=can.getContext('2d');let last=Date.now();
function map(d){c.clearRect(0,0,960,430);let cfg=d.geometry;if(!cfg)return;
let rs=cfg.scene.free_rectangles,lo=Math.min(...rs.map(r=>r[0])),hi=Math.max(...rs.map(r=>r[1])),
ylo=Math.min(...rs.map(r=>r[2])),yhi=Math.max(...rs.map(r=>r[3]));
let sc=Math.min(870/(hi-lo),340/(yhi-ylo));function p(x,y){return[45+(x-lo)*sc,385-(y-ylo)*sc]}
function box(b,col){let a=p(b.lo[0],b.hi[1]),z=p(b.hi[0],b.lo[1]);c.strokeStyle=col;c.lineWidth=2;c.strokeRect(a[0],a[1],z[0]-a[0],z[1]-a[1])}
rs.forEach(r=>box({lo:[r[0],r[2]],hi:[r[1],r[3]]},'#62768a'));box(cfg.goal,'#45ddc0');
if(d.next_stop)box(d.next_stop,'#ffc36e');let o=d.observation;if(!o||!o.valid)return;
let [x,y,h]=o.pose,q=p(x,y);c.save();c.translate(...q);c.rotate(-h);c.fillStyle='#71b7ff';
c.fillRect(-cfg.body.rear_m*sc,-cfg.body.half_width_m*sc,(cfg.body.front_m+cfg.body.rear_m)*sc,2*cfg.body.half_width_m*sc);
c.strokeStyle='white';c.beginPath();c.moveTo(0,0);c.lineTo(cfg.body.front_m*sc+14,0);c.stroke();c.restore();
if(o.error)box({lo:[x-o.error[0],y-o.error[1]],hi:[x+o.error[0],y+o.error[1]]},'#ffbf74')}
async function tick(){try{let d=await(await fetch('/status',{cache:'no-store'})).json();last=Date.now();
document.getElementById('mode').textContent=(d.experiment?.folder||d.experiment?.variant||'')+' / '+d.mode+' / '+(d.controller?.state||'MEASURE');
let prep=d.controller?.startup_measurement,prepLine=document.getElementById('preparation');prepLine.hidden=!prep?.enabled;
if(prep?.enabled)prepLine.textContent=d.controller?.state==='FAULT'?'준비/주행 중단 — 터미널에서 원인을 확인하세요.':
prep.complete?'출발 전 정지 측정 완료':d.controller?.state==='WAIT_ENTER'?'터미널 Enter로 준비 측정을 시작합니다.':
'출발 전 정지 측정 '+prep.stationary_s.toFixed(1)+' / '+prep.required_s.toFixed(0)+'초 — 차량을 정지 상태로 유지하세요. 완료 후 태그 확보/주행 조건을 확인합니다.';
document.getElementById('age').textContent='관측 경과 '+(d.observation_age_s??0).toFixed(2)+' s / 화면 경과 '+(d.preview_age_s??0).toFixed(2)+' s'+(d.stale?' — STALE, 최신 화면이 아닙니다':'');
let o=d.observation||{};document.getElementById('info').textContent=JSON.stringify({control:d.controller,pose:o.pose,error:o.error,bounded:o.bounded,camera_odometry:o.camera_odometry,tag_fusion:o.tag_fusion,error_budget:o.error_budget,camera:o.camera,tag:o.tag,tag_position_window:o.tag_position_window,odom:o.odom,imu:o.imu,performance:o.performance,recording:o.recording,control_recording:d.control_recording,can:d.can},null,2);map(d)}
catch(e){document.getElementById('age').textContent='연결 끊김 — 마지막 화면입니다';}setTimeout(tick,250)}tick();
</script></html>'''


def encode_preview(image, obs, cfg):
    import cv2
    import numpy as np
    canvas = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image.copy()
    corners = obs['tag'].get('corners')
    if corners:
        color = (120, 255, 140) if obs['tag']['ok'] else (40, 170, 255)
        cv2.polylines(canvas, [np.array(corners, np.int32)], True, color, 2)
    for p in obs['odom'].get('tracked_points', []):
        cv2.circle(canvas, (int(p[0]), int(p[1])), 2, (255, 200, 80), -1)
    width = min(canvas.shape[1], cfg['width'])
    if width != canvas.shape[1]:
        canvas = cv2.resize(canvas, (width, int(canvas.shape[0]*width/canvas.shape[1])))
    text = 'tag: %s / odom: %s' % (obs['tag']['reason'], obs['odom']['reason'])
    cv2.putText(canvas, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .45, (30, 230, 255), 1)
    ok, data = cv2.imencode('.jpg', canvas, [cv2.IMWRITE_JPEG_QUALITY, cfg['jpeg_quality']])
    return data.tobytes() if ok else None


class Dashboard:
    def __init__(self, port):
        self.lock = threading.Lock()
        self.jpeg = None
        self.image_time = None
        self.status = {}
        self.closed = threading.Event()
        parent = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass
            def do_GET(self):
                self.connection.settimeout(2)
                if self.path == '/':
                    self.reply('text/html; charset=utf-8', HTML.encode('utf8'))
                elif self.path == '/status':
                    with parent.lock:
                        value = dict(parent.status)
                        it = parent.image_time
                    now = time.monotonic()
                    ot = (value.get('observation') or {}).get('arrival')
                    value.update(observation_age_s=None if ot is None else now-ot,
                                 preview_age_s=None if it is None else now-it,
                                 stale=ot is None or now-ot > .5)
                    self.reply('application/json', json.dumps(value, allow_nan=False).encode())
                elif self.path == '/stream.mjpg':
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    last = None
                    try:
                        while not parent.closed.wait(.1):
                            with parent.lock:
                                jpeg, stamp = parent.jpeg, parent.image_time
                            if jpeg is None or stamp == last:
                                continue
                            last = stamp
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n')
                            self.wfile.flush()
                    except (OSError, TimeoutError):
                        pass
                else:
                    self.send_error(404)
            def reply(self, kind, data):
                self.send_response(200)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try:
                    self.wfile.write(data)
                except OSError:
                    pass
        class Server(ThreadingMixIn, HTTPServer):
            daemon_threads = True
        self.server = Server(('127.0.0.1', port), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def update(self, status, preview=None):
        with self.lock:
            self.status = status
            if preview:
                self.image_time, self.jpeg = preview

    def close(self):
        self.closed.set()
        self.server.shutdown()
        self.server.server_close()
