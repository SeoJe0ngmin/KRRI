"""Read-only loopback dashboard. Browser and JPEG never block control."""
import json
from .serialization import numpy_json
import threading
import time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer

HTML='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>Docking v6</title>
<style>body{background:#101923;color:#e5ecf5;font:16px system-ui;margin:24px}.grid{display:flex;gap:20px}img{width:640px;max-width:60vw}pre{white-space:pre-wrap}canvas{background:#172634}#status{color:#ffcf75}</style>
<h1>Docking v6 · 관측 기반 재계획</h1><p>준비 완료 후 SSH 터미널 Enter · 정지 s+Enter / Ctrl+C · 읽기 전용 화면</p><p id="status"></p><p id="measures"></p>
<div class="grid"><div><img src="/image"><br><canvas id="map" width="640" height="400"></canvas><p>평면도: 실선=현재 추정 차량 / 회색 점선=중심선 / 노란 선=계획 후보. 실제 정확도 보장 아님.</p></div><pre id="info"></pre></div>
<script>let canvas=document.getElementById('map'),c=canvas.getContext('2d');async function tick(){try{let d=await(await fetch('/state')).json(),o=d.observation||{},s=d.controller||{};document.getElementById('status').textContent=s.state+' / '+(s.reason||'')+' / '+(d.age_s==null?'관측 없음':d.age_s.toFixed(2)+'s 경과');let g=d.geometry;document.getElementById('measures').textContent=g?'포크 목표까지 '+g.remaining_m.toFixed(3)+'m / 중심선 좌우 '+g.lateral_m.toFixed(3)+'m / 방향 '+g.heading_deg.toFixed(2)+'° / 예상 도착 좌우 '+(g.predicted_arrival_y_m==null?'—':g.predicted_arrival_y_m.toFixed(3)+'m'):'위치 기준 수집 중';document.getElementById('info').textContent=JSON.stringify({control:s,pose:o.pose,confidence:o.confidence_radius,confidence_kind:o.confidence_kind,fork:o.fork_uncertainty,tag:o.tag,odom:o.odom&&Object.fromEntries(Object.entries(o.odom).filter(([k])=>k!=='tracked_points')),performance:o.performance},null,2);c.clearRect(0,0,640,400);if(o.pose){let [x,y,h]=o.pose,site=d.cfg_view.site,scale=35,minx=Math.min(-8,x-2),p=(a,b)=>[40+(a-minx)*scale,200-b*scale];c.strokeStyle='#94a4b8';c.setLineDash([5,5]);c.beginPath();c.moveTo(...p(minx,0));c.lineTo(...p(2,0));c.stroke();c.setLineDash([]);if(site.bay_width_m){c.strokeStyle='#777';for(let k of [-1,1]){c.beginPath();c.moveTo(...p(0,k*site.bay_width_m/2));c.lineTo(...p(3,k*site.bay_width_m/2));c.stroke()}}if(s.route_points&&s.route_points.length){c.strokeStyle='#e4c76b';c.setLineDash([4,4]);c.beginPath();s.route_points.forEach((v,i)=>i?c.lineTo(...p(v[0],v[1])):c.moveTo(...p(v[0],v[1])));c.stroke();c.setLineDash([])}c.save();c.translate(...p(x,y));c.rotate(-h);c.fillStyle='#71b7ff';let w=site.vehicle_width_m||1.5;c.fillRect(-site.rear_extent_m*scale,-w/2*scale,(site.camera_to_fork_tip_m+site.rear_extent_m)*scale,w*scale);c.restore();c.fillStyle='#45ddc0';c.beginPath();c.arc(...p(d.cfg_view.target,0),5,0,7);c.fill()}}
catch(e){document.getElementById('status').textContent='연결 끊김'}setTimeout(tick,250)}tick();let img=document.querySelector('img');setInterval(()=>img.src='/image?t='+Date.now(),200);</script></html>'''

def encode_preview(image,obs,cfg):
    import cv2
    out=image.copy()
    if out.ndim==2:out=cv2.cvtColor(out,cv2.COLOR_GRAY2BGR)
    import numpy as np
    if obs['tag'].get('stream')=='infrared' and obs['tag'].get('corners'):cv2.polylines(out,[np.array(obs['tag']['corners'],int)],True,(0,255,0) if obs['tag']['ok'] else (0,200,255),2)
    for p in obs['odom'].get('tracked_points',[]):cv2.circle(out,tuple(map(int,p)),2,(255,200,0),-1)
    cv2.putText(out,'tag: '+obs['tag']['reason']+' / odom: '+obs['odom']['reason'],(10,24),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,220,255),1)
    scale=min(1.,cfg['width']/out.shape[1]);out=cv2.resize(out,None,fx=scale,fy=scale)
    ok,buf=cv2.imencode('.jpg',out,[cv2.IMWRITE_JPEG_QUALITY,cfg['jpeg_quality']])
    return bytes(buf) if ok else b''

class Dashboard:
    def __init__(self,port):
        self.lock=threading.Lock();self.value={};self.jpeg=b'';owner=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*a):pass
            def do_GET(self):
                with owner.lock:value=owner.value;image=owner.jpeg
                if self.path.startswith('/state'):
                    o=value.get('observation') or {};body=json.dumps(dict(value,age_s=None if 'arrival' not in o else time.monotonic()-o['arrival']),default=numpy_json).encode();kind='application/json'
                elif self.path.startswith('/image'):body=image;kind='image/jpeg'
                else:body=HTML.encode();kind='text/html; charset=utf-8'
                try:
                    self.connection.settimeout(.5);self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
                except (OSError,TimeoutError):pass
        self.server=ThreadingHTTPServer(('127.0.0.1',port),Handler);self.server.daemon_threads=True
        threading.Thread(target=self.server.serve_forever,daemon=True).start()
    def update(self,value,preview):
        with self.lock:
            self.value=value
            if preview:self.jpeg=preview[1]
    def close(self):self.server.shutdown();self.server.server_close()
