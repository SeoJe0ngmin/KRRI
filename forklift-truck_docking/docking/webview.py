"""Read-only loopback dashboard. Browser and JPEG never block control."""
import json
from .serialization import numpy_json
import threading
import time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer

HTML='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>Docking v6</title>
<style>body{background:#101923;color:#e5ecf5;font:16px system-ui;margin:24px}.grid{display:flex;gap:20px;flex-wrap:wrap}img{width:640px;max-width:60vw}pre{white-space:pre-wrap;font-size:13px;max-width:520px}canvas{background:#172634}#status{color:#ffcf75}#fusion{color:#9fd8ff}small{color:#94a4b8}</style>
<h1>Docking v6 · 관측 기반 재계획</h1><p>준비 완료 후 SSH 터미널 Enter · 정지 s+Enter / Ctrl+C · 읽기 전용 화면</p><p id="status"></p><p id="measures"></p><p id="fusion"></p>
<div class="grid"><div><img src="/image"><br><canvas id="map" width="640" height="400"></canvas><br><small>입구 기준 평면도: 파란 상자=지금 추정 차량 · 하늘색 궤적=최근 추정 위치 · 흰 점=태그 창으로 앵커 갱신 · 자홍 고리=불가능한 점프를 1초 넘게 고집해 태그를 채택 · 초록 고리=첫 앵커 · 회색 점선=중심선 · 노란 선=계획 후보. 실제 정확도 보장 아님.</small></div>
<div><canvas id="odom" width="320" height="320"></canvas><br><small>OpenVINS 자기 기준(odom 원점) 궤적 · 화살표=지금 자세 · 추적이 끊겨 원점이 바뀌면 지워짐</small><pre id="info"></pre></div></div>
<script>
let canvas=document.getElementById('map'),c=canvas.getContext('2d'),oc=document.getElementById('odom').getContext('2d');
let trail=[],events=[],vtrail=[],lastAnchor=null,lastCapture=null,lastEpoch=null;
const f=(v,n)=>v==null||!isFinite(v)?'—':v.toFixed(n);
async function tick(){try{
let d=await(await fetch('/state')).json(),o=d.observation||{},s=d.controller||{};
document.getElementById('status').textContent=s.state+' / '+(s.reason||'')+' / '+(d.age_s==null?'관측 없음':d.age_s.toFixed(2)+'s 경과');
let g=d.geometry;document.getElementById('measures').textContent=g?'포크 목표까지 '+g.remaining_m.toFixed(3)+'m / 중심선 좌우 '+g.lateral_m.toFixed(3)+'m / 방향 '+g.heading_deg.toFixed(2)+'° / 예상 도착 좌우 '+(g.predicted_arrival_y_m==null?'—':g.predicted_arrival_y_m.toFixed(3)+'m'):'위치 기준 수집 중 (태그 창 3장 필요)';
let lp=o.local_pose||[],tf=o.tag_fusion||{},sr=o.stationary_report||{},sg=o.sigma||{};
document.getElementById('fusion').textContent='VIO 자기기준 x '+f(lp[0],3)+' y '+f(lp[1],3)+' yaw '+f(lp[2]==null?null:lp[2]*180/Math.PI,2)+'° / 앵커 뒤 이동 '+f(o.distance_since_anchor_m,2)+'m · 태그 본 지 '+f(o.blind_age_s,1)+'s / 방향 offset '+f(tf.heading_offset_deg,2)+'° ('+(tf.heading_offset_source||'—')+') / 융합: '+(tf.reason||'—')+(tf.residual_m?' 잔차 '+f(tf.residual_m[0],3)+','+f(tf.residual_m[1],3)+'m':'')+' / σ(x,y,ψ) '+(o.confidence_radius?o.confidence_radius.map((v,i)=>i<2?f(v/2,3):f(v/2*180/Math.PI,2)).join(','):'—')+' / 정지: '+(sr.reason||'—')+' (자이로 '+f(sr.gyro_p90_dps,2)+'°/s, VIO '+f(sr.vio_speed_mps,3)+'m/s)';
document.getElementById('info').textContent=JSON.stringify({control:s,pose:o.pose,local_pose:o.local_pose,sigma:sg,tag_fusion:tf,stationary:sr,tag:o.tag,odom:o.odom&&Object.fromEntries(Object.entries(o.odom).filter(([k])=>k!=='tracked_points')),performance:o.performance},null,1);
if(o.capture_id!=null&&o.capture_id!==lastCapture){lastCapture=o.capture_id;
 if(o.motion_epoch!==lastEpoch){lastEpoch=o.motion_epoch;vtrail=[]}
 if(lp.length===3){vtrail.push(lp);if(vtrail.length>1200)vtrail.shift()}
 if(o.pose){trail.push(o.pose);if(trail.length>1200)trail.shift();
  if(o.anchor_stamp!=null&&o.anchor_stamp!==lastAnchor){lastAnchor=o.anchor_stamp;events.push({p:o.pose,kind:tf.reason});if(events.length>400)events.shift()}}}
c.clearRect(0,0,640,400);
if(o.pose){let [x,y,h]=o.pose,site=d.cfg_view.site,minx=Math.min(-4,x-1),scale=Math.min(90,560/(2.5-minx)),p=(a,b)=>[40+(a-minx)*scale,200-b*scale];
 c.strokeStyle='#94a4b8';c.setLineDash([5,5]);c.beginPath();c.moveTo(...p(minx,0));c.lineTo(...p(2,0));c.stroke();c.setLineDash([]);
 if(site.bay_width_m){c.strokeStyle='#777';for(let k of [-1,1]){c.beginPath();c.moveTo(...p(0,k*site.bay_width_m/2));c.lineTo(...p(3,k*site.bay_width_m/2));c.stroke()}}
 if(trail.length>1){c.strokeStyle='#5fd3ff';c.lineWidth=1.5;c.beginPath();trail.forEach((v,i)=>i?c.lineTo(...p(v[0],v[1])):c.moveTo(...p(v[0],v[1])));c.stroke();c.lineWidth=1}
 for(let e of events){let [ex,ey]=p(e.p[0],e.p[1]);if(e.kind==='accepted_after_skip'){c.strokeStyle='#ff5fd2';c.lineWidth=2;c.beginPath();c.arc(ex,ey,7,0,7);c.stroke();c.lineWidth=1}else if(e.kind==='first_anchor'){c.strokeStyle='#45ddc0';c.lineWidth=2;c.beginPath();c.arc(ex,ey,7,0,7);c.stroke();c.lineWidth=1}else{c.fillStyle='#ffffff';c.beginPath();c.arc(ex,ey,1.5,0,7);c.fill()}}
 if(s.route_points&&s.route_points.length){c.strokeStyle='#e4c76b';c.setLineDash([4,4]);c.beginPath();s.route_points.forEach((v,i)=>i?c.lineTo(...p(v[0],v[1])):c.moveTo(...p(v[0],v[1])));c.stroke();c.setLineDash([])}
 c.save();c.translate(...p(x,y));c.rotate(-h);c.fillStyle='rgba(113,183,255,.85)';let w=site.vehicle_width_m||1.5;c.fillRect(-site.rear_extent_m*scale,-w/2*scale,(site.camera_to_fork_tip_m+site.rear_extent_m)*scale,w*scale);c.fillStyle='#fff';c.fillRect(-2,-2,4,4);c.restore();
 c.fillStyle='#45ddc0';c.beginPath();c.arc(...p(d.cfg_view.target,0),5,0,7);c.fill()}
else{c.fillStyle='#94a4b8';c.fillText('입구 기준 위치 없음 (태그 창 대기)',20,30)}
oc.clearRect(0,0,320,320);
if(vtrail.length){let xs=vtrail.map(v=>v[0]),ys=vtrail.map(v=>v[1]),cx=(Math.min(...xs)+Math.max(...xs))/2,cy=(Math.min(...ys)+Math.max(...ys))/2,span=Math.max(1,Math.max(...xs)-Math.min(...xs),Math.max(...ys)-Math.min(...ys))*1.2,k=280/span,q=(a,b)=>[160+(a-cx)*k,160-(b-cy)*k];
 oc.strokeStyle='#3a4b5c';oc.beginPath();oc.moveTo(...q(cx-span,0));oc.lineTo(...q(cx+span,0));oc.moveTo(...q(0,cy-span));oc.lineTo(...q(0,cy+span));oc.stroke();
 oc.strokeStyle='#ffd166';oc.lineWidth=1.5;oc.beginPath();vtrail.forEach((v,i)=>i?oc.lineTo(...q(v[0],v[1])):oc.moveTo(...q(v[0],v[1])));oc.stroke();oc.lineWidth=1;
 let v=vtrail[vtrail.length-1],[ax,ay]=q(v[0],v[1]);oc.strokeStyle='#fff';oc.beginPath();oc.moveTo(ax,ay);oc.lineTo(ax+18*Math.cos(v[2]),ay-18*Math.sin(v[2]));oc.stroke();oc.fillStyle='#fff';oc.beginPath();oc.arc(ax,ay,3,0,7);oc.fill();
 oc.fillStyle='#94a4b8';oc.fillText('눈금 '+(span/1.2).toFixed(1)+' m',8,312)}
else{oc.fillStyle='#94a4b8';oc.fillText('VIO 자세 없음',20,30)}
}catch(e){document.getElementById('status').textContent='연결 끊김 '+e}setTimeout(tick,250)}tick();let img=document.querySelector('img');setInterval(()=>img.src='/image?t='+Date.now(),200);</script></html>'''

def encode_preview(image,obs,cfg):
    import cv2
    out=image.copy()
    if out.ndim==2:out=cv2.cvtColor(out,cv2.COLOR_GRAY2BGR)
    import numpy as np
    stream=obs.get('preview_stream','infrared')
    # Tag corners are colour-image coordinates: draw them only on the colour preview (camera-only mode).
    if stream=='color' and obs['tag'].get('stream')=='color' and obs['tag'].get('corners'):cv2.polylines(out,[np.array(obs['tag']['corners'],int)],True,(0,255,0) if obs['tag']['ok'] else (0,200,255),2)
    if stream=='infrared':
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
