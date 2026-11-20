"""Read-only loopback dashboard. Browser and JPEG never block control."""
import json
import math
from .serialization import numpy_json
import threading
import time
import numpy as np
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlparse,parse_qs

HTML=r'''<!doctype html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Forklift Docking</title>
<style>
:root{color-scheme:dark;--bg:#0d1622;--panel:#142231;--alt:#192b3d;--line:#2a3e51;--text:#ecf3fa;--muted:#9bb0c4;--cyan:#52d4ea;--green:#73d9ac;--amber:#eac47a;--red:#f2939c}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px/1.5 -apple-system,BlinkMacSystemFont,"Apple SD Gothic Neo","Malgun Gothic",sans-serif}button{font:inherit}button:focus-visible,summary:focus-visible{outline:2px solid var(--cyan);outline-offset:3px}
.topbar{border-bottom:1px solid var(--line);background:var(--panel)}.top-inner{max-width:1600px;margin:auto;padding:20px 26px;display:flex;align-items:center;gap:14px;flex-wrap:wrap}.logo{width:42px;height:42px;border-radius:12px;background:#173d48;color:var(--cyan);display:grid;place-items:center;flex:none}.logo svg{width:27px;height:27px}h1{font-size:23px;letter-spacing:-.6px;line-height:1.25;margin:0;font-weight:650}.subtitle{font-size:12px;color:var(--muted);margin-top:5px}.header-status{margin-left:auto;display:flex;gap:8px;align-items:center;flex-wrap:wrap}.chip{display:inline-flex;gap:7px;align-items:center;border-radius:7px;padding:5px 10px;color:var(--muted);background:var(--alt);font-size:12px}.chip:before{content:"";height:6px;width:6px;border-radius:50%;background:currentColor}.chip.good{color:var(--green);background:#193b32}.chip.warn{color:var(--amber);background:#3e3423}.chip.bad{color:var(--red);background:#432a35}
main{max-width:1600px;margin:auto;padding:22px 26px;display:grid;gap:18px}.metrics{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:14px}.metric,.panel{border:1px solid var(--line);background:var(--panel);border-radius:14px;min-width:0;overflow:hidden}.metric{padding:18px 20px}.label,.caption{color:var(--muted)}.label{font-size:12px}.metric-value{font-size:34px;letter-spacing:-1px;font-weight:600;white-space:nowrap;line-height:1.35;margin:4px 0;font-variant-numeric:tabular-nums}.metric-value small{font-size:14px;font-weight:400;letter-spacing:0;color:var(--muted);margin-left:7px}.metric:first-child .metric-value{color:var(--cyan)}.caption{font-size:11px}.workspace{display:grid;grid-template-columns:minmax(0,1.65fr) minmax(0,1fr);gap:18px}.panel-head{display:flex;justify-content:space-between;align-items:center;gap:10px;padding:17px 18px 12px;flex-wrap:wrap}.panel-head h2,h2{font-size:15px;font-weight:600;margin:0;letter-spacing:-.2px}.panel-note{font-size:11px;color:var(--muted)}.map-panel{display:flex;flex-direction:column}.map-wrap{position:relative;flex:1;min-height:360px;margin:0 14px;border-radius:10px;overflow:hidden;background:#101e2c}.map-wrap canvas{width:100%;height:100%;position:absolute;inset:0;display:block}.legend{padding:12px 18px;display:flex;gap:14px;flex-wrap:wrap;font-size:11px;color:var(--muted)}.legend span{display:inline-flex;gap:6px;align-items:center}.key{width:18px;height:2px;background:var(--cyan);display:inline-block}.key.plan{height:0;background:none;border-top:2px dashed var(--cyan)}.key.history{background:#7d93ab}.key.tag{height:7px;width:7px;border-radius:50%;background:var(--amber)}.key.anchor{height:5px;width:5px;border-radius:50%;background:#fff}.key.jump{height:8px;width:8px;border:2px solid #e6a0da;border-radius:50%;background:none}.plan-box{margin:0 14px 14px;padding:13px;background:var(--alt);border-radius:10px}.plan-title{display:flex;justify-content:space-between;gap:10px;flex-wrap:wrap;font-size:12px;margin-bottom:10px}.plan-title span{color:var(--muted)}.points{display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:7px}.point{min-width:0;background:#142231;border:1px solid var(--line);border-radius:7px;color:var(--muted);padding:8px 3px;cursor:pointer;font-size:12px}.point.selected{color:var(--cyan);border-color:#3e8599;background:#173d48}.point:disabled{opacity:.4;cursor:default}.point-detail{font-size:11px;color:var(--muted);margin-top:9px;min-height:17px;overflow-wrap:anywhere}.sidebar{display:grid;gap:18px;align-content:start}.camera-frame{position:relative;background:#0a131d;margin:0 14px;aspect-ratio:4/3;overflow:hidden;border-radius:10px}.camera-frame img{display:block;width:100%;height:100%;object-fit:contain}.camera-placeholder{position:absolute;inset:0;display:grid;place-content:center;gap:8px;text-align:center;padding:20px;color:var(--muted);background:repeating-linear-gradient(0deg,transparent,transparent 29px,#22324555 30px),repeating-linear-gradient(90deg,transparent,transparent 29px,#22324555 30px),#101e2c}.camera-placeholder strong{font-size:14px;color:var(--text);font-weight:500}.camera-placeholder small{font-size:11px}.camera-caption{display:flex;justify-content:space-between;gap:10px;padding:10px 17px 13px;font-size:11px;color:var(--muted);flex-wrap:wrap}.odom-layout{display:grid;grid-template-columns:minmax(0,1fr) 125px;gap:14px;padding:0 14px 14px;align-items:center}.odom-map{background:#101e2c;border-radius:9px;height:155px;position:relative;overflow:hidden}.odom-map canvas{width:100%;height:100%;display:block}.odom-values{display:grid;gap:12px;font-size:12px;font-variant-numeric:tabular-nums}.odom-values div{display:flex;justify-content:space-between;gap:6px}.odom-values span{color:var(--muted)}.odom-values strong{font-weight:500;white-space:nowrap}.odom-note{padding:0 17px 13px;font-size:11px;color:var(--muted)}
.stages{padding:17px 18px}.stage-head{display:flex;justify-content:space-between;align-items:center;gap:10px;flex-wrap:wrap;margin-bottom:14px}.stage-grid{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:8px}.stage{background:var(--alt);border:1px solid var(--line);border-radius:8px;min-height:68px;display:grid;align-content:center;justify-items:center;gap:8px;color:var(--muted);font-size:12px;text-align:center;padding:9px 4px}.lamp{height:6px;width:6px;border-radius:50%;background:var(--muted);opacity:.3}.stage.active{color:var(--green);border-color:#34785c;background:#193b32}.stage.active .lamp{background:var(--green);opacity:1;box-shadow:0 0 0 4px #73d9ac16}.stage.fault.active{color:var(--red);border-color:#934e61;background:#432a35}.stage.fault.active .lamp{background:var(--red);box-shadow:none}.stage-detail{display:flex;gap:10px;align-items:baseline;flex-wrap:wrap;margin-top:14px;font-size:12px}.stage-detail strong{font-weight:600}.stage-detail span{color:var(--muted);overflow-wrap:anywhere}.stage-progress{font-size:12px;color:var(--cyan);font-variant-numeric:tabular-nums}
.diagnostics{padding:0 18px}summary{cursor:pointer;list-style:none;display:flex;justify-content:space-between;gap:12px;align-items:center;padding:16px 0;font-size:13px}summary::-webkit-details-marker{display:none}summary:after{content:"+";font-size:19px;color:var(--muted);margin-left:auto}details[open]>summary:after{content:"−"}.diagnostic-summary{font-size:11px;color:var(--muted);margin-left:auto;text-align:right}.diagnostic-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:24px;padding-bottom:18px}.diagnostic-grid h3{font-size:12px;font-weight:600;margin:2px 0 12px}.diagnostic-grid dl{display:grid;grid-template-columns:auto minmax(0,1fr);gap:9px 14px;margin:0;font-size:11px}.diagnostic-grid dt{color:var(--muted)}.diagnostic-grid dd{margin:0;text-align:right;font-variant-numeric:tabular-nums;overflow-wrap:anywhere}.raw{border-top:1px solid var(--line)}.raw summary{color:var(--muted);font-size:12px;padding:12px 0}pre{white-space:pre-wrap;overflow-wrap:anywhere;max-height:450px;overflow:auto;background:#0e1b29;color:#b1c3d5;font:11px/1.6 ui-monospace,SFMono-Regular,Consolas,monospace;border-radius:9px;padding:14px;margin:0 0 16px}.footer{font-size:11px;color:var(--muted);display:flex;justify-content:space-between;gap:10px;flex-wrap:wrap}.stale .metrics{opacity:.55}.stale canvas{opacity:.5}[hidden]{display:none!important}
.cameras{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:18px}.bev-wrap{position:relative;margin:0 14px;aspect-ratio:4/3;background:#101e2c;border-radius:10px;overflow:hidden}.bev-wrap canvas{position:absolute;inset:0;width:100%;height:100%;display:block}.bev-values{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px;padding:10px 17px 4px;font-size:12px;font-variant-numeric:tabular-nums}.bev-values div{display:flex;flex-direction:column;gap:2px}.bev-values span{color:var(--muted);font-size:11px}.workspace .odom-map{height:300px}
@media(min-width:1450px){.map-wrap{min-height:440px}}@media(max-width:1050px){.top-inner,main{padding-left:18px;padding-right:18px}.workspace{grid-template-columns:minmax(0,1.4fr) minmax(0,1fr)}.metric{padding:14px}.metric-value{font-size:28px}.odom-layout{grid-template-columns:1fr}.odom-map{height:130px}.odom-values{grid-template-columns:repeat(3,minmax(0,1fr));gap:12px;font-size:11px}.odom-values div{flex-direction:column;gap:2px}.diagnostic-grid{gap:14px}}@media(max-width:760px){.cameras{grid-template-columns:1fr;gap:14px}.top-inner{padding:16px}.header-status{margin-left:0;width:100%}main{padding:16px;gap:14px}.metrics{grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}.workspace{grid-template-columns:1fr;gap:14px}.map-wrap{height:370px;flex:none;min-height:0}.stage-grid{grid-template-columns:repeat(2,minmax(0,1fr))}.diagnostic-grid{grid-template-columns:1fr}.diagnostic-summary{max-width:60%}.camera-frame{max-height:400px}.odom-layout{grid-template-columns:minmax(0,1fr) 125px}.odom-values{display:grid;grid-template-columns:1fr}.odom-values div{flex-direction:row}.odom-map{height:170px}}@media(max-width:380px){main{padding:10px}.metric-value{font-size:24px}.points{gap:4px}.point{font-size:11px}.plan-title{font-size:11px}}
</style>
</head>
<body>
<header class="topbar"><div class="top-inner">
<div class="logo" aria-hidden="true"><svg viewBox="0 0 32 32" fill="none" stroke="currentColor" stroke-width="1.8"><path d="M5 21V11h12v10M5 14h12M8 11V6h7v5M22 5v19h6M25 6v15h4M17 18h5"/><circle cx="8" cy="24" r="3"/><circle cx="17" cy="24" r="3"/></svg></div>
<div><h1>Forklift Docking</h1><div class="subtitle">준비 완료 후 SSH 터미널 Enter · 정지 s+Enter / Ctrl+C · 읽기 전용 화면</div></div>
<div class="header-status"><span class="chip" id="mode">연결 대기</span><span class="chip" id="live">데이터 대기</span><span class="chip" id="odom-health">VIO 대기</span></div>
</div></header>
<main>
<section class="metrics" aria-label="도킹 주요 수치">
<div class="metric"><div class="label">포크 목표까지</div><div class="metric-value"><span id="remaining">—</span><small>m</small></div><div class="caption">포크 앞끝 기준 · 목표까지 남은 거리</div></div>
<div class="metric"><div class="label">중심선 좌우</div><div class="metric-value"><span id="lateral">—</span><small>m</small></div><div class="caption">차량 기준점 · 왼쪽 + / 오른쪽 −</div></div>
<div class="metric"><div class="label">차량 방향</div><div class="metric-value"><span id="heading">—</span><small>°</small></div><div class="caption">입구 법선 기준 · 목표 방향 0°</div></div>
<div class="metric"><div class="label">예상 도착 좌우</div><div class="metric-value"><span id="arrival-y">—</span><small>m</small></div><div class="caption">현재 방향으로 목표면까지 직진할 때</div></div>
</section>
<div class="cameras">
<section class="panel" id="panel-ir"><div class="panel-head"><h2>카메라 1 · IR</h2><span class="panel-note" id="camera-type-ir">IR 흑백 + OpenVINS 점</span></div><div class="camera-frame"><img id="preview-ir" alt="IR 영상과 VIO 추적점" hidden><div class="camera-placeholder" id="camera-placeholder-ir"><strong id="camera-message-ir">카메라 연결 대기</strong><small>영상이 수신되면 이곳에 표시됩니다.</small></div></div><div class="camera-caption"><span id="fps-ir">카메라 — / 처리 — fps</span><span id="image-age-ir">영상 대기</span></div></section>
<section class="panel" id="panel-color"><div class="panel-head"><h2>카메라 2 · 컬러</h2><span class="panel-note" id="camera-type-color">컬러 + 태그 + 지시선</span></div><div class="camera-frame"><img id="preview-color" alt="컬러 영상, 태그 사각형, 차폭 지시선" hidden><div class="camera-placeholder" id="camera-placeholder-color"><strong id="camera-message-color">카메라 연결 대기</strong><small>영상이 수신되면 이곳에 표시됩니다.</small></div></div><div class="camera-caption"><span id="fps-color">카메라 — / 처리 — fps</span><span id="image-age-color">영상 대기</span></div></section>
<section class="panel" id="panel-depth"><div class="panel-head"><h2>카메라 3 · depth</h2><span class="panel-note" id="camera-type-depth">depth 흑백 + 입구 양끝·중심</span></div><div class="camera-frame"><img id="preview-depth" alt="depth 영상과 depth 로 찾은 입구" hidden><div class="camera-placeholder" id="camera-placeholder-depth"><strong id="camera-message-depth">카메라 연결 대기</strong><small>영상이 수신되면 이곳에 표시됩니다.</small></div></div><div class="camera-caption"><span id="fps-depth">카메라 — / 처리 — fps</span><span id="image-age-depth">영상 대기</span></div></section>
<section class="panel"><div class="panel-head"><h2>입구 고정 버드아이뷰</h2><span class="panel-note" id="bev-note">depth 측정 대기</span></div><div class="bev-wrap"><canvas id="bev" aria-label="depth 로 찾은 입구를 고정한 위에서 본 지도"></canvas></div><div class="bev-values"><div><span>좌우 오차</span><strong id="bev-lat">— m</strong></div><div><span>입구까지</span><strong id="bev-dist">— m</strong></div><div><span>입구 폭</span><strong id="bev-width">— m</strong></div></div><div class="odom-note" id="bev-values">흰 점 = depth 로 잰 물체(바닥 5 cm~1.6 m), 노랑 = 찾은 입구 양끝, 초록 상자 = 차. 입구를 못 찾으면 차 기준으로 그립니다.</div></section>
</div>
<div class="workspace">
<section class="panel map-panel">
<div class="panel-head"><h2>도킹 경로</h2><span class="panel-note" id="map-mode">입구 기준 · x 진입 방향 / y 왼쪽</span></div>
<div class="map-wrap"><canvas id="map" aria-label="입구 기준 차량 추정 궤적과 계획점"></canvas></div>
<div class="legend"><span><i class="key"></i>다음 동작</span><span><i class="key plan"></i>이어지는 동작</span><span><i class="key jump" style="border-color:#f2939c"></i>depth 입구 중심</span><span><i class="key history"></i>추정 궤적</span><span><i class="key tag"></i>태그</span><span><i class="key anchor"></i>앵커 갱신</span><span><i class="key jump"></i>점프 후 채택</span></div>
<div class="plan-box"><div class="plan-title"><strong>예정 동작 (회전·직진 묶음)</strong><span id="plan-status">동작 대기</span></div><div class="points" id="points" aria-label="예정 동작점 조회"></div><div class="point-detail" id="point-detail">동작이 정해지면 그 끝점들을 표시합니다.</div></div>
</section>
<section class="panel"><div class="panel-head"><h2>OpenVINS 위치</h2><span class="panel-note">VIO 자체 좌표계</span></div><div class="odom-layout"><div class="odom-map"><canvas id="odom" aria-label="OpenVINS 자체 좌표계 추정 궤적"></canvas></div><div class="odom-values"><div><span>x</span><strong id="vio-x">— m</strong></div><div><span>y</span><strong id="vio-y">— m</strong></div><div><span>yaw</span><strong id="vio-yaw">— °</strong></div></div></div><div class="odom-note" id="odom-note">위치 수신 대기 · 입구 기준 좌표와는 별개입니다.</div></section>
</div>
<section class="panel stages"><div class="stage-head"><h2>현재 주행 단계</h2><span class="stage-progress" id="stage-progress"></span><span class="panel-note" id="state-code">상태 대기</span></div><div class="stage-grid" id="stages" aria-label="현재 주행 단계 표시"></div><div class="stage-detail" role="status"><strong id="stage-label">연결 대기</strong><span id="stage-reason">단계 표시는 차량 상태를 읽어 자동으로 바뀝니다.</span></div></section>
<details class="panel diagnostics" id="diagnostics"><summary><strong>센서 · 오차 · 보정 상세</strong><span class="diagnostic-summary" id="diagnostic-summary">측정 대기</span></summary><div class="diagnostic-grid">
<div><h3>태그 · 앵커 보정</h3><dl><dt>태그 상태</dt><dd id="diag-tag">—</dd><dt>태그 거리</dt><dd id="diag-range">—</dd><dt>태그 본 지</dt><dd id="diag-tag-age">—</dd><dt>앵커 뒤 이동</dt><dd id="diag-travel">—</dd><dt>융합 결과</dt><dd id="diag-fusion">—</dd><dt>위치 잔차 x / y</dt><dd id="diag-residual">—</dd><dt>방향 offset</dt><dd id="diag-offset">—</dd><dt>offset 갱신 방식</dt><dd id="diag-offset-source">—</dd><dt>방향 재설정</dt><dd id="diag-rezero">—</dd><dt>depth 입구</dt><dd id="diag-depth">—</dd><dt>depth 좌우 오차</dt><dd id="diag-depth-lat">—</dd></dl></div>
<div><h3>불확실성 · 정지 관측</h3><dl><dt>σ(x, y)</dt><dd id="diag-sigma-xy">—</dd><dt>σ(방향)</dt><dd id="diag-sigma-yaw">—</dd><dt>기준 측정</dt><dd id="diag-baseline">—</dd><dt>정지 판정</dt><dd id="diag-still">—</dd><dt>자이로 p90</dt><dd id="diag-gyro">—</dd><dt>VIO 속도</dt><dd id="diag-speed">—</dd><dt>추적 상태</dt><dd id="diag-odom">—</dd></dl></div>
<div><h3>처리 · 통신 · 기록</h3><dl><dt>관측 나이</dt><dd id="diag-age">—</dd><dt>카메라 / 처리</dt><dd id="diag-fps">—</dd><dt>SDK 프레임 누락</dt><dd id="diag-gaps">—</dd><dt>최신 프레임 교체</dt><dd id="diag-replaced">—</dd><dt>전달 지연</dt><dd id="diag-lag">—</dd><dt>기록 상태</dt><dd id="diag-recording">—</dd><dt>출력 명령</dt><dd id="diag-command">—</dd></dl></div>
</div><details class="raw" id="raw"><summary>원본 데이터 보기</summary><pre id="info"></pre></details></details>
<footer class="footer"><span>궤적은 추정 위치(태그+VIO), 빨간 고리는 depth 로 잰 입구 중심, 점선은 예정 동작입니다.</span><span id="status">수신 대기 · 화면에서 차량을 조작하지 않습니다.</span></footer>
</main>
<script>
'use strict';
const $=id=>document.getElementById(id),finite=v=>typeof v==='number'&&Number.isFinite(v);
const fmt=(v,n=2)=>finite(v)?v.toFixed(n):'—',signed=(v,n=3)=>finite(v)?(v>0?'+':'')+v.toFixed(n):'—';
const poseOK=p=>Array.isArray(p)&&p.length>=3&&p.slice(0,3).every(finite);
const set=(id,value)=>{$(id).textContent=value;};
let data={},trail=[],vtrail=[],anchors=[],lastCapture=null,lastEpoch=null,lastGeneration=null,lastAnchor=null;
let routeKey='',selectedPoint=0,latestRoute=[],connected=false,lastReceive=0,lastImage=0;
const STAGES=[['measure','정지·측정'],['wait','출발 대기'],['search','태그 탐색'],['rough','러프 접근'],['fine','세밀 조정 (depth)'],['final','최종 확인'],['entry','진입'],['recover','추적 회복'],['end','종료']];
for(const [key,label] of STAGES){const tile=document.createElement('div');tile.className='stage';tile.dataset.stage=key;const lamp=document.createElement('i');lamp.className='lamp';lamp.setAttribute('aria-hidden','true');const name=document.createElement('span');name.textContent=label;tile.append(lamp,name);$('stages').append(tile);}
for(let i=0;i<6;i++){const b=document.createElement('button');b.type='button';b.className='point';b.textContent='P'+(i+1);b.disabled=true;b.title='계획점 좌표 조회 · 차량 제어 아님';b.onclick=()=>{selectedPoint=i;renderPoints();drawMaps();};$('points').append(b);}
const REASONS={stationary_preparation:'출발 전 정지 상태를 측정하고 있습니다.',ready_press_enter:'준비 완료. SSH 터미널에서 Enter를 누르면 시작합니다.',rechecking_before_enter:'정지 상태를 다시 확인하고 있습니다.',stationary_baseline_with_tag:'태그를 보며 정지 기준값을 수집하고 있습니다.',baseline_waiting_for_pose:'정지 기준 측정에 필요한 위치 관측을 기다립니다.',baseline_waiting_for_stillness:'정지 상태가 확인되면 기준 측정을 이어갑니다.',six_step_beam_search:'최대 6단계 후보를 계산하고 있습니다.',measured_motion_feedback:'실제 관측 이동량을 확인하며 동작 중입니다.',observing_coast_and_rebound:'정지 명령 후 잔여 이동과 되튐을 확인합니다.',tracking_lost_during_motion:'추적 회복을 기다리고 있습니다.',need_stopped_observation:'정지 관측을 기다리고 있습니다.',waiting_for_tag_consensus:'태그 관측이 모일 때까지 기다립니다.',waiting_recent_tag_before_blind:'최종 진입 전 최근 태그 관측을 기다립니다.',blind_entry_vio_progress:'VIO 이동량을 기준으로 최종 진입 중입니다.',blind_entry_vio_lost_timed_fallback:'VIO 손실로 시간 기준 최종 진입 중입니다.',blind_entry_done_recorded_not_judged:'최종 진입 종료. 도착 결과를 기록했습니다.',measured_stop_at_target_not_accuracy_certificate:'목표 정지 조건에 도달했습니다.',already_past_target_recorded_not_judged:'목표를 지난 상태에서 결과를 기록했습니다.',operator_started:'Enter 입력. 태그를 찾거나 정지 기준 측정으로 넘어갑니다.',baseline_done:'정지 기준 측정 완료. 첫 계획을 세웁니다.',tracking_recovered_replan:'추적이 돌아와 현재 위치에서 다시 계획합니다.',stopped_pose_reacquisition:'정지 뒤 위치를 다시 잡고 있습니다.',rotation_changed_visibility_replan:'회전으로 태그 가시성이 바뀌어 다시 계획합니다.',observation_changed_during_plan:'계획 중 관측이 바뀌어 다시 계획합니다.',pose_untrusted:'위치를 믿을 수 없어 멈추고 회복을 기다립니다.',absolute_pose_untrusted:'입구 기준 위치를 믿을 수 없어 멈춥니다.',fast_imu_unavailable:'자이로가 3초 넘게 끊겨 멈추고 회복을 기다립니다.',target_reached_with_lateral_error:'목표 거리에 도달했으나 좌우 오차가 남았습니다.',blind_predicted_lateral_outside_tolerance:'예상 도착 좌우가 허용(3 cm)을 벗어나 보정합니다.',transition_corrections_exhausted:'입구 보정 회전을 5회 다 써서 중단했습니다.',recent_tag_before_blind_timeout:'최종 진입 전 최근 태그를 60초 안에 못 봐 중단했습니다.',search_exhausted:'360° 탐색에도 태그를 못 찾아 중단했습니다.',search_tracking_lost_unmeasured_angle:'탐색 회전 중 추적을 잃어 돈 각도를 알 수 없어 중단했습니다.',motion_time_limit:'동작 시간 한도를 넘겨 중단했습니다.',motion_direction_mismatch:'명령과 반대 방향으로 움직여 중단했습니다.',bay_side_clearance:'탑재부 벽에 닿을 위치라 중단했습니다.',recovery_timeout:'회복을 60초 기다렸지만 돌아오지 않아 중단했습니다.',planner_timeout:'경로 계획이 제한 시간 안에 끝나지 않아 중단했습니다.',mission_timeout:'전체 주행 제한 시간을 넘겨 중단했습니다.',preparation_timeout:'출발 준비(정지 측정)가 제한 시간 안에 끝나지 않아 중단했습니다.',imu_calibration_failed:'IMU 준비에 실패해 중단했습니다.',sensor_clock_fault:'센서 시계 이상으로 중단했습니다.',legs_done:'예정 동작을 마쳤습니다. 다시 재고 정합니다.',rezero_heading_from_tag:'서서 태그로 방향 기준을 다시 잡고 있습니다.',fine_stage_start:'세밀 조정 시작: depth 로 입구 중심을 봅니다.',measuring_depth_entrance:'depth 로 입구 중심을 재고 있습니다(정지 중).',entry_vio_progress:'VIO 이동량을 기준으로 진입 중입니다.',entry_vio_lost_timed_fallback:'VIO 손실로 시간 기준 진입 중입니다.',entry_no_progress_timed_fallback:'진행이 멈춰 시간 기준으로 진입을 이어갑니다.',entry_done_recorded_not_judged:'진입 종료. 도착 결과를 기록했습니다.',entry_distance_zero_recorded:'진입 거리가 0 이라 그 자리에서 끝냈습니다.',final_corrections_exhausted_recorded_no_entry:'최종 확인 보정 횟수를 다 써서 들어가지 않고 기록했습니다.',final_no_room_recorded_no_entry:'고칠 공간이 없어 들어가지 않고 기록했습니다.',truck_side_clearance:'탑재부 벽에 닿을 위치라 중단했습니다.',no_progress_repeated:'진행 없음 정지가 반복되어 중단했습니다.'};
function stageFor(s,o){
 const state=s.state||'',cmd=s.command||'',phase=s.phase||'';
 if(state==='FAULT'||state==='COMPLETE')return 'end';   // one tile: text and colour say which (manual s+Enter ends the process, no state)
 if(state==='WAIT_ENTER')return 'wait';if(['MEASURE','PREPARING','BASELINE'].includes(state))return 'measure';
 if(state==='RECOVER')return 'recover';
 if(state==='MOVING'&&s.search_active)return 'search';
 if(state==='PLAN'&&!s.baseline_done)return 'search';
 if(phase==='ROUGH')return 'rough';if(phase==='FINE_ENTER'||phase==='FINE')return 'fine';if(phase==='FINAL')return 'final';
 if(phase==='ENTRY'||phase==='FALLBACK'||s.blind)return 'entry';
 return null;
}
function badge(id,label,kind=''){set(id,label);$(id).className='chip'+(kind?' '+kind:'');}
function localAvailable(o){return poseOK(o.local_pose)&&(o.local_valid===true||(o.odom||{}).ok===true);}
function collect(o){
 if(o.capture_id==null||o.capture_id===lastCapture)return;
 const reset=lastEpoch!==o.motion_epoch||lastGeneration!==o.tracking_generation;
 if(reset){vtrail=[];if(trail.length&&trail[trail.length-1]!==null)trail.push(null);lastAnchor=null;}
 lastEpoch=o.motion_epoch;lastGeneration=o.tracking_generation;lastCapture=o.capture_id;
 if(localAvailable(o)){vtrail.push(o.local_pose.slice(0,3));if(vtrail.length>1200)vtrail.shift();}
 else if(vtrail.length&&vtrail[vtrail.length-1]!==null)vtrail.push(null);
 if(poseOK(o.pose)){
  trail.push(o.pose.slice(0,3));if(trail.length>1200)trail.shift();
  if(o.anchor_stamp!=null&&o.anchor_stamp!==lastAnchor){lastAnchor=o.anchor_stamp;anchors.push({p:o.pose.slice(0,3),kind:(o.tag_fusion||{}).reason});if(anchors.length>400)anchors.shift();}
 }else if(trail.length&&trail[trail.length-1]!==null)trail.push(null);
}
function renderPoints(){
 const s=data.controller||{},o=data.observation||{},usable=poseOK(o.pose);
 const route=Array.isArray(s.route_points)?s.route_points:[];
 // route_points[0] is the planning origin, not P1. Never create missing points.
 latestRoute=route.length&&route.every(poseOK)?route:[];
 const key=JSON.stringify(latestRoute);if(key!==routeKey){routeKey=key;selectedPoint=0;}
 const candidates=latestRoute.slice(1),history=['COMPLETE','FAULT','RECOVER'].includes(s.state)||s.blind;
 [...$('points').children].forEach((b,i)=>{b.disabled=!candidates[i];b.className='point'+(candidates[i]&&selectedPoint===i?' selected':'');b.setAttribute('aria-pressed',String(!!candidates[i]&&selectedPoint===i));});
 set('plan-status',candidates.length?(history?'최근 동작 · ':'')+candidates.length+'개 끝점':'동작 대기');
 const p=candidates[selectedPoint];
 set('point-detail',p?'P'+(selectedPoint+1)+' · x '+fmt(p[0],3)+' m / y '+signed(p[1],3)+' m / 방향 '+signed(p[2]*180/Math.PI,2)+'°'+(history?' · 이전 계획 조회':!usable?' · 현재 위치 수신 대기':' · 조회 전용'):'동작이 정해지면 P1부터 표시합니다. 측정 모드에서는 움직이지 않습니다.');
}
function render(){
 const o=data.observation||{},s=data.controller||{},g=data.geometry||{},tf=o.tag_fusion||{},sr=o.stationary_report||{},perf=o.performance||{},odom=o.odom||{};
 collect(o);renderPoints();
 set('remaining',fmt(g.remaining_m,3));set('lateral',signed(g.lateral_m,3));set('heading',signed(g.heading_deg,2));set('arrival-y',signed(g.predicted_arrival_y_m,3));
 set('mode',(data.mode||s.state||'대기').toUpperCase());
 badge('odom-health',odom.ok?'VIO 추적 중':odom.reason==='openvins_initializing'?'VIO 초기화 중':'VIO 대기 / 확인',odom.ok?'good':'warn');
 const lp=localAvailable(o)?o.local_pose:[];
 set('vio-x',fmt(lp[0],3)+' m');set('vio-y',fmt(lp[1],3)+' m');set('vio-yaw',fmt(lp[2]==null?null:lp[2]*180/Math.PI,2)+' °');
 set('odom-note',(odom.reason||'위치 수신 대기')+' · 추적 기준이 바뀌면 VIO 궤적을 새로 시작합니다.');
 const panels=(data.cfg_view||{}).panels;if(Array.isArray(panels)&&panels.length){activeStreams=STREAMS.filter(k=>panels.includes(k));for(const k of STREAMS)$('panel-'+k).hidden=!panels.includes(k);}
 for(const k of ['ir','color','depth'])set('fps-'+k,'카메라 '+fmt(perf.camera_fps,1)+' / 처리 '+fmt(perf.processed_fps,1)+' fps');
 // Preview pixels are supplied by the existing image encoder; no camera work happens here.
 const dep=o.depth||{};set('camera-type-depth',dep.ok?'입구 폭 '+fmt(dep.width_m,2)+' m · '+fmt(dep.x_body_m,2)+' m 앞':'입구: '+(dep.reason||'depth 대기'));
 set('diag-depth',dep.ok?'찾음 · '+fmt(dep.width_m,2)+' m / '+fmt(dep.x_body_m,2)+' m':(dep.reason||'—'));const dm=s.depth||{};set('diag-depth-lat',finite(dm.lateral_error_m)?signed(dm.lateral_error_m,3)+' m ('+(dm.source||'')+')':'—');
 const rz=o.heading_rezero||{};set('diag-rezero',(rz.info&&finite(rz.info.change_deg))?signed(rz.info.change_deg,2)+'° ('+rz.count+'회)':(rz.requested?'요청 '+rz.requested+' · 대기':'—'));
 const stage=stageFor(s,o);
 set('stage-label',stage==='end'?(s.state==='FAULT'?'오류·중단':'완료'):(STAGES.find(v=>v[0]===stage)||[null,s.state||'상태 대기'])[1]);
 set('stage-reason',REASONS[s.reason]||s.reason||((s.state==='MEASURE')?'측정 전용 · CAN을 열지 않습니다.':'상태 수신 대기'));
 set('state-code',(s.state||'대기')+' / '+(s.command||'—'));
 const prep=s.preparation||{};
 set('stage-progress',s.state==='PREPARING'?'정지 측정 '+fmt(prep.stable_seconds,1)+' / '+fmt(prep.required_seconds,1)+' s':s.state==='BASELINE'?'기준 측정 '+fmt(s.baseline_seconds,1)+' s':stage==='search'?'탐색 누적 '+fmt(s.search_sweep_deg,1)+'°':stage==='final'?'보정 '+fmt(s.corrections,0)+' 회':stage==='rough'?'사이드스텝 '+fmt(s.sidesteps,0)+' 회':'');
 const sigma=Array.isArray(o.confidence_radius)?o.confidence_radius.map(v=>finite(v)?v/2:null):[];
 set('diagnostic-summary','σ '+fmt(sigma[0],3)+' / '+fmt(sigma[1],3)+' m · '+fmt(sigma[2]==null?null:sigma[2]*180/Math.PI,2)+'° · '+(sr.reason||'정지 관측 대기'));
 set('diag-tag',(o.tag||{}).reason||'—');set('diag-range',fmt(tf.range_m,2)+' m');set('diag-tag-age',fmt(o.blind_age_s,1)+' s');set('diag-travel',fmt(o.distance_since_anchor_m,3)+' m');
 set('diag-fusion',(tf.reason||'—')+(finite(tf.frames)?' · '+tf.frames+'장':''));
 const residual=tf.residual_m||[];set('diag-residual',fmt(residual[0],3)+' / '+fmt(residual[1],3)+' m');set('diag-offset',fmt(tf.heading_offset_deg,2)+'°');set('diag-offset-source',tf.heading_offset_source||'—');
 set('diag-sigma-xy',fmt(sigma[0],3)+' / '+fmt(sigma[1],3)+' m');set('diag-sigma-yaw',fmt(sigma[2]==null?null:sigma[2]*180/Math.PI,2)+'°');set('diag-baseline',(o.sigma||{}).baseline_source||'—');
 set('diag-still',sr.reason||'—');set('diag-gyro',fmt(sr.gyro_p90_dps,2)+'°/s');set('diag-speed',fmt(sr.vio_speed_mps,3)+' m/s');set('diag-odom',odom.reason||'—');set('diag-age',fmt(data.age_s,2)+' s');
 set('diag-fps',fmt(perf.camera_fps,1)+' / '+fmt(perf.processed_fps,1)+' fps');set('diag-gaps',fmt(perf.sdk_frame_gaps,0)+(perf.sdk_gap_count_reliable===false?' (카운터 신뢰 제한)':''));
 set('diag-replaced',fmt(perf.latest_replaced,0));set('diag-lag',fmt(o.transport_lag_s,3)+' s');const rec=data.control_recording;set('diag-recording',rec?(rec.error?'오류: '+rec.error:'기록 중'):'—');set('diag-command',s.command||'—');
 if($('raw').open)renderRaw();refreshHealth();drawMaps();
}
function renderRaw(){const o=data.observation||{},od=o.odom||{};set('info',JSON.stringify({mode:data.mode,control:data.controller,pose:o.pose,local_pose:o.local_pose,sigma:o.sigma,tag_fusion:o.tag_fusion,stationary:o.stationary_report,tag:o.tag,odom:Object.fromEntries(Object.entries(od).filter(([k])=>k!=='tracked_points')),performance:o.performance,recording:data.control_recording},null,2));}
$('raw').addEventListener('toggle',()=>{if($('raw').open)renderRaw();});
function refreshHealth(){
 const elapsed=lastReceive?(performance.now()-lastReceive)/1000:0;
 const age=finite(data.age_s)?data.age_s+elapsed:null;
 // Display-only 1 s freshness indication; does not alter any control threshold.
 const stale=!connected||elapsed>3||(finite(age)&&age>1);
 document.body.classList.toggle('stale',stale);
 badge('live',!connected?(lastReceive?'연결 끊김':'연결 대기'):!finite(age)?'관측 대기':stale?'관측 지연 '+fmt(age,1)+' s':'관측 '+fmt(age,2)+' s',!connected&&lastReceive?'bad':stale||!finite(age)?'warn':'good');
 const stage=stageFor(data.controller||{},data.observation||{});
 for(const tile of $('stages').children){const active=!stale&&tile.dataset.stage===stage;tile.classList.toggle('active',active);if(active)tile.setAttribute('aria-current','step');else tile.removeAttribute('aria-current');
  if(tile.dataset.stage==='end'){const fault=active&&(data.controller||{}).state==='FAULT';tile.classList.toggle('fault',fault);tile.querySelector('span').textContent=active?(fault?'오류·중단':'완료'):'종료';}}
 set('status',!connected?(lastReceive?'연결 끊김 · 마지막 수신값 표시':'서버 연결 대기'):stale?'관측 지연 · 마지막 수신값 표시':'수신 중 · 화면에서 차량을 조작하지 않습니다.');
 if(stale)badge('odom-health','VIO 현재 상태 확인 대기','warn');
 if(!connected&&lastReceive){set('stage-label','연결 끊김');set('stage-reason','마지막 상태: '+((data.controller||{}).state||'—')+' · 현재 차량 상태는 확인할 수 없습니다.');badge('odom-health','VIO 현재 상태 확인 불가','warn');}
 const imageAge=lastImage?(performance.now()-lastImage)/1000:null;
 for(const k of STREAMS)set('image-age-'+k,imageAge==null?'영상 대기':imageAge>2?'영상 지연 '+fmt(imageAge,1)+' s':'영상 수신 중');
}
function surface(id){const canvas=$(id),r=canvas.getBoundingClientRect(),w=Math.max(1,r.width),h=Math.max(1,r.height),dpr=Math.min(window.devicePixelRatio||1,2);if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}const c=canvas.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);return {c,w,h};}
function mapTransform(points,w,h,pad=42){const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]),x0=Math.min(...xs),x1=Math.max(...xs),y0=Math.min(...ys),y1=Math.max(...ys);const spanx=Math.max(1.5,x1-x0),spany=Math.max(1.5,y1-y0),scale=Math.max(.01,Math.min((w-2*pad)/spanx,(h-2*pad)/spany));return {p:(x,y)=>[w/2+(x-(x0+x1)/2)*scale,h/2-(y-(y0+y1)/2)*scale],scale,cx:(x0+x1)/2,cy:(y0+y1)/2};}
function line(c,a,b,color,width=1,dash=[]){c.beginPath();c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dash);c.moveTo(...a);c.lineTo(...b);c.stroke();c.setLineDash([]);}
function dot(c,p,r,fill,stroke){c.beginPath();c.arc(...p,r,0,Math.PI*2);if(fill){c.fillStyle=fill;c.fill();}if(stroke){c.strokeStyle=stroke;c.lineWidth=1.5;c.stroke();}}
function label(c,text,x,y,color='#9bb0c4',align='left'){c.font='11px -apple-system, BlinkMacSystemFont, "Malgun Gothic", sans-serif';c.textAlign=align;c.textBaseline='middle';c.fillStyle=color;c.fillText(text,x,y);}
function drawTrail(c,points,p,color){c.beginPath();c.strokeStyle=color;c.lineWidth=1.8;let pen=false;for(const v of points){if(!v){pen=false;continue;}if(pen)c.lineTo(...p(v[0],v[1]));else c.moveTo(...p(v[0],v[1]));pen=true;}c.stroke();}
function grid(c,w,h,t){const raw=60/t.scale,base=Math.pow(10,Math.floor(Math.log10(raw))),step=[1,2,5,10].map(v=>v*base).find(v=>v>=raw)||base*10;
 for(let x=Math.floor((t.cx-w/2/t.scale)/step)*step;x<t.cx+w/2/t.scale;x+=step){const a=t.p(x,0);line(c,[a[0],0],[a[0],h],'#25394b',.6);}
 for(let y=Math.floor((t.cy-h/2/t.scale)/step)*step;y<t.cy+h/2/t.scale;y+=step){const a=t.p(0,y);line(c,[0,a[1]],[w,a[1]],'#25394b',.6);}
 label(c,'격자 '+Number(step.toPrecision(2))+' m',12,h-14);}
function drawMaps(){drawDock();drawOdom();drawBev();}
function drawDock(){
 const {c,w,h}=surface('map'),o=data.observation||{},s=data.controller||{},view=data.cfg_view||{},site=view.site||{},g=data.geometry||{},pose=poseOK(o.pose)?o.pose:null;
 const central=view.mode==='central',left=view.mode==='left';   // mode from runtime (targets are common.json values since 2026-10-20)
 const tagY=central?0:left&&finite(site.left_tag_offset_m)?site.left_tag_offset_m:null;
 const tag=finite(site.tag_plane_offset_m)&&finite(tagY)?[site.tag_plane_offset_m,tagY]:null;
 set('map-mode',(central?'중앙 태그':left?(finite(tagY)&&tagY<0?'우측 태그':'좌측 태그'):'입구 기준')+' · x 진입 / y 왼쪽');
 const points=[[-1.5,0],[1.5,0],...trail.filter(Boolean),...latestRoute];
 if(pose)points.push(pose);if(tag)points.push(tag);if(finite(view.target))points.push([view.target,0]);
 if(finite(site.truck_inner_width_m)&&site.truck_inner_width_m>0)points.push([1.5,site.truck_inner_width_m/2],[1.5,-site.truck_inner_width_m/2]);
 const dc=Array.isArray(s.depth_centre_dock)&&s.depth_centre_dock.length>=2&&s.depth_centre_dock.every(finite)?s.depth_centre_dock:null;if(dc)points.push(dc);
 let body=[];
 if(pose&&[site.vehicle_width_m,site.rear_extent_m,site.camera_to_fork_tip_m].every(finite)){
  const [x,y,ang]=pose,cs=Math.cos(ang),sn=Math.sin(ang);
  body=[[-site.rear_extent_m,-site.vehicle_width_m/2],[site.camera_to_fork_tip_m,-site.vehicle_width_m/2],[site.camera_to_fork_tip_m,site.vehicle_width_m/2],[-site.rear_extent_m,site.vehicle_width_m/2]].map(([a,b])=>[x+a*cs-b*sn,y+a*sn+b*cs]);points.push(...body);
 }
 const t=mapTransform(points,w,h),p=t.p;grid(c,w,h,t);
 const zero=p(0,0);line(c,[0,zero[1]],[w,zero[1]],'#5e7389',1,[5,6]);line(c,[zero[0],20],[zero[0],h-30],'#657a8e',1,[3,5]);
 label(c,'입구면',Math.max(30,Math.min(w-40,zero[0]+7)),24);
 if(finite(site.truck_inner_width_m)&&site.truck_inner_width_m>0){const edge=site.truck_inner_width_m/2,deep=finite(site.truck_depth_m)&&site.truck_depth_m>0?site.truck_depth_m:1.5;for(const y of [-edge,edge])line(c,p(0,y),p(deep,y),'#6e8397',2);label(c,'탑재부 '+fmt(site.truck_inner_width_m,2)+' m',w-12,24,'#9bb0c4','right');}
 if(dc){const q=p(dc[0],dc[1]);dot(c,q,7,null,'#f2939c');line(c,[q[0]-5,q[1]],[q[0]+5,q[1]],'#f2939c');label(c,'depth 입구 중심',q[0]+10,q[1]-12,'#f2939c');}
 drawTrail(c,trail,p,'#7d93ab');
 for(const a of anchors){const pos=p(a.p[0],a.p[1]);dot(c,pos,a.kind==='accepted_after_skip'?6:a.kind==='first_anchor'?5:2,a.kind==='accepted_after_skip'?null:a.kind==='first_anchor'?'#73d9ac':'#e5ecf5',a.kind==='accepted_after_skip'?'#e6a0da':null);}
 if(latestRoute.length>1){
  const muted=s.blind||['FAULT','COMPLETE','RECOVER'].includes(s.state),color=muted?'#627c8b':'#52d4ea';
  for(let i=1;i<latestRoute.length;i++){line(c,p(...latestRoute[i-1]),p(...latestRoute[i]),color,i===1?3:1.6,i===1?[]:[6,6]);const pos=p(...latestRoute[i]);dot(c,pos,selectedPoint===i-1?12:10,'#142231',color);label(c,String(i),pos[0],pos[1],color,'center');}
 }
 if(finite(view.target)){const target=p(view.target,0);dot(c,target,8,null,'#73d9ac');line(c,[target[0]-4,target[1]],[target[0]+4,target[1]],'#73d9ac');line(c,[target[0],target[1]-4],[target[0],target[1]+4],'#73d9ac');label(c,'포크 정지 목표',target[0],Math.min(h-30,target[1]+22),'#73d9ac','center');}
 if(tag){const pos=p(...tag);dot(c,pos,6,'#eac47a');label(c,'태그'+(site.tag_id==null?'':' '+site.tag_id),pos[0]+12,Math.max(18,pos[1]-16),'#eac47a');}
 if(body.length){c.beginPath();body.forEach((v,i)=>i?c.lineTo(...p(...v)):c.moveTo(...p(...v)));c.closePath();c.fillStyle='#52d4ea12';c.fill();c.strokeStyle='#52d4ea66';c.lineWidth=1;c.stroke();}
 if(pose){const pos=p(...pose),ang=pose[2];dot(c,pos,8,'#52d4ea','#c2f5fc');line(c,pos,[pos[0]+25*Math.cos(ang),pos[1]-25*Math.sin(ang)],'#c2f5fc',2);label(c,'지게차 기준점',Math.max(55,Math.min(w-60,pos[0])),Math.min(h-30,pos[1]+25),'#b8eef7','center');
  if(Array.isArray(g.fork_tip)&&g.fork_tip.length>=2&&g.fork_tip.every(finite)){const tip=p(...g.fork_tip);dot(c,tip,3,'#73d9ac');label(c,'포크 끝',tip[0]+7,tip[1]-10,'#73d9ac');}
 }else{label(c,'입구 기준 위치 수신 대기',w/2,h/2,'#b8c8d8','center');}
 label(c,'카메라 위치 기준 · 위쪽 +y',w-12,h-14,'#9bb0c4','right');
}
function drawOdom(){
 const {c,w,h}=surface('odom'),o=data.observation||{},points=vtrail.filter(Boolean);
 if(!points.length){label(c,'VIO 위치 수신 대기',w/2,h/2,'#9bb0c4','center');return;}
 const t=mapTransform(points,w,h,24),p=t.p,z=p(0,0);
 line(c,[0,z[1]],[w,z[1]],'#334a60');line(c,[z[0],0],[z[0],h],'#334a60');drawTrail(c,vtrail,p,'#eac47a');
 const v=points[points.length-1],a=p(...v);dot(c,a,4,'#ecf3fa');line(c,a,[a[0]+18*Math.cos(v[2]),a[1]-18*Math.sin(v[2])],'#ecf3fa',2);
 label(c,localAvailable(o)?'VIO 궤적':'마지막 유효 궤적',10,15);const metres=Math.max(.1,Number((w/3/t.scale).toPrecision(1)));line(c,[12,h-17],[12+metres*t.scale,h-17],'#8097ad',2);label(c,metres+' m',12,h-29);
}
function drawBev(){
 const {c,w,h}=surface('bev'),o=data.observation||{},view=data.cfg_view||{},site=view.site||{},dep=o.depth||{},bev=o.depth_bev||{},cells=Array.isArray(bev.cells)?bev.cells:[];
 const ok=dep.ok&&Array.isArray(dep.left_body)&&Array.isArray(dep.right_body);
 // entrance frame from the depth pair (body metres): origin = midpoint, n = into the truck, tau = towards the left edge
 let cx=0,cy=0,nx=1,ny=0;
 if(ok){const l=dep.left_body,r=dep.right_body;cx=(l[0]+r[0])/2;cy=(l[1]+r[1])/2;let tx=l[0]-r[0],ty=l[1]-r[1];const len=Math.hypot(tx,ty)||1;tx/=len;ty/=len;nx=ty;ny=-tx;if(nx<0){nx=-nx;ny=-ny;}}
 const tx=-ny,ty=nx;   // tau = n rotated +90 deg (left)
 const E=(x,y)=>ok?[(x-cx)*nx+(y-cy)*ny,(x-cx)*tx+(y-cy)*ty]:[x,y];   // body -> entrance frame (or body frame when no pair)
 const half=finite(site.truck_inner_width_m)?site.truck_inner_width_m/2:.7,deep=finite(site.truck_depth_m)&&site.truck_depth_m>0?site.truck_depth_m:1.5;
 const tip=finite(site.camera_to_fork_tip_m)?site.camera_to_fork_tip_m:.2,rear=finite(site.rear_extent_m)?site.rear_extent_m:2,vw=finite(site.vehicle_width_m)?site.vehicle_width_m/2:.64;
 const body=[[tip,vw],[tip,-vw],[-rear,-vw],[-rear,vw]].map(([x,y])=>E(x,y));
 const pts=[...body,...cells.map(([x,y])=>E(x,y)),ok?[0,0]:E(3,0),ok?[deep,half]:E(3,half),ok?[deep,-half]:E(3,-half)];
 // map: up = +x of the frame (into the truck), left = +y
 const xs=pts.map(v=>v[0]),ys=pts.map(v=>v[1]),x0=Math.min(...xs),x1=Math.max(...xs),y0=Math.min(...ys),y1=Math.max(...ys);
 const pad=18,scale=Math.max(.01,Math.min((h-2*pad)/Math.max(1.5,x1-x0),(w-2*pad)/Math.max(1.5,y1-y0)));
 const P=(x,y)=>[w/2-(y-(y0+y1)/2)*scale,h/2-(x-(x0+x1)/2)*scale];
 const raw=60/scale,base=Math.pow(10,Math.floor(Math.log10(raw))),step=[1,2,5,10].map(v=>v*base).find(v=>v>=raw)||base*10;
 for(let x=Math.floor(x0/step)*step;x<=x1;x+=step){const a=P(x,y0);line(c,[0,a[1]],[w,a[1]],'#25394b',.6);}
 const cs=Math.max(1,bev.cell_m*scale||2);c.fillStyle='#d9e6f2';for(const [x,y] of cells){const q=E(x,y),v=P(q[0],q[1]);c.fillRect(v[0]-cs/2,v[1]-cs/2,cs,cs);}
 if(ok){const a=P(0,half),b=P(0,-half);line(c,P(0,y0),P(0,y1),'#eac47a',1.5,[6,5]);line(c,a,P(deep,half),'#6e8397',2);line(c,b,P(deep,-half),'#6e8397',2);dot(c,a,4,'#eac47a');dot(c,b,4,'#eac47a');line(c,P(x0,0),P(x1,0),'#f2939c',1,[3,5]);label(c,'입구면',P(0,y1)[0]+4,P(0,y1)[1]-8,'#eac47a');}
 c.beginPath();body.forEach((v,i)=>{const q=P(v[0],v[1]);i?c.lineTo(...q):c.moveTo(...q);});c.closePath();c.fillStyle='#73d9ac22';c.fill();c.strokeStyle='#73d9ac';c.lineWidth=1.5;c.stroke();
 const f=E(tip,0),fp=P(f[0],f[1]),g=E(tip+1.5,0),gp=P(g[0],g[1]);line(c,fp,gp,'#73d9ac',1,[4,4]);
 const dm=(data.controller||{}).depth||{};
 set('bev-lat',(finite(dm.lateral_error_m)?signed(dm.lateral_error_m,3):'—')+' m');set('bev-dist',(finite(dm.distance_m)?fmt(dm.distance_m,2):finite(dep.x_body_m)?fmt(dep.x_body_m,2):'—')+' m');set('bev-width',(finite(dep.width_m)?fmt(dep.width_m,2):'—')+' m');
 set('bev-note',ok?'입구 고정 · 쌍 방향 '+signed(dep.pair_heading_deg,1)+'°':cells.length?'입구 못 찾음 ('+(dep.reason||'—')+') · 차 기준':'depth 측정 대기');
 label(c,ok?'입구 기준 · 위쪽 = 안쪽':'차 기준 · 위쪽 = 앞',10,15);
}
async function tick(){
 const abort=new AbortController(),timeout=setTimeout(()=>abort.abort(),2500);
 try{const response=await fetch('/state',{cache:'no-store',signal:abort.signal});if(!response.ok)throw Error('HTTP '+response.status);const next=await response.json();if(!next||typeof next!=='object'||Array.isArray(next))throw Error('invalid state');data=next;connected=true;lastReceive=performance.now();render();}
 catch(error){connected=false;refreshHealth();set('status','상태 수신 실패 · '+(error.name==='AbortError'?'응답 시간 초과':error.message));}
 finally{clearTimeout(timeout);setTimeout(tick,250);}
}
// One JPEG request at a time: a slow link must not accumulate image requests. The three streams take turns.
const STREAMS=['ir','color','depth'];let imageTimer=null,imageDeadline=null,streamIndex=0,activeStreams=STREAMS.slice();
const previews=Object.fromEntries(STREAMS.map(k=>[k,$('preview-'+k)]));
function scheduleImage(delay=120){clearTimeout(imageDeadline);clearTimeout(imageTimer);imageTimer=setTimeout(pollImage,delay);}
for(const k of STREAMS){const img=previews[k];
 img.onload=()=>{if(!img.naturalWidth){img.onerror();return;}lastImage=performance.now();img.hidden=false;$('camera-placeholder-'+k).hidden=true;set('image-age-'+k,'영상 수신 중');scheduleImage();};
 img.onerror=()=>{img.hidden=true;$('camera-placeholder-'+k).hidden=false;set('camera-message-'+k,lastImage?'영상 재연결 대기':'카메라 연결 대기');scheduleImage(400);};}
function pollImage(){if(document.hidden||!activeStreams.length){imageTimer=setTimeout(pollImage,500);return;}const k=activeStreams[streamIndex%activeStreams.length];streamIndex++;const img=previews[k];
 imageDeadline=setTimeout(()=>{img.removeAttribute('src');img.onerror();},4000);img.src='/image?s='+k+'&t='+Date.now();}
window.addEventListener('resize',drawMaps);
if(typeof ResizeObserver!=='undefined'){const ro=new ResizeObserver(drawMaps);ro.observe($('map').parentElement);ro.observe($('odom').parentElement);ro.observe($('bev').parentElement);}
setInterval(refreshHealth,500);render();tick();pollImage();
</script>
</body>
</html>
'''

def _body_to_pixel(pt_body,T_body_cam,model):
    """Body-frame point -> pixel through a camera model (cv2 projection with its lens coefficients); None if behind/near."""
    import cv2
    T=np.asarray(T_body_cam,float);cam=(np.asarray(pt_body,float)-T[:3,3])@T[:3,:3]
    if cam[2]<=.15:return None
    K=np.array([[model['fx'],0,model['cx']],[0,model['fy'],model['cy']],[0,0,1.]])
    uv=cv2.projectPoints(cam.reshape(1,1,3),np.zeros(3),np.zeros(3),K,np.array(model.get('coeffs') or [0.]*5,float))[0].reshape(2)
    if not np.all(np.isfinite(uv)):return None
    return int(round(uv[0])),int(round(uv[1]))

def encode_preview(image,obs,cfg,stream='infrared',cal=None):
    """One dashboard panel: 'infrared'/'ir' = IR + VIO points, 'color' = colour + tag outline + parking guide lines + depth
    entrance, 'depth' = greyscale depth + entrance edges/centre (2026-10-22). Display only; nothing here is a measurement."""
    import cv2
    cal=cal or {};stream={'ir':'infrared'}.get(stream,stream)
    depth=obs.get('depth') or {}
    if stream=='depth':
        d=image.astype(np.float32)*float(cal.get('depth_scale',obs.get('depth_scale',.001)));top=float(cfg.get('depth_max_m',6.))
        valid=(d>0)&np.isfinite(d)
        g=(255-215*np.clip(d,0,top)/top).astype(np.uint8);g[~valid]=0
        out=cv2.cvtColor(g,cv2.COLOR_GRAY2BGR);h,w=d.shape
        if depth.get('ok'):
            for u in (depth.get('u_left'),depth.get('u_right')):
                if u is not None:cv2.line(out,(int(u),0),(int(u),h-1),(0,220,255),2)
            if depth.get('u_goal') is not None:ug=int(depth['u_goal']);cv2.line(out,(ug,0),(ug,h-1),(0,0,255),2);cv2.circle(out,(ug,h//2),9,(0,0,255),2)
            txt='entrance: width %.2f m at %.2f m, centre %+.3f m left of the body axis'%(depth.get('width_m',0.),depth.get('x_body_m',0.),depth.get('lateral_body_m',0.))
        else:
            txt='entrance %s: %s'%(('%.2f m'%depth['width_required_m']) if depth.get('width_required_m') else '','%s'%depth.get('reason','not computed'))
            cnd=depth.get('candidate') or {}
            if cnd.get('ok'):   # nearest opening of any width: orange, display only
                for u in (cnd.get('u_left'),cnd.get('u_right')):
                    if u is not None:cv2.line(out,(int(u),0),(int(u),h-1),(0,140,255),1)
                if cnd.get('u_goal') is not None:ug=int(cnd['u_goal']);cv2.line(out,(ug,0),(ug,h-1),(0,140,255),2);cv2.circle(out,(ug,h//2),9,(0,140,255),2)
                txt+=' | nearest opening (orange): width %.2f m at %.2f m, centre %+.3f m, edge dz %.2f m %s'%(cnd.get('width_m') or 0.,cnd.get('z_m') or 0.,cnd.get('goal_lateral_m') or 0.,cnd.get('dz_m') or 0.,'OK' if cnd.get('depth_match') else 'UNEQUAL')
            elif cnd:txt+=' | nearest opening: '+str(cnd.get('reason'))
        if depth.get('expected_u') is not None:cv2.line(out,(int(depth['expected_u']),h//2-20),(int(depth['expected_u']),h//2+20),(180,180,180),1)
        cv2.rectangle(out,(0,h-44),(w-1,h-1),(0,0,0),-1)
        cv2.putText(out,txt,(10,h-26),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,255) if depth.get('ok') else (0,165,255),1)
        cv2.putText(out,'valid %d%% | white near -> dark %.0f m, black = none | grey tick = expected centre'%(int(100*valid.mean()),top),(10,h-8),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
    else:
        out=image.copy()
        if out.ndim==2:out=cv2.cvtColor(out,cv2.COLOR_GRAY2BGR)
    if stream=='color':
        # Tag corners are colour-image coordinates.
        if obs['tag'].get('stream')=='color' and obs['tag'].get('corners'):cv2.polylines(out,[np.array(obs['tag']['corners'],int)],True,(0,255,0) if obs['tag']['ok'] else (0,200,255),2)
        T=cal.get('T_body_camera');model=cal.get('camera_model');site=cal.get('site') or {}
        if T is not None and model:
            vw=float(site.get('vehicle_width_m') or 1.28)/2;tw=float(site.get('truck_inner_width_m') or 1.4)/2
            bias=math.radians(float(site.get('forward_path_bias_deg') or 0.))   # the path actually driven: shown as the lines' direction
            for yb,col,wd in ((vw,(0,220,0),2),(-vw,(0,220,0),2),(tw,(255,200,0),1),(-tw,(255,200,0),1)):
                uv=[]
                for x in np.linspace(.4,6.,24):
                    q=_body_to_pixel((x*math.cos(bias)-yb*math.sin(bias),x*math.sin(bias)+yb*math.cos(bias),0.),T,model)
                    if q is not None:uv.append(q)
                if len(uv)>1:cv2.polylines(out,[np.array(uv,np.int32)],False,col,wd)
            for xm in (1.,2.,3.,4.,5.):
                a=_body_to_pixel((xm,vw,0.),T,model);b=_body_to_pixel((xm,-vw,0.),T,model)
                if a and b:cv2.line(out,a,b,(0,255,0),1);cv2.putText(out,'%.0f m'%xm,(b[0]+4,b[1]-2),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,0),1)
            if depth.get('ok') and depth.get('left_body') and depth.get('right_body'):
                hz=float(np.asarray(cal.get('T_body_ir',np.eye(4)),float)[2,3])   # entrance marks at camera height
                for pt in (depth['left_body'],depth['right_body']):
                    q=_body_to_pixel((pt[0],pt[1],hz),T,model)
                    if q:cv2.line(out,(q[0],0),(q[0],out.shape[0]-1),(0,220,255),2)
                cq=_body_to_pixel((depth['x_body_m'],depth['lateral_body_m'],hz),T,model)
                if cq:cv2.line(out,(cq[0],0),(cq[0],out.shape[0]-1),(0,0,255),2);cv2.circle(out,cq,10,(0,0,255),2)
    if stream=='infrared':
        for p in obs['odom'].get('tracked_points',[]):cv2.circle(out,tuple(map(int,p)),2,(255,200,0),-1)
    cv2.putText(out,'tag: '+obs['tag']['reason']+' / odom: '+obs['odom']['reason'],(10,24),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,220,255),1)
    scale=min(1.,cfg['width']/out.shape[1]);out=cv2.resize(out,None,fx=scale,fy=scale)
    ok,buf=cv2.imencode('.jpg',out,[cv2.IMWRITE_JPEG_QUALITY,cfg['jpeg_quality']])
    return bytes(buf) if ok else b''

class Dashboard:
    def __init__(self,port):
        self.lock=threading.Lock();self.value={};self.jpeg={};owner=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*a):pass
            def do_GET(self):
                with owner.lock:value=owner.value;images=dict(owner.jpeg)
                if self.path.startswith('/state'):
                    o=value.get('observation') or {};body=json.dumps(dict(value,age_s=None if 'arrival' not in o else time.monotonic()-o['arrival']),default=numpy_json).encode();kind='application/json'
                elif self.path.startswith('/image'):
                    q=parse_qs(urlparse(self.path).query);stream=(q.get('s') or ['ir'])[0]
                    body=images.get(stream) or images.get('ir') or b'';kind='image/jpeg'
                else:body=HTML.encode();kind='text/html; charset=utf-8'
                try:
                    self.connection.settimeout(.5);self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
                except (OSError,TimeoutError):pass
        self.server=ThreadingHTTPServer(('127.0.0.1',port),Handler);self.server.daemon_threads=True
        threading.Thread(target=self.server.serve_forever,daemon=True).start()
    def update(self,value,preview):
        with self.lock:
            self.value=value
            if preview:
                images=preview[1]
                if isinstance(images,dict):self.jpeg.update({k:v for k,v in images.items() if v})
                elif images:self.jpeg['ir']=images
    def close(self):self.server.shutdown();self.server.server_close()
