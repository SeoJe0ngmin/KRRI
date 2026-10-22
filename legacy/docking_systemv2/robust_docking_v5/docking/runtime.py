"""Supervisor: bounded queues; independent camera/planner/CAN processes.

Blocking SDK reads, disk I/O, JPEG and route search never run in this loop.
This is deadline-monitored soft real time, not a hard real-time guarantee.
"""
import argparse
import json
import math
import multiprocessing as mp
import os
import queue
import select
import signal
import sys
import time
import uuid
from pathlib import Path
from .config import load,COMMON,WORK
from .controller import Controller
from .geometry import cut_position,remaining,front,arrival_y
from .recording import Recorder
from .console import Console

CAN_CONFIG=dict(channel=0,bitrate=500000,diagnostics=dict(enabled=True,
    max_ack_age_s=.2,max_write_gap_s=.15,poll_budget_s=.001,status_period_s=.1,
    max_rx_per_poll=64,max_pending=4096))

def drain(q):
    last=None
    for _ in range(8):
        try:last=q.get_nowait()
        except queue.Empty:break
    return last

def run(args,cfg_override=None,stop_when=None):
    cfg=cfg_override or load(args.common,args.mode,require_mount=args.command!='measure')
    sensor=cfg['sensor'];live=args.command=='drive';active=args.command!='measure'
    if active and (getattr(args,'no_tag',False) or getattr(args,'no_odom',False)):
        raise ValueError('--no-tag/--no-odom은 measure에서만 사용 가능합니다')
    if getattr(args,'no_tag',False):sensor['tag']['enabled']=False
    if getattr(args,'no_odom',False):sensor['odom']['enabled']=False
    if live and (not args.execute or not sys.stdin.isatty()):raise ValueError('실주행은 --execute와 대화형 SSH 터미널이 필요합니다')
    cut=cut_position(cfg) if active else None
    output=Path(args.output) if args.output else WORK/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]+'_'+args.command)
    output.mkdir(parents=True,exist_ok=False)
    (output/'config_used.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2))
    ctx=mp.get_context('spawn');stop=ctx.Event();results=ctx.Queue(1);previews=ctx.Queue(1)
    hints=ctx.Array('d',[0.,1.,0.]);fast=ctx.Array('d',[0.]*8)
    from .camera import worker
    camera=ctx.Process(target=worker,args=(sensor,stop,results,previews,str(output),args.view),kwargs=dict(motion_hint=hints,fast_imu=fast),daemon=True)
    planner=None;can=None;dash=None;obs=None;controller=None;last_print=0;last_capture=None
    rec=Recorder(str(output),sensor['logging'],stream='control');t0=time.monotonic()
    console=Console();failure=None;fast_now=[0.]*8
    status=dict(state='MEASURE',command='stop');terminal='stopped';old_handlers={};stdin_flags=None;buf=b''
    def interrupt(signum,frame):raise KeyboardInterrupt
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):old_handlers[sig]=signal.signal(sig,interrupt)
    try:
        camera.start()
        if active:
            from .planner import worker as plan_worker
            requests=ctx.Queue(1);plans=ctx.Queue(1)
            planner=ctx.Process(target=plan_worker,args=(cfg,requests,plans,stop),daemon=True);planner.start()
            controller=Controller(cfg,cut,t0)
            stdin_flags=os.get_blocking(sys.stdin.fileno());os.set_blocking(sys.stdin.fileno(),False)
        if live:
            from .canbus import Can
            can=Can(CAN_CONFIG,cfg['tuning']['lease_s'])
        if args.view:
            from .webview import Dashboard
            dash=Dashboard(args.port)
            print('화면: SSH 포트 전달 후 http://127.0.0.1:%d'%args.port,flush=True)
        print('출력: '+str(output),flush=True)
        print('정지 상태로 준비 측정 중. READY 이후 Enter로 출발. s+Enter / Ctrl+C 정지.' if active else '측정 전용: CAN을 열지 않습니다.',flush=True)
        last_tick=time.monotonic();preparation_saved=False
        while True:
            now=time.monotonic();loop_gap=now-last_tick;last_tick=now
            update=drain(results)
            if update is not None:
                if 'worker_error' in update:raise RuntimeError(update['worker_error'])
                obs=update
            if not camera.is_alive() or stop.is_set():raise RuntimeError('camera worker stopped')
            if obs is None and now-t0>30:raise RuntimeError('camera startup timeout')
            enter=False
            if active and select.select([sys.stdin],[],[],0)[0]:
                chunk=os.read(sys.stdin.fileno(),4096)
                if not chunk:terminal='ssh_eof';break
                buf+=chunk
                while b'\n' in buf:
                    line,buf=buf.split(b'\n',1);line=line.strip().lower()
                    if line in (b's',b'q',b'stop'):raise KeyboardInterrupt
                    if not line:enter=True
                if len(buf)>4096:raise ValueError('terminal input overflow')
            if can:
                health=can.status()
                if health['latched'] or not health['alive'] or health['tx_worker_age_s']>cfg['tuning']['lease_s']:
                    raise RuntimeError('CAN watchdog/transport: '+str(health))
            if active:
                if not planner.is_alive():raise RuntimeError('planner worker stopped')
                if loop_gap>cfg['tuning']['lease_s'] and controller.command!='stop':raise RuntimeError('supervisor deadline missed')
                reply=drain(plans)
                if reply:controller.accept_plan(reply,now,obs)
                # Raw gyro freshness is independent of delayed video snapshots.
                if fast.get_lock().acquire(False):
                    try:fast_now=fast[:]
                    finally:fast.get_lock().release()
                if controller.command!='stop' and (not fast_now[7] or now-fast_now[1]>cfg['tuning']['imu_age_s'] or fast_now[6]):
                    controller.recover(now,'fast_imu_unavailable')
                status=controller.step(now,obs,enter)
                if controller.request:
                    try:requests.put_nowait(controller.request);controller.request=None
                    except queue.Full:pass
                if can:can.set(status['command'])
                if controller.state=='WAIT_ENTER' and not preparation_saved:
                    rec.put(controller.preparation_report,'startup_measurement.json');preparation_saved=True
                    console.write('READY: 준비 완료. Enter를 누르면 차량이 움직입니다.' if live else 'READY: Enter로 shadow 시작(CAN 없음).')
            with hints.get_lock():
                neutral=status['command']=='stop';since=hints[2] if hints[1]==1 and neutral else now
                hints[:]=[now,float(neutral),since]
            if update is not None:
                if last_capture is not None and update['capture_id']<=last_capture:raise RuntimeError('nonmonotonic observation')
                last_capture=update['capture_id']
                rec.put(dict(now=now,status=status,loop_gap_s=loop_gap,observation_capture=last_capture,can=can.status() if can else None))
            if rec.status().get('error'):raise RuntimeError(rec.status()['error'])
            if dash:
                geometry=None
                if obs and obs.get('pose') is not None:
                    p=obs['pose'];geometry=dict(remaining_m=float(remaining(p,cfg)),
                        lateral_m=p[1],heading_deg=p[2]*180/3.141592653589793,
                        fork_tip=front(p,cfg).tolist(),predicted_arrival_y_m=float(arrival_y(p,cfg)))
                    if not math.isfinite(geometry['predicted_arrival_y_m']):geometry['predicted_arrival_y_m']=None
                dash.update(dict(mode=args.command,controller=status,observation=obs,geometry=geometry,cfg_view=dict(site=cfg['site'],target=cfg['target_front_x']),control_recording=rec.status()),drain(previews))
            if now-last_print>=2:
                perf=(obs or {}).get('performance',{})
                console.write('%s | %s | camera %.1f / process %.1f fps | %s'%(status['state'],status['command'],perf.get('camera_fps',0),perf.get('processed_fps',0),status.get('reason','')));last_print=now
            if status['state'] in ('FAULT','COMPLETE'):terminal=status.get('reason',status['state']);break
            if update is not None and stop_when and stop_when(update):terminal='measurement_complete';break
            if args.seconds and now-t0>=args.seconds:terminal='requested_duration';break
            stop.wait(.01)
    except KeyboardInterrupt:
        terminal='operator_stop'
    except Exception as exc:
        terminal='error';failure=type(exc).__name__+': '+str(exc)
        raise
    finally:
        # STOP/latch before joins, disk flush, UI shutdown or other blocking work.
        if can:can.abort()
        stop.set()
        if can:can.close()
        if stdin_flags is not None:os.set_blocking(sys.stdin.fileno(),stdin_flags)
        for sig,fn in old_handlers.items():signal.signal(sig,fn)
        for process in (camera,planner):
            if process and process.pid:
                process.join(3)
                if process.is_alive():process.terminate();process.join(1)
        if dash:dash.close()
        rec.put(dict(reason=terminal,error=failure,last_status=status,can_enabled=live,elapsed_s=time.monotonic()-t0),'session.json');rec.close()
        console.close()
    if status['state']=='FAULT':raise RuntimeError(status['reason']+' (로그: '+str(output)+')')
    return output

def parser():
    p=argparse.ArgumentParser(description='v5 IR/depth/IMU + receding-horizon docking')
    p.add_argument('command',choices=['measure','shadow','drive'])
    p.add_argument('--common',default=str(COMMON));p.add_argument('--mode',choices=['central','left'],default='central')
    p.add_argument('--seconds',type=float,default=0);p.add_argument('--view',action='store_true');p.add_argument('--port',type=int,default=8765)
    p.add_argument('--execute',action='store_true');p.add_argument('--output')
    p.add_argument('--no-tag',action='store_true');p.add_argument('--no-odom',action='store_true')
    return p
