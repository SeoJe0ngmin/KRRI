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

# 2026-10-18: transmit only (v4 parity). Bus status/error counters are logged every 5 s like v4; nothing received stops the vehicle.
CAN_CONFIG=dict(channel=0,bitrate=500000,diagnostics=dict(enabled=True,status_period_s=5.))

def drain(q):
    last=None
    for _ in range(8):
        try:last=q.get_nowait()
        except queue.Empty:break
    return last

def read_fast_imu(shared, previous):
    """Nonblocking snapshot; a writer collision does not invalidate cached data."""
    lock=shared.get_lock()
    if not lock.acquire(False):return previous
    try:return shared[:]
    finally:lock.release()


def fast_imu_available(snapshot, now, max_age):
    return bool(snapshot[7] and not snapshot[6] and math.isfinite(snapshot[1])
                and now-snapshot[1]<=max_age)


def advance_capture(previous,update):
    # Startup video previews can precede a delayed first estimator result.
    # They never establish the control-observation ordering or grant readiness.
    if update.get('preview_only'):return previous
    if previous is not None and update['capture_id']<=previous:raise RuntimeError('nonmonotonic observation')
    return update['capture_id']

def note(args):
    """Attach a tape-measured result to a run folder (2026-10-20): work/<run>/manual_measurement.json, one entry per call."""
    folder=Path(args.folder)
    if not (folder/'observations.jsonl').exists():raise SystemExit('판 폴더가 아닙니다 (observations.jsonl 없음): '+str(folder))
    path=folder/'manual_measurement.json'
    entries=json.loads(path.read_text()) if path.exists() else []
    entries.append(dict(written_at=time.strftime('%Y-%m-%dT%H:%M:%S'),fork_lateral_cm=args.lateral_cm,heading_deg=args.heading_deg,
                        longitudinal_cm=args.longitudinal_cm,memo=args.memo,sign='좌우 왼쪽 +, 방향 왼쪽 +, 종방향 더 들어감 +'))
    path.write_text(json.dumps(entries,ensure_ascii=False,indent=2)+'\n');print('기록: '+str(path));return 0

def run(args,cfg_override=None,stop_when=None):
    if getattr(args,'command',None)=='note':return note(args)
    """Retry only recoverable IMU acquisition faults in measurement mode.

    Each attempt owns new processes/queues and an independent output directory.
    Drive/shadow faults always return through the existing CAN abort cleanup.
    """
    import copy
    for attempt in range(4):
        current=copy.copy(args)
        if attempt and getattr(args,'output',None):
            current.output=str(args.output)+'_retry_'+str(attempt)
        try:
            return _run_once(current,copy.deepcopy(cfg_override),stop_when)
        except RuntimeError as exc:
            recoverable=any(x in str(exc) for x in ('unbracketed_imu',))   # IMU gaps are no longer faults (2026-10-19)
            if args.command!='measure' or not recoverable or attempt==3:raise
            if stop_when is not None:
                reset=getattr(stop_when,'reset',None)
                if reset is None:raise
                reset()
            print(f'IMU 공백: 이전 측정 구간 폐기. 센서/OpenVINS 재시작 {attempt+1}/3 (각 시도 로그 별도 보존)',flush=True)
            time.sleep(2.)


def _run_once(args,cfg_override=None,stop_when=None):
    cfg=cfg_override or load(args.common,args.mode,require_mount=args.command!='measure')
    sensor=cfg['sensor'];active=args.command!='measure'
    live=args.command=='drive'
    if active and getattr(args,'no_tag',False):
        raise ValueError('--no-tag은 measure에서만 사용 가능합니다')
    if getattr(args,'no_tag',False):sensor['tag']['enabled']=False
    sensor['camera_only']=bool(getattr(args,'camera_only',False))
    sensor['mapping']['enabled']=getattr(args,'profile','core')=='mapped'
    if active and sensor['camera_only']:raise ValueError('camera-only는 measure에서만 가능합니다')
    if not sensor['camera_only']:
        from .ros_adapter import check_build_ready
        check_build_ready()
    if live and (not args.execute or not sys.stdin.isatty()):raise ValueError('실주행은 --execute와 대화형 SSH 터미널이 필요합니다')
    cut=None
    # work/<YYYYMMDD>/<HHMMSS>_<id>_<command>/ (2026-10-20 user request: one folder per day)
    output=Path(args.output) if args.output else WORK/time.strftime('%Y%m%d')/(time.strftime('%H%M%S')+'_'+uuid.uuid4().hex[:6]+'_'+args.command)
    output.mkdir(parents=True,exist_ok=False)
    (output/'config_used.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2))
    ctx=mp.get_context('spawn');stop=ctx.Event();results=ctx.Queue(1);previews=ctx.Queue(1);setup=ctx.Queue(1)
    hints=ctx.Array('d',[0.,1.,0.,0.]);fast=ctx.Array('d',[0.]*8)   # [now, neutral, neutral_since, baseline_lock]
    from .camera import worker
    camera=ctx.Process(target=worker,args=(sensor,stop,results,previews,str(output),args.view),kwargs=dict(motion_hint=hints,fast_imu=fast,setup=setup),daemon=True)
    planner=None;can=None;dash=None;obs=None;controller=None;last_print=0;last_capture=None
    rec=Recorder(str(output),sensor['logging'],stream='control');t0=time.monotonic()
    console=Console();failure=None;fast_now=[0.]*8
    status=dict(state='MEASURE',command='stop');terminal='stopped';old_handlers={};stdin_flags=None;buf=b''
    def interrupt(signum,frame):raise KeyboardInterrupt
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):old_handlers[sig]=signal.signal(sig,interrupt)
    try:
        camera.start()
        # stdin stays blocking: select() guards every read, and O_NONBLOCK on a shared
        # pty description would also make stdout/stderr raise BlockingIOError on an SSH stall.
        if args.view:
            from .webview import Dashboard
            dash=Dashboard(args.port)
            print('화면: SSH 포트 전달 후 http://127.0.0.1:%d'%args.port,flush=True)
        print('출력: '+str(output),flush=True)
        print('브레이크를 잡고 실제 정차 상태를 유지하세요. 진동·준비 측정 중. READY 이후 Enter로 출발. s+Enter / Ctrl+C 정지.' if active else '측정 전용: CAN을 열지 않습니다.',flush=True)
        last_tick=time.monotonic();preparation_saved=False;recording_warned=False
        while True:
            now=time.monotonic();loop_gap=now-last_tick;last_tick=now
            prepared=drain(setup)
            if prepared:
                cfg['sensor']['calibration']=prepared['calibration']
                (output/'config_used.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2))
                if active:
                    cut=cut_position(cfg)
                    from .planner import worker as plan_worker
                    requests=ctx.Queue(1);plans=ctx.Queue(1)
                    planner=ctx.Process(target=plan_worker,args=(cfg,requests,plans,stop),daemon=True);planner.start()
                    controller=Controller(cfg,cut,now)
                    if live:
                        from .canbus import Can
                        can=Can(CAN_CONFIG,cfg['tuning']['lease_s'])
            update=drain(results)
            if update is not None:
                if 'worker_error' in update:raise RuntimeError(update['worker_error'])
                obs=update
            if not camera.is_alive() or stop.is_set():
                error_file=output/'camera_error.txt'
                detail=error_file.read_text() if error_file.exists() else 'No Python traceback; inspect openvins.log and OS logs for native crash/OOM.'
                raise RuntimeError(f'camera worker stopped (exitcode={camera.exitcode}, logs={output})\n{detail}')
            if obs is None and now-t0>sensor['camera']['startup_timeout_s']+25:raise RuntimeError('camera startup timeout')
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
            if active and controller is not None:
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
                    try:
                        requests.put_nowait(controller.request);controller.request=None
                        if controller.await_plan:controller.await_plan['at']=now   # planner_timeout counts from hand-over, not creation
                    except queue.Full:pass
                if can:can.set(status['command'])
                if controller.state=='WAIT_ENTER' and not preparation_saved:
                    rec.put(controller.preparation_report,'startup_measurement.json');preparation_saved=True
                    console.write('READY: 준비 완료. Enter를 누르면 차량이 움직입니다.' if live else 'READY: Enter로 shadow 시작(CAN 없음).')
            with hints.get_lock():
                neutral=status['command']=='stop';since=hints[2] if hints[1]==1 and neutral else now
                hints[:]=[now,float(neutral),since,float(bool(controller and controller.baseline_done))]
            if update is not None:
                last_capture=advance_capture(last_capture,update)
                rec.put(dict(now=now,status=status,loop_gap_s=loop_gap,observation_capture=last_capture,can=can.status() if can else None))
            if rec.status().get('error') and not recording_warned:
                recording_warned=True   # Recording never stops the vehicle; it is reported once and in session.json.
                console.write('기록 오류 (주행은 계속): '+str(rec.status()['error']))
            if dash:
                geometry=None
                if obs and obs.get('pose') is not None:
                    p=obs['pose'];geometry=dict(remaining_m=float(remaining(p,cfg)),
                        lateral_m=p[1],heading_deg=p[2]*180/3.141592653589793,
                        fork_tip=front(p,cfg).tolist(),predicted_arrival_y_m=float(arrival_y(p,cfg)))
                    if not math.isfinite(geometry['predicted_arrival_y_m']):geometry['predicted_arrival_y_m']=None
                dash.update(dict(mode=args.command,controller=status,observation=obs,geometry=geometry,cfg_view=dict(site=cfg['site'],target=cfg['target_front_x'],mode=cfg['mode']),control_recording=rec.status()),drain(previews))
            if now-last_print>=2:
                perf=(obs or {}).get('performance',{})
                odom=(obs or {}).get('odom',{})
                tag=(obs or {}).get('tag') or {};tf=(obs or {}).get('tag_fusion') or {}
                tag_text=(tag.get('reason') or 'no_tag')+(' %.2fm'%tf['range_m'] if isinstance(tf.get('range_m'),(int,float)) else '')   # 2026-10-20: tag state on the terminal too
                console.write('%s | %s | camera %.1f / process %.1f fps | odom %s: %s | tag %s | %s'%(status['state'],status['command'],perf.get('camera_fps',0),perf.get('processed_fps',0),odom.get('ok',False),odom.get('reason','sensor_starting'),tag_text,status.get('reason','')));last_print=now
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
        if can:
            can.close()
            can_final=can.status()
            if can_final.get('diagnostics',{}).get('fault'):
                rec.put(can_final,'can_fault.json')
        if stdin_flags is not None:os.set_blocking(sys.stdin.fileno(),stdin_flags)
        for sig,fn in old_handlers.items():signal.signal(sig,fn)
        for process in (camera,planner):
            if process and process.pid:
                process.join(12)
                if process.is_alive():process.terminate();process.join(1)
        if dash:dash.close()
        rec.put(dict(reason=terminal,error=failure,last_status=status,can_enabled=live,elapsed_s=time.monotonic()-t0),'session.json');rec.close()
        console.close()
    if status['state']=='FAULT':
        # The controller stopped the vehicle for a stated reason; that is a result, not a crash (2026-10-19).
        print('정지(판단): '+status['reason']+' · 로그: '+str(output),flush=True)
    return output

def parser():
    p=argparse.ArgumentParser(description='v6 OpenVINS IR stereo/IMU + color AprilTag docking')
    p.add_argument('command',choices=['measure','shadow','drive','note'])
    p.add_argument('--folder',help='note: 판 폴더 (work/<시각>_drive)');p.add_argument('--lateral-cm',type=float,help='note: 줄자로 잰 포크 끝 좌우 [cm], 왼쪽 +')
    p.add_argument('--heading-deg',type=float,help='note: 잰 방향 [deg], 왼쪽 +');p.add_argument('--longitudinal-cm',type=float,help='note: 목표 대비 종방향 [cm], 더 들어감 +');p.add_argument('--memo',default='')
    p.add_argument('--common',default=str(COMMON));p.add_argument('--mode',choices=['central','left'],default='central')
    p.add_argument('--seconds',type=float,default=0);p.add_argument('--view',action='store_true');p.add_argument('--port',type=int,default=8765)
    p.add_argument('--execute',action='store_true');p.add_argument('--output')
    p.add_argument('--no-tag',action='store_true',help='measure only; skip tag detection')
    p.add_argument('--camera-only',action='store_true',help='measure only; no OpenVINS, no driving')
    p.add_argument('--profile',choices=['core','mapped'],default='core')
    return p
