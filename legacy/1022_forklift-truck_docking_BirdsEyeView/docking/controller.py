"""Nonblocking mission state machine (2026-10-22). All motion requires fresh measured feedback.

Flow (user decisions 2026-10-21/22):
  ROUGH       heading straight (one fine turn), sidestep whenever |lateral| >= one real step (at most twice), then ONE
              long drive that stops rough_stop_before_cut_m before the tag cut (tag ruler + VIO; a lost tag continues on
              the VIO; no progress for no_progress_s stops the leg).
  FINE_ENTER  stand still, re-zero the heading reference from the tag here (4 m: far better than the first window at 9 m).
  FINE        measure the entrance with the depth stream (the pair truck_inner_width_m apart), lateral error > tolerance ->
              one bundle (turn alpha, drive d, turn back straight; fine.py), else a straight leg <= fine_step_max_m, until
              final_check_distance_m before the entrance plane. Depth never found -> tag lateral while the tag is in view;
              found then lost -> the last centre carried by the VIO.
  FINAL       last depth check: |lateral| <= final_lateral_tol_m and |heading| <= final_heading_tol_deg -> ENTRY; else a fine
              turn, or back out (never behind the tag cut) and bundle again; fine_corrections_max in all -> stop, record,
              COMPLETE without entering.
  ENTRY       one VIO leg to the fork-tip target along heading -forward_path_bias_deg; timed fallback if the VIO is lost.
  FALLBACK    no depth measurement at all: tag-pose blind entry on the fused pose (bounded aim turns on the predicted arrival).
The six-step beam planner (planner.plan) is no longer called; planner.Responses only tunes the braking reserve.
"""
import math
import numpy as np
from .geometry import wrap,remaining,arrival_y,inside_truck,path_clear,front,finite,moved,path_bias
from .planner import Responses
from .fine import plan_bundle,swing_back_y,entrance_in_dock,lateral_error,distance_to_entrance,final_check_distance

PHASES=('ROUGH','FINE_ENTER','FINE','FINAL','ENTRY','FALLBACK')

class Controller:
    def __init__(self,cfg,cut,now):
        self.cfg=cfg;self.t=cfg['tuning'];self.cut=cut;self.created=now
        self.state='PREPARING';self.reason='stationary_preparation';self.command='stop'
        self.mission_start=None;self.stable_start=None;self.last_capture=None
        self.prep_seconds=0.;self.prep_last=None;self.prep_anchor=None;self.prep_pause=None
        self.responses=Responses();self.request=None   # request stays None: no planner process (runtime checks it)
        self.action=None;self.brake=None;self.blind=False
        self.search_start=None;self.sweep=0.;self.motion_start=None;self.stop_at=None
        self.recovery_at=None;self.post_settle='PLAN'
        self.events=[];self.initialized_at=None;self.preparation_report={};self.last_seen=None
        self.final_report=None          # entry outcome: what ended the leg and the measured error (record only)
        self.last_lateral_check=None    # the last |arrival| <= tolerance decision with its σ (record only, 2026-10-19)
        self.baseline_done=False;self.baseline_seconds=0.;self.baseline_last=None   # 30 s at rest with the tag in view, after Enter and after any search
        self.baseline_lost_since=None;self.overshoot_backed=False
        # ---- flow state ----
        self.phase='ROUGH';self.legs=[];self.route_points=[];self.route=[]
        self.sidesteps=0;self.heading_turns=0;self.no_progress_stops=0
        self.depth_samples=[];self.depth_collect_since=None;self.depth_last_stamp=None
        self.depth_seen=False;self.depth_centre_dock=None;self.depth_measure=None
        self.rezero_request=0;self.rezero_since=None;self.rezero_report=None
        self.corrections=0;self.fallback_corrections=0;self.fine_bundles=0

    # ---------------------------------------------------------------- reporting
    def fault(self,reason):
        self.state='FAULT';self.reason=reason;self.command='stop';self.request=None
        return self.status()

    def status(self):
        return dict(state=self.state,phase=self.phase,reason=self.reason,command=self.command,blind=self.blind,final_report=self.final_report,
                    search_active=bool(self.action and self.action.get('after')=='SEARCH'),
                    lateral_check=self.last_lateral_check,baseline_done=self.baseline_done,baseline_seconds=self.baseline_seconds,
                    events=list(self.events),search_sweep_deg=math.degrees(self.sweep),
                    route=[dict(kind=l['kind'],value=float(l['value']),label=l.get('label')) for l in self.legs],
                    route_points=[list(p) for p in self.route_points],
                    preparation=dict(self.preparation_report),responses={k:dict(v) for k,v in self.responses.data.items()},
                    depth=self.depth_measure,depth_centre_dock=None if self.depth_centre_dock is None else list(self.depth_centre_dock),
                    rezero_request=self.rezero_request,rezero=self.rezero_report,corrections=self.corrections,sidesteps=self.sidesteps,
                    heading_target_deg=math.degrees(self.heading_target()),
                    estimate_use='response estimates affect braking only, never measured pose')

    def fresh(self,now,o,age=None):
        # age: sensor_age_s (0.3 s) for decisions at rest; motion_age_s (3 s) while moving.
        age=self.t['sensor_age_s'] if age is None else age
        return bool(o and not o.get('clock_fault') and o.get('sensor_ready') and not o.get('imu_bad') and
                    finite(o.get('local_pose',[])) and len(o.get('local_pose',[]))==3 and
                    (not o.get('usable') or (o.get('pose') is not None and finite(o['pose']))) and
                    0<=now-o.get('arrival',-math.inf)<=age and
                    o.get('transport_lag_s',0)<=age and o.get('local_valid'))

    def heading_target(self):
        """Heading at which a straight drive actually tracks the truck axis: minus the measured drift (forward_path_bias_deg)."""
        return wrap(-path_bias(self.cfg))

    def accept_plan(self,reply,now,o):
        return   # kept for the runtime contract; no planner requests are issued

    # ---------------------------------------------------------------- motion primitives
    def set_legs(self,now,o,legs,label):
        """Queue legs (dict kind='turn' value=absolute heading | kind='drive' value=signed metres) and preview them for the map."""
        self.legs=list(legs);p=np.array(o['pose'],float);pts=[p.tolist()];q=p
        for l in self.legs:
            q=moved(q,wrap(l['value']-q[2]),0.,self.cfg['pivot']) if l['kind']=='turn' else moved(q,0.,l['value'],self.cfg['pivot'])
            pts.append(q.tolist())
        self.route_points=pts
        self.events.append(dict(t=now,event='legs',label=label,legs=[dict(kind=l['kind'],value=float(l['value']),label=l.get('label')) for l in self.legs],pose=[float(v) for v in p]))

    def next_leg(self,now,o):
        """Execute the next queued leg. Tag visibility is not required (the fused pose is propagated by the VIO)."""
        p=o.get('pose')
        while self.legs:
            leg=self.legs.pop(0)
            if p is None:break
            if leg['kind']=='turn':
                turn=wrap(leg['value']-p[2])
                if abs(turn)<math.radians(self.t['minimum_turn_fine_deg'])/2:continue   # fine turns are accepted from half the floor
                strength=leg.get('strength') or ('fine' if abs(turn)<=math.radians(self.t['turn_max_deg'])+1e-9 else 'slow')
                self.begin(now,o,'rotate_'+('left' if turn>0 else 'right')+'_'+strength,abs(turn),'LEGS',label=leg.get('label'));return
            d=float(leg['value'])
            if abs(d)<self.t['minimum_translation_m']:continue
            if not path_clear(p,0,d,self.cfg):self.fault('truck_side_clearance');return
            self.begin(now,o,'backward' if d<0 else 'forward',abs(d),'LEGS',label=leg.get('label'))
            self.action.update(stop_x=leg.get('stop_x'),watch=bool(leg.get('watch')),watch_sign=leg.get('watch_sign'))
            return
        self.legs=[];self.state='PLAN';self.reason='legs_done'

    def begin(self,now,o,cmd,target,after,label=None):
        r=self.responses.get(cmd);expected=r['startup']+target/max(1e-6,r['speed'])
        turn='rotate' in cmd
        default=self.t['search_turn_timeout_s'] if after=='SEARCH' else self.t['rotation_timeout_s'] if turn else self.t['translation_timeout_s']
        self.action=dict(command=cmd,target=target,start_local=o['local_pose'][:],generation=o['tracking_generation'],
                         epoch=o['motion_epoch'],started=now,after=after,speed=0.,progress=0.,previous=0.,last=now,blind=False,label=label,
                         # 2026-10-20 KW: the tag-anchored dock-frame pose is a second ruler for straight steps (the VIO alone froze twice).
                         start_world=list(o['pose']) if o.get('usable') and o.get('pose') is not None else None,progress_parts=None,
                         # time cap scaled to the leg (a 5 m rough leg takes ~20 s): never the reason a healthy leg ends.
                         cap_s=max(default,self.t['entry_timed_scale']*expected+3.))
        self.state='MOVING';self.command=cmd;self.reason='measured_motion_feedback';self.motion_start=None
        self.depth_collect_since=None;self.depth_samples=[]   # a measurement window belongs to one stop

    def begin_entry(self,now,o,distance,source):
        """ENTRY (2026-10-19 user decision kept): ONE leg for the whole remaining distance, stopped on OpenVINS progress.
        If the VIO is lost on the way the leg continues on the time model instead of stopping inside the truck."""
        target=max(0.,float(distance))
        if target<.005:self.state='COMPLETE';self.reason='entry_distance_zero_recorded';return
        if not path_clear(o['pose'],0,target,self.cfg):self.fault('truck_side_clearance');return
        r=self.responses.get('forward')
        self.begin(now,o,'forward',target,'COMPLETE',label='entry')
        self.action.update(blind=True,timed=False,hold_s=r['startup']+target/r['speed'])
        self.blind=True;self.phase='ENTRY';self.reason='entry_vio_progress'
        self.events.append(dict(t=now,event='entry',distance_m=target,source=source,pose=[float(v) for v in o['pose']],
                                heading_deg=float(math.degrees(o['pose'][2])),lateral_error_m=None if self.depth_measure is None else self.depth_measure.get('lateral_error_m')))

    def progress(self,o):
        a=self.action;p=np.array(o['local_pose']);start=np.array(a['start_local'])
        if 'rotate' in a['command']:
            return wrap(p[2]-start[2])*(1 if 'left' in a['command'] else -1)
        delta=p[:2]-start[:2];signed=float(delta @ [math.cos(start[2]),math.sin(start[2])])
        local=-signed if a['command']=='backward' else signed
        world=None
        if a.get('start_world') is not None and o.get('usable') and o.get('pose') is not None and finite(o['pose']):
            w=np.array(o['pose']);s0=np.array(a['start_world']);dw=w[:2]-s0[:2]
            sw=float(dw @ [math.cos(s0[2]),math.sin(s0[2])]);world=-sw if a['command']=='backward' else sw
        a['progress_parts']=(local,world)
        # Whichever ruler says the step is done, stop (early rather than late).
        return local if world is None else max(local,world)

    def step_event(self,now,b,end,vio_ok):
        """One line per executed step (2026-10-20): what was commanded, what the stop decision saw, what happened after."""
        turn='rotate' in b['command'];learned=self.responses.get(b['command'])
        self.events.append(dict(t=now,event='step_done',command=b['command'],after=b.get('after'),label=b.get('label'),unit='rad' if turn else 'm',
            target=b.get('target'),stop_progress=b.get('stop_progress'),end_progress=end,
            coast=None if end is None or b.get('stop_progress') is None else end-b['stop_progress'],
            active_s=b['stop_time']-b['started'],dead_s=(b['motion_start']-b['started']) if b.get('motion_start') else None,
            settle_s=now-b['stop_time'],stop_reserve=b.get('stop_reserve'),stop_remaining_m=b.get('stop_remaining_m'),
            stop_obs_age_s=b.get('stop_obs_age_s'),stop_coast_est=b.get('stop_coast_est'),speed_est=b.get('speed'),progress_parts_vio_tag=b.get('progress_parts'),
            stop_cause=b.get('stop_cause'),timed=bool(b.get('timed')),vio_ok=bool(vio_ok),
            learned=dict(speed=learned['speed'],coast=learned['coast'],startup=learned['startup'],n=learned['n'])))

    def stop(self,now,post='PLAN',cause='reached'):
        self.command='stop';self.state='SETTLE';self.stop_at=now;self.post_settle=post
        if self.action:self.brake=dict(self.action,stop_progress=self.action['progress'],stop_time=now,motion_start=self.motion_start,stop_cause=cause)
        self.reason='observing_coast_and_rebound'

    def recover(self,now,reason):
        if self.action and self.action['after']=='SEARCH':
            self.fault('search_tracking_lost_unmeasured_angle');return
        self.command='stop';self.state='RECOVER';self.recovery_at=now;self.reason=reason
        self.action=None;self.brake=None;self.request=None
        # queued legs are dropped: the next PLAN re-measures and decides again from the actual pose
        self.legs=[];self.route_points=[]

    def lateral_within_tolerance(self,o,arrival=False):
        """2026-10-19 rule: |predicted arrival| <= 3 cm; σ is recorded, not a gate."""
        report=o.get('fork_uncertainty') or {}
        if arrival:mean=report.get('predicted_arrival_y_m');std=report.get('predicted_arrival_std_m')
        else:mean=(report.get('position') or [None,None])[1];std=(report.get('std_m') or [None,None])[1]
        if mean is None or not math.isfinite(mean):return False
        space=(self.cfg['site']['truck_inner_width_m']-self.cfg['site']['vehicle_width_m'])/2
        self.last_lateral_check=dict(arrival=arrival,mean_m=mean,std_m=std,tolerance_m=min(space,self.t['position_tolerance_m']),
                                     sigma=o.get('sigma'),certified_error_bound=False)
        return abs(mean)<=min(space,self.t['position_tolerance_m'])

    # ---------------------------------------------------------------- depth entrance measurement (at rest)
    def depth_measurement(self,now,o,p):
        """Collect per-frame depth entrance readings while standing still; return the decision input once the window is full.
        None = keep collecting. source: 'depth' | 'vio_propagated' (last centre carried by the fused pose) | 'tag' (entrance =
        dock origin, while a tag window is fresh) | 'none'."""
        if self.depth_collect_since is None:self.depth_collect_since=now;self.depth_samples=[]
        d=o.get('depth') or {}
        if d.get('stamp') is not None and d['stamp']!=self.depth_last_stamp:
            self.depth_last_stamp=d['stamp']
            if d.get('ok') and finite((d.get('x_body_m'),d.get('lateral_body_m'))) and 0<d['x_body_m']<=self.t['depth_goal_max_range_m']:
                cx,cy=entrance_in_dock(p,d)
                self.depth_samples.append((cx,cy,float(d.get('width_m') or 0.),d.get('pair_heading_deg'),bool(d.get('walls')),d.get('axis_heading_deg')))
        n=len(self.depth_samples);elapsed=now-self.depth_collect_since
        if n>=self.t['depth_measure_min_frames'] and elapsed>=self.t['depth_measure_s']:
            arr=np.array([(s[0],s[1],s[2]) for s in self.depth_samples],float);cx,cy,w=np.median(arr,axis=0)
            ph=[s[3] for s in self.depth_samples if s[3] is not None]
            # the truck axis from the side-panel line fit (walls) — a heading measurement independent of the VIO/tag reference
            wall_axes=[s[5] for s in self.depth_samples if s[4] and s[5] is not None];walls=len(wall_axes)*2>n and len(wall_axes)>=2
            centre=(float(cx),float(cy));self.depth_seen=True;self.depth_centre_dock=centre
            m=dict(source='depth',centre_dock=list(centre),lateral_error_m=lateral_error(p,centre),distance_m=distance_to_entrance(p,centre),
                   width_m=float(w),frames=n,spread_m=float(np.std(arr[:,1])),pair_heading_deg=None if not ph else float(np.median(ph)),
                   walls=bool(walls),axis_heading_deg=None if not walls else float(np.median(wall_axes)),
                   tag_lateral_m=float(p[1]),tag_distance_m=float(-p[0]),t=now)
            self.depth_measure=m;return m
        if elapsed<max(self.t['depth_wait_s'],self.t['depth_measure_s']+.1):return None
        if self.depth_seen and self.depth_centre_dock is not None:
            c=self.depth_centre_dock
            m=dict(source='vio_propagated',centre_dock=list(c),lateral_error_m=lateral_error(p,c),distance_m=distance_to_entrance(p,c),frames=n,t=now)
        elif o.get('blind_age_s') is not None and o['blind_age_s']<=1.:
            m=dict(source='tag',centre_dock=[0.,0.],lateral_error_m=float(p[1]),distance_m=float(-p[0]),frames=n,t=now)
        else:m=dict(source='none',frames=n,t=now)
        self.depth_measure=m;return m

    # ---------------------------------------------------------------- the state machine
    def step(self,now,o,enter=False):
        if self.state in ('FAULT','COMPLETE'):return self.status()
        if self.mission_start is not None and now-self.mission_start>self.t['mission_timeout_s']:return self.fault('mission_timeout')
        if o and o.get('clock_fault'):return self.fault('sensor_clock_fault')
        if o and o.get('imu',{}).get('calibration',{}).get('state')=='FAILED':return self.fault('imu_calibration_failed')
        fresh=self.fresh(now,o)
        new=bool(o and o.get('capture_id')!=self.last_capture)
        if new:self.last_capture=o['capture_id']
        if new and fresh and o.get('tag',{}).get('detected'):self.last_seen=now
        still=fresh and o.get('stationary')
        if self.state in ('PREPARING','WAIT_ENTER'):
            if new:
                # Retain confirmed preparation time across a short (<= 3 s) gyro flicker at the same place,
                # but not across real movement, stale tracking or coordinate resets.
                anchor=self.prep_anchor
                same=bool(anchor and fresh and
                    o.get('motion_epoch')==anchor.get('motion_epoch') and
                    o.get('tracking_generation')==anchor.get('tracking_generation') and
                    np.linalg.norm(np.array(o['local_pose'][:2])-anchor['local_pose'][:2])<=.004 and
                    abs(wrap(o['local_pose'][2]-anchor['local_pose'][2]))<=math.radians(.35))
                if still:
                    if self.prep_pause is not None and (not same or now-self.prep_pause>3.):self.prep_seconds=0.
                    if self.prep_last is not None and 0<o['capture_id']-self.prep_last<=self.t['frame_gap_s']:
                        self.prep_seconds+=o['capture_id']-self.prep_last
                    self.prep_last=o['capture_id'];self.prep_anchor=dict(o);self.prep_pause=None
                else:
                    self.prep_last=None
                    if self.prep_pause is None:self.prep_pause=now
                    if not same or now-self.prep_pause>3.:self.prep_seconds=0.;self.prep_anchor=None
            if not fresh:
                self.prep_seconds=0.;self.prep_last=None;self.prep_anchor=None
            duration=self.prep_seconds
            self.preparation_report=dict(stable_seconds=duration,required_seconds=self.t['preparation_s'],
                stationary_report=(o or {}).get('stationary_report'),
                imu_ready=bool(o and o.get('sensor_ready')),tag_seen=bool(o and o.get('valid')),
                imu_calibration=(o or {}).get('imu',{}).get('calibration'),
                tag_repeatability=(o or {}).get('tag_position_window'),
                sigma_baseline=(o or {}).get('sigma_baseline'),baseline_ready=bool((o or {}).get('baseline_ready')),
                performance=(o or {}).get('performance'),
                certified_error_bound=False)
            if self.state=='PREPARING':
                if now-self.created>self.t['preparation_timeout_s']:return self.fault('preparation_timeout')
                if still and duration>=self.t['preparation_s']:
                    self.state='WAIT_ENTER';self.reason='ready_press_enter';self.initialized_at=now
            elif not still:
                self.state='PREPARING';self.created=now;self.reason='rechecking_before_enter'
            elif enter:
                self.mission_start=now;self.state='PLAN';self.reason='operator_started'
            return self.status()
        if self.state=='MOVING':return self.step_moving(now,o,new,still)
        self.command='stop'
        if self.state=='SETTLE':return self.step_settle(now,o,fresh,still)
        if self.state=='RECOVER':
            if still and o.get('usable'):
                self.state='PLAN';self.reason='tracking_recovered_replan'
            elif now-self.recovery_at>=self.t['recovery_timeout_s']:return self.fault('recovery_timeout')
            return self.status()
        if self.state=='BASELINE':
            # Tag found (or visible from the start): stand still baseline_s with it in view before the first decision.
            if not o.get('usable'):
                # A momentary VIO reset drops 'usable' for ~0.5 s: wait here. Only a tag not seen for 2 s restarts the search.
                self.baseline_last=None
                if o.get('tag',{}).get('detected'):self.baseline_lost_since=None
                elif self.baseline_lost_since is None:self.baseline_lost_since=now
                elif now-self.baseline_lost_since>2.:self.state='PLAN';self.baseline_lost_since=None
                self.reason='baseline_waiting_for_pose';return self.status()
            self.baseline_lost_since=None
            if still:
                if self.baseline_last is not None and 0<o['capture_id']-self.baseline_last<=self.t['frame_gap_s']:
                    self.baseline_seconds+=o['capture_id']-self.baseline_last
                self.baseline_last=o['capture_id'];self.reason='stationary_baseline_with_tag'
            else:self.baseline_last=None;self.reason='baseline_waiting_for_stillness'
            if self.baseline_seconds>=self.t['baseline_s']:self.baseline_done=True;self.state='PLAN';self.reason='baseline_done'
            return self.status()
        if not still:
            self.recover(now,'need_stopped_observation');return self.status()
        if self.state=='PLAN':return self.step_plan(now,o)
        return self.status()

    def step_moving(self,now,o,new,still):
        a=self.action
        # While moving, an observation up to motion_age_s old is still used (predictive stop adds its age to the reserve).
        lost=not self.fresh(now,o,self.t['motion_age_s']) or o['tracking_generation']!=a['generation'] or o['motion_epoch']!=a['epoch']
        turn='rotate' in a['command'];search=a['after']=='SEARCH';entry=a.get('blind')
        if entry:
            # Entry: a lost VIO does not stop the vehicle inside the truck; the leg finishes on time.
            if lost and not a['timed']:
                a['timed']=True;a['timed_since']=now;self.reason='entry_vio_lost_timed_fallback'
            if a['timed']:
                if now-a['started']>=a['hold_s']:self.stop(now,'COMPLETE','timed')
                elif now-a['started']>=min(a['cap_s'],2*a['hold_s']+3.):
                    self.command='stop';return self.fault('motion_time_limit')
                return self.status()
        elif lost:
            self.recover(now,'tracking_lost_during_motion');return self.status()
        if not search and not entry and not o.get('usable'):
            self.recover(now,'pose_untrusted');return self.status()
        if o.get('usable') and not inside_truck(o['pose'],self.cfg):return self.fault('truck_side_clearance')
        if new:
            p=self.progress(o);dt=now-a['last']
            # Rate from the VIO ruler alone (the tag ruler refreshes in 0.5 s windows and jumps a few cm); clamped.
            parts=a.get('progress_parts');rate_src=parts[0] if parts and parts[0] is not None else p
            prev_src=a.get('previous_src',rate_src)
            if dt>0:a['speed']=float(min(.45 if not turn else .6,max(0.,.65*a['speed']+.35*(rate_src-prev_src)/dt)))
            a.update(progress=p,previous=p,previous_src=rate_src,last=now)
            if p>.005 and self.motion_start is None:self.motion_start=now;a['np_progress']=p;a['np_since']=now
            if p<(-math.radians(2) if turn else -.03):return self.fault('motion_direction_mismatch')
            # No-progress safety (2026-10-22 user decision): a drive whose rulers do not advance for no_progress_s is not
            # pushing on; stop it and decide again. The entry leg switches to the time model instead (a lip is driven over).
            if not turn and self.motion_start is not None:
                if p>a.get('np_progress',-1.)+.02:a['np_progress']=p;a['np_since']=now
                elif now-a.get('np_since',now)>=self.t['no_progress_s']:
                    if entry:
                        if not a['timed']:a['timed']=True;a['timed_since']=now;self.reason='entry_no_progress_timed_fallback';self.events.append(dict(t=now,event='entry_no_progress_timed',progress_m=float(p)))
                    else:
                        self.no_progress_stops+=1;self.events.append(dict(t=now,event='no_progress_stop',n=self.no_progress_stops,progress_m=float(p),target_m=float(a['target'])))
                        if self.no_progress_stops>self.t['no_progress_stops_max']:self.command='stop';return self.fault('no_progress_repeated')
                        self.stop(now,'SEARCH' if search else 'LEGS' if a['after']=='LEGS' else 'PLAN','no_progress');return self.status()
            # Bundle drive watched on the depth centre: stop when the end lateral (after the turn back) crosses the centreline.
            if a.get('watch') and a.get('watch_sign') and o.get('usable') and self.depth_centre_dock is not None:
                d=o.get('depth') or {}
                if d.get('ok') and d.get('stamp') is not None and d['stamp']!=self.depth_last_stamp and 0<d.get('x_body_m',0)<=self.t['depth_goal_max_range_m']:
                    self.depth_last_stamp=d['stamp'];cx,cy=entrance_in_dock(o['pose'],d);self.depth_centre_dock=(cx,cy)
                    pred=lateral_error(o['pose'],(cx,cy))+swing_back_y(o['pose'],self.cfg['pivot'],self.heading_target())
                    if pred*a['watch_sign']<=self.t['final_lateral_tol_m']/2:
                        self.events.append(dict(t=now,event='bundle_drive_stopped_on_depth',predicted_end_lateral_m=float(pred),progress_m=float(a['progress'])))
                        a.update(stop_reserve=0.,stop_obs_age_s=float(now-o['arrival']),stop_coast_est=float(self.responses.get(a['command'])['coast']),stop_remaining_m=None)
                        self.stop(now,'LEGS','depth_centre');return self.status()
        if search and o['tag'].get('detected'):
            self.stop(now,'PLAN');return self.status()
        if search and self.sweep+max(0,a['progress'])>=2*math.pi:
            self.stop(now,'SEARCH_LIMIT');return self.status()
        if now-a['started']>=a['cap_s']:
            self.command='stop';return self.fault('motion_time_limit')
        response=self.responses.get(a['command']);reserve=max(0.,a['speed'])*(response['coast']+max(0,now-o['arrival']))
        # Predictive stopping applies only after motion has actually been observed, never before the vehicle starts.
        reached=False
        if not search and not turn and not entry and a['command']!='backward' and o.get('usable'):
            reached=remaining(o['pose'],self.cfg)<=reserve   # never drive past the fork-tip target plane
            if a.get('stop_x') is not None:   # rough leg: stop at the planned x (before the tag cut) by the dock-frame ruler
                reached=reached or (a['stop_x']-o['pose'][0])/max(.2,math.cos(o['pose'][2]))<=reserve
        if self.motion_start is not None and (a['progress']>=a['target']-reserve or reached):
            a.update(stop_reserve=float(reserve),stop_obs_age_s=float(now-o['arrival']),stop_coast_est=float(response['coast']),
                     stop_remaining_m=float(remaining(o['pose'],self.cfg)) if o.get('usable') and o.get('pose') is not None else None)
            self.stop(now,a['after'] if a['after'] in ('SEARCH','COMPLETE','LEGS') else 'PLAN')
        return self.status()

    def step_settle(self,now,o,fresh,still):
        if self.post_settle=='COMPLETE':
            # Entry done. Wait for stillness, or stable_s (SETTLE_S 2 s) if the VIO cannot confirm it.
            if not (still or now-self.stop_at>=self.t['stable_s']):return self.status()
            b=self.brake or {};usable=bool(o and o.get('usable') and o.get('pose') is not None)
            vio_ok=bool(fresh and b and o['tracking_generation']==b.get('generation') and o['motion_epoch']==b.get('epoch'))
            if vio_ok and not b.get('timed'):
                end=self.progress(o)
                self.responses.observe(b['command'],max(0,b['stop_progress']),b['stop_time']-b['started'],max(0,end-b['stop_progress']),
                    dead_s=(b['motion_start']-b['started']) if b.get('motion_start') else None)
            if b:self.step_event(now,b,self.progress(o) if vio_ok else None,vio_ok)
            dc=self.depth_centre_dock
            self.final_report=dict(ended_by='timed_fallback' if b.get('timed') else 'vio_progress',target_m=b.get('target'),
                vio_progress_m=self.progress(o) if vio_ok else None,
                remaining_m=float(remaining(o['pose'],self.cfg)) if usable else None,
                fork_lateral_m=float(front(o['pose'],self.cfg)[1]) if usable else None,
                lateral_vs_depth_centre_m=None if (dc is None or not usable) else float(lateral_error(o['pose'],dc)),
                heading_deg=float(math.degrees(o['pose'][2])) if usable else None,
                pose_available=usable,certified_error_bound=False)
            self.brake=None;self.action=None
            self.state='COMPLETE';self.reason='entry_done_recorded_not_judged'
            return self.status()
        if still:
            if self.brake and o['tracking_generation']==self.brake['generation'] and o['motion_epoch']==self.brake['epoch']:
                end=self.progress(o);b=self.brake
                if b['after']=='SEARCH':self.sweep+=max(0,end)
                self.responses.observe(b['command'],max(0,b['stop_progress']),b['stop_time']-b['started'],max(0,end-b['stop_progress']),
                    dead_s=(b['motion_start']-b['started']) if b.get('motion_start') else None)
                self.step_event(now,b,end,True)
            elif self.brake:self.step_event(now,self.brake,None,False)   # tracking reset during the settle: no measured coast
            self.brake=None;self.action=None
            if self.post_settle=='SEARCH_LIMIT':return self.fault('search_360_exhausted')
            if self.post_settle=='LEGS' and o.get('usable'):self.next_leg(now,o);return self.status()
            self.legs=[];self.state='PLAN'
        elif now-self.stop_at>self.t['settle_timeout_s']:
            stopped_at=self.stop_at
            self.recover(now,'stopped_pose_reacquisition')
            self.recovery_at=stopped_at
        return self.status()

    # ---------------------------------------------------------------- decisions at rest
    def step_plan(self,now,o):
        if not o.get('usable'):
            if self.blind or o.get('valid'):
                self.recover(now,'absolute_pose_untrusted');return self.status()
            if self.search_start is None:self.search_start=now
            if now-self.search_start>self.t['search_timeout_s'] or self.sweep>=2*math.pi-math.radians(1):return self.fault('search_exhausted')
            # A detected but ambiguous tag calls for stationary re-observation.
            if o['tag'].get('detected') or o['tag']['reason']=='not_scheduled' or (self.last_seen is not None and now-self.last_seen<2.):
                self.reason='waiting_for_tag_consensus';return self.status()
            target=min(math.radians(self.t['search_step_deg']),2*math.pi-self.sweep)
            self.begin(now,o,'rotate_left_slow',target,'SEARCH',label='search');return self.status()
        if not self.baseline_done:
            if self.t['baseline_s']<=0:self.baseline_done=True
            else:self.state='BASELINE';self.reason='stationary_baseline_with_tag';self.baseline_last=None;return self.status()
        p=o['pose'];rem=remaining(p,self.cfg)
        if rem < -self.t['longitudinal_tolerance_m']:
            # Already past the target (desk run 2026-10-19): back out once if the rear space allows, else finish and record.
            back=min(-rem,self.t['backward_max_m'])
            if not self.overshoot_backed and back>=self.t['minimum_translation_m'] and path_clear(p,0,-back,self.cfg):
                self.overshoot_backed=True;self.events.append(dict(t=now,event='past_target_backing_%.2fm'%back))
                self.set_legs(now,o,[dict(kind='drive',value=-back,label='past_target_back')],'past_target_back');self.next_leg(now,o);return self.status()
            self.events.append(dict(t=now,event='past_target_finished_without_backing',remaining_m=float(rem)))
            self.state='COMPLETE';self.reason='already_past_target_recorded_not_judged';return self.status()
        if abs(rem)<=self.t['longitudinal_tolerance_m']:
            if self.lateral_within_tolerance(o):
                self.state='COMPLETE';self.reason='measured_stop_at_target_not_accuracy_certificate'
            else:self.fault('target_reached_with_lateral_error')
            return self.status()
        if self.legs:self.next_leg(now,o);return self.status()
        if self.phase=='ROUGH':return self.plan_rough(now,o,p)
        if self.phase=='FINE_ENTER':return self.plan_fine_enter(now,o,p)
        if self.phase in ('FINE','FINAL'):return self.plan_fine(now,o,p)
        if self.phase=='FALLBACK':return self.plan_fallback(now,o,p)
        return self.status()

    def straighten(self,now,o,p,label):
        """One fine turn to the straight heading when the heading error exceeds the tolerance; bounded per decision point."""
        herr=wrap(p[2]-self.heading_target())
        if abs(herr)<=math.radians(self.t['straight_heading_tol_deg']) or self.heading_turns>=3:return False
        if abs(herr)<math.radians(self.t['minimum_turn_fine_deg'])/2:return False
        if not path_clear(p,wrap(self.heading_target()-p[2]),0.,self.cfg):
            self.events.append(dict(t=now,event='straighten_blocked_by_walls',heading_err_deg=float(math.degrees(herr))));return False
        self.heading_turns+=1
        self.set_legs(now,o,[dict(kind='turn',value=self.heading_target(),label=label)],label);self.next_leg(now,o);return True

    def plan_rough(self,now,o,p):
        # Order (user, 2026-10-22): sidestep first, straighten what is left, then the long drive.
        if (abs(p[1])>=self.t['sidestep_min_lateral_m'] and self.sidesteps<self.t['sidestep_max_count']
                and self.cut-p[0]>=self.t['sidestep_min_room_m']):
            # Face the centreline (+-90 deg), drive across, face the truck. Both in-place turns swing the body on the pivot
            # arc; the drive is solved so the body lands on the centreline (runs 162223-174331: 40-70 % of the wanted move).
            self.sidesteps+=1;ht=self.heading_target()
            face=-math.pi/2 if p[1]>0 else math.pi/2;side=1. if face>0 else -1.
            q1=moved(p,wrap(face-p[2]),0,self.cfg['pivot']);y_turns=moved(q1,wrap(ht-q1[2]),0,self.cfg['pivot'])[1]
            d=float(np.clip(self.t['sidestep_fraction']*(-side*y_turns),0.,self.t['sidestep_max_m']))
            self.events.append(dict(t=now,event='sidestep',n=self.sidesteps,lateral_m=float(p[1]),heading_deg=float(math.degrees(p[2])),
                turn_swing_m=float(y_turns-p[1]),drive_m=float(d),predicted_end_lateral_m=float(y_turns+side*d),pose=[float(v) for v in p]))
            self.set_legs(now,o,[dict(kind='turn',value=face,strength='slow',label='sidestep_face'),dict(kind='drive',value=d,label='sidestep_drive'),
                                 dict(kind='turn',value=ht,strength='slow',label='sidestep_back')],'sidestep')
            self.heading_turns=0;self.next_leg(now,o);return self.status()
        if self.straighten(now,o,p,'straighten_rough'):return self.status()
        stop_x=self.cut-self.t['rough_stop_before_cut_m']
        d=(stop_x-p[0])/max(.2,math.cos(p[2]))
        if d>=self.t['minimum_translation_m']:
            d=min(d,self.t['rough_leg_max_m']);self.heading_turns=0
            self.set_legs(now,o,[dict(kind='drive',value=d,label='rough_drive',stop_x=stop_x)],'rough_drive');self.next_leg(now,o);return self.status()
        self.phase='FINE_ENTER';self.rezero_request+=1;self.rezero_since=now;self.heading_turns=0
        self.events.append(dict(t=now,event='rough_done',pose=[float(v) for v in p],sidesteps=self.sidesteps,cut=float(self.cut)))
        self.reason='rezero_heading_from_tag';return self.status()

    def plan_fine_enter(self,now,o,p):
        rz=o.get('heading_rezero') or {}
        done=rz.get('count',0)>=self.rezero_request
        if done or now-self.rezero_since>self.t['heading_rezero_timeout_s']:
            self.rezero_report=dict(applied=bool(done),info=rz.get('info'),waited_s=now-self.rezero_since)
            self.events.append(dict(t=now,event='heading_rezero_'+('applied' if done else 'timeout'),info=rz.get('info'),pose=[float(v) for v in p]))
            self.phase='FINE';self.heading_turns=0;self.depth_collect_since=None;self.reason='fine_stage_start'
            return self.status()
        self.reason='rezero_heading_from_tag';return self.status()

    def plan_fine(self,now,o,p):
        if self.straighten(now,o,p,'straighten_fine'):return self.status()
        m=self.depth_measurement(now,o,p)
        if m is None:self.reason='measuring_depth_entrance';return self.status()
        if m['source']=='none':
            self.phase='FALLBACK';self.events.append(dict(t=now,event='depth_unavailable_fallback',pose=[float(v) for v in p]))
            return self.plan_fallback(now,o,p)
        ht=self.heading_target();e=m['lateral_error_m'];dist=m['distance_m'];tol=self.t['final_lateral_tol_m']
        check=final_check_distance(self.cfg);room=dist-check;dmin=self.t['minimum_translation_m']
        if self.phase=='FINE':
            if room>=dmin:
                blocked=False
                if abs(e)>tol:
                    b=plan_bundle(p,m['centre_dock'][1],self.cfg['pivot'],room,dmin,math.radians(self.t['turn_max_deg']),ht)
                    if b is not None:
                        q2=moved(moved(p,b['alpha'],0.,self.cfg['pivot']),0.,b['d'],self.cfg['pivot'])
                        if not (path_clear(p,b['alpha'],b['d'],self.cfg) and path_clear(q2,wrap(ht-q2[2]),0.,self.cfg)):
                            self.events.append(dict(t=now,event='bundle_blocked_by_walls',alpha_deg=float(math.degrees(b['alpha'])),drive_m=float(b['d'])));b=None;blocked=True
                    if b is not None:
                        self.fine_bundles+=1
                        self.events.append(dict(t=now,event='bundle',n=self.fine_bundles,source=m['source'],lateral_error_m=float(e),distance_m=float(dist),
                            alpha_deg=float(math.degrees(b['alpha'])),drive_m=float(b['d']),predicted_end_lateral_m=float(b['predicted_end_y']-m['centre_dock'][1]),partial=b['partial']))
                        self.set_legs(now,o,[dict(kind='turn',value=wrap(p[2]+b['alpha']),label='bundle_turn'),
                                             dict(kind='drive',value=b['d'],label='bundle_drive',watch=True,watch_sign=1. if e>0 else -1.),
                                             dict(kind='turn',value=ht,label='bundle_back')],'bundle')
                        self.heading_turns=0;self.next_leg(now,o);return self.status()
                if not blocked:
                    d=min(room,self.t['fine_step_max_m'])
                    self.events.append(dict(t=now,event='fine_straight',source=m['source'],lateral_error_m=float(e),distance_m=float(dist),drive_m=float(d)))
                    self.set_legs(now,o,[dict(kind='drive',value=d,label='fine_straight')],'fine_straight');self.heading_turns=0;self.next_leg(now,o);return self.status()
            self.phase='FINAL';self.events.append(dict(t=now,event='final_check_start',source=m['source'],lateral_error_m=float(e),distance_m=float(dist),pose=[float(v) for v in p]))
        # ---- FINAL
        herr=wrap(p[2]-ht);heading_src='vio'
        if m.get('walls') and m.get('axis_heading_deg') is not None:
            # 2026-10-22: with the truck's side panels in view the body heading relative to the TRUCK AXIS comes from the depth
            # line fit (axis_heading_deg = axis direction in the body frame). The straight target is -bias relative to that axis, so
            # the error is -(axis) - (-bias) = bias - axis. This bypasses a tag plane that is not parallel to the truck.
            herr=wrap(path_bias(self.cfg)-math.radians(m['axis_heading_deg']));heading_src='truck_walls'
        ok_lat=abs(e)<=tol;ok_h=abs(herr)<=math.radians(self.t['final_heading_tol_deg'])
        if ok_lat and ok_h:
            distance=dist+self.cfg['target_front_x']-self.cfg['site']['camera_to_fork_tip_m']
            self.events.append(dict(t=now,event='final_check_ok',source=m['source'],lateral_error_m=float(e),heading_err_deg=float(math.degrees(herr)),heading_source=heading_src,distance_m=float(dist)))
            self.begin_entry(now,o,distance/max(.2,math.cos(herr)),m['source']);return self.status()
        self.corrections+=1
        if self.corrections>self.t['fine_corrections_max']:
            self.events.append(dict(t=now,event='final_corrections_exhausted',lateral_error_m=float(e),heading_err_deg=float(math.degrees(herr)),source=m['source']))
            self.final_report=dict(ended_by='corrections_exhausted_no_entry',lateral_error_m=float(e),heading_err_deg=float(math.degrees(herr)),distance_m=float(dist),
                                   source=m['source'],pose_available=True,certified_error_bound=False)
            self.state='COMPLETE';self.reason='final_corrections_exhausted_recorded_no_entry';return self.status()
        target=wrap(p[2]-herr)   # absolute dock heading that zeroes the measured error (= ht when the error comes from the VIO)
        if not ok_h and path_clear(p,wrap(target-p[2]),0.,self.cfg):
            self.heading_turns=0
            self.events.append(dict(t=now,event='final_heading_turn',n=self.corrections,heading_err_deg=float(math.degrees(herr)),heading_source=heading_src))
            self.set_legs(now,o,[dict(kind='turn',value=target,label='final_heading')],'final_heading');self.next_leg(now,o);return self.status()
        # lateral off at the check point: back out (never behind the tag cut), then FINE bundles again with the room gained
        need=abs(e)/math.sin(math.radians(self.t['turn_max_deg']))+dmin
        limit=min(self.t['backward_max_m'],p[0]-(self.cut+.1))
        back=min(limit,max(dmin,need-max(0.,room)))
        if back>=dmin and path_clear(p,0,-back,self.cfg):
            self.events.append(dict(t=now,event='final_back_out',n=self.corrections,lateral_error_m=float(e),back_m=float(back)))
            self.phase='FINE';self.set_legs(now,o,[dict(kind='drive',value=-back,label='final_back')],'final_back');self.next_leg(now,o);return self.status()
        if room>=dmin:   # no room to back out but some ahead: one more bundle from here
            self.phase='FINE';self.events.append(dict(t=now,event='final_bundle_without_backing',n=self.corrections,room_m=float(room)));return self.plan_fine(now,o,p)
        self.events.append(dict(t=now,event='final_no_room_to_correct',lateral_error_m=float(e),room_m=float(room),back_limit_m=float(limit)))
        self.final_report=dict(ended_by='no_room_to_correct_no_entry',lateral_error_m=float(e),heading_err_deg=float(math.degrees(herr)),distance_m=float(dist),
                               source=m['source'],pose_available=True,certified_error_bound=False)
        self.state='COMPLETE';self.reason='final_no_room_recorded_no_entry';return self.status()

    def plan_fallback(self,now,o,p):
        """Tag-pose blind entry (no depth): aim the predicted fork-tip arrival at the centreline (bounded fine turns), then one leg."""
        rem=remaining(p,self.cfg);h=p[2]+path_bias(self.cfg)
        L=rem/max(.2,math.cos(h))+self.cfg['site']['camera_to_fork_tip_m']
        if abs(arrival_y(p,self.cfg))>self.t['position_tolerance_m'] and self.fallback_corrections<self.t['transition_corrections_max'] and L>0 and abs(p[1])<L:
            self.fallback_corrections+=1
            aim=wrap(-math.asin(p[1]/L)-path_bias(self.cfg))   # heading whose straight path arrives on the centreline
            self.events.append(dict(t=now,event='fallback_aim_turn',n=self.fallback_corrections,arrival_m=float(arrival_y(p,self.cfg)),aim_deg=float(math.degrees(aim))))
            self.set_legs(now,o,[dict(kind='turn',value=aim,label='fallback_aim')],'fallback_aim');self.next_leg(now,o);return self.status()
        self.lateral_within_tolerance(o,arrival=True)   # records the decision with its σ
        self.begin_entry(now,o,rem/max(.2,math.cos(h)),'fallback_tag_pose');return self.status()
