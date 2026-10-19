"""Nonblocking mission state machine. All motion requires fresh measured feedback."""
import math
import numpy as np
from .geometry import wrap,remaining,arrival_y,inside_bay,path_clear,front,finite,tag_margin,moved
from .planner import Responses

class Controller:
    def __init__(self,cfg,cut,now):
        self.cfg=cfg;self.t=cfg['tuning'];self.cut=cut;self.created=now
        self.state='PREPARING';self.reason='stationary_preparation';self.command='stop'
        self.mission_start=None;self.stable_start=None;self.last_capture=None
        self.prep_seconds=0.;self.prep_last=None;self.prep_anchor=None;self.prep_pause=None
        self.responses=Responses();self.plan_id=0;self.request=None;self.route=[];self.route_points=[]
        self.action=None;self.pending_distance=None;self.brake=None;self.blind=False
        self.search_start=None;self.sweep=0.;self.motion_start=None;self.stop_at=None
        self.recovery_at=None;self.post_settle='PLAN';self.await_plan=None
        self.events=[];self.initialized_at=None;self.preparation_report={};self.last_seen=None
        self.blind_wait_since=None   # waiting at the tag-cut transition for a fresh tag; bounded by recovery_timeout_s
        self.transition_corrections=0   # correction plans issued at the transition; bounded by transition_corrections_max
        self.final_report=None          # blind entry outcome: what ended the leg and the measured error (record only)
        self.last_lateral_check=None    # the last |arrival| <= tolerance decision with its σ (record only, 2026-10-19)
        self.baseline_done=False;self.baseline_seconds=0.;self.baseline_last=None   # 30 s at rest with the tag in view, after Enter and after any search
        self.mount_yaw_unlearned=False   # entered the blind leg without any good straight leg (record only)
        self.baseline_lost_since=None;self.overshoot_backed=False

    def fault(self,reason):
        self.state='FAULT';self.reason=reason;self.command='stop';self.request=None
        return self.status()

    def status(self):
        return dict(state=self.state,reason=self.reason,command=self.command,blind=self.blind,final_report=self.final_report,
                    lateral_check=self.last_lateral_check,baseline_done=self.baseline_done,baseline_seconds=self.baseline_seconds,
                    mount_yaw_unlearned=self.mount_yaw_unlearned,events=list(self.events),
                    search_sweep_deg=math.degrees(self.sweep),route=[dict(a) for a in self.route],
                    preparation=dict(self.preparation_report),responses={k:dict(v) for k,v in self.responses.data.items()},
                    route_points=[list(p) for p in self.route_points],
                    estimate_use='response estimates affect planning/braking only, never measured pose')

    def fresh(self,now,o,age=None):
        # age: sensor_age_s (0.3 s) for decisions at rest; motion_age_s (1.0 s, v4 BLIND_MAX_S) while moving.
        age=self.t['sensor_age_s'] if age is None else age
        return bool(o and not o.get('clock_fault') and o.get('sensor_ready') and not o.get('imu_bad') and
                    finite(o.get('local_pose',[])) and len(o.get('local_pose',[]))==3 and
                    (not o.get('usable') or (o.get('pose') is not None and finite(o['pose']))) and
                    0<=now-o.get('arrival',-math.inf)<=age and
                    o.get('transport_lag_s',0)<=age and o.get('local_valid'))

    def accept_plan(self,reply,now,o):
        if self.state!='PLANNING' or reply.get('id')!=self.plan_id:return
        if reply.get('error'):self.fault('planner: '+reply['error']);return
        old=self.await_plan
        if not self.fresh(now,o) or not o.get('usable') or not o.get('stationary'):
            self.recover(now,'observation_changed_during_plan');return
        if o['motion_epoch']!=old['epoch'] or np.linalg.norm(np.array(o['pose'][:2])-old['pose'][:2])>.01 or abs(wrap(o['pose'][2]-old['pose'][2]))>math.radians(.5):
            self.state='PLAN';return
        result=reply['plan'];self.route=result['actions']
        point=np.array(o['pose']);self.route_points=[point.tolist()]
        for action in self.route:
            point=moved(point,action['turn'],action['distance'],self.cfg['pivot'])
            self.route_points.append(point.tolist())
        if not self.route:
            if result.get('complete'):self.state='PLAN'
            else:self.fault(result['reason'])
            return
        a=self.route[0];self.pending_distance=(a['distance'],a['fine']) if a['turn'] else None
        if a['turn']:
            side='left' if a['turn']>0 else 'right';cmd='rotate_'+side+('_fine' if a['fine'] else '_slow')
            target=abs(a['turn'])
            # If shortened, don't blindly execute the original diagonal leg.
            if target<abs(a['turn'])-1e-8:self.pending_distance=None
            self.begin(now,o,cmd,target,'SETTLE')
        else:self.begin_drive(now,o,a['distance'],a['fine'])

    def begin_drive(self,now,o,distance,fine):
        # 2026-10-19 user decision: byte 67 everywhere. 97 (forward_slow) was never driven on the truck.
        cmd='backward' if distance<0 else 'forward'
        target=min(abs(distance),self.t['backward_max_m'] if distance<0 else self.t['forward_max_m'])
        if distance>0:target=min(target,max(0.,remaining(o['pose'],self.cfg)/max(.2,math.cos(o['pose'][2]))))
        signed=-target if distance<0 else target
        if not path_clear(o['pose'],0,signed,self.cfg):self.fault('bay_side_clearance');return
        if not self.blind and tag_margin(moved(o['pose'],0,signed,self.cfg['pivot']),self.cfg)<self.cfg['sensor']['tag']['edge_px']:
            self.state='PLAN';self.reason='rotation_changed_visibility_replan';return
        if target<.005:self.state='PLAN';return
        self.begin(now,o,cmd,target,'SETTLE')

    def begin(self,now,o,cmd,target,after):
        self.action=dict(command=cmd,target=target,start_local=o['local_pose'][:],generation=o['tracking_generation'],
                         epoch=o['motion_epoch'],started=now,after=after,speed=0.,progress=0.,previous=0.,last=now,blind=False)
        self.state='MOVING';self.command=cmd;self.reason='measured_motion_feedback';self.motion_start=None

    def begin_blind(self,now,o,distance):
        """Blind entry (2026-10-19 user decision): ONE leg for the whole remaining distance, stopped on
        OpenVINS progress. If VIO is lost on the way the leg continues on the v4 time model
        (startup + distance/speed) instead of stopping inside the bay. No judgment afterwards."""
        target=max(0.,float(distance))
        if target<.005:self.state='PLAN';return
        if not path_clear(o['pose'],0,target,self.cfg):self.fault('bay_side_clearance');return
        r=self.responses.get('forward')
        self.begin(now,o,'forward',target,'COMPLETE')
        self.action.update(blind=True,timed=False,hold_s=r['startup']+target/r['speed'])
        self.reason='blind_entry_vio_progress'

    def progress(self,o):
        a=self.action;p=np.array(o['local_pose']);start=np.array(a['start_local'])
        if 'rotate' in a['command']:
            return wrap(p[2]-start[2])*(1 if 'left' in a['command'] else -1)
        delta=p[:2]-start[:2];signed=float(delta @ [math.cos(start[2]),math.sin(start[2])])
        return -signed if a['command']=='backward' else signed

    def stop(self,now,post='PLAN'):
        self.command='stop';self.state='SETTLE';self.stop_at=now;self.post_settle=post
        if self.action:self.brake=dict(self.action,stop_progress=self.action['progress'],stop_time=now,motion_start=self.motion_start)
        self.reason='observing_coast_and_rebound'

    def recover(self,now,reason):
        if self.action and self.action['after']=='SEARCH':
            self.fault('search_tracking_lost_unmeasured_angle');return
        self.command='stop';self.state='RECOVER';self.recovery_at=now;self.reason=reason
        self.pending_distance=None;self.action=None;self.brake=None;self.await_plan=None;self.request=None
        self.route=[];self.route_points=[]

    def lateral_within_tolerance(self,o,arrival=False):
        """2026-10-19 (apriltag_v5 rule): enter when |predicted arrival| <= 3 cm; σ is recorded, not a gate.
        The |mean| + 2σ test that preceded this never passed at 3 m with a single tag (σ ≈ 32 mm)."""
        report=o.get('fork_uncertainty') or {}
        if arrival:mean=report.get('predicted_arrival_y_m');std=report.get('predicted_arrival_std_m')
        else:mean=(report.get('position') or [None,None])[1];std=(report.get('std_m') or [None,None])[1]
        if mean is None or not math.isfinite(mean):return False
        space=(self.cfg['site']['bay_width_m']-self.cfg['site']['vehicle_width_m'])/2
        self.last_lateral_check=dict(arrival=arrival,mean_m=mean,std_m=std,tolerance_m=min(space,self.t['position_tolerance_m']),
                                     sigma=o.get('sigma'),certified_error_bound=False)
        return abs(mean)<=min(space,self.t['position_tolerance_m'])

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
        if self.state=='MOVING':
            a=self.action
            # While moving, a 1 s old observation is still used (predictive stop adds its age to the reserve).
            lost=not self.fresh(now,o,self.t['motion_age_s']) or o['tracking_generation']!=a['generation'] or o['motion_epoch']!=a['epoch']
            if a.get('blind'):
                # Blind entry: a lost VIO does not stop the vehicle inside the bay; the leg finishes on time (v4).
                if lost and not a['timed']:
                    a['timed']=True;a['timed_since']=now;self.reason='blind_entry_vio_lost_timed_fallback'
                if a['timed']:
                    if now-a['started']>=a['hold_s']:self.stop(now,'COMPLETE')
                    elif now-a['started']>=min(self.t['translation_timeout_s'],2*a['hold_s']):
                        self.command='stop';return self.fault('motion_time_limit')
                    return self.status()
            elif lost:
                self.recover(now,'tracking_lost_during_motion');return self.status()
            search=a['after']=='SEARCH'
            if not search and not a.get('blind') and not o.get('usable'):
                self.recover(now,'pose_untrusted');return self.status()
            if o.get('usable') and not inside_bay(o['pose'],self.cfg):return self.fault('bay_side_clearance')
            if new:
                p=self.progress(o);dt=now-a['last']
                if dt>0:a['speed']=.65*a['speed']+.35*(p-a['previous'])/dt
                a.update(progress=p,previous=p,last=now)
                if p>.005 and self.motion_start is None:self.motion_start=now
                if p<(-math.radians(2) if 'rotate' in a['command'] else -.03):return self.fault('motion_direction_mismatch')
            if search and o['tag'].get('detected'):
                self.stop(now,'PLAN');return self.status()
            if search and self.sweep+max(0,a['progress'])>=2*math.pi:
                self.stop(now,'SEARCH_LIMIT');return self.status()
            turn='rotate' in a['command']
            cap=self.t['search_turn_timeout_s'] if search else self.t['rotation_timeout_s'] if turn else self.t['translation_timeout_s']
            if now-a['started']>=cap:
                self.command='stop';return self.fault('motion_time_limit')
            response=self.responses.get(a['command']);reserve=max(0.,a['speed'])*(response['coast']+max(0,now-o['arrival']))
            # No target/2 clamp on the braking reserve: it made every small correction turn stop at
            # half its target and then coast past it (v5 lesson, 2026-10-19: a 1° request executed as
            # 1.6° and the transition correction oscillated for 600 s). Predictive stopping applies
            # only after motion has actually been observed, never before the vehicle starts.
            reached=(not search and not turn and a['command']!='backward' and remaining(o['pose'],self.cfg)<=reserve)
            if a.get('blind'):reached=False   # the blind leg ends on VIO progress only; the dock-frame pose is not re-judged
            if self.motion_start is not None and (a['progress']>=a['target']-reserve or reached):self.stop(now,a['after'] if a['after'] in ('SEARCH','COMPLETE') else 'PLAN')
            return self.status()
        self.command='stop'
        if self.state=='SETTLE':
            if self.post_settle=='COMPLETE':
                # Blind entry done. Wait for stillness, or stable_s (v4 SETTLE_S 2 s) if VIO cannot confirm it.
                if not (still or now-self.stop_at>=self.t['stable_s']):return self.status()
                b=self.brake or {};usable=bool(o and o.get('usable') and o.get('pose') is not None)
                vio_ok=bool(fresh and b and o['tracking_generation']==b.get('generation') and o['motion_epoch']==b.get('epoch'))
                if vio_ok and not b.get('timed'):
                    end=self.progress(o)
                    self.responses.observe(b['command'],max(0,b['stop_progress']),b['stop_time']-b['started'],max(0,end-b['stop_progress']),
                        dead_s=(b['motion_start']-b['started']) if b.get('motion_start') else None)
                self.final_report=dict(ended_by='timed_fallback' if b.get('timed') else 'vio_progress',target_m=b.get('target'),
                    vio_progress_m=self.progress(o) if vio_ok else None,
                    remaining_m=float(remaining(o['pose'],self.cfg)) if usable else None,
                    fork_lateral_m=float(front(o['pose'],self.cfg)[1]) if usable else None,
                    pose_available=usable,certified_error_bound=False)
                self.brake=None;self.action=None
                self.state='COMPLETE';self.reason='blind_entry_done_recorded_not_judged'
                return self.status()
            if still:
                if self.brake and o['tracking_generation']==self.brake['generation'] and o['motion_epoch']==self.brake['epoch']:
                    end=self.progress(o);b=self.brake
                    if b['after']=='SEARCH':self.sweep+=max(0,end)
                    self.responses.observe(b['command'],max(0,b['stop_progress']),b['stop_time']-b['started'],max(0,end-b['stop_progress']),
                        dead_s=(b['motion_start']-b['started']) if b.get('motion_start') else None)
                self.brake=None
                self.action=None
                if self.post_settle=='SEARCH_LIMIT':return self.fault('search_360_exhausted')
                if self.pending_distance and o.get('usable'):
                    d,fine=self.pending_distance;self.pending_distance=None;self.begin_drive(now,o,d,fine)
                else:self.state='PLAN'
            elif now-self.stop_at>self.t['settle_timeout_s']:
                stopped_at=self.stop_at
                self.recover(now,'stopped_pose_reacquisition')
                self.recovery_at=stopped_at
            return self.status()
        if self.state=='RECOVER':
            if still and o.get('usable'):
                self.state='PLAN';self.reason='tracking_recovered_replan'
            elif now-self.recovery_at>=self.t['recovery_timeout_s']:return self.fault('recovery_timeout')
            return self.status()
        if self.state=='BASELINE':
            # Tag found (or visible from the start): stand still baseline_s with it in view before the first plan.
            if not o.get('usable'):
                # A momentary VIO reset drops 'usable' for ~0.5 s: wait here. Only a tag not seen for 2 s means the
                # search must restart (desk run 2026-10-19: BASELINE -> PLAN -> RECOVER flicker on every VIO reset).
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
        if self.state=='PLANNING':
            if now-self.await_plan['at']>self.t['planning_timeout_s']:return self.fault('planner_timeout')
            return self.status()
        if self.state=='PLAN':
            if not o.get('usable'):
                if self.blind or o.get('valid'):
                    self.recover(now,'absolute_pose_untrusted');return self.status()
                if self.search_start is None:self.search_start=now
                if now-self.search_start>self.t['search_timeout_s'] or self.sweep>=2*math.pi-math.radians(1):return self.fault('search_exhausted')
                # A detected but ambiguous tag calls for stationary re-observation.
                if o['tag'].get('detected') or o['tag']['reason']=='not_scheduled' or (self.last_seen is not None and now-self.last_seen<2.):
                    self.reason='waiting_for_tag_consensus';return self.status()
                target=min(math.radians(self.t['search_step_deg']),2*math.pi-self.sweep)
                self.begin(now,o,'rotate_left_slow',target,'SEARCH');return self.status()
            if not self.baseline_done:
                if self.t['baseline_s']<=0:self.baseline_done=True
                else:self.state='BASELINE';self.reason='stationary_baseline_with_tag';self.baseline_last=None;return self.status()
            p=o['pose'];rem=remaining(p,self.cfg)
            if rem < -self.t['longitudinal_tolerance_m']:
                # Already past the target (desk run 2026-10-19: camera placed inside 2 m -> the run died). Not a fault:
                # back out once if the rear space and the minimum step allow it, otherwise finish and record it.
                back=min(-rem,self.t['backward_max_m'])
                if not self.overshoot_backed and back>=self.t['minimum_translation_m'] and path_clear(p,0,-back,self.cfg):
                    self.overshoot_backed=True;self.events.append(dict(t=now,event='past_target_backing_%.2fm'%back))
                    self.begin_drive(now,o,-back,False);return self.status()
                self.events.append(dict(t=now,event='past_target_finished_without_backing',remaining_m=float(rem)))
                self.state='COMPLETE';self.reason='already_past_target_recorded_not_judged';return self.status()
            if abs(rem)<=self.t['longitudinal_tolerance_m']:
                if self.lateral_within_tolerance(o):
                    self.state='COMPLETE';self.reason='measured_stop_at_target_not_accuracy_certificate'
                else:self.fault('target_reached_with_lateral_error')
                return self.status()
            # Re-evaluate clipping at the observed lateral position/heading;
            # the centreline cut x alone can be several cm too late.
            margin=tag_margin(p,self.cfg)
            at_transition=(p[0]>=self.cut-.05 or
                (p[0]>=self.cut-self.t['final_zone_m'] and self.cfg['sensor']['tag']['edge_px']<=margin<=self.t['cut_margin_px']+self.t['transition_margin_slack_px']))
            if at_transition or self.blind:
                if not self.blind:
                    if o.get('blind_age_s') is None or o['blind_age_s']>1.:
                        # Stay in PLAN (vehicle neutral) and keep ONE timer. recover() here reset
                        # recovery_at every loop and never timed out (observed in review, 2026-10-18).
                        if self.blind_wait_since is None:self.blind_wait_since=now
                        if now-self.blind_wait_since>self.t['recovery_timeout_s']:return self.fault('recent_tag_before_blind_timeout')
                        self.reason='waiting_recent_tag_before_blind';return self.status()
                    self.blind_wait_since=None
                    if o.get('good_legs')==0 and not self.mount_yaw_unlearned:
                        # No straight leg good enough to learn the mount yaw: enter anyway on the pre-measured value
                        # (2026-10-19 user decision: a bad result is a result, not a reason to stop). Record it.
                        self.mount_yaw_unlearned=True;self.events.append(dict(t=now,event='entering_with_unlearned_mount_yaw'))
                    if abs(arrival_y(p,self.cfg))>self.t['position_tolerance_m']:
                        # Still visible: plan a fine correction, no separate yaw gate. Bounded: a turn the
                        # vehicle cannot execute finely enough would otherwise oscillate until mission_timeout.
                        self.transition_corrections+=1
                        if self.transition_corrections>self.t['transition_corrections_max']:
                            return self.fault('transition_corrections_exhausted')
                    else:self.lateral_within_tolerance(o,arrival=True);self.blind=True   # records the decision with its σ
                if self.blind:
                    if not self.lateral_within_tolerance(o,arrival=True):
                        return self.fault('blind_predicted_lateral_outside_tolerance')
                    self.begin_blind(now,o,rem/max(.2,math.cos(p[2])))   # whole remaining distance, one leg
                    return self.status()
            self.plan_id+=1;self.await_plan=dict(at=now,pose=np.array(p),epoch=o['motion_epoch'])
            self.request=dict(id=self.plan_id,pose=p,cut=self.cut,responses=self.responses.data.copy())
            self.state='PLANNING';self.reason='six_step_beam_search'
        return self.status()
