"""Continuous measured odometry with qualified tag corrections.

Confidence radii are operational heuristics, NOT maximum-error certificates.
No command, commanded distance, learned speed or acceleration integral enters pose.
"""
import collections
import math
import time
import numpy as np
import cv2
from .vision import Tag, SparseOdom, inverse
from .geometry import pose, matrix, wrap, finite

class Observer:
    def __init__(self,cfg):
        self.cfg=cfg; self.t=cfg['tuning'];self.neutral=True
        cv2.setNumThreads(cfg['opencv_threads'])
        self.tag=Tag(cfg['tag']) if cfg['tag']['enabled'] else None
        self.odom=SparseOdom(cfg['odom']) if cfg['odom']['enabled'] else None
        self.last_tag=-math.inf;self.last_stamp=None;self.previous_imu=None;self.generation=None
        self.local=np.zeros(3);self.anchor=None;self.radius=np.array([.02,.02,.01])
        self.stationary=False;self.history=collections.deque();self.tags=collections.deque()
        self.anchor_stamp=None;self.travel=0.;self.conflicts=0;self.held_local=None
        self.bias_hint=0.;self.stop_since=None;self.motion_epoch=0

    def process(self,gray,depth,K,dist,stamp,imu_delta=None,image_quality=None,capture_imu=None):
        began=time.monotonic();t=self.t;cal=self.cfg['calibration']
        Tbc=np.array(cal['T_body_camera']) if cal.get('T_body_camera') is not None else None
        Tdt=np.array(cal['T_dock_tag']) if cal.get('T_dock_tag') is not None else None
        od=self.odom.update(gray,depth,K,dist,stamp) if self.odom else dict(ok=False,reason='disabled',generation=0)
        odone=time.monotonic();dt=stamp-self.last_stamp if self.last_stamp is not None else 0.
        if not math.isfinite(stamp) or (self.last_stamp is not None and dt<=0): raise ValueError('video clock reversed/nonfinite')
        self.last_stamp=stamp
        changed=od['generation']!=self.generation
        if changed:
            self.local=np.zeros(3);self.anchor=None;self.tags.clear();self.history.clear();self.held_local=None
            self.motion_epoch+=1
        self.generation=od['generation']
        # Do not bridge a failed image pair with commanded motion or IMU translation.
        local_ok=bool(od['ok'] and Tbc is not None)
        pair=None
        if local_ok and capture_imu and self.previous_imu:
            a,b=self.previous_imu,capture_imu
            if a['device_stamp_s']==od.get('reference_stamp') and a['gaps']==b['gaps'] and not changed:
                pair=b['yaw_rad']-a['yaw_rad']
        delta=None;reason='tracked' if local_ok else od['reason']
        if local_ok:
            delta=pose(Tbc @ od['T_previous_current'] @ inverse(Tbc))
            if not finite(delta) or (pair is not None and not math.isfinite(pair)):
                raise ValueError('nonfinite odometry/gyro')
            if pair is not None and abs(wrap(pair-delta[2]))>t['imu_disagreement_rad']:
                local_ok=False;reason='visual_gyro_disagreement'
            elif pair is not None:
                # No long-term gyro heading: only exposure-aligned increments.
                # In confirmed stillness use vision; estimate only a small rate bias.
                weight=0. if self.stationary and self.neutral else t['gyro_weight']
                delta[2]=wrap(delta[2]+weight*wrap(pair-self.bias_hint*max(dt,0)-delta[2]))
        if image_quality and image_quality.get('reject_motion_blur'):
            local_ok=False;reason='motion_blur'
        if local_ok:
            self.local=pose(matrix(self.local) @ matrix(delta))
            distance=float(np.linalg.norm(delta[:2]));self.travel+=distance
            self.radius[:2]+=t['drift_per_m']*distance+t['drift_per_rad']*abs(delta[2])+t['drift_per_s']*dt
            self.radius[2]+=math.radians(.1)*distance+math.radians(.01)*dt
        else:
            self.anchor=None;self.tags.clear();self.history.clear();self.held_local=None
            self.stationary=False
        self.previous_imu=capture_imu
        if local_ok:
            self.history.append((stamp,self.local.copy(),capture_imu))
        while self.history and stamp-self.history[0][0]>t['stable_s']+.15:self.history.popleft()
        still=False
        if local_ok and self.neutral and self.history and stamp-self.history[0][0]>=t['stable_s']:
            pts=np.array([p for _,p,_ in self.history]);dy=np.array([wrap(p[2]-pts[0,2]) for p in pts])
            rates=[abs(i['rate_rps']) for _,_,i in self.history if i]
            still=(np.ptp(pts[:,:2],axis=0).max()<t['still_translation_m'] and
                   np.ptp(dy)<t['still_yaw_rad'] and len(rates)>.8*len(pts) and
                   float(np.percentile(rates,90))<t['still_rate_rps'])
        self.stationary=bool(still)
        # Freeze published stationary pose only while measured displacement from
        # this reference stays tiny. On real displacement release ALL accumulated
        # local motion, including coast/rebound; never reset local pose per STOP.
        if still and self.held_local is None:self.held_local=self.local.copy()
        if self.held_local is not None:
            d=self.local-self.held_local
            if not still or not self.neutral or np.linalg.norm(d[:2])>t['still_translation_m'] or abs(wrap(d[2]))>t['still_yaw_rad']:
                self.held_local=None
        published_local=self.held_local if self.held_local is not None else self.local
        if still and pair is not None and dt>0:
            innovation=(pair-pose(Tbc @ od['T_previous_current'] @ inverse(Tbc))[2])/dt
            if abs(innovation)<math.radians(.3):self.bias_hint=.995*self.bias_hint+.005*innovation
        tag=dict(ok=False,reason='not_scheduled',corners=None)
        if self.tag and stamp-self.last_tag>=self.cfg['tag']['period_s']-1e-9:
            tag=self.tag.update(gray,K,dist,stamp);self.last_tag=stamp
        fusion='no_tag';tag_pose=None
        if tag['ok'] and Tbc is not None and Tdt is not None and local_ok:
            tag_pose=pose(Tdt @ inverse(tag['T_camera_tag']) @ inverse(Tbc))
            # Store dock<-local transforms: moving observations are compared in
            # the SAME local frame rather than rejecting genuine vehicle motion.
            candidate=pose(matrix(tag_pose) @ inverse(matrix(self.local)))
            if self.tags:
                candidate[2]=self.tags[-1][1][2]+wrap(candidate[2]-self.tags[-1][1][2])
            self.tags.append((stamp,candidate))
            while self.tags and stamp-self.tags[0][0]>t['tag_window_s']+.3:self.tags.popleft()
            if len(self.tags)>=t['tag_min_samples'] and stamp-self.tags[0][0]>=t['tag_window_s']*.7:
                vals=np.array([p for _,p in self.tags]);med=np.median(vals,axis=0)
                spread=np.percentile(np.abs(vals-med),90,axis=0)
                stable=max(spread[:2])<=t['tag_spread_m'] and spread[2]<=t['tag_spread_rad']
                if stable:
                    prior=pose(self.anchor) if self.anchor is not None else None
                    err=med-prior if prior is not None else np.zeros(3);err[2]=wrap(err[2])
                    acceptable=prior is None or (np.linalg.norm(err[:2])<=t['tag_gate_m'] and abs(err[2])<=t['tag_gate_rad'])
                    if acceptable:
                        a=med if prior is None else prior+t['tag_weight']*err
                        self.anchor=matrix(a);self.anchor_stamp=stamp;self.conflicts=0;self.travel=0.
                        self.radius=np.maximum([t['uncertainty_seed_m']]*2+[t['uncertainty_seed_rad']],spread+np.abs(med-a))
                        fusion='qualified_tag_correction'
                    else:
                        self.conflicts+=1;fusion='tag_vo_conflict'
                        # Persistent conflict stops control. Reanchor only after
                        # a fresh full stationary window, never during movement.
                        if self.stationary and self.neutral and self.conflicts>=t['tag_min_samples']:
                            self.anchor=matrix(med);self.anchor_stamp=stamp;self.travel=0.;self.conflicts=0
                            self.radius=np.maximum([t['uncertainty_seed_m']]*2+[t['uncertainty_seed_rad']],spread)
                            fusion='stationary_tag_relocalization';self.motion_epoch+=1
                else:fusion='tag_window_unstable'
            else:fusion='collecting_tag_window'
        world=pose(self.anchor @ matrix(published_local)) if self.anchor is not None and local_ok else None
        if tag.get('corners') and Tbc is not None:
            uv=np.mean(tag['corners'],axis=0);ray=cv2.undistortPoints(np.array([[uv]],float),K,dist)[0,0]
            b=Tbc[:3,:3] @ [ray[0],ray[1],1.]
            tag['bearing_body_rad']=math.atan2(b[1],b[0])
        blind_age=None if self.anchor_stamp is None else stamp-self.anchor_stamp
        usable=bool(world is not None and self.conflicts==0 and max(self.radius[:2])<t['uncertainty_limit_m'] and self.radius[2]<t['uncertainty_limit_rad'] and
                    self.travel<=t['blind_distance_limit_m'] and blind_age<=t['blind_time_limit_s'])
        return dict(valid=world is not None,usable=usable,bounded=False,motion_bounded=False,
            pose=None if world is None else world.tolist(),error=None,confidence_radius=self.radius.tolist(),
            confidence_kind='heuristic_not_certified',stationary=self.stationary,local_valid=local_ok,
            local_pose=self.local.tolist(),local_motion=dict(xy=self.local[:2].tolist(),bounded=False),
            tracking_generation=self.generation,motion_epoch=self.motion_epoch,
            anchor_stamp=self.anchor_stamp,distance_since_anchor_m=self.travel,blind_age_s=blind_age,
            tag_fusion=dict(reason=fusion,raw_tag_pose=None if tag_pose is None else tag_pose.tolist()),
            imu_consistent=pair is not None and local_ok,
            tag={k:v for k,v in tag.items() if not k.startswith('T_')},
            odom={**{k:v for k,v in od.items() if not k.startswith('T_')},'ok':local_ok,'reason':reason},
            gyro_bias_hint_rps=self.bias_hint,
            stage_ms=dict(odom=(odone-began)*1000,tag_and_fusion=(time.monotonic()-odone)*1000))
