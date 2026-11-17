"""OpenVINS observations -> existing controller contract, no commanded pose.

2026-10-19: tag main, VIO assist (see fusion.py). Stationary judgment by gyro (stationary.py).
"""
import math
import numpy as np
from .stationary import StationaryDetector
from .history import PoseHistory,State
from .fusion import Fusion,tag_bearing_range,pnp_dock_pose
from .geometry import pose,path_bias
from .vio_quality import quality
from .uncertainty import (transform,inverse,perturb,jacobian,covariance,se3_planar,
    fork_uncertainty,target_lateral_uncertainty)


class Observer:
    def __init__(self,cfg):
        self.cfg=cfg;self.t=cfg['tuning'];self.u=cfg['uncertainty'];self.cal=cfg['calibration']
        self.history=PoseHistory(self.u['history_s'],self.u['history_samples'],self.u['interpolation_gap_s'])
        self.fusion=Fusion(self.t,self.cal['T_body_camera'],self.cal['T_dock_tag']);self.motion_epoch=0
        self.last_tag=dict(ok=False,reason='not_seen',detected=False,search_candidate=False)
        self.tag_report=dict(reason='no_tag');self.last_tag_stamp=None
        self.previous_local=None;self.last_state=None;self.rejected_tags=0
        self.stationary_detector=StationaryDetector(self.t,cfg.get('camera',{}).get('gyro_hz',200.))
        self.still=False
        self.rezero_target=0   # requested count (supervisor hint); applied when enough still tag frames exist

    def body_state(self, packet):
        # Native adapter publishes T_odom_imu and a world-frame left-error P.
        T=transform(packet['T_odom_imu']);P=covariance(packet['covariance'],6,True)
        Tbi=transform(self.cal['T_body_imu'])
        Tib=inverse(Tbi)
        Tb=T @ Tib
        # Analytic left-error Jacobian avoids Rodrigues inverse's small-angle
        # dead zone, which would otherwise erase orientation covariance.
        r=T[:3,:3]@Tib[:3,3]
        cross=np.array([[0.,-r[2],r[1]],[r[2],0.,-r[0]],[-r[1],r[0],0.]])
        J=np.eye(6);J[:3,3:]=-cross
        return State(float(packet['stamp']),int(packet['epoch']),Tb,J@P@J.T,
                     float(packet['arrival']),int(packet['features']))

    def add_gyro(self,stamp,w):
        """Raw gyro sample (200 Hz) for the stationary judgment; fed by the camera worker."""
        self.stationary_detector.add_gyro(stamp,w)

    def add_vio(self, packet, neutral=True):
        if packet.get('fault'):raise ValueError('OpenVINS: '+packet['fault'])
        if not packet.get('initialized'):
            return None
        self.stationary_detector.set_bias(packet.get('gyro_bias'))
        if packet.get('baseline_lock'):self.fusion.baseline.freeze()   # controller finished its 30 s baseline phase (after the tag search)
        report=quality(packet,self.cfg)
        if not report['state_current']:
            # Never label an old estimator state with a new camera timestamp or
            # feed it to tag fusion, relative motion or stationary detection.
            self.history.items.clear();self.last_state=None
            self.fusion.reset(packet['epoch']);self.stationary_detector.reset_window()
            return dict(capture_id=packet['stamp'],arrival=packet['arrival'],sensor_epoch=packet['epoch'],
                sensor_ready=True,local_valid=False,local_pose=[],pose=None,usable=False,stationary=False,
                clock_fault=False,imu_bad=True,transport_lag_s=packet['transport_lag_s'],
                tag=dict(self.last_tag),odom=report,imu=dict(ready=True))
        state=self.body_state(packet)
        previous=self.last_state
        new_epoch=previous is None or previous.epoch!=state.epoch
        continuity=(not new_epoch and packet.get('delta_valid') and
                    abs(packet.get('delta_from_stamp',-1)-previous.stamp)<1e-5 and
                    state.stamp-previous.stamp<=self.t['frame_gap_s'])
        if new_epoch or not continuity:
            self.fusion.reset(state.epoch);self.stationary_detector.reset_window();self.previous_local=None
            self.motion_epoch+=1
        p,P=se3_planar(state.T,state.P)
        if continuity:
            D=transform(packet['relative_transform']);Pd=covariance(packet['relative_covariance'],6)
            Tbi=transform(self.cal['T_body_imu']);Tib=inverse(Tbi)
            fn=lambda e:pose(Tbi@perturb(D,e)@Tib)
            d=fn(np.zeros(6));J=jacobian(fn,np.zeros(6),(2,));Q=J@Pd@J.T
            heading=p[2]+(self.fusion.offset or 0.)
            self.fusion.vio_step(d[:2],Q,heading)
        # Additional engineering uncertainty per missing IMU interval (counted, never fatal — rule of 2026-10-19).
        # Native integration uses real timestamps; no synthetic zero samples.
        gaps=packet.get('imu_gaps',0)
        extra=max(0,gaps-getattr(self,'last_imu_gaps',0))
        self.last_imu_gaps=gaps
        if extra:self.fusion.Pvio=self.fusion.Pvio+extra*np.diag([.03**2,.03**2,math.radians(2.)**2])
        self.history.add(state);self.last_state=state
        self.previous_local=p.copy()
        report=quality(packet,self.cfg,continuity=bool(continuity));good=report['ok']
        report['imu_gaps']=gaps
        report['imu_gap_uncertainty_added']=bool(extra)
        imu_age=packet['arrival']-packet.get('imu_arrival',-math.inf)
        since_neutral=packet.get('since_neutral_s',math.inf if neutral else None)
        self.still=self.stationary_detector.update(state.stamp,p,neutral and packet.get('captured_after_neutral',True),
            since_neutral,good and 0<=imu_age<=self.t['imu_age_s'],packet.get('gyro_norm'))
        return self.snapshot(packet,p,P,report,bool(self.still))

    def request_rezero(self,count):
        """Supervisor asks (count increases) for a heading re-zero from the tag; applied on the next still tag frames."""
        self.rezero_target=max(self.rezero_target,int(count))

    def add_tag(self, tag):
        self.last_tag={k:v for k,v in tag.items() if k not in ('T_camera_tag','covariance')}
        self.last_tag_stamp=tag['stamp']
        if not tag.get('ok'):
            self.tag_report=dict(accepted=False,reason=tag['reason']);return self.tag_report
        try:
            state=self.history.at(tag['stamp'],tag['sensor_epoch'])
            op,_=se3_planar(state.T,state.P)
            bearing,rng=tag_bearing_range(tag['T_camera_tag'],self.cal['T_body_camera'])
            raw=pnp_dock_pose(tag['T_camera_tag'],self.cal['T_body_camera'],self.cal['T_dock_tag'])
            report=self.fusion.tag(tag['stamp'],bearing,rng,float(raw[2]),op,self.still,
                None if self.stationary_detector.last_rate is None else math.degrees(self.stationary_detector.last_rate))
            report=dict(report,raw_tag_pose=raw.tolist(),bearing_deg=math.degrees(bearing),range_m=rng)
        except ValueError as exc:
            report=dict(accepted=False,reason=str(exc));self.rejected_tags+=1
        self.tag_report=report
        if self.rezero_target>self.fusion.rezero_count and self.still and report.get('accepted'):
            self.fusion.rezero(self.t['heading_rezero_s'],int(self.t['depth_measure_min_frames']))   # None until enough still frames
        return report

    def snapshot(self, packet, local, Plocal, odom, still):
        good=odom['ok']
        state=self.last_state;fork_report=None;age=None
        world,P,parts=self.fusion.estimate(local)
        if world is not None:
            age=max(0.,state.stamp-self.fusion.anchor_stamp)
            # Diagnostic at the controlled point, not only the camera origin.
            site=self.cfg['site'];f,F=fork_uncertainty(world,P,site['camera_to_fork_tip_m'],self.u['fork_length_std_m'])
            y,Y=target_lateral_uncertainty(world,P,self.cfg['target_front_x'],path_bias(self.cfg))
            fork_report=dict(position=f.tolist(),covariance=F.tolist(),std_m=np.sqrt(np.diag(F)).tolist(),
                predicted_arrival_y_m=y,predicted_arrival_std_m=None if Y is None else math.sqrt(Y),
                certified_error_bound=False)
        radius=None if P is None else self.u['confidence_sigma']*np.sqrt(np.maximum(np.diag(P),0))
        # σ is recorded, not a gate (2026-10-19). Only VIO-only duration/distance bound the pose.
        usable=bool(good and world is not None and age<=self.t['blind_time_limit_s'] and self.fusion.travel<=self.t['blind_distance_limit_m'])
        tag=dict(self.last_tag)
        if self.last_tag_stamp is None or state.stamp-self.last_tag_stamp>self.u['tag_status_age_s']:
            tag=dict(ok=False,reason='missing_or_stale',detected=False,search_candidate=False)
        else:
            # Controller's detected field means a useful search candidate.
            tag['detected']=tag.get('search_candidate',False)
        imu_age=packet['arrival']-packet.get('imu_arrival',-math.inf)
        return dict(capture_id=state.stamp,arrival=packet['arrival'],sensor_epoch=state.epoch,
            clock_fault=False,sensor_ready=True,imu_bad=not 0<=imu_age<=self.t['imu_age_s'],
            transport_lag_s=packet['transport_lag_s'],pose=None if world is None else world.tolist(),
            covariance=None if P is None else P.tolist(),confidence_radius=None if radius is None else radius.tolist(),
            confidence_kind='tag_window_plus_30s_baseline_plus_vio_relative_variance_not_certified',bounded=False,
            sigma=parts,sigma_baseline=self.fusion.baseline.value,baseline_ready=self.fusion.baseline.value is not None,
            valid=world is not None,usable=usable,local_pose=local.tolist(),local_valid=good,
            stationary=still,stationary_report=dict(self.stationary_detector.report),tracking_generation=state.epoch,motion_epoch=self.motion_epoch,
            anchor_stamp=self.fusion.anchor_stamp,blind_age_s=age,distance_since_anchor_m=self.fusion.travel,
            heading_source='vio_yaw_plus_offset' if self.fusion.offset is not None else None,
            mount_yaw_online_deg=math.degrees(self.fusion.mount_yaw),good_legs=len(self.fusion.mount_yaw_samples),last_leg=self.fusion.last_leg,
            tag=tag,tag_fusion=dict(self.tag_report),tag_position_window=self.fusion.last_window,fork_uncertainty=fork_report,
            heading_rezero=dict(count=self.fusion.rezero_count,requested=self.rezero_target,info=self.fusion.rezero_info),
            odom=dict(odom,usable_for_docking=usable),
            imu=dict(ready=True,age_s=imu_age,gyro_bias_rps=packet.get('gyro_bias'),
                     accel_bias_mps2=packet.get('accel_bias'),calibration=dict(state='READY',reason='openvins_initialized')),
            source='AprilTag bearing/range windows (main) + OpenVINS heading/propagation (assist)')
