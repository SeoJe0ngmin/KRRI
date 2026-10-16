"""OpenVINS observations -> existing controller contract, no commanded pose."""
from collections import deque
import math
import numpy as np
from .alignment import Alignment,AnchorEvent
from .history import PoseHistory,State
from .tag import dock_measurement
from .geometry import pose,wrap,matrix
from .uncertainty import (transform,inverse,perturb,jacobian,covariance,se3_planar,
    anchor_observation,compose_cov,fork_uncertainty,target_lateral_uncertainty,drift_bound)


class Observer:
    def __init__(self,cfg):
        self.cfg=cfg;self.t=cfg['tuning'];self.u=cfg['uncertainty'];self.cal=cfg['calibration']
        self.history=PoseHistory(self.u['history_s'],self.u['history_samples'],self.u['interpolation_gap_s'])
        self.alignment=Alignment(self.u);self.stationary_window=deque();self.motion_epoch=0
        self.last_tag=dict(ok=False,reason='not_seen',detected=False,search_candidate=False)
        self.fusion=dict(reason='no_tag');self.last_tag_stamp=None;self.conflict=False
        self.travel=0.;self.previous_local=None;self.last_anchor=None
        self.last_state=None;self.rejected_tags=0;self.noise=np.zeros(3)

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

    def add_vio(self, packet, neutral=True):
        if packet.get('fault'):raise ValueError('OpenVINS: '+packet['fault'])
        if not packet.get('initialized'):
            return None
        state=self.body_state(packet)
        previous=self.last_state
        new_epoch=previous is None or previous.epoch!=state.epoch
        continuity=(not new_epoch and packet.get('delta_valid') and
                    abs(packet.get('delta_from_stamp',-1)-previous.stamp)<1e-5 and
                    state.stamp-previous.stamp<=self.t['frame_gap_s'])
        if new_epoch or not continuity:
            self.alignment.reset(state.epoch);self.stationary_window.clear();self.previous_local=None
            self.motion_epoch+=1;self.last_anchor=None;self.travel=0.;self.conflict=False;self.noise=np.zeros(3)
        if continuity:
            D=transform(packet['relative_transform']);Pd=covariance(packet['relative_covariance'],6)
            Tbi=transform(self.cal['T_body_imu']);Tib=inverse(Tbi)
            fn=lambda e:pose(Tbi@perturb(D,e)@Tib)
            d=fn(np.zeros(6));J=jacobian(fn,np.zeros(6),(2,));Q=J@Pd@J.T
            prev=pose(previous.T);current=pose(state.T)
            A=np.eye(4) if self.alignment.mean is None else matrix(self.alignment.mean)
            def anchor_delta(e):return pose(A@matrix(current)@inverse(matrix(prev)@matrix(d+e)))
            H=jacobian(anchor_delta,np.zeros(3),(2,))
            self.noise+=np.sqrt(np.maximum(np.diag(H@Q@H.T),0.))
        state.noise=self.noise.copy();self.history.add(state);self.last_state=state
        p,P=se3_planar(state.T,state.P)
        if self.previous_local is not None:self.travel+=float(np.linalg.norm(p[:2]-self.previous_local[:2]))
        self.previous_local=p.copy()
        good=bool(packet.get('visual_valid') and state.features>=self.u['min_features'])
        if good and neutral:
            self.stationary_window.append((state.stamp,p.copy(),float(packet.get('gyro_norm',math.inf))))
        else:self.stationary_window.clear()
        while self.stationary_window and state.stamp-self.stationary_window[0][0]>self.t['stable_s']+.1:
            self.stationary_window.popleft()
        still=False
        if self.stationary_window and state.stamp-self.stationary_window[0][0]>=self.t['stable_s']:
            pts=np.array([x[1] for x in self.stationary_window]);angles=np.array([wrap(x-pts[0,2]) for x in pts[:,2]])
            rates=[x[2] for x in self.stationary_window]
            still=(np.ptp(pts[:,:2],axis=0).max()<self.t['still_translation_m'] and
                np.ptp(angles)<self.t['still_yaw_rad'] and np.percentile(rates,90)<self.u['still_gyro_norm_rps'])
        return self.snapshot(packet,p,P,good,bool(still))

    def add_tag(self, tag):
        self.last_tag={k:v for k,v in tag.items() if k not in ('T_camera_tag','covariance')}
        self.last_tag_stamp=tag['stamp']
        if not tag.get('ok'):
            self.fusion=dict(reason=tag['reason']);return self.fusion
        try:
            state=self.history.at(tag['stamp'],tag['sensor_epoch'])
            op,Po=se3_planar(state.T,state.P)
            tp,Pt=dock_measurement(tag['T_camera_tag'],tag['covariance'],self.cal['T_body_camera'],
                self.cal['mount_covariance'],self.cal['T_dock_tag'],self.u['tag_position_floor_m'],self.u['tag_yaw_floor_rad'])
            a,P=anchor_observation(tp,Pt,op,np.zeros((3,3)))
            event=AnchorEvent(tag['stamp'],str(tag.get('capture_key',tag['stamp'])),tag['sensor_epoch'],a,P,state.noise.copy())
            report=self.alignment.add(event)
            report['raw_tag_pose']=tp.tolist()
            if report.get('accepted'):
                self.conflict=False
                if self.last_anchor!=self.alignment.stamp:self.travel=0.;self.last_anchor=self.alignment.stamp
            elif report['reason']=='tag_vio_conflict':self.conflict=True
        except ValueError as exc:
            report=dict(accepted=False,reason=str(exc));self.rejected_tags+=1
        self.fusion=report
        return report

    def snapshot(self, packet, local, Plocal, good, still):
        state=self.last_state;world=P=None;fork_report=None;age=None
        if self.alignment.ready:
            # VIO's arbitrary global gauge cancels in relative motion. Propagate
            # native clone/current JOINT covariance, not two absolute marginals.
            Pa=drift_bound(self.alignment.P,self.noise-self.alignment.noise)
            world,P=compose_cov(self.alignment.mean,Pa,local,np.zeros((3,3)),unknown_correlation=False)
            age=max(0.,state.stamp-self.alignment.stamp)
            # Diagnostic + gate at the controlled point, not only camera origin.
            site=self.cfg['site'];f,F=fork_uncertainty(world,P,site['camera_to_fork_tip_m'],self.u['fork_length_std_m'])
            y,Y=target_lateral_uncertainty(world,P,self.cfg['target_front_x'])
            fork_report=dict(position=f.tolist(),covariance=F.tolist(),std_m=np.sqrt(np.diag(F)).tolist(),
                predicted_arrival_y_m=y,predicted_arrival_std_m=None if Y is None else math.sqrt(Y),
                certified_error_bound=False)
        radius=None if P is None else self.u['confidence_sigma']*np.sqrt(np.maximum(np.diag(P),0))
        usable=bool(good and world is not None and not self.conflict and
            max(radius[:2])<self.t['uncertainty_limit_m'] and radius[2]<self.t['uncertainty_limit_rad'] and
            age<=self.t['blind_time_limit_s'] and self.travel<=self.t['blind_distance_limit_m'])
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
            confidence_kind='joint_relative_vio_covariance_ci_plus_declared_floors_not_certified',bounded=False,
            valid=world is not None,usable=usable,local_pose=local.tolist(),local_valid=good,
            stationary=still,tracking_generation=state.epoch,motion_epoch=self.motion_epoch,
            anchor_stamp=self.alignment.stamp,blind_age_s=age,distance_since_anchor_m=self.travel,
            tag=tag,tag_fusion=dict(self.fusion),fork_uncertainty=fork_report,
            odom=dict(ok=good,reason='openvins_tracked' if good else 'openvins_visual_unavailable',
                      features=state.features,tracked_points=packet.get('points',[])),
            imu=dict(ready=True,age_s=imu_age,gyro_bias_rps=packet.get('gyro_bias'),
                     accel_bias_mps2=packet.get('accel_bias'),calibration=dict(state='READY',reason='openvins_initialized')),
            source='OpenVINS + capture-time AprilTag alignment')
