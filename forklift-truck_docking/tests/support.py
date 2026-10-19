"""Synthetic geometry only; never field values or hardware connections."""
from pathlib import Path
import copy
import numpy as np
from docking.config import load
from docking.runtime import CAN_CONFIG
ROOT=Path(__file__).resolve().parents[1]

def config(mode='central'):
    c=load(mode=mode)
    # Pin every field number the geometry uses: the live common.json differs per machine (Jetson had tag 0.2 m).
    c['site'].update(vehicle_width_m=1.3,bay_width_m=1.5,left_tag_offset_m=.9,tag_size_m=.3,tag_height_m=1.58,camera_height_m=.57,
                     camera_to_fork_tip_m=1.5,rear_extent_m=1.,rear_space_m=.5,tag_plane_offset_m=0.,camera_lateral_m=0.)
    c['sensor']['tag']['size_m']=.3;c['tuning']['backward_max_m']=.5
    c['tuning']['baseline_s']=0.   # controller flow tests skip the 30 s post-search baseline; test_startup_flow covers it
    T=np.eye(4);T[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];T[:3,3]=[0,0,.57]
    D=np.eye(4);D[:3,:3]=[[0,0,-1],[-1,0,0],[0,1,0]];D[:3,3]=[0,.9 if mode=='left' else 0,1.58]
    c['sensor']['calibration'].update(T_body_camera=T.tolist(),T_dock_tag=D.tolist(),camera_model=dict(
        width=640,height=480,fx=380.,fy=380.,cx=320.,cy=240.,coeffs=[0.]*5,distortion='distortion.none',serial='SIM',stream='infrared'))
    c['can']=copy.deepcopy(CAN_CONFIG);c['can']['diagnostics']['enabled']=False
    return c

def synthetic_tag(sensor,dock_pose,stamp,epoch=1,heading_bias_rad=0.):
    """Exact tag view from a body pose in the dock frame. heading_bias rotates the PnP solution about the tag
    normal: bearing and range stay exact, the PnP heading (and therefore the PnP lateral) go wrong — the
    2026-10-02 failure mode of a single tag."""
    from docking.geometry import matrix
    from docking.uncertainty import inverse
    cal=sensor['calibration'];Tbc=np.array(cal['T_body_camera']);Tdt=np.array(cal['T_dock_tag'])
    Tct=inverse(Tbc)@inverse(matrix(dock_pose))@Tdt
    if heading_bias_rad:   # about the tag's vertical axis (tag y = dock z): yaw bias, exact translation
        c,s=np.cos(heading_bias_rad),np.sin(heading_bias_rad)
        Tct=Tct@np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1.]])
    return dict(ok=True,reason='accepted',detected=True,search_candidate=True,stamp=float(stamp),sensor_epoch=epoch,
                capture_key=str(stamp),T_camera_tag=Tct,covariance=(np.eye(6)*1e-6).tolist())

class VioSource:
    """OpenVINS-like packets for a body pose expressed in an arbitrary odom gauge (T_body_imu = I)."""
    def __init__(self,gauge=(3.,-2.,.7),epoch=1):
        from docking.geometry import matrix
        self.G=matrix(gauge);self.epoch=epoch;self.previous=None
    def packet(self,t,dock_pose,gyro_norm=0.,gyro_bias=(0.,0.,0.),**extra):
        from docking.geometry import matrix
        from docking.uncertainty import inverse
        T=self.G@matrix(dock_pose)
        if self.previous is None:D=np.eye(4);delta_from=t-.1
        else:D=inverse(self.previous[1])@T;delta_from=self.previous[0]
        self.previous=(t,T)
        p=dict(quality_version=2,tracking_valid=True,continued_features=50,state_stamp=t,
            msckf_update_features=10,slam_update_features=0,correction_applied=True,correction_age_s=0.,zero_velocity_update=False,
            delta_valid=True,delta_from_stamp=delta_from,relative_transform=D,relative_covariance=np.eye(6)*1e-8,stamp=t,epoch=self.epoch,
            arrival=t+.02,imu_arrival=t+.02,features=50,initialized=True,visual_valid=True,
            T_odom_imu=T,covariance=np.eye(6)*1e-5,transport_lag_s=.01,gyro_norm=gyro_norm,gyro_bias=list(gyro_bias))
        p.update(extra);return p

def observation(now,world=(-5.,0.,0.),local=(0.,0.,0.),**kwargs):
    value=dict(capture_id=now,arrival=now,clock_fault=False,sensor_ready=True,imu_bad=False,
        imu=dict(calibration=dict(state='READY')),local_valid=True,usable=True,valid=True,
        pose=list(world),local_pose=list(local),stationary=True,motion_epoch=0,tracking_generation=0,
        blind_age_s=0.,tag=dict(ok=True,detected=True,reason='accepted'))
    value.update(kwargs)
    if value.get('pose') is not None:
        from docking.geometry import front,arrival_y
        c=config(kwargs.pop('mode','central'))
        value['fork_uncertainty']=dict(position=front(value['pose'],c).tolist(),std_m=[.0001,.0001],
            predicted_arrival_y_m=arrival_y(value['pose'],c),predicted_arrival_std_m=.0001)
    value.update(kwargs)
    return value
