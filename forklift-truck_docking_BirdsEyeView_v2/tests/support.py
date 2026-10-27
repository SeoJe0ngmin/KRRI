from pathlib import Path
import copy
import numpy as np
from docking.config import load
from docking.runtime import CAN_CONFIG
ROOT=Path(__file__).resolve().parents[1]

def config(mode='central'):
    c=load(mode=mode)
    c['site'].update(vehicle_width_m=1.3,truck_inner_width_m=1.5,truck_depth_m=None,left_tag_offset_m=.9,tag_size_m=.3,tag_height_m=1.58,camera_height_m=.57,
                     camera_to_fork_tip_m=1.5,rear_extent_m=1.,rear_space_m=.5,tag_plane_offset_m=0.,camera_lateral_m=0.,
                     rotation_center_behind_fork_tip_m=1.035,rotation_center_lateral_m=-.112)
    c['sensor']['tag']['size_m']=.3;c['tuning']['backward_max_m']=.5;c['pivot']=[.465,-.112]
    c['tuning']['baseline_s']=0.
    c['target_front_x']=c['sensor']['target_front_x']=-.5 if mode=='central' else 1.;c['site']['forward_path_bias_deg']=0.
    T=np.eye(4);T[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];T[:3,3]=[0,0,.57]
    D=np.eye(4);D[:3,:3]=[[0,0,-1],[-1,0,0],[0,1,0]];D[:3,3]=[0,.9 if mode=='left' else 0,1.58]
    c['sensor']['calibration'].update(T_body_camera=T.tolist(),T_dock_tag=D.tolist(),camera_model=dict(
        width=640,height=480,fx=380.,fy=380.,cx=320.,cy=240.,coeffs=[0.]*5,distortion='distortion.none',serial='SIM',stream='infrared'))
    c['can']=copy.deepcopy(CAN_CONFIG);c['can']['diagnostics']['enabled']=False
    return c

def synthetic_tag(sensor,dock_pose,stamp,epoch=1,heading_bias_rad=0.):
    from docking.geometry import matrix
    from docking.uncertainty import inverse
    cal=sensor['calibration'];Tbc=np.array(cal['T_body_camera']);Tdt=np.array(cal['T_dock_tag'])
    Tct=inverse(Tbc)@inverse(matrix(dock_pose))@Tdt
    if heading_bias_rad:
        c,s=np.cos(heading_bias_rad),np.sin(heading_bias_rad)
        Tct=Tct@np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1.]])
    return dict(ok=True,reason='accepted',detected=True,search_candidate=True,stamp=float(stamp),sensor_epoch=epoch,
                capture_key=str(stamp),T_camera_tag=Tct,covariance=(np.eye(6)*1e-6).tolist())

class VioSource:
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
        p=dict(quality_version=3,tracking_valid=True,continued_features=50,state_stamp=t,
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


def depth_obs(x_body,lateral_body,stamp,width=1.5,ok=True,pair_heading_deg=0.):
    if not ok:return dict(ok=False,reason='no_pair',stamp=stamp)
    return dict(ok=True,reason='opening',stamp=stamp,x_body_m=float(x_body),lateral_body_m=float(lateral_body),width_m=float(width),
                z_left_m=float(x_body),z_right_m=float(x_body),u_goal=320.,u_left=200.,u_right=440.,pair_heading_deg=float(pair_heading_deg),
                left_body=[float(x_body),float(lateral_body)+width/2],right_body=[float(x_body),float(lateral_body)-width/2])

def depth_from_world(world,stamp,width=1.5,**kw):
    import math
    x,y,h=world;c,s=math.cos(h),math.sin(h)
    dx,dy=-x,-y
    return depth_obs(c*dx+s*dy,-s*dx+c*dy,stamp,width=width,pair_heading_deg=-math.degrees(h),**kw)


def rails_from_world(world,rail_width,centre_y=0.,axis_rad=0.,front_x=0.,fit_max_m=3.,min_len_m=.4,x_min=.2):
    """What depth_goal.measure_rails would report for rails along the dock x axis through (front_x, centre_y) with direction
    axis_rad (dock frame), seen from the body pose: both rails in view when at least min_len_m of rail lies inside the fit range.
    lateral_error_m = body origin's signed distance left of the rail centreline, heading_err_deg = body heading minus the rail
    direction. None when the rails are not in the fit range (rails behind the camera are not seen)."""
    import math
    x,y,h=world;rel=front_x-x   # rail front ahead of the camera along the dock x (small angles)
    if rel>fit_max_m-min_len_m:return None
    d=-math.sin(axis_rad)*x+math.cos(axis_rad)*(y-centre_y)
    herr=h-axis_rad
    return dict(ok=True,reason='rails',sides=['left','right'],lateral_error_m=float(d),axis_heading_deg=float(-math.degrees(herr)),heading_err_deg=float(math.degrees(herr)),
                y_left_m=float(-d+rail_width/2),y_right_m=float(-d-rail_width/2),width_m=float(rail_width),len_left_m=1.,len_right_m=1.,n_left=40,n_right=40)
