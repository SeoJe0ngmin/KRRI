"""Synthetic geometry only; never field values or hardware connections."""
from pathlib import Path
import copy
import numpy as np
from docking.config import load
from docking.runtime import CAN_CONFIG
ROOT=Path(__file__).resolve().parents[2]

def config(mode='central'):
    c=load(mode=mode)
    c['site'].update(vehicle_width_m=1.3,bay_width_m=1.5,left_tag_offset_m=.9)
    T=np.eye(4);T[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];T[:3,3]=[0,0,.57]
    D=np.eye(4);D[:3,:3]=[[0,0,-1],[-1,0,0],[0,1,0]];D[:3,3]=[0,.9 if mode=='left' else 0,1.58]
    c['sensor']['calibration'].update(T_body_camera=T.tolist(),T_dock_tag=D.tolist(),camera_model=dict(
        width=640,height=480,fx=380.,fy=380.,cx=320.,cy=240.,coeffs=[0.]*5,distortion='distortion.none',serial='SIM',stream='infrared'))
    c['can']=copy.deepcopy(CAN_CONFIG);c['can']['diagnostics']['enabled']=False
    return c

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
