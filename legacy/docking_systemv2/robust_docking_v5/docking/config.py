"""One human-edited common document; tuning is explicitly heuristic, not measured bounds."""
import copy
import hashlib
import json
import math
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
COMMON=ROOT/'configs/common.json'
WORK=ROOT.parent/'work'/'docking_systemv2'/'robust_docking_v5'

TUNING=dict(preparation_s=60.,preparation_timeout_s=180.,mission_timeout_s=600.,
    settle_timeout_s=20.,recovery_timeout_s=60.,stable_s=2.,search_timeout_s=180.,
    translation_timeout_s=10.,rotation_timeout_s=8.,search_turn_timeout_s=15.,
    search_step_deg=60.,forward_max_m=.8,backward_max_m=.3,turn_max_deg=15.,
    depth=6,beam=8,expansions=48,planning_timeout_s=3.,final_zone_m=1.5,last_alignment_m=.15,
    position_tolerance_m=.03,longitudinal_tolerance_m=.05,cut_margin_px=45.,transition_margin_slack_px=5.,
    sensor_age_s=.3,imu_age_s=.25,lease_s=.3,frame_gap_s=.4,
    still_translation_m=.004,still_yaw_rad=math.radians(.35),still_rate_rps=math.radians(.5),
    tag_gate_m=.20,tag_gate_rad=math.radians(6),tag_window_s=1.,tag_min_samples=4,
    tag_spread_m=.04,tag_spread_rad=math.radians(1.5),tag_weight=.2,
    uncertainty_limit_m=.20,uncertainty_limit_rad=math.radians(6),
    uncertainty_seed_m=.02,uncertainty_seed_rad=math.radians(.5),
    drift_per_m=.025,drift_per_rad=.01,drift_per_s=.0005,
    blind_distance_limit_m=5.,blind_time_limit_s=60.,
    gyro_weight=.7,imu_disagreement_rad=math.radians(4),
    first_forward_m=.2,first_turn_deg=5.,minimum_translation_m=.04,minimum_turn_deg=.5)

def decode(node):
    if not isinstance(node,dict): return node
    if set(node)=={'description','value'}: return node['value']
    if set(node)=={'description','fields'}: return {k:decode(v) for k,v in node['fields'].items()}
    raise ValueError('설정은 description/value 또는 description/fields 형식이어야 합니다')

def mount_signature(site):
    keys=('tag_id','tag_size_m','tag_height_m','camera_height_m','camera_lateral_m')
    return hashlib.sha256(json.dumps({k:site[k] for k in keys},sort_keys=True).encode()).hexdigest()

def load(path=COMMON, mode='central', require_mount=False):
    if mode not in ('central','left'): raise ValueError('mode must be central or left')
    doc=json.loads(Path(path).read_text());c=decode(doc)
    site=c['site'];t=copy.deepcopy(TUNING)
    for k,v in c.get('operation',{}).items():
        if k not in t or not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) or v<=0:
            raise ValueError('invalid operation setting: '+k)
        t[k]=v
    def number(v):return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
    for k in ('tag_size_m','camera_height_m','tag_height_m','camera_to_fork_tip_m','rear_extent_m'):
        if not number(site[k]) or site[k]<=0: raise ValueError('invalid '+k)
    for k in ('camera_lateral_m','tag_plane_offset_m'):
        if not number(site[k]):raise ValueError('invalid '+k)
    if type(site['tag_id']) is not int or not 0<=site['tag_id']<587:raise ValueError('invalid tag36h11 tag_id')
    if site['left_tag_offset_m'] is not None and (not number(site['left_tag_offset_m']) or site['left_tag_offset_m']<0):
        raise ValueError('left_tag_offset_m은 왼쪽 양수[m]입니다')
    if mode=='left' and site['left_tag_offset_m'] is None and require_mount:
        raise ValueError('common: left_tag_offset_m 미입력')
    if require_mount:
        for k in ('vehicle_width_m','bay_width_m'):
            if not number(site[k]) or site[k]<=0:raise ValueError('common: '+k+' 미입력')
        if site['bay_width_m']<=site['vehicle_width_m']:raise ValueError('탑재부 폭은 차폭보다 커야 합니다')
    sensor=json.loads((ROOT/'configs/sensor.json').read_text())
    sensor['tag'].update(id=site['tag_id'],size_m=site['tag_size_m'])
    sensor['rig'].update(tag_height_m=site['tag_height_m'],camera_height_m=site['camera_height_m'],camera_to_fork_tip_m=site['camera_to_fork_tip_m'])
    mount=c['mount']
    good=bool(mount and mount.get('usable') and mount.get('input_signature')==mount_signature(site))
    if require_mount and not good:raise ValueError('새 장착 기준 측정 필요: preflight.sh mount')
    if good:
        R=np.array(mount['R_body_camera'],float)
        if R.shape!=(3,3) or not np.isfinite(R).all() or not np.allclose(R.T@R,np.eye(3),atol=1e-5) or abs(np.linalg.det(R)-1)>1e-5:raise ValueError('invalid mount rotation')
        T=np.eye(4);T[:3,:3]=R;T[:3,3]=[0,site['camera_lateral_m'],site['camera_height_m']]
        D=np.eye(4);D[:3,:3]=[[0,0,-1],[-1,0,0],[0,1,0]]
        D[:3,3]=[site['tag_plane_offset_m'],(site['left_tag_offset_m'] or 0) if mode=='left' else 0,site['tag_height_m']]
        sensor['calibration'].update(T_body_camera=T.tolist(),T_dock_tag=D.tolist(),camera_model=mount['camera_model'])
        sensor['camera']['serial']=mount['camera_model']['serial']
    sensor['tuning']=t
    from .diagnostics import validate_sensor_options
    validate_sensor_options(sensor)
    if require_mount:
        if not sensor['tag']['enabled'] or not sensor['odom']['enabled'] or not sensor['camera']['depth_enabled']:
            raise ValueError('drive/shadow에는 태그·depth VO가 모두 필요합니다')
        if not sensor['imu']['startup_calibration']['enabled']:raise ValueError('drive/shadow에는 IMU 초기 보정이 필요합니다')
    return dict(site=site,tuning=t,sensor=sensor,mode=mode,pivot=[.465,-.112],
                pivot_source='apriltag_v4 historical estimate; candidate planning only',
                target_front_x=-.5 if mode=='central' else 1.,common_path=str(path))

def save_mount(path, report, expected):
    path=Path(path)
    if path.read_bytes()!=expected:raise ValueError('common changed during mount measurement')
    doc=json.loads(expected);site=decode(doc)['site']
    if not report.get('usable'):raise ValueError('mount measurement failed')
    report=dict(report,input_signature=mount_signature(site))
    doc['fields']['mount']['value']=report
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(doc,ensure_ascii=False,indent=2)+'\n');tmp.replace(path)
