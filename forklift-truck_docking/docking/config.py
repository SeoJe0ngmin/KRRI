"""One human-edited common document; tuning is explicitly heuristic, not measured bounds."""
import copy
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from .frames import R_REFERENCE_TAG

ROOT=Path(__file__).resolve().parents[1]
COMMON=ROOT/'configs/common.json'
MOUNT=ROOT/'configs/mount.json'   # measured on the Jetson by preflight mount --write; deploys never touch it (2026-10-20)
WORK=ROOT/'work'   # logs stay inside this folder; update_jetson.sh excludes work/ from --delete

TUNING=dict(preparation_s=5.,preparation_timeout_s=180.,mission_timeout_s=600.,   # before Enter: OpenVINS up + gyro still 5 s (user choice 2026-10-19; the floor would be ~2 s = OpenVINS init window); the 30 s baseline (baseline_s) comes after Enter and after the tag search
    settle_timeout_s=20.,recovery_timeout_s=60.,stable_s=2.,search_timeout_s=180.,
    translation_timeout_s=15.,rotation_timeout_s=8.,search_turn_timeout_s=15.,   # 15/8 = v4 FWD_SAFETY_MAX_S / ROT_SAFETY_MAX_S
    search_step_deg=50.,forward_max_m=.8,backward_max_m=.5,turn_max_deg=15.,   # search 50 = v4 (70 deg FOV overlaps); backward_max_m is overridden by site rear_space_m
    depth=6,beam=8,expansions=48,planning_timeout_s=3.,final_zone_m=1.5,last_alignment_m=.15,
    position_tolerance_m=.03,longitudinal_tolerance_m=.05,cut_margin_px=45.,transition_margin_slack_px=5.,
    sensor_age_s=.3,motion_age_s=3.,imu_age_s=3.,lease_s=1.,frame_gap_s=.4,   # motion_age_s/imu_age_s 3.0 (2026-10-20 user decision 'generous'): Jetson IMU delivery stalls of 1.4-1.7 s on the desk 10/19 tripped the v4 values 1.0 (BLIND_MAX_S/IMU_STALE_SEC) and put the vehicle into RECOVER; the predictive stop adds the observation age to its reserve, so a stale observation stops early, never late. lease_s 1.0 = v4 DEADMAN_S stays: the supervisor loop itself does not stall
    # Stationary by gyro (2026-10-19, apriltag_v4): 0.5 s window (WINDOW_S), 0.5 deg/s (IMU_MOVING_DPS);
    # straight coast is invisible to the gyro -> VIO speed <= 1/10 nominal (0.29 m/s), else stable_s after neutral.
    still_window_s=.5,still_gyro_dps=.5,still_speed_mps=.03,
    # Tag-main fusion: 0.5 s windows of >= 3 frames (v4 WINDOW_S/MIN_N). Tag-vs-VIO disagreement is judged by physics
    # (fusion.jump_allowed): margin max(3 cm, 1 cm/m x range) + 0.29 m/s x time since the last accepted window; a window
    # beyond it is skipped, after 1 s of skipping the tag wins. PnP heading as a weak reference: 5 % per window at rest.
    # σ baseline 30 s (v4 60 s halved) is record-only.
    tag_window_s=.5,tag_window_min_n=3,jump_margin_m=.03,jump_margin_per_m=.01,jump_accept_after_s=1.,heading_offset_gain=.05,baseline_s=30.,
    # Online mount yaw (apriltag_v5 F4 'good straight leg'): tag within 5 m, |bearing| <= 10 deg, > 0.5 m travelled.
    leg_max_range_m=5.,leg_max_bearing_deg=10.,leg_min_travel_m=.5,
    blind_distance_limit_m=5.,blind_time_limit_s=60.,
    minimum_translation_m=.17,minimum_turn_deg=.5,minimum_turn_fine_deg=.84,   # first-motion clips removed 2026-10-19 (v4 had none); fine floor 0.84 = v4 ROT_FLOOR_FINE_DEG   # minimum_translation 0.17 = smallest real step measured 2026-10-01 (0.167 m, apriltag_v5 F1-3)
    transition_corrections_max=5)   # correction turns at the tag-cut transition before giving up (apriltag_v5 MAX_CORRECTIONS, from KWU)

def decode(node):
    if not isinstance(node,dict): return node
    if set(node)=={'description','value'}: return node['value']
    if set(node)=={'description','fields'}: return {k:decode(v) for k,v in node['fields'].items()}
    raise ValueError('설정은 description/value 또는 description/fields 형식이어야 합니다')

def mount_path(common_path):
    return Path(common_path).with_name('mount.json')

def read_mount(common_path, decoded):
    """configs/mount.json beside common.json (2026-10-20); falls back to the pre-split 'mount' field inside common.json."""
    p=mount_path(common_path)
    if p.exists():
        m=json.loads(p.read_text());return m if isinstance(m,dict) and m else None
    return decoded.get('mount') or None

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
    rear=site.get('rear_space_m')
    if rear is not None:
        if not number(rear) or rear<0:raise ValueError('invalid rear_space_m')
        t['backward_max_m']=float(rear)                 # 뒤 공간 = 한 번 후진 상한 (v4 BACK_MAX_M, 현장 값)
    elif require_mount:raise ValueError('common: rear_space_m 미입력 (차 뒤 빈 공간)')
    if require_mount:
        for k in ('vehicle_width_m','bay_width_m'):
            if not number(site[k]) or site[k]<=0:raise ValueError('common: '+k+' 미입력')
        if site['bay_width_m']<=site['vehicle_width_m']:raise ValueError('탑재부 폭은 차폭보다 커야 합니다')
    sensor=json.loads((ROOT/'configs/runtime.json').read_text())
    gap=sensor['openvins']['max_correction_gap_s']
    if not number(gap) or not 0<gap<=1.:raise ValueError('max_correction_gap_s must be within (0, 1] seconds')
    sensor['tag'].update(id=site['tag_id'],size_m=site['tag_size_m'])
    sensor['calibration']={}
    sensor['rig']=dict(tag_height_m=site['tag_height_m'],camera_height_m=site['camera_height_m'],camera_to_fork_tip_m=site['camera_to_fork_tip_m'])
    mount=read_mount(path,c)
    good=bool(mount and mount.get('usable') and mount.get('input_signature')==mount_signature(site))
    if require_mount and not good:raise ValueError('새 장착 기준 측정 필요: preflight.sh mount')
    if good:
        R=np.array(mount['R_body_camera'],float)
        if R.shape!=(3,3) or not np.isfinite(R).all() or not np.allclose(R.T@R,np.eye(3),atol=1e-5) or abs(np.linalg.det(R)-1)>1e-5:raise ValueError('invalid mount rotation')
        T=np.eye(4);T[:3,:3]=R;T[:3,3]=[0,site['camera_lateral_m'],site['camera_height_m']]
        D=np.eye(4);D[:3,:3]=R_REFERENCE_TAG
        D[:3,3]=[site['tag_plane_offset_m'],(site['left_tag_offset_m'] or 0) if mode=='left' else 0,site['tag_height_m']]
        sensor['calibration'].update(T_body_camera=T.tolist(),T_dock_tag=D.tolist(),camera_model=mount['camera_model'])
        sensor['camera']['serial']=mount['camera_model']['serial']
        sensor['calibration']['mount_covariance']=mount.get('covariance')
    else:
        # Preview/measurement may use an explicitly nominal mounting frame.
        # drive/shadow require the validated common mounting result above.
        T=np.eye(4);T[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
        T[:3,3]=[0,site['camera_lateral_m'],site['camera_height_m']]
        D=np.eye(4);D[:3,:3]=R_REFERENCE_TAG
        D[:3,3]=[site['tag_plane_offset_m'],(site['left_tag_offset_m'] or 0) if mode=='left' else 0,site['tag_height_m']]
        sensor['calibration'].update(T_body_camera=T.tolist(),T_dock_tag=D.tolist(),camera_model=None,mount_covariance=None)
    u=sensor['uncertainty']
    if sensor['calibration']['mount_covariance'] is None:
        sensor['calibration']['mount_covariance']=np.diag([u['mount_position_std_m']**2]*3+[u['mount_rotation_std_rad']**2]*3).tolist()
    sensor['calibration']['mount_valid']=good
    sensor['calibration']['T_body_ir']=sensor['calibration']['T_body_camera']
    sensor['site']=copy.deepcopy(site)
    # Fork-tip stop target per mode (2026-10-20 moved to common.json site.target_front_x_{central,left}_m; absent/null -> the
    # previous code constants -0.5 central / +1.0 left).
    target=site.get('target_front_x_central_m') if mode=='central' else site.get('target_front_x_left_m')
    if target is None:target=-.5 if mode=='central' else 1.
    if not number(target):raise ValueError('invalid target_front_x_'+mode+'_m')
    sensor['target_front_x']=float(target)
    sensor['tuning']=t
    from .uncertainty import covariance
    covariance(sensor['calibration']['mount_covariance'],6)
    if require_mount and not sensor['tag']['enabled']:raise ValueError('drive/shadow requires AprilTag')
    return dict(site=site,tuning=t,sensor=sensor,mode=mode,pivot=[.465,-.112],
                pivot_source='apriltag_v4 historical estimate; candidate planning only',
                target_front_x=float(target),common_path=str(path))

def save_mount(path, report, expected):
    path=Path(path)
    if path.read_bytes()!=expected:raise ValueError('common changed during mount measurement')
    doc=json.loads(expected);site=decode(doc)['site']
    if not report.get('usable'):raise ValueError('mount measurement failed')
    report=dict(report,input_signature=mount_signature(site))
    # 2026-10-20: written beside common.json, never into it, so deploys can overwrite common.json (the Mac is its only
    # author) while the Jetson's measured mount survives (update_jetson.sh excludes configs/mount.json).
    out=mount_path(path);tmp=out.with_suffix('.tmp');tmp.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');tmp.replace(out)
    return out
