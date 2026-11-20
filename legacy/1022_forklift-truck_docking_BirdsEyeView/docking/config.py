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
    translation_timeout_s=15.,rotation_timeout_s=8.,search_turn_timeout_s=15.,   # 15/8 = FWD_SAFETY_MAX_S / ROT_SAFETY_MAX_S of the earlier code
    search_step_deg=50.,forward_max_m=.8,backward_max_m=.5,turn_max_deg=15.,   # search 50 (70 deg FOV overlaps); backward_max_m is overridden by site rear_space_m
    # planner_budget_s = the beam search's own time (2.4 s, formerly 0.8 x planning_timeout_s). planning_timeout_s = how long the
    # controller waits for the reply; 6 s leaves room for one queued request (2026-10-20 run 162223: a backlog of replans
    # made the reply late -> 'planner_timeout' ended the run). A late reply now re-requests up to planner_retries times.
    depth=6,beam=8,expansions=48,planning_timeout_s=6.,planner_budget_s=2.4,planner_retries=3,final_zone_m=1.5,last_alignment_m=.15,
    # Far from the tag cut a step may be 1.5 m (earlier step cap); inside far_from_cut_m of it 0.8 m (2026-10-20 user decision).
    forward_max_far_m=1.5,far_from_cut_m=3.,
    position_tolerance_m=.03,longitudinal_tolerance_m=.05,cut_margin_px=45.,transition_margin_slack_px=5.,
    sensor_age_s=.3,motion_age_s=3.,imu_age_s=3.,lease_s=1.,frame_gap_s=.4,   # motion_age_s/imu_age_s 3.0 (2026-10-20 user decision 'generous'): Jetson IMU delivery stalls of 1.4-1.7 s on the desk 10/19 tripped the earlier values 1.0 (BLIND_MAX_S/IMU_STALE_SEC) and put the vehicle into RECOVER; the predictive stop adds the observation age to its reserve, so a stale observation stops early, never late. lease_s 1.0 = the earlier DEADMAN_S stays: the supervisor loop itself does not stall
    # Stationary by gyro (2026-10-19): 0.5 s window (WINDOW_S), 0.5 deg/s (IMU_MOVING_DPS);
    # straight coast is invisible to the gyro -> VIO speed <= 1/10 nominal (0.29 m/s), else stable_s after neutral.
    still_window_s=.5,still_gyro_dps=.5,still_speed_mps=.03,
    # Tag-main fusion: 0.5 s windows of >= 3 frames (WINDOW_S/MIN_N). Tag-vs-VIO disagreement is judged by physics
    # (fusion.jump_allowed): margin max(3 cm, 1 cm/m x range) + 0.29 m/s x time since the last accepted window; a window
    # beyond it is skipped, after 1 s of skipping the tag wins. PnP heading as a weak reference: 5 % per window at rest.
    # σ baseline 30 s (60 s halved) is record-only.
    tag_window_s=.5,tag_window_min_n=3,jump_margin_m=.03,jump_margin_per_m=.01,jump_accept_after_s=1.,heading_offset_gain=.05,baseline_s=30.,
    # Online mount yaw ('good straight leg' rule): tag within 5 m, |bearing| <= 10 deg, > 0.5 m travelled.
    # 2026-10-20 KW: 5 m / 10 deg (central tag) never matched the left tag 1.27 m beside the lane (bearing 15-25 deg
    # inside 5 m, > 5 m beyond) -> 13/13 legs rejected, mount yaw never learned, entered 9 cm off. 9 m = today's start,
    # 25 deg = bearing at the tag cut (3.3 m) with margin; 10/1 data: far/oblique legs were only ~0.3 deg worse.
    leg_max_range_m=9.,leg_max_bearing_deg=25.,leg_min_travel_m=.5,
    blind_distance_limit_m=6.,blind_time_limit_s=60.,   # 6 (2026-10-20): the fork-front camera loses the tag at 3.3 m -> blind leg 4.1 m sat at the old 5 m bound
    minimum_translation_m=.17,minimum_turn_deg=.5,minimum_turn_fine_deg=.84,   # first-motion clips removed 2026-10-19 (the earlier code had none); fine floor 0.84 = ROT_FLOOR_FINE_DEG   # minimum_translation 0.17 = smallest real step measured 2026-10-01 (0.167 m)
    transition_corrections_max=5,
    # Sidestep (2026-10-20 KW, user decision): a start far off the centreline is fixed first by
    # turning to face the centreline, driving the lateral error, turning back. Else the beam planner kills the lateral
    # error with a long oblique approach and arrives at the tag cut with ~30 deg to straighten, and each in-place turn
    # there shifts the camera sideways (pivot 0.78 m behind it) -> 'no_feasible_progress' (run 160536).
    # 2026-10-21: the drive distance accounts for both in-place turns (pivot arc, see controller); fraction 1.0 = aim for the
    # centreline; one retry if the re-measured lateral is still > sidestep_retry_lateral_m right after a sidestep.
    # 2026-10-22: sidestep whenever the lateral is at least one real step (user: 'always'); room 1 m is what the two turns need.
    sidestep_min_lateral_m=.17,sidestep_min_room_m=1.,sidestep_fraction=1.,sidestep_max_m=1.5,sidestep_retry_lateral_m=.17,sidestep_max_count=2,
    # flow: rough approach (heading straight, sidestep, one long drive to rough_stop_before_cut_m before the tag cut) ->
    # heading re-zero from the tag -> fine stage on the depth-measured entrance (bundles of turn/drive/turn-back, straight
    # legs <= fine_step_max_m) -> final check at final_check_distance_m (computed from the depth HFOV unless set) ->
    # entry on VIO + forward_path_bias_deg. Tolerances are editable in common.json 'operation'.
    rough_stop_before_cut_m=1.,rough_leg_max_m=9.,straight_heading_tol_deg=1.,
    final_lateral_tol_m=.03,final_heading_tol_deg=1.,fine_corrections_max=5,fine_step_max_m=1.5,
    depth_goal_max_range_m=4.5,truck_width_tol_m=.2,final_check_distance_m=None,depth_hfov_deg=80.,final_check_margin_m=.1,
    depth_measure_s=.6,depth_measure_min_frames=4,depth_wait_s=4.,heading_rezero_s=2.,heading_rezero_timeout_s=8.,
    no_progress_s=2.,no_progress_stops_max=3,entry_timed_scale=2.)   # transition_corrections_max: correction turns at the tag-cut transition before giving up (MAX_CORRECTIONS, from KWU)

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
        if k not in t:raise ValueError('invalid operation setting: '+k)
        if v is None:continue   # empty = keep the code default (e.g. final_check_distance_m: computed)
        if not isinstance(v,(int,float)) or isinstance(v,bool) or not math.isfinite(v) or v<=0:
            raise ValueError('invalid operation setting: '+k)
        t[k]=v
    def number(v):return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
    # Camera position on the vehicle (2026-10-21): longitudinal from the fork tip (camera_to_fork_tip_m), lateral from the
    # vehicle centreline (camera_lateral_m, left +), height from the floor. rear_extent_m may be left empty: it is then
    # vehicle_length_m - camera_to_fork_tip_m, so moving the camera only needs the two tape values.
    if site.get('rear_extent_m') is None and number(site.get('vehicle_length_m')) and number(site.get('camera_to_fork_tip_m')):
        site['rear_extent_m']=float(site['vehicle_length_m'])-float(site['camera_to_fork_tip_m'])
    for k in ('tag_size_m','camera_height_m','tag_height_m','camera_to_fork_tip_m','rear_extent_m'):
        if not number(site[k]) or site[k]<=0: raise ValueError('invalid '+k)
    for k in ('camera_lateral_m','tag_plane_offset_m'):
        if not number(site[k]):raise ValueError('invalid '+k)
    # Straight-drive drift (KWU FORWARD_PATH_BIAS_DEG). Empty = 0: no compensation until it is measured (2026-10-21).
    b=site.get('forward_path_bias_deg');site['forward_path_bias_deg']=0. if b is None else float(b)
    if not number(site['forward_path_bias_deg']) or abs(site['forward_path_bias_deg'])>10:raise ValueError('invalid forward_path_bias_deg')
    if type(site['tag_id']) is not int or not 0<=site['tag_id']<587:raise ValueError('invalid tag36h11 tag_id')
    # Tag side = sign (2026-10-22): left +, right -, same convention as camera_lateral_m / forward_path_bias_deg.
    if site['left_tag_offset_m'] is not None and not number(site['left_tag_offset_m']):
        raise ValueError('left_tag_offset_m은 숫자[m]입니다 (왼쪽 +, 오른쪽 −)')
    if mode=='left' and site['left_tag_offset_m'] is None and require_mount:
        raise ValueError('common: left_tag_offset_m 미입력')
    rear=site.get('rear_space_m')
    if rear is not None:
        if not number(rear) or rear<0:raise ValueError('invalid rear_space_m')
        t['backward_max_m']=float(rear)                 # 뒤 공간 = 한 번 후진 상한 (BACK_MAX_M, 현장 값)
    elif require_mount:raise ValueError('common: rear_space_m 미입력 (차 뒤 빈 공간)')
    if 'bay_width_m' in site:raise ValueError('common: bay_width_m 은 truck_inner_width_m 으로 이름이 바뀌었습니다')
    if require_mount:
        for k in ('vehicle_width_m','truck_inner_width_m'):
            if not number(site.get(k)) or site[k]<=0:raise ValueError('common: '+k+' 미입력')
        if site['truck_inner_width_m']<=site['vehicle_width_m']:raise ValueError('탑재부 안쪽 폭은 차폭보다 커야 합니다')
    depth=site.get('truck_depth_m')
    if depth is not None and (not number(depth) or depth<=0):raise ValueError('invalid truck_depth_m')
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
    if depth is not None and require_mount and float(target)>float(depth)-.05:
        raise ValueError('포크 끝 목표 %.2f m 가 탑재부 깊이 %.2f m 를 넘습니다 (target_front_x_%s_m / truck_depth_m)'%(float(target),float(depth),mode))
    sensor['tuning']=t
    from .uncertainty import covariance
    covariance(sensor['calibration']['mount_covariance'],6)
    if require_mount and not sensor['tag']['enabled']:raise ValueError('drive/shadow requires AprilTag')
    # Rotation centre (planning kinematics only) in the body frame, whose origin is ON THE VEHICLE CENTRELINE at the camera's
    # longitudinal position: x = camera_to_fork_tip - (centre behind the fork tip), y = centre lateral from the centreline.
    # Both inputs are vehicle properties, so moving the camera (forward/back via camera_to_fork_tip_m, sideways via
    # camera_lateral_m) recomputes it with no other change (2026-10-20/21).
    behind=site.get('rotation_center_behind_fork_tip_m');behind=1.005 if behind is None else float(behind)
    plat=site.get('rotation_center_lateral_m');plat=-.112 if plat is None else float(plat)
    if not number(behind) or not number(plat):raise ValueError('invalid rotation_center_*')
    return dict(site=site,tuning=t,sensor=sensor,mode=mode,pivot=[float(site['camera_to_fork_tip_m'])-behind,plat],
                pivot_source='earlier calibrate (0.465 ahead of the carriage camera) expressed from the fork tip; candidate planning only',
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
