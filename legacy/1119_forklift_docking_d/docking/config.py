"""One human-edited common document; tuning is explicitly heuristic, not measured bounds."""

import copy
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from .frames import R_REFERENCE_TAG

ROOT = Path(__file__).resolve().parents[1]
COMMON = ROOT / 'configs/common.json'
MOUNT = (
    ROOT / 'configs/mount.json'
)  # measured on the Jetson by preflight mount --write; deploys never touch it (2026-10-20)
WORK = ROOT / 'work'  # logs stay inside this folder; update_jetson.sh excludes work/ from --delete

TUNING = dict(
    preparation_s=5.0,
    preparation_timeout_s=180.0,
    mission_timeout_s=600.0,  # before Enter: OpenVINS up + gyro still 5 s (user choice 2026-10-19; the floor would be ~2 s = OpenVINS init window); the 30 s baseline (baseline_s) comes after Enter and after the tag search
    settle_timeout_s=20.0,
    recovery_timeout_s=60.0,
    stable_s=2.0,
    search_timeout_s=180.0,
    translation_timeout_s=15.0,
    rotation_timeout_s=8.0,
    search_turn_timeout_s=15.0,  # 15/8 = FWD_SAFETY_MAX_S / ROT_SAFETY_MAX_S of the earlier code
    search_step_deg=50.0,
    forward_max_m=0.8,
    backward_max_m=0.5,
    turn_max_deg=15.0,  # search 50 (70 deg FOV overlaps); backward_max_m is overridden by site rear_space_m
    # planner_budget_s = the beam search's own time (2.4 s, formerly 0.8 x planning_timeout_s). planning_timeout_s = how long the
    # controller waits for the reply; 6 s leaves room for one queued request (2026-10-20 run 162223: a backlog of replans
    # made the reply late -> 'planner_timeout' ended the run). A late reply now re-requests up to planner_retries times.
    depth=6,
    beam=8,
    expansions=48,
    planning_timeout_s=6.0,
    planner_budget_s=2.4,
    planner_retries=3,
    final_zone_m=1.5,
    last_alignment_m=0.15,
    # Far from the tag cut a step may be 1.5 m (earlier step cap); inside far_from_cut_m of it 0.8 m (2026-10-20 user decision).
    forward_max_far_m=1.5,
    far_from_cut_m=3.0,
    position_tolerance_m=0.03,
    longitudinal_tolerance_m=0.05,
    cut_margin_px=45.0,
    transition_margin_slack_px=5.0,
    sensor_age_s=0.3,
    motion_age_s=3.0,
    imu_age_s=3.0,
    lease_s=1.0,
    frame_gap_s=0.4,  # motion_age_s/imu_age_s 3.0 (2026-10-20 user decision 'generous'): Jetson IMU delivery stalls of 1.4-1.7 s on the desk 10/19 tripped the earlier values 1.0 (BLIND_MAX_S/IMU_STALE_SEC) and put the vehicle into RECOVER; the predictive stop adds the observation age to its reserve, so a stale observation stops early, never late. lease_s 1.0 = the earlier DEADMAN_S stays: the supervisor loop itself does not stall
    # Stationary by gyro (2026-10-19): 0.5 s window (WINDOW_S), 0.5 deg/s (IMU_MOVING_DPS);
    # straight coast is invisible to the gyro -> VIO speed <= 1/10 nominal (0.29 m/s), else stable_s after neutral.
    still_window_s=0.5,
    still_gyro_dps=0.5,
    still_speed_mps=0.03,
    # Tag-main fusion: 0.5 s windows of >= 3 frames (WINDOW_S/MIN_N). Tag-vs-VIO disagreement is judged by physics
    # (fusion.jump_allowed): margin max(3 cm, 1 cm/m x range) + 0.29 m/s x time since the last accepted window; a window
    # beyond it is skipped, after 1 s of skipping the tag wins. PnP heading as a weak reference: 5 % per window at rest.
    # σ baseline 30 s (60 s halved) is record-only.
    tag_window_s=0.5,
    tag_window_min_n=3,
    jump_margin_m=0.03,
    jump_margin_per_m=0.01,
    jump_accept_after_s=1.0,
    heading_offset_gain=0.05,
    baseline_s=30.0,
    # Online mount yaw ('good straight leg' rule): tag within 5 m, |bearing| <= 10 deg, > 0.5 m travelled.
    # 2026-10-20 KW: 5 m / 10 deg (central tag) never matched the left tag 1.27 m beside the lane (bearing 15-25 deg
    # inside 5 m, > 5 m beyond) -> 13/13 legs rejected, mount yaw never learned, entered 9 cm off. 9 m = today's start,
    # 25 deg = bearing at the tag cut (3.3 m) with margin; 10/1 data: far/oblique legs were only ~0.3 deg worse.
    leg_max_range_m=9.0,
    leg_max_bearing_deg=25.0,
    leg_min_travel_m=0.5,
    blind_distance_limit_m=6.0,
    blind_time_limit_s=60.0,  # 6 (2026-10-20): the fork-front camera loses the tag at 3.3 m -> blind leg 4.1 m sat at the old 5 m bound
    minimum_translation_m=0.17,
    minimum_turn_deg=0.5,
    minimum_turn_fine_deg=0.84,  # first-motion clips removed 2026-10-19 (the earlier code had none); fine floor 0.84 = ROT_FLOOR_FINE_DEG   # minimum_translation 0.17 = smallest real step measured 2026-10-01 (0.167 m)
    transition_corrections_max=5,
    # Sidestep (2026-10-20 KW, user decision): a start far off the centreline is fixed first by
    # turning to face the centreline, driving the lateral error, turning back. Else the beam planner kills the lateral
    # error with a long oblique approach and arrives at the tag cut with ~30 deg to straighten, and each in-place turn
    # there shifts the camera sideways (pivot 0.78 m behind it) -> 'no_feasible_progress' (run 160536).
    # 2026-10-21: the drive distance accounts for both in-place turns (pivot arc, see controller); fraction 1.0 = aim for the
    # centreline; one retry if the re-measured lateral is still > sidestep_retry_lateral_m right after a sidestep.
    # 2026-10-22: sidestep whenever the lateral is at least one real step (user: 'always'); room 1 m is what the two turns need.
    sidestep_min_lateral_m=0.17,
    sidestep_min_room_m=1.0,
    sidestep_fraction=1.0,
    sidestep_max_m=1.5,
    sidestep_retry_lateral_m=0.17,
    sidestep_max_count=2,
    # flow: rough approach (heading straight, sidestep, one long drive to rough_stop_before_cut_m before the tag cut) ->
    # heading re-zero from the tag -> fine stage on the depth-measured entrance (bundles of turn/drive/turn-back, straight
    # legs <= fine_step_max_m) -> final check at final_check_distance_m (computed from the depth HFOV unless set) ->
    # entry on VIO + forward_path_bias_deg. Tolerances are editable in common.json 'operation'.
    rough_stop_before_cut_m=1.0,
    rough_leg_max_m=9.0,
    straight_heading_tol_deg=1.0,
    final_lateral_tol_m=0.03,
    final_heading_tol_deg=1.0,
    fine_corrections_max=5,
    fine_step_max_m=1.5,
    depth_goal_max_range_m=6.5,
    truck_width_tol_m=0.2,
    final_check_distance_m=None,
    depth_hfov_deg=80.0,
    final_check_margin_m=0.1,
    depth_measure_s=0.6,
    depth_measure_min_frames=4,
    depth_wait_s=4.0,
    heading_rezero_s=2.0,
    heading_rezero_timeout_s=8.0,
    no_progress_s=2.0,
    no_progress_stops_max=3,
    entry_timed_scale=2.0,
    # Weak commands (2026-10-29 BirdsEyeView_v4): after the rough stop (depth-only stage: FINE/FINAL/ENTRY/FALLBACK) every turn,
    # bundle drive and back-out uses a CAN byte closer to neutral so the smallest real step/turn gets smaller (2026-10-28: fine
    # turns of 1-2 deg came out 1.4-3.7 deg). Long straights (fine_straight, entry) keep the normal byte. Bytes come from
    # common.json operation; defaults = forward_slow (97) and the fine rotation (+-20) = no change until set on site.
    # weak_deadband_s: a weak command that has not moved the vehicle this long is in the dead band -> that leg is re-issued
    # with the normal byte and the weak kind (drive/rotate) is retired for the rest of the run.
    fine_weak_commands=1,
    weak_forward_byte=97,
    weak_rotate_delta=20,
    weak_deadband_s=4.0,
    # docking (2026-10-30): depth windows and the choice of reference. tag_fresh_s: a tag window this recent makes the tag the
    # lateral/distance reference. entrance_window_m / lattice_window_m: depth slabs around the entrance line / the lattice (site
    # truck_depth_m) in which the doors' inner lines / the lattice members are looked for. lattice_pair_tol_m: the 2|3 member pair
    # width tolerance. lattice_good_*: the lattice pair is believed over the doors only with >= frames readings, a window spread
    # <= spread and a range <= range (farther its lateral is worse than the doors'). lattice_axis_*: the lattice line's normal
    # becomes the heading target only after adopt_stops consecutive stops disagreeing with the dock axis by > adopt_deg in the same
    # direction (a tilted tag plane), never beyond max_deg; spread_deg gates one window. projector_when_stopped: 1 = the IR
    # projector on only while the vehicle stands (depth texture in the shaded bay; a static dot pattern cannot fool the VIO),
    # 0 = always off. Values are judgement values, editable in common.json operation.
    tag_fresh_s=1.0,
    entrance_window_m=0.1,
    lattice_window_m=0.35,
    lattice_pair_tol_m=0.05,
    lattice_line_min_len_m=1.0,
    lattice_line_max_residual_m=0.05,
    lattice_max_heading_deg=15.0,
    lattice_good_frames=4,
    lattice_good_spread_m=0.02,
    lattice_good_range_m=3.5,
    lattice_axis_enabled=1,
    lattice_axis_spread_deg=1.0,
    lattice_axis_max_deg=10.0,
    lattice_axis_adopt_deg=1.0,
    lattice_axis_adopt_stops=2,
    projector_when_stopped=1,
    projector_on_delay_s=0.3,
    # 2026-10-30 run 102526: the 2|3 centre scatters 2-5 cm between 0.6 s windows at 3 m while the final tolerance is 3 cm, so the
    # final check flipped pass/fail and backed out twice for nothing. final_measure_s: the window at the final-check distance;
    # final_aim_max_m: a lateral error up to this is taken out by aiming the entry run (asin(e/L)) instead of backing out.
    final_measure_s=1.5,
    final_aim_max_m=0.04,
    # docking_v3: the lattice is a fixed landmark, so its entrance-plane lane centre is the median over the last
    # lattice_history_stops stops (each stop's own window median; shifted with every anchor), not one stop's reading; 1 = off.
    lattice_history_stops=3,
    # v4 (2026-11-01): online straight-drive drift. forward_path_bias_deg was measured (-1.7..-4.2 deg on 2026-10-20) but left empty;
    # with the inner tags visible the drift of every straight leg >= path_bias_min_travel_m is measured (dock-frame track direction
    # minus the heading at the start) and, after path_bias_min_legs legs, the median is used as the drift (bounded by
    # path_bias_max_deg) when the configured value is empty. path_bias_online 0 = record only.
    path_bias_online=1,
    path_bias_min_travel_m=0.5,
    path_bias_min_legs=2,
    path_bias_max_deg=5.0,
    # 직진하는 동안 기준 태그가 그대로였던 쏠림 표본이 path_bias_min_legs 개 이상이면 그것만 쓴다(0 = 표본을 가리지 않음)
    path_bias_clean_first=1,
    # v4: the two-tag bundle is the heading/position reference only within this range (camera -> tag plane). Its heading comes
    # from the range difference of the two tags, i.e. from their apparent sizes: at 4.9 m (38 px tags) one frame is ~3 deg off
    # (rendered test 2026-11-01), at 3.1 m ~0.5 deg, at 1.6 m ~0.2 deg. Farther out each inner tag is a bearing/range anchor
    # like the entrance tag ('inner' rows) and the heading stays the tag-zeroed VIO.
    inner_bundle_max_range_m=3.5,
    # 2026-11-12 정민님 결정(태그 주 기준): 0 = 태그가 신선하면 depth 격자 자 · depth 직진 감시가 차를 세우지 않음(기록만), 1 = 예전처럼 먼저 닿는 쪽
    entry_depth_ruler_with_tag=0,
    bundle_watch_depth_with_tag=0,
    # 겨냥 진입 각도에 제자리 회전의 회전중심 둘레 이동을 넣는다(1), 예전 식 asin(e/L)(0)
    entry_aim_pivot=1,
    # 마지막 확인에서 포크 끝이 닿을 좌우까지 본다(1) — 좌우·방향이 따로는 기준 안이어도 합쳐서 벗어나면 겨냥해 진입
    final_arrival_check=1,
    # 진입 중 태그 묶음으로 포크 끝이 닿을 좌우를 계속 본다(1): 입구선 앞에서 entry_lateral_watch_m 넘게 벗어나면 세우고 다시 확인
    entry_lateral_watch=1,
    entry_lateral_watch_m=0.03,
    # depth 격자 쌍이 믿는 위치의 중심선에서 이만큼 넘게 떨어지면 옆 칸으로 보고 버린다(칸 피치 0.405 의 절반), 0 = 끔
    lattice_alias_gate_m=0.2,
    # depth 높이 띠의 점이 이보다 적은 장면은 쓰지 않는다: 10-30 정오 판 800~2300, 오후 판 5000 이상(먼 거리 2200~3900), 0 = 끔
    depth_min_points=2000,
    # 마지막 확인은 서 있는 동안 모은 태그 기준 자세의 가운데 값으로 판단한다(1), 그 순간의 자세(0)
    final_pose_average=0,
    # 겨냥 회전 뒤 실제로 돈 각으로 닿을 자리를 다시 보고, 벗어나 있으면 한 번 더 겨냥한다(1)
    entry_aim_recheck=0,
    # VIO 자가 태그 자보다 뚜렷이 짧으면(vio_scale_min_gap 넘게) VIO 거리를 그 비율로 늘려 읽는다(1): 2026-10-30 정오 판 0.79~0.90
    vio_scale_online=1,
    vio_scale_min_m=1.0,
    vio_scale_min_gap=0.05,
    vio_scale_max=1.3,
)  # transition_corrections_max: correction turns at the tag-cut transition before giving up (MAX_CORRECTIONS, from KWU)


def number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


INNER_TAG_KEYS = (
    'inner_tag_ids',
    'inner_tag_size_m',
    'inner_tag_height_m',
    'inner_tag_spacing_m',
    'inner_tag_lateral_m',
    'inner_tag_from_entrance_m',
    'inner_tag_plate_width_m',
)
# 실험판 D (2026-11-19): the inner tags are no longer assumed to sit on the lattice. Their plane is given directly as the distance
# from the entrance line (inner_tag_from_entrance_m); inner_tag_plane_offset_m (plane = truck_depth_m - offset) is gone.
# The fork-tip target may lie beyond the tag plane (2026-11-19 site: the tags hang above the lane, 0.7 m inside, target 1.0 m).
# Site keys renamed 2026-11-01 (entrance tag / inner tag): older common.json files are read through this map.
LEGACY_SITE_KEYS = {
    'tag_id': 'entrance_tag_id',
    'tag_size_m': 'entrance_tag_size_m',
    'tag_height_m': 'entrance_tag_height_m',
    'left_tag_offset_m': 'entrance_tag_lateral_m',
    'tag_plane_offset_m': 'entrance_tag_plane_offset_m',
    'inner_tag_centre_offset_m': 'inner_tag_lateral_m',
}
LATTICE_HEIGHT_KEYS = ('lattice_bottom_bar_top_m', 'lattice_top_bar_bottom_m', 'lattice_bar_thickness_m')


def inner_tag_frames(site):
    """v4: the two inner AprilTags on the lattice bars -> dock-frame poses of each tag and of the INNER frame (origin = midpoint of
    the two tag centres, orientation of an upright tag facing the entrance: x = dock +y (left), y = up, z = dock +x). The lane
    centre from the FINE stage on is this midpoint plus inner_tag_lateral_m. None when inner_tag_ids is empty (the v3 behaviour).
    """
    ids = site.get('inner_tag_ids') or []
    if not ids:
        return None
    if (
        not isinstance(ids, (list, tuple))
        or len(ids) != 2
        or any(type(i) is not int or not 0 <= i < 587 for i in ids)
        or ids[0] == ids[1]
    ):
        raise ValueError('inner_tag_ids 는 36h11 ID 두 개 [왼쪽, 오른쪽] 이어야 합니다')
    if site.get('entrance_tag_id') in ids:
        raise ValueError('inner_tag_ids 는 입구 태그 entrance_tag_id 와 달라야 합니다')

    def num(k, positive=True):
        v = site.get(k)
        if not number(v) or (positive and v <= 0):
            raise ValueError('invalid ' + k)
        return float(v)

    size = num('inner_tag_size_m')
    height = num('inner_tag_height_m')
    spacing = num('inner_tag_spacing_m')
    centre = num('inner_tag_lateral_m', False)
    x = num('inner_tag_from_entrance_m')  # 실험판 D: entrance line -> tag plane, measured on site (not truck_depth_m - offset)
    plate = site.get('inner_tag_plate_width_m')
    plate = 1.3 * size if plate is None else plate
    if not number(plate) or plate < size:
        raise ValueError('invalid inner_tag_plate_width_m (태그보다 넓어야 함)')
    left, right = int(ids[0]), int(ids[1])

    def T(y):
        D = np.eye(4)
        D[:3, :3] = R_REFERENCE_TAG
        D[:3, 3] = [x, y, height]
        return D.tolist()

    return dict(
        ids=[left, right],
        size_m=size,
        spacing_m=spacing,
        height_m=height,
        plane_x_m=x,
        plate_half_width_m=float(plate) / 2,
        lateral_m=centre,
        offsets={
            left: spacing / 2,
            right: -spacing / 2,
        },  # each tag's centre along the tag x axis (= dock +y, left) from the inner origin
        T_dock_inner=T(centre),
        tags={left: T(centre + spacing / 2), right: T(centre - spacing / 2)},
    )


def apply_inner(sensor, site):
    """v4: inner_tag_frames(site) -> the detector's ids/sizes (sensor['tag']['inner']) and the dock-frame frames (sensor['calibration']['inner'])."""
    inner = inner_tag_frames(site)
    sensor['tag']['inner'] = (
        None
        if inner is None
        else dict(
            ids=list(inner['ids']),
            size_m=inner['size_m'],
            spacing_m=inner['spacing_m'],
            offsets={str(k): v for k, v in inner['offsets'].items()},
        )
    )
    sensor['calibration']['inner'] = (
        None
        if inner is None
        else dict(
            ids=list(inner['ids']),
            T_dock_inner=inner['T_dock_inner'],
            tags={str(k): v for k, v in inner['tags'].items()},
            plane_x_m=inner['plane_x_m'],
            height_m=inner['height_m'],
            size_m=inner['size_m'],
            spacing_m=inner['spacing_m'],
            lateral_m=inner['lateral_m'],
            plate_half_width_m=inner['plate_half_width_m'],
        )
    )
    return inner


def decode(node):
    if not isinstance(node, dict):
        return node
    if set(node) == {'description', 'value'}:
        return node['value']
    if set(node) == {'description', 'fields'}:
        return {k: decode(v) for k, v in node['fields'].items()}
    raise ValueError('설정은 description/value 또는 description/fields 형식이어야 합니다')


def mount_path(common_path):
    return Path(common_path).with_name('mount.json')


def read_mount(common_path, decoded):
    """configs/mount.json beside common.json (2026-10-20); falls back to the pre-split 'mount' field inside common.json."""
    p = mount_path(common_path)
    if p.exists():
        m = json.loads(p.read_text())
        return m if isinstance(m, dict) and m else None
    return decoded.get('mount') or None


def mount_signature(site):
    """Hash of the values the mount measurement depends on. Keyed by the pre-2026-11-01 names so a mount.json measured under them
    stays valid after the entrance_tag_* rename."""
    legacy = dict(
        tag_id=site['entrance_tag_id'],
        tag_size_m=site['entrance_tag_size_m'],
        tag_height_m=site['entrance_tag_height_m'],
        camera_height_m=site['camera_height_m'],
        camera_lateral_m=site['camera_lateral_m'],
    )
    return hashlib.sha256(json.dumps(legacy, sort_keys=True).encode()).hexdigest()


def load(path=COMMON, mode='left', require_mount=False):
    if mode != 'left':
        raise ValueError('docking: left mode only (the central mode was removed 2026-10-30)')
    doc = json.loads(Path(path).read_text())
    c = decode(doc)
    site = c['site']
    t = copy.deepcopy(TUNING)
    for old, new in LEGACY_SITE_KEYS.items():  # older files
        if old in site and new not in site:
            site[new] = site.pop(old)
    FLAGS = (
        'fine_weak_commands',
        'lattice_axis_enabled',
        'projector_when_stopped',
        'path_bias_online',
        'path_bias_clean_first',
        'entry_depth_ruler_with_tag',
        'bundle_watch_depth_with_tag',
        'entry_aim_pivot',
        'final_arrival_check',
        'entry_lateral_watch',
        'lattice_alias_gate_m',
        'depth_min_points',
        'final_pose_average',
        'entry_aim_recheck',
        'vio_scale_online',
    )  # 0 allowed: on/off switches
    for k, v in c.get('operation', {}).items():
        if k not in t:
            raise ValueError('invalid operation setting: ' + k)
        if v is None:
            continue  # empty = keep the code default (e.g. final_check_distance_m: computed)
        if (
            not isinstance(v, (int, float))
            or isinstance(v, bool)
            or not math.isfinite(v)
            or (v <= 0 and not (k in FLAGS and v == 0))
        ):
            raise ValueError('invalid operation setting: ' + k)
        t[k] = v
    from .canbus import weak_payloads

    weak_payloads(
        dict(forward_byte=t['weak_forward_byte'], rotate_delta=t['weak_rotate_delta'])
    )  # range check (68..126 / 1..30)
    # number(): module level since v4
    # Camera position on the vehicle (2026-10-21): longitudinal from the fork tip (camera_to_fork_tip_m), lateral from the
    # vehicle centreline (camera_lateral_m, left +), height from the floor. rear_extent_m may be left empty: it is then
    # vehicle_length_m - camera_to_fork_tip_m, so moving the camera only needs the two tape values.
    if (
        site.get('rear_extent_m') is None
        and number(site.get('vehicle_length_m'))
        and number(site.get('camera_to_fork_tip_m'))
    ):
        site['rear_extent_m'] = float(site['vehicle_length_m']) - float(site['camera_to_fork_tip_m'])
    for k in (
        'entrance_tag_size_m',
        'camera_height_m',
        'entrance_tag_height_m',
        'camera_to_fork_tip_m',
        'rear_extent_m',
    ):
        if not number(site[k]) or site[k] <= 0:
            raise ValueError('invalid ' + k)
    for k in ('camera_lateral_m', 'entrance_tag_plane_offset_m'):
        if not number(site[k]):
            raise ValueError('invalid ' + k)
    # Straight-drive drift (KWU FORWARD_PATH_BIAS_DEG). Empty = 0: no compensation until it is measured (2026-10-21).
    b = site.get('forward_path_bias_deg')
    site['forward_path_bias_deg'] = 0.0 if b is None else float(b)
    if not number(site['forward_path_bias_deg']) or abs(site['forward_path_bias_deg']) > 10:
        raise ValueError('invalid forward_path_bias_deg')
    if type(site['entrance_tag_id']) is not int or not 0 <= site['entrance_tag_id'] < 587:
        raise ValueError('invalid tag36h11 entrance_tag_id')
    # Tag side = sign (2026-10-22): left +, right -, same convention as camera_lateral_m / forward_path_bias_deg.
    if site['entrance_tag_lateral_m'] is not None and not number(site['entrance_tag_lateral_m']):
        raise ValueError('entrance_tag_lateral_m 은 숫자[m]입니다 (왼쪽 +, 오른쪽 −)')
    if mode == 'left' and site['entrance_tag_lateral_m'] is None and require_mount:
        raise ValueError('common: entrance_tag_lateral_m 미입력')
    rear = site.get('rear_space_m')
    if rear is not None:
        if not number(rear) or rear < 0:
            raise ValueError('invalid rear_space_m')
        t['backward_max_m'] = float(rear)  # 뒤 공간 = 한 번 후진 상한 (BACK_MAX_M, 현장 값)
    elif require_mount:
        raise ValueError('common: rear_space_m 미입력 (차 뒤 빈 공간)')
    if 'bay_width_m' in site:
        raise ValueError('common: bay_width_m 은 truck_inner_width_m 으로 이름이 바뀌었습니다')
    if require_mount:
        for k in ('vehicle_width_m', 'truck_inner_width_m'):
            if not number(site.get(k)) or site[k] <= 0:
                raise ValueError('common: ' + k + ' 미입력')
        if site['truck_inner_width_m'] <= site['vehicle_width_m']:
            raise ValueError('탑재부 안쪽 폭은 차폭보다 커야 합니다')
    depth = site.get('truck_depth_m')
    if depth is not None and (not number(depth) or depth <= 0):
        raise ValueError('invalid truck_depth_m')
    # docking (2026-10-30): the lattice at the back of the bay. truck_depth_m = entrance line -> lattice front; lattice_pair_width_m =
    # the facing inner faces of the members bounding openings 2 and 3; lattice_centre_offset_m = lane centre minus 2|3 centre (left +).
    lw = site.get('lattice_pair_width_m')
    if lw is not None and (not number(lw) or lw <= 0):
        raise ValueError('invalid lattice_pair_width_m')
    lo = site.get('lattice_centre_offset_m')
    if lo is not None and not number(lo):
        raise ValueError('invalid lattice_centre_offset_m')
    site.setdefault('lattice_pair_width_m', None)
    site.setdefault('lattice_centre_offset_m', 0.0)
    for k in INNER_TAG_KEYS + LATTICE_HEIGHT_KEYS:
        site.setdefault(k, None)  # v4 fields; an older common.json keeps the v3 behaviour
    sensor = json.loads((ROOT / 'configs/runtime.json').read_text())
    # v4: the lattice heights are site values; when all three are given the depth height bands follow them (the v3 numbers are
    # reproduced exactly: bar 0.405/0.765, thickness 0.04 -> 0.33~0.43 and 0.42~0.75).
    lb, lt, th = (site.get(k) for k in LATTICE_HEIGHT_KEYS)
    if all(number(v) and v > 0 for v in (lb, lt, th)):
        if not lb + th < lt:
            raise ValueError('격자 높이: 아래 가로살 윗면 + 두께 < 위 가로살 아랫면 이어야 합니다')
        sensor['depth_goal'].update(
            lattice_z_min=round(lb - th - 0.035, 3),
            lattice_z_max=round(lb + 0.025, 3),
            lattice_pair_z_min=round(lb + 0.015, 3),
            lattice_pair_z_max=round(lt - 0.015, 3),
        )
    elif any(number(v) for v in (lb, lt, th)):
        raise ValueError(
            '격자 높이 세 값(lattice_bottom_bar_top_m, lattice_top_bar_bottom_m, lattice_bar_thickness_m)은 모두 넣거나 모두 비웁니다'
        )
    gap = sensor['openvins']['max_correction_gap_s']
    if not number(gap) or not 0 < gap <= 1.0:
        raise ValueError('max_correction_gap_s must be within (0, 1] seconds')
    sensor['tag'].update(id=site['entrance_tag_id'], size_m=site['entrance_tag_size_m'])
    sensor['calibration'] = {}
    sensor['rig'] = dict(
        tag_height_m=site['entrance_tag_height_m'],
        camera_height_m=site['camera_height_m'],
        camera_to_fork_tip_m=site['camera_to_fork_tip_m'],
    )
    mount = read_mount(path, c)
    good = bool(mount and mount.get('usable') and mount.get('input_signature') == mount_signature(site))
    if require_mount and not good:
        raise ValueError('새 장착 기준 측정 필요: preflight.sh mount')
    if good:
        R = np.array(mount['R_body_camera'], float)
        if (
            R.shape != (3, 3)
            or not np.isfinite(R).all()
            or not np.allclose(R.T @ R, np.eye(3), atol=1e-5)
            or abs(np.linalg.det(R) - 1) > 1e-5
        ):
            raise ValueError('invalid mount rotation')
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [0, site['camera_lateral_m'], site['camera_height_m']]
        D = np.eye(4)
        D[:3, :3] = R_REFERENCE_TAG
        D[:3, 3] = [
            site['entrance_tag_plane_offset_m'],
            site['entrance_tag_lateral_m'] or 0,
            site['entrance_tag_height_m'],
        ]
        sensor['calibration'].update(
            T_body_camera=T.tolist(), T_dock_tag=D.tolist(), camera_model=mount['camera_model']
        )
        sensor['camera']['serial'] = mount['camera_model']['serial']
        sensor['calibration']['mount_covariance'] = mount.get('covariance')
    else:
        # Preview/measurement may use an explicitly nominal mounting frame.
        # drive/shadow require the validated common mounting result above.
        T = np.eye(4)
        T[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        T[:3, 3] = [0, site['camera_lateral_m'], site['camera_height_m']]
        D = np.eye(4)
        D[:3, :3] = R_REFERENCE_TAG
        D[:3, 3] = [
            site['entrance_tag_plane_offset_m'],
            site['entrance_tag_lateral_m'] or 0,
            site['entrance_tag_height_m'],
        ]
        sensor['calibration'].update(
            T_body_camera=T.tolist(), T_dock_tag=D.tolist(), camera_model=None, mount_covariance=None
        )
    apply_inner(
        sensor, site
    )  # v4: inner tags (lattice) -> detector ids/sizes and dock-frame frames for the fusion/controller
    u = sensor['uncertainty']
    if sensor['calibration']['mount_covariance'] is None:
        sensor['calibration']['mount_covariance'] = np.diag(
            [u['mount_position_std_m'] ** 2] * 3 + [u['mount_rotation_std_rad'] ** 2] * 3
        ).tolist()
    sensor['calibration']['mount_valid'] = good
    sensor['calibration']['T_body_ir'] = sensor['calibration']['T_body_camera']
    sensor['site'] = copy.deepcopy(site)
    # Fork-tip stop target (site.target_front_x_left_m; absent/null -> 1.0). The central mode and its target were removed 2026-10-30.
    target = site.get('target_front_x_left_m')
    if target is None:
        target = 1.0
    if not number(target):
        raise ValueError('invalid target_front_x_left_m')
    sensor['target_front_x'] = float(target)
    if depth is not None and require_mount and float(target) > float(depth) - 0.05:
        raise ValueError(
            '포크 끝 목표 %.2f m 가 탑재부 깊이 %.2f m 를 넘습니다 (target_front_x_%s_m / truck_depth_m)'
            % (float(target), float(depth), mode)
        )
    sensor['tuning'] = t
    from .uncertainty import covariance

    covariance(sensor['calibration']['mount_covariance'], 6)
    if require_mount and not sensor['tag']['enabled']:
        raise ValueError('drive/shadow requires AprilTag')
    # Rotation centre (planning kinematics only) in the body frame, whose origin is ON THE VEHICLE CENTRELINE at the camera's
    # longitudinal position: x = camera_to_fork_tip - (centre behind the fork tip), y = centre lateral from the centreline.
    # Both inputs are vehicle properties, so moving the camera (forward/back via camera_to_fork_tip_m, sideways via
    # camera_lateral_m) recomputes it with no other change (2026-10-20/21).
    behind = site.get('rotation_center_behind_fork_tip_m')
    behind = 1.005 if behind is None else float(behind)
    plat = site.get('rotation_center_lateral_m')
    plat = -0.112 if plat is None else float(plat)
    if not number(behind) or not number(plat):
        raise ValueError('invalid rotation_center_*')
    return dict(
        site=site,
        tuning=t,
        sensor=sensor,
        mode=mode,
        pivot=[float(site['camera_to_fork_tip_m']) - behind, plat],
        pivot_source='earlier calibrate (0.465 ahead of the carriage camera) expressed from the fork tip; candidate planning only',
        target_front_x=float(target),
        common_path=str(path),
    )


def save_mount(path, report, expected):
    path = Path(path)
    if path.read_bytes() != expected:
        raise ValueError('common changed during mount measurement')
    doc = json.loads(expected)
    site = decode(doc)['site']
    if not report.get('usable'):
        raise ValueError('mount measurement failed')
    report = dict(report, input_signature=mount_signature(site))
    # 2026-10-20: written beside common.json, never into it, so deploys can overwrite common.json (the Mac is its only
    # author) while the Jetson's measured mount survives (update_jetson.sh excludes configs/mount.json).
    out = mount_path(path)
    tmp = out.with_suffix('.tmp')
    tmp.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    tmp.replace(out)
    return out
