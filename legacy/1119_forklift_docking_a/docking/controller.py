"""Nonblocking mission state machine (2026-10-22). All motion requires fresh measured feedback.

Flow (user decisions 2026-10-21/22):
  ROUGH       heading straight (one fine turn), sidestep whenever |lateral| >= one real step (at most twice), then ONE
              long drive that stops rough_stop_before_cut_m before the tag cut (tag ruler + VIO; a lost tag continues on
              the VIO; no progress for no_progress_s stops the leg).
  FINE_ENTER  stand still, re-zero the heading reference from the tag here (4 m: far better than the first window at 9 m).
  FINE        at every stop a depth window (depth_goal: the doors' inner lines in the entrance slab, the lattice member pair and
              bottom bar in the lattice slab) and the choice of what to believe (docking 2026-10-30, user decisions):
                lateral  : tag while it is fresh -> lattice pair when GOOD (>= lattice_good_frames, spread <= lattice_good_spread_m,
                           range <= lattice_good_range_m) -> doors -> the last centre carried by the VIO
                distance : tag -> lattice line (same gates) -> doors -> carried
                heading  : the tag-zeroed VIO; the lattice line is recorded and taken over as the axis only when two consecutive
                           stops disagree with the VIO by more than lattice_axis_adopt_deg in the same direction
              lateral error > tolerance -> one bundle (turn alpha, drive d, turn back; fine.py), else a straight leg <=
              fine_step_max_m, until final_check_distance_m before the entrance line. Every accepted depth reading is also sent
              back to the observer as an anchor (the fused pose is pulled onto the measured structure, so a lost tag or a VIO
              reset never leaves the vehicle without a position).
  FINAL       last check at final_check_distance_m: |lateral| <= final_lateral_tol_m and |heading| <= final_heading_tol_deg -> ENTRY;
              else a fine turn, or back out (never behind the tag cut) and bundle again, fine_corrections_max in all; when the
              corrections are exhausted or there is no room the vehicle ENTERS ANYWAY on its best state (straight, then aimed at
              the arrival point) and the residual error is recorded (user 2026-10-30: never end without entering).
  ENTRY       one leg to the fork-tip target; stopped by the VIO distance or the lattice range, whichever says 'arrived' first;
              timed fallback if the VIO is lost.
  FALLBACK    nothing measured at all (no tag, no doors, no lattice, no carried centre): tag-pose blind entry on the fused pose.
  RECOVER     while the fused pose is untrusted the depth window keeps running; doors/lattice readings re-anchor the pose
              absolutely (tag -> doors -> lattice) and the run continues.
v4 (2026-11-01, inner tags): two AprilTags on the lattice bars (site inner_tag_*) are read by the colour camera all the way in.
The observer anchors the fused pose on them (fusion kinds 'inner' = one tag bearing/range, 'bundle' = both tags solved together =
strong heading within inner_bundle_max_range_m), so at every stop the reference order becomes
  lateral/distance : fresh tag window (bundle 'inner_tags' > one inner tag 'inner_tag' > entrance 'tag') -> lattice pair -> doors -> carried
  heading          : the fused heading (bundle-zeroed when the bundle is the landmark, else the tag-zeroed VIO); the lattice-line axis
                     rule is off once a bundle window has anchored the run (the tags define the lane axis)
  entry            : a third ruler, the bundle's range to the tag plane (tag_remaining_m), stops the leg like the lattice ruler; after the
                     stop the bundle frames of the settle window give the TAG-MEASURED final pose (final_report.tag_measured)
  path bias        : the drift of every straight leg driven with a fresh tag reference is measured (path_bias_sample) and, after
                     path_bias_min_legs legs, adopted as forward_path_bias_deg when the site value is empty (path_bias_adopted)
The six-step beam planner (planner.plan) is no longer called; planner.Responses only tunes the braking reserve.
2026-10-29 BirdsEyeView_v4: in the depth-only stage (FINE/FINAL/ENTRY/FALLBACK) every fine turn, bundle drive and back-out is
sent with the WEAK CAN bytes (forward_fine/backward_fine/rotate_*_fine2, bytes from common.json operation); a weak command
that does not move the vehicle within weak_deadband_s falls back to the normal byte for that leg and the rest of the run.
2026-10-29 BirdsEyeView_v3: ONE heading reference: straighten() and FINAL turn to the same target (an adopted axis or the dock
axis), so they never alternate. docking keeps that and feeds the axis from the lattice line instead of the side-wall fits.
"""

import math
import numpy as np
from .geometry import wrap, remaining, arrival_y, inside_truck, path_clear, front, finite, moved, path_bias
from .planner import Responses
from .canbus import WEAK_NAMES, STRONG_OF
from .fine import plan_bundle, entrance_in_dock, lateral_error, final_check_distance, aim_for, tip_arrival_y

PHASES = ('ROUGH', 'FINE_ENTER', 'FINE', 'FINAL', 'ENTRY', 'FALLBACK')


class Controller:
    def __init__(self, cfg, cut, now):
        self.cfg = cfg
        self.t = cfg['tuning']
        self.cut = cut
        self.created = now
        self.state = 'PREPARING'
        self.reason = 'stationary_preparation'
        self.command = 'stop'
        self.mission_start = None
        self.stable_start = None
        self.last_capture = None
        self.prep_seconds = 0.0
        self.prep_last = None
        self.prep_anchor = None
        self.prep_pause = None
        self.responses = Responses()
        self.request = None  # request stays None: no planner process (runtime checks it)
        self.action = None
        self.brake = None
        self.blind = False
        self.search_start = None
        self.sweep = 0.0
        self.motion_start = None
        self.stop_at = None
        self.recovery_at = None
        self.post_settle = 'PLAN'
        self.events = []
        self.initialized_at = None
        self.preparation_report = {}
        self.last_seen = None
        self.final_report = None  # entry outcome: what ended the leg and the measured error (record only)
        self.last_lateral_check = None  # the last |arrival| <= tolerance decision with its σ (record only, 2026-10-19)
        self.baseline_done = False
        self.baseline_seconds = 0.0
        self.baseline_last = None  # 30 s at rest with the tag in view, after Enter and after any search
        self.baseline_lost_since = None
        self.overshoot_backed = False
        # ---- flow state ----
        self.phase = 'ROUGH'
        self.legs = []
        self.route_points = []
        self.route = []
        self.sidesteps = 0
        self.heading_turns = 0
        self.no_progress_stops = 0
        self.depth_collect_since = None
        self.depth_last_stamp = None
        self.depth_frames = 0
        self.samples = dict(
            doors=[], lattice=[], line=[]
        )  # per-frame readings of the current window (dock frame), docking 2026-10-30
        self.depth_seen = False
        self.depth_centre_dock = None
        self.depth_measure = None
        self.lattice_point_dock = (
            None  # a point on the lattice plane (dock frame) from the last good line, carried by the VIO
        )
        self.lattice_history = []
        self.lattice_rejected = []  # 이 정지 창에서 옆 칸으로 보고 버린 격자 쌍의 좌우 거리
        self.depth_low = []  # 이 정지 창에서 점이 적어 버린 depth 장면의 점 수
        self.bundle_step_stamp = None  # 이미 기록한 묶음 방향 맞춤의 시각
        self.structure_window = []  # 이 정지 동안 구조물(적재함 모서리가 모이는 점)로 잰 방향 — 기록용
        self.structure_stamp = None
        self.pose_window = []  # 마지막 확인 창에서 모은 태그 기준 자세
        self.pose_window_capture = None
        self.vio_scale = 1.0  # VIO 가 잰 직진 거리에 곱하는 배율(태그 자와 견줘 VIO 가 짧을 때만 1 보다 큼)
        self.vio_scale_samples = []
        self.lattice_history_stamp = (
            None  # docking_v3: (t, x, y) entrance-plane lane centre per stop from the 2|3 pair, dock frame
        )
        self.lattice_depth = None if cfg['site'].get('truck_depth_m') is None else float(cfg['site']['truck_depth_m'])
        self.lattice_offset = float(cfg['site'].get('lattice_centre_offset_m') or 0.0)
        self.axis_diffs = []  # lattice-line axis minus the dock axis at consecutive stops (deg), for the adoption rule
        self.anchor_request = None
        self.anchor_id = 0
        self.anchor_applied = 0  # depth re-anchoring of the fused pose (observer applies)
        self.pending_entry = None  # best-effort entry queued behind the last turn (corrections exhausted / no room)
        self.recover_samples = []
        self.rezero_request = 0
        self.rezero_since = None
        self.rezero_report = None
        self.corrections = 0
        self.fallback_corrections = 0
        self.fine_bundles = 0
        self.weak_dead = {}  # 2026-10-29 v4: weak command kinds ('drive'/'rotate') found in the dead band this run
        self.ref_axis = None
        self.ref_axis_report = None
        self.axis_source = None  # truck axis direction (dock frame) adopted from the lattice line
        # ---- v4: inner tags ----
        inner = (cfg.get('sensor') or {}).get('calibration', {}).get('inner') or {}
        self.inner_plane_x = (
            None if not inner else float(inner['plane_x_m'])
        )  # dock x of the tag plane (the entry's tag ruler)
        self.inner_seen = False  # an inner-tag window has anchored the pose in this run: the tags define the lane axis
        self.bias_samples = []
        self.bias_clean = []  # 표본마다 직진하는 동안 기준 태그가 그대로였는지
        self.bias_adopted = None
        self.bias_configured = bool(cfg['site'].get('forward_path_bias_deg'))  # online straight-drive drift
        self.final_tag_samples = []  # bundle frames while settling after the entry -> final_report.tag_measured

    # ---------------------------------------------------------------- reporting
    def fault(self, reason):
        self.state = 'FAULT'
        self.reason = reason
        self.command = 'stop'
        self.request = None
        return self.status()

    def status(self):
        return dict(
            state=self.state,
            phase=self.phase,
            reason=self.reason,
            command=self.command,
            blind=self.blind,
            final_report=self.final_report,
            search_active=bool(self.action and self.action.get('after') == 'SEARCH'),
            lateral_check=self.last_lateral_check,
            baseline_done=self.baseline_done,
            baseline_seconds=self.baseline_seconds,
            events=list(self.events),
            search_sweep_deg=math.degrees(self.sweep),
            route=[dict(kind=l['kind'], value=float(l['value']), label=l.get('label')) for l in self.legs],
            route_points=[list(p) for p in self.route_points],
            preparation=dict(self.preparation_report),
            responses={k: dict(v) for k, v in self.responses.data.items()},
            depth=self.depth_measure,
            depth_centre_dock=None if self.depth_centre_dock is None else list(self.depth_centre_dock),
            rezero_request=self.rezero_request,
            rezero=self.rezero_report,
            corrections=self.corrections,
            sidesteps=self.sidesteps,
            weak_dead=dict(self.weak_dead),
            ref_axis_deg=None if self.ref_axis is None else float(math.degrees(self.ref_axis)),
            ref_axis=self.ref_axis_report,
            axis_source=self.axis_source,
            lattice_point_dock=None if self.lattice_point_dock is None else list(self.lattice_point_dock),
            anchor_request=self.anchor_request,
            anchor_applied=self.anchor_applied,
            heading_target_deg=math.degrees(self.heading_target()),
            inner_seen=self.inner_seen,
            path_bias=dict(
                samples_deg=[math.degrees(v) for v in self.bias_samples],
                adopted_deg=self.bias_adopted,
                configured=self.bias_configured,
                applied_deg=float(self.cfg['site'].get('forward_path_bias_deg') or 0.0),
            ),
            estimate_use='response estimates affect braking only, never measured pose',
        )

    def fresh(self, now, o, age=None):
        # age: sensor_age_s (0.3 s) for decisions at rest; motion_age_s (3 s) while moving.
        age = self.t['sensor_age_s'] if age is None else age
        return bool(
            o
            and not o.get('clock_fault')
            and o.get('sensor_ready')
            and not o.get('imu_bad')
            and finite(o.get('local_pose', []))
            and len(o.get('local_pose', [])) == 3
            and (not o.get('usable') or (o.get('pose') is not None and finite(o['pose'])))
            and 0 <= now - o.get('arrival', -math.inf) <= age
            and o.get('transport_lag_s', 0) <= age
            and o.get('local_valid')
        )

    def heading_target(self):
        """Heading at which a straight drive actually tracks the truck axis: minus the measured drift (forward_path_bias_deg).
        Once the lattice axis has been adopted the axis is that direction in the dock frame, not the dock x axis."""
        return wrap(self.axis() - path_bias(self.cfg))

    def axis(self):
        return 0.0 if self.ref_axis is None else self.ref_axis

    def to_axis(self, p, centre):
        """Pose relative to the truck centreline through the entrance centre: (along the axis past the entrance plane, left of the
        centreline, heading minus the axis). Without an adopted axis this is the dock frame shifted to the centre."""
        th = self.axis()
        c, s = math.cos(th), math.sin(th)
        dx = float(p[0]) - float(centre[0])
        dy = float(p[1]) - float(centre[1])
        return c * dx + s * dy, -s * dx + c * dy, wrap(float(p[2]) - th)

    def accept_plan(self, reply, now, o):
        return  # kept for the runtime contract; no planner requests are issued

    # ---------------------------------------------------------------- motion primitives
    def set_legs(self, now, o, legs, label):
        """Queue legs (dict kind='turn' value=absolute heading | kind='drive' value=signed metres) and preview them for the map."""
        self.legs = list(legs)
        p = np.array(o['pose'], float)
        pts = [p.tolist()]
        q = p
        for l in self.legs:
            q = (
                moved(q, wrap(l['value'] - q[2]), 0.0, self.cfg['pivot'])
                if l['kind'] == 'turn'
                else moved(q, 0.0, l['value'], self.cfg['pivot'])
            )
            pts.append(q.tolist())
        self.route_points = pts
        self.events.append(
            dict(
                t=now,
                event='legs',
                label=label,
                legs=[dict(kind=l['kind'], value=float(l['value']), label=l.get('label')) for l in self.legs],
                pose=[float(v) for v in p],
            )
        )

    def weak_stage(self):
        """2026-10-29 v4: after the rough stop the vehicle is positioned on depth/VIO in small steps; those use the weak bytes."""
        return bool(self.t.get('fine_weak_commands')) and self.phase in ('FINE', 'FINAL', 'ENTRY', 'FALLBACK')

    WEAK_DRIVE_LABELS = (
        'bundle_drive',
        'final_back',
        'past_target_back',
    )  # short corrective drives; fine_straight/entry keep the normal byte

    def next_leg(self, now, o):
        """Execute the next queued leg. Tag visibility is not required (the fused pose is propagated by the VIO)."""
        p = o.get('pose')
        while self.legs:
            leg = self.legs.pop(0)
            if p is None:
                break
            if leg['kind'] == 'turn':
                start = leg.get('from_heading')  # 서서 여러 창을 모아 잰 방향이 있으면 그것에서 돌 각을 정한다
                turn = wrap(leg['value'] - (p[2] if start is None else start))
                if abs(turn) < math.radians(self.t['minimum_turn_fine_deg']) / 2:
                    continue  # fine turns are accepted from half the floor
                strength = leg.get('strength') or (
                    'fine' if abs(turn) <= math.radians(self.t['turn_max_deg']) + 1e-9 else 'slow'
                )
                if strength == 'fine' and self.weak_stage() and not self.weak_dead.get('rotate'):
                    strength = 'fine2'
                self.begin(
                    now,
                    o,
                    'rotate_' + ('left' if turn > 0 else 'right') + '_' + strength,
                    abs(turn),
                    'LEGS',
                    label=leg.get('label'),
                )
                return
            d = float(leg['value'])
            if abs(d) < self.t['minimum_translation_m']:
                continue
            if not path_clear(p, 0, d, self.cfg):
                self.fault('truck_side_clearance')
                return
            cmd = 'backward' if d < 0 else 'forward'
            if leg.get('label') in self.WEAK_DRIVE_LABELS and self.weak_stage() and not self.weak_dead.get('drive'):
                cmd += '_fine'
            self.begin(now, o, cmd, abs(d), 'LEGS', label=leg.get('label'))
            self.action.update(stop_x=leg.get('stop_x'), watch=bool(leg.get('watch')), watch_sign=leg.get('watch_sign'))
            return
        self.legs = []
        self.state = 'PLAN'
        self.reason = 'legs_done'

    def begin(self, now, o, cmd, target, after, label=None):
        self.flush_structure(now, o)  # 정지 판단 뒤에 더 들어온 구조물 방향도 움직이기 전에 적어 둔다
        r = self.responses.get(cmd)
        expected = r['startup'] + target / max(1e-6, r['speed'])
        turn = 'rotate' in cmd
        default = (
            self.t['search_turn_timeout_s']
            if after == 'SEARCH'
            else self.t['rotation_timeout_s'] if turn else self.t['translation_timeout_s']
        )
        self.action = dict(
            command=cmd,
            target=target,
            start_local=o['local_pose'][:],
            generation=o['tracking_generation'],
            epoch=o['motion_epoch'],
            started=now,
            after=after,
            speed=0.0,
            progress=0.0,
            previous=0.0,
            last=now,
            blind=False,
            label=label,
            # 2026-10-20 KW: the tag-anchored dock-frame pose is a second ruler for straight steps (the VIO alone froze twice).
            start_world=list(o['pose']) if o.get('usable') and o.get('pose') is not None else None,
            progress_parts=None,
            # time cap scaled to the leg (a 5 m rough leg takes ~20 s): never the reason a healthy leg ends.
            cap_s=max(default, self.t['entry_timed_scale'] * expected + 3.0),
        )
        self.state = 'MOVING'
        self.command = cmd
        self.reason = 'measured_motion_feedback'
        self.motion_start = None
        self.depth_collect_since = None
        self.samples = dict(doors=[], lattice=[], line=[])
        self.depth_frames = 0  # a measurement window belongs to one stop
        if (
            cmd in ('forward', 'forward_fine') and self.tag_fresh(o) and o.get('usable') and o.get('pose') is not None
        ):  # v4: path bias is measured over tag-anchored straight legs
            self.action['bias_start'] = ([float(v) for v in o['pose']], now)
        if 'rotate' not in cmd and self.tag_fresh(o) and o.get('usable'):
            # VIO 자와 태그 자를 견줄 직진: 시작 때의 기준 태그를 적어 둔다
            self.action['scale_landmark'] = (o.get('landmark'), o.get('landmark_switches'))

    def begin_entry(self, now, o, distance, source):
        """ENTRY (2026-10-19 user decision kept): ONE leg for the whole remaining distance, stopped on OpenVINS progress.
        If the VIO is lost on the way the leg continues on the time model instead of stopping inside the truck."""
        target = max(0.0, float(distance))
        if target < 0.005:
            self.state = 'COMPLETE'
            self.reason = 'entry_distance_zero_recorded'
            return
        if not path_clear(o['pose'], 0, target, self.cfg):
            self.fault('truck_side_clearance')
            return
        r = self.responses.get('forward')
        self.begin(now, o, 'forward', target, 'COMPLETE', label='entry')
        self.action.update(blind=True, timed=False, hold_s=r['startup'] + target / r['speed'], lattice_remaining_m=None)
        self.blind = True
        self.phase = 'ENTRY'
        self.reason = 'entry_vio_progress'
        self.events.append(
            dict(
                t=now,
                event='entry',
                distance_m=target,
                source=source,
                pose=[float(v) for v in o['pose']],
                heading_deg=float(math.degrees(o['pose'][2])),
                lateral_error_m=None if self.depth_measure is None else self.depth_measure.get('lateral_error_m'),
            )
        )

    def sample_vio_scale(self, now, ratio, label):
        """직진 한 번의 태그 자 ÷ VIO 자를 모아, VIO 가 뚜렷이 짧을 때만 VIO 거리를 그만큼 늘려 읽는다(더 들어가지 않는 쪽)."""
        used = 0.8 <= ratio <= 1.4  # 2026-10-30 판들의 범위(0.96~1.27) 밖은 태그 쪽 문제로 보고 버린다
        if used:
            self.vio_scale_samples.append(ratio)
        med = float(np.median(self.vio_scale_samples)) if self.vio_scale_samples else 1.0
        new = min(med, self.t['vio_scale_max']) if med >= 1.0 + self.t['vio_scale_min_gap'] else 1.0
        self.events.append(
            dict(
                t=now, event='vio_scale_sample', ratio=float(ratio), used=bool(used), label=label, median=med, scale=new
            )
        )
        if abs(new - self.vio_scale) > 0.01:
            self.events.append(
                dict(
                    t=now,
                    event='vio_scale_adopted',
                    scale=new,
                    previous=self.vio_scale,
                    samples=len(self.vio_scale_samples),
                )
            )
            self.vio_scale = new

    def progress(self, o):
        a = self.action
        p = np.array(o['local_pose'])
        start = np.array(a['start_local'])
        if 'rotate' in a['command']:
            return wrap(p[2] - start[2]) * (1 if 'left' in a['command'] else -1)
        delta = p[:2] - start[:2]
        signed = float(delta @ [math.cos(start[2]), math.sin(start[2])])
        raw = -signed if a['command'].startswith('backward') else signed
        local = raw * self.vio_scale
        world = None
        if a.get('start_world') is not None and o.get('usable') and o.get('pose') is not None and finite(o['pose']):
            w = np.array(o['pose'])
            s0 = np.array(a['start_world'])
            dw = w[:2] - s0[:2]
            sw = float(dw @ [math.cos(s0[2]), math.sin(s0[2])])
            world = -sw if a['command'].startswith('backward') else sw
        a['progress_parts'] = (local, world)
        if world is not None and raw >= self.t['vio_scale_min_m'] and a.get('scale_landmark') is not None:
            if self.tag_fresh(o) and (o.get('landmark'), o.get('landmark_switches')) == a['scale_landmark']:
                a['scale_ratio'] = world / raw  # 같은 태그가 신선한 동안의 태그 자 ÷ VIO 자
        # Whichever ruler says the step is done, stop (early rather than late).
        return local if world is None else max(local, world)

    def step_event(self, now, b, end, vio_ok):
        """One line per executed step (2026-10-20): what was commanded, what the stop decision saw, what happened after."""
        turn = 'rotate' in b['command']
        learned = self.responses.get(b['command'])
        self.events.append(
            dict(
                t=now,
                event='step_done',
                command=b['command'],
                after=b.get('after'),
                label=b.get('label'),
                unit='rad' if turn else 'm',
                target=b.get('target'),
                stop_progress=b.get('stop_progress'),
                end_progress=end,
                coast=None if end is None or b.get('stop_progress') is None else end - b['stop_progress'],
                active_s=b['stop_time'] - b['started'],
                dead_s=(b['motion_start'] - b['started']) if b.get('motion_start') else None,
                settle_s=now - b['stop_time'],
                stop_reserve=b.get('stop_reserve'),
                stop_remaining_m=b.get('stop_remaining_m'),
                stop_obs_age_s=b.get('stop_obs_age_s'),
                stop_coast_est=b.get('stop_coast_est'),
                speed_est=b.get('speed'),
                progress_parts_vio_tag=b.get('progress_parts'),
                scale_ratio=b.get('scale_ratio'),
                stop_cause=b.get('stop_cause'),
                timed=bool(b.get('timed')),
                vio_ok=bool(vio_ok),
                learned=dict(
                    speed=learned['speed'], coast=learned['coast'], startup=learned['startup'], n=learned['n']
                ),
            )
        )
        if not turn and self.t.get('vio_scale_online') and b.get('scale_ratio') is not None:
            self.sample_vio_scale(now, float(b['scale_ratio']), b.get('label'))

    def stop(self, now, post='PLAN', cause='reached'):
        self.command = 'stop'
        self.state = 'SETTLE'
        self.stop_at = now
        self.post_settle = post
        if self.action:
            self.brake = dict(
                self.action,
                stop_progress=self.action['progress'],
                stop_time=now,
                motion_start=self.motion_start,
                stop_cause=cause,
            )
        self.reason = 'observing_coast_and_rebound'

    def recover(self, now, reason):
        if self.action and self.action['after'] == 'SEARCH':
            self.fault('search_tracking_lost_unmeasured_angle')
            return
        self.command = 'stop'
        self.state = 'RECOVER'
        self.recovery_at = now
        self.reason = reason
        self.action = None
        self.brake = None
        self.request = None
        # queued legs are dropped: the next PLAN re-measures and decides again from the actual pose
        self.legs = []
        self.route_points = []

    def lateral_within_tolerance(self, o, arrival=False):
        """2026-10-19 rule: |predicted arrival| <= 3 cm; σ is recorded, not a gate."""
        report = o.get('fork_uncertainty') or {}
        if arrival:
            mean = report.get('predicted_arrival_y_m')
            std = report.get('predicted_arrival_std_m')
        else:
            mean = (report.get('position') or [None, None])[1]
            std = (report.get('std_m') or [None, None])[1]
        if mean is None or not math.isfinite(mean):
            return False
        space = (self.cfg['site']['truck_inner_width_m'] - self.cfg['site']['vehicle_width_m']) / 2
        self.last_lateral_check = dict(
            arrival=arrival,
            mean_m=mean,
            std_m=std,
            tolerance_m=min(space, self.t['position_tolerance_m']),
            sigma=o.get('sigma'),
            certified_error_bound=False,
        )
        return abs(mean) <= min(space, self.t['position_tolerance_m'])

    # ---------------------------------------------------------------- depth window at a stop (docking 2026-10-30)
    LANDMARK_SOURCE = {
        'entrance': 'tag',
        'inner': 'inner_tag',
        'bundle': 'inner_tags',
    }  # v4: what the events call each fresh tag reference

    def flush_structure(self, now, o):
        """이 정지 동안 구조물로 잰 방향을 한 줄로 적는다: 융합 방향과의 차이와 방향 보정에 반영된 횟수(판단에는 쓰지 않는다)."""
        rows, self.structure_window = self.structure_window, []
        if not rows:
            return
        good = [r for r in rows if r.get('ok') and r.get('minus_fused_deg') is not None]
        self.events.append(
            dict(
                t=now,
                event='structure_heading',
                frames=len(rows),
                ok_frames=len(good),
                heading_deg=float(np.median([r['heading_deg'] for r in good])) if good else None,
                minus_fused_deg=float(np.median([r['minus_fused_deg'] for r in good])) if good else None,
                applied=sum(1 for r in good if r.get('applied')),
                landmark=(o or {}).get('landmark'),
                reason=rows[-1].get('reason'),
            )
        )

    def tag_fresh(self, o):
        """The tag reference is current: a tag window within tag_fresh_s (the fused pose is then the tag's, not a VIO carry).
        v4: returns the landmark kind ('entrance' | 'inner' | 'bundle') or None."""
        age = o.get('blind_age_s')
        if age is None or age > self.t['tag_fresh_s']:
            return None
        return o.get('landmark') or 'entrance'

    def collect_depth(self, o, p):
        """One depth frame into the current window: dock-frame readings of the doors (entrance centre), the lattice pair (lane
        centre at the lattice, the 2|3 centre plus lattice_centre_offset_m) and the lattice line (a plane point and the axis).
        """
        d = o.get('depth') or {}
        if d.get('stamp') is None or d['stamp'] == self.depth_last_stamp:
            return False
        self.depth_last_stamp = d['stamp']
        if self.t.get('projector_when_stopped') and 'emitter_on' in o and not o.get('emitter_on'):
            return False  # window only with the projector on
        if not self.depth_ok(d):
            self.depth_low.append(int(d['points']))
            return False
        self.depth_frames += 1
        maxr = self.t['depth_goal_max_range_m']
        doors = d.get('doors') or {}
        if (
            doors.get('ok')
            and finite((doors.get('x_body_m'), doors.get('lateral_body_m')))
            and 0 < doors['x_body_m'] <= maxr
        ):
            cx, cy = entrance_in_dock(p, doors)
            self.samples['doors'].append(
                (cx, cy, float(doors.get('width_m') or 0.0), doors.get('pair_heading_deg'), float(doors['x_body_m']))
            )
        lp = d.get('lattice_pair') or {}
        if lp.get('ok') and finite((lp.get('x_body_m'), lp.get('lateral_body_m'))) and 0 < lp['x_body_m'] <= maxr:
            cx, cy = entrance_in_dock(
                p, dict(x_body_m=lp['x_body_m'], lateral_body_m=lp['lateral_body_m'] + self.lattice_offset)
            )
            off = self.lattice_alias(o, (cx, cy))
            if off is None:
                self.samples['lattice'].append((cx, cy, float(lp.get('width_m') or 0.0), float(lp['x_body_m'])))
            else:
                self.lattice_rejected.append(off)
        ln = d.get('lattice_line') or {}
        if (
            ln.get('ok')
            and finite((ln.get('x_body_m'), ln.get('heading_err_deg')))
            and 0 < ln['x_body_m'] <= maxr
            and abs(float(ln['heading_err_deg'])) <= self.t['lattice_max_heading_deg']
        ):
            fx, fy = entrance_in_dock(p, dict(x_body_m=ln['x_body_m'], lateral_body_m=0.0))
            self.samples['line'].append(
                (fx, fy, -float(ln['heading_err_deg']), float(ln['x_body_m']), float(ln.get('residual_m') or 0.0))
            )
        return True

    def aggregate(self, p):
        """Medians of the window per reference, with the gates that make the lattice readings 'good'."""
        t = self.t
        out = dict(doors=None, lattice=None, line=None)
        d = self.samples['doors']
        if len(d) >= t['depth_measure_min_frames']:
            a = np.array([(s[0], s[1], s[2], s[4]) for s in d], float)
            ph = [s[3] for s in d if s[3] is not None]
            out['doors'] = dict(
                centre=(float(np.median(a[:, 0])), float(np.median(a[:, 1]))),
                width_m=float(np.median(a[:, 2])),
                range_m=float(np.median(a[:, 3])),
                spread_m=float(np.std(a[:, 1])),
                frames=len(d),
                pair_heading_deg=None if not ph else float(np.median(ph)),
            )
        l = self.samples['lattice']
        if len(l) >= t['lattice_good_frames']:
            a = np.array(l, float)
            spread = float(np.std(a[:, 1]))
            rng = float(np.median(a[:, 3]))
            out['lattice'] = dict(
                centre=(float(np.median(a[:, 0])), float(np.median(a[:, 1]))),
                width_m=float(np.median(a[:, 2])),
                range_m=rng,
                spread_m=spread,
                frames=len(l),
                good=bool(spread <= t['lattice_good_spread_m'] and rng <= t['lattice_good_range_m']),
            )
        n = self.samples['line']
        if len(n) >= t['depth_measure_min_frames']:
            a = np.array(n, float)
            spread = float(np.std(a[:, 2]))
            rng = float(np.median(a[:, 3]))
            out['line'] = dict(
                point=(float(np.median(a[:, 0])), float(np.median(a[:, 1]))),
                axis_body_deg=float(np.median(a[:, 2])),
                range_m=rng,
                spread_deg=spread,
                frames=len(n),
                residual_m=float(np.median(a[:, 4])),
                good=bool(spread <= t['lattice_axis_spread_deg'] and rng <= t['lattice_good_range_m']),
            )
        return out

    def shift_history(self, req):
        """docking_v3: an applied anchor moved the dock frame mapping; the stored lattice centres (dock frame) move with it, so the
        running median stays one physical point. An absolute re-anchor (VIO reset) invalidates them."""
        if not req:
            return
        if req.get('absolute'):
            self.lattice_history = []
            return
        dx, dy = float(req.get('dx') or 0.0), float(req.get('dy') or 0.0)
        self.lattice_history = [(t_, x + dx, y + dy) for t_, x, y in self.lattice_history]

    def depth_ok(self, d):
        """높이 띠 안의 점이 depth_min_points 이상인 장면만 쓴다(점이 적은 장면에서 드물게 나오는 검출은 틀린 값이었다)."""
        lim = self.t.get('depth_min_points')
        n = d.get('points')
        return not lim or n is None or n >= lim

    def lattice_alias(self, o, centre):
        """격자 쌍이 가리키는 차로 중심이 믿는 위치의 중심선에서 반 칸 넘게 떨어졌으면(옆 칸) 그 좌우 거리, 아니면 None."""
        gate = self.t.get('lattice_alias_gate_m')
        if not gate or not o.get('usable'):
            return None
        off = float(self.to_axis((centre[0], centre[1], 0.0), (0.0, 0.0))[1])
        return off if abs(off) > gate else None

    def entrance_point_from_lattice(self, point):
        """The entrance plane is lattice_depth before the lattice plane along the axis: a point on it from a lattice plane point."""
        th = self.axis()
        D = self.lattice_depth or 0.0
        return (point[0] - D * math.cos(th), point[1] - D * math.sin(th))

    def depth_measurement(self, now, o, p):
        """Collect per-frame depth readings while standing still; once the window is full choose, per quantity, the most accurate
        reference available (see the module docstring) and return the decision input. None = keep collecting (up to depth_wait_s).
        source: 'tag' | 'depth' (doors or lattice) | 'vio_propagated' (carried centre) | 'none'."""
        if self.depth_collect_since is None:
            self.depth_collect_since = now
            self.samples = dict(doors=[], lattice=[], line=[])
            self.depth_frames = 0
            self.lattice_rejected = []
            self.depth_low = []
            self.pose_window = []
        self.collect_depth(o, p)
        t = self.t
        elapsed = now - self.depth_collect_since
        counts = {k: len(v) for k, v in self.samples.items()}
        full = any(n >= t['depth_measure_min_frames'] for n in counts.values())
        tag = self.tag_fresh(o)
        if tag and o.get('usable') and o.get('capture_id') != self.pose_window_capture:
            self.pose_window_capture = o.get('capture_id')
            self.pose_window.append([float(v) for v in p])
        if not (full or tag) and elapsed < t['depth_wait_s']:
            return None
        near_final = (
            self.phase == 'FINAL' or float(-p[0]) <= final_check_distance(self.cfg) + t['minimum_translation_m']
        )
        if (full or tag) and elapsed < (t['final_measure_s'] if near_final else t['depth_measure_s']):
            return None  # the final check averages a longer window (run 102526: the 2|3 centre scattered 2-5 cm between 0.6 s windows)
        averaged = None
        if tag and near_final and t.get('final_pose_average') and len(self.pose_window) >= 3:
            # 마지막 확인: 서 있는 동안 모은 태그 기준 자세의 가운데 값을 쓴다(태그 창 하나의 흔들림을 줄임)
            w = np.array(self.pose_window, float)
            p = np.array([np.median(w[:, 0]), np.median(w[:, 1]), wrap(float(np.median(np.unwrap(w[:, 2]))))])
            averaged = len(self.pose_window)
        agg = self.aggregate(p)
        if self.depth_low:
            # 높이 띠의 점이 너무 적은 장면(햇빛에 IR 이 죽음)은 이 창에서 쓰지 않았다
            self.events.append(
                dict(
                    t=now,
                    event='depth_low_points',
                    frames=len(self.depth_low),
                    points=int(np.median(self.depth_low)),
                    used_frames=self.depth_frames,
                )
            )
            self.depth_low = []
        if self.lattice_rejected:
            # 믿는 위치에서 반 칸 넘게 떨어진 격자 쌍(옆 칸)은 이 창에서 쓰지 않았다
            self.events.append(
                dict(
                    t=now,
                    event='lattice_pair_rejected',
                    frames=len(self.lattice_rejected),
                    offset_m=float(np.median(self.lattice_rejected)),
                    kept=counts['lattice'],
                )
            )
            self.lattice_rejected = []
        self.flush_structure(now, o)
        doors = agg['doors']
        lat = agg['lattice']
        line = agg['line']
        # ---- lateral: tag -> lattice (good) -> doors -> carried
        lat_centre = None
        if lat and lat['good']:
            # the 2|3 centre moved truck_depth_m forward onto the entrance plane (run 082754 2026-10-30: used raw, every distance came
            # out 2.4 m long); docking_v3: then the median over the last lattice_history_stops stops (one entry per window)
            ep = self.entrance_point_from_lattice(lat['centre'])
            if self.lattice_history_stamp != self.depth_collect_since:
                self.lattice_history_stamp = self.depth_collect_since
                self.lattice_history.append((now, float(ep[0]), float(ep[1])))
                self.lattice_history = self.lattice_history[-max(1, int(t['lattice_history_stops'])) :]
            lat_centre = (
                float(np.median([h[1] for h in self.lattice_history])),
                float(np.median([h[2] for h in self.lattice_history])),
            )
        if tag:
            centre = (0.0, 0.0)
            lat_src = self.LANDMARK_SOURCE.get(tag, 'tag')
        elif lat and lat['good']:
            centre = lat_centre
            lat_src = 'lattice'
        elif doors:
            centre = doors['centre']
            lat_src = 'doors'
        elif self.depth_centre_dock is not None:
            centre = self.depth_centre_dock
            lat_src = 'carried'
        else:
            centre = None
            lat_src = None
        # ---- distance to the entrance line: tag -> lattice line (good) -> lattice pair (good; its centre is truck_depth_m behind the
        #      entrance line) -> doors -> carried lattice point -> carried centre (always an entrance-plane point)
        if tag:
            dist = float(-p[0])
            dist_src = lat_src
        elif line and line['good']:
            dist = float(-self.to_axis(p, self.entrance_point_from_lattice(line['point']))[0])
            dist_src = 'lattice'
        elif lat and lat['good']:
            dist = float(-self.to_axis(p, centre)[0])
            dist_src = 'lattice_pair'
        elif doors:
            dist = float(-self.to_axis(p, doors['centre'])[0])
            dist_src = 'doors'
        elif self.lattice_point_dock is not None:
            dist = float(-self.to_axis(p, self.entrance_point_from_lattice(self.lattice_point_dock))[0])
            dist_src = 'carried_lattice'
        elif centre is not None:
            dist = float(-self.to_axis(p, centre)[0])
            dist_src = 'carried'
        else:
            dist = None
            dist_src = None
        if centre is None and dist is not None:
            centre = (0.0, 0.0)
            lat_src = 'pose'  # only the lattice line: lateral = the fused pose vs the dock centreline
        # ---- heading: the lattice line is recorded; adopted only on two consecutive stops disagreeing the same way
        if line and line['spread_deg'] <= t['lattice_axis_spread_deg'] and not self.inner_seen:
            self.consider_axis(now, p, line)  # v4: the tags define the axis once seen
        else:
            self.axis_diffs = []
        measured = (
            lat_centre if (lat and lat['good']) else (doors['centre'] if doors else None)
        )  # the best measured centre ON THE ENTRANCE PLANE, also while the tag is the reference
        if measured is not None:
            self.depth_seen = True
            self.depth_centre_dock = measured
        if line and line['good']:
            self.lattice_point_dock = line['point']
        if centre is None:
            m = dict(source='none', frames=self.depth_frames, counts=counts, t=now)
            self.depth_measure = m
            return m
        along, e, _ = self.to_axis(p, centre)
        source = (
            'tag'
            if tag
            else (
                'depth'
                if (lat_src in ('lattice', 'doors') or dist_src in ('lattice', 'lattice_pair', 'doors'))
                else 'vio_propagated'
            )
        )
        m = dict(
            source=source,
            centre_dock=list(centre),
            lateral_error_m=float(e),
            distance_m=float(dist),
            lateral_source=lat_src,
            distance_source=dist_src,
            frames=self.depth_frames,
            counts=counts,
            tag_fresh=tag,
            doors=(
                None
                if not doors
                else dict(
                    doors,
                    centre=list(doors['centre']),
                    lateral_m=float(self.to_axis(p, doors['centre'])[1]),
                    dist_m=float(-self.to_axis(p, doors['centre'])[0]),
                )
            ),
            lattice=(
                None
                if not lat
                else dict(
                    lat,
                    centre=list(lat['centre']),
                    entrance_centre=None if lat_centre is None else list(lat_centre),
                    entrance_centre_now=list(self.entrance_point_from_lattice(lat['centre'])),
                    history_n=len(self.lattice_history) if lat_centre is not None else 0,
                    lateral_m=float(
                        self.to_axis(
                            p, lat_centre if lat_centre is not None else self.entrance_point_from_lattice(lat['centre'])
                        )[1]
                    ),
                    lateral_now_m=float(self.to_axis(p, lat['centre'])[1]),
                    dist_m=float(
                        -self.to_axis(
                            p, lat_centre if lat_centre is not None else self.entrance_point_from_lattice(lat['centre'])
                        )[0]
                    ),
                )
            ),
            line=(
                None
                if not line
                else dict(
                    line,
                    point=list(line['point']),
                    dist_m=float(-self.to_axis(p, self.entrance_point_from_lattice(line['point']))[0]),
                )
            ),
            ref_axis_deg=None if self.ref_axis is None else float(math.degrees(self.ref_axis)),
            axis_source=self.axis_source,
            tag_lateral_m=float(p[1]),
            tag_distance_m=float(-p[0]),
            t=now,
        )
        if averaged:
            m.update(pose=[float(v) for v in p], pose_samples=averaged)
        # ---- disagreement between the chosen value and the others: record only, never a stop (user 2026-10-30)
        others = {
            k: v['lateral_m']
            for k, v in (('doors', m['doors']), ('lattice', m['lattice']))
            if v is not None and k != lat_src
        }
        diffs = {k: float(v - e) for k, v in others.items()}
        if any(abs(v) > t['final_lateral_tol_m'] for v in diffs.values()):
            self.events.append(
                dict(t=now, event='lateral_sources_disagree', chosen=lat_src, lateral_error_m=float(e), others=diffs)
            )
        # 태그가 기준일 때 depth 가 잰 입구까지 거리와의 차이도 기록만(안쪽 태그 줄자 값 점검용)
        if tag:
            dists = {
                k: v['dist_m']
                for k, v in (('doors', m['doors']), ('lattice', m['lattice']), ('line', m['line']))
                if v is not None and v.get('dist_m') is not None
            }
            ddiffs = {k: float(v - dist) for k, v in dists.items()}
            if any(abs(v) > t['final_lateral_tol_m'] for v in ddiffs.values()):
                self.events.append(
                    dict(
                        t=now, event='distance_sources_disagree', chosen=dist_src, distance_m=float(dist), others=ddiffs
                    )
                )
        if source == 'depth':
            self.request_anchor(now, o, p, m)
        self.depth_measure = m
        return m

    def consider_axis(self, now, p, line):
        """The lattice line's normal as the truck axis: adopt it (heading target + centreline direction) only when two consecutive
        stops disagree with the dock axis by more than lattice_axis_adopt_deg in the same direction and it lies within
        lattice_axis_max_deg; otherwise record the disagreement (a tilted tag plane shows up here first)."""
        axis = wrap(float(p[2]) + math.radians(line['axis_body_deg']))
        diff = math.degrees(wrap(axis - 0.0))
        self.axis_diffs.append(diff)
        ev = dict(
            t=now,
            axis_dock_deg=float(math.degrees(axis)),
            diff_deg=float(diff),
            spread_deg=line['spread_deg'],
            range_m=line['range_m'],
            frames=line['frames'],
            prev_diffs_deg=[float(v) for v in self.axis_diffs[-3:-1]],
        )
        if not self.t.get('lattice_axis_enabled'):
            self.events.append(dict(ev, event='lattice_axis_recorded', reason='disabled'))
            return False
        if abs(axis) > math.radians(self.t['lattice_axis_max_deg']):
            self.events.append(dict(ev, event='lattice_axis_recorded', reason='axis_off_dock'))
            return False
        need = int(self.t['lattice_axis_adopt_stops'])
        recent = self.axis_diffs[-need:]
        if (
            len(recent) < need
            or not all(abs(v) > self.t['lattice_axis_adopt_deg'] for v in recent)
            or len({math.copysign(1, v) for v in recent}) != 1
        ):
            self.events.append(
                dict(
                    ev,
                    event='lattice_axis_recorded',
                    reason=(
                        'within_tolerance'
                        if abs(diff) <= self.t['lattice_axis_adopt_deg']
                        else 'waiting_for_second_stop'
                    ),
                )
            )
            return False
        if self.ref_axis is not None and abs(wrap(axis - self.ref_axis)) <= math.radians(0.2):
            return True
        self.ref_axis = axis
        self.axis_source = 'lattice'
        self.ref_axis_report = dict(
            source='lattice',
            axis_deg=float(math.degrees(axis)),
            diff_deg=float(diff),
            frames=line['frames'],
            spread_deg=line['spread_deg'],
            t=now,
        )
        self.events.append(dict(ev, event='lattice_axis_adopted'))
        return True

    def request_anchor(self, now, o, p, m):
        """Pull the fused pose onto the measured structure: the body is distance_m before the entrance line and lateral_error_m
        left of the lane centreline (axis frame). The observer shifts its anchor by the difference; applied asynchronously.
        """
        th = self.axis()
        c, s = math.cos(th), math.sin(th)
        along = -m['distance_m']
        left = m['lateral_error_m']
        x = c * along - s * left
        y = s * along + c * left  # the body's dock position implied by the measurement (axis frame -> dock frame)
        dx = float(x - p[0])
        dy = float(y - p[1])
        if math.hypot(dx, dy) < 0.01:
            return
        self.anchor_id += 1
        self.anchor_request = dict(
            id=self.anchor_id,
            absolute=False,
            dx=dx,
            dy=dy,
            dh=0.0,
            t=now,
            source=m['lateral_source'] + '/' + str(m['distance_source']),
        )
        self.events.append(
            dict(
                t=now,
                event='anchor_requested',
                id=self.anchor_id,
                dx_m=dx,
                dy_m=dy,
                lateral_source=m['lateral_source'],
                distance_source=m['distance_source'],
            )
        )

    def recover_from_depth(self, now, o):
        """RECOVER with an untrusted pose: read the doors / lattice in the BODY frame and hand the observer an absolute dock pose
        (tag -> doors -> lattice order is implicit: a fresh tag makes the pose usable by itself)."""
        d = o.get('depth') or {}
        if d.get('stamp') is None or d['stamp'] == self.depth_last_stamp:
            return
        self.depth_last_stamp = d['stamp']
        doors = d.get('doors') or {}
        lp = d.get('lattice_pair') or {}
        ln = d.get('lattice_line') or {}
        self.recover_samples.append(
            (doors if doors.get('ok') else None, lp if lp.get('ok') else None, ln if ln.get('ok') else None)
        )
        self.recover_samples = self.recover_samples[-10:]
        n = int(self.t['depth_measure_min_frames'])
        ds = [a for a, _, _ in self.recover_samples if a]
        ls = [b for _, b, _ in self.recover_samples if b]
        ns = [c for _, _, c in self.recover_samples if c]
        if len(ds) < n and len(ls) < n:
            return
        h = None
        if len(ns) >= n:
            h = math.radians(float(np.median([v['heading_err_deg'] for v in ns])))
        elif len(ds) >= n and any(v.get('pair_heading_deg') is not None for v in ds):
            h = math.radians(
                float(np.median([v['pair_heading_deg'] for v in ds if v.get('pair_heading_deg') is not None]))
            )
        if h is None:
            return
        c, s = math.cos(h), math.sin(h)
        if len(ds) >= n:  # doors: their centre is the dock origin
            xb = float(np.median([v['x_body_m'] for v in ds]))
            yb = float(np.median([v['lateral_body_m'] for v in ds]))
            cx, cy = 0.0, 0.0
            src = 'doors'
        else:  # lattice pair: its centre (plus the offset) is (lattice_depth, 0)
            xb = float(np.median([v['x_body_m'] for v in ls]))
            yb = float(np.median([v['lateral_body_m'] for v in ls])) + self.lattice_offset
            cx, cy = float(self.lattice_depth or 0.0), 0.0
            src = 'lattice'
        x = cx - (c * xb - s * yb)
        y = cy - (s * xb + c * yb)
        self.anchor_id += 1
        self.recover_samples = []
        self.anchor_request = dict(
            id=self.anchor_id,
            absolute=True,
            x=float(x),
            y=float(y),
            h=float(h),
            local_pose=[float(v) for v in o['local_pose']],
            t=now,
            source=src,
        )
        self.events.append(
            dict(
                t=now,
                event='anchor_requested_absolute',
                id=self.anchor_id,
                x_m=float(x),
                y_m=float(y),
                heading_deg=float(math.degrees(h)),
                source=src,
            )
        )

    # ---------------------------------------------------------------- v4: inner tags after the entry, online path bias
    def collect_final_tag(self, o):
        it = ((o or {}).get('inner_tags') or {}).get('bundle') or {}
        if it.get('ok') and it.get('stamp') is not None and all(s[0] != it['stamp'] for s in self.final_tag_samples):
            self.final_tag_samples.append(
                (float(it['stamp']), float(it['x_body_m']), float(it['lateral_body_m']), float(it['heading_err_deg']))
            )

    def tag_measured(self):
        """The final pose measured by the lattice tags alone (medians over the settle window): fork-tip lateral from the lane
        centreline (left +), heading vs the lane axis (left +), fork-tip distance still to go to the target plane."""
        if not self.final_tag_samples or self.inner_plane_x is None:
            return None
        a = np.array(self.final_tag_samples, float)
        xb, yb = float(np.median(a[:, 1])), float(np.median(a[:, 2]))
        h = math.radians(float(np.median(a[:, 3])))
        f = self.cfg['site']['camera_to_fork_tip_m']
        c, s = math.cos(h), math.sin(h)
        lateral = -yb * c + (f - xb) * s  # signed distance of the fork tip from the lane line through the tag origin
        to_plane = (xb - f) * c + yb * s  # fork tip -> tag plane along the lane
        return dict(
            frames=len(a),
            fork_lateral_m=float(lateral),
            heading_err_deg=float(math.degrees(h)),
            range_to_plane_m=float(xb),
            remaining_m=float(to_plane - (self.inner_plane_x - self.cfg['target_front_x'])),
            certified_error_bound=False,
        )

    def sample_path_bias(self, now, o, b):
        """The drift of one straight leg: the dock-frame direction the vehicle actually travelled minus its heading at the start,
        valid when a tag reference was fresh at both ends and the heading did not change on the way. The median over the last
        legs becomes forward_path_bias_deg (heading_target, arrival predictions) when the site left it empty."""
        s = (b or {}).get('bias_start')
        if (
            not s
            or not self.t.get('path_bias_online')
            or not (o and o.get('usable') and o.get('pose') is not None)
            or not self.tag_fresh(o)
        ):
            return
        p0 = np.array(s[0], float)
        p1 = np.array(o['pose'], float)
        d = p1[:2] - p0[:2]
        L = float(np.hypot(*d))
        if L < self.t['path_bias_min_travel_m'] or abs(wrap(p1[2] - p0[2])) > math.radians(1.0):
            return
        bias = float(wrap(math.atan2(d[1], d[0]) - p0[2]))
        if abs(bias) > math.radians(self.t['path_bias_max_deg']):
            self.events.append(
                dict(
                    t=now, event='path_bias_implausible', bias_deg=math.degrees(bias), travel_m=L, label=b.get('label')
                )
            )
            return
        # 직진하는 동안 기준 태그가 바뀌면 위치가 튄 양이 이 표본에 섞인다
        at_start = b.get('scale_landmark')
        changed = at_start is not None and at_start != (o.get('landmark'), o.get('landmark_switches'))
        self.bias_samples.append(bias)
        self.bias_samples = self.bias_samples[-5:]
        self.bias_clean.append(not changed)
        self.bias_clean = self.bias_clean[-5:]
        need = int(self.t['path_bias_min_legs'])
        use = self.bias_samples
        if self.t.get('path_bias_clean_first'):
            clean = [s for s, c in zip(self.bias_samples, self.bias_clean) if c]
            if len(clean) >= need:
                use = clean  # 기준 태그가 그대로였던 표본이 충분하면 그것만 쓴다
        est = float(np.median(use)) if len(self.bias_samples) >= need else None
        self.events.append(
            dict(
                t=now,
                event='path_bias_sample',
                bias_deg=math.degrees(bias),
                travel_m=L,
                n=len(self.bias_samples),
                estimate_deg=None if est is None else math.degrees(est),
                label=b.get('label'),
            )
        )
        if changed:
            self.events[-1]['landmark_changed'] = True
        if est is not None and not self.bias_configured:
            previous = float(self.cfg['site'].get('forward_path_bias_deg') or 0.0)
            if abs(math.degrees(est) - previous) >= 0.1:
                self.cfg['site']['forward_path_bias_deg'] = math.degrees(est)
                self.bias_adopted = math.degrees(est)
                self.events.append(
                    dict(
                        t=now,
                        event='path_bias_adopted',
                        bias_deg=math.degrees(est),
                        n=len(self.bias_samples),
                        previous_deg=previous,
                    )
                )
                if len(use) < len(self.bias_samples):
                    self.events[-1]['clean_samples'] = len(use)

    # ---------------------------------------------------------------- the state machine
    def step(self, now, o, enter=False):
        if self.state in ('FAULT', 'COMPLETE'):
            return self.status()
        if self.mission_start is not None and now - self.mission_start > self.t['mission_timeout_s']:
            return self.fault('mission_timeout')
        if o and o.get('clock_fault'):
            return self.fault('sensor_clock_fault')
        if o and o.get('imu', {}).get('calibration', {}).get('state') == 'FAILED':
            return self.fault('imu_calibration_failed')
        fresh = self.fresh(now, o)
        new = bool(o and o.get('capture_id') != self.last_capture)
        if new:
            self.last_capture = o['capture_id']
        if new and fresh and o.get('tag', {}).get('detected'):
            self.last_seen = now
        if (
            new and o.get('landmark') in ('inner', 'bundle') and not self.inner_seen
        ):  # v4: from here the lane axis is the inner tags' (bundle heading when in range)
            self.inner_seen = True
            self.events.append(
                dict(
                    t=now,
                    event='inner_tags_reference',
                    pose=None if o.get('pose') is None else [float(v) for v in o['pose']],
                    previous_axis_source=self.axis_source,
                )
            )
            if self.ref_axis is not None:
                self.ref_axis = None
                self.axis_source = None
                self.axis_diffs = []
        st = o.get('structure') if new else None
        if st and st.get('stamp') != self.structure_stamp:
            self.structure_stamp = st.get('stamp')
            self.structure_window.append(st)
        step = (o.get('inner_fusion') or {}).get('bundle_heading_step') if new else None
        if step and step.get('from_landmark') != 'bundle' and step.get('stamp') != self.bundle_step_stamp:
            # 묶음이 처음 방향을 맞춘 순간: 그때까지 VIO 로 이어 온 방향이 묶음과 얼마나 달랐는지(기록만)
            self.bundle_step_stamp = step.get('stamp')
            self.events.append(
                dict(
                    t=now,
                    event='bundle_heading_set',
                    step_deg=float(step['step_deg']),
                    from_landmark=step.get('from_landmark'),
                    pose=None if o.get('pose') is None else [float(v) for v in o['pose']],
                )
            )
        if (
            new
            and self.anchor_request
            and o.get('anchor_applied') is not None
            and int(o['anchor_applied']) >= self.anchor_request['id']
        ):
            # the observer moved the fused pose onto the measured structure: the carried references now sit at their nominal places
            self.anchor_applied = int(o['anchor_applied'])
            self.events.append(
                dict(
                    t=now,
                    event='anchor_applied',
                    id=self.anchor_applied,
                    absolute=bool(self.anchor_request.get('absolute')),
                )
            )
            self.shift_history(self.anchor_request)
            if self.depth_centre_dock is not None:
                self.depth_centre_dock = (0.0, 0.0)
            if self.lattice_point_dock is not None and self.lattice_depth is not None:
                self.lattice_point_dock = (
                    self.lattice_depth * math.cos(self.axis()),
                    self.lattice_depth * math.sin(self.axis()),
                )
            self.anchor_request = None
        still = fresh and o.get('stationary')
        if self.state in ('PREPARING', 'WAIT_ENTER'):
            if new:
                # Retain confirmed preparation time across a short (<= 3 s) gyro flicker at the same place,
                # but not across real movement, stale tracking or coordinate resets.
                anchor = self.prep_anchor
                same = bool(
                    anchor
                    and fresh
                    and o.get('motion_epoch') == anchor.get('motion_epoch')
                    and o.get('tracking_generation') == anchor.get('tracking_generation')
                    and np.linalg.norm(np.array(o['local_pose'][:2]) - anchor['local_pose'][:2]) <= 0.004
                    and abs(wrap(o['local_pose'][2] - anchor['local_pose'][2])) <= math.radians(0.35)
                )
                if still:
                    if self.prep_pause is not None and (not same or now - self.prep_pause > 3.0):
                        self.prep_seconds = 0.0
                    if self.prep_last is not None and 0 < o['capture_id'] - self.prep_last <= self.t['frame_gap_s']:
                        self.prep_seconds += o['capture_id'] - self.prep_last
                    self.prep_last = o['capture_id']
                    self.prep_anchor = dict(o)
                    self.prep_pause = None
                else:
                    self.prep_last = None
                    if self.prep_pause is None:
                        self.prep_pause = now
                    if not same or now - self.prep_pause > 3.0:
                        self.prep_seconds = 0.0
                        self.prep_anchor = None
            if not fresh:
                self.prep_seconds = 0.0
                self.prep_last = None
                self.prep_anchor = None
            duration = self.prep_seconds
            self.preparation_report = dict(
                stable_seconds=duration,
                required_seconds=self.t['preparation_s'],
                stationary_report=(o or {}).get('stationary_report'),
                imu_ready=bool(o and o.get('sensor_ready')),
                tag_seen=bool(o and o.get('valid')),
                imu_calibration=(o or {}).get('imu', {}).get('calibration'),
                tag_repeatability=(o or {}).get('tag_position_window'),
                sigma_baseline=(o or {}).get('sigma_baseline'),
                baseline_ready=bool((o or {}).get('baseline_ready')),
                performance=(o or {}).get('performance'),
                certified_error_bound=False,
            )
            if self.state == 'PREPARING':
                if now - self.created > self.t['preparation_timeout_s']:
                    return self.fault('preparation_timeout')
                if still and duration >= self.t['preparation_s']:
                    self.state = 'WAIT_ENTER'
                    self.reason = 'ready_press_enter'
                    self.initialized_at = now
            elif not still:
                self.state = 'PREPARING'
                self.created = now
                self.reason = 'rechecking_before_enter'
            elif enter:
                self.mission_start = now
                self.state = 'PLAN'
                self.reason = 'operator_started'
            return self.status()
        if self.state == 'MOVING':
            return self.step_moving(now, o, new, still)
        self.command = 'stop'
        if self.state == 'SETTLE':
            return self.step_settle(now, o, fresh, still)
        if self.state == 'RECOVER':
            if still and o.get('usable'):
                self.state = 'PLAN'
                self.reason = 'tracking_recovered_replan'
                self.recover_samples = []
                self.depth_collect_since = None
            elif now - self.recovery_at >= self.t['recovery_timeout_s']:
                return self.fault('recovery_timeout')
            elif still and fresh and new:
                self.recover_from_depth(now, o)  # doors / lattice re-anchor the pose while the tag is gone
            return self.status()
        if self.state == 'BASELINE':
            # Tag found (or visible from the start): stand still baseline_s with it in view before the first decision.
            if not o.get('usable'):
                # A momentary VIO reset drops 'usable' for ~0.5 s: wait here. Only a tag not seen for 2 s restarts the search.
                self.baseline_last = None
                if o.get('tag', {}).get('detected'):
                    self.baseline_lost_since = None
                elif self.baseline_lost_since is None:
                    self.baseline_lost_since = now
                elif now - self.baseline_lost_since > 2.0:
                    self.state = 'PLAN'
                    self.baseline_lost_since = None
                self.reason = 'baseline_waiting_for_pose'
                return self.status()
            self.baseline_lost_since = None
            if still:
                if self.baseline_last is not None and 0 < o['capture_id'] - self.baseline_last <= self.t['frame_gap_s']:
                    self.baseline_seconds += o['capture_id'] - self.baseline_last
                self.baseline_last = o['capture_id']
                self.reason = 'stationary_baseline_with_tag'
            else:
                self.baseline_last = None
                self.reason = 'baseline_waiting_for_stillness'
            if self.baseline_seconds >= self.t['baseline_s']:
                self.baseline_done = True
                self.state = 'PLAN'
                self.reason = 'baseline_done'
            return self.status()
        if not still:
            self.recover(now, 'need_stopped_observation')
            return self.status()
        if self.state == 'PLAN':
            return self.step_plan(now, o)
        return self.status()

    def step_moving(self, now, o, new, still):
        a = self.action
        # While moving, an observation up to motion_age_s old is still used (predictive stop adds its age to the reserve).
        lost = (
            not self.fresh(now, o, self.t['motion_age_s'])
            or o['tracking_generation'] != a['generation']
            or o['motion_epoch'] != a['epoch']
        )
        turn = 'rotate' in a['command']
        search = a['after'] == 'SEARCH'
        entry = a.get('blind')
        if entry:
            # Entry: a lost VIO does not stop the vehicle inside the truck; the leg finishes on time.
            if lost and not a['timed']:
                a['timed'] = True
                a['timed_since'] = now
                self.reason = 'entry_vio_lost_timed_fallback'
            if a['timed']:
                if now - a['started'] >= a['hold_s']:
                    self.stop(now, 'COMPLETE', 'timed')
                elif now - a['started'] >= min(a['cap_s'], 2 * a['hold_s'] + 3.0):
                    self.command = 'stop'
                    return self.fault('motion_time_limit')
                return self.status()
        elif lost:
            self.recover(now, 'tracking_lost_during_motion')
            return self.status()
        if not search and not entry and not o.get('usable'):
            self.recover(now, 'pose_untrusted')
            return self.status()
        if o.get('usable') and not inside_truck(o['pose'], self.cfg):
            return self.fault('truck_side_clearance')
        if a['command'] in WEAK_NAMES and self.motion_start is None and now - a['started'] >= self.t['weak_deadband_s']:
            # Dead band (2026-10-29 v4): the weak byte did not move the vehicle -> same leg with the normal byte, and this
            # weak kind is retired for the run (the response model never learned a speed for it either).
            kind = 'rotate' if turn else 'drive'
            strong = STRONG_OF[a['command']]
            self.weak_dead[kind] = True
            self.events.append(
                dict(
                    t=now,
                    event='weak_command_deadband',
                    command=a['command'],
                    fallback=strong,
                    waited_s=float(now - a['started']),
                    label=a.get('label'),
                )
            )
            r = self.responses.get(strong)
            expected = r['startup'] + a['target'] / max(1e-6, r['speed'])
            a.update(
                command=strong,
                started=now,
                last=now,
                speed=0.0,
                cap_s=max(a['cap_s'], self.t['entry_timed_scale'] * expected + 3.0),
            )
            self.command = strong
            return self.status()
        if new:
            p = self.progress(o)
            dt = now - a['last']
            # Rate from the VIO ruler alone (the tag ruler refreshes in 0.5 s windows and jumps a few cm); clamped.
            parts = a.get('progress_parts')
            rate_src = parts[0] if parts and parts[0] is not None else p
            prev_src = a.get('previous_src', rate_src)
            if dt > 0:
                a['speed'] = float(
                    min(0.45 if not turn else 0.6, max(0.0, 0.65 * a['speed'] + 0.35 * (rate_src - prev_src) / dt))
                )
            a.update(progress=p, previous=p, previous_src=rate_src, last=now)
            if p > 0.005 and self.motion_start is None:
                self.motion_start = now
                a['np_progress'] = p
                a['np_since'] = now
            if p < (-math.radians(2) if turn else -0.03):
                return self.fault('motion_direction_mismatch')
            if entry and self.lattice_depth is not None:
                # 2026-10-30: the lattice range is a second ruler for the entry leg: the fork tips stop (lattice_depth - target_front_x)
                # before the lattice plane; whichever ruler says 'arrived' first stops the leg (the less deep stop wins).
                dep = o.get('depth') or {}
                if not self.depth_ok(dep):
                    dep = {}
                ln = dep.get('lattice_line') or {}
                ref = (
                    ln if ln.get('ok') else (dep.get('lattice_pair') or {})
                )  # the bar line, else the 2|3 pair (same plane; the bar was never seen on 2026-10-30)
                if (
                    ref.get('ok')
                    and dep.get('stamp') != a.get('lattice_stamp')
                    and 0 < ref.get('x_body_m', 0) <= self.t['depth_goal_max_range_m']
                ):
                    a['lattice_stamp'] = dep['stamp']
                    a['lattice_ruler'] = 'line' if ln.get('ok') else 'pair'
                    a['lattice_remaining_m'] = float(
                        (ref['x_body_m'] - self.cfg['site']['camera_to_fork_tip_m'])
                        - (self.lattice_depth - self.cfg['target_front_x'])
                    )
            if entry and self.inner_plane_x is not None:
                # v4: the bundle's range to the tag plane is the third ruler (independent of the VIO and of the depth)
                it = (o.get('inner_tags') or {}).get('bundle') or {}
                if (
                    it.get('ok')
                    and it.get('stamp') != a.get('tag_stamp')
                    and 0 < float(it.get('x_body_m') or 0.0) <= self.t['depth_goal_max_range_m']
                ):
                    a['tag_stamp'] = it['stamp']
                    a['tag_seen_at'] = now  # 태그 자가 신선한지 감독 시계로 본다
                    a['tag_remaining_m'] = float(
                        (it['x_body_m'] - self.cfg['site']['camera_to_fork_tip_m'])
                        - (self.inner_plane_x - self.cfg['target_front_x'])
                    )
            # No-progress safety (2026-10-22 user decision): a drive whose rulers do not advance for no_progress_s is not
            # pushing on; stop it and decide again. The entry leg switches to the time model instead (a lip is driven over).
            if not turn and self.motion_start is not None:
                if p > a.get('np_progress', -1.0) + 0.02:
                    a['np_progress'] = p
                    a['np_since'] = now
                elif now - a.get('np_since', now) >= self.t['no_progress_s']:
                    if entry:
                        if not a['timed']:
                            a['timed'] = True
                            a['timed_since'] = now
                            self.reason = 'entry_no_progress_timed_fallback'
                            self.events.append(dict(t=now, event='entry_no_progress_timed', progress_m=float(p)))
                    else:
                        self.no_progress_stops += 1
                        self.events.append(
                            dict(
                                t=now,
                                event='no_progress_stop',
                                n=self.no_progress_stops,
                                progress_m=float(p),
                                target_m=float(a['target']),
                            )
                        )
                        if self.no_progress_stops > self.t['no_progress_stops_max']:
                            self.command = 'stop'
                            return self.fault('no_progress_repeated')
                        self.stop(
                            now, 'SEARCH' if search else 'LEGS' if a['after'] == 'LEGS' else 'PLAN', 'no_progress'
                        )
                        return self.status()
            # Bundle drive watched on the depth centre: stop when the end lateral (after the turn back) crosses the centreline.
            if a.get('watch') and a.get('watch_sign') and o.get('usable') and self.depth_centre_dock is not None:
                d = o.get('depth') or {}
                if not self.depth_ok(d):
                    d = {}
                dd = d.get('doors') or {}
                lp = d.get('lattice_pair') or {}
                src = (
                    dd
                    if dd.get('ok')
                    else (dict(lp, lateral_body_m=lp['lateral_body_m'] + self.lattice_offset) if lp.get('ok') else None)
                )
                if src and not dd.get('ok') and self.lattice_alias(o, entrance_in_dock(o['pose'], src)) is not None:
                    src = None  # 옆 칸으로 잡힌 격자 쌍은 직진 감시에 쓰지 않는다
                if (
                    src
                    and d.get('stamp') is not None
                    and d['stamp'] != self.depth_last_stamp
                    and 0 < src.get('x_body_m', 0) <= self.t['depth_goal_max_range_m']
                ):
                    self.depth_last_stamp = d['stamp']
                    cx, cy = entrance_in_dock(o['pose'], src)
                    if not dd.get('ok'):
                        cx, cy = self.entrance_point_from_lattice(
                            (cx, cy)
                        )  # the lattice pair sits truck_depth_m inside
                    self.depth_centre_dock = (cx, cy)
                    q = moved(
                        o['pose'], wrap(self.heading_target() - o['pose'][2]), 0.0, self.cfg['pivot']
                    )  # after the pending turn back
                    pred = self.to_axis(q, (cx, cy))[1]
                    crossed = pred * a['watch_sign'] <= self.t['final_lateral_tol_m'] / 2
                    if crossed and self.tag_fresh(o) and not self.t.get('bundle_watch_depth_with_tag'):
                        # 2026-11-12 정민님 결정: 태그가 신선하면 depth 는 보정 직진을 멈추지 않는다(멈췄을 자리만 기록)
                        if not a.get('depth_watch_ignored'):
                            a['depth_watch_ignored'] = True
                            self.events.append(
                                dict(
                                    t=now,
                                    event='bundle_drive_depth_watch_ignored',
                                    predicted_end_lateral_m=float(pred),
                                    progress_m=float(a['progress']),
                                )
                            )
                    elif crossed:
                        self.events.append(
                            dict(
                                t=now,
                                event='bundle_drive_stopped_on_depth',
                                predicted_end_lateral_m=float(pred),
                                progress_m=float(a['progress']),
                            )
                        )
                        a.update(
                            stop_reserve=0.0,
                            stop_obs_age_s=float(now - o['arrival']),
                            stop_coast_est=float(self.responses.get(a['command'])['coast']),
                            stop_remaining_m=None,
                        )
                        self.stop(now, 'LEGS', 'depth_centre')
                        return self.status()
        if search and o['tag'].get('detected'):
            self.stop(now, 'PLAN')
            return self.status()
        if search and self.sweep + max(0, a['progress']) >= 2 * math.pi:
            self.stop(now, 'SEARCH_LIMIT')
            return self.status()
        if now - a['started'] >= a['cap_s']:
            self.command = 'stop'
            return self.fault('motion_time_limit')
        response = self.responses.get(a['command'])
        reserve = max(0.0, a['speed']) * (response['coast'] + max(0, now - o['arrival']))
        # Predictive stopping applies only after motion has actually been observed, never before the vehicle starts.
        reached = False
        tag_live = a.get('tag_seen_at') is not None and now - a['tag_seen_at'] <= self.t['tag_fresh_s']
        if entry and self.t.get('entry_lateral_watch') and self.motion_start is not None and o.get('usable'):
            # 진입 중에도 태그 묶음이 신선하면 포크 끝이 닿을 좌우를 계속 본다
            along, lat, hrel = self.to_axis(o['pose'], (0.0, 0.0))
            pred = self.entry_arrival(o['pose'], (along, lat)) if tag_live else None
            if pred is None or abs(pred) <= self.t['entry_lateral_watch_m']:
                a['watch_since'] = None
            elif a.get('watch_since') is None:
                a['watch_since'] = now
            elif now - a['watch_since'] >= self.t['tag_window_s']:
                tip_x = along + self.cfg['site']['camera_to_fork_tip_m'] * math.cos(hrel)
                if tip_x + reserve < 0.0 and self.corrections < self.t['fine_corrections_max']:
                    # 포크 끝이 멈춘 뒤에도 입구선 앞이면 세우고 마지막 확인부터 다시 한다
                    self.corrections += 1
                    self.events.append(
                        dict(
                            t=now,
                            event='entry_lateral_watch_stop',
                            predicted_arrival_m=pred,
                            fork_tip_x_m=float(tip_x),
                            progress_m=float(a['progress']),
                        )
                    )
                    a.update(
                        stop_reserve=float(reserve),
                        stop_obs_age_s=float(now - o['arrival']),
                        stop_coast_est=float(response['coast']),
                        stop_remaining_m=None,
                    )
                    self.phase = 'FINAL'
                    self.blind = False
                    self.stop(now, 'PLAN', 'entry_lateral_watch')
                    return self.status()
                if not a.get('watch_recorded'):
                    # 입구선을 넘은 뒤에는 돌지 않으므로 기록만 한다
                    a['watch_recorded'] = True
                    self.events.append(
                        dict(
                            t=now,
                            event='entry_lateral_watch_recorded',
                            predicted_arrival_m=pred,
                            fork_tip_x_m=float(tip_x),
                            progress_m=float(a['progress']),
                        )
                    )
        if entry and a.get('lattice_remaining_m') is not None and self.motion_start is not None:
            if tag_live and not self.t.get('entry_depth_ruler_with_tag'):
                # 2026-11-12 정민님 결정: 태그 자가 신선하면 depth 격자 자는 진입을 멈추지 않는다(멈췄을 자리만 기록)
                if a['lattice_remaining_m'] <= reserve and not a.get('lattice_ignored'):
                    a['lattice_ignored'] = True
                    self.events.append(
                        dict(
                            t=now,
                            event='entry_depth_ruler_ignored',
                            lattice_remaining_m=float(a['lattice_remaining_m']),
                            tag_remaining_m=a.get('tag_remaining_m'),
                            lattice_ruler=a.get('lattice_ruler'),
                            progress_m=float(a['progress']),
                        )
                    )
            else:
                reached = a['lattice_remaining_m'] <= reserve
        if entry and a.get('tag_remaining_m') is not None and self.motion_start is not None:
            reached = reached or a['tag_remaining_m'] <= reserve  # v4
        if not search and not turn and not entry and not a['command'].startswith('backward') and o.get('usable'):
            reached = remaining(o['pose'], self.cfg) <= reserve  # never drive past the fork-tip target plane
            if (
                a.get('stop_x') is not None
            ):  # rough leg: stop at the planned x (before the tag cut) by the dock-frame ruler
                reached = reached or (a['stop_x'] - o['pose'][0]) / max(0.2, math.cos(o['pose'][2])) <= reserve
        if self.motion_start is not None and (a['progress'] >= a['target'] - reserve or reached):
            a.update(
                stop_reserve=float(reserve),
                stop_obs_age_s=float(now - o['arrival']),
                stop_coast_est=float(response['coast']),
                stop_remaining_m=(
                    float(remaining(o['pose'], self.cfg)) if o.get('usable') and o.get('pose') is not None else None
                ),
            )
            self.stop(now, a['after'] if a['after'] in ('SEARCH', 'COMPLETE', 'LEGS') else 'PLAN')
        return self.status()

    def step_settle(self, now, o, fresh, still):
        if self.post_settle == 'COMPLETE':
            # Entry done. Wait for stillness, or stable_s (SETTLE_S 2 s) if the VIO cannot confirm it.
            self.collect_final_tag(o)  # v4: bundle frames of the settle window
            if not (still or now - self.stop_at >= self.t['stable_s']):
                return self.status()
            b = self.brake or {}
            usable = bool(o and o.get('usable') and o.get('pose') is not None)
            vio_ok = bool(
                fresh and b and o['tracking_generation'] == b.get('generation') and o['motion_epoch'] == b.get('epoch')
            )
            if vio_ok and not b.get('timed'):
                end = self.progress(o)
                self.responses.observe(
                    b['command'],
                    max(0, b['stop_progress']),
                    b['stop_time'] - b['started'],
                    max(0, end - b['stop_progress']),
                    dead_s=(b['motion_start'] - b['started']) if b.get('motion_start') else None,
                )
            if b:
                self.step_event(now, b, self.progress(o) if vio_ok else None, vio_ok)
            dc = self.depth_centre_dock
            self.final_report = dict(
                ended_by='timed_fallback' if b.get('timed') else 'vio_progress',
                target_m=b.get('target'),
                vio_progress_m=self.progress(o) if vio_ok else None,
                remaining_m=float(remaining(o['pose'], self.cfg)) if usable else None,
                fork_lateral_m=float(front(o['pose'], self.cfg)[1]) if usable else None,
                lateral_vs_depth_centre_m=None if (dc is None or not usable) else float(lateral_error(o['pose'], dc)),
                fork_lateral_vs_axis_m=(
                    None
                    if (dc is None or not usable)
                    else float(self.to_axis((*front(o['pose'], self.cfg), o['pose'][2]), dc)[1])
                ),
                ref_axis_deg=None if self.ref_axis is None else float(math.degrees(self.ref_axis)),
                axis_source=self.axis_source,
                lattice_remaining_m=b.get('lattice_remaining_m'),
                lattice_ruler=b.get('lattice_ruler'),
                tag_remaining_m=b.get('tag_remaining_m'),
                tag_measured=self.tag_measured(),  # v4
                heading_deg=float(math.degrees(o['pose'][2])) if usable else None,
                heading_err_deg=float(math.degrees(wrap(o['pose'][2] - self.heading_target()))) if usable else None,
                pose_available=usable,
                certified_error_bound=False,
            )
            self.sample_path_bias(now, o, self.brake)
            self.brake = None
            self.action = None
            self.state = 'COMPLETE'
            self.reason = 'entry_done_recorded_not_judged'
            return self.status()
        if still:
            if (
                self.brake
                and o['tracking_generation'] == self.brake['generation']
                and o['motion_epoch'] == self.brake['epoch']
            ):
                end = self.progress(o)
                b = self.brake
                if b['after'] == 'SEARCH':
                    self.sweep += max(0, end)
                self.responses.observe(
                    b['command'],
                    max(0, b['stop_progress']),
                    b['stop_time'] - b['started'],
                    max(0, end - b['stop_progress']),
                    dead_s=(b['motion_start'] - b['started']) if b.get('motion_start') else None,
                )
                self.step_event(now, b, end, True)
                self.sample_path_bias(now, o, b)  # v4
            elif self.brake:
                self.step_event(now, self.brake, None, False)  # tracking reset during the settle: no measured coast
            self.brake = None
            self.action = None
            if self.post_settle == 'SEARCH_LIMIT':
                return self.fault('search_360_exhausted')
            if self.post_settle == 'LEGS' and o.get('usable'):
                self.next_leg(now, o)
                return self.status()
            self.legs = []
            self.state = 'PLAN'
        elif now - self.stop_at > self.t['settle_timeout_s']:
            stopped_at = self.stop_at
            self.recover(now, 'stopped_pose_reacquisition')
            self.recovery_at = stopped_at
        return self.status()

    # ---------------------------------------------------------------- decisions at rest
    def step_plan(self, now, o):
        if not o.get('usable'):
            if self.blind or o.get('valid'):
                self.recover(now, 'absolute_pose_untrusted')
                return self.status()
            if self.search_start is None:
                self.search_start = now
            if now - self.search_start > self.t['search_timeout_s'] or self.sweep >= 2 * math.pi - math.radians(1):
                return self.fault('search_exhausted')
            # A detected but ambiguous tag calls for stationary re-observation.
            if (
                o['tag'].get('detected')
                or o['tag']['reason'] == 'not_scheduled'
                or (self.last_seen is not None and now - self.last_seen < 2.0)
            ):
                self.reason = 'waiting_for_tag_consensus'
                return self.status()
            target = min(math.radians(self.t['search_step_deg']), 2 * math.pi - self.sweep)
            self.begin(now, o, 'rotate_left_slow', target, 'SEARCH', label='search')
            return self.status()
        if not self.baseline_done:
            if self.t['baseline_s'] <= 0:
                self.baseline_done = True
            else:
                self.state = 'BASELINE'
                self.reason = 'stationary_baseline_with_tag'
                self.baseline_last = None
                return self.status()
        p = o['pose']
        rem = remaining(p, self.cfg)
        if rem < -self.t['longitudinal_tolerance_m']:
            # Already past the target (desk run 2026-10-19): back out once if the rear space allows, else finish and record.
            back = min(-rem, self.t['backward_max_m'])
            if (
                not self.overshoot_backed
                and back >= self.t['minimum_translation_m']
                and path_clear(p, 0, -back, self.cfg)
            ):
                self.overshoot_backed = True
                self.events.append(dict(t=now, event='past_target_backing_%.2fm' % back))
                self.set_legs(now, o, [dict(kind='drive', value=-back, label='past_target_back')], 'past_target_back')
                self.next_leg(now, o)
                return self.status()
            self.events.append(dict(t=now, event='past_target_finished_without_backing', remaining_m=float(rem)))
            self.state = 'COMPLETE'
            self.reason = 'already_past_target_recorded_not_judged'
            return self.status()
        if abs(rem) <= self.t['longitudinal_tolerance_m']:
            within = self.lateral_within_tolerance(o)
            self.final_report = dict(
                ended_by='at_target',
                remaining_m=float(rem),
                fork_lateral_m=float(front(p, self.cfg)[1]),
                heading_deg=float(math.degrees(p[2])),
                heading_err_deg=float(math.degrees(wrap(p[2] - self.heading_target()))),
                lateral_within_tolerance=bool(within),
                pose_available=True,
                certified_error_bound=False,
            )
            self.state = 'COMPLETE'
            self.reason = (
                'measured_stop_at_target_not_accuracy_certificate'
                if within
                else 'target_reached_lateral_error_recorded'
            )  # recorded, never a fault (user 2026-10-30)
            return self.status()
        if self.legs:
            self.next_leg(now, o)
            return self.status()
        if self.phase == 'ROUGH':
            return self.plan_rough(now, o, p)
        if self.phase == 'FINE_ENTER':
            return self.plan_fine_enter(now, o, p)
        if self.phase in ('FINE', 'FINAL'):
            return self.plan_fine(now, o, p)
        if self.phase == 'FALLBACK':
            return self.plan_fallback(now, o, p)
        return self.status()

    def turn_allowed(self, now, p, label):
        """No in-place turn once the fork tips are past the entrance line (user 2026-10-30, run 082754: a 1.1 deg straighten 0.8 m
        inside the bay): inside, only straight moves (or backing out, after which a turn is allowed again). Recorded when refused.
        """
        tip = front(p, self.cfg)
        if self.to_axis((tip[0], tip[1], p[2]), (0.0, 0.0))[0] <= 0.0:
            return True
        key = (label, round(float(tip[0]), 2))
        if (
            getattr(self, '_suppressed', None) != key
        ):  # once per stop and label (the plan ticks every 0.1 s during the depth window)
            self._suppressed = key
            self.events.append(
                dict(
                    t=now,
                    event='turn_suppressed_inside',
                    label=label,
                    fork_tip_x_m=float(tip[0]),
                    heading_deg=float(math.degrees(p[2])),
                )
            )
        return False

    def straighten(self, now, o, p, label):
        """One fine turn to the straight heading when the heading error exceeds the tolerance; bounded per decision point."""
        herr = wrap(p[2] - self.heading_target())
        if abs(herr) <= math.radians(self.t['straight_heading_tol_deg']) or self.heading_turns >= 3:
            return False
        if abs(herr) < math.radians(self.t['minimum_turn_fine_deg']) / 2:
            return False
        if not self.turn_allowed(now, p, label):
            return False
        if not path_clear(p, wrap(self.heading_target() - p[2]), 0.0, self.cfg):
            self.events.append(
                dict(t=now, event='straighten_blocked_by_walls', heading_err_deg=float(math.degrees(herr)))
            )
            return False
        self.heading_turns += 1
        self.set_legs(now, o, [dict(kind='turn', value=self.heading_target(), label=label)], label)
        self.next_leg(now, o)
        return True

    def plan_rough(self, now, o, p):
        # Order (user, 2026-10-22): sidestep first, straighten what is left, then the long drive.
        if (
            abs(p[1]) >= self.t['sidestep_min_lateral_m']
            and self.sidesteps < self.t['sidestep_max_count']
            and self.cut - p[0] >= self.t['sidestep_min_room_m']
        ):
            # Face the centreline (+-90 deg), drive across, face the truck. Both in-place turns swing the body on the pivot
            # arc; the drive is solved so the body lands on the centreline (runs 162223-174331: 40-70 % of the wanted move).
            self.sidesteps += 1
            ht = self.heading_target()
            face = -math.pi / 2 if p[1] > 0 else math.pi / 2
            side = 1.0 if face > 0 else -1.0
            q1 = moved(p, wrap(face - p[2]), 0, self.cfg['pivot'])
            y_turns = moved(q1, wrap(ht - q1[2]), 0, self.cfg['pivot'])[1]
            d = float(np.clip(self.t['sidestep_fraction'] * (-side * y_turns), 0.0, self.t['sidestep_max_m']))
            self.events.append(
                dict(
                    t=now,
                    event='sidestep',
                    n=self.sidesteps,
                    lateral_m=float(p[1]),
                    heading_deg=float(math.degrees(p[2])),
                    turn_swing_m=float(y_turns - p[1]),
                    drive_m=float(d),
                    predicted_end_lateral_m=float(y_turns + side * d),
                    pose=[float(v) for v in p],
                )
            )
            self.set_legs(
                now,
                o,
                [
                    dict(kind='turn', value=face, strength='slow', label='sidestep_face'),
                    dict(kind='drive', value=d, label='sidestep_drive'),
                    dict(kind='turn', value=ht, strength='slow', label='sidestep_back'),
                ],
                'sidestep',
            )
            self.heading_turns = 0
            self.next_leg(now, o)
            return self.status()
        if self.straighten(now, o, p, 'straighten_rough'):
            return self.status()
        stop_x = self.cut - self.t['rough_stop_before_cut_m']
        d = (stop_x - p[0]) / max(0.2, math.cos(p[2]))
        if d >= self.t['minimum_translation_m']:
            d = min(d, self.t['rough_leg_max_m'])
            self.heading_turns = 0
            self.set_legs(now, o, [dict(kind='drive', value=d, label='rough_drive', stop_x=stop_x)], 'rough_drive')
            self.next_leg(now, o)
            return self.status()
        self.phase = 'FINE_ENTER'
        self.rezero_request += 1
        self.rezero_since = now
        self.heading_turns = 0
        self.events.append(
            dict(t=now, event='rough_done', pose=[float(v) for v in p], sidesteps=self.sidesteps, cut=float(self.cut))
        )
        self.reason = 'rezero_heading_from_tag'
        return self.status()

    def plan_fine_enter(self, now, o, p):
        rz = o.get('heading_rezero') or {}
        done = rz.get('count', 0) >= self.rezero_request
        if done or now - self.rezero_since > self.t['heading_rezero_timeout_s']:
            self.rezero_report = dict(applied=bool(done), info=rz.get('info'), waited_s=now - self.rezero_since)
            self.events.append(
                dict(
                    t=now,
                    event='heading_rezero_' + ('applied' if done else 'timeout'),
                    info=rz.get('info'),
                    pose=[float(v) for v in p],
                )
            )
            self.phase = 'FINE'
            self.heading_turns = 0
            self.depth_collect_since = None
            self.reason = 'fine_stage_start'
            return self.status()
        self.reason = 'rezero_heading_from_tag'
        return self.status()

    def plan_fine(self, now, o, p):
        if self.pending_entry is not None:
            return self.run_pending_entry(now, o, p)  # the aim turn is done: go
        if self.straighten(now, o, p, 'straighten_fine'):
            return self.status()
        m = self.depth_measurement(now, o, p)
        if m is None:
            self.reason = 'measuring_depth_entrance'
            return self.status()
        if m['source'] == 'none':
            self.phase = 'FALLBACK'
            self.events.append(dict(t=now, event='depth_unavailable_fallback', pose=[float(v) for v in p]))
            return self.plan_fallback(now, o, p)
        steady = m.get('pose') is not None
        if steady:
            p = np.array(m['pose'], float)  # 마지막 확인은 서서 모은 자세로 판단한다
        ht = self.heading_target()
        e = m['lateral_error_m']
        dist = m['distance_m']
        tol = self.t['final_lateral_tol_m']
        check = final_check_distance(self.cfg)
        room = dist - check
        dmin = self.t['minimum_translation_m']
        if self.phase == 'FINE':
            if room >= dmin:
                blocked = False
                if abs(e) > tol and self.turn_allowed(now, p, 'bundle'):
                    pa = self.to_axis(
                        p, m['centre_dock']
                    )  # planned in the truck-axis frame (centreline y = 0); the kinematics are frame-free
                    b = plan_bundle(
                        pa,
                        0.0,
                        self.cfg['pivot'],
                        room,
                        dmin,
                        math.radians(self.t['turn_max_deg']),
                        wrap(ht - self.axis()),
                    )
                    if b is not None:
                        q2 = moved(moved(p, b['alpha'], 0.0, self.cfg['pivot']), 0.0, b['d'], self.cfg['pivot'])
                        if not (
                            path_clear(p, b['alpha'], b['d'], self.cfg)
                            and path_clear(q2, wrap(ht - q2[2]), 0.0, self.cfg)
                        ):
                            self.events.append(
                                dict(
                                    t=now,
                                    event='bundle_blocked_by_walls',
                                    alpha_deg=float(math.degrees(b['alpha'])),
                                    drive_m=float(b['d']),
                                )
                            )
                            b = None
                            blocked = True
                    if b is not None:
                        self.fine_bundles += 1
                        self.events.append(
                            dict(
                                t=now,
                                event='bundle',
                                n=self.fine_bundles,
                                source=m['source'],
                                lateral_source=m.get('lateral_source'),
                                distance_source=m.get('distance_source'),
                                lateral_error_m=float(e),
                                distance_m=float(dist),
                                alpha_deg=float(math.degrees(b['alpha'])),
                                drive_m=float(b['d']),
                                predicted_end_lateral_m=float(b['predicted_end_y']),
                                partial=b['partial'],
                            )
                        )
                        self.set_legs(
                            now,
                            o,
                            [
                                dict(kind='turn', value=wrap(p[2] + b['alpha']), label='bundle_turn'),
                                dict(
                                    kind='drive',
                                    value=b['d'],
                                    label='bundle_drive',
                                    watch=True,
                                    watch_sign=1.0 if e > 0 else -1.0,
                                ),
                                dict(kind='turn', value=ht, label='bundle_back'),
                            ],
                            'bundle',
                        )
                        self.heading_turns = 0
                        self.next_leg(now, o)
                        return self.status()
                if not blocked:
                    d = min(room, self.t['fine_step_max_m'])
                    self.events.append(
                        dict(
                            t=now,
                            event='fine_straight',
                            source=m['source'],
                            lateral_source=m.get('lateral_source'),
                            distance_source=m.get('distance_source'),
                            lateral_error_m=float(e),
                            distance_m=float(dist),
                            drive_m=float(d),
                        )
                    )
                    self.set_legs(now, o, [dict(kind='drive', value=d, label='fine_straight')], 'fine_straight')
                    self.heading_turns = 0
                    self.next_leg(now, o)
                    return self.status()
            self.phase = 'FINAL'
            self.events.append(
                dict(
                    t=now,
                    event='final_check_start',
                    source=m['source'],
                    lateral_source=m.get('lateral_source'),
                    distance_source=m.get('distance_source'),
                    lateral_error_m=float(e),
                    distance_m=float(dist),
                    pose=[float(v) for v in p],
                )
            )
        # ---- FINAL
        # ONE heading reference (v3, kept): ht = the adopted lattice axis minus the drift bias, else the dock axis; straighten() and
        # this check turn to the same target, so they never alternate (2026-10-28: the door axis <-> VIO fight shifted the camera
        # 3-5 cm per turn).
        herr = wrap(p[2] - ht)
        heading_src = 'vio' if self.ref_axis is None else 'lattice'
        ok_lat = abs(e) <= tol
        ok_h = abs(herr) <= math.radians(self.t['final_heading_tol_deg'])
        pred = self.entry_arrival(p, (-float(dist), float(e))) if self.t.get('final_arrival_check') else None
        ok_arr = pred is None or abs(pred) <= tol  # 지금 방향 그대로 들어가면 포크 끝이 닿을 좌우
        if ok_lat and ok_h and ok_arr:
            distance = dist + self.cfg['target_front_x'] - self.cfg['site']['camera_to_fork_tip_m']
            self.events.append(
                dict(
                    t=now,
                    event='final_check_ok',
                    source=m['source'],
                    lateral_source=m.get('lateral_source'),
                    distance_source=m.get('distance_source'),
                    lateral_error_m=float(e),
                    heading_err_deg=float(math.degrees(herr)),
                    heading_source=heading_src,
                    ref_axis_deg=None if self.ref_axis is None else float(math.degrees(self.ref_axis)),
                    distance_m=float(dist),
                    predicted_arrival_m=pred,
                )
            )
            self.begin_entry(now, o, distance / max(0.2, math.cos(herr)), m['source'])
            return self.status()
        self.corrections += 1
        if self.corrections > self.t['fine_corrections_max']:
            self.events.append(
                dict(
                    t=now,
                    event='final_corrections_exhausted',
                    lateral_error_m=float(e),
                    heading_err_deg=float(math.degrees(herr)),
                    source=m['source'],
                )
            )
            return self.best_effort_entry(now, o, p, m, e, herr, dist, 'corrections_exhausted')
        target = wrap(
            p[2] - herr
        )  # absolute dock heading that zeroes the measured error (= ht when the error comes from the VIO)
        if (
            not ok_h
            and self.turn_allowed(now, p, 'final_heading')
            and path_clear(p, wrap(target - p[2]), 0.0, self.cfg)
        ):
            self.heading_turns = 0
            self.events.append(
                dict(
                    t=now,
                    event='final_heading_turn',
                    n=self.corrections,
                    heading_err_deg=float(math.degrees(herr)),
                    heading_source=heading_src,
                )
            )
            leg = dict(kind='turn', value=target, label='final_heading', from_heading=float(p[2]) if steady else None)
            self.set_legs(now, o, [leg], 'final_heading')
            self.next_leg(now, o)
            return self.status()
        if abs(e) <= self.t['final_aim_max_m']:
            # tol..final_aim_max_m (run 102526: 3.4 cm -> back out, re-measure, 3.4 again ...): backing out changes nothing but the
            # reading, so aim the entry run at the arrival point (asin(e/L), about 1 deg) and go.
            self.events.append(
                dict(
                    t=now,
                    event='final_small_lateral_aim',
                    n=self.corrections,
                    lateral_error_m=float(e),
                    heading_err_deg=float(math.degrees(herr)),
                    lateral_source=m.get('lateral_source'),
                    predicted_arrival_m=pred,
                )
            )
            return self.best_effort_entry(now, o, p, m, e, herr, dist, 'small_lateral' if not ok_lat else 'arrival')
        # lateral off at the check point: back out (never behind the tag cut), then FINE bundles again with the room gained
        need = abs(e) / math.sin(math.radians(self.t['turn_max_deg'])) + dmin
        limit = min(self.t['backward_max_m'], p[0] - (self.cut + 0.1))
        back = min(limit, max(dmin, need - max(0.0, room)))
        if back >= dmin and path_clear(p, 0, -back, self.cfg):
            self.events.append(
                dict(t=now, event='final_back_out', n=self.corrections, lateral_error_m=float(e), back_m=float(back))
            )
            self.phase = 'FINE'
            self.set_legs(now, o, [dict(kind='drive', value=-back, label='final_back')], 'final_back')
            self.next_leg(now, o)
            return self.status()
        if room >= dmin:  # no room to back out but some ahead: one more bundle from here
            self.phase = 'FINE'
            self.events.append(
                dict(t=now, event='final_bundle_without_backing', n=self.corrections, room_m=float(room))
            )
            return self.plan_fine(now, o, p)
        self.events.append(
            dict(
                t=now,
                event='final_no_room_to_correct',
                lateral_error_m=float(e),
                room_m=float(room),
                back_limit_m=float(limit),
            )
        )
        return self.best_effort_entry(now, o, p, m, e, herr, dist, 'no_room')

    def entry_arrival(self, p, at):
        """지금 방향 그대로 곧게 들어갈 때 포크 끝이 목표 깊이에서 닿을 좌우(차로 축 기준). at = 축 기준 (앞뒤, 좌우). 못 구하면 None."""
        hrel = wrap(float(p[2]) - self.axis())
        y = tip_arrival_y(
            (at[0], at[1], hrel),
            self.cfg['pivot'],
            hrel,
            path_bias(self.cfg),
            self.cfg['site']['camera_to_fork_tip_m'],
            self.cfg['target_front_x'],
        )
        return float(y) if math.isfinite(y) else None

    def best_effort_entry(self, now, o, p, m, e, herr, dist, why, tries=0):
        """Corrections exhausted or no room (user 2026-10-30: never end without entering): enter on the best state — turn to the
        heading whose straight run puts the fork tips on the centreline at the target (straight when the lateral is already in),
        then one entry leg; the residual error is recorded in the events and the final report."""
        ht = self.heading_target()
        L = float(dist) + self.cfg['target_front_x']  # along-axis distance from the camera to the target plane
        aim = ht if (L <= 0 or abs(e) >= L) else wrap(ht - math.asin(e / L))
        if self.t.get('entry_aim_pivot') and L > 0:
            # 제자리 회전이 회전중심 둘레로 차를 옮기는 양까지 넣어 겨냥 방향을 푼다
            th = self.axis()
            rel = aim_for(
                (-float(dist), float(e), wrap(p[2] - th)),
                self.cfg['pivot'],
                path_bias(self.cfg),
                self.cfg['site']['camera_to_fork_tip_m'],
                self.cfg['target_front_x'],
                math.radians(self.t['turn_max_deg']),
            )
            if rel is not None:
                aim = wrap(th + rel)
        self.events.append(
            dict(
                t=now,
                event='best_effort_entry',
                reason=why,
                lateral_error_m=float(e),
                heading_err_deg=float(math.degrees(herr)),
                aim_deg=float(math.degrees(aim)),
                distance_m=float(dist),
                source=m['source'],
                lateral_source=m.get('lateral_source'),
            )
        )
        th = self.axis()
        self.pending_entry = dict(
            plane_x=float(p[0]) + float(dist) * math.cos(th),
            plane_y=float(p[1]) + float(dist) * math.sin(th),
            source=str(m['source']) + '_best_effort',
            source_base=str(m['source']),
            lateral_source=m.get('lateral_source'),
            lateral_error_m=float(e),
            why=why,
            tries=int(tries),
        )
        turn = wrap(aim - p[2])
        if (
            abs(turn) >= math.radians(self.t['minimum_turn_fine_deg']) / 2
            and self.turn_allowed(now, p, 'best_effort_aim')
            and path_clear(p, turn, 0.0, self.cfg)
        ):
            self.heading_turns = 0
            # 회전 뒤 실제로 돈 각으로 닿을 자리를 다시 보려고, 돌기 전 자세와 VIO 방향을 적어 둔다
            self.pending_entry.update(pose0=[float(v) for v in p], local_yaw0=float(o['local_pose'][2]))
            leg = dict(kind='turn', value=aim, label='best_effort_aim', from_heading=float(p[2]))
            self.set_legs(now, o, [leg], 'best_effort_aim')
            self.next_leg(now, o)
            return self.status()
        return self.run_pending_entry(now, o, p)

    def run_pending_entry(self, now, o, p):
        pe = self.pending_entry
        self.pending_entry = None
        if pe.get('pose0') is not None and self.t.get('entry_aim_recheck') and pe['tries'] < 1:
            # 겨냥 회전이 실제로 돈 각(VIO)으로 닿을 자리를 다시 계산하고, 아직 벗어나 있으면 한 번만 더 겨냥한다
            turned = wrap(float(o['local_pose'][2]) - pe['local_yaw0'])
            q = moved(pe['pose0'], turned, 0.0, self.cfg['pivot'])
            a0, shift, _ = self.to_axis(q, (pe['plane_x'], pe['plane_y']))
            lat = (
                pe['lateral_error_m'] + shift
            )  # 적어 둔 입구 점은 돌기 전 차와 같은 좌우에 있다: 회전이 옮긴 양만 더한다
            pred = self.entry_arrival(q, (a0, lat))
            if pred is not None and abs(pred) > self.t['final_lateral_tol_m'] and a0 < 0:
                self.events.append(
                    dict(
                        t=now,
                        event='best_effort_reaim',
                        predicted_arrival_m=pred,
                        turned_deg=float(math.degrees(turned)),
                        wanted_deg=float(math.degrees(wrap(o['pose'][2] - q[2]))) if o.get('pose') else None,
                    )
                )
                stub = dict(source=pe['source_base'], lateral_source=pe.get('lateral_source'))
                herr = wrap(q[2] - self.heading_target())
                return self.best_effort_entry(now, o, q, stub, lat, herr, -a0, pe['why'], tries=pe['tries'] + 1)
        along = -self.to_axis(p, (pe['plane_x'], pe['plane_y']))[0]  # distance to the entrance plane from here
        distance = along + self.cfg['target_front_x'] - self.cfg['site']['camera_to_fork_tip_m']
        self.begin_entry(now, o, distance / max(0.2, math.cos(wrap(p[2] - self.heading_target()))), pe['source'])
        return self.status()

    def plan_fallback(self, now, o, p):
        """Tag-pose blind entry (no depth): aim the predicted fork-tip arrival at the centreline (bounded fine turns), then one leg."""
        rem = remaining(p, self.cfg)
        h = p[2] + path_bias(self.cfg)
        L = rem / max(0.2, math.cos(h)) + self.cfg['site']['camera_to_fork_tip_m']
        if (
            abs(arrival_y(p, self.cfg)) > self.t['position_tolerance_m']
            and self.fallback_corrections < self.t['transition_corrections_max']
            and L > 0
            and abs(p[1]) < L
            and self.turn_allowed(now, p, 'fallback_aim')
        ):
            self.fallback_corrections += 1
            aim = wrap(
                -math.asin(p[1] / L) - path_bias(self.cfg)
            )  # heading whose straight path arrives on the centreline
            self.events.append(
                dict(
                    t=now,
                    event='fallback_aim_turn',
                    n=self.fallback_corrections,
                    arrival_m=float(arrival_y(p, self.cfg)),
                    aim_deg=float(math.degrees(aim)),
                )
            )
            self.set_legs(now, o, [dict(kind='turn', value=aim, label='fallback_aim')], 'fallback_aim')
            self.next_leg(now, o)
            return self.status()
        self.lateral_within_tolerance(o, arrival=True)  # records the decision with its σ
        self.begin_entry(now, o, rem / max(0.2, math.cos(h)), 'fallback_tag_pose')
        return self.status()
