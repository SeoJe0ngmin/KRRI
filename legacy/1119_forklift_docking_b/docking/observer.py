"""OpenVINS observations -> existing controller contract, no commanded pose.

2026-10-19: tag main, VIO assist (see fusion.py). Stationary judgment by gyro (stationary.py).
"""

import math
import numpy as np
from .stationary import StationaryDetector
from .history import PoseHistory, State
from .fusion import Fusion, tag_bearing_range, pnp_dock_pose
from .geometry import pose, path_bias, wrap
from .vio_quality import quality
from .uncertainty import (
    transform,
    inverse,
    covariance,
    se3_planar,
    conjugated_planar_jacobian,
    fork_uncertainty,
    target_lateral_uncertainty,
)


def inner_body(inner, T_body_camera, stamp):
    """v4: what the lattice tags say in the BODY frame, independent of the VIO (the entry ruler, the final report and
    scripts/tag_test.py use it): bundle = range straight ahead to the tag plane, lateral of the inner origin (the lane centre
    point on the tag plane), body heading vs the plane normal (left +); single = the first tag solved alone."""
    if not inner:
        return None
    Tbc = np.asarray(T_body_camera, float)
    b = inner.get('bundle') or {}
    out = dict(
        stamp=float(stamp),
        seen=[int(i) for i in (inner.get('seen') or [])],
        bundle=None,
        single=None,
        bundle_reason=b.get('reason'),
        bundle_reproj_px=b.get('reproj_px'),
        bundle_ambiguous=bool(b.get('ambiguous')),
    )
    if b.get('ok') and b.get('T_camera_inner') is not None:
        Tbi = Tbc @ np.asarray(b['T_camera_inner'], float)
        o = Tbi[:3, 3]
        nrm = Tbi[:3, 2]  # inner z axis = the plane normal, pointing into the bay
        x0 = float((nrm[0] * o[0] + nrm[1] * o[1]) / nrm[0]) if abs(nrm[0]) > 1e-6 else float(o[0])
        out['bundle'] = dict(
            ok=True,
            stamp=float(stamp),
            x_body_m=x0,
            lateral_body_m=float(o[1]),
            heading_err_deg=float(-math.degrees(math.atan2(nrm[1], nrm[0]))),
            reproj_px=b.get('reproj_px'),
            ambiguous=bool(b.get('ambiguous')),
        )
    single = [(k, v) for k, v in (inner.get('tags') or {}).items() if v.get('ok') and v.get('T_camera_tag') is not None]
    if single:
        k, v = single[0]
        Tbt = Tbc @ np.asarray(v['T_camera_tag'], float)
        out['single'] = dict(
            id=int(k),
            x_body_m=float(Tbt[0, 3]),
            lateral_body_m=float(Tbt[1, 3]),
            bearing_deg=float(math.degrees(math.atan2(Tbt[1, 3], Tbt[0, 3]))),
            n=len(single),
        )
    return out


class Observer:
    def __init__(self, cfg):
        self.cfg = cfg
        self.t = cfg['tuning']
        self.u = cfg['uncertainty']
        self.cal = cfg['calibration']
        self.history = PoseHistory(self.u['history_s'], self.u['history_samples'], self.u['interpolation_gap_s'])
        self.fusion = Fusion(self.t, self.cal['T_body_camera'], self.cal['T_dock_tag'])
        self.motion_epoch = 0
        self.last_tag = dict(ok=False, reason='not_seen', detected=False, search_candidate=False)
        self.tag_report = dict(reason='no_tag')
        self.last_tag_stamp = None
        self.previous_local = None
        self.last_state = None
        self.rejected_tags = 0
        self.stationary_detector = StationaryDetector(self.t, cfg.get('camera', {}).get('gyro_hz', 200.0))
        self.still = False
        self.rezero_target = 0  # requested count (supervisor hint); applied when enough still tag frames exist
        self.anchor_applied = 0
        self.anchor_info = None  # docking 2026-10-30: depth re-anchor requests from the controller, applied once per id
        self.last_inner = None
        self.inner_report = dict(reason='no_inner_tags')
        self.switches = 0  # v4: inner (lattice) tags
        self._Tbi = (
            None  # body <- IMU, validated once (it is a calibration constant; it was re-validated twice per packet)
        )

    def body_imu(self):
        raw = np.asarray(self.cal['T_body_imu'], float)
        if self._Tbi is None or not np.array_equal(raw, self._Tbi[0]):
            Tbi = transform(raw)
            self._Tbi = (Tbi, inverse(Tbi))
        return self._Tbi

    def body_state(self, packet):
        # Native adapter publishes T_odom_imu and a world-frame left-error P.
        T = transform(packet['T_odom_imu'])
        P = covariance(packet['covariance'], 6, True)
        Tbi, Tib = self.body_imu()
        Tb = T @ Tib
        # Analytic left-error Jacobian avoids Rodrigues inverse's small-angle
        # dead zone, which would otherwise erase orientation covariance.
        r = T[:3, :3] @ Tib[:3, 3]
        cross = np.array([[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]])
        J = np.eye(6)
        J[:3, 3:] = -cross
        return State(
            float(packet['stamp']),
            int(packet['epoch']),
            Tb,
            J @ P @ J.T,
            float(packet['arrival']),
            int(packet['features']),
        )

    def add_gyro(self, stamp, w):
        """Raw gyro sample (200 Hz) for the stationary judgment; fed by the camera worker."""
        self.stationary_detector.add_gyro(stamp, w)

    def add_vio(self, packet, neutral=True):
        if packet.get('fault'):
            raise ValueError('OpenVINS: ' + packet['fault'])
        if not packet.get('initialized'):
            return None
        self.stationary_detector.set_bias(packet.get('gyro_bias'))
        if packet.get('baseline_lock'):
            self.fusion.baseline.freeze()  # controller finished its 30 s baseline phase (after the tag search)
        report = quality(packet, self.cfg)
        if not report['state_current']:
            # Never label an old estimator state with a new camera timestamp or
            # feed it to tag fusion, relative motion or stationary detection.
            self.history.items.clear()
            self.last_state = None
            self.fusion.reset(packet['epoch'])
            self.stationary_detector.reset_window()
            return dict(
                capture_id=packet['stamp'],
                arrival=packet['arrival'],
                sensor_epoch=packet['epoch'],
                sensor_ready=True,
                local_valid=False,
                local_pose=[],
                pose=None,
                usable=False,
                stationary=False,
                clock_fault=False,
                imu_bad=True,
                transport_lag_s=packet['transport_lag_s'],
                tag=dict(self.last_tag),
                odom=report,
                imu=dict(ready=True),
            )
        state = self.body_state(packet)
        previous = self.last_state
        new_epoch = previous is None or previous.epoch != state.epoch
        continuity = (
            not new_epoch
            and packet.get('delta_valid')
            and abs(packet.get('delta_from_stamp', -1) - previous.stamp) < 1e-5
            and state.stamp - previous.stamp <= self.t['frame_gap_s']
        )
        if new_epoch or not continuity:
            self.fusion.reset(state.epoch)
            self.stationary_detector.reset_window()
            self.previous_local = None
            self.motion_epoch += 1
        p, P = se3_planar(state.T, state.P)
        if self.t.get('vio_scale_fused'):
            self.fusion.set_vio_scale(packet.get('vio_scale', 1.0), p[:2])  # 감독이 채택한 VIO 거리 배율
        if continuity:
            D = transform(packet['relative_transform'])
            Pd = covariance(packet['relative_covariance'], 6)
            Tbi, Tib = self.body_imu()
            d = pose(Tbi @ D @ Tib)
            J = conjugated_planar_jacobian(Tbi, D)
            Q = J @ Pd @ J.T
            heading = p[2] + (self.fusion.offset or 0.0)
            self.fusion.vio_step(d[:2], Q, heading)
        # Additional engineering uncertainty per missing IMU interval (counted, never fatal — rule of 2026-10-19).
        # Native integration uses real timestamps; no synthetic zero samples.
        gaps = packet.get('imu_gaps', 0)
        extra = max(0, gaps - getattr(self, 'last_imu_gaps', 0))
        self.last_imu_gaps = gaps
        if extra:
            self.fusion.Pvio = self.fusion.Pvio + extra * np.diag([0.03**2, 0.03**2, math.radians(2.0) ** 2])
        self.history.add(state)
        self.last_state = state
        self.previous_local = p.copy()
        report = quality(packet, self.cfg, continuity=bool(continuity))
        good = report['ok']
        report['imu_gaps'] = gaps
        report['imu_gap_uncertainty_added'] = bool(extra)
        imu_age = packet['arrival'] - packet.get('imu_arrival', -math.inf)
        since_neutral = packet.get('since_neutral_s', math.inf if neutral else None)
        self.still = self.stationary_detector.update(
            state.stamp,
            p,
            neutral and packet.get('captured_after_neutral', True),
            since_neutral,
            good and 0 <= imu_age <= self.t['imu_age_s'],
            packet.get('gyro_norm'),
        )
        return self.snapshot(packet, p, P, report, bool(self.still))

    def request_rezero(self, count):
        """Supervisor asks (count increases) for a heading re-zero from the tag; applied on the next still tag frames."""
        self.rezero_target = max(self.rezero_target, int(count))

    def add_tag(self, tag):
        self.last_tag = {k: v for k, v in tag.items() if k not in ('T_camera_tag', 'covariance', 'inner')}
        self.last_tag_stamp = tag['stamp']
        inner = tag.get('inner') or {}
        if inner:
            self.last_tag['inner_view'] = dict(
                tags={
                    str(k): dict(ok=bool(v.get('ok')), corners=v.get('corners'))
                    for k, v in (inner.get('tags') or {}).items()
                },
                bundle_ok=bool((inner.get('bundle') or {}).get('ok')),
            )  # v4: for the colour panel
        self.last_inner = self.inner_summary(inner, tag['stamp'])
        if not tag.get('ok'):
            self.tag_report = dict(accepted=False, reason=tag['reason'])
        if not tag.get('ok') and not inner:
            return self.tag_report
        try:
            state = self.history.at(tag['stamp'], tag['sensor_epoch'])
            op, _ = se3_planar(state.T, state.P)
        except ValueError as exc:
            self.rejected_tags += 1
            if tag.get('ok'):
                self.tag_report = dict(accepted=False, reason=str(exc))
            if inner:
                self.inner_report = dict(accepted=False, reason=str(exc), kind=None)
            return self.tag_report
        gyro = None if self.stationary_detector.last_rate is None else math.degrees(self.stationary_detector.last_rate)
        accepted = False
        if tag.get('ok'):
            try:
                bearing, rng = tag_bearing_range(tag['T_camera_tag'], self.cal['T_body_camera'])
                raw = pnp_dock_pose(tag['T_camera_tag'], self.cal['T_body_camera'], self.cal['T_dock_tag'])
                report = self.fusion.tag(tag['stamp'], bearing, rng, float(raw[2]), op, self.still, gyro)
                report = dict(report, raw_tag_pose=raw.tolist(), bearing_deg=math.degrees(bearing), range_m=rng)
            except ValueError as exc:
                report = dict(accepted=False, reason=str(exc))
                self.rejected_tags += 1
            self.tag_report = report
            accepted = bool(report.get('accepted'))
        if inner:
            self.inner_report = self.add_inner(inner, tag['stamp'], op, gyro)
            accepted = accepted or bool(self.inner_report.get('accepted'))
            if self.inner_report.get('reason') == 'landmark_switch' or (
                tag.get('ok') and self.tag_report.get('reason') == 'landmark_switch'
            ):
                self.switches += 1
        if self.rezero_target > self.fusion.rezero_count and self.still and accepted:
            self.fusion.rezero(
                self.t['heading_rezero_s'], int(self.t['depth_measure_min_frames'])
            )  # None until enough still frames
        return self.tag_report

    def add_inner(self, inner, stamp, op, gyro):
        """v4: the lattice tags into the fusion. Both solved together (bundle) -> the PnP body pose is the anchor and the strong
        heading; exactly one solved -> a bearing/range anchor to that tag's dock position (weak heading); both solved alone but
        the joint solve rejected -> nothing (the mismatch is visible as bundle_reproj_px in the log: check inner_tag_spacing_m).
        """
        cal = self.cal.get('inner') or {}
        if not cal:
            return dict(accepted=False, reason='inner_tags_not_configured', kind=None)
        Tbc = self.cal['T_body_camera']
        b = inner.get('bundle') or {}
        summary = self.last_inner or {}
        near = bool(
            (summary.get('bundle') or {}).get('x_body_m') is not None
            and summary['bundle']['x_body_m'] <= float(self.t.get('inner_bundle_max_range_m', 3.5))
        )
        try:
            if b.get('ok') and b.get('T_camera_inner') is not None and near:
                Tdi = np.asarray(cal['T_dock_inner'], float)
                picked = None
                cands = [
                    np.asarray(c['T_camera_inner'], float)
                    for c in (b.get('candidates') or [])
                    if c.get('T_camera_inner') is not None
                ] or [np.asarray(b['T_camera_inner'], float)]
                poses = [pose(transform(Tdi) @ inverse(T) @ inverse(Tbc)) for T in cands]
                if b.get('ambiguous') and len(poses) > 1:
                    # The planar twin fits nearly as well (a nearly frontal target): the fused heading picks the candidate; without
                    # a heading reference yet the frame is skipped rather than guessed.
                    if self.fusion.offset is None:
                        return dict(
                            accepted=False,
                            reason='bundle_ambiguous_no_heading_prior',
                            kind='bundle',
                            reproj_px=b.get('reproj_px'),
                        )
                    prior = wrap(float(op[2]) + self.fusion.offset)
                    k = int(np.argmin([abs(wrap(p[2] - prior)) for p in poses]))
                    picked = 'heading_prior'
                else:
                    k = 0
                Tci = cands[k]
                pose_dock = poses[k]
                bearing, rng = tag_bearing_range(Tci, Tbc)
                r = self.fusion.tag(
                    stamp,
                    bearing,
                    rng,
                    float(pose_dock[2]),
                    op,
                    self.still,
                    gyro,
                    kind='bundle',
                    tag_xy=Tdi[:2, 3],
                    pose_dock=pose_dock,
                )
                return dict(
                    r,
                    kind='bundle',
                    pose_dock=pose_dock.tolist(),
                    bearing_deg=math.degrees(bearing),
                    range_m=rng,
                    reproj_px=b.get('reproj_px'),
                    picked=picked,
                    alternatives_deg=[float(math.degrees(p[2])) for p in poses],
                )
            tags = inner.get('tags') or {}
            ok = [
                (int(k), v)
                for k, v in tags.items()
                if v.get('ok') and v.get('T_camera_tag') is not None and str(k) in cal['tags']
            ]
            if ok and (not b.get('ok') or not near or len(ok) == 1):
                # Each tag on its own: a bearing/range anchor to its dock position, the heading stays the VIO's (beyond
                # inner_bundle_max_range_m the bundle heading is worse than the VIO; with one tag there is no bundle).
                r = None
                ids = []
                for i, v in ok:
                    Tct = np.asarray(v['T_camera_tag'], float)
                    Tdt = np.asarray(cal['tags'][str(i)], float)
                    raw = pnp_dock_pose(Tct, Tbc, Tdt)
                    bearing, rng = tag_bearing_range(Tct, Tbc)
                    r = self.fusion.tag(
                        stamp, bearing, rng, float(raw[2]), op, self.still, gyro, kind='inner', tag_xy=Tdt[:2, 3]
                    )
                    ids.append(i)
                return dict(
                    r,
                    kind='inner',
                    id=ids[-1],
                    ids=ids,
                    bearing_deg=math.degrees(bearing),
                    range_m=rng,
                    bundle_far=bool(b.get('ok') and not near),
                    bundle_reason=b.get('reason'),
                )
            if len(ok) == 2:
                return dict(
                    accepted=False,
                    reason='bundle_rejected:' + str(b.get('reason')),
                    kind='inner_pair',
                    reproj_px=b.get('reproj_px'),
                )
            return dict(accepted=False, reason=b.get('reason') or 'inner_tags_not_seen', kind=None)
        except ValueError as exc:
            self.rejected_tags += 1
            return dict(accepted=False, reason=str(exc), kind=None)

    def inner_summary(self, inner, stamp):
        return inner_body(inner, self.cal['T_body_camera'], stamp)

    def request_anchor(self, req):
        """Depth re-anchor requested by the controller (id, absolute, dx/dy or x/y/h + local_pose); applied once per id."""
        if not req or int(req.get('id') or 0) <= self.anchor_applied:
            return
        stamp = self.last_state.stamp if self.last_state is not None else None
        if req.get('absolute'):
            ok = self.fusion.reanchor_absolute(req['x'], req['y'], req['h'], req['local_pose'], stamp)
        else:
            ok = self.fusion.reanchor_shift(
                req['dx'], req['dy'], stamp, None if self.previous_local is None else self.previous_local[:2]
            )
        self.anchor_applied = int(req['id'])
        self.anchor_info = dict({k: v for k, v in req.items() if k != 'local_pose'}, applied=bool(ok), stamp=stamp)

    def snapshot(self, packet, local, Plocal, odom, still):
        good = odom['ok']
        state = self.last_state
        fork_report = None
        age = None
        world, P, parts = self.fusion.estimate(local)
        if world is not None:
            age = max(0.0, state.stamp - self.fusion.anchor_stamp)
            # Diagnostic at the controlled point, not only the camera origin.
            site = self.cfg['site']
            f, F = fork_uncertainty(world, P, site['camera_to_fork_tip_m'], self.u['fork_length_std_m'])
            y, Y = target_lateral_uncertainty(world, P, self.cfg['target_front_x'], path_bias(self.cfg))
            fork_report = dict(
                position=f.tolist(),
                covariance=F.tolist(),
                std_m=np.sqrt(np.diag(F)).tolist(),
                predicted_arrival_y_m=y,
                predicted_arrival_std_m=None if Y is None else math.sqrt(Y),
                certified_error_bound=False,
            )
        radius = None if P is None else self.u['confidence_sigma'] * np.sqrt(np.maximum(np.diag(P), 0))
        # σ is recorded, not a gate (2026-10-19). Only VIO-only duration/distance bound the pose.
        usable = bool(
            good
            and world is not None
            and age <= self.t['blind_time_limit_s']
            and self.fusion.travel <= self.t['blind_distance_limit_m']
        )
        tag = dict(self.last_tag)
        inner_tags = self.last_inner
        if self.last_tag_stamp is None or state.stamp - self.last_tag_stamp > self.u['tag_status_age_s']:
            tag = dict(ok=False, reason='missing_or_stale', detected=False, search_candidate=False)
            inner_tags = None
        else:
            # Controller's detected field means a useful search candidate.
            tag['detected'] = tag.get('search_candidate', False)
        imu_age = packet['arrival'] - packet.get('imu_arrival', -math.inf)
        out = dict(
            capture_id=state.stamp,
            arrival=packet['arrival'],
            sensor_epoch=state.epoch,
            clock_fault=False,
            sensor_ready=True,
            imu_bad=not 0 <= imu_age <= self.t['imu_age_s'],
            transport_lag_s=packet['transport_lag_s'],
            pose=None if world is None else world.tolist(),
            covariance=None if P is None else P.tolist(),
            confidence_radius=None if radius is None else radius.tolist(),
            confidence_kind='tag_window_plus_30s_baseline_plus_vio_relative_variance_not_certified',
            bounded=False,
            sigma=parts,
            sigma_baseline=self.fusion.baseline.value,
            baseline_ready=self.fusion.baseline.value is not None,
            valid=world is not None,
            usable=usable,
            local_pose=local.tolist(),
            local_valid=good,
            stationary=still,
            stationary_report=dict(self.stationary_detector.report),
            tracking_generation=state.epoch,
            motion_epoch=self.motion_epoch,
            anchor_stamp=self.fusion.anchor_stamp,
            blind_age_s=age,
            distance_since_anchor_m=self.fusion.travel,
            heading_source='vio_yaw_plus_offset' if self.fusion.offset is not None else None,
            mount_yaw_online_deg=math.degrees(self.fusion.mount_yaw),
            good_legs=len(self.fusion.mount_yaw_samples),
            last_leg=self.fusion.last_leg,
            tag=tag,
            tag_fusion=dict(self.tag_report),
            tag_position_window=self.fusion.last_window,
            fork_uncertainty=fork_report,
            inner_tags=inner_tags,
            inner_fusion=dict(self.inner_report),
            landmark=self.fusion.landmark,
            landmark_switches=self.switches,  # v4
            heading_rezero=dict(
                count=self.fusion.rezero_count, requested=self.rezero_target, info=self.fusion.rezero_info
            ),
            anchor_applied=self.anchor_applied,
            anchor_info=self.anchor_info,
            odom=dict(odom, usable_for_docking=usable),
            imu=dict(
                ready=True,
                age_s=imu_age,
                gyro_bias_rps=packet.get('gyro_bias'),
                accel_bias_mps2=packet.get('accel_bias'),
                calibration=dict(state='READY', reason='openvins_initialized'),
            ),
            source='AprilTag bearing/range windows (main) + OpenVINS heading/propagation (assist)',
        )
        if self.t.get('vio_scale_fused'):
            out['vio_scale_fused'] = float(self.fusion.vio_scale)  # 융합 위치에 곱한 VIO 거리 배율(기록)
        return out
