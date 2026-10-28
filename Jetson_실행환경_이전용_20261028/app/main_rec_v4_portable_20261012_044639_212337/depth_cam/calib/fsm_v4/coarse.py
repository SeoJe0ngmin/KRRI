"""Distance-adaptive lateral trigger -> IMU turn/FWD/opposite 90-degree turn."""
import math
from statistics import median

from calib.config import COLOR_META
from calib.control import configure_motion_deadline, motion_deadline_expired
from . import config as cfg
from .motion import drive_seconds
from .pose import wrap_180


COARSE_BLIND_STATES = frozenset({
    "COARSE_PREPARE", "COARSE_ROTATE", "COARSE_ROTATE_SETTLE",
    "COARSE_DRIVE", "COARSE_DRIVE_SETTLE", "COARSE_RETURN_ROTATE",
    "COARSE_RETURN_SETTLE",
})
COARSE_STATES = COARSE_BLIND_STATES | {"COARSE_REACQUIRE"}


def coarse_lateral_limit_m(distance_m):
    """Pallet-normal pivot distance, metres; far limits stop growing at 5 m.

    The measured domain is 3..5 m. Below 3 m the available approach term
    shrinks to zero at staging; that extension is not a measured boundary.
    """
    if not math.isfinite(distance_m):
        raise ValueError("invalid coarse distance")
    approach = max(0., min(distance_m, cfg.COARSE_LATERAL_CALIBRATION_MAX_DISTANCE_M)
                   - cfg.STAGING_DISTANCE_M)
    return cfg.COARSE_LATERAL_BASE_M + cfg.COARSE_LATERAL_GAIN * approach


def coarse_plan(pose):
    """Positive rotation is right; pallet X is right along the front face."""
    lateral = float(pose.rot_x_pallet_m)
    yaw = float(pose.yaw_deg)
    if not all(map(math.isfinite, (lateral, yaw, pose.rot_z_pallet_m))):
        raise ValueError("invalid coarse pose")
    if abs(yaw) >= 90.0:
        raise ValueError("coarse correction requires a front-facing pallet pose")
    if abs(lateral) < cfg.COARSE_MIN_LATERAL_M or (
            cfg.COARSE_TRAVEL_LIMITS_ENABLED and abs(lateral) > cfg.COARSE_MAX_LATERAL_M):
        raise ValueError("coarse lateral outside configured travel limits")
    if -pose.rot_z_pallet_m < cfg.STAGING_DISTANCE_M:
        raise ValueError("insufficient front-plane clearance for coarse rotation")
    side = math.copysign(1.0, lateral)
    turn = wrap_180(yaw - side * 90.0)
    duration = drive_seconds(abs(lateral), apply_max_limit=cfg.COARSE_TRAVEL_LIMITS_ENABLED)
    if not math.isfinite(duration) or (cfg.COARSE_TRAVEL_LIMITS_ENABLED
            and duration >= min(cfg.FWD_COMMAND_MAX_SEC, cfg.FWD_MAX_COMMAND_SEC)):
        raise ValueError("coarse distance exceeds fitted forward time range")
    return turn, abs(lateral), duration, side * 90.0


class CoarseAlignmentMixin:
    @property
    def requires_imu(self):
        return cfg.COARSE_IMU_ENABLED

    @property
    def imu_active_required(self):
        return cfg.COARSE_IMU_ENABLED and self.state in COARSE_BLIND_STATES

    def _coarse_reset(self):
        self._coarse_done = False
        self._coarse_started = None
        self._coarse_bias = 0.0
        self._coarse_bias_samples = []
        self._coarse_bias_stamp = None
        self._coarse_imu_ready_mono = None
        self._coarse_brake_ready_mono = None
        self._coarse_last_quiet = None
        self._coarse_target = 0.0
        self._coarse_progress = 0.0
        # Source is bound by the runtime, and survives operator FSM resets.
        if not hasattr(self, "imu_source"):
            self.imu_source = None
        configure_motion_deadline(None)

    def _coarse_required(self, pose):
        if not cfg.COARSE_IMU_ENABLED or getattr(self, "_coarse_done", False):
            return False
        if not all(map(math.isfinite, (pose.rot_x_pallet_m, pose.rot_z_pallet_m, pose.yaw_deg))):
            return True  # Let coarse_plan reject invalid input with STOP.
        return abs(pose.rot_x_pallet_m) > coarse_lateral_limit_m(-pose.rot_z_pallet_m)

    def _maybe_begin_coarse(self, pose, now, lines):
        required = self._coarse_required(pose)
        if (cfg.COARSE_IMU_ENABLED and not getattr(self, "_coarse_done", False)
                and all(map(math.isfinite, (pose.rot_x_pallet_m, pose.rot_z_pallet_m)))):
            lines.append((f"[COARSE CHECK] |L|={abs(pose.rot_x_pallet_m):.3f}m, "
                          f"limit={coarse_lateral_limit_m(-pose.rot_z_pallet_m):.3f}m, "
                          f"D={-pose.rot_z_pallet_m:.3f}m -> "
                          f"{'initial correction' if required else 'normal approach'}", COLOR_META))
        if not required:
            return False
        self._exec("STOP")
        try:
            self._coarse_plan = coarse_plan(pose)
        except ValueError as exc:
            self._fail(str(exc), lines)
            return True
        self._coarse_done = True
        self._coarse_started = now
        self._coarse_bias_samples = []
        self._coarse_bias_stamp = None
        self._coarse_imu_ready_mono = None
        # STOP history is shared with the preceding visual settle. Sensor
        # readiness and fresh bias collection below remain independent.
        stopped = getattr(self, '_stopped_since_mono', None)
        self._coarse_brake_ready_mono = (
            None if stopped is None else stopped + cfg.COARSE_IMU_POST_STOP_DELAY_SEC
        )
        self._set_state("COARSE_PREPARE")
        self._trace_begin("v4_coarse_alignment", "STOP", model_yaw_deg=pose.yaw_deg,
                          lateral_m=pose.rot_x_pallet_m,
                          trigger_basis="distance_adaptive_lateral",
                          normal_distance_m=-pose.rot_z_pallet_m,
                          lateral_limit_m=coarse_lateral_limit_m(-pose.rot_z_pallet_m),
                          first_turn_deg=self._coarse_plan[0],
                          forward_hold_sec=self._coarse_plan[2],
                          stopped_since_mono=stopped,
                          brake_guard_ready_mono=self._coarse_brake_ready_mono,
                          return_turn_deg=self._coarse_plan[3])
        lines.append(("[COARSE] stopped gyro bias measurement", COLOR_META))
        return True

    def _coarse_abort(self, reason, lines):
        self._exec("STOP")
        configure_motion_deadline(None)
        self._fail(reason, lines)

    def _coarse_yaw(self, sample):
        yaw, stamp, _rate, _error = sample
        return yaw - self._coarse_bias * (stamp - self._coarse_bias_origin)

    def _coarse_turn(self, target, sample, now, state):
        self._coarse_ref = self._coarse_yaw(sample)
        self._coarse_target = target
        self._coarse_stop_deg = (cfg.COARSE_IMU_90_STOP_DEG
                                 if math.isclose(abs(target), 90., abs_tol=1e-9, rel_tol=0.)
                                 else abs(target))
        self._coarse_progress = 0.0
        self._coarse_command = "ROT_RIGHT" if target > 0 else "ROT_LEFT"
        self._set_state(state, cfg.COARSE_ROTATION_TIMEOUT_SEC)
        configure_motion_deadline(now + cfg.COARSE_COMMAND_LEASE_SEC)
        self._exec(self._coarse_command)
        self._trace_begin("v4_coarse_turn", self._coarse_command,
                          target_deg=target, stop_threshold_deg=self._coarse_stop_deg,
                          control_basis="imu_relative_yaw")

    def _coarse_settle(self, state, now):
        self._exec("STOP")
        configure_motion_deadline(None)
        self._coarse_last_quiet = None
        self._set_state(state, cfg.COARSE_SETTLE_TIMEOUT_SEC)

    def _coarse_step(self, now, lines, det_ok, yaw_smooth, offset_smooth, dist_z, vision_meta):
        if self._coarse_started is None or cfg.task_time_expired(now - self._coarse_started, cfg.COARSE_TOTAL_TIMEOUT_SEC):
            self._coarse_abort("coarse total timeout", lines)
            return
        if self.state == "COARSE_REACQUIRE":
            self._exec("STOP")
            if cfg.task_deadline_expired(now, self._state_deadline_mono):
                self._coarse_abort("coarse pallet reacquisition timeout", lines)
                return
            pose, _reason = self._observe_pose(det_ok, yaw_smooth, offset_smooth, dist_z, vision_meta, now)
            if pose is None:
                self._samples.clear()
                return
            center, margin = self._vision_values(vision_meta, pose)
            stable = self._stable_observation(pose, center, margin)
            if stable is not None:
                self._approve_stopped_observation(stable)
                self._trace_end(control_basis="coarse_complete_visual_reacquired")
                self._invalid_since_mono = None
                self._alignment_started_mono = None
                self._set_state("ACQUIRE_VERIFY", cfg.ACQUIRE_VERIFY_TIMEOUT_SEC)
            return
        if motion_deadline_expired():
            self._coarse_abort("coarse command lease expired", lines)
            return
        sample = self.imu_source.snapshot() if self.imu_source is not None else None
        warming = (self.state == "COARSE_PREPARE"
                   and self._coarse_imu_ready_mono is None)
        valid = (sample is not None and sample[1] is not None and not sample[3]
                 and all(map(math.isfinite, sample[:3]))
                 and 0 <= now - sample[1] <= cfg.COARSE_IMU_MAX_AGE_SEC)
        if (warming and not valid and (sample is None or not sample[3])
                and now - self._state_entered_mono < cfg.COARSE_IMU_STARTUP_TIMEOUT_SEC):
            self._exec("STOP")
            lines.append(("[COARSE] IMU starting; waiting for fresh gyro while stopped", COLOR_META))
            return
        if sample is None or sample[1] is None:
            self._coarse_abort("IMU unavailable", lines)
            return
        yaw, stamp, rate, error = sample
        if (error or not all(map(math.isfinite, (yaw, stamp, rate)))
                or not 0 <= now - stamp <= cfg.COARSE_IMU_MAX_AGE_SEC):
            self._coarse_abort(f"IMU invalid/stale: {error or 'sample age'}", lines)
            return
        if self.state == "COARSE_PREPARE":
            self._exec("STOP")
            if self._coarse_imu_ready_mono is None:
                self._coarse_imu_ready_mono = now
            guard_ready = getattr(self, '_coarse_brake_ready_mono', None)
            if guard_ready is None:
                # Conservative fallback for an unknown prior command history.
                guard_ready = self._coarse_imu_ready_mono + cfg.COARSE_IMU_POST_STOP_DELAY_SEC
            measurement_start = max(self._coarse_imu_ready_mono, guard_ready)
            if stamp < measurement_start:
                lines.append(("[COARSE] waiting for post-motion brake guard before gyro bias sampling", COLOR_META))
                return
            if stamp != self._coarse_bias_stamp:
                self._coarse_bias_samples.append(rate)
                self._coarse_bias_stamp = stamp
            if stamp - measurement_start < cfg.COARSE_SETTLE_SEC:
                return
            values = self._coarse_bias_samples
            minimum, maximum = min(values), max(values)
            spread = maximum - minimum
            reasons = []
            if len(values) < 10:
                reasons.append("insufficient_samples")
            if (cfg.COARSE_IMU_STABILITY_CHECK_ENABLED
                    and spread > cfg.COARSE_BIAS_MAX_RATE_SPREAD_DEG_S):
                reasons.append("rate_spread_exceeded")
            bias = median(values)
            logger = getattr(getattr(self, "tracer", None), "log_imu_bias_check", None)
            if logger is not None:
                logger(state=self.state, sample_count=len(values), min_samples=10,
                       rate_min_deg_s=minimum, rate_max_deg_s=maximum,
                       rate_spread_deg_s=spread,
                       max_rate_spread_deg_s=cfg.COARSE_BIAS_MAX_RATE_SPREAD_DEG_S,
                       median_bias_deg_s=bias, accepted=not reasons, reasons=reasons,
                       stability_check_enabled=cfg.COARSE_IMU_STABILITY_CHECK_ENABLED,
                       brake_guard_ready_mono=guard_ready,
                       imu_ready_mono=self._coarse_imu_ready_mono,
                       measurement_start_mono=measurement_start,
                       last_sample_mono=stamp)
            if reasons:
                self._coarse_abort(
                    "IMU bias measurement rejected: " + ", ".join(reasons)
                    + f" (n={len(values)}, min={minimum:.4f}, max={maximum:.4f}, "
                    + f"spread={spread:.4f}/{cfg.COARSE_BIAS_MAX_RATE_SPREAD_DEG_S:.4f} deg/s)",
                    lines)
                return
            self._coarse_bias = bias
            self._coarse_bias_origin = stamp
            self._coarse_turn(self._coarse_plan[0], sample, now, "COARSE_ROTATE")
            return
        if self.state in {"COARSE_ROTATE", "COARSE_RETURN_ROTATE"}:
            self._coarse_progress = self._coarse_yaw(sample) - self._coarse_ref
            sign = math.copysign(1.0, self._coarse_target)
            if sign * self._coarse_progress >= self._coarse_stop_deg:
                self._trace_end(actual_delta_deg=self._coarse_progress,
                                stop_threshold_deg=self._coarse_stop_deg,
                                stop_reason="imu stop threshold reached")
                self._coarse_settle("COARSE_ROTATE_SETTLE" if self.state == "COARSE_ROTATE"
                                    else "COARSE_RETURN_SETTLE", now)
            elif cfg.task_deadline_expired(now, self._state_deadline_mono) or sign * self._coarse_progress < -5.0:
                self._coarse_abort("coarse turn timeout or reversed IMU direction", lines)
            else:
                configure_motion_deadline(now + cfg.COARSE_COMMAND_LEASE_SEC)
                lines.append((f"[COARSE IMU] {self._coarse_progress:+.1f}/{self._coarse_target:+.1f} deg", COLOR_META))
            return
        if self.state == "COARSE_DRIVE":
            if now >= self._coarse_drive_end:
                self._trace_end(command_duration_sec=self._coarse_plan[2], actual_travel_m=None)
                self._coarse_settle("COARSE_DRIVE_SETTLE", now)
            else:
                configure_motion_deadline(min(now + cfg.COARSE_COMMAND_LEASE_SEC,
                                              self._coarse_drive_end + cfg.COARSE_COMMAND_LEASE_SEC))
                lines.append((f"[COARSE FWD] {self._coarse_plan[1]:.3f} m / {self._coarse_drive_end-now:.2f}s left (time estimate)", COLOR_META))
            return
        if self.state == "COARSE_ROTATE_SETTLE":
            # Fixed inertia delay after first-turn STOP. Keep this separate
            # from the quiet-rate/angle gates; the full drive starts afterward.
            self._exec("STOP")
            elapsed = now - self._state_entered_mono
            if elapsed < cfg.COARSE_IMU_POST_STOP_DELAY_SEC:
                lines.append((f"[COARSE STOP] inertia wait "
                              f"{elapsed:.2f}/{cfg.COARSE_IMU_POST_STOP_DELAY_SEC:.2f}s", COLOR_META))
                return
            self._coarse_drive_end = now + self._coarse_plan[2]
            self._set_state("COARSE_DRIVE")
            configure_motion_deadline(now + cfg.COARSE_COMMAND_LEASE_SEC)
            self._exec("FWD")
            self._trace_begin("v4_coarse_drive", "FWD", target_distance_m=self._coarse_plan[1],
                              fitted_duration_sec=self._coarse_plan[2], control_basis="fitted_time_no_vision")
            return

        if self.state == "COARSE_RETURN_SETTLE":
            self._exec("STOP")
            elapsed = now - self._state_entered_mono
            if elapsed < cfg.COARSE_IMU_POST_STOP_DELAY_SEC:
                lines.append((f"[COARSE RETURN STOP] inertia wait "
                              f"{elapsed:.2f}/{cfg.COARSE_IMU_POST_STOP_DELAY_SEC:.2f}s", COLOR_META))
                return
            self._pose_filter.reset()
            self._set_state("COARSE_REACQUIRE", cfg.COARSE_REACQUIRE_TIMEOUT_SEC)
            return

        self._exec("STOP")
        if not cfg.COARSE_IMU_STABILITY_CHECK_ENABLED:
            if now - self._state_entered_mono < cfg.COARSE_IMU_POST_STOP_DELAY_SEC:
                return
            if self.state == "COARSE_DRIVE_SETTLE":
                self._coarse_turn(self._coarse_plan[3], sample, now, "COARSE_RETURN_ROTATE")
            return
        if abs(rate - self._coarse_bias) <= cfg.COARSE_STABLE_RATE_DEG_S:
            if self._coarse_last_quiet is None:
                self._coarse_last_quiet = now
        else:
            self._coarse_last_quiet = None
        if cfg.task_deadline_expired(now, self._state_deadline_mono):
            self._coarse_abort("coarse stop did not settle", lines)
            return
        if self._coarse_last_quiet is None or stamp - self._coarse_last_quiet < max(
                cfg.COARSE_SETTLE_SEC, cfg.COARSE_IMU_POST_STOP_DELAY_SEC):
            return
        if self.state == "COARSE_DRIVE_SETTLE":
            self._coarse_turn(self._coarse_plan[3], sample, now, "COARSE_RETURN_ROTATE")
