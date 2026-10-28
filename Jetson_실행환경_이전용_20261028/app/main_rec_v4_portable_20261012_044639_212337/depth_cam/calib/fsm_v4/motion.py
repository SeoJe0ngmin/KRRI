"""Time conversions and model-based endpoint helpers for FSM v4."""

from __future__ import annotations

import math

from . import config as cfg


def drive_seconds(distance_m: float, *, apply_max_limit: bool = True) -> float:
    """Inverse of the selected forward ON-command delayed-linear fit."""

    effective = max(
        0.0,
        cfg.FWD_DISTANCE_SCALE * abs(float(distance_m)) + cfg.FWD_DISTANCE_BIAS_M,
    )
    seconds = cfg.FWD_T0_SEC + effective / cfg.FWD_SLOPE_M_S
    if apply_max_limit:
        seconds = min(cfg.FWD_COMMAND_MAX_SEC, seconds)
    return max(cfg.FWD_COMMAND_MIN_SEC, seconds)


def forward_seconds(distance_m: float) -> float:
    return drive_seconds(distance_m)


def backward_seconds(distance_m: float) -> float:
    """Preserve the previous backward conversion: no backward data was refitted."""
    effective = max(0., cfg.FWD_DISTANCE_SCALE * abs(float(distance_m)) + cfg.FWD_DISTANCE_BIAS_M)
    a, ta = cfg.FWD_ACCEL_M_S2, cfg.FWD_ACCEL_DURATION_SEC
    da = .5 * a * ta * ta
    seconds = cfg.BACK_LEGACY_T0_SEC + (math.sqrt(2*effective/a) if effective <= da
                                      else ta + (effective-da)/(a*ta))
    return min(cfg.BACK_MAX_COMMAND_SEC, max(cfg.FWD_COMMAND_MIN_SEC, seconds))


def fitted_forward_distance(elapsed_sec: float) -> float:
    """Displacement while FWD is held; excludes motion after STOP."""
    t = float(elapsed_sec)
    if not math.isfinite(t) or t < 0:
        raise ValueError('Elapsed time must be finite and nonnegative')
    return cfg.FWD_SLOPE_M_S * max(t-cfg.FWD_T0_SEC, 0.)


def fitted_forward_speed(elapsed_sec: float) -> float:
    """Expected forward speed at elapsed time after the FWD CAN write."""

    return cfg.FWD_SLOPE_M_S if float(elapsed_sec) > cfg.FWD_T0_SEC else 0.


def bounded_forward_lookahead(
    elapsed_sec: float,
    measured_speed_m_s: float,
    horizon_sec: float,
) -> tuple[float, float]:
    """Return bounded predictive advance and the speed actually used.

    A one-frame PnP jump must not turn into 0.7--0.8 m of fictitious travel.
    The measured speed is capped by the fitted response envelope, and the
    complete lookahead (latency + coast) is capped independently.
    """

    fitted_limit = (
        fitted_forward_speed(elapsed_sec)
        * cfg.FWD_PREDICTIVE_SPEED_MULTIPLIER
    )
    used_speed = min(max(0.0, float(measured_speed_m_s)), fitted_limit)
    raw_advance = (
        used_speed * max(0.0, float(horizon_sec))
        + cfg.FWD_COAST_ALLOWANCE_M
    )
    advance = min(raw_advance, cfg.FWD_PREDICTIVE_MAX_ADVANCE_M)
    return advance, used_speed


def forward_predictive_stop_ready(travelled_m: float, target_m: float) -> bool:
    """Whether enough real displacement exists to allow predictive STOP."""

    target = max(0.0, float(target_m))
    if target <= 0.0:
        return True
    return max(0.0, float(travelled_m)) >= (
        target * cfg.FWD_PREDICTIVE_MIN_PROGRESS_RATIO
    )


def measurement_age(vision_meta: dict | None, now_mono: float) -> float:
    if vision_meta:
        try:
            measured = float(vision_meta["measurement_mono"])
            inference = max(0.0, now_mono - measured)
            return inference + cfg.SENSOR_PIPELINE_LATENCY_SEC
        except (KeyError, TypeError, ValueError):
            pass
    return cfg.FALLBACK_INFERENCE_LATENCY_SEC + cfg.SENSOR_PIPELINE_LATENCY_SEC
