"""Nominal tag correction with an independently preserved error envelope.

This is a bounded-gain complementary update, not an EKF or full VIO. Quality
scores choose a nominal correction only; they are not measured covariances.
The tag and VO share imagery, so repeated observations never earn a 1/sqrt(N)
reduction in a configured error bound.
"""
import math


def wrap_near(angle, reference):
    return reference + (angle-reference+math.pi) % (2*math.pi)-math.pi


def validate_estimation(sensor, live=False):
    if not isinstance(sensor.get('odom', {}).get('gyro_aided_tracking', False), bool):
        raise ValueError('odom.gyro_aided_tracking must be boolean')
    c = sensor.get('tag_fusion', {})
    if not isinstance(c, dict):
        raise ValueError('tag_fusion must be an object')
    if not isinstance(c.get('enabled', False), bool):
        raise ValueError('tag_fusion.enabled must be boolean')
    if c.get('enabled'):
        gains = c.get('max_tag_weight')
        if (not isinstance(gains, list) or len(gains) != 3 or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                    not math.isfinite(v) or not 0 < v <= 1 for v in gains)):
            raise ValueError('tag_fusion.max_tag_weight needs three values in (0, 1]')
    fps = sensor['camera']['fps']
    if isinstance(fps, bool) or not isinstance(fps, (int, float)) or not math.isfinite(fps) or fps <= 0:
        raise ValueError('camera.fps must be finite and positive')


def tag_quality(tag, config):
    """Dimensionless tuning heuristic, never an accuracy certificate.

    Use the existing acceptance scales; missing quality metadata gets the
    neutral score 1. Accepted-but-marginal detections get smaller corrections.
    Geometric ambiguity still has to pass the independent Tag.update gate.
    """
    terms = []
    for name, scale, larger_better in (
            ('min_edge_px', config.get('min_tag_px'), True),
            ('margin', config.get('min_margin'), True),
            ('reproj_px', config.get('max_reproj_px'), False),
            ('ambiguity_ratio', config.get('max_ambiguity_ratio'), False)):
        value = tag.get(name)
        if value is None or scale is None:
            continue
        if not math.isfinite(value) or value < 0:
            return 0.
        if scale <= 0:
            continue
        terms.append(value/(value+scale) if larger_better else scale/(scale+value))
    return min(terms) if terms else 1.


def fuse_tag(prior, prior_error, measured, measured_error, tag, sensor):
    """Return a nominal pose and a radius containing the FULL tag box.

    If z +/- e contains the true pose, then any chosen centre x with radius
    e + abs(z-x) also contains it. No sensor independence or Gaussian noise
    assumption is needed. This keeps smoothing lag inside the safety check.
    Missing measured bounds remain missing; they are never inferred from jitter.
    """
    c = sensor.get('tag_fusion', {})
    z = list(measured)
    if prior is not None:
        z[2] = wrap_near(z[2], prior[2])
    if prior is None or not c.get('enabled'):
        return z, None if measured_error is None else list(measured_error), {
            'reason': 'initialized_from_tag' if c.get('enabled') else 'disabled',
            'raw_tag_pose': z, 'tag_weight': [1., 1., 1.],
            'recenter_padding': [0., 0., 0.], 'statistical_covariance': False}
    quality = tag_quality(tag, sensor['tag'])
    weights = []
    for i, limit in enumerate(c['max_tag_weight']):
        weight = limit
        if prior_error is not None and measured_error is not None:
            total = prior_error[i]+measured_error[i]
            weight = min(limit, prior_error[i]/total) if total > 0 else 0.
        weights.append(weight*quality)
    residual = [b-a for a, b in zip(prior, z)]
    result = [a+w*d for a, w, d in zip(prior, weights, residual)]
    padding = [abs(a-b) for a, b in zip(result, z)]
    error = None if measured_error is None else [e+p for e, p in zip(measured_error, padding)]
    return result, error, {'reason': 'weighted_tag_update', 'raw_tag_pose': z,
        'prior_pose': list(prior), 'innovation': residual, 'tag_weight': weights,
        'quality_score': quality, 'recenter_padding': padding,
        'statistical_covariance': False}


def drift_budget(bounds, distance, angle, frames, elapsed, camera_fps):
    """Every frame/time interval contributes, including rest and visible tags.

    Use at least elapsed * configured FPS exposure intervals. Dropping work
    must not buy a smaller error budget. This reuses the existing per-frame
    envelope, not a newly fitted time/noise coefficient. Its physical coverage
    still needs validation at the configured rate and expected motion/vibration.
    Missing/null coefficients mean unknown, not zero.
    """
    values = (('distance', 'odom_error_per_m', distance),
              ('rotation', 'odom_error_per_rad', angle),
              ('frames', 'odom_error_per_frame', frames),
              ('unprocessed_intervals', 'odom_error_per_frame', max(0., elapsed*camera_fps-frames)))
    terms = {}
    for label, key, amount in values:
        coefficient = bounds.get(key)
        terms[label] = None if coefficient is None else [v*amount for v in coefficient]
    total = None if any(v is None for v in terms.values()) else [
        sum(v[i] for v in terms.values()) for i in range(3)]
    return total, terms
