"""Camera mounting orientation from an operator-established reference pose.

The vehicle must be level and normal to an upright, level tag plane. This
assumption is not observable from a stationary camera. No CAN or old controller
is used. Repeatability checks below are NOT absolute pose-error certificates.
"""
import copy
import datetime
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import uuid

import numpy as np
import cv2



# Current Tag.update/IPPE corner convention, not the old apriltag_v4 frame.
R_OPTICAL_TO_BODY = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
R_REFERENCE_TAG = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
SCHEMA = 'camera_mount_reference_v1'
LIMITS = dict(min_accepted_hz=2., min_coverage=.8, max_tag_gap_s=2.,
              block_spread_p95_deg=1., half_change_deg=.5,
              translation_spread_p95_m=.05, max_mount_angle_deg=30.)


def _project_rotation(matrix):
    u, _, vt = np.linalg.svd(matrix)
    return u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt


def _angle(a, b):
    return math.degrees(math.acos(float(np.clip((np.trace(a @ b.T)-1.)/2., -1., 1.))))


def _centre(rotations):
    return _project_rotation(np.median(np.asarray(rotations), axis=0))


def estimate(rows, sensor, camera_model, seconds=60.):
    """Analyse the last requested interval; old mounting transforms are unused."""
    if not math.isfinite(seconds) or not 30 <= seconds <= 180:
        raise ValueError('measurement interval must be 30..180 seconds')
    report = dict(schema=SCHEMA, id=uuid.uuid4().hex, usable=False,
                  method='upright_tag_vehicle_level_and_normal_to_plane',
                  reference_alignment_confirmed=True, certified_error_bound=False,
                  limits=LIMITS.copy(), reasons=[], seconds=seconds,
                  camera_model=copy.deepcopy(camera_model),
                  tag={k: sensor['tag'][k] for k in ('id', 'size_m')})
    reasons = report['reasons']
    if not rows:
        reasons.append('no_observations')
        return report
    times = np.array([r['arrival'] for r in rows], float)
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        reasons.append('invalid_observation_clock')
        return report
    end, start = times[-1], times[-1]-seconds
    if times[-1]-times[0] < seconds:
        reasons.append('measurement_too_short')
    window = [r for r in rows if r['arrival'] >= start]
    if any(r.get('clock_fault') for r in window):
        reasons.append('sensor_clock_fault')
    if any(r.get('imu', {}).get('calibration', {}).get('state') != 'READY' for r in window):
        reasons.append('imu_not_ready_throughout_measurement')
    imus = [r.get('imu', {}) for r in window]
    if any(i.get('age_s') is None or i['age_s'] > .5 for i in imus):
        reasons.append('imu_stale')
    if len({i.get('gaps') for i in imus}) != 1 or any(i.get('clock_reversals', 0) for i in imus):
        reasons.append('imu_gap_or_clock_reversal')
    if any(r.get('recording', {}).get('error') or r.get('recording', {}).get('dropped', 0) for r in window):
        reasons.append('incomplete_recording')
    # OpenVINS gyro bias and visual stationarity must agree; no IMU-only
    # zero-velocity claim and no instruction/command-derived stationarity.
    if any(not r.get('stationary') for r in window):
        reasons.append('visual_stationarity_not_confirmed')
    rates=[r.get('vio_gyro_norm',float('inf')) for r in window]
    if any(v is None or not math.isfinite(v) for v in rates):
        reasons.append('gyro_quality_missing')
    elif np.percentile(rates,90)>sensor['uncertainty']['still_gyro_norm_rps']:
        reasons.append('rotation_during_measurement')

    bins, accepted, stamps, positions = {}, [], [], []
    attempts = sum(r.get('tag', {}).get('reason') not in ('not_scheduled', None) for r in window)
    for row in window:
        tag = row.get('tag', {})
        if not tag.get('ok') or tag.get('reason') != 'accepted':
            continue
        rv, xyz = np.asarray(tag.get('camera_rotation_vector_rad'), float), np.asarray(tag.get('camera_translation_m'), float)
        if rv.shape != (3,) or xyz.shape != (3,) or not np.isfinite([rv, xyz]).all():
            if 'invalid_tag_pose' not in reasons:
                reasons.append('invalid_tag_pose')
            continue
        R = R_REFERENCE_TAG @ cv2.Rodrigues(rv)[0].T
        accepted.append(R)
        positions.append(xyz)
        stamps.append(row['arrival'])
        bins.setdefault(int(row['arrival']-start), []).append((R, xyz))
    report.update(tag_attempts=attempts, accepted=len(accepted))
    if len(accepted) < seconds*LIMITS['min_accepted_hz']:
        reasons.append('too_few_accepted_tags')
    if not accepted:
        return report
    max_gap = max(np.diff([start, *stamps, end]))
    blocks = [v for v in bins.values() if len(v) >= 2]
    report.update(covered_seconds=len(blocks), max_tag_gap_s=float(max_gap))
    if len(blocks) < seconds*LIMITS['min_coverage'] or max_gap > LIMITS['max_tag_gap_s']:
        reasons.append('insufficient_tag_time_coverage')
    if len(blocks) < 4:
        return report
    # Each one-second block has one vote; a burst of frames cannot dominate.
    rotations = [_centre([x[0] for x in block]) for block in blocks]
    R = _centre(rotations)
    spread = [_angle(r, R) for r in rotations]
    delta = R @ R_OPTICAL_TO_BODY.T
    yaw = math.degrees(math.atan2(delta[1, 0], delta[0, 0]))
    pitch = math.degrees(math.atan2(-delta[2, 0], math.hypot(delta[0, 0], delta[1, 0])))
    roll = math.degrees(math.atan2(delta[2, 1], delta[2, 2]))
    half = len(rotations)//2
    change = _angle(_centre(rotations[:half]), _centre(rotations[half:]))
    centres = np.array([np.median([x[1] for x in block], axis=0) for block in blocks])
    position_spread = float(np.percentile(np.linalg.norm(centres-np.median(centres, axis=0), axis=1), 95))
    report.update(R_body_camera=R.tolist(), yaw_left_deg=yaw, pitch_down_deg=pitch,
                  roll_deg=roll, block_spread_p95_deg=float(np.percentile(spread, 95)),
                  first_last_half_change_deg=change, translation_spread_p95_m=position_spread)
    if report['block_spread_p95_deg'] > LIMITS['block_spread_p95_deg'] or change > LIMITS['half_change_deg']:
        reasons.append('orientation_unstable')
    if position_spread > LIMITS['translation_spread_p95_m']:
        reasons.append('tag_translation_unstable')
    if _angle(R, R_OPTICAL_TO_BODY) > LIMITS['max_mount_angle_deg']:
        reasons.append('check_reference_pose_or_tag_orientation')
    report['usable'] = not reasons
    return report


