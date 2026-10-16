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



from .frames import R_OPTICAL_TO_BODY, R_REFERENCE_TAG
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


def measurement_window(rows, seconds):
    """Accumulate confirmed time only; never count settling or cross blind gaps."""
    total=0.; first=len(rows)-1
    for i in range(len(rows)-1,0,-1):
        a,b=rows[i-1],rows[i];dt=b['arrival']-a['arrival']
        if a.get('stationary') and b.get('stationary') and 0<dt<=.4:
            total+=dt
        first=i-1
        if total>=seconds:break
    return rows[first:],total


def settling_is_observed(window):
    """Allow only short, visually observed rechecks with unchanged tag pose."""
    for i,row in enumerate(window):
        if row.get('stationary'):continue
        if (row.get('stationary_report',{}).get('reason')!='settling_window'
                or not row.get('sensor_ready') or not row.get('local_valid')):return False
        left=i-1
        while left>=0 and not window[left].get('stationary'):left-=1
        right=i+1
        while right<len(window) and not window[right].get('stationary'):right+=1
        if left<0 or right==len(window):return False
        segment=window[left:right+1]
        if segment[-1]['arrival']-segment[0]['arrival']>3.:return False
        for key in ('sensor_epoch','motion_epoch'):
            if len({x.get(key) for x in segment})!=1:return False
        if any(b['arrival']-a['arrival']>.4 for a,b in zip(segment,segment[1:])):return False
        tags=[x.get('tag',{}) for x in segment]
        if any(not t.get('ok') or t.get('reason')!='accepted' for t in tags):return False
        try:
            xyz=np.asarray([t['camera_translation_m'] for t in tags],float)
            rotations=[cv2.Rodrigues(np.asarray(t['camera_rotation_vector_rad'],float))[0] for t in tags]
            if xyz.shape!=(len(tags),3) or not np.isfinite(xyz).all():return False
            if np.max(np.linalg.norm(xyz-xyz[0],axis=1))>.01:return False
            if any(_angle(x,rotations[0])>.5 for x in rotations):return False
        except (KeyError,ValueError,cv2.error):return False
    return True


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
    window,confirmed_s=measurement_window(rows,seconds)
    end,start=window[-1]['arrival'],window[0]['arrival']
    report['confirmed_stationary_s']=confirmed_s
    report['elapsed_window_s']=end-start
    if confirmed_s<seconds:reasons.append('measurement_too_short')
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
    if not settling_is_observed(window):
        reasons.append('visual_stationarity_not_confirmed')
    from collections import Counter
    report['stationarity_reasons'] = dict(Counter(
        r.get('stationary_report', {}).get('reason', 'not_reported') for r in window))
    report['stationary_fraction'] = sum(bool(r.get('stationary')) for r in window)/len(window)
    rates=[r.get('vio_gyro_norm',float('inf')) for r in window]
    if any(v is None or not math.isfinite(v) for v in rates):
        reasons.append('gyro_quality_missing')
    elif np.percentile(rates,90)>sensor['uncertainty']['still_gyro_norm_rps']:
        reasons.append('rotation_during_measurement')

    bins, accepted, stamps, positions = {}, [], [], []
    attempts = sum(r.get('tag', {}).get('reason') not in ('not_scheduled', None) for r in window)
    seen=set()
    for row in window:
        if not row.get('stationary'):continue
        tag = row.get('tag', {})
        if not tag.get('ok') or tag.get('reason') != 'accepted':
            continue
        key=tag.get('capture_key')
        if key is not None:
            if key in seen:continue
            seen.add(key)
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
    # Settling tags validate continuity, but are excluded from the fitted mount.
    observed_stamps=[r['arrival'] for r in window if r.get('tag',{}).get('ok')
                     and r.get('tag',{}).get('reason')=='accepted']
    max_gap = max(np.diff([start, *observed_stamps, end]))
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


