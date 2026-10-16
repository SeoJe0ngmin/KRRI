"""Bounded observation recovery; never clears CAN/operator safety latches."""
import math
import copy
from .core import finite, transition


def validate_recovery(cfg, live=False):
    r = cfg.get('recovery', {})
    if not r.get('enabled'):
        return
    for k in ('timeout_s', 'stable_s', 'max_sample_gap_s', 'max_attempts', 'min_samples',
              'min_tag_samples', 'pause_error_ratio', 'resume_error_ratio',
              'return_heading_tolerance_rad', 'max_return_actions'):
        if not finite([r.get(k)]) or r[k] <= 0:
            raise ValueError('invalid recovery.'+k)
    for k in ('max_attempts', 'min_samples', 'min_tag_samples', 'max_return_actions'):
        if not isinstance(r[k], int) or isinstance(r[k], bool):
            raise ValueError('recovery count must be an integer: '+k)
    if not 0 < r['resume_error_ratio'] < r['pause_error_ratio'] <= 1:
        raise ValueError('recovery requires hysteresis below the observation budget')
    if r['stable_s'] >= r['timeout_s'] or min(r['min_samples'], r['min_tag_samples']) < 2:
        raise ValueError('invalid recovery stability window')
    if r['return_heading_tolerance_rad'] >= math.pi/2:
        raise ValueError('return heading tolerance must be below 90 degrees')
    if live and not cfg.get('validation', {}).get('recovery'):
        raise ValueError('missing validation evidence: recovery')
    blind = r.get('blind_return', {})
    if blind.get('enabled'):
        if live and not blind.get('verified'):
            raise ValueError('blind_return needs verified STOP/rest and geometry evidence')
        for k in ('stop_rest_after_s', 'rest_speed_upper_mps', 'max_heading_error_rad'):
            if blind.get(k) is None and not live and not blind.get('verified'):
                continue
            if not finite([blind.get(k)]) or blind[k] <= 0:
                raise ValueError('invalid blind_return.'+k)
        if blind.get('verified') and blind['rest_speed_upper_mps'] > cfg['limits']['stop_speed_mps']:
            raise ValueError('blind_return rest speed exceeds stop criterion')


def return_config(cfg):
    """Qualified return may use coarse geometry without promising visual texture.

    All body, clearance, entry and action sweep checks remain unchanged.
    This permission applies only to returning, never ordinary translations.
    """
    c = cfg.get('recovery', {}).get('blind_return', {})
    if not (c.get('enabled') and c.get('verified')):
        return cfg
    out = copy.deepcopy(cfg)
    out['visibility'] = dict(out.get('visibility', {}), enabled=False)
    return out


def tag_heading(box, cfg, fallback):
    cal = cfg.get('sensor', {}).get('calibration', {})
    bc, dt = cal.get('T_body_camera'), cal.get('T_dock_tag')
    if not cal.get('verified') or bc is None or dt is None:
        return fallback
    # Recompute after lateral displacement; the old heading need not see the tag.
    x, y, h = box.center
    x += math.cos(h)*bc[0][3]-math.sin(h)*bc[1][3]
    y += math.sin(h)*bc[0][3]+math.cos(h)*bc[1][3]
    axis = math.atan2(bc[1][2], bc[0][2])
    desired = math.atan2(dt[1][3]-y, dt[0][3]-x)-axis
    return h+(desired-h+math.pi) % (2*math.pi)-math.pi


def return_space_available(start, action, cfg):
    """Reserve a full turn anywhere in this leg, including return drift/STOP.

    This conservative disk-like AABB is independent of lost image features.
    It requires site free-space evidence; a cone boundary is not a sensor.
    """
    from .core import Box, action_box, safe
    if not cfg.get('recovery', {}).get('reserve_return_sweep'):
        return True
    rotations = [a for a in cfg['actions'] if a['kind'] == 'rotate']
    if not rotations:
        return False
    envelope = action_box(start, action, (0., 1.), True)
    count = cfg['recovery']['max_return_actions']
    margin = [count*max(a['sweep_error'][i] for a in rotations) for i in (0, 1)]
    reserve = Box([envelope.lo[0]-margin[0], envelope.lo[1]-margin[1], -math.pi],
                  [envelope.hi[0]+margin[0], envelope.hi[1]+margin[1], math.pi])
    return safe(reserve, return_config(cfg))


def return_observation(ctl, now, obs):
    """Coarse return-only tube when visual tracking cannot confirm a stop.

    This is explicitly model-bound evidence, NEVER relocalization or visual
    stillness. Requires a measured STOP-to-rest upper bound, continuous gyro,
    and an already checked full action/STOP envelope. Cannot restart driving.
    """
    c = ctl.recovery.get('blind_return', {})
    ref, tube = ctl.return_reference, ctl.pause_envelope
    if not (c.get('enabled') and c.get('verified') and ctl.phase == 'SIDESTEP' and
            ctl.recovery_started is not None and ctl.return_allowed and ref and tube and
            (not obs.get('bounded') or obs.get('tracking_recovery', {}).get('coarse') or
             obs.get('tracking_recovery', {}).get('predicting'))):
        return obs
    imu = obs.get('capture_imu') or {}
    old = ref['imu']
    if (not finite([imu.get('yaw_rad'), imu.get('device_stamp_s'), imu.get('gaps')]) or
            imu['gaps'] != old['gaps'] or imu['device_stamp_s'] != obs.get('capture_id') or
            imu['device_stamp_s'] < old['device_stamp_s']):
        return obs
    from .core import Box, action_box
    if ctl.returning and ctl.action is not None:
        tube = action_box(ctl.start_box, ctl.action, (0., 1.), True)
    dt = imu['device_stamp_s']-old['device_stamp_s']
    h = ref['pose'][2]+imu['yaw_rad']-old['yaw_rad']
    rate_error = ctl.cfg['sensor']['bounds'].get('gyro_rate_error_rps')
    if not finite([rate_error]) or rate_error < 0:
        return obs
    e = ref['error'][2]+rate_error*dt
    age = max(0., now-obs['capture_lower'])
    if e+ctl.cfg['limits']['max_yaw_rate_rps']*age > c['max_heading_error_rad']:
        return obs
    b = Box([tube.lo[0], tube.lo[1], h-e], [tube.hi[0], tube.hi[1], h+e])
    out = dict(obs, pose=b.center, error=b.radius, bounded=True, valid=True,
               return_only=True, anchor_stamp=ref['anchor_stamp'], motion_bounded=False, speed_upper=None,
               stop_motion=dict(bounded=False, reason='no_visual_stop_evidence'))
    w = obs.get('yaw_rate_upper')
    out['qualified_return_stop'] = bool(ctl.command == 'stop' and ctl.braked is not None and
        now-ctl.braked >= c['stop_rest_after_s'] and finite([w]) and
        0 <= w <= ctl.cfg['limits']['stop_yaw_rate_rps'] and ctl.angular_stop_ok is not False)
    out['stop_proof'] = 'qualified_STOP_to_rest_bound_not_visual'
    return out


def within_budget(box, cfg, ratio=1.):
    return all(e <= bound*ratio for e, bound in zip(box.radius, cfg['observation_error']))


def return_rotation(box, heading, cfg):
    """Select an existing, checked rotation toward the pre-sidestep heading.

    Never invent a timed 90-degree command or change a motion contract's angle.
    All intermediate poses (including a STOP) must remain safe. A qualified
    coarse return widens the angular contract; a completed gyro turn is not
    an absolute heading measurement.
    """
    target = box.center[2]+(heading-box.center[2]+math.pi) % (2*math.pi)-math.pi
    before = max(abs(box.lo[2]-target), abs(box.hi[2]-target))
    if before <= cfg['recovery']['return_heading_tolerance_rad']:
        return None
    blind = cfg['recovery'].get('blind_return', {})
    angular_pad = 0.
    if blind.get('enabled') and blind.get('verified'):
        cap = blind['max_heading_error_rad']
        if box.radius[2] > cap:
            return None
        # The upper-bound gyro cutoff can brake up to 2*error early in the
        # actual heading. Widen, never reuse a precision endpoint certificate
        # outside its original observation budget. CAN/target/reserve stay fixed.
        angular_pad = 2*cap
    candidates = []
    for i, original in enumerate(cfg['actions']):
        action = original
        if action['kind'] != 'rotate' or action['target']*(target-box.center[2]) <= 0:
            continue
        if angular_pad:
            action = copy.deepcopy(original)
            for field in ('terminal_error', 'sweep_error'):
                action[field][2] += angular_pad
            action['return_yaw_inflation_rad'] = angular_pad
        end = transition(box, action, cfg)
        if end is None:
            continue
        after = max(abs(end.lo[2]-target), abs(end.hi[2]-target))
        # Require progress even after the next measurement uncertainty is added.
        if after+cfg['observation_error'][2] < before-1e-9:
            candidates.append((after, i, action, end))
    if not candidates:
        return None
    _, _, action, end = min(candidates, key=lambda x: x[:2])
    return action, end
