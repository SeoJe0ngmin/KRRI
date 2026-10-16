"""Keep host timing separate from site geometry and trial objectives."""
import copy
import json
import math
from .configuration import resolve_config


def merge(base, overlay):
    out = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def build_trial(cfg, task):
    c = copy.deepcopy(cfg)
    c['validation']['approved'] = False
    spec = c['trial']
    spec['kind'] = task
    p = c['prealignment']
    front = c['body']['front_m']
    needed = [spec.get('forward_tolerance_m'), spec.get('lateral_tolerance_m'), spec.get('yaw_tolerance_rad')]
    if task == 'wall_stop':
        p['enabled'] = False
        needed += [spec.get('standoff_m'), spec.get('tag_plane_x_m')]
    elif task == 'cone_entry':
        p['enabled'] = True
        needed += [spec.get('insertion_front_m'), p.get('entry_x_m'), c['body'].get('rear_m'), c['body'].get('half_width_m')]
    else:
        raise ValueError('unknown trial: '+task)
    if any(v is None for v in needed):
        c['goal'] = None
        return c  # Editable bench profile, clearly incomplete for driving.
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in needed):
        raise ValueError('trial dimensions must be finite numbers')
    ex, ey, eh = needed[:3]
    if min(ex, ey, eh) <= 0:
        raise ValueError('trial tolerances must be positive')
    if task == 'wall_stop':
        if spec['standoff_m'] <= ex:
            raise ValueError('wall standoff must exceed longitudinal tolerance')
        x = spec['tag_plane_x_m']-spec['standoff_m']-front
        # Stop before a tag plane; no cargo-bay crossing constraint applies.
        p['enabled'] = False
    else:
        x = p['entry_x_m']+spec['insertion_front_m']-front
        if x-ex-c['body']['rear_m'] < p['entry_x_m']:
            raise ValueError('cone stop depth does not put the complete vehicle inside')
    y = p['center_y_m']
    c['goal'] = {'lo': [x-ex, y-ey, -eh], 'hi': [x+ex, y+ey, eh]}
    if task == 'cone_entry':
        from .core import Box, body_aabb
        extent = body_aabb(Box(c['goal']['lo'], c['goal']['hi']), c['body'])
        if extent[0] < p['entry_x_m']:
            raise ValueError('cone stop depth does not contain rotated vehicle and goal uncertainty')
    return c


def prepare(base, site_path, task):
    # Deliberately do not accept platform timing/CAN overrides via a site file.
    with open(site_path) as f:
        site = json.load(f)
    if '_shared' in site:
        from .shared import manager
        site = manager().load(site_path, site)
    allowed = {'site', 'body', 'scene', 'grid', 'trial', 'prealignment', 'visibility', 'sensor', 'startup_search'}
    if set(site)-allowed:
        raise ValueError('site file contains host/control fields: '+','.join(sorted(set(site)-allowed)))
    if set(site.get('sensor', {}))-{'rig', 'calibration'}:
        raise ValueError('site.sensor may only supply rig/calibration geometry')
    if set(site.get('startup_search', {}))-{'free_radius_m'}:
        raise ValueError('site.startup_search may only supply initial free radius')
    return build_trial(resolve_config(merge(base, site)), task)


def readiness(cfg):
    """Actionable list of remaining inputs, without touching camera/CAN."""
    from .core import validate
    from .runtime import check_sensor
    from .motion import validate_motion
    from .canhealth import validate_diagnostics
    missing = []
    unverified = []
    def walk(value, path):
        if isinstance(value, dict):
            for key, child in value.items():
                if child is None:
                    missing.append(path+key)
                elif isinstance(child, (dict, list)):
                    walk(child, path+key+'.')
        elif isinstance(value, list):
            for i, child in enumerate(value):
                if child is None:
                    missing.append(path+str(i))
    # Only required sections: optional exposure/serial/gain nulls are valid.
    for section in ('body', 'scene', 'grid', 'goal', 'actions', 'observation_error', 'limits'):
        if cfg.get(section) is None:
            missing.append(section)
        else:
            walk(cfg[section], section+'.')
    sensor = cfg.get('sensor', cfg)
    walk(sensor['bounds'], 'sensor.bounds.')
    for section in ('stop_window', 'tracking_recovery'):
        options = sensor.get(section, {})
        if options.get('enabled'):
            walk(options, 'sensor.'+section+'.')
            if not options.get('verified'):
                unverified.append('sensor.'+section+'.verified')
    if sensor['imu'].get('stationary_recalibration', {}).get('enabled'):
        walk(sensor['imu']['stationary_recalibration'], 'sensor.imu.stationary_recalibration.')
        if not sensor['imu']['stationary_recalibration'].get('verified'):
            unverified.append('sensor.imu.stationary_recalibration.verified')
    for section in ('bounds', 'calibration'):
        if not sensor[section].get('verified'):
            unverified.append('sensor.'+section+'.verified')
    if not sensor['imu'].get('yaw_sign_verified'):
        unverified.append('sensor.imu.yaw_sign_verified')
    if sensor['imu'].get('max_age_s') is None:
        missing.append('sensor.imu.max_age_s')
    for key in ('T_body_camera', 'T_dock_tag'):
        if sensor['calibration'].get(key) is None:
            missing.append('sensor.calibration.'+key)
    if cfg.get('visibility', {}).get('enabled'):
        if sensor['calibration'].get('camera_model') is None:
            missing.append('sensor.calibration.camera_model')
        if cfg['visibility'].get('projection_margin_px') is None:
            missing.append('visibility.projection_margin_px')
    if cfg.get('prealignment', {}).get('enabled'):
        for key in ('corridor_half_width_m', 'staging_x_range_m', 'alignment_y_m', 'alignment_yaw_rad'):
            if cfg['prealignment'].get(key) is None:
                missing.append('prealignment.'+key)
    if cfg.get('startup_search', {}).get('enabled'):
        walk(cfg['startup_search'], 'startup_search.')
    if cfg.get('recovery', {}).get('enabled'):
        options = cfg['recovery'].get('blind_return', {})
        if options.get('enabled'):
            walk(options, 'recovery.blind_return.')
            if not options.get('verified'):
                unverified.append('recovery.blind_return.verified')
    diagnostics = cfg.get('can', {}).get('diagnostics', {})
    if diagnostics.get('enabled'):
        walk(diagnostics, 'can.diagnostics.')
        if not diagnostics.get('verified'):
            unverified.append('can.diagnostics.verified')
    if 'sensor' in cfg:
        evidence = ['geometry', 'extrinsics', 'vision_error', 'motion_contracts', 'timing', 'stop_detection']
        evidence += [name for name in ('recovery', 'startup_search') if cfg.get(name, {}).get('enabled')]
        for name in evidence:
            if not cfg.get('validation', {}).get(name):
                missing.append('validation.'+name)
        if not cfg.get('validation', {}).get('approved'):
            unverified.append('validation.approved')
        deployment = cfg.get('deployment', {})
        if deployment.get('platform') == 'ubuntu_vm' and not deployment.get('timing_verified_on_this_host'):
            unverified.append('deployment.timing_verified_on_this_host')
    errors = []
    if 'sensor' in cfg:
        try:
            validate(cfg, live=True)
        except (ValueError, KeyError, TypeError) as e:
            errors.append(str(e))
    try:
        check_sensor(sensor, live=True)
    except (ValueError, KeyError, TypeError) as e:
        errors.append(str(e))
    if 'sensor' in cfg:
        for check in (lambda: validate_motion(sensor, cfg.get('limits'), live=True),
                      lambda: validate_diagnostics(cfg.get('can', {}), live=True)):
            try:
                check()
            except (ValueError, KeyError, TypeError) as e:
                errors.append(str(e))
    return {'drive_ready': 'sensor' in cfg and not (errors or missing or unverified),
            'missing_inputs': sorted(set(missing)), 'unverified_inputs': sorted(set(unverified)),
            'validation_errors': errors, 'hardware_opened': False}
