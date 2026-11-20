"""Explicit D435/D435i exposure controls, independent of vision dependencies.

Configuration uses microseconds. D435 RGB option ticks are 100 us; stereo
IR/depth option ticks are 1 us. Frame metadata has its own SDK-defined units
and must NOT be passed through this option-unit conversion.
"""
import math


def recovery_camera_settings(camera, request, control, now, last_request):
    """AE fallback only on a fresh, explicit startup request while neutral."""
    if not camera.get('allow_startup_auto_exposure') or request is None or control is None:
        return None
    nonce, requested = request
    issued, neutral, since = control
    if (nonce <= last_request or nonce <= 0 or neutral != 1 or since > requested or
            not 0 <= now-requested <= .5 or not 0 <= now-issued <= .5):
        return None
    return dict(camera, auto_exposure=True, exposure_us=None, gain=None, auto_exposure_priority=False)


def validate_exposure(camera):
    if camera.get('stream') not in ('infrared', 'color'):
        raise ValueError('camera stream must be infrared or color')
    if not isinstance(camera.get('auto_exposure'), bool):
        raise ValueError('camera.auto_exposure must be explicitly true or false')
    exp, gain = camera.get('exposure_us'), camera.get('gain')
    for name, value in (('exposure_us', exp), ('gain', gain)):
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))
                                  or not math.isfinite(value) or value < 0):
            raise ValueError('invalid camera.'+name)
    if camera['auto_exposure']:
        if exp is not None or gain is not None:
            raise ValueError('automatic exposure requires exposure_us/gain=null')
    elif exp is None or exp <= 0:
        raise ValueError('manual exposure requires positive camera.exposure_us')
    priority = camera.get('auto_exposure_priority')
    if priority is not None and not isinstance(priority, bool):
        raise ValueError('auto_exposure_priority must be bool or null')
    if priority is not None and not camera['auto_exposure']:
        raise ValueError('auto_exposure_priority requires automatic exposure')


def select_sensor(device, stream, rs):
    target = rs.stream.infrared if stream == 'infrared' else rs.stream.color
    matches = [sensor for sensor in device.query_sensors()
               if any(p.stream_type() == target for p in sensor.get_stream_profiles())]
    if len(matches) != 1:
        raise ValueError('expected one sensor for '+stream)
    return matches[0]


def configure_exposure(device, camera, rs):
    """Preflight all requests, apply in order, verify requested controls.

    Readback confirms SDK control settings, not AE convergence or actual frame
    exposure. Unspecified manual gain is left unchanged and reported.
    """
    validate_exposure(camera)
    model = device.get_info(rs.camera_info.name)
    if 'D435' not in model.upper():
        raise ValueError('exposure option units verified only for D435/D435i: '+model)
    sensor = select_sensor(device, camera['stream'], rs)
    unit = 1. if camera['stream'] == 'infrared' else 100.
    requested = [('enable_auto_exposure', int(camera['auto_exposure']))]
    if camera.get('auto_exposure_priority') is not None:
        requested.append(('auto_exposure_priority', int(camera['auto_exposure_priority'])))
    if not camera['auto_exposure']:
        requested.append(('exposure', camera['exposure_us']/unit))
        if camera.get('gain') is not None:
            requested.append(('gain', camera['gain']))
    ranges = {}
    for name, value in requested:
        option = getattr(rs.option, name)
        if not sensor.supports(option) or sensor.is_option_read_only(option):
            raise ValueError('selected sensor cannot set '+name)
        r = sensor.get_option_range(option)
        ranges[name] = {'min': r.min, 'max': r.max, 'step': r.step}
        if not r.min <= value <= r.max:
            raise ValueError('%s=%g outside SDK range [%g,%g]' % (name, value, r.min, r.max))
        if r.step > 0 and abs((value-r.min)/r.step-round((value-r.min)/r.step)) > 1e-5:
            raise ValueError('%s=%g is not on SDK step %g (min %g)' % (name, value, r.step, r.min))
    # Disable AE BEFORE writing manual exposure/gain. A manual write can itself
    # disable AE, so never write exposure/gain when automatic mode is requested.
    for name, value in requested:
        sensor.set_option(getattr(rs.option, name), value)
    applied = {}
    for name, value in requested:
        actual = sensor.get_option(getattr(rs.option, name))
        if not math.isfinite(actual) or abs(actual-value) > max(1e-5, abs(value)*1e-6):
            raise RuntimeError('exposure control readback mismatch: '+name)
        applied[name] = actual
    for name in ('exposure', 'gain'):
        option = getattr(rs.option, name)
        if name not in applied and sensor.supports(option):
            actual = sensor.get_option(option)
            if math.isfinite(actual):
                applied[name] = actual
    return {'device': model, 'stream': camera['stream'], 'option_unit_us': unit,
            'requested': {k: camera.get(k) for k in
                          ('auto_exposure', 'exposure_us', 'gain', 'auto_exposure_priority')},
            'option_ranges_sdk_units': ranges, 'readback_sdk_units_at_start': applied,
            'readback_exposure_us_at_start': applied['exposure']*unit if 'exposure' in applied else None,
            'readback_note': 'SDK controls at startup; not per-frame exposure or AE convergence'}


def frame_exposure(frame, rs):
    """Cheap metadata reads on the original selected video frame (no USB poll).

    rs_frame.h defines actual_exposure metadata in microseconds, independently
    of the RGB *option* ticks. Unsupported metadata remains null, never zero.
    """
    out = {'actual_exposure_us': None, 'gain': None, 'auto_exposure': None}
    errors = []
    for key, name in (('actual_exposure_us', 'actual_exposure'), ('gain', 'gain_level'),
                      ('auto_exposure', 'auto_exposure')):
        option = getattr(rs.frame_metadata_value, name, None)
        if option is None:
            continue
        try:
            if frame.supports_frame_metadata(option):
                value = frame.get_frame_metadata(option)
                if math.isfinite(value):
                    out[key] = bool(value) if key == 'auto_exposure' else value
        except RuntimeError:
            errors.append(name)
    if errors:
        out['read_errors'] = errors
    return out
