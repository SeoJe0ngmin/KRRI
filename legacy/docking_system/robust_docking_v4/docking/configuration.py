"""Connect measured rig dimensions to the ground-referenced docking frames.

The body origin is on the centreline, at ground level and at the camera's
longitudinal station. It is a fixed body point, NOT the instantaneous centre
of rotation. Unknown rotations/offsets remain unknown; heights do not supply
them. Uses only the standard library, including on the synthesis host.
"""
import copy
import json
import math


BODY_REFERENCE = 'ground_centerline_at_camera_station'


def _number(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('invalid geometry value: '+name)
    if positive and value <= 0:
        raise ValueError('geometry value must be positive: '+name)
    return value


def rig_dimensions(sensor):
    rig = sensor.get('rig')
    if rig is None:
        return None  # Legacy/synthetic profiles have their own explicit frame.
    if rig.get('body_reference') != BODY_REFERENCE:
        raise ValueError('unsupported rig body_reference')
    for name in ('tag_height_m', 'camera_height_m', 'camera_to_fork_tip_m'):
        _number(rig.get(name), name, positive=True)
    return {
        'body_reference': BODY_REFERENCE,
        'tag_height_m': rig['tag_height_m'],
        'camera_height_m': rig['camera_height_m'],
        'camera_to_fork_tip_m': rig['camera_to_fork_tip_m'],
        'height_difference_m': rig['tag_height_m']-rig['camera_height_m'],
    }


def _rotation(R, name):
    if not isinstance(R, (list, tuple)) or len(R) != 3 or any(
            not isinstance(row, (list, tuple)) or len(row) != 3 for row in R):
        raise ValueError('expected 3x3 rotation: '+name)
    for row in R:
        for v in row:
            _number(v, name)
    for i in range(3):
        for j in range(3):
            dot = sum(R[k][i]*R[k][j] for k in range(3))
            if abs(dot-(1 if i == j else 0)) > 1e-5:
                raise ValueError('rotation not orthonormal: '+name)
    det = sum(R[0][i]*(R[1][(i+1)%3]*R[2][(i+2)%3]-R[1][(i+2)%3]*R[2][(i+1)%3]) for i in range(3))
    if abs(det-1) > 1e-5:
        raise ValueError('rotation has reflection: '+name)


def _transform(R, xyz):
    return [list(R[i])+[xyz[i]] for i in range(3)]+[[0, 0, 0, 1]]


def _resolve_transform(cal, key, R, xyz):
    """Fill a complete transform only when every component is provided.

    Existing matrices must agree with known measurements and any specified
    component, so an old matrix cannot silently override a changed height.
    """
    if R is not None:
        _rotation(R, key)
    for v in xyz:
        if v is not None:
            _number(v, key)
    existing = cal.get(key)
    if existing is not None:
        if not isinstance(existing, (list, tuple)) or len(existing) != 4 or any(
                not isinstance(row, (list, tuple)) or len(row) != 4 for row in existing):
            raise ValueError('expected 4x4 transform: '+key)
        if existing[3] != [0, 0, 0, 1]:
            raise ValueError('invalid homogeneous transform: '+key)
        actual_R = [row[:3] for row in existing[:3]]
        _rotation(actual_R, key)
        for i in range(3):
            _number(existing[i][3], key)
            if xyz[i] is not None and abs(existing[i][3]-xyz[i]) > 1e-6:
                raise ValueError(key+' translation conflicts with measured rig geometry')
            if R is not None and any(abs(a-b) > 1e-6 for a, b in zip(actual_R[i], R[i])):
                raise ValueError(key+' rotation conflicts with configured components')
    elif R is not None and all(v is not None for v in xyz):
        cal[key] = _transform(R, xyz)


def resolve_config(config):
    """Resolve on a copy before synthesis, policy loading or sensor startup."""
    cfg = copy.deepcopy(config)
    cfg.pop('_shared', None)
    sensor = cfg.get('sensor', cfg)
    rig = rig_dimensions(sensor)
    if rig is None:
        return cfg
    cal = sensor['calibration']
    xy = cal.get('tag_position_xy_m', [None, None])
    if not isinstance(xy, (list, tuple)) or len(xy) != 2:
        raise ValueError('tag_position_xy_m must contain dock x/y')
    _resolve_transform(cal, 'T_body_camera', cal.get('R_body_camera'),
                       [0., cal.get('camera_lateral_m'), rig['camera_height_m']])
    _resolve_transform(cal, 'T_dock_tag', cal.get('R_dock_tag'),
                       [xy[0], xy[1], rig['tag_height_m']])
    # With this particular body reference, camera-to-front is body.front_m.
    # Rear length/width cannot be inferred from the front distance.
    if 'sensor' in cfg:
        body = cfg.get('body')
        if body is None:
            body = cfg['body'] = {'front_m': rig['camera_to_fork_tip_m'],
                                  'rear_m': None, 'half_width_m': None}
        front = body.get('front_m')
        if front is None:
            body['front_m'] = rig['camera_to_fork_tip_m']
        elif abs(_number(front, 'body.front_m', positive=True)-rig['camera_to_fork_tip_m']) > 1e-6:
            raise ValueError('body.front_m conflicts with camera_to_fork_tip_m')
    return cfg


def load_config(path):
    with open(path) as f:
        config = json.load(f)
    if '_shared' in config:
        from .shared import manager
        config = manager().load(path, config)
    return resolve_config(config)
