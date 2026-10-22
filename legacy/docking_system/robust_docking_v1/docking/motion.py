"""Stop evidence uses measured time baselines, independently of absolute pose.

No averaging away correlated error; every endpoint bound includes acceleration.
The image estimator must supply a directly measured window displacement error,
not a per-frame error divided by a longer time.
"""
import math
from .core import finite
from .feedback import imu_ready


class AngularStopGate:
    """Veto stale video stillness on EVERY supervisor tick, including duplicates.

    This is only angular evidence; it never replaces measured translation or
    supplies an absolute heading. No integrated gyro state is modified.
    """
    def __init__(self, cfg):
        self.cfg = cfg
        self.last_stamp = self.last_gaps = None

    def update(self, imu, required):
        if not required:
            return None  # Explicit offline/synthetic observations.
        if not imu_ready(imu, self.cfg):
            self.last_stamp = self.last_gaps = None
            return False
        stamp, gaps = imu['device_stamp_s'], imu['gaps']
        continuous = ((self.last_stamp is None or stamp >= self.last_stamp) and
                      (self.last_gaps is None or gaps == self.last_gaps))
        self.last_stamp, self.last_gaps = stamp, gaps
        error = self.cfg['sensor']['bounds'].get('gyro_rate_error_rps')
        if not continuous or not finite([error]) or error < 0:
            return False
        upper = abs(imu['rate_rps'])+error
        age = imu['age_s']
        if age > 0:
            window = self.cfg['sensor'].get('stop_window', {})
            accel = window.get('max_angular_acceleration_rps2')
            if not window.get('verified') or not finite([accel]) or accel < 0:
                return False  # No justified current-rate bound from an aged sample.
            upper += accel*age
        return upper <= self.cfg['limits']['stop_yaw_rate_rps']


def velocity_bound(distance, error, dt, acceleration):
    if not finite([distance, error, dt, acceleration]) or min(distance, error, acceleration) < 0 or dt <= 0:
        return None
    return (distance+error)/dt + acceleration*dt/2


def validate_motion(sensor, limits=None, live=False):
    for name, fields in (
            ('stop_window', ('min_s', 'max_s', 'reference_period_s', 'max_frame_gap_s',
                             'translation_error_m', 'yaw_error_rad', 'max_angular_acceleration_rps2')),
            ('tracking_recovery', ('hold_keyframe_s', 'max_bridge_s', 'max_episode_s',
                                   'max_unobserved_speed_mps'))):
        c = sensor.get(name, {})
        if not c.get('enabled'):
            continue
        if live and not c.get('verified'):
            raise ValueError('unverified sensor.'+name)
        for k in fields:
            v = c.get(k)
            if v is None and not live and not c.get('verified'):
                continue
            if isinstance(v, bool) or not finite([v]) or v <= 0:
                raise ValueError('invalid '+name+'.'+k)
        if name == 'stop_window':
            if not 0 < c['reference_period_s'] <= c['min_s'] < c['max_s']:
                raise ValueError('invalid stop_window time baselines')
            if math.ceil(c['max_s']/c['reference_period_s']) > 16:
                raise ValueError('stop_window reference memory budget exceeded')
            if limits and c.get('verified'):
                e, a = c['translation_error_m'], sensor['bounds']['max_acceleration_mps2']
                if not finite([a]) or a < 0:
                    raise ValueError('stop_window needs an acceleration bound')
                t = c['max_s'] if a == 0 else max(c['min_s'], min(c['max_s'], math.sqrt(2*e/a)))
                age = sensor['bounds'].get('capture_to_callback_max_s', 0.) or 0.
                if velocity_bound(0., e, t, a)+a*age >= limits['stop_speed_mps']:
                    raise ValueError('stop_speed_mps is below the observable stop_window velocity floor')
        elif all(finite([c.get(k)]) for k in ('max_bridge_s', 'hold_keyframe_s', 'max_episode_s')):
            if not c['max_bridge_s'] <= c['hold_keyframe_s'] < c['max_episode_s']:
                raise ValueError('invalid tracking_recovery horizons')
            if limits and c.get('verified') and c['max_unobserved_speed_mps'] < limits['max_speed_mps']:
                raise ValueError('tracking recovery speed must cover physical speed limit')


def stop_speed(obs, now):
    """Missing window evidence must never silently fall back to noisy pairs."""
    window = obs.get('stop_motion')
    if window is None:  # Older explicit synthetic observations.
        return obs.get('speed_upper'), obs.get('motion_bounded', True)
    v, a, lower = window.get('speed_upper'), window.get('acceleration_bound'), obs.get('capture_lower')
    if not window.get('bounded') or not finite([v, a, lower]) or not lower <= now:
        return None, False
    return v+a*(now-lower), True


def stopped(obs, now, limits, angular_ok=None):
    v, bounded = stop_speed(obs, now)
    w = obs.get('yaw_rate_upper')
    return bool(angular_ok is not False and bounded and finite([v, w]) and 0 <= v <= limits['stop_speed_mps'] and
                0 <= w <= limits['stop_yaw_rate_rps'] and not obs.get('imu_bad'))
