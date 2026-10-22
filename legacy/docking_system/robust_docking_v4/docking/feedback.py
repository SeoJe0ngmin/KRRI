"""Image-independent gyro delivery and bounded rotation cutoff.

The supervisor polls the latest callback sample; no image queue is involved.
This is not a hard real-time thread. Host scheduling, IMU transport and CAN
response still belong in the measured braking reserve and watchdog limits.
"""
import time
from .core import finite


def read_imu(shared, now=None):
    with shared.get_lock():
        count, arrival, stamp, yaw, rate, gaps, reversals, ready = shared[:]
    now = time.monotonic() if now is None else now
    return dict(count=int(count), device_stamp_s=stamp, yaw_rad=yaw, host_arrival_s=arrival,
                rate_rps=rate, gaps=int(gaps), clock_reversals=int(reversals),
                ready=bool(ready), age_s=now-arrival)


def imu_ready(imu, cfg):
    age = cfg['sensor']['imu'].get('max_age_s')
    return bool(imu and imu.get('count', 0) > 0 and imu.get('ready') and
                not imu.get('clock_reversals') and
                finite([age, imu.get('age_s'), imu.get('device_stamp_s'),
                        imu.get('yaw_rad'), imu.get('rate_rps'), imu.get('gaps')]) and
                0 <= imu['age_s'] <= age)


class RotationFeedback:
    def __init__(self, cfg, action, center_yaw, obs, imu):
        self.cfg, self.action = cfg, action
        ref = obs.get('capture_imu') or {}
        rate_error = cfg['sensor']['bounds'].get('gyro_rate_error_rps')
        if (not imu_ready(imu, cfg) or not finite([rate_error]) or rate_error < 0 or
                not finite([ref.get('yaw_rad'), ref.get('device_stamp_s'), ref.get('gaps')]) or
                ref['device_stamp_s'] != obs['capture_id'] or
                ref['device_stamp_s'] > imu['device_stamp_s'] or ref['gaps'] != imu['gaps']):
            raise ValueError('rotation_imu_reference_unavailable')
        self.ref, self.rate_error = ref, rate_error
        self.heading, self.heading_error = obs['pose'][2], obs['error'][2]
        self.center_yaw = center_yaw  # Same absolute target as policy synthesis.
        self.arm_yaw, self.arm_stamp = imu['yaw_rad'], imu['device_stamp_s']
        self.last_stamp = self.arm_stamp
        self.sign = 1 if action['target'] > 0 else -1
        self.cutoff = abs(action['target'])-action['brake_reserve']
        if self.progress_upper(imu) >= self.cutoff:
            raise ValueError('rotation_start_inside_braking_region')

    def progress_upper(self, imu):
        dt = imu['device_stamp_s']-self.ref['device_stamp_s']
        heading = self.heading+imu['yaw_rad']-self.ref['yaw_rad']
        error = (self.heading_error+self.rate_error*dt +
                 self.cfg['limits']['max_yaw_rate_rps']*imu['age_s'])
        return self.sign*(heading-self.center_yaw)+error

    def update(self, imu):
        if imu and imu.get('clock_reversals'):
            return 'clock_reversed', False
        if not imu_ready(imu, self.cfg):
            return 'unavailable', False
        if imu['device_stamp_s'] < self.last_stamp:
            return 'clock_reversed', False
        if imu['gaps'] != self.ref['gaps']:
            return 'gap', False
        self.last_stamp = imu['device_stamp_s']
        dt = imu['device_stamp_s']-self.arm_stamp
        moved = self.sign*(imu['yaw_rad']-self.arm_yaw)-self.rate_error*dt > 0
        return ('brake' if self.progress_upper(imu) >= self.cutoff else 'moving'), moved
