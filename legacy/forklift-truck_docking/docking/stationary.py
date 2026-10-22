"""Stationary judgment by gyro, apriltag_v4 style (2026-10-19 user decision).

Rotation-still: p90 of |gyro - bias| over the last still_window_s (0.5 s, v4 WINDOW_S) <= still_gyro_dps
(0.5 deg/s, v4 IMU_MOVING_DPS). The bias is OpenVINS' running estimate when available; before that the raw
norm is used and the report says so. Raw 200 Hz samples are preferred; without them (tests, old logs) the
per-packet gyro norm is used.

Translation-still: a straight coast is invisible to the gyro, so VIO speed over the window must be below
still_speed_mps (0.03 m/s = 1/10 of the 0.29 m/s nominal; the opposite side of v4's 1/3-nominal start rule,
chosen by hand). Without VIO the v4 time rule applies: stable_s (2 s) after the neutral command.

Never blocks for long: the controller's preparation only waits for these two facts. Limits are
engineering heuristics requiring vehicle validation, not guarantees.
"""
import math
from collections import deque
import numpy as np


class StationaryDetector:
    def __init__(self,tuning,gyro_hz=200.):
        self.t=tuning;self.samples=deque();self.norms=deque();self.positions=deque()
        self.bias=np.zeros(3);self.bias_source='raw_no_bias'
        self.min_samples=max(5,int(.5*gyro_hz*tuning['still_window_s']))
        self.report=dict(reason='no_gyro_samples');self.last_rate=None

    def reset_window(self):
        self.positions.clear()

    def add_gyro(self,stamp,w):
        w=np.asarray(w,float)
        if w.shape!=(3,) or not np.isfinite(w).all() or not math.isfinite(stamp):return
        self.samples.append((float(stamp),w))
        while self.samples and stamp-self.samples[0][0]>4*self.t['still_window_s']:self.samples.popleft()

    def set_bias(self,bias):
        if bias is None:return
        b=np.asarray(bias,float)
        if b.shape==(3,) and np.isfinite(b).all():self.bias=b;self.bias_source='openvins'

    def rate(self,stamp):
        """(p90 rate [rad/s], sample count, source) over the window ending at stamp; (None,0,source) if too few."""
        w=self.t['still_window_s']
        raw=[np.linalg.norm(s[1]-self.bias) for s in self.samples if stamp-w<s[0]<=stamp]
        if len(raw)>=self.min_samples:return float(np.percentile(raw,90)),len(raw),'raw_200hz'
        packets=[v for s,v in self.norms if stamp-w<s<=stamp]
        if len(packets)>=3:return float(np.percentile(packets,90)),len(packets),'vio_packet_norm'
        return None,len(raw)+len(packets),'insufficient'

    def update(self,stamp,p,neutral,since_neutral_s,vio_ok,gyro_norm=None):
        if gyro_norm is not None and math.isfinite(gyro_norm):
            self.norms.append((stamp,float(gyro_norm)))
            while self.norms and stamp-self.norms[0][0]>4*self.t['still_window_s']:self.norms.popleft()
        if vio_ok and p is not None and np.isfinite(p).all():
            self.positions.append((stamp,np.asarray(p,float)[:2].copy()))
            while self.positions and stamp-self.positions[0][0]>self.t['still_window_s']:self.positions.popleft()
        else:self.positions.clear()
        rate,count,source=self.rate(stamp);self.last_rate=rate
        if rate is None:
            self.report=dict(reason='no_gyro_samples',gyro_samples=count,gyro_source=source);return False
        rotating=rate>math.radians(self.t['still_gyro_dps'])
        speed=None
        if len(self.positions)>=2 and self.positions[-1][0]-self.positions[0][0]>=.6*self.t['still_window_s']:
            speed=float(np.linalg.norm(self.positions[-1][1]-self.positions[0][1])/(self.positions[-1][0]-self.positions[0][0]))
        if speed is not None:translating=speed>self.t['still_speed_mps'];rule='vio_speed'
        else:
            translating=not (since_neutral_s is not None and since_neutral_s>=self.t['stable_s']);rule='time_since_neutral'
        still=bool(neutral and not rotating and not translating)
        self.report=dict(reason='stationary' if still else 'command_not_neutral' if not neutral else 'rotating' if rotating else 'translating_or_waiting',
            gyro_p90_dps=math.degrees(rate),gyro_limit_dps=self.t['still_gyro_dps'],gyro_samples=count,gyro_source=source,bias_source=self.bias_source,
            vio_speed_mps=speed,speed_limit_mps=self.t['still_speed_mps'],translation_rule=rule,since_neutral_s=since_neutral_s,
            certified_error_bound=False)
        return still
