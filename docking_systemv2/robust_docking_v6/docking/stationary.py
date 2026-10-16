"""Bounded startup vibration baseline and post-neutral settling, not a pose reset.

Limits are engineering heuristics requiring vehicle validation, not guarantees.
"""
from collections import deque
import numpy as np


class StationaryDetector:
    def __init__(self,tuning,uncertainty):
        self.t=tuning;self.u=uncertainty;self.rows=deque();self.baseline=None
        self.locked=False;self.report=dict(reason='collecting_vibration_baseline')
        self.last_stamp=None;self.neutral_token=None;self.observed_std=np.zeros(3)

    def reset_window(self):
        self.rows.clear();self.last_stamp=None

    @property
    def pose_std(self):
        return np.zeros(3) if self.baseline is None else np.maximum(self.baseline['std'],self.observed_std).copy()

    def update(self,stamp,p,gyro,good,neutral,neutral_token=None):
        if not neutral:self.locked=True
        if neutral_token!=self.neutral_token:
            self.reset_window();self.neutral_token=neutral_token
        if self.last_stamp is not None and (stamp<=self.last_stamp or stamp-self.last_stamp>self.t['frame_gap_s']):
            self.reset_window()
        self.last_stamp=stamp
        if not good or not neutral or not np.isfinite([*p,gyro]).all():
            self.rows.clear();self.report=dict(reason='moving_or_tracking_unavailable');return False
        self.rows.append((stamp,np.asarray(p).copy(),gyro))
        duration=self.t['vibration_baseline_s'] if self.baseline is None else self.t['stable_s']
        while len(self.rows)>1 and stamp-self.rows[1][0]>=duration:self.rows.popleft()
        if stamp-self.rows[0][0]<duration:
            self.report=dict(reason='collecting_vibration_baseline' if self.baseline is None else 'settling_window');return False
        times=np.array([r[0] for r in self.rows]);pts=np.array([r[1] for r in self.rows])
        pts[:,2]=np.unwrap(pts[:,2]);times-=times[0]
        # Quarter-second medians suppress fast oscillation. Every block must have
        # enough real observations; missing visual tracking cannot look like rest.
        bucket=np.floor(times/self.t['still_bucket_s']).astype(int)
        centers=[];bt=[]
        for b in range(int(duration/self.t['still_bucket_s'])):
            mask=bucket==b
            if mask.sum()<3:
                self.report=dict(reason='insufficient_stationary_samples');return False
            centers.append(np.median(pts[mask],axis=0));bt.append(np.median(times[mask]))
        centers=np.array(centers);bt=np.array(bt)
        trend=np.abs(np.polyfit(bt,centers,1)[0])*duration
        excursion=np.ptp(centers,axis=0)
        # Checking ALL block centers also catches out-and-back rebound with zero
        # net displacement. Trend thresholds never grow with vibration amplitude.
        limit=np.array([self.t['still_translation_m']]*2+[self.t['still_yaw_rad']])
        quiet=bool(np.all(trend<limit) and np.all(excursion<limit))
        residual=pts-np.median(pts,axis=0)
        amplitude=np.percentile(np.abs(residual),95,axis=0)
        std=np.sqrt(np.mean(residual**2,axis=0))
        rate=float(np.percentile([r[2] for r in self.rows],90))
        cap=np.array([self.t['vibration_position_cap_m']]*2+[self.t['vibration_yaw_cap_rad']])
        bounded=bool(np.all(amplitude<cap) and rate<self.t['vibration_gyro_cap_rps'])
        if self.baseline is None:
            if self.locked:
                self.report=dict(reason='startup_baseline_required');return False
            if quiet and bounded:
                self.baseline=dict(amplitude=amplitude,std=std,gyro=rate)
            else:
                self.report=dict(reason='baseline_drift_or_excess_vibration',trend=trend.tolist(),excursion=excursion.tolist());return False
        allowed=np.minimum(cap,np.maximum(limit,2*self.baseline['amplitude']))
        gyro_limit=min(self.t['vibration_gyro_cap_rps'],max(self.u['still_gyro_norm_rps'],2*self.baseline['gyro']))
        still=bool(quiet and bounded and np.all(amplitude<=allowed) and rate<=gyro_limit)
        if still:self.observed_std=std.copy()
        self.report=dict(reason='stationary_with_vibration' if still else 'residual_motion_or_rebound',
            baseline_ready=True,baseline_locked=self.locked,trend=trend.tolist(),excursion=excursion.tolist(),
            amplitude=amplitude.tolist(),allowed_amplitude=allowed.tolist(),gyro_p90=rate,
            pose_std=self.pose_std.tolist(),certified_error_bound=False)
        return still
