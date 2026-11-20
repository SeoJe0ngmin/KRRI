"""Small bounded transports; IMU is ordered and loss is an explicit fault."""
from collections import deque
import threading
import time
import numpy as np


class ClockUnwrap:
    """D4xx hardware_clock stamps come from a 32-bit microsecond counter and wrap every
    2^32 us = 4294.967296 s (71.6 min) of camera uptime; global_time is disabled here on purpose.
    Each stream is unwrapped against its own previous stamp (all streams share one counter)."""
    PERIOD_S=2**32/1e6
    def __init__(self):
        self.last={};self.offset={};self.wraps=0
    def __call__(self,stream,stamp):
        previous=self.last.get(stream)
        if previous is not None and stamp<previous-self.PERIOD_S/2:
            self.offset[stream]=self.offset.get(stream,0.)+self.PERIOD_S;self.wraps+=1
        self.last[stream]=stamp
        return stamp+self.offset.get(stream,0.)


def publish_latest(q,item):
    import queue
    try:q.put_nowait(item);return
    except queue.Full:pass
    try:q.get_nowait()
    except queue.Empty:pass
    try:q.put_nowait(item)
    except queue.Full:pass


class Latest:
    def __init__(self):
        self.lock=threading.Lock();self.item=None;self.dropped=0
    def put(self,item):
        with self.lock:
            if self.item is not None:self.dropped+=1
            self.item=item
    def get(self):
        with self.lock:
            value=self.item;self.item=None;return value


class OutputWatchdog:
    """Start the result deadline only once the estimator has video input.

    More input does not extend the deadline. Only an actual output does.
    Camera startup has its own independent deadline.
    """
    def __init__(self,timeout=10.):self.timeout=timeout;self.last_output_or_input=None
    def input(self,now):
        if self.last_output_or_input is None:self.last_output_or_input=now
    def output(self,now):self.last_output_or_input=now
    def expired(self,now):
        return self.last_output_or_input is not None and now-self.last_output_or_input>self.timeout


class VideoDeadline:
    """Incomplete/repeated composite frames do not establish video readiness."""
    def __init__(self,now,startup_timeout=30.,stream_timeout=2.):
        self.last=now;self.startup_timeout=startup_timeout;self.stream_timeout=stream_timeout
        self.started=False;self.incomplete=0
    def accepted(self,now):self.last=now;self.started=True
    def check(self,now):
        limit=self.stream_timeout if self.started else self.startup_timeout
        if now-self.last>limit:
            phase='streaming' if self.started else 'startup'
            raise RuntimeError(f'video {phase}: no fresh complete IR-left/IR-right/color frames for {limit}s; '
                               f'incomplete_framesets={self.incomplete}; check USB connection and camera ownership')


class StereoSequence:
    """A composite frameset can repeat IR when a different stream updates.

    Frame counters may be missing/reset by SDK metadata while hardware time
    advances. Track those anomalies; use hardware timestamps for continuity.
    Actual timestamp reversal/nonadvancement on new imagery is still a fault.
    """
    def __init__(self):
        self.last=None;self.duplicates=0;self.counter_anomalies=0
    def accept(self,left_number,right_number,left_stamp,right_stamp):
        current=(int(left_number),int(right_number),float(left_stamp),float(right_stamp))
        if not np.isfinite(current).all():raise ValueError('nonfinite_stereo_sequence')
        if current==self.last:
            self.duplicates+=1;return False
        if self.last is not None:
            for name,n,t,old_n,old_t in (('left',current[0],current[2],self.last[0],self.last[2]),
                                       ('right',current[1],current[3],self.last[1],self.last[3])):
                if t<=old_t:
                    raise ValueError(f'stereo_sequence_invalid {name}: frame {old_n}->{n}, timestamp {old_t:.9f}->{t:.9f}')
                if n<=old_n:self.counter_anomalies+=1
        self.last=current;return True


class ImuSynchronizer:
    """Interpolate acceleration at every gyro time, using bracketing samples.

    Acceleration is first rotated into the SDK gyro frame; no command-derived
    yaw, zero filling or unbounded last-value holding is used.
    """
    def __init__(self,capacity=4096,max_gap=.05,startup_window_s=0.,short_gap_s=None):
        # 2026-10-19 (user decision, apriltag_v4 rule): a gap between samples is COUNTED, never fatal. v4 skipped
        # integration across gaps > 10 samples and only stopped when no sample arrived for IMU_STALE_SEC (1 s);
        # that is tuning imu_age_s here. A desk run died after 30 s on a 0.1 s gap under the old rule.
        self.short_gap_s=max_gap if short_gap_s is None else short_gap_s
        self.gaps=0;self.last_gap=None;self.largest_gap_s=0.;self.reversals=0
        self.accel=deque();self.gyro=deque();self.capacity=capacity;self.max_gap=max_gap
        self.last={};self.lock=threading.Lock();self.fault=None;self.sequence=0
        self.arrival=0.;self.recent=deque(maxlen=1000);self.arrivals={};self.counts={}
        self.startup_window_s=startup_window_s;self.startup_ready=startup_window_s<=0
        self.startup_resets=0;self.startup_discarded=0;self.last_startup_reset=None
    def add(self,kind,stamp,vector,arrival=None):
        with self.lock:
            arrival=time.monotonic() if arrival is None else arrival
            if self.fault:return
            if kind not in ('gyro','accel'):self.fault='unknown_imu_stream: '+str(kind);return
            v=np.asarray(vector,float)
            if v.shape!=(3,) or not np.isfinite(v).all() or not np.isfinite(stamp):
                self.fault='nonfinite_imu';return
            previous=self.last.get(kind)
            if previous is not None and not 0<stamp-previous<=self.max_gap:
                delta=stamp-previous
                reason='clock_reversed' if delta<0 else 'duplicate_timestamp' if delta==0 else 'sample_gap'
                detail=(f'imu_clock_or_sample_gap: stream={kind} reason={reason} '
                    f'device_stamp={previous:.9f}->{stamp:.9f} delta_s={delta:.9f} limit_s={self.max_gap:.6f} '
                    f'host_arrival_gap_s={arrival-self.arrivals[kind]:.6f} paired_samples={self.sequence}')
                if self.startup_ready:
                    if delta<=0:
                        self.reversals+=1;self.last_gap=detail;return   # a non-monotonic sample cannot be integrated: skip it
                    self.gaps+=1;self.last_gap=detail;self.largest_gap_s=max(self.largest_gap_s,delta)   # record, continue
                else:
                    # No IMU or image has reached the estimator yet. Discard BOTH
                    # streams and start a new candidate window; never interpolate
                    # across this gap or recover this way after streaming begins.
                    self.last_startup_reset=detail;self.startup_resets+=1
                    self.startup_discarded+=len(self.gyro)+len(self.accel)
                    self.gyro.clear();self.accel.clear();self.last.clear();self.arrivals.clear()
            self.last[kind]=stamp
            q=self.gyro if kind=='gyro' else self.accel
            if not self.startup_ready:
                while q and stamp-q[0][0]>max(1.,2*self.startup_window_s):
                    q.popleft();self.startup_discarded+=1
            if len(q)>=self.capacity:self.fault='imu_queue_overflow';return
            q.append((stamp,v,arrival));self.arrival=arrival;self.arrivals[kind]=arrival
            self.counts[kind]=self.counts.get(kind,0)+1
    def begin(self,video_stamp,now):
        """Commit once: continuous live IMU window and matching video clock.

        Both device AND host spans are required, so a burst of old buffered
        samples cannot pass. No fabricated timestamps or zero-motion claim.
        Caller must drain IMU before publishing the first stereo image.
        """
        with self.lock:
            if self.fault:raise RuntimeError(self.fault)
            if self.startup_ready:return True
            for q in (self.gyro,self.accel):
                if len(q)<2:return False
                a,b=q[0],q[-1]
                if b[0]-a[0]<self.startup_window_s or b[2]-a[2]<self.startup_window_s:return False
                if not 0<=now-b[2]<=2*self.max_gap:return False
                if a[0]>video_stamp or abs(b[0]-video_stamp)>2*self.max_gap:return False
            self.startup_ready=True
            return True
    def status(self):
        with self.lock:
            return dict(counts=dict(self.counts),last_device_stamp=dict(self.last),last_arrival=dict(self.arrivals),
                gaps=self.gaps,reversals=self.reversals,largest_gap_s=self.largest_gap_s,last_gap=self.last_gap,paired=self.sequence,gyro_pending=len(self.gyro),accel_pending=len(self.accel),fault=self.fault,
                startup_ready=self.startup_ready,startup_resets=self.startup_resets,
                startup_discarded=self.startup_discarded,last_startup_reset=self.last_startup_reset)
    def drain(self):
        out=[]
        with self.lock:
            if self.fault:raise RuntimeError(self.fault)
            if not self.startup_ready:return out
            while self.gyro and len(self.accel)>=2:
                t,g,arrival=self.gyro[0]
                if t<self.accel[0][0]:
                    # Only startup gyro samples preceding the first accel.
                    if self.sequence:raise RuntimeError('unbracketed_imu')
                    self.gyro.popleft();continue
                while len(self.accel)>2 and self.accel[1][0]<=t:self.accel.popleft()
                a,b=self.accel[0],self.accel[1]
                if t>b[0]:break
                if b[0]-a[0]>self.short_gap_s:self.gaps+=1;self.largest_gap_s=max(self.largest_gap_s,b[0]-a[0])   # wide accel bracket: interpolate anyway, count
                w=(t-a[0])/(b[0]-a[0]);acc=(1-w)*a[1]+w*b[1]
                self.sequence+=1;self.gyro.popleft()
                sample=dict(stamp=t,gyro=g.tolist(),accel=acc.tolist(),sequence=self.sequence,arrival=max(arrival,b[2]))
                self.recent.append(sample);out.append(sample)
        return out


class CaptureClock:
    """Device seconds mapped to host monotonic for age checks, never wall time."""
    def __init__(self):self.offset=None;self.last={}
    def observe(self,kind,stamp,arrival):
        if not np.isfinite([stamp,arrival]).all():raise ValueError('nonfinite_clock')
        if kind in self.last and stamp<=self.last[kind]:raise ValueError(f'sensor_clock_reversed {kind}: {self.last[kind]:.9f}->{stamp:.9f}')
        self.last[kind]=stamp
        offset=arrival-stamp
        # Minimum observed transport offset avoids calling a queued frame fresh.
        self.offset=offset if self.offset is None else min(self.offset,offset)
        return self.offset
    def host_time(self,stamp):
        if self.offset is None:raise ValueError('clock_not_initialized')
        return stamp+self.offset
