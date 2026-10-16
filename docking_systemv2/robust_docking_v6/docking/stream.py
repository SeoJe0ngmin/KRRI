"""Small bounded transports; IMU is ordered and loss is an explicit fault."""
from collections import deque
import threading
import time
import numpy as np


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


class ImuSynchronizer:
    """Interpolate acceleration at every gyro time, using bracketing samples.

    Acceleration is first rotated into the SDK gyro frame; no command-derived
    yaw, zero filling or unbounded last-value holding is used.
    """
    def __init__(self,capacity=4096,max_gap=.05):
        self.accel=deque();self.gyro=deque();self.capacity=capacity;self.max_gap=max_gap
        self.last={};self.lock=threading.Lock();self.fault=None;self.sequence=0
        self.arrival=0.;self.recent=deque(maxlen=1000)
    def add(self,kind,stamp,vector,arrival=None):
        with self.lock:
            arrival=time.monotonic() if arrival is None else arrival
            if self.fault:return
            v=np.asarray(vector,float)
            if v.shape!=(3,) or not np.isfinite(v).all() or not np.isfinite(stamp):
                self.fault='nonfinite_imu';return
            previous=self.last.get(kind)
            if previous is not None and not 0<stamp-previous<=self.max_gap:
                self.fault='imu_clock_or_sample_gap';return
            self.last[kind]=stamp
            q=self.gyro if kind=='gyro' else self.accel
            if len(q)>=self.capacity:self.fault='imu_queue_overflow';return
            q.append((stamp,v,arrival));self.arrival=arrival
    def drain(self):
        out=[]
        with self.lock:
            if self.fault:raise RuntimeError(self.fault)
            while self.gyro and len(self.accel)>=2:
                t,g,arrival=self.gyro[0]
                if t<self.accel[0][0]:
                    # Only startup gyro samples preceding the first accel.
                    if self.sequence:raise RuntimeError('unbracketed_imu')
                    self.gyro.popleft();continue
                while len(self.accel)>2 and self.accel[1][0]<=t:self.accel.popleft()
                a,b=self.accel[0],self.accel[1]
                if t>b[0]:break
                if b[0]-a[0]>self.max_gap:raise RuntimeError('accel_interpolation_gap')
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
        if kind in self.last and stamp<=self.last[kind]:raise ValueError('sensor_clock_reversed')
        self.last[kind]=stamp
        offset=arrival-stamp
        # Minimum observed transport offset avoids calling a queued frame fresh.
        self.offset=offset if self.offset is None else min(self.offset,offset)
        return self.offset
    def host_time(self,stamp):
        if self.offset is None:raise ValueError('clock_not_initialized')
        return stamp+self.offset
