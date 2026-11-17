"""Bounded capture-time history. No extrapolation and no cross-epoch bridging."""
from dataclasses import dataclass,field
import bisect
import numpy as np
import cv2
from .uncertainty import transform, covariance


@dataclass
class State:
    stamp: float
    epoch: int
    T: np.ndarray
    P: np.ndarray
    arrival: float
    features: int = 0
    noise: np.ndarray = field(default_factory=lambda:np.zeros(3))


class PoseHistory:
    def __init__(self, seconds=5., max_samples=300, max_gap=.15):
        self.seconds, self.max_samples, self.max_gap = seconds, max_samples, max_gap
        self.items = []
        self.epoch = None

    def add(self, state):
        if not np.isfinite([state.stamp, state.arrival]).all():
            raise ValueError('nonfinite state time')
        transform(state.T); covariance(state.P, 6)
        changed = self.epoch is not None and state.epoch != self.epoch
        if changed:
            self.items.clear()
        self.epoch = state.epoch
        if self.items and state.stamp <= self.items[-1].stamp:
            raise ValueError('nonmonotonic VIO state')
        self.items.append(state)
        while len(self.items) > self.max_samples or self.items[-1].stamp-self.items[0].stamp > self.seconds:
            self.items.pop(0)
        return changed

    def at(self, stamp, epoch):
        if epoch != self.epoch or not self.items:
            raise ValueError('history_epoch_unavailable')
        ts = [s.stamp for s in self.items]
        i = bisect.bisect_left(ts, stamp)
        if i < len(ts) and abs(ts[i]-stamp) < 1e-7:
            return self.items[i]
        if i == 0 or i == len(ts):
            raise ValueError('capture_outside_history')
        a, b = self.items[i-1:i+1]
        if b.stamp-a.stamp > self.max_gap:
            raise ValueError('history_gap')
        w = (stamp-a.stamp)/(b.stamp-a.stamp)
        T = np.eye(4); T[:3, 3] = (1-w)*a.T[:3, 3]+w*b.T[:3, 3]
        rotation = cv2.Rodrigues(a.T[:3, :3].T @ b.T[:3, :3])[0]
        T[:3, :3] = a.T[:3, :3] @ cv2.Rodrigues(w*rotation)[0]
        # Convex upper bound with unknown correlation; do not shrink variance
        # merely because two adjacent states were used for interpolation.
        P = (1-w)*a.P+w*b.P
        return State(stamp, epoch, T, P, max(a.arrival, b.arrival), min(a.features, b.features),a.noise.copy())
