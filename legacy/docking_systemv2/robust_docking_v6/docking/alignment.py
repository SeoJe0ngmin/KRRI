"""Capture-time dock<-odom correction, bounded delayed-event replay.

The dock correction is never integrated into physical odometry. Statistical
NIS is distinguished from our correlation-conservative innovation gate.
"""
import bisect
from dataclasses import dataclass,field
import numpy as np
from .uncertainty import covariance, innovation, intersection,drift_bound


@dataclass
class AnchorEvent:
    stamp: float
    key: str
    epoch: int
    mean: np.ndarray
    cov: np.ndarray
    noise: np.ndarray = field(default_factory=lambda:np.zeros(3))


class Alignment:
    def __init__(self, cfg):
        self.cfg=cfg; self.epoch=None; self.base=None; self.events=[]
        self.mean=self.P=None; self.stamp=None; self.closed_before=-float('inf')
        self.reports={}; self.revision=0

    def reset(self, epoch):
        self.__init__(self.cfg); self.epoch=epoch

    def _update(self, state, event):
        if state is None:
            return (event.mean.copy(),event.cov.copy(),event.stamp,1,event.noise.copy()),dict(accepted=True,reason='first_tag_anchor')
        a,P,stamp,count,noise=state
        P=drift_bound(P,event.noise-noise)
        d,score=innovation(a,P,event.mean,event.cov)
        nominal=float(d @ np.linalg.solve(P+event.cov+np.eye(3)*1e-12,d))
        accepted=(score<=self.cfg['innovation_gate'] and np.linalg.norm(d[:2])<=self.cfg['max_anchor_jump_m']
                  and abs(d[2])<=self.cfg['max_anchor_jump_rad'])
        report=dict(accepted=bool(accepted),residual=d.tolist(),nis_independence_assumption=nominal,
                    conservative_mahalanobis_sq=score,nis_dof=3,
                    nis_chi_square_valid=False,reason='tag_ci_update' if accepted else 'tag_vio_conflict')
        if not accepted:
            return state,report
        a,P=intersection(a,P,event.mean,event.cov)
        return (a,P,event.stamp,count+1,event.noise.copy()),report

    def add(self, event):
        covariance(event.cov,3,True)
        if self.epoch is None:self.epoch=event.epoch
        if event.epoch!=self.epoch:raise ValueError('anchor_epoch_mismatch')
        if not np.isfinite(event.mean).all() or not np.isfinite(event.stamp):raise ValueError('nonfinite anchor')
        if event.stamp<=self.closed_before:return dict(accepted=False,reason='tag_older_than_replay_history')
        if any(e.key==event.key for e in self.events):return dict(accepted=False,reason='duplicate_tag_event')
        # One update per capture-time bucket. Frame rate alone cannot improve P.
        bucket=int(event.stamp/self.cfg['correlation_window_s'])
        if any(int(e.stamp/self.cfg['correlation_window_s'])==bucket for e in self.events):
            return dict(accepted=False,reason='correlated_time_bucket')
        bisect.insort(self.events,event,key=lambda e:(e.stamp,e.key))
        states=[]; state=self.base; self.reports={}
        for e in self.events:
            state,report=self._update(state,e);states.append(state);self.reports[e.key]=report
        # Keep the already-assimilated checkpoint, discard only replay history.
        newest=self.events[-1].stamp
        remove=0
        while remove<len(self.events)-1 and (newest-self.events[remove].stamp>self.cfg['history_s']
                or len(self.events)-remove>self.cfg['max_events']):remove+=1
        if remove:
            self.base=states[remove-1];self.closed_before=self.events[remove-1].stamp
            self.events=self.events[remove:]
        if state is not None:
            self.mean,self.P,self.stamp,self.count,self.noise=state
        self.revision+=1
        return self.reports[event.key]

    @property
    def ready(self):
        return self.mean is not None and getattr(self,'count',0)>=self.cfg['min_anchor_observations']
