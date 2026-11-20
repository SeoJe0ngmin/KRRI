"""Tag-main, VIO-assist dock pose (2026-10-19 user decision).

What the tag gives that we trust (apriltag_v4/v5 field data): the screen position of the tag
(bearing β, repeatable to ~0.004 deg) and the range. What we do not trust frame by frame: the PnP
heading and therefore the PnP lateral (biased by up to +4/-3 deg per viewing pose, 2026-10-02).

So:
  heading   = VIO yaw + offset          (offset: PnP heading as a WEAK reference, learned at rest only)
  position  = tag_xy - R(heading) · (tag vector in the body frame from β, range)
  anchor    = dock <- odom translation, refreshed from every 0.5 s window of >= 3 tag frames
              (apriltag_v4 WINDOW_S / MIN_N), at rest or while moving
  no tag    = VIO only (odom pose through the last anchor), bounded by the controller's blind limits
  check     = predicted (VIO) vs new tag window by PHYSICS, not statistics: a window may differ by at most
              max(3 cm, 1 cm/m × range) + 0.29 m/s × time since the last accepted window. A window beyond that
              is skipped (VIO carries on, as when the tag is hidden); if windows keep disagreeing for 1 s the
              tag wins anyway. Nothing here can stop the vehicle. (3 cm = docking tolerance; 1 cm/m = 39 mm
              single-frame lateral noise at 3.7 m on 2026-09-07; 0.29 m/s = nominal forward speed.)

σ = sqrt(baseline² + window jitter² + accumulated VIO relative variance). The baseline is the drift of the
tag at rest over the 30 s preparation (apriltag_v4 '밀림', shortened from 60 s on 2026-10-19 with 10/1 data:
4 of 6 runs within 30 % of the 60 s value). It is recorded and scaled with range; it is not used for any
decision (a desk run on 2026-10-19 measured 0.1 mm and a 3σ gate then fired on 0.05 mm residuals).
Nothing here is a certified error bound.
"""
import math
from collections import deque
import numpy as np
from .geometry import wrap, pose
from .uncertainty import transform, inverse

# No tag during preparation -> the baseline cannot be measured. Fallback from the 2026-10-02 calibrate run
# (60 s at rest, 4.2 m): lateral drift 19 mm. Heading: PnP heading vs gyro sd 1.4-1.7 deg (2026-10-01 runs).
# Longitudinal drift was not measured separately; the lateral value is reused as a stand-in.
SIGMA_FLOOR_M=.01;SIGMA_FLOOR_RAD=math.radians(.5)   # see Baseline.scaled (σ is record-only since the jump bound)
NOMINAL_SPEED_MPS=.29   # forward speed at byte 67 (KWU 0.290, ours 0.284-0.290 on 2026-09-07/10-01); bounds the plausible tag jump
DEFAULT_BASELINE=dict(lateral_m=.019,longitudinal_m=.019,heading_rad=math.radians(1.5),distance_m=4.2,
                      blocks=0,span_s=0.,source='default_20261002_calibrate_not_measured_here')


def tag_bearing_range(T_camera_tag,T_body_camera):
    """Planar bearing (CCW from body x) and range of the tag centre in the body frame."""
    Tbt=transform(T_body_camera)@transform(T_camera_tag)
    x,y=float(Tbt[0,3]),float(Tbt[1,3])
    return math.atan2(y,x),math.hypot(x,y)


def pnp_dock_pose(T_camera_tag,T_body_camera,T_dock_tag):
    """Full PnP body pose in the dock frame [x,y,yaw] — the heading is the weak reference, the rest is for logs."""
    return pose(transform(T_dock_tag)@inverse(T_camera_tag)@inverse(T_body_camera))


def body_in_dock(heading,bearing,rng,tag_xy):
    c,s=math.cos(heading),math.sin(heading)
    bx,by=rng*math.cos(bearing),rng*math.sin(bearing)
    return np.array([tag_xy[0]-(c*bx-s*by),tag_xy[1]-(s*bx+c*by)])


def fit_mount_yaw(frames,iterations=3):
    """Mount-yaw correction ε from tag frames taken while moving (apriltag_v5 F4 idea, solved as a landmark fit).

    frames: (bearing, range, vio_yaw, vio_xy). A fixed tag must sit at one odom point T_i = p_i + d_i·u(ψ_i+β_i+ε)
    for every frame; ε is the one unknown that makes the T_i agree. Linearised least squares, a few iterations.
    Returns (ε [rad], residual sd of the T_i [m])."""
    b=np.array([f[0] for f in frames]);d=np.array([f[1] for f in frames]);psi=np.array([f[2] for f in frames]);p=np.array([f[3] for f in frames],float)
    eps=0.
    for _ in range(iterations):
        th=psi+b+eps;T=p+d[:,None]*np.stack([np.cos(th),np.sin(th)],1);u=T-T.mean(0)
        v=d[:,None]*np.stack([-np.sin(th),np.cos(th)],1);v=v-v.mean(0)
        den=float((v*v).sum())
        if den<1e-12:return 0.,float('inf')
        eps-=float((u*v).sum())/den
    th=psi+b+eps;T=p+d[:,None]*np.stack([np.cos(th),np.sin(th)],1)
    return float(eps),float(np.sqrt(((T-T.mean(0))**2).sum(1).mean()))


def _rotate(theta,v):
    c,s=math.cos(theta),math.sin(theta)
    return np.array([c*v[0]-s*v[1],s*v[0]+c*v[1]])


class Baseline:
    """30 s stationary tag drift (block medians -> sd across blocks; halves -> the larger). Record only."""
    def __init__(self,seconds,block_s):
        self.seconds=seconds;self.block_s=block_s;self.rows=deque();self.value=None;self.frozen=False

    def add(self,stamp,bx,by,heading,rng,still,gyro_p90):
        if self.frozen:return
        self.rows.append((stamp,bx,by,heading,rng,bool(still),gyro_p90 if gyro_p90 is not None else math.inf))
        while self.rows and stamp-self.rows[0][0]>4*self.seconds:self.rows.popleft()
        still_rows=[r for r in self.rows if r[5] and stamp-r[0]<=self.seconds]
        if still_rows and still_rows[-1][0]-still_rows[0][0]>=.9*self.seconds:
            value=self._compute(still_rows,'still_%ds'%int(self.seconds))
            if value is not None:self.value=value

    def freeze(self):
        """Called when the vehicle first moves. Never blocks: quietest window, then the declared default."""
        if self.frozen:return self.value
        self.frozen=True
        if self.value is None and self.rows:
            rows=list(self.rows);best=None
            for i,start in enumerate(rows):
                window=[r for r in rows[i:] if r[0]-start[0]<=self.seconds]
                if window[-1][0]-start[0]<.9*self.seconds and i>0:continue
                score=float(np.mean([r[6] for r in window]))
                if best is None or score<best[0]:best=(score,window)
            if best:self.value=self._compute(best[1],'quietest_%ds_not_confirmed_still'%int(self.seconds))
        if self.value is None:self.value=dict(DEFAULT_BASELINE)
        return self.value

    def _compute(self,rows,source):
        t0=rows[0][0];blocks={}
        for r in rows:blocks.setdefault(int((r[0]-t0)/self.block_s),[]).append(r)
        keys=sorted(k for k,v in blocks.items() if len(v)>=1)
        if len(keys)<4:return None
        med=np.array([[np.median([r[1] for r in blocks[k]]),np.median([r[2] for r in blocks[k]]),
                       np.median(np.unwrap([r[3] for r in blocks[k]])),np.median([r[4] for r in blocks[k]])] for k in keys])
        def sd(part):
            if len(part)<2:return np.zeros(3)
            h=np.unwrap(part[:,2])
            return np.array([np.std(part[:,0]),np.std(part[:,1]),np.std(h)])
        half=len(med)//2
        s=np.max([sd(med),sd(med[:half]),sd(med[half:])],axis=0)
        return dict(longitudinal_m=float(s[0]),lateral_m=float(s[1]),heading_rad=float(s[2]),
                    distance_m=float(np.median(med[:,3])),blocks=len(keys),span_s=float(rows[-1][0]-t0),source=source,
                    certified_error_bound=False)

    def scaled(self,rng):
        """Lateral and heading drift grow with range (bearing/PnP angle noise); range noise with range² (tag pixels).
        Floors: 1 cm position (a third of the 3 cm docking tolerance: a disagreement that small can never matter for
        docking) and 0.5 deg heading. Without the floor, the 4.2 m baseline scaled to 1 m gave σ 2-5 mm and a hand-held
        run on 2026-10-19 flagged 92 of 628 tag frames as conflicts for 1-2 cm residuals; with 1 cm, none."""
        b=self.value or DEFAULT_BASELINE
        k=max(.2,rng/max(.5,b['distance_m'])) if rng is not None and math.isfinite(rng) else 1.
        return np.maximum(np.array([b['longitudinal_m']*k*k,b['lateral_m']*k,b['heading_rad']*k]),
                          [SIGMA_FLOOR_M,SIGMA_FLOOR_M,SIGMA_FLOOR_RAD])


class Fusion:
    def __init__(self,tuning,T_body_camera,T_dock_tag):
        self.t=tuning;self.Tbc=np.asarray(T_body_camera,float);self.Tdt=np.asarray(T_dock_tag,float)
        self.tag_xy=np.array([self.Tdt[0,3],self.Tdt[1,3]])
        self.baseline=Baseline(tuning['baseline_s'],tuning['tag_window_s'])
        self.reset(None)

    def reset(self,epoch):
        self.epoch=epoch;self.offset=None;self.offset_n=0;self.anchor=None;self.anchor_stamp=None
        self.anchor_jitter=np.zeros(3);self.window=deque();self.Pvio=np.zeros((3,3));self.offset_updated_at=-math.inf
        self.travel=0.;self.last_window=None;self.report=dict(reason='no_tag_window')
        self.leg=[]   # tag frames of the current moving stretch (a VIO reset ends it without a fit)
        # The mount yaw is a physical constant of this camera mounting: samples survive VIO resets.
        if not hasattr(self,'mount_yaw_samples'):self.mount_yaw_samples=[];self.mount_yaw=0.;self.last_leg=None;self.legs_seen=0

    # ---- online mount yaw (2026-10-19 user decision: learn it while driving, apriltag_v4/v5 style) -----------
    def _finish_leg(self,stamp):
        frames=self.leg;self.leg=[]
        if len(frames)<4:return
        self.legs_seen+=1
        p=np.array([f[3] for f in frames]);travel=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum())
        rng=max(f[1] for f in frames);bearing=max(abs(f[0]) for f in frames)
        good=(travel>=self.t['leg_min_travel_m'] and rng<=self.t['leg_max_range_m'] and bearing<=math.radians(self.t['leg_max_bearing_deg']))
        eps,residual=fit_mount_yaw(frames)
        reason='good_leg' if good else 'short' if travel<self.t['leg_min_travel_m'] else 'far' if rng>self.t['leg_max_range_m'] else 'oblique'
        if good and abs(eps)>math.radians(15):good=False;reason='implausible_%.0fdeg'%math.degrees(eps)   # a mount 15 deg off is visible to the eye: the fit, not the camera, is wrong
        if good:
            self.mount_yaw_samples.append(eps);new=float(np.median(self.mount_yaw_samples))
            if self.offset is not None:self.offset=wrap(self.offset-(new-self.mount_yaw))   # keep heading = vio + offset consistent
            self.mount_yaw=new
        self.last_leg=dict(stamp=stamp,frames=len(frames),travel_m=travel,max_range_m=rng,max_bearing_deg=math.degrees(bearing),
            eps_deg=math.degrees(eps),residual_m=residual,accepted=good,reason=reason,mount_yaw_deg=math.degrees(self.mount_yaw),good_legs=len(self.mount_yaw_samples))

    # ---- VIO side -------------------------------------------------------------------------------------------
    def vio_step(self,delta_xy,Q_body,heading):
        """One VIO increment (body frame) since the previous state: accumulate dock-frame variance and travel."""
        self.travel+=float(np.hypot(*delta_xy))
        if self.anchor is None:return
        R=np.eye(3);c,s=math.cos(heading),math.sin(heading);R[:2,:2]=[[c,-s],[s,c]]
        self.Pvio=self.Pvio+R@np.asarray(Q_body,float)@R.T

    def position_variance(self):
        """VIO position variance since the anchor, plus (travel × σψ)² for the heading error accumulated since then."""
        v=np.maximum(np.diag(self.Pvio),0.)
        return v[:2]+v[2]*self.travel**2

    # ---- tag side -------------------------------------------------------------------------------------------
    def tag(self,stamp,bearing,rng,psi_pnp,p_local,still,gyro_p90=None):
        """One accepted tag frame with the VIO state at its capture time. Returns the fusion report.
        bearing/psi_pnp are as measured with the configured mount; the online mount yaw is applied here."""
        psi_vio=float(p_local[2]);xy=np.asarray(p_local[:2],float)
        if still:
            if self.leg:self._finish_leg(stamp)
        else:self.leg.append((bearing,rng,psi_vio,xy.copy()))
        bearing=bearing+self.mount_yaw;psi_pnp=psi_pnp-self.mount_yaw
        self.window.append((stamp,bearing,rng,psi_pnp,psi_vio,xy,bool(still)))
        while self.window and stamp-self.window[0][0]>self.t['tag_window_s']:self.window.popleft()
        # Baseline rows: tag vector in the body frame (what β and range give directly) and the PnP heading.
        self.baseline.add(stamp,rng*math.cos(bearing),rng*math.sin(bearing),psi_pnp,rng,still,gyro_p90)
        n=len(self.window)
        if n<self.t['tag_window_min_n']:
            self.report=dict(accepted=False,reason='tag_window_filling',frames=n);return self.report
        rows=list(self.window)
        pnp_minus_vio=float(np.median(np.unwrap([wrap(r[3]-r[4]) for r in rows])))
        if self.offset is None:
            self.offset=wrap(pnp_minus_vio);self.offset_n=1;self.offset_updated_at=stamp;source='first_window'
        elif all(r[6] for r in rows):
            # PnP heading is only a weak reference: pulled toward it at rest with a small gain, at most once per
            # window span (gyro/VIO yaw drift 0.6 deg/60 s on 2026-10-01 is caught; a per-pose PnP bias of 1-3 deg
            # is followed only 5 % per 0.5 s, i.e. 20-40 % during a 2-5 s stop).
            if stamp-self.offset_updated_at>=self.t['tag_window_s']:
                self.offset=wrap(self.offset+self.t['heading_offset_gain']*wrap(pnp_minus_vio-self.offset));self.offset_n+=1
                self.offset_updated_at=stamp;source='rest_update'
            else:source='rest_hold_until_next_window'
        else:source='held_while_moving'
        pts=[];heads=[]
        for r in rows:
            h=wrap(r[4]+self.offset);heads.append(h)
            pts.append(body_in_dock(h,r[1],r[2],self.tag_xy)-_rotate(self.offset,r[5]))   # anchor candidate per frame
        pts=np.array(pts);rng_med=float(np.median([r[2] for r in rows]))
        residual=None;allowed=None;decision='first_anchor';use=pts
        if self.anchor is not None:
            # Physical plausibility instead of a statistical gate (2026-10-19 user decision): a tag window may move
            # the position by at most what the vehicle could have travelled since the last accepted window, plus a
            # range-proportional measurement margin. These are vehicle/tag constants, so the bound never shrinks to
            # zero on a quiet day (a 3σ gate fired on 0.05 mm residuals on a desk) and never blocks motion.
            allowed=self.jump_allowed(rng_med,stamp-self.anchor_stamp)
            agree=np.linalg.norm(pts-self.anchor,axis=1)<=allowed
            if agree.sum()*2>=n:use=pts[agree];decision='refreshed'
            elif stamp-self.anchor_stamp>=self.t['jump_accept_after_s']:use=pts[~agree];decision='accepted_after_skip'
            else:use=pts[~agree];decision='tag_jump_skipped'
            residual=np.median(use,axis=0)-self.anchor
        cand=np.median(use,axis=0);m=len(use)
        jitter=np.array([*(np.std(use,axis=0)/math.sqrt(m)),float(np.std(np.unwrap(heads))/math.sqrt(n))])
        if decision!='tag_jump_skipped':
            self.anchor=cand;self.anchor_stamp=stamp;self.anchor_jitter=jitter;self.Pvio=np.zeros((3,3));self.travel=0.
        self.last_window=dict(stamp=stamp,frames=n,range_m=rng_med,jitter=jitter.tolist(),still=bool(all(r[6] for r in rows)))
        self.report=dict(accepted=decision!='tag_jump_skipped',reason=decision,frames=n,range_m=rng_med,
            residual_m=None if residual is None else residual.tolist(),allowed_jump_m=allowed,
            skipped_for_s=None if self.anchor_stamp is None else stamp-self.anchor_stamp,
            heading_offset_deg=math.degrees(self.offset),heading_offset_source=source,heading_offset_updates=self.offset_n,
            pnp_minus_vio_deg=math.degrees(wrap(pnp_minus_vio)),window_jitter=jitter.tolist(),
            mount_yaw_deg=math.degrees(self.mount_yaw),good_legs=len(self.mount_yaw_samples),certified_error_bound=False)
        return self.report

    def jump_allowed(self,rng,dt):
        """Largest tag-vs-prediction disagreement that is physically plausible: measurement margin
        max(jump_margin_m, jump_margin_per_m × range) + nominal speed × time since the last accepted window."""
        return max(self.t['jump_margin_m'],self.t['jump_margin_per_m']*rng)+NOMINAL_SPEED_MPS*max(0.,dt)

    # ---- output ---------------------------------------------------------------------------------------------
    @property
    def ready(self):
        return self.anchor is not None and self.offset is not None

    def estimate(self,p_local):
        """Dock pose from the current VIO state, plus σ per dock axis [x, y, yaw] (not a certified bound)."""
        if not self.ready:return None,None,None
        heading=wrap(float(p_local[2])+self.offset)
        xy=_rotate(self.offset,np.asarray(p_local[:2],float))+self.anchor
        world=np.array([xy[0],xy[1],heading])
        rng=float(np.hypot(*(self.tag_xy-xy)))
        base=self.baseline.scaled(rng)
        vio=np.r_[self.position_variance(),max(0.,self.Pvio[2,2])]
        sigma=np.sqrt(base**2+self.anchor_jitter**2+vio)
        parts=dict(baseline=base.tolist(),window_jitter=self.anchor_jitter.tolist(),vio_since_anchor=np.sqrt(vio).tolist(),
                   travel_since_anchor_m=self.travel,baseline_source=(self.baseline.value or DEFAULT_BASELINE)['source'])
        return world,np.diag(sigma**2),parts
