"""Manual forward-leg mounting calibration, inspired by apriltag_v4 aim.fit_leg.

Uses the trajectory of tag translations (range/bearing) rather than assuming
vehicle heading relative to the tag. Requires an upright tag and level straight
forward travel without sideslip. Fits effective travel alignment, not a certified
mechanical extrinsic. Never issues CAN commands.
"""
import math
import uuid
import cv2
import numpy as np


def tag_pose(row):
    t=row.get('tag',{})
    if not t.get('ok') or t.get('reason')!='accepted' or t.get('capture_key') is None:return None
    try:
        p=np.asarray(t['camera_translation_m'],float);v=np.asarray(t['camera_rotation_vector_rad'],float)
        if p.shape!=(3,) or v.shape!=(3,) or not np.isfinite([p,v]).all():return None
        # A held/cached tag cannot certify a new interval.
        if 'stamp' in t and abs(row['capture_id']-t['stamp'])>.3:return None
        return p,cv2.Rodrigues(v)[0]
    except (KeyError,ValueError,cv2.error):return None


def healthy(row):
    try:rate=float(row.get('vio_gyro_norm'))
    except (TypeError,ValueError):return False
    return bool(row.get('sensor_ready') and not row.get('imu_bad') and not row.get('clock_fault')
        and np.isfinite(rate))


def tracking_ok(rows):
    """Only <=0.5s correction-stale intervals may use fresh tag + IMU backup."""
    bad_start=None
    for r in rows:
        if not healthy(r):return False
        if r.get('local_valid'):
            if bad_start is not None and r['arrival']-bad_start>.5:return False
            bad_start=None
        else:
            if r.get('odom',{}).get('reason')!='correction_stale' or tag_pose(r) is None:return False
            if bad_start is None:bad_start=r['arrival']
            if r['arrival']-bad_start>.5:return False
    return True


def stationary_dwell(rows):
    """Confirm rest from fresh tag geometry and gyro, independent of boolean resets."""
    if len(rows)<2 or rows[-1]['arrival']-rows[0]['arrival']<3:return False
    if not tracking_ok(rows) or any(b['arrival']-a['arrival']>.4 for a,b in zip(rows,rows[1:])):return False
    unique={}
    for r in rows:
        pose=tag_pose(r)
        if pose is None:return False
        unique[r['tag']['capture_key']]=(r['arrival'],pose)
    entries=list(unique.values())
    if len(entries)<6 or entries[-1][0]-entries[0][0]<2.5:return False
    pts=np.array([x[1][0] for x in entries]);rots=[x[1][1] for x in entries]
    if np.max(np.linalg.norm(pts-np.median(pts,axis=0),axis=1))>.01:return False
    angles=[math.degrees(math.acos(float(np.clip((np.trace(R@rots[0].T)-1)/2,-1,1)))) for R in rots]
    return max(angles)<=.5 and max(abs(r['vio_gyro_norm']) for r in rows)<=math.radians(1.)


def latest_dwell(rows):
    if not rows:return []
    end=rows[-1]['arrival'];start=len(rows)-1
    while start>0 and end-rows[start]['arrival']<3:start-=1
    return rows[start:]


class Collection:
    """One explicitly announced forward leg. Invalid legs never poison later ones."""
    def __init__(self):
        self.rows=[];self.ready=False;self.resets=0;self.last_reset=None
    def reset(self):
        self.rows=[];self.ready=False
    def add(self,row):
        if 'capture_id' not in row:return None
        reason=None
        if self.rows:
            old=self.rows[-1];dt=row['arrival']-old['arrival']
            if not 0<dt<=.4:reason='observation_gap'
            elif any(old.get(k)!=row.get(k) for k in ('sensor_epoch','motion_epoch')):reason='tracking_reset'
        recent=latest_dwell(self.rows+[row])
        if not tracking_ok(recent):reason=reason or 'tracking_unavailable'
        if self.ready:
            observed=[r for r in self.rows if tag_pose(r) is not None]
            if observed and row['arrival']-observed[-1]['arrival']>1.:reason=reason or 'tag_gap'
        if reason:
            was_ready=self.ready;self.reset();self.resets+=1;self.last_reset=reason
            if healthy(row) and row.get('local_valid'):self.rows.append(row)
            return 'restart' if was_ready else None
        self.rows.append(row)
        if not self.ready:
            self.rows=latest_dwell(self.rows)
            if stationary_dwell(self.rows):self.ready=True;return 'ready'
        return None


def estimate(rows, sensor, model):
    out=dict(schema='camera_mount_reference_v1',id=uuid.uuid4().hex,usable=False,
        method='manual_forward_tag_trajectory',certified_error_bound=False,
        reference_alignment_confirmed=False,camera_model=model,reasons=[],
        assumptions=['upright_tag','level_forward_travel','no_sideslip'])
    def fail(reason):
        out['reasons'].append(reason);return out
    window=rows
    if not window:return fail('waiting_for_initial_stationary_tag_3s')
    initial=[]
    for row in window:
        initial.append(row)
        if row['arrival']-window[0]['arrival']>=3:break
    if not stationary_dwell(initial):return fail('waiting_for_initial_stationary_tag_3s')
    for key in ('sensor_epoch','motion_epoch'):
        if len({r.get(key) for r in window})!=1:return fail('tracking_reset_repeat_straight_measurement')
    if not tracking_ok(window):return fail('tracking_lost_repeat_straight_measurement')
    if not stationary_dwell(latest_dwell(window)):return fail('waiting_for_final_stationary_3s')
    times=np.array([r['arrival'] for r in window])
    if np.any(np.diff(times)<=0) or np.max(np.diff(times))>.4:return fail('observation_gap')
    seen=set();points=[];rotations=[];stamps=[]
    for r in window:
        t=r.get('tag',{});key=t.get('capture_key')
        if not t.get('ok') or t.get('reason')!='accepted' or key is None or key in seen:continue
        seen.add(key)
        p=np.asarray(t.get('camera_translation_m'),float);rv=np.asarray(t.get('camera_rotation_vector_rad'),float)
        if p.shape!=(3,) or rv.shape!=(3,) or not np.isfinite([p,rv]).all():return fail('invalid_tag_pose')
        points.append(p);rotations.append(cv2.Rodrigues(rv)[0]);stamps.append(r['arrival'])
    if len(points)<30:return fail('too_few_tags')
    if max(np.diff([times[0],*stamps,times[-1]]))>1.:return fail('tag_gap')
    pts=np.array(points);rots=np.array(rotations);stamps=np.array(stamps)
    # Constant orientation is required, not inferred from the line fit alone.
    angles=[math.degrees(math.acos(float(np.clip((np.trace(R@rots[0].T)-1)/2,-1,1)))) for R in rots]
    local=np.array([r['local_pose'] for r in window if r.get('local_valid')])
    if max(angles)>2. or np.ptp(np.unwrap(local[:,2]))>math.radians(1.):return fail('rotation_during_straight_leg')
    bins=np.floor((stamps-stamps[0])/.5).astype(int)
    centers=np.array([np.median(pts[bins==b],axis=0) for b in sorted(set(bins)) if (bins==b).sum()>=2])
    if len(centers)<8:return fail('insufficient_time_coverage')
    start=np.median(pts[stamps<=stamps[0]+2],axis=0)
    end=np.median(pts[stamps>=stamps[-1]-2],axis=0)
    delta=start-end;span=float(np.linalg.norm(delta));out['travel_span_m']=span
    if span<.6:return fail('forward_travel_too_short_min_0_6m')
    if span>2.:return fail('forward_travel_too_long_max_2m')
    up=np.median(rots[:,:,1],axis=0);up/=np.linalg.norm(up)
    if up[1]>-math.cos(math.radians(30)):return fail('tag_not_upright_or_mount_tilt_over_30deg')
    _,_,vh=np.linalg.svd(centers-np.mean(centers,axis=0),full_matrices=False)
    forward=vh[0]
    if forward@delta<0:forward=-forward
    if forward[2]<math.cos(math.radians(30)):return fail('not_forward_or_mount_over_30deg')
    if abs(forward@up)>math.sin(math.radians(3)):return fail('nonlevel_travel_or_tag_not_upright')
    forward-=up*(forward@up);forward/=np.linalg.norm(forward)
    left=np.cross(up,forward);left/=np.linalg.norm(left);up=np.cross(forward,left)
    Rbc=np.array([forward,left,up])
    residual=centers-centers[0];residual-=np.outer(residual@forward,forward)
    p95=float(np.percentile(np.linalg.norm(residual,axis=1),95));out['line_residual_p95_m']=p95
    if p95>.015:return fail('trajectory_not_straight_or_tag_noise')
    center_progress=(start-centers)@forward
    if np.any(np.diff(center_progress)<-.03):return fail('reversal_during_forward_leg')
    progress=(start-pts)@forward
    # Distinct spatial portions must independently agree: a small residual alone
    # must not turn a short/noisy or gently curved trajectory into exact yaw.
    directions=[]
    for lo,hi in ((.1,.5),(.5,.9)):
        part=pts[(progress>=span*lo)&(progress<=span*hi)]
        if len(part)<6:return fail('insufficient_moving_samples')
        _,_,v=np.linalg.svd(part-part.mean(axis=0),full_matrices=False)
        d=v[0];d=d if d@forward>=0 else -d;directions.append(d)
    disagreement=math.degrees(math.acos(float(np.clip(directions[0]@directions[1],-1,1))))
    out['segment_direction_difference_deg']=disagreement
    if disagreement>2.:return fail('inconsistent_forward_direction')
    from .frames import R_OPTICAL_TO_BODY
    deltaR=Rbc@R_OPTICAL_TO_BODY.T
    sigma=max(sensor['uncertainty']['mount_rotation_std_rad'],math.atan2(p95,span*.4),math.radians(disagreement))
    out.update(usable=True,R_body_camera=Rbc.tolist(),yaw_left_deg=math.degrees(math.atan2(deltaR[1,0],deltaR[0,0])),
        block_spread_p95_deg=math.degrees(sigma),accepted=len(pts),
        selected_start_capture=window[0]['capture_id'],selected_end_capture=window[-1]['capture_id'],
        covariance=np.diag([sensor['uncertainty']['mount_position_std_m']**2]*3+[sigma*sigma]*3).tolist())
    return out
