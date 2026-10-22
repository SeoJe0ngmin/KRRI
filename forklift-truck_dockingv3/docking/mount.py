"""Camera mounting rotation, apriltag_v4 `calibrate camyaw` style (2026-10-19 user decision: no motion, no CAN).

The vehicle is parked roughly facing the tag normal. Over a recorded `measure` run the median PnP rotation of the
tag while the gyro says "stationary" is taken as the mounting rotation (body = dock orientation is ASSUMED, by
eye, as on 2026-10-01; v4 config note: "절대 정확도는 모른다"). The assumption is recorded as a warning, never
checked, never a gate. Only "cannot compute" (fewer than MIN_TAGS tag frames at rest) stops a save.
"""
import math
import numpy as np
import cv2
from .frames import R_OPTICAL_TO_BODY, R_REFERENCE_TAG

MIN_TAGS=3          # unique tag frames needed (apriltag_v4 MIN_N)


def tag_pose(row):
    """(position, rotation) of an accepted tag in the camera frame, or None. A held/cached tag never certifies a new interval."""
    t=row.get('tag',{})
    if not t.get('ok') or t.get('reason')!='accepted' or t.get('capture_key') is None:return None
    try:
        p=np.asarray(t['camera_translation_m'],float);v=np.asarray(t['camera_rotation_vector_rad'],float)
        if p.shape!=(3,) or v.shape!=(3,) or not np.isfinite([p,v]).all():return None
        if 'stamp' in t and abs(row['capture_id']-t['stamp'])>.3:return None
        return p,cv2.Rodrigues(v)[0]
    except (KeyError,ValueError,cv2.error):return None


def healthy(row):
    try:rate=float(row.get('vio_gyro_norm'))
    except (TypeError,ValueError):return False
    return bool(row.get('sensor_ready') and not row.get('imu_bad') and not row.get('clock_fault') and np.isfinite(rate))



def gravity_tilt(vio_rows,T_left_imu):
    """Camera pitch-down and roll relative to level from the IMU at rest (OpenVINS odom frame is gravity aligned).
    Independent of the tag; assumes the vehicle stands on level ground. Returns (pitch, roll, n) in radians."""
    down=[]
    for v in vio_rows:
        T=np.asarray(v.get('T_odom_imu'),float) if v.get('T_odom_imu') is not None else None
        if T is None or T.shape!=(4,4) or not v.get('initialized'):continue
        down.append(T[:3,:3].T@np.array([0.,0.,-1.]))   # gravity direction expressed in the IMU frame
    if len(down)<MIN_TAGS:return None,None,len(down)
    d=np.median(np.array(down),axis=0);d/=np.linalg.norm(d)
    d=np.asarray(T_left_imu,float)[:3,:3]@d              # into the left-IR optical frame (x right, y down, z forward)
    # level camera: down = +y. Pitched down: down leans toward +z (forward). Rolled: down leans toward ±x.
    # d = (sin r cos p, cos r cos p, sin p) for R = Ry(p) Rx(r) on the optical frame -> exact inverse:
    return math.atan2(d[2],math.hypot(d[0],d[1])),math.atan2(d[0],d[1]),len(down)


def estimate(rows,sensor,model,vio_rows=None,T_left_imu=None):
    """apriltag_v4 camyaw: median tag rotation while parked facing the tag normal (assumed, not measured) -> yaw.
    Pitch/roll come from gravity (IMU) when vio_rows are given, otherwise 0 (level camera)."""
    import uuid
    report=dict(schema='camera_mount_reference_v1',id=uuid.uuid4().hex,usable=False,method='standing_tag_yaw_imu_pitch_roll_assumes_square_to_tag',
        certified_error_bound=False,reference_alignment_confirmed=False,camera_model=model,reasons=[],
        warnings=['assumes_vehicle_parallel_to_tag_normal_by_eye'],assumptions=['upright_tag','vehicle_level','body_parallel_to_tag_normal'])
    warn=report['warnings'].append
    try:
        poses={}
        for r in rows:
            if not r.get('stationary'):continue
            pose=tag_pose(r)
            if pose is not None:poses[r['tag']['capture_key']]=(r['arrival'],pose)
        if len(poses)<MIN_TAGS:raise ValueError('정지 중 태그 관측 부족 (고유 관측 %d개 필요, %d개)'%(MIN_TAGS,len(poses)))
        items=sorted(poses.values());span=items[-1][0]-items[0][0];report.update(accepted=len(items),rest_s=span)
        if span<30:warn('rest_short_%.0fs_v4_used_60s'%span)
        # One vote per 0.5 s block (v4 WINDOW_S): a burst of frames cannot dominate the median.
        t0=items[0][0];blocks={}
        for t,(xyz,Rct) in items:blocks.setdefault(int((t-t0)/.5),[]).append((xyz,Rct))
        def centre(mats):
            U,_,V=np.linalg.svd(np.mean(mats,axis=0));return U@np.diag([1.,1.,np.linalg.det(U@V)])@V
        rotations=[centre([R_REFERENCE_TAG@Rct.T for _,Rct in b]) for b in blocks.values()]
        R=centre(rotations)
        spread=[math.degrees(math.acos(float(np.clip((np.trace(r@R.T)-1)/2,-1,1)))) for r in rotations]
        half=len(rotations)//2
        change=0. if half<1 else math.degrees(math.acos(float(np.clip((np.trace(centre(rotations[:half])@centre(rotations[half:]).T)-1)/2,-1,1))))
        report.update(blocks=len(rotations),block_spread_p95_deg=float(np.percentile(spread,95)),first_last_half_change_deg=change)
        if report['block_spread_p95_deg']>1:warn('block_spread_%.2fdeg'%report['block_spread_p95_deg'])
        if change>.5:warn('half_change_%.2fdeg'%change)
        D=R@R_OPTICAL_TO_BODY.T
        tag_pitch=math.atan2(-D[2,0],math.hypot(D[0,0],D[1,0]));tag_roll=math.atan2(D[2,1],D[2,2])
        # Yaw from the tag (v4 CAM_YAW_OFFSET_DEG). Pitch/roll from gravity: a standing tag cannot tell camera tilt from
        # tag tilt (desk run 2026-10-19: a leaning sheet gave -23 deg), the IMU can (v4 measured camera pitch the same way).
        pitch=roll=0.;source='level_assumed'
        if vio_rows is not None and T_left_imu is not None:
            gp,gr,n=gravity_tilt(vio_rows,T_left_imu)
            if gp is not None:pitch,roll,source=gp,gr,'imu_gravity_%d_samples'%n
            else:warn('no_imu_attitude_in_log_camera_assumed_level')
        else:warn('camera_assumed_level')
        if abs(tag_pitch-pitch)>math.radians(5) or abs(tag_roll-roll)>math.radians(5):
            warn('tag_tilted_vs_gravity_%.0f_%.0fdeg'%(math.degrees(tag_pitch-pitch),math.degrees(tag_roll-roll)))
        def rz(a):c,s_=math.cos(a),math.sin(a);return np.array([[c,-s_,0.],[s_,c,0.],[0.,0.,1.]])
        def ry(a):c,s_=math.cos(a),math.sin(a);return np.array([[c,0.,s_],[0.,1.,0.],[-s_,0.,c]])
        def rx(a):c,s_=math.cos(a),math.sin(a);return np.array([[1.,0.,0.],[0.,c,-s_],[0.,s_,c]])
        # The tag-derived rotation is R_meas = T_lean · Rz(yaw) Ry(p) Rx(r) R_OTB, where T_lean is the tag's own lean
        # about horizontal axes (unknown) and p, r are the camera's true tilt (from gravity). Peel the known part off and
        # factor the rest as Ry Rx Rz: the Rz is the yaw, unbiased by the lean (an Euler split of R_meas is not:
        # a 20 deg lean turned 7 deg into 7.44 deg in the synthetic check).
        M=R@(ry(pitch)@rx(roll)@R_OPTICAL_TO_BODY).T
        ty=math.atan2(M[0,2],M[2,2]);tx=-math.asin(float(np.clip(M[1,2],-1,1)))
        Z=rx(tx).T@ry(ty).T@M;yaw=math.atan2(Z[1,0],Z[0,0])
        report.update(tag_lean_deg=[math.degrees(ty),math.degrees(tx)])
        R=rz(yaw)@ry(pitch)@rx(roll)@R_OPTICAL_TO_BODY
        sigma=max(sensor['uncertainty']['mount_rotation_std_rad'],math.radians(report['block_spread_p95_deg']))
        report.update(usable=True,R_body_camera=R.tolist(),yaw_left_deg=math.degrees(yaw),pitch_roll_source=source,
            pitch_down_deg=math.degrees(pitch),roll_deg=math.degrees(roll),tag_pitch_down_deg=math.degrees(tag_pitch),tag_roll_deg=math.degrees(tag_roll),
            rotation_std_deg=math.degrees(sigma),covariance=np.diag([sensor['uncertainty']['mount_position_std_m']**2]*3+[sigma*sigma]*3).tolist())
    except ValueError as exc:report['reasons'].append(str(exc))
    return report
