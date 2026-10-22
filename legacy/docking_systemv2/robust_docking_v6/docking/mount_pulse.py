"""One-shot CAN mounting experiment. No automatic retry or return motion."""
import math
import numpy as np
import cv2
from .straight_mount import healthy, tag_pose
from .frames import R_OPTICAL_TO_BODY


def endpoint(rows):
    unique={}
    for r in rows:
        pose=tag_pose(r)
        if pose is not None:unique[r['tag']['capture_key']]=(r,pose)
    values=list(unique.values())
    if len(values)<10:raise ValueError('태그 관측 부족 (고유 관측 10개 필요)')
    duration=rows[-1]['arrival']-rows[0]['arrival']
    covered=len({int(r['arrival']-rows[0]['arrival']) for r,_ in values})
    if covered<.7*duration:raise ValueError('정지 구간 태그 시간 커버리지 70% 미만')
    pts=np.array([p[0] for _,p in values])
    center=np.median(pts,axis=0)
    spread=float(np.percentile(np.linalg.norm(pts-center,axis=1),95))
    return center,spread,np.array([p[1][:,1] for _,p in values])


def estimate(rows,start,end,sensor,model,stop_capture=None):
    import uuid
    report=dict(schema='camera_mount_reference_v1',id=uuid.uuid4().hex,usable=False,
        method='stationary_30s_can_forward_3s',certified_error_bound=False,
        reference_alignment_confirmed=False,camera_model=model,reasons=[],
        command_seconds=3.,assumptions=['upright_tag','level_forward_travel','no_sideslip'])
    try:
        if not start or not end:raise ValueError('정지 측정 미완료')
        if start[-1]['arrival']-start[0]['arrival']<29:raise ValueError('시작 정지 측정 시간 부족')
        for part in (start,end):
            if sum(bool(r.get('stationary')) and healthy(r) for r in part)/len(part)<.8:
                raise ValueError('정지 관측 비율 80% 미만')
        sensor_epochs={r.get('sensor_epoch') for r in rows}
        motion_epochs={r.get('motion_epoch') for r in rows}
        report['sensor_epoch_count']=len(sensor_epochs)
        report['motion_epoch_count']=len(motion_epochs)
        if len(sensor_epochs)!=1:raise ValueError('센서 시간 기준 재설정; 구간 연결 불가')
        # The mount translation uses raw camera/tag coordinates, not local VIO
        # coordinates. A reset during baseline or settling need not invalidate it.
        moving=[r for r in rows if r['capture_id']>=start[-1]['capture_id']
                and (stop_capture is None or r['capture_id']<=stop_capture)]
        if len({r.get('motion_epoch') for r in moving})>1:
            raise ValueError('전진 중 추적 기준 재설정; 직진 회전 검증 불가')
        report['rest_only_motion_reset_tolerated']=len(motion_epochs)>1
        valid=[r for r in rows if r.get('local_valid')]
        movement=[r for r in rows if stop_capture is None or r['capture_id']<=stop_capture]
        if not movement or sum(bool(r.get('local_valid')) for r in movement)<.8*len(movement):
            raise ValueError('유효 이동 추적 부족')
        # Never subtract headings belonging to different local coordinate frames.
        for epoch in motion_epochs:
            yaw=np.unwrap([r['local_pose'][2] for r in valid if r.get('motion_epoch')==epoch])
            if len(yaw)>1 and np.ptp(yaw)>math.radians(2):raise ValueError('직진 중 회전 2도 초과')
        def representative_rotation(part):
            poses={r['tag']['capture_key']:tag_pose(r) for r in part if tag_pose(r) is not None}
            if len(poses)<10:raise ValueError('태그 자세 관측 부족')
            U,_,V=np.linalg.svd(np.mean([p[1] for p in poses.values()],axis=0))
            return U@np.diag([1.,1.,np.linalg.det(U@V)])@V
        Ra,Rb=representative_rotation(start),representative_rotation(end)
        angle=math.degrees(math.acos(float(np.clip((np.trace(Rb@Ra.T)-1)/2,-1,1))))
        report['endpoint_rotation_change_deg']=angle
        if angle>2:raise ValueError('시작·도착 대표 태그 자세 변화 2도 초과')
        a,sa,ua=endpoint(start);b,sb,ub=endpoint(end)
        delta=a-b;span=float(np.linalg.norm(delta));report['travel_span_m']=span
        if span<.15 or span>2:raise ValueError('관측 이동 거리 0.15~2m 범위 밖')
        up=np.median(np.vstack([ua,ub]),axis=0);up/=np.linalg.norm(up)
        forward=delta/span
        if up[1]>-math.cos(math.radians(30)) or forward[2]<math.cos(math.radians(30)):
            raise ValueError('태그 수직/전진 방향 확인 실패')
        if abs(up@forward)>math.sin(math.radians(3)):raise ValueError('수평 직진 확인 실패')
        forward-=up*(up@forward);forward/=np.linalg.norm(forward)
        left=np.cross(up,forward);left/=np.linalg.norm(left);up=np.cross(forward,left)
        R=np.array([forward,left,up]);D=R@R_OPTICAL_TO_BODY.T
        # Keep endpoint dispersion as systematic uncertainty; never divide by sqrt(N).
        sigma=max(sensor['uncertainty']['mount_rotation_std_rad'],math.atan2(sa+sb,span))
        report.update(endpoint_spread_m=[sa,sb],rotation_std_deg=math.degrees(sigma))
        if sigma>math.radians(5):raise ValueError('이동량 대비 태그 산포가 너무 큼 (방향 불확실성 5도 초과)')
        report.update(usable=True,R_body_camera=R.tolist(),yaw_left_deg=math.degrees(math.atan2(D[1,0],D[0,0])),
            covariance=np.diag([sensor['uncertainty']['mount_position_std_m']**2]*3+[sigma*sigma]*3).tolist())
    except ValueError as exc:report['reasons'].append(str(exc))
    return report


class Pulse:
    def __init__(self):
        self.phase='WAIT_ENTER';self.command='stop';self.rows=[];self.start=[];self.end=[]
        self.since=None;self.last=None;self.stable_since=None;self.done=False
        self.wait_reason=None;self.stop_capture=None
    def __call__(self,row):return self.done
    def tick(self,now,row,enter=False):
        fresh=row is not None and 'capture_id' in row and not row.get('preview_only')
        if fresh:self.last=row
        if self.phase=='WAIT_ENTER':
            if enter:
                if not self.last or not healthy(self.last) or not self.last.get('local_valid'):
                    print('센서 준비 후 Enter를 다시 누르세요.',flush=True)
                else:
                    self.phase='BASELINE';self.since=now
                    print('정지 30초 측정 시작. 완료하면 자동으로 3초 전진합니다.',flush=True)
            return
        if fresh:self.rows.append(row)
        if self.phase=='BASELINE':
            if fresh:self.start.append(row)
            if now-self.since>=30:
                if not self.start or sum(healthy(r) and bool(r.get('stationary')) for r in self.start)/len(self.start)<.8:
                    raise RuntimeError('시작 정지 관측 부족; 자동 전진하지 않습니다')
                endpoint(self.start)
                self.phase='FORWARD';self.since=now;self.command='forward'
                print('CAN 전진 3초 시작 (기존 forward 바이트 67).',flush=True)
        elif self.phase=='FORWARD':
            if now-self.since>=3:
                self.command='stop';self.phase='SETTLE';self.since=now;self.end=[]
                self.stable_since=None;self.stop_capture=self.last['capture_id'] if self.last else None
                print('정지 명령 전송. 잔여 이동·되튐 확인 중.',flush=True)
        elif self.phase=='SETTLE':
            self.command='stop'
            if now-self.since>=60:
                raise RuntimeError('정지 후 관측 확인 60초 초과; 보정 미저장: '+str(self.wait_reason))
            reason=self.observation_problem(now)
            if not reason and not self.last.get('stationary'):reason='잔여 이동·되튐 확인 중'
            # A silent stream gap must reset the dwell even without a new row.
            if reason:
                self.stable_since=None;self.end=[]
            elif fresh:
                if self.end and row['arrival']-self.end[-1]['arrival']>.4:
                    self.stable_since=None;self.end=[]
                if self.stable_since is None:self.stable_since=now
                self.end.append(row)
                if now-self.stable_since>=3:
                    try:endpoint(self.end)
                    except ValueError as exc:reason=str(exc)
                    else:self.done=True;self.phase='COMPLETE'
            if reason!=self.wait_reason:
                print('정지 유지: '+(reason or '추적 회복, 안정 관측 3초 수집'),flush=True)
                self.wait_reason=reason
        if self.phase=='FORWARD':
            reason=self.observation_problem(now)
            if reason:
                self.command='stop';raise RuntimeError('직진 보정 중단: '+reason)

    def observation_problem(self,now):
        if not self.last or now-self.last['arrival']>.4:return '관측 지연 (0.4초 초과)'
        if not healthy(self.last):return 'IMU 또는 센서 상태 무효'
        if not self.last.get('local_valid'):
            return '위치 추적 무효: '+self.last.get('odom',{}).get('reason','unknown')
        return None
