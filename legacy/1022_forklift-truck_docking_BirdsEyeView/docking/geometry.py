"""Dock frame: x into bay, y left, yaw CCW. Body origin at camera station."""
import math
import numpy as np

def finite(v):
    try:
        return all(math.isfinite(float(x)) for x in v)
    except (TypeError, ValueError):
        return False

def wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi

def matrix(p):
    x, y, h = p
    c, s = math.cos(h), math.sin(h)
    return np.array([[c,-s,0,x],[s,c,0,y],[0,0,1,0],[0,0,0,1.]])

def pose(T):
    return np.array([T[0,3], T[1,3], math.atan2(T[1,0],T[0,0])])

def moved(p, turn, distance, pivot):
    # Nominal geometry for candidate planning ONLY. Never used as observation.
    q = np.array(p, float).copy()
    c, s = math.cos(q[2]), math.sin(q[2])
    center = q[:2] + np.array([[c,-s],[s,c]]) @ np.array(pivot)
    q[2] = wrap(q[2]+turn)
    c, s = math.cos(q[2]), math.sin(q[2])
    q[:2] = center - np.array([[c,-s],[s,c]]) @ np.array(pivot)
    q[:2] += distance*np.array([c,s])
    return q

def front(p, cfg):
    f = cfg['site']['camera_to_fork_tip_m']
    return np.array(p[:2])+f*np.array([math.cos(p[2]),math.sin(p[2])])

def remaining(p, cfg):
    return cfg['target_front_x']-front(p,cfg)[0]

def path_bias(cfg):
    """Measured drift of a straight drive, radians, left +. apriltag/KWU FORWARD_PATH_BIAS_DEG (2026-10-21): the vehicle
    does not track its heading exactly (2026-10-20 blind legs: 1-2 deg to the right, every run). Empty/0 = no compensation."""
    v = (cfg.get('site') or {}).get('forward_path_bias_deg')
    return math.radians(float(v)) if v is not None and math.isfinite(float(v)) else 0.

def arrival_y(p, cfg):
    h = p[2]+path_bias(cfg)          # direction the vehicle actually travels when driven straight
    c = math.cos(h)
    if c <= .2:
        return math.inf
    # Forward projection to target plane, including the fork lever arm.
    return p[1]+(remaining(p,cfg)/c+cfg['site']['camera_to_fork_tip_m'])*math.sin(h)

def inside_truck(p, cfg):
    """Only the truck bed's side walls are constrained; the outside approach is operator-cleared."""
    w, bw = cfg['site']['vehicle_width_m'], cfg['site']['truck_inner_width_m']
    if w is None or bw is None:
        return False
    T=matrix(p); f=cfg['site']['camera_to_fork_tip_m']; r=cfg['site']['rear_extent_m']
    corners=[(T @ [x,y,0,1])[:2] for x,y in [(f,w/2),(f,-w/2),(-r,-w/2),(-r,w/2)]]
    clipped=[]
    for a,b in zip(corners,corners[1:]+corners[:1]):
        if a[0]>=0: clipped.append(a)
        if (a[0]<0)!=(b[0]<0):
            clipped.append(a+(b-a)*(-a[0]/(b[0]-a[0])))
    return all(abs(q[1])<=bw/2 for q in clipped)

def path_clear(p, turn, distance, cfg):
    # Sample nominal sweep, not a claim of a certified swept error envelope.
    n=max(1,int(abs(turn)/math.radians(2))+1)
    for i in range(n+1):
        if not inside_truck(moved(p,turn*i/n,0,cfg['pivot']),cfg): return False
    q=moved(p,turn,0,cfg['pivot']);n=max(1,int(abs(distance)/.025)+1)
    return all(inside_truck(moved(q,0,distance*i/n,cfg['pivot']),cfg) for i in range(n+1))

def tag_margin(p, cfg):
    """Predicted tag edge margin; distortion is evaluated using the measured model."""
    model=cfg['sensor']['calibration'].get('camera_model')
    if not model: return -math.inf
    import cv2
    Tcb=np.linalg.inv(np.array(cfg['sensor']['calibration']['T_body_camera']))
    Tct=Tcb @ np.linalg.inv(matrix(p)) @ np.array(cfg['sensor']['calibration']['T_dock_tag'])
    a=cfg['site']['tag_size_m']/2
    corners=np.array([[-a,a,0],[a,a,0],[a,-a,0],[-a,-a,0]])
    xyz=corners @ Tct[:3,:3].T+Tct[:3,3]
    if xyz[:,2].min()<=.1:return -math.inf
    K=np.array([[model['fx'],0,model['cx']],[0,model['fy'],model['cy']],[0,0,1.]])
    uv=cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),K,np.array(model['coeffs']))[0].reshape(-1,2)
    return float(min(uv[:,0].min(),uv[:,1].min(),model['width']-1-uv[:,0].max(),model['height']-1-uv[:,1].max()))

def cut_position(cfg):
    target=cfg['target_front_x']-cfg['site']['camera_to_fork_tip_m']
    xs=np.arange(-15.,min(target, -.02)+.01,.025)
    good=[x for x in xs if tag_margin([x,0,0],cfg)>=cfg['tuning']['cut_margin_px']]
    if not good: raise ValueError('중심선에서 태그 가시 구간을 찾지 못했습니다. 태그 위치/장착각을 확인하세요.')
    return min(max(good),target)
