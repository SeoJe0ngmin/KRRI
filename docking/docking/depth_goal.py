"""Depth measurements for docking (2026-10-30, `docking`). Everything is read in the BIRD'S-EYE view: the depth frame
becomes 3-D body-frame points, a height band keeps the structures of interest, and each reference is looked for in a DEPTH
WINDOW around where the fused pose says it must be (the finder never searches the whole map, so open doors, the scissor lift
and the far structures cannot be mistaken for each other):

  doors         the two innermost vertical lines of the opened doors (the hinge lines) in the slab around the ENTRANCE LINE:
                left innermost member, right innermost member, their gap = truck_inner_width_m. Lateral + range.
  lattice_pair  the two vertical members of the rear lattice that bound openings 2 and 3 (their facing inner faces are
                lattice_pair_width_m apart) in the slab around the lattice (truck_depth_m). Lateral (+ centre offset) + range.
  lattice_line  the lattice's bottom bar as a frontal line in its own height band: slope = truck axis, range to the lattice.

The 2026-10-22 cluster-pair finder ("innermost edge of the front 35 cm of a blob") and the 2026-10-21 1-D band finder are
gone: at KRRI the first read the free end of a swung-open door and the second found gaps of any width. Pure geometry here;
no sensors, no control.
"""
import math
import numpy as np


# ---------------------------------------------------------------------------------------------------------------------
# shared helpers
def bev_points(depth_m,intr,T_body_ir,step=2,max_m=6.,z_min=.05,z_max=1.6,x_min=.2):
    """Depth -> body-frame points between z_min and z_max above the floor and at least x_min ahead (N x 3), with their pixels."""
    from .bev import depth_to_points,camera_to_body
    pts,u,v=depth_to_points(depth_m,intr,step,max_m)
    T=np.asarray(T_body_ir,float);body=camera_to_body(pts,T[:3,:3],T[:3,3])
    keep=(body[:,2]>=z_min)&(body[:,2]<=z_max)&(body[:,0]>=x_min)
    return body[keep],u[keep],v[keep]


def expected_column(pose,intr,T_body_ir):
    """Pixel column where the entrance centre (dock origin) should appear for the fused dock pose, in the depth/IR image.
    None when the centre is not in front of the camera."""
    if pose is None:return None
    T=np.asarray(T_body_ir,float)
    c,s=math.cos(pose[2]),math.sin(pose[2])
    body=np.array([c*(-pose[0])+s*(-pose[1]),-s*(-pose[0])+c*(-pose[1]),T[2,3]])   # dock origin in the body frame, at camera height
    cam=T[:3,:3].T@(body-T[:3,3])
    if cam[2]<=.3:return None
    return float(intr['cx']+intr['fx']*cam[0]/cam[2])


def body_to_column(pt_body,intr,T_body_ir):
    """Body point -> depth/IR image column (for drawing)."""
    T=np.asarray(T_body_ir,float);cam=T[:3,:3].T@(np.asarray(pt_body,float)-T[:3,3])
    if cam[2]<=.1:return None
    return float(intr['cx']+intr['fx']*cam[0]/cam[2])


def corridor_cells(grid,vehicle_half_width_m,cell_m=.05,forward_m=8.,half_width_m=3.,x_from=.3,x_to=2.5,margin_m=.05):
    """Occupied cells inside the vehicle's straight-ahead corridor (record only; an obstacle stop is a deferred item)."""
    rows,cols=grid.shape;r=np.arange(rows);cc=np.arange(cols)
    x=(rows-1-r+.5)*cell_m;y=half_width_m-(cc+.5)*cell_m
    sel=np.ix_((x>=x_from)&(x<=x_to),np.abs(y)<=vehicle_half_width_m+margin_m)
    return int((grid[sel]>=2).sum())


def members(pts,bin_m=.05,min_points=4):
    """Vertical members seen from above: runs of occupied lateral (y) bins with at least min_points points each. Each member
    carries its lateral faces (y_lo, y_hi), centre, median range and point count."""
    if pts is None or not len(pts):return []
    bins={}
    for x,y in zip(pts[:,0],pts[:,1]):bins.setdefault(int(math.floor(y/bin_m)),[]).append((x,y))
    keys=sorted(k for k,v in bins.items() if len(v)>=min_points)
    runs=[];run=[]
    for k in keys:
        if run and k!=run[-1]+1:runs.append(run);run=[]
        run.append(k)
    if run:runs.append(run)
    out=[]
    for run in runs:
        xy=np.array([v for k in run for v in bins[k]]);y_lo=float(np.percentile(xy[:,1],3));y_hi=float(np.percentile(xy[:,1],97))   # faces from the points, not the bin edges
        out.append(dict(y_lo=y_lo,y_hi=y_hi,yc=float((y_lo+y_hi)/2),x=float(np.median(xy[:,0])),n=int(len(xy)),bins=int(len(run))))
    return out


# ---------------------------------------------------------------------------------------------------------------------
def pair_in_window(body,expected_x_m,window_m,width_m,width_tol_m,heading_rad=0.,bin_m=.05,min_points=4,y_max=1.5,inner_min_m=.3,x_min=.3):
    """The doors' inner vertical lines: in the slab |x - expected_x_m| <= window_m take the INNERMOST vertical member on each
    side of the body axis (the hinge-side edge of each opened door; the door surface, the side walls and anything deeper lie
    outside the slab or further out). Their lateral gap must be width_m +- width_tol_m, and their range difference, after the
    body's own yaw is taken out (tan(heading) x gap), must stay within the slab. Returns dict(ok, reason, x_body_m (range to
    the line at the pair centre), x0_m (range straight ahead), lateral_body_m (centre, left +), width_m, pair_heading_deg
    (body heading relative to the line normal, left +), left_body, right_body, n_left, n_right, n)."""
    out=dict(ok=False,reason='no_points',n=0)
    if body is None or not len(body):return out
    sel=body[(np.abs(body[:,0]-float(expected_x_m))<=window_m)&(np.abs(body[:,1])<=y_max)&(body[:,0]>=x_min)]
    out['n']=int(len(sel));out['expected_x_m']=float(expected_x_m)
    if len(sel)<2*min_points:out['reason']='too_few_points_in_window';return out
    ml=members(sel[sel[:,1]>=inner_min_m],bin_m,min_points);mr=members(sel[sel[:,1]<=-inner_min_m],bin_m,min_points)
    if not ml or not mr:out['reason']='no_member_on_'+('left' if not ml else 'right');return out
    L=min(ml,key=lambda m:m['y_lo']);R=max(mr,key=lambda m:m['y_hi'])   # innermost on each side
    width=L['y_lo']-R['y_hi']
    if abs(width-width_m)>width_tol_m:out.update(reason='width_%.2f_not_%.2f'%(width,width_m),width_m=float(width));return out
    gap=L['y_lo']-R['y_hi'];dx=(L['x']-R['x'])-math.tan(heading_rad)*gap
    if abs(dx)>window_m:out.update(reason='depth_mismatch_%.2fm'%dx,width_m=float(width));return out
    slope=(L['x']-R['x'])/max(1e-6,gap)                 # dx/dy of the line in the body frame = tan(body heading vs the line normal)
    yc=(L['y_lo']+R['y_hi'])/2;x0=L['x']-slope*L['y_lo'];xc=x0+slope*yc
    out.update(ok=True,reason='doors',x_body_m=float(xc),x0_m=float(x0),lateral_body_m=float(yc),width_m=float(width),
               pair_heading_deg=float(math.degrees(math.atan(slope))),left_body=[L['x'],L['y_lo']],right_body=[R['x'],R['y_hi']],
               n_left=L['n'],n_right=R['n'])
    return out


def lattice_pair_in_window(body,expected_x_m,window_m,width_m,width_tol_m,expected_y_m=0.,bin_m=.05,min_points=4,y_max=1.2,x_min=.3):
    """The two lattice members bounding openings 2 and 3: in the slab around the lattice, list the vertical members and take the
    pair whose FACING inner faces are width_m +- width_tol_m apart, nearest to expected_y_m (the dock centreline). Returns
    dict(ok, reason, x_body_m, lateral_body_m (centre of the pair, left +), width_m, left_body, right_body, members, n)."""
    out=dict(ok=False,reason='no_points',n=0,members=0)
    if body is None or not len(body):return out
    sel=body[(np.abs(body[:,0]-float(expected_x_m))<=window_m)&(np.abs(body[:,1])<=y_max)&(body[:,0]>=x_min)]
    out['n']=int(len(sel));out['expected_x_m']=float(expected_x_m)
    if len(sel)<2*min_points:out['reason']='too_few_points_in_window';return out
    ms=members(sel,bin_m,min_points);out['members']=len(ms)
    if len(ms)<2:out['reason']='fewer_than_two_members';return out
    cands=[]
    for i,L in enumerate(ms):
        for R in ms[:i]+ms[i+1:]:
            if L['yc']<=R['yc']:continue
            width=L['y_lo']-R['y_hi']            # facing faces: the left member's right face to the right member's left face
            if abs(width-width_m)>width_tol_m:continue
            yc=(L['y_lo']+R['y_hi'])/2;cands.append((abs(yc-expected_y_m),L,R,width,yc))
    if not cands:out['reason']='no_pair_of_width_%.2fm'%width_m;return out
    _,L,R,width,yc=min(cands,key=lambda c:c[0])
    out.update(ok=True,reason='lattice_pair',x_body_m=float((L['x']+R['x'])/2),lateral_body_m=float(yc),width_m=float(width),
               left_body=[L['x'],L['y_lo']],right_body=[R['x'],R['y_hi']],n_left=L['n'],n_right=R['n'])
    return out


def line_in_window(body,expected_x_m,window_m,bin_m=.05,min_len_m=1.,min_bins=4,max_residual_m=.05,x_min=.3,y_max=.7):
    """A frontal straight member (the lattice's bottom bar) in the slab around expected_x_m: bin the points across y, keep the
    near face per bin (20th percentile of x), fit x = x0 + s*y with one robust pass. Returns dict(ok, reason, x_body_m (range
    straight ahead), heading_err_deg (body heading relative to the line normal = truck axis, left +), slope, span_m, bins, n,
    residual_m)."""
    out=dict(ok=False,reason='no_points',n=0)
    if body is None or not len(body):return out
    pts=body[(np.abs(body[:,0]-float(expected_x_m))<=window_m)&(np.abs(body[:,1])<=y_max)&(body[:,0]>=x_min)]
    out['n']=int(len(pts));out['expected_x_m']=float(expected_x_m)
    if len(pts)<min_bins:out['reason']='too_few_points_in_window';return out
    bins={}
    for x,y in zip(pts[:,0],pts[:,1]):bins.setdefault(int(math.floor(y/bin_m)),[]).append(x)
    bins={k:float(np.percentile(v,20)) for k,v in bins.items() if len(v)>=2}
    if len(bins)<min_bins:out['reason']='too_few_bins';return out
    ys=np.array([(k+.5)*bin_m for k in bins]);xs=np.array([bins[k] for k in bins])
    if ys.max()-ys.min()<min_len_m:out['reason']='span_%.2fm_below_%.2fm'%(ys.max()-ys.min(),min_len_m);return out
    A=np.column_stack([np.ones_like(ys),ys]);coef,_,_,_=np.linalg.lstsq(A,xs,rcond=None)
    res=xs-A@coef;good=np.abs(res)<=max(.02,2*np.std(res)) if len(xs)>4 else np.ones(len(xs),bool)
    if good.sum()>=min_bins:coef,_,_,_=np.linalg.lstsq(A[good],xs[good],rcond=None);res=xs-A@coef
    else:good=np.ones(len(xs),bool)
    rms=float(np.sqrt(np.mean(res[good]**2)))
    if rms>max_residual_m:out['reason']='not_a_line_rms_%.3fm'%rms;return out
    yg=ys[good];span=float(yg.max()-yg.min())
    if span<min_len_m:out['reason']='span_%.2fm_below_%.2fm'%(span,min_len_m);return out
    x0,s=float(coef[0]),float(coef[1])
    out.update(ok=True,reason='line',x_body_m=x0,heading_err_deg=float(math.degrees(math.atan(s))),slope=s,span_m=span,bins=int(good.sum()),residual_m=rms)
    return out


# ---------------------------------------------------------------------------------------------------------------------
DEFAULT_PARAMS=dict(step=2,max_m=4.,lattice_max_m=6.5,z_min=.45,z_max=.72,lattice_z_min=.33,lattice_z_max=.43,x_min=.2,cell_m=.05,forward_m=8.,half_width_m=3.,
                    entrance_window_m=.2,lattice_window_m=.35,bin_m=.05,min_points=4,line_min_len_m=1.,line_max_residual_m=.05)

def measure_bev(depth_m,intr,T_body_ir,entrance,lattice=None,params=None,stamp=None,vehicle_half_width_m=.64):
    """One depth frame -> the three references. entrance = dict(expected_x_m, width_m, width_tol_m, heading_rad);
    lattice = dict(expected_x_m, expected_y_m, pair_width_m, pair_tol_m) or None (lattice not configured). Output: ok/reason/
    x_body_m/lateral_body_m/width_m/left_body/right_body/u_* describe the PRIMARY reference for display (doors if found, else
    the lattice pair), plus 'doors', 'lattice_pair', 'lattice_line' in full, 'points', 'obstacle_cells' and the occupancy
    'grid' (numpy, for the dashboard; the worker strips it)."""
    from .bev import occupancy
    p=dict(DEFAULT_PARAMS);p.update({k:v for k,v in (params or {}).items() if k in p and v is not None})
    far=max(p['max_m'],p['lattice_max_m']) if lattice else p['max_m']
    zlo=min(p['z_min'],p['lattice_z_min']) if lattice else p['z_min'];zhi=max(p['z_max'],p['lattice_z_max']) if lattice else p['z_max']
    body,_,_=bev_points(depth_m,intr,T_body_ir,int(p['step']),far,zlo,zhi,p['x_min'])
    band=body[(body[:,2]>=p['z_min'])&(body[:,2]<=p['z_max'])]
    near=band[band[:,0]<=p['max_m']]
    grid=occupancy(near,p['forward_m'],p['half_width_m'],p['cell_m'],z_min=-math.inf,z_max=math.inf)
    doors=pair_in_window(near,entrance['expected_x_m'],p['entrance_window_m'],entrance['width_m'],entrance['width_tol_m'],
                         heading_rad=float(entrance.get('heading_rad') or 0.),bin_m=p['bin_m'],min_points=int(p['min_points']))
    out=dict(ok=False,reason=doors['reason'],method='window',stamp=stamp,points=int(len(band)),doors=doors,lattice_pair=None,lattice_line=None,
             obstacle_cells=corridor_cells(grid,vehicle_half_width_m,p['cell_m'],p['forward_m'],p['half_width_m']),grid=grid)
    if lattice:
        lat=band[band[:,0]<=p['lattice_max_m']]
        out['lattice_pair']=lattice_pair_in_window(lat,lattice['expected_x_m'],p['lattice_window_m'],lattice['pair_width_m'],lattice['pair_tol_m'],
                                                   expected_y_m=float(lattice.get('expected_y_m') or 0.),bin_m=p['bin_m'],min_points=int(p['min_points']))
        bar=body[(body[:,2]>=p['lattice_z_min'])&(body[:,2]<=p['lattice_z_max'])&(body[:,0]<=p['lattice_max_m'])]
        out['lattice_line']=line_in_window(bar,lattice['expected_x_m'],p['lattice_window_m'],bin_m=p['bin_m'],min_len_m=p['line_min_len_m'],max_residual_m=p['line_max_residual_m'])
    primary=doors if doors.get('ok') else (out['lattice_pair'] if out['lattice_pair'] and out['lattice_pair'].get('ok') else None)
    if primary is not None:
        T=np.asarray(T_body_ir,float);hz=float(T[2,3])
        out.update(ok=True,reason=primary['reason'],x_body_m=primary['x_body_m'],lateral_body_m=primary['lateral_body_m'],width_m=primary['width_m'],
                   left_body=primary['left_body'],right_body=primary['right_body'],pair_heading_deg=primary.get('pair_heading_deg'),
                   u_left=body_to_column((primary['left_body'][0],primary['left_body'][1],hz),intr,T_body_ir),
                   u_right=body_to_column((primary['right_body'][0],primary['right_body'][1],hz),intr,T_body_ir),
                   u_goal=body_to_column((primary['x_body_m'],primary['lateral_body_m'],hz),intr,T_body_ir))
    return out
