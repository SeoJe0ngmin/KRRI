"""Bay-opening goal from the depth image (2026-10-21, user request). DISPLAY AND LOG ONLY: never used for control.

Seen from the front, the bay opening is a FAR region (the inside) between two NEAR vertical edges (the side walls'
front ends). In a horizontal band around the image centre:
  1. per column, the median of the valid depths in the band -> a 1-D depth profile z(u) (holes skipped);
  2. a running median smooths it;
  3. depth edges = jumps between neighbouring valid columns larger than max(min_jump_m, rel_jump x depth):
     'rise' = near -> far going right (a left wall edge), 'fall' = far -> near (a right wall edge);
  4. every rise followed by a fall is a candidate opening; the one whose middle is closest to the image centre wins;
  5. goal = the middle of that opening. Lateral positions use the depth on the NEAR side of each edge (the wall end)
     and the depth-stream intrinsics: X = (u - cx) / fx x z (camera optical frame, right +), reported as left +.
Passive stereo (IR projector off): good with texture, holes on plain surfaces, error grows ~quadratically beyond ~4 m.
"""
import math
import numpy as np

DEFAULTS=dict(band_frac=.2,max_m=6.,min_jump_m=.15,rel_jump=.08,smooth=5,min_valid_frac=.3,max_gap_px=8,min_width_px=8,same_range_m=.2)


def column_profile(depth_m,band_frac=.2,max_m=6.,min_valid_frac=.3):
    h,w=depth_m.shape;half=max(2,int(h*band_frac/2));c=h//2;r0,r1=c-half,c+half+1
    band=depth_m[r0:r1].astype(np.float64)
    valid=np.isfinite(band)&(band>0)&(band<=max_m*1.5)
    frac=valid.mean(axis=0)
    z=np.where(valid,band,np.inf);z.sort(axis=0)
    n=valid.sum(axis=0);prof=np.full(w,np.nan)
    ok=(n>0)&(frac>=min_valid_frac)
    idx=np.where(ok)[0]
    prof[idx]=z[(n[idx]-1)//2,idx]          # median of the valid values in each column (lower median)
    return prof,(r0,r1)


def smooth_profile(prof,k=5):
    """Running median over k columns, holes stay holes. Vectorised (the per-column Python loop cost ~25 ms per frame on the Jetson)."""
    if k<=1:return prof.copy()
    h=k//2;pad=np.full(h,np.nan);ext=np.concatenate([pad,prof,pad])
    stack=np.stack([ext[i:i+len(prof)] for i in range(k)])
    with np.errstate(all='ignore'):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter('ignore');out=np.nanmedian(stack,axis=0)
    out[~np.isfinite(prof)]=np.nan            # do not invent depth inside holes
    return out


def find_edges(prof,min_jump_m=.15,rel_jump=.08,max_gap_px=8,min_width_px=8):
    """Depth jumps between neighbouring valid columns. A run of columns WITHOUT depth (holes / beyond range) that is at least
    min_width_px wide counts as 'far' (2026-10-22): an opening onto open space or a distant background has no return."""
    valid=np.isfinite(prof);cols=np.where(valid)[0];edges=[]
    for i,j in zip(cols[:-1],cols[1:]):
        a,b=prof[i],prof[j]
        if j-i-1>=min_width_px:            # a wide hole between two valid columns: near->nothing and nothing->near
            edges.append(dict(u=i+.5,kind='rise',near_u=int(i),near_z=float(a),far_z=math.inf))
            edges.append(dict(u=j-.5,kind='fall',near_u=int(j),near_z=float(b),far_z=math.inf))
            continue
        if j-i>max_gap_px:continue
        thr=max(min_jump_m,rel_jump*min(a,b))
        if b-a>thr:edges.append(dict(u=(i+j)/2,kind='rise',near_u=int(i),near_z=float(a),far_z=float(b)))
        elif a-b>thr:edges.append(dict(u=(i+j)/2,kind='fall',near_u=int(j),near_z=float(b),far_z=float(a)))
    if len(cols):                          # image borders: a near blob touching the edge of the image still has an inner edge
        if cols[0]>=min_width_px:edges.insert(0,dict(u=cols[0]-.5,kind='fall',near_u=int(cols[0]),near_z=float(prof[cols[0]]),far_z=math.inf))
        if len(prof)-1-cols[-1]>=min_width_px:edges.append(dict(u=cols[-1]+.5,kind='rise',near_u=int(cols[-1]),near_z=float(prof[cols[-1]]),far_z=math.inf))
    return edges


def analyse(depth_m,intr,params=None):
    """Band profile + edges of one depth frame (the expensive part), shared by several selections."""
    p=dict(DEFAULTS);p.update(params or {})
    h,w=depth_m.shape
    prof,(r0,r1)=column_profile(depth_m,p['band_frac'],p['max_m'],p['min_valid_frac'])
    sm=smooth_profile(prof,int(p['smooth']))
    edges=find_edges(sm,p['min_jump_m'],p['rel_jump'],p['max_gap_px'],p['min_width_px'])
    return dict(params=p,width=w,band=(int(r0),int(r1)),valid_columns=float(np.isfinite(prof).mean()),profile=sm,edges=edges)


def same_range_tolerance(z_near,width_m,heading=None,same_range_m=.2):
    """Largest range difference the two entrance edges may show: measurement noise (same_range_m + 15 % of range) plus the
    geometric difference width x |sin(heading)| when the body is not square to the entrance; unknown heading -> 0.5 m."""
    noise=max(same_range_m,.15*z_near)
    if heading is None:return max(.5,.15*z_near)
    return noise+float(width_m or 1.4)*abs(math.sin(heading))


def select_opening(a,intr,width_m=None,width_tol_m=.2,expected_u=None,heading=None,same_range_m=.2):
    """Pick the opening from an analysis: with width_m only a left/right pair whose inner edges are width_m +- width_tol_m
    apart (metres, from each edge's own depth) and at about the same range, nearest to expected_u (default: image centre);
    without width_m the opening nearest the centre of ANY width (display only: it also accepts gaps between chair legs)."""
    p=a['params'];edges=a['edges'];sm=a['profile'];w=a['width']
    out=dict(ok=False,band_rows=list(a['band']),valid_columns=a['valid_columns'],edges=[dict(u=float(e['u']),kind=e['kind']) for e in edges],width_required_m=width_m)
    fx,cx=float(intr['fx']),float(intr['cx']);uc=float(cx if expected_u is None else expected_u)
    cands=[];loose=[]
    for ea in [e for e in edges if e['kind']=='rise']:
        for eb in [e for e in edges if e['kind']=='fall' and e['u']>ea['u']]:
            if eb['u']-ea['u']<p['min_width_px']:continue
            xl=(ea['near_u']-cx)/fx*ea['near_z'];xr=(eb['near_u']-cx)/fx*eb['near_z'];wid=abs(xr-xl)
            if width_m is not None and abs(wid-width_m)>width_tol_m:continue
            # Both edges at the same range (2026-10-22 user: the left and right posts/walls are equally deep). Without a width
            # (the orange display candidate) a near wall on one side must not be paired with a far object on the other.
            dz=abs(ea['near_z']-eb['near_z']);same=dz<=same_range_tolerance(min(ea['near_z'],eb['near_z']),width_m,heading,same_range_m)
            item=(abs((ea['u']+eb['u'])/2-uc),ea,eb,xl,xr,dz)
            if same:cands.append(item)
            elif width_m is None:loose.append(item)
    depth_match=bool(cands)
    if not cands and width_m is None and loose:cands=[min(loose,key=lambda c:c[5])]   # display only: no equally-deep pair -> the LEAST unequal one
    if not cands:
        out['reason']=('no_pair_of_width_%.2fm'%width_m if width_m is not None else 'no_opening') if edges else 'no_depth_edges'
        return out
    _,ea,eb,xl,xr,dz=min(cands,key=lambda c:c[0])
    inside=sm[int(math.ceil(ea['u'])):int(math.floor(eb['u']))+1];inside=inside[np.isfinite(inside)]
    ug=(ea['u']+eb['u'])/2
    out.update(ok=True,reason='opening',u_left=float(ea['u']),u_right=float(eb['u']),u_goal=float(ug),depth_match=depth_match,dz_m=float(dz),
               z_left_m=ea['near_z'],z_right_m=eb['near_z'],z_inside_m=float(np.median(inside)) if len(inside) else None,
               left_edge_lateral_m=float(-xl),right_edge_lateral_m=float(-xr),           # left +
               goal_lateral_m=float(-(xl+xr)/2),width_m=float(abs(xr-xl)),
               goal_bearing_deg=float(math.degrees(math.atan2(-(ug-cx),fx))))           # left +
    return out


def find_goal(depth_m,intr,params=None,width_m=None,width_tol_m=.2,expected_u=None,heading=None):
    """depth_m: HxW metres (0/NaN = no data); intr: dict(fx, cx) of the depth stream. See select_opening for the rule."""
    p=dict(DEFAULTS);p.update(params or {})
    return select_opening(analyse(depth_m,intr,params),intr,width_m,width_tol_m,expected_u,heading,p.get('same_range_m',.2))


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


def measure(depth_m,intr,T_body_ir,width_m,width_tol_m,expected_u=None,params=None,stamp=None,candidate=True,heading=None):
    """One depth frame -> the entrance in the BODY frame (x forward, y left of the body origin on the vehicle centreline).
    find_goal works in the depth camera frame (left IR lens); the lens sits T_body_ir[:3,3] from the body origin.
    Fields: ok, reason, lateral_body_m (centre y), x_body_m (centre x), width_m, z_left_m, z_right_m, u_goal, u_left,
    u_right, pair_heading_deg (body heading relative to the entrance normal implied by the two edges, left +), stamp,
    candidate (display only: the nearest opening of any width)."""
    a=analyse(depth_m,intr,params);srm=float((params or {}).get('same_range_m',.2))
    g=select_opening(a,intr,width_m=width_m,width_tol_m=width_tol_m,expected_u=expected_u,heading=heading,same_range_m=srm)
    out=dict(ok=bool(g.get('ok')),reason=g.get('reason'),stamp=stamp,width_required_m=width_m,valid_columns=g.get('valid_columns'))
    if candidate:
        c=select_opening(a,intr,expected_u=expected_u)
        out['candidate']=dict(ok=bool(c.get('ok')),reason=c.get('reason'),u_left=c.get('u_left'),u_right=c.get('u_right'),u_goal=c.get('u_goal'),
                              width_m=c.get('width_m'),z_m=None if not c.get('ok') else (c['z_left_m']+c['z_right_m'])/2,goal_lateral_m=c.get('goal_lateral_m'),depth_match=c.get('depth_match'),dz_m=c.get('dz_m'))
    if not g.get('ok'):return out
    T=np.asarray(T_body_ir,float);R=T[:3,:3];t=T[:3,3]
    def body_xy(u,z):
        cam=np.array([(u-intr['cx'])/intr['fx']*z,0.,z]);b=R@cam+t;return float(b[0]),float(b[1])
    l=body_xy(g['u_left'],g['z_left_m']);r=body_xy(g['u_right'],g['z_right_m'])
    cx,cy=(l[0]+r[0])/2,(l[1]+r[1])/2
    tau=np.array([l[0]-r[0],l[1]-r[1]]);tau=tau/max(1e-9,np.linalg.norm(tau));n=np.array([tau[1],-tau[0]])
    if n[0]<0:n=-n
    pair_heading=math.atan2(-n[1],n[0])   # body x axis relative to the entrance normal (left +)
    out.update(lateral_body_m=cy,x_body_m=cx,width_m=float(math.hypot(l[0]-r[0],l[1]-r[1])),z_left_m=g['z_left_m'],z_right_m=g['z_right_m'],
               u_goal=g['u_goal'],u_left=g['u_left'],u_right=g['u_right'],pair_heading_deg=math.degrees(pair_heading),
               left_body=[l[0],l[1]],right_body=[r[0],r[1]])
    return out
