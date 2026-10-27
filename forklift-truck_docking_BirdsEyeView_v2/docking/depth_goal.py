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


# ---------------------------------------------------------------------------------------------------------------------
# 2026-10-22: bird's-eye entrance finder. The depth frame becomes 3-D points, the floor and anything above the truck
# opening are dropped by height, the rest is clustered on a 5 cm top-down grid and the entrance is the pair of clusters whose
# inner edges are truck_inner_width_m apart. Long clusters (truck side panels) also give the truck axis direction by a line
# fit, which two cones cannot. The 1-D band finder above remains the fallback (method 'band').

def bev_points(depth_m,intr,T_body_ir,step=2,max_m=6.,z_min=.05,z_max=1.6,x_min=.2):
    """Depth -> body-frame points between z_min and z_max above the floor and at least x_min ahead (3 x N), with their pixels."""
    from .bev import depth_to_points,camera_to_body
    pts,u,v=depth_to_points(depth_m,intr,step,max_m)
    T=np.asarray(T_body_ir,float);body=camera_to_body(pts,T[:3,:3],T[:3,3])
    keep=(body[:,2]>=z_min)&(body[:,2]<=z_max)&(body[:,0]>=x_min)
    return body[keep],u[keep],v[keep]


def _principal_angle(xy):
    """Direction of the long axis of a point set, radians in (-pi/2, pi/2] relative to body x."""
    c=xy-xy.mean(0);cov=c.T@c/max(1,len(c));w,v=np.linalg.eigh(cov);d=v[:,1]
    a=math.atan2(d[1],d[0])
    if a>math.pi/2:a-=math.pi
    if a<=-math.pi/2:a+=math.pi
    return a,float(w[1]/max(1e-12,w[0]))   # angle, elongation ratio


def find_entrance_bev(body,width_m,width_tol_m,expected_xy=None,cell_m=.05,forward_m=8.,half_width_m=3.,min_cells=3,min_points=4,
                      wall_min_len_m=.4,same_range_m=.2,max_range_m=None,heading=None):
    """Entrance from height-filtered body points. Returns a dict (ok, reason, ... , grid) — grid is the hit-count occupancy
    (for the dashboard), NOT JSON-serialisable: the caller strips it."""
    import cv2
    from .bev import occupancy
    grid=occupancy(body,forward_m,half_width_m,cell_m,z_min=-math.inf,z_max=math.inf)
    out=dict(ok=False,method='bev',grid=grid,blobs=0)
    if not len(body):out['reason']='no_points';return out
    # Cluster every occupied cell (a wall seen at a grazing angle leaves only 1-2 points per cell far away) and bridge one-cell
    # gaps with a 3x3 dilation; clusters are then filtered by their occupied-cell and point counts.
    occupied=(grid>=1).astype(np.uint8)
    n,labels,stats,_=cv2.connectedComponentsWithStats(cv2.dilate(occupied,np.ones((3,3),np.uint8)),connectivity=8)
    rows=grid.shape[0];r=rows-1-(body[:,0]/cell_m).astype(int);c=((half_width_m-body[:,1])/cell_m).astype(int)
    inside=(r>=0)&(r<rows)&(c>=0)&(c<grid.shape[1]);lab=np.zeros(len(body),int);lab[inside]=labels[r[inside],c[inside]]
    blobs=[]
    for k in range(1,n):
        rr,cc=np.nonzero((labels==k)&(occupied>0))
        if len(rr)<min_cells:continue
        sel=body[lab==k]
        if len(sel)<min_points:continue
        # Direction and length from the CELLS (one vote per 5 cm cell): a wall seen at a grazing angle has most of its POINTS
        # near its front end, which would make a point-based fit read as a short blob.
        cxy=np.column_stack([(rows-1-rr+.5)*cell_m,half_width_m-(cc+.5)*cell_m])
        ang,elong=_principal_angle(cxy) if len(cxy)>=3 else (0.,1.)
        d=np.array([math.cos(ang),math.sin(ang)]);proj=cxy@d;length=float(proj.max()-proj.min()+cell_m)
        x_front=float(np.percentile(sel[:,0],5))
        front=sel[sel[:,0]<=x_front+.35];front=front if len(front)>=min_points else sel   # edges at the FRONT of the cluster (a rotated wall's inner face moves with x)
        y_lo=float(np.percentile(front[:,1],5));y_hi=float(np.percentile(front[:,1],95))
        def edge_x(y):   # range of the inner edge itself (a cone's side silhouette sits at its centre x, a wall's inner face at its front)
            e=sel[np.abs(sel[:,1]-y)<=.04];return float(np.percentile(e[:,0],10)) if len(e) else x_front
        blobs.append(dict(n=int(len(sel)),x_front=x_front,x_lo=edge_x(y_lo),x_hi=edge_x(y_hi),y_lo=y_lo,y_hi=y_hi,
                          yc=float(sel[:,1].mean()),xc=float(sel[:,0].mean()),length=length,angle=ang,elong=elong,cells=int(len(cxy))))
    out['blobs']=len(blobs)
    if len(blobs)<2:out['reason']='fewer_than_two_clusters';return out
    cands=[]
    for L in blobs:
        for R in blobs:
            if L is R or L['yc']<=R['yc']:continue
            width=L['y_lo']-R['y_hi']                    # inner edge of the left cluster to inner edge of the right cluster
            if width<=0:continue
            if width_m is not None and abs(width-width_m)>width_tol_m:continue
            xl,xr=L['x_lo'],R['x_hi']
            if abs(xl-xr)>same_range_tolerance(min(xl,xr),width_m,heading,same_range_m):continue   # left and right edges are equally deep (user, 2026-10-22)
            cx=(xl+xr)/2;cy=(L['y_lo']+R['y_hi'])/2
            if max_range_m is not None and cx>max_range_m:continue
            key=math.hypot(cx-expected_xy[0],cy-expected_xy[1]) if expected_xy is not None else abs(cy)+.2*cx
            cands.append((key,L,R,width,cx,cy))
    if not cands:
        out['reason']='no_pair_of_width_%.2fm'%width_m if width_m is not None else 'no_pair';return out
    _,L,R,width,cx,cy=min(cands,key=lambda t:t[0])
    walls=bool(L['length']>=wall_min_len_m and R['length']>=wall_min_len_m and L['elong']>3 and R['elong']>3)
    if walls:
        axis=math.atan2(math.sin(L['angle'])+math.sin(R['angle']),math.cos(L['angle'])+math.cos(R['angle']))   # truck axis in the body frame
        pair_heading=-axis                                                                                       # body heading relative to the axis (left +)
    else:
        tau=np.array([L['x_lo']-R['x_hi'],L['y_lo']-R['y_hi']]);tau=tau/max(1e-9,np.linalg.norm(tau));nrm=np.array([tau[1],-tau[0]])
        if nrm[0]<0:nrm=-nrm
        pair_heading=math.atan2(-nrm[1],nrm[0]);axis=-pair_heading
    out.update(ok=True,reason='opening',x_body_m=float(cx),lateral_body_m=float(cy),width_m=float(width),
               left_body=[L['x_lo'],L['y_lo']],right_body=[R['x_hi'],R['y_hi']],z_left_m=L['x_lo'],z_right_m=R['x_hi'],
               walls=walls,axis_heading_deg=float(math.degrees(axis)),pair_heading_deg=float(math.degrees(pair_heading)),
               left_len_m=L['length'],right_len_m=R['length'],left_points=L['n'],right_points=R['n'])
    return out


def corridor_cells(grid,vehicle_half_width_m,cell_m=.05,forward_m=8.,half_width_m=3.,x_from=.3,x_to=2.5,margin_m=.05):
    """Occupied cells inside the vehicle's straight-ahead corridor (record only; an obstacle stop is a deferred item)."""
    rows,cols=grid.shape;r=np.arange(rows);cc=np.arange(cols)
    x=(rows-1-r+.5)*cell_m;y=half_width_m-(cc+.5)*cell_m
    sel=np.ix_((x>=x_from)&(x<=x_to),np.abs(y)<=vehicle_half_width_m+margin_m)
    return int((grid[sel]>=2).sum())


def body_to_column(pt_body,intr,T_body_ir):
    """Body point -> depth/IR image column (for drawing)."""
    T=np.asarray(T_body_ir,float);cam=T[:3,:3].T@(np.asarray(pt_body,float)-T[:3,3])
    if cam[2]<=.1:return None
    return float(intr['cx']+intr['fx']*cam[0]/cam[2])


def measure_rails(body,rail_width_m,band_in_m=.1,band_out_m=.2,x_min=.2,x_max=3.,bin_m=.05,min_bins=8,min_len_m=.4,inner_k=3):
    """Corridor rails (2026-10-27, BirdsEyeView_v2): the two guide rails seen as LINES while entering. From height-filtered
    body points, take the points near where each rail's inner face is expected (y = +-rail_width/2, band_in_m inside ..
    band_out_m outside), bin them along x and keep the innermost y per bin (the inner face), fit a line y = a + b*x per side.
    Corridor centre at the camera (x = 0) = (a_left + a_right)/2; with one rail only, that rail +- rail_width/2.
    Returns dict(ok, reason, sides, lateral_error_m (body y - centre y, left +), axis_heading_deg (rail direction in the body
    frame), heading_err_deg (body heading relative to the rails, left +), y_left_m, y_right_m, width_m, len_left_m, len_right_m,
    n_left, n_right). Pure geometry; no sensors."""
    half=rail_width_m/2;out=dict(ok=False,reason='no_points',sides=[],n_left=0,n_right=0)
    if body is None or not len(body):return out
    sel=body[(body[:,0]>=x_min)&(body[:,0]<=x_max)]
    if not len(sel):out['reason']='no_points_in_range';return out
    def side(sign):
        y=sign*sel[:,1];keep=(y>=half-band_in_m)&(y<=half+band_out_m)
        pts=sel[keep]
        if len(pts)<3:return None
        bins={}
        for x,yy in zip(pts[:,0],sign*pts[:,1]):bins.setdefault(int(x/bin_m),[]).append(yy)
        # innermost y per x bin = the inner face; the inner_k-th innermost point, so a few floor-noise points above z_min
        # (sparse, the rail top is dense) cannot pull the line inward; bins with fewer points are dropped
        bins={k:sorted(v)[inner_k-1] for k,v in bins.items() if len(v)>=inner_k}
        if len(bins)<min_bins:return None
        xs=np.array([(k+.5)*bin_m for k in bins]);ys=np.array([bins[k] for k in bins])
        if xs.max()-xs.min()<min_len_m:return None
        A=np.column_stack([np.ones_like(xs),xs]);coef,_,_,_=np.linalg.lstsq(A,ys,rcond=None)
        res=ys-A@coef;good=np.abs(res)<=max(.02,2*np.std(res)) if len(ys)>4 else np.ones(len(ys),bool)
        if good.sum()>=min_bins:coef,_,_,_=np.linalg.lstsq(A[good],ys[good],rcond=None)   # one robust pass
        a,b=float(coef[0])*sign,float(coef[1])*sign   # back to body y (left +)
        return dict(a=a,b=b,n=int(len(pts)),bins=int(len(bins)),length=float(xs.max()-xs.min()))
    L=side(+1.);R=side(-1.)
    if L is None and R is None:out['reason']='no_rail_lines';return out
    sides=[s for s,v in (('left',L),('right',R)) if v is not None]
    if L is not None and R is not None:
        yc=(L['a']+R['a'])/2;b=(L['b']+R['b'])/2;width=L['a']-R['a']
    elif L is not None:yc=L['a']-half;b=L['b'];width=None
    else:yc=R['a']+half;b=R['b'];width=None
    axis=math.atan(b)                                  # rail direction in the body frame (left +)
    out.update(ok=True,reason='rails',sides=sides,lateral_error_m=float(-yc),axis_heading_deg=float(math.degrees(axis)),
               heading_err_deg=float(-math.degrees(axis)),y_left_m=None if L is None else L['a'],y_right_m=None if R is None else R['a'],
               width_m=None if width is None else float(width),len_left_m=None if L is None else L['length'],len_right_m=None if R is None else R['length'],
               n_left=0 if L is None else L['n'],n_right=0 if R is None else R['n'],
               slope_left=None if L is None else L['b'],slope_right=None if R is None else R['b'])
    return out


def measure_bev(depth_m,intr,T_body_ir,width_m,width_tol_m,expected_xy=None,params=None,stamp=None,vehicle_half_width_m=.64,fallback=True,expected_u=None,heading=None,rail_width_m=None,rail_params=None):
    """Measurement: bird's-eye entrance first, the 1-D band finder as fallback. Same fields as measure() plus
    method, walls, axis_heading_deg, obstacle_cells, blobs; 'grid' (numpy) is for the dashboard and is stripped by the worker."""
    p=dict(step=2,max_m=6.,z_min=.05,z_max=1.6,x_min=.2,cell_m=.05,forward_m=8.,half_width_m=3.,min_cells=3,wall_min_len_m=.4,same_range_m=.2)
    p.update({k:v for k,v in (params or {}).items() if k in p})
    body,_,_=bev_points(depth_m,intr,T_body_ir,int(p['step']),p['max_m'],p['z_min'],p['z_max'],p['x_min'])
    rails=None
    if rail_width_m:
        try:rails=measure_rails(body,float(rail_width_m),**{k:v for k,v in (rail_params or {}).items() if k in ('band_in_m','band_out_m','x_min','x_max','bin_m','min_bins','min_len_m','inner_k')})
        except Exception as exc:rails=dict(ok=False,reason='error: '+str(exc),sides=[])
    g=find_entrance_bev(body,width_m,width_tol_m,expected_xy,p['cell_m'],p['forward_m'],p['half_width_m'],int(p['min_cells']),
                        wall_min_len_m=p['wall_min_len_m'],same_range_m=p['same_range_m'],max_range_m=p['max_m'],heading=heading)
    grid=g.pop('grid')
    out=dict(ok=bool(g.get('ok')),reason=g.get('reason'),method='bev',stamp=stamp,width_required_m=width_m,blobs=g.get('blobs',0),
             points=int(len(body)),obstacle_cells=corridor_cells(grid,vehicle_half_width_m,p['cell_m'],p['forward_m'],p['half_width_m']),grid=grid)
    if rails is not None:out['rails']=rails
    if g.get('ok'):
        T=np.asarray(T_body_ir,float);hz=float(T[2,3])
        out.update({k:g[k] for k in ('x_body_m','lateral_body_m','width_m','left_body','right_body','z_left_m','z_right_m','walls','axis_heading_deg','pair_heading_deg','left_len_m','right_len_m','left_points','right_points')})
        out.update(u_left=body_to_column((g['left_body'][0],g['left_body'][1],hz),intr,T_body_ir),u_right=body_to_column((g['right_body'][0],g['right_body'][1],hz),intr,T_body_ir),
                   u_goal=body_to_column((g['x_body_m'],g['lateral_body_m'],hz),intr,T_body_ir))
        return out
    if fallback:
        b=measure(depth_m,intr,T_body_ir,width_m,width_tol_m,expected_u=expected_u,params=params,stamp=stamp,candidate=True,heading=heading)
        b.update(method='band_fallback',bev_reason=out['reason'],blobs=out['blobs'],points=out['points'],obstacle_cells=out['obstacle_cells'],grid=grid)
        if rails is not None:b['rails']=rails
        return b
    return out
