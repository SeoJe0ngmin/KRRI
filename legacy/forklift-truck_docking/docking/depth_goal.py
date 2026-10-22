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

DEFAULTS=dict(band_frac=.2,max_m=6.,min_jump_m=.15,rel_jump=.08,smooth=5,min_valid_frac=.3,max_gap_px=8,min_width_px=8)


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
    if k<=1:return prof.copy()
    out=prof.copy();h=k//2
    for i in range(len(prof)):
        win=prof[max(0,i-h):i+h+1];win=win[np.isfinite(win)]
        if len(win):out[i]=np.median(win)
    out[~np.isfinite(prof)]=np.nan            # do not invent depth inside holes
    return out


def find_edges(prof,min_jump_m=.15,rel_jump=.08,max_gap_px=8):
    cols=np.where(np.isfinite(prof))[0];edges=[]
    for i,j in zip(cols[:-1],cols[1:]):
        if j-i>max_gap_px:continue
        a,b=prof[i],prof[j];thr=max(min_jump_m,rel_jump*min(a,b))
        if b-a>thr:edges.append(dict(u=(i+j)/2,kind='rise',near_u=int(i),near_z=float(a),far_z=float(b)))
        elif a-b>thr:edges.append(dict(u=(i+j)/2,kind='fall',near_u=int(j),near_z=float(b),far_z=float(a)))
    return edges


def find_goal(depth_m,intr,params=None):
    """depth_m: HxW metres (0/NaN = no data); intr: dict(fx, cx) of the depth stream. Returns a JSON-able dict."""
    p=dict(DEFAULTS);p.update(params or {})
    h,w=depth_m.shape
    prof,(r0,r1)=column_profile(depth_m,p['band_frac'],p['max_m'],p['min_valid_frac'])
    sm=smooth_profile(prof,int(p['smooth']))
    edges=find_edges(sm,p['min_jump_m'],p['rel_jump'],p['max_gap_px'])
    out=dict(ok=False,band_rows=[int(r0),int(r1)],valid_columns=float(np.isfinite(prof).mean()),
             edges=[dict(u=float(e['u']),kind=e['kind']) for e in edges])
    cands=[]
    for a in [e for e in edges if e['kind']=='rise']:
        falls=[e for e in edges if e['kind']=='fall' and e['u']>a['u']]
        if not falls:continue
        b=falls[0]
        if b['u']-a['u']<p['min_width_px']:continue
        cands.append((abs((a['u']+b['u'])/2-w/2),a,b))
    if not cands:
        out['reason']='no_opening' if edges else 'no_depth_edges'
        return out
    _,a,b=min(cands,key=lambda c:c[0])
    fx,cx=float(intr['fx']),float(intr['cx'])
    xl=(a['near_u']-cx)/fx*a['near_z'];xr=(b['near_u']-cx)/fx*b['near_z']   # camera optical X, right +
    inside=sm[int(math.ceil(a['u'])):int(math.floor(b['u']))+1];inside=inside[np.isfinite(inside)]
    ug=(a['u']+b['u'])/2
    out.update(ok=True,reason='opening',u_left=float(a['u']),u_right=float(b['u']),u_goal=float(ug),
               z_left_m=a['near_z'],z_right_m=b['near_z'],z_inside_m=float(np.median(inside)) if len(inside) else None,
               left_edge_lateral_m=float(-xl),right_edge_lateral_m=float(-xr),           # left +
               goal_lateral_m=float(-(xl+xr)/2),width_m=float(abs(xr-xl)),
               goal_bearing_deg=float(math.degrees(math.atan2(-(ug-cx),fx))))           # left +
    return out
