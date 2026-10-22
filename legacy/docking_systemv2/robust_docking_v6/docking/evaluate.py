"""Same-frame timestamp-paired pose evaluation; never aligns away docking bias."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from .geometry import wrap
from .uncertainty import covariance


def evaluate(estimates,truth,max_dt=.03,fork_length=1.5):
    # Ground truth CSV: device_stamp,x_m,y_m,yaw_rad, all in the dock frame.
    truth=sorted(truth,key=lambda r:float(r['device_stamp']));ts=np.array([float(r['device_stamp']) for r in truth])
    if len(truth)<2 or np.any(np.diff(ts)<=0):raise ValueError('truth requires increasing timestamps in the same clock and dock frame')
    errors=[];fork=[];nees=[];matched=[]
    for e in estimates:
        if e.get('pose') is None:continue
        t=float(e['capture_id']);i=int(np.argmin(abs(ts-t)))
        if abs(ts[i]-t)>max_dt:continue
        gt=np.array([float(truth[i][k]) for k in ('x_m','y_m','yaw_rad')]);p=np.asarray(e['pose']);d=p-gt;d[2]=wrap(d[2])
        errors.append(d);matched.append((t,p,gt))
        f=p[:2]+fork_length*np.array([np.cos(p[2]),np.sin(p[2])]);g=gt[:2]+fork_length*np.array([np.cos(gt[2]),np.sin(gt[2])]);fork.append(f-g)
        if e.get('covariance') is not None:
            P=covariance(e['covariance'],3,True);nees.append(float(d@np.linalg.solve(P,d)))
    if len(errors)<2:raise ValueError('fewer than two aligned samples; verify clock/frame and ground-truth file')
    d=np.array(errors);f=np.array(fork);rpe=[]
    from .uncertainty import relative
    for a,b in zip(matched,matched[1:]):
        q=relative(a[1],b[1])-relative(a[2],b[2]);q[2]=wrap(q[2]);rpe.append(q)
    rpe=np.array(rpe)
    return dict(samples=len(d),pose_bias=d.mean(axis=0).tolist(),pose_repeatability_std=d.std(axis=0).tolist(),
        ate_position_rmse_m=float(np.sqrt(np.mean(np.sum(d[:,:2]**2,axis=1)))),
        yaw_rmse_rad=float(np.sqrt(np.mean(d[:,2]**2))),fork_position_rmse_m=float(np.sqrt(np.mean(np.sum(f**2,axis=1)))),
        fork_lateral_abs_p95_m=float(np.percentile(abs(f[:,1]),95)),rpe_adjacent_rmse=np.sqrt(np.mean(rpe*rpe,axis=0)).tolist(),
        mean_nees=None if not nees else float(np.mean(nees)),nees_dof=3,
        note='No SE3/scale fitting. NEES assumes independent accurate truth; adjacent errors may be correlated. Accuracy and repeatability are different.')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('observations');p.add_argument('truth_csv');p.add_argument('--output',required=True)
    p.add_argument('--max-dt',type=float,default=.03);p.add_argument('--fork-length',type=float,default=1.5);a=p.parse_args()
    estimates=[json.loads(s) for s in Path(a.observations).read_text().splitlines()]
    with open(a.truth_csv) as f:truth=list(csv.DictReader(f))
    report=evaluate(estimates,truth,a.max_dt,a.fork_length);Path(a.output).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
