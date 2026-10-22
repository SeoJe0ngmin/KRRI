"""Offline lens and hand-eye tools. Reports never silently overwrite common."""
import argparse
import glob
import json
from pathlib import Path
import cv2
import numpy as np
from .uncertainty import transform,inverse


def hand_eye(samples):
    if len(samples)<12:raise ValueError('at least 12 independently measured poses required')
    B=[transform(s['T_base_body']) for s in samples];C=[transform(s['T_camera_tag']) for s in samples]
    rotations=np.array([cv2.Rodrigues((inverse(B[0])@b)[:3,:3])[0].ravel() for b in B[1:]])
    singular=np.linalg.svd(rotations,compute_uv=False)
    if singular[1]<.15:raise ValueError('hand_eye_unobservable: multiple nonparallel rotations required; planar yaw alone is insufficient')
    R,t=cv2.calibrateHandEye([b[:3,:3] for b in B],[b[:3,3] for b in B],[c[:3,:3] for c in C],[c[:3,3] for c in C],method=cv2.CALIB_HAND_EYE_PARK)
    X=np.eye(4);X[:3,:3]=R;X[:3,3]=t.ravel();transform(X)
    targets=[b@X@c for b,c in zip(B,C)];centre=np.median([t[:3,3] for t in targets],axis=0)
    residual=[np.linalg.norm(t[:3,3]-centre) for t in targets]
    angles=[np.linalg.norm(cv2.Rodrigues(t[:3,:3]@targets[0][:3,:3].T)[0]) for t in targets]
    return dict(T_body_camera=X.tolist(),translation_residual_p95_m=float(np.percentile(residual,95)),
        rotation_residual_p95_rad=float(np.percentile(angles,95)),validated=False,
        note='independent reference motion required; residual is not absolute accuracy; review before common mounting conversion')


def intrinsics(args):
    if args.columns<3 or args.rows<3 or not np.isfinite(args.square_m) or args.square_m<=0:
        raise ValueError('checkerboard requires at least 3x3 internal corners and a positive measured square size')
    obj=np.zeros((args.columns*args.rows,3),np.float32);obj[:,:2]=np.mgrid[0:args.columns,0:args.rows].T.reshape(-1,2)*args.square_m
    objects=[];corners=[];size=None;centres=[]
    for path in sorted(glob.glob(args.images)):
        image=cv2.imread(path,cv2.IMREAD_GRAYSCALE)
        if image is None:continue
        wh=image.shape[::-1]
        if size is not None and size!=wh:raise ValueError('mixed image resolutions')
        size=wh;ok,pts=cv2.findChessboardCornersSB(image,(args.columns,args.rows))
        if ok:objects.append(obj);corners.append(pts.astype(np.float32));centres.append(pts.reshape(-1,2).mean(axis=0)/size)
    if len(objects)<15:raise ValueError('at least 15 checkerboard views required')
    if np.ptp(centres,axis=0).min()<.3:raise ValueError('insufficient image coverage: include edges and tilted views')
    # Reserve independent images for validation instead of using training RMS.
    train=[i for i in range(len(objects)) if i%4];test=[i for i in range(len(objects)) if not i%4]
    rms,K,d,rv,tv,std,_,_=cv2.calibrateCameraExtended([objects[i] for i in train],[corners[i] for i in train],size,None,None)
    errs=[]
    for i in test:
        ok,r,t=cv2.solvePnP(objects[i],corners[i],K,d)
        if not ok:raise ValueError('heldout PnP failed')
        projected=cv2.projectPoints(objects[i],r,t,K,d)[0]
        errs.append(float(np.sqrt(np.mean((projected-corners[i])**2))))
    return {args.stream:dict(serial=args.serial,width=size[0],height=size[1],fx=float(K[0,0]),fy=float(K[1,1]),cx=float(K[0,2]),cy=float(K[1,2]),
        coeffs=d.ravel().tolist(),intrinsic_covariance=np.diag(std[:4+len(d.ravel()),0]**2).tolist(),distortion='brown_conrady',
        source='checkerboard_offline',validated=False,training_rms_px=float(rms),heldout_rms_px=errs,
        note='review heldout error and target dimensions; set validated true only after independent validation')}


def main():
    p=argparse.ArgumentParser();sub=p.add_subparsers(dest='command',required=True)
    h=sub.add_parser('hand-eye');h.add_argument('poses');h.add_argument('--output',required=True)
    k=sub.add_parser('intrinsics');k.add_argument('--images',required=True);k.add_argument('--columns',type=int,required=True)
    k.add_argument('--rows',type=int,required=True);k.add_argument('--square-m',type=float,required=True);k.add_argument('--serial',required=True)
    k.add_argument('--stream',choices=['left','right','color'],required=True);k.add_argument('--output',required=True)
    a=p.parse_args();report=hand_eye(json.loads(Path(a.poses).read_text())['samples']) if a.command=='hand-eye' else intrinsics(a)
    Path(a.output).write_text(json.dumps(report,ensure_ascii=False,indent=2));print(a.output)
if __name__=='__main__':main()
