"""SDK calibration -> rectified stereo OpenVINS rig. No guessed intrinsics."""
import copy
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from .tag import model_matrix
from .uncertainty import transform


def device_profiles(device, requested):
    """Match stream modes on ONE device instance, sharing its extrinsics graph.

    Separate pipeline.resolve() calls may return independent device instances.
    Never query inter-sensor extrinsics across those instances.
    """
    def key(p):
        mode=(p.stream_type(),p.stream_index(),p.format(),p.fps())
        if p.is_video_stream_profile():
            v=p.as_video_stream_profile()
            mode+=(v.width(),v.height())
        return mode
    available=[p for sensor in device.query_sensors() for p in sensor.get_stream_profiles()]
    selected=[]
    for request in requested:
        matches=[p for p in available if key(p)==key(request)]
        if len(matches)!=1:
            raise RuntimeError(f'Factory calibration profile match failed: {key(request)} ({len(matches)} matches)')
        selected.append(matches[0])
    return tuple(selected)


def extrinsic(source,target,label='source -> target'):
    try:
        e=source.get_extrinsics_to(target)
    except RuntimeError as exc:
        raise RuntimeError(f'RealSense factory extrinsics unavailable ({label}): {exc}') from exc
    T=np.eye(4);T[:3,:3]=np.array(e.rotation).reshape(3,3,order='F');T[:3,3]=e.translation
    return transform(T)


def model(profile,serial,override=None):
    i=profile.as_video_stream_profile().get_intrinsics()
    kind=str(i.model).split('.')[-1]
    if override is not None:
        return validate_override(dict(serial=serial,width=i.width,height=i.height),override)
    if kind not in ('none','brown_conrady','modified_brown_conrady'):
        if np.any(np.abs(i.coeffs)>1e-10):raise ValueError('unsupported lens model: '+kind)
    # Modified Brown projection differs from OpenCV when distortion is nonzero.
    if kind=='modified_brown_conrady' and np.any(np.abs(i.coeffs)>1e-10):
        raise ValueError('modified Brown requires a verified OpenCV lens calibration override')
    return dict(serial=serial,width=i.width,height=i.height,fx=i.fx,fy=i.fy,cx=i.ppx,cy=i.ppy,
                coeffs=list(i.coeffs),distortion=kind,source='device_factory')


def validate_override(base,override):
    for key in ('serial','width','height'):
        if override.get(key)!=base.get(key):raise ValueError('intrinsic override mismatch: '+key)
    for key in ('fx','fy','cx','cy'):
        if not np.isfinite(override[key]):raise ValueError('invalid intrinsic: '+key)
    if min(override['fx'],override['fy'])<=0:raise ValueError('nonpositive focal length')
    if len(override['coeffs']) not in (4,5,8) or not np.isfinite(override['coeffs']).all():raise ValueError('invalid distortion')
    if not override.get('validated'):raise ValueError('intrinsics must be explicitly validated before application')
    return copy.deepcopy(override)


def rectification(left,right,T_right_left):
    K0,K1=map(model_matrix,(left,right));size=(left['width'],left['height'])
    if size!=(right['width'],right['height']):raise ValueError('stereo resolutions differ')
    T=transform(T_right_left)
    if not .01<np.linalg.norm(T[:3,3])<.5:raise ValueError('invalid stereo baseline')
    R0,R1,P0,P1,Q,_,_=cv2.stereoRectify(K0,np.array(left['coeffs']),K1,np.array(right['coeffs']),size,T[:3,:3],T[:3,3],flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
    maps=[cv2.initUndistortRectifyMap(K,np.array(m['coeffs']),R,P[:3,:3],size,cv2.CV_32FC1)
          for K,m,R,P in ((K0,left,R0,P0),(K1,right,R1,P1))]
    return maps,(R0,R1),(P0,P1)


class OpenCvYaml(yaml.SafeDumper):
    # OpenCV FileStorage does not resolve YAML anchors/aliases. The IMU
    # identity matrices share a Python object, but must be written in full.
    def ignore_aliases(self,data):
        return True
OpenCvYaml.add_representer(list,lambda dumper,value:dumper.represent_sequence('tag:yaml.org,2002:seq',value,flow_style=True))

def yaml_file(path,data):
    # OpenCV FileStorage does not accept PyYAML's indentless block sequences.
    Path(path).write_text('%YAML:1.0\n'+yaml.dump(data,Dumper=OpenCvYaml,sort_keys=False,default_flow_style=False,width=10000))


def prepare(cfg,profiles,serial,output,overrides=None):
    left,right,color,gyro,accel=profiles
    data=json.loads(Path(overrides).read_text()) if overrides else {}
    models=[model(p,serial,data.get(name)) for p,name in zip((left,right,color),('left','right','color'))]
    T_rl=extrinsic(left,right,'IR left -> IR right');T_li=extrinsic(gyro,left,'gyro -> IR left');T_lc=extrinsic(color,left,'color -> IR left')
    Rga=extrinsic(accel,gyro,'accel -> gyro')[:3,:3]
    maps,Rs,Ps=rectification(models[0],models[1],T_rl)
    cal=cfg['calibration'];old=cal.get('camera_model')
    if cal.get('mount_valid') and old:
        # Shared mounting is defined on the raw left IR optical frame.
        for k in ('serial','width','height','fx','fy','cx','cy'):
            if isinstance(old.get(k),str):ok=old[k]==models[0][k]
            else:ok=old.get(k) is not None and abs(old[k]-models[0][k])<1e-3
            if not ok:raise ValueError('mount camera profile changed: '+k)
        if not np.allclose(old['coeffs'],models[0]['coeffs'],atol=1e-5):raise ValueError('mount lens calibration changed')
    T_bl=transform(cal['T_body_ir']);cal.update(T_body_imu=(T_bl@T_li).tolist(),
        T_body_camera=(T_bl@T_lc).tolist(),camera_model=models[2])
    chain={};rect_models=[];T_ri=T_rl@T_li
    for i,(R,P,Tci) in enumerate(zip(Rs,Ps,(T_li,T_ri))):
        Tr=np.eye(4);Tr[:3,:3]=R;Trect=Tr@Tci
        chain['cam'+str(i)]=dict(T_cam_imu=Trect.tolist(),cam_overlaps=[1-i],camera_model='pinhole',
            distortion_coeffs=[0.,0.,0.,0.],distortion_model='radtan',intrinsics=[float(P[0,0]),float(P[1,1]),float(P[0,2]),float(P[1,2])],
            resolution=[models[i]['width'],models[i]['height']],rostopic='stereo',timeshift_cam_imu=0.)
        rect_models.append(dict(K=P[:3,:3].tolist(),P=P.tolist(),T_camera_imu=Trect.tolist(),width=models[i]['width'],height=models[i]['height']))
    folder=Path(output);folder.mkdir(parents=True,exist_ok=True)
    template=Path(__file__).resolve().parents[1]/'configs/openvins_template.yaml'
    params=yaml.safe_load('\n'.join(template.read_text().splitlines()[1:]))
    o=cfg['openvins'];params.update(max_cameras=2,calib_cam_extrinsics=False,calib_cam_intrinsics=False,
        calib_cam_timeoffset=False,try_zupt=True,zupt_only_at_beginning=True,init_max_disparity=o['init_max_disparity'],
        init_window_time=o['init_window_time'],init_imu_thresh=o['init_imu_thresh'],max_clones=o['max_clones'],
        max_slam=o['max_slam'],num_pts=o['num_pts'],num_opencv_threads=1,track_frequency=cfg['camera']['fps']+1.,
        relative_config_imu='imu.yaml',relative_config_imucam='cameras.yaml')
    yaml_file(folder/'estimator.yaml',params);yaml_file(folder/'cameras.yaml',chain)
    eye=np.eye(3).tolist()
    imu=dict(T_i_b=np.eye(4).tolist(),accelerometer_noise_density=o['accel_noise_density'],accelerometer_random_walk=o['accel_random_walk'],
        gyroscope_noise_density=o['gyro_noise_density'],gyroscope_random_walk=o['gyro_random_walk'],
        rostopic='imu',time_offset=0.,update_rate=cfg['camera']['gyro_hz'],model='kalibr',Tw=eye,Ta=eye,
        R_IMUtoGYRO=eye,R_IMUtoACC=eye,Tg=np.zeros((3,3)).tolist())
    yaml_file(folder/'imu.yaml',dict(imu0=imu))
    bundle=dict(serial=serial,raw_models=dict(zip(('left','right','color'),models)),rectified=rect_models,
        T_left_color=T_lc.tolist(),T_left_imu=T_li.tolist(),calibration=copy.deepcopy(cal),
        timestamp_offset_source='SDK common hardware clock; online time calibration disabled',
        calibration_accuracy='factory calibration plus declared floors, not an accuracy certificate')
    (folder/'device.json').write_text(json.dumps(bundle,indent=2))
    return bundle,maps,Rga,folder/'estimator.yaml'
