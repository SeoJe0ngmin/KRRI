"""Optional RTAB-Map stereo mapping with EXTERNAL OpenVINS odometry.

Map loop closures are logged as candidates, not fused as independent evidence
or used to silently replace a lost VIO origin. Each run creates a fresh map.
"""
from collections import OrderedDict
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import numpy as np
from .ros_adapter import set_stamp
from .uncertainty import inverse


def quaternion(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat().tolist()


class Mapping:
    def __init__(self,bridge,bundle,cfg,output):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import CameraInfo
        from geometry_msgs.msg import TransformStamped
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        from rtabmap_msgs.msg import Info
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        self.bridge=bridge;self.bundle=bundle;self.cfg=cfg['mapping'];self.Odometry=Odometry;self.CameraInfo=CameraInfo
        self.images=OrderedDict();self.last=-1.;self.lock=threading.Lock();self.report=dict(state='starting',used_for_control=False)
        n=bridge.node;q=QoSProfile(depth=2,reliability=ReliabilityPolicy.RELIABLE)
        self.odom=n.create_publisher(Odometry,'map_input/odom',q)
        self.pubs={name:n.create_publisher(bridge.Image,'map_input/'+name+'/image_rect',q) for name in ('left','right')}
        self.infos={name:n.create_publisher(CameraInfo,'map_input/'+name+'/camera_info',q) for name in ('left','right')}
        self.sub=n.create_subscription(Info,'map/info',self.info,q)
        self.tf=StaticTransformBroadcaster(n);transforms=[]
        for name,cal in zip(('left','right'),bundle['rectified']):
            T=inverse(cal['T_camera_imu']);m=TransformStamped();m.header.frame_id='imu';m.child_frame_id=name+'_rect'
            m.transform.translation.x,m.transform.translation.y,m.transform.translation.z=map(float,T[:3,3])
            m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w=quaternion(T[:3,:3]);transforms.append(m)
        self.tf.sendTransform(transforms)
        self.log=open(Path(output)/'rtabmap.log','w')
        params={'subscribe_stereo':True,'subscribe_rgb':False,'subscribe_depth':False,'approx_sync':False,
            # A nonempty odom_frame_id makes RTAB-Map ignore the odom topic
            # and request a dynamic TF instead. We supply exact-stamp messages.
            'subscribe_odom':True,'frame_id':'imu','odom_frame_id':'','map_frame_id':'map','publish_tf':False,
            'database_path':str(Path(output)/'map.db'),'Rtabmap/DetectionRate':str(self.cfg['frequency_hz']),
            'Rtabmap/MemoryThr':str(self.cfg['memory_threshold']),'Rtabmap/TimeThr':str(self.cfg['time_threshold_ms']),
            'RGBD/CreateOccupancyGrid':'false','Mem/IncrementalMemory':'true'}
        # YAML keeps RTAB parameter strings distinct from ROS bools/numbers.
        import yaml
        file=Path(output)/'rtabmap_params.yaml';file.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        cmd=['ros2','run','rtabmap_slam','rtabmap','--ros-args','-r','__ns:='+bridge.namespace,'-r','__node:=map',
             '--params-file',str(file),'-r','odom:=map_input/odom','-r','info:=map/info']
        for name in ('left','right'):
            for topic in ('image_rect','camera_info'):cmd+=['-r',name+'/'+topic+':=map_input/'+name+'/'+topic]
        self.process=subprocess.Popen(cmd,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
    def image(self,stamp,left,right):
        self.images[round(stamp,6)]=(left.copy(),right.copy())
        while len(self.images)>60:self.images.popitem(last=False)
    def state(self,p):
        if self.process.poll() is not None:raise RuntimeError('RTAB-Map exited; see rtabmap.log')
        if not p['initialized'] or not p['visual_valid'] or p['stamp']-self.last<1/self.cfg['frequency_hz']:return
        imgs=self.images.get(round(p['stamp'],6))
        if imgs is None:return
        self.last=p['stamp'];T=np.array(p['T_odom_imu']);m=self.Odometry();set_stamp(m.header.stamp,p['stamp'])
        m.header.frame_id='odom';m.child_frame_id='imu';m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,T[:3,3])
        m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=quaternion(T[:3,:3])
        m.pose.covariance=np.array(p['covariance']).ravel().tolist();self.odom.publish(m)
        for name,img,cal in zip(('left','right'),imgs,self.bundle['rectified']):
            info=self.CameraInfo();info.header.stamp=m.header.stamp;info.header.frame_id=name+'_rect'
            info.width=cal['width'];info.height=cal['height'];info.distortion_model='plumb_bob';info.d=[0.]*5
            info.k=np.array(cal['K']).ravel().tolist();info.r=np.eye(3).ravel().tolist();info.p=np.array(cal['P']).ravel().tolist()
            self.infos[name].publish(info);self.pubs[name].publish(self.bridge.image(img,p['stamp'],name+'_rect'))
    def info(self,m):
        stats=dict(zip(m.stats_keys,m.stats_values));inliers=stats.get('Loop/Visual_inliers/',stats.get('Loop/Visual_inliers',0))
        candidate=m.loop_closure_id>0 or m.proximity_detection_id>0
        with self.lock:self.report=dict(state='loop_candidate' if candidate else 'mapping',map_epoch=self.bridge.epoch,
            node_id=m.ref_id,loop_id=m.loop_closure_id,inliers=inliers,geometric_candidate=bool(candidate and inliers>=self.cfg['minimum_loop_inliers']),
            used_for_control=False,reason='requires_independent_dock_landmark_validation',nodes=len(m.wm_state))
    def status(self):
        with self.lock:return dict(self.report)
    def close(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGTERM)
            try:self.process.wait(5)
            except subprocess.TimeoutExpired:os.killpg(self.process.pid,signal.SIGKILL);self.process.wait()
        self.log.close()
