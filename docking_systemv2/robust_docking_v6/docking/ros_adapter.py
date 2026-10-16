"""ROS is imported only for VIO runs; no simulated fallback on missing libraries."""
from collections import deque
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
import numpy as np


def set_stamp(stamp,t):
    ns=round(float(t)*1_000_000_000);stamp.sec=ns//1_000_000_000;stamp.nanosec=ns%1_000_000_000


class Bridge:
    def __init__(self,config,output,epoch,max_age=.3):
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile,ReliabilityPolicy
            from docking_v6_msgs.msg import Motion,Stereo,VioState
            from sensor_msgs.msg import Image
        except ImportError as e:
            raise RuntimeError('OpenVINS/ROS 2 설치 또는 환경 로딩 필요: scripts/install_jetson.sh, scripts/run_nano.sh. 카메라 확인은 measure --camera-only 사용.') from e
        self.rclpy=rclpy;self.Motion=Motion;self.Stereo=Stereo;self.Image=Image
        self.epoch=epoch;self.sequence=0;self.lock=threading.Lock();self.states=deque(maxlen=32);self.dropped=0
        self.context=rclpy.Context();rclpy.init(context=self.context)
        self.namespace='/docking_'+uuid.uuid4().hex[:10]
        self.node=rclpy.create_node('sensor_adapter',namespace=self.namespace,context=self.context)
        qos=lambda n:QoSProfile(depth=n,reliability=ReliabilityPolicy.RELIABLE)
        self.imu_pub=self.node.create_publisher(Motion,'imu',qos(2048))
        self.image_pub=self.node.create_publisher(Stereo,'stereo',qos(2))
        self.sub=self.node.create_subscription(VioState,'vio_state',self.receive,qos(32))
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
        self.log=open(Path(output)/'openvins.log','w')
        self.process=None
        try:
            self.process=subprocess.Popen(['ros2','run','docking_openvins','bridge','--ros-args','-r','__ns:='+self.namespace,
                '-p','parent_pid:='+str(os.getpid()),'-p','config_path:='+str(config),'-p','max_input_age_s:='+str(max_age)],stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+20
            while self.imu_pub.get_subscription_count()<1 or self.image_pub.get_subscription_count()<1:
                self.check()
                if time.monotonic()>deadline:raise RuntimeError('OpenVINS DDS discovery timeout; see openvins.log')
                time.sleep(.05)
        except Exception:
            self.close();raise
    def receive(self,m):
        p=dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,epoch=m.epoch,initialized=m.initialized,
            visual_valid=m.visual_valid,zero_velocity_update=m.zero_velocity_update,features=m.features,
            T_odom_imu=np.array(m.t_odom_imu).reshape(4,4).tolist(),covariance=np.array(m.covariance).reshape(6,6).tolist(),
            gyro_bias=list(m.gyro_bias),accel_bias=list(m.accel_bias),gyro_norm=m.gyro_norm,last_imu_stamp=m.last_imu_stamp,
            delta_valid=m.delta_valid,delta_from_stamp=m.delta_from_stamp,
            relative_transform=np.array(m.relative_transform).reshape(4,4).tolist(),relative_covariance=np.array(m.relative_covariance).reshape(6,6).tolist(),
            stereo_dropped=m.stereo_dropped,points=[[p.x,p.y] for p in m.left_points],fault=m.fault,arrival=time.monotonic())
        with self.lock:
            if len(self.states)==self.states.maxlen:self.dropped+=1
            self.states.append(p)
    def check(self):
        if self.process and self.process.poll() is not None:raise RuntimeError('OpenVINS process exited; see openvins.log')
        if not self.thread.is_alive():raise RuntimeError('ROS executor stopped')
    def drain(self):
        self.check()
        with self.lock:
            items=list(self.states);self.states.clear()
        return items
    def imu(self,s):
        m=self.Motion();set_stamp(m.header.stamp,s['stamp']);m.header.frame_id='imu';m.epoch=self.epoch
        m.sequence=s['sequence'];m.gyro=s['gyro'];m.accel=s['accel'];self.imu_pub.publish(m)
    def image(self,array,stamp,frame):
        a=np.ascontiguousarray(array,dtype=np.uint8);m=self.Image();set_stamp(m.header.stamp,stamp)
        m.header.frame_id=frame;m.height,m.width=a.shape;m.encoding='mono8';m.is_bigendian=False;m.step=m.width;m.data=a.tobytes();return m
    def stereo(self,left,right,stamp):
        self.sequence+=1;m=self.Stereo();set_stamp(m.header.stamp,stamp);m.epoch=self.epoch;m.sequence=self.sequence
        m.left=self.image(left,stamp,'left_rect');m.right=self.image(right,stamp,'right_rect');self.image_pub.publish(m)
    def close(self):
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGTERM)
            try:self.process.wait(3)
            except subprocess.TimeoutExpired:os.killpg(self.process.pid,signal.SIGKILL);self.process.wait()
        self.executor.shutdown(timeout_sec=2);self.thread.join(2);self.node.destroy_node()
        self.rclpy.shutdown(context=self.context);self.log.close()
