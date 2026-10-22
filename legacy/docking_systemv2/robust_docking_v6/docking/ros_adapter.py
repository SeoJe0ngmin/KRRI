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
from .vio_quality import QUALITY_FIELDS,QUALITY_VERSION,REBUILD_MESSAGE,check_message_schema


def set_stamp(stamp,t):
    ns=round(float(t)*1_000_000_000);stamp.sec=ns//1_000_000_000;stamp.nanosec=ns%1_000_000_000


def check_build_ready():
    ws=Path(os.environ.get('DOCK_V6_WS',str(Path.home()/'.local/share/krri-v6/ros_ws')))
    if (ws/'.docking_rebuild_pending').exists():
        raise RuntimeError('OpenVINS 재빌드가 완료되지 않았습니다. 이전 실행 파일은 사용하지 않습니다. '
                           '시계·빌드 오류 해결 후 bash scripts/rebuild_openvins.sh')


class Bridge:
    def __init__(self,config,output,epoch,max_age=.3,cancel=None):
        check_build_ready()
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile,ReliabilityPolicy
            from docking_v6_msgs.msg import Motion,Stereo,VioState
            from sensor_msgs.msg import Image
        except ImportError as e:
            raise RuntimeError('OpenVINS/ROS 2 설치 또는 환경 로딩 필요: scripts/install_jetson.sh, scripts/run_nano.sh. 카메라 확인은 measure --camera-only 사용.') from e
        check_message_schema(VioState)
        self.receive_error=None;self.closing=False
        self.rclpy=rclpy;self.Motion=Motion;self.Stereo=Stereo;self.Image=Image
        self.epoch=epoch;self.sequence=0;self.last_imu_sequence=0;self.lock=threading.Lock();self.states=deque(maxlen=32);self.dropped=0
        self.context=self.node=self.executor=self.thread=self.log=self.process=None
        self.log_path=Path(output)/'openvins.log'
        try:
            self.context=rclpy.Context();rclpy.init(context=self.context)
            self.namespace='/docking_'+uuid.uuid4().hex[:10]
            self.node=rclpy.create_node('sensor_adapter',namespace=self.namespace,context=self.context)
            qos=lambda n:QoSProfile(depth=n,reliability=ReliabilityPolicy.RELIABLE)
            self.imu_pub=self.node.create_publisher(Motion,'imu',qos(2048))
            self.image_pub=self.node.create_publisher(Stereo,'stereo',qos(2))
            self.sub=self.node.create_subscription(VioState,'vio_state',self.receive,qos(32))
            self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
            self.thread=threading.Thread(target=self.spin,daemon=True);self.thread.start()
            self.log=open(self.log_path,'w')
            self.process=subprocess.Popen(['ros2','run','docking_openvins','bridge','--ros-args','-r','__ns:='+self.namespace,
                '-p','parent_pid:='+str(os.getpid()),'-p','config_path:='+str(config),'-p','max_input_age_s:='+str(max_age)],stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+20
            # Wait for both input subscriptions AND the return state publisher.
            # Discovering one direction does not establish the other direction.
            while not self.peers_ready():
                if cancel is not None and cancel.is_set():raise RuntimeError('OpenVINS startup cancelled')
                self.check()
                if time.monotonic()>deadline:raise RuntimeError('OpenVINS DDS discovery timeout; '+str(self.status())+'\n'+self.log_tail())
                time.sleep(.05)
        except Exception:
            self.close();raise
    def peers_ready(self):
        return (self.imu_pub.get_subscription_count()>0 and self.image_pub.get_subscription_count()>0 and
                self.sub.get_publisher_count()>0)
    def spin(self):
        try:self.executor.spin()
        except Exception as exc:
            if not self.closing:self.receive_error='ROS executor: '+type(exc).__name__+': '+str(exc)
    def log_tail(self):
        try:
            with open(self.log_path,'rb') as f:
                f.seek(0,2);size=f.tell();f.seek(max(0,size-6000));return f.read().decode(errors='replace')
        except OSError:return '(no openvins.log)'
    def receive(self,m):
        try:self._receive(m)
        except Exception as exc:self.receive_error='OpenVINS state decoding: '+type(exc).__name__+': '+str(exc)
    def _receive(self,m):
        if m.quality_version!=QUALITY_VERSION and not m.fault:
            self.receive_error=REBUILD_MESSAGE;return
        p=dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,epoch=m.epoch,initialized=m.initialized,
            visual_valid=m.visual_valid,zero_velocity_update=m.zero_velocity_update,features=m.features,
            T_odom_imu=np.array(m.t_odom_imu).reshape(4,4).tolist(),covariance=np.array(m.covariance).reshape(6,6).tolist(),
            gyro_bias=list(m.gyro_bias),accel_bias=list(m.accel_bias),gyro_norm=m.gyro_norm,last_imu_stamp=m.last_imu_stamp,
            delta_valid=m.delta_valid,delta_from_stamp=m.delta_from_stamp,
            relative_transform=np.array(m.relative_transform).reshape(4,4).tolist(),relative_covariance=np.array(m.relative_covariance).reshape(6,6).tolist(),
            stereo_dropped=m.stereo_dropped,points=[[p.x,p.y] for p in m.left_points],fault=m.fault,arrival=time.monotonic())
        p.update({key:getattr(m,key) for key in QUALITY_FIELDS})
        with self.lock:
            if len(self.states)==self.states.maxlen:self.dropped+=1
            self.states.append(p)
    def status(self):
        return dict(stereo_published=self.sequence,imu_published=self.last_imu_sequence,
            imu_subscribers=self.imu_pub.get_subscription_count(),
            stereo_subscribers=self.image_pub.get_subscription_count(),
            state_publishers=self.sub.get_publisher_count(),
            process_exitcode=None if self.process is None else self.process.poll(),
            executor_alive=self.thread.is_alive())
    def check(self):
        if self.receive_error:raise RuntimeError(self.receive_error)
        if self.process and self.process.poll() is not None:raise RuntimeError('OpenVINS process exited; see openvins.log\n'+self.log_tail())
        if not self.thread.is_alive():raise RuntimeError('ROS executor stopped')
    def drain(self):
        self.check()
        with self.lock:
            items=list(self.states);self.states.clear()
        return items
    def imu(self,s):
        m=self.Motion();set_stamp(m.header.stamp,s['stamp']);m.header.frame_id='imu';m.epoch=self.epoch
        m.sequence=s['sequence'];m.gyro=s['gyro'];m.accel=s['accel'];self.imu_pub.publish(m)
        self.last_imu_sequence=s['sequence']
    def image(self,array,stamp,frame):
        a=np.ascontiguousarray(array,dtype=np.uint8);m=self.Image();set_stamp(m.header.stamp,stamp)
        m.header.frame_id=frame;m.height,m.width=a.shape;m.encoding='mono8';m.is_bigendian=False;m.step=m.width;m.data=a.tobytes();return m
    def stereo(self,left,right,stamp):
        self.sequence+=1;m=self.Stereo();set_stamp(m.header.stamp,stamp);m.epoch=self.epoch;m.sequence=self.sequence
        m.left=self.image(left,stamp,'left_rect');m.right=self.image(right,stamp,'right_rect');self.image_pub.publish(m)
    def close(self):
        if self.closing:return
        self.closing=True;errors=[]
        def cleanup(name,fn):
            try:fn()
            except Exception as exc:errors.append(name+': '+str(exc))
        def terminate():
            # The ros2 launcher can exit before its child. Signal its process
            # group even then; do not leave a native estimator on the next run.
            for sig,timeout in ((signal.SIGTERM,3),(signal.SIGKILL,1)):
                try:os.killpg(self.process.pid,sig)
                except ProcessLookupError:break
                try:self.process.wait(timeout)
                except subprocess.TimeoutExpired:continue
                if sig==signal.SIGTERM:
                    try:os.killpg(self.process.pid,signal.SIGKILL)
                    except ProcessLookupError:pass
                break
        if self.process:cleanup('native process',terminate)
        if self.executor:cleanup('executor',lambda:self.executor.shutdown(timeout_sec=2))
        if self.thread:cleanup('executor thread',lambda:self.thread.join(2))
        if self.node:cleanup('node',self.node.destroy_node)
        if self.context:cleanup('context',lambda:self.rclpy.shutdown(context=self.context))
        if self.log:
            if errors:cleanup('cleanup log',lambda:self.log.write('\n[cleanup] '+'; '.join(errors)+'\n'))
            cleanup('log close',self.log.close)
        return errors
