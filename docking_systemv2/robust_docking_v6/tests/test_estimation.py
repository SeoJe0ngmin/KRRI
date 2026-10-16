"""Synthetic invariants, including correlated uncertainty and physical coast."""
import copy
import math
import unittest
import numpy as np
import cv2
from docking.uncertainty import *
from docking.alignment import Alignment,AnchorEvent
from docking.history import PoseHistory,State
from docking.observer import Observer
from docking.stream import ImuSynchronizer
from docking.tag import pnp_covariance,dock_measurement,depth_check
from docking.sensor_calibration import rectification,validate_override
from docking.calibration_tools import hand_eye
from docking.controller import Controller
from support import config,observation


class Estimation(unittest.TestCase):
    def setUp(self):self.c=config();self.s=self.c['sensor'];self.u=self.s['uncertainty']
    def test_ci_correlated_repeats_do_not_collapse_covariance(self):
        a=np.zeros(3);P=np.diag([.01,.02,.03])
        for _ in range(100):a,P=intersection(a,P,np.zeros(3),np.diag([.01,.02,.03]))
        np.testing.assert_allclose(np.diag(P),[.01,.02,.03],rtol=1e-10)
    def test_delayed_events_replay_in_capture_order(self):
        events=[AnchorEvent(i*.3,str(i),1,np.array([.01*i,0.,0.]),np.eye(3)*.01) for i in range(6)]
        a,b=Alignment(self.u),Alignment(self.u)
        for e in events:a.add(e)
        for i in (0,1,3,4,2,5):b.add(events[i])
        np.testing.assert_allclose(a.mean,b.mean);np.testing.assert_allclose(a.P,b.P)
        self.assertEqual(b.add(events[-1])['reason'],'duplicate_tag_event')
        with self.assertRaisesRegex(ValueError,'epoch'):b.add(AnchorEvent(4.,'other',2,np.zeros(3),np.eye(3)))
    def test_outlier_does_not_relocate_or_reduce_uncertainty(self):
        a=Alignment(self.u)
        for i in range(4):a.add(AnchorEvent(i*.3,str(i),1,np.zeros(3),np.eye(3)*.001))
        prior=a.P.copy();r=a.add(AnchorEvent(2.,'bad',1,np.array([2.,0.,0.]),np.eye(3)*.001))
        self.assertFalse(r['accepted']);np.testing.assert_allclose(a.mean,0);np.testing.assert_allclose(a.P,prior)
    def test_capture_history_rejects_extrapolation_and_epoch(self):
        h=PoseHistory(max_gap=.2);P=np.eye(6)*.01
        h.add(State(0.,1,np.eye(4),P,1));T=np.eye(4);T[0,3]=.1;h.add(State(.1,1,T,P,1.1))
        self.assertAlmostEqual(h.at(.05,1).T[0,3],.05);np.testing.assert_allclose(h.at(.05,1).P,P)
        for t,e in ((-.1,1),(.2,1),(.05,2)):
            with self.assertRaises(ValueError):h.at(t,e)
        h.add(State(.2,2,T,P,1.2));self.assertEqual(len(h.items),1)
    def test_fork_yaw_uncertainty_is_amplified_by_lever_arm(self):
        _,P=fork_uncertainty([0,0,0],np.diag([0,0,math.radians(1)**2]),1.5)
        self.assertAlmostEqual(math.sqrt(P[1,1]),1.5*math.radians(1))
        _,v=target_lateral_uncertainty([-4,0,0],np.diag([0,0,math.radians(1)**2]),1)
        self.assertAlmostEqual(math.sqrt(v),5*math.radians(1))
    def packet(self,t,x,epoch=1):
        T=np.eye(4);T[0,3]=x
        previous=getattr(self,'last_packet',(t,x));D=np.eye(4);D[0,3]=x-previous[1];self.last_packet=(t,x)
        return dict(delta_valid=True,delta_from_stamp=previous[0],relative_transform=D,relative_covariance=np.eye(6)*1e-9,stamp=t,epoch=epoch,arrival=t+1,imu_arrival=t+1,features=50,initialized=True,visual_valid=True,
            T_odom_imu=T,covariance=np.eye(6)*1e-5,transport_lag_s=.01,gyro_norm=0.)
    def test_physical_1point2m_is_not_a_tag_correction_or_command(self):
        self.s['calibration']['T_body_imu']=np.eye(4).tolist();o=Observer(self.s)
        for i in range(13):a=o.add_vio(self.packet(i*.1,i*.1),neutral=False)
        self.assertAlmostEqual(a['local_pose'][0],1.2)
        for i in range(4):o.alignment.add(AnchorEvent(.3*i,str(i),1,np.array([-5.,0.,0.]),np.eye(3)*1e-5))
        before=a['local_pose'];a=o.add_vio(self.packet(1.3,1.2),neutral=False)
        np.testing.assert_allclose(a['local_pose'],before);self.assertAlmostEqual(a['pose'][0],-3.8)
    def test_global_vio_gauge_is_not_counted_twice_as_dock_error(self):
        self.s['calibration']['T_body_imu']=np.eye(4).tolist();o=Observer(self.s)
        for i in range(14):
            packet=self.packet(i*.1,0);packet['covariance']=np.eye(6)*100
            result=o.add_vio(packet)
            if i in (1,4,7,10):o.alignment.add(AnchorEvent(i*.1,str(i),1,np.array([-2.,0,0]),np.eye(3)*1e-6,o.noise.copy()))
        self.assertTrue(result['usable']);self.assertLess(max(result['confidence_radius']),.01)
        # Lost clone continuity invalidates the anchor instead of inventing noise.
        packet=self.packet(1.4,.1);packet['delta_valid']=False
        result=o.add_vio(packet);self.assertFalse(result['usable']);self.assertIsNone(result['pose'])

    def test_native_rotation_covariance_never_disappears(self):
        self.s['calibration']['T_body_imu']=np.eye(4).tolist();o=Observer(self.s)
        state=o.body_state(self.packet(1,0));np.testing.assert_allclose(state.P,np.eye(6)*1e-5)
        self.s['calibration']['T_body_imu'][0][3]=1.5
        state=o.body_state(self.packet(2,0));self.assertGreater(state.P[1,1],3e-5)
    def test_stationary_requires_visual_tracking_and_neutral(self):
        self.s['calibration']['T_body_imu']=np.eye(4).tolist();o=Observer(self.s)
        for i in range(220):a=o.add_vio(self.packet(i/30,0))
        self.assertTrue(a['stationary'])
        p=self.packet(220/30,0);p['visual_valid']=False;a=o.add_vio(p)
        self.assertFalse(a['stationary']);self.assertFalse(a['usable'])
    def test_pnp_covariance_increases_with_distance(self):
        K=np.array([[900.,0,640],[0,900,360],[0,0,1.]])
        P=[]
        for z in (2.,4.):
            T=np.eye(4);T[:3,:3]=cv2.Rodrigues(np.array([.1,.2,0.]))[0];T[2,3]=z
            p,_=pnp_covariance(T,.3,K,np.zeros(5));P.append(p)
        self.assertGreater(P[1][2,2],P[0][2,2]*2)
    def test_dock_geometry_matches_known_pose(self):
        cal=self.s['calibration'];Tdb=matrix([-3.,.1,.03]);Tct=inverse(cal['T_body_camera'])@inverse(Tdb)@np.array(cal['T_dock_tag'])
        p,P=dock_measurement(Tct,np.eye(6)*1e-7,cal['T_body_camera'],np.eye(6)*1e-7,cal['T_dock_tag'],.005,.002)
        np.testing.assert_allclose(p,[-3,.1,.03],atol=1e-8);self.assertGreater(P[1,1],.005**2)
    def test_imu_keeps_200hz_and_interpolates_100hz(self):
        q=ImuSynchronizer()
        for i in range(101):q.add('accel',i*.01,[i*.01,9.81,0],i*.01+1)
        for i in range(201):q.add('gyro',i*.005,[0,0,.01],i*.005+1)
        values=q.drain();self.assertEqual(len(values),201)
        for s in values:self.assertAlmostEqual(s['accel'][0],s['stamp'])
        q.add('gyro',2,[0,0,0]);self.assertEqual(q.fault,'imu_clock_or_sample_gap')
    def test_imu_queue_overflow_is_not_silent_drop(self):
        q=ImuSynchronizer(capacity=2)
        for i in range(3):q.add('gyro',i*.005,[0,0,0])
        with self.assertRaisesRegex(RuntimeError,'overflow'):q.drain()
    def test_stereo_geometry_and_intrinsic_profile_binding(self):
        m=dict(serial='A',width=640,height=480,fx=380.,fy=380.,cx=320.,cy=240.,coeffs=[0.]*5)
        T=np.eye(4);T[0,3]=-.05
        maps,R,P=rectification(m,m,T);self.assertAlmostEqual(P[1][0,3]/P[1][0,0],-.05)
        self.assertEqual(maps[0][0].shape,(480,640))
        with self.assertRaises(ValueError):validate_override(m,dict(m,serial='B',validated=True))
    def test_depth_is_gate_not_independent_fused_pose(self):
        image=np.full((100,100),4.);corners=[[20,20],[80,20],[80,80],[20,80]]
        r=depth_check(image,corners,2.,self.u);self.assertFalse(r['consistent'])
        image[:]=0;self.assertFalse(depth_check(image,corners,2.,self.u)['available'])
    def test_large_uncertainty_cannot_enter_blind_or_finish(self):
        c=Controller(self.c,-2.5,0);c.state='PLAN';c.mission_start=0
        o=observation(1,world=(-2.5,0,0));o['fork_uncertainty']['predicted_arrival_std_m']=.1
        c.step(1,o);self.assertFalse(c.blind);self.assertEqual(c.command,'stop')
        o['capture_id']=o['arrival']=10;c.step(10,o);self.assertEqual(c.state,'RECOVER')
        o['capture_id']=o['arrival']=62;c.step(62,o);self.assertEqual(c.state,'FAULT')
    def test_sdk_rig_yaml_is_readable_by_opencv_and_uses_actual_extrinsics(self):
        import tempfile,json
        from types import SimpleNamespace
        from docking.sensor_calibration import prepare
        class Profile:
            def __init__(self,x,w=640,h=480):self.x=x;self.w=w;self.h=h
            def as_video_stream_profile(self):return self
            def get_intrinsics(self):return SimpleNamespace(model='distortion.none',width=self.w,height=self.h,fx=380.,fy=380.,ppx=self.w/2,ppy=self.h/2,coeffs=[0.]*5)
            def get_extrinsics_to(self,target):return SimpleNamespace(rotation=np.eye(3).ravel(order='F').tolist(),translation=[self.x-target.x,0.,0.])
        with tempfile.TemporaryDirectory() as folder:
            profiles=(Profile(0),Profile(.05),Profile(.02,1280,720),Profile(.01),Profile(.01))
            bundle,maps,R,file=prepare(self.s,profiles,'SIM',folder)
            fs=cv2.FileStorage(str(file),cv2.FILE_STORAGE_READ)
            self.assertTrue(fs.isOpened());self.assertEqual(fs.getNode('max_cameras').real(),2);fs.release()
            fs=cv2.FileStorage(folder+'/cameras.yaml',cv2.FILE_STORAGE_READ)
            self.assertEqual(fs.getNode('cam0').getNode('T_cam_imu').size(),4);fs.release()
            self.assertAlmostEqual(np.array(bundle['T_left_imu'])[0,3],.01)
            self.assertEqual(bundle['raw_models']['color']['width'],1280)

    def test_hand_eye_rejects_planar_unobservable_motions(self):
        samples=[dict(T_base_body=matrix([i*.1,0,i*.1]).tolist(),T_camera_tag=np.eye(4).tolist()) for i in range(12)]
        with self.assertRaisesRegex(ValueError,'unobservable'):hand_eye(samples)
    def test_hand_eye_recovers_independent_rich_motion(self):
        rng=np.random.default_rng(4);X=np.eye(4);X[:3,:3]=cv2.Rodrigues(np.array([.1,-.2,.3]))[0];X[:3,3]=[.3,-.1,.2]
        D=np.eye(4);D[:3,3]=[2.,1.,.5];samples=[]
        for i in range(20):
            B=np.eye(4);B[:3,:3]=cv2.Rodrigues(rng.normal(0,.3,3))[0];B[:3,3]=rng.normal(0,.3,3)
            samples.append(dict(T_base_body=B,T_camera_tag=inverse(X)@inverse(B)@D))
        report=hand_eye(samples);np.testing.assert_allclose(report['T_body_camera'],X,atol=1e-7)

if __name__=='__main__':unittest.main()
