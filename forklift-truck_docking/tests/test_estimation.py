"""Synthetic invariants: geometry, capture-time history, tag-main fusion through the observer (2026-10-19)."""
import math
import unittest
import numpy as np
import cv2
from docking.uncertainty import *
from docking.history import PoseHistory,State
from docking.observer import Observer
from docking.stream import ImuSynchronizer
from docking.tag import pnp_covariance,dock_measurement,depth_check
from docking.sensor_calibration import rectification,validate_override
from docking.controller import Controller
from support import config,observation,synthetic_tag,VioSource


class Estimation(unittest.TestCase):
    def setUp(self):self.c=config();self.s=self.c['sensor'];self.u=self.s['uncertainty']
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
        return dict(quality_version=2,tracking_valid=True,continued_features=50,state_stamp=t,
            msckf_update_features=10,slam_update_features=0,correction_applied=True,correction_age_s=0.,zero_velocity_update=False,
            delta_valid=True,delta_from_stamp=previous[0],relative_transform=D,relative_covariance=np.eye(6)*1e-9,stamp=t,epoch=epoch,arrival=t+1,imu_arrival=t+1,features=50,initialized=True,visual_valid=True,
            T_odom_imu=T,covariance=np.eye(6)*1e-5,transport_lag_s=.01,gyro_norm=0.)
    def observer(self):
        self.s['calibration']['T_body_imu']=np.eye(4).tolist();return Observer(self.s)
    def test_tag_windows_recover_the_dock_pose_in_an_arbitrary_odom_gauge(self):
        o=self.observer();src=VioSource(gauge=(3.,-2.,.7));truth=(-5.,.2,.05)
        for i in range(10):
            r=o.add_vio(src.packet(i*.1,truth))
            if i>=2:o.add_tag(synthetic_tag(self.s,truth,i*.1))
        self.assertTrue(r['usable']);np.testing.assert_allclose(r['pose'],truth,atol=1e-6)
        self.assertEqual(r['heading_source'],'vio_yaw_plus_offset');self.assertEqual(r['tag_fusion']['reason'],'refreshed')
    def test_physical_motion_is_vio_propagation_not_a_tag_correction_or_command(self):
        o=self.observer();src=VioSource()
        for i in range(10):
            r=o.add_vio(src.packet(i*.1,(-5.,0.,0.)));o.add_tag(synthetic_tag(self.s,(-5.,0.,0.),i*.1))
        for i in range(10,23):r=o.add_vio(src.packet(i*.1,(-5.+(i-9)*.1,0.,0.)))   # 1.3 m forward, no tag
        self.assertAlmostEqual(r['pose'][0],-3.7,places=6);self.assertTrue(r['usable'])
        self.assertAlmostEqual(r['distance_since_anchor_m'],1.3,places=6)
        self.assertGreater(r['sigma']['vio_since_anchor'][0],0.)
    def test_biased_pnp_heading_does_not_move_the_fused_lateral(self):
        """2026-10-02 failure mode: after an in-place turn the PnP heading is off by 2 deg -> PnP lateral off by
        ~10 cm at 3 m. Bearing/range + VIO-continued heading keep the fused lateral within 1 cm."""
        o=self.observer();src=VioSource();a=(-3.,0.,0.);b=(-3.,0.,math.radians(10))
        for i in range(8):
            o.add_vio(src.packet(i*.1,a));o.add_tag(synthetic_tag(self.s,a,i*.1))
        for i in range(8,20):o.add_vio(src.packet(i*.1,(-3.,0.,math.radians(10)*(i-7)/12)))   # turning, no tag
        for i in range(20,28):
            r=o.add_vio(src.packet(i*.1,b));o.add_tag(synthetic_tag(self.s,b,i*.1,heading_bias_rad=math.radians(2)))
        self.assertLess(abs(r['pose'][1]),.01)
        self.assertGreater(abs(r['tag_fusion']['raw_tag_pose'][1]),.08)
        self.assertAlmostEqual(math.degrees(r['pose'][2]),10.,delta=.2)
    def test_lost_clone_continuity_invalidates_the_anchor_instead_of_inventing_noise(self):
        o=self.observer()
        for i in range(8):
            r=o.add_vio(self.packet(i*.1,0));o.add_tag(synthetic_tag(self.s,(-2.,0.,0.),i*.1))
        self.assertTrue(r['usable'])
        packet=self.packet(.8,.1);packet['delta_valid']=False
        r=o.add_vio(packet);self.assertFalse(r['usable']);self.assertIsNone(r['pose'])
    def test_impossible_tag_jump_is_skipped_then_accepted_after_1s(self):
        o=self.observer();src=VioSource()
        for i in range(8):
            r=o.add_vio(src.packet(i*.1,(-4.,0.,0.)));o.add_tag(synthetic_tag(self.s,(-4.,0.,0.),i*.1))
        reasons=[]
        for i in range(8,22):   # VIO says still, the tag says 1 m to the left: physically impossible -> skipped, then the tag wins
            r=o.add_vio(src.packet(i*.1,(-4.,0.,0.)));reasons.append(o.add_tag(synthetic_tag(self.s,(-4.,1.,0.),i*.1))['reason'])
        self.assertIn('tag_jump_skipped',reasons);self.assertIn('accepted_after_skip',reasons)
        self.assertAlmostEqual(r['pose'][1],1.,places=6)

    def test_native_rotation_covariance_never_disappears(self):
        self.s['calibration']['T_body_imu']=np.eye(4).tolist();o=Observer(self.s)
        state=o.body_state(self.packet(1,0));np.testing.assert_allclose(state.P,np.eye(6)*1e-5)
        self.s['calibration']['T_body_imu'][0][3]=1.5
        state=o.body_state(self.packet(2,0));self.assertGreater(state.P[1,1],3e-5)
    def test_stationary_is_gyro_based_and_requires_neutral(self):
        o=self.observer()
        for i in range(60):
            for k in range(7):o.add_gyro(i/30+k/200,[0.,0.,0.])
            a=o.add_vio(self.packet(i/30,0))
        self.assertTrue(a['stationary']);self.assertEqual(a['stationary_report']['gyro_source'],'raw_200hz')
        for i in range(60,75):   # half a second of 3 deg/s: the 0.5 s window is no longer quiet
            for k in range(7):o.add_gyro(i/30+k/200,[0.,0.,math.radians(3.)])
            a=o.add_vio(self.packet(i/30,0))
        self.assertFalse(a['stationary']);self.assertEqual(a['stationary_report']['reason'],'rotating')
        a=o.add_vio(self.packet(2.6,0),neutral=False);self.assertFalse(a['stationary'])
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
        q.add('gyro',2,[0,0,0]);self.assertIsNone(q.fault);self.assertEqual(q.status()['gaps'],1)   # gaps are counted, not fatal (v4 rule, 2026-10-19)
        self.assertIn('reason=sample_gap',q.status()['last_gap']);self.assertAlmostEqual(q.status()['largest_gap_s'],1.,places=6)
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
    def test_sigma_is_recorded_but_does_not_block_blind_entry(self):
        """2026-10-19 (apriltag_v5 rule): |predicted arrival| <= 3 cm decides; σ goes to the log."""
        c=Controller(self.c,-2.5,0);c.state='PLAN';c.mission_start=0
        o=observation(1,world=(-2.5,0,0));o['fork_uncertainty']['predicted_arrival_std_m']=.1;o['sigma']=dict(baseline=[.1,.1,.01])
        c.step(1,o);self.assertTrue(c.blind);self.assertEqual(c.command,'forward')
        self.assertAlmostEqual(c.status()['lateral_check']['std_m'],.1);self.assertEqual(c.status()['lateral_check']['sigma']['baseline'][0],.1)
        c=Controller(self.c,-2.5,0);c.state='PLAN';c.mission_start=0
        c.step(1,observation(1,world=(-2.5,.05,0)));self.assertFalse(c.blind);self.assertEqual(c.command,'stop')   # 5 cm off: correction, not entry
    def test_sdk_rig_yaml_is_readable_by_opencv_and_uses_actual_extrinsics(self):
        import tempfile,json
        from pathlib import Path
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
            # Exercise every matrix as OpenVINS' Eigen parser does. Merely
            # opening the YAML or reading an unrelated node misses aliases.
            fs=cv2.FileStorage(folder+'/imu.yaml',cv2.FILE_STORAGE_READ)
            node=fs.getNode('imu0')
            for name in ('Tw','Ta','R_IMUtoACC','R_IMUtoGYRO','Tg'):
                m=node.getNode(name)
                self.assertTrue(m.isSeq(),name);self.assertEqual(m.size(),3,name)
                actual=[[m.at(r).at(c).real() for c in range(3)] for r in range(3)]
                np.testing.assert_array_equal(actual,np.zeros((3,3)) if name=='Tg' else np.eye(3))
            fs.release()
            self.assertNotIn('&id',(Path(folder)/'imu.yaml').read_text())

if __name__=='__main__':unittest.main()
