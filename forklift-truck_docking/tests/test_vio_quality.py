import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
import numpy as np
from support import config,synthetic_tag
from docking.observer import Observer
from docking.vio_quality import quality,check_message_schema


def packet(t):
    return dict(quality_version=3,stamp=t,state_stamp=t,epoch=1,arrival=t+1,imu_arrival=t+1,
        initialized=True,tracking_valid=True,features=50,continued_features=45,
        correction_applied=True,correction_age_s=0.,msckf_update_features=5,slam_update_features=0,
        zero_velocity_update=False,transport_lag_s=.01,gyro_norm=0.,
        T_odom_imu=np.eye(4),covariance=np.eye(6)*.001,
        delta_valid=True,delta_from_stamp=t-.1,relative_transform=np.eye(4),relative_covariance=np.eye(6)*1e-8)


class VioQualityTests(unittest.TestCase):
    def setUp(self):
        self.cfg=config()['sensor'];self.cfg['calibration']['T_body_imu']=np.eye(4).tolist()

    def test_correction_cadence_does_not_equal_tracking_loss(self):
        p=packet(1);p.update(correction_applied=False,correction_age_s=.5,msckf_update_features=0)
        q=quality(p,self.cfg)
        self.assertTrue(q['ok']);self.assertTrue(q['tracking_ok'])
        self.assertFalse(q['corrected_this_frame']);self.assertFalse(q['correction_recent'])
        self.assertEqual(q['reason'],'openvins_tracking_between_updates')
        p['correction_age_s']=1.001;q=quality(p,self.cfg)
        self.assertFalse(q['ok']);self.assertTrue(q['tracking_ok'])
        self.assertEqual(q['reason'],'correction_stale')

    def test_missing_correction_loss_delay_and_mismatch_still_block(self):
        for change,reason in [
            ({'correction_age_s':-1},'waiting_for_correction'),
            ({'continued_features':0},'insufficient_continuous_tracks'),
            ({'features':19},'insufficient_continuous_tracks'),
            ({'tracking_valid':False},'insufficient_continuous_tracks'),
            ({'state_stamp':.9},'state_timestamp_mismatch'),
            ({'transport_lag_s':3.01},'vio_output_delayed'),   # motion_age_s 3.0 (2026-10-20); 0.3 s applies at rest in the controller
            ({'imu_arrival':-1.1},'imu_stale')]:   # packet arrival 2.0: IMU 3.1 s old > imu_age_s 3.0 (2026-10-20)
            with self.subTest(reason=reason):
                p=packet(1);p.update(change);q=quality(p,self.cfg)
                self.assertFalse(q['ok']);self.assertEqual(q['reason'],reason)
        self.assertFalse(quality(packet(1),self.cfg,continuity=False)['ok'])

    def test_old_build_gives_rebuild_instruction(self):
        class Old:
            @staticmethod
            def get_fields_and_field_types():return {'visual_valid':'bool'}
        with self.assertRaisesRegex(RuntimeError,'rebuild_openvins'):check_message_schema(Old)
        p=packet(1);p['quality_version']=0
        with self.assertRaisesRegex(RuntimeError,'rebuild_openvins'):quality(p,self.cfg)

    def test_good_tracking_does_not_grant_docking_without_anchor(self):
        o=Observer(self.cfg);o.add_vio(packet(0));r=o.add_vio(packet(.1))
        self.assertTrue(r['odom']['ok']);self.assertFalse(r['usable'])
        self.assertFalse(r['odom']['usable_for_docking'])

    def test_timestamp_mismatch_never_enters_history_and_resets_anchor(self):
        o=Observer(self.cfg);o.add_vio(packet(0));o.add_vio(packet(.1))
        p=packet(.2);p['state_stamp']=.1;r=o.add_vio(p)
        self.assertFalse(r['local_valid']);self.assertFalse(r['stationary'])
        self.assertEqual(len(o.history.items),0);self.assertIsNone(o.last_state)
        r=o.add_vio(packet(.3));self.assertFalse(r['local_valid']);self.assertIsNone(r['pose'])
        self.assertTrue(o.add_vio(packet(.4))['local_valid'])

    def test_intermittent_updates_keep_tracking_and_stale_correction_blocks_local_validity(self):
        o=Observer(self.cfg)
        for i in range(270):
            p=packet(i/30);p.update(delta_from_stamp=(i-1)/30,correction_age_s=(i%18)/30,correction_applied=i%18==0)
            r=o.add_vio(p)
        self.assertTrue(r['stationary']);self.assertTrue(r['odom']['ok'])
        p=packet(9.);p.update(delta_from_stamp=269/30,correction_age_s=1.1,correction_applied=False)
        r=o.add_vio(p);self.assertFalse(r['local_valid']);self.assertFalse(r['usable'])
        # Stationarity is a gyro fact (2026-10-19); the controller separately requires local_valid before acting.
        self.assertTrue(r['stationary'])

    def test_pose_needs_a_tag_window_and_sigma_is_recorded_not_gated(self):
        o=Observer(self.cfg)
        for i in range(12):
            r=o.add_vio(packet(i*.1))
            if i in (3,4,5,6):o.add_tag(synthetic_tag(self.cfg,(-2.,0.,0.),i*.1))   # 4 frames inside one 0.5 s window
            if i==3:self.assertFalse(r['usable'])
        r=o.add_vio(packet(1.2))
        self.assertTrue(r['odom']['ok']);self.assertTrue(r['usable']);self.assertIsNotNone(r['pose'])
        self.assertIsNotNone(r['confidence_radius']);self.assertEqual(r['sigma']['baseline_source'],'default_20261002_calibrate_not_measured_here')

    def test_initialized_zupt_only_stream_can_finish_stationary_baseline(self):
        o=Observer(self.cfg)
        for i in range(230):
            p=packet(i/30);p.update(delta_from_stamp=(i-1)/30,msckf_update_features=0,zero_velocity_update=True)
            r=o.add_vio(p)
        self.assertTrue(r['imu']['ready']);self.assertTrue(r['stationary'])
        self.assertTrue(r['odom']['ok']);self.assertEqual(r['odom']['correction_source'],'zupt')
        self.assertFalse(r['usable']) # An initialized local filter is not a dock anchor.

    @unittest.skipUnless(shutil.which('c++'),'C++ compiler required for native adapter quality checks')
    def test_native_quality_tracking_and_correction_evidence(self):
        root=Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            exe=str(Path(tmp)/'quality_test')
            subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror',
                '-I',str(root/'ros2/docking_openvins/src'),str(root/'tests/native_quality.cpp'),'-o',exe],check=True,capture_output=True)
            subprocess.run([exe],check=True,capture_output=True)
