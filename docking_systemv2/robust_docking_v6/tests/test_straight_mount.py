import unittest
import numpy as np
import cv2
from docking.straight_mount import estimate
from docking.frames import R_OPTICAL_TO_BODY,R_REFERENCE_TAG
from support import config

class StraightMountTest(unittest.TestCase):
    def rows(self,yaw=7.,heading=25.,travel=1.,turn=False):
        def rz(d):
            a=np.radians(d);return np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1.]])
        Rbc=rz(yaw)@R_OPTICAL_TO_BODY;Rdb=rz(heading)
        Rct=Rbc.T@Rdb.T@R_REFERENCE_TAG
        rows=[]
        for i in range(121):
            t=i*.1;d=travel*np.clip((t-4)/4,0,1)
            xyz=Rbc.T@(Rdb.T@np.array([4.,1.,1.])-np.array([d,0,0]))
            rows.append(dict(arrival=t,capture_id=t,stationary=t<4 or t>8,sensor_ready=True,local_valid=True,
                vio_gyro_norm=0.,local_pose=[d,0,.1*d if turn else 0],sensor_epoch=1,motion_epoch=1,
                tag=dict(ok=True,reason='accepted',capture_key=str(i),camera_translation_m=xyz.tolist(),
                    camera_rotation_vector_rad=cv2.Rodrigues(Rct)[0].ravel().tolist())))
        return rows,Rbc
    def test_oblique_tag_and_unknown_mount_recovered(self):
        for yaw in (-8,0,7):
            rows,R=self.rows(yaw=yaw)
            r=estimate(rows,config()['sensor'],{})
            self.assertTrue(r['usable'],r)
            np.testing.assert_allclose(r['R_body_camera'],R,atol=1e-8)
            self.assertAlmostEqual(r['yaw_left_deg'],yaw)
    def test_short_reverse_rotation_and_tracking_loss_rejected(self):
        for kw in (dict(travel=.1),dict(travel=-1.),dict(turn=True)):
            rows,_=self.rows(**kw)
            self.assertFalse(estimate(rows,config()['sensor'],{})['usable'])
        rows,_=self.rows();rows[55]['local_valid']=False
        self.assertFalse(estimate(rows,config()['sensor'],{})['usable'])

    def test_camera_pitch_roll_and_nonzero_initial_heading(self):
        rows,R=self.rows(yaw=6,heading=-20)
        S=cv2.Rodrigues(np.radians(np.array([3.,0.,2.])))[0]
        for row in rows:
            tag=row['tag']
            tag['camera_translation_m']=(S@tag['camera_translation_m']).tolist()
            old=cv2.Rodrigues(np.array(tag['camera_rotation_vector_rad']))[0]
            tag['camera_rotation_vector_rad']=cv2.Rodrigues(S@old)[0].ravel().tolist()
        report=estimate(rows,config()['sensor'],{})
        self.assertTrue(report['usable'],report)
        np.testing.assert_allclose(report['R_body_camera'],R@S.T,atol=1e-8)
    def test_stationary_only_cannot_manufacture_mount(self):
        rows,_=self.rows(travel=0)
        for row in rows:row['stationary']=True
        self.assertFalse(estimate(rows,config()['sensor'],{})['usable'])
    def test_brief_correction_stale_with_tag_and_imu_is_accepted(self):
        rows,_=self.rows()
        for r in rows[50:52]:
            r['local_valid']=False;r['odom']=dict(reason='correction_stale')
        self.assertTrue(estimate(rows,config()['sensor'],{})['usable'])
        rows[51]['tag']['ok']=False
        self.assertFalse(estimate(rows,config()['sensor'],{})['usable'])
    def test_final_boolean_settling_does_not_override_stable_geometry(self):
        rows,_=self.rows()
        for r in rows[-30:]:
            r['stationary']=False;r['stationary_report']=dict(reason='settling_window')
        self.assertTrue(estimate(rows,config()['sensor'],{})['usable'])
        rows[-5]['tag']['camera_translation_m'][0]+=.05
        self.assertFalse(estimate(rows,config()['sensor'],{})['usable'])
    def test_reset_discards_failed_leg_and_accepts_new_announced_leg(self):
        from docking.straight_mount import Collection
        c=Collection();rows,_=self.rows();events=[]
        for r in rows[:60]:events.append(c.add(r))
        broken=dict(rows[60],motion_epoch=2);self.assertEqual(c.add(broken),'restart')
        self.assertFalse(c.ready)
        new,_=self.rows()
        for r in new:
            r['arrival']+=20;r['capture_id']+=20;r['motion_epoch']=2
            events.append(c.add(r))
        self.assertTrue(c.ready);self.assertGreaterEqual(events.count('ready'),2)
        self.assertGreaterEqual(c.rows[0]['arrival'],20)
        self.assertTrue(estimate(c.rows,config()['sensor'],{})['usable'])
    def test_long_invalid_interval_and_missing_gyro_rejected(self):
        rows,_=self.rows()
        for r in rows[50:60]:r['local_valid']=False;r['odom']=dict(reason='correction_stale')
        self.assertFalse(estimate(rows,config()['sensor'],{})['usable'])
        rows,_=self.rows();rows[-1].pop('vio_gyro_norm')
        self.assertFalse(estimate(rows,config()['sensor'],{})['usable'])
