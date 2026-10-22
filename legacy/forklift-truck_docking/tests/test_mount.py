"""Mounting rotation from a stationary log (apriltag_v4 camyaw style). Synthetic rows; the body-parallel-to-normal
assumption is exactly what the fixture provides, so the recovered yaw is the one put in."""
import json
import shutil
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import cv2
import numpy as np
from docking.mount import estimate
from docking.frames import R_OPTICAL_TO_BODY,R_REFERENCE_TAG
from support import config


def rows(yaw=7.,n=350,stationary=True):
    """Camera yawed `yaw` deg on the body; body parallel to the tag normal, 3 m from the tag."""
    a=np.radians(yaw);Rbc=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1.]])@R_OPTICAL_TO_BODY
    Rct=Rbc.T@R_REFERENCE_TAG;rv=cv2.Rodrigues(Rct)[0].ravel().tolist();xyz=(Rbc.T@np.array([3.,0.,1.])).tolist()
    return [dict(arrival=i/5,capture_id=i/5,stationary=stationary,sensor_ready=True,local_valid=True,vio_gyro_norm=.002,
                 tag=dict(ok=True,reason='accepted',capture_key=str(i),stamp=i/5,camera_translation_m=xyz,camera_rotation_vector_rad=rv)) for i in range(n)]


class MountTests(unittest.TestCase):
    def test_yaw_recovered_from_rest(self):
        for yaw in (-8.,0.,7.):
            r=estimate(rows(yaw),config()['sensor'],{})
            self.assertTrue(r['usable'],r);self.assertAlmostEqual(r['yaw_left_deg'],yaw,places=3)
            self.assertEqual(r['warnings'][0],'assumes_vehicle_parallel_to_tag_normal_by_eye')   # the assumption is always on record
    def test_moving_rows_do_not_count_and_short_rest_is_a_warning_not_a_gate(self):
        r=estimate(rows(stationary=False),config()['sensor'],{});self.assertFalse(r['usable'])
        r=estimate(rows(n=60),config()['sensor'],{});self.assertTrue(r['usable']);self.assertTrue(any(w.startswith('rest_short') for w in r['warnings']))
    def test_mount_command_saves_and_drive_config_accepts_it(self):
        from docking.preflight import mount
        from docking.config import COMMON,load
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d)/'run';folder.mkdir()
            (folder/'observations.jsonl').write_text('\n'.join(json.dumps(r) for r in rows())+'\n')
            (folder/'device.json').write_text(json.dumps(dict(T_left_color=np.eye(4).tolist(),raw_models=dict(color=dict(serial='SIM'),left=dict(serial='SIM')))))
            common=Path(d)/'common.json';shutil.copy(COMMON,common)
            mount(SimpleNamespace(folder=str(folder),common=str(common),mode='central',write=True))
            saved=json.loads((Path(d)/'mount.json').read_text());self.assertNotIn('mount',json.loads(common.read_text())['fields'])
            self.assertAlmostEqual(saved['yaw_left_deg'],7.,places=3);self.assertEqual(saved['measured_stream'],'color_converted_to_raw_left_ir')
            self.assertTrue(load(common,'central',require_mount=True)['sensor']['calibration']['mount_valid'])

if __name__=='__main__':unittest.main()


class GravityTiltTests(unittest.TestCase):
    def test_pitch_and_roll_come_from_gravity_not_from_the_tag(self):
        from docking.mount import gravity_tilt
        def ry(a):c,s=np.cos(a),np.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
        def rx(a):c,s=np.cos(a),np.sin(a);return np.array([[1,0,0],[0,c,-s],[0,s,c]])
        for pitch,roll in ((10.,0.),(0.,-4.),(-23.,3.)):
            Rbc=ry(np.radians(pitch))@rx(np.radians(roll))@R_OPTICAL_TO_BODY       # camera tilted on a level body
            T=np.eye(4);T[:3,:3]=Rbc                                               # IMU = camera frame, odom = level body
            vio=[dict(T_odom_imu=T.tolist(),initialized=True) for _ in range(5)]
            p,r,n=gravity_tilt(vio,np.eye(4))
            self.assertAlmostEqual(np.degrees(p),pitch,places=6);self.assertAlmostEqual(np.degrees(r),roll,places=6);self.assertEqual(n,5)
        # a leaning TAG must not become camera pitch: yaw from the tag, tilt from gravity (level here)
        rws=rows(7.)
        for r_ in rws:
            Rct=cv2.Rodrigues(np.array(r_['tag']['camera_rotation_vector_rad']))[0]
            tilt=cv2.Rodrigues(np.array([np.radians(20.),0,0]))[0]                 # tag leaned 20 deg about its x axis
            r_['tag']['camera_rotation_vector_rad']=cv2.Rodrigues(Rct@tilt)[0].ravel().tolist()
        T=np.eye(4);T[:3,:3]=R_OPTICAL_TO_BODY                                     # a level camera: optical frame in the level body frame
        level=[dict(T_odom_imu=T.tolist(),initialized=True) for _ in range(5)]
        rep=estimate(rws,config()['sensor'],{},vio_rows=level,T_left_imu=np.eye(4).tolist())
        self.assertTrue(rep['usable']);self.assertAlmostEqual(rep['yaw_left_deg'],7.,places=2)
        self.assertAlmostEqual(rep['pitch_down_deg'],0.,places=6);self.assertTrue(any(w.startswith('tag_tilted_vs_gravity') for w in rep['warnings']))
