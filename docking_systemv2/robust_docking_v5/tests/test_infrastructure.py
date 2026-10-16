import ast
import json
import tempfile
import time
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from docking.canbus import PAYLOADS,effective,Can
from docking.canhealth import guard
from docking.config import COMMON,load,save_mount
from docking.stream import Imu,Latest
from docking.recording import Recorder
from docking.console import Console
from support import config,ROOT

class Infrastructure(unittest.TestCase):
    def test_can_bytes_match_v4_config_including_fine_turn(self):
        source=ROOT.parent/'apriltag_v4/config/control.py'
        values={}
        for n in ast.parse(source.read_text()).body:
            if isinstance(n,ast.Assign) and isinstance(n.value,ast.Constant):
                for target in n.targets:
                    if isinstance(target,ast.Name):values[target.id]=n.value.value
        self.assertEqual(PAYLOADS['forward'][2],127-values['FORWARD_JOYSTICK_DEFLECTION'])
        self.assertEqual(PAYLOADS['forward_slow'][2],127-values['FORWARD_SLOW_JOYSTICK_DEFLECTION'])
        self.assertEqual(PAYLOADS['backward'][2],127+values['BACKWARD_JOYSTICK_DEFLECTION'])
        for suffix,key in [('slow','ROTATE_JOYSTICK_DEFLECTION'),('fine','ROTATE_FINE_JOYSTICK_DEFLECTION')]:
            self.assertEqual(PAYLOADS['rotate_left_'+suffix][1],127+values[key])
            self.assertEqual(PAYLOADS['rotate_right_'+suffix][1],127-values[key])
        for v in PAYLOADS.values():
            guard(SimpleNamespace(id=0x1E3,data=v,flags=0))
            self.assertTrue(all(v[i]==127 for i in (0,3,4,5,6,7)))
    def test_unknown_bytes_lift_combined_extended_or_other_id_blocked(self):
        cases=[(0x1E3,[127,157,67,127,127,127,127,127],0),
               (0x1E3,[127,127,127,127,140,127,127,127],0),
               (0x1E3,[127,127,66,127,127,127,127,127],0),
               (0x2E3,[0x42,0,0,0x15,0,0x40,0x69,0x93],0),
               (0x1E3,[127]*8,4),(0x123,[0],0)]
        for cid,data,flags in cases:
            with self.assertRaises(RuntimeError):guard(SimpleNamespace(id=cid,data=data,flags=flags))
    def test_watchdog_expiry_and_unknown_index_latch(self):
        self.assertEqual(effective(1,10,False,10.01),(0,True))
        self.assertEqual(effective(99,20,False,10),(0,True))
        self.assertEqual(effective(1,20,True,10),(0,True))
    def test_plan_status_is_json_serializable_and_immutable_for_writer(self):
        from docking.controller import Controller
        from docking.planner import plan
        from support import observation
        c=config();x=Controller(c,-2.275,0.)
        x.state='PLANNING';x.plan_id=1
        import numpy as np
        x.await_plan=dict(pose=np.array([-4.,0,0]),epoch=0,at=0.)
        result=plan(c,[-4,0,0],-2.275)
        x.accept_plan(dict(id=1,plan=result),.1,observation(.1,world=(-4,0,0)))
        before=x.status();json.dumps(before,allow_nan=False)
        x.responses.data['forward']=dict(speed=.2,coast=.5,n=1)
        self.assertNotIn('forward',before['responses'])
    def test_latest_capture_replaces_backlog(self):
        q=Latest()
        for i in range(1000):q.put(i)
        self.assertEqual(q.get(.01),999);self.assertEqual(q.replaced,999)
        self.assertIsNone(q.get(.001))
    def test_imu_exposure_interpolation_contains_rate_and_gap_state(self):
        c=config()['sensor']['imu'];c['startup_calibration']['enabled']=False
        i=Imu(c);i.add(1,[0,-.1,0]);i.add(1.01,[0,-.2,0])
        value=i.at(1.005);self.assertAlmostEqual(value['rate_rps'],.15)
        self.assertEqual(value['gaps'],0);self.assertIsNone(i.at(2))
    def test_recorder_backpressure_does_not_block_producer(self):
        with tempfile.TemporaryDirectory() as d:
            with patch.object(Recorder,'_run',lambda _:None):r=Recorder(d,{'queue_size':2})
            t=time.monotonic()
            for i in range(1000):r.put({'value':i})
            self.assertLess(time.monotonic()-t,.1);self.assertEqual(r.status()['dropped'],998);r.close()
    def test_stalled_ssh_console_does_not_block_supervisor(self):
        gate=threading.Event()
        with patch.object(Console,'_run',lambda _:gate.wait(1.)):
            c=Console();t=time.monotonic()
            for _ in range(1000):c.write('stalled')
            self.assertLess(time.monotonic()-t,.1);self.assertGreater(c.dropped,0)
            gate.set();c.close()
    def test_unfilled_dimensions_do_not_arm_drive(self):
        with self.assertRaisesRegex(ValueError,'미입력'):load(require_mount=True)
    def test_mount_write_preserves_descriptions_and_detects_concurrent_edit(self):
        from docking.mount_calibration import R_OPTICAL_TO_BODY
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'common.json';p.write_bytes(COMMON.read_bytes());raw=p.read_bytes()
            c=config();report=dict(usable=True,R_body_camera=R_OPTICAL_TO_BODY.tolist(),camera_model=c['sensor']['calibration']['camera_model'])
            save_mount(p,report,raw);doc=json.loads(p.read_text())
            self.assertEqual(doc['fields']['site'],json.loads(raw)['fields']['site'])
            self.assertIsNotNone(load(p)['sensor']['calibration']['T_body_camera'])
            with self.assertRaises(ValueError):save_mount(p,report,raw)
            doc['fields']['site']['fields']['camera_height_m']['value']=.8;p.write_text(json.dumps(doc))
            self.assertIsNone(load(p)['sensor']['calibration']['T_body_camera'])

if __name__=='__main__':unittest.main()
