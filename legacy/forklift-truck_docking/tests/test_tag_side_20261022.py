"""Tag side is the sign of left_tag_offset_m (2026-10-22): left +, right -. A right-side tag used to be refused by the loader."""
import json,tempfile,unittest
import numpy as np
from pathlib import Path
from docking import config
from docking.fusion import pnp_dock_pose
from docking.geometry import matrix
from docking.uncertainty import inverse

class TagSideTests(unittest.TestCase):
    def load_with(self,offset,mode='left'):
        doc=json.loads(Path(config.COMMON).read_text());doc['fields']['site']['fields']['left_tag_offset_m']['value']=offset
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'common.json';p.write_text(json.dumps(doc));return config.load(p,mode=mode)
    def test_right_tag_is_negative(self):
        cal=self.load_with(-1.27)['sensor']['calibration']
        self.assertAlmostEqual(cal['T_dock_tag'][1][3],-1.27)
    def test_left_tag_is_positive(self):
        self.assertAlmostEqual(self.load_with(1.27)['sensor']['calibration']['T_dock_tag'][1][3],1.27)
    def test_central_ignores_offset(self):
        self.assertAlmostEqual(self.load_with(-1.27,'central')['sensor']['calibration']['T_dock_tag'][1][3],0.)
    def test_non_number_rejected(self):
        with self.assertRaises(ValueError):self.load_with('x')
    def test_right_tag_pose_round_trip(self):
        # Vehicle standing 3 m before the entrance, 0.2 m right of centre, straight: the PnP pose must come back the same
        # with the tag on the right, i.e. the sign flows through T_dock_tag into the dock pose.
        cal=self.load_with(-1.27)['sensor']['calibration'];truth=[-3.,-.2,0.]
        Tct=inverse(cal['T_body_camera'])@inverse(matrix(truth))@np.array(cal['T_dock_tag'])
        np.testing.assert_allclose(pnp_dock_pose(Tct,cal['T_body_camera'],cal['T_dock_tag']),truth,atol=1e-9)
        self.assertGreater(Tct[0,3],0.)   # tag lies to the camera's right (camera x axis points right: body y = -camera x)

if __name__=='__main__':unittest.main()
