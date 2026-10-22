import math,unittest
import numpy as np
from docking.depth_goal import find_goal

INTR=dict(fx=380.,cx=320.)

def scene(left_wall_u,right_wall_u,wall_z=2.,inside_z=4.,holes=0.):
    z=np.full((480,640),inside_z,np.float32)
    z[:,:left_wall_u]=wall_z;z[:,right_wall_u:]=wall_z
    if holes:
        rng=np.random.default_rng(1);z[rng.random(z.shape)<holes]=0
    return z

class DepthGoalTests(unittest.TestCase):
    def test_centred_opening(self):
        r=find_goal(scene(200,440),INTR)
        self.assertTrue(r['ok'],r);self.assertAlmostEqual(r['u_goal'],320,delta=1.)
        self.assertAlmostEqual(r['goal_lateral_m'],0.,delta=.01)
        self.assertAlmostEqual(r['width_m'],240/380*2.,delta=.03)
    def test_opening_to_the_left_is_left_positive(self):
        r=find_goal(scene(100,300),INTR)
        self.assertTrue(r['ok']);self.assertGreater(r['goal_lateral_m'],.2);self.assertGreater(r['goal_bearing_deg'],0)
    def test_survives_holes(self):
        r=find_goal(scene(200,440,holes=.3),INTR)
        self.assertTrue(r['ok'],r);self.assertAlmostEqual(r['u_goal'],320,delta=3.)
    def test_flat_wall_has_no_goal(self):
        r=find_goal(np.full((480,640),3.,np.float32),INTR)
        self.assertFalse(r['ok']);self.assertEqual(r['reason'],'no_depth_edges')
    def test_preview_draws_goal(self):
        import cv2
        from docking.webview import encode_preview
        z=(scene(200,440)*1000).astype(np.uint16)
        goal=find_goal(z.astype(np.float32)*.001,INTR)
        obs=dict(depth=dict(goal,x_body_m=2.,lateral_body_m=0.),tag=dict(reason='accepted'),odom=dict(reason='openvins_corrected'))
        img=cv2.imdecode(np.frombuffer(encode_preview(z,obs,dict(width=640,jpeg_quality=90,depth_max_m=6.),'depth',dict(depth_scale=.001)),np.uint8),cv2.IMREAD_COLOR)
        b,g,r=img[60,320].astype(int)
        self.assertGreater(r,g+60);self.assertGreater(r,b+60)

if __name__=='__main__':unittest.main()
