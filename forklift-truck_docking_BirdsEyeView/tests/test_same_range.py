import math,unittest
from docking.depth_goal import same_range_tolerance

class SameRangeTests(unittest.TestCase):
    def test_tolerance_is_tight_at_rest_and_opens_with_heading(self):
        self.assertAlmostEqual(same_range_tolerance(3.,1.4,0.),max(.2,.45))
        self.assertAlmostEqual(same_range_tolerance(1.,1.4,0.),.2)
        self.assertAlmostEqual(same_range_tolerance(1.,1.4,math.radians(15.)),.2+1.4*math.sin(math.radians(15.)))
        self.assertAlmostEqual(same_range_tolerance(1.,1.4,None),.5)
    def test_band_finder_rejects_a_pair_at_different_depths(self):
        import numpy as np
        from docking.depth_goal import find_goal
        INTR=dict(fx=380.,cx=320.)
        z=np.full((480,640),4.,np.float32);z[:,:200]=2.;z[:,440:]=2.4
        self.assertFalse(find_goal(z,INTR,width_m=1.3,width_tol_m=.3,heading=0.)['ok'])
        self.assertTrue(find_goal(z,INTR,width_m=1.3,width_tol_m=.3,heading=math.radians(15.))['ok'])
        z[:,440:]=2.1;self.assertTrue(find_goal(z,INTR,width_m=1.3,width_tol_m=.3,heading=0.)['ok'])

    def test_display_candidate_prefers_an_equally_deep_pair(self):
        import numpy as np
        from docking.depth_goal import find_goal
        INTR=dict(fx=380.,cx=320.)
        z=np.full((480,640),6.,np.float32);z[:,:120]=1.;z[:,260:300]=3.;z[:,460:500]=3.
        g=find_goal(z,INTR,heading=0.)
        self.assertTrue(g['ok']);self.assertGreater(g['u_left'],250.)
        self.assertAlmostEqual(g['z_left_m'],3.,delta=.01);self.assertAlmostEqual(g['z_right_m'],3.,delta=.01)

if __name__=='__main__':unittest.main()
