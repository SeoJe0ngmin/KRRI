"""Bird's-eye-view projection (2026-10-22): floor removed, two cones land where they stand, guide lines project below the horizon."""
import math,unittest
import numpy as np
from docking.bev import mount_rotation,depth_to_points,camera_to_body,occupancy,grid_xy,ground_line,body_to_pixels

INTR=dict(fx=380.,fy=380.,cx=320.,cy=240.)
H=.46

def scene(cones,pitch_deg=0.):
    """Render a floor plane plus vertical cylinders (x, y, radius, height in body metres) into a 480x640 depth image."""
    R=mount_rotation(pitch_deg);t=np.array([0.,0.,H]);depth=np.zeros((480,640),np.float32)
    v,u=np.mgrid[0:480,0:640];d=np.stack([(u-INTR['cx'])/INTR['fx'],(v-INTR['cy'])/INTR['fy'],np.ones_like(u,float)],-1)@R.T   # ray dirs in body
    down=d[...,2]<-1e-6;zf=np.where(down,-H/np.where(down,d[...,2],1),np.inf)                                               # floor hit (camera z units == ray scale)
    z=zf.copy()
    for (x,y,r,h) in cones:
        # ray-cylinder intersection in body xy
        a=d[...,0]**2+d[...,1]**2;b=-2*(d[...,0]*x+d[...,1]*y);c=x*x+y*y-r*r;disc=b*b-4*a*c
        s=np.where(disc>=0,(-b-np.sqrt(np.maximum(disc,0)))/(2*a+1e-12),np.inf)
        hit=(s>0)&(H+s*d[...,2]>=0)&(H+s*d[...,2]<=h);z=np.where(hit&(s<z),s,z)
    z[~np.isfinite(z)]=0
    return z,R,t

class BevTests(unittest.TestCase):
    def test_floor_is_removed_and_cones_are_placed(self):
        z,R,t=scene([(3.3,.7,.15,.6),(3.3,-.7,.15,.6)])
        pts,_,_=depth_to_points(z,INTR,2);body=camera_to_body(pts,R,t)
        grid=occupancy(body,8.,3.,.05)
        rows,cols=np.nonzero(grid);xs=[grid_xy(r,c)[0] for r,c in zip(rows,cols)];ys=[grid_xy(r,c)[1] for r,c in zip(rows,cols)]
        self.assertTrue(len(rows));self.assertTrue(all(3.0<x<3.6 for x in xs),(min(xs),max(xs)))          # nothing from the floor
        self.assertTrue(any(y>.5 for y in ys) and any(y<-.5 for y in ys))                                      # one blob each side
    def test_pitch_is_compensated(self):
        z,R,t=scene([(2.5,0.,.15,.6)],pitch_deg=5.)
        pts,_,_=depth_to_points(z,INTR,2);body=camera_to_body(pts,R,t)
        floorish=body[(body[:,2]<.03)&(body[:,2]>-.03)];self.assertGreater(len(floorish),1000)                 # floor lands at z=0 even when pitched
        grid=occupancy(body);rows,cols=np.nonzero(grid);xs=[grid_xy(r,c)[0] for r,c in zip(rows,cols)]
        self.assertTrue(all(2.2<x<2.8 for x in xs))
    def test_guide_line_projects_below_the_horizon_and_converges(self):
        R=mount_rotation(0.);t=np.array([0.,0.,H])
        line=ground_line(.64,.5,6.,R,t,INTR)
        self.assertTrue((line[:,1]>INTR['cy']).all());self.assertLess(line[-1,1],line[0,1])                   # below centre, rising with distance
        u,v,zc=body_to_pixels(np.array([[2.,0.,0.]]),R,t,INTR);self.assertAlmostEqual(u[0],320.);self.assertGreater(v[0],240.)

class EntranceFrameTests(unittest.TestCase):
    def test_vehicle_pose_relative_to_a_facing_entrance(self):
        from docking.bev import entrance_frame,vehicle_in_entrance,to_entrance
        f=entrance_frame((3.3,.7),(3.3,-.7));pos,hdg=vehicle_in_entrance(f)
        self.assertAlmostEqual(pos[0],-3.3);self.assertAlmostEqual(pos[1],0.);self.assertAlmostEqual(hdg,0.)
        self.assertAlmostEqual(to_entrance(np.array([[4.3,0.]]),f)[0][0],1.)        # 1 m inside the truck
    def test_offset_and_turned_vehicle(self):
        from docking.bev import entrance_frame,vehicle_in_entrance
        th=math.radians(10.);c,s=math.cos(th),math.sin(th)
        # the truck axis is rotated +10 deg (to the left) relative to the vehicle, and its centre is 0.2 m to the vehicle's right
        centre=np.array([3.,-.2]);tau=np.array([-s,c]);l=centre+.7*tau;r=centre-.7*tau
        f=entrance_frame(l,r);pos,hdg=vehicle_in_entrance(f)
        self.assertAlmostEqual(math.degrees(hdg),-10.,places=6)                       # vehicle heading is -10 deg relative to the truck axis
        self.assertAlmostEqual(pos[1],(-centre)@tau,places=9)

if __name__=='__main__':unittest.main()
