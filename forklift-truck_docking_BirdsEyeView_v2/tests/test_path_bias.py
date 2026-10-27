import math,unittest
import numpy as np
from support import config
from docking.geometry import arrival_y,path_bias
from docking.uncertainty import target_lateral_uncertainty

class PathBiasTests(unittest.TestCase):
    def test_empty_means_zero(self):
        c=config('left');self.assertEqual(path_bias(c),0.)
        c['site']['forward_path_bias_deg']=None;self.assertEqual(path_bias(c),0.)
    def test_arrival_follows_the_drift(self):
        c=config('left');p=(-3.3,0.,0.);base=arrival_y(p,c)
        c['site']['forward_path_bias_deg']=-1.7
        self.assertLess(arrival_y(p,c),base-.05)
        q=(-3.3,0.,math.radians(1.7));self.assertAlmostEqual(arrival_y(q,c),base,places=9)
    def test_uncertainty_uses_the_same_direction(self):
        p=np.array([-3.3,0.,0.]);P=np.eye(3)*1e-4
        y0,_=target_lateral_uncertainty(p,P,1.);y1,_=target_lateral_uncertainty(p,P,1.,math.radians(-1.7))
        self.assertAlmostEqual(y1-y0,4.3*math.tan(math.radians(-1.7)),places=9)
    def test_config_rejects_absurd_values(self):
        import json,tempfile
        from pathlib import Path
        from docking import config as cfgmod
        doc=json.loads(Path(cfgmod.COMMON).read_text());doc['fields']['site']['fields']['forward_path_bias_deg']['value']=25.
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/'c.json';f.write_text(json.dumps(doc))
            with self.assertRaises(ValueError):cfgmod.load(f,mode='left')

if __name__=='__main__':unittest.main()
