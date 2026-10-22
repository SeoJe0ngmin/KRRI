import json,tempfile,unittest
from pathlib import Path
from docking import config

class TargetOffsetTests(unittest.TestCase):
    def load_with(self,central,left,mode):
        doc=json.loads(Path(config.COMMON).read_text());f=doc['fields']['site']['fields']
        f['target_front_x_central_m']=dict(description='',value=central);f['target_front_x_left_m']=dict(description='',value=left)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'common.json';p.write_text(json.dumps(doc));return config.load(p,mode=mode)
    def test_values_from_common(self):
        self.assertAlmostEqual(self.load_with(-.2,.7,'central')['target_front_x'],-.2)
        self.assertAlmostEqual(self.load_with(-.2,.7,'left')['sensor']['target_front_x'],.7)
    def test_missing_keeps_old_constants(self):
        self.assertAlmostEqual(self.load_with(None,None,'central')['target_front_x'],-.5)
        self.assertAlmostEqual(self.load_with(None,None,'left')['target_front_x'],1.)
    def test_non_number_rejected(self):
        with self.assertRaises(ValueError):self.load_with('x',1.,'central')

if __name__=='__main__':unittest.main()
