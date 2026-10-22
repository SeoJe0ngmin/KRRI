import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from docking.runtime import note,parser

class NoteTests(unittest.TestCase):
    def test_appends_entries_and_refuses_non_run_folders(self):
        with tempfile.TemporaryDirectory() as d:
            run=Path(d)/'20261020_120000_abc_drive';run.mkdir();(run/'observations.jsonl').write_text('')
            a=parser().parse_args(['note','--folder',str(run),'--lateral-cm','2.5','--heading-deg','-0.4','--memo','first'])
            note(a);note(SimpleNamespace(folder=str(run),lateral_cm=None,heading_deg=None,longitudinal_cm=3.,memo=''))
            entries=json.loads((run/'manual_measurement.json').read_text())
            self.assertEqual(len(entries),2);self.assertEqual(entries[0]['fork_lateral_cm'],2.5);self.assertEqual(entries[1]['longitudinal_cm'],3.)
            with self.assertRaises(SystemExit):note(SimpleNamespace(folder=d,lateral_cm=1.,heading_deg=None,longitudinal_cm=None,memo=''))

if __name__=='__main__':unittest.main()
