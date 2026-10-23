import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from docking.recording import Recorder
from docking.serialization import numpy_json

class RecordingNumpyTest(unittest.TestCase):
    def test_nested_sensor_types_written_to_stream_and_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            r=Recorder(directory,{})
            value={'ok':np.bool_(True),'nested':{'n':np.int64(2),'p':np.array([1.,2.]),'x':np.float32(.5)}}
            self.assertTrue(r.put(value))
            self.assertTrue(r.put(value,'metrics.json'))
            status=r.close()
            self.assertIsNone(status['error'],status)
            self.assertEqual(status['written'],2)
            for name in ('observations.jsonl','metrics.json'):
                actual=json.loads((Path(directory)/name).read_text())
                self.assertIs(actual['ok'],True)
                self.assertEqual(actual['nested'],dict(n=2,p=[1.,2.],x=.5))
    def test_invalid_values_still_fail(self):
        with self.assertRaises(TypeError):json.dumps(object(),default=numpy_json)
        with self.assertRaises(ValueError):json.dumps(np.array([np.nan]),default=numpy_json,allow_nan=False)
