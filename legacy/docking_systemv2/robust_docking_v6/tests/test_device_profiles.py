import unittest
from types import SimpleNamespace
from legacy.docking_systemv2.robust_docking_v6.docking.sensor_calibration import device_profiles, extrinsic

class Profile:
    def __init__(self,kind,graph,width=640):self.kind,self.graph,self.w=kind,graph,width
    def stream_type(self):return self.kind
    def stream_index(self):return 0
    def format(self):return 'y8' if self.is_video_stream_profile() else 'xyz'
    def fps(self):return 30 if self.is_video_stream_profile() else 200
    def is_video_stream_profile(self):return self.kind=='IR'
    def as_video_stream_profile(self):return self
    def width(self):return self.w
    def height(self):return 480
    def get_extrinsics_to(self,target):
        if self.graph!=target.graph:raise RuntimeError('Requested extrinsics are not available')
        return SimpleNamespace(rotation=[1,0,0,0,1,0,0,0,1],translation=[.01,0,0])

class DeviceProfilesTest(unittest.TestCase):
    def device(self,profiles):
        return SimpleNamespace(query_sensors=lambda:[SimpleNamespace(get_stream_profiles=lambda:profiles)])
    def test_independent_pipeline_profiles_use_common_factory_graph(self):
        requested=[Profile('IR','video'),Profile('gyro','motion')]
        with self.assertRaisesRegex(RuntimeError,'gyro -> IR'):
            extrinsic(requested[1],requested[0],'gyro -> IR')
        factory=[Profile('IR','factory'),Profile('gyro','factory')]
        selected=device_profiles(self.device(factory),requested)
        self.assertIs(selected[0],factory[0]);self.assertIs(selected[1],factory[1])
        self.assertAlmostEqual(extrinsic(selected[1],selected[0])[0,3],.01)
    def test_wrong_resolution_or_ambiguous_profile_rejected(self):
        p=Profile('IR','factory')
        for profiles in ([Profile('IR','factory',1280)],[p,p]):
            with self.assertRaisesRegex(RuntimeError,'profile match failed'):
                device_profiles(self.device(profiles),[p])
    def test_missing_factory_extrinsics_still_fail(self):
        selected=device_profiles(self.device([Profile('IR','a'),Profile('gyro','b')]),
                                 [Profile('IR','request'),Profile('gyro','request')])
        with self.assertRaisesRegex(RuntimeError,'gyro -> IR'):
            extrinsic(selected[1],selected[0],'gyro -> IR')
