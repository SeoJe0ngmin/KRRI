"""--preview depth (2026-10-21): the depth stream opens for display only and the colormap encoder works on raw z16."""
import unittest
import numpy as np
from docking.runtime import parser

class DepthPreviewTests(unittest.TestCase):
    def test_option_parses_and_defaults_to_ir(self):
        self.assertEqual(parser().parse_args(['measure']).preview,'ir')
        self.assertEqual(parser().parse_args(['measure','--view','--preview','depth']).preview,'depth')
    def test_encoder_colours_depth_and_blacks_out_holes(self):
        import cv2
        from docking.webview import encode_preview
        z=np.full((480,640),2000,np.uint16);z[:100,:]=0            # 2 m everywhere, top rows without data
        obs=dict(preview_stream='depth',depth_scale=.001,tag=dict(reason='accepted'),odom=dict(reason='openvins_corrected'))
        jpg=encode_preview(z,obs,dict(width=640,jpeg_quality=80,depth_max_m=6.))
        img=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR)
        self.assertEqual(img.shape,(480,640,3))
        self.assertLess(img[50,320].max(),40)                       # hole -> black
        self.assertGreater(img[300,100].max(),100)                  # 2 m -> coloured

if __name__=='__main__':unittest.main()
