"""--panels (2026-10-22): choose which camera panels the dashboard shows and the worker encodes."""
import unittest
from docking.runtime import parser,view_panels
from docking.webview import HTML

class ViewPanelsTests(unittest.TestCase):
    def test_option_parses(self):
        self.assertEqual(parser().parse_args(['measure']).panels,'all')
        self.assertEqual(view_panels('all'),['ir','color','depth']);self.assertEqual(view_panels('depth'),['depth'])
        self.assertEqual(view_panels('depth,color'),['color','depth']);self.assertEqual(view_panels(' COLOR '),['color'])
        self.assertEqual(view_panels(''),['ir','color','depth'])   # empty = all
        for bad in ('rgb','ir,,x','none'):
            with self.assertRaises(ValueError):view_panels(bad)
    def test_dashboard_has_one_section_per_panel(self):
        for k in ('ir','color','depth'):self.assertIn('id="panel-'+k+'"',HTML);self.assertIn('id="preview-'+k+'"',HTML)
        self.assertIn('activeStreams',HTML)

if __name__=='__main__':unittest.main()
