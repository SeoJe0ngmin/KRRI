import importlib.util
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from docking.ros_adapter import check_build_ready

SPEC=importlib.util.spec_from_file_location('build_support',Path(__file__).resolve().parents[1]/'scripts/build_support.py')
build=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(build)


class BuildRecoveryTests(unittest.TestCase):
    def test_1970_or_future_source_requires_clock_fix(self):
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)/'VioState.msg';source.write_text('uint32 quality_version\n')
            now=time.time()
            for bad in (0,now-120):
                with self.assertRaisesRegex(RuntimeError,'시계'):build.check_clock(d,bad)
            build.check_clock(d,now)
            self.assertEqual(source.read_text(),'uint32 quality_version\n')

    def test_stale_generated_headers_removed_from_search_paths_only(self):
        with tempfile.TemporaryDirectory() as d:
            ws=Path(d)
            for base in ('src','build','install'):
                for package in (*build.PACKAGES,'ov_msckf'):
                    p=ws/base/package/'artifact';p.parent.mkdir(parents=True,exist_ok=True);p.write_text('old')
                    os.utime(p,(time.time()+1e5,)*2)
            backup=build.quarantine_generated(ws)
            for package in build.PACKAGES:
                self.assertTrue((ws/'src'/package/'artifact').exists())
                for base in ('build','install'):
                    self.assertFalse((ws/base/package).exists())
                    self.assertEqual((backup/base/package/'artifact').read_text(),'old')
            self.assertTrue((ws/'install/ov_msckf/artifact').exists())
            self.assertTrue((ws/'build/ov_msckf/artifact').exists())

    def test_merged_or_external_cache_is_not_moved(self):
        with tempfile.TemporaryDirectory() as d:
            ws=Path(d);(ws/'install').mkdir()
            (ws/'install/.colcon_install_layout').write_text('merged')
            with self.assertRaisesRegex(RuntimeError,'isolated'):build.quarantine_generated(ws)
            (ws/'install/.colcon_install_layout').unlink()
            (ws/'external').mkdir();(ws/'build').symlink_to(ws/'external',target_is_directory=True)
            with self.assertRaisesRegex(RuntimeError,'경로'):build.quarantine_generated(ws)

    def test_failed_build_blocks_runtime_until_rebuilt(self):
        with tempfile.TemporaryDirectory() as d,patch.dict(os.environ,{'DOCK_WS':d}):
            marker=Path(d)/'.docking_rebuild_pending';marker.touch()
            with self.assertRaisesRegex(RuntimeError,'재빌드가 완료되지'):check_build_ready()
            marker.unlink();check_build_ready()
