"""Device SDK doubles: no hardware and no real CAN transmissions in tests."""
import ast
import io
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from docking import device_check as dc
from docking.__main__ import default_config, main

ROOT = pathlib.Path(__file__).resolve().parents[1]


def config():
    with open(str(ROOT/'configs/hardware.json')) as f:
        return json.load(f)


class Clock:
    def __init__(self):
        self.now = 10.

    def time(self):
        return self.now


class Stop:
    def __init__(self, clock):
        self.clock, self.value, self.tick = clock, False, None

    def is_set(self):
        return self.value

    def set(self):
        self.value = True

    def wait(self, dt):
        self.clock.now += dt
        if self.tick:
            self.tick()
        return self.value


class NoMessage(Exception):
    pass


class CanDouble:
    """Writes throw, including those hidden in cleanup or failure paths."""
    Driver = types.SimpleNamespace(NORMAL=4, SILENT=1)
    Open = types.SimpleNamespace(EXCLUSIVE=8, REQUIRE_INIT_ACCESS=128)
    ChannelCap = types.SimpleNamespace(VIRTUAL=65536, SIMULATED=131072, SILENT_MODE=2097152)
    Bitrate = types.SimpleNamespace(BITRATE_125K=-4, BITRATE_250K=-3, BITRATE_500K=-2, BITRATE_1M=-1)
    MessageFlag = types.SimpleNamespace(ERROR_FRAME=1, OVERRUN=2, TXACK=4, TXRQ=8)
    Stat = types.SimpleNamespace(BUS_OFF=1, ERROR_PASSIVE=2)
    CanNoMsg = NoMessage

    def __init__(self, clock, frames=()):
        self.clock, self.frames = clock, list(frames)
        self.calls, self.count, self.mode = [], 1, self.Driver.NORMAL
        self.cap = self.ChannelCap.SILENT_MODE
        self.params = (250000, 3, 2, 1, 1, 0)
        self.state, self.fail, self.ignore_silent = 0, None, False

    def call(self, name, *args):
        self.calls.append((name, args))
        if self.fail == name:
            raise RuntimeError(name+'_failed')

    def getNumberOfChannels(self):
        self.call('enumerate')
        return self.count

    def ChannelData(self, channel):
        self.call('data', channel)
        return types.SimpleNamespace(channel_name='physical Kvaser', channel_cap=self.cap)

    def openChannel(self, channel, flags):
        self.call('open', channel, flags)
        return self

    def getBusOutputControl(self):
        self.call('get_mode')
        return self.mode

    def getBusParams(self):
        self.call('get_params')
        return self.params

    def setBusParams(self, *args):
        self.call('params', *args)

    def setBusOutputControl(self, value):
        self.call('mode', value)
        if not self.ignore_silent or value != self.Driver.SILENT:
            self.mode = value

    def busOn(self):
        self.call('on')
        assert self.mode == self.Driver.SILENT

    def read(self, timeout):
        self.call('read', timeout)
        self.clock.now += timeout/1000.
        if self.frames:
            return self.frames.pop(0)
        raise NoMessage()

    def readStatus(self):
        self.call('status')
        return self.state

    def busOff(self):
        self.call('off')

    def close(self):
        self.call('close')

    def write(self, *args, **kwargs):
        raise AssertionError('CAN write is forbidden')

    writeWait = writeSync = write


class CanProbeTests(unittest.TestCase):
    def run_probe(self, modify=None, frames=()):
        clock = Clock()
        api = CanDouble(clock, frames)
        if modify:
            modify(api)
        with patch.object(dc.time, 'monotonic', clock.time):
            report = dc.probe_can(config()['can'], 1., Stop(clock), api)['can']
        return api, report

    def test_silent_before_buson_and_restore_only_after_busoff(self):
        frame = types.SimpleNamespace(id=0x123, flags=0)
        api, report = self.run_probe(frames=[frame])
        self.assertEqual(report['status'], 'PASS')
        self.assertEqual(report['tx_calls'], 0)
        self.assertEqual(report['received_ids'], ['0x123'])
        self.assertIn(('open', (0, 136)), api.calls)
        self.assertLess(api.calls.index(('mode', (1,))), api.calls.index(('on', ())))
        self.assertLess(api.calls.index(('off', ())), api.calls.index(('mode', (4,))))
        self.assertIn(('params', api.params), api.calls)
        self.assertEqual(api.calls[-1][0], 'close')

    def test_no_traffic_is_warning_not_wiring_success(self):
        _, report = self.run_probe()
        self.assertEqual(report['status'], 'WARN')
        self.assertIn('wiring_not_confirmed', report['reason'])

    def test_missing_channel_never_opens(self):
        api, report = self.run_probe(lambda a: setattr(a, 'count', 0))
        self.assertEqual(report['status'], 'FAIL')
        self.assertNotIn('open', [c[0] for c in api.calls])

    def test_virtual_simulated_and_no_silent_support_never_open(self):
        for cap in (0, CanDouble.ChannelCap.VIRTUAL, CanDouble.ChannelCap.SIMULATED):
            api, report = self.run_probe(lambda a: setattr(a, 'cap', cap))
            self.assertEqual(report['status'], 'FAIL')
            self.assertNotIn('open', [c[0] for c in api.calls])

    def test_mode_readback_failure_never_goes_on_bus(self):
        api, report = self.run_probe(lambda a: setattr(a, 'ignore_silent', True))
        self.assertEqual(report['status'], 'FAIL')
        self.assertNotIn('on', [c[0] for c in api.calls])
        self.assertEqual(api.calls[-1][0], 'close')

    def test_in_use_channel_fails_without_normal_mode_fallback(self):
        api, report = self.run_probe(lambda a: setattr(a, 'fail', 'open'))
        self.assertEqual(report['status'], 'FAIL')
        self.assertNotIn('on', [c[0] for c in api.calls])

    def test_receive_failure_and_partial_buson_failure_close_resources(self):
        for method in ('read', 'on'):
            api, report = self.run_probe(lambda a: setattr(a, 'fail', method))
            self.assertEqual(report['status'], 'FAIL')
            self.assertIn(('off', ()), api.calls)
            self.assertEqual(api.calls[-1][0], 'close')

    def test_busoff_failure_never_restores_normal(self):
        api, report = self.run_probe(lambda a: setattr(a, 'fail', 'off'))
        self.assertEqual(report['status'], 'FAIL')
        self.assertNotIn(('mode', (4,)), api.calls)
        self.assertEqual(api.calls[-1][0], 'close')

    def test_bus_fault_and_error_frames_fail(self):
        for state in (1, 2):
            _, report = self.run_probe(lambda a: setattr(a, 'state', state))
            self.assertEqual(report['status'], 'FAIL')
        for flag in (1, 2):
            _, report = self.run_probe(frames=[types.SimpleNamespace(id=0, flags=flag)])
            self.assertEqual(report['status'], 'FAIL')
            self.assertEqual(report['rx_frames'], 0)

    def test_transmit_notifications_do_not_prove_external_bus_reception(self):
        _, report = self.run_probe(frames=[types.SimpleNamespace(id=3, flags=4), types.SimpleNamespace(id=4, flags=8)])
        self.assertEqual(report['status'], 'WARN')
        self.assertEqual(report['rx_frames'], 0)

    def test_cleanup_failure_is_reported(self):
        _, report = self.run_probe(lambda a: setattr(a, 'fail', 'close'))
        self.assertEqual(report['status'], 'FAIL')
        self.assertIn('cleanup_errors', report)

    def test_source_has_no_transmit_path_or_controller_import(self):
        tree = ast.parse((ROOT/'docking/device_check.py').read_text())
        banned = {'write', 'writeWait', 'writeSync', 'Can', 'Controller', 'Frame', 'templates'}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                fn = node.func
                name = fn.attr if isinstance(fn, ast.Attribute) else fn.id if isinstance(fn, ast.Name) else None
                self.assertNotIn(name, banned)


class RsDouble:
    stream = types.SimpleNamespace(gyro='gyro', accel='accel', infrared='infrared', color='color', depth='depth')
    format = types.SimpleNamespace(motion_xyz32f='motion', y8='y8', bgr8='bgr8', z16='z16')
    camera_info = types.SimpleNamespace(serial_number='serial')

    def __init__(self, clock, stop):
        self.clock, self.stop = clock, stop
        self.serials, self.pipes, self.requests = ['D435i-test'], [], []
        self.fail_start, self.missing, self.frozen, self.invalid = set(), set(), set(), set()
        self.callback = None
        self.last_video = self.last_motion = 0.
        stop.tick = self.tick

    def context(self):
        return types.SimpleNamespace(query_devices=lambda: [types.SimpleNamespace(get_info=lambda _, s=s: s) for s in self.serials])

    def config(self):
        api = self
        class Config:
            def __init__(self):
                self.streams, self.serial = [], None
            def enable_device(self, serial):
                self.serial = serial
            def enable_stream(self, *args):
                self.streams.append(args)
        cfg = Config()
        api.requests.append(cfg)
        return cfg

    def frame(self, kind):
        stamp = 1. if kind in self.frozen else self.clock.now
        xyz = types.SimpleNamespace(x=float('nan') if kind in self.invalid else 0., y=0., z=0.)
        f = types.SimpleNamespace(get_timestamp=lambda: stamp*1000., get_frame_timestamp_domain=lambda: 'hardware_clock',
                                  get_profile=lambda: types.SimpleNamespace(stream_type=lambda: kind),
                                  get_motion_data=lambda: xyz, is_motion_frame=lambda: True)
        f.as_motion_frame = lambda: f
        return f

    def tick(self):
        if self.callback and self.clock.now-self.last_motion >= .005:
            self.last_motion = self.clock.now
            for kind in ('gyro', 'accel'):
                if kind not in self.missing:
                    self.callback(self.frame(kind))

    def pipeline(self):
        api = self
        class Pipeline:
            def __init__(self):
                self.stopped = False
            def start(self, cfg, callback=None):
                self.kind = 'imu' if callback else 'camera'
                if self.kind in api.fail_start:
                    raise RuntimeError(self.kind+'_start_failed')
                if callback:
                    api.callback = callback
                return object()
            def poll_for_frames(self):
                if api.clock.now-api.last_video < 1/30.:
                    return None
                api.last_video = api.clock.now
                def frame(kind):
                    return None if kind in api.missing else api.frame(kind)
                return types.SimpleNamespace(get_infrared_frame=lambda _: frame('image'),
                    get_color_frame=lambda: frame('image'), get_depth_frame=lambda: frame('depth'))
            def stop(self):
                self.stopped = True
        pipe = Pipeline()
        api.pipes.append(pipe)
        return pipe


class CameraProbeTests(unittest.TestCase):
    def run_probe(self, change=None, cfg=None):
        clock = Clock()
        stop = Stop(clock)
        rs = RsDouble(clock, stop)
        if change:
            change(rs)
        with patch.object(dc.time, 'monotonic', clock.time):
            report = dc.probe_realsense(cfg or config()['sensor'], 1., stop, rs)
        return rs, report

    def test_camera_depth_gyro_accel_received_on_same_serial(self):
        rs, report = self.run_probe()
        self.assertEqual(report['camera']['status'], 'PASS')
        self.assertEqual(report['imu']['status'], 'PASS')
        self.assertEqual([p.kind for p in rs.pipes], ['imu', 'camera'])
        self.assertTrue(all(p.stopped for p in rs.pipes))
        self.assertTrue(all(c.serial == 'D435i-test' for c in rs.requests))
        self.assertGreater(report['imu']['streams']['gyro']['samples'], 100)

    def test_imu_failure_does_not_hide_working_camera(self):
        _, report = self.run_probe(lambda r: r.fail_start.add('imu'))
        self.assertEqual(report['imu']['status'], 'FAIL')
        self.assertEqual(report['camera']['status'], 'PASS')

    def test_camera_failure_does_not_hide_imu_reception(self):
        rs, report = self.run_probe(lambda r: r.fail_start.add('camera'))
        self.assertEqual(report['camera']['status'], 'FAIL')
        self.assertIn(report['imu']['status'], ('PASS', 'WARN'))
        self.assertGreater(report['imu']['streams']['gyro']['samples'], 2)
        self.assertTrue(rs.pipes[0].stopped)

    def test_missing_each_required_stream_is_failure(self):
        for stream, section in (('image', 'camera'), ('depth', 'camera'), ('gyro', 'imu'), ('accel', 'imu')):
            _, report = self.run_probe(lambda r: r.missing.add(stream))
            self.assertEqual(report[section]['status'], 'FAIL')

    def test_color_without_depth_does_not_require_depth(self):
        cfg = config()['sensor']
        cfg['camera'].update(stream='color', depth_enabled=False)
        rs, report = self.run_probe(lambda r: r.missing.add('depth'), cfg)
        self.assertEqual(report['camera']['status'], 'PASS')
        self.assertNotIn('depth', report['camera']['streams'])
        self.assertIn('color', [s[0] for s in rs.requests[1].streams])

    def test_repeated_and_nan_motion_samples_fail(self):
        for field in ('frozen', 'invalid'):
            _, report = self.run_probe(lambda r: getattr(r, field).add('gyro'))
            self.assertEqual(report['imu']['status'], 'FAIL')

    def test_multiple_devices_require_serial_and_missing_device_fails(self):
        for serials in ([], ['one', 'two']):
            rs, report = self.run_probe(lambda r: setattr(r, 'serials', serials))
            self.assertEqual(report['camera']['status'], 'FAIL')
            self.assertEqual(report['imu']['status'], 'FAIL')
            self.assertFalse(rs.pipes)
        cfg = config()['sensor']
        cfg['camera']['serial'] = 'two'
        rs, report = self.run_probe(lambda r: setattr(r, 'serials', ['one', 'two']), cfg)
        self.assertEqual(report['camera']['serial'], 'two')
        self.assertTrue(all(c.serial == 'two' for c in rs.requests))


class CheckCommandTests(unittest.TestCase):
    def test_real_spawn_missing_sdks_and_timeout_report_all_devices(self):
        # Fake SDK files are visible in spawned interpreters. No hardware is
        # accessed even if an SDK happens to be installed on the test host.
        with tempfile.TemporaryDirectory() as folder:
            stub = pathlib.Path(folder)
            (stub/'canlib.py').write_text('raise ImportError("test_can_sdk_missing")\n')
            env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONPATH=folder)
            for platform, rs_source in (('nano', 'raise ImportError("test_rs_sdk_missing")\n'),
                                        ('vm', 'import time\ntime.sleep(60)\n')):
                (stub/'pyrealsense2.py').write_text(rs_source)
                output = stub/(platform+'.json')
                proc = subprocess.run([sys.executable, '-m', 'docking', 'check-devices', '--platform', platform,
                                       '--seconds', '1', '--timeout', '2', '--output', str(output)],
                                      cwd=str(ROOT), env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=12)
                self.assertEqual(proc.returncode, 2, proc.stderr.decode())
                report = json.loads(output.read_text())
                self.assertEqual(set(report['checks']), {'camera', 'imu', 'can'})
                self.assertFalse(report['command_frames_sent'])
                self.assertIn('test_can_sdk_missing', report['checks']['can']['reason'])
                self.assertEqual(report['checks']['camera']['status'], 'FAIL')
                if platform == 'vm':
                    self.assertEqual(report['checks']['imu']['reason'], 'device_probe_timeout')

    def test_platform_defaults_and_return_codes(self):
        for platform in ('nano', 'vm'):
            with patch('sys.argv', ['docking', 'check-devices', '--platform', platform]), \
                 patch.object(dc, 'run', return_value=1) as run:
                self.assertEqual(main(), 1)
                self.assertEqual(run.call_args[0][0].config, default_config('drive', platform))

    def test_custom_config_and_channel_options(self):
        with patch('sys.argv', ['docking', 'check-devices', '--config', 'custom.json', '--can-channel', '2', '--bitrate', '250000']), \
             patch.object(dc, 'run', return_value=0) as run:
            self.assertEqual(main(), 0)
            args = run.call_args[0][0]
            self.assertEqual((args.config, args.can_channel, args.bitrate), ('custom.json', 2, 250000))

    def test_invalid_duration_and_config_open_no_devices(self):
        for seconds in (0, -1, float('nan'), float('inf'), 61):
            with patch.object(dc.mp, 'get_context') as ctx:
                with self.assertRaises(ValueError):
                    dc.check_devices(config(), seconds)
                ctx.assert_not_called()
        for timeout in (0, 1, float('nan'), 121):
            with self.assertRaises(ValueError):
                dc.check_devices(config(), 1., timeout)

    def test_missing_sdk_produces_per_device_failures(self):
        class Connection:
            def send(self, value): self.value = value
            def close(self): self.closed = True
        for kind, names in (('sensor', ('camera', 'imu')), ('can', ('can',))):
            connection = Connection()
            with patch.object(dc, 'probe_realsense', side_effect=ImportError('no SDK')), \
                 patch.object(dc, 'probe_can', side_effect=ImportError('no SDK')), \
                 patch.object(dc.signal, 'signal'):
                dc._worker(kind, {}, 1., None, connection)
            self.assertEqual(set(connection.value), set(names))
            self.assertTrue(all(v['status'] == 'FAIL' for v in connection.value.values()))
            self.assertTrue(connection.closed)

    def test_hung_sensor_does_not_hide_can_result(self):
        clock, done = Clock(), {'can': dc.result('WARN', 'no_traffic')}
        stop = Stop(clock)
        hang = (types.SimpleNamespace(is_alive=lambda: True), types.SimpleNamespace(poll=lambda: False))
        ready = (types.SimpleNamespace(is_alive=lambda: False), types.SimpleNamespace(poll=lambda: True, recv=lambda: done))
        with patch.object(dc.time, 'monotonic', clock.time):
            report = dc.collect_jobs({'sensor': hang, 'can': ready}, .1, stop)
        self.assertEqual(report['can'], done['can'])
        self.assertEqual(report['camera']['reason'], 'device_probe_timeout')
        self.assertEqual(report['imu']['status'], 'FAIL')
        self.assertTrue(stop.is_set())

    def test_report_saved_without_changing_hardware_config(self):
        before = (ROOT/'configs/hardware.json').read_bytes()
        report = dict(status='WARN', checks={n: dc.result('WARN', 'example') for n in ('camera', 'imu', 'can')}, command_frames_sent=False)
        with tempfile.TemporaryDirectory() as folder:
            args = types.SimpleNamespace(config=str(ROOT/'configs/hardware.json'), platform='nano', seconds=1., timeout=None,
                                         can_channel=2, bitrate=250000, output=folder+'/report.json')
            with patch.object(dc, 'check_devices', return_value=report) as check, patch('sys.stdout', new_callable=io.StringIO):
                self.assertEqual(dc.run(args), 1)
            self.assertEqual(check.call_args[0][0]['can']['channel'], 2)
            with open(args.output) as f:
                self.assertFalse(json.load(f)['command_frames_sent'])
        self.assertEqual(before, (ROOT/'configs/hardware.json').read_bytes())

    def test_default_report_saved_in_sibling_work_from_any_directory(self):
        report = dict(status='WARN', checks={n: dc.result('WARN', 'example') for n in ('camera', 'imu', 'can')},
                      command_frames_sent=False)
        with tempfile.TemporaryDirectory() as folder:
            work = pathlib.Path(folder)/'krri'/'work'
            other = pathlib.Path(folder)/'elsewhere'
            other.mkdir()
            args = types.SimpleNamespace(config=str(ROOT/'configs/hardware.json'), platform='nano', seconds=1.,
                                         timeout=None, can_channel=None, bitrate=None, output=None)
            previous = os.getcwd()
            try:
                os.chdir(other)
                with patch.object(dc, 'WORK_ROOT', str(work)), patch.object(dc, 'check_devices', return_value=report), \
                     patch('sys.stdout', new_callable=io.StringIO):
                    self.assertEqual(dc.run(args), 1)
            finally:
                os.chdir(previous)
            saved = list(work.glob('device_check_*.json'))
            self.assertEqual(len(saved), 1)
            self.assertFalse(json.loads(saved[0].read_text())['command_frames_sent'])
            self.assertEqual(list(other.iterdir()), [])

    def test_launchers_forward_correct_platform(self):
        with tempfile.TemporaryDirectory() as folder:
            fake = pathlib.Path(folder)/'fake-python'
            fake.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
            fake.chmod(0o755)
            uname = pathlib.Path(folder)/'uname'
            uname.write_text('#!/bin/sh\nprintf "Linux\\n"\n')
            uname.chmod(0o755)
            env = dict(os.environ, RD_PYTHON=str(fake), PATH=folder+os.pathsep+os.environ['PATH'])
            for platform in ('nano', 'vm'):
                output = subprocess.check_output(['bash', str(ROOT/'scripts'/('run_'+platform+'.sh')), 'check-devices', '--seconds', '3'], env=env)
                self.assertEqual(output.decode().splitlines(), ['-m', 'docking', 'check-devices', '--platform', platform, '--seconds', '3'])


class SampleTests(unittest.TestCase):
    def test_stalled_stream_and_reversed_clock_fail(self):
        samples = dc.Samples()
        samples.add(1., 1., 'clock')
        samples.add(1.1, 1.1, 'clock')
        self.assertEqual(samples.report(3., 2., 30)['status'], 'FAIL')
        samples.add(1.05, 3., 'clock')
        self.assertEqual(samples.report(3., 2., 30)['clock_reversals'], 1)

    def test_clock_domain_change_is_failure(self):
        samples = dc.Samples()
        samples.add(1., 1., 'hardware')
        samples.add(1.1, 1.1, 'system')
        self.assertEqual(samples.report(1.1, .1, 30)['status'], 'FAIL')


if __name__ == '__main__':
    unittest.main()
