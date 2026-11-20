import argparse
import importlib.util
import json
import os
import sys
import time

from .configuration import load_config
from .mission import build, load_policy

from .paths import PROJECT_ROOT, WORK_ROOT


def default_config(command, platform):
    if command in ('measure', 'calibrate-mount'):
        name = 'vm_measure.json' if platform == 'vm' else 'nano_measure.json'
    else:
        name = 'hardware_vm.json' if platform == 'vm' else 'hardware.json'
    return os.path.join(PROJECT_ROOT, 'configs', name)


def doctor(platform):
    import platform as host
    print('Target:', platform)
    print('Host:', host.system(), host.release(), host.machine())
    print('Python:', sys.version.split()[0], sys.executable)
    if host.system() != 'Linux':
        print('RealSense/CAN execution target is Linux (VM or Jetson). macOS may run core tests/synthesis.')
    for m in ('numpy', 'cv2', 'pupil_apriltags', 'pyrealsense2', 'canlib'):
        spec = importlib.util.find_spec(m)
        print('%-16s %s' % (m, 'OK (import path found)' if spec else 'MISSING'))
    print('Core synthesis/tests use only the standard library. No devices were opened.')


def write_json(path, data):
    folder = os.path.dirname(path)
    if folder:
        os.makedirs(folder, exist_ok=True)
    with open(path, 'w') as f:
        json.dump(data, f, indent=2, allow_nan=False)


def replay(cfg, table, path):
    from .controller import Controller
    c = Controller(cfg, load_policy(cfg, table))
    counts = {}
    for line in open(path):
        o = json.loads(line)
        result = c.step(o['published'], o)
        counts[result['state']] = counts.get(result['state'], 0)+1
    return {'result': c.status(), 'state_counts': counts, 'can_connected': False}


def main():
    ap = argparse.ArgumentParser(description='Robust docking: Nano sensing, finite reachability, Kvaser CAN')
    sub = ap.add_subparsers(dest='command')
    doc = sub.add_parser('doctor', help='dependency check, no device/CAN access')
    doc.add_argument('--platform', choices=('nano', 'vm'), default='nano')
    devices = sub.add_parser('check-devices', help='camera + IMU reception and silent CAN check; no transmissions')
    devices.add_argument('--platform', choices=('nano', 'vm'), default='nano')
    devices.add_argument('--config', help='hardware profile; missing driving calibration is allowed')
    devices.add_argument('--seconds', type=float, default=5., help='receive window, 1..60 seconds')
    devices.add_argument('--timeout', type=float, help='total SDK deadline; default seconds + 20')
    devices.add_argument('--can-channel', type=int, help='override CAN channel for this check only')
    devices.add_argument('--bitrate', type=int, help='override CAN bitrate for this check only')
    devices.add_argument('--output', help='new JSON report path')
    mount = sub.add_parser('calibrate-mount', help='stationary reference-pose camera mounting measurement; no CAN')
    mount.add_argument('--platform', choices=('nano', 'vm'), default='nano')
    mount.add_argument('--config', help='sensor profile; default platform measurement profile')
    mount.add_argument('--seconds', type=float, default=60., help='measurement interval, 30..180; adds 15s startup allowance')
    mount.add_argument('--view', action='store_true')
    mount.add_argument('--port', type=int, default=8765)
    apply = mount.add_mutually_exclusive_group()
    apply.add_argument('--apply', action='store_true', help='write successful result to shared common config')
    apply.add_argument('--apply-versions', action='store_true', help='write successful result to shared config for v1/v2/v3/v4')
    mount.add_argument('--camera-lateral-m', type=float, help='separately measured camera offset from body centreline, left positive')
    mount.add_argument('--aligned-reference', action='store_true', help='confirm level vehicle normal to upright, level tag plane')
    syn = sub.add_parser('synthesize', help='offline finite observation-feedback policy synthesis')
    syn.add_argument('--config', required=True, help='explicit prepared hardware configuration')
    syn.add_argument('--output', default=os.path.join(WORK_ROOT, 'policy.json'))
    rp = sub.add_parser('replay', help='replay observation JSONL through shadow controller')
    rp.add_argument('log')
    rp.add_argument('--config', required=True)
    rp.add_argument('--policy', required=True)
    prep = sub.add_parser('prepare', help='combine host profile, site geometry and trial goal; no devices')
    prep.add_argument('--config', required=True)
    prep.add_argument('--site', required=True)
    prep.add_argument('--task', choices=('wall_stop', 'cone_entry'), required=True)
    prep.add_argument('--output', required=True)
    check = sub.add_parser('check-config', help='list missing/invalid driving inputs; no devices')
    check.add_argument('--config', required=True)
    for name in ('measure', 'shadow', 'drive'):
        p = sub.add_parser(name)
        p.add_argument('--platform', choices=('nano', 'vm'), default='nano', help='execution profile; same controller on both hosts')
        p.add_argument('--config', help='explicit config overrides the platform default')
        p.add_argument('--policy')
        p.add_argument('--output')
        p.add_argument('--seconds', type=float, default=0, help='0 runs until Ctrl+C')
        p.add_argument('--view', action='store_true', help='read-only localhost dashboard for SSH forwarding')
        p.add_argument('--port', type=int, default=8765)
        p.add_argument('--execute', action='store_true', help='only drive may open CAN; Enter still required')
        p.add_argument('--no-tag', action='store_true', help='measure-only camera/odometry benchmark')
        p.add_argument('--no-odom', action='store_true', help='measure-only camera/tag benchmark')
        p.add_argument('--no-depth', action='store_true', help='measure-only color/tag benchmark; disables depth and odometry')
    args = ap.parse_args()
    if args.command is None:
        ap.print_help()
        return 0
    try:
        if args.command == 'doctor':
            doctor(args.platform)
        elif args.command == 'check-devices':
            from .device_check import run as check_devices
            if args.config is None:
                args.config = default_config(args.command, args.platform)
            return check_devices(args)
        elif args.command == 'calibrate-mount':
            from .mount_calibration import run as calibrate_mount
            if args.config is None:
                args.config = default_config(args.command, args.platform)
            return calibrate_mount(args)
        elif args.command in ('prepare', 'check-config'):
            from .profiles import prepare, readiness
            cfg = load_config(args.config)
            if args.command == 'prepare':
                cfg = prepare(cfg, args.site, args.task)
                write_json(args.output, cfg)
            print(json.dumps(readiness(cfg), ensure_ascii=False, indent=2))
        elif args.command == 'synthesize':
            cfg = load_config(args.config)
            t0 = time.monotonic()
            table = build(cfg)
            write_json(args.output, table)
            print('winning %d / %d nodes, %d layers, %.2fs -> %s' %
                  (table['winning_nodes'], table['total_nodes'], table['layers'], time.monotonic()-t0, args.output))
        elif args.command == 'replay':
            print(json.dumps(replay(load_config(args.config), json.load(open(args.policy)), args.log), indent=2))
        else:
            from .runtime import run
            if args.config is None:
                args.config = default_config(args.command, args.platform)
            run(args)
        return 0
    except KeyboardInterrupt:
        print('Interrupted; device checks closed without sending commands.', file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, OSError, ImportError, KeyError) as e:
        print('ERROR: '+str(e), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
