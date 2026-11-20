#!/usr/bin/env python3
"""Explicitly start and manage an SSH-independent, logged FSM v4 user service."""
import argparse
import contextlib
import fcntl
import importlib
import json
import os
from pathlib import Path
import runpy
import signal
import stat
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent
DEPTH_CAM = ROOT / 'depth_cam'
PYTHON = ROOT / '.venv-jetson/bin/python'
UNIT = 'jetson-lifter-v4.service'
MONITOR_PORT = 8768
LOCK_PATH = Path.home() / '.cache/lifter-remote/runtime.lock'
CHOICE_PATH = Path.home() / '.local/state/lifter-remote/last-model.json'
ACTIVE_STATES = {'active', 'activating', 'reloading'}


class ManagerError(Exception):
    pass


def _catalog():
    if str(DEPTH_CAM) not in sys.path:
        sys.path.insert(0, str(DEPTH_CAM))
    return importlib.import_module('model_catalog')


def _private_directory(path):
    path.mkdir(parents=True, exist_ok=True, mode=0o700)
    info = path.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.geteuid():
        raise ManagerError('Runtime state directory must belong to the current user.')
    path.chmod(0o700)


@contextlib.contextmanager
def _runtime_lock():
    _private_directory(LOCK_PATH.parent)
    fd = os.open(LOCK_PATH, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        info = os.fstat(fd)
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid()
                or info.st_nlink != 1 or info.st_mode & 0o022):
            raise ManagerError('Unsafe runtime lock file.')
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ManagerError('Another GUI or detached FSM v4 runtime holds the lock.') from exc
        yield
    finally:
        os.close(fd)


def _assert_no_other_runtime():
    own_pid = os.getpid()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdecimal() or int(proc.name) == own_pid:
            continue
        try:
            arguments = (proc / 'cmdline').read_bytes().split(b'\0')
        except (OSError, ProcessLookupError):
            continue
        if any(Path(os.fsdecode(arg)).name in ('main_rec_v4.py', 'main_rec.py')
               for arg in arguments if arg):
            raise ManagerError(f'Existing vehicle runtime detected (PID {proc.name}); stop it first.')


def _unit_state():
    result = subprocess.run(
        ['systemctl', '--user', 'show', UNIT, '--no-pager',
         '--property=ActiveState,SubState,MainPID,Result,LoadState'],
        capture_output=True, text=True, check=False,
    )
    values = {}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition('=')
        if separator and key in ('ActiveState', 'SubState', 'MainPID', 'Result', 'LoadState'):
            values[key] = value
    if result.returncode and not values:
        raise ManagerError('Cannot query the user service manager; a persistent user session is required.')
    return {'unit': UNIT, 'active_state': values.get('ActiveState', 'inactive'),
            'sub_state': values.get('SubState', 'dead'),
            'main_pid': int(values.get('MainPID', '0') or '0'),
            'result': values.get('Result', ''),
            'load_state': values.get('LoadState', 'not-found')}


def _preflight():
    subprocess.run([str(PYTHON), str(ROOT / 'CHECK_DEVICES_JETSON.py'), '--stream'],
                   cwd=ROOT, env=_runtime_env(), check=True)


def _runtime_env():
    environment = os.environ.copy()
    environment.update(PYTHONDONTWRITEBYTECODE='1', YOLO_AUTOINSTALL='false')
    return environment


def _validated_model(name):
    catalog = _catalog()
    selected = catalog.resolve_model(name)
    catalog.validate_pose_model(selected)
    return selected.name


def _save_choice(name, clahe, demo=False):
    _private_directory(CHOICE_PATH.parent)
    fd, temporary = tempfile.mkstemp(prefix='.model-choice-', dir=CHOICE_PATH.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as handle:
            json.dump({'model': name, 'clahe': bool(clahe), 'demo': bool(demo)}, handle)
            handle.write('\n')
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, CHOICE_PATH)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def _choose_model(clahe_override=None, demo_override=None):
    catalog = _catalog()
    selected = catalog.choose_model(allow_demo=True)
    if selected is None:
        return None
    clahe = catalog.configured_clahe() if clahe_override is None else clahe_override
    demo = catalog.configured_demo_mode() if demo_override is None else demo_override
    name = _validated_model(selected.name)
    _save_choice(name, clahe, demo)
    return name, bool(clahe), bool(demo)


def _service_command(model, clahe, demo=False):
    return [
        'systemd-run', '--user', '--unit=' + UNIT, '--collect', '--no-ask-password',
        '--property=Type=exec', '--property=Restart=no',
        '--property=KillSignal=SIGINT', '--property=KillMode=mixed',
        '--property=TimeoutStopSec=15s', '--property=SendSIGHUP=no',
        '--property=WorkingDirectory=' + str(DEPTH_CAM),
        '--property=StandardInput=null', '--property=StandardOutput=journal',
        '--property=StandardError=journal',
        '--setenv=PYTHONDONTWRITEBYTECODE=1', '--setenv=YOLO_AUTOINSTALL=false',
        str(PYTHON), str(ROOT / 'REMOTE_JETSON.py'), 'run', '--model', model,
        '--clahe' if clahe else '--no-clahe',
        '--demo' if demo else '--no-demo',
    ]


def start(args):
    state = _unit_state()
    if state['active_state'] in ACTIVE_STATES:
        print(json.dumps(dict(state, already_running=True)))
        return 0
    if state['active_state'] == 'deactivating':
        raise ManagerError('The previous runtime is stopping; wait for shutdown to finish.')
    if not sys.stdin.isatty():
        raise ManagerError('Start from an interactive terminal so you can enter RUN.')
    with _runtime_lock():
        _assert_no_other_runtime()
    # Validate explicitly supplied models before touching camera streams.
    model = _validated_model(args.model) if args.model else None
    _preflight()
    print('이 실행은 젯슨에서 카메라·CAN 자동 제어를 시작하며 SSH 연결이 끊겨도 계속됩니다.', flush=True)
    response = input('실제 제어를 시작하려면 RUN 입력: ')
    if response != 'RUN':
        print(json.dumps({'cancelled': True, 'started': False}))
        return 0
    if model is None:
        choice = _choose_model(args.clahe, args.demo)
        if choice is None:
            print(json.dumps({'cancelled': True, 'started': False}))
            return 0
        model, clahe, demo = choice
    else:
        clahe = bool(args.clahe)
        demo = bool(args.demo)
        _save_choice(model, clahe, demo)
    # The service repeats the exclusive guard and preflight before CAN startup.
    # Do not hold the probe lock across handoff to the service process.
    subprocess.run(_service_command(model, clahe, demo), check=True, env=_runtime_env())
    print(json.dumps({'unit': UNIT, 'started': True, 'model': model,
                      'clahe': clahe, 'demo': demo, 'monitor_port': MONITOR_PORT}))
    return 0


def _cleanup_control():
    control = sys.modules.get('calib.control')
    if control is None:
        return
    try:
        control.set_runtime_stop_latched(True)
    except Exception as exc:
        print(f'[REMOTE cleanup] stop latch failed: {type(exc).__name__}', file=sys.stderr)
    try:
        control.can_close()
    except Exception as exc:
        print(f'[REMOTE cleanup] CAN close failed: {type(exc).__name__}', file=sys.stderr)


def run(args):
    with _runtime_lock():
        _assert_no_other_runtime()
        model = _validated_model(args.model)
        old_signals = {number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)}
        old_argv = sys.argv[:]
        old_cwd = Path.cwd()
        try:
            def interrupt(_number, _frame):
                raise KeyboardInterrupt
            for number in old_signals:
                signal.signal(number, interrupt)
            for variable in ('DISPLAY', 'XAUTHORITY'):
                os.environ.pop(variable, None)
            os.environ.update(PYTHONDONTWRITEBYTECODE='1', YOLO_AUTOINSTALL='false')
            _preflight()
            os.chdir(DEPTH_CAM)
            sys.argv = [str(DEPTH_CAM / 'main_rec_v4.py'), '--no-display',
                        '--monitor-port', str(MONITOR_PORT),
                        '--demo' if args.demo else '--no-demo',
                        '--model', model, '--clahe' if args.clahe else '--no-clahe']
            runpy.run_path(str(DEPTH_CAM / 'main_rec_v4.py'), run_name='__main__')
        except KeyboardInterrupt:
            print('[REMOTE] Stop requested; closing vehicle control.', flush=True)
            return 0
        finally:
            _cleanup_control()
            sys.argv = old_argv
            os.chdir(old_cwd)
            for number, handler in old_signals.items():
                signal.signal(number, handler)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    commands = parser.add_subparsers(dest='command', required=True)
    for action in ('choose', 'start', 'run'):
        command = commands.add_parser(action, allow_abbrev=False)
        if action != 'choose':
            command.add_argument('--model', required=(action == 'run'))
        command.add_argument('--clahe', action=argparse.BooleanOptionalAction, default=None)
        command.add_argument('--demo', action=argparse.BooleanOptionalAction, default=None)
    commands.add_parser('status')
    commands.add_parser('stop')
    logs = commands.add_parser('logs')
    logs.add_argument('--follow', '-f', action='store_true')
    args = parser.parse_args(argv)
    try:
        if args.command == 'start':
            return start(args)
        if args.command == 'run':
            return run(args)
        if args.command == 'choose':
            choice = _choose_model(args.clahe, args.demo)
            print(json.dumps({'cancelled': True} if choice is None
                             else {'model': choice[0], 'clahe': choice[1], 'demo': choice[2]}))
            return 0
        if args.command == 'status':
            state = _unit_state()
            print(json.dumps(state))
            return 0 if state['active_state'] in ACTIVE_STATES else 3
        if args.command == 'stop':
            return subprocess.run(['systemctl', '--user', 'stop', UNIT], check=False).returncode
        command = ['journalctl', '--user', '--unit=' + UNIT, '--no-pager', '--lines=80']
        if args.follow:
            command.append('--follow')
        return subprocess.run(command, check=False).returncode
    except ManagerError as exc:
        print(f'[REMOTE] {exc}', file=sys.stderr)
        return 2
    except subprocess.CalledProcessError as exc:
        print(f'[REMOTE] Required command failed (exit {exc.returncode}); runtime was not restarted.', file=sys.stderr)
        return exc.returncode or 2
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    sys.exit(main())
