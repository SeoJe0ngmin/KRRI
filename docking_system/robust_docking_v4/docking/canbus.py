"""Restricted Kvaser CAN transport, in a separate watchdog process.

Protocol source: apriltag_v5/src/models/kwu/control.py and control/frames.py.
Reimplemented for Python 3.6; no fork/lift/reach commands are exported.
Software STOP is not a hardware emergency-stop or power-loss guarantee.
"""
import multiprocessing as mp
import time
import signal
import queue
from .canhealth import Diagnostics, validate_diagnostics

NAMES = ('stop', 'forward', 'forward_slow', 'backward', 'rotate_left_slow', 'rotate_right_slow')


def templates(cfg):
    n = 127
    out = {key: [n]*8 for key in NAMES}
    for key in ('forward', 'forward_slow', 'backward', 'rotate'):
        d = cfg['deflections'][key]
        if not isinstance(d, int) or not 1 <= d <= 100:
            raise ValueError('invalid joystick deflection')
    out['forward'][2] -= cfg['deflections']['forward']
    out['forward_slow'][2] -= cfg['deflections']['forward_slow']
    out['backward'][2] += cfg['deflections']['backward']
    out['rotate_left_slow'][1] += cfg['deflections']['rotate']
    out['rotate_right_slow'][1] -= cfg['deflections']['rotate']
    for name, frame in out.items():
        if len(frame) != 8 or any(frame[i] != n for i in (0, 3, 4, 5, 6, 7)):
            raise ValueError('non-drive CAN byte changed')
        if frame[1] != n and frame[2] != n:
            raise ValueError('combined translation/rotation forbidden')
    return out


def effective(index, deadline, latched, now):
    if latched or index not in range(len(NAMES)):
        return 0, bool(latched or index not in range(len(NAMES)))
    if index != 0 and now >= deadline:
        return 0, True
    return index, False


def _worker(cfg, shared, abort, close, ready, health, reports=None, faults=None):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    if hasattr(signal, 'SIGHUP'):
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
    ch = diagnostic = None
    last_report = 0.
    def report(value):
        if reports is not None:
            try:
                reports.put_nowait(value)
            except queue.Full:
                pass
    try:
        from canlib import canlib, Frame
        frames = templates(cfg)
        rates = {125000: canlib.Bitrate.BITRATE_125K, 250000: canlib.Bitrate.BITRATE_250K,
                 500000: canlib.Bitrate.BITRATE_500K, 1000000: canlib.Bitrate.BITRATE_1M}
        if cfg['bitrate'] not in rates:
            raise ValueError('unsupported bitrate')
        ch = canlib.openChannel(cfg['channel'])
        ch.setBusParams(rates[cfg['bitrate']])
        if cfg.get('diagnostics', {}).get('enabled'):
            diagnostic = Diagnostics(ch, canlib, cfg['diagnostics'])
        ch.busOn()
        write = diagnostic.write if diagnostic else ch.write
        counter = 10
        def tx_movement(index):
            # Check latch at the actual write boundary too.
            if abort.is_set():
                index = 0
            write(Frame(id_=0x1E3, data=frames[NAMES[index]], flags=0))
        def tx_control():
            nonlocal counter
            counter = (counter+1) & 15
            write(Frame(id_=0x2E3, data=[0x42, 0, 0, 0x0A, counter, 0x40, 0x69, 0x93], flags=0))
        for _ in range(5):
            tx_control()
            tx_movement(0)
            time.sleep(.005)
        ch.writeSync(200)
        if diagnostic:
            report(diagnostic.poll())
        with health.get_lock():
            health[0], health[1] = time.monotonic(), 0
        ready.put({'ok': True})
        next_control = next_movement = next_heartbeat = time.monotonic()
        last, burst = 0, 0
        while not close.is_set():
            now = time.monotonic()
            with shared.get_lock():
                index, deadline = int(shared[0]), shared[1]
            index, latch = effective(index, deadline, abort.is_set(), now)
            if latch:
                abort.set()
            if index != last:
                if index == 0 and diagnostic:
                    diagnostic.stop_transition()
                burst = 5 if last == 0 and index != 0 else 1
                next_movement = next_control = now
                last = index
            if now >= next_control:
                tx_control()
                next_control = now+.005
            if now >= next_movement:
                tx_movement(index)
                burst = max(0, burst-1)
                next_movement = now+(.005 if burst else .010)
            if now >= next_heartbeat:
                write(Frame(id_=0x764, data=[0], flags=0))
                next_heartbeat = now+.2
            with health.get_lock():
                health[0], health[1] = now, index
            if diagnostic:
                stats = diagnostic.poll()
                if now-last_report >= .1:
                    report(stats)
                    last_report = now
            close.wait(.002)
    except Exception as e:
        abort.set()
        failure = dict(diagnostic.report if diagnostic else {}, fault=str(e))
        report(failure)
        if faults is not None:
            try:
                faults.put_nowait(failure)  # Separate terminal slot cannot fill with routine reports.
            except queue.Full:
                pass
        try:
            ready.put_nowait({'ok': False, 'error': str(e)})
        except Exception:
            pass
    finally:
        abort.set()
        if ch is not None:
            try:
                if diagnostic:
                    diagnostic.stop_transition()
            except Exception:
                pass
            try:
                for _ in range(3):
                    ch.write(Frame(id_=0x1E3, data=[127]*8, flags=0))
                    time.sleep(.01)
                ch.writeSync(200)
            except Exception:
                pass
            try:
                ch.busOff()
                ch.close()
            except Exception:
                pass


class Can:
    def __init__(self, cfg, lease_s):
        templates(cfg)
        validate_diagnostics(cfg, live=True)
        self.lease_s = lease_s
        ctx = mp.get_context('spawn')
        self.shared = ctx.Array('d', [0., 0.])
        self.health = ctx.Array('d', [0., 0.])
        self.abort_event, self.close_event = ctx.Event(), ctx.Event()
        ready = ctx.Queue(2)
        self.reports = ctx.Queue(4)
        self.faults = ctx.Queue(1)
        self.diagnostics = {}
        self.process = ctx.Process(target=_worker, args=(cfg, self.shared, self.abort_event, self.close_event, ready, self.health, self.reports, self.faults), daemon=True)
        self.process.start()
        try:
            status = ready.get(timeout=5)
            if not status.get('ok'):
                raise RuntimeError(status.get('error', 'CAN init failed'))
        except Exception:
            self.close()
            raise

    def set(self, command):
        if command not in NAMES:
            self.abort()
            raise ValueError('command not whitelisted')
        if self.abort_event.is_set() or not self.process.is_alive():
            if command != 'stop':
                raise RuntimeError('CAN abort is latched; restart required')
            return
        with self.shared.get_lock():
            self.shared[0] = NAMES.index(command)
            self.shared[1] = time.monotonic()+self.lease_s

    def status(self):
        while True:
            try:
                self.diagnostics = self.reports.get_nowait()
            except queue.Empty:
                break
        try:
            failure = self.faults.get_nowait()
            self.diagnostics = failure
            self.terminal_fault = failure
        except queue.Empty:
            pass
        if getattr(self, 'terminal_fault', None):
            self.diagnostics = self.terminal_fault
        with self.health.get_lock():
            stamp, index = self.health[:]
        return {'alive': self.process.is_alive(), 'latched': self.abort_event.is_set(),
                'tx_worker_age_s': time.monotonic()-stamp, 'command': NAMES[int(index)],
                'diagnostics': dict(self.diagnostics)}

    def abort(self):
        self.abort_event.set()

    def close(self):
        self.abort_event.set()
        self.close_event.set()
        if self.process.is_alive():
            self.process.join(timeout=3)
        # Do not forcibly kill a process while it may be issuing neutral frames.
