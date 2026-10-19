"""Restricted Kvaser CAN transport, in a separate watchdog process.

Protocol source: apriltag_v4/src/models/kwu/control.py and control/frames.py.
No fork/lift/reach commands are exported. Software STOP is not a hardware
emergency-stop or power-loss guarantee.

2026-10-18 (user decision): transmit only, like v4 that drove the truck on
2026-10-01. The receive queue is never read and TXACK is off. What still
stops the vehicle: producer lease expiry (latched), a write failure, the worker
process dying, SIGTERM, or the parent process dying. Bus status and error
counters are read every status_period_s for the log only (v4: every 5 s).
"""
import multiprocessing as mp
import time
import signal
import queue
from .canhealth import BusStatus, guard, validate_diagnostics

# Frozen decimal payloads from apriltag_v4 control/frames.py + control.py.
PAYLOADS = {
 'stop': (127,127,127,127,127,127,127,127),
 'forward': (127,127,67,127,127,127,127,127),
 'forward_slow': (127,127,97,127,127,127,127,127),
 'backward': (127,127,187,127,127,127,127,127),
 'rotate_left_slow': (127,157,127,127,127,127,127,127),
 'rotate_right_slow': (127,97,127,127,127,127,127,127),
 'rotate_left_fine': (127,147,127,127,127,127,127,127),
 'rotate_right_fine': (127,107,127,127,127,127,127,127),
}
NAMES = tuple(PAYLOADS)
# Producer waits for the mailbox lock like v4's `with lock:` did (2026-10-19 user decision). The worker holds it
# for microseconds, so only a dead or stuck worker makes this wait run out; the bound equals the lease (1 s) so a
# supervisor blocked here is caught by its own loop deadline instead of hanging on a lock a dead process left behind.
SET_LOCK_WAIT_S = 1.0

def templates(cfg):
    if cfg.get('bitrate') != 500000 or cfg.get('channel') != 0:
        raise ValueError('v4 CAN profile requires channel 0 / 500 kbit/s')
    return {k:list(v) for k,v in PAYLOADS.items()}


def effective(index, deadline, latched, now, expires_at=0.):
    if latched or index not in range(len(NAMES)):
        return 0, bool(latched or index not in range(len(NAMES)))
    if expires_at and expires_at<=deadline and now>=expires_at:
        return 0, False  # Scheduled pulse completion; producer lease remains independently enforced.
    if index != 0 and now >= deadline:
        return 0, True
    return index, False


def _worker(cfg, shared, abort, close, ready, health, reports=None, faults=None):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    if hasattr(signal, 'SIGHUP'):
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
    # SIGTERM (systemd stop, pkill): leave the loop so the neutral burst and busOff still run.
    signal.signal(signal.SIGTERM, lambda *_: close.set())
    parent = mp.parent_process()
    ch = status = None
    stage = 'import_canlib'
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
        stage = 'openChannel'
        ch = canlib.openChannel(cfg['channel'])
        stage = 'setBusParams'
        ch.setBusParams(rates[cfg['bitrate']])
        if cfg.get('diagnostics', {}).get('enabled'):
            status = BusStatus(ch, canlib, cfg['diagnostics'])
        stage = 'busOn'
        ch.busOn()
        def write(frame):
            guard(frame)
            ch.write(frame)
            if status:
                status.wrote()
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
        stage = 'startup_neutral_burst'
        for _ in range(5):
            tx_control()
            write(Frame(id_=0x764, data=[0], flags=0))
            tx_movement(0)
            time.sleep(.005)
        stage = 'startup_writeSync_200ms'
        ch.writeSync(200)
        if status:
            snapshot = status.poll(time.monotonic())
            if snapshot:
                report(snapshot)
        with health.get_lock():
            health[0], health[1] = time.monotonic(), 0
        stage = 'running'
        ready.put({'ok': True})
        next_control = next_movement = time.monotonic()
        next_heartbeat = next_control+.2
        last, burst = 0, 0
        index, deadline, expires_at = 0, 0., 0.
        while not close.is_set():
            now = time.monotonic()
            if parent is not None and not parent.is_alive():
                break   # Orphaned worker: neutral burst, busOff, exit.
            # A suspended producer holding this lock must not stall STOP.
            if shared.get_lock().acquire(False):
                try:
                    index, deadline = int(shared[0]), shared[1]
                    expires_at = shared[2] if len(shared)>2 else 0.
                finally:shared.get_lock().release()
            index, latch = effective(index, deadline, abort.is_set(), now, expires_at)
            if latch:
                abort.set()
            if index != last:
                burst = 5 if last == 0 and index != 0 else 1
                next_movement = next_control = now
                last = index
            if burst and now >= next_movement:
                # v4 command burst: control -> heartbeat -> movement, five
                # bundles at entry, one on other command transitions.
                tx_control()
                write(Frame(id_=0x764, data=[0], flags=0))
                tx_movement(index)
                burst -= 1
                next_control = now+.005
                next_movement = now+(.005 if burst else .010)
                if not burst:
                    ch.writeSync(100)  # v4 command-change transmission completion.
                    now = time.monotonic()
                    next_control, next_movement = now+.005, now+.010
            else:
                if now >= next_control:
                    tx_control()
                    next_control = now+.005
                if now >= next_movement:
                    tx_movement(index)
                    next_movement = now+.010
                if now >= next_heartbeat:
                    write(Frame(id_=0x764, data=[0], flags=0))
                    next_heartbeat = now+.2
            if health.get_lock().acquire(False):
                try:health[0], health[1] = now, index
                finally:health.get_lock().release()
            if status:
                snapshot = status.poll(now)
                if snapshot:
                    report(snapshot)
            close.wait(.002)
    except Exception as e:
        abort.set()
        # Do not delay neutral for diagnostic reads or queue/log operations.
        if ch is not None:
            try:
                ch.write(Frame(id_=0x1E3, data=[127]*8, flags=0))
            except Exception as stop_error:
                if status:status.report['immediate_stop_error']=str(stop_error)
        failure = dict(status.report if status else {}, fault=str(e), stage=stage)
        report(failure)
        if faults is not None:
            try:
                faults.put_nowait(failure)  # Separate terminal slot cannot fill with routine reports.
            except queue.Full:
                pass
        try:
            ready.put_nowait({'ok': False, 'error': stage+': '+str(e)})
        except Exception:
            pass
    finally:
        abort.set()
        if ch is not None:
            try:
                for _ in range(3):
                    counter = (counter+1) & 15
                    ch.write(Frame(id_=0x2E3, data=[0x42,0,0,0x0A,counter,0x40,0x69,0x93], flags=0))
                    ch.write(Frame(id_=0x764, data=[0], flags=0))
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
        # Probe the native SDK before spawning: python-canlib may exit during
        # import when this library is missing, leaving the ready queue empty.
        import ctypes
        try:ctypes.CDLL('libcanlib.so')
        except OSError as exc:
            raise RuntimeError('Kvaser CANlib 시스템 라이브러리를 로드할 수 없습니다: '+str(exc)+
                '. Python canlib 설치만으로는 부족합니다. 시스템 SDK 설치/검색 경로를 확인하세요. CAN 미시작.') from exc
        templates(cfg)
        validate_diagnostics(cfg, live=True)
        self.lease_s = lease_s
        self.last_requested_command = 'stop'
        ctx = mp.get_context('spawn')
        self.shared = ctx.Array('d', [0., 0., 0.])
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

    def set(self, command, expires_at=None):
        if command not in NAMES:
            self.abort()
            raise ValueError('command not whitelisted')
        if self.abort_event.is_set() or not self.process.is_alive():
            if command != 'stop':
                raise RuntimeError('CAN abort is latched; restart required')
            return
        # Wait for the lock (v4 parity). 5 ms (2026-10-18) could still lose to a preempted worker; now up to the
        # lease, so this only fails when the worker died holding the lock.
        if not self.shared.get_lock().acquire(True, SET_LOCK_WAIT_S):
            # A moving-to-STOP transition that still cannot publish aborts; a
            # repeated STOP need not.
            if command == 'stop' and self.last_requested_command != 'stop':self.abort()
            return  # Existing lease still expires; never extend on contention.
        try:
            self.shared[0] = NAMES.index(command)
            deadline = time.monotonic()+self.lease_s
            self.shared[1] = deadline
            self.shared[2] = 0. if expires_at is None else expires_at
            self.last_requested_command = command
        finally:self.shared.get_lock().release()

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
        if self.health.get_lock().acquire(False):
            try:self.last_health = self.health[:]
            finally:self.health.get_lock().release()
        stamp, index = getattr(self, 'last_health', [0., 0.])
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
