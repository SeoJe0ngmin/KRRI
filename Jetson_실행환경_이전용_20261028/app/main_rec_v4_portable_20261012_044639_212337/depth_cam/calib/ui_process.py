"""Isolated UI owner; shared RGB slots, bounded metadata, no live control objects.

The child is a fresh interpreter, not a fork of a CUDA/CAN/camera process.
Only the parent may run inference, own the FSM, or act on keyboard commands.
"""
from __future__ import annotations

from contextlib import contextmanager
import importlib.abc
import mmap
from multiprocessing.connection import Connection
import os
from pathlib import Path
import pickle
import queue
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

MAX_MESSAGE = 65536


def _send(pipe, value):
    payload = pickle.dumps(value, protocol=4)
    if len(payload) > MAX_MESSAGE:
        raise ValueError('UI metadata exceeds bounded message size')
    pipe.send_bytes(payload)


def _receive(pipe):
    return pickle.loads(pipe.recv_bytes(MAX_MESSAGE))


class UIProcess:
    """Two acknowledged input slots. A slow UI never queues old RGB frames."""

    def __init__(self, spec, fps=20., tracing=False, startup_timeout=15.):
        import numpy as np
        self.spec, self.fps = dict(spec), float(fps)
        self.shape = tuple(spec['frame_shape'])
        if len(self.shape) != 3 or self.shape[2] != 3:
            raise ValueError('Expected BGR RGB-slot shape')
        self.frame_bytes = int(np.prod(self.shape))
        self.storage = tempfile.TemporaryFile(prefix='lifter-ui-rgb-')
        self.storage.truncate(self.frame_bytes * 2)
        self.mapping = mmap.mmap(self.storage.fileno(), self.frame_bytes * 2)
        self.free, self.inflight = [0, 1], {}
        self.serial = 0
        self.statistics = dict(submitted=0, rendered=0, replaced=0, perf_dropped=0)
        self.process = self.pipe = None
        self.closed = False
        self.last_recording = None
        self.last_finish = False
        self.last_poll = []
        a, b = socket.socketpair()
        self.pipe = Connection(a.detach())
        environment = os.environ.copy()
        source = str(Path(__file__).resolve().parents[1])
        environment['PYTHONPATH'] = source + os.pathsep + environment.get('PYTHONPATH', '')
        environment['PYTHONDONTWRITEBYTECODE'] = '1'
        try:
            self.process = subprocess.Popen(
                [sys.executable, '-m', 'calib.ui_process', '--worker', str(b.fileno()),
                 str(self.storage.fileno()), str(os.getpid())],
                env=environment, pass_fds=(b.fileno(), self.storage.fileno()))
            b.close()
            _send(self.pipe, dict(type='configure', spec=self.spec, fps=self.fps, tracing=bool(tracing)))
            deadline = time.monotonic() + startup_timeout
            while time.monotonic() < deadline:
                messages = self.poll()
                for item in messages:
                    if item['type'] == 'ready':
                        self.statistics.update(worker_pid=item['pid'], forbidden_modules=item['forbidden_modules'])
                        return
                    if item['type'] == 'error':
                        raise RuntimeError('UI startup failed: ' + item['error'])
                time.sleep(.005)
            raise TimeoutError('UI process did not become ready')
        except BaseException:
            b.close()
            self.close()
            raise

    def poll(self):
        rows = []
        if self.pipe is None or self.closed:
            return rows
        try:
            for _ in range(64):
                if not self.pipe.poll():
                    break
                item = _receive(self.pipe)
                if item['type'] == 'ack':
                    slot = self.inflight.pop(item['serial'], None)
                    if slot is not None:
                        self.free.append(slot)
                elif item['type'] in ('rendered', 'stopped'):
                    self.statistics.update(item.get('statistics', {}))
                    if 'digest' in item:
                        self.statistics['last_digest'] = item['digest']
                    if item['type'] == 'stopped':
                        self.statistics['stopped_cleanly'] = True
                rows.append(item)
        except (EOFError, OSError):
            if not self.statistics.get('stopped_cleanly'):
                rows.append(dict(type='error', error='UI channel closed unexpectedly'))
        if (self.process is not None and self.process.poll() is not None
                and not self.statistics.get('stopped_cleanly')):
            rows.append(dict(type='error', error=f'UI process exited: {self.process.returncode}'))
        return rows

    def submit(self, packet):
        import numpy as np
        if not self.free:
            return False
        frame = packet['image']
        if frame.shape != self.shape or frame.dtype != np.uint8:
            raise ValueError('UI frame shape/dtype changed')
        slot = self.free.pop()
        self.serial += 1
        view = np.ndarray(self.shape, dtype=np.uint8, buffer=self.mapping, offset=slot*self.frame_bytes)
        np.copyto(view, frame)
        meta = {k: v for k, v in packet.items() if k != 'image'}
        _send(self.pipe, dict(type='frame', slot=slot, serial=self.serial, packet=meta))
        self.inflight[self.serial] = slot
        self.statistics['submitted'] += 1
        return True

    def sync_recording(self, path, finished):
        if path != self.last_recording:
            _send(self.pipe, dict(type='recording', path=path))
            self.last_recording = path
        if finished and not self.last_finish:
            _send(self.pipe, dict(type='finish_recording'))
            self.last_finish = True

    def close(self, on_message=None, timeout=30.):
        if self.closed:
            return
        try:
            if self.pipe is not None and self.process is not None:
                if self.process.poll() is None:
                    try:
                        _send(self.pipe, dict(type='stop'))
                    except (BrokenPipeError, OSError):
                        pass
                    deadline = time.monotonic() + timeout
                    while self.process.poll() is None and time.monotonic() < deadline:
                        for row in self.poll():
                            if on_message is not None:
                                on_message(row)
                        time.sleep(.005)
                for row in self.poll():
                    if on_message is not None:
                        on_message(row)
                if self.process.poll() is None:
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=2.)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait()
                    raise RuntimeError('UI process did not drain on shutdown')
                self.process.wait()
        finally:
            self.closed = True
            if self.pipe is not None:
                self.pipe.close()
            self.mapping.close()
            self.storage.close()


class _TraceRelay:
    """Keep UI timing in the existing parent trace, with child timestamps/CPU."""

    def __init__(self):
        self.queue = queue.Queue(8192)
        self.dropped = 0

    def event(self, kind, stage, **values):
        from . import performance as perf
        item = dict(event=kind, stage=stage, host_mono_ns=time.monotonic_ns(),
                    thread=threading.current_thread().name, source_process='ui',
                    process_id=os.getpid(), **perf.context_snapshot())
        item.update(values)
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    @contextmanager
    def span(self, stage, role='host_work', **values):
        start, cpu = time.monotonic_ns(), time.thread_time_ns()
        fields = dict(values)
        try:
            yield fields
        except BaseException as exc:
            fields['exception_type'] = type(exc).__name__
            raise
        finally:
            end = time.monotonic_ns()
            self.event('span', stage, role=role, start_mono_ns=start, end_mono_ns=end,
                       wall_ms=(end-start)/1e6, cpu_thread_ms=(time.thread_time_ns()-cpu)/1e6, **fields)

    def flush(self, pipe):
        rows = []
        for _ in range(24):
            try:
                rows.append(self.queue.get_nowait())
            except queue.Empty:
                break
        if rows:
            _send(pipe, dict(type='performance', rows=rows))


def _worker(channel_fd, storage_fd, parent_pid):
    # A fresh UI interpreter must never initialize CUDA or physical device APIs.
    class NoHardware(importlib.abc.MetaPathFinder):
        def find_spec(self, fullname, path=None, target=None):
            if (fullname.split('.')[0] in ('torch', 'pyrealsense2', 'canlib', 'main_rec', 'main_rec_v4')
                    or fullname in ('calib.control', 'calib.fsm_v4.top', 'calib.perception')):
                raise ImportError('UI worker cannot import hardware/inference: ' + fullname)
    sys.meta_path.insert(0, NoHardware())
    import ctypes
    ctypes.CDLL(None).prctl(1, signal.SIGTERM, 0, 0, 0)  # Linux parent-death signal.
    if os.getppid() != parent_pid:
        return
    pipe = Connection(channel_fd)
    mapping = video = monitor = None
    relay = None
    stopping = threading.Event()
    signal.signal(signal.SIGTERM, lambda *args: stopping.set())
    signal.signal(signal.SIGINT, lambda *args: stopping.set())
    statistics = dict(rendered=0, replaced=0, perf_dropped=0)
    try:
        configuration = _receive(pipe)
        if configuration['type'] != 'configure':
            raise ValueError('Missing UI configuration')
        spec, fps = configuration['spec'], float(configuration['fps'])
        import numpy as np
        import cv2
        from .runtime_view import make_renderer, fit_view_for_display
        from .runtime_threads import AsyncVideo
        from . import performance as perf
        render = make_renderer(spec)
        shape = tuple(spec['frame_shape'])
        frame_bytes = int(np.prod(shape))
        mapping = mmap.mmap(storage_fd, frame_bytes*2)
        os.close(storage_fd)
        if configuration['tracing']:
            relay = _TraceRelay()
            perf.global_set(relay)
        if spec.get('monitor_port'):
            from .remote_monitor import RemoteMonitor
            monitor = RemoteMonitor(int(spec['monitor_port'])).start()
            print(f'[REMOTE MONITOR] {monitor.url} (viewer disconnect does not stop execution)', flush=True)
        no_display = bool(spec.get('no_display'))
        def show(image):
            if monitor is not None:
                monitor.publish(image)
            elif not no_display:
                cv2.imshow(spec['window_title'], fit_view_for_display(image, spec.get('display_limit')))
        def read_key():
            return monitor.read_key() if monitor is not None else -1 if no_display else cv2.waitKey(1) & 0xff
        errors = queue.Queue()
        _send(pipe, dict(type='ready', pid=os.getpid(), forbidden_modules=[n for n in
                         ('torch','pyrealsense2','canlib','calib.control','calib.perception','calib.fsm_v4.top') if n in sys.modules]))
        pending = None
        next_display = 0.
        video_path = None
        recording_finished = False
        heartbeat = time.monotonic()
        while not stopping.is_set():
            for _ in range(32):
                if not pipe.poll():
                    break
                message = _receive(pipe)
                kind = message['type']
                if kind == 'stop':
                    stopping.set()
                    break
                if kind == 'frame':
                    if pending is not None:
                        _send(pipe, dict(type='ack', serial=pending['serial']))
                        statistics['replaced'] += 1
                    pending = message
                elif kind == 'recording':
                    video_path = message['path']
                elif kind == 'finish_recording':
                    recording_finished = True
                    if video is not None:
                        video.close()
            if not errors.empty():
                raise RuntimeError('UI video writer failed: ' + repr(errors.get_nowait()))
            now = time.monotonic()
            if pending is not None and now >= next_display and not stopping.is_set():
                packet = pending['packet']
                slot = pending['slot']
                packet['image'] = np.array(np.ndarray(shape, np.uint8, buffer=mapping, offset=slot*frame_bytes))
                _send(pipe, dict(type='ack', serial=pending['serial']))
                pending = None
                if recording_finished:
                    packet['recording'] = False
                with perf.using_context(packet['performance_context']):
                    perf.queue_wait('ui.latest_packet_age', packet['published_mono'])
                    with perf.span('ui.render', 'host_work'):
                        frame = render(packet)
                    if packet['recording'] and video_path and not recording_finished:
                        if video is None:
                            video = AsyncVideo(video_path, fps, errors.put_nowait)
                        video.submit(frame, packet['source_frame'], packet['captured_mono'])
                    elif video is not None:
                        video.pause()
                    with perf.span('ui.show', 'display_io'):
                        show(frame)
                    if packet['captured_mono'] is not None:
                        perf.event('latency.input_to_show_return', latency_ms=(time.monotonic()-packet['captured_mono'])*1000,
                                   note='Host show() return; not physical screen/display scanout.')
                statistics['rendered'] += 1
                response = dict(type='rendered', statistics=dict(statistics), source_frame=packet['source_frame'])
                if spec.get('pixel_digest'):
                    import hashlib
                    response['digest'] = hashlib.sha256(frame.tobytes()).hexdigest()
                _send(pipe, response)
                next_display = max(now + 1./fps, time.monotonic())
            key = read_key()
            if key in (27, ord('r'), ord(' ')):
                _send(pipe, dict(type='key', key=key))
            if relay is not None:
                relay.flush(pipe)
                if time.monotonic()-heartbeat >= 1.:
                    statistics['perf_dropped'] = relay.dropped
                    perf.event('ui.process_heartbeat', process_cpu_time_ns=time.process_time_ns(),
                               **statistics)
                    heartbeat = time.monotonic()
            stopping.wait(.001)
    except (EOFError, BrokenPipeError):
        pass
    except BaseException:
        import traceback
        try:
            _send(pipe, dict(type='error', error=traceback.format_exc(limit=8)))
        except (OSError, EOFError):
            pass
    finally:
        try:
            if video is not None:
                video.close()
                statistics['video'] = dict(accepted=video.accepted, written=video.written, dropped=video.dropped)
            if monitor is not None:
                monitor.close()
                statistics['monitor'] = monitor.telemetry
            if 'spec' in locals() and not spec.get('no_display'):
                cv2.destroyAllWindows()
            if relay is not None:
                statistics['perf_dropped'] = relay.dropped
                while not relay.queue.empty():
                    relay.flush(pipe)
            _send(pipe, dict(type='stopped', statistics=statistics))
        except (OSError, EOFError):
            pass
        if mapping is not None:
            mapping.close()
        pipe.close()


if __name__ == '__main__':
    if len(sys.argv) != 5 or sys.argv[1] != '--worker':
        raise SystemExit('Internal UI worker entrypoint')
    _worker(*(int(v) for v in sys.argv[2:]))
