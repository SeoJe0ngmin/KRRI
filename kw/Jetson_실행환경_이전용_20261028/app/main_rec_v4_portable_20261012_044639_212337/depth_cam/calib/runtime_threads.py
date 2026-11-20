"""Single control owner, latest-only presentation, bounded background file IO.

No camera, inference model or live FSM object is owned by the presentation process.
Video sidecars map accepted frames to source/host time; queue drops are explicit.
"""
from __future__ import annotations

import copy
import json
import queue
import threading
import time
from collections import deque
from types import SimpleNamespace

import cv2
from .performance import span, profiled, context_snapshot, using_context, queue_wait, event, aggregate_span


class OrderedIO:
    """Bounded FIFO. Critical log overflow fails the session instead of blocking."""

    def __init__(self, name, on_error, capacity=4096):
        self.queue = queue.Queue(capacity)
        self.on_error = on_error
        self.error = None
        self.closing = threading.Event()
        self.thread = threading.Thread(target=self._run, name=name)
        self.thread.start()

    def submit(self, call, *args):
        if self.error is not None:
            raise RuntimeError("background file IO failed") from self.error
        if self.closing.is_set():
            raise RuntimeError("background file IO is closed")
        try:
            self.queue.put_nowait((call, args, time.monotonic(), context_snapshot()))
        except queue.Full as exc:
            self.on_error(RuntimeError("critical log queue full"))
            raise RuntimeError("critical log queue full") from exc

    def _run(self):
        while not self.closing.is_set() or not self.queue.empty():
            try:
                call, args, queued, context = self.queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                with using_context(context):
                    owner = getattr(call, '__self__', None)
                    path = str(getattr(owner, 'path', ''))
                    queue_wait('trace_io.queue', queued, task=call.__name__, path=path, queue_depth=self.queue.qsize())
                    with span('trace_io.' + call.__name__, 'file_io', path=path):
                        call(*args)
            except BaseException as exc:
                self.error = exc
                self.on_error(exc)
            finally:
                self.queue.task_done()

    def close(self):
        self.closing.set()
        self.thread.join()


class AsyncTextFile:
    """File operations deferred; caller still creates timestamps/serialized rows."""

    def __init__(self, io, path):
        self.io, self.handle = io, None
        self.path = path
        io.submit(self._open, path)

    def _open(self, path):
        self.handle = open(path, "w", encoding="utf-8")

    def write(self, text):
        self.io.submit(self._write, text)

    def _write(self, text):
        self.handle.write(text)

    def flush(self):
        self.io.submit(self._flush)

    def _flush(self):
        if self.handle is not None:
            self.handle.flush()

    def close(self):
        # Called only after all producers are stopped. Drain first even on error.
        self.io.close()
        if self.handle is not None:
            self.handle.close()


class AsyncVideo:
    """One writer owner; accepted frame index is allocated before inference logs.

    Gaps use the previous accepted image to preserve elapsed playback time.
    Sidecars distinguish these repeated frames from newly accepted images.
    """

    def __init__(self, path, fps, on_error, capacity=8):
        self.path, self.fps = path, float(fps)
        self.on_error = on_error
        self.queue = queue.Queue(capacity)
        self.closing = threading.Event()
        self.accepted = self.dropped = self.written = 0
        self.next_index = 0
        self.epoch = None
        self.epoch_index = 0
        self.thread = threading.Thread(target=self._run, name="Video:" + path)
        self.thread.start()

    def submit(self, frame, source_frame=None, captured_mono=None):
        if self.closing.is_set():
            return None
        if self.queue.full():
            self.dropped += 1
            event('video.drop', path=self.path, dropped=self.dropped, queue_depth=self.queue.qsize())
            return None
        timestamp = time.monotonic() if captured_mono is None else captured_mono
        if self.epoch is None:
            self.epoch, self.epoch_index = timestamp, self.next_index
        index = max(self.next_index, self.epoch_index + round((timestamp-self.epoch)*self.fps))
        with span('video.frame_copy', 'host_work', path=self.path):
            copied_frame = frame.copy()
        item = (copied_frame, dict(index=index, source_frame=source_frame,
                captured_mono=captured_mono, queued_mono=time.monotonic(),
                repeat_from_index=self.next_index, repeat_count=index-self.next_index,
                dropped_before=self.dropped), context_snapshot())
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            self.dropped += 1
            event('video.drop', path=self.path, dropped=self.dropped, queue_depth=self.queue.qsize())
            return None
        self.accepted += 1
        self.next_index = index + 1
        return index

    def pause(self):
        """Start a new recording time segment after an explicit recording pause."""
        self.epoch = None

    def _run(self):
        writer = mapping = previous = None
        try:
            while not self.closing.is_set() or not self.queue.empty():
                try:
                    frame, meta, perf_context = self.queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                try:
                    with using_context(perf_context):
                        queue_wait('video.queue', meta['queued_mono'], path=self.path, queue_depth=self.queue.qsize())
                        if writer is None:
                            h, w = frame.shape[:2]
                            with span('video.encoder_open', 'codec_and_file_io', path=self.path):
                                writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"),
                                                         self.fps, (w, h))
                            if not writer.isOpened():
                                raise RuntimeError("VideoWriter failed: " + self.path)
                            mapping = open(self.path + ".frames.jsonl", "w", encoding="utf-8")
                        for _ in range(meta['repeat_count']):
                            with span('video.encode_write_api', 'codec_and_file_io', path=self.path, repeated=True):
                                writer.write(previous if previous is not None else frame)
                            self.written += 1
                        with span('video.encode_write_api', 'codec_and_file_io', path=self.path, repeated=False):
                            writer.write(frame)
                        previous = frame
                        with span('video.sidecar_write', 'file_io', path=self.path):
                            mapping.write(json.dumps(meta) + "\n")
                        self.written += 1
                finally:
                    self.queue.task_done()
        except BaseException as exc:
            self.on_error(exc)
        finally:
            if writer is not None:
                with span('video.finalize', 'codec_and_file_io', path=self.path):
                    writer.release()
            if mapping is not None:
                mapping.write(json.dumps(dict(summary=True, accepted=self.accepted,
                    written=self.written, dropped=self.dropped, fps=self.fps)) + "\n")
                mapping.close()

    def close(self):
        self.closing.set()
        self.thread.join()


def snapshot_fsm(fsm):
    """Explicit UI contract, evaluated on the FSM owner, never a live FSM copy."""
    names = ("state", "align_sub", "recover_sub", "failure_state", "_failure_state",
             "failure_reason", "_failure_reason", "_visual_recovery_origin_state",
             "debug_step_enabled", "debug_step_running", "debug_step_count", "cmd_status")
    values = {key: copy.deepcopy(getattr(fsm, key, None)) for key in names}
    status = getattr(fsm, "motion_target_status", None)
    values["motion_target_status"] = copy.deepcopy(status() if callable(status) else status)
    return SimpleNamespace(**values)


class RuntimePresentation:
    def __init__(self, stop_motion, reset_motion, fps=20.0, process_ui=True):
        self.stop_motion, self.reset_motion = stop_motion, reset_motion
        self.fps = fps
        self.process_ui = bool(process_ui)
        self.ui_process = None
        self.stop = threading.Event()
        self.done = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.keys = queue.Queue(32)
        self.error = None
        self.next_publish = 0.0
        self.raw = self.video = None
        self.recording_finished = threading.Event()
        self.video_path = None
        self.trace_io = None
        self.diagnostics = deque(maxlen=100000)
        self.previous_step = None
        self.max_step_gap = 0.0
        self.step_count = 0
        self.trace_base = None
        self.cleanups = []
        self.can_write_gaps = deque(maxlen=100000)
        self.can_write_count = 0
        self.last_can_write_ms = None
        self.max_can_gap_ms = 0.0

    def observe_can_tx(self, **event):
        """Called on CAN owner: constant-time collection, no serialization/IO."""
        if event.get('event') != 'movement_write':
            return
        now = event.get('write_start_host_mono_ms')
        if now is None:
            return
        self.can_write_count += 1
        if self.last_can_write_ms is not None:
            gap = now - self.last_can_write_ms
            self.can_write_gaps.append(gap)
            self.max_can_gap_ms = max(self.max_can_gap_ms, gap)
        self.last_can_write_ms = now

    def instrument_fsm(self, fsm, trace_base):
        self.trace_base = trace_base
        original = fsm.step

        def step(*args, **kwargs):
            if self.stop.is_set():
                return []
            now = time.monotonic()
            tick = time.perf_counter()
            meta = kwargs.get('vision_meta') or {}
            interval = None if self.previous_step is None else tick - self.previous_step
            if interval is not None:
                self.max_step_gap = max(self.max_step_gap, interval)
            self.previous_step = tick
            self.step_count += 1
            result = original(*args, **kwargs)
            work = time.perf_counter() - tick
            measured = meta.get('measurement_mono')
            pose = meta.get('pose_result_mono')
            self.diagnostics.append((interval, None if measured is None else now-measured,
                                     None if pose is None else now-pose, work))
            return result

        fsm.step = step

    def write_diagnostics(self):
        import numpy as np
        report = dict(step_count=self.step_count, retained_samples=len(self.diagnostics),
                      max_step_gap_ms=self.max_step_gap * 1000, ui_fps_limit=self.fps,
                      error=None if self.error is None else repr(self.error))
        if self.ui_process is not None:
            report['ui_process'] = dict(self.ui_process.statistics)
        for index, key in enumerate(('step_interval', 'observation_to_step', 'pose_to_step', 'step_work')):
            values = [row[index] * 1000 for row in self.diagnostics if row[index] is not None]
            report[key] = None if not values else dict(mean_ms=float(np.mean(values)),
                p95_ms=float(np.percentile(values, 95)), p99_ms=float(np.percentile(values, 99)),
                max_ms=max(values))
        for key, writer in (('raw_video', self.raw), ('ui_video', self.video)):
            if writer is not None:
                report[key] = dict(accepted=writer.accepted, written=writer.written, dropped=writer.dropped)
        report['can_movement_write_count'] = self.can_write_count
        report['can_movement_write_gap'] = None if not self.can_write_gaps else dict(
            p95_ms=float(np.percentile(self.can_write_gaps, 95)),
            p99_ms=float(np.percentile(self.can_write_gaps, 99)), max_ms=self.max_can_gap_ms)
        if self.trace_base:
            with open(self.trace_base + '_runtime_threads.json', 'w', encoding='utf-8') as handle:
                json.dump(report, handle, indent=2)
        print('[RUNTIME THREADS]', json.dumps(report), flush=True)

    def request_stop(self):
        self.stop.set()
        self.stop_motion()

    def fail(self, exc):
        with self.lock:
            if self.error is None:
                self.error = exc
        self.request_stop()

    def wants_frame(self):
        return time.perf_counter() >= self.next_publish

    @profiled('ui.publish_copy')
    def publish(self, image, fsm, lines, recording, overlay=None,
                yaw=None, z=None, source_frame=None, captured_mono=None):
        if not self.wants_frame():
            return
        self.next_publish = time.perf_counter() + 1.0 / self.fps
        packet = dict(image=image.copy(), fsm=snapshot_fsm(fsm), lines=copy.deepcopy(lines),
                      recording=recording, overlay=copy.deepcopy(overlay), yaw=yaw, z=z,
                      source_frame=source_frame, captured_mono=captured_mono, performance_context=context_snapshot(),
                      published_mono=time.monotonic())
        with self.lock:
            self.latest = packet

    def take(self):
        with self.lock:
            packet, self.latest = self.latest, None
        return packet

    def configure_recording(self, raw_path, video_path, fps):
        if self.recording_finished.is_set():
            return
        if self.raw is None:
            self.raw = AsyncVideo(raw_path, fps, self.fail)
        self.video_path = video_path

    def finish_recording(self):
        """Called by control owner; UI closes its own encoder on its next tick."""
        self.recording_finished.set()
        if self.raw is not None:
            self.raw.close()

    def _ui_message(self, message):
        if message['type'] == 'error':
            self.fail(RuntimeError(message['error']))
        elif message['type'] == 'stopped':
            # Also stop when the UI ends cleanly (e.g. external SIGTERM).
            self.request_stop()
        elif message['type'] == 'key':
            key = message['key']
            if key == 27:
                self.request_stop()
            elif key in (ord('r'), ord(' ')):
                try:
                    self.keys.put_nowait(key)
                except queue.Full:
                    self.fail(RuntimeError('UI command queue full'))
        elif message['type'] == 'performance':
            from .performance import active
            trace = active()
            if trace is not None:
                for row in message['rows']:
                    fields = dict(row)
                    kind, stage = fields.pop('event'), fields.pop('stage')
                    trace.event(kind, stage, **fields)

    def _run_process(self, control, ui_spec):
        from .ui_process import UIProcess
        from .performance import active
        self.reset_motion()
        self.trace_io = OrderedIO('RuntimeTraceIO', self.fail)
        thread = None
        try:
            # Complete UI startup before allowing camera/CAN/FSM initialization.
            self.ui_process = UIProcess(ui_spec, self.fps, tracing=active() is not None)
            def worker():
                try:
                    control()
                except BaseException as exc:
                    self.fail(exc)
                finally:
                    self.request_stop()
                    for cleanup in self.cleanups:
                        try:
                            cleanup()
                        except Exception as exc:
                            self.fail(exc)
                    self.done.set()
            thread = threading.Thread(target=worker, name='PerceptionFSM')
            thread.start()
            while not self.done.is_set() and not self.stop.is_set():
                for message in self.ui_process.poll():
                    self._ui_message(message)
                if self.stop.is_set():
                    break
                self.ui_process.sync_recording(self.video_path, self.recording_finished.is_set())
                if self.ui_process.free:
                    packet = self.take()
                    if packet is not None:
                        with span('ui.ipc_submit', 'host_work'):
                            self.ui_process.submit(packet)
                self.done.wait(.001)
        except BaseException as exc:
            self.fail(exc)
        finally:
            # Child failures/ESC inhibit movement before recorder/IPC draining.
            self.request_stop()
            if thread is not None:
                thread.join()
            if self.raw is not None:
                self.raw.close()
            if self.ui_process is not None:
                try:
                    self.ui_process.close(on_message=self._ui_message)
                except BaseException as exc:
                    self.fail(exc)
                video = self.ui_process.statistics.get('video')
                if video is not None:
                    self.video = SimpleNamespace(**video)
            self.trace_io.close()
            self.write_diagnostics()
        if self.error is not None:
            raise self.error

    def run(self, control, render, show, read_key, close_windows, ui_spec=None):
        if self.process_ui and ui_spec is not None:
            return self._run_process(control, ui_spec)
        self.reset_motion()
        self.trace_io = OrderedIO("RuntimeTraceIO", self.fail)

        def worker():
            try:
                control()
            except BaseException as exc:
                self.fail(exc)
            finally:
                self.request_stop()
                for cleanup in self.cleanups:
                    try:
                        cleanup()
                    except Exception as exc:
                        self.fail(exc)
                self.done.set()

        thread = threading.Thread(target=worker, name="PerceptionFSM")
        thread.start()
        next_display = 0.0
        try:
            while not self.done.is_set() and not self.stop.is_set():
                if (self.recording_finished.is_set() and self.video is not None
                        and not self.video.closing.is_set()):
                    self.video.close()
                now = time.perf_counter()
                if now >= next_display:
                    packet = self.take()
                    if packet is not None:
                        if self.recording_finished.is_set():
                            packet['recording'] = False
                        with using_context(packet['performance_context']):
                            queue_wait('ui.latest_packet_age', packet['published_mono'])
                            with span('ui.render', 'host_work'):
                                frame = render(packet)
                        if (packet['recording'] and self.video_path
                                and not self.recording_finished.is_set()):
                            if self.video is None:
                                self.video = AsyncVideo(self.video_path, self.fps, self.fail)
                            with using_context(packet['performance_context']):
                                self.video.submit(frame, packet['source_frame'], packet['captured_mono'])
                        elif self.video is not None:
                            self.video.pause()
                        with using_context(packet['performance_context']):
                            with span('ui.show', 'display_io'):
                                show(frame)
                            if packet['captured_mono'] is not None:
                                event('latency.input_to_show_return', latency_ms=(time.monotonic()-packet['captured_mono'])*1000,
                                      note='Host show() return; not physical screen/display scanout.')
                    next_display = max(now + 1.0 / self.fps, time.perf_counter())
                with aggregate_span('ui.key_poll', 'ui_event_poll'):
                    key = read_key()
                if key == 27:
                    self.request_stop()
                elif key in (ord('r'), ord(' ')):
                    try:
                        self.keys.put_nowait(key)
                    except queue.Full:
                        self.fail(RuntimeError("UI command queue full"))
                with aggregate_span('ui.idle_wait', 'intentional_wait'):
                    self.done.wait(0.001)
        except BaseException as exc:
            self.fail(exc)
        finally:
            # Inhibit movement first, then drain control and recording resources.
            self.request_stop()
            thread.join()
            for writer in (self.raw, self.video):
                if writer is not None:
                    writer.close()
            self.trace_io.close()
            close_windows()
            self.write_diagnostics()
        if self.error is not None:
            raise self.error
