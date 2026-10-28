"""Bounded, asynchronous diagnostic tracing; never owns hardware or motion.

Wall time includes scheduling. Thread CPU time excludes descheduling/blocking.
CUDA events describe a stream interval, not exclusive GPU busy/kernel time.
Profiling failure/dropped events are reported, never used to change control.
"""
from __future__ import annotations

from collections import defaultdict, deque
from contextlib import contextmanager, nullcontext
from datetime import datetime
import functools
import hashlib
import itertools
import json
import os
from pathlib import Path
import queue
import shutil
import subprocess
import threading
import time
import uuid

_active = None
_local = threading.local()
_null = nullcontext()


def active():
    return _active


def enabled(camera, demo):
    from . import config
    camera = config.CAMERA_ENABLED if camera is None else camera
    return (bool(camera) and not demo and config.TRACE_ENABLE
            and os.environ.get('LIFTER_PERF_LOG', '1') != '0')


def context(**values):
    if _active is not None:
        _local.context = dict(getattr(_local, 'context', {}), **values)


def context_snapshot():
    return dict(getattr(_local, 'context', {})) if _active is not None else {}


@contextmanager
def using_context(values):
    previous = getattr(_local, 'context', {})
    _local.context = values
    try:
        yield
    finally:
        _local.context = previous


def queue_wait(stage, queued_mono, **values):
    if _active is not None:
        _active.event('span', stage, role='queue_wait', wall_ms=max(0., (time.monotonic()-queued_mono)*1000),
                      cpu_thread_ms=0., **values)


@contextmanager
def aggregate_span(stage, role):
    """Keep frequent UI polling cheap; emit per-second totals and max."""
    if _active is None:
        yield
        return
    start, cpu = time.monotonic_ns(), time.thread_time_ns()
    try:
        yield
    finally:
        end = time.monotonic_ns()
        values = getattr(_local, 'aggregates', None)
        if values is None:
            values = _local.aggregates = {}
        item = values.setdefault(stage, [start, 0, 0., 0., 0.])
        wall = (end-start)/1e6
        item[1] += 1; item[2] += wall
        item[3] += (time.thread_time_ns()-cpu)/1e6
        item[4] = max(item[4], wall)
        if end-item[0] >= 1e9:
            _active.event('aggregate', stage, role=role, calls=item[1],
                total_wall_ms=item[2], total_cpu_thread_ms=item[3], max_call_ms=item[4],
                window_ms=(end-item[0])/1e6)
            values[stage] = [end, 0, 0., 0., 0.]


def iteration(state, vision_enabled):
    if _active is None:
        return
    now = time.monotonic_ns()
    previous = getattr(_local, 'iteration_start', None)
    _local.context = dict(iteration_id=getattr(_local, 'iteration_id', 0) + 1,
                          fsm_state=state, vision_enabled=bool(vision_enabled),
                          camera_frame_number=None)
    _local.iteration_id = _local.context['iteration_id']
    _local.iteration_start = now
    if previous is not None:
        _active.event('loop_interval', 'loop.start_interval',
                      interval_ms=(now-previous)/1e6,
                      note='Previous control iteration including intentional waits; not compute time.')


def event(stage, **values):
    if _active is not None:
        _active.event('metric', stage, **values)


def span(stage, role='host_work', **values):
    return _active.span(stage, role, **values) if _active is not None else _null


def profiled(stage, role='host_work'):
    def decorate(fn):
        @functools.wraps(fn)
        def call(*args, **kwargs):
            if _active is None:
                return fn(*args, **kwargs)
            with _active.span(stage, role):
                return fn(*args, **kwargs)
        return call
    return decorate


@contextmanager
def timed_lock(lock, stage):
    with span(stage, 'lock_wait'):
        lock.acquire()
    try:
        yield
    finally:
        lock.release()


def gpu_span(stage, device):
    return _active.gpu_span(stage, device) if _active is not None else _null


def traced_runtime(fn):
    """One owner outside recursive main/UI workers; closes even on startup error."""
    @functools.wraps(fn)
    def call(*args, **kwargs):
        from . import config
        camera = kwargs.get('camera_enabled', config.CAMERA_ENABLED)
        camera = config.CAMERA_ENABLED if camera is None else camera
        enabled = (config.TRACE_ENABLE and camera and not kwargs.get('demo_mode', False)
                   and os.environ.get('LIFTER_PERF_LOG', '1') != '0')
        if _active is not None or not enabled:
            return fn(*args, **kwargs)
        paths = fn.__globals__['setup_video_writer_filename'](
            kwargs.get('recording_prefix', 'forklift_recording'))
        kwargs['_performance_paths'] = paths
        trace = PerformanceTrace(str(Path(paths[0]).with_suffix('')))
        global_set(trace)
        try:
            trace.event('session', 'runtime.start', camera_enabled=bool(camera),
                        can_enabled=bool(kwargs.get('can_enabled', True)),
                        no_display=bool(kwargs.get('no_display', False)),
                        debug_step_mode=bool(kwargs.get('debug_step_mode', False)),
                        threaded_ui=bool(kwargs.get('threaded_ui', True)))
            return fn(*args, **kwargs)
        finally:
            try:
                trace.close()
            finally:
                global_set(None)
    return call


def global_set(trace):
    global _active
    _active = trace
    _local.context = {}


class PerformanceTrace:
    def __init__(self, base, capacity=8192, system_sampling=True):
        self.base = str(base)
        self.session_id = Path(base).name + '-' + uuid.uuid4().hex[:8]
        self.queue = queue.Queue(capacity)
        self.stop = threading.Event()
        self.closed = False
        self.dropped = 0
        self.written = 0
        self.error = None
        self.gpu_skipped = 0
        self.ids = itertools.count(1)
        self.pending = deque()
        self.gpu_lock = threading.Lock()
        self.gpu_free = []
        self.gpu_created = 0
        self.summary = defaultdict(lambda: dict(count=0, sum_ms=0., cpu_ms=0., cpu_count=0, max_ms=0., recent=deque(maxlen=2048)))
        self.io_write_ms = 0.
        self.io_cpu_ms = 0.
        self.io_queue_max_ms = 0.
        self.started_ns = time.monotonic_ns()
        self.system_sampling = system_sampling
        self.thread = threading.Thread(target=self._run, name='PerformanceLog', daemon=True)
        self.thread.start()

    def event(self, kind, stage, **values):
        if self.closed or self.error is not None:
            return
        item = dict(schema=1, session_id=self.session_id, event=kind, stage=stage,
                    record_id=next(self.ids), host_mono_ns=time.monotonic_ns(),
                    thread=threading.current_thread().name,
                    **getattr(_local, 'context', {}))
        item.update(values)
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    @contextmanager
    def span(self, stage, role='host_work', **values):
        begin, cpu = time.monotonic_ns(), time.thread_time_ns()
        fields = dict(values)
        try:
            yield fields
        except BaseException as exc:
            fields['exception_type'] = type(exc).__name__
            raise
        finally:
            end, cpu_end = time.monotonic_ns(), time.thread_time_ns()
            self.event('span', stage, role=role, start_mono_ns=begin, end_mono_ns=end,
                       wall_ms=(end-begin)/1e6, cpu_thread_ms=(cpu_end-cpu)/1e6, **fields)

    @contextmanager
    def gpu_span(self, stage, device):
        pair = None
        try:
            import torch
            if str(device).startswith('cuda'):
                with self.gpu_lock:
                    if self.gpu_free:
                        pair = self.gpu_free.pop()
                    elif self.gpu_created < 128:
                        pair = (torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True))
                        self.gpu_created += 1
                if pair is not None:
                    stream = torch.cuda.current_stream(device)
                    pair[0].record(stream)
                    info = dict(getattr(_local, 'context', {}))
                    info['producer_thread'] = threading.current_thread().name
                    host_start = time.monotonic_ns()
                else:
                    self.gpu_skipped += 1
        except Exception as exc:
            self.event('diagnostic', 'cuda.timing_unavailable', detail=repr(exc))
            pair = None
        try:
            yield
        finally:
            if pair is not None:
                try:
                    pair[1].record(stream)
                    with self.gpu_lock:
                        self.pending.append((pair, stage, str(device), host_start, info))
                except Exception as exc:
                    self.event('diagnostic', 'cuda.timing_unavailable', detail=repr(exc))

    def _poll_gpu(self):
        # query() only: no synchronize(), no added wait on the control stream.
        with self.gpu_lock:
            jobs = list(self.pending)
            self.pending.clear()
        retry = []
        for pair, stage, device, host_start, info in jobs:
            try:
                if not pair[1].query():
                    retry.append((pair, stage, device, host_start, info))
                    continue
                self.event('cuda_stream', stage, device=device, gpu_stream_ms=pair[0].elapsed_time(pair[1]),
                           launch_host_mono_ns=host_start, **info)
                with self.gpu_lock:
                    self.gpu_free.append(pair)
            except Exception as exc:
                self.event('diagnostic', 'cuda.event_read_error', detail=repr(exc))
        with self.gpu_lock:
            self.pending.extend(retry)

    def _snapshot(self, final=False):
        result = dict(schema=1, session_id=self.session_id, final=final,
                      host_mono_ns=time.monotonic_ns(), written_events=self.written,
                      dropped_events=self.dropped, pending_events=self.queue.qsize(),
                      error=self.error, gpu_timing_skipped=self.gpu_skipped,
                      pending_gpu_events=len(self.pending),
                      performance_writer=dict(write_wall_ms=self.io_write_ms,
                          write_cpu_ms=self.io_cpu_ms, max_queue_wait_ms=self.io_queue_max_ms), stages={})
        for key, item in self.summary.items():
            recent = sorted(item['recent'])
            n = item['count']
            def pct(q):
                p = (len(recent)-1)*q
                a = int(p); b = min(a+1, len(recent)-1)
                return recent[a] + (recent[b]-recent[a])*(p-a)
            result['stages'][key] = dict(count=n, mean_ms=item['sum_ms']/n,
                mean_cpu_thread_ms=(item['cpu_ms']/item['cpu_count'] if item['cpu_count'] else None), max_ms=item['max_ms'],
                recent_window_count=len(recent), recent_window_p50_ms=pct(.5),
                recent_window_p95_ms=pct(.95), recent_window_p99_ms=pct(.99))
        path = Path(self.base + '_performance_summary.json')
        tmp = path.with_suffix('.json.tmp')
        tmp.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        tmp.replace(path)

    def _run(self):
        process = sampler = None
        try:
            Path(self.base).parent.mkdir(parents=True, exist_ok=True)
            with open(self.base + '_performance.jsonl', 'w', encoding='utf-8') as handle:
                self.event('session', 'schema', wall_iso=datetime.now().isoformat(),
                    pid=os.getpid(), clock='host monotonic nanoseconds; same host across processes',
                    definitions=dict(cpu_thread_ms='CPU time of this thread only; not device time',
                      wall_ms='elapsed including CPU scheduling/blocking; nested/parallel spans overlap',
                      gpu_stream_ms='CUDA stream interval, includes in-stream contention/launch gaps, not isolated busy time'),
                    queue_capacity=self.queue.maxsize)
                root = Path(__file__).resolve().parents[1]
                paths = [root/'main_rec.py',root/'main_rec_v4.py'] + list((root/'calib').glob('*.py'))
                paths += list((root/'calib/fsm_v4').glob('*.py'))
                paths += list((root/'calib/fsm_v4').glob('*.json'))
                self.event('session', 'source_identity', sha256={str(p.relative_to(root)):
                    hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
                executable = shutil.which('tegrastats')
                if self.system_sampling and executable:
                    try:
                        process = subprocess.Popen([executable, '--interval', '1000'], stdout=subprocess.PIPE,
                                                   stderr=subprocess.STDOUT, text=True, bufsize=1)
                    except OSError as exc:
                        self.event('diagnostic', 'system_sampler_error', detail=repr(exc))
                if process is not None:
                    def read_system():
                        for line in process.stdout:
                            if self.stop.is_set():
                                break
                            self.event('system', 'tegrastats', wall_iso=datetime.now().isoformat(),
                                       sampled_at='stdout receipt', raw=line.strip(),
                                       process_cpu_time_ns=time.process_time_ns(),
                                       python_thread_count=threading.active_count())
                    sampler = threading.Thread(target=read_system, name='PerformanceSystem', daemon=True)
                    sampler.start()
                elif not executable or not self.system_sampling:
                    self.event('diagnostic', 'system_sampler', available=bool(executable),
                               enabled=self.system_sampling)
                flushed = time.monotonic()
                gpu_polled = 0.
                while not self.stop.is_set() or not self.queue.empty():
                    if self.stop.is_set() or time.monotonic()-gpu_polled >= .02:
                        self._poll_gpu()
                        gpu_polled = time.monotonic()
                    try:
                        row = self.queue.get(timeout=.05)
                    except queue.Empty:
                        row = None
                    if row is not None:
                        start, cpu = time.monotonic_ns(), time.thread_time_ns()
                        delay = (start-row['host_mono_ns'])/1e6
                        row['performance_queue_wait_ms'] = delay
                        self.io_queue_max_ms = max(self.io_queue_max_ms, delay)
                        handle.write(json.dumps(row, ensure_ascii=False, default=str) + '\n')
                        self.written += 1
                        duration = row.get('wall_ms', row.get('gpu_stream_ms'))
                        if duration is not None:
                            key = row['stage'] + '|' + row.get('role', row['event'])
                            item = self.summary[key]
                            item['count'] += 1
                            item['sum_ms'] += duration
                            item['cpu_ms'] += row.get('cpu_thread_ms', 0.)
                            item['cpu_count'] += 'cpu_thread_ms' in row
                            item['max_ms'] = max(item['max_ms'], duration)
                            item['recent'].append(duration)
                        self.io_write_ms += (time.monotonic_ns()-start)/1e6
                        self.io_cpu_ms += (time.thread_time_ns()-cpu)/1e6
                        self.queue.task_done()
                    if time.monotonic()-flushed >= 1.:
                        handle.flush()
                        self._snapshot()
                        flushed = time.monotonic()
                handle.flush()
            self._snapshot(final=True)
        except Exception as exc:
            self.error = repr(exc)
            print('[PERFORMANCE LOG] logging disabled:', self.error, flush=True)
        finally:
            if process is not None:
                if process.poll() is None:
                    process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
                if sampler is not None:
                    sampler.join(timeout=2)
                process.stdout.close()

    def close(self):
        if self.closed:
            return
        # Existing inference D2H normally completes events. Never force GPU sync.
        self.event('session', 'runtime.end')
        self.stop.set()
        self.thread.join()
        self.closed = True
