"""Run the route search in one child process, with optional CUDA visibility.

The owner remains responsible for STOP, result freshness and all execution
decisions.  Cancelling work destroys the process; its next job starts a new one.
Keep the application's hardware imports inside its ``__main__`` guard because
the spawn start method imports that entry point in the child.
"""
from __future__ import annotations

import multiprocessing
import os
import pickle
import signal
import sys
import time
from ..performance import span, profiled

_gpu_cache = None
_gpu_failure = None


def _gpu_visibility_backend():
    global _gpu_cache, _gpu_failure
    from . import config as cfg
    key = tuple(getattr(cfg, name) for name in (
        'CAMERA_TO_ROT_CENTER_X_M', 'CAMERA_TO_ROT_CENTER_Z_M',
        'CAMERA_YAW_IN_VEHICLE_DEG', 'PALLET_FRONT_VISIBILITY_WIDTH_M',
        'VISIBILITY_MIN_CORNER_DEPTH_M', 'VISIBILITY_PATH_SAMPLE_STEP_DEG',
        'VISIBILITY_FORWARD_SEARCH_ITERATIONS')) + (cfg._pose_config.CUDA_DEVICE,)
    if _gpu_failure is not None and _gpu_failure[0] == key:
        raise RuntimeError(_gpu_failure[1])
    if _gpu_cache is None or _gpu_cache[0] != key:
        _close_gpu()
        from .gpu_visibility import GPUVisibility
        try:
            _gpu_cache = (key, GPUVisibility(device=cfg._pose_config.CUDA_DEVICE))
        except Exception as exc:
            _gpu_failure = (key, f'{type(exc).__name__}: {exc}')
            raise
    return _gpu_cache[1]


def _close_gpu():
    global _gpu_cache
    if _gpu_cache is not None:
        _gpu_cache[1].close()
        _gpu_cache = None


def planner_config_snapshot() -> dict:
    """Detach all public configuration constants, including fitted responses."""
    from . import config as cfg

    values = {name: value for name, value in vars(cfg).items()
              if name.isupper() and not name.startswith('_')}
    # Fail explicitly for a non-pickleable runtime patch rather than silently
    # allowing the child to use a different configuration.
    return pickle.loads(pickle.dumps(values, protocol=pickle.HIGHEST_PROTOCOL))


def _compute(request: dict, *, telemetry=None):
    from . import config as cfg
    from .route_planner import plan_fine_route, rotation_seconds
    from .controllers import heading_rotation_limit_deg

    overrides = request.get('config', {})
    if not isinstance(overrides, dict):
        raise TypeError('planner config must be a dict')
    for name in overrides:
        if (not isinstance(name, str) or not name.isupper()
                or name.startswith('_') or not hasattr(cfg, name)):
            raise ValueError(f'unknown planner configuration: {name!r}')
    previous = {name: getattr(cfg, name) for name in overrides}
    try:
        for name, value in overrides.items():
            setattr(cfg, name, value)

        def turn_time(turn, fine):
            slope = request.get('slope_right' if turn > 0 else 'slope_left', 1.)
            return rotation_seconds(
                turn, fine, slope_multiplier=slope,
                time_scale=request.get('time_scale', 1.),
            )

        arguments = dict(
            remaining_cycles=request['remaining_cycles'],
            remaining_seconds=request.get('remaining_seconds', float('inf')),
            turn_time=turn_time,
            rotation_limit_deg=heading_rotation_limit_deg(
                slope_left=request.get('slope_left',1.), slope_right=request.get('slope_right',1.),
                time_scale=request.get('time_scale',1.)),
            alignment_entered=request.get('alignment_entered', False),
            remaining_direction_changes=request.get('remaining_direction_changes'),
            last_rotation_command=request.get('last_rotation_command'),
        )
        if telemetry is not None:
            telemetry['backend'] = 'cpu'
        if request.get('gpu_visibility', cfg.ROUTE_GPU_VISIBILITY_ENABLED):
            try:
                init_started = time.monotonic()
                backend = _gpu_visibility_backend()
                if hasattr(backend, 'enable_timing'):
                    backend.enable_timing(bool(request.get('profile_timing', False)))
                if telemetry is not None:
                    telemetry['backend_init_wall_ms'] = (time.monotonic()-init_started)*1000
                before = dict(backend.stats)
                route = plan_fine_route(request['pose'], request.get('vision_meta'),
                    request['remaining_forward_m'], visibility_backend=backend, **arguments)
                metrics = {name:value-before[name] for name,value in backend.stats.items()}
                if telemetry is not None:
                    telemetry.update(backend='cuda-float64-visibility', gpu=metrics)
                    telemetry['gpu_timing_error'] = getattr(backend, 'timing_error', None)
                print('[ROUTE GPU]', metrics, flush=True)
                return route
            except Exception as exc:
                _close_gpu()
                if telemetry is not None:
                    telemetry.update(backend='cpu-fallback', gpu_error=f'{type(exc).__name__}: {exc}')
                print(f'[ROUTE GPU] CPU fallback: {type(exc).__name__}: {exc}', flush=True)
        return plan_fine_route(
            request['pose'], request.get('vision_meta'),
            request['remaining_forward_m'],
            **arguments,
        )
    finally:
        for name, value in previous.items():
            setattr(cfg, name, value)


def _parent_death_guard(parent_pid: int) -> None:
    if not sys.platform.startswith('linux'):
        return
    try:
        import ctypes

        # PR_SET_PDEATHSIG: kill even a busy search if its owner disappears.
        ctypes.CDLL(None, use_errno=True).prctl(1, signal.SIGKILL, 0, 0, 0)
    except (AttributeError, OSError):
        pass
    # Cover the race where the parent died before prctl was installed.
    if os.getppid() != parent_pid:
        os._exit(1)


def _worker(connection, parent_pid: int) -> None:
    worker_entry_mono_ns = time.monotonic_ns()
    for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
                 'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
        os.environ[name] = '1'
    _parent_death_guard(parent_pid)
    try:
        while True:
            try:
                receive_started = time.monotonic_ns()
                # Receive before importing the planner so submit only transfers
                # a small request and never waits for the search itself.
                job_id, request = connection.recv()
            except EOFError:
                return
            started = time.monotonic()
            cpu_started = time.process_time()
            response = {'id': job_id, 'worker_pid': os.getpid(),
                        'worker_entry_mono_ns': worker_entry_mono_ns,
                        'request_received_mono_ns': time.monotonic_ns(),
                        'worker_idle_receive_ms': (time.monotonic_ns()-receive_started)/1e6}
            try:
                response['route'] = _compute(request, telemetry=response)
            except Exception as exc:
                response['error'] = f'{type(exc).__name__}: {exc}'
            response['elapsed_sec'] = time.monotonic() - started
            response['worker_cpu_ms'] = (time.process_time()-cpu_started)*1000
            response['compute_done_mono_ns'] = time.monotonic_ns()
            try:
                connection.send(response)
            except (BrokenPipeError, EOFError, OSError):
                return
    finally:
        _close_gpu()
        connection.close()


class RoutePlannerProcess:
    """A lazy, reusable spawn worker with at most one outstanding request."""

    def __init__(self, *, join_timeout: float = .5):
        self._context = multiprocessing.get_context('spawn')
        self._join_timeout = max(.01, float(join_timeout))
        self._process = None
        self._connection = None
        self._pending_id = None
        self._submitted_mono = None
        self._next_id = 1
        self._closed = False

    @profiled('route.worker_start', 'process_start')
    def _start(self) -> None:
        if self._process is not None and self._process.is_alive():
            return
        self._dispose()
        parent, child = self._context.Pipe(duplex=True)
        process = self._context.Process(
            target=_worker, args=(child, os.getpid()),
            name='lifter-route-planner', daemon=True,
        )
        try:
            process.start()
        except BaseException:
            parent.close()
            child.close()
            raise
        child.close()
        self._connection, self._process = parent, process

    @profiled('route.submit', 'host_mixed')
    def submit(self, request: dict) -> int:
        if self._closed:
            raise RuntimeError('route planner is closed')
        if self._pending_id is not None:
            raise RuntimeError('route planner already has a pending request')
        if not isinstance(request, dict):
            raise TypeError('planning request must be a dict')
        # Validate serialization before launching a process and freeze mutable
        # parent data before handing ownership to the IPC connection.
        with span('route.request_serialize', 'host_work'):
            payload = pickle.loads(pickle.dumps(request, protocol=pickle.HIGHEST_PROTOCOL))
        self._start()
        job_id = self._next_id
        try:
            with span('route.ipc_send', 'ipc_wait'):
                self._connection.send((job_id, payload))
        except BaseException:
            self._dispose()
            raise
        self._next_id += 1
        self._pending_id = job_id
        self._submitted_mono = time.monotonic()
        return job_id

    def poll(self):
        if self._pending_id is None:
            return None
        try:
            if self._connection.poll():
                with span('route.ipc_receive', 'ipc_wait'):
                    result = self._connection.recv()
                if not isinstance(result, dict) or result.get('id') != self._pending_id:
                    raise RuntimeError('planner returned an unexpected request id')
                self._pending_id = self._submitted_mono = None
                return result
            if self._process.is_alive():
                return None
            raise RuntimeError(f'planner worker exited (code {self._process.exitcode})')
        except (EOFError, OSError, RuntimeError) as exc:
            result = {
                'id': self._pending_id,
                'error': f'{type(exc).__name__}: {exc}',
                'elapsed_sec': time.monotonic() - self._submitted_mono,
                'worker_pid': self._process.pid,
            }
            self._pending_id = self._submitted_mono = None
            self._dispose()
            return result

    def _dispose(self) -> None:
        process, connection = self._process, self._connection
        connection_error = None
        if connection is not None:
            try:
                connection.close()
                self._connection = None
            except Exception as exc:
                connection_error = exc
        if process is not None:
            if process.is_alive():
                process.terminate()
            process.join(self._join_timeout)
            if process.is_alive():
                process.kill()
                process.join(self._join_timeout)
            if process.is_alive():
                raise RuntimeError('route worker did not stop; restart is blocked')
            process.close()
            self._process = None
        if connection_error is not None:
            raise connection_error

    def cancel(self) -> None:
        self._pending_id = self._submitted_mono = None
        self._dispose()

    def restart(self) -> None:
        """Dispose the failed child; the next submit spawns a clean process."""
        self.cancel()
        self._closed = False

    def close(self) -> None:
        self._closed = True
        self.cancel()
