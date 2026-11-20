"""Raw RealSense motion acquisition with a native queue and bounded lifetime.

Video configuration and vehicle state belong to the caller.  This class never
opens a video stream, resets the camera, or sends vehicle commands.
"""
import math
import threading
import time

from .imu_stream import ImuVideoStream


class NativeVideoFrameSource:
    """Adapt a video-only native queue to the runtime's frameset interface."""

    def __init__(self, native_queue):
        self.queue = native_queue

    def poll_for_frames(self):
        latest = None
        while True:
            frame = self.queue.poll_for_frame()
            if not frame:
                return latest
            frames = frame.as_frameset()
            if frames:
                latest = frames

    def wait_for_frames(self, timeout_ms=5000):
        return self.queue.wait_for_frame(timeout_ms).as_frameset()


class OnDemandImuSource:
    """Integrate every gyro frame while motion is explicitly enabled.

    ``enable`` opens/starts the native sensor, then returns without waiting for a
    first sample.  The caller must remain stopped until ``snapshot`` is ready.
    Snapshots retain the existing ``(yaw, received_mono, rate, error)`` contract.
    """

    def __init__(self, rs, sign=1.0, max_gap=0.25, startup_settle_sec=0.3):
        self.rs = rs
        self.sign = sign
        self.max_gap = max_gap
        if not math.isfinite(startup_settle_sec) or startup_settle_sec < 0:
            raise ValueError("Invalid IMU startup settling time")
        self.startup_settle_sec = startup_settle_sec
        self._stream = ImuVideoStream(rs, sign, max_gap)
        self._lifecycle_lock = threading.Lock()
        self._counts_lock = threading.Lock()
        self._counts = {"gyro": 0, "accel": 0}
        self._sensor = None
        self._queue = None
        self._worker = None
        self._stop = None
        self._opened = False
        self._started = False
        self._active = False
        self._generation = 0
        self._global_time_restore = None

    @property
    def active(self):
        return self._active

    @property
    def sample_counts(self):
        with self._counts_lock:
            return dict(self._counts)

    def snapshot(self):
        return self._stream.snapshot()

    @staticmethod
    def _set_error(stream, message):
        with stream.lock:
            if stream.error is None:
                stream.error = str(message)

    def _select_sensor(self, device):
        for sensor in device.query_sensors():
            profiles = list(sensor.get_stream_profiles())
            selection = []
            for kind, preferred in ((self.rs.stream.gyro, 200),
                                    (self.rs.stream.accel, 100)):
                candidates = [p for p in profiles if p.stream_type() == kind
                              and p.format() == self.rs.format.motion_xyz32f]
                if not candidates:
                    break
                # Prefer the requested rate, then the nearest actual supported
                # rate (e.g. older BMI055 accel profiles expose 63/250 Hz).
                selection.append(min(candidates, key=lambda p: (
                    abs(float(p.fps()) - preferred), -float(p.fps()))))
            if len(selection) == 2:
                return sensor, selection
        raise RuntimeError("RealSense gyro and accelerometer profiles unavailable")

    def enable(self, device):
        """Start one activation, returning True; an active call is a no-op.

        A failed start is cleaned up before raising.  A previous worker or sensor
        that could not be closed must be disabled before a new activation.
        """
        with self._lifecycle_lock:
            if self._active:
                return False
            if self._worker is not None and self._worker.is_alive():
                raise RuntimeError("Previous IMU worker has not stopped")
            if self._sensor is not None:
                raise RuntimeError("Previous IMU sensor has not been closed")
            self._generation += 1
            generation = self._generation
            stream = ImuVideoStream(self.rs, self.sign, self.max_gap)
            self._stream = stream
            with self._counts_lock:
                self._counts = {"gyro": 0, "accel": 0}
            try:
                sensor, profiles = self._select_sensor(device)
                self._sensor = sensor
                option = getattr(getattr(self.rs, "option", None),
                                 "global_time_enabled", None)
                if option is not None and sensor.supports(option):
                    original = float(sensor.get_option(option))
                    if original != 0.0:
                        # Motion must keep one hardware-clock domain throughout
                        # integration.  SDK global-time synchronization can
                        # otherwise switch/adjust timestamps after startup.
                        # Record before writing so partial failures restore too.
                        self._global_time_restore = (option, original)
                        sensor.set_option(option, 0.0)
                # Keep the native queue alive until the sensor and worker stop.
                self._queue = self.rs.frame_queue(2048)
                self._stop = threading.Event()
                # Mark attempts before invoking the API: an exception may occur
                # after the native side partially acquired its resources.
                self._opened = True
                sensor.open(profiles)
                self._started = True
                activated_mono = time.monotonic()
                sensor.start(self._queue)
                self._active = True
                self._worker = threading.Thread(
                    target=self._run,
                    args=(generation, stream, self._queue, self._stop,
                          activated_mono),
                    name="realsense-imu", daemon=True,
                )
                self._worker.start()
                return True
            except Exception as exc:
                self._active = False
                self._set_error(stream, f"IMU startup failed: {exc}")
                try:
                    self._disable_locked()
                except Exception:
                    # Preserve the original startup error and retained handles.
                    pass
                raise RuntimeError(f"IMU startup failed: {exc}") from exc

    def _run(self, generation, stream, native_queue, stop, activated_mono):
        first_gyro_ms = None
        # D435I HID may deliver several cached frames from the previous open,
        # followed by a large timestamp jump. Discard a stopped startup interval
        # and drain it completely before making any sample available to the FSM.
        prime_until = activated_mono + self.startup_settle_sec
        capture_anchor = activated_mono if self.startup_settle_sec == 0 else None
        try:
            while not stop.is_set() and generation == self._generation:
                frame = native_queue.poll_for_frame()
                if not frame:
                    if capture_anchor is None and time.monotonic() >= prime_until:
                        capture_anchor = time.monotonic()
                    stop.wait(0.002)
                    continue
                if stop.is_set() or generation != self._generation:
                    break
                if capture_anchor is None:
                    continue
                kind = frame.get_profile().stream_type()
                if kind == self.rs.stream.gyro:
                    stamp_ms = float(frame.get_timestamp())
                    if first_gyro_ms is None:
                        first_gyro_ms = stamp_ms
                    # Keep a fixed hardware -> monotonic mapping after priming.
                    # Later backlogs keep their age instead of receiving a new
                    # host timestamp when the worker dequeues them.
                    received = min(time.monotonic(), capture_anchor
                                   + (stamp_ms - first_gyro_ms) / 1000.0)
                    rate = float(frame.as_motion_frame().get_motion_data().y)
                    if not math.isfinite(stamp_ms) or not math.isfinite(rate):
                        self._set_error(stream, "non-finite IMU sample")
                        return
                    stream.feed_gyro(rate, stamp_ms, received)
                    label = "gyro"
                elif kind == self.rs.stream.accel:
                    label = "accel"
                else:
                    continue
                with self._counts_lock:
                    # A stopped generation must never affect the next counts.
                    if stop.is_set() or generation != self._generation:
                        break
                    self._counts[label] += 1
                if stream.snapshot()[3] is not None:
                    return
        except Exception as exc:
            if not stop.is_set() and generation == self._generation:
                self._set_error(stream, f"IMU acquisition failed: {exc}")

    def _disable_locked(self):
        self._active = False
        if self._stop is not None:
            self._stop.set()
        errors = []
        if self._sensor is not None and self._started:
            try:
                self._sensor.stop()
                self._started = False
            except Exception as exc:
                errors.append(f"stop: {exc}")
        worker = self._worker
        if worker is not None:
            # A thread that failed to start has no ident and cannot be joined.
            if worker.ident is not None:
                worker.join(timeout=1.0)
            if worker.is_alive():
                errors.append("IMU worker did not stop")
        if (worker is None or not worker.is_alive()) and self._sensor is not None:
            if self._opened:
                try:
                    self._sensor.close()
                    self._opened = self._started = False
                except Exception as exc:
                    errors.append(f"close: {exc}")
            if not self._opened and not self._started:
                if self._global_time_restore is not None:
                    option, original = self._global_time_restore
                    try:
                        self._sensor.set_option(option, original)
                        self._global_time_restore = None
                    except Exception as exc:
                        errors.append(f"restore motion global time: {exc}")
                if self._global_time_restore is None:
                    self._sensor = None
                    self._queue = None
                    self._worker = None
                    self._stop = None
        with self._stream.lock:
            self._stream.received = None
        if errors:
            reason = "IMU shutdown failed: " + "; ".join(errors)
            self._set_error(self._stream, reason)
            raise RuntimeError(reason)

    def disable(self):
        """Stop acquisition and invalidate freshness; safe to call repeatedly."""
        with self._lifecycle_lock:
            self._disable_locked()

    close = disable
