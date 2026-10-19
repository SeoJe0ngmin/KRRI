"""IMU delivery cadence straight from the RealSense SDK, independent of the docking app (2026-10-19).

Why: the bridge log shows IMU samples pausing 1-1.5 s and then arriving in a burst while device stamps stay
continuous. This measures the host arrival gaps of gyro/accel callbacks under three loads, so the pause can be
pinned on the SDK/USB path or on the app.

  python scripts/check_imu_cadence.py imu            # gyro+accel only
  python scripts/check_imu_cadence.py ir             # + IR left/right 640x480@30
  python scripts/check_imu_cadence.py full           # + colour 1280x720@30 (the docking configuration, depth off)
  python scripts/check_imu_cadence.py full --depth   # + depth 640x480@30
Options: --seconds 60 --gyro 200 --accel 100
Prints, per stream: samples, worst host gap, number of gaps > 100 ms / > 500 ms, and the device-time gap at the
worst host gap (continuous device time + large host gap = delivered late, not lost).
"""
import argparse
import threading
import time

import numpy as np
import pyrealsense2 as rs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('load', choices=['imu', 'ir', 'full'])
    ap.add_argument('--depth', action='store_true')
    ap.add_argument('--seconds', type=float, default=60.)
    ap.add_argument('--gyro', type=int, default=200)
    ap.add_argument('--accel', type=int, default=100)
    a = ap.parse_args()
    ctx = rs.context()
    stats = {k: dict(host=[], dev=[], last_host=None, last_dev=None, n=0) for k in ('gyro', 'accel')}
    lock = threading.Lock()

    def motion(frame):
        if not frame.is_motion_frame():
            return
        f = frame.as_motion_frame()
        kind = 'gyro' if f.get_profile().stream_type() == rs.stream.gyro else 'accel'
        now = time.monotonic(); dev = f.get_timestamp() / 1000.
        with lock:
            s = stats[kind]; s['n'] += 1
            if s['last_host'] is not None:
                s['host'].append(now - s['last_host']); s['dev'].append(dev - s['last_dev'])
            s['last_host'] = now; s['last_dev'] = dev

    ipipe = rs.pipeline(ctx); ic = rs.config()
    ic.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, a.gyro)
    ic.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, a.accel)
    ipipe.start(ic, motion)                       # IMU first (RSUSB: whoever opens the IMU first owns it)
    vpipe = None; video_frames = [0]
    if a.load != 'imu':
        vpipe = rs.pipeline(ctx); vc = rs.config()
        for index in (1, 2):
            vc.enable_stream(rs.stream.infrared, index, 640, 480, rs.format.y8, 30)
        if a.load == 'full':
            vc.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        if a.depth:
            vc.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        vp = vc.resolve(vpipe)
        for sensor in vp.get_device().query_sensors():
            if sensor.supports(rs.option.emitter_enabled):
                sensor.set_option(rs.option.emitter_enabled, 0.)
        vpipe.start(vc)
    print('load=%s depth=%s: measuring %.0f s ...' % (a.load, a.depth, a.seconds), flush=True)
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < a.seconds:
            if vpipe is not None:
                fs = vpipe.poll_for_frames()         # keep the video pipeline drained like the app does
                if fs:
                    video_frames[0] += 1
                else:
                    time.sleep(.002)
            else:
                time.sleep(.01)
    finally:
        ipipe.stop()
        if vpipe is not None:
            vpipe.stop()
    elapsed = time.monotonic() - t0
    print('video frames %d (%.1f fps)' % (video_frames[0], video_frames[0] / elapsed))
    with lock:
        for kind, s in stats.items():
            h = np.array(s['host']); d = np.array(s['dev'])
            if not len(h):
                print('%-5s: no samples' % kind); continue
            i = int(np.argmax(h))
            print('%-5s: %5d samples (%.0f Hz) | host gap p50 %.1f ms p99 %.1f ms max %.0f ms | >100 ms: %d, >500 ms: %d | device gap at worst host gap %.1f ms'
                  % (kind, s['n'], s['n'] / elapsed, 1000 * np.median(h), 1000 * np.percentile(h, 99), 1000 * h.max(),
                     int((h > .1).sum()), int((h > .5).sum()), 1000 * d[i]))
            burst = [(round(t, 1), round(1000 * g)) for t, g in zip(np.cumsum(h), h) if g > .3]
            if burst:
                print('       gaps > 300 ms at (s, ms): %s' % burst[:20])


if __name__ == '__main__':
    main()
