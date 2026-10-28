"""Inspect hardware without opening a CAN channel or sending vehicle commands."""
import argparse
import os
from pathlib import Path
import sys
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--stream", action="store_true", help="Read a few camera frames; no CAN transmission")
args = parser.parse_args()
root = Path(__file__).resolve().parent
sys.path.insert(0, str(root / "depth_cam"))
from model_catalog import DEFAULT_MODEL, MODEL_ENV, resolve_model
os.environ[MODEL_ENV] = str(resolve_model(DEFAULT_MODEL))
from calib.fsm_v4 import config
from calib.config import STREAM_FPS
from calib.control import CAN_CHANNEL, CAN_BITRATE
failures = []

import pyrealsense2 as rs
devices = list(rs.context().query_devices())
print("RealSense cameras:", len(devices), flush=True)
if not devices:
    failures.append("Connect the RealSense camera to a USB 3 port.")
else:
    for dev in devices:
        print("Camera:", dev.get_info(rs.camera_info.name), flush=True)
    dev = devices[0]
    streams = {p.stream_type() for s in dev.query_sensors() for p in s.get_stream_profiles()}
    if config.COARSE_IMU_ENABLED and not {rs.stream.gyro, rs.stream.accel}.issubset(streams):
        failures.append("Current FSM requires a camera with gyro and accelerometer (IMU).")
    pipeline = rs.pipeline()
    camera_config = rs.config()
    camera_config.enable_device(dev.get_info(rs.camera_info.serial_number))
    fps = STREAM_FPS or 30
    camera_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, fps)
    camera_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, fps)
    try:
        camera_config.resolve(rs.pipeline_wrapper(pipeline))
        print("Camera profile resolved: 640x480 depth/color; fps", fps,
              "; IMU on demand", flush=True)
        resolved = camera_config
    except RuntimeError as exc:
        failures.append("No compatible video profile: " + str(exc))
        resolved = None
    if resolved is not None and args.stream:
        from calib.imu_on_demand import NativeVideoFrameSource, OnDemandImuSource
        video = NativeVideoFrameSource(rs.frame_queue(1))
        imu = OnDemandImuSource(rs, config.COARSE_IMU_SIGN,
                                config.COARSE_IMU_MAX_AGE_SEC)
        started = False

        def read_phase(need_imu=False):
            deadline = time.monotonic() + 8.0
            frames = 0
            while time.monotonic() < deadline:
                sample = video.poll_for_frames()
                if sample and sample.get_depth_frame() and sample.get_color_frame():
                    frames += 1
                _, received, _, error = imu.snapshot()
                if error:
                    raise RuntimeError(error)
                fresh = (received is not None and
                         0 <= time.monotonic() - received <= config.COARSE_IMU_MAX_AGE_SEC)
                counts = imu.sample_counts
                motion_ready = fresh and counts["gyro"] >= 50 and counts["accel"] >= 25
                if frames >= 10 and (not need_imu or motion_ready):
                    return
                time.sleep(.002)
            raise RuntimeError("Video/IMU startup timeout: frames=%s, motion=%s" %
                               (frames, imu.sample_counts))

        try:
            profile = pipeline.start(resolved, video.queue)
            started = True
            read_phase()
            if config.COARSE_IMU_ENABLED:
                imu.enable(profile.get_device())
                read_phase(need_imu=True)
                print("IMU_ON_OK:", imu.sample_counts, flush=True)
                imu.disable()
                read_phase()
                print("IMU_OFF_OK: video continues; motion sensor closed.", flush=True)
            print("CAMERA_STREAM_OK: video at %sfps; required on-demand IMU checked; no CAN transmission." % fps,
                  flush=True)
        except Exception as exc:
            failures.append("Camera stream failed: " + str(exc))
        finally:
            try:
                imu.close()
            except Exception as exc:
                failures.append("IMU cleanup failed: " + str(exc))
            if started:
                try:
                    pipeline.stop()
                except Exception as exc:
                    failures.append("Video cleanup failed: " + str(exc))

from canlib import canlib
print("CANlib:", canlib.dllversion(), flush=True)
count = canlib.getNumberOfChannels()
print("CAN channels:", count, "required channel:", CAN_CHANNEL, "bitrate:", CAN_BITRATE, flush=True)
for index in range(count):
    data = canlib.ChannelData(index)
    virtual = bool(data.channel_cap & canlib.ChannelCap.VIRTUAL)
    print("CAN channel", index, data.channel_name, "virtual=", virtual, flush=True)
    if index == CAN_CHANNEL and virtual:
        failures.append("The configured CAN channel is virtual; a physical Kvaser channel is required.")
if count <= CAN_CHANNEL:
    failures.append("Connect a Kvaser CAN interface; configured physical CAN channel is unavailable.")
for failure in failures:
    print("NOT_READY:", failure, flush=True)
print("No CAN channel opened, no busOn and no vehicle commands sent.", flush=True)
if failures:
    raise SystemExit(2)
print("DEVICE_PREFLIGHT_OK. CAN wiring/termination/vehicle response still require an on-site check.", flush=True)
