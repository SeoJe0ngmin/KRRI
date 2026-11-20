# main_rec.py (실행 즉시 자동 녹화 시작 버전: 첫 프레임 생성 시 VideoWriter 자동 초기화)

import sys
import cv2
import numpy as np
try:
    import pyrealsense2 as rs
except ImportError:
    rs = None
import os
import math
import time
import hashlib
from datetime import datetime
from calib import config as runtime_config
from calib.performance import span, profiled

from calib.config import (
    STREAM_W, STREAM_H, STREAM_FPS, REC_FPS_PROBE_FRAMES,
    TRACE_ENABLE, TRACE_STATE_HZ, MODEL_PATH, POSE_FACE_KPTS, POSE_CENTER_KPT,
    POSE_BACK_KPTS, POSE_USE_MULTI_KPT, POSE_MULTI_MIN_FRONT,
    POSE_KPT_VIS_THR,
    PALLET_FACE_W, PALLET_WIDTH_M, PALLET_LENGTH_M, PALLET_HEIGHT_M,
    COLOR_ALERT, COLOR_STATUS_OK, COLOR_STATUS_TRK, COLOR_META, COLOR_BOX, COLOR_CNT, COLOR_CENTER,
    COLOR_YAW, COLOR_OFFSET, COLOR_WIDTH,
)
from calib.camera_overlay import draw_camera_overlay
from calib.hud import draw_panel
from calib.runtime_view import (compose_runtime_view as _compose_runtime_view,
    screen_work_area_limit as _screen_work_area_limit, fit_view_for_display as _fit_view_for_display)
from calib.perception import Perception
from calib.geometry import pose_from_visible_kpts_pnp
from calib.tracelog import TraceLogger
from calib.fsm import CalibrationFSM
from calib.control import (
    can_init, can_close, configure_can_enabled, configure_can_tx_observer,
    get_can_status,
    set_runtime_stop_latched,
)
from calib.initialisation import InitialisationError, run_initialisation
from calib.utils import fmt_deg, fmt_m
from ui.diagram import draw_fsm_diagram_panel


RECORDING_POST_DONE_SEC = 5.0


class _RecordingCompletionTimer:
    def __init__(self):
        self.deadline = None
        self.stopped = False

    def update(self, state, now):
        """Return True once, five seconds after insertion completion."""
        if self.deadline is None and state == "DONE":
            self.deadline = now + RECORDING_POST_DONE_SEC
        if not self.stopped and self.deadline is not None and now >= self.deadline:
            self.stopped = True
            return True
        return False


def _file_sha256(path: str) -> str:
    """Stable inference-weight identity for fit provenance."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def setup_video_writer_filename(prefix="forklift_recording"):
    """(합성화면, raw원본) 두 경로를 같은 타임스탬프로 반환"""
    rec_dir = "./rec"
    os.makedirs(rec_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(rec_dir, f"{prefix}_{ts}")
    return f"{base}.mp4", f"{base}_raw.mp4"


def list_common_fps(width: int, height: int):
    """depth(z16)와 color(bgr8)가 해당 해상도에서 공통으로 지원하는 fps 를 내림차순으로."""
    dfps, cfps = set(), set()
    try:
        for dev in rs.context().query_devices():
            for sensor in dev.query_sensors():
                for prof in sensor.get_stream_profiles():
                    try:
                        vp = prof.as_video_stream_profile()
                    except Exception:
                        continue
                    if vp is None or vp.width() != width or vp.height() != height:
                        continue
                    if prof.stream_type() == rs.stream.depth and prof.format() == rs.format.z16:
                        dfps.add(int(prof.fps()))
                    elif prof.stream_type() == rs.stream.color and prof.format() == rs.format.bgr8:
                        cfps.add(int(prof.fps()))
    except Exception as e:
        print(f"⚠️  지원 fps 조회 실패({e}) — 기본 후보로 진행")
    common = sorted(dfps & cfps, reverse=True)
    return common if common else [60, 30, 15, 6]


def realsense_check_or_exit():
    if rs is None:
        print("RealSense SDK unavailable. Install pyrealsense2 or set CAMERA_ENABLED = False.")
        sys.exit(1)
    ctx = rs.context()
    devs = ctx.query_devices()
    if devs.size() == 0:
        print("❌ RealSense 카메라를 찾을 수 없습니다. USB3 포트/케이블을 확인하세요.")
        sys.exit(1)
    print(f"✅ RealSense 장치 {devs.size()}대 연결됨.")
    for d in devs:
        name = d.get_info(rs.camera_info.name) if d.supports(rs.camera_info.name) else "Unknown"
        sn   = d.get_info(rs.camera_info.serial_number) if d.supports(rs.camera_info.serial_number) else "N/A"
        print(f"  - {name} (S/N {sn})")








def _sync_on_demand_imu(fsm, source, device):
    """Switch only at stopped coarse boundaries; the source owns all gyro reads."""
    needed = bool(getattr(fsm, "imu_active_required", False))
    if needed == source.active:
        return
    fsm._exec("STOP")
    if needed:
        if getattr(fsm, "state", None) != "COARSE_PREPARE":
            raise RuntimeError("IMU activation is only permitted during COARSE_PREPARE")
        source.enable(device)
        print("[IMU] ON: initial correction; video stays at 30fps", flush=True)
    else:
        source.disable()
        print("[IMU] OFF: normal camera/inference", flush=True)


class _FSMStageDebugGate:
    """Run FSM v4 from one stopped high-level checkpoint to the next."""

    ACTIVE_ACTION_STATES = frozenset({
        "FACE_ROTATE", "STANDOFF_MOVE", "RECENTER_ROTATE",
        "WAYPOINT_TURN", "WAYPOINT_DRIVE", "FINAL_ROTATE",
        "INSERT_DRIVE",
    })
    MOTION_ORIGIN_STATES = ACTIVE_ACTION_STATES | frozenset({"SEARCH_SWEEP", "COARSE_PREPARE"})
    CHECKPOINT_STATES = frozenset({
        "COARSE_PREPARE", "COARSE_REACQUIRE",
        "PRECHECK", "SEARCH_SWEEP", "ACQUIRE_VERIFY", "RECOVER_VISUAL",
        "FACE_ROTATE", "STANDOFF_MOVE", "RECENTER_ROTATE",
        "WAYPOINT_TURN", "WAYPOINT_DRIVE", "FINAL_ROTATE",
        "INSERT_DRIVE",
        "FACE_SETTLE", "STANDOFF_VERIFY", "STANDOFF_SETTLE",
        "STAGING_PLAN", "RECENTER_SETTLE", "WAYPOINT_TURN_SETTLE",
        "WAYPOINT_DRIVE_SETTLE", "FINAL_POSE_LOCK", "FINAL_SETTLE",
        "READY_TO_INSERT", "INSERT_SETTLE", "DONE", "FAILED",
    })
    TERMINAL_STATES = frozenset({"DONE", "FAILED"})
    TIMER_ATTRS = (
        "_coarse_started", "_coarse_imu_ready_mono",
        "_pipeline_started_mono", "_state_entered_mono",
        "_state_deadline_mono", "_invalid_since_mono",
        "_search_started_mono", "_alignment_started_mono",
        "_translation_started_mono", "_translation_deadline_mono",
        "_settle_started_mono", "_settle_evaluation_started_mono",
        "_insertion_started_mono",
        "_insertion_fine_started_mono",
    )

    def __init__(self, fsm, enabled=False):
        self.fsm = fsm
        self.enabled = bool(enabled)
        self.running = not self.enabled
        self.origin_state = None
        self.left_origin = False
        self.frames_running = 0
        self.completed_steps = 0
        self.pause_started_mono = time.monotonic() if self.enabled else None
        self._publish()
        if self.enabled:
            self._set_movement_enabled(False)
            self._stop()
            self._log("pause", initial=True, completed_steps=0)

    def _publish(self):
        self.fsm.debug_step_enabled = self.enabled
        self.fsm.debug_step_running = self.running
        self.fsm.debug_step_origin = self.origin_state
        self.fsm.debug_step_count = self.completed_steps

    def _stop(self):
        stop = getattr(self.fsm, "_exec", None)
        if callable(stop):
            stop("STOP")

    def _set_movement_enabled(self, enabled):
        executor = getattr(self.fsm, "execu", None)
        setter = getattr(executor, "set_movement_enabled", None)
        if callable(setter):
            setter(enabled)

    def _log(self, action, **details):
        tracer = getattr(self.fsm, "tracer", None)
        logger = getattr(tracer, "log_debug_step", None)
        if callable(logger):
            logger(action, getattr(self.fsm, "state", "UNKNOWN"), **details)

    def _shift_fsm_timers(self, paused_sec):
        if paused_sec <= 0.0:
            return
        for attr in self.TIMER_ATTRS:
            value = getattr(self.fsm, attr, None)
            if value is not None:
                setattr(self.fsm, attr, float(value) + paused_sec)
        rotation = getattr(self.fsm, "_rotation", None)
        if rotation is not None and getattr(rotation, "active", False):
            rotation.started_mono = float(rotation.started_mono) + paused_sec
            last_measurement = getattr(rotation, "last_measurement_mono", None)
            if last_measurement is not None:
                rotation.last_measurement_mono = (
                    float(last_measurement) + paused_sec
                )
        # 동작 체크포인트에서 기다린 시간은 해당 동작 로그 시간에서도 뺀다.
        tracer = getattr(self.fsm, "tracer", None)
        open_step = getattr(tracer, "_open_step", None)
        if isinstance(open_step, dict) and open_step.get("t_mono") is not None:
            open_step["t_mono"] = float(open_step["t_mono"]) + paused_sec

    def _reset_perception_window(self):
        pose_filter = getattr(self.fsm, "_pose_filter", None)
        reset = getattr(pose_filter, "reset", None)
        if callable(reset):
            reset()
        samples = getattr(self.fsm, "_samples", None)
        if hasattr(samples, "clear"):
            samples.clear()
        for attr in (
            "_last_valid_pose", "_last_valid_center", "_last_valid_margin",
            "_last_valid_vision_meta", "_invalid_since_mono",
            "_approved_stopped_observation",
        ):
            if hasattr(self.fsm, attr):
                setattr(self.fsm, attr, None)
        if hasattr(self.fsm, "_invalid_frames"):
            self.fsm._invalid_frames = 0

    def request_step(self):
        if not self.enabled or self.running:
            return False
        state = getattr(self.fsm, "state", "UNKNOWN")
        if state in self.TERMINAL_STATES:
            return False
        now = time.monotonic()
        paused_sec = max(0.0, now - (self.pause_started_mono or now))
        self._shift_fsm_timers(paused_sec)
        self._reset_perception_window()
        # 판단 단계의 SPACE는 다음 동작을 계획만 한다. FITTED FORWARD 같은
        # 실제 동작 화면에서 다시 누른 SPACE만 차량 동작을 허용한다.
        movement_enabled = state in self.MOTION_ORIGIN_STATES
        self._set_movement_enabled(movement_enabled)
        resume_motion = getattr(self.fsm, 'resume_debug_motion', None)
        if movement_enabled and callable(resume_motion):
            resume_motion()
        self.running = True
        self.origin_state = state
        self.left_origin = False
        self.frames_running = 0
        self._publish()
        self._log(
            "resume", origin_state=state,
            paused_sec=round(paused_sec, 4),
            movement_enabled=movement_enabled,
            requested_step=self.completed_steps + 1,
        )
        print(
            f"[DEBUG STEP] RUN #{self.completed_steps + 1}: {state}",
            flush=True,
        )
        return True

    @property
    def should_step(self):
        return not self.enabled or self.running

    def after_step(self, previous_state):
        if not self.enabled or not self.running:
            return
        self.frames_running += 1
        current = getattr(self.fsm, "state", "UNKNOWN")
        if current != self.origin_state:
            self.left_origin = True
        complete = current in self.TERMINAL_STATES
        complete = complete or (
            self.left_origin and current in self.CHECKPOINT_STATES
        )
        # READY may deliberately hold when automatic insertion is disabled.
        complete = complete or (
            self.origin_state == "READY_TO_INSERT"
            and current == "READY_TO_INSERT"
            and self.frames_running >= 1
        )
        if not complete:
            return
        self.completed_steps += 1
        self.running = False
        self.pause_started_mono = time.monotonic()
        self._set_movement_enabled(False)
        self._stop()
        self._publish()
        self._log(
            "pause", origin_state=self.origin_state,
            reached_state=current, frames_running=self.frames_running,
            completed_steps=self.completed_steps,
        )
        print(
            f"[DEBUG STEP] PAUSE #{self.completed_steps}: "
            f"{self.origin_state} -> {current}",
            flush=True,
        )

    def hud_lines(self):
        if not self.enabled:
            return []
        state = getattr(self.fsm, "state", "UNKNOWN")
        if self.running:
            return [(
                f"[DEBUG STEP] RUNNING #{self.completed_steps + 1}: "
                f"{self.origin_state} -> next checkpoint",
                COLOR_STATUS_TRK,
            )]
        if state in self.TERMINAL_STATES:
            return [(
                f"[DEBUG STEP] terminal state: {state}", COLOR_ALERT,
            )]
        return [(
            f"[DEBUG STEP] PAUSED at {state} | press SPACE to run one stage",
            COLOR_STATUS_OK,
        )]


def _run_camera_off_preview(
    fsm_factory, diagram_drawer, window_title, camera_display_scale,
    interface_panel_width, fsm_panel_width,
):
    """Display the real HUD without inventing camera observations or motion."""
    configure_can_enabled(False)
    configure_can_tx_observer(None)
    print("[CAMERA OFF] UI preview: no camera/inference/recording; FSM paused, CAN OFF. ESC exits.")
    try:
        fsm = fsm_factory(tracer=None)
        display_limit = _screen_work_area_limit()
        vis = np.full((STREAM_H, STREAM_W, 3), (25, 28, 31), dtype=np.uint8)
        cv2.putText(vis, "CAMERA OFF", (35, 75), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (90, 220, 255), 2, cv2.LINE_AA)
        cv2.putText(vis, "No camera connected / UI preview", (35, 115),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 225, 230), 1, cv2.LINE_AA)
        lines = [
            ("CAMERA OFF | UI PREVIEW", COLOR_STATUS_OK),
            ("FSM paused: no camera observations", COLOR_META),
            ("CAN OFF | Inference OFF | Recording OFF", COLOR_META),
            ("Enable CAMERA_ENABLED in calib/config.py", COLOR_META),
            ("Restart to apply. ESC: exit", COLOR_META),
        ]
        while True:
            show = _compose_runtime_view(
                vis, fsm, diagram_drawer, camera_display_scale,
                interface_lines=lines, cmd_status=fsm.cmd_status,
                interface_panel_width=interface_panel_width,
                fsm_panel_width=fsm_panel_width,
            )
            cv2.imshow(window_title, _fit_view_for_display(show, display_limit))
            if cv2.waitKey(33) & 0xFF == 27:
                break
            if cv2.getWindowProperty(window_title, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        can_close()
        cv2.destroyAllWindows()


def main(
    fsm_factory=CalibrationFSM,
    recording_prefix="forklift_recording",
    window_title="Forklift HUD + FSM",
    fsm_version="v1",
    diagram_drawer=draw_fsm_diagram_panel,
    can_enabled=True,
    camera_display_scale=1.0,
    debug_step_mode=False,
    interface_panel_width=640,
    fsm_panel_width=900,
    camera_enabled=None,
    demo_mode=False,
    threaded_ui=True,
    _presentation=None,
    no_display=False,
    monitor_port=0,
    _demo_monitor=None,
    _performance_paths=None,
):
    from calib import performance as _perf
    if TRACE_ENABLE and _perf.active() is None and _perf.enabled(camera_enabled, demo_mode):
        options = locals().copy()
        options.pop('_perf', None)
        return _perf.traced_runtime(main)(**options)
    monitor_port = int(monitor_port)
    if not 0 <= monitor_port <= 65535:
        raise ValueError("Remote monitor port must be between 0 and 65535")
    if monitor_port and not no_display:
        raise ValueError("Remote monitor requires no_display=True")
    if no_display and not (runtime_config.CAMERA_ENABLED if camera_enabled is None else camera_enabled):
        raise ValueError("No-display real execution requires camera input")
    if no_display and demo_mode and monitor_port and _demo_monitor is None:
        # Demo stays headless: expose only status/stop, never render or record.
        options = locals().copy()
        options.pop('_perf', None)
        from calib.remote_monitor import RemoteMonitor
        monitor = RemoteMonitor(monitor_port, demo_mode=True).start()
        options.update(monitor_port=0, _demo_monitor=monitor)
        try:
            return main(**options)
        finally:
            monitor.close()
    if (threaded_ui or (no_display and _presentation is None)) and not demo_mode and (runtime_config.CAMERA_ENABLED if camera_enabled is None else camera_enabled):
        options = locals().copy()
        options.pop('_perf', None)
        from calib.runtime_threads import RuntimePresentation
        presentation = RuntimePresentation(
            stop_motion=lambda: set_runtime_stop_latched(True),
            reset_motion=lambda: set_runtime_stop_latched(False),
        )
        options.update(threaded_ui=False, _presentation=presentation)
        display_limit = None if no_display else _screen_work_area_limit()
        monitor = None
        if monitor_port and not presentation.process_ui:
            from calib.remote_monitor import RemoteMonitor
            monitor = RemoteMonitor(monitor_port).start()
            print(f"[REMOTE MONITOR] {monitor.url} (viewer disconnect does not stop execution)", flush=True)

        if presentation.process_ui:
            options.update(monitor_port=0)

        def render(packet):
            vis = packet['image']
            if packet['overlay'] is not None:
                draw_camera_overlay(vis, **packet['overlay'])
            show = _compose_runtime_view(
                vis, packet['fsm'], diagram_drawer, camera_display_scale,
                interface_lines=packet['lines'], cmd_status=packet['fsm'].cmd_status,
                interface_panel_width=interface_panel_width, fsm_panel_width=fsm_panel_width,
                prominent_yaw_deg=packet['yaw'], prominent_z_distance_m=packet['z'],
            )
            return show

        try:
            return presentation.run(
                control=lambda: main(**options), render=render,
                show=(monitor.publish if monitor is not None else
                      (lambda image: None) if no_display else
                      lambda image: cv2.imshow(window_title, _fit_view_for_display(image, display_limit))),
                read_key=(monitor.read_key if monitor is not None else
                          (lambda: -1) if no_display else
                          lambda: cv2.waitKey(1) & 0xFF),
                close_windows=(lambda: None) if no_display else cv2.destroyAllWindows,
                ui_spec=(dict(diagram_module=diagram_drawer.__module__, diagram_name=diagram_drawer.__name__,
                    camera_display_scale=camera_display_scale, interface_panel_width=interface_panel_width,
                    fsm_panel_width=fsm_panel_width, window_title=window_title, no_display=no_display,
                    display_limit=display_limit, monitor_port=monitor_port, frame_shape=(STREAM_H, STREAM_W, 3))
                    if presentation.process_ui else None),
            )
        finally:
            if monitor is not None:
                print(f"[REMOTE MONITOR STATS] {monitor.telemetry}", flush=True)
                monitor.close()
    if _presentation is None:
        set_runtime_stop_latched(False)
    demo_mode = bool(demo_mode)
    trace_enabled = TRACE_ENABLE and not demo_mode
    if demo_mode:
        debug_step_mode = False
        print("[DEMO] No GUI, recording or file logs. Ctrl+C stops; DONE/FAILED exits.")
    camera_display_scale = float(camera_display_scale)
    if camera_display_scale <= 0.0:
        raise ValueError("camera_display_scale must be positive")
    if camera_enabled is None:
        camera_enabled = runtime_config.CAMERA_ENABLED
    if not camera_enabled:
        if demo_mode:
            raise ValueError("Demo mode requires camera input")
        return _run_camera_off_preview(
            fsm_factory, diagram_drawer, window_title, camera_display_scale,
            interface_panel_width, fsm_panel_width,
        )
    with _perf.span('startup.camera_presence', 'device_wait'):
        realsense_check_or_exit()
    display_limit = None if demo_mode or _presentation else _screen_work_area_limit()

    # CAN 초기화
    configure_can_enabled(can_enabled)
    if _presentation is not None:
        _presentation.cleanups.append(can_close)
    if can_enabled:
        print("CAN 통신 초기화 중...")
        with _perf.span('startup.can_init', 'device_wait'):
            can_ready = can_init()
        if not can_ready:
            print("❌ CAN 통신 초기화 실패: 안전을 위해 실행을 중단합니다.")
            return
        print("✅ CAN 통신 초기화 성공(하트비트 자동)")
    else:
        can_init()
        print("🖥️ CAN OFF: 카메라/FSM/녹화만 실행하며 차량 명령은 송신하지 않습니다.")

    # 모듈
    # Start the driving FSM only after the fork reaches its initial pose.
    # CAN OFF runs the same timed sequence but control.py blocks every write.
    try:
        with _perf.span('startup.fork_initialisation', 'intentional_wait'):
            run_initialisation()
    except KeyboardInterrupt:
        print("[INITIALISATION] interrupted; driving FSM will not start.")
        can_close()
        return
    except InitialisationError as exc:
        print(f"[INITIALISATION ERROR] {exc}")
        print("Driving FSM will not start for safety.")
        can_close()
        return
    except Exception as exc:
        print(
            "[INITIALISATION ERROR] unexpected failure: "
            f"{type(exc).__name__}: {exc}"
        )
        print("Driving FSM will not start for safety.")
        can_close()
        return

    with _perf.span('startup.model_load_warmup', 'host_mixed'):
        perception = Perception()

    # === 로깅/녹화 파일 베이스 (영상과 로그가 같은 타임스탬프를 공유) ===
    video_filename, raw_filename = (("", "") if demo_mode else
                                      (_performance_paths or setup_video_writer_filename(recording_prefix)))
    trace_base = os.path.splitext(video_filename)[0]
    tracer_kwargs = {} if _presentation is None else {"io_worker": _presentation.trace_io}
    tracer = TraceLogger(trace_base, state_hz=TRACE_STATE_HZ, enabled=trace_enabled, **tracer_kwargs)
    if _presentation is not None:
        _presentation.cleanups.append(tracer.close)
    if _presentation is not None:
        def observe_tx(**event):
            _presentation.observe_can_tx(**event)
            if trace_enabled:
                tracer.log_can_tx(**event)
        configure_can_tx_observer(observe_tx)
    else:
        configure_can_tx_observer(tracer.log_can_tx if trace_enabled else None)
    if trace_enabled:
        print(
            f"📝 시퀀스 로그: {trace_base}_meta.json / _pallet_state.csv / "
            "_inference_timing.csv / _control_seq.jsonl"
        )

    fsm = fsm_factory(tracer=None if demo_mode else tracer)
    enable_planning = getattr(fsm, 'enable_async_planning', None)
    if callable(enable_planning):
        enable_planning()
        if _presentation is not None:
            _presentation.cleanups.append(fsm.close)
    if _presentation is not None:
        _presentation.instrument_fsm(fsm, trace_base)
    debug_gate = _FSMStageDebugGate(fsm, enabled=debug_step_mode)

    # === RealSense 파이프라인 구성 ===
    pipeline = rs.pipeline()
    if _presentation is not None:
        _presentation.cleanups.append(pipeline.stop)
    imu_stream = None
    native_video = None
    camera_device = None
    if getattr(fsm, "requires_imu", False):
        from calib.imu_on_demand import OnDemandImuSource, NativeVideoFrameSource
        from calib.fsm_v4 import config as imu_config
        imu_stream = OnDemandImuSource(rs, imu_config.COARSE_IMU_SIGN,
                                       imu_config.COARSE_IMU_MAX_AGE_SEC)
        fsm.imu_source = imu_stream
        native_video = NativeVideoFrameSource(rs.frame_queue(1))
        if _presentation is not None:
            _presentation.cleanups.append(imu_stream.close)
    # 설정한 영상 FPS로 시작한다. 0이면 지원 FPS를 내림차순으로 시도한다.
    candidates = [STREAM_FPS] if STREAM_FPS else list_common_fps(STREAM_W, STREAM_H)
    print(f"📷 {STREAM_W}x{STREAM_H} 지원 fps 후보: {candidates}")

    stream_fps = None
    for f in candidates:
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, STREAM_W, STREAM_H, rs.format.z16, f)
        cfg.enable_stream(rs.stream.color, STREAM_W, STREAM_H, rs.format.bgr8, f)
        try:
            if native_video is not None:
                with _perf.span('startup.camera_stream', 'device_wait'):
                    active_profile = pipeline.start(cfg, native_video.queue)
                camera_device = active_profile.get_device()
            else:
                with _perf.span('startup.camera_stream', 'device_wait'):
                    pipeline.start(cfg)
            stream_fps = f
            streams_label = "depth+color; IMU on demand" if imu_stream is not None else "depth+color"
            print(f"✅ 스트림 시작: {STREAM_W}x{STREAM_H} @ {f}fps ({streams_label})")
            break
        except Exception as e:
            print(f"⚠️  {f}fps 시작 실패 → 다음 후보 시도: {e}")
    if stream_fps is None:
        can_close()
        print("❌ 어떤 fps 로도 스트림을 시작하지 못했습니다.")
        sys.exit(1)
    frame_source = native_video if native_video is not None else pipeline

    align = rs.align(rs.stream.color)

    # === 녹화: 실행 즉시 자동 녹화 시작 ===
    video_writer = None
    raw_writer = None             # 오버레이 없는 원본 컬러 영상
    raw_video_frame_count = 0     # 실제 *_raw.mp4 에 기록된 프레임 수
    recording = not demo_mode              # ← 기본값: True (자동 녹화)
    recording_completion = _RecordingCompletionTimer()
    if not demo_mode:
        print(f"📹 자동 녹화 대기: {video_filename}")
        print(f"📹 원본(raw) 녹화 대기: {raw_filename}")
        print(f"📹 첫 {REC_FPS_PROBE_FRAMES}프레임으로 실제 fps 를 재고 그 값으로 저장합니다")
        print("📹 'r' 녹화 토글, 'ESC' 종료")
    if debug_gate.enabled:
        print("🐞 디버그 단계 실행: 'SPACE'를 누르면 FSM 한 스테이지 수행")

    # === fps 실측 ===
    # 저장 fps 를 스트림 공칭값으로 박으면 실제 처리속도와 어긋나 재생 배속이 깨진다.
    meta_written = False
    frame_i = 0
    probe_t0 = None
    probe_i0 = 0
    rec_fps = None            # 실측 결과(저장용). None 이면 아직 녹화 시작 전
    fps_disp = 0.0            # HUD 표시용 이동 평균
    disp_t0 = time.time()
    disp_n = 0

    blind_view = np.zeros((STREAM_H, STREAM_W, 3), dtype=np.uint8)

    try:
        while True:
            _perf.iteration(fsm.state, not getattr(fsm, 'vision_independent', False))
            if _demo_monitor is not None and _demo_monitor.read_key() == 27:
                print('[DEMO] Remote stop requested.', flush=True)
                break
            if imu_stream is not None:
                try:
                    with _perf.span('camera.imu_transition', 'device_wait'):
                        _sync_on_demand_imu(fsm, imu_stream, camera_device)
                except Exception as exc:
                    fsm._fail(f"on-demand IMU transition failed: {exc}", [])
                    break
            if not demo_mode and recording_completion.update(fsm.state, time.monotonic()):
                recording = False
                if _presentation is not None:
                    _presentation.finish_recording()
                for writer in (video_writer, raw_writer):
                    if writer is not None:
                        writer.release()
                video_writer = raw_writer = None
                print("[REC] Insertion complete +5s: recording stopped and videos saved.")
            if _presentation is not None:
                if _presentation.stop.is_set():
                    break
                while not _presentation.keys.empty():
                    key = _presentation.keys.get_nowait()
                    if key == ord(' '):
                        debug_gate.request_step()
                    elif key == ord('r') and not recording_completion.stopped:
                        recording = not recording
                        if not recording and _presentation.raw is not None:
                            _presentation.raw.pause()
            if getattr(fsm, "vision_independent", False):
                # Insertion timing must not wait for detections, depth or even
                # a new camera frame. No perception/PnP call occurs here.
                lines = [("Inference / PnP OFF: IMU coarse / insertion / stopped", COLOR_META)]
                previous_fsm_state = fsm.state
                if debug_gate.should_step:
                    lines.extend(fsm.step(False, None, None, None, None))
                    debug_gate.after_step(previous_fsm_state)
                if demo_mode:
                    if fsm.state in {"DONE", "FAILED"}:
                        print(f"[DEMO] {fsm.state}: {getattr(fsm, 'failure_reason', '')}")
                        break
                    with _perf.span('control.idle_wait', 'intentional_wait'):
                        time.sleep(0.01)
                    continue
                lines.extend(debug_gate.hud_lines())
                with _perf.span('camera.poll', 'device_poll'):
                    frames = frame_source.poll_for_frames()
                color_frame = frames.get_color_frame() if frames else None
                if color_frame:
                    blind_view = np.asanyarray(color_frame.get_data()).copy()
                    if _presentation is not None and recording and _presentation.raw is not None:
                        _presentation.raw.submit(blind_view, int(color_frame.get_frame_number()), time.monotonic())
                    if recording and raw_writer is not None:
                        raw_writer.write(blind_view)
                        raw_video_frame_count += 1
                if _presentation is not None:
                    _presentation.publish(blind_view, fsm, lines, recording,
                                          captured_mono=time.monotonic())
                    with _perf.span('control.idle_wait', 'intentional_wait'):
                        _presentation.stop.wait(0.01)
                    continue
                show = _compose_runtime_view(
                    blind_view, fsm, diagram_drawer, camera_display_scale,
                    interface_lines=lines, cmd_status=fsm.cmd_status,
                    interface_panel_width=interface_panel_width,
                    fsm_panel_width=fsm_panel_width,
                )
                if recording and video_writer is not None:
                    video_writer.write(show)
                cv2.imshow(window_title, _fit_view_for_display(show, display_limit))
                key = cv2.waitKey(10) & 0xFF
                if key == 27:
                    break
                if key == ord(' '):
                    debug_gate.request_step()
                elif key == ord('r') and not recording_completion.stopped:
                    recording = not recording
                continue
            with _perf.span('camera.frame_wait', 'device_wait'):
                frames = frame_source.wait_for_frames()

            with _perf.span('camera.align', 'host_mixed'):
                aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            # 프레임 누락 시 안전 처리
            if not color_frame or not depth_frame:
                missing_frame_options = ({'vision_meta': {'camera_frame_missing': True}}
                                         if getattr(fsm, 'accepts_vision_meta', False) else {})
                if demo_mode:
                    fsm.step(False, None, None, None, None, **missing_frame_options)
                    if fsm.state in {"DONE", "FAILED"}:
                        break
                    with _perf.span('control.idle_wait', 'intentional_wait'):
                        time.sleep(0.01)
                    continue
                if _presentation is not None:
                    if debug_gate.should_step:
                        previous_fsm_state = fsm.state
                        fsm.step(False, None, None, None, None, **missing_frame_options)
                        debug_gate.after_step(previous_fsm_state)
                    _presentation.publish(np.zeros((480, 640, 3), dtype=np.uint8), fsm,
                        [("프레임 없음", COLOR_ALERT)] + debug_gate.hud_lines(), recording)
                    with _perf.span('control.idle_wait', 'intentional_wait'):
                        _presentation.stop.wait(0.01)
                    continue
                if debug_gate.should_step:
                    previous_fsm_state = fsm.state
                    fsm.step(False, None, None, None, None, **missing_frame_options)
                    debug_gate.after_step(previous_fsm_state)
                vis = np.zeros((480, 640, 3), dtype=np.uint8)
                missing_lines = [("프레임 없음", COLOR_ALERT)]
                missing_lines.extend(debug_gate.hud_lines())
                rotation_hud = getattr(
                    fsm, "rotation_diagnostic_hud_lines", None,
                )
                if callable(rotation_hud):
                    missing_lines.extend(rotation_hud())
                show = _compose_runtime_view(
                    vis, fsm, diagram_drawer, camera_display_scale,
                    interface_lines=missing_lines,
                    cmd_status=fsm.cmd_status,
                    interface_panel_width=interface_panel_width,
                    fsm_panel_width=fsm_panel_width,
                )

                # ★ 자동 녹화: 첫 show가 생성되면 VideoWriter 자동 초기화
                if recording and rec_fps is not None and video_writer is None:
                    h, w = show.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    video_writer = cv2.VideoWriter(video_filename, fourcc, rec_fps, (w, h))
                    if video_writer.isOpened():
                        print(f"🔴 녹화 시작: {video_filename}")
                    else:
                        print("❌ VideoWriter 초기화 실패")
                        recording = False
                        video_writer = None

                if recording:
                    if video_writer is not None:
                        video_writer.write(show)

                cv2.imshow(
                    window_title, _fit_view_for_display(show, display_limit),
                )
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    break
                elif key == ord('r') and not recording_completion.stopped:
                    recording = not recording
                    print("🔴 녹화 시작" if recording else "⏹️ 녹화 중지")
                elif key == ord(' '):
                    debug_gate.request_step()
                continue

            # Capture both clocks before copying the image to host memory.
            # RealSense time is the preferred image-to-image interval; host
            # monotonic time shares a clock with inference start/end timestamps.
            frame_received_mono_ns = time.monotonic_ns()
            frame_received_mono = frame_received_mono_ns / 1_000_000_000.0
            frame_received_mono_ms = frame_received_mono_ns / 1_000_000.0
            try:
                color_sensor_timestamp_ms = float(color_frame.get_timestamp())
            except Exception:
                color_sensor_timestamp_ms = None
            try:
                color_timestamp_domain = str(
                    color_frame.get_frame_timestamp_domain()
                )
            except Exception:
                color_timestamp_domain = None
            try:
                color_frame_number = int(color_frame.get_frame_number())
            except Exception:
                color_frame_number = None
            _perf.context(camera_frame_number=color_frame_number, frame_i=frame_i + 1,
                          input_mono_ns=frame_received_mono_ns)
            _perf.event('camera.input', sensor_timestamp_ms=color_sensor_timestamp_ms,
                        timestamp_domain=str(color_timestamp_domain), input_mono_ns=frame_received_mono_ns)
            with _perf.span('camera.host_image_view', 'host_mixed'):
                color_img = np.asanyarray(color_frame.get_data())
            if not demo_mode:
                blind_view = color_img.copy()
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            # 자세(PnP)는 컬러 영상의 키포인트로 푸니 컬러 내부파라미터를 쓴다
            color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

            # === fps 실측 ===
            frame_i += 1
            if not demo_mode:
                disp_n += 1
                now_t = time.time()
                if now_t - disp_t0 >= 0.5:                 # HUD 표시용
                    fps_disp = disp_n / (now_t - disp_t0)
                    disp_n = 0
                    disp_t0 = now_t
                if probe_t0 is None and frame_i > 5:       # 워밍업 5프레임 건너뛰고 측정 시작
                    probe_t0 = now_t
                    probe_i0 = frame_i
                elif rec_fps is None and probe_t0 is not None \
                        and (frame_i - probe_i0) >= REC_FPS_PROBE_FRAMES:
                    dt = max(1e-6, now_t - probe_t0)
                    rec_fps = float(min(120.0, max(1.0, (frame_i - probe_i0) / dt)))
                    if _presentation is not None:
                        print(f"📏 처리 {rec_fps:.1f} fps → raw {stream_fps}fps / UI {_presentation.fps:.0f}fps 비동기 녹화")
                    else:
                        print(f"📏 실측 {rec_fps:.1f} fps (스트림 공칭 {stream_fps}fps) → 이 값으로 녹화 시작")

                # 0) raw 녹화: 오버레이가 얹히기 전 원본 컬러를 그대로 기록
                #    (프레임 누락 시엔 color_frame 이 없으므로 raw 에는 아무것도 안 남는다)
            raw_video_frame_index = None
            if _presentation is not None and recording and rec_fps is not None:
                _presentation.configure_recording(raw_filename, video_filename, stream_fps)
                raw_video_frame_index = _presentation.raw.submit(
                    color_img, color_frame_number, frame_received_mono)
            if _presentation is None and recording and rec_fps is not None:
                if raw_writer is None:
                    rh, rw = color_img.shape[:2]
                    raw_writer = cv2.VideoWriter(
                        raw_filename, cv2.VideoWriter_fourcc(*'mp4v'), rec_fps, (rw, rh))
                    if raw_writer.isOpened():
                        print(f"🔴 원본(raw) 녹화 시작: {raw_filename}")
                    else:
                        print("❌ raw VideoWriter 초기화 실패")
                        raw_writer = None
                if raw_writer is not None:
                    raw_writer.write(color_img)
                    # timing CSV에 실제 비디오의 0-based 위치를 함께 남긴다.
                    # 녹화 토글로 건너뛴 camera frame이 있어도 다음 index는
                    # raw 파일에서의 물리적 순서를 정확히 이어 간다.
                    raw_video_frame_index = raw_video_frame_count
                    raw_video_frame_count += 1

            vis = None if demo_mode or _presentation else color_img.copy()
            H, W = color_img.shape[:2]

            # meta.json 은 intrinsics 가 확정된 첫 프레임에 1회만
            if trace_enabled and not meta_written:
                try:
                    _units = float(depth_frame.get_units())
                except Exception:
                    _units = None
                trace_extra = {
                    "model": os.path.basename(MODEL_PATH),
                    "model_sha256": _file_sha256(MODEL_PATH),
                    "inference_preprocessing": perception.preprocessing_metadata(),
                    "fsm_version": fsm_version,
                    "face_kpts": list(POSE_FACE_KPTS),
                    "back_kpts": list(POSE_BACK_KPTS),
                    "center_kpt": POSE_CENTER_KPT,
                    "pose_use_multi_kpt": bool(POSE_USE_MULTI_KPT),
                    "pose_multi_min_front": int(POSE_MULTI_MIN_FRONT),
                    "pose_method": "visible_keypoints_ransac",
                    "pose_kpt_visibility_threshold": float(POSE_KPT_VIS_THR),
                    "can_enabled": bool(can_enabled),
                    "pallet_size_m": {
                        "width": PALLET_WIDTH_M,
                        "length": PALLET_LENGTH_M,
                        "height": PALLET_HEIGHT_M,
                    },
                }
                trace_extra.update(getattr(fsm, "trace_metadata", {}))
                if _presentation is not None:
                    trace_extra['runtime_threads'] = {
                        'ui_fps_limit': _presentation.fps,
                        'video_queue_frames': 8,
                        'video_time_mapping': '*.mp4.frames.jsonl; host time is authoritative',
                        'video_overflow': 'drop incoming frame; raw timing index blank',
                        'video_gaps': 'repeat previous image to preserve time; sidecar repeat_count',
                    }
                tracer.write_meta(depth_intrin, _units, stream_fps, extra=trace_extra)
                _perf.event('model.identity', model=trace_extra['model'],
                            model_sha256=trace_extra['model_sha256'],
                            inference_preprocessing=trace_extra['inference_preprocessing'])
                meta_written = True

            # 1) 감지 — pose 키포인트(전면부 외곽 4모서리)
            #    FSM이 지정한 개방루프 이동/회전 중에는 탐지를 끈다.
            skip_det = fsm.skip_detection
            # 호환 API 이름은 유지하되 값은 현재 프레임의 원시 PnP 결과다.
            # 모델 추론 결과에 과거 프레임 EMA나 sample-hold를 적용하지 않는다.
            offset_smooth = None
            yaw_smooth = None
            width_smooth = None
            inference_start_host_mono_ms = None
            inference_end_host_mono_ms = None
            inference_ran = False
            model_det_ok = False
            if skip_det:
                _perf.event('vision.skipped', reason='FSM skip_detection', fsm_state=fsm.state)
                det_ok, kpts4, bbox_now, kpts_all = False, None, None, None
            else:
                inference_start_host_mono_ms = time.monotonic_ns() / 1_000_000.0
                with _perf.span('vision.inference_call', 'host_mixed'):
                    det_ok, kpts4, bbox_now, kpts_all = perception.infer_front(color_img)
                inference_end_host_mono_ms = time.monotonic_ns() / 1_000_000.0
                inference_ran = True
                model_det_ok = bool(det_ok)

            # 2) 6D pose — 4모서리를 평면에 투영해 yaw / center / width 산출
            ok_plane = False
            rvec = tvec = None
            yaw_deg = None
            pitch_deg = None
            roll_deg = None
            ex = ey = ez = None
            width_now = None
            dist_euclid = None
            dist_z = None
            pallet_center_bearing_deg = None
            front_face_center_bearing_deg = None

            pose_src = None
            pose_n_used = None
            pose_rms_px = None
            selected_front_face = None
            selected_face_area_px2 = None
            selected_face_corners_px = None
            selected_face_corners_camera_m = None
            projected_face_areas_px2 = None
            if det_ok and (kpts_all is not None):
                # livegt 프로젝트와 동일하게 visibility 임계값을 통과한 전면/후면/
                # 중심 키포인트를 모두 쓴다. 정확히 전면 4점만 남으면 IPPE,
                # 그 외에는 RANSAC EPNP로 이상치를 제거한다.
                with _perf.span('pose.pnp', 'host_work'):
                    (ok_plane, yaw_deg, pitch_deg, roll_deg, center3d, rvec,
                     tvec, pose_info) = pose_from_visible_kpts_pnp(
                        kpts=kpts_all, intrin=color_intrin,
                    )
                pose_src = pose_info.get("src")
                pose_n_used = pose_info.get("n_used")
                pose_rms_px = pose_info.get("rms")
                selected_front_face = pose_info.get("selected_front_face")
                selected_face_area_px2 = pose_info.get("selected_face_area_px2")
                selected_face_corners_px = pose_info.get("selected_face_corners_px")
                projected_face_areas_px2 = pose_info.get("projected_face_areas_px2")
                # 전면 폭은 모델 실치수로 이미 아는 값 — depth 로 재던 sanity 가 불필요
                width_now = (
                    pose_info.get("selected_face_width_m", PALLET_FACE_W)
                    if ok_plane else None
                )
                if ok_plane:
                    ex, ey, ez = (float(center3d[0]), float(center3d[1]), float(center3d[2]))
                    # Physical outer face corners for the v4 predictive FOV
                    # planner.  rvec/tvec have already been canonicalised to
                    # the selected camera-facing vertical face.
                    try:
                        face_rotation, _ = cv2.Rodrigues(
                            np.asarray(rvec, dtype=np.float64).reshape(3, 1)
                        )
                        physical_face_width = (
                            PALLET_WIDTH_M
                            if selected_front_face in {"z_min", "z_max"}
                            else PALLET_LENGTH_M
                        )
                        half_w = 0.5 * float(physical_face_width)
                        half_h = 0.5 * float(PALLET_HEIGHT_M)
                        face_local = np.asarray([
                            [-half_w, -half_h, 0.0],
                            [ half_w, -half_h, 0.0],
                            [ half_w,  half_h, 0.0],
                            [-half_w,  half_h, 0.0],
                        ], dtype=np.float64)
                        face_center = np.asarray(
                            tvec, dtype=np.float64,
                        ).reshape(1, 3)
                        selected_face_corners_camera_m = (
                            (face_rotation @ face_local.T).T + face_center
                        ).astype(float).tolist()
                    except (TypeError, ValueError, cv2.error):
                        selected_face_corners_camera_m = None
                    # solvePnP object origin is the centre of the four front-face
                    # corners.  v4 FACE uses this optical-axis bearing rather
                    # than the model's separate keypoint-8 centre.
                    front_face_center_bearing_deg = math.degrees(
                        math.atan2(ex, ez)
                    )

                    # PnP 성공 후 모델의 8번 중심 키포인트 좌표로 수평 방위각을 계산한다.
                    # 중심점 visibility는 탐지 성공 기준으로 사용하지 않고 유한 좌표만 확인한다.
                    if kpts_all is not None and len(kpts_all) > POSE_CENTER_KPT:
                        center_kpt = kpts_all[POSE_CENTER_KPT]
                        center_u = float(center_kpt[0])
                        if math.isfinite(center_u):
                            center_ray_x = (
                                (center_u - float(color_intrin.ppx)) / float(color_intrin.fx)
                            )
                            pallet_center_bearing_deg = math.degrees(
                                math.atan2(center_ray_x, 1.0)
                            )
                    cur_off = np.array([ex, ey, ez], dtype=np.float32)

                    offset_smooth = cur_off
                    yaw_smooth = yaw_deg
                    width_smooth = width_now

                    dist_euclid = float(np.linalg.norm(center3d))
                    dist_z = ez
                else:
                    # 자세 산출 실패 = 이번 프레임은 미탐지로 취급 (FSM 이 RECOVER 로 판단)
                    det_ok = False

            # 3) 시각화 — 키포인트 0~7 (0~3 전면, 4~7 후면) 을 인덱스와 함께 표시
            pose_result_host_mono_ms = (
                time.monotonic_ns() / 1_000_000.0 if inference_ran else None
            )
            _perf.event('vision.result', inference_ran=inference_ran, detected=bool(model_det_ok),
                        pnp_ok=bool(ok_plane), input_mono_ns=frame_received_mono_ns,
                        pose_result_mono_ms=pose_result_host_mono_ms)
            if trace_enabled:
                tracer.log_inference_timing(
                    frame_i=frame_i,
                    fsm_state=fsm.state,
                    align_sub=fsm.align_sub,
                    camera_frame_number=color_frame_number,
                    camera_sensor_timestamp_ms=color_sensor_timestamp_ms,
                    camera_timestamp_domain=color_timestamp_domain,
                    camera_input_host_mono_ms=frame_received_mono_ms,
                    inference_start_host_mono_ms=inference_start_host_mono_ms,
                    inference_end_host_mono_ms=inference_end_host_mono_ms,
                    pose_result_host_mono_ms=pose_result_host_mono_ms,
                    inference_ran=inference_ran,
                    model_det_ok=model_det_ok,
                    pnp_ok=ok_plane,
                    yaw_deg=yaw_deg,
                    pos_x_m=ex,
                    pos_z_m=ez,
                    center_bearing_deg=front_face_center_bearing_deg,
                    pose_src=pose_src,
                    pose_n_used=pose_n_used,
                    pose_rms_px=pose_rms_px,
                    raw_video_frame_index=raw_video_frame_index,
                )

            overlay = None
            if not demo_mode:
                from types import SimpleNamespace
                overlay = dict(
                    color_intrin=SimpleNamespace(fx=float(color_intrin.fx), fy=float(color_intrin.fy),
                        ppx=float(color_intrin.ppx), ppy=float(color_intrin.ppy),
                        coeffs=list(getattr(color_intrin, 'coeffs', [0.0]*5))), kpts_all=kpts_all,
                    selected_face_corners_px=selected_face_corners_px,
                    selected_front_face=selected_front_face,
                    rvec=rvec, tvec=tvec, yaw_deg=yaw_deg,
                    pitch_deg=pitch_deg, roll_deg=roll_deg,
                )
                if _presentation is None:
                    draw_camera_overlay(vis, **overlay)

                # 4) HUD 텍스트
                lines = []
                if skip_det:
                    lines.append(("detection OFF (stored snapshot / timer)", COLOR_META))
                else:
                    lines.append(("detected" if det_ok else "no detection",
                                  COLOR_STATUS_OK if det_ok else COLOR_ALERT))

                # yaw: 현재 프레임의 원시 PnP 결과(EMA 없음)
                if ok_plane and (yaw_smooth is not None) and (yaw_deg is not None):
                    lines.append((f"yaw(now): {fmt_deg(yaw_deg)} deg", COLOR_YAW))
                    lines.append((f"yaw(model raw): {fmt_deg(yaw_smooth)} deg", COLOR_YAW))
                    if selected_front_face is not None:
                        lines.append((
                            f"front(area): {selected_front_face} | "
                            f"{float(selected_face_area_px2):.0f} px2",
                            COLOR_YAW,
                        ))
                else:
                    lines.append(("yaw: N/A", COLOR_ALERT))

                # pitch/roll: 평면 법선(pitch) + 전면 상변 방향(roll)
                if (pitch_deg is not None) and (roll_deg is not None):
                    lines.append((f"pitch: {fmt_deg(pitch_deg)} deg | roll: {fmt_deg(roll_deg)} deg", COLOR_YAW))
                else:
                    lines.append(("pitch/roll: N/A", COLOR_ALERT))

                # offset/width
                if ex is not None and (offset_smooth is not None):
                    lines.append((f"offset_x(now): {fmt_m(ex)} m", COLOR_OFFSET))
                    lines.append((f"offset(model raw): ({fmt_m(offset_smooth[0])}, {fmt_m(offset_smooth[1])}, {fmt_m(offset_smooth[2])})", COLOR_OFFSET))
                else:
                    lines.append(("offset(model raw): N/A", COLOR_OFFSET))

                if (width_now is not None) and (width_smooth is not None):
                    lines.append((f"pallet width(now): {width_now:.3f} m", COLOR_WIDTH))
                    lines.append((f"pallet width(model raw): {width_smooth:.3f} m", COLOR_WIDTH))
                else:
                    lines.append(("pallet width(model raw): N/A", COLOR_WIDTH))

                if (dist_euclid is not None) and (dist_z is not None):
                    lines.append((f"distance: euclid {dist_euclid:.3f} m | z {dist_z:.3f} m", COLOR_META))
                else:
                    lines.append(("distance: N/A", COLOR_META))

                recording_rate = (
                    f" | raw {stream_fps} / UI {_presentation.fps:.0f} fps"
                    if _presentation is not None and rec_fps else
                    (f" | rec {rec_fps:.1f}" if rec_fps else " | rec measuring...")
                )
                lines.append((f"fps: {fps_disp:5.1f} / stream {stream_fps}" + recording_rate,
                              COLOR_META))

                # 5) FSM 구동
                #    (로깅은 step 직전에: 제어 스텝의 'pallet_state_before' 가 된다)
                tracer.log_state(
                    frame_i=frame_i, fsm_state=fsm.state, align_sub=fsm.align_sub,
                    det_ok=det_ok,
                    center3d=(None if ex is None else (ex, ey, ez)),
                    yaw_deg=yaw_deg, yaw_model=yaw_smooth, width_m=width_now,
                    fps=fps_disp,
                    rotation_diagnostics=getattr(
                        fsm, "rotation_diagnostics", None,
                    ),
                )
            bbox_margin_norm = None
            if bbox_now is not None:
                try:
                    _bbox = np.asarray(bbox_now, dtype=np.float64).reshape(-1)
                    if len(_bbox) >= 4:
                        _x1, _y1, _x2, _y2 = map(float, _bbox[:4])
                        bbox_margin_norm = min(
                            _x1 / max(1.0, float(W)),
                            (float(W) - _x2) / max(1.0, float(W)),
                            _y1 / max(1.0, float(H)),
                            (float(H) - _y2) / max(1.0, float(H)),
                        )
                except Exception:
                    bbox_margin_norm = None
            fsm_step_kwargs = dict(
                det_ok=det_ok,
                detected_length=width_smooth if width_smooth is not None else None,
                dist_z=dist_z if dist_z is not None else None,
                yaw_smooth=yaw_smooth if yaw_smooth is not None else None,
                offset_smooth=offset_smooth if offset_smooth is not None else None,
                target_bearing_deg=pallet_center_bearing_deg,
            )
            if getattr(fsm, "accepts_vision_meta", False):
                face_corners_px = None
                if selected_face_corners_px is not None:
                    face_corners_px = selected_face_corners_px
                elif kpts4 is not None:
                    try:
                        face_corners_px = [
                            [float(point[0]), float(point[1])] for point in kpts4
                        ]
                    except Exception:
                        face_corners_px = None
                fsm_step_mono = time.monotonic()
                fsm_step_kwargs["vision_meta"] = {
                    "center_bearing_deg": (
                        front_face_center_bearing_deg
                        if front_face_center_bearing_deg is not None
                        else pallet_center_bearing_deg
                    ),
                    "front_face_center_bearing_deg": front_face_center_bearing_deg,
                    "model_center_bearing_deg": pallet_center_bearing_deg,
                    "bbox_margin_norm": bbox_margin_norm,
                    "frame_width": W,
                    "frame_height": H,
                    "fx": float(color_intrin.fx),
                    "fy": float(color_intrin.fy),
                    "ppx": float(color_intrin.ppx),
                    "ppy": float(color_intrin.ppy),
                    "face_corners_px": face_corners_px,
                    "face_corners_camera_m": selected_face_corners_camera_m,
                    "selected_front_face": selected_front_face,
                    "selected_face_area_px2": selected_face_area_px2,
                    "projected_face_areas_px2": projected_face_areas_px2,
                    "yaw_raw_deg": yaw_deg,
                    "pos_x_m": ex,
                    "pos_z_m": ez,
                    "measurement_mono": frame_received_mono,
                    "fsm_step_mono": fsm_step_mono,
                    "inference_latency_sec": max(
                        0.0, fsm_step_mono - frame_received_mono,
                    ),
                    "inference_start_mono": (
                        None if inference_start_host_mono_ms is None
                        else inference_start_host_mono_ms / 1000.0
                    ),
                    "inference_end_mono": (
                        None if inference_end_host_mono_ms is None
                        else inference_end_host_mono_ms / 1000.0
                    ),
                    "model_inference_latency_sec": (
                        None
                        if (
                            inference_start_host_mono_ms is None
                            or inference_end_host_mono_ms is None
                        )
                        else (
                            inference_end_host_mono_ms
                            - inference_start_host_mono_ms
                        ) / 1000.0
                    ),
                    "pose_result_mono": (
                        None if pose_result_host_mono_ms is None
                        else pose_result_host_mono_ms / 1000.0
                    ),
                    "camera_frame_number": color_frame_number,
                    "sensor_timestamp_ms": color_sensor_timestamp_ms,
                    "sensor_timestamp_domain": color_timestamp_domain,
                }
            guide_lines = []
            if debug_gate.should_step:
                previous_fsm_state = fsm.state
                guide_lines = fsm.step(**fsm_step_kwargs)
                debug_gate.after_step(previous_fsm_state)
            else:
                # 디버그 단계 진행은 멈춰도 회전 STOP 이후의 실제 관성은
                # 계속 발생하므로 PnP 기반 진단값만 실시간으로 갱신한다.
                telemetry_observer = getattr(
                    fsm, "observe_debug_telemetry", None,
                )
                if callable(telemetry_observer):
                    telemetry_observer(**fsm_step_kwargs)
            if demo_mode:
                if fsm.state in {"DONE", "FAILED"}:
                    print(f"[DEMO] {fsm.state}: {getattr(fsm, 'failure_reason', '')}")
                    break
                continue
            lines.extend(guide_lines)
            lines.extend(debug_gate.hud_lines())
            rotation_hud = getattr(fsm, "rotation_diagnostic_hud_lines", None)
            if callable(rotation_hud):
                lines.extend(rotation_hud())

            # CAN 송신기가 실제로 살아 있는지와 현재 유지 명령을 HUD에서 확인한다.
            can_status = get_can_status()
            can_counts = can_status["tx_counts"]
            if not can_status["enabled"]:
                lines.append((
                    f"CAN: OFF (monitor) logical_cmd={can_status['movement']}",
                    COLOR_META,
                ))
            else:
                can_ok = can_status["ready"] and can_status["errors"] == 0
                lines.append((
                    f"CAN: {'OK' if can_ok else 'ERROR'} cmd={can_status['movement']} "
                    f"mov={can_counts['movement']} ctrl={can_counts['control']} "
                    f"err={can_status['errors']}",
                    COLOR_STATUS_OK if can_ok else COLOR_ALERT,
                ))

            # 6~7) 카메라와 분리된 정보 패널 + FSM 패널 합성
            if _presentation is not None:
                _presentation.publish(color_img, fsm, lines, recording, overlay,
                    yaw_deg, dist_z, color_frame_number, frame_received_mono)
                continue
            show = _compose_runtime_view(
                vis, fsm, diagram_drawer, camera_display_scale,
                interface_lines=lines,
                cmd_status=fsm.cmd_status,
                interface_panel_width=interface_panel_width,
                fsm_panel_width=fsm_panel_width,
                prominent_yaw_deg=yaw_deg,
                prominent_z_distance_m=dist_z,
            )

            # ★ 자동 녹화: 첫 show가 준비되면 즉시 VideoWriter 초기화
            if recording and rec_fps is not None and video_writer is None:
                h, w = show.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter(video_filename, fourcc, rec_fps, (w, h))
                if video_writer.isOpened():
                    print(f"🔴 녹화 시작: {video_filename}")
                else:
                    print("❌ VideoWriter 초기화 실패")
                    recording = False
                    video_writer = None

            # 8) 녹화 쓰기
            if recording:
                if video_writer is not None:
                    video_writer.write(show)

            # 9) 표시 & 키 입력
            cv2.imshow(
                window_title, _fit_view_for_display(show, display_limit),
            )
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            elif key == ord('r') and not recording_completion.stopped:
                recording = not recording
                print("🔴 녹화 시작" if recording else "⏹️ 녹화 중지")
            elif key == ord(' '):
                debug_gate.request_step()

    finally:
        print("프로그램 종료 처리 중...")
        if _presentation is not None:
            _presentation.request_stop()
            can_close()
        try:
            if video_writer is not None:
                video_writer.release()
                print(f"✅ 비디오 저장 완료: {video_filename}")
        except Exception:
            pass
        try:
            if raw_writer is not None:
                raw_writer.release()
                print(f"✅ 원본(raw) 저장 완료: {raw_filename}")
        except Exception:
            pass
        try:
            if can_enabled and _presentation is None:
                can_close()
        except Exception:
            pass
        try:
            close_planning = getattr(fsm, 'close', None)
            if callable(close_planning):
                close_planning()
        except Exception as exc:
            print(f'[ROUTE ASYNC cleanup] {exc}')
        try:
            configure_can_tx_observer(None)
            tracer.close()
            if trace_enabled:
                print(f"✅ 시퀀스 로그 저장 완료: {trace_base}_*")
        except Exception:
            pass
        try:
            if imu_stream is not None:
                imu_stream.close()
        except Exception as exc:
            print(f"[IMU cleanup] {exc}")
        try:
            pipeline.stop()
        except Exception:
            pass
        if not demo_mode and _presentation is None:
            cv2.destroyAllWindows()
        if _presentation is not None:
            _presentation.cleanups.clear()
        print("✅ 리소스 정리 완료")


if __name__ == "__main__":
    main()
