"""RealSense SDK 에서 가져올 만한 것만 골라 옮긴 것들.

librealsense 의 processing block 은 **거의 전부 depth 전용**이다
(spatial / temporal / hole-filling / threshold / disparity / colorizer /
units-transform / hdr-merge — 소스에서 `_stream_filter.stream = RS2_STREAM_DEPTH`
로 못박혀 있다). 우리는 color 스트림에서 태그를 찾으므로 그쪽은 볼 것이 없다.

color 에 실제로 쓸모 있는 것은 네 가지뿐이었다.

1. **RGB AE ROI** — 컬러 센서의 자동노출을 태그 사각형에만 걸 수 있다.
   ds/d400/d400-color.cpp:160 이 FW 5.10.9 이상에서 SETRGBAEROI 펌웨어 명령을
   컬러 센서에 붙인다. 역광 도크에서 이게 있고 없고가 검출률을 가른다. -> ExposureROI

2. **YUYV 원본 휘도** — BGR8 을 달라고 하면 SDK 가 BT.601 **limited range** 로
   변환한다(proc/color-formats-converter.cpp: `c = y - 16`, `298*c >> 8`, 0~255 클램프).
   그 결과 Y<=16 은 전부 0 으로, Y>=235 는 전부 255 로 뭉개진다. 실측으로
   **256 단계 중 38 단계(15%)가 소실**된다. -> yuyv_to_luma

3. **프레임 드롭 회계** — pipeline 의 출력 큐는 용량 1 이다
   (src/pipeline/aggregator.cpp:16 `single_consumer_frame_queue<frame_holder>(1)`).
   넘치면 **앞(오래된 것)을 버린다**(rsutils concurrency.h `single_consumer_queue::enqueue`).
   즉 wait_for_frames 는 늘 최신 프레임을 주지만 그 사이는 조용히 사라진다. -> FrameStats

4. **프레임 메타데이터** — 프레임별 실제 노출/게인. 검출 실패 원인을 로그로 남길 수 있다.
   다만 리눅스에서는 커널 패치가 필요하다(아래 frame_meta 설명 참고).
   -> frame_meta / MetaReader / diagnose_frame

5. **컬러 수동 노출** — 모션블러가 실제 실패 원인이라 노출을 끊는 게 가장 크게 듣는다.
   주의: rs.option.auto_exposure_limit 은 **컬러에 없다** — SDK 가 뎁스 센서에만,
   그것도 글로벌 셔터일 때만 등록한다(d400-device.cpp:1062-1070). D435i RGB 는
   롤링 셔터라 해당 없다. 컬러는 수동 노출뿐이다. -> set_color_exposure

[근거] librealsense (SDK 소스), pyrealsense2 2.58.3
"""
import numpy as np


# ── 1. YUYV 원본 휘도 ────────────────────────────────────────────────────────

# BGR8 왕복에서 죽는 휘도 단계 수. 실측(SDK 변환식을 numpy 로 그대로 재현):
#   Y  0..16  -> gray 0    (17 단계가 하나로 뭉침)
#   Y 235..255-> gray 255  (21 단계가 하나로 뭉침)
#   중간 구간은 기울기 1.164 로 늘어나고 원래 Y 와 평균 9.1 단계 어긋난다.
LUMA_CLIPPED_LEVELS = 38


def yuyv_to_luma(buf):
    """pyrealsense2 가 준 YUYV 버퍼에서 **센서 원본 휘도**를 뽑는다.

    왜 이게 필요한가:
        `config.enable_stream(..., rs.format.bgr8, ...)` 로 받으면 SDK 가
        YUY2 -> BGR8 을 BT.601 limited range 로 변환한다. 그 뒤 우리가 다시
        cv2.COLOR_BGR2GRAY 로 되돌린다. 두 번 왕복하면서 **256 단계 중 38 단계가
        영구히 소실**된다(LUMA_CLIPPED_LEVELS 주석 참고).
        AprilTag 는 검은칸/흰칸의 국소 대비로 사각형을 찾고, refine_edges 는
        **경사(gradient)** 로 모서리를 서브픽셀까지 당긴다. 클리핑된 영역은
        경사가 0 이라 모서리가 그 자리에 못 박히고, 그게 그대로 자세 오차가 된다.
        YUYV 의 Y 바이트는 센서가 내보낸 휘도 그 자체다 — 왕복이 없다.

    **파이썬 래퍼의 함정 (검증함):**
        wrappers/python/pyrs_frame.cpp 의 get_frame_data 는 RGB8/BGR8/RGBA8/BGRA8
        만 (H, W, C) uint8 로 내보내고, **그 외는 전부 (H, W) 에 bytes-per-pixel
        크기의 스칼라**로 내보낸다. YUYV 는 bpp=2 라서
        `np.asanyarray(f.get_data())` 가 **(H, W) uint16** 으로 온다.
        (H, W, 2) uint8 이 아니다. software_device 로 실제 확인했다.
        그래서 uint8 로 다시 봐야 Y 바이트에 닿는다.

    Args:
        buf: (H, W) uint16 (pyrealsense2 YUYV 프레임) 또는 (H, 2W) uint8 원시 바이트

    Returns:
        (H, W) uint8, C-contiguous. 검출기는 연속 버퍼를 요구하므로 복사한다
        (1920x1080 실측 0.46 ms — 30fps 예산 33 ms 안에서 무시할 수준).
    """
    a = np.asanyarray(buf)
    if a.ndim != 2:
        raise ValueError("YUYV 버퍼는 2-D 여야 한다: %r" % (a.shape,))
    if a.dtype != np.uint8:
        a = a.view(np.uint8)                 # (H, W) uint16 -> (H, 2W) uint8
    # 짝수 바이트가 Y, 홀수 바이트가 U/V 가 번갈아 든다. Y 만 걷어낸다.
    return np.ascontiguousarray(a[:, 0::2])


# ── 2. 컬러 자동노출 ROI ─────────────────────────────────────────────────────

class ExposureROI:
    """컬러 센서의 자동노출을 태그 사각형에만 건다.

    왜 이게 SDK 에서 건질 가장 큰 물건인가:
        도크의 태그는 보통 화면의 작은 일부다. 기본 AE 는 **화면 전체**의
        히스토그램을 보므로, 뒤편 출입구로 햇빛이 들어오면 전체 평균에 맞춰
        노출을 줄이고 태그는 새까맣게 깔린다. 반대로 어두운 창고에서는 태그가
        하얗게 날아간다. 어느 쪽이든 검은칸/흰칸 대비가 무너져 검출이 끊긴다.
        ROI 를 태그에 걸면 펌웨어가 **태그만 보고** 노출을 정한다.

    이게 진짜 있는지:
        ds/d400/d400-color.cpp:155-161 `register_color_features()` 가
        FW >= 5.10.9 이고 D405/D401 이 아닐 때 컬러 센서에
        auto_exposure_roi_feature(rgb=True) 를 등록한다. 안쪽은 펌웨어 명령
        `SETRGBAEROI` 다(ds/features/auto-exposure-roi-feature.cpp).
        pyrealsense2 에서는 sensor.as_roi_sensor().set_region_of_interest() 로 닿는다.
        **소프트웨어 필터가 아니라 펌웨어 기능**이라 파이썬 대체물이 없다.
        직접 노출을 계산해 set_option(exposure) 하는 건 한 프레임 늦고 진동한다.

    쓰는 법:
        roi = ExposureROI(profile)          # open_realsense 가 준 profile
        for i, t, img in frames:
            rs = detect(det, to_gray(img))
            roi.follow(rs, img.shape)       # 태그를 찾았으면 거기에 건다

    주의:
        - 자동노출이 켜져 있어야 의미가 있다. AE 를 끄고 수동 노출을 쓰면 무시된다.
        - 태그를 놓친 프레임에는 **직전 ROI 를 유지**한다. 매 프레임 전체화면으로
          되돌리면 노출이 왕복하며 펄떡거려서 오히려 재검출을 방해한다.
        - ROI 가 너무 작으면 펌웨어가 거부한다(실패는 조용히 삼키고 False 를 준다).
    """

    #: ROI 한 변의 최소 픽셀. 이보다 작으면 펌웨어가 받지 않는 경우가 있어 넓혀 준다.
    MIN_SIDE_PX = 32

    def __init__(self, profile, pad=0.35, stream="color"):
        """
        Args:
            profile: pipeline.start(config) 가 준 객체
            pad: 태그 바운딩박스를 이 비율만큼 사방으로 넓혀 잡는다.
                0 이면 태그 안쪽만 본다 — 검은칸이 많아 노출이 과하게 밝아진다.
                0.35 면 태그 주변 배경이 조금 섞여 평형이 맞는다.
            stream: "color" 또는 "infrared". IR 은 depth 센서의 AE ROI 를 쓴다.
        """
        self.pad = float(pad)
        self.sensor = None
        self.last = None
        self.supported = False
        try:
            import pyrealsense2 as rs
            dev = profile.get_device()
            for s in dev.query_sensors():
                name = s.get_info(rs.camera_info.name).lower()
                want_rgb = (stream == "color")
                if want_rgb != name.startswith("rgb"):
                    continue
                if s.is_roi_sensor():
                    self.sensor = s.as_roi_sensor()
                    self.supported = True
                break
        except Exception:
            pass                              # 지원 안 하는 장치/펌웨어면 조용히 비활성

    def follow(self, detections, shape):
        """검출된 태그들을 덮는 사각형으로 AE ROI 를 옮긴다.

        Args:
            detections: detect() 가 준 리스트. 비어 있으면 아무것도 안 하고 False.
            shape: 영상 (H, W) 또는 (H, W, C)

        Returns:
            True 면 ROI 를 실제로 바꿨다.
        """
        if not self.supported or not detections:
            return False
        h, w = shape[0], shape[1]
        pts = np.concatenate([np.asarray(d.corners, dtype=np.float64)
                              for d in detections], axis=0)
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        px, py = (x1 - x0) * self.pad, (y1 - y0) * self.pad
        return self.set_box(x0 - px, y0 - py, x1 + px, y1 + py, w, h)

    def set_box(self, x0, y0, x1, y1, w, h):
        """픽셀 사각형으로 직접 건다. 화면 밖은 잘라내고 최소 크기를 보장한다."""
        if not self.supported:
            return False
        import pyrealsense2 as rs
        # 최소 크기 확보 — 중심을 유지한 채 벌린다
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        half = max(self.MIN_SIDE_PX / 2.0, (x1 - x0) / 2.0)
        x0, x1 = cx - half, cx + half
        half = max(self.MIN_SIDE_PX / 2.0, (y1 - y0) / 2.0)
        y0, y1 = cy - half, cy + half

        box = (max(0, int(x0)), max(0, int(y0)),
               min(w - 1, int(round(x1))), min(h - 1, int(round(y1))))
        if box[2] - box[0] < 4 or box[3] - box[1] < 4:
            return False
        if box == self.last:
            return False                      # 같은 값을 다시 쓰면 펌웨어 왕복만 낭비
        roi = rs.region_of_interest()
        roi.min_x, roi.min_y, roi.max_x, roi.max_y = box
        try:
            self.sensor.set_region_of_interest(roi)
        except Exception:
            return False                      # 펌웨어가 거부. 다음 프레임에 다시 시도
        self.last = box
        return True

    def reset(self, w, h):
        """ROI 를 화면 전체로 되돌린다. 태그를 오래 놓쳤을 때 탐색용으로 쓴다."""
        return self.set_box(0, 0, w - 1, h - 1, w, h)


# ── 3. 프레임 드롭 회계 ──────────────────────────────────────────────────────

class FrameStats:
    """프레임을 몇 장 흘렸는지 센다.

    왜 필요한가 — SDK 소스에서 확인한 사실:
        pipeline 의 출력 큐는 **용량 1** 이다
        (src/pipeline/aggregator.cpp:16). 그리고 큐가 넘치면
        `single_consumer_queue::enqueue` 가 **맨 앞(가장 오래된 것)을 버린다**
        (third-party/rsutils/.../concurrency.h). 실측으로도 용량 2 짜리 큐에
        1..7 을 넣고 빼면 [6, 7] 만 나온다.

        즉 wait_for_frames 는 **절대 밀리지 않는다**(늘 최신 프레임을 준다).
        대신 우리 루프가 33 ms 를 넘기면 그 사이 프레임은 아무 소리 없이 사라진다.
        25~32 fps 가 나온다면 "카메라가 30fps 를 못 낸다"가 아니라
        "우리가 30fps 로 소비하지 못해 버려지고 있다" 쪽일 수 있는데,
        SDK 는 이걸 알려주지 않는다. frame_number 의 구멍을 세면 알 수 있다.

    쓰는 법:
        st = FrameStats()
        for i, t, img in frames:
            st.update(img)                    # Frame 이면 frame_number 를 알아서 읽는다
        print(st.summary())
    """

    def __init__(self):
        self.received = 0
        self.dropped = 0
        self.gaps = 0            # 구멍이 난 횟수 (연속 드롭 1회 = 1)
        self.first_number = None
        self.last_number = None
        self._t_first = None
        self._t_last = None

    def update(self, frame_or_number, t=None):
        """프레임 하나를 반영한다.

        Args:
            frame_or_number: Frame(open_realsense 가 준 것) / 정수 frame_number /
                None(번호를 모를 때 — 개수만 센다)
            t: 초 단위 타임스탬프. 주면 fps 를 같이 낸다.

        Returns:
            int — **이 프레임 바로 앞에서 사라진 장 수.** 0 이면 연속이다.
            open_realsense 가 이 값을 Frame.dropped_before 에 실어 보낸다.
            그래야 "이 프레임에서 검출이 끊긴 게 앞 프레임이 통째로 사라져서인지"를
            프레임 단위로 따질 수 있다(diagnose_frame 참고). 번호를 모르면 0.
        """
        n = getattr(frame_or_number, "frame_number", None)
        if n is None and isinstance(frame_or_number, (int, np.integer)):
            n = int(frame_or_number)
        self.received += 1
        if t is not None:
            if self._t_first is None:
                self._t_first = float(t)
            self._t_last = float(t)
        if n is None:
            return 0
        n = int(n)
        miss = 0
        if self.last_number is not None:
            miss = n - self.last_number - 1
            if miss > 0:
                self.dropped += miss
                self.gaps += 1
            else:
                miss = 0                 # 번호가 되감기면(bag loop) 음수가 나온다. 무시.
        if self.first_number is None:
            self.first_number = n
        self.last_number = n
        return miss

    @property
    def fps(self):
        """소비 fps. 타임스탬프를 안 줬으면 0.0."""
        if self._t_first is None or self._t_last is None:
            return 0.0
        dt = self._t_last - self._t_first
        return (self.received - 1) / dt if dt > 0 else 0.0

    @property
    def sent(self):
        """카메라가 보낸 장 수 = 받은 것 + 버려진 것. "300 중 12 버림"의 300."""
        return self.received + self.dropped

    @property
    def drop_rate(self):
        """버려진 비율 0.0~1.0. 카메라가 보낸 것 대비다."""
        return (self.dropped / self.sent) if self.sent else 0.0

    def summary(self):
        """한 줄 요약.

        읽는 법 — 이게 이 클래스를 만든 이유다:
            버림이 0 인데 fps 가 30 을 밑돈다  -> 카메라/USB 가 못 내주고 있다.
                                                (조명이 어두워 AE 가 fps 를 떨군
                                                 경우가 흔하다. auto_exposure_priority
                                                 를 0 으로 두면 막힌다.)
            버림이 있다                        -> 우리 루프가 33ms 를 못 지켰다.
                                                pipeline 출력 큐가 용량 1 이라
                                                밀린 것은 조용히 버려진다.
            둘은 대책이 정반대라 반드시 갈라 봐야 한다.
        """
        s = "받음 %d / 보냄 %d, 버림 %d (%.1f%%), 구멍 %d회" % (
            self.received, self.sent, self.dropped, self.drop_rate * 100.0, self.gaps)
        if self.fps:
            s += ", 소비 %.1f fps" % self.fps
        if self.last_number is None:
            s += "  [frame_number 없음 — 개수만 셈]"
        return s


# ── 4. 프레임 메타데이터 / 타임스탬프 ─────────────────────────────────────────

# 값이 있을 때 자세 실패 원인 규명에 실제로 쓰이는 것들만 추렸다.
_META_KEYS = ("actual_exposure", "gain_level", "frame_counter", "sensor_timestamp",
              "time_of_arrival", "backend_timestamp", "actual_fps",
              "auto_exposure", "white_balance", "frame_laser_power_mode")


def frame_meta(f, keys=_META_KEYS):
    """프레임 하나의 메타데이터를 dict 로. 없는 항목은 아예 넣지 않는다.

    검출에 실패한 프레임에 이걸 같이 찍어두면 "왜 못 찾았나"가 사후에 보인다.
    노출이 튀어 흐려졌는지(actual_exposure 급증), 게인이 올라 노이즈가 낀 건지
    (gain_level), 그냥 프레임이 밀린 건지(frame_counter 구멍)가 구분된다.

    **리눅스/WSL 경고 (SDK 소스로 확인):**
        `actual_exposure` / `gain_level` / `sensor_timestamp` / `frame_counter` 는
        UVC 벤더 메타데이터 페이로드에서 나온다. 리눅스에서는 v4l2 가
        `V4L2_META_FMT_D4XX` 로 메타데이터 노드를 열어줘야 하고
        (src/linux/backend-v4l2.cpp:2875), 그건 librealsense 커널 패치가
        깔려 있어야 생긴다. **WSL2 의 기본 uvcvideo 에는 없다.**
        메타데이터가 없으면 SDK 는 조용히 시스템 시각으로 폴백한다
        (src/ds/ds-timestamp.cpp:60-74, "UVC metadata payloads not available").

        반대로 `time_of_arrival` 은 **항상** 있다 — src/sensor.cpp:79 의
        sensor_base 생성자가 모든 센서에 무조건 등록한다(하드웨어와 무관).

        그러니 이 함수가 빈 dict 를 주더라도 코드 잘못이 아니다.
        무엇이 실제로 살아 있는지는 describe_metadata() 로 한 번 찍어보면 된다.

    Args:
        f: pyrealsense2 frame
        keys: 읽어볼 rs.frame_metadata_value 이름들

    Returns:
        {이름: 정수값}. 지원 안 하는 항목은 키 자체가 없다.
    """
    try:
        import pyrealsense2 as rs
    except Exception:
        return {}
    out = {}
    for k in keys:
        mv = getattr(rs.frame_metadata_value, k, None)
        if mv is None:
            continue
        try:
            if f.supports_frame_metadata(mv):
                out[k] = int(f.get_frame_metadata(mv))
        except Exception:
            pass
    return out


def describe_metadata(f):
    """이 장치/커널에서 **실제로** 살아 있는 메타데이터를 전부 훑어 문자열로.

    카메라를 새 PC 에 꽂았을 때 한 번 돌려보는 진단용이다.
    `actual_exposure` 가 목록에 없으면 커널 패치가 안 깔린 것이다
    (frame_meta 설명 참고). 그 상태에서 노출 로그를 기대하면 안 된다.
    """
    try:
        import pyrealsense2 as rs
    except Exception:
        return "pyrealsense2 를 못 불러왔다"
    names = [a for a in dir(rs.frame_metadata_value)
             if not a.startswith("_") and a not in ("name", "value")]
    got = []
    for n in names:
        mv = getattr(rs.frame_metadata_value, n)
        try:
            if f.supports_frame_metadata(mv):
                got.append("  %-28s %s" % (n, f.get_frame_metadata(mv)))
        except Exception:
            pass
    head = "timestamp=%.3f ms  domain=%s  frame_number=%d" % (
        f.get_timestamp(), f.get_frame_timestamp_domain(), f.get_frame_number())
    if not got:
        return head + "\n  (메타데이터 없음 — 리눅스면 커널 패치 미적용)"
    return head + "\n" + "\n".join(got)


def timestamp_domain(f):
    """이 프레임의 타임스탬프가 무슨 시계인지 문자열로.

    셋 중 하나다(SDK 의 global_timestamp_reader 를 읽고 정리한 것).
        hardware_clock  카메라 내부 시계. UVC 메타데이터가 살아 있을 때만 나온다.
        global_time     hardware_clock 을 호스트 시계에 맞춰 보정한 값.
                        **hardware_clock 일 때만** 나온다 —
                        global_timestamp_reader::get_frame_timestamp 가
                        `ts_domain == HARDWARE_CLOCK` 을 먼저 확인한다.
        system_time     메타데이터가 없어 폴백한 호스트 도착시각.

    그래서 **메타데이터가 없는 환경(WSL2 등)에서는 global_time 을 켜도 아무 일도
    일어나지 않는다.** system_time 이 그대로 나온다. 이 경우 타임스탬프는
    "센서가 찍은 순간"이 아니라 "호스트에 도착한 순간"이므로, USB 지연과
    우리 루프의 대기시간이 섞여 있다. 속도 추정에 그대로 쓰면 안 된다.
    """
    return str(f.get_frame_timestamp_domain())


def set_global_time(profile, enabled=True):
    """모든 센서의 global_time_enabled 를 켜고/끈다.

    켜면 하드웨어 시계를 호스트 시계에 맞춰 보정해 준다 — depth/color/IMU 를
    한 시간축에서 볼 수 있게 된다. **단 hardware_clock 도메인일 때만 동작한다**
    (timestamp_domain 설명 참고).

    Returns:
        {센서이름: 적용됨(bool)}
    """
    out = {}
    try:
        import pyrealsense2 as rs
    except Exception:
        return out
    for s in profile.get_device().query_sensors():
        name = s.get_info(rs.camera_info.name)
        try:
            if s.supports(rs.option.global_time_enabled):
                s.set_option(rs.option.global_time_enabled, 1.0 if enabled else 0.0)
                out[name] = True
            else:
                out[name] = False
        except Exception:
            out[name] = False
    return out


# ── 5. 프레임별 메타데이터 수집기 ────────────────────────────────────────────

class MetaReader:
    """프레임마다 노출/게인/센서시각을 뽑아 Frame 에 실어 보내기 위한 수집기.

    왜 frame_meta() 를 그냥 매 프레임 부르지 않는가:
        frame_meta() 는 후보 키 10개를 전부 supports_frame_metadata() 로 물어본다.
        그 중 **어떤 키가 살아 있는지는 장치/커널이 정해지면 변하지 않는다.**
        (UVC 벤더 페이로드가 오느냐 마느냐의 문제라 스트리밍 중에 바뀌지 않는다.)
        그래서 **첫 프레임에서 한 번만 조사**하고, 이후로는 살아 있는 키만
        get_frame_metadata() 로 읽는다. WSL2 처럼 하나도 안 오는 환경에서는
        첫 프레임 이후 read() 가 곧장 빈 dict 를 돌려주므로 사실상 공짜다.

    왜 이걸 모으는가 (이게 이 클래스의 존재 이유다):
        검출에 실패한 프레임을 놓고 **원인을 세 갈래로 가를 수 있다.**
            actual_exposure 가 크다  -> 모션블러. 실측 블러 절벽 참고
                                        (10px 까지 100%, 32px 에서 0%).
            gain_level 이 크다       -> 어두워서 게인으로 밀어올린 것. 노이즈.
            frame_number 에 구멍     -> 애초에 그 프레임을 못 받은 것(FrameStats).
        이 셋을 구분 못 하면 "가끔 안 잡힌다"에서 한 발짝도 못 나간다.

    **다만 리눅스에서는 대개 아무것도 안 온다.** 커널 패치가 없으면
    UVC 벤더 메타데이터 노드 자체가 없다 — frame_meta() 설명 참고.
    그 경우 이 수집기는 조용히 빈 dict 만 내놓는다(예외를 던지지 않는다).
    살아 있는지 확인은 `src/etc/realsense_check.py` 4단계나 supported 로 본다.
    """

    def __init__(self, keys=_META_KEYS):
        self._keys = tuple(keys)
        self._probed = None          # [(이름, rs.frame_metadata_value)] — 첫 프레임에 정해진다
        self.supported = ()          # 실제로 살아 있는 키 이름들

    def probe(self, f):
        """첫 프레임에서 살아 있는 키를 한 번만 조사한다."""
        try:
            import pyrealsense2 as rs
        except Exception:
            self._probed, self.supported = (), ()
            return self.supported
        live = []
        for k in self._keys:
            mv = getattr(rs.frame_metadata_value, k, None)
            if mv is None:
                continue
            try:
                if f.supports_frame_metadata(mv):
                    live.append((k, mv))
            except Exception:
                pass
        self._probed = tuple(live)
        self.supported = tuple(k for k, _ in live)
        return self.supported

    def read(self, f):
        """이 프레임의 메타데이터 dict. 없으면 {} (예외 없음)."""
        if self._probed is None:
            self.probe(f)
        if not self._probed:
            return {}
        out = {}
        for k, mv in self._probed:
            try:
                out[k] = int(f.get_frame_metadata(mv))
            except Exception:
                pass                 # 중간에 빠지는 항목이 있어도 나머지는 살린다
        return out


# ── 6. 검출 실패 원인 규명 ───────────────────────────────────────────────────

# 모션블러 절벽. 1920x1080, 태그 238px(=tag36h11 한 칸 23.8px)에서 실측한 값이다.
# 가로 블러를 픽셀 단위로 넣어가며 28프레임 클립을 다시 돌렸다:
#     0~10px  28/28 100%      (한 칸의 0.42배까지는 멀쩡하다)
#      12px   27/28  96.4%
#      16px   26/28  92.9%
#      20px   25/28  89.3%
#      24px   22/28  78.6%
#      28px   14/28  50.0%
#      32px    0/28   0.0%    (한 칸의 1.34배. 여기서 완전히 무너진다)
# 눈여겨볼 것: 서서히 나빠지다 32px 에서 **뚝 끊긴다.** 절벽이지 경사가 아니다.
BLUR_CLEAN_PX = 10.0     #: 여기까지는 검출률 100%
BLUR_DEAD_PX = 32.0      #: 여기서 0%

#: 컬러 센서의 rs.option.exposure 한 눈금이 몇 마이크로초인가. set_color_exposure 참고.
COLOR_EXPOSURE_UNIT_US = 100.0


def motion_blur_px(exposure_us, speed_mps, fx, z_m):
    """노출시간 동안 태그가 화면에서 몇 픽셀 밀리는가.

        blur_px = fx * v * t / z

    핀홀 투영을 시간으로 미분한 것이다. 카메라(지게차)가 태그에 대해 v[m/s] 로
    가로로 움직이면, 거리 z 에서 각속도가 v/z 이고 화면에서는 fx 를 곱한 만큼 밀린다.

    이 값을 BLUR_CLEAN_PX(10) / BLUR_DEAD_PX(32) 와 대보면 된다. 위 상수 주석에
    실측 절벽표가 있다. D435i 컬러 1920x1080 (fx=1359.2) 기준 환산:

        속도      z=1m 노출 33.3ms      z=2m 노출 33.3ms
        0.3 m/s   14px  (~90%)           7px  (100%)
        0.5 m/s   23px  (~50%)          11px  (~90%)
        1.0 m/s   45px  ( 0%)           23px  (~50%)
        2.0 m/s   91px  ( 0%)           45px  ( 0%)

    **읽는 법: 어두운 창고에서 자동노출이 33.3ms(30fps 한 프레임 전체)까지 늘어나면
    1.0 m/s 로 접근하는 지게차는 z=1m 에서 태그를 아예 못 찾는다.**
    나빠지는 게 아니라 0 이 된다. 밝기 문제로 보이지만 실제로는 블러 문제다.

    Args:
        exposure_us: 노출시간 [us]. Frame.exposure_us 를 그대로 넣으면 된다.
        speed_mps: 태그에 대한 **가로방향** 상대속도 [m/s].
            정면으로 다가가기만 하면 태그는 화면에서 거의 안 밀리므로
            블러도 훨씬 적다. 최악을 보려면 접근속도를 그대로 넣어라.
        fx: 초점거리 [px]
        z_m: 태그까지 거리 [m]

    Returns:
        float [px]
    """
    if not z_m or z_m <= 0:
        return float("inf")
    return float(fx) * float(speed_mps) * (float(exposure_us) * 1e-6) / float(z_m)


def exposure_budget_us(speed_mps, fx, z_m, blur_px=BLUR_CLEAN_PX):
    """검출률 100% 를 지키려면 노출을 몇 us 안으로 끊어야 하는가.

    motion_blur_px 를 t 에 대해 뒤집은 것이다. 기본값 blur_px=BLUR_CLEAN_PX(10px)
    는 실측에서 28/28 이 유지된 마지막 지점이다.

    fx=1359.2, z=1m 기준 실측 환산:
        0.3 m/s -> 24.5 ms    0.5 m/s -> 14.7 ms
        1.0 m/s ->  7.4 ms    2.0 m/s ->  3.7 ms
    (30fps 한 프레임이 33.3ms 이므로, 1 m/s 부터는 **프레임시간의 1/4 이하**로
     끊어야 한다는 뜻이다. 자동노출을 그대로 두면 절대 안 지켜진다.)

    Returns:
        float [us]. 속도가 0 이면 inf.
    """
    v = abs(float(speed_mps))
    if v <= 0:
        return float("inf")
    return float(blur_px) * float(z_m) / (float(fx) * v) * 1e6


def diagnose_frame(img, fx=None, z_m=None, speed_mps=None):
    """이 프레임에서 태그를 못 찾았다면 **왜 못 찾았는지** 한 줄로.

    open_realsense 가 Frame 에 붙여 보낸 것(dropped_before / meta / exposure_us /
    gain)만 가지고 판단한다. 검출 결과는 안 본다 — 실패한 프레임에 대고 부르는
    함수다.

        for i, t, img in frames:
            r = detect(det, to_gray(img))
            if not r:
                print(i, diagnose_frame(img, fx=intr.fx, z_m=last_z, speed_mps=0.5))

    갈래는 셋이다(MetaReader 설명 참고):
        - 앞에서 프레임이 통째로 사라졌다        -> dropped_before > 0
        - 노출이 길어 흘렀다                      -> exposure_us + 속도/거리로 블러 환산
        - 어두워 게인으로 밀어올렸다              -> gain 이 크다
    메타데이터가 안 오는 환경(WSL2 등)에서는 판단할 재료가 없다고 **그렇게 말한다.**
    없는 걸 있는 척 추정하지 않는다.

    Args:
        fx, z_m, speed_mps: 셋 다 주면 블러를 픽셀로 환산해 절벽표와 대준다.
            없으면 노출시간만 그대로 적는다.

    Returns:
        한국어 한 줄 문자열.
    """
    parts = []
    n = getattr(img, "dropped_before", 0) or 0
    if n:
        parts.append("직전 %d장 유실(파이프라인 큐가 버림)" % n)

    exp = getattr(img, "exposure_us", None)
    gain = getattr(img, "gain", None)
    if exp is None and gain is None:
        # "메타데이터가 비었다"와 "애초에 RealSense 프레임이 아니다"는 다른 얘기다.
        # 영상파일/웹캠 프레임에 대고 "커널 패치를 깔아라"라고 하면 헛다리다.
        if getattr(img, "meta", None) is None:
            why = "RealSense 프레임이 아니다(영상파일/웹캠) — 노출 정보가 원래 없다"
        else:
            why = "프레임 메타데이터가 안 온다(리눅스면 커널 패치 미적용)"
        if not parts:
            return "판단 재료 없음 — " + why
        parts.append("노출/게인 불명 — " + why)
        return ", ".join(parts)

    if exp is not None:
        s = "노출 %.1fms" % (exp / 1000.0)
        if fx and z_m and speed_mps:
            b = motion_blur_px(exp, speed_mps, fx, z_m)
            if b >= BLUR_DEAD_PX:
                s += " -> 블러 %.1fpx: 실측 0%% 구간(%.0fpx 에서 검출이 끊긴다)" % (b, BLUR_DEAD_PX)
            elif b > BLUR_CLEAN_PX:
                s += " -> 블러 %.1fpx: 실측 열화 구간(%.0fpx 넘음)" % (b, BLUR_CLEAN_PX)
            else:
                s += " -> 블러 %.1fpx: 블러는 무죄" % b
            s += " (노출을 %.1fms 로 끊으면 100%%)" % (
                exposure_budget_us(speed_mps, fx, z_m) / 1000.0)
        parts.append(s)
    if gain is not None:
        parts.append("게인 %d" % gain)
    return ", ".join(parts)


# ── 7. 컬러 센서 노출 제어 ───────────────────────────────────────────────────

def set_color_exposure(profile, exposure_us=None, ae_priority=None, gain=None):
    """컬러 센서의 노출을 직접 잡는다. 모션블러를 끊는 **유일한** 방법이다.

    **먼저 읽어야 할 SDK 사실 — auto_exposure_limit 은 여기서 못 쓴다:**
        rs.option.auto_exposure_limit(마이크로초 단위로 AE 상한을 거는 옵션)은
        pyrealsense2 의 enum 에는 있지만 **D435i 컬러 센서에는 등록되지 않는다.**
        src/ds/d400/d400-device.cpp:1062-1070 이 auto_exposure_limit_feature 를
        `get_depth_sensor()` 에만 붙이고, 그나마도
            "// ae / gain limit feature is not supported on rolling-shutter"
            if (fw >= 5.12.10.11 && (caps & CAP_GLOBAL_SHUTTER))
        로 **글로벌 셔터일 때만** 등록한다. D435i 의 RGB 는 롤링 셔터다.
        그러니 enum 이 있다고 set_option 을 부르면 그냥 실패한다.

        컬러 센서가 실제로 받는 것은 ds-color-common.cpp:84-97 에 등록된 네 개뿐이다:
            EXPOSURE / GAIN / ENABLE_AUTO_EXPOSURE / AUTO_EXPOSURE_PRIORITY
        따라서 노출을 끊으려면 **수동 노출로 못박는 수밖에 없다.**

    **함정 1 — 단위가 100us 다 (뎁스와 100배 다르다).**
        컬러의 EXPOSURE 는 uvc_pu_option 이라 값이 v4l2 로 그대로 간다
        (backend-v4l2.cpp:2476 -> V4L2_CID_EXPOSURE_ABSOLUTE). 이 v4l2 컨트롤은
        UVC 규격대로 **100us 눈금**이다. 즉 7.4ms 는 74 이지 7400 이 아니다.
        반면 **뎁스 센서의 EXPOSURE 는 us 단위**다. 같은 이름 같은 enum 인데
        센서에 따라 100배가 다르다. 그래서 이 함수는 인자를 us 로 받아
        COLOR_EXPOSURE_UNIT_US 로 나눠 넣고, **넣은 뒤 되읽어서** 실제로 적용된
        값을 돌려준다. 짐작하지 말고 반환값을 보라.

    **함정 2 — 노출을 건드리면 자동노출이 꺼진다.**
        ds-color-common.cpp:90-93 이 EXPOSURE 를 auto_disabling_control 로 한 번 더
        감싸 두었다. set(EXPOSURE) 하면 SDK 가 ENABLE_AUTO_EXPOSURE 를 0 으로
        내려버린다. **의도된 동작이지만 조용하다.** 노출을 고정하면 조명이 바뀔 때
        따라가지 못하므로, 도크 조명이 일정한 현장에서만 쓸 것.
        조명이 오락가락하면 차라리 AE 를 두고 ExposureROI 로 태그만 재게 하는 편이 낫다.

    왜 이걸 쓰나 — 실측 근거:
        검출은 블러 10px 까지 100%, 32px 에서 0% 다(BLUR_CLEAN_PX 주석의 절벽표).
        어두운 창고에서 AE 는 노출을 프레임시간 전체(30fps -> 33.3ms)까지 늘리는데,
        그러면 1.0 m/s, z=1m 에서 블러가 45px 이라 **검출이 0** 이 된다.
        7.4ms 로 끊으면 10px 이라 100% 다(exposure_budget_us 참고).
        짧게 끊는 대가는 거의 없다 — 정지 장면 노출 스윕에서 1/16 노출까지도
        28/28(100%) 이 유지됐다(decision_margin 은 72.4 -> 19.5 로 떨어지지만
        MIN_DECISION_MARGIN=20 의 바로 위다).

        **단, 이 노출 스윕은 시뮬레이션이다**(카메라가 없어 센서 모델로 재현했다).
        블러 절벽은 실제 영상에 블러를 넣어 잰 실측이다. 현장에서 한 번 확인할 것.

    Args:
        profile: pipeline.start() 가 준 것. open_realsense 는 frames.profile 로 준다.
        exposure_us: 수동 노출 [us]. None 이면 안 건드린다(자동노출 유지).
        ae_priority: AUTO_EXPOSURE_PRIORITY. 0 이면 "fps 를 지켜라"(노출이
            프레임시간을 못 넘는다), 1 이면 "어두우면 fps 를 떨어뜨려도 좋다".
            **기본이 어느 쪽인지는 펌웨어 나름이라 확인하고 싶으면 0 으로 박아라.**
            30fps 를 보장해야 하는 도킹에서는 0 이 맞다. 다만 0 이어도 상한이
            33.3ms 라 블러는 여전히 못 막는다 — 그건 exposure_us 로 끊어야 한다.
        gain: 수동 게인. None 이면 안 건드린다. 노출을 줄인 만큼 밝기를 보상하려면
            올려야 하지만, 노이즈가 늘면 refine_edges 가 흔들린다. 자동에 맡기는
            편이 대체로 낫다(노출만 고정해도 게인은 따라 움직이지 않는다 —
            AE 가 통째로 꺼지므로 게인도 그 순간 값에 고정된다. 그래서 어두우면
            여기서 같이 올려줘야 한다).

    Returns:
        {"exposure_us": 실제 적용된 노출 [us] 또는 None,
         "ae_priority": 실제 값 또는 None,
         "gain": 실제 값 또는 None,
         "auto_exposure": 적용 후 AE 가 켜져 있는지 (bool) 또는 None,
         "errors": [실패한 항목 이름...]}
        되읽기가 핵심이다 — 단위를 잘못 넣었으면 여기서 티가 난다.
    """
    out = {"exposure_us": None, "ae_priority": None, "gain": None,
           "auto_exposure": None, "errors": []}
    try:
        import pyrealsense2 as rs
    except Exception:
        out["errors"].append("pyrealsense2")
        return out

    sensor = None
    for s in profile.get_device().query_sensors():
        try:
            if s.get_info(rs.camera_info.name).lower().startswith("rgb"):
                sensor = s
                break
        except Exception:
            pass
    if sensor is None:
        out["errors"].append("rgb sensor 없음")
        return out

    def _set(opt, value, name):
        try:
            if not sensor.supports(opt):
                out["errors"].append(name + "(미지원)")
                return None
            rng = sensor.get_option_range(opt)
            v = min(max(float(value), rng.min), rng.max)   # 범위를 넘기면 예외가 난다
            sensor.set_option(opt, v)
            return sensor.get_option(opt)                  # 되읽기 — 짐작 금지
        except Exception as exc:
            out["errors"].append("%s(%s)" % (name, exc.__class__.__name__))
            return None

    if ae_priority is not None:
        v = _set(rs.option.auto_exposure_priority, float(ae_priority), "ae_priority")
        out["ae_priority"] = None if v is None else int(v)
    if exposure_us is not None:
        # us -> 100us 눈금. 함정 1 참고.
        v = _set(rs.option.exposure, float(exposure_us) / COLOR_EXPOSURE_UNIT_US, "exposure")
        out["exposure_us"] = None if v is None else float(v) * COLOR_EXPOSURE_UNIT_US
    if gain is not None:
        v = _set(rs.option.gain, float(gain), "gain")
        out["gain"] = None if v is None else float(v)
    try:
        if sensor.supports(rs.option.enable_auto_exposure):
            out["auto_exposure"] = bool(sensor.get_option(rs.option.enable_auto_exposure))
    except Exception:
        pass
    return out
