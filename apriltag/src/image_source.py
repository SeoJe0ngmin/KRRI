"""1단계 — 이미지 얻기.

블로그는 RealSense 에서 직접 받았다(pyrealsense2). 카메라가 아직 없으므로
여기서는 rosbag 과 이미지 파일에서 읽는 경로를 대신 둔다.
카메라를 사면 from_camera() 만 채우면 뒤쪽 단계는 그대로 돌아간다.

자세 추정에 필요한 카메라 값은 (fx, fy, cx, cy) 네 개뿐이다.
블로그는 이걸 RealSense 에서 읽어왔고, 우리는 yaml 이나 캘리브레이션에서 받는다.
"""
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

# ROS 의 Bayer 이름과 OpenCV 의 Bayer 이름은 한 픽셀 어긋나 있다.
# 그래서 ROS 가 rggb 라고 말해도 OpenCV 에는 BayerBG 를 넣어야 맞는 식이다.
# 틀린 패턴을 넣으면 색이 이상해지는 정도가 아니라 검출기가 세그폴트로 죽는다.
ROS_BAYER_TO_CV = {
    "bayer_rggb8": cv2.COLOR_BayerBG2BGR,
    "bayer_bggr8": cv2.COLOR_BayerRG2BGR,
    "bayer_gbrg8": cv2.COLOR_BayerGR2BGR,
    "bayer_grbg8": cv2.COLOR_BayerGB2BGR,
}


@dataclass
class CameraIntrinsics:
    """detection_pose() 에 넘길 카메라 값."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 0
    height: int = 0
    distortion: tuple = ()          # radtan (k1,k2,p1,p2,k3). 비어 있으면 왜곡 없음으로 본다

    @property
    def params(self):
        """apriltag 가 받는 (fx, fy, cx, cy) 튜플."""
        return (self.fx, self.fy, self.cx, self.cy)

    @property
    def K(self):
        """3x3 카메라 행렬."""
        return np.array([[self.fx, 0, self.cx],
                         [0, self.fy, self.cy],
                         [0, 0, 1]], dtype=np.float64)

    @classmethod
    def from_realsense(cls, profile, stream="color", index=1):
        """RealSense 가 들고 있는 공장 캘리브레이션 값을 그대로 읽는다.

        D435i 는 출고 시 캘리브레이션이 되어 있어 체스보드를 찍을 필요가 없다.
        블로그가 하던 것과 같은 경로다.

            profile = pipeline.start(config)
            intr = CameraIntrinsics.from_realsense(profile)

        Args:
            profile: pipeline.start(config) 가 돌려준 객체
            stream: "color" / "infrared" / "depth"
                **스트림마다 렌즈가 달라 내부파라미터도 다르다.** 영상을 받은 스트림과
                같은 것을 넣어야 한다. color 영상에 depth 값을 넣으면 거리가 틀어진다.
            index: infrared 일 때 좌(1)/우(2) 선택
        """
        import pyrealsense2 as rs

        kinds = {"color": rs.stream.color, "depth": rs.stream.depth,
                 "infrared": rs.stream.infrared}
        if stream not in kinds:
            raise ValueError("모르는 스트림: %s" % stream)
        if stream == "infrared":
            vsp = profile.get_stream(kinds[stream], index).as_video_stream_profile()
        else:
            vsp = profile.get_stream(kinds[stream]).as_video_stream_profile()
        i = vsp.get_intrinsics()
        # RealSense 는 주점을 ppx, ppy 로 부른다 (= cx, cy)
        return cls(fx=i.fx, fy=i.fy, cx=i.ppx, cy=i.ppy,
                   width=i.width, height=i.height,
                   distortion=tuple(i.coeffs))

    @classmethod
    def from_yaml(cls, path, cam="cam0"):
        """tagslam/kalibr 형식의 cameras.yaml 에서 읽는다."""
        import yaml
        d = yaml.safe_load(Path(path).read_text())[cam]
        fx, fy, cx, cy = d["intrinsics"]
        w, h = d.get("resolution", [0, 0])
        return cls(fx, fy, cx, cy, w, h, tuple(d.get("distortion_coeffs", ())))

    def undistort(self, img):
        """렌즈 왜곡을 편다. 왜곡계수가 없으면 그대로 돌려준다.

        detection_pose() 는 왜곡 없는 핀홀 카메라를 가정하므로,
        왜곡이 큰 렌즈에서는 이걸 먼저 통과시켜야 자세가 정확해진다.
        """
        if not self.distortion:
            return img
        d = np.array(self.distortion, dtype=np.float64)
        return cv2.undistort(img, self.K, d)


def debayer(mosaic, ros_encoding="bayer_rggb8"):
    """Bayer 모자이크 한 장 -> BGR 컬러.

    모자이크는 픽셀 하나가 R/G/B 중 하나만 담고 있는 상태라,
    그대로 검출기에 넣으면 체크무늬가 윤곽으로 잡혀 죽는다. 반드시 거쳐야 한다.
    """
    code = ROS_BAYER_TO_CV.get(ros_encoding)
    if code is None:
        raise ValueError("모르는 Bayer 인코딩: %s" % ros_encoding)
    return cv2.cvtColor(mosaic, code)


def to_gray(img):
    """검출기에 넣을 흑백 이미지. 컬러면 변환하고 이미 흑백이면 그대로.

    블로그 설명대로, 검출기는 밝기 채널만 쓰므로 흑백이 빠르고 패턴도 또렷하다.
    """
    if img.ndim == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def from_rosbag(bag_path, topic=None):
    """rosbag 에서 이미지를 순서대로 꺼낸다.

    Yields:
        (index, timestamp_sec, bgr_image)

    압축(CompressedImage)·비압축(Image) 둘 다 받고, Bayer 면 자동으로 편다.
    """
    from rosbags.highlevel import AnyReader

    with AnyReader([Path(bag_path)]) as reader:
        conns = [c for c in reader.connections if "Image" in c.msgtype]
        if topic:
            conns = [c for c in conns if c.topic == topic]
        if not conns:
            raise KeyError("이미지 토픽을 찾지 못했다: %s" % bag_path)
        t0 = reader.start_time
        for i, (conn, ts, raw) in enumerate(reader.messages(connections=conns)):
            msg = reader.deserialize(raw, conn.msgtype)
            fmt = getattr(msg, "format", "") or getattr(msg, "encoding", "")
            if hasattr(msg, "format"):                       # CompressedImage
                buf = np.frombuffer(msg.data, np.uint8)
                if "bayer" in fmt:
                    mosaic = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
                    bgr = debayer(mosaic, fmt.split(";")[0].strip())
                else:
                    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            else:                                            # Image (비압축)
                arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
                bgr = debayer(arr[:, :, 0], fmt) if "bayer" in fmt else arr
            yield i, (ts - t0) / 1e9, bgr


def from_files(folder, pattern="*.png"):
    """폴더 안의 이미지들을 이름순으로 꺼낸다.

    Yields:
        (index, filename, bgr_image)
    """
    for i, p in enumerate(sorted(Path(folder).glob(pattern))):
        img = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if img is None:
            continue
        yield i, p.name, img


def from_camera(index=0):
    """일반 USB 카메라. 내부파라미터는 별도로 캘리브레이션해야 한다."""
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없다: index=%d" % index)
    try:
        i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            yield i, cv2.getTickCount() / cv2.getTickFrequency(), frame
            i += 1
    finally:
        cap.release()


def open_realsense(stream="color", width=None, height=None, fps=30,
                   depth=False, emitter=None, ir_index=1):
    """RealSense 를 열고 (프레임 제너레이터, CameraIntrinsics) 를 돌려준다.

    블로그의 pyrealsense2 파이프라인을 우리 구조에 맞춰 옮긴 것이다.

        frames, intr = open_realsense()                    # RGB
        frames, intr = open_realsense(stream="infrared")   # IR (글로벌 셔터)
        for i, t, img in frames:
            results = detect(detector, to_gray(img))

    ── color 와 infrared 중 무엇을 쓸까 ────────────────────────────
    D435i 는 **RGB 가 롤링 셔터, IR(스테레오) 가 글로벌 셔터**다.
    주행 중 촬영이면 롤링 셔터가 화면을 비스듬히 밀어 자세를 왜곡시킨다.
    그래서 IR 쪽이 유리한데, 대신 해상도가 낮고 화각이 넓어 같은 태그가
    절반 크기로 찍힌다(fx 약 1386 -> 674). 인식 거리가 절반이 된다.

        RGB : 1920x1080, 69x42도, 롤링,  0.2m 태그를 3.1m 까지
        IR  : 1280x720,  87x58도, 글로벌, 0.2m 태그를 1.5m 까지

    IR 을 쓰면 태그를 2배로 키워야 같은 거리가 나온다.
    실물로 두 스트림을 비교해보고 정하는 게 맞다.

    Args:
        stream: "color" 또는 "infrared"
        emitter: IR 점 패턴 프로젝터. None 이면 stream 에 따라 자동
            (infrared 면 끔, color 면 건드리지 않음).
            **IR 영상을 쓸 때는 반드시 꺼야 한다.** 켜져 있으면 점 패턴이
            태그 무늬 위에 뿌려져 검출을 방해한다. 끄면 depth 품질이 떨어지지만
            자세 추정에는 depth 를 쓰지 않으므로 손해가 아니다.
        depth: True 면 depth 도 받아 color 에 정렬한다 (stream="color" 일 때만).
    """
    import pyrealsense2 as rs

    if width is None or height is None:
        width, height = (1920, 1080) if stream == "color" else (1280, 720)

    pipeline = rs.pipeline()
    config = rs.config()
    if stream == "color":
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    elif stream == "infrared":
        config.enable_stream(rs.stream.infrared, ir_index, width, height, rs.format.y8, fps)
    else:
        raise ValueError("stream 은 color 또는 infrared")
    if depth:
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, fps)

    profile = pipeline.start(config)

    # IR 점 프로젝터 제어
    want_emitter = (stream != "infrared") if emitter is None else bool(emitter)
    try:
        ds = profile.get_device().first_depth_sensor()
        if ds.supports(rs.option.emitter_enabled):
            ds.set_option(rs.option.emitter_enabled, 1 if want_emitter else 0)
    except Exception:
        pass                                    # 장치에 따라 없을 수 있다

    intr = CameraIntrinsics.from_realsense(profile, stream, ir_index)
    align = rs.align(rs.stream.color) if (depth and stream == "color") else None

    def frames():
        t0 = None
        i = 0
        try:
            while True:
                fs = pipeline.wait_for_frames()
                if align is not None:
                    fs = align.process(fs)
                f = fs.get_color_frame() if stream == "color" \
                    else fs.get_infrared_frame(ir_index)
                if not f:
                    continue
                ts = f.get_timestamp() / 1000.0
                if t0 is None:
                    t0 = ts
                img = np.asanyarray(f.get_data())      # color=BGR, infrared=흑백
                if depth:
                    df = fs.get_depth_frame()
                    yield i, ts - t0, img, (np.asanyarray(df.get_data()) if df else None)
                else:
                    yield i, ts - t0, img
                i += 1
        finally:
            pipeline.stop()

    return frames(), intr


def meters2xyz(depth_img, intrinsics):
    """깊이영상 -> 포인트클라우드. RGB-D 카메라를 쓸 때만 필요하다.

    자세 추정에는 쓰이지 않는다. 태그 자세는 호모그래피만으로 나오기 때문에
    깊이 없이도 구해진다. 이 함수는 나중에 탑재부 형상을 보거나,
    태그 거리를 깊이로 한 번 더 확인하고 싶을 때를 위해 남겨둔다.

    Args:
        depth_img: [H x W] 깊이영상. 단위는 입력 그대로 나온다(보통 mm 또는 m).
    Returns:
        [H x W x 3] 카메라 좌표계 점구름. 축 순서는 (전방, 좌, 상).

    [원본] joonhyung-lee/mujoco-robotics-usage · utils/util.py
           블로그 본문 버전에는 fy 자리에 fx 를 넣는 오타가 있어, 원본 쪽을 따랐다.
    """
    fx, fy, cx, cy = intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy
    height, width = depth_img.shape[:2]
    indices = np.indices((height, width), dtype=np.float32).transpose(1, 2, 0)

    z_e = depth_img
    x_e = (indices[..., 1] - cx) * z_e / fx
    y_e = (indices[..., 0] - cy) * z_e / fy

    return np.stack([z_e, -x_e, -y_e], axis=-1)


def remove_color_overlay(bgr, channel="green", margin=40, dilate=1, radius=3):
    """영상에 새겨진 색상 오버레이를 지운다.

    테스트 영상 Testing_apriltag.mp4 는 렌더링 단계에서 디버그 표시(초록 사각형,
    십자선, 글씨)를 **태그 위에** 그려 넣었다. 그 선이 태그의 검은 테두리와
    데이터 칸을 덮어서 검출이 실패한다. 실측: 17/80 -> 28/80 프레임으로 회복.

    실물 카메라 영상에는 이런 게 없으므로 그때는 쓰지 않는다.
    (다만 IR 스트림에서 emitter 를 안 끄면 점 패턴이 비슷한 문제를 만든다.
     그건 이 함수가 아니라 open_realsense(emitter=False) 로 막는다.)

    Args:
        channel: 지울 색 ("green" / "red" / "blue")
        margin: 다른 두 채널보다 이만큼 크면 그 색으로 본다
        dilate: 마스크를 몇 픽셀 부풀릴지 (선 가장자리까지 덮으려고)
        radius: inpaint 반경
    """
    if bgr.ndim != 3:
        return bgr                              # 흑백(IR)이면 색이 없다
    b, g, r = (bgr[:, :, i].astype(int) for i in range(3))
    if channel == "green":
        mask = (g - b > margin) & (g - r > margin)
    elif channel == "red":
        mask = (r - b > margin) & (r - g > margin)
    elif channel == "blue":
        mask = (b - g > margin) & (b - r > margin)
    else:
        raise ValueError("모르는 채널: %s" % channel)

    mask = mask.astype(np.uint8)
    if not mask.any():
        return bgr
    if dilate:
        mask = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=dilate)
    return cv2.inpaint(bgr, mask, radius, cv2.INPAINT_TELEA)
