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
    """실물 카메라. 카메라를 사면 여기만 채우면 된다.

    블로그의 pyrealsense2 파이프라인이 들어갈 자리이기도 하다.
    """
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
