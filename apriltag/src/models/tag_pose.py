"""AprilTag 도킹 파이프라인 — 이미지 한 장에서 도킹 값까지 한 파일에서.

    이미지 얻기 -> 태그 찾기 -> 자세 구하기 -> 품질 판정 -> 도킹 값
    (open_*)      (detect)     (estimate_pose) (pose_quality) (docking_state)

예전에는 image_source.py / detection.py / pose.py 세 파일이었다. 셋이 서로를
지연 import 로 참고하고 있었고(순환을 피하려던 것), 그 지연 import 가 실패해도
except 로 삼켜져 **조용히 다른 코드경로로 빠지는** 함정이 있었다. 한 파일이면
그 문제가 아예 없어진다. 대신 길어지므로 아래 6단계 배너로 나눠 둔다.

    1) 카메라 값     CameraIntrinsics
    2) 이미지 얻기   Frame, open_realsense, open_bag, from_video, to_gray
    3) 태그 찾기     make_detector, detect, tag_pixel_size
    4) 자세 구하기   estimate_pose, docking_state, tag_tilt_deg
    5) 품질 판정     pose_quality, depth_cross_check, 임계값들
    6) 파이프라인    TagPipeline, Result

6단계는 편의층일 뿐이다. **1~5 의 함수들은 그대로 따로 쓸 수 있고, 앞으로도
그래야 한다** — 도구/노트북이 중간 단계만 골라 쓰는 경우가 실제로 많다.

    from src.models.tag_pose import make_detector, detect, to_gray   # 낱개로
    with TagPipeline.from_video(path, tag_size=0.20) as pipe:        # 한 번에
        for res in pipe:
            print(res.primary()["docking"]["forward"])

소스가 무엇이든 프레임 3-튜플 계약은 하나다: `for i, ts, img in frames:`.
depth 를 같이 받아도 **튜플은 3개 그대로**이고 img 가 depth 를 달고 오는
Frame 이 된다(Frame / depth_at 참고). 이 계약을 깨면 소비자가 전부 죽는다.

검출은 `apriltag` (AT2) 패키지 0.0.16 을 쓴다. pupil_apriltags 가 같은 env 에
깔려 있지만 **쓰면 안 된다** — 여기서 기대는 detector.detection_pose() 가 없고
API 가 다르다. (설치본 원본: envs/krri/lib/python3.11/site-packages/apriltag.py)

[원본] 블로그: joonhyung-lee.github.io/blog/2023/apriltag-pose-estimation/
"""
from dataclasses import dataclass, field
from pathlib import Path

import apriltag
import cv2
import numpy as np

from ..utils.util import r2rpy, invert_T, t2pr
from ..utils.rs_tuning import COLOR_EXPOSURE_UNIT_US


# ===========================================================================
# 1) 카메라 값 — CameraIntrinsics
# ===========================================================================
#
# 자세 추정에 필요한 카메라 값은 (fx, fy, cx, cy) 네 개뿐이다.
# 블로그는 이걸 RealSense 에서 읽어왔고, 우리는 그 외에 yaml(kalibr/tagslam)이나
# 화각 가정에서도 받는다. **거리는 fx 에 정비례한다** — fx 가 틀리면 거리가
# 그 비율만큼 통째로 틀어지고, 그래도 화면은 멀쩡해 보인다.


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


# 보정값이 아예 없을 때 쓰는 수평화각 가정 [도].
# 웹캠/영상파일처럼 카메라 값을 모르는 입력에서만 쓴다.
ASSUMED_HFOV_DEG = 60.0


def intrinsics_from_hfov(shape, hfov_deg=ASSUMED_HFOV_DEG):
    """화면 크기 + 수평화각 가정으로 CameraIntrinsics 를 지어낸다.

    **지어낸 값이다.** 왜곡 없음, 주점은 화면 정중앙, fx=fy 로 둔다.
    영상파일처럼 카메라 값이 어디에도 안 적혀 있는 입력에서 "그래도 대충 몇
    미터인지는 보자"는 용도다.

    거리는 fx 에 정비례하므로 **화각 가정이 틀린 만큼 거리가 통째로 틀어진다.**
    60도 가정인데 실제가 69도(D435i 컬러)면 fx 가 약 14% 작아지고 거리도 그만큼
    짧게 나온다. 그래서 이 경로로 만든 값을 쓴 결과에는 반드시 ASSUMED 표시를
    달아야 한다 — TagPipeline 은 .intrinsics_assumed 로 그 사실을 들고 다닌다.
    """
    h, w = shape[:2]
    fx = (w / 2.0) / np.tan(np.deg2rad(float(hfov_deg)) / 2.0)
    return CameraIntrinsics(fx, fx, w / 2.0, h / 2.0, w, h)


# ===========================================================================
# 2) 이미지 얻기 — Frame / open_realsense / open_bag / from_video / to_gray
# ===========================================================================
#
# 어디서 오든 결과 모양은 하나다: `for i, ts, img in frames:` 3-튜플.
#     open_realsense()  실물 D435i
#     open_bag()        realsense-viewer 의 Record 로 뽑은 .bag 재생
#     from_video()      영상 파일 (cv2.VideoCapture)
# 그래서 소스를 갈아끼워도 뒤쪽 단계는 한 줄도 안 바뀐다.
# 튜닝은 .bag 으로 하는 게 좋다 — 카메라 없이, 매번 완전히 같은 입력으로 돌 수 있다.
#
# depth 를 같이 받고 싶으면 with_depth=True 를 주면 된다. 튜플이 4개로 늘어나는 게
# 아니라 img 가 depth 를 달고 오는 Frame 이 된다(Frame / depth_at 참고).
#
# 예전에 있던 from_rosbag() / from_files() / debayer() 는 지웠다 — 이 프로젝트에서
# 부르는 곳이 한 군데도 없었고, ROS Bayer 이름과 OpenCV Bayer 이름이 한 칸 어긋나는
# 함정만 계속 들고 다니게 된다. 필요해지면 git 이력에서 되살릴 것.


class Frame(np.ndarray):
    """BGR/흑백 이미지 그 자체이면서, 있으면 depth 를 곁들여 들고 다니는 배열.

    왜 이렇게 하나:
        기존 소비자는 전부 `for i, t, img in frames:` 로 **3-튜플**을 푼다
        (tools/live_pose.py). depth 를 4번째 칸으로
        내보내면 그 자리에서 ValueError 로 죽는다. 그래서 튜플 모양은 손대지 않고
        이미지 **자체**에 depth 를 붙여 보낸다. ndarray 를 상속했으므로
        img.shape / img.copy() / cv2.* 호출은 전부 그대로 돌아간다.

    Attributes:
        depth: [H x W] uint16 원시 깊이맵. 없으면 None.
            **미터가 아니라 장치 단위다.** depth_scale 을 곱해야 미터가 된다.
        depth_scale: 깊이 한 칸이 몇 미터인가. D435i 실측 0.0010000000474974513.
            0.0 이면 depth 가 없다는 뜻.
        luma: YUYV 로 받았을 때의 센서 원본 휘도 [H x W] uint8. 없으면 None.
            to_gray() 가 있으면 알아서 이걸 쓴다.

        ── 아래는 "이 프레임에서 왜 검출이 실패했나"를 사후에 가리기 위한 것들이다 ──
        frame_number: 카메라가 매긴 번호. **연속이 아니면 그 사이가 사라진 것이다.**
        dropped_before: 이 프레임 **바로 앞에서** 사라진 장 수. 0 이면 연속.
            pipeline 출력 큐가 용량 1 이라(aggregator.cpp:16) 우리 루프가 늦으면
            중간 프레임이 아무 소리 없이 버려진다. 그걸 여기서 알 수 있다.
        meta: 프레임 메타데이터 dict. **없으면 {}** — 리눅스에서 커널 패치가
            없으면 정상적으로 비어 있다(rs_tuning.frame_meta 설명 참고).
        exposure_us: 이 프레임의 실제 노출시간 [us]. 못 읽으면 None.
            **모션블러의 원인 변수다.** rs_tuning.motion_blur_px() 에 그대로 넣는다.
        gain: 이 프레임의 게인. 못 읽으면 None. 크면 어두워서 밀어올린 것 = 노이즈.

    쓰는 법:
        if not results:
            print(rs_tuning.diagnose_frame(img, fx=intr.fx, z_m=z, speed_mps=v))

    주의:
        cv2 함수를 통과하면 결과는 평범한 ndarray 라 depth 가 떨어져 나간다.
        (예: to_gray(img).depth 는 없다.) 이건 의도된 동작이다 — depth 는
        원본 프레임에 붙어 있는 것이고, 파생 이미지에 따라다닐 이유가 없다.
        depth 를 쓸 거면 generator 가 준 원본을 그대로 들고 있어야 한다.
    """
    depth = None
    depth_scale = 0.0
    luma = None
    frame_number = None
    dropped_before = 0
    # meta 는 **클래스 속성으로 dict 를 두면 안 된다** — 모든 Frame 이 같은 dict 를
    # 공유하게 된다. None 을 두고 _attach 에서 프레임마다 새로 넣는다.
    meta = None
    exposure_us = None
    gain = None

    def __array_finalize__(self, obj):
        # 뷰/슬라이스로 파생될 때도 속성이 따라가게 한다.
        if obj is None:
            return
        self.depth = getattr(obj, "depth", None)
        self.depth_scale = getattr(obj, "depth_scale", 0.0)
        self.luma = getattr(obj, "luma", None)
        self.frame_number = getattr(obj, "frame_number", None)
        self.dropped_before = getattr(obj, "dropped_before", 0)
        self.meta = getattr(obj, "meta", None)
        self.exposure_us = getattr(obj, "exposure_us", None)
        self.gain = getattr(obj, "gain", None)


# 컬러 센서의 노출 메타데이터 눈금. 컬러의 노출값은 UVC 원값이라 100us 눈금이고,
# 뎁스/IR 은 us 다. (SDK: ds-color-common.cpp:117 이 md_rgb_control::manual_exp 를
# 가공 없이 넘긴다.)
# **숫자는 rs_tuning 에서 가져온다.** 예전엔 여기에 100.0 을 한 번 더 적어 두고
# "rs_tuning 과 같은 값"이라고 주석만 달아 놨는데, 같은 숫자를 두 군데 적으면
# 한쪽만 고쳐지는 날이 반드시 온다. rs_tuning 은 모듈 최상단에서 numpy 만 쓰고
# src 안의 무엇도 import 하지 않으므로 여기서 위로 끌어와도 순환이 생기지 않는다.
_EXPOSURE_UNIT_US = {"color": COLOR_EXPOSURE_UNIT_US, "infrared": 1.0}


def _attach(img, depth=None, depth_scale=0.0, luma=None, frame_number=None,
            meta=None, dropped_before=0, exposure_unit_us=1.0):
    """이미지에 곁다리 정보를 붙여 Frame 으로 만든다.

    depth 가 없어도(None) Frame 으로 감싼다 — luma/메타데이터만 달고 갈 때가 있다.
    exposure_us / gain 은 meta 에서 꺼내 **미리 계산해 둔다.** 검출 실패 프레임마다
    호출자가 단위 환산을 다시 짜는 걸 막으려는 것이다(컬러 100us vs 뎁스 1us 함정).
    """
    f = img.view(Frame)
    f.depth = depth
    f.depth_scale = float(depth_scale)
    f.luma = luma
    f.frame_number = frame_number
    f.dropped_before = int(dropped_before or 0)
    f.meta = meta if meta is not None else {}
    exp = f.meta.get("actual_exposure")
    f.exposure_us = None if exp is None else float(exp) * float(exposure_unit_us)
    g = f.meta.get("gain_level")
    f.gain = None if g is None else int(g)
    return f


class _FrameStream:
    """프레임 제너레이터 + 곁다리 정보(ae_roi, profile).

    왜 클래스인가: 제너레이터 객체에는 __dict__ 가 없어 `gen.ae_roi = ...` 가
    AttributeError 로 죽는다. 그래서 감싼다. 밖에서 보이는 동작은 제너레이터와
    같다 — `for i, t, img in gen:` 도, `gen.close()` 도 그대로다
    (TagPipeline 의 `close=frames.close` 가 여기에 걸린다).

    Attributes:
        stats: rs_tuning.FrameStats. **호출자가 안 줘도 항상 붙어 있다.**
            프레임 드롭 회계는 공짜(정수 뺄셈 하나)인데 없으면 25~32fps 가
            "카메라가 못 낸 것"인지 "우리가 늦어 버려진 것"인지 영영 알 수 없어서다.
            루프가 끝나면 `print(frames.stats.summary())` 한 줄이면 된다.
        ae_roi: open_realsense(ae_roi=True) 일 때만 ExposureROI, 아니면 None.
        profile: pipeline.start() 반환값. rs_tuning.set_color_exposure 에 넘긴다.
    """

    def __init__(self, gen, ae_roi=None, profile=None, stats=None,
                 tuning=None, settings_before=None, cleanup=None):
        self._gen = gen
        self.ae_roi = ae_roi          # rs_tuning.ExposureROI 또는 None
        self.profile = profile        # pipeline.start() 가 준 것. 센서 옵션을 만질 때 쓴다
        self.stats = stats            # rs_tuning.FrameStats — **항상 있다**
        self.tuning = tuning          # tune= 을 줬을 때 {옵션: (전, 후)}. 아니면 None
        # 손대기 전 CameraSettings. **되돌리려면 이게 있어야 한다** —
        # UVC 값은 스트림을 닫아도 장치에 남아서, 다음에 realsense-viewer 를 열면
        # 우리가 박아 둔 8.3ms 를 그대로 물려받는다.
        self.settings_before = settings_before
        # 파이프라인 정지 + 카메라 상태 복원. **제너레이터가 아니라 여기가 들고 있다.**
        # 왜 여기냐 — 아래 close() 주석에 실측 근거가 있다.
        self._cleanup = cleanup

    def __iter__(self):
        return self

    def __next__(self):
        return next(self._gen)

    def close(self):
        """스트림을 닫고 카메라를 원래대로 돌려놓는다. 몇 번 불러도 안전하다.

        **왜 제너레이터의 finally 에만 맡기면 안 되는가 (실측):**
            한 번도 돌리지 않은 제너레이터는 close() 해도 **본문이 실행되지 않는다.**
            파이썬이 시작 안 된 제너레이터를 그냥 닫힌 것으로 표시하고 끝내기 때문에
            `try/finally` 의 finally 자체가 안 돈다. 그래서

                frames, intr = open_realsense(tune=True)
                frames.close()                 # 한 프레임도 안 받고 그만둘 때

            이 두 줄만으로 pipeline.stop() 이 **영영 안 불린다.** 실제로 재현했다 —
            바로 다음 open_realsense() 가 "Device or resource busy" 로 죽는다.
            게다가 tune= 은 pipeline.start() 직후에 이미 카메라를 만져 놨으므로
            노출 8.3ms / AE 꺼짐 상태가 장치에 그대로 남는다.
            그래서 정리 책임을 파이프라인을 실제로 소유한 **이 껍데기**로 올렸다.
            제너레이터의 finally 도 같은 함수를 부르고, 그 함수가 한 번만 돌게 잠근다.
        """
        try:
            self._gen.close()
        finally:
            c, self._cleanup = self._cleanup, None
            if c is not None:
                c()

    # `with open_realsense()[0] as frames:` 를 쓸 수 있게 한다.
    # 루프 중간에 break 하고 frames 를 계속 들고 있으면 close() 가 안 불려
    # 파이프라인이 잡힌 채로 남는다(실측: 다음 open 이 "Device or resource busy").
    # 제너레이터의 일반적인 성질이라 놀랄 일은 아니지만, 카메라는 파일과 달리
    # **한 프로세스만 열 수 있어** 대가가 크다. with 문이 가장 싼 예방책이다.
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def __del__(self):
        # 마지막 안전망. 참조가 사라질 때라도 파이프라인은 놓아 준다.
        # (인터프리터 종료 중에는 무엇이든 이미 없을 수 있어 통째로 삼킨다.)
        try:
            self.close()
        except Exception:
            pass

    def send(self, value):
        return self._gen.send(value)

    def throw(self, *a, **kw):
        return self._gen.throw(*a, **kw)


def open_realsense(stream="color", width=None, height=None, fps=30,
                   depth=False, emitter=None, ir_index=1,
                   with_depth=False, depth_size=(1280, 720), depth_fps=None,
                   color_format="bgr8", ae_roi=False, stats=None, meta=True,
                   exposure_us=None, ae_priority=None, tune=False, restore=True):
    """RealSense 를 열고 (프레임 제너레이터, CameraIntrinsics) 를 돌려준다.

    블로그의 pyrealsense2 파이프라인을 우리 구조에 맞춰 옮긴 것이다.

        frames, intr = open_realsense()                    # RGB
        frames, intr = open_realsense(stream="infrared")   # IR (글로벌 셔터)
        for i, t, img in frames:
            results = detect(detector, to_gray(img))

        frames, intr = open_realsense(with_depth=True)     # depth 도 같이
        for i, t, img in frames:                           # 튜플 모양은 그대로 3개다
            z = depth_at(img, *r.corners.mean(axis=0))     # 태그 중심 실측 거리 [m]

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
            (with_depth=True 로 거리를 대조할 거라면 얘기가 다르다. 그때는
             emitter=True 를 명시해 켜 두는 편이 depth 구멍이 훨씬 적다.)
        with_depth: True 면 depth 도 받아 이미지에 붙여 보낸다(Frame).
            **튜플 길이는 안 바뀐다** — 여전히 (i, ts, img) 3개다.
            stream="color" 면 depth 를 color 에 정렬(rs.align)하므로
            img.depth.shape == img.shape[:2] 이고, 반환된 color 내부파라미터로
            그대로 인덱싱하면 된다.
            stream="infrared" 면 정렬하지 않는다 — D435i 의 depth 는 원래
            **좌측 IR(ir_index=1) 에 등록**되어 있어 그대로 픽셀이 맞는다.
            ir_index=2 와는 맞지 않으니 그 조합으로 거리를 읽으면 안 된다.
        depth: with_depth 의 옛 이름. 예전에는 4-튜플을 뱉었지만 이제 with_depth 와
            같은 동작(3-튜플 + Frame)이다. 쓰던 코드가 깨지지 않게 남겨둔 별칭이다.
        depth_size: depth 스트림 해상도. width/height 와 별개다 —
            D435i 는 depth 가 지원하는 조합이 color 와 다르다.
        depth_fps: depth 프레임률. None 이면 min(fps, 30).
            **fps 를 그대로 쓰면 안 된다** — 1280x720 depth 는 30fps 가 최대라
            fps=60 으로 넘기면 "Couldn't resolve requests" 로 시작 자체가 실패한다(실측).
        color_format: "bgr8"(기본, 예전과 동일) 또는 "yuyv".
            stream="color" 일 때만 의미가 있다.

            "yuyv" 는 센서가 내보내는 **원본 포맷을 그대로** 받아온다.
            그러면 img.luma 에 손대지 않은 휘도 평면이 실려 오고 to_gray() 가
            그걸 그대로 쓴다. 왜 이게 나은가:
                bgr8 을 요구하면 SDK 가 BT.601 limited range 로 변환한다
                (proc/color-formats-converter.cpp: c=y-16, 298*c>>8, 0~255 클램프).
                우리가 다시 BGR2GRAY 로 되돌린다. 이 왕복에서 **Y<=16 은 전부 0,
                Y>=235 는 전부 255 로 뭉개져 256 단계 중 38 단계가 사라진다**(실측).
                AprilTag 의 refine_edges 는 경사로 모서리를 서브픽셀까지 당기는데,
                클리핑된 영역은 경사가 0 이라 모서리가 그 자리에 박힌다.
            비용은 1920x1080 기준 프레임당 약 0.6 ms 다(YUV2BGR 0.13 + Y 복사 0.46,
            실측). 30fps 예산 33 ms 안에서 무시할 만하다.
            **img 자체는 여전히 BGR 3채널**이라 그리기/표시 코드는 그대로 돈다.
        ae_roi: True 면 ExposureROI 객체를 하나 만들어 제너레이터에 붙여 준다
            (frames.ae_roi). 자동노출을 태그에만 걸고 싶을 때 쓴다 —
            역광 도크에서 검출률을 가르는 물건이다. rs_tuning.ExposureROI 참고.
            **여기서는 만들어만 준다.** 매 프레임 roi.follow(results, img.shape) 를
            불러 주는 건 호출자 몫이다(검출 결과를 여기서 모르기 때문).
        stats: rs_tuning.FrameStats 인스턴스를 주면 그걸 쓴다. **안 줘도 하나 만든다**
            (frames.stats 로 꺼낸다). 드롭 회계는 정수 뺄셈 하나라 사실상 공짜인데,
            없으면 25~32fps 가 "카메라가 못 냈다"인지 "우리가 늦어 버려졌다"인지
            구분할 방법이 아예 없다. pipeline 출력 큐가 용량 1 이고 넘치면 오래된
            것을 조용히 버리기 때문이다(aggregator.cpp:16).
                for i, t, img in frames: ...
                print(frames.stats.summary())   # "받음 288 / 보냄 300, 버림 12 (4.0%)"
        meta: True(기본)면 프레임마다 노출/게인 메타데이터를 읽어 Frame 에 붙인다
            (img.exposure_us / img.gain / img.meta). 검출이 끊긴 프레임의 원인을
            노출(모션블러) / 게인(노이즈) / 드롭 셋으로 가르기 위한 것이다 —
            rs_tuning.diagnose_frame() 이 이 셋을 읽는다.
            첫 프레임에서 살아 있는 항목만 한 번 조사하고 이후로는 그것만 읽는다
            (rs_tuning.MetaReader). **리눅스에서는 대개 하나도 안 온다** —
            UVC 벤더 메타데이터에 커널 패치가 필요하다. 그 경우 img.meta 는 {} 이고
            비용도 0 이 된다. 살아 있는지는 src/etc/realsense_check.py 4단계로 본다.
        exposure_us: 주면 컬러 센서를 **수동 노출**로 못박는다 [us]. None 이면
            건드리지 않는다(= 예전과 같은 자동노출).
            왜 있는가: 실측상 검출을 끊는 것은 밝기가 아니라 **모션블러**다.
            블러 10px 까지 28/28(100%), 32px 에서 0/28 로 뚝 끊긴다. 어두운 창고에서
            자동노출은 노출을 프레임시간 전체(33.3ms)까지 늘리는데, 1.0 m/s 로
            접근하는 지게차는 z=1m 에서 그게 45px 이라 **아예 안 잡힌다.**
            7.4ms 로 끊으면 10px 이라 100% 다.
            **주의: 노출을 고정하면 SDK 가 자동노출을 같이 꺼버린다**(의도된 동작,
            ds-color-common.cpp:90-93 의 auto_disabling_control). 조명이 일정한
            현장에서만 쓰고, 오락가락하면 대신 ae_roi=True 를 쓰는 편이 낫다.
            단위 함정과 근거는 rs_tuning.set_color_exposure() 에 다 적어 두었다.
        ae_priority: 컬러 센서의 auto_exposure_priority. 0 이면 "fps 를 지켜라"
            (노출이 프레임시간을 못 넘는다), 1 이면 "어두우면 fps 를 떨궈도 좋다".
            30fps 를 보장해야 하는 도킹에서는 0 이다. 다만 0 이어도 상한이 33.3ms 라
            블러는 못 막는다 — 그건 exposure_us 로 끊어야 한다.
            (rs.option.auto_exposure_limit 은 **컬러에 없다.** SDK 가 뎁스 센서에만,
             그것도 글로벌 셔터일 때만 등록한다 — d400-device.cpp:1062-1070.
             D435i RGB 는 롤링 셔터다. enum 이 있다고 부르면 그냥 실패한다.)
        tune: 컬러 센서를 **한 번에** 태그 검출용 상태로 맞춘다.
            **기본 False — 아무것도 안 건드린다. 예전과 완전히 같다.**
            False           손대지 않는다(공장 자동노출 그대로).
            True            camera_control.CameraSettings.docking() 을 건다:
                            자동노출 끔 + 노출 8.3ms 고정 + 게인 64 + 60Hz +
                            프레임률 고정 + AWB 잠금 + 큐 깊이 1 + 글로벌 시각.
            CameraSettings  직접 만든 묶음.
            숫자             접근속도 [m/s] 로 보고 노출을 거기 맞춰 계산한다
                            (tune=1.5 -> 1.5 m/s 를 견디는 노출).

            exposure_us / ae_priority 와의 관계: **tune 이 먼저 깔리고 그 뒤에
            exposure_us / ae_priority 가 덮어쓴다.** 그러니 둘을 같이 주면
            개별 인자가 이긴다. 전체를 한 번에 세팅하되 노출만 현장값으로
            바꿔 보고 싶을 때 그렇게 쓴다.

            왜 묶음이 따로 필요한가 — exposure_us 하나로는 **되돌릴 수가 없다.**
            UVC 값은 스트림을 닫아도 장치에 남는다. tune 을 쓰면 손대기 전 상태를
            frames.settings_before 에 떠 두므로 원상복구가 된다:

                frames, intr = open_realsense(tune=True)
                print(frames.settings_before.describe())      # 손대기 전
                for k,(b,a) in frames.tuning.items(): ...     # 전/후 표
                ...
                frames.settings_before.apply(frames.profile)  # 원상복구

            **다만 이제는 손으로 안 불러도 된다 — restore 를 보라.**
        restore: True(기본)면 스트림을 닫을 때 **카메라를 손대기 전 상태로 되돌린다**
            (노출/AE/AWB/전원주파수/큐깊이/글로벌시각 + AE ROI).
            tune / exposure_us / ae_priority 중 하나라도 줬을 때만 의미가 있다 —
            아무것도 안 건드렸으면 되돌릴 것도 없다.

            **왜 기본이 True 인가:** UVC PU 값과 AE ROI 는 파이프라인을 닫아도,
            프로세스가 죽어도 **장치에 남는다.** 예전에는 아무도 되돌리지 않아서
            `tools/live_pose.py --exposure-ms 7.4` 를 한 번 돌리고 realsense-viewer 를
            열면 AE 가 꺼진 7.4ms 고정 화면이 나왔다(실측 확인). 카메라가
            고장난 것처럼 보이는데 원인은 우리다. 되돌리는 값은 옵션 몇 개
            쓰기(수 ms)뿐이라 안 할 이유가 없다.

            되돌리기는 **어느 경로로 끝나든** 정확히 한 번 돈다 —
            close() / with 문 / 루프 소진 / 예외 / 참조 소멸 전부.
            (한 프레임도 안 받고 close() 하는 경우까지 포함한다.
             그 경우 제너레이터 finally 는 안 돌기 때문에 특별히 따로 처리했다.)

            False 로 두는 경우: 여러 스트림을 이어 열면서 노출을 유지하고 싶을 때,
            또는 일부러 카메라를 그 상태로 남겨 realsense-viewer 로 확인하고 싶을 때.
            그때 되돌릴 재료는 frames.settings_before 에 그대로 있다.
    """
    import pyrealsense2 as rs

    want_depth = bool(depth or with_depth)

    if width is None or height is None:
        width, height = (1920, 1080) if stream == "color" else (1280, 720)

    pipeline = rs.pipeline()
    config = rs.config()
    if stream == "color":
        if color_format not in ("bgr8", "yuyv"):
            raise ValueError("color_format 은 bgr8 또는 yuyv")
        # D400 컬러 센서가 실제로 내줄 수 있는 포맷은 RGB8/RGBA8/BGR8/BGRA8/YUYV 뿐이다
        # (device.cpp:197 map_supported_color_formats). Y8 은 목록에 없어서
        # "휘도만 달라"고는 못 한다 — YUYV 를 받아 우리가 Y 를 떼는 수밖에 없다.
        cfmt = rs.format.bgr8 if color_format == "bgr8" else rs.format.yuyv
        config.enable_stream(rs.stream.color, width, height, cfmt, fps)
    elif stream == "infrared":
        config.enable_stream(rs.stream.infrared, ir_index, width, height, rs.format.y8, fps)
    else:
        raise ValueError("stream 은 color 또는 infrared")
    if want_depth:
        dw, dh = depth_size
        config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16,
                             int(depth_fps or min(fps, 30)))

    try:
        profile = pipeline.start(config)
    except RuntimeError as exc:
        # D400 은 한 프로세스에서 하나만 열 수 있다. 이 실패의 압도적 다수는
        # **직전 스트림을 안 닫은 것**이다 — 루프에서 break 하고 frames 를 계속
        # 들고 있으면 제너레이터가 살아 있어 파이프라인도 잡힌 채로 남는다(실측).
        # SDK 문구("Device or resource busy")만으로는 원인이 안 보여서 덧붙인다.
        if "busy" in str(exc).lower():
            raise RuntimeError(
                "%s — 앞서 연 스트림을 아직 안 닫았을 가능성이 높다. "
                "루프에서 break 했으면 frames.close() 를 부르거나 "
                "`with open_realsense()[0] as frames:` 로 열어라. "
                "(realsense-viewer 등 다른 프로세스가 잡고 있어도 같은 오류가 난다.)"
                % exc) from exc
        raise

    # IR 점 프로젝터 제어 + depth 눈금 읽기 (둘 다 depth 센서에 달려 있다)
    want_emitter = (stream != "infrared") if emitter is None else bool(emitter)
    depth_scale = 0.0
    try:
        ds = profile.get_device().first_depth_sensor()
        if ds.supports(rs.option.emitter_enabled):
            ds.set_option(rs.option.emitter_enabled, 1 if want_emitter else 0)
        if want_depth:
            # 원시 uint16 을 미터로 바꾸는 눈금. 0.001 로 박아두지 말고 장치에서 읽는다.
            depth_scale = ds.get_depth_scale()
    except Exception:
        pass                                    # 장치에 따라 없을 수 있다

    intr = CameraIntrinsics.from_realsense(profile, stream, ir_index)
    align = rs.align(rs.stream.color) if (want_depth and stream == "color") else None

    # 컬러 자동노출 ROI. 태그를 찾은 뒤 호출자가 roi.follow(...) 를 불러 준다.
    roi = None
    if ae_roi:
        from ..utils.rs_tuning import ExposureROI
        roi = ExposureROI(profile, stream=stream)

    # 묶음 튜닝. **pipeline.start() 뒤, 첫 wait_for_frames 전에** 건다 —
    # 그래야 첫 프레임부터 우리가 정한 노출로 온다(AE 가 한두 프레임 사냥하다
    # 꺼지는 구간이 없어진다).
    # 개별 인자(exposure_us / ae_priority)보다 **먼저** 깐다. 그래야 둘을 같이 줬을 때
    # 개별 인자가 이긴다 — "전체를 세팅하되 노출만 손으로" 가 자연스러운 순서다.
    want_tune = (tune is not False and tune is not None and stream == "color")
    want_exposure = ((exposure_us is not None or ae_priority is not None)
                     and stream == "color")

    # **카메라를 만지기 전에 지금 상태를 뜬다.**
    # tune= 뿐 아니라 exposure_us / ae_priority 로 노출만 건드릴 때도 뜬다 —
    # 예전에는 tune= 일 때만 떠서, `tools/live_pose.py --exposure-ms 7.4` 로 돌리면
    # 되돌릴 재료 자체가 없었다. UVC 값은 스트림을 닫아도 장치에 남으므로
    # 그 뒤 realsense-viewer 를 열면 AE 꺼진 7.4ms 고정을 그대로 물려받는다(실측).
    tuning = None
    settings_before = None
    if want_tune or want_exposure:
        try:
            from ..utils.camera_control import CameraSettings as _CS
            settings_before = _CS.from_sensor(profile)
        except Exception:
            settings_before = None            # 못 뜨면 복원도 포기한다(아래 경고)

    if want_tune:
        from ..utils.camera_control import CameraSettings, tune_for_tags
        if isinstance(tune, CameraSettings):
            _b, tuning = tune_for_tags(profile, settings=tune)
        elif tune is True:
            _b, tuning = tune_for_tags(profile)
        else:
            # 숫자 = 접근속도 [m/s]. 그 속도에서 블러가 10px 를 넘지 않게 노출을 잡는다.
            # fx 는 지금 연 스트림의 실제 값을 쓴다 — 해상도를 낮추면 fx 도 줄어
            # 같은 속도라도 블러 픽셀수가 달라지기 때문이다.
            _b, tuning = tune_for_tags(profile, speed_mps=float(tune), fx=intr.fx)
        settings_before = settings_before or _b

    # 노출/AE 우선순위. **pipeline.start() 뒤, 첫 wait_for_frames 전에** 걸어야 한다.
    if want_exposure:
        from ..utils.rs_tuning import set_color_exposure
        applied = set_color_exposure(profile, exposure_us=exposure_us,
                                     ae_priority=ae_priority)
        if applied["errors"]:
            # 조용히 실패하면 "걸었다고 믿는" 상태가 된다. 그게 제일 나쁘다.
            import warnings
            warnings.warn("컬러 노출 설정 실패: %s" % ", ".join(applied["errors"]))

    if restore and (want_tune or want_exposure) and settings_before is None:
        import warnings
        warnings.warn("카메라 상태를 뜨지 못해 원상복구를 못 한다 — "
                      "끝난 뒤 realsense-viewer 가 우리 노출을 물려받는다")

    # AE ROI 도 펌웨어에 남는 상태다. 걸기 전 상자를 기억해 둔다.
    # 기본 상자(가운데 3/4)로 되돌리는 것과 원래 상자로 되돌리는 것은 다르다 —
    # 사용자가 뷰어에서 직접 잡아 둔 상자였을 수 있다.
    roi_before = None
    if roi is not None and roi.supported:
        try:
            from ..utils.camera_control import ae_roi_of
            roi_before = ae_roi_of(profile)
        except Exception:
            roi_before = None

    # 드롭 회계는 호출자가 안 줘도 항상 돈다 — 정수 뺄셈 하나 값이다.
    if stats is None:
        from ..utils.rs_tuning import FrameStats
        stats = FrameStats()
    reader = None
    if meta:
        from ..utils.rs_tuning import MetaReader
        reader = MetaReader()
    exp_unit = _EXPOSURE_UNIT_US.get(stream, 1.0)

    want_yuyv = (stream == "color" and color_format == "yuyv")

    # ── 정리(정지 + 원상복구). 어느 경로로 끝나든 **정확히 한 번** 돈다 ──────────
    #
    # 왜 카메라를 되돌려야 하나: UVC PU 값(노출/AE/AWB/전원주파수)과 AE ROI 는
    # **파이프라인을 닫아도 장치에 남는다.** 우리 도구를 돌린 뒤 realsense-viewer 를
    # 열면 AE 가 꺼진 8.3ms 고정 화면이 나오고, 사용자는 카메라가 고장난 줄 안다.
    # 되돌리는 비용은 옵션 몇 개 쓰기(수 ms)뿐이라 안 할 이유가 없다.
    #
    # 왜 pipeline.stop() **앞**에서 되돌리나: 옵션 쓰기는 장치 핸들이 살아 있어야
    # 한다. 먼저 stop 하면 되돌리기가 조용히 실패할 여지가 생긴다.
    _done = []

    def _cleanup():
        if _done:
            return                            # close() 와 제너레이터 finally 가 둘 다 부른다
        _done.append(True)
        if restore:
            if roi_before is not None:
                try:
                    from ..utils.camera_control import aim_ae_at_bbox
                    aim_ae_at_bbox(profile, roi_before, (height, width), pad=0.0)
                except Exception:
                    pass                      # 되돌리기 실패로 종료를 막지는 않는다
            if settings_before is not None:
                try:
                    settings_before.apply(profile)
                except Exception:
                    pass
        try:
            pipeline.stop()
        except Exception:
            pass

    def frames():
        t0 = None
        i = 0
        try:
            while True:
                # wait_for_frames 는 절대 밀리지 않는다 — pipeline 의 출력 큐가
                # 용량 1 이고(aggregator.cpp:16) 넘치면 오래된 것을 버리기 때문이다.
                # 즉 항상 최신 프레임이지만, 우리가 느리면 그 사이는 조용히 사라진다.
                # rs2::syncer 를 따로 끼울 이유는 없다 — pipeline 이 이미
                # 내부에 syncer_process_unit 을 물고 있다(pipeline.cpp:219).
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
                buf = np.asanyarray(f.get_data())
                luma = None
                if want_yuyv:
                    # 파이썬 래퍼는 YUYV 를 (H, W) uint16 으로 준다 —
                    # (H, W, 2) uint8 이 아니다(pyrs_frame.cpp get_frame_data).
                    from ..utils.rs_tuning import yuyv_to_luma
                    luma = yuyv_to_luma(buf)
                    # 표시/그리기용 BGR 은 여기서 만든다. 소비자 눈에는 예전과 같은
                    # 3채널 BGR 이라 tools/live_pose.py 는 한 줄도 안 바뀐다.
                    # **(H, W, 2) 로 접어야 한다.** cv2 의 YUY2 변환은 2채널
                    # CV_8UC2 만 받는다 — (H, 2W) 1채널을 주면 그 자리에서
                    # "Bad number of channels" 로 죽는다(실측).
                    img = cv2.cvtColor(
                        buf.view(np.uint8).reshape(buf.shape[0], buf.shape[1], 2),
                        cv2.COLOR_YUV2BGR_YUY2)
                else:
                    img = buf                         # color=BGR, infrared=흑백
                dm = None
                if want_depth:
                    df = fs.get_depth_frame()
                    dm = np.asanyarray(df.get_data()) if df else None
                # 드롭 회계를 먼저 돌려 "이 프레임 앞에서 몇 장 사라졌나"를 받아온다.
                # 그래야 그 값을 이 프레임에 실어 보낼 수 있다(Frame.dropped_before).
                fn = f.get_frame_number()
                missed = stats.update(fn, ts - t0)
                img = _attach(img, dm, depth_scale, luma=luma, frame_number=fn,
                              meta=(reader.read(f) if reader is not None else None),
                              dropped_before=missed, exposure_unit_us=exp_unit)
                # depth 를 켜든 말든 항상 3-튜플이다. 소비자(tools/live_pose.py)가
                # `for i, _ts, frame in ...` 로 푸는 모양을 절대 바꾸면 안 된다.
                yield i, ts - t0, img
                i += 1
        finally:
            _cleanup()

    # 반환 튜플 모양(gen, intr)과 `for i, t, img in gen:` / gen.close() 는 그대로 두고,
    # ae_roi / profile 만 곁다리로 달기 위한 얇은 껍데기다.
    # (제너레이터 객체에는 __dict__ 가 없어 속성을 못 붙인다 — 그래서 감싼다.)
    # cleanup 을 껍데기에도 넘기는 이유는 _FrameStream.close() 주석에 있다 —
    # **한 번도 안 돌린 제너레이터는 close() 해도 finally 가 안 돈다.**
    return _FrameStream(frames(), ae_roi=roi, profile=profile, stats=stats,
                        tuning=tuning, settings_before=settings_before,
                        cleanup=_cleanup), intr


def open_bag(path, loop=False, realtime=False, stream="color",
             with_depth=False, ir_index=1, timeout_ms=2000, meta=True, tune=False):
    """realsense-viewer(또는 rs-record)로 녹화한 .bag 을 재생한다.

    open_realsense 와 **똑같이** (프레임 제너레이터, CameraIntrinsics) 를 돌려주므로
    소스를 한 줄만 바꿔치기하면 된다.

        frames, intr = open_bag("/data/dock_01.bag")
        for i, t, img in frames:
            ...

    왜 필요한가:
        카메라 없이, 그리고 **완벽히 똑같은 입력**으로 알고리즘을 튜닝할 수 있다.
        임계값 하나 바꾸고 같은 장면을 다시 돌려 숫자를 비교하는 게 가능해진다.

    Args:
        path: .bag 파일 경로
        loop: True 면 끝나면 처음으로 되감아 계속 돈다(repeat_playback).
            **이러면 제너레이터가 영원히 안 끝난다.** 호출자가 멈춰야 한다.
        realtime: 기본 False.
            False = 소비하는 속도에 맞춰 재생한다. **한 프레임도 안 버린다.**
                    튜닝할 때는 이쪽이어야 한다. 처리에 100ms 가 걸려도
                    프레임이 건너뛰지 않는다.
            True  = 벽시계 속도로 재생한다. 처리가 느리면 녹화 때와 똑같이
                    프레임을 흘려버린다. 실시간 동작을 재현해볼 때만 쓴다.
        stream: "color" 또는 "infrared". 녹화에 그 스트림이 없으면 즉시 터진다.
        with_depth: open_realsense 와 같다. bag 에 depth 가 녹화돼 있어야 한다.
        ir_index: infrared 일 때 좌(1)/우(2)
        timeout_ms: 프레임을 이만큼 기다려도 안 오면 파일 끝으로 본다.
        meta: open_realsense 와 같다. **녹화 당시의** 노출/게인이 bag 에 같이
            들어 있으면 그대로 살아난다 — 현장에서 찍어 온 bag 을 놓고
            "그때 왜 못 잡았나"를 따질 수 있다는 뜻이다(diagnose_frame).
            녹화한 PC 에 커널 패치가 없었으면 당연히 비어 있다.

    Returns:
        (frames, CameraIntrinsics)
        frames 는 open_realsense 와 같은 껍데기다 — `for i, t, img in frames:`,
        frames.close(), frames.stats 가 전부 그대로 된다.
        **realtime=False 면 frames.stats 의 버림은 0 이어야 정상이다**(한 프레임도
        안 버리는 재생이므로). 0 이 아니면 녹화 당시에 이미 빠진 것이다.
        내부파라미터는 **bag 자신의 스트림 프로파일**에서 읽는다.
        지금 꽂혀 있는 카메라나 손으로 적은 값을 쓰면 거리가 조용히 틀어진다.
        (녹화한 해상도가 다르면 fx/cx 가 통째로 다른 값이다.)

        tune: **재생에서는 아무것도 못 바꾼다.** open_realsense 와 인자 이름을
            맞춰 두기 위해 받되, 하는 일은 "녹화 당시 카메라가 어떤 설정이었는지"를
            읽어 frames.settings_before 에 담아 주는 것뿐이다(frames.tuning 은 None).

            왜 쓰기가 아니라 읽기인가: playback_device 의 옵션은 녹화된 값을
            재생하는 것이라 써 봐야 픽셀이 안 바뀐다. 이미 찍힌 프레임의 노출을
            뒤늦게 바꿀 수는 없다. 그런데 **읽기는 진짜 쓸모가 있다** —
            검출이 안 되는 bag 을 받았을 때 "노출이 33ms 로 붙어 있었네"가
            한 줄로 나온다. 그게 곧 모션블러 진단이다.

                frames, intr = open_bag("dock_01.bag", tune=True)
                print(frames.settings_before.describe())

            주의: playback 은 옵션 **범위**를 녹화된 한 값으로 뭉개므로
            min/max/step 은 믿을 수 없다. 값 자체는 진짜다.

    타임스탬프는 open_realsense 와 같은 규칙 — 첫 프레임 기준 0.0 초부터의 초.
    """
    import pyrealsense2 as rs

    p = Path(path)
    if not p.exists():
        raise FileNotFoundError("bag 이 없다: %s" % p)
    if stream not in ("color", "infrared"):
        raise ValueError("stream 은 color 또는 infrared")

    pipeline = rs.pipeline()
    config = rs.config()
    # 스트림을 enable_stream 으로 지정하지 않는다 — 녹화에 들어 있는 조합을
    # 그대로 재생하는 게 안전하다. 해상도/포맷을 다시 요구하면 안 맞을 때 터진다.
    config.enable_device_from_file(str(p), repeat_playback=bool(loop))

    profile = pipeline.start(config)
    playback = profile.get_device().as_playback()
    # 기본이 False 인 게 핵심이다. True 면 벽시계로 밀어붙여 프레임을 버린다.
    playback.set_real_time(bool(realtime))

    try:
        intr = CameraIntrinsics.from_realsense(profile, stream, ir_index)
    except Exception:
        pipeline.stop()
        raise KeyError("bag 에 %s 스트림이 없다: %s" % (stream, p))

    depth_scale = 0.0
    if with_depth:
        try:
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        except Exception:
            depth_scale = 0.001                 # 녹화에 depth 센서 정보가 없을 때의 최후값
    align = rs.align(rs.stream.color) if (with_depth and stream == "color") else None

    from ..utils.rs_tuning import FrameStats
    stats = FrameStats()
    reader = None
    if meta:
        from ..utils.rs_tuning import MetaReader
        reader = MetaReader()
    exp_unit = _EXPOSURE_UNIT_US.get(stream, 1.0)

    # open_realsense 와 같은 이유로 껍데기가 정리를 들고 있는다 —
    # 한 프레임도 안 받고 close() 하면 제너레이터 finally 가 아예 안 돌아
    # 재생 파이프라인이 열린 채로 남는다(_FrameStream.close() 주석 참고).
    _done = []

    def _cleanup():
        if _done:
            return
        _done.append(True)
        try:
            pipeline.stop()
        except Exception:
            pass

    def frames():
        t0 = None
        i = 0
        try:
            while True:
                # 파일 끝은 예외가 아니라 상태로도 온다. 둘 다 본다.
                # (non-realtime 재생은 EOF 에서 wait_for_frames 가 그냥 안 돌아온다.)
                try:
                    ok, fs = pipeline.try_wait_for_frames(timeout_ms)
                except RuntimeError:
                    break                       # wait 계열이 던지는 EOF/타임아웃
                if not ok:
                    break                       # 더 줄 프레임이 없다 = 파일 끝
                if not loop and playback.current_status() == rs.playback_status.stopped:
                    break
                if align is not None:
                    fs = align.process(fs)
                f = fs.get_color_frame() if stream == "color" \
                    else fs.get_infrared_frame(ir_index)
                if not f:
                    continue
                ts = f.get_timestamp() / 1000.0
                if t0 is None:
                    t0 = ts
                img = np.asanyarray(f.get_data())
                dm = None
                if with_depth:
                    df = fs.get_depth_frame()
                    dm = np.asanyarray(df.get_data()) if df else None
                fn = f.get_frame_number()
                missed = stats.update(fn, ts - t0)
                img = _attach(img, dm, depth_scale, frame_number=fn,
                              meta=(reader.read(f) if reader is not None else None),
                              dropped_before=missed, exposure_unit_us=exp_unit)
                # open_realsense 와 같은 3-튜플. 소스를 바꿔 끼워도 소비자가 그대로다.
                yield i, ts - t0, img
                i += 1
        finally:
            _cleanup()

    # open_realsense 와 같은 껍데기로 감싼다 — frames.stats 를 밖에서 볼 수 있게.
    # (반복/close 동작은 제너레이터와 동일하다.)
    # 재생 장치는 옵션을 쓸 수 없다 — 녹화된 값을 되읽어 주기만 한다.
    # (그래도 값 자체는 진짜라, 검출이 안 되는 bag 의 노출을 확인하는 데 쓸모가 있다.)
    settings_before = None
    if tune is not False and tune is not None:
        try:
            from ..utils.camera_control import CameraSettings
            settings_before = CameraSettings.from_sensor(profile)
        except Exception:
            settings_before = None              # 녹화에 컬러 센서 정보가 없을 수 있다

    return _FrameStream(frames(), profile=profile, stats=stats,
                        settings_before=settings_before, cleanup=_cleanup), intr


def from_video(path, loop=False):
    """영상 파일을 3-튜플로 흘린다. 돌려주는 건 (frames, None) 이다.

    **두 번째 칸이 None 인 게 핵심이다.** 영상파일에는 카메라 값이 어디에도
    안 적혀 있다. open_realsense/open_bag 이 (frames, intr) 를 주는 것과 모양을
    맞추되, 여기서는 "모른다"를 None 으로 정직하게 말한다. 쓰는 쪽이
    intrinsics_from_hfov() 로 지어내든 --intrinsics 로 받든 결정해야 한다.

    타임스탬프는 **영상 자체의 재생시각[초]** 이다(CAP_PROP_POS_MSEC).
    벽시계(time.time())가 아니다 — 같은 파일을 몇 번을 돌려도 같은 숫자가 나와야
    프레임 단위로 비교가 된다. POS_MSEC 를 못 주는 코덱이면 index/fps 로 만든다.

    Args:
        loop: True 면 끝에서 되감아 계속 돈다. **그러면 안 끝난다** — 호출자가 멈춰야 한다.

    Returns:
        (_FrameStream, None). frames.close() 로 캡처를 놓아준다.
        `with` 문도 된다 — 카메라만큼 급하진 않지만 파일 핸들도 열린 자원이다.
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError("영상이 없다: %s" % p)
    cap = cv2.VideoCapture(str(p))
    if not cap.isOpened():
        raise RuntimeError("영상을 열 수 없다(코덱?): %s" % p)

    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    _done = []

    def _cleanup():
        if _done:
            return
        _done.append(True)
        try:
            cap.release()
        except Exception:
            pass

    def frames():
        i = 0
        try:
            while True:
                ms = cap.get(cv2.CAP_PROP_POS_MSEC)     # read() 전이 이 프레임의 시각
                ok, bgr = cap.read()
                if not ok:
                    if not loop:
                        break
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ms = 0.0
                    ok, bgr = cap.read()
                    if not ok:
                        break                            # 되감아도 안 나오면 진짜 끝
                ts = (ms / 1000.0) if ms and ms > 0 else (i / fps if fps > 0 else float(i))
                yield i, ts, bgr
                i += 1
        finally:
            _cleanup()

    return _FrameStream(frames(), cleanup=_cleanup), None


def depth_at(depth, u, v, patch=5, scale=None):
    """(u, v) 픽셀 주변의 실측 거리 [m]. 못 재면 None.

    왜 한 픽셀이 아니라 패치의 중앙값인가:
        D435i 의 depth 는 스테레오 매칭 결과라 구멍이 숭숭 뚫려 있고,
        구멍은 0 으로 그대로 나온다. 하필 태그 중심은 검은 칸이 많아
        한 픽셀만 읽으면 0 이 나오는 일이 흔하다. 그래서 patch x patch 를
        긁어 **0 을 버리고** 남은 값의 중앙값을 쓴다. 평균이 아니라 중앙값인 이유는
        태그 가장자리를 물면 배경 거리가 섞여 들어오는데, 중앙값이 그걸 덜 탄다.

    Args:
        depth: 다음 셋 중 아무거나
            - Frame (open_realsense/open_bag 의 with_depth=True 가 준 이미지).
              내부의 .depth 와 .depth_scale 을 알아서 쓴다.
            - [H x W] uint16 원시 깊이맵 (scale 을 같이 줘야 정확하다)
            - [H x W] float 깊이맵 (이미 미터로 본다)
        u, v: 픽셀 좌표 (x, y). float 여도 되고 반올림해서 쓴다.
        patch: 긁을 정사각형 한 변. 홀수를 권한다.
        scale: 깊이 한 칸의 미터값. None 이면 Frame 은 자기 값을,
            uint 배열은 0.001(mm)을, float 배열은 1.0 을 쓴다.
            **0.001 하드코딩에 기대지 말 것** — 장치가 주는 값을 쓰는 게 맞다.

    Returns:
        float 거리 [m], 또는 None (depth 가 없거나 / 화면 밖 / 전부 0일 때)

    자세와 대조할 때:
        이 값은 **광축 방향 Z** 다. 그러므로 pose_to_xyzrpy(T)["z"] 와 직접 비교하면 된다.
        docking_state(T)["forward"] 와 비교하면 안 된다 — 그건 태그 평면 기준
        수직거리라 다른 양이다. 포인트클라우드로 펼친 좌표(축 순서가
        (전방,좌,상) 이다)와도 다르다.
    """
    dm = getattr(depth, "depth", None)
    if dm is not None:                       # Frame 이 들어온 경우
        if scale is None:
            scale = getattr(depth, "depth_scale", 0.0) or 0.0
    else:
        dm = depth
    if dm is None:
        return None
    dm = np.asanyarray(dm)
    if dm.ndim != 2 or dm.size == 0:
        return None
    if scale is None:
        scale = 1.0 if np.issubdtype(dm.dtype, np.floating) else 0.001
    if not scale:
        return None                          # depth_scale=0 == depth 없음

    h, w = dm.shape
    u, v = int(round(float(u))), int(round(float(v)))
    r = max(0, int(patch) // 2)
    x0, x1 = max(0, u - r), min(w, u + r + 1)
    y0, y1 = max(0, v - r), min(h, v + r + 1)
    if x0 >= x1 or y0 >= y1:
        return None                          # 화면 밖

    win = dm[y0:y1, x0:x1]
    nz = win[win > 0]                        # 0 = 측정 실패. 평균에 섞으면 거리가 당겨진다
    if nz.size == 0:
        return None
    return float(np.median(nz.astype(np.float64)) * scale)


#: to_gray(channel=...) 에서 쓰는 BGR 채널 번호.
_BGR_CHANNEL = {"blue": 0, "green": 1, "red": 2}


def to_gray(img, channel=None):
    """검출기에 넣을 흑백 이미지. 컬러면 변환하고 이미 흑백이면 그대로.

    블로그 설명대로, 검출기는 밝기 채널만 쓰므로 흑백이 빠르고 패턴도 또렷하다.

    **open_realsense(color_format="yuyv") 로 받은 Frame 이면 BGR2GRAY 를 건너뛰고
    센서 원본 휘도(img.luma)를 그대로 돌려준다.** BGR8 왕복은 256 단계 중
    38 단계를 뭉개기 때문이다(rs_tuning.LUMA_CLIPPED_LEVELS 주석 참고).
    다른 경로(파일/rosbag/bgr8)에서는 예전과 완전히 같은 동작이다.

    ── 전처리를 더 얹지 않은 이유 (실측하고 버렸다) ──────────────────────────
    "흑백 변환을 바꾸거나 CLAHE 를 넣으면 검출이 좋아지지 않을까"는 그럴듯하지만,
    28프레임 클립(오버레이 제거한 것)으로 9가지를 다 돌려본 결과는 **전부 무의미**했다.
        BGR2GRAY / B / G / R / max(BGR) / min(BGR) / CLAHE / R+CLAHE / LAB-L
        -> 아홉 개 모두 28/28 (100%). 하나도 다르지 않았다.
    깨끗한 영상에서는 검출률에 손댈 여지가 애초에 없다는 뜻이다.

    **CLAHE 는 오히려 해롭다.** 같은 프레임에서 BGR2GRAY 대비 모서리가
        평균 0.392px, p95 1.041px, 최대 2.572px 밀렸다
    (단일채널은 0.02~0.04px 로 무시할 수준이다 — CLAHE 만 10~19배다).
    대가로 얻는 건 decision_margin 73.2 -> 77.8 뿐인데, 마진은 이미 하한
    MIN_DECISION_MARGIN=20 의 3.7배라 남아돈다. 238px 태그(z=1m)에서 0.4px 는
    거리 오차 약 1.7mm 다. **없는 여유를 얻자고 있는 정확도를 파는 거래**라
    넣지 않았다. 나중에 누가 다시 넣으려거든 이 숫자를 먼저 반박할 것.

    Args:
        channel: None(기본)이면 위와 같이 표준 동작. "red"/"green"/"blue" 를 주면
            **그 채널 하나만** 흑백으로 쓴다.

            이건 화질 개선이 아니라 **한 가지 색으로 오염된 영상에서 오염을
            통째로 피하는** 수단이다. 실측 — 초록 디버그 오버레이가 태그 데이터칸을
            가로지르는 원본 영상에서:
                BGR2GRAY        14/28  50.0%   0.12ms
                channel="red"   25/28  89.3%   0.79ms  <- 오버레이가 안 보이는 채널
                channel="blue"  25/28  89.3%   0.79ms
                channel="green" 14/28  50.0%          <- 오염된 채널을 고르면 당연히 그대로
                                28/28 100.0%  34.7ms  <- 완벽하지만 30fps 예산(33ms)을 넘긴다
            즉 **공짜에 가까운 값으로 39%p 를 되찾는다.** 다만 100% 는 아니다.

            이 클립 말고도 쓸모가 있다: 바닥 도색선, 색 경광등처럼 단색으로
            오염되는 상황이면 같은 수가 통한다. 반대로 **IR 프로젝터 점 패턴은
            이걸로 못 피한다** — 그건 open_realsense(emitter=False) 로 끈다.

            깨끗한 영상에서는 켜도 손해가 거의 없지만(모서리 0.03px) 이득도 없다.
            기본값이 None 인 이유다.
    """
    if channel is not None:
        c = _BGR_CHANNEL.get(channel)
        if c is None:
            raise ValueError("channel 은 red/green/blue: %r" % (channel,))
        if img.ndim != 3:
            return np.asarray(img)              # 이미 흑백이면 고를 채널이 없다
        # 뷰가 아니라 복사여야 한다 — 검출기는 C-contiguous 버퍼를 요구한다.
        return np.ascontiguousarray(np.asarray(img)[:, :, c])
    luma = getattr(img, "luma", None)
    if luma is not None and luma.shape[:2] == img.shape[:2]:
        return luma
    if img.ndim == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


# ===========================================================================
# 3) 태그 찾기 — make_detector / detect / tag_pixel_size
# ===========================================================================
#
# 블로그의 이 부분에 해당한다.
#
#     detector = apriltag.Detector()
#     results, dimg = detector.detect(img_gray, return_image=True)
#
# 검출 결과 하나에는 다음이 들어 있다.
#     tag_id           태그 번호. 탑재부 상단/내부를 이걸로 구분한다
#     homography       3x3. 정사각형 태그가 화면에서 얼마나 찌그러졌는가
#     corners          네 모서리 픽셀좌표 (반시계 방향)
#     center           중심 픽셀좌표
#     decision_margin  디코딩 품질. 낮으면 잘못 읽었을 수 있다
#     hamming          고친 오류 비트 수. 0 이 아니면 의심해볼 만하다
#
# 자세는 여기서 나오지 않는다. homography 를 4단계로 넘겨야 나온다.


# quad_blur — 검출 전 가우시안 블러. 영상 종류에 따라 효과가 정반대다.
#
#   노이즈 있는 실촬영(tagslam) : 0 -> 56개,  2.0 -> 84개,  4.0 -> 112개   크게 좋아짐
#   깨끗한 합성영상             : 0 -> 17개,  2.0 -> 14개                  오히려 나빠짐
#
# 자세 정확도는 살짝 손해다(같은 평판 태그들의 일치 편차 0.70도 -> 0.84도).
# 영상마다 반대로 작용하므로 기본은 끄고, 실촬영에서 검출이 모자라면 켜는 쪽으로 둔다.
DEFAULT_QUAD_BLUR = 0.0


def make_detector(families="tag36h11", quad_blur=DEFAULT_QUAD_BLUR, **options):
    """검출기를 만든다.

    Args:
        families: 태그 종류. AprilTag 기본이자 블로그가 쓴 것은 tag36h11.
                  tag16h5 는 코드 배치가 성겨 오검출이 많으니 피할 것.
        quad_blur: 검출 전 가우시안 블러 세기. 0 이면 끈다. 위 주석 참고.
        **options: apriltag.DetectorOptions 에 그대로 넘어간다.
            quad_decimate  1.0 이 원본 해상도. 올리면 빨라지지만 작은 태그를 놓친다
            refine_edges   모서리를 한 번 더 다듬는다. 자세 정확도가 올라간다
    """
    if quad_blur:
        options.setdefault("quad_blur", quad_blur)
    opts = apriltag.DetectorOptions(families=families, **options)
    return apriltag.Detector(opts)


def detect(detector, img_gray, min_margin=0.0, max_hamming=0):
    """태그를 찾는다.

    Args:
        img_gray: 흑백 이미지. 2단계의 to_gray() 를 거친 것.
        min_margin: decision_margin 이 이보다 낮으면 버린다. 오검출 방지용.
        max_hamming: 고친 오류 비트가 이보다 많으면 버린다.
                     블로그 표에 적힌 대로, 오류를 많이 고칠수록 오검출이 급증한다.

    Returns:
        검출 결과 리스트. tag_id 오름차순.
    """
    results = detector.detect(img_gray)
    keep = [r for r in results
            if r.decision_margin >= min_margin and r.hamming <= max_hamming]
    return sorted(keep, key=lambda r: r.tag_id)


# 예전에 있던 summarize() / center_offset() 은 지웠다 — 둘 다 부르는 곳이 없었다.
#   summarize()      검출 결과를 사람이 읽을 문자열로 찍던 것. 지금은 tools/live_pose.py 의
#                    패널과 tools/verify.py 의 표가 각자 제 형식으로 찍는다.
#   center_offset()  태그 중심을 화면중앙 기준 오프셋[px]으로 주던 것. Testing_apriltag.mp4
#                    좌측 상단 HUD("Center X/Y coord")와 규약을 맞추려고 만든, 그 영상
#                    전용 비교자였다. 도킹 값은 화면 오프셋이 아니라 docking_state() 의
#                    lateral/forward 로 판단하므로 파이프라인에는 쓸 자리가 없다.
#                    **거기서 건진 사실은 버리지 않았다**: 모서리평균과 검출기 center 는
#                    태그가 기울면 갈라지고(frame11 기준 화면중앙 오프셋 1.8px vs 9.7px),
#                    그래서 depth 를 뜰 때는 무게중심 쪽을 쓴다 — _sample_depth_m 참고.
# 필요해지면 git 이력에서 되살릴 것.

# 태그가 화면에서 몇 픽셀인가 — 자세를 믿을 수 있는지의 1차 관문.
#
#   MIN_TAG_PX    이보다 작으면 검출 자체가 간헐적이다. tag36h11 은 한 변에
#                 8칸(테두리 포함 10칸)이 들어가야 해서, 한 칸이 2px 아래로
#                 떨어지면 디코딩이 무너진다.
#   STABLE_TAG_PX 검출은 되지만 20~50px 구간은 모서리 한 픽셀의 흔들림이
#                 그대로 각도/거리로 증폭된다. 거리 오차는 대략 1/tag_px 에
#                 비례하므로, 픽셀이 절반이 되면 오차는 두 배가 된다.
#
# 감이 오도록 실제 장비 기준으로 환산해 둔다 (D435i 컬러 1920x1080, fx=1359):
#   tag_px ≈ fx * tag_size / z 이므로 0.20m 태그는
#       z=2m  -> 136px   (충분)
#       z=5m  -> 54px    (겨우 안정권)
#       z=13m -> 21px    (검출 한계. 이 거리는 태그를 키우는 수밖에 없다)
MIN_TAG_PX = 20.0
STABLE_TAG_PX = 50.0


def tag_pixel_size(detection):
    """태그 한 변의 화면상 길이 [px].

    네 모서리를 이은 사변형의 **네 변 길이 평균**이다. 변 하나만 재면
    태그가 기울었을 때 어느 변을 골랐느냐에 따라 값이 크게 갈리지만,
    평균은 원근으로 짧아진 변과 길어진 변이 서로 상쇄돼 훨씬 덜 흔들린다.

    예전에 도구들이 쓰던 cv2.contourArea 의 sqrt 와는 정면일 때 1px 안쪽으로
    일치하지만(실측 241.6 vs 242.6), 크게 기울면 갈라진다. 면적은 원근으로
    납작해진 만큼 그대로 줄어들어(실측 74도에서 121px) "태그가 멀어졌다"와
    "태그가 돌아갔다"를 구분하지 못한다. 변 길이 평균은 그 상황에서도
    190px 를 유지한다. **거리 판단에는 이쪽을 쓴다.**

    판정 기준은 MIN_TAG_PX(20px, 검출 하한) / STABLE_TAG_PX(50px, 안정권)이다.
    위 상수 주석에 거리 환산표가 있다.

    Returns:
        float [px]
    """
    import numpy as _np
    c = _np.asarray(detection.corners, dtype=_np.float64)
    edges = _np.linalg.norm(c - _np.roll(c, -1, axis=0), axis=1)
    return float(edges.mean())


# ===========================================================================
# 4) 자세 구하기 — estimate_pose / docking_state / tag_tilt_deg
# ===========================================================================
#
# 블로그의 이 부분에 해당한다.
#
#     pose, e0, e1 = detector.detection_pose(
#         detection=r, camera_params=cam_params_rgb, tag_size=tag_size)
#
# 원리: 정사각형 태그가 화면에서 얼마나 찌그러졌는지(homography)와
#       카메라 값(fx,fy,cx,cy), 그리고 태그의 실제 크기를 알면
#       거리와 각도가 역산된다. depth 센서는 필요 없다.
#       라이브러리 내부에서 pose_from_homography() 가 이 일을 한다.
#
# 주는 값은 T_camera_tag — "카메라 기준으로 본 태그의 자세"다.
# 반대로 "태그 기준으로 본 카메라" 가 필요하면 invert_T 로 뒤집는다
# (docking_state 가 안에서 그걸 한다. 예전에 있던 camera_pose_in_tag() 는
#  invert_T 한 줄짜리 껍데기라 지웠다).
#
# ── 아래 두 함수는 손대지 말 것 ────────────────────────────────────────────
# docking_state() 와 tag_tilt_deg() 의 **본문은 얼어 있다.**
# src/etc/viewer_filter/apriltag-detection.cpp 가 같은 계산을 C++ 로 그대로
# 옮겨 놓았고(뷰어 오버레이가 같은 숫자를 띄워야 한다), 한쪽만 고치면 두 화면이
# 조용히 다른 값을 보여준다. 상수를 예쁘게 빼는 것조차 하지 않았다 — 245번째 줄의
# 리터럴 10.0 은 RELIABLE_TILT_DEG 와 같은 값이지만 **일부러** 그대로 둔 것이다.


# 태그 네 모서리의 3D 좌표. detection.corners 와 같은 순서다.
# 실측 확인: 정면 태그에서 corners[0] 이 화면 오른쪽아래에 오고 (-s,-s) 에 대응한다.
def _object_points(tag_size):
    s = tag_size / 2.0
    return np.array([[-s, -s, 0.], [s, -s, 0.], [s, s, 0.], [-s, s, 0.]], dtype=np.float64)


def pose_by_pnp(detection, intrinsics, tag_size):
    """OpenCV solvePnP 로 자세를 구한다. detection_pose 의 대안.

    라이브러리의 detection_pose() 는 호모그래피를 분해하는 방식이라,
    태그가 **정확히 정면(기울기 0.00도)이고 화면축과 나란할 때** 계산이 퇴화해
    NaN 을 낸다(라이브러리 내부 homography_to_pose 의 "had ta normalize!" 경고).
    도킹은 정렬각 0 이 목표 상태라 바로 그 지점에서 값이 필요하므로 이 경로를 둔다.

    실측: 기울기 0도에서 오차 0.00도, 다른 각도에서는 detection_pose 와 소수점까지 일치.

    Returns:
        (T_camera_tag 4x4, 재투영오차[px])
    """
    obj = _object_points(tag_size)
    img = np.asarray(detection.corners, dtype=np.float64).reshape(-1, 1, 2)
    dist = np.array(intrinsics.distortion, dtype=np.float64) if intrinsics.distortion else np.zeros(5)

    ok, rvec, tvec = cv2.solvePnP(obj, img, intrinsics.K, dist,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return np.full((4, 4), np.nan), float("inf")

    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.ravel()

    proj, _ = cv2.projectPoints(obj, rvec, tvec, intrinsics.K, dist)
    err = float(np.sqrt(((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2).sum(axis=1)).mean())
    return T, err


def estimate_pose(detector, detection, intrinsics, tag_size, method="auto"):
    """태그 하나의 자세를 구한다.

    Args:
        intrinsics: CameraIntrinsics
        tag_size: 태그 실제 한 변 길이 [m]. 검은 테두리까지 포함한 값이다.
                  이 값이 틀리면 거리가 그 비율만큼 통째로 틀어진다.
        method:
            "auto"  detection_pose 를 쓰되 NaN 이 나오면 solvePnP 로 넘어간다 (기본)
            "tag"   detection_pose 만 쓴다 (블로그 원본 그대로)
            "pnp"   solvePnP 만 쓴다

    Returns:
        (T_camera_tag 4x4, init_error, final_error)
        두 오차는 다듬기 전/후 재투영 오차다. 둘이 비슷하게 크면 자세를 믿기 어렵다.
    """
    if method == "pnp":
        T, err = pose_by_pnp(detection, intrinsics, tag_size)
        return T, err, err

    T, e0, e1 = detector.detection_pose(
        detection=detection,
        camera_params=intrinsics.params,
        tag_size=tag_size,
    )
    T = np.asarray(T)

    if method == "auto" and not np.isfinite(T).all():
        T, err = pose_by_pnp(detection, intrinsics, tag_size)
        return T, err, err
    return T, e0, e1


def pose_to_xyzrpy(T, unit='deg'):
    """4x4 -> 읽을 수 있는 형태.

    Returns:
        dict(x, y, z, roll, pitch, yaw, distance)
    """
    p, R = t2pr(T)
    rpy = r2rpy(R, unit=unit)
    return {"x": p[0], "y": p[1], "z": p[2],
            "roll": rpy[0], "pitch": rpy[1], "yaw": rpy[2],
            "distance": float(np.linalg.norm(p))}


def docking_state(T_camera_tag):
    """도킹 제어가 바로 쓸 수 있는 형태로 바꾼다.

    detection_pose() 는 "카메라 기준 태그"를 준다. 도킹에서 알고 싶은 건 반대,
    **태그(탑재부) 기준으로 지게차가 어디에 어떻게 서 있나**다.

    그런데 이게 하나로 안 된다. 두 가지가 서로 다른 것을 말한다.

        approach_deg : 태그 정면축 기준으로 내가 **어느 방향에** 있나  (위치)
        heading_deg  : 내가 태그 축과 **나란히 서 있나**              (자세)

    둘은 독립이다. 정면축 위에 있어도(approach=0) 고개를 돌리고 있으면 heading≠0 이고,
    반대로 옆에 비켜서 태그를 똑바로 바라보면 heading=0 이지만 approach≠0 이다.
    **진입하려면 둘 다 0 이어야 한다.**

    부호 규약 주의: AprilTag 의 태그 좌표계는 **+z 가 태그 뒤쪽**을 향한다.
    그대로 쓰면 카메라가 항상 z 음수에 놓여 "앞으로 몇 m" 가 음수로 나온다.
    여기서는 사람이 읽기 편하게 뒤집어서, forward 가 **양수면 태그 앞쪽**이 되게 한다.

    Returns:
        dict
          lateral    [m]  정면축에서 좌우 벗어남
          vertical   [m]  위아래 벗어남
          forward    [m]  태그면까지 수직 거리 (양수 = 태그 앞쪽)
          distance   [m]  직선 거리
          approach_deg [도] 정면축에서 벗어난 방향각
          heading_deg  [도] 진입 방향이 축과 이루는 각 (0 이면 축과 나란함)
          reliable_angle [bool] approach_deg 를 믿어도 되는지.
              태그가 정면에 가까우면(약 10도 미만) 원근 왜곡이 픽셀 이하라
              각도를 못 잰다. 그때는 lateral 로 판단해야 한다.
    """
    T_tag_cam = invert_T(np.asarray(T_camera_tag))
    p = T_tag_cam[:3, 3]
    R = T_tag_cam[:3, :3]

    lateral = float(p[0])
    vertical = float(p[1])
    forward = float(-p[2])                      # 태그 앞쪽이 양수가 되게 뒤집는다
    distance = float(np.linalg.norm(p))

    # 내가 태그 정면축에서 몇 도 벗어난 위치에 있나
    approach = float(np.degrees(np.arctan2(abs(lateral), abs(forward))))

    # 카메라 광축(+z)이 태그 좌표계에서 어디를 향하나.
    # 태그를 정면으로 마주보고 축과 나란하면 태그의 +z 방향(뒤쪽)을 향한다.
    fwd = R @ np.array([0.0, 0.0, 1.0])
    heading = float(np.degrees(np.arctan2(fwd[0], fwd[2])))

    return {"lateral": lateral, "vertical": vertical, "forward": forward,
            "distance": distance,
            "approach_deg": approach, "heading_deg": heading,
            "reliable_angle": approach >= 10.0}


def tag_tilt_deg(T_camera_tag):
    """태그면이 카메라를 정면으로 마주보는 정도 [도].

    0 이면 태그가 화면과 완전히 나란하다(정면). 이 값이 약 10도 미만이면
    원근 왜곡이 픽셀 이하라 **각도 추정을 믿을 수 없다.**
    좌표계 규약과 무관해서 roll/pitch/yaw 보다 판정 기준으로 쓰기 좋다.
    """
    n = np.asarray(T_camera_tag)[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(min(1.0, abs(n[2])))))


# ===========================================================================
# 5) 품질 판정 — pose_quality / depth_cross_check / 임계값
# ===========================================================================


# ---------------------------------------------------------------------------
# 자세 품질 판정 — "이 프레임의 자세를 믿어도 되는가"
# ---------------------------------------------------------------------------
#
# 세 가지를 따로 본다. 서로 다른 실패를 잡기 때문이다.
#   tag_px       태그가 너무 작다        -> 모든 값이 같이 흔들린다
#   재투영오차   모델과 픽셀이 안 맞는다 -> 오검출/잘못된 intrinsics/모서리 오염
#   depth 교차검증 기하 거리와 실측 거리가 갈린다 -> tag_size 나 fx 가 틀렸다
#
# 실측 근거는 /home/jeongmin/work/projects/krri/data/apriltag/Testing_apriltag_trim.mp4
# 28프레임(검출 100%)을 돌린 값이다. 아래 상수 주석에 그대로 적어 둔다.

# 재투영 RMS 상한 [px].
# 위 클립 실측: 기울기 46도 이하 구간에서 RMS 0.75~1.91px 에 모두 들어갔고,
# 50도를 넘어가야 2.4~3.9px 로 올라갔다. 즉 2px 은 "정상 검출의 위쪽 끝"이다.
# 주의: 여기 임계값은 **RMS[px]** 기준이다. detection_pose 가 돌려주는 e1 은
# px 이 아니라 px^2(모서리별 제곱거리의 평균)이라 반드시 sqrt 를 취해서 비교한다.
MAX_REPROJ_RMS_PX = 2.0

# decision_margin 하한.
# 위 클립 실측값은 71.9~73.6 으로 아주 높다. 실패 사례가 없어서 데이터로
# 임계값을 뽑을 수 없었다. 그래서 tag36h11 에서 통상 쓰는 보수적 하한 20 을
# 쓰되, "측정된 정상값의 1/3 수준" 이라는 여유만 확인해 둔 값이다.
# 근거가 약한 숫자이므로 실촬영 데이터가 쌓이면 다시 잡을 것.
MIN_DECISION_MARGIN = 20.0

# 각도를 믿을 수 있는 최소 기울기 [도]. tag_tilt_deg 의 docstring 과 같은 값이고
# src/etc/viewer_filter/apriltag-detection.cpp 의 10.0 과도 같은 숫자다.
# 태그가 정면에 가까우면 원근 왜곡이 픽셀 이하라 각도가 아예 안 잡힌다.
RELIABLE_TILT_DEG = 10.0

# depth 교차검증 허용 오차 [m].
# D435i 스테레오 깊이 오차는 거리의 **제곱**으로 커진다(시차가 1/z 이므로).
# 실사용 기준 0.6m 에서 약 2%(=0.012m) 이고, 이를 sigma(z)=0.033*z^2 로 두면
#   0.6m -> 0.012m(2%)   1m -> 0.033m(3.3%)   2m -> 0.13m(6.7%)   3m -> 0.30m(10%)
# 3m 쯤 가면 depth 쪽이 자세 추정보다 더 부정확해져서, 두 값이 갈려도
# **어느 쪽이 틀렸는지 말할 수 없다.** 그래서 이 검사는 근거리 전용이다.
# 허용치는 1.5*sigma 에 2cm 바닥(자세 자체 오차 + 중심 픽셀 표본 오차)을 더한 값.
DEPTH_TOL_COEF = 0.05           # tol = coef * z^2
DEPTH_TOL_FLOOR_M = 0.02
DEPTH_CHECK_MAX_Z = 1.5         # 이 거리를 넘으면 허용치가 너무 헐거워 무의미


def _sample_depth_m(detection, depth, patch, depth_scale):
    """태그 중심 주변 patch x patch 의 depth 중앙값 [m]. 못 재면 None.

    중심은 detection.center 가 아니라 **모서리 평균**을 쓴다. 기울어진 태그에서
    둘이 갈라진다 — 실측(frame11)에서 화면중앙 기준 오프셋이 모서리평균 쪽은
    1.8px, 검출기가 준 center 쪽은 9.7px 로 갈렸다. depth 를 뜰 때는 무게중심
    쪽이 태그면 위에 확실히 얹힌다.

    실제 표본추출(구멍=0 을 버리고 중앙값)은 2단계의 depth_at 하나에만 있다.
    예전에는 같은 규칙을 여기에 한 번 더 구현해 두고 `try: from .image_source
    import depth_at / except ImportError` 로 갈랐었다. 파일이 갈라져 있던 시절,
    "그 함수가 없는 버전과 섞여 돌아가더라도 죽지 않도록" 이라는 이유였다.
    한 파일로 합친 지금 그 ImportError 는 **절대 안 난다** — 그러면 남는 건
    같은 규칙의 두 번째 구현뿐이고, 그건 조용히 갈라지기만 한다. 그래서 지웠다.
    """
    if depth is None:
        return None
    u, v = np.asarray(detection.corners, dtype=np.float64).mean(axis=0)
    return depth_at(depth, u, v, patch=patch, scale=depth_scale)


def depth_cross_check(detection, depth, T_camera_tag, patch=5, depth_scale=None):
    """태그로 잰 거리와 depth 센서로 잰 거리를 맞춰 본다.

    이 둘은 **원리가 완전히 다르다**. 태그 쪽은 "정사각형이 화면에서 얼마나
    찌그러졌나"만 보는 순수 기하이고(tag_size 와 fx 를 믿는다), depth 쪽은
    좌우 IR 두 장의 시차를 재는 물리 측정이다. 공통 입력이 없으므로 둘이
    어긋나면 **둘 중 하나는 확실히 틀린 것**이다. 대개는

        tag_size 가 실측과 다르다   -> 거리 전체가 그 비율만큼 밀린다
        intrinsics(fx) 가 틀렸다    -> 역시 거리가 통째로 밀린다
        엉뚱한 것을 태그로 읽었다   -> 거리가 아예 논다

    비교 대상은 pose_to_xyzrpy(T)["z"], 즉 **광축 방향 Z** 다.
    docking_state(T)["forward"] 와 헷갈리지 말 것 — 그건 태그 좌표계에서
    태그면까지의 수직거리라 아예 다른 양이다. 포인트클라우드로 펼친 좌표도
    축 순서가 (forward,left,up) 이라 여기 쓸 수 없다.

    정직하게 적어 둔다: **이 검사는 근거리 전용이다.** 스테레오 깊이 오차는
    거리의 제곱으로 커져서 0.6m 에서 약 2% 이지만 3m 면 10% 수준이 되고,
    그 지점에서는 자세 추정 쪽이 오히려 더 정확하다. 그래서 허용치도 z^2 로
    벌어지게 두었고, DEPTH_CHECK_MAX_Z(1.5m) 를 넘으면 in_range=False 로
    표시해 "통과했다"에 의미를 두지 말라고 알린다.

    Args:
        detection: AT2 검출 결과
        depth: HxW depth 맵. 정수면 장치 단위, 실수면 이미 m 로 본다.
               open_realsense/open_bag 이 depth 를 얹어 준 Frame 을 그대로 넘겨도 된다.
               None 이면 z_depth=None 으로 돌아온다.
        T_camera_tag: estimate_pose 가 준 4x4
        patch: 중심 주변 몇 픽셀을 볼지. 홀수 권장. depth 구멍 때문에 1 은 위험하다.
        depth_scale: depth 한 칸이 몇 m 인가. RealSense 는 실측 0.001(1mm).
                     None 이면 dtype 으로 추정한다.

    Returns:
        dict
          z_pose   [m]    태그 기하로 구한 광축거리
          z_depth  [m]    depth 실측. 못 재면 None (구멍이거나 depth 자체가 없음)
          diff_m   [m]    z_depth - z_pose. z_depth 가 None 이면 None
          diff_pct [%]    z_pose 대비 몇 %
          tol_m    [m]    이번 거리에서의 허용치
          in_range [bool] 검사가 의미 있는 거리인가 (z_pose <= 1.5m)
          agree    [bool] |diff| <= tol. z_depth 가 없으면 False
    """
    z_pose = float(np.asarray(T_camera_tag)[2, 3])
    tol = max(DEPTH_TOL_FLOOR_M, DEPTH_TOL_COEF * z_pose * z_pose)
    out = {"z_pose": z_pose, "z_depth": None, "diff_m": None, "diff_pct": None,
           "tol_m": float(tol), "in_range": bool(0.0 < z_pose <= DEPTH_CHECK_MAX_Z),
           "agree": False}

    z_depth = _sample_depth_m(detection, depth, patch, depth_scale)
    if z_depth is None or not np.isfinite(z_pose) or z_pose <= 0:
        return out

    diff = z_depth - z_pose
    out["z_depth"] = float(z_depth)
    out["diff_m"] = float(diff)
    out["diff_pct"] = float(100.0 * diff / z_pose)
    out["agree"] = bool(abs(diff) <= tol)
    return out


def pose_quality(detector, detection, intrinsics, tag_size, T_camera_tag, method="auto"):
    """한 프레임의 자세를 믿어도 되는지 한 번에 판정한다.

    호출부가 매번 재투영오차/픽셀크기/기울기를 따로 계산하고 임계값을 각자
    적어 넣는 일을 없애려는 함수다. 임계값은 전부 이 모듈 상단 상수에 있고
    출처를 주석으로 적어 두었다.

    **reproj_err 는 e1(다듬은 뒤 오차)이다.** estimate_pose 가 주는 (T,e0,e1)
    중 뒤엣것. e0 는 다듬기 전이라 원래 크다(실측 4.9 -> 1002 까지 편차가 크다).
    단위 함정이 하나 있다:
        detection_pose 경로 -> e1 은 **px^2** (모서리별 제곱거리의 평균)
        solvePnP 경로       -> err 은 **px**  (모서리별 유클리드거리의 평균)
    그래서 판정은 reproj_err 이 아니라 항상 reproj_rms_px(=단위를 px 로 맞춘 값)
    으로 한다. 어느 경로였는지는 reproj_units 로 알려 준다.

    reliable_angle 주의: 여기서는 **tag_tilt_deg >= 10도**다.
    docking_state() 에도 같은 이름의 키가 있지만 그건 approach_deg >= 10 으로
    "내가 태그 정면축에서 얼마나 비켜 서 있나"를 본다. 둘은 다른 값이고
    실제로 갈린다(실측 frame4: tilt 18.7도라 여기선 True, approach 7.5도라
    docking_state 에서는 False). docking_state 쪽은 C++ 필터가 그대로 따라
    쓰고 있어 건드리지 않았다.

    ok 에 기울기를 넣지 않은 이유: 정면에 가까운 태그(tilt<10)는 각도를 못 믿을
    뿐 **거리는 멀쩡하다**. 반대로 많이 기운 태그도 진입 중이면 정상 상황이다.
    기울기는 "각도를 믿지 마라"는 신호일 뿐 자세 전체를 버릴 이유가 아니라서
    reliable_angle 로 따로 내보낸다.

    Returns:
        dict
          reproj_err     estimate_pose 의 e1 그대로 (단위는 reproj_units 참고)
          reproj_units   "px^2" 또는 "px"
          reproj_rms_px  px 로 통일한 재투영 RMS. 판정은 이걸로 한다
          tag_px         태그 한 변의 화면 길이 [px]
          tilt_deg       태그면 기울기 [도]
          reliable_angle [bool] tilt_deg >= RELIABLE_TILT_DEG(10도)
          decision_margin, hamming  검출 자체의 신뢰도
          ok             [bool] 아래 조건을 모두 만족
          reasons        [list[str]] ok 가 False 인 이유들. 비어 있으면 통과

        ok 조건 (임계값 출처는 각 상수 주석):
          T 가 유한하다                              (NaN 이면 자세 계산 실패)
          tag_px >= STABLE_TAG_PX (50px)             안정권
          reproj_rms_px <= MAX_REPROJ_RMS_PX (2px)   정상 검출의 위쪽 끝
          decision_margin >= MIN_DECISION_MARGIN (20)
          hamming == 0                               오류비트를 고쳤으면 의심
    """
    T = np.asarray(T_camera_tag, dtype=np.float64)

    # estimate_pose 와 같은 분기를 여기서 다시 탄다. 어느 경로로 나온
    # 오차인지(px^2 인지 px 인지) 알아야 임계값 비교가 되는데, estimate_pose
    # 의 반환값만 봐서는 구분이 안 되기 때문이다.
    if method == "pnp":
        _, err = pose_by_pnp(detection, intrinsics, tag_size)
        e1, squared = err, False
    else:
        Th, _e0, e1 = detector.detection_pose(
            detection=detection, camera_params=intrinsics.params, tag_size=tag_size)
        squared = True
        if method == "auto" and not np.isfinite(np.asarray(Th)).all():
            _, err = pose_by_pnp(detection, intrinsics, tag_size)
            e1, squared = err, False

    e1 = float(e1)
    rms = float(np.sqrt(e1)) if (squared and e1 >= 0) else e1

    tag_px = tag_pixel_size(detection)
    tilt = tag_tilt_deg(T)
    margin = float(getattr(detection, "decision_margin", 0.0))
    hamming = int(getattr(detection, "hamming", 0))

    reasons = []
    if not np.isfinite(T).all():
        reasons.append("pose_nan")
    if tag_px < STABLE_TAG_PX:
        reasons.append(f"tag_px<{STABLE_TAG_PX:g}")
    if not np.isfinite(rms) or rms > MAX_REPROJ_RMS_PX:
        reasons.append(f"reproj>{MAX_REPROJ_RMS_PX:g}px")
    if margin < MIN_DECISION_MARGIN:
        reasons.append(f"margin<{MIN_DECISION_MARGIN:g}")
    if hamming != 0:
        reasons.append("hamming!=0")

    return {"reproj_err": e1,
            "reproj_units": "px^2" if squared else "px",
            "reproj_rms_px": rms,
            "tag_px": tag_px,
            "tilt_deg": tilt,
            "reliable_angle": bool(tilt >= RELIABLE_TILT_DEG),
            "decision_margin": margin,
            "hamming": hamming,
            "ok": not reasons,
            "reasons": reasons}


# ===========================================================================
# 6) 파이프라인 — TagPipeline / Result
# ===========================================================================
#
# 1~5 를 매번 같은 순서로 엮는 코드가 도구마다 복사돼 있었다(열고 -> to_gray ->
# detect -> estimate_pose -> docking_state -> 실패 삼키기). 그 조립을 한 군데로
# 모은 **편의층**이다. 낱개 함수들은 그대로 살아 있고, 앞으로도 그래야 한다.
#
# 여기서 삼키는 실패: 검출기가 넘어지는 것, 자세가 NaN 으로 나오는 것.
# 삼키되 **말은 한다** — Result.errors 에 이유가 남는다. 조용히 빈 결과를
# 돌려주면 "태그가 없었다"와 "우리가 터졌다"를 구분할 수 없다.


@dataclass
class Result:
    """한 프레임에서 나온 것 전부. TagPipeline 이 프레임마다 하나씩 만든다.

    dict 세 개(poses/docking/quality)의 키는 전부 **tag_id** 다. 리스트 인덱스로
    맞추지 않는 이유는, 중간에 한 태그의 자세 계산만 실패해도 인덱스가 밀려
    엉뚱한 태그의 숫자를 읽게 되기 때문이다. 실패한 태그는 그냥 키가 없다.

    Attributes:
        index      프레임 번호 (소스가 준 것)
        timestamp  [초] 소스 기준 시각. 영상파일이면 재생시각, RealSense 면
                   첫 프레임 기준 경과초.
                   **오버레이를 지운 쪽**이다 — 검출 좌표와 픽셀이 맞아야
                   그 위에 그린 그림이 태그에 붙는다.
                   주의: 지우기(cv2.inpaint)를 거치면 평범한 ndarray 라
                   depth/luma 가 떨어져 나간다. depth 는 원본에서 뜬다(아래 참고).
        detections AT2 검출 결과 리스트. tag_id 오름차순.
        poses      {tag_id: T_camera_tag 4x4}. 자세가 NaN 이면 **키가 없다.**
        docking    {tag_id: docking_state() dict}
        quality    {tag_id: pose_quality() dict}. quality=False 로 만들었으면 빈 dict.
                   depth 가 같이 온 프레임이면 여기에 "depth" 키가 하나 더 붙는다
                   (= depth_cross_check() 결과). 자세거리와 실측거리 대조다.
        intrinsics 이 프레임에 실제로 쓴 CameraIntrinsics
        errors     {tag_id 또는 "detect": 사유 문자열}. 삼킨 실패를 여기 남긴다.
                   비어 있어야 정상이다.
    """
    index: int
    timestamp: float
    image: object
    detections: list = field(default_factory=list)
    poses: dict = field(default_factory=dict)
    docking: dict = field(default_factory=dict)
    quality: dict = field(default_factory=dict)
    intrinsics: object = None
    errors: dict = field(default_factory=dict)

    @property
    def tag_ids(self):
        return [int(d.tag_id) for d in self.detections]

    def __len__(self):
        return len(self.detections)

    def primary(self, tag_id=None):
        """이 프레임에서 **믿고 쓸 태그 하나**를 골라 한 묶음으로 돌려준다.

        Args:
            tag_id: 정해진 태그가 있으면 그 번호. 없으면 화면에서 제일 큰 태그.

        고르는 기준이 "제일 큰"인 이유: 태그 픽셀 크기가 곧 자세 정확도다
        (거리 오차는 대략 1/tag_px 에 비례한다). 크기는 cv2.contourArea 가 아니라
        tag_pixel_size(네 변 길이 평균)로 잰다 — 면적은 태그가 기울면 원근으로
        납작해져서 "멀어졌다"와 "돌아갔다"를 구분하지 못한다(실측 74도에서
        면적환산 121px vs 변평균 190px).

        Returns:
            dict(tag_id, detection, T, docking, quality) 또는 None.
            자세가 없는 태그(NaN 으로 실패)는 고르지 않는다. tag_id 를 지정했는데
            그 태그가 없거나 자세가 없으면 None 이다 — **빈 dict 가 아니라 None**
            이라 `if res.primary():` 한 줄로 갈린다.
        """
        usable = [d for d in self.detections if int(d.tag_id) in self.poses]
        if not usable:
            return None
        if tag_id is not None:
            usable = [d for d in usable if int(d.tag_id) == int(tag_id)]
            if not usable:
                return None
            det = usable[0]
        else:
            det = max(usable, key=tag_pixel_size)
        tid = int(det.tag_id)
        return {"tag_id": tid, "detection": det, "T": self.poses[tid],
                "docking": self.docking.get(tid), "quality": self.quality.get(tid)}


#: TagPipeline 쪽 인자 이름들. 소스 여는 함수(open_realsense 등)로 새어 들어가면
#: TypeError 가 나므로 classmethod 에서 여기 적힌 것만 걸러낸다.
#: **open_realsense 의 depth= 와 헷갈리지 않게 파이프라인 쪽은 depth_check 다.**
_PIPE_KEYS = ("detector", "families", "quad_blur", "method", "min_margin",
              "max_hamming", "gray_channel",
              "hfov", "quality", "depth_check", "label", "origin")


class TagPipeline:
    """소스 한 개 + 검출기 한 개를 들고, 프레임마다 Result 를 뱉는다.

            for res in pipe:
                p = res.primary()
                if p:
                    print(res.index, p["docking"]["forward"], p["docking"]["lateral"])

    소스 없이 이미지만 넣어도 된다. 프레임을 직접 들고 있는 코드(노트북, 다른
    캡처 루프)를 위한 경로다.

        pipe = TagPipeline(tag_size=0.20, intrinsics=intr)
        res = pipe.process(bgr)

    **이건 편의층이다.** 1~5단계 함수들을 대체하지 않는다. 조금이라도 다르게
    엮어야 하면 낱개 함수를 직접 부르는 쪽이 맞다.

    Attributes:
        frames  소스 제너레이터(_FrameStream 또는 아무 3-튜플 iterable). 없으면 None.
        intr    CameraIntrinsics. 영상파일이면 첫 프레임에서 지어낸다.
        intrinsics_assumed  intr 이 화각 가정으로 지어낸 것인가.
            **True 면 거리값에 의심 표시를 붙여야 한다** — 거리는 fx 에 정비례한다.
        tag_size 태그 실제 한 변 [m]. 기본값이 없다(아래 참고).
        origin  intr 이 어디서 왔는가를 적은 문자열. 화면에 그대로 띄우라고 둔 것.
    """

    def __init__(self, frames=None, intrinsics=None, tag_size=None, detector=None,
                 families="tag36h11", quad_blur=DEFAULT_QUAD_BLUR, method="auto",
                 min_margin=0.0, max_hamming=0, gray_channel=None,
                 hfov=ASSUMED_HFOV_DEG, quality=True, depth_check=True,
                 label="", origin="", close=None):
        """
        Args:
            tag_size: **필수다. 기본값을 두지 않았다.**
                이 값이 틀리면 거리 전체가 그 비율만큼 조용히 틀어진다(화면은
                멀쩡해 보인다). 0.20 을 기본으로 박아두면 20cm 가 아닌 태그를
                쓰는 날 아무도 눈치채지 못한다. 인쇄한 태그를 자로 재서 넣을 것 —
                **검은 테두리 바깥까지** 포함한 한 변이다.
            intrinsics: 없으면 첫 프레임에서 hfov 가정으로 지어낸다(ASSUMED).
            detector: 이미 만든 AT2 검출기가 있으면 그걸 쓴다. 없으면 하나 만든다.
                검출기는 프레임마다 만들면 안 된다 — 태그군 테이블을 매번 새로 짠다.
            method: estimate_pose 의 "auto"/"tag"/"pnp".
            gray_channel: to_gray(channel=...). 단색 오버레이가 있는 영상에서
                오염된 채널을 피하는 수단이다.
            quality: pose_quality 를 같이 계산할지. 끄면 detection_pose 를 한 번
                덜 부른다(그 함수가 재투영오차를 다시 뽑느라 자세를 다시 푼다).
            depth_check: 프레임에 depth 가 있으면 거리 대조도 할지.
                depth 가 없으면 있으나 마나다(비용 0).
            close: 소스를 닫는 함수. from_* 가 알아서 채운다.
        """
        if tag_size is None:
            raise ValueError(
                "tag_size 를 반드시 줘야 한다 [m]. 거리가 이 값에 정비례하므로 "
                "기본값을 두면 틀린 거리가 조용히 나온다. 검은 테두리 바깥까지 잰 한 변.")
        self.frames = frames
        self.intr = intrinsics
        self.tag_size = float(tag_size)
        self.detector = detector if detector is not None else make_detector(
            families=families, quad_blur=quad_blur)
        self.method = method
        self.min_margin = float(min_margin)
        self.max_hamming = int(max_hamming)
        self.gray_channel = gray_channel
        self.hfov = float(hfov)
        self.quality = bool(quality)
        self.depth_check = bool(depth_check)
        self.label = label
        self.origin = origin or ("given" if intrinsics is not None else "")
        self.intrinsics_assumed = False
        self._close = close
        self._n = 0

    # ---------------------------------------------------------------- 소스별 생성
    @staticmethod
    def _split_kw(kw):
        """kw 를 (파이프라인용, 소스용) 으로 가른다. 이름이 겹치는 인자는 없다."""
        pipe = {k: kw.pop(k) for k in list(kw) if k in _PIPE_KEYS}
        return pipe, kw

    @classmethod
    def from_realsense(cls, tag_size, stream="color", **kw):
        """실물 D435i 를 열어 파이프라인을 만든다. 남는 인자는 open_realsense 로 간다.

            with TagPipeline.from_realsense(0.20, tune=True) as pipe: ...

        **카메라는 한 프로세스에서 하나만 열 수 있다.** with 문으로 쓰거나
        close() 를 꼭 부를 것 — 안 그러면 다음 실행이 "Device or resource busy" 다.
        내부파라미터는 장치의 공장 캘리브레이션이라 ASSUMED 가 아니다.
        """
        pipe_kw, open_kw = cls._split_kw(kw)
        frames, intr = open_realsense(stream=stream, **open_kw)
        pipe_kw.setdefault("label", "realsense/%s" % stream)
        pipe_kw.setdefault("origin", "factory (RealSense %s)" % stream)
        return cls(frames, intrinsics=intr, tag_size=tag_size,
                   close=frames.close, **pipe_kw)

    @classmethod
    def from_bag(cls, path, tag_size, **kw):
        """녹화한 .bag 을 재생한다. 남는 인자는 open_bag 으로 간다.

        카메라 없이, 매번 **완전히 같은 입력**으로 돌 수 있다. 임계값을 만질 때는
        이쪽이 맞다. 내부파라미터는 bag 자신의 스트림 프로파일에서 읽으므로
        지금 꽂힌 카메라와 달라도 정확하다.
        """
        pipe_kw, open_kw = cls._split_kw(kw)
        frames, intr = open_bag(str(path), **open_kw)
        pipe_kw.setdefault("label", "bag (%s)" % Path(path).name)
        pipe_kw.setdefault("origin", "bag stream profile")
        return cls(frames, intrinsics=intr, tag_size=tag_size,
                   close=frames.close, **pipe_kw)

    @classmethod
    def from_video(cls, path, tag_size, intrinsics=None, hfov=ASSUMED_HFOV_DEG,
                   loop=False, **kw):
        """영상 파일. 카메라 값이 없으므로 intrinsics 를 주거나 화각을 가정한다.

        Args:
            intrinsics: 그 영상을 찍은 카메라의 값. 있으면 이게 맞는 길이다.
            hfov: intrinsics 가 없을 때의 수평화각 가정 [도].
                이 경로로 가면 .intrinsics_assumed 가 True 가 되고,
                **거리는 가정값이다** — 화면에 그대로 띄울 때 표시를 붙일 것.
        """
        pipe_kw, _rest = cls._split_kw(kw)
        if _rest:
            raise TypeError("모르는 인자: %s" % ", ".join(sorted(_rest)))
        frames, _none = from_video(path, loop=loop)
        pipe_kw.setdefault("label", "video (%s)" % Path(path).name)
        pipe_kw.setdefault("origin", "given" if intrinsics is not None else "")
        return cls(frames, intrinsics=intrinsics, tag_size=tag_size, hfov=hfov,
                   close=frames.close, **pipe_kw)

    # ---------------------------------------------------------------- 본체
    def intrinsics_for(self, shape):
        """카메라 값을 확정한다. 없으면 화면 크기 + 화각 가정으로 지어낸다.

        영상/웹캠은 **첫 프레임을 받아봐야 해상도를 안다.** 그래서 미리 못 만들고
        여기서 늦게 만든다. 지어낸 순간 intrinsics_assumed 에 낙인을 찍는다.
        """
        if self.intr is None:
            self.intr = intrinsics_from_hfov(shape, self.hfov)
            self.intrinsics_assumed = True
            self.origin = "ASSUMED from hfov=%.0fdeg" % self.hfov
        return self.intr

    def process(self, img, index=None, timestamp=None):
        """이미지 한 장 -> Result. 어떤 실패도 밖으로 새지 않는다.

        순서: (오버레이 제거) -> to_gray -> detect -> estimate_pose ->
              docking_state -> pose_quality (+ depth 대조)

        depth 는 **원본 프레임**에서 뜬다. 오버레이 제거를 거치면 cv2.inpaint 가
        평범한 ndarray 를 돌려주면서 Frame 에 붙어 있던 depth/luma 가 떨어져
        나가기 때문이다. 검출 좌표는 지운 이미지 기준이지만 두 이미지는 픽셀이
        1:1 로 같으므로 그대로 인덱싱해도 맞는다.
        """
        src = img                                   # depth/luma 가 붙어 있는 원본
        gray = to_gray(img, channel=self.gray_channel)
        intr = self.intrinsics_for(np.asarray(img).shape)

        if index is None:
            index = self._n
        self._n = index + 1

        res = Result(index=int(index),
                     timestamp=float(timestamp) if timestamp is not None else 0.0,
                     image=img, intrinsics=intr)

        try:
            res.detections = detect(self.detector, gray,
                                    min_margin=self.min_margin,
                                    max_hamming=self.max_hamming)
        except Exception as exc:
            # 검출기가 넘어져도 루프는 계속 돌아야 한다(한 프레임의 노이즈일 수 있다).
            # 다만 조용히 "태그 없음"으로 보이면 안 되니 이유를 남긴다.
            res.errors["detect"] = "%s: %s" % (type(exc).__name__, exc)
            return res

        depth = getattr(src, "depth", None)
        depth_scale = getattr(src, "depth_scale", 0.0) or None

        for d in res.detections:
            tid = int(d.tag_id)
            try:
                T, _e0, _e1 = estimate_pose(self.detector, d, intr, self.tag_size,
                                            method=self.method)
            except Exception as exc:
                res.errors[tid] = "estimate_pose %s: %s" % (type(exc).__name__, exc)
                continue
            T = np.asarray(T, dtype=np.float64)
            if T.shape != (4, 4) or not np.isfinite(T).all():
                # NaN 자세는 **키를 아예 안 만든다.** 넣어두면 뒤에서 NaN 거리가
                # 숫자처럼 흘러다닌다. 이유는 errors 에 남는다.
                res.errors[tid] = "non-finite pose (NaN)"
                continue

            res.poses[tid] = T
            res.docking[tid] = docking_state(T)
            if self.quality:
                q = pose_quality(self.detector, d, intr, self.tag_size, T,
                                 method=self.method)
                if self.depth_check and depth is not None:
                    # 자세로 잰 거리와 depth 로 잰 거리를 맞춰 본다. 원리가 아예
                    # 다른 두 측정이라 갈리면 둘 중 하나가 확실히 틀린 것이다.
                    q["depth"] = depth_cross_check(d, src, T, depth_scale=depth_scale)
                res.quality[tid] = q
        return res

    # ---------------------------------------------------------------- 소스 다루기
    def __iter__(self):
        """소스를 끝까지 돌며 Result 를 뱉는다. 3-튜플 계약을 그대로 소비한다."""
        if self.frames is None:
            raise RuntimeError("소스 없이 만든 파이프라인이다 — process(img) 로 한 장씩 넣어라.")
        for i, ts, img in self.frames:
            yield self.process(img, index=i, timestamp=ts)

    @property
    def stats(self):
        """소스의 프레임 드롭 회계(FrameStats). 없는 소스면 None.

        _FrameStream 의 속성을 밖에서 직접 꺼내 쓰던 자리를 여기로 모았다 —
        `getattr(source.frames, "stats", None)` 은 이름이 바뀌면 **조용히**
        None 이 되어 회계가 사라진 걸 아무도 모른다.
        """
        return getattr(self.frames, "stats", None)

    @property
    def ae_roi(self):
        """자동노출 ROI 객체(open_realsense(ae_roi=True) 일 때만). 없으면 None.

        **매 프레임 roi.follow(detections, img.shape) 를 불러 주는 건 호출자 몫이다.**
        여기서 자동으로 부르지 않는 이유: 태그를 쫓아가는 게 항상 옳지는 않다.
        """
        return getattr(self.frames, "ae_roi", None)

    def close(self):
        """소스를 닫는다. 몇 번 불러도 안전하다.

        RealSense 는 **꼭 닫아야 한다** — UVC 설정이 장치에 남고, 파이프라인을
        놓지 않으면 다음 실행이 "Device or resource busy" 로 죽는다.
        """
        c, self._close = self._close, None
        if c is not None:
            try:
                c()
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
