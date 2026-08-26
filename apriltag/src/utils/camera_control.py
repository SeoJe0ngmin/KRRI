"""RealSense 컬러 센서 **쓰기(제어)** 쪽만 모아 둔 곳.

왜 rs_tuning.py 도 models/tag_pose.py 도 아닌 새 파일인가:
    - models/tag_pose.py 의 open_realsense 는 "프레임을 어떻게 얻는가"다.
      스트림을 열고 버퍼를 넘긴다. (예전 이름은 image_source.py 였다.)
    - rs_tuning.py 는 "카메라가 무슨 상태인지 **읽는**" 쪽이다
      (yuyv_to_luma / FrameStats / frame_meta / describe_metadata). 전부 읽기·관측이고
      틀려도 관측이 틀릴 뿐이다.
    - 이 파일만 **장치에 값을 쓴다.** 쓰기는 성질이 다르다.
      (a) 순서가 중요하다 — AE 를 끄면 SDK 가 노출을 공장기본값으로 되돌려버린다
          (아래 CameraSettings.apply 참고). 순서를 틀리면 조용히 원하는 값이 아니게 된다.
      (b) 장치에 남는다 — 스트림을 닫아도 UVC PU 값은 그대로다. 다음 실행이,
          심지어 realsense-viewer 가 그 값을 물려받는다.
      (c) src/etc/viewer_filter 의 C++ 튜닝 결과가 그대로 여기로 들어온다. 뷰어에서 만진
          값 하나가 파이썬 필드 하나에 1:1 로 대응해야 옮겨 적기가 안전하다.
    섞어 두면 "읽기만 하는 줄 알았는데 카메라 상태가 바뀌어 있더라"가 난다. 그래서 가른다.

여기 들어온 것은 **컬러 스트림 태그 검출에 실제로 영향이 있다고 근거가 선 것만**이다.
근거 없이 들어온 값은 하나도 없고, 근거가 "그럴 것 같다" 수준인 것은 전부 빠졌다.
빠진 것과 그 이유는 이 파일 맨 아래 _WHY_NOT 에 적어 뒀다 — 나중에 누가 다시
"샤프니스 올리면 낫지 않을까" 하고 돌아오는 것을 막기 위해서다.

rs_tuning.set_color_exposure() 와의 관계:
    거기는 **노출 하나를 지금 당장 바꾸는** 명령형 함수다(us 를 받아 넣고 되읽는다).
    여기는 **상태 묶음**이다 — 뜨고(from_sensor) 바꾸고(apply) 되돌린다.
    둘 다 필요하다. 진단하며 노출만 툭툭 바꿔볼 때는 저쪽이 짧고,
    "도킹 상태로 만들었다가 끝나면 원래대로" 는 이쪽이라야 원래대로가 가능하다.
    같은 옵션을 만지므로 한 프로세스에서 섞어 쓰면 나중에 쓴 쪽이 이긴다.

[근거] librealsense 소스(직접 확인), pyrealsense2 2.58.3,
       그리고 data/apriltag/Testing_apriltag_trim.mp4 28프레임 실측.
"""
from dataclasses import dataclass, asdict, replace, fields


# ── 단위와 상수 ──────────────────────────────────────────────────────────────

# 컬러 exposure 눈금(100us)과 블러 절벽(10px/32px)은 rs_tuning 이 이미 들고 있다.
# 여기서 다시 정의하지 않고 가져다 쓴다 — 같은 숫자를 두 군데 적어 두면 한쪽만
# 고쳐지는 날이 반드시 온다. (뎁스 센서의 exposure 는 us 단위라 100배 다르다.)
from .rs_tuning import (COLOR_EXPOSURE_UNIT_US,
                        BLUR_CLEAN_PX as BLUR_PX_100PCT,
                        BLUR_DEAD_PX as BLUR_PX_ZERO)

#: 한국 상용전원 60 Hz. 형광등/저가 LED 는 그 **두 배**인 120 Hz 로 깜빡인다.
#: 그래서 깜빡임에 안 걸리는 노출시간은 1/120 s = 8.333 ms 의 정수배다.
MAINS_HALF_CYCLE_MS = 1000.0 / 120.0

# BLUR_PX_100PCT(10) / BLUR_PX_ZERO(32) 는 위에서 rs_tuning 것을 그대로 쓴다.
# 실측 절벽: 0~10px 28/28(100%) | 12px 96.4% | 16px 92.9% | 20px 89.3%
#            24px 78.6% | 28px 50.0% | 32px 0/28(0.0%)
# 태그 238px(tag36h11 은 10칸이므로 한 칸 23.8px) 기준이라 10px 는 0.42칸,
# 완전붕괴 32px 는 1.34칸이다. **절벽은 픽셀이 아니라 "태그 칸의 몇 분의 몇"에 있다.**

#: realsense-viewer 의 AE ROI "reset" 이 쓰는 상자 = 화면 가운데 3/4
#: (common/stream-model.cpp:331-343, 사방으로 크기의 1/8 씩 뗀다).
#: 1920x1080 이면 (240, 135, 1679, 944). 사실상 이게 기본 ROI 다.
AE_ROI_MARGIN_FRACTION = 1.0 / 8.0


# ── 노출 시간을 숫자로 정하는 근거 ───────────────────────────────────────────

def exposure_ms_for_motion(speed_mps, range_m, fx=1359.2, blur_px=BLUR_PX_100PCT):
    """이 속도/거리에서 블러를 blur_px 안에 묶는 노출시간 [ms].

    유일한 식은 blur_px = fx * v * t / z 다. 지어낸 게 아니라 핀홀 그 자체다 —
    옆으로 v [m/s] 로 움직이면 z [m] 거리의 점은 이미지에서 fx*v/z [px/s] 로 흐르고,
    셔터가 t 초 열려 있으면 그만큼 번진다.

    이걸 위의 **실측 블러 절벽**과 붙이면 노출 상한이 나온다.
    fx=1359.2 (1920x1080 공장 내부파라미터), 태그 0.2m 기준 실측표:

        속도     거리   10px(100%)  32px(0%)   AE 최대치 33.3ms 일 때 블러 -> 검출률
        0.3 m/s  1.0 m   24.5 ms     78.5 ms      14 px  -> 약 90%
        0.5 m/s  1.0 m   14.7 ms     47.1 ms      23 px  -> 약 50%
        1.0 m/s  1.0 m    7.4 ms     23.5 ms      45 px  -> 0%
        1.5 m/s  1.0 m    4.9 ms     15.7 ms      68 px  -> 0%
        2.0 m/s  1.0 m    3.7 ms     11.8 ms      91 px  -> 0%

    마지막 열이 이 파일이 존재하는 이유다. 어두운 창고에서 자동노출은 프레임시간
    33.3 ms 를 다 쓸 수 있고, 그러면 1 m/s 로 접근하는 지게차는 **검출률이 떨어지는
    게 아니라 0 이 된다.** 서서 찍은 사진이 잘 나온다고 안심하면 안 된다.

    Args:
        speed_mps: 태그를 가로지르는 상대속도 [m/s]. 정면 접근이면 실제로는
            더 느리게 흐르므로 이 값은 보수적(안전한) 쪽이다.
        range_m: 태그까지 거리 [m]. 가까울수록 빡세진다 — 같은 속도라도
            0.5 m 에서는 1 m 일 때의 **절반** 노출만 허용된다.
        fx: 초점거리 [px]. 1920x1080 공장값 1359.2. 해상도를 낮추면 같이 줄여야 한다.
        blur_px: 허용 블러. 기본 10px = 실측 100% 검출 상한.

    Returns:
        노출시간 [ms]. float.
    """
    speed_mps = abs(float(speed_mps))
    if speed_mps <= 0.0:
        return float("inf")                  # 정지 상태면 블러 상한이 없다
    return 1000.0 * float(blur_px) * float(range_m) / (float(fx) * speed_mps)


def exposure_units(ms, unit_us=COLOR_EXPOSURE_UNIT_US):
    """밀리초 -> 컬러 exposure 옵션 값. 최소 1 칸은 보장한다."""
    return max(1, int(round(float(ms) * 1000.0 / float(unit_us))))


def exposure_ms(units, unit_us=COLOR_EXPOSURE_UNIT_US):
    """컬러 exposure 옵션 값 -> 밀리초."""
    return float(units) * float(unit_us) / 1000.0


#: 도킹 기본 노출값. 왜 하필 83 인가 — 두 가지 제약이 같은 곳에서 만난다.
#:  1) 블러: 위 표에서 1.0 m/s / 1.0 m 를 100% 로 지키려면 7.4 ms 이하여야 한다.
#:  2) 깜빡임: 수동노출이어도 형광등 120 Hz 는 그대로 있다. 노출이 반주기의
#:     정수배가 아니면 프레임마다 밝기가 맥놀이치고, 짧은 노출에서는 화면에
#:     가로 줄무늬가 앉는다. 줄무늬가 태그를 가로지르면 사각형이 쪼개진다.
#:     그래서 8.333 ms 의 정수배여야 한다.
#: 8.3 ms(=83칸) 이 1번 상한 7.4 ms 를 아주 살짝 넘지만, 그 대가는
#: 블러 11.3px = 실측 12px 구간(96.4%) 이고, 대신 깜빡임 맥놀이를 거의 없앤다.
#: **"거의"인 이유를 분명히 해 둔다:** 컬러 노출은 100us 격자라
#: 반주기 8.333 ms 를 정확히 못 짚는다. 짚을 수 있는 건 8.3 또는 8.4 ms 이고,
#: 83칸은 반주기와 33us(0.4%) 어긋난다. 그래서 맥놀이가 사라지는 게 아니라
#: 주기가 아주 길어져(약 250 프레임= 8초) 프레임 간 밝기 변화가 눈에 안 띌 뿐이다.
#: 반주기와 한참 어긋난 7.4 ms(11% 오차)를 고르면 프레임마다 밝기가 출렁인다.
#: 더 빠르게 붙일 계획이면 여기가 아니라 exposure_ms_for_motion() 으로 다시 계산할 것.
DOCKING_EXPOSURE_UNITS = exposure_units(MAINS_HALF_CYCLE_MS)   # = 83


# ── 설정 묶음 ────────────────────────────────────────────────────────────────

@dataclass
class CameraSettings:
    """컬러 센서에서 **태그 검출에 실제로 영향이 있는** 옵션만 담은 묶음.

    쓰는 법:
        before = CameraSettings.from_sensor(sensor)   # 지금 상태를 뜬다
        CameraSettings.docking().apply(sensor)        # 도킹용으로 바꾼다
        ...
        before.apply(sensor)                          # 원래대로 되돌린다

    필드가 None 이면 **건드리지 않는다**는 뜻이다. 0 이나 False 와 다르다 —
    False 는 "꺼라"이고 None 은 "네가 알아서 해라"다. 부분 설정을 표현하려고
    이렇게 뒀다(예: 노출만 바꾸고 화이트밸런스는 현장 그대로 두고 싶을 때).

    여기 **없는** 것들(샤프니스/감마/대비/밝기/채도/색조/화이트밸런스 값/
    역광보정)은 빠뜨린 게 아니라 근거를 대고 뺀 것이다. 이유는 _WHY_NOT 참고.
    """

    #: 자동노출. False 면 노출이 프레임 사이에 안 움직인다.
    #: 왜 끄나: AprilTag 의 임계화는 국소 적응형이라 **느린** 밝기 변화는 견딘다.
    #: 문제는 AE 가 접근 중에 다시 수렴한다는 것이다. 지게차가 붙으면 태그가
    #: 화면에서 커지면서 화면 평균 밝기를 끌고 가고, AE 는 하필 자세가 가장
    #: 중요한 구간에서 사냥을 시작한다. 프레임마다 흑백 경계가 움직이면
    #: 모서리가 떨고, 그게 그대로 자세 떨림이다.
    enable_auto_exposure: bool = None

    #: 노출값. **단위는 100us** (COLOR_EXPOSURE_UNIT_US). 83 = 8.3 ms.
    #: 여기에 값을 넣으면 자동노출은 **자동으로 꺼진다**. 예외가 나지 않는다 —
    #: SDK 소스로 확인: exposure 는 auto_disabling_control 로 한 겹 싸여 있고
    #: (ds-color-common.cpp:90-93), 그 set() 은 AE 가 켜져 있으면 먼저 AE 를 0 으로
    #: 내린 뒤 값을 쓴다(option.cpp:84-105). 즉 "AE 를 켠 채로 노출만 지정"은
    #: 애초에 불가능하고, 시도하면 조용히 AE 가 꺼진다.
    #: (뎁스 센서의 exposure 는 **us 단위**라 100배 다르다. 여기는 컬러다.)
    exposure: float = None

    #: 아날로그 게인. 녹화된 D435 기본값 64.
    #: 노출을 줄인 만큼 여기서 되받는 것이 정석이지만, **우리 경우엔 아직
    #: 올릴 필요가 없다**: 실측 노출 스윕에서 기본 노출(156) 대비 1/16 까지
    #: 낮춰도 28/28(100%) 이 유지됐고 margin 은 19.5 였다. 우리가 쓰는 83 은
    #: 0.53배에 불과해 margin 이 약 52 로 남는다 — 하한 20 대비 2.6배 여유다.
    #: 게인을 올리면 그 여유를 노이즈로 바꾸는 셈이라 지금은 손해다.
    #: 실제 창고가 이 영상보다 어두우면 그때 올린다(margin 이 20 에 붙는지로 판단).
    gain: float = None

    #: 0=끔 1=50Hz 2=60Hz 3=자동 (d400-color.cpp:205-212 의 값 매핑 그대로).
    #: 한국은 2. 자동(3)은 영상만 보고 추측하는 것이라 혼합조명에서 잘못 물 수 있다.
    #: 주의: 수동노출에서는 이 값이 **아무 일도 안 한다** — AE 탐색을 제약하는
    #: 물건이기 때문이다. 그래도 넣어 두는 이유는 (a) AE 로 되돌릴 때를 위해서고
    #: (b) 이 값이 맞아야 realsense-viewer 로 눈으로 볼 때 같은 그림이 나오기 때문이다.
    #: 수동노출에서 깜빡임을 피하는 진짜 수단은 노출을 8.333ms 의 정수배로 두는 것이다.
    power_line_frequency: int = None

    #: 1 이면 어두울 때 AE 가 **프레임률을 떨어뜨려서** 노출을 더 벌 수 있다. 기본값이 1.
    #: 0 이면 프레임률을 고정하고 AE 는 프레임시간(30fps 면 33.3ms) 안에 머문다.
    #: 왜 0 인가: 어두운 통로에서 조용히 30fps -> 15fps 로 떨어지면 자세 갱신 간격이
    #: 두 배가 된다. 지게차는 그동안에도 움직인다. 이건 검출 문제처럼 보이는
    #: 제어 문제다. 수동노출이면 무의미하지만, AE 로 되돌릴 때를 위해 같이 박아 둔다.
    auto_exposure_priority: int = None

    #: 자동 화이트밸런스. 잠근다.
    #: 근거(실측): librealsense 자신의 YUY2->BGR 정수행렬(298/409/-100/-208/516,
    #: color-formats-converter.cpp:293-301)에 cv2.COLOR_BGR2GRAY 를 합성해서
    #: YUV 큐브를 훑었더니 **클리핑 안 된 픽셀에서는 색차항이 정확히 상쇄**된다
    #: (40048 샘플에서 최대 오차 1 단계). 즉 제대로 노출된 회색 태그에서는
    #: 화이트밸런스가 진짜로 무의미하다.
    #: **그런데 한쪽 채널이 클리핑되는 순간 상쇄가 깨진다.** 하필 태그가 사는
    #: 밝기(흰칸 Y=235 근처, 검은칸 Y=16~30)에서 U,V 가 +-40 흔들리면 회색값이
    #: 최대 28 단계까지 밀린다. 즉 AWB 는 "노출이 맞을 땐 무의미, 태그가 날아가거나
    #: 뭉개지는 순간 진짜 방해"다. 잠그는 건 공짜고 비동기 사냥 루프 하나를 없앤다.
    #: (값 자체인 white_balance 는 넣지 않았다 — 위 상쇄 결과 때문에 어떤 켈빈값을
    #:  골라도 회색 1 단계 안이다. 튜닝할 가치가 없다.)
    enable_auto_white_balance: bool = None

    #: SDK 쪽 프레임 큐 깊이(펌웨어 아님). 녹화된 기본값 16.
    #: 화질 옵션이 아니지만 도킹에서는 중요하다: 16 짜리 큐에 우리 루프가 30fps 를
    #: 못 따라가면 큐가 밀리고 wait_for_frames 가 **과거 프레임**을 준다. 지게차가
    #: 0.5초 전에 있던 자리의 자세를 정확하게 계산해 봐야 소용이 없다.
    #: docking_state() 가 그 자세를 먹으므로 지연은 제어 오차로 나타난다.
    #: 1 로 두면 쌓는 대신 버린다 — 제어 루프에서는 이쪽이 맞다.
    frames_queue_size: int = None

    #: 프레임 타임스탬프를 호스트 시계에 맞춘다. 검출 품질과는 무관하다.
    #: 지게차 오도메트리나 호스트 시계로 도는 제어기와 자세를 엮을 때만 의미가 있다.
    #: 주의: 하드웨어 시계 도메인일 때만 동작한다. WSL2 처럼 UVC 메타데이터가
    #: 없는 환경에서는 켜도 아무 일도 안 일어난다(rs_tuning.timestamp_domain 참고).
    global_time_enabled: bool = None

    # ── 만들기 ──────────────────────────────────────────────────────────────

    @classmethod
    def docking(cls, speed_mps=None, range_m=1.0, fx=1359.2):
        """도킹용 기본 묶음.

        speed_mps 를 주면 노출을 그 속도에 맞춰 다시 계산한다(반주기 정수배로 내림).
        안 주면 DOCKING_EXPOSURE_UNITS(=83, 8.3ms) 를 쓴다 — 1 m/s 접근까지 커버한다.

            CameraSettings.docking()                      # 8.3 ms
            CameraSettings.docking(speed_mps=2.0)         # 더 빠르면 더 짧게

        Args:
            speed_mps: 접근 속도 [m/s]. None 이면 기본 8.3 ms.
            range_m: 이 속도를 견뎌야 하는 가장 가까운 거리 [m]. 가까울수록 빡세다.
            fx: 초점거리 [px]. 해상도를 낮췄으면 같이 줄일 것.
        """
        units = DOCKING_EXPOSURE_UNITS
        if speed_mps:
            ms = exposure_ms_for_motion(speed_mps, range_m, fx)
            # 깜빡임 때문에 반주기(8.333ms)의 정수배로 내린다. 한 주기 밑으로는 못 간다 —
            # 그 아래는 어차피 깜빡임을 피할 수 없으니 블러 쪽 요구를 그대로 따른다.
            n = int(ms / MAINS_HALF_CYCLE_MS)
            units = exposure_units(n * MAINS_HALF_CYCLE_MS) if n >= 1 else exposure_units(ms)
        return cls(
            enable_auto_exposure=False,
            exposure=float(units),
            gain=64.0,                        # 공장 기본값 그대로. 위 gain 주석 참고
            power_line_frequency=2,           # 60Hz (한국)
            auto_exposure_priority=0,         # 프레임률 고정
            enable_auto_white_balance=False,
            frames_queue_size=1,              # 최신 프레임만
            global_time_enabled=True,
        )

    @classmethod
    def from_sensor(cls, sensor):
        """지금 센서 상태를 그대로 뜬다. 지원 안 하는 옵션은 None 으로 남는다.

        되돌리기용으로 먼저 떠 두는 것이 핵심 용도다 — UVC PU 값은 스트림을 닫아도
        장치에 남아서, 안 떠 두면 realsense-viewer 를 다음에 열었을 때 우리가 바꾼
        상태를 물려받는다.
        """
        import pyrealsense2 as rs
        s = color_sensor(sensor)
        out = {}
        for f in fields(cls):
            opt = getattr(rs.option, f.name, None)
            if opt is None:
                continue
            try:
                if not s.supports(opt):
                    continue
                v = s.get_option(opt)
            except Exception:
                continue
            if f.type is bool or f.name.startswith(("enable_", "global_time")):
                out[f.name] = bool(round(v))
            elif f.type is int:
                out[f.name] = int(round(v))
            else:
                out[f.name] = float(v)
        return cls(**out)

    # ── 쓰기 ────────────────────────────────────────────────────────────────

    def apply(self, sensor, strict=False):
        """센서에 쓴다. **순서가 전부다.**

        순서를 이렇게 잡은 이유:
          1) power_line_frequency / auto_exposure_priority 를 먼저 쓴다.
             AE 를 끄기 전에 써야 AE 로 되돌렸을 때도 그대로 유효하다.
          2) enable_auto_exposure 를 그 다음에 쓴다.
          3) exposure / gain 의 자리는 **AE 를 켜느냐 끄느냐로 갈린다.**
             - 끄는 경우(도킹): AE 를 먼저 끄고 노출/게인을 그 뒤에.
             - 켜는 경우(되돌리기): 노출/게인을 먼저 쓰고 AE 를 마지막에.
               (AE 를 먼저 켜 버리면 그 뒤의 노출 쓰기가 AE 를 도로 끈다.)
             끄는 경우가 왜 그래야 하냐면 —
             SDK 소스에서 확인한 함정: uvc_pu_auto_exposure_option::set() 은
             AE 를 0 으로 쓰면 곧바로 exposure 옵션에 **공장 기본값(156, 15.6ms)** 을
             다시 써 넣는다(ds/ds-color-common.cpp:26-49). 방금 보던 밝기로
             얼어붙는 게 아니라 기본값으로 튄다는 뜻이다.
             그래서 "AE 끄기" 만으로는 아무것도 고정되지 않고, 노출은 AE 를 끈
             **뒤에** 써야 한다. 반대 순서로 하면 조용히 15.6 ms 로 돌아가 있고,
             그 15.6ms 는 1 m/s 접근에서 블러 21px = 검출률 약 89% 다.
             (반대로 exposure 를 먼저 써도 결과는 같다 — auto_disabling_control 이
              AE 를 내린 **뒤** 값을 쓰기 때문이다. 하지만 그 경로는 AE 를 끄는 일이
              부작용으로 일어나 읽는 사람이 놓치기 쉬워, 명시적인 순서를 택했다.)
          4) 화이트밸런스, 큐 깊이, 글로벌 시각은 서로 독립이라 아무데나.

        범위는 장치에서 읽어(get_option_range) 클램프한다. 노출/게인의 실제
        min/max/step 은 이 개체에서 재보지 못했고(카메라 미연결), 재생 bag 은
        범위를 녹화된 한 값으로 뭉개 버려서 알 수 없었다. 그래서 하드코딩하지
        않고 매번 장치에 물어본다.

        Args:
            sensor: rs.sensor / rs.pipeline_profile / rs.device 아무거나
            strict: True 면 못 쓴 옵션에 예외를 던진다. 기본 False —
                펌웨어/모델에 따라 없는 옵션이 있고, 그것 때문에 파이프라인 전체가
                죽는 것보다 조용히 넘기고 보고하는 편이 낫다.

        Returns:
            {옵션이름: (이전값, 이후값)}. 못 쓴 것은 (이전값, None).
            **실제로 뭐가 바뀌었는지 눈으로 확인하라고 돌려주는 것이다.**
            쓰기에 성공해도 펌웨어가 다른 값으로 물릴 수 있어서, 쓴 값이 아니라
            **다시 읽은 값**을 넣는다.
        """
        import pyrealsense2 as rs
        # **아무것도 쓰기 전에** 모순부터 본다. 순서가 중요한 만큼 중간에 터지면
        # 카메라가 "절반만 적용된" 상태로 남는데, 그게 제일 고약하다 —
        # 노출은 안 바뀌었는데 AE 는 꺼져 있는 식이라 밝기가 엉뚱해진다.
        #
        # 다만 **기본은 예외가 아니라 건너뛰기**다. 이유가 있다:
        # from_sensor() 로 뜬 공장상태는 AE=True 이면서 exposure=156, gain=64 를
        # 같이 들고 있다(AE 가 켜져 있어도 현재 노출값은 읽히니까). 그게 정상이다.
        # 여기서 예외를 던지면 **되돌리기가 불가능해진다** — 가장 중요한 용도가 막힌다.
        # AE 를 켜는 요청에서는 exposure/gain 을 **먼저 쓰고 AE 를 마지막에 켠다**
        # (아래 order 참고). 레지스터 값까지 원래대로 돌려놓고 AE 도 켜진 채로
        # 끝나므로, 되돌리기가 실제로 왕복한다(뜬 값과 되돌린 값이 정확히 일치).
        # 사용자가 직접 만든 설정의 모순을 잡고 싶으면 strict=True 를 주면 된다.
        if self.enable_auto_exposure is True and strict:
            self.validate()
        s = color_sensor(sensor)
        report = {}

        common = ("power_line_frequency", "auto_exposure_priority",
                  "enable_auto_white_balance", "frames_queue_size",
                  "global_time_enabled")
        if self.enable_auto_exposure is True:
            # **자동노출을 켜는 요청(주로 되돌리기).**
            # 노출/게인을 먼저 쓰고 AE 를 마지막에 켠다. 그래야 레지스터 값까지
            # 원래대로 복원되면서 AE 도 켜진 채로 끝난다.
            # (AE=1 을 쓰는 것은 노출을 안 건드린다 — uvc_pu_auto_exposure_option::set 은
            #  value==0 일 때만 기본값을 덮어쓴다, ds-color-common.cpp:26-49.
            #  반대로 AE 를 먼저 켜고 노출을 쓰면 그 노출 쓰기가 AE 를 도로 꺼버린다.)
            order = common + ("exposure", "gain", "enable_auto_exposure")
        else:
            # **수동으로 고정하는 요청(도킹).**
            # AE 를 먼저 끄고 노출/게인을 그 뒤에 쓴다 — AE 를 끄는 순간 SDK 가
            # 노출을 공장기본값 156(15.6ms) 으로 되돌려버리기 때문이다.
            order = common + ("enable_auto_exposure", "exposure", "gain")

        for name in order:
            want = getattr(self, name)
            if want is None:
                continue                      # None = 건드리지 않는다
            opt = getattr(rs.option, name, None)
            if opt is None:
                if strict:
                    raise AttributeError("pyrealsense2 에 rs.option.%s 가 없다" % name)
                continue
            try:
                if not s.supports(opt):
                    if strict:
                        raise RuntimeError("센서가 %s 를 지원하지 않는다" % name)
                    report[name] = (None, None)
                    continue
                before = s.get_option(opt)
            except Exception:
                report[name] = (None, None)
                continue

            val = float(want)
            try:                              # 장치가 아는 범위로 자른다
                r = s.get_option_range(opt)
                val = min(max(val, r.min), r.max)
            except Exception:
                pass
            try:
                s.set_option(opt, val)
                after = s.get_option(opt)     # 쓴 값이 아니라 읽은 값을 보고한다
            except Exception:
                if strict:
                    raise
                after = None
            report[name] = (before, after)
        return report

    def validate(self):
        """모순된 조합을 걸러낸다. apply(strict=True) 가 **쓰기 전에** 부른다.

        **기본 apply() 는 이걸 부르지 않는다.** from_sensor() 로 뜬 공장상태가
        AE=True + exposure=156 + gain=64 라는 정상적인 조합이고, 여기서 던지면
        되돌리기가 막히기 때문이다. 그 경우 apply() 는 exposure/gain 을 쓰되
        AE 켜기를 **맨 뒤로** 미뤄 둘 다 성립시킨다.
        이 함수는 "손으로 쓴 설정이 말이 되나" 확인하고 싶을 때 쓰는 것이다.

        지금 걸리는 것은 하나뿐이다:
            enable_auto_exposure=True 인데 exposure 나 gain 도 지정한 경우.
            이건 "자동노출을 쓰면서 노출도 내가 정하겠다"라 애초에 성립하지 않는다.
            그리고 조용히 실패하지도 않는다 — exposure 가 auto_disabling_control 로
            싸여 있어서(ds-color-common.cpp:90-93) 쓰는 순간 SDK 가 AE 를 꺼버린다
            (option.cpp:84-105). 호출자는 AE 가 켜져 있다고 믿는데 실제로는 꺼진다.
            그 어긋남이 나중에 "왜 어두운 데서 안 밝아지지"로 돌아온다.

        Raises:
            ValueError
        """
        if self.enable_auto_exposure is True:
            bad = [n for n in ("exposure", "gain") if getattr(self, n) is not None]
            if bad:
                raise ValueError(
                    "enable_auto_exposure=True 와 %s 를 같이 줄 수 없다 — "
                    "쓰는 순간 SDK 가 AE 를 꺼버린다"
                    "(ds-color-common.cpp:90-93 + option.cpp:84-105). "
                    "노출을 고정하려면 enable_auto_exposure=False 를, "
                    "자동노출을 쓰려면 %s 를 None 으로 둘 것."
                    % (" / ".join(bad), " / ".join(bad)))
        return self

    # ── 곁다리 ──────────────────────────────────────────────────────────────

    @property
    def exposure_ms(self):
        """노출을 밀리초로. None 이면 None."""
        return None if self.exposure is None else exposure_ms(self.exposure)

    def merged(self, **kw):
        """일부만 바꾼 새 묶음. 원본은 안 건드린다."""
        return replace(self, **kw)

    def to_dict(self):
        """JSON 으로 떨궈 두고 나중에 CameraSettings(**d) 로 되살릴 수 있다."""
        return asdict(self)

    def describe(self):
        """사람이 읽을 여러 줄 문자열. 로그에 한 번 찍어 두면 나중에 살아난다."""
        lines = []
        for f in fields(self):
            v = getattr(self, f.name)
            if v is None:
                lines.append("  %-26s -            (건드리지 않음)" % f.name)
            elif f.name == "exposure":
                lines.append("  %-26s %-12g (%.2f ms)" % (f.name, v, exposure_ms(v)))
            else:
                lines.append("  %-26s %s" % (f.name, v))
        return "\n".join(lines)


#: 도킹 기본값. 모듈 상수로도 하나 놔둔다 — open_realsense(tune=True) 가 이걸 쓴다.
DOCKING_SETTINGS = CameraSettings.docking()


# ── 센서 찾기 ────────────────────────────────────────────────────────────────

def color_sensor(obj):
    """무엇을 주든 컬러 센서를 찾아 준다.

    profile / device / sensor 를 다 받는 이유는 호출부마다 손에 든 게 다르기
    때문이다. open_realsense 안에서는 profile 이, 진단 스크립트에서는 device 가,
    이미 센서를 찾아 둔 곳에서는 sensor 가 손에 있다. 매번 변환 코드를 쓰게 하면
    그 변환이 조금씩 다르게 복사된다.

    이름으로 고르는 이유: first_color_sensor() 는 컬러 스트림 프로파일이 있는
    센서를 고르는데, D435i 는 depth 센서도 (정렬용) 컬러를 알고 있는 경우가 있어
    헷갈릴 여지가 있다. "RGB Camera" 라는 이름이 확실하다
    (d400-color.cpp 가 그 이름으로 등록한다). 이름으로 못 찾으면 그때 폴백한다.
    """
    import pyrealsense2 as rs
    if isinstance(obj, rs.sensor) or hasattr(obj, "get_option_range"):
        return obj
    dev = obj.get_device() if hasattr(obj, "get_device") else obj
    for s in dev.query_sensors():
        try:
            if s.get_info(rs.camera_info.name).lower().startswith("rgb"):
                return s
        except Exception:
            pass
    return dev.first_color_sensor()           # 이름을 못 찾았을 때의 최후수단


# ── 한 방에 정리 ─────────────────────────────────────────────────────────────

def tune_for_tags(target, settings=None, speed_mps=None, range_m=1.0, fx=1359.2,
                  verbose=False):
    """카메라를 태그 검출하기 좋은 상태로 만든다. 한 번만 부르면 된다.

        profile = pipeline.start(config)
        before, report = tune_for_tags(profile, verbose=True)
        ...
        before.apply(profile)                 # 끝나고 원상복구

    하는 일과 근거는 CameraSettings 각 필드 주석에 있다. 요약하면:
        자동노출 끄고 8.3 ms 고정 / 게인 기본값 / 60Hz / 프레임률 고정 /
        AWB 잠금 / 큐 깊이 1 / 글로벌 시각 켜기.

    **언제 부르나:** pipeline.start() 직후, 첫 wait_for_frames 전에.
    UVC 값은 스트림 중에 써도 먹지만, 첫 프레임부터 우리가 정한 노출로 받는 편이
    깔끔하다(AE 가 한두 프레임 사냥하고 꺼지는 걸 안 봐도 된다).

    **없는 것 하나 — "자동노출 쓰되 8ms 를 넘지 마라"는 못 한다.**
    그게 움직이는 지게차가 진짜 원하는 물건이지만 컬러 센서에는 없다.
    소스가 명확하다: auto_exposure_limit / gain_limit 은
    get_depth_sensor() 에만, 그것도 CAP_GLOBAL_SHUTTER 일 때만 등록된다
    (ds/d400/d400-device.cpp:1060-1070, 주석 그대로
     "ae / gain limit feature is not supported on rolling-shutter").
    D435i 의 RGB 이미저는 롤링 셔터라 두 겹으로 걸린다.
    pyrealsense2 에 rs.option.auto_exposure_limit **열거값이 있는 것**과
    이 센서가 그걸 **지원하는 것**은 다른 얘기다 — 열거값은 늘 있다.
    실제 답은 ae_limit_supported() 가 카메라에 물어봐 준다.
    그래서 컬러에서 블러를 묶는 방법은 수동노출뿐이고, 그게 이 함수가 하는 일이다.
    (차선책인 auto_exposure_priority=0 은 노출을 프레임시간 33.3ms 로만 묶는데,
     위 표에서 보듯 1 m/s 에서 45px 블러 = 검출 0% 라 전혀 부족하다.)

    Args:
        target: pipeline.start() 가 준 profile / device / 컬러 sensor
        settings: 직접 만든 CameraSettings. None 이면 docking() 을 쓴다.
        speed_mps, range_m, fx: settings 를 안 줬을 때 노출 계산에 쓰인다.
            exposure_ms_for_motion() 참고.
        verbose: True 면 전/후 값을 표로 찍는다.

    Returns:
        (before, report)
        before: 손대기 전 상태(CameraSettings). **이걸 들고 있어야 되돌린다.**
        report: {옵션이름: (이전, 이후)}. 값이 (x, None) 이면 못 쓴 것이다.
    """
    s = color_sensor(target)
    want = settings or CameraSettings.docking(speed_mps=speed_mps, range_m=range_m, fx=fx)
    before = CameraSettings.from_sensor(s)
    report = want.apply(s)
    if verbose:
        print("[tune_for_tags] 컬러 센서 설정")
        for k, (b, a) in report.items():
            if a is None:
                print("  %-26s %-12s -> (못 씀 / 미지원)" % (k, b))
            else:
                extra = "  (%.2f ms)" % exposure_ms(a) if k == "exposure" else ""
                mark = "" if b == a else "  *"
                print("  %-26s %-12g -> %-12g%s%s" % (k, b if b is not None else float("nan"), a, extra, mark))
    return before, report


def ae_limit_supported(target):
    """"자동노출 상한" 이 이 센서에 정말 있는지 카메라에 직접 물어본다.

    소스를 읽은 결론은 **컬러에는 없다**이고(tune_for_tags 설명 참고), 그 결론이
    설계를 좌우했으므로 한 줄로 재확인할 수단을 남겨 둔다. 카메라를 다시 꽂으면
    이 함수 하나로 끝난다.

    Returns:
        {"color": bool, "depth": bool}. 예상은 {"color": False, "depth": False}
        (D435i 는 depth 도 롤링셔터가 아니라 글로벌셔터지만 CAP_GLOBAL_SHUTTER
         플래그와 FW 5.12.10.11 조건이 붙는다 — 그래서 depth 도 실제로 물어본다).
        만약 color 가 True 로 나오면 소스 읽기가 틀린 것이니, 그때는 수동노출
        대신 auto_exposure_limit 를 us 단위로 걸고 auto_exposure_limit_toggle=1
        을 켜는 쪽이 낫다(AE 의 밝기 적응력을 유지하면서 블러만 묶을 수 있다).
    """
    import pyrealsense2 as rs
    dev = target.get_device() if hasattr(target, "get_device") else target
    out = {}
    opt = getattr(rs.option, "auto_exposure_limit", None)
    for key, getter in (("color", "first_color_sensor"), ("depth", "first_depth_sensor")):
        try:
            s = color_sensor(dev) if key == "color" else getattr(dev, getter)()
            out[key] = bool(opt is not None and s.supports(opt))
        except Exception:
            out[key] = False
    return out


def describe_color_options(target):
    """컬러 센서의 모든 옵션을 값/범위와 함께 훑어 문자열로.

    카메라를 새로 꽂았을 때 제일 먼저 돌려볼 것. 특히 exposure 범위가
    1..10000 으로 나오면 COLOR_EXPOSURE_UNIT_US=100 가맞다는 뜻이다
    (재생 bag 은 범위를 녹화된 한 값으로 뭉개 버려서 여기서 확인이 안 된다).
    """
    import pyrealsense2 as rs
    s = color_sensor(target)
    lines = []
    for name in sorted(a for a in dir(rs.option) if not a.startswith("_")
                       and a not in ("name", "value")):
        opt = getattr(rs.option, name)
        try:
            if not s.supports(opt):
                continue
            v = s.get_option(opt)
        except Exception:
            continue
        try:
            r = s.get_option_range(opt)
            rng = "[%g .. %g] step %g def %g" % (r.min, r.max, r.step, r.default)
        except Exception:
            rng = "(범위 없음)"
        lines.append("  %-28s %-12g %s" % (name, v, rng))
    return "\n".join(lines) if lines else "  (읽을 수 있는 옵션이 없다)"


# ── 자동노출 ROI ─────────────────────────────────────────────────────────────
#
# 상태를 들고 매 프레임 따라다니는 판본은 rs_tuning.ExposureROI 에 이미 있다.
# 여기 있는 것은 **상태 없는 한 방짜리**다 — bbox 하나를 받아 그 자리에 건다.
# 둘 다 필요한 이유: 루프 안에서는 ExposureROI 가 직전 상자를 기억해 같은 값을
# 다시 안 쓰는 게 이득이고(펌웨어 왕복 낭비), 진단/스크립트에서는 그냥 한 번
# 걸어보고 싶을 뿐이라 상태가 짐이 된다.

def ae_roi_supported(target):
    """이 장치의 컬러 센서가 AE ROI 를 받는지.

    소스상으로는 받는다: d400-color.cpp:154-159 의 register_color_features() 가
    FW >= 5.10.9 이고 D405/D401_GMSL 이 아닌 모든 PID 에서 컬러 센서에
    auto_exposure_roi_feature(..., rgb=True) 를 등록한다. rgb=True 가
    깊이용이 아닌 **RGB 전용 펌웨어 명령 SETRGBAEROI** 를 고른다.
    다만 재생(playback) 장치는 무조건 False 를 주므로 bag 으로는 확인이 안 된다.
    """
    try:
        return bool(color_sensor(target).is_roi_sensor())
    except Exception:
        return False


def aim_ae_at_bbox(target, bbox, shape, pad=0.35):
    """자동노출 계측창을 태그 자리에 건다.

    **닭이 먼저냐 달걀이 먼저냐 — 이건 2패스다.**
    태그를 이미 찾았어야 어디에 걸지 알 수 있는데, 노출이 나빠서 태그를 못 찾는
    상황을 고치려고 이 함수를 부른다. 순환이다. 실제로 쓰는 순서는 이렇다:

        1) 기본(가운데 3/4) 상자로 그냥 한 번 검출한다.
        2) 찾으면 그 corners 의 bbox 로 이 함수를 부른다.
           -> 다음 프레임부터 펌웨어가 **태그만 보고** 노출을 정한다.
        3) 접근하면서 태그가 커지면 매번 다시 걸어 준다(ExposureROI 가 이 일을 한다).
        4) 놓치면 바로 되돌리지 말 것. 직전 상자를 몇 프레임 유지하다가
           오래 못 찾으면 center_ae_roi() 로 되돌린다. 매 프레임 전체화면으로
           왕복하면 노출이 펄떡거려 재검출을 오히려 방해한다.

    즉 이건 **첫 검출을 만들어 주는 물건이 아니라, 얻은 검출을 지켜 주는 물건**이다.
    처음부터 완전 역광이라 한 번도 못 찾는 상황이면 이걸로는 못 뚫는다.
    그때는 수동노출을 직접 낮춰 잡거나 태그 쪽 조명을 손봐야 한다.

    왜 그래도 큰가: 도크 태그는 보통 화면의 작은 일부다. 뒤편 출입구로 햇빛이
    들어오면 전체화면 AE 는 그 밝은 배경에 맞춰 노출을 줄이고, 태그의 검은칸과
    흰칸이 서로 붙어 버려 임계화가 둘을 못 가른다. 계측창을 태그에만 걸면
    노출이 태그 기준으로 잡힌다. 역광 도크에서는 이게 검출률을 가른다.

    Args:
        target: profile / device / 컬러 sensor
        bbox: (x0, y0, x1, y1) 픽셀. **현재 컬러 스트림 좌표계**다 —
            펌웨어는 스트림 픽셀을 그대로 받는다(common/stream-model.cpp:331-343).
            해상도를 바꾸면 상자도 같이 바뀌어야 한다.
            detect() 결과에서는 d.corners.min(axis=0) / max(axis=0) 이 그대로 이 값이다.
        shape: 영상 (H, W) 또는 (H, W, C). 화면 밖으로 나가지 않게 자르는 데 쓴다.
        pad: 사방으로 이 비율만큼 넓힌다. 0 이면 태그 안쪽만 보는데, 태그는
            검은칸이 절반이라 노출이 과하게 밝아진다. 0.35 면 주변 배경이 조금
            섞여 평형이 맞는다.

    Returns:
        True 면 실제로 걸었다. False 면 미지원이거나 펌웨어가 거부했다
        (거부는 예외로 안 올라온다 — 다음 프레임에 다시 걸면 그만이라 삼킨다).
    """
    import pyrealsense2 as rs
    try:
        s = color_sensor(target)
        if not s.is_roi_sensor():
            return False
        r = s.as_roi_sensor()
    except Exception:
        return False

    h, w = int(shape[0]), int(shape[1])
    x0, y0, x1, y1 = (float(v) for v in bbox)
    px, py = (x1 - x0) * pad, (y1 - y0) * pad
    x0, y0, x1, y1 = x0 - px, y0 - py, x1 + px, y1 + py

    box = (max(0, int(x0)), max(0, int(y0)),
           min(w - 1, int(round(x1))), min(h - 1, int(round(y1))))
    if box[2] - box[0] < 8 or box[3] - box[1] < 8:
        return False                          # 너무 작으면 펌웨어가 거부한다
    roi = rs.region_of_interest()
    roi.min_x, roi.min_y, roi.max_x, roi.max_y = box
    try:
        r.set_region_of_interest(roi)
    except Exception:
        return False                          # rs.cpp:1793 은 min<=max 만 보고
    return True                               # 실제 거부는 펌웨어가 한다


def center_ae_roi(target, width, height):
    """AE ROI 를 기본 상자(가운데 3/4)로 되돌린다.

    realsense-viewer 의 "reset" 이 쓰는 바로 그 상자다 — 사방으로 크기의 1/8 씩
    떼어낸다(common/stream-model.cpp:331-343). 1920x1080 이면
    (240, 135, 1679, 944). 사실상 이게 기본값이라, 태그를 오래 놓쳤을 때
    여기로 돌아오면 "튜닝 안 한 카메라"와 같은 상태에서 다시 찾기 시작하게 된다.

    화면 전체(0,0,w-1,h-1) 로 되돌리지 않는 이유가 이것이다 — 전체화면은
    기본값이 아니고, 가장자리의 밝은 하늘/조명이 다 섞여 오히려 더 나쁘다.
    """
    mx = int(width * AE_ROI_MARGIN_FRACTION)
    my = int(height * AE_ROI_MARGIN_FRACTION)
    return aim_ae_at_bbox(target, (mx, my, width - 1 - mx, height - 1 - my),
                          (height, width), pad=0.0)


def ae_roi_of(target):
    """지금 걸려 있는 AE ROI 를 (x0, y0, x1, y1) 로. 못 읽으면 None.

    걸었다고 끝이 아니라 **되읽어서 확인**해야 한다. 펌웨어가 상자를 조용히
    물려 놓거나 아예 무시할 수 있는데, set 쪽은 그걸 알려주지 않는다.
    """
    try:
        r = color_sensor(target).as_roi_sensor().get_region_of_interest()
        return (r.min_x, r.min_y, r.max_x, r.max_y)
    except Exception:
        return None


# ── 일부러 뺀 것들 ───────────────────────────────────────────────────────────

#: 나중에 "이것도 넣으면 낫지 않나" 하고 돌아오는 것을 막으려고 남긴다.
#: 전부 근거를 대고 뺀 것이지 빠뜨린 게 아니다.
_WHY_NOT = {
    "sharpness":
        "위험해서 뺐다. 언샤프 마스크는 흑백 경계에 오버슈트 링잉을 얹는데, "
        "AprilTag 는 바로 그 경계에 직선을 맞춰 모서리를 잡는다. 링잉은 밝은 쪽에서 "
        "선을 바깥으로, 어두운 쪽에서 안으로 민다 — 거리에 따라 달라지는 계통오차이고 "
        "그대로 estimate_pose 와 MAX_REPROJ_RMS_PX 로 들어간다. 흐린 영상에서 "
        "검출 '개수'는 늘려 놓고 자세 '정확도'는 망칠 수 있어 도킹에서 최악의 실패 방식이다.",
    "contrast":
        "무의미해서 뺐다. 단조 톤커브인데 AprilTag 의 임계화는 국소 적응형이라 "
        "타일마다 min/max 를 재고 중간에서 자른다. 단조 변환은 그 판정을 안 바꾼다. "
        "할 수 있는 일이라곤 흰칸을 클리핑시키거나 검은칸을 0 으로 눌러 정보를 "
        "없애는 것뿐이다.",
    "gamma":
        "contrast 와 같은 이유. 흑백 두 무리가 0~255 중 어디 앉는지를 바꿀 뿐 "
        "얼마나 잘 갈라지는지를 못 바꾼다. 흑백 타깃이니 감마가 도움이 될 것 같다는 "
        "직관이 강해서 굳이 적어 둔다.",
    "brightness":
        "노출 뒤에 더해지는 DC 오프셋이라 검은칸과 흰칸을 똑같이 민다. "
        "임계화가 보는 것은 둘의 '차이'라 한쪽이 클리핑될 때까지 아무 변화가 없다. "
        "오프셋 말고 신호를 바꾸는 노출이 언제나 낫다.",
    "saturation":
        "색차 전용. 실측으로 상쇄가 확인됐다 — U,V 를 키워도 BGR2GRAY 결과는 "
        "클리핑 안 된 픽셀에서 1 단계 안이다(40048 샘플). 진짜 무의미.",
    "hue":
        "saturation 과 같다. U/V 회전이라 회색에는 안 남는다.",
    "white_balance":
        "AWB 를 잠그는 것(enable_auto_white_balance)은 넣었지만 켈빈 값 자체는 뺐다. "
        "위 상쇄 결과 때문에 범위 안 어떤 값을 골라도 회색 1 단계 안이다. "
        "뷰어에서 이걸 튜닝하는 데 시간을 쓰지 말 것.",
    "backlight_compensation":
        "역광 도크에서 제일 먼저 손이 가는 물건인데 틀린 답이다. 0/1 두 단계뿐인 "
        "문서화 안 된 전체화면 계측 재가중이고, 수동노출로 가면 아예 아무 일도 안 한다. "
        "같은 문제를 aim_ae_at_bbox() 가 정확히, 우리 통제 아래 푼다. "
        "(이 개체에서 픽셀이 실제로 바뀌는지는 못 재봤다 — 카메라 미연결.)",
    "auto_exposure_limit / auto_exposure_limit_toggle":
        "이게 진짜로 원했던 물건이라 특히 분명히 해 둔다: **컬러 센서에는 없다.** "
        "d400-device.cpp:1060-1070 이 get_depth_sensor() 에만, 그것도 "
        "CAP_GLOBAL_SHUTTER 일 때만 등록한다. 주석이 대놓고 "
        "'ae / gain limit feature is not supported on rolling-shutter' 라고 적혀 있고 "
        "D435i 의 RGB 는 롤링셔터다. pyrealsense2 에 열거값이 있다는 것은 "
        "지원한다는 뜻이 아니다(열거값은 장치와 무관하게 늘 있다). "
        "확인은 ae_limit_supported() 한 줄이면 된다. 그래서 수동노출로 간다.",
}
