from .control import HEAD_TOL_DEG, LAT_TOL_M   # 흔들림 문턱을 도킹 허용치에서 뽑는다

TAG_SIZE_M = 0.300             # 태그 한 변 [m], 검은 테두리 바깥까지  ★인쇄물 실측
TAG_ID = 1                     # 멀리서 접근할 때 보는 태그
COLOR_SIZE = (1920, 1080)      # D435i 컬러 최대. 프레임당 12.9ms
IR_SIZE = (1280, 720)          # IR 은 fx 가 컬러의 1/3 이라 거리도 1/3

MIN_TAG_PX = 20.0              # 태그 한 변이 이보다 작으면 자세를 안 믿는다
STABLE_TAG_PX = 30.0           # 이 이상이면 안정적                    △실장비 검증 전
MAX_REPROJ_RMS_PX = 2.0        # 재투영 오차 상한 [px]
MIN_DECISION_MARGIN = 20.0     # 저조도 하한 (거리에는 둔감하다)
RELIABLE_TILT_DEG = 10.0       # 옛 각도 신뢰 판정. 지금은 폴백 전용
DEFAULT_QUAD_BLUR = 0.0        # 검출 전 블러. 안 넣는 게 최선이었다

DEPTH_TOL_COEF = 0.05          # depth 대조 허용치 = max(FLOOR, COEF x z^2)
DEPTH_TOL_FLOOR_M = 0.02       # 그 하한 [m]
DEPTH_CHECK_MAX_Z = 1.5        # 이 거리 넘으면 판정이 무의미 [m]

CORNER_NOISE_PX = 0.07         # 코너 검출 잡음 [px]                   △거리마다 다름
SIGMA_SAMPLES = 60             # 그 예측에 쓰는 몬테카를로 표본 수

MEASURE_FRAMES = 30            # 모으는 프레임 수 (흔들림 1/5.5)
MEASURE_MAX_FRAMES = MEASURE_FRAMES * 5   # 이만큼 봐도 못 모으면 포기

STABLE_LATERAL_M = LAT_TOL_M / 3.0          # 표준오차가 이보다 크면 명령 안 냄
STABLE_HEADING_DEG = HEAD_TOL_DEG / 3.0     # 위와 같음 [도]
MAX_HEADING_SIGMA_DEG = HEAD_TOL_DEG / 4.0  # 예측 흔들림이 이보다 크면 각도 불신
STABLE_SPREAD_K = 3.0          # 관측이 예측의 이 배를 넘으면 모르는 일이 있는 것

CAM_YAW_OFFSET_DEG = 0.0       # 광축이 지게차 정면과 어긋난 각 [도]   ★첫 도킹 후

BLUR_CLEAN_PX = 10.0           # 여기까지 검출 100%
BLUR_DEAD_PX = 32.0            # 여기서 0%
COLOR_EXPOSURE_UNIT_US = 100.0 # 컬러 노출 눈금 (UVC 규격). 83 = 8.3ms
TAG_CELLS = 8                  # tag36h11 한 변의 칸 수 (테두리 포함)
MM_PER_INCH = 25.4
D435I_COLOR_REF = (1920, 1080, 1359.2, 1359.0, 956.9, 571.3)   # w,h,fx,fy,cx,cy

