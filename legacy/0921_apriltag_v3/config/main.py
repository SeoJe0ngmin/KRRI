"""설정을 모아 부르는 자리. **여기에는 값을 쓰지 않는다.**

값은 성격별로 나눠 두었다. 고칠 때는 해당 파일을 열면 된다.

    config/detection.py   태그를 보고 위치를 내는 데 쓰는 값
                          태그 크기·번호, 해상도, 검출 품질 문턱, 프레임 수, 장비 규격
    config/control.py     잰 값을 보고 어떻게 움직일지 정하는 값
                          도킹 허용치, 전진 방식, 종료 조건, 회전 문턱, 탐색
    config/imu.py         자이로 자체에 관한 값
                          샘플 주파수, 부호, 보정 시간, 끊김 판정
    config/sim.py         시뮬레이터 장면. **실사용에는 안 쓴다** (여기서 안 부른다)

의존 방향은 한쪽이다:  detection  ->  control
    검출의 흔들림 문턱(STABLE_*)이 도킹 허용치(LAT_TOL_M / HEAD_TOL_DEG)에서
    나오기 때문이다. "측정 흔들림은 맞춰야 할 값보다 한참 작아야 한다."
    따로 박아두면 허용치를 바꿀 때 안 따라와 서로 안 맞는 조합이 된다.

쓰는 쪽은 이 파일 하나만 보면 된다:

    from config.main import LAT_TOL_M, TAG_SIZE_M, IMU_GYRO_HZ

여기 없는 것
    fx / fy / cx / cy   카메라가 직접 준다. 적어두면 해상도 바꿀 때 어긋난다
    FWD_*               직진 시간 모델. 다른 팀 코드라 src/models/control/fwd_time_model.py
    멈출 거리            숫자로 안 정한다. 태그가 화면에서 잘리기 직전까지 간다
    바이어스·회전축      매번 다시 잰다. 온도에 따라 변해서 적어두면 틀린다

**계산으로 나오는 것은 상수로 두지 않는다.** 손으로 박으면 배치가 바뀔 때
안 따라온다 — 실제로 카메라 높이를 1.20 -> 0.50m 로 바꾸니 예전 STOP_M=1.50 이
근접 한계(2.55m)보다 작아져서 매번 태그를 잃는 값이 됐다.
"""
from .control import *      # noqa: F401,F403
from .detection import *    # noqa: F401,F403
from .imu import *          # noqa: F401,F403
