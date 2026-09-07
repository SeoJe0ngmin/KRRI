IMU_GYRO_HZ = 200              # 자이로 샘플 주파수. 이 장치의 유효값은 200/400 뿐
IMU_YAW_SIGN = +1.0            # 자이로 -> 우리 부호규약(+가 반시계).
IMU_BIAS_SEC = 2.0             # 정지 바이어스 측정 시간 [s].
IMU_STALE_SEC = 0.3            # 이만큼 샘플이 안 오면 죽은 것으로 본다.
IMU_DT_GAP_SAMPLES = 10        # 샘플 간격이 이 개수를 넘게 벌어지면 그 구간은 적분하지 않는다.
IMU_CALIB_MIN_RATIO = 0.5      # 보정 샘플이 기대치의 이 비율은 와야 통과
IMU_MOVING_DPS = 0.5           # 보정 중 자이로 표준편차/평균이 이보다 크면 움직인 것.
