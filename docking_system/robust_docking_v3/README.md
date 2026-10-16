# robust_docking_v3

VO + 자이로 방향 + 태그 가중 보정. 이전 폴더: `robust_docking_v2`.

공통 설정은 [../configs/common.json](../configs/common.json) 하나다. 이 폴더의 `configs/*.json`은 생성본이며 실행할 때 공통 원본을 읽는다. 알고리즘 코드는 버전별로 유지한다. 이 폴더만 따로 복사하지 말고 `docking_system` 전체를 업데이트한다.

- [버전 대응·구성](../README.md)
- [현장 명령 요약](../run.md)
- [직접 입력·자동 측정 구분](../CONFIG.md)
- [장착각 측정 조건](docs/MOUNT_CALIBRATION.md)

현재 주행 입력·검증은 미완성이며 CAN 없는 측정은 가능하다.
