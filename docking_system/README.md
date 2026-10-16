# Docking system

v1~v4의 **공통 설정 원본은 [configs/common.json](configs/common.json) 하나**다. 차량 치수·태그 배치·센서 설정·장착 보정·제어 입력을 여기서 관리한다. 버전별 `configs/*.json`은 읽기용 생성본이며, 실행할 때 공통 원본을 다시 읽는다. 수정 후 실행 중인 측정을 종료하고 다시 시작하면 적용된다.

공통 원본은 표준 JSON이다. 각 항목의 `value`를 수정하고 `description`에서 의미·단위를 확인한다. 상위 묶음에는 `fields`가 있다. 자동 보정값 저장 시 설명이 유지되며, 실행용 생성본과 로그에는 값만 들어간다.

```text
krri/
├── docking_system/
│   ├── configs/common.json       직접 입력 + 공통 장착 보정 + 점검 요약
│   ├── calibration/              점검/장착 보정 보고서
│   ├── preflight.py              공통 검사·장착 보정
│   ├── update_jetson.sh           네 버전과 공통 도구 함께 업데이트
│   ├── run.md                    현장 명령 요약
│   ├── robust_docking_v1/         기존 robust_docking
│   ├── robust_docking_v2/         기존 robust_docking_v1
│   ├── robust_docking_v3/         기존 robust_docking_v2
│   └── robust_docking_v4/         기존 robust_docking_v3
└── work/docking_system/
    ├── robust_docking_v1/         새 v1 측정 로그
    ├── robust_docking_v2/
    ├── robust_docking_v3/
    └── robust_docking_v4/
```

이전 `krri/work` 기록은 이동하거나 삭제하지 않았다. 새 번호의 기록은 `work/docking_system` 아래에 남겨 이전 버전 번호와 섞이지 않는다.

| 새 폴더 | 기존 폴더 | 유지한 추정 구성 |
|---|---|---|
| v1 | robust_docking | 현재 원본: 태그 가중 보정 + 자이로 보조 VO |
| v2 | robust_docking_v1 | VO + 자이로 방향 + 태그 직접 보정 |
| v3 | robust_docking_v2 | VO + 자이로 방향 + 태그 가중 보정 |
| v4 | robust_docking_v3 | 자이로 보조 VO + 자이로 방향 + 태그 가중 보정 |

**현재 v1과 v4의 추정 옵션은 같다.** 원본과 비교 버전을 보존해 이름만 재배치했다. 네 가지 서로 다른 알고리즘을 새로 구현한 변경이 아니며, 완전한 VIO를 도입한 것도 아니다.

공통 설정의 `values`는 모든 버전의 기본값, `sites`는 작업별 태그/공간/목표, `platforms`는 실행 호스트 차이, `measurement`는 측정 전용 옵션이다. `automatic`은 검사 결과이며 구동 승인이나 영구 IMU 영점으로 쓰지 않는다. 입력해야 할 항목은 각 `description`과 [설정 안내](CONFIG.md)에 설명했다.

장착각 측정이 성공하면 공통 `R_body_camera`, 카메라 내부 파라미터와 serial이 갱신된다. 네 버전의 다음 실행에서 같은 값이 적용된다. 실패·중단된 측정은 기존 보정값을 변경하지 않는다. 실행 중 공통 파일을 수정하면 보정 결과 적용을 거부하여 수동 변경을 덮지 않는다.

실주행은 Enter 후 IMU 초기 보정과 출발 전 연속 정지 측정(기본 60초)을 수행한 뒤 태그 확보/주행 절차로 진행한다. 준비 중에는 탐색 회전도 하지 않는다. 진행 시간은 터미널·화면에 표시하고 결과는 각 실행의 `startup_measurement.json`에 남긴다. 공통 `values.startup_measurement.duration_s`로 시간을 바꾼다. 정지 관측 결과를 차량 제동 모델이나 주행 중 오차 상한으로 자동 대입하지 않는다.

주행 오차·제동 범위·현장 공간 등은 여전히 검증/입력이 필요하다. 이 구조 정리는 그 값들을 추정하거나 주행 승인을 부여하지 않는다. 현재 네 버전의 `drive_ready`는 모두 false다. 자세한 명령은 [run.md](run.md)에 있다.

운영 설정에는 실장비·측정·현장 설정만 둔다. 회귀 테스트용 가상 자료는 각 버전의 `tests/fixtures`에 있으며 주행 기본값으로 사용하지 않는다. 정책 생성은 `synthesize --config <실제 설정>`처럼 설정을 명시해야 한다.
