# 현장 실행 요약

이 폴더는 기존 `docking_system`과 분리된 새 실행본입니다. 모든 명령은 차량이 멈춘 상태에서 시작합니다. 코드 업데이트 중에는 측정/주행을 종료하세요.

## 1. 맥 → Jetson 코드 업데이트

맥 터미널:

```bash
bash /Users/seojeongmin/krri/docking_systemv2/update_jetson.sh --host jetson-wired
```

USB 주소를 직접 쓰려면 `--host jetson@192.168.55.1`, 공유기라면 `--host jetson@192.168.0.27`입니다. 기본값은 `jetson-wired`입니다.

처음에는 `common.json`도 복사합니다. 다음 업데이트부터 Jetson의 현장 값·장착 보정을 보존합니다. 맥에서 수정한 common까지 덮어쓰려는 경우에만 `--with-common`을 추가합니다. `--dry-run`은 전송 목록만 확인합니다. `krri/work`는 전송·삭제 대상 밖입니다.

## 2. 현장 값 입력 및 카메라 재장착 기준 측정

VS Code에서 Jetson의 `/home/jetson/krri/docking_systemv2/configs/common.json`을 엽니다. 각 `description` 아래 `value`만 수정합니다.

- `vehicle_width_m`: 실제 차폭, `bay_width_m`: 실제 탑재부 내부 폭.
- `left_tag_offset_m`: 입구 중심에서 좌측 태그 중심까지 거리(왼쪽 양수). 중앙 모드는 사용하지 않음.
- 태그 ID·검은 사각형 크기·높이, 카메라 높이, 카메라→포크 끝 거리 확인.
- 카메라 좌우 위치 0, 태그면=입구면 0이 현재 설정입니다.
- `mount`는 자동 작성하므로 직접 수정하지 않습니다.

Jetson 터미널:

```bash
cd /home/jetson/krri/docking_systemv2/robust_docking_v5
bash scripts/preflight.sh check --sensor-only --devices
```

**카메라를 다시 달았으면:** 차량 자체를 태그면에 수직으로 정렬하고, 수평 바닥에서 태그를 똑바로 세운 뒤 고정합니다. 카메라가 비뚤어진 각도와 차량 자체가 비뚤어진 각도는 정지 영상만으로 구분할 수 없으므로 이 기준 정렬은 사람이 해야 합니다. 태그는 여유 있게 전체가 보여야 합니다.

```bash
bash scripts/preflight.sh mount --reference-aligned --view
```

IMU 준비 후 60초를 측정합니다. 성공한 장착각만 공통 파일에 저장하며 CAN은 열지 않습니다. 같은 장착 상태라면 매 run마다 mount를 다시 할 필요는 없습니다. 실행 중 실패하면 오류 이유와 출력 폴더의 `mount_report.json`을 확인합니다.

## 3. 카메라·태그·VO·IMU만 보기

```bash
bash scripts/run_nano.sh measure --seconds 60 --view
```

CAN을 열지 않습니다. IMU만 원인 진단하려면:

```bash
/home/jetson/.venvs/krri/bin/python -m docking.imu_diagnostic --seconds 20
```

영상 부하를 분리해 비교하려면 `measure --seconds 30 --no-tag --no-odom`을 사용합니다. 이 옵션은 실주행에는 사용할 수 없습니다.

화면은 VS Code **포트** 탭에서 `8765`를 전달한 뒤 맥 브라우저의 `http://127.0.0.1:8765`로 엽니다. 또는 맥에서 `ssh -N -L 8765:127.0.0.1:8765 jetson-wired`를 별도 터미널에 유지합니다. RDP는 필요 없습니다.

## 4. 실주행

중앙 태그 — **포크 앞끝이 입구면 0.5m 전 정지**:

```bash
bash scripts/preflight.sh check --mode central
bash scripts/run_nano.sh drive --mode central --execute --view
```

좌측 태그 — **입구 중심선으로 접근하고 포크 앞끝이 입구면을 지나 1m에서 정지**:

```bash
bash scripts/preflight.sh check --mode left
bash scripts/run_nano.sh drive --mode left --execute --view
```

시작하면 중립 상태에서 IMU 초기 보정과 정지 측정을 합니다. **READY 이후 Enter**를 눌러야 움직입니다. 준비 중 눌렀던 Enter는 저장하지 않습니다. 준비 후 차량을 움직이면 준비를 다시 합니다.

**정지: `Ctrl+C` 또는 `s`+Enter.** SSH 입력 종료, 센서 지연·추적 실패, CAN 송신 감시 오류도 중립 처리합니다. 소프트웨어 정지로 실제 제동거리나 전원·통신 단절 시 정지를 보장할 수는 없습니다. 차량의 물리적 정지 수단을 사용 가능한 상태로 두세요.

CAN 없는 같은 상태 전환을 보려면 `drive --execute` 대신 `shadow`를 씁니다. 차량이 실제로 움직이지 않으면 계획한 동작을 관측으로 확인할 수 없어 시간 제한으로 멈추는 것이 정상입니다.

## 5. 자동으로 하는 것 / 사람이 정하는 것

| 구분 | 내용 |
|---|---|
| 사람 입력 | 현장 치수·태그 위치·장착 기준 정렬 |
| mount 검사 작성 | 카메라 장착 방향, 실제 카메라 내부 파라미터·시리얼 |
| 각 run 출발 전 | IMU bias·중력 방향, 연속 정지 관측, 태그가 보이면 위치 기준 확보 |
| 각 run 진행 중 | 실제 VO·IMU 이동량, 태그 보정, 정지 후 잔여 이동, 응답 시간·잔여 이동의 계획용 추정 |
| 과거 로그 | 다음 run의 동작이나 학습값으로 자동 재사용하지 않음 |

운영 제한: 출발 후 600초, 정지 확인 20초, 추적 회복 60초, 출발 전 준비 최대 180초. 계획 깊이 6, 1회 전진 최대 0.8m·후진 0.3m·일반 회전 15°. 첫 동작은 더 짧게 확인합니다. 태그 탐색은 왼쪽 60°씩 누적 360° 한도입니다. **계획 깊이 6은 총 주행거리 4.8m 제한이 아닙니다.**

화면의 `pose`는 입구 기준 카메라 위치의 지면 투영 기준점 `[x, y, 방향rad]`입니다. x=0이 입구면, 안쪽 양수, y는 왼쪽 양수입니다. 초록 목표점은 포크 앞끝 목표입니다. `not_scheduled`는 이번 영상에서 태그 검사를 쉬었다는 뜻입니다. 카메라 30fps와 태그 검사 약 6fps, 화면 5fps는 서로 다릅니다.

출력은 `/home/jetson/krri/work/docking_systemv2/robust_docking_v5/`에 저장합니다. `metrics.json`의 FPS·처리 지연·버려진 영상 수, `observations.jsonl`의 위치/신뢰 상태, `control.jsonl`의 명령·CAN 진단, `session.json`의 종료 이유를 확인합니다. 파일명은 날짜가 잘못되어도 임의 식별자를 붙여 중복을 피합니다.

## 구현상 한계

- 구현은 **IR-depth VO + 짧은 구간 gyro 보조 + 태그 위치 보정**입니다. IMU 가속도를 두 번 적분하는 관성 위치 추정이나 완전한 최적화 기반 VIO는 아닙니다.
- 태그가 안 보이고 VO도 끊기면 위치를 모르는 상태로 정지합니다. 명령 거리나 과거 평균 속도로 위치를 만들어 계속 진입하지 않습니다.
- ±3cm는 **추정된 도착 위치의 판정 기준**입니다. 단일 태그의 계통 오차, 깊이 오차, 진동, 가림에 대한 실제 정확도 보증은 아닙니다. 표시되는 오차 반경도 운영용 추정치입니다.
- 차량 회전 중심과 속도·잔여 이동 초깃값은 이전 코드 참고값입니다. 경로 후보/제동 예상에만 사용하며 실제 위치를 대체하지 않습니다. 회전 방향 불일치·목표 초과·관측 불일치가 감지되면 정지합니다.
- 현장 치수와 새 장착 기준은 아직 채우지 않았습니다. 로컬 자동 테스트는 실제 Jetson·카메라·차량 CAN 실험을 대신하지 않습니다.
