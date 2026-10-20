# 실행 참고 — 화면·로그 읽는 법과 주행 흐름 (README 보충)

명령은 [README](README.md) 하나만 본다. 여기는 그 명령을 돌렸을 때 **무엇이 보이고, 무엇을 뜻하며, 왜 그렇게 하는지**만 적는다.

## 1. 설치·빌드에서 걸리는 것

- 이 폴더는 다른 `krri` 폴더를 읽지 않는다. 실행에 필요한 것은 이 폴더 + `~/.venvs/krri` + `~/.local/share/krri/ros_ws` + `/opt/ros/jazzy` 뿐이다(2026-10-20: 옛 `krri-v6`·`docking-v6` 폴더는 지웠고 코드도 더는 찾지 않는다).
- 재빌드가 중간에 실패하면 워크스페이스에 "미완료" 표시가 남아 VIO 실행이 막힌다. 표시 파일을 손으로 지우지 말고 재빌드를 끝낸다. 이전 산출물은 `~/.local/share/krri/ros_ws/rebuild_backups/` 에 남는다.
- Jetson 시계가 1970년이면(개발 보드에 시계 배터리가 없어 전원을 끄면 초기화, 인터넷이 있으면 NTP 로 자동 복구) `update_jetson.sh` 가 배포 중에 맥 시간으로 맞춘다(2026-10-20). 배포 없이 맞추려면 **맥 터미널에서** 아래 한 줄(USB 연결만으로 됨). 날짜는 빌드에만 영향을 주고 IMU 시간 검사는 장치 시각을 쓰므로, 날짜를 고쳤다고 IMU 문제가 풀린 것은 아니다.

```bash
ssh -t jetson-wired "sudo date -u -s '$(date -u '+%Y-%m-%d %H:%M:%S')'"
```

- 설치 스크립트는 OpenVINS 원본을 그대로 두고 빌드용 사본의 CMake 만 Jazzy·C++17 용으로 만든다. 추정 알고리즘은 원본과 같고 예제·시각화 실행 파일은 빌드하지 않는다.
- `configs/common.json` 의 주인은 맥이다. `update_jetson.sh` 가 매번 맥 것으로 덮고, 덮기 전에 Jetson 것과 다른 부분을 보여 준다(2026-10-20). 현장 치수는 맥에서 고쳐 배포한다. 장착각 측정 결과는 Jetson 의 `configs/mount.json` 에 따로 있고 배포가 건드리지 않는다(첫 배포 때 옛 common.json 의 `mount` 값을 자동으로 옮긴다).
- 2026-10-16 임시 치수: 차폭 1.3 m, 탑재부 내부 폭 1.4 m(한쪽 여유 5 cm). 정렬 허용은 3 cm 그대로.

## 2. measure 화면 읽는 법

위에서 아래로(2026-10-20 배치):

1. **status(노란 글)**: `상태 / 이유 / 관측 경과`. measure 에선 `MEASURE / / 0.05s` 처럼 이유가 비어 있는 것이 정상(상태기계가 안 돔). shadow·drive 에선 PREPARING → WAIT_ENTER → BASELINE → PLAN → MOVING → SETTLE → COMPLETE, 또는 RECOVER·FAULT 와 그 이유.
2. **measures**: `포크 목표까지 m / 중심선 좌우 m / 방향 ° / 예상 도착 좌우 m`(입구 기준 융합 결과). 태그 창 3장이 모이기 전엔 `위치 기준 수집 중`.
3. **fusion(하늘색)**: OpenVINS 자기 기준 x·y·yaw / 앵커 뒤 이동 거리 · 태그 본 지 몇 초 / 방향 offset(출처) / 융합 상태(`refreshed` 정상, `tag_jump_skipped` 태그가 물리적으로 불가능하게 튀어 건너뜀, `accepted_after_skip` 1초 넘게 계속 어긋나 태그를 채택)와 잔차 / σ / 정지 판정(자이로 °/s, VIO m/s).
4. **카메라(창 폭 전체)**: 보통은 IR 흑백 + OpenVINS 추적점. `--camera-only` 면 컬러 + 태그 사각형(초록=채택, 주황=탈락).
5. **평면도(왼쪽)**: 입구 기준. 파란 상자=지금 추정 차량, 하늘색 궤적=최근 위치, 흰 점=태그 창으로 앵커 갱신, 자홍 고리=`accepted_after_skip`, 초록 고리=첫 앵커, 회색 점선=중심선, 노란 선=계획 후보. **OpenVINS 궤적(오른쪽)**: 자기 원점 기준. 추적이 끊겨 원점이 바뀌면 지워진다.
6. **info**: 관측 JSON 전체(아래 3절의 항목이 그대로 보임).

**손으로 카메라를 들고 하는 실험**(CAN 없음): 태그를 벽에 붙이고 `measure --view` 로 천천히 움직이면 위 2·3번 줄과 평면도가 반응한다. 태그를 손으로 가리면 궤적이 VIO 만으로 이어지고, 다시 보이면 흰 점이 찍히며 보정된다. 처음 2초는 카메라를 가만히 두어야 OpenVINS 가 초기화된다. 실험 전 `configs/common.json` 세 값: `tag_size_m` 은 **검은 테두리 바깥 변**(A4 출력 0.2, 현장 0.3), `tag_height_m` 은 태그 중심 높이, `camera_height_m` 은 손 높이. 장착 보정은 measure 엔 필요 없다.

`--camera-only` 는 위치 추정·주행 준비 판정을 하지 않는다. OpenVINS 초기화가 끝나도 태그가 유효하게 보이기 전엔 입구 기준 pose 가 없다.

## 3. 로그·항목 뜻

`odom`:
- `tracking_ok`: 앞 프레임과 같은 특징점을 계속 추적하는지.
- `corrected_this_frame` / `correction_recent` / `correction_age_s`: 보정 여부와 마지막 보정 뒤 초. `correction_recent=false` 만으로 추적 실패는 아니다.
- `ok`: 추적·시각·지연·IMU·상대 이동 연속성·최대 보정 공백 검사 통과 여부. 실패 이유는 `reason`.
- `usable_for_docking`: 입구 기준 위치까지 있는지. `ok=true` 와 별개.

시간 제한: 카메라 첫 영상 30초, OpenVINS 무응답은 첫 영상 뒤 10초(입력만 계속 와도 갱신되지 않음). 준비 중 영상은 제어에 쓰지 않는다. 오류가 나면 실행 폴더의 `camera_error.txt`, `pipeline_status.json`, `openvins.log`.

`control.jsonl` 의 `status.events` 에는 걸음마다 `step_done` 한 줄(2026-10-20): 명령·목표·정지 명령 때 진행 거리와 그때 본 남은 거리·예측 여유·관측 나이, 가라앉은 뒤 실제 진행 거리와 미끄러짐(`coast`)·출발 지연(`dead_s`)·그때의 학습값. 회전은 rad, 직진은 m(`unit`). 줄자 실측은 `note` 명령으로 `manual_measurement.json` 에 넣는다(README).

`openvins.log` 의 `[pipeline]` 줄(2초마다): 받은 IMU·영상의 장치 시각, 큐 길이, `dropped`(IMU 가 영상 시각을 덮기 전에 0.3초가 지나 버린 영상), `outputs`, `imu_gaps`(장치 시각 공백 > 50 ms), `imu_sequence_jumps`·`imu_lost`(빠진 IMU 표본), `imu_arrival_gap_max_s`·`image_arrival_gap_max_s`(콜백 도착 간격 최대, 2초 창). 정상은 `dropped=0`, 도착 간격 0.1 s 아래.

IMU 정책: 표본 공백·역전은 **세고 기록만**(2026-10-19, v4 규칙). 차가 서는 건 IMU 가 3초 안 올 때뿐(`imu_age_s`, 2026-10-20 1→3 s). 브리지의 `imu` 구독은 **best-effort**(2026-10-20): reliable 수신이 빠진 표본 하나를 되받을 때까지 뒤를 ~1초 붙들어 영상 25~50장을 버리던 것이 VIO 1~2초 멈춤의 원인이었다. 빠진 표본은 5~10 ms 구멍으로 세고 OpenVINS 가 그 위를 적분한다. 누락 구간을 임의로 채우지 않는다. `pipeline_status.json`(1초마다 덮어씀)과 `observations.jsonl` 의 `kind: health` 행에 `gc`·`loop_stage_max_ms`·`delivery_gaps`·`imu_delivery`(콜백 간격 최대)가 남는다.

시작: OpenVINS 에 첫 데이터를 주기 전 IMU 를 0.5초 모아 장치 시각·호스트 시각이 모두 이어지고 IR 좌·우 시각과 맞을 때만 시작한다. 그 사이 시간 점프가 있으면 버퍼를 비우고 다시 모은다(`startup_resets`). 이때 `odom.reason=imu_startup_stabilizing`.

## 4. 현장 입력

`configs/common.json` 의 `value` 를 고친다: 차폭, 탑재부 내부 폭, 좌측 태그 거리(좌측 모드), 태그 크기·높이, 카메라 높이, 카메라→포크 끝 거리. 포크 정지 목표는 `target_front_x_central_m`(기본 −0.5 = 입구 0.5 m 앞) · `target_front_x_left_m`(기본 1.0 = 입구를 지나 1 m), 입구면 기준 안쪽 양수(2026-10-20 코드 상수에서 옮김; 비우면 그 기본값). 입력 뒤 `preflight.sh check --mode left`(또는 `central`) 로 검사한다. 공통값·보정 실패를 임의 숫자로 채우지 않는다.

**장착각**(v4 camyaw 방식, 차를 안 움직임): 차를 태그 정면에 눈대중으로 세우고 30초 이상 가만히 찍은 measure 로그를 `preflight.sh mount --folder … --write` 에 준다. 자이로가 "정지"라고 한 동안의 PnP 태그 회전을 0.5초 블록마다 모아 중앙값에서 **yaw 만** 장착각으로, pitch·roll 은 같은 로그의 IMU 중력 방향으로 잰다(태그 기울기와 구분). 통과 조건은 없다: 정지 중 고유 태그 3장이 없을 때만 계산 불가이고, 30초 미만·블록 흩어짐 1° 초과·앞뒤 절반 차이 0.5° 초과는 `warnings` 에 적고 저장한다. 결과는 컬러에서 재서 raw IR 기준으로 바꿔 `configs/mount.json` 에 들어가고(common.json 은 건드리지 않음), 태그·카메라 치수 서명이 바뀌면 `drive` 가 다시 재라고 거부한다. 카메라를 다시 달면 다시 잰다. 주행 중엔 좋은 직진마다 yaw 를 다듬는다.

## 5. 주행 흐름 (shadow / drive)

1. **준비**: OpenVINS 초기화 + 자이로 정지 5초(`preparation_s`, 전체 180초 안) → `READY`. 정지 판정은 자이로 0.5초 창 p90 ≤ 0.5°/s 와 VIO 속도.
2. **Enter** → 태그가 안 보이면 50°씩 최대 360° 돌며 찾고, 보이면 즉시 멈춘다.
3. **30초 정차**(`baseline_s`): 태그가 보이는 자리에서 σ 기준선을 잰다. 합격·불합격은 없고 기록용. VIO 가 잠깐 끊겨도 기다리고, 태그가 2초 넘게 안 보이면 다시 찾는다.
4. **걸음 반복**: 최대 6단계 후보를 그려 첫 걸음만 실행 → 가는 중엔 관측마다 예측 정지(남은 거리 ≤ 속도 × (관성 + 관측 나이)) → 멈춘 뒤 가라앉기 → 실제 간 거리·미끄러짐으로 속도·출발 지연·관성을 갱신(그 판 안에서만) → 새 위치에서 다시 계획. 장착 yaw 는 좋은 직진(태그 5 m 안·β 10° 안·0.5 m 이상)마다 다시 배우고, 없었어도 들어가되 로그에 남긴다.
5. **진입**: 예상 도착 좌우 ±3 cm 면 눈 감고 진입, 아니면 보정 회전 뒤 다시 잰다(최대 5회). σ 는 기록만.
6. **정지**: `s`+Enter 또는 Ctrl+C. 주행 중 관측·IMU 가 3초 끊기면 중립으로 서고 RECOVER → 돌아오면 재계획(탐색 회전 중에만 FAULT). FAULT 는 이유 한 줄과 로그 폴더를 찍고 끝난다.

shadow 는 명령만 표시하고 차를 안 움직이므로 움직임 피드백이 없어 동작 시간 제한으로 끝난다.

## 6. 그 밖에

- `--profile mapped`(RTAB-Map 지도 기록)는 남아 있지만 쓰지 않는다. 삭제 여부는 보류.
- 코드 시험은 맥에서 README 의 명령으로. Jetson 에서는 `DOCK_PYTHON=...` 로 파이썬을 지정하거나 가상환경에서 `python -m unittest discover -s tests -q`.
