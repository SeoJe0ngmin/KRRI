> 입력 위치 변경: 이제 `docking_system/configs/common.json`에서 한 번만 수정한다. 아래 설정 경로는 기본적으로 `values` 아래이며 작업별 값은 `sites` 아래다. [새 입력 안내](../../CONFIG.md).

# 실차 설정과 첫 측정

## 초기 IMU 보정의 공통 판정

2026-10-15 측정에서 사용한 창 평균 판정을 `nano_measure.json`/`vm_measure.json`과 `hardware.json`/`hardware_vm.json`에 공통 적용했다. 실제 차량 진동에서의 보정 성공과 오차 범위는 아직 확인하지 않았다. 과거 로그를 실행 시 읽거나 보정값을 영구 저장하지 않고, 사용자가 고정한 상태에서 매 실행 새로 10초를 수집한다.

| 설정 | 측정 프로필 값 | 용도 |
|---|---:|---|
| `duration_s` / `timeout_s` | 10 / 60초 | 필요한 연속 수집 시간 / 전체 대기 제한 |
| `motion_window_s` | 0.1초 | 장치 시각 기준 최근 구간의 자이로 벡터 평균 |
| `moving_dps` | 0.5°/s | 기존 bias를 뺀 구간 평균의 크기 제한 |
| `max_gyro_peak_dps` | 5°/s | 원시 순간값 크기 제한; 넘으면 즉시 초기화 |
| `max_gyro_std_dps` | 0.3°/s | 수집한 원시 자이로의 축별 표준편차 벡터 크기 제한 |
| `max_gyro_window_std_dps` | 0.15°/s | 구간 평균의 표준편차 벡터 크기 제한; 느린 변화 검사 |

기존 판정은 원시값 하나가 0.5°/s를 넘으면 전부 초기화했다. 두 번째 고정 측정에서 3,996개 중 156개가 넘었고 가장 긴 통과 구간은 1.18초였다. 첫 10초의 원시 표준편차 벡터 크기는 약 0.267°/s로 기존 0.15°/s도 초과했다. 새 0.3°/s와 순간값 5°/s는 벤치 측정을 위한 시험값이며 제조사 보증 잡음이나 주행 오차 상한으로 해석하지 않는다. 단일 기록에 맞춘 기준이 일반 장착·온도·진동에서도 유효한지는 확인해야 한다.

원시 표본을 자르거나 제외해 분산을 작게 만들지 않는다. 영점은 통과 구간의 모든 원시 자이로 표본 평균이며, 운전 중 자이로 적분은 기존 원시 표본을 계속 사용한다. 가속도, 장치 시각 역행, 누락, 표본 수, 중력 방향 검사는 유지된다. `motion_window_s`가 없는 기존 프로필은 원래의 순간값 판정을 사용한다.

두 번째 기록을 다시 입력한 검사에서는 10.000초 후 READY, 첫 번째 큰 진동 기록은 미완료였다. 이는 저장 기록에 대한 검증이며 실기 측정·차량 정지 확인을 대신하지 않는다. IMU만으로 작은 일정 회전과 bias, 일정 병진과 정지를 구별할 수 없으므로 실제 고정이 필요하다. 실차 적용 전 별도 고정 측정과 실제 움직임 거부를 확인해야 하며, `bounds.gyro_rate_error_rps`나 `verified` 값을 이 결과로 채우지 않는다.

## 2026-10-12 수동 입력·검증 목록

아래는 현재 Nano/VM 실차 프로필에 필요한 입력을 묶은 목록이다. 센서만 확인하는 `measure`와 송신 없는 `check-devices`는 이 실차 값이 완성되기 전에도 실행할 수 있다. 주행 로그에서 평균 속도·관성을 학습하는 기능은 없지만, 적용할 하중·바닥·조명·속도 조건 안에서 측정 오차와 이동·제동 범위를 확인해야 한다. 실험에서 관측한 최대값만으로 모든 환경의 절대 상한을 인증할 수는 없다.

태그 높이 1.58 m, 카메라 높이 0.57 m, 카메라에서 앞끝까지 1.50 m, 태그 ID 1/변 길이 0.30 m는 이미 입력돼 있다. 같은 장착·태그라면 다시 입력하지 않는다. `null`이라고 모두 미입력 오류는 아니다. 자동 노출의 `exposure_us`/`gain`, 단일 카메라의 `serial`, 자동 산출을 사용하는 `prealignment.trigger_lateral_m` 등은 비어 있어도 된다.

2026-10-15 사용자 제공 정보 반영: 카메라 뒤 CAN박스까지 약 0.5m, 계산용 뒤끝 1.0m. 실제 차폭은 약 1.3m로 추정하고 1.5m 이하라고 하여 계산용 전체 폭은 1.5m(반폭 0.75m)다. 둘 다 실측 인증값이 아닌 사용자 제공 보수적 범위다. 탑재부 폭과 실제 측면 여유는 아직 모른다. 좌우 목표 허용 오차 ±0.03m와 실제 간격을 혼동하지 않는다.

| 구분 | 직접 제공하거나 실험으로 확인할 값 | 설정 위치 |
|---|---|---|
| 차량·카메라 장착 | 기준점에서 뒤끝까지 거리, 전체 차폭/2, 카메라 좌우 위치와 장착 방향 | `body.rear_m`, `body.half_width_m`, `sensor.calibration.camera_lateral_m`, `R_body_camera` |
| 태그 설치 | 입구 중앙 기준 태그 앞뒤·좌우 위치, 태그 면의 방향 | `sensor.calibration.tag_position_xy_m`, `R_dock_tag` |
| 현장 공간 | 실제 장애물 없는 영역과 추가 여유, 최초 회전할 수 있는 반경 | `scene.free_rectangles`, `scene.clearance_m`, `startup_search.free_radius_m` |
| 시험 목표 | 좌우·방향 허용 오차, 콘 구간의 최종 진입 깊이 | `trial.lateral_tolerance_m`, `yaw_tolerance_rad`, `insertion_front_m`; 벽 앞 0.5 m/앞뒤 허용 0.1 m는 기존값 확인 |
| 사이드스텝 공간·정렬 | 통로 반폭, 입구 밖 정렬 구간, 허용 좌우/방향 오차 | `prealignment.corridor_half_width_m`, `staging_x_range_m`, `alignment_y_m`, `alignment_yaw_rad` |
| 영상 위치 오차 | 기준 거리/각도 대비 태그 위치·방향 오차, 배경 추적의 거리·회전·프레임별 누적 오차 | `sensor.bounds.tag_pose_error`, `odom_error_per_m`, `odom_error_per_rad`, `odom_error_per_frame`, `relative_translation_error_m` |
| IMU·물리 운동 | 좌우 회전 부호, 회전 스케일, 잔여 자이로 오차, 최대 가속도·속도·각속도 | `sensor.imu.yaw_sign_verified`, `gyro_scale`, `sensor.bounds.gyro_rate_error_rps`, `max_acceleration_mps2`, `limits.max_speed_mps`, `max_yaw_rate_rps` |
| 영상 정지 확인 | 기준 영상과 현재 영상 사이 직접 이동/회전 오차, 각가속도 상한과 정지 문턱 | `sensor.stop_window.translation_error_m`, `yaw_error_rad`, `max_angular_acceleration_rps2`, `limits.stop_speed_mps`, `stop_yaw_rate_rps`, `stop_dwell_s` |
| 전진·후진·좌우 회전 | 후보 이동량/각도, 최소 실행량, 제동 여유, 출발 지연, 동작/STOP 시간, 종료점·이동 중 오차 | 각 `actions[]`의 `target`, `min_target`, `brake_reserve`, `max_start_delay_s`, `max_duration_s`, `max_stop_response_s`, `terminal_error`, `sweep_error` |
| 호스트·USB·CAN 시간 | 촬영→콜백 지연, 영상/IMU 만료, 제어 갱신 간격, 정지 확인 예산, CAN 전송 알림·쓰기 간격 | `sensor.bounds.capture_to_callback_max_s`, `sensor.imu.max_age_s`, `limits.max_age_s`, `lease_s`, `settle_timeout_s`, `can.diagnostics.max_ack_age_s`, `max_write_gap_s` |
| 최초 탐색 회전 | 회전 출발/STOP 지연, 잔류 회전량, 회전 중 병진 편차 | `startup_search.max_start_delay_s`, `max_stop_response_s`, `brake_reserve_rad`, `max_center_drift_m`, `stop_drift_reserve_m` |
| 추적 일시 소실 | 관측이 없을 때의 최대 병진 속도와 허용 예측 구간 | `sensor.tracking_recovery.max_unobserved_speed_mps`; `max_bridge_s`, `hold_keyframe_s` 등 초기값도 조건에 맞게 확인 |
| 배경 소실 중 복귀 | STOP 후 정지 범위에 들어오는 시간·잔류 속도, 복귀 허용 방향 오차 | `recovery.blind_return.stop_rest_after_s`, `rest_speed_upper_mps`, `max_heading_error_rad` |
| 정지 중 자이로 재보정 | 영상 정지 속도·각속도 기준, 상대 회전 오차, 허용 바이어스 변경량 | `sensor.imu.stationary_recalibration.max_speed_mps`, `max_visual_yaw_rate_rps`, `relative_yaw_error_rad`, `max_bias_change_rps` |
| 가시성·추적 가능 구역 | 투영 오차 여유, 태그가 없을 때 실제로 배경/깊이가 추적되는 위치·방향·조명 범위 | `visibility.projection_margin_px`, `odometry_pose_regions` 또는 `odometry_regions`, `odometry_evidence` |
| 계획·운용 정책 | 계획 공간/격자 해상도, 계획용 관측 오차 예산, 재시도·시간/동작 수 제한 | `grid`, `observation_error`, `recovery`, `startup_search.improvement`, `limits.run_timeout_s/max_steps` 등. 실측 상한과 구분해 설계하고 검증 |

`min_tag_px`, `edge_px`, 재투영 오차·검출 margin, 추적 inlier/거리 문턱, 자동 노출·흐림 거부 문턱은 초기값이 있으며 빈 필수 측정값은 아니다. 해상도·야외 조명·동작 속도가 바뀌면 수락률과 정확도를 함께 확인한다. `operational.min_rotate_rad` 등 기존 참고값도 새 명령/장비 조건에서 자동 검증되는 값은 아니다. CAN 채널·비트레이트는 연결 장비와 맞춰야 하며 기존 주행 바이트와 강도는 유지한다.

자동으로 얻는 값은 매 실행의 시작 자이로 bias/중력 방향, 실시간 태그/상대 이동 관측, SDK 카메라 내부 파라미터·시각·노출 정보와 처리율이다. 조건이 검증된 뒤 정지 중 bias 재보정도 가능하다. 이 값들을 `configs/*.json`에 자동 반영하지는 않는다. `preflight.py mount`가 성공하면 `sensor.calibration.camera_model`과 장착 회전을 공통 파일에 자동 적용한다. 입력한 회전/위치와 기존 높이에서 `T_body_camera`/`T_dock_tag`를 구성하고, `prepare`가 `trial`에서 `goal`을 계산하므로 이 행렬과 목표 상자를 따로 손계산해 중복 입력할 필요는 없다.

`validation.*` 근거와 각 기능의 `verified`, `validation.approved`는 자동 추정 결과가 아니다. 해당 운용 조건에서 확인을 마친 후 설정하는 승인 표시다. Nano와 VM의 물리 치수는 같아도 USB·스케줄링·관측/제어 지연은 따로 확인하며, VM은 `deployment.timing_verified_on_this_host`도 필요하다. 다음 명령은 새 정지 창·추적 복구·무관측 복귀·CAN 진단까지 포함한 미입력 값과 미검증 표시를 나열하며 장비를 열지 않는다.

```bash
python3 -m docking check-config --config configs/hardware.json
python3 -m docking check-config --config configs/hardware_vm.json
```

## 바로 측정할 수 있는 것

`nano_measure.json`에는 센서 처리와 표시 설정이 들어 있다. 카메라/IMU 처리가 실제로 돌아가야 Nano의 FPS, 단계별 시간, 누락량을 알 수 있으므로 위치 보정이 아직 없어도 실행된다. CAN은 열지 않는다.

기본 태그 ID 1, 검출용 태그 변 길이 0.3 m는 v5 설정에서 확인한 값이다. 실제 사용하는 태그와 맞춰야 한다. 색영상/IR 선택과 해상도, 카메라 USB/전원 상태에 따라 결과가 달라지므로 기록의 `config_used.json`을 함께 보관한다. IR projector를 끈 기본값은 투사 무늬가 고정 배경 특징으로 추적되는 문제를 피하려는 초기 설정이다. 자연 무늬가 적으면 수동 stereo 깊이도 부족할 수 있으므로 이것만으로 추적 가능성을 보장하지 않는다.

2026-10-11 선택 기준은 “깊이를 쓰는 현재 방식에서는 IR 유지, 깊이를 쓰지 않는 방식으로 바꾸면 컬러 사용”이다. 현재 `odom.enabled=true`이며 `SparseOdom.seed()`가 특징점 주변의 깊이를 읽어 3차원 점을 만들고, `update()`가 다음 영상과의 대응으로 상대 이동량을 계산한다. 따라서 현재 Nano/VM 측정·주행 프로필의 `camera.stream="infrared"`를 유지한다. 태그 자체의 PnP에는 깊이를 사용하지 않는다. 깊이를 이용한 주변 장애물 지도 작성이나 충돌 검출은 구현하지 않았다.

이는 이번 프로젝트의 선택이며 깊이 사용의 기술적 필수 조건은 아니다. RealSense는 [공식 컬러 정렬 예제](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py)처럼 컬러+깊이도 지원한다. 코드에도 그 경로가 있지만 현재 두 스트림에 같은 폭·높이를 요청한다. `camera.depth_enabled=false`이면 깊이 수신·정렬을 하지 않는다. 측정 명령의 `--no-depth`는 컬러로 전환하고 odometry와 IR용 장착/내부 보정을 비활성화한다. `--no-odom`만 주면 깊이 수신·정렬은 유지하여 병목을 따로 비교할 수 있다. 실차 제어는 현재 깊이 기반 상대 이동 추적을 필수로 요구한다.

기존 v5 주행 영상은 컬러 1920×1080이다. 새 Nano/VM 측정용 기본값은 IR 640×480으로 변경했으며, 두 설정의 인식 거리/정확도가 같다는 의미는 아니다. 태그 중심 높이 1.58 m, 카메라 높이 0.57 m는 v5 `config/sites/kw.py`, 카메라에서 접힌 포크를 포함한 앞끝까지 1.50 m는 `config/control.py`에서 가져왔다. 사용자가 2026-10-11에 장착 조건이 같다고 확인했으므로 네 실장비 프로필의 `rig`에 복원했다. 앞끝 거리는 차체 형상에 연결하고, 높이는 아래 변환 구성에 사용한다. 높이만으로 미확인 장착 방향까지 정하지는 않는다.

처음에는 차량을 움직이지 않고 화면·태그·깊이 특징점 수·IMU 누락부터 확인한다. 이후 사람이 기존 장비 절차로 움직이는 동안 측정 모드를 사용하면, 새 코드가 차량에 명령하지 않으면서 태그 소실 전후의 추적 가능성을 확인할 수 있다. 사람의 실제 정지 위치/거리 기준이 없으면 이 로그만으로 절대 정확도를 알 수 없다.

## 노출 설정과 확인

Nano/VM 모두 `camera.auto_exposure=true`, `exposure_us=null`, `gain=null`로 시작한다. v5의 현재 자동 노출 사용 의도를 유지하되, 현재 선택된 IR 센서에 명시적으로 적용한다. D435i의 IR/depth와 RGB는 별도 노출 제어다. IR 선택 시 stereo 센서만, color 선택 시 RGB 센서만 설정한다. 컬러+깊이 경로에서는 이 설정으로 depth 센서 노출까지 바뀌지는 않는다.

| camera 필드 | 동작 |
|---|---|
| `auto_exposure` | 필수 bool. 자동이면 노출·gain을 직접 쓰지 않음 |
| `exposure_us` | 수동일 때 필요한 양의 노출 시간, **µs**. 예: 8300은 8.3 ms이며 권장 확정값은 아님 |
| `gain` | 수동일 때 선택적으로 지정하는 SDK gain. null이면 현재 값을 유지하고 읽어서 기록 |
| `auto_exposure_priority` | 기본 null: 건드리지 않음. 지원 센서에서 false를 지정하면 AE의 FPS 가변 옵션을 끔. 30 FPS 수신/처리 보장은 아님 |

**측정 환경은 야외다. 자동 노출을 기본으로 한다.** 8.3 ms 수동값을 기본으로 넣지 않았다. 조명에 맞춰 정해야 하며, 과거 실외에서 8.3 ms 고정 노출로 영상이 포화됐던 이력이 있으므로 이 값을 실외 기본값으로 쓰지 않는다. 짧은 노출은 이동 흐림을 줄이지만 어두워지거나 gain 잡음이 커질 수 있다. 정지 화면만으로 주행 노출을 확정하지 않는다.

D435/D435i에서는 RGB 노출 option이 100 µs 단위, IR option이 1 µs 단위라 코드가 변환한다. 다른 모델은 같은 단위라고 가정하지 않는다. 요청한 모든 제어의 지원 여부·범위·step을 먼저 검사하며 임의로 clamp하지 않는다. 수동 모드는 AE를 끈 뒤 exposure/gain을 쓴다. 설정값을 다시 읽어 불일치가 있으면 오류로 종료한다.

- `camera_settings.json`: 시작 시 요청값, SDK 적용값, 단위, 옵션 범위. **자동 노출에서 이때 읽은 exposure는 이후 프레임의 노출값이 아니다.**
- `observations.jsonl`과 원격 화면의 `camera.frame_metadata`: 원본 선택 영상에서 읽은 프레임 노출·gain·AE 정보. SDK가 제공하지 않으면 null. 메타데이터 노출은 [SDK 정의](https://github.com/realsenseai/librealsense/blob/master/include/librealsense2/h/rs_frame.h)에 따른 µs이며 RGB option용 100배 변환을 다시 적용하지 않는다.

매 프레임 센서 option을 USB로 조회하거나 노출을 재설정하지 않는다. 프레임 메타데이터만 읽는다. 자동 노출 수렴, metadata 제공 여부, 실제 FPS와 태그 검출률은 카메라 연결 후 확인해야 한다. 센서별 제어가 독립적인 근거는 [RealSense HDR 설명](https://dev.realsenseai.com/docs/high-dynamic-range-with-stereoscopic-depth-cameras/)을 참고한다.

## 좌표와 장착 보정

`T_A_B`는 B 좌표의 점을 A 좌표로 바꾸는 4×4 동차 변환이다.

실장비 프로필의 차량 기준점은 **카메라와 앞뒤 위치가 같은 차량 중심선상의 바닥 높이 점**이다(`rig.body_reference=ground_centerline_at_camera_station`). 차체에 고정된 계산 기준이며 실제 회전축으로 가정하지 않는다. 태그/카메라 높이는 같은 바닥 기준으로 해석한다. 이 정의에서 `body.front_m=1.50`이고 카메라의 차체 기준 이동 성분은 `[0, camera_lateral_m, 0.57]`이다. 카메라 광학 원점과 줄자 기준의 차이, IR/컬러 간 차이는 장착 보정에서 확인한다.

`configuration.load_config()`가 합성·실행·재생에 동일하게 적용된다. `R_body_camera`와 `camera_lateral_m`가 입력되면 알려진 높이로 `T_body_camera`를 만들고, `R_dock_tag`와 `tag_position_xy_m`가 입력되면 태그 높이로 `T_dock_tag`를 만든다. 이미 전체 행렬을 입력했다면 알려진 치수와 일치하는지 검사한다. 미입력 회전/좌우 위치를 0 또는 단위행렬로 가정하지 않으며, 후방 길이와 차폭도 별도 실측이 필요하다.

- `T_body_camera`: 현재 선택한 영상 카메라 좌표 → 차량 기준점 좌표. 차량 x 전방, y 왼쪽, z 위. 카메라 x 오른쪽, y 아래, z 광축 전방. 카메라 장착 위치·yaw/pitch/roll을 반영한다. IR과 컬러는 같은 외부 파라미터를 무조건 공유하면 안 된다.
- `T_dock_tag`: 태그 좌표 → 탑재부 좌표. 탑재부 x 진입 방향, y 왼쪽, z 위. 태그 원점은 중심이며 로컬 꼭짓점 순서는 OpenCV IPPE square 규약이다. 태그가 입구 중심에 있다는 가정을 자동으로 넣지 않는다.
- IMU `yaw_axis`: 자이로 센서 좌표에서 차량 회전축에 해당하는 단위벡터. 기본 `[0,-1,0]`은 측정용 가정이다. 기울어진 장착/gyro-frame extrinsic과 부호를 검증해야 한다.
- `gyro_bias_rps`, `gyro_scale`: 해당 장착/온도/센서에 맞춘 값. 프로그램이 움직이는 동안의 평균을 bias로 자동 학습하지 않는다.

변환 관계는 다음과 같다.

`T_dock_body = T_dock_tag × inverse(T_camera_tag) × inverse(T_body_camera)`

장착을 바꾸면 변환·오차 상한을 다시 확인해야 한다. `calibration.verified=true`는 계산으로 자동 인증되는 값이 아니라 설정 책임자가 보정 근거를 확보했다는 선언이다.

실제 태그는 입구 중앙이 아닌 좌/우측에 설치할 예정이다. 탑재부 원점을 입구 중앙 바닥에 놓으면, 태그의 입구 앞뒤 위치와 **부호 있는 좌우 거리**를 `tag_position_xy_m`에 넣는다. y는 진입 방향을 바라보는 차량 기준 왼쪽이 양수다. 태그 면이 입구 면과 평행하면 태그에서 얻은 진입 방향과 평행하고 입구 중앙을 지나는 선이 목표 진입선이다. 거리만 알아서는 방향을 결정할 수 없으며, 비스듬히 부착하면 `R_dock_tag`에 그 회전도 넣어야 한다.

태그 앞 0.50 m 정지 시험은 차량 앞끝 기준이다. 태그 면과 차량이 정렬됐고 태그 면을 dock x=0으로 정하면 기준점 목표 x는 `-0.50-1.50=-2.00 m`다. 이는 전방 성분이며 태그 중심까지의 3차원 직선거리와 다르다. 콘 시험은 통로에 들어간 뒤 더 전진해서 정지하는 목표이며, 정지 깊이와 뒤끝 길이는 아직 정하지 않았다. `prepare --task wall_stop|cone_entry`로 두 목표를 분리한 설정을 만든다. `configs/sites/kw.json`, `krri.json`은 현장 입력 틀이고 Nano/VM의 시간 설정을 덮어쓰지 않는다. 콘 목표는 yaw 허용 범위의 뒤쪽 모서리까지 입구 안에 있는지 검사한다. 미입력 허용 오차/진입 깊이는 임의로 채우지 않는다.

## 차체와 현장

| 필드 | 의미 |
|---|---|
| `body.front_m`, `rear_m`, `half_width_m` | 기준점에서 차체 전체(접힌 포크 포함)의 앞·뒤·반폭 |
| `scene.free_rectangles` | `[xmin,xmax,ymin,ymax]` 형식의 확실한 자유공간. 벽/기둥/외부 돌출물을 포함시키면 안 됨 |
| `scene.clearance_m` | 차체 외 추가 여유. 차체 치수 오차가 있으면 여기에 포함 |
| `goal.lo`, `goal.hi` | 최종 기준점 x/y/yaw의 허용 구간. 전체 진입 여부는 차체 앞뒤 길이로 확인 |
| `grid.lo`, `step`, `count` | 관측 격자 시작/해상도/개수. yaw는 ±π wrap을 가로지르지 않는 지역 범위로 설정 |
| `observation_error` | 정지 후 계획에 사용할 x/y/yaw 오차 예산. 오래된 촬영 시각으로부터의 전파 오차도 포함되어야 함 |

목표 좌우 허용 오차와 실제 통로 여유는 서로 다르다. 실제 차체 반폭과 통로 폭을 넣고 다시 합성하면 도달 가능한 노드가 없을 수 있다. 이 경우 격자를 무작정 굵게 하거나 오차 숫자를 작게 써서 주행을 열면 안 된다.

## 동작 범위

각 실차 동작에는 `kind`, `command`, `target`, `min_target`, `brake_reserve`, `max_start_delay_s`, `max_duration_s`, `max_stop_response_s`, `terminal_error`, `sweep_error`가 필요하다. 공통 설정의 `values.actions`에 작성한다. 양의 translate는 `forward_slow`, 음수는 `backward`; 양의 rotate는 `rotate_left_slow`, 음수는 `rotate_right_slow`다. 모든 각도는 rad다.

전진/후진의 오차 배열은 **진행방향 m, 횡방향 m, heading rad**다. 회전 오차는 **dock x m, dock y m, heading rad**다. 단위가 다른 항목을 같은 수로 비교하지 않는다.

이 범위는 특정 명령 강도에서 실제로 실행할 폐루프 정지 동작 전체에 관한 계약이다. 시작 잔류 속도, 관측 지연, 명령 반영 지연, 최소 움직임, 중간 STOP의 잔여 이동, 제동 후 되튐, 회전 중심 이동, 배터리/하중/바닥의 적용 범위를 포함해야 한다. 이전 주행의 평균이나 p99를 곧바로 절대 상한으로 쓰지 않는다. 모델/장비 사양과 검증 가능한 운용 범위를 함께 정의하고, 실험에서 그 범위가 깨지면 설정/운용 범위를 수정해야 한다.

`limits`의 최대 속도/각속도는 age 동안의 위치 전파에 쓴다. 최소 정지 유지 시간, 정지 속도 기준, 관측 만료 시간도 정책의 적용 조건이다. 수신 속도가 느리다는 이유로 만료 기준만 늘리면 정지 오차가 커지므로 다른 상한도 함께 재검토해야 한다.

## 센서 오차 범위

| 필드 | 의미 |
|---|---|
| `tag_pose_error[3]` | 거리·각도별 PnP 바이어스와 장착 오차까지 포함하는 차량 pose 오차 상한 |
| `odom_error_per_m[3]` | 상대 추적의 이동 거리당 누적 오차 |
| `odom_error_per_rad[3]` | 회전량당 상대 추적 누적 오차 |
| `odom_error_per_frame[3]` | 프레임당 오차. 처리 수와 `경과시간×설정 FPS` 중 큰 수만큼 누적하여 처리 누락으로 오차가 줄지 않게 함. 정지해도 초기화하지 않음 |
| `relative_translation_error_m` | 두 영상 사이의 차량 기준점 상대 이동 오차 상한; 정지 속도 검사에 사용 |
| `max_acceleration_mps2` | 평균 이동 속도와 순간 속도의 차이를 제한하는 가속도 상한 |
| `gyro_rate_error_rps` | gyro 잡음/잔류 bias/scale/투영 오차의 각속도 상한 |
| `capture_to_callback_max_s` | 촬영부터 Python 수신 시점까지의 최대 지연. SDK/USB/호스트 대기 포함 |

저장된 오차 필드가 null인 측정 프로필에서 RANSAC residual은 표시되지만 `bounded`는 false다. 이를 오류라고 보고 임의의 작은 숫자를 채우면 안 된다. 정지 속도 상한이 너무 커 정지를 확인하지 못한다면 측정 방식/운용 조건의 문제다.

## 태그 보정 비중

`sensor.tag_fusion`(측정 프로필에서는 최상위 `tag_fusion`)의 `enabled=true`는 현재 VO·gyro 예측과 태그를 결합한다. `max_tag_weight=[0.5, 0.5, 0.5]`는 x/y/yaw별 최대 태그 보정 비중이며 **물리 실측값이 아닌 초기 설계값**이다. 검출 품질과 기존 오차 범위로 실제 비중을 낮춘다. 새 시간당 오차 계수나 과거 run 학습 파일은 요구하지 않는다.

보정 후 오차 반경은 원시 태그 오차 + 보정 중심과 원시 태그 사이 차이다. 평활화로 오차 상한을 줄이지 않으며, 범위 충돌/평면 자세 모호성 거부는 유지한다. 장착 변환이 없으면 절대 위치 결합도 하지 않는다. 기존처럼 원시 태그로 기준을 갱신하려면 `enabled=false`로 비교할 수 있다. 비중 변경을 포함한 설정 변경은 정책 재생성과 검증 대상이다.

화면/관측 로그의 `tag_fusion`에서 원시 태그·예측·실제 보정 비중·남은 차이를, `error_budget`에서 기준 오차·거리/회전/프레임/누락 시간 기여를 확인한다. 확률 공분산으로 표시하지 않는다. [방법·근거·현장 비교](ODOMETRY_METHODS.md)를 참고한다.

## 주행 설정 검증

`validation`에는 `geometry`, `extrinsics`, `vision_error`, `motion_contracts`, `timing`, `stop_detection`의 근거 문자열을 남긴다. 예: 검증 보고서 경로, 시험 조건/범위와 날짜. 각 근거가 실제로 확인된 후 `approved=true`로 설정한다. 코드가 그 문서를 읽고 실차 정확도를 인증하는 것은 아니다.

`hardware.json`은 미완성 현장 설정이다. 확인된 세 치수와 노출 제어는 반영했지만, 전체 장착 보정·차체/현장 형상·오차/동작 범위와 장비 검증이 남아 있다. 시작 IMU 보정, 조건부 사전 정렬, 태그 화각/크기와 현장 추적 구역 검사, 두 시험 목표 생성은 구현했다. 실제 추적 가능한 구역과 수치 범위는 장비/현장에서 확인해야 한다. 측정 모드는 전체 주행 설정 완료를 기다리지 않고 실행할 수 있다.

설정을 변경하면 정책 파일의 hash가 달라져 이전 정책을 거부한다. PC에서 다시 합성한 뒤 같은 설정 파일과 정책을 Nano에 옮겨야 한다. 정책 파일은 텍스트 JSON이며 실행 코드로 역직렬화하지 않는다.


## 야외 영상 품질과 태그 흔들림

`tag.min_tag_px=20`은 태그 네 모서리 중 가장 짧은 투영 변 길이의 하한이다. `quad_sigma=0`은 검출기에 명시적으로 전달한다. `edge_px=15`는 640×480의 초기 검사값이며 v5의 60 px 화각 여유와 동등하다고 검증된 값은 아니다. 실제 스트림의 내부 파라미터를 `camera_model.json`에 기록하고, 가시성 합성은 그 모델을 사용한다.

`camera.quality`에 밝기 250 이상/5 이하 픽셀 비율과 사용 가능한 깊이 비율을 저해상도 샘플로 기록한다. 회전 흐림 추정은 `max(fx,fy) × 자이로 벡터 크기 × 실제 노출 시간`이다. 병진 흐림, rolling shutter, 시야 위치에 따른 차이까지 덮는 엄밀한 상한은 아니다. `diagnostics.reject_rotational_blur_px`는 기본 null(진단만)이며 실제 비교 후에만 거부 문턱으로 정한다. v5의 10/32 px를 다른 영상 조건에 그대로 적용하지 않는다.

`tag_position_window`는 수락된 태그의 카메라 상대 xyz에 대해 최대 60초/6,000개 창의 표준편차를 기록한다. 사람이 정지시켜 둔 상태에서 해석해야 한다. 움직이는 동안에도 통계를 계산하므로 이를 자동 정지 판정이나 절대 오차 상한으로 사용하지 않는다. 작은 태그/정면 모호성 때문에 모든 표본이 거부되면 표본 수가 적거나 0일 수 있다.

## IMU 시작 보정

실장비 설정의 `imu.startup_calibration`을 사용한다. 영상 30장을 모으는 방식이 아니다.

| 필드 | 기본/뜻 |
|---|---|
| `duration_s` | 10초 연속 정지 창 |
| `min_sample_ratio` | 0.5. 실제 시각 길이와 200/100 Hz 요청값으로 계산한 최소 고유 샘플 수 |
| `max_gap_s` | 0.05초. 창 내 각 센서 스트림의 시각 간격 검사 |
| `moving_dps` | 0.5°/s. 설정 bias를 뺀 gyro 크기가 넘으면 다시 수집 |
| `timeout_s` | 60초 내 정지 보정이 끝나지 않으면 실패 |
| `max_gyro_std_dps`, `max_accel_std_mps2` | 각각 0.15 초기 품질 문턱. 실측 오차 상한은 아님 |
| `align_axis_to_gravity` | 중력 방향으로 yaw 투영축 기울기 보정, 기존 축 방향의 부호 유지 |
| `max_axis_tilt_deg` | 30° 초기 장착 일관성 검사. 잘못된 축을 조용히 뒤집어 주행하지 않음 |
| `yaw_sign_verified` | 기본 false. 장비에서 좌/우 회전과 영상/자이로 부호를 별도 확인해야 함 |

정지 구간의 평균 gyro를 해당 실행의 bias로 사용한다. 과거 주행 평균을 학습하거나 다음 실행 설정 파일에 자동 덮어쓰지 않는다. 정지 보정은 차량을 움직이지 않은 상태에서 수행한다. 중력만으로 yaw 절대각이나 등속 직진 중 여부를 알 수는 없다.

## 서로 다른 시간 제한

| 설정 | 측정 시작점과 의미 |
|---|---|
| 동작 `max_start_delay_s` | 명령 발행 → 이동이 오차 범위를 넘어 처음 관측됨. 명령 지연과 이동 검출 지연 모두 포함 |
| 동작 `max_duration_s` | 위 이동 관측 이후의 최대 동작 시간 |
| 동작 `max_stop_response_s` | STOP 전달/반영/잔류 운동에 부여하는 시간 예산 |
| `limits.settle_timeout_s` | STOP 응답 예산에 추가하는 정지 관측 확인 예산 |
| `limits.stop_dwell_s` | 속도/각속도 상한이 정지 기준 아래인 새 관측의 유지 시간. 초기 0.4초는 실차 확정값이 아님 |
| `limits.max_age_s` | 현재 판단에 사용하는 카메라 pose의 최대 나이 |
| `imu.max_age_s` | gyro 수신 자료의 최대 나이. 명령 지연과 다른 항목 |
| `limits.lease_s` | 제어 프로세스가 CAN 송신기에 명령을 갱신하는 최대 간격. 차량 응답을 기다리는 시간과 다름 |
| `limits.run_timeout_s`, `max_steps` | 전체 제어 실행 기본 300초/20개 동작, 현장에 맞게 변경 가능 |
| `prealignment.max_duration_s`, `max_actions` | 정렬 단계 기본 120초/5개 동작. 동작별 출발/정지 대기도 포함 |

실차 프로필의 관측 만료·정지 확인·CAN lease·IMU 만료는 null로 두었다. 측정 프로필의 IMU 0.15초는 상태 진단용 초기값이며 실차 프로필에 확정값으로 전승하지 않는다. 동작별 세 시간은 실차 action 입력 시 필수다. 과거 v5의 회전 8초·직진 15초·deadman 1초·정착 2초는 현 제어의 제한값이 아니므로 설정에서 제거했다.

시간을 늘릴 때 `brake_reserve`, `terminal_error`, `sweep_error`의 **거리/각도 여유**도 검토해야 한다. 지연 중 실제 움직임의 범위가 커진다면 시간 설정만 늘려도 정지 정확도는 좋아지지 않는다. 출발이 늦어도 연속 관측은 계속하며, 자료가 끊기면 해당 지연 대기 예산이 남아 있어도 STOP한다.

## 사전 정렬·가시성 설정

`prealignment`는 콘 시험에서 사용하고 벽 앞 정지 시험에서는 끈다. `entry_x_m`, `center_y_m`은 탑재부 좌표계 정의이고 `corridor_half_width_m`는 실제 반폭이다. `staging_x_range_m`는 정렬 완료 후의 차량 기준점 앞뒤 구간이며, 회전한 차체 전체가 입구 밖에 남아야 한다. `alignment_y_m`, `alignment_yaw_rad`의 전체 조합이 통로 폭 안에 맞는지도 확인한다.

`trigger_lateral_m=null`이면 통로 반폭에서 차체 반폭과 여유를 뺀 문턱을 쓴다. 이 값은 측면 태그까지의 거리가 아닌 **통로 중심선까지의 횡오차**와 비교한다. 일반 접근이 존재하면 문턱과 무관하게 일반 접근을 먼저 선택한다. 실제 알고리즘과 단계 연결은 METHOD.md를 참고한다.

가시성 검사를 위해 `sensor.calibration.camera_model`에 해당 장비/스트림의 기록값을 넣고, 왜곡·보정 오차를 덮는 `visibility.projection_margin_px`를 확인한다. 가시성 검사는 pinhole 구간 투영에 그 픽셀 여유를 더한다. 태그 소실 구간의 `odometry_regions`는 `[xmin,xmax,ymin,ymax]`이며, 이 영역에서 허용할 자세/이동·조명 조건에 대한 현장 추적 근거를 `odometry_evidence`에 남겨야 한다. 코드가 문자열을 읽어 사실을 인증하지는 않는다. 실시간 추적 불량은 계속 거부한다.

`operational.max_forward_m=1.5`, `max_backward_m=0.5`는 이전 최대 단계 길이를 설정 검증에 복원한 값이다. `min_rotate_rad=0.84°`는 이전 저강도 회전 최소량 참고를 보수적으로 반영했으며, 선택한 CAN 강도/새 센서 조건에서도 유효한지는 확인해야 한다. 회전 중심의 예전 평균값은 고정 참값으로 사용하지 않지만 실제 회전 중 병진 편차는 반드시 동작 오차 범위에 포함해야 한다.

## 일시정지·자동 재출발 설정 (2026-10-11)

Nano/VM 실차 프로필의 `recovery.enabled=true`. 아래 기본값은 운용 정책의 초기 선택이며 실제 명령 지연/센서 정확도를 측정한 값이 아니다. `validation.recovery`에 검증 근거가 없으면 실차 모드는 거부한다. 정상 정책 파일도 변경한 설정으로 다시 합성해야 한다.

| 항목 | 초기값 | 의미 |
|---|---:|---|
| `pause_error_ratio` / `resume_error_ratio` | 0.9 / 0.6 | 앞뒤·좌우·yaw 관측 오차 예산 대비 정지/재출발 비율. 재출발 조건이 더 엄격 |
| `stable_s` / `min_samples` | 1초 / 3개 | 실제 새 관측의 시간과 개수를 모두 충족 |
| `min_tag_samples` | 3개 | 추적 소실 후 실제 정지 상태에서 필요한 서로 다른 태그 재검출 |
| `max_sample_gap_s` | 0.5초 | 복구 안정 구간에 허용하는 관측 간격. stale 검사와 별개 |
| `timeout_s` / `max_attempts` | 120초 / 3회 | 한 복구 회차 시간 / 한 실행 전체 회복 시도 수. 전체 300초 제한 등은 계속 적용 |
| `reserve_return_sweep` | true | 각 사이드스텝 동작의 전 구간에서 복귀 회전·STOP의 차체 공간까지 검사 |
| `return_heading_tolerance_rad` | 0.035 rad | 시작 방향 부근에서 더 이상의 탐색 회전을 하지 않을 기준. 태그가 실제 보인다는 보장은 아님 |
| `max_return_actions` | 6회 | 회차당 복귀 회전 수. 전체 동작 수와 사이드스텝 5회 상한도 동시에 적용 |

회전 후보의 CAN 명령·목표 각도·제동 여유는 `actions`의 기존 동작을 쓴다. 검증된 무관측 복귀에서는 방향 오차가 큰 상태를 포함하도록 yaw 종료/이동 범위를 추가로 넓혀 검사한다. 반대 90° 회전이 필요하면 실제 강도 20에서 확인한 ±π/2 rad 동작 계약 또는 작은 회전 후보들을 제공한다. 어느 것도 확인되지 않으면 90° 명령을 임의 생성하지 않는다. `visibility.odometry_pose_regions`는 `[xmin,xmax,ymin,ymax,yawmin,yawmax]`로 방향까지 확인한 추적 구역이다. 기존 `odometry_regions`는 xy 구역으로, 쓰려면 운용하는 모든 방향의 추적을 확인해야 한다. 둘 다 아직 빈 목록이며 실제 배경/깊이/회전 방향의 근거 `odometry_evidence`가 필요하다.

`sensor.imu.stationary_recalibration`: 기본 활성 요청이지만 `verified=false`이므로 연결만으로 임의 재보정을 적용하지 않는다. `duration_s=3`, `max_visual_gap_s=0.4`, `control_max_age_s=1`은 초기 운용값이다. 다음 네 값은 측정 전 `null`로 남긴다.

- `max_speed_mps`: 영상 상대 위치 오차와 구간 가속도를 포함한 속도 상한의 정지 문턱.
- `max_visual_yaw_rate_rps`: 영상 기반 구간 회전 속도 상한의 정지 문턱.
- `relative_yaw_error_rad`: 영상 두 장 사이 상대 회전의 검증된 오차 상한.
- `max_bias_change_rps`: 시작 보정값에서 허용할 자이로 바이어스 벡터의 총 변경량. 작은 변경을 여러 번 누적해 이 범위를 넘길 수 없음.

장착 변환, 상대 이동/회전 오차, 시각 정지 검사가 준비되어야 재보정 증거를 얻을 수 있다. 가속도/자이로가 조용하거나 중립 명령을 보냈다는 사실만으로 재보정하지 않는다. 기본 10초 시작 보정은 이 기능과 별개다. 새 재보정 뒤에도 `bounds.gyro_rate_error_rps`가 실제 잔여 오차를 포함하는지 확인하고 `verified=true`로 설정한다. 카메라 프로세스에 전달하는 중립 명령 정보는 재보정 자격만 주며 CAN watchdog을 해제하지 않는다.

자동 재출발 조건을 먼저 충족해도 이번 정지의 적격 자이로 보정 창이 수집 중이면 완료를 기다린다. 따라서 1초 관측 안정 조건이 3초 보정 창을 매번 중단시키지 않는다. 보정 창이 부적격으로 취소되면 기존 바이어스·오차 상한을 유지해 재출발 조건을 판단한다. 전체 복구 제한 시간은 기다리는 동안에도 흐른다.

화면/로그: `controller.recovery_attempts`, `recovery_samples`, `recovery_tag_samples`, `returning_to_tag`; `observation.anchor_stamp`, `tracking_generation`, `imu.stationary_recalibration`. 보정이 실제 적용되면 세션에 `stationary_imu_0001.json` 등이 저장된다.

## 최초 한 번의 태그 탐색 (2026-10-11)

Nano/VM 실차 프로필의 `startup_search.enabled=true`다. `INITIAL_WAIT → INITIAL_SEARCH → INITIAL_BRAKING → INITIAL_MEASURE`를 거쳐 일반 제어기로 넘긴다. 처음부터 태그가 보이면 회전은 건너뛴다. 완료한 초기화는 같은 실행의 주행 중 소실/복구에서 다시 실행하지 않는다.

| 항목 | 초기값 | 의미 |
|---|---:|---|
| `direction` | left | 기존 강도 20의 좌 제자리 회전. right도 지원 |
| `timeout_s` / `max_rotation_s` | 180초 / 120초 | 최초 획득 전체 / 실제 회전 시작 관측 후 시간. 전역 실행 시간 제한도 유지 |
| `initial_wait_s` / `min_missing_samples` | 1초 / 3개 | 센서 준비·정지 후 실제 태그 검사로 부재 확인 |
| `stable_s` / `min_tag_samples` | 1초 / 3개 | STOP 후 새로 수락된 태그 자세로 위치 확정 |
| `max_sample_gap_s` / `pose_error_ratio` | 0.5초 / 0.6 | 관측 연속성 / 출발 오차 예산 비율 |
| `max_sweep_rad` | 2π | 제동 후 회전까지 포함한 최대 각도. 제동 여유만큼 먼저 STOP |

위 값은 운용 기본값이며 장비 측정값이 아니다. 다음 항목은 측정 전 `null`이다.

- `max_start_delay_s`, `max_stop_response_s`: 회전 출발 지연과 STOP 응답 상한. 회전 시간과 구분한다.
- `brake_reserve_rad`: 지연·잔류 회전까지 덮는 제동 각도 여유.
- `max_center_drift_m`, `stop_drift_reserve_m`: 최초 차량 기준점의 병진 편차 예산과 제동 중 남겨 둘 여유.
- `free_radius_m`: 실제 출발 기준점 주변에서 확인한 장애물 없는 원의 반경. 차체 전체 회전 반경 + 병진 편차 + 공간 여유 이상이어야 한다. 카메라가 자동으로 장애물을 탐지해 채우는 값이 아니다.

`validation.startup_search`에 확인 근거가 필요하다. 현장 파일은 `startup_search.free_radius_m`만 덮어쓸 수 있고 시간/응답 설정은 Nano/VM 프로필에 남긴다. 변경한 설정으로 정책을 다시 생성해야 한다. 상태는 `controller.initial_acquisition`에 최초 탐색 단계, 회전/병진 오차 포함 상한, 태그 관측 수로 기록된다.


## 리뷰 4·5·6·7 보완 설정

다음 수치는 학습 파라미터가 아니라 측정·운용 범위이다. Nano와 VM은 각각 검증한다. 센서만 연결한 `measure`는 미측정 상태에서도 가능하지만 `drive`는 검증되지 않은 활성 기능을 거부한다. 아래 초기 시간값은 장비 측정 결과가 아니다.

| 설정 | 초기 상태 | 용도와 검증 |
|---|---|---|
| `sensor.tracking_recovery` | enabled=true, verified=false | `hold_keyframe_s=0.6`: 일시 실패 시 옛 영상 유지. `max_bridge_s=0.3`: 기존 명령을 유지할 수 있는 최대 예측 구간. `max_episode_s=120`: 재연결 후에도 태그 재관측 없이 오차가 누적되는 회차 상한 |
| `max_unobserved_speed_mps` | null | 추적이 없는 동안 가능한 병진 속도 상한. 전체 속도 한계 이상이어야 하며 위치 오차는 매 구간 이 값×시간만큼 증가 |
| `recovery.blind_return` | enabled=true, verified=false | 완전한 배경 소실 시 연속 자이로와 검사한 동작 범위를 사용하는 복귀 전용 경로 |
| `blind_return.stop_rest_after_s` | null | STOP 요청부터 잔류 속도가 지정 상한 이내가 될 때까지의 검증된 시간. 통신 지연·관성·하중·노면 포함. 단순 임의 sleep 값이 아님 |
| `blind_return.rest_speed_upper_mps` | null | 위 시간이 지난 후 잔류 속도 상한. 정지 문턱 이하이고 회전 동작 계약이 이 잔류 움직임을 포함해야 함 |
| `blind_return.max_heading_error_rad` | null | 복귀에 허용하는 자이로 방향 오차. 정밀 위치 관측 예산과 별도로 사용 |
| `sensor.stop_window` | enabled=true, verified=false | min_s=0.15, max_s=0.75, reference_period_s=0.15, max_frame_gap_s=0.5. 영상 개수가 아닌 실제 촬영 간격 사용 |
| `stop_window.translation_error_m`, `yaw_error_rad` | null | 선택한 기준 영상→현재 영상 **직접 변환**의 오차. 인접 프레임 오차를 표본 수로 나눈 값이 아님 |
| `stop_window.max_angular_acceleration_rps2` | null | 시각 회전 속도 상한의 구간/노후화 보정과 최신 자이로 샘플의 수신 후 경과 시간 보정. 병진 가속도는 기존 `bounds.max_acceleration_mps2` 사용 |
| `startup_search.improvement` | enabled=true | 2초 재시도 간격, 최대 4회. 잘림 시 방향 보정, 재소실 시 최초 탐색 예산 안에서 재검색, 과다/과소 노출이면 AE 요청 |
| `camera.allow_startup_auto_exposure` | true | 초기 정지 재측정 중 수동 노출에서 자동 노출로 바꾸는 명시적 허용. 주행 중에는 자동 전환 요청하지 않음 |
| `can.diagnostics` | enabled=true, verified=false | TXACK, 송신 간격, 외부 주행 프레임, bus-off/error-passive/overrun 검사 |
| `can.diagnostics.max_ack_age_s`, `max_write_gap_s` | null | 호스트별 실제 부하에서 검증할 전송 알림 지연/쓰기 간격 상한. 차량 명령 반응시간과 다른 항목 |

정지 속도 상한은 `(직접 측정 변위 + 변위 오차)/관측 간격 + 가속도 상한×간격/2`에 영상 노후화 항을 더한다. 간격을 늘려도 가속도 항은 커진다. 가능한 간격에서 정지 속도 문턱을 만족시킬 수 없는 설정은 **실차 실행 전에 거부**한다. 무조건 창을 늘리거나 오차를 표본 수의 제곱근으로 나눠 정지 판정을 통과시키지 않는다. 정지 영상과 알려진 저속 이동을 둘 다 확인해야 한다.

`stop_motion`은 절대 목표 위치와 독립적인 시각 이동 검사다. 이 자료가 없는 프레임에서 예전 `speed_upper` 값으로 자동 대체하지 않는다. `return_only`는 이 시각 검사를 대신하지 못하며 자이로 재보정에 사용하지 않는다.

`blind_return`이 검증된 경우에만 복귀 후보의 시각 가시성 조건을 생략한다. 촬영 후 지연까지 포함한 방향 오차가 `max_heading_error_rad` 이하이어야 하며, 상한으로 제동 시점을 잡을 때 실제 방향이 최대 두 배의 오차만큼 일찍 멈출 수 있어 회전 종료/이동 yaw 범위에 `2*max_heading_error_rad`를 추가한다. 명령 강도·목표 각도·제동 여유는 변경하지 않는다. 복귀 STOP 뒤에도 관측 오차 범위를 종료점과의 교집합으로 줄이지 않는다. 차체·통로 입구·자유공간·회전 동작 범위 검사는 유지한다. 일반 전진/후진 정책의 가시성 조건은 그대로다. 복귀는 정상 도달 정책 밖의 복구 절차이므로 성공적인 태그 재획득이나 최종 도달을 무조건 보장하지 않는다.

CAN 진단은 별도 프로세스에서 비차단 수신 시간/개수를 제한한다. 시작 중립, 정지 전환, 종료 중립 payload와 주행 mode `0x0A`는 유지한다. STOP 전환에는 대기 중인 TX 큐를 비워 오래된 주행 명령이 뒤늦게 재생되는 것을 줄인다. **TXACK는 전송 알림이며 차량 구동·기계적 정지 피드백이 아니다.** 동일 payload가 반복되므로 개별 명령별 ECU 반응 시간 측정에도 사용할 수 없다. 진단 실패는 중립 송신 시도와 해제 불가 abort로 처리한다. 전기적 단선/버스 고장 시 중립 도착을 보장하지 않는다.

관측 로그에는 `tracking_recovery`, `stop_motion`, `tag.bearing_body_rad`가 기록된다. `return_only`와 `stop_proof`는 제어기 내부 복귀 판단에서 사용하며 원래 카메라 관측을 덮어쓰지 않는다. 제어 로그·화면·최종 session에는 CAN diagnostics, 복구 횟수, 복귀 동작 상태가 기록된다. AE 변경은 `exposure_recovery_N.json`에 남긴다.

추가 제어 상태 `latest_gyro_stop_ok`는 현재 제어 주기의 자이로 정지 조건이다. 실차에서 False이면 영상이 조용해도 정지 유지 시간을 초기화한다. 최신 샘플의 수신 후 경과 시간이 양수인데 검증된 각가속도 상한이 없으면 정지로 인정하지 않는다. `stop_proof`는 복구 정지 유지의 `visual`/`model_return`을 구분한다. 모델 근거에서는 특징점 `tracking_generation` 변경만으로 유지 시간을 초기화하지 않지만, 카메라 만료·샘플 간격 초과·자이로 움직임/끊김에서는 초기화한다.

`recovery.max_sample_gap_s`는 태그 재획득 관측의 유효 간격에도 적용한다. 한 장만 보인 뒤 이 간격을 넘기면 태그 관측 개수와 안정성 검사를 다시 시작한다. 사이드스텝 복귀 중이면 남은 복구 예산 안에서 복귀 후보를 재선택할 수 있다. 새로 수락한 태그에서 이어진 유효한 배경 추적은 복귀 회전의 위치 근거로 쓸 수 있지만, 일반 재출발에는 여전히 여러 태그 관측과 정지·안정성 조건이 필요하다. 복귀 중 태그 발견으로 STOP해도 기존 시간·동작·복구 횟수 예산을 갱신하지 않는다.
