# v6 구현 검사 기록

2026-10-16, macOS arm64. Python 3.10.19 / NumPy 1.26.4 / OpenCV 4.12.

## 2026-10-19 태그 주·VIO 보조 재설계 (macOS, Python 3.12 / NumPy / OpenCV 5.0)

- `PYTHONPATH=.:tests python3 -m unittest discover -s tests`: **161개 통과** (OpenCV 5.0 환경; 4.x 전용 3개 오류는 사라짐). 합성 관측만 썼고 실차·정확도 주장은 없다.
- `docking/fusion.py` 신설: β·거리 + VIO 방향(offset) → 위치, 0.5 초/≥3 장 창 앵커, 3σ 대조와 3 연속 충돌 재앵커, 30 초 σ 기준선(합격·불합격 없음, 절반 중 큰 값, 조용한 30 초·기본값 폴백). `alignment.py`(covariance intersection)·`uncertainty.py` 의 CI/innovation/drift_bound 삭제.
- `docking/stationary.py` 재작성: 자이로 p90(0.5 초, 0.5°/s) + VIO 속도(0.03 m/s) + 중립 뒤 2 초 규칙. 카메라 워커가 200 Hz 원시 자이로를 관측기에 넘긴다.
- 컨트롤러: 준비 30 초, 진입 조건 |예상 도착 좌우| ≤ 3 cm 만(σ 는 `lateral_check` 기록), `blind_entry_uncertainty_too_large` 경로 삭제.
- mount: VIO 정지 증명·IMU gap·기록 gate 삭제, 수락 태그 ≥1 Hz·커버리지 50 %·공백 10 s. 반복성 gate(1°/0.5°/5 cm/30°) 유지.
- 합성 검사: 임의 odom gauge 에서 dock pose 복원, PnP 방향 2° 치우침에도 융합 좌우 1 cm 안(PnP 는 8 cm 밖), 태그 없는 1.3 m VIO 전파와 σ 증가, 0.5 m 점프의 창 분리·재앵커, 기준선 계산·폴백·동결, 자이로 회전/직진/시간 규칙/bias/패킷 폴백, mount 1 Hz 통과·0.5 Hz 탈락.
- mount-straight(같은 날, 뒤이어): 전진 3 → 5 초(`PULSE_S`), 포크 여유 거리도 그에 맞춰 ≈3.1 m. 통과 조건을 없애고 "계산 불가" 둘(양 끝 태그 3장 미만, 이동이 끝점 흩어짐 2배에 묻힘)만 저장을 막는다. 정지 확인 비율·전진 중 회전·VIO 리셋·수평도·σ 는 `warnings` 로 기록. 60 초 안에 정지가 확정되지 않으면 마지막 3 초 관측을 끝점으로 쓰고 경고. 162개 통과.
- 정리(같은 날): 서서 재는 `preflight mount`(법선 정렬 전제)와 `mount_calibration.py`, 수동 직진 `straight_mount.py`(CLI 없음), hand-eye 도구(3차원 운동 필요), 맥 보관소 검사 `scripts/verify_preparation.py` 와 그 테스트를 지웠다. `mount-straight` 가 유일한 장착 보정이다. 144개 통과.
- 같은 날: CAN 우편함 자물쇠 대기 5 ms → 1 s(`SET_LOCK_WAIT_S`, v4 의 `with lock:` 과 같게, 상한은 lease). 태그 `quad_decimate` 1.5 → 1.0(사용자 결정: 멀리서 작은 태그 우선, CPU 여유 확인됨 — 검출 스레드 부하 증가는 Jetson 에서 다시 볼 것).
- Jetson 손 실험 2회(같은 날 20:12·20:22, `work/` 에 보관): ① 1회차는 태그를 안 봤고 손으로 438°/s 까지 돌려 OpenVINS 가 7.8 s 이후 갱신 0 회로 발산(40 s 에 400 m). ② 2회차는 태그 222/275 검출, VIO 초기화 2.7 s 뒤 계속 보정, 정지 시 융합 위치 안정(잔차 p90 9 mm), PnP−VIO 방향 4.1~5.5° 일정. 드러난 문제 둘: (a) 1~2 s 프레임 공백 7회(파이썬 멈춤 아님, 감독 루프 최대 14 ms; 카메라 워커에 새 프레임이 안 옴) → VIO epoch 7회 리셋; (b) 4.2 m 기준선을 1 m 로 비례 축소하면 σ 2~5 mm 라 1~2 cm 잔차가 충돌 92/628 → σ 바닥 1 cm·0.5° 추가(허용 3 cm 의 1/3) 뒤 재계산 0. 공백 원인은 카메라 쪽 계측(`delivery_gaps`, `incomplete_framesets`, `exposure`)을 넣고 다음 실험에서 가린다. 태그 검출기는 본 태그 전부(`seen`)와 영상 밝기를 기록.
- 3회차(20:30, 책상 위 1.12 m, 124 s 정지): 태그 723/723, 융합 위치 sd x 0.6 mm·y 0.4 mm·방향 0.02°(PnP 그대로 0.8 mm·0.05°), β sd 0.007°, 거리 sd 0.2 mm, PnP−VIO 방향 −4.1° 일정. 30 s 기준선이 0.1 mm 로 잡혀 58.8 s 부터 3σ 대조가 0.05 mm 잔차를 충돌 98회로 셈 → **대조 검사를 물리 상한으로 교체**(`fusion.jump_allowed`: max(3 cm, 1 cm/m×거리) + 0.29 m/s×경과; 건너뛰기, 1 s 뒤 태그 채택; σ 는 기록만). 멈춤 0.3~1.1 s 11회는 책상 위에서도 났고 기록 큐는 비어 있었음(pending ≤3) → 기록 적체는 아님; 카메라 워커 프로세스 전체가 서는 양상(GC 등)을 다음 실험의 계측으로 가린다. 145개 통과.
- 4회차(21:16, 책상 위, py-spy 없이): 30 s 만에 브리지 `imu_sequence_or_clock_gap` 으로 종료 — 멈춤이 IMU 표본을 떨어뜨릴 만큼 길어진 것. **IMU 공백을 치명 오류에서 뺌**(사용자 결정, v4 규칙): 파이썬 `ImuSynchronizer` 와 C++ 브리지 모두 공백·역전을 세기만 하고(`gaps`, `reversals`, `largest_gap_s`), 정지는 `imu_age_s` 1 s(v4 IMU_STALE_SEC)만. `quality_version` 1→2 로 올려 옛 브리지 바이너리는 거부(rebuild 필요). 멈춤 원인용 계측 추가: GC 멈춤 시간·세대, 메인 루프 단계별 최대 소요(ms), 프레임 공백 — 1초마다 `pipeline_status.json` 과 `observations.jsonl`(`kind: health`). 기록 다이어트: 추적점 좌표를 기록에서 빼고 개수만(관측 8.5→5 KB, VIO 6.2→3.2 KB). 145개 통과.
- 정리 2(같은 날): 실행 경로에서 아무도 안 쓰던 것 삭제 — `dependencies/references/`(WPILib·robot_localization·VBO 사본)와 `REFERENCE_CODE.md`, `docking/evaluate.py`, `docking/calibration_tools.py` 와 `--sensor-calibration` 옵션, 루트 `preflight.py`, `uncertainty.compose_cov`, 의존성 기록의 RTAB-Map·ORB-SLAM3 항목. 환경은 가상환경 `~/.venvs/krri` 하나·작업 공간 `~/.local/share/krri/ros_ws` 하나로(설치 스크립트 `--no-apt` 추가, 옛 경로는 삭제 전까지 인식). 143개 통과.
- 장착각(같은 날, 사용자 결정 "CAN 으로 보정 전진 보내는 것 싫다"): `mount-straight` 와 `Pulse`(CAN 5초 전진), runtime 의 보정 분기, 포크 여유 검사 삭제. 새 `preflight mount --folder <measure 폴더> [--write]` 가 "정지 ≥5 s → 수동 직진 → 정지 ≥2 s" 로 찍힌 로그를 자이로 정지 판정으로 잘라 같은 계산(`mount_pulse.estimate`)을 돌리고 저장. 합성 로그로 yaw 7° 복원·저장·`require_mount` 통과 확인. 137개 통과.
- 장착각 2(같은 날, 사용자: "v4 는 안 움직이고 쟀다", "수동 직진은 더 별로"): v4 `calibrate camyaw`(법선 정면에 눈대중 정차, PnP 회전 블록 중앙값, "절대 정확도 모름" 주석)를 `docking/mount.py` 로 복원하고 **이것만** 남김. 직진 방향 방식·`mount_pulse.py` 삭제. 합성 로그로 yaw ±8°/0°/7° 복원·저장·`require_mount` 통과 확인. Jetson 에서 `test_tag_cut_transition_accounts_for_observed_yaw` 가 실패한 원인은 테스트가 Jetson 의 `common.json`(태그 0.2 m)을 읽어 태그컷이 옮겨진 것 → `tests/support.config` 가 현장 값을 전부 고정하도록 수정. 설치·재빌드 스크립트는 빌드 확인 직후 `.docking_rebuild_pending` 을 지우고 코드 검사는 그 뒤에 돌린다(검사 실패가 실행을 막지 않게). 132개 통과.
- 시작 순서·주행 중 장착각(같은 날, 사용자 결정): Enter 전 준비 5 s(`preparation_s`, common.json operation 도 5.0; 사용자 선택, 최소는 OpenVINS 초기화 창 2 s) → Enter → 태그 없으면 탐색 → 태그 보이는 자리에서 30 s `BASELINE` 상태(σ 기준선, 컨트롤러가 `baseline_lock` 으로 카메라 워커에 동결 신호) → 계획. 장착 yaw 는 `fusion.fit_mount_yaw`(고정 태그의 odom 위치 일치 조건, 선형화 최소제곱)로 좋은 직진마다 배워 β·PnP 방향에 적용, VIO 리셋을 넘어 유지. 전환점에서 `good_legs==0` 이어도 들어가고 `entering_with_unlearned_mount_yaw` 를 기록(사용자: 멈추지 말 것). 합성 검사: 2° 장착 오차를 0.96 m 직진 한 번에 0.1° 안으로 복원하고 방향·위치가 맞음, 시작 순서 3경우, 규칙 1경우. 138개 통과.
- 책상 예행(23:18, shadow): `mount` 가 yaw −2.12° 와 함께 pitch −23°(기운 종이 태그)를 저장해 거리가 cos 23° 만큼 줄었고, 카메라가 목표(태그면 2.0 m 앞)보다 가까워 `target_overshoot` 로 종료. → `mount` 는 **yaw 만 적용**(pitch·roll 기록만, 5° 넘으면 경고), BASELINE 은 2 s 안의 VIO 끊김을 기다리고 태그가 2 s 넘게 안 보일 때만 탐색으로 복귀. 두 폴더 모두 적용. 138/135개 통과.
- 죽지 않기(23:18 예행 뒤, 사용자: "지금처럼 죽는 일이 있으면 안 돼"): 시작부터 목표를 지난 경우(`target_overshoot`)는 FAULT 가 아니라 뒤 공간·최소걸음 안이면 후진 한 번, 아니면 `already_past_target_recorded_not_judged` 로 완료. 컨트롤러가 세운 모든 FAULT 는 트레이스백 대신 `정지(판단): <이유> · 로그: <폴더>` 한 줄로 끝남(차는 이미 정지). `mount` 는 yaw 는 태그, pitch·roll 은 IMU 중력(OpenVINS 자세, `gravity_tilt`)에서 — 책상에서 카메라도 기울었다는 지적; 태그 기울기와 5° 넘게 다르면 경고. 
- 미검증: 실차 자이로 bias·진동, OpenVINS 상대 공분산 크기, 정지 방식의 눈대중 정렬 오차, 실차 직진 다리에서의 ε 흩어짐(10/1 재계산은 0.4°), decimate 1.0 에서의 태그 검출 시간, 프레임 공백의 원인.

- 2026-10-21 **depth 화면 + 입구 골 지점**(사용자 요청, 표시·기록 전용): `--preview depth` 로 depth 스트림을 화면용으로만 켬(태그 검출기엔 안 넘김 — 프로젝터 꺼진 패시브 depth 의 구멍 때문에 `depth_disagrees` 로 멀쩡한 태그를 거부할 수 있어서). 흑백(가까울수록 흰색, 검정 = 값 없음). `docking/depth_goal.py`: 화면 가운데 20 % 띠의 열별 중앙값 depth → 이동 중앙값 → 이웃 열 depth 점프(> max(0.15 m, 8 %))를 경계로("가까움→멂" = 왼쪽 벽 끝, "멂→가까움" = 오른쪽 벽 끝) → 가운데에 가장 가까운 입구를 골로. 좌우는 depth 스트림 내부 파라미터와 경계의 가까운 쪽 depth 로(왼쪽 +). 결과는 화면(노랑 = 입구 경계, 빨강 = 골, 초록/하늘 = 모든 depth 경계)과 `observations.jsonl` 의 `kind: depth_goal` 행. 제어에는 안 씀. 합성 장면 시험 5 + 2 개. 실제 depth 로는 미검증.

## 2026-10-21 카메라 위치 입력 정리 + 사이드스텝 옆밀림 보정 (사용자 요청, 폴백 버전 위에)

- **카메라 위치**: common.json 에 줄자 값 셋만 — `camera_to_fork_tip_m`(렌즈→포크 앞끝, 앞뒤), `camera_lateral_m`(차 중심선→렌즈, 왼쪽 +·오른쪽 −), `camera_height_m`. `rear_extent_m` 은 비우면 새 항목 `vehicle_length_m`(2.17) − `camera_to_fork_tip_m` 로 계산. 회전중심은 차의 성질(포크 끝 뒤 1.005 m · 중심선 오른쪽 0.112 m)로 두고, 몸체 좌표 원점이 "카메라 앞뒤 위치의 차 중심선 위 점" 이라 카메라를 앞뒤·좌우로 옮겨도 코드 수정 없이 다시 계산됨(앞서 "옆으로 옮기면 회전중심 좌우도 고쳐야" 라고 한 것은 틀림).
- **사이드스텝 옆밀림**: 제자리 회전 두 번이 몸체 원점을 회전중심 둘레로 옮기는 양(= 회전중심x·sin(출발 방향) + 회전중심y·(cos−1))을 `moved()` 로 미리 계산해 옆 이동 거리에 반영, 목표는 중심선(fraction 1.0). 이동 상한 1.5 m 는 유지하고, 끝난 직후 다시 잰 좌우가 0.15 m 넘으면 **한 번 더**(총 2 회). 오늘 다섯 판의 출발값으로 재연: 예전 0.25~1.23 m 이동 → 새 규칙으로 163525·174331 은 0.00 m 에 도착, 1.5 m 상한에 걸린 162223·164024·173741 은 남은 0.27~0.87 m 를 재시도로. 시험 +4.
- **depth 골 지점**: 화면·로그에 "차 중심선 기준" 좌우(`goal_lateral_vehicle_m` = 카메라 기준 + `camera_lateral_m`)도 표시 — 카메라를 옆에 달 때 대비.

- 2026-10-21 **`forward_path_bias_deg`**(광운대 `FORWARD_PATH_BIAS_DEG` 자리, 사용자 결정): 직진 때 실제로 흐르는 방향각(왼쪽 +). 예상 도착 좌우(`arrival_y`·`target_lateral_uncertainty`)와 눈 감는 거리가 "차체 방향 + 이 각" 으로 계산되므로, 값을 넣으면 진입 전 보정 회전이 그만큼 반대로 미리 겨눈다. **지금은 비움(0)** — 내일 카메라를 옆으로 옮긴 뒤에도 오른쪽으로 가면 흐른 양으로 채운다(10/20 3 판: −1.7/−4.2/−3.0°). 걸음마다 다시 재는 보통 직진에는 안 쓴다. 시험 +4.

## 2026-10-21 폴백 — 163111 판 시점으로 (사용자 결정)

- 기점: 2026-10-20 판 `163111_9b3d3b_drive`(입구까지 가장 잘 간 판)를 돌린 코드. 그 뒤 오른쪽 출발 실패(164800·174331)를 고치려 넣은 것을 뺌: 사이드스텝 여유 1 m → **3 m**, 방향 가중치 → **15** 고정(`plan_heading_weight`·`plan_lateral_weight` 항목 삭제, common.json 의 `operation.plan_heading_weight` 도 삭제), 종착 좌우 6 → **3 cm**, 보정 소진 시 한 번 후진 → **FAULT**, 정렬선(`final_straight_m` 등)과 사이드스텝 이동량의 옆밀림 반영 → 삭제. 시험도 그 시점으로.
- 남긴 것: 학습 오염 수정(정속 다리만 학습·관성 상한 0.8 s·순간 속도 VIO 자로만 — 버그 수정이라 사용자 권고대로 유지), depth 화면 흑백 + 입구 골 지점(10/21, 표시·기록 전용), `rear_space_m` 1.5(설정).
- 확인: 지금 설정을 163111 의 `config_used.json` 과 대조 — 차이는 `rear_space_m`/`backward_max_m` 0.5 → 1.5 하나(의도). 회전중심 −0.775 동일. 시험 161 개 통과. 폴백 전 전체는 `../forklift-truck_docking_prefallback_20261021.tar.gz`(work/ 제외).

## 2026-10-20 광운대 첫 실주행 — 전진 걸음이 안 섬 (VIO 가 포크를 봄)

- 판 `work/20261020/142426_ad8a94_drive`(9.4 m, left) · `143218_47a1ee_drive`(8.7 m, 수동 정지): 준비 → Enter → 30 s 정차 → 우회전 3.75° 요청에 3.03° 실행(정상, `step_done`) → 전진 0.8 m 걸음이 **15 s 한도(3.5 m)** / **5.6 m(사람이 세움)** 까지 안 섬.
- 원인(첫 판 로그): 전진 14 s 동안 태그 거리 9.43 → 5.8 m 인데 **VIO 위치 변화 0.005 m**. OpenVINS 는 특징점 ~100 개·SLAM 갱신 21/프레임으로 "정상" 보고, 회전 3° 중에도 ZUPT 44/51(화면이 0.5 px 도 안 움직임). 화면 확인: IR 아래 절반이 포크 레일·캐리지이고 점이 거기 몰림 → 카메라와 같이 움직이는 점을 추적해 "정지" 로 판단. 책상에선 없던 조건. IR 영상은 살아 있었음(`ir_mean` 변화, 중복 0).
- 대책 ① 브리지 `mask_bottom_fraction`(runtime.json `openvins.ir_mask_bottom_fraction`, 0.5): IR 아래 비율을 OpenVINS 마스크(255)로 제외. 재빌드 필요. ② 컨트롤러 `progress()`: 직진 걸음의 진행 거리를 VIO 변위와 **태그 기준(입구 좌표) 변위 중 큰 쪽**으로 — 둘 중 하나가 목표에 닿으면 정지(보수적). 태그가 안 보이면 VIO 만(눈 감는 구간·탐색은 그대로). `step_done` 에 두 자 값(`progress_parts_vio_tag`) 기록. 시험 2개 추가(VIO 고정·태그 이동 → 0.8 m 에서 정지 / 태그 소실 → VIO 로 정지).
- 같은 판에서 확인된 것: 장착각(yaw −2.46° · pitch 1.06° · roll −0.49°, σ 0.66°, 70 s) 저장·서명 검사 통과, `translation_timeout_s` 30 으로(사용자 결정, 한도는 보험일 뿐), `camera_lateral_m` −0.015(컬러 렌즈가 중심선). 두 번째 판 끝은 `motion_direction_mismatch`(사람이 세운 뒤 되튐).

- 같은 날 오후(사용자 결정): 카메라를 **포크 앞쪽**으로 옮김 — `camera_height_m` 0.59→0.46, `camera_to_fork_tip_m` 1.47→0.23, `rear_extent_m` 0.70→1.94(전장 2.17), `camera_lateral_m` −0.015 유지. 포크·마스트가 IR 에 안 보이고 가까운 바닥이 보여 삼각측량이 되는 자리. **IR 마스크 코드는 제거**(브리지 파라미터·runtime.json 키·전달 경로; 마스크 0.5 는 남은 점이 전부 20 m 밖 건물이라 영상 갱신 0/0 → `correction_stale` 로 60 s RECOVER 후 종료, 판 `145147_ee34b7_drive`). **"VIO 변위·태그 변위 중 큰 쪽" 정지 규칙도 보류**(사용자가 더 생각, 코드·시험 제거; 그 판에서 전진 걸음은 이 규칙이 0.5 m 에서 세웠었음). 옛 장착 치수는 common.json 설명에 남김.

- 판 `152416_ab9b2f_drive`(8 m, left, 회전중심 옛 값): 접근은 끝까지(후진 0.5 → 회전 → 전진 ×3 → … → 약한 회전 → 눈 감고 진입). 눈 감는 거리 **4.1 m**(카메라 0.46 m 라 태그가 컬러 윗변으로 3.3 m 에서 잘림), VIO 진행 3.0 m 에서 **사람이 비상정지**(포크 끝이 입구면 6 cm 앞, 좌우 −9 cm, 오른쪽 꼬깔콘에 붙음); 그 뒤 18 s 동안 CAN forward 가 계속 나가다 30 s 한도 FAULT. 진입 판단 때 예상 도착 −0.7 cm(좌우 −12 cm 를 방향 +1.5° 로 상쇄) — 그 방향은 **눈대중 장착 yaw 그대로**: 좋은 직진 규칙(5 m·10°)이 13 걸음 전부 거부(`far`/`short`/`oblique`; left 태그는 5 m 안에서 방위각 15~25°). 멀리서 후진한 건 점수(좌우 25/m vs 거리 3.4/m)와 틀린 회전중심의 합작.
- 네 가지 반영(사용자 결정, 배포 대기): ③ 좋은 직진 규칙 **9 m·25°**(이 현장 기하: 출발 9 m, 태그컷 방위각 21°; v5 10/1 자료에서 먼·비스듬한 직진도 ~0.3° 만 나빴음). ④ 계획 점수: 마지막 정렬 단계에서도 실제 좌우·방향을 점수에 두고 종착 조건에 |좌우| ≤ 3 cm 추가(상쇄 대신 "중심 먼저, 그다음 직진"). ⑤ 눈 감는 구간에서 출발 뒤 **3 s 진행 없음 → 그 자리를 도착으로 기록하고 완료**(`blind_entry_stalled_recorded`, FAULT 아님). ⑥ `blind_distance_limit_m` 5 → 6. 회전중심은 포크 끝 기준(`rotation_center_behind_fork_tip_m` 1.005)으로 common.json 에. 시험 +1(막힘). 사이드스텝은 이걸로 안 되면 다음 후보.

- 판 `155618_a20d53_drive`(8.3 m, 새 회전중심·네 가지 반영): 후진 없음, 좋은 직진 4 → 장착 yaw 학습 −2.14→−0.98→**−0.71°**(눈대중 −0.65° 와 일치), 입구 4.7 m 앞에서 좌우 −7 cm·방향 2.6° 까지 정렬. 그 뒤 **0.2 m 전진 걸음에서 VIO 가 또 멈춤**(태그로는 1.7 m 이동, VIO 0.07 m; 특징점 100+, ZUPT 아님, 노출 일정 — 원인 미확정, 탑재부 앞 매끈한 바닥 추정) → 30 s 한도까지 전진, +2.5° 로 들어가 왼쪽 벽(비상정지).
- 판 `160536_6df6ab_drive`(5.8 m, 좌우 −1.1 m·방향 31°): VIO 정상(걸음마다 태그와 10 cm 안). 계획기가 31° 비스듬히 전진 ×3 으로 좌우를 지우고 태그컷에 30° 를 남긴 채 도착 → 제자리 약한 회전 4 번으로 폈는데 회전중심(카메라 뒤 0.78 m) 탓에 카메라가 오른쪽 21 cm 로 밀림 → 앞 여유 없음·마지막 구간엔 후진 후보 없음 → `no_feasible_progress` 종료(새 "좌우 3 cm" 조건이 상쇄 진입을 막은 결과).
- **세 가지 반영(사용자 결정)**: ① 직진 걸음 정지 = VIO 변위·태그 기준 변위 중 큰 쪽(`progress_parts_vio_tag` 기록). ② **사이드스텝** 한 번: 첫 계획 때 탑재부 중심선에서 좌우 ≥ 0.5 m 이고 태그컷까지 ≥ 3 m 남았으면 중심선을 향해 ±90° → 좌우 오차의 90 %(≤1.5 m) 전진 → 다시 입구 방향. 태그가 안 보여도 진행(VIO 로 전파된 위치), 회전 한도 15 s. ③ 마지막 구간에서 후보가 없으면 `rear_space_m` 만큼 **한 번** 후진 후 재계획, 두 번째는 FAULT. 시험 6개 추가.

- 판 `162223_b2b945_drive`(8.2 m): 사이드스텝 실행(우회전 → 전진 → 좌회전, 그동안 태그 없음) 뒤 보통 접근; 멀리서 계획 대기가 길어지다(자리 흔들림 재계획 반복) 답이 3 s 안에 못 와 `planner_timeout` 종료. **속도 둘 + 안전 하나 반영(사용자 결정)**: ① 계획 수락 검사 문턱을 고정 1 cm·0.5° 에서 **측정 흩어짐(confidence_radius = 2σ)** 으로. ② 태그컷 3 m 밖에선 한 걸음 **1.5 m**(`forward_max_far_m`), 안쪽 0.8. ③ 계획기 자체 시간(`planner_budget_s` 2.4)과 컨트롤러 대기(`planning_timeout_s` 6)를 분리하고, 늦은 답은 **3 번까지 재요청**(`planner_late_rerequest`) 후에만 FAULT. 시험 3개 추가.

- 판 `163111_9b3d3b_drive`(6.3 m, 왼쪽 출발): 가장 좋은 판 — 태그컷에서 포크 끝 좌우 −3.5 cm·방향 +0.3°, 눈 감고 3.1 m 동안 VIO 추정이 오른쪽으로 9 cm 흘러(1.7°) 입구면에서 `bay_side_clearance` 로 자동 정지. 실제로 오른쪽 꼬깔콘에 닿음 → **VIO 전파 추정이 맞았고 정지도 맞았음**. 남은 오차 = 눈 감는 3 m 의 방향 1~2°.
- 판 `164800_761b2a_drive`(5.9 m, 오른쪽 1.33 m·29°): 사이드스텝이 "태그컷까지 3 m 여유" 조건에 걸려 안 뜀 → 비스듬히 접근해 태그컷에 26~35° 를 남김 → 제자리 약한 회전 5 번(옆밀림으로 좌우 +0.25 → −0.16) → `transition_corrections_exhausted`. 계획기가 좌회전 15°·우회전 15°×2 지그재그로 옆밀림을 이용하려 한 것도 확인. **수정**: 사이드스텝 여유 3 → 1 m, 점수의 방향 가중치 15 → 60/rad(`plan_heading_weight`), 종착 좌우 조건 3 → 6 cm(`2×position_tolerance`), 보정 5 회 소진 → 한 번 후진(`corrections_exhausted_backing_once`) 후 재접근. 회전 상한 확대는 옆밀림(0.78 m × sin) 때문에 채택하지 않음.

- 판 `174331_7c04bd_drive`(6.4 m, 오른쪽 0.99 m·34°, 가중치 30): 사이드스텝 세 걸음은 목표대로(56°·0.89 m·90°)인데 순이동 0.33 m — 두 제자리 회전의 옆밀림(출발 각이 0 이 아니면 상쇄 안 됨)을 안 넣은 계산 실수. 태그컷에선 계획기가 "좌 3.7·좌 15·0.17 m·우 15·우 7.5·우 3.7" 같은 **옆밀림 이용 지그재그**를 내고 첫 걸음만 실행 → 재계획 반복 → 보정 소진 → 후진 → 재접근 → `no_feasible_progress`. 가중치 문제가 아니라 태그컷에 각을 남긴 채 도착하면 어느 회전도 답이 없는 구조.
- **정렬선 복원(v5 FINAL_STRAIGHT_M, 사용자 결정 C)**: `final_straight_m` 1.5 — 계획기는 정렬선(태그컷 1.5 m 앞)까지를 목표로 좌우·방향을 끝내고, 정렬선에서 컨트롤러가 예상 도착 ±3 cm 면 **1.5 m 를 곧게 한 번에**(태그 보며) 태그컷까지, 아니면 약한 조준 회전 한 번(최대 5 회, 소진 시 그냥 직진·기록) 후 재측정. 태그컷에선 제자리 회전 없음. **사이드스텝 이동량**은 회전 두 번의 옆밀림을 `moved()` 로 예측해 "돌아온 뒤 중심선에 오는 거리" 의 90 %. 시험 4 개 추가·4 개 갱신.

- 걸음이 짧아지던 원인(174331, 사용자 지적 "깔짝"): 0.8 m 걸음은 정지 명령 0.62~0.67 + 미끄러짐 0.11~0.16 = 0.74~0.80 으로 정상이었고, 0.17~0.4 m 는 계획기가 고른 짧은 다리. 그런데 174331 에선 **짧은 다리에서 배운 속도가 0.24 → 0.10 m/s 로 줄고 관성 시간이 0.5 → 1.36 s 로 불어** 예측 여유가 0.4~0.8 m 가 되어 0.76 m 걸음이 0.17 m 에서 서는 악순환(정속에 못 미친 다리로 progress/시간을 재면 속도가 낮게, residual/속도가 크게 나옴). 또 태그 자 도입 뒤 순간 속도 추정이 0.8 m/s 로 튐(태그 창 갱신 점프). **수정**: 학습은 0.4 m·1 s 이상 정속에 든 다리만, 관성 시간 상한 0.8 s; 순간 속도는 VIO 자로만 재고 0.45 m/s 로 클램프(정지 판단의 max 자는 유지). 시험 갱신.

## 2026-10-20 VIO 1~2 s 멈춤 추적 — 브리지 IMU 수신부(reliable 붙듦) → best-effort

- 증상: 책상 정지 measure 에서 VIO 출력이 0.4~1.9 s 비는 구간이 2~5 분에 6~18 번. 매번 브리지 `stereo_dropped` 가 25~54 장 뜀(= 그 시간의 영상 전부). 멈춤 순간 브리지는 영상은 방금 것까지 받았는데 마지막 IMU 는 0.7~1.3 s 뒤, IMU 큐 0, `imu_sequence_jumps`·`imu_gaps` 0 → IMU 가 빠진 게 아니라 늦게 몰아서 옴.
- 지운 후보(근거): ① SDK/USB — `scripts/check_imu_cadence.py`(SDK 만) IMU/+IR/+컬러 60~120 s 에서 0.5 s 초과 공백 0, 최악 0.26 s. ② 파이썬 콜백 — `health.imu_delivery` 콜백 간격 최대 0.026 s(5.5 분, 멈춤 18 번 모두 포함), 메인 루프 단계 최대 0.063 s. ③ 자이로·가속도 짝짓기(`stream.py drain`) — 같은 스크립트의 pairing wait 최대 35 ms, 가속도 시계 어긋남 ≤ 34 ms. ④ DDS 전송 방식 — `FASTDDS_BUILTIN_TRANSPORTS` LARGE_DATA / SHM / UDPv4(+소켓 8 MB) 2 분씩 `dropped` 114·148·155 (기준 ~180) → 무관. ⑤ 브리지 실행 루프·OpenVINS — 2 초 타이머 간격 최대 2.035 s, 큐 비어 있음. ⑥ **제3 구독자**(`ros2 topic hz`, 같은 토픽, 기본 QoS) 는 3 분 동안 IMU 간격 최대 0.078 s, 영상 0.17 s 로 멈춤 0 — 송신·DDS 는 제때 전달. 같은 시간 브리지는 1433 장 버림(구독자 둘 추가로 악화).
- 결론: 늦는 자리는 **브리지 프로세스 안의 IMU 수신부**. reliable 수신은 표본 하나가 빠지면 되받을 때까지 뒤를 붙드는데(순서 보장), 이 수신부에서만 그런 일이 생긴다(원인 미확정 — 아래 카운터로 확인).
- 수정(`bridge.cpp`, `quality_version` 2→3): `imu` 구독 reliable → **best-effort**(송신은 reliable 유지, 호환). 두 구독에 DDS message-lost 콜백(`imu_lost`·`stereo_lost`), 콜백 도착 간격 최대값(`imu_arrival_gap_max_s`·`image_arrival_gap_max_s`, 2 초 창) 을 `[pipeline]` 줄에 추가. 빠진 IMU 표본은 `imu_sequence_jumps`·`imu_gaps` 로 세고 OpenVINS 가 5 ms 구멍 위를 적분(이미 치명 아님). 정확도: 영상 1~2 s 를 통째로 버리던 것 대신 표본 한두 개 빠짐.
- **검증(Jetson, 08:45, `20261020_084543_ff8bab_measure`, 책상 정지 약 2.4 분, 화면 켬)**: `dropped` **0**(이전 2 분 114~155), VIO 출력 3763 개 ≈ 26 Hz 끊김 없음. `imu_lost` 20 · `imu_sequence_jumps` 11(27063 개 중 — 브리지 수신부에서 표본이 실제로 빠지고 있었음이 확인됨; `imu_gaps` 0 = 빠진 건 한두 개씩, 10 ms 구멍). 콜백 도착 간격 최대 IMU 0.076 s · 영상 0.092 s(2 초 창 최대들의 최대). 결론: reliable 수신이 빠진 표본을 되받느라 뒤를 붙들던 것이 멈춤의 기구였고, best-effort 로 사라졌다. 표본이 왜 수신부에서 빠지는지(FastDDS 내부)는 미확인 — 빈도 0.07 % 라 추적하지 않음.
- 같은 날: `motion_age_s`·`imu_age_s` 1→3 s(위 절). 테스트 140/136 통과.
- 로그 폴더 하루 단위(같은 날, 사용자 요청): `work/<YYYYMMDD>/<HHMMSS>_<id>_<명령>/`. 예전 `work/<YYYYMMDD_HHMMSS>_…` 는 그대로 읽힌다(폴더를 인자로 받는 명령뿐). README 의 "가장 최근 판" 명령은 `ls -td work/*/* | head -1`.
- 분석용 기록 보강(같은 날, 사용자 요청; 영상 저장은 용량 때문에 제외): ① 걸음마다 `step_done` 이벤트(명령·목표·정지 명령 때 진행 거리/남은 거리/예측 여유/관측 나이/관성 추정, 가라앉은 뒤 실제 진행 거리·미끄러짐·출발 지연, 그때의 학습값) — `control.jsonl` 의 `status.events`. ② `python -m docking note --folder <판> --lateral-cm --heading-deg --longitudinal-cm --memo` 가 줄자 실측을 `manual_measurement.json` 에 누적. 테스트 2개 추가.
- 장착각 파일 분리(같은 날, 사용자 결정): 측정 결과를 common.json 의 `mount` 필드 대신 **`configs/mount.json`**(Jetson 에만, 배포 제외, .gitignore)에 쓴다. common.json 은 맥이 유일한 주인이 되어 `update_jetson.sh` 가 매번 덮고(`--with-common` 옵션 삭제) 덮기 전 Jetson 것과의 diff 를 보여 준다. 옛 common.json 의 `mount` 값은 첫 배포 때 자동으로 mount.json 으로 옮긴다. `config.load` 는 mount.json 우선, 없으면 옛 필드. 테스트 갱신.
- 포크 정지 목표를 common.json 으로(같은 날, 사용자 요청): `site.target_front_x_central_m`(−0.5)·`target_front_x_left_m`(1.0). 코드 상수였던 값을 그대로 기본으로 두고, 항목이 없거나 null 이면 그 기본값(배포가 Jetson 의 common.json 을 덮지 않으므로). 화면은 central/left 를 상수 비교 대신 runtime 이 주는 `mode` 로 구분. 시험 3개 추가.\n- 이름 정리(같은 날, 사용자 요청): ROS 패키지 `docking_v6_msgs` → `docking_msgs`(폴더·CMake·package.xml·브리지 include·파이썬 import), 환경변수 `DOCK_V6_WS`/`DOCK_V6_ENV` → `DOCK_WS`/`DOCK_VENV`, 화면 제목 "Docking v6" → "Docking". `environment.sh`·`install_jetson.sh` 의 krri-v6/docking-v6 폴더 폴백 삭제(옛 폴더는 지워도 됨). `rebuild_openvins.sh` 가 워크스페이스의 옛 이름 산출물(src/build/install 의 docking_v6_msgs)을 한 번 지운다. 남은 v6 흔적은 `prepare_openvins.py` 의 마커 파일 이름 하나(바꾸면 OpenVINS 사본을 다시 생성해 긴 빌드가 나서 둠). 재빌드 필요. 테스트 140/136 통과.

## 2026-10-20 IMU 멈춤 허용 1 → 3 s

- 10/19 책상 측정(depth 켬 `20261019_233900_5139cf_measure` · 끔 `234703_b49e5f_measure`): IMU 표본이 1.4~1.7 s 멈췄다가 몰아서 오는 구간이 2~3 분에 10~16 번(0.4 s 초과 6~13 번, 시간 비율 3.9~6.8 %). depth 를 끄면 반으로 줄지만 최대 길이는 그대로. 원인은 SDK/USB 전달로 좁혀졌고 앱(GC·루프·카메라 전달)은 아님. 조사용 `scripts/check_imu_cadence.py`(SDK 만, IMU/+IR/+컬러 세 부하) 추가, `runtime.json depth_enabled` false.
- 주행 중 영향(코드 확인): 멈춤 < 1 s 는 걸음만 조금 짧게 끝남(예측 정지가 관측 나이를 여유에 더함). 1 s 초과는 `fast_imu_unavailable`·`tracking_lost_during_motion` 으로 중립 → RECOVER → 재계획(판은 안 끝남, 걸음 하나와 몇 초 손실). 탐색 회전 중만 FAULT(`search_tracking_lost_unmeasured_angle`) — 사용자: 탐색은 비상용이고 태그 보이게 시작하므로 수용.
- **사용자 결정("넉넉하게")**: `motion_age_s`·`imu_age_s` 1.0 → **3.0 s**(관측 최대 1.7 s 의 약 두 배). `lease_s` 1.0 은 유지(감시 루프는 안 멈춤). 눈 감는 거리 상한은 0.29 m/s × 3 s = 0.87 m 지만 예측 정지가 나이를 여유에 더하므로 늦게 서지 않는다. 시험 `test_vio_quality` 의 지연·IMU 나이 표본을 3.01 s·3.1 s 로.

## 통과한 검사

- `python -m unittest discover -s tests -q`: 2026-10-16 당시 **51개 통과**(2026-10-19 현재 161개, 위 절). 실제 장치 없이 합성 관측과 모의 CAN 전송을 사용했다.
- 지연된 태그의 촬영 시각별 재처리, 중복·이상값 거부, 상관된 프레임 반복으로 공분산이 0에 수렴하지 않는지 확인.
- 1.2m 실제 관측 이동과 태그 좌표 보정의 구분, VIO 원점의 불확실성 중복 계산 방지, 추적 연속성 상실 시 무효화.
- IMU 200Hz 표본 유지·가속도 보간·누락 감지, 정지 판단, 포크 끝 오차 전파·진입 거부, 준비/주행/정지 상태 전환.
- 장착 보정, 퇴화한 hand-eye 입력 거부, 렌즈/해상도/시리얼 바인딩, OpenCV가 생성된 OpenVINS YAML을 읽는지 확인.
- 정지 시 반복성과 실제 위치 편향을 구분하는 평가, camera-only의 주행 사용 거부.
- Python·셸 문법, JSON·ROS 패키지 XML, 실행 스크립트 도움말.
- v6 `canbus.py`, `canhealth.py`가 v5 원본과 바이트 단위로 동일함을 확인.
- 기존 v5/common/update 파일 및 보관한 공식 소스·참고 스냅샷 hash 유지 확인.
- OpenVINS 빌드 사본을 두 번 생성해 재실행 가능함을 확인. 알고리즘/헤더/데이터 109개 파일은 원본과 동일하며 생성 사본에 수동 변경이 있으면 중단한다.

## Jetson 설치 의존성 수정 — 2026-10-16

사용자 장비에서 NVIDIA `libopencv-dev` 4.8(우선순위 600)과 설치된 Ubuntu OpenCV 4.6(500)이 충돌함을 패키지 목록으로 확인했다. 설치 명령은 Ubuntu contrib의 4.6 버전과 개발/Python 패키지를 일치시키며, 전역 apt 우선순위는 바꾸지 않는다. 직접 수행하는 apt 설치에 `--no-remove`를 적용했다. [APT 공식 버전 지정·삭제 방지 옵션](https://manpages.ubuntu.com/manpages/noble/man8/apt-get.8.html).

수정 스크립트의 셸 문법 검사 및 모의 apt 입력 4개(4.6 선택, 후보 없음, 예상 밖 4.8 후보, apt 실패 전파)를 확인했다. 실제 패키지 설치·의존성 해결과 네이티브 빌드는 Jetson 재실행으로 확인해야 한다. 런타임 코드는 변경하지 않았다.

## 문서·전체 코드 복사 확인 — 2026-10-16

두 도식의 PNG/SVG를 현재 실행 경로에 맞춰 다시 생성하고 화면으로 확인했다. RTAB-Map 기록 경로와 미구현 ORB 자동 전환을 제어 보정과 구분했다.

루트 `update_jetson_all.sh`를 macOS 기본 rsync로 임시 로컬 대상에 시험했다(SSH만 대체). 미리보기 무변경, 첫 복사, 재복사에서 코드·문서·도식 포함 및 기존 common.json·로그·기존 폴더 보존을 확인했다. `/./` 접두부 처리에 의존하지 않고 상대 경로로 전달한다. 실제 Jetson 전송은 이 검사에 포함하지 않았다.

## 아직 실행하지 않은 검사

- Ubuntu 24.04 arm64 / ROS Jazzy에서 C++ 브리지 컴파일·링크 및 DDS 통신.
- 실제 RealSense의 IR stereo+color+depth+IMU 동시 수집, 공장 보정·타임스탬프·초기화.
- Jetson CPU/RAM/온도/처리 지연, RTAB-Map 지도 생성.
- Kvaser 실제 송수신, 차량 제동·진동·태그 소실 구간의 위치 정확도.

따라서 소프트웨어 모의 검사 통과를 실차 ±3cm 정확도나 설치 성공의 보증으로 해석하지 않는다. Jetson의 설치 스크립트는 네이티브 빌드·소프트웨어 검사를 수행한다. 이후 `run.md`의 measure → 현장 입력·mount → shadow/drive 순서로 실제 장치 동작을 확인한다. RTAB-Map은 지도 기록/후보 진단용이며 ORB-SLAM3 자동 전환은 구현하지 않았다.

### 2026-10-16 ROS manifest / RealSense extrinsics startup repair
- Replaced invalid `maintainer@localhost` with reserved placeholder `maintainer@example.com` in both local ROS manifests (not a real support address).
- Resolve requested calibration modes from a single SDK device instance, matching stream type/index/format/fps/video size. Avoid cross-instance extrinsics queries between independent pipeline resolutions. Keep IMU-before-video start order.
- Missing/ambiguous profiles or unavailable factory extrinsics still fail; errors name the transform. No fabricated calibration fallback.
- Existing 51 unit tests and 3 added profile-graph/error tests passed locally. `git diff --check` passed. Local environment has no catkin_pkg; actual rosdep/colcon validation and physical D435i validation remain on Jetson.

### 2026-10-16 bounded vibration-aware settling
- Startup-only 6s baseline, 0.25s median blocks, 2s post-neutral window; trend and block excursion reject drift/rebound. Tracking/IMU loss and discontinuities reset observation windows. Capture-time filtering excludes delayed pre-neutral frames.
- Learned vibration is bounded and retained as pose/yaw uncertainty; existing docking uncertainty/3cm gates and CAN bytes unchanged. Native OpenVINS initialization unchanged.
- Synthetic tests cover above-old-limit vibration, translation drift, rotation, excessive gyro/vibration, out-and-back rebound, neutral transitions, visual loss and gaps. These do not validate physical forklift vibration or certify stop detection.
- Common site values set to vehicle width 1.3m and temporary bay width 1.4m as requested.

### 2026-10-16 OpenVINS YAML aliases and video startup timeout
- Jetson log showed Tw/Ta/R_IMUtoACC/R_IMUtoGYRO matrix parse errors; shared identity list was emitted as YAML anchors/aliases unsupported by OpenCV FileStorage. Disabled aliases in the OpenCV YAML dumper. Regression now reads every IMU matrix element exactly as the native parser does; 19 estimation tests passed locally.
- Camera startup previously aborted on the first 1000ms frame wait. Capture now polls every 5ms with the configured 30s first-frame deadline and a 2s established-stream deadline, identifying the phase in the error. This does not establish that a USB/synchronization problem is resolved; physical retest required.
- Python/config generation changes only; installed native bridge does not need rebuilding for this fix.

### 2026-10-16 repeated stereo frames vs clock regression
- New Jetson screenshot reached `sensor_clock_reversed` after video startup; equality had been classified as clock reversal. Exact repeated stereo frame numbers AND timestamps are now skipped before VIO/clock processing and counted as `stereo_duplicates`.
- Counter resets, nonadvancing timestamps on different frame identities and actual clock regressions still fail with previous/current frame numbers and timestamps. Repeated-only delivery for 2s remains a video fault. No timestamp clamping or fabricated replacement times.
- Three transport regression tests passed, covering duplicate IR frames, genuine regression/reset, and diagnostic values. Physical reason for the reported timestamp fault remains to be verified from the next run.

### 2026-10-16 Jetson counter 5 -> 0 with increasing hardware time
- User supplied exact fault: left frame 5->0 while hardware timestamp 239.498699->239.669966. Previous counter monotonicity guard was too strict for this observed metadata behavior.
- Counter decrease/nonadvancement with strictly advancing hardware time is recorded as a metadata anomaly rather than a timing fault. SDK frame-gap counts are flagged unreliable after the first anomaly; capture time is not altered. Stereo skew, duplicate-frame handling and real timestamp reversal checks remain active.
- Four transport tests passed, including the reported sequence, repeated zero counters, duplicates, timestamp reversal and one-sided reversal. Camera module compiles and diff whitespace check passes. Physical verification remains required; no claim that all startup issues are resolved.

## 2026-10-16 VIO initialization, quality and startup diagnostics
- Identified a stationary startup path in pinned OpenVINS: `is_initialized_vio=true`, clone warmup returns before setting `timelastupdate`, then ZUPT keeps returning before the normal update end. Its public `initialized()` may stay false. Our adapter now exports the actual inertial initialization flag and separately validates timestamp, continued tracks and correction evidence; upstream files remain unchanged.
- Separated tracked features, actual qualifying correction evidence and local/docking eligibility. Count continuing IDs (not only new detections), MSCKF, existing accepted stereo SLAM landmarks and ZUPT. SLAM inference depends on the reviewed pinned UpdaterSLAM rejection-counter/completed-path contract, not merely the number of landmarks in the state.
- Recent correction (0.2 s) is diagnostic. Continuous valid tracking may bridge at most 1 s between verified corrections; no arbitrary retention of `ok` after lost tracks, stale IMU, delayed output, broken relative covariance or mismatched state time. Existing world/fork uncertainty gates remain active. This new 1 s limit is an engineering limit, not physically validated accuracy.
- Initial camera-only preview is excluded from control observation ordering/readiness. Native result watchdog starts at first stereo input, not before camera startup; more input alone cannot reset it. DDS discovery waits for the result publisher too. Native worker progress appears every 2 s in openvins.log and Python transport counts in pipeline_status.json; timeout includes those diagnostics.
- Added `scripts/rebuild_openvins.sh` for installed Jetsons (only messages/adapter, no apt/download). Message schema/version checks request this rebuild rather than silently accepting an old bridge.
- Python regression tests and standalone C++ adapter-quality tests cover intermittent corrections, accepted/rejected SLAM evidence, ZUPT-only readiness, lost tracks, old timestamps, stale corrections/IMU, output delay, unchanged uncertainty gating, startup timeout and both-way discovery. Full ROS/native compilation and RealSense/Jetson runs are not available on this Mac; user must copy, rebuild and run measure. Current intermittent native-output timeout is not proven resolved by these changes.

## 2026-10-16 build/startup recovery audit

- User log: `Clock skew detected` with approximately 1.79 billion seconds of future mtimes, then C++ `VioState` missing the new quality fields. This confirms a stale/mismatched generated interface; the pasted runtime used an unsuccessful build. The preceding generic `imu_clock_or_sample_gap` does not identify which sensor/condition failed, and is not explained by wall-clock correction alone.
- Rebuild checks the wall clock against source mtimes, preserves then recreates only the two adapter/message build/install trees, builds/verifies messages before the bridge, and leaves a runtime-blocking marker after any failure. Upstream build/source, logs and common remain intact. Full installer uses the same hygiene; no automatic date changes.
- Video polling now tolerates incomplete startup composite frames only within the original deadline; incomplete/repeated frames cannot reset freshness. Errors from capture/preview and ROS decoding/spin are surfaced. Partial ROS construction and repeated cleanup are handled, including native children after a launcher exit. Camera polling threads are joined before stream shutdown.
- IMU continuity remains fail-closed. Errors include stream, reason, device delta, host-arrival delta and paired count; per-stream state is saved. Actual USB/IMU failure cause remains unverified without new device logs.
- All 88 local tests passed (31.6 s), including standalone C++ quality checks. Regression covers future-dated stale build files, preserved upstream/source, failed-build blocking, partial video deadlines, IMU fault distinctions, ROS partial startup/cleanup and existing estimator/controller contracts. Python compilation and shell syntax checks passed. Local Mac tests do not compile ROS-generated packages or exercise RealSense USB hardware; Jetson rebuild and repeated sensor-only runs are still required.

## 2026-10-16 first accel sample jumps 26.031566 s

- Jetson rebuilt both packages successfully and passed software checks. Runtime then reported accel device 252.437301 -> 278.468867 s, host arrival delta 0.001871 s, paired_samples=0. This is a pre-estimator startup discontinuity, not evidence that 26 seconds elapsed in the running estimator. Stale SDK data/device startup discontinuity is plausible; exact SDK cause remains unverified.
- Added a pre-publication IMU window: both gyro and accel must span 0.5 s in device AND host arrival time, be fresh, and agree with the current video time. Only before this one-time commit, a discontinuity discards BOTH stream buffers and starts a new candidate window. No interpolation across the gap and no timestamps changed. After commit every prior gap/reversal/overflow check remains active, even before native inertial initialization succeeds.
- Warmup buffers are bounded to a recent window while waiting for video. Existing 30 s acquisition deadline bounds preparation; first native-output deadline starts only after actual stereo publication. Preview/tag processing continues during preparation without control readiness. Diagnostics preserve reset reasons/counts.
- All 94 local tests passed (33.8 s). Regression replays the exact reported discontinuity, rejects burst-only stale history and mismatched/stale video/IMU, checks both-buffer resets and bounded storage, and confirms a later 26 s gap still fails. Python compilation and diff checks passed. Physical USB testing remains required. Python-only change: no further native rebuild needed after the user's successful rebuild.

### 2026-10-16 upright-tag coordinate regression and stationary sampling
- Native pupil AprilTag corners + IPPE give frontal R_camera_tag=diag(-1,-1,1).
  Share the corresponding dock tag rotation between config and mounting; the previous
  outward-normal convention produced a ~180-degree heading and reversed entrance side.
- An independently generated upright tag passed through the native detector now yields
  near-zero heading and a camera position before the entrance (Mac synthetic test,
  not a physical accuracy validation). Full suite: 97 tests, 1 native dependency skip;
  native tag regression separately passed in the krri environment.
- Stationary block medians use 0.5 s instead of 0.25 s: still require 3 samples/block,
  so 14 Hz outputs no longer fail merely from bin occupancy. Motion, rebound, gaps,
  gyro, and uncertainty limits remain enforced. 14 Hz rest/drift and sparse-input
  regression tests pass. Mount reports now include stationarity reason counts.
- Hardware remeasurement required. Reference mount still assumes independently
  aligned vehicle heading; it cannot separate unknown vehicle yaw from camera yaw.

### Mount-only settling rechecks
Mount calibration accumulates 60 s of confirmed stationary intervals rather than
requiring an uninterrupted boolean flag. A settling recheck can bridge at most
3 s only with valid local tracking, unchanged sensor/motion epochs, observation
gaps <=0.4 s, and accepted tag observations throughout. Every observed tag pose
in the bridge must stay within 1 cm / 0.5 deg of the preceding stationary pose
(engineering gates, not certified bounds). Recheck poses are excluded from the
mount fit and recheck time is not counted. Real rebound, unavailable tracking,
and incomplete intervals remain failures. Runtime drive settling rules unchanged.

### Isolated IMU gaps and preparation recovery (2026-10-16)
- Camera opts into isolated positive IMU gaps <=100 ms (normal gap gate remains
  50 ms). More than one such event per device second, larger gaps, invalid data,
  and clock reversals remain faults. Gyro samples retain their actual timestamps;
  accel interpolation remains bracketed, never extrapolated. The observer adds
  3 cm / 2 degree engineering uncertainty per event to cumulative drift accounting.
  These are unvalidated floors, not measured error bounds. Existing freshness,
  correction, and docking confidence gates can still stop the vehicle.
- Measurement-only IMU gap faults restart the entire attempt after cleanup, up
  to 3 retries, with separate log folders and cleared mounting collection. Drive
  and shadow never use this restart wrapper; fault cleanup still aborts CAN.
- Preparation accumulates confirmed intervals only. At most 3 s of fresh local
  settling with unchanged epochs and <=4 mm / 0.35 deg displacement can preserve
  previous preparation time. Movement/stale tracking discards it. Enter is not
  latched during rechecking. Post-stop 20 s timeout now enters stopped recovery,
  whose existing 60 s deadline is measured from the original stop.
- Regression fixtures now provide actual periodic observations during preparation
  instead of two samples 1.1 seconds apart. Synthetic gap integration validates
  timestamp continuity only, NOT OpenVINS accuracy on vibrating hardware. Native
  OpenVINS/vehicle validation remains necessary; CAN bytes were not changed.

### Straight calibration segment recovery
- Collection now clears the active leg and requires a new observed 3 s stationary
  start after tracking epoch changes, observation gaps, invalid IMU or prolonged
  tracking degradation. The CLI explicitly instructs the operator to stop and
  announces readiness for a NEW manual leg; no CAN commands are issued.
- Only correction_stale intervals <=0.5 s with fresh accepted tag geometry and
  available IMU may bridge local_valid=false. Other failures are not waived.
- Start/end rest is checked directly using fresh unique tag samples over 3 s,
  <=1 cm position deviation, <=0.5 deg orientation change and <=1 deg/s gyro.
  These are engineering gates, not an accuracy certificate. Rebound remains tested.
- Durable observations are replayed through the same segment collector before
  saving, avoiding concatenation of failed legs. Drive quality gates unchanged.
- Eight synthetic straight-calibration regressions cover oblique mounting,
  pitch/roll, brief degradation, hard resets with a successful later leg,
  insufficient travel, reversal, motion, stale/missing data and settling flags.

2026-10-16 CAN 직진 mount: Enter 후 정지 30초 → forward 3초 → 정지 안정 3초
(최대 60초) → 두 정지 위치 중앙값으로 장착 방향 저장. 감독 루프와 독립 CAN
프로세스에 전진 종료 시각을 전달하며 기존 lease 만료·abort 보호는 유지한다.
측정 전용 재초기화 루프가 자동 전진을 반복하지 않도록 제외했다.
전체 테스트 120개 통과(1 skip); 이후 CAN 종료시각 수정에 CAN 3개 및 pulse 6개 테스트 통과.
실제 Jetson/RealSense/CAN 차량 검증은 아직 수행하지 않았다.

CAN 수신 오류 진단 개선: error frame / RX overrun / 동시 발생을 fault_code로 분리.
원본 플래그(정수/hex), 프레임 수신 시각, poll 간격, pending을 보존한다.
오류 시 abort 및 중립 전송 시도 후에만 readStatus/read_error_counters를 읽고,
캐시된 드라이버 상태일 수 있음을 after_stop_snapshot에 명시한다.
can_fault.json에 최종 진단을 별도 기록한다. CAN 바이트·보호 임계값은 변경하지 않았다.
모의 transport에서 세 오류 종류, 정상 TXACK, 외부 송신자, bus-off, ACK 누락,
오류 후 중립 전송 및 중립 이후 상태 읽기를 검사했다. 실제 현장 원인은 아직 미확정이다.

v4 CAN 비교 및 송신 묶음 수정:
- channel 0, 500 kbit/s, 표준 ID, 전진 67, 제어 템플릿/카운터,
  정규 이동 10ms/제어 5ms/heartbeat 200ms는 동일했다.
- 시작 5회, 정지→이동 진입 5회, 기타 명령 전환 1회에
  v4의 control→heartbeat→movement 순서를 적용했다. 종료 중립 3회도 같은 묶음이다.
- v4의 명령 변경 writeSync(100)는 그대로 둔다(v4와 같음). 그 100 ms 동안 lease/abort 검사가 멈추는 것도 v4와 같다.
- 2026-10-18 사용자 결정: **v4처럼 송신 전용.** 수신 큐를 읽지 않고 TXACK를 켜지 않는다. 에러 프레임·overrun·TXACK 지연·외부 0x1E3
  검사는 없앴다(2026-10-16 Jetson에서 에러 프레임 1개로 두 번 멈췄고, v4는 같은 버스에서 그것을 보지 않은 채 주행했다).
  버스 상태·오류 카운터는 v4처럼 5초마다 로그에만 남긴다. 남는 정지 조건: lease(1.0 s, v4 DEADMAN_S) 만료·쓰기 실패·프로세스 종료·SIGTERM·부모 종료.
- 2026-10-01 v4 기록: 이동 프레임 간격 중앙 10.4 ms, 최대 0.44~1.04 s(VM 멈춤)에도 차는 정상 주행 — 0.15 s 쓰기 간격·0.3 s lease는 필요 이상으로 엄격했다.
- CAN/보정 모의 테스트 11개 통과. 시작·전진 5개 묶음, 종료 3개 묶음 순서와
  오류/lease 종료 시 중립만 송신하는 동작을 확인했다.
