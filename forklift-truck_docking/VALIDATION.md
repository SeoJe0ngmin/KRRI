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
