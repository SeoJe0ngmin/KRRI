# v6 구현 검사 기록

2026-10-16, macOS arm64. Python 3.10.19 / NumPy 1.26.4 / OpenCV 4.12.

## 통과한 검사

- `python -m unittest discover -s tests -q`: **51개 통과**. 실제 장치 없이 합성 관측과 모의 CAN 전송을 사용했다.
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
- v4의 명령 변경 writeSync(100)는 복사하지 않았다. 별도 프로세스의 lease/3초
  종료 검사를 막지 않도록 기존 비동기 TXACK 확인을 유지한다.
- v4는 수신 error frame을 검사하지 않았다. v6의 오류 프레임 정지 및 진단은 유지한다.
- 이 차이가 현장의 error frame 원인이라는 증거는 아직 없다. 하드웨어 수정 완료 판정 아님.
- CAN/보정 모의 테스트 11개 통과. 시작·전진 5개 묶음, 종료 3개 묶음 순서와
  오류/lease 종료 시 중립만 송신하는 동작을 확인했다.
