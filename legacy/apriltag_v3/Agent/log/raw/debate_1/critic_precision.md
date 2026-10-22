### 쟁점 1: lateral 재구성 — scalar 식 vs 명시적 좌표변환
- 판정: 부분동의 — 명시적 변환·`d·sin(ψ̂−β)` 오라클 단위테스트·검증 (a)(b) 는 동의. **"부호 충돌" 진단과 A=1.46 전제는 반박.**
- 반박 근거: ① 기하 유도(태그프레임 광축 u=(sin h, cos h), 피벗이 카메라 뒤 A)는 코드 부호 `lat_p = lateral − A·sin h`(`/Users/seojeongmin/krri/apriltag_v3/src/models/control/control_from_pose.py:93`)를 준다. ② proposer 가 근거로 든 `config/control.py:64` 두 점은 18:14 `measure.jsonl` #5→#7(tilt 2.9~4.4°, 정면원뿔) 값. 같은 로그 tilt 23~28° 구간(#8→#11, IMU 회전 −6.7/+5.1/−2.3°)은 코드 부호로 lat_p = 1.005/1.005/1.015/0.991 m(±12 mm), 반대 부호면 −0.16~−0.37 m 요동 → 정면 두 점으로 부호를 뒤집는 것은 증거 위계 오류. ③ 더 근본: A=1.46 자체가 미확정. R 오차와 무관한 방위각 β_i = ψ_i + atan(lat_i/fwd_i)(=atan2(t_x,t_z))로 18:14 대회전 4건(IMU +9.7/−12.6/+24.6/−21.7°)을 보면 Δβ/Δψ = 0.88~0.99. 카메라가 회전중심 앞 1.46 m 면 1+A/d = 1.16(9 m)~1.43(3.4 m) 이어야 하고, #12→#13 만 봐도 예측 Δβ 33~35° vs 관측 21.6°. 로그는 |A| ≲ 0.4 m(카메라≈회전중심)를 가리킨다. 1.46 의 출처(`/Users/seojeongmin/krri/apriltag_v2/work_dirs/오차보정.txt` can_pulse 18:43)는 8 m·자이로 정지(yaw 0.00·rate 0.07 고정)·1회·Δlateral 0.15 m(8 m 잡음 수준) 값.
- 대안: A 를 상수 아닌 **추정 파라미터**로. 첫 실차: tilt≥20°·d 3~4 m 에서 정지→IMU 회전 15~25°→정지 ×5, 프레임별 (β_px, ψ_IMU) 회귀 Δβ = (1+A/d)·Δψ 로 A 와 부호를 동시 확정(카메라 ψ·lateral 미사용). 확정 전엔 A∈[−0.5, 1.5] 사전분포·σ_A 를 P 에 전파하고 컨트롤러는 lat_p 대신 β 로 조준. 변환 구현·오라클 테스트는 proposer 안대로.
- 대안 근거: β 는 픽셀급·R 무관이라 heading 바이어스에 안 오염되는 유일한 회전 관측량. v2 18:14 는 A=0 으로 돌았고(decision #5: dist_t 0.907 = hypot(0.046, 0.906) → lat_p=lateral) T 방위각 7°/15° 점프의 절반 이상은 정면 heading 과대변화(−1.1/−1.8°)×d; 여기에 A=1.46 항을 얹으면 회전 1° 당 25 mm 유령 lateral 이 추가된다(critic §2 가 경고한 "숨은 부호·상수 오류" 의 실례).
- 실차 검증법: 위 회전 5회 데이터로 기울기 피팅 — 예측 1.4(후방 1.46) vs 0.9(로그). 같은 데이터로 IMU `turned` 와 회전 후 카메라 Δψ 의 어긋남(18:14 11/11건, 0.3~2.3°) 원인 분리: 정지 판정 뒤 3 s 자이로를 더 적분해 스프링백/조기샘플/카메라 바이어스 중 무엇인지.

### 쟁점 2: bearing β — PnP atan2 vs 태그중심 픽셀
- 판정: 동의(B 주채택·A 대조) — 단, 실질 차이는 **왜곡 보정**이고 그 전제가 코드에서 안 지켜진다.
- 반박 근거: A·B 는 수학적으로 거의 같은 정보(t 방향 = 호모그래피 중심 광선)라 "PnP 독립" 이득은 작다. 진짜 차이는 왜곡: `detect()` 는 원 영상에 (fx,fy,cx,cy)만 넘겨 라이브러리 자세를 얻고(`/Users/seojeongmin/krri/apriltag_v3/src/models/detection/detection_tag.py:76-78`), `pose_by_pnp`·`heading_sigma_deg` 만 `intrinsics.distortion` 을 쓴다 — 두 경로가 다른 카메라 모델. `CameraIntrinsics.undistort` 는 호출자 0. RealSense 컬러 계수는 `inverse_brown_conrady` 모델로 오는데 `image.py:27` 은 정방향 radtan 으로 가정 — 계수≠0 이면 `undistortPoints` 가 왜곡을 거꾸로 넣는다. 롤링셔터 0.1~0.2° 는 과대(태그 120행 판독 ~2 ms → 8°/s 에서 0.02°); 대신 T 에서 태그가 화면 상단(margin 31~97 px, `decision.jsonl`)이라 방사왜곡 최대 영역이다.
- 대안: `i.model`·`coeffs` 를 로그·`config.json` 에 남기고, 계수≠0 이면 모델에 맞게 코너를 보정한 뒤 β 와 PnP 모두 같은 보정 코너로. β_B 채택·β_tag/β_T 분리 출력은 그대로.
- 대안 근거: 3.4 m 에서 코너 1 px = heading 7.6°(proposer 의 CRLB 식) — 상단 1~2 px 방사왜곡이 정면 heading 바이어스(쟁점 3) 후보. Abbas 2019(화상 중앙 유지), critic §3.
- 실차 검증법: 정적 3.5 m·태그 상단 배치에서 왜곡 보정 전/후 heading 100프레임 비교; 차이 >1° 면 왜곡이 바이어스 원인.

### 쟁점 3: heading PnP 2중해 처리
- 판정: 부분동의 — 두 해 명시 산출·pitch-cue(기하 유도상 틀린 해 pitch≈2ε 맞음)·3상태·ψ̂ 무관 단서로 재초기화 동의. **tilt 원뿔 게이트를 빼고 "정면도 판정 가능" 으로 간 것은 반박.**
- 반박 근거: 18:14 정면원뿔(tilt 3~4°, d 3.3~3.9 m) 측정은 틀린 branch 가 아니다(틀린 해면 tilt ≥2ε≈36°, 보고값 3~4°). 그런데 회전 전후 카메라 Δψ 가 IMU Δψ 와 11/11건 어긋남: 정면→정면 4건은 카메라가 13~25% 과대(+0.7~1.8°), oblique 종단 7건은 0.3~2.3° 과소 → 3.4~3.9 m 에서 lateral 40~135 mm, 30프레임 spread 는 0.02~0.09°. 즉 **옳은 branch 의 매끈한 yaw 바이어스**가 지배 — pitch-cue·err 비·마할라노비스 모두 통과시킨다(Rijlaarsdam 2022: 3.3 m ±10° 구별 불가). 또 pupil_apriltags `pose_err` 는 객체공간 m²(≈1e-6)라 `pose_quality` 의 2 px 게이트(`detection_pose.py:363`)는 AT3 경로에서 무효.
- 대안: heading_valid = **기하 원뿔(|LOS 대비 태그 법선각| ≥ θ_min(d), 초기 15°)** AND pitch-cue AND err비 — 원뿔은 하드, 나머지 소프트. 정면 구간 heading 은 마지막 oblique 앵커 + 자이로(PLAN ③·설계전제 "짧은 다리"). AMBIGUOUS 지속 시 타임아웃 → fallback(정지 후 oblique 재획득) 을 상태로 명시.
- 대안 근거: Richter 2022 정면 ±25° 회피, PLAN_legacy ③-①, critic §5·§19(대기 데드락). 바이어스는 σ 도 branch 도 아니어서 게이트할 수 있는 건 기하뿐.
- 실차 검증법: 정적 3.5 m, 바닥 기준선으로 진값 heading 0/±3/±6/±10/±15/±25° × 100프레임 — 두 해 pitch 히스토그램 + 옳은 해 yaw 바이어스 곡선; 바이어스 >1° 인 각 범위가 θ_min.

### 쟁점 4: 기하 관측성/신뢰도
- 판정: 부분동의 — 해석식 σ_ψ ∝ σ_c·fx/tag_px²·연속값 출력·MC 오프라인 격하 동의. **"식 + 플래그" 로 시작하면 정렬점에서 false confidence** 라는 점 반박.
- 반박 근거: 식의 3.4 m 30프레임 heading σ 는 0.2°(σ_c 0.15) → lateral 12 mm 인데 18:14 실측(쟁점 3)은 같은 자리에서 0.7~2.3° 바이어스 → 식의 3~10배, 그리고 정적 장면에서 일관되므로 σ_c 스케일링으론 안 잡힌다. σ_c 자체도 정지 0.07 px(can_pulse 8 m 정지 forward σ 12 mm 역산, `CORNER_NOISE_PX` 와 일치)·주행 중 ~0.3 px(같은 파일 forward 3 s 잔차 ~50 mm)로 4배 차 — "0.07 낙관" 은 주행 한정.
- 대안: σ_ψ,eff² = σ_random²(해석식, σ_c 정지/주행 2값) + b²(d, tilt)(정적 격자에서 잰 바이어스 상한, 관측 시간으로 안 줄어드는 floor). heading_valid 는 쟁점 3 의 기하 원뿔.
- 대안 근거: critic §6 "uncertainty≠오차"; Adámek 2023 은 정면 피크를 분산으로 두지만 우리 로그는 바이어스 — 바이어스를 σ 로 넣으면 √N 으로 잘못 줄어든다.
- 실차 검증법: 쟁점 3 정적 격자에서 |median − truth| 를 (d, tilt) 표로; 30프레임 spread 대비 비율 ≫1 이면 floor 채택.

### 쟁점 5: uncertainty ≠ 실제 오차 — calibration
- 판정: 부분동의 — 3검사·σ_floor·컨트롤러 계약(|e| > T + k·σ_eff 지속) 동의. **NIS 를 방어선으로 세운 점과 바이어스 검사 설계는 반박.**
- 반박 근거: NIS 는 예측–측정 불일치만 본다; 정적 장면의 매끈한 바이어스는 상태가 흡수해 NIS≈1 — proposer 가 스스로 든 worst(매끈하게 틀림)에 무력(critic §24 Failure 2). (3) 직진 lateral 기울기로 CAM_YAW_OFFSET 을 heading 0(정면)에서 재면 정면 yaw 바이어스와 오프셋이 섞인다. 그리고 회전 뒤 카메라–IMU 1~2° 차이를 dynamics 층이 이미 "회전 잔차 ±1.5~2°"(`config/control.py:70,75`)로 흡수 중 → 같은 오차의 이중 해석(critic §21 유형).
- 대안: 1차 캘리브레이션을 **교차센서 일관성**으로: (a) 회전마다 Δβ vs Δψ_IMU(A·배율), (b) 회전 전후 Δψ_cam vs Δψ_IMU(바이어스·스프링백), (c) 직진은 oblique(ψ_cam≈25°, d 6→4 m)에서 Δlat/Δs = sin(ψ_cam − δ) 로 δ. 정적 진값 검사는 그 다음, 커버리지 c 는 축별 하나가 아니라 (d, tilt) 셀별.
- 대안 근거: 두 센서가 공유하지 않는 오차만 검출 가능(Bar-Shalom C2 일관성 원리). 18:14 로그로 이미 (a)(b) 가 계산 가능했고 A 문제와 heading 바이어스 둘을 잡아냈다.
- 실차 검증법: 쟁점 1 회전 5회 + oblique 직진 2회(5→3.5 m) 한 세트에서 A, δ, 카메라–IMU 차 세 값이 나오고 재현되는지(회전 5회 중 4회 이상 같은 부호·±0.5°).

### 쟁점 6: 지연 보정 — timestamp·전파
- 판정: 동의 — 단일 시계(epoch)·t_capture 되감기·SE(2) 두 경우 퇴화·명령지연은 dynamics 층·상호상관 실험 모두 동의. 세부 셋 수정.
- 반박 근거: (i) "camera.py:219-236 이 global_time 을 켬" 은 사실이 아니다 — `set_global_time` 은 호출자 0, `CameraSettings.docking()` 의 `global_time_enabled=True` 도 `tune=` 을 줄 때만 적용되고 `run.py:181` 은 `from_realsense(tag_size, label=...)` 만 부른다. SDK 기본값이 켜져 있어 동작할 뿐 → assert 는 필수(맞음). (ii) 18:14 measure 구간은 30프레임에 1.30 s(≈23 fps, `rotation.jsonl`→`measure.jsonl` ts 차) — 소비가 30 fps 미달이라 ~25% 프레임이 빠지고 dt 가 불규칙; `image.py:415` "큐가 밀리지 않음" 주석은 검증된 적 없다. (iii) SE(2) 전파의 `카메라점 = pivot + A·u(ψ)` 는 A 가 틀리면 회전 중 예측이 A_err·Δψ 만큼 틀려 재포착 innovation 으로 튄다 — 쟁점 1 선행조건.
- 대안: proposer 안 + 프레임마다 (t_capture, t_arrival, t_detect_done) 로 큐 적체와 처리시간을 분리 기록; 처리가 30 fps 미달이면 최신 프레임만 쓰는 정책(frames_queue_size=1 또는 드레인) 명시. A 는 파라미터.
- 대안 근거: critic §7·§21, PX4 EKF2 지연 지평(C3), 로그의 23 fps 실측.
- 실차 검증법: 8°/s 회전 상호상관(proposer 안) + 정지 60 s 에서 t_arrival − t_capture 추세(자라면 적체) — 두 값으로 τ_cam 과 큐 문제가 분리된다.

### 쟁점 7: 주행 중 연속 추정기 구조
- 판정: 부분동의 — heading KF [ψ, b_g]·게이트 갱신·위치 KF·Q 스케줄·v̂ 재평가(d 로 재야 하고 병목은 σ_τ) 동의. **회전 중 predict-only 전제와 재포착 게이트에 결함.**
- 반박 근거: (i) "pivot 불변" 예측이 A=1.46 에 묶임(쟁점 1) — 24° 회전 뒤 예측 카메라점 0.6 m 오차 → 재포착 시 마할라노비스 기각→AMBIGUOUS 지속→wait deadlock(critic §19), 수용하면 lateral 점프. (ii) 18:14 에서 회전마다 카메라 최종 heading 이 IMU `turned` 와 0.3~2.3° 어긋남(11/11건) — 스프링백이면 자이로가 계속 적분해 KF 는 따라가지만 `rotate_to` 정지판정 뒤 샘플만으론 못 본다; 카메라 바이어스면 정면 구간에서 못 고친다. 어느 쪽이든 재포착 innovation 1~2°(3.4 m·60~120 mm)를 σ 0.05° 짜리 게이트가 매번 기각. (iii) [x_f, v]+[y_l] 분리 모델은 대각 주행(ψ 25°) 중 y_l 이 지연 — 유니사이클 ẋ=v cos ψ, ẏ=v sin ψ 결합이 필요. (iv) `rate_dps` 는 마지막 샘플에 얼어붙는다(`/Users/seojeongmin/krri/apriltag_v3/src/utils/imu_yaw.py:301` docstring; can_pulse 18:43 이 정지 자이로 0.07°/s 로 5° 회전을 "안 돌았다" 오판) — KF 예측 입력에 alive/gaps 게이트 필수.
- 대안: 상태 [x_p, y_p, v] + 캐스케이드 ψ(자이로), 유니사이클 전파; 회전 종료 후 oblique 재획득에서 ψ 를 카메라로 **재앵커**(N프레임, 게이트 σ 는 쟁점 4 floor 포함); A 는 파라미터; 자이로 stale 시 예측 정지·AMBIGUOUS.
- 대안 근거: Hurák C4·PX4 C3 캐스케이드, Liu B8, PLAN_legacy ③ 재초기화 규칙, 18:14 실측 11건.
- 실차 검증법: 정속 직진 bag 에서 v̂ 1 s std ≤ 5 mm/s(proposer 안) + 회전 5회 후 재포착 innovation 분포(예측: floor 미포함이면 5회 모두 3σ 밖).

### 가장 위험한 제안 TOP2
1. 쟁점 1 — A=1.46·부호를 "회전 펄스 1회" 로 확정해 T_tag←pivot 에 박는 것: 로그는 A 미확정(대회전 4건 Δβ/Δψ 0.88~0.99, 후방 1.46 이면 1.16~1.43)이고, 정면원뿔에서 실험하면 틀린 부호가 "확정" 된다 → 회전 1° 당 25 mm 유령 lateral 로 T 회전 반복 재현.
2. 쟁점 3+4 — tilt 원뿔 게이트를 빼고 pitch-cue·err비·해석식 σ 로 정면 heading 을 CONFIDENT 통과시키는 것: 18:14 의 지배 오차(옳은 branch 의 0.7~2.3° 매끈한 바이어스, spread 0.02~0.09°, lateral 40~135 mm)를 셋 다 못 잡아 false confidence 로 정렬(critic Failure 2).