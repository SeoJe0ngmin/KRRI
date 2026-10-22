### 쟁점 1: lateral 재구성 — scalar 식 vs 명시적 좌표변환
- 판정: 부분동의 — 명시적 T + 단위테스트 오라클은 동의. "부호 충돌" 은 실재하나 제안의 해석(코드 부호 or heading 규약 중 하나)과 오라클 식은 수정 필요.
- 반박 근거: 기하 정리 `pivot_x = lateral − A·sin ψ` 에서 A 는 **부호 있는** "pivot→카메라 종방향 거리" 다(카메라 = pivot + A·u(ψ), heading·lateral 둘 다 같은 태그 x축으로 재므로 규약 무관). 18:14 로그(rotate_ccw, heading −4.5°/lateral +112 mm)와 정합하는 시나리오는 둘: {실제 CW 회전 ∧ pivot 이 카메라 **앞**(A<0)} 또는 {CCW ∧ 로그 heading 부호가 틀림 — 9/7 결론의 flip 오염(A21 Springer&Kyas), 혹은 `/Users/seojeongmin/krri/apriltag_v3/config/imu.py:25-27` 이 기록한 IMU_YAW_SIGN 반전 이력}. 전자면 `fwd_p = forward + A·cos h`(`control_from_pose.py:94`)도 함께 틀리는데 제안은 lat_p 만 본다. 또 방법론 식 `d·sin(ψ̂−β)` 는 이 코드 규약에서 **부호 반대**: `detection_pose.py:51-53` 예제로 1.432·sin(2.05°−6.21°) = −0.104 인데 lateral=+0.104 → "<1 mm 일치" 오라클은 첫 실행에 실패하고 코드 부호를 뒤집게 유도한다(critic §2 가 경고한 함정).
- 대안: 오라클은 `lateral = d_h·sin(β−ψ)`, d_h=√(t_x²+t_z²)(|t| 아님), 허용 ±5 mm(R_mount≠I 이면 1 mm 는 안 나옴; 목적은 부호). A 를 부호 있는 상수로 두고 줄자로 카메라→구동축 부호 실측을 2차 증인으로. T_cam←pivot 은 (0,0,−A) 아닌 3-벡터(카메라–포크중심 횡 오프셋 포함). 검증 (c) 는 camera heading 대신 **자이로 Δψ** 기준.
- 대안 근거: critic §2·§24 Failure 1; Liu 2021(A22/B8: 이동 중 틀린 해가 재투영 최소) — camera heading 을 기준 삼은 (c) 는 flip 이 교란변수.
- 실차 검증법: 정지→rotate_ccw 펄스 1회(5~8°), 200 Hz 자이로 + 프레임별 lateral/heading 기록 → (i) sign(Δlat_cam)·sign(Δψ_gyro): + 면 pivot 뒤(코드 유지), − 면 pivot 앞(lat_p·fwd_p 둘 다 부호 반전); (ii) sign(Δhead_cam) vs Δψ_gyro 불일치·양봉이면 로그 heading 이 flip 이었다는 증명; (iii) |Δlat|/sin|Δψ_gyro| ≈ 1.46 재확인.

### 쟁점 2: bearing β — PnP atan2 vs 태그중심 픽셀
- 판정: 동의(B 채택) — 채택 조건과 지배 오차원은 수정.
- 반박 근거: 약원근에서 두 IPPE 해는 중심(t 방향)을 공유해 β_A 도 flip 에 거의 무관 → "β_A 양봉이 보이면 채택" 은 아마 관측되지 않고 B 채택 근거가 흔들린다. β 의 실제 적은 잡음(<1 mm, 계산 맞음)이 아니라 상수 바이어스: cx/장착 yaw(3 px=0.13°, CAM_YAW_OFFSET 과 구별 불가) 와 **롤** — R_mount 롤 오차 δφ → lateral 바이어스 1.1 m×δφ = **19 mm/°, 거리 무관**. 가속도계 영점 오프셋(수십 mg ≈ 1° 급)으로 잡은 R_mount 는 그만큼 틀린다. 롤링셔터는 "왜곡" 보다 **행 시각** 문제(태그 중심 행 = SOF + row/H·t_readout, 최대 ~30 ms → 8°/s 에서 0.24°).
- 대안: B 를 PnP 유효성과 무관하게 무조건(quality 실패 프레임에도 β 는 산출). 롤은 태그 상하변 방향(`pose_to_forklift` roll, 픽셀 정밀) 30 프레임 평균으로, 가속도계는 pitch 검증용으로 격하. 회전 중 β 는 행 시각 보정 후 자이로 되감기(쟁점 6) 에 태운다. 상수 바이어스는 쟁점 5(3) 직진 실험에서 CAM_YAW_OFFSET·횡 오프셋으로 흡수.
- 대안 근거: Abbas 2019(A20/B2) 화상 중앙; critic §3; Rijlaarsdam(B3)·Liu(B8) 의 2중해 모델(법선 거울상·중심 공유).
- 실차 검증법: 정적 5 m 100 프레임 β_A/β_B std·차이 + 카메라를 의도적으로 롤 2° 기울여 lateral 이 ≈38 mm 이동하는지(1.1 m×δφ 예측) → 롤 보정 필요성과 출처(태그 vs 가속도계) 한 번에 확정.

### 쟁점 3: heading PnP 2중해 처리
- 판정: 동의(pitch-cue 는 이 토론 최고의 통찰 — 최종 정면 자세(β≈0, ψ≈0) 를 유일하게 살리는 관측 단서) / 부분반박: 탈출 규칙·초기화·하드웨어 방향.
- 반박 근거: (a) "15 프레임 연속 반대" 는 "log-odds 누적" 과 모순이고 8 m 에선 옳은 해도 σ_pitch≈6.3° 라 6° 게이트를 34% 실패 → 연속 15 는 0.66¹⁵≈0.2% 로 영영 안 탈출(5 m 에선 반대로 즉시). (b) "태그를 15° 아래로 기울여" — 태그 면이 카메라 쪽(아래)을 향하면 법선–시선 각이 |ε−15°| 로 **줄어** d=1.1/tan15°≈4.1 m(= T 구간) 에서 완전 정면 → 모호성 최대. (c) cue 의 힘 2·atan(h/d) 은 h=1.1 m 에 구조 의존인데 h 는 `config/control.py:61` 주석에만 있다. (d) 첫 CONFIDENT 프레임이 ψ̂ 초기값이면 오분류 1~6% 가 lock-in 출발점. (e) 5 m 에서 틀린 해 잔차 ≈0.5~1 px vs 옳은 해 0.15 px → err 비 0.15~0.3, 0.2 게이트는 경계선.
- 대안: 탈출은 Wald SPRT — 프레임별 log[N(pitch; pitch_mount, σ)/N(pitch; 2ε, σ)] 누적, ±임계 도달 시 판정(연속 요구 없음, 8 m 도 수십 프레임이면 결론). 초기화/재초기화는 30 프레임 **다수결**. 하드웨어는 태그 면을 위쪽(카메라 반대) 15° 또는 좌우 yaw 15~20°(PhotonVision B10 "비스듬히", Richter A9 ±25° 원뿔 회피) — yaw 안은 최종 자세가 정면이 아니게 되어 근본 해결. 장착 조건 "h ≥ 0.6 m(3.5 m 에서 ε≥10°)" 를 설계 상수로.
- 대안 근거: Liu 2021(B8) 이동 중 재투영 기준 실패; Springer&Kyas(B9) 부호반전∧각속도 대조; critic §4·§24 Failure 1; SPRT 는 "연속 N" 의 정식 대체.
- 실차 검증법: 정적 5/8 m ×100 프레임에서 solvePnPGeneric(IPPE_SQUARE) 두 해의 pitch 히스토그램 → 봉우리 {pitch_mount, 2·atan(1.1/d)} 와 각 봉우리 σ 실측 → 게이트 폭·SPRT 임계를 그 σ 로(6° 고정 아님); 동시에 err 비 분포로 0.2 초과 빈도.

### 쟁점 4: 기하 관측성/신뢰도
- 판정: 동의(해석식 + AMBIGUOUS 플래그, MC 오프라인 강등, boolean→연속 σ_ψ) — 검증 증거 표현만 수정.
- 반박 근거: "정확히 재현" 은 태그 0.30 m·1080p 가정에서만 성립(20 cm 태그였으면 1.0°) 하고, 1차 원근 유도(좌우변 높이차 Δh = L²ψ/fx, σ_Δh = 2σ_c) 로는 계수가 2 — √2 는 사후 맞춤이라 검증 증거로 쓰면 안 된다(피팅 대상이라는 결론 자체는 옳음). "60~145 mm → σ_c≈0.1 px" 역산은 heading 가우시안 가정인데 9/7 결론은 양봉(flip) — 교란. σ_c 3배 변동(블러·역광)을 NIS 만으로 잡으면 1~2 s 늦다(그 사이 게이트 통과 = false confidence).
- 대안: σ_ψ = a·tag_px^−p 피팅 유지 + σ_c **직접 추정**(옳은 해의 재투영 잔차 2-DOF 를 30 프레임 평균, `decision_margin` 보조) 을 NIS 앞단에 두어 프레임 단위 R 스케일. `reliable_angle` 은 (σ_ψ, heading_valid) 두 값으로 대체(제안 그대로).
- 대안 근거: Adámek 2023(A19/B1) σ²∝S_n^−p; critic §5; `measure()` 의 예측 vs 관측 spread 대조(`/Users/seojeongmin/krri/apriltag_v3/src/models/detection/detection_pose.py:704-716`) 를 상시화.
- 실차 검증법: 정적 3/5/7/10 m ×100 프레임 × 노출 2단(정상/−2EV) → per-frame heading std 가 a·tag_px^−p 에 맞는지, 재투영 잔차 평균이 같은 프레임에서 std 변화(σ_c 3배) 를 따라가는지.

### 쟁점 5: uncertainty ≠ 실제 오차 — calibration
- 판정: 동의 + 빠진 검사 3개.
- 반박 근거: (a) 프레임 NIS≈1 은 **독립성**을 보장하지 않는다 — flip 이 20 프레임 유지되거나 진동·조명 기울기 같은 느린 계통오차면 프레임 NIS 는 정상인데 N 프레임 평균 오차는 σ/√N 보다 훨씬 크다(false confidence 의 실제 경로, critic §6·Failure 2). (b) 줄자 ±1 cm 진값은 3 m 에서 예측 σ_lat ≈10 mm 와 같아 c 를 못 잰다. (c) forward/거리 진값이 없어 TAG_SIZE 스케일 오차(1% = 3.5 m 에서 35 mm; depth 대조는 `DEPTH_CHECK_MAX_Z`=1.5 m 까지만) 와 카메라–포크 횡 오프셋·롤 상수 바이어스(쟁점 1·2) 를 못 잡는다.
- 대안: 커버리지를 프레임이 아니라 **KF 출력 1/3/10 s 창**에서 반복 — 창이 길수록 커버리지가 무너지면 상관 오차로 판정하고 σ_floor(d) 상향. 진값은 바닥 테이프 격자 + 레이저 거리계(±3 mm) 로 lateral·forward 둘 다, 기준점은 카메라 아닌 **포크 중심**; distance 진값 기울기로 TAG_SIZE 확정.
- 대안 근거: Bar-Shalom(C2) 일관성 검정은 백색성 포함; Chang 2014(B24); critic §6·§23(평균 아닌 95/99 백분위 KPI).
- 실차 검증법: 3.5 m 정적 300 프레임(10 s) 에서 z=(ê−truth)/σ̂ 를 1/3/10 s 창 KF 출력별로 → 68/95% 커버리지가 창 길이에 무관해야 통과.

### 쟁점 6: 지연 보정 — timestamp·전파
- 판정: 동의(단일 시계·캡처 시각·SE(2) 퇴화·명령지연은 dynamics 층) + 코드 사실 정정 2개 + 정책 1개.
- 반박 근거: "camera.py:219-236 이 켬" 은 사실이 아니다 — `set_global_time()` 은 정의만 있고 호출처 0건(grep); `CameraSettings.docking()` 의 global_time_enabled=True·frames_queue_size=1 은 `tune=` 을 줄 때만 적용되는데 `/Users/seojeongmin/krri/apriltag_v3/tools/run.py:181` 은 `from_realsense(args.tag_size, label="run")` 로 tune 없이 연다 → 지금은 SDK 기본값(global_time 기본 ON, **큐 16**) 의존. 큐 16 은 검출이 33 ms 를 넘는 순간 최대 ~0.5 s 묵은 프레임을 "현재" 로 넘긴다(critic Failure 5 의 실제 경로; 제안은 t_arrival 을 기록만 하고 정책이 없다). 자이로 motion frame 도 같은 global_time 도메인이므로 "시계 3종" 중 카메라–자이로는 이미 같은 시계일 가능성이 크다(확인 필요).
- 대안: 시작 시 컬러·자이로 두 스트림의 `timestamp_domain` 을 assert+로그, run 경로에 frames_queue_size=1 명시. **stale 정책**: t_arrival−t_capture > 150 ms 프레임은 미검출 취급(predict-only)+경고. 태그 중심 행 시각 보정(쟁점 2). time.time() NTP 점프 감시(연속 dt 가 33±10 ms 밖이면 플래그).
- 대안 근거: critic §7·§21; B27(RGB ≈66 ms); C3 PX4 지연 지평·C1 Larsen; `image.py:449` 상대시각 / `control_from_pose.py:317` monotonic / `event_log.py:29` epoch 혼용 확인함.
- 실차 검증법: 제안된 8°/s 회전 상호상관(τ_cam 예측 80~110 ms) 을 그대로 하되, 검출 스레드에 인위 부하(sleep 40 ms) 를 걸어 t_arrival−t_capture 가 어디까지 벌어지는지 함께 기록 → stale 문턱을 실측으로.

### 쟁점 7: 주행 중 연속 추정기 구조
- 판정: 동의(2-KF 구조, 구동기 모델 미포함, v̂ 재평가 — σ_slope=0.633·σ_d/s 재계산 일치) + 결합 리스크 3개.
- 반박 근거: (a) "측정은 d=|t|·β·ψ̂ 로 만든 pivot 좌표" — |t| 는 3D 거리라 1.1 m 높이차로 3.5 m 에서 3.67 m(**forward +17 cm**, 2 m 에서 +28 cm) → 정지점이 탑재부 쪽으로 밀린다. (b) 위치 KF 측정이 ψ̂ 를 타므로 heading 재초기화(쟁점 3) 순간 측정이 d·Δψ 점프 → 게이트가 옳은 측정을 연속 기각하거나 R 부풀림으로 응답이 죽는다; 재초기화 이벤트 전파가 없다. (c) b_g 는 카메라 갱신 없인 비관측 — 10→5 m AMBIGUOUS ~18 s 에 1°/분 드리프트 = 0.3° = 5 m 에서 26 mm; 5 m 30 프레임 재앵커도 σ_ψ≈0.5° → σ_lat 40~50 mm > 30 mm: **5 m 정렬점 lateral 은 30 mm 판정에 못 쓴다**(≤3.5 m 만).
- 대안: 위치 KF 는 쟁점 1 의 p̂_pivot(3D 변환) 만 소비(스칼라 재유도 금지). heading 재초기화 시 위치 KF 도 같은 Δψ 로 상태 회전 + P 팽창을 **원자적 이벤트**로. 출력에 `lateral_valid = (σ_lat_eff ≤ LAT_TOL/2)` 를 붙여 원거리는 β 전용임을 컨트롤러가 알게(컨트롤러 토론에 하드 제약으로 이관). 정지마다 자이로 바이어스 재추정(Hong&Park B18, 데드밴드 0.3°/s).
- 대안 근거: PLAN ①·B4 Hinderer·C4 Hurák 캐스케이드; critic §19 — `lateral_valid` 없이 원거리에서 "불확실하니 기다림" 이면 wait deadlock 그 경로.
- 실차 검증법: 정속 직진 bag 에서 KF v̂ 1 s std ≤5 mm/s(3 m) + 5 m 정지 30 프레임 lateral 을 10 회 반복해 회차 간 std → 40~50 mm 예측이 맞으면 5 m 판정 금지 확정.

### 가장 위험한 제안 TOP2
1. 쟁점 7 "d=|t|" — 3D 거리를 수평 forward 로 쓰면 3.5 m 에서 +17 cm(2 m 에서 +28 cm) 과대 → 마지막 직진이 탑재부 쪽으로 밀려 충돌 방향 오차; 위치 측정은 쟁점 1 의 3D 변환 결과만 써야 한다.
2. 쟁점 3 "태그를 15° 아래로 기울여" — 면이 카메라 쪽을 향하면 ≈4.1 m(T 구간) 에서 법선=시선 → 모호성 최대·pitch-cue 소멸; 위쪽 또는 좌우 yaw 15~20° 로 바꾸고 실측 전엔 손대지 말 것.