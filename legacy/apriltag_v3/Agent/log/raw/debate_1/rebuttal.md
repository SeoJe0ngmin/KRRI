## 재검산 (반박 검증에 쓴 로그 사실 — `/Users/seojeongmin/krri/apriltag_v2/work_dirs/docking_log/20260907_181437/{measure,rotation,decision}.jsonl`)

R-free 방위각 β = heading + atan2(lateral, forward) (= atan2(t_x,t_z), 같은 T 에서 항등) 로 13회 회전을 다시 셈:

| 회전(측정쌍) | d[m] | tilt 전→후 | Δψ_IMU | Δψ_cam | Δβ | Δβ/Δψ_IMU | A=+1.46 예측 1+A/d |
|---|---|---|---|---|---|---|---|
| r1 (m1→2) | 9.3 | 31→40 | +9.74 | +9.42 | +9.28 | 0.95 | 1.16 |
| r4 (m5→6) | 3.9 | 4.4→2.9 | −4.51 | −5.64 | −3.99 | 0.88 | 1.37 |
| r5 (m6→7) | 3.85 | 2.9→11.8 | −8.07 | −9.83 | −6.45 | 0.80 | 1.38 |
| r6 (m7→8) | 3.75 | 11.8→23.6 | −12.55 | −11.69 | −10.93 | 0.87 | 1.39 |
| r7 (m8→9) | 3.8 | 23.6→28 | −6.74 | −4.53 | −6.20 | 0.92 | 1.38 |
| r8 (m9→10) | 3.75 | 28→25 | +5.09 | +3.06 | +4.35 | 0.85 | 1.39 |
| r9 (m10→11) | 3.76 | 25→26 | −2.30 | −1.32 | −2.18 | 0.95 | 1.39 |
| r10 (m12→13) | 3.3 | 26.4→3.9 | +24.55 | +23.00 | +21.57 | 0.88 | 1.44 |
| r11 (m13→14) | 3.34 | 3.9→4.1 | +5.61 | +7.02 | +4.89 | 0.87 | 1.44 |
| r12 (m14→15) | 3.3 | 4.1→20.4 | +17.79 | +16.81 | +15.67 | 0.88 | 1.44 |
| r13 (m16→17) | 4.7 | 24→26 | +2.68 | +2.45 | +2.48 | 0.93 | 1.31 |
| r14 (m19→20) | 3.27 | 23→4.4 | −21.65 | −19.40 | −19.21 | 0.89 | 1.45 |
| r15 (m20→21) | 3.28 | 4.4→3.1 | −5.42 | −6.16 | −4.19 | 0.77 | 1.45 |

- 13/13 에서 Δβ/Δψ_IMU < 1 (평균 0.88). A=+1.46 은 1.16~1.45 를 요구 → 전부 기각. 경사쌍(r1·r6·r10·r12·r14) 의 Δθ_cam = Δβ−Δψ_cam 은 −0.14/+0.76/−1.43/−1.14/+0.19° 로 23° 회전에도 카메라 위치각이 거의 안 움직임 → **A ≈ −0.2±0.1 m (|A| ≤ 0.4, 부호 0 이하)**. 단 경사쌍 Δψ_cam/Δψ_IMU = 0.97/0.93/0.94/0.95/0.90 (IMU `turned` 가 ~6.5% 과대 또는 카메라 과소) 와 혼입.
- 정면쌍(r4·r5·r11·r15) Δψ_cam/Δψ_IMU = 1.25/1.22/1.25/1.14 → 카메라 heading 이 +1.1~1.8° 과대변화. 18:14 는 A=0·S=3.0 으로 돌았음(dec#5 dist_t 0.907 = hypot(0.046, 0.906)) → T 근처 6회 회전 반복(step 5~10, T 방위각 −2.9→−9.9→−24.6°) 의 원인은 팔 길이가 아니라 **정면 heading 바이어스 × d**.
- `vertical` 열: 정면(tilt<5°) −0.82~−0.88 m vs 경사 −0.98~−1.19 m, 같은 3.3~3.9 m 에서 0.25 m 계통 차 → 옳은 branch 의 pitch 가 어느 한쪽에서 ≥3~4° 바이어스.
- 1.46 의 출처 `/Users/seojeongmin/krri/apriltag_v2/work_dirs/오차보정.txt:46-49`: 자이로 yaw 0.00·rate 0.07 고정(스트림 사망), 8 m, Δlateral 0.128 m = heading 오차 1°×8 m 와 같은 크기, 도구 자체가 "차체가 안 돌았다" 출력.

---

### 쟁점 1
- 수용/유지/수정: **수정** — 명시적 변환·오라클은 유지, "부호 충돌" 진단과 A=1.46 전제는 철회.
- 이유+근거: 두 critic 이 맞다. (a) 기하: 카메라 = pivot + A·u(ψ) 에서 코드 `lat_p = lateral − A·sin h`, `fwd_p = forward + A·cos h`(`/Users/seojeongmin/krri/apriltag_v3/src/models/control/control_from_pose.py:93-94`) 는 정확하고, 18:14 의 +112/+226 mm 는 정면 heading 과대변화(−1.13°/−1.76° × 3.9 m ≈ +77/+118 mm) 로 대부분 설명된다(A=+1.46 이면 −115/−204 mm 로 부호 반대여야 함). (b) 오라클 부호: `detection_pose.py:51-53` 예제 β=atan2(0.151,1.387)=6.21°, ψ=2.04°, d_h=hypot(0.104,1.407)=1.411 → 1.411·sin(6.21−2.04)=+0.1026 ≈ lateral +0.104 → 코드 규약은 **lateral = d_h·sin(β−ψ)**; 방법론 `1. perception_methodology_v2.md:424` 의 d·sin(ψ̂−β) 는 반대이고 문서 스스로 429-437 행에서 "코드 기준 검증" 을 유보했다. (c) A: 위 표 13/13 기각. 정면 원뿔에서 회전 1회로 부호를 "확정" 하면 바이어스가 A 로 둔갑한다는 정밀 critic 의 TOP1 동의.
- 수정된 제안: `T_tag←pivot = T_tag←cam · T_cam←pivot`, 병진은 3-벡터 (x_off, 0, −A) 로 카메라–포크중심 횡 오프셋 포함; **A·x_off 는 추정 파라미터** (사전 A∈[−0.5,+0.5], 초기 0, σ_A 를 P 에 전파 — 1.46 은 사전에서 제외). 확정 실험: tilt≥20°·3~4 m 정지→회전 15~25°→정지 ×5, 자이로는 정지 판정 후 3 s 더 적분, 프레임별 (β_px, ψ_gyro) 회귀 Δβ=(1+A·cosβ/d)·Δψ 로 A·부호·자이로 스케일을 동시에; 줄자 카메라→구동축 실측이 2차 증인. 확정 전 컨트롤러는 lat_p 대신 β 로 조준. 오라클 = `|lateral − d_h·sin(β−ψ)| ≤ 5 mm`(R̂=raw R 조건).

### 쟁점 2
- 수용/유지/수정: **수정** — B(픽셀 β) 채택은 유지, 채택 근거·지배 오차원·롤 출처를 바꿈.
- 이유+근거: 약원근에서 두 IPPE 해는 t 방향을 공유 → "β_A 양봉 확인 후 채택" 실험은 아마 아무것도 안 보인다(안전 critic 맞음) — B 의 가치는 PnP 무관·quality 실패 프레임에도 산출·왜곡 모델 단일화다. 코드 확인: `/Users/seojeongmin/krri/apriltag_v3/src/models/detection/detection_tag.py:76-78` 은 camera_params=(fx,fy,cx,cy) 만 넘겨 라이브러리 자세가 왜곡 미보정이고 `pose_by_pnp`·`heading_sigma_deg` 만 distortion 을 쓴다(두 모델 혼용); `image.py:56-58` 은 `i.model` 을 버리고 coeffs 만 저장, `undistort` 호출자 0. T 에서 태그 상단 margin 31~97 px(`decision.jsonl` step 12/19) → 정규화 반경 r≈0.35: 계수가 k1≈0.1 급이면 중심 변위 ≈5.7 px, 태그 폭 116 px 에 걸친 차등 왜곡 0.1~0.2 px 만으로 3.4 m heading 0.8~1.5°(1 px ≙ 7.6°) → 정면 매끈 바이어스(0.7~2.3°) 와 `vertical` 열의 pitch 바이어스의 **후보 원인(가설; D435i 컬러 coeffs 는 장치별 0 일 수 있어 로그로 확정)**. 롤 19 mm/° 거리무관(1.1 m×δφ) 수용; BMI055 가속도 영점(수십 mg≈수 °) 보다 태그 상하변 방향(코너 0.1 px/130 px ≈ 0.04°/프레임) 이 롤 출처. 롤링셔터는 두 critic 이 다른 것을 말함 — 태그 내부 차등(~2 ms, 0.02°) 은 무시, 태그 행 시각 오프셋(≤30 ms, 8°/s 에서 0.24°) 은 쟁점 6 의 timestamp 문제.
- 수정된 제안: 코너를 `i.model` 에 맞게 보정(inverse_brown_conrady 면 역방향)해 β·PnP 모두 같은 보정 코너로; `i.model`·coeffs 를 `--record-events` config.json 에 기록; β 는 무조건 산출; 롤은 태그 roll 30프레임 평균(가속도계는 pitch 검증용, pitch 는 h 일관성 atan(h/d_h)−atan2(−t_y,t_z) 로 교차 확인); cx/장착 yaw/롤 상수 바이어스는 쟁점 5(c) 직진 실험으로 흡수. 검증: 정적 3.5 m 상단 배치, 왜곡 보정 전/후 heading·pitch 100프레임 + 카메라 롤 2° 시 lateral ≈38 mm 이동 확인.

### 쟁점 3
- 수용/유지/수정: **수정** — 두 해 명시 산출·pitch-cue·3상태·ψ̂ 무관 단서는 유지; 탈출→SPRT, 초기화→다수결, 하드웨어 방향 정정, **tilt 원뿔 하드 게이트 복원**.
- 이유+근거: (안전) "15연속" 은 8 m σ_pitch 6.3° 에서 0.66¹⁵≈0.2% 로 탈출 불능 — 맞음. "태그 15° 아래" 는 법선–시선각이 |ε−15°| 로 줄어 d=1.1/tan15°≈4.1 m(T 구간) 에서 정면 — 내 부호 오류, 철회. (정밀) 정면 4쌍 Δψ_cam/Δψ_IMU 1.14~1.25, spread 0.016~0.064° → 옳은 branch 의 매끈한 바이어스가 지배하고 pitch-cue·err비·마할라노비스 모두 통과 — 맞음. 내 추가 증거: `vertical` 열의 정면 0.25 m 계통 차 = 옳은 해 pitch 도 ≈3~4° 바이어스라 6° 게이트를 통과. `pose_err`≈1e-6 은 객체공간 m² 이므로 `pose_quality` 의 2 px 게이트(`detection_pose.py:363`) 는 AT3 경로에서 죽어 있음 — 확인. Richter 2022(±25° 원뿔 회피), Rijlaarsdam 2022(3.3 m ±10° 구별 불가).
- 수정된 제안: `heading_valid` = 기하 원뿔(tilt ≥ θ_min(d), 초기 15°) **[하드]** ∧ SPRT(프레임별 log N(pitch; pitch_mount,σ)/N(pitch; 2ε,σ) 누적, σ·임계는 정적 실측) [branch 선택] ∧ err비 [소프트]; 초기화·재초기화는 30프레임 다수결. 정면 구간 heading = 마지막 oblique 앵커 + 자이로(짧은 다리). |pitch−pitch_mount| 는 연속값 "부분 붕괴 지표" 로 출력하고 정적 격자에서 yaw 바이어스와의 상관을 검증(가설). AMBIGUOUS 타임아웃 → 정지·oblique 재획득 fallback 상태 명시. 하드웨어: 태그 좌우 yaw 15~20°(도킹축 = 태그프레임 상수 오프셋 → 컨트롤러 토론 이관) 또는 위쪽 15°; h ≥ 0.6 m 설계 상수; 실측 전 손대지 않음.

### 쟁점 4
- 수용/유지/수정: **수정** — 해석식·연속 σ·MC 오프라인 격하 유지; "정확히 재현" 은 증거에서 제외; σ_c 직접 추정; **바이어스 floor 추가**.
- 이유+근거: 재검산 — 정면 3.3~3.9 m 30프레임 spread 0.016~0.064° → 프레임당 0.09~0.35° 인데 해석식(σ_c 0.15) 은 1.2°: 식은 spread 를 3~13배 과대, 반면 실오차(바이어스 0.7~2.3°) 는 spread 의 10~50배 → spread 도 해석식도 오차의 대리가 못 된다(정밀 critic 맞음, 바이어스를 σ 에 넣으면 √N 으로 잘못 준다). 8 m 경사(can_pulse 18:43 settle, tilt 38°) heading 프레임 σ≈0.35°·lateral σ≈40 mm 는 식(0.1°) 의 3.6배 → 계수·지수·tilt 의존 모두 피팅 대상. 같은 파일 정지 forward σ≈20 mm → σ_c≈0.13 px, 주행 중 ~0.3 px → `CORNER_NOISE_PX=0.07`(`/Users/seojeongmin/krri/apriltag_v3/config/detection.py:34`) 은 정지에도 낙관.
- 수정된 제안: σ_ψ,eff² = [a·tag_px^−p·g(tilt)]²·(σ̂_c/σ_c0)² + b²(d, tilt). σ̂_c 는 옳은 해 재투영 잔차 30프레임 평균(NIS 앞단, `decision_margin` 보조), b 는 정적 격자 |median−truth| 표(관측 시간으로 안 줄어드는 floor). 출력은 (σ_ψ,eff, heading_valid) 두 값. Adámek 5+5 는 g(tilt) 1~2 파라미터로 대체.

### 쟁점 5
- 수용/유지/수정: **수정** — 3검사·컨트롤러 계약 유지; NIS 는 방어선→온라인 경보로 격하; 1차 = 교차센서 일관성; 진값 기준 = 포크 중심 + 레이저.
- 이유+근거: 정적 장면의 매끈한 바이어스는 상태가 흡수해 NIS≈1 (critic §24 Failure 2) — 18:14 spread≈0.03° 가 정확히 그 경우. 교차센서 (a)(b) 는 18:14 만으로 A 와 heading 바이어스를 이미 잡았다(위 표). (c) 의 1표본도 있다: can_pulse 18:46 forward 5 s(8 m, tilt 33°) 이동방향 atan(0.565/0.920)=31.6° vs heading 33.1° → δ≈1.5°. 이중 해석: `/Users/seojeongmin/krri/apriltag_v3/config/control.py:70,75` "회전 잔차 ±1.5°" 는 카메라–IMU 차(0.2~2.3°, 13/13) 를 dynamics 로 오귀속한 것. TAG_SIZE 스케일 1%(3.5 m 35 mm) 는 `DEPTH_CHECK_MAX_Z=1.5` 라 미검출 — 맞음.
- 수정된 제안: 순서 (1) 교차센서: 회전 ×5(Δβ vs Δψ_IMU → A·스케일; Δψ_cam vs Δψ_IMU(+3 s 적분) → 바이어스/스프링백 분리), oblique 직진 ×2(5→3.5 m, Δlat/Δs → δ, β·d·ψ̂ 공동 피팅) — 5회 중 4회 같은 부호·±0.5° 재현이 통과 기준; (2) 정적 진값: 바닥 격자+레이저(±3 mm), 기준 **포크 중심**, (d,tilt) 셀별 c 와 b, distance 진값 기울기로 TAG_SIZE; (3) 커버리지를 KF 출력 1/3/10 s 창에서 — 창 길이에 무관해야 통과(상관 오차 검출); NIS 는 감시만.

### 쟁점 6
- 수용/유지/수정: **수정** — 코드 사실 정정 2개 수용, stale·큐 정책 추가, A 선행조건.
- 이유+근거: grep 확인 — `set_global_time` 은 `/Users/seojeongmin/krri/apriltag_v3/src/utils/camera.py:219` 정의만 있고 호출 0; `CameraSettings.docking()`(`camera.py:484-485`) 의 queue=1·global_time=True 는 tune= 경로만; `/Users/seojeongmin/krri/apriltag_v3/tools/run.py:181` 은 `TagPipeline.from_realsense(args.tag_size, label="run")` → SDK 기본(큐 16). `image.py:415` "절대 밀리지 않음" 은 근거 없는 주석, `:449` 는 ts−t0 상대시각. 18:14 r4 종료(538.446)→m6(539.750) 1.30 s/30프레임 ≈ 23 fps 추정 동의(중간 대기 없다는 전제). 자이로 `imu_yaw.py:153` 도 global_time 이면 같은 시계일 가능성 — 확인 필요 동의.
- 수정된 제안: run 경로에 frames_queue_size=1(또는 드레인) 명시 + 컬러·자이로 `timestamp_domain` assert·로그; 프레임별 (t_capture, t_row = t_capture + row/H·t_readout, t_arrival, t_detect_done, t_decide, t_cmd_tx) 기록; stale 정책 t_arrival−t_capture > 150 ms → predict-only+경고; dt 33±10 ms 밖 플래그(NTP 점프); SE(2) 전파 `카메라점 = pivot + A·u(ψ)` 의 A 는 쟁점 1 파라미터로 σ_A 를 예측 P 에. 검증: 8°/s 상호상관(τ_cam 80~110 ms 예측) + 정지 60 s 의 t_arrival−t_capture 추세 + 인위 부하 40 ms.

### 쟁점 7
- 수용/유지/수정: **수정** — 2-KF·구동기 모델 미포함·v̂ 재평가 유지; d=|t| 철회; 유니사이클 결합; 재초기화 원자 이벤트; `lateral_valid`; 자이로 alive 게이트; A 파라미터.
- 이유+근거: |t| 는 3D 라 3.5 m +17 cm/2 m +28 cm — 내 표기 오류, 위치 측정은 쟁점 1 의 p̂_pivot 만(안전 TOP1 수용). 대각 주행 ψ 25° 에서 y 변화율 v·sinψ≈0.12 m/s → [y_l] 정위치 모델은 지연, 유니사이클 ẋ=v cosψ·ẏ=v sinψ 수용(v 는 여전히 측정 주도 상태, 명령은 Q 스케줄만). `rate_dps` 는 마지막 샘플 고정(`/Users/seojeongmin/krri/apriltag_v3/src/utils/imu_yaw.py:301-305`) + can_pulse 18:43 에서 자이로 사망 중 카메라 heading 33→38° — alive/gaps 게이트 필수. 5 m 30프레임 σ_ψ≈0.45° → σ_lat≈40 mm > 30 mm 수용. 단 부분 반박: 대각 접근 전제에선 T 까지 tilt≈|ψ|≥15° 라 yaw 가 관측 가능 → 자이로 단독 구간은 T 이후 ≤3.5 m(12~23 s, 드리프트 0.2~0.4°) 뿐이고 "10→5 m 18 s AMBIGUOUS" 는 정면 직진 접근에만 해당. 재포착 innovation 1~2°(13/13) 를 σ 0.05° 게이트가 매번 기각한다는 지적은 맞음 → 게이트 σ 에 쟁점 4 의 b floor.
- 수정된 제안: 위치 KF 상태 [x_p, y_p, v], 입력 ψ̂(자이로 캐스케이드, [ψ, b_g]), 측정 = 쟁점 1 의 p̂_pivot(σ_A 포함); 회전 중 predict-only 는 σ_A·Δψ 가 작을 때만, 종료 후 oblique N프레임 다수결로 ψ 재앵커(게이트 σ=√(σ_random²+b²)); heading 재초기화 시 위치 상태 회전+P 팽창을 원자 이벤트로; 정지마다 b_g 재추정(|ω|<0.3°/s, Hong&Park B18); 출력 `lateral_valid = σ_lat_eff ≤ LAT_TOL/2`·`heading_valid`·`gyro_alive`(컨트롤러 하드 제약으로 이관); 자이로 stale → 예측 정지·AMBIGUOUS. 검증: 정속 bag v̂ 1 s std ≤5 mm/s(3 m) + 5 m 정지 lateral 10회 반복 std(40~50 mm 예측 맞으면 5 m 판정 금지 확정) + 회전 5회 재포착 innovation 분포.

### 쟁점 간 의존관계 (수정)
- 1(A 파라미터·부호) 이 6(SE(2) 전파)·7(회전 중 predict-only) 의 선행조건이 됐고, 확정 전엔 컨트롤러가 β 만 쓴다; 2(왜곡 모델 단일화) 는 3·4 정적 격자 결과 해석의 선행조건(바이어스 원인 후보).
- 3 의 tilt 원뿔이 4 의 b(d,tilt) floor 정의 영역을 정하고, 5(1) 교차센서 캘리브레이션이 4·7 의 게이트 σ 를 공급한다.
- 실차 1순위(수정): ① 회전 ×5(tilt≥20°, 3~4 m, 자이로 +3 s) → A·부호·자이로 스케일·카메라–IMU 바이어스(1·5·6·7 동시) → ② 정적 격자 3/5/7/10 m × heading 0/±3/±6/±10/±15/±25° × 노출 2단 × 100프레임, 왜곡 보정 전/후(θ_min, b, pitch 양봉 σ, SPRT 임계, σ_c) → ③ oblique 직진 ×2 + 정속 bag + 5 m 10회 반복.