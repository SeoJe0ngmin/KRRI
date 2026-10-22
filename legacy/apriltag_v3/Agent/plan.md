# v3 Docking Plan (토론으로 완성 중)

> proposer ↔ critic 토론(`log/`)의 결론을 모은 최종 설계.
> 근거: 9/7 실차 로그 + 선행연구(`../docs/prior_art_docking.md`) + methodology 4문서 + critic.md.
> 상태: **대문제 1·2·3·4 토론 전부 완료**(2026-09-20~21, `log/debate_N_*.md`) — 사용자 검토 대기. 다음 단계 = 검토 → 정오표 반영 → 상수 대장 초안 → G_code-A(인프라 코딩, 사용자 결정 불필요).
> **작업 정의 정정(2026-09-21, 사용자 확정 — 토론 1~4 는 "팔레트 포크 삽입" 을 전제로 쓰였고 그 전제가 틀렸다)**:
> 도킹 = 지게차가 **탑재부(차고형 공간)에 들어가 주차**. 팔레트 삽입이 아니다 — **포크는 접힌 채** 진입, 삽입 깊이·포켓 폭·포크 폭은 **해당 없음**.
> **탑재부 폭 ≈ 지게차 폭** → 좌우 여유 **c = 3 cm 가 실물 근거값**(도면 유도 아님) = `LAT_TOL_M 0.030` **유지·확정**.
> **기준점 x_ref = 포크 끝**(접힌 상태, 카메라에서 `CAM_TO_REF_M = 1.52 m`) — 확정. 제일 앞이라 먼저 진입하고 비뚤면 먼저 닿는다.
> **탑재부 진입 깊이 L_dock 은 미측정**(내일도 못 잼) → **2 m 보수 가정** → `HEAD_TOL_DEG` 2.0 → **0.5°**(c/L_dock = 0.030/2 = 0.86°, 안전 여유; 토론 2·3 의 σ_θ ≤ 0.5° 요건과 독립적으로 일치). L_dock 실측 후 완화 가능.
> 아래 본문에서 **D = 탑재부 진입 깊이 L_dock(2 m 가정)** 으로 읽는다(원래 "포크 삽입 깊이" 였음).
> 주의: 대문제 1 절은 '상수 최소화 원칙'(CLAUDE.md, 9/20 저녁 추가) 이전에 쓰여 표(B_TABLE·C_TABLE·SIGMA_FLOOR)·상수 분류가 원칙과 어긋난다 — 감사 후 정정 예정. 대문제 2 절부터 원칙 적용.

## 목표 / 성공 기준
- (대문제 4 결론 + 9/21 정정) 성공 = x_ref(**포크 끝**, 카메라 앞 1.52 m)에서 **평행사변형 max(|e_l|, |e_l − L_dock·tan e_h|) ≤ c**, **c = LAT_TOL_M = 0.030**(탑재부 폭 − 지게차 폭 실물 여유, 사용자 확정), L_dock = 탑재부 진입 깊이(**미측정 → 2 m 가정**), 전후 [−FWD_TOL_M, +min(FWD_TOL_M, 안전선)], ∧ abort 없음 ∧ DOCK_TIME_BUDGET_S 안. 1차 KPI = **wrong-DONE 0/n**, 성공률은 (n, 실패수, 95% 하한)으로만 보고, "99%" 는 시뮬 + n≥59 로만 뒷받침.
- **내일(9/22) 실험 성공 기준은 위와 다르다**: 탑재부에 들어가지 않고 **태그면 앞 2.02 m(STANDOFF, 벽 충돌 방지)에서 정지** → 거기서 줄자로 (e_l, e_h) 를 재 평행사변형에 대입해 **사후 판정**만 한다.

---

## 대문제 1 — Perception (현재 상태 추정)
_status: debated 2026-09-20 — 결론 7 / 보류 0, **사용자 검토 대기** (근거·왕복은 `log/debate_1_perception.md`)_

### 1-1 lateral 재구성 — 명시적 변환, 회전팔 A 는 파라미터
- 결론: lateral·forward 는 `T_tag←pivot = T_tag←cam · T_cam←pivot` 로 재구성한다. 위치 계산엔 프레임 원시 R 대신 R̂ = R_mount·R_yaw(ψ̂). `T_cam←pivot` 병진은 3-벡터 (x_off, 0, −A). **A(카메라→회전중심, 부호 있음)·x_off 는 추정 파라미터** — `CAM_TO_PIVOT_M = 1.46` 은 철회(9/7 18:14 로그 13/13 회전에서 Δβ/Δψ_IMU 0.77~0.95, A=+1.46 이 요구하는 1.16~1.45 와 불일치; 1.46 의 출처는 자이로 사망 상태 8 m 1회 펄스). 코드 결합항 `lat_p = lateral − A·sin h`, `fwd_p = forward + A·cos h` 는 규약상 정확하므로 유지. 단위테스트 오라클은 코드 규약 `|lateral − d_h·sin(β−ψ)| ≤ 5 mm`(β=atan2(t_x,t_z), d_h=√(t_x²+t_z²)); 방법론 v2 424행 `d·sin(ψ̂−β)` 는 부호 반대 → 문서 수정(coder 단계).
- A 확정 전 컨트롤러는 lat_p 대신 **β_px 로 조준**한다(→ 대문제 3 하드 제약).
- 검증: tilt≥20°, d 3.5 m·6 m 두 거리, 정지→IMU 회전 15~25°→정지 ×5(자이로는 정지 판정 후 3 s 더 적분). 프레임별 (β_px, ψ_gyro) 회귀 `Δβ = s_gyro·(1 + A·cosβ/d)·Δψ` 로 자이로 스케일 s 와 A·부호 분리. 통과 = 5회 중 4회 같은 부호, |A| 산포 ≤ 0.1 m. 줄자 카메라→구동축 실측이 2차 증인. **정면 원뿔에서 하면 바이어스가 A 로 둔갑 — 금지.**
- 파라미터: `A_M` (사전 [−0.5, +0.5], 초기 0, σ_A → P 전파) · `X_OFF_M` (미측정) · `CAM_YAW_OFFSET_DEG` (1-5 로 확정).

### 1-2 bearing β — 픽셀 중심, 매 프레임 무조건
- 결론: β = `detection.center` → 왜곡보정 → 정규화 광선 → R_mount 회전 → atan2(x_n, z_n). **PnP 유효성·quality 와 무관하게 매 프레임 산출**(β_A=atan2(t_x,t_z) 는 대조·로그용). 왜곡 모델 단일화: `CameraIntrinsics.from_realsense` 가 버리는 `i.model` 을 보존해 `--record-events` config.json 에 coeffs 와 함께 기록, coeffs≠0 이면 모델(inverse_brown_conrady 면 역방향)에 맞게 코너 보정 후 β·PnP 모두 같은 코너(현재는 AT3 자세 미보정 / pose_by_pnp 보정 — 두 모델 혼용). 롤 바이어스(1.1 m×δφ ≈ 19 mm/°, 거리 무관)는 태그 상하변 방향(`pose_to_forklift` roll) 30 프레임 평균으로, 가속도계 R_mount 는 pitch 검증용. 출력 β_tag(원거리 화면중앙 유지 변수)·β_T(T 방위각, σ_βT 부착).
- 검증: 정적 3.5 m·태그 화면 상단 배치 100 프레임 — 왜곡 보정 전/후 heading·pitch 차(>1° 면 왜곡이 정면 바이어스 원인) + 카메라 롤 2° 시 lateral ≈38 mm 이동 확인.
- 파라미터: `DIST_MODEL`(로그로 확정) · `ROLL_AVG_FRAMES` (초기 30).

### 1-3 heading 2중해 — 기하 원뿔 하드 게이트 + 두 해 명시 + AMBIGUOUS
- 결론: `heading_valid = (tilt ≥ θ_min(d)) [하드] ∧ (두 해 err비 ≤ ERR_RATIO) [소프트] ∧ branch CONFIDENT`. 두 해는 `cv2.solvePnPGeneric(SOLVEPNP_IPPE_SQUARE)`(`_object_points` 순서 재정렬)로 명시 산출. 아니면 **AMBIGUOUS**(heading_valid=False, translation·β 는 계속 사용). pitch-cue(틀린 해 pitch ≈ 2·atan(h/d), 옳은 해 ≈ pitch_mount)는 1단계에서 두 해의 pitch·err 를 **로그만**, 정적 격자에서 양봉·σ 확인 후 Wald SPRT 로 승격("N 프레임 연속" 규칙 폐기). 초기화·재초기화(회전 >20° 후 재포착, T 정지)는 30 프레임 다수결, 단일 프레임 CONFIDENT 금지. AMBIGUOUS 지속 → 타임아웃 → 정지·oblique 재획득 fallback 상태. 정면 구간(tilt<θ_min) heading = 마지막 oblique 앵커 + 자이로(짧은 다리). `pose_quality` 의 2 px 재투영 게이트는 pose_err 가 객체공간 m² 라 AT3 경로에서 무효 → 픽셀 재투영으로 재계산. 하드웨어(태그 기울임·yaw 오프셋)는 실측 전 손대지 않음; "아래 15°" 는 T 구간(≈4.1 m)에서 정면이 되므로 금지.
- 근거 요약: 18:14 정면쌍 Δψ_cam/Δψ_IMU 1.14~1.25(+1.1~1.8°), spread 0.016~0.064° → 옳은 branch 의 **매끈한 바이어스**가 지배, 분산 기반 게이트는 전부 통과. T 근처 회전 6회 반복의 원인.
- 검증: 정적 3.5/5/8 m × 진값 heading 0/±3/±6/±10/±15/±25° × 100 프레임 — 두 해 pitch 히스토그램(봉우리·σ → SPRT 임계), 옳은 해 yaw 바이어스 곡선 → |바이어스| > max(1°, LAT_TOL/d) 인 범위가 θ_min(d); err비 > 0.2 빈도.
- 파라미터: `THETA_MIN_DEG(d)` (초기 15°, 격자로 확정) · `ERR_RATIO` (초기 0.2) · `INIT_FRAMES` (30) · `AMBIG_TIMEOUT_S` (대문제 3 과 함께) · `H_TAG_CAM_M` (직접 측정; config 1.1 vs 로그 0.85~1.2).

### 1-4 heading 신뢰도 — 연속값 σ_ψ,eff + 바이어스 floor
- 결론: boolean `reliable_angle` → (σ_ψ,eff, heading_valid). **σ_ψ,eff² = [a·tag_px^−p·g(tilt)]²·(σ̂_c/σ_c0)² + b²(d, tilt)**. a·p·g 는 정적 격자 피팅(p 초기 2, g 는 1~2 파라미터 — Adámek 5+5 대체). σ̂_c 는 옳은 해 **픽셀** 재투영 잔차 30 프레임 평균(NIS 앞단 R 스케일; 실측 정지 ≈0.07~0.13 px, 주행 ≈0.3 px — `CORNER_NOISE_PX=0.07` 은 낙관). b 는 |median−truth| 표 = 관측 시간으로 안 줄어드는 floor(**√N 축소 금지**). `heading_sigma_deg` 몬테카를로는 프레임 루프에서 내리고 오프라인 기준으로만.
- 검증: 정적 3/5/7/10 m × tilt 0/10/20/40° × 100 프레임 × 노출 2단(정상/−2EV) — per-frame std 가 모델에 맞는지, 재투영 잔차 평균이 std 변화(σ_c 3배)를 따라가는지, |median−truth|/spread ≫ 1 인 셀은 b floor 채택.
- 파라미터: `SIGMA_A, SIGMA_P, G_TILT(…)` · `SIGMA_C0_STATIC / _MOVING` · `B_TABLE(d, tilt)` · `SIGMA_C_WINDOW` (초기 30, 지연 1 s 위험 → 격자 후 결정).

### 1-5 covariance calibration — 교차센서 → 진값 → 창별 커버리지
- 결론: P 는 `estimated_covariance`. 게이트에는 **σ_eff = max(c·σ̂, σ_floor(d, tilt))** 만. 순서: **(1) 교차센서 일관성** — 회전 ×5 에서 Δβ_px vs Δψ_IMU(A·자이로 스케일), Δψ_cam vs Δψ_IMU(+3 s 적분으로 바이어스/스프링백 분리); oblique 직진 ×2(5→3.5 m, ψ_cam≈25°) 에서 Δlat/Δs = sin(ψ_cam−δ) → δ = CAM_YAW_OFFSET(정면 직진으로 재면 정면 yaw 바이어스와 섞임 — 금지). 통과 = 5회 중 4회 같은 부호·±0.5°. **(2) 정적 진값** — 바닥 테이프 격자 + 레이저(±3 mm), 기준점 **포크 중심**, (d,tilt) 셀별 c·b, distance 진값 기울기로 TAG_SIZE(1% = 3.5 m 35 mm). **(3) 커버리지는 KF 출력 1/3/10 s 창**에서 z=(ê−truth)/σ̂ 의 68/95% — 창이 길수록 무너지면 상관 오차 → σ_floor 상향. 온라인 NIS(1~2 s) 는 R 스케일 **경보**로만(매끈한 바이어스는 NIS≈1). 컨트롤러 계약: |e| > T + k·σ_eff 가 N 프레임 지속일 때만 판정, AMBIGUOUS 는 공분산 밖 하드 플래그.
- 이관: `config/control.py:70,75` "회전 잔차 ±1.5°" 는 카메라–IMU 차(13/13, 0.2~2.3°)의 오귀속 가능성 → **대문제 2 에 "같은 오차 이중해석 금지"**.
- 검증: 회전 ×5 + oblique 직진 ×2 한 세트에서 A·δ·카메라–IMU 차 재현; 3.5 m 정적 300 프레임에서 1/3/10 s 창 커버리지가 창 길이 무관.
- 파라미터: `C_TABLE(d, tilt)` · `SIGMA_FLOOR(d, tilt)` · `K_GATE` · `N_PERSIST` (대문제 3) · `TAG_SIZE_M` (진값으로 확정).

### 1-6 Timing — 단일 시계·캡처 타임스탬프·queue=1·stale 정책
- 결론: 단일 시계 = 호스트 epoch(`time.time()`). t_capture = RealSense global_time `get_timestamp()/1000`; **시작 시 컬러·자이로 두 스트림 `timestamp_domain` assert·로그**, 아니면 66 ms + 검출시간 폴백. run 경로에 `frames_queue_size=1`(또는 드레인) 명시 — 현재 `set_global_time` 호출 0, `CameraSettings.docking()` 의 queue=1·global_time 은 tune= 경로만, `run.py:181` 은 tune 없이 열어 **SDK 기본 큐 16** 에 실효 22~25 fps → 최대 ~0.5 s 묵은 프레임을 "현재" 로 소비 가능. `image.py:449` 상대시각 대신 절대 t_capture. 프레임별 (t_capture, t_row = t_capture + row/H·t_readout, t_arrival, t_detect_done, t_decide, t_cmd_tx) 기록. **stale: t_arrival − t_capture > 150 ms → predict-only + 경고.** dt 는 타임스탬프 차만(드롭 구간 33 ms 고정 적분 금지), 연속 dt 가 33±10 ms 밖이면 플래그. 전파는 SE(2) 이나 직진/제자리회전으로 두 경우 퇴화(pivot 상태, 카메라점 = pivot + A·u(ψ̂), σ_A 를 예측 P 에); 카메라 측정은 200 Hz 자이로 링버퍼로 t_capture 시점 ψ 에 되감아 융합. 축별 독립 외삽 불채택. 명령 지연(0.5/0.85 s) 앞 예측은 dynamics/controller 층.
- 검증: 8°/s 제자리회전 중 ψ_cam(t_capture) vs ψ_gyro(t) 상호상관 최대 지연 = τ_cam(예측 80~110 ms; ≈0 이면 타임스탬프가 도착시각) + 정지 60 s t_arrival−t_capture 추세 + 검출 스레드 인위 부하 40 ms → stale 문턱 실측.
- 파라미터: `STALE_MS` (초기 150) · `T_READOUT_MS` (D435i 컬러, 미측정) · `TAU_CAM_FALLBACK_MS` (66).

### 1-7 연속 추정기 — heading KF + 위치 KF(유니사이클), 구동기 모델 없음
- 결론: **heading KF [ψ, b_g]** — 자이로 200 Hz 예측, 카메라 갱신은 heading_valid 일 때만, 게이트 σ = √(σ_random² + b²)(b floor 없으면 재포착 innovation 1~2° 를 매번 기각 → AMBIGUOUS → wait deadlock), 정지마다 b_g 재추정(|ω| < 0.3°/s), `rate_dps` 가 마지막 샘플에 얼어붙으므로 gyro alive/gaps 게이트(stale → 예측 정지·AMBIGUOUS). **위치 KF [x_p, y_p, v]** 유니사이클(ẋ = v cosψ̂, ẏ = v sinψ̂, ψ̂ 입력), pivot 프레임; 측정은 **1-1 의 p̂_pivot 만** — d=|t| 는 3D 거리라 금지(3.9 m 에서 forward +8.5 cm, 2 m +28 cm). 회전 중 predict-only 는 σ_A·Δψ 가 작을 때만, 종료 후 oblique N 프레임 다수결로 ψ 재앵커. heading 재초기화 = 위치 상태 회전 + P 팽창 **원자 이벤트**. Q 는 명령을 상태입력이 아닌 분산 스케줄로만(forward ≥2 s 면 Q_v 작게, stop/start 직후 크게). v̂ 는 forward(거리)에서 재며 병목이 아니다(σ_v 3 m 2~5 mm/s, τ·σ_v 3~7 mm < σ_τ·v 7.5~15 mm); 램프 지연 0.3~0.5 s 는 dynamics 층 명령버퍼 prior 와 결합. 30 프레임 중앙값은 VERIFY 전용 + KF 와 2σ 대조(불일치 → 재초기화).
- 출력: (x̂, v̂, ψ̂, ω̂, σ_eff, heading_valid, **lateral_valid = σ_lat_eff ≤ LAT_TOL/2**, gyro_alive, AMBIGUOUS) → 원거리는 β 전용임을 컨트롤러가 하드 제약으로 안다(대문제 3 이관). 5 m 정렬점 lateral 은 정면 기하면 σ_lat 40~50 mm > 30 mm 라 판정 불가 후보(실측으로).
- 검증: 정속 직진 bag 3 m 구간 KF v̂ 1 s std ≤ 5 mm/s, stop 후 0.6 s 안에 코스팅 끝 ±1 cm; 5 m 정지 lateral 10회 반복 std(40~50 mm 면 5 m 판정 금지, 대각 tilt≥15° 에서 ≤15 mm 면 허용); 회전 5회 후 재포착 innovation 분포.
- 파라미터: `Q_V_CRUISE / Q_V_TRANSIENT` · `GYRO_STILL_DPS` (0.3) · `REANCHOR_FRAMES` · `LATERAL_VALID_K` (1/2).

### 대문제 1 → 실차 1순위 실험 (한 번 가서 순서대로)
1. **회전 ×5**(tilt≥20°, 3.5 m·6 m, 자이로 +3 s 적분) → A·부호·자이로 스케일·카메라–IMU 바이어스 (1-1·1-5·1-6·1-7 동시).
2. **정적 격자** 3/5/7/10 m × heading 0/±3/±6/±10/±15/±25° × 노출 2단 × 100 프레임, 왜곡 보정 전/후 → θ_min(d), b(d,tilt), pitch 양봉 σ, SPRT 임계, σ_c (1-2·1-3·1-4).
3. **oblique 직진 ×2**(5→3.5 m) + 정속 bag + 5 m 정지 10회 → δ, v̂, 5 m 판정 가부 (1-5·1-7).
4. 8°/s 회전 상호상관 + 정지 60 s 타임스탬프 추세 → τ_cam, stale 문턱 (1-6).

### 다른 대문제로 이관
- **대문제 2**: "회전 잔차 ±1.5°" 이중해석 금지(카메라–IMU 차와 분리) · 램프 지연 0.3~0.5 s 의 명령버퍼 prior · A 확정 실험과 회전 dynamics 실험 통합.
- **대문제 3**: 하드 제약 `heading_valid`·`lateral_valid`·`gyro_alive`·AMBIGUOUS · A 확정 전 β_px 조준 · AMBIGUOUS 타임아웃·fallback · 태그 yaw 오프셋 장착 시 도킹축 상수.
- **대문제 4**: timestamp_domain assert·6개 시각 로깅 · config.json 에 i.model·coeffs · 정적 격자·회전 ×5 를 현장 실험 목록 최상단으로.

### 미확정 (실차로)
A 값·부호 · x_off · 카메라–IMU Δψ 차 원인 · D435i 컬러 coeffs · θ_min(d) · h · 자이로 타임스탬프 도메인 · t_readout · 5 m lateral 유효성 · σ̂_c 창 길이.

## 대문제 2 — Vehicle Dynamics (명령→결과 예측)
_status: debated 2026-09-20 — 결론 7 / 보류 0, **사용자 검토 대기** (근거·왕복은 `log/debate_2_dynamics.md`; 상수 최소화 원칙 적용본)_

### 2-1 정지거리 — 셀별 비례 모델 + 잔차 분포, 2모드
- 결론: 셀 = (level∈{67,97}) × (dir∈{fwd,bwd}). **① 가시 정속정지**: `D̂ = τ_eff·v̂`(셀당 1파라미터, τ_eff = D̄/v̄), D_obs = **경로길이**(|Δp| 또는 Δforward/cos ψ̂; CAN write(TXACK) 시각의 KF x̂ − 정지판정 후 30프레임 중앙값). 표 {n, median, σ_e, q95⁺, max, 기대 커버리지 1−1/(n+1)}; q99 는 n≥59 전엔 "미검증". 발령 `x_rem(카메라 forward, A 무관) ≤ D̂`, **m=0**(T_f < max_n(e) 일 때만 한쪽 마진), "짧게 서고 97 크립 1회 복구" 조항 삭제(97 최소 신뢰 증분 ≈0.12 m > 복구할 3 cm). **② 정지출발 S(T) 분포표**(명령길이 T → 총이동 median·σ·CV; `fwd_time_model` 의 분포판, FWD_T0/T1/A 는 모듈 상수→캘리브 파일 값): 블라인드 최종 다리(태그 컷 3.3 m → 정지 2.02 m, 1.28 m)·정속 최소길이 미만 다리 전용, 최소 신뢰 증분 = CV<30% 최소 T. σ_S(블라인드) 가 T_f 를 못 만족하면 "dynamics 로 못 닫음" 으로 대문제 4(depth 정지/태그 높이/T 4.0 m) 에 보고.
- 검증: 3.5 m 정면(tilt<10°)·사각(30°) 67 정속정지 각 n=10 — 경로길이 D_obs 의 τ_eff 가 두 기하에서 차 <5% ∧ σ_e ≤ 2 cm; 같은 세션 97 정지출발 {2,3,4,5,6 s}×10 → S(T) CV 곡선·σ_S.
- 상수: 새 config 0 · 캘리브 파일 4셀×{τ_eff, σ_e, q95⁺, n} + S(T) 표 2레벨 · 실시간 v̂·D_obs·e(매 정지) · **삭제 `AIM_STOP_LEAD_M`, `AIM_STOP_CONFIRM`**.

### 2-2 명령 지연 — 정속 정지엔 분리 불필요, 시각은 IMU, 호스트 사건은 분리
- 결론: 정속에서 서는 모든 정지는 셀 τ_eff·σ_e 만 쓴다. 두 단 대수(`D = τ_d·v + v²/2a`)·카메라 절점 히스토그램은 추정기로 **폐기**(a 상대오차 45~90%, 절점 σ 15~30 ms ≈ 기대 σ_τ). τ_d·σ_τ 는 **200 Hz 가속도 t_resp 1차**, 카메라 절점은 부트스트랩 σ 첨부 교차검증만(σ_τ < 2×절점 σ 면 "지터 측정 불가" 보고). τ_start 는 σ_a,still 실측 후 센서(가속도/카메라) 결정, 그 전엔 최소 명령길이·워치독 캡용 보수값(q50+여유). a 는 사건별 대수 진단값(컨트롤러 미사용). **9/7 꼬리(8.23 s→0.66 m, #14 8.97 s, 185709 #3 goal 10.7° 초과·gyro-gaps)는 VM 호스트 사건** → τ_start,q99 로 모델링 금지; 모든 사건에 (loop tick 최대, CAN write 간격[TXACK], gyro gaps, 프레임 도착 간격, CAN RX) 부착, 호스트 이상은 표 제외·별도 카운트. 발령식에 `v̂·Δt_ctrl/2` 복원. plan 1-7 이관 "명령버퍼 prior" = KF 상태입력 아닌 **Q 스케줄 창** [t_tx, t_tx+τ_start+t_ramp] / [t_stop, t_stop+τ_d+v̂/a]. **CAN TX 전용 스레드 + 송신측 데드맨은 한 묶음**(값은 ros2_control/Nav2 관행 0.3~0.5 s 에서 시작, precheck loop-tick 히스토그램으로 확정; 소유 대문제 4, precheck 선행조건).
- 검증: 67 정지 n=20 — 가속도 t_resp vs 카메라 절점 τ_d 산포도(±30 ms, 부트스트랩 σ) + 97 크립 중 CAN 송신 0.5/1/2/5 s 의도적 차단 → 차량 워치독 유무·시간·마지막 명령 유지 여부.
- 상수: 새 config 0 · 캘리브 τ_start{q50}·τ_d(셀별) · 실시간 t_resp(매 명령) · **삭제 `SETTLE_SEC`(→코드 내부 캡 이관), `FWD_SCALE`, `FWD_BIAS`** · 코드 내부 데드맨 값.

### 2-3 IMU 가속도 — 사건 시각 계측기, e-stop 트리거 아님; 정지 판별 2모드, 문턱 실시간 도출
- 결론: 가속도 = 명령 창 [t_tx, t_tx+τ_q95] 안에서만 보는 **사건 시각 계측기(t_resp, 수평 성분 크기 |a_h|) + 로그 플래그**. D 입력·e-stop 트리거 아님. "응답 부재 → stop 재전송" 삭제(`movement_tx_loop` 가 10 ms 반복 송신). emergency 0x80 은 **하드 캡 위반(KF/DR 위치가 안전선 통과)에만**, precheck 에서 정지 상태 1회 송신→반응·해제 실측 후 채택. **정지 판별 2모드**: 가시 = KF v̂ 가 창 안 전 샘플 문턱 아래(Autoware 형) ∧ |ω|<γ_ω, IMU 는 확인; 블라인드 = t_resp(감속 전이) 검출 후 |a_h−ā_still|<γ_a ∧ |ω|<γ_ω 가 ≥0.3 s(200 Hz) + 시간 캡(τ_d,q95+v̂/a+0.5 s, 코드 내부); 전이 없이 τ_d,q95 경과면 "응답 부재" 경고·수동전환(정지 판정 금지). **문턱 γ 는 config 가 아니라 매 정지 정지창 지표(EMA|Δa|·std(|a|,0.3 s)·|ω|) 최대 × 여유배율 SF(코드 내부 1개) 로 실시간 도출**, 진입 0.7×/이탈 즉시(ArduPilot/PX4 형). 회전 정지는 |ω| 문턱 + alive/gaps 게이트만.
- 검증: 정지 30 s(비적재/적재 × 마스트 하강/상승 × 유압 펌프 on/off)·97 크립 30 s 의 EMA|Δa|·std(|a|,0.3 s)·|ω| 분포 분리 여부(겹치면 IMU 정지 판별 포기·카메라 v̂ 만) + 67 정지 n=20 t_resp 검출률·창 밖 오탐률(자이로·v̂·가속도 정지 세 시각 차 분포).
- 상수: 새 config 0 · 실시간 ā_still·σ_a,still·σ_ω,still(매 정지) · 코드 내부 SF·0.3 s·0.7 · **삭제 `ROT_SETTLE_MAX_SEC`, `ROT_SETTLE_MIN_SEC`, `ROT_SETTLE_POLL_SEC`, `ROT_SETTLE_RATE_K`, `ROT_SETTLE_RATE_FLOOR`, `ROT_SETTLE_RATE_CEIL`**(6개 → 도출 규칙 1개) · plan 1-7 `GYRO_STILL_DPS`(0.3) 도 같은 도출값으로 대체(상수 −1).

### 2-4 회전 정지 — 고정 ROT_LEAD_DEG 폐기, IMU 스레드 ω 규칙, L/R 셀
- 결론: `goal = |deg| − min(ROT_LEAD_DEG, |deg|/2)`(rot_control.py:102) 폐기 → 실시간 규칙 **θ_rem ≤ ω_pred·τ_r,dir + [가속중]·½α_up·τ_r² + ω_pred²/(2α_r,dir)**, ω_pred = ω̂ + [가속중]·α_up·τ_r(α_up 은 펄스 캘리브 상수, 온라인 미분 아님), 셀 L/R 분리(통합은 실측 후). **임시값은 순수지연 τ_r=0.18 s 하나**(α_r=∞; 0.18+20 동시 사용은 같은 n=1 점의 이중 역산이라 7.3°/s 에서 2.64° = 실측 2배). 캘리브 순서 = **개루프 펄스** L/R×{1.0,1.5,2.0,3.0 s}×4 + 5 s 자이로 전용 ×2(200 Hz 원시) → ω(t) 에서 τ_r·α_r·α_up 적합 → 폐루프 L/R×{5,15,45°}×6 검증. 규칙 평가는 **IMU 콜백 스레드(imu_yaw._on_frame)** 에서 CAN 상태 직접 플립(asyncio 판정 지터 8°/s×60 ms = 0.5° 회피; TX 스레드+데드맨과 묶음). 루프 안 gaps 검사 → 즉시 stop + 카메라 β 재앵커; 무응답 워치독(t_tx+τ_r,start 에 |ω|<γ_ω → 경고); 시간 캡 = ω̂ 기준 예상 ×2. 소각: |Δθ| < max(HEAD_TOL, θ_min,inc) 면 안 돎, 그 위는 ω 규칙 — 펄스 표 계층은 5° 폐루프에서 σ_θ>0.5° 일 때만; 회전 편향 {10,15,20} 스윕은 precheck 포함, 저속 회전 단은 20 에서 0.5° 실패 시만. **잔차 = 자이로 Δψ−target 만**(카메라–IMU 차 0.3~2.3° 는 perception, 1-5 이관 해소; `config/control.py:70,75` "회전 잔차 ±1.5°" 이중해석 폐기). σ_θ ≤ 0.5° 는 요건(블라인드 1.28 m × sin δψ: 1°→2.2 cm, 2°→4.5 cm > LAT_TOL).
- 검증: 개루프 3 s 펄스 L/R×4 의 200 Hz ω(t) 로 (τ_r, α_r, α_up) 적합 → 같은 15° 회전을 (a) asyncio 루프 판정 (b) IMU 콜백 스레드 판정 각 ×6 → σ_θ ≤ 0.5° ∧ |mean| < 0.3°((a)(b) 차 없으면 단순한 (a) 유지); plan 1-1 회전×5(tilt≥20°, 3.5/6 m) 와 같은 세션.
- 상수: 새 config 0 · 캘리브 L·R×{τ_r, α_r, α_up, σ_θ, θ_min,inc, n} · 실시간 ω̂·바이어스·σ_ω(매 정지) · 코드 내부 캡 배수·gaps 즉시정지 · **삭제 `ROT_LEAD_DEG`; 코드 내부로 `ROT_POLL_SEC`(IMU 스레드 평가로 소멸)·`ROT_WATCHDOG_GAIN`·`ROT_WRONG_WAY_DEG`; 캘리브 파일로 `ROT_T0`·`ROT_DEG_PER_SEC`(표시·dry-run 전용)** · `analyze_run.py:117` "평균 오버슈트 = ROT_LEAD 권장" 폐기 · `AIM_TOL_DEG`·`AIM_FINAL_TOL_DEG` 는 대문제 3 이 σ_θ 로 재도출.

### 2-5 저속 크립 byte2=97 — 이산 2단 액추에이터, 블라인드 안전속도 규칙(조건부)
- 결론: byte2 는 연속 스로틀이 아닌 **이산 2단 {67, 97} 액추에이터**로 다룬다(트랙션 컨트롤러 3사 모두 데드밴드+탈출 점프+곡률 프로그래머블, 원격모드 속도 캡 → 67 자체가 캡일 개연성; 비례는 실측 대상). 안 B = **"블라인드 구간(T 뒤 마지막 직진) 안전속도 규칙"**(0.24→~0.12 m/s: 충돌 에너지 ¼·감시자 개입 시간 2배; σ_D 절반은 블라인드 DR ±13 cm 앞에서 부차) 로 채택하되 stiction 검사에 조건부. 주행 중 67→97 전환은 1차엔 측정만. 스윕 순서: ① 97 정지출발 4 s×10 온셋 분포 — 2 s 초과·무이동 1건이면 B 보류→A(67 전구간)+마진 ② 편향 {10,15,20,25,30,45,60}(byte2 117/112/107/102/97/82/67)×4 s×3, 오름/내림 교대 → 무이동 문턱(데드밴드·히스테리시스)·탈출 점프·포화(v45=v60?)·v_ss·CV·램프·D(v); 움직인 ≥3 레벨로 D = c₀ + τ_d·v 적합, c₀ 95% CI 로 비례 검정(Klimenda C18 저속 절편 선례) ③ 97 펄스 {1.0,1.5,2.0,2.5 s}×5 → CV 곡선, 최소 신뢰 증분 = CV<30% 최소 펄스(2-1 S(T) 표와 같은 데이터; <1 s 펄스는 τ_start 안에 끝나 폐기) ④ 67→97 주행 중 전환 ×5 측정. 적재/비적재 각 1세트(Speed vs Torque 모드 판별), 배터리 전압·SOC 기록, v̂ 앵커용 T 4.0 m 셀 포함.
- 검증: 스윕 ①+② 한 세션 — 온셋 분포(2 s 초과·무이동 0건이면 B 확정) + 편향 7레벨×3 의 v_ss·D 표 → 절편·점프·포화 판정, c₀ 95% CI 0 포함 여부.
- 상수: **새 config 1: `FORWARD_SLOW`(97 템플릿 이식, 차량 종속값)** · 캘리브 편향→v_ss 표·데드존·최소 증분 · 실시간 v̂ · `TAG_CUT_MARGIN_PX` 는 v̂·D 기반 재정의 후보 → 대문제 3.

### 2-6 온라인 적응 — 없음. 고정 캘리브 + 잔차 로깅 + 경보 + 명시적 재캘리브
- 결론: EMA/RLS·v̂ 이중미분 a_obs·주행 중 자동 갱신은 쓰지 않는다(도킹 1회 = 정지 2~3건이라 EMA 는 지터 추종 = 유령 적응; D̂=τ_eff·v̂ 의 v̂ 가 run 간 −11~+15% 속도 변동을 이미 흡수 → 남는 미지수는 τ_eff 평균 이동뿐). 적재/비적재는 캘리브 2세트를 **임무 입력**으로 선택. 운영 규칙: 분위수는 (셀, 적재상태)별 최근 N 세션 창(N 코드 내부), 입력은 가시·정속·비스톨 정지만(블라인드·사각 cos 오염·호스트 정지 플래그 제외); 경보(최근 5건 |mean e| > σ_e) = 사람 알림 + **"재캘리브 전 도킹 금지" 플래그**(자동 마진 변경 없음); **캘리브 파일 부재 시 출발 거부**(dry-run 만, 삭제 상수의 코드 기본값 부활 금지); `dynamics_calib.json` 에 KF Q·τ_cam 설정 해시·호스트 종류·git·n·기대 커버리지를 넣고 불일치면 로드 거부(τ_eff 는 KF 지연을 흡수하므로 추정기 설정과 짝).
- 검증: 세션 시작·끝 67 정속정지 각 n=10 → |Δmean e| < σ_e; 적재 상태(화물 있으면) n=10 → 2세트 필요성; Q_v 2배 변경 후 재측정 τ_eff 이동량 ≠ 0 이면 KF 설정 짝 규칙 확정.
- 상수: 새 config 0 · 캘리브 세트 2 · 실시간 e·경보 통계 · 코드 내부 5건·N 세션.

### 2-7 사전실험 precheck — 안전 시험이 1순위, 원시 기록·오프라인 판정
- 결론: 원시 기록만 남기고 판정은 오프라인(`motion_started` 류 플래그를 기록 시점에 박지 않음); **호스트는 Jetson/Windows 만(VM 금지)**; 선행조건은 스크립트 assert(plan 1-6 queue=1·t_capture·timestamp_domain, 자이로 먼저 열기, `snapshot_config` + 캘리브 파일 해시). **순서 — 1일차 필수**: **(1) 안전** — 정지→97 크립·사람 제동 위치에서 heartbeat만/movement만/control(0x2E3)만/전부(busOff) 차단 각 ≥3회 + 0x2E3 카운터 고정 재생 → 차량 워치독 T_f 또는 "유지"(마지막 프레임 TXACK 시각 vs IMU/카메라 감속 개시), 끊김 정지 vs stop 프레임 정지 감속 프로파일, 복구(프레임 재개/mode+HB 버스트/키 사이클 = 래치 여부), emergency 0x80 정지 상태 1회 → 반응·해제, 송신측 데드맨 동작, Kvaser RX 스니핑(+TXACK 켜기; 차량측 EMCY/HB/상태 PDO 있으면 SDO 0x1016/0x1017 읽기 시도) **(2)** 67 정속정지 n=20 @3.5 m 정면+사각 **(3)** 97 stiction → 편향 스윕 → 펄스 **(4)** 회전 개루프 격자 → 폐루프 L/R×{5,15,45°}×6 + asyncio/IMU 스레드 A/B(plan 1-1 회전×5 통합); **2일차**: (5) 정속 장거리 8→4 m ×3×{67,97} → 런 내 σ_v/v(블라인드 DR 예산) (6) S(T) {2,3,4,5,6 s}×{67,97}×≥10 + 최종 정지 ×10 외부 줄자(태그면 기준; depth 정지 필요 여부 결정 데이터) (7) 세션 시작·끝 67 n=10. **기록**: t_cmd = TXACK 타임스탬프(+호스트 write 시각·payload), CAN RX 전량, TX 간격 최대치, loop tick, 프레임 도착 간격, gyro gaps, 직전 명령 종류(가설을 셀 이름에 박지 않음), 호스트 종류, KF 해시, epoch−monotonic 오프셋(광운대 훅 monotonic_ns ↔ 1-6 epoch), 프레임별 t_capture·β·d_h·p̂·quality·heading_valid, 자이로 200 Hz·가속도 원시, KF x̂ v̂ ψ̂, bag 동시 녹화. **진입 게이트**: 첫 10건 TXACK 간격 99% < 20 ms ∧ gyro gaps 0. 산출 `dynamics_calib.json`(셀별 값+n+기대 커버리지+날짜+git+호스트). **통과**: 67 σ_e ≤ 2 cm ∧ L/R σ_θ ≤ 0.5° ∧ 최종 정지 외부 σ ≤ T_f/3(T_f 자리); 실패 층(타이밍/동역학/블라인드)이 로그만으로 안 갈리면 "기록 항목 부족". 분위수는 n 명시·q99 금지(Wilks 단측 95/95 비모수 n=59) — 95% 보장이 필요하면 레벨 통합 회귀 잔차 풀 또는 정규성 QQ 확인 후 k-factor.
- 검증: precheck (1) 자체 — 차단 4종×≥3회의 T_f/유지 판정과 emergency 1회의 반응·해제가 TXACK·IMU·카메라 로그만으로 재구성되는가; 그 뒤 첫 10건 게이트 통과 후 본 실험 진입.
- 상수: 새 config 0 · 캘리브 파일 신설(출처 포함) · 실시간 0.

### 대문제 2 상수 회계 (순감 12)
- **삭제(config/control.py, 10)**: `SETTLE_SEC`(코드 캡 이관), `ROT_LEAD_DEG`, `ROT_SETTLE_MAX_SEC`, `ROT_SETTLE_MIN_SEC`, `ROT_SETTLE_POLL_SEC`, `ROT_SETTLE_RATE_K`, `ROT_SETTLE_RATE_FLOOR`, `ROT_SETTLE_RATE_CEIL`, `AIM_STOP_LEAD_M`, `AIM_STOP_CONFIRM`.
- **코드 내부로(3)**: `ROT_POLL_SEC`(소멸), `ROT_WATCHDOG_GAIN`, `ROT_WRONG_WAY_DEG`.
- **모듈 상수 삭제(2)**: `FWD_SCALE`, `FWD_BIAS`. **캘리브 파일로(5)**: `FWD_T0`, `FWD_T1`, `FWD_A`, `ROT_T0`, `ROT_DEG_PER_SEC`(실행경로에서는 S(T) 표·표시·dry-run 전용).
- **신설 config(1)**: `FORWARD_SLOW`(byte2=97 템플릿, 차량 종속값). **유지(물리)**: `DOCK_EXTRA_M`, 회전 편향 20.
- **대문제 3 재도출**: `AIM_TOL_DEG`, `AIM_FINAL_TOL_DEG`("회전 잔차 ±1.5°" 근거 폐기), `TAG_CUT_MARGIN_PX`(v̂·D 기반).
- **캘리브 파일 `dynamics_calib.json`**(사람이 정하는 값 아님, 출처·n·커버리지·호스트·git·KF 해시 포함): 4셀×{τ_eff, σ_e, q95⁺, n}, S(T) 표 2레벨, τ_start{q50}·τ_d, L·R×{τ_r, α_r, α_up, σ_θ, θ_min,inc}, 편향→v_ss 표·데드존·최소 증분, 적재/비적재 2세트.

### 대문제 2 → 실차 실험 (2-7 순서 그대로; plan 1 의 1순위 실험과 통합)
1일차: (1) CAN 차단·emergency·데드맨·RX 스니핑 → (2) 67 정속정지 n=20 정면+사각 → (3) 97 stiction·편향 스윕·펄스 → (4) 회전 개루프 펄스 → 폐루프 L/R×{5,15,45°}×6 + asyncio/IMU A/B (= plan 1-1 회전×5 tilt≥20°, 3.5/6 m 와 같은 세션).
2일차: (5) 정속 8→4 m ×3×{67,97} → (6) S(T) + 최종 정지 외부 줄자 ×10 → (7) 세션 시작·끝 n=10. plan 1 의 정적 격자(θ_min·왜곡)·oblique 직진은 2일차에 병행.

### 다른 대문제로 이관
- **대문제 3**: 발령식 `x_rem ≤ τ_eff·v̂ + v̂·Δt_ctrl/2`(m=0) · ω 규칙 · 정지 판별 2모드·이벤트 SETTLE · 하드 캡(예측 정지점이 안전선 안이면 즉시 stop) · 응답 부재 → 경고·수동전환 · `AIM_TOL_DEG`·`AIM_FINAL_TOL_DEG` 를 σ_θ 로 재도출 · `TAG_CUT_MARGIN_PX` 재정의 · 최소 명령길이 미만 다리는 S(T) 표.
- **대문제 4**: **블라인드 1.28 m(3.3→2.02 m) 해소** — depth 정지/태그 높이/T 4.0 m 검토, T_f 로 m·S(T) 허용 역산 · CAN TX 전용 스레드 + 송신측 데드맨(값 확정) · 차량 워치독·emergency 절차 · 캘리브 파일 관리·로드 거부 규칙 · 호스트 규정(Jetson/Windows) · TXACK 기반 timestamp.

### 미확정 (실차로)
T_f(차량 워치독)·emergency 반응·복구·RX 프레임 존재 · σ_a,still·σ_ω,still vs 97 크립 통계 · σ_τ 측정 가능 여부·τ_start 1차 센서 · v(97)·데드존·c₀·포화·stiction·주행 중 전환 램프 · 소각 유압 혼입·L/R 통합·저속 회전 단 · 적재 시 τ_eff · σ_S 가 T_f 안에 드는지 · 트랙션 컨트롤러 기종(Curtis/Zapi/Sevcon; Dropbox `철기원…/코드/` 확인 — 사용자) · 광운대 FSM v3/v4 fsm.py·configure_forward_slow 인자·실주행 로그(직접 요청 — 사용자).

## 대문제 3 — Controller (수렴 제어)
_status: debated 2026-09-21 — 결론 7 / 보류 0, **사용자 검토 대기** (근거·왕복은 `log/debate_3_controller.md`; 상수 최소화 원칙 적용본)_

### 3-0 기준점 x_ref 와 두 레버 (쟁점 3·5·6·7 의 공통 뿌리)
- 최종 기준점 lateral 오차 = **x_ref·δ**(앵커 heading 바이어스 δ; 태그 겨냥·정면 회전 어느 전략이든) + **(d_T−x_ref)·r**(T 에서의 조준 잔차 r = 회전 양자화·코스팅; β_px 로 사후 관측되나 제거 불가). config `DOCK_EXTRA_M` 주석(카메라 정지 2.02 m = 포크 끝 0.5 m, 포크 중심 ≈1.05 m) 기준 heading 1° 당 lateral: 카메라 35 / 포크 중심 18 / 포크 끝 8.7 mm.
- **x_ref 는 탑재부 사양이 정한다 — 사용자 확정 항목.** `DOCK_EXTRA_M` → `CAM_TO_REF_M`(물리: 카메라→기준점 종방향) + `STANDOFF_M`(운영) 으로 분리.
- 수치: (d_T−x_ref)·σ_θ ≈ 2.45 m × 0.5° = 21 mm, θ_min,inc/2 ≈ 0.65° 면 28 mm → T 에서의 회전 양자화가 30 mm 예산을 거의 소진. 1회 시도 |r| ≤ 0.7° 성공 ≈84%, 2회 ≈97%(judge 추정) → **99% 는 저속 회전 단(plan 2-4 조건부) 또는 레버 단축(대문제 4: 태그 가시거리·depth 정지) 없이는 불가.**

### 3-1 State machine — 전이표, DECIDE 래치, stop 전용
- 결론: 전이표(enum) OBSERVE→DECIDE→EXECUTE→STOPPING→SETTLE→VERIFY(+REACQUIRE·FALLBACK). 명령은 DECIDE→EXECUTE 전이에서만 래치, EXECUTE 이후 상태는 stop 만. OBSERVE 탈출 = 하드 플래그(heading/lateral_valid·gyro_alive·¬AMBIGUOUS) ∧ 최소 프레임(코드 내부) ∧ **직전 프리미티브 사후 일관성(3-5 ⓪) 통과**; σ 수렴은 연장 조건만. EXECUTE 중단 술어 = 종료식(plan 2-1 발령식·2-4 ω 규칙) + stale > 150 ms(plan 1-6) + 블라인드 중 gyro dead + 가시성 픽셀 여유(3-4) + 하드캡 emergency(plan 2-3). **CAN 상태 소유자는 TX 스레드 1개**(IMU 스레드·프레임 루프는 stop 요청만; plan 2-4 "IMU 스레드 직접 플립" 은 TX 10 ms 틱 경유로 구현 — 8°/s × 10 ms = 0.08° 라 양립, A/B 실험 유지). REACQUIRE = DECIDE 경유로 KF 예측 β 방향 ≤반화각 회전 1회만(360° 탐색 `next_set3_step`·FINAL 중 `recover_backup` 폐기). heading 축 판정 입력은 σ_ψ̂,eff(oblique 앵커 b ⊕ 자이로 드리프트 × 앵커 나이), `heading_valid` 는 "카메라 갱신 가능" 의미로 축소(T 정면에서 매 VERIFY hold 방지).
- 근거 요약: 181437 8→9→10 반전은 settle+30프레임 뒤 정지 측정에서 나옴 → Failure 3 이 아니라 측정 문제; 185709 결정 1~3 은 stable:false 인 채 실행(`plan_step` 은 stable 을 로그에만 씀); `_hold` 는 SETTLE_SEC 시간 대기만.
- 검증: 3.5 m·tilt 25° 정지 → β_px 조준 회전 ×10, VERIFY 가 hold 없이 판정·VERIFY 후 반전 ≤1/10; 검출 스레드 300 ms 부하 주입 → stale 술어 stop(CAN 로그 시각 대조). KPI = "VERIFY 후 반전 ≤1/도킹"·"T 구역 회전 ≤2"·SETTLE 캡 발화율.
- 상수: 새 config 0 · 코드 내부 최소 프레임·캡 · 삭제 `HOLD_MAX_CONSEC`, `MAX_STEPS`(→ 3-7 예산).

### 3-2 pivot decouple — A 실시간 도출 + A-민감도 최소 배치
- 결론: 결합은 plan 1-1 `T_tag←pivot` 에서 한 번 끊는다. **A 는 회피가 아니라 실시간 도출**: precheck 회전(tilt≥20°, 3.5/6 m ×5)으로 자이로 스케일 s 와 A 를 분리한 Â 가 초기값, 도킹 중 **매 회전마다 프레임 회귀** `Δβ_px = s·(1+Â·cosβ/d)·Δψ_gyro` 로 갱신(잡음 미분 아님), σ_A → KF P. 배치 규칙은 "A-민감도 최소" 로 유지: 태그 겨냥은 β_px = δ̂(CAM_YAW_OFFSET; A 무관), 축 위 J 겨냥은 게이트 σ_A·sinβ/d ≤ HEAD_TOL/k 통과 시만(미달이면 태그 직접 겨냥으로 d 를 줄인 뒤 재평가), 정지는 카메라 forward(plan 2-1). x_off(카메라↔포크 중심 횡오프셋)는 줄자 1회 물리 상수.
- 근거 요약: β_cam=0 ⇒ β_piv=0 은 태그 자체 겨냥에만 성립; J 겨냥은 d_J 3 m·β 20°·A ±0.4 → 17.7°/23.0°, 다리 끝 lateral ≈13 cm(d 무관); 정면 회전이 카메라를 A·sinα 옮김(σ_A 0.5·20° → ±17 cm).
- 검증: 회전×5 로 Â·s 확정 후 J 겨냥 3 m 다리 ×3 을 A=0 / Â 로 각각 실행 → 도착 lateral 차 ≈ Â·sinβ; 8 m 첫 조준 회전 1회의 Â 가 precheck Â 와 ±0.05 m.
- 상수: 새 config 0(`A_M`·`X_OFF_M`·`CAM_YAW_OFFSET_DEG` 는 plan 1 캘리브 항목) · 실시간 Â·σ_A · 삭제 `CAM_TO_PIVOT_M`.

### 3-3 계획 — 가이드 + 부등식 2개, 비용함수 없음
- 결론: 가이드 순서 = **대각 다리(x_J = T 기본, α* = atan(ℓ/(d−x_J)) 기하 도출) → 태그 겨냥 회전(β_px = δ̂) → 블라인드 직진(97, plan 2-1 S(T)·발령식)**. 비용함수·σ_J·5후보 롤아웃·N-step 지평은 도입하지 않고 J 는 로그만("필요 증명 후"). 실제 결정은 부등식 2개: ① 재조준 vs 계속(가시성·β 재중심, 3-4) ② **T-VERIFY 수용식** `|F̂| + k·σ_F ≤ T_l ∧ |atan(ℓ̂/d_T)| + kσ ≤ T_h`, F̂ = ℓ̂·x_ref/d_T + (d_T−x_ref)·r̂(회전 후 β_px 잔차), σ_F = x_ref·σ_ψ̂,eff. 롤아웃은 가시성·정지점 feasibility 검사로만. 재접근 = V-다리 식 폐기 → **현재 heading 으로 97 후진**(b ≥ 최소 신뢰 다리 ∧ 후진 후 α'' = atan((ℓ+b·sinα)/(d+b·cosα−x_J)) ≤ 가시성 해, 상한 = 직전 전진 경로길이; 후방 무관측은 대문제 4 잔여 리스크).
- 근거 요약: 9/7 step 12~15 는 1-step 지평 탓이 아니라 정면원뿔 heading 바이어스(Δcam 7.02° vs IMU 5.61°, 비 1.25) → perception 게이트가 고칠 오차의 이중해석; 직선 후진은 축 교점 불변(step 15 후진에 lateral −0.196→−0.918); `AIM_FINAL_MAX_LAT_M 0.10` 위반 단정은 x_ref 미정이라 철회.
- 검증: 포락선 안 동일 시작자세(ℓ≈1.5 m, d≈7 m) ×5 → 프리미티브 ≤8·반전 0 이면 비용함수 도입 근거 없음 확정; 정면 회전 직후 F̂(x_ref) 예측 → 최종 정지 외부 줄자 포크 lateral ×10 → |F̂−실측| σ ≤ x_ref·σ_ψ̂ 이면 식 확정(카메라 기준이면 예측이 실측의 2~4배 = 기준점 검정).
- 상수: 새 config `STANDOFF_M`(운영) · 물리 `CAM_TO_REF_M` · 실시간 F̂·σ_F·α*·재접근 b · 삭제 `AIM_FINAL_MAX_LAT_M`, `AIM_BACKUP_M`, `AIM_CHUNK_MAX_M`, `AIM_MAX_BACKUPS`, `STEP_M`, `FWD_SAFETY`(v3 src/tools 참조 0).

### 3-4 태그 가시성 — 하드 제약 + 관측값 중단 술어, 소각 규칙, 포락선
- 결론: 가시성은 cost 항이 아닌 **하드 제약(허용집합)**: 롤아웃 전 구간 |β_tag| ≤ β_vis(d) 와 상단 컷 예측을 만족하는 프리미티브만 후보. 실행 중 중단 술어는 **관측값** `margin_px − v̂·(τ_eff + t_now−t_capture)·∂margin/∂x − pitch_px ≤ 0`(pitch_px = precheck 67 정지 n=20 의 태그 상단 픽셀 최대 상승량) 로 실시간 도출; 해석식 β_vis·x_cut(h, VFOV, pitch) 은 로그 대조용(h 미확정, `tag_edge_margin_px` docstring "화면을 직접 보는 쪽이 무조건 맞다"). "태그 화면 중앙 유지" 는 이 하드 제약 + β 재중심 회전으로 구현, β=0 문자 그대로는 직접 겨냥 모드만. α* 는 기하 도출(선택 상수 아님); θ_min 하한은 σ_ψ̂,eff(앵커 나이×드리프트)가 σ_F 예산을 넘을 때만 x_J 를 뒤로 물리는 파생 조건(축 잔여 ≥ 최소 신뢰 다리 ∧ σ_F 개선 ≥ kσ). **포락선 밖**(|ℓ| > (d−x_J)·tanβ_vis; 9/7 시작 ℓ/(d−x_T)=0.98 > tan31°=0.62 → 태그 보며 축 합류 불가) = FAR 한정 자이로 **dog-leg 1회**(97, 블라인드 ≤ 최종 블라인드 다리급 ≈2 m/≤10 s, 재획득 후 30프레임 branch 재초기화 전 NEAR 결정 금지), 그래도 밖이면 출발 거부. **시작 포락선(8 m 에서 |ℓ| ≲ 2.8 m) 은 운영 요구사항 → 대문제 4.**
- 검증: 3.5/6 m 정지 저속 회전으로 검출 끊기는 β 좌우 ×3(해석식과 >3° 차면 관측값 채택) + 67 정지 20건 태그 상단 픽셀 상승 → pitch_px; 9/7 시작자세 dog-leg 1회 → β_px 재획득·30프레임 후 heading_valid ≤5 s·포락선 안 판정; 포락선 안 시작 20회 태그 손실 0.
- 상수: 새 config 0 · 실시간 margin 술어·α*·β_vis(d) · 캘리브 pitch_px·검출 한계 β(관측) · 삭제 `TAG_CUT_MARGIN_PX`, `AIM_MAX_BEARING_DEG`, `AIM_MAX_TAG_OFF_DEG`, `SIDESTEP_BACKWARD_GAIN_DEG`, `SEARCH_BACKUP_M`.

### 3-5 행동 게이트 — 5항, 교차센서 사후 일관성이 첫째
- 결론: **⓪ 프리미티브 사후 일관성(교차센서 중재)** — 회전 후 |Δψ_cam − Δψ_IMU| 와 |Δlat| ≤ Â_max·sinΔψ + k·σ_random(b 미포함) 대조; 초과면 카메라 재앵커·반전 금지, ψ̂ 는 자이로 증분 전파, lateral = d·sin(β_px − ψ̂) 재계산 후 판정; 카메라 재앵커는 innovation < kσ_random 이거나 앵커 나이 한도(3-7 도출값) 도달 시 30프레임 다수결만; 불일치 반복 → heading 축 판정불가 → Tier 1. ① 지속성(프레임 잡음용, N 코드 내부). ② **3분할**: 행동 `|e| > max(k·σ_eff, δ_q)` · 수용 `|e| + k·σ_eff ≤ T` · 판정불가 `k·σ_eff > T/2 → Tier 1 즉시`; 겹치면 수용 우선; δ_q = 최소 신뢰 증분의 결과공간 값(회전 θ_min,inc·(d_T−x_ref), 직진 97 최소 증분). ③ 최소 기대개선은 F̂ 공간에서. ④ 반전 가드(정지 후 오차가 T+kσ 를 넘어 0 을 지났을 때만, 구역당 1회) 는 ⓪ 뒤에. 회전 데드밴드는 HEAD_TOL 이 아니라 결과공간 LAT_TOL/(2·(d_T−x_ref))·θ_min,inc·kσ_θ 에서 도출.
- 근거 요약: proposer 의 4항 AND 를 181437 8→9 에 적용하면 전부 통과해 같은 반전을 냄(critic 재생); 원인 = oblique(θ_min 통과)에서 카메라 Δψ 가 IMU 의 0.67·0.60 배 → lateral −104 mm(제자리 회전이면 ≤47 mm 만 가능) — 정지마다 재앵커가 새 바이어스 표본을 주입하는 반전 기구, σ̂(점프/stderr 38×)는 못 봄(Failure 2); 행동/수용 문턱 사이 공백 (T−kσ, T+kσ) 은 항상 비어 있지 않음(σ 12 mm → (6, 54) mm 가 hold→캡→abort).
- 검증: 3.5 m·tilt 25° 정지에서 IMU 회전 ±5° ×10 → Δψ_cam/Δψ_IMU·Δlateral 분포; |Δψ_cam−Δψ_IMU| > kσ_random 비율 >10% 면 "정지마다 재앵커" 금지 확정, 자이로 전파 lateral 의 회전 전후 차 ≤ Â·sinΔψ + 2σ; 프레임별 bag 재생으로 T 구역 VERIFY 표본 (|e|,σ) 산점도에 세 영역 → 공백 표본 0.
- 상수: 새 config 0 · 코드 내부 k·N·반전 1회 · 실시간 σ_eff·δ_q·데드밴드 · 삭제 `AIM_TOL_DEG`, `AIM_FINAL_TOL_DEG`, `AIM_ABORT_WINDOW`, `AIM_DRIFT_ABORT_DEG`(직진 중단 술어는 `fwd_abort_deg` 하나, 레버 = x_cam_now − x_ref).

### 3-6 Zone — 결정 유의성 + 이벤트, 거리 상수 0
- 결론: **FAR** = `|ℓ̂| > k·σ_lat`(방향 결정만 유의; 67; β 재중심·대각 다리 진행; 30 mm급 lateral/heading 판정 금지) · **NEAR** = `σ_F = x_ref·σ_ψ̂,eff ≤ T_l/2`(기준점 최종 lateral 판정 유의; heading_valid 인 oblique 다리에서 진입) · **FINAL** = "태그 겨냥 회전 완료 + T-VERIFY 수용식 통과" 이벤트로만 진입(회전 전면 금지, 97, heading 은 자이로 짧은 다리, gyro dead → stop·ABORT). σ_ψ̂,eff 의 입력(b(d,tilt)·σ_floor·θ_min(d)·Â·s·x_off·δ̂·h·β_vis)은 **`perception_calib.json`** 으로 묶고 부재/불일치 → dynamics_calib 과 같은 **출발 거부**(임시 거리 규칙 없음). 전환 판정은 VERIFY(정지)에서만; 거리 성분 hysteresis = 다음 다리 계획 속도의 D̂(v_level, dynamics_calib) + 2σ_x; T 파생 여유 = q95⁺(dynamics_calib) + 제동피치 px.
- 근거 요약: σ_floor 격자 전엔 σ̂ 만으로 8.15 m(spread 2.9 mm)에서 NEAR 진입 = v1 유령 추종 부활; 8 m SNR 30 이라 β_J 방향 결정은 가능(FAR lateral 금지는 가이드와 모순); NEAR 를 d 레버(3.5 m → 61 mm/°)로 재면 x_ref 레버보다 1.7~3.3배 엄격.
- 검증: oblique 5→3.5 m 직진(plan 1 실험 3) ×10 의 σ_ψ̂,eff 시계열로 NEAR 진입 거리 분포 — 3.5 m 이전 진입 ≥9/10 이면 정의 확정, 미달이면 대문제 4(태그 높이·T); 정적 격자 후 5 m·tilt 20° 정지 lateral 10회 std 로 표가 NEAR 거리를 정하는지(사람 상수 0); 경계 ±2σ_x 잡음 주입 시뮬 FAR↔NEAR 왕복 0.
- 상수: 새 config 0 · 캘리브 파일 `perception_calib.json` 신설(부재 → 출발 거부) · 실시간 σ_F·hysteresis·여유 · 삭제 `AIM_NEAR_T_M`, `AIM_AT_T_M`, `AIM_STANDOFF_M`.

### 3-7 Deadlock — 진행도 예산 + 시간 예산 1개, 원인 분기 fallback, DONE-unverified
- 결론: 카운트 상수 5개 → **진행도 = 단계 완료 이벤트 카운트**(다리 종료·태그 겨냥 회전 완료·T-VERIFY 통과; J 는 로그만) + **시간 = `DOCK_TIME_BUDGET_S`**(운영자 값). **Tier 0 WAIT** = 판정불가 플래그 ∧ σ 가 아직 줄어드는 중(캡 코드 내부). **Tier 1 REPOSITION** = 관측성 개선 프리미티브(원거리 → 대각 진행, 정면 → 자이로 앵커; 한도 = σ_ψ̂(t)·x_ref ≤ T_l/2 가 깨지는 시각 = LAT_TOL/(2·x_ref) ÷ b_g 재추정 후 실측 랜덤워크 — 고정 60 s 아님). **Tier 2 원인 분기 1회**: T-VERIFY 수용식 초과 → 97 재접근 / 태그 실종(FINAL 제외) → REACQUIRE / 블라인드 직진 중 자이로 Δψ > fwd_abort_deg(x_cam_now−x_ref) → stop → 97 후진 → 재VERIFY / **양자화 공백**(|F̂| 수용 밖 ∧ < δ_q) → 즉시 Tier 3 + "저속 회전 단 필요" 로그(plan 2-4 조건부 항목 승격 근거). **Tier 3 ABORT** = stop 프레임 + heartbeat **지속 송신**(버스 침묵 금지; 차량 워치독 미확인·Curtis PDO Timeout 0 = 마지막 명령 유지) + 운영자 알림; 집합 = gyro dead(블라인드)·추정기 재초기화 ≥2·calib 부재·포락선 위반·응답 부재(plan 2-3)·하드캡·시간 예산 초과. **FINAL 은 대문제 4 해소 전 DONE-unverified(DR) 로 로그 표기**(최종 pose 를 카메라가 못 봄; 검증은 외부 줄자).
- 근거 요약: 181437 measure #21 뒤 final 5.97 s 직진의 최종 pose 기록 0 → "FINAL VERIFY 실패" 는 관측 불가; J 는 정면 회전 직후 A·sinα 이동으로 일시 증가 → 정체 오판; 60 s 는 새 상수이자 틀린 레버; 일괄 재접근은 양자화 원인이면 재현.
- 검증: 고장주입 4종(AMBIGUOUS 10 s·lateral_valid False·자이로 표류 2°·양자화 공백: 정면 회전 뒤 F̂ ∈ (T−kσ, θ_min,inc·(d_T−x_ref))) 각각 캡 안 Tier 도달·반전 0 + CAN 로그 stop 프레임 연속; 최종 정지 ×10 외부 줄자(plan 2-7(6)) — σ > T_f/3 이면 unverified 유지; 정지 60 s 자이로 랜덤워크 실측으로 Tier 1 한도 갱신.
- 상수: **새 config 1 `DOCK_TIME_BUDGET_S`** · 코드 내부 캡·재시도 1·재초기화 2 · 실시간 진행도·Tier 1 한도 · 삭제 `HOLD_MAX_CONSEC`, `MAX_STEPS`, `AIM_MAX_BACKUPS`, `SEARCH_MAX_ROUNDS`, `SEARCH_AFTER_MISSES`.

### 대문제 3 상수 회계 (순감 ≈20)
- **삭제(config/control.py, 22)**: `CAM_TO_PIVOT_M`, `TAG_CUT_MARGIN_PX`, `AIM_MAX_BEARING_DEG`, `AIM_MAX_TAG_OFF_DEG`, `AIM_TOL_DEG`, `AIM_FINAL_TOL_DEG`, `AIM_NEAR_T_M`, `AIM_AT_T_M`, `AIM_CHUNK_MAX_M`, `AIM_FINAL_MAX_LAT_M`, `AIM_BACKUP_M`, `AIM_MAX_BACKUPS`, `AIM_ABORT_WINDOW`, `AIM_DRIFT_ABORT_DEG`, `AIM_STANDOFF_M`, `SIDESTEP_BACKWARD_GAIN_DEG`, `SEARCH_BACKUP_M`, `SEARCH_AFTER_MISSES`, `SEARCH_MAX_ROUNDS`, `HOLD_MAX_CONSEC`, `MAX_STEPS`, `STEP_M`·`FWD_SAFETY`(미사용).
- **신설 config(2)**: `DOCK_TIME_BUDGET_S`(운영), `STANDOFF_M`(운영). **`DOCK_EXTRA_M` → `CAM_TO_REF_M`**(물리, x_ref 확정 후).
- **유지**: `LAT_TOL_M`·`HEAD_TOL_DEG`(탑재부 사양), `CAM_TO_REF_M`(물리), plan 2 `FORWARD_SLOW`.
- **캘리브 파일 신설 `perception_calib.json`**(부재 → 출발 거부): b(d,tilt)·σ_floor·θ_min(d)·Â·s·x_off·δ̂·h·β_vis(관측)·pitch_px. `dynamics_calib.json` 과 대칭.
- **폐기된 proposer 항목**: σ_J·비용 가중·V-다리 식·A 회피 단독·heading_valid hold·60 s 자이로 상수·"정지 후 2 s 반전 0" KPI.

### 대문제 3 → 실차 실험 (plan 1·2 실험에 얹음)
- 1일차 (4) 회전 세션에 추가: 3.5 m·tilt 25° 정지 IMU 회전 ±5° ×10(사후 일관성 분포, 재앵커 금지 판정); J 겨냥 3 m 다리 ×3 (A=0 / Â); 3.5/6 m 저속 회전으로 검출 끊기는 β 좌우 ×3.
- 2일차: 포락선 안 시작(ℓ≈1.5, d≈7) ×5 → 프리미티브 수·반전·T 도착 lateral; oblique 5→3.5 m ×10 의 NEAR 진입 거리; 최종 정지 ×10 외부 줄자로 F̂(x_ref) 검정; 9/7 시작자세 dog-leg 1회; 고장주입 4종; 검출 스레드 300 ms 부하 주입.
- 오프라인: 프레임별 bag 재생으로 (|e|,σ) 산점도·게이트 통과 결정 수; 시뮬 A∈{−0.5,0,+0.5} × 시작자세(갈림 지표 = 첫 정면 회전 뒤 재접근 0 vs ≥1); 경계 잡음 주입 FAR↔NEAR 왕복 0.

### 다른 대문제로 이관
- **대문제 4**: **(d_T−x_ref) 단축**(태그 가시거리·태그 높이·depth 정지) = r-레버의 유일한 결정적 해소 · 시작 포락선(8 m 에서 |ℓ| ≲ 2.8 m) 운영 요구 · 재접근 후진의 후방 무관측 · DONE-unverified 의 외부 검증 절차(줄자 ×10, T_f) · perception_calib.json 관리·로드 거부 규칙 · ABORT 의 물리 동작(stop+heartbeat 지속)과 차량 워치독 · KPI(VERIFY 후 반전·T 구역 회전·SETTLE 캡 발화율·abort 율·재접근 성공률).
- **plan 2-4 승격 근거**: 양자화 공백 발생 시 "저속 회전 단(편향 10/15) 필요" 로그 → 조건부 항목을 필수로 올릴 데이터.

### 미확정 (사용자·실차)
~~x_ref(사용자)~~ **확정: 포크 끝 1.52 m** · **탑재부 진입 깊이 L_dock(미측정, 2 m 가정)** · 카메라–IMU Δψ 차 원인(어느 센서가 옳은가; ⓪ 는 "반전 금지·전파" 이지 "자이로가 진실" 이 아님) · SETTLE 문턱 vs 97 크립 진동 겹침 · 8 m 단일 회전 Â 분해능 · 검출 한계 β·pitch_px · NEAR 진입 거리(태그 컷 전 σ_F ≤ T_l/2 가 한 번도 안 되면 대문제 4 하드웨어) · 재접근 1회 성공률(≈84%) · perception_calib 전 시연은 dry-run 만.

## 대문제 4 — System / Integration (Timing·오차분리·KPI·failure modes·블라인드·시뮬·실험·문서)
_status: debated 2026-09-21 — 결론 8 / 보류 0, **사용자 검토 대기** (근거·왕복은 `log/debate_4_system.md`; 상수 최소화 원칙 적용본)_

### 4-1 Timestamp — 노출중심 캡처시각, GLOBAL 대기, 2-지연 검정
- 결론: 단일 시계 = 호스트 epoch, 프레임마다 6시각 (t_capture, t_arrival, t_detect_done, t_decide, t_cmd_set, t_cmd_tx) 을 `frame.jsonl` 에. **t_capture = 노출중심 = get_timestamp() − (FRAME_TIMESTAMP − SENSOR_TIMESTAMP)/1000**(컬러 메타가 있으면 프레임별, 없으면 세션 실측 Δ_FS; get_timestamp() 는 USB 전송 시작이라 depth 예시로 노출중심과 36.8 ms 차) **+ 태그 중심 행 보정 (row − row_ref)·T_line(≈30 µs, 실측 확인; 1080 행 ≈32 ms, 화면 중앙 태그 ≈16 ms)**; 자이로는 get_timestamp()(global). 도메인은 첫 프레임 assert 가 아니라 **컬러·자이로 둘 다 GLOBAL_TIME 전환까지 대기**(코드 내부 타임아웃; 시작 직후 HARDWARE_CLOCK 은 정상) → 미전환이면 **출발 거부(dry-run 만, 66 ms 류 상수 폴백 없음)**; 런타임 |ts − host_now| 상한·역행 감지 → stale/predict-only + 플래그; 세션 시작 `hardware_reset`(2.56.x 는 71.6 min 랩 역행 미수정). **통과 = L = t_arrival − t_capture ∈ [50, 150] ms ∧ |ε| = |argmax xcorr(ψ_cam@t_capture, ψ_gyro)| ≤ 15 ms(행 보정 후; ε 는 세션 상수로 KF 적용) ∧ lag(ψ_cam@t_arrival) ≈ median L**(2-지연 검정; 한쪽만 0 이면 도메인 오류). t_cmd_tx = TX 태스크에서 **payload 가 바뀐 첫 ch.write() 직후 time.time()**(광운대 `_write` monotonic_ns 훅 + epoch 오프셋 1개); TXACK 는 TX 태스크 안 read(timeout=0) 로 배수하는 **감사 전용**(잔차 하한 포락선 = 오프셋, >20 ms = FM8a 플래그).
- 검증: 정지 60 s + 8°/s 제자리회전 ×3 을 태그 화면 중앙·상단 두 행 위치에서 → L 히스토그램(median·p99)·행별 ε(행 차 ≈15 ms 면 롤링셔터 확인); 검출 스레드 300 ms sleep 주입 재실행 → L 만 +300 ms·ε 불변이면 통과, ε 도 이동하면 도착시각 스탬프(실패). 컬러·자이로 GLOBAL 전환 시각, TOA − get_timestamp(장치→SDK) 와 wait_return − TOA(큐) 분리 기록.
- 상수: 새 config 0 · 코드 내부 GLOBAL 대기 타임아웃·STALE · **삭제 `TAU_CAM_FALLBACK_MS`·`T_READOUT_MS`(→ 실측 Δ_FS·T_line)·`STALE_MS`(config → 코드 내부)**(plan 1-6 정오표) · 실시간 L 통계·ε·Δ_FS·TXACK 오프셋.

### 4-2 오차 분리 — 캘리브 사슬 2층 로드 규칙, 회귀 폐기, 설계실험 1개
- 결론: 사슬 = **`timing_calib.json` → {`perception_calib.dynamic`(A·s·ε 등 이동 종속), `dynamics_calib.json`}; `perception_calib.static`(b(d,tilt)·θ_min·σ_c·SPRT 임계·flip 빈도)은 무의존(Day 0 생산)**. 로드 규칙 2층 — **정책 해시**(queue=1·domain=GLOBAL·stale 규칙·KF Q 스케줄·git·호스트 종류) 불일치 = 거부, **측정값은 허용창**(|ε_session − ε_calib| ≤ 15 ms ∧ gyro gaps = 0 ∧ L p99 < stale). precheck 는 앞 단계 게이트 미통과 시 dynamics 셀 진입 거부(게이트 "TXACK 첫 10건 max < 20 ms ∧ gaps 0"). 오차 귀속 회귀(잔차 vs L)·시각 기반 D_obs·"이동 중 perception 바이어스" 분류는 **폐기**(queue=1 이면 σ_L 10~20 ms → 효과 3~6 mm < σ_e 라 식별 불가 = 거짓 안심). 스탬프 무결성은 설계실험 하나 — **L 주입 {0, 150, 300} ms × 67 정속정지 n≥5 → τ_eff 이동 < σ_e/2**; 진단 로그 |KF x̂(t_cmd) − t_cmd 에 가장 가까운 원시 프레임 pose| 중앙값 < 1 cm(= KF 지연).
- 검증: 같은 날 타이밍 게이트 2회(아침·오후) → 두 timing_calib 모두로 dynamics_calib 로드 허용(정책 동일·|Δε| ≤ 15 ms) ∧ L 주입 3수준에서 τ_eff 이동 < σ_e/2. "주입 없는 회귀만 하면 아무것도 판별 못 함" 명시.
- 상수: 새 config 0 · 캘리브 파일 신설 1(timing) · 실시간 KF−원시 차.

### 4-3 99% 성공 정의 — 평행사변형 허용영역, wrong-DONE 1차 KPI
- 결론(9/21 정정 반영): 성공 = x_ref(**포크 끝, 확정**)에서 **max(|e_l|, |e_l − L_dock·tan e_h|) ≤ c**(진입 시작점 ∧ 최심부 통과점; 다이아몬드 |e_l|+L_dock·tan|e_h| ≤ c 는 넓이 절반의 진부분집합 → e_l=+c 에서 진짜 허용 e_h = 1.7°(L_dock=2) 인데 0° 로 과보수), **c = LAT_TOL_M = 0.030 (실물 여유, 사용자 확정)**, L_dock = 탑재부 진입 깊이(미측정 → 2 m 가정; 실측 시 c 고정·L_dock 만 갱신); trial 당 여유 m = c − max(…) 스칼라 보고(m 분위수 = 성공확률); 정규화 다이아몬드는 controller 비용 전용; 전후 [−FWD_TOL_M, +min(FWD_TOL_M, 안전선)]. **KPI 서열 = wrong-DONE 0/n(하드, FM10) → 결합영역 성공률(n, 실패수, 95% 하한; 0 실패 n=20 ≥86%, n=59 ≥95%) → abort 율(Tier·원인) → m 분위수·프리미티브 수·VERIFY 후 반전·T 구역 회전·SETTLE 캡 발화율·WAIT 시간·stale·gaps·호스트 이상·DONE-unverified 비율**. "99%" 는 시뮬 + n≥59 로만(n≈300 = 10~15 h). 시작 격자 = plan 3-4 포락선 안 명목 셀(8 m 전체, 6 m |ℓ| ≤ 1 m)과 밖 dog-leg 셀 분리 보고. 진값 = 근거리 태그 승격 후에도 매 세션 첫·끝 5회 줄자/레이저 3축(heading 은 포크 양끝 2점). KPI 는 구성(근거리 태그 유/무)별 분리.
- 검증: 명목 셀 n=20 최종 정지 외부 줄자(포크 끝 lateral·heading) → (e_l, D·tan e_h) 산점도에 평행사변형·다이아몬드 겹쳐 판정 갈리는 trial 수; wrong-DONE 0 ∧ 평행사변형 ≥17/20 → Stage A 통과, wrong-DONE 1건이면 FINAL 수용식(3-3) 재설계.
- 상수(9/21 정정): **유지 `LAT_TOL_M`(=c, 실물 여유 0.030) · `HEAD_TOL_DEG`(2.0 → **0.5**, L_dock=2 m 에서 유도) + 신설 물리 `DOCK_DEPTH_M`(L_dock, 잠정 2.0 = 미측정 가정) + 운영 `FWD_TOL_M`** · **삭제(해당 없음으로 폐기) `POCKET_W_M`·`FORK_W_M`·`INSERT_DEPTH_M` — 팔레트 삽입 전제의 유물** · **삭제 `STABLE_LATERAL_M`·`STABLE_HEADING_DEG`·`MAX_HEADING_SIGMA_DEG`·`STABLE_SPREAD_K`·`MEASURE_FRAMES`·`MEASURE_MAX_FRAMES`**(→ VERIFY 전용 코드 내부).

### 4-4 Failure modes — FM1~7 + FM8a/8b·9·10·11, 1:1 감지·대응·시험
- 결론: 각 FM 에 로그 코드 + 실시간 감지 + 대응 상태 + 고장주입 시험. **FM1** wrong-branch = pitch-cue SPRT(plan 1-3) ∧ **직진 다리 ≥1 m 일관성**(거짓 해 Δψ_cam ≈ −2Δφ 드리프트 vs 자이로 0, 또는 sign·크기 Δβ_obs vs Δβ_pred(ψ̂)) → AMBIGUOUS→REACQUIRE oblique; **회전 후 |Δψ_cam − Δψ_IMU| 는 부호·크기 모두 branch 판별력 없음(거짓 해 비 1−2A/d = 0.77~1.0) → FM2 전용**(plan 3-5 ⓪ 정오표). **FM2** false confidence = 1/3/10 s 창 z-커버리지 + NIS 경보 → σ_floor. **FM3** ping-pong = "결정 재생 반전 0"(181437, 개루프) + STOPPING 중 반대부호 프레임 주입 → 명령 0; 폐루프는 시뮬 G2. **FM4** deadzone = 명령 후 τ_q95 안 t_resp 부재·δ_q 공백 → Tier 3 + "저속 회전 단 필요" 로그. **FM5** 오차혼합 = 4-2 게이트 미통과 시 피팅 거부. **FM6** wait deadlock = WAIT 중 dσ_eff/dt ≈ 0(σ_eff = max(cσ̂, σ_floor); floor 지배면 즉시 Tier 1). **FM7** zone chattering = **구역 단조**(FAR→NEAR→FINAL 역행 없음; σ_F 재증가는 후퇴 아닌 Tier 1 이벤트, 상수 0). **FM8a** 루프/센서 스레드 지연 = TX 전용 스레드 데드맨 stop·gyro gaps 즉시 stop(시험 = 검출 스레드 300 ms sleep). **FM8b 호스트 동결 = 소프트웨어 대응 없음** — precheck (1) 로 차량 워치독 T_wd·래치·복구 실측 전 **"미완화 리스크"** + 사람 제동 위치 + 97 한정(시험 = `kill -STOP` 2 s). **FM9** CAN = canlib TXACK 부재·오류프레임·오류카운터·error passive·bus-off + 차량 EMCY 0x08x + 카메라/IMU 속도로 "적용" 확인; 복구 = stop(127) 홀드 → driving_mode(0x2E3) → NMT Start/키 순서 실측해 래치 분류. **FM10** wrong-DONE(하드 KPI). **FM11** 이중 송신원 = precheck 를 Kvaser listen-only 60 s 로 시작; 리모컨 수신기 살아 있으면 동일 데이터 잠복·다른 데이터 비결정 bus-off → 수신기 차단 규칙 + 명시적 RELEASE 상태(movement 송신 중단·heartbeat 만)를 조이스틱 개입 ×3 실측 후 확정. `HEARTBEAT_DATA=[0x00]` 은 CiA 301 boot-up 값(operational 0x05) → precheck 수용 차이 항목.
- 검증: precheck (1) ① listen-only 60 s ② 차체 고정·사람 제동 위치 97 크립 중 heartbeat/0x1E3/0x2E3/전부 차단 각 ×3 → T_wd·잔류속도(Zapi 형)·EMCY ③ 복구 순서 → 래치 분류 ④ kill -STOP 2 s ⑤ 수신기 켠 채 동일/다른 데이터 → bus-off 시간 ⑥ PC forward 중 조이스틱 stop ×3 ⑦ heartbeat 0x05 vs 0x00. 오프라인: 9/7 bag 회전 사건에 거짓 해 강제 → Δψ_flip/Δψ_IMU 0.8~1.0 이면 회전 기반 branch 판별 삭제 확정; oblique 직진 재생에서 거짓 해 드리프트 ≥2°; 고장주입 8종 ×3 전부 캡 안 Tier 도달 ∧ CAN 로그 stop 프레임 연속.
- 상수: 새 config 0 · 실측 T_wd(hb·pdo)·래치 유형·복구 순서·잔류속도 · 코드 내부 카운터·문턱은 plan 1·2·3 도출값 재사용.

### 4-5 블라인드 최종구간 — 근거리 태그 2장 + depth 캡, T 4.0 m 폐기
- 결론: **1순위 B′ = 카메라 높이 근거리 태그 2장**(탑재부에 고정 → 이 쌍이 기준; 배치 x·베이스라인 B 는 도면/줄자 물리상수; 원거리 태그 1 의 횡·높이만 공시야 30프레임 실시간 도출, 깊이 미도출 — 4~6 m 에서 0.2 m 태그 38~52 px 라 크기 역산 0.5 px = 5 cm). **FOV 제약(도출값, 상수 아님)**: 최종 카메라–태그면 거리 d_end 에서 |β_tag| + 태그 반각 ≤ HFOV/2(34.5°). 내일 실험은 d_end = **2.02 m(STANDOFF)** → B=0.6 정면이면 |β|=16.5° 로 여유; 실전은 탑재부 깊이만큼 더 들어가 d_end 가 작아지므로 B 를 줄이거나 태그를 안쪽에 둬야 한다(d_end = 0.9 m·B=0.6 이면 34.4° = 컷 직전) — **탑재부 길이 실측과 묶인 항목**. FINAL heading 방식은 (a) 두 태그 깊이차/B(PnP z 또는 depth z; 태그 크기·인쇄 오차 공통모드 상쇄) (b) 1장 10~25° 기울임(비평면 — 문헌상 yaw 이득 조건) (c) B 단일 + 자이로 짧은 다리(MegaTag2 형) 중 **정적 실험으로 확정**; 2장이어도 해선택·정면 heading 불신 규칙 유지(코플래너 번들은 2중해 잔존). **C depth = 정면 평면 RANSAC 독립 안전 캡**(fail-safe: 평면 없음 = 캡 없음 → 97 강제 + 경고; emergency 0x80 은 plan 2-3 실측 후) + 종방향 D 정지 후보(2 m σ≈1.5 cm·bias 2~4 cm 를 PnP z 공관측 구간 1점 보정; depth FOV 87° 가 RGB 컷 뒤에도 탑재부 면을 봄) — run 경로 depth 미개방이라 depth on 구성으로 4-1 타이밍 게이트 재실행. A 카메라 피치 = 보조(잔여 블라인드 ≈ 탑재부 깊이라 단독 불가). 대안 D(T 4.0 m + 직진) 폐기. **정밀도는 인지가 아닌 실행 정밀도 x_ref·σ_ψ̂ ⊕ (d_T′ − x_ref)·σ_r**(레버 2.45 → 1.0~1.5 m, 양자화 σ 28 → 11~17 mm, 1σ 예측 15~20 mm; "lateral ≈1 cm" 철회) 로 plan 3-0 재판정 — **c = 30 mm 가 확정된 실물값이라 완화 불가 → 근거리 태그만으로 87~95%/회 → 저속 회전 단(plan 2-4 조건부)이 사실상 필수**(9/21 정정: 예전엔 "c/D 완화" 가 퇴로였으나 c 는 탑재부 실물, L_dock 도 우리가 못 정한다).
- 검증: 근거리 태그 정적 실험 1개 — 2장 B ∈ {0.6, 0.8, 1.0} m × d ∈ {1.5, 2.0, 2.5} m × 진값 yaw {0, ±2, ±5}° × 100 프레임(+1장 10~25° 기울임 팔) → (a) 단일 PnP yaw σ·|median−진값|·flip 율 (b) atan(Δz_PnP/B) (c) atan(Δz_depth/B) 의 σ·bias·프레임 상관; 제어 통과 = heading std ≤ 0.5° ∧ |median − 진값| ≤ 0.5° ∧ lateral 줄자 |차| ≤ 10 mm → 통과 방식 채택, 전부 미달이면 (c) 로 격하하고 T_h 를 자이로 앵커 나이로 재산. 부수: 공시야 3.3~4 m ×30 태그 1 횡·높이 vs 줄자 ≤ 10 mm(깊이는 수 cm 차 확인), 태그 2 조준 T′ 2.5 m 최종 정지 ×10 줄자 σ vs 예측 15~20 mm, depth 평면거리 0.5~2.5 m bias 곡선 + depth on 타이밍 게이트.
- 상수: **새 config 3~4 `NEAR_TAG_IDS`·`NEAR_TAG_SIZE_M`(인쇄 실측)·`NEAR_TAG_X_M`·`NEAR_TAG_BASELINE_M`((b) 채택 시 `NEAR_TAG_TILT_DEG`; 전부 물리 배치)** · 실시간 태그 1 횡·높이 + depth bias(공관측) · **삭제 `DEPTH_TOL_COEF`·`DEPTH_TOL_FLOOR_M`·`DEPTH_CHECK_MAX_Z`(→ 실측 depth bias 표); `DOCK_EXTRA_M` → `CAM_TO_REF_M`(=1.52, 확정) + `STANDOFF_M`(운영; 내일 0.50)**. `INSERT_DEPTH_M` 은 폐기(삽입 없음).

### 4-6 시뮬레이터 — G1 계약·회귀(지금) / G2 순위 도구(perception_calib 후)
- 결론: **G1(G_code-A)**: 진짜 `TagPipeline.process`·`Result` 경로·상태기계 전이표·field 동일 jsonl 스키마(같은 analyze_run 으로 KPI)·세 캘리브 json 을 같은 로더로·고장주입(호스트 정지 = 전 스트림 동시 stale, 차량 워치독 T_wd/마지막 명령 유지 옵션, 이중 송신원, 검출 300 ms, AMBIGUOUS 10 s). **G2(perception_calib.static 후)**: 플랜트 = 이산 명령 × {67, 97} → 순수지연(0.5/0.85 s) + 램프 + 정속(0.28/≈0.12) + 코스팅 τ_d·σ_τ + run 간 게인 ±10~30% + L/R 비대칭 + 97 stiction(베르누이) + A ∈ {−0.5, 0, +0.5} + 자이로 바이어스 랜덤워크·스케일 s·gaps; 센서 = σ_c → PnP 야코비안 + **결정론적 매끈 b(d, tilt, 정면) 에 초 단위 시간상관**(프레임별 확률 추출은 30프레임 중앙값에서 사라져 재앵커 반전을 재현 못 함) + flip p(d, tilt) + 가시성/상단 컷(h·pitch·태그 2; v1 `sim_engine.visibility`) + 지연·드롭·stale; 시계 모델(캡처/도착 큐 16 옵션/검출/CAN 10 ms/TXACK), 추정기는 스탬프만 본다. **타당성 게이트 = 성분별 서명(정적 lateral 잡음 vs 거리, 정면쌍 Δψ_cam/Δψ_IMU 1.14~1.25, 죽은시간+선형, 회전 오버슈트 1.3°; n≥10) 이 실측 CI 안 ∧ 동결 v2 컨트롤러(import, 수정 금지) 성공률 ∈ [20, 80%]·스텝 분포가 21 포함**(합리성 검사; 9/7 n=2 재현은 단일 사례 점검). **채택 = 1차 KPI 개선 ∧ wrong-DONE 0 ∧ abort 비악화 ∧ 반전 비악화**, 페어드 시드 N=200 × 포락선 안 격자 × 잡음 × flip × 지연(해석형 전용); 렌더형(v1 render, σ_c 과소)은 계약·flip 존재 확인 1세트. 순서: 골격 → Day 0 격자 → b 주입 후 A/B. **시뮬은 순위 도구, 99% 인증 도구 아님.**
- 검증: Day 0 b(d, tilt) 주입 후 181437 상태열 초기조건으로 동결 v2 로직 폐루프 → 8→9→10 형 반전(21±5 스텝·반전 ≥1) 재현 1건 + 서명 4종 CI 안 → 통과 시에만 G2 A/B 채택. G1 = 계약 위반 0·고장주입 8종 각 캡 안 Tier 도달.
- 상수: 새 config 0(시뮬 값은 실사용 미참조 파일, v1 관례).

### 4-7 현장 실험 — Day 0 / 방문 A(데이터) / 방문 B(v3 폐루프)
- 결론: **Day 0**(사무실, 차량 불필요; **장착 기하 복제** — 삼각대 h 와 실차 h 차 ≤ 5 cm·같은 노출 프리셋·현장에 부착할 그 인쇄물을 강성 판에) = 정적 격자 3/5/7/10 m × heading 11값 × 노출 2단 × 100 프레임 + 왜곡 전/후 → `perception_calib.static`; 근거리 태그 정적 실험(4-5); 시뮬 G1 통과; dry-run. **방문 A**(데이터 수집; **G_code-A 산출물 = v3 tools 의 frame.jsonl·GLOBAL 대기·write 시각/TXACK·L 주입 옵션** — v1·v2 수정 금지; 호스트 Jetson/Windows): **(0) 타이밍 게이트 30 min** — bag on/off 각 60 s(arrival p99 차 < 30 ms 면 bag 구성으로 전 블록, 넘으면 bag 은 지정 셀만; bag 은 별도 USB 컨트롤러 SSD 직결 또는 720p 5 GB/분), GLOBAL 전환 시각·hardware_reset·컬러 SENSOR_TIMESTAMP 유무·loop tick·TXACK 첫 10건 max·gaps → timing_calib → **(1) 안전 45 min** — listen-only 60 s → 차단 4종 ×3 → 복구 순서 → emergency 0x80 정지 상태 ×1 → 송신 데드맨 → kill -STOP 2 s → 조이스틱 개입 ×3 → 이중 송신원 → heartbeat 0x05/0x00 → T_wd·래치·FM11 거동 → **(2) 장착 종속 perception 30 min** — h·R_mount 피치/롤·x_off 줄자·3.5 m tilt 20°·정면 두 셀 재검(사무실 b 와 ±0.3° 초과면 격자 현장 재실시)·태그 1 횡·높이 자기보정 → **(3a) 회전** — 개루프 L/R × {1.5, 3 s} × 4 + 5 s 자이로 ×2 + 폐루프 15° × 6 + tilt ≥ 20° 3.5/6 m × 5(A·s·ε) → **(4a) 67 정속정지 n=20 정면+사각 + 97 stiction 4 s × 10 [여기까지 필수 = dynamics_calib v1 보장]** → 선택(편향 스윕 {10..60} × 3 → 펄스 {1, 1.5, 2, 2.5 s} × 5 → 67→97 전환 ×5 → 정속 8→4 m ×3×{67, 97} → S(T) → depth bias 곡선 → L 주입 {150, 300} → 최종 정지 ×10 줄자 → 세션 말 67 n=10). **방문 B**(v3 폐루프, G_code-B 후): 포락선 안 시작 ×5·dog-leg 1·고장주입·300 ms 부하·명목 도킹 n=20(KPI Stage A). 게이트 실패 시 이후 블록은 원시 기록만, 캘리브 파일은 만들지 않음.
- 검증: 방문 A 종료 시 "원시 로그만으로 timing_calib / perception_calib(static+dynamic) / dynamics_calib 세 파일이 오프라인 생산되는가" — 하나라도 안 되면 plan 2-7 "기록 항목 부족"; Day 0 이전 h 차 ≤ 5 cm; (0) bag on/off arrival p99 차 < 30 ms.
- 상수: 새 config 0 · 산출 캘리브 파일 3. (plan 2-7 의 1일차/2일차 목록은 이 절이 대체; 정적 격자는 Day 0 로 앞당김 — 정오표.)

### 4-8 문서·plan 구조·코드 진입 게이트
- 결론: **단일 진실원 = plan.md**; methodology 4문서·PLAN_legacy 완전 동결(머리에 "status: rationale only — 결정은 plan.md"), 정정은 아래 **정오표 절**에만. plan.md 공통 절: 설계 불변식 10줄(단일 시계·캘리브 사슬·곡선 금지·태그 중앙 접근·카메라 절대/자이로 짧은 다리·상수 최소화·wrong-DONE 0·DONE-unverified 규칙) · **상수 대장**(config 이름 → 유지/삭제/캘리브/코드내부, 소유 대문제, 근거, wrong-DONE 하드 규칙 열; 목표는 숫자 아닌 규칙 "모든 config 항목은 {탑재부 사양/운영/물리 규격} 중 하나에 귀속, 나머지는 캘리브 파일 또는 코드 내부" + 자동검사 `snapshot_config` 키 ⊆ 대장·미귀속 0; 기준 개수 74 = control 39·detection 28·imu 7) · **캘리브 파일 등록부**(timing / perception static+dynamic / dynamics: 필드·생산 스크립트·소비자·정책 해시·허용창·거부 규칙) · 실험 마스터(4-7) · KPI(4-3) · **사용자 결정 목록**(아래). **코드 진입 2단 — G_code-A(인프라, 사용자 결정 0)**: 인터페이스 계약 3종(추정기 출력 튜플 plan 1-7·컨트롤러 enum/전이표 plan 3-1·캘리브 json 스키마) + 시뮬 G1 스펙 → 타이밍/로깅층·캘리브 로더·precheck·시뮬 플랜트·v3 tools 패치 코딩 허용; **G_code-B(제어)**: 사용자 결정 6개 확정 후 추정기 게이트·상태기계·구역·FINAL·근거리 태그. 코딩 순서 = 시뮬 골격(플랜트 + 계약) → 타이밍/로깅층 → 추정기 → 캘리브 로더 → (B) 상태기계 → analyze_run KPI 재작성(현행 ROT_LEAD_DEG/FWD_SCALE 권장 출력 삭제). 2→3 문서 병합(critic §22 구조)·archive 이동은 G_code-A 통과 후 coder 첫 작업.
- 검증: 상수 대장 초안에서 config 74개 전부 배정·미결 0 ∧ G_code-A 항목이 사용자 결정 6개를 참조하지 않음 ∧ 대장 검사 스크립트가 `--record-events` config.json 스냅샷의 미귀속 키 0; 실증 = G_code-A 통과 후 방문 A 로 세 캘리브 파일 생산.

### 대문제 4 상수 회계
- **신설 config**(9/21 정정: 팔레트 3종 폐기): 물리 `DOCK_DEPTH_M`(탑재부 진입 깊이, 잠정 2.0 = 미측정 가정)·`CAM_TO_REF_M`(← DOCK_EXTRA_M, =1.52 확정)·`NEAR_TAG_IDS`·`NEAR_TAG_SIZE_M`·`NEAR_TAG_X_M`·`NEAR_TAG_BASELINE_M`(+`NEAR_TAG_TILT_DEG`) / 운영 `FWD_TOL_M`·`STANDOFF_M`(plan 3)·`DOCK_TIME_BUDGET_S`(plan 3).
- **유지(정정)**: `LAT_TOL_M` 0.030(실물 여유 c) · `HEAD_TOL_DEG` **0.5**(c/DOCK_DEPTH_M 유도).
- **삭제**: `STABLE_LATERAL_M`·`STABLE_HEADING_DEG`·`MAX_HEADING_SIGMA_DEG`·`STABLE_SPREAD_K`·`MEASURE_FRAMES`·`MEASURE_MAX_FRAMES`·`DEPTH_TOL_COEF`·`DEPTH_TOL_FLOOR_M`·`DEPTH_CHECK_MAX_Z`·`DOCK_EXTRA_M`(분리) + plan 1-6 의 `TAU_CAM_FALLBACK_MS`·`T_READOUT_MS`·`STALE_MS`.
- **캘리브 파일 3**: `timing_calib.json`(신설), `perception_calib.json`(static/dynamic 절), `dynamics_calib.json`. 부재/정책 해시 불일치 → 출발 거부.
- 전체 대장은 G_code-A 첫 작업(74개 전부 귀속). 목표는 규칙·자동검사이지 개수가 아님.

### 사용자 결정 목록 (G_code-B 선행조건; G_code-A 는 이 목록과 무관)
1. ~~**x_ref**~~ — **확정: 포크 끝(접힘), `CAM_TO_REF_M` = 1.52 m.**
2. ~~**D**(포크 삽입 깊이)~~ — **폐기: 팔레트 삽입이 아님.** 대신 **탑재부 진입 깊이 `DOCK_DEPTH_M`** = 미측정 → **2.0 m 가정**(실측되면 갱신; 그때까지 코드는 잠정값 + 로그 경고).
3. ~~**tolerance 출처**~~ — **확정: c = `LAT_TOL_M` 0.030(탑재부 폭 ≈ 지게차 폭의 실물 여유), `HEAD_TOL_DEG` = 0.5**(c/DOCK_DEPTH_M = 0.86° 에 안전 여유). **남은 것은 `FWD_TOL_M`(종방향 허용오차) 하나** — 내일 태그면 앞 정지라 실험엔 불필요, 실전 전 필요.
4. **근거리 태그 2 장 허가·배치** — 정면 자리(화물·개방 프레임 간섭), x·B(FOV↔D 제약).
5. **카메라 피치 허가**(보조안).
6. **호스트** — Jetson / Windows(그램) (VM 실주행은 CLAUDE.md 조건부 허용이나 방문 A 권장 호스트 아님).
- 부수: 트랙션 컨트롤러 기종(Curtis/Sevcon/Zapi, Dropbox 코드) · 9/14 2차 실험 실시 여부(work_dirs 에 로그 없음) · Kalaitzakis 2021 원문(기관 접근).

### 정오표 (plan 1~3 절에 대한 정정 — 검토 후 본문 반영)
- **plan 1-6 47행·1 실험 4**: 상호상관 판정 반전 — 옳은 스탬프면 ε ≈ 0 이 정상, 80~110 ms 가 나오면 도착시각 스탬프. 통과 기준은 4-1 의 L·ε·2-지연 검정으로 대체. t_capture = get_timestamp() 그대로가 아니라 노출중심(SENSOR_TS/Δ_FS + 행 보정); "시작 시 timestamp_domain assert" → GLOBAL 전환 대기.
- **plan 1-6 파라미터**: `STALE_MS`·`T_READOUT_MS`·`TAU_CAM_FALLBACK_MS` 는 config 후보가 아님 — STALE 코드 내부, 나머지 삭제(실측 Δ_FS·T_line·ε).
- **plan 1 (1-4·1-5)**: `B_TABLE(d,tilt)`·`C_TABLE`·`SIGMA_FLOOR(d,tilt)`·`SIGMA_C0_*`·`SIGMA_A/P/G_TILT` 는 config 가 아니라 `perception_calib.static` 필드(상수 최소화 원칙); 1-1 의 A "사전실험 확정" → plan 3-2 실시간 도출.
- **plan 2-7**: 차량 워치독 기호 'T_f' → **T_wd**(T_f 는 전후 허용치와 충돌); 정적 격자 "2일차" → **Day 0**(4-7); 1일차/2일차 목록은 4-7 이 대체.
- **plan 3-5 ⓪**: 회전 후 |Δψ_cam − Δψ_IMU| 초과는 FM2(재앵커 바이어스) 신호이지 FM1(wrong-branch) 신호가 아님 — FM1 은 pitch-cue ∧ 직진 다리 일관성(4-4).
- **plan 3-0·3-3**: `LAT_TOL_M`·`HEAD_TOL_DEG` 기반 수용식 → 4-3 평행사변형으로. 단 **c = LAT_TOL_M 자체**이므로 (9/21 정정) 상수는 없어지지 않고 그대로 쓰인다; 99% 가능성은 단축 레버(1.0~1.5 m)·실행 정밀도 1σ 15~20 mm 로 재계산(4-5).
- **9/21 전면 정정 — 팔레트 삽입 전제 폐기**: plan 전체에서 `POCKET_W_M`·`FORK_W_M`·`INSERT_DEPTH_M`·"포크 삽입 깊이 D"·"포켓" 은 **해당 없음**. 도킹은 **접힌 포크로 탑재부에 진입·주차**. D 가 나오는 식은 전부 **L_dock(`DOCK_DEPTH_M`, 탑재부 진입 깊이, 잠정 2.0 m)** 으로 읽는다. c 는 도면 유도가 아니라 **실물 3 cm**. x_ref = 포크 끝 확정.
- **plan 3-7**: FINAL DONE-unverified 는 근거리 태그 통과 시 관측된 VERIFY 로 전환.
- **plan 3-6 vs 2-7**: perception_calib 부재 → 출발 거부 ↔ 정적 격자 2일차 모순 → static(Day 0)/dynamic(방문 A) 분리로 해소(4-2).
- **methodology v2 424행** lateral 부호(plan 1-1) — 문서 동결이므로 여기 기록만.

### 다른 대문제로 이관
없음(마지막 대문제). 남는 것은 사용자 결정 6개와 방문 A 실험.
