# 대문제 1 — Perception 토론 기록 (2026-09-20)

- 실행: `debate_workflow.js` n=1, run `wf_db146ef1-262`, Fable 5.1 effort max, 에이전트 5개(proposer / critic 안전·정밀 / proposer 재반박 / judge), 폴백·오류 0, 70분, 1.02M 토큰.
- judge 1차에서 hold 0 → researcher 미가동, 재판정 없음. verdict: critic 3 (쟁점 1·3·4) / compromise 4 (2·5·6·7) / proposer 0.
- 핵심 데이터: judge 가 `apriltag_v2/work_dirs/docking_log/20260907_181437/{measure,rotation,decision}.jsonl` 을 직접 재계산.
  **오케스트레이터 검산**(원본 jsonl 재처리): 13회 회전의 Δβ/Δψ_IMU 0.77~0.95, 정면쌍 Δψ_cam/Δψ_IMU 1.14~1.25, M5 distance 3.99 vs forward 3.906 — 전부 재현됨.
  코드 사실(set_global_time 호출 0 / run.py:181 tune 없음 / detect 왜곡계수 미전달 / rate_dps 마지막 샘플 고정 / 방법론 424행 부호) grep 으로 확인.

## 쟁점 1: lateral 재구성 — scalar 식 vs 명시적 좌표변환
- proposer: 명시적 변환 `T_tag←pivot`, 위치는 R̂=R_mount·R_yaw(ψ̂) 로 재구성, `d·sin(ψ̂−β)` 는 단위테스트 오라클로만. 코드 결합항 `lat_p = lateral − A·sin h` 가 18:14 로그(−4.5°→+112mm)와 "부호 충돌" 이라고 진단 — 근거: critic §2, Adámek 2023, control_from_pose.py:93, config/control.py:64 주석.
- critic(안전): 명시적 T·오라클 동의. "부호 충돌" 은 두 시나리오(실제 CW+피벗 앞 / 로그 heading 이 flip) 중 미분리이고, 오라클 식은 코드 규약에서 부호 반대(`detection_pose.py:51-53` 예제로 lateral = d_h·sin(β−ψ)). A 는 부호 있는 상수, 검증은 카메라 heading 아닌 자이로 Δψ 기준으로.
- critic(정밀): 코드 부호는 기하적으로 정확. 정면 두 점으로 부호를 뒤집는 건 증거 위계 오류(경사 tilt 23~28° 구간은 코드 부호로 lat_p ±12mm 불변). **더 근본: A=1.46 자체가 미확정** — R 무관 방위각 β=atan2(t_x,t_z) 의 회전 전후 변화 Δβ/Δψ_IMU 가 대회전 4건 0.88~0.99 인데 A=+1.46 이면 1.16~1.43 이어야 함. 1.46 출처(`오차보정.txt` can_pulse 18:43)는 8m·자이로 사망·1회.
- proposer 재반박: **수정** — 부호 충돌 진단·A=1.46 철회. 13회 전체 재계산 표 제시(Δβ/Δψ_IMU 평균 0.88, 13/13 <1). 18:14 의 +112/+226mm 는 정면 heading 과대변화(−1.13°/−1.76°×3.9m ≈ 77/118mm) 로 대부분 설명. A·x_off 를 추정 파라미터로, 확정 전 컨트롤러는 β 로 조준.
- judge: **critic** — 결정적 근거: r10(3.3m, IMU +24.55°) 에서 A=+1.46 은 Δβ 35° 예측, 관측 21.6° — heading 바이어스(≤2°)·IMU 스케일(≤10%) 어느 것으로도 설명 불가. 물리적으로도 |A|≲0.4 는 카메라가 구동축 근처라는 뜻이고 config 의 "뒷바퀴" 가정은 미검증.
- 결론: T_tag←pivot 명시적 변환, 병진 (x_off, 0, −A) 3-벡터, **A·x_off 는 추정 파라미터(사전 A∈[−0.5,+0.5], 초기 0, σ_A→P), CAM_TO_PIVOT_M=1.46 철회**. 코드 결합항 부호 유지, 오라클 `|lateral − d_h·sin(β−ψ)| ≤ 5mm`, 방법론 v2 424행 부호 수정. A 확정 전 컨트롤러는 β_px 조준.
  | 검증: tilt≥20°, 3.5m·6m 두 거리, 정지→IMU 회전 15~25°→정지 ×5(자이로 +3s 적분), Δβ = s_gyro·(1+A·cosβ/d)·Δψ 회귀로 s·A·부호 분리. 정면 원뿔에서는 금지(바이어스가 A 로 둔갑).
- 오케스트레이터 메모: 비율의 거리 의존(9.3m 0.95 vs 3.3~3.9m 0.87)은 두 거리에서 모두 |A|≈0.45m 를 주어 "A≠0" 쪽에 조금 유리하나 9m 표본은 1개 — 실험 설계(두 거리)가 이걸 분리한다.

## 쟁점 2: bearing β — PnP atan2(t_x,t_z) vs 태그중심 픽셀
- proposer: 픽셀 β(center→undistort→R_mount→atan2) 를 주로, PnP β 는 대조용. β_tag(화면중앙 유지 변수)·β_T(T 방위각) 분리 출력 — 근거: critic §3, Abbas 2019, `plan_aim` 의 tag_dir 이 R 경유라 flip 오염.
- critic(안전): B 채택 동의. 두 IPPE 해는 t 방향을 공유해 "β_A 양봉" 은 관측 안 됨 → 채택 근거 수정. 진짜 적은 상수 바이어스: 롤 δφ → lateral 1.1m×δφ = **19mm/°, 거리 무관**; 가속도계 영점(수 °급)보다 태그 상하변 방향이 롤 출처. β 는 quality 실패 프레임에도 무조건 산출.
- critic(정밀): A·B 정보량은 거의 같고 실차이는 **왜곡 보정**: `detect()` 는 (fx,fy,cx,cy) 만 넘겨 라이브러리 자세가 왜곡 미보정, `pose_by_pnp`·`heading_sigma_deg` 만 distortion 사용 — 두 모델 혼용. `i.model` 은 버려지고 `undistort` 호출자 0. T 에서 태그가 화면 상단(margin 31~97px) 이라 방사왜곡 최대 영역 → 3.4m 1px = heading 7.6° 라 정면 바이어스 후보.
- proposer 재반박: **수정** — 양봉 실험 철회, 왜곡 모델 단일화·i.model 기록 수용, 롤 출처를 태그 roll 30프레임 평균으로. 롤링셔터는 태그 내부 차등(0.02°) 무시, 행 시각 오프셋(≤30ms) 은 쟁점 6 으로.
- judge: **compromise** — 코드 grep 으로 critic 사실 확인. Rijlaarsdam/Liu 2중해 모델(중심 공유) 로 안전 렌즈 예측 타당. 왜곡→정면 바이어스는 가설(D435i 컬러 coeffs 가 0 일 수 있음, 로그로 확정).
- 결론: β 는 픽셀 중심→왜곡보정→정규화 광선→R_mount→atan2, **PnP 유효성과 무관하게 매 프레임 산출**. 왜곡 모델 단일화(i.model·coeffs 를 config.json 에 기록, ≠0 이면 코너 보정 후 β·PnP 동일 코너). 롤은 태그 roll 30프레임 평균, 가속도계는 pitch 검증용. β_tag/β_T 두 출력.
  | 검증: 정적 3.5m·태그 상단 배치 100프레임 — 왜곡 보정 전/후 heading·pitch 차(>1° 면 왜곡이 원인) + 카메라 롤 2° 시 lateral ≈38mm 이동 확인.

## 쟁점 3: heading PnP 2중해 처리 — 해 선택·AMBIGUOUS·lock-in 탈출
- proposer: `solvePnPGeneric(IPPE_SQUARE)` 로 두 해 명시 산출. **pitch-cue**: 태그가 카메라보다 h≈1.1m 위라 틀린 해의 pitch≈2·atan(h/d)(5m 25°/8m 16°), 옳은 해≈pitch_mount → 3상태(CONFIDENT_A/B/AMBIGUOUS), 15프레임 연속 반대면 재초기화, 태그 15° 아래 기울임 — 근거: Liu 2021, PhotonVision err비 0.2, Rijlaarsdam 2022, `pose_to_forklift` 가 이미 pitch 계산.
- critic(안전): pitch-cue 는 최고의 통찰(정면 자세를 살리는 유일한 단서). 단 "15연속" 은 8m σ_pitch 6.3° 에서 0.66¹⁵≈0.2% 로 영영 탈출 불능 → Wald SPRT 로. "15° 아래" 는 법선–시선각이 |ε−15°| 로 줄어 d≈4.1m(T 구간) 에서 정면 → 위쪽 15° 또는 좌우 yaw 15~20°. 초기화는 30프레임 다수결. h≥0.6m 설계 상수.
- critic(정밀): 18:14 정면원뿔(tilt 3~4°) 은 flip 이 아니다(틀린 해면 tilt≥36°). 그런데 회전 전후 Δψ_cam/Δψ_IMU 가 정면 4쌍 1.14~1.25(+0.7~1.8° 과대), spread 는 0.02~0.09° → **옳은 branch 의 매끈한 yaw 바이어스**가 지배하고 pitch-cue·err비·마할라노비스 모두 통과. `pose_quality` 의 2px 게이트는 pose_err 가 객체공간 m² 라 AT3 경로에서 무효. → tilt 원뿔 하드 게이트 복원(Richter ±25° 회피).
- proposer 재반박: **수정** — SPRT·다수결·기울임 방향 정정 수용, tilt 원뿔 하드 게이트 복원. 추가 증거: `vertical` 열이 정면 −0.82~−0.88m vs 경사 −0.98~−1.19m(같은 거리) → 옳은 해 pitch 도 3~4° 바이어스라 6° 게이트 통과.
- judge: **critic** — 정면쌍 재계산(1.25/1.22/1.25/1.14, spread 0.016~0.064°) 으로 "분산 아닌 바이어스" 확정. 게이트 가능한 건 기하뿐. 대각 접근 전제에서 heading 이 실제 쓰이는 구간은 ~5m→T 의 tilt≥15° 라 flip 기구는 단계적 최소형(CLAUDE.md "4중 yaw 게이트 보류" 준수).
- 결론: heading_valid = **기하 원뿔 tilt≥θ_min(d) [하드, 초기 15°]** ∧ 두 해 명시 산출 후 err비≤0.2 [소프트] ∧ branch CONFIDENT; 아니면 AMBIGUOUS(β·translation 은 계속 사용). pitch-cue 는 1단계에선 두 해의 pitch·err 를 로그만, 정적 격자로 양봉·σ 확인 후 SPRT 승격. 초기화·재초기화 30프레임 다수결, 단일 프레임 CONFIDENT 금지. AMBIGUOUS 타임아웃→정지·oblique 재획득 fallback. 정면 구간 heading = 마지막 oblique 앵커 + 자이로. `pose_quality` 재투영 게이트를 픽셀로 재계산. 하드웨어 기울임은 실측 전 손대지 않음.
  | 검증: 정적 3.5/5/8m × 진값 heading 0/±3/±6/±10/±15/±25° × 100프레임 — 두 해 pitch 히스토그램(봉우리·σ→SPRT 임계), 옳은 해 yaw 바이어스 곡선 → |바이어스|>max(1°, LAT_TOL/d) 인 범위가 θ_min(d).

## 쟁점 4: 기하 관측성/신뢰도 — σ_ψ 모델과 시작 단순도
- proposer: 해석식 σ_ψ ≈ σ_c·√2·fx/tag_px² 하나 + AMBIGUOUS 플래그로 시작, 지수·계수는 `a·tag_px^−p` 피팅. `heading_sigma_deg` MC(프레임당 solvePnP 60회) 는 오프라인으로. boolean `reliable_angle` → 연속값 σ_ψ + heading_valid. "1.4m 0.35px→0.45°" 실측을 식이 정확히 재현 — 근거: Adámek 2023, critic §5, detection_pose.py:205-252.
- critic(안전): 동의. "정확히 재현" 은 태그 0.30m·1080p 가정에서만이고 √2 는 사후 맞춤 — 피팅 대상이라는 결론만 취할 것. σ_c 3배 변동을 NIS 만으로 잡으면 1~2s 늦음 → σ_c 를 재투영 잔차로 **직접 추정**해 NIS 앞단에.
- critic(정밀): 식의 3.4m 30프레임 σ 0.2°(lateral 12mm) vs 실오차 0.7~2.3° 바이어스 → 식의 3~10배, 정적으로 일관되어 스케일링으론 안 잡힘. **바이어스를 σ 에 넣으면 √N 으로 잘못 줄어든다** → σ_eff² = σ_random² + b²(d,tilt) floor. σ_c 는 정지 0.07·주행 ~0.3px 로 4배 차.
- proposer 재반박: **수정** — 재현 주장 철회, σ_c 직접 추정, b floor 추가. 재검산: 정면 3.3~3.9m 프레임 σ 0.09~0.35° 인데 해석식(σ_c 0.15) 은 1.2° 로 3~13배 과대, 실오차는 spread 의 10~50배 → spread 도 식도 오차 대리가 못 됨. g(tilt) 1~2 파라미터로 Adámek 5+5 대체.
- judge: **critic** — m5/m6 에서 MC 0.25° vs 관측 프레임 σ 0.25~0.35° 로 분산 모델은 맞았으나 바이어스 1.1~1.8°(σ 의 5~7배) 는 못 봄 = false confidence 의 정체.
- 결론: 출력 (σ_ψ,eff, heading_valid). **σ_ψ,eff² = [a·tag_px^−p·g(tilt)]²·(σ̂_c/σ_c0)² + b²(d,tilt)** — a·p·g 정적 격자 피팅(p 초기 2), σ̂_c 는 옳은 해 픽셀 재투영 잔차 30프레임 평균(정지 ≈0.07~0.13·주행 ≈0.3px, CORNER_NOISE_PX=0.07 은 낙관), b 는 |median−truth| 표(√N 축소 금지). MC 는 오프라인 기준.
  | 검증: 정적 3/5/7/10m × tilt 0/10/20/40° × 100프레임 × 노출 2단 — per-frame std 가 a·tag_px^−p·g 에 맞는지, 재투영 잔차가 std 변화를 따라가는지, |median−truth|/spread ≫1 셀은 b floor.

## 쟁점 5: uncertainty ≠ 실제 오차 — covariance calibration 절차
- proposer: P 는 estimated_covariance 로만. (1) 줄자 진값 커버리지→팽창계수 c·σ_floor(d), (2) 온라인 NIS≈1 유지, (3) 자이로 고정 직진 lateral 기울기→CAM_YAW_OFFSET. 컨트롤러 계약 |e|>T+k·σ_eff 가 N프레임 지속 — 근거: critic §6·Failure 2, Bar-Shalom, Chang 2014, `measure()` 의 spread 대조.
- critic(안전): 동의 + 빠진 검사: 프레임 NIS≈1 은 **독립성** 보장 안 함(flip 20프레임·조명 기울기 → N프레임 평균 오차 ≫ σ/√N). 줄자 ±1cm 는 3m σ_lat 10mm 와 같아 c 못 잼 → 격자+레이저(±3mm), 기준 **포크 중심**, forward 진값으로 TAG_SIZE(1%=3.5m 35mm, DEPTH_CHECK 1.5m 라 미검출). 커버리지는 KF 출력 1/3/10s 창에서.
- critic(정밀): NIS 는 매끈한 바이어스를 상태가 흡수해 ≈1 — 방어선 불가. (3) 을 정면 직진으로 재면 정면 yaw 바이어스와 오프셋이 섞임 → oblique(ψ≈25°) 에서. `config/control.py:70,75` "회전 잔차 ±1.5°" 는 카메라–IMU 차(13/13, 0.2~2.3°) 의 오귀속 = 같은 오차 이중 해석. 1차는 **교차센서 일관성**(Δβ vs Δψ_IMU, Δψ_cam vs Δψ_IMU) — 18:14 로그만으로 이미 A·바이어스를 잡았다.
- proposer 재반박: **수정** — NIS 경보로 격하, 교차센서 1차, 포크중심+레이저, (d,tilt) 셀별 c. (c) 1표본: can_pulse 18:46 forward 5s 이동방향 31.6° vs heading 33.1° → δ≈1.5°(8m 잡음 수준, 참고만).
- judge: **compromise** — 18:14 로그 자체가 증명(새 장비 없이 교차센서만으로 두 문제 검출). 이동방향은 차체축·heading 은 카메라축이라 (c) 는 R 공통모드에 강건.
- 결론: 게이트는 σ_eff = max(c·σ̂, σ_floor(d,tilt)) 만. 순서 **(1) 교차센서 일관성**(회전×5: A·자이로 스케일·바이어스/스프링백, oblique 직진×2 5→3.5m: δ=CAM_YAW_OFFSET; 통과 = 5회 중 4회 같은 부호·±0.5°) → **(2) 정적 진값**(격자+레이저, 포크 중심, 셀별 c·b, TAG_SIZE) → **(3) KF 창별 커버리지**(1/3/10s, 창 길이 무관해야 통과). NIS 는 경보. "회전 잔차 ±1.5°" 이중해석 금지 → dynamics 토론 이관.
  | 검증: 회전×5 + oblique 직진×2 한 세트에서 A·δ·카메라–IMU 차 재현; 3.5m 정적 300프레임에서 1/3/10s 창 68/95% 커버리지.

## 쟁점 6: 지연 보정 — timestamp·전파 방식·큐 정책
- proposer: 단일 시계 = 호스트 epoch, RealSense global_time 의 `get_timestamp()` 를 t_capture(assert, 폴백 66ms+검출시간), 5개 시각 기록. SE(2) 전파가 직진/제자리회전으로 두 경우 퇴화(pivot 상태, 카메라점 = pivot + A·u). 카메라 측정은 자이로 링버퍼로 t_capture 시점에 되감아 융합. 명령 지연 예측은 dynamics 층 — 근거: critic §7·§21, Larsen, PX4 EKF2, 시계 3종 혼용(image.py:449 상대시각 / now() monotonic / event_log epoch).
- critic(안전): 동의 + 사실 정정: "camera.py 가 global_time 을 켬" 은 거짓 — `set_global_time` 호출 0, `docking()` 의 queue=1·global_time 은 tune= 경로만, run.py:181 은 tune 없이 → **SDK 기본 큐 16**. 검출이 33ms 넘으면 최대 ~0.5s 묵은 프레임을 "현재" 로 소비(Failure 5 의 실제 경로). 제안은 t_arrival 기록만 하고 정책이 없다 → stale>150ms predict-only, 태그 행 시각 보정, NTP 점프 감시.
- critic(정밀): 같은 사실 정정 + 18:14 measure 30프레임에 1.30s(≈23fps) 로 소비가 30fps 미달, `image.py:415` "밀리지 않음" 주석은 검증된 적 없음. SE(2) 의 카메라점 = pivot + A·u 는 A 가 틀리면 회전 중 예측이 A_err·Δψ 틀려 재포착 innovation 튐 → 쟁점 1 선행조건.
- proposer 재반박: **수정** — grep 으로 정정 확인·수용, frames_queue_size=1 명시, (t_capture, t_row, t_arrival, t_detect_done, t_decide, t_cmd_tx) 기록, stale 정책, dt 33±10ms 밖 플래그, A 는 파라미터·σ_A 예측 P 에.
- judge: **compromise** — rotation→measure 간격 5건 1.22~1.33s 로 22~25fps 확인; 자이로 motion frame 도 global_time 이면 카메라와 같은 시계일 가능성(assert 로 확인).
- 결론: 단일 시계 host epoch; 컬러·자이로 `timestamp_domain` assert·로그; run 경로 frames_queue_size=1(또는 드레인); 절대 t_capture 를 흘림; 프레임별 6개 시각 기록; **stale: t_arrival−t_capture>150ms → predict-only+경고**; dt 는 타임스탬프 차만. 전파 SE(2) 두 경우 퇴화(A 는 쟁점 1 파라미터), 카메라 측정은 t_capture 시점 ψ 로 되감아 융합, 축별 독립 외삽 불채택. 명령 지연 예측은 dynamics 층.
  | 검증: 8°/s 회전 중 ψ_cam(t_capture) vs ψ_gyro 상호상관 최대 지연 = τ_cam(예측 80~110ms, ≈0 이면 도착시각 증명) + 정지 60s t_arrival−t_capture 추세 + 인위 부하 40ms.

## 쟁점 7: 주행 중 연속 추정기 구조 — KF 구성·v̂ 잡음·재초기화
- proposer: KF 두 개(구동기 모델 없음): heading [ψ, b_g] 자이로 200Hz + CONFIDENT 시 카메라 갱신; 위치 [x_f, v]+[y_l], 측정 d=|t|·β·ψ̂, 회전 중 predict-only. **v̂ 재평가**: 접근속도는 거리에서 재야 하고 σ_d≈σ_c·d²/(fx·s) 3m 3~8mm, 30프레임 회귀 σ_v 2~5mm/s → τ·σ_v 3~7mm < σ_τ·v 7.5~15mm, v̂ 는 병목 아님. Q 는 명령을 분산 스케줄로만 — 근거: PLAN ①, Hinderer B4, Liu B8, Hurák C4.
- critic(안전): 구조·v̂ 재평가 동의(0.633 계수 재계산 일치). **d=|t| 는 3D 거리**라 1.1m 높이차로 3.5m +17cm/2m +28cm forward 과대 → 정지점이 탑재부 쪽으로 밀림(TOP1). heading 재초기화 시 위치 측정이 d·Δψ 점프 → 원자 이벤트 필요. b_g 는 카메라 없인 비관측 → 5m 30프레임 σ_ψ≈0.5° → σ_lat 40~50mm > 30mm → **5m lateral 은 30mm 판정에 못 씀**, `lateral_valid` 출력.
- critic(정밀): "pivot 불변" predict-only 가 A=1.46 에 묶임(24° 회전 뒤 0.6m 오차→재포착 기각→deadlock). 13/13 회전에서 카메라–IMU Δψ 차 0.3~2.3° → σ 0.05° 게이트가 재포착을 매번 기각 → 게이트 σ 에 b floor. [x_f,v]+[y_l] 분리는 대각 주행(ψ 25°) 에서 y 지연 → 유니사이클 결합. `rate_dps` 마지막 샘플 고정(can_pulse 18:43 자이로 사망 중 5° 회전을 "안 돌았다" 오판) → alive 게이트.
- proposer 재반박: **수정** — |t| 철회(M5: distance 3.99 vs forward 3.906), 유니사이클, 원자 이벤트, lateral_valid, alive 게이트, A 파라미터. 부분 반박: 대각 접근 전제에선 T 까지 tilt≈|ψ|≥15° 라 자이로 단독 구간은 T 이후 ≤3.5m(12~23s, 드리프트 0.2~0.4°) 뿐.
- judge: **compromise** — M5 로 |t| 과대 확인, imu_yaw.py:301-305 docstring 확인, proposer 의 부분 반박(정면 직진 접근에만 18s AMBIGUOUS 해당) 타당.
- 결론: **heading KF [ψ, b_g]**(자이로 200Hz 예측, heading_valid 시만 카메라 갱신, 게이트 σ=√(σ_random²+b²), 정지마다 b_g 재추정 |ω|<0.3°/s, gyro alive/gaps 게이트) + **위치 KF [x_p, y_p, v] 유니사이클**(ψ̂ 입력, 측정은 쟁점 1 의 p̂_pivot 만 — |t| 금지). 회전 중 predict-only 는 σ_A·Δψ 작을 때만, 종료 후 oblique N프레임 다수결로 ψ 재앵커. heading 재초기화 = 위치 상태 회전 + P 팽창 원자 이벤트. Q 는 분산 스케줄만. 출력 (x̂,v̂,ψ̂,ω̂,σ_eff, heading_valid, **lateral_valid = σ_lat_eff ≤ LAT_TOL/2**, gyro_alive, AMBIGUOUS). 30프레임 중앙값은 VERIFY 전용 + KF 2σ 대조.
  | 검증: 정속 직진 bag 3m 구간 KF v̂ 1s std ≤5mm/s, stop 후 0.6s 안 코스팅 끝 ±1cm; 5m 정지 lateral 10회 반복 std(40~50mm 면 5m 판정 금지 확정, 대각 tilt≥15° 에서 ≤15mm 면 허용); 회전 5회 후 재포착 innovation 분포.

## 핵심 결론 3줄 (judge)
1. lateral 은 (d_h, β_px, ψ̂) 에서 명시적 변환으로 재구성, A·x_off 는 추정 파라미터 — **A=1.46 철회**(13/13 회전 불일치, 출처는 자이로 사망 8m 1회). A 확정 전 β_px 조준.
2. heading 신뢰도는 분산 아닌 **기하**로 게이트 — 정면 원뿔의 오차는 spread 0.02~0.06° 에 바이어스 +1.1~1.8° 인 매끈한 바이어스라 σ·NIS·err비·pitch-cue 를 모두 통과한다. 18:14 T 근처 회전 6회 반복의 원인은 팔 길이가 아니라 **정면 heading 바이어스 × d**(당시 A=0 으로 돌았음).
3. 연속 추정기 = heading KF + 위치 KF 유니사이클, 단일 epoch 시계 + 캡처 타임스탬프 + queue=1 + stale 정책; covariance 는 교차센서→진값→창별 커버리지 순으로 보정된 뒤에만 게이트.

## 남는 리스크 (judge top 2)
- A 의 부호·크기 미확정인 채 pivot 변환·SE(2) 전파·predict-only 가 전부 A 에 걸려 있다. 첫 실험을 정면에서 하거나 1회로 끝내면 바이어스가 A 로 둔갑(1.46 이 그렇게 생겼다).
- 정면 원뿔의 매끈한 heading 바이어스는 분산 기반 어떤 게이트도 통과하는 false-confidence 경로인데, 유일한 방어선 θ_min(d)·b(d,tilt) 가 아직 미측정(15° 는 가정) → 정적 격자 전에는 lateral_valid 를 3.5m 이하·oblique 에만.

## 미확정 (실차로 결정)
A 값·부호(|A|≲0.4, Δβ/Δψ 0.88 이 A≈−0.4 인지 A≈0+IMU 6~10% 과대인지) · x_off · 카메라–IMU Δψ 차 0.3~2.3° 의 원인(스프링백/바이어스/스케일) · D435i 컬러 왜곡 coeffs 가 0 인지 · θ_min(d) · h(config 1.1m vs 로그 0.85~1.2m) · 자이로 타임스탬프 도메인 · t_readout · 5m lateral 유효성 · σ̂_c 창 길이.
