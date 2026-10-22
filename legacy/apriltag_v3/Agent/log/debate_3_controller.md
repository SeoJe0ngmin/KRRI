# 대문제 3 — Controller 토론 기록 (2026-09-20~21)

- 실행: `debate_workflow.js` n=3, run `wf_5d42d95d-b53`(상수 최소화 원칙 + plan 1·2 결론 참조). Fable 5.1 effort max, 에이전트 5개(proposer / critic 안전·정밀 / 재반박 / judge), 폴백·오류 0, 101분(정밀 critic 48분), 1.02M 토큰.
- judge 1차 hold 0 → researcher 미가동. verdict: compromise 5 / critic 2(쟁점 2·5) / proposer 0. 두 critic 이 proposer 의 9/7 인용을 원본 jsonl 로 재계산해 전부 재현했고, 근거 사례의 **해석**을 뒤집은 것이 이번 토론의 핵심.
- **오케스트레이터 검산**: 181437 8→9→10 반전(Δψ_cam −4.53/+3.06 vs IMU −6.74/+5.09, lateral 0.423→0.319→0.399, 회전 settle 후 1.22 s 뒤 결정) — debate_1 타임라인과 일치. 185709 결정 1~3 이 `stable:false`(lateral 흔들림 123.9 mm·17.6배) 인 채 sidestep·aim_drive 실행, `plan_step` 의 stable/reliable_angle 참조는 `_fmt_measure` 로그 포맷뿐. `STEP_M`·`FWD_SAFETY` v3 src/tools 참조 0. 삭제 후보 22개 전부 `config/control.py` 에 존재. `detection_tag.py` docstring "화면을 직접 보는 쪽이 무조건 맞다", `next_set3_step` 360°, `recover_backup`, `fwd_abort_deg`, `_hold` SETTLE_SEC 시간 대기 — 줄번호 일치. measure #21 뒤 final 5.97 s 직진의 최종 pose 기록 없음.

## 쟁점 1: Controller state machine
- proposer: 전이표(enum) OBSERVE→DECIDE→EXECUTE→STOPPING→SETTLE→VERIFY(+REACQUIRE·FALLBACK), 명령은 DECIDE→EXECUTE 에서만 래치·이후 stop 전용, OBSERVE 탈출 = σ 수렴, VERIFY 3조건. 근거: 181437 8→9→10 반전쌍(Failure 3), `_hold` 가 SETTLE_SEC 시간 대기만, 185709 가 stable:false 로 실행, autodock A11.
- critic(안전): 골격 동의. **반전쌍은 관성 중이 아니라 settle+30프레임 뒤 정지 상태 측정에서 나왔다**(rotation #7 ts →decision #9 1.22 s) → STOPPING/SETTLE 로 못 막고 "정지 후 2 s 반전 0" 은 v2 도 만족하는 공허한 KPI. OBSERVE 탈출을 σ 수렴으로 두면 stderr 2.7 mm 인 채 104 mm 튀는 매끈한 바이어스가 통과(Failure 2). 중단 술어에 stale·gyro dead·emergency 누락. `next_set3_step` 360° 탐색·`recover_backup` 은 추정기 있는 도킹에서 위험.
- critic(정밀): 같은 반전 재계산(카메라가 IMU 회전의 0.67·0.60 배만 따라감). `heading_valid=False → 축 hold` 는 plan 1-3 이 T(정면)를 heading_valid=False 로 정의하므로 T 에서 매 VERIFY hold 의 자기모순. CAN 상태 소유자 1개(TX 스레드), FINAL 태그 실종을 REACQUIRE 로 보내면 정지점 뒤 후진 사고.
- proposer 재반박: **수정** — 골격 유지, OBSERVE 탈출 = 하드 플래그 ∧ 최소 프레임 ∧ 직전 프리미티브 사후 일관성(쟁점 5 ⓪), heading 축 입력은 σ_ψ̂,eff, 중단 술어 추가, TX 스레드 단일 소유(IMU 스레드는 stop 요청만·10 ms 틱이면 0.08° 라 plan 2-4 와 양립), REACQUIRE 는 DECIDE 경유 1회·360° 폐기, KPI = VERIFY 후 반전 ≤1/도킹.
- judge: **compromise** — rotation/decision 시각 재계산 일치, `_hold`·`_fmt_measure`·`next_set3_step`·`recover_backup` 코드 확인, plan 1-6/2-2/2-3 확정 결론 반영 요구.
- 결론: 전이표 골격 + DECIDE 래치·stop 전용; OBSERVE 탈출 = 하드 플래그 ∧ 최소 프레임 ∧ 사후 일관성, σ 수렴은 연장 조건만; EXECUTE 중단 술어 = 종료식(plan 2-1·2-4) + stale>150 ms + 블라인드 gyro dead + 가시성 픽셀 여유 + 하드캡 emergency; CAN 소유자 TX 스레드 1개; REACQUIRE = KF 예측 β 방향 ≤반화각 1회(DECIDE 경유), 360° 탐색·FINAL 중 recover_backup 폐기; heading_valid 는 "카메라 갱신 가능" 의미로 축소.
  | 검증: 3.5 m·tilt 25° 정지 → β_px 조준 회전 ×10, VERIFY 후 반전 ≤1/10; 검출 스레드 300 ms 부하 주입 → stale 술어 stop 확인.

## 쟁점 2: pivot decouple — A 소거 배치 vs 실시간 도출
- proposer: 안 C = 결합은 plan 1-1 `T_tag←pivot` 에서 한 번 끊고 Controller 는 **A 항이 0 이 되는 배치**에서만 결정(태그 겨냥 β_px=0 은 A 무관, 최종 lateral 판정은 ψ≈0, 정지는 카메라 forward), 회전으로 lateral 을 고치지 않음. 근거: 상수 최소화 원칙, 181437 step 5~10 6연속 회전.
- critic(안전): **반박** — β_cam=0 ⇒ β_piv=0 은 태그 자체 겨냥에만 성립. 축 위 J 겨냥은 tan β_piv = d·sinβ/(d·cosβ+A) 에 A 가 남아 d_J 3 m·β 20°·A ±0.4 → 17.7°/23.0°, 다리 끝 lateral ≈13 cm, d 와 무관해 재조준으로 안 줄어듦. CLAUDE.md 원칙이 "회전팔 A" 를 실시간 도출 예로 명시 → 매 회전 Δβ_px = s·(1+A·cosβ/d)·Δψ_IMU 회귀, J 겨냥 게이트 σ_A·sinβ/d ≤ HEAD_TOL/k.
- critic(정밀): 정면 회전이 카메라를 A·sinα 옮기므로(σ_A 0.5·20° → ±17 cm) J 배치 계획에 Â 가 필요; δ(CAM_YAW_OFFSET)=0 암묵 가정(1°×1.5 m = 26 mm). 8 m·10° 회전 1회로 σ_A≈0.04 m.
- proposer 재반박: **수용** — C+D: Â 매 회전 회귀 갱신(precheck 3.5/6 m ×5 로 s·A 분리가 1차), σ_A → KF P; 배치 규칙은 "A-민감도 최소" 로 격하, 조준 목표 β_px = δ̂; "T 근처 회전 ≤1회" 예측 철회.
- judge: **critic** — 기하 재계산 일치, 원칙 명문, debate_1 판정(|A|≲0.4 미확정) 정합.
- 결론: A 는 회피가 아니라 **실시간 도출**(precheck Â 초기값 + 도킹 중 회전마다 프레임 회귀, σ_A → P). 배치 규칙 유지: 태그 겨냥 β_px=δ̂(A 무관), J 겨냥은 σ_A·sinβ/d ≤ HEAD_TOL/k 통과 시만, 정지는 카메라 forward. x_off 줄자 1회. 삭제 CAM_TO_PIVOT_M.
  | 검증: 회전×5 로 Â·s 확정 후 J 겨냥 3 m 다리 ×3 을 A=0 / Â 로 실행 → 도착 lateral 차 ≈ Â·sinβ; 8 m 첫 조준 회전의 Â 가 precheck Â 와 ±0.05 m.

## 쟁점 3: candidate/cost — 1-step 한계 vs 지평 vs 가이드
- proposer: 가이드(소각 대각 → J 에서 정면 회전 1회 → 축 직진 → 예측 정지) + 후보 5개 롤아웃 종단비용 J=Σ((e²+σ²)/T²) + 반전 페널티; 직접 겨냥 허용식 ℓ·x_stop/d ≤ LAT_TOL(3.5 m → 52 mm, `AIM_FINAL_MAX_LAT_M 0.10` 위반); V-다리 길이 = ℓ/tanα* − (d−x_T). 근거: 181437 step 12~15(T 에서 +25.1° 후 +5.2°·+18.0° → −196 mm 후진 = 1-step 이 미래 손해 못 봄), Yasunobu C11, MERL C12.
- critic(안전): 가이드가 순서를 정하면 실제 결정은 부등식 2개뿐 → 5후보 롤아웃·J 는 dynamics_calib 전부에 의존하는 과설계("필요 증명 후"). "≤8 vs 21" 은 9/7 시작이 포락선 밖이라 비교 불가. backward 67 은 후방 무관측+코스팅 14 cm → 97 전용.
- critic(정밀): **step 12~15 는 지평 탓이 아니다** — #13/#14 tilt 3.9/4.1° 정면원뿔에서 heading −3.39→+3.63(Δcam 7.02 vs IMU 5.61, 비 1.25 = debate_1 정면 바이어스 서명) → lateral −11→−135 mm; perception 게이트가 고칠 오차를 controller 근거로 쓴 이중해석. **V-다리 식은 기하 오류**(현재 heading 후진은 같은 직선 → 축 교점 불변, step 15 후진에 lateral −0.196→−0.918). **허용식 기준점**: DOCK_EXTRA_M 주석대로 카메라 정지 2.02 m = 포크 끝 0.5 m·포크 중심 ≈1.05 m → heading 1° 당 lateral 35/18/8.7 mm, `AIM_FINAL_MAX_LAT_M` 위반 단정 불가. 두 레버: 앵커 바이어스 δ → x_ref·δ, 조준 잔차 r → (d_T−x_ref)·r.
- proposer 재반박: **수정** — 이중해석 인정, V-다리 식 폐기(재접근 = 현재 heading 97 후진, b ≥ 최소 다리 ∧ α'' ≤ 가시성 해), 기준점 x_ref 선행(DOCK_EXTRA_M → CAM_TO_REF_M + STANDOFF_M), 비용함수는 로그만(critic 간 불일치 → critic 1 의 실차 ×5 가 판정 실험), T-VERIFY 수용식 |F̂|+kσ_F ≤ T_l(F̂ = ℓ̂·x_ref/d_T + (d_T−x_ref)·r̂, σ_F = x_ref·σ_ψ̂,eff).
- judge: **compromise** — 정면쌍 재계산 일치, 직선 후진 교점 불변, DOCK_EXTRA_M 주석, Poreski/autodock 고정 순서 선례, STEP_M·FWD_SAFETY 참조 0 grep.
- 결론: 가이드 순서 채택(x_J = T 기본, α* = atan(ℓ/(d−x_J)) 기하 도출), 비용함수·σ_J·롤아웃 미도입(J 로그만), 결정은 부등식 2개(재조준 vs 계속 / T-VERIFY 수용식), 롤아웃은 가시성·정지점 feasibility 만. 재접근 = 97 후진. **x_ref 확정이 코더 선행조건**. 삭제 AIM_FINAL_MAX_LAT_M·AIM_BACKUP_M·AIM_CHUNK_MAX_M·AIM_MAX_BACKUPS·STEP_M·FWD_SAFETY.
  | 검증: 포락선 안 시작(ℓ≈1.5, d≈7) ×5 → 프리미티브 ≤8·반전 0 이면 비용함수 근거 없음 확정; F̂(x_ref) 예측 vs 외부 줄자 포크 lateral ×10 → σ ≤ x_ref·σ_ψ̂(카메라 기준이면 2~4배 = 기준점 검정).

## 쟁점 4: 태그 가시성 — cost 항 vs 하드 제약, 소각 규칙
- proposer: 하드 제약(허용집합): 롤아웃 전 구간 |β_tag| ≤ β_vis(d)(반화각 35.2° − 각반경 − 코스팅), 상단 컷 x_cut(h,VFOV,pitch), 실행 중단 = margin_px − v̂·τ_eff·∂margin/∂x ≤ 0(TAG_CUT_MARGIN_PX 60 대체); α*∈[θ_min 15°, β_vis]; 포락선 밖(9/7 시작 ℓ 4.58/d 8.15 → 47.8°)은 dog-leg 1회 또는 수동 — critic 판정 요청.
- critic(안전): 하드 제약·부등식 대체 동의. (a) α* 하한 θ_min 은 불필요(대각 다리엔 β_px+자이로 앵커면 충분). (b) β_vis 해석식은 검출 한계 아님(`tag_edge_margin_px` docstring "화면을 직접 보는 쪽이 무조건 맞다"). (c) 제동 노즈다운 1° → 태그 상단 +24 px 로 STOPPING 중 컷. 포락선: 가시 상태로 축 합류 불가 → dog-leg 필수지만 ≤10 s(≈2 m) 1회, 시작 포락선은 운영 요구.
- critic(정밀): 동의. x_cut 은 h 미확정(1.1 vs 0.85~1.2)이라 관측 margin_px 와 변화율(≈154 px/m @3.3 m)만. 포락선 밖: 태그 보며 축에 닿는 경로 없음 → FAR 한정 자이로 dog-leg 1회(≈25 s), 시작 포락선(8 m 에서 |ℓ| ≤ 2.8 m) 을 대문제 4 운영 요구로.
- proposer 재반박: **수정** — 판정은 관측 margin_px − [v̂·(τ_eff + t_now−t_capture)]·∂margin/∂x − pitch_px(precheck 67 정지 n=20 상단 상승) 로, 해석식은 로그 대조용; α* 하한은 σ_ψ̂,eff 파생 조건으로 격하; dog-leg 은 짧은 쪽(≈2 m, 9/7 시작도 2 m 뒤 포락선 안); 시작 포락선 대문제 4 이관.
- judge: **compromise** — docstring·abort.jsonl 56/58 px·h 미확정 확인, dog-leg 제약은 블라인드 주행 안전 → 짧은 쪽.
- 결론: 가시성 = 하드 제약(허용집합) + 관측값 기반 중단 술어(실시간), 해석식은 로그용. α* 기하 도출, θ_min 하한은 파생 조건. 포락선 밖 = FAR 한정 dog-leg 1회(97, ≤2 m/≤10 s, 재획득 후 30프레임 재초기화 전 NEAR 결정 금지), 그래도 밖이면 출발 거부; 시작 포락선은 운영 요구(대문제 4). 삭제 TAG_CUT_MARGIN_PX·AIM_MAX_BEARING_DEG·AIM_MAX_TAG_OFF_DEG·SIDESTEP_BACKWARD_GAIN_DEG·SEARCH_BACKUP_M.
  | 검증: 3.5/6 m 정지 저속 회전으로 검출 끊기는 β 좌우 ×3(해석식과 >3° 차면 관측값) + 67 정지 20건 상단 픽셀 상승 → pitch_px; 9/7 시작자세 dog-leg 1회 → 재획득·포락선 안 판정; 포락선 안 시작 20회 태그 손실 0.

## 쟁점 5: 행동 게이트
- proposer: 4항 AND — ① 지속성(부호 불변 ≥10프레임) ② |e| > T + k·σ_eff(+하드 플래그 hold) ③ 최소 기대개선(E[J_after]+kσ_J < J_now ∧ 프리미티브 ≥ 최소 신뢰 증분) ④ 반전 가드(0 통과 시만, 구역당 1회); 회전 데드밴드 = max(HEAD_TOL, θ_min,inc, kσ_θ). 근거: critic §15·16·18, Poreski 스윕, MERL C12.
- critic(안전): **반박** — 4항을 181437 8→9 에 적용하면 **전부 통과해 같은 반전**(30프레임 부호 일정·|e| 6.1 > 3+0.1·개선 ≫ σ·0 통과 1회 허용), 9→10 은 "구역당 1회" 로 abort 로 바뀔 뿐. 원인: oblique 23.6→28°(θ_min 통과)에서 카메라 Δψ 가 IMU 의 0.67·0.60 배 → lateral −104 mm(제자리 회전이면 |A|≤0.4 로 ≤47 mm 만 가능) — SETTLE 의 "정지마다 ψ 재앵커" 가 매번 새 바이어스 표본을 주입하는 반전 기구, σ̂(점프/stderr 38×)는 못 봄. → ⓪ 교차센서 사후 일관성(Δψ_cam vs Δψ_IMU, |Δlat| ≤ A_max·sinΔψ + kσ_random) 초과면 재앵커·반전 금지, 자이로 전파로 재계산(step 9 를 ψ̂=−30.22° 로 풀면 회전 −0.6° → 반전 소멸). 데드밴드 ≈2.5° → 블라인드 누설 56 mm > LAT_TOL.
- critic(정밀): **반박** — 행동 |e|>T+kσ 와 수용 |e|+kσ≤T 사이 **공백 (T−kσ, T+kσ)** 이 항상 존재(σ 12 mm·k 2 → (6, 54) mm 는 고치지도 받지도 못함 → hold→캡→abort). 데드밴드는 HEAD_TOL 이 아니라 결과공간 LAT_TOL/(2·x_ref) 에서. → 3분할: 행동 |e| > max(kσ, δ_q) / 수용 |e|+kσ ≤ T / 판정불가 kσ > T/2 → Tier 1.
- proposer 재반박: **수용** — ⓪ 추가, ② 3분할(겹치면 수용 우선), δ_q = 최소 신뢰 증분의 결과공간 값(θ_min,inc·(d_T−x_ref) / 97 최소 증분), ③ 은 F̂ 공간, ④ 는 ⓪ 뒤에.
- judge: **critic** — 재생 재계산 일치, Failure 2, MERL 양자화 불변집합(T ≥ kσ + δ_q/2), plan 1-5 교차센서가 이미 1차 방어선.
- 결론: 게이트 5항 ⓪ 사후 일관성(초과면 재앵커·반전 금지, ψ̂ 자이로 전파, lateral = d·sin(β_px−ψ̂) 재계산; 재앵커는 innovation < kσ_random 또는 앵커 나이 한도 시 다수결만) ① 지속성 ② 3분할 ③ 기대개선(F̂ 공간) ④ 반전 가드. 데드밴드는 결과공간·θ_min,inc·kσ_θ 에서 도출. 삭제 AIM_TOL_DEG·AIM_FINAL_TOL_DEG·AIM_ABORT_WINDOW·AIM_DRIFT_ABORT_DEG(직진 중단은 `fwd_abort_deg` 하나, 레버 = x_cam_now−x_ref).
  | 검증: 3.5 m·tilt 25° 정지 IMU 회전 ±5° ×10 → |Δψ_cam−Δψ_IMU| > kσ_random 비율 >10% 면 "정지마다 재앵커" 금지 확정; bag 재생 (|e|,σ) 산점도에 공백 표본 0.

## 쟁점 6: Zone 전환 hysteresis·zone 별 목표
- proposer: 구역 = 판정가능성+이벤트: FAR = lateral_valid=False(β 재중심·가이드 진행, lateral/heading 결정 금지) / NEAR = lateral_valid 지속 True / FINAL = "정면 회전 완료 + T VERIFY 통과" 이벤트; 전환은 VERIFY 에서만, hysteresis = τ_eff·v̂ + 2σ_x, T = x_cut + 코스팅 + 여유. 삭제 AIM_NEAR_T_M·AIM_AT_T_M·AIM_STANDOFF_M.
- critic(안전): σ_floor 격자 전엔 c·σ̂ 만으로 8.15 m(stderr 2.9 mm) 에서 NEAR 진입 = v1 유령 추종 부활; "3.5 m 이하 제한" 은 거리 상수 위장 → σ_floor·θ_min·b·A·s·x_off·δ·h·β_vis 를 **perception_calib.json** 으로 묶어 부재 → 출발 거부. FINAL gyro dead 누락. hysteresis 는 정지 평가라 v̂≈0. "여유" 는 새 상수.
- critic(정밀): FAR 에서 "lateral 결정 금지" 는 가이드 β_J 가 lateral 을 쓰는 것과 자기모순(8 m SNR 30 → 방향 결정 가능). NEAR 진입을 d 레버(3.5 m → 61 mm/°)로 재면 x_ref 레버보다 1.7~3.3배 엄격 → "태그 컷 전 True 안 됨" 거의 확정 → 구역 = 결정 유의성: FAR |ℓ̂| > kσ_lat / NEAR σ_F = x_ref·σ_ψ̂,eff ≤ T_l/2 / FINAL 이벤트.
- proposer 재반박: **수용** — 두 안 합침(σ_F 의 입력이 perception_calib), FINAL gyro dead → stop·ABORT, hysteresis = 다음 다리 계획 속도의 D̂ + 2σ_x, 여유 = q95⁺ + 제동피치 px 파생.
- judge: **compromise** — measure #1 spread 2.9 mm 확인, Wang A14·Yilmaz&Temeltas, plan 2-6 출발 거부 대칭.
- 결론: FAR = |ℓ̂| > kσ_lat(방향 결정만; 67) / NEAR = σ_F ≤ T_l/2(heading_valid 인 oblique 다리에서 진입; 입력은 perception_calib.json, 부재 → 출발 거부, 임시 거리 규칙 없음) / FINAL = 이벤트 진입(회전 금지, 97, 자이로 짧은 다리, gyro dead → ABORT). 전환은 VERIFY 에서만, hysteresis·여유는 캘리브 파생. 삭제 AIM_NEAR_T_M·AIM_AT_T_M·AIM_STANDOFF_M.
  | 검증: oblique 5→3.5 m 직진 ×10 의 σ_ψ̂,eff 시계열로 NEAR 진입 거리 — 3.5 m 이전 ≥9/10 이면 확정, 미달이면 대문제 4; 경계 ±2σ_x 잡음 시뮬 FAR↔NEAR 왕복 0.

## 쟁점 7: "불확실하면 기다림" deadlock — timeout·fallback·수동전환
- proposer: 카운트 상수 5개 → 진행도(J 감소) + 시간 예산 `DOCK_TIME_BUDGET_S`(유일 신설 config). Tier 0 WAIT(캡 3 s) / Tier 1 REPOSITION(원거리 → 대각 진행, 정면 → 자이로 앵커 60 s) / Tier 2 RETRY(FINAL VERIFY 실패·반전 2회 → T 후진 재실행 1회) / Tier 3 ABORT→수동(집합 열거).
- critic(안전): **Tier 2 트리거 "FINAL VERIFY 실패" 는 관측 불가** — 최종 pose 를 카메라가 못 봄(181437 도 measure #21 뒤 final 5.97 s 기록 0) → "눈 감고 DONE". 자이로 60 s 는 "짧은 다리" 위반이자 새 상수. ABORT 가 버스 침묵이면 마지막 forward 유지 위험. → DONE-unverified, 관측 가능 트리거(블라인드 직진 중 자이로 Δψ > fwd_abort_deg), ABORT = stop 프레임+heartbeat 지속.
- critic(정밀): 진행도 J 는 정면 회전 직후 A·sinα 이동으로 일시 증가 → 정상 수렴을 정체로 오판. 자이로 한도 = LAT_TOL/(2·x_ref) ÷ 드리프트(26~48 s) 로 파생. Tier 2 일괄 재접근은 원인이 회전 양자화면 재현 → 원인 분기.
- proposer 재반박: **수용** — 전부.
- judge: **compromise** — measure/drive 시각 확인, autodock·Nav2 재시도 선례, IEC 62745(침묵 금지), Curtis PDO Timeout 0 = 마지막 명령 유지.
- 결론: 진행도 = 단계 완료 이벤트 카운트(J 는 로그), 시간 = DOCK_TIME_BUDGET_S. Tier 0 WAIT(σ 아직 감소 중, 캡 코드 내부) / Tier 1 REPOSITION(한도 = σ_ψ̂(t)·x_ref ≤ T_l/2 깨지는 시각, 실시간 도출) / Tier 2 원인 분기 1회(수용식 초과 → 97 재접근 / 태그 실종(FINAL 제외) → REACQUIRE / 블라인드 자이로 Δψ > fwd_abort_deg → stop·97 후진·재VERIFY / 양자화 공백 → 즉시 Tier 3 + "저속 회전 단 필요" 로그) / Tier 3 ABORT = stop 프레임 + heartbeat 지속 + 운영자 알림. FINAL 은 대문제 4 해소 전 **DONE-unverified(DR)**. 삭제 HOLD_MAX_CONSEC·MAX_STEPS·AIM_MAX_BACKUPS·SEARCH_MAX_ROUNDS·SEARCH_AFTER_MISSES.
  | 검증: 고장주입 4종(AMBIGUOUS 10 s·lateral_valid False·자이로 표류 2°·양자화 공백) 각각 캡 안 Tier 도달·반전 0 + CAN 로그 stop 프레임 연속; 최종 정지 ×10 외부 줄자(σ > T_f/3 이면 unverified 유지); 정지 60 s 자이로 랜덤워크로 Tier 1 한도 갱신.

## 핵심 결론 3줄 (judge)
1. 컨트롤러 = 전이표 상태기계(DECIDE 래치, 이후 stop 전용, CAN 소유자 TX 스레드 1개) + 가이드(대각 다리 → 태그 겨냥 β_px=δ̂ → 블라인드 직진) + 부등식 게이트 2개. 비용함수·N-step 지평은 근거 소멸(step 12~15 는 정면 heading 바이어스, 8→9→10 반전은 관성이 아니라 정지마다 카메라 재앵커) → J 는 로그만. **교차센서 사후 일관성 ⓪ 없이는 v2 반전이 v3 에서 그대로 재생된다.**
2. 모든 문턱은 **기준점 x_ref 두 레버**(앵커 바이어스 δ → x_ref, 조준 잔차 r → d_T−x_ref)에서 도출: 게이트 3분할, 구역 = 결정 유의성(FAR |ℓ̂|>kσ · NEAR σ_F ≤ T_l/2 · FINAL 이벤트), A 는 매 회전 회귀로 실시간 도출, perception_calib.json 부재 → 출발 거부. 거리 상수 0, config 순감 ≈20. **x_ref(탑재부 사양) 확정이 코더 선행조건.**
3. (d_T−x_ref)·σ_θ ≈ 2.45 m×0.5° = 21 mm, θ_min,inc/2 면 28 mm 라 **T 에서의 회전 양자화가 lateral 예산 30 mm 를 거의 다 먹는다** — 마지막 1.28 m 는 DONE-unverified, 양자화 공백은 즉시 수동 + "저속 회전 단 필요" 로그; 근본 해소(레버 단축·태그 높이·depth 정지·시작 포락선)는 대문제 4.

## 남는 리스크 (judge top 2)
- x_ref 미확정 + 회전 양자화: 확정 전엔 허용식·lateral_valid·zone 어느 상수도 맞는지 판별 불가; 확정돼도 σ_θ 0.5°·레버 2.45 m 면 1회 시도 |r|≤0.7° 성공 ≈84% → 99% 도킹은 저속 회전 단(plan 2-4 조건부) 또는 태그 가시거리 단축(대문제 4) 없이는 원리적으로 불가.
- 교차센서 중재의 방향: 카메라–IMU Δψ 불일치(2°급, 원인 미확정) 때 "재앵커 금지·자이로 전파" 는 카메라가 옳은 경우 바이어스를 안고 다음 다리를 달린다(재관측으로 고침); σ_random·b(d,tilt)·σ_floor 는 정적 격자(실차 2일차) 전엔 없어 perception_calib 부재 = 도킹 불가 — 3.5 m·tilt 25° ±5°×10 분포 실험과 정적 격자 없이는 게이트·구역이 전부 미검증.

## 미확정 (실차·사용자)
**x_ref = 탑재부 기준점(포크 끝/포크 중심/카메라) — 사용자 확정 항목** · 카메라–IMU Δψ 차 원인(어느 센서가 옳은가) · SETTLE 문턱 vs 97 크립 진동 겹침(캡 발화율) · 8 m 단일 회전의 Â 분해능(s 정밀도 종속) · 검출 끊기는 β 관측값·pitch_px · NEAR 진입 거리(태그 컷 전 σ_F ≤ T_l/2 가 한 번도 안 되면 대문제 4 하드웨어) · 재접근 1회 성공률(≈84%, judge 추정) · 시작 포락선 운영 규정.
