모든 근거 파일을 읽었다. 아래가 대문제 3 제안이다.

# 대문제 3: Controller — proposer 제안

전제(plan 1·2 결론): A 미확정(β_px 조준), 플래그 `heading_valid`/`lateral_valid`/`gyro_alive`/AMBIGUOUS, 정지 발령식 `x_rem(카메라 forward) ≤ τ_eff·v̂ + v̂·Δt/2`, 회전 ω 규칙(σ_θ ≤ 0.5° 요건), byte2 = 이산 2단 {67, 97}, 정지 판별 2모드. 코드 기준선은 `/Users/seojeongmin/krri/apriltag_v3/src/models/control/control_from_pose.py`(`plan_aim`·`plan_step`·`dock_live`), 실측 기준선은 9/7 `181437`(21스텝 성공) · `185709`(실패) 로그.

### 쟁점 1: Controller state machine
- 제안: 상태 = OBSERVE→DECIDE→EXECUTE→STOPPING→SETTLE→VERIFY(+REACQUIRE, FALLBACK). **명령은 DECIDE→EXECUTE 전이에서만 래치**되고, EXECUTE 이후엔 어느 상태도 `stop` 외의 CAN 상태 변경을 할 수 없게 **전이표(enum)** 로 막는다(플래그 아님). 진입/탈출: OBSERVE = KF σ_eff 가 쟁점 5 게이트를 "행동/완료" 어느 쪽으로든 판정 가능한 수준까지 수렴(고정 N 아님, 캡→쟁점 7) → DECIDE(순수함수 1틱; 명령에 **종료 술어**(plan 2 발령식·ω 규칙) + **중단 술어**(가시성·heading 누설·gyro gaps·하드 캡) 동봉) → EXECUTE(술어 감시만, 후보 재평가 금지) → 술어 발화 = stop TX → STOPPING(plan 2-3 정지 판별 2모드 + 코드 내부 캡, Q 과도창) → SETTLE(|ω|<γ_ω 창에서 b_g 재추정·oblique 면 ψ 재앵커·P 재수렴, 최소 0.3 s) → VERIFY(30프레임 중앙값 ↔ KF 2σ 대조, 허용오차+σ+정지 3조건) → DONE / DECIDE / REACQUIRE / FALLBACK.
- 근거: critic §13·§14·§25·Failure 3; autodock(A11) predock→steer→last_mile 상태기계, Wang 2021(A14) approaching→switching→docking. 코드: 현행은 `_execute()` 끝 `driver.stop()` 뒤 `CanDriver._hold` 가 `SETTLE_SEC` 0.8 s **시간 대기** 후 곧장 30프레임 측정 — 정지 확인 없음(plan 2-2 가 SETTLE_SEC 삭제). 185709 step 1~3 은 `stable:false`("lateral 흔들림 123.9 mm, 예측보다 17.6배")인 측정으로 aim_drive·sidestep 실행 — `plan_step` 은 `stable` 을 읽지 않는다(`control_from_pose.py:68`·`:437` 은 로그용). 9/7 181437 step 8→9→10 = cw −6.7°→ccw +6.1°→cw −3.7°(반전쌍).
- 예상 효과: 관성 중 반대명령(Failure 3) 이 구조적으로 불가능. 예측: 9/7 로그 재생(`DryRunDriver`) 에서 "정지 후 2 s 내 반전" 0건, 실차 도킹당 반전 ≤1(현행 181437: 반전 2회 + 후진 1회). 검증 = 로그 재생 + 실차 10회 반전 카운트.
- 상수: 새 config 0 / 실시간 도출 3(OBSERVE 종료 = σ 수렴, SETTLE 문턱 γ, 캡은 plan 2 값) / 삭제 `SETTLE_SEC`(이미), `HOLD_MAX_CONSEC`·`MAX_STEPS` → 쟁점 7 예산으로 흡수.
- 리스크(worst): 정지·크립 진동 통계가 겹쳐(plan 2-3 미확정) SETTLE 이 매번 캡으로 끝남 → 사실상 시간 기반 settle 로 회귀(느려지지만 안전). 캡 발화율을 KPI 로 기록.

### 쟁점 2: 회전중심(pivot) 좌표로 decouple — 어디서 한 번 끊나
| 안 | 끊는 곳 | 장 | 단 | 전제 |
|---|---|---|---|---|
| A. Perception 이 pivot pose 출력 | KF 상태 [x_p,y_p,v] (plan 1-7) | 한 곳, σ_A 가 P 로 전파 | A 오차가 회전마다 innovation 튐 | A 추정 필요 |
| B. Controller 가 카메라값에 A 적용 | 컨트롤러 내부 | 추정기 단순 | A 가 두 곳(추정·제어)에 흩어짐, 9/7 `plan_aim` 이 이 구조 | A 상수 |
| **C. A 유지 + "A 소거 배치" 규칙**(제안) | A(추정기) + B 의 결정변수만 A-불변으로 | A 미확정이어도 결정이 안 틀림 | 결정 배치가 제한됨 | x_off 실측 |
- 제안: C. 결합은 Perception 의 `T_tag←pivot`(plan 1-1) 에서 **한 번** 끊고, Controller 는 **A 항이 0 이 되는 배치에서만 결정**한다: (i) 조준 회전 목표는 β_px=0 — `tan β_piv = d·sinβ_cam/(d·cosβ_cam + A)` 라 β_cam=0 ⇒ β_piv=0 (A 무관); (ii) 최종 lateral 판정은 ψ≈0 배치에서 — `lat_p = lateral − A·sin ψ` 의 A 항 소멸; (iii) 정지는 카메라 forward(plan 2-1). **회전으로 lateral 을 고치지 않는다**(회전은 β 재중심 1종과 J/T 에서 "태그 정면 향하기" 1회만); 잔여 lateral 은 쟁점 3 의 V-다리로. 회전이 lateral 에 주는 영향 예측(methodology §11) 은 하지 않고 재관측으로 대체.
- 근거: 상수 최소화 원칙("실시간으로 안 보이는 오차는 회피 규칙 1개로"); plan 1-1(A∈[−0.5,0.5] 미확정, β_px 조준 이관). 9/7 181437 step 5~10: dist_t 0.83~0.92 m 에서 회전 6연속, 측정 lateral 0.046→0.158→0.384→0.423 m(forward 3.9 m 고정) — A=0 로 계산한 T 방위각이 회전마다 이동 + step 5·6 은 tilt 4.4°/2.9° 정면원뿔(heading 바이어스×3.9 m). Walter 2010(A1: KF+ey·eθ 조향), Richter 2022(A9: marker-centering 단계).
- 예상 효과: "회전→lateral 변화→재회전" 루프 계열 소멸. 예측: T 근처 회전 ≤1회/도킹(현행 6+3). 검증: 시뮬 A∈{−0.5,0,+0.5} 3값 × 9/7 시작자세 → 도착 lateral 차 < 5 mm(A 무관성).
- 상수: 새 config 0(`X_OFF_M`·`A_M` 은 plan 1 항목) / 실시간 0 / 삭제 `CAM_TO_PIVOT_M`(→ 캘리브 `A_M`).
- 리스크(worst): x_off(카메라↔포크 중심 횡오프셋) 가 0 이 아니면 β=0 정렬이 x_off 만큼 상수 편향 — 장착 기하(물리 상수) 이므로 줄자 실측 1회로 config 허용, 미측정 시 30 mm 예산을 통째로 먹을 수 있음.

### 쟁점 3: candidate/cost 컨트롤러 — 1-step 한계 vs 지평
| 안 | 요지 | 장 | 단 |
|---|---|---|---|
| 1-step cost | 다음 pose 만 평가 | 단순 | 대각 lateral 보정 못 찾음(이득이 미래) — critic §18 |
| N-step MPC | 명령 트리 탐색 | 원리적 | 5명령×느린 dynamics 에 과설계, 모델오차(τ_eff·σ_θ)로 깊은 지평 무의미 |
| **가이드 + 종단비용**(제안) | 기하 가이드가 2~3 프리미티브 참조열 생성, 후보는 **가이드 끝(정지점) 예측 pose** 로 비용 평가 | 미래 이득이 1-step 선택에 보임, 후보 ≤5 | 가이드 형상에 의존 |
- 제안: 가이드 = "소각 대각(α*)으로 축 위 J 에 합류 → J 에서 태그 정면 회전 1회 → 축 직진 → 예측 정지". 후보 프리미티브 {rotate-to-β_J, rotate-to-face, forward(종료 술어 동봉), backward(V-다리: 전방 여유 부족 시만), hold} 각각을 잔여 가이드까지 **롤아웃**(캘리브 τ_eff·σ_θ·S(T) 잔차 σ 포함) 해 종단 오차로 비용 `J = Σ((e_i²+σ_i²)/T_i²)`(critic §17 정규화, 가중치 없음 — 구역차는 쟁점 6 의 허용 프리미티브 집합으로) + 반전 페널티. **직접 겨냥 허용식**(상수 대신 허용치에서 도출): `ℓ·x_stop/d ≤ LAT_TOL ∧ atan(ℓ/d) ≤ HEAD_TOL` (x_stop 2.02 m, d 3.5 m → ℓ ≤ 52 mm; `AIM_FINAL_MAX_LAT_M=0.10` 은 이 식을 위반). V-다리 길이 = `ℓ/tanα* − (d − x_T)` 로 도출.
- 근거: Yasunobu 1985(C11: 후보 노치별 정지위치 예측→선택, 상용화), MERL US10093331(C12: 유한 명령집합 + 불확실성만큼 축소), autodock `parallel_correction`(회전–직진–회전 = 우리 가이드의 90° 판)·Poreski β=90°−α 를 α* 로 소각화, critic §16·§18. 9/7 step 12~15: T 에서 "태그 겨냥 +25.1°" 후 다시 +5.2°·+18.0° 회전 → lateral −196 mm 로 후진 — 1-step 이 미래 손해를 못 봄.
- 예상 효과: 예측: 9/7 시작자세에서 T 정렬까지 프리미티브 ≤8(현행 21), 정면원뿔 내 회전 = "정면 향하기" 1회만. 검증: 21개 측정 상태 오프라인 재생 + 시뮬(A 3값, τ_eff ±20%) 수렴 스텝 분포.
- 상수: 새 config 0 / 실시간 3(직접겨냥 허용식, V-다리 길이, σ_J) / 삭제 `AIM_FINAL_MAX_LAT_M`, `AIM_BACKUP_M`, `AIM_CHUNK_MAX_M`(다리 길이는 종료 술어·가시성이 정함), `STEP_M`·`FWD_SAFETY`(v3 src 참조 0 — 미사용 잔재).
- 리스크(worst): 롤아웃의 nominal dynamics 가 틀리면(τ_eff 이동·σ_θ>0.5°) 종단비용이 편향 → 매 프리미티브 후 OBSERVE 재계획(수렴지평)으로 구제되지만, 전방 여유 부족 상태에서 V-다리를 반복 선택 → 쟁점 7 예산이 끊어야 함.

### 쟁점 4: 태그 가시성 제약 — cost 항 vs 하드 제약 + 소각 대각 규칙
| 안 | 장 | 단 |
|---|---|---|
| cost 항 | 부드러운 절충 | lateral 큰 상태에서 "조금 안 보임" 을 최적화가 수용 → 9/7 sidestep 계열 실패 재현, 블라인드 결정 |
| **하드 제약(허용집합)**(제안) | 손실 자체가 불가능 | 시작 자세 포락선이 필요 |
- 제안: 프리미티브 허용조건 = 롤아웃 전 구간 `|β_tag| ≤ β_vis(d)`, β_vis = 반화각(atan(960/fx)=35.2°) − 태그 각반경 atan(s/2d) − 회전 코스팅 ω̂·τ_r; 직진은 **상단 컷** 예측 `x_stop ≥ x_cut(h, VFOV, pitch)`(장착 기하 상수에서 계산). 실행 중 중단 술어는 `margin_px − v̂·τ_eff·(∂margin/∂x)(d) ≤ 0` — 코스팅을 픽셀로 환산해 **실시간 도출**(`TAG_CUT_MARGIN_PX=60` 대체; 9/7 abort 56/58 px @3.26~3.30 m). **소각 규칙**: (x_J,0) 을 향한 다리의 종단 태그오프 = `atan(ℓ/(d−x_J))` 이므로 허용 ⇔ ≤ β_vis; 접근각 α* 는 `[θ_min(d)≈15°, β_vis]` 에서 택해 **대각 다리 내내 heading_valid** 를 확보(정면원뿔 회피를 경로 설계로 승격). 포락선 밖(`ℓ > (d−x_T)·tanβ_vis`, 9/7 시작 ℓ 4.58/d 8.15 → 47.8° 로 밖) 은 **자이로 유지 dog-leg 1회**(회전→짧은 직진→재획득·β 재앵커, ≤20 s = "짧은 다리") 또는 시작 포락선 위반 → 쟁점 7 수동전환 — 어느 쪽인지는 critic 판정 요청.
- 근거: Abbas 2019(화상 중앙·25~75° 관측각 최적), Adámek 2023(정면 yaw 분산 피크), Richter 2022(±25° 원뿔 회피), Walter 2010(7.5 m/횡 3 m 시작은 검출 실패 = 포락선), Nav2 A12(마커 재검출 실패 시 재시도, 절충 안 함). 코드: `AIM_MAX_BEARING_DEG 45`·`AIM_MAX_TAG_OFF_DEG 25` 두 상수가 위 부등식 하나로 대체됨.
- 예상 효과: 포락선 안 태그 손실 0. 예측: 시뮬 20 시작자세(안/밖) — 안 0 손실, 밖 dog-leg ≤1. 검증: 9/7 step 4 상태(bearing 48.6°)를 **출발 전** 비허용으로 분류(v2 는 3 m 다리 2개 달린 뒤 발견).
- 상수: 새 config 0 / 실시간 3(β_vis(d), x_cut, 코스팅 픽셀) / 삭제 `TAG_CUT_MARGIN_PX`, `AIM_MAX_BEARING_DEG`, `AIM_MAX_TAG_OFF_DEG`, `SIDESTEP_BACKWARD_GAIN_DEG`, `SEARCH_BACKUP_M`(복구 후진 = 직전 다리 길이 KF 값).
- 리스크(worst): 상단 컷 3.3 m(h≈1.1 m) 은 규칙으로 못 없앤다 — 대문제 4(태그 높이/T 4.0 m/depth 정지) 미해소면 마지막 1.28 m 는 이 제약과 무관하게 블라인드.

### 쟁점 5: 행동 게이트 — 구체 조건
- 제안: 게이트 = 4항 AND, 입력은 전부 추정기 출력. ① **지속성**: 보정축 오차의 부호가 SETTLE→VERIFY 창(≥10프레임 ∧ σ_eff 가 floor 도달) 내 불변. ② **불확실성 조정 문턱**: `|e| > T_i + k·σ_eff,i`(σ_eff = max(c·σ̂, σ_floor(d,tilt)), plan 1-5); `heading_valid=False`/`lateral_valid=False`/AMBIGUOUS/`gyro_alive=False` 는 그 축 **판정 불가(hold)** 하드 플래그. ③ **최소 기대개선**: 쟁점 3 롤아웃 `E[J_after] + k·σ_J < J_now` ∧ 프리미티브 크기 ≥ 캘리브 최소 신뢰 증분(회전 θ_min,inc, 97 ≈0.12 m, S(T) CV<30% 최소 T). ④ **반전 가드**: 직전 실행 프리미티브와 반대 부호는 정지 후 오차가 `T + k·σ_eff` 를 넘어 **0 을 지났을 때만**, 구역당 1회(2회째 → FALLBACK). 회전 데드밴드 = `max(HEAD_TOL, θ_min,inc, k·σ_θ)` 로 도출(`AIM_TOL_DEG 3.0`·`AIM_FINAL_TOL_DEG 1.5` 의 근거 "회전 잔차 ±1.5°" 는 plan 2-4 가 폐기).
- 근거: critic §15·§16·§18, Poreski 임계각 스윕(0.5°→23 %, 2°→53 %, 4°→27 %: 잡음 아래 문턱은 정지 과다 — ②가 σ 로 문턱을 자동 상향), MERL C12(양자화 액추에이터), Autoware/Nav2 stall(C21). 9/7: 7~10 m lateral σ 60~145 mm → `T+2σ ≥ 150 mm` 라 원거리 lateral 은 판정 불가 → β 만 행동(의도된 far 동작이 게이트에서 **도출**됨). 185709 step 2 는 heading 흔들림 0.95°·17.6배인데 sidestep 실행.
- 예상 효과: 유령 오차 추종·chatter 제거. 예측: 9/7 프레임별 bag 재생 시 게이트 통과 결정 ≤ v2 결정의 1/2, 반전 0. 검증: bag(plan 1 요구) 오프라인 재생 + 실차 반전 카운트.
- 상수: 새 config 0 / 실시간 4(σ_eff, σ_J, 최소증분, 데드밴드) / 코드 내부 k=2·10프레임·반전 1회 / 삭제 `AIM_TOL_DEG`, `AIM_FINAL_TOL_DEG`, `AIM_ABORT_WINDOW`, `AIM_DRIFT_ABORT_DEG`(중단 술어는 기존 `fwd_abort_deg` = asin(LAT_TOL/잔여거리) 하나로).
- 리스크(worst): σ_floor(d,tilt) 는 정적 격자 전엔 없고 프레임 σ̂ 는 정면원뿔에서 5~7배 과소(plan 1-4) → ②가 거짓 통과. 그 전엔 `lateral_valid`·`heading_valid` 하드 플래그를 3.5 m 이하·oblique 로 제한(plan 1 리스크와 동일 방어선).

### 쟁점 6: Zone 전환 hysteresis 와 zone 별 목표
- 제안: 구역은 거리 상수가 아니라 **판정가능성 + 이벤트**로: FAR = `lateral_valid=False` 인 동안(목표: β 재중심·가이드 진행, 67, lateral/heading 결정 금지); NEAR = `lateral_valid` 가 지속창 동안 True(목표: J 합류·정면 회전·V-다리, backward 는 여기서만); FINAL = **"정면 회전 완료 + T 에서 VERIFY 통과" 이벤트**로 진입(목표: plan 2 예측 정지, heading 은 자이로 짧은 다리, 블라인드 구간 97, 회전 전면 금지 — 실패 시 회전 대신 FALLBACK 후진). 전환 판정은 **VERIFY(정지 상태)에서만**, 거리 성분 hysteresis 폭 = 코스팅 `τ_eff·v̂ + 2σ_x`(실시간) 로 정지 오버슈트가 경계를 되넘어도 안 뒤집힘. T 는 `x_cut + 코스팅 + 여유` 로 파생(대문제 4 값 확정 시 자동 갱신).
- 근거: critic §20·Failure 7, Wang 2021(A14 3영역), autodock(last_mile 0.5 m 오도메트리), Nav2(staging 0.75 m), Richter(3.45 m→1 m 단계마다 정지·재측정), Yilmaz & Temeltas 2024(전환 시 추정 점프 → 정지에서 전환). plan 1-7: 5 m lateral σ 40~50 mm > 30 mm → NEAR 진입 거리는 측정으로 정해질 값이라 상수화 금지.
- 예상 효과: 매 프레임 구역 재평가가 없으니 chattering 원천 제거. 예측: 구역 왕복(FAR↔NEAR) 0/도킹. 검증: 시뮬 경계 ±코스팅 잡음 주입 + 실차 로그의 zone 열.
- 상수: 새 config 0 / 실시간 3(NEAR 진입, hysteresis 폭, T 파생) / 삭제 `AIM_NEAR_T_M`, `AIM_AT_T_M`, `AIM_STANDOFF_M`(파생) — `DOCK_EXTRA_M` 은 물리(카메라→포크 끝)라 유지.
- 리스크(worst): 태그 컷(3.3 m) 전에 `lateral_valid` 가 한 번도 True 가 안 되면 NEAR 진입 불가 = 정렬 불가 → 임시로 거리 진입(T+1 m)과 b-floor 게이트로 운용하되, 근본 해소는 대문제 4 하드웨어.

### 쟁점 7: "불확실하면 기다림" deadlock 의 timeout·fallback·수동전환
- 제안: 카운트 상수(`HOLD_MAX_CONSEC 8`, `MAX_STEPS 30`, `AIM_MAX_BACKUPS 2`, `SEARCH_MAX_ROUNDS 3`) → **진행도 예산 + 시간 예산 1개**. 진행도 = VERIFY 마다 J 가 σ_J 이상 감소했는가. 3단: **Tier 0 WAIT** — 판정불가 플래그 시 σ_eff 가 floor 에 닿을 때까지(KF P 수렴, 캡 3 s 코드 내부); **Tier 1 REPOSITION** — 기다려도 안 좋아지는 기하(원거리 lateral, 정면 heading)는 **관측성이 좋아지는 가이드 프리미티브**로 대체: 원거리는 대각 다리 진행(σ ∝ d² 개선), 정면은 자이로 앵커(oblique 다리에서 얻은 것, 드리프트 1°/분 → HEAD_TOL/2 까지 60 s 한도); **Tier 2 RETRY** — FINAL VERIFY 실패·반전가드 2회 → T 까지 후진 후 NEAR 재실행 1회; **Tier 3 ABORT→수동** — 재시도 실패, gyro stale 중 회전, 무응답(plan 2-3), 하드 캡, 재획득 실패, 추정기 재초기화 ≥2(flip lock-in 의심), 캘리브 파일 부재/불일치(plan 2-6), 시작 포락선 위반(쟁점 4), **총시간 > `DOCK_TIME_BUDGET_S`**(운영자가 정하는 값 → 유일한 신설 config).
- 근거: critic §19·Failure 6·Failure 1, autodock(실패 시 0.5 m 후퇴 재시도), Nav2(후퇴 3회), Poreski(정지 과다 실패), plan 1-3 AMBIGUOUS 타임아웃 이관. 9/7 181437: step 15 후진 1회로 재접근 성공(16~21) → 재시도 예산 1~2 가 현실적; 185709 는 sidestep 왕복으로 소진.
- 예상 효과: WAIT 무한대기 불가능(모든 Tier 에 캡), 실패 방향이 "정지·수동" 으로 안전측. 예측: 고장주입(AMBIGUOUS 10 s 강제, lateral_valid False 강제, 3.3 m 컷) 각각 캡 안에 Tier 도달·반전 0. 검증: 시뮬 고장주입 3종 + 실차 fallback 3종 의도 실행 체크리스트.
- 상수: 새 config 1(`DOCK_TIME_BUDGET_S`) / 실시간 2(진행도, WAIT 종료) / 코드 내부(캡 3 s·재시도 1·재초기화 2) / 삭제 `HOLD_MAX_CONSEC`, `MAX_STEPS`, `AIM_MAX_BACKUPS`, `SEARCH_MAX_ROUNDS`, `SEARCH_AFTER_MISSES`(태그 실종 타임아웃은 코드 내부).
- 리스크(worst): σ_J 과소평가 → 정상 수렴을 "정체" 로 오판해 수동전환 과다(가용성 저하, 안전측). abort rate 를 critic §23 KPI 로 추적.

### 쟁점 간 의존관계
- 1(전이표)이 골격: 5(게이트)는 DECIDE/VERIFY 안에서만, 6(구역)은 VERIFY 에서만 평가, 7(fallback)은 VERIFY 의 탈출 집합 — 넷이 한 모듈.
- 2(A 소거 배치) + 4(가시성 하드 제약)가 **허용 프리미티브 집합**을 정하고 3(가이드+종단비용)이 그 안에서 고른다; 3 은 plan 2 캘리브(τ_eff·σ_θ·S(T)·최소증분)와 plan 1 플래그(β_px·heading_valid·lateral_valid)를 입력으로 받는다.
- 상수 회계(config/control.py): 삭제 후보 22(`CAM_TO_PIVOT_M, TAG_CUT_MARGIN_PX, AIM_MAX_BEARING_DEG, AIM_MAX_TAG_OFF_DEG, AIM_TOL_DEG, AIM_FINAL_TOL_DEG, AIM_NEAR_T_M, AIM_AT_T_M, AIM_CHUNK_MAX_M, AIM_FINAL_MAX_LAT_M, AIM_BACKUP_M, AIM_MAX_BACKUPS, AIM_ABORT_WINDOW, AIM_DRIFT_ABORT_DEG, AIM_STANDOFF_M, SIDESTEP_BACKWARD_GAIN_DEG, SEARCH_BACKUP_M, SEARCH_AFTER_MISSES, SEARCH_MAX_ROUNDS, HOLD_MAX_CONSEC, MAX_STEPS, STEP_M, FWD_SAFETY` 중 미사용 2 포함), 신설 1(`DOCK_TIME_BUDGET_S`), 유지 `LAT_TOL_M`·`HEAD_TOL_DEG`(탑재부 사양)·`DOCK_EXTRA_M`(물리)·plan 2 `FORWARD_SLOW` → 순감 ≈21. 미해결 전제: T_f(forward 허용치)와 시작 포락선 규정은 대문제 4 몫.