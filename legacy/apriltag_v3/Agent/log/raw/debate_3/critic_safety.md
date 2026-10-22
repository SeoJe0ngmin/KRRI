근거 파일을 모두 읽고 9/7 원본 jsonl 로 proposer 인용을 재계산했다(181437 step 5~15·반전쌍·abort px·185709 `stable:false` 전부 일치, `STEP_M`/`FWD_SAFETY` v3 src 참조 0 일치). 아래는 쟁점별 판정이다.

### 쟁점 1: Controller state machine
- 판정: 부분동의
- 반박 근거: 전이표·명령 래치·stop 전용은 critic §14·§25·autodock A11 그대로라 옳다. 그러나 근거로 든 181437 step 8→9→10 반전은 **정지 후** 반전이다 — `rotation.jsonl` #7(ts …550.24) → `decision` #9(ts …551.46): settle(|rate|<문턱, rot_control.py:128-134) 뒤 30프레임(1.2 s) 잰 다음 반전했다. STOPPING/SETTLE 은 이걸 못 막고 v2 도 이미 settle+30프레임을 했으므로 "정지 후 2 s 내 반전 0건" 은 v2 에서도 참인 공허한 KPI. OBSERVE 탈출을 "σ_eff 수렴" 으로 두면 8→10 처럼 stderr 2.7 mm 인 채 값이 104 mm 튀는 매끈한 바이어스가 즉시 통과(critic Failure 2). 빠진 중단 술어: 프레임 stale(plan 1-6, 큐 16 이면 0.5 s 묵은 프레임)·블라인드 직진 중 gyro_alive=False·하드캡 emergency(plan 2-3) 가 "stop 외 금지" 전이표의 예외로 없음; REACQUIRE 가 DECIDE 밖에서 회전을 내면 래치 규칙 자체 위반.
- 대안: 전이표 채택 + (a) OBSERVE 탈출 = 하드 플래그 ∧ 최소 프레임(코드 내부) ∧ **직전 프리미티브 사후 일관성 통과**(쟁점 5), σ 수렴은 연장 조건만; (b) EXECUTE 중단 술어에 stale>150 ms·gyro dead(블라인드)·emergency 추가, CAN 쓰기는 세 발화자(카메라 루프·IMU 스레드·TX 데드맨)만; (c) REACQUIRE 는 KF 예측 β 방향 ≤반화각 1회 회전을 DECIDE 경유로만 — `next_set3_step` 의 360° 연속 탐색은 폐기(도킹 베이 후미 스윙).
- 대안 근거: `control_from_pose.py:611-622` `_hold` 는 시간 대기만이라 정지 확인 자체는 필요하나, 반전 원인은 측정(쟁점 5)이다; plan 1-6 stale·plan 2-2 데드맨·plan 2-3 emergency 는 확정 결론이므로 상태기계가 받아야 한다; 태그는 순간이동하지 않으므로 추정기가 있는 도킹에서 360° 탐색은 "추정기 사망" 신호 = ABORT 사유.
- 실차 검증법: 반전 카운트를 "VERIFY 종료 시각 전/후" 로 분리 기록 — SETTLE 중 반전은 구조상 0, 진짜 KPI 는 VERIFY 후 반전. 검출 스레드에 300 ms 인위 부하 주입 → EXECUTE 가 stale 술어로 stop 하는지.

### 쟁점 2: 회전중심(pivot) decouple — A 소거 배치
- 판정: 반박(부분)
- 반박 근거: (i) β_cam=0 ⇒ β_piv=0 은 **태그 자체**를 겨냥할 때만 A 무관이다. 쟁점 3 첫 프리미티브 `rotate-to-β_J` 는 축 위 J 를 겨냥하는데 tan β_piv = d·sinβ/(d·cosβ+A) 에 A 가 남는다: d_J 3 m·β 20° 에서 A=±0.4 → 17.7°/23.0° (Δ≈2.5° > HEAD_TOL) → 3 m 다리 끝 lateral ≈13 cm. "A 미확정이어도 결정이 안 틀림" 은 과장. (ii) ψ≈0 배치는 A 항은 죽이지만 heading 앵커 누설은 최대(lateral = d·sin(β−ψ_anchor), 3.5 m×0.5° = 30 mm = 예산 전부) — 트레이드오프 미명시. CLAUDE.md 상수 최소화 원칙은 "회전팔 A 는 실시간 도출" 을 명시 예시로 드는데 C 안은 회피만 한다.
- 대안: C+D — 소거 배치 (i)(ii)(iii) 는 "A-민감도 최소 배치 규칙" 으로 유지, **A 는 매 회전에서 실시간 도출**(β_px 는 R 무관·픽셀 정밀: Δβ = s·(1+A·cosβ/d)·Δψ_IMU, plan 1-1 식; s·A 분리는 precheck 두 거리 회전이 1차, 런 중 회전은 갱신) → σ_A 를 KF P 에, **J 겨냥 게이트 = σ_A·sinβ/d ≤ HEAD_TOL/k**(미달이면 태그 직접 겨냥으로 d 를 줄인 뒤 재평가). x_off 는 줄자 1회(물리 상수).
- 대안 근거: CLAUDE.md 원칙 명문; plan 1-1 검증식 그대로(8 m·10° 회전, β_px σ 0.05° → s 를 알 때 A 분해능 ≈0.05 m); debate_1 judge(|A|≲0.4 미확정). "T 근처 회전 ≤1회" 예측은 A 확정 전 근거 없음 — 9/7 step 5~10 6연속 회전의 원인은 정면 바이어스 + 매 정지 카메라 재앵커(쟁점 5).
- 실차 검증법: plan 1-1 회전×5(3.5/6 m, tilt≥20°) 로 A·s 확정 → 같은 날 J 겨냥 다리 3 m ×3 을 A=0 가정 / 추정 A 로 각각 실행, J 도착 lateral 차가 기하 예측(≈A·sinβ) 과 일치하는지.

### 쟁점 3: candidate/cost — 가이드 + 종단비용
- 판정: 부분동의
- 반박 근거: 가이드 순서·직접겨냥 허용식(3.5 m→52 mm, `AIM_FINAL_MAX_LAT_M 0.10` 위반 지적)·`STEP_M`·`FWD_SAFETY` 미사용은 옳다. 반박 셋: (a) 가이드가 순서를 정하므로 실제 결정은 "재조준 vs 계속"·"직접겨냥 vs V-다리" 두 부등식뿐 — 5후보 롤아웃·J 최소화는 dynamics_calib(τ_eff·σ_θ·S(T)) 전부에 의존하는 기계를 얹는 과설계(CLAUDE.md "필요 증명 후"); Yasunobu C11 은 연속 노치 선택 문제라 3-프리미티브 기하에 유추가 약하다. (b) 예측 "≤8 vs 21" 은 비교 불가 — 9/7 시작(ℓ 4.58/d 8.15) 은 쟁점 4 포락선 밖이라 새 설계는 **출발 거부**한다. (c) `backward` 를 67 로 허용하면 후방 무관측 + 코스팅 14 cm(9/7 step 15: 1.5 m 를 6.8 s 풀속 후진, `drive.jsonl` #3).
- 대안: 가이드 + 부등식 게이트(autodock A11 predock→parallel_correction→steer→last_mile 형), 롤아웃은 **가시성 하드제약·정지점 예측(plan 2-1) 의 feasibility 검사로만**, 비용함수·σ_J 삭제; α* 는 선택하지 않고 기하로 `α* = atan(ℓ/(d−x_J))`, x_J = T(가장 늦은 합류 = 최소각·태그 최중앙); backward 는 97 전용·거리 ≤ 직전 KF 전진 경로길이(알려진 빈 바닥).
- 대안 근거: A10 Poreski·A11 autodock 은 고정 순서+문턱으로 성공; critic §18 의 1-step 한계는 가이드가 미래 이득을 내장해 해소되므로 N-step 불필요; plan 2-5 블라인드 안전속도 논리(충돌 에너지 ¼)는 후방 무관측 후진에 동일 적용.
- 실차 검증법: 포락선 안 동일 시작자세(ℓ≈1.5 m·d≈7 m) ×5 에서 "가이드+게이트" 의 프리미티브 수·T 도착 lateral 분포 — 8 이하·반전 0 이면 비용함수 도입 근거 없음으로 확정.

### 쟁점 4: 태그 가시성 하드 제약 + 소각 규칙
- 판정: 부분동의
- 반박 근거: 하드 제약 > cost 항, `AIM_MAX_BEARING_DEG`·`AIM_MAX_TAG_OFF_DEG` 부등식 1개 대체는 동의. 반박 셋: (a) α*∈[θ_min 15°, β_vis] 하한은 "태그 화면 중앙·소각" 전제와 충돌(J 도착 시 태그 오프 = α* ≥15°) 이고 불필요 — 대각 다리에 필요한 건 β_px 와 gyro-앵커 ψ 이지 카메라 heading 이 아니다(앵커는 다리 전 tilt≥15° 순간 1회 + plan 1-7 정지마다 b_g 재추정); α* 는 기하(ℓ, d−x_J) 가 정한다. (b) β_vis 해석식(≈35°−각반경−ω̂τ_r) 은 검출 한계가 아니다 — 가장자리 왜곡·`MIN_DECISION_MARGIN`·회전 블러로 더 안쪽에서 끊기며, `detection_tag.py:25-37` `tag_edge_margin_px` docstring 자체가 "화면을 직접 보는 쪽이 무조건 맞다". (c) 제동 피치 미고려: 노즈다운 1° → 태그 상단 +≈24 px(fy≈1.4k) → STOPPING 중 컷 가능(9/7 abort 56/58 px 는 주행 중). 포락선: 9/7 시작은 ℓ/(d−x_J)=0.98 > tan β_vis≈0.62 이고 태그 겨냥으로 접근해도 ℓ/d 불변·ℓ/(d−x_J) 증가라 **가시 상태 J 합류 불가** → dog-leg 필수지만 4.6 m 횡이동은 블라인드 ≥20 s 로 "짧은 다리" 아님.
- 대안: β_vis·x_cut 은 사전값, 판정은 **관측 margin_px − 코스팅·stale 픽셀 환산**(v̂·(τ_eff + t_now−t_capture)·∂margin/∂x) 실시간; 시작 포락선 = 운영자 스테이징 요건(|ℓ| ≤ tanβ_vis·(d−x_J), 위반 시 출발 거부·사람이 재배치); dog-leg 는 블라인드 ≤10 s(≈2 m 횡) 1회만; 제동 피치는 precheck (2) 67 정지 n=20 의 태그 상단 픽셀 궤적으로 여유 px 도출.
- 대안 근거: Nav2 A12 staging pose·Walter A1(7.5 m/횡 3 m 시작 실패) = 포락선 선례; Richter A9 정면 회피는 heading 이 쓰이는 구간에만 필요; plan 1-6 stale 0.5 s 는 3.3 m 에서 ≈22 px.
- 실차 검증법: 3.5/6 m 정지 상태 저속 회전으로 검출이 끊기는 β 를 좌우 각 3회 기록 → 해석식과 차 >3° 면 관측값 채택; 67 정지 20건의 태그 상단 픽셀 최대 상승량 → 피치 여유 px.

### 쟁점 5: 행동 게이트
- 판정: 반박
- 반박 근거: 4항 AND 를 181437 step 8→9 에 적용하면 **전부 통과해 같은 반전을 낸다**: ① 30프레임 부호 일정(stderr lateral 2.7 mm·heading 0.04°), ② |e|=6.1° > 3+2·0.05, ③ 6° 회전 기대개선 ≫ σ_θ, ④ 오차 −6.7→+6.1 로 "0 을 지남" → 1회 허용; 9→10 은 "구역당 1회" 로만 막혀 FALLBACK — 유령을 인식 못 하고 abort 로 바꾼 것. 실제 원인: tilt 23.6→28.0°(oblique, θ_min 통과) 에서 IMU Δψ −6.74° vs 카메라 Δψ −4.53°(차 2.2° = plan 1-5 카메라–IMU 차 0.3~2.3°) → lateral 0.423→0.319(−104 mm; 제자리 회전이라 |A|≤0.4 로 최대 47 mm 만 가능) → T 방위각 8.3° 이동. σ̂ 는 이 바이어스를 못 보고(점프/stderr = 38×), 지속성 ①은 프레임 잡음용이라 정지 바이어스엔 무력. SETTLE 의 "oblique 면 ψ 재앵커" 가 매 정지 새 바이어스 표본 b(tilt_new) 를 주입하는 것이 반전 기구다. 데드밴드 max(HEAD_TOL, θ_min,inc, kσ_θ) 는 9/7 σ_θ 1.27° 로 ≈2.5° → 블라인드 1.28 m 누설 56 mm > LAT_TOL: σ_θ≤0.5°(plan 2-4) 달성 전엔 성립 불가.
- 대안: 게이트 ⓪ **프리미티브 사후 일관성**(cross-sensor arbiter) — 회전 후 카메라 Δψ·Δlateral 을 IMU 기반 예측(Δψ_cam=Δψ_IMU, |Δlat|≤A_max·sinΔψ) 과 **σ_random(b 미포함)** 으로 대조, 초과면 재앵커·반전 금지 → ψ̂ 는 자이로 증분 전파, lateral = d·sin(β_px−ψ̂) 재계산 후 판정; 카메라 재앵커는 innovation < k·σ_random(동의할 때) 또는 앵커 경과 >60 s 강제 다수결만. 재계산 예: step 9 를 ψ̂=−30.22° 로 풀면 lateral 0.465 m·T 방위각 −30.8° → 회전 −0.6°(데드밴드 안) → 반전 소멸, 바로 aim_drive.
- 대안 근거: plan 1-5 교차센서 일관성이 이미 1차 방어선; critic Failure 2·§21(같은 오차 이중해석 금지); Liu 2021 B8(KF 예측으로 해 선택)의 heading 판; Poreski 스윕은 잡음 아래 문턱 문제라 바이어스엔 무관.
- 실차 검증법: 3.5 m·tilt 25° 정지에서 IMU 회전 ±5° ×10 → Δψ_cam/Δψ_IMU·Δlateral 분포; |Δψ_cam−Δψ_IMU| > k·σ_random 비율 >10% 면 "정지마다 재앵커" 금지 확정, 자이로 전파 lateral 의 회전 전후 차 ≤ A·sinΔψ+2σ 인지.

### 쟁점 6: Zone 전환 hysteresis 와 zone 별 목표
- 판정: 부분동의
- 반박 근거: 판정가능성·이벤트 구역, VERIFY 에서만 전환, FINAL 진입 이벤트는 동의(Wang A14·autodock last_mile·critic §20). 반박: (a) lateral_valid = σ_lat,eff ≤ 15 mm 인데 σ_floor(d,tilt) 는 정적 격자 전엔 없다 → c·σ̂ 만으론 181437 step 1(8.15 m, stderr 2.9 mm) 에서 **8 m 에 NEAR 진입** = v1 유령 추종 부활; "3.5 m 이하·oblique 로 제한" 은 거리 상수의 위장. (b) FINAL 에서 gyro_alive=False 는 블라인드+heading 무관측인데 Tier 3 엔 "gyro stale 중 회전" 만 있다. (c) hysteresis "τ_eff·v̂+2σ_x" 는 정지 평가라 v̂≈0 → 실질 2σ_x; 의도(다음 다리 코스팅) 라면 계획 속도로 명시. (d) "T = x_cut+코스팅+여유" 의 여유는 새 상수.
- 대안: σ_floor·θ_min(d)·b(d,tilt)·A·s·x_off·δ·h·β_vis 를 **perception_calib.json** 으로 묶어 dynamics_calib 과 같은 "부재/불일치 → 출발 거부"; NEAR 진입 = calib 기반 lateral_valid 만(격자 전엔 도킹 안 함, 임시 거리 규칙 없음); FINAL 중단 술어에 gyro dead → stop·ABORT; 여유 = q95⁺(calib) + 제동피치 px 파생.
- 대안 근거: CLAUDE.md "config 에는 사람이 정하는 값만" — σ_floor 는 측정값; plan 2-6 출발 거부 규칙과 대칭; plan 1 리스크의 "격자 전 3.5 m·oblique 제한" 을 상수 대신 캘리브 부재로 표현.
- 실차 검증법: 정적 격자 후 5 m·tilt 20° 정지 lateral 10회 std — ≤15 mm 면 NEAR 5 m, 아니면 3.5 m 로 표가 정한다(사람 상수 0); 경계 ±2σ_x 잡음 주입 시뮬로 FAR↔NEAR 왕복 0.

### 쟁점 7: 기다림 deadlock 의 timeout·fallback·수동전환
- 판정: 부분동의
- 반박 근거: 진행도+시간 예산·유일 config `DOCK_TIME_BUDGET_S`·Tier 캡 전수는 동의(critic §19·Failure 6·Nav2/autodock 후퇴 재시도). 반박: (a) **Tier 2 트리거 "FINAL VERIFY 실패" 는 관측 불가** — 태그 컷 3.3 m→정지 2.02 m 라 최종 pose 를 카메라가 못 본다; 181437 "성공" 도 `measure.jsonl` 21건이 전부 마지막 5.97 s 직진 *이전* 값이라 최종 오차 기록이 0 이다 → VERIFY 3조건·RETRY 를 그 위에 세우면 "눈 감고 DONE". (b) Tier 1 "정면은 자이로 앵커 60 s" 는 "자이로 짧은 다리(몇 초)" 전제 위반이자 새 시간상수. (c) ABORT 의 물리 동작 미정 — 버스 침묵이면 차량 워치독 미확인(plan 2-7 (1)), 마지막 forward 유지 위험. (d) 진행도를 σ_J 로 재면 쟁점 3 비용함수에 종속.
- 대안: FINAL 은 **DONE-unverified(DR)** 로 선언·로그 표기(대문제 4 해소 전, 검증은 외부 줄자/운영자); Tier 2 트리거를 관측 가능한 것으로 교체 — ① T-VERIFY 허용식 실패(V-다리 1회 후) ② NEAR 반전가드 2회 ③ **블라인드 직진 중 자이로 heading 표류 > `fwd_abort_deg`**(`control_from_pose.py:75-77`, 1.28 m → asin(0.03/1.28)=1.34°) → stop → T 까지 97 후진(KF 경로길이) → 재VERIFY 1회; 진행도 = 남은거리 또는 정렬오차 감소 > kσ(비용함수 무관); ABORT = stop 프레임+heartbeat 지속 송신 + 운영자 알림(침묵 금지); Tier 1 자이로 한도는 b_g 재추정 후 실측 랜덤워크로 도출(σ_ψ(t) ≤ HEAD_TOL/2).
- 대안 근거: 블라인드에서 유일하게 관측 가능한 실패 신호가 자이로 heading 이고 그 문턱은 기존 함수가 준다; debate_2 top risk(블라인드 1.28 m)·IEC 62745(유효신호 부재 ≤0.5 s 정지)·plan 2-7 (1) 워치독 미관측.
- 실차 검증법: 고장주입 3종(AMBIGUOUS 10 s·lateral_valid False·자이로 표류 2° 주입) 각각 캡 안 Tier 도달 + CAN 로그에 stop 프레임 연속 확인; 최종 정지 ×10 외부 줄자(plan 2-7 (6)) — σ > T_f/3 이면 DONE-unverified 표기 유지.

### 가장 위험한 제안 TOP2
1. 쟁점 5(+쟁점 2 의 정지마다 재앵커): σ̂ 를 믿는 4항 게이트는 181437 8→9 반전(IMU −6.74° vs 카메라 −4.53°, lateral −104 mm, stderr 2.7 mm)을 그대로 재생하고 2회째를 abort 로 바꿀 뿐 — 자이로 중재 일관성 검사와 단일 앵커 없이는 v2 반전이 v3 에서 살아남는다.
2. 쟁점 7/6 FINAL: 마지막 1.28 m 가 블라인드인데 VERIFY 통과·RETRY 트리거·DONE 을 그 위에 세웠다 — 실차에서는 검증 없는 DONE 과 후방 무관측 67 후진이 나온다; DONE-unverified + 자이로 표류 중단 + 97 후진으로 바꿔야 한다.