# 대문제 2 — Vehicle Dynamics 토론 기록 (2026-09-20)

- 실행: `debate_workflow.js` n=2, run `wf_1722cb0c-076`(상수 최소화 원칙·A 미확정 CONTEXT 반영본; 옛 규칙 첫 실행 `wf_71ce9eaa-e99` 는 judge 단계에서 중단, 원문은 `raw/debate_2_old_rule_aborted/` 참고용). Fable 5.1 effort max, 에이전트 9개(proposer / critic 안전·정밀 / 재반박 / judge / researcher ×3 / judge 최종), 폴백·오류 0, 83분, 1.80M 토큰.
- judge 1차 hold 3(쟁점 3·5·7) → researcher 문헌 조사 → 최종 결론 7 / 보류 0. verdict: compromise 6 / proposer 1(쟁점 6) / critic 0.
- 핵심 데이터: 9/7 `181437/{rotation,drive,measure,decision,abort}.jsonl`, `185709/rotation.jsonl`, `오차보정.txt`(can_pulse). **오케스트레이터 검산**(원본 재처리): 회전 소각 8건 −0.53°/σ1.27, 대각 7건 −0.79°/σ0.59·7/7 언더슛, 25.1° 3.9 s vs 23.2° 8.97 s, 185709 #3 goal 초과 +8.24°·gyro-gaps, 태그 컷 56/58 px, DOCK_EXTRA_M −2.02, 9/7 호스트 프롬프트 `seojeongmin@ubuntu`(VM) — 전부 일치. 코드 사실(movement_tx_loop 10 ms 반복 송신, `_can_tx_worker` 데드맨 없음, rot_control 루프가 alive 만 보고 gaps 는 사후, 삭제 후보 config 상수 10개 존재) grep 확인.

## 쟁점 1: 정지거리 모델 — P(d_stop|v) 최소 구현
- proposer: 셀 (67/97)×(fwd/bwd) 별 1-파라미터 비례 `D̂ = τ_eff·v̂` + 잔차 경험분포 {n, median, σ_e, q95⁺, q99⁺, max}; D_obs = KF x̂(CAN write 시각) − 정지 후 30프레임 중앙값; 발령 `x_rem ≤ D̂ + m`, m 은 짧게 서는 쪽 편향(부족은 97 크립 1회 복구) — 근거: critic §8·§23, Yaskawa C10, MERL C12, `AIM_STOP_LEAD_M=0.15` 는 한 속도 점값, can_pulse 코스팅 5표본 0.09~0.17 m 은 정의 혼재.
- critic(안전): 셀·비례·분포·D_obs 고정은 동의. **크립 복구는 자기모순** — 97 최소 신뢰 증분 ≈ 램프 0.05 + 코스팅 0.066 ≈ 0.12 m > 복구할 3 cm, <2 s 명령 CV 100%(CLAUDE.md) → Failure 4·ping-pong. x_rem 좌표계 미지정(pivot 프레임이면 A ±0.5 m 오차가 2 cm 모델을 삼킴). 최종 정지는 블라인드라 x_final 을 카메라로 못 잼.
- critic(정밀): **캘리브 영역 ≠ 사용 영역** — 태그 컷 3.26~3.30 m(abort.jsonl)·DOCK_EXTRA_M −2.02 → 최종 정지는 1.28 m 블라인드 안, x_rem 은 추측항법 ±13 cm. n=20 으로 q99 불가(분포무관 P(new>max_n)=1/(n+1) → 95%). 한쪽 마진은 여유 0.5 m 앞에서 이득 없이 미도달만 만듦. D_obs 스칼라가 태그법선 forward 면 사각에서 cos ψ 만큼 13~24% 과소.
- proposer 재반박: **수정** — 크립 복구 조항 삭제, x_rem 은 카메라 forward(A 무관; KF 가 pivot 이어도 되돌리면 상쇄), D_obs 경로길이, m=0, q99 는 "미검증" 표기. 블라인드·단거리 다리용 **S(T) 분포표**(정지출발 T초→총이동, `fwd_time_model` 의 분포판) 신설, `FWD_T0/T1/A` "실행경로 제외" 철회(캘리브 파일 값으로 남김).
- judge: **compromise** — 결정적 근거: DOCK_EXTRA_M·abort.jsonl·decision #21 "마지막 1.26 m" 5.97 s 개루프(블라인드 확인), 97 최소 증분 0.12 m > 3 cm, can_pulse 코스팅 정의 오염(`can_pulse.py:232-234`, `:224` 3 cm 문턱), Wilks 1941/NIST 단측 95/95 비모수 n=59.
- 결론: 셀 ×2모드. ① 가시 정속정지 D̂=τ_eff·v̂, D_obs=경로길이, 표에 n·기대 커버리지 1−1/(n+1) 명시·q99 는 n≥59 전엔 미검증, 발령 x_rem(카메라 forward) ≤ D̂, m=0. ② S(T) 분포표를 블라인드 최종 다리·정속 최소길이 미만 다리 전용으로, 최소 신뢰 증분 = CV<30% 최소 T; σ_S 가 T_f 를 못 만족하면 "dynamics 로 못 닫음" → 대문제 4. 삭제 AIM_STOP_LEAD_M·AIM_STOP_CONFIRM.
  | 검증: 3.5 m 정면(tilt<10°)·사각(30°) 67 정속정지 각 n=10 — 경로길이 D_obs 의 τ_eff 가 두 기하에서 차 <5% ∧ σ_e ≤ 2 cm; 97 정지출발 {2,3,4,5,6 s}×10 → S(T) CV 곡선·σ_S.

## 쟁점 2: 명령 지연 분리 — start/stop/관성
- proposer: 미분 금지, 회귀 둘 — ① 두 속도 단 대수 `D = τ_d·v + v²/2a` ② 사건별 30 fps 궤적 적합(절점 해상도 5~10 ms) → τ_d 히스토그램 = σ_τ. τ_start = KF v̂ > k·σ_v 시각. 분리 불필요 = 정속에서 서는 모든 정지; 램프 중 정지는 최소 명령길이(≥ τ_start,q99 + t_ramp + 2 s)로 회피; 명령버퍼 prior 는 KF 상태입력이 아닌 **Q 스케줄 창**으로(plan 1-7 이관 수용). 회전 시작 0.85 s vs 코스팅 0.18 s → 시작≠정지 — 근거: Ghaffari C7, Devika C14, C15, Kalaria C9.
- critic(안전): ① 은 결정계이고 v² 항이 작아 a 상대오차 40~90%·음수 가능 → 무효. ② 절점 σ 는 10~30 ms 라 30~80 ms 지터와 겨우 분해. τ_start 카메라 교차는 KF 지연 0.1~0.3 s 편향 → "분포" 로 보고하면 허위 정밀. 발령식에 v̂·Δt_ctrl/2 복원.
- critic(정밀): 절점은 기울기 꺾임이라 a≈2 면 제동 4프레임 → a 미식별·τ_d ±33 ms. **"꼬리 분포" 표본이 전부 호스트 정지 의심** — 8.23 s→0.66 m, #14 8.97 s, 185709 #3 goal 10.7° 초과(10 ms 폴 루프가 ≥1.3 s 안 돎), 프롬프트 `seojeongmin@ubuntu` = VM → τ_start,q99 로 모델링하면 버그의 물리화. 최소 명령길이 ≈5 s → 0.4~0.8 m 미만 다리 전부 크립 의존. `_can_tx_worker` 는 데드맨 없이는 더 위험(결정 루프 멎어도 마지막 forward 계속 송신).
- proposer 재반박: **수정** — ① 폐기, ② 격하(부트스트랩 σ 첨부 교차검증만), 시각은 200 Hz 가속도 t_resp 1차, a 는 사건별 대수 진단값, 호스트 이상 사건은 (loop tick·CAN write 간격·gyro gaps·프레임 도착 간격·CAN RX) 로 표에서 제외, TX 전용 스레드 + 데드맨 한 묶음(소유 대문제 4).
- (researcher 7: ros2_control cmd_vel_timeout 0.5 s·Nav2 1.0 s = 송신측 데드맨 표준; Kvaser TXACK 타임스탬프로 실제 버스 전송 시각.)
- judge: **compromise** — 두 단 대수 검산(D̄₁−2D̄₂ = 0.0144/a → 0.7~1.4 cm vs σ 0.63 cm) 재현, VM 호스트 확인, `_can_tx_worker` 데드맨 부재 코드 확인. 150 ms 데드맨은 근거 없음 → 관행 0.3~0.5 s 에서 시작·loop-tick 히스토그램으로 확정.
- 결론: 정속 정지는 셀 τ_eff·σ_e 만. 두 단 대수·카메라 절점 히스토그램은 추정기로 폐기, τ_d·σ_τ 는 가속도 t_resp 1차, τ_start 는 σ_a,still 실측 후 센서 결정(전엔 최소 명령길이·워치독 캡용 보수값 q50+여유). 9/7 꼬리는 VM 사건 → 모델링 금지, 모든 사건에 호스트 계측 부착·이상은 표 제외. 발령식 v̂·Δt_ctrl/2 복원. Q 스케줄 창 [t_tx, t_tx+τ_start+t_ramp]/[t_stop, t_stop+τ_d+v̂/a]. CAN TX 전용 스레드 + 송신측 데드맨(0.3~0.5 s 시작) 한 묶음, precheck 선행조건. 삭제 SETTLE_SEC(→코드 캡 이관)·FWD_SCALE·FWD_BIAS.
  | 검증: 67 정지 n=20 가속도 t_resp vs 카메라 절점 τ_d 산포도(±30 ms, 부트스트랩 σ) + 97 크립 중 CAN 송신 0.5/1/2/5 s 의도적 차단 → 차량 워치독 유무·시간·마지막 명령 유지 여부.

## 쟁점 3: 가속도(IMU) 의 역할 — 응답 감지·정지 판별 한정
- proposer: 가속도 = 명령 창 [t_tx, t_tx+τ_q99] 안에서만 보는 사건 시각 계측기 + 응답 워치독, D 입력 아님. 정지 판별 v̂+ω ≥0.3 s. 워치독: 출발 무응답 → 재전송 1회 → 정지·경고; 정지 무응답 → stop 재전송 + emergency 프레임. ROT_SETTLE_* 6개 → 잡음 도출 규칙 1개 — 근거: critic §9·§10, FWD_A 0.14 < 피치 1° 누설 0.17, 제동 a≈2 m/s², Nav2 stall C21, Hong&Park B18.
- critic(안전): 재전송 무의미(`movement_tx_loop` 10 ms 반복 송신). emergency 0x80 은 차량측 의미·래치·해제 미확인 + σ_a,still 미측정 → 매 정지 오발(도킹 불능) 또는 무효(허위 안전) — TOP1. 블라인드에서 "v̂+ω" 불성립(v̂ 는 측정 없는 예측). 중력축은 up 만 → 전진축 미정.
- critic(정밀): 같은 재전송·emergency 반박. 이벤트 SETTLE 은 KF v̂ 감쇠 0.2~0.3 s 라 SETTLE_SEC 0.8 보다 느릴 수 있음 → 캡은 SETTLE_SEC 의 코드 이동(삭제 아닌 이관). 정지 판별 1차 = 자이로 |ω|, v̂ 는 확인.
- proposer 재반박: **수정** — 재전송·자동 e-stop 삭제, emergency 는 하드 캡 위반에만·precheck 실측 후, 정지 판별 2모드(가시 v̂+ω / 블라인드 |a_h|+|ω|), 수평 성분 크기 |a_h| 사용, SETTLE_SEC 코드 캡 이관.
- researcher 3: 표준형 = 창 통계<γ ∧ 지속 + 히스테리시스(PX4 LandDetector 1 s·즉시 탈락, ArduPilot OGNM 진입 0.7×·**문턱 = 정지 로그 최대 × SF**); Skog 2010 자이로 단독 ARED ≈ SHOE; Wahlström&Skog 2021 "문턱은 속도 의존·미해결"; Li 2024·Hitachi Rail US11774573 — IMU 는 등속 크립과 정지를 구분 못 함 → 정지 확정은 외부 변위, IMU 는 Δa 전이; 지게차 진동 PSD 3~8 Hz(Huang 2021) → 창 ≥0.3 s 물리 하한; BMI085 바닥 σ_a 0.008·σ_ω 0.14°/s ≪ 차량 진동 → σ_a,still 실측 필수; emergency 0x80 문헌 0.
- judge: **compromise** — 결정적 근거: movement_tx_loop 코드, emergency 문헌 0, IMU 의 크립/정지 비구분(연구 3-7·3-10), ArduPilot 형 "정지 로그 최대 × SF" 가 실시간 도출 절차와 일치.
- 결론: 가속도 = 명령 창 안 사건 시각 계측기(t_resp, |a_h|) + 로그 플래그, D 입력·e-stop 트리거 아님. 재전송 삭제; emergency 는 하드 캡 위반(KF/DR 위치가 안전선 통과)에만, precheck 실측 후 채택. 정지 판별 2모드 — 가시: KF v̂ 창 안 전 샘플 문턱 아래 ∧ |ω|<γ_ω; 블라인드: t_resp 검출 후 |a_h−ā|<γ_a ∧ |ω|<γ_ω ≥0.3 s + 시간 캡(코드 내부), 전이 없이 τ_d,q95 경과면 "응답 부재" 경고·수동전환. **γ 는 config 가 아니라 매 정지 정지창 지표 최대 × SF(코드 내부 1개) 로 실시간 도출**, 진입 0.7×/이탈 즉시. ROT_SETTLE_* 6개 삭제, SETTLE_SEC 코드 캡 이관, plan 1-7 GYRO_STILL_DPS 도 같은 도출값으로 대체(상수 −1).
  | 검증: 정지 30 s(비적재/적재 × 마스트 하강/상승 × 유압 on/off)·97 크립 30 s 의 EMA|Δa|·std(|a|,0.3 s)·|ω| 분포 분리 여부 + 67 정지 n=20 t_resp 검출률·창 밖 오탐률.

## 쟁점 4: 회전 정지 예측 — h(ω)·좌/우·소각·ROT_LEAD_DEG 대체
- proposer: `goal=|deg|−min(ROT_LEAD_DEG,|deg|/2)` 폐기 → 실시간 규칙 `θ_rem ≤ ω̂·τ_r + ω̂²/2α_r + m_θ`, L/R 셀, 가속 중에도 같은 규칙, 잔차 = 자이로 Δψ−target 만("회전 잔차 ±1.5°" 이중해석 폐기), 루프 안 gaps 즉시 정지, 임시 τ_r 0.18·α_r 20 — 근거: 9/7 소각 코스팅 0.4~3.5°(9배 산포) vs 대각 0.9~2.3°·7/7 언더슛, 185709 +8.2° = gaps, `analyze_run.py:117` 권장식은 ω 무관.
- critic(안전): τ_r 0.18 과 α_r 20 은 **같은 n=1 점**(1.3°@7.3°/s)의 두 극단 역산 — 합치면 2.64°(실측 2배) → 첫 세트 ~1.3° 언더슛이 HEAD_TOL 안이라 통과 오인. 소각 코스팅에 조향 유압 복귀 혼입 가능. 캘리브는 **개루프 펄스 우선**(ω(t) 적합), 폐루프는 검증만.
- critic(정밀): σ_θ 0.5° 가설이 **판정 루프 지터**(8°/s×60 ms=0.5°, 185709 #3 은 1.3 s)를 뺐다 → IMU 콜백 스레드 평가. 가속 중 α_up≈11°/s² 로 ω 상승 → 소각 체계 언더 예측. 블라인드 1.28 m×sin δψ: 1°→2.2 cm, 2°→4.5 cm > LAT_TOL → **0.5° 는 요건**. CCW 1.98° vs CW 1.34°(n 4/3) 비대칭 힌트.
- proposer 재반박: **수정** — 임시값 순수지연 τ_r 0.18 만(α_r=∞); 개루프 펄스 L/R×{1,1.5,2,3 s}×4 + 5 s ×2 → τ_r·α_r·α_up 적합 → 폐루프 검증; ω_pred = ω̂ + [가속중]·α_up·τ_r; IMU 콜백 스레드에서 CAN 상태 직접 플립; 소각 3단; 편향 {10,15,20} 스윕 필수.
- judge: **compromise** — rotation.jsonl 재계산 일치, 185709 #3 로 판정 루프 상한 보장 없음 확인, rot_control.py:110-123 alive 만 검사 확인. 펄스 표 계층은 5° 폐루프에서 σ_θ>0.5° 일 때만; 저속 회전 단은 20 에서 실패 시만.
- 결론: 규칙 θ_rem ≤ ω_pred·τ_r,dir + [가속중]·½α_up·τ_r² + ω_pred²/(2α_r,dir), L/R 셀(통합은 실측 후), 임시 순수지연 τ_r=0.18 하나. 캘리브 = 개루프 펄스 → 적합 → 폐루프 L/R×{5,15,45°}×6 검증. 평가는 IMU 콜백 스레드(TX 스레드+데드맨과 묶음), 루프 안 gaps → 즉시 stop + β 재앵커, 무응답 워치독(경고), 시간 캡 ω̂ 기준 ×2. 소각 |Δθ|<max(HEAD_TOL, θ_min,inc) 안 돎. 잔차 = 자이로만; AIM_TOL_DEG·AIM_FINAL_TOL_DEG 는 대문제 3 이 σ_θ 로 재도출. 삭제 ROT_LEAD_DEG; ROT_POLL_SEC(소멸)·ROT_WATCHDOG_GAIN·ROT_WRONG_WAY_DEG 코드 내부; ROT_T0·ROT_DEG_PER_SEC 캘리브 파일; analyze_run.py:117 권장식 폐기.
  | 검증: 개루프 3 s 펄스 L/R×4 의 200 Hz ω(t) 로 (τ_r, α_r, α_up) 적합 → 같은 15° 회전을 (a) asyncio (b) IMU 스레드 판정 각 ×6 → σ_θ ≤0.5° ∧ |mean|<0.3°; plan 1-1 회전×5 와 같은 세션.

## 쟁점 5: 저속 크립(byte2=97) 활용
- proposer: 안 B = 67 원거리 / **97 은 정지출발 구간 전용**(T 뒤 마지막 직진). 주행 중 단 전환 1차 금지. 편향 스윕 {15,20,25,30,40,60}×4 s×3 + 97 펄스 {0.5,0.8,1.0,1.5 s}×5 → 최소 신뢰 증분. 새 config 1(FORWARD_SLOW 템플릿) — 근거: `control_광운대.py:78` "tune above dead-band", NISTIR 8140 C16, Nav2 A12, Klimenda C18; 블라인드 DR ±13 cm 는 속도 단 무관 → 저속 이득은 정지부 한정.
- critic(안전): B 동의하되 **이유 교체** — σ_D 절반은 ±13 cm 앞에서 논거가 못 되고 stiction 발견 시 A 복귀 빌미; 진짜 근거는 **안전**(블라인드 1.28 m 를 0.24→0.12 m/s: 충돌 에너지 ¼·감시자 개입 시간 2배). 검증: 97 정지출발 4 s×10 온셋 분포, 2 s 초과·무이동 1건이면 보류.
- critic(정밀): Klimenda C18 50 mm/s→80±5 mm(순수지연이면 25) = 저속 절편 c₀ 선례 → 2셀 비례로는 c₀ 검출 불가, ≥3 레벨로 D=c₀+τ_d·v 적합. 펄스 0.5/0.8 s < τ_start 는 표본 낭비. 광운대 데드밴드는 주석 1줄(값·차량·배터리 없음) → 가설.
- proposer 재반박: **수정** — B = "블라인드 구간 안전속도 규칙", 스윕 순서 ① stiction ② 편향 ≥3 레벨 c₀ 검정 ③ 펄스 {1,1.5,2,2.5 s}, T 4.0 m 셀 포함.
- researcher 5: 광운대 97 튠 기록 = 주석 1줄뿐(리포 전체 grep) → 직접 요청 항목. 트랙션 컨트롤러 3사(Curtis Deadband/Offset/Map·Speed vs Torque Mode, Zapi FREQUENCY CREEP 0.6~4 Hz, Sevcon) 모두 데드밴드+탈출 점프+곡률 프로그래머블; Volvo CE US9656573 저속 평탄화; Toyota US11454965 원격모드 속도 캡; ISO 3691-4 도킹 ≤0.3 m/s ≈ 우리 풀 0.284 → **67 자체가 캡일 개연성**; Souza&Azzolin 2020 데드존은 정지→출발에서 두드러짐.
- judge: **compromise** — 결정적 근거: 튠 기록 부재, 3사 문서(비례 가정 금지·포화 확인), C18 c₀, 블라인드 DR 속도 무관.
- 결론: byte2 는 **이산 2단 {67, 97} 액추에이터**. B = 블라인드 구간 안전속도 규칙, stiction 검사 조건부. 스윕 ① 97 정지출발 4 s×10 온셋(2 s 초과·무이동 1건이면 B 보류→A+마진) ② 편향 {10,15,20,25,30,45,60}×4 s×3 오름/내림 교대 → 무이동 문턱·탈출 점프·포화(v45=v60?)·v_ss·CV·램프·D(v), ≥3 레벨 D=c₀+τ_d·v 적합으로 비례 검정 ③ 97 펄스 {1,1.5,2,2.5 s}×5 → CV 곡선·최소 신뢰 증분(S(T) 표와 같은 데이터) ④ 67→97 주행 중 전환 ×5 측정만. 적재/비적재 각 1세트, 배터리 전압·SOC 기록, T 4.0 m 셀. 새 config 1: FORWARD_SLOW.
  | 검증: 스윕 ①+② 한 세션 — 온셋 분포 + 편향 7레벨 v_ss·D 표 → 절편·점프·포화 판정, c₀ 95% CI 0 포함 여부.

## 쟁점 6: 온라인 파라미터 적응 — 고정 vs 적응
- proposer: **고정(캘리브 파일) + 매 정지 잔차 로깅 + 드리프트 경보(최근 5건 |mean e|>σ_e → 마진×1.5, 2세션 연속 → 재캘리브) + 명시적 재캘리브**. 세션 시작 시 누적 로그로 분위수 재계산. 적재/비적재 캘리브 2세트(임무 입력). v̂ 이중미분 금지. EMA 는 도킹 1회 = 정지 2~3건이라 지터 추종 = 유령 적응 — 근거: CONTEXT 규칙, Yaskawa C10, Yasunobu C11·KRRI C13(적응은 하루 수천 정차 전제), critic §1.
- critic(안전): 동의. "누적 재계산" 은 조건 다른 세션 혼합 = 은근한 적응; "마진×1.5 자동" 도 자동 갱신; 캘리브 부재 시 코드 기본값이면 삭제 상수 부활 → 출발 거부.
- critic(정밀): 동의 + τ_eff 는 KF 지연을 흡수하므로 **캘리브와 배포의 추정기 설정(Q·τ_cam)이 같아야** — 설정 해시를 캘리브 파일에 넣고 불일치 시 로드 거부. 경보 입력은 가시·정속·비스톨 정지만.
- proposer 재반박: **수정** — 전부 수용(분위수는 셀·적재상태별 최근 N 세션 창, 자동 마진 변경 폐기, 캘리브 부재 시 출발 거부, KF 설정 해시 짝).
- judge: **proposer** — 결정적 근거: CONTEXT 규칙·CLAUDE.md 저속 v̂ 이중미분 리스크·C10·critic 3자 일치; 운영 규칙만 critic 수정 반영.
- 결론: 고정 캘리브 + 잔차 로깅 + 경보(사람 알림 + "재캘리브 전 도킹 금지" 플래그, 자동 마진 변경 없음) + 명시적 재캘리브. 적재/비적재 2세트 임무 입력. 분위수는 (셀, 적재상태)별 최근 N 세션 창(N 코드 내부), 입력은 가시·정속·비스톨만. 캘리브 파일 부재 → 출발 거부(dry-run 만). `dynamics_calib.json` 에 KF Q·τ_cam 해시·호스트·git·n·기대 커버리지, 불일치 시 로드 거부. 새 config 0.
  | 검증: 세션 시작·끝 67 n=10 |Δmean e| < σ_e; 적재 n=10; Q_v 2배 변경 후 τ_eff 이동량 ≠0 이면 짝 규칙 확정.

## 쟁점 7: 사전실험 설계 — precheck 2종
- proposer: 원시 기록만, 판정 오프라인. 선행조건 assert(plan 1-6 타이밍, 자이로 먼저, snapshot_config+캘리브 해시). CAN 실제 write 시각 훅(`control_광운대.py:200-250`) 이식. 셀·n: 직진 67/97 × {4,6 s}×10, bwd, bwd→fwd 셀 6, 편향 스윕, 펄스, 회전 L/R×{5,15,45°}×6 + 개루프 펄스, 세션 시작·끝 n=10. 산출 `dynamics_calib.json`. 통과 67 σ_e≤2 cm, L/R σ_θ≤0.5°. 하루 예산.
- critic(안전): 9/7 두 느린 사건은 bwd→rot(+4.6°)→fwd 순서라 sequence 셀로 안 잡힘 → CAN RX 전량·TX 간격 최대치 필요. 블라인드 최종 정지 정확도 항목 부재(도킹 KPI 본체 미검증) → 외부 줄자 ×10. 광운대 훅은 monotonic_ns vs plan 1-6 epoch → 변환 필요. 회전 0.5 s 펄스 < 시작지연 0.85 s.
- critic(정밀): n=20 → 95% 만, q99 통과 기준 금지. 호스트 정지 계측(loop tick·CAN write 간격·gyro gaps·프레임 도착 간격) 누락. 차량 자체 워치독·emergency 시험 누락(안전 최우선). 시간 예산 과소(직진 80건 = 복귀 포함 2 h+). VM 실차 금지. 진입 게이트: 첫 10건 CAN write 간격 99%<20 ms ∧ gaps 0.
- proposer 재반박: **수정** — 순서 (1) 안전(CAN 차단·emergency·데드맨) (2) 67 n=20 정면+사각 (3) 97 스윕 (4) 회전 (5) 정속 장거리 8→4 m (6) S(T)+최종 정지 줄자 ×10 (7) 시작·끝; 기록 항목 추가(RX 전량·직전 명령 종류·호스트·KF 해시·epoch−monotonic 오프셋); 호스트 Jetson/Windows 만.
- researcher 7: 차량 프로토콜 문서 0(`control_forklift_v2.py:35` "250ms" 는 전언, RX 경로 없음 → 차량 반응 관측 이력 0). IEC 62745 유효신호 부재 ≤0.5 s 자동정지·재확립 시 재시동 금지; ISO 13850 e-stop 래치+별도 재시동; Curtis Fault 72 PDO Timeout(0 이면 마지막 명령 무한 유지)·Sevcon RPDO 500 ms → 값은 통합업체 설정, 실측 외 방법 없음. ID 가 CANopen 형(0x764=HB 노드 0x64, 0x1E3/0x2E3=TPDO1/2) → RX 스니핑·SDO 0x1016/0x1017 읽기 저비용 probe(가설). Kvaser TXACK 로 실제 전송 시각. Wilks 1941 단측 95/95 비모수 n=59.
- judge: **compromise** — 결정적 근거: 프로토콜 문서 부재·RX 경로 없음, 규범(≤0.5 s·재시동 금지), Curtis/Sevcon 설정 의존, TXACK, Wilks n=59, 9/7 사건 순서 확인.
- 결론: 원시 기록·오프라인 판정·assert 선행조건·호스트 Jetson/Windows 만. 순서 **(1)** 정지→97 크립·사람 제동 위치에서 heartbeat만/movement만/control만/전부 차단 각 ≥3회 + 카운터 고정 재생 → 차량 워치독 T_f 또는 "유지", 끊김 vs stop 프레임 감속 비교, 복구(재개/버스트/키 = 래치), emergency 0x80 정지 상태 1회, 데드맨 동작, Kvaser RX 스니핑(+TXACK, SDO 시도) **(2)** 67 n=20 @3.5 m 정면+사각 **(3)** 97 stiction→스윕→펄스 **(4)** 회전 개루프→폐루프 + asyncio/IMU 스레드 A/B(plan 1-1 통합) — 1일차; **(5)** 정속 8→4 m ×3×{67,97} → σ_v/v **(6)** S(T) {2~6 s}×{67,97}×≥10 + 최종 정지 ×10 외부 줄자 **(7)** 시작·끝 n=10 — 2일차. 기록: t_cmd=TXACK(+write 시각·payload), CAN RX 전량, TX 간격 최대, loop tick, 프레임 도착 간격, gyro gaps, 직전 명령 종류, 호스트, KF 해시, epoch−monotonic 오프셋, 프레임별 t_capture·β·d_h·p̂·quality·heading_valid, 자이로·가속도 원시, KF x̂ v̂ ψ̂, bag. 진입 게이트 첫 10건 TXACK 간격 99%<20 ms ∧ gaps 0. 통과 67 σ_e≤2 cm ∧ L/R σ_θ≤0.5° ∧ 최종 정지 외부 σ≤T_f/3(자리). 분위수 n 명시·q99 금지.
  | 검증: precheck (1) 자체 — 차단 4종×≥3회의 T_f/유지 판정과 emergency 반응·해제가 TXACK·IMU·카메라 로그만으로 재구성되는가; 첫 10건 게이트 통과 후 본 실험.

## 핵심 결론 3줄 (judge)
1. 정지 예측 = 셀(67/97×fwd/bwd)별 비례 D̂=τ_eff·v̂ + 잔차 분포(n 명시, n≤20 이면 q99 금지), 발령 x_rem(카메라 forward) ≤ D̂, m=0, 크립 복구 없음. **태그 컷 3.3 m→정지 2.02 m 의 마지막 1.28 m 는 카메라 블라인드**라 dynamics 로 못 닫음 → 97 안전속도 + S(T) 분포표로 다리, 해소(depth 정지/태그 높이/T 4.0 m)는 대문제 4.
2. 시각·응답은 200 Hz IMU(가속도 t_resp → τ_d, |ω| 문턱은 정지창 지표 최대×SF 실시간 도출), 정지 확정은 카메라 v̂. stop 재전송·자동 e-stop 폐기(movement_tx_loop 10 ms 반복, 0x80 의미 미확인). **9/7 무응답·오버슈트 꼬리는 VM 호스트 사건** → dynamics 상수로 넣지 않고 호스트 계측(TXACK·loop tick·gaps)·CAN TX 전용 스레드+데드맨·차량 워치독 실측으로 분리.
3. 회전은 고정 ROT_LEAD_DEG → IMU 콜백 스레드 ω 규칙(임시 τ_r 0.18 하나, L/R 셀, 개루프 펄스 캘리브, σ_θ≤0.5° 는 블라인드 lateral 누설 때문에 요건); 온라인 적응 없음. **config 순감 12**(삭제 10 + 코드 내부 3 − FORWARD_SLOW 신설 1), FWD_T0/T1/A·ROT_T0·ROT_DEG_PER_SEC 는 캘리브 파일로.

## 남는 리스크 (judge top 2)
- 블라인드 1.28 m: 가시 구간 σ_e 1~2 cm 가 최종 정지에선 추측항법 ±13 cm 로 둔갑, 외부 줄자 ×10 없이는 도킹 KPI 본체 미검증 — 대문제 4 의 depth 정지/태그 높이/T 4.0 m 결정에 전부 걸림.
- 차량측 CAN 워치독·emergency 0x80 동작이 한 번도 관측된 적 없고 9/7 데이터가 VM 에서 나왔다: precheck (1) 을 건너뛰면 호스트 정지가 τ_start,q99 상수로 굳고, 데드맨 없는 TX 스레드는 결정 루프가 죽어도 마지막 forward 를 계속 송신한다.

## 미확정 (실차로)
T_f(차량 워치독)·emergency 반응·복구 절차·RX 프레임 존재 · σ_a,still·σ_ω,still(적재·마스트·유압 조건) vs 97 크립 통계 겹침 여부 · σ_τ 측정 가능 여부·τ_start 1차 센서 · v(97)·데드존·c₀·포화·stiction·주행 중 전환 램프 · 소각 코스팅 유압 혼입·L/R 통합·저속 회전 단 · 적재 시 τ_eff 변화 · σ_S(97 정지출발) 가 T_f 안에 드는지 · 트랙션 컨트롤러 기종(Curtis/Zapi/Sevcon) — Dropbox `철기원…/코드/` 확인(사용자 행동 항목) · 광운대 FSM v3/v4 fsm.py·configure_forward_slow 인자·실주행 로그(직접 요청).
