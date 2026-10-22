### 쟁점 1: Timestamp 체계
- 판정: 부분동의 (6시각 스키마·global_time assert·큐 1·config 격하 동의; 통과기준·TXACK 역할 반박)
- 반박 근거: (a) **검증 (b) 의 통과기준이 뒤집혀 있다.** t_capture 가 진짜 캡처 시각이면 ψ_cam(t_capture) 와 ψ_gyro 는 같은 시각축이라 상호상관 지연 ≈ 0 이어야 하고, 도착시각을 쓰고 있을 때 지연 ≈ τ_cam(80~110 ms) 가 나온다. 제안(그리고 plan 1-6 원문)은 "≈0 이면 버그" 라 **정상 시스템이 게이트를 못 넘고 버그가 통과**한다(Larsen 1998·PX4 EKF2(C3) 의 시각정합은 정합 후 잔차 지연 0 이 정의; B27 66 ms 는 도착시각 기준 값). (b) TXACK↔host 최소제곱 적합은 호스트 지연이 +쪽으로만 붙는 분포라 스톨이 기울기·오프셋을 끌어간다. (c) t_cmd_set 의 실제 플립은 `CanDriver._hold`(`/Users/seojeongmin/krri/apriltag_v3/src/models/control/control_from_pose.py:615`) 의 `c.current_movement =` 이고 첫 송신은 `movement_tx_loop`(`control_forklift_v2.py:413`, 10 ms) 다음 틱 — 시각은 TX 루프에서 "payload 가 바뀐 첫 프레임" 에 찍혀야 한다. (d) TXACK 는 RX 큐로 오는데 v2 는 `busOn()` 만 하고 `read()` 0회(`control_forklift_v2.py:198-207`) → 리더 스레드 신설 = 1080p AT3 검출과 GIL 경쟁 1개 추가(FM8 후보). 자이로도 `imu_yaw._on_frame` 이 `m.get_timestamp()` 장치시각(`imu_yaw.py:153`) 이라 모션 스트림 domain assert 는 필수(제안 포함, 동의).
- 대안: 검증 (b) 를 **2-지연 검정**으로 — lag(ψ_cam@t_capture, ψ_gyro) = 0 ± 15 ms **∧** lag(ψ_cam@t_arrival, ψ_gyro) ≈ median(t_arrival − t_capture); 둘 다 맞아야 통과(한쪽만 0 이면 도메인 오류). t_cmd_tx 1차 = TX 스레드에서 payload 변경 첫 `ch.write()` 직전 `time.time()`(광운대 `_write` monotonic_ns 훅 + epoch 오프셋 1개 기록); TXACK 는 **감사 신호**: 잔차(host − txack) 의 하한 포락선으로 오프셋, 잔차 > 20 ms 는 FM8 플래그. 리더는 TX 스레드 안 `read(timeout=0)`.
- 대안 근거: 지연 보상 3종(Larsen·Kalaria·PX4) 모두 "측정 시각이 맞다" 를 전제하며 그 검증은 교차센서 정합만 가능; 하한 포락선은 편측 지연 분포의 표준 오프셋 추정; `control_광운대.py:207-209, 324` 이식.
- 실차 검증법: 8°/s 제자리회전 ×3 에서 2-지연 쌍 → (0±15 ms, 66~110 ms) 이면 통과; (66~110, 66~110) 이면 t_capture=도착시각, (0, 0) 이면 자이로도 도착시각.

### 쟁점 2: Perception 오차와 Dynamics 오차 분리
- 판정: 부분동의 (순서 강제·로드 거부 동의; 해시 사슬 구조·회귀 주장 반박)
- 반박 근거: (a) 세션마다 실측하는 τ_cam·arrival 통계가 해시에 들면 **매 세션 perception/dynamics_calib 이 전부 거부**된다 — 제안의 "τ_cam 필드만" 도 τ_cam 은 세션마다 ms 단위로 달라 같은 결과. (b) "①잔차 vs (t_arrival−t_capture) 기울기 ≈ v̂ 이면 도착시각" 은 성립 안 함: t_capture 가 가짜(=도착)면 회귀변수 분산이 0 이라 기울기가 **추정 불가**(v̂ 가 아님); 300 ms 부하 주입도 두 시각을 함께 밀어 무효. 이 회귀가 잡는 건 "KF 가 되감기를 했나" 뿐이다. (c) "TXACK 첫 10건 p99" 는 n=10 이라 max 와 동일 — plan 2-7 스스로 q99 금지(Wilks n≥59). (d) ② 의 t_stop,IMU 는 plan 2-3 이 매 정지 실시간 도출하는 문턱에 의존 → ①−② 차에 판별 지터가 섞여 "이동 중 perception 바이어스" 로 오분류될 수 있다.
- 대안: 사슬을 두 층으로 — **정책 해시**(queue=1·domain=global_time·stale 규칙·KF Q 스케줄·git·호스트 종류) 는 일치 필수, **측정값은 허용창**(|τ_cam,session − τ_cam,calib| ≤ 20 ms ∧ gaps=0 ∧ arrival p99 < stale). 가짜 t_capture 검출은 쟁점 1 의 2-지연 검정+도메인 assert 에 전담시키고 회귀는 "KF 되감기 검증" 으로 개명. 게이트 문구 "첫 10건 max < 20 ms".
- 대안 근거: critic §21(타이밍 → 로깅 → 피팅), plan 2-6 "τ_eff 는 KF 지연을 흡수하므로 추정기 **설정**과 짝"(짝은 설정이지 세션 측정값이 아님), Kalaria 2022(지연은 추정기 입력 — 입력의 진위는 시각정합으로만 확인).
- 실차 검증법: 같은 날 timing 게이트 2회(아침·오후) → 두 timing_calib 모두로 dynamics_calib 로드 허용(정책 동일·τ_cam 차 < 20 ms) ∧ 300 ms 부하 세션의 τ_eff 이동 < σ_e.

### 쟁점 3: 99% 성공 정의 — tolerance·KPI
- 판정: 부분동의 (결합 허용영역·(n, 실패수, 하한) 보고·구성별 분리 동의; KPI 누락·격자·진값 독립성 반박)
- 반박 근거: (a) **잘못된 DONE(외부 진값이 결합영역 밖인데 DONE 선언) 이 KPI 에 없다** — 안전 렌즈에서 1차 지표: abort 10% 는 운용 가능, wrong-DONE 1% 는 포크·탑재부 충돌. (b) 시작 격자 d=6·ℓ=±2 는 plan 3-4 포락선 |ℓ| ≤ (6−3.5)·tan31° = 1.55 m **밖** → dog-leg/출발거부 셀이 "도킹 실패" 와 섞이는 교란. (c) 태그 2 를 "계측기" 로 달아 진값을 얻고 같은 태그를 제어 입력으로 승격하면 진값과 제어가 같은 계통오차(태그 2 배치·크기 1%) 를 공유 — 독립 검증 상실; 태그 2 는 정면이라 heading 진값을 못 준다(제안 인정) → 3축 중 1축 진값 부재. (d) rule of three n≈300 은 회당 2~3분 → 10~15 h, 2일 방문에 불가 — "99%" 는 현장 검증 수치가 아니라 시뮬 + n≥59(95%) 로만 뒷받침됨을 명시해야 한다.
- 대안: KPI 서열 = **wrong-DONE 0/n(하드)** → 결합영역 성공률(95% 하한) → abort 율(Tier·원인) → 나머지. 격자는 포락선 안(8 m |ℓ|≤2.8, 6 m ≤1.5) 으로, 포락선 밖은 별도 "dog-leg KPI". 진값은 승격 후에도 **매 세션 첫·끝 5회 줄자/레이저 3축**(heading 은 포크 양끝 2점) 유지. 상수는 `LAT_TOL_M/HEAD_TOL_DEG` 대신 도면값 `POCKET_W_M·FORK_W_M·INSERT_DEPTH_M`(물리 3) 에서 T_l·T_h 파생(config 순감 유지).
- 대안 근거: critic §23(평균 금지·abort·correction), Richter 2022(인지≠실행 → 외부 진값 필수), Walter 2010(35/38 식 시도 수 보고), plan 3-4 포락선 수치, plan 2-7 Wilks 95/95.
- 실차 검증법: 명목 도킹 n=20 의 DONE 전건 줄자 3축 → wrong-DONE 0 ∧ 결합영역 ≥17/20 이면 Stage A 통과; wrong-DONE 1건이면 FINAL 수용식(plan 3-3) 재설계.

### 쟁점 4: Failure mode 7종 — 감지·대응·검증
- 판정: 부분동의 (FM 코드 1:1·고장주입·FM8/9 추가 동의; FM8 대책·FM1 신호·FM3 시험·수동전환 반박)
- 반박 근거: (a) **"TX 전용 스레드+데드맨이 stop 유지" 는 호스트 전체 정지(9/7 VM 사건)엔 무력** — 같은 프로세스 스레드는 함께 멈춘다; 그때 차를 세우는 건 차량측 워치독만인데 `control_forklift_v2.py:35` 주석("Heartbeat 250 ms 요구") 이 존재를 암시할 뿐 미검증. 185709 #3 은 gyro gaps 인데 회전은 10.7° 초과 계속 → **FM8a 센서 스레드 정지**(gaps→즉시 stop 으로 소프트웨어 해결, plan 2-4) 와 **FM8b 호스트 정지**(차량 워치독/외부 하드웨어만) 는 다른 FM 이다. (b) FM1 감지 "|Δψ_cam−Δψ_IMU| 초과" 는 plan 3-5 가 **옳은 branch 에서도** 비 0.6~1.25 로 관측한 신호 → FM2 와 같은 신호의 이중해석. 잘못된 branch 는 Δψ_cam ≈ −Δψ_IMU(부호 반전)이므로 **부호**가 분리 신호. (c) "181437 재생 반전 0" 은 plan 3-1 이 그 반전을 재앵커 바이어스(FM2) 로 판정했으므로 FM3 시험이 아니다. (d) **수동전환 미정의**: ABORT=stop+heartbeat 지속 중 운영자가 조이스틱을 잡으면 0x1E3 송신원 2개 → 차량 중재 거동 미지 — critic.md 밖의 빠진 FM.
- 대안: FM 목록에 **FM8a/8b 분리, FM10 wrong-DONE(쟁점 3), FM11 이중 송신원** 추가. FM11 대응 = 명시적 RELEASE 상태(movement 송신 중단·heartbeat 만 또는 물리 스위치) 를 precheck (1) 실측 후 결정. FM1 = pitch-cue SPRT ∧ sign(Δψ_cam) ≠ sign(Δψ_IMU). FM3 시험은 합성(STOPPING 중 반대부호 오차 프레임 주입 → 명령 0). FM8b 대책은 "차량 워치독 확인" 이 선행조건, 미확인이면 사람 제동 위치 + 97 한정 운용.
- 대안 근거: critic §24 Failure 1·3 + §19 fallback; plan 2-2 "9/7 꼬리 = VM 호스트 사건", plan 3-5 ⓪ 수치; Liu 2021(재투영으로 branch 못 잡음 → 부호·pitch 만 남음); robot_localization sensor_timeout(B23).
- 실차 검증법: precheck (1) 에 2항 추가 — ① 97 크립 중 heartbeat 만 차단 → 정지 여부·시간(FM8b 가 차량 워치독으로 닫히는가) ② PC 가 forward 송신 중 조이스틱 stop 조작 → 어느 명령이 이기는지 CAN RX+IMU 로 기록.

### 쟁점 5: 블라인드 최종구간 해소
- 판정: 부분동의 (B 1순위·계측기 선행·D 폐기·depth 는 캡 동의; 배치 보정 방향·수용치·heading 부재·depth 부하 반박)
- 반박 근거: (a) 자기보정 수용기준 "줄자와 ±20 mm" 는 T_l 30 mm 의 2/3 를 **프레임 평균으로 안 줄어드는 계통오차**로 소비; 공시야 4~6 m 에서 0.2 m 태그는 45~68 px 라 배치 std 자체가 10 mm 급. (b) 방향이 거꾸로다 — 포켓은 탑재부에 고정이므로 **탑재부 위 태그 2 가 기준**이고 원거리 태그 1 을 태그 2 기준으로 보정해야 한다(원거리는 β 만 쓰므로 20 mm 오차 무해). (c) 태그 2 정면·카메라 높이 → PnP heading 없음 + pitch-cue 도 소멸(h≈0 → 두 해 pitch 동일) → 최종 heading 은 자이로 앵커 나이에만 종속, KPI heading 진값 없음. (d) run 경로는 depth 를 열지 않는다(`run.py:181` `from_realsense` 기본, `with_depth` 참조 0) → 1080p+depth 동시 USB/CPU 부하가 쟁점 1 타이밍을 바꿈; emergency 0x80 미실측(plan 2-3); 평면 RANSAC 실패(구멍·반사)를 "캡 통과" 로 오독 가능.
- 대안: **B′ = 근거리 태그 2장(카메라 높이, 횡 기준선 b_T ≈ 0.6~1.0 m) 또는 1장 yaw 25° 기울임** — 2장이면 두 태그의 (x, z) 차로 heading(태그 크기 오차는 공통모드라 상쇄, 예측 σ_ψ 0.2~0.5°), 1장 기울임이면 plan 1-3 heading_valid 원뿔 안에서 PnP heading. 배치는 탑재부 도면·줄자 **물리상수**(태그 간 거리·포켓 중심 오프셋), 태그 1 은 이들 기준 자기보정. depth 캡은 "평면 없음 = 캡 없음 → 97 강제+경고" fail-safe, 타이밍 게이트를 depth on 구성으로 재실행.
- 대안 근거: Mateos AprilTags3D(기울인 태그 2개 99%), Oh & Kim 2025(1~2 m 2마커, PnP yaw 6.6° vs 회귀 3°), Seelinger & Yoder 2006(피듀셜 다중), Richter 2022(정면 원뿔 회피), Adámek 2023(정면 yaw 분산 피크); `tag_layout.fuse()`(`/Users/seojeongmin/krri/apriltag_v3/src/utils/tag_layout.py:67`) 가 다태그 융합 자리를 이미 가짐.
- 실차 검증법: 근거리 태그(2장 또는 기울임 1장) 정적 1/1.5/2 m × heading 0/±3° × 100 프레임 → heading std ≤ 0.5° ∧ |median−진값| ≤ 0.5° ∧ lateral 줄자 |차| ≤ 10 mm; 미달이면 B 단일+자이로 짧은 다리로 격하하고 T_h 를 자이로 앵커 나이로 재산.

### 쟁점 6: 시뮬레이터 검증 계획
- 판정: 부분동의 (진짜 파이프라인 통과·같은 jsonl 스키마·9/7 재현 게이트 동의; 시기·채택조건·플랜트 누락 반박)
- 반박 근거: (a) 9/7 재현에는 b(d,tilt)·재앵커 바이어스 구조(plan 3-5: 반전의 기구)가 필요한데 b 는 Day 0 정적 격자 이후에만 있다 → **현장 전 A/B 는 게이트 미통과 상태**(순환). (b) 채택조건 "1차 KPI 개선 ∧ 반전 비악화" 에 **wrong-DONE 0·abort 율 비악화** 가 없어 공격적 설정이 성공률을 올리며 wrong-DONE 을 만들 수 있다. (c) 플랜트에 차량 워치독·이중 송신원·호스트 정지(프레임·gyro 동시 stale) 파라미터가 없어 FM8/9/11 고장주입이 시뮬에서 불가.
- 대안: 시뮬 2등급 명시 — **G1 계약·회귀 하네스(지금)**: 진짜 `TagPipeline.process`·`Result`·상태기계 전이표·jsonl 스키마·고장주입(호스트 정지 = 전 스트림 동시 stale 포함) / **G2 순위 도구(perception_calib 후)**: b·σ_c·flip p·τ_eff 실측 주입 + 9/7 재현 통과 후에만 A/B 채택. 채택조건에 wrong-DONE 0 ∧ abort 율 비악화 추가.
- 대안 근거: `smoke_dock.py` 교훈(계약은 진짜 클래스만 잡음), Yasunobu 1985·MERL US10093331(시뮬은 후보 선택·불변집합 도구, 인증 도구 아님), Poreski 임계 스윕(순위엔 실측 잡음 필수), PLAN_legacy §4 6번은 계약검증만 앞당김.
- 실차 검증법: Day 0 격자 b 주입 후 v2 컨트롤러 재생이 181437 의 8→9→10 반전을 재현(21±5 스텝·반전 ≥1) — 재현 못 하면 G2 채택 금지 유지.

### 쟁점 7: 현장 실험 목록·순서
- 판정: 동의 (Day 0 격자 앞당김·plan 2-7 vs 3-6 모순 해소·파일 사슬 순서 옳음; 보완 4)
- 반박 근거: (a) bag 11 GB/분(`tools/check/live_pose.py:423`) = 183 MB/s 지속 쓰기 — Jetson 에서 카메라·Kvaser 와 USB 대역을 공유하면 드롭·arrival 지터를 **측정 대상인 타이밍에 주입**(관측자 효과); (0) 게이트는 bag 미녹화인데 격자·회전은 bag 녹화 구성이라 두 구성의 timing 이 다르다. (b) 사무실 b(d,tilt) 가 현장과 같다는 전제는 원인이 왜곡모델(plan 1-2 의심)일 때만 — 태그 휨·부착면이면 현장 종속. (c) Day 1 순 실험 255 min + 이동·배터리·재배치 ×1.5~2 → 7~8 h, 여유 0; 게이트 실패 시 격하 규칙은 있으나 **필수 최소 셀** 표기가 없다. (d) 수동전환·이중 송신원 시험(FM11) 이 (1) 에 없다.
- 대안: (0) 을 **bag on/off 두 구성**으로 각 60 s → arrival p99 차 기록, bag 은 별도 USB 컨트롤러 SSD 직결 또는 720p(5 GB/분). Day 0 격자는 **현장에 부착할 그 인쇄물을 강성 판에 붙여** 수행, Day 1 (2) 에서 3.5 m·tilt 20°·정면 두 셀만 재검(±0.3°). 블록마다 필수/선택 표기 — 필수 = (0)(1)(2) + (3) 폐루프 L/R×15°×6 + (4) 67 n=20·97 stiction ×10; (1) 에 조이스틱 개입 ×3 추가.
- 대안 근거: plan 2-7 순서·게이트, plan 1 실험 1~4, prior_art "다음 현장 측정 목록", CLAUDE.md VM 실주행 리스크(호스트 Jetson/Windows), critic §12·§21.
- 실차 검증법: Day 1 (0) 에서 bag on/off arrival p99 차 < 30 ms 이면 bag 구성으로 전 블록 진행, 넘으면 bag 은 지정 셀만.

### 쟁점 8: 문서 통합 — 병합·plan 구조·코드 진입 조건
- 판정: 동의 (단일 진실원 plan.md·상수 대장·캘리브 등록부·rationale 동결 동의; 게이트 1개 분리)
- 반박 근거: G_code 가 "사용자 결정 6개 확정" 을 전제하면 x_ref·D·tolerance 출처·태그 2·카메라 피치는 **수요기업/탑재부 도면 소유**라 주 단위 지연 가능 → 코딩 전면 정지. 그런데 제안의 코딩 순서 앞 4단(시뮬 골격→타이밍/로깅→추정기→캘리브 로더) 은 6개 결정과 무관하다. 또 "rationale 동결" 과 "v2 424행 부호 수정" 은 충돌(동결 문서를 고침).
- 대안: G_code 를 **G_core**(타이밍/로깅·추정기·캘리브 로더·시뮬 G1·상수 대장 삭제 목록 — 결정 무관, 즉시 진입 가능) 와 **G_final**(FINAL 상태·수용식·zone·근거리 태그 — 결정 6개 후) 로 분리. methodology 는 완전 동결, 오류는 plan 에 "정정 각주". 상수 대장에 wrong-DONE 하드 규칙 열 추가.
- 대안 근거: critic §22·§26 P11·12; CLAUDE.md "plan 확정 후 coder"(절 단위 확정 가능); Nav2/autodock 은 파라미터 파일 1개 + 명시 상태기계(A11·A12).
- 실차 검증법: (문서 항목) 검증 = 상수 대장 초안에서 config 75개(control 39·detection 28·imu 8) 전부가 유지/삭제/캘리브/코드내부 중 하나에 배정·미결 0 ∧ G_core 항목이 사용자 결정 6개 중 어느 것도 참조하지 않음을 리뷰로 확인.

### 가장 위험한 제안 TOP2
1. **쟁점 4 FM8** — "TX 스레드+데드맨이 stop 프레임 유지" 를 호스트 정지 대책으로 두는 것: 9/7 형 호스트 정지에선 무력하고 차량 워치독 미확인 상태에서 forward 중 정지하면 폭주, 게다가 수동전환(이중 송신원) 거동이 미정의 — precheck (1) 의 heartbeat 차단·조이스틱 개입 실측 없이는 어떤 실주행도 안전 근거가 없다.
2. **쟁점 1(b)+2** — 뒤집힌 타임스탬프 통과기준(≈0 = 버그) 과 원리적으로 못 잡는 도착시각 회귀 검출: 가짜 t_capture 가 게이트를 통과해 τ_eff 에 카메라 지연이 학습되고(critic §21 그대로), 그 위에 perception·dynamics 캘리브 사슬 전체가 세워진다.