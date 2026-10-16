# KRRI AprilTag 지게차 도킹 — 작업 규칙 (Claude 가 매 세션 읽는다)

카메라로 AprilTag 을 보고 탑재부 기준 지게차 위치를 낸 뒤 그 값으로 도킹한다.

## 저장소 규칙 (이걸 틀리면 안 된다)
- **활성 개발 = `apriltag_v5/`** (2026-10-01 시작 — v4 커밋 26d1c91 을 그대로 복사해 이어 감. 설계 = `apriltag_v5/plan.md`, v5 에서 바뀐 것은 0절).
  **`apriltag_v1/`·`apriltag_v2/`·`apriltag_v3/`·`apriltag_v4/` 는 동결 — 읽기만, 절대 수정 금지**(v1 사이드스텝, v2 조준-전진, v3 연속추정 시도·9/21 측정 도구,
  v4 = 2026-09-22~10-01 v2 장비층 + 광운대 신버전 방법론, 10/1 실차 첫 진입까지; 비교·참고용).
  복사해 온 원본(`apriltag_v5/src/utils/VENDOR.md`·`src/models/kwu/ORIGIN.md` 목록)도 수정 금지 — 원본과 바이트 동일해야 diff 가 된다.
  v5 코드는 사용자가 시킬 때만 고친다.
- **`apriltag_v6/`(2026-10-16)** = 칼만 필터(EKF) **오프라인 검토 도구**만 있다(`src/ekf.py`, `tools/ekf_replay.py`) — v5 기록을 읽기만 하고 차를 안 움직인다. 첫 결과·숫자 출처는 `apriltag_v6/README.md`. 활성 개발은 여전히 v5.
- **장소(v5, 2026-10-02)**: `config/site.py` 한 줄 `SITE` = `kw`(광운대) / `krri`(철도연). 장소마다 따로: 현장 값 `config/sites/<장소>.py`(태그·카메라 높이·노출·태그 좌우·
  탑재부 길이·뒤 공간 — control/detection 이 가져다 씀, 비면 출발 거부. **krri 는 지금 kw 값 그대로** — 사용자 결정) · 기록 · 학습 파일. 카메라는 매번 다시 달아서
  calibrate(camyaw·rotcenter)는 **달 때마다** — 그날 그 장소에서 안 했으면 run 이 SPACE 전에 경고(장소도 크게 띄운다).
- **기록 구조(v5)**: `work_dirs/<장소>/<날짜>/run/<시각>/` · `work_dirs/<장소>/<날짜>/calibrate/<시각>/` (git 무시). 찾을 때는 폴더 이름 순서 = 시간 순서(`src/bootstrap.sessions`).
  학습 파일 `config/learned/<장소>.json`(git 무시, 숫자만): `upto`(어디까지 넣었나) · `straight.forward/backward`(시간 직진 speed·dead + 카메라 직진 끊는 τ·잔여 + 최소걸음 + 합계) ·
  `inertia.strength_30/strength_20`(회전 응답 τ·잔여·출발지연·각속도 + 합계) · `gyro`(눈금 · 회전중심 + 합계 — 회전중심은 calibrate 회전중심 단계가 "장착 바뀜" 으로 새로 시작).
  run 이 **시작할 때** 그 장소 기록에서 갱신하고 그 판 내내 고정. 칸마다 자료가 MIN_N(3) 번 안 되면 다른 장소 값을 빌린다(회전중심은 안 빌림).
  기록에서 다시 만들 수 있다(`python -m src.models.control.learnfile --relearn <날짜> [--site krri]`). **seeds.json 은 없앴다**(판 폴더엔 그 판이 쓴 값 `values_used.json`).
  2026-10-01 실차 자료(주행 9판·calibrate 5번)는 VM 의 v4 `work_dirs/runs·calibrate` 에서 맥 `apriltag_v5/work_dirs/kw/20261001/{run,calibrate}/` 로 옮겨 둠 — VM 에 v5 를 보낼 때 VM 쪽도 같이 옮겨야 한다.
- **브랜치**
  - **평소 작업 = `jm_mac`.** 맥북·우분투용 개발·환경 변경은 전부 여기서 하고, 이 맥북은 항상 jm_mac 에 체크아웃.
  - `jm` : 사용자의 로컬(연구실) 컴퓨터 전용. **평소 흐름에서 배제** — 자동으로 당기지 않는다. 사용자가
    "jm 에서 이 부분 고쳤다, jm_mac 에 합칠지 보자" 고 가져올 때만 확인·논의 후 `git merge origin/jm`.
  - VM(`ubuntu-vm`) : `jm_mac` 을 받아 **쓰기만** 한다. VM 안에서 커밋하지 않는다.
    **`git commit` · `git push origin jm_mac` · VM 반영 — 셋 다 자동으로 하지 않는다.** 사용자가 각각 시킬 때만.
  - `main` : `jm_mac` 의 v2 를 주석·docstring 뺀 **부모 없는 단일 스냅샷**(GitHub 기본 브랜치). config/*.py 와
    다른 팀 파일 3종(control_forklift_v2, control_광운대, fwd_time_model)은 주석 유지. 배포용이라 VM 엔 안 올린다.
    루트에 `.gitignore`·`README.md`·`requirements.txt`(정리본, apriltag_v2 밖) + `apriltag_v2/`.
    **main 에 올리는 건 자동으로 하지 않는다** — 사용자가 "main 에 반영해줘" 라고 할 때만 rebuild·force-push.
- **config import 스타일**: control·detection 모두 `from config import control as C` / `... detection as D` + `C.X`/`D.X`.
  config 상수의 원산은 `config/detection.py`. 공개 API 는 `src/models/__init__` 가 config.detection 에서 직접 재수출한다.
- **requirements**: `requirements.txt`(리포 루트) **마커본 하나** — 패키지·OS 마커만, 설명 없음. 우분투/Jetson(arm64)는
  pyrealsense2 를 마커로 빼고 소스 빌드, WSL·그램(x86)은 PyPI 휠, macOS 는 macosx. (풀어 쓴 macos/ubuntu 파일은 없앴다.)
  설치는 OS 별로:
  - **Windows/그램** — 먼저 Kvaser Drivers+CANlib SDK 설치 후 `pip install -r requirements.txt`
  - **우분투/Jetson** — `tools/etc/setup_ubuntu_arm64.sh` 가 이걸 쓴다 (pyrealsense2 는 소스 빌드)
  - **macOS(개발용)** — `pip install -r requirements.txt` (실카메라 안 됨, 웹캠만)
  - **설치 확인** — `python tools/check/check_setup.py`
- **tools 구조(v2)**: 현장 주력 `tools/run.py`·`tools/analyze_run.py` 는 최상위, `tools/check/`(점검), `tools/etc/`(측정·테스트·설치).

## v3 설계 토론 팀 (`apriltag_v3/Agent/`, 로컬 전용·git 무시)
**(2026-09-21 토론 4개 완료. 결론은 v4 plan.md 에 흡수됨 — 아래는 기록용, 새 토론은 안 돈다)**
- 역할·권한·흐름·로그 형식은 **`apriltag_v3/Agent/README.md`** — 토론 관련 작업 전에 먼저 읽는다.
  이전 세션의 분석 결론·합의·리스크는 아래 **"설계 현황·합의"** 절.
- **"토론 시작해"** = `Workflow({scriptPath: "apriltag_v3/Agent/debate_workflow.js", args: {n: N}})` 실행.
  N = 1 Perception / 2 Vehicle Dynamics / 3 Controller / 4 System. 다음 N 은 `Agent/plan.md` 에서 `pending` 인 첫 대문제.
  이 문구는 사용자의 명시적 워크플로(다중 에이전트) 실행 요청이다.
- 끝나면 결과를 `Agent/log/debate_N_<key>.md`(쟁점당 6~8줄, 핵심만) 와 `Agent/plan.md` 해당 절(결론·검증 실험·파라미터 자리만)에
  쓰고, **PushNotification 보낸 뒤 멈춰서 사용자 검토**를 받는다. 대문제 하나씩. 근거 없는 결론은 plan 에 넣지 않는다.
- 토론 단계에선 `apriltag_v3` 코드를 **읽기만** 한다. 쓰기는 `Agent/` 안에서만. v1·v2 는 절대 수정 금지(기존 규칙).
- 알림: 대문제 완료 / 한도 대기 진입·재개 / 에이전트 실패 / 결정·권한 대기 때 PushNotification.
  한도 소진 시 리셋까지 자동 대기·재개(`.claude/settings.json` autoContinueAtUsageLimit) — 크레딧 구매 아님.
- 모델: 세션 Fable 5.1[1m]·effort xhigh → 불가 시 Opus 5[1m] 자동 폴백(settings.json). 토론 서브에이전트는 effort max
  (`debate_workflow.js` 의 `call()` 이 Fable→Opus 재시도까지 담당).
- 코드 작성은 plan.md 확정 후 사용자가 시킬 때 `coder` 에이전트로 v3 만 수정. main 반영은 사용자가 말할 때만(기존 규칙).

## 설계 현황·합의 (2026-10-01 기준 — v4 까지 구현, v5 는 같은 코드에서 시작. 근거·결정 요약·숫자 대장은 `apriltag_v5/plan.md` 0·13·14절)
**작업 정의**: 지게차가 **탑재부(차고형 공간)에 들어가 주차**한다. 포크는 접힌 채, 팔레트 삽입 아님. 탑재부 폭 = 지게차 폭 + 6 cm → 한쪽 여유 **3 cm**(`SIDE_GAP_M=0.030`, 실물).
1차 목표 = **포크 끝이 입구**에 정렬. 눈감고 가는 거리 = 태그컷 − 포크끝 − `STANDOFF_M`(실험용 0.5, 실증 땐 0) ≈ 1.3 m. 2차(전체 진입)는 나중.
**태그컷은 판단마다 다시 잰다 — 줄자 높이차 + 가속도계 피치**(2026-10-10, `limits.tag_cut_pitch_m`: 높이차 = 장소 파일 태그 높이 − 카메라 높이, 피치 = 판단 직전 0.5초 가속도 중앙값, 직진 중엔 마지막 값. 가속도계↔카메라 피치 어긋남은 0 으로 본다). 예전 PnP 식(`tag_cut_live_m`: `vertical` + 윗변 행)은 평지에서도 한 run 안 12~30 cm 흔들려서 가속도계가 없을 때만 쓴다. 10/1 기록: 새 식 3.00~3.10 m(run 안 폭 2~7 cm) · 실제(윗변이 60 px 에 닿는 거리) 3.12~3.13 m — 가깝게 잡아도 윗변은 42 px 위(안 잘림). 출발 자리 60초에서 어긋남을 맞추면 최대 23 cm 멀리 잡혀서 버렸다. **krri 는 태그 높이·카메라 높이를 줄자로 다시**(높이차 1 cm → 태그컷 2.7 cm). 3.3 m 는 피치 0 명목값(폴백). 눈 감은 뒤 경사가 바뀌는 건 못 본다(사용자 수용).
**measured.py·before_run.py 는 없앴다(2026-10-02)**: 하루 한 번 재던 20개 값 중 차 치수 3종(회전중심·카메라 어긋난 각·회전 하한)만 `tools/calibrate.py` → config, σ 기준선·태그 기울기는 run 이 출발 뒤 그 자리에서(`_baseline`), 나머지(회전 응답·속도)는 씨앗 + 학습 파일(v5: `config/learned/<장소>.json`, run 시작 때 기록에서 — seeds.json 은 없앴다). 태그컷·높이차는 프레임마다 live.
**실측으로 확정된 것** (9/7 실주행 30프레임 측정 38개 · 9/21 실차 3.68 m 정지 1459프레임):
- 좌우 = 거리 × tan(방향 흔들림). 3.7 m 에서 **한 장 좌우 sd 39 mm·방향 0.59°**, 8 m 에선 60~145 mm. 30장 평균해도 31 mm — 오차가 프레임 간 상관 0.6 으로 **천천히 밀려서** 평균으로 안 준다.
- 정면이 최악(두 해 모호), 비스듬·올려다보기가 조건수를 잡는다. 카메라–태그 **높이차가 지배**(실측 0.78~1.23 m 로 날마다 달랐음; config 1.10 은 실측 아님 → 줄자).
- 9/21 로컬 `apriltag_v3/work_dirs/first_run/*_all` 20개·`dock/*_sim` 은 전부 **가짜 소스**(리허설). 진짜 9/21 은 `apriltag_v3/work_dirs/20260921_측정기록.json` 하나.
- 회전 정지는 지수감쇠가 아니라 **유지 → 급정지 → −29 % 되튐**. 출발은 1.0 s 무반응 뒤 ~2 s 램프. 카메라 지연 중앙 21 ms(p99 95).
- **(2026-10-02 실차 calibrate, 4.2 m ±30° 스윙 18점) PnP 방향·좌우는 자세마다 치우친다**: 자이로 기준으로 방향이 +4 ~ −3°(좌우 ±0.2~0.3 m)씩 틀렸고, 같은 자리를 갈 때·올 때 **같은 쪽으로** 틀렸다 — 잡음이 아니라 치우침이라 정지 60초 σ(좌우 밀림 19 mm)에는 안 보인다. 믿을 수 있는 건 **화면위치 β(0.004°)·거리·자이로**. 그래서 회전중심은 거리+β 원 맞춤으로 잰다(카메라 앞 **0.46 ±0.17 m**, 오른쪽 0.12 m; v2 의 1.46 은 틀렸다). ★계획기는 아직 회전 뒤 PnP 좌우·방향을 그대로 믿는다 — 30 mm 를 맞추려면 방향을 다른 길(자이로·여러 자세·depth)로 얻어야 한다. 미해결.
- **(2026-10-02) 진입 판정은 차 맨 앞 기준**: 도착 좌우 = 카메라 좌우 + (눈 감는 거리 + `CAM_TO_FORK_TIP_M`) × sin(방향) (`limits.tip_lever_m`). 예전엔 눈 감는 거리만 곱해 카메라의 도착점을 봤다 — 둘째 실주행 끝(좌우 +0.24·방향 −13.7°)에 "여유 +11 mm" 가 나왔는데 맨 앞은 0.37 m 어긋나 있었다. 좌우 0 일 때 진입 가능한 방향은 ±0.67° 뿐이다.
- 둘째 실주행(20261001_151500): 8.7 m·옆 3.8 m 에서 사이드스텝 + 대각선 4걸음으로 67초 만에 법선 위 3.4 m 도착. 그 뒤 태그컷에서 제자리 보정 6회 → 정지. 원인: ① 작은 회전이 안 맞음(+1.75° 요청 → +5.09°. 하한 탓이 아니다 — rotfloor 실측 하한은 0.5°; 가속 중에 끊는 예측정지·잔여 학습이 흔들린 것) ② σ 가 3 m 에서 ±4 mm 로 과신(계획점−도착점 차는 ±3~17 cm) ③ 대각선을 태그컷까지 끌고 가 16° 틀어진 채 도착. 미해결.
- 같은 날 첫 `run.py` 실주행: 8.7 m 에서 PnP 가 좌우 −3.95 m·방향 +27.6°(σ ±0.02 m 로 확신)라 해서 사이드스텝을 골랐고, 회전 9° 에서 **프로세스가 0.32 s 통째로 멎어 데드맨(0.30 s)이 세웠다**. VM 에서 장비를 물리면 0.1 s 넘는 멈춤이 4초에 한 번꼴(장비 없이는 없음) → `DEADMAN_S`·`IMU_STALE_SEC` 를 1.0 으로(사용자 결정, 직결이면 0.30). 멈춤 원인(USB 부하/해상도)은 미확인.
- 같은 날: 실외 햇빛에서 고정 노출 8.3 ms 는 화면이 하얗게 날아가 태그를 못 찾는다 → config `COLOR_AUTO_EXPOSURE`(실외 True / 실내 False, `src/utils/tune.py`). VM 에서 `QUAD_DECIMATE 1.0` 은 12.5 fps(→2.0 으로 23 fps), 자이로는 60초에 0.6° 흐름(정지 판정은 각속도로), 자이로 샘플이 뭉텅이로 옴(처리간격 중앙 1.4 ms·p99 39 ms), 맥 화면이 꺼지면 USB 가 멈춰 CAN 스레드가 죽음(`caffeinate` 필수), 스윙 중 회전 관성 7~10° 가 두 번(USB 멈춤).
- 합성 시뮬로 정확도를 주장하지 않는다(모서리 잡음 0.05 px 가정이 실측 0.20 px 의 1/4 이었음). 시뮬·가짜 리그는 **없앴다**(2026-10-01) — 검증은 실차와 실측 로그로만.
**합의된 설계** (plan.md 3절, 2026-09-30~10-01 결정):
- **조향 강도 30**(광운대와 동일) → 그쪽 실측(출발 1.082 s·정속 12.01°/s·관성 0.283 s)을 `src/models/control/learn.py KWU_SEED` 로 물려받음 (후진 187 은 안 쟀다 — **직진과 같다고 가정**, 2026-10-02 사용자 결정; 첫 후진에서 학습). 강도 30 아니면 출발 거부.
  **직진·후진은 byte 67/187**. 97(forward_slow)은 광운대 옛 버전 잔재라 아무도 안 잰 값 → 안 쓴다.
- 흐름: **사이드스텝(처음 한 번, 통로 밖이면) → 중간점 계획(빔서치 6걸음 그려보고 첫 걸음만 실행, 재측정) → 진입 조건 → 눈감고 직진 → 정지.**
  통로 반폭 = `0.8 × tan(경로각상한) × (거리 − 태그컷)`, 경로각상한 = 반화각 − 태그반폭 − 가장자리 — 전부 `src/limits.py` 계산(상수 아님, intrinsics 는 카메라에서 읽은 것).
  사이드스텝 = 법선 기준 ±90° 회전(IMU 폐루드) → 읽은 좌우의 **절반**을 시간 개루프로 → 법선 정면. **앞으로만 간다.** 회전 상한 180.
- **마지막 직진 구간(2026-10-02 결정)**: 접근 계획은 **정렬선 = 태그컷 + `FINAL_STRAIGHT_M`(1.5 m)** 까지 좌우·정면을 끝내고, 그 안쪽은 태그를 보며 **1.5 m 를 한 번에** 간다(2026-10-01: 0.75 씩 끊던 것 폐기). 고치는 건 정렬선·태그컷 두 곳에서 **빗나감 > 3 cm** 일 때만이고, 돌기만 한 뒤 그 자리에서 다시 잰다. 그 구간의 회전은 **약한 강도 20**(`rotate(fine=True)`, 회전 직전에 프레임을 써넣고 8바이트 확인)만 쓰고, 판단은 PnP 방향·좌우가 아니라 **조준 빗나감**(`src/models/planning/aim.py`: 거리 × sin(c − β) + `CAM_LATERAL_OFFSET_M`)으로 한다. c 는 직진 다리마다 (거리·β·자이로) 직선 맞춤으로 실시간 도출하되 **좋은 직진(태그 5 m 안·|β| ≤ 10°·0.5 m 넘게 감)만 같은 무게로** 쓴다(v5 F4, 2026-10-10 — 10/1 28개: 좋은 직진 0.54~0.91°, 먼·비스듬·짧은 다리는 RMS 0.64~0.87° 벗어남). 좋은 직진이 없으면 정렬선에선 전부로 대충, 태그컷 진입은 안 함(정지·사람). 태그컷에서 `|빗나감| ≤ 3 cm` 면 진입(σ 는 기록용). **마지막 구간에 한 번 들어가면 머문다**(v5 리뷰 C, 2026-10-10 — 다시 잰 태그컷이 평지에서도 12~30 cm 흔들려 접근 계획으로 되돌아가 PnP 좌우를 믿고 섰다: 10/1 17:04). 조향+전진 동시는 여전히 안 함. ★`CAM_LATERAL_OFFSET_M`(카메라가 차 중심선에서 벗어난 양)은 줄자 미측정(0 가정). 옛 태그컷 분기(정면부터·후진)는 `final_m=0` 일 때만 타는 죽은 길로 남아 있다 — 정리 대상.
- **후진은 최후의 보루 한 곳뿐**: 중간점을 다 갔는데 진입 조건이 안 되면 **한 번** (남은좌우 > 2σ 일 때만, 양 = max(통로식, 최소걸음) ≤ 뒤공간). 사이드스텝·중간점 후보·탐색(3-8)에는 후진 없음.
- 회전 = 자이로 폐루프(SDK 콜백 안에서 끊음, **출발을 본 뒤에만** 예측 정지, 끝에 정지 한 번 더 — v5 #1). 상한 = 돌고 나서도 태그가 화면에 남는 각(`limits.turn_cap_deg`, 33~35°) — 광운대 17.1° 는 **v5 에서 뺐다**(R2). 하한 = config `ROT_FLOOR_DEG` **0.50°** · 약한 회전 `ROT_FLOOR_FINE_DEG` **0.84°**(calibrate 20261001_161003: 움직이자마자 끊은 10회의 최대. 되튐 뒤 남는 각은 0.1~0.5° — 강도 20 이 30 보다 곱지 않았다). 하한 아래 요청은 거부·학습 제외, 단 약한 회전은 하한의 절반(0.42°)부터 받는다. 주행 중 하한을 올리던 규칙은 삭제. **v5 R1: 강도 30 은 사이드스텝·탐색만, 나머지 회전(접근 걸음·마지막 구간)은 전부 강도 20**(되튐 뒤 오차 30 ±1.40° vs 20 ±0.37°).
- 직진 = 카메라 폐루프 예측정지 `남은거리 ≤ 속도×(검출지연 + τ + 주기/2) + 잔여`. 출발을 본 뒤에만 예측. 한 걸음 상한 1.5 m(계산값이 보통 더 짧다). "보이나" 확인과 출발 기준은 같은 측정 하나(v5 F2 — 예전엔 0.0 으로 잡혀 0.67 m 직진이 1.01 m 감). **출발·멈춤 판정(v5 F5, 2026-10-03)**: 예전엔 남은 거리(PnP 각도·태그컷 섞임)로 봐서 10/1 28번 중 23번 서 있는데 "출발", 12번 다 섰는데 "멈춤 확인 실패". 이제 출발 = 태그까지 3D 거리 속도가 정속의 1/3 을 3장 연속 넘으면(명령 뒤 화면만; 카메라가 못 보면 가속도를 더한 속도 > 정속의 절반; 그래도 못 보면 배운 출발 시간 × 2), 멈춤 = 가속도계로 제동을 본 뒤 조용함 0.3초(기준 = 직진 명령 직전 1초의 앞 가속도). 10/1 다시 돌림: 출발 너무 이름 0·놓침 0, 멈춤 28번 모두 정지 명령 뒤 0.45~1.05초. 시간 직진(사이드스텝·눈 감고 진입)은 그대로 2초 기다림.
- 학습(v5): **주행 중엔 아무것도 안 배운다.** 학습 파일이 run 시작 때 기록에서 쌓고 그 판 내내 고정. **회전** — "기준값으로 끊었다면 되튐 뒤 남았을 오차" 를 역산해 모은 전부로 오차 = a + c·ω(step 회전만, 좌우 합침, 강도별). **직진**(F1, 2026-10-02) — 카메라 직진만 재료: ① 시간 직진(사이드스텝 옆 이동·눈 감고 진입) 유지 = dead + 거리 ÷ speed 를 (명령 쥔 시간, 실제 간 거리) 직선으로 — 2026-10-01 24번 0.2895 m/s · 1.204 s ② 끊는 τ·잔여를 회전과 같은 역산으로 — 기준 τ **0.295 s**(정지 명령부터 미끄러진 시간 중앙; 처음 말한 0.48 은 카메라 지연을 두 번 센 값) → 24번 잔여 +0.007 m, 목표 대비 RMS 옛 8.1 → 3.8 cm. 멈춤 확인 실패(settle_timeout)는 넣고, 시작 측정 못 믿음·놓침·거부·안전장치 끊김은 뺀다. 후진은 자료 MIN_N 번까지 전진 값. 속도·검출지연·판정주기는 매 프레임 **실측**(학습 아님). ③ **최소걸음**(F1-3, 2026-10-11) = 가장 빨리 끊었을 때 실제로 가는 거리 = 출발 인정 순간(`t_onset_ok`)까지 간 거리 + 그때 속도 × τ_기준의 평균(학습 파일 `min_step_m`, 3D 거리 프레임으로; 옛 기록은 출발 규칙을 다시 돌림) — 10/1 **0.167 m ±0.027**(25번). 예전 식 정속 × τ = 0.085 는 출발 인정까지 가는 거리를 빼먹었다. 출발 인정하자마자 끊긴 직진(`cut_at_onset`)은 실제 거리를 그대로 쓰고 τ·잔여 학습에선 뺀다. 태그컷 도착 허용 = max(0.10, 최소걸음) = 0.167. 자료가 없으면 예전 식.
- **자이로(v5 R5)**: 서 있을 때 영점(정지 명령 뒤 1초부터 각을 얼리고 바이어스를 다시 잡는다 — Driver 가 명령마다 알림) · 눈금과 회전중심은 회전 전후 태그 자리(β·거리)로 판을 넘어 배운다(학습 파일 `gyro`). 표본 무게: 거리 방향 1/d⁴ · 화면위치 방향 1/d²(같은 무게면 먼 표본이 좌우 중심을 끌고 간다. 2026-10-01 자료로는 오차 모양을 못 가르고, 어느 모양이든 결과 같음). 눈금은 판끼리 흩어짐까지 넣고 확실할 때만(2026-10-01: 0.9909 ±0.0047 → 안 씀, 문턱 바로 아래). 회전중심은 지금 장착 자료가 MIN_N 이상이면 config 대신 쓴다(2026-10-01: 앞 0.405 · 오른쪽 0.013 m — 앞뒤는 안정, **좌우는 판 하나씩 빼면 ±3 cm 흔들림**. config 는 앞 0.465 · 오른쪽 0.112). 자이로가 끊긴 회전은 학습에서 뺀다. **"정면이 어디인가"(PnP 방향 대신 자이로) — 2026-10-11 사용자 결정: 지금대로(계획기는 PnP 방향·좌우 그대로).** 10/1: 제자리 회전 뒤 좌우 변화가 기하와 PnP RMS 17 cm · 자이로+기준 2 cm 어긋났지만, 태그 한 장으론 기준의 참값을 확인 못 하고(화면 위치 치우침이라 모아도 안 줄 수 있음) 자이로 끊김이 끝까지 남는 단점 때문에 보류. 포크 끝 3 cm 는 마지막 구간 조준이 맞춘다. 2차(방향 1° 안)로 갈 때 태그 두 장과 함께 다시.
- **태그 놓침(v5 R4)**: 단계 먼저 — 눈 감고 진입 중 → 정상 · 마지막 구간 "태그컷까지 곧장" 다리에서 태그컷 근처 → 바로 서고 그대로 진입(눈 감는 거리 = 마지막 본 거리 − 더 간 거리) · 그 밖 좌·우 가장자리 → 그쪽 탐색 · 위·가운데 → 정지·사람 · 출발 때 못 봄 → 탐색. 프레임은 오는데 태그만 없으면 `no_tag`(예전엔 stale → "카메라·시계 이상", 리뷰 G).
- 판단: 계산은 늘(정지·직진 중), **회전 실행만 정지 후**. 회전 중 카메라 자세 불신(자이로만, β 는 씀).
- **σ = √(밀림² + 떨림²)**: 밀림 = **출발 뒤 그 자리에서** 60초 정지(`SIGMA_STILL_S`, run.py `_baseline`)로 잰 기준선(거리 비례; 2026-10-02 부터 매 주행, before_run 없음. 연달아 돌릴 땐 `run.py --reuse-sigma` 로 직전 주행의 `baseline.json` 재사용 — 사람이 판단), 떨림 = 지금 창 잔차/√n. 두 해 헷갈리는 프레임은 한 장씩 제외. σ 없으면 `no_sigma` 로 안 움직인다. σ 는 ①고칠까 ②얼마나 갈까 ④들어갈까 에 쓰이고, 모르면 큰 쪽.
- 마지막 단계 상한: 보정 5회 / 120 s(같은 구간을 횟수·시간으로 이중). 실패 7종 → 정지 + 사람 호출. 기록은 주행을 절대 막지 않는다.
**상수 원칙**: 근거 없이 찍은 값 금지. 출처 분류(실물치수/광운대/우리 실측/규격·수학/식/사람이 정함/아직 찍힌 값)는 plan.md 14절. 새 숫자를 넣으면 거기 등재. 캘리브 값은 `tools/calibrate.py` 가 재서 config 에 적는다(`CAM_YAW_OFFSET_DEG` None 이면 출발 거부, 회전중심 None 이면 회전 상한 5°).
**코드 구조(v5 = v4 + `src/models/control/learnfile.py`)**: `tools/run.py`(본체: 메인=표시·키 / 제어 스레드=검출·계획·명령 / SDK 콜백=자이로) · `tools/calibrate.py`(한 번 재서 config 에 적는 값: 회전중심·카메라 어긋난 각(60초 중앙값)·회전 하한 — 카메라를 달 때마다; 기록 `work_dirs/<장소>/<날짜>/calibrate/<시각>/`, `--write` 면 config 자동 갱신) ·
`src/models/detection/{image,tag,pose,estimate}` · `src/models/control/{driver,frames,rotate,forward,sidestep,learn}` · `src/models/planning/plan` · `src/utils/{clock,gyro,record,hud}` · `src/limits.py` · `src/models/kwu/`(광운대 `calib/control.py` 원본).
시계는 전부 `clock.now()`(monotonic). 카메라 스탬프는 `CameraClock.see()` 로 변환 — 그냥 빼면 −17억 초(2026-09-21 로그 확인). 실차에서 `--dry-run`(CAN 안 보냄)으로 카메라·자이로·게이트 확인 후 실주행. **가짜 리그는 없앴다** — 가짜 데이터로 판단하지 않는다.
**미확정·다음 실차**: 줄자(태그 높이·카메라 높이·태그 좌우 오프셋·탑재부 길이·뒤 공간)는 **그 장소 `config/sites/<장소>.py` 에 직접**(포크 끝은 config/control.py) → `calibrate.py`(회전중심·카메라 어긋난 각·회전 하한 — 카메라 달 때마다) → `run.py` 가 출발 뒤 스스로(σ 정지 60초·태그 기울기). 회전 응답·직진·자이로 눈금·회전중심은 학습 파일(run 시작 때 그 장소 기록에서).
★**지금 배치로는 여유 30 mm 를 못 지킬 수 있다**(태그컷 3.3 m 에서 σ ≈ 32 mm) → 태그를 **낮게·크게** 달 준비. 첫 실주행에서 짧은 다리에 Ctrl+C 를 일부러 한 번 눌러 실제 정지를 확인한다.

## 코드 갱신 (**커밋·푸시·VM 반영 전부 사용자가 시킬 때만**)
파일 수정은 작업 트리(jm_mac 체크아웃)에서만 한다. **`git commit`, `git push origin jm_mac`, VM 반영 — 셋 다 자동으로 하지 않는다.**
- "커밋해" → 커밋만 (푸시 안 함)
- "푸시해" → `git push origin jm_mac` (VM 안 함)
- "우분투로 보내줘 / 받아와" → 아래 두 명령
각각 그렇게 말할 때만 실행한다:
```bash
git push ubuntu-vm:krri.git jm_mac:refs/heads/jm_mac      # GitHub 에도: git push origin jm_mac
ssh ubuntu-vm 'cd ~/krri && git fetch -q mac && git checkout -q -B jm_mac mac/jm_mac && git log --oneline -1'
```
VM 은 리모트 `mac`(맥이 밀어 넣는 bare `~/krri.git`) + `origin`(GitHub). VM 파이썬 `~/miniforge3/envs/krri/bin/python`(3.11).
**jm 병합은 자동으로 하지 않는다** — 사용자가 jm 변경을 가져와 "합칠지 보자" 할 때만 `git checkout jm_mac && git merge origin/jm` 후 논의·검증.

## 주의 — main 리빌드 시 work_dirs 소실 (2026-09-07 겪음)
`work_dirs/`(실주행·live_pose 로그)는 .gitignore 라 git 이 추적하지 않는다. main 을 부모 없는 orphan 단일
커밋으로 다시 만들 때 작업 트리에서 `rm -rf apriltag_v1` 하면 이 무시 폴더까지 지워지고, jm_mac 으로
`git checkout` 해도 **추적 파일만 복원되어 work_dirs 는 안 돌아온다**(맥 v1 work_dirs 가 이렇게 사라졌다, VM 에서 복구).
- 안전한 main 리빌드: 지우려는 폴더는 `git rm -r --cached apriltag_v1`(인덱스만) 로 빼고 작업 트리의 `rm -rf` 는
  피하거나, orphan 작업을 **별도 worktree/클론**에서 한다. 최소한 리빌드 전 `work_dirs` 를 백업하거나 VM 에 남겨 둔다.
- 로그가 없어졌으면 VM 에서 복구: `scp -r ubuntu-vm:~/krri/apriltag_v1/work_dirs apriltag_v1/`.

## CAN 제어 (v4 = 광운대 `calib/control.py` 원본을 `src/models/kwu/control.py` 로. 실차 확인 2026-09-07)
- 주행 프레임 0x1E3(중립 127): **byte2=전/후진**(전진 67, 후진 187), **byte1=제자리회전**(강도 **30** → 좌 157, 우 97). **byte4 는 포크 리프트** — 회전에 쓰면 포크가 올라간다(사고 주의).
- `src/models/control/frames.py` 가 출발 전 **동결 테이블**(6 동작)을 써넣고 8바이트를 전부 검사한다 — byte1/2 외 하나라도 중립이 아니면 출발 거부. `driver.py` 는 여섯 동작만 내보내고 `abort()` 뒤엔 어떤 명령도 안 나간다(Ctrl+C 즉시 정지 — 가짜 리그 0.001 s 확인, 실차 미확인).
- 실측: 직진 정속 0.284(우리 9/7)~0.290(광운대) m/s, 출발지연 ~1.0~1.5 s, 정지 관성 ~12~14 cm. 회전 정속은 강도 20 에서 8.2~8.8°/s(우리), 강도 30 에서 12.0°/s(광운대).
  회전 팔 A 는 **calibrate.py 동심원으로 한 번 재서 config `CAM_TO_ROT_CENTER_M` 에** — v2 의 1.46 m 은 폐기, 광운대 실측 0.68 m 은 그쪽 차 값.
- Dropbox `철기원…/코드/control_forklift_v2.py` 는 **옛 버그 버전**(rotate=byte4/5 → 포크 올라감). 실차엔 레포 v4 것만.

---

# 맥북(Apple Silicon) 위의 Ubuntu VM — 카메라 SDK·IMU·CAN 개발환경

macOS 에서는 librealsense 가 카메라를 못 연다(UVCAssistant 선점 + 2.56.5 IMU 크래시). 그래서 맥북 안에
**UTM + Ubuntu 24.04 arm64 VM** 을 두고 USB 장치(RealSense, Kvaser)를 VM 에 통째로 넘긴다.
VM 은 Jetson 과 같은 arm64 리눅스라 여기서 검증한 절차·코드가 Jetson 으로 그대로 간다.
2026-09-07 맥북 M2 / macOS 26.3 / UTM / Ubuntu 24.04.4 실측.

## 되는 것 / 안 되는 것
- 된다: 검출·자세, depth, IR, IMU, 프레임 메타데이터, bag 녹화, canlib(CAN 송수신), dry-run.
  실측: VM 이 카메라를 **USB 3.2** 로 받는다. 컬러 640x480/1280x720/1920x1080 모두 30 fps, depth 30,
  컬러+depth 동시 30, IMU(gyro 200Hz + accel 100Hz) OK, global_time 타임스탬프.
- RSUSB 백엔드의 버릇 둘 (Jetson 도 RSUSB 로 빌드하면 같다):
  1) **IMU 는 먼저 연 쪽이 갖는다.** 컬러를 먼저 열면 뒤에 여는 자이로가 'failed to set power state'.
     자이로 먼저, 컬러 나중이면 둘 다 된다 — run.py/live_pose 가 그렇게 연다.
  2) 컬러 actual_exposure/gain_level 메타데이터는 **자동노출 끈 때만** 온다. realsense_check 의 그 WARN·'커널 패치'는 V4L2 용이라 무시.
- 이 D435i IMU 는 BMI085 → accel 유효값 100/200/400 Hz(63/250 은 BMI055). imu_yaw 가 63/100/200/250 중 풀리는 첫 값을 고른다.
- **VM 실주행은 사용자 결정으로 허용(2026-09-20)** — 단 리스크가 실재한다(9/7 이상 사건이 VM 호스트 멈춤). 조건: 맥 절전 끔(`caffeinate -s`, 뚜껑 열기),
  허브 없이 USB 직결, 송신 데드맨, 루프 tick·CAN write 간격·자이로 gaps 기록, 사람이 제동 위치. Jetson/Windows 로 옮기면 이 조건은 불필요.

## VM 만들기 (한 번)
1. `brew install --cask utm`, Ubuntu 24.04 **desktop arm64** ISO(3.3 GB) 받아 sha256 확인.
2. UTM → 새 VM → **가상화** → Linux → ISO. Apple 가상화는 **체크 해제**(USB 전달은 QEMU 엔진만).
3. 메모리 6144 MB, CPU 4, 디스크 30 GB. 저장 후 편집 → **입력** → USB 지원 **3.0 (XHCI)**, 최대 공유 USB 4.
4. 설치: Erase disk(VM 디스크만), 사용자 `seojeongmin`, 자동 로그인. 재시작 후 GRUB 이 또 뜨면 "Boot from next volume",
   이후 툴바 CD 아이콘으로 ISO Eject.
5. VM 터미널에서 (맥 공개키 `~/.ssh/id_ed25519_vm.pub`, 맥 IP 는 VM 에서 192.168.64.1):
   ```bash
   sudo apt update && sudo apt install -y openssh-server curl git gh
   mkdir -p ~/.ssh && curl -s http://192.168.64.1:8000/mac.pub >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys
   sudo bash -c 'echo "seojeongmin ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/seojeongmin && chmod 440 /etc/sudoers.d/seojeongmin'
   hostname -I      # 보통 192.168.64.2
   ```
   (맥에서는 공개키 든 폴더에서 `python3 -m http.server 8000` 을 켜 둔다.)
6. 맥 `~/.ssh/config`: `Host ubuntu-vm / HostName 192.168.64.2 / User seojeongmin / IdentityFile ~/.ssh/id_ed25519_vm`.
   VS Code Remote-SSH 로 `ubuntu-vm` 붙어 `/home/seojeongmin/krri` 를 연다.

## 실행환경 설치 (SSH 로, 40~60분·대부분 librealsense 빌드)
```bash
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v2/tools/etc/setup_ubuntu_arm64.sh apt'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v2/tools/etc/setup_ubuntu_arm64.sh conda'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v2/tools/etc/setup_ubuntu_arm64.sh realsense'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v2/tools/etc/setup_ubuntu_arm64.sh pip'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v2/tools/etc/setup_ubuntu_arm64.sh can'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v2/tools/etc/setup_ubuntu_arm64.sh check'
```
GitHub 로그인이 없으면 리포는 맥에서 밀어 넣는다: `ssh ubuntu-vm 'git init -q --bare ~/krri.git'` → `git push ubuntu-vm:krri.git jm_mac:refs/heads/jm_mac` → VM 에서 clone.

## 장치 넘기기와 검증 (실험 때마다)
1. RealSense/Kvaser 를 맥에 **허브 없이 직결**. VM 창 툴바 **USB 아이콘** → 장치 체크 → VM 이 가져간다.
2. 확인:
   ```bash
   ssh ubuntu-vm 'lsusb | grep -i -E "intel|kvaser"'
   ssh ubuntu-vm 'cd ~/krri/apriltag_v2 && ~/miniforge3/envs/krri/bin/python tools/check/device_check.py --no-gui'   # 카메라+IMU
   ssh ubuntu-vm '~/miniforge3/envs/krri/bin/python -c "from canlib import canlib; print(canlib.getNumberOfChannels())"'
   ```
3. 실행 (VS Code Remote-SSH 터미널, sudo 불필요):
   ```bash
   cd ~/krri/apriltag_v2 && ~/miniforge3/envs/krri/bin/python tools/run.py --dry-run   # CAN 안 보냄
   cd ~/krri/apriltag_v2 && ~/miniforge3/envs/krri/bin/python tools/run.py             # 실주행
   ```
   SPACE 로 시작(터미널에서 직접 읽는다), Ctrl+C 로 비상정지 후 종료.
4. 화면(`--show`, live_pose): VM `~/.bashrc` 가 SSH 셸에 `DISPLAY=:0` 을 넣어 둬서(conda 단계가 넣음) 창이 **UTM VM 창**에 뜬다.
   맥 화면에 띄우려면 `ssh -X ubuntu-vm`(XQuartz).

## Jetson 으로 옮길 때
같은 스크립트를 같은 순서로. 다른 곳 셋: 커널 헤더(Jetson 은 nvidia-l4t, 스크립트가 분기), CUDA(있으면 켬, 분기),
USB 넘기기 없음(직접 꽂힌다). 화면 없는 Jetson 도 맥에서 SSH·Remote-SSH 로 똑같이 다룬다. USB 직결 주소 192.168.55.1.
