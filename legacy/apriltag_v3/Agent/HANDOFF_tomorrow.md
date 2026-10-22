# 내일(2026-09-22) 현장 — 이 문서만 보고 하면 된다

> 목표 순서: **장비 확인 → 측정(first_run) → 분석(analyze) → 도킹(dock)**.
> 새 코드가 이상하면 **v2 폴백 `tools/run.py`** 로 돌린다(맨 아래 "퇴로").
> 오늘 밤까지 나온 숫자·한계는 맨 끝 "가정값으로 도는 것" 에 전부 적어 뒀다.

파이썬은 VM/Jetson 둘 다 **`~/miniforge3/envs/krri/bin/python`**, 작업 폴더는
**`~/krri/apriltag_v3`**. 아래 줄은 복사해서 그대로 친다.

---

## 0. 시작 전 (5분)

```bash
# 맥이 자면 VM 이 멈춘다 — 뚜껑 열고
caffeinate -s &
# RealSense·Kvaser 를 맥에 **허브 없이 직결** → UTM 툴바 USB 아이콘에서 VM 으로 넘긴다
ssh ubuntu-vm 'lsusb | grep -i -E "intel|kvaser"'
ssh ubuntu-vm 'cd ~/krri/apriltag_v3 && ~/miniforge3/envs/krri/bin/python tools/check/device_check.py --no-gui'
ssh ubuntu-vm '~/miniforge3/envs/krri/bin/python -c "from canlib import canlib; print(canlib.getNumberOfChannels())"'
```

- **사람이 제동 위치에 선다.** 호스트 동결(FM8b)은 소프트웨어 대응이 없다 — 이게 유일한 대응이다.
- **포크는 우리가 건드리지 않는다.** 올리라고도 내리라고도 하지 않는다 — 있는 그대로 두면 된다.
  `byte4` 가 포크 리프트인데(2026-09-07 에 회전 명령이 포크를 올렸다) 우리 CAN 템플릿은
  **전부 byte4 = 127(중립)** 이고, `check_templates()` 가 출발 전에 그걸 검사해 아니면 출발을 거부한다.
- 리모컨 수신기를 끄거나, 켜 둘 거면 `first_run safety` 단계의 이중 송신원 항목을 반드시 본다.

---

## 1. 측정 — `first_run` (VS Code Remote-SSH 터미널에서)

```bash
cd ~/krri/apriltag_v3
~/miniforge3/envs/krri/bin/python tools/first_run.py all
```

- 중간에 끊겼으면 `--resume` 을 붙여 다음 단계부터: `... tools/first_run.py all --resume`
- 한 단계만: `... tools/first_run.py rotate`
- 순서 = `timing → safety → mount → rotate → forward → creep → tagcut → grid → oblique → search`
- **`timing` 이 게이트다**: TXACK 첫 10건 max < 20 ms ∧ 자이로 gaps = 0.
  미통과면 이후 단계는 기록만 남고 캘리브를 **안 만든다**.
- **`forward` 까지가 필수.** 여기까지 돌면 `dynamics_calib` 이 보장된다.
- `mount` 단계에서 줄자 숫자를 묻는다. **빈 줄로 넘기지 마라** — 아래 표의 절반이 여기서 나온다.
  - 카메라 높이(바닥→렌즈), 태그 중심 높이, 카메라 pitch, **roll**, **x_off(오른쪽 +)**,
    **카메라 yaw δ(광축이 차체 정면보다 반시계면 +)**, 카메라→접힌 포크 끝.
- 기록은 `work_dirs/first_run/<시각>_<mode>/` 에 `frame/imu/can/events.jsonl` + `config.json`.

---

### 현장 실험 리스트 — 어디 세우고, 몇 번 하고, 뭘 위한 것인가

번호로 하나씩 돌린다. 따로 돌려도 **한 세션 폴더에 쌓인다**(끝에 분석 한 번).
`python tools/first_run.py --list` 로 남은 것 확인.

태그는 벽에 붙어 있다. 아래 "거리" 는 **카메라 → 태그면**. 포크 끝은 카메라보다 1.52 m 앞이다.

| # | 단계 | 어디에 세우나 | 횟수 | 무엇을 얻나 | 대략 |
|---|---|---|---|---|---|
| 1 | timing | 3.5 m 정면 | 6 | 프레임 지연 L·ε, 출발 게이트 | 2분 |
| 2 | **safety** | **8 m** 정면, 앞 5 m 비움 | 20 | 차단되면 차가 서는가, 복구 순서 | 25분 |
| 3 | mount | 3.5 m — ①비스듬 ②정면 | 자리당 100프레임 | 줄자 6항목(카메라 높이·x_off·δ̂…) | 10분 |
| 4 | **rotate** | **3.5 m 와 6 m**, 둘 다 비스듬(tilt≥20°) | **60** | τ_r·**σ_θ**·**회전팔 A**·최소 회전각 | 15분 |
| 5 | **forward** | **5.5 m** — ①정면 ②사각(30°) | **32** | **τ_eff·정지 관성**·S(T)·명령 CV | 15분 |
| 6 | creep | **4.5 m** 정면 | **30** | **97 이 움직이나**·v₉₇ | 8분 |
| 7 | tagcut | 3.5 m 정면 | 1 | 태그 잘리는 거리 = 블라인드 길이 | 3분 |
| 8 | grid | 3.5 m, **옆으로 0 / 0.5 / 1 / 1.5 / 2 m** | 5자리 × 6흔들기 | **정면 heading 거짓말** → θ_min | 12분 |
| 9 | oblique | **5 m**, heading ≈25° | 2 | NEAR 진입 거리 | 5분 |
| 10 | search | 아무데나, **태그 안 보이는 방향** | 1 | 태그 찾는 회전이 되나 | 3분 |

**1~5 가 필수.** 전부 하면 1시간 半~2시간(자리 옮기는 시간 별도).

#### 자리 그림

```
        벽(태그)
   ════════■════════
            │
     2 m ───┤ ← 8 grid 5번 자리
   1.5 m ───┤
     1 m ───┤            3.5 m: 1 timing · 3 mount · 7 tagcut · 8 grid
   0.5 m ───┤                   4 rotate(첫 자리)
       0 ───┼──── 3.5 m ──── 4.5 m ──── 5.5 m ──── 6 m ──── 8 m
            │       ↑          ↑          ↑         ↑        ↑
       (도킹축)   위 4개     6 creep   5 forward  4 rotate  2 safety
                                                  (둘째)
                                    9 oblique = 5 m 에서 25° 비스듬
```

#### 단계별로 실제 무슨 일이 일어나나

**2 safety (8 m)** — 차가 저속(97)으로 기어가는 중에 CAN 을 일부러 끊는다. 4종류(heartbeat /
movement / control / 전부) × 3회. **차단된 4초 동안은 제어가 안 된다** — 사람이 제동 위치에 서야
하는 이유다. 시행마다 자동으로 8 m 로 되돌아가고, 그래도 가까워지면 2.32 m 에서 단계를 멈춘다.
차단마다 "차가 어떻게 됐나" 를 눈으로 보고 Enter.

**4 rotate (3.5 m → 6 m)** — 전부 **제자리 회전**이라 앞뒤로는 안 간다. 차 주변만 비우면 된다.
- 개루프 펄스 좌/우 × {1, 1.5, 2, 3}초 × 4회 = **32** → 회전 지연·가속도
- 5초 자이로 전용 × 2 → 200 Hz 원시 데이터
- 폐루프 15° × 6 → **σ_θ**(이게 δ_q 를 정한다)
- A 측정 × 10 → **3.5 m 에서 5회, 6 m 에서 5회.** 두 거리를 꼭 해야 자이로 스케일과
  회전팔 A 가 갈린다. 9/7 에 A=1.46 이 틀렸던 게 한 거리에서만 재서다.
- 소각 ±5° × 10 → 가장 작게 낼 수 있는 회전

**5 forward (5.5 m)** — 4초 전진(약 1.3 m) 후 자동 복귀를 20번. 정면 10 + 사각 10.
그다음 짧은 명령 {1.5, 2, 3}초 × 4회 = 12. 5.5 m 에서 시작하는 이유는 3.3 m 에서 태그가
잘리기 때문(4초 가면 4.2 m, 아직 보인다).

**8 grid (3.5 m, 옆으로 5자리)** — **여기만 원리가 다르다.**
차를 옆으로 옮길수록 태그를 비스듬히 보게 된다. 각 자리에서 차가 **±3° 만 살짝 흔들고**,
자이로가 잰 회전량과 카메라가 읽은 회전량을 비교한다.
```
자이로 3.77° · 카메라 3.49°  →  비 0.93  (카메라가 7% 틀림)
```
비가 1 에서 멀수록 그 각도에서 카메라를 못 믿는다는 뜻. **각도를 줄자로 잴 필요 없다** —
자리만 걸음으로 대충 옮기면 된다(비스듬한 정도는 카메라가 정확히 잰다).
관성으로 3° 를 못 맞춰도 상관없다 — 실제로 돈 만큼을 자이로가 재기 때문이다.

#### 시간이 모자라면 버리는 순서

1. **10 search → 9 oblique → 7 tagcut** — 버려도 내일 도킹은 돈다
2. **8 grid** — 버리면 "정면에서 heading 불신" 경계를 기본값 15° 로 쓴다
3. **6 creep** — 버리면 **97(저속)을 못 쓴다.** 67 로만 주행
4. **4 의 6 m 자리** — 버리면 **회전팔 A 를 못 구한다**(0 가정)

**절대 못 버리는 것**: 4의 폐루프 15°(σ_θ) · 5의 정속정지 20회(τ_eff) · 3 mount 줄자

## 2. 분석 — `analyze_first_run`

```bash
cd ~/krri/apriltag_v3
~/miniforge3/envs/krri/bin/python tools/analyze_first_run.py work_dirs/first_run/<그 폴더> --install
```

- `--install` 이 `config/` 에 캘리브 3종을 놓는다(= `dock.py` 가 읽는 자리):
  `config/timing_calib.json` · `config/perception_calib.json` · `config/dynamics_calib.json`
- 같이 나오는 `report.md` 를 **반드시 본다.** 맨 아래 "신설 필드" 표에서 `측정 부족` 인 줄이
  내일 가정값으로 도는 항목이다.
- 타이밍이 안 통과했으면 `--gate-failed` 를 붙여라 — 캘리브를 안 만들고 report 만 남긴다.

### 측정 → 캘리브 필드 (어느 숫자가 어디로 가나)

| 단계 | 재는 것 | 들어가는 자리 | 없으면 쓰는 가정값 |
|---|---|---|---|
| timing | L 중앙·p99 | `timing.L_ms.{median,p99}` | — (없으면 게이트 불통) |
| timing | ε (카메라↔자이로 지연) | `timing.epsilon_ms.value` | 0.0 |
| timing | Δ_FS · T_line · TXACK 오프셋 | `timing.{delta_fs_ms,t_line_us,txack_offset_ms}` | — |
| timing | 게이트 통과 여부 | `timing.gate.passed` | False → 실주행 거부 |
| forward | 셀별 τ_eff·σ_e·q95 | `dynamics.tau_eff_s.{67,97}_{fwd,bwd}.*` | **0.50 s** / σ 0.02 m |
| forward | 정속 v̂ | `dynamics.v_mps.67.value` | **0.28 m/s** |
| creep | 저속 정속 | `dynamics.v_mps.97.value` | **0.12 m/s** ★미측정 |
| creep | stiction·데드존 | `dynamics.stiction_ok` · `dynamics.deadzone_level` | None / None |
| rotate | 코스팅 시상수 | `dynamics.tau_r_s.{L,R}.value` | **0.18 s** |
| rotate | 회전 출발 지연 | `dynamics.tau_start_s.rot.value` | 0.85 s |
| forward/creep | **주행 출발 지연** ☆신설 | `dynamics.tau_start_s.fwd.value` | 1.00 s |
| rotate | **회전 가속·감속** ☆신설 | `dynamics.tau_r_s.{L,R}.{alpha_up_dps2,alpha_r_dps2}` | 없음 → 순수지연 항만 |
| forward | **S(T) 표**(명령시간→거리) ☆신설 | `dynamics.S_of_T.{67,97}` | 빈 표 → 다리 ≤1.0 m 로 쪼갬 |
| rotate | **폐루프 잔차 σ_θ** ☆신설 | `dynamics.rot_closed.{L,R}.{sigma_deg,mean_deg,n}` | **1.0°** |
| rotate | **최소 신뢰 회전각** ☆신설 | `dynamics.min_inc.turn_deg` (= 2σ_θ) | **2.0°** |
| forward/creep | **최소 신뢰 직진 증분** ☆신설 | `dynamics.min_inc.{fwd_67_m,fwd_97_m}` | **0.30 / 0.12 m** |
|  |  | ↳ `forward` 의 **짧은 명령 격자**(1.5/2.0/3.0 s)와 `creep` 펄스(1.0~2.5 s)에서만 나온다 | 격자를 건너뛰면 영영 null |
| rotate | **가시 한계 β** ☆신설 | `perception.static.beta_vis_deg.{L,R}` | **±28°** |
| forward | **제동 노즈다운 픽셀** ☆신설 | `perception.static.pitch_px` | **24 px** |
| rotate(--camera) | 회전팔 A · 자이로 스케일 s | `perception.dynamic.{A_m,gyro_scale}` | A 0.0 ± 0.5 / s 1.0 |
|  |  | ↳ \|A\| > 0.5 m 이거나 σ 를 못 내면 **value=null + why** 로 나간다(−7.55 m 같은 값이 '측정됨' 으로 들어가면 회전 때마다 초당 1 m 의 가짜 횡이동이 KF 에 실린다) | A 0.0 |
| grid | heading 바이어스 표 → θ_min | `perception.static.{heading_bias_deg,theta_min_deg}` | 표 없음 → b floor 1.5°(정면) / θ_min **15°** |
|  |  | ↳ θ_min = "**그 위에서** 바이어스가 허용치 아래" 인 첫 셀. 나쁜 셀이 하나도 없으면 **null**(0.0 을 유효값으로 안 쓴다 — 0.0 이면 정면 원뿔 게이트가 통째로 꺼진다) | 15° |
| grid/mount | 코너 잡음 σ_c · flip 비율 | `perception.static.{sigma_c_px,flip_rate}` | 0.30 px / — |
| **mount(줄자)** | 횡오프셋 ☆이관 | `perception.dynamic.x_off_m.value` | **0.0** |
| **mount(줄자)** | 카메라 yaw δ̂ ☆이관 | `perception.dynamic.cam_yaw_offset_deg.value` | **0.0** |
| **mount(줄자)** | 태그−카메라 높이차 ☆이관 | `perception.dynamic.h_tag_cam_m.value` | **1.10 m** |
| **mount(줄자)** | 카메라 roll ☆이관 | `perception.static.roll_deg.value` | 미측정 → σ 에 **19 mm**(=1°분) 추가 |
|  |  | ↳ **롤 보정은 아직 코드에 없다**(plan 1-2 미구현). 그래서 재 온 φ 는 고치는 데 못 쓰고 **h·\|sin φ\| 만큼을 σ 로 싣는다** — φ=0 이면 0, φ=2° 면 38 mm 라 자동으로 '판정불가' 가 된다 | — |
| tagcut | 태그 컷 거리·픽셀 여유 | `dynamics.tag_cut.{forward_m,margin_px}` | 3.3 m / 60 px |

☆ = 2026-09-21 에 새로 붙인 경로(예전 analyze 는 이 여섯을 안 옮겼다).

> **리허설로 `first_run --dry-run` 을 돌려도 config/ 는 안 더럽혀진다.** 가짜 소스로 만든
> 캘리브는 `fake_source: true` 라 `calib.load()` 가 거부하는데, 예전엔 그게 `--install` 로
> `config/` 에 깔려서 "파일은 있는데 dock 이 출발을 거부" 하는 상태가 됐다. 지금은 설치를
> 건너뛰고 그 사실을 화면에 찍는다.

---

## 3. 도킹 — `dock.py`

```bash
cd ~/krri/apriltag_v3

# (a) 먼저 아무것도 안 보내고 배선만 본다 (카메라·IMU 는 진짜)
~/miniforge3/envs/krri/bin/python tools/dock.py --no-can --show

# (b) 캘리브가 다 찼으면 그냥
~/miniforge3/envs/krri/bin/python tools/dock.py --show

# (c) 캘리브가 아직 없거나 게이트를 못 넘었으면 (= 내일 아침 첫 판)
~/miniforge3/envs/krri/bin/python tools/dock.py --assume-calib --final-anyway --show
```

- **SPACE** 로 출발, **Ctrl+C** 로 즉시 정지·종료.
- 기록은 `work_dirs/dock/<시각>/` 에 자동으로 남는다(`--no-record` 로 끌 수 있다).
- 쓸 만한 플래그: `--level {67,97}` · `--standoff 0.8`(더 멀리 선다) · `--dry-run`(가짜 리그) ·
  `--sim`(플랜트 폐루프) · `--force`(GLOBAL 전환 실패에도 강행).
  **`--standoff` 는 0.20 m 아래로 못 준다** — 음수면 포크 끝 목표가 태그면 안쪽이 된다(벽 충돌).

### 마지막 블라인드 구간이 어떻게 서나 (2026-09-21 에 고친 곳)

태그는 ~3.3 m 에서 화면 밖으로 나간다. 거기서 정지점 2.02 m 까지 **약 1.3 m 는 눈 감고** 간다.
그 구간은 시간만 보내므로 **`v_mps.97` 이 틀리면 그만큼 더 가거나 덜 간다.**

- 주행 예산을 **충돌점 기준**으로 잡는다: `(블라인드 진입 x̂) − 1.52 m − 0.15 m`.
  실제 주행은 v97 이 얼마든 이 값을 못 넘으므로 **포크 끝은 태그면 앞 15 cm 에서 반드시 선다.**
- 예산을 세는 속도는 v97 의 **상한**이다 — 실측이 있으면 `v97 × 1.3`, 없으면 **v67(0.28 m/s)**.
  그래서 `v_mps.97` 이 비어 있으면 구조적으로 **덜 가고 선다**(맥 시뮬: 목표 2.02 m 인데 2.2~2.8 m).
  그게 벽을 뚫는 것보다 낫다. **creep 단계에서 v97 을 재 오면 이 손해가 사라진다**(시뮬 2.01 m).
- 예산을 다 썼는데 장부로 남은 거리가 있으면 `FM4` 로 크게 찍고 `DONE_UNVERIFIED` 로 끝난다.
  화면에 `FINAL 주행 예산 … 소진` 이 뜨면 **줄자로 재고 v97 을 실측해 오라는 뜻**이다.

### 화면에 계속 뜨는 한 줄 읽는 법

```
FAR   execute   T0  d  4.14m  e_l -322mm(σ 83)  ψ -23.8°(σ1.0)  β +19.8°  v 0.00  Hl | 대각 다리 전진 0.38 m
└구역 └상태     └Tier └카메라→태그  └포크 끝 횡오차(1σ) └차 기울기(1σ)  └태그 화면각 └속도 └플래그 └다음 동작
```

- 플래그: `H`/`h` = heading 믿을 수 있나 · `L`/`l` = lateral 믿을 수 있나 ·
  `태그없음` · `자이로X` · `AMBIG`(PnP 두 해 구분 불가) · `STALE`(프레임이 150 ms 이상 묵음)
- **`e_l` 이 ±30 mm 안에 들고 σ 가 15 mm 아래**여야 정식으로 통과한다.
  내일은 σ ≈ 70~100 mm 라 **정식 통과가 안 난다** — 그래서 `--final-anyway` 가 필요하고,
  결과는 항상 `DONE_UNVERIFIED` 다. **줄자로 재고 그 숫자를 적어라.**

---

## 4. 뭐가 잘못되면 — 증상 → 할 일

| 화면에 보이는 것 | 뜻 | 할 일 |
|---|---|---|
| `ABORT (calib)` | 캘리브 없음. 기본은 실주행 거부 | `--assume-calib` 를 **직접** 준다(모르고 도는 일이 없게 일부러 막아 뒀다) |
| `캘리브 거부 사유: … 가짜 소스(dry-run)` | `config/` 에 dry-run 캘리브가 깔려 있다 | `config/*_calib.json` 3개를 지우고 **실측 세션**으로 `analyze … --install` 을 다시 |
| `캘리브 거부 사유: … 출발 게이트가 미통과` | `timing.gate.passed=false`(TXACK 미측정 또는 자이로 유실) | `first_run timing` 을 다시. 급하면 `--assume-calib` 로만 강행 |
| `캘리브 거부 사유: 프레임 지연 L p99 … ≥ stale` | 프레임이 150 ms 이상 묵는다 | `--show` 를 끄고, 1080p 대신 낮은 해상도. 그래도 크면 `timing.L_ms.p99` 가 stale 문턱을 자동으로 넓힌다 |
| `ABORT (sign_check)` | **배선 부호가 틀렸다** — 픽셀 경로와 PnP 경로가 같이 어긋났다 | 멈춰라. 코너 순서·왜곡 모델·`CAM_YAW_OFFSET_DEG` 배선을 본다. 다시 출발시키지 마라 |
| `ABORT (wrong_branch)` | 배선은 멀쩡한데 **PnP 2중해가 거짓 해로 잠겼다** | **배선을 고치지 마라.** 차를 좌우로 몇 도 비껴 세워(태그를 비스듬히 보게) 다시 출발 |
| `ABORT (envelope)` | 태그를 보면서는 축에 못 합류한다(시작이 너무 옆) | 차를 태그 쪽으로 더 돌려 세우고 다시. 8 m 에서 좌우 ≲2.8 m 안 |
| `ABORT (gyro_dead_blind)` | 블라인드 중 자이로가 죽었다 | 자이로를 다시 열어야 한다. `device_check` 부터 |
| `ABORT (no_response)` + `FM4` | 명령을 보냈는데 차가 안 움직였다(데드존/stiction) | 97 이 데드밴드 위인지 확인. `first_run creep` 다시 |
| `ABORT (quant_gap)` + "저속 회전 단 필요" | 고쳐야 할 양이 **최소 회전각보다 작다** | 오늘은 여기서 끝. 저속 회전 단(byte1 < 20)이 실차에 필요하다는 증거다 — 기록해서 가져와라 |
| `ABORT (no_convergence)` | Tier 1 재배치를 다 썼는데 판정이 안 선다 | σ 가 안 줄어든다는 뜻. 더 가까이서(3.5 m) 시작하거나 σ_θ·δ̂ 를 먼저 재라 |
| `ABORT (time_budget)` | 180 s 초과 | `config/control.py` 의 `DOCK_TIME_BUDGET_S` 는 사람이 정하는 값이다 |
| `FINAL 주행 예산 … 소진` + `FM4` | 블라인드를 **안전 상한까지만** 갔다(덜 갔다) | 정상이다. 줄자로 재고 `v_mps.97` 을 실측해 오면 다음 판부터 목표에 닿는다 |
| `블라인드 종료: 주행 예산이 …` | 같은 뜻 (더 낼 명령이 없다) | 위와 같다 |
| `정지 시간 캡 … 정지를 확인 못 했다`(FM8a) | IMU·카메라로 '멎음' 을 못 봤고 시간으로 넘어갔다 | 그 판은 **DONE 이 안 난다**(DONE_UNVERIFIED 고정). 자이로 원시 기록을 가져와라 |
| `Tier 1 후진은 byte2=187(전속)이다` | **저속 후진 단이 없다** | 정상 경고다. 후진은 한 번에 ≤0.3 m 로 묶여 있다. 사람이 뒤를 봐 줄 것 |
| `버스에 우리가 안 보낸 0x1E3 …`(FM11) | 리모컨·조이스틱이 같이 쏘고 있다 | 사람이 잡은 것으로 보고 **즉시 ABORT**. 리모컨을 끄고 다시 |
| `!! 가정값 사용 중: …` | 캘리브 항목이 비었다 | 그대로 돌아도 된다. 97 한정·다리 ≤1.0 m·회전 ≤15°·정지거리 ×1.5 가 자동으로 걸린다 |
| `!! 데드맨: …` | 판단 루프가 0.5 s 갱신 없음 → 강제 정지 | 호스트가 버벅인다. 화면(`--show`)을 끄고 다시 |
| `FM3` 왕복 경고가 자주 | 유령 오차를 쫓고 있다 | 기록을 가져와라. 멈출 필요는 없다(Tier 1 이 알아서 끊는다) |
| 태그를 아예 못 잡는다 | 노출·거리·태그 크기 | `--show` 로 보고, `--auto-exposure`, `config/detection.py` 의 `TAG_SIZE_M` 확인 |

**어느 경우든 ABORT 뒤에도 CAN 에는 stop 프레임이 계속 나간다**(버스 침묵 금지). 차는 서 있다.

---

## 5. 가정값으로 도는 것 (내일 실측 전까지)

| 항목 | 지금 쓰는 값 | 근거 / 위험 |
|---|---|---|
| `v_mps.97` (저속 전진 m/s) | 0.12 | **완전 미측정.** 데드밴드 위인지도 모른다. 그래서 다리를 1.0 m 로 자른다 |
| `tau_eff` (정지 시상수) | 0.50 s | 9/7 정지지연. 가정이면 정지거리를 ×1.5 해서 **일찍 선다** |
| `tau_r` (회전 코스팅) | 0.18 s | 9/7 관성 1.3~1.5° ÷ 7.3°/s. 가정이면 1회 회전 ≤15° |
| `rot_closed.σ_θ` | 1.0° | 9/7 폐루프 ±2°. **plan 은 ≤0.5° 를 요건**으로 본다 — 내일 재는 1순위 |
| 회전팔 `A` | 0.0 ± 0.5 m | 옛 1.46 m 는 **철회**. 실시간 도출 중 |
| `x_off`(카메라↔차체 중심) | 0.0 | mount 에서 줄자로. 빈 줄로 넘기면 계속 0 |
| `δ̂`(카메라 장착 yaw) | 0.0° | 태그 겨냥 목표가 **β\* = −δ̂** 다. 부호를 반대로 넣으면 δ 의 두 배만큼 틀어진 채 들어간다 |
| `roll` | 미측정 | **보정이 없다.** 미측정이면 19 mm(1°분), 재 왔으면 `h·\|sin φ\|` 를 σ 로 싣는다. φ 가 2° 면 38 mm 라 그 판은 자동으로 '판정불가' 가 된다 — 그게 맞다(안 고치는 바이어스를 0 으로 치면 wrong-DONE 이 난다) |
| 저속 **후진** 단 | **없다** | `MOVEMENT_TEMPLATES` 에 byte2=187(전속) 하나뿐이고 새 byte2 값은 실차 미검증이라 만들지 않았다. Tier 1 재배치 후진은 **≤0.30 m** 로 묶고 화면·로그에 '전속' 이라고 찍는다 |
| `θ_min`(정면 원뿔) | 15° | 이 안에서는 카메라 heading 을 **안 믿는다**. 태그를 화면 중앙에 두면 tilt ≈ 접근각이라 6 m·ℓ 0.8 m 에서 9° 뿐 → 거의 항상 정면 취급이다 |
| `beta_vis`(가시 한계) | ±28° | HFOV 69/2 − 태그 반각 |
| `pitch_px`(제동 노즈다운) | 24 px | plan 3-4 |
| 태그 컷 | 3.3 m | 9/7 18:14. 정지점 2.02 m 까지 **블라인드 약 1.3 m** |
| **탑재부 진입 깊이 `DOCK_DEPTH_M`** | **2.0 m** | ★미측정·보수 가정. `HEAD_TOL_DEG = 0.5°` 가 여기서 나왔다. 재면 완화 가능 |
| 차량 워치독 `T_wd` | 미측정 | 호스트 동결(FM8b)의 **유일한** 대응이 이것 — safety 단계에서 재라 |

### 내일 가져와야 할 숫자 (우선순위)
1. `byte2 = 97` 의 실제 m/s, 데드밴드, stiction
2. 회전 폐루프 잔차 σ_θ (L/R) — **0.5° 요건을 넘는지**
3. 회전팔 A 와 자이로 스케일 s (3.5 m·6 m 두 거리에서)
4. 줄자 4개: x_off · δ̂ · roll · 태그−카메라 높이차
5. 태그 컷 거리 실측 (블라인드가 정말 1.3 m 인가)
6. **탑재부 길이** (잴 수 있으면. 못 재면 2.0 m 가정 유지)

---

## 6. 퇴로 — 새 코드가 이상하면

```bash
cd ~/krri/apriltag_v3
~/miniforge3/envs/krri/bin/python tools/run.py --dry-run     # CAN 안 보냄
~/miniforge3/envs/krri/bin/python tools/run.py               # v2 방식(조준-전진) 실주행
```

`tools/run.py` 는 **건드리지 않았다.** 9/7 에 2회 중 1회 성공(21스텝)한 그 코드 그대로다.
v2 전용 상수만 `config/control.py` 에서 `src/models/control/control_from_pose.py` 로 옮겼고
값은 하나도 안 바꿨다.

---

## 7. 맥에서 미리 돌려 본 것 (2026-09-22 새벽, 하드웨어 없음)

```bash
python -m src.models.control.dock_fsm                          # 상태기계 자기검증 39항
python tools/analyze_first_run.py --selftest                   # 분석 자기검사(합성 데이터)
python tools/dock.py --sim  --assume-calib --final-anyway      # 끝까지 감 (done_dr)
python tools/dock.py --dry-run --assume-calib --final-anyway   # 가짜 리그로 배선 확인
python tools/first_run.py all --dry-run --yes --n 1 --root <임시폴더>   # 10 단계 전부
python tools/etc/test_dock.py --grid --n 5                     # 200회 몬테카를로
```

전부 통과했다(`dock_fsm` 39/39, `analyze --selftest` 통과, `test_dock` 실패 0 항).

200회 몬테카를로 (`--grid --n 5`, **캘리브 없음 = `--assume-calib` 상태**, 벽시계 87 s):

| | 값 | 뜻 |
|---|---|---|
| **wrong-DONE** | **0 / 200** | 하드 KPI. 못 미더운데 DONE 이라고 한 판이 한 번도 없다 |
| 거짓수용 · 계약 위반 | 0 · 0 | 수용식이 틀린 걸 통과시킨 적 0, 전이표·송신창구 위반 0 |
| 평행사변형 성공 | 13/200 (6%) · 포락선 안 13/160 (8%) | 허용 30 mm 안에 든 비율 |
| `\|e_l\|` 중앙 | 198 mm (포락선 안 168 mm) | 허용 30 mm |
| abort | 39/200 (20%) | envelope 14 · no_response 11 · wrong_branch 10 · time_budget 4 |
| 반전(FM3) | 37/200 = 0.19 /도킹 | plan 3-1 KPI "≤1" 안 |
| 부호 자가시험 · 계약 시험 · 고장주입 9종 | **전부 통과 (0 실패)** | |

> **`|e_l|` 가 전보다 커 보이는 건 블라인드를 덜 가고 서기 때문이다.** 포크 끝 오차는
> `ℓ + (1.52 + 남은 블라인드)·sin ψ` 라 **덜 갈수록 레버가 길어져 커진다.** 벽을 뚫던 예전
> 코드가 더 작은 숫자를 냈던 것이고, 그건 "가까이 갔다" 가 아니라 "지나갔다" 였다.
> `v_mps.97` 이 채워지면 레버가 다시 짧아진다.

참고로 **캘리브를 채워 넣고** 같은 200회를 돌리면(v97 0.12·σ_θ 0.4°·β_vis 26/27° 를 가정한
합성 캘리브) wrong-DONE 0 · 계약 위반 0 은 그대로이고, `lateral_valid False` 고장주입에서만
**반전이 도킹당 2회**로 KPI(≤1)를 넘었다 — β_vis 가 28° 보다 좁게 측정되면 재조준이 늘어난다.
**내일 `perception.static.beta_vis_deg` 가 28° 보다 작게 나오면 그 항목을 보고해 달라.**

### 시뮬에서 나온 것 중 내일 알고 있어야 할 것
- **`sign_check` 은 시뮬에서 0건이다.** 중단은 전부 `wrong_branch`(PnP 2중해가 거짓 해로
  잠김)·`envelope`·`no_response` 였다. 현장에서 `sign_check` 이 뜨면 그건 **진짜 배선 문제**다.
- 정식 수용식 통과는 **0/200** 이다 — σ(e_l) 이 허용치 절반(15 mm)보다 크기 때문이고,
  이건 코드로 줄일 수 없다(σ_θ·δ̂ 실측 + 레버 단축뿐). **그래서 `--final-anyway` 로 돌고
  줄자로 검증한다.**
- 종방향은 캘리브 없이는 `FWD_TOL_M = 0.10 m` 를 **크게** 넘는다(시뮬에서 2.2~2.8 m 에 섬).
  `v_mps.97` 이 채워지면 2.0 m 근처로 붙는다(시뮬 2.01 m).

---

## 8. 2026-09-22 새벽에 고친 것 (적대적 검토 뒤)

전 코드를 4렌즈로 훑어 41건이 나왔고 그중 안전·데이터사슬에 걸리는 것들을 고쳤다.
**현장 절차는 안 바뀐다.** 아래는 "화면에 다르게 보이는 것" 만 추렸다.

| 고친 것 | 전에는 | 지금은 |
|---|---|---|
| 블라인드 직진 거리 안전장치 | 시간만 보내서 v97 이 가정보다 빠르면 **포크 끝이 태그면 1.4 m 안쪽**까지 갔다 | 충돌점 기준 주행 예산. v97 이 0.12~0.30 m/s 어디든 **태그면 앞에서 선다**(시뮬 확인) |
| 블라인드 장부 | 명령을 **내기 전에** 계획값을 통째로 차감 → 끊겨도 '다 갔다' | 다리가 끝난 **뒤 실경과**로 차감 |
| `tau_start_s` | analyze 는 스칼라, 소비자 넷 중 셋이 다른 경로로 읽어 실측을 버렸다 | `tau_start_s.{fwd,rot}.value` 한 스키마 |
| 회전 α_up·α_r | analyze 가 안 만들어 영영 null | `tau_r_s.{L,R}.alpha_*_dps2` 로 나온다(펄스 격자도 {1,1.5,2,3} s 로) |
| `min_inc.fwd_67_m` · `S_of_T` | 명령길이가 하나뿐이라 구조적으로 null | `forward` 에 **짧은 명령 격자**(1.5/2.0/3.0 s)를 넣었다 |
| θ_min | 나쁜 셀이 없으면 **0.0** 이 저장돼 정면 원뿔 게이트가 꺼졌다 | 경계를 못 재면 null(→ 기본 15°) |
| 회전팔 A | −7.55 m 같은 값이 '측정됨' 으로 나갔다 | \|A\| > 0.5 m·σ 없음·통과기준 미달이면 null + 사유 |
| CAN 건강(FM9) | 세션 **누적 최대**라 한 번 튀면 영구 래치 → 180 s 제자리 | 최근 창 + 연속 3프레임 |
| 차단 시험(safety) | 예외·Ctrl+C 때 차단이 안 풀려 **stop 프레임까지 버려졌다** | try/finally + ChannelProbe 가 **stop 은 절대 안 버린다** |
| 데드맨 시험 | watchdog 과 겹쳐 13/20 확률로 "안 걸림" 오판 | trips 증가분으로 판정 |
| σ(e_l) | 같은 ψ 오차를 두 번 세어 7배 과대 → NEAR 가 영영 안 났다 | 실효 감도 `(lever − x̂)` 로 한 번만 |
| ψ̂ 를 모를 때 | 0 으로 치환해 "오차 0 · σ 0.5 mm" 를 발행하고 DONE 까지 갔다 | β 갱신 건너뜀 · σ NaN · **판정불가** |
| v̂ | 명령 prior 를 매 프레임 넣어 차가 멎어 있어도 +v97 → FM4 가 못 떴다 | prior σ 를 √N 으로 키운다. 움직임은 **카메라 x̂·자이로 ω** 로만 판정 |
| 롤 | 캘리브에 있기만 하면 σ 바닥을 치웠다(보정은 없는데) | 안 고치는 바이어스를 그대로 σ 로 싣는다 |
| `--standoff` | 음수를 주면 태그면 안쪽을 목표로 잡고 **성공**이라고 끝냈다 | 0.20 m 하한, 아래면 출발 거부 |
| 정지 판정(블라인드) | 인자 꼴이 안 맞아 **항상 False** → 전부 시간 캡 | IMU 원시행을 넘긴다. 캡으로 빠지면 **DONE 을 못 낸다** |
| `first_run --dry-run` | 가짜 캘리브를 `config/` 에 깔았다 | 안 깐다 |
| FM11(이중 송신원) | 감지자가 없었다 | 우리가 안 보낸 0x1E3 을 세서 즉시 ABORT |

**고쳤지만 완전히 못 없앤 것**
- `v_mps.97` 이 비어 있으면 블라인드를 **덜 가고 선다**(위 §3). 재 오면 없어진다.
- **저속 후진 단이 없다.** Tier 1 후진은 byte2=187(전속)·≤0.30 m 다. 새 byte2 값은 실차
  검증이 없어서 만들지 않았다 — 필요하면 내일 `can_pulse` 로 재고 오라.
- **롤 보정 자체는 아직 없다**(plan 1-2). 지금은 불확실도로만 다룬다.
