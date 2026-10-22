# G_code-B 인터페이스 계약 (경계면 확정판)

> 대상: **A(추정) · B(동역학) · C(상태기계) · D(시뮬)** 네 팀 + 통합 담당.
> 네 팀은 서로의 코드를 못 본다. **이 문서에 적힌 것만이 팀 사이를 건너간다.**
> 근거는 `plan.md`(단일 진실원) — 이 문서는 plan 의 **경계면 확정판**이고,
> plan 과 어긋나는 곳은 §0.2 정오표에 이유와 함께 적었다(그 항목은 이 문서가 이긴다).
> 코드 블록은 **그대로 붙여넣는 것**이 기준이다. 소유 팀 외에는 그 파일을 고치지 않는다.
> 작성: 2026-09-21. 실측 확인은 맥에서 실코드로 돌린 결과(§1.4)다.

---

## 0. 먼저 읽을 것

### 0.1 이 계약이 전제하는 것 (바꾸지 마라)
- 도킹 = **접힌 포크로 탑재부에 들어가 주차**. 팔레트 삽입 아님. 기준점 x_ref = **포크 끝**(카메라 앞 `CAM_TO_REF_M` 1.52 m).
- 성공 = `max(|e_l|, |e_l − DOCK_DEPTH_M·tan e_h|) ≤ LAT_TOL_M`(0.030), `DOCK_DEPTH_M` 2.0 은 **미측정 가정**.
- 사이드스텝·곡선주행 **없음**. stop-and-go **없음**(연속 추정). 카메라가 절대 기준, 자이로는 짧은 다리만.
- 가이드 = **대각 다리 → 태그 겨냥 회전 → 블라인드 직진**. 비용함수·N-step 롤아웃 **없음**.
- 구역 FAR→NEAR→FINAL **단조**. 모든 CAN 송신은 `SafeCanTx` 하나를 통해서만.
- **byte4 = 포크 리프트.** 주행·회전에 절대 쓰지 않는다. 쓰는 바이트는 byte1(조향/제자리회전)·byte2(전/후진)뿐.

### 0.2 이 문서가 plan.md 를 정정하는 곳 (정오표에 추가할 것)
| # | plan | 정정 | 이유 |
|---|---|---|---|
| E1 | 3-3 "태그 겨냥 회전(β_px = δ̂)" | **β\* = −δ̂** | `docking_state` 는 `heading = ψ_raw − δ`. 광축은 차체축보다 +δ(반시계). 차체를 태그에 맞추려면 태그가 광축 기준 −δ 에 있어야 한다. §1.4 에서 항등식으로 실측 확인(잔차 0.0003°). 부호를 반대로 쓰면 **δ 의 두 배만큼 틀어진 채** 블라인드로 들어간다 |
| E2 | 3-3 `F̂ = ℓ̂·x_ref/d_T + (d_T−x_ref)·r̂` | **e_l = ℓ̂ + (x_ref + s)·sin ψ̂** 하나로 통일 (§3.4) | r̂ 의 부호 규약이 plan 에 없다. β 규약(반시계 +)으로 r 을 쓰면 그 항은 **−**(d_T−x_ref)·r 이다. ψ̂ 한 변수로 쓰면 부호가 하나뿐이고 시뮬로 바로 검증된다 |
| E3 | 3-6 `σ_F = x_ref·σ_ψ,eff` | 레버는 **x_ref + s = d − x_stop** (§3.4) | 기준점은 정지까지 s 를 더 간다. x_ref 만 쓰면 레버를 1.52/2.8 = 54% 로 과소평가 → wrong-DONE 위험(하드 KPI 위반) |
| E4 | 2-6 "캘리브 부재 → 출발 거부" | 유지. 단 **사람이 `--assume-calib` 를 직접 준 경우**에만 가정값 + 97 강제 + 화면·로그 경고로 실행 (§5.4) | 내일 아침엔 세 파일이 전부 없다. 조용한 기본값 부활은 금지, 사람이 명시적으로 켜는 문은 허용 |
| E5 | 3-6 FINAL 진입 = 수용식 통과 | (a) 정식 통과 **또는** (b) `--final-anyway` 운영자 승인 → 결과는 항상 `DONE_UNVERIFIED` (§4.5) | σ_F 실측 예상 68 mm ≫ 15 mm(§3.4) 라 내일은 정식 통과가 안 난다. 승인 모드가 없으면 내일 실험 자체가 못 끝난다. **DONE 을 사칭하지 않으므로 wrong-DONE 0 은 지킨다** |
| E6 | — | `tools/etc/fake_rig.py` 의 렌더가 **화면에서 180° 뒤집혀 있다**(§7.4) | 실측. 시뮬의 β 부호·태그 컷 방향이 현실과 반대 → 폐루프 시뮬이 "잘 되는데 현장에서 반대로 가는" 전형적 사고를 만든다 |

---

## 1. 부호 규약 — **이 절이 틀리면 지게차가 반대로 간다**

### 1.1 한 문단 (외워라)
모든 각도의 **+ 는 반시계(위에서 봤을 때 좌회전)** 하나로 통일한다. `rotate_ccw`(byte1=**147**, 좌) = +, `rotate_cw`(byte1=**107**, 우) = −, 자이로 `GyroYaw.angle_deg` 도 + 가 반시계(`IMU_YAW_SIGN=+1`, 2026-09-06 실물 확정). 횡방향 `lateral`(ℓ) 은 **태그 좌표계의 x** 이고 그 축은 "**차가 태그를 볼 때의 왼쪽**" 을 가리킨다 — 즉 **ℓ > 0 ⟺ 태그가 화면 오른쪽에 보인다 ⟺ 차가 도킹축의 왼쪽에 있다.** 픽셀 방위각 `β_px`(`bearing_px_deg`) 는 반시계 + 이므로 **태그가 오른쪽에 보이면 β_px < 0**, 즉 **ℓ 과 β_px 는 같은 상황에서 부호가 반대**다(ℓ 은 위치, β 는 각도). `vertical` 은 카메라높이−태그높이라 **카메라가 태그보다 낮으면 음수**이고, 그때 태그는 **화면 위쪽**에 보이며 가까워지면 **위로** 잘린다. 종방향 `forward`(x) 는 카메라에서 태그면까지 남은 거리라 **항상 +**, 전진하면 줄고 속도 `v` 는 **다가갈 때 +**. 전진이 횡방향에 새는 양은 `dℓ/ds = sin ψ`(좌로 틀고 가면 ℓ 증가), 회전중심은 카메라보다 **뒤**에 있을 때 `A > 0` 이고 그때 `ℓ_pivot = ℓ − A·sin ψ`, `x_pivot = x + A·cos ψ`. 마지막으로 카메라 장착 yaw `δ`(`CAM_YAW_OFFSET_DEG`) 는 **`docking_state` 안에서 heading 에만 한 번 빠진다** — `β_px` 에는 안 빠져 있고, `lateral` 에도 안 들어간다.

### 1.2 표
| 기호 | 코드 | 단위 | + 의 뜻 | 반대편 |
|---|---|---|---|---|
| x | `docking_state["forward"]` | m | 카메라→태그면 남은 거리 (항상 +) | — |
| ℓ | `docking_state["lateral"]` | m | 태그가 **오른쪽**에 보임 = 차가 축의 **왼쪽** | ℓ<0 : 태그 왼쪽, 차는 축 오른쪽 |
| ψ | `docking_state["heading_deg"]` | ° | 차가 **반시계(좌)** 로 틀어짐. **δ 보정 완료본** | ψ<0 : 우로 틀어짐 |
| β | `bearing_px_deg(det, intr)` | ° | 태그가 화면 **왼쪽** = 좌회전해야 겨냥 | β<0 : 태그 오른쪽 |
| v | KF | m/s | 태그로 **접근**(x 감소) | v<0 : 후진 |
| ω | `GyroYaw.rate_dps` | °/s | **반시계** | — |
| A | `perception_calib.dynamic.A_m` | m | 회전중심이 카메라 **뒤** | A<0 : 앞 |
| δ | `CAM_YAW_OFFSET_DEG` | ° | 광축이 차체축보다 **반시계** | — |
| x_off | `perception_calib.dynamic.x_off_m` | m | 카메라가 차체 중심의 **오른쪽** (first_run 이 그렇게 묻는다) | **ℓ 과 방향이 반대다 — 주의** |
| e_l | 판정값 | m | 기준점(포크 끝)이 축의 **왼쪽**(ℓ 과 같은 축) | — |

### 1.3 못 박는 항등식 (전부 실측 확인, §1.4)
```
β = −atan2(ℓ, x)·180/π − ψ − δ                         … (I1)  잔차 0.0003°
ℓ_p(앞 a, 왼쪽 b) = ℓ + a·sin ψ + b·cos ψ               … (I2)
x_p(앞 a, 왼쪽 b) = x − a·cos ψ + b·sin ψ               … (I3)
회전중심      : a = −A, b = 0        → ℓ_piv = ℓ − A·sin ψ,  x_piv = x + A·cos ψ
차체 중심선   : a = 0,  b = +x_off   → ℓ_ctr = ℓ + x_off·cos ψ
포크 끝(기준점): a = CAM_TO_REF_M, b = +x_off
                → e_l = ℓ + CAM_TO_REF_M·sin ψ + x_off·cos ψ    … (I4)
태그 겨냥 목표: β* = −δ̂            (E1. δ̂ 없으면 0)          … (I5)
```
(I2)(I3) 의 b 부호가 x_off 와 반대인 이유: first_run 은 x_off 를 "오른쪽 +" 로 묻고, ℓ 축은 왼쪽이 + 다. **변환은 (I4) 한 줄로만 한다. 각자 다시 유도하지 마라.**

### 1.4 실측 근거 (2026-09-21, 맥, 실코드)
물리적으로 똑바로 선 tag36h11(id 1) 그림을 회전·거울 없이 화면 **오른쪽 위**에 붙이고 실검출기 → `pose_by_pnp` → `docking_state` 를 돌렸다:

| 태그를 그린 곳 | lateral | vertical | β_px | 읽는 법 |
|---|---|---|---|---|
| 오른쪽·위 | **+0.707** | **−0.348** | **−13.08** | 태그 오른쪽 → ℓ>0, β<0 ✔ / 태그 위 → vertical<0 ✔ |
| 왼쪽·위 | −0.696 | −0.352 | +13.02 | 좌우 뒤집으면 부호만 뒤집힘 ✔ |
| 오른쪽·아래 | +0.709 | +0.342 | −13.08 | 상하는 vertical 만 바뀜 ✔ |

(I1) 검정: δ = 0 / +3 / −3° 세 경우 모두 β − [−atan2(ℓ,x) − ψ − δ] = **+0.0003°**.
검출기 코너 순서는 `[BR, BL, TL, TR]`(화면 기준) 이고 `_object_points` 와 맞물려 태그축 x=화면왼쪽·y=위·z=카메라 반대쪽이 된다 — 위 표는 그 결과다.

---

## 2. `Estimate` — 추정기가 매 프레임 내놓는 것  (A팀 소유: `src/models/estimate/__init__.py`)

### 2.1 입력 한 줄 = `frame.jsonl` 한 줄
추정기는 **`FrameObs` 하나**만 먹는다. 필드는 `tools/first_run.py::_log_frame` 이 이미 쓰는 이름과 **완전히 같다** — 그래야 내일 로그로 오프라인 재생·시뮬·실주행이 한 코드로 돈다.

```python
@dataclass(frozen=True)
class FrameObs:
    """frame.jsonl 한 줄. 없는 값은 None (bool 은 False)."""
    seq: int                      # i — 프레임 일련번호
    stamps: object                # timing.FrameStamps (t_capture/t_arrival/stale/...)
    tag_seen: bool
    # docking_state (태그 좌표계, §1)
    lateral_m: float | None       # ℓ  + = 태그 오른쪽
    forward_m: float | None       # x
    vertical_m: float | None
    heading_deg: float | None     # ψ  δ 보정 완료본
    tilt_deg: float | None
    distance_m: float | None
    # 픽셀 관측
    beta_px_deg: float | None     # β  + = 태그 화면 왼쪽. **δ 미보정 생값**
    margin_px: float | None       # tag_edge_margin_px — 0 이 되는 순간 검출이 끊긴다
    tag_px: float | None
    center_px: tuple | None
    reproj_rms_px: float | None
    decision_margin: float | None
    hamming: int | None
    quality_ok: bool
    pnp2: dict | None             # pnp2_solutions() 그대로 {yaw_deg,pitch_deg,reproj_px,err_ratio,...}
    # 자이로
    gyro_deg: float | None        # 누적각 + = 반시계
    gyro_dps: float | None
    gyro_gaps: int
    gyro_alive: bool
    gyro_age_s: float | None
    # 명령 쪽 (전파 Q 스케줄용, plan 2-2)
    movement: str | None          # SafeCanTx.movement
    t_cmd_set: float | None
    t_cmd_tx: float | None

    @classmethod
    def from_row(cls, row: dict, stamps=None) -> "FrameObs": ...
```

### 2.2 출력
```python
@dataclass(frozen=True)
class Estimate:
    """추정기 출력. **프레임마다 무조건 하나** 나온다(태그가 안 보여도 예측만으로 낸다).

    부호는 contracts_B §1. 모르는 실수는 float('nan'), 모르는 bool 은 안전한 쪽(False).
    """
    # ── 시각 ────────────────────────────────────────────────────────────
    t: float                 # 이 추정이 말하는 시각 [s, host epoch] = 마지막 반영 프레임의 t_capture
    t_pub: float             # 이 객체를 만든 시각. t_pub − t 가 추정의 나이
    seq: int                 # 마지막으로 반영한 FrameObs.seq
    # ── 상태 (카메라 기준, 태그축 좌표) ───────────────────────────────────
    x_m: float               # 카메라 → 태그면 [m]  (+ 만 나온다)
    lat_m: float             # ℓ [m]  + = 태그가 오른쪽 = 차가 축의 왼쪽
    psi_deg: float           # ψ [°]  + = 반시계.  δ 보정 완료본
    v_mps: float             # + = 태그로 접근(x 감소)
    omega_dps: float         # + = 반시계
    beta_deg: float          # β [°] 이번 프레임 관측. 태그 없으면 nan
    beta_pred_deg: float     # β 의 KF 예측. **태그가 없어도 남는다**(REACQUIRE 가 쓴다)
    # ── 불확실도 (1σ, floor 적용 후) ─────────────────────────────────────
    sigma_x_m: float
    sigma_lat_m: float       # = max(σ̂_KF, d_h·σ_psi[rad])   ← 유령 정밀도 방지 floor
    sigma_psi_deg: float     # = √(σ_random² + b(d,tilt)² + (drift·anchor_age)²)
    sigma_beta_deg: float
    sigma_v_mps: float
    # ── 회전팔 (plan 1-1·3-2, 실시간 도출) ───────────────────────────────
    A_m: float               # + = 회전중심이 카메라 뒤
    sigma_A_m: float
    # ── 원관측 (게이트·로그) ─────────────────────────────────────────────
    tag_seen: bool
    margin_px: float
    tag_px: float
    tilt_deg: float
    reproj_px: float
    err_ratio: float         # pnp2 두 해 재투영 비 (1 에 가까우면 구분 불가)
    # ── 하드 플래그 (plan 1-7 → 3-1 하드 제약) ───────────────────────────
    heading_valid: bool      # "이 프레임으로 카메라 heading 을 **갱신**해도 되나" (plan 3-1 축소 정의)
    lateral_valid: bool      # sigma_lat_m ≤ LAT_TOL_M/2
    gyro_alive: bool
    ambiguous: bool          # PnP 2중해 구분 불가 (heading_valid=False 이나 x·β 는 계속 쓴다)
    stale: bool              # stamps.usable == False  (L > 150 ms·skew·역행)
    # ── 앵커·건강 ────────────────────────────────────────────────────────
    anchor_age_s: float      # 마지막 카메라 heading 갱신 이후 [s]
    gyro_gaps: int
    reinit_count: int        # 추정기 재초기화 횟수. ≥2 면 Tier 3 (plan 3-7)
    n_frames: int            # 초기화 이후 본 프레임 수
    degraded: str            # "" 또는 가정값 사용 사유 한 줄 (화면·로그에 그대로 뜬다)
    note: str
```

### 2.3 없을 때 값 / NaN 규칙
- 태그 미검출: `tag_seen=False`, `beta_deg/tilt_deg/tag_px/margin_px/reproj_px/err_ratio = nan`, `x_m/lat_m/psi_deg` 는 **예측값 유지**(σ 는 커진다), `heading_valid=lateral_valid=False`.
- 추정기 초기화 전: 전부 nan + `n_frames=0` + 모든 플래그 False.
- **게이트는 반드시 "긍정형"으로 쓴다**: `ok = (abs(e) + k*sigma <= T)`. NaN 은 모든 비교가 False 라 자동으로 "통과 안 함" 이 된다. `if e > T: 거부` 꼴로 쓰면 NaN 이 조용히 통과한다 — **금지**.
- `float('inf')` 는 쓰지 않는다(직렬화·비교에서 사고).

### 2.4 고정 헬퍼 (A팀이 같은 파일에 같이 둔다. 다른 팀은 **호출만**)
```python
def at_offset(est, ahead_m=0.0, left_m=0.0) -> tuple[float, float]:
    """(x, ℓ) 을 차체 어느 점 기준으로 옮긴다. §1.3 (I2)(I3)."""

def pivot(est) -> tuple[float, float]: ...                 # a = −A_m
def centerline(est, x_off_m) -> tuple[float, float]: ...    # b = +x_off  (부호 주의)
def fork_tip(est, cam_to_ref_m, x_off_m) -> tuple[float, float]: ...   # (I4)
def bearing_identity_residual(est, delta_deg) -> float:
    """(I1) 잔차 [°]. 0.2° 를 넘으면 부호·보정이 어긋난 것 — 즉시 정지 + Tier 3."""
```
`bearing_identity_residual` 은 **매 프레임 계산해 frame.jsonl 에 남긴다.** 이 한 줄이 부호 사고를 현장에서 즉시 잡는다.

### 2.5 A팀 금지사항
- `d = |t|`(3D 거리)를 종방향으로 쓰지 마라 — 2 m 에서 +28 cm 틀린다(plan 1-7).
- 잡음의 온라인 미분·EMA 적응 금지(plan 2-6). σ 는 calib 표 + 기하로만.
- `heading_sigma_deg`(몬테카를로) 를 프레임 루프에서 부르지 마라 — 오프라인 전용(plan 1-4).
- 정면(tilt < θ_min)에서 카메라 heading 으로 **갱신하지 마라**(`heading_valid=False`). 전파는 자이로로.

---

## 3. `Prediction` — 동역학이 내놓는 것  (B팀 소유: `src/models/dynamics/predict.py`)

### 3.1 자료형
```python
@dataclass(frozen=True)
class Prediction:
    kind: str          # "stop_forward" | "stop_rotate" | "leg_time" | "min_inc"
    value: float       # 정지거리 [m] / 정지각 [°] / 명령시간 [s] / 최소증분
    sigma: float       # 1σ. 모르면 nan
    q95: float         # 상측 95%. 모르면 value + 2*sigma, 그것도 못 내면 nan
    level: int         # 67 | 97
    cell: str          # "67_fwd" | "97_fwd" | "67_bwd" | "97_bwd" | "L" | "R"
    source: str        # "calib" | "assumed"      ← "assumed" 면 화면에 경고가 뜬다
    provisional: bool  # 캘리브가 provisional 이거나 n 이 모자람
    note: str
```

### 3.2 API
```python
class Dynamics:
    def __init__(self, calib, *, log=None): ...      # calib = utils.calib.load("dynamics")

    # 상태
    @property
    def degraded(self) -> bool: ...        # 가정값을 하나라도 썼다
    @property
    def force_slow(self) -> bool: ...      # True 면 전진은 forward_slow 만 (§5.4)
    def report(self, log=print) -> None: ...   # 어떤 값이 calib 이고 어떤 게 가정인지 한 화면

    # 직진
    def stop_distance(self, v_mps, level=67, direction="fwd", dt_ctrl_s=0.0) -> Prediction:
        """D̂ = τ_eff·v̂ + v̂·Δt_ctrl/2   (plan 2-1 m=0, 2-2 제어주기 반영)"""
    def should_stop_forward(self, x_rem_m, v_mps, level, dt_ctrl_s) -> bool:
        """x_rem ≤ D̂ 이면 True. x_rem 은 **카메라 forward 기준**(A 무관, plan 2-1)."""
    def command_time_for(self, dist_m, level) -> Prediction:
        """블라인드·짧은 다리용 명령시간. S(T) 표가 있으면 표, 없으면 fwd_time_model + 경고."""

    # 회전 (plan 2-4)
    def should_stop_rotate(self, theta_rem_deg, omega_dps, side, accelerating) -> bool:
        """θ_rem ≤ ω_pred·τ_r + [가속중]·½·α_up·τ_r² + ω_pred²/(2·α_r)
           ω_pred = ω̂ + [가속중]·α_up·τ_r.   α_r·α_up 이 없으면 그 항은 0(순수지연만)."""
    def rotate_timeout_s(self, deg, omega_dps) -> float: ...   # ω̂ 기준 예상 ×2 (코드 내부 배수)

    # 최소 신뢰 증분 · 데드밴드
    def min_forward_m(self, level) -> float: ...     # δ_x
    def min_turn_deg(self, side) -> float: ...       # θ_min,inc
    def deadband_level(self) -> int | None: ...      # 움직이기 시작하는 byte2 편향. 모르면 None

    # 정지 판별 2모드 (plan 2-3). 문턱은 **매 정지 실시간 도출** — config 아님
    def settle_visible(self, v_hist, omega_hist) -> bool: ...
    def settle_blind(self, imu_window) -> bool: ...
    def settle_timeout_s(self, v_mps, level) -> float: ...   # τ_d,q95 + v̂/a + 0.5 (코드 내부)
```

### 3.3 규칙
- **`should_stop_forward` 의 x_rem 은 카메라 기준**이다. 기준점(포크 끝)으로 환산하지 마라 — 정지식은 카메라 forward 로 세워져 있고 A·x_off 와 무관해야 한다(plan 3-2).
- 회전 정지 판정은 **IMU 콜백 주기**로 평가한다(8°/s × 60 ms = 0.5° 지터 회피, plan 2-4). 단 CAN 상태를 바꾸는 건 `SafeCanTx` 한 곳뿐이다(plan 3-1) — IMU 쪽은 **stop 요청만** 올린다.
- `rot_control.rotate_to()` 는 **B 층 실행경로에서 쓰지 않는다**(ROT_LEAD_DEG·ROT_SETTLE_* 를 읽는 옛 규칙). 표시·dry-run 참고용으로만 남긴다.
- 적응·학습 금지(plan 2-6). 값은 calib 로드 시점에 고정. 잔차는 로그·경보만.

### 3.4 기준점 최종 횡오차 — **두 팀이 공유하는 식이므로 여기서 못 박는다**
```
블라인드 다리를 s [m] 갈 때, 정지 시점 기준점(포크 끝) 횡오차:

    e_l = ℓ̂ + (CAM_TO_REF_M + s)·sin ψ̂ + x_off·cos ψ̂            … (F1)   [E2]

    σ(e_l)² = σ_ℓ² + ((CAM_TO_REF_M + s)·σ_ψ[rad])²              … (F2)   [E3]

여기서 ψ̂ 는 **다리를 시작하는 순간의 heading**, s 는 그 다리의 주행거리.
'태그 겨냥' 전략은 ψ̂ 를 고르는 방법일 뿐이고 식은 하나다.
태그를 겨냥해(β = β\* = −δ̂) 출발하면 (F1) 은 ℓ̂·x_stop/d − (d − x_stop)·r 로 정리된다
(r = 회전 후 남은 방위각 잔차, 반시계 +; **plan 3-3 의 +(d−x_ref)·r̂ 는 부호가 반대**).

수용식(T-VERIFY, plan 3-3):   |e_l| + k·σ(e_l) ≤ LAT_TOL_M   ∧   |ψ̂| + k·σ_ψ ≤ HEAD_TOL_DEG
성공 판정(plan 4-3):          max(|e_l|, |e_l − DOCK_DEPTH_M·tan e_h|) ≤ LAT_TOL_M
```
- `k = 2`(코드 내부 상수. config 아님).
- σ_ψ 에 들어가는 것: **겨냥으로 출발하면** σ_β ⊕ σ_θ(회전 잔차) ⊕ σ_δ, **축 정렬로 출발하면** σ_psi_deg.
- **수치 감각**(내일 예상): d 3.3 m, x_stop 0.5 m, σ_θ 1.0°, σ_δ 1.0° → σ(e_l) ≈ 2.8 m × 0.024 rad ≈ **68 mm ≫ T_l/2 = 15 mm**. 그래서 내일은 NEAR 정식 진입이 안 난다(§4.5 E5 가 그래서 있다). 이 숫자가 줄어드는 길은 **σ_θ·σ_δ 실측 + 레버 단축(근거리 태그)** 뿐이다 — 코드로는 못 줄인다.

---

## 4. 상태기계  (C팀 소유: `src/models/control/dock_fsm.py`, `failure.py`)

### 4.1 enum (그대로 붙여넣기)
```python
from enum import Enum, IntEnum

class Phase(Enum):
    IDLE = "idle"                 # 아직 출발 안 함(SPACE 대기·게이트 검사)
    OBSERVE = "observe"           # 프레임을 모아 판단 가능 상태를 만든다
    DECIDE = "decide"             # 명령을 고르고 **래치**한다 (명령 결정은 여기서만)
    EXECUTE = "execute"           # 명령 유지. 여기서 낼 수 있는 새 명령은 stop 뿐
    STOPPING = "stopping"         # stop 프레임 송신, 코스팅 중
    SETTLE = "settle"             # 실제로 멎었는지 **판정**(가시 v̂ / 블라인드 IMU)
    VERIFY = "verify"             # 정지 상태에서 30프레임 확인 + 구역 전환 판정
    REACQUIRE = "reacquire"       # 태그 실종 — 예측 β 방향으로 1회 회전(DECIDE 경유)
    FALLBACK = "fallback"         # AMBIGUOUS 지속 — oblique 로 재획득(DECIDE 경유)
    DONE = "done"                 # 관측으로 확인된 성공
    DONE_UNVERIFIED = "done_dr"   # 정지했으나 카메라로 확인 못 함(추측항법) — 기본값
    ABORT = "abort"               # stop 프레임 + heartbeat 지속 송신 + 사람 호출

class Zone(IntEnum):              # 단조. 역행 금지(FM7)
    FAR = 0
    NEAR = 1
    FINAL = 2

class Tier(IntEnum):
    NONE = 0
    WAIT = 1          # Tier 0 — σ 가 아직 줄고 있다. 그대로 관측
    REPOSITION = 2    # Tier 1 — 관측성 개선 동작
    CAUSE = 3         # Tier 2 — 원인 분기 1회
    ABORT = 4         # Tier 3 — 중단

class Primitive(Enum):
    NONE = "none"
    ROTATE = "rotate"                 # 제자리 회전. magnitude [°] + = 반시계
    FORWARD = "forward"               # 태그 보며 전진(67 또는 97)
    FORWARD_BLIND = "forward_blind"   # 태그 컷 뒤 마지막 직진(97 고정)
    BACKWARD = "backward"             # 재접근 후진(97 고정)
    DOGLEG = "dogleg"                 # 포락선 밖 FAR 한정 1회(97, ≤10 s)
    HOLD = "hold"                     # 아무것도 안 하고 관측만

class Movement(str, Enum):            # 값은 can_tx.SAFE_MOVEMENTS 문자열과 **완전히 같다**
    STOP = "stop"
    FORWARD = "forward"               # byte2 = 67
    FORWARD_SLOW = "forward_slow"     # byte2 = 97
    BACKWARD = "backward"             # byte2 = 187
    ROTATE_CCW = "rotate_ccw"         # byte1 = 147  (좌, +)
    ROTATE_CW = "rotate_cw"           # byte1 = 107  (우, −)

class StopReason(Enum):               # EXECUTE 를 끝낸 이유 (전부 can.jsonl 에 남는다)
    TARGET = "target"                 # 종료식 충족(정상)
    STALE = "stale"                   # 프레임이 150 ms 이상 묵음
    GYRO_DEAD = "gyro_dead"
    TAG_CUT = "tag_cut"               # 가시성 픽셀 여유 술어
    HARD_CAP = "hard_cap"             # 예측 정지점이 안전선을 넘음
    WRONG_WAY = "wrong_way"           # 회전이 반대로
    TIMEOUT = "timeout"
    DEADMAN = "deadman"               # SafeCanTx 가 먼저 끊음
    NO_RESPONSE = "no_response"       # 명령 창 안에 움직임이 없음(FM4)
    OPERATOR = "operator"

class AbortReason(Enum):              # Tier 3 집합 (plan 3-7)
    CALIB = "calib"                   # 캘리브 부재·거부
    GYRO_DEAD_BLIND = "gyro_dead_blind"
    REINIT_TWICE = "reinit_twice"     # 추정기 재초기화 ≥2
    ENVELOPE = "envelope"             # 시작 포락선 밖인데 dog-leg 로도 못 들어옴
    NO_RESPONSE = "no_response"
    HARD_CAP = "hard_cap"
    TIME_BUDGET = "time_budget"
    QUANT_GAP = "quant_gap"           # 양자화 공백 — "저속 회전 단 필요" 로그 동반
    SIGN_CHECK = "sign_check"         # (I1) 잔차 > 0.2° — 부호가 어긋났다
    CAN_FAULT = "can_fault"
    OPERATOR = "operator"

class FailureCode(Enum):              # 로그 코드 (plan 4-4). 감지되면 events.jsonl 에 남긴다
    FM1_WRONG_BRANCH = "FM1"; FM2_FALSE_CONF = "FM2"; FM3_PING_PONG = "FM3"
    FM4_DEADZONE = "FM4";     FM5_ERR_MIX = "FM5";    FM6_WAIT_DEADLOCK = "FM6"
    FM7_ZONE_CHATTER = "FM7"; FM8A_THREAD_LAG = "FM8a"; FM8B_HOST_FREEZE = "FM8b"
    FM9_CAN = "FM9";          FM10_WRONG_DONE = "FM10"; FM11_DUAL_SOURCE = "FM11"
```

### 4.2 래치되는 명령
```python
@dataclass(frozen=True)
class Command:
    primitive: Primitive
    movement: str            # Movement 값. SafeCanTx.set_movement 에 그대로 들어간다
    magnitude: float         # ROTATE [°] (+반시계) / 직진 [m] (+전진, 후진도 +로 주고 movement 로 방향)
    end_rule: str            # "camera_x" | "gyro_angle" | "time"
    end_value: float         # 목표 x_m [m] / 목표 Δψ [°] / 명령시간 [s]
    level: int               # 67 | 97
    expect: object           # Prediction (정지거리·정지각). 사후 잔차 계산에 쓴다
    timeout_s: float
    zone: Zone
    tier: Tier
    est_seq: int             # 어느 추정으로 결정했나
    t_decide: float
    why: str                 # 사람이 읽는 한 줄. events.jsonl 에 그대로
```
**`DECIDE → EXECUTE` 전이에서만 `set_movement()` 를 부른다.** 그 뒤로는 `feed()` 와 `stop()` 만.

### 4.3 허용 전이표
```python
ALLOWED = {
    Phase.IDLE:      {Phase.OBSERVE, Phase.ABORT},
    Phase.OBSERVE:   {Phase.DECIDE, Phase.REACQUIRE, Phase.FALLBACK, Phase.ABORT, Phase.OBSERVE},
    Phase.DECIDE:    {Phase.EXECUTE, Phase.VERIFY, Phase.OBSERVE, Phase.ABORT},
    Phase.EXECUTE:   {Phase.STOPPING, Phase.ABORT},
    Phase.STOPPING:  {Phase.SETTLE, Phase.ABORT},
    Phase.SETTLE:    {Phase.VERIFY, Phase.ABORT},
    Phase.VERIFY:    {Phase.OBSERVE, Phase.DONE, Phase.DONE_UNVERIFIED, Phase.ABORT},
    Phase.REACQUIRE: {Phase.DECIDE, Phase.OBSERVE, Phase.ABORT},
    Phase.FALLBACK:  {Phase.DECIDE, Phase.OBSERVE, Phase.ABORT},
    Phase.DONE: set(), Phase.DONE_UNVERIFIED: set(), Phase.ABORT: set(),
}
```
| from → to | 조건(guard) | 동작 |
|---|---|---|
| IDLE→OBSERVE | 캘리브 게이트 통과(또는 `--assume-calib`) ∧ `check_templates()` 통과 ∧ 컬러·자이로 GLOBAL ∧ 사람이 SPACE | 자이로 `calibrate()`, `snapshot()` |
| OBSERVE→OBSERVE | Tier 0 WAIT: 판정불가인데 σ 가 아직 줄고 있다 | 대기(캡은 코드 내부). 캡 초과 → Tier 1 |
| OBSERVE→DECIDE | `¬stale ∧ gyro_alive ∧ ¬ambiguous ∧ n_frames ≥ 최소(코드 내부) ∧ 직전 프리미티브 사후 일관성 ⓪ 통과 ∧ (I1) 잔차 ≤ 0.2°` | — |
| OBSERVE→REACQUIRE | `¬tag_seen` 이 일정 시간 지속 ∧ Zone ≠ FINAL | — |
| OBSERVE→FALLBACK | `ambiguous` 가 타임아웃까지 지속 | — |
| DECIDE→EXECUTE | 유효한 `Command` 를 만들었다 | **`set_movement()` (유일한 자리)** |
| DECIDE→VERIFY | 수용식 통과 — 움직일 필요 없음 | 명령 없음 |
| DECIDE→OBSERVE | 판정불가·양자화 공백 → Tier 처리 | 명령 없음 |
| EXECUTE→STOPPING | `StopReason` 하나라도 참 | `stop(why=reason)` |
| STOPPING→SETTLE | stop payload 가 실제로 나갔다(TXACK 또는 write 확인) | — |
| SETTLE→VERIFY | `settle_visible()` 또는 `settle_blind()` 또는 시간 캡 | 30프레임 `measure` |
| SETTLE→ABORT | 감속 전이 없이 τ_d,q95 경과(응답 부재) | 수동전환 |
| VERIFY→OBSERVE | 다음 다리가 필요 | 구역 전환 판정은 **여기서만** |
| VERIFY→DONE | 수용식 통과 ∧ **관측으로 확인**(기준점 pose 를 카메라가 봄) | — |
| VERIFY→DONE_UNVERIFIED | 수용식 통과이나 최종 pose 를 카메라가 못 봄(블라인드) | 줄자 검증 요청 로그 |
| REACQUIRE→DECIDE | 예측 β 방향으로 ≤ 반화각 회전 1회 계획 | 1회만. 2회째는 Tier 3 |
| FALLBACK→DECIDE | oblique 확보 동작 1회 계획 | 재획득 후 30프레임 다수결 전엔 NEAR 결정 금지 |

### 4.4 구역
```
FAR   : |ℓ̂| > k·σ_lat 로 **방향만** 유의. 67 허용. 30 mm 급 lateral/heading 판정 금지
NEAR  : σ(e_l) ≤ LAT_TOL_M/2  (§3.4 F2). heading_valid 인 oblique 다리에서만 진입
FINAL : "태그 겨냥 회전 완료 + 수용식 통과" **이벤트**로만 진입. 회전 전면 금지, 97 고정,
        heading 은 자이로 짧은 다리, gyro dead → 즉시 stop·ABORT
전환은 VERIFY(정지)에서만. 역행 금지 — 되돌리고 싶으면 Tier 1 이벤트로 기록.
```

### 4.5 FINAL 진입 두 문 (E5)
1. **정식**: §3.4 수용식 통과 → 결과는 관측 가능하면 `DONE`, 아니면 `DONE_UNVERIFIED`.
2. **운영자 승인**(`--final-anyway`): σ 가 커서 정식 통과가 안 날 때 사람이 명시적으로 켠다.
   - `events.jsonl` 에 `final_unproven` + 그 시점 (e_l, σ) 기록, 화면에 큰 경고.
   - 결과는 **무조건 `DONE_UNVERIFIED`**. 이 모드에서 `DONE` 을 내면 계약 위반(FM10).
   - 97 강제 + 블라인드 다리 길이 상한 유지.

### 4.6 Tier (plan 3-7)
| Tier | 트리거 | 동작 | 한도 |
|---|---|---|---|
| 0 WAIT | 판정불가 ∧ dσ/dt < 0 | 관측 지속 | 코드 내부 캡. σ 가 floor 지배면 즉시 Tier 1(FM6) |
| 1 REPOSITION | 판정불가 ∧ σ 정체 / 구역 역행 요구 | 관측성 개선 동작(원거리→대각 진행, 정면→자이로 앵커) | `σ_ψ(t)·(CAM_TO_REF+s) ≤ T_l/2` 가 깨지는 시각. 고정 60 s 금지 |
| 2 CAUSE | 원인별 1회 | 수용식 초과→97 재접근 / 태그 실종→REACQUIRE / 블라인드 중 Δψ 과다→stop·후진·재VERIFY / **양자화 공백→즉시 Tier 3 + "저속 회전 단 필요" 로그** | 원인당 1회 |
| 3 ABORT | `AbortReason` 집합 | **stop 프레임 + heartbeat 지속 송신**(버스 침묵 금지) + 사람 호출 | 종단 |

### 4.7 C팀 금지사항
- `EXECUTE` 안에서 stop 외의 명령 금지. 360° 탐색·FINAL 중 후진 복구 금지(v2 `next_set3_step`·`recover_backup` 폐기).
- 구역 역행 금지. "hold" 를 시간대기로 구현 금지(`SETTLE_SEC` 방식 폐기 — 판정으로 끝낸다).
- `config/control.py` 의 `AIM_*`·`CAM_TO_PIVOT_M`·`TAG_CUT_MARGIN_PX`·`MAX_STEPS`·`HOLD_MAX_CONSEC`·`STEP_M`·`FWD_SAFETY`·`SETTLE_SEC`·`ROT_LEAD_DEG`·`ROT_SETTLE_*`·`SEARCH_*` **읽지 마라**(전부 삭제 대상). B 층이 읽는 config 는 §8 목록이 전부다.

---

## 5. 캘리브 — B 층이 읽는 필드 전부

읽기는 `utils.calib.load(kind).get("점.경로", 기본값)` **하나로만**. 파일을 직접 열지 마라.
"내일" 열 = 2026-09-22 `first_run` + `analyze_first_run` 로 **실제로 채워지는가**(코드 확인 결과).

### 5.1 `dynamics_calib.json`
| 경로 | 단위·부호 | 내일 | 없을 때 가정값 | 근거 / 강등 |
|---|---|---|---|---|
| `tau_eff_s.67_fwd.value` | s | ✔ | **0.50** | 9/7 정지지연 0.5 s·관성 12~14 cm ÷ 0.28~0.30 m/s. 가정이면 D̂ ×1.5 + 97 강제 |
| `tau_eff_s.67_fwd.sigma_e_m` | m | ✔ | **0.02** | plan 2-7 통과기준 |
| `tau_eff_s.67_fwd.q95_m` | m | ✔ | value + 2σ | — |
| `tau_eff_s.97_fwd.*` | — | 창(creep)이 돌면 ✔ | 67 값 재사용 | 같은 제동 |
| `tau_eff_s.*_bwd.*` | — | ✘(후진 단계 없음) | fwd ×1.2 | 후진 다리는 **≤0.3 m** 로 제한 |
| `v_mps.67.value` | m/s | ✔ | **0.28** | 9/7 실측 0.28~0.30 |
| `v_mps.97.value` | m/s | creep 돌면 ✔ | **0.12** | 미측정. **없어도 97 을 유지**하고 블라인드 다리는 시간캡 + IMU 정지판정으로 닫는다. ⚠ `config/control.py` 주석의 "없으면 67 로 떨어진다" 는 **반대 방향(더 위험)** — 정정 필요 |
| `tau_r_s.L.value` / `.R.value` | s | ✔ | **0.18** | plan 2-4 임시값(9/7 관성 1.3~1.5° ÷ 7.3°/s). 가정이면 1회 회전 ≤15° |
| `tau_start_s` | s | ✔ | **0.85**(회전) / **1.0**(주행) | 9/7. 무응답 워치독에만 |
| `deadzone_level` | byte2 편향 | sweep 시 ✔ | None → 97·20 그대로 | — |
| `stiction_ok` | bool | creep 시 ✔ | None | None 이면 97 출발 무응답 감시 필수(FM4) |
| `S_of_T.67` / `.97` | [{T,median,cv,n}] | ✘(analyze 미구현) | `fwd_time_model`(T0 0.5069·T1 2.0357·A 0.1396, **byte2=67 기준**) | 블라인드 다리 ≤1.0 m 로 쪼개고 재관측 |
| `tag_cut.forward_m` | m | tagcut 시 ✔ | **3.3** | 9/7 18:14 |
| `tag_cut.margin_px` | px | tagcut 시 ✔ | **60** | 현 `TAG_CUT_MARGIN_PX` |
| `gyro_bias_dps` | °/s | ✘ | 런타임 `calibrate()` | 온도의존이라 파일로 안 쓴다 |
| **신설** `rot_closed.L/R.sigma_deg`,`.mean_deg`,`.n` | ° | 데이터 ✔ / analyze ✘ | σ **1.0**, mean 0 | 9/7 폐루프 ±2°. `min_turn_deg = max(2σ, 2.0°)` |
| **신설** `min_inc.fwd_67_m` / `.fwd_97_m` / `.turn_deg` | m,° | ✘ | **0.30 / 0.12 / 2.0** | plan 2-1(97 ≈0.12 m)·2-4 |

### 5.2 `perception_calib.json`
| 경로 | 단위·부호 | 내일 | 없을 때 가정값 | 근거 / 강등 |
|---|---|---|---|---|
| `static.theta_min_deg.value` | ° | grid 시 ✔ | **15.0** | plan 1-3. tilt < θ_min 이면 `heading_valid=False` |
| `static.heading_bias_deg` | [{truth,measured,bias,tilt,d,n}] | grid 시 ✔ | 빈 배열 → b floor **1.0°**(정면 **1.5°**) | 9/7 정면쌍 +1.1~1.8°. σ_psi 의 floor |
| `static.sigma_c_px.value` | px | ✔ | **0.30** | plan 1-4 주행 ≈0.3 px(정지 0.07~0.13) |
| `static.flip_rate.value` | 0~1 | ✔ | None | 로그만 |
| `static.roll_deg` | ° | ✘ | **0.0** | 롤 1°=19 mm. 0 가정 시 σ_lat 에 19 mm 추가 |
| **신설** `static.beta_vis_deg.L/.R` | ° | 데이터 ✔ / analyze ✘ | **±28** | HFOV 69°/2 − 태그 반각. 가시성 하드 제약 |
| **신설** `static.pitch_px` | px | ✘ | **24** | plan 3-4(제동 노즈다운 1° → 태그 상단 +24 px) |
| `dynamic.A_m.value` / `.sigma` | m | rotate 시 ✔ | **0.0 / 0.5** | plan 1-1 사전 [−0.5,+0.5]. σ_A 가 0.5 면 **J 겨냥 금지, 태그 직접 겨냥만** |
| `dynamic.gyro_scale.value` | — | rotate 시 ✔ | **1.0** | — |
| `dynamic.x_off_m` | m, **오른쪽 +** | mount 가 묻지만 analyze 가 안 옮김 ✘ | **0.0** | (I4) 로만 쓴다 |
| `dynamic.cam_yaw_offset_deg` | ° | ✘(analyze 미구현) | **0.0**(=`D.CAM_YAW_OFFSET_DEG`) | 태그 겨냥 목표 β\* = **−δ̂**(E1) |
| `dynamic.h_tag_cam_m` | m | mount 가 묻지만 안 옮김 ✘ | **1.10** | 로그·해석용 |
| `dynamic.epsilon_ms.value` | ms | timing 시 ✔ | **0.0** | KF 되감기 |

### 5.3 `timing_calib.json` (B 층이 직접 보는 건 이 셋뿐)
| 경로 | 내일 | 없을 때 | 용도 |
|---|---|---|---|
| `gate.passed` | ✔ | False | False 면 실주행 거부(캘리브 생산도 금지) |
| `L_ms.p99` | ✔ | None | STALE_S 와 대조해 세션 2층 판정 |
| `epsilon_ms.value` | ✔ | 0.0 | 카메라 관측을 t_capture 로 되감을 때 |
나머지(`delta_fs_ms`·`t_line_us`·`txack_offset_ms`)는 **타이밍층(`utils/timing.py`)이 쓴다. B 층은 읽지 마라.**

### 5.4 강등 사다리 (가정값으로 굴러갈 때의 규칙)
```
1) calib.gate(need=("timing","dynamics")) 가 allow=False → 기본은 **실주행 거부**(dry-run 만).
   사람이 --assume-calib 를 직접 주면: 실행 허용 + 아래 전부 강제.
2) 강등 상태에서 강제되는 것
   - 전진은 forward_slow(97) 만. 67 금지.
   - 정지 발령 D̂ = 1.5 × 계산값(일찍 선다).
   - 한 다리 길이 ≤ 1.0 m, 회전 1회 ≤ 15°.
   - FINAL 은 --final-anyway 없이는 진입 불가. 결과는 DONE_UNVERIFIED.
   - 화면 상단에 "**가정값 사용 중: <항목들>**" 고정 표시, events.jsonl 에 매 결정마다 degraded 문자열.
3) provisional=true(파일은 있으나 잠정) → 2) 와 같게 취급하되 실행 자체는 허용.
4) 자이로가 죽었거나 (I1) 잔차 > 0.2° → 강등이 아니라 **즉시 정지 + Tier 3**.
```

---

## 6. 파일 배치와 소유권

```
src/models/estimate/          A팀   __init__.py(FrameObs·Estimate·헬퍼) bearing.py heading.py track.py
src/models/dynamics/          B팀   __init__.py(Prediction) predict.py
src/models/control/dock_fsm.py  C팀  (enum·Command·전이표·가이드·수용식)
src/models/control/failure.py   C팀  (Tier·FailureCode·고장 감지·ABORT 절차)
tools/etc/sim_plant.py        D팀   플랜트(§7)
tools/etc/test_dock.py        D팀   계약·부호·고장주입 시험
tools/dock.py                 통합   실행 루프 하나. 여기서만 SafeCanTx 를 잡는다
config/control.py             통합   §8 두 줄만 추가
```
- **다른 팀 파일은 읽기만.** 남의 파일에 import 를 추가해야 하면 통합 담당에게 말한다.
- 의존 방향(역류 금지): `utils.{timing,calib,run_log,imu_yaw}` → `estimate` → `dynamics` → `control.dock_fsm` → `tools/dock.py`. `estimate` 는 `dynamics`·`control` 을 **import 하지 않는다**. `sim_plant` 는 `estimate` 를 import 하지 않는다(플랜트는 추정기를 모른다).
- 기존 자산 중 **B 층 실행경로에서 쓰지 않는 것**: `control_from_pose.py`(v2 조준-전진 전부), `rot_control.rotate_to`, `fwd_time_model` 의 `FWD_SCALE/FWD_BIAS`. 지우지는 말고 **호출만 하지 마라**(v2 비교용).
- **수정 금지**: `apriltag_v1/`, `apriltag_v2/`, `control_forklift_v2.py`, `control_광운대.py`, `docs/`.

### 실행 루프 (통합, `tools/dock.py`) — 각 팀 함수가 불리는 자리
```python
for i, ts, img in source:                      # 카메라 또는 sim_plant
    res  = pipe.process(img, index=i, timestamp=ts)     # 진짜 TagPipeline
    obs  = FrameObs.from_row(row_from(res), stamps=res.stamps)   # A팀 계약
    est  = estimator.update(obs)                        # A팀
    tick.tick(); tx.feed()                              # 타이밍·데드맨
    cmd  = fsm.step(est, dyn)                           # C팀 (+B팀 Dynamics)
    if cmd is not None and fsm.phase is Phase.EXECUTE:
        tx.set_movement(cmd.movement, why=cmd.why)      # 유일한 송신 자리
    if fsm.phase in (Phase.STOPPING, Phase.SETTLE):
        tx.stop(why=fsm.stop_reason.value)
    logger.frame(stamps=res.stamps, **row_from(res), est=est_row(est), phase=fsm.phase.value)
```

---

## 7. 플랜트 계약 (D팀 `tools/etc/sim_plant.py`)

### 7.1 인터페이스
```python
class Plant(Protocol):
    t: float                                   # 가상 시계 [s]. 실시간 대기 없음
    def set_movement(self, name: str) -> None: ...   # can_tx.SAFE_MOVEMENTS 문자열만. 그 외는 ValueError
    def step(self, dt: float) -> None: ...           # 시계를 dt 만큼 굴린다
    def truth(self) -> "PlantTruth": ...             # 진값(채점용). 추정기는 **절대** 못 본다
    def frame(self) -> tuple[int, float, object]: ...    # (i, ts_rel, img) — TagPipeline 3-튜플 계약
    def gyro_rows(self) -> list[dict]: ...           # run_log.imu 와 같은 꼴 [{"s":"gyro","t":..,...}]
    def stats(self) -> dict: ...

@dataclass
class PlantTruth:
    t: float; x_m: float; lat_m: float; psi_deg: float      # §1 부호 그대로
    v_mps: float; omega_dps: float; vertical_m: float
    cmd: str; since_cmd_s: float; moving: bool
```

### 7.2 반드시 들어가는 것 (plan 4-6 G2)
죽은시간(주행 1.0 s / 회전 0.85 s) · 램프 · 정속(67 0.28 / 97 미측정 0.12) · 코스팅 τ_d·σ_τ ·
run 간 게인 ±10~30% · L/R 비대칭 · 97 stiction(베르누이) · 회전팔 A ∈ {−0.5, 0, +0.5} ·
자이로 바이어스 랜덤워크 · 스케일 s · gaps · 코너잡음 σ_c → PnP · **결정론적 매끈한 b(d, tilt, 정면) 에 초 단위 시간상관**
(프레임별 난수는 30프레임 중앙값에서 사라져 재앵커 반전을 재현 못 한다) · flip p(d,tilt) ·
가시성/상단 컷(h·pitch) · 프레임 지연·드롭·stale · CAN 10 ms 틱.

### 7.3 수용 기준 — **부호 자가시험 4개를 못 넘기면 그 시뮬로 만든 결론은 전부 무효**
```
# test_dock.py 필수 4항 (진값 없이 관측만으로 판정한다)
1) 태그를 화면 오른쪽에 두면   → est.lat_m > 0  ∧  est.beta_deg < 0
2) rotate_ccw(147) 를 주면     → gyro angle_deg 증가 ∧ est.psi_deg 증가 ∧ est.beta_deg **감소**
                                 (좌로 돌면 태그는 화면 오른쪽으로 흐른다)
3) ψ > 0 인 채 전진하면        → est.lat_m 증가 ∧ est.x_m 감소
4) |β + atan2(ℓ,x)·180/π + ψ + δ| ≤ 0.2°   (I1)
추가: 카메라가 태그보다 낮으면(vertical < 0) 가까워질 때 태그는 **위로** 잘린다(margin_px 가 상단에서 0).
```
그 외 수용: 계약 위반 0(허용 전이표 밖 전이·EXECUTE 중 stop 아닌 명령·SafeCanTx 밖 송신 = 시험 실패),
고장주입 8종(호스트 정지=전 스트림 동시 stale, 차량 워치독, 이중 송신원, 검출 300 ms, AMBIGUOUS 10 s,
자이로 표류 2°, lateral_valid False, 양자화 공백) 각각 캡 안에 Tier 도달 ∧ CAN 로그 stop 프레임 연속.

### 7.4 ⚠ `tools/etc/fake_rig.py` 는 지금 **화면이 180° 뒤집혀 있다** (E6, D팀이 고친다)
- 증상(맥 실측): `FakePlant(lateral=+0.5, vertical=−1.10, heading=0)` 을 그리면 태그가 **화면 왼쪽 아래**(u 233 < cx 319, v 444 > cy 254)에 나온다. 현실은 §1.4 대로 **오른쪽 위**여야 한다.
- 왜 눈치채기 어렵나: 자세 복원이 같이 뒤집혀서 `docking_state` 로 되돌리면 (lateral, vertical) 이 **정확히 원래 값으로 돌아온다**. 뒤집힌 것은 **픽셀에서 나오는 값** 뿐이다 → `beta_px_deg` 부호 반대, 태그 컷이 아래쪽에서 발생.
- 원인: `FakeCamera.T_camera_tag()` 의 `R_tag_cam = Ry(ψ)`. 태그축(x=화면왼쪽·y=위·z=카메라 반대)과 카메라축(x=오른쪽·y=아래·z=전방)은 **z 축 180° 차이**가 난다.
- 고치는 법: `R_tag_cam = Ry(ψ) @ diag(−1, −1, +1)` (`render()` 의 코너 대응은 이미 맞다. 손대지 마라).
- 고친 뒤 §7.3 의 1)~4) 로 확인. **고치기 전 시뮬 결과는 β 를 쓰는 판단에 대해 전부 무효다.**

---

## 8. config 에 새로 넣는 것 — **2개뿐**

```python
# config/control.py 에 추가 (둘 다 "현장에서 사람이 정하는 운영값")
DOCK_TIME_BUDGET_S = 180.0     # 한 번의 도킹에 쓰는 시간 상한 [s]. 넘으면 Tier 3 ABORT.
                               # 8 m 접근 ~30s + 프리미티브 8회 x ~8s + 여유. 현장 흐름이 바뀌면 사람이 고친다
FWD_TOL_M = 0.10               # 종방향 허용오차 [m]. 정지 q95 ~2cm 위에 블라인드 S(T) 미측정분을 얹은 값.
                               # 탑재부 길이 실측 후 줄일 것
```
근거: `DOCK_TIME_BUDGET_S` 는 plan 3-7 의 유일한 신설(운영), `FWD_TOL_M` 은 plan 4-3 의 신설(운영)이자 사용자 결정 3 의 남은 항목.
**넣지 않는 것**: `NEAR_TAG_*`(근거리 태그는 사용자 결정 4 이후, 이번 범위 밖) · k·N·캡·최소 프레임 같은 구현 세부(코드 내부 상수 + 한 줄 주석) · 측정으로 나오는 값 전부(캘리브 파일).
B 층이 읽는 config 는 이게 전부다: `LAT_TOL_M` `HEAD_TOL_DEG` `DOCK_DEPTH_M` `CAM_TO_REF_M` `STANDOFF_M` `FORWARD_SLOW` `DOCK_TIME_BUDGET_S` `FWD_TOL_M` + `detection.TAG_ID` `TAG_SIZE_M` `CAM_YAW_OFFSET_DEG`.
파생값(코드에서 계산, 상수 아님): `X_STOP_CAM_M = CAM_TO_REF_M + STANDOFF_M` (내일 2.02 m).

---

## 9. 남은 미해결 — **전부 "실차 숫자 미측정"** (설계 선택은 남기지 않았다)
1. `v_mps.97` (byte2=97 의 m/s) · 데드밴드 · stiction.
2. 회전 폐루프 잔차 σ_θ(L/R) 와 최소 신뢰 증분 θ_min,inc — 내일 데이터는 나오지만 `analyze_first_run` 이 아직 안 뽑는다(§5.1 신설 필드).
3. 회전팔 A 와 자이로 스케일 s.
4. 카메라 장착 yaw δ̂, 횡오프셋 x_off — 내일 줄자로 재지만 analyze 가 캘리브로 안 옮긴다(§5.2).
5. 정면 heading 바이어스 b(d, tilt) 와 θ_min(d) — Day 0 정적 격자.
6. 태그 컷 거리·가시 한계 β·pitch_px.
7. 탑재부 진입 깊이 `DOCK_DEPTH_M`(2.0 가정) 과 차량 워치독 T_wd.
8. σ_τ(정지 시간지터) 가 측정 가능한지 자체.

### 통합 담당이 내일 전에 할 일 (코드, 사용자 결정 불필요)
- `analyze_first_run.py` 에 §5.1·§5.2 의 **신설 필드 5종**(rot_closed, min_inc, beta_vis_deg, x_off_m/cam_yaw_offset_deg/h_tag_cam_m 이관, pitch_px) 추가.
- `config/control.py` 의 `v_mps.97` 주석 방향 정정(없으면 67 이 아니라 **97 유지 + 다리 제한**).
- §8 두 상수 추가.
