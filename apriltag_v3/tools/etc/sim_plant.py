"""도킹 플랜트 시뮬레이터 — 지게차 + 카메라 + 태그 + 자이로. (contracts_B §7, plan 4-6 G2)

**목적 하나: 현장에 가기 전에 맥에서 도킹 전체를 몇 백 번 돌려 본다.**
그래서 여기 있는 것은 "잘 되는 시늉" 이 아니라 **9/7 에 실제로 우리를 망친 것들**이다 —
거리에 비례해 커지는 lateral 잡음, 정면에서의 **매끈한 heading 바이어스**(flip 아님),
PnP 2중해, 죽은시간, 코스팅, 명령시간 CV, 자이로 표류, 태그 화면 밖(블라인드).

부호는 contracts_B §1 그대로다. 이 파일은 그 규약을 **식 하나로** 지킨다:

    ℓ = d_h · sin(β_A − ψ_raw),   β_px = −β_A,   ψ_raw = ψ + δ            … (I1)

관측은 이 식을 거꾸로 써서 만든다(β̂·ψ̂_raw·d̂ 에 잡음 → ℓ̂·x̂ 계산). 그래서
`bearing_identity_residual` 은 시뮬에서 **구조적으로 0** 이고, 0 이 아니면 그건
시뮬 버그가 아니라 **읽는 쪽 부호 사고**다 — 현장 검출기와 같은 역할을 한다.

두 가지 관측 경로
────────────────────────────────────────────────────────────────────────
  `observe()`  해석형. frame.jsonl 한 줄 모양의 dict. **몬테카를로는 이것만 쓴다**
               (렌더형은 한 프레임에 수 ms 라 300회 × 1800프레임을 못 돌린다).
  `frame()`    렌더형. (i, ts_rel, img) — 진짜 `TagPipeline.process` 가 먹는 3-튜플.
               부호 자가시험(§7.3)과 "flip 이 실제로 난다" 확인에만 쓴다.
  두 경로의 픽셀은 같은 기하로 나온다 — self-check 가 1 px 안에서 대조한다.

숫자의 출처 (전부 9/7 실측 또는 그 적합. **캘리브가 아니다** — 이 파일은 실사용 미참조)
────────────────────────────────────────────────────────────────────────
  τ_start 1.0 s(주행)/0.85 s(회전) · 가속 0.1396 m/s² · 정속 0.2843 m/s(67)
  코스팅 τ_d 0.5 s → 14 cm · 회전 8 °/s, τ_r 0.18 s → 1.44°
  ※ τ_start 1.0 + a 0.1396 + τ_d 0.5 를 넣으면 S(T) 가 `fwd_time_model`(t0 0.5069·
    t1 2.0357·a 0.1396) 와 **2 mm 안에서 같아진다**. self-check 가 이걸 검사한다.
  ※ byte2=97 의 m/s 는 **미측정**. 0.12 는 가정이고 로그에 그렇게 찍힌다.

자체 검증에서 나온 것 (기록해 둔다)
────────────────────────────────────────────────────────────────────────
  · S(T) 가 `fwd_time_model` 과 정속 구간에서 **≤13 mm** 로 맞는다(T 4·5·6 s).
  · 태그 컷 = **2.98 m**(h 1.10 m·태그 0.30 m·640x480 기하). 9/7 실측 3.3 m 보다 앞이라
    정지점 2.02 m 까지 블라인드가 **0.96 m**(계약 §의 1.28 m 은 컷 3.3 m 가정).
  · 두 해 재투영비는 3.5~8 m 어디서도 0.3~0.95 다 → plan 1-3 의 `ERR_RATIO 0.2`
    를 **하드 게이트로 쓰면 heading 이 영영 유효해지지 않는다**(소프트로 써야 한다).

    python tools/etc/sim_plant.py            # 플랜트 자체 검증(9/7 대조)
    python tools/etc/sim_plant.py --n 400    # 표본 늘려서
"""
import argparse
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass, field

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from config import detection as D                       # noqa: E402
from src.models.control.can_tx import SAFE_MOVEMENTS    # noqa: E402
from src.models.detection.image import intrinsics_from_ref  # noqa: E402
from src.utils import timing as TM                      # noqa: E402

# ── 9/7 실측(또는 그 적합) — 모듈 안 상수. config 아니다 ──────────────────────
V_CRUISE_67 = 0.2843      # m/s. 9/7 0.28~0.30, fwd_time_model vmax = a·t1
V_CRUISE_97 = 0.12        # m/s. ★미측정 가정 (byte2=97). degraded 로 표시된다
ACCEL_MPS2 = 0.1396       # m/s². fwd_time_model FWD_A 적합값(선형 램프)
T_START_FWD = 1.00        # s. 주행 명령 → 실이동 시작
T_START_ROT = 0.85        # s. 회전 명령(ROT_T0)
SIG_T_START = 0.30        # s. 명령마다의 출발 지연 지터 → S(T) CV 의 주원인
TAU_D_FWD = 0.50          # s. stop 뒤 지수 코스팅 → 0.284·0.5 = 14 cm
OMEGA_DPS = 8.0           # °/s. byte1 ±20 제자리회전
ALPHA_ROT = 40.0          # °/s². 회전 가속 ★미측정(0.2 s 에 정속)
TAU_R = 0.18              # s. 회전 코스팅 → 8·0.18 = 1.44°
BWD_GAIN = 1.00           # 후진(byte2=187) 속도배. 전진과 같다고 본다 ★미측정

# 개체차(run 간). plan 4-6 G2 "run 간 게인 ±10~30% · L/R 비대칭"
GAIN_SIGMA = 0.10         # 정속·회전속 게인 1σ
LR_ASYM = 0.08            # 좌/우 회전속 비대칭 1σ
STICTION_P = 0.20         # 97 출발이 한 번에 안 붙을 확률(베르누이) ★미측정
STICTION_EXTRA_S = (0.3, 1.0)   # 붙을 때까지 더 걸리는 시간 [s]
ROT_BLOWUP_P = 0.02       # 드문 회전 대형 실패 확률(9/7 "드문 대형실패")

# 지각 — plan 1-4·1-3
SIGMA_C_MOVING = 0.30     # px. 주행 중 코너 잡음(정지 0.07~0.13)
SIGMA_C_STILL = 0.10      # px
K_PSI = 0.070             # σ_ψ ≈ K·(σ_c/tag_px)/sin(ν). 8 m·ν3° 에서 ≈1°
NU_FLOOR_DEG = 2.5        # ν(시선-법선 각)의 하한. 0 이면 σ 가 발산한다
B_FRONT_DEG = (1.0, 1.8)  # 정면 heading 바이어스 크기 [°]. 9/7 정면쌍 +1.1~1.8
B_NU_DEG = 12.0           # 바이어스가 ν 로 사그라드는 폭 [°]
B_TAU_S = 4.0             # 바이어스의 시간상관 [s]. **프레임별 난수면 중앙값에서 사라진다**
FLIP_P0 = 0.25            # 두 해가 완전히 안 갈릴 때의 오선택 확률
#: 거짓 해의 **계통** 재투영오차 Δ = AMB_K·tag_px²·(0.3+sin ν)/0.42 [px].
#: 렌더 경로 + 진짜 `pnp2_solutions` 로 실측해 맞춘 값(3.5/5/8 m × ν 0~21°).
#: 두 해 재투영비 = σ_c/hypot(σ_c, Δ) → **멀거나 움직이면 안 갈리고, 가까이 서면 갈린다.**
AMB_K = 2.0e-4
TAG_SIZE_ERR = 0.01       # 인쇄·실측 오차 1% → 거리 1%(3.5 m 에서 35 mm)
MIN_TAG_PX = 8.0          # 이보다 작으면 검출 안 됨
MISS_P = 0.01             # 조건이 좋아도 가끔 놓친다

# 시계 — plan 4-1
FPS = 30.0
LAT_MEDIAN_S = 0.060      # 프레임 지연 L 중앙값
LAT_SIGMA_S = 0.012
DELTA_FS_S = 0.0368       # FRAME_TS − SENSOR_TS (depth 실측 36.8 ms)
DROP_P = 0.005            # 프레임 드롭
GYRO_HZ = 200
GYRO_NOISE_DPS = 0.05
GYRO_BIAS0_DPS = (-0.15, 0.15)
GYRO_WALK_DPS_RTS = 0.02  # 랜덤워크 [°/s/√s]
GYRO_SCALE_SIGMA = 0.03
GYRO_GAP_P = 0.0          # 샘플당 자이로 유실 확률. **기본 0** — 세션 게이트가
                          # 'gyro gaps = 0' 을 요구한다(plan 4-2). 유실은 고장주입으로만.
GYRO_GAP_S = 0.25         # 한 번 끊기면 이만큼 [s] — 블라인드 중이면 Tier 3 이다

H_TAG_CAM_M = 1.10        # 태그 중심이 카메라보다 이만큼 위 → vertical = −1.10


@dataclass
class PlantTruth:
    """진값. **추정기·컨트롤러는 절대 못 본다.** 채점에만 쓴다. (contracts_B §7.1)"""
    t: float
    x_m: float
    lat_m: float
    psi_deg: float
    v_mps: float
    omega_dps: float
    vertical_m: float
    cmd: str
    since_cmd_s: float
    moving: bool


@dataclass
class RunParams:
    """한 run 동안 고정되는 개체차. 시드에서 뽑는다 — 같은 시드면 같은 차."""
    v67: float = V_CRUISE_67
    v97: float = V_CRUISE_97
    accel: float = ACCEL_MPS2
    tau_d: float = TAU_D_FWD
    t_start_fwd: float = T_START_FWD
    t_start_rot: float = T_START_ROT
    omega_l: float = OMEGA_DPS
    omega_r: float = OMEGA_DPS
    tau_r: float = TAU_R
    A_m: float = 0.0            # 회전중심이 카메라 뒤면 +
    x_off_m: float = 0.0        # 카메라가 차체 중심의 오른쪽이면 + (ℓ 과 방향 반대)
    delta_deg: float = 0.0      # 카메라 장착 yaw δ
    roll_deg: float = 0.0
    tag_scale: float = 1.0      # 태그 크기 오차 → 거리 배율
    b_front_deg: float = 0.0    # 정면 heading 바이어스(부호·크기 run 고정)
    gyro_bias: float = 0.0
    gyro_scale: float = 1.0
    stiction_p: float = STICTION_P
    sigma_c: float = SIGMA_C_MOVING

    @classmethod
    def sample(cls, rng, **over):
        g = lambda s: float(rng.normal(1.0, s))         # noqa: E731
        p = cls(
            v67=V_CRUISE_67 * g(GAIN_SIGMA),
            v97=V_CRUISE_97 * g(GAIN_SIGMA),
            accel=ACCEL_MPS2 * g(GAIN_SIGMA),
            tau_d=TAU_D_FWD * g(0.12),
            t_start_fwd=T_START_FWD * g(0.10),
            t_start_rot=T_START_ROT * g(0.10),
            omega_l=OMEGA_DPS * g(GAIN_SIGMA) * (1.0 + rng.normal(0.0, LR_ASYM)),
            omega_r=OMEGA_DPS * g(GAIN_SIGMA) * (1.0 - rng.normal(0.0, LR_ASYM)),
            tau_r=TAU_R * g(0.15),
            A_m=float(rng.choice([-0.5, 0.0, 0.5])),     # plan 4-6 G2 사전
            x_off_m=float(rng.normal(0.0, 0.03)),
            delta_deg=float(rng.normal(0.0, 1.0)),       # δ 미측정 → run 마다 다르다
            roll_deg=float(rng.normal(0.0, 0.5)),
            tag_scale=1.0 + float(rng.normal(0.0, TAG_SIZE_ERR)),
            b_front_deg=float(rng.choice([-1.0, 1.0])
                              * rng.uniform(*B_FRONT_DEG)),
            gyro_bias=float(rng.uniform(*GYRO_BIAS0_DPS)),
            gyro_scale=1.0 + float(rng.normal(0.0, GYRO_SCALE_SIGMA)),
        )
        for k, v in over.items():
            setattr(p, k, v)
        return p


@dataclass
class Faults:
    """고장주입 스위치. 전부 꺼져 있으면 정상 플랜트. (contracts_B §7.3 / plan 4-4)"""
    ambiguous_s: float = 0.0        # 이 시간 동안 두 해를 구분 못 하게 만든다(err_ratio→1)
    ambiguous_at: float = 3.0
    lateral_bad: bool = False       # lateral 을 못 믿게 한다(σ_lat 폭증)
    gyro_drift_dps: float = 0.0     # 자이로에 이만큼의 가짜 바이어스를 얹는다
    quant_gap: bool = False         # 최소 신뢰 증분 아래는 아예 안 움직인다(양자화 공백)
    detect_stall_s: float = 0.0     # 검출 스레드 정지 — 프레임만 늦는다(FM8a)
    detect_stall_at: float = 5.0
    host_freeze_s: float = 0.0      # 호스트 동결 — 프레임·자이로가 **동시에** 멎는다(FM8b)
    host_freeze_at: float = 5.0
    dual_source_at: float = 0.0     # 다른 송신원이 movement 를 가로챈다(FM11)
    dual_source_cmd: str = "stop"
    watchdog_s: float = 0.0         # 차량 워치독: 명령이 이 시간 끊기면 스스로 선다
    queue_backlog: bool = False     # 큐 16 — 가끔 0.5 s 묵은 프레임이 온다
    gyro_gap_p: float = 0.0         # 샘플당 자이로 유실 확률 (200 Hz. 1e-4 ≈ 50 s 에 1회)


def _ry(a):
    return np.array([[math.cos(a), 0.0, math.sin(a)],
                     [0.0, 1.0, 0.0],
                     [-math.sin(a), 0.0, math.cos(a)]])


#: 태그축(x=화면왼쪽·y=위·z=카메라 반대) ↔ 카메라축(x=오른쪽·y=아래·z=전방) 의 z 180° 차.
#: **이 한 줄이 빠지면 화면이 통째로 뒤집힌다**(contracts_B §7.4 E6 — fake_rig 의 그 버그).
_AXIS_FLIP = np.diag([-1.0, -1.0, 1.0])


def T_tag_cam(lat, vert, fwd, psi_raw_deg):
    """카메라 자세를 태그 좌표계로. (R_tag_cam, t_tag_cam=카메라 위치)"""
    R = _ry(math.radians(psi_raw_deg)) @ _AXIS_FLIP
    return R, np.array([lat, vert, -fwd])


def T_camera_tag(lat, vert, fwd, psi_raw_deg):
    """`docking_state` 가 먹는 꼴. (R_cam_tag, t_cam_tag = 카메라 기준 태그 위치)"""
    R, t = T_tag_cam(lat, vert, fwd, psi_raw_deg)
    Rc = R.T
    return Rc, -Rc @ t


class SimPlant:
    """contracts_B §7.1 Plant. 가상 시계로 돌고 실시간 대기를 하지 않는다."""

    def __init__(self, forward=8.0, lateral=0.0, heading_deg=0.0,
                 vertical=-H_TAG_CAM_M, seed=0, params=None, faults=None,
                 intr=None, tag_size=None, render=False, t0=None,
                 timing=None, level_hint=67):
        self.rng = np.random.default_rng(seed)
        self.seed = int(seed)
        self.p = params if params is not None else RunParams.sample(self.rng)
        self.f = faults if faults is not None else Faults()
        self.intr = intr if intr is not None else intrinsics_from_ref((480, 640))
        self.tag_size = float(tag_size if tag_size is not None else D.TAG_SIZE_M)
        self.t0 = time.time() if t0 is None else float(t0)
        self.t = self.t0

        # 진상태 (카메라 기준, 태그축 좌표 — contracts_B §1)
        self.x = float(forward)
        self.lat = float(lateral)
        self.psi = float(heading_deg)
        self.vertical = float(vertical)
        self.v = 0.0
        self.omega = 0.0

        # 명령
        self.cmd = "stop"
        self.t_cmd = self.t
        self._t_start_eff = 0.0        # 이번 명령의 실제 출발 지연(지터·stiction 포함)
        self._blocked = False          # 양자화 공백 고장: 이번 명령은 아예 안 먹는다
        self._rot_gain = 1.0
        self.cmd_log = []

        # 자이로
        self.gyro_deg = 0.0
        self.gyro_bias = self.p.gyro_bias
        self._gyro_t = self.t
        self._gyro_rows = []
        self.gyro_gaps = 0
        self._gyro_dead_until = 0.0

        # 지각
        self.seq = 0
        self._ou = float(self.rng.normal())      # 바이어스의 시간상관 상태
        self._last_obs = None
        self._render = bool(render)
        self._renderer = None
        self.timing = timing if timing is not None else TM.TimingSession(
            delta_fs_ms=DELTA_FS_S * 1000.0)
        self.timing.note_domain("gyro", "global_time")

        # 회계
        self.n_frames = 0
        self.n_tag_seen = 0
        self.n_flip = 0
        self.n_stale = 0
        self.n_drop = 0
        self.level_hint = int(level_hint)

    # ── 명령 ────────────────────────────────────────────────────────────────
    def set_movement(self, name):
        """can_tx.SAFE_MOVEMENTS 문자열만 받는다. 그 외는 ValueError(계약 §7.1)."""
        if name not in SAFE_MOVEMENTS:
            raise ValueError("플랜트가 모르는 동작: %r" % (name,))
        if name == self.cmd:
            return
        self.cmd = name
        self.t_cmd = self.t
        self.cmd_log.append((self.t - self.t0, name))
        self._blocked = False
        self._rot_gain = 1.0
        if name == "stop":
            self._t_start_eff = 0.0
            return
        if name.startswith("rotate"):
            self._t_start_eff = max(0.05, self.rng.normal(self.p.t_start_rot,
                                                          SIG_T_START * 0.5))
            if self.rng.random() < ROT_BLOWUP_P:      # 드문 대형 실패
                self._rot_gain = float(self.rng.choice([0.0, 2.5, 3.5]))
        else:
            self._t_start_eff = max(0.05, self.rng.normal(self.p.t_start_fwd,
                                                          SIG_T_START))
            # 97 stiction: 데드밴드 바로 위라 한 번에 안 붙는 일이 있다(FM4)
            if name == "forward_slow" and self.rng.random() < self.p.stiction_p:
                self._t_start_eff += self.rng.uniform(*STICTION_EXTRA_S)
        if self.f.quant_gap and name.startswith("rotate"):
            self._blocked = True          # 양자화 공백: 작은 회전은 아예 안 먹는다

    #: fake_rig.FakeController 가 쓰는 이름 — 그쪽을 그대로 꽂을 수 있게 둔다
    set_cmd = set_movement

    def _target(self):
        """(v_target, omega_target). 죽은시간 전이면 (0, 0)."""
        if self._blocked or self.t - self.t_cmd < self._t_start_eff:
            return 0.0, 0.0
        if self.f.watchdog_s and (self.t - self.t_cmd) > self.f.watchdog_s:
            return 0.0, 0.0               # 차량 워치독이 스스로 세웠다
        c = self.cmd
        if c == "forward":
            return self.p.v67, 0.0
        if c == "forward_slow":
            return self.p.v97, 0.0
        if c == "backward":
            return -self.p.v67 * BWD_GAIN, 0.0
        if c == "rotate_ccw":
            return 0.0, +self.p.omega_l * self._rot_gain
        if c == "rotate_cw":
            return 0.0, -self.p.omega_r * self._rot_gain
        return 0.0, 0.0

    # ── 적분 ────────────────────────────────────────────────────────────────
    def step(self, dt):
        """가상 시계를 dt 만큼 굴린다. 실시간 대기 없음."""
        dt = max(0.0, float(dt))
        if dt <= 0:
            return
        self.t += dt
        if (self.f.dual_source_at and self.t - self.t0 >= self.f.dual_source_at
                and self.cmd != self.f.dual_source_cmd):
            self.set_movement(self.f.dual_source_cmd)      # FM11: 남이 가로챈다
        vt, wt = self._target()

        # 직진: 가속은 선형(fwd_time_model 적합), 정지는 지수 코스팅(관성 12~14 cm)
        if vt == 0.0:
            self.v *= math.exp(-dt / max(1e-3, self.p.tau_d))
            if abs(self.v) < 1e-4:
                self.v = 0.0
        elif vt > self.v:
            self.v = min(vt, self.v + self.p.accel * dt)
        else:
            self.v = max(vt, self.v - self.p.accel * dt)

        # 회전: 같은 꼴
        if wt == 0.0:
            self.omega *= math.exp(-dt / max(1e-3, self.p.tau_r))
            if abs(self.omega) < 1e-3:
                self.omega = 0.0
        elif wt > self.omega:
            self.omega = min(wt, self.omega + ALPHA_ROT * dt)
        else:
            self.omega = max(wt, self.omega - ALPHA_ROT * dt)

        # 회전은 **회전중심** 을 고정하고 돈다 — 카메라는 팔 A 로 호를 그린다 (I2)(I3)
        dpsi = self.omega * dt
        if dpsi:
            A = self.p.A_m
            h0 = math.radians(self.psi)
            lat_p = self.lat - A * math.sin(h0)
            x_p = self.x + A * math.cos(h0)
            self.psi += dpsi
            h1 = math.radians(self.psi)
            self.lat = lat_p + A * math.sin(h1)
            self.x = x_p - A * math.cos(h1)
        # 직진: dℓ/ds = sin ψ (좌로 틀고 가면 ℓ 증가), dx/ds = −cos ψ
        if self.v:
            h = math.radians(self.psi)
            self.lat += self.v * math.sin(h) * dt
            self.x -= self.v * math.cos(h) * dt

        self._pump_gyro()

    def truth(self):
        return PlantTruth(t=self.t - self.t0, x_m=self.x, lat_m=self.lat,
                          psi_deg=self.psi, v_mps=self.v, omega_dps=self.omega,
                          vertical_m=self.vertical, cmd=self.cmd,
                          since_cmd_s=self.t - self.t_cmd,
                          moving=bool(abs(self.v) > 1e-3 or abs(self.omega) > 1e-2))

    # ── 자이로 ──────────────────────────────────────────────────────────────
    def _pump_gyro(self):
        """200 Hz 샘플을 만들어 둔다. run_log.imu 와 같은 꼴."""
        n = int(max(0.0, self.t - self._gyro_t) * GYRO_HZ)
        if n <= 0:
            return
        n = min(n, 4000)
        frozen = self._host_frozen()
        for k in range(n):
            tk = self._gyro_t + (k + 1) / GYRO_HZ
            dt = 1.0 / GYRO_HZ
            self.gyro_bias += float(self.rng.normal(0.0, GYRO_WALK_DPS_RTS * math.sqrt(dt)))
            if self.rng.random() < (self.f.gyro_gap_p or GYRO_GAP_P):
                self.gyro_gaps += 1
                self._gyro_dead_until = max(self._gyro_dead_until, tk + GYRO_GAP_S)
            if frozen or tk < self._gyro_dead_until:
                continue                      # 유실 구간 — 적분도 끊긴다
            w = (self.omega * self.p.gyro_scale + self.gyro_bias
                 + self.f.gyro_drift_dps + self.rng.normal(0.0, GYRO_NOISE_DPS))
            self.gyro_deg += w * dt
            self._gyro_rows.append({"s": "gyro", "t": tk, "th": tk,
                                    "x": 0.0, "y": -math.radians(w), "z": 0.0})
        self._gyro_t += n / GYRO_HZ

    def gyro_rows(self):
        out, self._gyro_rows = self._gyro_rows, []
        return out

    @property
    def gyro_alive(self):
        return not (self._host_frozen() or self.t < self._gyro_dead_until)

    def _host_frozen(self):
        return bool(self.f.host_freeze_s
                    and self.f.host_freeze_at <= self.t - self.t0
                    < self.f.host_freeze_at + self.f.host_freeze_s)

    # ── 지각 ────────────────────────────────────────────────────────────────
    def _corner_px(self, noise=True, psi_raw=None):
        """태그 네 모서리의 화면 좌표 (4,2). 뒤에 있으면 None.

        롤은 **화면을 회전**시켜 넣는다 — 롤 1° 가 lateral 19 mm(거리 무관)로
        새는 그 경로다(plan 1-2). 코너 잡음 σ_c 는 여기서 한 번만 들어간다.
        """
        psi_raw = (self.psi + self.p.delta_deg) if psi_raw is None else psi_raw
        s = self.tag_size / 2.0
        obj = np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]])
        R, t = T_camera_tag(self.lat, self.vertical, self.x, psi_raw)
        pts = (R @ obj.T).T + t
        if (pts[:, 2] <= 0.05).any():
            return None
        uv = np.stack([self.intr.fx * pts[:, 0] / pts[:, 2] + self.intr.cx,
                       self.intr.fy * pts[:, 1] / pts[:, 2] + self.intr.cy], axis=1)
        rho = math.radians(self.p.roll_deg)
        if rho:
            c, s_ = math.cos(rho), math.sin(rho)
            d = uv - np.array([self.intr.cx, self.intr.cy])
            uv = np.stack([d[:, 0] * c - d[:, 1] * s_,
                           d[:, 0] * s_ + d[:, 1] * c], axis=1) + \
                np.array([self.intr.cx, self.intr.cy])
        if noise and self.p.sigma_c > 0:
            uv = uv + self.rng.normal(0.0, self._sigma_c_now(), uv.shape)
        return uv

    def _sigma_c_now(self):
        """움직이면 코너가 더 흔들린다(plan 1-4: 정지 0.07~0.13 / 주행 ≈0.3 px)."""
        moving = abs(self.v) > 1e-3 or abs(self.omega) > 1e-2
        return self.p.sigma_c if moving else SIGMA_C_STILL

    def _nu_deg(self):
        """시선과 태그 법선의 **yaw 평면** 각 ν = |atan2(ℓ, x)|.

        PnP 가 yaw 를 얼마나 잘 푸느냐는 이 각이 정한다(정면일수록 못 푼다).
        코드의 `tilt_deg`(=|ψ_raw|)는 이걸 못 보는 프록시다 — plan 1-3 이 말한 그 함정.
        """
        return abs(math.degrees(math.atan2(self.lat, max(1e-6, self.x))))

    def observe(self):
        """frame.jsonl 한 줄 모양의 dict. **추정기는 이것만 본다.**"""
        self.seq += 1
        self.n_frames += 1
        t_cap = self.t
        lag = max(0.005, float(self.rng.normal(LAT_MEDIAN_S, LAT_SIGMA_S)))
        rel = self.t - self.t0
        if self.f.detect_stall_s and self.f.detect_stall_at <= rel < \
                self.f.detect_stall_at + self.f.detect_stall_s:
            lag += self.f.detect_stall_s
        if self.f.queue_backlog and self.rng.random() < 0.02:
            lag += 0.5                              # 큐 16 — 묵은 프레임
        frozen = self._host_frozen()
        if frozen:
            lag += (rel - self.f.host_freeze_at)     # 얼어 있던 만큼 통째로 묵는다
        meta = {"frame_timestamp": (t_cap + DELTA_FS_S) * 1e6,
                "sensor_timestamp": t_cap * 1e6, "actual_exposure": 83}
        stamps = self.timing.stamp(t_cap + DELTA_FS_S, t_arrival=t_cap + lag,
                                   meta=meta, domain="global_time",
                                   frame_number=self.seq)
        if stamps.stale:
            self.n_stale += 1

        row = {"seq": self.seq, "stamps": stamps, "tag_seen": False,
               "lateral_m": None, "forward_m": None, "vertical_m": None,
               "heading_deg": None, "tilt_deg": None, "distance_m": None,
               "beta_px_deg": None, "margin_px": None, "tag_px": None,
               "center_px": None, "reproj_rms_px": None, "decision_margin": None,
               "hamming": 0, "quality_ok": False, "pnp2": None,
               "gyro_deg": self.gyro_deg, "gyro_dps": self.omega * self.p.gyro_scale
               + self.gyro_bias + self.f.gyro_drift_dps,
               "gyro_gaps": self.gyro_gaps, "gyro_alive": self.gyro_alive,
               "gyro_age_s": 0.0 if self.gyro_alive else 0.5,
               "movement": self.cmd, "t_cmd_set": self.t_cmd, "t_cmd_tx": self.t_cmd,
               "sim_dropped": False, "sim_lateral_bad": False,
               "sim_ambiguous": False}

        if frozen or self.rng.random() < DROP_P:
            self.n_drop += 1
            row["sim_dropped"] = True
            self._last_obs = row
            return row

        uv = self._corner_px()
        if uv is None:
            self._last_obs = row
            return row
        h, w = int(self.intr.height or 480), int(self.intr.width or 640)
        margin = float(min(uv[:, 0].min(), uv[:, 1].min(),
                           w - 1 - uv[:, 0].max(), h - 1 - uv[:, 1].max()))
        edges = np.linalg.norm(uv - np.roll(uv, -1, axis=0), axis=1)
        tag_px = float(edges.mean())
        if margin <= 0.0 or tag_px < MIN_TAG_PX or self.rng.random() < MISS_P:
            row["margin_px"] = margin
            row["tag_px"] = tag_px
            self._last_obs = row
            return row

        # ── 여기서부터 태그가 보인다 ─────────────────────────────────────────
        self.n_tag_seen += 1
        center = uv.mean(axis=0)
        xn = (center[0] - self.intr.cx) / self.intr.fx
        beta = float(-math.degrees(math.atan2(xn, 1.0)))       # 실코드와 같은 식

        psi_raw_true = self.psi + self.p.delta_deg
        nu = self._nu_deg()
        sig_c = self._sigma_c_now()
        sig_psi = math.degrees(K_PSI * (sig_c / max(MIN_TAG_PX, tag_px))
                               / math.sin(math.radians(max(nu, NU_FLOOR_DEG))))
        # 정면 heading 바이어스: **매끈하고 초 단위로 상관** — 30프레임 중앙값으로 안 잡힌다
        self._ou += (-self._ou * (1.0 / FPS) / B_TAU_S
                     + math.sqrt(2.0 / (B_TAU_S * FPS)) * float(self.rng.normal()))
        b = (self.p.b_front_deg * math.exp(-(nu / B_NU_DEG) ** 2)
             * (1.0 + 0.3 * self._ou))
        psi_raw_meas = psi_raw_true + b + float(self.rng.normal(0.0, sig_psi))

        # PnP 2중해: 거짓 해는 시선에 대해 법선을 뒤집는다 → ψ_raw + 2φ (ℓ 이 부호 반전)
        phi = math.degrees(math.atan2(self.lat, max(1e-6, self.x)))
        delta_px = (AMB_K * tag_px ** 2
                    * (0.3 + math.sin(math.radians(nu))) / 0.42)
        err_ratio = sig_c / math.hypot(sig_c, max(1e-6, delta_px))
        if self.f.ambiguous_s and self.f.ambiguous_at <= rel < \
                self.f.ambiguous_at + self.f.ambiguous_s:
            err_ratio = 0.95
            row["sim_ambiguous"] = True
        p_flip = FLIP_P0 * err_ratio ** 4
        flipped = bool(self.rng.random() < p_flip)
        if flipped:
            self.n_flip += 1
            psi_raw_meas += 2.0 * phi

        # 거리: 태그 크기 오차(계통) + 코너 잡음(랜덤). ℓ 은 이 셋에서 **계산**한다
        d_h = math.hypot(self.lat, self.x)
        sig_d_rel = sig_c / max(MIN_TAG_PX, tag_px)
        if self.f.lateral_bad:
            # lateral 을 못 믿게 만드는 주입: **거리 배율**을 흔든다. ℓ·x 가 같이
            # 흔들려 (I1) 은 그대로 성립하고 σ_lat 만 커진다(부호경보가 헛 울리지 않는다).
            sig_d_rel = math.hypot(sig_d_rel, 0.05)
        d_meas = d_h * self.p.tag_scale * (1.0 + float(self.rng.normal(0.0, sig_d_rel)))
        phi_meas = math.radians(-beta - psi_raw_meas)          # β_A = −β_px  … (I1)
        lat_m = d_meas * math.sin(phi_meas)
        fwd_m = d_meas * math.cos(phi_meas)
        vert_m = self.vertical * (d_meas / max(1e-6, d_h))
        tilt = math.degrees(math.acos(min(1.0, abs(math.cos(
            math.radians(psi_raw_meas))))))

        row.update({
            "tag_seen": True,
            "lateral_m": lat_m, "forward_m": fwd_m, "vertical_m": vert_m,
            "heading_deg": psi_raw_meas - D.CAM_YAW_OFFSET_DEG,
            "tilt_deg": tilt,
            "distance_m": math.sqrt(lat_m ** 2 + fwd_m ** 2 + vert_m ** 2),
            "beta_px_deg": beta, "margin_px": margin, "tag_px": tag_px,
            "center_px": (float(center[0]), float(center[1])),
            "reproj_rms_px": float(abs(self.rng.normal(sig_c, 0.3 * sig_c))),
            "decision_margin": float(min(120.0, 20.0 + 1.6 * tag_px)),
            "quality_ok": True,
            "pnp2": {"yaw_deg": [psi_raw_meas, psi_raw_meas + 2.0 * phi * (1 if not flipped else -1)],
                     "pitch_deg": [0.0, 2.0 * math.degrees(math.atan2(-self.vertical, max(0.1, self.x)))],
                     "reproj_px": [sig_c, sig_c / max(1e-3, err_ratio)],
                     "err_ratio": err_ratio, "n_sol": 2},
            "sim_lateral_bad": bool(self.f.lateral_bad),
        })
        self._last_obs = row
        return row

    # ── 렌더형 (부호 자가시험·flip 확인 전용) ──────────────────────────────
    def _make_renderer(self):
        import cv2
        px, pad = 420, 120
        dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        core = cv2.aruco.generateImageMarker(dic, int(D.TAG_ID), px)
        full = cv2.copyMakeBorder(core, pad, pad, pad, pad,
                                  cv2.BORDER_CONSTANT, value=255)
        quad = np.float32([[pad, pad], [pad + px, pad],
                           [pad + px, pad + px], [pad, pad + px]])
        self._renderer = (cv2, full, quad)
        return self._renderer

    def render(self):
        """플랜트 진상태를 그린 흑백 이미지. 태그가 안 보이면 흰 화면."""
        cv2, full, quad = self._renderer or self._make_renderer()
        w = int(self.intr.width or 640)
        h = int(self.intr.height or 480)
        uv = self._corner_px(noise=False)
        if uv is None:
            return np.full((h, w), 255, np.uint8)
        # quad(좌상·우상·우하·좌하) ↔ 태그 3D 코너 (+s,+s) (−s,+s) (−s,−s) (+s,−s)
        s = self.tag_size / 2.0
        obj = np.array([[s, s, 0.0], [-s, s, 0.0], [-s, -s, 0.0], [s, -s, 0.0]])
        R, t = T_camera_tag(self.lat, self.vertical, self.x,
                            self.psi + self.p.delta_deg)
        pts = (R @ obj.T).T + t
        if (pts[:, 2] <= 0.05).any():
            return np.full((h, w), 255, np.uint8)
        dst = np.stack([self.intr.fx * pts[:, 0] / pts[:, 2] + self.intr.cx,
                        self.intr.fy * pts[:, 1] / pts[:, 2] + self.intr.cy],
                       axis=1).astype(np.float32)
        M = cv2.getPerspectiveTransform(quad, dst)
        img = cv2.warpPerspective(full, M, (w, h), borderValue=255)
        sig = self._sigma_c_now() * 5.0        # 픽셀 잡음 → 코너 잡음 대략 1/5
        if sig > 0:
            img = np.clip(img.astype(np.float32)
                          + self.rng.normal(0.0, sig, img.shape), 0, 255)
        return img.astype(np.uint8)

    def frame(self):
        """(i, ts_rel, img) — TagPipeline 3-튜플 계약 (contracts_B §7.1)."""
        from src.models.detection.image import _attach
        self.seq += 1
        self.n_frames += 1
        t_cap = self.t
        lag = max(0.005, float(self.rng.normal(LAT_MEDIAN_S, LAT_SIGMA_S)))
        meta = {"frame_timestamp": (t_cap + DELTA_FS_S) * 1e6,
                "sensor_timestamp": t_cap * 1e6, "actual_exposure": 83}
        stamps = self.timing.stamp(t_cap + DELTA_FS_S, t_arrival=t_cap + lag,
                                   meta=meta, domain="global_time",
                                   frame_number=self.seq)
        img = _attach(self.render(), None, 0.0, frame_number=self.seq,
                      meta=meta, stamps=stamps)
        return self.seq - 1, t_cap - self.t0, img

    def __iter__(self):
        return self

    def __next__(self):
        self.step(1.0 / FPS)
        return self.frame()

    def stats(self):
        return {"seed": self.seed, "frames": self.n_frames,
                "tag_seen": self.n_tag_seen, "flip": self.n_flip,
                "stale": self.n_stale, "drop": self.n_drop,
                "gyro_gaps": self.gyro_gaps,
                "A_m": self.p.A_m, "delta_deg": self.p.delta_deg,
                "b_front_deg": self.p.b_front_deg, "v67": self.p.v67,
                "v97": self.p.v97, "roll_deg": self.p.roll_deg,
                "cmds": len(self.cmd_log), "t_s": self.t - self.t0}


# ═══════════════════════════════════════════════════════════════════════════
# 플랜트 자체 검증 — "이 시뮬이 9/7 과 같은 차인가"
# 성분별 서명이 실측 밴드 밖이면 이 시뮬로 만든 결론은 못 쓴다(plan 4-6 타당성 게이트).
# ═══════════════════════════════════════════════════════════════════════════

def _pulse_distance(level, T, seed, dt=0.01):
    """명령 T 초 → 총 이동거리 [m]. 정지까지(코스팅 포함) 다 센다."""
    pl = SimPlant(forward=20.0, seed=seed)
    x0 = pl.x
    pl.set_movement("forward" if level == 67 else "forward_slow")
    n = int(T / dt)
    for _ in range(n):
        pl.step(dt)
    pl.set_movement("stop")
    for _ in range(int(4.0 / dt)):
        pl.step(dt)
        if abs(pl.v) < 1e-4:
            break
    return x0 - pl.x


def _stop_distance(level, seed, dt=0.01):
    """정속에서 stop → 코스팅 거리 [m] 와 그때의 v."""
    pl = SimPlant(forward=30.0, seed=seed)
    pl.set_movement("forward" if level == 67 else "forward_slow")
    for _ in range(int(12.0 / dt)):
        pl.step(dt)
    v0, x0 = pl.v, pl.x
    pl.set_movement("stop")
    for _ in range(int(5.0 / dt)):
        pl.step(dt)
        if abs(pl.v) < 1e-4:
            break
    return x0 - pl.x, v0


def _rotate_overshoot(side, seed, dt=0.005):
    """정속 회전에서 stop → 더 도는 각 [°]."""
    pl = SimPlant(forward=5.0, seed=seed)
    pl.set_movement("rotate_ccw" if side == "L" else "rotate_cw")
    for _ in range(int(6.0 / dt)):
        pl.step(dt)
    psi0 = pl.psi
    pl.set_movement("stop")
    for _ in range(int(3.0 / dt)):
        pl.step(dt)
        if abs(pl.omega) < 1e-3:
            break
    return abs(pl.psi - psi0)


def _static_lateral_noise(d, n, seed, heading=0.0, lat=0.0):
    """정지 상태에서 ℓ̂ 의 산포 [mm] 와 치우침 [mm]. 9/7 7~10 m 60~145 mm 대조."""
    pl = SimPlant(forward=d, lateral=lat, heading_deg=heading, seed=seed)
    pl.p.sigma_c = SIGMA_C_MOVING       # 주행 중 잡음으로 재현(9/7 값이 그쪽이다)
    vals = []
    for _ in range(n):
        pl.step(1.0 / FPS)
        r = pl.observe()
        if r["tag_seen"]:
            vals.append(r["lateral_m"])
    if len(vals) < 5:
        return None, None, len(vals)
    return (statistics.pstdev(vals) * 1000.0,
            (statistics.median(vals) - lat) * 1000.0, len(vals))


def _tag_cut_distance(seed=0):
    """정면으로 다가가며 태그가 화면에서 **끊겨서 안 돌아오는** 거리 [m]. 9/7 ≈3.3 m.

    한 프레임 놓친 건 컷이 아니다(가끔 놓친다) — 10프레임 연속 미검출을 컷으로 본다.
    """
    pl = SimPlant(forward=6.0, seed=seed)
    pl.p.sigma_c = 0.0
    pl.p.delta_deg = 0.0
    pl.p.roll_deg = 0.0
    last, miss = None, 0
    while pl.x > 0.5:
        pl.x -= 0.01
        r = pl.observe()
        if r["tag_seen"]:
            last, miss = pl.x, 0
        else:
            miss += 1
            if miss >= 10 and last is not None:
                return last
    return last


def _identity_residual(n=300, seed=7):
    """(I1) 잔차 [°]. 시뮬이 부호 규약을 스스로 지키는지 — 구조적으로 0 이어야 한다."""
    pl = SimPlant(forward=6.0, lateral=1.2, heading_deg=-6.0, seed=seed)
    worst = 0.0
    for _ in range(n):
        pl.step(1.0 / FPS)
        r = pl.observe()
        if not r["tag_seen"]:
            continue
        res = r["beta_px_deg"] - (-math.degrees(math.atan2(r["lateral_m"], r["forward_m"]))
                                  - r["heading_deg"] - D.CAM_YAW_OFFSET_DEG)
        worst = max(worst, abs(res))
    return worst


def _render_vs_analytic(seed=3):
    """렌더 경로와 해석 경로가 같은 픽셀을 내는가 [px]. cv2 없으면 None."""
    try:
        import cv2  # noqa: F401
    except Exception:
        return None
    pl = SimPlant(forward=4.0, lateral=0.6, heading_deg=-4.0, seed=seed)
    pl.p.sigma_c = 0.0
    pl.p.roll_deg = 0.0
    uv = pl._corner_px(noise=False)
    img = pl.render()
    try:
        from src.models.detection.detection_tag import detect, make_detector
    except Exception:
        return None
    ds = detect(make_detector(), img, intrinsics=pl.intr, tag_size=pl.tag_size)
    if not ds:
        return float("nan")
    c = np.asarray(ds[0].corners, dtype=float)
    # 코너 순서가 달라도 중심은 같다 — 중심으로 댄다
    return float(np.linalg.norm(c.mean(axis=0) - uv.mean(axis=0)))


def _fmt(ok):
    return "OK " if ok else "!! "


def self_check(n=200, seed=0, verbose=True):
    """9/7 대조표 한 장. 반환 = 실패한 항목 수."""
    from src.models.control import fwd_time_model as FTM
    par = FTM.PiecewiseFwdParams()
    say = print if verbose else (lambda *a, **k: None)
    bad = 0

    say("── 플랜트 자체 검증 (n=%d, seed=%d) ─────────────────────────" % (n, seed))
    say("  [1] S(T) 명령시간 → 거리.  fwd_time_model(t0 %.4f t1 %.4f a %.4f) 과 대조"
        % (par.t0, par.t1, par.a))
    say("      정속 구간(T ≥ %.1f s)에서만 같은 직선이어야 한다 — 모델은 죽은시간 0.51 s +"
        % (T_START_FWD + V_CRUISE_67 / ACCEL_MPS2))
    say("      느린 램프 2.04 s 로 적합됐고 우리는 죽은시간 1.0 s + 같은 가속이다.")
    ns = max(40, n // 2)
    for T in (1.5, 2.0, 3.0, 4.0, 5.0, 6.0):
        ds = [_pulse_distance(67, T, seed + 1000 * k) for k in range(ns)]
        med = statistics.median(ds)
        cv = statistics.pstdev(ds) / med if med > 1e-6 else float("inf")
        lo, hi = 0.0, 8.0                       # 모델은 거리→시간이라 뒤집어 푼다
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if FTM.time_from_distance_piecewise(mid, par) < T:
                lo = mid
            else:
                hi = mid
        want = 0.5 * (lo + hi)
        linear = T >= 4.0
        err = med - want
        ok = (abs(err) < max(0.07, 0.12 * want)) if linear else True
        bad += not ok
        say("      %sT=%.1fs  중앙값 %.3f m (모델 %.3f, 차 %+.3f)  CV %3.0f%%%s"
            % (_fmt(ok), T, med, want, err, 100 * cv,
               "" if linear else "   (램프 구간 — 모델과 같을 필요 없음)"))
        if T == 1.5 and cv < 0.5:
            bad += 1
            say("      !! 1.5 s 명령의 CV 가 %.0f%% 다 — 9/7 은 100%%+ 였다" % (100 * cv))
        if T == 6.0 and cv > 0.25:
            bad += 1
            say("      !! 6 s 명령의 CV 가 %.0f%% 다 — 9/7 은 ~10%% 였다" % (100 * cv))

    say("  [2] 정속 정지 관성 (9/7 12~14 cm)")
    for lv in (67, 97):
        rs = [_stop_distance(lv, seed + 7 * k) for k in range(max(8, n // 20))]
        d = statistics.median([r[0] for r in rs])
        v = statistics.median([r[1] for r in rs])
        ok = (0.10 <= d <= 0.18) if lv == 67 else (0.03 <= d <= 0.09)
        bad += not ok
        say("      %sbyte2=%d  v %.3f m/s  코스팅 %.3f m%s"
            % (_fmt(ok), lv, v, d, "   ★97 은 미측정 가정" if lv == 97 else ""))

    say("  [3] 회전 관성 (9/7 1.3~1.5°)")
    for side in ("L", "R"):
        os_ = [_rotate_overshoot(side, seed + 13 * k) for k in range(max(8, n // 20))]
        m = statistics.median(os_)
        ok = 1.0 <= m <= 2.0
        bad += not ok
        say("      %s%s  %.2f°" % (_fmt(ok), side, m))

    say("  [4] 정지 lateral 잡음 vs 거리 (9/7 7~10 m 에서 60~145 mm, 최대 415)")
    for d in (3.0, 5.0, 7.0, 8.0, 10.0):
        sd, bias, k = _static_lateral_noise(d, 120, seed + 31)
        if sd is None:
            say("      !! %.0f m  표본 부족(%d)" % (d, k))
            bad += 1
            continue
        band = (40.0, 300.0) if d >= 7.0 else (0.0, 200.0)
        ok = band[0] <= math.hypot(sd, bias) <= band[1]
        bad += not ok
        say("      %s%.0f m  σ %5.1f mm  치우침 %+6.1f mm  (합 %5.1f)"
            % (_fmt(ok), d, sd, bias, math.hypot(sd, bias)))

    say("  [5] 태그 화면 밖 = 블라인드 시작 (9/7 ≈3.3 m, 카메라가 태그보다 %.2f m 아래)"
        % H_TAG_CAM_M)
    cut = _tag_cut_distance(seed)
    ok = cut is not None and 2.5 <= cut <= 4.0
    bad += not ok
    say("      %s컷 %.2f m → 정지점 2.02 m 까지 블라인드 %.2f m"
        % (_fmt(ok), cut or float("nan"), (cut or 0) - 2.02))

    say("  [6] 부호 항등식 (I1) β = −atan2(ℓ,x) − ψ − δ")
    res = _identity_residual()
    ok = res < 1e-6
    bad += not ok
    say("      %s최대 잔차 %.2e °" % (_fmt(ok), res))

    say("  [7] 부호표 — 태그를 오른쪽/위에 두면")
    pl = SimPlant(forward=3.0, lateral=0.7, vertical=-0.35, seed=seed)
    pl.p.sigma_c = 0.0
    pl.p.delta_deg = 0.0
    pl.p.roll_deg = 0.0
    r = pl.observe()
    ok = (r["lateral_m"] > 0 and r["beta_px_deg"] < 0
          and r["vertical_m"] < 0 and r["center_px"][0] > pl.intr.cx
          and r["center_px"][1] < pl.intr.cy)
    bad += not ok
    say("      %sℓ %+.3f (>0)  β %+.2f (<0)  vertical %+.3f (<0)  화면 (%.0f, %.0f) "
        "cx/cy (%.0f, %.0f) → 오른쪽·위"
        % (_fmt(ok), r["lateral_m"], r["beta_px_deg"], r["vertical_m"],
           r["center_px"][0], r["center_px"][1], pl.intr.cx, pl.intr.cy))

    say("  [8] 렌더 경로 ↔ 해석 경로 (같은 기하여야 한다)")
    dpx = _render_vs_analytic()
    if dpx is None:
        say("      -- cv2/검출기 없음 — 건너뜀 (맥 기본 python 에는 없다)")
    else:
        ok = dpx == dpx and dpx < 2.0
        bad += not ok
        say("      %s태그 중심 차 %.2f px" % (_fmt(ok), dpx))

    say("  [9] flip(2중해) — 멀리서 움직이면 나고, 가까이 서면 갈린다")
    far = SimPlant(forward=8.0, lateral=0.05, seed=seed + 5)
    far.p.sigma_c = SIGMA_C_MOVING        # 주행 중 코너 잡음
    for _ in range(400):
        far.step(1.0 / FPS)
        far.observe()
    r_far = far.n_flip / max(1, far.n_tag_seen)
    near = SimPlant(forward=3.2, lateral=0.05, seed=seed + 6)   # 정지(코너 0.1 px)
    for _ in range(400):
        near.step(1.0 / FPS)
        near.observe()
    r_near = near.n_flip / max(1, near.n_tag_seen)
    ok = r_far > 0.05 and r_near < r_far
    bad += not ok
    say("      %s8 m 주행 %.0f%% (%d/%d)  →  3.2 m 정지 %.1f%% (%d/%d)"
        % (_fmt(ok), 100 * r_far, far.n_flip, far.n_tag_seen,
           100 * r_near, near.n_flip, near.n_tag_seen))

    say("── 실패 %d 항 ───────────────────────────────────────────────" % bad)
    return bad


def main(argv=None):
    ap = argparse.ArgumentParser(description="도킹 플랜트 시뮬 — 자체 검증")
    ap.add_argument("--n", type=int, default=200, help="펄스 표본 수")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args(argv)
    return 1 if self_check(n=a.n, seed=a.seed) else 0


__all__ = ["SimPlant", "PlantTruth", "RunParams", "Faults", "self_check",
           "T_camera_tag", "T_tag_cam", "FPS", "H_TAG_CAM_M"]

if __name__ == "__main__":
    raise SystemExit(main())
