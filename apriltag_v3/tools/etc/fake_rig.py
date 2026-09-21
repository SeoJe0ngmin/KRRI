"""가짜 장비 한 벌 — 카메라·자이로·CAN 이 없어도 **코드 경로가 그대로 돈다**.

`tools/first_run.py --dry-run` 과 맥(카메라·canlib 없음) 검증에 쓴다. 목적은
"도킹이 잘 되나" 가 아니라 **부품 사이 계약이 맞나** 다(smoke_dock.py 와 같은 정신):
진짜 TagPipeline 에 진짜 이미지를 넣고, 진짜 docking_state 를 거쳐, 진짜 기록층으로 나간다.

숫자는 9/7 실측(CLAUDE.md·plan 2 요약)에서 가져왔다 — **이 값을 캘리브로 쓰면 안 된다.**
가짜 값으로 만든 캘리브가 진짜처럼 보이지 않게, 이 소스로 만든 기록에는
`fake: true` 가 박히고 analyze 는 그 폴더를 provisional 로만 쓴다.

    지연 1.0 s(주행)/0.85 s(회전) · 정속 0.28 m/s(67)/0.12 m/s(97) · 8 도/s(회전 강도 20)
    정지 코스팅 τ 0.5 s(주행)/0.18 s(회전) · 회전 팔 A 0.3 m
"""
import math
import time

import cv2
import numpy as np


# ── 차량 ────────────────────────────────────────────────────────────────────

class FakePlant:
    """이산 명령 → 죽은시간 + 정속 + 코스팅. 가상 시계로 돈다(실시간 대기 없음)."""

    V = {"forward": 0.28, "forward_slow": 0.12, "backward": -0.28}
    OMEGA = {"rotate_ccw": +8.0, "rotate_cw": -8.0}      # [도/s]
    T_DEAD_FWD = 1.00
    T_DEAD_ROT = 0.85
    TAU_FWD = 0.50            # 정지 명령 뒤 지수 감속 시상수 [s] → 코스팅 ≈ v·τ
    TAU_ROT = 0.18
    A_M = 0.30                # 회전중심이 카메라 **뒤** 로 이만큼 [m]. 계약 §1.2 의 A 와
                              # 같은 부호다 — A>0 이면 좌회전(+ψ)에 ℓ 이 **는다**(dℓ/dψ = +A).
                              # (2026-09-21 통합: 전에는 반대였다. sim_plant·추정기와 어긋나
                              #  회전팔 추정 부호가 뒤집혀 나왔다)

    def __init__(self, forward=3.5, lateral=0.0, heading_deg=0.0, vertical=-1.10,
                 seed=0):
        self.forward = float(forward)
        self.lateral = float(lateral)
        self.heading = float(heading_deg)
        self.vertical = float(vertical)
        self.v = 0.0
        self.omega = 0.0
        self.t = time.time()                 # 가상 시계(초). 실제 시계와 같은 단위
        self.cmd = "stop"
        self.t_cmd = self.t
        self.rng = np.random.default_rng(seed)
        self.gyro_bias_dps = 0.05

    # -- 명령 --------------------------------------------------------------
    def set_cmd(self, name):
        if name != self.cmd:
            self.cmd = name
            self.t_cmd = self.t

    def _target(self):
        age = self.t - self.t_cmd
        if self.cmd in self.V:
            return (self.V[self.cmd] if age >= self.T_DEAD_FWD else 0.0), 0.0
        if self.cmd in self.OMEGA:
            return 0.0, (self.OMEGA[self.cmd] if age >= self.T_DEAD_ROT else 0.0)
        return 0.0, 0.0

    def step(self, dt):
        """가상 시계를 dt 만큼 굴린다."""
        dt = max(0.0, float(dt))
        self.t += dt
        v_t, w_t = self._target()
        # 1차 지연(가속은 빠르게, 정지는 τ 로 코스팅)
        self.v += (v_t - self.v) * min(1.0, dt / self.TAU_FWD)
        self.omega += (w_t - self.omega) * min(1.0, dt / self.TAU_ROT)
        h = math.radians(self.heading)
        # 회전중심 기준으로 움직이고, 카메라는 팔 A 만큼 떨어져 호를 그린다
        dpsi = self.omega * dt
        self.forward -= self.v * math.cos(h) * dt
        self.lateral += self.v * math.sin(h) * dt
        if abs(dpsi) > 0:
            # 회전중심을 고정하고 돈다 — 카메라는 팔 A 로 호를 그린다 (계약 §1.3 (I2)(I3)):
            #   ℓ_piv = ℓ − A·sin ψ (고정) → ℓ = ℓ_piv + A·sin ψ  ⇒ dℓ/dψ = +A
            #   x_piv = x + A·cos ψ (고정) → x = x_piv − A·cos ψ
            h2 = math.radians(self.heading + dpsi)
            self.forward -= self.A_M * (math.cos(h2) - math.cos(h))
            self.lateral += self.A_M * (math.sin(h2) - math.sin(h))
        self.heading += dpsi

    @property
    def rate_dps(self):
        return self.omega + self.gyro_bias_dps + float(self.rng.normal(0.0, 0.05))


# ── 가짜 카메라 ─────────────────────────────────────────────────────────────

class FakeCamera:
    """플랜트 상태를 그려 주는 프레임 소스. TagPipeline 이 그대로 먹는다."""

    def __init__(self, plant, intr, tag_size, tag_id=1, noise=1.5, fps=30.0,
                 px=420, pad=120, timing=None, seed=0):
        self.plant = plant
        self.intr = intr
        self.tag_size = float(tag_size)
        self.tag_id = int(tag_id)
        self.noise = float(noise)
        self.dt = 1.0 / float(fps)
        self.timing = timing
        self.i = 0
        self.t0 = None
        self.rng = np.random.default_rng(seed)
        dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        core = cv2.aruco.generateImageMarker(dic, self.tag_id, px)
        self.full = cv2.copyMakeBorder(core, pad, pad, pad, pad,
                                       cv2.BORDER_CONSTANT, value=255)
        self.src_quad = np.float32([[pad, pad], [pad + px, pad],
                                    [pad + px, pad + px], [pad, pad + px]])
        self.stats = _FakeStats()

    # -- 기하 --------------------------------------------------------------
    #: 태그축(x=화면왼쪽·y=위·z=카메라 반대) ↔ 카메라축(x=오른쪽·y=아래·z=전방) 은
    #: **z 축 180° 차이**다. 이 한 줄이 없으면 화면이 통째로 뒤집혀서 태그가
    #: 오른쪽·위 대신 왼쪽·아래에 그려진다 — 자세 복원은 같이 뒤집혀 원값으로
    #: 돌아오므로 `docking_state` 로는 안 보이고 **픽셀에서 나오는 값만**
    #: (β_px 부호·태그 컷 방향) 반대가 된다 (계약 §7.4 E6, 2026-09-21 통합에서 고침).
    _AXIS_FLIP = np.diag([-1.0, -1.0, 1.0])

    def T_camera_tag(self):
        p = self.plant
        a = math.radians(p.heading)
        R_tag_cam = np.array([[math.cos(a), 0.0, math.sin(a)],
                              [0.0, 1.0, 0.0],
                              [-math.sin(a), 0.0, math.cos(a)]]) @ self._AXIS_FLIP
        t_tag_cam = np.array([p.lateral, p.vertical, -p.forward])
        R = R_tag_cam.T
        return R, -R @ t_tag_cam            # R_cam_tag, t_cam_tag

    def render(self):
        s = self.tag_size / 2.0
        # src_quad(좌상·우상·우하·좌하) ↔ 태그 3D 코너. 순서는 tools/check/verify.py
        # synth_source 와 같게 맞췄다 — 검출기가 읽는 태그 방향이 여기서 정해진다.
        obj = np.float32([[s, s, 0], [-s, s, 0], [-s, -s, 0], [s, -s, 0]])
        R, t = self.T_camera_tag()
        pts = (R @ obj.T).T + t
        if (pts[:, 2] <= 0.05).any():
            return None                      # 태그가 카메라 뒤 — 검출 없음
        uv = (self.intr.K @ pts.T).T
        uv = (uv[:, :2] / uv[:, 2:]).astype(np.float32)
        if not np.isfinite(uv).all() or np.abs(uv).max() > 1e5:
            return None
        M = cv2.getPerspectiveTransform(self.src_quad, uv)
        img = cv2.warpPerspective(self.full, M, (self.intr.width, self.intr.height),
                                  borderValue=255)
        if self.noise > 0:
            img = np.clip(img.astype(np.float32)
                          + self.rng.normal(0.0, self.noise, img.shape), 0, 255)
        return img.astype(np.uint8)

    # -- 프레임 소스 -------------------------------------------------------
    def next_frame(self, advance=True):
        """다음 프레임 (i, ts_rel, img). 가상 시계를 한 프레임만큼 굴린다."""
        from src.models.detection.image import _attach
        if advance:
            self.plant.step(self.dt)
        ts = self.plant.t
        if self.t0 is None:
            self.t0 = ts
        img = self.render()
        if img is None:                       # 태그가 안 보이는 프레임도 프레임이다
            img = np.full((self.intr.height, self.intr.width), 255, np.uint8)
        stamps = None
        if self.timing is not None:
            meta = {"frame_timestamp": ts * 1e6, "sensor_timestamp": ts * 1e6 - 20000.0,
                    "actual_exposure": 83}
            stamps = self.timing.stamp(ts, t_arrival=ts + 0.035, meta=meta,
                                       domain="global_time", frame_number=self.i)
        f = _attach(img, None, 0.0, frame_number=self.i, meta={}, stamps=stamps)
        out = (self.i, ts - self.t0, f)
        self.i += 1
        return out

    def __iter__(self):
        return self

    def __next__(self):
        return self.next_frame()

    def close(self):
        pass


class _FakeStats:
    def update(self, *a, **kw):
        return 0

    def summary(self):
        return "fake"

    @property
    def fps(self):
        return 30.0


# ── 가짜 자이로 ─────────────────────────────────────────────────────────────

class FakeGyro:
    """GyroYaw 와 같은 얼굴. 플랜트의 heading 을 200 Hz 로 적분한 척한다."""

    HZ = 200

    def __init__(self, plant):
        self.plant = plant
        self._zero = plant.heading
        self._t_last = plant.t
        self._raw = None
        self._n = 0
        self.calibrated = True
        self.noise_dps = 0.05
        self.domain = "global_time"

    # -- 기록 --------------------------------------------------------------
    def enable_raw(self, maxlen=None):
        self._raw = []
        return self

    def pump(self):
        """가상 시계가 흐른 만큼 200 Hz 샘플을 만들어 둔다."""
        now = self.plant.t
        n = int(max(0.0, now - self._t_last) * self.HZ)
        if n <= 0:
            return
        n = min(n, 20000)
        for k in range(n):
            t = self._t_last + (k + 1) / self.HZ
            if self._raw is not None:
                w = math.radians(self.plant.rate_dps)
                self._raw.append({"s": "gyro", "t": t, "th": t,
                                  "x": 0.0, "y": -w, "z": 0.0})
                if k % 2 == 0:
                    self._raw.append({"s": "accel", "t": t, "th": t,
                                      "x": 0.0, "y": -9.81, "z": 0.0})
            self._n += 1
        self._t_last += n / self.HZ

    def drain_raw(self):
        self.pump()
        if self._raw is None:
            return []
        out, self._raw = self._raw, []
        return out

    # -- 읽기 --------------------------------------------------------------
    @property
    def angle_deg(self):
        return self.plant.heading - self._zero

    @property
    def rate_dps(self):
        return self.plant.rate_dps

    @property
    def alive(self):
        return True

    @property
    def age_sec(self):
        return 0.0

    @property
    def bias_dps(self):
        return (0.0, -self.plant.gyro_bias_dps, 0.0)

    def zero(self):
        self._zero = self.plant.heading

    def calibrate(self, sec=2.0):
        self.zero()
        return {"n": int(sec * self.HZ), "sec": sec, "accel_mean": (0.0, -9.81, 0.0),
                "bias_dps": (0.0, -self.plant.gyro_bias_dps, 0.0),
                "noise_dps": self.noise_dps, "drift_dpm": 0.2, "moving": False,
                "mean_dps": 0.0, "axis": (0.0, -1.0, 0.0), "axis_src": "가짜(fake_rig)"}

    def stats(self):
        return {"n": self._n, "gaps": 0, "hz": float(self.HZ)}

    def axis_note(self):
        return "가짜(fake_rig)"

    def close(self):
        pass


# ── 가짜 CAN ────────────────────────────────────────────────────────────────

class FakeController:
    """DirectFrameForkliftController 의 얼굴만. current_movement 만 진짜처럼 쓴다."""

    def __init__(self, plant=None, logger=None):
        self.plant = plant
        self.logger = logger
        self.ch_a = None
        self.is_running = True
        self._movement = "stop"
        self.current_control_type = "driving_mode"
        self.emergency_stop = False
        self.sent = []

    @property
    def current_movement(self):
        return self._movement

    @current_movement.setter
    def current_movement(self, name):
        if name != self._movement:
            self.sent.append((getattr(self.plant, "t", time.time()), name))
            if self.logger is not None:
                self.logger.can(dir="fake_tx", movement=name,
                                t_cmd_tx=getattr(self.plant, "t", time.time()))
        self._movement = name
        if self.plant is not None:
            self.plant.set_cmd(name)

    def connect_can(self):
        return True

    def disconnect_can(self):
        self.current_movement = "stop"


__all__ = ["FakePlant", "FakeCamera", "FakeGyro", "FakeController"]
