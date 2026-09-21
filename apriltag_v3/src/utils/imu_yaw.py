"""D435i 자이로로 **상대 yaw**(제자리 회전각)를 잰다.

절대 방위각은 못 잰다 — 이 IMU(BMI055)에는 자력계가 없다. 할 수 있는 것은
"영점 선언 후 몇 도 돌았나"뿐이고, 그게 정확히 회전 명령에 필요한 값이다.
절대 기준은 태그가 보일 때 heading_deg 로 다시 잡으면 된다.

실측(2026-08-31): 바이어스 미보정이면 5.5도/분 드리프트, 정지 2초 평균으로
보정하면 1.0도/분. 회전 한 번(수 초)에는 0.1도 수준이라 HEAD_TOL_DEG=2.0
대비 충분하다. 대신 **보정은 반드시 완전히 선 상태에서** 해야 한다 —
움직이며 보정하면 그 속도가 통째로 바이어스에 들어간다.

컬러 파이프라인과 같은 프로세스에서 같이 쓴다. Motion Module 은 별개
센서라 파이프라인을 따로 열어도 충돌하지 않는다(카메라 한 프로세스 규칙은
같은 센서를 두 프로세스가 여는 문제다).

사용 순서:

    yaw = GyroYaw().start()
    yaw.calibrate()            # 정지 2초. 지게차가 선 뒤에 부를 것
                               # **블로킹이다.** asyncio 루프 안에서는
                               #   await asyncio.to_thread(yaw.calibrate)
    ... 회전 명령 중 yaw.angle_deg 감시, 목표각에서 정지 ...
    yaw.close()

수직축을 어떻게 잡나
────────────────────────────────────────────────────────────────────────
제자리 회전은 **월드 수직축** 회전이다. 자이로는 센서 축으로 재므로,
카메라가 기울어 달렸으면 그 회전이 여러 축에 나뉘어 들어온다.
y축만 읽으면 cos(기울기)만큼 적게 세어, 15도 기울이면 90도 명령이
실제 93.2도가 된다(HEAD_TOL_DEG=2.0 초과).

그래서 보정할 때 가속도계로 **중력 방향**을 같이 재서, 자이로 벡터를
그 축에 투영한다. 카메라를 위로 기울여 달아도 각도가 맞는다.
가속도계를 못 열면 y축만 쓰는 예전 방식으로 떨어지고, 그때는
axis_note() 가 경고를 돌려준다.
"""
import collections
import math
import threading
import time

from config.imu import (IMU_BIAS_SEC, IMU_CALIB_MIN_RATIO, IMU_DT_GAP_SAMPLES,
                      IMU_GYRO_HZ, IMU_MOVING_DPS, IMU_STALE_SEC,
                      IMU_YAW_SIGN)


# accel 스트림 속도 후보. 낮은 것부터 — 중력축만 잡으면 되니 느려도 된다.
ACCEL_HZ_CANDIDATES = (63, 100, 200, 250)


class GyroYaw:
    """자이로 적분 상대 yaw. +가 반시계(왼쪽) — heading_deg 와 같은 부호 규약."""

    def __init__(self, hz=IMU_GYRO_HZ, sign=IMU_YAW_SIGN, use_accel=True):
        self.hz = int(hz)
        self.sign = float(sign)
        self.use_accel = bool(use_accel)
        self._pipe = None
        self._lock = threading.Lock()
        # 적분 상태. 전부 _lock 아래에서만 만진다 — 콜백은 SDK 스레드에서 온다.
        self._angle_rad = 0.0
        self._bias = (0.0, 0.0, 0.0)          # rad/s. calibrate() 가 채움
        self._last_w = (0.0, 0.0, 0.0)        # 마지막 원시 각속도 [rad/s]
        self._t_last = None                   # 마지막 샘플의 장치 시각 [s]
        self._wall_last = None                # 마지막 샘플의 벽시계 (age 용)
        self._n = 0
        self._gaps = 0                        # dt 가 비정상이라 적분을 건너뛴 횟수
        self._t_first = None
        self._calib = None                    # 보정 중이면 자이로 샘플 리스트
        self._calib_a = None                  # 보정 중이면 가속도 샘플 리스트
        self._calibrated = False              # calibrate() 를 마쳤나
        self._noise_dps = None                # 보정 때 잰 정지 잡음 [도/s]
        # 회전을 셀 축(센서 좌표). 기본은 예전과 같은 -y.
        # calibrate() 가 중력을 재면 그 방향으로 바꾼다.
        self._axis = (0.0, -1.0, 0.0)
        self._axis_src = "가정(카메라 수평)"
        # 샘플이 이보다 벌어지면 그 구간은 적분하지 않는다(유실 구간을
        # 마지막 각속도로 메꾸면 조용히 틀어진다. 안 더한 쪽이 눈에 보인다).
        self._dt_max = float(IMU_DT_GAP_SAMPLES) / self.hz
        # 타임스탬프 도메인(plan 4-1: 컬러·자이로 **둘 다** GLOBAL 이어야 출발).
        self._domain = None
        # 원시 기록용 링버퍼. enable_raw() 를 부른 뒤 drain_raw() 로 퍼 간다.
        self._raw = None

    # ── 수명 ────────────────────────────────────────────────────────────────

    def start(self):
        """gyro(+accel) 파이프라인을 콜백으로 연다. self 를 돌려준다."""
        import pyrealsense2 as rs
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, self.hz)
        if self.use_accel:
            # 중력축을 잡으려고 같이 연다. 없어도 자이로만으로 돌아간다.
            # accel 유효값은 IMU 칩마다 다르다 — BMI055 는 63/250, BMI085 는
            # 100/200/400 (2026-09-07 D435i 실측). 풀리는 첫 값을 쓴다.
            for accel_hz in ACCEL_HZ_CANDIDATES:
                cfg_try = rs.config()
                cfg_try.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, self.hz)
                cfg_try.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, accel_hz)
                if cfg_try.can_resolve(pipe):
                    cfg = cfg_try
                    break
            else:
                self.use_accel = False
        try:
            pipe.start(cfg, self._on_frame)
        except RuntimeError as exc:
            if self.use_accel:
                # accel 조합이 안 되는 장치일 수 있다. 자이로만으로 재시도.
                self.use_accel = False
                cfg = rs.config()
                cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, self.hz)
                try:
                    pipe.start(cfg, self._on_frame)
                except RuntimeError as exc2:
                    raise RuntimeError(
                        "gyro %dHz 스트림을 못 열었다 (%s) — 이 장치의 유효값은 200/400 뿐이다. "
                        "장치 확인: tools/realsense_check.py" % (self.hz, exc2)) from exc2
            else:
                raise RuntimeError(
                    "gyro %dHz 스트림을 못 열었다 (%s) — 이 장치의 유효값은 200/400 뿐이다. "
                    "장치 확인: tools/realsense_check.py" % (self.hz, exc)) from exc
        self._pipe = pipe
        # 모션 스트림도 **호스트 시계(global time)** 로 찍게 한다 — 컬러와 같은 시계라야
        # ψ_cam(t_capture) 과 ψ_gyro 를 같은 축에서 비교할 수 있다(plan 4-1).
        try:
            prof = pipe.get_active_profile()
            for s in prof.get_device().query_sensors():
                if s.supports(rs.option.global_time_enabled):
                    s.set_option(rs.option.global_time_enabled, 1.0)
        except Exception:
            pass                                # 지원 안 하면 도메인 검사에서 걸린다
        return self

    def close(self):
        """파이프라인을 닫는다. 몇 번 불러도 안전하다."""
        p, self._pipe = self._pipe, None
        if p is not None:
            try:
                p.stop()
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # ── 콜백 (SDK 스레드) ───────────────────────────────────────────────────

    def _on_frame(self, f):
        if not f.is_motion_frame():
            return
        m = f.as_motion_frame()
        name = m.get_profile().stream_type().name   # rs 를 다시 import 하지 않으려고
        v = m.get_motion_data()
        if name == "accel":
            with self._lock:
                if self._calib_a is not None:
                    self._calib_a.append((v.x, v.y, v.z))
                if self._raw is not None:
                    self._raw.append({"s": "accel", "t": m.get_timestamp() / 1000.0,
                                      "th": time.time(),
                                      "x": v.x, "y": v.y, "z": v.z})
            return
        if name != "gyro":
            return
        ts = m.get_timestamp() / 1000.0       # 장치 시각 [s]
        try:
            domain = str(m.get_frame_timestamp_domain())
        except Exception:
            domain = None
        with self._lock:
            self._domain = domain
            if self._raw is not None:
                self._raw.append({"s": "gyro", "t": ts, "th": time.time(),
                                  "x": v.x, "y": v.y, "z": v.z})
            self._n += 1
            self._last_w = (v.x, v.y, v.z)
            if self._t_first is None:
                self._t_first = ts
            if self._t_last is not None:
                dt = ts - self._t_last
                if 0.0 < dt <= self._dt_max:
                    self._angle_rad += self._project(v) * dt
                else:
                    self._gaps += 1
            self._t_last = ts
            self._wall_last = time.monotonic()
            if self._calib is not None:
                self._calib.append((v.x, v.y, v.z))

    def _project(self, w):
        """센서 각속도 -> 우리 부호규약의 yaw 각속도 [rad/s]. **락 안에서만 부른다.**

        회전축(self._axis)에 내적한다. 축이 기본값 (0,-1,0) 이면
        결과가 -w.y 이라 예전 y축 방식과 정확히 같다.
        """
        ax, ay, az = self._axis
        bx, by, bz = self._bias
        return self.sign * ((w.x - bx) * ax + (w.y - by) * ay + (w.z - bz) * az)

    # ── 보정 / 영점 ─────────────────────────────────────────────────────────

    def calibrate(self, sec=IMU_BIAS_SEC):
        """**완전히 선 상태에서** sec 초 모아 바이어스·회전축을 잡고 영점을 선언한다.

        **블로킹이다** (time.sleep). asyncio 루프 안에서 그냥 부르면 그 시간만큼
        다른 코루틴이 멈춘다 — CAN heartbeat 가 끊길 수 있다.
        루프 안에서 쓸 때는 ``await asyncio.to_thread(yaw.calibrate)``.

        돌려주는 것:
            {'n': 400, 'sec': 2.0,
             'bias_dps': (bx, by, bz),     축별 바이어스 [도/s]
             'noise_dps': ny,              회전축 표준편차 [도/s]
             'drift_dpm': 0.9,             남는 드리프트 예상 [도/분]
             'moving': False,              움직인 의심. True 면 **아무것도 커밋 안 하고**
                                           이전 보정을 유지한다 — 세우고 다시 부를 것
             'axis': (x, y, z),            회전을 세는 센서 축
             'axis_src': '중력(기울기 4.1도)'}
        """
        if self._pipe is None:
            raise RuntimeError("start() 전에 calibrate() 를 불렀다")
        with self._lock:
            self._calib = []
            self._calib_a = [] if self.use_accel else None
        time.sleep(float(sec))
        with self._lock:
            got, self._calib = self._calib, None
            got_a, self._calib_a = self._calib_a, None
        need = self.hz * sec * IMU_CALIB_MIN_RATIO
        if len(got) < need:
            raise RuntimeError("보정 샘플이 %d개뿐이다(기대 %d) — gyro 가 안 들어온다"
                               % (len(got), int(self.hz * sec)))
        n = len(got)
        mean = [sum(v[i] for v in got) / n for i in range(3)]      # 평균 = 바이어스

        if not self.use_accel:
            axis, src = ((0.0, -1.0, 0.0),
                         "가정(카메라 수평) — accel 스트림이 안 열림 (동시 스트림 거부, 자이로 단독)")
        elif got_a is not None and len(got_a) < 3:
            axis, src = ((0.0, -1.0, 0.0),
                         "가정(카메라 수평) — accel 스트림은 열렸는데 샘플이 %d개뿐" % len(got_a))
        else:
            axis, src = self._axis_from_gravity(got_a)

        # 회전축 성분의 표준편차. 축이 정해진 뒤에 재야 의미가 맞는다.
        proj = [sum((v[i] - mean[i]) * axis[i] for i in range(3)) for v in got]
        var = sum(p * p for p in proj) / max(n - 1, 1)
        sd = math.sqrt(var)

        deg = math.degrees
        # 표준편차만 보면 **느리게 등속 회전** 중인 것을 못 잡는다(흔들림이 없으니까).
        # 정지 자이로의 평균은 보통 0.5도/s 를 한참 밑돌므로 평균도 같이 본다.
        # 주의: 개체별 영점 오프셋(BMI055 스펙 +-1도/s급)과 실회전을 이 검사로는
        # 못 가른다 — mean_dps 를 보고서에 실어 사람이 판단하게 한다.
        mean_dps = abs(deg(sum(mean[i] * axis[i] for i in range(3))))
        moving = deg(sd) > IMU_MOVING_DPS or mean_dps > IMU_MOVING_DPS

        # **움직인 의심이면 커밋하지 않는다.** 그 속도가 통째로 바이어스에
        # 들어간 채 폐루프 회전이 시작되는 게 최악이다. 이전 보정(있으면)을
        # 유지하고, 호출자는 moving=True 를 보고 다시 부른다.
        if not moving:
            with self._lock:
                self._bias = tuple(mean)
                self._axis, self._axis_src = axis, src
                self._angle_rad = 0.0             # 보정 끝 = 영점 선언
                self._calibrated = True
                self._noise_dps = math.degrees(sd)
        accel_mean = (tuple(sum(v[i] for v in got_a) / len(got_a) for i in range(3))
                      if got_a else None)
        return {"n": n, "sec": float(sec),
                "accel_mean": accel_mean,     # 가속도 원시 평균 [m/s^2]. 규약 확인용
                "bias_dps": tuple(deg(b) for b in mean),
                "noise_dps": deg(sd),
                # 바이어스 추정 오차(sd/sqrt(n))가 그대로 드리프트가 된다.
                "drift_dpm": deg(sd / math.sqrt(n)) * 60.0,
                "moving": moving,             # True 면 위 값들은 **커밋 안 됨** — 다시 부를 것
                "mean_dps": mean_dps,         # 회전축 방향 평균 [도/s]. 오프셋/실회전 구분용
                "axis": axis, "axis_src": src}

    def _axis_from_gravity(self, samples):
        """정지 가속도 평균 -> 회전을 셀 센서 축(위쪽 단위벡터). (축, 설명).

        **부호 규약에 안 기댄다.** 가속도계가 정지에서 +1g 를 주는지 -1g 를
        주는지는 장치·SDK 마다 말이 달라서, 잘못 짚으면 축이 통째로 뒤집혀
        모든 폐루프 회전이 wrong-way 로 죽는다. 대신 확실한 사실 하나만 쓴다 —
        카메라를 뒤집어 달지는 않는다. 그래서 측정 벡터의 ± 둘 중
        기본축 (0,-1,0) 과 90도 이내인 쪽을 위로 잡는다. 어느 규약이든 맞는다.

        그렇게 잡아도 기울기가 60도를 넘게 나오면 가속도 자체를 못 믿는
        상황(진동·오독)이므로 기본축으로 후퇴한다.
        """
        default = (0.0, -1.0, 0.0)
        if not samples or len(samples) < 3:
            return default, "가정(카메라 수평) — 가속도 없음"
        n = len(samples)
        g = [sum(v[i] for v in samples) / n for i in range(3)]
        mag = math.sqrt(sum(c * c for c in g))
        if not (7.0 < mag < 12.5):            # 9.81 근처가 아니면 못 믿는다
            return default, "가정(카메라 수평) — 중력 크기 %.1f" % mag
        up = tuple(c / mag for c in g)
        if up[1] > 0:                          # 기본축과 반대쪽이면 뒤집는다
            up = tuple(-c for c in up)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -up[1]))))
        if tilt > 60.0:
            return default, "가정(카메라 수평) — 중력축 기울기 %.0f도, 못 믿음" % tilt
        return up, "중력(기울기 %.1f도)" % tilt

    def zero(self):
        """지금 각도를 0으로 선언한다."""
        with self._lock:
            self._angle_rad = 0.0

    # ── 읽기 ────────────────────────────────────────────────────────────────

    @property
    def angle_deg(self):
        """영점 이후 돈 각 [도]. +가 반시계(왼쪽)."""
        with self._lock:
            return math.degrees(self._angle_rad)

    @property
    def rate_dps(self):
        """지금 회전 속도 [도/s]. 부호는 angle_deg 와 같다.

        **마지막 샘플 값이다.** 자이로가 끊기면 그 값에 얼어붙으므로,
        멎었는지 볼 때는 alive 도 같이 확인할 것.
        """
        with self._lock:
            w = _V(*self._last_w)
            return math.degrees(self._project(w))

    @property
    def age_sec(self):
        """마지막 gyro 샘플 이후 지난 시간 [s]. 샘플이 아직 없으면 inf."""
        with self._lock:
            w = self._wall_last
        return float("inf") if w is None else time.monotonic() - w

    @property
    def alive(self):
        """gyro 가 지금도 들어오고 있나. False 면 이 값으로 회전하면 안 된다."""
        return self.age_sec <= IMU_STALE_SEC

    @property
    def calibrated(self):
        """calibrate() 를 마쳤나. False 면 바이어스가 0 이라 5.5도/분 흘러간다."""
        return self._calibrated

    @property
    def noise_dps(self):
        """보정 때 잰 정지 잡음 [도/s]. 보정 전이면 None.

        "회전이 멎었나" 판정 문턱을 이 값에서 뽑는다 — 장비·온도가 달라도
        고정 상수보다 잘 맞는다.
        """
        return self._noise_dps

    @property
    def domain(self):
        """마지막 gyro 프레임의 타임스탬프 도메인 문자열. GLOBAL 이 아니면 실주행 금지."""
        with self._lock:
            return self._domain

    @property
    def bias_dps(self):
        """지금 쓰고 있는 바이어스 [도/s] 3축. 기록용."""
        with self._lock:
            return tuple(math.degrees(b) for b in self._bias)

    def enable_raw(self, maxlen=200 * 300):
        """200 Hz 원시 샘플을 링버퍼에 쌓기 시작한다(imu.jsonl 용). drain_raw() 로 퍼 간다."""
        with self._lock:
            if self._raw is None:
                self._raw = collections.deque(maxlen=int(maxlen))
        return self

    def drain_raw(self):
        """쌓인 원시 샘플을 통째로 꺼내 온다. 기록 스레드가 주기적으로 부른다."""
        with self._lock:
            if self._raw is None:
                return []
            out = list(self._raw)
            self._raw.clear()
        return out

    def axis_note(self):
        """회전축을 어떻게 정했는지 한 줄. 로그에 남겨 두면 나중에 원인을 찾기 쉽다."""
        return self._axis_src

    def stats(self):
        """수신 요약. {'n', 'hz', 'gaps'}. gaps 가 늘면 샘플이 새고 있는 것."""
        with self._lock:
            n, gaps = self._n, self._gaps
            span = (self._t_last - self._t_first) if (
                self._t_first is not None and self._t_last is not None) else 0.0
        return {"n": n, "gaps": gaps,
                "hz": (n - 1) / span if span > 0 else 0.0}


class _V:
    """_project 가 SDK 의 motion_data 처럼 .x/.y/.z 로 읽을 수 있게 하는 껍데기."""

    __slots__ = ("x", "y", "z")

    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


def imu_panel_lines(yaw):
    """IMU 계기판을 화면 패널 항목(tuple)으로. run.py 와 live_pose.py 공용.

    회전은 이 숫자만 보고 도는 폐루프라, 화면에 안 보이면 실물에서
    부호·드리프트·끊김을 확인할 방법이 없다. yaw 가 None(미장착/실패)이어도
    그 사실을 표시한다.
    """
    if yaw is None:
        return [("kv", "IMU", "없음 — 회전이 개루프", "bad")]
    alive = yaw.alive
    gaps = yaw.stats().get("gaps", 0)
    out = [("kv", "IMU 각도", "%+.2f 도" % yaw.angle_deg, "ok" if alive else "bad"),
           ("kv", "IMU 속도", "%+.2f 도/s" % yaw.rate_dps, "ok" if alive else "dim")]
    if not alive:
        out.append(("kv", "", "끊김 %.1fs 째 — 회전 금지" % yaw.age_sec, "bad"))
    elif not yaw.calibrated:
        out.append(("kv", "", "보정 전 — 5.5도/분 흘러간다", "warn"))
    elif gaps:
        out.append(("kv", "", "샘플 누락 %d회" % gaps, "warn"))
    return out
