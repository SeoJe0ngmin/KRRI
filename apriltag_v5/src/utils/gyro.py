"""GyroYaw 위에 얹는다 — 최신 가속도 보관 · 회전 정지 판정 · 서 있을 때 영점(ZUPT) · 눈금.

적분·바이어스·중력축 투영·끊김은 전부 부모 imu_yaw.GyroYaw 가 한다 — **그게 실제로 도는 코드다.**
다만 `GyroYaw(...)` 를 직접 만들지는 않는다. 항상 이 파일의 `Gyro(...)` 를 만들고,
그러면 부모 코드가 그 안에서 돈다 (imu_yaw.py 는 v3 원본 그대로 둬서 diff 가 되게 한다).
정지 판정을 **콜백 안에서** 하는 이유: 메인 루프가 밀려도 제때 멈추려고 (plan 4-2-1).

v5 에서 얹은 것 (2026-10-01 결정 R5, plan 0절):
  · 서 있을 때 영점 — 정지 명령 뒤 STILL_AFTER_S 부터는 차가 서 있다. 그동안 쌓이는 각은 전부 흐름이라 되돌리고(얼림),
    그 각속도로 바이어스를 다시 잡는다(최근 IMU_BIAS_SEC 어치 정지 샘플의 평균 — 출발 전 10초 보정과 같은 양).
    명령은 Driver 가 알려 준다(commanded). CAN 스레드가 스스로 세운 것은 모르니 그땐 영점을 안 잡는다(안전한 쪽).
    ZUPT_BLOCK_S 블록마다 평균 각속도가 잡음으로 설명되나 본다(SETTLE_K × 보정 잡음 ÷ √n, 0.5초면 약 0.045 도/s).
    넘으면(되튐 꼬리·밀림) 그 블록은 얼린 각을 돌려놓고 바이어스에도 안 넣는다 — 0.5 도/s(IMU_MOVING_DPS) 같은 굵은 기준으로는
    반쯤 움직인 블록이 통과해 바이어스를 1 도/분 넘게 망친다(자체 시험).
  · 눈금 — 회전 전후 태그 위치로 판을 넘어 배운 값(config/learned/<장소>.json, learnfile). 확실할 때만 1.0 이 아니다.
  · 출발을 본 뒤에만 예측 정지 (#1 ①) — 잔여가 요청보다 크면 차가 움직이기도 전에 끊겼다(2026-10-01 15:15 +0.82° → 0.13°).
  · 앞 가속도 기록 (2026-10-03 결정 F5) — 카메라 가속도 세 축을 최근 ACCEL_KEEP_S 초 들고 있다. 직진이 멈췄나를
    가속도계로 본다(accel_fwd → forward._settle: 제동을 본 뒤 조용해지면). 서 있을 때 카메라 피치(pitch_deg — 중력 방향)도 여기서
    — 태그컷 계산에 쓴다 (2026-10-10). 장치 시각을 우리 시계로 옮겨 둔다 — USB 가 몰아서 보내도 시각이 안 뭉친다.
"""
import math
import time
from collections import deque
from dataclasses import dataclass

from config import imu as I
from .imu_yaw import GyroYaw, _V


@dataclass(frozen=True)
class Rotation:
    """한 번의 회전 목표. 통째로 갈아끼운다 — 필드를 따로 고치면 중간 상태가 생긴다."""
    target_deg: float           # 부호 포함. +가 반시계
    tau_s: float                # 관성 시간 (학습 파일 값)
    residual_deg: float = 0.0   # 설명 안 되는 나머지 (학습 파일 값)
    period_s: float = 0.005     # 판정 주기. 자이로 200 Hz 면 5 ms
    start_angle: float = 0.0
    t_cmd: float = 0.0
    max_deg: float = 0.0        # 자이로가 죽어도 이보다 더 돌지 않는다


@dataclass
class RotationResult:
    target_deg: float = 0.0
    turned_deg: float = 0.0
    turned_at_stop: float = 0.0
    coast_deg: float = 0.0
    omega_at_stop: float = 0.0
    start_angle: float = 0.0    # 회전을 건 순간의 자이로 각 — 되튐 뒤 각(rotate_final)을 여기서 잰다
    t_cmd: float = 0.0
    t_onset: float = 0.0
    t_stop_cmd: float = 0.0
    t_settled: float = 0.0
    reason: str = ""            # predicted / crossed / max_angle / gyro_stale
    tau_observed: float = 0.0
    gaps_during: int = 0

    @property
    def done(self):
        return self.t_settled > 0.0

    @property
    def ok(self):
        """제대로 끝난 회전. 배우는 것도 다음 걸음도 이것만 믿는다."""
        return self.done and self.reason == "predicted"


def max_rate_dps(ts, angles, window_s):
    """(시각, 적분 각도) 열에서 window_s 블록마다 |각도 변화| ÷ 시간 의 최대 [도/s] — 긴 정지 창에서 "차가 움직였나" 를 본다.
    IMU_MOVING_DPS(보정 때와 같은 기준)와 견준다. 적분 각도의 **차**로 보면 안 된다: 드리프트가 쌓인다
    (2026-10-02 실측 — 60초에 0.59도 흘렀고 그동안 화면 속 태그는 0.004도도 안 움직였다).
    v5 부터는 서 있는 동안 각을 얼리므로(ZUPT) 진짜 정지면 0 이다. 명령 없이 움직인 블록은 Gyro 가 되돌려 놓아 여기서 보인다."""
    pts = [(float(t), float(a)) for t, a in zip(ts, angles) if a is not None and math.isfinite(a)]
    best, i = 0.0, 0
    while i < len(pts):
        j = i
        while j + 1 < len(pts) and pts[j + 1][0] - pts[i][0] <= window_s:
            j += 1
        dt = pts[j][0] - pts[i][0]
        if dt >= window_s / 2:                       # 프레임 두어 장짜리 꼬투리 블록은 잡음이 크다 — 뺀다
            best = max(best, abs(pts[j][1] - pts[i][1]) / dt)
        i = j + 1
    return best

class Gyro(GyroYaw):
    # 판정 문턱. apriltag_v3/src/models/control/rot_control.py 의 실측 근거 값들
    ONSET_DEG = 0.15            # 이만큼 움직이면 "출발했다"
    WRONG_WAY_DEG = 5.0         # 반대로 이만큼 돌면 부호가 뒤집힌 것 → 즉시 정지
    SETTLE_K = 3.0              # 멎음 판정 = 보정 때 잰 잡음 x 이 값
    SETTLE_FLOOR_DPS = 0.5      # 그 판정의 하한. 광운대 COARSE_STABLE_RATE_DEG_S.
                                # 1.0 이면 아직 도는 중에 멎었다고 본다 (꼬리 0.3도)
    SETTLE_CEIL_DPS = 2.0       # 그 판정의 상한. 광운대 COARSE_BIAS_MAX_RATE_SPREAD_DEG_S
    SETTLE_MIN_S = 0.2          # 끊은 뒤 최소 대기 (명령 반영 지연)
    SETTLE_MAX_S = 2.0          # 멎기 대기 상한
    # 서 있을 때 영점 (v5)
    STILL_AFTER_S = 1.0         # 정지 명령 뒤 이만큼 지나야 "서 있다" — 되튐이 이 안에 끝난다 (2026-10-01 실측: 대부분 0.25 s 안,
                                # 광운대 정지→자세안정 p90 0.88 s). run.STILL_S(회전 뒤 기다리는 1초)·learnfile.FINAL_AFTER_S 와 같은 근거
    ZUPT_BLOCK_S = 0.5          # 서 있나를 보는 블록 = estimate.WINDOW_S (run 이 max_rate_dps 를 부르는 블록과 같다)
    ACCEL_KEEP_S = 6.0          # 앞 가속도를 이만큼 들고 있다 — 직진 명령 직전 1초(기준)부터 멈춤 확인(정지 명령 뒤 최대 2초)까지 꺼낼 수 있게.
                                # 직진 하나(최대 FWD_SAFETY_MAX_S)를 통째로 들 필요는 없다: 기준은 명령 때, 멈춤은 정지 명령 뒤만 본다

    def __init__(self, **kw):
        super().__init__(**kw)
        self.accel = None                       # 최신 중력 방향. 태그 기울기 보정 (plan 5-3)
        self.intervals = deque(maxlen=4000)     # 처리 간격 — GIL 대기를 여기서 본다
        self._mono_last = None
        self._rot = None
        self._stop_fn = None
        self._result = None
        self._gaps0 = 0
        self.scale = 1.0                        # 눈금 — 학습 파일 gyro.scale (run.py 가 넣는다). 1.0 = 보정 안 함
        # 서 있을 때 영점 — 아래 상태는 콜백 스레드가 _lock 아래에서만 바꾼다 (명령 표시 둘만 Driver 쪽 스레드가 쓴다)
        self._moving_cmd = True                 # 명령을 모르면 움직인다고 본다 — 영점을 안 잡는 쪽이 안전하다
        self._t_still_from = None               # 정지 명령 + STILL_AFTER_S. 이 뒤부터 "서 있다"
        self._zb = None                         # 지금 블록 [시작, Σ각속도(보정 바이어스 기준), n, 얼린 각]
        self._zq = deque()                      # 받아들인 블록 (Σ, n) — 최근 IMU_BIAS_SEC 어치만
        self._zs, self._zn = 0.0, 0
        self._bias0 = None                      # 출발 전 보정 바이어스 (3축)
        self._corr = 0.0                        # 그 뒤 더 흐르는 양 [rad/s] (우리 부호, 회전축 방향)
        self._recent = deque(maxlen=int(self.hz * self.ZUPT_BLOCK_S) + 1)   # 최근 각속도 — 회전 직전 "멎었나"
        self.zupt = {"still_s": 0.0, "frozen_deg": 0.0, "blocks": 0, "moved_blocks": 0}
        # 가속도 (F5) — (장치 시각, x, y, z). 콜백이 append, 제어 스레드가 복사해 읽는다 (deque append 는 한 번에 끝난다)
        self._acc = deque(maxlen=int(self.ACCEL_KEEP_S * I.IMU_ACCEL_HZ * 2))   # 2 = 설정보다 빨리 와도 모자라지 않게
        self._acc_off = None                    # 우리 시계 − 장치 시각 의 최솟값 = 가장 덜 늦게 도착한 샘플 기준 (clock.CameraClock 과 같은 방식)

    # ── SDK 콜백 ────────────────────────────────────────────────────
    def _on_frame(self, f):
        t_prev = self._t_last                   # 부모가 이 샘플을 적분하기 전 시각 — 얼릴 몫을 같은 dt 로 낸다
        super()._on_frame(f)
        try:
            if not f.is_motion_frame():
                return
            m = f.as_motion_frame()
            kind = m.get_profile().stream_type().name
        except Exception:
            return
        if kind == "accel":
            # GyroYaw 는 보정 중에만 가속도를 모은다. 우리는 항상 최신값이 필요하다
            v = m.get_motion_data()
            self.accel = (v.x, v.y, v.z)
            try:
                ts = m.get_timestamp() / 1000.0
                off = time.monotonic() - ts
                if self._acc_off is None or off < self._acc_off:
                    self._acc_off = off             # 한 방향으로만 당긴다 (CameraClock 과 같다)
                self._acc.append((ts, float(v.x), float(v.y), float(v.z)))
            except Exception:
                pass
            return
        if kind != "gyro":
            return
        now = time.monotonic()
        if self._mono_last is not None:
            self.intervals.append((now - self._mono_last) * 1000.0)
        self._mono_last = now
        self._zupt(now, t_prev)
        if self._rot is not None:
            self._judge(now)

    # ── 눈금 ────────────────────────────────────────────────────────
    @property
    def angle_deg(self):
        """영점 이후 돈 각 [도] × 눈금. +가 반시계(왼쪽)."""
        return GyroYaw.angle_deg.fget(self) * self.scale

    @property
    def rate_dps(self):
        return GyroYaw.rate_dps.fget(self) * self.scale

    @property
    def raw_angle_deg(self):
        """눈금을 안 곱한 각 — 눈금을 배울 때 쓴다."""
        return GyroYaw.angle_deg.fget(self)

    # ── 앞 가속도 (F5) ──────────────────────────────────────────────
    def accel_fwd(self, t0, t1=None):
        """[t0, t1] (우리 시계 = clock.now 와 같은 monotonic) 사이의 카메라 앞(z) 가속도 [(t, m/s²)] — 시각 순.
        카메라 축이 앞이라 전진이 빨라지면 +, 서면(제동) − (2026-10-01 기록: 출발 +0.2~0.4 · 제동 −0.8~−1.2 m/s²). 중력이 섞인 몫
        (카메라가 숙인 만큼)은 그대로 들어 있다 — 쓰는 쪽이 직전 정지 때의 값을 빼서 쓴다. 아직 없으면 []."""
        return [(t, z) for t, _, _, z in self._accel_between(t0, t1)]

    def pitch_deg(self, t0, t1=None, min_n=10):
        """[t0, t1] 동안 카메라 피치 [도] (+ 위를 본다) — 중력 방향(가속도 세 축의 중앙값)에서: atan2(z, √(x² + y²)).
        **서 있을 때만** 뜻이 있다(달리는 중엔 가속이 섞인다). 가속도계(IMU) 축 기준 — 컬러 카메라 축과의 어긋남은 2026-10-01 기록에서
        −0.04 ~ −0.19° 라 같다고 본다(run._pitch_now). 샘플이 min_n 미만이면 None."""
        pts = self._accel_between(t0, t1)
        if len(pts) < min_n:
            return None
        mid = len(pts) // 2
        x, y, z = (sorted(p[k] for p in pts)[mid] for k in (1, 2, 3))
        return math.degrees(math.atan2(z, math.hypot(x, y)))

    def _accel_between(self, t0, t1):
        off = self._acc_off
        if off is None:
            return []
        out = []
        for ts, x, y, z in list(self._acc):
            t = ts + off
            if t >= t0 and (t1 is None or t <= t1):
                out.append((t, x, y, z))
        return out

    # ── 서 있을 때 영점 (ZUPT) ──────────────────────────────────────
    def calibrate(self, sec=I.IMU_BIAS_SEC):
        """부모 보정(정지 sec 초) 뒤 영점 상태를 새로 시작한다. 보정 샘플은 '이미 잰 정지' 로 넣어 둔다 —
        짧은 정지 한 번이 10초 보정을 덮어쓰지 않게(평균의 무게가 샘플 수만큼)."""
        with self._lock:                         # 보정하는 동안·끝나는 순간엔 영점을 안 만진다 — 새 보정값을 옛 기준으로 덮어쓰지 않게
            prev = (self._bias0, self._corr, self._zq, self._zs, self._zn)
            self._bias0, self._zb = None, None
        rep = None
        try:
            rep = super().calibrate(sec)
        finally:
            with self._lock:
                if rep is not None and not rep.get("moving"):
                    self._bias0 = tuple(self._bias)
                    self._corr = 0.0
                    n0 = int(rep.get("n") or 0)
                    self._zq = deque([(0.0, n0)]) if n0 else deque()
                    self._zs, self._zn = 0.0, n0
                else:                            # 움직여서 안 받아들였거나 실패 — 부모가 전 바이어스를 그대로 두니 영점 상태도 그대로
                    self._bias0, self._corr, self._zq, self._zs, self._zn = prev
                self._zb = None
        return rep

    def commanded(self, moving):
        """Driver 가 명령을 바꿀 때마다 부른다 (driver.on_movement). 콜백 안(회전 끊기)에서도 불린다 — 표시만 바꾼다."""
        if moving:
            self._moving_cmd = True
            return
        if self._moving_cmd or self._t_still_from is None:      # 정지가 거듭 와도 처음 정지부터 센다
            self._t_still_from = time.monotonic() + self.STILL_AFTER_S
        self._moving_cmd = False

    def recent_rate_dps(self):
        """최근 ZUPT_BLOCK_S 동안 평균 각속도 [도/s] (눈금 포함). 회전 명령 직전에 "차가 멎어 있었나" 를 적는다. 없으면 None."""
        with self._lock:
            xs = list(self._recent)
        return math.degrees(sum(xs) / len(xs)) * self.scale if xs else None

    def zupt_report(self):
        with self._lock:
            return dict(self.zupt, drift_dpm=round(math.degrees(self._corr) * 60.0, 3),
                        bias_samples=self._zn, still_now=bool(self._zb))

    def _zupt(self, now, t_prev):
        with self._lock:
            if self._calib is not None or not self._calibrated or self._bias0 is None:
                self._zb = None                  # 보정 중·보정 전 — 부모가 바이어스를 쥐고 있다
                return
            ts = self._t_last
            dt = (ts - t_prev) if (t_prev is not None and ts is not None) else 0.0
            r = self._project(_V(*self._last_w))                 # 지금 영점 기준 각속도 [rad/s]
            self._recent.append(r)
            still = not self._moving_cmd and self._t_still_from is not None and now >= self._t_still_from
            if not still:
                if self._zb is not None:
                    self._close_block()          # 움직이라는 명령 직전까지는 서 있었다 — 받을지는 블록 평균이 정한다
                return
            frozen = r * dt if 0.0 < dt <= self._dt_max else 0.0
            self._angle_rad -= frozen            # 서 있는 동안 쌓인 각 = 전부 흐름 → 되돌린다
            if self._zb is None:
                self._zb = [now, 0.0, 0, 0.0]
            b = self._zb
            b[1] += r + self._corr               # 출발 전 보정 바이어스 기준 — 영점을 바꿔도 같은 기준으로 쌓인다
            b[2] += 1
            b[3] += frozen
            if now - b[0] >= self.ZUPT_BLOCK_S:
                self._close_block()

    def _close_block(self):
        """_lock 아래에서만. 블록 평균이 잡음으로 설명되면 바이어스에 넣고, 아니면 얼린 각을 돌려놓는다."""
        _, s, n, frozen = self._zb
        self._zb = None
        if n == 0:
            return
        # 서 있는 블록의 평균 각속도는 잡음 ÷ √n 안에 있어야 한다. 잡음은 출발 전 보정이 잰 값 (멎음 판정과 같은 SETTLE_K 배)
        lim = (self.SETTLE_K * math.radians(self._noise_dps) / math.sqrt(n) if self._noise_dps
               else math.radians(I.IMU_MOVING_DPS))
        if abs(s / n - self._corr) > lim:
            self._angle_rad += frozen            # 서 있지 않았다(되튐 꼬리·밀림) — 그 각은 진짜다
            self.zupt["moved_blocks"] += 1
            return
        self.zupt["blocks"] += 1
        self.zupt["still_s"] += n / float(self.hz)
        self.zupt["frozen_deg"] += math.degrees(frozen)
        self._zq.append((s, n))
        self._zs += s
        self._zn += n
        cap = int(I.IMU_BIAS_SEC * self.hz)      # 최근 이만큼만 — 흐르는 속도가 1분 안에서도 바뀐다 (2026-10-01: 0.23 → 0.50 도/분)
        while self._zn > cap and self._zq:
            s_old, n_old = self._zq[0]
            extra = self._zn - cap
            if n_old <= extra:
                self._zq.popleft()
                self._zs -= s_old
                self._zn -= n_old
            else:
                keep = n_old - extra
                s_keep = s_old * keep / n_old
                self._zq[0] = (s_keep, keep)
                self._zs -= s_old - s_keep
                self._zn -= extra
        self._corr = self._zs / self._zn if self._zn else 0.0
        k = self._corr / self.sign               # _project = sign·((w − 바이어스)·축) 이라 축 방향으로 이만큼 옮기면 각속도가 corr 만큼 준다
        self._bias = tuple(b0 + k * a for b0, a in zip(self._bias0, self._axis))

    # ── 회전 ────────────────────────────────────────────────────────
    def arm(self, rot: Rotation, stop_fn):
        """회전을 건다. stop_fn 은 콜백 안에서 불린다 — 변수 하나 바꾸는 수준이어야 한다."""
        self._result = RotationResult(target_deg=rot.target_deg, t_cmd=rot.t_cmd, start_angle=rot.start_angle)
        self._gaps0 = self.stats().get("gaps", 0)
        self._stop_fn = stop_fn
        self._rot = rot                          # 마지막에 건다. 먼저 걸면 미완성 상태를 본다

    def disarm(self):
        self._rot, self._stop_fn = None, None

    def result(self):
        return self._result

    def _judge(self, now):
        rot, res = self._rot, self._result
        if rot is None or res is None:
            return
        if not self.alive:
            self._cut(now, "gyro_stale", self.angle_deg - rot.start_angle)
            return
        turned = self.angle_deg - rot.start_angle
        if res.t_onset == 0.0 and abs(turned) >= self.ONSET_DEG:
            res.t_onset = now
        if res.t_stop_cmd == 0.0:
            remaining = abs(rot.target_deg) - abs(turned)
            lead = abs(self.rate_dps) * (rot.tau_s + rot.period_s / 2) + rot.residual_deg
            # (lead 를 목표의 절반으로 누르던 줄은 뺐다. 2026-10-02 실차 5도 스텝: 2.5도에서 끊고 관성 3.5도 → 매번 6.1~6.4도.
            #  관성이 목표의 절반보다 크면 더 일찍 끊어야 맞다. 너무 작은 각은 rotate() 가 하한으로 거부한다)
            # **출발을 본 뒤에만** 예측으로 끊는다 (#1 ①, v5). 잔여가 요청보다 크면 차가 움직이기도 전에 "다 돌았다" 가 되고,
            # 그 정지가 회전 명령보다 먼저 나가면 회전 명령이 덮어써 감시 없이 돈다(리뷰 A — 실차 0번, 코드 재현 200번 중 7번)
            if res.t_onset and remaining <= lead:
                self._cut(now, "predicted", turned)
            elif turned * rot.target_deg < 0 and abs(turned) >= self.WRONG_WAY_DEG:
                self._cut(now, "wrong_way", turned)      # 부호가 뒤집혔다
            elif rot.max_deg > 0 and abs(turned) >= rot.max_deg:
                self._cut(now, "max_angle", turned)
            return
        # 끊은 뒤 — 조용해지면 끝. 문턱을 보정 때 잰 잡음에서 뽑는다 (고정값보다 낫다)
        waited = now - res.t_stop_cmd
        quiet = min(self.SETTLE_CEIL_DPS,
                    max(self.SETTLE_FLOOR_DPS, (self.noise_dps or 0.3) * self.SETTLE_K))
        settled = waited > self.SETTLE_MIN_S and abs(self.rate_dps) < quiet
        if settled or waited > self.SETTLE_MAX_S:
            # 문턱을 넘은 순간엔 아직 잡음만큼 돌고 있다. 그 **남은 꼬리**를 모델로 채운다.
            # 안 채우면 실제보다 적게 적히고(3도/s·tau 0.3 이면 0.27도), 잔여보정이 그만큼
            # 반대로 배워서 차는 조용히 더 돈다. 관성이 지수면 꼬리 = 지금각속도 x tau 다.
            tail = self.rate_dps * rot.tau_s if settled else 0.0
            res.t_settled = now
            res.turned_deg = turned + tail
            res.coast_deg = res.turned_deg - res.turned_at_stop
            res.tau_observed = (abs(res.coast_deg) / abs(res.omega_at_stop)
                                if abs(res.omega_at_stop) > 0.2 else 0.0)
            res.gaps_during = self.stats().get("gaps", 0) - self._gaps0
            if not settled:
                res.reason = "settle_timeout"      # 아직 돌고 있다 — 믿지 마라
            self.disarm()

    def _cut(self, now, reason, turned):
        res = self._result
        res.t_stop_cmd, res.reason = now, reason
        res.omega_at_stop, res.turned_at_stop = self.rate_dps, turned
        fn, self._stop_fn = self._stop_fn, None
        if fn is not None:
            try:
                fn()
            except Exception:
                pass
        if reason == "gyro_stale":
            res.t_settled = now
            res.turned_deg = turned
            self.disarm()

    # ── 품질 ────────────────────────────────────────────────────────
    def quality(self):
        import statistics
        xs = sorted(self.intervals)
        st = self.stats()
        p99 = xs[int(0.99 * (len(xs) - 1))] if xs else None
        if xs:
            # 수신율은 **우리 시계**로 — 장치 스탬프는 global time 이 켜지는 순간 기준이 바뀌어(17억 초 점프) 첫·끝을 못 뺀다
            st["hz"] = 1000.0 * len(xs) / sum(xs) if sum(xs) > 0 else 0.0
        st["process_interval_ms"] = {          # 5 ms 를 크게 넘으면 GIL 대기다
            "median": round(statistics.median(xs), 2) if xs else None,
            "p99": round(p99, 2) if p99 else None,
            "max": round(xs[-1], 2) if xs else None}
        st["scale"] = self.scale
        st["zupt"] = self.zupt_report()
        return st


if __name__ == "__main__":
    # 자체 시험 — 하드웨어 없이. 가짜 자이로 샘플을 콜백에 그대로 넣는다(SDK 프레임 모양만 흉내).
    # (1) 서 있으면 각을 얼리고 바이어스를 잡는다 (2) 명령 없이 움직이면 얼린 각을 돌려놓는다 (3) 움직이는 동안은 안 얼린다
    # (4) 눈금 (5) 출발 전엔 예측 정지를 안 한다
    import sys
    from pathlib import Path
    sys.path.insert(0, str(next(d for d in Path(__file__).resolve().parents if (d / "config").is_dir() and (d / "src").is_dir())))

    class _M:
        def __init__(self, kind, xyz, ts):
            self.kind, self.xyz, self.ts = kind, xyz, ts

        def is_motion_frame(self):
            return True

        def as_motion_frame(self):
            return self

        def get_profile(self):
            k = self.kind
            return type("P", (), {"stream_type": lambda s: type("S", (), {"name": k})()})()

        def get_motion_data(self):
            x, y, z = self.xyz
            return type("D", (), {"x": x, "y": y, "z": z})()

        def get_timestamp(self):
            return self.ts * 1000.0

        def get_frame_timestamp_domain(self):
            return "global_time"

    class VT:
        t = 5000.0

    time.monotonic = lambda: VT.t
    g = Gyro(hz=200)
    DT = 1.0 / 200
    axis = (0.0, -1.0, 0.0)                         # 카메라 수평 — 회전축 = −y, 우리 부호 + 반시계

    def feed(rate_dps, seconds):
        """참 각속도 rate_dps 로 seconds 동안. 센서는 바이어스 bias_true 만큼 더 읽는다."""
        for _ in range(int(round(seconds / DT))):
            VT.t += DT
            w = -math.radians(rate_dps) + bias_true     # 센서 y = −(우리 각속도) + 바이어스
            g._on_frame(_M("gyro", (0.0, w, 0.0), VT.t))

    # 보정: 바이어스를 0.2 도/분 틀리게 잡은 것처럼 — 참 바이어스와 보정값이 다르다
    bias_true = math.radians(0.6 / 60.0)            # 센서가 0.6 도/분 흐른다 (2026-10-01 60초 정지 최대 +0.63)
    with g._lock:
        g._bias, g._axis, g._calibrated, g._angle_rad = (0.0, math.radians(0.4 / 60.0), 0.0), axis, True, 0.0
        g._noise_dps = 0.149                        # 2026-10-01 보정 10초의 잡음 (드리프트 0.19~0.20 도/분 = sd/√2000)
    g._bias0, g._zq, g._zs, g._zn = tuple(g._bias), deque([(0.0, 2000)]), 0.0, 2000
    feed(0.0, 0.05)
    # (1) 정지 명령 → 1초 뒤부터 얼린다. 60초 서 있으면 각이 거의 안 변하고 바이어스가 참값으로 간다
    g.commanded(False)
    a0 = g.angle_deg
    feed(0.0, 60.0)
    drift_left = (g.angle_deg - a0)
    err_dpm = (math.degrees(-bias_true) - math.degrees(-g._bias[1])) * 60.0   # 남은 바이어스 오차 [도/분]
    print("60초 정지: 각 변화 %+.4f도 (첫 1초만 흐름) · 남은 흐름 %+.3f 도/분 (보정만 했으면 %+.2f)" % (drift_left, err_dpm, 0.2))
    assert abs(drift_left) < 0.01 and abs(err_dpm) < 0.02, (drift_left, err_dpm)
    rep = g.zupt_report()
    assert rep["blocks"] >= 100 and rep["moved_blocks"] == 0, rep
    # (3) 움직이라는 명령 중엔 안 얼린다 — 10도/s 로 3초 = 30도
    g.commanded(True)
    a1 = g.angle_deg
    feed(10.0, 3.0)
    assert abs((g.angle_deg - a1) - 30.0) < 0.05, g.angle_deg - a1
    # (2) 정지 명령인데 움직인다 → 블록이 되돌려져 각이 남고, 반쯤 걸친 블록도 바이어스에 안 들어간다
    g.commanded(False)
    feed(0.0, 1.2)
    b_before = g._bias[1]
    for push in (0.8, 0.1):                         # 굵게 밀림 · 되튐 꼬리만큼 느리게
        a2 = g.angle_deg
        feed(push, 2.0)
        feed(0.0, 0.6)                              # 열려 있던 블록이 닫혀야 돌려놓는다 (늦어도 ZUPT_BLOCK_S)
        moved = g.angle_deg - a2
        print("명령 없이 %.1f 도/s 2초: 자이로 %+.3f도 (참 %+.3f) · 되돌린 블록 %d"
              % (push, moved, 2 * push, g.zupt_report()["moved_blocks"]))
        assert abs(moved - 2 * push) < 0.02, (push, moved)
    assert abs(math.degrees(g._bias[1] - b_before) * 60.0) < 0.05, "움직인 블록이 바이어스에 섞였다"
    # (4) 눈금
    g.scale = 0.99
    assert abs(g.angle_deg - 0.99 * g.raw_angle_deg) < 1e-12
    g.scale = 1.0
    # (5) 출발 전 예측 정지 없음 — 잔여(2도)가 요청(1도)보다 커도 차가 안 움직이면 안 끊는다
    cut = []
    g.commanded(True)
    g.arm(Rotation(target_deg=1.0, tau_s=0.1, residual_deg=2.0, start_angle=g.angle_deg, t_cmd=VT.t, max_deg=2.0),
          lambda: cut.append(VT.t))
    feed(0.0, 0.8)                                  # 출발 지연 — 안 움직인다
    assert not cut, "출발 전에 끊었다"
    feed(5.0, 0.2)                                  # 움직이기 시작 → 출발을 본 뒤 바로 끊는다
    assert cut and g.result().t_onset > 0, g.result()
    # (6) 앞 가속도 기록 (F5) — 장치 시각을 우리 시계로. 가장 덜 늦게 온 샘플이 기준이라, USB 가 몰아 보내도 시각이 안 뭉친다
    g2 = Gyro(hz=200)
    for i in range(10):                             # 장치 시각 0.1 s 간격. 첫 샘플만 바로, 나머지는 0.05 s 늦게 몰려 온다
        VT.t = 7000.0 + 0.1 * i + (0.0 if i == 0 else 0.05)
        g2._on_frame(_M("accel", (0.0, -9.8, 0.01 * i), 100.0 + 0.1 * i))
    xs = g2.accel_fwd(0.0)
    assert len(xs) == 10 and all(abs(t - (7000.0 + 0.1 * i)) < 1e-9 for i, (t, _) in enumerate(xs)), xs
    assert abs(xs[3][1] - 0.03) < 1e-12 and g2.accel == (0.0, -9.8, 0.09)
    assert g2.accel_fwd(7000.25, 7000.55) == [p for p in xs if 7000.25 <= p[0] <= 7000.55]
    assert Gyro(hz=200).accel_fwd(0.0) == []
    # (7) 카메라 피치 — 위로 2도 숙인(= 올려다본) 카메라면 중력이 z 쪽으로 sin 2° 만큼 섞인다
    g3 = Gyro(hz=200)
    th = math.radians(2.0)
    for i in range(20):
        VT.t = 8000.0 + 0.01 * i
        g3._on_frame(_M("accel", (0.0, -9.8 * math.cos(th), 9.8 * math.sin(th)), 200.0 + 0.01 * i))
    assert abs(g3.pitch_deg(0.0) - 2.0) < 1e-9 and g3.pitch_deg(0.0, 7000.0) is None
    print("gyro 자체 시험 통과")
