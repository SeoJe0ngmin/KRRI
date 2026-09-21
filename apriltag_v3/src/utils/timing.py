"""타이밍층 — 한 세션의 시계 하나. (plan 4-1 "노출중심 캡처시각, GLOBAL 대기, 2-지연 검정")

**단일 시계 = 호스트 epoch `time.time()`.** 프레임마다 여섯 시각을 남긴다:

    t_capture     노출중심(아래) — 이 프레임이 "언제의 세상" 인가
    t_arrival     wait_for_frames 가 돌아온 순간 (호스트)
    t_detect_done 검출·자세 계산이 끝난 순간
    t_decide      그 프레임으로 판단을 내린 순간
    t_cmd_set     current_movement 를 바꾼 순간
    t_cmd_tx      payload 가 바뀐 첫 ch.write() 직후 (can_tx.ChannelProbe)

t_capture 를 어떻게 잡나 (plan 4-1, researcher 1-12)
────────────────────────────────────────────────────────────────────────
`get_timestamp()`(= FRAME_TIMESTAMP)는 **USB 전송 시작**이지 노출중심이 아니다
(depth 실측 FRAME − SENSOR = 36.8 ms). 그래서

    t_capture = get_timestamp()/1000 − Δ_FS + (row − row_ref)·T_line
    Δ_FS = (FRAME_TIMESTAMP − SENSOR_TIMESTAMP)/1e6   [s]  프레임별 메타가 있으면 그 값,
                                                     없으면 세션 시작에 잰 중앙값

롤링셔터 보정 T_line(D435 컬러 ≈30 µs, 1080행이면 32 ms)은 **실측값이 있을 때만**
건다 — timing_calib 의 `t_line_us` 가 없으면 0 으로 두고 플래그만 남긴다.
**66 ms 류 상수 폴백은 없다**(plan 4-1: 미전환·미측정이면 출발 거부).

도메인은 첫 프레임 assert 가 아니라 **GLOBAL 전환까지 대기**한다 — librealsense 는
첫 동기 샘플 전까지 HARDWARE_CLOCK 을 주는 게 정상이다(#4505). 타임아웃까지도
GLOBAL 이 아니면 실주행 거부(dry-run 만).

stale/역행
────────────────────────────────────────────────────────────────────────
L = t_arrival − t_capture 가 STALE_S 를 넘으면 그 프레임으로 **판단·정지하지 않는다**
(predict-only + 경고). |t_capture − host_now| 가 SKEW_MAX_S 를 넘거나 t_capture 가
역행하면(2^32 µs = 71.6 분 랩) 같은 취급 — 세션 시작 hardware_reset 이 1차 대응.

이 파일의 숫자는 전부 **코드 내부 상수**다. config 에 올리지 않는다(plan 4-1 상수 회계).
"""
import statistics
import time

#: 프레임이 이보다 묵었으면 판단·정지에 쓰지 않는다 [s] (plan 4-1; 옛 config STALE_MS 대체)
STALE_S = 0.150
#: hardware_reset 뒤 장치가 다시 열거될 때까지 기다리는 최대 시간 [s].
#: VM(UTM USB 전달)은 재열거가 느리다 — 고정 5 s 로는 모자라 현장에서 죽었다.
RESET_WAIT_MAX_S = 40.0

#: 컬러·자이로가 GLOBAL 로 전환될 때까지 기다리는 시간 [s] (researcher 1-3: 첫 15 s 불안정)
GLOBAL_WAIT_S = 15.0
#: |t_capture − host_now| 상한 [s]. 넘으면 시계가 어긋난 것 (도메인 오류·랩)
SKEW_MAX_S = 1.0
#: 연속 프레임 간격이 이 범위 밖이면 플래그 [s] (30 fps ± 10 ms, plan 1-6)
DT_NOMINAL_S = 1.0 / 30.0
DT_TOL_S = 0.010


def hardware_reset(wait_s=5.0, log=None):
    """세션 시작 hardware_reset (plan 4-1: 2.56.x 는 71.6 분 랩 역행 미수정).

    장치가 없거나 pyrealsense2 가 없으면 조용히 False 를 돌려준다(맥 개발·dry-run).
    """
    try:
        import pyrealsense2 as rs
    except Exception:
        return False
    try:
        devs = rs.context().query_devices()
        if not len(devs):
            return False
        devs[0].hardware_reset()
    except Exception as exc:
        if log:
            log("  hardware_reset 실패(%s) — 그대로 진행" % exc)
        return False
    # 고정 sleep 이 아니라 **장치가 돌아올 때까지 폴링**한다.
    # VM 으로 USB 를 넘기면(UTM) 재열거가 호스트보다 오래 걸려 5 s 로는 모자란다
    # — 2026-09-21 현장에서 "gyro 200Hz 스트림을 못 열었다 (No device connected)" 로 죽었다.
    deadline = time.time() + max(float(wait_s), RESET_WAIT_MAX_S)
    if log:
        log("  hardware_reset — 장치가 돌아올 때까지 최대 %.0fs 기다린다..."
            % RESET_WAIT_MAX_S)
    t0 = time.time()
    time.sleep(1.0)                      # 리셋 직후엔 옛 핸들이 남아 있을 수 있다
    while time.time() < deadline:
        try:
            if len(rs.context().query_devices()):
                dt = time.time() - t0
                time.sleep(1.0)          # 열거된 뒤 스트림이 준비될 여유
                if log:
                    log("  장치 복귀 %.1fs" % dt)
                return True
        except Exception:
            pass
        time.sleep(0.5)
    if log:
        log("  !! %.0fs 안에 장치가 안 돌아왔다 — UTM USB 아이콘에서 다시 넘기거나 "
            "`--no-reset` 으로 돌려라" % RESET_WAIT_MAX_S)
    return False


class FrameStamps:
    """프레임 하나의 시각들. jsonl 한 줄이 되는 값."""

    __slots__ = ("t_capture", "t_arrival", "t_detect_done", "t_decide",
                 "t_cmd_set", "t_cmd_tx", "t_raw", "domain", "delta_fs_ms",
                 "row_px", "row_corr_ms", "stale", "skew", "backward",
                 "dt_s", "frame_number", "dropped_before", "exposure_us")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))
        self.stale = bool(kw.get("stale", False))
        self.skew = bool(kw.get("skew", False))
        self.backward = bool(kw.get("backward", False))

    @property
    def latency_s(self):
        """L = t_arrival − t_capture."""
        if self.t_capture is None or self.t_arrival is None:
            return None
        return self.t_arrival - self.t_capture

    @property
    def usable(self):
        """이 프레임으로 판단·정지해도 되나."""
        return not (self.stale or self.skew or self.backward)

    def as_dict(self):
        d = {k: getattr(self, k) for k in self.__slots__}
        d["L_ms"] = None if self.latency_s is None else round(self.latency_s * 1000.0, 3)
        d["usable"] = self.usable
        return d


class TimingSession:
    """세션 하나의 시계. 프레임마다 stamp() 를 부르고, 끝에 summary() 를 남긴다."""

    def __init__(self, t_line_us=None, delta_fs_ms=None, row_ref=0.0,
                 stale_s=STALE_S, log=None):
        self.t_line_s = None if t_line_us in (None, 0) else float(t_line_us) * 1e-6
        self.delta_fs_s = None if delta_fs_ms is None else float(delta_fs_ms) * 1e-3
        self.row_ref = float(row_ref)
        self.stale_s = float(stale_s)
        self.log = log
        self.domain_color = None
        self.domain_gyro = None
        self.t_global_color = None      # 컬러가 GLOBAL 로 바뀐 시각
        self.t_global_gyro = None
        self.t_open = time.time()
        self._delta_fs_samples = []
        self._last_capture = None
        self.n = 0
        self.n_stale = 0
        self.n_skew = 0
        self.n_backward = 0
        self.n_dt_odd = 0
        self._L = []
        self._dt = []

    def apply_row(self, stamps, row_px):
        """검출이 끝난 뒤 **태그 중심 행**으로 롤링셔터 보정을 다시 건다.

        stamp() 는 프레임을 받은 순간 불리므로 태그가 몇 행에 있는지 모른다. 1080행
        컬러에서 T_line 30 µs 면 화면 위/아래가 32 ms 차이라, 이걸 안 걸면 위쪽 태그의
        t_capture 가 그만큼 틀린다. `t_line_us` 가 캘리브에 없으면 아무것도 안 한다.
        """
        if stamps is None or self.t_line_s is None or row_px is None:
            return stamps
        try:
            row = float(row_px)
        except (TypeError, ValueError):
            return stamps
        prev = stamps.row_corr_ms or 0.0
        corr = (row - self.row_ref) * self.t_line_s
        stamps.row_px = row
        stamps.row_corr_ms = corr * 1000.0
        if stamps.t_capture is not None:
            stamps.t_capture += corr - prev * 1e-3
            if stamps.t_arrival is not None:
                stamps.stale = (stamps.t_arrival - stamps.t_capture) > self.stale_s
        return stamps

    # ── 도메인 ─────────────────────────────────────────────────────────────
    @staticmethod
    def is_global(domain):
        return domain is not None and "global" in str(domain).lower()

    def note_domain(self, which, domain):
        """컬러/자이로 도메인 문자열을 받아 전환 시각을 기록한다."""
        attr = "domain_%s" % which
        prev = getattr(self, attr)
        setattr(self, attr, domain)
        if self.is_global(domain) and getattr(self, "t_global_%s" % which) is None:
            setattr(self, "t_global_%s" % which, time.time())
            if self.log and not self.is_global(prev):
                self.log("  %s 타임스탬프가 GLOBAL 로 전환됨 (+%.1fs)"
                         % (which, time.time() - self.t_open))
        return domain

    @property
    def global_ready(self):
        """컬러·자이로 **둘 다** GLOBAL 인가. 아니면 실주행 금지."""
        return self.is_global(self.domain_color) and self.is_global(self.domain_gyro)

    def global_missing(self):
        out = []
        if not self.is_global(self.domain_color):
            out.append("컬러 도메인 %s" % self.domain_color)
        if not self.is_global(self.domain_gyro):
            out.append("자이로 도메인 %s" % self.domain_gyro)
        return out

    # ── 프레임 ─────────────────────────────────────────────────────────────
    def observe_delta_fs(self, meta):
        """메타에서 Δ_FS = (FRAME_TS − SENSOR_TS) 를 뽑는다 [s]. 없으면 None."""
        if not meta:
            return None
        f = meta.get("frame_timestamp")
        s = meta.get("sensor_timestamp")
        if f is None or s is None:
            return None
        d = (float(f) - float(s)) * 1e-6        # 메타는 µs
        if not (-1.0 < d < 1.0):                # 말이 안 되는 값은 버린다
            return None
        if len(self._delta_fs_samples) < 600:
            self._delta_fs_samples.append(d)
        return d

    @property
    def delta_fs_measured_s(self):
        if not self._delta_fs_samples:
            return None
        return statistics.median(self._delta_fs_samples)

    def stamp(self, ts_device_s, t_arrival=None, meta=None, domain=None,
              row_px=None, frame_number=None, dropped_before=0):
        """프레임 하나의 시각을 계산한다. **pyrealsense2 없이도 돈다**(가짜 소스 공용)."""
        t_arrival = time.time() if t_arrival is None else float(t_arrival)
        if domain is not None:
            self.note_domain("color", domain)

        d_fs = self.observe_delta_fs(meta)
        if d_fs is None:
            d_fs = self.delta_fs_s if self.delta_fs_s is not None else self.delta_fs_measured_s
        t_cap = float(ts_device_s) - (d_fs or 0.0)

        row_corr = 0.0
        if self.t_line_s and row_px is not None:
            row_corr = (float(row_px) - self.row_ref) * self.t_line_s
            t_cap += row_corr

        L = t_arrival - t_cap
        stale = L > self.stale_s or L < -self.stale_s
        skew = abs(t_cap - t_arrival) > SKEW_MAX_S
        backward = self._last_capture is not None and t_cap < self._last_capture
        dt = None if self._last_capture is None else t_cap - self._last_capture
        if dt is not None and dt > 0:
            self._dt.append(dt)
            if abs(dt - DT_NOMINAL_S) > DT_TOL_S:
                self.n_dt_odd += 1
        self._last_capture = t_cap

        self.n += 1
        self._L.append(L)
        self.n_stale += bool(stale)
        self.n_skew += bool(skew)
        self.n_backward += bool(backward)

        return FrameStamps(t_capture=t_cap, t_arrival=t_arrival, t_raw=float(ts_device_s),
                           domain=domain if domain is not None else self.domain_color,
                           delta_fs_ms=None if d_fs is None else d_fs * 1000.0,
                           row_px=row_px, row_corr_ms=row_corr * 1000.0,
                           stale=stale, skew=skew, backward=backward,
                           dt_s=dt, frame_number=frame_number,
                           dropped_before=int(dropped_before or 0),
                           exposure_us=(meta or {}).get("actual_exposure"))

    # ── 요약 ───────────────────────────────────────────────────────────────
    @staticmethod
    def _pct(vals, q):
        if not vals:
            return None
        s = sorted(vals)
        i = min(len(s) - 1, max(0, int(round(q * (len(s) - 1)))))
        return s[i]

    def summary(self):
        L = [x * 1000.0 for x in self._L]
        dt = [x * 1000.0 for x in self._dt]
        return {"n": self.n,
                "L_ms_median": statistics.median(L) if L else None,
                "L_ms_p99": self._pct(L, 0.99),
                "L_ms_max": max(L) if L else None,
                "dt_ms_median": statistics.median(dt) if dt else None,
                "dt_ms_max": max(dt) if dt else None,
                "dt_odd": self.n_dt_odd,
                "stale": self.n_stale, "skew": self.n_skew, "backward": self.n_backward,
                "delta_fs_ms": (None if self.delta_fs_measured_s is None
                                else self.delta_fs_measured_s * 1000.0),
                "t_line_us": None if self.t_line_s is None else self.t_line_s * 1e6,
                "domain_color": self.domain_color, "domain_gyro": self.domain_gyro,
                "t_global_color_s": (None if self.t_global_color is None
                                     else self.t_global_color - self.t_open),
                "t_global_gyro_s": (None if self.t_global_gyro is None
                                    else self.t_global_gyro - self.t_open)}


class LoopTick:
    """판단 루프가 한 바퀴 도는 간격 [ms]. VM 에서 이게 튀면 사건 원인이 된다."""

    def __init__(self, name="loop"):
        self.name = name
        self._last = None
        self.ticks = []

    def tick(self, t=None):
        t = time.time() if t is None else t
        dt = None if self._last is None else (t - self._last) * 1000.0
        self._last = t
        if dt is not None:
            self.ticks.append(dt)
        return dt

    def summary(self):
        if not self.ticks:
            return {"n": 0, "p99": None, "max": None, "median": None}
        s = sorted(self.ticks)
        i = min(len(s) - 1, int(round(0.99 * (len(s) - 1))))
        return {"n": len(s), "median": statistics.median(s), "p99": s[i], "max": s[-1]}


__all__ = ["STALE_S", "GLOBAL_WAIT_S", "SKEW_MAX_S", "FrameStamps", "TimingSession",
           "LoopTick", "hardware_reset"]
