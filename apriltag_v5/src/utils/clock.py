"""시각은 여기서만 읽는다. 시계가 섞이면 조용히 전부 틀어진다.

2026-09-21 실주행 로그에서 확인한 것 — 시계가 셋이다:

    카메라 프레임 스탬프   1789968377.5   1970년 기준(epoch). global_time 을 켜면 이렇다
    CAN 송신 시각          (monotonic)    광운대 control.py 는 monotonic_ns 를 쓴다
    우리 계산              2847.3         time.monotonic. 부팅 기준

그냥 빼면 `now - t_capture` 가 **-17억 초**가 된다. 그러면 "사진이 얼마나 낡았나"가
음수라, 미리 끊는 계산이 무의미해지고 태그를 놓쳤을 때 안전망이 영영 안 걸린다.

**카메라 스탬프가 epoch 라고 가정하지 않는다.** 스탬프의 **차이**만 쓰고 기준점 하나를
프레임 몇 장으로 직접 잰다. 그러면 global_time 이 꺼져 있어도(장치 부팅 기준),
첫 프레임 기준 상대시간이어도, NTP 가 벽시계를 끌어당겨도 간격이 안 망가진다.

기준점은 **관측된 지연의 최소값**으로 잡는다. 제일 빨리 온 프레임이 진짜 바닥에
제일 가깝다. 그래서 우리가 아는 노출시각은 진짜보다 "바닥 지연"만큼 늦다 — 그건
상수라 학습된 tau 에 흡수된다(plan 4-10). 창이 찬 뒤에도 **더 빠른 프레임을 보면 기준점을 그쪽으로 당긴다** — 한 방향으로만
움직이고, 당기면 노출시각이 앞으로 가서 "측정이 더 낡았다"가 된다. 즉 **안전한 쪽으로만**
틀린다(더 일찍 끊는다). 반대로 얼려 버리면, 창 안이 우연히 다 느렸을 때 그 오차가
"측정이 더 싱싱하다"로 굳어 늦게 끊는다.
"""
import threading
import time

WARMUP_FRAMES = 30      # 기준점을 잡을 프레임 수. 30 fps 면 1 초
MAX_AGE_S = 0.30        # 이보다 낡거나(또는 미래인) 측정은 못 쓴다.
                        # 광운대 MAX_MEASUREMENT_AGE_SEC


def now():
    """우리 시계. 도중에 벽시계가 튀어도 간격이 안 망가진다."""
    return time.monotonic()


class CameraClock:
    """카메라 스탬프 -> 우리 시계. 카메라 하나당 하나."""

    def __init__(self, warmup=WARMUP_FRAMES):
        self.warmup = int(warmup)
        self._lock = threading.Lock()
        self._lat = []          # 도착(우리시계) - 스탬프(카메라시계)
        self._offset = None
        self._frozen = False
        self._pulled = None     # 창이 찬 뒤 기준점을 얼마나 더 당겼나 (진단용)
        self.n = 0

    # ── 프레임이 올 때 ──────────────────────────────────────────────
    def see(self, ts_camera, t_arrival=None):
        """프레임마다 부른다. **우리 시계의 노출 시각**을 돌려준다."""
        a = now() if t_arrival is None else float(t_arrival)
        d = a - float(ts_camera)
        with self._lock:
            self.n += 1
            if len(self._lat) < self.warmup:
                self._lat.append(d)
                self._offset = min(self._lat)
                self._frozen = len(self._lat) >= self.warmup
            elif d < self._offset:
                # 한 방향으로만 당긴다 (안전한 쪽). 얼마나 당겼는지는 진단에 남긴다
                self._pulled = (self._pulled or 0.0) + (self._offset - d)
                self._offset = d
            off = self._offset
        return float(ts_camera) + off

    @property
    def ready(self):
        return self._frozen

    # ── 확인 ────────────────────────────────────────────────────────
    def check(self, log=print):
        """출발 전 검사. 통과 못 하면 이유를 돌려준다(빈 문자열이면 통과)."""
        with self._lock:
            n, frozen, lat = self.n, self._frozen, list(self._lat)
        if not frozen:
            return "카메라 시계 기준점이 아직 안 잡혔다 (프레임 %d / %d 장 필요)" % (n, self.warmup)
        lo, hi = min(lat), max(lat)
        if log:
            log("  카메라 시계: 기준점 %.6f s, 지연 산포 %.1f~%.1f ms (%d 장)"
                % (self._offset, (lo - lo) * 1000, (hi - lo) * 1000, len(lat)))
        return ""

    def report(self):
        with self._lock:
            lat = sorted(self._lat)
            return {"n": self.n, "frozen": self._frozen, "offset_s": self._offset,
                    "latency_spread_ms": None if not lat else
                    round((lat[-1] - lat[0]) * 1000, 2),
                    "pulled_after_warmup_ms": None if self._pulled is None else
                    round(self._pulled * 1000, 2)}


def age_ok(age_s):
    """측정이 쓸 만한 나이인가. **음수면 시계가 틀린 것** — 조용히 넘기면 안 된다."""
    return -1e-3 <= age_s <= MAX_AGE_S


def wall_from(t_mono, _anchor=[]):
    """기록에 사람이 읽을 벽시계를 같이 남길 때만. 계산에는 쓰지 않는다."""
    if not _anchor:
        _anchor.append((time.monotonic(), time.time()))
    m0, w0 = _anchor[0]
    return w0 + (float(t_mono) - m0)
