"""로깅층 — 한 실행 폴더에 종류별 jsonl. (plan 4-1·4-7 "원시 기록만, 판정은 오프라인")

    frame.jsonl    프레임마다 6시각 + tag_px·margin_px·β_px + lateral/forward/heading/
                   tilt/quality/spread + pnp2(두 해의 yaw·pitch·재투영오차)
    imu.jsonl      200 Hz gyro + accel **원시**
    can.jsonl      명령 종류·바이트·호스트 write 시각·TXACK
    events.jsonl   단계·프롬프트 응답·정지판정 등 사람이 읽는 사건
    config.json    snapshot_config(control/detection/imu) + 캘리브 내용·해시

왜 원시인가: `motion_started` 같은 **판정을 기록 시점에 박으면** 그 판정이 틀렸을 때
로그가 통째로 못 쓰게 된다(plan 2-7). 여기서는 숫자만 남기고 판정은
`tools/analyze_first_run.py` 가 오프라인에서 한다.

기록 때문에 주행이 멈추면 안 된다 — 모든 메서드는 예외를 삼킨다. 200 Hz IMU 를
줄마다 flush 하면 I/O 가 루프를 잡아먹으므로 파일 핸들을 열어 두고 버퍼링한다.
"""
import json
import os
import time

KINDS = ("frame", "imu", "can", "events")


def _clean(v):
    """json 으로 쓸 수 있는 꼴로. numpy 스칼라·배열도 받아 준다."""
    if v is None or isinstance(v, (bool, int, float, str)):
        if isinstance(v, float) and v != v:       # NaN → null
            return None
        return v
    if isinstance(v, dict):
        return {str(k): _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    tolist = getattr(v, "tolist", None)
    if tolist is not None:
        try:
            return _clean(tolist())
        except Exception:
            pass
    item = getattr(v, "item", None)
    if item is not None:
        try:
            return _clean(item())
        except Exception:
            pass
    return str(v)


class RunLogger:
    """실행 폴더 하나. 없으면(record_dir=None) 전부 no-op 이라 호출부에 if 가 없다."""

    def __init__(self, record_dir, flush_sec=1.0):
        self.dir = record_dir
        self.flush_sec = float(flush_sec)
        self._files = {}
        self._last_flush = time.time()
        self.counts = {k: 0 for k in KINDS}
        if record_dir:
            try:
                os.makedirs(record_dir, exist_ok=True)
            except OSError:
                self.dir = None

    # ── 내부 ───────────────────────────────────────────────────────────────
    def _f(self, kind):
        if not self.dir:
            return None
        f = self._files.get(kind)
        if f is None:
            try:
                f = open(os.path.join(self.dir, "%s.jsonl" % kind), "a",
                         encoding="utf-8")
            except OSError:
                return None
            self._files[kind] = f
        return f

    def _write(self, kind, row):
        f = self._f(kind)
        if f is None:
            return
        try:
            f.write(json.dumps(_clean(row), ensure_ascii=False) + "\n")
            self.counts[kind] = self.counts.get(kind, 0) + 1
            now = time.time()
            if now - self._last_flush >= self.flush_sec:
                self._last_flush = now
                f.flush()
        except Exception:
            pass

    # ── 쓰기 ───────────────────────────────────────────────────────────────
    def frame(self, stamps=None, **fields):
        """프레임 한 줄. stamps 는 timing.FrameStamps (없어도 된다)."""
        row = dict(stamps.as_dict()) if stamps is not None else {}
        row.update(fields)
        row.setdefault("ts", time.time())
        self._write("frame", row)

    def imu(self, rows):
        """자이로/가속도 원시 샘플 여러 줄. [(kind, t_device, x, y, z), ...]"""
        f = self._f("imu")
        if f is None or not rows:
            return
        try:
            for r in rows:
                if isinstance(r, dict):
                    row = r
                else:
                    kind, t, x, y, z = r
                    row = {"s": kind, "t": t, "x": x, "y": y, "z": z}
                f.write(json.dumps(_clean(row), ensure_ascii=False) + "\n")
                self.counts["imu"] = self.counts.get("imu", 0) + 1
            now = time.time()
            if now - self._last_flush >= self.flush_sec:
                self._last_flush = now
                f.flush()
        except Exception:
            pass

    def can(self, **fields):
        fields.setdefault("ts", time.time())
        self._write("can", fields)

    def event(self, kind, **fields):
        row = dict(fields)
        row["event"] = kind
        row.setdefault("ts", time.time())
        row.setdefault("time", time.strftime("%H:%M:%S"))
        self._write("events", row)

    # ── config.json ────────────────────────────────────────────────────────
    def snapshot(self, calibs=None, argv=None, extra=None):
        """snapshot_config + 캘리브 내용·해시 + 정책 해시를 config.json 으로."""
        if not self.dir:
            return None
        from .event_log import snapshot_config
        add = dict(extra or {})
        if calibs is not None:
            from .calib import snapshot as calib_snapshot
            add["calib"] = calib_snapshot(calibs)
        return snapshot_config(self.dir, argv=argv, extra=add)

    # ── 수명 ───────────────────────────────────────────────────────────────
    def flush(self):
        for f in self._files.values():
            try:
                f.flush()
            except Exception:
                pass

    def close(self):
        for f in self._files.values():
            try:
                f.flush()
                f.close()
            except Exception:
                pass
        self._files = {}

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def summary(self):
        return ", ".join("%s %d줄" % (k, n) for k, n in sorted(self.counts.items()) if n)


__all__ = ["RunLogger", "KINDS"]
