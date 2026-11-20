"""기록. 주행 > 기록 — 기록 때문에 차가 멈추거나 늦어지는 일은 없다 (plan 6-6).

판단 루프는 큐에 넣기만 하고, 별도 스레드가 디스크에 쓴다.
예외는 밖으로 안 나가되 세고 마지막 원인을 남긴다. 정지는 쓰기 스레드를 먼저 세워
순서를 지킨 뒤 바로 쓴다 — 곧 죽을 수 있으니 큐에 남겨두면 사라진다.
"""
import json
import math
import queue
import threading
import time
from pathlib import Path

from config import control as C

#: 정지 이유는 정해진 목록에서만 — 자유 문자열이면 나중에 셀 수 없다
STOP_REASONS = ("normal", "emergency", "ctrl_c", "deadman", "fork_guard",
                "gyro_stale", "tag_lost", "timeout", "exception", "no_converge")


def _clean(v):
    """NaN·inf 는 JSON 규격에 없다. 그냥 쓰면 깨진 파일이 된다."""
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    if isinstance(v, dict):
        return {str(k): _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    if isinstance(v, (str, int, bool)) or v is None:
        return v
    return str(v)


class Recorder:
    def __init__(self, session_dir, flush_s=None, capacity=None, max_mb=None):
        self.dir = Path(session_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.queue = queue.Queue(capacity or C.LOG_QUEUE)
        self.flush_s = flush_s or C.LOG_FLUSH_S
        self.max_bytes = (max_mb or C.SESSION_MAX_MB) * 1_000_000
        self.log_errors = 0
        self.last_error = None          # 세기만 하면 나중에 못 고친다
        self.dropped_rows = 0
        self.written = 0
        self.bytes = 0
        self._files = {}
        self._lock = threading.Lock()   # _files 와 쓰기를 두 스레드가 만진다
        self._done = threading.Event()
        self._thread = threading.Thread(target=self._run, name="Record", daemon=True)
        self._thread.start()

    # ── 부르는 쪽 ───────────────────────────────────────────────────
    def frame(self, **row):
        """프레임 한 줄. 큐가 차면 이것부터 버린다."""
        self._put("frame", row, droppable=True)

    def event(self, kind, **row):
        """회전·직진·결정 같은 사건. 절대 안 버린다."""
        self._put("event", dict(row, event=kind), droppable=False)

    def imu(self, rows):
        """자이로·가속도 **원시** 샘플 여러 줄. [(kind, t, x, y, z), ...] 또는 dict.

        원시로 남기는 이유: 판정을 기록 시점에 박으면 그 판정이 틀렸을 때 되돌릴 수 없다.
        """
        for r in rows or ():
            if not isinstance(r, dict):
                kind, t, x, y, z = r
                r = {"s": kind, "t": t, "x": x, "y": y, "z": z}
            self._put("imu", r, droppable=True)

    def can(self, **row):
        """명령 종류·바이트·write 시각·TXACK."""
        row.setdefault("ts", time.time())
        self._put("can", row, droppable=True)

    def snapshot(self, argv=None, extra=None):
        """이 주행에 쓴 설정 전부를 config.json 으로. 나중에 재현·대조에 쓴다."""
        import importlib
        import subprocess
        body = {"ts": time.time(), "argv": list(argv or []),
                "git": _git(), "host": _host()}
        for name in ("control", "detection", "imu"):
            mod = importlib.import_module("config." + name)
            body[name] = {k: _clean(getattr(mod, k)) for k in dir(mod)
                          if k.isupper() and not k.startswith("_")}
        body.update(_clean(extra or {}))
        body["hash"] = _hash(body)
        try:
            (self.dir / "config.json").write_text(
                json.dumps(body, ensure_ascii=False, indent=1))
        except Exception as e:
            self._fail(e)
        return body

    def stop(self, why, **row):
        """정지 기록. 쓰기 스레드를 세우고 남은 걸 비운 뒤 바로 쓴다."""
        if why not in STOP_REASONS:
            why = "exception"
        self._halt()
        try:
            self._drain()
            self._write("event", dict(row, event="stop", why=why, t=time.time()))
            self._flush()
        except Exception as e:
            self._fail(e)

    def close(self, outcome=None):
        self._halt()
        try:
            self._drain()
            self._flush()
            if outcome is not None:
                (self.dir / "outcome.json").write_text(
                    json.dumps(_clean(outcome), ensure_ascii=False, indent=1))
            # 정상 종료 표시. 없으면 다음 실행이 ended_unexpectedly 로 잡아낸다
            (self.dir / "closed").write_text(str(time.time()))
        except Exception as e:
            self._fail(e)
        with self._lock:
            for f in self._files.values():
                try:
                    f.close()
                except Exception as e:
                    self._fail(e)
            self._files.clear()
        return self.summary()

    def summary(self):
        return {"written": self.written, "log_errors": self.log_errors,
                "last_error": self.last_error, "dropped_rows": self.dropped_rows,
                "bytes": self.bytes}

    # ── 안쪽 ────────────────────────────────────────────────────────
    def _fail(self, exc):
        self.log_errors += 1
        self.last_error = "%s: %s" % (type(exc).__name__, exc)

    def _halt(self):
        self._done.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _put(self, kind, row, droppable):
        try:
            self.queue.put_nowait((kind, row))
        except queue.Full:
            if droppable:
                self.dropped_rows += 1
            else:
                # 이벤트 큐가 찼다 = 뭔가 크게 잘못됐다. 멈추는 게 안전한 행동이다
                self._fail(RuntimeError("기록 큐가 가득 찼다"))
                raise RuntimeError("기록 큐가 가득 찼다 — 정지한다")
        except Exception as e:
            self._fail(e)

    def _run(self):
        last = time.monotonic()
        while not self._done.is_set():
            self._drain(block_s=0.05)
            if time.monotonic() - last >= self.flush_s:
                self._flush()
                last = time.monotonic()

    def _drain(self, block_s=0.0):
        while True:
            try:
                kind, row = (self.queue.get(timeout=block_s) if block_s
                             else self.queue.get_nowait())
            except queue.Empty:
                return
            except Exception as e:
                self._fail(e)
                return
            try:
                self._write(kind, row)
            except Exception as e:
                self._fail(e)

    def _write(self, kind, row):
        line = json.dumps(_clean(row), ensure_ascii=False) + "\n"
        with self._lock:
            if kind == "frame" and self.bytes >= self.max_bytes:
                self.dropped_rows += 1      # 폴더 상한. 프레임부터 솎는다
                return
            f = self._files.get(kind)
            if f is None:
                f = self._files[kind] = open(self.dir / ("%s.jsonl" % kind), "a",
                                             encoding="utf-8")
            f.write(line)
            self.written += 1
            self.bytes += len(line)

    def _flush(self):
        with self._lock:
            for f in self._files.values():
                try:
                    f.flush()
                except Exception as e:
                    self._fail(e)


def _git():
    import subprocess
    try:
        r = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                           capture_output=True, text=True, timeout=3)
        d = subprocess.run(["git", "status", "--porcelain"],
                           capture_output=True, text=True, timeout=3)
        return {"commit": r.stdout.strip(), "dirty": bool(d.stdout.strip())}
    except Exception:
        return {"commit": None, "dirty": None}


def _host():
    import platform
    return "%s-%s" % (platform.system(), platform.machine())


def _hash(body):
    import hashlib
    raw = json.dumps({k: v for k, v in body.items() if k != "hash"},
                     ensure_ascii=False, sort_keys=True)
    return hashlib.sha256(raw.encode()).hexdigest()[:12]


def unfinished_sessions(runs_root):
    """닫힘 기록이 없는 세션 — 강제 종료·정전으로 끝난 것. 품질에서 자동 탈락시킨다."""
    root = Path(runs_root)
    if not root.is_dir():
        return []
    return sorted(d.name for d in root.iterdir()
                  if d.is_dir() and not (d / "closed").exists())
