"""도킹 중 벌어지는 일을 JSON Lines 로 기록하는 곳.

record_dir(실행마다 새 폴더) 아래에 **종류별로 파일을 나눠** 쌓는다 —
종류마다 열(스키마)이 달라서 한 파일에 섞으면 표로 펴 보기 불편하다.

    measure.jsonl    lateral/forward/heading 등 측정값  (dock/dock_live 가 기록)
                     30프레임을 중앙값 하나로 누른 뒤라 사이클당 한 줄이다
    rotation.jsonl   목표각/실제각/오버슈트/걸린시간     (rot_control.rotate_to 가 기록)
    drive.jsonl      전진/후진 방향/명령시간             (CanDriver._hold 가 기록)
    config.json      이 주행에 쓴 config 값 전체         (run.py 가 시작 때 snapshot_config)

모든 줄에 ts(초, epoch)가 붙으므로 시간순으로 다시 합쳐 볼 수 있다 —
read_events() 가 그 일을 한다. run.py 의 --record-events 로 켠다.
"""
import glob
import json
import os
import sys
import time


def record_event(record_dir, kind, **fields):
    """record_dir/<kind>.jsonl 에 한 줄 추가. record_dir 가 None 이면 아무 일도 안 한다.

    실패해도 예외를 던지지 않는다 — 기록 때문에 도킹이 멈추면 안 된다.
    """
    if not record_dir:
        return
    entry = dict(fields, ts=time.time())
    try:
        os.makedirs(record_dir, exist_ok=True)
        path = os.path.join(record_dir, "%s.jsonl" % kind)
        with open(path, "a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")
    except OSError:
        pass


# ── config 스냅샷 ───────────────────────────────────────────────────────────
# 실험할 때마다 config 를 손대므로(허용치·문턱·부호 등), "이 주행은 어떤 값이었나"
# 를 나중에 되짚으려면 그 순간의 config 를 기록과 같이 남겨야 한다.
# record_dir/config.json 하나에 세 모듈(control/detection/imu)의 대문자 상수를
# 통째로 박고, git 커밋·시각·실행 인자도 함께 남긴다.
_CONFIG_MODULES = ("config.control", "config.detection", "config.imu")


def _git_commit():
    """현재 체크아웃된 커밋(짧은 해시). 미커밋 변경이 있으면 +dirty. 실패하면 None."""
    import subprocess
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        rev = subprocess.run(["git", "-C", here, "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=3).stdout.strip()
        if not rev:
            return None
        dirty = subprocess.run(["git", "-C", here, "status", "--porcelain"],
                              capture_output=True, text=True, timeout=3).stdout.strip()
        return rev + ("+dirty" if dirty else "")
    except Exception:
        return None


def _jsonable(v):
    if v is None or isinstance(v, (bool, int, float, str)):
        return True
    if isinstance(v, (list, tuple)):
        return all(_jsonable(x) for x in v)
    return False


def snapshot_config(record_dir, argv=None):
    """실행 시점의 config 상수를 record_dir/config.json 으로 저장하고 경로를 돌려준다.

    control/detection/imu 세 모듈의 **대문자 상수**를 자동으로 긁으므로, 새 파라미터를
    추가해도 코드를 안 고쳐도 따라 남는다. 기록처럼 실패해도 예외를 안 던진다 —
    스냅샷 때문에 주행이 막히면 안 된다. record_dir 가 None 이면 아무 일도 안 한다.
    """
    if not record_dir:
        return None
    import importlib
    snap = {"ts": time.time(),
            "time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "git": _git_commit(),
            "argv": list(sys.argv if argv is None else argv)}
    # config.json 안에서 control / detection / imu 를 각각 한 묶음(층)으로 나눈다.
    values = {}
    for modname in _CONFIG_MODULES:
        try:
            m = importlib.import_module(modname)
        except Exception:
            continue
        vals = {}
        for k in dir(m):
            if not k.isupper():
                continue
            v = getattr(m, k)
            if _jsonable(v):
                vals[k] = list(v) if isinstance(v, tuple) else v
        values[modname.split(".")[-1]] = vals   # {"control": {...}, "detection": {...}, "imu": {...}}
    snap["config"] = values
    try:
        os.makedirs(record_dir, exist_ok=True)
        path = os.path.join(record_dir, "config.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(snap, f, ensure_ascii=False, indent=2, sort_keys=True)
        return path
    except OSError:
        return None


def read_events(record_dir, kind=None):
    """기록을 읽는다. kind 를 주면 그 파일만, 안 주면 전부 시간순으로 합쳐서.

    분석할 때 쓴다:
        rotations = read_events(d, "rotation")          # 회전만 표로
        timeline  = read_events(d)                       # 사건 전체를 시간순으로
    합칠 때는 어느 파일에서 왔는지 "kind" 를 붙여 준다.
    """
    events = []
    pattern = "%s.jsonl" % (kind or "*")
    for path in glob.glob(os.path.join(record_dir, pattern)):
        name = os.path.splitext(os.path.basename(path))[0]
        with open(path, encoding="utf-8") as f:
            for line in f:
                if line.strip():
                    events.append(dict(json.loads(line), kind=name))
    events.sort(key=lambda e: e.get("ts", 0.0))
    return events
