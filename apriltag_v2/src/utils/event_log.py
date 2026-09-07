import glob
import json
import os
import sys
import time

def record_event(record_dir, kind, **fields):
    if not record_dir:
        return
    entry = dict(fields, ts=time.time())
    try:
        os.makedirs(record_dir, exist_ok=True)
        path = os.path.join(record_dir, '%s.jsonl' % kind)
        with open(path, 'a', encoding='utf-8') as f:
            f.write(json.dumps(entry, ensure_ascii=False) + '\n')
    except OSError:
        pass
_CONFIG_MODULES = ('config.control', 'config.detection', 'config.imu')

def _git_commit():
    import subprocess
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        rev = subprocess.run(['git', '-C', here, 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True, timeout=3).stdout.strip()
        if not rev:
            return None
        dirty = subprocess.run(['git', '-C', here, 'status', '--porcelain'], capture_output=True, text=True, timeout=3).stdout.strip()
        return rev + ('+dirty' if dirty else '')
    except Exception:
        return None

def _jsonable(v):
    if v is None or isinstance(v, (bool, int, float, str)):
        return True
    if isinstance(v, (list, tuple)):
        return all((_jsonable(x) for x in v))
    return False

def snapshot_config(record_dir, argv=None):
    if not record_dir:
        return None
    import importlib
    snap = {'ts': time.time(), 'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'git': _git_commit(), 'argv': list(sys.argv if argv is None else argv)}
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
        values[modname.split('.')[-1]] = vals
    snap['config'] = values
    try:
        os.makedirs(record_dir, exist_ok=True)
        path = os.path.join(record_dir, 'config.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(snap, f, ensure_ascii=False, indent=2, sort_keys=True)
        return path
    except OSError:
        return None

def read_events(record_dir, kind=None):
    events = []
    pattern = '%s.jsonl' % (kind or '*')
    for path in glob.glob(os.path.join(record_dir, pattern)):
        name = os.path.splitext(os.path.basename(path))[0]
        with open(path, encoding='utf-8') as f:
            for line in f:
                if line.strip():
                    events.append(dict(json.loads(line), kind=name))
    events.sort(key=lambda e: e.get('ts', 0.0))
    return events
