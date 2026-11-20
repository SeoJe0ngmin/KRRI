"""경로를 찾는 유일한 곳. 모든 tool 의 첫 줄이 이것만 부른다.

parents[N] 을 세지 않는다 — 파일을 옮기면 깨지기 때문(2026-09-21 사고).
"""
import os
import sys
from datetime import datetime
from pathlib import Path


def repo_root(start=None):
    """config/ 와 src/ 가 같이 있는 폴더를 위로 올라가며 찾는다."""
    here = Path(start or __file__).resolve()
    for d in [here] + list(here.parents):
        if (d / "config").is_dir() and (d / "src").is_dir():
            return d
    raise RuntimeError("apriltag 루트(config/ 와 src/ 가 같이 있는 폴더)를 못 찾았다: %s" % here)


ROOT = repo_root()


def setup():
    """import 경로를 잡는다. 여러 번 불러도 안전."""
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    return ROOT


def work_root(override=None, site=None):
    """기록 저장 루트 = <기준>/<장소>.  기준: --out > KRRI_WORK_ROOT > 리포 안 work_dirs/ · 장소: config/site.py SITE (2026-10-02).
    장소마다 기록이 따로 쌓이고, 학습 파일(config/learned/<장소>.json)도 그 장소 기록에서만 만든다."""
    if site is None:
        from config.site import SITE as site            # setup() 뒤에 부른다 — config 를 찾을 수 있다
    for cand in (override, os.environ.get("KRRI_WORK_ROOT")):
        if cand:
            return Path(cand).expanduser().resolve() / site
    return ROOT / "work_dirs" / site


def check_writable(path, need_mb=200):
    """출발 전 확인 — 붙어 있나·쓸 수 있나·여유가 있나. 주행 중 실패보다 낫다."""
    path = Path(path)
    try:
        path.mkdir(parents=True, exist_ok=True)
        probe = path / ".write_probe"
        probe.write_text("ok")
        probe.unlink()
    except Exception as e:
        raise SystemExit("!! 기록 경로에 쓸 수 없다: %s (%s)" % (path, e))
    free_mb = os.statvfs(path).f_bavail * os.statvfs(path).f_frsize / 1e6
    if free_mb < need_mb:
        raise SystemExit("!! 기록 경로 여유가 %.0f MB 뿐이다 (%d MB 필요): %s"
                         % (free_mb, need_mb, path))
    return path


def new_session_dir(root, kind, need_mb=200):
    """기록 폴더 하나를 만든다 — <work_root>/<날짜>/<종류>/<시각>/ = work_dirs/<장소>/<날짜>/<종류>/<시각>/ (v5, 2026-10-01·02 사용자 결정).
    kind = "run" / "calibrate". 그 종류 폴더에 쓸 수 있는지·여유가 있는지 먼저 본다 (check_writable)."""
    now = datetime.now()
    base = check_writable(Path(root) / now.strftime("%Y%m%d") / kind, need_mb=need_mb)
    d = base / now.strftime("%H%M%S")
    d.mkdir(parents=True, exist_ok=True)
    return d


def sessions(root, kind=None):
    """기록 폴더들을 **시간 순서로** (날짜·시각이 폴더 이름이라 이름이 곧 시간). kind 를 주면 그 종류만 (run / calibrate).
    수정 시각으로 고르지 않는다 — 폴더를 복사하면 수정 시각이 바뀐다."""
    out = []
    for day in Path(root).glob("[0-9]" * 8):
        if not day.is_dir():
            continue
        for kd in day.iterdir():
            if not kd.is_dir() or (kind is not None and kd.name != kind):
                continue
            for d in kd.iterdir():
                if d.is_dir() and len(d.name) == 6 and d.name.isdigit():
                    out.append(d)
    return sorted(out, key=session_key)


def session_key(d):
    """시간 순서 열쇠 — 20261001_151500"""
    d = Path(d)
    return "%s_%s" % (d.parent.parent.name, d.name)


def session_name(d):
    """기록 폴더를 사람이 읽는 이름으로 — kw/20261001/run/151500 (장소/날짜/종류/시각)"""
    d = Path(d)
    return "%s/%s/%s/%s" % (d.parent.parent.parent.name, d.parent.parent.name, d.parent.name, d.name)
