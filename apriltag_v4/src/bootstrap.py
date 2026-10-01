"""경로를 찾는 유일한 곳. 모든 tool 의 첫 줄이 이것만 부른다.

parents[N] 을 세지 않는다 — 파일을 옮기면 깨지기 때문(2026-09-21 사고).
"""
import os
import sys
from pathlib import Path


def repo_root(start=None):
    """config/ 와 src/ 가 같이 있는 폴더를 위로 올라가며 찾는다."""
    here = Path(start or __file__).resolve()
    for d in [here] + list(here.parents):
        if (d / "config").is_dir() and (d / "src").is_dir():
            return d
    raise RuntimeError("apriltag_v4 루트를 못 찾았다: %s" % here)


ROOT = repo_root()


def setup():
    """import 경로를 잡는다. 여러 번 불러도 안전."""
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    return ROOT


def work_root(override=None):
    """기록 저장 루트.  --out  >  KRRI_WORK_ROOT  >  리포 안 work_dirs/"""
    for cand in (override, os.environ.get("KRRI_WORK_ROOT")):
        if cand:
            return Path(cand).expanduser().resolve()
    return ROOT / "work_dirs"


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
