"""Local build hygiene; never changes the clock, source, logs or upstream build."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import time
import uuid

PACKAGES = ('docking_msgs', 'docking_openvins')


def check_clock(source, now=None):
    now = time.time() if now is None else now
    files = [p for p in Path(source).rglob('*') if p.is_file()]
    newest = max(files, key=lambda p: p.stat().st_mtime, default=None)
    if now < 1704067200 or (newest and newest.stat().st_mtime > now + 60):
        shown = datetime.fromtimestamp(now, timezone.utc).isoformat()
        raise RuntimeError(f'Jetson 시계 확인 필요: 현재 UTC {shown}. 맥에서 시간을 맞춘 뒤 다시 빌드하세요. '
                           f'소스 최신 파일: {newest}. 시스템 시간을 자동 변경하지 않았습니다.')


def quarantine_generated(workspace):
    """Move only the two generated package trees, including stale CMake caches."""
    ws = Path(workspace).resolve()
    targets = [ws / base / package for base in ('build', 'install') for package in PACKAGES]
    # Validate everything before moving anything. Do not follow a build/install
    # directory symlink into another workspace or modify a merged installation.
    for base in ('build', 'install'):
        if (ws / base).is_symlink():
            raise RuntimeError(f'별도 확인이 필요한 경로: {ws / base}')
    layout = ws / 'install/.colcon_install_layout'
    if layout.exists() and layout.read_text().strip() != 'isolated':
        raise RuntimeError('이 재빌드는 isolated colcon 설치만 지원합니다.')
    if (ws / 'rebuild_backups').is_symlink():
        raise RuntimeError(f'별도 확인이 필요한 경로: {ws / "rebuild_backups"}')
    backup = ws / 'rebuild_backups' / uuid.uuid4().hex
    for src in targets:
        if src.exists() or src.is_symlink():
            dest = backup / src.relative_to(ws)
            dest.parent.mkdir(parents=True, exist_ok=True)
            src.rename(dest)
    if backup.exists():
        (backup / 'COLCON_IGNORE').touch()
    return backup


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('source', type=Path)
    p.add_argument('--workspace', type=Path)
    a = p.parse_args()
    try:
        check_clock(a.source)
        if a.workspace:
            print('이전 메시지·브리지 산출물 보관:', quarantine_generated(a.workspace))
    except RuntimeError as exc:
        p.exit(2, str(exc) + '\n')
