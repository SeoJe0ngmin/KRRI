#!/usr/bin/env bash
# Ship the whole system. Preserve field configuration/results unless requested.
set -euo pipefail
RD_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RD_HOST=jetson@192.168.55.1
RD_DRY_RUN=0
RD_WITH_CONFIG=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --host)
            [[ $# -ge 2 ]] || { echo '--host needs a value' >&2; exit 2; }
            RD_HOST="$2"; shift 2 ;;
        --with-config) RD_WITH_CONFIG=1; shift ;;
        --dry-run) RD_DRY_RUN=1; shift ;;
        -h|--help)
            echo 'Usage: bash update_jetson.sh [--host jetson@IP] [--with-config] [--dry-run]'
            echo 'Default USB: 192.168.55.1. Update all four versions and shared tools.'
            echo 'Existing Jetson common.json is preserved; --with-config replaces it with the Mac copy.'
            exit 0 ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
done
[[ "$(uname -s)" == Darwin ]] || { echo '맥북 터미널에서 실행하세요.' >&2; exit 2; }
[[ "$RD_HOST" =~ ^[A-Za-z0-9][A-Za-z0-9_.@-]*$ ]] || { echo '잘못된 SSH 대상입니다.' >&2; exit 2; }
[[ -f "$RD_ROOT/configs/common.json" && -f "$RD_ROOT/preflight.py" ]]
for RD_N in 1 2 3 4; do test -f "$RD_ROOT/robust_docking_v$RD_N/version.json"; done
command -v rsync >/dev/null
command -v ssh >/dev/null
echo "대상: $RD_HOST:/home/jetson/krri/docking_system/ (v1~v4 전체)"
echo '업데이트가 끝날 때까지 Jetson의 측정/주행/보정을 시작하지 마세요.'
if [[ "$RD_WITH_CONFIG" == 1 ]]; then
    echo '공통 설정도 맥북 사본으로 교체합니다. Jetson에서 측정·입력한 기존 값도 교체됩니다.'
else
    echo 'Jetson의 기존 common.json과 측정 기록은 유지합니다. 최초 설치 때만 설정을 복사합니다.'
fi
ssh -o ConnectTimeout=10 "$RD_HOST" bash -s -- "$RD_DRY_RUN" <<'REMOTE'
set -euo pipefail
command -v rsync >/dev/null
command -v python3 >/dev/null
if pgrep -u "$(id -u)" -f '[p]ython[^ ]*.*(-m[[:space:]]+docking([[:space:].]|$)|preflight[.]py)' >/dev/null; then
    echo 'docking/점검 실행 중입니다. 종료 후 다시 업데이트하세요.' >&2
    exit 1
fi
test -d /home/jetson/krri
if [[ "$1" != 1 ]]; then mkdir -p /home/jetson/krri/docking_system/configs; fi
REMOTE
RD_FLAGS=(-rlptzc --itemize-changes --delay-updates
    --exclude='/calibration/' --exclude='work/' --exclude='.git/'
    --exclude='.venv/' --exclude='venv/' --exclude='__pycache__/'
    --exclude='*.pyc' --exclude='.DS_Store')
if [[ "$RD_DRY_RUN" == 1 ]]; then RD_FLAGS+=(--dry-run); fi
RD_SSH='ssh -o ConnectTimeout=10 -o ServerAliveInterval=15 -o ServerAliveCountMax=3'
RD_DEST="$RD_HOST:/home/jetson/krri/docking_system"
# No broad --delete or migration of measurements; prune named retired files below.
if [[ "$RD_WITH_CONFIG" == 1 ]]; then
    rsync "${RD_FLAGS[@]}" -e "$RD_SSH" "$RD_ROOT/" "$RD_DEST/"
else
    rsync "${RD_FLAGS[@]}" --exclude='/configs/common.json' -e "$RD_SSH" "$RD_ROOT/" "$RD_DEST/"
    rsync "${RD_FLAGS[@]}" --ignore-existing --relative -e "$RD_SSH" "$RD_ROOT/./configs/common.json" "$RD_DEST/"
fi
if [[ "$RD_DRY_RUN" == 1 ]]; then
    echo '미리보기 완료. 파일을 전송하거나 삭제하지 않았습니다.'
    exit 0
fi
ssh -o ConnectTimeout=10 "$RD_HOST" 'bash -s' <<'REMOTE'
set -euo pipefail
cd /home/jetson/krri/docking_system
python3 -B - <<'PY'
import ast
from pathlib import Path
# Retired stock files only; never delete arbitrary field files or follow symlinks.
retired = [
    'robust_docking_v1/configs/demo.json',
    'robust_docking_v1/configs/demo_sidestep.json',
    'robust_docking_v1/docking/variants.py',
    'robust_docking_v1/scripts/migrate_work.py',
    'robust_docking_v1/scripts/legacy_work_cleanup.json',
    'robust_docking_v1/tests/test_work_migration.py',
    'robust_docking_v1/configs/reference/action_contracts.json',
    'robust_docking_v1/configs/experiments/kw_wall_stop/a_baseline.json',
    'robust_docking_v1/configs/experiments/kw_wall_stop/readiness.json',
    'robust_docking_v1/configs/experiments/kw_wall_stop/c_gyro_vo.json',
    'robust_docking_v1/configs/experiments/kw_wall_stop/b_tag_fusion.json',
    'robust_docking_v1/configs/experiments/kw_park_left_tag/a_baseline.json',
    'robust_docking_v1/configs/experiments/kw_park_left_tag/readiness.json',
    'robust_docking_v1/configs/experiments/kw_park_left_tag/c_gyro_vo.json',
    'robust_docking_v1/configs/experiments/kw_park_left_tag/b_tag_fusion.json',
    'robust_docking_v1/configs/experiments/nano/a_baseline.json',
    'robust_docking_v1/configs/experiments/nano/readiness.json',
    'robust_docking_v1/configs/experiments/nano/c_gyro_vo.json',
    'robust_docking_v1/configs/experiments/nano/b_tag_fusion.json',
    'robust_docking_v2/configs/demo.json',
    'robust_docking_v2/configs/demo_sidestep.json',
    'robust_docking_v2/docking/variants.py',
    'robust_docking_v2/scripts/migrate_work.py',
    'robust_docking_v2/scripts/legacy_work_cleanup.json',
    'robust_docking_v2/tests/test_work_migration.py',
    'robust_docking_v2/configs/reference/action_contracts.json',
    'robust_docking_v2/configs/readiness.json',
    'robust_docking_v2/docs/VERSIONS.md',
    'robust_docking_v3/configs/demo.json',
    'robust_docking_v3/configs/demo_sidestep.json',
    'robust_docking_v3/docking/variants.py',
    'robust_docking_v3/scripts/migrate_work.py',
    'robust_docking_v3/scripts/legacy_work_cleanup.json',
    'robust_docking_v3/tests/test_work_migration.py',
    'robust_docking_v3/configs/reference/action_contracts.json',
    'robust_docking_v3/configs/readiness.json',
    'robust_docking_v3/docs/VERSIONS.md',
    'robust_docking_v4/configs/demo.json',
    'robust_docking_v4/configs/demo_sidestep.json',
    'robust_docking_v4/docking/variants.py',
    'robust_docking_v4/scripts/migrate_work.py',
    'robust_docking_v4/scripts/legacy_work_cleanup.json',
    'robust_docking_v4/tests/test_work_migration.py',
    'robust_docking_v4/configs/reference/action_contracts.json',
    'robust_docking_v4/configs/readiness.json',
    'robust_docking_v4/docs/VERSIONS.md',
]
for relative in retired:
    path = Path(relative)
    if path.is_file() and not path.is_symlink() and not any(p.is_symlink() for p in path.parents):
        path.unlink()
for path in Path('.').rglob('*.py'):
    ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
print('Python 문법 확인 완료')
PY
python3 -B preflight.py sync
REMOTE
echo '업데이트 완료. 센서/CAN은 실행하지 않았습니다.'
