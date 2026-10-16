#!/usr/bin/env bash
# Copy the current code families, keeping Jetson logs and common configuration.
set -euo pipefail
KRRI_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KRRI_HOST=jetson-wired
KRRI_DRY=0
while (($#)); do
  case "$1" in
    --host) KRRI_HOST="${2:?--host 값 필요}"; shift 2 ;;
    --dry-run) KRRI_DRY=1; shift ;;
    -h|--help)
      echo 'update_jetson_all.sh [--host jetson-wired] [--dry-run]'
      echo 'apriltag_v1~v6 + docking_system + docking_systemv2 코드·문서·도식을 복사합니다.'
      echo '기존 Jetson common.json과 측정 기록을 보존하며 대상 파일을 삭제하지 않습니다.'
      exit 0 ;;
    *) echo "알 수 없는 옵션: $1" >&2; exit 2 ;;
  esac
done
[[ "$(uname -s)" == Darwin ]] || { echo '맥북 터미널에서 실행하세요.' >&2; exit 2; }
[[ "$KRRI_HOST" =~ ^[A-Za-z0-9][A-Za-z0-9_.@-]*$ ]] || { echo '올바른 SSH 호스트가 필요합니다.' >&2; exit 2; }
command -v rsync >/dev/null
command -v ssh >/dev/null
KRRI_ITEMS=(apriltag_v1 apriltag_v2 apriltag_v3 apriltag_v4 apriltag_v5 apriltag_v6
  docking_system docking_systemv2 README.md requirements.txt code_explanation.md
  robust_docking_versions.md update_jetson_versions.sh update_jetson_all.sh)
KRRI_SOURCES=()
for KRRI_ITEM in "${KRRI_ITEMS[@]}"; do
  [[ -e "$KRRI_ROOT/$KRRI_ITEM" ]] || { echo "원본이 없습니다: $KRRI_ITEM" >&2; exit 2; }
  KRRI_SOURCES+=("$KRRI_ITEM")
done
# macOS openrsync does not strip the prefix before /./ with --relative.
# Use actual relative source paths so the destination has krri/<code family>.
cd "$KRRI_ROOT"
KRRI_SSH='ssh -o ConnectTimeout=10 -o ServerAliveInterval=15 -o ServerAliveCountMax=3'
echo "복사 대상: $KRRI_HOST:/home/jetson/krri/"
echo '전송 중에는 측정·주행·설치/빌드를 시작하지 마세요.'
ssh -o ConnectTimeout=10 "$KRRI_HOST" bash -s -- "$KRRI_DRY" <<'REMOTE'
set -euo pipefail
command -v rsync >/dev/null
if pgrep -af '[p]ython[^ ]*.*(-m[[:space:]]+(docking|src)([[:space:].]|$)|[/ ](run|calibrate|preflight)[.]py)|[b]ash .*scripts/install_(jetson|realsense)[.]sh|[c]olcon build'; then
  echo '측정·주행·보정 또는 설치/빌드가 실행 중입니다. 종료 후 다시 전송하세요.' >&2
  exit 2
fi
if [[ "$1" == 0 ]]; then mkdir -p /home/jetson/krri; fi
REMOTE
KRRI_FLAGS=(-rlptz --relative --itemize-changes --delay-updates
  --exclude='.git/' --exclude='.claude/' --exclude='.venv/' --exclude='venv/'
  --exclude='__pycache__/' --exclude='.pytest_cache/' --exclude='*.pyc'
  --exclude='.DS_Store' --exclude='.env' --exclude='.env.*'
  --exclude='work/' --exclude='work_dirs/' --exclude='Agent/'
  --exclude='/apriltag_v5/config/learned/' --exclude='/docking_system/calibration/')
if ((KRRI_DRY)); then KRRI_FLAGS+=(--dry-run); fi
rsync "${KRRI_FLAGS[@]}" -e "$KRRI_SSH" \
  --exclude='/docking_system/configs/common.json' \
  --exclude='/docking_systemv2/configs/common.json' \
  "${KRRI_SOURCES[@]}" "$KRRI_HOST:/home/jetson/krri/"
# First copy only: don't overwrite field dimensions or measured mount calibration.
rsync "${KRRI_FLAGS[@]}" --ignore-existing -e "$KRRI_SSH" \
  docking_system/configs/common.json \
  docking_systemv2/configs/common.json "$KRRI_HOST:/home/jetson/krri/"
if ((KRRI_DRY)); then
  echo '미리보기 완료. 파일은 전송·삭제하지 않았습니다.'
else
  echo '전체 코드 복사 완료. VS Code에서 /home/jetson/krri 폴더를 열면 모든 버전이 보입니다.'
fi
echo 'work·work_dirs·과거 백업·가상환경은 복사하지 않습니다. 라이브러리 설치는 별도입니다.'
