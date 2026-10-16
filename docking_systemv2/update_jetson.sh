#!/usr/bin/env bash
set -euo pipefail
DOCK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCK_HOST=jetson-wired
DOCK_DRY=0
DOCK_COMMON=0
while (($#)); do
  case "$1" in
    --host) DOCK_HOST="${2:?--host 값 필요}"; shift 2 ;;
    --dry-run) DOCK_DRY=1; shift ;;
    --with-common) DOCK_COMMON=1; shift ;;
    -h|--help) echo 'update_jetson.sh [--host jetson-wired] [--dry-run] [--with-common]'; exit 0 ;;
    *) echo "알 수 없는 옵션: $1" >&2; exit 2 ;;
  esac
done
[[ "$DOCK_HOST" =~ ^[a-zA-Z0-9_.@-]+$ && "$DOCK_HOST" != -* ]] || { echo '올바른 SSH 호스트 필요' >&2; exit 2; }
DOCK_TARGET=/home/jetson/krri/docking_systemv2
DOCK_RSYNC=(-az --itemize-changes)
if ((DOCK_DRY)); then
  DOCK_RSYNC+=(--dry-run)
else
  # Replacing code under a running drive process is forbidden.
  ssh -o ConnectTimeout=10 "$DOCK_HOST" 'if pgrep -af "[p]ython[^ ]* .*[-]m docking( |$)|[p]ython[^ ]* .*[/]docking_systemv2/preflight.py"; then
    echo "측정/주행 프로세스를 종료한 후 업데이트하세요." >&2; exit 2
  fi
  mkdir -p /home/jetson/krri/docking_systemv2/configs'
fi
rsync "${DOCK_RSYNC[@]}" --delete \
  --exclude='.git/' --exclude='__pycache__/' --exclude='*.pyc' \
  --exclude='.venv/' --exclude='work/' --exclude='configs/common.json' \
  "$DOCK_ROOT/" "$DOCK_HOST:$DOCK_TARGET/"
if ((DOCK_COMMON)); then
  rsync "${DOCK_RSYNC[@]}" "$DOCK_ROOT/configs/common.json" "$DOCK_HOST:$DOCK_TARGET/configs/"
else
  rsync "${DOCK_RSYNC[@]}" --ignore-existing "$DOCK_ROOT/configs/common.json" "$DOCK_HOST:$DOCK_TARGET/configs/"
fi
if ((DOCK_DRY)); then
  echo '미리보기만 완료: 파일은 전송/삭제하지 않았습니다.'
else
  echo '업데이트 완료. 기존 Jetson common.json(현장 값·장착 보정)은 --with-common을 지정한 경우에만 덮어씁니다.'
fi
echo '측정 로그 /home/jetson/krri/work/ 는 업데이트 대상 밖입니다.'
