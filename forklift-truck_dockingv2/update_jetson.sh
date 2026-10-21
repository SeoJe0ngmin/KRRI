#!/usr/bin/env bash
set -euo pipefail
DOCK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCK_HOST=jetson-wired
DOCK_DRY=0
while (($#)); do
  case "$1" in
    --host) DOCK_HOST="${2:?--host 값 필요}"; shift 2 ;;
    --dry-run) DOCK_DRY=1; shift ;;
    -h|--help) echo 'update_jetson.sh [--host jetson-wired] [--dry-run]   # common.json 은 맥 것으로 덮고, configs/mount.json(장착각)·work/ 는 안 건드림'; exit 0 ;;
    *) echo "알 수 없는 옵션: $1" >&2; exit 2 ;;
  esac
done
[[ "$DOCK_HOST" =~ ^[a-zA-Z0-9_.@-]+$ && "$DOCK_HOST" != -* ]] || { echo '올바른 SSH 호스트 필요' >&2; exit 2; }
DOCK_TARGET=/home/jetson/krri/$(basename "$DOCK_ROOT")   # v2 lives beside v1 on the Jetson (2026-10-22)
# One SSH connection shared by every step below (ControlMaster): the password is asked once, not per step.
mkdir -p "$HOME/.ssh/ctl"
DOCK_SSH=(ssh -o ConnectTimeout=10 -o ControlMaster=auto -o ControlPath="$HOME/.ssh/ctl/%r@%h:%p" -o ControlPersist=120)
DOCK_RSYNC=(-az --itemize-changes -e "${DOCK_SSH[*]}")
if ((DOCK_DRY)); then DOCK_RSYNC+=(--dry-run); fi
# Replacing code under a running drive process is forbidden.
"${DOCK_SSH[@]}" "$DOCK_HOST" 'if pgrep -af "[p]ython[^ ]* .*[-]m docking([. ]|$)"; then
  echo "측정/주행 프로세스를 종료한 후 업데이트하세요." >&2; exit 2
fi
mkdir -p '"$DOCK_TARGET"'/configs'
# The dev kit has no RTC battery: after a power cycle the clock reads 1970 until NTP syncs (hotspot only). A wrong
# clock only mislabels work/ folders at run time but refuses builds (check_clock), so set it from the Mac here.
if ((!DOCK_DRY)) && [[ "$("${DOCK_SSH[@]}" "$DOCK_HOST" 'date -u +%Y')" -lt 2024 ]]; then
  echo "Jetson 시계가 틀려 맥 시간으로 맞춥니다 (sudo 비밀번호):"
  "${DOCK_SSH[@]}" -t "$DOCK_HOST" "sudo date -u -s '$(date -u '+%Y-%m-%d %H:%M:%S')'"
fi
# 2026-10-20: common.json has one author (the Mac) and is overwritten on every deploy. The measured mount lives in
# configs/mount.json on the Jetson and is never sent or deleted. Show what the overwrite changes, then migrate a
# pre-split mount value (common.json "mount" field) into mount.json once, before it is overwritten.
if "${DOCK_SSH[@]}" "$DOCK_HOST" "test -f $DOCK_TARGET/configs/common.json"; then
  if ! "${DOCK_SSH[@]}" "$DOCK_HOST" "cat $DOCK_TARGET/configs/common.json" | diff - "$DOCK_ROOT/configs/common.json" >/tmp/dock_common.diff; then
    echo 'Jetson common.json 과 다른 부분 (< Jetson, > 맥; 맥 값으로 덮습니다):'; cat /tmp/dock_common.diff
  fi
  if ((!DOCK_DRY)); then
    "${DOCK_SSH[@]}" "$DOCK_HOST" "/usr/bin/python3 - $DOCK_TARGET/configs" <<'PY'
import json,pathlib,sys
d=pathlib.Path(sys.argv[1]);c=d/'common.json';m=d/'mount.json'
if not m.exists():
    try:v=json.loads(c.read_text())['fields'].get('mount',{}).get('value')
    except Exception:v=None
    if isinstance(v,dict) and v:m.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n');print('장착각을 configs/mount.json 으로 옮겼습니다 (common.json 의 mount 값)')
PY
  fi
fi
rsync "${DOCK_RSYNC[@]}" --delete \
  --exclude='.git/' --exclude='__pycache__/' --exclude='*.pyc' \
  --exclude='.venv/' --exclude='work/' --exclude='configs/mount.json' \
  "$DOCK_ROOT/" "$DOCK_HOST:$DOCK_TARGET/"
# First deploy of a new version folder: reuse the mount (camera pitch/yaw) measured with the previous folder when the camera has not moved.
if ((!DOCK_DRY)); then
  "${DOCK_SSH[@]}" "$DOCK_HOST" "if [ ! -f '$DOCK_TARGET/configs/mount.json' ] && [ -f /home/jetson/krri/forklift-truck_docking/configs/mount.json ]; then cp /home/jetson/krri/forklift-truck_docking/configs/mount.json '$DOCK_TARGET/configs/mount.json' && echo 'mount.json 을 forklift-truck_docking 에서 복사했습니다 (카메라를 옮겼으면 preflight.sh mount 로 다시)'; fi"
fi
if ((DOCK_DRY)); then
  echo '미리보기만 완료: 파일은 전송/삭제하지 않았습니다.'
else
  echo '업데이트 완료. common.json 은 맥 것으로 덮었고, configs/mount.json(장착각)과 work/ 는 그대로입니다.'
fi
echo '첫 배포 뒤 또는 C++ 브리지가 바뀐 뒤에는 Jetson 에서 bash scripts/rebuild_openvins.sh 를 실행합니다.'
