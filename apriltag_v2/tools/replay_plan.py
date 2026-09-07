"""v1 운행 기록을 v2 판단기에 다시 넣어 본다 — "그때 그 측정값이면 v2 는 뭘 했을까".

    python tools/replay_plan.py                      가장 최근 기록 (v1 폴더 포함해서 찾는다)
    python tools/replay_plan.py 20260907_160811      그 실행
    python tools/replay_plan.py --all                오늘 기록 전부
    python tools/replay_plan.py <폴더 경로>

측정값(measure.jsonl)마다 v1 이 실제로 내린 결정(decision.jsonl)과 v2 plan_step 의
결정을 나란히 찍는다. **정적 재생**이다 — 차가 v2 대로 움직였으면 다음 측정값이
달라졌을 테니, "v2 라면 이 상황에서 이렇게 판단한다" 까지만 보여 준다.
기하(부호·조준각)와 잡음 문턱이 실제 숫자에서 어떻게 작동하는지 확인하는 용도.
"""
import argparse
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from config import control as C                                          # noqa: E402
from src.models.control.control_from_pose import is_near, plan_step      # noqa: E402

LOG_ROOTS = [os.path.join(ROOT, "work_dirs", "docking_log")]
for sib in ("apriltag_v1", "apriltag"):
    p = os.path.join(os.path.dirname(ROOT), sib, "work_dirs", "docking_log")
    if os.path.isdir(p):
        LOG_ROOTS.append(p)


def list_runs():
    runs = []
    for root in LOG_ROOTS:
        if not os.path.isdir(root):
            continue
        for d in os.listdir(root):
            full = os.path.join(root, d)
            if os.path.isdir(full) and os.path.exists(os.path.join(full, "measure.jsonl")):
                runs.append(full)
    return sorted(runs, key=os.path.basename)


def read_jsonl(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def fmt_amount(action, amount):
    if action == "approach":
        return "%.2fm @%+.1f도" % (amount[0], amount[1])
    if action == "sidestep":
        return "%+.1f도, %.0fmm %s" % (amount[0], amount[1] * 1000, amount[2])
    if action in ("rotate_ccw", "rotate_cw"):
        return "%.2f도" % amount
    if action in ("forward", "final", "backup", "recover_backup"):
        return "%.2fm" % amount
    return ""


def replay(run_dir):
    measures = read_jsonl(os.path.join(run_dir, "measure.jsonl"))
    decisions = read_jsonl(os.path.join(run_dir, "decision.jsonl"))
    print("\n=== %s  (%s) ===" % (os.path.basename(run_dir), os.path.dirname(run_dir)))
    print("  ALIGN_M %.1f  MIN_DZ %.1f  APPROACH_MAX %.0f도  NEAR_ABSORB %.1f도  CAM_PIVOT %.2f  NOISE_K %.0f"
          % (C.ALIGN_M, C.APPROACH_MIN_DZ_M, C.APPROACH_MAX_DEG, C.NEAR_ABSORB_DEG,
             C.CAM_PIVOT_M, C.NOISE_K))
    print("  %-4s %8s %7s %7s %6s %6s | %-10s | %-10s %s"
          % ("step", "lat_mm", "fwd_m", "head", "σlat", "σhead", "기록", "지금 v2", "양 / 이유"))
    st = {"half_fov_deg": 35.0, "prev_forward": None, "last_action": None,
          "near": False, "backups": 0, "near_rots": 0}
    for m in measures:
        dec = next((d for d in decisions if abs(d["ts"] - m["ts"]) < 0.5), None)
        st["margin_px"] = (dec or {}).get("margin_px")
        st["near"] = is_near(m["forward"], st)
        action, amount, _sec, why = plan_step(m, st)
        sp = m.get("spread") or {}
        print("  %-4s %+8.0f %7.2f %+7.2f %6.1f %6.2f | %-10s | %-10s %s"
              % ((dec or {}).get("step", "?"), m["lateral"] * 1000, m["forward"],
                 m["heading_deg"], sp.get("lateral", 0) * 1000, sp.get("heading_deg", 0),
                 (dec or {}).get("action", "?"), action, fmt_amount(action, amount)))
        print("  %-4s %s" % ("", why))
        # 정적 재생이라 다음 측정은 기록된 차가 실제로 움직인 뒤의 값이다 — 상태(near,
        # backups, near_rots, hold 판정)는 지금 v2 가 낸 동작 기준으로 흉내만 낸다.
        st["prev_forward"], st["last_action"] = m["forward"], action
        if action == "backup":
            st["backups"] += 1
        if action in ("rotate_ccw", "rotate_cw") and st["near"]:
            st["near_rots"] += 1
        elif action in ("forward", "final", "approach", "backup", "sidestep"):
            st["near_rots"] = 0


def main():
    ap = argparse.ArgumentParser(description="v1 기록 -> v2 판단 재생")
    ap.add_argument("run", nargs="?", help="실행 폴더 이름 또는 경로. 안 주면 최신")
    ap.add_argument("--all", action="store_true", help="찾은 기록 전부")
    a = ap.parse_args()
    runs = list_runs()
    if a.all:
        targets = runs
    elif a.run and os.path.isdir(a.run):
        targets = [a.run]
    elif a.run:
        targets = [r for r in runs if os.path.basename(r) == a.run]
    else:
        targets = runs[-1:]
    if not targets:
        raise SystemExit("기록이 없다. 찾은 곳: %s" % ", ".join(LOG_ROOTS))
    for r in targets:
        replay(r)


if __name__ == "__main__":
    main()
