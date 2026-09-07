"""운행 기록(--record-events)을 읽고 세 가지를 답한다.

    ① 잘 됐나        결과(outcome)와 마지막 측정값을 허용치와 비교
    ② 뭐가 있었나     단계별 판단·중단 이력, --timeline 으로 전체 시간순
    ③ 파라미터는      회전 오버슈트 -> ROT_LEAD_DEG,
                      회전 시간 적합 -> ROT_T0 / ROT_DEG_PER_SEC,
                      직진 명령거리 vs 실제거리 -> FWD_SCALE

    python tools/analyze_run.py                    최신 실행
    python tools/analyze_run.py 20260904_073730    그 실행
    python tools/analyze_run.py --timeline         사건 전체를 시간순으로
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from config.control import HEAD_TOL_DEG, LAT_TOL_M                       # noqa: E402
from src.models.control.fwd_time_model import fwd_sec_from_offset_piecewise  # noqa: E402
from src.utils.event_log import read_events                              # noqa: E402

LOG_ROOT = os.path.join(ROOT, "work_dirs", "docking_log")


def pick_run(name):
    if name:
        return name if os.path.isdir(name) else os.path.join(LOG_ROOT, name)
    runs = sorted(d for d in os.listdir(LOG_ROOT)
                  if os.path.isdir(os.path.join(LOG_ROOT, d)))
    if not runs:
        raise SystemExit("기록이 없다: %s" % LOG_ROOT)
    return os.path.join(LOG_ROOT, runs[-1])


def sec_to_distance(sec):
    """명령 시간 -> 명령 거리 [m]. 전진 시간모델의 역함수 (이분법)."""
    lo, hi = 0.0, 10.0
    for _ in range(50):
        mid = (lo + hi) / 2
        if fwd_sec_from_offset_piecewise(mid) < sec:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def linear_fit(xs, ys):
    """최소제곱 직선 y = a + b*x. (a, b). x 가 다 같으면 None."""
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx < 1e-9:
        return None
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return my - b * mx, b


def show_summary(events):
    results = [e for e in events if e["kind"] == "result"]
    measures = [e for e in events if e["kind"] == "measure"]
    dur = events[-1]["ts"] - events[0]["ts"] if len(events) > 1 else 0.0
    print("── 결과 " + "─" * 58)
    if results:
        r = results[-1]
        label = {"done": "도착", "manual": "수동전환 (태그를 끝내 못 찾음)",
                 "max_steps": "단계 초과 (수렴 실패)", "user_stop": "사용자 중단",
                 "incomplete": "비정상 종료 (예외)"}.get(r["outcome"], r["outcome"])
        print("   %s   %d단계, %.0f초" % (label, r.get("steps", 0), dur))
    else:
        print("   result 기록 없음 (도중에 죽었나) — %.0f초 분량" % dur)
    if measures:
        m = measures[-1]
        lat_ok = abs(m["lateral"]) <= LAT_TOL_M
        head_ok = abs(m["heading_deg"]) <= HEAD_TOL_DEG
        print("   마지막 측정: lat %+.0fmm %s   head %+.2f도 %s   fwd %.2fm"
              % (m["lateral"] * 1000, "OK" if lat_ok else "**허용초과**",
                 m["heading_deg"], "OK" if head_ok else "**허용초과**", m["forward"]))
    aborts = [e for e in events if e["kind"] == "abort"]
    for a in aborts:
        print("   중단 이력: [%d] %s" % (a.get("step", -1), a["why"]))

    # 흔들림은 이제 주행을 멈추지 않는다 — 대신 얼마나 흔들렸는지 여기서 센다.
    # 자주 뜨면 config 의 STABLE_* 문턱이 현장과 안 맞는다는 뜻이다.
    decisions = [e for e in events if e["kind"] == "decision"]
    shaky = [d for d in decisions if d.get("stable") is False]
    if decisions:
        print("   흔들림: %d/%d 사이클 (%.0f%%)"
              % (len(shaky), len(decisions), 100.0 * len(shaky) / len(decisions)))
        counts = {}
        for d in shaky:
            for r in (d.get("reasons") or []):
                key = r.split()[0] + " " + (r.split()[1] if len(r.split()) > 1 else "")
                counts[key.strip()] = counts.get(key.strip(), 0) + 1
        for k, n in sorted(counts.items(), key=lambda kv: -kv[1])[:4]:
            print("      %-24s %d회" % (k, n))


def show_rotation(events):
    rots = [e for e in events if e["kind"] == "rotation"]
    print("\n── 회전 (%d회) " % len(rots) + "─" * 50)
    if not rots:
        return
    good = [r for r in rots if r.get("ok") and r.get("turned") is not None]
    bad = [r for r in rots if not r.get("ok")]
    for reason in sorted({b.get("reason", "?") for b in bad}):
        n = sum(1 for b in bad if b.get("reason") == reason)
        print("   실패 %d회: %s" % (n, reason))
    if not good:
        return
    overs = [r["overshoot"] for r in good]
    mean_over = sum(overs) / len(overs)
    sd_over = (sum((o - mean_over) ** 2 for o in overs) / max(len(overs) - 1, 1)) ** 0.5
    print("   오버슈트: 평균 %+.2f도  표준편차 %.2f도  최대 %+.2f도  (n=%d)"
          % (mean_over, sd_over, max(overs, key=abs), len(good)))
    # 오버슈트는 지금 LEAD 를 쓴 결과라, 권장 LEAD = 지금 LEAD + 평균 오버슈트
    from config.control import ROT_LEAD_DEG
    print("   -> config/control.py 의 ROT_LEAD_DEG = %.1f 권장 (지금 %.1f + 평균 오버슈트 %+.2f)"
          % (max(0.0, ROT_LEAD_DEG + mean_over), ROT_LEAD_DEG, mean_over))
    fit = linear_fit([abs(r["target"]) for r in good],
                     [r["elapsed_sec"] for r in good]) if len(good) >= 3 else None
    if fit:
        t0, slope = fit
        if slope > 1e-6:
            print("   시간 적합: elapsed = %.2fs + 각도/%.1f도/s   (성공 %d회 기준)"
                  % (t0, 1.0 / slope, len(good)))
            print("   -> rot_control.py 의 ROT_T0 = %.2f, ROT_DEG_PER_SEC = %.1f 권장"
                  % (max(0.0, t0), 1.0 / slope))
    else:
        print("   시간 적합: 회전이 3회 미만이거나 각도가 다 같아서 못 함")


def show_pivot(events):
    """회전 전후 lateral 변화로 CAM_PIVOT_M(회전 중심에서 카메라까지)을 추정한다.

    모델: lateral 변화 = R · (sin h1 − sin h0).  회전만 있고 사이에 직진이 없는 쌍만 쓴다.
    """
    import math
    rots = [e for e in events if e["kind"] == "rotation" and e.get("ok")]
    measures = [e for e in events if e["kind"] == "measure"]
    drives = [e for e in events if e["kind"] == "drive"]
    print("\n── 회전이 카메라를 옮기는 양 (CAM_PIVOT_M) " + "─" * 30)
    ests = []
    for r in rots:
        before = [m for m in measures if m["ts"] < r["ts"]]
        after = [m for m in measures if m["ts"] > r["ts"]]
        if not before or not after:
            continue
        b, a = before[-1], after[0]
        if any(b["ts"] < d["ts"] < a["ts"] for d in drives):
            continue
        if any(b["ts"] < x["ts"] < a["ts"] for x in rots if x is not r):
            continue
        ds = math.sin(math.radians(a["heading_deg"])) - math.sin(math.radians(b["heading_deg"]))
        if abs(ds) < math.sin(math.radians(2.0)):
            continue                       # 2도 미만 회전은 잡음에 묻힌다
        dl = a["lateral"] - b["lateral"]
        ests.append(dl / ds)
        print("   회전 %+6.1f도 (카메라 %+.1f -> %+.1f도)  lateral %+.0f -> %+.0fmm  ->  R = %+.2fm"
              % (r["turned"], b["heading_deg"], a["heading_deg"],
                 b["lateral"] * 1000, a["lateral"] * 1000, dl / ds))
    if len(ests) >= 2:
        ests.sort()
        med = ests[len(ests) // 2]
        print("   -> 중앙값 R = %+.2fm (n=%d, 범위 %+.2f ~ %+.2f).  config/control.py 의 CAM_PIVOT_M = %+.1f 권장"
              % (med, len(ests), ests[0], ests[-1], med))
    else:
        print("   회전 전후 측정이 짝지어진 회전이 2회 미만이라 못 함")


def show_drive(events):
    drives = [e for e in events if e["kind"] == "drive"]
    measures = [e for e in events if e["kind"] == "measure"]
    rots = [e for e in events if e["kind"] == "rotation"]
    print("\n── 직진 (%d회) " % len(drives) + "─" * 50)
    ratios = []
    for d in drives:
        before = [m for m in measures if m["ts"] < d["ts"]]
        after = [m for m in measures if m["ts"] > d["ts"]]
        if not before or not after:
            continue                       # Set2 안의 이동은 태그가 안 보여 짝이 없다
        b, a = before[-1], after[0]
        # 사이에 회전이 끼면(=Set2) 전후 위치 차가 직진만의 결과가 아니다
        if any(b["ts"] < r["ts"] < a["ts"] for r in rots):
            continue
        commanded = sec_to_distance(d["sec"])
        # v2 조준 직진은 대각선이라 forward 차만 보면 cos(조준각)만큼 짧게 보인다 — 평면 이동량으로
        actual = ((b["forward"] - a["forward"]) ** 2 + (b["lateral"] - a["lateral"]) ** 2) ** 0.5
        if commanded > 0.05:
            ratios.append(actual / commanded)
            print("   %-8s 명령 %.2fm (%.1fs)  ->  실제 %.2fm   비율 %.3f"
                  % (d["movement"], commanded, d["sec"], actual, actual / commanded))
    if ratios:
        mean_ratio = sum(ratios) / len(ratios)
        print("   -> 실제/명령 평균 %.3f.  1 에서 멀면 fwd_time_model.py 의 "
              "FWD_SCALE = %.3f 로 보정 (n=%d)" % (mean_ratio, 1.0 / mean_ratio, len(ratios)))
    else:
        print("   전후 측정이 짝지어진 전진이 없다 (Set2 안의 이동은 태그가 안 보여 제외)")


def show_timeline(events):
    t0 = events[0]["ts"]
    print("\n── 시간순 전체 " + "─" * 51)
    for e in events:
        t = e["ts"] - t0
        k = e["kind"]
        if k == "measure":
            line = "lat %+7.1fmm  fwd %5.2fm  head %+6.2f도" % (
                e["lateral"] * 1000, e["forward"], e["heading_deg"])
        elif k == "decision":
            line = "[%d] %s — %s" % (e.get("step", -1), e["action"], e["why"])
        elif k == "rotation":
            line = "목표 %+.1f도 -> 실제 %s (%.1fs) %s" % (
                e["target"],
                "%+.1f도" % e["turned"] if e.get("turned") is not None else "?",
                e.get("elapsed_sec", 0.0), "" if e.get("ok") else "!! " + e.get("reason", ""))
        elif k == "drive":
            line = "%s %.1fs" % (e["movement"], e["sec"])
        elif k == "abort":
            line = "!! " + e["why"]
        elif k == "result":
            line = "끝: %s (%d단계)" % (e["outcome"], e.get("steps", 0))
        else:
            line = str({x: v for x, v in e.items() if x not in ("kind", "ts")})
        print("   %7.1fs  %-9s %s" % (t, k, line))


def main():
    ap = argparse.ArgumentParser(description="운행 기록 분석")
    ap.add_argument("run", nargs="?", help="실행 폴더 이름. 안 주면 최신")
    ap.add_argument("--timeline", action="store_true", help="사건 전체를 시간순으로")
    a = ap.parse_args()

    run_dir = pick_run(a.run)
    events = read_events(run_dir)
    if not events:
        raise SystemExit("비어 있다: %s" % run_dir)
    print("실행: %s   (%d개 사건)" % (os.path.basename(run_dir), len(events)))
    show_summary(events)
    show_rotation(events)
    show_pivot(events)
    show_drive(events)
    if a.timeline:
        show_timeline(events)


if __name__ == "__main__":
    main()
