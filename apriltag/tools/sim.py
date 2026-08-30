"""시뮬레이터를 한 곳에서 굴린다 — ablation 용.

손댈 값이 simulate.py 에 CLI 23개로 흩어져 있다. 여기 SCENARIO 하나만
고치면 되고, 한 값씩 바꿔가며 오차를 보는 것도 한 줄이면 된다.

    python tools/sim.py --live                   슬라이더로 실시간 (제일 편하다)
    python tools/sim.py                          SCENARIO 그대로 한 번
    python tools/sim.py distance 1 2 3 5         거리만 바꿔가며
    python tools/sim.py noise 0 2 5 10           센서 노이즈만
    python tools/sim.py heading -30 -15 0 15 30

코드에서:
    from tools.sim import run, ablate
    run(distance=5.0, noise=3.0)                 한 번 재기
    ablate("blur_px", [0, 5, 10, 20, 32])        표로

정답은 우리가 만든 것이라 소수점 열두 자리까지 정확하다. 카메라도 줄자도
필요 없다. 실카메라 검증은 tools/verify.py 가 한다(정답이 줄자라 ±5mm).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.simulate_view import (RESOLUTIONS, as_place, intrinsics_for,  # noqa: E402
                                 measure)


# ── 손댈 값 전부. 여기만 고치면 된다 ────────────────────────────────────────
SCENARIO = dict(
    # 배치 — 전부 바닥 기준으로 잰다 (실제 방을 재는 방식 그대로)
    tag_height = 1.60,      # m   태그 중심 높이
    cam_height = 1.20,      # m   카메라 장착 높이. 태그와 다르면 가까이서 화면 이탈
    distance   = 3.00,      # m   태그면까지 (forward)
    lateral    = -1.03,     # m   태그 축에서 좌우. +가 오른쪽
    heading    = 20.0,      # deg 지게차가 태그 축과 틀어진 각

    # 태그
    tag_size   = 0.200,     # m   한 변, 검은 테두리 바깥까지
    tag_id     = 1,

    # 카메라 — RESOLUTIONS 의 키 중 하나
    res        = "1920x1080",

    # 검출
    method     = "auto",    # auto | tag(AT3 내장) | pnp(solvePnP)
    quad_blur  = None,      # None 이면 노이즈에 맞춰 자동, 0 이면 끔

    # 화질 열화 — 실제로 겪는 것들
    blur_px    = 0.0,       # px  모션블러. 실측 10px 까지 100%, 32px 에서 0%
    noise      = 0.0,       # 그레이 단계. 어두운 곳일수록 크다
    seed       = 0,
)

_PLACE_KEYS = ("tag_height", "cam_height", "distance", "lateral", "heading")


def run(**over):
    """SCENARIO 에서 몇 개만 바꿔 한 번 재고 결과를 돌려준다.

    반환은 simulate_view.measure() 의 것이다. 자주 쓰는 것:
        r["err"]["lateral"]   정답 대비 오차 [m]
        r["est"]["tag_px"]    화면상 태그 크기 [px]
        r["ok"], r["reason"]  못 쟀으면 왜인지
    """
    cfg = dict(SCENARIO)
    bad = set(over) - set(cfg)
    if bad:
        raise SystemExit("모르는 값: %s (가능: %s)" % (", ".join(sorted(bad)),
                                                    ", ".join(sorted(cfg))))
    cfg.update(over)
    if cfg["res"] not in RESOLUTIONS:
        raise SystemExit("모르는 해상도: %s (가능: %s)"
                         % (cfg["res"], ", ".join(RESOLUTIONS)))
    place = as_place({k: cfg[k] for k in _PLACE_KEYS})
    return measure(place, intrinsics_for(cfg["res"]),
                   tag_size=cfg["tag_size"], tag_id=cfg["tag_id"],
                   method=cfg["method"], blur_px=cfg["blur_px"],
                   noise=cfg["noise"], seed=cfg["seed"],
                   quad_blur=cfg["quad_blur"])


ROWS = (("lateral", "mm", 1000.0), ("forward", "mm", 1000.0),
        ("z_optical", "mm", 1000.0), ("heading", "deg", 1.0), ("tilt", "deg", 1.0))


def ablate(knob, values, log=print, **over):
    """한 값만 바꿔가며 오차를 표로. 나머지는 SCENARIO 그대로."""
    if knob not in SCENARIO:
        raise SystemExit("모르는 값: %s (가능: %s)" % (knob, ", ".join(sorted(SCENARIO))))
    head = "%-12s" % knob + "".join("%12s" % ("%s[%s]" % (k, u)) for k, u, _ in ROWS)
    log(head + "%9s" % "tag_px")
    log("-" * len(head + "%9s" % "tag_px"))
    out = []
    for v in values:
        r = run(**{knob: v}, **over)
        out.append((v, r))
        if not r["ok"]:
            log("%-12s  %s" % (v, r["reason"]))
            continue
        line = "%-12s" % v
        for k, _u, f in ROWS:
            line += "%12.2f" % (r["err"][k] * f)
        log(line + "%9.1f" % r["est"]["tag_px"])
    return out


# ── 슬라이더로 실시간 ───────────────────────────────────────────────────────

#: (이름, 최소, 최대, 단위)  — 슬라이더로 뺄 값들
_SLIDERS = (("distance",   0.3,  8.0, "m"),
            ("lateral",   -3.0,  3.0, "m"),
            ("heading",  -60.0, 60.0, "deg"),
            ("tag_height", 0.5,  2.5, "m"),
            ("cam_height", 0.5,  2.5, "m"),
            ("tag_size",  0.05,  0.6, "m"),
            ("blur_px",    0.0, 40.0, "px"),
            ("noise",      0.0, 20.0, "lv"))


def _draw3d(ax, cfg, ok):
    """배치를 3D 로 그린다. 좌표는 **태그 기준**이다.

        x = 태그면에서 앞으로 (forward)
        y = 좌우 (lateral, +가 오른쪽)
        z = 높이 (바닥 기준)
    """
    import matplotlib.pyplot as plt
    import numpy as np
    d, lat = cfg["distance"], cfg["lateral"]
    th, ch, hd = cfg["tag_height"], cfg["cam_height"], np.radians(cfg["heading"])
    ts = cfg["tag_size"]

    ax.clear()
    # 축 범위는 실제로 쓰는 만큼만 — 넓게 잡으면 그림이 납작해진다
    x0, x1 = -0.3, d * 1.15 + 0.3
    y0, y1 = min(lat, 0.0) - 0.6, max(lat, 0.0) + 0.6
    z1 = max(th, ch) + 0.5

    # 바닥 격자
    for v in np.arange(0, x1, 0.5):
        ax.plot([v, v], [y0, y1], [0, 0], color="#e5e7eb", lw=0.5, zorder=0)
    for v in np.arange(np.ceil(y0 * 2) / 2, y1, 0.5):
        ax.plot([0, x1], [v, v], [0, 0], color="#e5e7eb", lw=0.5, zorder=0)

    # 태그 — x=0 평면에 선 정사각형
    h = ts / 2.0
    ax.plot([0, 0, 0, 0, 0], [-h, h, h, -h, -h],
            [th - h, th - h, th + h, th + h, th - h], color="#c1352b", lw=2.5)
    ax.plot([0], [0], [th], "o", color="#c1352b", ms=5)
    ax.plot([0, 0], [0, 0], [0, th], color="#c1352b", lw=1, ls=":")

    # 태그 정면축 (도킹 목표선)
    ax.plot([0, x1], [0, 0], [th, th], color="#c1352b", lw=1, ls="--", alpha=0.6)

    # 카메라
    cam = np.array([d, lat, ch])
    col = "#2563eb" if ok else "#ef4444"
    ax.plot([cam[0]], [cam[1]], [cam[2]], "o", color=col, ms=7)
    ax.plot([cam[0], cam[0]], [cam[1], cam[1]], [0, ch], color=col, lw=1, ls=":")

    # 보는 방향 + 시야 (좌우 35.2도)
    for a, ln, al in ((0.0, 1.0, 1.0), (35.2, 0.8, 0.35), (-35.2, 0.8, 0.35)):
        th_ = hd + np.radians(a)
        v = np.array([-np.cos(th_), np.sin(th_), 0.0])   # heading=0 이면 태그 쪽(-x)
        e = cam + v * (d * ln)
        ax.plot([cam[0], e[0]], [cam[1], e[1]], [cam[2], e[2]],
                color=col, lw=1.6 if a == 0 else 1.0, alpha=al)

    # lateral / forward 치수선
    ax.plot([d, d], [0, lat], [th, th], color="#6b7280", lw=1.2)
    ax.plot([0, d], [0, 0], [th, th], color="#6b7280", lw=1.2)
    ax.text(d, lat / 2, th + 0.12, "lateral %.2fm" % lat, color="#374151", fontsize=8)
    ax.text(d / 2, 0, th + 0.12, "forward %.2fm" % d, color="#374151", fontsize=8)

    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_zlim(0, z1)
    ax.set_xlabel("forward [m]", fontsize=8, labelpad=-4)
    ax.set_ylabel("lateral [m]", fontsize=8, labelpad=-4)
    ax.set_zlabel("height [m]", fontsize=8, labelpad=-6)
    ax.tick_params(labelsize=6.5, pad=-2)
    for a in (ax.xaxis, ax.yaxis, ax.zaxis):      # 눈금이 겹치지 않게
        a.set_major_locator(plt.MaxNLocator(4))
    ax.view_init(elev=20, azim=-62)
    try:
        # 세로를 실제 비율보다 키운다 — 안 그러면 납작해서 안 보인다
        ax.set_box_aspect((x1 - x0, y1 - y0, z1 * 1.8))
    except Exception:
        pass


def _use_korean_font():
    """한글이 네모로 나오지 않게. DejaVu 에는 한글 글자가 없다."""
    import matplotlib
    from matplotlib import font_manager
    for path in ("/home/jeongmin/.local/share/fonts/malgun.ttf",
                 "/mnt/c/Windows/Fonts/malgun.ttf"):
        try:
            font_manager.fontManager.addfont(path)
            name = font_manager.FontProperties(fname=path).get_name()
            matplotlib.rcParams["font.family"] = name
            matplotlib.rcParams["axes.unicode_minus"] = False
            return name
        except Exception:
            continue
    return None


def live():
    """슬라이더를 움직이며 검출되는지 / 얼마나 틀리는지를 실시간으로 본다.

    한 번 재는 데 약 30ms 라 슬라이더를 끌면 바로 따라온다.
    검출이 안 되면 이유를 화면에 크게 띄운다(시야 밖 / 너무 작음 / 블러 등).
    """
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib.widgets import RadioButtons, Slider
    _use_korean_font()

    state = dict(SCENARIO)

    fig = plt.figure(figsize=(13.5, 8.2))
    fig.canvas.manager.set_window_title("simulate — live")
    ax_img = fig.add_axes([0.02, 0.40, 0.32, 0.56])
    ax_3d = fig.add_axes([0.35, 0.38, 0.34, 0.60], projection="3d")
    ax_txt = fig.add_axes([0.71, 0.40, 0.28, 0.56])
    ax_img.set_xticks([]); ax_img.set_yticks([])
    ax_txt.axis("off")

    sliders = {}
    for i, (name, lo, hi, unit) in enumerate(_SLIDERS):
        ax = fig.add_axes([0.08, 0.33 - i * 0.038, 0.52, 0.022])
        sliders[name] = Slider(ax, "%s [%s]" % (name, unit), lo, hi,
                               valinit=float(state[name]), valfmt="%.2f")

    ax_res = fig.add_axes([0.68, 0.20, 0.13, 0.13])
    ax_res.set_title("res", fontsize=9)
    r_res = RadioButtons(ax_res, list(RESOLUTIONS),
                         active=list(RESOLUTIONS).index(state["res"]))
    ax_met = fig.add_axes([0.84, 0.20, 0.13, 0.13])
    ax_met.set_title("method", fontsize=9)
    r_met = RadioButtons(ax_met, ["auto", "tag", "pnp"])

    def redraw(_=None):
        for k in sliders:
            state[k] = float(sliders[k].val)
        state["res"] = r_res.value_selected
        state["method"] = r_met.value_selected
        r = run(**{k: state[k] for k in state})

        ax_img.clear(); ax_img.set_xticks([]); ax_img.set_yticks([])
        if r["image"] is not None:
            ax_img.imshow(r["image"][:, :, ::-1])
            if r["detection"] is not None:
                c = r["detection"].corners
                ax_img.plot(list(c[:, 0]) + [c[0, 0]], list(c[:, 1]) + [c[0, 1]],
                            "-", color="#22c55e", lw=2)
        ax_img.set_facecolor("#111")

        _draw3d(ax_3d, state, r["ok"])
        ax_txt.clear(); ax_txt.axis("off")
        if not r["ok"]:
            ax_img.text(0.5, 0.5, "검출 실패", color="#ef4444", fontsize=26,
                        ha="center", va="center", transform=ax_img.transAxes)
            ax_txt.text(0, 0.95, "안 되는 이유\n\n  %s" % r["reason"],
                        color="#ef4444", fontsize=12, va="top")
        else:
            d = r["truth"]["docking"]
            lines = ["%-10s%9s%9s%9s" % ("", "정답", "추정", "오차"), "-" * 37]
            for k, u, f in ROWS:
                t = d.get(k, d.get(k + "_deg", r["truth"].get(k)))
                lines.append("%-10s%9.1f%9.1f%9.2f%s"
                             % (k, t * f, r["est"][k] * f, r["err"][k] * f, u))
            q = r["quality"]
            lines += ["", "tag_px    %8.1f  (하한20/안정50)" % r["est"]["tag_px"],
                      "재투영    %8.3f px" % q.get("reproj_rms_px", float("nan")),
                      "margin    %8.1f" % q.get("decision_margin", float("nan")),
                      "각도신뢰  %8s" % r["est"]["rel_tilt"],
                      "품질 ok   %8s" % q.get("ok")]
            if q.get("reasons"):
                lines.append("  걸린 것: " + ", ".join(q["reasons"]))
            ax_txt.text(0, 0.98, "\n".join(lines), fontsize=9.5, va="top")
        fig.canvas.draw_idle()

    for sl in sliders.values():
        sl.on_changed(redraw)
    r_res.on_clicked(redraw)
    r_met.on_clicked(redraw)
    redraw()
    plt.show()


def main():
    args = sys.argv[1:]
    if args and args[0] in ("--live", "-l"):
        live()
        return
    if not args:
        r = run()
        print("SCENARIO:")
        for k, v in SCENARIO.items():
            print("  %-11s %s" % (k, v))
        print()
        if not r["ok"]:
            raise SystemExit("못 쟀다: %s" % r["reason"])
        print("%-12s%12s%12s%12s" % ("", "정답", "추정", "오차"))
        for k, u, f in ROWS:
            d = r["truth"]["docking"]      # 키 이름이 조금 다르다
            t = d.get(k, d.get(k + "_deg", r["truth"].get(k)))
            print("%-12s%12.3f%12.3f%12.2f%s"
                  % (k, t * f, r["est"][k] * f, r["err"][k] * f, u))
        print("\ntag_px %.1f   재투영 %.3fpx   각도신뢰 %s"
              % (r["est"]["tag_px"], r["quality"].get("reproj_rms_px", float("nan")),
                 r["est"]["rel_tilt"]))
        return
    knob, values = args[0], args[1:]
    cast = type(SCENARIO[knob]) if SCENARIO.get(knob) is not None else float
    if cast is str:
        vals = values
    elif cast is int:
        vals = [int(v) for v in values]
    else:
        vals = [float(v) for v in values]
    ablate(knob, vals)


if __name__ == "__main__":
    main()
