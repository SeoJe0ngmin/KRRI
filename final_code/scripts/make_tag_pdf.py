#!/usr/bin/env python3
"""Printable AprilTag 36h11 PDFs (v4, 2026-11-01). Port of the legacy apriltag_v2 make_tag_pdf.py (cv2.aruco pattern + matplotlib page).

  python scripts/make_tag_pdf.py                    # ids 2 and 3 at 200 mm on B4 -> tags/tag36h11_id{2,3}_200mm_B4.pdf (the inner tags)
  python scripts/make_tag_pdf.py --id 1 --size 300 --paper B3
  python scripts/make_tag_pdf.py --check            # also decode the rendered pattern with pupil_apriltags (id + hamming 0)

size = the black border's outer edge (= site entrance_tag_size_m / inner_tag_size_m). Print at 100 % and measure the 100 mm bar and the
tag itself; put the measured tag size in configs/common.json. The white quiet zone around the tag should be at least one cell
(size/8): on a plate the plate is the quiet zone.
"""

import argparse
import os
import sys
import numpy as np
import cv2

CELLS = 8  # tag36h11: 6x6 data cells + the black border = 8x8
MM_PER_INCH = 25.4
PAPERS = {
    "A4": (210, 297),
    "A4L": (297, 210),
    "A3": (297, 420),
    "A3L": (420, 297),
    "B4": (257, 364),
    "B4L": (364, 257),
    "B3": (364, 515),
    "B3L": (515, 364),
    "B4ISO": (250, 353),
    "B3ISO": (353, 500),
}


def tag_cells(tag_id):
    """tag36h11 as an 8x8 boolean grid, True = black."""
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    img = cv2.aruco.generateImageMarker(dic, tag_id, CELLS * 10)
    return np.array([[img[r * 10 + 5, c * 10 + 5] < 128 for c in range(CELLS)] for r in range(CELLS)])


def fits(size_mm, paper):
    pw, ph = PAPERS[paper]
    if size_mm > min(pw, ph):
        return False, "%s: short side %.0f mm < tag %.0f mm" % (paper, min(pw, ph), size_mm)
    margin = min((pw - size_mm) / 2, (ph - size_mm) / 2)
    quiet = size_mm / CELLS
    return True, (
        ("margin %.0f mm < quiet zone %.0f mm: mount on a white plate" % (margin, quiet)) if margin < quiet else ""
    )


def build(tag_id, size_mm, paper, out):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    pw, ph = PAPERS[paper]
    quiet = size_mm / CELLS
    margin = min((pw - size_mm) / 2, (ph - size_mm) / 2)
    fig = plt.figure(figsize=(pw / MM_PER_INCH, ph / MM_PER_INCH))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, pw)
    ax.set_ylim(0, ph)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.add_patch(Rectangle((0, 0), pw, ph, fc="white", ec="none", zorder=0))
    x0 = (pw - size_mm) / 2
    y0 = ph - size_mm - max(margin, 8)
    cell = size_mm / CELLS
    for r, row in enumerate(tag_cells(tag_id)):
        for c, black in enumerate(row):
            if black:
                ax.add_patch(
                    Rectangle(
                        (x0 + c * cell, y0 + (CELLS - 1 - r) * cell),
                        cell,
                        cell,
                        fc="black",
                        ec="none",
                        lw=0,
                        zorder=2,
                        snap=False,
                    )
                )
    for dx in (0, size_mm):
        ax.plot([x0 + dx], [y0], marker="+", ms=9, mew=1.0, color="0.45", zorder=3)
    ax.annotate(
        "",
        xy=(x0, y0 - 6),
        xytext=(x0 + size_mm, y0 - 6),
        arrowprops=dict(arrowstyle="<->", lw=0.8, color="0.35"),
        zorder=3,
    )
    ax.text(
        pw / 2,
        y0 - 10,
        "%.0f mm (black border to black border)" % size_mm,
        ha="center",
        va="top",
        fontsize=8,
        color="0.25",
    )
    ry = 22
    rx = (pw - 100) / 2
    ax.plot([rx, rx + 100], [ry, ry], lw=1.0, color="0.2", zorder=3)
    for i in range(11):
        ax.plot([rx + i * 10] * 2, [ry, ry + (4 if i % 5 == 0 else 2.2)], lw=0.8, color="0.2", zorder=3)
        if i % 5 == 0:
            ax.text(rx + i * 10, ry - 2, "%d" % (i * 10), ha="center", va="top", fontsize=6, color="0.3")
    ax.text(
        pw / 2,
        ry + 7,
        "VERIFY: this bar must measure exactly 100 mm",
        ha="center",
        va="bottom",
        fontsize=7.5,
        color="0.2",
    )
    warn = (
        ("   [!] margin %.0f mm < quiet zone %.0f mm - mount on a white plate" % (margin, quiet))
        if margin < quiet
        else ""
    )
    ax.text(
        pw / 2,
        12,
        "tag36h11  id=%d   size=%.0f mm   %s   PRINT AT 100%% / ACTUAL SIZE%s" % (tag_id, size_mm, paper, warn),
        ha="center",
        va="top",
        fontsize=7,
        color="0.3",
    )
    fig.savefig(out, format="pdf")
    plt.close(fig)
    return dict(paper=paper, size_mm=size_mm, margin=margin, quiet=quiet)


def check(tag_id):
    """Decode the rendered pattern with the detector the vehicle uses: the printed id must come back with hamming 0."""
    try:
        from pupil_apriltags import Detector
    except ImportError:
        return "pupil_apriltags not installed: pattern not checked"
    cells = tag_cells(tag_id)
    n = 40
    img = np.full(((CELLS + 4) * n, (CELLS + 4) * n), 255, np.uint8)
    for r in range(CELLS):
        for c in range(CELLS):
            if cells[r, c]:
                img[(r + 2) * n : (r + 3) * n, (c + 2) * n : (c + 3) * n] = 0
    found = Detector(families="tag36h11", nthreads=1).detect(img)
    ok = [d for d in found if d.tag_id == tag_id and d.hamming == 0]
    return (
        "decoded id %d hamming 0" % tag_id
        if ok
        else "NOT decoded as id %d (%s)" % (tag_id, [(d.tag_id, d.hamming) for d in found])
    )


def main():
    ap = argparse.ArgumentParser(description="printable AprilTag 36h11 PDF")
    ap.add_argument("--id", type=int, action="append", help="tag id (repeatable); default 2 and 3 = the inner tags")
    ap.add_argument("--size", type=float, default=200.0, help="edge of the black border [mm]; default 200 (inner tags)")
    ap.add_argument("--paper", default="B4", choices=sorted(PAPERS))
    ap.add_argument(
        "--out-dir", default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tags")
    )
    ap.add_argument("--check", action="store_true", help="decode the pattern with pupil_apriltags")
    a = ap.parse_args()
    ids = a.id or [2, 3]
    ok, note = fits(a.size, a.paper)
    if not ok:
        raise SystemExit(note)
    os.makedirs(a.out_dir, exist_ok=True)
    for tid in ids:
        out = os.path.join(a.out_dir, "tag36h11_id%d_%.0fmm_%s.pdf" % (tid, a.size, a.paper))
        r = build(tid, a.size, a.paper, out)
        print(
            "%s  (margin %.0f mm, quiet zone %.0f mm%s)%s"
            % (out, r["margin"], r["quiet"], "; " + note if note else "", ("  " + check(tid)) if a.check else "")
        )
    print("print at 100 %, measure the 100 mm bar and the tag; the measured tag size goes into configs/common.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
