"""인쇄용 AprilTag PDF 생성.

프린터가 제멋대로 축소하는 것("페이지에 맞춤")이 거리 오차의 최대 원인이다.
여기서는 **실물 치수를 고정한 벡터 PDF**를 만들고, 인쇄물을 자로 검증할 수 있게
눈금자까지 같이 찍는다.

    python src/etc/make_tag_pdf.py --id 1 --size 200 --paper A4

tag_size 규약: **검은 테두리 바깥까지**. tag36h11 은 6x6 데이터 + 검은테두리 1칸 = 8칸이고,
OpenCV 가 만드는 이미지의 바깥 경계가 정확히 그 지점이다. 우리 src/models/tag_pose.py 가 쓰는 기준과 같다.

인쇄할 때 반드시:
    - 배율 100% / 실제 크기 (Actual size). "페이지에 맞춤" 절대 금지
    - 무광 용지 (광택지는 조명 반사로 검출이 깨진다)
    - 인쇄 후 눈금자를 자로 재서 맞는지 확인하고, 실측값을 --tag-size 로 넘길 것
"""
import argparse

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

MM_PER_INCH = 25.4
CELLS = 8                      # tag36h11: 6x6 데이터 + 검은 테두리 1칸
PAPERS = {                     # (가로, 세로) mm
    "A4": (210, 297), "A4L": (297, 210),
    "A3": (297, 420), "A3L": (420, 297),
}


def tag_cells(tag_id):
    """tag36h11 을 8x8 불리언 격자로. True = 검은 칸.

    래스터 이미지를 배치하면 PDF 변환행렬에서 100µm 수준 반올림이 생긴다.
    칸마다 벡터 사각형을 그리면 치수가 정확하고 인쇄도 선명하다.
    """
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    img = cv2.aruco.generateImageMarker(dic, tag_id, CELLS * 10)
    return np.array([[img[r * 10 + 5, c * 10 + 5] < 128
                      for c in range(CELLS)] for r in range(CELLS)])


def build(tag_id, size_mm, paper, out):
    pw, ph = PAPERS[paper]
    quiet = size_mm / CELLS                       # 여백 1칸 권장치
    margin = min((pw - size_mm) / 2, (ph - size_mm) / 2)

    fig = plt.figure(figsize=(pw / MM_PER_INCH, ph / MM_PER_INCH))
    ax = fig.add_axes([0, 0, 1, 1])               # 용지 전체를 mm 좌표로
    ax.set_xlim(0, pw); ax.set_ylim(0, ph)
    ax.set_aspect("equal"); ax.axis("off")
    ax.add_patch(Rectangle((0, 0), pw, ph, fc="white", ec="none", zorder=0))

    # 태그: 가로 중앙, 위쪽에 배치 (아래는 설명·눈금자 자리)
    x0 = (pw - size_mm) / 2
    y0 = ph - size_mm - max(margin, 8)
    cell = size_mm / CELLS
    for r, row in enumerate(tag_cells(tag_id)):
        for c, black in enumerate(row):
            if black:
                ax.add_patch(Rectangle(
                    (x0 + c * cell, y0 + (CELLS - 1 - r) * cell), cell, cell,
                    fc="black", ec="none", lw=0, zorder=2, snap=False))

    # 모서리 표시 — 자로 잴 때 어디서 어디까지인지 알려준다
    for dx, dy, ha, va in ((0, 0, "right", "top"), (size_mm, 0, "left", "top")):
        ax.plot([x0 + dx], [y0 + dy], marker="+", ms=9, mew=1.0, color="0.45", zorder=3)
    ax.annotate("", xy=(x0, y0 - 6), xytext=(x0 + size_mm, y0 - 6),
                arrowprops=dict(arrowstyle="<->", lw=0.8, color="0.35"), zorder=3)
    ax.text(pw / 2, y0 - 10, f"{size_mm:.0f} mm  (black border to black border)",
            ha="center", va="top", fontsize=8, color="0.25")

    # 검증용 눈금자 100mm — 인쇄물에서 이게 100mm 면 배율이 정확한 것이다
    ry = 22
    rx = (pw - 100) / 2
    ax.plot([rx, rx + 100], [ry, ry], lw=1.0, color="0.2", zorder=3)
    for i in range(11):
        h = 4 if i % 5 == 0 else 2.2
        ax.plot([rx + i * 10] * 2, [ry, ry + h], lw=0.8, color="0.2", zorder=3)
        if i % 5 == 0:
            ax.text(rx + i * 10, ry - 2, f"{i*10}", ha="center", va="top",
                    fontsize=6, color="0.3")
    ax.text(pw / 2, ry + 7, "VERIFY: this bar must measure exactly 100 mm",
            ha="center", va="bottom", fontsize=7.5, color="0.2")

    warn = ""
    if margin < quiet:
        warn = (f"   [!] margin {margin:.0f}mm < quiet zone {quiet:.0f}mm"
                f" - mount on white board")
    ax.text(pw / 2, 12,
            f"tag36h11  id={tag_id}   size={size_mm:.0f}mm   {paper}"
            f"   PRINT AT 100% / ACTUAL SIZE{warn}",
            ha="center", va="top", fontsize=7, color="0.3")

    fig.savefig(out, format="pdf")
    plt.close(fig)
    return dict(paper=paper, page=(pw, ph), size_mm=size_mm,
                margin=margin, quiet=quiet, ok=margin >= quiet)


def main():
    ap = argparse.ArgumentParser(description="인쇄용 AprilTag PDF")
    ap.add_argument("--id", type=int, default=1, help="태그 ID")
    ap.add_argument("--size", type=float, default=200, help="한 변 [mm], 검은 테두리 바깥까지")
    ap.add_argument("--paper", default="A4", choices=sorted(PAPERS))
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    # 기본 출력은 저장소 루트 기준으로 고정한다. 예전엔 cwd 상대라서, 어디서
    # 돌리느냐에 따라 PDF 가 엉뚱한 데 생기고 work_dirs/tags 가 여기저기 만들어졌다.
    # (이 파일은 <repo>/src/etc/ 에 있으므로 루트는 parents[2].)
    import os
    ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    out = a.out or os.path.join(
        ROOT, f"work_dirs/tags/tag36h11_id{a.id}_{a.size:.0f}mm_{a.paper}.pdf")
    if os.path.dirname(out):        # --out 을 파일명만으로 준 경우 dirname 이 빈 문자열이다
        os.makedirs(os.path.dirname(out), exist_ok=True)
    r = build(a.id, a.size, a.paper, out)

    print(f"  생성: {out}")
    print(f"  용지 {r['page'][0]:.0f}x{r['page'][1]:.0f}mm   태그 {r['size_mm']:.0f}mm"
          f"   여백 {r['margin']:.0f}mm (권장 {r['quiet']:.0f}mm)")
    if not r["ok"]:
        print(f"  [!] 여백이 부족하다. 흰 판에 붙이면 판이 여백 역할을 하니 실사용엔 문제없다.")
        print(f"      여백까지 종이로 넣으려면 A3 를 쓰거나 태그를 "
              f"{min(*r['page'])/10*8:.0f}mm 로 줄여라.")
    print(f"  인쇄 후 눈금자 100mm 를 자로 재고, 태그도 재서 실측값을 --tag-size 로 넘길 것")


if __name__ == "__main__":
    main()
