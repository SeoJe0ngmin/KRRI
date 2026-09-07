import argparse
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
import cv2
import numpy as np
from config.detection import MM_PER_INCH, TAG_CELLS as CELLS
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
PAPERS = {'A4': (210, 297), 'A4L': (297, 210), 'A3': (297, 420), 'A3L': (420, 297), 'B4': (257, 364), 'B4L': (364, 257), 'B3': (364, 515), 'B3L': (515, 364), 'B4ISO': (250, 353), 'B3ISO': (353, 500)}

def fits(size_mm, paper):
    pw, ph = PAPERS[paper]
    if size_mm > min(pw, ph):
        return (False, '%s 는 짧은 변이 %.0fmm 라 %.0fmm 태그가 안 들어간다' % (paper, min(pw, ph), size_mm))
    margin = min((pw - size_mm) / 2, (ph - size_mm) / 2)
    quiet = size_mm / CELLS
    if margin < quiet:
        return (True, '여백 %.0fmm < 권장 %.0fmm — 흰 판에 붙일 것' % (margin, quiet))
    return (True, '')

def tag_cells(tag_id):
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    img = cv2.aruco.generateImageMarker(dic, tag_id, CELLS * 10)
    return np.array([[img[r * 10 + 5, c * 10 + 5] < 128 for c in range(CELLS)] for r in range(CELLS)])

def build(tag_id, size_mm, paper, out):
    pw, ph = PAPERS[paper]
    quiet = size_mm / CELLS
    margin = min((pw - size_mm) / 2, (ph - size_mm) / 2)
    fig = plt.figure(figsize=(pw / MM_PER_INCH, ph / MM_PER_INCH))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, pw)
    ax.set_ylim(0, ph)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.add_patch(Rectangle((0, 0), pw, ph, fc='white', ec='none', zorder=0))
    x0 = (pw - size_mm) / 2
    y0 = ph - size_mm - max(margin, 8)
    cell = size_mm / CELLS
    for r, row in enumerate(tag_cells(tag_id)):
        for c, black in enumerate(row):
            if black:
                ax.add_patch(Rectangle((x0 + c * cell, y0 + (CELLS - 1 - r) * cell), cell, cell, fc='black', ec='none', lw=0, zorder=2, snap=False))
    for dx, dy, ha, va in ((0, 0, 'right', 'top'), (size_mm, 0, 'left', 'top')):
        ax.plot([x0 + dx], [y0 + dy], marker='+', ms=9, mew=1.0, color='0.45', zorder=3)
    ax.annotate('', xy=(x0, y0 - 6), xytext=(x0 + size_mm, y0 - 6), arrowprops=dict(arrowstyle='<->', lw=0.8, color='0.35'), zorder=3)
    ax.text(pw / 2, y0 - 10, f'{size_mm:.0f} mm  (black border to black border)', ha='center', va='top', fontsize=8, color='0.25')
    ry = 22
    rx = (pw - 100) / 2
    ax.plot([rx, rx + 100], [ry, ry], lw=1.0, color='0.2', zorder=3)
    for i in range(11):
        h = 4 if i % 5 == 0 else 2.2
        ax.plot([rx + i * 10] * 2, [ry, ry + h], lw=0.8, color='0.2', zorder=3)
        if i % 5 == 0:
            ax.text(rx + i * 10, ry - 2, f'{i * 10}', ha='center', va='top', fontsize=6, color='0.3')
    ax.text(pw / 2, ry + 7, 'VERIFY: this bar must measure exactly 100 mm', ha='center', va='bottom', fontsize=7.5, color='0.2')
    warn = ''
    if margin < quiet:
        warn = f'   [!] margin {margin:.0f}mm < quiet zone {quiet:.0f}mm - mount on white board'
    ax.text(pw / 2, 12, f'tag36h11  id={tag_id}   size={size_mm:.0f}mm   {paper}   PRINT AT 100% / ACTUAL SIZE{warn}', ha='center', va='top', fontsize=7, color='0.3')
    fig.savefig(out, format='pdf')
    plt.close(fig)
    return dict(paper=paper, page=(pw, ph), size_mm=size_mm, margin=margin, quiet=quiet, ok=margin >= quiet)

def main():
    ap = argparse.ArgumentParser(description='인쇄용 AprilTag PDF')
    ap.add_argument('--id', type=int, default=1, help='태그 ID')
    ap.add_argument('--size', type=float, default=200, help='한 변 [mm], 검은 테두리 바깥까지')
    ap.add_argument('--paper', default='A4', choices=sorted(PAPERS))
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    ok, note = fits(a.size, a.paper)
    if not ok:
        best = [p for p in sorted(PAPERS) if fits(a.size, p)[0]]
        raise SystemExit('  %s\n  들어가는 용지: %s' % (note, ', '.join(best) or '없음'))
    import os
    ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    out = a.out or os.path.join(ROOT, f'work_dirs/tags/tag36h11_id{a.id}_{a.size:.0f}mm_{a.paper}.pdf')
    if os.path.dirname(out):
        os.makedirs(os.path.dirname(out), exist_ok=True)
    r = build(a.id, a.size, a.paper, out)
    print(f'  생성: {out}')
    print(f"  용지 {r['page'][0]:.0f}x{r['page'][1]:.0f}mm   태그 {r['size_mm']:.0f}mm   여백 {r['margin']:.0f}mm (권장 {r['quiet']:.0f}mm)")
    if not r['ok']:
        need = a.size * (1 + 2.0 / CELLS)
        roomy = [p for p in sorted(PAPERS, key=lambda x: min(PAPERS[x])) if min(PAPERS[p]) >= need]
        print(f'  [!] 여백이 부족하다. 흰 판에 붙이면 판이 여백 역할을 하니 실사용엔 문제없다.')
        print(f"      여백까지 종이로 넣으려면 {(roomy[0] if roomy else '더 큰 종이')}(짧은 변 {need:.0f}mm 이상)를 쓰거나, 이 종이에선 태그를 {min(*r['page']) / (1 + 2.0 / CELLS):.0f}mm 로 줄여라.")
    print(f'  인쇄 후 눈금자 100mm 를 자로 재고, 태그도 재서 실측값을 --tag-size 로 넘길 것')
if __name__ == '__main__':
    main()
