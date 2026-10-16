"""Render the reviewed design as editable SVG and matching PNG; no robot I/O.

Requires Pillow and a Korean font. Override DOCKING_DIAGRAM_FONT on another OS.
"""
from pathlib import Path
import html
import math
import os
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent / 'images'
FONT = os.environ.get('DOCKING_DIAGRAM_FONT', '/System/Library/Fonts/AppleSDGothicNeo.ttc')
INK, MUTED, LINE = '#142C40', '#506779', '#6A8294'
BLUE, TEAL, GOLD, RED, GRAY = '#E9F2FF', '#E6F5F0', '#FFF3DC', '#FDECEF', '#F0F3F7'


class Canvas:
    def __init__(self, width, height, title, subtitle):
        self.width, self.height = width, height
        self.im = Image.new('RGB', (width, height), '#FAFCFF')
        self.draw = ImageDraw.Draw(self.im)
        self.svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
                    f'<rect width="{width}" height="{height}" fill="#FAFCFF"/>']
        self.text(65, 48, title, 48, True)
        self.text(65, 112, subtitle, 25, color=MUTED)

    def font(self, size, bold=False):
        return ImageFont.truetype(FONT, size, index=6 if bold and FONT.endswith('.ttc') else 0)

    def text(self, x, y, text, size=28, bold=False, color=INK):
        font = self.font(size, bold)
        self.draw.text((x, y), text, font=font, fill=color, anchor='lt')
        weight = '700' if bold else '400'
        self.svg.append(f'<text x="{x}" y="{y}" dominant-baseline="text-before-edge" '
                        f'font-family="Apple SD Gothic Neo, Noto Sans CJK KR, sans-serif" '
                        f'font-size="{size}" font-weight="{weight}" fill="{color}">{html.escape(text)}</text>')

    def box(self, x, y, w, h, title, lines, color=BLUE, size=28):
        self.draw.rounded_rectangle((x, y, x+w, y+h), radius=20, fill=color, outline='#CCD8E3', width=2)
        self.svg.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="20" fill="{color}" stroke="#CCD8E3" stroke-width="2"/>')
        self.text(x+24, y+22, title, 31, True)
        yy = y+72
        for line in lines:
            assert self.draw.textlength(line, font=self.font(size)) <= w-48, (title, line)
            assert yy+size <= y+h-14, (title, 'height')
            self.text(x+24, yy, line, size)
            yy += 39

    def arrow(self, points, color=LINE, dashed=False):
        self.draw.line(points, fill=color, width=4, joint='curve')
        x0,y0 = points[-2]; x,y = points[-1]
        angle=math.atan2(y-y0,x-x0)
        head=[(x,y), (x-17*math.cos(angle-.45), y-17*math.sin(angle-.45)),
              (x-17*math.cos(angle+.45), y-17*math.sin(angle+.45))]
        self.draw.polygon(head, fill=color)
        path=' '.join(f'{a},{b}' for a,b in points)
        dash=' stroke-dasharray="10 7"' if dashed else ''
        self.svg.append(f'<polyline points="{path}" fill="none" stroke="{color}" stroke-width="4"{dash}/>')
        self.svg.append(f'<polygon points="{" ".join(f"{a},{b}" for a,b in head)}" fill="{color}"/>')

    def save(self, name):
        OUT.mkdir(parents=True, exist_ok=True)
        self.im.save(OUT/f'{name}.png')
        (OUT/f'{name}.svg').write_text('\n'.join(self.svg+['</svg>'])+'\n')


def correction():
    c=Canvas(1800, 1920, 'v6 · 현재 위치와 오차를 어떻게 보정하나',
             '설계안 · OpenVINS 중심 / 태그로 도킹 기준 보정 / 지도·보조 추적은 조건부 사용')
    c.box(65,180,795,178,'IR 좌·우 + IMU',[
        '스테레오 IR 640×480 / 자이로·가속도',
        '캡처 시각 · 좌표계 · 보정값을 먼저 일치시킴'], BLUE)
    c.box(940,180,795,178,'컬러 1280×720 + AprilTag',[
        '태그 크기·ID·장착 위치는 공통 설정에서 읽음',
        '탑재부 입구 중심에 대한 측정값을 만듦'], GOLD)
    c.arrow([(462,358),(462,410)])
    c.arrow([(1338,358),(1338,410)])
    c.box(65,410,795,220,'① OpenVINS · 연속적인 이동 추정',[
        'IMU로 예측 → 영상 특징점으로 보정',
        '위치·자세·속도·IMU bias와 공분산 추정',
        '관성으로 더 간 거리와 회전도 관측에 반영'], BLUE)
    c.box(940,410,795,220,'② 태그 관측 품질 검사',[
        '작은 태그 · 흐림 · PnP 모호성 · 튐 검사',
        '거리·자세에 따른 오차와 연속 프레임 상관 반영',
        '검출됐다는 이유만으로 위치값을 채택하지 않음'], GOLD)
    c.arrow([(462,630),(462,690)])
    c.arrow([(1338,630),(1338,690)])
    c.box(65,690,795,183,'③ 시각별 오도메트리 이력',[
        '이동량 계산은 연속 odom 좌표에서 수행',
        '지도 원점 변경을 실제 차량 이동으로 세지 않음'], BLUE)
    c.box(940,690,795,183,'④ 촬영 시점에서 비교',[
        '늦게 온 태그와 같은 시각의 VIO를 비교',
        '도킹 기준 변환을 보정한 뒤 현재 위치에 적용'], GOLD)
    c.arrow([(462,873),(462,920),(900,920),(900,970)])
    c.arrow([(1338,873),(1338,920),(900,920)])
    c.box(330,970,1140,225,'⑤ 도킹 기준 추정 · 태그 + 연속 VIO',[
        '도킹 좌표에서 위치·각도와 불확실성을 함께 유지',
        '같은 IMU·영상·지도에서 나온 정보를 독립 관측으로 중복 합치지 않음',
        '태그가 없으면 VIO로 이어가되, 드리프트가 사라졌다고 보지 않음'], TEAL)
    c.arrow([(900,1195),(900,1560)])
    c.box(65,1235,795,222,'지도 보조 · RTAB-Map',[
        'OpenVINS odom + IR 키프레임 → 지도·재방문',
        '입구 위치는 태그와 연결해야 알 수 있음',
        '지도 보정은 검증 후 반영 · 즉시 이동량에 합산 금지'], GRAY,27)
    c.box(940,1235,795,222,'선택적 보조 추적 · ORB-SLAM3',[
        '유효한 지도 또는 미리 실행된 추적기가 필요',
        '좌표 정렬·지도 세대·추적 상태 검증 후 전환',
        'OpenVINS 실패 시 무조건 성공하는 대체품은 아님'], GRAY,27)
    c.arrow([(462,1457),(462,1510),(900,1510),(900,1560)])
    c.arrow([(1338,1457),(1338,1510),(900,1510)])
    c.box(330,1560,1140,183,'⑥ 품질 판정 → 실제 포크 끝 위치 → 반복 재계획',[
        '연속 odom으로 잔여 이동 확인 / 도킹 좌표로 목표 오차 확인',
        '추적 불량: 정지·회복 → 검증 후 재계획 / 오차 0으로 리셋하지 않음'], TEAL)
    c.text(65,1800,'핵심: 정지 명령 ≠ 실제 정지 · 작은 공분산 ≠ 실제 오차 보장 · 화면 표시보다 내부 추정이 기준',27,True)
    c.text(65,1850,'설계 문서용 도식 · 실행 코드·실차 성능 검증은 아직 진행하지 않음',24,color=MUTED)
    c.save('error_correction')


def overall():
    c=Canvas(1800, 2180, 'v6 · 자동 도킹 전체 흐름',
             '기존 반복 재계획과 CAN 명령을 유지하고, 실제 위치 추정을 OpenVINS 중심으로 교체하는 설계')
    mainx, mainw, center=90,1010,595
    c.box(mainx,180,mainw,180,'1  현장 입력 + 주행 전 검사',[
        'common: 태그·카메라 치수, 차폭·입구 폭, 좌측 태그 거리',
        '센서 프로필·시간·장착 보정·CAN 프레임과 연결 검사'], BLUE)
    c.arrow([(center,360),(center,402)])
    c.box(mainx,402,mainw,180,'2  정지 초기화 → 준비 완료 후 Enter',[
        'IMU bias·중력·정지 상태·영상 품질 확인 / 초기 VIO 상태 확인',
        '60초 측정 목표, 최대 180초 · 실패하면 출발하지 않음'], BLUE)
    c.arrow([(center,582),(center,624)])
    c.box(mainx,624,mainw,180,'3  태그 탐색 → 입구 기준 확보',[
        '안 보이면 왼쪽으로 60°씩 나누어 최대 한 바퀴 탐색',
        '작거나 불안정한 태그는 “위치 확보”로 인정하지 않음'], GOLD)
    c.arrow([(center,804),(center,846)])
    c.box(mainx,846,mainw,180,'4  현재 위치에서 최대 6단계 후보 계획',[
        '입구 중심 법선에 접근하며 좌우·각도 오차를 함께 줄임',
        '거리·회전·예상 소요시간 비교 → 첫 동작 하나만 실행'], TEAL)
    c.arrow([(center,1026),(center,1068)])
    c.box(mainx,1068,mainw,180,'5  이동 → 정지 명령 → 잔여 이동·되튐 관측',[
        '전진 최대 0.8m / 후진 최대 0.3m / 일반 회전 최대 15°',
        '태그·VIO·IMU로 실제 이동을 추적 · 정지 관측 최대 20초'], TEAL)
    c.arrow([(center,1248),(center,1290)])
    c.box(mainx,1290,mainw,180,'6  실제 도착 위치 확인 · 목표까지 반복',[
        '목표 미도달 → 현재 위치로 6단계 다시 계획',
        '마지막 정렬 전까지 각도도 조정 / 마지막 단계는 좌우 중심'], TEAL)
    c.arrow([(1100,1380),(1150,1380),(1150,936),(1100,936)])
    c.text(1165,1070,'반복',26,True,color='#187968')
    c.arrow([(center,1470),(center,1512)])
    c.box(mainx,1512,mainw,180,'7  필요한 경우 비가시 구간으로 전환',[
        '목표에 먼저 도달하면 바로 정지 / 잘림 예상 전 정렬 확인',
        'VIO 관측 유지 · 좌측 진입의 예상 도착 좌우 오차 ±3cm 확인'], GOLD)
    c.arrow([(center,1692),(center,1734)])
    c.box(mainx,1734,mainw,180,'8  모드별 목표 도달 → 정지·최종 오차 기록',[
        '중앙: 포크 끝이 입구면 0.5m 앞에서 정지',
        '좌측: 포크 끝이 입구면을 지나 1m 지점에서 정지'], BLUE)
    c.box(1250,180,470,260,'상시 · 관측 품질 검사',[
        '지연·누락·추적 상태 검사',
        '태그·VIO 불일치 감시',
        '지도·원점 변경 감지',
        'IMU 누적 오차 감시'], GRAY,26)
    c.box(1250,510,470,260,'추적 불량 → 정지·회복',[
        '정지 관측과 회복은 병행',
        '같은 시간축으로 최대 60초',
        '복구 확인 후 현재 위치로',
        '다시 계획 · 실패하면 종료'], RED,26)
    c.arrow([(1485,440),(1485,510)])
    c.arrow([(1250,640),(1190,640),(1190,890),(1100,890)],color='#B86772')
    c.box(1250,870,470,310,'주행 시간 제한',[
        '출발 전 준비 시간 제외',
        '전체 주행 최대 600초',
        '6단계는 매번 다시 계산',
        '전체 이동거리가 4.8m라는',
        '뜻이 아님'], GRAY,25)
    c.box(1250,1210,470,260,'CAN 전송은 별도 감시',[
        '합의한 주행 바이트 유지',
        '포크·리프트 명령 추가 없음',
        '계산·지도·로그가 늦어도',
        '정지 명령 경로는 분리'], RED,26)
    c.box(1250,1560,470,260,'화면 · 지도 · 기록',[
        'IR 미리보기와 특징점',
        '컬러는 태그 검출에 사용',
        '지도·추적은 역할별 실행',
        '기록·화면은 제어와 분리'], GRAY,26)
    c.box(90,1970,1630,130,'판단 기준',[
        '명령 거리나 학습 속도를 현재 위치로 쓰지 않음 · 태그가 없어도 주변 영상과 IMU로 이동을 관측'], TEAL,28)
    c.text(90,2130,'설계 문서용 도식 · 센서·현장 치수 검증 및 실차 시험 전에는 ±3cm 달성을 보장하지 않음',25,color=MUTED)
    c.save('overall_flow')


if __name__ == '__main__':
    correction()
    overall()
    print(OUT)
