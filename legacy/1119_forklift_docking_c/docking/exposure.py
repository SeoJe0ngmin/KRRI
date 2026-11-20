"""컬러 카메라 자동노출 영역을 태그 자리에 맞춘다.

밝은 마당과 어두운 적재함이 한 화면에 있으면 화면 전체로 맞춘 노출로는 적재함 안의 태그가 어둡게 찍힌다. 안쪽 태그가 보이면
그 자리로, 입구 태그만 보이면 입구 태그와 정해 둔 영역(적재함 입구)을 함께, 아무 태그도 안 보이면 정해 둔 영역(없으면 화면
전체)으로 자동노출 영역을 옮긴다. 컬러 영상은 태그 검출에만 쓰므로 VIO 와 depth(IR)에는 영향이 없다.

다만 영역을 카메라에 넘기는 호출 자체가 IMU · 영상 전달을 0.1초쯤 끊으므로(2026-11-19 Jetson 실측), 차가 서 있을 때만
넘긴다(`camera.color_ae_roi_standing_only`, 기본 true; `ColorExposure.update(..., allow)`).

그늘 기준(2026-11-19 현장 로그, `camera.color_ae_roi_shade`): 안쪽 태그 자리에 영역을 꼭 맞추면 흰 종이가 영역을 채워 카메라가
오히려 노출을 내렸다(영역을 태그로 옮긴 직후 화면 평균 146→131, 120→83; 오른쪽 태그 점수는 기준판과 같은 30, 영역 밖의 왼쪽 태그는
16~20). 반대로 카메라를 켠 직후 노출이 아직 높던 1초 동안에는 두 태그가 5.4 m 에서 점수 29~45 로, 8 m 에서도 24~30 으로 읽혔다.
곧 안쪽 태그는 자동노출이 주는 것보다 훨씬 밝게 찍어야 읽힌다. 그래서 안쪽 태그가 한 장이라도 채택되면 영역을 태그 종이가 아니라
두 태그 사이의 그늘진 틈(흰 종이를 뺀 자리, 한 장만 보이면 다른 한 장이 있을 쪽)에 둔다: 카메라가 그늘을 기준으로 노출을 올린다.
- 입구 태그가 최근 ENTRANCE_CLEAR_S 안에 보였으면 쓰지 않는다(입구 태그는 햇빛을 받아 노출을 올리면 날아간다:
  시작 직후 화면 평균 219 에서 점수 33, 228 이상에서는 검출 안 됨).
- 안쪽 태그가 SHADE_RANGE_M 보다 멀면 쓰지 않는다(그 거리에서는 입구 태그가 아직 화면에 있을 수 있다).
- 차가 그대로 서 있는데, 꾸준히 보이던 안쪽 태그가 그늘 기준으로 바꾼 뒤 SHADE_LOST_S 동안 한 장도 안 잡히면 너무 밝아진
  것으로 보고 그 실행에서는 그늘 기준을 끄고 태그 자리 영역으로 되돌린다(기록에 `shade off`).
"""

from collections import deque

MARGIN = 2.0  # 태그 상자를 이 배수로 넓힌다: 흰 테두리와 둘레 배경이 함께 들어가야 노출이 한쪽으로 쏠리지 않는다(가정)
MIN_PX = 48  # 영역 한 변의 최소 화소: 너무 작으면 장면마다 노출이 흔들린다(가정)
PERIOD_S = 0.5  # 영역을 다시 정하는 최소 간격(가정)
HOLD_S = 2.0  # 태그가 안 보여도 이 시간 동안은 마지막 영역을 그대로 둔다(가정)
CHANGE = 0.25  # 영역이 짧은 변의 이 비율 넘게 달라져야 다시 정한다(가정)
FAILURES_MAX = 20  # 카메라가 영역 설정을 이만큼 잇달아 거절하면 그만둔다(IR 영역 설정의 재시도 횟수와 같게)
SHADE_HEIGHT = 2.0  # 그늘 상자의 세로 = 태그 높이의 이 배수(태그를 붙인 가로살의 위아래 빈 자리까지, 가정)
SHADE_RANGE_M = 4.5  # 안쪽 태그가 이보다 가까울 때만 그늘 기준(2026-11-19: 보통 노출로 안쪽 태그가 처음 잡힌 거리 4.2 m 와
#                      입구 태그가 마지막으로 보인 자리(안쪽 태그까지 약 5.3 m) 사이, 가정). camera.color_ae_roi_shade_range_m
SHADE_LOST_S = 1.5  # 그늘 기준으로 바꾼 뒤 서 있는 채로 이 시간 동안 안쪽 태그가 한 장도 안 잡히면 그늘 기준을 끈다(가정)
SHADE_SETTLE_S = 3.0  # 바꾼 뒤 이 시간이 지나서도 안쪽 태그가 잡혔으면 노출이 자리 잡고도 보인 것: 그 뒤의 놓침은 노출 탓으로 보지 않는다
#                       (2026-11-19: 영역을 바꾼 뒤 화면 밝기가 자리 잡는 데 1.5~2초, 가정)
SHADE_STEADY = (3, 5)  # 최근 태그 결과 5개 중 3개 이상에서 안쪽 태그가 잡혀야(꾸준히 보여야) 그늘 기준으로 바꾼다(가정):
#                        꾸준히 보이던 태그여야 바꾼 뒤 사라졌을 때 노출 탓이라고 말할 수 있다
ENTRANCE_CLEAR_S = 5.0  # 입구 태그가 이 시간 동안 안 보인 뒤에만 그늘 기준(시작 직후에는 시작 시각부터 센다, 가정)


def corner_box(corners):
    """네 귀퉁이 화소 좌표를 덮는 상자 (x0, y0, x1, y1)."""
    xs = [float(p[0]) for p in corners]
    ys = [float(p[1]) for p in corners]
    return min(xs), min(ys), max(xs), max(ys)


def tag_boxes(tag):
    """태그 결과에서 검출된 태그의 상자: (입구 태그 상자 또는 None, 안쪽 태그 상자 목록)."""
    tag = tag or {}
    entrance = corner_box(tag['corners']) if tag.get('detected') and tag.get('corners') else None
    inner = []
    for part in ((tag.get('inner') or {}).get('tags') or {}).values():
        if isinstance(part, dict) and part.get('detected') and part.get('corners'):
            inner.append(corner_box(part['corners']))
    return entrance, inner


def union(boxes):
    """상자들을 모두 덮는 상자."""
    return (
        min(b[0] for b in boxes),
        min(b[1] for b in boxes),
        max(b[2] for b in boxes),
        max(b[3] for b in boxes),
    )


def widen(box, width, height, margin=MARGIN, min_px=MIN_PX):
    """상자를 가운데 기준으로 margin 배 넓히고(한 변 최소 min_px) 화면 안으로 자른 정수 상자."""
    cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
    hw = max((box[2] - box[0]) * margin, min_px) / 2
    hh = max((box[3] - box[1]) * margin, min_px) / 2
    return (
        int(max(0, round(cx - hw))),
        int(max(0, round(cy - hh))),
        int(min(width - 1, round(cx + hw))),
        int(min(height - 1, round(cy + hh))),
    )


def ae_box(tag, width, height, region=None):
    """이번 태그 결과로 정한 자동노출 영역. 안쪽 태그가 보이면 그 자리만, 입구 태그만 보이면 입구 태그와 region 을 함께 덮는다.
    검출된 태그가 없으면 None."""
    entrance, inner = tag_boxes(tag)
    if inner:
        return widen(union(inner), width, height)
    if entrance is None:
        return None
    box = widen(entrance, width, height)
    return box if region is None else tuple(int(v) for v in union([box, region]))


def shade_layout(inner):
    """안쪽 태그 배치(cfg['calibration']['inner']) → 그늘 상자에 쓰는 비율. 배치가 없거나 두 종이 사이에 틈이 없으면 None.
    ids = [왼쪽, 오른쪽](화면에서도 왼쪽 태그가 왼쪽), edge = 태그 변에서 종이 끝까지, gap = 두 종이 사이 틈; 둘 다 태그 변 길이의 배수."""
    if not inner:
        return None
    size = float(inner['size_m'])
    plate = 2.0 * float(inner['plate_half_width_m'])
    gap = (float(inner['spacing_m']) - plate) / size
    if gap <= 0:
        return None
    return dict(ids=[str(i) for i in inner['ids']], edge=(plate - size) / 2 / size, gap=gap)


def inner_seen(tag):
    """채택된 안쪽 태그: {ID 글자: (상자, 카메라에서 태그까지 거리[m])}."""
    out = {}
    for key, part in (((tag or {}).get('inner') or {}).get('tags') or {}).items():
        if isinstance(part, dict) and part.get('ok') and part.get('corners') and part.get('camera_translation_m'):
            out[str(key)] = (
                corner_box(part['corners']),
                sum(float(v) ** 2 for v in part['camera_translation_m']) ** 0.5,
            )
    return out


def shade_box(tag, width, height, layout, max_range=SHADE_RANGE_M):
    """두 안쪽 태그 사이의 그늘진 틈(흰 종이를 뺀 자리)의 정수 상자. 한 장만 채택됐으면 그 태그의 화면 폭으로 다른 한 장 쪽 틈을
    잡는다(두 태그는 한 평면이라 비스듬히 봐도 폭과 틈이 같은 비율로 줄어든다). 채택된 태그가 없거나 max_range 보다 멀거나 틈이
    화면 밖이면 None."""
    if not layout:
        return None
    seen = inner_seen(tag)
    if not seen or min(r for _, r in seen.values()) > max_range:
        return None
    left, right = layout['ids']
    a = seen.get(left, (None,))[0]
    b = seen.get(right, (None,))[0]
    if a is not None and b is not None:
        x0 = a[2] + layout['edge'] * (a[2] - a[0])
        x1 = b[0] - layout['edge'] * (b[2] - b[0])
        y0, y1 = min(a[1], b[1]), max(a[3], b[3])
    elif a is not None:
        x0 = a[2] + layout['edge'] * (a[2] - a[0])
        x1 = x0 + layout['gap'] * (a[2] - a[0])
        y0, y1 = a[1], a[3]
    elif b is not None:
        x1 = b[0] - layout['edge'] * (b[2] - b[0])
        x0 = x1 - layout['gap'] * (b[2] - b[0])
        y0, y1 = b[1], b[3]
    else:
        return None
    cy = (y0 + y1) / 2
    hh = max((y1 - y0) * SHADE_HEIGHT, MIN_PX) / 2
    box = (
        int(max(0, round(x0))),
        int(max(0, round(cy - hh))),
        int(min(width - 1, round(x1))),
        int(min(height - 1, round(cy + hh))),
    )
    if box[2] - box[0] < MIN_PX or box[3] - box[1] < MIN_PX:
        return None
    return box


def differs(a, b, change=CHANGE):
    """두 영역이 다시 정할 만큼 다른가."""
    if a is None or b is None:
        return a != b
    short = max(1, min(a[2] - a[0], a[3] - a[1]))
    return max(abs(x - y) for x, y in zip(a, b)) > change * short


class ColorExposure:
    """새 태그 결과가 올 때마다 자동노출 영역을 고르고, 달라졌을 때만 카메라에 넘긴다. 실패해도 주행을 막지 않는다."""

    def __init__(self, width, height, region, set_roi, layout=None, shade_range_m=SHADE_RANGE_M):
        self.width = int(width)
        self.height = int(height)
        self.region = None if region is None else tuple(int(v) for v in region)
        self.full = (0, 0, self.width - 1, self.height - 1)
        self.set_roi = set_roi
        self.current = None
        self.seen_at = None
        self.set_at = None
        self.failures = 0
        self.state = 'pending'
        # 그늘 기준(layout = shade_layout(...) 또는 None = 끔)
        self.layout = layout
        self.shade_range_m = float(shade_range_m)
        self.shade_off = None  # 이 실행에서 그늘 기준을 끈 이유(끄지 않았으면 None)
        self.kind = None  # 지금 카메라에 걸려 있는 영역의 종류
        self.started_at = None
        self.entrance_at = None
        self.inner_at = None
        self.moved_at = None
        self.shade_at = None
        self.shade_check = False  # 지금 건 그늘 상자가 해로운지 아직 판정 중인가
        self.tight = None  # 그늘 기준으로 바꿀 때의 태그 자리 영역(되돌릴 곳)
        self.recent = deque(maxlen=SHADE_STEADY[1])

    def shade(self, now, tag):
        """지금 그늘 기준을 써도 되면 그늘 상자, 아니면 None."""
        if self.layout is None or self.shade_off:
            return None
        clear_since = self.entrance_at if self.entrance_at is not None else self.started_at
        if clear_since is None or now - clear_since < ENTRANCE_CLEAR_S:
            return None
        if self.kind != 'shade' and sum(self.recent) < SHADE_STEADY[0]:
            return None  # 들어갈 때만 꾸준함을 본다: 이미 그늘 기준이면 한두 번 놓쳐도 태그 자리로 되돌아가 왔다 갔다 하지 않는다
        return shade_box(tag, self.width, self.height, self.layout, self.shade_range_m)

    def blinded(self, now, allow):
        """그늘 기준으로 바꾼 뒤 차는 그대로 서 있는데, 꾸준히 보이던 안쪽 태그가 SHADE_LOST_S 동안 한 장도 안 잡혔는가."""
        if self.kind != 'shade' or not self.shade_check or not allow:
            return False
        if self.moved_at is not None and self.moved_at >= self.shade_at:
            return False  # 그 뒤로 차가 움직였다: 태그가 안 보이는 까닭이 노출인지 알 수 없다
        last = self.shade_at if self.inner_at is None else max(self.shade_at, self.inner_at)
        if last - self.shade_at > SHADE_SETTLE_S:
            self.shade_check = False  # 노출이 자리 잡은 뒤에도 보였다: 이번에 건 그늘 상자는 더 판정하지 않는다
            return False
        return now - last > SHADE_LOST_S

    def target(self, now, tag):
        box = self.shade(now, tag)
        if box is not None:
            self.seen_at = now
            return box, 'shade'
        box = ae_box(tag, self.width, self.height, self.region)
        if box is not None:
            self.seen_at = now
            return box, 'tags'
        if self.seen_at is not None and now - self.seen_at <= HOLD_S and self.current is not None:
            return self.current, 'held'
        return (self.region, 'region') if self.region is not None else (self.full, 'full')

    def update(self, now, tag, allow=True):
        """새 태그 결과 하나를 반영한다. 지금 상태를 글로 돌려준다(기록용).

        allow=False(차가 움직이는 중)이면 카메라에 영역을 넘기지 않는다. 영역을 넘기는 호출 한 번에 카메라가 0.1초쯤 멈춰
        IMU 가 80 ms 빠지고 영상이 137 ms 늦게 온다(2026-11-19 Jetson 실측, 호출이 없으면 0). 태그를 본 시각은 계속
        기억하므로, 차가 다음에 설 때 그때의 태그 자리로 한 번에 옮긴다."""
        if self.failures >= FAILURES_MAX:
            return self.state
        entrance, inner = tag_boxes(tag)
        if self.started_at is None:
            self.started_at = now
        if entrance is not None:
            self.entrance_at = now
        if inner:
            self.inner_at = now
        if not allow:
            self.moved_at = now
        if self.blinded(now, allow):
            # 너무 밝아져 태그가 날아갔다: 이 실행에서는 그늘 기준을 끄고, 바꾸기 전의 태그 자리 영역으로 되돌린다
            self.shade_off = 'tags lost'
            self.recent.clear()
            return self.apply(now, self.tight or self.region or self.full, 'tags')
        self.recent.append(bool(inner))
        box, why = self.target(now, tag)
        if not allow:
            return self.state
        if why == 'held' or not differs(self.current, box):
            return self.state
        if self.set_at is not None and now - self.set_at < PERIOD_S:
            return self.state
        if why == 'shade':
            self.shade_check = True
            self.tight = ae_box(tag, self.width, self.height, self.region)
        return self.apply(now, box, why)

    def apply(self, now, box, why):
        """영역 하나를 카메라에 넘긴다. 실패해도 예외를 내지 않고 상태 글로 돌려준다."""
        self.set_at = now
        try:
            self.set_roi(box)
        except Exception as exc:
            self.failures += 1
            self.state = ('gave up: ' if self.failures >= FAILURES_MAX else 'retry %d: ' % self.failures) + str(exc)
            return self.state
        self.failures = 0
        self.current = box
        self.kind = why
        if why == 'shade':
            self.shade_at = now
        self.state = '%s %d,%d..%d,%d' % ((why,) + tuple(box)) + (' | shade off: ' + self.shade_off if self.shade_off else '')
        return self.state
