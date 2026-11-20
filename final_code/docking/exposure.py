"""컬러 카메라 자동노출 영역 (실험판 D, 2026-11-19 사용자 결정).

컬러 영상은 태그 검출에만 쓴다(VIO 와 depth 는 IR). 카메라의 노출은 화면 전체에 하나뿐이라, 그늘진 안쪽 태그(2·3)에 맞추면 햇빛을
받는 입구 태그(1)가 하얗게 날아간다. 태그를 입구 쪽으로 당겨 단 뒤로는 출발 자리에서부터 2·3 이 잡혀, 실험판 C 의 규칙("안쪽
태그가 보이면 그 자리로")이 입구 태그를 쓰는 동안에 노출을 가져가 버렸다(2026-11-19 오후 현장). 그래서:

1. 입구 태그를 쓰는 동안에는 노출을 건드리지 않는다: 카메라 기본 영역 그대로(기준판과 같다).
2. 입구 태그가 화면에서 잘리는 자리(태그 컷)를 지났거나 안쪽 태그가 정확해지는 거리(inner_bundle_max_range_m) 안에 들어온 뒤에만
   안쪽 태그에 노출을 맞춘다. 둘 중 먼저 오는 쪽부터다: 채택된 안쪽 태그까지의 거리(PnP)가
   max(태그 면 − 태그 컷, inner_bundle_max_range_m) 이하일 때(`inner_range`).
3. 그 안에서는 먼저 태그 자리(`tags`), 안쪽 태그가 꾸준히 잡히면 두 태그 사이의 그늘진 틈(`shade`, 흰 종이를 뺀 자리; 한 장만
   보이면 다른 한 장이 있을 쪽)에 영역을 둔다. 태그 자리에 꼭 맞추면 흰 종이 때문에 카메라가 노출을 내리고(2026-11-19 낮 로그:
   화면 평균 146→131, 120→83), 그늘을 기준으로 해야 어두운 쪽 태그까지 밝아진다.
4. 서 있는 채로, 꾸준히 보이던 안쪽 태그가 그늘 기준으로 바꾼 뒤 SHADE_LOST_S 동안 한 장도 안 잡히면 너무 밝아진 것으로 보고
   그 실행에서는 그늘 기준을 끄고 태그 자리 영역으로 되돌린다(기록에 `shade off`).
5. 안쪽 태그가 HOLD_S 넘게 안 잡히면 카메라 기본 영역으로 돌아간다.

영역을 카메라에 넘기는 호출 자체가 IMU · 영상 전달을 0.1초쯤 끊으므로(2026-11-19 Jetson 실측) 차가 서 있을 때만 넘긴다
(`camera.color_ae_roi_standing_only`, `ColorExposure.update(..., allow)`). 끄려면 `camera.color_ae_roi_tags` 를 false 로: 이 모듈은
카메라를 전혀 건드리지 않고 기본 노출 그대로 달린다.
"""

from collections import deque

MARGIN = 2.0  # 태그 상자를 이 배수로 넓힌다: 흰 테두리와 둘레 배경이 함께 들어가야 노출이 한쪽으로 쏠리지 않는다(가정)
MIN_PX = 48  # 영역 한 변의 최소 화소: 너무 작으면 장면마다 노출이 흔들린다(가정)
PERIOD_S = 0.5  # 영역을 다시 정하는 최소 간격(가정)
HOLD_S = 2.0  # 안쪽 태그가 안 보여도 이 시간 동안은 마지막 영역을 그대로 둔다(가정)
CHANGE = 0.25  # 영역이 짧은 변의 이 비율 넘게 달라져야 다시 정한다(가정)
FAILURES_MAX = 20  # 카메라가 영역 설정을 이만큼 잇달아 거절하면 그만둔다(IR 영역 설정의 재시도 횟수와 같게)
INNER_RANGE_M = 3.5  # 태그 컷도 inner_bundle_max_range_m 도 알 수 없을 때의 거리(inner_bundle_max_range_m 의 기본값)
SHADE_HEIGHT = 2.0  # 그늘 상자의 세로 = 태그 높이의 이 배수(태그 위아래 빈 자리까지, 가정)
SHADE_LOST_S = 1.5  # 그늘 기준으로 바꾼 뒤 서 있는 채로 이 시간 동안 안쪽 태그가 한 장도 안 잡히면 그늘 기준을 끈다(가정)
SHADE_SETTLE_S = 3.0  # 바꾼 뒤 이 시간이 지나서도 안쪽 태그가 잡혔으면 노출이 자리 잡고도 보인 것: 그 뒤의 놓침은 노출 탓으로 보지 않는다
#                       (2026-11-19: 영역을 바꾼 뒤 화면 밝기가 자리 잡는 데 1.5~2초, 가정)
SHADE_STEADY = (3, 5)  # 최근 태그 결과 5개 중 3개 이상에서 안쪽 태그가 잡혀야(꾸준히 보여야) 그늘 기준으로 바꾼다(가정):
#                        꾸준히 보이던 태그여야 바꾼 뒤 사라졌을 때 노출 탓이라고 말할 수 있다


def corner_box(corners):
    """네 귀퉁이 화소 좌표를 덮는 상자 (x0, y0, x1, y1)."""
    xs = [float(p[0]) for p in corners]
    ys = [float(p[1]) for p in corners]
    return min(xs), min(ys), max(xs), max(ys)


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


def differs(a, b, change=CHANGE):
    """두 영역이 다시 정할 만큼 다른가."""
    if a is None or b is None:
        return a != b
    short = max(1, min(a[2] - a[0], a[3] - a[1]))
    return max(abs(x - y) for x, y in zip(a, b)) > change * short


def inner_range(inner, cut_x=None, accurate_m=None, configured=None):
    """안쪽 태그에 노출을 맞추기 시작하는 거리(카메라→안쪽 태그)[m]. 설정값(camera.color_ae_roi_inner_range_m)이 있으면 그 값.
    없으면 둘 중 먼 쪽(다가가면서 먼저 오는 쪽): 입구 태그가 화면에서 잘리는 자리에서 본 거리(태그 면 − 태그 컷; cut_x 는
    geometry.cut_position 의 카메라 x, 입구 = 0, 바깥 −)와 안쪽 태그가 정확해지는 거리(inner_bundle_max_range_m)."""
    if configured is not None:
        return float(configured)
    found = []
    plane = (inner or {}).get('plane_x_m')
    if plane is not None and cut_x is not None:
        found.append(float(plane) - float(cut_x))
    if accurate_m is not None:
        found.append(float(accurate_m))
    return max(found) if found else INNER_RANGE_M


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


def inner_seen(tag, max_range=None):
    """채택된 안쪽 태그: {ID 글자: (상자, 카메라에서 태그까지 거리[m])}. max_range 가 있으면, 가장 가까운 태그가 그보다 멀 때 빈 dict."""
    out = {}
    for key, part in (((tag or {}).get('inner') or {}).get('tags') or {}).items():
        if isinstance(part, dict) and part.get('ok') and part.get('corners') and part.get('camera_translation_m'):
            out[str(key)] = (
                corner_box(part['corners']),
                sum(float(v) ** 2 for v in part['camera_translation_m']) ** 0.5,
            )
    if max_range is not None and out and min(r for _, r in out.values()) > max_range:
        return {}
    return out


def tags_box(seen, width, height):
    """채택된 안쪽 태그를 덮는 영역(두 배로 넓힘)."""
    return widen(union([box for box, _ in seen.values()]), width, height)


def shade_box(seen, width, height, layout):
    """두 안쪽 태그 사이의 그늘진 틈(흰 종이를 뺀 자리)의 정수 상자. seen = inner_seen(...). 한 장만 채택됐으면 그 태그의 화면
    폭으로 다른 한 장 쪽 틈을 잡는다(두 태그는 한 평면이라 비스듬히 봐도 폭과 틈이 같은 비율로 줄어든다). 틈이 화면 밖이면 None."""
    if not layout or not seen:
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


class ColorExposure:
    """새 태그 결과가 올 때마다 자동노출 영역을 고르고, 달라졌을 때만 카메라에 넘긴다. 실패해도 주행을 막지 않는다.

    default = 카메라가 시작할 때 쓰고 있던 영역(읽지 못했으면 None → 화면 전체). 안쪽 태그가 inner_range_m 안에서 채택되기 전에는
    카메라를 건드리지 않는다. layout = shade_layout(...) 또는 None(그늘 기준 끔)."""

    def __init__(self, width, height, set_roi, default=None, layout=None, inner_range_m=INNER_RANGE_M):
        self.width = int(width)
        self.height = int(height)
        self.full = (0, 0, self.width - 1, self.height - 1)
        self.default = self.full if default is None else tuple(int(v) for v in default)
        self.set_roi = set_roi
        self.layout = layout
        self.inner_range_m = float(inner_range_m)
        self.current = self.default  # 카메라는 이미 기본 영역이다: 안쪽 태그에 맞출 때까지 넘기지 않는다
        self.kind = 'default'  # 지금 카메라에 걸려 있는 영역의 종류: default · tags · shade
        self.seen_at = None
        self.set_at = None
        self.failures = 0
        self.state = 'default (untouched) | inner within %.2f m' % self.inner_range_m
        self.shade_off = None  # 이 실행에서 그늘 기준을 끈 이유(끄지 않았으면 None)
        self.inner_at = None
        self.moved_at = None
        self.shade_at = None
        self.shade_check = False  # 지금 건 그늘 상자가 해로운지 아직 판정 중인가
        self.tight = None  # 그늘 기준으로 바꿀 때의 태그 자리 영역(되돌릴 곳)
        self.recent = deque(maxlen=SHADE_STEADY[1])

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

    def target(self, now, seen):
        """seen = 거리 안에서 채택된 안쪽 태그(없으면 빈 dict). (영역, 까닭)."""
        if seen:
            self.seen_at = now
            if self.layout is not None and not self.shade_off:
                # 들어갈 때만 꾸준함을 본다: 이미 그늘 기준이면 한두 번 놓쳐도 태그 자리로 되돌아가 왔다 갔다 하지 않는다
                if self.kind == 'shade' or sum(self.recent) >= SHADE_STEADY[0]:
                    box = shade_box(seen, self.width, self.height, self.layout)
                    if box is not None:
                        return box, 'shade'
            return tags_box(seen, self.width, self.height), 'tags'
        if self.kind != 'default' and self.seen_at is not None and now - self.seen_at <= HOLD_S:
            return self.current, 'held'
        return self.default, 'default'

    def update(self, now, tag, allow=True):
        """새 태그 결과 하나를 반영한다. 지금 상태를 글로 돌려준다(기록용).

        allow=False(차가 움직이는 중)이면 카메라에 영역을 넘기지 않는다. 영역을 넘기는 호출 한 번에 카메라가 0.1초쯤 멈춰
        IMU 가 80 ms 빠지고 영상이 137 ms 늦게 온다(2026-11-19 Jetson 실측, 호출이 없으면 0)."""
        if self.failures >= FAILURES_MAX:
            return self.state
        seen = inner_seen(tag, self.inner_range_m)
        if seen:
            self.inner_at = now
        if not allow:
            self.moved_at = now
        if self.blinded(now, allow):
            # 너무 밝아져 태그가 날아갔다: 이 실행에서는 그늘 기준을 끄고, 바꾸기 전의 태그 자리 영역으로 되돌린다
            self.shade_off = 'tags lost'
            self.recent.clear()
            return self.apply(now, self.tight or self.default, 'tags' if self.tight else 'default')
        self.recent.append(bool(seen))
        box, why = self.target(now, seen)
        if not allow:
            return self.state
        if why == 'held' or not differs(self.current, box):
            return self.state
        if self.set_at is not None and now - self.set_at < PERIOD_S:
            return self.state
        if why == 'shade':
            self.shade_check = True
            self.tight = tags_box(seen, self.width, self.height)
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
