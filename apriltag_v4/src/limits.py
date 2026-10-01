"""계산으로 나오는 값. config 에 박지 않는다 — 배치가 바뀌면 안 따라오기 때문.

가정값을 두지 않는다. 판정에 필요한 흔들림·오차는 전부 **그때 잰 값**을 인자로 받는다.
근거는 plan.md 1절 · 3-3 · 4-5 · 2026-09-30 결정 ④⑥(통로·후진).

intr 는 어느 꼴이든 받는다 — (w,h,fx,fy,cx,cy) 튜플 · measured.intrinsics 같은 dict ·
CameraIntrinsics 객체. **카메라에서 읽은 것**을 넘기는 게 원칙이고 D435I_COLOR_REF 는 폴백이다.
"""
import math

from config import control as C
from config import detection as D

#: 폴백. 카메라가 준 intrinsics 가 있으면 그걸 쓴다
COLOR_REF = D.D435I_COLOR_REF


def _intr(intr):
    """어떤 꼴이든 (w, h, fx, fy, cx, cy) 로. w·h 가 없으면 주점의 2배로 본다.

    None 이거나 비어 있으면(Measured().intrinsics 의 기본값 {}) 폴백 — 카메라를 아직 안 열었다는 뜻이다.
    """
    if intr is None or (isinstance(intr, (dict, tuple, list)) and not intr):
        return COLOR_REF
    if isinstance(intr, dict):
        g = intr.get
        fx, fy, cx, cy = g("fx"), g("fy"), g("cx"), g("cy")
        w, h = g("w", g("width")), g("h", g("height"))
    elif hasattr(intr, "fx"):
        fx, fy, cx, cy = intr.fx, intr.fy, intr.cx, intr.cy
        w = getattr(intr, "width", None) or getattr(intr, "w", None)
        h = getattr(intr, "height", None) or getattr(intr, "h", None)
    else:
        w, h, fx, fy, cx, cy = intr
    w = int(round(cx * 2)) if not w else int(w)
    h = int(round(cy * 2)) if not h else int(h)
    return w, h, float(fx), float(fy), float(cx), float(cy)


# ── 기하 ────────────────────────────────────────────────────────────
def lateral_leak_m(distance_m, heading_deg):
    """그 방향으로 그만큼 가면 옆으로 새는 양."""
    return distance_m * math.sin(math.radians(heading_deg))


def tag_cut_m(intr=None, height_diff_m=None):
    """태그 윗변이 화면 위로 나가는 거리. 높이차와 intrinsics 에서 나온다.

    height_diff_m 을 안 주면 config 의 **명목값**(TAG_HEIGHT_M − CAMERA_HEIGHT_M)이다 — 문서·자체시험용.
    실행 경로는 before_run 이 잰 높이차(또는 잰 tag_cut_m)를 넘긴다. 실측·명목을 섞지 않는다.
    """
    _, _, _, fy, _, cy = _intr(intr)
    dz = (D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M) if height_diff_m is None else float(height_diff_m)
    top = dz + D.TAG_SIZE_M / 2.0
    return top * fy / (cy - D.TAG_EDGE_MARGIN_PX)


def tag_cut_live_m(intr, height_diff_m, forward_m, top_px):
    """**지금 프레임**에서 다시 잰 태그컷 — 이 자세(피치)와 높이가 유지되면 윗변이 여유선(TAG_EDGE_MARGIN_PX)에
    닿는 거리. 오르막·요철·카메라 처짐이 전부 들어온다 (2026-10-01: 거리 숫자 하나로 예측하지 않는다).

    height_diff_m 은 자세가 준 카메라 높이(−Fix.vertical_m, 태그 좌표계 = 피치와 무관한 **위치**),
    top_px 는 화면에서 실제로 본 윗변 행(Fix.top_px). 둘의 차가 곧 카메라 피치다:
        피치 = atan(윗변높이 ÷ 거리) − atan((cy − 윗변행) ÷ fy)       (+ 위를 본다 → 태그가 화면 아래로 → 더 가까이 가도 된다)
        태그컷 = 윗변높이 ÷ tan(여유각 + 피치)
    피치 0 이면 tag_cut_m 과 같은 식. 눈 감은 뒤 경사가 바뀌는 건 어떤 식으로도 못 본다.
    """
    _, _, _, fy, _, cy = _intr(intr)
    top = float(height_diff_m) + D.TAG_SIZE_M / 2.0
    e_margin = math.atan2(cy - D.TAG_EDGE_MARGIN_PX, fy)
    pitch = math.atan2(top, max(1e-6, float(forward_m))) - math.atan2(cy - float(top_px), fy)
    return top / math.tan(max(1e-3, e_margin + pitch))


def look_up_deg(distance_m):
    """태그를 올려다보는 각. 0 에 가까울수록 PnP 두 해가 헷갈린다."""
    return math.degrees(math.atan2(D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M, distance_m))


def half_fov_deg(intr=None):
    """가로 반화각 (좌·우 중 작은 쪽). 회전 상한의 근거."""
    w, _, fx, _, cx, _ = _intr(intr)
    return min(math.degrees(math.atan2(cx, fx)),
               math.degrees(math.atan2(w - cx, fx)))


def edge_margin_deg(intr=None):
    """가장자리 여유(TAG_EDGE_MARGIN_PX)를 각으로. 화면 끝에 이만큼 붙으면 잘리기 직전이다."""
    _, _, fx, _, _, _ = _intr(intr)
    return math.degrees(math.atan2(D.TAG_EDGE_MARGIN_PX, fx))


# ── 통로 · 후진 (2026-09-30 결정 ④⑥) ─────────────────────────────────
def path_angle_max_deg(z_arrive_m, intr=None):
    """도착점(태그면에서 z_arrive_m)에서도 태그가 화면에 남는 최대 경로각 [도].

    비스듬히 직진해 (0, z_arrive) 에 닿는 순간 태그는 경로각만큼 옆에 보인다.
    그 위에 태그 반폭과 가장자리 여유까지 얹어도 반화각 안이어야 한다.
    """
    tag = math.degrees(math.atan2(D.TAG_SIZE_M / 2.0, max(1e-6, z_arrive_m)))
    return max(0.0, half_fov_deg(intr) - tag - edge_margin_deg(intr))


def _corridor_slope(intr, cut_m):
    """반폭 = 이 기울기 x (거리 - 태그컷). CORRIDOR_K 는 광운대 80 % 안전계수."""
    return C.CORRIDOR_K * math.tan(math.radians(path_angle_max_deg(cut_m, intr)))


def corridor_half_m(distance_m, intr=None, cut_m=None):
    """이 거리에서 한 번의 대각 직진으로 태그를 안 놓치고 닿을 수 있는 좌우 반폭 [m].

    distance_m 은 태그면까지 법선 거리(Fix.forward_m). 밖이면 사이드스텝(sidestep.py).
    cut_m 은 before_run 이 잰 태그컷(measured.tag_cut_m). 없으면 계산값.
    """
    cut = tag_cut_m(intr) if cut_m is None else float(cut_m)
    return max(0.0, _corridor_slope(intr, cut) * (distance_m - cut))


def backup_needed_m(lateral_m, distance_m, intr=None, cut_m=None):
    """지금 좌우가 통로 안에 들도록 뒤로 물러나야 하는 거리 [m]. 이미 안이면 0.

    반폭(거리) = 기울기 x (거리 - 태그컷) 을 거리로 풀면
    필요거리 = 태그컷 + |좌우| / 기울기.  뒤공간과의 비교는 sidestep.backup() 이 한다.
    """
    cut = tag_cut_m(intr) if cut_m is None else float(cut_m)
    need = cut + abs(lateral_m) / _corridor_slope(intr, cut) - distance_m
    return max(0.0, need)


# ── 눈을 감아도 되나 ────────────────────────────────────────────────
def arrival_lateral_m(lateral_m, heading_deg, blind_m=None):
    """지금 이대로 눈 감고 가면 도착했을 때 좌우가 얼마일까."""
    blind = C.BLIND_M if blind_m is None else blind_m
    return lateral_m + lateral_leak_m(blind, heading_deg)


def commit_margin_m(lateral_m, heading_deg, lateral_sigma_m, heading_sigma_deg,
                    blind_m=None):
    """눈 감아도 되나 — 예측 도착점에 흔들림을 얹고도 남는 여유 [m].

    양수면 간다. 음수면 더 고친다. 좌우 예산을 방향/위치로 미리 쪼개지 않는다 —
    쪼개려면 "좌우 위치가 몇 mm 를 쓴다" 를 가정해야 하는데 그건 아직 모른다.

    ★주의: 이 판정이 빡빡해서 **영영 양수가 안 나올 수 있다.**
    고치고 다시 재도 수렴 한계(최소 회전각·최소 전진량)가 예산보다 나쁘면
    계속 고치기만 한다. 그래서 부르는 쪽이 반드시 상한을 건다 —
    MAX_CORRECTIONS · FINE_TIME_LIMIT_S · MAX_STEPS (plan 3-10).
    상한에 걸리면 "수렴 못 함" 으로 멈추고 기록한다. 무한 반복은 없어야 한다.
    """
    blind = C.BLIND_M if blind_m is None else blind_m
    worst = (abs(arrival_lateral_m(lateral_m, heading_deg, blind))
             + abs(lateral_sigma_m)
             + abs(lateral_leak_m(blind, heading_sigma_deg)))
    return C.SIDE_GAP_M - worst


def offset_target_m(heading_deg, blind_m=None):
    """좌우 미리 비켜서기(plan 3-9) — 정렬할 때 목표로 삼을 좌우값.

    비뚤어진 만큼 반대쪽에서 출발하면 도착했을 때 상쇄된다. 추가 동작은 없다.
    """
    blind = C.BLIND_M if blind_m is None else blind_m
    return -lateral_leak_m(blind, heading_deg)


# ── 한 걸음 상한 ────────────────────────────────────────────────────
def turn_cap_deg(center_known, tag_range_m=None, center_m=None, intr=None):
    """한 걸음 회전 상한. 화각과 회전중심 불확실성 중 작은 쪽."""
    if not center_known:
        return C.TURN_MAX_UNKNOWN_CENTER_DEG
    cap = C.TURN_HARD_MAX_DEG
    if tag_range_m and center_m is not None:
        # 돌면 태그가 화면에서 (거리+중심)/거리 배로 움직인다 (광운대 bearing_gain)
        gain = max(0.2, (tag_range_m + center_m) / tag_range_m)
        # 돌고 나서도 태그가 화면에 남을 여유 — 경로각 상한과 같은 식
        cap = min(cap, path_angle_max_deg(tag_range_m, intr) / gain)
    return cap


def step_forward_max_m(heading_sigma_deg, lateral_room_m, tag_range_m, remaining_m,
                       intr=None, cut_m=None):
    """한 걸음 최대 전진. 좌우여유·화각·남은거리 중 제일 작은 것.

    lateral_room_m 은 지금 남아 있는 좌우 여유다(가정값이 아니라 그때 계산한 값).
    """
    sigma = max(0.05, abs(heading_sigma_deg))
    by_room = max(0.0, lateral_room_m) / math.sin(math.radians(sigma))
    cut = tag_cut_m(intr) if cut_m is None else float(cut_m)    # 실행 경로는 잰 태그컷을 넘긴다
    by_view = max(0.0, tag_range_m - cut)
    return max(0.0, min(by_room, by_view, remaining_m, C.STEP_FORWARD_HARD_MAX_M))


# ── 2차(전체 진입) 전용 · 문서용 ────────────────────────────────────
def heading_tol_inside_deg(vehicle_len_m):
    """비스듬한 차체가 탑재부 폭을 먹는 제약. 1차에는 안 걸린다(차가 밖에 있다).

    (안에 든 길이) x sin(방향) <= 좌우 여유 x 2.  ★지게차 길이가 필요하다.
    """
    return math.degrees(math.asin(min(1.0, 2 * C.SIDE_GAP_M / vehicle_len_m)))


def heading_tol_deg(lateral_budget_m, blind_m=None):
    """문서·보고용 — 좌우에 그만큼 떼어줬을 때 방향에 남는 몫.

    운용 판정에는 쓰지 않는다(commit_margin_m 이 한다). 인자로 받는 이유는
    "무엇을 가정한 값인지" 가 부르는 쪽에 드러나게 하려는 것이다.
    """
    blind = C.BLIND_M if blind_m is None else blind_m
    room = C.SIDE_GAP_M - abs(lateral_budget_m)
    return 0.0 if room <= 0 else math.degrees(math.asin(min(1.0, room / blind)))


if __name__ == "__main__":
    # 자체 시험 — 하드웨어 없이. intr 세 꼴이 같은 답을 내고, 통로 표가 말이 되는지
    ref = COLOR_REF
    as_dict = dict(zip(("w", "h", "fx", "fy", "cx", "cy"), ref))

    class _Obj:                                  # CameraIntrinsics 흉내
        width, height, fx, fy, cx, cy = ref
    for f in (tag_cut_m, half_fov_deg, edge_margin_deg):
        a, b, c = f(ref), f(as_dict), f(_Obj())
        assert abs(a - b) < 1e-9 and abs(a - c) < 1e-9, f.__name__
        assert f({}) == f(None) == a, f.__name__                 # 안 잰 intrinsics({}) 는 폴백
    # measured.intrinsics 처럼 width/height 이름이어도, w·h 가 아예 없어도 같은 답
    assert abs(tag_cut_m({"fx": ref[2], "fy": ref[3], "cx": ref[4], "cy": ref[5]}) - tag_cut_m(ref)) < 1e-9
    assert abs(half_fov_deg(dict(width=ref[0], height=ref[1], fx=ref[2], fy=ref[3], cx=ref[4], cy=ref[5]))
               - half_fov_deg(ref)) < 1e-9
    cut = tag_cut_m()
    # 실시간 식: 피치 0 (윗변 행이 피치 없는 모델 그대로) 이면 고정식과 같고, 위를 보면(윗변이 더 아래 행) 가까워진다
    dz = D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M
    fwd = 5.0
    row0 = ref[5] - ref[3] * (dz + D.TAG_SIZE_M / 2) / fwd
    assert abs(tag_cut_live_m(ref, dz, fwd, row0) - cut) < 1e-9
    assert tag_cut_live_m(ref, dz, fwd, row0 + 30) < cut < tag_cut_live_m(ref, dz, fwd, row0 - 30)
    assert abs(tag_cut_live_m(ref, dz, cut, D.TAG_EDGE_MARGIN_PX) - cut) < 1e-9     # 윗변 행이 여유선(60 px)에 선 순간 = 태그컷
    pmax = path_angle_max_deg(cut)
    print("태그컷 %.2f m · 반화각 %.1f도 · 가장자리 %.2f도 · 경로각 상한 %.1f도"
          % (cut, half_fov_deg(), edge_margin_deg(), pmax))
    assert abs(pmax - (half_fov_deg() - math.degrees(math.atan2(D.TAG_SIZE_M / 2, cut))
                       - edge_margin_deg())) < 1e-9
    print("\n거리[m]  통로 반폭[m]   (좌우 1.0 m 일 때 후진 필요량[m])")
    for d in (3.0, 3.3, 4.0, 5.0, 6.0, 8.0, 10.0):
        half = corridor_half_m(d)
        print("  %4.1f     %.3f          %.3f" % (d, half, backup_needed_m(1.0, d)))
    assert corridor_half_m(cut) == 0.0 and corridor_half_m(2.0) == 0.0     # 태그컷 안쪽은 통로가 없다
    assert corridor_half_m(8.0) > corridor_half_m(5.0) > 0.0
    # 후진 필요량은 "그 거리에서 반폭 == |좌우|" 가 되는 지점까지 — 되돌려서 확인
    for lat in (0.5, 1.0, 2.0):
        d0 = 5.0
        back = backup_needed_m(lat, d0)
        assert abs(corridor_half_m(d0 + back) - lat) < 1e-9 or back == 0.0, lat
    assert backup_needed_m(0.1, 8.0) == 0.0                                  # 이미 안이면 0
    # 측정 태그컷을 넘기면 그걸 쓴다
    assert corridor_half_m(6.0, cut_m=3.0) > corridor_half_m(6.0, cut_m=3.5)
    # 회전 상한은 경로각 상한을 넘지 않는다 (같은 여유 식)
    assert turn_cap_deg(True, tag_range_m=3.5, center_m=-1.5) <= C.TURN_HARD_MAX_DEG
    assert turn_cap_deg(False) == C.TURN_MAX_UNKNOWN_CENTER_DEG
    print("\nlimits 자체 시험 통과")
