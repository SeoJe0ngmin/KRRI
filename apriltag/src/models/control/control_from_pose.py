"""도킹값 -> 주행 명령. (아직 비어 있음)

detection_pose 가 낸 lateral / forward / heading 을 받아
control_forklift_v2 의 CAN 명령으로 바꾼다.
직진 시간은 fwd_time_model 이 계산한다. 회전 모델은 아직 없다.


도킹 순서 — 각 단계에서 태그를 보는지
────────────────────────────────────────────────────────────────────────

  0  멈춤          [측정]  30프레임 모아 중앙값. 움직이며 재지 않는다
                          1프레임은 lateral ±9.0mm / heading ±0.45도 로 흔들린다
                          30프레임 평균이면 ±1.4mm / ±0.08도

  1  회전 (90-heading)도    돌다가 태그가 시야 밖으로 나간다(좌우 35.2도)
                          heading 만큼이 아니다 — 옆으로 가려면 태그 축과 직각을 봐야 한다

  2  직진 |lateral|  [실명]  옆을 보고 가므로 태그가 안 보인다. **개루프**
                          그래서 나누지 않고 한 번에 간다
                          (나눌 때마다 회전이 2번씩 붙어 훨씬 비싸다)

  3  회전 90도              태그가 다시 들어온다. heading 도 0 이 된다
                          20도 -> (90-20)회전 -> 90도 -> -90회전 -> 0도

  0' 멈춤          [측정]  lateral 이 아직 LAT_TOL 보다 크면 1~3 반복
                          2단계가 개루프라 한 번에 맞을 거라고 보면 안 된다

  4  직진 min(남은거리, STEP_M)   태그를 보며 간다
                          조각마다 멈춰서 [측정] -> 4 반복
                          3m 를 1m 씩 3조각이면 1.46배 느려진다(측정 포함)

     멈춤          [측정]  태그1 이 화면에서 사라지기 전에 정지
                          태그높이-카메라높이 = 0.4m 면 1.31m 에서 잘린다

  5  태그2 검출      [측정]  tag_layout 으로 같은 좌표계 환산 -> 진입
                          환산 없이 바꾸면 좌표가 튄다

────────────────────────────────────────────────────────────────────────

태그를 못 보는 구간은 2단계 하나뿐이다.
직진(4)은 계속 보이므로 나눠 가며 매번 고쳐 잡는다.


정해야 할 값

    LAT_TOL   lateral 이 이보다 작으면 됐다고 본다
              명령 하한이 1.0s = 실이동 17mm, 측정 오차가 1.4mm
    STEP_M    직진 한 조각. 1.0m 면 1.46배 느려짐
    STOP_M    태그1 을 놓기 전 멈출 거리. 지금 높이차면 1.31m


CAN 명령 (control_forklift_v2.py, MOVEMENT_TEMPLATES)
────────────────────────────────────────────────────────────────────────

    8바이트, 중립 127. 우리가 쓰는 것은 다섯 개다.

        동작            byte0 1   2   3   4   5   6   7
        정지            127 127 127 127 127 127 127 127
        전진            127 127  67 127 127 127 127 127     byte2 = 127-60
        후진            127 127 187 127 127 127 127 127     byte2 = 127+60
        제자리 좌회전    127 127 127 127 118 127 127 127     byte4 = 127-9
        제자리 우회전    127 127 127 127 127 118 127 127     byte5 = 127-9

    주행 중 조향(byte1)은 안 쓴다. 직진과 제자리회전만으로 위 순서가 다 된다.

    **명령에 양이 없다.** 켜고 끄는 것뿐이라 "얼마나" 는 시간으로 준다.
        전진 N초    -> fwd_time_model.fwd_sec_from_offset_piecewise(거리)
        회전 N초    -> 모델 없음


막고 있는 것 — 회전 각속도

    직진은 로그로 적합된 모델이 있다(t0=0.507s 지연, vmax=0.284m/s).
    회전은 같은 것이 없다. **"몇 도 돌리려면 몇 초"를 모른다.**

    우리 카메라로 직접 잴 수 있다. fwd_time_model 을 만든 방식 그대로다:
        1) 태그를 정면에 두고 [측정] -> heading0
        2) 제자리 회전 N초 명령
        3) 멈춰서 [측정] -> heading1
        4) N 을 바꿔가며 반복 -> (시간, 각도) 를 적합
    한 번에 35도 넘게 돌면 태그가 시야 밖으로 나가니 작게 나눠 잴 것.
    직진처럼 지연 t0 가 있을 테니 절편도 같이 적합해야 한다.


코드와 스펙표가 다른 곳

    control_forklift_v2.py:73-74  JOYSTICK_ROTATE_CCW/CW = 30  ->  127-30 = 97
    스펙표는 118 (= 127-9). 같은 줄 주석에는 "# 118" 이라고 적혀 있다.
    97 은 중립에서 더 멀어 회전이 빠를 것이다. 의도한 것인지 확인할 것.
"""


from ...config import (HEAD_TOL_DEG, LAT_TOL_M, ROT_DEG_PER_SEC, ROT_MAX_SEC,
                       ROT_MIN_SEC, ROT_T0_SEC, STEP_M, STOP_M, TAG_ID)
from .fwd_time_model import fwd_sec_from_offset_piecewise


# ── 부호 약속 (실장비로 확인할 것) ──────────────────────────────────────────
#
#   rotate_ccw  ->  heading 이 **증가**한다
#   rotate_cw   ->  heading 이 **감소**한다
#
# heading 은 atan2(fwd[0], fwd[2]) 로, 태그의 +z 에서 +x 쪽으로 잰 각이다.
# 그게 지게차의 어느 쪽 회전인지는 태그를 어떻게 붙였느냐에 달렸다.
# **반대면 지게차가 반대로 돈다.** 처음 한 번은 이렇게 확인할 것:
#     measure() -> rotate_ccw 1초 -> 정지 -> measure()
#     heading 이 커졌으면 이 약속이 맞다. 작아졌으면 아래 두 곳을 맞바꿔라.
#         plan_step 의 "rotate_ccw" if head < 0 else "rotate_cw"
#         dock 의     rotate_ccw if turn > 0 else rotate_cw


# ── 회전 시간 ───────────────────────────────────────────────────────────────

def rot_sec_from_deg(deg):
    """돌 각도[도] -> 명령 시간[s]. **아직 실측 모델이 아니다.**

    직진과 같은 꼴로 가정했다 — 지연 t0 를 더하고 각속도로 나눈다.
    ROT_T0_SEC / ROT_DEG_PER_SEC 가 측정값이 아니므로 **결과를 믿지 마라.**

    재는 법 (fwd_time_model 을 만든 방식 그대로):
        1) 태그를 정면에 두고 measure() -> heading0
        2) 제자리 회전 N초 명령, 정지
        3) measure() -> heading1,  돈 각도 = heading1 - heading0
        4) N 을 1,2,3,4s 로 바꿔가며 반복. (N, 각도) 를 직선으로 적합
           기울기 = 각속도[도/s],  x절편 = 지연 t0[s]
        한 번에 35도 넘게 돌면 태그가 시야 밖으로 나간다. 작게 나눠 잴 것.
    """
    d = abs(float(deg))
    if not d:
        return 0.0
    t = ROT_T0_SEC + d / ROT_DEG_PER_SEC
    return max(ROT_MIN_SEC, min(ROT_MAX_SEC, t))


def _norm180(deg):
    """각도를 -180..180 으로."""
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return d + 360.0 if d <= -180.0 else d


def _sidestep(lat, head):
    """옆이동 한 묶음을 정한다. (지금 방식 — 비스듬히 접근은 보류)

    비스듬히 가는 대안이 2~3배 빠르고 태그도 안 놓친다(목표점을 향해 회전 ->
    한 번에 직진 -> 정면으로 회전). 회전이 작아 시야 +-35도 안에 남기 때문이다.
        lateral 0.08m / forward 3.0m 에서  옆이동 21.4s  vs  비스듬히 8.8s
    **일단 이 방식이 실제로 되는지 본 뒤에 고민한다.** -> (회전각[도], 거리[m], "forward"|"backward")

    lateral 이 +면 태그축의 오른쪽에 있다는 뜻이라 **왼쪽(-x)** 으로 가야 한다.
    그 방향을 보려면 heading 이 -90도, 반대면 +90도여야 한다.

    전진과 후진 둘 다 되므로(CAN 에 both 있다) **회전이 작은 쪽**을 고른다.
    그러면 회전이 90도를 절대 안 넘는다 — 크게 돌수록 태그를 오래 놓친다.
    복귀 회전은 어느 쪽이든 90도다(회전 뒤 heading 이 +-90 이 되므로).
    """
    face = -90.0 if lat > 0 else 90.0        # 가야 할 방향을 보는 heading
    turn_f = _norm180(face - head)            # 그대로 보고 전진
    turn_b = _norm180(face + 180.0 - head)    # 반대를 보고 후진
    if abs(turn_f) <= abs(turn_b):
        return turn_f, abs(lat), "forward"
    return turn_b, abs(lat), "backward"


# ── 다음 동작 하나를 고른다 ─────────────────────────────────────────────────

def plan_step(m, lat_tol=None, head_tol=None, step_m=None, stop_m=None):
    """측정값 하나 -> 다음에 할 동작 하나. **순수 함수다** (하드웨어가 필요 없다).

    m 은 measure() 의 반환값. 돌려주는 것:
        (동작, 양, 명령시간[s], 이유)
        동작 = "rotate_ccw" | "rotate_cw" | "forward" | "done" | "hold"

    한 번에 하나만 고른다. 실행한 뒤 다시 재고 다시 부르는 식이다.
    2단계(옆이동)만 세 동작을 묶어 돌려준다 — 그 사이엔 태그가 안 보여서
    중간에 다시 잴 수가 없기 때문이다.
    """
    lat_tol = LAT_TOL_M if lat_tol is None else lat_tol
    head_tol = HEAD_TOL_DEG if head_tol is None else head_tol
    step_m = STEP_M if step_m is None else step_m
    stop_m = STOP_M if stop_m is None else stop_m

    if m is None:
        return ("hold", 0.0, 0.0, "태그를 못 봤다")
    if not m["stable"]:
        return ("hold", 0.0, 0.0, "흔들린다: " + ", ".join(m["reasons"]))

    lat, fwd, head = m["lateral"], m["forward"], m["heading_deg"]

    # ① 옆으로 벗어나 있으면 — 회전, 직진, 회전을 한 묶음으로
    if abs(lat) > lat_tol:
        if not m["reliable_angle"]:
            return ("hold", 0.0, 0.0,
                    "각도를 못 믿는다 (tilt %.1f도). 비스듬한 자리로 옮겨서 다시 재라"
                    % m["tilt_deg"])
        turn, dist, way = _sidestep(lat, head)
        sec = (rot_sec_from_deg(turn) + fwd_sec_from_offset_piecewise(dist)
               + rot_sec_from_deg(90.0))
        return ("sidestep", (turn, dist, way), sec,
                "lateral %.0fmm 를 지운다 (%+.1f도 회전 -> %.0fmm %s -> 90도 복귀)"
                % (lat * 1000, turn, dist * 1000,
                   "전진" if way == "forward" else "후진"))

    # ② 방향만 틀어져 있으면 (옆이동이 보통 같이 고쳐서 잘 안 온다)
    if abs(head) > head_tol and m["reliable_angle"]:
        return ("rotate_ccw" if head < 0 else "rotate_cw", abs(head),
                rot_sec_from_deg(head), "heading %.1f도 를 지운다" % head)

    # ③ 앞으로 — 태그를 보며 조각내서
    gap = fwd - stop_m
    if gap > lat_tol:
        d = min(gap, step_m)
        return ("forward", d, fwd_sec_from_offset_piecewise(d),
                "남은 %.2fm 중 %.2fm 전진 (태그를 보며)" % (gap, d))

    return ("done", 0.0, 0.0, "태그1 구간 끝. forward %.2fm — 태그2 로 넘어갈 것" % fwd)


# ── 루프 ────────────────────────────────────────────────────────────────────

async def dock(pipe, driver, tag_id=None, max_steps=30, log=print):
    """멈춤 -> 측정 -> 동작 하나 -> 반복. 태그1 구간까지만 한다.

    **async 인 이유**: control_forklift_v2 의 TX 루프(movement 10ms, control 5ms,
    heartbeat 200ms)가 같은 이벤트 루프에서 계속 돌아야 한다. 멈추면 수신기
    워치독이 물어서 지게차가 선다. measure() 는 30프레임 = 약 1초 블로킹이라
    to_thread 로 빼서 TX 가 안 끊기게 한다.

    driver 는 async forward/backward/rotate_ccw/rotate_cw/stop(초) 를 갖춘 무엇이든.
    DryRunDriver 를 넣으면 CAN 없이 순서만 확인할 수 있다.
    """
    import asyncio
    from ..detection.detection_pose import measure
    tag_id = TAG_ID if tag_id is None else tag_id
    history = []

    for i in range(max_steps):
        m = await asyncio.to_thread(measure, pipe, tag_id)
        action, amount, sec, why = plan_step(m)
        log("[%2d] %-10s %s" % (i, action, why))
        history.append((action, amount, sec, why))

        if action == "done":
            await driver.stop()
            return history
        if action == "hold":
            await asyncio.sleep(0.2)
            continue
        if action == "sidestep":
            turn, dist, way = amount
            spin = driver.rotate_ccw if turn > 0 else driver.rotate_cw
            await spin(rot_sec_from_deg(turn))
            drive = driver.forward if way == "forward" else driver.backward
            await drive(fwd_sec_from_offset_piecewise(dist))
            back = driver.rotate_cw if turn > 0 else driver.rotate_ccw
            await back(rot_sec_from_deg(90.0))
        elif action == "forward":
            await driver.forward(sec)
        elif action == "rotate_ccw":
            await driver.rotate_ccw(sec)
        elif action == "rotate_cw":
            await driver.rotate_cw(sec)
        await driver.stop()

    log("!! %d 단계를 넘겼다. 수렴하지 않는다" % max_steps)
    return history


# ── 명령을 보내는 층 ────────────────────────────────────────────────────────

class DryRunDriver:
    """아무것도 안 보내고 찍기만 한다. CAN 하드웨어 없이 순서를 볼 때."""

    def __init__(self, log=print, realtime=False):
        self.log = log
        self.realtime = realtime      # True 면 명령 시간만큼 실제로 기다린다
        self.sent = []

    async def _do(self, what, sec):
        import asyncio
        self.sent.append((what, sec))
        self.log("       -> %-11s %5.2fs" % (what, sec))
        if self.realtime and sec:
            await asyncio.sleep(sec)

    async def forward(self, sec):
        await self._do("forward", sec)

    async def backward(self, sec):
        await self._do("backward", sec)

    async def rotate_ccw(self, sec):
        await self._do("rotate_ccw", sec)

    async def rotate_cw(self, sec):
        await self._do("rotate_cw", sec)

    async def stop(self):
        await self._do("stop", 0.0)


class CanDriver:
    """control_forklift_v2 의 컨트롤러에 명령을 태운다.

    그쪽 코드를 **한 줄도 안 고친다.** movement_tx_loop 이 10ms 마다
    MOVEMENT_TEMPLATES[self.current_movement] 를 보내고 있으므로,
    우리는 그 변수만 바꿨다가 되돌리면 된다. 키보드 루프가 하던 일과 같다.

        current_movement = "forward"   ->  N초 대기  ->  "stop"

    control_tx_loop / heartbeat_loop 는 그대로 돌아야 하므로
    이 코루틴에서 블로킹하면 안 된다(asyncio.sleep 만 쓴다).
    """

    def __init__(self, controller, log=None):
        self.c = controller
        self.log = log

    async def _hold(self, movement, sec):
        import asyncio
        if self.log:
            self.log("       -> %-11s %5.2fs" % (movement, sec))
        self.c.current_movement = movement
        try:
            if sec:
                await asyncio.sleep(sec)
        finally:
            self.c.current_movement = "stop"
        await asyncio.sleep(0.15)          # 관성이 잦아들 시간. 재기 전에 확실히 선다

    async def forward(self, sec):
        await self._hold("forward", sec)

    async def backward(self, sec):
        await self._hold("backward", sec)

    async def rotate_ccw(self, sec):
        await self._hold("rotate_ccw", sec)

    async def rotate_cw(self, sec):
        await self._hold("rotate_cw", sec)

    async def stop(self):
        self.c.current_movement = "stop"
