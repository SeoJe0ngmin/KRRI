"""CAN 송신 안전층 — 데드맨·write 시각·TXACK·RX 스니핑. (plan 4-1·4-4 FM8a/FM9)

**다른 팀 파일(control_forklift_v2.py)의 구조는 건드리지 않는다.** 그쪽 TX 루프
(movement 10 ms / control 5 ms / heartbeat 200 ms)는 그대로 돌고, 우리는 두 군데만 낀다:

    1) `controller.ch_a` 를 ChannelProbe 로 감싼다 — write 를 그대로 흘려보내면서
       **payload 가 바뀐 첫 write 직후 time.time()** 을 t_cmd_tx 로 찍고(plan 4-1),
       같은 자리에서 read(timeout=0) 로 TXACK·차량 프레임을 배수한다(감사 전용).
    2) `current_movement` 를 우리만 바꾼다 + 데드맨 — 판단 루프가 DEADMAN_S 안에
       feed() 를 안 하면 stop 을 강제한다(FM8a).

데드맨이 못 막는 것: **호스트 전체 동결**(FM8b, 9/7 VM 사건). 같은 프로세스 스레드는
같이 멈춘다 — 차량 워치독 T_wd 만 남는데 그건 내일 first_run safety 로 실측한다.
그 전까지는 "미완화 리스크 + 사람이 제동 위치 + 97 한정" 이 대응이다.
"""
import asyncio
import time

#: 판단 루프가 이 시간 안에 갱신하지 않으면 stop 강제 [s]. 코드 내부 상수(plan 2-2: ros2_control·Nav2 0.3~0.5)
DEADMAN_S = 0.5
#: 데드맨 감시 주기 [s]
WATCH_S = 0.02
#: CAN 건강 지표를 보는 **창** [s]. 누적 최대를 보면 한 번 튄 값이 영영 안 내려가
#: 남은 주행 전체가 그 한 번에 묶인다(2026-09-21: 스파이크 1회로 180 s 를 제자리에서 소진).
HEALTH_WINDOW_S = 2.0
#: 주행 프레임 CAN ID. 여기 우리가 안 보낸 payload 가 오면 이중 송신원(FM11)이다.
MOVEMENT_CAN_ID = 0x1E3
#: 우리가 쓰는 동작 — 이 다섯(+저속 전진)만 byte1/2 를 쓰고 나머지는 중립이어야 한다
SAFE_MOVEMENTS = ("stop", "forward", "forward_slow", "backward", "rotate_ccw", "rotate_cw")
#: byte4 는 **포크 리프트**다(2026-09-07 실차). 회전·주행에 절대 안 쓴다.
FORK_BYTE = 4
#: 주행·제어 프레임 ID 와 중립값 — _fork_guard 가 쓴다(순환 import 를 피해 여기 둔다)
MOVEMENT_ID = 0x01E3
CONTROL_ID = 0x02E3
AN_NEUTRAL = 127


def check_templates(names=SAFE_MOVEMENTS, log=print, forward_slow_expect=None):
    """출발 전 CAN 템플릿 검사. byte1/2 외 비중립이면 **출발 거부**(SystemExit).

    2026-09-07 실차에서 byte4 가 포크 리프트였다(회전 명령이 포크를 올림).
    forward_slow_expect 를 주면 저속 전진 byte2 가 그 값인지도 본다(config FORWARD_SLOW 대조).
    """
    from .control_forklift_v2 import MOVEMENT_TEMPLATES as M, AN_NEUTRAL
    for name in names:
        if name not in M:
            raise SystemExit("!! CAN 템플릿에 %s 가 없다 — 출발하지 않는다" % name)
        tpl = M[name]
        if len(tpl) != 8:
            raise SystemExit("!! CAN 템플릿 %s 의 길이가 %d 다 (8 이어야 한다)" % (name, len(tpl)))
        bad = [i for i in (0, 3, 4, 5, 6, 7) if tpl[i] != AN_NEUTRAL]
        if bad:
            raise SystemExit(
                "!! CAN 템플릿 %s 의 byte%s 가 중립(%d)이 아니다 — byte%d 는 이 지게차에서 "
                "포크 리프트다. 출발하지 않는다" % (name, bad, AN_NEUTRAL, FORK_BYTE))
        if name == "stop" and (tpl[1] != AN_NEUTRAL or tpl[2] != AN_NEUTRAL):
            raise SystemExit("!! stop 템플릿이 중립이 아니다 — 출발하지 않는다")
        if name.startswith("rotate") and tpl[2] != AN_NEUTRAL:
            raise SystemExit("!! 회전 템플릿 %s 가 byte2(주행)를 쓴다 — 출발하지 않는다" % name)
        if name.startswith(("forward", "backward")) and tpl[1] != AN_NEUTRAL:
            raise SystemExit("!! 직진 템플릿 %s 가 byte1(조향)을 쓴다 — 출발하지 않는다" % name)
    if forward_slow_expect is not None and "forward_slow" in M:
        want = AN_NEUTRAL - int(forward_slow_expect)
        got = M["forward_slow"][2]
        if got != want:
            raise SystemExit("!! forward_slow byte2 가 %d 인데 config FORWARD_SLOW 로는 %d 여야 한다"
                             % (got, want))
    if log:
        log("  CAN 템플릿 확인: rotate_ccw byte1=%d  rotate_cw byte1=%d  forward byte2=%d  "
            "forward_slow byte2=%s  backward byte2=%d  (그 외 바이트 전부 중립)"
            % (M["rotate_ccw"][1], M["rotate_cw"][1], M["forward"][2],
               M.get("forward_slow", [None] * 3)[2], M["backward"][2]))
    return {k: list(M[k]) for k in names if k in M}


class ChannelProbe:
    """canlib Channel 을 감싸 **write 시각·TXACK·RX** 를 기록한다. 그 외는 그대로 통과."""

    TXACK_FLAG = 0x0040          # canstat.canMSG_TXACK

    def __init__(self, channel, logger=None, txack=True, rx=True):
        self._ch = channel
        self.logger = logger
        self.rx = bool(rx)
        # 차단 시험용(first_run safety, plan 4-4 FM9). 여기 든 CAN ID 는 **안 내보낸다** —
        # 다른 팀 TX 루프는 그대로 돌게 두고 우리 창구에서만 막는다.
        self.blocked = set()
        self.blocked_all = False
        self.blocked_writes = 0
        # 차단 시험은 '주행 명령을 막는' 것이지 '정지를 막는' 게 아니다. stop 템플릿
        # (byte1=byte2=중립)만은 절대 안 버린다 — 안 그러면 차단 구간에서 난 예외·Ctrl+C
        # 의 stop 프레임이 전부 여기서 사라져 차가 마지막 forward_slow 로 계속 간다.
        self.never_block_stop = True
        self.foreign_movement_frames = 0     # 우리가 안 보낸 0x1E3 수신 수 (FM11)
        self.last_payload = {}           # can_id -> tuple(bytes)
        self.writes = 0
        self.fork_guard_drops = 0        # 포크를 움직일 뻔해서 버린 프레임 수 (0 이어야 정상)
        self.tx_marks = []               # (t_cmd_tx, can_id, payload) — payload 가 바뀐 것만
        self.txack_ms = []               # write → TXACK 잔차 [ms]
        self.txack_t = []                # 그 시각 [s]
        self.write_gap_ms = []           # write 간격 [ms] (VM 스톨 감시)
        self.write_gap_t = []            # 그 시각 [s]
        self._t_last_write = None
        self._pending = []               # [(t_write, can_id, payload)]
        self.txack_on = False
        if txack:
            try:
                self._ch.iocontrol.local_txack = True
                self.txack_on = True
            except Exception:
                self.txack_on = False

    # -- 통과 --------------------------------------------------------------
    def __getattr__(self, name):
        return getattr(self._ch, name)

    @staticmethod
    def _is_stop_payload(frame):
        try:
            d = [int(b) for b in frame.data]
        except Exception:
            return False
        return len(d) == 8 and all(b == 127 for b in d)

    #: 주행 프레임에서 byte1(조향)·byte2(주행) 말고는 전부 중립이어야 한다.
    #: byte4 가 포크 리프트라 여기가 마지막 방어선이다(2026-09-07 사고).
    _DRIVE_FREE_BYTES = (1, 2)
    #: 제어 프레임(0x2E3)에서 **절대 안 보내는** 모드들. byte3 이 모드 선택자다.
    #: lift_mode 0x05 · lift_up 0x15 · lift_down 0x25 · fold 0x26 · unfold 0x16 ·
    #: reach_forward 0x19 … 우리가 쓰는 건 driving_mode 0x0A 뿐이다.
    _CTRL_ALLOW_BYTE3 = (0x0A,)

    def _fork_guard(self, frame):
        """포크·마스트를 움직일 수 있는 프레임이면 **버리고 알린다**.

        set_movement 가 이미 SAFE_MOVEMENTS 6개만 허용하고, 템플릿은 출발 전
        check_templates 가 본다. 여기는 **런타임에 값이 바뀌어도** 못 나가게 하는
        마지막 관문이다. 우리는 직진·후진·제자리회전만 한다.
        """
        try:
            cid = int(getattr(frame, "id", -1))
            data = [int(b) for b in frame.data]
        except Exception:
            return None
        if cid == MOVEMENT_ID and len(data) >= 8:
            bad = [i for i in range(8)
                   if i not in self._DRIVE_FREE_BYTES and data[i] != AN_NEUTRAL]
            if bad:
                return ("주행 프레임 byte%s 가 중립(%d)이 아니다 (byte%d = 포크 리프트)"
                        % (bad, AN_NEUTRAL, FORK_BYTE))
        if cid == CONTROL_ID and len(data) >= 4:
            if data[3] not in self._CTRL_ALLOW_BYTE3:
                return ("제어 프레임 모드 0x%02X — 우리는 driving_mode(0x0A) 만 쓴다 "
                        "(0x05/0x15/0x25 = 리프트, 0x16/0x26 = 폴딩)" % data[3])
        return None

    def write(self, frame):
        why = self._fork_guard(frame)
        if why is not None:
            self.fork_guard_drops += 1
            msg = "!! 포크 차단: %s — 이 프레임은 안 보낸다" % why
            if self.logger is not None:
                self.logger.can(dir="fork_blocked", can_id=int(getattr(frame, "id", -1)),
                                data=[int(b) for b in frame.data], why=why, t=time.time())
            if self.fork_guard_drops == 1:
                print("  " + msg)
            return None
        blocked = self.blocked_all or (self.blocked
                                       and int(getattr(frame, "id", -1)) in self.blocked)
        if blocked and self.never_block_stop and self._is_stop_payload(frame):
            blocked = False                  # 정지만은 언제나 나간다
        if blocked:
            self.blocked_writes += 1
            if self.logger is not None and self.blocked_writes % 50 == 1:
                self.logger.can(dir="blocked", can_id=int(getattr(frame, "id", -1)),
                                n=self.blocked_writes, t=time.time())
            return None
        out = self._ch.write(frame)
        t = time.time()                  # **write 직후** — 이게 t_cmd_tx 다
        self.writes += 1
        if self._t_last_write is not None:
            self.write_gap_ms.append((t - self._t_last_write) * 1000.0)
            self.write_gap_t.append(t)
        self._t_last_write = t
        try:
            cid = int(frame.id)
            payload = tuple(int(b) for b in frame.data)
        except Exception:
            return out
        if self.last_payload.get(cid) != payload:
            self.last_payload[cid] = payload
            self.tx_marks.append((t, cid, payload))
            self._pending.append((t, cid, payload))
            if len(self._pending) > 64:
                self._pending.pop(0)
            if self.logger is not None:
                # movement 를 같이 남긴다 — analyze 의 cmd_pairs 가 set↔tx 를 이 이름으로
                # 짝지어 **명령 지연**(결정→버스)을 낸다. 없으면 n=0 이 된다
                # (2026-09-21 현장: dry-run 은 fake_tx 에 movement 가 있어 통과했는데
                #  실주행 tx 에는 없어서 명령 지연이 통째로 안 나왔다).
                self.logger.can(dir="tx", can_id=cid, data=list(payload), t_cmd_tx=t,
                                movement=self._movement_now(), changed=True)
        if self.rx:
            self._drain()
        return out

    def _movement_now(self):
        """지금 송신 중인 동작 이름. SafeCanTx 가 붙어 있으면 그쪽 값을 쓴다."""
        try:
            c = getattr(self, "_owner", None)
            if c is not None:
                return getattr(c, "movement", None)
        except Exception:
            pass
        return None

    def _drain(self, limit=8):
        """read(timeout=0) 로 TXACK·차량 프레임을 배수한다. 감사 전용이라 실패는 무시."""
        for _ in range(limit):
            try:
                f = self._ch.read(timeout=0)
            except Exception:
                return
            if f is None:
                return
            t = time.time()
            try:
                cid = int(f.id)
                data = [int(b) for b in f.data]
                flags = int(getattr(f, "flags", 0) or 0)
            except Exception:
                continue
            is_ack = bool(flags & self.TXACK_FLAG)
            if not is_ack and cid == MOVEMENT_CAN_ID:
                mine = self.last_payload.get(cid)
                if mine is not None and list(mine) != data:
                    # 우리가 안 보낸 주행 프레임 = 리모컨·조이스틱이 같이 쏘고 있다
                    self.foreign_movement_frames += 1
            resid = None
            if is_ack:
                for i, (tw, c, p) in enumerate(self._pending):
                    if c == cid and list(p) == data:
                        resid = (t - tw) * 1000.0
                        self.txack_ms.append(resid)
                        self.txack_t.append(t)
                        self._pending.pop(i)
                        break
            if self.logger is not None:
                self.logger.can(dir="txack" if is_ack else "rx", can_id=cid, data=data,
                                flags=flags, t_rx=t, resid_ms=resid)

    def stats(self, now=None):
        """통계 한 줄.

        **주행 중 술어가 보는 값은 창(최근 HEALTH_WINDOW_S)이다.** 누적 최대
        (`*_max_ms`)는 보고·게이트용으로 남기되 건강 판정에는 쓰지 않는다 —
        한 번 튀면 영원히 안 내려가서 도킹을 브릭한다.
        """
        def p99(v):
            if not v:
                return None
            s = sorted(v)
            return s[min(len(s) - 1, int(round(0.99 * (len(s) - 1))))]

        def win_max(vals, ts):
            if not vals:
                return None
            t_end = (time.time() if now is None else now)
            w = [v for v, tv in zip(vals, ts) if tv >= t_end - HEALTH_WINDOW_S]
            return max(w) if w else None

        first10 = self.txack_ms[:10]
        return {"writes": self.writes, "changes": len(self.tx_marks),
                "fork_guard_drops": self.fork_guard_drops,   # 0 이 아니면 즉시 조사
                "txack_on": self.txack_on, "txack_n": len(self.txack_ms),
                "txack_first10_max_ms": max(first10) if first10 else None,
                "txack_p99_ms": p99(self.txack_ms),
                "txack_win_max_ms": win_max(self.txack_ms, self.txack_t),
                "write_gap_p99_ms": p99(self.write_gap_ms),
                "write_gap_win_max_ms": win_max(self.write_gap_ms, self.write_gap_t),
                "write_gap_max_ms": max(self.write_gap_ms) if self.write_gap_ms else None,
                "foreign_movement_frames": self.foreign_movement_frames}


class SafeCanTx:
    """우리 쪽 송신 창구. current_movement 는 **여기로만** 바꾼다."""

    def __init__(self, controller, logger=None, log=None, deadman_s=DEADMAN_S,
                 clock=None):
        self.c = controller
        self.logger = logger
        self.log = log
        self.deadman_s = float(deadman_s)
        # 시계. dry-run 은 가상 시계를 주입해 가짜 기록의 시각이 서로 맞게 한다.
        self.clock = clock or time.time
        self.probe = None
        self.movement = "stop"
        self.t_cmd_set = None
        self.t_feed = self.clock()
        self.deadman_trips = 0
        self._stopped_by_deadman = False

    # -- 수명 --------------------------------------------------------------
    def attach(self):
        """controller.ch_a 를 ChannelProbe 로 감싼다. CAN 연결 뒤에 부를 것."""
        ch = getattr(self.c, "ch_a", None)
        if ch is None or isinstance(ch, ChannelProbe):
            return self.probe
        self.probe = ChannelProbe(ch, logger=self.logger)
        self.probe._owner = self          # tx 로그에 movement 를 넣으려고
        self.c.ch_a = self.probe
        if self.log:
            self.log("  CAN write 계측 붙음 (TXACK %s)"
                     % ("켜짐" if self.probe.txack_on else "안 켜짐 — 감사만 못 한다"))
        return self.probe

    # -- 명령 --------------------------------------------------------------
    def set_movement(self, name, why=""):
        """동작을 바꾼다. 돌려주는 값 = t_cmd_set (plan 4-1 의 다섯째 시각)."""
        if name not in SAFE_MOVEMENTS:
            raise ValueError("허용되지 않은 동작: %s" % name)
        t = self.clock()
        self.movement = name
        self.t_cmd_set = t
        self.t_feed = t
        self._stopped_by_deadman = False
        self.c.current_movement = name
        if self.logger is not None:
            self.logger.can(dir="set", movement=name, t_cmd_set=t, why=why)
        return t

    def feed(self):
        """"판단 루프가 살아 있다" 는 신호. 명령을 유지하는 동안 계속 불러야 한다."""
        self.t_feed = self.clock()

    def stop(self, why="stop"):
        return self.set_movement("stop", why=why)

    # -- 데드맨 ------------------------------------------------------------
    def check(self, now=None):
        """한 번 검사. 데드맨이 걸렸으면 True."""
        now = self.clock() if now is None else now
        if self.movement == "stop":
            return False
        if now - self.t_feed > self.deadman_s:
            self.deadman_trips += 1
            if self.log:
                self.log("  !! 데드맨: 판단 루프가 %.2fs 갱신 없음 — stop 강제"
                         % (now - self.t_feed))
            if self.logger is not None:
                self.logger.event("deadman", was=self.movement,
                                  since_s=now - self.t_feed)
            # **set_movement 를 거친다** — 안 그러면 t_cmd_set 이 옛 전진 명령 시각에
            # 남고 can.jsonl 에 dir="set" 줄이 안 남아, 추정기의 명령버퍼가 코스팅을
            # 수 초 전부터 센 것으로 계산해 정지 판정이 실제 감속을 안 보고 통과한다.
            # 재귀 걱정 없다 — set_movement 가 t_feed 를 갱신하므로 다음 check 는 False.
            self.stop(why="deadman")
            self._stopped_by_deadman = True
            return True
        return False

    async def watchdog(self):
        """asyncio 태스크로 돌리는 데드맨. 루프가 멈추면 이 태스크도 멈춘다(FM8b)."""
        try:
            while True:
                self.check()
                await asyncio.sleep(WATCH_S)
        except asyncio.CancelledError:
            self.c.current_movement = "stop"
            raise

    @property
    def foreign_movement_frames(self):
        """우리가 안 보낸 0x1E3 수신 수 (FM11 이중 송신원). 프로브가 없으면 0."""
        return 0 if self.probe is None else self.probe.foreign_movement_frames

    def stats(self):
        # fork_guard_drops 는 아래 dict 에 합쳐진다(0 이 아니면 즉시 조사할 것)
        out = {"deadman_trips": self.deadman_trips, "deadman_s": self.deadman_s}
        if self.probe is not None:
            out.update(self.probe.stats(now=self.clock()))
        return out


__all__ = ["SafeCanTx", "ChannelProbe", "check_templates", "DEADMAN_S",
           "SAFE_MOVEMENTS"]
