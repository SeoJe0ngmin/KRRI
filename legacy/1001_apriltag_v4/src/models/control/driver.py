"""CAN 송신 창구. 여섯 동작만 나간다.

채널·스레드·하트비트는 광운대 kwu/control.py 가 전부 들고 있다. 여기는 그 앞의 관문이다:
출발 전 동결 테이블 확인 · 동작 화이트리스트 · 데드맨 · 송신 시각 기록 (plan 9절).

시각은 clock.now()(monotonic) 하나만 쓴다. 기록에는 사람이 읽을 벽시계를 wall 로 병기한다.
"""
from config import control as C
from ...utils import clock
from ..kwu import control as kwu
from . import frames

#: 우리가 부르는 여섯 개. 포크·폴딩·리치 함수는 이름조차 안 적는다
MOVES = {
    "stop": kwu.issue_command_stop,
    "forward": kwu.issue_command_forward,
    "forward_slow": kwu.issue_command_forward_slow,
    "backward": kwu.issue_command_backward,
    "rotate_left_slow": lambda: kwu.issue_command_rotate_in_place(+1),
    "rotate_right_slow": lambda: kwu.issue_command_rotate_in_place(-1),
}


class Aborted(RuntimeError):
    """사람이 세운 뒤에 나온 명령. 어떤 동작도 다시 시작하면 안 된다 (검토 지적: Ctrl+C 뒤 재출발)."""


class Driver:
    """current_movement 를 바꾸는 유일한 자리. 정지는 기록보다 먼저 나간다."""

    def __init__(self, dry_run=False, recorder=None, log=print):
        self.dry_run = bool(dry_run)
        self.rec = recorder
        self.log = log
        self.movement = "stop"
        self.aborted = False            # abort() 뒤 True. set/lease 가 전부 막힌다
        self.table = None
        self.t_cmd = 0.0
        self.rotate_deflection = C.ROTATE_JOYSTICK_DEFLECTION   # 지금 회전 프레임에 써 있는 강도

    # ── 수명 ────────────────────────────────────────────────────────
    def open(self, channel=None, bitrate=None):
        """동결 테이블을 써넣고 확인한 **뒤에** 버스를 연다. 다르면 출발하지 않는다."""
        self.table = frames.apply_and_verify(log=self.log)
        kwu.configure_can_enabled(not self.dry_run)
        if self.rec is not None:
            kwu.configure_can_tx_observer(self._on_tx)
        if self.dry_run:
            if self.log:
                self.log("  dry-run — CAN 은 안 보낸다")
            return True
        ok = kwu.can_init(*(x for x in (channel, bitrate) if x is not None))
        if not ok:
            raise SystemExit("!! CAN 을 못 열었다: %s" % kwu.get_can_status().get("last_error"))
        return ok

    def close(self):
        try:
            self.stop()
        finally:
            kwu.configure_can_tx_observer(None)
            if not self.dry_run:
                kwu.can_close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # ── 명령 ────────────────────────────────────────────────────────
    def set(self, movement, why=""):
        """동작을 바꾼다. 화이트리스트 밖이면 거부한다."""
        if self.aborted:
            self.stop_now()
            raise Aborted("사람이 세웠다 — %s 거부" % movement)
        fn = MOVES.get(movement)
        if fn is None:
            raise SystemExit("!! 쓰지 않는 동작이다: %r (여섯 개만 쓴다)" % movement)
        self.movement = movement
        self.t_cmd = clock.now()
        fn()
        if self.aborted:                # 검사와 fn() 사이에 abort 가 끼어든 경우 — 나간 명령을 바로 되돌린다
            self.stop_now()
            raise Aborted("사람이 세웠다 — %s 취소" % movement)
        if self.rec is not None:
            self.rec.event("cmd", movement=movement, why=why, t_cmd=self.t_cmd,
                           wall=clock.wall_from(self.t_cmd))
        return self.t_cmd

    def stop(self, why="normal"):
        """**정지가 먼저, 기록이 나중.** 기록이 망가져도 차는 반드시 선다 (plan 6-6 ④)."""
        try:
            kwu.issue_command_stop()
        finally:
            self.movement = "stop"
            if self.rec is not None:
                try:
                    t = clock.now()
                    self.rec.event("cmd", movement="stop", why=why, t_cmd=t,
                                   wall=clock.wall_from(t))
                except Exception:
                    pass

    def stop_now(self):
        """**콜백 안에서만** 부른다 — CAN 만 끊고 기록은 안 한다(기록은 부른 쪽이 나중에)."""
        kwu.issue_command_stop()
        self.movement = "stop"

    def lease(self, seconds):
        """이 시간 안에 다음 명령이 없으면 CAN 스레드가 알아서 세운다 (데드맨)."""
        if self.aborted:
            return                      # 세운 뒤엔 대기 루프가 lease 를 되살리지 못한다
        kwu.configure_motion_deadline(clock.now() + float(seconds))   # kwu 도 monotonic 이다

    def lease_expired(self):
        return self.aborted or kwu.motion_deadline_expired()

    def abort(self, why="emergency"):
        """사람이 세움. **CAN 정지가 먼저**, 그 뒤 모든 set/lease 를 막는다. 어느 스레드에서 불러도 된다."""
        try:
            self.stop_now()
        finally:
            self.aborted = True
            if self.rec is not None:
                try:
                    self.rec.event("cmd", movement="stop", why=why, t_cmd=clock.now(), aborted=True)
                except Exception:
                    pass

    def set_rotate_strength(self, fine):
        """회전 강도를 고른다 — fine=True 면 약한 회전(ROTATE_FINE_JOYSTICK_DEFLECTION). **서 있을 때만.**
        프레임을 다시 써넣고 8바이트를 확인한다(frames.set_rotate_strength)."""
        if self.movement != "stop":
            raise RuntimeError("움직이는 중엔 회전 강도를 못 바꾼다 (%s)" % self.movement)
        d = frames.set_rotate_strength(C.ROTATE_FINE_JOYSTICK_DEFLECTION if fine else C.ROTATE_JOYSTICK_DEFLECTION)
        self.rotate_deflection = d
        return d

    def clear_lease(self):
        kwu.configure_motion_deadline(None)

    def arm_rotation_timeout(self, movement, hold_s):
        """우리 코드가 다 죽어도 CAN 스레드가 이 시간에 세운다. 회전의 마지막 안전망."""
        cmd = "ROT_LEFT" if movement == "rotate_left_slow" else "ROT_RIGHT"
        return kwu.arm_timed_rotation(cmd, float(hold_s))

    def status(self):
        return kwu.get_can_status()

    # ── 관측 ────────────────────────────────────────────────────────
    def _on_tx(self, **event):
        """실제로 버스에 나간 시각. 명령 지연을 여기서 잰다 (9/21 실측 0.17 ms)."""
        try:
            self.rec.can(**event)
        except Exception:
            pass


if __name__ == "__main__":
    # 자체 시험 — 하드웨어 없이(dry-run, CAN 안 보냄). 동결 테이블 · 화이트리스트 · 데드맨 · 기록의 벽시계 병기
    import time

    class Rec:
        def __init__(self):
            self.events, self.cans = [], []

        def event(self, kind, **row):
            self.events.append((kind, row))

        def can(self, **row):
            self.cans.append(row)

    rec = Rec()
    with Driver(dry_run=True, recorder=rec, log=print) as drv:
        t = drv.table
        assert t["forward"][2] == 127 - C.FORWARD_JOYSTICK_DEFLECTION == 67           # 2026-09-30 결정 ② byte 67
        assert t["backward"][2] == 127 + C.BACKWARD_JOYSTICK_DEFLECTION == 187
        assert t["rotate_left_slow"][1] == 127 + C.ROTATE_JOYSTICK_DEFLECTION           # 결정 ① 강도 30
        assert all(b == 127 for b in t["stop"]) and all(t[m][4] == 127 for m in t)      # byte4(포크)는 늘 중립
        t0 = drv.set("forward", why="시험")
        assert drv.movement == "forward" and drv.status()["movement"] == "forward"
        kind, row = rec.events[-1]
        assert kind == "cmd" and row["t_cmd"] == t0 and abs(t0 - clock.now()) < 0.5    # 우리 시계(monotonic)
        assert abs(row["wall"] - time.time()) < 1.0                                      # 기록엔 벽시계 병기
        # 데드맨: 이 안에 갱신이 없으면 CAN 쪽이 알아서 stop 으로 떨어진다
        drv.lease(0.02)
        assert not drv.lease_expired()
        time.sleep(0.03)
        assert drv.lease_expired() and drv.status()["movement"] == "stop"
        drv.clear_lease()
        assert not drv.lease_expired()
        # 화이트리스트 밖은 거부 — 포크·폴딩·리치는 이름조차 없다
        for bad in ("lift_up", "fold", "forward_left", "reach_forward"):
            try:
                drv.set(bad)
                raise AssertionError(bad)
            except SystemExit:
                pass
        assert drv.arm_rotation_timeout("rotate_left_slow", 2.0) > 0                    # 회전 안전망을 걸 수 있다
        drv.stop("시험 끝")
        assert drv.movement == "stop" and rec.events[-1][1]["why"] == "시험 끝"
        assert drv.status()["movement"] == "stop" and not drv.status()["enabled"]
    print("driver 자체 시험 통과 (dry-run)")
