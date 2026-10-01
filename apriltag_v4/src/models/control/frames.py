"""동결 테이블 — 우리가 쓸 6개 프레임을 써넣고 바이트로 확인한다.

2026-09-07 에 회전 명령이 byte4(포크 리프트)를 써서 포크가 올라간 사고가 있었다.
Dropbox 에 아직 그 버그 버전이 있다. 시작할 때마다 8바이트를 전부 확인한다 (plan 9절).
"""
from types import MappingProxyType

from config import control as C
from ..kwu import control as kwu

#: 우리가 쓰는 동작. 이 여섯 말고는 이름조차 안 적는다
SAFE = ("stop", "forward", "forward_slow", "backward",
        "rotate_left_slow", "rotate_right_slow")
NEUTRAL = 127
#: byte1 = 조향, byte2 = 전후진. 나머지는 전부 중립이어야 한다. byte4 는 포크 리프트다
FREE_BYTES = (1, 2)


def expected():
    """config 의 강도에서 6개 프레임을 만든다. 광운대 control.py 와 같은 계산."""
    n = NEUTRAL
    f = n - C.FORWARD_JOYSTICK_DEFLECTION
    fs = n - C.FORWARD_SLOW_JOYSTICK_DEFLECTION
    b = n + C.BACKWARD_JOYSTICK_DEFLECTION
    rl = n + C.ROTATE_JOYSTICK_DEFLECTION
    rr = n - C.ROTATE_JOYSTICK_DEFLECTION
    return {
        "stop":              (n, n, n, n, n, n, n, n),
        "forward":           (n, n, f, n, n, n, n, n),
        "forward_slow":      (n, n, fs, n, n, n, n, n),
        "backward":          (n, n, b, n, n, n, n, n),
        "rotate_left_slow":  (n, rl, n, n, n, n, n, n),
        "rotate_right_slow": (n, rr, n, n, n, n, n, n),
    }


def apply_and_verify(log=print):
    """config 강도를 써넣고 바이트를 확인한다. 하나라도 다르면 출발을 거부한다."""
    want = expected()
    kwu.configure_rotate_in_place(C.ROTATE_JOYSTICK_DEFLECTION)
    kwu.configure_forward_slow(C.FORWARD_SLOW_JOYSTICK_DEFLECTION)
    kwu.configure_drive_deflection(C.FORWARD_JOYSTICK_DEFLECTION,
                                   C.BACKWARD_JOYSTICK_DEFLECTION)
    # stop 은 어떤 configure_* 도 안 건드린다. 제일 중요한 프레임이라 직접 써넣는다
    kwu.MOVEMENT_TEMPLATES["stop"] = list(want["stop"])
    for name, w in want.items():
        got = kwu.MOVEMENT_TEMPLATES.get(name)
        if got is None:
            raise SystemExit("!! CAN 템플릿에 %s 가 없다 — 출발하지 않는다" % name)
        got = tuple(int(b) for b in got)
        if got != w:
            bad = [i for i in range(8) if got[i] != w[i]]
            raise SystemExit(
                "!! %s 의 byte%s 가 다르다: 기대 %s, 실제 %s — 출발하지 않는다\n"
                "   (byte4 는 이 지게차에서 포크 리프트다)" % (name, bad, list(w), list(got)))
        for i in range(8):
            if i not in FREE_BYTES and got[i] != NEUTRAL:
                raise SystemExit("!! %s 의 byte%d 가 중립이 아니다 (%d)" % (name, i, got[i]))
    if log:
        log("  CAN 동결 확인: rotate %d/%d · forward %d · slow %d · backward %d (그 외 중립)"
            % (want["rotate_left_slow"][1], want["rotate_right_slow"][1],
               want["forward"][2], want["forward_slow"][2], want["backward"][2]))
    return MappingProxyType({k: bytes(v) for k, v in want.items()})
