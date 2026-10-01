"""카메라 노출 설정 한 곳 — run.py · calibrate.py 가 같은 설정으로 연다.

    실내(COLOR_AUTO_EXPOSURE=False)  자동노출 끔 · 8.3 ms(60 Hz 반주기, 형광등 깜빡임을 피한다) · 게인 64  = CameraSettings.docking()
    실외(COLOR_AUTO_EXPOSURE=True)   자동노출 켬. 햇빛 아래 8.3 ms 고정은 화면이 하얗게 날아가 태그를 못 찾는다 (2026-10-02 실차)
큐 1(최신 프레임만) · 프레임률 고정 · global time 은 둘 다 같다.
"""
from dataclasses import replace

from config import detection as D
from .camera import CameraSettings


def camera_tune():
    base = CameraSettings.docking()
    if D.COLOR_AUTO_EXPOSURE:
        # 노출·게인을 None 으로 — 값을 쓰면 장치가 자동노출을 도로 끈다 (CameraSettings.apply 의 순서 규칙)
        return replace(base, enable_auto_exposure=True, exposure=None, gain=None)
    return base
