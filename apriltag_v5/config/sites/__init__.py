"""장소별 현장 값 — config/site.py 의 SITE 가 고른다. 파일마다 이름이 같아야 한다(config/control.py · detection.py 가 가져다 쓴다)."""
import importlib

from config.site import SITE

#: 비어(None) 있으면 run·calibrate 가 출발하지 않는 값 — 줄자로 재거나 사람이 정한다.
#: DOCK_LENGTH_M 은 2차(전체 진입) 전용이라 비어 있어도 된다
REQUIRED = ("TAG_HEIGHT_M", "CAMERA_HEIGHT_M", "COLOR_AUTO_EXPOSURE", "TAG_LATERAL_OFFSET_M", "BACK_MAX_M")
NAMES = {"kw": "광운대", "krri": "철도연"}          # 화면에 띄우는 이름 (run 이 SPACE 전에)


def load(site=None):
    return importlib.import_module("config.sites." + (site or SITE))


def missing(site=None):
    """비어 있는 필수 값 이름들. 빈 목록이면 출발해도 된다."""
    m = load(site)
    return [k for k in REQUIRED if getattr(m, k, None) is None]
