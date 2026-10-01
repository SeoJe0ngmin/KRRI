"""모든 모듈을 import 하고 모든 tool 을 --help 로 돌려본다. 커밋 전 몇 초.

2026-09-21 에 파일을 옮겼더니 import 가 깨졌다 — 그걸 즉시 잡으려는 것이다.
"""
import importlib
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from src.bootstrap import ROOT, setup  # noqa: E402

setup()
SKIP = {"__init__", "check_imports"}
#: 이 머신에 없을 수 있는 바깥 패키지. 이것 때문에 난 실패는 **우리 경로 문제가 아니다**
THIRD_PARTY = {"cv2", "numpy", "pyrealsense2", "canlib", "pupil_apriltags", "PIL", "serial"}


def modules():
    for path in sorted(list((ROOT / "src").rglob("*.py")) + list((ROOT / "config").rglob("*.py"))):
        if path.stem in SKIP:
            continue
        yield ".".join(path.relative_to(ROOT).with_suffix("").parts)


def tools():
    for path in sorted((ROOT / "tools").rglob("*.py")):
        if path.stem not in SKIP:
            yield path


def main():
    bad, skipped = [], []
    for name in modules():
        try:
            importlib.import_module(name)
            print("  ok   %s" % name)
        except ModuleNotFoundError as e:
            if (e.name or "").split(".")[0] in THIRD_PARTY:
                print("  --   %s (이 머신에 %s 없음)" % (name, e.name))
                skipped.append(name)
            else:
                print("  FAIL %s — %s" % (name, e))
                bad.append(name)
        except Exception as e:
            print("  FAIL %s — %s: %s" % (name, type(e).__name__, e))
            bad.append(name)
    for path in tools():
        rel = path.relative_to(ROOT)
        r = subprocess.run([sys.executable, str(path), "--help"],
                           capture_output=True, text=True, timeout=60, cwd=ROOT)
        err = (r.stderr or "")
        if r.returncode == 0:
            print("  ok   %s --help" % rel)
        elif "ModuleNotFoundError" in err and any(t in err for t in THIRD_PARTY):
            print("  --   %s --help (바깥 패키지 없음)" % rel)
            skipped.append(str(rel))
        else:
            print("  FAIL %s --help — %s" % (rel, (r.stderr or "").strip().splitlines()[-1:]))
            bad.append(str(rel))
    tail = "" if not skipped else "  (환경 없어 건너뜀 %d)" % len(skipped)
    print("\n%s%s" % ("전부 통과" if not bad else "실패 %d: %s" % (len(bad), bad), tail))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
