"""RealSense 진단 - **librealsense SDK CLI 가 못 하는 것만** 남긴 최소 도구."""
import argparse
import os
import platform
import shutil
import sys
import time
from pathlib import Path

try:
    import cv2
    CV2_ERROR = None
except Exception as exc:                                          # pragma: no cover
    cv2 = None
    CV2_ERROR = exc

# 결과는 한 줄에 [ PASS ] / [ FAIL ] / [ WARN ] / [ SKIP ] 로만 찍음
TALLY = {"PASS": 0, "FAIL": 0, "WARN": 0, "SKIP": 0}


def mark(status, title, detail=""):
    """PASS/FAIL 한 줄. detail 은 같은 줄 뒤에 붙임."""
    TALLY[status] = TALLY.get(status, 0) + 1
    print("[ %-4s ] %s%s" % (status, title, (" - " + str(detail)) if detail else ""))


def info(text=""):
    """PASS/FAIL 이 아닌 부연. 집계에 안 들어감."""
    print(("         " + text) if text else "")


def section(n, title):
    print("\n" + "=" * 74)
    print(" %d) %s" % (n, title))
    print("=" * 74)


def short(exc, limit=200):
    """예외를 한 줄로. librealsense 메시지는 길어서 자름."""
    s = " ".join(("%s: %s" % (type(exc).__name__, exc)).split())
    return s if len(s) <= limit else s[:limit] + " ..."


# --- 1) pyrealsense2 : SDK CLI 가 대신 못 해주는 검사 ---
def step_import():
    section(1, "pyrealsense2 임포트 (이 인터프리터 기준)")
    try:
        import pyrealsense2 as rs
    except Exception as exc:
        mark("FAIL", "import pyrealsense2", short(exc))
        info("→ pip install pyrealsense2")
        if sys.version_info < (3, 10):
            info("→ ★ 지금 파이썬은 %d.%d 다. PyPI 리눅스 x86-64 휠이 cp310 부터라"
                 " 3.9 이하 env 는 pip 설치가 아예 안 된다." % sys.version_info[:2])
            info("   3.11 env 를 쓰거나(예: conda activate krri) 소스 빌드해야 한다.")
        return None

    ver = (getattr(rs, "__version__", None) or getattr(rs, "__full_version__", None)
           or "(버전 정보 없음)")
    mark("PASS", "import pyrealsense2", "version=%s" % ver)
    info("모듈: %s" % getattr(rs, "__file__", "?"))
    info("파이썬: %s (%s)" % (platform.python_version(), sys.executable))
    return rs


def count_devices(rs):
    """장치 '유무'만 셈. 상세는 rs-enumerate-devices 담당이라 여기선 안 찍음."""
    section(2, "장치 유무")
    try:
        n = len(list(rs.context().query_devices()))
    except Exception as exc:
        mark("FAIL", "rs.context() 조회", short(exc))
        return 0
    if not n:
        mark("FAIL", "연결된 장치", "0 개 - USB 가 안 붙었거나 권한 문제다")
        return 0
    mark("PASS", "연결된 장치", "%d 개 (상세는 rs-enumerate-devices)" % n)
    return n


# --- 2) GUI : viewer 는 GLFW/OpenGL. cv2 highgui 는 별개 스택이라 직접 열어봐야 안다 ---
def step_gui(enabled=True):
    section(3, "GUI (cv2.imshow)")
    disp, wl = os.environ.get("DISPLAY", ""), os.environ.get("WAYLAND_DISPLAY", "")
    info("DISPLAY=%s  WAYLAND_DISPLAY=%s" % (disp or "(없음)", wl or "(없음)"))
    if not enabled:
        return mark("SKIP", "창 띄우기", "--no-gui")
    if cv2 is None:
        return mark("FAIL", "cv2 임포트", short(CV2_ERROR))
    if not disp and not wl:
        mark("FAIL", "디스플레이", "DISPLAY/WAYLAND_DISPLAY 둘 다 없다 - 헤드리스")
        info("→ SSH 면 -X, WSL2 면 WSLg 가 살아있어야 한다(/mnt/wslg 확인).")
        info("→ 화면 없이 쓸 거면 cv2.imwrite 로 파일 저장하는 경로를 써라.")
        return

    # 환경변수가 있다고 창이 뜬다는 보장은 없음. 실제로 열어봐야 앎.
    try:
        import numpy as np
        win = "realsense_check"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 160, 120)
        cv2.imshow(win, np.zeros((120, 160, 3), dtype="uint8"))
        cv2.waitKey(120)
        cv2.destroyWindow(win)
        cv2.waitKey(1)
        mark("PASS", "창 띄우기", "cv2 %s - imshow 열고 닫기 성공" % cv2.__version__)
    except Exception as exc:
        mark("FAIL", "창 띄우기", short(exc))
        info("→ opencv-python-headless 가 깔려 있으면 imshow 자체가 없다. "
             "pip install opencv-python 으로 교체.")
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


# --- 4) 프레임 메타데이터 : 이건 SDK CLI 로 못 봄. 커널 패치 유무가 여기서 갈린다 ---
def step_frame_metadata(rs):
    """프레임별 노출/게인/센서시각이 **실제로** 오는지 확인함."""
    section(4, "프레임 메타데이터 / 타임스탬프")
    try:
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        prof = pipe.start(cfg)
    except Exception as exc:
        return mark("WARN", "컬러 스트림 열기", short(exc))
    try:
        for _ in range(5):                      # 자동노출이 자리잡을 시간
            fs = pipe.wait_for_frames()
        f = fs.get_color_frame()

        dom = str(f.get_frame_timestamp_domain())
        if "hardware" in dom or "global" in dom:
            mark("PASS", "타임스탬프 도메인", "%s - 센서 시계가 살아 있다" % dom)
        else:
            mark("WARN", "타임스탬프 도메인", "%s - 호스트 도착시각이다" % dom)
            info("→ 이 타임스탬프에는 USB 지연과 우리 루프 대기가 섞여 있다.")
            info("→ 속도/움직임 추정에 그대로 쓰지 마라.")

        names = [a for a in dir(rs.frame_metadata_value)
                 if not a.startswith("_") and a not in ("name", "value")]
        got = []
        for n in names:
            mv = getattr(rs.frame_metadata_value, n)
            try:
                if f.supports_frame_metadata(mv):
                    got.append((n, f.get_frame_metadata(mv)))
            except Exception:
                pass
        want = {"actual_exposure", "gain_level", "sensor_timestamp", "frame_counter"}
        have = want & {n for n, _ in got}
        if have == want:
            mark("PASS", "프레임별 노출/게인", "%d 항목 - 실패 프레임 로그를 남길 수 있다"
                 % len(got))
        elif got:
            mark("WARN", "프레임별 노출/게인", "%d 항목만 온다 (없는 것: %s)"
                 % (len(got), ", ".join(sorted(want - have)) or "-"))
        else:
            mark("WARN", "프레임별 노출/게인", "하나도 안 온다")
        if have != want:
            info("→ 리눅스면 librealsense 커널 패치(patch-realsense-dkms.sh)가 필요하다.")
            info("→ WSL2 의 기본 uvcvideo 로는 안 된다. time_of_arrival 만 항상 온다.")
        for n, v in got[:12]:
            info("   %-26s %s" % (n, v))

        # 컬러 AE ROI - 역광 도크에서 검출률을 가르는 물건이라 따로 확인함
        roi_ok = False
        for sen in prof.get_device().query_sensors():
            if not sen.get_info(rs.camera_info.name).lower().startswith("rgb"):
                continue
            roi_ok = sen.is_roi_sensor()
        if roi_ok:
            mark("PASS", "컬러 자동노출 ROI", "set_region_of_interest 사용 가능")
        else:
            mark("WARN", "컬러 자동노출 ROI", "지원 안 함 (FW 5.10.9 미만이거나 D405)")

        # --- 노출 눈금 대조 : 컬러는 100us, 뎁스는 us 라 100배가 다르다 ---
        try:
            csen = None
            for sen in prof.get_device().query_sensors():
                if sen.get_info(rs.camera_info.name).lower().startswith("rgb"):
                    csen = sen
            mv = getattr(rs.frame_metadata_value, "actual_exposure", None)
            if csen is not None and mv is not None and f.supports_frame_metadata(mv) \
                    and csen.supports(rs.option.exposure):
                md = int(f.get_frame_metadata(mv))
                opt = float(csen.get_option(rs.option.exposure))
                if abs(md - opt) <= max(1.0, opt * 0.02):
                    mark("PASS", "노출 눈금 대조",
                         "metadata=%d, option=%.0f 일치 -> 컬러는 100us 눈금(=%.1fms)"
                         % (md, opt, md * 100.0 / 1000.0))
                else:
                    mark("WARN", "노출 눈금 대조",
                         "metadata=%d vs option=%.0f 불일치 - 단위 가정을 다시 봐라" % (md, opt))
                    info("→ src/utils/camera.py COLOR_EXPOSURE_UNIT_US 와 Frame.exposure_us 가 틀어진다.")
        except Exception:
            pass

        # --- 프레임 드롭 : SDK 는 이걸 알려주지 않는다 ---
        try:
            sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # 코드 폴더 (src 가 있는 곳)
            from src.utils.camera import FrameStats
            st = FrameStats()
            t0 = None
            for _ in range(60):
                fr = pipe.wait_for_frames().get_color_frame()
                if not fr:
                    continue
                ts = fr.get_timestamp() / 1000.0
                if t0 is None:
                    t0 = ts
                st.update(fr.get_frame_number(), ts - t0)
            if st.dropped == 0:
                mark("PASS", "프레임 드롭", st.summary())
            else:
                mark("WARN", "프레임 드롭", st.summary())
                info("→ 우리 소비 루프가 늦어 SDK 가 버린 것이다(카메라 성능 문제가 아니다).")
            if st.fps and st.fps < 25 and st.dropped == 0:
                info("→ 버린 것 없이 fps 가 낮다 = 카메라가 못 내주고 있다.")
                info("→ 어두우면 AE 가 fps 를 떨군다. auto_exposure_priority=0 으로 막는다.")
        except Exception as exc:
            mark("WARN", "프레임 드롭", short(exc))
    except Exception as exc:
        mark("WARN", "메타데이터 읽기", short(exc))
    finally:
        try:
            pipe.stop()
        except Exception:
            pass


# --- 3) 장치가 없을 때의 안내 : librealsense 는 "No device detected" 한 줄이 전부다 ---
def step_no_device_help():
    section(4, "장치가 없다 - 다음에 할 일")
    try:
        is_wsl = "microsoft" in Path("/proc/version").read_text().lower()
    except Exception:
        is_wsl = False
    videos = sorted(str(p) for p in Path("/dev").glob("video*"))
    info("커널         : %s" % platform.release())
    info("WSL2 여부    : %s" % ("예" if is_wsl else "아니오"))
    info("lsusb        : %s" % (shutil.which("lsusb") or "미설치 (apt install usbutils)"))
    info("/dev/bus/usb : %s" % ("있음" if Path("/dev/bus/usb").exists()
                                else "없음 → USB 가 WSL 로 전달된 적 없음"))
    info("/dev/video*  : %s" % (", ".join(videos) if videos else "없음"))
    if not is_wsl:
        info("")
        info("일반 리눅스라면: USB3 직결인지, lsusb 에 8086:0b3a 가 뜨는지,")
        info("퍼미션(udev 규칙)이 적용됐는지를 순서대로 확인해라.")
        return

    print("""
--- WSL2 에서 D435i 를 붙이는 절차 -------------------------------------------
[전제] 이 커널(6.6.x)엔 uvcvideo/videodev/vhci-hcd 가 이미 있다. "WSL 커스텀 커널
  빌드"는 필요 없고, USB 를 Windows 에서 넘겨주는 usbipd 가 유일한 관문이다.

[Windows/PowerShell]
  1. winget install --interactive --exact dorssel.usbipd-win
     (--interactive 없으면 드라이버 설치로 경고 없이 재부팅될 수 있다. 설치 후 창 새로)
  2. D435i 를 USB3(파란색/SS) 포트에 **허브 없이 직결**. 케이블도 USB3 데이터용.
  3. WSL 터미널을 하나 열어둔다 (경량 VM 이 살아있어야 attach 가 붙는다).
  4. usbipd list  → VID:PID 8086:0b3a 인 줄 (D435=0b07, D455=0b5c).
                    Depth/RGB/HID 3기능이 한 BUSID 로 묶여 나오는 게 정상.
  5. usbipd bind --busid <BUSID>   ← 관리자 필수. 거부되면 --force (드라이버 점유).
                                     STATE 가 Shared 로 바뀌어야 한다.
  6. usbipd attach --wsl --busid <BUSID>   ← 관리자 불필요, --auto-attach 로 자동 재연결
  7. 다 쓰면 usbipd detach --busid <BUSID>
     ※ attach 중엔 Windows 쪽 realsense-viewer 가 카메라를 못 본다(배타적).
     ※ attach 는 비영구적. 재부팅 / wsl --shutdown / 재연결 때마다 6 을 다시.

[WSL 쪽]
  sudo apt install -y usbutils && lsusb        → 8086:0b3a 가 보여야 한다
  sudo modprobe uvcvideo && ls -l /dev/video*  → video0~5 가 생긴다
  sudo chmod a+rw /dev/video* ; sudo chmod -R a+rw /dev/bus/usb
     (WSL2 는 systemd/udev 가 없어 setup_udev_rules.sh 가 무효.
      /etc/wsl.conf 에 [boot] systemd=true 가 근본 해결.)
  pip install pyrealsense2  → 반드시 python 3.10+ env (휠이 cp310~cp314 만 있다)
  rs-hello-realsense        → 파이썬 없이 붙었는지만 확인

[그래도 안 되면] RSUSB 백엔드 소스 빌드가 WSL2 에서 가장 견고하다. UVC/HID 를
  유저스페이스(libusb)로 처리해 /dev/video* 도 커널 HID-sensor 도 필요 없어진다.
  이 커널은 CONFIG_HID_SENSOR_HUB 가 꺼져 있어 **IMU 는 RSUSB 빌드로만 나온다.**
  git clone https://github.com/IntelRealSense/librealsense && cd librealsense
  mkdir build && cd build && cmake .. -DFORCE_RSUSB_BACKEND=true \\
      -DBUILD_PYTHON_BINDINGS=true -DBUILD_SHARED_LIBS=false \\
      -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=$(which python)
  make -j$(nproc) && sudo make install
  ※ scripts/patch-realsense-ubuntu-lts-hwe.sh (커널 패치)는 WSL2 에서 실행 금지.

[증상별 대처]
  lsusb 에 아무것도 없음            → usbutils 미설치, 또는 attach 안 됨(STATE 확인)
  lsusb 엔 보이나 /dev/video* 없음  → sudo modprobe uvcvideo, dmesg | tail -50
  lsusb 보이나 No device detected   → chmod -R a+rw /dev/bus/usb, 안 되면 RSUSB 빌드
  USB2 인식 / 프레임 드랍           → usbip 는 USB over TCP 라 대역폭·지터가 나쁘다.
      848x480@30 이하로 낮추고 실제 전송률은 rs-data-collect 로 재라 (체감보다 낮다).

[먼저] Windows 용 RealSense SDK 2.0 의 realsense-viewer 로 카메라 자체가 멀쩡한지
  확인해라. 거기서 실패하면 WSL 얘기는 무의미하다. 목적이 "실시간으로 보고 싶다"뿐
  이면 네이티브 Windows 파이썬이 낫다. WSL2 는 리눅스 전용 파이프라인 코드에 카메라를
  물려야 할 때만 택할 이유가 있다.
------------------------------------------------------------------------------""")


def main():
    ap = argparse.ArgumentParser(
        description="RealSense 진단 - SDK CLI 가 못 하는 것만 (절대 죽지 않는다)")
    ap.add_argument("--no-gui", dest="gui", action="store_false", help="GUI 검사 생략")
    ap.add_argument("--force-mac", action="store_true",
                    help="macOS 에서도 SDK 로 장치를 열어 본다 (2.56.5 는 여기서 segfault 난다)")
    args = ap.parse_args()

    print("=" * 74)
    print(" RealSense 진단  |  %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    print(" host=%s  kernel=%s" % (platform.node(), platform.release()))
    print(" 장치정보/프로파일/내부파라미터 -> rs-enumerate-devices (내부파라미터는 -c)")
    print(" 실측 fps -> rs-data-collect,   라이브 뷰/펌웨어 -> realsense-viewer")
    print("=" * 74)

    rs = step_import()
    if rs is not None and sys.platform == "darwin" and not args.force_mac:
        # macOS 12+ 는 시스템 UVCAssistant 가 UVC 인터페이스를 선점한다. SDK 는
        # 'failed to set power state' 로 못 잡고, sudo 로 뺏어도 2.56.5 는 IMU(HID)
        # 초기화에서 segfault 난다(librealsense #14302, 2026-09-06 맥북 실측). 장치를
        # 만드는 순간 이 도구도 같이 죽으므로 건드리지 않는다.
        section(2, "장치 유무")
        mark("SKIP", "장치 조회", "macOS 에서는 SDK 로 카메라를 못 연다 (--force-mac 으로 강행)")
        info("→ UVCAssistant 가 카메라를 선점해 librealsense 가 못 잡고, sudo 로도")
        info("   2.56.5 는 IMU 초기화에서 segfault 난다 (librealsense #14302).")
        info("→ 컬러만 보려면  python tools/live_pose.py --source webcam   (sudo 불필요)")
        info("→ depth / IR / IMU / bag 녹화는 Jetson 이나 Windows 에서.")
        ndev = None                                   # 모름 — 세지 않았다
    elif rs is not None:
        ndev = count_devices(rs)
    else:
        section(2, "장치 유무")
        mark("SKIP", "장치 조회", "pyrealsense2 없음 (rs-enumerate-devices 로는 확인 가능)")
        ndev = 0

    step_gui(enabled=args.gui)
    if ndev == 0:
        step_no_device_help()
    elif rs is not None and ndev:
        step_frame_metadata(rs)

    print("\n" + "=" * 74)
    print(" 결과: PASS %d  FAIL %d  WARN %d  SKIP %d"
          % (TALLY["PASS"], TALLY["FAIL"], TALLY["WARN"], TALLY["SKIP"]))
    if ndev == 0:
        print(" 카메라를 못 찾았다. 위 4번 절차를 순서대로 밟아라.")
    elif TALLY["FAIL"]:
        print(" 파이썬 쪽 실패 단계가 있다. FAIL 줄의 '→' 안내를 따라라.")
    elif ndev is None:
        print(" macOS: 파이썬 바인딩·GUI 정상. 카메라는 live_pose.py --source webcam 으로 봐라.")
    else:
        print(" 파이썬 바인딩·GUI 정상. 카메라 상세는 rs-enumerate-devices 로 봐라.")
    print("=" * 74)
    # 진단 도구는 종료코드로도 실패를 알림 (스크립트에서 물려 쓰기 좋게).
    return 1 if (ndev == 0 or TALLY["FAIL"]) else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n중단됨")
        sys.exit(130)
    except Exception as exc:                       # 여기까지 오면 안 되지만, 최후의 그물
        print("\n[ FAIL ] 진단 도구 자체가 예외로 죽었다: %s" % short(exc))
        sys.exit(2)
