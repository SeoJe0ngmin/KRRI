# -*- coding: utf-8 -*-
"""실카메라에 tune_for_tags() 를 걸고 **전/후 옵션 값을 찍는다.**

이 파일이 있는 이유: camera_control.py 를 만들 때 D435i 가 물리적으로 빠져 있어서
(usbipd 에 "Persisted" 만 있고 "Connected" 에 8086:0b3a 가 없었다) 쓰기 경로를
실장비로 확인하지 못했다. 케이블을 꽂고 이거 하나만 돌리면 그 구멍이 메워진다.

    bash src/etc/wsl_attach_camera.sh
    /home/jeongmin/anaconda3/envs/krri/bin/python src/etc/camera_tune_check.py

확인해야 할 것 네 가지 (출력에 그대로 표시된다):
  1. exposure 범위가 1..10000 인가  -> 한 눈금이 100us 라는 가정이 맞다는 뜻
  2. AE ROI 가 실제로 걸리는가       -> 소스상 되지만 실장비 확인은 이게 처음
  3. auto_exposure_limit 이 컬러에 없는가 -> 없어야 정상(설계 근거)
  4. 노출을 83 으로 박은 뒤 되읽었을 때 83 인가 -> 순서 함정을 이겼다는 뜻

끝나면 **원래 설정으로 되돌린다.** UVC 값은 스트림을 닫아도 장치에 남아서,
안 되돌리면 다음에 realsense-viewer 를 열었을 때 8.3ms 로 어둡게 나온다.
"""
import sys, os
# 이 파일은 <repo>/src/etc/ 에 있다. `src.` 를 임포트하려면 그 위위위인
# apriltag/ 가 sys.path 에 있어야 한다 — dirname 세 번. (tools/ 시절엔 두 번이었다.)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import pyrealsense2 as rs
from src.utils import camera_control as cc
from src.utils.camera_control import CameraSettings, tune_for_tags
from src.models.tag_pose import open_realsense, to_gray, make_detector, detect


def main():
    if len(rs.context().query_devices()) == 0:
        print("장치 없음. 케이블을 꽂고 'bash src/etc/wsl_attach_camera.sh' 를 먼저 돌릴 것.")
        return 1

    # tune=False 로 먼저 연다 — 공장 상태를 그대로 보기 위해서다.
    frames, intr = open_realsense(stream="color", tune=False)
    profile = frames.profile
    sensor = cc.color_sensor(profile)
    print("장치:", profile.get_device().get_info(rs.camera_info.name),
          " FW", profile.get_device().get_info(rs.camera_info.firmware_version))
    print("스트림: %dx%d  fx=%.1f" % (intr.width or 0, intr.height or 0, intr.fx))

    print("\n[1] 컬러 센서 전체 옵션 (BEFORE) — exposure 범위가 1..10000 인지 볼 것")
    print(cc.describe_color_options(sensor))

    print("\n[2] auto_exposure_limit 지원 여부 — color 가 False 여야 설계가 맞다")
    print("   ", cc.ae_limit_supported(profile))

    print("\n[3] AE ROI 지원 여부 (실장비 확인은 이게 처음)")
    print("    is_roi_sensor:", cc.ae_roi_supported(profile), "  현재 ROI:", cc.ae_roi_of(profile))

    print("\n[4] tune_for_tags() 적용 — 전/후")
    before, report = tune_for_tags(profile, verbose=True)

    print("\n[5] 되읽기 검증 — exposure 가 83(8.3ms) 으로 남아 있어야 한다")
    got = CameraSettings.from_sensor(sensor)
    print(got.describe())
    ok = (got.exposure == 83)
    print("    -> " + ("OK" if ok else
          "!! %s 다. AE 끄기 뒤 노출 재기록 순서를 의심할 것" % got.exposure))

    print("\n[6] 태그를 한 번 찾아 AE ROI 를 거기에 걸어 본다 (2패스)")
    det = make_detector()
    hit = None
    for i, t, img in frames:
        r = detect(det, to_gray(img))
        if r:
            hit = r[0]
            break
        if i > 90:
            break
    if hit is None:
        print("    태그를 못 찾아 ROI 시험은 건너뛴다 (태그를 카메라 앞에 두고 다시 실행)")
    else:
        c = hit.corners
        bbox = (c[:, 0].min(), c[:, 1].min(), c[:, 0].max(), c[:, 1].max())
        print("    tag bbox =", tuple(round(float(v)) for v in bbox))
        print("    ROI 걸기 ->", cc.aim_ae_at_bbox(profile, bbox, img.shape))
        print("    되읽은 ROI =", cc.ae_roi_of(profile), " (걸린 값이 그대로 나오면 성공)")
        print("    가운데 3/4 로 복귀 ->", cc.center_ae_roi(profile, img.shape[1], img.shape[0]))
        print("    되읽은 ROI =", cc.ae_roi_of(profile))

    print("\n[7] 원상복구")
    rep = before.apply(profile)
    for k, (b, a) in rep.items():
        print("    %-26s %-10s -> %s" % (k, b, a))
    frames.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
