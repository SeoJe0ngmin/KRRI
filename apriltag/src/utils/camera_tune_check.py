# -*- coding: utf-8 -*-
"""실카메라에 tune_for_tags() 를 걸고 **전/후 옵션 값을 찍는다.**"""
import sys, os
# 이 파일은 <repo>/src.utils/ 에 있다. `src.` 를 임포트하려면 그 위위위인
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import pyrealsense2 as rs
from src.utils import camera as cc
from src.utils.camera import CameraSettings, tune_for_tags
from src.models import open_realsense, to_gray, make_detector, detect


def main():
    if len(rs.context().query_devices()) == 0:
        print("장치 없음. 케이블을 꽂고 'bash tools/wsl_attach_camera.sh' 를 먼저 돌릴 것.")
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
