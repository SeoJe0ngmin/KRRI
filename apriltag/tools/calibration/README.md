# 카메라 캘리브레이션

출처: https://github.com/Team293/AprilTagDetection  (camera calibration/)

## 왜 필요한가
`detection_pose()` 는 fx, fy, cx, cy 로 픽셀을 미터로 환산한다.
이 값이 틀리면 **거리가 그 비율만큼 통째로 틀어진다.** 각도는 상대적으로 덜 민감하다.
검출(detect) 자체에는 관여하지 않는다.

## 순서 (실물 카메라를 산 뒤)
1. 체스보드 인쇄 (기본 설정은 14x9 격자)
2. `python generate_calibration_images.py`  — 스페이스바로 20장쯤 여러 각도에서 촬영
3. `python camera_calibration.py`           — 같은 폴더의 *.jpg 로 계산 → CameraCalibration.npz
4. 나온 값을 `src.image_source.CameraIntrinsics` 에 넣는다

`camera_calibration_gui.py` 는 GUI 버전인데 `customtkinter` 가 추가로 필요하다.

## 주의
- 세 스크립트 모두 `__main__` 가드가 없다. **import 하지 말고 실행만** 할 것.
  (import 하면 웹캠이 켜지거나 npz 를 덮어쓴다)
- `CameraCalibration_team293.npz` 는 참고용이다. 주점이 cx=312.8, cy=257.5 로
  **약 626x515 해상도 카메라** 값이므로 다른 해상도 영상에 그대로 쓰면 안 된다.
