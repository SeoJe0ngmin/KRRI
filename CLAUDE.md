# KRRI AprilTag 지게차 도킹 — 작업 규칙 (Claude 가 매 세션 읽는다)

카메라로 AprilTag 을 보고 탑재부 기준 지게차 위치를 낸 뒤 그 값으로 도킹한다.

**이 문서는 이 컴퓨터(연구실 WSL, `jm` 브랜치) 기준이다.**
맥북+VM 환경 규칙은 `jm_mac` 브랜치의 CLAUDE.md 에 따로 있다 — 서로 다른 내용이니
브랜치 merge 때 한쪽이 다른 쪽을 덮어쓰지 않게 주의(각 브랜치가 자기 것을 유지).

## 이 컴퓨터 (WSL)
- WSL2 라 **카메라·IMU·CAN 실장비를 직접 못 쓴다**(USB 통과가 커널에서 막힘).
  여기서 하는 일: 코드 개발 · 시뮬레이션 · 로그 분석 · `--dry-run`.
  실카메라/실주행은 그램(Windows) 또는 Jetson 에서.
- conda env **`krri`**(python 3.11). env 작업은 반드시 **전체 경로 바이너리**로:
  `/home/jeongmin/anaconda3/envs/krri/bin/python -m pip ...`
  (bare `pip`/`python` 은 PATH 때문에 다른 env 에 깔린다.)
- **git push 는 사용자가 명시적으로 요청할 때만.** 자동 push 절대 금지.
- private repo (`origin`), 이 컴퓨터의 작업 브랜치 = `jm`.

## 브랜치 모델
- **`jm`** : 이 컴퓨터(WSL)의 개발 브랜치.
- **`jm_mac`** : 맥북+VM 의 개발 브랜치. 실차 실험도 노트북(그램) 들고 가서 jm_mac 으로 한다.
- 둘 다 살아 있는 개발 브랜치다. 한쪽에서 좋은 알고리즘이 나오면 **사용자 판단으로** 다른 쪽에 반영(merge).
  자동으로 당기거나 합치지 않는다.
- **`main`** : 배포용 스냅샷(GitHub 기본). 사용자가 "main 반영해줘" 할 때만 건드린다.

## 저장소 규칙
- **활성 개발 = `apriltag_v2/`**. `apriltag_v1/` 은 **동결 — 수정 금지**(사이드스텝 기반, 참고용).
  v2 는 lateral 을 직접 안 잡는 조준-전진(lateral_no) 규칙을 개발하는 곳.
- **config import 스타일**: control·detection 모두 `from config import control as C` / `... detection as D`,
  본문은 `C.X`/`D.X`. config 상수 원산은 `config/detection.py`.
- **tools 구조(v2)**: 주력 `tools/run.py`·`tools/analyze_run.py` 는 최상위,
  `tools/check/`(점검), `tools/etc/`(측정·테스트·설치).
- **requirements**: 루트 `requirements.txt` 마커본 하나(패키지·OS 마커만).
  WSL·그램(x86)은 PyPI 휠, 우분투/Jetson(arm64)은 pyrealsense2 를 마커로 빼고 소스 빌드, macOS 는 macosx.

## 이 컴퓨터에서 돌리는 것 (WSL, 실장비 없이)
```bash
P=/home/jeongmin/anaconda3/envs/krri/bin/python
cd apriltag_v2 && $P tools/run.py --dry-run     # CAN 안 보냄, 판단 순서만 확인
cd apriltag_v2 && $P tools/analyze_run.py       # 주행 로그 분석 (파라미터 권장값)
cd apriltag_v2 && $P tools/etc/smoke_dock.py    # 하드웨어 없이 도킹 루프 계약 검사
```
실카메라가 필요한 것(check/realsense_check, check/device_check, live_pose --source realsense)은
여기선 안 된다 — 그램/Jetson/VM 에서.

## CAN 제어 (control_forklift_v2, 실차 확인 2026-09-07) — 안전 필수
- 주행 프레임 0x1E3(중립 127): **byte2=전/후진**(전진 67, 후진 187),
  **byte1=조향/제자리회전**(좌 187, 우 67; 제자리 회전은 byte1 ±20 = 147/107).
  **byte4 는 포크 리프트** — 회전에 쓰면 포크가 올라간다(실차 사고, 절대 주의).
- `tools/run.py` 는 출발 전 CAN 템플릿을 검사한다(다섯 동작이 byte1/2 만 쓰고 나머지 중립이 아니면 출발 거부).
- 실측: 회전 팔 A=1.46m(카메라→회전중심), 직진 정속 ~0.30m/s, 명령 지연 ~1s,
  정지 관성 ~0.12m, 회전 관성 ~1.5도.
