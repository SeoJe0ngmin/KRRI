# AprilTag 지게차 도킹

카메라로 AprilTag 을 보고 **지게차가 탑재부 기준 어디에 어떻게 서 있는지**를 낸다.
그 값을 주행 명령으로 바꾸는 것이 다음 단계다.

```
src/models/tag_pose.py    카메라 → 태그 → lateral / forward / heading      (완료)
src/models/control.py     그 값 → 회전 몇 도, 몇 m 전진                     (빈 파일)
```

---

## 폴더

```
src/models/    image.py       1. 이미지 얻기 — 카메라/bag/영상 -> 프레임 + 내부파라미터
               detection_tag.py    2. 태그 찾기 — 흑백 이미지 -> 검출 결과
               detection_pose.py   3. 자세 구하기 — 검출 -> lateral/forward/heading + 파이프라인
               control.py     (빈 파일) 도킹값 -> 주행 명령
               fwd_time_model.py   다른 팀 모델. 거리 -> 명령 시간
src/utils/     camera.py      카메라 읽기·쓰기
               tag_layout.py  태그 여러 개를 한 좌표계로
               drawing.py     화면에 그리기 (표시 전용)
               util.py        블로그 원본 그대로 둔다. 5개만 씀
               realsense_check.py / make_tag_pdf.py   실행 스크립트
tools/         live_pose.py   실시간 화면          <- 실사용
               verify.py      실측 검증 (줄자)
               sim.py         가상 검증 — SCENARIO 하나로 굴린다 (--live 로 3D)
               sim_measure.py / sim_engine.py   그 엔진
               run.py         (빈 파일) 통합 실행
               viewer.sh      realsense-viewer 실행기 (rsview 로 링크됨)
               wsl_attach_camera.sh
legacy/        viewer_filter/ realsense-viewer 용 C++ 확장. 실사용 경로 아님
               옛 노트북, 블로그 원본 유틸, 캘리브레이션 스크립트
librealsense/  SDK 소스 (555MB, .gitignore)
```

의존 방향은 한 줄이다. 순환이 없다.

```
image.py  ->  detection_tag.py  ->  detection_pose.py
```


---

## 자주 쓰는 명령

```bash
./tools/wsl_attach_camera.sh                          # 재부팅·재연결 후 카메라 붙이기
rsview                                                # realsense-viewer (카메라 자동 연결)
python tools/live_pose.py --source realsense          # 실시간 화면
python tools/verify.py --frames 200 --truth-z 2.00    # 줄자 대조
python tools/sim.py --live                            # 3D + 슬라이더로 실시간
python tools/sim.py distance 1 2 3 5 --plot           # 한 값만 바꿔가며
python src/utils/make_tag_pdf.py --id 1 --size 200 --paper A3
```

---

# 알게 된 사실

아래는 전부 **실측이나 SDK 소스 확인**으로 얻은 것이다. 코드의 상수들이 왜 그 값인지가 여기 있다.

## 1. 카메라 (D435i)

### 공장 내부파라미터 — 해상도마다 다르다

| 해상도 | fx | fy | cx | cy |
|---|---|---|---|---|
| 1920×1080 | 1359.2 | 1359.0 | 956.9 | 571.3 |
| 1280×720 | 906.1 | 906.0 | 637.9 | 380.9 |
| 640×480 | 604.1 | 604.0 | 318.6 | 253.9 |
| IR 640×480 | 389.6 | 389.6 | 324.9 | 232.9 |

같은 렌즈라도 해상도에 비례한다. `CameraIntrinsics.from_realsense()` 가 스트림에서
직접 읽으므로 해상도를 바꿔도 자동으로 맞는다.

초기에 HFOV 60° 로 가정해 `fx≈1662` 를 썼는데 실제는 1359.2 였다 — **22% 차이**.
거리가 그만큼 틀렸다.

### 왜곡계수가 전부 0

RealSense 가 이미 보정해서 준다. `undistort()` 를 파이프라인에 연결할 필요가 없다.

### 노출

- 단위는 **100 µs**. `exposure=83` 이 8.3 ms. 범위 1..10000 으로 확인.
- **`auto_exposure_limit` 은 color 센서에 없다.** depth 전용이고, 롤링셔터는 제외돼 있다
  (`d400-device.cpp:1062` — "ae / gain limit feature is not supported on rolling-shutter").
  → 블러를 막으려면 **수동 고정뿐**이다.
- 검출은 밝기에 매우 둔감하다. **약 11스톱** 범위에서 100% 검출.
  1/16 배로 어둡게 해도, 32배로 96% 픽셀이 포화돼도 검출된다.
  → **어두운 건 문제가 아니다. 노출이 길어져서 생기는 블러가 문제다.**

### AE ROI (자동노출 영역 지정)

카메라 펌웨어 기능(`SETRGBAEROI = 0x75`). FW 5.10.9 이상. 우리는 5.15.1.55.

실장비로 확인한 제약 두 가지:

1. **자동노출이 켜져 있어야 한다.** 꺼두고 걸면 `hwmon command 0x75 failed`.
2. **스트림이 안정된 뒤라야 한다.** 시작 직후에 걸면 전부 거부, 30프레임쯤 받은 뒤엔 성공.

→ **노출 고정과 AE ROI 는 동시에 못 쓴다.** 정지 상태에서 ROI 로 적정 노출을 찾고,
그 값을 고정한 뒤 주행하는 2단계로 써야 한다.

### 픽셀 포맷 — bgr8 vs yuyv

`bgr8` 을 요청하면 SDK 가 BT.601 limited-range 로 변환하는데, 이때
**Y≤16 은 전부 0, Y≥235 는 전부 255** 가 된다. 256단계 중 38단계(15%)가 사라진다.
하필 그 구간이 태그의 검은 칸과 흰 칸이다. 뭉개진 픽셀은 기울기가 0 이라
`refine_edges` 가 모서리를 정밀화할 재료가 없다.

`color_format="yuyv"` 로 받으면 센서 원본 휘도를 그대로 쓴다. 비용 약 0.6 ms/프레임.
(현재 기본값은 `bgr8`. 실카메라 A/B 미측정.)

### 프레임 큐

파이프라인 출력 큐는 **용량 1** 이고 넘치면 **오래된 것을 버린다**.
`wait_for_frames` 는 절대 밀리지 않는 대신 조용히 드랍한다.
`frame_number` 의 구멍으로만 알 수 있다 → `FrameStats`.

### 메타데이터 (WSL2)

프레임별 실제 노출·게인은 UVC 벤더 페이로드로 오는데, 리눅스에서는
`V4L2_META_FMT_D4XX` 메타 노드가 필요하고 그건 librealsense 커널 패치가 만든다.
**WSL2 기본 uvcvideo 에는 없다.** 없으면 SDK 가 시스템 시간으로 조용히 대체한다.
→ `global_time_enabled` 도 무의미하고, 타임스탬프를 미분해 속도를 내면 안 된다.

---

## 2. 모션 블러 — 실패의 진짜 원인

```
blur_px = fx · v · t / z          (핀홀 기하 그대로)
```

측정된 절벽 (1920×1080, 태그 238px = 10칸이므로 1칸 23.8px):

| 블러 | 검출률 |
|---|---|
| 0~10 px | 100% |
| 12 px | 96.4% |
| 20 px | 89.3% |
| 28 px | 50.0% |
| 32 px | **0%** |

블러가 **1칸의 0.42배까지는 멀쩡하고 1.34배에서 완전히 무너진다.**

실제 지게차 속도 0.284 m/s (다른 팀 `fwd_time_model` 의 vmax) 기준:

| 거리 | 자동노출 33.3 ms | 고정 8.3 ms |
|---|---|---|
| 3.0 m | 4.3 px (100%) | 1.1 px (100%) |
| 2.0 m | 6.4 px (100%) | 1.6 px (100%) |
| 1.0 m | 12.9 px (87%) | 3.2 px (100%) |
| 0.6 m | 21.4 px (48%) | 5.3 px (100%) |

**가까울수록 나빠진다** — 태그가 크게 보이니 화면상 이동량이 커진다.
하필 도킹 마지막 구간이다. **노출을 8.3 ms 로 고정하면 전 구간 100%.**

8.3 ms 는 60 Hz 반주기이기도 해서 형광등 플리커에도 유리하다.

---

## 3. 검출 라이브러리 — AT2 에서 AT3 로

| | AT2 (`apriltag` 0.0.16) | AT3 (`pupil_apriltags`) |
|---|---|---|
| lateral 오차 평균 | 333.0 mm | **60.5 mm** |
| lateral 오차 최대 | **3069.2 mm** | 235.2 mm |
| heading 오차 평균 | 3.81° | **1.07°** |
| 노이즈 σ=5 | **세그폴트** | 정상 |
| 노이즈 σ=70 | 세그폴트 | 정상 |

**AT2 는 가끔 자세를 통째로 뒤집는다.** 5 m / heading 30° 배치에서 lateral 을
−1.5 m 대신 +1.6 m 로, heading 을 30° 대신 −4° 로 보고했다. 평면 자세 모호성에서
잘못된 해를 고른 것이다. 주행 중 이러면 지게차가 반대로 움직인다.

그리고 노이즈가 조금만 있어도 `contour_detect` 에서 **세그폴트로 프로세스가 죽는다.**
파이썬 `try/except` 로 못 막는다 — 주행 중이면 도킹이 통째로 멈춘다.

### 함정 — 모서리 순서가 반대다

```
AT2   반시계
AT3   시계
```

그대로 두면 자세가 뒤집힌다. `detect()` 가 AT3 결과를 뒤집어 AT2 순서로 맞춘다.
그래서 `_object_points()` 와 `pose_by_pnp()` 는 예전 그대로 동작한다.

### decision_margin 은 거리를 못 잡는다

태그가 238 px → 19.5 px 로 줄어도 margin 은 71.8~73.2 로 평평했다.
검출률은 100% → 89.3% 로 떨어지는데도 그렇다.

→ **`tag_pixel_size()` 가 거리를 잡는 유일한 게이트다.**
margin 은 밝기에는 반응하므로 저조도 하한으로만 쓴다.

---

## 4. 전처리 — 넣지 않은 이유

28프레임 클립으로 9가지를 다 돌린 결과 **전부 28/28 (100%)** 로 같았다.

```
BGR2GRAY / B / G / R / max(BGR) / min(BGR) / CLAHE / R+CLAHE / LAB-L
```

**CLAHE 는 오히려 해롭다.** 모서리를 평균 0.392 px, 최대 2.572 px 밀어낸다
(단일 채널은 0.02~0.04 px). 얻는 건 margin 73.2 → 77.8 뿐인데 margin 은 이미
하한의 3.7배라 남아돈다.

이유: AprilTag 은 **내부에서 지역 적응 임계화**를 한다. 밖에서 대비를 키워봐야
중복이고, 비선형 변환은 모서리 부근 기울기를 뒤틀어 `refine_edges` 를 방해한다.

**선형 대비 확장은 다르다.** 어두운 이미지(80~130 압축)에서 원본 대비 모서리 오차가
1.095 px → 0.144 px 로 **7.6배 개선**됐다. 선형이라 밝기 곡선 모양이 보존되기 때문이다.
다만 노이즈도 같이 증폭되므로 실카메라 확인이 필요하다.

libealsense 의 처리 블록(spatial / temporal / hole-filling / disparity / colorizer 등)은
**거의 전부 depth 전용**이라 우리가 가져올 게 없었다. 소스에서
`_stream_filter.stream = RS2_STREAM_DEPTH` 로 못박혀 있다.

---

## 5. 좌표계 — 제일 헷갈리는 부분

### 카메라 기준 vs 태그 기준

```
x, y, z                     카메라 기준 태그 위치.  라이브러리가 주는 것
lateral, vertical, forward  태그 기준 카메라 위치.  역변환한 것 = 제어에 쓰는 값
```

역변환은 `p = -Rᵀ·t` 다. **부호만 뒤집는 게 아니라 회전이 곱해진다.**

같은 자리에 서서 방향만 돌리면:

| heading | x | z | lateral | forward |
|---|---|---|---|---|
| −20° | −1.966 | 2.477 | −1.000 | 3.000 |
| 0° | −1.000 | 3.000 | −1.000 | 3.000 |
| +20° | +0.086 | 3.161 | −1.000 | 3.000 |
| +30° | +0.634 | 3.098 | −1.000 | 3.000 |

**`lateral`/`forward` 는 안 변하고 `x`/`z` 는 부호까지 바뀐다.**
그래서 제어에 `x` 를 쓰면 지게차가 고개를 돌릴 때마다 목표가 움직인다.

`vertical = 카메라높이 − 태그높이` 다 (그 반대가 아니다).

### yaw ≠ heading

축 이름이 좌표계마다 다른 축을 가리킨다.

| | 항공기·CG | 카메라 (OpenCV) |
|---|---|---|
| x | 오른쪽 | 오른쪽 |
| y | **위** | **아래** |
| z | 앞 | 앞 |

| 실제 회전 | 항공기 기준 | 카메라 기준 |
|---|---|---|
| 좌우로 돎 | yaw | **pitch** |
| 갸우뚱 | roll | **yaw** |
| 위아래 끄덕 | pitch | **roll** |

측정으로 확인:

```
지게차가 30° 틀어짐   →  yaw   0.00°,  pitch 30.00°,  heading −30.00°
카메라만 30° 갸우뚱   →  yaw −30.00°,  pitch  0.00°,  heading   0.00°
지게차 20° + 카메라 15°  →  yaw −15.00°,  pitch 20.00°,  heading −20.00°
```

**`yaw` 는 카메라 장착 기울기를 잰다. 지게차 방향과 무관하다.**
`heading_deg` 만이 "지게차가 태그 축에서 몇 도 틀어졌나"에 답한다.

`docking_state()` 는 rpy 를 거치지 않고 **회전행렬에서 직접** heading 을 뽑는다
(`fwd = R·[0,0,1]`, `heading = atan2(fwd[0], fwd[2])`). 오일러 각은 분해 순서에 따라
값이 달라지고 짐벌락에서 튀기 때문이다.

### approach vs heading — 독립이다

```
approach   어디 있나 (위치).  태그에서 봤을 때 축에서 몇 도 벗어난 자리인가
heading    어디 보나 (자세).  지게차가 향한 방향이 축과 몇 도 어긋났나
```

축 위에 있어도 고개를 돌리고 있으면 `heading≠0`, 옆에 비켜서 태그를 똑바로 보면
`heading=0` 인데 `approach≠0` 이다. **진입하려면 `lateral` 과 `heading` 이 둘 다 0.**

---

## 6. 각도를 언제 믿을 수 있나

`tilt_deg` = 태그면 법선과 카메라 광축이 이루는 각. **화면에서 찌그러진 정도**다.

3 m 거리, 25 cm 태그에서 좌변·우변 길이 차이:

| tilt | 좌변 | 우변 | 차이 |
|---|---|---|---|
| 0° | 75.5 px | 75.5 px | 0.0% |
| 2° | 75.7 px | 75.4 px | 0.3 px |
| 5° | 76.1 px | 75.5 px | 0.6 px |
| 10° | 77.2 px | 76.1 px | 1.1 px |
| 45° | 111.4 px | 102.5 px | 8.9 px |

**tilt 2° 에서는 좌우 변 차이가 0.3 px** 다. 모서리 검출 노이즈에 묻힌다.
그래서 `RELIABLE_TILT_DEG = 10.0`.

각도 오차 실측 (3 m, AT3):

| heading | 각도 오차 | lateral 오차 |
|---|---|---|
| 0° | −4.47° | 235 mm |
| 5° | −1.59° | 85 mm |
| 10° | −2.04° | 108 mm |
| 20° | −0.14° | 9 mm |
| 30° | +0.50° | −25 mm |

**검출은 어느 각도에서든 100% 된다. 틀리는 건 각도 값뿐이다.**

### 판정 기준을 `approach` 에서 `tilt` 로 바꾼 이유

예전 `reliable_angle = approach >= 10°` 는 **축 위에 서면(lateral 0) 영영 false** 였다.
30° 틀어져 있어도 approach 가 0 이기 때문이다.

실측 로그에서도 `tilt 18.9°, approach 2.3° → reliable 0` 이 나왔다 —
각도를 잴 수 있는 상태인데 "믿지 마라"고 답한 것이다.

→ `reliable_angle = tilt_deg >= 10°`. 파이썬 2곳과 C++ 필터 1곳을 같이 맞췄다.

### 설계상 잘 맞는다

```
멀리서 비스듬히   tilt 크다 → 각도 정확  → heading 으로 회전
가까이 정렬됨     tilt 작다 → 각도 부정확 → lateral 로 미세조정
```

도킹 목표가 heading 0° 이므로 **정렬될수록 각도를 못 재게 되는데, 그때는 각도가 필요 없다.**

---

## 7. 검출 거리와 화면 이탈

### 태그 크기별 최대 거리 (1920×1080, fx=1359.2)

검출에는 태그 한 변이 최소 20 px, 안정적으로는 50 px 필요.

| 태그 | 하한(20px) | 실용(30px) | 안정(50px) |
|---|---|---|---|
| 20 cm | 13.6 m | 9.1 m | 5.4 m |
| 30 cm | 20.4 m | 13.6 m | 8.2 m |
| 40 cm | 27.2 m | 18.1 m | 10.9 m |

해상도를 낮추면 비례해서 준다 (640×480 이면 20 cm 태그가 안정 2.4 m).
IR 은 fx 가 389.6 이라 컬러의 약 1/3.

### 가까울 때 안 되는 이유 — 화면 이탈

카메라를 수평으로 두고 태그가 위에 있으면, 가까이 갈수록 태그가 화면 위로 나간다.

| 높이차 | 태그 중심이 나가는 거리 | 테두리까지 |
|---|---|---|
| 0.0 m | 항상 보임 | 항상 |
| 0.2 m | 0.50 m | 0.81 m |
| 0.4 m | 1.01 m | **1.31 m** |
| 0.8 m | 2.01 m | 2.32 m |

태그 1.60 m / 카메라 1.20 m 배치면 **1.31 m 아래에서 잘린다.**
태그 크기 한계(0.30 m)나 초점(0.10 m)보다 훨씬 먼저 걸린다.

AprilTag 은 네 모서리로 사각형을 닫아야 하므로 **부분 검출이 없다.**
"점점 나빠지는" 게 아니라 어느 거리에서 뚝 끊긴다.

→ **카메라 높이를 태그 높이에 맞추면** 거리와 무관하게 항상 보인다.

### 여백(quiet zone)

태그 주위에 흰 여백이 **한 칸 이상** 필요하다. 20 cm 태그면 25 mm.
A4 에는 20 cm 태그가 여백 5 mm 로 겨우 들어간다 → **A3 권장.**

---

## 8. 오차 예산

거리는 `z = fx · tag_size / 화면상_픽셀크기` 로 역산한다. 그래서:

| 거리 | 화면상 태그 | 모서리 0.2px 오차 | tag_size 1mm 오차 | fx 0.3% 오차 | 합성 |
|---|---|---|---|---|---|
| 1 m | 272 px | 0.7 mm | 5.0 mm | 3.0 mm | 5.9 mm |
| 2 m | 136 px | 2.9 mm | 10.0 mm | 6.0 mm | 12.0 mm |
| 3 m | 91 px | 6.6 mm | 15.0 mm | 9.0 mm | 18.7 mm |
| 5 m | 54 px | 18.4 mm | 25.0 mm | 15.0 mm | 34.5 mm |

**오차의 주범은 카메라가 아니라 `tag_size` 측정이다.**
자 대신 캘리퍼를 쓰면 5 m 오차가 34 mm → 23 mm 가 된다.

모서리 오차의 영향은 **거리의 제곱**에 비례한다 (멀수록 태그가 작아지므로).

### 프레임간 흔들림 (정지 상태 실측, 1.44 m)

| 값 | 표준편차 |
|---|---|
| x, y, z | 0.05 ~ 0.31 mm |
| forward | 0.89 mm |
| **lateral** | **3.79 mm** |
| heading | 0.148° |

`lateral` 이 12배 더 흔들리는 이유: **회전 노이즈가 거리에 곱해진다.**

```
0.145° × 1.44 m = 3.64 mm    ≈ lateral 실측 3.79 mm
```

거리에 비례하므로 5 m 에서는 ±13 mm 가 된다.
**N프레임 평균을 내면 √N 로 줄어든다** (10프레임이면 1/3).

---

## 9. 환경 (WSL2)

### 카메라 붙이기

```bash
./tools/wsl_attach_camera.sh
```

재부팅·재연결 때마다 필요하다. 하는 일:

1. 윈도우쪽 `usbipd` 로 장치를 WSL 에 넘김 (interop 으로 `powershell.exe` 호출)
2. `uvcvideo` 커널 모듈 로드
3. `/dev/video*`, `/dev/bus/usb/*` 권한을 `plugdev` 로

**sudo 비밀번호가 필요 없다** — 윈도우 경유로 root 를 얻는다 (`wsl -u root`).

초기에 "WSL2 커널에 uvcvideo 가 없다"고 판단했는데 **틀렸다.**
모듈로 존재하고 로드만 안 돼 있었다 (`/proc/config.gz` 에 `CONFIG_USB_VIDEO_CLASS=m`).
`lsmod`(로드된 목록)만 보고 판단하면 안 된다.

WSL2 에는 udev 데몬이 없어 `/etc/udev/rules.d` 규칙이 자동 적용되지 않는다.
그래서 권한을 수동으로 맞춘다.

### 카메라는 한 프로세스만 연다

```
1번 프로세스: 열림
2번 프로세스: 실패 — xioctl(VIDIOC_S_FMT) failed, errno=16 (EBUSY)
```

**언어와 무관하다.** 파이썬끼리도 안 된다. 리눅스 V4L2 드라이버 수준의 제약이다.

→ `realsense-viewer` 를 켜면 파이썬이 카메라를 못 연다. 동시에 쓰려면
DDS 네트워크 스트리밍(`rs-dds-adapter`)이 필요한데, SDK 를 `BUILD_WITH_DDS=ON` 으로
다시 빌드해야 하고 프로세스가 3개가 된다.

### librealsense 가 두 벌

```
pip 휠 안         .../site-packages/pyrealsense2/*.so     파이썬이 씀 (정적 링크)
/usr/local/lib    librealsense2.so.2.58                   realsense-viewer 가 씀
```

pip 모듈은 시스템 라이브러리를 **참조하지 않는다** (`ldd` 에 realsense 없음).
서로 안 섞이지만, 업그레이드할 때는 **둘 다** 해야 버전이 안 갈라진다.

---

## 10. realsense-viewer

**태그를 모른다** (바이너리에 apriltag 문자열 0개). 그래서 C++ 필터를 만들어 끼웠다.

`legacy/viewer_filter/` — SDK 가 열어둔 후처리 필터 슬롯에 클래스 하나를 등록한다.
viewer 본체(39,647줄)는 손대지 않는다.

### 그 과정에서 찾은 인텔 코드 버그 두 개

**① 오버레이가 매 틱 지워진다**

```cpp
if( ! odf ) {                     // D435i 엔 객체검출 스트림이 없어 항상 여기
    if( ++ticks_without_od_frame > 3 )
        objects->clear();         // 후처리 필터가 방금 채운 것도 같이 지움
}
```

객체검출 파이프라인과 후처리 필터가 **같은 컨테이너를 공유**하는데, OD 스트림이 없으면
4번째 틱부터 계속 비웠다. **인텔 자기네 얼굴검출 예제도 같은 이유로 깨져 있었다.**
→ `od_produced` 플래그로 OD 가 만든 것만 만료시키게 고쳤다.

**② 텍스트가 박스보다 크면 통째로 생략된다**

```cpp
if( size.y < h && size.x < bbox.w )   // 안 들어가면 아무것도 안 그림
```

2~5 m 에서 태그 박스는 50~130 px 인데 숫자 4줄은 그보다 넓다.
**정작 필요한 거리에서 아무것도 안 보였다.** → 박스 밖에 그리도록 고쳤다.

변경분은 `legacy/viewer_filter/viewer_overlay.patch` 에 있다.

### 역할

```
realsense-viewer   카메라 세팅·캘리브레이션 점검용. 값이 밖으로 안 나간다
live_pose.py       실사용. 화면도 보여주고 값도 손안에 있다
```

viewer 에만 있는 유용한 기능: **On-Chip Calibration + Health-Check**
(공장 내부파라미터가 아직 맞는지 확인). 펌웨어 업데이트, .bag 원클릭 녹화.

---

## 11. 남은 일

```
① control.py            도킹값 → 주행 명령. 다른 팀 fwd_time_model 이 거리→시간을 맡는다
                        (회전 모델은 아직 없음)
② run.py                pose + control 통합 실행
③ 실측 검증             verify.py + 줄자. tag_size 를 캘리퍼로 재는 게 먼저
④ 태그 2               탑재부 내부 태그. tag_layout 에 실측 배치 입력
⑤ 젠슨 나노 포팅        지금 설치본은 전부 x86_64. pyrealsense2 는 ARM 휠이 없어
                        librealsense 를 소스 빌드해야 한다
```

### `tag_size` 가 지금 가장 큰 오차원

```
tag size : 0.200 m  (ASSUMED default)
```

인쇄한 태그를 **검은 테두리 바깥까지** 재서 넣어야 한다. 1 mm 틀리면 2 m 에서 10 mm,
5 m 에서 25 mm 가 통째로 틀어진다.

---

## 도킹 순서 — 언제 태그를 보나

```
        멈춤          [측정]  30프레임 -> 중앙값. 움직이며 재지 않는다
          |
   1  회전 (90-heading)도      돌다가 35도 넘으면 태그가 시야 밖으로
          |
   2  직진 |lateral|   [실명]  옆을 보고 가므로 태그가 안 보인다. 개루프
          |
   3  회전 90도                태그가 다시 들어온다. heading 도 0 이 된다
          |
        멈춤          [측정]  lateral 이 아직 크면 1~3 반복
          |
   4  직진 min(남은거리, 1m)   태그를 보며 간다
          |                    조각마다 멈춰서 [측정] -> 4 반복
        멈춤          [측정]  태그1 이 화면에서 사라지기 전에 정지
          |
   5  태그2 검출        [측정]  tag_layout 으로 같은 좌표계 환산 -> 진입
```

**태그를 못 보는 구간은 2단계 하나뿐이다.** 그래서 옆이동은 한 번에 가고,
끝난 뒤 3단계에서 다시 보고 확인한다. 직진(4)은 태그가 계속 보이므로
나눠 가며 매번 고쳐 잡는다.

나누는 대가 (3m 직진, 측정 1s 포함):

| 조각 | 조각거리 | 전체 시간 | 느려짐 |
|---|---|---|---|
| 1 | 3.00m | 14.1s | 1.00배 |
| 3 | 1.00m | 19.1s | 1.46배 |
| 6 | 0.50m | 26.7s | 2.04배 |

옆이동은 반대다 — 나눌 때마다 회전이 2번씩 붙어 훨씬 비싸다.

### 3단계가 heading 도 같이 고친다

```
처음                heading = 20도
(90-20)도 회전 후    heading = 90도
90도 되돌린 후       heading =  0도
```

따로 "정면 맞추기" 단계가 필요 없다.

### 정해야 할 값

```
LAT_TOL   lateral 이 이보다 작으면 됐다고 본다. 명령 하한 17mm, 측정오차 1.4mm
STEP_M    직진 한 조각. 1.0m 면 1.46배 느려짐
STOP_M    태그1 을 놓기 전 멈출 거리. 지금 높이차(0.4m)면 1.31m 에서 잘린다
```

### 막고 있는 것

1·3단계의 **회전 모델이 없다.** 받은 것은 직진(거리->시간)뿐이다.
다른 팀에 물어볼 것 — 제자리 회전 명령이 있나, 각도를 주나 시간을 주나, 몇 도에 몇 초인가.


---

## 12. SDK 소스 근거

코드에서 인용을 걷어냈으므로 여기 모은다. 경로는 `librealsense/src/` 기준.

| 알게 된 사실 | 소스 |
|---|---|
| `auto_exposure_limit` 은 depth 센서에만 등록. 그나마 `CAP_GLOBAL_SHUTTER` 일 때만 (주석: "not supported on rolling-shutter") | `ds/d400/d400-device.cpp:1062-1070` |
| 컬러 센서가 실제로 받는 옵션은 넷뿐 — EXPOSURE / GAIN / ENABLE_AUTO_EXPOSURE / AUTO_EXPOSURE_PRIORITY | `ds/ds-color-common.cpp:84-97` |
| 컬러 노출은 v4l2 로 직행하므로 **100 µs 눈금** (UVC 규격) | `linux/backend-v4l2.cpp:2476` → `V4L2_CID_EXPOSURE_ABSOLUTE` |
| **뎁스/IR 노출은 µs 단위** — 같은 enum 인데 센서에 따라 100배 다르다 | `ds/ds-color-common.cpp:117` |
| **노출을 쓰면 AE 가 조용히 꺼진다.** EXPOSURE 가 `auto_disabling_control` 로 감싸여 있어 set 하면 SDK 가 AE 를 0 으로 내린다 | `ds/ds-color-common.cpp:90-93`, `option.cpp:84-105` |
| gain 은 `value==0` 일 때만 기본값을 덮어쓴다 | `ds/ds-color-common.cpp:26-49` |
| `power_line_frequency` 값 매핑 (0=끔 1=50Hz 2=60Hz 3=자동) | `ds/d400/d400-color.cpp:205-212` |
| 파이프라인 출력 큐는 **용량 1**, 넘치면 오래된 것을 버린다 | `pipeline/aggregator.cpp:16` |
| pipeline 이 내부에 syncer 를 물고 있다 | `pipeline.cpp:219` |
| 프레임 메타데이터엔 `V4L2_META_FMT_D4XX` 노드가 필요하고, 그건 커널 패치가 만든다 | `linux/backend-v4l2.cpp:2875` |
| 메타데이터가 없으면 **조용히** 시스템 시간으로 대체 ("UVC metadata payloads not available") | `ds/ds-timestamp.cpp:60-74` |
| `time_of_arrival` 은 항상 온다 | `sensor.cpp:79` |
| AE ROI 기본값은 가운데 — 사방으로 크기의 1/8 씩 뗀 영역 | `common/stream-model.cpp:331-343` |
| BGR8 변환식 (BT.601 limited-range) | `color-formats-converter.cpp:293-301` |
| Y8 은 컬러 스트림 포맷 목록에 없다 | `device.cpp:197` `map_supported_color_formats` |

### 여기서 나오는 함정 하나 — 노출과 AE 는 같이 못 쓴다

```
set_option(EXPOSURE, ...)   →  SDK 가 ENABLE_AUTO_EXPOSURE 를 0 으로 내린다
```

의도된 동작이지만 **아무 경고가 없다.** 쓴 뒤에는 반드시 되읽어 확인할 것.
AE ROI 가 실패하는 것도 같은 이유다 (ROI 는 AE 가 켜져 있어야 걸린다).


---

## 참고

- 원본 블로그: <https://joonhyung-lee.github.io/blog/2023/apriltag-pose-estimation/>
  (`legacy/blog_post.html` 에 저장. 블로그는 카메라 설정을 하나도 안 건드렸다 —
   정지 데모라 블러가 없었기 때문이다.)
- 다른 팀의 주행 명령 모델: `../forward/fwd_time_model.py`
  거리 → 명령 시간 변환. `t0=0.507s` 지연, 가속 2.036 s, `vmax=0.284 m/s`.
