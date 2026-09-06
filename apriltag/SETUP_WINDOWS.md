# 현장 노트북(Windows) 셋업 — 한 번에 되게

목표: 집/연구실에서 아래 "미리 챙길 것"만 준비해 가면, 현장에서는
**장비 꽂고 `run.py` 켜는 것 말고 할 일이 없다.**

## 연결 그림

```
지게차 CAN ──(케이블)── Kvaser USB 인터페이스 ──┐
                                                노트북(Windows) ── USB ── D435i 카메라
```

노트북에서는 **네이티브 Windows 로** 실행한다. WSL 을 쓰면 카메라·IMU·CAN
전부 USB 통과 고생을 하게 된다 — 네이티브면 셋 다 그냥 된다.

---

## 1. 미리 챙길 것 (인터넷 있는 곳에서)

| 챙길 것 | 어디서 |
|---|---|
| Python 3.11 설치파일 | python.org (설치 시 "Add to PATH" 체크) |
| **Kvaser Drivers for Windows** 설치파일 | kvaser.com → Downloads. **pip 로 안 된다** — 이거 없으면 canlib 가 장비를 못 찾는다 |
| **Kvaser CANlib SDK** 설치파일 | 같은 곳. **둘 다 필요하다** — Drivers 만 깔면 pip canlib 가 레지스트리 키(CANLIB32)를 못 찾아 `WinError 2` 로 죽는다 (2026-09-06 그램에서 실증) |
| 이 저장소 | `git clone https://github.com/SeoJe0ngmin/KRRI.git` (private — 로그인 필요). 또는 zip 으로 |
| (인터넷 없는 현장 대비) 오프라인 휠 | 개발 PC 에서 `./tools/make_offline_bundle.sh` → `offline_wheels/` 가 생김. 저장소 폴더째 USB 에 복사 |

## 2. 노트북 셋업 (이것도 가능하면 미리)

```
① Python 3.11 설치
② Kvaser Drivers for Windows 설치
③ KRRI\apriltag 폴더에서  setup_windows.bat  더블클릭
      - offline_wheels\ 가 있으면 인터넷 없이 설치된다
      - 끝에 점검(check_setup.py)이 자동으로 돈다
④ 점검 결과가 "판정: 준비 완료" 인지 확인
```

점검만 다시 돌리려면: `python tools\check_setup.py`

## 3. 리허설 (지게차 없이, 카메라만 꽂고)

```
python tools\realsense_check.py          카메라가 붙었나 — 전부 PASS 여야 함
python tools\imu_check.py                IMU 부호 — 카메라를 반시계로 90도 돌려
                                         yaw 가 +90 이면 OK, -90 이면
                                         config/imu.py 의 IMU_YAW_SIGN 을 뒤집는다
python tools\run.py --dry-run --show     태그 앞에 세워 두고 판단이 맞는지
                                         (CAN 없이 알고리즘 전체가 돈다)
```

여기까지 집에서 끝내 두면 현장에서는 4번만 남는다.

## 4. 현장

```
① Kvaser 를 노트북 USB 와 지게차 CAN 에 연결
② python tools\run.py --show --record-events
③ SPACE 로 시작, ESC 가 비상정지
④ 끝나면  python tools\analyze_run.py   -> 결과 판정 + 파라미터 권장값
```

### 첫 주행에서 눈으로 확인할 것 딱 하나

**rotate_ccw 명령에 지게차가 왼쪽(반시계)으로 도는가.**
반대로 돌면 5도에서 자동 정지(wrong-way)하니 위험하진 않다 —
`src/models/control/rot_control.py` 의 rotate_to() 안 movement 매핑
두 곳만 맞바꾸면 된다 (코드 주석에 위치가 적혀 있다).

### 안 될 때

| 증상 | 볼 곳 |
|---|---|
| "CAN 연결 실패" | Kvaser Drivers+SDK 설치했나 → `check_setup.py` 의 Kvaser 채널 줄. 채널 번호는 `control_forklift_v2.py` 의 `CAN_CHANNEL`(기본 0), 속도 500kbps 가 지게차와 같아야 함 |
| **ch0 가 Virtual** 인데 실장비를 꽂았다 | 명령이 가상 버스로 빠져 지게차가 안 움직이는데 에러도 안 난다. check_setup 에서 ch0 가 `Kvaser Leaf...` 인지 반드시 확인. Virtual 이 앞자리를 차지하면 Kvaser Device Guide 에서 가상 채널을 끄거나 재부팅 |
| 회전이 wrong-way 로 계속 멈춤 | 위 "첫 주행 확인" — movement 매핑 스왑 |
| 회전이 안 되고 시간모델 개루프 경고 | IMU 를 못 열었다 — 카메라 USB 를 다시 꽂고 재시작 |
| 태그를 계속 못 봄 | `--show` 로 화면을 보면서 노출(`e`, `[`, `]`) 조절 |
