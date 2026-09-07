# KRRI AprilTag 지게차 도킹 — 작업 규칙 (Claude 가 매 세션 읽는다)

카메라로 AprilTag 을 보고 탑재부 기준 지게차 위치를 낸 뒤 그 값으로 도킹한다.

## 저장소 규칙 (이걸 틀리면 안 된다)
- **활성 개발 = `apriltag_v2/`**. `apriltag_v1/` 은 **동결 — 절대 수정 금지**(사이드스텝 기반, 참고용).
  v2 는 lateral 을 직접 안 잡는 조준-전진(lateral_no) 규칙을 개발하는 곳이다.
- **브랜치**
  - **평소 작업 = `jm_mac`.** 맥북·우분투용 개발·환경 변경은 전부 여기서 하고, 이 맥북은 항상 jm_mac 에 체크아웃.
  - `jm` : 사용자의 로컬(연구실) 컴퓨터 전용. **평소 흐름에서 배제** — 자동으로 당기지 않는다. 사용자가
    "jm 에서 이 부분 고쳤다, jm_mac 에 합칠지 보자" 고 가져올 때만 확인·논의 후 `git merge origin/jm`.
  - VM(`ubuntu-vm`) : `jm_mac` 을 받아 **쓰기만** 한다. VM 안에서 커밋하지 않는다.
  - `main` : `jm_mac` 의 v2 를 주석·docstring 뺀 **부모 없는 단일 스냅샷**(GitHub 기본 브랜치). config/*.py 와
    다른 팀 파일 3종(control_forklift_v2, control_광운대, fwd_time_model)은 주석 유지. 배포용이라 VM 엔 안 올린다.
    루트에 `.gitignore`·`README.md`·`requirements.txt`(정리본, apriltag_v2 밖) + `apriltag_v2/`.
    **main 에 올리는 건 자동으로 하지 않는다** — 사용자가 "main 에 반영해줘" 라고 할 때만 rebuild·force-push.
- **config import 스타일**: control·detection 모두 `from config import control as C` / `... detection as D` + `C.X`/`D.X`.
  config 상수의 원산은 `config/detection.py`. 공개 API 는 `src/models/__init__` 가 config.detection 에서 직접 재수출한다.
- **requirements**: `requirements.txt`(리포 루트) **마커본 하나** — 패키지·OS 마커만, 설명 없음. 우분투/Jetson(arm64)는
  pyrealsense2 를 마커로 빼고 소스 빌드, WSL·그램(x86)은 PyPI 휠, macOS 는 macosx. (풀어 쓴 macos/ubuntu 파일은 없앴다.)
  설치는 OS 별로:
  - **Windows/그램** — 먼저 Kvaser Drivers+CANlib SDK 설치 후 `pip install -r requirements.txt`
  - **우분투/Jetson** — `tools/etc/setup_ubuntu_arm64.sh` 가 이걸 쓴다 (pyrealsense2 는 소스 빌드)
  - **macOS(개발용)** — `pip install -r requirements.txt` (실카메라 안 됨, 웹캠만)
  - **설치 확인** — `python tools/check/check_setup.py`
- **tools 구조(v2)**: 현장 주력 `tools/run.py`·`tools/analyze_run.py` 는 최상위, `tools/check/`(점검), `tools/etc/`(측정·테스트·설치).

## 코드 갱신 (평소: jm_mac → VM)
맥에서 jm_mac 을 고쳐 커밋한 뒤 VM 에 반영한다:
```bash
git push ubuntu-vm:krri.git jm_mac:refs/heads/jm_mac      # GitHub 에도: git push origin jm_mac
ssh ubuntu-vm 'cd ~/krri && git fetch -q mac && git checkout -q -B jm_mac mac/jm_mac && git log --oneline -1'
```
VM 은 리모트 `mac`(맥이 밀어 넣는 bare `~/krri.git`) + `origin`(GitHub). VM 파이썬 `~/miniforge3/envs/krri/bin/python`(3.11).
**jm 병합은 자동으로 하지 않는다** — 사용자가 jm 변경을 가져와 "합칠지 보자" 할 때만 `git checkout jm_mac && git merge origin/jm` 후 논의·검증.

## 주의 — main 리빌드 시 work_dirs 소실 (2026-09-07 겪음)
`work_dirs/`(실주행·live_pose 로그)는 .gitignore 라 git 이 추적하지 않는다. main 을 부모 없는 orphan 단일
커밋으로 다시 만들 때 작업 트리에서 `rm -rf apriltag_v1` 하면 이 무시 폴더까지 지워지고, jm_mac 으로
`git checkout` 해도 **추적 파일만 복원되어 work_dirs 는 안 돌아온다**(맥 v1 work_dirs 가 이렇게 사라졌다, VM 에서 복구).
- 안전한 main 리빌드: 지우려는 폴더는 `git rm -r --cached apriltag_v1`(인덱스만) 로 빼고 작업 트리의 `rm -rf` 는
  피하거나, orphan 작업을 **별도 worktree/클론**에서 한다. 최소한 리빌드 전 `work_dirs` 를 백업하거나 VM 에 남겨 둔다.
- 로그가 없어졌으면 VM 에서 복구: `scp -r ubuntu-vm:~/krri/apriltag_v1/work_dirs apriltag_v1/`.

## CAN 제어 (control_forklift_v2, 실차 확인 2026-09-07)
- 주행 프레임 0x1E3(중립 127): **byte2=전/후진**(전진 67, 후진 187), **byte1=조향/제자리회전**(좌 187, 우 67;
  제자리 회전은 byte1 을 ±20 = 147/107). **byte4 는 포크 리프트** — 회전에 쓰면 포크가 올라간다(사고 주의).
- `tools/run.py` 는 출발 전 CAN 템플릿을 검사한다(다섯 동작이 byte1/2 만 쓰고 나머지 중립이 아니면 출발 거부).
- 실측: 회전 팔 A=1.46m(카메라→회전중심), 직진 정속 ~0.30m/s, 명령 지연 ~1s, 정지 관성 ~0.12m. 회전 관성 ~1.5도.

---

# 맥북(Apple Silicon) 위의 Ubuntu VM — 카메라 SDK·IMU·CAN 개발환경

macOS 에서는 librealsense 가 카메라를 못 연다(UVCAssistant 선점 + 2.56.5 IMU 크래시). 그래서 맥북 안에
**UTM + Ubuntu 24.04 arm64 VM** 을 두고 USB 장치(RealSense, Kvaser)를 VM 에 통째로 넘긴다.
VM 은 Jetson 과 같은 arm64 리눅스라 여기서 검증한 절차·코드가 Jetson 으로 그대로 간다.
2026-09-07 맥북 M2 / macOS 26.3 / UTM / Ubuntu 24.04.4 실측.

## 되는 것 / 안 되는 것
- 된다: 검출·자세, depth, IR, IMU, 프레임 메타데이터, bag 녹화, canlib(CAN 송수신), dry-run.
  실측: VM 이 카메라를 **USB 3.2** 로 받는다. 컬러 640x480/1280x720/1920x1080 모두 30 fps, depth 30,
  컬러+depth 동시 30, IMU(gyro 200Hz + accel 100Hz) OK, global_time 타임스탬프.
- RSUSB 백엔드의 버릇 둘 (Jetson 도 RSUSB 로 빌드하면 같다):
  1) **IMU 는 먼저 연 쪽이 갖는다.** 컬러를 먼저 열면 뒤에 여는 자이로가 'failed to set power state'.
     자이로 먼저, 컬러 나중이면 둘 다 된다 — run.py/live_pose 가 그렇게 연다.
  2) 컬러 actual_exposure/gain_level 메타데이터는 **자동노출 끈 때만** 온다. realsense_check 의 그 WARN·'커널 패치'는 V4L2 용이라 무시.
- 이 D435i IMU 는 BMI085 → accel 유효값 100/200/400 Hz(63/250 은 BMI055). imu_yaw 가 63/100/200/250 중 풀리는 첫 값을 고른다.
- **하지 말 것: VM 에서 실차 주행.** 제어 경로에 macOS→USB 전달→VM 이 끼어 맥 절전·USB 재연결 순간 명령이 끊긴다.
  실주행은 Jetson/Windows(그램). (굳이 하면 맥 절전 끄고, 허브 없이 직결, 사람이 제동 위치.)

## VM 만들기 (한 번)
1. `brew install --cask utm`, Ubuntu 24.04 **desktop arm64** ISO(3.3 GB) 받아 sha256 확인.
2. UTM → 새 VM → **가상화** → Linux → ISO. Apple 가상화는 **체크 해제**(USB 전달은 QEMU 엔진만).
3. 메모리 6144 MB, CPU 4, 디스크 30 GB. 저장 후 편집 → **입력** → USB 지원 **3.0 (XHCI)**, 최대 공유 USB 4.
4. 설치: Erase disk(VM 디스크만), 사용자 `seojeongmin`, 자동 로그인. 재시작 후 GRUB 이 또 뜨면 "Boot from next volume",
   이후 툴바 CD 아이콘으로 ISO Eject.
5. VM 터미널에서 (맥 공개키 `~/.ssh/id_ed25519_vm.pub`, 맥 IP 는 VM 에서 192.168.64.1):
   ```bash
   sudo apt update && sudo apt install -y openssh-server curl git gh
   mkdir -p ~/.ssh && curl -s http://192.168.64.1:8000/mac.pub >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys
   sudo bash -c 'echo "seojeongmin ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/seojeongmin && chmod 440 /etc/sudoers.d/seojeongmin'
   hostname -I      # 보통 192.168.64.2
   ```
   (맥에서는 공개키 든 폴더에서 `python3 -m http.server 8000` 을 켜 둔다.)
6. 맥 `~/.ssh/config`: `Host ubuntu-vm / HostName 192.168.64.2 / User seojeongmin / IdentityFile ~/.ssh/id_ed25519_vm`.
   VS Code Remote-SSH 로 `ubuntu-vm` 붙어 `/home/seojeongmin/krri` 를 연다.

## 실행환경 설치 (SSH 로, 40~60분·대부분 librealsense 빌드)
```bash
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v2/tools/etc/setup_ubuntu_arm64.sh apt'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v2/tools/etc/setup_ubuntu_arm64.sh conda'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v2/tools/etc/setup_ubuntu_arm64.sh realsense'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v2/tools/etc/setup_ubuntu_arm64.sh pip'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v2/tools/etc/setup_ubuntu_arm64.sh can'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v2/tools/etc/setup_ubuntu_arm64.sh check'
```
GitHub 로그인이 없으면 리포는 맥에서 밀어 넣는다: `ssh ubuntu-vm 'git init -q --bare ~/krri.git'` → `git push ubuntu-vm:krri.git jm_mac:refs/heads/jm_mac` → VM 에서 clone.

## 장치 넘기기와 검증 (실험 때마다)
1. RealSense/Kvaser 를 맥에 **허브 없이 직결**. VM 창 툴바 **USB 아이콘** → 장치 체크 → VM 이 가져간다.
2. 확인:
   ```bash
   ssh ubuntu-vm 'lsusb | grep -i -E "intel|kvaser"'
   ssh ubuntu-vm 'cd ~/krri/apriltag_v2 && ~/miniforge3/envs/krri/bin/python tools/check/device_check.py --no-gui'   # 카메라+IMU
   ssh ubuntu-vm '~/miniforge3/envs/krri/bin/python -c "from canlib import canlib; print(canlib.getNumberOfChannels())"'
   ```
3. 실행 (VS Code Remote-SSH 터미널, sudo 불필요):
   ```bash
   cd ~/krri/apriltag_v2 && ~/miniforge3/envs/krri/bin/python tools/run.py --dry-run   # CAN 안 보냄
   cd ~/krri/apriltag_v2 && ~/miniforge3/envs/krri/bin/python tools/run.py             # 실주행
   ```
   SPACE 로 시작(터미널에서 직접 읽는다), Ctrl+C 로 비상정지 후 종료.
4. 화면(`--show`, live_pose): VM `~/.bashrc` 가 SSH 셸에 `DISPLAY=:0` 을 넣어 둬서(conda 단계가 넣음) 창이 **UTM VM 창**에 뜬다.
   맥 화면에 띄우려면 `ssh -X ubuntu-vm`(XQuartz).

## Jetson 으로 옮길 때
같은 스크립트를 같은 순서로. 다른 곳 셋: 커널 헤더(Jetson 은 nvidia-l4t, 스크립트가 분기), CUDA(있으면 켬, 분기),
USB 넘기기 없음(직접 꽂힌다). 화면 없는 Jetson 도 맥에서 SSH·Remote-SSH 로 똑같이 다룬다. USB 직결 주소 192.168.55.1.
