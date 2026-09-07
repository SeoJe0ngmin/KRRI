# 폴더 구조 (2026-09-07 재편)
- `apriltag_v1/` 오늘까지의 코드 (사이드스텝 기반, lateral_yes). `apriltag_v2/` 는 v1 복사본에서 sim 을 뺀 것 — lateral 을 직접 안 잡는 조준-전진 규칙(lateral_no)을 여기서 개발.
- `docs/` 모든 md 와 requirements*.txt. 설치: `pip install -r docs/requirements.txt` (맥은 requirements-macos.txt).

# 맥북(Apple Silicon) 위의 Ubuntu VM — 카메라 SDK·IMU·CAN 까지 되는 개발환경

macOS 에서는 librealsense 가 카메라를 못 연다(requirements-macos.txt 참고). 그래서 맥북 안에
**UTM + Ubuntu 24.04 arm64 VM** 을 두고, USB 장치(RealSense, Kvaser)를 VM 에 통째로 넘겨서 쓴다.
VM 은 Jetson 과 같은 arm64 리눅스라 여기서 검증한 절차와 코드가 Jetson 으로 그대로 간다.
2026-09-07 맥북 M2 / macOS 26.3 / UTM / Ubuntu 24.04.4 에서 실측.

## 되는 것 / 안 되는 것
- 된다: 검출·자세, depth, IR, IMU, 프레임 메타데이터, bag 녹화, canlib(CAN 송수신), dry-run
- 실측(2026-09-07): VM 이 카메라를 **USB 3.2** 로 받는다. 컬러 640x480/1280x720/1920x1080 모두 30 fps,
  depth 30 fps, 컬러+depth 동시 30 fps, IMU(gyro 200Hz + accel 100Hz) OK, 타임스탬프 global_time.
- RSUSB 백엔드의 버릇 둘 (Jetson 을 RSUSB 로 빌드해도 같다):
  1) **IMU 는 먼저 연 쪽이 갖는다.** 컬러 파이프라인을 먼저 열면 같은 프로세스의 두 번째
     파이프라인도, 별도 프로세스도 IMU 를 못 연다('failed to set power state'). 자이로를 먼저 열고
     컬러를 나중에 열면 둘 다 잘 돈다 — run.py / live_pose.py 가 그렇게 연다(jm_mac 에서 고침, 실측 확인).
  2) 컬러 프레임의 actual_exposure / gain_level 메타데이터는 **자동노출을 끈 때만** 온다
     (수동 노출이면 정상). realsense_check.py 의 그 WARN 과 '커널 패치' 안내는 V4L2 백엔드용이라 무시.
- 이 D435i 의 IMU 는 BMI085 라 accel 유효값이 100/200/400 Hz 다(63/250 은 BMI055). imu_yaw.py 는
  63/100/200/250 중 풀리는 첫 값을 고른다(고정 63 이면 'Couldn't resolve requests' 로 자이로만 열렸다).
- 하지 말 것: VM 에서 실차 주행. 제어 경로에 macOS → USB 전달 → VM 이 끼어서 맥 절전·USB 재연결
  순간 명령이 끊긴다. 실주행은 Jetson/Windows 에서. (굳이 하면 맥 절전 끄고, 허브 없이 직결, 사람이 제동 위치)

## 브랜치 규칙
- `jm`: 로컬(연구실) 컴퓨터에서만 수정. 맥북과 VM 은 절대 안 건드린다.
- `jm_mac`: 맥북에서 `jm` 을 받아(`git merge origin/jm`) **환경 관련 변경만** 얹는다. 설치 마커, 이 문서, 스크립트.
- VM: `jm_mac` 을 받아 쓰기만 한다. VM 안에서 커밋하지 않는다.

## 1. VM 만들기 (한 번)
1. `brew install --cask utm`, Ubuntu 24.04 **desktop arm64** ISO(3.3 GB) 다운로드 후 sha256 확인
2. UTM → 새 VM → **가상화** → Linux → ISO 선택. Apple 가상화는 **체크 해제**(USB 전달은 QEMU 엔진에서만 된다)
3. 메모리 6144 MB, CPU 4, 디스크 30 GB. 저장 후 편집 → **입력** → USB 지원 **3.0 (XHCI)**, 최대 공유 USB 장치 4
4. Ubuntu 설치: Erase disk(VM 디스크만 지움), 사용자 `seojeongmin`, 자동 로그인. 재시작 후 GRUB 이 또 뜨면
   "Boot from next volume", 이후 툴바 CD 아이콘으로 ISO 를 Eject
5. VM 터미널에서 (맥 공개키는 `~/.ssh/id_ed25519_vm.pub`, 맥 IP 는 VM 에서 192.168.64.1):
   ```bash
   sudo apt update && sudo apt install -y openssh-server curl git gh
   mkdir -p ~/.ssh && curl -s http://192.168.64.1:8000/mac.pub >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys
   sudo bash -c 'echo "seojeongmin ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/seojeongmin && chmod 440 /etc/sudoers.d/seojeongmin'
   hostname -I      # 보통 192.168.64.2
   ```
   (맥에서는 공개키가 든 폴더에서 `python3 -m http.server 8000` 을 켜 둔다)
6. 맥 `~/.ssh/config` 에 `Host ubuntu-vm / HostName 192.168.64.2 / User seojeongmin / IdentityFile ~/.ssh/id_ed25519_vm`.
   VS Code 의 Remote-SSH 로 `ubuntu-vm` 에 붙어 `/home/seojeongmin/krri` 를 연다.

## 2. 실행환경 (스크립트, SSH 로)
리포를 VM 에 넣는다. GitHub 로그인이 없으면 맥에서 밀어 넣는다:
```bash
# 맥에서
ssh ubuntu-vm 'git init -q --bare ~/krri.git'
git push ubuntu-vm:krri.git jm_mac:refs/heads/jm_mac
ssh ubuntu-vm 'git clone -q -b jm_mac ~/krri.git ~/krri && git -C ~/krri remote rename origin mac && git -C ~/krri remote add origin https://github.com/SeoJe0ngmin/KRRI.git'
```
그 다음 단계별로 (전부 합쳐 40~60분, 대부분 librealsense 빌드):
```bash
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v1/tools/setup_ubuntu_arm64.sh apt'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v1/tools/setup_ubuntu_arm64.sh conda'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v1/tools/setup_ubuntu_arm64.sh realsense'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v1/tools/setup_ubuntu_arm64.sh pip'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v1/tools/setup_ubuntu_arm64.sh can'
ssh ubuntu-vm 'cd ~/krri && bash apriltag_v1/tools/setup_ubuntu_arm64.sh check'
```
파이썬은 `~/miniforge3/envs/krri/bin/python`(3.11). 스크립트 안에 각 단계가 무엇을 왜 하는지 적혀 있다.

## 3. 장치 넘기기와 검증 (실험 때마다)
1. RealSense/Kvaser 를 맥에 **허브 없이 직결**. VM 창 툴바의 **USB 아이콘** → 장치 체크 → VM 이 가져간다.
2. 확인:
   ```bash
   ssh ubuntu-vm 'lsusb | grep -i -E "intel|kvaser"'
   ssh ubuntu-vm 'cd ~/krri/apriltag_v1 && ~/miniforge3/envs/krri/bin/python tools/realsense_check.py --no-gui'   # depth·메타데이터·IMU
   ssh ubuntu-vm '~/miniforge3/envs/krri/bin/python -c "from canlib import canlib; print(canlib.getNumberOfChannels())"'
   ```
3. 실행 (VS Code Remote-SSH 터미널에서, sudo 불필요):
   ```bash
   cd ~/krri/apriltag_v1 && ~/miniforge3/envs/krri/bin/python tools/run.py --dry-run   # CAN 안 보냄
   cd ~/krri/apriltag_v1 && ~/miniforge3/envs/krri/bin/python tools/run.py             # 실주행
   ```
   SPACE 로 시작(터미널에서 직접 읽는다), Ctrl+C 로 비상정지 후 종료.
4. 화면(`--show`, live_pose): VM 의 `~/.bashrc` 끝에 SSH 셸이 VM 화면(`DISPLAY=:0`)을 쓰도록 넣어 두었다
   (setup_ubuntu_arm64.sh conda 단계가 넣는다). 그래서 VS Code SSH 터미널에서 돌려도 창은 **UTM 의 VM 창**에 뜬다.
   그 설정이 없으면 SSH 터미널에는 DISPLAY 가 없어 `qt.qpa.xcb: could not connect to display` 로 죽는다.
   맥 화면에 띄우려면 `ssh -X ubuntu-vm`(XQuartz).
5. realsense_check.py 에서 남는 WARN 은 '프레임별 노출/게인' 하나뿐이고(자동노출 켜진 컬러의 정상 동작) 나머지는 PASS 다.

## 4. 코드 갱신 (jm → jm_mac → VM)
```bash
# 맥에서
git fetch origin && git checkout jm_mac && git merge origin/jm && git push origin jm_mac
git push ubuntu-vm:krri.git jm_mac:refs/heads/jm_mac
ssh ubuntu-vm 'cd ~/krri && git fetch -q mac && git checkout -q -B jm_mac mac/jm_mac && git log --oneline -1'
```
VM 에서 `gh auth login` 을 해 두면 `git pull origin jm_mac` 으로도 된다.

## Jetson 으로 옮길 때
같은 스크립트를 같은 순서로 돌린다. 다른 곳은 셋: 커널 헤더(Jetson 은 nvidia-l4t 패키지, 스크립트가 분기),
CUDA(있으면 켠다, 스크립트가 분기), USB 넘기기 과정 없음(직접 꽂힌다). 화면 없는 Jetson 은 맥에서 SSH 와
VS Code Remote-SSH 로 똑같이 다룬다. USB 직결 시 주소는 192.168.55.1.
