# 기존 맥북 Ubuntu VM에서 실행

공통 설정 입력은 [CONFIG.md](CONFIG.md), 점검·주행 명령은 [run.md](run.md)를 따른다. 아래는 새 v1 예시이며 v2~v4도 같은 구조다.

Nano용과 VM용은 **같은 제어 코드**를 실행한다. Linux 실행 환경과 USB 연결 위치만 달라진다. VM에서 알고리즘/장비 연동을 확인한 뒤 같은 소스를 Nano로 옮길 수 있다. 지연과 성능 측정 결과까지 같은 것으로 취급하지는 않는다.

저장소의 기존 환경 기록은 UTM + Ubuntu 24.04 arm64, SSH 별칭 `ubuntu-vm`, Python `~/miniforge3/envs/krri/bin/python`이다. 아래 스크립트는 이 Python이 있으면 우선 사용하고, 없으면 `python3`를 사용한다. VM에 다른 환경을 만들었다면 `RD_PYTHON`으로 실제 Python 경로를 지정한다.

## 1. 기존 VM을 켠다

UTM에서 기존 Ubuntu VM을 시작한다. OS나 카메라/CAN 드라이버를 처음부터 다시 설치하지 않는다. RealSense와 Kvaser를 맥북에 연결하고 UTM의 USB 장치 선택 메뉴에서 두 장치를 Ubuntu 게스트에 연결한다. 카메라 측정만 할 때는 Kvaser 연결 없이도 실행 가능하다.

맥북에서 접속되는지 먼저 확인한다.

```bash
ssh ubuntu-vm
```

VM 터미널에서 장치가 보이는지 확인한다.

```bash
lsusb
```

USB 목록에 RealSense와 Kvaser가 없으면 Python 코드보다 UTM USB 전달 상태부터 확인한다. 기존 다른 카메라 프로그램은 종료한 상태에서 새 프로그램을 실행한다.

## 2. 새 폴더를 VM에 전송한다

아래는 **맥북 터미널**에서 실행한다. 이전 버전 폴더는 건드리지 않는다. `krri/work/`의 실험 기록이나 Python 가상환경은 복사하지 않는다.

```bash
cd /Users/seojeongmin/krri
ssh ubuntu-vm 'mkdir -p ~/krri/docking_system/configs'
rsync -av --exclude work/ --exclude calibration/ --exclude __pycache__/ --exclude .venv/ --exclude /configs/common.json \
  docking_system/ ubuntu-vm:~/krri/docking_system/
rsync -av --ignore-existing docking_system/configs/common.json ubuntu-vm:~/krri/docking_system/configs/
```

공통 도구 때문에 버전 하나만 복사하지 않고 `docking_system` 전체를 전송한다. VM의 기존 common.json과 보정 보고서는 유지한다. 처음에는 맥북 공통 설정을 복사한다. 맥북에서 수정한 설정으로 의도적으로 교체할 때만 마지막 명령의 `--ignore-existing`을 제거한다.

새 기록은 `~/krri/work/docking_system/robust_docking_vN/`에 저장한다. 기존 기록을 이동하거나 삭제할 필요 없다. `update_jetson.sh`는 Jetson 사용자·경로 전용이므로 VM에는 위 전송 절차를 쓴다. 실제 VM 전송은 이번 코드 변경에 포함하지 않았다.

## 3. VS Code로 연다

맥북 VS Code에 Microsoft **Remote - SSH** 확장을 설치한다.

1. `Cmd+Shift+P` → `Remote-SSH: Connect to Host…` → `ubuntu-vm`.
2. 원격 접속 후 `/home/seojeongmin/krri/docking_system/robust_docking_v1` 폴더를 연다.
3. VS Code 터미널을 연다. 왼쪽 아래 접속 대상이 VM인지 확인한다.
4. 다음을 실행한다.

```bash
bash scripts/run_vm.sh doctor
bash scripts/run_vm.sh check-devices
bash scripts/run_vm.sh measure --view
```

`run_vm.sh`는 macOS에서 직접 실행하면 VM 내부에서 실행하라는 메시지를 내고 끝난다. 맥북의 Python으로 RealSense/CAN을 여는 실수를 줄이기 위한 구분이다. 프로그램을 시작하는 Enter는 이 **원격 터미널**에서 누른다.

VS Code의 `Terminal → Run Task…`에서도 `VM: environment check (no devices)`와 `VM: camera + dashboard (no CAN)`를 선택할 수 있다. 이 작업 목록은 `robust_docking_v1` 자체를 열린 폴더로 사용할 때를 기준으로 작성했다.

## 4. 맥북 브라우저로 화면을 본다

VS Code의 **Ports** 패널에서 `Forward a Port` → `8765`를 입력하고 표시되는 Local Address를 브라우저에서 연다. 보통 `http://127.0.0.1:8765`이다. 다른 프로그램이 이미 로컬 포트를 사용하면 VS Code가 다른 주소를 제시할 수 있으므로 패널의 주소를 따른다.

VS Code의 포트 전달 대신 SSH 터널을 사용할 수도 있다. 이 경우 맥북 터미널에서 아래 접속을 유지한다.

```bash
ssh -t -L 8765:127.0.0.1:8765 ubuntu-vm
```

접속된 VM에서:

```bash
cd ~/krri/docking_system/robust_docking_v1
bash scripts/run_vm.sh measure --view
```

맥북 브라우저에서 `http://127.0.0.1:8765`를 연다. XQuartz나 UTM 화면 내부의 `cv2.imshow`는 필요하지 않다. 화면 구성과 CAN 제어 방식은 Nano 버전과 같다.

## 5. 측정과 실차 실행 구분

```bash
# 카메라/IMU/화면: CAN 열지 않음
bash scripts/run_vm.sh measure --view --seconds 60

# 직접 Python을 지정할 때
RD_PYTHON="$HOME/miniforge3/envs/krri/bin/python" bash scripts/run_vm.sh doctor

# 현장 설정 완료 후 정책 생성
bash scripts/run_vm.sh synthesize --config configs/hardware_vm.json --output ../../work/docking_system/robust_docking_v1/vm_policy.json

# 판단만 실행
bash scripts/run_vm.sh shadow --policy ../../work/docking_system/robust_docking_v1/vm_policy.json --view

# Kvaser 연결 + 터미널 Enter로 주행 시작
bash scripts/run_vm.sh drive --policy ../../work/docking_system/robust_docking_v1/vm_policy.json --view --execute
```

종료/정지는 Ctrl+C다. 주행 중에는 `s`+Enter로도 정지할 수 있다. 실차 설정이 비어 있는 현재 상태에서는 마지막 주행 명령은 CAN 초기화 전에 거부된다.

`hardware_vm.json`은 VM용 실물/센서/지연 검증 설정이다. 기본 `deployment.timing_verified_on_this_host=false`이며, VM에서 확인한 근거가 준비되기 전에는 주행할 수 없다. VM으로 바꿨다고 관측 만료 기준을 자동으로 크게 늘리지 않는다. 같은 카메라라도 UTM USB 전달/호스트 부하가 추가되므로 Nano에서 만든 지연 상한과 정책을 그대로 사용하지 않는다.

VM 측정용 `vm_measure.json`의 초기 센서 부하는 Nano 측정용과 같게 두었다. VM이 빠를 것이라고 가정해 해상도나 특징점 수를 자동으로 올리지 않는다. 실제 측정 후 각 설정을 조정한다. 설정을 조정한 실차 정책은 다시 생성해야 한다.

기존 저장소의 VM 운용 조건대로 실험 중 맥북 절전을 막고 뚜껑을 열어 둔다. 맥북 터미널에서 `caffeinate -s`를 켜 두고 USB를 직접 연결한다. 이 조건은 소프트웨어 감시만으로 호스트 전체 멈춤까지 해결할 수 없다는 점과 관련 있다.

## Nano의 VS Code 접속과 다른 점

VS Code Remote-SSH는 원격 컴퓨터/VM의 SSH 서버에 접속하여 그곳의 파일과 터미널을 사용하는 기능이다. 따라서 코드는 VM/Nano에서 실행되고, 맥북은 편집/조작/화면 표시를 담당한다. [Microsoft 공식 안내](https://code.visualstudio.com/docs/remote/ssh)

현재 Remote Development Linux 기본 조건에는 kernel 4.18 이상, glibc 2.28 이상, libstdc++ 3.4.25 이상 등이 포함된다. 기존 Ubuntu 24.04 VM과 달리 오래된 일반 Jetson Nano OS에서는 이 조건을 확인해야 한다. 공식 FAQ의 별도 sysroot 방법 등이 있지만 보드/OS를 확인하기 전에 시스템 glibc나 JetPack을 임의로 교체하지 않는다. 필요하면 맥북 VS Code에서 파일을 편집하고 일반 SSH 터미널로 실행/화면 전달하는 방법부터 사용할 수 있다. [Linux 조건](https://code.visualstudio.com/docs/remote/linux), [Remote FAQ](https://code.visualstudio.com/docs/remote/faq)

Nano 초기 설치는 다음에 사용자가 질문할 때 보드 모델 확인 → 전원/저장장치 → 해당 모델의 OS 설치 → 첫 부팅/네트워크 → SSH → SDK/드라이버 → 코드 전송 → 센서 측정 → CAN 확인 순서로 진행한다. 일반 Nano와 Orin Nano에 같은 OS 이미지를 쓰지 않는다.
