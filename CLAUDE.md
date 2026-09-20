# KRRI AprilTag 지게차 도킹 — 작업 규칙 (Claude 가 매 세션 읽는다)

카메라로 AprilTag 을 보고 탑재부 기준 지게차 위치를 낸 뒤 그 값으로 도킹한다.

## 저장소 규칙 (이걸 틀리면 안 된다)
- **활성 개발 = `apriltag_v3/`** (v2 를 복사해 시작, 연속 추정·수렴 제어로 재설계 중).
  **`apriltag_v1/`·`apriltag_v2/` 는 동결 — 읽기만, 절대 수정 금지**(v1 사이드스텝, v2 조준-전진 lateral_no; 비교·참고용).
  v3 코드도 토론 단계에선 읽기만 하고, plan.md 확정 후 사용자가 시킬 때 `coder` 로만 수정한다.
- **브랜치**
  - **평소 작업 = `jm_mac`.** 맥북·우분투용 개발·환경 변경은 전부 여기서 하고, 이 맥북은 항상 jm_mac 에 체크아웃.
  - `jm` : 사용자의 로컬(연구실) 컴퓨터 전용. **평소 흐름에서 배제** — 자동으로 당기지 않는다. 사용자가
    "jm 에서 이 부분 고쳤다, jm_mac 에 합칠지 보자" 고 가져올 때만 확인·논의 후 `git merge origin/jm`.
  - VM(`ubuntu-vm`) : `jm_mac` 을 받아 **쓰기만** 한다. VM 안에서 커밋하지 않는다.
    **`git commit` · `git push origin jm_mac` · VM 반영 — 셋 다 자동으로 하지 않는다.** 사용자가 각각 시킬 때만.
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

## v3 설계 토론 팀 (`apriltag_v3/Agent/`, 로컬 전용·git 무시)
- 역할·권한·흐름·로그 형식은 **`apriltag_v3/Agent/README.md`** — 토론 관련 작업 전에 먼저 읽는다.
  이전 세션의 분석 결론·합의·리스크는 아래 **"설계 현황·합의"** 절.
- **"토론 시작해"** = `Workflow({scriptPath: "apriltag_v3/Agent/debate_workflow.js", args: {n: N}})` 실행.
  N = 1 Perception / 2 Vehicle Dynamics / 3 Controller / 4 System. 다음 N 은 `Agent/plan.md` 에서 `pending` 인 첫 대문제.
  이 문구는 사용자의 명시적 워크플로(다중 에이전트) 실행 요청이다.
- 끝나면 결과를 `Agent/log/debate_N_<key>.md`(쟁점당 6~8줄, 핵심만) 와 `Agent/plan.md` 해당 절(결론·검증 실험·파라미터 자리만)에
  쓰고, **PushNotification 보낸 뒤 멈춰서 사용자 검토**를 받는다. 대문제 하나씩. 근거 없는 결론은 plan 에 넣지 않는다.
- 토론 단계에선 `apriltag_v3` 코드를 **읽기만** 한다. 쓰기는 `Agent/` 안에서만. v1·v2 는 절대 수정 금지(기존 규칙).
- 알림: 대문제 완료 / 한도 대기 진입·재개 / 에이전트 실패 / 결정·권한 대기 때 PushNotification.
  한도 소진 시 리셋까지 자동 대기·재개(`.claude/settings.json` autoContinueAtUsageLimit) — 크레딧 구매 아님.
- 모델: 세션 Fable 5.1[1m]·effort xhigh → 불가 시 Opus 5[1m] 자동 폴백(settings.json). 토론 서브에이전트는 effort max
  (`debate_workflow.js` 의 `call()` 이 Fable→Opus 재시도까지 담당).
- 코드 작성은 plan.md 확정 후 사용자가 시킬 때 `coder` 에이전트로 v3 만 수정. main 반영은 사용자가 말할 때만(기존 규칙).

## 설계 현황·합의 (2026-09-20 기준 — 토론·코드 작업의 전제)
**9/7 로그 결론** (v1 11회 전부 실패, v2 2회 중 1회 성공·21스텝; 로그는 맥 `work_dirs/` 로컬에만):
- 뿌리: **7~10m lateral 잡음 60~145mm(최대 415) > 허용치 30mm** → 유령 오차 추종 → 좌우 왕복.
- 잡음 정체: lateral 을 전체 자세 R(yaw)에서 뽑아 **yaw×거리**로 증폭(8m·1°=140mm). 정면 heading 튐은 **PnP 2중해 flip**(중앙값으로 안 잡힘).
- 구조: **stop-and-go**(정지→30프레임 눈감고→동작 1개 개루프→정지), 프레임 간 기억 없음. 회전이 lateral 을 움직임(1.46m·10°=25cm).
- 직진 명령시간→거리는 **죽은시간+선형**(시그모이드 아님). <2s 명령 CV 100%+, ≥3s ~10%. 정지는 관성 coast. 회전은 ±2° 폐루프 + 드문 대형실패.
**합의된 설계 전제** (agent 프롬프트·`debate_workflow.js` PROBLEMS 에도 박힘):
- 사이드스텝(90°) 폐기 → **태그를 화면 중앙(β≈0)에 유지하며 소각 대각 접근**. 곡선(조향+전진 동시)은 안 함.
- **멈추지 말고 연속 추정·보정**(검출↔판단 사이 추정기 레이어). 30프레임 중앙값은 정지 후 확인용.
- 자이로는 **짧은 다리만**(회전 정지 판정·최종 정면 몇 초). 카메라가 절대 기준. 초기 탐색엔 자이로 기억 불필요.
- 컨트롤러는 `control_forklift_v2.py` 사용; `control_광운대.py` 는 참고용(실행 안 함) — 저속 97·entry-burst 는 **이식** 대상.
**리뷰에서 짚은 리스크** (토론에서 반드시 다룸):
- v̂(속도추정)가 정지 예측의 린치핀인데 **저속(0.15~0.2m/s)에서 프레임당 5~7mm ≈ 잡음 ±9mm** → 명령버퍼 prior 또는 마지막 몇 cm depth 정지.
- 1-step candidate 컨트롤러는 **대각 lateral 보정을 못 찾음**(이득이 미래) → 2~N step 지평 또는 T-조준 가이드.
- "태그 화면 안 유지" 제약이 controller 문서에 없음 → cost/하드 제약 추가. dynamics 모델 의존은 작은 step+재관측으로 구제.
- 과설계 보류: 온라인 τ/a 적응, 펄스표·학습펄스, R(d,φ) 5+5 모델, 4중 yaw 게이트 — **필요 증명 후**. docs 2·3(dynamics) 통합 권장.
**코드 구조(v3=v2 복사, 코드 변경 0)**: image→detection_tag→detection_pose(`docking_state`·`measure`)→control_from_pose(`dock_live`·`plan_aim`)→
CanDriver → 회전 `rotate_by()`(래퍼)→`rot_control.rotate_to()`(IMU 폐루프 엔진) / 직진 `fwd_time_model`(개루프) / CAN `control_forklift_v2`.
`--record-events` 는 run 폴더에 `config.json`(control/detection/imu) 스냅샷도 남김.
**미확정**: 탑재부 허용오차(LAT_TOL_M 0.030/HEAD_TOL_DEG 2.0 임시), byte2 비례 여부·97 의 m/s, 최소 제어량·데드존, σ_τ. 다음 실차 1순위 =
byte2 스윕·σ_τ·정적 R(d,φ)·프레임별 bag. 토론은 아직 0개 완료. (PLAN_legacy 의 9/14 2차 실험 실시 여부 미확인.)

## 코드 갱신 (**커밋·푸시·VM 반영 전부 사용자가 시킬 때만**)
파일 수정은 작업 트리(jm_mac 체크아웃)에서만 한다. **`git commit`, `git push origin jm_mac`, VM 반영 — 셋 다 자동으로 하지 않는다.**
- "커밋해" → 커밋만 (푸시 안 함)
- "푸시해" → `git push origin jm_mac` (VM 안 함)
- "우분투로 보내줘 / 받아와" → 아래 두 명령
각각 그렇게 말할 때만 실행한다:
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
- 실측(2026-09-07): 회전 팔 A=1.46m(카메라→회전중심), 직진 정속 ~0.28~0.30m/s, 출발 지연 ~1s(주행)/~0.85s(회전, ROT_T0),
  정지 지연 ~0.5s → 정지 관성 ~12~14cm, 회전 관성 ~1.3~1.5°. 저속 전진은 byte2=97(광운대 `forward_slow`, 데드밴드 위) — m/s 미측정.
- Dropbox `철기원…/코드/control_forklift_v2.py` 는 **옛 버그 버전**(rotate=byte4/5 → 포크 올라감). 실차엔 레포 v3 것만.

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
