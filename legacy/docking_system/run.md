# 현장 명령 요약

## 1. 코드 업데이트 — 맥북

Jetson에서 측정/주행을 끝낸 뒤 실행. 기본 대상은 USB `192.168.55.1`이다.

```bash
bash /Users/seojeongmin/krri/docking_system/update_jetson.sh
```

공유기는 `--host jetson@192.168.0.27`. 네 버전과 공통 도구를 함께 보내고, Jetson의 기존 공통 설정·기록은 보존한다. 제거된 데모·중복 설정·이전 도구는 정해진 파일 목록에 한해 Jetson에서도 정리한다. **맥북에서 수정한 common.json도 보내려면 `--with-config`**를 붙인다. 최초 설치에는 옵션 없이 설정도 복사된다.

## 2. 준비·공통 입력 확인 — Jetson

```bash
source /home/jetson/.venvs/krri/bin/activate
cd /home/jetson/krri/docking_system
python preflight.py check
```

직접 입력할 파일: `configs/common.json`. 각 항목의 `value`를 수정하고 `description`은 설명으로 읽는다. 높이·폭·태그 위치·공간 등을 확인한다. 설명은 [CONFIG.md](CONFIG.md). 미완성 설정이면 check는 종료코드 2와 미입력 목록을 표시하며 장치는 열지 않는다.

## 3. 카메라 재장착 후 공통 장착각 측정

차량을 수평 바닥에 고정하고 **차체 정면을 태그 면에 수직으로** 맞춘다. 태그는 수직·윗변 수평. 카메라 화면 중앙에만 맞추는 것으로 대신할 수 없다.

```bash
python preflight.py mount --seconds 60 --view --camera-lateral-m 0
```

성공하면 v1~v4에 공통으로 자동 적용. 카메라 좌우 위치는 사용자 지정으로 중앙 장착(0m)을 가정한다. 실측값이 있으면 `--camera-lateral-m 0`의 0을 그 값으로 변경한다(m, 왼쪽 +). 이 옵션을 생략하면 좌우 위치는 미측정으로 남는다. IMU 초기 보정과 장착각 보정은 다르다. CAN을 열지 않는다.

## 4. 초기 보정·태그·VO 상태 점검

```bash
python preflight.py --version v1 inspect --seconds 60 --view
```

처음에는 카메라/차량을 고정하고 `IMU 초기 보정: READY` 확인. v2/v3/v4를 비교할 때는 `--version`만 바꾼다. 점검 결과는 공통 기록에 저장되며 CAN은 열지 않는다. 카메라만 연결한 현재 상태에서도 실행할 수 있다.

위 `inspect`는 별도 진단이 필요할 때 사용한다. 실주행 자체에서도 Enter 후 IMU 초기 보정과 출발 전 정지 측정을 수행하므로, 그 측정만을 위해 매번 inspect를 먼저 실행할 필요는 없다.

CAN도 연결한 현장에서 수신 상태 점검:

```bash
python preflight.py devices --seconds 5
```

이 검사는 CAN 수신 전용이며 주행/중립 명령을 보내지 않는다. VM에서는 `python preflight.py --platform vm devices`처럼 사용한다.

## 5. 카메라 화면만 계속 보기

```bash
cd /home/jetson/krri/docking_system/robust_docking_v1
bash scripts/run_nano.sh measure --no-odom --view
```

태그도 검사하지 않으려면 `--no-tag` 추가. Ctrl+C 종료. VS Code 포트 탭에서 **8765** 전달 → 맥북 브라우저 **http://127.0.0.1:8765**. Windows App은 필요 없다.

## 6. 실주행 — 현장 값·검증을 완료한 뒤

**현재 네 버전 모두 주행 필수 입력/검증이 남아 있다. 아래 명령만으로 출발 가능한 상태가 되는 것은 아니다.** v1 예시이며 v2~v4는 폴더와 출력 이름을 함께 바꾼다.

```bash
cd /home/jetson/krri/docking_system/robust_docking_v1
mkdir -p /home/jetson/krri/work/docking_system/robust_docking_v1
python -m docking prepare --config configs/hardware.json --site configs/sites/kw_wall_stop.json --task wall_stop --output /home/jetson/krri/work/docking_system/robust_docking_v1/drive_config.json
```

생성한 `drive_config.json`은 공통 값을 반영한 고정 사본이다. 해당 버전의 실제 검증 결과·승인을 반영한 뒤 아래 설정 검사에서 `drive_ready: true`를 확인한다. 공통 설정/장착을 변경하면 이 사본과 정책을 다시 만든다.

```bash
python -m docking check-config --config /home/jetson/krri/work/docking_system/robust_docking_v1/drive_config.json
python -m docking synthesize --config /home/jetson/krri/work/docking_system/robust_docking_v1/drive_config.json --output /home/jetson/krri/work/docking_system/robust_docking_v1/policy.json
bash scripts/run_nano.sh drive --config /home/jetson/krri/work/docking_system/robust_docking_v1/drive_config.json --policy /home/jetson/krri/work/docking_system/robust_docking_v1/policy.json --view --execute
```

차량을 정지시키고 주변 확인 후 터미널 Enter로 시작 요청. **IMU 준비 → 연속 정지 측정 60초 → 태그 확보/주행 조건 확인 → 출발** 순서로 자동 진행한다. 준비 중에는 초기 탐색 회전도 하지 않는다. 터미널과 화면에 측정 진행 시간을 표시하며, 움직임/센서 문제로 유효 구간이 끊기면 다시 모은다. 전체 대기 제한 180초 또는 보정 실패 때는 출발하지 않는다. 정지는 준비 중에도 Ctrl+C 또는 `s` + Enter다.

공통 `values.startup_measurement.duration_s`를 30으로 바꾸면 정지 측정 30초. 변경 후 위 prepare/정책 생성을 다시 수행한다. 결과는 해당 실행 폴더의 `startup_measurement.json`에 저장되며, 차량 제동·주행 오차의 미입력값을 자동으로 채우지는 않는다.

좌측 태그 주차는 준비 명령의 site를 `configs/sites/kw_park_left_tag.json`, task를 `cone_entry`로 바꾼다. VM은 `hardware_vm.json`, `run_vm.sh`를 쓰고 호스트 지연을 별도 확인한다.

공통 보고서: `docking_system/calibration/`. 상세 기록: `krri/work/docking_system/robust_docking_vN/`. 버전별 설정 사본을 갱신해서 읽고 싶으면 `docking_system`에서 `python preflight.py sync`를 실행한다. 실행 시 원본을 읽으므로 보통 별도 sync는 필요 없다.
