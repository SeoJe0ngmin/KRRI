# forklift-truck_dockingv2 — 실행 명령

실행 명령과 현장 순서만 적는다. 설명은 README.md.

## 1. 현장 순서

화면: VS Code 가 8765 포트를 넘겨 주므로 맥 브라우저에서 `http://127.0.0.1:8765`. 모든 명령은 `cd /home/jetson/krri/forklift-truck_dockingv2` 에서. 끝낼 땐 Ctrl+C 한 번.

**0. 현장 값** — 맥 `configs/common.json` 의 `site` 를 줄자 값으로 고치고 배포. 뭘 재야 하는지는 바로 아래 2절 표. 카메라를 옮겼으면 아래 2. 장착각까지.

**1. 점검** (카메라 필요, CAN 불필요)
```bash
bash scripts/preflight.sh check --sensor-only        # 설치·장치
bash scripts/run_nano.sh measure --seconds 90 --view # IR 에 OpenVINS 점이 바닥·벽에 찍히고 tag: accepted 가 유지되는지. 점이 포크·마스트에 찍히면 안 됨
grep "\[pipeline\]" $(ls -td work/*/measure/* | head -1)/openvins.log | tail -1   # dropped=0 이면 VIO 정상
```
카메라만 볼 때 `--camera-only`, 패널 고르기 `--panels color` / `--panels ir,depth`. 도킹 앱을 끈 상태에서 버드아이뷰만 보려면 `source scripts/environment.sh && "$DOCK_PYTHON" scripts/bev_test.py` → `http://127.0.0.1:8082`.

**2. 장착각** (카메라를 달거나 옮겼을 때, 또는 태그 높이·크기·ID 를 바꿨을 때)
차를 태그 **정면**에 3.3~4 m 두고 선다. 좌우 치우침은 상관없고 차 진행축이 탑재부 축과 평행이면 된다.
```bash
bash scripts/run_nano.sh measure --view                                            # 30초 넘게 가만히, Ctrl+C
bash scripts/preflight.sh mount --folder $(ls -td work/*/measure/* | head -1) --write   # 장착각 → configs/mount.json
bash scripts/preflight.sh check --mode left                                        # 현장값·장착각 서명 검사. 여기서 거부되면 drive 도 거부
```

**3. 리허설** (CAN 없이 흐름만)
```bash
bash scripts/run_nano.sh shadow --mode left --view   # READY 뒤 Enter → 탐색 → 30초 정차 → 첫 명령까지. 차가 안 움직이니 motion_time_limit 로 끝나는 게 정상
```

**4. 실주행** (사람은 브레이크 자리)
```bash
bash scripts/run_nano.sh drive --mode left --execute --view   # READY 뒤 Enter. 정지 s+Enter / Ctrl+C
```
흐름·화면 읽는 법은 README.md. 터미널 줄의 `상태/단계` 와 대시보드 단계 타일로 진행을 본다.

## 2. 새 현장에서 common.json 에 바꿀 값

**차에 붙은 값은 그대로, 현장(탑재부·태그) 값만 다시 잰다.** `configs/common.json` → `site`, 고친 뒤 배포.

| 구분 | 항목 | 메모 |
|---|---|---|
| 현장마다 | `truck_inner_width_m` | 두 옆판 안쪽 폭. depth 입구 찾기의 폭 조건(±`truck_width_tol_m`) |
| | `truck_depth_m` | 입구면→안쪽 막힌 곳. 비우면 검사 없음 |
| | `target_front_x_left_m` | 포크 앞끝이 멈출 자리(입구면 기준, 안쪽 +). 깊이 − 5 cm 안이어야 출발 |
| | `tag_height_m` | 태그 **중심** 높이. 바꾸면 장착각 재측정 |
| | `left_tag_offset_m` | 입구 중심→태그 중심 좌우. 왼쪽 +, 오른쪽 − |
| | `tag_plane_offset_m` | 입구면↔태그면 앞뒤(안쪽 +). 입구면에 붙이면 0 |
| | `rear_space_m` | 차 뒤 빈 공간(후진 상한) |
| | `forward_path_bias_deg` | 비우고 시작. 들어간 뒤 한쪽으로 쏠리면 흐른 각(왼쪽 +) |
| | `tag_id` `tag_size_m` | 태그판을 바꿀 때만 |
| 차에 붙은 값 | `vehicle_width_m` `vehicle_length_m` `rotation_center_*` `rear_extent_m`(비움) `target_front_x_central_m` | 그대로 |
| 카메라 옮겼을 때만 | `camera_height_m` `camera_lateral_m` `camera_to_fork_tip_m` | 좌측 IR 렌즈 기준(컬러 렌즈로 쟀으면 왼쪽 1.48 cm 차이). 바꾸면 1절의 2. 장착각 |

`operation`(시간 한도·허용 오차)은 현장과 무관. `mount.json` 은 카메라·태그 값이 안 바뀌면 그대로이고, 바뀌면 `preflight.sh check` 가 재측정을 요구한다.

## 3. 연결

```
# ~/.ssh/config (맥)
Host jetson-wired        # USB 직결
    HostName 192.168.55.1
    User jetson
    IdentityFile ~/.ssh/id_ed25519_vm
Host jetson-kw           # 광운대 핫스팟
    HostName 172.20.10.4
    User jetson
    IdentityFile ~/.ssh/id_ed25519_vm
```

- 이 문서는 `jetson-wired` 로 적는다. 핫스팟이면 `jetson-kw` 로 바꿔 친다.
- `ssh jetson-wired '…'` 로 시작하는 줄은 **맥 터미널**에서, `cd /home/jetson/…` 로 시작하는 줄은 **Jetson 터미널(VS Code)** 에서 친다.
- **시계**: Jetson 은 껐다 켜면 1970년이다. 맥에서 ssh·VS Code·배포로 한 번 붙으면 자동으로 맞춰진다(맥 `~/.ssh/config` 의 `LocalCommand` + Jetson `/etc/sudoers.d/clock`, 설정 완료). 맥이 아닌 컴퓨터에서는 Jetson 터미널에서 `sudo date -s "2026-10-24 09:30:00"` 처럼 직접 맞춘다. 안 맞으면 로그가 `work/19700101/` 에 쌓일 뿐 주행엔 영향 없다.
- **새 핫스팟**: ① Jetson(한 번) USB 로 들어가 `sudo nmtui` 또는 `sudo nmcli dev wifi connect "SSID" password "비밀번호"`, `hostname -I` 로 IP 확인 ② 맥 `~/.ssh/config` 에 `Host jetson-<이름>` 항목(HostName = 그 IP). IP 가 바뀌면 HostName 만 고친다. 키·sudoers 는 다시 안 한다.

## 4. 맥 → Jetson 배포

```bash
bash /Users/seojeongmin/krri/forklift-truck_dockingv2/update_jetson.sh --host jetson-wired              # 바뀐 파일만 보냄
bash /Users/seojeongmin/krri/forklift-truck_dockingv2/update_jetson.sh --host jetson-wired --dry-run    # 보낼 파일·common.json 차이만 보기
```

- `configs/common.json` 은 **맥 것이 Jetson 을 덮는다**(주인은 맥). 덮기 전에 다른 줄을 보여 준다.
- `work/`(로그)와 `configs/mount.json`(장착각)은 보내지도 지우지도 않는다. mount.json 을 새로 넣을 땐 `scp /Users/seojeongmin/krri/forklift-truck_dockingv2/configs/mount.json jetson-wired:/home/jetson/krri/forklift-truck_dockingv2/configs/mount.json`.
- 파이썬·설정만 바뀌었으면 배포로 끝. `ros2/`(C++)가 바뀐 날만 아래 재빌드.

## 5. Jetson 설치·재빌드

```bash
cd /home/jetson/krri/forklift-truck_dockingv2
bash scripts/install_jetson.sh            # 새 Jetson 에 한 번: ROS Jazzy·가상환경 ~/.venvs/krri·OpenVINS+브리지 빌드 (30~60분, 인터넷 필요)
bash scripts/rebuild_openvins.sh          # C++ 브리지가 바뀌었거나 폴더 이름을 바꿨을 때: 우리 패키지 둘만 재컴파일 (몇 분)
```

빌드와 가상환경은 프로젝트 밖(`~/.local/share/krri/ros_ws`, `~/.venvs/krri`)에 있어 폴더를 지우거나 다시 배포해도 남는다. 재빌드를 안 하면 "OpenVINS 브리지 업데이트 필요" 로 스스로 멈춘다.

## 6. 로그

판 폴더: `work/<날짜>/<measure|shadow|drive>/<시각>_<id>_<명령>/`. 안에 `observations.jsonl`(관측·depth 매 프레임), `control.jsonl`(제어 상태·CAN; `status.events` 에 sidestep·bundle·final_check_ok·entry·step_done), `session.json`(끝난 이유), `openvins.log`.

```bash
ls -td /home/jetson/krri/forklift-truck_dockingv2/work/*/*/* | head -1                                                   # 가장 최근 판
scp -r jetson-wired:/home/jetson/krri/forklift-truck_dockingv2/work/<날짜> /Users/seojeongmin/krri/forklift-truck_dockingv2/work/                                     # 그날 전부 맥으로
grep "\[pipeline\]" $(ls -td /home/jetson/krri/forklift-truck_dockingv2/work/*/drive/* | head -1)/openvins.log | tail -1   # dropped=0·imu_lost 수십 이하면 VIO 정상
```

큰 폴더를 Jetson 과 주고받을 땐 rsync 가 깨질 수 있어 tar 를 쓴다: `ssh jetson-wired 'cd /home/jetson && tar czf - krri' > jetson_krri.tar.gz` / `tar czf - 폴더 | ssh jetson-wired 'tar xzf - -C /home/jetson/krri'`.

## 7. 그 외

**와이파이(핫스팟) 설정** — Jetson 터미널에서 (USB 직결로 들어가서)
```bash
sudo nmtui                                              # 화면: "Activate a connection" → 망 선택 → 비밀번호
sudo nmcli dev wifi connect "SSID" password "비밀번호"   # 한 줄 버전
hostname -I                                             # 받은 IP → 맥 ~/.ssh/config 의 HostName 에
```

**CPU·온도**
```bash
ssh jetson-wired 'for z in /sys/class/thermal/thermal_zone*; do t=$(cat $z/temp 2>/dev/null) && echo "$(cat $z/type): $((t/1000))°C"; done'   # 온도 한 번
ssh -t jetson-wired 'tegrastats --interval 2000'      # CPU %·온도·전력 계속 (Ctrl+C)
sudo jetson_clocks                                    # Jetson 에서, 실주행 때 클럭 고정 (재부팅까지)
```
CPU 70 °C 를 넘으면 처리 fps 가 떨어질 수 있다.

## 8. 옵션

**`run_nano.sh <measure|shadow|drive>`**

| 옵션 | 뜻 |
|---|---|
| `--mode left` | 태그가 입구 옆에 있을 때(왼쪽·오른쪽 모두). 좌우는 `left_tag_offset_m` 부호. 기본이 central 이라 매번 쓴다 |
| `--view` / `--port 8765` | 대시보드 켬 / 포트 |
| `--panels all` | 카메라 패널: `ir` `color` `depth` 쉼표 조합. 끈 패널은 만들지도 보내지도 않음 |
| `--execute` | drive 에서 CAN 을 진짜 보냄(없으면 거부). 대화형 터미널 필요 |
| `--seconds 60` | N 초 뒤 자동 종료 |
| `--camera-only` / `--no-tag` | measure: OpenVINS 없이 / 태그 검출 끔 |
| `--output 경로` / `--common 경로` | 기록 폴더 / common.json 직접 지정 |

**`preflight.sh <check|mount>`**: `--mode left`(check) · `--sensor-only`(설치·장치만) · `--camera-only` · `--folder 폴더`(mount: measure 판) · `--write`(mount: mount.json 저장)

**`update_jetson.sh`**: `--host jetson-wired|jetson-kw` · `--dry-run`

**`scripts/bev_test.py`** (도킹 앱 끈 상태): `--port 8082` · `--width 1.4` · `--width-tol 0.2` · `--emitter` · `--no-filter` · `--decay 0.7` · `--save 폴더 --every 2` · `--pitch 2` · `--step 2` · `--fps 15` · `--mode left`

## 9. 기타 점검

```bash
cd /Users/seojeongmin/krri/forklift-truck_dockingv2 && PYTHONPATH=.:tests python3 -m unittest discover -s tests -q     # 맥에서 코드 검사(장치 불필요)
source ~/.venvs/krri/bin/activate && python scripts/check_imu_cadence.py full --seconds 60   # Jetson, 앱 끈 상태: IMU·영상 도착 간격(USB 의심될 때만)
```
