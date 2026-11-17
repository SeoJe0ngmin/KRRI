# docking — 실행 명령

실행 명령과 현장 순서만 적는다. 설명은 README.md.

## 1. 현장 순서

화면: VS Code 가 8765 포트를 넘겨 주므로 맥 브라우저에서 `http://127.0.0.1:8765`. 모든 명령은 `cd /home/jetson/krri/docking` 에서. 끝낼 땐 Ctrl+C 한 번.

**0. 현장 값** — 맥 `configs/common.json` 의 `site` 를 줄자 값으로 고치고 배포. 뭘 재야 하는지는 바로 아래 2절 표. 카메라를 옮겼으면 아래 2. 장착각까지.

**1. 점검** (카메라 필요, CAN 불필요)
```bash
bash scripts/preflight.sh check --sensor-only        # 설치·장치
bash scripts/run_nano.sh measure --seconds 90 --view # IR 에 OpenVINS 점이 바닥·벽에 찍히고 tag: accepted 가 유지되는지. 점이 포크·마스트에 찍히면 안 됨
grep "\[pipeline\]" $(ls -td work/*/measure/* | head -1)/openvins.log | tail -1   # dropped=0 이면 VIO 정상
```
카메라만 볼 때 `--camera-only`, 패널 고르기 `--panels color` / `--panels ir,depth`. 도킹 앱을 끈 상태에서 depth 가 문 세로선·격자를 어떻게 잡는지만 보려면 `source scripts/environment.sh && "$DOCK_PYTHON" scripts/bev_test.py` → `http://127.0.0.1:8082` (융합 위치가 없어 창을 앞뒤로 훑어 찾는다; `--dist 2.6` 처럼 입구까지 거리를 주면 그 자리만 본다).

**2. 장착각** (카메라를 달거나 옮겼을 때, 또는 태그 높이·크기·ID 를 바꿨을 때)
차를 태그 **정면**에 3.3~4 m 두고 선다. 좌우 치우침은 상관없고 차 진행축이 탑재부 축과 평행이면 된다.
```bash
bash scripts/run_nano.sh measure --view                                            # 30초 넘게 가만히, Ctrl+C
bash scripts/preflight.sh mount --folder $(ls -td work/*/measure/* | head -1) --write   # 장착각 → configs/mount.json
bash scripts/preflight.sh check                                                    # 현장값·장착각 서명 검사. 여기서 거부되면 drive 도 거부
```

**3. 리허설** (CAN 없이 흐름만)
```bash
bash scripts/run_nano.sh shadow --view   # READY 뒤 Enter → 탐색 → 30초 정차 → 첫 명령까지. 차가 안 움직이니 motion_time_limit 로 끝나는 게 정상
```

**4. 실주행** (사람은 브레이크 자리)
```bash
bash scripts/run_nano.sh drive --execute --view   # READY 뒤 Enter. 정지 s+Enter / Ctrl+C
```
끝나면 터미널 마지막 줄에 `완료 | 좌우 ±x.xx m | 방향 ±x.x° | 남은 x.xx m | <끝난 이유>` 가 남는다(프롬프트 위 보통 줄). 흐름·화면 읽는 법은 README.md.

## 2. 새 현장에서 common.json 에 바꿀 값

**차에 붙은 값은 그대로, 현장(탑재부·태그) 값만 다시 잰다.** `configs/common.json` → `site`, 고친 뒤 배포. 입구 가로선 = 문 안쪽(경첩 쪽) 세로선 둘을 잇는 선 = 태그면. 모든 "안쪽 +" 는 이 선 기준.

| 구분 | 항목 | 메모 |
|---|---|---|
| 현장마다 | `truck_inner_width_m` | **문 안쪽 세로선 사이 폭 1.65** = depth 가 입구로 찾는 쌍의 폭 = 충돌 검사 폭. 허용 ±`truck_width_tol_m` 0.2(0.1 고려) |
| | `truck_depth_m` | **입구 가로선 → 격자 프레임 앞면** (임시 2.5, 줄자로 확정). 격자 쌍을 찾을 자리·진입 중 두 번째 자(격자까지 거리)·대시보드 그림에 쓴다. 비우면 격자 기능 전부 꺼짐 |
| | `lattice_pair_width_m` | 격자 **2번·3번 칸 경계 세로살의 마주보는 안쪽 면 사이** 0.77(= 36.5 + 4 + 36.5, 재측정). ±`lattice_pair_tol_m` 0.1. 비우면 격자 기능 꺼짐 |
| | `lattice_centre_offset_m` | 차로 중심 − 2\|3 중심 (왼쪽 +). 차로 중심이 2\|3 중심보다 3.5 cm 왼쪽 → 0.035 |
| | `target_front_x_left_m` | 포크 앞끝이 멈출 자리(입구 가로선 기준 안쪽 +) 1.5. `truck_depth_m` − 5 cm 안이어야 출발 |
| | `tag_height_m` | 태그 **중심** 높이. 바꾸면 장착각 재측정 |
| | `left_tag_offset_m` | 입구 중심→태그 중심 좌우. 왼쪽 +, 오른쪽 − (−0.785) |
| | `tag_plane_offset_m` | 입구 가로선↔태그면 앞뒤(안쪽 +). 태그가 문 세로선 선상이면 **0** |
| | `rear_space_m` | 차 뒤 빈 공간(후진 상한) |
| | `forward_path_bias_deg` | 비우고 시작. 들어간 뒤 한쪽으로 쏠리면 흐른 각(왼쪽 +) |
| | `tag_id` `tag_size_m` | 태그판을 바꿀 때만 |
| 차에 붙은 값 | `vehicle_width_m` `vehicle_length_m` `rotation_center_*` `rear_extent_m`(비움) | 그대로 |
| 카메라 옮겼을 때만 | `camera_height_m` `camera_lateral_m` `camera_to_fork_tip_m`(포크 접힘 0.07 = 카메라→차 맨 앞) | 좌측 IR 렌즈 기준(컬러 렌즈로 쟀으면 왼쪽 1.48 cm 차이). 바꾸면 1절의 2. 장착각 |

`operation`(시간 한도·허용 오차)은 현장과 무관. `mount.json` 은 카메라·태그 값이 안 바뀌면 그대로이고, 바뀌면 `preflight.sh check` 가 재측정을 요구한다.

**docking 설계 (2026-10-30, v3 베이스).** 기준물을 문 안쪽 세로선 쌍(입구) + 안쪽 격자 프레임(2|3 세로살 쌍 = 좌우, 아래 가로살 = 거리·축)으로 바꾸고 X 프레임·벽 축·central 모드를 뺐다. 흐름은 그대로: 러프(−3.6 정지) → 태그 재영점 → 세밀(묶음 보정) → 마지막 확인(입구 0.6 m 앞, `final_check_distance_m`) → 진입. **정지 창마다 "가장 정확한 것 하나"를 쓴다**: 좌우 = 태그(1 s 안에 본 태그 창, `tag_fresh_s`) → 격자 쌍(좋을 때: 4 프레임 이상 `lattice_good_frames`, 창 안 흔들림 ≤ 2 cm `lattice_good_spread_m`, 거리 ≤ 3.5 m `lattice_good_range_m`) → 문 쌍 → 끌고 온 값(VIO); 거리 = 태그 → 격자 가로살 선(좋을 때) → **격자 2·3 쌍 중심 − `truck_depth_m`**(좋을 때; 10-30 082754 판: 가로살은 점 부족으로 0건이라 이게 실제 격자 거리) → 문 쌍 → 끌고 온 격자 점 → 끌고 온 중심(항상 입구면 위의 점으로 저장); 방향 = 태그로 영점 잡은 VIO, 격자 선 축은 연속 2 정지에서 같은 방향으로 1° 넘게 어긋날 때만 넘겨받음(`lattice_axis_adopt_deg`/`_stops`, `lattice_axis_enabled` 0 이면 기록만). 평균·중앙값 섞기 없음, 다른 기준끼리 3 cm 넘게 다르면 `lateral_sources_disagree` 로 **기록만**. depth 로 잰 값은 `anchor_requested` 로 융합 위치를 그 자리로 당긴다(태그 창이 하듯). 창: 입구 ±`entrance_window_m` 0.2, 격자 ±`lattice_window_m` 0.35, 띠 0.45~0.72(문 세로선·격자 세로살), 격자 가로살 띠 0.33~0.43, 문 최대 `max_m` 4.0, 격자 최대 `lattice_max_m` 6.5. **멈추지 않는 원칙**: 보정 횟수 소진·후진 공간 없음 → `best_effort_entry`(똑바로 세운 뒤 도착점을 겨냥해 진입, 잔여 오차 기록); 좌우 오차 남은 채 목표 도달 → COMPLETE + 기록(FAULT 아님); 유일한 정지는 벽 충돌 예측 `truck_side_clearance`. **포크 끝이 입구선을 넘은 뒤에는 제자리 회전 금지**(`turn_suppressed_inside` 기록; 직진·후진만, 후진해서 나오면 다시 허용 — 10-30 082754 판에서 안쪽 0.8 m 에서 1.1° 회전한 뒤 추가). VIO 를 잃어도 RECOVER 에서 태그 → 문 → 격자 순으로 위치를 다시 잡는다(`anchor_requested_absolute`). 진입은 VIO 자와 격자까지 거리 자 중 **덜 들어가는 쪽**에서 선다(`lattice_remaining_m`). **프로젝터** `projector_when_stopped` 1(기본): 서 있을 때만 0.3 s 뒤 켜고 움직이기 전에 끈다, 0 이면 항상 끔. **약한 명령**: 러프 뒤 15° 이하 회전과 묶음 드라이브·후진은 약한 바이트(`weak_rotate_delta` 10, `weak_forward_byte` 97), 긴 직진은 보통 바이트; 안 움직이면 보통 바이트로 다시(`weak_command_deadband`), 학습값 저장 없음.

## 3. 연결

```
# ~/.ssh/config (맥)
Host jetson-ap           # 젠슨이 직접 쏘는 와이파이 (AP 모드) — 2026-10-29부터 기본
    HostName 10.42.0.1
    User jetson
    IdentityFile ~/.ssh/id_ed25519_vm
Host jetson-wired        # USB 직결
    HostName 192.168.55.1
    User jetson
    IdentityFile ~/.ssh/id_ed25519_vm
Host jetson-kw           # 광운대 핫스팟
    HostName 172.20.10.4
    User jetson
    IdentityFile ~/.ssh/id_ed25519_vm
```

- 이 문서는 `jetson-ap` 로 적는다. 유선이면 `jetson-wired`, 핫스팟이면 `jetson-kw` 로 바꿔 친다. `jetson-ap` 항목이 맥 `~/.ssh/config` 에 아직 없으면 위 블록을 직접 붙여 넣는다(자동 편집이 막혀 있음).
- **젠슨 AP**: 젠슨이 `jetson-ap`(비번 12345678, 10.42.0.1) 와이파이를 직접 쏜다. 맥은 그 와이파이에 붙고 `--host jetson-ap`. 만든 법(젠슨, 한 번): `sudo nmcli dev wifi hotspot ifname wlP1p1s0 con-name jetson-ap ssid jetson-ap password "12345678"` → `sudo nmcli con modify jetson-ap ipv4.method shared ipv4.addresses 10.42.0.1/24 connection.autoconnect yes connection.autoconnect-priority 50`. 부팅하면 AP 가 먼저 뜬다. 인터넷이 필요하면 `sudo nmcli con down jetson-ap && sudo nmcli con up 경원`, 끝나면 `sudo nmcli con up jetson-ap`. AP 동안은 맥도 인터넷이 안 된다.
- `ssh jetson-ap '…'` 로 시작하는 줄은 **맥 터미널**에서, `cd /home/jetson/…` 로 시작하는 줄은 **Jetson 터미널(VS Code)** 에서 친다.
- **시계**: Jetson 은 껐다 켜면 1970년이다. 맥에서 ssh·VS Code·배포로 한 번 붙으면 자동으로 맞춰진다(맥 `~/.ssh/config` 의 `LocalCommand` + Jetson `/etc/sudoers.d/clock`, 설정 완료). 맥이 아닌 컴퓨터에서는 Jetson 터미널에서 `sudo date -s "2026-10-24 09:30:00"` 처럼 직접 맞춘다. 안 맞으면 로그가 `work/19700101/` 에 쌓일 뿐 주행엔 영향 없다.
- **새 핫스팟**: ① Jetson(한 번) USB 로 들어가 `sudo nmtui` 또는 `sudo nmcli dev wifi connect "SSID" password "비밀번호"`, `hostname -I` 로 IP 확인 ② 맥 `~/.ssh/config` 에 `Host jetson-<이름>` 항목(HostName = 그 IP). IP 가 바뀌면 HostName 만 고친다. 키·sudoers 는 다시 안 한다.

## 4. 맥 → Jetson 배포

```bash
bash /Users/seojeongmin/krri/docking/update_jetson.sh --host jetson-ap              # 바뀐 파일만 보냄
bash /Users/seojeongmin/krri/docking/update_jetson.sh --host jetson-ap --dry-run    # 보낼 파일·common.json 차이만 보기
```

- `configs/common.json` 은 **맥 것이 Jetson 을 덮는다**(주인은 맥). 덮기 전에 다른 줄을 보여 준다.
- `work/`(로그)와 `configs/mount.json`(장착각)은 보내지도 지우지도 않는다. **처음 배포하는 폴더**라 mount.json 이 없으면 v3 것을 복사한다: `ssh jetson-ap 'cp /home/jetson/krri/forklift-truck_docking_BirdsEyeView_v3/configs/mount.json /home/jetson/krri/docking/configs/'` (맥에 있으면 `scp /Users/seojeongmin/krri/docking/configs/mount.json jetson-ap:/home/jetson/krri/docking/configs/mount.json`).
- 파이썬·설정만 바뀌었으면 배포로 끝. 폴더를 새로 만든 첫날과 `ros2/`(C++)가 바뀐 날은 아래 재빌드.

## 5. Jetson 설치·재빌드

```bash
cd /home/jetson/krri/docking
bash scripts/install_jetson.sh            # 새 Jetson 에 한 번: ROS Jazzy·가상환경 ~/.venvs/krri·OpenVINS+브리지 빌드 (30~60분, 인터넷 필요)
bash scripts/rebuild_openvins.sh          # C++ 브리지가 바뀌었거나 폴더 이름을 바꿨을 때(docking 첫 배포 포함): 우리 패키지 둘만 재컴파일 (몇 분)
```

빌드와 가상환경은 프로젝트 밖(`~/.local/share/krri/ros_ws`, `~/.venvs/krri`)에 있어 폴더를 지우거나 다시 배포해도 남는다. 재빌드를 안 하면 "OpenVINS 브리지 업데이트 필요" 로 스스로 멈춘다.

## 6. 로그

판 폴더: `work/<날짜>/<measure|shadow|drive>/<시각>_<id>_<명령>/`. 안에 `observations.jsonl`(관측·depth 매 프레임: `depth.doors`·`depth.lattice_pair`·`depth.lattice_line`, 서 있을 때 `depth_bev_xy` 버드아이 셀), `control.jsonl`(제어 상태·CAN; `status.events` 에 sidestep·bundle·fine_straight·final_check_start/ok·final_heading_turn·final_back_out·best_effort_entry·entry·step_done, 기준 선택 `lateral_source`/`distance_source`, `lateral_sources_disagree`, `anchor_requested`/`anchor_applied`, `lattice_axis_recorded`/`lattice_axis_adopted`, `weak_command_deadband`), `session.json`(끝난 이유·`final_report`: 좌우·방향·남은 거리·격자 자), `openvins.log`.

```bash
ls -td /home/jetson/krri/docking/work/*/*/* | head -1                                                   # 가장 최근 판
scp -r jetson-ap:/home/jetson/krri/docking/work/<날짜> /Users/seojeongmin/krri/docking/work/                                     # 그날 전부 맥으로
grep "\[pipeline\]" $(ls -td /home/jetson/krri/docking/work/*/drive/* | head -1)/openvins.log | tail -1   # dropped=0·imu_lost 수십 이하면 VIO 정상
```

큰 폴더를 Jetson 과 주고받을 땐 rsync 가 깨질 수 있어 tar 를 쓴다: `ssh jetson-ap 'cd /home/jetson && tar czf - krri' > jetson_krri.tar.gz` / `tar czf - 폴더 | ssh jetson-ap 'tar xzf - -C /home/jetson/krri'`.

## 7. 그 외

**와이파이(핫스팟) 설정** — Jetson 터미널에서 (USB 직결로 들어가서)
```bash
sudo nmtui                                              # 화면: "Activate a connection" → 망 선택 → 비밀번호
sudo nmcli dev wifi connect "SSID" password "비밀번호"   # 한 줄 버전
hostname -I                                             # 받은 IP → 맥 ~/.ssh/config 의 HostName 에
```

**맥 VM(ubuntu-vm)에서 돌리기** — Jetson 을 못 쓸 때의 대체. 2026-10-27 에 같은 스택(ROS Jazzy·OpenVINS·브리지·librealsense·Kvaser)을 깔아 둠. 경로·명령은 Jetson 과 똑같이 `/home/jetson/krri/<폴더>` 이고 호스트만 `ubuntu-vm`.
```bash
bash /Users/seojeongmin/krri/docking/update_jetson.sh --host ubuntu-vm       # 배포 (mount.json 은 scp 로 따로)
ssh ubuntu-vm                                     # 또는 VS Code Remote-SSH → ubuntu-vm, 대시보드는 같은 8765 포트 전달
```
- VM 을 UTM 에서 켠 뒤 **UTM 창 위 USB 아이콘**으로 RealSense·Kvaser 를 VM 에 넘긴다(실험마다). 맥엔 허브 없이 직결, 맥 절전 끄기(`caffeinate -s`).
- VM 은 USB 전달이 0.3초 넘게 멈추는 일이 있어(2026-10-01) 실주행은 사람이 제동 자리에 있을 때만.
- `tegrastats`·`jetson_clocks` 는 Jetson 전용이라 VM 에선 없음. 온도 대신 `top` 로 CPU 만 본다.

**CPU·온도**
```bash
ssh jetson-ap 'for z in /sys/class/thermal/thermal_zone*; do t=$(cat $z/temp 2>/dev/null) && echo "$(cat $z/type): $((t/1000))°C"; done'   # 온도 한 번
ssh -t jetson-ap 'tegrastats --interval 2000'      # CPU %·온도·전력 계속 (Ctrl+C)
sudo jetson_clocks                                 # Jetson 에서, 실주행 때 클럭 고정 (재부팅까지)
```
CPU 70 °C 를 넘으면 처리 fps 가 떨어질 수 있다.

## 8. 옵션

**`run_nano.sh <measure|shadow|drive>`**

| 옵션 | 뜻 |
|---|---|
| `--mode left` | 기본이자 유일한 모드(central 모드는 2026-10-30 제거). 태그가 입구 옆, 좌우는 `left_tag_offset_m` 부호 |
| `--view` / `--port 8765` | 대시보드 켬 / 포트 |
| `--panels all` | 카메라 패널: `ir` `color` `depth` 쉼표 조합. 끈 패널은 만들지도 보내지도 않음 |
| `--execute` | drive 에서 CAN 을 진짜 보냄(없으면 거부). 대화형 터미널 필요 |
| `--seconds 60` | N 초 뒤 자동 종료 |
| `--camera-only` / `--no-tag` | measure: OpenVINS 없이 / 태그 검출 끔 |
| `--output 경로` / `--common 경로` | 기록 폴더 / common.json 직접 지정 |

**`preflight.sh <check|mount>`**: `--sensor-only`(설치·장치만) · `--camera-only` · `--folder 폴더`(mount: measure 판) · `--write`(mount: mount.json 저장)

**`update_jetson.sh`**: `--host jetson-ap|jetson-wired|jetson-kw|ubuntu-vm` · `--dry-run`

**`scripts/bev_test.py`** (도킹 앱 끈 상태): `--port 8082` · `--dist 2.6`(입구까지 거리를 알 때; 없으면 창을 훑음) · `--width 1.65` · `--width-tol 0.2` · `--emitter` · `--no-filter` · `--decay 0.7` · `--save 폴더 --every 2` · `--pitch 2` · `--step 2` · `--fps 15`. 격자는 common.json 의 `truck_depth_m`·`lattice_pair_width_m` 이 있을 때만 찾는다.

## 9. 기타 점검

```bash
cd /Users/seojeongmin/krri/docking && PYTHONPATH=.:tests /opt/anaconda3/bin/python3 -m unittest discover -s tests -q     # 맥에서 코드 검사(장치 불필요; 시스템 python3 에는 numpy 가 없음)
source ~/.venvs/krri/bin/activate && python scripts/check_imu_cadence.py full --seconds 60   # Jetson, 앱 끈 상태: IMU·영상 도착 간격(USB 의심될 때만)
```
