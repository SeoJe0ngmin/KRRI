# forklift-truck_docking_BirdsEyeView_v5 — 실행 명령

실행 명령과 현장 순서만 적는다. 설명은 README.md.

## 1. 현장 순서

화면: VS Code 가 8765 포트를 넘겨 주므로 맥 브라우저에서 `http://127.0.0.1:8765`. 모든 명령은 `cd /home/jetson/krri/forklift-truck_docking_BirdsEyeView_v5` 에서. 끝낼 땐 Ctrl+C 한 번.

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
| 현장마다 | `truck_inner_width_m` | 차체가 지나가는 개구 폭 = 벽 충돌 검사 폭(문짝 개구 1.65) |
| | `depth_pair_width_m` | **v3: depth 가 입구로 찾는 쌍의 안쪽 폭** = X 프레임 사이 1.38(줄자). ±`truck_width_tol_m` 0.08. 비우면 truck_inner_width_m |
| | `truck_depth_m` | 입구면→안쪽 막힌 곳. 비우면 검사 없음 |
| | `target_front_x_left_m` | 포크 앞끝이 멈출 자리(입구면 기준, 안쪽 +). 깊이 − 5 cm 안이어야 출발. v3: 입구면 = X 앞끝이라 문짝면 기준 자리 − `tag_plane_offset_m` 의 절댓값 (2.17 − 1.06 = 1.11) |
| | `tag_height_m` | 태그 **중심** 높이. 바꾸면 장착각 재측정 |
| | `left_tag_offset_m` | 입구 중심→태그 중심 좌우. 왼쪽 +, 오른쪽 − |
| | `tag_plane_offset_m` | 입구면↔태그면 앞뒤(안쪽 +). v3: 입구면 = X 앞끝, 태그는 그보다 차 쪽 → **음수** −1.06(로그: 태그면→무릎 높이 X 앞끝). 고치면 `target_front_x_left_m` 도 같은 만큼 |
| | `rear_space_m` | 차 뒤 빈 공간(후진 상한) |
| | `forward_path_bias_deg` | 비우고 시작. 들어간 뒤 한쪽으로 쏠리면 흐른 각(왼쪽 +) |
| | `tag_id` `tag_size_m` | 태그판을 바꿀 때만 |
| 차에 붙은 값 | `vehicle_width_m` `vehicle_length_m` `rotation_center_*` `rear_extent_m`(비움) `target_front_x_central_m` | 그대로 |
| 카메라 옮겼을 때만 | `camera_height_m` `camera_lateral_m` `camera_to_fork_tip_m` | 좌측 IR 렌즈 기준(컬러 렌즈로 쟀으면 왼쪽 1.48 cm 차이). 바꾸면 1절의 2. 장착각 |

`operation`(시간 한도·허용 오차)은 현장과 무관. `mount.json` 은 카메라·태그 값이 안 바뀌면 그대로이고, 바뀌면 `preflight.sh check` 가 재측정을 요구한다.

**v3 기준물·벽 축 (2026-10-29).** 10-28 철기연 29 판 분석 결과를 반영한 v1 다듬기. ① 기준물을 문짝에서 **가위 리프트 X 프레임**으로(줄자: X 사이 1.38, 바닥~꼭대기 0.78, 태그→X 입구쪽 바닥 다리 0.60): 새 키 `depth_pair_width_m` 이 depth 쌍의 폭(줄자 1.38 이지만 띠 높이에서 depth 가 본 안쪽 면 **1.44** 를 씀), `truck_inner_width_m` 1.65 는 충돌 검사 폭으로 남김(1.4 를 충돌 폭으로 쓰면 한쪽 5 cm 라 3 cm·1° 안에서도 `truck_side_clearance` 가 남), `truck_width_tol_m` 0.08(1.36~1.52 만; 턱 1.288·튜브 1.485·문짝 1.65·옆 구조물 1.72 탈락), `tag_plane_offset_m` **−1.06**(입구면 = 띠 높이의 X 앞끝, 10-29 144148 로그가 태그면에서 1.06 으로 직접 잼; 줄자 태그→바닥 다리는 0.60 이라 X 가 사선인 만큼 앞끝이 0.46 안쪽), `target_front_x_left_m` **1.11**(= 2.17 − 1.06, 같은 자리), `left_tag_offset_m` −0.72(X 중심이 문짝 중심보다 6 cm 오른쪽, depth 한 판 관측 → 줄자로 확정), `final_check_distance_m` **1.3**(비우면 0.96 = 카메라가 문짝 10 cm 안에서 회전; 1.3 = 문짝 24 cm 앞). 눈감는 거리 1.3 + 1.11 − 0.07 = 2.34 m. ② `runtime.json` `depth_goal`: `max_m` 3.5(10-29 144148: 3.0 이면 둘째 세밀 정지에서 X 가 3.2 m 라 안 보여 폴백; 러프 정지 4.7 m 에서는 여전히 안 보여 첫 걸음은 태그로), `wall_min_len_m` 1.0(축 후보 최소 길이), 띠 0.45~0.68 그대로(X 는 바닥~0.78 라 들어오고 가로빔 아랫면 0.77 은 안 들어옴). 10-29 144148 판: 문짝 10 cm 안에서 X 쌍 145 프레임(폭 1.44, 양옆 선 1.9~2.2 m, 축 8°±5°·평행 4°±11° 로 흔들려 축은 안 받음 — 의도대로), 약한 직진 97 은 반 속도(0.14 m/s). ③ `blind_time_limit_s` 120(최종 확인 중 태그 못 본 시간 중앙값 23 s·최대 41 s, 보정 5 번이면 90 s). ④ **방향 기준 하나**: v1·v4 는 마지막 확인에서만 "벽 축"(양옆 긴 덩어리의 선 방향)을 두 번째 방향 기준으로 써서, 그게 VIO 와 다르면 `final_heading_turn`(벽 축) ↔ `straighten_fine`(VIO) 이 번갈아 도는 왕복이 났다(10-29 v4: depth 를 잡은 3 판 전부 2.5~6° 로 5 번 왕복 → 진입 포기). v3 는 `wall_axis_enabled` **0 이 기본** — 철기연은 위에서 본 구조물이 직선이 아니라(X 지그재그 8°±5°, 안쪽 1.72 구조물 2.5~6° 어긋남, 문짝 각 가변) 방향은 태그/VIO 하나로 가고, 좌우만 X 쌍 앞끝 두 점으로 잰다. 측정된 축은 `wall_axis_rejected`(reason `disabled`) 로 계속 기록되니 몇 판 모아 부호·크기가 일정하면 태그면 기울기로 보고 태그를 바로 붙이거나 `forward_path_bias_deg` 로 상쇄. 1 로 켜면 `wall_parallel_tol_deg` 3·`wall_axis_spread_deg` 1·`wall_axis_max_deg` 10 세 조건을 넘을 때만 축을 넘겨받아(`wall_axis_adopted`) 방향 목표·좌우 오차를 그 축 기준으로 쓴다(광운대처럼 평평한 옆판이 있는 곳용). 대시보드 버드아이 아래 `벽 축 ±x°` 는 넘겨받았을 때만.

**v5 격자 프레임 (2026-10-29 저녁).** v3 위에 탑재부 **제일 안쪽 격자 프레임**(트럭에 달린 것, 정면을 가로지르는 직선) 측정을 얹음. `site` 의 `frame_behind_pair_m`(입구면 = X 앞끝(55 cm 높이)에서 프레임 앞면까지 안쪽 거리, 줄자) 을 넣으면 켜지고, 비우면 v3 와 동일. depth 점 중 예상 거리 ± `frame_window_m`(0.4) 안의 것을 좌우 5 cm 칸으로 나눠 가로 직선 `x = x0 + s·y` 를 맞춘다(격자 구멍은 상관없고 양끝 사이 길이가 `frame_min_len_m` 1.0 이상, 맞춤 잔차 5 cm 이하). 얻는 것 ① **방향**: 기울기 s → 트럭 축. 창 안에서 안정(`frame_axis_spread_deg` 1)하고 태그 축 근처(`wall_axis_max_deg` 10)면 v3 의 축 넘겨받기와 같은 방식으로 기준이 됨(`events` 의 `frame_axis_adopted`/`frame_axis_rejected`; `frame_axis_enabled` 0 이면 기록만). 벽 축과 같은 창에 둘 다 있으면 프레임이 이김. ② **거리**: 프레임까지 정면 거리 − `frame_behind_pair_m` = 입구면까지 거리(`distance_source: frame`; X 앞끝의 흐릿한 거리 대신). ③ **좌우**: X 쌍이 보이면 그대로 X(`lateral_source: pair`); 안 보이고 프레임 양끝이 다 보이면(`frame_inner_width_m` ± `frame_width_tol_m`) 프레임 중심(`frame_ends`); 둘 다 없으면 이전 depth 중심(`carried`) → 태그(`tag`) → 자세(`pose`) 순. 프레임만 보여도 폴백으로 안 가고 세밀 단계가 이어진다. `runtime.json` `depth_goal` 의 `frame_max_m`(5.0) 은 프레임 찾기만의 최대 거리라 X 쌍(`max_m`)보다 멀리서도 보고, `frame_z_min/z_max` 를 주면 프레임만 다른 높이 띠로 본다(가로대 아랫면 0.77 같은 **가로 부재**를 쓰려면 거리와 이 띠(예 0.80~0.95)만 바꾸면 같은 코드로 됨). 대시보드 버드아이 아래 `프레임 x.xx m ±x°`, `축(frame)`. 현장 줄자: 55 cm 높이 X 제일 앞↔프레임 앞면, 프레임 기둥 안쪽 폭, 가로 부재 높이.

**v4 약한 명령도 포함 (2026-10-29).** 러프 정지 뒤 depth 로만 가는 단계(세밀·최종·진입·폴백)에서 회전·묶음 드라이브·후진을 약한 CAN 바이트로 보낸다(`forward_fine`/`backward_fine`/`rotate_*_fine2`). 긴 직진(fine_straight·진입)은 그대로. `operation` 의 `weak_forward_byte`(기본 97 = 예전 forward_slow, 68~126)·`weak_rotate_delta`(기본 20 = fine 과 같음, 1~30; 현장에서 12~15 로 내려 보기)·`fine_weak_commands`(0 이면 v1 과 동일). 약한 명령으로 `weak_deadband_s`(4 s) 동안 안 움직이면 그 걸음을 보통 바이트로 다시 보내고 그 종류는 그 판 동안 안 씀(`status.events` 의 `weak_command_deadband`). 응답 모델은 명령별로 배우므로 처음 몇 걸음은 예측이 거칠다.

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
Host jetson-ap           # 젠슨이 직접 쏘는 와이파이 (AP 모드)
    HostName 10.42.0.1
    User jetson
    IdentityFile ~/.ssh/id_ed25519_vm
```

- 이 문서는 `jetson-wired` 로 적는다. 핫스팟이면 `jetson-kw`, 젠슨 와이파이(AP)면 `jetson-ap` 로 바꿔 친다.
- **젠슨 AP (2026-10-29부터 기본)**: 젠슨이 `jetson-ap`(비번 12345678, 10.42.0.1) 와이파이를 직접 쏜다. 맥은 그 와이파이에 붙고 `--host jetson-ap`. 만든 법(젠슨, 한 번): `sudo nmcli dev wifi hotspot ifname wlP1p1s0 con-name jetson-ap ssid jetson-ap password "12345678"` → `sudo nmcli con modify jetson-ap ipv4.method shared ipv4.addresses 10.42.0.1/24 connection.autoconnect yes connection.autoconnect-priority 50`. 부팅하면 AP 가 먼저 뜬다. 인터넷이 필요하면 `sudo nmcli con down jetson-ap && sudo nmcli con up 경원`, 끝나면 `sudo nmcli con up jetson-ap`. AP 동안은 맥도 인터넷이 안 된다.
- `ssh jetson-wired '…'` 로 시작하는 줄은 **맥 터미널**에서, `cd /home/jetson/…` 로 시작하는 줄은 **Jetson 터미널(VS Code)** 에서 친다.
- **시계**: Jetson 은 껐다 켜면 1970년이다. 맥에서 ssh·VS Code·배포로 한 번 붙으면 자동으로 맞춰진다(맥 `~/.ssh/config` 의 `LocalCommand` + Jetson `/etc/sudoers.d/clock`, 설정 완료). 맥이 아닌 컴퓨터에서는 Jetson 터미널에서 `sudo date -s "2026-10-24 09:30:00"` 처럼 직접 맞춘다. 안 맞으면 로그가 `work/19700101/` 에 쌓일 뿐 주행엔 영향 없다.
- **새 핫스팟**: ① Jetson(한 번) USB 로 들어가 `sudo nmtui` 또는 `sudo nmcli dev wifi connect "SSID" password "비밀번호"`, `hostname -I` 로 IP 확인 ② 맥 `~/.ssh/config` 에 `Host jetson-<이름>` 항목(HostName = 그 IP). IP 가 바뀌면 HostName 만 고친다. 키·sudoers 는 다시 안 한다.

## 4. 맥 → Jetson 배포

```bash
bash /Users/seojeongmin/krri/forklift-truck_docking_BirdsEyeView_v5/update_jetson.sh --host jetson-wired              # 바뀐 파일만 보냄
bash /Users/seojeongmin/krri/forklift-truck_docking_BirdsEyeView_v5/update_jetson.sh --host jetson-wired --dry-run    # 보낼 파일·common.json 차이만 보기
```

- `configs/common.json` 은 **맥 것이 Jetson 을 덮는다**(주인은 맥). 덮기 전에 다른 줄을 보여 준다.
- `work/`(로그)와 `configs/mount.json`(장착각)은 보내지도 지우지도 않는다. mount.json 을 새로 넣을 땐 `scp /Users/seojeongmin/krri/forklift-truck_docking_BirdsEyeView_v5/configs/mount.json jetson-wired:/home/jetson/krri/forklift-truck_docking_BirdsEyeView_v5/configs/mount.json`.
- 파이썬·설정만 바뀌었으면 배포로 끝. `ros2/`(C++)가 바뀐 날만 아래 재빌드.

## 5. Jetson 설치·재빌드

```bash
cd /home/jetson/krri/forklift-truck_docking_BirdsEyeView_v5
bash scripts/install_jetson.sh            # 새 Jetson 에 한 번: ROS Jazzy·가상환경 ~/.venvs/krri·OpenVINS+브리지 빌드 (30~60분, 인터넷 필요)
bash scripts/rebuild_openvins.sh          # C++ 브리지가 바뀌었거나 폴더 이름을 바꿨을 때: 우리 패키지 둘만 재컴파일 (몇 분)
```

빌드와 가상환경은 프로젝트 밖(`~/.local/share/krri/ros_ws`, `~/.venvs/krri`)에 있어 폴더를 지우거나 다시 배포해도 남는다. 재빌드를 안 하면 "OpenVINS 브리지 업데이트 필요" 로 스스로 멈춘다.

## 6. 로그

판 폴더: `work/<날짜>/<measure|shadow|drive>/<시각>_<id>_<명령>/`. 안에 `observations.jsonl`(관측·depth 매 프레임), `control.jsonl`(제어 상태·CAN; `status.events` 에 sidestep·bundle·final_check_ok·entry·step_done), `session.json`(끝난 이유), `openvins.log`.

```bash
ls -td /home/jetson/krri/forklift-truck_docking_BirdsEyeView_v5/work/*/*/* | head -1                                                   # 가장 최근 판
scp -r jetson-wired:/home/jetson/krri/forklift-truck_docking_BirdsEyeView_v5/work/<날짜> /Users/seojeongmin/krri/forklift-truck_docking_BirdsEyeView_v5/work/                                     # 그날 전부 맥으로
grep "\[pipeline\]" $(ls -td /home/jetson/krri/forklift-truck_docking_BirdsEyeView_v5/work/*/drive/* | head -1)/openvins.log | tail -1   # dropped=0·imu_lost 수십 이하면 VIO 정상
```

큰 폴더를 Jetson 과 주고받을 땐 rsync 가 깨질 수 있어 tar 를 쓴다: `ssh jetson-wired 'cd /home/jetson && tar czf - krri' > jetson_krri.tar.gz` / `tar czf - 폴더 | ssh jetson-wired 'tar xzf - -C /home/jetson/krri'`.

## 7. 그 외

**와이파이(핫스팟) 설정** — Jetson 터미널에서 (USB 직결로 들어가서)
```bash
sudo nmtui                                              # 화면: "Activate a connection" → 망 선택 → 비밀번호
sudo nmcli dev wifi connect "SSID" password "비밀번호"   # 한 줄 버전
hostname -I                                             # 받은 IP → 맥 ~/.ssh/config 의 HostName 에
```

**맥 VM(ubuntu-vm)에서 돌리기** — Jetson 을 못 쓸 때의 대체. 2026-10-27 에 같은 스택(ROS Jazzy·OpenVINS·브리지·librealsense·Kvaser)을 깔아 둠. 경로·명령은 Jetson 과 똑같이 `/home/jetson/krri/<폴더>` 이고 호스트만 `ubuntu-vm`.
```bash
bash /Users/seojeongmin/krri/forklift-truck_docking_BirdsEyeView_v5/update_jetson.sh --host ubuntu-vm       # 배포 (mount.json 은 scp 로 따로)
ssh ubuntu-vm                                     # 또는 VS Code Remote-SSH → ubuntu-vm, 대시보드는 같은 8765 포트 전달
```
- VM 을 UTM 에서 켠 뒤 **UTM 창 위 USB 아이콘**으로 RealSense·Kvaser 를 VM 에 넘긴다(실험마다). 맥엔 허브 없이 직결, 맥 절전 끄기(`caffeinate -s`).
- VM 은 USB 전달이 0.3초 넘게 멈추는 일이 있어(2026-10-01) 실주행은 사람이 제동 자리에 있을 때만.
- `tegrastats`·`jetson_clocks` 는 Jetson 전용이라 VM 에선 없음. 온도 대신 `top` 로 CPU 만 본다.

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
cd /Users/seojeongmin/krri/forklift-truck_docking_BirdsEyeView_v5 && PYTHONPATH=.:tests python3 -m unittest discover -s tests -q     # 맥에서 코드 검사(장치 불필요)
source ~/.venvs/krri/bin/activate && python scripts/check_imu_cadence.py full --seconds 60   # Jetson, 앱 끈 상태: IMU·영상 도착 간격(USB 의심될 때만)
```
