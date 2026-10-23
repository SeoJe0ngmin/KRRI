# reference — 참고용 외부 코드·환경 목록 (배포·빌드와 무관)

이 폴더는 **읽기용**이다. 여기 있는 것은 Jetson 에서 `scripts/install_jetson.sh`(전체 설치) 와
`scripts/rebuild_openvins.sh`(브리지 재빌드) 가 `~/.local/share/krri/` 와 `~/.venvs/krri/` 에 깔아 두는 것들의 사본·목록이다.
`update_jetson.sh` 는 이 폴더를 보내지 않고, 여기서 빌드하지도 않는다. 고칠 코드는 각 프로젝트 폴더(`forklift-truck_docking_*`)에 있다.

| 항목 | Jetson 실제 위치 | 여기 | 비고 |
|---|---|---|---|
| OpenVINS 소스(고정 커밋 `69488123ed93`) | `~/.local/share/krri/ros_ws/src/open_vins` (원본 클론은 `ros_ws/upstream/open_vins`) | `open_vins/` | 소스는 원본 그대로. `prepare_openvins.py` 는 CMakeLists.txt 세 개만 새로 만든다 |
| OpenVINS 중 실제로 빌드되는 부분 | `~/.local/share/krri/ros_ws/src/open_vins` | `open_vins_used/` | `prepare_openvins.py` 를 맥에서 돌려 만든 것. ov_core/ov_init/ov_msckf 의 .cpp 48개·헤더 60개 + 생성된 CMakeLists 3개(1.6 MB). 다시 만들기: `python3 <프로젝트>/scripts/prepare_openvins.py reference/open_vins reference/open_vins_used` |
| librealsense 소스(`v2.56.5`) | `~/.local/share/krri/realsense/source` (`install_realsense.sh` 로 빌드했을 때만) | `librealsense/` | 소스 빌드 안 했으면 GitHub 태그를 그대로 받은 것 |
| OpenVINS 골라 올리는 스크립트 | 각 프로젝트 폴더 `scripts/prepare_openvins.py` | 없음 | 우리 스크립트라 프로젝트 폴더에서 본다 |
| 우리 브리지(C++) | 프로젝트 폴더 `ros2/docking_openvins`, `ros2/docking_msgs` (작업공간엔 링크) | 없음 | 프로젝트 폴더에서 본다 |
| apt 패키지 목록 | 시스템 | `jetson_apt_packages.txt` | `dpkg -l` 출력 |
| pip 패키지 목록 | `~/.venvs/krri` | `jetson_pip_packages.txt` | `pip freeze` 출력 |
| ROS 작업공간에 빌드된 패키지 | `~/.local/share/krri/ros_ws/install` | `jetson_ros_ws_packages.txt` | 폴더 이름 목록 |

빌드 결과물(`ros_ws/build`, `ros_ws/install`, `realsense/install`, 가상환경 본체)은 ARM 리눅스 바이너리라 맥에서 쓸 수 없어 가져오지 않는다.

## 원본 저장소 바로가기

- OpenVINS: https://github.com/rpng/open_vins — 쓰는 커밋 [`69488123ed93`](https://github.com/rpng/open_vins/tree/69488123ed9362dd44b6f28e7f4680abbff1442b)
- librealsense: https://github.com/realsenseai/librealsense — 쓰는 태그 [`v2.56.5`](https://github.com/realsenseai/librealsense/tree/v2.56.5)

git 에는 `README.md` 와 `open_vins_used/` 만 올린다. `open_vins/`·`librealsense/` 클론은 용량이 크고 위 링크에서 그대로 받을 수 있어 올리지 않는다(.gitignore).

## 다시 받는 법

GitHub 에서 (Jetson 없이):
```bash
cd /Users/seojeongmin/krri/reference
git clone https://github.com/rpng/open_vins.git open_vins && git -C open_vins checkout --detach 69488123ed9362dd44b6f28e7f4680abbff1442b
git clone --branch v2.56.5 --depth 1 https://github.com/realsenseai/librealsense.git librealsense
```

Jetson 에서 목록 뽑기 (USB 직결 `jetson-wired`, 핫스팟이면 `jetson-kw`):
```bash
cd /Users/seojeongmin/krri/reference
ssh jetson-wired 'dpkg -l' > jetson_apt_packages.txt
ssh jetson-wired '~/.venvs/krri/bin/pip freeze' > jetson_pip_packages.txt
ssh jetson-wired 'ls ~/.local/share/krri/ros_ws/install' > jetson_ros_ws_packages.txt
```
