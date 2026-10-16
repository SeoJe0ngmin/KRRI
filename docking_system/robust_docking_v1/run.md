# robust_docking_v1 실행

전체 명령은 [공통 run.md](../run.md)에 정리했다. 공통 설정 파일은 [common.json](../configs/common.json)이다.

```bash
source /home/jetson/.venvs/krri/bin/activate
cd /home/jetson/krri/docking_system/robust_docking_v1
bash scripts/run_nano.sh measure --seconds 60 --view
```

v1~v4의 입력값과 장착 보정은 공유하며, 기록은 `krri/work/docking_system/robust_docking_v1/`에 저장한다.
