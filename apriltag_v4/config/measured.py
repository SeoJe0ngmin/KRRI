"""before_run.py 가 잰 값. 사람이 고치는 건 ACTIVE_RUN 한 줄뿐이다.

값 자체는 work_dirs/before_run/<ACTIVE_RUN>/measured.json 에 있다.
옛 측정으로 되돌리려면 이 한 줄만 바꾼다. 없으면 조용히 기본값을 쓰지 않고 시끄럽게 실패한다.
"""
import json
from dataclasses import dataclass, field
from pathlib import Path

#: before_run 이 만든 폴더 이름. None 이면 아직 아무것도 안 쟀다는 뜻
ACTIVE_RUN = None

#: 광운대 실측(조향 강도 30) — 우리가 재기 전까지 쓸 **출발점**이다. 학습이 덮어쓴다.
#: 강도를 30 으로 맞춘 이유가 이걸 물려받기 위해서다 (2026-09-22 결정).
#: 출처 project_backup_lean_20260921_174252/extracted/depth_cam/calib/fsm_v4/config.py
KWU_DEFLECTION = 30
KWU_SEED = {
    # ROT_RESPONSE_STARTUP_DELAY_SEC = 1.082 (+-0.105). 우리 실측 1.0~1.32 와 겹친다
    "rot_startup_s": {"L": 1.082, "R": 1.082},
    # ROT_RESPONSE_MAX_RATE_DEG_S = 12.01. 우리 강도 20 실측 8.2~8.8 과 방향이 맞는다
    "rot_rate_dps": {"L": 12.01, "R": 12.01},
    # 끊고 더 도는 시간 [s]. 그쪽 프로파일에서 뽑았다:
    #   유지 STOP_DELAY 0.352 s + 감속 12.01/(2*300) 0.020 s = 0.372 s
    #   되튐 보정 COAST_HEURISTIC_REDUCTION 1.07도 / 12.01 = 0.089 s 를 뺀다
    #   -> 0.283 s. (우리 강도 20 실측 순코스팅은 0.168 s 였다 — 강도를 타는 값이다)
    "rot_tau_s": {"L": 0.283, "R": 0.283},
    # 이보다 작은 각은 못 돈다 [도] — **유도값, 실측 아님**. 정지지연 동안 가속하며 도는 각
    # ½ x 13.06 x 0.352² = 0.81 로 계산했다(그쪽 fallback 세트). before_run 이 실측으로 덮는다.
    "rot_floor_deg": 0.81,
    # 직진(byte 67) — 그쪽이 고른 delayed_linear 적합 (motion_trajectory.selected.json, 10회)
    #   거리 = 0.28973 x 유지시간 - 0.43394,  죽은시간 1.4977 s.  우리 9/7 실측 0.284 m/s 와 일치
    #   사이드스텝(눈감고 시간으로 갈 때)과 직진 lead 의 출발점
    "fwd_speed_mps": {"67": 0.28973},
    "fwd_startup_s": {"67": 1.4977},
}

#: 우리 차에서 **반드시 직접 재야** 하는 것. 남의 값으로 대신할 수 없다.
MUST_MEASURE = ("cam_to_rot_center_m", "rot_center_lateral_m",
                "cam_yaw_offset_deg",
                "sigma_drift_lateral_m", "sigma_drift_heading_deg", "sigma_ref_distance_m")
# tag_cut_m 은 2026-10-01 부터 여기 없다 — 실행이 프레임마다 다시 잰다 (limits.tag_cut_live_m). before_run 의 실측은 대조용


@dataclass
class Measured:
    source: str = ""
    date: str = ""
    rotate_deflection: int | None = None    # 이 값들을 잰 회전 강도. 다르면 못 쓴다
    cam_to_rot_center_m: float | None = None    # 음수 = 카메라 앞
    rot_center_lateral_m: float | None = None
    circle_rms_mm: float | None = None          # 5 mm 이하면 중심이 고정이다 (plan 4-7)
    cam_yaw_offset_deg: float | None = None
    tag_roll_deg: float | None = None
    tag_cut_m: float | None = None              # before_run tagcut 이 실제로 잰 값 — **대조용**. 실행은 안 쓴다
    rot_tau_s: dict = field(default_factory=dict)   # {"L": .., "R": ..}
    rot_rate_dps: dict = field(default_factory=dict)
    rot_startup_s: dict = field(default_factory=dict)
    rot_residual_deg: dict = field(default_factory=dict)
    rot_floor_deg: float | None = None
    fwd_tau_s: dict = field(default_factory=dict)   # {"67": .., "187": ..}  97 은 안 쓴다
    fwd_residual_m: float | None = None
    fwd_speed_mps: dict = field(default_factory=dict)      # {"67": .., "187": ..}
    fwd_startup_s: dict = field(default_factory=dict)
    back_speed_mps: float | None = None                    # byte 187. 9/7 후진 7회에서 확인
    back_startup_s: float | None = None
    # ── 카메라 (열자마자 읽는다. D435I_COLOR_REF 는 참고값일 뿐) ──
    intrinsics: dict = field(default_factory=dict)         # fx fy cx cy w h half_fov_deg
    # ── 차가 쏠리는 양 [도]. "직진" 명령에 실제로 비껴 가는 각 (5-4) ──
    veer_deg: float | None = None
    # ── σ 기준선 (2026-09-30 결정). 0.5초 창 평균의 흔들림, 정지 60초에서 ──
    sigma_ref_distance_m: float | None = None
    sigma_drift_lateral_m: float | None = None             # 밀림. 창 평균에서 안 줄어드는 부분
    sigma_drift_heading_deg: float | None = None
    sigma_drive_lateral_m: float | None = None             # 직진 60초에서 같은 것. 정지보다 작으면 판단을 직진 중에
    sigma_drive_heading_deg: float | None = None
    sigma_drift_distance_m: float | None = None            # 거리도. 없으면 estimate 가 한 장 흔들림으로 대신
    # ── 카메라 대조 3종 (결정 11): 줄자 거리에서 태그 px · 거리 · 화면 행 — 예측 vs 실제 ──
    camera_check: dict = field(default_factory=dict)
    # ── 사람이 줄자로 (before_run 이 묻는다) ──
    height_diff_m: float | None = None                     # 카메라가 잰 높이차 (−vertical 중앙값, sigma_still). 경사·기울기 포함

    @property
    def provisional(self):
        return self.cam_to_rot_center_m is None

    def require(self, *names):
        """없으면 이유를 대고 멈춘다. 모르는 채로 주행하지 않는다."""
        missing = [n for n in names if getattr(self, n, None) is None]
        if missing:
            raise SystemExit("!! 측정값이 없다: %s — before_run.py 를 먼저 돌려라"
                             % ", ".join(missing))


def seeds():
    """학습이 시작할 값. 우리 측정이 없으면 광운대 것으로 출발한다.

    기하(회전중심·카메라 어긋난 각)는 여기 없다 — 그건 차마다 달라서 남의 값을
    쓰면 안 된다. 여기 있는 건 **학습이 곧 덮어쓰는 출발점**뿐이다.
    """
    return {k: dict(v) if isinstance(v, dict) else v for k, v in KWU_SEED.items()}


def load(work_root, deflection_now=None):
    """ACTIVE_RUN 을 읽는다. deflection_now 를 주면 강도 각인을 대조한다."""
    if ACTIVE_RUN is None:
        m = Measured(source="광운대 씨앗(강도 %d)" % KWU_DEFLECTION,
                     rotate_deflection=KWU_DEFLECTION, **seeds())
        if deflection_now is not None and deflection_now != KWU_DEFLECTION:
            raise SystemExit(
                "!! 광운대 씨앗은 조향 강도 %d 짜리인데 지금 설정은 %s 다 — "
                "config/control.ROTATE_JOYSTICK_DEFLECTION 을 맞추거나 직접 측정하라"
                % (KWU_DEFLECTION, deflection_now))
        return m
    path = Path(work_root) / "before_run" / ACTIVE_RUN / "measured.json"
    if not path.is_file():
        raise SystemExit("!! ACTIVE_RUN=%s 인데 파일이 없다: %s" % (ACTIVE_RUN, path))
    raw = json.loads(path.read_text())
    m = Measured(**{k: v for k, v in raw.items() if k in Measured.__dataclass_fields__})
    m.source = str(path)
    if deflection_now is not None and m.rotate_deflection not in (None, deflection_now):
        raise SystemExit(
            "!! 이 측정값은 회전 강도 %s 에서 잰 것인데 지금 설정은 %s 다 — 재측정하라"
            % (m.rotate_deflection, deflection_now))
    return m


def write(work_root, run_name, measured: Measured):
    """before_run 이 쓴다. ACTIVE_RUN 갱신은 사람이 하거나 --activate 로."""
    d = Path(work_root) / "before_run" / run_name
    d.mkdir(parents=True, exist_ok=True)
    body = {k: getattr(measured, k) for k in Measured.__dataclass_fields__}
    (d / "measured.json").write_text(json.dumps(body, ensure_ascii=False, indent=1))
    return d / "measured.json"
