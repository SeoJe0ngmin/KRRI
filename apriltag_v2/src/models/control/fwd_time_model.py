# fwd_time_model.py
"""
전진 명령 시간 모델 (가속 구간 이후 정속 구간, piecewise).

- 파라미터: t0(latency), t1(가속 지속시간), a(가속도)
    - vmax  = a * t1
    - d_acc = 0.5 * a * t1^2

- 변환식
    - d <= d_acc 인 경우: t = t0 + sqrt(2d/a)
    - d >  d_acc 인 경우: t = t0 + t1 + (d - d_acc)/vmax

- 후처리
    - [min_sec, max_sec] 범위로 클램프
    - 현장 튜닝용 선형 보정: d_eff = scale*|offset| + bias

- FWD_* 기본값의 출처
    - 이론값 아님. 로그(t_monotonic, dist_z) 기반 적합 결과
    - 장비 교체나 노면/적재 조건 변경 시 재적합 필요
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


# ===== 전진시간 피팅(가속→정속) 파라미터 =====
# 호출부에서 이 모델의 사용 여부를 결정하는 스위치 (이 파일 내부에서는 미참조)
USE_PIECEWISE_FWD_FIT = True

FWD_T0 = 0.5069       # s (명령 후 실이동 시작까지의 지연)
FWD_T1 = 2.0357       # s (가속 구간 지속시간)
FWD_A = 0.139644      # m/s^2 (초기 가속도)
# 위 값으로부터 vmax 약 0.284 m/s, d_acc 약 0.289 m
# 즉 0.289 m 미만의 이동은 정속 도달 없이 종료

FWD_SCALE = 0.97      # d_eff = FWD_SCALE*|offset| + FWD_BIAS
FWD_BIAS = -0.09      # m (재적합 없이 현장 오차를 보정하는 용도)

# 안전 클램프(전진 명령 타이머). 센서 이상값 유입 시의 최종 방어선
FWD_MIN_SEC = 1.0
FWD_MAX_SEC = 15.0


@dataclass(frozen=True)
class PiecewiseFwdParams:
    """
    가속 구간 및 정속 구간 모델의 파라미터 묶음.

    - frozen 설정이므로 생성 후 값 변경 불가
    - override 시 새 인스턴스 생성 또는 함수 keyword 인자 사용
    """
    t0: float = FWD_T0            # latency (s)
    t1: float = FWD_T1            # accel duration (s)
    a: float = FWD_A              # acceleration (m/s^2)
    scale: float = FWD_SCALE      # d_eff = scale*|offset| + bias
    bias: float = FWD_BIAS        # meters
    min_sec: float = FWD_MIN_SEC
    max_sec: float = FWD_MAX_SEC

    @property
    def vmax(self) -> float:
        """가속 종료 후의 정속 속도 (m/s)."""
        return self.a * self.t1

    @property
    def d_acc(self) -> float:
        """가속 구간의 주행 거리 (m). 두 case를 가르는 경계값."""
        return 0.5 * self.a * (self.t1 ** 2)


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _safe_sqrt(x: float) -> float:
    # 수치 오차로 음수 입력 시 예외 없이 0 반환
    return (x if x > 0 else 0) ** 0.5


def time_from_distance_piecewise(d: float, params: PiecewiseFwdParams) -> float:
    """
    거리(m)의 시간(s) 변환. 순수 kinematics 계산.

    Args:
        d: 전진 거리 (m). 음수 입력 시 0으로 처리
        params: 모델 파라미터

    Returns:
        [min_sec, max_sec] 범위로 클램프된 시간 (s)
    """
    d = max(0.0, d)
    d_acc = params.d_acc
    vmax = max(1e-6, params.vmax)  # t1 또는 a가 0일 때의 ZeroDivisionError 방지

    if d <= d_acc:
        t_cmd = params.t0 + _safe_sqrt(2.0 * d / max(1e-9, params.a))
    else:
        t_cmd = params.t0 + params.t1 + (d - d_acc) / vmax

    return _clamp(t_cmd, params.min_sec, params.max_sec)


def fwd_sec_from_offset_piecewise(
    offset_m: float,
    *,
    t0: Optional[float] = None,
    t1: Optional[float] = None,
    a: Optional[float] = None,
    scale: Optional[float] = None,
    bias: Optional[float] = None,
    min_sec: Optional[float] = None,
    max_sec: Optional[float] = None,
) -> float:
    params = PiecewiseFwdParams(
        t0=FWD_T0 if t0 is None else t0,
        t1=FWD_T1 if t1 is None else t1,
        a=FWD_A if a is None else a,
        scale=FWD_SCALE if scale is None else scale,
        bias=FWD_BIAS if bias is None else bias,
        min_sec=FWD_MIN_SEC if min_sec is None else min_sec,
        max_sec=FWD_MAX_SEC if max_sec is None else max_sec,
    )

    d_eff = max(0.0, params.scale * abs(offset_m) + params.bias)

    return time_from_distance_piecewise(d_eff, params)
