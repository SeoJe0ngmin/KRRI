# 현장에서 재야 하는 값 / 이미 정해진 값

`config/control.py`(25개) + `config/detection.py`(33개) 를 성격으로 갈랐다.
**현장에서 손댈 것은 아래 A 의 9개뿐이고, 나머지 49개는 그대로 두면 된다.**

---

## A. 현장에서 재거나 받아야 하는 값 (9개)

### A-1. 남한테 받아야 하는 것 — 탑재부 사양 (2개) ★가장 중요

| 값 | 지금 | 어떻게 정하나 |
|---|---|---|
| `LAT_TOL_M` | 0.030 | **탑재부가 허용하는 좌우 오차 [m]**. 지금 값은 우리 사정(명령 하한 17mm)만 보고 정한 것 |
| `HEAD_TOL_DEG` | 2.0 | **탑재부가 허용하는 각도 오차 [도]** |

이 둘이 뿌리다. `detection.py` 의 `STABLE_LATERAL_M`/`STABLE_HEADING_DEG`/
`MAX_HEADING_SIGMA_DEG` 가 여기서 자동으로 파생되므로, **이 둘만 고치면
나머지는 따라온다.** 반대로 이 둘이 실제보다 빡세면 도킹이 영영 안 끝나고,
헐거우면 탑재부에 안 들어간다.

### A-2. 자로 재는 것 (2개)

| 값 | 지금 | 어떻게 재나 |
|---|---|---|
| `TAG_SIZE_M` | 0.300 | 인쇄한 태그의 **검은 테두리 바깥 한 변**을 자로. 프린터가 30.0cm 로 정확히 안 뽑는다. 여기가 1% 틀리면 모든 거리가 1% 틀린다 |
| `DOCK_EXTRA_M` | 0.0 | 태그면에서 **실제 정지 목표까지 더 갈 거리**. 태그가 탑재부 끝에 안 붙어 있으면 그 차이를 양수로 |

### A-3. 주행해 보고 로그에서 얻는 것 (5개)

주행 후 `python tools/analyze_run.py` 가 권장값을 숫자로 찍어준다.

| 값 | 지금 | 어디서 |
|---|---|---|
| `ROT_LEAD_DEG` | 0.0 | analyze_run 의 "오버슈트 평균" → 그 값을 넣는다 (관성만큼 미리 끊기) |
| `ROT_WATCHDOG_GAIN` | 5.0 | 지금은 넉넉하게 크게. 회전 시간이 안정되면 낮춰도 된다 |
| `ROT_SETTLE_RATE_K` / `ROT_SETTLE_RATE_FLOOR` / `ROT_SETTLE_RATE_CEIL` | 3.0 / 1.0 / 2.0 | 실차 진동에서 "멎었다" 판정이 너무 이르거나 늦으면 조정 |
| `CAM_YAW_OFFSET_DEG` | 0.0 | 도킹이 **매번 같은 방향으로** 삐뚤면 그 각도. 랜덤이면 이 문제가 아니다 |
| `CORNER_NOISE_PX` | 0.07 | 실장비 로그로 역산해 다시 넣을 수 있다 (거리마다 달라진다) |

> A-3 는 **지금 값이 전부 "아무것도 안 하는" 안전한 쪽**이다.
> `ROT_LEAD_DEG=0` 은 보정 안 함, `CAM_YAW_OFFSET_DEG=0` 은 카메라가 똑바르다고 봄.
> 즉 안 고치고 주행해도 위험하지 않고, 로그를 보고 나중에 채우면 된다.

---

## B. 이미 정해진 값 — 현장에서 안 건드린다 (49개)

### B-1. 사실 (바꿀 수 없는 것)

```
COLOR_SIZE (1920,1080)   IR_SIZE          D435I_COLOR_REF     ASSUMED_HFOV_DEG(계산)
TAG_CELLS 8              MM_PER_INCH      COLOR_EXPOSURE_UNIT_US 100.0
TAG_ID 1                 TAG2_ID 2        LUMA_CLIPPED_LEVELS 38
```
장비 스펙·수학 상수·우리가 부여한 번호.

### B-2. 이미 실측으로 확정한 것

```
BLUR_CLEAN_PX 10.0     여기까지 검출 100%
BLUR_DEAD_PX 32.0      여기서 0% (12px 96% / 20px 89% / 28px 50%)
MIN_TAG_PX 20.0        19.5px 에서 검출률 89.3%
MIN_DECISION_MARGIN 20.0   저조도 하한. 거리에는 둔감
DEFAULT_QUAD_BLUR 0.0  9가지 비교에서 안 넣는 게 최선
MAX_REPROJ_RMS_PX 2.0  코너 잡음 바닥의 약 6배
```

### B-3. 계산·파생 (손으로 고치면 안 됨)

```
STABLE_LATERAL_M      = LAT_TOL_M / 3
STABLE_HEADING_DEG    = HEAD_TOL_DEG / 3
MAX_HEADING_SIGMA_DEG = HEAD_TOL_DEG / 4
MEASURE_MAX_FRAMES    = MEASURE_FRAMES * 5
ASSUMED_HFOV_DEG      = fx 에서 계산
```
A-1 을 고치면 자동으로 따라온다.

### B-4. 설계 판단 — 틀려도 느려질 뿐

```
STEP_M 1.0            FWD_SAFETY 0.9        WARMUP_FRACTION 0.33
FWD_ABORT_K 3.0       MAX_STEPS 30          HOLD_RETRY_SEC 0.2
SETTLE_SEC 0.15       ROT_POLL_SEC 0.01     SEARCH_BACKUP_M 0.5
ROT_SETTLE_MAX_SEC 1.0    ROT_SETTLE_MIN_SEC 0.2    ROT_SETTLE_POLL_SEC 0.05
SEARCH_AFTER_MISSES 3     SEARCH_MAX_ROUNDS 3       MEASURE_FRAMES 30
SIGMA_SAMPLES 60          STABLE_SPREAD_K 3.0       STABLE_TAG_PX 50.0
SIDESTEP_BACKWARD_GAIN_DEG 0.0                      TAG_CUT_MARGIN_PX 30.0
DEPTH_TOL_COEF 0.05       DEPTH_TOL_FLOOR_M 0.02    DEPTH_CHECK_MAX_Z 1.5
RELIABLE_TILT_DEG 10.0 (폴백 전용)
ROT_WRONG_WAY_DEG 5.0 (안전장치)
```
2배 틀려도 동작이 굼떠질 뿐 위험하지 않다. 급하지 않으면 그대로 둔다.

---

## 현장 순서 요약

```
가기 전     태그 인쇄 -> 자로 재서 TAG_SIZE_M 확정
현장 도착   탑재부 사양 확인 -> LAT_TOL_M / HEAD_TOL_DEG
            정지 목표까지 거리 재기 -> DOCK_EXTRA_M
주행        python tools/run.py --show --record-events
주행 후     python tools/analyze_run.py  -> ROT_LEAD_DEG 등 권장값 반영
```
