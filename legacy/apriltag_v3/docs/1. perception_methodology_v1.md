# Perception Problem 해결 방법론

## 1. 목표

현재 AprilTag 기반 docking 시스템에서 가장 먼저 해결해야 할 문제는 **Perception 단계에서 발생하는 측정값의 흔들림과 outlier**이다.

예를 들어 실제 지게차는 목표 지점으로 일정하게 접근하고 있는데도 `forward` 측정값이 다음과 같이 나올 수 있다.

```text
실제 경향:
1.80 → 1.70 → 1.60 → 1.50

Detection:
1.80 → 1.70 → 1.78 → 1.59
                  ↑
               비정상적인 튐
```

현재처럼 이러한 raw detection 값을 그대로 제어에 사용하면 실제 차량의 오차가 아니라 **센서 측정 오차를 차량 위치 오차로 착각하여 불필요한 제어 명령을 발생**시킬 수 있다.

따라서 Perception 단계의 목표는 다음과 같다.

> **AprilTag의 raw 측정값을 그대로 사용하는 것이 아니라, 측정 품질과 시간적 연속성을 이용하여 현재 차량의 실제 상태에 가까운 값을 실시간으로 추정한다.**

---

# 2. 현재 문제

## 2.1 AprilTag Detection Noise

AprilTag로부터 다음 값을 계산하고 있다.

- `forward`
- `lateral`
- `heading`

하지만 동일한 위치에서도 측정값이 매 frame마다 완전히 동일하지 않다.

예:

```text
forward:
1.802
1.794
1.811
1.760
1.799
1.805
```

이 중 일부는 일반적인 측정 noise이고, 일부는 순간적인 outlier일 수 있다.

---

## 2.2 현재 30-frame 기반 방식의 한계

현재 시스템은 여러 frame을 모은 후 median 등의 대표값을 사용하는 방식이다.

```text
30 frame 수집
    ↓
median 계산
    ↓
최종 pose 결정
```

이 방식은 차량이 완전히 정지한 상황에서는 noise를 줄이는 데 유효하다.

하지만 차량이 움직이는 동안에는 다음 문제가 있다.

- 과거 frame이 현재 위치 계산에 포함됨
- 현재 차량 상태가 즉시 반영되지 않음
- 주행 중에는 결과적으로 시간 지연(latency)이 발생함
- noise를 줄이는 대신 현재 위치 추정이 늦어짐

따라서 **주행 중 측정 방식과 정지 후 정밀 측정 방식을 분리할 필요가 있다.**

---

# 3. 제안하는 Perception 구조

전체 구조는 다음과 같이 구성한다.

```text
AprilTag Raw Detection
        ↓
1. Detection Quality Check
        ↓
2. Outlier Detection / Gating
        ↓
3. Real-time State Estimation
        ↓
Filtered Forward / Lateral / Heading
        ↓
Controller
```

---

# 4. Step 1 - Detection Quality Check

먼저 AprilTag detection 자체의 품질이 낮은 frame을 제거한다.

현재 코드에서 이미 사용하고 있거나 사용할 수 있는 정보는 다음과 같다.

- Tag pixel size
- Reprojection error
- Decision margin
- Hamming distance
- Heading uncertainty
- Tag detection 여부

예를 들어 다음과 같은 경우 해당 frame의 pose를 사용하지 않는다.

```text
Tag가 지나치게 작음
Reprojection error가 큼
Decision margin이 너무 낮음
Detection 결과가 불안정함
```

이 단계는 **이미 명백하게 잘못된 detection을 1차적으로 제거하는 역할**을 한다.

---

# 5. Step 2 - Temporal Outlier Detection

Detection 자체가 성공했더라도 이전 상태와 비교했을 때 물리적으로 말이 안 되는 값이 나올 수 있다.

예:

```text
t = 0.0 s → forward = 1.80 m
t = 0.1 s → forward = 1.70 m
t = 0.2 s → forward = 1.79 m
```

지게차가 계속 앞으로 움직이고 있다면 짧은 시간 안에 갑자기 목표에서 멀어지는 값은 의심할 수 있다.

따라서 단순히 AprilTag의 detection 성공 여부뿐 아니라 다음을 비교한다.

```text
이전 추정 위치
+
현재 추정 속도
+
경과 시간
        ↓
이번 detection이 가능한 범위인가?
```

예상 위치를 단순하게 다음처럼 계산할 수 있다.

\[
\hat d_t = d_{t-1} - v_{t-1}\Delta t
\]

그리고 실제 측정값과 예상값의 차이를 계산한다.

\[
r_t = z_t - \hat d_t
\]

여기서 `r_t`가 지나치게 크면 해당 frame은 다음 중 하나로 처리한다.

- 완전히 reject
- 낮은 weight로 반영
- 추가 frame이 들어올 때까지 보류

초기 구현에서는 복잡한 방법보다 **threshold 기반 gating**으로 시작해도 된다.

---

# 6. Step 3 - 실시간 State Estimation

현재 30-frame median을 주행 중 계속 사용하는 대신 **매 frame마다 현재 상태를 업데이트하는 실시간 estimator**를 사용한다.

핵심 원리는 다음과 같다.

> 이전 상태를 이용해 현재 상태를 예상하고, 새로운 AprilTag measurement를 이용해 그 예상을 수정한다.

---

## 6.1 Forward

Forward는 위치만 추정하지 않고 속도까지 같이 추정하는 것이 좋다.

상태:

\[
x =
\begin{bmatrix}
forward \\
velocity
\end{bmatrix}
\]

기본 motion model:

\[
forward_{t+1}
=
forward_t - velocity_t \Delta t
\]

이렇게 하면 다음 두 값을 동시에 얻을 수 있다.

- 현재 목표까지 남은 거리
- 현재 접근 속도

이 속도값은 이후 Vehicle Dynamics 문제에서 **정지 관성 예측**에도 직접 사용할 수 있다.

초기 구현 후보:

- Alpha-Beta Filter
- Constant Velocity Kalman Filter

처음부터 EKF나 복잡한 비선형 estimator를 사용할 필요는 없다.

---

## 6.2 Lateral

Lateral은 AprilTag 측정값을 주로 사용하되 frame 단위의 갑작스러운 jump를 그대로 사용하지 않는다.

예:

```text
+10 mm
+13 mm
+11 mm
+47 mm  ← outlier 가능
+12 mm
```

초기 구현 후보:

- Outlier gating
- 짧은 low-pass filter
- 1D Kalman Filter

---

## 6.3 Heading

Heading은 AprilTag 하나만 사용하는 것보다 IMU와 결합하는 것이 유리하다.

구조:

```text
AprilTag Heading
       +
IMU Yaw 변화량
       ↓
Estimated Heading
```

IMU의 장점:

- 짧은 시간 동안 회전 변화량을 빠르게 측정 가능
- frame 간 heading 변화 추적에 유리

AprilTag의 장점:

- 절대적인 heading 기준을 제공 가능
- IMU drift를 다시 보정할 수 있음

따라서 두 센서를 상호 보완적으로 사용한다.

초기 구현 후보:

- Complementary Filter
- Kalman Filter

---

# 7. 주행 중 / 정지 후 측정 방식 분리

현재 30-frame 방식 자체를 완전히 제거할 필요는 없다.

대신 역할을 분리한다.

## 주행 중

실시간 estimator를 사용한다.

```text
AprilTag Frame
     ↓
Quality Check
     ↓
Outlier Check
     ↓
Real-time Filter
     ↓
Current Pose
```

매 frame마다 최신 상태를 controller에 전달한다.

---

## 정지 후 최종 확인

차량이 완전히 정지한 후에는 기존의 multi-frame median 방식이 유리하다.

```text
차량 정지
    ↓
여러 frame 수집
    ↓
Median / Robust Statistics
    ↓
최종 Pose 확인
```

정지 상태에서는 과거 frame을 사용하는 것이 latency 문제를 만들지 않기 때문이다.

따라서 최종 구조는 다음과 같다.

```text
주행 중
→ Real-time State Estimation

정지 후
→ Multi-frame Robust Measurement
```

---

# 8. Raw Log를 사용하는 이유

Raw log는 정답 데이터를 학습하기 위한 데이터가 아니다.

목적은 **현재 AprilTag measurement가 실제로 얼마나 흔들리는지 측정하는 것**이다.

확인해야 하는 항목:

- Forward noise 크기
- Lateral noise 크기
- Heading noise 크기
- Outlier 발생 빈도
- Outlier 크기
- 거리별 noise 변화
- Tag pixel size와 noise의 관계
- Reprojection error와 실제 흔들림의 관계

Raw log 자체가 불확실해도 문제가 없다.

예를 들어 차량이 완전히 정지해 있다면 실제 pose는 변하지 않는다.

따라서 정지 상태에서 발생하는 측정값의 변화는 대부분 **Perception noise의 특성**으로 볼 수 있다.

---

# 9. 사전 실험

Perception filter를 실제 주행에 바로 넣기 전에 간단한 사전 실험을 수행하는 것이 좋다.

---

## 실험 1 - 정지 상태 측정

지게차를 완전히 정지시킨 상태에서 1~2분 동안 AprilTag raw detection을 저장한다.

저장 항목 예:

```text
timestamp
forward
lateral
heading
tag_px
reprojection_error
decision_margin
hamming
detection_valid
```

분석 항목:

- 평균
- 표준편차
- 최대/최소
- Outlier 빈도
- 시간에 따른 흔들림

이 실험을 통해 기본적인 sensor noise 수준을 추정한다.

---

## 실험 2 - 일정한 직진 주행

지게차를 일정한 속도로 앞으로 이동시키면서 raw detection을 저장한다.

정확한 ground truth가 없어도 된다.

차량이 계속 목표를 향해 이동 중이라면 `forward`는 전체적으로 감소하는 형태여야 한다.

예:

```text
정상적인 경향

1.80
1.74
1.68
1.62
1.56
```

다음과 같은 값이 발생한다면 temporal outlier 후보로 볼 수 있다.

```text
1.80
1.74
1.68
1.79  ← 비정상 jump
1.56
```

이 실험을 통해 주행 중 noise와 outlier 특성을 분석한다.

---

# 10. 주행 전 사전 실험용 Python Script 추가

실제 docking 주행을 시작하기 전에 **Perception 상태를 점검하는 별도의 실행 Python 파일**을 만드는 것이 좋다.

예시 파일명:

```text
tools/precheck_perception.py
```

또는

```text
tools/perception_test.py
```

이 파일은 차량 제어를 수행하지 않고 **AprilTag detection 상태만 검사**한다.

주요 기능:

1. Camera 연결 확인
2. AprilTag detection 여부 확인
3. Raw `forward / lateral / heading` 출력
4. Detection quality 출력
5. 일정 시간 동안 raw log 저장
6. Forward / lateral / heading의 표준편차 계산
7. Outlier 개수 계산
8. 현재 detection 상태를 PASS / WARNING 형태로 표시

예시:

```text
=== Perception Pre-check ===

Camera            : OK
Tag detected      : OK
Samples           : 300

Forward mean      : 1.802 m
Forward std       : 0.006 m

Lateral mean      : 0.012 m
Lateral std       : 0.004 m

Heading mean      : 0.32 deg
Heading std       : 0.18 deg

Outliers          : 3 / 300
Detection quality : PASS
```

---

# 11. 사전 실험 Python의 목적

이 파일의 목적은 매번 모델을 다시 학습하는 것이 아니다.

실제 주행 전에 다음을 확인하는 것이다.

- 오늘 카메라 상태가 정상인지
- Tag detection이 정상적으로 되는지
- 측정값이 평소보다 심하게 흔들리는지
- 카메라 위치가 변하지 않았는지
- 조명이나 반사 등으로 detection 품질이 저하되지 않았는지
- Filter를 적용하기 전에 raw sensor 상태가 정상인지

즉,

```text
Docking 실행 전
     ↓
precheck_perception.py
     ↓
Detection 정상 확인
     ↓
실제 docking 실행
```

과 같은 workflow를 만든다.

추후에는 사전 실험 결과를 이용해 현재 환경에 맞춰 filter의 measurement noise parameter를 자동 조정하는 기능까지 확장할 수 있다.

---

# 12. 1차 구현 권장 순서

## Phase 1

Raw logging 기능 추가

```text
timestamp
forward
lateral
heading
quality 정보
```

---

## Phase 2

`precheck_perception.py` 구현

- 정지 상태 detection 검사
- 통계 출력
- CSV 저장

---

## Phase 3

Temporal outlier gating 추가

```text
Raw Detection
    ↓
Quality Check
    ↓
Temporal Gating
```

---

## Phase 4

Forward real-time estimator 구현

우선 가장 중요한 `forward + velocity`부터 구현한다.

추천:

```text
Constant Velocity Kalman Filter
```

또는 더 단순한:

```text
Alpha-Beta Filter
```

---

## Phase 5

Lateral / Heading estimator 추가

- Lateral filtering
- Heading + IMU fusion

---

## Phase 6

Controller 입력 교체

기존:

```text
Raw / 30-frame Pose
      ↓
Controller
```

변경:

```text
Real-time Estimated Pose
          ↓
Controller
```

---

# 13. 최종 목표 구조

```text
Camera
  ↓
AprilTag Detection
  ↓
Detection Quality Check
  ↓
Temporal Outlier Rejection
  ↓
Real-time State Estimator
  ├─ Forward
  ├─ Velocity
  ├─ Lateral
  └─ Heading
       ↑
      IMU
       ↓
Estimated Current Vehicle State
       ↓
Controller
```

그리고 정지 후에는 별도로:

```text
Vehicle Stop
    ↓
30-frame Robust Measurement
    ↓
Final Pose Verification
```

을 수행한다.

---

# 핵심 결론

Perception 문제의 핵심은 단순히 많은 frame을 평균내는 것이 아니다.

현재 필요한 것은 다음 세 가지이다.

1. **잘못된 frame을 판별한다.**
2. **과거 여러 frame을 기다리지 않고 매 frame마다 현재 상태를 추정한다.**
3. **주행 중 실시간 estimation과 정지 후 정밀 measurement를 분리한다.**

또한 실제 docking을 시작하기 전에 `precheck_perception.py`를 실행하여 그날의 카메라 및 AprilTag detection 상태를 사전에 점검하고 raw log를 확보하는 절차를 추가한다.
