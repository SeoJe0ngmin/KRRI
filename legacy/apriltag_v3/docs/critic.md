# Critic Review - Docking Methodology

## 목적

본 문서는 다음 네 개 방법론 문서를 실제 구현 관점에서 비판적으로 검토한 결과를 정리한 것이다.

1. `perception_methodology_v2.md`
2. `vehicle_dynamics_straight_methodology.md`
3. `vehicle_dynamics_methodology.md`
4. `controller_methodology.md`

검토 기준은 다음과 같다.

- 방법론 자체에 논리적 오류가 없는가
- 불확실한 값을 확실한 값처럼 다루고 있지 않은가
- 실제 코드로 구현했을 때 오차 때문에 좌우/전후 correction이 반복되지 않는가
- Perception / Dynamics / Controller 사이에서 동일한 오차를 중복 해석하지 않는가
- 실차 환경에서 예상되는 failure mode가 빠져 있지 않은가
- 99% 수준의 docking 성공률을 검증할 수 있는 구조인가

---

# 1. 전체 평가

전체적인 방향은 타당하다.

현재 설계의 큰 흐름은 다음과 같다.

```text
Perception
→ 현재 상태 추정

Vehicle Dynamics
→ 명령 후 결과 예측

Controller
→ 목표와 예측 결과 비교
→ 다음 명령

실제 차량
→ 다시 관측
```

기존의 단순한

```text
heading 맞춤
→ lateral 맞춤
→ heading 재보정
→ lateral 재보정
```

구조보다 훨씬 발전된 형태이다.

다만 현재 문서에는 아직 다음과 같은 위험한 가정이 일부 남아 있다.

```text
필터가 값을 출력했으니 그 값은 믿을 만하다
모델이 예측값을 출력했으니 그 값은 대표값이다
uncertainty가 작게 나오면 실제 오차도 작다
현재 속도를 알면 stop distance를 거의 결정할 수 있다
한 번의 candidate evaluation으로 좋은 command를 고를 수 있다
```

실제 환경에서는 모두 조건부로만 성립한다.

따라서 최종 구조는

> **정확한 하나의 값을 계산하는 시스템**

보다

> **불확실한 추정과 모델 오차를 인정한 상태에서 안정적으로 수렴하는 폐루프 시스템**

으로 설계해야 한다.

---

# 2. Perception - Lateral Reconstruction

## 현재 방향

현재 문서는 lateral을 독립적인 raw measurement로 filtering하지 않고,

```text
distance
+
bearing
+
filtered heading
↓
lateral reconstruction
```

하는 방향을 제안한다.

이 방향 자체는 타당하다.

---

## 문제점

문서에는 다음 형태의 식이 예시로 들어가 있다.

\[
lateral = d \cdot \sin(\hat{\psi}-\beta)
\]

하지만 이 식을 그대로 최종 구현식처럼 사용하면 위험하다.

실제 시스템에는 다음 요소가 존재한다.

- camera coordinate axis
- tag coordinate axis
- forklift coordinate axis
- camera yaw / pitch mounting offset
- camera to pivot translation
- heading sign convention
- forward / lateral sign convention

따라서 scalar 식 하나로 처리하면 좌표계 부호 오류가 숨어들 가능성이 높다.

---

## 권장 수정

lateral을 직접 공식으로 만들기보다 명시적인 좌표변환을 구성한다.

개념적으로:

\[
T_{\text{tag}\leftarrow\text{pivot}}
=
T_{\text{tag}\leftarrow\text{camera}}
T_{\text{camera}\leftarrow\text{pivot}}
\]

또는 시스템 convention에 맞는 equivalent transform을 사용한다.

그 뒤 pivot frame에서:

```text
forward
lateral
heading
```

을 추출한다.

즉 핵심은:

```text
경험식 기반 lateral 계산
```

보다

```text
Camera → Vehicle Pivot 좌표변환
```

이다.

---

# 3. Bearing 계산도 완전히 독립적이라고 착각하면 안 됨

현재 bearing 후보는 다음과 같다.

\[
\beta = atan2(t_x,t_z)
\]

하지만 `t_x`, `t_z`도 PnP 결과이므로 PnP 오차에서 완전히 독립적이지 않다.

가능하다면 비교 실험을 수행하는 것이 좋다.

## 방법 A

```text
PnP translation
→ atan2(tx, tz)
```

## 방법 B

```text
undistorted tag center pixel
→ normalized image coordinate
→ bearing
```

예:

\[
\beta
\approx
\tan^{-1}
\left(
\frac{u-c_x}{f_x}
\right)
\]

실제 로그에서 두 방법의 흔들림과 bias를 비교한 뒤 선택한다.

---

# 4. Heading - PnP Candidate Selection의 False Confidence

## 현재 방향

```text
PnP 후보
→ IMU prediction과 비교
→ 가까운 후보 선택
→ fusion
```

은 좋은 방향이다.

---

## 예상 문제

IMU prediction에 가까운 후보가 항상 실제 정답이라는 보장은 없다.

예를 들어 estimator가 이전 frame에서 이미 잘못된 branch를 잡았다고 하자.

```text
실제 heading = +1°

Estimator = -1°
```

다음 frame에서:

```text
Candidate A = +1°
Candidate B = -1°
```

이라면 estimator prediction과 가까운 B를 다시 선택한다.

그러면 잘못된 branch에 계속 lock-in 될 수 있다.

---

## 권장 수정

heading solution selection 결과를 2개가 아니라 3개 상태로 만든다.

```text
SOLUTION_A_CONFIDENT
SOLUTION_B_CONFIDENT
AMBIGUOUS
```

즉,

```text
둘 중 하나를 반드시 선택
```

하지 않는다.

불확실하면:

```text
heading_valid = False
heading_confidence = low
```

로 전달한다.

핵심 원칙:

> 모를 때는 억지로 값을 만들지 않는다.

---

# 5. Geometric Observability를 단순 각도 threshold로만 보면 안 됨

정면에 가까운 경우 heading이 불안정해질 수 있다는 방향은 맞다.

하지만 다음 조건 하나로 끝내면 안 된다.

```text
abs(observation_angle) < threshold
→ heading unreliable
```

실제 관측성은 여러 조건에 영향을 받는다.

- distance
- tag pixel size
- tag plane tilt
- perspective deformation
- candidate reprojection errors
- candidate reprojection error ratio
- image blur
- corner localization quality

따라서 최종 quality는 예를 들어 다음처럼 구성하는 것이 좋다.

```text
tag_px
distance
observation geometry
reprojection_error_candidate_1
reprojection_error_candidate_2
solution_error_ratio
```

그리고 이 값들로 heading reliability를 계산한다.

---

# 6. Perception Uncertainty를 실제 오차로 착각하면 안 됨

현재 구조는 controller에 다음 값을 넘기려 한다.

```text
forward_uncertainty
lateral_uncertainty
heading_uncertainty
```

이 방향은 매우 좋다.

하지만 가장 위험한 오해가 발생할 수 있다.

예:

```text
Kalman covariance → ±2 mm
```

라고 출력되었다고 해서

```text
실제 오차도 ±2 mm
```

라는 뜻은 아니다.

Estimator covariance는 다음이 맞다는 조건에서만 의미가 있다.

```text
motion model이 적절함
measurement noise model이 적절함
systematic bias가 충분히 제거됨
outlier가 적절히 처리됨
```

모델이 잘못되면 다음이 가능하다.

```text
Estimator:
"나는 매우 확신함"

실제:
큰 오차
```

---

## 권장 수정

초기에는 다음 용어를 사용한다.

```text
estimated_covariance
estimated_confidence
```

그리고 실험으로 calibration한다.

검증 항목:

```text
confidence가 높다고 할 때
실제 error도 정말 작았는가?
```

Calibration이 확인된 뒤 controller의 강한 decision gate로 사용한다.

---

# 7. Measurement Latency Compensation의 한계

현재 문서는 다음 형태를 사용한다.

\[
forward_{now}
\approx
forward_{measurement} - v\tau
\]

\[
heading_{now}
\approx
heading_{measurement} + \omega\tau
\]

짧은 직선 주행에서는 유용하다.

하지만 회전과 전진이 동시에 있는 경우에는 forward / lateral / heading을 각각 독립적으로 extrapolation하는 것이 정확하지 않을 수 있다.

---

## 권장 수정

가능하면 vehicle pose 전체를 SE(2) motion으로 propagate한다.

입력:

```text
v
omega
dt
```

출력:

```text
current x
current y
current heading
```

---

## Timestamp 문제

가장 중요한 점은 timestamp가 실제 image capture timestamp인지 확인해야 한다는 것이다.

다음은 서로 다를 수 있다.

```text
Image capture time
Frame receive time
Detection start time
Detection finish time
Controller decision time
Command transmit time
```

카메라 내부 buffering이 존재하면 `frame을 받은 시간`이 실제 촬영 시간과 다를 수 있다.

따라서 위 timestamp를 가능한 범위에서 모두 기록하는 것이 좋다.

---

# 8. Vehicle Dynamics - Point Prediction의 문제

현재 Dynamics 문서는 다음과 같은 형태의 모델을 제안한다.

\[
d_{stop}=g(v)
\]

이것은 초기 모델로 타당하다.

하지만 실제 시스템에서는 같은 속도에서도 stop distance가 반복마다 달라질 수 있다.

따라서 실제 관계는 다음에 가깝다.

\[
d_{stop}=g(v)+\epsilon
\]

예:

```text
v = 0.15 m/s

47 mm
52 mm
44 mm
58 mm
49 mm
```

---

## 권장 수정

모델 출력은 하나의 숫자만이 아니라 다음을 포함해야 한다.

```text
predicted_stop_distance
residual_std
percentile range
```

예:

```text
predicted = 50 mm
95% residual range = ±8 mm
```

즉 가능한 경우:

\[
P(d_{stop}|v)
\]

를 관리한다.

NN을 사용할 필요는 없다.

실험 데이터로 다음을 저장하면 충분하다.

```text
mean
median
std
95 percentile
99 percentile
worst case
```

---

# 9. d_stop = f(v, a)의 해석 주의

현재 문서는 필요하면 다음 모델을 사용할 수 있다고 한다.

\[
d_{stop}=f(v,a)
\]

이 표현은 조건부로만 타당하다.

Stop 명령을 보내기 직전에는 정상 주행 상태라서:

```text
v > 0
a ≈ 0
```

일 수 있다.

우리가 실제로 알고 싶은 것은 Stop 이후에 발생할 미래 감속이다.

따라서 Stop 전 현재 acceleration이 미래 braking deceleration을 직접 알려준다고 가정하면 안 된다.

---

## 권장 역할

### Stop 명령 전

주로:

```text
velocity
direction
vehicle mode
context
```

를 사용한다.

예:

\[
d_{stop}=f(v,\text{direction},\text{context})
\]

### Stop 명령 후

negative acceleration이 실제로 관측되면:

```text
current velocity
+
observed deceleration
↓
remaining stopping distance update
```

에 활용한다.

즉 acceleration은:

> **Stop timing predictor**

보다는

> **vehicle response detector + braking phase estimator**

에 더 가깝다.

---

# 10. IMU Linear Acceleration 자체도 불확실함

Raw IMU acceleration을 그대로 사용하면 다음의 영향을 받을 수 있다.

- chassis vibration
- floor unevenness
- IMU mounting angle
- pitch
- gravity projection
- sensor bias

따라서 한 frame에서 acceleration sign이 바뀌었다고 바로 판단하면 안 된다.

---

## 권장 구조

```text
Raw IMU
↓
Vehicle frame transform
↓
Gravity compensation
↓
Filtering
↓
Threshold + persistence time
↓
Motion state
```

예:

```text
negative acceleration이
N ms 이상 지속
AND
velocity 감소가 동시에 관측
↓
deceleration_started = True
```

처럼 처리하는 것이 안전하다.

---

# 11. Rotation Dynamics - 좌우 대칭 가정 금지

현재 모델:

\[
\theta_{stop}=h(\omega)
\]

는 좋은 초기 모델이다.

하지만 실차에서는 좌우 회전이 완전히 대칭이라는 보장이 없다.

가능한 관계:

\[
h_L(\omega)
\neq
h_R(\omega)
\]

따라서 사전실험은 최소한 다음을 분리한다.

```text
Left rotation
Right rotation
```

필요하면:

```text
forward / reverse
steering state
load state
```

등도 별도로 확인한다.

---

# 12. Low-speed Final Approach의 숨은 문제

속도를 낮추면 일반적으로 다음이 좋아진다.

- stopping distance 감소
- overshoot 감소
- latency 영향 감소

하지만 너무 작은 명령에서는 다음 현상이 발생할 수 있다.

```text
명령
→ 안 움직임
→ 조금 더 명령
→ 갑자기 움직임
```

즉:

- deadzone
- stiction
- minimum motor pulse
- minimum steering response

가 존재할 수 있다.

---

## 반드시 측정할 항목

```text
minimum reliable forward increment
minimum reliable reverse increment
minimum reliable left rotation
minimum reliable right rotation
```

Final controller는 이 최소 제어량보다 작은 correction을 명령해서는 안 된다.

---

# 13. Controller - 현재 구조만으로 Ping-Pong이 완전히 사라지지 않음

현재 Controller에는 이미 다음이 들어 있다.

```text
Hysteresis
Minimum command interval
Minimum expected improvement
State uncertainty check
Command reversal prevention
```

좋은 방향이다.

하지만 실시간으로 candidate score를 매 frame 다시 계산하면 다음 문제가 가능하다.

```text
Frame 1:
LEFT cost < RIGHT cost

Frame 2:
RIGHT cost < LEFT cost

Frame 3:
LEFT cost < RIGHT cost
```

그러면 다시:

```text
LEFT
RIGHT
LEFT
RIGHT
```

형태의 command chatter가 발생할 수 있다.

---

# 14. 반드시 추가해야 할 Controller State Machine

가장 중요한 수정 사항이다.

Controller를 다음 상태로 명시적으로 분리한다.

```text
OBSERVE
↓
DECIDE
↓
EXECUTE
↓
STOPPING
↓
SETTLE
↓
VERIFY
↓
필요한 경우 다시 DECIDE
```

---

## 핵심 규칙

### EXECUTE

선택한 motion을 수행한다.

### STOPPING

Stop 명령 이후 차량이 감속 중인 상태.

```text
새로운 반대 correction 금지
```

### SETTLE

차량이 거의 멈춘 후 센서와 chassis 진동이 안정되는 구간.

```text
새 correction 금지
```

### VERIFY

Perception 상태가 충분히 안정적일 때만 error를 다시 평가한다.

이 구조가 없으면 차량이 아직 관성으로 움직이는 중인데 다음 frame을 보고 반대 명령을 내릴 수 있다.

이것이 기존 ping-pong을 다시 만드는 가장 큰 원인 중 하나이다.

---

# 15. Correction을 한 Frame으로 결정하면 안 됨

예:

```text
Frame 1: lateral = +12 mm
Frame 2: lateral = -3 mm
Frame 3: lateral = +11 mm
```

Frame 1만 보고 즉시 correction을 수행하면 measurement noise를 실제 error로 착각할 수 있다.

그렇다고 다시 30-frame median으로 돌아가야 하는 것은 아니다.

필요한 것은 **evidence persistence**이다.

예:

```text
estimated error가 같은 방향으로 일정 시간 지속

AND

error가 uncertainty-adjusted threshold보다 큼

AND

correction의 expected improvement가 충분함
```

일 때만 correction을 수행한다.

---

# 16. Minimum Expected Improvement를 Controller의 핵심 조건으로 승격

예:

```text
현재 lateral error = 15 mm

small correction prediction:
-12 mm ± 10 mm
```

이 경우 correction 결과 자체가 매우 불확실하다.

반면:

```text
현재 error = 40 mm

predicted correction:
-25 mm ± 5 mm
```

라면 correction의 가치가 높다.

따라서 개념적으로 다음 조건을 사용하는 것이 좋다.

\[
Expected\ Improvement
>
Model\ Uncertainty
+
Switching\ Cost
\]

정확한 수식은 실험을 통해 조정한다.

핵심은:

> **오차가 존재한다는 이유만으로 움직이지 않는다.**

---

# 17. Candidate Cost Function의 단위 문제

현재 기본 cost:

\[
J =
w_f e_f^2
+
w_l e_l^2
+
w_h e_h^2
\]

여기에는 단위 문제가 있다.

```text
forward = m
lateral = m
heading = degree 또는 rad
```

서로 단위가 다르므로 weight 의미가 불명확해질 수 있다.

---

## 권장 수정

허용오차로 normalize한다.

\[
J=
w_f
\left(
\frac{e_f}{T_f}
\right)^2
+
w_l
\left(
\frac{e_l}{T_l}
\right)^2
+
w_h
\left(
\frac{e_h}{T_h}
\right)^2
\]

여기서:

```text
T_f = forward tolerance
T_l = lateral tolerance
T_h = heading tolerance
```

이렇게 하면 각 항은:

> 허용오차 대비 현재 오차가 몇 배인가

라는 공통 의미를 갖는다.

---

# 18. 1-step Candidate Controller의 한계

1-step controller는 다음 상황에서 잘못된 선택을 할 수 있다.

```text
Command A
→ 바로 다음 pose는 크게 좋아짐
→ 그 다음 상태는 나빠짐

Command B
→ 바로 다음 pose는 조금만 좋아짐
→ 이후 수렴이 쉬워짐
```

1-step controller는 A를 선택할 수 있다.

따라서:

```text
처음에는 1-step
↓
실험에서 local optimum / oscillation 발생
↓
2~N step predictive control 또는 MPC
```

로 확장하는 것이 좋다.

초기 구현에서 MPC가 필수는 아니지만, 1-step이 항상 좋은 선택을 한다고 가정하면 안 된다.

---

# 19. "불확실하면 기다린다" 전략의 Deadlock

현재 Controller는 다음 전략을 포함한다.

```text
error 작음
uncertainty 큼
→ 기다림
```

하지만 geometry 자체가 좋지 않은 경우에는 기다려도 uncertainty가 줄지 않을 수 있다.

예:

```text
정면
멀리 있음
tag 작음
```

그러면:

```text
WAIT
WAIT
WAIT
...
```

상태에 빠질 수 있다.

---

## 반드시 Fallback 필요

예:

```text
WAIT
↓
일정 시간 동안 confidence 개선 없음
↓
Fallback
```

Fallback 후보:

```text
완전 정지 후 재측정
관측 geometry가 좋아지는 위치로 이동
다른 tag 사용
다른 sensor 기준 사용
operator intervention
abort
```

---

# 20. Zone-based Controller의 Chattering

예:

```text
Near Zone < 1.0 m
```

일 때:

```text
0.99 m → Near
1.01 m → Far
0.98 m → Near
```

처럼 measurement noise로 controller mode가 계속 바뀔 수 있다.

---

## 해결

Zone transition에도 hysteresis를 넣는다.

예:

```text
Far → Near:
distance < 1.0 m

Near → Far:
distance > 1.2 m
```

Controller mode도 state machine처럼 안정적으로 유지해야 한다.

---

# 21. Perception Error와 Dynamics Error를 섞지 않도록 주의

Dynamics 실험에서 측정값이 늦게 들어오면 다음 두 오차가 섞인다.

```text
실제 stopping distance
+
measurement latency error
```

예를 들어 실제 차량은 5 cm 더 움직였는데 measurement latency 때문에 8 cm처럼 보일 수 있다.

그러면 Dynamics 모델이:

```text
차량은 항상 8 cm 더 감
```

으로 잘못 학습된다.

---

## 권장 순서

```text
Perception timestamp / latency 정리
↓
Dynamics logging
↓
Dynamics model fitting
```

Perception timing이 먼저 어느 정도 안정되어야 Dynamics model을 신뢰할 수 있다.

---

# 22. Vehicle Dynamics 문서 중복 문제

현재 다음 두 문서가 겹친다.

```text
vehicle_dynamics_straight_methodology.md
vehicle_dynamics_methodology.md
```

장기적으로 두 문서에서 서로 다른 model / threshold / 설명이 생길 가능성이 있다.

---

## 권장 구조

```text
vehicle_dynamics_methodology.md
├ Straight Dynamics
├ Rotation Dynamics
├ Stop Models
├ Uncertainty
└ Experiment Plan
```

으로 통합한다.

그리고 기존 Straight 전용 문서는:

```text
experiments/forward_dynamics_test_plan.md
```

처럼 실험 문서로 변경하는 것이 좋다.

---

# 23. 가장 중요한 누락 - 99% 정확도의 정의

현재 목표는 "거의 99% 정확한 docking"이다.

하지만 이것이 수치적으로 정의되어 있지 않으면 개선 여부를 평가할 수 없다.

반드시 성공 조건을 정해야 한다.

예:

```text
99% of trials satisfy:

|forward error| <= T_f
|lateral error| <= T_l
|heading error| <= T_h
```

여기서 tolerance를 실제 요구사항에 맞게 정한다.

---

## 평균값만 보면 안 됨

예:

```text
mean lateral error = 3 mm
```

라고 하더라도,

```text
100번 중 5번
50 mm error
```

가 발생하면 높은 신뢰성이 필요한 docking 시스템에서는 실패일 수 있다.

따라서 다음 KPI를 관리한다.

```text
median error
mean error
95 percentile
99 percentile
maximum error
success rate
average correction count
maximum correction count
docking time
abort rate
```

---

# 24. 예상되는 대표 Failure Modes

## Failure 1 - Wrong PnP branch lock-in

```text
잘못된 heading 선택
→ IMU prediction도 그 branch 기준
→ 계속 잘못된 branch 선택
```

대응:

```text
AMBIGUOUS state
reinitialization
additional geometry check
```

---

## Failure 2 - False Confidence

```text
Kalman covariance 작음
→ controller가 강하게 신뢰
→ 실제 systematic bias 존재
```

대응:

```text
offline confidence calibration
systematic bias test
```

---

## Failure 3 - Controller Ping-Pong

```text
Left correction
→ inertia 중 Right 판단
→ Right command
→ 다시 Left
```

대응:

```text
STOPPING / SETTLE state
command latch
reversal guard
```

---

## Failure 4 - Minimum Motion Deadzone

```text
작은 correction
→ 차량 안 움직임
→ command 증가
→ 갑자기 overshoot
```

대응:

```text
minimum controllable motion 실험
```

---

## Failure 5 - Perception/Dynamics Error Mixing

```text
measurement latency
→ stop distance처럼 관측
→ 잘못된 dynamics model
```

대응:

```text
timing calibration first
```

---

## Failure 6 - Wait Deadlock

```text
confidence low
→ 기다림
→ geometry 그대로
→ confidence 개선 안 됨
```

대응:

```text
timeout + fallback state
```

---

## Failure 7 - Zone Chattering

```text
Far / Near 경계
→ measurement noise
→ mode 반복 변경
```

대응:

```text
zone hysteresis
```

---

# 25. 권장 Controller 최종 State Machine

```text
OBSERVE
   ↓
DECIDE
   ↓
EXECUTE
   ↓
STOPPING
   ↓
SETTLE
   ↓
VERIFY
   ├─ success → DONE
   ├─ confident error → DECIDE
   ├─ ambiguous → REOBSERVE
   └─ repeated failure → FALLBACK / ABORT
```

---

## OBSERVE

Perception state 확보.

조건:

```text
measurement_valid
confidence acceptable
timestamp valid
```

---

## DECIDE

candidate commands 평가.

고려:

```text
pose error
dynamics prediction
prediction uncertainty
command switching cost
minimum controllable motion
```

---

## EXECUTE

선택 command 수행.

이 상태에서는 다른 correction을 새로 결정하지 않는다.

---

## STOPPING

Stop 이후 실제 감속 중.

```text
반대 command 금지
```

---

## SETTLE

차량 정지와 vibration 안정 대기.

조건:

```text
velocity sufficiently small
yaw_rate sufficiently small
position stable
```

---

## VERIFY

최종 pose 재평가.

한 frame이 아니라 일정한 evidence persistence를 요구한다.

---

# 26. 수정 우선순위

## Priority 1

Controller에 다음 state machine 추가.

```text
EXECUTE
→ STOPPING
→ SETTLE
→ VERIFY
```

---

## Priority 2

Dynamics 모델을

```text
point prediction
```

에서

```text
prediction + residual uncertainty
```

로 변경.

---

## Priority 3

Perception uncertainty를 실제 정확도로 해석하지 않도록 명시하고 calibration 절차 추가.

---

## Priority 4

Lateral 계산을 scalar 경험식보다 명시적 coordinate transform 기반으로 변경.

---

## Priority 5

PnP ambiguity에:

```text
AMBIGUOUS / UNKNOWN
```

상태 추가.

---

## Priority 6

Camera / perception / command timestamp 체계 구축.

---

## Priority 7

가속도를 미래 braking을 직접 예측하는 값처럼 사용하지 않도록 역할 수정.

---

## Priority 8

Minimum controllable motion / actuator deadzone 실험 추가.

---

## Priority 9

Candidate cost function normalization.

---

## Priority 10

Zone hysteresis + perception timeout + fallback 추가.

---

## Priority 11

Straight / General Vehicle Dynamics 문서 통합.

---

## Priority 12

99% 성공 기준과 평가 KPI 명시.

---

# 최종 결론

현재 방법론의 전체 방향은 맞다.

하지만 그대로 구현하면 여전히 다음 형태의 반복 correction이 발생할 가능성이 있다.

```text
Perception noise
↓
candidate ranking 변경
↓
반대 correction
↓
아직 관성으로 움직이는 중
↓
다음 correction
↓
ping-pong
```

이를 막기 위해 가장 중요한 것은 단순히 filter나 dynamics model을 더 정교하게 만드는 것이 아니다.

핵심은:

1. **불확실한 값을 불확실하다고 표현한다.**
2. **모를 때 억지로 하나의 값을 선택하지 않는다.**
3. **Dynamics prediction에도 residual uncertainty를 포함한다.**
4. **한 correction이 끝나기 전에 다음 correction을 허용하지 않는다.**
5. **STOPPING / SETTLE / VERIFY 상태를 명시적으로 둔다.**
6. **작은 error가 아니라 확실하고 지속적인 error에만 반응한다.**
7. **실패하거나 관측이 불가능한 경우 fallback을 둔다.**

최종 목표는:

```text
완벽한 deterministic model
```

이 아니라,

```text
불확실성이 존재해도
오작동하거나 oscillation하지 않고
목표 pose로 안정적으로 수렴하는
closed-loop docking system
```

을 만드는 것이다.
