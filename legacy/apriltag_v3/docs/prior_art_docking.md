# 선행연구 조사 — AprilTag 지게차 도킹: 정지-관측 진동 제거, 지연·관성 보상

조사일 2026-09-10. 세 갈래(A 마커 기반 도킹 시스템 / B pose 필터링·IMU 융합 / C 지연·관성 보상)로 나눠
영·한 검색 후 핵심 자료는 본문(PDF)까지 읽음. "초록 수준"은 본문 접근 실패로 초록·2차자료만 확인한 것.

## 우리 상황 (관련도 판단 기준)
- 전동지게차, 단일 AprilTag(tag36h11, 30 cm), RealSense D435i 컬러 1080p@30fps + 자이로 200 Hz.
- ~10 m 에서 접근, 태그면 앞 ~3 m 정지. 목표 ±3 cm lateral / ±2° heading.
- CAN 조이스틱 프레임 5개 명령: stop / forward / backward / 제자리회전 L / R. 곡선주행 불가(팀 결정). 스로틀 크기는 준비례 가능성.
- 명령→동작 지연 ≈0.5 s(주행) / ≈0.85 s(회전 시작). stop 후 0.28 m/s 에서 ≈14 cm 코스팅, 회전 ≈1.3°.
- 현재 루프: 정지 → 30프레임 중앙값 → 동작 1개(자이로 폐루프 회전 or T초 개루프 직진) → 정지 → 반복.
- 문제: 7–10 m 에서 lateral 잡음 60–145 mm(허용 30 mm) → 노이즈 추종 → 좌우 왕복, 21스텝 수렴. 수요기업은 부드럽고 결단력 있는 동작 원함.

---

## 0. 결론 — 설계 결정 (A·B·C 종합)

```
[10 → ~5 m]  방위각(bearing)+거리만 사용. lateral/yaw 로 결정하지 않음.
             직진하며 방위 감시 → |방위| > k·σ(d) 일 때만 정지·회전.                   (A)
[~5 m 정렬점] 한 번 정지 → N프레임 → 회전–직진–회전 1회로 접근선 위에 올라탐.          (A: autodock / Poreski)
[5 → 3 m]    정속(가능하면 0.15–0.2 m/s) ≥2–3 s 직진, heading 소폭 보정만.             (C)
[정지 판단]  x̂(t+τ) = 명령버퍼 순방향 적분; 남은거리 ≤ v̂·τ + c(v̂) + k·σ 이면 stop. 한 번에. (C)
[정지 후]    1회 재측정. 잔차 있으면 학습형 펄스 1회.                                    (C)
```

1. **lateral 은 태그 방위각(atan2, 픽셀 정밀) + 필터된 heading 으로 재구성.** 프레임별 태그 yaw 로 회전시켜 구하면 yaw 1° × 8 m = 140 mm 가 lateral 잡음으로 들어옴 — 우리 60–145 mm 의 주범으로 추정. (B: Abbas 2019, Adámek 2023) → `detection_pose.py` 의 lateral 계산식 확인.
2. **정면 관측의 heading 튐은 잡음이 아니라 평면 PnP 2중해(flip).** R 을 키우는 것으론 안 됨. 두 해를 받아 필터 예측과 가까운 해 선택(Liu 2021), 오차비 > 0.2 면 yaw 갱신 생략·translation 만 갱신(PhotonVision), 자이로 각속도와 대조해 flip 검출(Springer & Kyas). 정면 ±15–25° 원뿔은 yaw 신뢰 안 함(Richter). (A, B)
3. **heading 은 자이로 앵커.** 카메라 yaw 는 조건 만족 시에만 갱신. 회전 중 predict-only, 재포착 시 마할라노비스 게이트 통과하면 갱신. 최종 자세가 정면(=모호성 정중앙)이라 최종 heading 은 자이로+기하. 장기적으론 태그 2개(오프셋/기울임). (A, B)
4. **추정기 = 축별 등속 KF, 구동기 모델 없음.** R 은 태그 픽셀면적·시야각 함수(Adámek 함수형), 3σ/마할라노비스 게이트, 카메라 타임스탬프로 `x_meas + v̂·τ_cam` 외삽(Larsen). 속도 v 는 측정 주도(큰 Q) — 명령 게인은 run 마다 10–30% 변함. (B, C)
5. **정지 판단에만 구동기 모델.** `D(v) = τ_d·v + c(v)` (Yaskawa 특허 형태). 우리 14 cm ≈ 0.28×0.5 는 순수 지연항으로 설명되므로 30 fps 궤적에서 τ_d 와 감속 램프 a 를 분리 식별. 변동성 예산 `σ_D² ≈ (τ·σ_v)² + (v·σ_τ)² + σ_c²` — **σ_τ 실측이 최우선**(0.05–0.1 s 만으로 1.4–2.8 cm). (C)
6. **마지막 구간은 "정속 ≥2–3 s 후 한 번에 정지".** 2 s 미만 이동은 순수 과도상태라 "빨리 가다 멈추고 크립 재시동" 은 불리. 잔차는 학습형 펄스 표(펄스폭→거리, 매 회 자이로/카메라로 갱신). (C)
7. **회전 정지**: `θ̂ + ω̂·τ_r,stop + c_r(ω̂)`; 정지지연은 시작지연(0.85 s, 토크 형성 포함)과 별도로 자이로로 실측. 10° 미만 소각은 정속 구간이 없으니 펄스 표. 회전 직전 정지 상태에서 자이로 바이어스 재추정. (C)
8. **구동기 파라미터 온라인 갱신**(정지마다 τ, a 또는 D(v) 이동평균/RLS), 적재·비적재 별도. 범위 이탈 시 경고. (C: Yaskawa 캘리브레이션 관행, TASC 적응 파라미터)
9. **데드밴드/임계는 실측 σ(d) 기반.** Poreski 임계각 스윕: 0.5°→23 %, 2°→53 %, 4°→27 % — 최적점 존재. 노이즈 아래 임계는 정지 과다로 실패. (A)
10. **현실적 목표**: ±3 cm/±2° 는 마지막 3–5 m 에서 정렬을 끝낼 때만. 6–8 m 에서 NMPC 연속제어도 lateral 4.45±1.74 cm(Pang 2025). 인지 3 mm/0.15° 인데 최종 4.5 cm/0.4°(Richter) — 실행(지연·관성) 이 병목. (A)

### 다음 현장 측정 목록
1. byte2 스윕 137/147/167/187 × 3 s, 카메라로 속도·정지 코스팅 → 속도표 + D(v) (비례 여부 판별 포함)
2. 정지 지연 τ_d 와 지터 σ_τ — stop 명령 시각 vs 30 fps 궤적, 5회 이상
3. 회전 정지 지연 τ_r,stop — 자이로 200 Hz
4. 정적 R(d, φ) 데이터: 3/5/7/10 m × tilt 0/10/20/40°, 각 100프레임 (Adámek 피팅)
5. 접근 중 프레임별 로그 `live_pose --log` 1회 (필터 오프라인 개발용; 현재 measure.jsonl 은 중앙값만 있음)

---

## A. 마커 기반 도킹 시스템 (지게차 / AGV / 충전 도킹)

### 핵심
- 연속 주행이 표준. 정지-관측-1동작은 소수(학사논문·소형 AMR·ROS 데모)이고, 임계가 작으면 정지 과다로 실패한다고 보고.
- 7–10 m 에서 단일 마커 lateral/yaw 를 믿는 시스템 없음. 1–3.5 m 에 staging/marker-centering 지점(Nav2 0.75 m, Richter 3.45 m, autodock predock). 원거리는 bearing 만.
- 정렬-후-접근(2단계) 지배적. 횡오프셋은 회전–직진–회전 1회로 제거(autodock `parallel_correction`, Poreski β=90°−α, way=|v|·sinα).
- 지연·관성은 거의 모델링 안 함. 예외: Poreski 거리보정함수 E=8.92·ln(W)−28.35 cm; Richter "인지 3 mm 인데 실행 4.5 cm".
- 지게차 요구정밀도: Köhne 리뷰 lateral ≤20 mm/각 ≤2.9°; Kita ≤50 mm/≤3°; Ren ±6 mm(1000회); MIT 는 10–14 cm 에서도 삽입 성공.

### 항목
| # | 자료 | 요지 | 적용점 | 관련도 |
|---|---|---|---|---|
| A1 | Walter, Karaman, Frazzoli, Teller, *Closed-loop Pallet Manipulation in Unstructured Environments*, IROS 2010. https://agile.csail.mit.edu/publications/walter10a.pdf (Teller ICRA 2010 https://dspace.mit.edu/handle/1721.1/62144 ; Walter JFR 2015 DOI 10.1002/rob.21539) | 2700 kg Toyota 지게차, LIDAR 팔레트 검출, KF 로 접근 중 자세 갱신, 연속 조향 δ=Ky·atan(ey)+Kθ·eθ. 35/38, 30/30 성공. 7.5 m/횡 3 m 이상 시작은 검출 실패. | 대형 지게차도 필터+연속 접근. 7.5 m 이상 관측 불신 경험치. | 높음 |
| A2 | Seelinger & Yoder, *Automatic visual guidance of a forklift engaging a pallet*, RAS 2006, DOI 10.1016/j.robot.2005.10.009 | 팔레트에 원형 피듀셜 3개 + 포크 마커, MCSM. 98 % (2차자료). | 피듀셜 지게차 진입 원조. 마커 다중 배치. | 중 (초록) |
| A3 | Kita & Kato (AIST), *Approach and Fork Insertion to Target Pallet Based on Image Measurement*, Sensors 2026 26(1):154, DOI 10.3390/s26010154 | 리치형 AGF, 5 Hz 비전 + 100 Hz 오도메트리 보간, pure pursuit, 1 km/h, 3 m 시작. 성공기준 횡 ≤50 mm/요 ≤3°. | 저주기 비전을 자이로/오도메트리로 보간하는 구조. | 높음 |
| A4 | Ren et al., *Deep Learning-Based Intelligent Forklift Cargo Accurate Transfer System*, Sensors 2022 22(21):8437, DOI 10.3390/s22218437 | RGB-D 키포인트, MPC 류 연속 궤적, 1000회 최대 ±6 mm. | 연속 제어로 mm 급 가능. | 중 |
| A5 | Köhne et al., *Methods for autonomous load handling with forklifts: review*, IJAMT 2026, DOI 10.1007/s00170-026-17516-9 | 62편 리뷰. 횡 ≤20 mm/각 ≤2.9° 결정적. | 요구정밀도·문헌 지도. | 중 |
| A6 | Kesuma et al., ICAIIC 2023, DOI 10.1109/ICAIIC57133.2023.10066999 | 팔레트 ArUco + YOLOv5n, 거리오차 2.28 cm. | 측정 위주. | 낮음–중 |
| A7 | 박지훈·김민환·이석·이경창, 네트워크 기반 무인지게차 팔레트 자율적재, 제어로봇시스템학회논문지 17(10) 2011. https://scienceon.kisti.re.kr/srch/selectPORSrchArticle.do?cn=JAKO201101152699386 | CLARK CRX-10, 단일 카메라, CAN 분산 PID, 1.5 m 시작, 10 s 내 10 cm. | 국내 CAN 전동지게차 실차. | 중 |
| A8 | 현대위아 무인지게차 2026 (kcenews.kr/9615, fnnews 202609070849505835) | 뎁스+2D LiDAR, ICP 실시간 보정, 연속. | 산업 맥락. | 낮음–중 |
| A9 | Richter, Bohlig, Nüchter, Schilling (Würzburg), *Advanced Edge Detection of AprilTags for Precise Docking*, 2022. https://robotik.informatik.uni-wuerzburg.de/telematics/download/ta2022_1.pdf | 710 kg 로봇. 3단계(접근 → marker-centering 3.45 m, 요각 25–27° → 도킹 1 m), 단계마다 정지·재측정. 인지 3 mm/0.15°, 최종 4.5 cm/0.4°, σy 2.57 cm — dead-reckoning·오버슈트 탓. **정면 ±25° 원뿔 회피.** | 정면 회피 규칙; 실행이 병목이라는 증거. | 높음 |
| A10 | Poreski, *Autonomous Docking with Optical Positioning*, BSc Univ. Hamburg TAMS 2016. https://tams.informatik.uni-hamburg.de/publications/2016/BSc_Kolja_Poreski.pdf ; 코드 https://github.com/TAMS-Group/turtlebot_visual_docking | 우리와 같은 명령세트(제자리회전/직진) 5단계 정지-관측-동작. IMU 폐루프 회전, 개루프 직진 + 가감속 보정함수 E(W)=8.92·ln(W)−28.35 cm. 정지 시 N=10, 주행 중 N=40 평균. 임계각 스윕 0.5°→23 %, 2°→53 %, 4°→27 %. | 우리 컨트롤러 축소판: 데드밴드 실측 잡기, 관성 보정함수, 횡오프셋 1회 기하. | 높음 |
| A11 | osrf/autodock. https://github.com/osrf/autodock (autodock_examples/configs/mock_robot.yaml) | predock(5샘플 평균, 횡 > 0.16 m 면 parallel_correction=회전–직진–회전) → steer_dock → last_mile(0.5 m 부터 오도메트리, 정지거리 0.10 m) → 실패 시 0.5 m 후퇴 재시도. stop_yaw_diff 0.03 rad, stop_trans_diff 0.02 m. | 정지-회전형 상태기계 참조 구현. | 높음 |
| A12 | Nav2 opennav_docking. https://github.com/ros-navigation/navigation2/tree/main/nav2_docking ; https://docs.nav2.org/tutorials/docs/using_docking.html ; AprilTag 예제 https://automaticaddison.com/autonomous-docking-with-apriltags-using-nav2-ros-2-jazzy/ | staging 0.75 m → 마커 재검출하며 연속 접근(graceful controller, v 0.1–0.15 m/s), 저역통과 filter_coef 0.1, docking_threshold 0.02–0.05 m, 실패 시 후퇴 3회. | 현행 ROS 표준. heading 성분만 이산화해 차용. | 높음 |
| A13 | Oh & Kim, *Regression-Based Docking System for AMRs*, Sensors 2025 25(12):3742, DOI 10.3390/s25123742 | 1–2 m, ArUco 2개, 회전→P접근. 회귀 2 cm/3.07° vs SolvePnP 58.5 cm/6.6°. | PnP yaw 불안정 실측; 회전-후-직진 동일. | 중–높음 |
| A14 | Wang, Shan, Yue, Wang, *Autonomous Target Docking of Nonholonomic Mobile Robots Using Relative Pose Measurements*, IEEE TIE 68(8) 2021, DOI 10.1109/TIE.2020.3001805 | approaching → switching region → docking; 간헐 관측용 EKF 상대자세. | 필터 갱신→결정 구조 근거. | 중–높음 (초록) |
| A15 | Dai & Lee (영남대), *Multi-Sensor Fusion for AMR Docking: LiDAR + YOLO AprilTag + Depth*, Electronics 2025 14(14):2769, DOI 10.3390/electronics14142769 | 횡 먼저 → 요 미세조정 → 직진. 태그 yaw 불신, LiDAR 대칭성으로 heading. | 단계 분리; 뎁스로 거리 보강. | 중 (초록) |
| A16 | Grzechca et al., Fuzzy AGV docking, Sensors 2025, PMC12526989 | ToF 3개, 전환거리 실험. <60 mm/<9°. | 낮음–중 | |
| A17 | Lu et al. (SJTU), *Trailer Tag Hitch*, ICCSIP 2022, DOI 10.1007/978-981-99-0617-8_20 | 대형 트랙터 AprilTag 후진 결합, 횡·종 분리 시각서보, 254회 95 %. | 대형·저응답 차량 실용성; 횡/종 분리. | 중 |
| A18 | Pang et al., *Robust Docking Maneuvers for Autonomous Trolley Collection*, arXiv 2509.07413 (2025) | 6–8 m 시작, NMPC + ESO(외란·지연). 횡 4.45±1.74 cm, 각 2.16±0.64°. | 원거리 시작 시 도달 가능 정밀도 참고치. | 중 |
| A19 | Adámek et al., Sensors 2023 23(12):5746, DOI 10.3390/s23125746 | (B1 과 동일) yaw 분산 정면 피크, x 분산은 거리 의존. R(d,β) 적응 EKF > 고정 R. | 관측성 게이팅의 원리적 버전. | 높음 |
| A20 | Abbas et al., Sensors 2019 19(24):5480, PMC6960891 | (B2) 관측각이 오차 주원인. 25–75°·화상 중앙에서 최고. | 태그를 화상 중앙에 두고 측정. | 높음 |
| A21 | Springer & Kyas, arXiv 2203.10180 (2022) | (B 참조) 30 cm 태그 1–3 m, 플립 불연속 2.6–16.1 %, 멀수록 증가. | 중앙값 30프레임으로 플립(양봉) 제거 불완전. | 중–높음 |
| A22 | Liu, Schofield, Shan, arXiv 2104.12954 (2021) | (B 참조) 오도메트리 KF 예측으로 2중해 선택. | 자이로 예측으로 해 선택. | 중 |
| A23 | Long-Duration Fully Autonomous Rotorcraft UAS, arXiv 1908.06381 (2019) | 태그 크기별 거리-오차 곡선, 48 cm 를 4 m 2σ 4.5 cm 로 선정. RLS. | 요구정밀도 → 태그 크기·개시거리 역산. | 중 |
| A24 | 정세영·**박찬호(한국철도기술연구원)**·김성주·임지원·황성호, 카메라를 사용한 아루코 마커 기반 차량 자율 선적 측위, 한국자동차공학회 춘계 2025. https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE12278748 | **철기연 공저.** DBpia 유료로 미확인. | 팀 내부에서 원문 확보 권장. | 중 (미확인) |
| A25 | 박상배 외, 3차원 카메라 기반 AGV 자율주행·도킹, 산업기술연구논문지 28(2) 2023. KCI ART002977399 | 충전 도킹 평균 19.7 mm. | 낮음–중 | |
| A26 | Bostelman & Hong, NISTIR 8140 *Review of Research for Docking AGVs*, 2016, DOI 10.6028/NIST.IR.8140 | Nygårds 팔레트 도킹 ±5 mm; EKF 추측항법+비전 5.1±3.0 mm; 정밀 사례 전부 ≤0.2 m/s. | 정밀도 벤치마크, 저속 상한 근거. | 중 |
| 기타 | Yilmaz & Temeltas Robotica 2024 (전환 시 추정 점프→잠깐 정지/가중평균); Biernacki & Ziebinski Sci Rep 2025 (LiDAR 반사마커 1 cm/0.05°); Bolanakis IEEE/ASME 2021 (바닥 QR, approximate→precision); Adlink-ROS/apriltag_docking (D435, jog 0.2 m, tune_angle 0.42 rad); Mateos AprilTags3D arXiv 2001.08622 (기울어진 태그 2개, 99 %); MiR EP4321955B1 (LiDAR vs 오도메트리 슬립 검출). | | | 낮음–중 |

---

## B. 마커 pose 잡음 특성·필터링·IMU 융합

### 핵심
- 마커 자세 잡음은 거리·픽셀면적·시야각의 해석적 함수로 모델링됨(Adámek 2023). 위치 분산 ∝ 정규화면적^(−p), **yaw 분산은 정면(φ≈0) 가우시안 피크**.
- 정면 heading 불안정은 잡음이 아닌 평면 PnP 2중해(flip). **이동 중엔 틀린 해가 재투영오차가 더 낮은 경우 실험 확인**(Liu 2021) → 예측값과의 3D 오브젝트공간 오차를 함께 써야 함.
- 우리 7–10 m lateral 60–145 mm 는 대부분 yaw 잡음 × 거리(8 m × 1° ≈ 140 mm). Abbas 2019 도 "보정 안 된 방향 불확실성이 위치오차 주원인".
- 간헐 관측 도킹 EKF 표준형: 목표 자세를 바디프레임 상태로, 오도메트리/자이로 예측, 마커 보이면 갱신, 미관측은 predict-only(Wang 2021, robot_localization).
- 실무: apriltag `estimate_tag_pose_orthogonal_iteration()` 이 두 해(pose1/err1, pose2/err2) 반환; PhotonVision 은 err 비 > 0.2 기각. apriltag_ros covariance 는 0 (issue #122) → 자체 R 필수.
- RealSense RGB 파이프라인 지연 ≈66 ms(realsense-ros #2686) + 검출 → 측정은 ~100 ms 과거.
- 못 찾은 것: fiducial 전용 α-β 문헌; 30 cm 태그 7–10 m 특성화 논문; 국내 2015 무인지게차 도킹(이상진·송재복) 원문.

### 항목
| # | 자료 | 요지 | 적용점 | 관련도 |
|---|---|---|---|---|
| B1 | Adámek, Brablc, Vávra, Dobossy, Formánek, Radil, *Analytical Models for Pose Estimate Variance of Planar Fiducial Markers*, Sensors 2023 23(12):5746, DOI 10.3390/s23125746, PMC10300747 (전문) | ArUco 112 mm, 0.4–1.4 m, 40 100장. yaw: σ²=p₁S_n^(−p₂)e^(−φ/p₃)+p₄/(90−|φ|)+p₅; 위치: σ²=p₁S_n^(−p₂)/(90−|φ|)+p₃. 야코비안 전파, EKF 융합. 적응 RMSE 12.8 mm vs 고정 15.4–22.3 mm. | R(d, tilt) 함수형 채택. 입력은 태그 픽셀면적. 3/5/7/10 m × tilt 정적 데이터로 피팅. | 높음 |
| B2 | Abbas, Aslam, Berns, Muhammad, *Analysis and Improvements in AprilTag Based State Estimation*, Sensors 2019 19(24):5480, PMC6960891 (전문) | MoCap 기준. 70 cm 에서 카메라 yaw 70–110° 변동 시 분산 0.007→194 cm². yaw 불확실성이 주원인. soft yaw 보정, 짐벌, GP 센서모델. | lateral 잡음 주범 = yaw. 태그 화상 중앙 유지. | 높음 |
| B3 | Rijlaarsdam, Zwick, Kuiper, *A novel encoding element for robust pose estimation using planar fiducials*, Frontiers Robotics & AI 2022, DOI 10.3389/frobt.2022.838128 (전문) | 3.3 m 에서 표준 ArUco 는 −10~+10° 피치 구별 불가, flip 다발(원근효과 소실). 해법: 돌출 반사요소(Mantis). | 정면 heading 잡음의 정량 원인. 마커 못 바꾸면 소프트 게이팅. | 중–높음 |
| B4 | Hinderer, Scheffler, Yang, *ArUco Marker Placement for Planar Indoor Localization*, arXiv 2509.17345 (2025) | 선형 KF [x,vx,y,vy] 등속 + 적응 R. 정면 마커가 더 나쁨. | lateral/forward 최소 필터 구조 선례. | 중 |
| B5 | Kallwies, Forkel, Wuensche, *Determining and Improving the Localization Accuracy of AprilTag Detection*, ICRA 2020, DOI 10.1109/ICRA40945.2020.9197427. https://www.mucar3.de/icra2020-apriltags/ , https://github.com/UniBwTAS/apriltags_tas | 코너 정제(엣지 정제) 2종, 부분가림 필터, ROS 패키지. | 원거리 코너 잡음 → R 자체 축소 전처리. | 중 |
| B6 | Kalaitzakis et al., *Fiducial Markers for Pose Estimation: ARTag, AprilTag, ArUco, STag*, JIRS 2021 101:71, DOI 10.1007/s10846-020-01307-9 | 거리·각도별 비교. | 검출기 선택 근거(수치 미확인). | 중 (초록) |
| B7 | Laurent, Sandoz, *FMAC*, arXiv 2601.07723 (2026) | 0.5–1.5 m: AprilTag 위치 MAE 0.62 mm, 회전 0.1°. | 근거리 대조군 — 문제는 원거리+정면 조합. | 중–낮음 |
| B8 | Liu, Schofield, Shan (York), *Navigation of a Self-Driving Vehicle Using One Fiducial Marker*, arXiv 2104.12954 (2021) (전문) | KF [x,y,ψ,vx,vy,ψ̇]. IPPE 두 해에 대해 e=재투영오차 + KF 예측자세로 변환한 코너 3D 오차 → 작은 쪽. **주행 시작 후 오답 해가 재투영오차 낮아져 "재투영 최소 + KF" 실패**; 제안법 안정. 11 Hz. | 이동 중 연속 필터링의 핵심 레시피. `estimate_tag_pose` 단일 해 불신. | 높음 |
| B9 | Springer, Kyas, *Orientation Ambiguity and Detection Rate in April Tag and WhyCode*, arXiv 2203.10180 (2022); 후속 arXiv 2302.00786 (전문) | 30 cm 마커 1–3 m 480p. 불연속 판정 = 부호반전 ∧ 프레임간 각속도 > 1 rad/s. 48h12 2.6 % vs 24h10 16.1 %. | 자이로 각속도 대조로 flip 검출 강화. 다중 태그 번들. | 중–높음 |
| B10 | PhotonVision 3D Tracking 문서 + apriltag `apriltag_pose.h`. https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html , https://github.com/AprilRobotics/apriltag/blob/master/apriltag_pose.h | ambiguity ratio = 두 해 오차비 → 0.2 초과 기각; 비스듬히 장착; `estimate_tag_pose_orthogonal_iteration(info,&err1,&pose1,&err2,&pose2,n)`. | 최소 변경 게이트: ratio>0.2 면 yaw 생략, translation 유지. | 높음(실무) |
| B11 | Muñoz-Salinas et al., *Mapping and Localization from Planar Markers*, Pattern Recognition 2018, arXiv 1606.00151 | ratio test 원류; 애매하면 검출 폐기. | 단일 마커 실시간엔 "yaw 만 폐기, translation 유지 + 자이로". | 중 |
| B12 | Wang et al., *A Robust Planar Marker-Based Visual SLAM*, Sensors 2023 23(2):917, PMC9865496 | ratio + 시야각(FOV 0.4–0.6) 이중 조건 초기화. 추적 <55 % → >99 %. | ratio + 시야각 게이팅 선례. | 중 |
| B13 | Ch'ng et al., *Resolving Marker Pose Ambiguity by Robust Rotation Averaging*, ICRA 2020, arXiv 1909.11888 | 다중 뷰·마커 회전 평균. 단일 마커 실시간 부적합. | 낮음–중 | |
| B14 | Jin, Matikainen, Srinivasa, *Sensor Fusion for Fiducial Tags: RGBD*, IROS 2017. https://pengjujin.github.io/files/iros_aptag.pdf | 태그 내부 깊이 평면 피팅으로 해 선택. 0.65–1.85 m. | D435i 근거리(≲3 m) yaw 해 확인 보험. | 중(근거리) |
| B15 | Wang, Shan, Yue, Wang, IEEE TIE 2021 (A14 와 동일) | 목표 자세를 바디프레임 상태로 두는 EKF, 간헐 관측. | 우리 상태 정의와 동일 구조. | 높음(구조) (초록) |
| B16 | Fang, Li, Li, *Split Covariance Intersection Filter Visual Localization with AprilTag Map for Warehouse Robot*, arXiv 2310.17879 v3 (2025) | **창고 지게차 로봇**, IMU+엔코더+카메라. 적응 R=0.25·(L/α²)·‖예측−측정‖, 지연 측정 back-projection, SCIF. RMSE 0.45 m. | 지게차 실환경; R 스케일링·지연 처리 참고. 절대 정밀도는 거침. | 중–높음 |
| B17 | Alatise, Hancke, *Pose Estimation of a Mobile Robot Based on Fusion of IMU and Vision Using EKF*, Sensors 2017 17(10):2164, PMC5676736 | 기본형. heading 최대 0.95°. 바이어스 상태·미관측·지연 없음. | 부족한 점의 대조. | 중 |
| B18 | Hong, Park, *Minimal-Drift Heading Measurement using a MEMS Gyro*, Sensors 2008 8(11), PMC3787445 | r_m=(1+s)r+b+w, 기동 시 스케일·바이어스 LS, 정지 시 |r|<0.3°/s → 0. 1.67°→0.60°. | 회전 블라인드 구간 자이로 처리 기본기. | 중 |
| B19 | Kalibr IMU Noise Model. https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model ; https://github.com/ori-drs/allan_variance_ros | σ_gd=σ_g/√Δt, σ_bgd=σ_bg·√Δt; 저가 MEMS 10배 부풀리기. | D435i BMI055 Q 설정법. | 중(실무) |
| B20 | Tang et al., 3D-AprilTag 수중 로봇, JMSE 2025 13(5):833, DOI 10.3390/jmse13050833 | 거리·시야각·신뢰도 가중 관측전환 KF; 큐브형 태그. | R 함수 선례; 하드웨어 해법. | 중 |
| B21 | Dai & Lee 2025 (A15) | 태그 yaw 불신 → LiDAR heading. | 우리는 자이로가 그 역할. | 중 |
| B22 | 서경욱·황동윤·이민호·송진우·Do Hoang Viet, 미지의 ArUco 마커 위치 추정과 특징점 속도 기반 재추정, 전기학회논문지 73(4) 2024. KCI ART003068788 | 단안+IMU EKF, 미관측 구간 속도 예측. | 국내 predict-only 설계. | 중 |
| B23 | robot_localization ekf_localization_node 문서 + apriltag_ros issue #122. https://github.com/cra-ros-pkg/robot_localization/blob/noetic-devel/doc/state_estimation_nodes.rst , https://github.com/AprilRobotics/apriltag_ros/issues/122 | 마할라노비스 기각 임계, sensor_timeout predict-only, smooth_lagged_data, use_control. apriltag_ros covariance 전부 0. | ROS 최소 구성. 2중해 선택은 래퍼에서. | 높음(실무) |
| B24 | Chang, *Robust Kalman filtering based on Mahalanobis distance*, J. Geodesy 2014 88:391, DOI 10.1007/s00190-013-0690-8 | 이상치를 버리지 않고 R 부풀려 거리 맞춤. | 400 mm 튐 소프트 가중. | 중 (메타) |
| B25 | Digerud et al., *Vision-based positioning of USVs using Fiducial Markers for docking*, IFAC-PoL 2022 55(31):78, DOI 10.1016/j.ifacol.2022.10.412 | PnP + 등속 KF, RTK 대비. | 마커 스트림 속도 추정 선례. | 중 (초록) |
| B26 | Lee, Johnson, *Latency Compensated VIO*, Sensors 2020 20(8):2209, PMC7218848 | 지연 ≈45 ms, 과거 상태 보간, stochastic cloning, 미지 지연 random-walk 상태. | 캡처 시각 되돌려 적용 표준 패턴. | 중 |
| B27 | realsense-ros issue #2686. https://github.com/IntelRealSense/realsense-ros/issues/2686 | RGB ≈66 ms, depth ≈9 ms. | 측정 ~100 ms 과거 → 0.3 m/s 3 cm, 7°/s 0.7°. | 중(실무) |
| B28 | Sevostyanov, *Delay-compensating visual positioning*, J. Phys. Conf. Ser. 2021 1864:012038 | 시뮬레이션. | 낮음 | |
| 보조 | Kam, Yu, Wong SNPD 2018 (ArUco KF, DOI 10.1109/SNPD.2018.8441049); Mehralian, Soryani EKFPnP IET IP 2020 / arXiv 1906.10324 (픽셀공간 측정모델 EKF — 4코너 픽셀을 직접 측정으로 쓰면 R 단순화); Springer & Kyas 2023 arXiv 2302.00786 (짐벌 heading); **이상진·박찬수·송재복 "비전 정보를 활용한 무인 지게차의 도킹" ICROS 2015.05 / 이상진·송재복 "상하 카메라를 이용한 무인 지게차의 도킹" 제어로봇시스템학회논문지 2015.10 — 존재 확인, 원문 접근 실패, 직접 확인 권장**; 최지훈·김해창·송재복 JKROS 2020 15(4) (태그 보드 캘리브레이션); 장태호 외 2016 대한기계학회논문집 A 40(8). | | | |

### B 시사점(요약)
- 필터: 처음엔 (a) heading 상보/KF(자이로 적분 + 간헐 절대 yaw, 바이어스 상태), (b) lateral·forward 등속 KF 두 개 분리. 나중에 바디프레임 상대상태 EKF [y_lat, x_fwd, ψ, v, b_g] 로 통합.
- **lateral 계산 순서 변경**: 카메라프레임 translation 의 방위각 atan2(y,x)(픽셀 정밀) + 필터된 ψ 로 재구성.
- R(d,φ): Adámek 함수형, 입력은 검출 태그 픽셀면적. 우리 실측(3 m ±9 mm, 7–10 m 60–145 mm)은 d^2.3 증가 → 픽셀잡음 ∝ 1/면적과 일치. ±3σ 밖은 마할라노비스 게이트 또는 R 부풀리기.
- tilt 게이팅: 두 해 ratio > 0.2 또는 |φ| < 10–15° 이고 태그 작을 때 yaw 갱신 생략. 해 선택 Liu 식. 근거리는 depth 평면(Jin) 보험.
- 회전 블라인드: 카메라 차단/R 극대, 자이로 predict-only, 회전 전 정지에서 바이어스 재추정(데드밴드 0.3°/s). 재획득 시 예측 ψ 로 해 선택 → 게이트 → 연속 N프레임 큰 혁신이면 재초기화.
- 지연: 캡처 타임스탬프, 과거 시각 되감아 적용, 제어는 0.5 s 앞 예측 상태.

---

## C. 지연·관성 보상, bang-bang 정지 제어

### 핵심
- "지연 + 관성 오버런" 을 명시적으로 모델링해 정지를 조기 발령하는 산업 선례: 야스카와 코스팅 예측 특허(오버런 = 속도비례 + 고정지연×속도), 철도 정위치 정차 TASC(공주시간·감속 편차 예측해 노치 선택). **Yasunobu 예측 퍼지 ATO, MERL 양자화 제동 특허는 "이산 명령·지연·정지창" 구조가 우리와 동일.**
- 추정기 지연 처리 3종: (i) 지연 측정 시각정합 융합(Larsen 1998, Bar-Shalom OOSM, PX4 EKF2), (ii) 명령 버퍼 순방향 적분(Kalaria 2022, Carlos 2020), (iii) 액추에이터 1차지연 상태 증강. Smith 예측기는 (ii) 의 연속 피드백형.
- 느린 카메라 + 빠른 자이로 캐스케이드(Hurák & Řezáč 2010, Rupp 2021) = 우리 회전 제어에 그대로 대응.
- 차동구동 로봇 0.5 s 입력지연 식별·Smith 보상 사례(Ghaffari & Desai 2021) — 우리 지연 크기 이례적 아님.
- 산업 AGV ±3–5 mm 는 저속 크립 + 근접센서/스토퍼(NISTIR 8140; 국내 포크형 AGV 최저속 ≤18.6 mm). 우리 크립이 불안정(2 s 미만 순수 과도)이면 "정속 접근 + 예측 조기정지 + 학습형 펄스 보정".
- 지연 있는 2차계 시간최적 제어는 여전히 bang-bang, 스위칭 곡선이 "지연 후 예측 상태" 기준으로 이동(Ragg & Stapleton 1969) = `x + v·τ + coast(v)` 근거.
- **공백**: 자이로 각속도 × 지연 제자리회전 조기정지 정식 논문 없음; 지게차 CAN 데드타임–정지정밀도 논문 없음(가장 가까운 건 대형차 공압 브레이크 지연 보상).

### 항목
| # | 자료 | 요지 | 적용점 | 관련도 |
|---|---|---|---|---|
| C1 | Larsen, Andersen, Ravn, Poulsen, *Incorporation of Time Delayed Measurements in a Discrete-time Kalman Filter*, CDC 1998, DOI 10.1109/CDC.1998.761918 | 지연 측정을 현재로 외삽 후 최적 게인. 인용 230+. | `x_meas + v̂·τ_cam` 외삽 — 등속 근사라 충분. 입력지연 예: Phung 2017 arXiv 1703.03649. | 높음 |
| C2 | Bar-Shalom, *Update with Out-of-Sequence Measurements: Exact Solution*, IEEE TAES 2002 38(3):769, DOI 10.1109/TAES.2002.1039398; MathWorks Retrodiction | OOSM 1-step-lag 정확해. | 우리 지연은 작고 규칙적 → 불필요. | 중 |
| C3 | PX4 EKF2 문서. https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf ; Kim, Kang, Ahn 2025 Sensors PMC12527077 (카메라 0.3 s 지연 보정, 횡 RMSE 0.12–0.20→0.055–0.09 m); Lee & Johnson 2020 DOI 10.3390/s20082209 | 지연 융합 지평 + 센서별 FIFO + IMU 로 현재까지 전파. | 카메라(느림)+자이로(200 Hz) 정석 구조. | 높음(구현) |
| C4 | Hurák, Řezáč, *Delay Compensation in a Dual-rate Cascade Visual Servomechanism*, CDC 2010, DOI 10.1109/CDC.2010.5717039 | 수정 Smith 에서 모델 출력 대신 내부루프 실측 속도 적분으로 비전 지연 보상. | 카메라 yaw 앵커 + 자이로 전파의 이론 근거. | 높음 |
| C5 | Rupp, Valder, Knoll, Sawodny, *Cascaded Time Delay Compensation and Sensor Data Fusion for Visual Servoing*, SMC 2021, DOI 10.1109/SMC52423.2021.9659177 | KF 2개 캐스케이드(과거 시각 KF + 비동기 융합 KF). | 포즈 지연 별도 필터 분리. | 중–높음 |
| C6 | Normey-Rico & Camacho, *Control of Dead-time Processes*, Springer 2007, DOI 10.1007/978-1-84628-829-6; Normey-Rico et al. Frontiers Control Eng. 2022 DOI 10.3389/fcteg.2022.953768; Krstic, *Delay Compensation for Nonlinear, Adaptive, and PDE Systems*, 2009, DOI 10.1007/978-0-8176-4877-0 | SP → FSP(강인성 필터) → 예측기 피드백 x̂(t+τ)=e^{Aτ}x+∫e^{A(τ−s)}Bu(t−τ+s)ds. | bang-bang 엔 예측기 피드백의 이산형(버퍼 적분)을 정지 판단에만. 지연 불확실 크면 보수적 마진. | 중(배경) |
| C7 | Ghaffari, Desai, *Safety-Control of Mobile Robots Under Time-Delay Using Barrier Certificates and a Two-Layer Predictor*, arXiv 2104.15047 (2021) | 차동구동 휠 V/U=(5.94s+1.45)/(s²+7.40s+1.42)·e^{−0.50s}. Smith 3개 2층. 수렴 4배, 윤곽오차 1.69 cm. | 같은 크기 지연 헤딩 제어 사례; 지연 식별 절차(스텝응답→FOPDT/SOPDT). | 높음 |
| C8 | Velasco-Villa et al., *Smith-predictor Compensator for a Delayed Omnidirectional Mobile Robot*, MED 2007, DOI 10.1109/MED.2007.4433837 | 정확 이산화 + SP. | 30 fps 이산시간에서 지연을 정수 샘플로. | 중 |
| C9 | Kalaria, Lin, Dolan, *Delay-aware Robust Control for Safe Autonomous Driving*, IEEE IV 2022, arXiv 2109.07101 | 액추에이터 1차 ODE 상태 증강 + 명령 버퍼로 t̂_d 동안 구간적분해 x̂(t+t̂_d) 에서 tube-MPC; t_c 상한 적응 KF. 드론판 Carlos 2020 arXiv 2010.11264. | **우리 레시피 그대로**: 버퍼 속 forward 로 τ_d 순방향 적분한 x̂ 기준으로 stop 판단. | 높음 |
| C10 | Yaskawa, *Robot System*, US 8,812,159 B2 (우선 2010). https://patents.google.com/patent/US8812159B2/en | θ_ds=(ω/ω_max)·θ_D,max, θ_df=t_d·ω, θ_d=θ_ds+θ_df. 최대하중·최대속도 실측 후 스케일. | D(v)=τ_d·v+c(v) 산업 표준형; 적재/비적재 실측 후 v 스케일. | 높음 |
| C11 | Yasunobu, Miyamoto, Ihara 1983 Trans. SICE 19(11):873 DOI 10.9746/sicetr1965.19.873; Yasunobu & Miyamoto 1985 *ATO by Predictive Fuzzy Control* http://www.ics.esys.tsukuba.ac.jp/yasu2016/papers/ATO1985.pdf ; Sandidzadeh & Shamszadeh 2012 InTech https://cdn.intechopen.com/pdfs/34434/InTech-Improvement_of_automatic_train_operation_using_enhanced_predictive_fuzzy_control_method.pdf | 이산 노치, 시변 제동, 저해상도 속도, 큰 지연. 후보 노치별 정지위치 X_t 시뮬레이션 예측 → 정확정지(±30 cm)·승차감·추종성 평가 → 선택. 센다이 지하철 상용화 1987. | **후보 명령(stop now / next tick / keep)별 정지위치 예측 → 창 안 후보 선택** 구조 원형. 예측모델 정확도 요구를 허용치로 역산. | 높음 |
| C12 | MERL, *Train Automatic Stopping Control with Quantized Throttle and Braking*, US 10,093,331 B2 (2018). https://patents.google.com/patent/US10093331B2/en | soft landing 선형 부등식, 유한 명령집합의 제어불변 부분집합(양자화·불확실성만큼 backward-reachable 축소), 액추에이터 τ_a 포함 receding-horizon. | 정지창 보장 원하면 불변집합 관점; 축약형 "예측치±kσ 가 창 안일 때만 stop". | 높음 |
| C13 | 김정태·이재호·김무선·박철홍, 칼만 필터를 이용한 도시철도 열차 정위치 정차, 한국산학기술학회논문지 17(11):655 (2016). KCI ART002171353; **한국철도기술연구원 특허 KR 10-2026264 (2018) "제동거리 편차를 최소화하는 철도차량의 제동제어 방법"** https://patents.google.com/patent/KR102026264B1/ko | KF 로 "제동기가 실제 동작할 시점의 상태" 예측 후 입력 도출. 특허: 응답시간 Δt 반영해 (t+Δt) 목표/실제 위치 예측·비교. | **국내·같은 기관 선례** — 지연 후 시점 상태 예측 → 명령 결정. 과제 문서 인용 가치. | 중–높음 |
| C14 | Bu, Tan, *Pneumatic Brake Control for Precision Stopping of Heavy-Duty Vehicles*, IEEE TCST 2007 15(1):53, DOI 10.1109/TCST.2006.883238; Devika et al. 2021 Proc IMechE C 235(13):2333 DOI 10.1177/0954406220952822 (Padé vs 상태예측, 상태예측 우수, 시상수 100 %·지연 40 % 변동 강인); Lee, Kim, Kim, Huh 2014 IJAT 15(2):341 DOI 10.1007/s12239-014-0035-5 | 관성 큰 차량: 지연은 상태예측, 감속 편차는 적응. | 우리 속도 CV 10–30 % → 명령→속도 게인 적응 추정 필요. | 중–높음 (일부 초록) |
| C15 | Wu & Wang 2014 CCC DOI 10.1109/ChiCC.2014.6895502 (적응 GPC + 온라인 식별); Zhang et al. 2019 IECON DOI 10.1109/IECON.2019.8927061 (공압 브레이크를 순수 지연으로 취급하는 한계) | "순수 지연" 모델 경고. | 우리 14 cm ≈ v×0.5 s 는 순수지연으로 설명되나 감속 램프 유무를 30 fps 궤적으로 확인. | 중 |
| C16 | Bostelman, Hong, NISTIR 8140 (A26 동일) | 정밀 도킹 사례 전부 ≤0.2 m/s; EKF 추측항법+비전 5.1±3.0 mm. | 접근속도 상한 근거. | 중–높음 |
| C17 | 우승범·정경훈·김정민·박정제·김성신, 중량물 운송 AGV 주행 제어, 한국지능시스템학회 논문지 20(3):394 (2010). ScienceON JAKO201004140972084; 한국생산기술연구원 2017 "자율주행 AGV 정지정도 향상" ScienceON TRKO201800036712 | 하역 구간 최저속 유지, 정지 최대오차 18.64 mm. 목표 ±15 mm. | 국내 포크형 AGV 크립 전략; 우리 크립 불안정과 대조. | 중 |
| C18 | Klimenda et al., *Stopping the Mobile Robotic Vehicle at a Defined Distance by IR Sensor*, Sensors 2021 21(17):5959, DOI 10.3390/s21175959 | 제동거리 ∝ 속도(지연 지배). 50 mm/s 에서 80±5 mm. | 우리 실측과 일치하는 관찰. | 중(낮음) |
| C19 | Ragg, Stapleton, *Time Optimal Control of Second-order Systems with Transport Lag*, Int. J. Control 1969 9(3):243, DOI 10.1080/00207176908905748; Watkins, Piper, Leitner ACC 2003 (지연 이중적분기); Tsypkin *Relay Control Systems* 1984 | 지연 있어도 bang-bang; 스위칭은 τ 후 예측 상태 기준. 이중적분기: x+v·τ+v²/(2a)=x_target. | `x + v·τ + coast(v)` 규칙의 이론 근거. | 중 |
| C20 | Sorensen, Singhose, Dickerson, IFAC 2005 DOI 10.3182/20050703-6-cz-1902.00497; CEP 2007 15(7):825 DOI 10.1016/j.conengprac.2006.03.005 | 온오프 릴레이 크레인 정밀 위치결정, 피드백 + 입력성형. 인용 330+. | 이산 명령 대관성 시스템 선례; 펄스폭 계획 개념(소각 회전 보정). | 중 (초록) |
| C21 | Autoware `autoware_pid_longitudinal_controller` 문서 (delay_compensation_time 0.17 s, 정지 시퀀스 0.5 m → 약제동 → 과주 시 강제동); Nav2 opennav_docking (v 0.1–0.25 m/s, docking_threshold 0.05 m, stall detection, 속도의존 조기정지 없음) | 자율주행 스택 "지연만큼 앞선 상태로 피드백" 표준. | 판정 로직(threshold, stall) 참고. | 중 |

### C 시사점(요약)
**(a) 전진 정지** — `D(v)=τ_d·v+c(v)`, c(v)=v²/(2a) 또는 (v/v_max)·D_max. 실측 14 cm 는 순수 지연항 → τ_d 와 a 분리 식별(적재/비적재, 3–5 속도). 발령: 매 프레임 명령버퍼로 x̂(t+τ_d) 적분, `x_target − x̂ ≤ v̂·τ_d + c(v̂) + v̂·Δt_ctrl/2 + m` 이면 stop. v̂ 는 **실측 속도 추정치**(명령 게인 아님). 변동성 `σ_D² ≈ (τ_d·σ_v)² + (v·σ_τ)² + σ_c²`; σ_τ 0.05–0.1 s → 1.4–2.8 cm → **σ_τ 실측 최우선**. 접근속도 0.15–0.2 m/s 로 낮추면 D, σ_D 모두 ∝ v 로 감소. 2 s 미만 이동은 과도상태이므로 **정속 유지 가능한 길이(≥2–3 s) 확보 후 한 번에 정지**; 잔차는 학습형 펄스(펄스길이→거리 표 매 회 갱신) 1회. 비대칭 비용(충돌 금지)이면 예측치+kσ 가 창 안일 때만 발령.

**(b) 회전 정지** — `θ_target − θ̂ ≤ ω̂·τ_r,stop + c_r(ω̂) + m_θ`. 시작지연 0.85 s 는 토크 형성 포함 → **정지지연 별도 실측**(자이로 200 Hz 로 stop 시각 vs ω 감소 개시). 코스팅 1.3° ≈ ω×0.19 s. 각속도 run 간 σ 2.1°/s 는 자이로가 직접 주므로 무관; 남는 건 지연 지터(0.1 s × 6.8°/s ≈ 0.7°). 헤딩은 PX4/Hurák 구조. 회전 직전 정지에서 바이어스 재추정. **10° 미만 소각은 정속 구간 없음 → 캘리브레이션 펄스 표(펄스폭→각) + 매 회 자이로로 갱신.**

**(c) 추정기 지연 모델** — FOPDT(순수지연 + 1차지연) 액추에이터 + 명령 버퍼. 상태 [x,v]/[θ,ω], `v̇=(K·u_act−v)/T`, `u_act(t)=u_cmd(t−τ_d)`. K 는 run 마다 10–30 % 변하므로 (i) 등속 + 큰 Q 로 측정 주도, 또는 (ii) K 를 느린 random-walk 상태로. Smith 예측기는 연속 피드백용 — 정지 판단엔 버퍼 적분 예측이 등가. FSP 교훈: 예측을 보수적 마진으로 감싸라. 카메라 τ_cam 은 타임스탬프 + Larsen 외삽 또는 PX4 식 지연 지평.

---

## 읽기 우선순위 (전체)
1. **Richter et al. 2022** — 3단계 정지·재측정 도킹, 정면 ±25° 회피, 인지 vs 실행 오차 분리.
2. **Poreski 2016** — 우리와 같은 명령세트의 정지-관측-동작 도킹 전과정(기하·임계 스윕·관성 보정).
3. **Adámek et al. 2023** — R(d,φ) 함수형 + EKF, 정면 yaw 피크.
4. **Liu, Schofield, Shan 2021** — 이동 중 2중해를 KF 예측으로 선택; 재투영오차 기준 실패 실험.
5. **Kalaria, Lin, Dolan 2022** — 명령버퍼 순방향 적분 + 액추에이터 상태 증강: 정지 판단 구현 뼈대.
6. **Yasunobu & Miyamoto 1985 / Sandidzadeh 2012** — 이산 명령별 정지위치 예측·선택.
7. **Yaskawa US 8,812,159** — 코스팅 = 속도비례 + 지연×속도, 캘리브레이션 절차.
8. osrf/autodock README + Nav2 docking README — 참조 구현.
9. Hurák & Řezáč 2010 + PX4 EKF2 — 느린 카메라/빠른 자이로 지연 보상.
10. Abbas 2019, Wang 2021 TIE, robot_localization 문서, Ghaffari & Desai 2021, KRRI KR 10-2026264.

## 내부에서 확보해야 할 국내 자료
- 정세영·박찬호(KRRI)·김성주·임지원·황성호, 한국자동차공학회 춘계 2025 — ArUco 차량 자율 선적 측위 (DBpia).
- 이상진·박찬수·송재복, ICROS 2015.05 / 이상진·송재복, 제어로봇시스템학회논문지 2015.10 — 무인지게차 비전 도킹.
- KRRI 특허 KR 10-2026264 + 김정태 외 2016 — 철도 정위치 정차 지연 예측(과제 맥락 인용).

## 확인 한계
초록·메타데이터 수준: Seelinger & Yoder 2006, Wang 2021 TIE, Dai & Lee 2025, Kalaitzakis 2021, Digerud 2022, Chang 2014, Tang 2025, Bu & Tan 2007, Sorensen 2005/2007, Lee 2014, Watkins 2003, 정세영 2025, 이상진 2015. 나머지는 본문/PDF 확인. Adámek 수치는 PDF 원문 재확인값.
