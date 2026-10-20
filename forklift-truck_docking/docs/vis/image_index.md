# 이미지 목록

[전체 설명·규칙 Markdown](algorithm_and_rules.md)

작성일: 2026-10-19. 실행 코드·설정은 변경하지 않았다.

## 도식

PNG는 공유용, SVG는 확대·편집용, Mermaid는 도식 원문이다.

| 번호 | 내용 | 파일 |
|---|---|---|
| 1 | 전체 주행 알고리즘 | [PNG](diagrams/01-overall-flow.png) · [SVG](diagrams/01-overall-flow.svg) · [원문](diagrams/01-overall-flow.mmd) |
| 2 | 후보 계획 생성 | [PNG](diagrams/02-candidate-planning.png) · [SVG](diagrams/02-candidate-planning.svg) · [원문](diagrams/02-candidate-planning.mmd) |
| 3 | 오차·드리프트·관성 보정 개요 | [PNG](diagrams/03-error-correction.png) · [SVG](diagrams/03-error-correction.svg) · [원문](diagrams/03-error-correction.mmd) |
| 4 | 태그 창 수용·건너뛰기·앵커 갱신 | [PNG](diagrams/04-tag-anchor-update.png) · [SVG](diagrams/04-tag-anchor-update.svg) · [원문](diagrams/04-tag-anchor-update.mmd) |
| 5 | 태그 검출부터 융합까지의 탈락 규칙 | [PNG](diagrams/05-tag-rejection.png) · [SVG](diagrams/05-tag-rejection.svg) · [원문](diagrams/05-tag-rejection.mmd) |
| 6 | VIO 유효성 | [PNG](diagrams/06-vio-validity.png) · [SVG](diagrams/06-vio-validity.svg) · [원문](diagrams/06-vio-validity.mmd) |
| 7 | 정지 판단 | [PNG](diagrams/07-stationarity.png) · [SVG](diagrams/07-stationarity.svg) · [원문](diagrams/07-stationarity.mmd) |
| 8 | 관성·되튐·출발 지연 | [PNG](diagrams/08-inertia-rebound.png) · [SVG](diagrams/08-inertia-rebound.svg) · [원문](diagrams/08-inertia-rebound.mmd) |
| 9 | 눈감고 진입 | [PNG](diagrams/09-blind-entry.png) · [SVG](diagrams/09-blind-entry.svg) · [원문](diagrams/09-blind-entry.mmd) |
| 10 | 센서 누락·재시작 | [PNG](diagrams/10-sensor-restart.png) · [SVG](diagrams/10-sensor-restart.svg) · [원문](diagrams/10-sensor-restart.mmd) |
| 11 | CAN·deadman·작업자 종료 | [PNG](diagrams/11-can-deadman.png) · [SVG](diagrams/11-can-deadman.svg) · [원문](diagrams/11-can-deadman.mmd) |
| 12 | 초기 직진 장착 보정 | [PNG](diagrams/12-mount-straight.png) · [SVG](diagrams/12-mount-straight.svg) · [원문](diagrams/12-mount-straight.mmd) |

## 상세 표

긴 표는 최대 11개 항목씩 나누었으며 모든 표의 원문은 전체 Markdown에 포함했다.

| 표 번호 | 내용 | 이미지 |
|---|---|---|
| 1 | 전체 흐름의 주요 값 (1/2) | [PNG](tables/01-main-parameters-01.png) · [SVG](tables/01-main-parameters-01.svg) |
| 1 | 전체 흐름의 주요 값 (2/2) | [PNG](tables/01-main-parameters-02.png) · [SVG](tables/01-main-parameters-02.svg) |
| 2 | 센서와 추정값의 역할 | [PNG](tables/02-sensor-roles.png) · [SVG](tables/02-sensor-roles.svg) |
| 3 | 앵커 갱신 케이스 | [PNG](tables/03-anchor-cases.png) · [SVG](tables/03-anchor-cases.svg) |
| 4 | 2.90m·2.40m 예시 | [PNG](tables/04-jump-examples.png) · [SVG](tables/04-jump-examples.svg) |
| 5 | 태그·Depth·시각 검사 (1/2) | [PNG](tables/05-tag-depth-checks-01.png) · [SVG](tables/05-tag-depth-checks-01.svg) |
| 5 | 태그·Depth·시각 검사 (2/2) | [PNG](tables/05-tag-depth-checks-02.png) · [SVG](tables/05-tag-depth-checks-02.svg) |
| 6 | VIO 품질·유효기간 | [PNG](tables/06-vio-quality.png) · [SVG](tables/06-vio-quality.svg) |
| 7 | 관성 보정·학습 규칙 (1/2) | [PNG](tables/07-inertia-rules-01.png) · [SVG](tables/07-inertia-rules-01.svg) |
| 7 | 관성 보정·학습 규칙 (2/2) | [PNG](tables/07-inertia-rules-02.png) · [SVG](tables/07-inertia-rules-02.svg) |
| 8 | 차량 반응 초기값 | [PNG](tables/08-motion-initial-values.png) · [SVG](tables/08-motion-initial-values.svg) |
| 9 | 눈감고 진입의 예외 | [PNG](tables/09-blind-exceptions.png) · [SVG](tables/09-blind-exceptions.svg) |
| 10 | 상태 전환·종료 사유 (1/3) | [PNG](tables/10-state-rules-01.png) · [SVG](tables/10-state-rules-01.svg) |
| 10 | 상태 전환·종료 사유 (2/3) | [PNG](tables/10-state-rules-02.png) · [SVG](tables/10-state-rules-02.svg) |
| 10 | 상태 전환·종료 사유 (3/3) | [PNG](tables/10-state-rules-03.png) · [SVG](tables/10-state-rules-03.svg) |
| 11 | 주행 타임아웃 한눈에 보기 | [PNG](tables/11-timeouts.png) · [SVG](tables/11-timeouts.svg) |
| 12 | 센서·시간 검사 (1/3) | [PNG](tables/12-sensor-time-rules-01.png) · [SVG](tables/12-sensor-time-rules-01.svg) |
| 12 | 센서·시간 검사 (2/3) | [PNG](tables/12-sensor-time-rules-02.png) · [SVG](tables/12-sensor-time-rules-02.svg) |
| 12 | 센서·시간 검사 (3/3) | [PNG](tables/12-sensor-time-rules-03.png) · [SVG](tables/12-sensor-time-rules-03.svg) |
| 13 | 큐가 밀리는 경우 | [PNG](tables/13-queues.png) · [SVG](tables/13-queues.svg) |
| 14 | CAN 보호·종료 조건 (1/2) | [PNG](tables/14-can-rules-01.png) · [SVG](tables/14-can-rules-01.svg) |
| 14 | CAN 보호·종료 조건 (2/2) | [PNG](tables/14-can-rules-02.png) · [SVG](tables/14-can-rules-02.svg) |
| 15 | 장착 보정 규칙 (1/2) | [PNG](tables/15-mount-rules-01.png) · [SVG](tables/15-mount-rules-01.svg) |
| 15 | 장착 보정 규칙 (2/2) | [PNG](tables/15-mount-rules-02.png) · [SVG](tables/15-mount-rules-02.svg) |
| 16 | 환경·작업자·기록 예외 (1/2) | [PNG](tables/16-other-exceptions-01.png) · [SVG](tables/16-other-exceptions-01.svg) |
| 16 | 환경·작업자·기록 예외 (2/2) | [PNG](tables/16-other-exceptions-02.png) · [SVG](tables/16-other-exceptions-02.svg) |

## 빠른 보기

[오프라인 이미지 갤러리](index.html)
