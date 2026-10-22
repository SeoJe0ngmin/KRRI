# v3 Docking 설계 토론 팀 (Agent)

목적: proposer ↔ critic 이 **근거를 대며** 토론해 v3 도킹 **plan.md** 를 완성한다.
코드는 plan 확정 후 coder 가 v3 에 작성한다.

## 역할 (정의: `<repo>/.claude/agents/*.md`)
| agent | 도구 | 역할 | 쓰는 skill |
|---|---|---|---|
| **proposer** | 읽기만 | 쟁점별 방법론 제안 + 근거 | scientific-brainstorming, hypothesis-generation, source-comparison, what-if-oracle |
| **critic** | 읽기만 | 반박 + 대안 + 근거, critic.md failure mode 대조 | scientific-critical-thinking, peer-review, what-if-oracle, source-comparison |
| **judge** | 읽기만 | 왕복을 독립 채점 → 쟁점별 결론. 근거 부족은 보류 | scientific-critical-thinking, scholar-evaluation, consciousness-council |
| **researcher** | 읽기+웹 | judge 가 보류한 쟁점의 문헌 근거 조사(prior_art 형식) | deep-research, literature-review, source-comparison |
| **coder** | 읽기+쓰기(v3만) | plan 확정 후 v3 코드 구현 | sympy, statistical-analysis, statsmodels, scikit-learn (필요시) |

## 토론 흐름 (대문제 1개당, `debate_workflow.js` 로 실행)
```
proposer 제안(쟁점별)
 → critic 2렌즈 병렬 반박·대안  [안전·신뢰성 렌즈 | 정밀도·수렴 렌즈]
 → proposer 재반박/수용
 → judge 판정(구조화: 쟁점별 verdict/결론/검증실험/보류)
    └ 보류(근거 부족) 쟁점만 → researcher 문헌 조사 → judge 재판정
 → 오케스트레이터가 log/debate_<n>_<주제>.md + plan.md 해당 절 작성
 → 체크포인트(사용자 검토) → OK 면 다음 대문제
```
실행: `Workflow({ scriptPath: "apriltag_v3/Agent/debate_workflow.js", args: { n: 1 } })` (n=1..4).
쟁점 목록·참조 파일은 스크립트의 `PROBLEMS` 에 정의돼 있다(수정하려면 거기).

## 모델 정책
- 기본: 세션 모델 **Fable 5.1[1m]** + 세션 effort **xhigh**(설정 가능한 최고) + ultracode + Workflow. 토론 **서브에이전트는 effort `max`**(에이전트 호출이 받는 최고 단계).
- **1단계 폴백**: 에이전트 호출이 실패하면(재시도 후 API 오류 = Fable 토큰/한도 소진 등) **같은 프롬프트를 Opus 5 로 재시도, effort 는 max 유지** — `debate_workflow.js` 의 `call()`.
- **2단계 대기·재개**: Opus 도 실패하면(전체 한도 소진 → 기다려야 함) 워크플로가 `QUOTA_EXHAUSTED` 로 멈추고 **그때까지의 진행은 캐시로 보존**. 한도 재생성 후 `Workflow({scriptPath, args, resumeFromRunId})` 로 재개하면 완료된 호출은 즉시 복원되고 실패 지점부터만 다시 돈다 — 처음부터 다시 하지 않는다.
- **세션 레벨 (`.claude/settings.json`, 로컬 전용)**:
  - `fallbackModel: ["claude-opus-5[1m]"]` — Fable 불가/과부하 시 세션도 Opus 5[1m] 로 자동 전환(1M 컨텍스트 유지).
  - `effortLevel: "xhigh"` + `modelSettings.{claude-fable-5-1, claude-opus-5}.effortLevel: "xhigh"` — 전환돼도 effort 유지.
  - **`autoContinueAtUsageLimit: true`** — usage limit 으로 세션이 멈추면 **리셋까지 기다렸다가 자동으로 이어감**(공식 설정).
  - `agentPushNotifEnabled` / `inputNeededNotifEnabled: true` — 푸시 알림·입력 대기 알림 허용.
- **우선순위**: Fable 되면 Fable → 안 되면 Opus 5 → 둘 다 한도면 **기다렸다가 재생성 후 재개**(처음부터 다시 안 함).

## 알림 정책 (PushNotification)
오케스트레이터가 다음 시점에 푸시 알림(데스크톱 + Remote Control 연결 시 폰)을 보낸다:
- 대문제 하나 **토론 완료** → 결론 수·보류 수 요약 ("대문제 1 완료: 결론 5/보류 2 — 검토 요망")
- **멈춤/병목**: 한도 소진(QUOTA_EXHAUSTED), 에이전트 실패, 권한 대기, 사용자 결정 필요
- **재개**: 한도 재생성 후 이어서 돌기 시작했을 때
- 사용자가 터미널을 보고 있으면 중복이라 자동으로 생략됨(도구 동작).

## 근거 규칙 (필수)
의견·반박은 반드시 근거를 동반한다:
- 선행연구 — `../docs/prior_art_docking.md` 의 항목 인용 (예: Adámek 2023, Liu 2021)
- 실측치 — `../../CLAUDE.md`(지연·관성·잡음), 9/7 로그 분석
- 코드 근거 — v1/v2/v3 실제 코드

## 권한 (엄수)
- 읽기: `apriltag_v1`, `apriltag_v2`, `apriltag_v3`, `apriltag_v3/docs`(methodology 4개 + critic.md + problem_definition), `prior_art_docking.md`, 루트 `CLAUDE.md`
- 쓰기: **`apriltag_v3/Agent/` 안에서만.**
- **v1·v2 는 절대 수정 금지. v3 코드도 이 토론 단계에선 읽기만**(수정은 coder 단계에서만).

## 산출물
- `plan.md` — 토론 결론을 모은 최종 설계(대문제 1~4).
- `log/debate_<n>_<주제>.md` — 대문제별 토론 기록(핵심만).

## 대문제 (토론 단위, 각 끝날 때 체크포인트)
1. **Perception** — 현재 차량 상태 추정 (lateral 재구성 / heading 해선택·AMBIGUOUS / 관측성 / uncertainty calibration / 지연보정)
2. **Vehicle Dynamics** — 명령→결과 예측 (stop 예측 점→분포 / 좌우 비대칭 / 저속 데드존·최소제어량 / 가속도 역할)
3. **Controller** — 수렴 제어 (state machine EXECUTE→STOPPING→SETTLE→VERIFY / pivot decouple / candidate cost 정규화·1-step 한계 / zone hysteresis / wait deadlock fallback)
4. **System/Integration** — Timing·timestamp / Perception↔Dynamics 오차 분리 / 99% 성공 KPI / failure modes / 문서 통합

## 로그 형식 (핵심만 — 쟁점당 6~8줄)
```
## 쟁점 N: <이름>
- proposer: <제안> — 근거: <선행연구/실측/코드>
- critic(안전): <반박/대안> — 근거: ...
- critic(정밀): <반박/대안> — 근거: ...
- proposer 재반박: 수용/유지/수정 — <이유>
- (researcher: <새 근거> — 보류됐던 경우만)
- judge: <verdict> — 결정적 근거: ...
- 결론: <plan.md 에 들어간 문장>  | 검증: <실차/시뮬 실험 1개>
```
plan.md 에는 **결론·검증·파라미터 자리**만 옮긴다(토론 과정은 log 에만).
