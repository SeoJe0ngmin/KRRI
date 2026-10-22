// v3 도킹 설계 토론 Workflow — 대문제 1개를 proposer → critic(2렌즈) → 재반박 → judge(+researcher) 로 돌린다.
// 실행: Workflow({ scriptPath: ".../Agent/debate_workflow.js", args: { n: 1 } })   n = 1..4
// 에이전트는 전부 읽기 전용(텍스트만 반환). 로그·plan.md 는 오케스트레이터(메인 세션)가 쓴다.
// 모델 정책: 세션 모델(Fable 5.1) + effort max → 호출 실패(한도 소진 등) 시 Opus 5 로 재시도, effort 유지.
//            Opus 도 실패하면 QUOTA_EXHAUSTED 로 멈춤(진행 캐시 보존) → 한도 재생성 후 resumeFromRunId 로 재개. (call() 참고)

export const meta = {
  name: 'v3-design-debate',
  description: 'v3 도킹 설계 대문제 하나를 proposer/critic/judge 로 근거 기반 토론해 쟁점별 결론을 낸다',
  phases: [
    { title: 'Propose', detail: 'proposer 가 쟁점별 방법론 + 근거' },
    { title: 'Critique', detail: 'critic 2렌즈(안전·신뢰성 / 정밀도·수렴) 병렬 반박' },
    { title: 'Rebut', detail: 'proposer 가 두 반박에 재반박' },
    { title: 'Judge', detail: 'judge 가 왕복을 채점해 쟁점별 판정(구조화)' },
    { title: 'Research', detail: '근거 부족 쟁점만 researcher 가 문헌 조사' },
    { title: 'Final Judge', detail: '조사 결과 반영해 judge 재판정' },
  ],
}

const ROOT = '/Users/seojeongmin/krri'
const DOCS = `${ROOT}/apriltag_v3/docs`
const SRC = `${ROOT}/apriltag_v3/src`

// ── 대문제 정의 ─────────────────────────────────────────────────────────────
const PROBLEMS = {
  1: {
    key: 'perception',
    title: 'Perception — 현재 차량 상태(회전중심 기준 forward/lateral/heading + 속도 + 신뢰도) 추정',
    docs: [
      `${DOCS}/0. forklift_docking_problem_definition.md`,
      `${DOCS}/1. perception_methodology_v2.md`,
      `${DOCS}/critic.md  (특히 §2~§7, §24 Failure 1·2)`,
      `${DOCS}/prior_art_docking.md  (특히 B 절, A9·A19·A20·A21·A22)`,
      `${SRC}/models/detection/detection_pose.py  (docking_state·measure — 현재 lateral/heading 계산)`,
      `${SRC}/utils/imu_yaw.py  (자이로 yaw)`,
    ],
    issues: [
      'lateral 재구성 — scalar 식 d·sin(ψ̂−β) 로 갈지, 명시적 좌표변환(T_tag←pivot) 으로 갈지. 부호 convention 검증 방법',
      'bearing β 계산 — PnP translation atan2(t_x,t_z) vs 태그중심 픽셀→정규화좌표. 어느 쪽이 PnP 오차에 덜 묶이나',
      'heading PnP 2중해 처리 — 해 선택 규칙, AMBIGUOUS 상태 도입, wrong-branch lock-in 탈출(재초기화 조건)',
      '기하 관측성/신뢰도 — 단순 각도 threshold 가 아닌 (tag_px, distance, tilt, err ratio) 기반 heading 신뢰도. 처음엔 얼마나 단순하게 시작할지',
      'uncertainty ≠ 실제 오차 — 추정 covariance 를 controller 게이트로 쓰기 전 calibration 절차',
      '지연 보정 — 어떤 timestamp(캡처/수신/검출완료)를 쓰고, 독립 외삽 vs SE(2) 전파 중 무엇으로, 회전+전진 동시일 때',
      '주행 중 연속 추정기 구조 — 축별 등속 KF vs α-β vs 상보필터, 구동기 모델 미포함 원칙, 저속에서 v̂ 잡음 문제(프레임당 5~7mm ≈ 잡음 ±9mm) 완화',
    ],
  },
  2: {
    key: 'dynamics',
    title: 'Vehicle Dynamics — 명령→실제 이동/회전 결과 예측 (정지 예측 포함)',
    docs: [
      `${DOCS}/0. forklift_docking_problem_definition.md`,
      `${DOCS}/3. vehicle_dynamics_methodology.md`,
      `${DOCS}/2. vehicle_dynamics_straight_methodology.md  (직진 심화, 3번과 겹침)`,
      `${DOCS}/critic.md  (특히 §8~§12, §21, §24 Failure 4·5)`,
      `${DOCS}/prior_art_docking.md  (특히 C 절: C1·C7·C9·C10·C11·C12·C19)`,
      `${SRC}/models/control/fwd_time_model.py  (현재 거리→시간 개루프)`,
      `${SRC}/models/control/rot_control.py  (rotate_to 고정 리드 ROT_LEAD_DEG)`,
      `${SRC}/models/control/control_forklift_v2.py  (byte2 풀67 / 광운대 저속97 값 참고)`,
      `${SRC}/models/control/control_광운대.py  (forward_slow=97, entry burst)`,
    ],
    issues: [
      '정지거리 모델 — d_stop=g(v) 를 점 예측이 아니라 분포 P(d_stop|v)(잔차 std·95/99 percentile) 로 관리하는 최소 구현',
      '명령 지연 분리 — start delay / stop delay / 관성 을 실측에서 어떻게 분리하고, 분리 안 해도 되는 범위',
      '가속도(IMU) 의 역할 — 미래 제동 예측기가 아니라 응답 감지·정지 판별로 한정하는 규칙(임계+지속시간)',
      '회전 정지 예측 — θ_stop=h(ω) 좌/우 비대칭 분리, 소각(<10°) 처리, 고정 ROT_LEAD_DEG 대체',
      '저속 크립(byte2=97) 활용 — 데드존·최소 제어량 측정, 저속 구간 길이(≥2~3s 정속) 확보 방법, 비례 여부 판별 실험',
      '온라인 파라미터 적응(τ·a EMA/RLS) 을 넣을지 — 고정 실측 상수로 시작 vs 적응. 적응 시 노이즈 v̂ 이중미분 위험',
      '사전실험 설계 — precheck_forward_dynamics / precheck_rotation_dynamics 가 기록할 항목과 반복 횟수',
    ],
  },
  3: {
    key: 'controller',
    title: 'Controller — 불확실한 추정·모델 오차 아래서 oscillation 없이 목표 pose 로 수렴',
    docs: [
      `${DOCS}/0. forklift_docking_problem_definition.md`,
      `${DOCS}/4. controller_methodology.md`,
      `${DOCS}/critic.md  (특히 §13~§20, §24 Failure 3·6·7, §25 state machine, §26)`,
      `${DOCS}/prior_art_docking.md  (특히 A10 Poreski, A11 autodock, A12 Nav2, C11 Yasunobu, C12 MERL)`,
      `${SRC}/models/control/control_from_pose.py  (plan_step·plan_aim·dock_live — 현재 stop-and-go 루프)`,
    ],
    issues: [
      'Controller state machine — OBSERVE→DECIDE→EXECUTE→STOPPING→SETTLE→VERIFY 각 상태의 진입/탈출 조건과 "STOPPING·SETTLE 중 반대 명령 금지"',
      '회전중심(pivot) 좌표로 decouple — lateral↔heading coupling 을 어디서 한 번 끊나(Perception 출력 vs Controller 입력)',
      'candidate/cost 컨트롤러 — 허용오차 정규화 cost, 1-step 이 대각 lateral 보정을 못 찾는 문제(이득이 미래에 옴) → 2~N step 지평 vs T-조준 가이드',
      '태그 가시성 제약 — "태그를 화면 중앙(β≈0)에 유지" 를 cost 항으로 둘지 하드 제약으로 둘지. 사이드스텝 폐기 후 소각 대각 접근 규칙',
      '행동 게이트 — evidence persistence(N프레임 지속) + uncertainty-adjusted threshold + minimum expected improvement + reversal guard 의 구체 조건',
      'Zone(far/near/final) 전환 hysteresis 와 zone 별 목표(방위각만 / 정렬 / 예측정지)',
      '"불확실하면 기다림" deadlock 의 timeout·fallback(재측정·이동·중단) 과 수동전환 조건',
    ],
  },
  4: {
    key: 'system',
    title: 'System / Integration — Timing 체계, 오차 분리, 99% 성공 정의·KPI, failure modes, 검증 계획',
    docs: [
      `${DOCS}/0. forklift_docking_problem_definition.md`,
      `${DOCS}/critic.md  (특히 §1, §7 timestamp, §21~§26)`,
      `${DOCS}/1. perception_methodology_v2.md  §11·§17·§18`,
      `${DOCS}/3. vehicle_dynamics_methodology.md  §12~§17`,
      `${DOCS}/4. controller_methodology.md  §20~§24`,
      `${DOCS}/PLAN_legacy.md  §4 개발순서·2차실험 측정목록 (참고만)`,
      `${DOCS}/prior_art_docking.md  (A26 NISTIR 8140, B26·B27 지연, C3 PX4)`,
      `${ROOT}/CLAUDE.md  (실측치·브랜치·CAN 규칙)`,
    ],
    issues: [
      'Timestamp 체계 — 캡처/수신/검출/판단/송신 시각을 어떻게 기록하고 latency 를 어떻게 실측하나(카메라 버퍼링 포함)',
      'Perception 오차와 Dynamics 오차 분리 — timing 정리 → dynamics 로깅 → 피팅 순서를 강제하는 절차',
      '99% 성공 정의 — tolerance(T_f/T_l/T_h) 확정 방법과 KPI(median/95/99/max, correction 수, 시간, abort율)',
      'Failure mode 7종(wrong-branch lock-in, false confidence, ping-pong, deadzone, 오차혼합, wait deadlock, zone chattering) 각각의 감지·대응·검증',
      '블라인드 최종구간 해소 — **실전은 포크가 태그면을 지나 삽입 깊이 D 만큼 들어감**(실험의 DOCK_EXTRA_M=−2.02 는 벽 충돌 방지용 임시). 카메라는 포크끝 1.52m 뒤라 실전 블라인드 = 3.3 − (1.52 − D) ≈ 1.8+D m (D 0.5~1.0 → 2.3~2.8 m, 실험 1.28 m 의 2배↑, 추측항법 ±25 cm 급). 후보: 태그 높이/카메라 피치, 2번째 저위치 태그, depth 종방향 정지, 3.3 m 정렬 품질+직진 — 비용·정확도·실차 검증법 비교',
      '시뮬레이터 검증 계획 — 실측 파라미터·잡음·flip 주입으로 각 설계 변경을 A/B 하는 하네스(v1 sim 이식)',
      '현장 실험 목록·순서 — byte2 스윕, σ_τ, R(d,φ), 프레임별 로그(bag), 최소 제어량. 한 번 갔을 때 다 얻는 순서',
      '문서 통합 — 2·3 dynamics 문서 병합, plan.md 구조, 코드 단계로 넘어가는 완료 조건',
    ],
  },
}

// ── 공통 텍스트 ─────────────────────────────────────────────────────────────
const n = (args && args.n) ? Number(args.n) : 1
const P = PROBLEMS[n]
if (!P) throw new Error(`unknown problem n=${n}`)

const prevPlan = n > 1
  ? [`${ROOT}/apriltag_v3/Agent/plan.md  (앞선 대문제의 확정 결론·"다른 대문제로 이관" 항목 — 전제로 삼고, 모순되면 근거를 들어 명시할 것)`]
  : []
const docList = [...P.docs, ...prevPlan].map(d => `- ${d}`).join('\n')
const issueList = P.issues.map((s, i) => `${i + 1}. ${s}`).join('\n')

const CONTEXT = `
# 대문제 ${n}: ${P.title}

## 반드시 읽을 파일 (읽기 전용 — 절대 수정하지 말 것)
${docList}
- ${ROOT}/CLAUDE.md  (실측치: 지연 ~0.5s 주행/0.85s 회전, 정지관성 ~14cm, 회전관성 ~1.3°, 정속 0.28m/s, 회전팔 A 미확정(1.46 철회 — 대문제 1 결론), 7~10m lateral 잡음 60~145mm vs 허용치 30mm)

## 이 대문제의 쟁점 (번호를 그대로 써서 답할 것)
${issueList}

## 설계 전제 (바꿀 수 없음)
- CAN 명령: stop / forward / backward / 제자리회전 L·R 만. 곡선(조향+전진 동시) 주행 없음.
- byte2 스로틀 최소 2단(풀 67 / 저속 97, 광운대 검증값), 데드밴드 있음, 비례 여부 미확인.
- 자이로는 짧은 다리(회전 정지 판정, 정면 구간)로만. 카메라가 절대 기준. 초기 탐색에 자이로 기억 불필요.
- 사이드스텝(90° 회전) 폐기. 태그를 화면 중앙(β≈0)에 유지하며 소각 대각 접근.
- 목표: 정확한 하나의 값이 아니라 **불확실성 아래서 oscillation 없이 수렴하는 폐루프**.
- **상수 최소화 원칙**: 하드코딩 상수는 물리 사실(태그 크기·장착 기하)만. 그 자리에서 직접·깨끗하게 관측되는 양(바이어스·잡음 수준·회전팔 A 등)은
  **실시간 도출**(정지 시 재추정·잔차·회귀). 실시간으로 안 보이는 오차(예: 정면 heading 치우침)는 표로 모델링하지 말고 **그 영역을 회피하는 규칙 1개**로.
  잡음 신호를 미분해 쫓는 온라인 적응은 금지. 제안마다 "새 상수 n개 / 실시간 도출 m개" 를 명시하고 상수는 근거 없이 늘리지 않는다.
  **config 에는 현장에서 사람이 정하는 값만** 올린다(차량·현장 바뀌면 바꿀 것). 필터 창·타임아웃·N프레임 같은 구현 세부는 코드 내부 고정값으로 두고
  config 후보로 제안하지 않는다. 기존 config 상수 중 새 설계로 불필요해지는 것은 **삭제 목록**으로 명시한다(상수 수 순감이 목표).
`

// ── 모델 / effort 정책 ──────────────────────────────────────────────────────
// 1차: 세션 모델(현재 Fable 5.1) 상속 + effort max (서브에이전트 호출이 받는 최고 단계).
// agent() 가 null 을 돌려주면(재시도 후에도 API 종료 오류 = 토큰/한도 소진 등) 같은 프롬프트를
// Opus 5 로 한 번 더 시도한다. effort 는 그대로 max. (사용자가 에이전트를 skip 한 경우도 null 이라 재시도됨 — 감수)
// 세션 자체의 한도 대기·자동 재개는 .claude/settings.json 의 autoContinueAtUsageLimit 가 담당.
const EFFORT = 'max'
const FALLBACK_MODEL = 'opus'   // = Opus 5
async function call(prompt, opts) {
  // 1단계: 세션 모델(Fable) — 여기는 반드시 원시 agent() 를 부른다 (call 재귀 금지)
  const primary = await agent(prompt, { ...opts, effort: EFFORT })
  if (primary !== null && primary !== undefined) return primary
  // 2단계: Fable 불가 → Opus 5, effort 그대로
  log(`${opts.label || 'agent'}: 1차 실패(모델 한도/오류) → ${FALLBACK_MODEL} 로 재시도 (effort ${EFFORT})`)
  const fb = await agent(prompt, { ...opts, model: FALLBACK_MODEL, effort: EFFORT })
  if (fb !== null && fb !== undefined) return fb
  // 3단계: 둘 다 실패 = 전체 한도 소진 → 여기서 멈추고 진행 캐시 보존. 한도 재생성 후 resumeFromRunId 로 이어서.
  throw new Error(`QUOTA_EXHAUSTED at ${opts.label || 'agent'}: Fable·Opus 모두 실패. 한도 재생성 후 resumeFromRunId 로 이어서 실행`)
}

// ── Phase 1: Propose ────────────────────────────────────────────────────────
phase('Propose')
log(`대문제 ${n} (${P.key}) 토론 시작 — proposer 제안`)
const proposal = await call(
  `${CONTEXT}
너의 역할 정의(system prompt)에 따라 위 쟁점 ${P.issues.length}개 각각에 대해 제안하라.
쟁점 번호를 그대로 쓰고, 근거 없는 제안은 쓰지 마라. 출력 형식은 역할 정의의 형식을 따른다.`,
  { agentType: 'proposer', label: 'proposer', phase: 'Propose' },
)
if (!proposal) throw new Error('proposer returned nothing')

// ── Phase 2: Critique (2 lenses, parallel — Rebut 는 둘 다 필요하므로 barrier 정당) ──
phase('Critique')
const LENSES = [
  { key: 'safety', name: '실차 안전·신뢰성 렌즈',
    focus: '그대로 구현했을 때 실차에서 깨지는 경우. critic.md 의 failure mode(ping-pong, deadzone, wait deadlock, zone chattering, false confidence, wrong-branch lock-in, latency/dynamics 오차 혼합)와 대조. 오작동 시 안전(수동전환·abort).' },
  { key: 'precision', name: '정밀도·수렴 렌즈',
    focus: '최종 ±3cm/±2° 가 실제로 나오나. 잡음(60~145mm)>허용치(30mm) 구조를 정말 벗어나나. 추정기·모델·컨트롤러가 같은 오차를 중복 해석하지 않나. 수렴 스텝 수와 시간. 저속에서 v̂ 잡음.' },
]
const critiques = (await parallel(LENSES.map(L => () => call(
  `${CONTEXT}
## 너의 렌즈: ${L.name}
${L.focus}

## proposer 의 제안 (이걸 비판하라)
${proposal}

역할 정의의 형식대로 쟁점별 [판정/반박 근거/대안/대안 근거/실차 검증법] 을 쓰라. 옳은 제안은 옳다고 명시하라.`,
  { agentType: 'critic', label: `critic:${L.key}`, phase: 'Critique' },
)))).filter(Boolean)
if (!critiques.length) throw new Error('no critiques')
const critiqueText = LENSES.map((L, i) => `### [${L.name}]\n${critiques[i] || '(없음)'}`).join('\n\n')

// ── Phase 3: Rebut ──────────────────────────────────────────────────────────
phase('Rebut')
const rebuttal = await call(
  `${CONTEXT}
너는 방금 아래 제안을 낸 proposer 다. 두 critic 의 반박을 받았다.

## 너의 원래 제안
${proposal}

## critic 반박 (2렌즈)
${critiqueText}

쟁점별로 **재반박 또는 수용**을 쓰라. 형식:
### 쟁점 N
- 수용/유지/수정: <어느 쪽>
- 이유+근거: <반박이 맞으면 왜 맞는지, 틀리면 왜 틀린지 — 출처 명시>
- 수정된 제안: <바뀌었으면 최종 형태 한두 문장>
근거 없이 버티지 마라. 반박이 더 강한 근거를 가졌으면 수용하라.`,
  { agentType: 'proposer', label: 'proposer:rebuttal', phase: 'Rebut' },
)

// ── Phase 4: Judge (구조화 출력) ────────────────────────────────────────────
const VERDICT_SCHEMA = {
  type: 'object',
  properties: {
    issues: {
      type: 'array',
      items: {
        type: 'object',
        properties: {
          num: { type: 'integer', description: '쟁점 번호' },
          name: { type: 'string' },
          verdict: { type: 'string', enum: ['proposer', 'critic', 'compromise', 'hold'] },
          decisive_evidence: { type: 'string', description: '판정을 가른 근거와 출처' },
          conclusion: { type: 'string', description: 'plan.md 에 넣을 결론 1~3문장. 구체적 방법 + 파라미터 자리' },
          verification: { type: 'string', description: '이 결론을 확인할 실차/시뮬 실험 1개' },
          needs_research: { type: 'boolean' },
          research_query: { type: 'string', description: 'needs_research 면 검색 질문' },
          unresolved: { type: 'string' },
        },
        required: ['num', 'name', 'verdict', 'conclusion', 'needs_research'],
      },
    },
    key_conclusions: { type: 'array', items: { type: 'string' }, description: '이 대문제의 핵심 결론 3줄' },
    top_risks: { type: 'array', items: { type: 'string' }, description: '남는 가장 큰 리스크 2개' },
  },
  required: ['issues', 'key_conclusions'],
}

phase('Judge')
const judgePrompt = (extra) => `${CONTEXT}
너는 judge 다. 아래 왕복을 역할 정의의 판정 기준(근거 강도 > failure mode 회피 > 전제 준수 > 실차 검증 가능 > 단순성)으로 채점해 쟁점별 결론을 내라.
근거가 부족해 결정 못 하는 쟁점은 verdict=hold, needs_research=true 로 두고 research_query 에 검색 질문을 써라. 억지로 정하지 마라.

## proposer 제안
${proposal}

## critic 반박 (2렌즈)
${critiqueText}

## proposer 재반박
${rebuttal}
${extra || ''}`

let verdict = await call(judgePrompt(), { agentType: 'judge', label: 'judge', phase: 'Judge', schema: VERDICT_SCHEMA })
if (!verdict) throw new Error('judge returned nothing')

// ── Phase 5: Research (근거 부족 쟁점만) ────────────────────────────────────
phase('Research')
const pending = (verdict.issues || []).filter(i => i.needs_research && i.research_query)
log(`judge 1차: hold ${pending.length}개 / 전체 ${(verdict.issues || []).length}개`)

const researchOne = async (iss) => {
  const p = `${CONTEXT}
## 조사할 쟁점 ${iss.num}: ${iss.name}
검색 질문: ${iss.research_query}
judge 의 보류 사유: ${iss.unresolved || '(없음)'}
prior_art_docking.md 에 이미 있는 항목은 재검색하지 말고, 새 근거만 역할 정의의 표 형식으로 돌려라.`
  try {
    return await call(p, { agentType: 'researcher', label: `researcher:${iss.num}`, phase: 'Research' })
  } catch (e) {
    // researcher 타입이 아직 등록 안 됐으면 일반 에이전트가 정의 파일을 읽고 같은 역할을 한다
    log(`researcher 타입 실패(${e && e.message}) → general-purpose 로 대체`)
    return await call(
      `먼저 ${ROOT}/.claude/agents/researcher.md 를 읽고 그 역할·형식·권한을 그대로 따르라(파일 수정 금지, 웹 검색 가능).\n${p}`,
      { agentType: 'general-purpose', label: `researcher-fallback:${iss.num}`, phase: 'Research' },
    )
  }
}
const research = pending.length
  ? (await parallel(pending.map(iss => () => researchOne(iss).then(r => ({ num: iss.num, name: iss.name, result: r }))))).filter(Boolean)
  : []

// ── Phase 6: Final Judge (조사 반영 재판정) ─────────────────────────────────
phase('Final Judge')
let final = verdict
if (research.length) {
  const researchText = research.map(r => `### 쟁점 ${r.num}: ${r.name}\n${r.result || '(조사 실패)'}`).join('\n\n')
  final = await call(
    judgePrompt(`
## researcher 조사 결과 (보류했던 쟁점에 대한 새 근거)
${researchText}

위 새 근거를 반영해 **모든 쟁점을 다시** 판정하라. 여전히 근거가 없으면 hold 로 두되 unresolved 에 "실차 실험으로 결정: <실험>" 을 쓰라.`),
    { agentType: 'judge', label: 'judge:final', phase: 'Final Judge', schema: VERDICT_SCHEMA },
  ) || verdict
}
const holds = (final.issues || []).filter(i => i.verdict === 'hold').length
log(`대문제 ${n} 완료 — 결론 ${(final.issues || []).length - holds}개, 보류 ${holds}개`)

return {
  problem: n, key: P.key, title: P.title, issues: P.issues,
  proposal, critiques: LENSES.map((L, i) => ({ lens: L.name, text: critiques[i] || null })),
  rebuttal, verdict, research, final,
}
