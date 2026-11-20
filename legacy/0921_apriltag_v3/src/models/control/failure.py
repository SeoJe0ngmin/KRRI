"""고장 모드 대장·실시간 감지·Tier 사다리·ABORT 절차. (plan 4-4 표, plan 3-7 Tier)

C팀 소유 두 파일 중 하나. **dock_fsm.py 가 이 파일을 import 한다**(반대는 없다).
순환 import 를 피하려고 여기서는 상태(Phase)를 **문자열로만** 가리킨다.

    Tier / FailureCode / AbortReason   계약 §4.1 enum. dock_fsm 이 그대로 재수출한다
    FAILURE_MODES                      FM1~FM11 : 로그코드·감지조건·대응상태·고장주입 시험
    rotation_consistency / straight_consistency / can_health   실시간 감지 보조
    FailureMonitor                     카운터 + "원인당 1회" 장부 + 반전 장부
    AbortKeeper                        ABORT 뒤에도 stop 프레임 + heartbeat 를 계속 보낸다

**FM8b(호스트 동결)에는 소프트웨어 대응이 없다.** 데드맨도 같은 프로세스의 스레드라
호스트가 멈추면 같이 멈춘다(9/7 VM 사건). 남는 건 차량 워치독 T_wd 뿐인데 그건 아직
미측정이다 — 그래서 이 파일은 세션마다 **"미완화 리스크"** 를 한 번 크게 찍는다.
대응은 코드가 아니라 사람이다: 사람이 제동 위치에 서고, 전진은 97 로만.

**wrong-DONE(FM10)이 1차 하드 KPI다.** 애매하면 DONE 하지 말고 ABORT — 이 파일의
모든 문턱은 "모르면 안 움직이고, 못 미더우면 성공이라고 안 한다" 쪽으로 기운다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, IntEnum


# ── enum (계약 §4.1) ────────────────────────────────────────────────────────

class Tier(IntEnum):
    """대응 사다리 (plan 3-7). 숫자가 클수록 심하다."""
    NONE = 0
    WAIT = 1          # Tier 0 — σ 가 아직 줄고 있다. 그대로 관측
    REPOSITION = 2    # Tier 1 — 관측성 개선 동작
    CAUSE = 3         # Tier 2 — 원인 분기 1회
    ABORT = 4         # Tier 3 — 중단


class FailureCode(Enum):
    """events.jsonl 에 남는 고장 코드 (plan 4-4)."""
    FM1_WRONG_BRANCH = "FM1"
    FM2_FALSE_CONF = "FM2"
    FM3_PING_PONG = "FM3"
    FM4_DEADZONE = "FM4"
    FM5_ERR_MIX = "FM5"
    FM6_WAIT_DEADLOCK = "FM6"
    FM7_ZONE_CHATTER = "FM7"
    FM8A_THREAD_LAG = "FM8a"
    FM8B_HOST_FREEZE = "FM8b"
    FM9_CAN = "FM9"
    FM10_WRONG_DONE = "FM10"
    FM11_DUAL_SOURCE = "FM11"


class AbortReason(Enum):
    """Tier 3 집합 (plan 3-7). 전부 events.jsonl 에 남는다."""
    CALIB = "calib"                   # 캘리브 부재·거부
    GYRO_DEAD_BLIND = "gyro_dead_blind"
    REINIT_TWICE = "reinit_twice"     # 추정기 재초기화 ≥2
    ENVELOPE = "envelope"             # 시작 포락선 밖인데 dog-leg 로도 못 들어옴
    NO_RESPONSE = "no_response"
    HARD_CAP = "hard_cap"
    TIME_BUDGET = "time_budget"
    QUANT_GAP = "quant_gap"           # 양자화 공백 — "저속 회전 단 필요" 로그 동반
    SIGN_CHECK = "sign_check"         # (I1) 잔차 > 0.2° — **배선 부호**가 어긋났다.
                                      # 원관측(픽셀 경로 vs PnP 경로)이 같이 틀어졌을 때만
    WRONG_BRANCH = "wrong_branch"     # [추가] (I1) 은 깨졌는데 **원관측은 멀쩡하다** —
                                      # 배선이 아니라 PnP 2중해가 거짓 해로 잠긴 것(FM1).
                                      # 계약 §4.1 에 없지만 갈라 찍지 않으면 현장에서
                                      # "부호 배선 의심" 으로 오독한다(D팀 보고 A,
                                      # 시뮬 200회 중 39회가 이것이었다). 대응은 같다 —
                                      # 즉시 정지 + Tier 3. 이름만 진실을 말한다
    CAN_FAULT = "can_fault"
    OPERATOR = "operator"
    NO_CONVERGENCE = "no_convergence"  # [추가] Tier 1 재배치를 다 썼는데 판정이 안 선다.
                                      # 계약 §4.1 에 없지만 'no_response'(차가 반응 안 함)로
                                      # 적으면 사람이 원인을 잘못 짚는다
    INTERNAL = "internal"             # [추가] 상태기계 내부 예외·계약 위반.
                                      # 계약 §4.1 목록에 없지만 **없으면 다른 사유를
                                      # 사칭하게 된다** — 사칭보다 항목 하나가 낫다


# ── 대장 (plan 4-4 표 그대로) ───────────────────────────────────────────────

@dataclass(frozen=True)
class FailureMode:
    code: FailureCode
    title: str        # 한 줄 이름
    detect: str       # 실시간 감지 조건
    respond: str      # 대응 상태 (Phase 는 문자열로만)
    test: str         # 고장주입 시험 (plan 4-4 검증)
    mitigated: bool = True    # False = 소프트웨어 대응이 **없다**


_MODES = (
    FailureMode(
        FailureCode.FM1_WRONG_BRANCH, "PnP 2중해 오선택",
        "pitch-cue SPRT(A팀) ∧ 직진 다리 ≥1 m 에서 Δψ_cam 이 자이로와 어긋남"
        "(거짓 해는 −2Δφ 로 흐른다) 또는 Δβ_obs 와 Δβ_pred(ψ̂) 의 부호·크기 불일치. "
        "**회전 뒤 |Δψ_cam−Δψ_IMU| 는 판별력이 없다**(거짓 해 비 0.77~1.0) — 그건 FM2",
        "ambiguous 로 보고 FALLBACK(oblique 재획득) 1회. 재획득 30프레임 전엔 NEAR 결정 금지",
        "9/7 bag 회전 사건에 거짓 해 강제 → oblique 직진 재생에서 드리프트 ≥2°"),
    FailureMode(
        FailureCode.FM2_FALSE_CONF, "σ 를 믿었는데 틀림(재앵커 바이어스)",
        "회전 뒤 |Δψ_cam − Δψ_IMU| > max(k·σ_random, 1°) 또는 "
        "|Δℓ| > Â_max·|sinΔψ| + k·σ_ℓ. 1/3/10 s 창 z-커버리지·NIS 경보는 A팀",
        "카메라 재앵커·반전 **금지**(ψ̂ 는 자이로 증분 전파), σ 바닥 확대, 반복 시 Tier 1",
        "3.5 m·tilt 25° 정지에서 IMU 회전 ±5° ×10 의 Δψ_cam/Δψ_IMU 분포"),
    FailureMode(
        FailureCode.FM3_PING_PONG, "좌우 왕복(유령 오차 추종)",
        "같은 구역에서 보정 부호 반전 2회, 또는 STOPPING 중에 stop 아닌 명령이 남",
        "반전은 구역당 1회만 허용. 초과면 Tier 1, 또 나오면 Tier 3",
        "STOPPING 중 반대부호 프레임 주입 → 명령 0 이어야 한다"),
    FailureMode(
        FailureCode.FM4_DEADZONE, "데드존·양자화 공백",
        "명령 뒤 τ_start + 여유 안에 v̂·ω̂ 둘 다 잡음 수준(움직이지 않음), "
        "또는 필요한 보정이 최소 신뢰 증분 δ_q 보다 작아 낼 명령이 없음",
        "Tier 3 ABORT + **'저속 회전 단 필요'** 로그(plan 2-4 조건부 항목을 필수로 올릴 근거)",
        "양자화 공백 주입: 정면 회전 뒤 F̂ ∈ (T−kσ, θ_min,inc·(d_T−x_ref))"),
    FailureMode(
        FailureCode.FM5_ERR_MIX, "오차 혼합(타이밍이 섞여 캘리브가 오염)",
        "타이밍 게이트 미통과(gate.passed=False)인데 캘리브를 쓰려 함",
        "실주행 거부(dry-run 만). 캘리브 생산도 금지 — 이 판정은 calib.gate() 가 한다",
        "L 주입 {0,150,300} ms × 67 정속정지 n≥5 → τ_eff 이동 < σ_e/2"),
    FailureMode(
        FailureCode.FM6_WAIT_DEADLOCK, "WAIT 교착",
        "Tier 0 WAIT 중 σ_eff 가 더 안 줄어듦(dσ/dt ≈ 0). σ 가 바닥(floor)에 눌려 있으면 "
        "기다려도 영원히 안 줄어든다",
        "즉시 Tier 1 REPOSITION(관측성 개선 동작). 시간 대기로 버티지 않는다",
        "lateral_valid=False 를 10 s 주입 → 캡 안에 Tier 1 도달"),
    FailureMode(
        FailureCode.FM7_ZONE_CHATTER, "구역 왕복",
        "FAR→NEAR→FINAL 역행 요구(σ 재증가 등)",
        "구역은 **단조**. 역행은 아예 안 하고 Tier 1 이벤트로만 기록",
        "경계 ±2σ_x 잡음 주입 시뮬에서 FAR↔NEAR 왕복 0"),
    FailureMode(
        FailureCode.FM8A_THREAD_LAG, "루프·센서 스레드 지연",
        "프레임 stale(>150 ms) · 자이로 gaps 증가 · CAN write 간격 급증 · 데드맨 발화",
        "즉시 stop. 블라인드 중이면 Tier 3",
        "검출 스레드 300 ms sleep 주입 → stale 술어로 stop(CAN 로그 시각 대조)"),
    FailureMode(
        FailureCode.FM8B_HOST_FREEZE, "호스트 동결",
        "**감지 불가** — 판단 루프·데드맨 스레드가 같이 멈춘다(9/7 VM 사건)",
        "**소프트웨어 대응 없음 = 미완화 리스크.** 차량 워치독 T_wd 실측 전까지 "
        "사람이 제동 위치 + 전진 97 한정. 세션마다 이 사실을 로그·화면에 남긴다",
        "kill -STOP 2 s → 차가 어떻게 되는지 사람이 본다(코드가 할 수 있는 게 없다)",
        False),
    FailureMode(
        FailureCode.FM9_CAN, "CAN 이상",
        "TXACK 부재·오류프레임·error passive·bus-off·차량 EMCY(0x08x). "
        "write 간격 p99 급증도 같이 본다",
        "stop 홀드 → Tier 3 CAN_FAULT. 복구 순서(stop 127 → driving_mode → NMT)는 실측 후",
        "listen-only 60 s + 차단 4종 ×3 (first_run safety)"),
    FailureMode(
        FailureCode.FM10_WRONG_DONE, "성공이 아닌데 성공이라 함",
        "실시간 감지 불가(사후 줄자). 실시간 규칙은 하나 — **수용식을 못 넘었으면 DONE 금지**",
        "DONE 대신 DONE_UNVERIFIED 또는 ABORT. --final-anyway 로 들어간 FINAL 은 "
        "**무조건 DONE_UNVERIFIED**. 이게 1차 하드 KPI(0/n)",
        "명목 셀 n=20 최종 정지 외부 줄자 → 평행사변형 판정과 대조"),
    FailureMode(
        FailureCode.FM11_DUAL_SOURCE, "이중 송신원(리모컨·조이스틱)",
        "우리가 안 보낸 0x1E3 수신, 또는 사람이 조이스틱으로 개입",
        "RELEASE — movement 송신 중단, heartbeat 만. 사람 호출",
        "수신기 켠 채 동일/다른 데이터 → bus-off 시간, 조이스틱 개입 ×3"),
)

FAILURE_MODES = {m.code: m for m in _MODES}


def table(log=print):
    """대장 한 화면. 출발 전에 한 번 찍어 둔다."""
    log("고장 모드 대장 (plan 4-4)")
    for m in _MODES:
        mark = " " if m.mitigated else "!"
        log("  %s %-5s %-22s → %s" % (mark, m.code.value, m.title, m.respond.split(".")[0]))
    log("  ! = 소프트웨어 대응 없음(미완화 리스크)")


# ── 실시간 감지 보조 ────────────────────────────────────────────────────────
#: 두 센서 차의 최소 문턱 [°]. σ̂ 가 낙관적으로 작아도 이 아래로는 안 내려간다
CROSS_FLOOR_DEG = 1.0
#: 직진 사후 일관성을 볼 최소 다리 길이 [m] (plan 4-4 FM1: 짧은 다리는 판별력 없음)
STRAIGHT_MIN_LEG_M = 1.0


def _fin(*vals):
    return all(v is not None and isinstance(v, (int, float)) and math.isfinite(v) for v in vals)


def rotation_consistency(dpsi_cam, dpsi_imu, sigma_deg, dlat_m=None,
                         a_max_m=None, sigma_lat_m=None, k=2.0):
    """⓪ 회전 뒤 교차센서 사후 일관성 (plan 3-5 ⓪, FM2).

    (ok, detail) 을 돌려준다. **ok=False 면 카메라 재앵커·반전 금지**이고 ψ̂ 는
    자이로 증분으로 전파해야 한다. 문턱에 정면 바이어스 b 를 일부러 **안** 넣는다 —
    넣으면 바이어스가 클수록 검사가 무뎌져 9/7 의 8→9→10 반전을 그대로 통과시킨다.
    """
    d = {"dpsi_cam": dpsi_cam, "dpsi_imu": dpsi_imu}
    if not _fin(dpsi_cam, dpsi_imu):
        d["note"] = "값 없음 — 검사 못 함"
        return True, d
    thr = max(k * float(sigma_deg or 0.0), CROSS_FLOOR_DEG)
    diff = float(dpsi_cam) - float(dpsi_imu)
    d.update({"diff_deg": diff, "thr_deg": thr})
    ok = abs(diff) <= thr
    if _fin(dlat_m, a_max_m):
        lat_thr = abs(a_max_m) * abs(math.sin(math.radians(dpsi_imu))) \
            + k * float(sigma_lat_m or 0.0)
        d.update({"dlat_m": dlat_m, "dlat_thr_m": lat_thr})
        if abs(dlat_m) > lat_thr:
            ok = False
            d["lat_violation"] = True
    return ok, d


def straight_consistency(dpsi_cam, dpsi_imu, leg_m, sigma_deg, k=2.0):
    """직진 다리 뒤 일관성 (FM1 wrong-branch). 짧은 다리(<1 m)는 판별력이 없어 건너뛴다."""
    d = {"dpsi_cam": dpsi_cam, "dpsi_imu": dpsi_imu, "leg_m": leg_m}
    if not _fin(dpsi_cam, dpsi_imu, leg_m) or abs(leg_m) < STRAIGHT_MIN_LEG_M:
        d["note"] = "다리가 짧거나 값 없음 — 검사 안 함"
        return True, d
    thr = max(k * float(sigma_deg or 0.0), CROSS_FLOOR_DEG)
    diff = float(dpsi_cam) - float(dpsi_imu)
    d.update({"diff_deg": diff, "thr_deg": thr})
    return abs(diff) <= thr, d


def can_health(stats, base_trips=0, gap_max_ms=200.0, txack_max_ms=20.0):
    """CAN 건강 한 줄 (FM9·FM8a). stats = SafeCanTx.stats().

    **창(최근 몇 초) 값만 본다.** 예전에는 세션 누적 최대(`write_gap_max_ms`)와
    세션 첫 10건 TXACK 을 봤는데 둘 다 단조라 한 번 튀면 영원히 not-ok 였다 —
    일시적 지연 1회가 남은 주행 내내 모든 EXECUTE 를 다음 프레임에 정지시켜
    180 s 예산을 제자리에서 태웠다(2026-09-21 재현: 1 cm 이동, FM9 5250건).
    첫 10건 TXACK 은 **출발 전 게이트**(first_run timing)에서만 쓴다.
    """
    s = dict(stats or {})
    bad = []
    trips = int(s.get("deadman_trips") or 0)
    if trips > base_trips:
        bad.append("데드맨 %d회" % (trips - base_trips))
    g = s.get("write_gap_win_max_ms")
    if _fin(g) and g > gap_max_ms:
        bad.append("write 간격(최근 창) %.0f ms" % g)
    t = s.get("txack_win_max_ms")
    if _fin(t) and t > txack_max_ms:
        bad.append("TXACK(최근 창) %.1f ms" % t)
    return (not bad), {"why": ", ".join(bad), "stats": s}


# ── 장부 ────────────────────────────────────────────────────────────────────

class FailureMonitor:
    """고장 기록·카운터·'원인당 1회' 장부. 판단은 dock_fsm 이 하고 기록은 여기가 한다."""

    def __init__(self, log=None, logger=None):
        self.log = log
        self.logger = logger
        self.counts = {}
        self.notes = []            # (t, code, why)
        self._causes = set()       # Tier 2 원인당 1회
        self._reversals = {}       # zone -> 쓴 횟수
        self._said_8b = False

    # -- 기록 --------------------------------------------------------------
    def note(self, code, why, t=None, tier=None, **fields):
        key = code.value if isinstance(code, FailureCode) else str(code)
        self.counts[key] = self.counts.get(key, 0) + 1
        self.notes.append((t, key, why))
        if self.log:
            self.log("  [%s] %s" % (key, why))
        if self.logger is not None:
            self.logger.event("failure", fm=key, why=why, t=t,
                              tier=(int(tier) if tier is not None else None),
                              n=self.counts[key], **fields)
        return key

    def count(self, code):
        return self.counts.get(code.value if isinstance(code, FailureCode) else str(code), 0)

    # -- Tier 2 : 원인당 1회 ------------------------------------------------
    def cause_once(self, key):
        """그 원인으로 처음 분기하는 거면 True. 두 번째부터는 False(→ Tier 3)."""
        if key in self._causes:
            return False
        self._causes.add(key)
        return True

    # -- 반전 : 구역당 1회 --------------------------------------------------
    def reversal_allowed(self, zone):
        return self._reversals.get(int(zone), 0) < 1

    def mark_reversal(self, zone):
        self._reversals[int(zone)] = self._reversals.get(int(zone), 0) + 1
        return self._reversals[int(zone)]

    # -- FM8b : 세션에 한 번 크게 --------------------------------------------
    def unmitigated_notice(self, t=None):
        """호스트 동결은 대응이 없다. 그 사실 자체를 남긴다(plan 4-4 FM8b)."""
        if self._said_8b:
            return
        self._said_8b = True
        m = FAILURE_MODES[FailureCode.FM8B_HOST_FREEZE]
        if self.log:
            self.log("  !! 미완화 리스크 FM8b 호스트 동결 — 소프트웨어 대응이 없다.")
            self.log("     사람이 제동 위치에 서 있을 것. 전진은 97 로만. (차량 워치독 T_wd 미측정)")
        if self.logger is not None:
            self.logger.event("unmitigated_risk", fm=m.code.value, title=m.title,
                              detect=m.detect, respond=m.respond, t=t)

    def summary(self):
        return {"counts": dict(self.counts), "causes": sorted(self._causes),
                "reversals": dict(self._reversals), "n": len(self.notes)}


# ── ABORT 절차 ──────────────────────────────────────────────────────────────

class AbortKeeper:
    """ABORT 뒤에도 **stop 프레임 + heartbeat 를 계속 보낸다**(버스 침묵 금지, plan 3-7).

    차량 워치독 T_wd 가 미측정이고 Curtis 류는 PDO Timeout 0 = **마지막 명령 유지**라,
    버스를 조용히 두면 직전 명령이 latch 될 수 있다. heartbeat 자체는
    control_forklift_v2 의 TX 루프가 200 ms 로 계속 보내므로, 우리가 할 일은
    **current_movement 를 stop 으로 계속 다시 못 박고 데드맨을 먹이는 것**이다.

    RELEASE(FM11 조이스틱 개입)일 때만 movement 송신을 멈추고 heartbeat 만 남긴다.
    """
    RESEND_S = 0.5                # stop 을 다시 못 박는 주기 [s]

    def __init__(self, tx, log=None, logger=None):
        self.tx = tx
        self.log = log
        self.logger = logger
        self.reason = None
        self.released = False
        self.t_start = None
        self.t_last = None
        self.resends = 0

    def start(self, reason, why="", t=None):
        self.reason = reason
        self.t_start = t
        if self.log:
            self.log("  ** ABORT (%s) %s" % (getattr(reason, "value", reason), why))
            self.log("     stop 프레임 + heartbeat 를 계속 보낸다. 사람을 부를 것.")
        if self.logger is not None:
            self.logger.event("abort", reason=getattr(reason, "value", reason),
                              why=why, t=t)
        self._send(t, first=True)

    def release(self, why="", t=None):
        """FM11 — 사람이 잡았다. movement 송신을 놓고 heartbeat 만 남긴다."""
        self.released = True
        if self.log:
            self.log("  ** RELEASE — movement 송신 중단, heartbeat 만. (%s)" % why)
        if self.logger is not None:
            self.logger.event("release", why=why, t=t)

    def pump(self, t=None):
        """매 루프 한 번. 데드맨을 먹이고 주기마다 stop 을 다시 못 박는다."""
        if self.tx is None:
            return False
        try:
            self.tx.feed()
        except Exception:
            pass
        if self.released:
            return False
        if self.t_last is None or t is None or (t - self.t_last) >= self.RESEND_S:
            self._send(t)
            return True
        return False

    def _send(self, t=None, first=False):
        self.t_last = t
        if self.tx is None:
            return
        try:
            self.tx.stop(why="abort:%s" % getattr(self.reason, "value", self.reason))
            self.resends += 1
        except Exception as exc:
            if self.log and first:
                self.log("  !! ABORT 중 stop 송신 실패: %s" % exc)

    def stats(self):
        return {"reason": getattr(self.reason, "value", self.reason),
                "released": self.released, "stop_resends": self.resends}


__all__ = ["Tier", "FailureCode", "AbortReason", "FailureMode", "FAILURE_MODES",
           "table", "rotation_consistency", "straight_consistency", "can_health",
           "FailureMonitor", "AbortKeeper", "CROSS_FLOOR_DEG", "STRAIGHT_MIN_LEG_M"]
