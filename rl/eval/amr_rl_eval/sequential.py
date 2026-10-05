"""
0 충돌 순차 감시 — Gazebo 에피소드마다 결과를 보며 접촉률 NI(δ = 2 %p) 를 선언한다.

근거: 이론편 「0충돌 실증은 순차 검정과 잘 맞는다」 절, evaluation_statistics_theory.md §4
(calc4.py·calc6.py).

H0: p ≥ p0 (p0 = 기준 접촉률 + δ, DWA 0 회면 δ 자체) 대 H1: p = p1.
  무충돌 에피소드마다 Λ += s = ln((1 − p1)/(1 − p0)), 충돌마다 Λ += f = ln(p1/p0).
  Λ ≥ A 이면 NI 선언. A = ln(1/α) (Ville 경계) 면 p ≥ p0 에서 우도비가 supermartingale 이므로
  매 에피소드 들여다봐도 1 종 오류 ≤ α (Wald 근사 아님). Wald 경계 A = ln((1−β)/α) 는 더 일찍 끝나지만
  근사다 (실제 α 가 α/(1−β) 까지 갈 수 있다).
  p0 = 2 %, p1 = 0.5 %, α = 0.025, Ville: 0 충돌이면 243 회에 종료, 충돌 1 회마다 약 92 회 연장.

success-run e-process E_n = (1 − p0)^(−n)·1{S_n = 0} 는 대립 p1 = 0 의 특수형이다.
"""

from dataclasses import dataclass, field
import math

import numpy as np


@dataclass
class SPRTDesign:
    """접촉률 순차 검정 설계 (사전 등록 대상 — PROTOCOL.md §6.2)."""

    p0: float = 0.02
    p1: float = 0.005
    alpha: float = 0.025
    beta: float = 0.2
    boundary: str = 'ville'      # 'ville' = ln(1/α), 'wald' = ln((1−β)/α)
    futility: bool = False       # True 면 하한 B = ln(β/(1−α)) 에서 'NI 포기' 로 끝낸다
    n_max: int = 600

    @property
    def upper(self):
        """NI 선언 경계 A."""
        if self.boundary == 'ville':
            return math.log(1.0 / self.alpha)
        if self.boundary == 'wald':
            return math.log((1.0 - self.beta) / self.alpha)
        raise ValueError(self.boundary)

    @property
    def lower(self):
        """포기 경계 B (futility=False 면 −∞)."""
        if not self.futility:
            return -math.inf
        return math.log(self.beta / (1.0 - self.alpha))

    @property
    def step_ok(self):
        """무충돌 에피소드 증분 s."""
        return math.log((1.0 - self.p1) / (1.0 - self.p0))

    @property
    def step_fail(self):
        """충돌 에피소드 증분 f (음수)."""
        return math.log(self.p1 / self.p0)

    @property
    def n_zero(self):
        """0 충돌로 NI 를 선언하는 에피소드 수 ⌈A/s⌉."""
        return int(math.ceil(self.upper / self.step_ok - 1e-12))

    @property
    def cost_per_failure(self):
        """충돌 1 회가 상쇄하는 무충돌 에피소드 수 (s − f)/s."""
        return (self.step_ok - self.step_fail) / self.step_ok


@dataclass
class SPRTState:
    """감시 상태 — 에피소드 결과를 하나씩 넣는다."""

    design: SPRTDesign
    n: int = 0
    failures: int = 0
    llr: float = 0.0
    decision: str = 'continue'   # continue | ni | futile | inconclusive
    history: list = field(default_factory=list)

    def update(self, failed):
        """에피소드 하나 (failed = 접촉 여부). 결정이 이미 났으면 무시하고 결정을 돌려준다."""
        if self.decision != 'continue':
            return self.decision
        d = self.design
        self.n += 1
        if failed:
            self.failures += 1
            self.llr += d.step_fail
        else:
            self.llr += d.step_ok
        self.history.append(self.llr)
        if self.llr >= d.upper:
            self.decision = 'ni'
        elif self.llr <= d.lower:
            self.decision = 'futile'
        elif self.n >= d.n_max:
            self.decision = 'inconclusive'
        return self.decision


def run_sprt(outcomes, design=None):
    """0/1 접촉 열(실행 순서대로)을 감시 → SPRTState."""
    st = SPRTState(design or SPRTDesign())
    for o in outcomes:
        if st.update(bool(o)) != 'continue':
            break
    return st


def simulate_oc(design, p_true, reps=40000, seed=0):
    """
    운용 특성(OC) 모의 — (P(NI 선언), P(포기), P(미결), E[N], N 의 90 % 분위).

    calc6.py 의 sim() 과 같은 벡터화 방법 (에피소드 독립, 상한 n_max 에서 절단).
    """
    rng = np.random.default_rng(seed)
    n = design.n_max
    x = rng.random((reps, n)) < p_true
    llr = np.cumsum(np.where(x, design.step_fail, design.step_ok), axis=1)
    hit_a = llr >= design.upper
    hit_b = llr <= design.lower
    t_a = np.where(hit_a.any(1), hit_a.argmax(1) + 1, n + 1)
    t_b = np.where(hit_b.any(1), hit_b.argmax(1) + 1, n + 1)
    t = np.minimum(np.minimum(t_a, t_b), n)
    ni = (t_a < t_b) & (t_a <= n)
    fut = (t_b < t_a) & (t_b <= n)
    return {
        'p_ni': float(ni.mean()), 'p_futile': float(fut.mean()),
        'p_inconclusive': float((~(ni | fut)).mean()),
        'expected_n': float(t.mean()), 'n90': float(np.percentile(t, 90)),
    }


def success_run_n(p0, alpha):
    """0 충돌 e-process 종료 시점 = 고정 표본 zero-failure 검정 ⌈ln α / ln(1 − p0)⌉."""
    return int(math.ceil(math.log(alpha) / math.log(1.0 - p0) - 1e-12))


def success_run_e(n, failures, p0):
    """success-run e-value E_n = (1 − p0)^(−n)·1{사건 0}."""
    return 0.0 if failures else (1.0 - p0) ** (-n)
