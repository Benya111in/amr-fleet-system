"""
투표 코어 상태기계 속성 시험 (Hypothesis RuleBasedStateMachine).

규칙: 요청 · 배달(임의 순서 = 재정렬 · 지연) · 손실 · 중복 · 로봇별 시계 진행 · 시계 리셋(epoch) · tick ·
크래시 · 재시작 · 멤버십 관측 어긋남 · 진입 · 진출 · 포기. 매 단계 Cluster.check() 의 불변식
(I1 허가 배타 · I2 물리 배타 · I3 점유 ⊆ 허가 · I4 허가 ⇒ 과반 확정 · I5 낮은 epoch 수용 0)을 확인한다.

예제 수 · 단계 수는 환경 변수로 키운다 (CI 기본은 작게, 결정적):
  AMR_DIST_HYP_EXAMPLES (기본 400), AMR_DIST_HYP_STEPS (기본 80),
  AMR_DIST_HYP_SEED=1 이면 무작위 탐색 (시드는 pytest --hypothesis-seed=N), 없으면 derandomize
대조: 보고서가 기각한 "생존 멤버 전원 REPLY" 변형(RaLiveAgent, test_dist_rejected_variant.py)은
같은 하네스에서 반례가 나와야 한다. 축소 반례를 고정한 결정적 시험은 그 파일에 있다 (hypothesis 없이도 돈다).
"""

import collections
import json
import os
import random

from amr_fleet.distributed.simnet import Cluster
from amr_fleet.distributed.vote_core import VoteConfig, ZoneAgent
import pytest
from test_dist_rejected_variant import RaLiveAgent

# 모듈 수준 pytest.importorskip 은 쓰지 않는다 — 이미지의 launch_testing pytest 플러그인이 수집 단계에서
# 디렉터리의 모든 모듈을 import 하므로 Skipped 하나가 디렉터리 전체 수집을 멈춘다 (test_task_msg.py 주석,
# 이 파일에서도 실측: hypothesis 를 막자 test_dist_ordering.py 만 골라 돌려도 0 개 수집).
# 그래서 import 를 감싸고, hypothesis 가 없으면 장식자를 빈 껍데기로 바꾼 뒤 pytestmark 로 건너뛴다.
try:
    from hypothesis import HealthCheck, settings, strategies as st
    from hypothesis.stateful import (
        invariant, precondition, rule, RuleBasedStateMachine, run_state_machine_as_test,
    )
    HAVE_HYPOTHESIS = True
except ImportError:                                   # CI 이미지에 python3-hypothesis 가 없을 때
    HAVE_HYPOTHESIS = False

    class _Stub:
        def __getattr__(self, name):
            return lambda *a, **k: None

    st = _Stub()
    HealthCheck = settings = run_state_machine_as_test = None
    RuleBasedStateMachine = object

    def _passthrough(*a, **k):
        return lambda f: f

    invariant = precondition = rule = _passthrough

pytestmark = pytest.mark.skipif(not HAVE_HYPOTHESIS,
                                reason='hypothesis 없음 (package.xml test_depend 요청)')

MEMBERS = ('r1', 'r2', 'r3', 'r4', 'r5')
CHAINS = (('zone:a',), ('zone:b',), ('zone:a', 'zone:b'), ('pocket:p', 'zone:b'))
RESOURCES = ('zone:a', 'zone:b', 'pocket:p')
DTS = (0.25, 1.0, 3.0, 10.0)
SKEW_MAX = 12.0          # 로봇 간 시계 어긋남 상한 [s] (잠정 기한 4 s 의 3 배 — 만료 순서 뒤집힘은 충분히 생긴다)
CFG = VoteConfig(MEMBERS, tentative_s=4.0, retry_s=0.5, backoff_min_s=0.2, backoff_max_s=1.0,
                 tombstone_cap=8)

EXAMPLES = int(os.environ.get('AMR_DIST_HYP_EXAMPLES', '400'))
STEPS = int(os.environ.get('AMR_DIST_HYP_STEPS', '80'))
SEED = os.environ.get('AMR_DIST_HYP_SEED')
SKIP_I4 = os.environ.get('AMR_DIST_SKIP_I4') == '1'   # 변이 검사: 핵심 불변식만으로 잡히나
STATS_PATH = os.environ.get('AMR_DIST_HYP_STATS')     # 주면 실행 통계를 JSON 으로 쓴다
STATS = collections.Counter()


class MutexViolation(AssertionError):
    """불변식 위반 (반례 trace 를 메시지에 싣는다)."""


def hyp_settings(examples=None, steps=None):
    """CI 기본: 결정적(derandomize). AMR_DIST_HYP_SEED 를 주면 그 시드로 무작위 탐색."""
    kw = dict(max_examples=examples or EXAMPLES, stateful_step_count=steps or STEPS,
              deadline=None, database=None, print_blob=False,
              suppress_health_check=list(HealthCheck))
    if SEED is None:
        kw['derandomize'] = True
    return settings(**kw)


robots = st.sampled_from(MEMBERS)


class VoteMachine(RuleBasedStateMachine):
    """정적 과반 배타 투표 + 2단계 진입 (ZoneAgent)."""

    BOOT = staticmethod(ZoneAgent.boot)
    STRICT = True
    CRASH = True
    MAX_CRASHES = 3          # 실행당 상한 — 흔들기만 하면 진행이 없어 허가 상태에 거의 닿지 못한다
    MAX_CLOCK_RESETS = 2

    def __init__(self):
        super().__init__()
        self.c = Cluster(CFG, seed=0, boot=type(self).BOOT, strict=type(self).STRICT,
                         check_i4=not SKIP_I4)
        STATS[f'{type(self).__name__}.runs'] += 1
        self.crashes = 0
        self.resets = 0

    def _count(self, what):
        STATS[f'{type(self).__name__}.{what}'] += 1

    # ---- 로봇 동작
    @rule(r=robots, chain=st.sampled_from(CHAINS), prio=st.integers(0, 2))
    def request(self, r, chain, prio):
        self._count('request')
        self.c.request(r, chain, prio)

    @rule(rs=st.lists(robots, min_size=2, max_size=5, unique=True),
          chain=st.sampled_from(CHAINS))
    def contend(self, rs, chain):
        """여러 로봇이 같은 사슬을 동시에 요청 (경합)."""
        self._count('contend')
        for r in rs:
            self.c.request(r, chain)

    @rule(seed=st.integers(0, 2 ** 16), loss=st.sampled_from((0.0, 0.1, 0.3)),
          dup=st.sampled_from((0.0, 0.1)), hold=st.sampled_from((0.0, 0.2, 0.5)),
          go_in=st.booleans(), go_out=st.booleans(), dt=st.sampled_from(DTS))
    def chaos_round(self, seed, loss, dup, hold, go_in, go_out, dt):
        """
        한 라운드를 무작위 순서 · 손실 · 중복 · 지연(다음 라운드로 미룸)으로 돌린다.

        개별 deliver(i) 규칙만으로는 표가 갈리는 끼어들기 순서에 거의 닿지 못한다 (REPORT 변이 1 차 결과).
        """
        self._count('chaos_round')
        rng = random.Random(seed)
        self.c.chaos(rng, loss, dup, hold)
        self.c.tick_all()
        self.c.chaos(rng, loss, dup, hold)
        if go_in:
            self.enter_all()
        if go_out:
            for r in MEMBERS:
                self.exit_one(r)
        self.c.advance_all(dt)

    @rule(r=robots)
    def enter(self, r):
        if self.c.enter(r):
            self._count('enter')
            if sum(1 for x in MEMBERS if self.c.inside[x]) >= 2:
                self._count('enter_while_other_inside')

    @rule()
    def enter_all(self):
        for r in MEMBERS:
            self.enter(r)

    @rule(r=robots)
    def exit_one(self, r):
        if self.c.inside[r]:
            self._count('exit')
            self.c.exit(r, sorted(self.c.inside[r])[0])

    @rule(r=robots, z=st.sampled_from(RESOURCES))
    def exit(self, r, z):
        self.c.exit(r, z)

    @rule(r=robots, z=st.sampled_from(RESOURCES))
    def give_up(self, r, z):
        self.c.give_up(r, z)

    @rule(r=robots, view=st.sets(robots, min_size=2, max_size=4))
    def set_view(self, r, view):
        self.c.set_view(r, view)

    @rule(r=robots)
    def clear_view(self, r):
        self.c.set_view(r, MEMBERS)

    # ---- 네트워크
    @precondition(lambda self: self.c.net)
    @rule(data=st.data())
    def deliver(self, data):
        self.c.deliver(data.draw(st.integers(0, len(self.c.net) - 1), label='i'))

    @precondition(lambda self: self.c.net)
    @rule(data=st.data())
    def drop(self, data):
        self.c.drop(data.draw(st.integers(0, len(self.c.net) - 1), label='i'))

    @precondition(lambda self: self.c.net and len(self.c.net) < 400)
    @rule(data=st.data())
    def duplicate(self, data):
        self.c.duplicate(data.draw(st.integers(0, len(self.c.net) - 1), label='i'))

    @precondition(lambda self: self.c.net)
    @rule()
    def deliver_all(self):
        self.c.deliver_all(max_msgs=300)

    @rule(dt=st.sampled_from(DTS), go_in=st.booleans(), go_out=st.booleans())
    def protocol_round(self, dt, go_in, go_out):
        """
        손실 없는 한 라운드: 배달 → tick → 배달 → (허가받은 로봇 진입) → (안에 있는 로봇 하나씩 진출) → 시계 dt.

        로봇은 허가를 받으면 곧 들어가고 언젠가 나온다 — 그 흐름을 자주 만들어야 위반이 날 수 있는
        상태(누가 안에 있는데 다른 로봇이 같은 자원을 요청)에 닿는다.
        """
        self._count('round')
        self.c.deliver_all(max_msgs=300)
        self.c.tick_all()
        self.c.deliver_all(max_msgs=300)
        if go_in:
            self.enter_all()
        if go_out:
            for r in MEMBERS:
                self.exit_one(r)
        self.c.advance_all(dt)

    # ---- 시계 · 타이머
    @precondition(lambda self: max(self.c.clock.values()) - min(self.c.clock.values()) < SKEW_MAX)
    @rule(r=robots, dt=st.sampled_from(DTS))
    def advance(self, r, dt):
        """로봇 하나의 시계만 진행 (시계 어긋남). 어긋남 상한 SKEW_MAX — 넘으면 서로의 기한이 늘 지나 있다."""
        self.c.advance(r, dt)

    @rule(dt=st.sampled_from(DTS))
    def advance_all(self, dt):
        self.c.advance_all(dt)

    @rule(r=robots)
    def tick(self, r):
        self.c.tick(r)

    @rule()
    def tick_all(self):
        self.c.tick_all()

    @precondition(lambda self: self.resets < type(self).MAX_CLOCK_RESETS)
    @rule(r=robots)
    def reset_clock(self, r):
        if self.c.alive(r):
            self._count('clock_reset')
            self.resets += 1
        self.c.reset_clock(r, 0.0)

    # ---- 크래시
    @precondition(lambda self: type(self).CRASH and self.crashes < type(self).MAX_CRASHES)
    @rule(r=robots)
    def crash(self, r):
        if self.c.alive(r):
            self.crashes += 1
            self._count('crash')
            if self.c.inside[r] or self.c.net:
                self._count('crash_mid_protocol')
        self.c.crash(r)

    @precondition(lambda self: type(self).CRASH)
    @rule(r=robots)
    def restart(self, r):
        self.c.restart(r)

    @precondition(lambda self: type(self).CRASH and self.crashes < type(self).MAX_CRASHES)
    @rule(r=robots)
    def crash_corrupt(self, r):
        """
        크래시 + 저장소 손상. 재시작하면 기억 상실 투표자(영구 기권)가 된다.

        몸체가 자원 안에 있거나 허가를 쥔 로봇은 고르지 않는다 — 그 로봇은 자기 허가를 잊으므로 I3 가
        정의상 깨진다 (남의 확정 표가 남아 다른 로봇은 여전히 못 들어간다. REPORT 범위 밖 절 참고).
        """
        a = self.c.agents.get(r)
        if a is None or self.c.inside[r] or a.authorized() or a.attempt is not None:
            return
        self.crashes += 1
        self._count('crash_corrupt')
        self.c.crash(r)
        self.c.stores[r].corrupt()

    # ---- 불변식
    @invariant()
    def safety(self):
        self._count('steps')
        auth = {}
        for r in MEMBERS:
            if self.c.alive(r):
                for z in self.c.agents[r].authorized():
                    auth[z] = r
        if auth:
            self._count('steps_with_authorized')
            for r in MEMBERS:
                a = self.c.agents.get(r)
                att = getattr(a, 'attempt', None) if a is not None else None
                res = getattr(att, 'resources', None) or getattr(a, 'req_res', frozenset())
                if any(auth.get(z) not in (None, r) for z in res):
                    self._count('steps_with_contention')   # 남이 허가받은 자원을 요청 중
                    break
        bad = self.c.check()
        if bad:
            tail = '\n'.join(self.c.trace[-60:])
            raise MutexViolation('; '.join(bad) + '\n--- trace (마지막 60) ---\n' + tail)


class RaLiveMachine(VoteMachine):
    """같은 규칙 · 같은 불변식 (I1–I3, I5), 기각 변형."""

    BOOT = staticmethod(RaLiveAgent.boot)
    STRICT = False
    CRASH = False


# ---------------------------------------------------------------------- 시험
def _dump_stats():
    if STATS_PATH:
        with open(STATS_PATH, 'w') as f:
            json.dump(dict(STATS, examples_setting=EXAMPLES, steps_setting=STEPS,
                           seed_mode='random' if SEED else 'derandomize'), f, indent=1,
                      sort_keys=True)


def test_vote_core_state_machine():
    """정적 과반 투표: 손실 · 중복 · 재정렬 · 크래시 재시작 · 시계 · 멤버십 어긋남에서 불변식 위반 0."""
    try:
        run_state_machine_as_test(VoteMachine, settings=hyp_settings())
    finally:
        _dump_stats()


def test_rejected_live_member_variant_has_counterexample():
    """대조: 기각 변형은 같은 하네스에서 반례가 나와야 한다 (시험이 공허하지 않다는 증거)."""
    try:
        with pytest.raises(MutexViolation) as ei:
            # 반례는 생성 예제 100 여 개째에 처음 나온다 (REPORT 측정) — 예산은 넉넉히 2000
            run_state_machine_as_test(RaLiveMachine,
                                      settings=hyp_settings(examples=max(EXAMPLES, 2000)))
    finally:
        _dump_stats()
    assert 'I1' in str(ei.value) or 'I2' in str(ei.value)
