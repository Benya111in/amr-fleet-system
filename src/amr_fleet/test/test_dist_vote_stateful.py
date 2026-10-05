"""
투표 코어 상태기계 속성 시험 (Hypothesis RuleBasedStateMachine).

규칙: 요청 · 배달(임의 순서 = 재정렬 · 지연) · 손실 · 중복 · 로봇별 시계 진행 · 시계 리셋(epoch) · tick ·
크래시 · 재시작 · 멤버십 관측 어긋남 · 진입 · 진출 · 포기. 매 단계 Cluster.check() 의 불변식
(I1 허가 배타 · I2 물리 배타 · I3 점유 ⊆ 허가 · I4 허가 ⇒ 과반 확정 · I5 낮은 epoch 수용 0)을 확인한다.

예제 수 · 단계 수는 환경 변수로 키운다 (CI 기본은 작게, 결정적):
  AMR_DIST_HYP_EXAMPLES (기본 150), AMR_DIST_HYP_STEPS (기본 60), AMR_DIST_HYP_SEED (없으면 derandomize)
대조: 보고서가 기각한 "생존 멤버 전원 REPLY" 변형(RaLiveAgent)은 같은 하네스에서 반례가 나와야 한다.
"""

import collections
import json
import os

import pytest

hypothesis = pytest.importorskip('hypothesis')

from hypothesis import HealthCheck, settings, strategies as st  # noqa: E402
from hypothesis.stateful import (  # noqa: E402
    invariant, precondition, rule, RuleBasedStateMachine, run_state_machine_as_test,
)

from amr_fleet.distributed import messages as m  # noqa: E402
from amr_fleet.distributed.messages import VoteMsg  # noqa: E402
from amr_fleet.distributed.ordering import PriorityKey, ReqId  # noqa: E402
from amr_fleet.distributed.simnet import Cluster  # noqa: E402
from amr_fleet.distributed.vote_core import VoteConfig, ZoneAgent  # noqa: E402

MEMBERS = ('r1', 'r2', 'r3', 'r4', 'r5')
CHAINS = (('zone:a',), ('zone:b',), ('zone:a', 'zone:b'), ('pocket:p', 'zone:b'))
RESOURCES = ('zone:a', 'zone:b', 'pocket:p')
DTS = (0.25, 1.0, 3.0, 10.0)
SKEW_MAX = 12.0          # 로봇 간 시계 어긋남 상한 [s] (잠정 기한 4 s 의 3 배 — 만료 순서 뒤집힘은 충분히 생긴다)
CFG = VoteConfig(MEMBERS, tentative_s=4.0, retry_s=0.5, backoff_min_s=0.2, backoff_max_s=1.0,
                 tombstone_cap=8)

EXAMPLES = int(os.environ.get('AMR_DIST_HYP_EXAMPLES', '150'))
STEPS = int(os.environ.get('AMR_DIST_HYP_STEPS', '60'))
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


# ---------------------------------------------------------------------- 기각 변형 (시험 전용)
class RaLiveAgent:
    """
    보고서가 기각한 변형 — "현재 생존 멤버 전원의 REPLY 를 받고 그 멤버가 3 대 이상이면 진입".

    Ricart–Agrawala 응답 규칙(자기가 안에 있거나, 자기 요청이 더 우선이면 미룸)에 동적 정족수(각자 관측한
    생존 멤버)를 붙인 것. 관측이 어긋나면({A,B,C} 대 {C,D,E}) C 가 둘 다에 REPLY 해 둘이 함께 들어간다.
    ZoneAgent 와 같은 인터페이스(boot/handle/tick/request/release/authorized …)라 같은 하네스에서 돈다.
    """

    def __init__(self, me, cfg):
        self.me = me
        self.cfg = cfg
        self.view = set(cfg.members)
        self.max_epoch = 0
        self.lamport = 0
        self.seq = 0
        self.req = None
        self.req_res = frozenset()
        self.req_ts = None
        self.asked = set()
        self.replies = set()
        self.granted = {}
        self.deferred = []

    @classmethod
    def boot(cls, robot_id, config, store, now, inside=(), rng=None):
        return cls(robot_id, config), []

    held = None          # Cluster 가 authorized() 를 쓰게 한다

    def set_view(self, view):
        self.view = set(view) | {self.me}

    def authorized(self):
        out = set()
        for z in self.granted.values():
            out |= z
        return out

    def protocol_state(self):
        return {}

    def on_time_jump(self, now):
        return []

    def tick(self, now):
        return []

    def _msg(self, kind, dst, req, res, key=None):
        self.lamport += 1
        return VoteMsg(kind, self.me, dst, 0, req, frozenset(res), key, None, self.lamport)

    def request(self, resources, now, priority=0, task_deadline=None):
        self.seq += 1
        self.req = ReqId(self.me, 1, self.seq)
        self.req_res = frozenset(resources) - self.authorized()
        self.lamport += 1
        self.req_ts = self.lamport
        self.asked = set(self.view) | {self.me}
        self.replies = set()
        if not self.req_res:
            self.req = None
            return []
        key = PriorityKey(0, None, self.req_ts)
        return [self._msg(m.REQUEST, v, self.req, self.req_res, key) for v in sorted(self.asked)]

    def release(self, resources, now):
        want = set(resources)
        for req in list(self.granted):
            self.granted[req] -= want
            if not self.granted[req]:
                del self.granted[req]
        return self._flush_deferred()

    def _blocks(self, src, res, ts):
        if any(z & res for z in self.granted.values()):
            return True
        if self.req is not None and self.req_res & res and src != self.me:
            return (self.req_ts, self.me) < (ts, src)
        return False

    def _flush_deferred(self):
        out, keep = [], []
        for (src, req, res, ts) in self.deferred:
            if self._blocks(src, res, ts):
                keep.append((src, req, res, ts))
            else:
                out.append(self._msg(m.VOTE, src, req, res))
        self.deferred = keep
        return out

    def handle(self, msg, now):
        self.lamport = max(self.lamport, msg.lamport) + 1
        if msg.kind == m.REQUEST:
            ts = msg.key.since if msg.key else msg.lamport
            if self._blocks(msg.src, msg.resources, ts):
                self.deferred.append((msg.src, msg.req, msg.resources, ts))
                return []
            return [self._msg(m.VOTE, msg.src, msg.req, msg.resources)]
        if msg.kind == m.VOTE and msg.req == self.req:
            self.replies.add(msg.src)
            if self.replies >= self.asked and len(self.asked) >= 3:
                self.granted[self.req] = set(self.req_res)
                self.req = None
                return self._flush_deferred()
        return []


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


def _ra_counterexample_ops(c):
    """축소 반례 고정 (hypothesis 가 찾은 꼴을 손으로 옮김): A={r1,r2,r3}, E={r3,r4,r5} 관측."""
    c.set_view('r1', ['r2', 'r3'])
    c.set_view('r5', ['r3', 'r4'])
    c.request('r1', ['zone:a'])
    c.request('r5', ['zone:a'])
    c.deliver_all()
    c.enter('r1')
    c.enter('r5')


def test_rejected_variant_fixed_counterexample():
    """보고서 반례 (A·E 가 C 의 REPLY 를 함께 받아 동시 진입) — 기각 변형은 위반, 투표 코어는 통과."""
    ra = Cluster(CFG, boot=RaLiveAgent.boot, strict=False)
    _ra_counterexample_ops(ra)
    bad = ra.check()
    assert any(b.startswith('I1') for b in bad), bad
    assert sorted(ra.inside['r1'] | ra.inside['r5']) == ['zone:a']
    good = Cluster(CFG)
    _ra_counterexample_ops(good)
    good.settle(20)
    good.enter('r1')
    good.enter('r5')
    assert good.check() == []
    assert len([r for r in MEMBERS if 'zone:a' in good.inside[r]]) <= 1
