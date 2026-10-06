"""
기각된 변형 "생존 멤버 전원 REPLY" (시험 전용) 과 그 반례를 고정한 결정적 시험.

보고서(「분산 플릿 관리 협상 할당과 폴백」 76–80 행)가 기각한 규칙을 ZoneAgent 와 같은 인터페이스로 구현해
같은 모의 플릿(simnet.Cluster)에 올린다. Hypothesis 상태기계(test_dist_vote_stateful.py)는 이 변형에서 반례를
찾아야 하고 (시험의 힘 대조), 찾은 축소 반례는 여기서 hypothesis 없이 결정적으로 재현한다.
"""

from amr_fleet.distributed import messages as m
from amr_fleet.distributed.messages import VoteMsg
from amr_fleet.distributed.ordering import PriorityKey, ReqId
from amr_fleet.distributed.simnet import Cluster
from amr_fleet.distributed.vote_core import VoteConfig

MEMBERS = ('r1', 'r2', 'r3', 'r4', 'r5')
CFG = VoteConfig(MEMBERS, tentative_s=4.0, retry_s=0.5, backoff_min_s=0.2, backoff_max_s=1.0,
                 tombstone_cap=8)


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


def _ra_shrunk_one_sided_ops(c):
    """Hypothesis 가 축소한 반례 (시드 0·1 공통 꼴): 한 로봇만 관측이 어긋나도 충분하다."""
    c.request('r1', ['zone:a'])                  # r1 은 5 대 전원에게 묻는다
    c.set_view('r3', ['r2', 'r4'])               # r3 은 r1 을 못 본다고 믿는다
    c.request('r3', ['zone:a'])
    c.deliver_all()
    c.enter('r1')
    c.enter('r3')


def test_rejected_variant_shrunk_one_sided_counterexample():
    """
    축소 반례 고정: r3 이 r1 에게 묻지 않으니 r1 은 미룰 기회가 없고, r2·r4 는 둘 다에 REPLY 한다.

    보고서의 대칭 반례({A,B,C} 대 {C,D,E})보다 약한 조건 — 한쪽 관측 오류 하나로 두 대가 들어간다.
    같은 순서에서 투표 코어는 r1 만 허가한다.
    """
    ra = Cluster(CFG, boot=RaLiveAgent.boot, strict=False)
    _ra_shrunk_one_sided_ops(ra)
    assert any(b.startswith('I1') for b in ra.check())
    assert 'zone:a' in ra.inside['r1'] and 'zone:a' in ra.inside['r3']
    good = Cluster(CFG)
    _ra_shrunk_one_sided_ops(good)
    good.settle(20)
    good.enter('r1')
    good.enter('r3')
    assert good.check() == []
    assert good.agents['r1'].authorized() == {'zone:a'} and not good.agents['r3'].authorized()
