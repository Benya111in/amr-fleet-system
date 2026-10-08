"""
협상 낙찰 → 실제 실행기 (b4-wire, p3.md §10.2) — 코어 + loopback, 실행기는 시험이 흉내 낸다.

가짜 실행기(SimExecutor) 없이: CBAA 낙찰(잠정 과반)이면 dispatch 로 넘기고, 실행기 응답 · task_status 로 끝을 받는다.
- 실행기가 받고 확정(COMMIT_TASK → HELD) 뒤 완료 → 끝난 작업 (다시 잡히지 않는다), 같은 작업 확정 2 번 이상 0.
- 거부(busy) → 잠정 잠금을 놓고 막지 않는다 (다시 입찰), 거부(invalid) → 막는다.
- 픽업 전 실패 → 잠금을 놓아 다른 로봇이 잡는다. 실행기가 IDLE 이 아니거나 중앙 작업 중이면 입찰하지 않는다.
"""

import collections

import pytest

from amr_fleet.agent.core import (
    BoardTask, CoreConfig, OP_COMMIT_TASK, STATUS_COMPLETED, STATUS_FAILED,
)
from amr_fleet.agent.loopback import LoopbackFleet
from amr_fleet.agent.modes import DEGRADED, ModeConfig
from amr_fleet.distributed.vote_core import VoteConfig

ROBOTS = ('amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05')
POSES = {'amr_01': (0.0, 0.0), 'amr_02': (5.0, 0.0), 'amr_03': (10.0, 0.0),
         'amr_04': (0.0, 5.0), 'amr_05': (5.0, 5.0)}
VOTE = VoteConfig(ROBOTS, tentative_s=10.0, retry_s=0.5, backoff_min_s=0.3, backoff_max_s=1.5)


def _degraded_fleet(gc=True):
    cfgs = [CoreConfig(robot_id=r, members=ROBOTS, zone_vote=VOTE, task_vote=VOTE,
                       modes=ModeConfig(resync_s=3.0, isolate_s=1.0, boot_grace_s=2.0),
                       stale_lock_gc=gc)
            for r in ROBOTS]
    f = LoopbackFleet(cfgs, poses=POSES)
    f.server().alive = False                     # 서버 없음 → unknown_dead_s 뒤 DEGRADED
    f.run(8.0)
    assert set(f.modes().values()) == {DEGRADED}
    for c in f.cores.values():
        c.set_executor_phase('IDLE')
    return f


class FakeExecutors:
    """실제 실행기 흉내: 받으면 commit_after 뒤 COMMIT_TASK, HELD 면 pickup_s 뒤 완료, denied 면 실패."""

    def __init__(self, f, reject=None, commit_after=1.0, pickup_s=1.0):
        self.f = f
        self.reject = dict(reject or {})          # 로봇 → 거부 사유
        self.commit_after, self.pickup_s = commit_after, pickup_s
        self.jobs = {}                            # 로봇 → [task, 받은 t, 확정 시작 t]
        self.seen = 0
        self.pickups = collections.defaultdict(list)    # 작업 → [로봇]
        self.done = []

    def step(self, f):
        for r, task, _ in f.dispatched[self.seen:]:
            core = f.cores[r]
            why = self.reject.get(r)
            if why or r in self.jobs:
                core.on_negotiated_result(task, 'rejected', why or 'busy:MOVING', f.t)
            else:
                core.on_negotiated_result(task, 'accepted', 'ok', f.t)
                self.jobs[r] = [task, f.t, None]
                core.set_executor_phase('MOVING')
        self.seen = len(f.dispatched)
        for r, job in list(self.jobs.items()):
            core = f.cores[r]
            task, t0, tp = job
            if tp is None and f.t - t0 >= self.commit_after:
                core.command(OP_COMMIT_TASK, [task], 0, f.t)
                if core.lock.is_held(task):
                    job[2] = f.t
                    self.pickups[task].append(r)
                elif core.lock.is_denied(task):
                    self._end(core, r, task, STATUS_FAILED)
            elif tp is not None and f.t - tp >= self.pickup_s:
                self.done.append(task)
                self._end(core, r, task, STATUS_COMPLETED)

    def _end(self, core, r, task, status):
        core.on_task_status(task, status)
        core.set_executor_phase('IDLE')
        del self.jobs[r]


def _tasks(n, x0=20.0):
    return [BoardTask(f'd{i}', 0, x0 + 2.0 * i, 0.0) for i in range(n)]


def test_negotiated_tasks_reach_the_executor_and_complete_once():
    f = _degraded_fleet()
    ex = FakeExecutors(f)
    f.post(_tasks(6))
    f.run(60.0, hook=ex.step)
    assert sorted(set(ex.done)) == [f'd{i}' for i in range(6)], (ex.done, f.dispatched)
    assert all(len(rs) == 1 for rs in ex.pickups.values()), dict(ex.pickups)
    assert len(ex.done) == len(set(ex.done))
    assert all(c.negotiated == '' for c in f.cores.values())
    # 끝난 작업은 확정 잠금을 그대로 쥔다 (완료 뒤 해제 안 함) — 다시 넘기지 않는다
    assert collections.Counter(t for _, t, _ in f.dispatched).most_common(1)[0][1] >= 1


def test_busy_rejection_releases_without_blocking_and_invalid_blocks():
    f = _degraded_fleet()
    ex = FakeExecutors(f, reject={'amr_01': 'busy:RETURNING', 'amr_02': 'invalid:no_dock:x'})
    f.post(_tasks(1, x0=0.0))                    # amr_01 쪽 (가장 가깝다)
    f.run(4.0, hook=ex.step)
    rej = [e for e in f.events if e['event'] == 'nego_rejected']
    assert rej and rej[0]['robot'] == 'amr_01'
    c1 = f.cores['amr_01']
    assert 'd0' not in c1.cbaa.blocked           # busy 는 막지 않는다
    assert c1.lock.phase('d0') in ('idle', 'pending', 'voted')
    ex.reject.pop('amr_01')
    f.run(30.0, hook=ex.step)
    assert ex.done == ['d0']
    for e in rej:
        if e['robot'] == 'amr_02':
            assert 'd0' in f.cores['amr_02'].cbaa.blocked


@pytest.mark.parametrize('gc', [True, False])
def test_failure_before_pickup_lets_another_robot_take_it(gc):
    """
    픽업 전 실패한 로봇의 LOCKED 를 남이 지워야 다시 잡힌다 (stale_lock_gc).

    끄면(P3 동작) 그 작업은 DEGRADED 동안 끝나지 않는다 — CBAA 는 잠정 과반에서 LOCKED 를 알리고 남은 그것을 못
    지운다. 그 결함을 고정해 두는 대조군이다.
    """
    f = _degraded_fleet(gc)
    ex = FakeExecutors(f, commit_after=3.0)
    f.post(_tasks(1, x0=0.0))
    first = {}

    def hook(fl):
        if fl.dispatched and not first:
            r, task, _ = fl.dispatched[0]
            first['r'] = r
            core = fl.cores[r]
            core.on_negotiated_result(task, 'accepted', 'ok', fl.t)
            core.on_task_status(task, STATUS_FAILED)         # 주행 실패 (확정 전)
            ex.seen = 1
            return
        ex.step(fl)
    f.run(40.0, hook=hook)
    assert any(e['event'] == 'nego_done' and not e['held'] for e in f.events)
    if not gc:
        assert ex.done == [] and not ex.pickups            # 아무도 다시 못 잡는다 (P3 결함)
        return
    assert ex.done == ['d0']
    assert len(ex.pickups['d0']) == 1 and ex.pickups['d0'][0] != first['r']


def test_busy_executor_does_not_bid():
    f = _degraded_fleet()
    for r in ROBOTS:
        f.cores[r].set_executor_phase('MOVING')           # 모두 바쁘다
    f.cores['amr_03'].central_task = 'c1'
    f.post(_tasks(3))
    f.run(10.0)
    assert f.dispatched == []
    for r in ROBOTS:
        if r != 'amr_03':
            f.cores[r].set_executor_phase('IDLE')
    f.run(20.0)
    assert {r for r, _, _ in f.dispatched} <= set(ROBOTS) - {'amr_03'}
    assert f.dispatched
