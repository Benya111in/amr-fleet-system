"""
경로 게이트 (agent/path_gate.py) 와 그 코어 결합 — b4-wire (p3.md §10.1).

순수 결정: 사전 요청 거리 · 결정점(요청 거리) 안 blocking · 사슬 전부-아니면-전무 · 구역 안에서 다음 구역을 기다리지
않음 · 도착 · 멈추는 단계 · 자세 모름.
코어 + 운동학 이동 (loopback 5 대, 2 대가 움직인다 — 감속 1.0 m/s² 로만 선다): 실제 몸체 기준 동시 점유 0, 둘 다
도착, 혼자면 한 번도 서지 않음(사전 요청), 지나온 구역 해제, ISOLATED 면 결정점에서 섬, 멈추는 단계면 요청 취소.
"""

import math

import numpy as np
import pytest

from amr_fleet.agent.core import CoreConfig, disc_clear_of_polygon, GATE_ZONE_MAP
from amr_fleet.agent.loopback import LoopbackFleet
from amr_fleet.agent.modes import ISOLATED, ModeConfig
from amr_fleet.agent.path_gate import (
    PathGate, PathGateConfig, remaining_path, UNKNOWN_POSE,
)
from amr_fleet.comm_emulator import CommEmulator, LinkModel, Partition
from amr_fleet.distributed.vote_core import VoteConfig
from amr_fleet.traffic_zones import zones_from_config

ROBOTS = ('amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05')
SQ = [[4.0, -1.0], [6.0, -1.0], [6.0, 1.0], [4.0, 1.0]]


def _zones(*extra):
    entries = [dict(id='z', kind='intersection', polygon=SQ)] + list(extra)
    return zones_from_config(entries)


def _gate(*extra, prerequest=1.0, approach=2.0):
    zs = _zones(*extra)
    return PathGate(zs, {z.zone_id: approach for z in zs}, PathGateConfig(prerequest_m=prerequest))


H = np.array([[0.0, 0.0], [12.0, 0.0]])          # 가로로 z 를 지난다 (입구 x = 4)


# ---------------------------------------------------------------------- 순수 결정
def test_prerequest_then_block_inside_decision_distance():
    g = _gate()
    assert g.decide((0.0, 0.0, 0.0), H, set()).request == ()          # 입구 4 m > 2 + 1
    d = g.decide((1.2, 0.0, 0.0), H, set())                            # 입구 2.8 m: 사전 요청, 아직 안 섬
    assert d.request == ('zone:z',) and d.blocking == ()
    assert d.first_entry_m == pytest.approx(2.8, abs=0.11)
    d = g.decide((2.5, 0.0, 0.0), H, set())                            # 1.5 m ≤ 2.0: 허가 없으면 선다
    assert d.blocking == ('zone:z',)
    assert g.decide((2.5, 0.0, 0.0), H, {'zone:z'}).blocking == ()     # 허가 있으면 간다


def test_inside_and_after_the_zone():
    g = _gate()
    d = g.decide((5.0, 0.0, 0.0), H, {'zone:z'})
    assert d.request == () and 'zone:z' in d.body and 'zone:z' in d.ahead
    d = g.decide((8.0, 0.0, 0.0), H, {'zone:z'})
    assert d.request == () and not d.body and not d.ahead               # 해제 후보 (코어가 몸체 밖을 확인)


def test_chain_is_all_or_nothing():
    # z 출구에서 1 m (≤ chain_gap 1.5) — 한 사슬
    near = dict(id='w', kind='intersection',
                polygon=[[7.0, -1.0], [8.0, -1.0], [8.0, 1.0], [7.0, 1.0]])
    g = _gate(near)
    d = g.decide((2.5, 0.0, 0.0), H, set())
    assert d.request == ('zone:z', 'zone:w') and d.blocking == ('zone:z', 'zone:w')
    d = g.decide((2.5, 0.0, 0.0), H, {'zone:z'})                       # 첫 구역만 쥐면 그 앞에서 선다
    assert d.blocking == ('zone:w',)


def test_next_unchained_zone_is_not_requested_from_inside_a_zone():
    # z 출구에서 2.6 m (> chain_gap) — 따로
    far = dict(id='f', kind='intersection',
               polygon=[[8.6, -1.0], [9.6, -1.0], [9.6, 1.0], [8.6, 1.0]])
    g = _gate(far)
    d = g.decide((5.5, 0.0, 0.0), H, {'zone:z'})                       # 몸체가 z 안
    assert d.request == ()                                             # 안에서 f 를 기다리지 않는다
    d = g.decide((6.5, 0.0, 0.0), H, {'zone:z'})                       # 몸체가 z 를 나왔다
    assert d.request == ('zone:f',)


def test_arrival_inactive_phase_and_unknown_pose():
    g = _gate()
    short = np.array([[0.0, 0.0], [3.0, 0.0]])
    assert g.decide((2.9, 0.0, 0.0), short, set()).request == ()       # 남은 0.1 m — 도착
    assert g.decide((2.5, 0.0, 0.0), H, set(), active=False) .blocking == ()
    assert g.decide((2.5, 0.0, 0.0), H, set(), active=False).request == ()
    assert g.decide(None, H, set()).blocking == (UNKNOWN_POSE,)        # 판정 불가면 움직이지 않는다
    assert g.decide(None, H, set(), active=False).blocking == ()
    assert g.decide((2.5, 0.0, 0.0), None, set()).blocking == ()       # 경로가 없으면 갈 곳도 없다


def test_remaining_path_projects_the_pose():
    path, rem = remaining_path(H, 3.0, 0.5, 60.0)
    assert rem == pytest.approx(9.0 + 0.5) and path[0].tolist() == [3.0, 0.5]


def test_config_validation():
    with pytest.raises(ValueError):
        PathGateConfig(prerequest_m=-1.0)
    with pytest.raises(ValueError):
        PathGateConfig(step_m=0.0)
    with pytest.raises(ValueError):
        PathGate(_zones(), {})                                         # 요청 거리 없는 구역


# ---------------------------------------------------------------------- 코어 + 운동학 이동
VOTE = VoteConfig(ROBOTS, tentative_s=10.0, retry_s=0.5, backoff_min_s=0.3, backoff_max_s=1.5)
V, ACC, DT = 1.0, 1.0, 0.1


class Mover:
    """경로를 따라 V 로 가고, 게이트가 막으면 ACC 로만 선다 (한 단계 늦게 본다 — BT 반응)."""

    def __init__(self, path):
        self.path = np.asarray(path, dtype=float)
        seg = np.linalg.norm(np.diff(self.path, axis=0), axis=1)
        self.cum = np.concatenate(([0.0], np.cumsum(seg)))
        self.s, self.v, self.blocked, self.stops, self.done = 0.0, 0.0, False, 0, False

    def pose(self):
        x = float(np.interp(self.s, self.cum, self.path[:, 0]))
        y = float(np.interp(self.s, self.cum, self.path[:, 1]))
        k = min(int(np.searchsorted(self.cum, self.s, side='right')) - 1, len(self.path) - 2)
        d = self.path[k + 1] - self.path[k]
        return x, y, math.atan2(d[1], d[0])

    def step(self):
        if self.done:
            return
        want = 0.0 if self.blocked else V
        was_moving = self.v > 0.0
        self.v = min(want, self.v + ACC * DT) if want > self.v else max(want, self.v - ACC * DT)
        if was_moving and self.v == 0.0:
            self.stops += 1
        self.s = min(self.cum[-1], self.s + self.v * DT)
        self.done = self.s >= self.cum[-1] - 1e-9


def _fleet(comm=None):
    cfgs = [CoreConfig(robot_id=r, members=ROBOTS, zone_vote=VOTE, task_vote=VOTE,
                       release_gate=GATE_ZONE_MAP,
                       modes=ModeConfig(resync_s=3.0, isolate_s=1.0, boot_grace_s=2.0))
            for r in ROBOTS]
    poses = {'amr_03': (20.0, 20.0), 'amr_04': (22.0, 20.0), 'amr_05': (24.0, 20.0)}
    return LoopbackFleet(cfgs, comm=comm, poses=poses, path_gate=_gate())


def _drive(f, movers, seconds):
    """movers: 로봇 → Mover. 단계마다 자세 · plan · 단계를 넣고, 막힘은 지난 상태로 본다. 동시 점유를 센다."""
    overlap = []
    for _ in range(int(round(seconds / DT))):
        for r, m in movers.items():
            core = f.cores[r]
            st = core.state()
            m.blocked = any(z.startswith('zone:') and z not in st.held for z in st.requested)
            m.step()
            core.set_pose(*m.pose())
            core.set_plan(None if m.done else m.path)
            core.set_executor_phase('IDLE' if m.done else 'MOVING')
        f.step(DT)
        inside = [r for r, m in movers.items()
                  if not disc_clear_of_polygon(*m.pose()[:2], 0.36, SQ)]
        if len(inside) > 1:
            overlap.append((round(f.t, 2), inside))
    return overlap


def test_single_robot_prerequest_passes_without_stopping_and_releases():
    f = _fleet()
    f.run(1.0)
    m = Mover(H)
    assert _drive(f, {'amr_01': m}, 16.0) == []
    assert m.done and m.stops == 0                          # 사전 요청으로 결정점 전에 허가
    assert f.holders('zone:z') == []                        # 지나온 구역은 몸체가 밖일 때 해제
    ev = [e['event'] for e in f.events if e.get('robot') == 'amr_01']
    assert 'zone_request' in ev and 'gate_release_request' in ev and 'zone_release' in ev


@pytest.mark.parametrize('seed,loss', [(1, 0.0), (1, 0.2), (2, 0.2), (3, 0.2)])
def test_crossing_robots_never_share_the_zone_and_both_arrive(seed, loss):
    comm = CommEmulator(seed=seed, default=LinkModel(0.0, 100.0, loss))
    f = _fleet(comm)
    f.run(2.0)
    a = Mover(H)
    b = Mover([[5.0, -5.0], [5.0, 7.0]])                    # 세로로 z 를 지난다 (입구 y = -1)
    overlap = _drive(f, {'amr_01': a, 'amr_02': b}, 40.0)
    assert overlap == [], overlap
    assert a.done and b.done, (a.s, b.s)
    assert a.stops + b.stops >= 1                           # 경합이 실제로 있었다 (공허 검사)


def test_isolated_robot_stops_at_the_decision_point():
    comm = CommEmulator(seed=1, partitions=[
        Partition(0.0, 1e9, (frozenset({'amr_01'}), frozenset(ROBOTS[1:])))])
    f = _fleet(comm)
    f.run(4.0)
    assert f.modes()['amr_01'] == ISOLATED
    m = Mover(H)
    assert _drive(f, {'amr_01': m}, 10.0) == []
    assert not m.done and m.v == 0.0
    x, _, _ = m.pose()
    assert 4.0 - x >= 1.2                                   # 입구 1.2 m 앞 (2.0 − 제동 0.5 − 반응 …)
    assert disc_clear_of_polygon(x, 0.0, 0.36, SQ)
    st = f.cores['amr_01'].state()
    assert st.requested == ('zone:z',)                      # 막은 구역을 상태에 싣는다
    assert any(e['event'] == 'gate_refused' for e in f.events)


def test_stopping_phase_cancels_the_pending_request():
    f = _fleet()
    f.run(1.0)
    holder = f.cores['amr_02']                              # amr_02 가 게이트로 z 를 얻고 안에 들어가 있다
    holder.set_pose(3.0, 0.0, 0.0)
    holder.set_plan(H)
    holder.set_executor_phase('MOVING')
    f.run(1.0)
    holder.set_pose(5.0, 0.0, 0.0)
    f.run(0.5)
    assert f.holders('zone:z') == ['amr_02']
    core = f.cores['amr_01']
    core.set_pose(2.5, 0.0, 0.0)                            # 입구 1.5 m — 결정점 안
    core.set_plan(H)
    core.set_executor_phase('MOVING')
    f.run(1.0)
    assert core.zone.requested() == frozenset({'zone:z'})
    assert core.state().requested == ('zone:z',)
    core.set_executor_phase('ERROR')                        # 작업 실패 — 더 가지 않는다
    f.run(0.5)
    assert core.zone.requested() == frozenset() and core.state().requested == ()
    assert any(e['event'] == 'gate_cancel' and e['robot'] == 'amr_01' for e in f.events)
