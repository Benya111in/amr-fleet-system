"""복제 대기 그래프 시험 — 결정적 Tarjan(deadlock.py 재사용), knot, timestamp 희생자, 진행 확인."""

import itertools

from amr_fleet import deadlock
from amr_fleet.distributed import wfg as wfg_mod
from amr_fleet.distributed.ordering import Stamp
from amr_fleet.distributed.wfg import (
    deadlock_candidates, OutEdge, ProgressConfirmer, ReplicatedWfg, select_victim, WaitInfo,
)


def info(r, edges, prio=0, req=None, seq=1, inc=1, rem=None):
    return WaitInfo(r, inc, seq, tuple(OutEdge(h, why) for h, why in edges), prio,
                    None if req is None else Stamp.coerce(req), rem)


def test_reuses_existing_tarjan():
    assert wfg_mod.strongly_connected_components is deadlock.strongly_connected_components
    assert wfg_mod.WaitForGraph is deadlock.WaitForGraph


def test_token_cycle_is_candidate_and_replicas_agree_regardless_of_order():
    beacons = [info('a', [('b', 'token')], prio=1, req=10.0),
               info('b', [('c', 'crossing')], prio=1, req=12.0),
               info('c', [('a', 'token')], prio=1, req=11.0),
               info('d', [('a', 'token')], prio=0, req=1.0)]   # 사이클 밖에서 기다리는 로봇
    verdicts = set()
    for perm in itertools.permutations(beacons):
        w = ReplicatedWfg()
        for b in perm:
            w.update(b, 0.0)
        cands = w.candidates(0.5)
        fresh = w.fresh(0.5)
        verdicts.add((tuple(map(tuple, cands)),
                      tuple(select_victim(c, fresh) for c in cands)))
    assert verdicts == {((('a', 'b', 'c'),), ('b',))}       # 같은 우선순위 → 가장 늦게 요청한 b


def test_victim_order_priority_then_youngest_then_id():
    infos = {'a': info('a', [], prio=0, req=5.0), 'b': info('b', [], prio=2, req=9.0),
             'c': info('c', [], prio=0, req=7.0)}
    assert select_victim(['a', 'b', 'c'], infos) == 'c'     # 우선순위 0 중 더 어린 c
    infos['c'] = info('c', [], prio=0, req=5.0)
    assert select_victim(['a', 'b', 'c'], infos) == 'c'     # 시각도 같으면 id 큰 쪽
    assert select_victim(['a', 'x'], {'a': infos['a']}) == 'x'   # 정보 없음 = 가장 어림


def test_victim_is_stable_when_victim_rerequests():
    """희생자가 비켰다 다시 요청해도 처음 요청 시각을 유지하면 같은 쌍에서 방향이 안 뒤집힌다."""
    first = {'a': info('a', [('b', 'token')], req=1.0), 'b': info('b', [('a', 'token')], req=2.0)}
    v1 = select_victim(['a', 'b'], first)
    again = {'a': info('a', [('b', 'token')], req=1.0, seq=2),
             'b': info('b', [('a', 'token')], req=2.0, seq=2)}
    assert v1 == 'b' and select_victim(['a', 'b'], again) == v1


def test_block_scc_needs_knot():
    g = deadlock.WaitForGraph()
    g.add('a', 'b', 'block')
    g.add('b', 'a', 'block')
    g.add('b', 'c', 'token')        # b 는 a 말고 c 에도 막혀 있다 → 밖으로 나가는 간선
    assert deadlock_candidates(g) == []
    g2 = deadlock.WaitForGraph()
    g2.add('a', 'b', 'block')
    g2.add('b', 'a', 'block')
    assert deadlock_candidates(g2) == [['a', 'b']]
    g3 = deadlock.WaitForGraph()    # AND 대기만이면 밖으로 나가는 간선이 있어도 후보
    g3.add('a', 'b', 'token')
    g3.add('b', 'a', 'crossing')
    g3.add('b', 'c', 'token')
    assert deadlock_candidates(g3) == [['a', 'b']]


def test_stale_and_old_beacons_are_ignored():
    w = ReplicatedWfg(ttl_s=2.0)
    assert w.update(info('a', [('b', 'token')], seq=5), 0.0)
    assert w.update(info('b', [('a', 'token')], seq=5), 0.0)
    assert not w.update(info('a', [], seq=4), 0.5)            # 재정렬된 옛 비컨
    assert w.candidates(1.0) == [['a', 'b']]
    assert w.candidates(2.5) == []                             # 둘 다 묵음 → 판정에서 빠진다
    assert w.update(info('a', [], seq=0, inc=2), 3.0)          # 재시작한 로봇(새 incarnation)
    assert w.fresh(3.0)['a'].incarnation == 2


def test_progress_confirmer_resets_on_progress():
    pc = ProgressConfirmer(confirm_s=1.0, eps_m=0.05)
    c = [['a', 'b']]
    assert pc.update(0.0, c, {'a': 5.0, 'b': 3.0}) == []
    assert pc.update(0.9, c, {'a': 5.0, 'b': 2.9}) == []       # b 가 0.1 m 나아갔다 → 다시 잰다
    assert pc.update(1.5, c, {'a': 5.0, 'b': 2.9}) == []
    assert pc.update(1.9, c, {'a': 4.99, 'b': 2.9}) == [frozenset({'a', 'b'})]
    assert pc.update(3.0, c, {'a': 4.99, 'b': 2.9}) == []      # 한 번만
    assert pc.update(3.1, [], {}) == []                         # 사라지면 기록 삭제
    assert pc.update(3.2, c, {'a': 4.99, 'b': 2.9}) == []
