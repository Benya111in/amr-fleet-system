"""deadlock: wait-for 그래프 · Tarjan SCC · 사이클 확정 · 정체 탐지 · 유형 분류."""

import itertools
import math
import random

import pytest

from amr_fleet.deadlock import (
    DL_CIRCULAR, DL_HEAD_ON, DL_MUTUAL, EDGE_BLOCK, EDGE_CROSSING, EDGE_TOKEN, CycleConfirmer,
    StallDetector, WaitForGraph, classify_deadlock, deadlock_components, find_cycle,
    strongly_connected_components,
)


def test_wait_for_graph_edges_and_reasons():
    g = WaitForGraph()
    g.add_node('c')
    g.add('a', 'b', EDGE_TOKEN)
    g.add('a', 'b', EDGE_BLOCK)          # 같은 쌍은 먼저 넣은 이유 유지
    g.add('b', 'a', EDGE_CROSSING)
    g.add('a', 'a', EDGE_BLOCK)          # 자기 간선은 무시
    assert g.nodes() == ['a', 'b', 'c'] and len(g) == 3
    assert g.reason('a', 'b') == EDGE_TOKEN and g.reason('b', 'c') is None
    assert g.successors('a') == ['b'] and g.successors('c') == []
    assert g.edges() == [('a', 'b', EDGE_TOKEN), ('b', 'a', EDGE_CROSSING)]
    assert g.adjacency() == {'a': ['b'], 'b': ['a'], 'c': []}
    assert deadlock_components(g) == [['a', 'b']]
    with pytest.raises(ValueError):
        g.add('a', 'b', 'nope')


def test_predecessors_closure_follows_only_given_reason():
    g = WaitForGraph()
    g.add('b', 'a', EDGE_BLOCK)          # b 가 a 뒤에 줄 섰다
    g.add('c', 'b', EDGE_BLOCK)
    g.add('d', 'c', EDGE_TOKEN)          # 토큰 간선은 따라가지 않는다
    g.add('a', 'x', EDGE_TOKEN)
    assert g.predecessors_closure({'a'}, EDGE_BLOCK) == {'b', 'c'}
    assert g.predecessors_closure({'a'}) == {'b', 'c', 'd'}
    assert g.predecessors_closure({'x'}, EDGE_BLOCK) == set()


def _reach(adj, s):
    seen, stack = {s}, [s]
    while stack:
        for v in adj.get(stack.pop(), ()):
            if v not in seen:
                seen.add(v)
                stack.append(v)
    return seen


@pytest.mark.parametrize('seed', range(40))
def test_scc_matches_reachability(seed):
    rng = random.Random(seed)
    nodes = [f'r{i}' for i in range(rng.randint(1, 9))]
    adj = {n: [m for m in nodes if m != n and rng.random() < 0.25] for n in nodes}
    comps = strongly_connected_components(adj)
    assert sorted(itertools.chain(*comps)) == sorted(nodes)
    reach = {n: _reach(adj, n) for n in nodes}
    for comp in comps:
        for a, b in itertools.combinations(comp, 2):
            assert b in reach[a] and a in reach[b]
    for c1, c2 in itertools.combinations(comps, 2):
        assert not (c2[0] in reach[c1[0]] and c1[0] in reach[c2[0]])
    for comp in comps:                   # 크기 ≥ 2 SCC 에는 실제 사이클이 있다
        cyc = find_cycle(adj, comp)
        if len(comp) < 2:
            assert cyc == []
            continue
        assert set(cyc) <= set(comp) and len(cyc) >= 2 and cyc[0] == min(comp)
        for u, v in zip(cyc, cyc[1:] + cyc[:1]):
            assert v in adj[u]


def test_scc_deep_chain_is_iterative():
    n = 5000                              # 재귀 구현이면 RecursionError
    adj = {f'n{i}': [f'n{i + 1}'] for i in range(n)}
    adj[f'n{n}'] = ['n0']
    comps = strongly_connected_components(adj)
    assert len(comps) == 1 and len(comps[0]) == n + 1


def test_scc_includes_nodes_only_seen_as_targets():
    assert strongly_connected_components({'a': ['b']}) == [['a'], ['b']]


def test_find_cycle_without_cycle_returns_empty():
    assert find_cycle({'a': ['b'], 'b': []}, ['a', 'b']) == []


def test_cycle_confirmer_waits_and_reports_once():
    c = CycleConfirmer(confirm_s=1.0)
    assert c.update(0.0, [['a', 'b']]) == []
    assert c.pending(0.5) == {frozenset('ab'): 0.5}
    assert c.update(0.5, [['b', 'a']]) == []
    assert c.update(1.0, [['a', 'b']]) == [frozenset('ab')]
    assert c.update(1.5, [['a', 'b']]) == []          # 한 번만
    assert c.reported() == {frozenset('ab')} and c.pending(1.5) == {}
    assert c.update(2.0, []) == []                    # 사라지면 기록 삭제
    assert c.reported() == set()
    assert c.update(2.5, [['a', 'b']]) == []          # 다시 나타나면 처음부터
    assert c.update(3.5, [['a', 'b']]) == [frozenset('ab')]


def test_stall_detector_progress_detour_goal_change():
    s = StallDetector(stall_time_s=5.0, progress_m=0.5, detour_reset_m=2.0, goal_tol_m=0.5)
    g = (10.0, 0.0)
    assert not s.update('a', 0.0, 10.0, g)
    assert s.anchor_time('a') == 0.0 and s.anchor_time('zz') is None
    assert not s.update('a', 3.0, 9.8, g)             # 0.2 m 는 진전이 아니다
    assert s.update('a', 5.0, 9.7, g)                 # 5 s 무진전 → 정체
    assert s.stalled_for('a', 5.0) == pytest.approx(5.0)
    assert not s.update('a', 6.0, 9.0, g)             # 진전 → 기준 갱신
    assert s.anchor_time('a') == 6.0
    assert not s.update('a', 10.0, 12.0, g)           # 큰 우회 = 새 경로 → 기준 갱신
    assert s.anchor_time('a') == 10.0
    assert not s.update('a', 16.0, 12.0, (0.0, 5.0))  # 목표 변경 → 새로 시작
    assert s.anchor_time('a') == 16.0
    assert not s.update('a', 30.0, 12.0, (0.0, 5.0), watch=False)   # 감시 안 함 → 기록 삭제
    assert s.anchor_time('a') is None and s.stalled_for('a', 30.0) == 0.0
    assert not s.update('a', 31.0, None, g)
    s.update('b', 0.0, 3.0, g)
    s.reset('b')
    assert s.anchor_time('b') is None


def test_classify_deadlock():
    h = {'a': 0.0, 'b': math.pi, 'c': math.pi / 2}
    assert classify_deadlock(['a', 'b'], h) == DL_HEAD_ON
    assert classify_deadlock(['a', 'c'], h) == DL_MUTUAL
    assert classify_deadlock(['a', 'x'], h) == DL_MUTUAL      # 방향 모름
    assert classify_deadlock(['a', 'b', 'c'], h) == DL_CIRCULAR
