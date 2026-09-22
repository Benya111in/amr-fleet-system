"""allocation: nearest / load_balance / hungarian + 최적성 지표 (명세 4.9)."""

import math
import random

import pytest

from amr_fleet import allocation as al
from amr_fleet.task_schema import Pose2D, TaskSpec


def task(task_id, px, py, dx=None, dy=None, robot_id=''):
    dx = px if dx is None else dx
    dy = py if dy is None else dy
    return TaskSpec(task_id=task_id, pickup=Pose2D(px, py), dropoff=Pose2D(dx, dy),
                    item_type='small', item_mass=2.0, robot_id=robot_id)


def robot(robot_id, x, y, load=0, available=True):
    return al.RobotInfo(robot_id, x, y, load=load, available=available)


# ---------- 공통 유틸 ----------

def test_distance_and_travel_time():
    assert al.euclid(0, 0, 3, 4) == 5.0
    assert al.approach_distance(robot('r', 0, 0), task('t', 3, 4)) == 5.0
    assert al.delivery_distance(task('t', 0, 0, 6, 8)) == 10.0
    assert al.travel_time(10.0, 2.0) == 5.0
    with pytest.raises(ValueError):
        al.travel_time(1.0, 0.0)
    with pytest.raises(ValueError):
        al.NearestStrategy(nominal_speed=-1.0)


def test_registry():
    assert al.available_strategies() == ['hungarian', 'load_balance', 'nearest']
    assert isinstance(al.create_strategy('nearest', nominal_speed=1.5), al.NearestStrategy)
    assert al.create_strategy('hungarian').name == 'hungarian'
    with pytest.raises(KeyError):
        al.create_strategy('does_not_exist')

    @al.register_strategy('always_first')
    class AlwaysFirst(al.AllocationStrategy):
        def assign(self, tasks, robots):
            return {t.task_id: robots[0].robot_id for t in tasks[:1]}

    try:
        s = al.create_strategy('always_first')
        assert s.assign([task('t', 0, 0)], [robot('r9', 0, 0)]) == {'t': 'r9'}
    finally:
        del al._REGISTRY['always_first']


# ---------- nearest ----------

def test_nearest_picks_closest_robot_per_task_in_order():
    robots = [robot('r_far', 100, 0), robot('r_near', 1, 0), robot('r_mid', 10, 0)]
    tasks = [task('t1', 0, 0), task('t2', 12, 0)]
    out = al.NearestStrategy().assign(tasks, robots)
    assert out == {'t1': 'r_near', 't2': 'r_mid'}


def test_nearest_one_task_per_robot_and_tiebreak_by_id():
    robots = [robot('b', 0, 0), robot('a', 0, 0)]
    out = al.NearestStrategy().assign([task('t1', 0, 0), task('t2', 0, 0), task('t3', 0, 0)],
                                      robots)
    assert out == {'t1': 'a', 't2': 'b'}


# ---------- load_balance ----------

def test_load_balance_prefers_fewest_load_then_nearest():
    robots = [robot('busy_near', 0, 0, load=3), robot('idle_far', 50, 0, load=0),
              robot('idle_mid', 20, 0, load=0)]
    out = al.LoadBalanceStrategy().assign([task('t1', 0, 0), task('t2', 0, 0)], robots)
    assert out == {'t1': 'idle_mid', 't2': 'idle_far'}


def test_load_balance_does_not_mutate_input():
    robots = [robot('r1', 0, 0, load=0)]
    al.LoadBalanceStrategy().assign([task('t1', 0, 0)], robots)
    assert robots[0].load == 0


# ---------- hungarian ----------

CROSS_COST = [
    [1.0, 2.0, 100.0],
    [2.0, 100.0, 1.0],
    [100.0, 1.0, 2.0],
]


@pytest.mark.parametrize('use_scipy', [True, False])
def test_hungarian_3x3_equals_brute_force(use_scipy):
    best_cost, best = al.brute_force_assignment(CROSS_COST)
    pairs = al.solve_assignment(CROSS_COST, use_scipy=use_scipy)
    assert sum(CROSS_COST[r][c] for r, c in pairs) == pytest.approx(best_cost)
    assert best_cost == 3.0                      # (0,0) + (1,2) + (2,1) = 1 + 1 + 1
    assert pairs == best == [(0, 0), (1, 2), (2, 1)]


def test_hungarian_3x3_hand_instance_known_optimum():
    cost = [[4.0, 1.0, 3.0], [2.0, 0.0, 5.0], [3.0, 2.0, 2.0]]
    pairs = al.hungarian(cost)
    assert pairs == [(0, 1), (1, 0), (2, 2)]
    assert sum(cost[r][c] for r, c in pairs) == 5.0
    assert al.brute_force_assignment(cost)[0] == 5.0


@pytest.mark.parametrize('n, m', [(1, 1), (2, 3), (3, 2), (4, 4), (5, 3), (3, 6), (6, 6)])
def test_hungarian_random_matches_brute_force(n, m):
    rng = random.Random(1234 + n * 10 + m)
    for _ in range(15):
        cost = [[rng.uniform(0, 50) for _ in range(m)] for _ in range(n)]
        best_cost, _ = al.brute_force_assignment(cost)
        for use_scipy in (True, False):
            pairs = al.solve_assignment(cost, use_scipy=use_scipy)
            assert len(pairs) == min(n, m)
            assert len({r for r, _ in pairs}) == len(pairs)
            assert len({c for _, c in pairs}) == len(pairs)
            assert sum(cost[r][c] for r, c in pairs) == pytest.approx(best_cost, abs=1e-9)


def test_hungarian_empty_inputs():
    assert al.hungarian([]) == []
    assert al.hungarian([[]]) == []
    assert al.HungarianStrategy().assign([], [robot('r', 0, 0)]) == {}
    assert al.HungarianStrategy().assign([task('t', 0, 0)], []) == {}


def test_hungarian_strategy_beats_nearest_on_crossing_instance():
    # 로봇 A 는 t1 에 아주 가깝지만 t2 에도 가깝고, 로봇 B 는 t2 에만 닿는다.
    robots = [robot('A', 0, 0), robot('B', 10, 0)]
    tasks = [task('t1', 9, 0), task('t2', 1, 0)]      # 스케줄러 순서: t1 먼저
    nearest = al.NearestStrategy().assign(tasks, robots)
    hung = al.HungarianStrategy().assign(tasks, robots)
    assert nearest == {'t1': 'B', 't2': 'A'}
    assert hung == {'t1': 'B', 't2': 'A'}
    # 진짜 갈리는 인스턴스: 탐욕은 t1 을 A 에 주고 t2 는 먼 B 가 맡는다
    tasks = [task('t1', 4, 0), task('t2', 0, 0)]
    nearest = al.NearestStrategy().assign(tasks, robots)
    hung = al.HungarianStrategy().assign(tasks, robots)
    assert nearest == {'t1': 'A', 't2': 'B'}
    assert hung == {'t1': 'B', 't2': 'A'}

    def total_approach(out):
        by_r = {r.robot_id: r for r in robots}
        by_t = {t.task_id: t for t in tasks}
        return sum(al.approach_distance(by_r[rid], by_t[tid]) for tid, rid in out.items())

    assert total_approach(hung) == 6.0 < total_approach(nearest) == 14.0


def test_hungarian_cost_matrix_is_travel_time():
    s = al.HungarianStrategy(nominal_speed=2.0)
    cm = s.cost_matrix([task('t', 4, 0)], [robot('r', 0, 0)])
    assert cm == [[2.0]]


# ---------- allocate() 래퍼 + 지표 ----------

def test_allocate_honours_pinned_and_unavailable_robots():
    robots = [robot('amr_01', 0, 0), robot('amr_02', 5, 0, available=False), robot('amr_03', 9, 0)]
    tasks = [task('pin', 0, 0, robot_id='amr_03'), task('pin_busy', 0, 0, robot_id='amr_02'),
             task('pin_unknown', 0, 0, robot_id='amr_99'), task('auto', 8, 0)]
    res = al.allocate(al.NearestStrategy(), tasks, robots)
    assert res.assignments == {'pin': 'amr_03', 'auto': 'amr_01'}
    assert res.metrics['strategy'] == 'nearest'
    assert res.metrics['n_assigned'] == 2 and res.metrics['n_tasks'] == 4
    assert res.metrics['n_robots'] == 3


def test_allocate_metrics_total_distance_and_makespan():
    robots = [robot('r1', 0, 0), robot('r2', 0, 10)]
    tasks = [task('t1', 3, 4, 6, 8), task('t2', 0, 10, 0, 20)]   # t1: 5 + 5, t2: 0 + 10
    res = al.allocate(al.HungarianStrategy(nominal_speed=1.0), tasks, robots)
    assert res.assignments == {'t1': 'r1', 't2': 'r2'}
    m = res.metrics
    assert m['total_travel_distance_m'] == pytest.approx(20.0)
    assert m['approach_distance_m'] == pytest.approx(5.0)
    assert m['mean_approach_distance_m'] == pytest.approx(2.5)
    assert m['makespan_s'] == pytest.approx(10.0)
    assert m['compute_time_ms'] >= 0.0


def test_allocate_with_nothing():
    res = al.allocate(al.NearestStrategy(), [], [])
    assert res.assignments == {} and res.metrics['makespan_s'] == 0.0
    assert res.metrics['mean_approach_distance_m'] == 0.0


def test_allocate_ignores_strategy_output_for_taken_robots():
    class Greedy(al.AllocationStrategy):
        name = 'greedy'

        def assign(self, tasks, robots):
            return {t.task_id: robots[0].robot_id for t in tasks}   # 같은 로봇을 중복 반환

    res = al.allocate(Greedy(), [task('a', 0, 0), task('b', 0, 0)], [robot('r', 0, 0)])
    assert res.assignments == {'a': 'r'}


def test_evaluate_assignment_direct():
    robots = [robot('r', 0, 0)]
    tasks = [task('t', 0, 3, 4, 3)]
    m = al.evaluate_assignment({'t': 'r'}, tasks, robots, nominal_speed=0.5)
    assert m['total_travel_distance_m'] == pytest.approx(7.0)
    assert m['makespan_s'] == pytest.approx(14.0)
    assert math.isclose(m['n_assigned'], 1.0)


# ---------- select_batch (head-of-line blocking 방지) ----------

def test_select_batch_skips_pinned_tasks_of_busy_or_unknown_robots():
    robots = [robot('amr_t1', 0, 0)]                    # amr_t2 는 바쁨(후보 아님)
    ordered = [task('next_t2', 1, 1, robot_id='amr_t2'), task('typo', 1, 1, robot_id='amr_5'),
               task('auto', 2, 2)]
    batch = al.select_batch(ordered, robots)
    assert [t.task_id for t in batch] == ['auto']
    assert al.allocate(al.NearestStrategy(), batch, robots).assignments == {'auto': 'amr_t1'}


def test_select_batch_keeps_order_and_one_pinned_task_per_robot():
    robots = [robot('a', 0, 0), robot('b', 5, 0), robot('c', 9, 0, available=False)]
    ordered = [task('p_a1', 0, 0, robot_id='a'), task('p_a2', 0, 0, robot_id='a'),
               task('p_c', 0, 0, robot_id='c'), task('u1', 1, 0), task('u2', 2, 0)]
    assert [t.task_id for t in al.select_batch(ordered, robots)] == ['p_a1', 'u1']
    assert [t.task_id for t in al.select_batch(ordered, robots, k=3)] == ['p_a1', 'u1', 'u2']
    assert [t.task_id for t in al.select_batch(ordered, robots, k=1)] == ['p_a1']


def test_select_batch_without_robots_or_tasks():
    assert al.select_batch([task('t', 0, 0)], []) == []
    assert al.select_batch([task('t', 0, 0)], [robot('r', 0, 0, available=False)]) == []
    assert al.select_batch([], [robot('r', 0, 0)]) == []
