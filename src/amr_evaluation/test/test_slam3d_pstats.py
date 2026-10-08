"""
P 단계 통계·보조 지표 시험 (amr_evaluation.slam3d.pstats·pmetrics) — 닫힌 식과 손 계산 기대값.

사전 등록 분석(peval.py compare)이 기대는 성질을 고정한다: Wilcoxon 정확 p 의 최솟값 2/2^n, Holm 단조성,
Wilson 0/n 상한, 두 단계 부트스트랩 구간이 상수 차를 그대로 내는 것, 판정 규칙의 네 갈래, 통로 축·횡 분해,
블로커 가림, 로그 지연 파싱.
"""

import math

from amr_evaluation.slam3d import gridmap, localizability, pmetrics, pstats
import numpy as np
import pytest


# ---------------------------------------------------------------- Wilcoxon
def test_wilcoxon_all_same_sign_is_min_p():
    for n in range(1, 10):
        r = pstats.wilcoxon_exact([float(i + 1) for i in range(n)])
        assert r['n'] == n
        assert r['p'] == pytest.approx(pstats.min_p_wilcoxon(n))
        assert pstats.min_p_wilcoxon(n) == pytest.approx(min(1.0, 2.0 / 2 ** n))


def test_wilcoxon_symmetric_sample_is_one():
    assert pstats.wilcoxon_exact([-1.0, 1.0])['p'] == pytest.approx(1.0)
    assert pstats.wilcoxon_exact([])['p'] == 1.0
    assert pstats.wilcoxon_exact([0.0, 0.0])['n'] == 0


def test_wilcoxon_hand_value():
    # d = [−1, 2, 3, 4]: 순위 1..4, W+ = 9, E = 5, |9 − 5| = 4. 16 부호 중 |W+ − 5| ≥ 4 인 것:
    # W+ ∈ {0, 1, 9, 10} → {} , {1}, {2,3,4}(=9), {1,2,3,4} = 4 개 → p = 4/16
    r = pstats.wilcoxon_exact([-1.0, 2.0, 3.0, 4.0])
    assert r['w_plus'] == pytest.approx(9.0)
    assert r['p'] == pytest.approx(4 / 16)


def test_wilcoxon_matches_scipy_exact():
    sp = pytest.importorskip('scipy.stats')
    d = [0.3, -0.1, 0.5, 0.7, -0.25, 0.9, 1.1]
    ref = sp.wilcoxon(d, method='exact').pvalue
    assert pstats.wilcoxon_exact(d)['p'] == pytest.approx(ref, abs=1e-12)


def test_wilcoxon_ties_use_mid_ranks():
    r = pstats.wilcoxon_exact([1.0, 1.0, -1.0])
    # 순위 모두 2, W+ = 4, E = 3. 8 부호 중 |W+ − 3| ≥ 1: W+ ∈ {0, 2, 4, 6} 중 2 와 4 는 |·| = 1 → 전부 → p = 1
    assert r['w_plus'] == pytest.approx(4.0)
    assert r['p'] == pytest.approx(1.0)


# ---------------------------------------------------------------- Holm, Wilson
def test_holm_hand_values_and_order():
    adj = pstats.holm([0.01, 0.04, 0.03])
    assert adj == pytest.approx([0.03, 0.06, 0.06])
    assert pstats.holm([0.5, 0.9]) == pytest.approx([1.0, 1.0])


def test_wilson_zero_of_ten():
    lo, hi = pstats.wilson(0, 10)
    assert lo == pytest.approx(0.0, abs=1e-12)
    assert hi == pytest.approx(0.2775, abs=5e-4)
    lo, hi = pstats.wilson(10, 10)
    assert hi == pytest.approx(1.0) and lo == pytest.approx(0.7225, abs=5e-4)
    assert pstats.wilson(0, 0) == (0.0, 1.0)


# ---------------------------------------------------------------- 부트스트랩·분산 성분·판정
def test_cluster_bootstrap_constant_difference():
    pairs = {f'b{i}': [(1.0 + i, 0.5 + i), (2.0 + i, 1.5 + i)] for i in range(5)}
    b = pstats.cluster_bootstrap(pairs, n_boot=500)
    assert b['mean'] == pytest.approx(0.5)
    assert b['lo'] == pytest.approx(0.5) and b['hi'] == pytest.approx(0.5)
    assert b['n_bags'] == 5


def test_cluster_bootstrap_ignores_nan_pairs_and_is_seeded():
    pairs = {'a': [(1.0, 0.0), (math.nan, 0.0)], 'b': [(3.0, 0.0)], 'c': [(math.nan, 1.0)]}
    b1 = pstats.cluster_bootstrap(pairs, n_boot=300, seed=1)
    b2 = pstats.cluster_bootstrap(pairs, n_boot=300, seed=1)
    assert b1 == b2
    assert b1['n_bags'] == 2
    assert b1['mean'] == pytest.approx(2.0)
    assert pstats.bag_diffs(pairs)['a'] == pytest.approx(1.0)
    assert math.isnan(pstats.bag_diffs(pairs)['c'])


def test_variance_components():
    vc = pstats.variance_components({'a': [1.0, 3.0], 'b': [5.0, 7.0]})
    assert vc['within_sd'] == pytest.approx(math.sqrt(2.0))
    assert vc['between_sd'] == pytest.approx(math.sqrt(8.0))
    assert vc['n_bags'] == 2 and vc['n_values'] == 4


def test_verdict_branches():
    assert pstats.verdict(0.01, -2.0, -1.0, -1.5, 0.2) == 'A<B'
    assert pstats.verdict(0.01, 1.0, 2.0, 1.5, 0.2) == 'A>B'
    # 구간이 0 을 포함 → 기각해도 판정 없음
    assert pstats.verdict(0.01, -1.0, 0.5, -0.2, 0.2) == 'undecided'
    # p 가 α 이상
    assert pstats.verdict(0.06, -2.0, -1.0, -1.5, 0.2) == 'undecided'
    # 차가 δ 보다 작다 (실용상 무의미) → 동등 한계 안이면 equiv
    assert pstats.verdict(0.001, -0.15, -0.05, -0.1, 0.2) == 'equiv'
    assert pstats.verdict(None, -0.1, 0.1, 0.0, 0.2) == 'equiv'
    assert pstats.verdict(0.01, math.nan, 1.0, 0.5, 0.2) == 'undecided'


def test_replay_rule():
    assert pstats.replay_rule([1.0, 1.0, 1.0], [0.5, 0.5, 0.5], 0.2) == 'differs'
    assert pstats.replay_rule([1.0, 1.0, 1.0], [0.9, 0.9, 0.9], 0.2) == 'undecided'
    assert pstats.replay_rule([1.0, 2.0, 3.0], [1.5, 1.5, 1.5], 0.2) == 'undecided'
    assert pstats.replay_rule([], [1.0], 0.2) == 'undecided'


# ---------------------------------------------------------------- pmetrics
def test_aisle_errors_split_axis_and_lateral():
    gt = np.array([[0.0, -10.0], [0.0, -9.0], [5.0, 0.0]])
    est = gt + np.array([[0.01, 0.03], [-0.01, -0.03], [1.0, 1.0]])
    r = pmetrics.aisle_errors(gt, est)
    assert r['axis']['n'] == 2 and r['lateral']['n'] == 2
    assert r['axis']['rmse'] == pytest.approx(0.03)
    assert r['lateral']['rmse'] == pytest.approx(0.01)
    assert r['axis']['mean'] == pytest.approx(0.0)
    assert r['axis']['max'] == pytest.approx(0.03)


def test_aisle_errors_empty_outside():
    r = pmetrics.aisle_errors(np.array([[5.0, 5.0]]), np.array([[5.0, 5.0]]))
    assert r['axis']['n'] == 0 and math.isnan(r['axis']['rmse'])


def test_blocker_rects_wall_and_queue():
    wall = pmetrics.blocker_rects(localizability.s6_blockers('wall'))
    assert wall[0] == pytest.approx((-0.5, 0.5, -8.0, -7.5))
    assert wall[1] == pytest.approx((-0.5, 0.5, -12.5, -12.0))
    q = pmetrics.blocker_rects(localizability.s6_blockers('queue'))
    assert len(q) == 1
    x0, x1, y0, y1 = q[0]
    # AMR 0.6(길이) × 0.4(폭) 이 yaw −π/2 → x 폭 0.4, y 길이 0.6, + 여유 0.1
    assert (x1 - x0) == pytest.approx(0.6, abs=1e-9) and (y1 - y0) == pytest.approx(0.8, abs=1e-9)
    assert (y0 + y1) / 2 == pytest.approx(-5.5)
    assert pmetrics.blocker_rects(localizability.s6_blockers('open')) == []


def test_mask_boxes_removes_cells_from_both_counts():
    occ = np.zeros((10, 10), dtype=bool)
    occ[5, 5] = True
    g = gridmap.Grid(occ, np.ones_like(occ), 0.1, 0.0, 0.0)
    m = pmetrics.mask_boxes(g, [(0.5, 0.6, 0.5, 0.6)])
    assert not m.occupied[5, 5] and not m.known[5, 5]
    assert m.known.sum() == 99
    assert g.occupied[5, 5]                       # 원본은 그대로


def test_latency_parsing():
    txt = ('Odom: ratio=0.0, std dev=99m|99rad, update time=0.004s delay=0.01s\n'
           'Odom: ratio=0.8, std dev=0.01m|0.003rad, update time=0.050s delay=0.02s\n'
           'Odom: ratio=0.8, std dev=0.01m|0.003rad, update time=0.070s delay=0.02s\n'
           'rtabmap (1): Rate=1.00s, Limit=0.000s, Conversion=0.0004s, RTAB-Map=0.1000s, x\n')
    a = pmetrics.icp_latency(txt)
    assert a['n'] == 2 and a['max_ms'] == pytest.approx(70.0)
    assert a['p50_ms'] == pytest.approx(60.0)
    b = pmetrics.rtabmap_latency(txt)
    assert b['n'] == 1 and b['p50_ms'] == pytest.approx(100.0)
    assert pmetrics.icp_latency('')['n'] == 0


def test_region_raster_and_coverage():
    xy = np.array([[0.0, 0.0], [10.0, 0.0]])
    reg = pmetrics.region_raster(xy, radius=2.0, cell=0.5)
    pts = np.array([[5.0, 1.0, 0.0], [5.0, 3.0, 0.0], [-1.5, 0.0, 0.0], [12.6, 0.0, 0.0]])
    assert pmetrics.in_region(reg, pts).tolist() == [True, False, True, False]
    assert pmetrics.coverage(np.array([1.0, 9.0]), np.array([0.0, 10.0])) == pytest.approx(0.8)
    assert pmetrics.coverage(np.array([]), np.array([0.0, 10.0])) == 0.0
