"""
평가 집합 분할 시험 (amr_isaac.eval_sets, Isaac 불필요).

- 배치 직진 여유 = 스칼라 기준판 eval_metrics.straight_approach (무작위 자세·실제 창고 사각형, 손 사례)
- 끝 접선 자세 규칙 = eval_metrics.goal_clearances
- 문턱 0.40 경계, 균형 표본, 길이 범위, 교란 부분 집합
- 커밋된 집합 파일 (rl/isaac/eval_sets/b1_eval_sets_v1.json) 이 경로 풀에서 다시 만들어지는지
  (풀 캐시가 있을 때만 — AMR_ISAAC_ASSETS 또는 기본 위치)
"""

import json
import math
import os

import numpy as np
import pytest

from amr_isaac import eval_metrics as EV
from amr_isaac import eval_sets as ES
from amr_isaac import warehouse_geom as W

HERE = os.path.dirname(os.path.abspath(__file__))
SET_FILE = os.path.join(os.path.dirname(HERE), 'eval_sets', 'b1_eval_sets_v1.json')
ASSET_DIRS = [os.environ.get('AMR_ISAAC_ASSETS', ''),
              '/home/CAPYI_AI/runs/bonus1/b1-binfra/assets',
              '/home/CAPYI_AI/runs/bonus1/b1-isaac-phi/assets']


@pytest.fixture(scope='module')
def obb():
    return W.obstacles_2d(W.load_boxes())


def test_batch_matches_scalar_random_warehouse(obb):
    """창고 사각형 근처 무작위 자세 3000 개에서 배치판 = 스칼라판 (inf·0 포함, 비트 단위가 아니면 1e-12)."""
    rng = np.random.default_rng(7)
    k = rng.integers(0, obb.shape[0], 3000)
    ang = rng.uniform(-math.pi, math.pi, 3000)
    r = rng.uniform(0.0, 2.0, 3000) + np.maximum(obb[k, 2], obb[k, 3])
    poses = np.stack([obb[k, 0] + r * np.cos(ang), obb[k, 1] + r * np.sin(ang),
                      rng.uniform(-math.pi, math.pi, 3000)], 1)
    got = ES.straight_approach_batch(poses, obb)
    ref = np.array([EV.straight_approach(tuple(p), obb) for p in poses])
    assert np.array_equal(np.isinf(got), np.isinf(ref))
    fin = np.isfinite(ref)
    assert np.max(np.abs(got[fin] - ref[fin])) <= 1e-12
    # 세 경우가 모두 나왔는지 (겹침 0, 유한, inf)
    assert (ref == 0).sum() > 50 and fin.sum() > 1000 and (~fin).sum() > 50


def test_batch_hand_cases():
    """정면 상자·옆 상자·겹침·광선 경우(꼭짓점이 띠 밖인 기울어진 상자)."""
    obb = np.array([[2.0, 0.0, 0.5, 0.5, 0.0]])
    poses = [[1.0, 0.0, 0.0], [0.0, 0.0, math.pi / 2], [1.6, 0.0, 0.0]]
    d = ES.straight_approach_batch(poses, obb)
    assert d[0] == pytest.approx(0.2) and math.isinf(d[1]) and d[2] == 0.0
    # 꼭짓점이 모두 차폭 띠 밖이지만 변이 띠를 가로지르는 45° 큰 상자: 광선 경우만으로 닿는다
    big = np.array([[3.0, 0.5, 1.0, 1.0, math.pi / 4]])
    p = [[0.0, 0.0, 0.0]]
    ref = EV.straight_approach(tuple(p[0]), big)
    assert ES.straight_approach_batch(p, big)[0] == pytest.approx(ref, abs=1e-12)
    assert ref == pytest.approx(3.0 - math.sqrt(2.0), abs=1e-9)
    # max_dist 넘으면 inf
    far = np.array([[9.0, 0.0, 0.5, 0.5, 0.0]])
    assert math.isinf(ES.straight_approach_batch(p, far)[0])


def test_end_tangent_and_pool_clearance_match_goal_clearances(obb):
    """작은 합성 풀에서 front_clearance = goal_clearances['goal_front_straight'], 자세 = goal_heading."""
    rng = np.random.default_rng(3)
    paths = []
    for _ in range(40):
        a = rng.uniform(-math.pi, math.pi)
        o = obb[rng.integers(0, obb.shape[0])]
        n = int(rng.integers(3, 60))
        s = np.arange(n) * 0.05
        g = o[:2] + (np.maximum(o[2], o[3]) + rng.uniform(0.3, 1.5)) * np.array(
            [math.cos(a), math.sin(a)])
        h = rng.uniform(-math.pi, math.pi)
        pts = g - (s[::-1, None]) * np.array([math.cos(h), math.sin(h)])
        paths.append(pts)
    offs = np.concatenate([[0], np.cumsum([len(p) for p in paths])])
    pts = np.concatenate(paths)
    got = ES.front_clearance(pts, offs, obb, chunk=7)
    for k, p in enumerate(paths):
        ref = EV.goal_clearances(p, obb)
        pose = ES.end_tangent_pose(p)
        assert pose[2] == pytest.approx(ref['goal_heading'], abs=1e-12)
        if math.isinf(ref['goal_front_straight']):
            assert math.isinf(got[k])
        else:
            assert got[k] == pytest.approx(ref['goal_front_straight'], abs=1e-12)


def test_classify_threshold_boundary():
    """0.40 은 주 집합, 바로 아래는 스트레스, inf 는 주."""
    m, s = ES.classify([0.40, 0.40 - 1e-12, math.inf, 0.0])
    assert m.tolist() == [True, False, True, False]
    assert s.tolist() == [False, True, False, True]


def test_balanced_sample():
    """K < n: 각 원소 ⌊n/K⌋ 또는 ⌈n/K⌉ 번. K ≥ n: 중복 없음. seed 고정이면 같다."""
    rng = np.random.default_rng(0)
    ids = np.arange(100, 137)                       # K = 37
    out = ES.balanced_sample(ids, 512, rng)
    cnt = np.bincount(out - 100)
    assert out.shape == (512,) and cnt.min() == 512 // 37 and cnt.max() == 512 // 37 + 1
    out2 = ES.balanced_sample(np.arange(5000), 2048, np.random.default_rng(1))
    assert np.unique(out2).shape[0] == 2048
    a = ES.balanced_sample(ids, 100, np.random.default_rng(5))
    b = ES.balanced_sample(ids, 100, np.random.default_rng(5))
    assert np.array_equal(a, b)
    with pytest.raises(ValueError):
        ES.balanced_sample(np.array([], dtype=int), 3, rng)


def test_build_eval_sets_rules():
    """집합 원소는 길이 범위·여유 규칙을 지키고, perturb 는 main_3_15 의 앞부분이다."""
    rng = np.random.default_rng(11)
    L = rng.uniform(3.0, 30.0, 6000)
    c = np.where(rng.random(6000) < 0.05, rng.uniform(0.0, 0.4, 6000), rng.uniform(0.4, 5.0, 6000))
    c[rng.random(6000) < 0.2] = np.inf
    sets, stats = ES.build_eval_sets(L, c, n_main=512, n_stress=128, n_perturb=100, seed=2)
    for name, ids in sets.items():
        hi = 15.0 if '3_15' in name else 30.0
        assert np.all((L[ids] >= 3.0) & (L[ids] <= hi))
        if name.startswith('stress'):
            assert np.all(c[ids] < ES.THRESHOLD)
        else:
            assert np.all(c[ids] >= ES.THRESHOLD)
    assert np.array_equal(sets['perturb_3_15'], sets['main_3_15'][:100])
    assert stats['main_3_15']['distinct_paths'] == 512          # 후보가 많으면 비복원
    assert stats['stress_3_15']['n'] == 128
    sets2, _ = ES.build_eval_sets(L, c, n_main=512, n_stress=128, n_perturb=100, seed=2)
    assert all(np.array_equal(sets[k], sets2[k]) for k in sets)


def test_pool_digest_sensitive():
    """경로 풀 내용이 한 값만 달라도 해시가 바뀐다."""
    pts = np.zeros((10, 2))
    offs = np.array([0, 4, 10])
    L = np.array([3.0, 5.0])
    d0 = ES.pool_digest(pts, offs, L)
    pts2 = pts.copy()
    pts2[3, 1] = 1e-9
    assert ES.pool_digest(pts2, offs, L) != d0
    assert ES.pool_digest(pts, offs, L) == d0


def test_committed_set_file_shape():
    """커밋된 집합 파일: 형식·네 집합 + 교란 부분 집합·n·규칙 값."""
    doc = ES.load_sets(SET_FILE)
    s = doc['sets']
    assert set(s) == {'main_3_15', 'stress_3_15', 'main_3_30', 'stress_3_30', 'perturb_3_15'}
    assert len(s['main_3_15']) == 2048 and len(s['main_3_30']) == 2048
    assert len(s['stress_3_15']) >= 512 and len(s['stress_3_30']) >= 512
    assert s['perturb_3_15'] == s['main_3_15'][:1024]
    assert doc['threshold_m'] == ES.THRESHOLD and doc['pool_seed'] == 1


def _find_assets():
    for d in ASSET_DIRS:
        if d and os.path.isdir(d) and any(f.endswith('_s1.npz') for f in os.listdir(d)):
            return d
    return None


@pytest.mark.skipif(_find_assets() is None, reason='held-out 경로 풀 캐시 없음')
def test_committed_set_file_reproduces_from_pool():
    """경로 풀 캐시에서 다시 만든 집합 = 커밋된 파일 (풀 해시·경로 번호·여유 규칙)."""
    from amr_isaac.assets import load_geometry, load_pool
    d = _find_assets()
    doc = ES.load_sets(SET_FILE)
    geo = load_geometry(d)
    pool = load_pool(seed=1, out_dir=d, log=None)
    ES.check_pool(doc, pool['pts'], pool['offs'], pool['length'])
    clear = ES.front_clearance(pool['pts'], pool['offs'], geo['obb'])
    sets, _ = ES.build_eval_sets(pool['length'], clear, seed=doc['sample_seed'],
                                 threshold=doc['threshold_m'])
    assert json.dumps({k: [int(x) for x in v] for k, v in sets.items()}, sort_keys=True) == \
        json.dumps(doc['sets'], sort_keys=True)


def test_cluster_bootstrap_ci():
    """군집 부트스트랩: 군집 안 상관이 크면 독립 가정보다 넓고, 군집 하나면 점추정, 점추정 = 평균."""
    x = np.array([1.0] * 50 + [0.0] * 50)
    g_corr = np.array([0] * 50 + [1] * 50)             # 군집이 결과와 완전히 겹침
    g_ind = np.arange(100)
    e1, lo1, hi1 = ES.cluster_bootstrap_ci(x, g_corr, reps=4000)
    e2, lo2, hi2 = ES.cluster_bootstrap_ci(x, g_ind, reps=4000)
    assert e1 == e2 == 0.5
    assert (hi1 - lo1) > 2 * (hi2 - lo2)
    assert ES.cluster_bootstrap_ci(x, np.zeros(100)) == (0.5, 0.5, 0.5)
    rng = np.random.default_rng(0)
    y = (rng.random(400) < 0.9).astype(float)
    e, lo, hi = ES.cluster_bootstrap_ci(y, np.arange(400) // 4)
    assert lo < e < hi and e == pytest.approx(y.mean())
