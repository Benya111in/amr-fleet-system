"""
s2-eval 리뷰의 남은 결함 시험 (b2-rec, evaluation.md §10).

N6 (se2 정렬 실패 → failed=1 행), N11 (observed_mask 메모리), N4 (queue 정지점 감도 변형 queue@Y).
"""

import csv
import math
import tracemalloc

from amr_evaluation.slam3d import cli, gridmap, localizability, results, se2
import numpy as np
import pytest


# ---------------------------------------------------------------- N6
def _write_tum(path, rows):
    with open(path, 'w', encoding='utf-8') as f:
        for t, x, y in rows:
            f.write(f'{t:.6f} {x:.6f} {y:.6f} 0 0 0 0 1\n')


def _run_traj(tmp_path, gt_rows, est_rows, align=('origin', 'se2')):
    g, e, out = tmp_path / 'gt.tum', tmp_path / 'est.tum', tmp_path / 'r.csv'
    _write_tum(g, gt_rows)
    _write_tum(e, est_rows)
    rc = cli.main(['traj', '--gt', str(g), '--est', str(e), '--out', str(out), '--run-id', 'n6',
                   '--scenario', 'SYN', '--bag-id', 'b', '--row', 'r', '--stage', 'slam',
                   '--git-head', 'x', '--align', *align])
    return rc, results.read_csv(str(out))


def test_n6_single_associated_sample_writes_failed_rows_for_se2(tmp_path):
    """연관 표본이 1 개: 예전에는 umeyama_se2 ValueError 로 CLI 가 멈추고 행이 남지 않았다."""
    gt = [(0.0 + 0.02 * i, 0.02 * i, 0.0) for i in range(50)]
    est = [(0.5, 0.51, 0.01)]
    rc, rows = _run_traj(tmp_path, gt, est)
    assert rc == 0
    by = {}
    for r in rows:
        if r['metric'] == 'ate_trans':
            by.setdefault(r['alignment'], []).append(r)
    assert all(r['failed'] == 0 for r in by['none']) and by['none'][0]['n'] == 1
    assert by['none'][0]['value'] == pytest.approx(math.hypot(0.01, 0.01))
    assert all(r['failed'] == 0 for r in by['origin'])
    se2_rows = by['se2']
    assert se2_rows and all(r['failed'] == 1 and math.isnan(r['value']) for r in se2_rows)
    assert all(r['n'] == 1 and '정렬 se2 실패' in r['notes'] for r in se2_rows)
    yaw = [r for r in rows if r['metric'] == 'ate_yaw' and r['alignment'] == 'se2']
    assert yaw and all(r['failed'] == 1 for r in yaw)


def test_n6_no_overlap_keeps_nan_rows_not_failed(tmp_path):
    """연관 표본 0 (시간이 안 겹침): ate() 가 빈 결과를 내므로 실패가 아니라 n = 0, NaN 행 (스키마 그대로)."""
    gt = [(0.02 * i, 0.02 * i, 0.0) for i in range(50)]
    est = [(10.0 + 0.1 * i, 0.0, 0.0) for i in range(5)]
    rc, rows = _run_traj(tmp_path, gt, est)
    assert rc == 0
    ate = [r for r in rows if r['metric'] == 'ate_trans']
    assert ate and all(r['n'] == 0 and math.isnan(r['value']) and r['failed'] == 0 for r in ate)
    assert 'errors' in se2.ate(se2.Traj2D([0, 1], [[0, 0], [1, 0]], [0, 0]),
                               se2.Traj2D([5.0], [[0, 0]], [0]))


def test_n6_csv_failed_column_round_trip(tmp_path):
    gt = [(0.02 * i, 0.02 * i, 0.0) for i in range(50)]
    rc, _ = _run_traj(tmp_path, gt, [(0.5, 0.5, 0.0)], align=('se2',))
    with open(tmp_path / 'r.csv', encoding='utf-8', newline='') as f:
        flags = {(r['alignment'], r['failed']) for r in csv.DictReader(f)}
    assert ('se2', '1') in flags and ('none', '0') in flags


# ---------------------------------------------------------------- N11
def _old_observed_mask(est, target, margin, free_only=False):
    """예전 구현 그대로 (대조 기준)."""
    from scipy import ndimage
    rows, cols = np.mgrid[0:target.shape[0], 0:target.shape[1]]
    pts = np.stack([target.origin_x + (cols.ravel() + 0.5) * target.resolution,
                    target.origin_y + (rows.ravel() + 0.5) * target.resolution], 1)
    known = np.zeros(len(pts), dtype=bool)
    src = est.known & ~est.occupied if free_only else est.known
    if abs(est.origin_yaw) < 1e-12:
        r, c, ok = est.lookup(pts)
        known[ok] = src[r[ok], c[ok]]
    else:
        cs, sn = math.cos(-est.origin_yaw), math.sin(-est.origin_yaw)
        d = pts - [est.origin_x, est.origin_y]
        lx, ly = cs * d[:, 0] - sn * d[:, 1], sn * d[:, 0] + cs * d[:, 1]
        c = np.floor(lx / est.resolution).astype(np.int64)
        r = np.floor(ly / est.resolution).astype(np.int64)
        ok = (c >= 0) & (c < est.shape[1]) & (r >= 0) & (r < est.shape[0])
        known[ok] = src[r[ok], c[ok]]
    mask = known.reshape(target.shape)
    if margin > 0:
        dist = ndimage.distance_transform_edt(~mask) * target.resolution
        mask = dist <= margin + 1e-9
    return mask


def _random_est(rng, shape=(30, 40), res=0.05, ox=-0.73, oy=-0.41, yaw=0.0):
    known = rng.random(shape) < 0.35
    occ = known & (rng.random(shape) < 0.3)
    return gridmap.Grid(occ, known, res, ox, oy, yaw)


def _target(shape=(170, 230), res=0.01, ox=-1.0, oy=-0.8):
    z = np.zeros(shape, dtype=bool)
    return gridmap.Grid(z, z.copy(), res, ox, oy)


@pytest.mark.parametrize('yaw', [0.0, 0.37])
@pytest.mark.parametrize('margin', [0.0, 0.03, 0.07, 0.10])
@pytest.mark.parametrize('free_only', [False, True])
def test_n11_observed_mask_equals_old_implementation(yaw, margin, free_only):
    rng = np.random.default_rng(7)
    est = _random_est(rng, yaw=yaw)
    tgt = _target()
    new = gridmap.observed_mask(est, tgt, margin, free_only, chunk=37)
    old = _old_observed_mask(est, tgt, margin, free_only)
    assert new.shape == old.shape and np.array_equal(new, old)


@pytest.mark.parametrize('radius', [0.5, 1.0, 2.0, 3.0000001, 4.5, 10.0000001])
def test_disk_dilate_equals_distance_threshold(radius):
    from scipy import ndimage
    rng = np.random.default_rng(3)
    m = rng.random((61, 47)) < 0.02
    m[0, 0] = True                                # 모서리 (경계 처리)
    ref = ndimage.distance_transform_edt(~m) <= radius
    for chunk in (5, 16, 1000):
        assert np.array_equal(gridmap.disk_dilate(m, radius, chunk), ref)


def test_disk_dilate_empty_mask_stays_empty():
    assert not gridmap.disk_dilate(np.zeros((10, 10), dtype=bool), 3.0).any()


def test_n11_memory_is_bounded_by_chunk_not_by_grid():
    """
    0.01 GT 에 가까운 격자 (2400 x 3600) 의 최대 추가 메모리.

    새 구현 ≤ bool 격자 둘(입력·출력) + 행 묶음 임시 40 MB, 예전 구현은 셀당 수십 B (좌표 float64 (N, 2)·
    색인 int64·거리변환 float64).
    """
    rng = np.random.default_rng(1)
    est = _random_est(rng, shape=(480, 720), res=0.05, ox=-18.0, oy=-12.0)
    tgt = _target(shape=(2400, 3600), res=0.01, ox=-18.0, oy=-12.0)
    cells = tgt.shape[0] * tgt.shape[1]
    tracemalloc.start()
    new = gridmap.observed_mask(est, tgt, 0.10)
    peak_new = tracemalloc.get_traced_memory()[1]
    tracemalloc.stop()
    assert peak_new < 2 * cells + 40e6, peak_new / cells
    tracemalloc.start()
    old = _old_observed_mask(est, tgt, 0.10)
    peak_old = tracemalloc.get_traced_memory()[1]
    tracemalloc.stop()
    assert np.array_equal(new, old)
    assert peak_old > 5 * peak_new, (peak_old / cells, peak_new / cells)
    print(f'N11 셀당 최대 추가 메모리: 예전 {peak_old / cells:.1f} B, 새 {peak_new / cells:.2f} B')


# ---------------------------------------------------------------- N4
def test_n4_queue_at_variant_matches_queue_at_request_point():
    a = localizability.s6_blockers('queue')
    b = localizability.s6_blockers('queue@-5.5')
    assert np.allclose(a.c, b.c) and np.allclose(a.R, b.R) and np.allclose(a.h, b.h)
    c = localizability.s6_blockers('queue@-6.6')
    assert c.c[0, 1] == pytest.approx(-6.6) and c.label == ['blocker_amr']
