"""
대칭 비교표 시험 (amr_evaluation.slam3d.symmetric) — 합성 궤적의 닫힌 식 기대값.

모든 행을 같은 SE(2) 지표·같은 시간창으로 재고, 3D 열은 3D 행에만 둔다는 규칙을 고정한다.
"""

import json
import math

from amr_evaluation.slam3d import se2, symmetric
import numpy as np
import pytest


def _line(n=401, dt=0.05):
    """직선 x = t (1 m/s), y = 0, yaw = 0 — 20 s, 20 m."""
    t = np.arange(n) * dt
    return t, np.stack([t, np.zeros(n), np.zeros(n)], 1), np.tile([0.0, 0.0, 0.0, 1.0], (n, 1))


def _q_roll(rad):
    return np.array([math.sin(rad / 2), 0.0, 0.0, math.cos(rad / 2)])


def _write_tum(path, t, xyz, q):
    np.savetxt(path, np.column_stack([t, xyz, q]), fmt='%.9f')


def test_common_window_is_intersection():
    lo, hi = symmetric.common_window([np.arange(0, 11), np.arange(2, 9), np.arange(1, 10)])
    assert (lo, hi) == (2.0, 8.0)
    with pytest.raises(ValueError):
        symmetric.common_window([np.arange(0, 3), np.arange(5, 9)])
    with pytest.raises(ValueError):
        symmetric.common_window([np.arange(0, 3), np.zeros(0)])
    with pytest.raises(ValueError):
        symmetric.common_window([])


def test_se2_columns_offset_and_window_clip():
    """추정 = GT + (0.1, 0): 정렬 없음 10 cm, SE(2) 정렬 0. 창 밖(t > 15 s)의 1 m 오차는 빠진다."""
    t, xyz, q = _line()
    gt = se2.Traj2D(t, xyz[:, :2], np.zeros(len(t)))
    exy = xyz[:, :2] + [0.1, 0.0]
    exy[t > 15.0] += [1.0, 0.0]
    est = se2.Traj2D(t, exy, np.zeros(len(t)))
    c = symmetric.se2_columns(gt, est, (0.0, 15.0))
    assert c['ate_xy_rmse_cm'] == pytest.approx(10.0)
    assert c['ate_xy_max_cm'] == pytest.approx(10.0)
    assert c['ate_xy_se2_rmse_cm'] == pytest.approx(0.0, abs=1e-9)
    assert c['ate_yaw_rmse_deg'] == pytest.approx(0.0, abs=1e-12)
    assert c['rpe1_pct'] == pytest.approx(0.0, abs=1e-9)
    assert c['n'] == c['n_window'] == int(np.count_nonzero(t <= 15.0))
    full = symmetric.se2_columns(gt, est, (0.0, 20.0))
    assert full['ate_xy_max_cm'] == pytest.approx(110.0)


def test_se2_columns_yaw():
    t, xyz, _ = _line()
    gt = se2.Traj2D(t, xyz[:, :2], np.zeros(len(t)))
    est = se2.Traj2D(t, xyz[:, :2], np.full(len(t), math.radians(2.0)))
    c = symmetric.se2_columns(gt, est, (0.0, 20.0))
    assert c['ate_yaw_rmse_deg'] == pytest.approx(2.0)
    assert c['ate_yaw_max_deg'] == pytest.approx(2.0)
    assert c['ate_xy_rmse_cm'] == pytest.approx(0.0, abs=1e-12)


def test_dim3_columns_z_and_roll_in_window():
    """추정 z = 5 cm, roll 1°: 창 안만 잰다 — 창 밖 z 50 cm 는 빠진다."""
    t, xyz, q = _line()
    exyz = xyz.copy()
    exyz[:, 2] = 0.05
    exyz[t > 15.0, 2] = 0.5
    eq = np.tile(_q_roll(math.radians(1.0)), (len(t), 1))
    c = symmetric.dim3_columns((t, xyz, q), (t, exyz, eq), (0.0, 15.0))
    assert c['z_abs_max_cm'] == pytest.approx(5.0)
    assert c['z_rmse_cm'] == pytest.approx(5.0)
    assert c['roll_rmse_deg'] == pytest.approx(1.0)
    assert c['pitch_rmse_deg'] == pytest.approx(0.0, abs=1e-9)
    assert c['rp_abs_max_deg'] == pytest.approx(1.0)


def test_floor_columns():
    """바닥 아래 비율 = z < −5 cm 점 / 전체, 바닥 대역(z < 15 cm) z 1 백분위 (선형 보간)."""
    pts = np.array([[0, 0, -0.10], [0, 0, 0.0], [0, 0, 0.0], [0, 0, 0.5]])
    c = symmetric.floor_columns(pts)
    assert c['below_floor_pct'] == pytest.approx(25.0)
    assert c['floor_z_p01_cm'] == pytest.approx(100.0 * np.percentile([-0.1, 0.0, 0.0], 1))
    assert c['n_points'] == 4
    pts2 = np.array([[0, 0, -0.04], [0, 0, -0.06]])
    assert symmetric.floor_columns(pts2)['below_floor_pct'] == pytest.approx(50.0)
    assert symmetric.floor_columns(np.zeros((0, 3)))['n_points'] == 0


def test_spread_and_missing():
    s = symmetric.spread([1.0, 2.0, 3.0, None, float('nan')])
    assert s['n'] == 3 and s['n_missing'] == 2
    assert s['mean'] == pytest.approx(2.0) and s['sd'] == pytest.approx(1.0)
    assert s['range'] == pytest.approx(2.0)
    one = symmetric.spread([4.0])
    assert one['n'] == 1 and math.isnan(one['sd'])
    assert symmetric.spread([None])['n'] == 0


def test_differs_preregistered_rule():
    """범위가 겹치지 않고 평균 차 > 2·max(sd) 일 때만 '다르다'."""
    sp = symmetric.spread
    assert symmetric.differs(sp([1.0, 1.1, 0.9]), sp([2.0, 2.1, 1.9])) is True
    assert symmetric.differs(sp([1.0, 2.0, 1.5]), sp([1.8, 2.5, 2.2])) is False   # 겹침
    # 겹치지 않지만 평균 차 0.65 < 2 × sd 0.5 = 1.0
    assert symmetric.differs(sp([0.0, 1.0, 0.5]), sp([1.1, 1.2, 1.15])) is False
    assert symmetric.differs(sp([1.0]), sp([2.0, 2.1])) is None
    assert symmetric.noise_threshold(0.1, 0.3) == pytest.approx(0.6)


def test_evaluate_manifest_dims_and_markdown(tmp_path):
    """3D 행만 3D 열을 갖고, 공통 창은 모든 실행의 교집합, 행별 산포는 재생 수만큼."""
    t, xyz, q = _line()
    _write_tum(tmp_path / 'gt.tum', t, xyz, q)
    runs = []
    for k, off in enumerate((0.01, 0.02, 0.03)):
        e = xyz.copy()
        e[:, 0] += off
        m = t <= 18.0 - k                          # 끝이 다르다 → 공통 창 끝 = 16 s
        _write_tum(tmp_path / f'a{k}.tum', t[m], e[m], q[m])
        runs.append({'row': 'r3d', 'replay': k, 'tum': str(tmp_path / f'a{k}.tum'), 'dim': '3d'})
    e = xyz.copy()
    e[:, 2] = 0.2                                  # 2D 행의 z 는 보지 않는다
    _write_tum(tmp_path / 'b.tum', t[t >= 1.0], e[t >= 1.0], q[t >= 1.0])
    runs.append({'row': 'r2d', 'replay': 0, 'tum': str(tmp_path / 'b.tum'), 'dim': '2d',
                 'oracle': True})
    man = {'gt': str(tmp_path / 'gt.tum'), 'runs': runs}
    (tmp_path / 'm.json').write_text(json.dumps(man), encoding='utf-8')
    res = symmetric.evaluate(man)
    assert res['window'] == pytest.approx([1.0, 16.0])
    r3 = res['rows']['r3d']['cols']
    assert r3['ate_xy_rmse_cm']['n'] == 3
    assert r3['ate_xy_rmse_cm']['mean'] == pytest.approx(2.0)
    assert r3['ate_xy_rmse_cm']['sd'] == pytest.approx(1.0)
    assert r3['z_abs_max_cm']['mean'] == pytest.approx(0.0, abs=1e-9)
    r2 = res['rows']['r2d']['cols']
    assert r2['z_abs_max_cm']['n'] == 0 and r2['ate_xy_rmse_cm']['n'] == 1
    in_win = int(np.count_nonzero((t >= 1.0 - 1e-9) & (t <= 16.0 + 1e-9)))
    assert [run['n_window'] for run in res['runs']] == [in_win] * 4
    md = symmetric.markdown(res, symmetric.DIM3_COLS)
    assert '해당 없음' in md and '오라클' in md
    out = tmp_path / 'o.json'
    assert symmetric.main(['--manifest', str(tmp_path / 'm.json'), '--out', str(out),
                           '--md', str(tmp_path / 'o.md')]) == 0
    assert json.loads(out.read_text(encoding='utf-8'))['window'] == pytest.approx([1.0, 16.0])
