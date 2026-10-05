"""
보너스 2 평가 지표 시험 — 합성 궤적·지도·점군의 해석값 (amr_evaluation.slam3d).

모든 기대값은 손으로 유도한 닫힌 식이다 (독스트링·주석에 유도).
"""

import math
from pathlib import Path

from amr_evaluation.slam3d import cli, cloud, gridmap, results, se2
import numpy as np
import pytest


# ---------------------------------------------------------------- SE(2) 궤적
def line(n=201, dt=0.05, v=1.0, yaw=0.0, t0=0.0):
    """직선 궤적 x = v t (방위 yaw 고정 — 진행 방향과 무관하게 방위만 준다)."""
    t = t0 + np.arange(n) * dt
    return se2.Traj2D(t, np.stack([v * (t - t0), np.zeros(n)], 1), np.full(n, yaw))


def test_identical_is_zero():
    g = line()
    r = se2.ate(g, g)
    assert r['trans']['rmse'] == 0 and r['yaw_deg']['max'] == 0 and r['n_used'] == len(g)
    for d, st in se2.rpe(g, g, (1.0, 5.0)).items():
        assert st['trans']['max'] == pytest.approx(0, abs=1e-12)
    assert se2.rte(g, g, 2.0)['trans']['max'] == pytest.approx(0, abs=1e-12)


def test_constant_offset_none_vs_aligned():
    """추정 = GT + (0.1, 0): 정렬 없음 ATE = 0.1, origin·se2 정렬 후 0, RPE 0 (상대 운동 같음)."""
    g = line()
    e = se2.Traj2D(g.t, g.xy + [0.1, 0.0], g.yaw)
    assert se2.ate(g, e)['trans']['rmse'] == pytest.approx(0.1)
    assert se2.ate(g, e, 'origin')['trans']['max'] == pytest.approx(0, abs=1e-12)
    assert se2.ate(g, e, 'se2')['trans']['max'] == pytest.approx(0, abs=1e-12)
    assert se2.rpe(g, e, (1.0,))[1.0]['trans']['max'] == pytest.approx(0, abs=1e-12)


def test_yaw_bias_rpe():
    """
    방위만 ε 틀림: ATE 병진 0, 방위 ε. 거리 Δ 쌍의 상대 병진은 추정 좌표계에서 R(−ε)d 라 오차 2Δ sin(ε/2).

    상대 회전은 같아서 RPE 회전 0.
    """
    eps = 0.01
    g = line()
    e = se2.Traj2D(g.t, g.xy, g.yaw + eps)
    a = se2.ate(g, e)
    assert a['trans']['max'] == 0 and a['yaw_deg']['mean'] == pytest.approx(math.degrees(eps))
    r = se2.rpe(g, e, (1.0, 5.0))
    assert r[1.0]['trans']['mean'] == pytest.approx(2 * 1.0 * math.sin(eps / 2))
    assert r[5.0]['trans']['mean'] == pytest.approx(2 * 5.0 * math.sin(eps / 2))
    assert r[1.0]['rot_deg']['max'] == pytest.approx(0, abs=1e-9)


def test_rotation_about_origin_se2_alignment_recovers():
    """추정 = R(θ)·GT: 정렬 없음 ATE 는 ‖p‖·2 sin(θ/2), se2 정렬은 θ 를 되찾아 0."""
    th = math.radians(2.0)
    g = line()
    c, s = math.cos(th), math.sin(th)
    e = se2.Traj2D(g.t, g.xy @ np.array([[c, s], [-s, c]]), g.yaw + th)
    a = se2.ate(g, e)
    assert a['trans']['max'] == pytest.approx(np.linalg.norm(g.xy[-1]) * 2 * math.sin(th / 2))
    b = se2.ate(g, e, 'se2')
    assert b['trans']['max'] == pytest.approx(0, abs=1e-9)
    assert b['align'][2] == pytest.approx(-th)


def test_scale_drift_rpe_percent():
    """추정 위치 = 1.01 × GT (원점에서 x 축 직선): Δ = 1 m 쌍마다 병진 오차 0.01 m = 1 %."""
    g = line()
    e = se2.Traj2D(g.t, 1.01 * g.xy, g.yaw)
    r = se2.rpe(g, e, (1.0,))[1.0]
    assert r['trans']['mean'] == pytest.approx(0.01)
    assert r['trans_pct']['mean'] == pytest.approx(1.0)
    assert se2.ate(g, e)['trans']['max'] == pytest.approx(0.01 * 10.0)


def test_rte_constant_velocity_drift():
    """추정 x = GT x + 0.01 t: 10 s 창 상대 병진 오차 = 0.1 m."""
    g = line(n=601)
    e = se2.Traj2D(g.t, g.xy + np.stack([0.01 * g.t, 0 * g.t], 1), g.yaw)
    assert se2.rte(g, e, 10.0)['trans']['mean'] == pytest.approx(0.1)


def test_association_interpolates_and_respects_gap():
    """GT 50 Hz 직선, 추정은 표본 사이 시각 → 보간이 정확해 오차 0. GT 에 0.5 s 구멍 → 그 안 추정은 버린다."""
    g = line(n=501, dt=0.02)
    te = np.arange(0.011, 9.9, 0.1)
    e = se2.Traj2D(te, np.stack([te, 0 * te], 1), np.zeros(len(te)))
    a = se2.ate(g, e)
    assert a['trans']['max'] == pytest.approx(0, abs=1e-12) and a['n_used'] == len(te)
    hole = (g.t > 3.0) & (g.t < 3.5)
    g2 = se2.Traj2D(g.t[~hole], g.xy[~hole], g.yaw[~hole])
    in_hole = int(np.count_nonzero((te > 2.99) & (te < 3.51)))
    assert in_hole > 0
    assert se2.ate(g2, e)['n_used'] == len(te) - in_hole
    # GT 범위 밖 (외삽 금지)
    late = se2.Traj2D(np.array([20.0]), np.array([[20.0, 0.0]]), np.zeros(1))
    assert se2.ate(g, late)['n_used'] == 0


def test_yaw_wrap_in_error_and_interpolation():
    """π 근처: GT 3.13 → −3.13 (0.0232 rad 회전) 사이 중간의 보간 방위는 ±π, 오차는 작은 각."""
    g = se2.Traj2D([0.0, 0.1], [[0, 0], [0, 0]], [3.13, -3.13])
    e = se2.Traj2D([0.05], [[0, 0]], [math.pi - 1e-9])
    assert se2.ate(g, e)['yaw_deg']['max'] == pytest.approx(0, abs=1e-6)
    # t = 0: GT 3.13, 추정 −3.13 → 차이 −6.26 rad 를 감으면 2π − 6.26 = 0.0232 rad (358.7° 가 아니다)
    e2 = se2.Traj2D([0.0], [[0, 0]], [-3.13])
    assert se2.ate(g, e2)['yaw_deg']['max'] == pytest.approx(math.degrees(2 * math.pi - 6.26))


def test_consecutive_pairs_count():
    """10 m 직선, Δ = 1 m: 겹치지 않는 쌍 10 개, 모든 시작점 쌍은 더 많다."""
    g = line(n=201, dt=0.05)
    i, j = se2._pairs(se2.path_length(g.xy), 1.0, all_pairs=False)
    assert len(i) == 10 and np.all(i[1:] == j[:-1])
    i2, _ = se2._pairs(se2.path_length(g.xy), 1.0, all_pairs=True)
    assert len(i2) > len(i)


def test_umeyama_recovers_transform():
    rng = np.random.default_rng(3)
    src = rng.uniform(-5, 5, (50, 2))
    th, t = 0.3, np.array([1.5, -2.0])
    c, s = math.cos(th), math.sin(th)
    dst = src @ np.array([[c, s], [-s, c]]) + t
    tx, ty, th2 = se2.umeyama_se2(src, dst)
    assert (tx, ty, th2) == pytest.approx((1.5, -2.0, 0.3))
    with pytest.raises(ValueError):
        se2.umeyama_se2(src[:1], dst[:1])


def test_umeyama_never_returns_reflection():
    """
    거울상 자료(dst = (x, −y))에도 반환은 **회전**의 최적해다 (Umeyama 의 det 보정).

    det 보정이 없으면 SVD 가 반사 행렬을 주고, 그 각을 회전으로 쓰면 1° 격자로 찾은 최선 회전보다 잔차가 크다.
    """
    rng = np.random.default_rng(7)
    src = rng.uniform(-5, 5, (40, 2)) * [1.0, 0.2]
    dst = src * [1.0, -1.0]
    tx, ty, th = se2.umeyama_se2(src, dst)

    def residual(a, t=None):
        c, s = math.cos(a), math.sin(a)
        moved = src @ np.array([[c, s], [-s, c]])
        shift = dst.mean(0) - moved.mean(0) if t is None else t
        return float(np.sum((moved + shift - dst) ** 2))
    best = min(residual(math.radians(d)) for d in range(360))
    assert residual(th, np.array([tx, ty])) <= best + 1e-9


def test_ate_by_motion_thresholds():
    """정지 2 s → 직선 2 s → 제자리 회전 2 s. 오차: 정지 0.02, 직선 0.06, 회전 0.01 m."""
    dt = 0.02
    t = np.arange(0, 6.0, dt)
    x = np.where(t < 2, 0.0, np.where(t < 4, t - 2, 2.0))
    yaw = np.where(t < 4, 0.0, (t - 4) * 1.0)
    g = se2.Traj2D(t, np.stack([x, 0 * x], 1), yaw)
    err = np.where(t < 2, 0.02, np.where(t < 4, 0.06, 0.01))
    e = se2.Traj2D(t, np.stack([x, err], 1), yaw)
    r = se2.ate_by_motion(g, e)
    assert r['정지']['rmse_ok'] and r['회전']['max_ok']
    assert not r['직선']['rmse_ok'] and r['직선']['trans']['median'] == pytest.approx(0.06)


def test_tum_and_csv_readers(tmp_path):
    p = tmp_path / 'a.tum'
    yaw = 0.4
    p.write_text('# t x y z qx qy qz qw\n'
                 f'0.0 1 2 0.5 0 0 {math.sin(yaw / 2)} {math.cos(yaw / 2)}\n'
                 f'0.1 1.1 2 0.5 0 0 {math.sin(yaw / 2)} {math.cos(yaw / 2)}\n')
    tr = se2.tum_to_se2(str(p))
    assert tr.yaw == pytest.approx([yaw, yaw]) and tr.xy[1].tolist() == [1.1, 2.0]
    q = tmp_path / 'a.csv'
    q.write_text('stamp,x,y,theta\n0,0,0,0.1\n1,1,0,0.2\n')
    tc = se2.read_csv_se2(str(q))
    assert tc.t.tolist() == [0.0, 1.0] and tc.yaw.tolist() == [0.1, 0.2]
    with pytest.raises(ValueError):
        se2.Traj2D([0.0, 0.0], [[0, 0], [1, 0]], [0, 0])


def test_z_rp_diagnostics():
    t = np.arange(5) * 0.1
    q0 = np.tile([0, 0, 0, 1.0], (5, 1))
    a = math.radians(1.0)
    q1 = np.tile([math.sin(a / 2), 0, 0, math.cos(a / 2)], (5, 1))      # roll 1°
    xyz = np.zeros((5, 3))
    r = se2.z_rp_errors(t, xyz, q0, t, xyz + [0, 0, 0.02], q1)
    assert r['z']['mean'] == pytest.approx(0.02) and r['roll_deg']['mean'] == pytest.approx(1.0)
    assert r['pitch_deg']['max'] == pytest.approx(0, abs=1e-9)


# ---------------------------------------------------------------- 2D 지도
def grid(occ, res, ox, oy, known=None):
    occ = np.asarray(occ, dtype=bool)
    return gridmap.Grid(occ, np.ones_like(occ) if known is None else known, res, ox, oy)


def wall_gt(res=0.01, size=2.0):
    """GT: x ∈ [0, res) 한 열 점유 (벽 면 x = 0), 범위 [−1, 1)²."""
    n = int(round(size / res))
    occ = np.zeros((n, n), dtype=bool)
    occ[:, int(round(1.0 / res))] = True
    return grid(occ, res, -1.0, -1.0)


def test_grid_adnn_analytic():
    """
    GT 0.01 벽 열 중심 x = 0.005. 추정 0.05 셀 중심 x = 0.025 → 거리 0.02, 한 셀 더 → 0.07.

    재현율: GT 셀 → 최근접 추정 점(칠한 셀 중심 0.025) = 0.02 ≤ τ 0.05.
    """
    gt = wall_gt()
    occ = np.zeros((40, 40), dtype=bool)
    occ[:, 20] = True                      # 열 20 의 중심 x = −1 + 20.5 × 0.05 = 0.025
    est = grid(occ, 0.05, -1.0, -1.0)
    m = gridmap.grid_metrics(est, gt)
    assert m['adnn0'] == pytest.approx(0.02, abs=1e-9)
    assert m['precision_tau'] == 1.0 and m['recall_tau'] == 1.0 and m['f1_tau'] == 1.0
    # GT 셀 → 칠한 추정 점: x 거리 0.02, 행 거리는 5 행 주기로 2, 1, 0, 1, 2 셀
    period = [math.hypot(0.02, 0.01 * d) for d in (2, 1, 0, 1, 2)]
    assert m['gt_to_est_mean'] == pytest.approx(np.mean(period))
    assert m['chamfer0'] == pytest.approx(0.5 * (0.02 + np.mean(period)))
    occ2 = np.zeros((40, 40), dtype=bool)
    occ2[:, 21] = True
    m2 = gridmap.grid_metrics(grid(occ2, 0.05, -1.0, -1.0), gt)
    assert m2['adnn0'] == pytest.approx(0.07, abs=1e-9)
    assert m2['precision_tau'] == 0.0 and m2['recall_tau'] == 0.0 and m2['f1_tau'] == 0.0


def test_grid_surface_vs_filled_reference():
    """
    두께 0.10 m 벽(GT 0.01, x ∈ [0, 0.10)). 추정 A: 면 바로 앞(x = −0.025) 셀 열 — 면을 넘지 않은 지도.

    추정 B: 벽 **안쪽** x = 0.075 셀 열 (면 뒤로 7.5 cm 밀림). filled 기준 ADNN₀ 는 B 를 0 으로 보지만
    surface 기준은 가까운 면(x = 0.095 경계 셀 중심)까지 0.02 를 준다. 재현율은 surface 기준에서 A 가 1,
    filled 기준은 관측 여유 0.10 m 가 벽 내부 셀을 분모에 넣어 1 보다 작다.
    """
    occ = np.zeros((200, 200), dtype=bool)
    occ[:, 100:110] = True
    gt = grid(occ, 0.01, -1.0, -1.0)
    a = np.zeros((40, 40), dtype=bool)
    a[:, 19] = True                        # 중심 x = −1 + 19.5 × 0.05 = −0.025
    known = np.zeros((40, 40), dtype=bool)
    known[:, :20] = True                   # 벽 앞(x < 0)만 관측
    ma = gridmap.grid_metrics(grid(a, 0.05, -1.0, -1.0, known), gt)
    # 거리는 GT 셀 **중심**까지 (면 x = 0 이 아니라 첫 열 중심 x = 0.005) — 반 GT 셀 규약
    assert ma['adnn0'] == pytest.approx(0.03, abs=1e-9)
    assert ma['adnn0_surface'] == pytest.approx(0.03, abs=1e-9)
    # surface 관측 = 자유 셀(x < −0.05)에서 0.10 안 → 앞면 열(x = 0.005)만, 뒷면(x = 0.095)은 빠진다.
    # filled 관측 = 알려진 셀(x < 0)에서 0.10 안 → 벽 내부·뒷면 열까지 분모에 들어와 재현율이 낮다.
    assert ma['recall_tau'] > 0.95 and ma['recall_filled_tau'] < 0.8
    b = np.zeros((40, 40), dtype=bool)
    b[:, 21] = True                        # 중심 x = 0.075 (벽 안)
    mb = gridmap.grid_metrics(grid(b, 0.05, -1.0, -1.0), gt)
    assert mb['adnn0'] == 0.0
    assert mb['adnn0_surface'] == pytest.approx(0.02, abs=1e-9)


def test_grid_observed_mask_limits_recall():
    """GT 벽 두 개 (x = 0, y = 0.5). 추정은 y < 0 만 알고 그 안의 벽만 그렸다 → 재현율은 관측 영역 안에서만 센다."""
    gt = wall_gt()
    gt.occupied[150, :] = True            # y = −1 + 150.5 × 0.01 = 0.505
    occ = np.zeros((40, 40), dtype=bool)
    occ[:20, 20] = True                   # x = 0.025, y < 0
    known = np.zeros((40, 40), dtype=bool)
    known[:20, :] = True
    est = grid(occ, 0.05, -1.0, -1.0, known)
    m = gridmap.grid_metrics(est, gt, observed_margin=0.0)
    assert m['recall_filled_tau'] == pytest.approx(1.0) and m['n_gt_observed_filled'] == 100
    # 관측 영역 바깥까지 세면 (margin 이 큼) 재현율이 떨어진다
    m_all = gridmap.grid_metrics(est, gt, observed_margin=5.0)
    assert m_all['recall_filled_tau'] < 0.5


def test_iou_common_identical_and_shifted():
    gt = wall_gt()
    occ = np.zeros((40, 40), dtype=bool)
    occ[:, 20] = True
    est = grid(occ, 0.05, -1.0, -1.0)
    spec = gridmap.common_spec(0.05, (-1.0, 1.0, -1.0, 1.0))
    i = gridmap.iou_common(est, gt, spec)
    assert i['iou_tau'] == 1.0 and i['fp'] == 0 and i['fn'] == 0
    occ3 = np.zeros((40, 40), dtype=bool)
    occ3[:, 23] = True                    # 3 셀 밖 → τ = 1 셀 밖
    j = gridmap.iou_common(grid(occ3, 0.05, -1.0, -1.0), gt, spec)
    assert j['iou_tau'] == 0.0 and j['precision_tau'] == 0.0


def test_read_map_yaml_trinary(tmp_path):
    img = np.array([[0, 205, 254]], dtype=np.uint8)
    (tmp_path / 'm.pgm').write_bytes(b'P5\n3 1\n255\n' + img.tobytes())
    (tmp_path / 'm.yaml').write_text('image: m.pgm\nresolution: 0.05\norigin: [1.0, 2.0, 0]\n'
                                     'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.19\n')
    g = gridmap.read_map_yaml(tmp_path / 'm.yaml')
    assert g.occupied.tolist() == [[True, False, False]]
    assert g.known.tolist() == [[True, False, True]]
    assert (g.origin_x, g.origin_y, g.resolution) == (1.0, 2.0, 0.05)


# ---------------------------------------------------------------- 3D 점군
def plane(n=60, spacing=0.02, z=0.0):
    xs = (np.arange(n) + 0.5) * spacing
    x, y = np.meshgrid(xs, xs)
    return np.stack([x.ravel(), y.ravel(), np.full(x.size, z)], 1)


def test_cloud_identical_and_offsets():
    g = plane()
    m = cloud.cloud_metrics(g, g)
    assert m['fscore@0.05'] == 1.0 and m['chamfer_sum'] == 0.0
    m3 = cloud.cloud_metrics(g + [0, 0, 0.03], g)
    assert m3['acc_mean'] == pytest.approx(0.03) and m3['fscore@0.05'] == 1.0
    assert m3['ac@0.05'] == pytest.approx(0.03)
    assert m3['chamfer_sum'] == pytest.approx(0.06) and m3['chamfer_half'] == pytest.approx(0.03)
    m7 = cloud.cloud_metrics(g + [0, 0, 0.07], g)
    assert m7['precision@0.05'] == 0.0 and m7['fscore@0.05'] == 0.0
    assert m7['precision@0.1'] == 1.0 and math.isnan(m7['ac@0.05'])


def test_cloud_partial_coverage_fscore():
    """추정이 GT 의 왼쪽 절반만 (정확히) 덮음 → P = 1, R = 0.5 (경계 0.05 안 열 제외 근사 → 정확한 열 수로 계산)."""
    g = plane()
    est = g[g[:, 0] < 0.6]
    m = cloud.cloud_metrics(est, g, taus=(0.01,))
    # τ = 0.01 < 간격 0.02 → 덮인 열만 재현: 30/60
    assert m['precision@0.01'] == 1.0 and m['recall@0.01'] == pytest.approx(0.5)
    assert m['fscore@0.01'] == pytest.approx(2 * 0.5 / 1.5)
    assert m['com@0.01'] == m['recall@0.01']


def test_ghost_ratio_and_dynamic_scores():
    g = plane()
    far_in = np.tile([0.5, 0.5, 1.0], (10, 1))          # 동적 영역 안의 유령 점 10
    far_out = np.tile([5.0, 5.0, 1.0], (5, 1))          # 영역 밖이지만 설명 안 되는 점 5
    est = np.vstack([g, far_in, far_out])

    def region(xy):
        return xy[:, 0] < 2.0
    r = cloud.ghost_ratio(est, g, 0.05, region)
    assert r['unexplained'] == pytest.approx(15 / len(est))
    assert r['ghost'] == pytest.approx(10 / len(est))
    pts = np.vstack([g, far_in + np.arange(10)[:, None] * [0, 0, 0.5]])
    lab = np.r_[np.zeros(len(g), bool), np.ones(10, bool)]
    out = np.vstack([g, (far_in + np.arange(10)[:, None] * [0, 0, 0.5])[:5]])
    s = cloud.dynamic_scores(pts, lab, out, 0.01)
    assert s['SA'] == 1.0 and s['DA'] == 0.5 and s['AA'] == pytest.approx(math.sqrt(0.5))


def test_ply_round_trip_and_select(tmp_path):
    xyz = np.array([[0, 0, 0], [1, 2, 3], [5, 5, 0.5]], dtype=float)
    lab = np.array([0, 3, 3], dtype=np.uint8)
    cloud.write_ply(str(tmp_path / 'a.ply'), xyz, lab)
    a = cloud.read_ply(str(tmp_path / 'a.ply'))
    assert a.dtype.names == ('x', 'y', 'z', 'label') and len(a) == 3
    assert cloud.select(a, labels=[3]).tolist() == [[1, 2, 3], [5, 5, 0.5]]
    assert cloud.select(a, roi=(-1, 2, -1, 3), z=(0, 5)).tolist() == [[0, 0, 0], [1, 2, 3]]
    cloud.write_ply(str(tmp_path / 'b.ply'), xyz)
    assert cloud.read_ply(str(tmp_path / 'b.ply'), mmap=False).dtype.names == ('x', 'y', 'z')
    assert len(cloud.voxel_downsample(np.vstack([xyz, xyz + 0.01]), 0.5)) == 3


# ---------------------------------------------------------------- 결과 CSV
def common():
    return results.common_fields('SYN-1', 'SYN', 'b0', 0, 'rtabmap_3d', 'slam', 'abc', 'test',
                                 'none', {'sensor': '3d', 'wheel_prior': 'on'})


def test_results_round_trip_and_append(tmp_path):
    p = str(tmp_path / 'r.csv')
    st = se2.stats([0.1, 0.2, 0.3])
    rows = results.rows_from_stats(common(), 'ate_trans', st, 'm', segment='all')
    assert results.write_csv(p, rows) == 5
    assert results.write_csv(p, [results.value_row(common(), 'map_adnn0', 0.0259, 'm', 100)]) == 1
    back = results.read_csv(p)
    assert len(back) == 6 and back[0]['value'] == pytest.approx(st['rmse'])
    assert back[0]['factor_sensor'] == '3d' and back[0]['factor_dof'] == ''
    assert back[-1]['metric'] == 'map_adnn0' and back[-1]['n'] == 100


@pytest.mark.parametrize('bad', [
    {'alignment': 'sim3'}, {'scenario': 'S9'}, {'stage': 'mapping'}, {'factor_sensor': '2D'},
    {'unit': 'cm'}, {'bogus': 1}, {'run_id': ''}, {'value': float('nan')}, {'n': -1},
    {'value': 'abc'}, {'schema': 2},
])
def test_results_rejects(bad):
    row = results.value_row(common(), 'ate_trans', 0.1, 'm', 3)
    row.update(bad)
    with pytest.raises(results.SchemaError):
        results.validate(row)


def test_results_failed_row_may_be_nan(tmp_path):
    row = results.value_row(dict(common(), failed=1), 'ate_trans', float('nan'), 'm', 3)
    assert math.isnan(results.validate(row)['value'])
    empty = results.value_row(common(), 'loc_trans', float('nan'), 'm', 0)     # 표본 없음
    assert math.isnan(results.validate(empty)['value'])
    p = tmp_path / 'r.csv'
    p.write_text('a,b\n1,2\n')
    with pytest.raises(results.SchemaError):
        results.write_csv(str(p), [row])


# ---------------------------------------------------------------- CLI 끝-끝
def test_cli_end_to_end(tmp_path):
    g = line(n=301, dt=0.05)
    e = se2.Traj2D(g.t, g.xy + [0.02, 0.0], g.yaw)

    def tum(path, tr):
        with open(path, 'w') as f:
            for t, (x, y), yaw in zip(tr.t, tr.xy, tr.yaw):
                f.write(f'{t:.3f} {x} {y} 0 0 0 {math.sin(yaw / 2)} {math.cos(yaw / 2)}\n')
    tum(tmp_path / 'gt.tum', g)
    tum(tmp_path / 'est.tum', e)
    out = str(tmp_path / 'res.csv')
    meta = ['--out', out, '--run-id', 'SYN-a', '--scenario', 'SYN', '--bag-id', 'b0',
            '--row', 'test', '--stage', 'slam', '--git-head', 'abc', '--factor', 'sensor=3d']
    assert cli.main(['traj', '--gt', str(tmp_path / 'gt.tum'), '--est',
                     str(tmp_path / 'est.tum'), '--rpe-deltas', '1', '--by-motion'] + meta) == 0
    rows = results.read_csv(out)
    ate = [r for r in rows if r['metric'] == 'ate_trans' and r['stat'] == 'rmse']
    assert {r['alignment']: round(r['value'], 9) for r in ate} == {'none': 0.02, 'origin': 0.0,
                                                                   'se2': 0.0}
    # 지도: 위 grid 해석 시험과 같은 구성 → ADNN₀ 0.02
    gt = wall_gt()
    gridmap_dir = tmp_path / 'maps'
    gridmap_dir.mkdir()

    def save(gr, stem):
        img = np.where(gr.occupied, 0, 254).astype(np.uint8)[::-1]
        (gridmap_dir / f'{stem}.pgm').write_bytes(
            f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode() + img.tobytes())
        (gridmap_dir / f'{stem}.yaml').write_text(
            f'image: {stem}.pgm\nresolution: {gr.resolution}\n'
            f'origin: [{gr.origin_x}, {gr.origin_y}, 0]\nnegate: 0\noccupied_thresh: 0.65\n'
            'free_thresh: 0.19\n')
    occ = np.zeros((40, 40), dtype=bool)
    occ[:, 20] = True
    save(gt, 'gt')
    save(grid(occ, 0.05, -1.0, -1.0), 'est')
    assert cli.main(['grid', '--gt-map', str(gridmap_dir / 'gt.yaml'), '--est-map',
                     str(gridmap_dir / 'est.yaml'), '--gt-tag', 'wall'] + meta) == 0
    rows = results.read_csv(out)
    adnn = [r for r in rows if r['metric'] == 'map_adnn0' and r['stat'] == 'value']
    assert [r['param'].split(';')[-1] for r in adnn] == ['ref=filled', 'ref=surface']
    assert [r['value'] for r in adnn] == pytest.approx([0.02, 0.02], abs=1e-9)
    assert [r['value'] for r in rows if r['metric'] == 'map_iou'] == [1.0]
    # 점군
    cloud.write_ply(str(tmp_path / 'g.ply'), plane(), np.zeros(3600, np.uint8))
    cloud.write_ply(str(tmp_path / 'e.ply'), plane() + [0, 0, 0.03])
    assert cli.main(['cloud', '--gt-ply', str(tmp_path / 'g.ply'), '--est-ply',
                     str(tmp_path / 'e.ply'), '--gt-labels', '0'] + meta) == 0
    rows = results.read_csv(out)
    f = {r['param']: r['value'] for r in rows if r['metric'] == 'cloud_fscore'}
    assert set(f.values()) == {1.0} and len(f) == 2
    assert Path(out).read_text().count('\n') == len(rows) + 1
