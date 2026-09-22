"""worldmap(visual 단면·정합) · docks · metrics(TTC·이탈 구간) · rates(명세 주기·깊이)."""

import math
from pathlib import Path

from amr_itest import docks, metrics, rates, worldmap
import numpy as np
import pytest

WORLD = """<?xml version="1.0"?>
<sdf version="1.8"><world name="w">
  <model name="ground_plane"><static>true</static>
    <link name="l"><visual name="v"><geometry><box><size>100 100 0.4</size></box></geometry>
    </visual></link></model>
  <model name="walls"><static>true</static>
    <link name="l">
      <visual name="v"><pose>0 5 1 0 0 0</pose><geometry><box><size>10 0.2 2</size></box>
      </geometry></visual>
      <collision name="c"><pose>0 5 1 0 0 0</pose><geometry><box><size>10 0.4 2</size></box>
      </geometry></collision>
      <visual name="v2"><pose>0 -5 1 0 0 0</pose><geometry><box><size>10 0.2 2</size></box>
      </geometry></visual>
      <visual name="v3"><pose>5 0 1 0 0 0</pose><geometry><box><size>0.2 10 2</size></box>
      </geometry></visual>
      <visual name="v4"><pose>-5 0 1 0 0 0</pose><geometry><box><size>0.2 10 2</size></box>
      </geometry></visual></link></model>
  <model name="marker"><static>true</static><pose>-4.9 2 0 0 0 0</pose>
    <link name="link"><collision name="c"><pose>0 0 0.25 0 0 0</pose>
      <geometry><box><size>0.02 0.30 0.30</size></box></geometry></collision></link></model>
  <include><uri>model://crate</uri><name>crate_1</name><pose>2 2 0 0 0 0</pose></include>
  <include><uri>model://cart</uri><name>forklift</name><pose>-2 -2 0 0 0 0</pose>
    <plugin filename="diff-drive" name="dd"/></include>
</world></sdf>
"""
CRATE = """<?xml version="1.0"?>
<sdf version="1.8"><model name="crate"><link name="l"><pose>0 0 0.15 0 0 0</pose>
  <visual name="v"><geometry><box><size>0.5 0.4 0.3</size></box></geometry></visual>
  </link></model></sdf>
"""


@pytest.fixture
def world(tmp_path):
    for name in ('crate', 'cart'):
        (tmp_path / 'models' / name).mkdir(parents=True)
        (tmp_path / 'models' / name / 'model.sdf').write_text(CRATE)
    (tmp_path / 'w.sdf').write_text(WORLD)
    return tmp_path


def test_footprints_visual_collision_movers(world):
    """gpu_lidar 는 visual 을 잰다: 지도 비교는 visual 단면, 움직이는 모델(<plugin>)·지면은 뺀다."""
    vis = worldmap.footprints(world / 'w.sdf', world / 'models', 0.20)
    assert len(vis) == 5                         # 벽 visual 4 + 물리 상자 (지면·지게차 제외)
    assert not any(isinstance(s, worldmap.Rect) and s.sx >= 100 for s in vis)
    col = worldmap.footprints(world / 'w.sdf', world / 'models', 0.20, 'collision')
    assert sorted(round(s.sy, 2) for s in col) == [0.3, 0.4]      # 벽 충돌체 + 마커 판
    both = worldmap.footprints(world / 'w.sdf', world / 'models', 0.20, 'both')
    assert len(both) == len(vis) + len(col)
    static = worldmap.footprints(world / 'w.sdf', world / 'models', 0.20, static_only=True)
    assert len(static) == 4                      # 물리 상자(비정적) 제외
    with pytest.raises(ValueError):
        worldmap.footprints(world / 'w.sdf', world / 'models', 0.20, 'mesh')
    assert worldmap.box_size(world / 'w.sdf', 'marker') == pytest.approx((0.02, 0.30, 0.30))
    assert worldmap.box_size(world / 'w.sdf', 'nope') is None


def _room_points(res=0.05):
    """방 + 기둥 + 기운 상자, 지도 점유 = 구조물 면 (LiDAR 가 보는 가장자리, SLAM 지도처럼)."""
    shapes = [worldmap.Rect(0, 5, 0, 10, 0.2), worldmap.Rect(0, -5, 0, 10, 0.2),
              worldmap.Rect(5, 0, 0, 0.2, 10), worldmap.Rect(-5, 0, 0, 0.2, 10),
              worldmap.Circle(2.0, 1.0, 0.25), worldmap.Rect(-2, -2, 0.3, 1.0, 0.5)]
    pts = np.vstack([s.perimeter(res) for s in shapes])
    ix = np.floor((pts[:, 0] + 6.0) / res).astype(int)
    iy = np.floor((pts[:, 1] + 6.0) / res).astype(int)
    grid = np.zeros((int(12 / res), int(12 / res)), dtype=int)
    grid[iy, ix] = 100
    return shapes, worldmap.occupied_points(grid, res, (-6.0, -6.0))


def test_register_identity_and_offsets():
    """지도 = 월드 가정을 조용히 믿지 않는다: 항등이면 통과, 이동·회전·16 m 어긋남이면 실패."""
    shapes, pts = _room_points()
    ident = worldmap.register(pts, shapes)
    assert ident.identity_ok(), ident.as_dict()
    # 지도 점유 = 셀 중심이라 면 위치는 ± 반 셀(2.5 cm) 양자화된다 — 항등 판정 5 cm 는 그보다 넉넉하다
    assert ident.translation < 0.035 and abs(math.degrees(ident.dyaw)) < 0.05
    shifted = worldmap.register(pts + np.array([0.12, -0.05]), shapes)
    assert not shifted.identity_ok()
    assert shifted.dx == pytest.approx(-0.12, abs=0.03)
    assert shifted.dy == pytest.approx(0.05, abs=0.03)
    c, s = math.cos(0.01), math.sin(0.01)
    rotated = worldmap.register(pts @ np.array([[c, s], [-s, c]]), shapes)
    assert not rotated.identity_ok() and abs(math.degrees(rotated.dyaw)) > 0.3
    far = worldmap.register(pts + np.array([16.0, 2.0]), shapes)
    assert not far.identity_ok() and far.median_identity > 1.0
    empty = worldmap.register(np.zeros((0, 2)), shapes)
    assert empty.points == 0 and not empty.identity_ok()
    assert set(ident.as_dict()) >= {'dx_m', 'dy_m', 'dyaw_deg', 'median_identity_m', 'points'}


def test_occupied_points_cell_centers():
    grid = np.zeros((4, 5), dtype=int)
    grid[1, 3] = 100
    grid[2, 0] = -1
    pts = worldmap.occupied_points(grid, 0.5, (10.0, 20.0))
    assert pts.tolist() == [[11.75, 20.75]]


def test_docks_table_and_docked_pose(tmp_path):
    repo = Path(__file__).resolve().parents[3]
    table = docks.load_docks(repo / 'src' / 'amr_behavior' / 'config' / 'behavior.yaml')
    assert {'dock_1', 'dock_2', 'dock_a', 'dock_b'} <= set(table)
    assert table['dock_1'].staging[2] == pytest.approx(math.pi)
    assert table['dock_a'].standoff > 0 and table['dock_1'].perceive_yaw is not None
    # 마커 +x = 판 바깥 법선 (계약 C3): 입고 마커(방위 0) 는 +x 로, 출고 마커(방위 π) 는 −x 로 물러난다
    x, y, yaw = docks.docked_pose((-29.99, 17.0, 0.0), 0.65)
    assert (x, y) == pytest.approx((-29.33, 17.0)) and abs(yaw) == pytest.approx(math.pi)
    x, y, yaw = docks.docked_pose((29.99, 13.0, math.pi), 0.65)
    assert (x, y) == pytest.approx((29.33, 13.0)) and yaw == pytest.approx(0.0, abs=1e-9)
    err, ang = docks.pose_error((1.0, 2.0, math.pi - 0.01), (1.0, 2.03, -math.pi + 0.01))
    assert err == pytest.approx(0.03) and ang == pytest.approx(0.02)
    y = tmp_path / 'b.yaml'
    y.write_text('/**:\n  ros__parameters:\n    docks:\n      ids: [d]\n'
                 '      d: {staging: [1, 2, 0.5], standoff: 0.5}\n')
    d = docks.load_docks(y)['d']
    assert d.staging == (1.0, 2.0, 0.5) and d.marker_id == -1 and d.perceive_yaw is None


def test_ttc_circle():
    assert metrics.ttc_circle(5.0, 0.0, -1.0, 0.0, 1.0) == pytest.approx(4.0)   # 정면 접근
    assert metrics.ttc_circle(0.5, 0.0, 1.0, 0.0, 1.0) == 0.0                    # 이미 안
    assert math.isinf(metrics.ttc_circle(5.0, 3.0, -1.0, 0.0, 1.0))            # 비켜 감
    assert math.isinf(metrics.ttc_circle(5.0, 0.0, 1.0, 0.0, 1.0))             # 멀어짐
    assert math.isinf(metrics.ttc_circle(5.0, 0.0, 0.0, 0.0, 1.0))             # 정지


def test_deviation_episodes_return_time():
    """명세 4.7 원경로 복귀: 이탈 > 0.3 m 구간의 최대 이탈 → 0.15 m 이하 복귀 시간."""
    t = [0, 1, 2, 3, 4, 5, 6, 7]
    d = [0.0, 0.4, 0.6, 0.2, 0.1, 0.5, 0.4, 0.35]
    eps = metrics.deviation_episodes(t, d)
    assert len(eps) == 2
    assert (eps[0].t_start, eps[0].t_peak, eps[0].peak, eps[0].t_end) == (1, 2, 0.6, 4)
    assert eps[0].returned and eps[0].return_time(7) == 2
    assert not eps[1].returned and eps[1].return_time(7) == 2
    assert metrics.deviation_episodes([0, 1], [0.1, float('nan')]) == []


def test_rates_spec_minimum_and_tolerance():
    """기준 주기 = max(설정, 명세): 설정을 낮춰도 기준이 내려가지 않고, 9 Hz LiDAR 는 탈락."""
    assert rates.nominal_rate(8.0, 10.0) == 10.0 and rates.nominal_rate(30.0, 15.0) == 30.0
    nine = rates.rate_stats([i / 9.0 for i in range(50)])
    assert not nine.meets(rates.nominal_rate(10.0, 10.0))       # 예전 기준(0.9× 평균)으로는 통과
    assert nine.meets(10.0, mean_tol=0.85, median_tol=0.85)
    ten = rates.rate_stats([i / 10.0 for i in range(50)])
    assert ten.meets(10.0)
    drops = rates.rate_stats([i / 10.0 for i in range(50) if i % 10])   # 구독자 쪽 10 % 드롭
    assert drops.median_rate == pytest.approx(10.0) and drops.meets(10.0)


def test_depth_noise_and_range():
    rng = np.random.default_rng(0)
    base = np.full((20, 30), 3.0)
    base[0, :] = np.inf                                          # 최대 거리 밖
    frames = [base + rng.normal(0.0, 0.005, base.shape) for _ in range(12)]
    assert rates.per_pixel_noise(frames, 10.0) == pytest.approx(0.005, rel=0.2)
    assert rates.finite_max(frames) < 3.1
    assert math.isnan(rates.per_pixel_noise(frames[:1], 10.0))
    assert math.isnan(rates.per_pixel_noise([np.full((2, 2), np.inf)] * 3, 10.0))
    assert math.isnan(rates.finite_max([np.zeros((2, 2))]))
