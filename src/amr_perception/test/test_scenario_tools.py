"""기능 시험 도구(scripts/*_scenario.py, yolo_fps_probe.py)의 순수 함수 테스트 — 해석 TTC 와 레이캐스트."""

import importlib.util
import math
import os

import numpy as np
import pytest

SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts')


def _load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(SCRIPTS, name + '.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _Args:
    robot_radius = 0.361
    radius = 0.2
    x0 = 4.0
    y0 = -4.0
    vx = 0.0
    vy = 1.0


def test_tracker_scenario_analytic_ttc():
    mod = _load('tracker_scenario')
    r = _Args.robot_radius + _Args.radius
    # 로봇 (t, 0), 장애물 (4, -4 + t) → τ = 4 - t - R/√2 (tracking.md §5 폐형식)
    for t in (0.0, 1.0, 2.5):
        expected = 4.0 - t - r / math.sqrt(2)
        assert mod.analytic_ttc(t, _Args, 1.0) == pytest.approx(expected, abs=2e-3)
    assert math.isinf(mod.analytic_ttc(0.0, _Args, 0.0, horizon=2.0))   # 정지 로봇, 지평 안 충돌 없음


def test_tracker_scenario_raycast():
    mod = _load('tracker_scenario')
    angles, ranges = mod.raycast(np.zeros(2), 0.0, np.array([3.0, 0.0]), 0.5, noise=0.0)
    assert len(angles) == 720 and angles[360] == pytest.approx(0.0)
    assert ranges[360] == pytest.approx(2.5)                 # 정면 빔: 원 표면까지
    assert np.isinf(ranges[0])                               # 후방 빔: 없음
    _, noisy = mod.raycast(np.zeros(2), 0.0, np.array([3.0, 0.0]), 0.5, noise=0.03,
                           rng=np.random.default_rng(0))
    assert abs(noisy[360] - 2.5) < 0.15


def test_other_tools_import():
    for name in ('safety_scenario', 'yolo_fps_probe'):
        mod = _load(name)
        assert callable(mod.main)


GZ_SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'scripts')


def _load_gz(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(GZ_SCRIPTS, name + '.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_gz_trials_geometry():
    """Gazebo 시험기의 지면 진실 거리: 풋프린트(0.60 × 0.40) ↔ 원·상자."""
    mod = _load_gz('safety_gz_trials')
    robot = (0.0, 0.0, 0.0)
    # 전면 모서리 x = 0.30 → 반경 0.2 원 중심 (1.0, 0) 까지 0.5
    assert mod.footprint_distance(robot, ('circle', 0.2), (1.0, 0.0, 0.0)) == pytest.approx(0.5)
    assert mod.footprint_distance(robot, ('circle', 0.2), (0.1, 0.0, 0.0)) == 0.0
    # 상자 0.4 × 0.4 중심 (1.0, 0.5): 가까운 모서리 (0.8, 0.3) ↔ 풋프린트 꼭짓점 (0.3, 0.2)
    d = mod.footprint_distance(robot, ('box', 0.4, 0.4), (1.0, 0.5, 0.0))
    assert d == pytest.approx(math.hypot(0.5, 0.1))
    # 겹침 = 0, 90° 돌린 로봇은 폭 방향이 x
    assert mod.footprint_distance(robot, ('box', 0.4, 0.4), (0.3, 0.0, 0.0)) == 0.0
    turned = mod.footprint_distance((0.0, 0.0, math.pi / 2), ('circle', 0.1), (0.5, 0.0, 0.0))
    assert turned == pytest.approx(0.2)
    assert mod.braking_tolerance(0.2) == pytest.approx(0.2 * 0.25 + 0.02)
    assert mod.percentile([3.0, 1.0, 2.0], 50) == pytest.approx(2.0)
    assert mod.percentile([], 50) is None


def test_gz_test_world_strips_dynamic_obstacles(tmp_path):
    mod = _load_gz('safety_test_world')
    src = """<sdf version="1.8">
  <world name="warehouse">
    <actor name="worker_a">
      <skin/>
    </actor>
    <!-- forklift_main: 지게차 -->
    <include>
      <name>forklift_main</name>
      <plugin/>
    </include>
    <!-- shuttle_amr: 셔틀 -->
    <include>
      <name>shuttle_amr</name>
    </include>
    <model name="rack"/>
  </world>
</sdf>
"""
    out = mod.build(src)
    assert '<actor' not in out and 'forklift_main' not in out and 'shuttle_amr' not in out
    assert '<model name="rack"/>' in out
    for name in ('obs_person', 'obs_box_low', 'obs_forklift'):
        assert name in out
    assert out.count('VelocityControl') == 2 and out.index('obs_forklift') < out.index('</world>')
    src_file = tmp_path / 'in.sdf'
    dst_file = tmp_path / 'out.sdf'
    src_file.write_text(src, encoding='utf-8')
    assert mod.main([str(src_file), str(dst_file)]) == 0
    assert 'obs_person' in dst_file.read_text(encoding='utf-8')
    assert mod.main([]) == 2
