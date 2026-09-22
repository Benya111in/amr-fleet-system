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
