"""보충 분석 (psupp.py — EKF 결함 bag 판정, odom_drift.py — 표류율) 시험."""

import importlib.util
import math
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]


def _load(name):
    spec = importlib.util.spec_from_file_location(name, PKG / 'scripts' / f'{name}.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ps = _load('psupp')
od = _load('odom_drift')


def test_bias_line_states(tmp_path):
    log = tmp_path / 'launch.log'
    log.write_text('[imu_filter_node-15] [WARN] [1] [amr_01.imu_filter_node]: motion detected '
                   'during startup bias estimation; falling back to parameter bias\n',
                   encoding='utf-8')
    assert ps.bias_line(str(log))[0] == 'fallback'
    log.write_text('[imu_filter_node-15] [INFO] [1] [x]: startup bias estimated from 12001 '
                   'samples: gyro_bias [0.01, 0.01, -0.01]\n', encoding='utf-8')
    assert ps.bias_line(str(log))[0] == 'estimated'
    log.write_text('[imu_filter_node-15] [INFO] [1] [x]: motion detected during startup bias '
                   'estimation; using 9015 samples collected before motion: gyro_bias [0, 0, 0]\n',
                   encoding='utf-8')
    assert ps.bias_line(str(log))[0] == 'partial'
    assert ps.bias_line(str(tmp_path / 'none.log'))[0] == 'none'


def test_clean_sets_exclude_defect_bags():
    assert not set(ps.CLEAN_E2) & set(ps.DEFECT_BAGS)
    assert len(ps.CLEAN_E2) == 5 and len(ps.CLEAN_E3) == 4
    assert 'map_long_a1' not in ps.CLEAN_E3


class _T:
    def __init__(self, t, xy, yaw):
        self.t, self.xy, self.yaw = t, xy, yaw


def test_drift_recovers_constant_yaw_rate():
    t = np.linspace(0.0, 100.0, 1001)
    gt = _T(t, np.zeros((len(t), 2)), np.zeros(len(t)))
    # 정지한 로봇, odom yaw 만 0.01 rad/s 로 표류 → 표류율 0.01, xy 오차 0
    r = od.drift(t, np.zeros((len(t), 2)), 0.01 * t, gt)
    assert math.isclose(r['yaw_drift_rate_rad_s'], 0.01, rel_tol=1e-6)
    assert r['err_max_cm'] < 1e-9
    assert math.isclose(r['yaw_err_max_deg'], math.degrees(1.0), rel_tol=1e-6)
