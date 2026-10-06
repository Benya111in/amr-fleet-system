"""IMU 방위 비교 시험 (amr_slam3d.imustat) — 닫힌 식 기대값."""

import math

from amr_slam3d import imustat
import numpy as np
import pytest


def _q(roll=0.0, pitch=0.0, yaw=0.0):
    """ZYX 오일러 → (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]


def test_quat_to_rpy_roundtrip():
    rpy = [(0.1, -0.2, 3.0), (0.0, 0.0, -3.1), (-0.5, 0.3, 0.0)]
    got = imustat.quat_to_rpy(np.array([_q(*a) for a in rpy]))
    assert got == pytest.approx(np.array(rpy), abs=1e-12)


def test_rpy_error_wraps_and_is_signed():
    e = imustat.rpy_error_deg(np.array([_q(yaw=math.radians(179.0))]),
                              np.array([_q(yaw=math.radians(-179.0))]))
    assert e[0, 2] == pytest.approx(-2.0)                  # 감아서 −2°, 358° 아님
    e2 = imustat.rpy_error_deg(np.array([_q(pitch=math.radians(1.5))]), np.array([_q()]))
    assert e2[0] == pytest.approx([0.0, 1.5, 0.0], abs=1e-9)


def test_stats_and_empty():
    s = imustat.stats([3.0, -4.0])
    assert s['rmse'] == pytest.approx(math.sqrt(12.5)) and s['max'] == 4.0 and s['n'] == 2
    assert imustat.stats([])['n'] == 0 and math.isnan(imustat.stats([])['rmse'])


def test_quat_diff_and_angle_ignore_sign():
    a = np.array([_q(yaw=0.3)])
    assert imustat.quat_diff(a, -a)[0] == pytest.approx(0.0)     # q 와 −q 는 같은 회전
    assert imustat.quat_angle_deg(a, -a)[0] == pytest.approx(0.0, abs=1e-6)
    b = np.array([_q(yaw=0.3 + math.radians(2.0))])
    assert imustat.quat_angle_deg(a, b)[0] == pytest.approx(2.0, abs=1e-6)
    assert imustat.quat_diff(a, b)[0] > 0
    with pytest.raises(ValueError):
        imustat.quat_diff(a, np.vstack([a, a]))
    with pytest.raises(ValueError):
        imustat.quat_angle_deg(a, np.vstack([a, a]))


def test_pair_by_stamp_unsorted_and_tolerance():
    ta = [0.0, 0.1, 0.2, 0.3]
    tb = [0.3, 0.0, 0.25, 0.1 + 5e-7]
    ia, ib = imustat.pair_by_stamp(ta, tb)
    assert list(ia) == [0, 1, 3] and list(ib) == [1, 3, 0]
    ia, ib = imustat.pair_by_stamp(ta, [])
    assert len(ia) == 0 and len(ib) == 0
