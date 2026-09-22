import math
from types import SimpleNamespace

from amr_evaluation import msg_adapters
import pytest


def _header(sec=0, nanosec=0):
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))


def _pose(x, y, yaw):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))


def _odom(t, x, y, yaw, vx, vy, wz):
    sec = int(t)
    return SimpleNamespace(
        header=_header(sec, int(round((t - sec) * 1e9))),
        pose=SimpleNamespace(pose=_pose(x, y, yaw)),
        twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=vx, y=vy, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=wz))))


def test_stamp_of_with_fallback():
    assert msg_adapters.stamp_of(_header(3, 250_000_000)) == pytest.approx(3.25)
    assert msg_adapters.stamp_of(_header(), fallback=7.5) == 7.5
    assert msg_adapters.stamp_of(_header()) == 0.0


def test_odom_to_sample():
    s = msg_adapters.odom_to_sample(_odom(12.5, 1.0, 2.0, 0.7, 0.3, 0.4, -0.2))
    assert s.t == pytest.approx(12.5)
    assert (s.x, s.y) == (1.0, 2.0)
    assert s.yaw == pytest.approx(0.7)
    assert s.v == pytest.approx(0.5)
    assert s.w == -0.2
    zero = msg_adapters.odom_to_sample(_odom(0.0, 0, 0, 0, 0, 0, 0), fallback_time=99.0)
    assert zero.t == 99.0


def test_pose_stamped_to_sample():
    msg = SimpleNamespace(header=_header(1, 0), pose=_pose(3.0, -1.0, -1.2))
    s = msg_adapters.pose_stamped_to_sample(msg)
    assert (s.t, s.x, s.y) == (1.0, 3.0, -1.0)
    assert s.yaw == pytest.approx(-1.2)
    assert (s.v, s.w) == (0.0, 0.0)


def test_path_to_points():
    msg = SimpleNamespace(poses=[SimpleNamespace(pose=_pose(i, 2 * i, 0.0)) for i in range(4)])
    pts = msg_adapters.path_to_points(msg)
    assert pts.shape == (4, 2)
    assert pts[3].tolist() == [3.0, 6.0]
    assert msg_adapters.path_to_points(SimpleNamespace(poses=[])).shape == (0, 2)


def test_twist_to_velocity():
    msg = SimpleNamespace(linear=SimpleNamespace(x=0.3, y=-0.4, z=0.0),
                          angular=SimpleNamespace(x=0.0, y=0.0, z=0.25))
    assert msg_adapters.twist_to_velocity(msg) == pytest.approx((0.5, 0.25))
