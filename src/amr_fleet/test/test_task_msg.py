"""
task_msg: TaskSpec ↔ amr_msgs/Task 왕복 변환 (amr_msgs 가 빌드된 환경에서만 실행).

주의: 모듈 수준 pytest.importorskip 은 쓰지 않는다. 이미지의 launch_testing pytest 플러그인이
수집 단계에서 디렉토리의 모든 모듈을 import 하므로, 한 모듈의 Skipped 예외가 디렉토리 전체
수집을 중단시킨다(0 tests, 실패도 아님). skipif 마커는 실행 단계에서 평가되어 안전하다.
"""

import math

import pytest

from amr_fleet.task_schema import (
    STATUS_IN_PROGRESS, STATUS_PENDING, Pose2D, TaskSpec,
)

try:
    from amr_fleet import task_msg as tm
    from amr_msgs.msg import Task
    from builtin_interfaces.msg import Time
    HAVE_MSGS = True
except ImportError:
    HAVE_MSGS = False

pytestmark = pytest.mark.skipif(not HAVE_MSGS, reason='amr_msgs 가 빌드되지 않은 환경')


def make_spec(**kw):
    base = dict(task_id='t_42', pickup=Pose2D(1.5, -2.0, 0.7, 'map'),
                dropoff=Pose2D(30.25, 12.0, -2.9, 'map'), item_type='medium', item_mass=10.0,
                priority=200, deadline=1_700_000_123.456, robot_id='amr_03',
                created=1_700_000_000.25, status=STATUS_IN_PROGRESS)
    base.update(kw)
    return TaskSpec(**base)


def test_time_conversion_round_trip_and_zero():
    t = tm.float_to_time(1_700_000_000.123456789)
    assert t.sec == 1_700_000_000
    assert abs(t.nanosec - 123456789) < 1000        # float64 분해능(≈ 2e-7 s) 안에서 일치
    assert tm.time_to_float(t) == pytest.approx(1_700_000_000.123456789, abs=1e-6)
    assert tm.time_to_float(tm.float_to_time(None)) is None
    assert tm.time_to_float(tm.float_to_time(-1.0)) is None
    assert tm.time_to_float(Time(sec=0, nanosec=0)) is None
    # 반올림이 1 s 를 넘기는 경계
    t = tm.float_to_time(5.9999999999)
    assert (t.sec, t.nanosec) == (6, 0)


@pytest.mark.parametrize('yaw', [0.0, 0.7, -2.9, math.pi - 1e-6, -math.pi / 2])
def test_yaw_quaternion_round_trip(yaw):
    assert tm.quaternion_to_yaw(*tm.yaw_to_quaternion(yaw)) == pytest.approx(yaw, abs=1e-9)


def test_spec_to_msg_fields():
    spec = make_spec()
    msg = tm.spec_to_msg(spec)
    assert isinstance(msg, Task)
    assert msg.task_id == 't_42' and msg.robot_id == 'amr_03' and msg.priority == 200
    assert msg.status == Task.STATUS_IN_PROGRESS == STATUS_IN_PROGRESS
    assert msg.item_type == 'medium' and msg.item_mass == pytest.approx(10.0)
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1_700_000_000, 250_000_000)
    assert msg.deadline.sec == 1_700_000_123
    assert msg.pickup_pose.header.frame_id == 'map'
    assert msg.pickup_pose.pose.position.x == 1.5
    assert msg.dropoff_pose.pose.position.y == 12.0
    # 우선순위는 0~255 로 잘라낸다
    assert tm.spec_to_msg(make_spec(priority=999)).priority == 255


def test_round_trip_spec_msg_spec():
    spec = make_spec()
    back = tm.spec_from_msg(tm.spec_to_msg(spec), now=0.0)
    assert back.task_id == spec.task_id and back.robot_id == spec.robot_id
    assert back.priority == spec.priority and back.status == spec.status
    assert back.item_type == spec.item_type and back.item_mass == pytest.approx(10.0)
    assert back.deadline == pytest.approx(spec.deadline, abs=1e-6)
    assert back.created == pytest.approx(spec.created, abs=1e-6)
    for a, b in ((back.pickup, spec.pickup), (back.dropoff, spec.dropoff)):
        assert (a.x, a.y, a.frame_id) == (b.x, b.y, b.frame_id)
        assert a.yaw == pytest.approx(b.yaw, abs=1e-9)


def test_round_trip_msg_spec_msg_with_no_deadline():
    spec = make_spec(deadline=None, robot_id='', status=STATUS_PENDING)
    msg1 = tm.spec_to_msg(spec)
    assert (msg1.deadline.sec, msg1.deadline.nanosec) == (0, 0)
    msg2 = tm.spec_to_msg(tm.spec_from_msg(msg1, now=0.0))
    assert msg2 == msg1


def test_spec_from_msg_generates_id_and_mass_and_created():
    msg = Task()
    msg.item_type = 'large'
    msg.pickup_pose.pose.orientation.w = 1.0
    msg.dropoff_pose.pose.orientation.w = 1.0
    spec = tm.spec_from_msg(msg, now=99.0, id_factory=lambda: 'gen_1')
    assert spec.task_id == 'gen_1' and spec.item_mass == 25.0 and spec.created == 99.0
    assert spec.deadline is None and spec.pickup.frame_id == 'map'
    spec2 = tm.spec_from_msg(msg, now=99.0, item_masses={'large': 7.0})
    assert spec2.item_mass == 7.0
    msg.item_type = 'unknown'
    assert tm.spec_from_msg(msg, now=0.0).item_mass == 0.0


def test_pose_to_msg_stamp_and_default_frame():
    msg = tm.pose_to_msg(Pose2D(1, 2, 0, ''), stamp=Time(sec=3))
    assert msg.header.stamp.sec == 3
    assert tm.pose_from_msg(msg).frame_id == 'map'
