"""probe · actions · cases: rclpy 루프백 (한 프로세스 안에서 발행 → 구독)."""

from types import SimpleNamespace as NS
import unittest

from amr_itest import actions, cases, catalog
from amr_itest.probe import GraphProbe, stamp_seconds, TopicRecord
from amr_itest.scenario import Context
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
import pytest
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger


@pytest.fixture
def probe():
    p = GraphProbe('unit_probe', namespace='/itest_unit')
    yield p
    p.close()


def _pump(p, topic, msg, rec, n=1, timeout=5.0):
    """디스커버리 전 유실을 피하려고 받을 때까지 반복 발행."""
    return p.wait_until(lambda: p.publish(topic, msg) and rec.count >= n, timeout, 0.05)


def test_topics_and_raw_header(probe):
    rec = probe.subscribe('chatter', String, keep_messages=10)
    assert probe.subscribe('chatter', String) is rec
    assert _pump(probe, 'chatter', String(data='hi'), rec, 3)
    assert rec.last().data == 'hi' and rec.last_recv() is not None
    assert rec.count_since(0.0) >= 3 and len(rec.messages(0.0)) >= 3
    assert rec.stamps() == [] and len(rec.recv_times()) >= 3
    assert probe.record('chatter') is rec
    assert probe.matching_qos('chatter') == 'reliable'
    assert probe.matching_qos('no_pub') == 'sensor'
    assert probe.publisher_count('chatter') == 1 and probe.wait_for_publisher('chatter', 1.0)
    assert probe.wait_for_messages(rec, 1, 1.0)
    rec.clear()
    assert rec.last() is None and rec.last_recv() is None
    raw = probe.subscribe('hdr', Header, 'sensor', raw=True)
    assert _pump(probe, 'hdr', Header(stamp=Time(sec=5, nanosec=250000000), frame_id='f'), raw)
    assert raw.stamps()[0] == pytest.approx(5.25) and raw.frame_id == 'f' and raw.last() is None
    assert stamp_seconds(Header(stamp=Time(sec=2))) is None
    assert stamp_seconds(NS(header=NS(stamp=Time(sec=2)))) == 2.0
    assert stamp_seconds(NS(clock=Time(sec=3))) == 3.0
    assert '/itest_unit/unit_probe' in probe.node_names()
    assert probe.wait_for_node('/itest_unit/unit_probe', 2.0)
    assert probe.now() > 0 and probe.sleep_ros(0.05, 1.0)
    assert not GraphProbe.wait_until(lambda: False, 0.05)


def test_service_and_tf(probe):
    probe.node.create_service(Trigger, 'svc', lambda req, res: Trigger.Response(
        success=True, message='ok'))
    assert probe.service_available(Trigger, 'svc', 5.0)
    res = probe.call(Trigger, 'svc', Trigger.Request(), 5.0)
    assert res.success and res.message == 'ok'
    assert probe.call(Trigger, 'nobody', Trigger.Request(), 0.2) is None
    assert not probe.service_available(Trigger, 'nobody2', 0.1)
    assert probe.tf_snapshot().edges == []
    graph = probe.capture_tf()
    assert probe.capture_tf() is graph
    buf = probe.tf_buffer()
    assert probe.tf_buffer() is buf
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    tr = TransformStamped()
    tr.header.frame_id, tr.child_frame_id = 'base_link', 'lidar_link'
    tr.transform.translation.x = 0.15
    tr.transform.rotation.w = 1.0
    StaticTransformBroadcaster(probe.node).sendTransform([tr])
    assert probe.wait_until(lambda: probe.tf_snapshot().edge('base_link', 'lidar_link'), 5.0)
    assert probe.tf_snapshot().edge('base_link', 'lidar_link').translation[0] == 0.15


def _odom(v, w=0.0):
    return NS(twist=NS(twist=NS(linear=NS(x=v, y=0.0), angular=NS(z=w))),
              header=NS(stamp=NS(sec=0, nanosec=0)),
              pose=NS(pose=NS(position=NS(x=0.0, y=0.0),
                              orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0))))


def test_action_helpers_pure(probe):
    assert actions.is_zero(actions.twist()) and not actions.is_zero(actions.twist(0.1))
    assert actions.odom_speed(_odom(0.3, 0.2)) == (0.3, 0.2)
    ps = actions.pose_stamped(1.0, 2.0, 0.0, 'map')
    assert ps.header.frame_id == 'map' and ps.pose.orientation.w == 1.0
    js = actions.obstacles_json(world_boxes=[(0, 0, 1, 1)], robot_circles=[(1, 0, 0.2)]).data
    assert '"boxes": [[0, 0, 1, 1]]' in js
    rec = TopicRecord('gt')
    for t, v in ((1.0, 0.5), (2.0, 0.0), (3.0, 0.0)):
        rec.add(t, None, _odom(v))
    assert actions.first_after(rec, 1.5, lambda m: m.twist.twist.linear.x == 0.0)[0] == 2.0
    assert actions.first_after(rec, 0.0, lambda m: False) is None
    assert actions.wait_first(probe, rec, 0.0, lambda m: True, 0.1)[0] == 1.0
    assert actions.wait_first(probe, rec, 9.0, lambda m: True, 0.05) is None
    assert actions.wait_rest(probe, rec, hold=0.05, timeout=2.0)
    moving = TopicRecord('gt2')
    moving.add(1.0, None, _odom(1.0))
    assert not actions.wait_rest(probe, moving, hold=0.05, timeout=0.1)
    assert not actions.wait_rest(probe, TopicRecord('empty'), hold=0.05, timeout=0.1)


def test_commander_drive_and_waypoints(probe):
    cmd = probe.subscribe('cmd', Twist, keep_messages=500)
    com = actions.Commander(probe, 'cmd', rate=50.0).start()
    assert com.start() is com
    com.set(0.3, 0.1)
    assert probe.wait_until(lambda: cmd.last() is not None and cmd.last().linear.x == 0.3, 5.0)
    com.stop()
    assert probe.wait_until(lambda: actions.is_zero(cmd.last()), 2.0)
    assert actions.drive_for(probe, 'cmd', 0.2, 0.0, 0.05, 2.0, rate=100.0)
    assert not actions.drive_for(probe, 'cmd', 0.2, 0.0, 1.0, -1.0)
    gt = TopicRecord('gt')
    gt.add(0.0, None, _odom(0.0))
    reached = actions.follow_waypoints(probe, gt, 'cmd', [(0.1, 0.0), (5.0, 0.0)], tol=0.3,
                                       timeout_per_wp=0.2)
    assert reached == [True, False]
    assert probe.wait_until(lambda: actions.is_zero(cmd.last()), 2.0)
    assert actions.follow_waypoints(probe, TopicRecord('none'), 'cmd', [(1.0, 0.0)],
                                    timeout_per_wp=0.1) == [False]


def test_action_caller(probe):
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import Wait
    from rclpy.action import ActionServer, GoalResponse

    def execute(handle):
        handle.succeed()
        return Wait.Result()
    ActionServer(probe.node, Wait, 'ok_wait', execute)
    ActionServer(probe.node, Wait, 'no_wait', execute,
                 goal_callback=lambda goal: GoalResponse.REJECT)
    ok = actions.ActionCaller(probe, Wait, 'ok_wait')
    assert ok.wait_server(5.0)
    status, result = ok.call(Wait.Goal(), 5.0)
    assert status == GoalStatus.STATUS_SUCCEEDED and result is not None
    rejected = actions.ActionCaller(probe, Wait, 'no_wait')
    assert rejected.wait_server(5.0)
    assert rejected.call(Wait.Goal(), 5.0) == (None, None)


def test_seed_pose_amcl_and_ekf(probe):
    """배치 단계: initialpose 발행 + map EKF set_pose + AMCL 무이동 갱신 (kidnap_monitor 와 같은 묶음)."""
    import math
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from robot_localization.srv import SetPose
    from std_srvs.srv import Empty
    got = {'ekf': [], 'nomotion': 0}

    def on_set_pose(req, res):
        got['ekf'].append(req.pose)
        return res

    def on_nomotion(req, res):
        got['nomotion'] += 1
        return res
    probe.node.create_service(SetPose, '/robot_x/' + actions.EKF_SET_POSE, on_set_pose)
    probe.node.create_service(Empty, '/robot_x/' + actions.AMCL_NOMOTION, on_nomotion)
    rec = probe.subscribe('/robot_x/initialpose', PoseWithCovarianceStamped, keep_messages=10)
    assert probe.wait_for_publisher('/robot_x/initialpose', 0.1) is False     # 아직 발행자 없음
    out = actions.seed_pose(probe, 'robot_x', 0.8, 5.0, math.pi / 2, timeout=5.0)
    assert out == {'initialpose': 3, 'ekf_set_pose': True, 'amcl_nomotion_updates': 2}
    assert probe.wait_until(lambda: rec.count >= 1, 5.0)
    msg = rec.last()
    assert msg.header.frame_id == 'map' and msg.pose.pose.position.x == pytest.approx(0.8)
    assert msg.pose.covariance[0] == pytest.approx(0.05 ** 2)
    assert got['nomotion'] == 2 and len(got['ekf']) == 1
    ekf = got['ekf'][0].pose.pose
    assert ekf.position.y == pytest.approx(5.0) and ekf.orientation.z == pytest.approx(
        math.sin(math.pi / 4))
    # 서비스가 없으면 발행만 하고 실패를 보고한다 (예외 없음)
    none = actions.seed_pose(probe, 'nobody', 0.0, 0.0, 0.0, timeout=0.2, repeats=1)
    assert none == {'initialpose': 1, 'ekf_set_pose': False, 'amcl_nomotion_updates': 0}


def test_cases(itest_env):
    ctx = Context(catalog.get(9)).begin()

    class Case(cases.ProbeCase):
        CTX = ctx

        def test_a(self):
            self.check('ok', 1.0, 2.0, True, 'm')
            self.measure('k', 1)
            assert self.timeout(2.0) == 2.0
            with self.assertRaises(AssertionError):
                self.check('bad', 3.123456, 2.0, False, 'm', 'detail')
            rec = self.probe.subscribe('silent', String)
            with self.assertRaises(AssertionError):
                self.require_topic(rec, 1, 0.05)

    result = unittest.TextTestRunner(verbosity=0).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(Case))
    assert result.wasSuccessful()
    names = [c['name'] for c in ctx.record.data['checks']]
    assert names == ['ok', 'bad'] and ctx.record.data['checks'][1]['value'] == 3.1235
    assert cases.fmt(float('nan')) != cases.fmt(0.5)

    class NoCtx(cases.ProbeCase):
        def test_x(self):
            pass
    bad = unittest.TextTestRunner(verbosity=0).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(NoCtx))
    assert not bad.wasSuccessful()

    after = cases.AfterShutdown('test_exit_codes')
    after.CTX = ctx
    after.test_exit_codes([NS(process_name='ekf_node-1', returncode=0),
                           NS(process_name='gazebo-2', returncode=-9),
                           NS(process_name='python3-3', returncode=-2)])
    with pytest.raises(AssertionError):
        after.test_exit_codes([NS(process_name='safety_node-1', returncode=-11)])
    after.test_zz_finalize()
    assert ctx.record.data['state'] == 'finished'
