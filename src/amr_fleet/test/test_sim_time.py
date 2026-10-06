"""
플릿 노드의 시간 축 (보너스 4 P0 "모든 타이머 sim-time").

- fleet_manager · traffic_manager · fleet_adapter 의 타이머는 모두 노드 시계(ROS_TIME, use_sim_time 이면
  /clock)에서 돈다. 벽시계 예외는 /clock 감시 하나뿐이다 (/clock 이 없다는 것 자체를 알리는 타이머).
- TimerQueue(송신 지연 큐)와 EmulatedInbox(수신 측 통신 모의)는 sim 시각이 멈추면 벽시계가 흘러도
  내보내지 않고, sim 시각이 해제 시각을 넘으면 내보낸다.
목록과 근거: docs/research/fleet-distributed/p0.md §2. rclpy·amr_msgs 가 없으면 건너뛴다.
"""

import time

import pytest

try:
    from amr_fleet.comm_emulator import CommEmulator, LinkModel, Partition
    from amr_fleet.fleet_adapter_node import FleetAdapterNode
    from amr_fleet.fleet_manager_node import FleetManagerNode
    from amr_fleet.timer_queue import EmulatedInbox, TimerQueue
    from amr_fleet.traffic_manager_node import TrafficManagerNode
    import rclpy
    from rclpy.clock import ClockType
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='rclpy/amr_msgs 가 없는 환경')


def _params(d):
    return [Parameter(k, value=v) for k, v in d.items()]


@pytest.fixture(scope='module')
def ctx():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def _spin_for(executor, wall_s):
    end = time.monotonic() + wall_s
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.02)


def test_fleet_node_timers_run_on_node_clock(ctx, tmp_path):
    sim = {'use_sim_time': True}
    manager = FleetManagerNode(namespace='/st_fleet', parameter_overrides=_params(dict(
        sim, robot_ids='amr_s1', discover_robots=True, log_dir=str(tmp_path))))
    traffic = TrafficManagerNode(namespace='/st_fleet', parameter_overrides=_params(dict(
        sim, robot_ids='amr_s1')))
    adapter = FleetAdapterNode(namespace='/amr_s1', parameter_overrides=_params(dict(
        sim, log_dir=str(tmp_path))))
    try:
        for node in (manager, traffic, adapter):
            assert node.timers, node.get_name()
            for timer in node.timers:
                if node is manager and timer is manager._clock_watchdog:
                    # 유일한 벽시계 예외: /clock 이 오지 않는 것을 감시한다 (ROS 시계로는 감시 불가)
                    assert timer.clock.clock_type == ClockType.STEADY_TIME
                    continue
                assert timer.clock.clock_type == ClockType.ROS_TIME, node.get_name()
                assert timer.clock is node.get_clock(), node.get_name()
        assert manager.get_clock().ros_time_is_active                 # use_sim_time → /clock
    finally:
        for node in (manager, traffic, adapter):
            node.destroy_node()


def test_timer_queue_and_inbox_release_on_sim_time(ctx):
    node = rclpy.create_node('st_queue', parameter_overrides=_params({'use_sim_time': True}))
    clock = node.get_clock()
    clock.set_ros_time_override(Time(seconds=100))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    out = []
    try:
        q = TimerQueue(node, lambda item: out.append(('q', item, node.get_clock().now())))
        em = CommEmulator(seed=3, default=LinkModel(50.0, 50.0),        # 고정 50 ms
                          partitions=[Partition(200.0, 300.0, (frozenset({'amr_b'}),))])
        inbox = EmulatedInbox(node, em, 'amr_a', lambda item: out.append(('in', item, None)))
        assert q.push('m1', 0.05) == pytest.approx(100.05)
        assert inbox.receive('amr_b', 'beacon_1') == pytest.approx(100.05)
        assert inbox.receive('amr_a', 'self') == pytest.approx(100.0)   # 자기 루프는 바로
        assert out == [('in', 'self', None)]
        _spin_for(executor, 0.4)                    # 벽시계 0.4 s — sim 은 멈춰 있다
        assert len(out) == 1 and len(q) == 1 and len(inbox) == 1
        clock.set_ros_time_override(Time(seconds=100, nanoseconds=60_000_000))
        _spin_for(executor, 0.3)
        assert [o[:2] for o in out[1:]] == [('q', 'm1'), ('in', 'beacon_1')] or \
            [o[:2] for o in out[1:]] == [('in', 'beacon_1'), ('q', 'm1')]
        released_at = [o[2] for o in out if o[0] == 'q'][0]
        assert released_at.nanoseconds * 1e-9 == pytest.approx(100.06)  # sim 시각에 내보냄
        # 분할 구간 안: amr_b → amr_a 유실, 구간 밖으로 시간이 가도 이미 버린 것은 오지 않는다
        clock.set_ros_time_override(Time(seconds=250))
        assert inbox.receive('amr_b', 'beacon_2') is None and inbox.dropped == 1
        assert inbox.receive('amr_c', 'beacon_3') == pytest.approx(250.05)   # 나머지 ↔ 나머지
        clock.set_ros_time_override(Time(seconds=251))
        _spin_for(executor, 0.3)
        assert [o[1] for o in out if o[0] == 'in'] == ['self', 'beacon_1', 'beacon_3']
    finally:
        executor.remove_node(node)
        node.destroy_node()
