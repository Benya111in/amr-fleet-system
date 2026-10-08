"""
fleet_manager_node 하트비트 · 관찰자 복귀 (분산 모드, p3.md §7 요청 1 — b4-wire).

- 기본(heartbeat.enabled false): /fleet/heartbeat 발행자 · /fleet_p2p/beacon 구독이 없다 (중앙 모드 그대로).
- 켜면: 관찰(observe_s) 동안 ROLE_OBSERVER 로 비컨의 fence_epoch 최댓값 + 1 을 후보로 내고, 그동안 받은 작업을
  할당하지 않는다. 관찰이 끝나면 그 epoch 로 ACTIVE — assign_task 요청 · task_events 가 그 epoch 를 싣고,
  epoch_file 에 남는다. seq 는 역할이 바뀌어도 1 씩 이어진다.
- 하트비트 주기는 노드 시계: use_sim_time 에서 /clock 이 멈추면 하트비트도 멈춘다.
"""

import json
import time

import pytest

try:
    from amr_fleet.fleet_manager_node import FleetManagerNode
    from amr_msgs.msg import AgentBeacon, FleetHeartbeat, RobotState, Task
    from amr_msgs.srv import AssignTask
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock as ClockMsg
    from std_msgs.msg import String
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='rclpy/amr_msgs 가 없는 환경')

ROBOT = 'amr_h1'
BE = None if not HAVE_ROS else QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)


def _params(d):
    return [Parameter(k, value=v) for k, v in d.items()]


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


class _Rig:
    """관리자 + 가짜 로봇(robot_state · assign_task) + 비컨 발행 · 하트비트 구독."""

    def __init__(self, manager_params, namespace='/fleet'):
        self.manager = FleetManagerNode(namespace=namespace,
                                        parameter_overrides=_params(manager_params))
        self.node = rclpy.create_node('hb_test_client')
        self.robot = rclpy.create_node('fake_robot', namespace='/' + ROBOT)
        self.beats = []
        self.requests = []
        self.events = []
        self.node.create_subscription(
            FleetHeartbeat, '/fleet/heartbeat',
            lambda m: self.beats.append((time.monotonic(), m)), BE)
        self.node.create_subscription(Task, f'{namespace}/task_events', self.events.append, 100)
        self.beacon = self.node.create_publisher(AgentBeacon, '/fleet_p2p/beacon', BE)
        self.request = self.node.create_publisher(String, f'{namespace}/task_request', 10)
        self.state = self.robot.create_publisher(RobotState, 'robot_state', 10)
        self.robot.create_service(AssignTask, 'assign_task', self._on_assign)
        self.robot.create_timer(0.1, self._tick)
        self.ex = SingleThreadedExecutor()
        for n in (self.manager, self.node, self.robot):
            self.ex.add_node(n)

    def _tick(self):
        msg = RobotState(robot_id=ROBOT, status=0, battery_level=90.0)
        msg.header.stamp = self.robot.get_clock().now().to_msg()
        self.state.publish(msg)

    def _on_assign(self, req, resp):
        self.requests.append((time.monotonic(), req.task.task_id, req.task.epoch,
                              req.task.leader_id, req.task.seq))
        resp.success, resp.robot_id, resp.message = True, ROBOT, 'ok'
        return resp

    def spin(self, seconds, until=None, step=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.ex.spin_once(timeout_sec=0.02)
            if step is not None:
                step()
            if until is not None and until():
                return True
        return until is None

    def close(self):
        self.ex.shutdown(timeout_sec=2.0)
        for n in (self.manager, self.node, self.robot):
            n.destroy_node()


def _base(**extra):
    d = {'robot_ids': [ROBOT], 'discover_robots': False, 'simulate_latency': False,
         'allocation_period_s': 0.1, 'status_period_s': 0.5}
    d.update(extra)
    return d


def test_default_manager_has_no_heartbeat_endpoints(ros, tmp_path):
    rig = _Rig(_base(log_dir=str(tmp_path)), namespace='/fleet_default')
    try:
        rig.spin(1.0)
        pubs = rig.node.get_publishers_info_by_topic('/fleet/heartbeat')
        subs = rig.node.get_subscriptions_info_by_topic('/fleet_p2p/beacon')
        mine = [e for e in pubs + subs if e.node_namespace == '/fleet_default']
        assert mine == [] and rig.manager.commanding
        assert rig.beats == []
    finally:
        rig.close()


def test_observer_then_active_with_max_seen_plus_one(ros, tmp_path):
    epoch_file = tmp_path / 'epoch'
    rig = _Rig(_base(log_dir=str(tmp_path), epoch_file=str(epoch_file),
                     **{'heartbeat.enabled': True, 'heartbeat.period_s': 0.1,
                        'heartbeat.observe_s': 1.5}))
    try:
        assert rig.spin(5.0, until=lambda: rig.manager._robots[ROBOT].state is not None)
        assert epoch_file.read_text().strip() == '1'          # resolve_epoch: 파일 없음 → 1
        # 관찰 중: 로봇이 본 최고 epoch 7 을 비컨으로 → 후보 8. 그동안 낸 작업은 할당하지 않는다
        last = {'t': 0.0}

        def beacons():
            if time.monotonic() - last['t'] > 0.1:
                last['t'] = time.monotonic()
                rig.beacon.publish(AgentBeacon(robot_id=ROBOT, fence_epoch=7, fence_leader='x'))

        rig.request.publish(String(data=json.dumps({
            'task_id': 'hb_t1', 'pickup': {'x': 0.0, 'y': 0.0}, 'dropoff': {'x': 1.0, 'y': 0.0},
            'item_type': 'small'})))
        assert rig.spin(10.0, step=beacons, until=lambda: any(
            b.role == FleetHeartbeat.ROLE_ACTIVE for _, b in rig.beats))
        rig.spin(1.0, step=beacons, until=lambda: rig.requests)
        obs = [(t, b) for t, b in rig.beats if b.role == FleetHeartbeat.ROLE_OBSERVER]
        act = [(t, b) for t, b in rig.beats if b.role == FleetHeartbeat.ROLE_ACTIVE]
        assert obs and act
        assert all(b.epoch == 8 for _, b in act), [b.epoch for _, b in act]
        assert all(b.leader_id == '/fleet/fleet_manager_node' for _, b in rig.beats)
        seqs = [b.seq for _, b in rig.beats]
        assert seqs == list(range(seqs[0], seqs[0] + len(seqs))), seqs   # 역할이 바뀌어도 이어진다
        # 할당은 ACTIVE 뒤에만, epoch 8 로
        assert rig.requests, '관찰이 끝난 뒤에도 할당하지 않았다'
        t_req, task_id, epoch, leader, _ = rig.requests[0]
        assert task_id == 'hb_t1' and epoch == 8 and leader == '/fleet/fleet_manager_node'
        assert t_req >= act[0][0] - 0.05
        assert epoch_file.read_text().strip() == '8'          # 올린 epoch 을 남긴다 (재기동은 9 부터)
        assert not rig.manager.superseded_warned
        # ACTIVE 뒤 더 높은 권위자를 본 로봇 → 경보 한 번 (스스로 올리지 않는다)
        rig.beacon.publish(AgentBeacon(robot_id=ROBOT, fence_epoch=20, fence_leader='y'))
        assert rig.spin(3.0, until=lambda: rig.manager.superseded_warned)
        n = len(rig.beats)
        rig.spin(0.5)
        assert all(b.epoch == 8 for _, b in rig.beats[n:])
    finally:
        rig.close()


def test_heartbeat_follows_the_node_clock(ros, tmp_path):
    rig = _Rig(_base(log_dir=str(tmp_path), use_sim_time=True,
                     **{'heartbeat.enabled': True, 'heartbeat.period_s': 0.5}),
               namespace='/fleet')
    clock = rig.node.create_publisher(ClockMsg, '/clock', 10)

    def set_sim(t):
        clock.publish(ClockMsg(clock=rclpy.time.Time(nanoseconds=int(t * 1e9)).to_msg()))

    try:
        set_sim(100.0)
        rig.spin(1.0, step=lambda: set_sim(100.0))
        n0 = len(rig.beats)
        rig.spin(1.5, step=lambda: set_sim(100.0))         # sim 이 멈춘 동안 (벽시계 1.5 s)
        assert len(rig.beats) - n0 <= 1, len(rig.beats) - n0
        t = {'sim': 100.0}

        def advance():
            t['sim'] += 0.05
            set_sim(t['sim'])
        n1 = len(rig.beats)
        rig.spin(1.0, step=advance)                          # sim 이 대략 1–2.5 s 흐른다
        assert len(rig.beats) - n1 >= 1
        stamps = [b.stamp.sec + b.stamp.nanosec * 1e-9 for _, b in rig.beats[n1:]]
        assert all(s >= 100.0 for s in stamps)               # stamp 도 sim 시각
    finally:
        rig.close()
