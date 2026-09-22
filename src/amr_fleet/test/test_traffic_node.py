"""
traffic_manager_node ROS 배선 시험 (한 프로세스).

가짜 로봇 2대가 plan · odometry/filtered_map · robot_state 를
내고, 1차선 통로에서 마주 서면 교착 탐지 → 우선순위 낮은 로봇에 yield_pose + latched hold →
/fleet/traffic_events(traffic/DEADLOCK) 를 fleet_manager_node 가 deadlock_count 로 센다 →
비키면 RESOLVED + hold=false. latched 토픽(hold · keepout_mask · costmap_filter_info)은 늦게 뜬
구독자가 받는지 본다. rclpy·amr_msgs·nav2_msgs 가 없는 환경에서는 건너뛴다.
"""

import math
import time

import numpy as np
import pytest

try:
    from amr_fleet.fleet_manager_node import FleetManagerNode
    from amr_fleet.traffic_geometry import GridSpec
    from amr_fleet.traffic_manager_node import (
        TrafficManagerNode, default_zones_path, mask_to_msg, parse_ids, path_to_array,
        yaw_from_quaternion,
    )
    from amr_fleet.traffic_resolution import KeepoutMask
    from amr_msgs.msg import RobotState, Task
    from builtin_interfaces.msg import Time
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.msg import CostmapFilterInfo
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='rclpy/amr_msgs/nav2_msgs 가 없는 환경')

ROBOTS = ('amr_x1', 'amr_x2')
MAP_TOPIC = '/traffic_test/map'
LAYOUT = """
frame_id: map
zones:
  - {id: corr, kind: corridor, polygon: [[4, 4.5], [16, 4.5], [16, 5.5], [4, 5.5]]}
pockets:
  - {id: pe_n, x: 18.0, y: 8.0}
  - {id: pw_n, x: 2.0, y: 8.0}
"""
NO_POCKET_LAYOUT = """
frame_id: map
zones:
  - {id: corr, kind: corridor, polygon: [[4, 4.5], [16, 4.5], [16, 5.5], [4, 5.5]]}
"""


def _params(d):
    return [Parameter(k, value=v) for k, v in d.items()]


def _latched():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _map_msg():
    occ = np.full((100, 200), 100, dtype=np.int8)
    occ[5:95, 5:40] = 0
    occ[5:95, 160:195] = 0
    occ[45:55, 40:160] = 0
    occ[10:30, 40:160] = 0          # 남쪽 우회 도로 (전략 2 대체 경로)
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = 0.1
    msg.info.width, msg.info.height = 200, 100
    msg.info.origin.orientation.w = 1.0
    msg.data = occ.ravel().tolist()
    return msg


@pytest.fixture(scope='module')
def ros(tmp_path_factory):
    tmp = tmp_path_factory.mktemp('traffic')
    layout = tmp / 'zones.yaml'
    layout.write_text(LAYOUT)
    rclpy.init()
    traffic = TrafficManagerNode(parameter_overrides=_params({
        'robot_ids': ','.join(ROBOTS), 'update_rate_hz': 5.0, 'map_topic': MAP_TOPIC,
        'zones_file': str(layout), 'zones.tokens': False,
        'deadlock.stationary_time_s': 0.6, 'deadlock.confirm_s': 0.4,
        'resolution.pocket_search_radius_m': 20.0,
    }))
    manager = FleetManagerNode(parameter_overrides=_params({
        'robot_ids': ','.join(ROBOTS), 'discover_robots': False, 'log_dir': str(tmp),
        'status_period_s': 0.2,
    }))
    client = rclpy.create_node('traffic_test_client')
    got = {'events': [], 'alerts': [], 'yield': []}
    rel = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    client.create_subscription(DiagnosticArray, '/fleet/traffic_events', got['events'].append, rel)
    client.create_subscription(DiagnosticArray, '/fleet/alerts', got['alerts'].append, rel)
    client.create_subscription(PoseStamped, f'/{ROBOTS[1]}/traffic/yield_pose',
                               got['yield'].append, 10)
    pubs = {'map': client.create_publisher(OccupancyGrid, MAP_TOPIC, _latched()),
            'task': client.create_publisher(Task, '/fleet/task_events', rel)}
    for rid in ROBOTS:
        pubs[f'{rid}/odom'] = client.create_publisher(Odometry,
                                                      f'/{rid}/odometry/filtered_map', 10)
        pubs[f'{rid}/plan'] = client.create_publisher(Path, f'/{rid}/plan', 5)
        pubs[f'{rid}/state'] = client.create_publisher(RobotState, f'/{rid}/robot_state', 10)
    executor = SingleThreadedExecutor()
    for node in (traffic, manager, client):
        executor.add_node(node)
    pose = {ROBOTS[0]: (9.0, 5.0, 0.0), ROBOTS[1]: (9.8, 5.0, math.pi)}
    goal = {ROBOTS[0]: (18.0, 5.0), ROBOTS[1]: (2.0, 5.0)}
    task = {ROBOTS[0]: 'task_hi', ROBOTS[1]: 'task_lo'}

    def publish_robots():
        now = traffic.get_clock().now().to_msg()
        for rid in ROBOTS:
            x, y, yaw = pose[rid]
            od = Odometry()
            od.header.frame_id = 'map'
            od.header.stamp = now
            od.pose.pose.position.x, od.pose.pose.position.y = x, y
            od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(yaw / 2), \
                math.cos(yaw / 2)
            pubs[f'{rid}/odom'].publish(od)
            st = RobotState(robot_id=rid, current_task_id=task[rid],
                            status=RobotState.STATUS_MOVING)
            pubs[f'{rid}/state'].publish(st)
            path = Path()
            path.header.frame_id = 'map'
            for px, py in ((x, y), goal[rid]):
                ps = PoseStamped()
                ps.pose.position.x, ps.pose.position.y = float(px), float(py)
                path.poses.append(ps)
            pubs[f'{rid}/plan'].publish(path)

    def spin_until(pred, timeout=20.0):
        end = time.monotonic() + timeout
        last = 0.0
        while time.monotonic() < end:
            if time.monotonic() - last > 0.1:
                publish_robots()
                last = time.monotonic()
            executor.spin_once(timeout_sec=0.02)
            if pred():
                return True
        return False

    pubs['map'].publish(_map_msg())
    for rid, prio in zip(ROBOTS, (200, 100)):
        pubs['task'].publish(Task(task_id=task[rid], robot_id=rid, priority=prio))
    assert spin_until(lambda: traffic.tm.tgrid is not None and len(traffic._tasks) == 2)
    yield dict(traffic=traffic, manager=manager, client=client, got=got, pubs=pubs,
               spin=spin_until, pose=pose, goal=goal, executor=executor)
    executor.shutdown()
    for node in (traffic, manager, client):
        node.destroy_node()
    rclpy.try_shutdown()


def _statuses(msgs):
    return [st for m in msgs for st in m.status]


def _late_value(ros, msg_type, topic, timeout=5.0):
    """transient_local 구독자를 새로 만들어 래치된 마지막 값을 받는다."""
    got = []
    sub = ros['client'].create_subscription(msg_type, topic, got.append, _latched())
    try:
        ros['spin'](lambda: bool(got), timeout)
    finally:
        ros['client'].destroy_subscription(sub)
    return got[-1] if got else None


def test_helpers():
    assert parse_ids(' amr_01, bad-id,amr_01,amr_02 ') == ['amr_01', 'amr_02']
    assert parse_ids(['amr_03', '']) == ['amr_03'] and parse_ids(None) == []
    assert yaw_from_quaternion(math.sin(0.4), math.cos(0.4)) == pytest.approx(0.8)
    assert default_zones_path().endswith('traffic_zones.yaml')
    p = Path()
    assert path_to_array(p) is None
    for x in (0.0, 1.5):
        ps = PoseStamped()
        ps.pose.position.x = x
        p.poses.append(ps)
    assert path_to_array(p).tolist() == [[0.0, 0.0], [1.5, 0.0]]
    m = KeepoutMask(GridSpec(0.5, -1.0, 2.0, 3, 2), np.array([[0, 100, 0], [0, 0, 100]],
                                                             dtype=np.int8))
    msg = mask_to_msg(m, 'map', Time())
    assert (msg.info.width, msg.info.height, msg.info.origin.position.x) == (3, 2, -1.0)
    assert list(msg.data) == [0, 100, 0, 0, 0, 100] and msg.header.frame_id == 'map'


def test_latched_initial_state(ros):
    info = _late_value(ros, CostmapFilterInfo, f'/{ROBOTS[1]}/costmap_filter_info')
    assert info is not None and info.type == 0
    assert info.filter_mask_topic == f'/{ROBOTS[1]}/keepout_mask'
    assert info.base == 0.0 and info.multiplier == 1.0
    mask = _late_value(ros, OccupancyGrid, f'/{ROBOTS[1]}/keepout_mask')
    assert mask is not None and (mask.info.width, mask.info.height) == (400, 200)   # 0.05 m, 빈 마스크
    assert max(mask.data) == 0
    hold = _late_value(ros, Bool, f'/{ROBOTS[0]}/traffic/hold')
    assert hold is not None and hold.data is False


def test_deadlock_detected_counted_and_resolved(ros):
    got, traffic, manager = ros['got'], ros['traffic'], ros['manager']
    assert ros['spin'](lambda: any(s.name == 'traffic/DEADLOCK' for s in _statuses(got['events'])))
    dl = next(s for s in _statuses(got['events']) if s.name == 'traffic/DEADLOCK')
    vals = {kv.key: kv.value for kv in dl.values}
    assert vals['victim'] == ROBOTS[1] and vals['strategy'] == 'YIELD'
    assert vals['type'] == 'HEAD_ON'
    assert vals['pocket'] == 'pe_n' and dl.hardware_id == ROBOTS[1]
    # 우선순위 낮은 로봇(task_events 의 priority 100)에 포켓 + latched hold
    assert ros['spin'](lambda: bool(got['yield']))
    assert (got['yield'][-1].pose.position.x, got['yield'][-1].pose.position.y) == (18.0, 8.0)
    assert _late_value(ros, Bool, f'/{ROBOTS[1]}/traffic/hold').data is True
    assert _late_value(ros, Bool, f'/{ROBOTS[0]}/traffic/hold').data is False
    # fleet_manager 가 traffic/DEADLOCK 만 센다
    assert ros['spin'](lambda: manager._kpi.deadlock_count == 1, 5.0)
    alerts = [s for s in _statuses(got['alerts']) if s.name == 'fleet/DEADLOCK']
    assert alerts and alerts[0].level == b'\x02'
    avals = {kv.key: kv.value for kv in alerts[0].values}
    assert avals['robots'] == ','.join(ROBOTS) and avals['task_id'] == 'task_lo'
    # 희생 로봇이 포켓으로, 상대는 목표에 도착 → 해소, hold 해제
    ros['pose'][ROBOTS[1]] = (18.0, 8.0, 0.0)
    ros['pose'][ROBOTS[0]] = (18.0, 5.0, 0.0)
    assert ros['spin'](lambda: any(s.name == 'traffic/RESOLVED' for s in _statuses(got['events'])))
    res = next(s for s in _statuses(got['events']) if s.name == 'traffic/RESOLVED')
    assert {kv.key: kv.value for kv in res.values}['strategy'] == 'YIELD'
    assert ros['spin'](lambda: traffic._robots[ROBOTS[1]].hold is False, 5.0)
    assert _late_value(ros, Bool, f'/{ROBOTS[1]}/traffic/hold').data is False
    ros['spin'](lambda: False, 0.6)
    assert manager._kpi.deadlock_count == 1                 # RESOLVED 는 세지 않는다
    assert traffic.event_counts['traffic/DEADLOCK'] == 1 and len(traffic.tick_ms) > 0


def test_keepout_strategy_publishes_and_clears_mask(ros, tmp_path):
    """포켓이 없으면 전략 2: 희생 로봇 전용 keepout_mask 에 lethal 을 칠해 latched 로 내고, 해소 뒤 비운다."""
    layout = tmp_path / 'no_pocket.yaml'
    layout.write_text(NO_POCKET_LAYOUT)
    ids = ('amr_y1', 'amr_y2')
    node = TrafficManagerNode(namespace='/fleet_alt', parameter_overrides=_params({
        'robot_ids': ','.join(ids), 'update_rate_hz': 5.0, 'map_topic': MAP_TOPIC,
        'zones_file': str(layout), 'zones.tokens': False, 'resolution.auto_pockets': False,
        'deadlock.stationary_time_s': 0.6, 'deadlock.confirm_s': 0.4,
    }))
    ex, client, got = ros['executor'], ros['client'], ros['got']
    ex.add_node(node)
    pubs = {rid: (client.create_publisher(Odometry, f'/{rid}/odometry/filtered_map', 10),
                  client.create_publisher(Path, f'/{rid}/plan', 5),
                  client.create_publisher(RobotState, f'/{rid}/robot_state', 10)) for rid in ids}
    rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    task_pub = client.create_publisher(Task, '/fleet/task_events', rel)
    state = {'amr_y1': [(9.0, 5.0, 0.0), [(18.0, 5.0)]],
             'amr_y2': [(9.8, 5.0, math.pi), [(2.0, 5.0)]]}
    masks = []
    client.create_subscription(OccupancyGrid, '/amr_y2/keepout_mask', masks.append, _latched())

    def pump(pred, timeout=20.0):
        end, last = time.monotonic() + timeout, 0.0
        while time.monotonic() < end:
            if time.monotonic() - last > 0.1:
                last = time.monotonic()
                for rid, prio in zip(ids, (200, 100)):
                    task_pub.publish(Task(task_id=f'task_{rid}', robot_id=rid, priority=prio))
                    (x, y, yaw), pts = state[rid]
                    od = Odometry()
                    od.header.frame_id = 'map'
                    od.pose.pose.position.x, od.pose.pose.position.y = x, y
                    od.pose.pose.orientation.z = math.sin(yaw / 2)
                    od.pose.pose.orientation.w = math.cos(yaw / 2)
                    pubs[rid][0].publish(od)
                    path = Path()
                    path.header.frame_id = 'map'
                    for px, py in [(x, y)] + pts:
                        ps = PoseStamped()
                        ps.pose.position.x, ps.pose.position.y = float(px), float(py)
                        path.poses.append(ps)
                    pubs[rid][1].publish(path)
                    pubs[rid][2].publish(RobotState(robot_id=rid, current_task_id=f'task_{rid}',
                                                    status=RobotState.STATUS_MOVING))
            ex.spin_once(timeout_sec=0.02)
            if pred():
                return True
        return False

    def events(name):
        return [{kv.key: kv.value for kv in s.values} for s in _statuses(got['events'])
                if s.name == name and s.hardware_id == 'amr_y2']
    try:
        assert pump(lambda: any(max(m.data) == 100 for m in masks))
        esc = events('traffic/ESCALATED')
        assert esc and esc[0]['reason'] == 'no_pocket' and esc[0]['strategy'] == 'ALT_PATH'
        mask = next(m for m in masks if max(m.data) == 100)
        assert (mask.info.width, mask.info.height, mask.header.frame_id) == (400, 200, 'map')
        grid = np.asarray(mask.data, dtype=np.int8).reshape(200, 400)
        assert grid[100, 170] == 100 and grid[100, 196] == 0     # 상대 몸 (8.5, 5) · 자기 자리 (9.8, 5)
        assert not node._robots['amr_y2'].hold                     # 전략 2 는 세우지 않는다
        # 희생 로봇이 남쪽 우회로로 재계획해 분쟁 구간을 벗어났고, 상대는 목표 도착 → 해소 · 빈 마스크
        state['amr_y2'][1] = [(16.8, 5.0), (16.8, 2.0), (3.0, 2.0), (2.0, 5.0)]
        pump(lambda: False, 1.0)
        state['amr_y2'] = [(16.8, 2.0, math.pi), [(3.0, 2.0), (2.0, 5.0)]]
        state['amr_y1'][0] = (18.0, 5.0, 0.0)
        assert pump(lambda: any(e['strategy'] == 'ALT_PATH' for e in events('traffic/RESOLVED')))
        assert pump(lambda: max(masks[-1].data) == 0, 5.0)
    finally:
        ex.remove_node(node)
        node.destroy_node()


def test_bad_inputs_do_not_kill_node(ros):
    traffic = ros['traffic']
    before = traffic.internal_errors
    bad = _map_msg()
    bad.info.width = 7                                      # 크기 불일치 → 콜백 예외 → INTERNAL_ERROR
    ros['pubs']['map'].publish(bad)
    assert ros['spin'](lambda: traffic.internal_errors == before + 1, 5.0)
    assert ros['spin'](lambda: any(s.name == 'fleet/INTERNAL_ERROR'
                                   for s in _statuses(ros['got']['alerts'])), 5.0)
    wrong = Path()
    wrong.header.frame_id = 'odom'
    ros['pubs'][f'{ROBOTS[0]}/plan'].publish(wrong)          # 다른 프레임 plan 은 무시 (경고만)
    ros['spin'](lambda: False, 0.5)
    assert traffic.internal_errors == before + 1 and traffic.tm.tgrid is not None
