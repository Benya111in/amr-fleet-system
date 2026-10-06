"""
traffic_manager_node ROS 배선 시험 (한 프로세스).

가짜 로봇 2대가 plan · odometry/filtered_map · robot_state 를
내고, 1차선 통로에서 마주 서면 교착 탐지 → 우선순위 낮은 로봇에 yield_pose + latched hold →
/fleet/traffic_events(traffic/DEADLOCK) 를 fleet_manager_node 가 deadlock_count 로 센다 →
비키면 RESOLVED + hold=false. latched 토픽(hold · yield_pose · keepout_mask · costmap_filter_info)은
늦게 뜬 구독자가 받는지, yield_pose 가 hold=true 보다 먼저 나가고 양보가 끝나면 비워지는지 본다.
hold 는 평가한 값만 낸다: 기동 시 초기값이 없고, 관측한 로봇은 첫 평가 뒤, 한 번도 관측 못 한 로봇은
hold 구독자가 붙고 obs_timeout_s 뒤 false (hold_failsafe.md). 평가한 값은 hold_republish_s 마다 다시 낸다 — 평가 전에는
내지 않고, 포켓-먼저 미룸 중에는 false 를 다시 내지 않으며, hold=true 는 포켓 뒤에. hold writer 의 DDS 매칭 구독자 수가
바뀌면 기록한다 (hold_failsafe.md §7, D9 · D10).
rclpy·amr_msgs·nav2_msgs 가 없는 환경에서는 건너뛴다.
"""

import math
import time

import numpy as np
import pytest

try:
    from amr_fleet.fleet_manager_node import FleetManagerNode
    from amr_fleet.traffic_manager import RobotCommand
    from amr_fleet.traffic_geometry import GridSpec
    from amr_fleet.traffic_manager_node import (
        TrafficManagerNode, default_zones_path, hold_republish_due, mask_to_msg, parse_ids,
        path_to_array, yaw_from_quaternion,
    )
    from amr_fleet.traffic_resolution import KeepoutMask
    from amr_msgs.msg import RobotState, Task
    from builtin_interfaces.msg import Time
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.msg import CostmapFilterInfo
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    import rclpy
    from rclpy.duration import Duration
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
    # 픽스처가 두 로봇을 관측시킨 뒤다 → hold 는 첫 평가 값 (기동 시 초기값은 내지 않는다)
    info = _late_value(ros, CostmapFilterInfo, f'/{ROBOTS[1]}/costmap_filter_info')
    assert info is not None and info.type == 0
    assert info.filter_mask_topic == f'/{ROBOTS[1]}/keepout_mask'
    assert info.base == 0.0 and info.multiplier == 1.0
    mask = _late_value(ros, OccupancyGrid, f'/{ROBOTS[1]}/keepout_mask')
    assert mask is not None and (mask.info.width, mask.info.height) == (400, 200)   # 0.05 m, 빈 마스크
    assert max(mask.data) == 0
    hold = _late_value(ros, Bool, f'/{ROBOTS[0]}/traffic/hold')
    assert hold is not None and hold.data is False
    pose = _late_value(ros, PoseStamped, f'/{ROBOTS[0]}/traffic/yield_pose')
    assert pose is not None and pose.header.frame_id == ''          # 포켓 없음


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
    assert ros['spin'](lambda: traffic._robots[ROBOTS[1]].hold is True, 5.0)   # 포켓 다음 주기
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


def test_yield_pose_before_hold_and_cleared_after(ros):
    """새 포켓은 hold=true 보다 한 주기 먼저 (latched), 양보가 끝나면 hold=false 뒤에 빈 포켓."""
    from amr_fleet.traffic_manager import RobotCommand
    traffic = ros['traffic']
    traffic._add_robot('amr_x9')
    r = traffic._robots['amr_x9']
    # 이미 hold=false 를 낸 로봇 (관리자는 기동 시 초기값을 내지 않는다 — 아직 낸 적 없는 로봇의 같은
    # 경로는 test_first_hold_behind_new_pocket_is_deferred_not_false)
    traffic._publish_hold(r, False)
    calls = []
    pose_pub, hold_pub = r.pub_yield.publish, r.pub_hold.publish
    r.pub_yield.publish = lambda m: (calls.append(('pose', m.header.frame_id)), pose_pub(m))
    r.pub_hold.publish = lambda m: (calls.append(('hold', m.data)), hold_pub(m))
    yield_cmd = RobotCommand(True, 'yield:incident9', (2.0, 8.0, 0.0), None, 9)
    traffic._apply(r, yield_cmd)
    assert calls == [('pose', 'map')] and r.hold is False            # 이번 주기는 포켓만
    late = _late_value(ros, PoseStamped, '/amr_x9/traffic/yield_pose')
    assert late.header.frame_id == 'map' and late.pose.position.y == 8.0
    traffic._apply(r, yield_cmd)
    assert calls[-1] == ('hold', True) and len(calls) == 2           # 다음 주기에 hold=true
    traffic._apply(r, RobotCommand(True, 'zone:corr'))                # 토큰 대기로 바뀜: hold 유지, 포켓 해제
    assert calls[2:] == [('pose', '')]
    traffic._apply(r, RobotCommand(True, 'yield:incident10', (18.0, 8.0, 0.0), None, 10))
    traffic._apply(r, RobotCommand())
    # 이미 hold → 포켓만, 끝나면 hold=false → 포켓 해제
    assert calls[3:] == [('pose', 'map'), ('hold', False), ('pose', '')]
    assert _late_value(ros, PoseStamped, '/amr_x9/traffic/yield_pose').header.frame_id == ''
    assert _late_value(ros, Bool, '/amr_x9/traffic/hold').data is False


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


def _own_executor(nodes):
    """픽스처 실행기와 따로 도는 실행기 + 조건 대기 (each: 매 반복 전에 부를 함수)."""
    ex = SingleThreadedExecutor()
    for node in nodes:
        ex.add_node(node)

    def spin(pred, timeout=5.0, each=None):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if each is not None:
                each()
            ex.spin_once(timeout_sec=0.02)
            if pred():
                return True
        return False
    return ex, spin


def test_hold_published_only_after_evaluation(ros):
    """
    기동 시 hold 초기값을 내지 않는다 (평가 전 false = 허가).

    관측한 로봇은 첫 평가 주기에 첫 값을 받고, 한 번도 관측 못 한 로봇은 hold 구독자가 붙은 주기부터
    obs_timeout_s 뒤 false 를 받는다 (관리할 수 없는 로봇을 영영 세우지 않는다). 구독자가 없는 로봇은
    기다리는 실행기가 없으므로 내지 않는다.
    """
    ids = ('amr_w1', 'amr_w2')
    node = TrafficManagerNode(namespace='/fleet_w', parameter_overrides=_params({
        'robot_ids': ','.join(ids + ('amr_w3',)), 'update_rate_hz': 10.0,
        'map_topic': '/traffic_test_w/map', 'zones.source': 'none', 'obs_timeout_s': 1.0,
    }))
    client = rclpy.create_node('traffic_test_w_client')
    got = {rid: [] for rid in ids}
    for rid in ids:     # (값, 받은 시각 — 관리자 시계)
        client.create_subscription(
            Bool, f'/{rid}/traffic/hold',
            lambda m, rid=rid: got[rid].append((m.data, node._now())), _latched())
    pose = []
    client.create_subscription(PoseStamped, f'/{ids[1]}/traffic/yield_pose', pose.append,
                               _latched())
    odom = client.create_publisher(Odometry, f'/{ids[0]}/odometry/filtered_map', 10)

    def publish_odom():
        od = Odometry()
        od.header.frame_id = 'map'
        od.pose.pose.position.x = 3.0
        od.pose.pose.orientation.w = 1.0
        odom.publish(od)

    ex, spin = _own_executor((node, client))
    try:
        assert spin(lambda: all(client.count_publishers(f'/{rid}/traffic/hold') == 1
                                for rid in ids) and bool(pose))
        assert pose[-1].header.frame_id == ''                      # 포켓 없음은 기동 시 latched
        # 관측 전: 주기는 돌지만 hold 는 아무도 받지 않는다
        assert spin(lambda: node._robots[ids[1]].listen_since is not None)
        assert not spin(lambda: any(got.values()), 0.5)
        assert node._robots[ids[0]].hold is None and node._robots[ids[1]].hold is None
        # 관측한 로봇: 첫 평가 값 (막을 것 없음 → false)
        assert spin(lambda: bool(got[ids[0]]), 5.0, publish_odom)
        assert [v for v, _ in got[ids[0]]] == [False]
        # 관측 못 한 로봇: obs_timeout_s 를 채운 뒤에야 false
        assert spin(lambda: bool(got[ids[1]]), 5.0, publish_odom)
        assert [v for v, _ in got[ids[1]]] == [False]
        assert got[ids[1]][0][1] - node._robots[ids[1]].listen_since >= 1.0
        assert node._robots[ids[1]].hold is False
        # 값은 평가가 바뀔 때만 바뀐다: 그 뒤의 수신은 같은 값의 재발행(D9, 기본 1 s)뿐이다
        spin(lambda: False, 0.6, publish_odom)
        assert {v for v, _ in got[ids[0]]} == {False} and {v for v, _ in got[ids[1]]} == {False}
        # 구독자가 없는 로봇: 기다리는 실행기가 없다 → 내지 않는다
        w3 = node._robots['amr_w3']
        assert w3.hold is None and w3.listen_since is None
    finally:
        ex.shutdown()
        node.destroy_node()
        client.destroy_node()


def test_first_hold_behind_new_pocket_is_deferred_not_false(ros):
    """포켓 먼저 규칙으로 hold=true 를 한 주기 미룰 때, 아직 hold 를 낸 적이 없으면 false 를 내지 않는다."""
    node = TrafficManagerNode(namespace='/fleet_z', parameter_overrides=_params({
        'robot_ids': 'amr_z1', 'update_rate_hz': 0.1, 'map_topic': '/traffic_test_z/map',
        'zones.source': 'none',
    }))
    client = rclpy.create_node('traffic_test_z_client')
    holds, poses = [], []
    client.create_subscription(Bool, '/amr_z1/traffic/hold', holds.append, _latched())
    client.create_subscription(PoseStamped, '/amr_z1/traffic/yield_pose', poses.append,
                               _latched())
    ex, spin = _own_executor((node, client))
    try:
        r = node._robots['amr_z1']
        cmd = RobotCommand(hold=True, reason='yield:incident1', yield_pose=(18.0, 8.0, 0.0))
        node._apply(r, cmd)
        assert r.hold is None and r.yield_pose == (18.0, 8.0, 0.0)
        assert spin(lambda: any(p.header.frame_id == 'map' for p in poses))
        assert not spin(lambda: bool(holds), 0.3)
        node._apply(r, cmd)                                         # 다음 주기: hold=true
        assert spin(lambda: bool(holds))
        assert [m.data for m in holds] == [True] and r.hold is True
    finally:
        ex.shutdown()
        node.destroy_node()
        client.destroy_node()


def test_hold_republish_due_rules():
    """재발행 판정 (D9): 평가 전 없음 · 포켓-먼저 미룸 중 false 없음 · 주기 경계 · 시계 되감김 · 끔."""
    due = hold_republish_due
    for pocket in (False, True):
        assert not due(None, pocket, None, 10.0, 1.0)            # 평가 전 (D5)
        assert not due(None, pocket, 0.0, 10.0, 1.0)
    assert not due(False, True, 0.0, 10.0, 1.0)                  # 포켓-먼저 미룸: false 를 다시 내면 포켓이 지워진다
    assert due(True, True, 0.0, 10.0, 1.0)                       # 양보 중 (포켓 + hold)
    assert due(False, False, 0.0, 10.0, 1.0) and due(True, False, 0.0, 10.0, 1.0)
    assert due(False, False, None, 0.0, 1.0)                     # 시각 기록 없음 → 바로
    assert due(False, False, 9.0, 10.0, 1.0)                     # 경계 포함
    assert not due(False, False, 9.0 + 1e-6, 10.0, 1.0)
    assert due(False, False, 50.0, 10.0, 1.0)                    # sim 재시작으로 시계가 뒤로
    for period in (0.0, -1.0):
        assert not due(False, False, 0.0, 10.0, period)          # 끔


def test_hold_republished_at_period_pocket_first(ros):
    """
    평가한 hold 를 주기마다 다시 낸다 (D9).

    바뀐 값 발행 뒤 첫 재발행까지도 한 주기. 포켓-먼저 미룸(포켓은 냈고 hold=true 는 다음 주기) 동안에는 false 를 다시
    내지 않는다 — 실행기는 false 를 받으면 포켓을 지운다. hold=true 를 다시 낼 때는 포켓을 먼저 다시 낸다.
    """
    period = 0.3
    node = TrafficManagerNode(namespace='/fleet_r', parameter_overrides=_params({
        'robot_ids': 'amr_r1', 'update_rate_hz': 20.0, 'map_topic': '/traffic_test_r/map',
        'zones.source': 'none', 'hold_republish_s': period,
    }))
    ex, spin = _own_executor((node,))
    calls = []
    r = node._robots['amr_r1']
    pose_pub, hold_pub = r.pub_yield.publish, r.pub_hold.publish
    r.pub_yield.publish = lambda m: (calls.append(('pose', m.header.frame_id, node._now())),
                                     pose_pub(m))
    r.pub_hold.publish = lambda m: (calls.append(('hold', m.data, node._now())), hold_pub(m))
    try:
        # 평가 전(None): 주기가 돌아도 재발행 없음 (이 로봇은 관측되지 않고 구독자도 없다 → D6 해제도 없음)
        spin(lambda: False, 2 * period)
        assert calls == [] and node.republished == 0
        node._publish_hold(r, False)                         # 평가한 값 (바뀐 값 발행)
        t0 = calls[-1][2]
        spin(lambda: False, 4 * period + 0.05)
        holds = [c for c in calls if c[0] == 'hold']
        assert [c[1] for c in holds] == [False] * len(holds) and len(holds) >= 4
        times = [c[2] for c in holds]
        assert times[0] == t0
        gaps = [b - a for a, b in zip(times, times[1:])]
        assert min(gaps) >= period - 0.02, gaps              # 주기마다 (매 평가 주기가 아니다)
        assert max(gaps) <= 2 * period + 0.1, gaps            # 공유 부하 여유
        assert node.republished == len(holds) - 1
        # 포켓-먼저 미룸: 포켓만 나가고 hold 는 다음 평가 → 그동안 false 재발행 없음
        cmd = RobotCommand(True, 'yield:incident1', (18.0, 8.0, 0.0), None, 1)
        node._apply(r, cmd)
        n_before = len(calls)
        assert calls[-1][:2] == ('pose', 'map') and r.hold is False
        spin(lambda: False, 3 * period)
        assert calls[n_before:] == [], calls[n_before:]
        node._apply(r, cmd)                                  # 다음 평가: hold=true (바뀐 값)
        assert calls[-1][:2] == ('hold', True)
        n_change = len(calls)
        spin(lambda: False, 3 * period + 0.05)
        again = [c[:2] for c in calls[n_change:]]
        assert len(again) >= 4 and again == [('pose', 'map'), ('hold', True)] * (len(again) // 2)
        assert calls[n_change][2] - calls[n_change - 1][2] >= period - 0.02
    finally:
        ex.shutdown()
        node.destroy_node()


def test_hold_matching_logged_and_incompatible_not_counted(ros):
    """
    로봇별 hold writer 의 DDS 매칭 구독자 수가 바뀌면 기록한다 (D10).

    get_subscription_count 는 그래프(발견)가 아니라 매칭 수다: QoS 가 맞지 않는 구독자(deadline 요구 — 발행자는
    무한)는 그래프에는 보이고 매칭에는 들지 않는다 (hold_failsafe.md §7 E3).
    """
    node = TrafficManagerNode(namespace='/fleet_m', parameter_overrides=_params({
        'robot_ids': 'amr_m1,amr_m2', 'update_rate_hz': 20.0, 'map_topic': '/traffic_test_m/map',
        'zones.source': 'none',
    }))
    client = rclpy.create_node('traffic_test_m_client')
    ex, spin = _own_executor((node, client))
    try:
        assert spin(lambda: False, 0.3) is False and list(node.hold_match_log) == []
        sub = client.create_subscription(Bool, '/amr_m1/traffic/hold', lambda m: None, _latched())
        assert spin(lambda: [e[1:] for e in node.hold_match_log] == [('amr_m1', 0, 1)])
        strict = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            deadline=Duration(seconds=1))
        client.create_subscription(Bool, '/amr_m2/traffic/hold', lambda m: None, strict)
        assert spin(lambda: node.count_subscribers('/amr_m2/traffic/hold') == 1)
        spin(lambda: False, 0.3)
        assert node._robots['amr_m2'].pub_hold.get_subscription_count() == 0
        assert node._robots['amr_m2'].hold_matched == 0
        assert all(e[1] != 'amr_m2' for e in node.hold_match_log)
        client.destroy_subscription(sub)
        assert spin(lambda: [e[1:] for e in node.hold_match_log][-1:] == [('amr_m1', 1, 0)])
        assert node._robots['amr_m1'].hold_matched == 0
    finally:
        ex.shutdown()
        node.destroy_node()
        client.destroy_node()
