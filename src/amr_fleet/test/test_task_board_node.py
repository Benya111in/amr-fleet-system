"""
fleet_manager 의 /fleet/task_board 경로와 권위자 표식 (보너스 4 P0, 한 프로세스).

- 관리자보다 먼저 낸 게시물도 받는다 (TRANSIENT_LOCAL 이력 — 재기동한 관리자 흉내).
- 재전송 · 두 경로(task_board · task_request)로 온 같은 작업은 한 번만 접수하고 알림을 내지 않는다.
- 같은 판 다른 내용 · 낮은 판 · 높은 판 · task_id 없음 · PENDING 아닌 게시물은 fleet/INVALID_TASK.
- task_events 마다 (epoch, leader_id, seq): epoch 는 파라미터, seq 는 1 씩 는다.
rclpy·amr_msgs 가 없으면 건너뛴다.
"""

import json
import time

import pytest

try:
    from amr_fleet.fleet_manager_node import FleetManagerNode
    from amr_fleet.task_board import BOARD_TOPIC
    from amr_fleet.task_msg import spec_to_msg
    from amr_fleet.task_schema import Pose2D, TaskSpec
    from amr_msgs.msg import Task
    from diagnostic_msgs.msg import DiagnosticArray
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='rclpy/amr_msgs 가 없는 환경')

ROBOT = 'amr_b1'          # 등록만 한다 (robot_state 없음 → 할당 후보가 없어 작업은 PENDING 에 머문다)


def _params(d):
    return [Parameter(k, value=v) for k, v in d.items()]


def _posting(task_id, x=1.0, version=0, status=Task.STATUS_PENDING):
    spec = TaskSpec(task_id=task_id, pickup=Pose2D(x, 2.0, 0.5), dropoff=Pose2D(5.0, 6.0),
                    item_type='small', item_mass=2.0, priority=20, version=version)
    msg = spec_to_msg(spec)
    msg.status = status
    return msg


@pytest.fixture(scope='module')
def board(tmp_path_factory):
    rclpy.init()
    client = rclpy.create_node('board_test_client')
    qos_board = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub_board = client.create_publisher(Task, BOARD_TOPIC, qos_board)
    # 관리자가 뜨기 전에 낸다 — 이력으로 받아야 한다
    pub_board.publish(_posting('src.000001'))
    pub_board.publish(_posting('src.000002', x=3.0))
    got = {'events': [], 'alerts': []}
    rel = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
    client.create_subscription(Task, '/fleet/task_events', got['events'].append, rel)
    client.create_subscription(DiagnosticArray, '/fleet/alerts', got['alerts'].append, rel)
    pub_json = client.create_publisher(String, '/fleet/task_request', rel)
    manager = FleetManagerNode(parameter_overrides=_params({
        'robot_ids': ROBOT, 'discover_robots': False, 'simulate_latency': False,
        'log_dir': str(tmp_path_factory.mktemp('board')), 'epoch': 7}))
    executor = SingleThreadedExecutor()
    for node in (manager, client):
        executor.add_node(node)

    def spin_until(pred, timeout=15.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            if pred():
                return True
        return False

    yield dict(manager=manager, got=got, board=pub_board, json=pub_json, spin=spin_until)
    executor.shutdown(timeout_sec=2.0)
    for node in (manager, client):
        node.destroy_node()
    rclpy.try_shutdown()


def _pending(got, task_id):
    return [e for e in got['events'] if e.task_id == task_id and e.status == Task.STATUS_PENDING]


def _invalid(got, task_id):
    out = []
    for a in got['alerts']:
        for s in a.status:
            values = {kv.key: kv.value for kv in s.values}
            if s.name == 'fleet/INVALID_TASK' and values.get('task_id') == task_id:
                out.append(s.message)
    return out


def test_late_manager_receives_board_history(board):
    got = board['got']
    assert board['spin'](lambda: _pending(got, 'src.000001') and _pending(got, 'src.000002'))
    assert board['manager']._sm.get('src.000002').spec.pickup.x == pytest.approx(3.0)


def test_retransmission_and_second_path_are_ignored_silently(board):
    got, m = board['got'], board['manager']
    assert board['spin'](lambda: _pending(got, 'src.000001'))
    before = m.board_duplicates
    board['board'].publish(_posting('src.000001'))                        # 재전송
    board['json'].publish(String(data=json.dumps({                       # 같은 작업, JSON 경로
        'task_id': 'src.000001', 'priority': 20, 'item_type': 'small',
        'pickup': {'x': 1.0, 'y': 2.0, 'yaw': 0.5}, 'dropoff': {'x': 5.0, 'y': 6.0}})))
    assert board['spin'](lambda: m.board_duplicates >= before + 2)
    assert len(_pending(got, 'src.000001')) == 1 and not _invalid(got, 'src.000001')


@pytest.mark.parametrize('task_id, kwargs, reason', [
    ('src.000001', {'x': 9.0}, '내용이 다르다'),
    ('src.000003', {'status': Task.STATUS_COMPLETED}, 'PENDING'),
])
def test_conflicting_or_closed_postings_are_rejected(board, task_id, kwargs, reason):
    got = board['got']
    board['board'].publish(_posting(task_id, **kwargs))
    assert board['spin'](lambda: any(reason in t for t in _invalid(got, task_id)))
    assert len(_pending(got, task_id)) == (1 if task_id == 'src.000001' else 0)


def test_versions_and_missing_id(board):
    got = board['got']
    board['board'].publish(_posting('src.000010', version=2))
    assert board['spin'](lambda: _pending(got, 'src.000010'))
    board['board'].publish(_posting('src.000010', version=1))           # 옛 판
    board['board'].publish(_posting('src.000010', version=3))           # 새 판 (갱신 미지원)
    assert board['spin'](lambda: len(_invalid(got, 'src.000010')) >= 2)
    texts = _invalid(got, 'src.000010')
    assert any('옛 판' in t for t in texts) and any('새 판' in t for t in texts)
    n = board['manager'].invalid_tasks
    board['board'].publish(_posting(''))
    assert board['spin'](lambda: board['manager'].invalid_tasks > n)
    assert len(_pending(got, 'src.000010')) == 1


def test_task_events_carry_epoch_leader_and_increasing_seq(board):
    got = board['got']
    assert board['spin'](lambda: len(got['events']) >= 3)
    events = list(got['events'])
    assert all(e.epoch == 7 and e.leader_id == '/fleet/fleet_manager_node' for e in events)
    seqs = [e.seq for e in events]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs) and seqs[0] >= 1
    assert board['manager']._command.epoch == 7
