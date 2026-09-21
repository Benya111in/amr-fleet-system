"""로거 노드 콜백 단위 테스트 — rclpy 가 있는 환경(컨테이너 colcon test)에서만 돈다."""

import math

from amr_evaluation import io
import pytest

rclpy = pytest.importorskip('rclpy')
nav_msgs = pytest.importorskip('nav_msgs.msg')
geometry_msgs = pytest.importorskip('geometry_msgs.msg')


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield rclpy
    rclpy.try_shutdown()


def _overrides(tmp_path, **extra):
    from rclpy.parameter import Parameter
    values = {'output_dir': str(tmp_path), 'run_name': 'run_t', 'report_period': 0.0}
    values.update(extra)
    return [Parameter(k, value=v) for k, v in values.items()]


def _odom(t, x, y, yaw=0.0, v=0.0, w=0.0):
    msg = nav_msgs.Odometry()
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    msg.twist.twist.linear.x = float(v)
    msg.twist.twist.angular.z = float(w)
    return msg


def _path(points):
    msg = nav_msgs.Path()
    for x, y in points:
        ps = geometry_msgs.PoseStamped()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        msg.poses.append(ps)
    return msg


def test_pose_error_logger_node(ros, tmp_path):
    from amr_evaluation.pose_error_logger import PoseErrorLogger
    node = PoseErrorLogger(parameter_overrides=_overrides(tmp_path))
    try:
        for i in range(101):
            node.on_ground_truth(_odom(i * 0.02, i * 0.02, 0.0, v=1.0))
        for i in range(10, 90):
            node.on_estimate(_odom(i * 0.02 + 0.005, i * 0.02 + 0.005 + 0.01, 0.0))
        node.flush()
        assert node.rows == 80
        assert '80 샘플' in node.progress_text()
        # 스탬프 0 인 메시지는 노드 시각으로 대체된다 (GT 범위 밖이라 폐기)
        node.on_estimate(_odom(0.0, 0.0, 0.0))
        node.flush()
        assert node.pairer.dropped >= 0
    finally:
        node.finish()
        node.destroy_node()
    rows = io.read_csv(tmp_path / 'run_t' / io.POSE_FILE)
    assert len(rows) == 80
    assert all(abs(r['error'] - 0.01) < 1e-6 for r in rows)
    assert {r['segment'] for r in rows} == {'직선'}


def test_cte_logger_node(ros, tmp_path):
    from amr_evaluation.cte_logger import CteLogger
    node = CteLogger(parameter_overrides=_overrides(tmp_path, log_rate=0.0))
    try:
        node.on_ground_truth(_odom(0.0, 0.0, 1.0))        # 경로 없음 → 건너뜀
        assert node.skipped_no_path == 1
        node.on_path(_path([(0, 0), (10, 0)]))
        assert node.paths_received == 1
        node.on_ground_truth(_odom(1.0, 5.0, 0.3))
        node.on_ground_truth(_odom(1.05, 6.0, -0.2))
        assert node.rows == 2
        node.on_path(_path([(0, 0)]))                      # 정점 부족 → 경로 비움
        node.on_ground_truth(_odom(2.0, 1.0, 1.0))
        assert node.rows == 2 and node.skipped_no_path == 2
        assert '2 샘플' in node.progress_text()
    finally:
        node.finish()
        node.destroy_node()
    rows = io.read_csv(tmp_path / 'run_t' / io.CTE_FILE)
    assert [r['cte'] for r in rows] == pytest.approx([0.3, -0.2])
    assert rows[0]['planned_x'] == 5.0 and rows[0]['segment'] == '직선'


def test_cte_logger_decimates(ros, tmp_path):
    from amr_evaluation.cte_logger import CteLogger
    node = CteLogger(parameter_overrides=_overrides(tmp_path, log_rate=10.0))
    try:
        node.on_path(_path([(0, 0), (10, 0)]))
        for i in range(50):
            node.on_ground_truth(_odom(1.0 + i * 0.02, i * 0.02, 0.0))   # 50 Hz 입력, 1 s
        assert 9 <= node.rows <= 11
    finally:
        node.finish()
        node.destroy_node()


def test_response_time_logger_pose_mode(ros, tmp_path):
    from amr_evaluation.response_time_logger import ResponseTimeLogger
    node = ResponseTimeLogger(parameter_overrides=_overrides(tmp_path, cmd_type='pose'))
    try:
        node.on_motion(_odom(0.0, 0.0, 0.0, v=0.0))
        goal = geometry_msgs.PoseStamped()
        goal.header.stamp.sec = 1
        node.on_goal_pose(goal)
        node.on_motion(_odom(1.05, 0.0, 0.0, v=0.0))
        node.on_motion(_odom(1.15, 0.0, 0.0, v=0.3))
        assert node.rows == 1 and node.commands == 1
        assert '1 샘플' in node.progress_text()
    finally:
        node.finish()
        node.destroy_node()
    rows = io.read_csv(tmp_path / 'run_t' / io.RESPONSE_FILE)
    assert rows[0]['latency_ms'] == pytest.approx(150.0)
    assert rows[0]['cmd_id'] == 'goal_1'


def test_response_time_logger_task_mode(ros, tmp_path):
    amr_msgs = pytest.importorskip('amr_msgs.msg')
    from amr_evaluation.response_time_logger import ResponseTimeLogger
    node = ResponseTimeLogger(parameter_overrides=_overrides(tmp_path, robot_id='amr_01'))
    try:
        node.on_motion(_odom(0.0, 0.0, 0.0))
        task = amr_msgs.Task()
        task.task_id = 't1'
        task.robot_id = 'amr_02'
        task.status = amr_msgs.Task.STATUS_IN_PROGRESS
        task.header.stamp.sec = 2
        node.on_task(task)                               # 다른 로봇 → 무시
        task.robot_id = 'amr_01'
        task.status = amr_msgs.Task.STATUS_PENDING
        node.on_task(task)                               # PENDING → 명령 아님
        task.status = amr_msgs.Task.STATUS_IN_PROGRESS
        node.on_task(task)                               # 전이 → 명령
        node.on_task(task)                               # 반복 IN_PROGRESS → 무시
        assert node.commands == 1
        node.on_motion(_odom(2.2, 0.0, 0.0, w=0.5))
        assert node.rows == 1
    finally:
        node.finish()
        node.destroy_node()
    rows = io.read_csv(tmp_path / 'run_t' / io.RESPONSE_FILE)
    assert rows[0]['cmd_id'] == 't1' and rows[0]['latency_ms'] == pytest.approx(200.0)


def test_response_time_logger_twist_mode(ros, tmp_path):
    from amr_evaluation.response_time_logger import ResponseTimeLogger
    node = ResponseTimeLogger(parameter_overrides=_overrides(
        tmp_path, cmd_type='pose', motion_type='twist', motion_topic='cmd_vel',
        motion_threshold=0.001))
    try:
        node.on_twist(geometry_msgs.Twist())             # 정지 명령 → 휴지 상태
        goal = geometry_msgs.PoseStamped()
        now = node.now_sec()
        goal.header.stamp.sec = int(now)
        goal.header.stamp.nanosec = int((now - int(now)) * 1e9)
        node.on_goal_pose(goal)
        cmd = geometry_msgs.Twist()
        cmd.linear.x = 0.01
        node.on_twist(cmd)                               # 첫 cmd_vel ≠ 0 → 응답
        assert node.rows == 1
    finally:
        node.finish()
        node.destroy_node()
    rows = io.read_csv(tmp_path / 'run_t' / io.RESPONSE_FILE)
    assert 0.0 <= rows[0]['latency_ms'] < 1000.0


@pytest.mark.parametrize('bad', [{'cmd_type': 'bogus'}, {'motion_type': 'bogus'}])
def test_response_time_logger_rejects_bad_types(ros, tmp_path, bad):
    from amr_evaluation.response_time_logger import ResponseTimeLogger
    with pytest.raises(ValueError):
        ResponseTimeLogger(parameter_overrides=_overrides(tmp_path, **bad))


def test_cpu_sampler_node(ros, tmp_path):
    from amr_evaluation.cpu_sampler import CpuSampler, cpu_columns
    stat = tmp_path / 'stat'
    stat.write_text('cpu  100 0 100 800 0 0 0 0 0 0\ncpu1 50 0 50 400 0 0 0 0 0 0\n'
                    'cpu0 50 0 50 400 0 0 0 0 0 0\n', encoding='utf-8')
    node = CpuSampler(parameter_overrides=_overrides(tmp_path, proc_stat_path=str(stat)))
    try:
        node.sample()                                    # 첫 샘플은 기준점만
        assert node.writer is None
        stat.write_text('cpu  200 0 200 1600 0 0 0 0 0 0\ncpu1 100 0 100 800 0 0 0 0 0 0\n'
                        'cpu0 100 0 100 800 0 0 0 0 0 0\n', encoding='utf-8')
        node.sample()
        assert node.rows == 1
        assert node.writer.columns == ['timestamp', 'cpu_total_percent', 'cpu0', 'cpu1']
        stat.write_text('cpu  300 0 300 2400 0 0 0 0 0 0\ncpu0 150 0 150 1200 0 0 0 0 0 0\n',
                        encoding='utf-8')
        node.sample()                                    # cpu1 사라짐 → NaN
        assert '2 샘플 (2 코어)' == node.progress_text()
    finally:
        node.finish()
        node.destroy_node()
    rows = io.read_csv(tmp_path / 'run_t' / io.CPU_FILE)
    assert rows[0]['cpu_total_percent'] == pytest.approx(20.0)
    assert rows[0]['cpu0'] == pytest.approx(20.0)
    assert math.isnan(rows[1]['cpu1'])
    assert cpu_columns({'cpu': [], 'cpu10': [], 'cpu2': []}) == \
        ['timestamp', 'cpu_total_percent', 'cpu2', 'cpu10']
