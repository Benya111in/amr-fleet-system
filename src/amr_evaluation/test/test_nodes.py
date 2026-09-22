"""로거 노드 콜백 단위 테스트 — rclpy 가 있는 환경(컨테이너 colcon test)에서만 돈다."""

import importlib.util
import math
import os
from pathlib import Path
import time

from amr_evaluation import io
import pytest

rclpy = pytest.importorskip('rclpy')
nav_msgs = pytest.importorskip('nav_msgs.msg')
geometry_msgs = pytest.importorskip('geometry_msgs.msg')
std_msgs = pytest.importorskip('std_msgs.msg')
action_msgs = pytest.importorskip('action_msgs.msg')


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield rclpy
    rclpy.try_shutdown()


def _overrides(tmp_path, **extra):
    """테스트 스탬프는 시뮬 시간(수 초)이므로 기본 use_sim_time=True (launch 기본값과 같다)."""
    from rclpy.parameter import Parameter
    values = {'output_dir': str(tmp_path), 'run_name': 'run_t', 'report_period': 0.0,
              'use_sim_time': True}
    values.update(extra)
    return [Parameter(k, value=v) for k, v in values.items()]


def _stamp(msg, t):
    msg.sec = int(t)
    msg.nanosec = int(round((t - int(t)) * 1e9))


def _odom(t, x, y, yaw=0.0, v=0.0, w=0.0):
    msg = nav_msgs.Odometry()
    _stamp(msg.header.stamp, t)
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    msg.twist.twist.linear.x = float(v)
    msg.twist.twist.angular.z = float(w)
    return msg


def _path(points, t=0.0):
    msg = nav_msgs.Path()
    _stamp(msg.header.stamp, t)
    for x, y in points:
        ps = geometry_msgs.PoseStamped()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        msg.poses.append(ps)
    return msg


def _nav_status(*goals):
    msg = action_msgs.GoalStatusArray()
    for stamp, status in goals:
        st = action_msgs.GoalStatus()
        _stamp(st.goal_info.stamp, stamp)
        st.status = status
        msg.status_list.append(st)
    return msg


LINE_5M = [(0.05 * i, 0.0) for i in range(101)]


def test_pose_error_logger_node(ros, tmp_path):
    from amr_evaluation.pose_error_logger import PoseErrorLogger
    node = PoseErrorLogger(parameter_overrides=_overrides(tmp_path, use_sim_time=True))
    try:
        for i in range(101):
            node.on_ground_truth(_odom(i * 0.02, i * 0.02, 0.0, v=1.0))
        for i in range(10, 90):
            node.on_estimate(_odom(i * 0.02 + 0.005, i * 0.02 + 0.005 + 0.01, 0.0))
        node.flush()
        assert node.rows == 80 and node.pairer.dropped == 0
        assert '80 샘플' in node.progress_text()
        assert not node.clock_mismatch                  # 시뮬 시계 노드 + 시뮬 스탬프
    finally:
        node.finish()
        node.destroy_node()
    rows = io.read_csv(tmp_path / 'run_t' / io.POSE_FILE)
    assert len(rows) == 80
    assert all(abs(r['error'] - 0.01) < 1e-6 for r in rows)
    assert {r['segment'] for r in rows} == {'직선'}


def test_pose_error_logger_zero_stamp_fallback_and_clock_check(ros, tmp_path):
    from amr_evaluation.pose_error_logger import PoseErrorLogger
    node = PoseErrorLogger(parameter_overrides=_overrides(tmp_path, use_sim_time=False))
    try:
        for i in range(1, 102):
            node.on_ground_truth(_odom(i * 0.02, i * 0.02, 0.0, v=1.0))
        # 시뮬 스탬프(0~2 s)를 벽시계 노드가 받음 → 스트림마다 세고 처음 한 번 경고
        assert node.clock_mismatch == {'ground_truth': 101}
        assert '시계 불일치 ground_truth 101' in node.clock_text()
        # 스탬프 0 → 노드 시각(벽시계)으로 대체: 대기열의 시각이 now 와 같다
        before = node.now_sec()
        node.on_estimate(_odom(0.0, 0.0, 0.0))
        after = node.now_sec()
        assert node.pairer.pending == 1
        assert before <= node.pairer._pending[-1].t <= after
        assert 'estimate' not in node.clock_mismatch   # 대체 시각은 노드 시계라 불일치 아님
        # GT(0~2 s) 보다 max_wait 넘게 앞선 시각 → 정확히 한 개 폐기, 행은 없음
        node.flush()
        assert node.pairer.dropped == 1 and node.pairer.pending == 0 and node.rows == 0
    finally:
        node.finish()
        node.destroy_node()


def test_cte_logger_node_basic_and_endpoint(ros, tmp_path):
    from amr_evaluation.cte_logger import CteLogger
    node = CteLogger(parameter_overrides=_overrides(tmp_path, log_rate=0.0))
    try:
        node.on_ground_truth(_odom(0.0, 0.0, 1.0, v=0.5))        # 경로 없음 → 건너뜀
        assert node.skipped_no_path == 1
        node.on_path(_path([(0, 0), (10, 0)], t=0.5))
        assert node.paths_received == 1
        node.on_ground_truth(_odom(1.0, 5.0, 0.3, v=0.5))
        node.on_ground_truth(_odom(1.05, 6.0, -0.2, v=0.5))
        assert node.rows == 2
        node.on_ground_truth(_odom(1.1, 10.15, 0.0, v=0.5))    # 끝점 너머: 종방향 거리 → 제외
        node.on_ground_truth(_odom(1.15, 6.0, 0.01))             # 정지 → 제외
        assert node.rows == 2
        assert node.gate.skipped['endpoint'] == 1 and node.gate.skipped['stopped'] == 1
        node.on_path(_path([(0, 0)]))                            # 정점 부족 → 경로 비움
        node.on_ground_truth(_odom(2.0, 1.0, 1.0, v=0.5))
        assert node.rows == 2 and node.skipped_no_path == 2
        assert '2 샘플' in node.progress_text() and 'endpoint 1' in node.progress_text()
    finally:
        node.finish()
        node.destroy_node()
    rows = io.read_csv(tmp_path / 'run_t' / io.CTE_FILE)
    assert [r['cte'] for r in rows] == pytest.approx([0.3, -0.2])
    assert rows[0]['planned_x'] == 5.0 and rows[0]['segment'] == '직선'


def test_cte_logger_parked_robot_does_not_score_stale_plan(ros, tmp_path):
    """리뷰 재현(r2_cte_idle): 5 m 직선 1 cm 오차로 주행 → 끝점 0.15 m 너머/도킹 위치에서 10 s 대기."""
    from amr_evaluation import analyze
    from amr_evaluation.cte_logger import CteLogger
    for i, park in enumerate(((5.15, 0.0), (5.6, 0.05), (4.9, 0.08))):
        node = CteLogger(parameter_overrides=_overrides(tmp_path, run_name=f'park{i}'))
        try:
            node.on_path(_path(LINE_5M, t=0.5))
            t = 1.0
            while t < 11.0:
                node.on_ground_truth(_odom(t, 0.5 * (t - 1.0), 0.01, v=0.5))
                t += 0.02
            driving = node.rows
            while t < 21.0:
                node.on_ground_truth(_odom(t, park[0], park[1]))
                t += 0.02
            assert node.rows == driving                   # 주차 중 표본은 하나도 없다
            assert node.gate.skipped['stopped'] >= 150    # 20 Hz 데시메이션, 10 s
        finally:
            node.finish()
            node.destroy_node()
        rows = io.read_csv(node.writer.path)
        st = next(r for r in analyze.analyze_cte(rows, analyze.Thresholds())
                  if r.segment == '직선')
        assert st.value == pytest.approx(0.01, abs=1e-4) and st.passed


def test_cte_logger_nav_status_and_phase(ros, tmp_path):
    from amr_evaluation.cte_logger import CteLogger
    node = CteLogger(parameter_overrides=_overrides(tmp_path, log_rate=0.0))
    try:
        node.on_path(_path(LINE_5M, t=10.0))
        node.on_nav_status(_nav_status((9.0, 2)))            # 목표 EXECUTING
        node.on_ground_truth(_odom(11.0, 1.0, 0.02, v=0.5))
        assert node.rows == 1
        node.on_nav_status(_nav_status((9.0, 4)))            # SUCCEEDED → 도킹 구동 (경로 아님)
        node.on_ground_truth(_odom(12.0, 4.9, 0.3, v=0.1))
        assert node.rows == 1 and node.gate.skipped['nav_inactive'] == 1
        node.on_nav_status(_nav_status((9.0, 4), (20.0, 1)))  # 새 목표, 경로는 아직 이전 것
        node.on_ground_truth(_odom(20.5, 2.0, 1.0, v=0.5))
        assert node.rows == 1 and node.gate.skipped['stale_plan'] == 1
        node.on_path(_path([(2.0, 1.0), (7.0, 1.0)], t=20.2))
        node.on_ground_truth(_odom(21.0, 3.0, 1.03, v=0.5))
        assert node.rows == 2
        node.on_phase(std_msgs.String(data='loading'))
        node.on_ground_truth(_odom(22.0, 3.5, 1.0, v=0.5))
        assert node.rows == 2 and node.gate.skipped['phase'] == 1
    finally:
        node.finish()
        node.destroy_node()
    rows = io.read_csv(tmp_path / 'run_t' / io.CTE_FILE)
    assert [r['cte'] for r in rows] == pytest.approx([0.02, 0.03])


def test_cte_logger_plan_timeout_default(ros, tmp_path):
    """상태 원천이 없는 구성: 경로를 받은 지 plan_timeout(기본 3 s, 노드 시계) 이 지나면 stale_plan."""
    from amr_evaluation.cte_logger import CteLogger
    node = CteLogger(parameter_overrides=_overrides(tmp_path, log_rate=0.0))
    try:
        assert node.gate.config.plan_timeout == 3.0
        node.on_path(_path(LINE_5M, t=1.0))
        node.on_ground_truth(_odom(1.1, 1.0, 0.01, v=0.5))
        assert node.rows == 1
        node.gate.plan_rx = node.now_sec() - 3.5          # 재계획이 멈춘 뒤 3.5 s
        node.on_ground_truth(_odom(1.2, 1.05, 0.01, v=0.5))
        assert node.rows == 1 and node.gate.skipped['stale_plan'] == 1
    finally:
        node.finish()
        node.destroy_node()


def test_cte_logger_decimates_and_recovers_from_sim_reset(ros, tmp_path):
    from amr_evaluation.cte_logger import CteLogger
    node = CteLogger(parameter_overrides=_overrides(tmp_path, log_rate=10.0))
    try:
        node.on_path(_path([(0, 0), (10, 0)]))
        for i in range(50):
            node.on_ground_truth(_odom(30.0 + i * 0.02, i * 0.02, 0.0, v=0.5))   # 50 Hz, 1 s
        assert 9 <= node.rows <= 11
        before = node.rows
        node.on_ground_truth(_odom(1.0, 0.5, 0.0, v=0.5))    # 시뮬 리셋 → 경로도 무효
        assert node.resets == 1 and node.gate.plan_rx is None
        node.on_path(_path([(0, 0), (10, 0)]))
        for i in range(50):
            node.on_ground_truth(_odom(1.1 + i * 0.02, i * 0.02, 0.0, v=0.5))
        assert node.rows - before >= 9                       # 리셋 뒤에도 계속 기록
        assert '시뮬 리셋 1' in node.progress_text()
    finally:
        node.finish()
        node.destroy_node()


def test_cte_logger_namespaced_file(ros, tmp_path):
    from amr_evaluation.cte_logger import CteLogger
    a = CteLogger(parameter_overrides=_overrides(tmp_path), namespace='/amr_01')
    b = CteLogger(parameter_overrides=_overrides(tmp_path), namespace='/amr_02')
    c = CteLogger(parameter_overrides=_overrides(tmp_path), namespace='/amr_02')   # 같은 이름 재기동
    try:
        assert a.writer.path.name == 'cte_amr_01.csv'
        assert b.writer.path.name == 'cte_amr_02.csv'
        assert c.writer.path.name == 'cte_amr_02_1.csv'
    finally:
        for n in (a, b, c):
            n.finish()
            n.destroy_node()


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
        assert '1 샘플' in node.progress_text() and 'goal 1' in node.progress_text()
    finally:
        node.finish()
        node.destroy_node()
    rows = io.read_csv(tmp_path / 'run_t' / io.RESPONSE_FILE)
    assert rows[0]['latency_ms'] == pytest.approx(150.0)
    assert rows[0]['cmd_id'] == 'goal_1' and rows[0]['cmd_source'] == 'goal'
    assert list(rows[0]) == io.RESPONSE_COLUMNS


def _task(amr_msgs, task_id, robot_id, status, header=0.0, pickup=0.0, dropoff=0.0):
    task = amr_msgs.Task()
    task.task_id = task_id
    task.robot_id = robot_id
    task.status = status
    _stamp(task.header.stamp, header)
    _stamp(task.pickup_pose.header.stamp, pickup)
    _stamp(task.dropoff_pose.header.stamp, dropoff)
    return task


def test_response_time_logger_task_mode_dispatch_stamp(ros, tmp_path):
    amr_msgs = pytest.importorskip('amr_msgs.msg')
    from amr_evaluation.response_time_logger import ResponseTimeLogger
    ip, pending = amr_msgs.Task.STATUS_IN_PROGRESS, amr_msgs.Task.STATUS_PENDING
    node = ResponseTimeLogger(parameter_overrides=_overrides(tmp_path, robot_id='/amr_01/'))
    try:
        assert node.robot_id == 'amr_01'
        node.on_motion(_odom(0.0, 0.0, 0.0))
        node.on_task(_task(amr_msgs, 't0', 'amr_02', ip, header=2.0))      # 다른 로봇 → 무시
        node.on_task(_task(amr_msgs, 't1', 'amr_01', pending, header=1.0))
        # 명령(할당 요청) 1.80 s → 수락 뒤 IN_PROGRESS 2.00 s → 첫 움직임 2.20 s
        node.on_task(_task(amr_msgs, 't1', 'amr_01', ip, header=2.0, pickup=1.8, dropoff=1.5))
        node.on_task(_task(amr_msgs, 't1', 'amr_01', ip, header=2.0, pickup=1.8))   # 반복 무시
        assert node.commands == 1 and node.sources == {'dispatch': 1}
        node.on_motion(_odom(2.2, 0.0, 0.0, w=0.5))
        assert node.rows == 1
        # 옛 fleet: pickup stamp == header stamp → 전이 시각으로 대체 (경고 한 번)
        node.on_motion(_odom(3.0, 0.0, 0.0))
        node.on_task(_task(amr_msgs, 't2', '/amr_01', ip, header=4.0, pickup=4.0))
        node.on_motion(_odom(4.1, 0.2, 0.0, v=0.2))
        assert node.rows == 2 and node.sources == {'dispatch': 1, 'event': 1}
    finally:
        node.finish()
        node.destroy_node()
    rows = io.read_csv(tmp_path / 'run_t' / io.RESPONSE_FILE)
    assert rows[0]['cmd_id'] == 't1' and rows[0]['latency_ms'] == pytest.approx(400.0)
    assert rows[0]['cmd_source'] == 'dispatch' and rows[0]['cmd_time'] == pytest.approx(1.8)
    assert rows[1]['cmd_source'] == 'event' and rows[1]['latency_ms'] == pytest.approx(100.0)


@pytest.mark.parametrize('mode, expected_ms, source', [
    ('dropoff', 700.0, 'request'), ('header', 200.0, 'event'), ('pickup', 400.0, 'dispatch')])
def test_response_time_logger_cmd_stamp_modes(ros, tmp_path, mode, expected_ms, source):
    amr_msgs = pytest.importorskip('amr_msgs.msg')
    from amr_evaluation.response_time_logger import ResponseTimeLogger
    node = ResponseTimeLogger(parameter_overrides=_overrides(tmp_path, cmd_stamp=mode))
    try:
        node.on_motion(_odom(0.0, 0.0, 0.0))
        node.on_task(_task(amr_msgs, 't1', 'amr_01', amr_msgs.Task.STATUS_IN_PROGRESS,
                           header=2.0, pickup=1.8, dropoff=1.5))
        node.on_motion(_odom(2.2, 0.3, 0.0, v=0.3))
    finally:
        node.finish()
        node.destroy_node()
    row = io.read_csv(node.writer.path)[0]
    assert row['latency_ms'] == pytest.approx(expected_ms) and row['cmd_source'] == source


def test_response_time_logger_late_event_and_clock_mismatch(ros, tmp_path):
    amr_msgs = pytest.importorskip('amr_msgs.msg')
    from amr_evaluation.response_time_logger import ResponseTimeLogger
    ip = amr_msgs.Task.STATUS_IN_PROGRESS
    node = ResponseTimeLogger(parameter_overrides=_overrides(tmp_path, use_sim_time=True))
    try:
        for i in range(10):
            node.on_motion(_odom(1.0 + i * 0.02, 0.0, 0.0))
        for i in range(10, 20):                                  # 1.20 s 부터 주행
            node.on_motion(_odom(1.0 + i * 0.02, 0.1, 0.0, v=0.3))
        # IN_PROGRESS 이벤트가 출발보다 늦게 도착: 명령 1.05 s → 소급해서 150 ms
        node.on_task(_task(amr_msgs, 'late', 'amr_01', ip, header=1.1, pickup=1.05))
        assert node.rows == 1
        # fleet 만 use_sim_time:=false (벽시계 에포크) → 짝짓지 않고 경고
        node.on_task(_task(amr_msgs, 'wall', 'amr_01', ip, header=1.79e9 + 0.2, pickup=1.79e9))
        assert node.matcher.clock_mismatch == 1 and node.matcher.pending == 0
        assert node.clock_mismatch == {'command': 1}
        assert '시계 불일치 1' in node.progress_text()
    finally:
        node.finish()
        node.destroy_node()
    rows = io.read_csv(tmp_path / 'run_t' / io.RESPONSE_FILE)
    assert len(rows) == 1 and rows[0]['latency_ms'] == pytest.approx(150.0)


def test_response_time_logger_twist_mode_and_rtf(ros, tmp_path):
    from amr_evaluation.response_time_logger import ResponseTimeLogger
    node = ResponseTimeLogger(parameter_overrides=_overrides(
        tmp_path, cmd_type='pose', motion_type='twist', motion_topic='cmd_vel',
        motion_threshold=0.001, rtf_window=5.0, use_sim_time=False))
    try:
        # 벽시계 노드(use_sim_time false): RTF ≈ 1 을 쌓는다
        node.rtf.add(node.now_sec() - 2.0, time.monotonic() - 2.0)
        node.on_twist(geometry_msgs.Twist())             # 정지 명령 → 휴지 상태
        goal = geometry_msgs.PoseStamped()
        goal.header.stamp = node.get_clock().now().to_msg()
        node.on_goal_pose(goal)
        cmd = geometry_msgs.Twist()
        cmd.linear.x = 0.01
        node.on_twist(cmd)                               # 첫 cmd_vel ≠ 0 → 응답
        assert node.rows == 1
    finally:
        node.finish()
        node.destroy_node()
    row = io.read_csv(tmp_path / 'run_t' / io.RESPONSE_FILE)[0]
    assert 0.0 <= row['latency_ms'] < 1000.0
    assert row['rtf'] == pytest.approx(1.0, abs=0.05)
    assert row['latency_wall_ms'] == pytest.approx(row['latency_ms'] / row['rtf'], abs=1e-5)


@pytest.mark.parametrize('bad', [{'cmd_type': 'bogus'}, {'motion_type': 'bogus'},
                                 {'cmd_stamp': 'bogus'}])
def test_response_time_logger_rejects_bad_types(ros, tmp_path, bad):
    from amr_evaluation.response_time_logger import ResponseTimeLogger
    with pytest.raises(ValueError):
        ResponseTimeLogger(parameter_overrides=_overrides(tmp_path, **bad))


def test_cpu_sampler_node(ros, tmp_path):
    from amr_evaluation.cpu_sampler import cpu_columns, CpuSampler
    stat = tmp_path / 'stat'
    stat.write_text('cpu  100 0 100 800 0 0 0 0 0 0\ncpu1 50 0 50 400 0 0 0 0 0 0\n'
                    'cpu0 50 0 50 400 0 0 0 0 0 0\n', encoding='utf-8')
    proc = tmp_path / 'proc' / '77'
    proc.mkdir(parents=True)
    fields = ['S'] + ['0'] * 10 + ['100', '0'] + ['0'] * 6 + ['5']
    (proc / 'stat').write_text('77 (gz) ' + ' '.join(fields))
    (proc / 'cmdline').write_bytes(b'ruby\0/usr/bin/ign\0gazebo\0-s\0')
    cg = tmp_path / 'cg'
    cg.mkdir()
    (cg / 'cpu.stat').write_text('usage_usec 1000000\n')
    node = CpuSampler(parameter_overrides=_overrides(
        tmp_path, proc_stat_path=str(stat), proc_root=str(tmp_path / 'proc'),
        cgroup_root=str(cg), num_cpus=2, use_sim_time=False))
    try:
        node.sample()                                    # 첫 샘플은 기준점만
        assert node.writer is None
        stat.write_text('cpu  200 0 200 1600 0 0 0 0 0 0\ncpu1 100 0 100 800 0 0 0 0 0 0\n'
                        'cpu0 100 0 100 800 0 0 0 0 0 0\n', encoding='utf-8')
        fields[11] = str(100 + int(os.sysconf('SC_CLK_TCK')))    # +1 CPU·s
        (proc / 'stat').write_text('77 (gz) ' + ' '.join(fields))
        (cg / 'cpu.stat').write_text('usage_usec 2000000\n')
        node._prev_times = (node._prev_times[0], node._prev_times[1] - 1.0)   # 1 s 경과로
        node.sample()
        assert node.rows == 1
        assert node.writer.columns == [
            'timestamp', 'cpu_total_percent', 'cpu_ros_percent', 'cpu_gazebo_percent',
            'cpu_nav_percent', 'procs_ros', 'procs_gazebo', 'procs_nav', 'cpu_cgroup_percent',
            'rtf', 'load1', 'cpu0', 'cpu1']
        stat.write_text('cpu  300 0 300 2400 0 0 0 0 0 0\ncpu0 150 0 150 1200 0 0 0 0 0 0\n',
                        encoding='utf-8')
        node.sample()                                    # cpu1 사라짐 → NaN
        assert node.progress_text().startswith('2 샘플 (2 코어, ros ')
    finally:
        node.finish()
        node.destroy_node()
    rows = io.read_csv(tmp_path / 'run_t' / io.CPU_FILE)
    assert rows[0]['cpu_total_percent'] == pytest.approx(20.0)
    assert rows[0]['cpu0'] == pytest.approx(20.0)
    assert rows[0]['procs_gazebo'] == 1 and rows[0]['procs_nav'] == 0
    # 1 CPU·s / (≈1 s × 2 CPU) ≈ 50 % (실제 경과가 조금 더 길어 조금 작다)
    assert 40.0 < rows[0]['cpu_gazebo_percent'] <= 50.0
    assert 40.0 < rows[0]['cpu_cgroup_percent'] <= 50.0
    assert rows[0]['rtf'] >= 0.0 and rows[0]['load1'] >= 0.0
    assert math.isnan(rows[1]['cpu1'])
    assert cpu_columns({'cpu': [], 'cpu10': [], 'cpu2': []}) == \
        ['timestamp', 'cpu_total_percent', 'rtf', 'load1', 'cpu2', 'cpu10']


def test_launch_passes_string_parameters():
    """run_name:=20260922 같은 숫자 인자도 STRING 파라미터로 (예전엔 INTEGER 라 로거 4개가 죽었다)."""
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    share = Path(__file__).resolve().parents[1] / 'launch' / 'evaluation.launch.py'
    spec = importlib.util.spec_from_file_location('evaluation_launch', share)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ctx = LaunchContext()
    ctx.launch_configurations.update({'run_name': '20260922', 'robot_id': '0042'})
    out = evaluate_parameters(ctx, normalize_parameters(
        [{'run_name': mod._as_str('run_name'), 'robot_id': mod._as_str('robot_id')}]))
    assert list(out) == [{'run_name': '20260922', 'robot_id': '0042'}]
    desc = mod.generate_launch_description()
    args = {a.name: a for a in desc.entities if hasattr(a, 'default_value')}
    assert set(args) >= {'run_name', 'output_dir', 'namespace', 'robot_id', 'use_sim_time',
                         'with_cpu', 'params_file'}


@pytest.mark.parametrize('module', ['pose_error_logger', 'cte_logger', 'response_time_logger',
                                    'cpu_sampler'])
def test_main_spins_and_finishes(ros, tmp_path, monkeypatch, module):
    """main(): 노드 생성 → spin (Ctrl-C) → finish·destroy·shutdown. 실제 컨텍스트는 fixture 것을 쓴다."""
    import importlib
    mod = importlib.import_module(f'amr_evaluation.{module}')
    monkeypatch.setenv('ROS_WS', str(tmp_path))
    calls = []

    class FakeRclpy:
        @staticmethod
        def init(args=None):
            calls.append('init')

        @staticmethod
        def spin(node):
            calls.append(node.get_name())
            raise KeyboardInterrupt

        @staticmethod
        def try_shutdown():
            calls.append('shutdown')

    monkeypatch.setattr(mod, 'rclpy', FakeRclpy)
    mod.main()
    assert calls == ['init', module, 'shutdown']
    runs = list((tmp_path / 'logs' / 'eval').iterdir())
    assert len(runs) == 1 and runs[0].name.startswith('run_')
