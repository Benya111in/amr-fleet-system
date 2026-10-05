"""stack 구성(실제/대역 선택) · 대역 노드 로직 (메서드 직접 호출)."""

import json
import math
import time
from types import SimpleNamespace as NS
import unittest

from amr_itest import catalog, stack as stack_mod
from amr_itest import requirements as req
from amr_itest.scenario import Context, GAZEBO, KINEMATIC
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from launch.actions import IncludeLaunchDescription
import launch_testing.actions
from nav_msgs.msg import Odometry
import pytest
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool, String


@pytest.fixture
def fake_launch(tmp_path, monkeypatch):
    """모든 launch 파일이 설치된 것처럼 (IncludeLaunchDescription 은 실행 전에는 파일을 읽지 않는다)."""
    f = tmp_path / 'x.launch.py'
    f.write_text('')
    monkeypatch.setattr(req, 'launch_file', lambda pkg, name: f)
    return f


def test_component_stack_with_standins(itest_env, monkeypatch):
    monkeypatch.setenv('ITEST_STANDINS', 'always')
    monkeypatch.setenv('ITEST_FRAME_PREFIX', 'amr_01/')
    # 워크스페이스에 amr_evaluation 이 있어도 '미설치' 경로를 시험한다 (EKF 는 실제 설치본)
    monkeypatch.setattr(req, 'has_executable', lambda pkg, exe: pkg == 'robot_localization')
    ctx = Context(catalog.get(9)).begin()
    st = stack_mod.Stack(ctx, KINEMATIC)
    st.simulator(1.0, 2.0, 0.5)
    st.description()
    st.localization(amcl=True, ekf=True)
    st.velocity_chain()
    st.task_executor()
    assert st.eval_logger('pose_error_logger', {}) is False
    ld = st.launch_description()
    assert isinstance(ld.entities[0], launch_testing.actions.ReadyToTest)
    comp = st.components
    for key in ('simulator', 'imu_filter', 'scan_filter', 'description', 'wheel_odometry',
                'amcl', 'velocity_profiler', 'safety', 'task_executor'):
        assert comp[key] == stack_mod.STANDIN, key
    assert comp['ekf'] == stack_mod.REAL and comp['evaluation'] == 'harness-only'
    assert st.drive_topic == 'cmd_vel_nav' and 'safety=standin' in st.summary()
    assert 'standin_kinematic_sim' in st.standin_names
    assert ctx.record.data['components'] == comp
    st.add('entity')
    assert st.entities[-1] == 'entity'


def test_choose_policies_and_real_nodes(itest_env, monkeypatch):
    ctx = Context(catalog.get(13))
    st = stack_mod.Stack(ctx, KINEMATIC)
    assert st.choose('x', [req.package('rclpy')]) == stack_mod.REAL
    assert st.choose('y', [req.package('no_such_pkg_xyz')]) == stack_mod.STANDIN
    st.real_node('amr_localization', 'wheel_odometry_node', params=[{'a': 1}])
    original = req.has_executable
    monkeypatch.setattr(req, 'has_executable', lambda p, e: True)
    st.velocity_chain(profiler=False, safety=True)
    assert st.drive_topic == 'cmd_vel_smoothed'
    # 운동학 대역에는 깊이 카메라가 없다: 실제 safety_node 의 깊이 점군 입력을 끄고 기록한다
    # (켜 두면 점군 없음 → 전진 0.2 m/s 상한으로 09 가 0.3 m/s 주행을 못 했다)
    assert st.components['safety'] == stack_mod.REAL
    assert st.components['depth_cloud'].startswith('off')
    monkeypatch.setattr(req, 'has_executable', original)
    monkeypatch.setenv('ITEST_STANDINS', 'never')
    strict = stack_mod.Stack(Context(catalog.get(13)), KINEMATIC)
    with pytest.raises(unittest.SkipTest):
        strict.choose('z', [req.package('no_such_pkg_xyz')])
    with pytest.raises(unittest.SkipTest):
        strict.task_executor()
    with pytest.raises(unittest.SkipTest):
        strict.include('no_such_pkg_xyz', 'a.launch.py', {})


def test_gazebo_and_system_profiles(itest_env, monkeypatch, fake_launch):
    # 설치 여부를 고정한다 (예전: 설치된 워크스페이스에 따라 결과가 달라 통합 트리에서 실패했다)
    monkeypatch.setattr(req, 'has_executable', lambda p, e: p != 'amr_localization')
    ctx = Context(catalog.get(2))
    st = stack_mod.Stack(ctx, GAZEBO)
    st.simulator()
    st.description()                   # Gazebo 는 스폰 런치가 포함 — 추가 없음
    assert st.use_sim_time and st.components['simulator'] == stack_mod.REAL
    # imu/data_raw → 필터: amr_localization 이 없으면 대역, 있으면 실제 노드
    assert st.components['imu_filter'] == stack_mod.STANDIN
    assert isinstance(st.entities[0], IncludeLaunchDescription)
    monkeypatch.setattr(req, 'has_executable', lambda p, e: True)
    real = stack_mod.Stack(Context(catalog.get(2)), GAZEBO)
    real.simulator()
    assert real.components['imu_filter'] == stack_mod.REAL
    real.velocity_chain()              # Gazebo: 실제 safety_node + 실제 깊이 점군 필터
    assert real.components['pointcloud_filter'] == stack_mod.REAL
    assert 'depth_cloud' not in real.components
    monkeypatch.setattr(req, 'has_executable', lambda p, e: True)
    assert st.eval_logger('response_time_logger', {'x': 1}) is True
    assert st.components['evaluation'] == stack_mod.REAL
    sysctx = Context(catalog.get(7))
    sys_stack = stack_mod.Stack(sysctx, GAZEBO, 'system')
    sys_stack.system(use_behavior=True, use_perception=False,
                     extra_args={'localization_mode': 'localization'})
    assert sys_stack.components['navigation'] == stack_mod.REAL
    assert sys_stack.components['behavior'] == stack_mod.REAL
    assert sys_stack.components['perception'] == 'off'
    assert sys_stack.drive_topic == 'cmd_vel_nav'
    # system.launch.py 의 스위치는 with_<스택> (use_* 는 선언되지 않아 무시된다), 스폰은 월드 원점
    args = dict(sys_stack.entities[-1].launch_arguments)
    assert {k: args[k] for k in ('with_localization', 'with_navigation', 'with_perception',
                                 'with_behavior', 'with_fleet', 'x', 'y', 'yaw')} == {
        'with_localization': 'true', 'with_navigation': 'true', 'with_perception': 'false',
        'with_behavior': 'true', 'with_fleet': 'false', 'x': '0.0', 'y': '0.0', 'yaw': '0.0'}
    assert not any(k.startswith('use_') and k != 'use_sim_time' for k in args)
    reqs = stack_mod.system_requirements(use_navigation=False, use_perception=False)
    assert [r.package for r in reqs] == ['amr_bringup', 'amr_simulation', 'amr_localization']


# --- 대역 노드 로직 ---------------------------------------------------------------------------

def _capture(node, attr):
    out = []
    getattr(node, attr).publish = out.append
    return out


def _stamp(t):
    sec = int(t)
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)))


def test_kinematic_sim_and_relays(ros):
    from amr_itest.standins.cmd_relay import CmdRelay
    from amr_itest.standins.kinematic_sim import KinematicSim
    from amr_itest.standins.topic_relay import TopicRelay
    sim = KinematicSim()
    try:
        gts = _capture(sim, 'pub_gt')
        sim.on_obstacles(String(data=json.dumps({
            'world': {'circles': [[3.0, 0.0, 0.3]], 'boxes': [[-3, -1, -2, 1]]},
            'robot': {'circles': [[1.0, 0.0, 0.1]], 'boxes': [[0.5, -0.1, 0.7, 0.1]]}})))
        sim.on_obstacles(String(data='{bad json'))
        assert len(sim.world_circles) == 1 and len(sim.robot_boxes) == 1
        sim.on_cmd(_twist(0.5))
        for _ in range(30):
            time.sleep(0.01)
            sim.tick()
        assert sim.state.v > 0.1 and sim.state.x > 0.0 and gts
        sim.t_prev = sim._now() + 10.0
        sim.tick()                                  # dt ≤ 0 → 무시
        info = sim._camera_info(_stamp(1.0), 'rgb_camera')
        assert info.width == 640 and info.k[0] == pytest.approx(320 / math.tan(1.518436 / 2))
    finally:
        sim.destroy_node()
    relay = CmdRelay()
    try:
        out = _capture(relay, 'pub')
        relay.on_cmd(_twist(0.2))
        relay.republish()
        relay.last_time = -1e9
        relay.republish()
        assert [m.linear.x for m in out] == [0.2, 0.2, 0.0]
    finally:
        relay.destroy_node()
    TopicRelay().destroy_node()


def _twist(v, w=0.0):
    t = Twist()
    t.linear.x, t.angular.z = float(v), float(w)
    return t


def test_wheel_odometry_and_amcl(ros):
    from amr_itest.standins.amcl_pose import AmclPose
    from amr_itest.standins.wheel_odometry import WheelOdometry
    odo = WheelOdometry()
    try:
        out = _capture(odo, 'pub')
        odo.on_joints(JointState(name=['other'], position=[0.0]))
        for i in range(11):
            odo.on_joints(JointState(header=_hdr(i * 0.02),
                                     name=['left_wheel_joint', 'right_wheel_joint'],
                                     position=[i * 0.5, i * 0.5]))
        odo.on_joints(JointState(header=_hdr(0.1), name=['left_wheel_joint', 'right_wheel_joint'],
                                 position=[9.0, 9.0]))           # 역행 스탬프 → 무시
        assert len(out) == 10
        assert out[-1].pose.pose.position.x == pytest.approx(5.0 * 0.0825, rel=0.05)
        assert out[-1].twist.covariance[0] == pytest.approx(1e-4)
    finally:
        odo.destroy_node()
    amcl = AmclPose()
    try:
        out = _capture(amcl, 'pub')
        amcl.sample()                                   # GT 없음 → 무시
        gt = Odometry()
        gt.header.stamp = _stamp(3.0)
        gt.pose.pose.position.x = 1.0
        gt.pose.pose.orientation.w = 1.0
        amcl.on_gt(gt)
        amcl.sample()
        amcl.release()
        assert out == []                                # 지연 전
        time.sleep(amcl.latency + 0.02)
        amcl.release()
        assert len(out) == 1 and out[0].pose.pose.position.x == pytest.approx(1.0, abs=0.15)
        assert out[0].header.stamp.sec == 3
    finally:
        amcl.destroy_node()


def _hdr(t):
    from std_msgs.msg import Header
    return Header(stamp=_stamp(t))


def _scan(ranges):
    msg = LaserScan()
    msg.angle_min, msg.angle_increment = -math.pi, 2 * math.pi / len(ranges)
    msg.range_min, msg.range_max = 0.1, 25.0
    msg.ranges = [float(r) for r in ranges]
    return msg


def test_safety_gate(ros):
    from amr_itest.standins.safety_gate import CLEAR, CRITICAL, SafetyGate, STOP, WARNING
    g = SafetyGate()
    try:
        cmds = _capture(g, 'pub_cmd')
        g.on_scan(_scan([25.0] * 720))
        g.on_cmd(_twist(1.0, 0.2))
        assert cmds[-1].linear.x == 1.0 and g.current_zone() == CLEAR
        ranges = [25.0] * 720
        ranges[360] = 0.35      # 정면 빔: 라이다 x 0.15 + 0.35 − 풋프린트 전면 0.3 = 여유 0.2 m
        g.on_scan(_scan(ranges))
        assert g.proximity_stop and cmds[-1].linear.x == 0.0 and g.current_zone() == STOP
        g.on_scan(_scan([25.0] * 720))
        assert not g.proximity_stop
        for clearance, zone, cap in ((0.4, CRITICAL, 0.2), (0.8, WARNING, 0.5)):
            g.clearance = clearance
            assert g.current_zone() == zone
            g.on_cmd(_twist(1.0))
            assert cmds[-1].linear.x == pytest.approx(cap)
        g.clearance = math.inf
        g.on_button('robot', Bool(data=True))
        assert cmds[-1].linear.x == 0.0 and g.effective_stop()
        res = g.on_reset(None, NS(success=None, message=''))
        assert res.success is False and 'robot' in res.message
        g.on_button('robot', Bool(data=False))
        assert g.effective_stop()                      # 래치 유지
        res = g.on_reset(None, NS(success=None, message=''))
        assert res.success and not g.effective_stop()
        g.scan_time = g.now() - 10.0
        assert g.scan_stale() and g.effective_stop()
        g.cmd_time = -math.inf
        g.scan_time = g.now()
        g.publish()
        assert cmds[-1].linear.x == 0.0                # 명령 타임아웃
    finally:
        g.destroy_node()


def test_task_executor_imu_static(ros):
    from amr_itest.standins.imu_filter import ImuFilter
    from amr_itest.standins.static_tf import StaticTf
    from amr_itest.standins.task_executor import TaskExecutor
    from amr_msgs.msg import Task
    from amr_msgs.srv import AssignTask
    ex = TaskExecutor()
    try:
        statuses = _capture(ex, 'pub_status')
        drives = _capture(ex, 'pub_cmd')
        ex.tick()                                       # 작업 없음
        req_msg = AssignTask.Request()
        req_msg.task.task_id = 't1'
        res = ex.on_assign(req_msg, AssignTask.Response())
        assert res.success and statuses[-1].status == Task.STATUS_IN_PROGRESS
        assert not ex.on_assign(req_msg, AssignTask.Response()).success
        ex.tick()
        assert drives[-1].linear.x == pytest.approx(ex.speed)
        ex.t_end = 0.0
        ex.tick()
        assert statuses[-1].status == Task.STATUS_COMPLETED and ex.task is None
        assert drives[-1].linear.x == 0.0
    finally:
        ex.destroy_node()
    imu = ImuFilter()
    try:
        out = _capture(imu, 'pub')
        for i in range(300):
            m = Imu()
            m.header = _hdr(i * 0.01)
            m.angular_velocity.z = 0.01
            imu.on_imu(m)
        assert imu.bias[2] == pytest.approx(0.01)
        assert out and out[-1].angular_velocity.z == pytest.approx(0.0)
    finally:
        imu.destroy_node()
    StaticTf().destroy_node()


def test_run_entry_handles_interrupt():
    from amr_itest.standins import common

    def factory():
        raise KeyboardInterrupt
    common.run(factory, args=[])
