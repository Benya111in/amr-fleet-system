"""
도킹 측정용 단일 로봇 시스템.

amr_bringup system.launch.py 와 같은 조립에서 인지 스택의 YOLO·객체 위치·표시 노드만 끈다
(도킹 체인과 무관, 부하만 줄인다).

켜지는 것: Gazebo 창고 월드(GPU 렌더링 카메라), description/spawn, localization(EKF + AMCL,
maps/warehouse), navigation(Nav2 + velocity_profiler), perception(aruco_detector_node, pointcloud,
tracker, safety_node), behavior(docking_server_node, task_executor_node, battery_model_node).
fleet/dashboard/evaluation 끔.

    ros2 launch src/amr_behavior/test/scripts/dock_system.launch.py x:=-25.5 y:=15.0 yaw:=3.14159 \
        with_fleet:=false with_dashboard:=false with_evaluation:=false headless:=true
"""

from dataclasses import replace

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# amr_bringup 는 함수 안에서 들여온다: 이 디렉터리는 pytest 가 훑고 launch_testing 플러그인이
# *.launch.py 를 수집하므로, amr_bringup 이 없는 단독 패키지 체크아웃에서 모듈 수준 import 가
# 실패하면 amr_behavior 의 python 시험이 통째로 수집되지 않는다 (실측).


def _patch_perception(lu):
    """인지 스택에서 YOLO·객체 위치·표시 노드만 끈다 (도킹 체인과 무관, 부하만 줄인다)."""
    original = lu.stack_extra_arguments

    def _extra(label, opts):
        out = dict(original(label, opts))
        if label == 'perception':
            out.update({'use_yolo': 'false', 'use_localizer': 'false', 'use_markers': 'false'})
        return out

    lu.stack_extra_arguments = _extra


def _setup(context):
    from amr_bringup import fleet_spawn
    from amr_bringup import launch_utils as lu
    _patch_perception(lu)
    opts = lu.read_options(context, 'dock')
    spawn = fleet_spawn.load_fleet_spawn(LaunchConfiguration('robots_file').perform(context))
    x, y, yaw = (float(LaunchConfiguration(k).perform(context)) for k in ('x', 'y', 'yaw'))
    robot = fleet_spawn.parse_robot({'name': 'amr_01', 'x': x, 'y': y, 'yaw': yaw}, 0)
    listed = fleet_spawn.find_robot(spawn, 'amr_01')
    if listed is not None:
        robot = replace(robot, comm_latency_ms=listed.comm_latency_ms)
    return lu.bringup_actions([robot], spawn, opts, prefixed=False)


def generate_launch_description():
    from amr_bringup import launch_utils as lu
    args = [DeclareLaunchArgument('x', default_value='-25.5'),
            DeclareLaunchArgument('y', default_value='15.0'),
            DeclareLaunchArgument('yaw', default_value='3.14159')]
    return LaunchDescription(args + lu.common_arguments() + [OpaqueFunction(function=_setup)])
