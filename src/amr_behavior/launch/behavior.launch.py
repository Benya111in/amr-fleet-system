"""
작업 수행 계층 기동: task_executor_node + docking_server_node + battery_model_node (components.md §3.5).

인자
    namespace / robot_name   로봇 네임스페이스 (예: amr_01). 비우면 전역 (단일 로봇 개발용)
    use_sim_time             Gazebo /clock 사용 (기본 true)
    params_file              behavior.yaml (기본 share/amr_behavior/config/behavior.yaml)
    robot_params_file        물품 표가 있는 robot_params.yaml
                             (기본 $ROS_WS/config/robot_params.yaml, 없으면 생략)
    frame_prefix             TF 프레임 접두어 (기본: 네임스페이스가 있으면 "<ns>/", multi_robot.md §2)
    groot_publisher_port     Groot ZMQ PUB 포트 (기본 1666). amr_bringup 은 로봇 i(0부터)에 1666 + 2i 를 준다
    groot_server_port        Groot ZMQ 서버 포트 (기본 1667, bringup: 1667 + 2i)
    waiting_pose             대기 구역 "x,y,yaw" (비우면 params_file 값)
    start_executor / start_docking   각 노드 기동 여부 (기본 true)
    start_battery_model      시뮬레이션 배터리 battery_model_node (기본 true, 실기·다른 battery_state 발행자가
                             있으면 false)
    battery_initial_percent  시작 잔량 [%] ('' = params_file 값, 충전 경로 시험용)
    log_level                기본 info
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text: str) -> bool:
    return text.strip().lower() in ('true', '1', 'yes')


def _parse_pose(text: str):
    parts = [p for p in text.replace(' ', '').split(',') if p]
    if not parts:
        return None
    if len(parts) != 3:
        raise ValueError(f'waiting_pose 는 "x,y,yaw" 형식이어야 한다: {text!r}')
    return [float(p) for p in parts]


def _default_robot_params() -> str:
    ws = os.environ.get('ROS_WS', '')
    path = os.path.join(ws, 'config', 'robot_params.yaml') if ws else ''
    return path if path and os.path.isfile(path) else ''


def _overrides(arg, frame_prefix: str):
    """런치 인자 → 노드별 파라미터 덮어쓰기 (실행기, 도킹 서버, 배터리 모델)."""
    use_sim_time = _as_bool(arg('use_sim_time'))
    executor = {
        'use_sim_time': use_sim_time,
        'groot.publisher_port': int(arg('groot_publisher_port')),
        'groot.server_port': int(arg('groot_server_port')),
    }
    waiting = _parse_pose(arg('waiting_pose'))
    if waiting is not None:
        executor['waiting_pose'] = waiting
    docking = {'use_sim_time': use_sim_time, 'base_frame': f'{frame_prefix}base_link'}
    battery = {'use_sim_time': use_sim_time}
    if arg('battery_initial_percent').strip():
        battery['initial_percent'] = float(arg('battery_initial_percent'))
    return executor, docking, battery


def _launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    namespace = arg('namespace') or arg('robot_name')
    namespace = namespace.strip('/')
    frame_prefix = arg('frame_prefix')
    if frame_prefix == 'auto':
        frame_prefix = f'{namespace}/' if namespace else ''

    params_files = [arg('params_file')]
    robot_params = arg('robot_params_file') or _default_robot_params()
    if robot_params:
        params_files.insert(0, robot_params)   # 물품 표 → behavior.yaml 순 (뒤가 덮어쓴다)
    executor_overrides, docking_overrides, battery_overrides = _overrides(arg, frame_prefix)
    log_args = ['--ros-args', '--log-level', arg('log_level')]

    nodes = []
    if _as_bool(arg('start_docking')):
        nodes.append(Node(
            package='amr_behavior', executable='docking_server_node',
            name='docking_server_node', namespace=namespace, output='screen',
            parameters=params_files + [docking_overrides], arguments=log_args))
    if _as_bool(arg('start_executor')):
        nodes.append(Node(
            package='amr_behavior', executable='task_executor_node',
            name='task_executor_node', namespace=namespace, output='screen',
            parameters=params_files + [executor_overrides], arguments=log_args))
    if _as_bool(arg('start_battery_model')):
        nodes.append(Node(
            package='amr_behavior', executable='battery_model_node',
            name='battery_model_node', namespace=namespace, output='screen',
            parameters=params_files + [battery_overrides], arguments=log_args))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('amr_behavior')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('robot_name', default_value='',
                              description='namespace 의 별칭 (docker-compose 규약)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'behavior.yaml')),
        DeclareLaunchArgument('robot_params_file', default_value=''),
        DeclareLaunchArgument('frame_prefix', default_value='auto'),
        DeclareLaunchArgument('groot_publisher_port', default_value='1666'),
        DeclareLaunchArgument('groot_server_port', default_value='1667'),
        DeclareLaunchArgument('waiting_pose', default_value=''),
        DeclareLaunchArgument('start_executor', default_value='true'),
        DeclareLaunchArgument('start_docking', default_value='true'),
        DeclareLaunchArgument('start_battery_model', default_value='true'),
        DeclareLaunchArgument('battery_initial_percent', default_value=''),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_launch_setup),
    ])
