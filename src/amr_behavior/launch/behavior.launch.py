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
    traffic_hold_policy      auto | required ('' = params_file 값 auto). traffic/hold 첫 값을 받기 전 규칙:
                             교통 관리자(traffic_manager_node)를 함께 띄운다고 아는 런치는 required 를 준다
                             (docs/research/fleet-traffic-deadlock/hold_failsafe.md)
    fleet_mode               central(기본) | distributed — 보너스 4 분산 플릿 (fleet-distributed/p3.md §10).
                             distributed 면 (1) 실행기 fleet.distributed:=true (존 노드 · 분산 트리), (2) 실행기
                             assign_task 를 executor/assign_task 로 리매핑, (3) 같은 네임스페이스에
                             fleet_agent (amr_fleet fleet_agent_node — 펜스 프록시 · 경로 게이트 · 협상).
                             셋을 한곳에서 정해 로봇마다 어긋나지 않게 한다 (프록시 없이 리매핑만 되거나 그 반대면
                             assign_task 서버가 없거나 둘이 된다). 상위 런치(multi_robot fleet_mode)의 값을
                             launch 인자 상속으로 받는다 — 이 인자가 없는 맥락(시험의 직접 호출)은 central.
    fleet_members            분산 모드 정적 멤버 ("amr_01,…", '' = fleet_agent.yaml 5 대). multi_robot 이 준다
    fleet_store_dir          분산 모드 에이전트 저장소 ('' = $ROS_WS/var/fleet_agent). 실행마다 새 경로를 준다
    log_level                기본 info
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
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
    policy = arg('traffic_hold_policy').strip()
    if policy:
        if policy not in ('auto', 'required'):
            raise ValueError(f'traffic_hold_policy 는 auto | required: {policy!r}')
        executor['traffic_hold.policy'] = policy
    if fleet_mode(arg) == 'distributed':
        executor['fleet.distributed'] = True
    docking = {'use_sim_time': use_sim_time, 'base_frame': f'{frame_prefix}base_link'}
    battery = {'use_sim_time': use_sim_time}
    if arg('battery_initial_percent').strip():
        battery['initial_percent'] = float(arg('battery_initial_percent'))
    return executor, docking, battery


FLEET_MODES = ('central', 'distributed')


def fleet_mode(arg) -> str:
    """fleet_mode 인자 (없으면 central). arg 는 이름 → 문자열 (없는 이름은 KeyError 또는 '')."""
    try:
        mode = (arg('fleet_mode') or '').strip().lower() or 'central'
    except KeyError:
        mode = 'central'
    if mode not in FLEET_MODES:
        raise ValueError(f'fleet_mode 는 {" | ".join(FLEET_MODES)}: {mode!r}')
    return mode


def _fleet_agent(arg, namespace: str, use_sim_time: bool, log_args):
    """분산 모드 로봇 에이전트 (amr_fleet fleet_agent_node) — 펜스 프록시 · 경로 게이트 · 협상."""
    if not namespace:
        raise ValueError('fleet_mode:=distributed 에는 로봇 네임스페이스가 필요하다 (robot_id = 네임스페이스)')
    try:
        share = get_package_share_directory('amr_fleet')
    except PackageNotFoundError as exc:
        raise RuntimeError('fleet_mode:=distributed 에는 amr_fleet 이 필요하다') from exc
    params = {'use_sim_time': use_sim_time, 'proxy_assign_task': True,
              'executor_service': 'executor/assign_task', 'path_gate.enabled': True,
              'release_gate': 'zone_map',
              'zones_file': os.path.join(share, 'config', 'traffic_zones.yaml'),
              'traffic_params_file': os.path.join(share, 'config', 'traffic.yaml')}
    for key, name in (('members', 'fleet_members'), ('store_dir', 'fleet_store_dir')):
        try:
            value = arg(name).strip()
        except KeyError:
            value = ''
        if value:
            params[key] = value
    return Node(package='amr_fleet', executable='fleet_agent_node', name='fleet_agent',
                namespace=namespace, output='screen',
                parameters=[os.path.join(share, 'config', 'fleet_agent.yaml'), params],
                arguments=log_args)


def _launch_setup(context, *args, **kwargs):
    def arg(name):
        if name.startswith('fleet_') and name not in context.launch_configurations:
            raise KeyError(name)        # 상위 런치가 주지 않은 분산 인자 (기본 central)
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
        distributed = fleet_mode(arg) == 'distributed'
        nodes.append(Node(
            package='amr_behavior', executable='task_executor_node',
            name='task_executor_node', namespace=namespace, output='screen',
            parameters=params_files + [executor_overrides], arguments=log_args,
            # 분산 모드: 에이전트 펜스 프록시가 assign_task 를 받고 통과한 것만 실행기로 넘긴다
            remappings=[('assign_task', 'executor/assign_task')] if distributed else []))
        if distributed:
            nodes.append(_fleet_agent(arg, namespace, _as_bool(arg('use_sim_time')), log_args))
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
        DeclareLaunchArgument('traffic_hold_policy', default_value='',
                              description="auto | required ('' = params_file)"),
        DeclareLaunchArgument('fleet_mode', default_value='central',
                              description='central | distributed (보너스 4 분산 플릿)'),
        DeclareLaunchArgument('fleet_members', default_value='',
                              description="분산 모드 정적 멤버 ('' = fleet_agent.yaml)"),
        DeclareLaunchArgument('fleet_store_dir', default_value='',
                              description="분산 모드 에이전트 저장소 ('' = $ROS_WS/var/fleet_agent)"),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_launch_setup),
    ])
