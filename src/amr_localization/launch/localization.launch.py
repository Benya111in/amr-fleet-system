r"""
위치 추정 스택 (명세 4.2/4.3, docs/architecture/components.md §3.2).

    ros2 launch amr_localization localization.launch.py mode:=slam                 # 매핑
    ros2 launch amr_localization localization.launch.py mode:=localization \
        map:=/ros2_ws/maps/warehouse.yaml initial_x:=0.0 initial_y:=0.0   # 주행
    ros2 launch amr_localization localization.launch.py robot_name:=amr_02 start_map_server:=false

인자
    mode              slam | localization | odom (기본 localization)
                        odom         : 전처리 3 노드 + ekf_filter_node_odom 만 (드리프트 실험·단위 확인)
                        slam         : + slam_toolbox (map→odom 발행, /map 발행)
                        localization : + map_server(루트, 선택) + amcl + ekf_filter_node_map
                                       + kidnap_monitor_node
    robot_name        네임스페이스 (기본 amr_01, '' 이면 네임스페이스 없음)
    frame_prefix      프레임 접두어 (기본 'auto' = robot_name + '/', robot_name 이 '' 면 '')
    use_sim_time      (기본 true)
    config_dir        최상위 config (robot_params.yaml, sensors.yaml, ekf.yaml). 기본 $ROS_WS/config
    map               map_server YAML (기본 $ROS_WS/maps/warehouse.yaml)
    start_map_server  루트 map_server + lifecycle_manager_map 기동 (기본 true; 다중 로봇은 한 번만)
    initial_x/y/yaw   AMCL 초기 자세 = 스폰 자세 [m, rad]
    use_kidnap_monitor (기본 true), kidnap_fallback_cmd_vel ('' 기본, 단독 시험만 cmd_vel)
    amcl_half_cell_fix  (기본 true) amcl_map_adapter 가 /map 을 원점 반 셀 보정해 map_amcl 로 재발행하고
                        AMCL 은 그것을 구독한다 (nav2_amcl 반올림 규약 편향 보정, docs/algorithms/slam.md §4)
    bias_estimation_time  IMU 기동 바이어스 추정 시간 [s] ('' = imu_filter.yaml 값)
    noise_seed        인코더 잡음 seed (0 = 무작위)
    log_level         (기본 info)

단일 출처 (docs/architecture/components.md §6): 바퀴 r·b 는 robot_params.yaml, 인코더 틱·슬립·주기와
IMU 잡음, LiDAR 거리·extrinsic 은 sensors.yaml 에서 읽어 노드 파라미터로 넘긴다 (패키지 YAML 뒤에 붙여 덮어씀).
EKF 는 config/ekf.yaml 한 파일에 프레임 접두어만 뒤에서 덮어쓴다 (multi_robot.md §2).
두 EKF 의 서비스(set_pose, toggle, enable)는 같은 네임스페이스에서 이름이 겹치므로 노드 이름 아래로 리맵한다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

MODES = ('slam', 'localization', 'odom')


def _ws_path(*parts: str) -> str:
    ros_ws = os.environ.get('ROS_WS', '/ros2_ws')
    return os.path.join(ros_ws, *parts)


def _read_params(path: str) -> dict:
    """최상위 config YAML 의 /** ros__parameters 블록 (없으면 {})."""
    if not os.path.isfile(path):
        return {}
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    return data.get('/**', {}).get('ros__parameters', {})


def shared_parameters(config_dir: str) -> dict:
    """robot_params.yaml / sensors.yaml → 노드별 덮어쓸 파라미터 (키가 없으면 패키지 YAML 기본값 유지)."""
    robot = _read_params(os.path.join(config_dir, 'robot_params.yaml')).get('robot', {})
    sensors = _read_params(os.path.join(config_dir, 'sensors.yaml'))
    enc = sensors.get('wheel_encoder', {})
    imu = sensors.get('imu', {})
    lidar = sensors.get('lidar', {})
    out = {'wheel': {}, 'imu': {}, 'scan': {}, 'lidar_offset': None}
    for src, dst in (('wheel_radius', 'wheel_radius'), ('wheel_separation', 'wheel_separation')):
        if src in robot:
            out['wheel'][dst] = float(robot[src])
    if 'ticks_per_revolution' in enc:
        out['wheel']['ticks_per_revolution'] = int(enc['ticks_per_revolution'])
    if 'slip_noise_stddev' in enc:
        out['wheel']['slip_noise_stddev'] = float(enc['slip_noise_stddev'])
    if 'update_rate' in enc:
        out['wheel']['publish_rate'] = float(enc['update_rate'])
    for key in ('gyro_noise_stddev', 'accel_noise_stddev'):
        if key in imu:
            out['imu'][key] = float(imu[key])
    for key in ('range_min', 'range_max'):
        if key in lidar:
            out['scan'][key] = float(lidar[key])
    ext = lidar.get('extrinsic')
    if isinstance(ext, dict):
        out['lidar_offset'] = [float(ext.get('x', 0.0)), float(ext.get('y', 0.0)),
                               float(ext.get('yaw', 0.0))]
    return out


def _flag(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def _setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODES:
        raise RuntimeError(f'mode:={mode} (허용: {", ".join(MODES)})')
    robot_name = LaunchConfiguration('robot_name').perform(context).strip('/')
    prefix = LaunchConfiguration('frame_prefix').perform(context)
    if prefix == 'auto':
        prefix = f'{robot_name}/' if robot_name else ''
    use_sim_time = _flag(context, 'use_sim_time')
    config_dir = LaunchConfiguration('config_dir').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    pkg_config = os.path.join(get_package_share_directory('amr_localization'), 'config')
    shared = shared_parameters(config_dir)
    ekf_yaml = os.path.join(config_dir, 'ekf.yaml')
    if not os.path.isfile(ekf_yaml):
        raise RuntimeError(f'ekf.yaml 이 없다: {ekf_yaml} (config_dir:= 로 지정)')

    odom_frame = f'{prefix}odom'
    base_frame = f'{prefix}base_footprint'
    common = {'use_sim_time': use_sim_time}
    ros_args = ['--ros-args', '--log-level', log_level]
    ns = robot_name or ''

    def node(package, executable, name, params, remappings=None, namespace=ns):
        return Node(package=package, executable=executable, name=name, namespace=namespace,
                    output='screen', parameters=params, remappings=remappings or [],
                    arguments=ros_args)

    wheel = dict(shared['wheel'], frame_prefix=prefix,
                 noise_seed=int(LaunchConfiguration('noise_seed').perform(context)), **common)
    imu = dict(shared['imu'], **common)
    bias_time = LaunchConfiguration('bias_estimation_time').perform(context)
    if bias_time:
        imu['bias_estimation_time'] = float(bias_time)
    actions = [
        LogInfo(msg=f'[localization] mode={mode} ns=/{ns} prefix="{prefix}" config={config_dir}'),
        node('amr_localization', 'wheel_odometry_node', 'wheel_odometry_node',
             [os.path.join(pkg_config, 'wheel_odometry.yaml'), wheel]),
        node('amr_localization', 'imu_filter_node', 'imu_filter_node',
             [os.path.join(pkg_config, 'imu_filter.yaml'), imu]),
        node('amr_localization', 'scan_filter_node', 'scan_filter_node',
             [os.path.join(pkg_config, 'scan_filter.yaml'), dict(shared['scan'], **common)]),
    ]

    def ekf(name, world_frame, extra_remaps):
        remaps = [(srv, f'{name}/{srv}') for srv in ('set_pose', 'toggle', 'enable')]
        return node('robot_localization', 'ekf_node', name,
                    [ekf_yaml, {'map_frame': 'map', 'odom_frame': odom_frame,
                                'base_link_frame': base_frame, 'world_frame': world_frame,
                                **common}], remaps + extra_remaps)

    actions.append(ekf('ekf_filter_node_odom', odom_frame, []))

    if mode == 'slam':
        actions.append(node('slam_toolbox', 'async_slam_toolbox_node', 'slam_toolbox',
                            [os.path.join(pkg_config, 'slam_toolbox.yaml'),
                             {'odom_frame': odom_frame, 'base_frame': base_frame,
                              'map_frame': 'map', **common}]))
    elif mode == 'localization':
        actions.append(ekf('ekf_filter_node_map', 'map',
                           [('odometry/filtered', 'odometry/filtered_map')]))
        if _flag(context, 'start_map_server'):
            map_yaml = LaunchConfiguration('map').perform(context)
            actions += [
                node('nav2_map_server', 'map_server', 'map_server',
                     [{'yaml_filename': map_yaml, 'topic_name': 'map', 'frame_id': 'map',
                       **common}], namespace=''),
                node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_map',
                     [{'autostart': True, 'node_names': ['map_server'], **common}],
                     namespace=''),
            ]
        initial = {f'initial_pose.{k}': float(LaunchConfiguration(f'initial_{k}').perform(context))
                   for k in ('x', 'y', 'yaw')}
        amcl_map = {}
        if _flag(context, 'amcl_half_cell_fix'):
            actions.append(node('amr_localization', 'amcl_map_adapter', 'amcl_map_adapter',
                                [{'input_topic': '/map', 'output_topic': 'map_amcl', **common}]))
            amcl_map = {'map_topic': 'map_amcl'}
        actions += [
            node('nav2_amcl', 'amcl', 'amcl',
                 [os.path.join(pkg_config, 'amcl.yaml'),
                  {'global_frame_id': 'map', 'odom_frame_id': odom_frame,
                   'base_frame_id': base_frame, 'initial_pose.z': 0.0, **initial, **amcl_map,
                   **common}]),
            node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_localization',
                 [{'autostart': True, 'node_names': ['amcl'], 'bond_timeout': 10.0, **common}]),
        ]
        if _flag(context, 'use_kidnap_monitor'):
            kidnap = {'ekf_set_pose_service': 'ekf_filter_node_map/set_pose',
                      'fallback_cmd_vel_topic':
                          LaunchConfiguration('kidnap_fallback_cmd_vel').perform(context),
                      **common}
            if shared['lidar_offset'] is not None:
                kidnap['lidar_offset'] = shared['lidar_offset']
            actions.append(node('amr_localization', 'kidnap_monitor_node', 'kidnap_monitor_node',
                                [os.path.join(pkg_config, 'kidnap_monitor.yaml'), kidnap]))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='localization',
                              description='slam | localization | odom'),
        DeclareLaunchArgument('robot_name', default_value='amr_01',
                              description="네임스페이스 ('' = 없음)"),
        DeclareLaunchArgument('frame_prefix', default_value='auto',
                              description="프레임 접두어 ('auto' = robot_name + '/')"),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config_dir', default_value=_ws_path('config')),
        DeclareLaunchArgument('map', default_value=_ws_path('maps', 'warehouse.yaml')),
        DeclareLaunchArgument('start_map_server', default_value='true'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_kidnap_monitor', default_value='true'),
        DeclareLaunchArgument('kidnap_fallback_cmd_vel', default_value=''),
        DeclareLaunchArgument('amcl_half_cell_fix', default_value='true',
                              description='AMCL 에 원점 반 셀 보정 맵(map_amcl)을 준다'),
        DeclareLaunchArgument('bias_estimation_time', default_value=''),
        DeclareLaunchArgument('noise_seed', default_value='0'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_setup),
    ])
