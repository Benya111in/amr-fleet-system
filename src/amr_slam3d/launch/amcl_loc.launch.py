"""
2D 위치추정 행 (보너스 2 E3(b), b2-eval) — 배포 위치추정(amr_localization) 설정 그대로, 지도만 바꾼다.

    ros2 launch amr_slam3d amcl_loc.launch.py map:=/data/map/map.yaml initial_x:=18 \
        initial_y:=-16 initial_yaw:=1.5708 [chain:=deploy|amcl]

chain:=deploy (기본, E3(b) 의 2D 행): 배포 `localization.launch.py mode:=localization` 의 위치추정 부분을
같은 파라미터로 띄운다 — 루트 map_server + lifecycle 관리자, amcl_map_adapter (반 셀 보정 map_amcl),
nav2_amcl (amcl.yaml, `tf_broadcast: false` 그대로), ekf_filter_node_map (config/ekf.yaml —
휠 odom·IMU·amcl_pose·scan_match_pose 융합, **map → odom 발행자**), scan_matcher_node, kidnap_monitor_node
(LOST·복구 — 회전 명령 없음). 파라미터 조립은 배포 런치의 `shared_parameters` 를 그대로 불러 쓴다.
배포와 다른 것: 전처리 노드(휠·IMU 필터·스캔 필터·odom EKF)는 띄우지 않는다 — 그 출력(`scan_filtered`, `wheel_odom`,
`imu/data`, `odometry/filtered`, EKF `odom → base_footprint` TF)은 bag 에서 튼다 (같은 녹화 = 같은 입력).
chain:=amcl: AMCL 만 (`tf_broadcast: true`) — 보조 진단용.

인자
    map          map_server YAML (필수)
    robot        네임스페이스·프레임 접두어 (기본 amr_01)
    initial_x/y/yaw  AMCL 초기 자세 [m, rad] (스폰 자세 — 배포와 같은 정보)
    chain        deploy | amcl (기본 deploy)
    config_dir   최상위 config (ekf.yaml·robot_params.yaml·sensors.yaml, 기본 /ros2_ws/config)
    log_level    (기본 info)
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: 배포 localization.launch.py 와 같은 bond 상한 [s]
BOND_TIMEOUT_S = 30.0
CHAINS = ('deploy', 'amcl')


def amcl_overrides(robot: str, x: float, y: float, yaw: float, chain: str) -> dict:
    """amcl.yaml 뒤에 덧붙일 값 — 배포 런치가 주는 것 그대로. chain:=amcl 만 tf_broadcast true."""
    if chain not in CHAINS:
        raise RuntimeError(f'chain:={chain} ({" | ".join(CHAINS)})')
    prefix = f'{robot}/' if robot else ''
    out = {'global_frame_id': 'map', 'odom_frame_id': f'{prefix}odom',
           'base_frame_id': f'{prefix}base_footprint', 'initial_pose.z': 0.0,
           'initial_pose.x': float(x), 'initial_pose.y': float(y),
           'initial_pose.yaw': float(yaw), 'map_topic': 'map_amcl', 'use_sim_time': True}
    if chain == 'amcl':
        out['tf_broadcast'] = True
    return out


def _deploy_module():
    """배포 런치(amr_localization/launch/localization.launch.py)를 모듈로 — shared_parameters 를 같이 쓴다."""
    path = os.path.join(get_package_share_directory('amr_localization'), 'launch',
                        'localization.launch.py')
    spec = importlib.util.spec_from_file_location('amr_localization_launch', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    robot = arg('robot').strip('/')
    prefix = f'{robot}/' if robot else ''
    map_yaml = arg('map')
    if not map_yaml or not os.path.isfile(map_yaml):
        raise RuntimeError(f'map:={map_yaml!r} 가 없다')
    chain = arg('chain').strip().lower()
    ros_args = ['--ros-args', '--log-level', arg('log_level')]
    common = {'use_sim_time': True}
    cfg = os.path.join(get_package_share_directory('amr_localization'), 'config')
    over = amcl_overrides(robot, float(arg('initial_x')), float(arg('initial_y')),
                          float(arg('initial_yaw')), chain)

    def node(package, executable, name, params, remappings=None, namespace=robot):
        return Node(package=package, executable=executable, name=name, namespace=namespace,
                    output='screen', parameters=params, remappings=remappings or [],
                    arguments=ros_args)

    actions = [
        LogInfo(msg=f'[amr_slam3d] amcl_loc chain={chain} map={map_yaml} robot={robot} '
                    f'init=({over["initial_pose.x"]}, {over["initial_pose.y"]}, '
                    f'{over["initial_pose.yaw"]})'),
        node('nav2_map_server', 'map_server', 'map_server',
             [{'yaml_filename': map_yaml, 'topic_name': 'map', 'frame_id': 'map', **common}],
             namespace=''),
        node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_map',
             [{'autostart': True, 'node_names': ['map_server'], 'bond_timeout': BOND_TIMEOUT_S,
               **common}], namespace=''),
        node('amr_localization', 'amcl_map_adapter', 'amcl_map_adapter',
             [{'input_topic': '/map', 'output_topic': 'map_amcl', **common}]),
        node('nav2_amcl', 'amcl', 'amcl', [os.path.join(cfg, 'amcl.yaml'), over]),
        node('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_localization',
             [{'autostart': True, 'node_names': ['amcl'], 'bond_timeout': BOND_TIMEOUT_S,
               **common}]),
    ]
    if chain == 'deploy':
        mod = _deploy_module()
        config_dir = arg('config_dir')
        shared = mod.shared_parameters(config_dir)
        ekf_yaml = os.path.join(config_dir, 'ekf.yaml')
        if not os.path.isfile(ekf_yaml):
            raise RuntimeError(f'ekf.yaml 이 없다: {ekf_yaml}')
        remaps = [(srv, f'ekf_filter_node_map/{srv}') for srv in ('set_pose', 'toggle', 'enable')]
        actions.append(node('robot_localization', 'ekf_node', 'ekf_filter_node_map',
                            [ekf_yaml, {'map_frame': 'map', 'odom_frame': f'{prefix}odom',
                                        'base_link_frame': f'{prefix}base_footprint',
                                        'world_frame': 'map', **common}],
                            remaps + [('odometry/filtered', 'odometry/filtered_map')]))
        kidnap = {'ekf_set_pose_service': 'ekf_filter_node_map/set_pose',
                  'fallback_cmd_vel_topic': '', **common}
        matcher = {'base_frame': f'{prefix}base_footprint', **common}
        if shared['lidar_offset'] is not None:
            kidnap['lidar_offset'] = shared['lidar_offset']
            matcher['lidar_offset'] = shared['lidar_offset']
        if shared['range_noise'] is not None:
            matcher['range_noise'] = shared['range_noise']
        actions += [
            node('amr_localization', 'kidnap_monitor_node', 'kidnap_monitor_node',
                 [os.path.join(cfg, 'kidnap_monitor.yaml'), kidnap]),
            node('amr_localization', 'scan_matcher_node', 'scan_matcher_node',
                 [os.path.join(cfg, 'scan_matcher.yaml'), matcher]),
        ]
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('robot', default_value='amr_01'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('chain', default_value='deploy', description='deploy | amcl'),
        DeclareLaunchArgument('config_dir',
                              default_value=os.path.join(os.environ.get('ROS_WS', '/ros2_ws'),
                                                         'config')),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_setup),
    ])
