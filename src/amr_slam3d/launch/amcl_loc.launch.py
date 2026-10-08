"""
AMCL 위치추정 행 (보너스 2 E3(b), b2-eval) — 배포 AMCL 설정(amr_localization/config/amcl.yaml) 그대로, 지도만 바꾼다.

    ros2 launch amr_slam3d amcl_loc.launch.py map:=/data/map/map.yaml initial_x:=18 \
        initial_y:=-16 \
        initial_yaw:=1.5708

배포(localization.launch.py mode:=localization)에서 가져온 것: map_server(루트, /map) + lifecycle 관리자,
amcl_map_adapter
(반 셀 보정 map_amcl — 배포 기본 true), nav2_amcl(같은 amcl.yaml, 프레임 접두어, 초기 자세 = 스폰 자세, bond 30 s).
배포와 **다른 것** (재생 비교용 — 결과 문서에 적는다):
- `tf_broadcast: true` — 배포는 map→odom 을 ekf_filter_node_map(AMCL + scan_matcher 융합)이 내고
  AMCL 은 TF 를 내지 않는다.
  이 행은 그 융합·scan_matcher·kidnap_monitor 를 띄우지 않고 **AMCL 자세 자체**를 잰다 (보고서 "AMCL 자리만 바꾸는 구성").
- 전처리 노드(휠·IMU·스캔 필터·odom EKF)는 띄우지 않는다 — 그 출력(`scan_filtered`,
  EKF `odom → base_footprint` TF)은 bag 에서 튼다.

인자
    map          map_server YAML (필수)
    robot        네임스페이스·프레임 접두어 (기본 amr_01)
    initial_x/y/yaw  AMCL 초기 자세 [m, rad] (스폰 자세)
    half_cell_fix    (기본 true) amcl_map_adapter 로 원점 반 셀 보정 지도를 준다 (배포 기본)
    log_level    (기본 info)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: 배포 localization.launch.py 와 같은 bond 상한 [s]
BOND_TIMEOUT_S = 30.0


def amcl_overrides(robot: str, x: float, y: float, yaw: float, half_cell_fix: bool) -> dict:
    """amcl.yaml 뒤에 덧붙일 값 — 배포 런치가 주는 것 + tf_broadcast true (이 행의 유일한 의도적 차이)."""
    prefix = f'{robot}/' if robot else ''
    out = {'global_frame_id': 'map', 'odom_frame_id': f'{prefix}odom',
           'base_frame_id': f'{prefix}base_footprint', 'initial_pose.x': float(x),
           'initial_pose.y': float(y), 'initial_pose.z': 0.0, 'initial_pose.yaw': float(yaw),
           'set_initial_pose': True, 'tf_broadcast': True, 'use_sim_time': True}
    if half_cell_fix:
        out['map_topic'] = 'map_amcl'
    return out


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    robot = arg('robot').strip('/')
    map_yaml = arg('map')
    if not map_yaml or not os.path.isfile(map_yaml):
        raise RuntimeError(f'map:={map_yaml!r} 가 없다')
    fix = arg('half_cell_fix').lower() in ('true', '1', 'yes')
    ros_args = ['--ros-args', '--log-level', arg('log_level')]
    common = {'use_sim_time': True}
    amcl_yaml = os.path.join(get_package_share_directory('amr_localization'), 'config',
                             'amcl.yaml')
    over = amcl_overrides(robot, float(arg('initial_x')), float(arg('initial_y')),
                          float(arg('initial_yaw')), fix)
    actions = [
        LogInfo(msg=f'[amr_slam3d] amcl_loc map={map_yaml} robot={robot} '
                    f'init=({over["initial_pose.x"]}, '
                    f'{over["initial_pose.y"]}, {over["initial_pose.yaw"]}) half_cell_fix={fix}'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', namespace='',
             output='screen', arguments=ros_args,
             parameters=[{'yaml_filename': map_yaml, 'topic_name': 'map', 'frame_id': 'map',
                          **common}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', namespace='', output='screen', arguments=ros_args,
             parameters=[{'autostart': True, 'node_names': ['map_server'],
                          'bond_timeout': BOND_TIMEOUT_S, **common}]),
    ]
    if fix:
        actions.append(Node(package='amr_localization', executable='amcl_map_adapter',
                            name='amcl_map_adapter', namespace=robot, output='screen',
                            arguments=ros_args,
                            parameters=[{'input_topic': '/map', 'output_topic': 'map_amcl',
                                         **common}]))
    actions += [
        Node(package='nav2_amcl', executable='amcl', name='amcl', namespace=robot,
             output='screen', arguments=ros_args, parameters=[amcl_yaml, over]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', namespace=robot, output='screen',
             arguments=ros_args,
             parameters=[{'autostart': True, 'node_names': ['amcl'],
                          'bond_timeout': BOND_TIMEOUT_S, **common}]),
    ]
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('robot', default_value='amr_01'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('half_cell_fix', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_setup),
    ])
