"""
slam_toolbox 매핑 행 (2D 기준선) — 기존 amr_localization/config/slam_toolbox.yaml 을 **읽기만** 해서 재사용한다.

    ros2 launch amr_slam3d slam_toolbox.launch.py

현행 2D 스택(localization.launch.py mode:=slam)과 같은 YAML·같은 입력(scan_filtered)이고, 다른 점은 셋이다
(보고서 「3D LiDAR SLAM 도입 방안」 5단계 'slam_toolbox(동기, scan_queue_size 상향)'):
  - sync_slam_toolbox_node — 재생 중 처리 지연 때 스캔을 버리지 않는다 (async 는 바쁘면 버린다)
  - scan_queue_size — 덮어쓴다 (humble 기본 1, tf2 MessageFilter 큐)
  - map_frame = slam_map — 월드 고정은 anchor_tf (map → slam_map, 스폰 자세)
노드 이름은 YAML 의 `/**/slam_toolbox` 와 맞춰 'slam_toolbox' 로 둔다.

인자: robot (기본 amr_01), map_frame (slam_map), scan_queue_size (10), mode (sync | async, 기본 sync),
      map_topic (기본 /amr_01/slam_toolbox/map — 다른 /map 과 섞이지 않게), log_level
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    robot = arg('robot').strip('/')
    prefix = f'{robot}/' if robot else ''
    mode = arg('mode').strip().lower()
    if mode not in ('sync', 'async'):
        raise RuntimeError(f'mode:={mode} (sync | async)')
    yaml_path = os.path.join(get_package_share_directory('amr_localization'), 'config',
                             'slam_toolbox.yaml')
    map_topic = arg('map_topic') or f'/{robot}/slam_toolbox/map'
    overrides = {'odom_frame': f'{prefix}odom', 'base_frame': f'{prefix}base_footprint',
                 'map_frame': arg('map_frame'), 'use_sim_time': True,
                 'scan_queue_size': int(arg('scan_queue_size')), 'map_name': map_topic}
    return [
        LogInfo(msg=f'[amr_slam3d] slam_toolbox {mode} yaml={yaml_path} (읽기 전용) '
                    f'map={map_topic}'),
        Node(package='slam_toolbox', executable=f'{mode}_slam_toolbox_node', name='slam_toolbox',
             namespace=robot, output='screen', parameters=[yaml_path, overrides],
             arguments=['--ros-args', '--log-level', arg('log_level')]),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='amr_01'),
        DeclareLaunchArgument('map_frame', default_value='slam_map'),
        DeclareLaunchArgument('scan_queue_size', default_value='10'),
        DeclareLaunchArgument('mode', default_value='sync'),
        DeclareLaunchArgument('map_topic', default_value=''),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_setup),
    ])
