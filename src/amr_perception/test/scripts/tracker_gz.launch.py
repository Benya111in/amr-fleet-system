"""
추적기 Gazebo 시험 하네스 — 실제 AMCL + EKF 위치 추정에서 정적 배경 분리·동적 판정 (tracking.md §9.5).

    ros2 launch <이 파일> align_to_map:=true
    python3 tracker_gz_route.py --out /tmp/perc/route.json

구성: amr_simulation warehouse.launch.py (창고 월드 그대로: actor 6 + 지게차·셔틀, obstacle_truth 켬),
amr_localization localization.launch.py (mode localization, $ROS_WS/maps/warehouse.yaml,
AMCL + EKF 2 개), amr_perception pointcloud_filter_node + obstacle_tracker_node + safety_node
(YOLO 없음). 접두사 없음 (1 대).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

START = ('3.0', '-4.6', '-1.5707963')   # 좁은 통로 입구 앞 횡단 작업자(y = -7, 1.0 m/s)를 보는 자세


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    ws = os.environ.get('ROS_WS', '/ros2_ws')
    config_dir = os.path.join(ws, 'config')
    sim = get_package_share_directory('amr_simulation')
    loc = get_package_share_directory('amr_localization')
    perc = get_package_share_directory('amr_perception')
    x, y, yaw = START
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'warehouse.launch.py')),
            launch_arguments={'x': x, 'y': y, 'yaw': yaw, 'obstacle_truth': 'true',
                              'collision_monitor': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(loc, 'launch', 'localization.launch.py')),
            # 납치 감시는 끈다: 과부하(RTF 0.2)에서 정답 자세의 scan-map inlier 0.35 로 '분실' 을 선언하고 AMCL
            # 전역 재초기화를 걸어 추정이 18 m 튀었다 (실측) — 추적기 시험은 AMCL + EKF 만으로 한다
            launch_arguments={'mode': 'localization', 'robot_name': 'amr_01', 'frame_prefix': '',
                              'map': os.path.join(ws, 'maps', 'warehouse.yaml'),
                              'use_kidnap_monitor': 'false',
                              'initial_x': x, 'initial_y': y, 'initial_yaw': yaw}.items()),
    ]
    files = [os.path.join(config_dir, 'robot_params.yaml'),
             os.path.join(config_dir, 'sensors.yaml'),
             os.path.join(perc, 'config', 'perception.yaml')]
    common = {'use_sim_time': True, 'frame_prefix': ''}
    tracker = dict(common, **{'background.align_to_map': arg('align_to_map').lower() == 'true'})
    for exe, over in (('pointcloud_filter_node', {'use_sim_time': True}),
                      ('obstacle_tracker_node', tracker), ('safety_node', common)):
        actions.append(Node(package='amr_perception', executable=exe, name=exe,
                            namespace='amr_01', output='screen', parameters=files + [over]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('align_to_map', default_value='true',
                              description='false = 예전 고정 반경 배경 판정 (비교용)'),
        OpaqueFunction(function=_setup),
    ])
