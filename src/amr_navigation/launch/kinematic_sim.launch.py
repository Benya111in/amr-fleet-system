"""
Gazebo 없는 폐루프 시험대: 합성 창고 지도 + 운동학 시뮬레이터 + navigation.launch.py.

    ros2 launch amr_navigation kinematic_sim.launch.py [map:=<map.yaml>] [robot_name:=amr_01]
    ros2 run amr_navigation closed_loop_eval.py --mode follow --out /tmp/eval \
        --ros-args -r __ns:=/amr_01

map 이 비어 있으면 합성 창고 지도(warehouse_map.build_warehouse)를 임시 디렉토리에 만들어 쓴다.
map_server(/map, 공유) + map → <prefix>odom 항등 정적 TF + kinematic_sim(odom·TF·스캔·참값) 을 띄운다.
cmd_topic: 안전 노드 없이 시험할 때는 cmd_vel_smoothed (프로파일러 출력) 를 직접 적분한다.
"""
import os
import tempfile

from amr_navigation import warehouse_map
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    share = get_package_share_directory('amr_navigation')
    robot_name = LaunchConfiguration('robot_name').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    map_yaml = LaunchConfiguration('map').perform(context)
    if not map_yaml:
        out = os.path.join(tempfile.mkdtemp(prefix='amr_nav_map_'), 'warehouse')
        map_yaml = warehouse_map.write_map(warehouse_map.build_warehouse(), out)
    obstacles = [float(v) for v in
                 LaunchConfiguration('moving_obstacles').perform(context).split(',') if v]
    sim_params = {
        'map_yaml': map_yaml, 'frame_prefix': prefix,
        'cmd_topic': LaunchConfiguration('cmd_topic').perform(context),
        'x0': float(LaunchConfiguration('x0').perform(context)),
        'y0': float(LaunchConfiguration('y0').perform(context)),
        'yaw0': float(LaunchConfiguration('yaw0').perform(context)),
    }
    if obstacles:
        sim_params['moving_obstacles'] = obstacles
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': map_yaml, 'topic_name': '/map',
                                           'frame_id': 'map'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server']}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='map_to_odom', namespace=robot_name,
             arguments=['--frame-id', 'map', '--child-frame-id', prefix + 'odom']),
        Node(package='amr_navigation', executable='kinematic_sim.py', name='kinematic_sim',
             namespace=robot_name, output='screen', parameters=[sim_params]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'robot_name': robot_name, 'prefix': prefix, 'use_sim_time': 'false',
                'use_ttc_bt': LaunchConfiguration('use_ttc_bt').perform(context),
                'log_level': LaunchConfiguration('log_level').perform(context),
            }.items()),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='amr_01'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('cmd_topic', default_value='cmd_vel_smoothed'),
        DeclareLaunchArgument('x0', default_value='0.0'),
        DeclareLaunchArgument('y0', default_value='0.0'),
        DeclareLaunchArgument('yaw0', default_value='0.0'),
        DeclareLaunchArgument('moving_obstacles', default_value='',
                              description='x,y,vx,vy,r 반복 (왕복 등속 원판 장애물)'),
        DeclareLaunchArgument('use_ttc_bt', default_value='auto'),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_setup),
    ])
