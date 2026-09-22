"""
Gazebo 없는 폐루프 시험대: 합성 창고 지도 + 운동학 시뮬레이터 + navigation.launch.py.

    ros2 launch amr_navigation kinematic_sim.launch.py [map:=<map.yaml>] [robot_name:=amr_01]
    ros2 run amr_navigation closed_loop_eval.py --mode follow --out /tmp/eval \
        --ros-args -r __ns:=/amr_01

map 이 비어 있으면 합성 창고 지도(warehouse_map.build_warehouse)를 임시 디렉토리에 만들어 쓴다.
map_server(/map, 공유) + map → <prefix>odom 항등 정적 TF
+ <prefix>base_footprint → <prefix>lidar_link 정적 TF (config/robot_params.yaml base_link_height
+ config/sensors.yaml lidar.extrinsic — 지면 +0.20, 앞 0.15) + kinematic_sim(odom·TF·스캔·참값) 을
띄운다. 스캔은 lidar_link 에서 720 빔, σ = sensors.yaml lidar.noise_stddev.
cmd_topic: 안전 노드 없이 시험할 때는 cmd_vel_smoothed (프로파일러 출력) 를 직접 적분한다.
"""
import os
import tempfile

import yaml

from amr_navigation import warehouse_map
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def lidar_mount(config_dir: str):
    """(x, z, σ, 빔 수): base_footprint 기준 lidar_link 위치와 거리 잡음 (없으면 재배치 후 기본값)."""
    x, z, sigma, beams = 0.15, 0.20, 0.03, 720
    try:
        with open(os.path.join(config_dir, 'robot_params.yaml'), encoding='utf-8') as f:
            base = yaml.safe_load(f)['/**']['ros__parameters']['robot']['base_link_height']
        with open(os.path.join(config_dir, 'sensors.yaml'), encoding='utf-8') as f:
            lidar = yaml.safe_load(f)['/**']['ros__parameters']['lidar']
        x = float(lidar['extrinsic']['x'])
        z = float(base) + float(lidar['extrinsic']['z'])
        sigma = float(lidar['noise_stddev'])
        beams = int(lidar['samples'])
    except (OSError, KeyError, TypeError, ValueError):
        pass
    return x, z, sigma, beams


def _setup(context):
    share = get_package_share_directory('amr_navigation')
    ros_ws = os.environ.get('ROS_WS', '/ros2_ws')
    config_dir = (LaunchConfiguration('config_dir').perform(context)
                  or os.path.join(ros_ws, 'config'))
    lx, lz, sigma, beams = lidar_mount(config_dir)
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
        'lidar_x': lx, 'scan_noise_std': sigma, 'scan_beams': beams,
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
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_lidar', namespace=robot_name,
             arguments=['--x', str(lx), '--z', str(lz), '--frame-id', prefix + 'base_footprint',
                        '--child-frame-id', prefix + 'lidar_link']),
        Node(package='amr_navigation', executable='kinematic_sim.py', name='kinematic_sim',
             namespace=robot_name, output='screen', parameters=[sim_params]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'navigation.launch.py')),
            launch_arguments={
                # 부모 범위의 launch 인자가 포함한 런치에 그대로 보이므로('' 이면 navigation.launch.py 기본값이
                # 가려진다) 항상 실제 경로를 넘긴다
                'params_file': LaunchConfiguration('params_file').perform(context)
                or os.path.join(share, 'config', 'nav2_params.yaml'),
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
        DeclareLaunchArgument(
            'params_file', default_value='',
            description="nav2 파라미터 ('' = amr_navigation config/nav2_params.yaml)"),
        DeclareLaunchArgument(
            'config_dir', default_value='',
            description="robot_params.yaml·sensors.yaml 디렉토리 ('' = $ROS_WS/config)"),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_setup),
    ])
