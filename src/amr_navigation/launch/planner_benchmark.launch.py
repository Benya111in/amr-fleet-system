"""
전역 계획기 벤치마크용 planner_server 단독 기동 (AStar / NavFn / Smac, 같은 전역 코스트맵).

    ros2 launch amr_navigation planner_benchmark.launch.py map:=<map.yaml>
    ros2 run amr_navigation bench_planners.py --map <map.yaml> --pairs <pairs.csv> --out <dir>

config/nav2_params.yaml 의 planner_server·global_costmap 설정을 그대로 쓰되, 센서가 없으므로 전역 코스트맵
레이어를 static + inflation 으로 줄이고(keepout 필터 제외) 프레임 접두어를 비운다. 로봇 TF 는 항등 정적 TF.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _setup(context):
    share = get_package_share_directory('amr_navigation')
    with open(os.path.join(share, 'config', 'nav2_params.yaml'), encoding='utf-8') as f:
        params = yaml.safe_load(f.read().replace('<prefix>', '').replace('<robot_ns>', ''))
    gc = params['global_costmap']['global_costmap']['ros__parameters']
    gc['plugins'] = ['static_layer', 'inflation_layer']
    gc.pop('filters', None)          # 빈 목록은 파라미터 값이 될 수 없다 → 키 자체를 뺀다 (기본 = 필터 없음)
    for layer in ('obstacle_layer', 'depth_layer', 'sensor_inflation_layer', 'keepout_filter'):
        gc.pop(layer, None)
    gc['use_sim_time'] = False
    ps = params['planner_server']['ros__parameters']
    ps['use_sim_time'] = False
    bench = {'planner_server': params['planner_server'],
             'global_costmap': params['global_costmap']}
    path = os.path.join(tempfile.mkdtemp(prefix='amr_nav_bench_'), 'bench_params.yaml')
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(bench, f)
    map_yaml = LaunchConfiguration('map').perform(context)
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': map_yaml, 'topic_name': '/map'}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_base',
             arguments=['--frame-id', 'map', '--child-frame-id', 'base_footprint']),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[path]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_bench', output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'planner_server']}]),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('map', description='map_server 형식 지도 YAML'),
        OpaqueFunction(function=_setup),
    ])
