"""
안전 게이트 Gazebo 시험 하네스 (tracking.md §9.4).

    python3 safety_test_world.py $(ros2 pkg prefix amr_simulation)/share/amr_simulation/worlds/\
warehouse.sdf /tmp/perc/warehouse.sdf
    ros2 launch <이 파일> world:=/tmp/perc/warehouse.sdf x:=-6.0 y:=14.0 yaw:=0.0
    python3 safety_gz_trials.py --out /tmp/perc/trials.json

구성 (위치 추정 스택 대신 지면 진실 TF, 계약 C4 map = world)
    amr_simulation warehouse.launch.py (시험 월드, 로봇 amr_01, 접두사 없음)
    ros_gz_bridge: 이동체 cmd_vel(ROS→GZ)·odometry(GZ→ROS)  /model/obs_*/...
    amr_localization: wheel_odometry_node, imu_filter_node, scan_filter_node (EKF 없음)
    gt_tf_relay.py: ground_truth/odom → TF odom→base_footprint, map→odom 항등
    amr_perception: pointcloud_filter_node, safety_node (+ 선택 obstacle_tracker_node)
        safety_node 의 휠·IMU 감시 토픽은 gated:=true 면 wheel_odom_gated / imu/data_gated
        (시험기가 끊김을 주입해 중계한다 — 센서 디바운스 시험)
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
MOVERS = ('obs_person', 'obs_box_low')


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    ns = 'amr_01'
    ws = os.environ.get('ROS_WS', '/ros2_ws')
    config_dir = os.path.join(ws, 'config')
    loc_share = get_package_share_directory('amr_localization')
    perc_share = get_package_share_directory('amr_perception')
    sim_share = get_package_share_directory('amr_simulation')
    # localization.launch.py 의 공유 파라미터 함수 재사용 (robot_params·sensors.yaml → 노드 파라미터)
    spec = importlib.util.spec_from_file_location(
        'loc_launch', os.path.join(loc_share, 'launch', 'localization.launch.py'))
    loc = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loc)
    shared = loc.shared_parameters(config_dir)
    common = {'use_sim_time': True}
    cfg = os.path.join(loc_share, 'config')

    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'warehouse.launch.py')),
        launch_arguments={'world': arg('world'), 'x': arg('x'), 'y': arg('y'),
                          'yaw': arg('yaw'), 'obstacle_truth': 'false',
                          'collision_monitor': 'false'}.items())]
    bridge_args = []
    for m in MOVERS:
        bridge_args += [f'/model/{m}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                        f'/model/{m}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry']
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='mover_bridge', arguments=bridge_args, parameters=[common]))

    def node(pkg, exe, params):
        return Node(package=pkg, executable=exe, name=exe, namespace=ns, output='screen',
                    parameters=params)

    actions += [
        node('amr_localization', 'wheel_odometry_node',
             [os.path.join(cfg, 'wheel_odometry.yaml'),
              dict(shared['wheel'], frame_prefix='', **common)]),
        node('amr_localization', 'imu_filter_node',
             [os.path.join(cfg, 'imu_filter.yaml'), dict(shared['imu'], **common)]),
        node('amr_localization', 'scan_filter_node',
             [os.path.join(cfg, 'scan_filter.yaml'), dict(shared['scan'], **common)]),
        ExecuteProcess(cmd=['python3', os.path.join(HERE, 'gt_tf_relay.py'), '--ros-args',
                            '-r', f'__ns:=/{ns}', '-p', 'use_sim_time:=true'], output='screen'),
    ]
    files = [os.path.join(config_dir, 'robot_params.yaml'),
             os.path.join(config_dir, 'sensors.yaml'),
             os.path.join(perc_share, 'config', 'perception.yaml')]
    safety = dict(common, frame_prefix='')
    if arg('gated').lower() == 'true':
        safety['sensor_topics.wheel_encoder'] = 'wheel_odom_gated'
        safety['sensor_topics.imu'] = 'imu/data_gated'
    actions += [node('amr_perception', 'pointcloud_filter_node', files + [common]),
                node('amr_perception', 'safety_node', files + [safety])]
    if arg('with_tracker').lower() == 'true':
        actions.append(node('amr_perception', 'obstacle_tracker_node',
                            files + [dict(common, frame_prefix='')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', description='safety_test_world.py 가 만든 SDF 절대 경로'),
        DeclareLaunchArgument('x', default_value='-6.0'),
        DeclareLaunchArgument('y', default_value='14.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gated', default_value='true'),
        DeclareLaunchArgument('with_tracker', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
