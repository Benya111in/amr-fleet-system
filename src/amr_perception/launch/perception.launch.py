"""
amr_perception 전체 기동 (components.md §3.4): 인식(YOLO → 3D → 마커), ArUco, 깊이 점군, 추적, 안전 게이트.

    ros2 launch amr_perception perception.launch.py                         # /amr_01, 접두사 없음
    ros2 launch amr_perception perception.launch.py robot_name:=amr_02 frame_prefix:=amr_02/
    ros2 launch amr_perception perception.launch.py use_yolo:=false device:=cpu

인자
    robot_name     네임스페이스 (기본 amr_01). 모든 토픽은 상대 이름 → /<robot_name>/...
    frame_prefix   TF 프레임 접두사 (기본 "" — 단일 로봇. 다중 로봇은 "<robot_name>/", multi_robot.md §2)
    use_sim_time   (기본 true)
    config_dir     robot_params.yaml, sensors.yaml 위치 (기본 $ROS_WS/config, 미설정 시 /ros2_ws/config)
    params_file    인지 파라미터 (기본 share/amr_perception/config/perception.yaml)
    device         yolo_node 장치 auto|cuda|cpu (기본 auto)
    weights        yolo_node 가중치 (기본 "" = params_file 값)
    use_yolo, use_localizer, use_markers, use_aruco, use_pointcloud, use_tracker, use_safety
                   노드별 켜기/끄기 (기본 모두 true)

파라미터 순서: robot_params.yaml → sensors.yaml → params_file → 런치 오버라이드 (뒤가 앞을 덮는다).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _nodes(context):
    lc = LaunchConfiguration
    robot = lc('robot_name').perform(context)
    prefix = lc('frame_prefix').perform(context)
    use_sim_time = lc('use_sim_time').perform(context).lower() in ('true', '1', 'yes')
    config_dir = lc('config_dir').perform(context)
    params_file = lc('params_file').perform(context)
    if not params_file:
        params_file = os.path.join(FindPackageShare('amr_perception').perform(context), 'config',
                                   'perception.yaml')
    shared = [os.path.join(config_dir, 'robot_params.yaml'),
              os.path.join(config_dir, 'sensors.yaml')]
    shared = [f for f in shared if os.path.exists(f)]
    common = {'use_sim_time': use_sim_time}
    framed = dict(common, frame_prefix=prefix)

    yolo_over = dict(common, device=lc('device').perform(context))
    weights = lc('weights').perform(context)
    if weights:
        yolo_over['weights'] = weights

    def node(executable, overrides, flag, env=None):
        return Node(
            package='amr_perception', executable=executable, name=executable, namespace=robot,
            output='screen', parameters=shared + [params_file, overrides],
            additional_env=env or {}, condition=IfCondition(lc(flag)))

    return [
        node('yolo_node', yolo_over, 'use_yolo', {'YOLO_CONFIG_DIR': '/tmp/Ultralytics'}),
        node('object_localizer_node', framed, 'use_localizer'),
        node('detection_marker_node', common, 'use_markers'),
        node('aruco_detector_node', framed, 'use_aruco'),
        node('pointcloud_filter_node', common, 'use_pointcloud'),
        node('obstacle_tracker_node', framed, 'use_tracker'),
        node('safety_node', framed, 'use_safety'),
    ]


def generate_launch_description() -> LaunchDescription:
    ws = os.environ.get('ROS_WS', '/ros2_ws')
    args = [
        DeclareLaunchArgument('robot_name', default_value='amr_01'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config_dir', default_value=os.path.join(ws, 'config')),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('device', default_value='auto'),
        DeclareLaunchArgument('weights', default_value=''),
    ]
    for flag in ('use_yolo', 'use_localizer', 'use_markers', 'use_aruco', 'use_pointcloud',
                 'use_tracker', 'use_safety'):
        args.append(DeclareLaunchArgument(flag, default_value='true'))
    return LaunchDescription(args + [OpaqueFunction(function=_nodes)])
