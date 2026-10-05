"""
3D LiDAR 점군 전처리 단독 기동 (보통은 amr_description spawn.launch.py 가 lidar3d 로봇에 자동으로 띄운다).

    ros2 launch amr_lidar3d preprocessor.launch.py namespace:=amr_01
    # SLAM 노드와 같은 컴포넌트 컨테이너에 올려 intra-process 로 넘길 때
    ros2 launch amr_lidar3d preprocessor.launch.py namespace:=amr_01 \
        container:=/amr_01/slam_container

인자
    namespace     로봇 네임스페이스 (기본 amr_01). 토픽은 그 아래 상대 이름
    input_topic   (기본 lidar3d/points_raw)  output_topic (기본 lidar3d/points)
    frame_id      '' = 입력 header.frame_id 유지 (Gazebo ignition_frame_id = <prefix>lidar3d_link)
    lazy          false(기본) = 기동 즉시 입력 구독. true = 출력 구독자가 있을 때만 입력 구독 —
                  sim 시각 T 에 켜면 센서가 T x 10 장을 몰아 렌더한다 (src/cloud_preprocessor.cpp 머리말)
    container     비우면 독립 프로세스, 컨테이너 이름을 주면 그 컨테이너에 컴포넌트로 싣는다
    use_sim_time  (기본 true)
계약: docs/research/slam3d/sensor.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    params = {'input_topic': arg('input_topic'), 'output_topic': arg('output_topic'),
              'frame_id': arg('frame_id'), 'lazy': arg('lazy').lower() in ('true', '1'),
              'use_sim_time': arg('use_sim_time').lower() in ('true', '1')}
    container = arg('container').strip()
    if container:
        return [LoadComposableNodes(target_container=container, composable_node_descriptions=[
            ComposableNode(package='amr_lidar3d', plugin='amr_lidar3d::CloudPreprocessor',
                           name='lidar3d_preprocessor', namespace=arg('namespace'),
                           parameters=[params],
                           extra_arguments=[{'use_intra_process_comms': True}])])]
    return [Node(package='amr_lidar3d', executable='cloud_preprocessor_node',
                 name='lidar3d_preprocessor', namespace=arg('namespace'), output='screen',
                 parameters=[params])]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='amr_01'),
        DeclareLaunchArgument('input_topic', default_value='lidar3d/points_raw'),
        DeclareLaunchArgument('output_topic', default_value='lidar3d/points'),
        DeclareLaunchArgument('frame_id', default_value=''),
        DeclareLaunchArgument('lazy', default_value='false'),
        DeclareLaunchArgument('container', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
