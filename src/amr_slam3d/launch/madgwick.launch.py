"""
IMU 방위 재추정 — imu_filter_madgwick (use_mag false, publish_tf false). 파일럿 기록 때 함께 띄워 bag 에 남긴다.

    ros2 launch amr_slam3d madgwick.launch.py robot:=amr_01

입력 /<robot>/imu/data (imu_filter_node: 저역 통과 + 바이어스 보정, 방위 필드는 Gazebo 정답 그대로) →
출력 /<robot>/imu/data_madgwick (방위 = 자이로·가속도만으로 다시 추정). madgwick 은 입력 방위를 읽지 않는다.
리맵 규칙은 원래 이름에만 한 번 적용되므로(연쇄 없음) imu/data_raw→imu/data, imu/data→imu/data_madgwick 가
섞이지 않는다.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    robot = LaunchConfiguration('robot').perform(context).strip('/')
    params = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                                           'config', 'madgwick.yaml'))
    return [Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
                 name='imu_filter_madgwick', namespace=robot, output='screen',
                 parameters=[params],
                 remappings=[('imu/data_raw', LaunchConfiguration('input').perform(context)),
                             ('imu/data', LaunchConfiguration('output').perform(context))])]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='amr_01'),
        DeclareLaunchArgument('input', default_value='imu/data'),
        DeclareLaunchArgument('output', default_value='imu/data_madgwick'),
        OpaqueFunction(function=_setup),
    ])
