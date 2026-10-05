"""
LIO-SAM 매핑 비교 행 — amr-ai/liosam:0.1 이미지에서 녹화 bag 에만 돌린다 (보고서: 별도 컨테이너, GTSAM 4.1.1).

    ros2 launch /ros2_ws/src/amr_slam3d/launch/liosam.launch.py loop:=true
    ros2 launch /ros2_ws/src/amr_slam3d/launch/liosam.launch.py loop:=false

이 이미지에는 우리 작업공간이 빌드돼 있지 않으므로 경로로 띄운다 — 설정은 이 파일 기준 ../config/liosam.yaml 이다
(소스 트리·설치 share 둘 다 같은 상대 위치). 상류 run.launch.py 와 다른 점: rviz·robot_state_publisher·
map→odom 항등 TF 를 띄우지 않는다 (URDF 는 bag 의 /tf_static, 월드 고정은 anchor_tf). imuPreintegration 실행 파일은
노드 둘(/lio_sam_imu_preintegration, /lio_sam_transformFusion)을 띄우므로 name= 을 주지 않는다
(s0-images VERSIONS.md §6 — 주면 두 노드 이름이 같아진다).

인자: loop (true | false — loopClosureEnableFlag 덮어쓰기), params_file (기본 ../config/liosam.yaml),
      log_level
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

EXECUTABLES = ('lio_sam_imuPreintegration', 'lio_sam_imageProjection',
               'lio_sam_featureExtraction', 'lio_sam_mapOptimization')


def default_params() -> str:
    return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                                         'config', 'liosam.yaml'))


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    loop = arg('loop').strip().lower()
    if loop not in ('true', 'false'):
        raise RuntimeError(f'loop:={loop} (true | false)')
    params = arg('params_file') or default_params()
    override = {'loopClosureEnableFlag': loop == 'true'}
    out = [LogInfo(msg=f'[amr_slam3d] LIO-SAM loop={loop} params={params}')]
    out += [Node(package='lio_sam', executable=exe, output='screen',
                 parameters=[params, override],
                 arguments=['--ros-args', '--log-level', arg('log_level')])
            for exe in EXECUTABLES]
    return out


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_setup),
    ])
