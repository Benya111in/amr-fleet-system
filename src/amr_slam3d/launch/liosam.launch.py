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
      log_level, 그리고 평면·절제 변형 행(리서치 R2·R4·R6)용 덮어쓰기 — 빈 값이면 설정 파일 그대로:
      imu_topic (imuTopic), rpy_weight (imuRPYWeight), z_tol (z_tollerance),
      rot_tol (rotation_tollerance),
      oracle (true 일 때만 정답 방위 IMU 토픽을 받는다 — 상한 참고 행. 아니면 거절한다)
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


#: oracle:=true 없이는 거절하는 IMU 토픽 접미사. data_raw·data_gtorient = 정답 방위
#: (sensor.md §6, madg_lockstep.py),
#: imu/data = 방위가 항등이라 roll·pitch 0 이 평평한 월드의 정답과 같다 (imu_filter_node.cpp 147–149행)
ORACLE_IMU_SUFFIXES = ('imu/data_raw', 'imu/data', 'imu/data_gtorient')


def overrides(loop: str, imu_topic: str = '', rpy_weight: str = '', z_tol: str = '',
              rot_tol: str = '', oracle: str = 'false') -> dict:
    """런치 인자 → 설정 파일 위에 덮어쓸 파라미터 (런치와 시험이 같이 쓴다)."""
    loop, oracle = loop.strip().lower(), oracle.strip().lower()
    if loop not in ('true', 'false'):
        raise RuntimeError(f'loop:={loop} (true | false)')
    if oracle not in ('true', 'false'):
        raise RuntimeError(f'oracle:={oracle} (true | false)')
    out = {'loopClosureEnableFlag': loop == 'true'}
    imu = imu_topic.strip()
    is_oracle = bool(imu) and imu.rstrip('/').endswith(ORACLE_IMU_SUFFIXES)
    if is_oracle and oracle != 'true':
        raise RuntimeError(f'imu_topic:={imu} 는 Gazebo 정답 방위를 담는다 — 상한 참고 행만 oracle:=true 로 쓴다')
    if oracle == 'true' and not is_oracle:
        raise RuntimeError(f'oracle:=true 인데 imu_topic:={imu or "(설정 파일)"} 는 정답 방위 토픽이 아니다')
    if imu:
        out['imuTopic'] = imu
    for key, val in (('imuRPYWeight', rpy_weight), ('z_tollerance', z_tol),
                     ('rotation_tollerance', rot_tol)):
        if val.strip():
            v = float(val)
            if v < 0.0:
                raise RuntimeError(f'{key}={v} < 0')
            out[key] = v
    return out


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    override = overrides(arg('loop'), arg('imu_topic'), arg('rpy_weight'), arg('z_tol'),
                         arg('rot_tol'), arg('oracle'))
    params = arg('params_file') or default_params()
    out = [LogInfo(msg=f'[amr_slam3d] LIO-SAM params={params} overrides={override}')]
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
        DeclareLaunchArgument('imu_topic', default_value=''),
        DeclareLaunchArgument('rpy_weight', default_value=''),
        DeclareLaunchArgument('z_tol', default_value=''),
        DeclareLaunchArgument('rot_tol', default_value=''),
        DeclareLaunchArgument('oracle', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
