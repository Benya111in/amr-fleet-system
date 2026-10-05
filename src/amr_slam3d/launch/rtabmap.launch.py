"""
RTAB-Map 매핑 행 (구성 B) — 3D(점군)·2D(스캔)를 같은 설정으로 띄운다. 보너스 2 5단계 (amr-ai/slam 이미지).

    ros2 launch amr_slam3d rtabmap.launch.py dim:=3d database_path:=/out/rtabmap.db
    ros2 launch amr_slam3d rtabmap.launch.py dim:=2d database_path:=/out/rtabmap.db

구성 B (보고서 「3D LiDAR SLAM 도입 방안」 '위치추정은 AMCL 자리만…' 절): icp_odometry 가 휠-EKF odom 을 초기값
(guess_frame_id)으로 받아 `icp_odom → odom` 보정 TF 만 내고, rtabmap 이 `slam_map → icp_odom` 을 낸다.
TF 트리: map ─(anchor_tf, 스폰 자세)→ slam_map → <robot>/icp_odom → <robot>/odom(EKF, bag)
→ <robot>/base_footprint.

인자
    dim            3d | 2d. 3d = scan_cloud(cloud_topic) 구독, 2d = scan(scan_topic) 구독. 나머지 설정은 같다
                   (config/rtabmap.yaml 하나) — 차원 효과만 떼어 내기 위해 (보고서 요인 A).
    robot          네임스페이스·프레임 접두어 (기본 amr_01)
    cloud_topic    3D 입력 (기본 /<robot>/lidar3d/points — 전처리 출력, sensor.md)
    scan_topic     2D 입력 (기본 /<robot>/scan_filtered — slam_toolbox 행과 같은 2D 입력)
    imu_topic      madgwick 재추정 IMU (기본 /<robot>/imu/data_madgwick). '' = IMU 안 씀.
                   **정답 방위가 든 imu/data_raw·imu/data 는 넣지 않는다** (test_config.py 가 막는다)
    use_guess      true(기본) = 휠-EKF guess (요인 B 있음), false = guess 없음
    map_frame      rtabmap map_frame_id (기본 slam_map). 월드 고정은 anchor_tf 가 map → slam_map 으로 한다
    database_path  RTAB-Map DB (기본 /tmp/rtabmap_<dim>.db). 시작 때 지운다 (-d)
    params_file    기본 share/amr_slam3d/config/rtabmap.yaml
    log_level      (기본 info)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: 정답 방위가 든 IMU 토픽 — RTAB-Map 입력으로 쓰면 오라클 행이 된다 (sensor.md §6)
ORACLE_IMU_SUFFIXES = ('imu/data_raw', 'imu/data')


def check_imu_topic(topic: str) -> None:
    """정답 방위 IMU 를 SLAM 입력으로 주면 거절한다 (madgwick 출력만 허용)."""
    if topic and topic.rstrip('/').endswith(ORACLE_IMU_SUFFIXES):
        raise RuntimeError(f'imu_topic:={topic} 는 Gazebo 정답 방위를 담는다 — madgwick 출력'
                           '(imu/data_madgwick)만 쓴다 (docs/research/slam3d/sensor.md §6)')


def node_setup(dim: str, robot: str, cloud_topic: str, scan_topic: str, imu_topic: str,
               use_guess: bool, map_frame: str, database_path: str, params_file: str,
               log_level: str):
    """두 노드의 (파라미터, 리맵) — 런치와 시험(test_config.py)이 같이 쓴다."""
    if dim not in ('3d', '2d'):
        raise RuntimeError(f'dim:={dim} (3d | 2d)')
    check_imu_topic(imu_topic)
    prefix = f'{robot}/' if robot else ''
    frames = {'frame_id': f'{prefix}base_footprint'}
    odom = {**frames, 'odom_frame_id': f'{prefix}icp_odom',
            'guess_frame_id': f'{prefix}odom' if use_guess else '',
            'wait_imu_to_init': bool(imu_topic)}
    slam = {**frames, 'map_frame_id': map_frame, 'database_path': database_path,
            'subscribe_scan_cloud': dim == '3d', 'subscribe_scan': dim == '2d'}
    # 안 쓰는 입력은 없는 토픽으로 돌린다: icp_odometry 는 scan 과 scan_cloud 를 둘 다 구독한다 (기동 로그)
    if dim == '3d':
        inputs = [('scan_cloud', cloud_topic), ('scan', 'scan_not_used')]
    else:
        inputs = [('scan', scan_topic), ('scan_cloud', 'scan_cloud_not_used')]
    remaps = inputs + [('odom', 'icp_odom'), ('imu', imu_topic or 'imu_not_used')]
    args = ['--ros-args', '--log-level', log_level]
    return ([params_file, odom], [params_file, slam], remaps, args)


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    dim = arg('dim').strip().lower()
    robot = arg('robot').strip('/')
    cloud = arg('cloud_topic') or f'/{robot}/lidar3d/points'
    scan = arg('scan_topic') or f'/{robot}/scan_filtered'
    imu = arg('imu_topic')
    if imu == 'default':
        imu = f'/{robot}/imu/data_madgwick'
    db = arg('database_path') or f'/tmp/rtabmap_{dim}.db'
    params = arg('params_file') or os.path.join(
        get_package_share_directory('amr_slam3d'), 'config', 'rtabmap.yaml')
    use_guess = arg('use_guess').strip().lower() in ('true', '1', 'yes')
    odom_params, slam_params, remaps, ros_args = node_setup(
        dim, robot, cloud, scan, imu, use_guess, arg('map_frame'), db, params, arg('log_level'))
    return [
        LogInfo(msg=f'[amr_slam3d] rtabmap dim={dim} robot={robot} input='
                    f'{cloud if dim == "3d" else scan} imu={imu or "(없음)"} guess={use_guess} '
                    f'db={db}'),
        Node(package='rtabmap_odom', executable='icp_odometry', name='icp_odometry',
             namespace=robot, output='screen', parameters=odom_params, remappings=remaps,
             arguments=ros_args),
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', namespace=robot,
             output='screen', parameters=slam_params, remappings=remaps,
             arguments=['-d'] + ros_args),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('dim', default_value='3d', description='3d | 2d'),
        DeclareLaunchArgument('robot', default_value='amr_01'),
        DeclareLaunchArgument('cloud_topic', default_value=''),
        DeclareLaunchArgument('scan_topic', default_value=''),
        DeclareLaunchArgument('imu_topic', default_value='default',
                              description="'default' = /<robot>/imu/data_madgwick, '' = 없음"),
        DeclareLaunchArgument('use_guess', default_value='true'),
        DeclareLaunchArgument('map_frame', default_value='slam_map'),
        DeclareLaunchArgument('database_path', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('log_level', default_value='info'),
        OpaqueFunction(function=_setup),
    ])
