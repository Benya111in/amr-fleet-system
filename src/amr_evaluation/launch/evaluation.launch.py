"""
성능 측정 로거 일괄 기동 (명세 4.10).

    ros2 launch amr_evaluation evaluation.launch.py run_name:=ekf_tuning_01 namespace:=amr_01

pose_error_logger / cte_logger / response_time_logger / cpu_sampler 를 같은 런 디렉토리
(<output_dir>/<run_name>/, 기본 $ROS_WS/logs/eval/<run_name>/) 로 기동한다.
끝나면: ros2 run amr_evaluation analyze --input <런 디렉토리>
"""

from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 런치 파일을 읽는 순간 한 번만 정해져 네 로거가 같은 디렉토리를 쓴다
_DEFAULT_RUN_NAME = datetime.now().strftime('run_%Y%m%d_%H%M%S')
_DEFAULT_OUTPUT_DIR = os.path.join(os.environ.get('ROS_WS', os.getcwd()), 'logs', 'eval')


def generate_launch_description() -> LaunchDescription:
    params_file = os.path.join(
        get_package_share_directory('amr_evaluation'), 'config', 'amr_evaluation.yaml')

    run_name = LaunchConfiguration('run_name')
    output_dir = LaunchConfiguration('output_dir')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = LaunchConfiguration('params_file')

    common = [params, {
        'run_name': run_name,
        'output_dir': output_dir,
        'use_sim_time': use_sim_time,
    }]

    def logger(executable: str, extra: dict = None) -> Node:
        return Node(
            package='amr_evaluation', executable=executable, name=executable,
            namespace=namespace, parameters=common + ([extra] if extra else []),
            output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('run_name', default_value=_DEFAULT_RUN_NAME,
                              description='런 이름 → <output_dir>/<run_name>/'),
        DeclareLaunchArgument('output_dir', default_value=_DEFAULT_OUTPUT_DIR,
                              description='로그 루트 (기본 $ROS_WS/logs/eval)'),
        DeclareLaunchArgument('namespace', default_value='amr_01',
                              description='측정 대상 로봇 네임스페이스'),
        DeclareLaunchArgument('robot_id', default_value=namespace,
                              description='응답 시간: 이 로봇의 작업 이벤트만 (기본 = namespace)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=params_file),
        logger('pose_error_logger'),
        logger('cte_logger'),
        logger('response_time_logger', {'robot_id': LaunchConfiguration('robot_id')}),
        logger('cpu_sampler'),
    ])
