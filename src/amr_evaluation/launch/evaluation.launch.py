"""
성능 측정 로거 일괄 기동 (명세 4.10).

    ros2 launch amr_evaluation evaluation.launch.py run_name:=ekf_tuning_01 namespace:=amr_01

pose_error_logger / cte_logger / response_time_logger / cpu_sampler 를 같은 런 디렉토리
(<output_dir>/<run_name>/, 기본 $ROS_WS/logs/eval/<run_name>/) 로 기동한다. CSV 이름에 네임스페이스가
붙으므로(pose_error_amr_01.csv) 로봇마다 같은 run_name 으로 띄워도 된다 — CPU 는 호스트 값이라 한 번만
재도록 두 번째 로봇부터 with_cpu:=false 를 준다.
use_sim_time 기본값은 true — fleet_manager.launch.py · dashboard.launch.py 와 같다 (multi_robot.md §6).
시계가 다른 발행자가 섞이면 로거가 경고하고 짝짓지 않는다.
끝나면: ros2 run amr_evaluation analyze --input <런 디렉토리>
"""

from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 런치 파일을 읽는 순간 한 번만 정해져 네 로거가 같은 디렉토리를 쓴다
_DEFAULT_RUN_NAME = datetime.now().strftime('run_%Y%m%d_%H%M%S')
_DEFAULT_OUTPUT_DIR = os.path.join(os.environ.get('ROS_WS', os.getcwd()), 'logs', 'eval')


def _as_str(name: str) -> ParameterValue:
    """Launch 인자를 문자열 파라미터로 (run_name:=20260922 같은 숫자도 STRING 으로 넘긴다)."""
    return ParameterValue(LaunchConfiguration(name), value_type=str)


def generate_launch_description() -> LaunchDescription:
    params_file = os.path.join(
        get_package_share_directory('amr_evaluation'), 'config', 'amr_evaluation.yaml')

    namespace = LaunchConfiguration('namespace')
    params = LaunchConfiguration('params_file')

    common = [params, {
        'run_name': _as_str('run_name'),
        'output_dir': _as_str('output_dir'),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }]

    def logger(executable: str, extra: dict = None, condition=None) -> Node:
        return Node(
            package='amr_evaluation', executable=executable, name=executable,
            namespace=namespace, parameters=common + ([extra] if extra else []),
            output='screen', condition=condition)

    return LaunchDescription([
        DeclareLaunchArgument('run_name', default_value=_DEFAULT_RUN_NAME,
                              description='런 이름 → <output_dir>/<run_name>/'),
        DeclareLaunchArgument('output_dir', default_value=_DEFAULT_OUTPUT_DIR,
                              description='로그 루트 (기본 $ROS_WS/logs/eval)'),
        DeclareLaunchArgument('namespace', default_value='amr_01',
                              description='측정 대상 로봇 네임스페이스'),
        DeclareLaunchArgument('robot_id', default_value=namespace,
                              description='응답 시간: 이 로봇의 작업 이벤트만 (기본 = namespace, '
                                          '앞뒤 / 는 노드가 뗀다)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='시뮬레이션 시계 (fleet·dashboard 런치 기본값과 같게 true)'),
        DeclareLaunchArgument('with_cpu', default_value='true',
                              description='cpu_sampler 기동 (여러 로봇을 잴 때는 한 곳에서만 true)'),
        DeclareLaunchArgument('params_file', default_value=params_file),
        logger('pose_error_logger'),
        logger('cte_logger'),
        logger('response_time_logger', {'robot_id': _as_str('robot_id')}),
        logger('cpu_sampler', condition=IfCondition(LaunchConfiguration('with_cpu'))),
    ])
