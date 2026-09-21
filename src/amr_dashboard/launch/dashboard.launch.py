"""
웹 대시보드 런치 (docker-compose dashboard 서비스가 실행).

    ros2 launch amr_dashboard dashboard.launch.py
        (인자) port:=8080  host:=127.0.0.1  config:=<yaml>  log_dir:=<dir>  use_sim_time:=true

host 기본값은 루프백이다: compose 가 network_mode: host 라 0.0.0.0 이면 인증 없는 E-stop 해제·작업 투입
API 가 LAN 전체에 열린다. 다른 PC 에서 볼 때만 host:=0.0.0.0 을 준다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare('amr_dashboard'), 'config', 'dashboard.yaml'])

    args = [
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='dashboard_node 파라미터 YAML'),
        DeclareLaunchArgument(
            'port', default_value='8080', description='HTTP 포트'),
        DeclareLaunchArgument(
            'host', default_value='127.0.0.1',
            description='HTTP 바인드 주소 (LAN 에 공개하려면 0.0.0.0)'),
        DeclareLaunchArgument(
            'log_dir', default_value='',
            description='조작 로그 디렉토리 (빈 문자열이면 $ROS_WS/logs)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='시뮬레이션 시계 사용 여부'),
    ]

    node = Node(
        package='amr_dashboard',
        executable='dashboard_node',
        name='dashboard_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),
            {
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                'host': ParameterValue(LaunchConfiguration('host'), value_type=str),
                'log_dir': ParameterValue(LaunchConfiguration('log_dir'), value_type=str),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            },
        ],
    )

    return LaunchDescription(args + [node])
