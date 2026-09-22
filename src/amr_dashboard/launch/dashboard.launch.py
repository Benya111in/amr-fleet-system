"""
웹 대시보드 런치 (docker-compose dashboard 서비스가 실행).

    ros2 launch amr_dashboard dashboard.launch.py
        (인자) config:=<yaml>  port:=8080  host:=127.0.0.1  log_dir:=<dir>  api_token:=<문자열>
               use_sim_time:=true

host / port / log_dir / api_token 은 **준 것만** config 값을 덮어쓴다 (기본 '' = config 값 그대로) —
config:= 로 넘긴 YAML 의 값이 launch 기본값에 가려지지 않는다.
host 기본값(config)은 루프백이다: compose 가 network_mode: host 라 0.0.0.0 이면 조작 API 가 LAN 에
열린다. 다른 PC 에서 볼 때만 host:=0.0.0.0 을 주고, allowed_hosts·api_token 을 함께 설정한다.
토큰은 환경 변수 AMR_DASHBOARD_TOKEN 으로도 줄 수 있다 (launch 인자가 우선). 파라미터로 준 토큰은 같은
ROS 도메인에서 ros2 param get 으로 읽히므로 공유 서버에서는 환경 변수를 쓴다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# launch 인자 → 파라미터 이름·형 변환 (값이 '' 이면 config 를 그대로 둔다)
OVERRIDES = {'host': str, 'port': int, 'log_dir': str, 'api_token': str}


def overrides_from(values: dict) -> dict:
    """비어 있지 않은 launch 인자만 파라미터로 (port 는 정수). 형이 틀리면 ValueError."""
    out = {}
    for name, cast in OVERRIDES.items():
        raw = (values.get(name) or '').strip()
        if raw:
            out[name] = cast(raw)
    return out


def node_parameters(config: str, values: dict, use_sim_time: str) -> list:
    """Node parameters: [config YAML, {준 인자만 + use_sim_time}] — 뒤의 dict 가 YAML 을 덮어쓴다."""
    flag = (use_sim_time or '').strip().lower() in ('true', '1', 'yes')
    return [config, dict(overrides_from(values), use_sim_time=flag)]


def _node(context):
    values = {name: LaunchConfiguration(name).perform(context) for name in OVERRIDES}
    params = node_parameters(LaunchConfiguration('config').perform(context), values,
                             LaunchConfiguration('use_sim_time').perform(context))
    return [Node(
        package='amr_dashboard',
        executable='dashboard_node',
        name='dashboard_node',
        output='screen',
        parameters=params,
    )]


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare('amr_dashboard'), 'config', 'dashboard.yaml'])
    args = [
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='dashboard_node 파라미터 YAML'),
        DeclareLaunchArgument(
            'port', default_value='', description='HTTP 포트 (빈 값: config 값, 기본 8080)'),
        DeclareLaunchArgument(
            'host', default_value='',
            description='HTTP 바인드 주소 (빈 값: config 값, 기본 127.0.0.1. LAN 공개는 0.0.0.0)'),
        DeclareLaunchArgument(
            'log_dir', default_value='',
            description='조작 로그 디렉토리 (빈 값: config 값, 기본 $ROS_WS/logs)'),
        DeclareLaunchArgument(
            'api_token', default_value='',
            description='조작 API 토큰 (빈 값: config 값 → $AMR_DASHBOARD_TOKEN → 없음)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='시뮬레이션 시계 사용 여부'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_node)])
