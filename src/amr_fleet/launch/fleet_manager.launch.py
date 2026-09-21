"""
Fleet Manager + 로봇별 어댑터 런치 (docker-compose `fleet` 서비스가 사용).

    ros2 launch amr_fleet fleet_manager.launch.py
    ros2 launch amr_fleet fleet_manager.launch.py robot_ids:=amr_01,amr_02 \
        allocation_strategy:=hungarian comm_latency_ms:='[0.0, 100.0]'
    ros2 launch amr_fleet fleet_manager.launch.py auto_complete_after_s:=3.0   # 실행기 없이 모의 완료

- fleet_manager_node 는 /fleet, fleet_adapter_node 는 /<robot_id> 네임스페이스.
- 기본값은 config/fleet.yaml (params_file) 한 곳에 둔다. 아래 인자는 비어 있지 않을 때만 덮어쓴다.
- comm_latency_ms, simulate_latency 는 manager(assign_task 호출)와 adapter(robot_state) 양쪽에 준다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

# 인자 이름 → 값 변환. 빈 문자열이면 fleet.yaml 값을 그대로 쓴다.
_MANAGER_ARGS = ('allocation_strategy', 'comm_latency_ms', 'simulate_latency', 'log_dir')
_ADAPTER_ARGS = ('comm_latency_ms', 'simulate_latency', 'auto_complete_after_s',
                 'serve_assign_task', 'log_dir')


def _typed(name, text):
    """런치 인자 문자열 → 파라미터 값 (YAML 해석, 목록은 실수로 통일)."""
    if name in ('allocation_strategy', 'log_dir'):
        return text
    value = yaml.safe_load(text)
    if name == 'comm_latency_ms':
        if isinstance(value, (list, tuple)):
            return [float(v) for v in value]
        return float(value)
    if name == 'auto_complete_after_s':
        return float(value)
    if name in ('simulate_latency', 'serve_assign_task'):
        if not isinstance(value, bool):
            raise ValueError(f'{name} 는 true/false: {text!r}')
    return value


def _overrides(context, names):
    out = {}
    for name in names:
        text = LaunchConfiguration(name).perform(context).strip()
        if text:
            out[name] = _typed(name, text)
    return out


def _launch_setup(context):
    robot_ids = [s.strip() for s in LaunchConfiguration('robot_ids').perform(context).split(',')
                 if s.strip()]
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = _typed('use_sim_time', LaunchConfiguration('use_sim_time').perform(context))
    common = {'use_sim_time': bool(use_sim_time)}

    manager = Node(
        package='amr_fleet', executable='fleet_manager_node', name='fleet_manager_node',
        namespace='fleet', output='screen',
        parameters=[params_file, dict(common, robot_ids=','.join(robot_ids),
                                      **_overrides(context, _MANAGER_ARGS))])
    actions = [manager]
    if _typed('launch_adapters', LaunchConfiguration('launch_adapters').perform(context)):
        adapter_overrides = _overrides(context, _ADAPTER_ARGS)
        actions += [
            Node(package='amr_fleet', executable='fleet_adapter_node', name='fleet_adapter_node',
                 namespace=rid, output='screen',
                 parameters=[params_file, dict(common, robot_id=rid, **adapter_overrides)])
            for rid in robot_ids
        ]
    return actions


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution([FindPackageShare('amr_fleet'), 'config', 'fleet.yaml'])
    args = [
        DeclareLaunchArgument('robot_ids', default_value='amr_01,amr_02,amr_03,amr_04,amr_05',
                              description='쉼표로 구분한 로봇 네임스페이스 목록'),
        DeclareLaunchArgument('allocation_strategy', default_value='',
                              description="nearest | load_balance | hungarian ('' = fleet.yaml)"),
        DeclareLaunchArgument('comm_latency_ms', default_value='',
                              description="통신 지연 [lo, hi] ms 예: '[0.0, 100.0]' ('' = yaml)"),
        DeclareLaunchArgument('simulate_latency', default_value='',
                              description="true | false ('' = fleet.yaml)"),
        DeclareLaunchArgument('auto_complete_after_s', default_value='',
                              description='> 0 이면 어댑터가 N s 뒤 완료를 흉내 (실행기 없는 테스트)'),
        DeclareLaunchArgument('serve_assign_task', default_value='',
                              description="어댑터가 assign_task 제공 여부 (실행기가 제공하면 false)"),
        DeclareLaunchArgument('launch_adapters', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_dir', default_value='',
                              description="로그 디렉토리 ('' = fleet.yaml, 그것도 비면 $ROS_WS/logs)"),
        DeclareLaunchArgument('params_file', default_value=default_params),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
