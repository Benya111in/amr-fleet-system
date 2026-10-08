"""
Fleet Manager + 로봇별 어댑터 런치 (docker-compose `fleet` 서비스가 사용).

    ros2 launch amr_fleet fleet_manager.launch.py
    ros2 launch amr_fleet fleet_manager.launch.py robot_ids:=amr_01,amr_02 \
        allocation_strategy:=hungarian comm_latency_ms:='[0.0, 100.0]'
    # 시뮬레이터(/clock)·실행기 없이 단독 모의 시험
    ros2 launch amr_fleet fleet_manager.launch.py use_sim_time:=false auto_complete_after_s:=3.0
    # 교통 관리자 끄기 / 탐지만 (기준선 측정)
    ros2 launch amr_fleet fleet_manager.launch.py traffic_manager:=false
    ros2 launch amr_fleet fleet_manager.launch.py traffic_mode:=observe
    # 중앙과 어댑터를 다른 프로세스·컨테이너로 (중앙을 멈춰도 어댑터는 산다 — 컨테이너 단위 장애 실험)
    ros2 launch amr_fleet fleet_manager.launch.py launch_adapters:=false      # 중앙 두 노드만
    ros2 launch amr_fleet fleet_adapters.launch.py robot_ids:=amr_01,amr_02  # 어댑터만

- fleet_manager_node · traffic_manager_node 는 /fleet, fleet_adapter_node 는 /<robot_id> 네임스페이스.
- traffic_manager:=true(기본)면 traffic_manager_node 를 config/traffic.yaml(traffic_params_file)로
  함께 띄운다 (robot_ids · use_sim_time 은 같은 값, traffic_mode 가 비어 있지 않으면 mode 를 덮어쓴다).
- 기본값은 config/fleet.yaml (params_file) 한 곳에 둔다. 아래 인자는 비어 있지 않을 때만 덮어쓴다.
- comm_latency_ms, simulate_latency, seed, comm_model_file 은 manager(assign_task 호출)와
  adapter(robot_state) 양쪽에 준다 (통신 모의 CommEmulator — 링크마다 결정적 난수, p0.md §1).
- launch_manager · traffic_manager · launch_adapters (기본 모두 true) 로 띄울 묶음을 고른다. 기본값으로는
  예전과 같이 한 런치(한 컨테이너)에 셋이 다 뜬다. fleet_adapters.launch.py 는 이 파일을
  launch_manager:=false traffic_manager:=false 기본값으로 다시 쓴다 (인자 해석이 한 곳에 있다).
- use_sim_time 기본 true: 전 노드가 /clock 을 쓴다 (multi_robot.md §6, dashboard·evaluation 런치와 같다).
  /clock 없이 단독으로 띄우면 노드 시계가 0 에 멈추므로 use_sim_time:=false 를 준다.
- serve_assign_task: 비우면 auto_complete_after_s > 0 (모의 시험)일 때만 true, 그 외에는 fleet.yaml(false).
  실제 assign_task 서버는 task_executor_node 이므로 통합 실행에서 어댑터가 서버를 띄우지 않는다.
- fleet_mode (central 기본 | distributed — 보너스 4, fleet-distributed/p3.md §10): distributed 면
  관리자가 /fleet/heartbeat 를 내고 heartbeat_observe_s 동안 관찰자로 로봇 펜스 epoch 를 모은 뒤 명령한다
  (heartbeat.enabled · observe_s, epoch_file 은 인자 그대로), 교통 관리자는 구역 토큰을 끈다
  (zones.tokens false — 구역 허가는 로봇 정족수 투표가 진다. 교차 예측 · 교착 해소는 그대로,
  실행기는
  서버가 살아 있을 때만 hold 를 따른다). 로봇 쪽(에이전트 · 실행기 리매핑)은 behavior.launch.py 의 같은 인자가
  맡는다.
  상위 런치(multi_robot.launch.py)의 값을 launch 인자 상속으로 받는다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

# 인자 이름 → 값 변환. 빈 문자열이면 fleet.yaml 값을 그대로 쓴다.
_MANAGER_ARGS = ('allocation_strategy', 'comm_latency_ms', 'simulate_latency', 'seed',
                 'comm_model_file', 'log_dir', 'epoch_file')
FLEET_MODES = ('central', 'distributed')
_ADAPTER_ARGS = ('comm_latency_ms', 'simulate_latency', 'seed', 'comm_model_file',
                 'auto_complete_after_s', 'serve_assign_task', 'log_dir')
# generate_launch_description(defaults) 로 기본값을 바꿀 수 있는 인자
_ARG_NAMES = frozenset({'launch_manager', 'launch_adapters', 'traffic_manager'})


def _typed(name, text):
    """런치 인자 문자열 → 파라미터 값 (YAML 해석, 목록은 실수로 통일)."""
    if name in ('allocation_strategy', 'log_dir', 'comm_model_file', 'epoch_file'):
        return text
    value = yaml.safe_load(text)
    if name == 'comm_latency_ms':
        if isinstance(value, (list, tuple)):
            return [float(v) for v in value]
        return float(value)
    if name == 'auto_complete_after_s':
        return float(value)
    if name == 'seed':
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f'seed 는 0 이상의 정수: {text!r}')
        return value
    if name in ('simulate_latency', 'serve_assign_task', 'traffic_manager', 'launch_adapters',
                'launch_manager'):
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


def fleet_mode(context) -> str:
    """분산 모드 인자 fleet_mode 값 (상위 런치가 주지 않았으면 central)."""
    mode = (context.launch_configurations.get('fleet_mode', '') or 'central').strip().lower()
    if mode not in FLEET_MODES:
        raise ValueError(f'fleet_mode 는 {" | ".join(FLEET_MODES)}: {mode!r}')
    return mode


def _launch_setup(context):
    robot_ids = [s.strip() for s in LaunchConfiguration('robot_ids').perform(context).split(',')
                 if s.strip()]
    distributed = fleet_mode(context) == 'distributed'
    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = _typed('use_sim_time', LaunchConfiguration('use_sim_time').perform(context))
    common = {'use_sim_time': bool(use_sim_time)}

    actions = []
    if _typed('launch_manager', LaunchConfiguration('launch_manager').perform(context)):
        manager = dict(common, robot_ids=','.join(robot_ids), **_overrides(context, _MANAGER_ARGS))
        if distributed:
            manager['heartbeat.enabled'] = True
            manager['heartbeat.observe_s'] = float(
                context.launch_configurations.get('heartbeat_observe_s', '') or 3.0)
        actions.append(Node(
            package='amr_fleet', executable='fleet_manager_node', name='fleet_manager_node',
            namespace='fleet', output='screen', parameters=[params_file, manager]))
    if _typed('traffic_manager', LaunchConfiguration('traffic_manager').perform(context)):
        traffic_overrides = dict(common, robot_ids=','.join(robot_ids))
        mode = LaunchConfiguration('traffic_mode').perform(context).strip()
        if mode:
            traffic_overrides['mode'] = mode
        if distributed:
            traffic_overrides['zones.tokens'] = False     # 구역 허가는 로봇 정족수 (분산 모드)
        actions.append(Node(
            package='amr_fleet', executable='traffic_manager_node', name='traffic_manager_node',
            namespace='fleet', output='screen',
            parameters=[LaunchConfiguration('traffic_params_file').perform(context),
                        traffic_overrides]))
    if _typed('launch_adapters', LaunchConfiguration('launch_adapters').perform(context)):
        adapter_overrides = _overrides(context, _ADAPTER_ARGS)
        if ('serve_assign_task' not in adapter_overrides
                and adapter_overrides.get('auto_complete_after_s', 0.0) > 0.0):
            adapter_overrides['serve_assign_task'] = True   # 모의 완료는 어댑터 서버와 짝
        actions += [
            Node(package='amr_fleet', executable='fleet_adapter_node', name='fleet_adapter_node',
                 namespace=rid, output='screen',
                 parameters=[params_file, dict(common, robot_id=rid, **adapter_overrides)])
            for rid in robot_ids
        ]
    return actions


def generate_launch_description(defaults=None) -> LaunchDescription:
    """
    런치 설명. defaults 로 인자 기본값을 바꿀 수 있다 (fleet_adapters.launch.py 가 쓴다).

    ros2 launch 는 인자 없이 부른다 (기본값 그대로).
    """
    defaults = dict(defaults or {})
    unknown = set(defaults) - _ARG_NAMES
    if unknown:
        raise ValueError(f'모르는 런치 인자 기본값: {sorted(unknown)}')
    default_params = PathJoinSubstitution([FindPackageShare('amr_fleet'), 'config', 'fleet.yaml'])
    traffic_params = PathJoinSubstitution([FindPackageShare('amr_fleet'), 'config',
                                           'traffic.yaml'])
    args = [
        DeclareLaunchArgument('robot_ids', default_value='amr_01,amr_02,amr_03,amr_04,amr_05',
                              description='쉼표로 구분한 로봇 네임스페이스 목록'),
        DeclareLaunchArgument('allocation_strategy', default_value='',
                              description="nearest | load_balance | hungarian ('' = fleet.yaml)"),
        DeclareLaunchArgument('comm_latency_ms', default_value='',
                              description="통신 지연 [lo, hi] ms 예: '[0.0, 100.0]' ('' = yaml)"),
        DeclareLaunchArgument('simulate_latency', default_value='',
                              description="true | false ('' = fleet.yaml)"),
        DeclareLaunchArgument('seed', default_value='',
                              description="통신 모의 루트 시드 (0 = 무작위로 뽑아 기록, '' = fleet.yaml)"),
        DeclareLaunchArgument('comm_model_file', default_value='',
                              description="통신 모의 YAML (링크별 규칙 · 버스트 · 분할, '' = fleet.yaml)"),
        DeclareLaunchArgument('auto_complete_after_s', default_value='',
                              description='> 0 이면 어댑터가 N s 뒤 완료를 흉내 (실행기 없는 테스트)'),
        DeclareLaunchArgument('serve_assign_task', default_value='',
                              description="어댑터 모의 assign_task 서버 ('' = 모의 완료 시에만 true)"),
        DeclareLaunchArgument('launch_manager',
                              default_value=defaults.get('launch_manager', 'true'),
                              description='fleet_manager_node 실행 (/fleet)'),
        DeclareLaunchArgument('launch_adapters',
                              default_value=defaults.get('launch_adapters', 'true'),
                              description='로봇별 fleet_adapter_node 실행 (false = 중앙 노드만 — 어댑터는 '
                                          'fleet_adapters.launch.py 로 다른 프로세스·컨테이너에)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='/clock 사용 (시뮬레이터 없이 단독 실행이면 false)'),
        DeclareLaunchArgument('log_dir', default_value='',
                              description="로그 디렉토리 ('' = fleet.yaml, 그것도 비면 $ROS_WS/logs)"),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('traffic_manager',
                              default_value=defaults.get('traffic_manager', 'true'),
                              description='traffic_manager_node 실행 (교통 관리 · 교착 탐지/해소)'),
        DeclareLaunchArgument('traffic_mode', default_value='',
                              description="active | observe ('' = traffic.yaml)"),
        DeclareLaunchArgument('traffic_params_file', default_value=traffic_params),
        DeclareLaunchArgument('fleet_mode', default_value='central',
                              description='central | distributed (보너스 4 분산 플릿 — 하트비트 · 토큰 끔)'),
        DeclareLaunchArgument('heartbeat_observe_s', default_value='3.0',
                              description='분산 모드 관리자 관찰 시간 [s, sim] (기동 뒤 로봇 epoch 수집)'),
        DeclareLaunchArgument('epoch_file', default_value='',
                              description="관리자 epoch 영속 파일 ('' = fleet.yaml — 영속 안 함)"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
