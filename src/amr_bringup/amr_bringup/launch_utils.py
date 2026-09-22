"""
통합 런치 구성 요소 — system.launch.py(1대)와 multi_robot.launch.py(N대)가 같은 순서로 조립한다.

    월드 1개: amr_simulation warehouse.launch.py (spawn_robot:=false, /clock 브리지 1개)
    로봇마다 description.launch.py (robot_state_publisher, 런치 시작 즉시 — xacro 가 월드 로드와 겹친다)
    world_gate hold  : 월드 로드(/world/<w>/create) 대기 → 일시 정지
      → 로봇마다 spawn.launch.py (create + 브리지) 를 한꺼번에
      → world_gate release : 모든 모델 확인 → 재개
        → 로봇마다 스택 localization / navigation / perception / behavior (설치돼 있을 때만)
    fleet_manager.launch.py / dashboard.launch.py / evaluation.launch.py (런치 시작 즉시)
    맨 앞: FASTRTPS_DEFAULT_PROFILES_FILE 이 비면 config/fastdds_multi_robot(_localhost).xml

include 는 모두 GroupAction(scoped) 으로 감싸고 인자는 문자열로 확정해 넘긴다. Humble 의
IncludeLaunchDescription 은 launch 인자를 부모 범위에 그대로 남기고, DeclareLaunchArgument 는 값이 이미
있으면 기본값을 쓰지 않으므로, 감싸지 않으면 앞 로봇의 robot_name·namespace 가 뒤 include 의 기본값을
가린다. 이벤트 처리기(스폰·스택)는 나중에 실행되므로 LaunchConfiguration 을 그때 평가하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
import os
import shutil
import sys
import tempfile
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from amr_bringup import fleet_spawn, world_gate
from amr_bringup.fleet_spawn import FleetSpawn, RobotSpec
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import yaml

# (인자, 패키지, 런치 파일, 로그 이름) — 로봇별 온보드 스택. 런치 시점에 있을 때만 포함한다.
STACKS: Tuple[Tuple[str, str, str, str], ...] = (
    ('with_localization', 'amr_localization', 'localization.launch.py', 'localization'),
    ('with_navigation', 'amr_navigation', 'navigation.launch.py', 'navigation'),
    ('with_perception', 'amr_perception', 'perception.launch.py', 'perception'),
    ('with_behavior', 'amr_behavior', 'behavior.launch.py', 'behavior'),
)
GLOBALS = ('with_fleet', 'with_dashboard', 'with_evaluation')


def truthy(text: str) -> bool:
    """Launch 인자 문자열 → bool ('true'/'false' 외에는 ValueError)."""
    value = text.strip().lower()
    if value in ('true', '1', 'yes'):
        return True
    if value in ('false', '0', 'no'):
        return False
    raise ValueError(f'true/false 여야 한다: {text!r}')


def text(value: bool) -> str:
    return 'true' if value else 'false'


def common_arguments() -> List[DeclareLaunchArgument]:
    """두 런치 파일이 같은 이름·기본값으로 선언하는 인자."""
    args = [
        DeclareLaunchArgument('robots_file', default_value=PathJoinSubstitution(
                                  [FindPackageShare('amr_bringup'), 'config', 'fleet_spawn.yaml']),
                              description='스폰 목록·공통 스폰 옵션 YAML (config/fleet_spawn.yaml)'),
        DeclareLaunchArgument('headless', default_value='true',
                              description='Gazebo 서버만 (EGL 센서 렌더링). false 면 GUI 포함'),
        DeclareLaunchArgument('world', default_value='warehouse.sdf',
                              description='amr_simulation worlds/ 의 월드 파일 '
                                          '(이름 = <world name>.sdf)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='시뮬레이션 시계 사용 여부'),
        DeclareLaunchArgument('eval_run_name', default_value='',
                              description="evaluation 런 이름 ('' = <entry>_YYYYmmdd_HHMMSS)"),
        DeclareLaunchArgument('allocation_strategy', default_value='',
                              description="fleet 할당 전략 ('' = fleet.yaml)"),
        DeclareLaunchArgument('dashboard_port', default_value='',
                              description="대시보드 HTTP 포트 ('' = dashboard.yaml)"),
        DeclareLaunchArgument('localization_mode', default_value='localization',
                              description='localization 스택 mode (localization | slam | odom). '
                                          'slam 은 매핑 — 루트 map_server·AMCL 대신 slam_toolbox'),
        # auto = amr_navigation 기본 (TTC 재계획 BT). 한때 TTC BT 가 BT tick 마다 재계획하는 결함
        # (RateController 가 ReactiveFallback 아래서 halt 됨) 때문에 false 로 우회했으나 amr_navigation 이
        # 구조를 고쳤다 (Gazebo ComputePathToPose 41 → 0.25 회/s, costmap.md). false 는 TTC 조건 없는 BT
        DeclareLaunchArgument('nav_ttc_bt', default_value='auto',
                              description='navigation.launch.py use_ttc_bt (auto | true | false)'),
        # 이름이 'map' 이면 안 된다: 포함한 런치가 부모 범위의 launch 인자를 그대로 보므로(모듈 설명)
        # localization.launch.py 의 map 기본값이 '' 로 가려져 map_server 가 지도 없이 뜬다 (실측)
        DeclareLaunchArgument('map_yaml', default_value='',
                              description="루트 map_server 지도 YAML ('' = localization.launch.py 기본 "
                                          '$ROS_WS/maps/warehouse.yaml)'),
    ]
    args += [DeclareLaunchArgument(name, default_value='true',
                                   description=f'{name[5:]} 런치 포함') for name in GLOBALS]
    args += [DeclareLaunchArgument(flag, default_value='true',
                                   description=f'로봇별 {label} 스택 ({pkg}/launch/{launch}; '
                                               '설치돼 있지 않으면 건너뜀)')
             for flag, pkg, launch, label in STACKS]
    return args


@dataclass
class Options:
    """런치 인자를 확정한 값."""

    world: str = 'warehouse.sdf'
    headless: bool = True
    use_sim_time: bool = True
    flags: Dict[str, bool] = field(default_factory=dict)
    eval_run_name: str = ''
    allocation_strategy: str = ''
    dashboard_port: str = ''
    localization_mode: str = 'localization'
    map_yaml: str = ''
    nav_ttc_bt: str = 'auto'

    @property
    def world_name(self) -> str:
        """월드 파일 이름에서 확장자를 뗀 것 = SDF <world name> (warehouse.launch.py 와 같은 규칙)."""
        return os.path.splitext(os.path.basename(self.world))[0]


def read_options(context: LaunchContext, entry: str) -> Options:
    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    flags = {name: truthy(arg(name)) for name in GLOBALS}
    flags.update({flag: truthy(arg(flag)) for flag, _, _, _ in STACKS})
    return Options(world=arg('world'), headless=truthy(arg('headless')),
                   use_sim_time=truthy(arg('use_sim_time')), flags=flags,
                   eval_run_name=arg('eval_run_name').strip()
                   or f'{entry}_{datetime.now():%Y%m%d_%H%M%S}',
                   allocation_strategy=arg('allocation_strategy').strip(),
                   dashboard_port=arg('dashboard_port').strip(),
                   localization_mode=arg('localization_mode').strip() or 'localization',
                   map_yaml=arg('map_yaml').strip(),
                   nav_ttc_bt=arg('nav_ttc_bt').strip().lower() or 'auto')


def launch_file(pkg: str, name: str) -> Tuple[Optional[str], str]:
    """(share/<pkg>/launch/<name> 경로 또는 None, 없을 때 이유). ament index 를 런치 시점에 조회한다."""
    try:
        share = get_package_share_directory(pkg)
    except PackageNotFoundError:
        return None, f'{pkg} not in ament index'
    path = os.path.join(share, 'launch', name)
    if not os.path.isfile(path):
        return None, f'{pkg} has no launch/{name}'
    return path, ''


def include(pkg: str, name: str, arguments: Dict[str, str]) -> GroupAction:
    """필수 하위 런치를 범위 격리해 포함한다. 없으면 RuntimeError."""
    path, why = launch_file(pkg, name)
    if path is None:
        raise RuntimeError(f'{pkg}/launch/{name} 를 찾을 수 없다 ({why}) — '
                           'colcon build 후 install/setup.bash 를 소싱했는지 확인')
    return include_path(path, arguments)


def include_path(path: str, arguments: Dict[str, str]) -> GroupAction:
    return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=[(k, str(v)) for k, v in arguments.items()])])


def frame_prefix(robot: RobotSpec, prefixed: bool) -> str:
    """TF 프레임 접두어: 다중 로봇은 '<이름>/', 단일 로봇은 '' (multi_robot.md §2, sensor_calibration.md)."""
    return f'{robot.name}/' if prefixed else ''


def world_actions(opts: Options) -> List[GroupAction]:
    return [include('amr_simulation', 'warehouse.launch.py', {
        'headless': text(opts.headless), 'world': opts.world,
        'use_sim_time': text(opts.use_sim_time), 'spawn_robot': 'false'})]


def description_action(robot: RobotSpec, prefix: str, opts: Options) -> GroupAction:
    return include('amr_description', 'description.launch.py', {
        'robot_name': robot.name, 'prefix': prefix, 'use_sim_time': text(opts.use_sim_time)})


def spawn_action(robot: RobotSpec, spawn: FleetSpawn, opts: Options) -> GroupAction:
    return include('amr_description', 'spawn.launch.py', {
        'robot_name': robot.name, 'world': opts.world_name,
        'x': repr(robot.x), 'y': repr(robot.y), 'z': repr(spawn.spawn_z), 'yaw': repr(robot.yaw),
        'use_sim_time': text(opts.use_sim_time)})


GROOT_BASE_PORT = 1666     # amr_behavior behavior.yaml groot.publisher_port 기본값 (server = +1)


def stack_arguments(robot: RobotSpec, prefix: str, opts: Options, index: int) -> Dict[str, str]:
    """
    스택 런치에 주는 공통 인자. 스택마다 쓰는 이름이 달라 같은 값을 여러 이름으로 준다.

    선언하지 않은 인자는 무시된다. robot_name/namespace = 네임스페이스, prefix/frame_prefix = TF 접두어
    (단일 로봇은 명시적 ''), initial_* = 스폰 자세(AMCL 초기 자세), waiting_pose = 스폰 자세(로봇마다 다른
    대기 위치), start_map_server = 첫 로봇만 true (루트 map_server 1개, multi_robot.md §4),
    groot_*_port = 로봇마다 다른 Groot ZMQ 포트 (한 호스트에서 같은 포트를 두 번 bind 할 수 없다 —
    index i 는 1666 + 2i / 1667 + 2i, 단일 로봇은 behavior.yaml 기본값과 같은 1666 / 1667).
    """
    return {
        'robot_name': robot.name, 'namespace': robot.name,
        'prefix': prefix, 'frame_prefix': prefix,
        'use_sim_time': text(opts.use_sim_time),
        'initial_x': repr(robot.x), 'initial_y': repr(robot.y), 'initial_yaw': repr(robot.yaw),
        'waiting_pose': f'{robot.x!r},{robot.y!r},{robot.yaw!r}',
        'start_map_server': text(index == 0),
        'groot_publisher_port': str(GROOT_BASE_PORT + 2 * index),
        'groot_server_port': str(GROOT_BASE_PORT + 2 * index + 1),
    }


def stack_extra_arguments(label: str, opts: Options) -> Dict[str, str]:
    """
    스택 하나에만 주는 인자 (다른 스택에 새지 않게).

    localization: mode(slam 매핑)·map. navigation: use_ttc_bt (nav_ttc_bt 인자, 기본 auto).
    """
    if label == 'navigation':
        return {'use_ttc_bt': opts.nav_ttc_bt}
    if label != 'localization':
        return {}
    extra = {'mode': opts.localization_mode}
    if opts.map_yaml:
        extra['map'] = opts.map_yaml
    return extra


def stack_actions(robots: Sequence[RobotSpec], prefixed: bool, opts: Options) -> List:
    """켜진 스택마다: 설치돼 있으면 로봇별 include, 없으면 한 줄 로그."""
    out: List = []
    names = ','.join(r.name for r in robots)
    for flag, pkg, name, label in STACKS:
        if not opts.flags.get(flag, True):
            continue
        path, why = launch_file(pkg, name)
        if path is None:
            out.append(LogInfo(msg=f'[bringup] {label} skipped: package not built yet ({why}); '
                                   f'robots {names} run without it'))
            continue
        out.append(LogInfo(msg=f'[bringup] {label}: {pkg}/launch/{name} × {len(robots)}'))
        out += [include_path(path, {**stack_arguments(r, frame_prefix(r, prefixed), opts, i),
                                    **stack_extra_arguments(label, opts)})
                for i, r in enumerate(robots)]
    return out


def fleet_params_file(spawn: FleetSpawn, robots: Sequence[RobotSpec]) -> Optional[str]:
    """공통·로봇별 comm_latency_ms 가 있으면 fleet.yaml 에 얹은 임시 파라미터 파일 경로 (없으면 None)."""
    base = os.path.join(get_package_share_directory('amr_fleet'), 'config', 'fleet.yaml')
    with open(base, encoding='utf-8') as f:
        params = fleet_spawn.fleet_params_with_latency(yaml.safe_load(f) or {}, spawn, robots)
    if params is None:
        return None
    with tempfile.NamedTemporaryFile('w', prefix='amr_fleet_params_', suffix='.yaml',
                                     delete=False, encoding='utf-8') as f:
        yaml.safe_dump(params, f, sort_keys=False, allow_unicode=True)
        return f.name


def global_actions(spawn: FleetSpawn, robots: Sequence[RobotSpec], opts: Options) -> List:
    """로봇 수와 무관한 fleet / dashboard / evaluation (evaluation 은 로봇마다, cpu_sampler 는 한 번)."""
    out: List = []
    sim = text(opts.use_sim_time)
    if opts.flags['with_fleet']:
        args = {'robot_ids': ','.join(fleet_spawn.robot_ids(robots)), 'use_sim_time': sim}
        params = fleet_params_file(spawn, robots)
        if params:
            args['params_file'] = params
            out.append(LogInfo(msg=f'[bringup] fleet comm_latency_ms 적용 파라미터: {params}'))
        if opts.allocation_strategy:
            args['allocation_strategy'] = opts.allocation_strategy
        out.append(include('amr_fleet', 'fleet_manager.launch.py', args))
    if opts.flags['with_dashboard']:
        args = {'use_sim_time': sim}
        if opts.dashboard_port:
            args['port'] = opts.dashboard_port
        out.append(include('amr_dashboard', 'dashboard.launch.py', args))
    if opts.flags['with_evaluation']:
        out += [include('amr_evaluation', 'evaluation.launch.py', {
            'namespace': r.name, 'robot_id': r.name, 'run_name': opts.eval_run_name,
            'use_sim_time': sim, 'with_cpu': text(i == 0)}) for i, r in enumerate(robots)]
    return out


def gate_process(name: str, args: Sequence[str]) -> ExecuteProcess:
    return ExecuteProcess(cmd=[sys.executable, '-m', 'amr_bringup.world_gate', *args],
                          name=name, output='screen')


def after_hold(returncode: int, robots: Sequence[RobotSpec], spawn: FleetSpawn, opts: Options,
               release: ExecuteProcess) -> List:
    """
    world_gate hold 종료 뒤 할 일: 전 로봇 스폰 + release.

    0 정상, 4 일시 정지 실패(멈추지 않은 채로라도 스폰) 일 때만 스폰한다. 3(월드 없음)이나 신호로 끊긴
    경우(런치 종료 중, 음수)는 스폰하지 않는다.
    """
    if returncode not in (world_gate.EXIT_OK, world_gate.EXIT_CONTROL):
        return [LogInfo(msg=f'[bringup] 월드 대기 종료 코드 {returncode} → 스폰하지 않는다')]
    return ([LogInfo(msg=f'[bringup] 스폰 {len(robots)}대: {", ".join(r.name for r in robots)}')]
            + [spawn_action(r, spawn, opts) for r in robots] + [release])


def after_release(returncode: int, after_spawn: Callable[[], List]) -> List:
    """world_gate release 종료 뒤 할 일: 일부 모델이 빠져도(2) 스택은 띄운다. 신호로 끊기면 생략."""
    if returncode not in (world_gate.EXIT_OK, world_gate.EXIT_MISSING, world_gate.EXIT_CONTROL):
        return [LogInfo(msg=f'[bringup] 스폰 확인 종료 코드 {returncode} → 스택 생략')]
    return after_spawn()


def gate_commands(robots: Sequence[RobotSpec], spawn: FleetSpawn,
                  opts: Options) -> Tuple[List[str], List[str]]:
    """world_gate hold / release 인자."""
    pause = spawn.pause_during_spawn
    hold = ['hold', '--world', opts.world_name] + (['--pause'] if pause else [])
    release = (['release', '--world', opts.world_name,
                '--models', ','.join(r.name for r in robots),
                '--timeout', repr(spawn.spawn_timeout_s)] + (['--unpause'] if pause else []))
    return hold, release


def spawn_sequence(robots: Sequence[RobotSpec], spawn: FleetSpawn, opts: Options,
                   after_spawn: Callable[[], List]) -> List:
    """hold(월드 로드 대기 + 일시 정지) → 전 로봇 스폰 → release(모델 확인 + 재개) → after_spawn()."""
    hold_args, release_args = gate_commands(robots, spawn, opts)
    hold = gate_process('world_gate_hold', hold_args)
    release = gate_process('world_gate_release', release_args)
    return [
        hold,
        RegisterEventHandler(OnProcessExit(
            target_action=hold,
            on_exit=lambda event, _ctx: after_hold(event.returncode, robots, spawn, opts,
                                                   release))),
        RegisterEventHandler(OnProcessExit(
            target_action=release,
            on_exit=lambda event, _ctx: after_release(event.returncode, after_spawn))),
    ]


DDS_PROFILE_VAR = 'FASTRTPS_DEFAULT_PROFILES_FILE'


SHM_MIN_BYTES = 2 * 1024 ** 3   # 16 MB SHM 세그먼트 × 참가자 ≈ 150 (5대)를 담을 /dev/shm 하한


def shm_capacity(path: str = '/dev/shm') -> int:
    """/dev/shm 전체 용량 [B] (없거나 못 읽으면 0)."""
    try:
        return shutil.disk_usage(path).total
    except OSError:
        return 0


def dds_profile_name(environ=os.environ, shm_bytes: Optional[int] = None) -> str:
    """
    프로파일 파일 이름 (config/ 아래).

    /dev/shm 가 SHM_MIN_BYTES 보다 작으면 SHM 확대 없는 변형 (docker 기본 64 MB 에서 16 MB 세그먼트가 공간을 다 써
    통신이 멎었다). 그 밖에 ROS_LOCALHOST_ONLY=1 이면 UDPv4 를 127.0.0.1 로 묶은 변형 (사용자 전송이 rmw 의
    localhost 설정을 대체한다).
    """
    if (shm_capacity() if shm_bytes is None else shm_bytes) < SHM_MIN_BYTES:
        return 'fastdds_multi_robot_noshm.xml'
    if environ.get('ROS_LOCALHOST_ONLY', '0').strip() == '1':
        return 'fastdds_multi_robot_localhost.xml'
    return 'fastdds_multi_robot.xml'


def dds_environment() -> List:
    """
    Fast DDS 기본 프로파일 (config/fastdds_multi_robot*.xml, 머리말 참고).

    1) builtin.mutation_tries 400: 5대 전체 스택은 참가자가 ≈150 개인데 Fast DDS 2.6 은 수신 포트를 100 번까지만
    바꿔 시도해 101 번째 이후 참가자가 그래프에서 빠진다. 2) SHM 세그먼트 16 MB: 카메라 영상이 기본 0.5 MB 에
    들어가지 않아 UDP 조각으로 떨어진다 (쓰는 쪽 = 이미지 브리지도 같은 프로파일이어야 한다).
    사용자가 이미 프로파일을 줬으면 건드리지 않는다.
    """
    if os.environ.get(DDS_PROFILE_VAR):
        return [LogInfo(msg=f'[bringup] {DDS_PROFILE_VAR} 유지: {os.environ[DDS_PROFILE_VAR]}')]
    try:
        path = os.path.join(get_package_share_directory('amr_bringup'), 'config',
                            dds_profile_name())
    except PackageNotFoundError:
        return []
    if not os.path.isfile(path):
        return []
    note = ('/dev/shm 가 작아 SHM 확대 없음 — ipc: host 또는 shm_size ≥ 2 GB 권장'
            if path.endswith('_noshm.xml') else '참가자 100 개 초과·영상 SHM')
    return [SetEnvironmentVariable(DDS_PROFILE_VAR, path),
            LogInfo(msg=f'[bringup] {DDS_PROFILE_VAR}={path} ({note})')]


def bringup_actions(robots: Sequence[RobotSpec], spawn: FleetSpawn, opts: Options,
                    prefixed: bool) -> List:
    """월드 + 로봇 N대 + 전역 노드 전체 (위 모듈 설명의 순서)."""
    fleet_spawn.check_separation(robots, spawn.min_separation_m)
    return (dds_environment()
            + world_actions(opts)
            + [description_action(r, frame_prefix(r, prefixed), opts) for r in robots]
            + spawn_sequence(robots, spawn, opts,
                             after_spawn=lambda: stack_actions(robots, prefixed, opts))
            + global_actions(spawn, robots, opts))
