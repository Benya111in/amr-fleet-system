"""
다중 로봇 통합 런치 (명세 4.9 "최소 5대 동시 운용 / 독립 네임스페이스 / TF 충돌 없음 / 통신 지연").

    ros2 launch amr_bringup multi_robot.launch.py                        # 5대, 헤드리스, 전체
    ros2 launch amr_bringup multi_robot.launch.py num_robots:=3 headless:=false
    ros2 launch amr_bringup multi_robot.launch.py with_navigation:=false with_evaluation:=false

인자
    robots_file   스폰 목록 + 공통 스폰 옵션 (기본 share/amr_bringup/config/fleet_spawn.yaml)
    num_robots    대수 (기본 5). 목록 앞에서부터 쓴다 (1 ~ 목록 길이)
    headless / world / use_sim_time            월드 런치(amr_simulation warehouse.launch.py)로 전달
    with_fleet / with_dashboard / with_evaluation
                  (기본 true) fleet_manager.launch.py (robot_ids = 스폰한 로봇), dashboard.launch.py,
                  evaluation.launch.py (로봇마다, cpu_sampler 는 첫 로봇에서 한 번)
    with_localization / with_navigation / with_perception / with_behavior
                  (기본 true) 로봇별 스택. 패키지와 launch/<스택>.launch.py 가 런치 시점에 설치돼 있을 때만
                  포함하고, 없으면 "skipped: package not built yet" 을 남기고 넘어간다
    eval_run_name / allocation_strategy / dashboard_port   ('' = 각 패키지 기본값)
    localization_mode  localization(기본) | slam | odom — localization 스택 mode
                  (slam = 매핑: map_server·AMCL 대신 slam_toolbox)
    map_yaml      루트 map_server 지도 YAML ('' = $ROS_WS/maps/warehouse.yaml, map 프레임 = 월드 좌표)
    nav_ttc_bt    navigation use_ttc_bt (기본 auto = TTC 재계획 BT, false = TTC 조건 없는 BT)
    stack_stagger_s  로봇 i 스택 시작 지연 = i × 이 값 [s] (기본 6, 0 = 동시). 5대를 한꺼번에 띄우면
                  Nav2 lifecycle_manager 가 get_state 응답을 못 받아 스택을 내린다 (launch_utils 주석)
    fleet_mode    central(기본) | distributed — 보너스 4 분산 플릿 (fleet-distributed/p3.md §10).
                  이 인자 하나로 켠다: fleet_manager.launch.py 가 하트비트 · 관찰자 기동 · 구역 토큰 끔,
                  behavior.launch.py 가 로봇마다 fleet_agent + 실행기 존 노드 · 분산 트리 · assign_task 리매핑.
                  두 하위 런치는 이 값을 launch 인자 상속으로 받는다 (launch_utils 가 인자를 따로 넘기지 않는다).
                  정족수 멤버 = 스폰한 로봇 (fleet_members 로 넘긴다). 기본 central 은 예전과 같다.
Groot ZMQ 포트는 로봇 i(0부터)마다 1666 + 2i / 1667 + 2i (task_executor_node, 같은 호스트에서 포트 충돌 방지).

로봇 이름 = 네임스페이스 = Gazebo 모델 이름 (amr_01 …), TF 프레임 접두어 '<이름>/' — URDF 링크와 Gazebo 센서
frame_id 모두 (description.launch.py prefix). /tf, /tf_static, /clock, /map, /fleet/* 만 전역
(docs/architecture/multi_robot.md). 기동 순서와 스폰 동안 일시 정지는 amr_bringup/launch_utils.py,
world_gate.py 참고. 로봇별 통신 지연은 fleet_spawn.yaml 의 comm_latency_ms (fleet_spawn.py).
"""

from amr_bringup import fleet_spawn
from amr_bringup import launch_utils as lu
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration

FLEET_MODES = ('central', 'distributed')


def _setup(context):
    """런치 시점에 스폰 목록을 읽고 로봇 N대를 조립한다."""
    opts = lu.read_options(context, 'multi')
    spawn = fleet_spawn.load_fleet_spawn(LaunchConfiguration('robots_file').perform(context))
    num = LaunchConfiguration('num_robots').perform(context)
    try:
        robots = fleet_spawn.select_robots(spawn, int(num))
    except ValueError as exc:
        raise RuntimeError(f'num_robots:={num}: {exc}') from exc
    summary = ', '.join(f'{r.name}({r.x:g}, {r.y:g}, {r.yaw:g})' for r in robots)
    mode = LaunchConfiguration('fleet_mode').perform(context).strip().lower() or 'central'
    if mode not in FLEET_MODES:
        raise RuntimeError(f'fleet_mode:={mode}: {" | ".join(FLEET_MODES)} 중 하나')
    fleet = []
    if mode == 'distributed':
        if len(robots) < 3:
            raise RuntimeError(f'fleet_mode:=distributed 는 정족수 투표라 3 대 이상: {len(robots)}')
        fleet = [SetLaunchConfiguration('fleet_members', ','.join(r.name for r in robots)),
                 LogInfo(msg=f'[bringup] 분산 플릿 모드: 멤버 {len(robots)} 대 (정족수 '
                             f'{len(robots) // 2 + 1})')]
    return (fleet + [LogInfo(msg=f'[bringup] {len(robots)}대 (프레임 접두어 <이름>/): {summary}')]
            + lu.bringup_actions(robots, spawn, opts, prefixed=True))


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [DeclareLaunchArgument('num_robots', default_value='5',
                               description='동시 운용 대수 (robots_file 앞에서부터)'),
         DeclareLaunchArgument('fleet_mode', default_value='central',
                               description='central | distributed (보너스 4 분산 플릿)')]
        + lu.common_arguments()
        + [OpaqueFunction(function=_setup)])
