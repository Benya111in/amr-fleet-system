"""
전체 시스템 통합 런치 — 단일 로봇 (명세 10장).

    ros2 launch amr_bringup system.launch.py
    ros2 launch amr_bringup system.launch.py robot_name:=amr_03 x:=0.0 y:=0.0 yaw:=0.0
    ros2 launch amr_bringup system.launch.py with_fleet:=false with_dashboard:=false \
        with_evaluation:=false

다중 로봇(5대)은 multi_robot.launch.py. 두 런치는 같은 구성 요소(amr_bringup/launch_utils.py)를 같은 순서로
쓴다: 월드 → description → 월드 일시 정지 → 스폰 → 재개 → 스택, 그리고 fleet / dashboard / evaluation.
차이는 대수와 TF 프레임 접두어뿐이다.

인자 (multi_robot.launch.py 와 같은 이름·기본값에 더해)
    robot_name    네임스페이스 + Gazebo 모델 이름 (기본 amr_01). 토픽은 단일 로봇에서도 /amr_01/... 아래
    prefix        TF 프레임 접두어 (기본 '' — 단일 로봇은 접두어 없이 base_link, lidar_link …;
                  multi_robot.md §2, sensor_calibration.md §1.2).
                  다중 로봇과 같은 프레임 이름을 쓰려면 prefix:=amr_01/
    x, y, yaw     스폰 자세 (기본 '' = robots_file 에서 robot_name 항목, 없으면 0, 0, 0 — 랙 B-C 통로 중앙)
    robots_file / headless / world / use_sim_time / with_* / eval_run_name / allocation_strategy /
    dashboard_port / localization_mode / map_yaml / nav_ttc_bt
                  multi_robot.launch.py 참고. 공통 스폰 옵션(일시 정지, 시간 상한, 지연)도 robots_file
                  에서 읽는다
    매핑 (maps/warehouse 재생성): localization_mode:=slam x:=0.0 y:=0.0 yaw:=0.0 \
        with_navigation:=false with_perception:=false with_behavior:=false — map 프레임 = 시작 자세이므로
        월드 원점·yaw 0 에서 시작해야 지도 좌표 = 월드 좌표 (behavior.yaml 도크 표·플릿 작업·스폰 초기
        자세가 모두 월드 좌표를 쓴다)
"""

from dataclasses import replace

from amr_bringup import fleet_spawn
from amr_bringup import launch_utils as lu
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _pose(context, spawn, name):
    """x/y/yaw 인자가 비어 있으면 스폰 목록의 같은 이름 로봇 자세, 그것도 없으면 0."""
    listed = fleet_spawn.find_robot(spawn, name)
    pose = []
    for key in ('x', 'y', 'yaw'):
        raw = LaunchConfiguration(key).perform(context).strip()
        pose.append(float(raw) if raw else (getattr(listed, key) if listed else 0.0))
    return pose, listed


def _setup(context):
    """단일 로봇 1대를 multi_robot 과 같은 구성 요소로 조립한다."""
    opts = lu.read_options(context, 'system')
    spawn = fleet_spawn.load_fleet_spawn(LaunchConfiguration('robots_file').perform(context))
    name = LaunchConfiguration('robot_name').perform(context).strip('/')
    (x, y, yaw), listed = _pose(context, spawn, name)
    robot = fleet_spawn.parse_robot({'name': name, 'x': x, 'y': y, 'yaw': yaw}, 0)
    if listed is not None:
        robot = replace(robot, comm_latency_ms=listed.comm_latency_ms)
    prefix = LaunchConfiguration('prefix').perform(context)
    if prefix not in ('', f'{name}/'):
        raise RuntimeError(f"prefix 는 '' 또는 '{name}/' (multi_robot.md §2): {prefix!r}")
    return ([LogInfo(msg=f'[bringup] 단일 로봇 {name} ({x:g}, {y:g}, {yaw:g}), '
                         f'프레임 접두어 {prefix!r}')]
            + lu.bringup_actions([robot], spawn, opts, prefixed=bool(prefix)))


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [DeclareLaunchArgument('robot_name', default_value='amr_01',
                               description='로봇 네임스페이스 및 Gazebo 모델 이름'),
         DeclareLaunchArgument('prefix', default_value='',
                               description="TF 프레임 접두어 ('' = 단일 로봇 규약, '<robot_name>/' 허용)"),
         DeclareLaunchArgument('x', default_value='', description="스폰 x [m] ('' = robots_file)"),
         DeclareLaunchArgument('y', default_value='', description="스폰 y [m] ('' = robots_file)"),
         DeclareLaunchArgument('yaw', default_value='',
                               description="스폰 yaw [rad] ('' = robots_file)")]
        + lu.common_arguments()
        + [OpaqueFunction(function=_setup)])
