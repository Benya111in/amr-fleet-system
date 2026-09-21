"""
물류센터 월드(Gazebo Fortress) 기동 + /clock 브리지 + (선택) 로봇 스폰.

    ros2 launch amr_simulation warehouse.launch.py                     # 헤드리스 서버 + amr_01
    ros2 launch amr_simulation warehouse.launch.py headless:=false     # GUI 포함 (DISPLAY 필요)
    ros2 launch amr_simulation warehouse.launch.py spawn_robot:=false  # 월드만 (다중 로봇: amr_fleet)

인자
    headless      true(기본): ign gazebo -s --headless-rendering (서버 전용, EGL 로 센서 렌더링).
                  false: 서버 + GUI
    world         worlds/ 안의 파일 이름 또는 절대 경로 (기본 warehouse.sdf).
                  파일 이름(확장자 제외) = <world name> 이어야 한다 (스폰 런치가 그 이름으로 create 를 부른다)
    use_sim_time  (기본 true)
    spawn_robot   true(기본)면 amr_description 의 description.launch.py + spawn.launch.py 를 포함한다
    robot_name    스폰할 로봇 이름/네임스페이스 (기본 amr_01)
    x, y, yaw     스폰 자세 [m, rad]. 기본 (0, 0, 0) = 랙 B열-C열 사이 5 m 통로 한가운데 (자유 셀)

좌표계 (worlds/gen_warehouse_world.py)
    원점 = 창고 중심, x = 60 m 축(+x 동쪽 = 출고구역), y = 40 m 축(+y 북쪽 = 도크/메인 통로), 바닥 z = 0.

리소스/플러그인 경로
    IGN_GAZEBO_RESOURCE_PATH / GZ_SIM_RESOURCE_PATH 에 이 패키지의 share/worlds, share/models 를
    덧붙인다(기존 값 유지). model://rack 같은 URI 와 worker 스킨 메시가 여기서 풀린다.
    시스템 플러그인 경로에는 LD_LIBRARY_PATH 를 덧붙인다 (ros_gz_sim gz_sim.launch.py 와 같은 규칙:
    ROS 패키지가 설치한 gz 시스템 플러그인도 찾게).

토픽
    /clock  (GZ→ROS, rosgraph_msgs/Clock) — 전역으로 한 번만 브리지한다. 로봇별 토픽은 spawn.launch.py 담당.
    Gazebo 쪽 지면 진실: /world/<world>/pose/info (모델), /model/forklift_main/pose (지게차).
    작업자 actor 는 렌더링 쪽에서만 움직여 pose/info 에 없다 → 궤적은 SDF <trajectory> 를 sim time 으로 보간.

Gazebo 기동 방식 (결정: ign 실행 파일 직접 실행, ros_gz_sim gz_sim.launch.py 미사용)
    gz_sim.launch.py 는 Gazebo 를 셸(sh -c)로 띄운다. 터미널 Ctrl-C 는 프로세스 그룹 전체에 가서 문제없지만,
    스크립트/launch_testing 이 ros2 launch 에만 SIGINT 를 보내면 sh 만 죽고 Gazebo 서버(ruby)가 고아로 남는다
    (Fortress 6.18 실측). 반복 시험(명세 4.7 회피 30회 등)에서 남은 서버가 다음 실행과 충돌하므로, ign 을
    직접 실행해 런치가 서버 프로세스에 바로 신호를 보내게 한다. 서버가 끝나면 런치 전체를 내린다 (Shutdown).
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler, SetEnvironmentVariable, Shutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_robot = LaunchConfiguration("spawn_robot")
    robot_name = LaunchConfiguration("robot_name")

    args = [
        DeclareLaunchArgument("headless", default_value="true",
                              description="서버 전용(-s --headless-rendering). false 면 GUI 포함"),
        DeclareLaunchArgument("world", default_value="warehouse.sdf",
                              description="worlds/ 안의 월드 파일 또는 절대 경로 (이름 = <world name>.sdf)"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="시뮬레이션 시계 사용 여부"),
        DeclareLaunchArgument("spawn_robot", default_value="true",
                              description="amr_description 런치로 로봇을 스폰할지"),
        DeclareLaunchArgument("robot_name", default_value="amr_01",
                              description="스폰할 로봇 이름 (네임스페이스 + Gazebo 모델 이름)"),
        DeclareLaunchArgument("x", default_value="0.0", description="스폰 x [m]"),
        DeclareLaunchArgument("y", default_value="0.0", description="스폰 y [m]"),
        DeclareLaunchArgument("yaw", default_value="0.0", description="스폰 yaw [rad]"),
    ]

    pkg_share = FindPackageShare("amr_simulation")
    worlds_dir = PathJoinSubstitution([pkg_share, "worlds"])
    models_dir = PathJoinSubstitution([pkg_share, "models"])
    # 절대 경로면 os.path.join 규칙대로 그대로 쓰인다
    world_file = PathJoinSubstitution([worlds_dir, world])
    # 파일 이름에서 확장자를 뗀 것이 <world name> (…/warehouse.sdf → warehouse)
    world_name = PythonExpression(
        ["__import__('os').path.splitext(__import__('os').path.basename('", world, "'))[0]"])

    # 리소스 경로: 변수마다 자기 기존 값 뒤에 덧붙인다 (Dockerfile 의 src 경로 등 유지).
    # Fortress 6.18 은 IGN_* 와 GZ_SIM_*(Garden 이후 이름)를 둘 다 읽는다.
    set_env = [
        SetEnvironmentVariable(var, [EnvironmentVariable(var, default_value=""),
                                     ":", worlds_dir, ":", models_dir])
        for var in ("IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH")
    ] + [
        SetEnvironmentVariable(var, [EnvironmentVariable(var, default_value=""), ":",
                                     EnvironmentVariable("LD_LIBRARY_PATH", default_value="")])
        for var in ("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH")
    ]

    # -r: 즉시 실행. headless 는 서버만(-s) + EGL 렌더링, 아니면 서버+GUI.
    # sigterm_timeout: SIGINT 후 서버가 정리(렌더링 해제 포함)할 시간. 부하가 크면 5 s 기본값으로 부족하다.
    def gazebo(extra, condition):
        return ExecuteProcess(
            cmd=["ign", "gazebo", "-r", *extra, world_file, "--force-version", "6"],
            name="gazebo", output="screen", condition=condition, sigterm_timeout="15",
            on_exit=Shutdown(reason="Gazebo 서버 종료"))

    gazebo_headless = gazebo(["-s", "--headless-rendering"], IfCondition(headless))
    gazebo_gui = gazebo([], UnlessCondition(headless))

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 로봇: URDF 로드(robot_state_publisher) → Gazebo 스폰 + 로봇별 브리지 (amr_description 계약).
    # 조건이 false 면 경로 치환을 평가하지 않으므로 amr_description 런치가 없어도 월드만 뜬다.
    # 스폰은 월드의 create 서비스가 뜬 뒤에 포함한다: ros_gz_sim create 는 서비스 요청을 5 s 만에 포기하는데
    # 이 월드 로드는 그보다 오래 걸린다 (헤드리스 17 s 실측, 부하가 크면 수 분) → 먼저 스폰하면 로봇이 없다.
    desc_launch_dir = PathJoinSubstitution([FindPackageShare("amr_description"), "launch"])
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([desc_launch_dir, "description.launch.py"])),
        condition=IfCondition(spawn_robot),
        launch_arguments={
            "robot_name": robot_name,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([desc_launch_dir, "spawn.launch.py"])),
        condition=IfCondition(spawn_robot),
        launch_arguments={
            "robot_name": robot_name,
            "world": world_name,
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "yaw": LaunchConfiguration("yaw"),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    wait_world = ExecuteProcess(
        cmd=["bash", "-c",
             ["until ign service -l 2>/dev/null | grep -qx '/world/", world_name, "/create'; "
              "do sleep 2; done"]],
        name="wait_for_world", output="log", condition=IfCondition(spawn_robot))

    def spawn_when_ready(event, _context):
        if event.returncode != 0:     # 종료 중 신호로 끊긴 경우
            return [LogInfo(msg="월드 준비 대기가 중단되어 로봇을 스폰하지 않는다")]
        return [LogInfo(msg="월드 create 서비스 확인 → 로봇 스폰"), spawn]

    spawn_after_world = RegisterEventHandler(
        OnProcessExit(target_action=wait_world, on_exit=spawn_when_ready),
        condition=IfCondition(spawn_robot))

    return LaunchDescription(
        args + set_env + [gazebo_headless, gazebo_gui, clock_bridge, description,
                          wait_world, spawn_after_world])
