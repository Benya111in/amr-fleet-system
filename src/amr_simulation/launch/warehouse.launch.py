"""
물류센터 월드(Gazebo Fortress) 기동 + /clock 브리지 + (선택) 로봇 스폰 + 동적 장애물 지면 진실·충돌 판정.

    # 헤드리스 서버 + amr_01 (일시정지 기동 → 스폰 확인 → 해제)
    ros2 launch amr_simulation warehouse.launch.py
    ros2 launch amr_simulation warehouse.launch.py headless:=false     # GUI 포함 (DISPLAY 필요)
    ros2 launch amr_simulation warehouse.launch.py payload:=large      # 대형 박스(25 kg) 적재 상태로 스폰
    # 월드만 (다중 로봇: 아래 "일시정지 기동" 순서)
    ros2 launch amr_simulation warehouse.launch.py spawn_robot:=false paused:=true

인자
    headless      true(기본): ign gazebo -s --headless-rendering (서버 전용, EGL 로 센서 렌더링).
                  false: 서버 + GUI
    world         worlds/ 안의 파일 이름 또는 절대 경로 (기본 warehouse.sdf).
                  파일 이름(확장자 제외) = <world name> 이어야 한다 (스폰이 그 이름으로 create 를 부른다)
    paused        true 면 월드를 일시정지(sim 0)로 띄운다. 기본값 = spawn_robot (로봇을 스폰하면 true).
                  로봇을 스폰했으면 생성 확인 뒤 이 런치가 풀고(unpause), spawn_robot:=false 면 부르는 쪽이 푼다
    use_sim_time  (기본 true)
    spawn_robot   true(기본)면 amr_description 의 description.launch.py + spawn.launch.py 를 포함한다
    robot_name    스폰할 로봇 이름/네임스페이스 (기본 amr_01)
    prefix        TF 프레임 접두어 (기본 "" — 다중 로봇이면 "<robot_name>/")
    x, y, z, yaw  스폰 자세 [m, rad]. 기본 (0, 0, 0.02, 0) = 랙 B열-C열 사이 5 m 통로 한가운데 (자유 셀)
    payload, payload_mass
                  스폰 시점 적재물 (amr_description description.launch.py 로 전달, 기본 없음).
                  주행 중 적재/하역은 spawn.launch.py 가 로봇마다 띄우는 payload_manager_node 가 한다
                  (payload/attach·payload/mass → 화물 모델 + DetachableJoint,
                  amr_simulation/payload.py)
    bridge_odom_tf
                  DiffDrive odom TF 를 /tf 로 브리지 (EKF 이전 초기 매핑용, 기본 false — spawn.launch.py)
    obstacle_truth
                  true(기본)면 obstacle_truth_node(/sim/dynamic_obstacles) + 지게차·셔틀 오도메트리 브리지를 띄운다
    collision_monitor
                  true(기본)면 collision_monitor_node 를 띄운다 (명세 4.7 충돌 횟수)
    monitor_robots
                  충돌 판정 대상 로봇 (쉼표 구분, 기본 = robot_name. 다중 로봇이면 amr_01,amr_02,...)

좌표계 (worlds/gen_warehouse_world.py)
    원점 = 창고 중심, x = 60 m 축(+x 동쪽 = 출고구역), y = 40 m 축(+y 북쪽 = 도크/메인 통로), 바닥 z = 0.

리소스/플러그인 경로
    IGN_GAZEBO_RESOURCE_PATH / GZ_SIM_RESOURCE_PATH 에 이 패키지의 share/worlds, share/models 를
    덧붙인다(기존 값 유지). model://rack 같은 URI 와 worker 스킨 메시·표지판 텍스처가 여기서 풀린다.
    시스템 플러그인 경로에는 LD_LIBRARY_PATH 를 덧붙인다 (ros_gz_sim gz_sim.launch.py 와 같은 규칙).

토픽
    /clock  (GZ→ROS, rosgraph_msgs/Clock) — 전역으로 한 번만 브리지한다. 로봇별 토픽은 spawn.launch.py 담당.
    /sim/forklift_main/odom, /sim/shuttle_amr/odom
            (GZ→ROS, nav_msgs/Odometry, config/obstacle_bridge.yaml)
    /sim/dynamic_obstacles, /sim/dynamic_obstacles/tracks, /sim/dynamic_obstacles/info
            (obstacle_truth_node)
    /<robot>/collision_monitor/*, /sim/collision_monitor/*  (collision_monitor_node)

일시정지 기동 (결정)
    Fortress 센서의 다음 갱신 시각은 sim 0 에서 시작해 한 주기씩만 늘어나므로, sim 시간 T 에 스폰된 로봇은 T × 주기 번을
    매 물리 스텝마다 몰아 갱신한다 (실측: T ≈ 92 s 스폰 → 센서 ~1 kHz 폭주, RTF 0.12 → 0.004). 그래서 월드를 sim 0 에
    멈춘 채 띄우고 → 로봇을 스폰·확인하고(amr_description scripts/gz_world.py spawn) → 모두 확인되면 푼다
    (gz_world.py unpause, 이 패키지 unpause.launch.py). 첫 scan 부터 10 Hz 다 (실측).
    다중 로봇(amr_bringup multi_robot.launch.py 등) 순서:
      1) 이 런치: spawn_robot:=false paused:=true monitor_robots:=amr_01,...,amr_05
      2) 로봇마다 amr_description description.launch.py(robot_name, prefix=<robot_name>/)
         + spawn.launch.py(robot_name, x, y, yaw)
      3) unpause.launch.py robots:=amr_01,...,amr_05  — 모든 모델이 월드에 생긴 것을 확인한 뒤 일시정지를 푼다

Gazebo 기동 방식 (결정: ign 실행 파일 직접 실행, ros_gz_sim gz_sim.launch.py 미사용)
    gz_sim.launch.py 는 Gazebo 를 셸(sh -c)로 띄운다. 터미널 Ctrl-C 는 프로세스 그룹 전체에 가서 문제없지만,
    스크립트/launch_testing 이 ros2 launch 에만 SIGINT 를 보내면 sh 만 죽고 Gazebo 서버(ruby)가 고아로 남는다
    (Fortress 6.18 실측). 반복 시험(명세 4.7 회피 30회 등)에서 남은 서버가 다음 실행과 충돌하므로, ign 을
    직접 실행해 런치가 서버 프로세스에 바로 신호를 보내게 한다. 서버가 끝나면 런치 전체를 내린다 (Shutdown).
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_TRUE = ("true", "1", "yes")


def _setup(context, *args, **kwargs):
    """인자 값에 따라 Gazebo(일시정지 여부)·스폰·해제·지면 진실 노드를 구성한다."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    pkg_share = FindPackageShare("amr_simulation").perform(context)
    world_arg = arg("world")
    world_file = (world_arg if os.path.isabs(world_arg)
                  else os.path.join(pkg_share, "worlds", world_arg))
    world_name = os.path.splitext(os.path.basename(world_file))[0]
    headless = arg("headless").lower() in _TRUE
    spawn_robot = arg("spawn_robot").lower() in _TRUE
    paused = arg("paused").lower() in _TRUE
    use_sim_time = arg("use_sim_time")
    robot_name = arg("robot_name")

    # -r: 즉시 실행 (paused 면 빼서 sim 0 에 멈춘 채 뜬다). headless 는 서버만(-s) + EGL 렌더링.
    # sigterm_timeout: SIGINT 후 서버가 정리(렌더링 해제 포함)할 시간. 부하가 크면 5 s 기본값으로 부족하다.
    cmd = ["ign", "gazebo"] + ([] if paused else ["-r"]) \
        + (["-s", "--headless-rendering"] if headless else []) \
        + [world_file, "--force-version", "6"]
    actions = [ExecuteProcess(cmd=cmd, name="gazebo", output="screen", sigterm_timeout="15",
                              on_exit=Shutdown(reason="Gazebo 서버 종료"))]

    actions.append(Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        parameters=[{"use_sim_time": use_sim_time in _TRUE}]))

    if spawn_robot:
        # 로봇: URDF 로드(robot_state_publisher) → 스폰(create 서비스 대기 + 생성 확인, 실패 시 런치 종료) + 브리지
        desc_dir = os.path.join(FindPackageShare("amr_description").perform(context), "launch")
        desc_args = {"robot_name": robot_name, "use_sim_time": use_sim_time,
                     "payload": arg("payload"), "payload_mass": arg("payload_mass")}
        # 빈 값은 ros2 launch 인자로 넘길 수 없다 (description.launch.py 주석)
        if arg("prefix"):
            desc_args["prefix"] = arg("prefix")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc_dir, "description.launch.py")),
            launch_arguments=desc_args.items()))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(desc_dir, "spawn.launch.py")),
            launch_arguments={"robot_name": robot_name, "world": world_name,
                              "x": arg("x"), "y": arg("y"), "z": arg("z"), "yaw": arg("yaw"),
                              "use_sim_time": use_sim_time,
                              "bridge_odom_tf": arg("bridge_odom_tf")}.items()))
        if paused:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "unpause.launch.py")),
                launch_arguments={"world": world_name, "robots": robot_name}.items()))

    params = os.path.join(pkg_share, "config", "dynamic_obstacles.yaml")
    sim_time = {"use_sim_time": use_sim_time in _TRUE, "world_file": world_file}
    if arg("obstacle_truth").lower() in _TRUE:
        actions.append(Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="obstacle_bridge",
            output="screen",
            parameters=[{"config_file": os.path.join(pkg_share, "config", "obstacle_bridge.yaml"),
                         "use_sim_time": use_sim_time in _TRUE}]))
        actions.append(Node(
            package="amr_simulation", executable="obstacle_truth_node.py",
            name="obstacle_truth_node", output="screen", parameters=[params, sim_time]))
    if arg("collision_monitor").lower() in _TRUE:
        robots = [r for r in (arg("monitor_robots") or robot_name).split(",") if r]
        actions.append(Node(
            package="amr_simulation", executable="collision_monitor_node.py",
            name="collision_monitor_node", output="screen",
            parameters=[params, sim_time, {"robots": robots}]))
    return actions


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("headless", default_value="true",
                              description="서버 전용(-s --headless-rendering). false 면 GUI 포함"),
        DeclareLaunchArgument("world", default_value="warehouse.sdf",
                              description="worlds/ 안의 월드 파일 또는 절대 경로 (이름 = <world name>.sdf)"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="시뮬레이션 시계 사용 여부"),
        DeclareLaunchArgument("spawn_robot", default_value="true",
                              description="amr_description 런치로 로봇을 스폰할지"),
        DeclareLaunchArgument("paused", default_value=LaunchConfiguration("spawn_robot"),
                              description="일시정지(sim 0)로 기동 (기본 = spawn_robot). 스폰 확인 후 해제"),
        DeclareLaunchArgument("robot_name", default_value="amr_01",
                              description="스폰할 로봇 이름 (네임스페이스 + Gazebo 모델 이름)"),
        DeclareLaunchArgument("prefix", default_value="",
                              description="TF 프레임 접두어 (다중 로봇: '<robot_name>/')"),
        DeclareLaunchArgument("x", default_value="0.0", description="스폰 x [m]"),
        DeclareLaunchArgument("y", default_value="0.0", description="스폰 y [m]"),
        DeclareLaunchArgument("z", default_value="0.02", description="스폰 z [m]"),
        DeclareLaunchArgument("yaw", default_value="0.0", description="스폰 yaw [rad]"),
        DeclareLaunchArgument("payload", default_value="none",
                              description="적재물 종류 none|small|medium|large"),
        DeclareLaunchArgument("payload_mass", default_value="-1",
                              description="적재 질량 [kg] 덮어쓰기 (-1 = 종류의 질량)"),
        DeclareLaunchArgument("bridge_odom_tf", default_value="false",
                              description="DiffDrive odom TF 를 /tf 로 브리지 (EKF 이전 초기 매핑용)"),
        DeclareLaunchArgument("obstacle_truth", default_value="true",
                              description="동적 장애물 지면 진실 노드 + 지게차·셔틀 오도메트리 브리지"),
        DeclareLaunchArgument("collision_monitor", default_value="true",
                              description="로봇 ↔ 동적 장애물 최소 거리·접촉 횟수 노드"),
        DeclareLaunchArgument("monitor_robots", default_value="",
                              description="충돌 판정 대상 로봇 (쉼표 구분, 비우면 robot_name)"),
    ]

    # 리소스 경로: 변수마다 자기 기존 값 뒤에 덧붙인다 (Dockerfile 의 src 경로 등 유지).
    # Fortress 6.18 은 IGN_* 와 GZ_SIM_*(Garden 이후 이름)를 둘 다 읽는다.
    pkg_share = FindPackageShare("amr_simulation")
    worlds_dir = PathJoinSubstitution([pkg_share, "worlds"])
    models_dir = PathJoinSubstitution([pkg_share, "models"])
    set_env = [
        SetEnvironmentVariable(var, [EnvironmentVariable(var, default_value=""),
                                     ":", worlds_dir, ":", models_dir])
        for var in ("IGN_GAZEBO_RESOURCE_PATH", "GZ_SIM_RESOURCE_PATH")
    ] + [
        SetEnvironmentVariable(var, [EnvironmentVariable(var, default_value=""), ":",
                                     EnvironmentVariable("LD_LIBRARY_PATH", default_value="")])
        for var in ("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "GZ_SIM_SYSTEM_PLUGIN_PATH")
    ]
    return LaunchDescription(args + set_env + [OpaqueFunction(function=_setup)])
