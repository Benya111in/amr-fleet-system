"""
Gazebo Fortress 에 로봇 스폰(확인 포함) + 로봇별 ros_gz 브리지.

description.launch.py 가 먼저 떠서 /<robot_name>/robot_description 을 발행하고 있어야 한다.
보통은 amr_simulation/launch/warehouse.launch.py 가 두 런치를 함께 포함한다.

    ros2 launch amr_description spawn.launch.py x:=0.0 y:=0.0 yaw:=0.0
    # 두 번째 로봇 (description.launch.py 도 robot_name:=amr_02 prefix:=amr_02/ 로 별도 기동)
    ros2 launch amr_description spawn.launch.py robot_name:=amr_02 x:=2.0

인자
    robot_name    네임스페이스 + Gazebo 모델 이름 (기본 amr_01)
    world         스폰 대상 월드 이름 (기본 warehouse; SDF <world name=...>)
    x, y, z, yaw  스폰 자세 [m, rad]. z 기본 0.02: 바퀴가 지면에 살짝 떠서 내려앉도록 (초기 관통 방지)
    use_sim_time  (기본 true) — 브리지 노드에 전달
    config_dir    sensors.yaml 디렉토리 (기본 $ROS_WS/config, ROS_WS 미설정 시 /ros2_ws/config).
                  브리지 토픽 목록을 sensors.yaml 의 topic 키에서 만든다 (xacro 와 같은 단일 진실 공급원)
    bridge_config 직접 작성한 parameter_bridge YAML 로 자동 생성을 대체할 때 (기본 "" = 자동 생성)
    bridge_odom_tf
                  true 면 DiffDrive 의 odom→base_footprint TF(gz odom_gz/tf)를 /tf 로 브리지한다 (기본 false).
                  EKF(ekf_filter_node_odom)가 생기기 전 초기 매핑(slam_toolbox)용 임시 경로 — EKF 와 함께 켜면
                  같은 변환의 발행자가 둘이 되어 TF 가 튄다. 이 TF 는 조인트 적분값이라 슬립·양자화 노이즈가 없다
    spawn_timeout create 서비스(월드 로드) 대기 한도 [s] (기본 900)

스폰 (결정: ros_gz_sim create 대신 scripts/gz_world.py spawn)
    create 서비스를 기다리고, 같은 이름이 없는지 확인한 뒤 생성하고, scene/info 에서 모델이 실제로 생겼는지 확인한다.
    어느 단계든 실패하면 ERROR 와 함께 런치 전체를 내린다 (ros_gz_sim create 0.244.26 은 응답 5 s 시간 초과에도
    "OK creation"·종료 코드 0 으로 끝나 고부하에서 스폰 실패가 조용히 지나갔다 — 확인).

토픽 (ROS 는 모두 /<robot_name>/ 아래 상대 이름, gz 는 /<robot_name>/<같은 이름>)
    GZ→ROS  joint_states, scan, imu/data_raw, camera/image_raw, camera/camera_info,
            camera/depth/image_raw, camera/depth/camera_info,
            camera/depth/points (gz: camera/depth/image_raw/points, lazy — 구독자가 있을 때만 브리지·생성),
            odom_gz (DiffDrive 자체 오도메트리, 비교용), ground_truth/odom (평가용, frame_id world)
            (bridge_odom_tf:=true 일 때만) /tf ← gz odom_gz/tf
    ROS→GZ  cmd_vel
    QoS: parameter_bridge 의 발행자는 reliable (depth 10), image_bridge 는 image_transport 기본(reliable).
         구독 측은 best-effort(sensor data)로 받아도 된다.
    브리지하지 않음(기본): DiffDrive/OdometryPublisher 의 tf — odom→base_footprint 는 EKF 가 발행

브리지 구성 (결정)
    - parameter_bridge 는 config_file(생성한 YAML, 상대 이름) + expand_gz_topic_names=true 로 네임스페이스
      /<robot_name> 에서 동작한다 (ros_gz_bridge 0.244.26 에서 확인). 이름은 sensors.yaml 에서 읽으므로 토픽을
      바꾸면 xacro(gz 센서 <topic>)와 브리지가 함께 따라온다. 생성한 임시 YAML 은 런치 종료 시 지운다.
    - 이미지 두 토픽은 ros_gz_image image_bridge 로 보낸다: image_transport 를 거치므로 compressed /
      compressedDepth / theora 파생 토픽이 함께 생겨 웹 대시보드·rosbag 대역폭에 유리하고, gz 토픽 이름을
      ROS 이름과 같게 잡아 두어 재매핑이 필요 없다. camera_info 는 parameter_bridge 가 넘긴다.
    - 640x480 점군은 메시지당 3.6 MB(15 Hz → 55 MB/s)라 lazy 로 둔다: ROS 구독자가 없으면 브리지가 gz
      구독을 열지 않고, 그러면 gz-sensors 깊이 카메라도 points 메시지를 채우지 않는다 (소스 기준).

월드 요건 (amr_simulation 담당) — 아래 시스템이 월드 SDF 에 있어야 한다
    ignition-gazebo-physics-system, ignition-gazebo-user-commands-system (create 서비스),
    ignition-gazebo-scene-broadcaster-system (scene/info — 스폰 확인에 쓴다),
    ignition-gazebo-sensors-system (<render_engine>ogre2</render_engine>; 헤드리스는 EGL),
    ignition-gazebo-imu-system.
    /clock 브리지는 월드 런치가 전역으로 한 번만 띄운다 (여기서 띄우지 않는다).

다중 로봇 (기동 순서 — docs/architecture/multi_robot.md §5)
    로봇마다 description.launch.py(robot_name, prefix="<robot_name>/") + 이 런치(robot_name, x, y)를
    한 쌍씩 기동한다. 토픽은 네임스페이스로, TF 프레임/관절 이름은 prefix 로 분리되어 충돌하지 않는다
    (docs/architecture/multi_robot.md §2). Gazebo 모델 이름 = robot_name 이라 같은 이름으로 두 번 스폰할 수 없다.
    Fortress(ign-sensors6) 센서의 다음 갱신 시각은 sim 0 에서 시작해 한 주기씩만 늘어나므로, sim 시간 T 에 스폰된
    로봇의 센서는 T × update_rate 번을 매 물리 스텝마다 갱신하며 따라잡는다 (실측: T ≈ 92 s 에 스폰한 amr_02 의
    scan/imu/camera 가 ~1 kHz(sim)로 발행, 월드 RTF 0.12 → 0.004). 그래서
      1) 월드를 일시정지로 띄운다 (warehouse.launch.py paused:=true — 로봇을 함께 스폰하면 기본값)
      2) 모든 로봇을 이 런치로 스폰한다 (각자 생성 확인)
      3) 전원 확인 후 일시정지를 푼다 (amr_simulation unpause.launch.py robots:=amr_01,amr_02,...)
    이 순서면 모든 센서가 sim 0 에서 시작해 첫 메시지부터 정상 주기다. 운용 중(sim T > 0) 재스폰은 여전히 따라잡기를
    일으키므로 피한다.

Fortress 점군 주의
    camera/depth/points 의 header.frame_id 는 camera_depth_optical_frame 이지만 데이터는 센서 본체 규약
    (x 전방, y 좌, z 상)이다. 소비 노드에서 frame_id 를 camera_link 로 바꿔 해석하거나 depth 이미지 +
    camera_info 에서 점군을 직접 만들 것 (urdf/amr_gazebo.xacro, config/sensors.yaml 주석 참조).
"""

import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_config_dir() -> str:
    """최상위 config 디렉토리 기본값: $ROS_WS/config, 없으면 컨테이너 규약 경로."""
    ros_ws = os.environ.get("ROS_WS", "")
    return os.path.join(ros_ws, "config") if ros_ws else "/ros2_ws/config"


def _gz_camera_info_topic(image_topic: str) -> str:
    """gz-sensors 규칙: 이미지 <topic> 의 디렉터리 + /camera_info (Fortress 는 별도 지정 불가)."""
    head, sep, _ = image_topic.rpartition("/")
    return head + sep + "camera_info"


def _bridge_entries(sensors: dict, bridge_odom_tf: bool = False) -> list:
    """parameter_bridge YAML 항목. 상대 이름 → 브리지 노드의 네임스페이스로 확장된다."""
    lidar, rgb, depth, imu = (sensors["lidar"], sensors["rgb_camera"],
                              sensors["depth_camera"], sensors["imu"])

    def gz_to_ros(ros, gz, ros_type, gz_type, lazy=False):
        return {"ros_topic_name": ros, "gz_topic_name": gz, "ros_type_name": ros_type,
                "gz_type_name": gz_type, "direction": "GZ_TO_ROS", "lazy": lazy}

    entries = [
        gz_to_ros("joint_states", "joint_states",
                  "sensor_msgs/msg/JointState", "ignition.msgs.Model"),
        gz_to_ros(lidar["topic"], lidar["topic"],
                  "sensor_msgs/msg/LaserScan", "ignition.msgs.LaserScan"),
        gz_to_ros(imu["topic"], imu["topic"],
                  "sensor_msgs/msg/Imu", "ignition.msgs.IMU"),
        gz_to_ros(rgb["info_topic"], _gz_camera_info_topic(rgb["topic"]),
                  "sensor_msgs/msg/CameraInfo", "ignition.msgs.CameraInfo"),
        gz_to_ros(depth["info_topic"], _gz_camera_info_topic(depth["topic"]),
                  "sensor_msgs/msg/CameraInfo", "ignition.msgs.CameraInfo"),
        gz_to_ros(depth["points_topic"], depth["topic"] + "/points",
                  "sensor_msgs/msg/PointCloud2", "ignition.msgs.PointCloudPacked", lazy=True),
        {"ros_topic_name": "cmd_vel", "gz_topic_name": "cmd_vel",
         "ros_type_name": "geometry_msgs/msg/Twist", "gz_type_name": "ignition.msgs.Twist",
         "direction": "ROS_TO_GZ"},
        gz_to_ros("odom_gz", "odom_gz", "nav_msgs/msg/Odometry", "ignition.msgs.Odometry"),
        gz_to_ros("ground_truth/odom", "ground_truth/odom",
                  "nav_msgs/msg/Odometry", "ignition.msgs.Odometry"),
    ]
    if bridge_odom_tf:
        # 초기 매핑용 임시 TF (인자 설명 참조). ROS 이름은 절대 이름 /tf (네임스페이스를 붙이지 않는다)
        entries.append(gz_to_ros("/tf", "odom_gz/tf", "tf2_msgs/msg/TFMessage",
                                 "ignition.msgs.Pose_V"))
    return entries


def _setup(context, *args, **kwargs):
    """런치 시점에 sensors.yaml 을 읽어 브리지 설정을 만들고 노드를 구성한다."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    robot_name = arg("robot_name")
    world = arg("world")
    use_sim_time = arg("use_sim_time").lower() in ("true", "1")
    config_dir = arg("config_dir")
    bridge_config = arg("bridge_config")
    bridge_odom_tf = arg("bridge_odom_tf").lower() in ("true", "1")

    sensors_yaml = os.path.join(config_dir, "sensors.yaml")
    if not os.path.isfile(sensors_yaml):
        raise RuntimeError(
            f"sensors.yaml 이 없다: {sensors_yaml} (config_dir:= 로 최상위 config 디렉토리를 지정)")
    with open(sensors_yaml, encoding="utf-8") as f:
        sensors = yaml.safe_load(f)["/**"]["ros__parameters"]

    actions = []
    if not bridge_config:
        with tempfile.NamedTemporaryFile(
                "w", prefix=f"amr_bridge_{robot_name}_", suffix=".yaml",
                delete=False, encoding="utf-8") as f:
            yaml.safe_dump(_bridge_entries(sensors, bridge_odom_tf), f, sort_keys=False)
            generated = f.name
        bridge_config = generated

        def _cleanup(_event, _context):
            try:
                os.remove(generated)
            except OSError:
                pass

        actions += [
            LogInfo(msg=f"[{robot_name}] bridge config generated: {generated}"),
            RegisterEventHandler(OnShutdown(on_shutdown=_cleanup)),
        ]
    if bridge_odom_tf:
        actions.append(LogInfo(msg=f"[{robot_name}] bridge_odom_tf: DiffDrive odom→base_footprint "
                                   "TF 를 /tf 로 브리지 (EKF 와 동시 사용 금지)"))

    # 모델 스폰 + 생성 확인 (scripts/gz_world.py). robot_description 토픽에서 URDF 를 읽는다
    spawn = Node(
        package="amr_description",
        executable="gz_world.py",
        name="spawn",
        namespace=robot_name,
        output="screen",
        arguments=[
            "spawn", "--world", world, "--name", robot_name,
            "--topic", f"/{robot_name}/robot_description",
            f"--x={arg('x')}", f"--y={arg('y')}", f"--z={arg('z')}", f"--yaw={arg('yaw')}",
            f"--world-timeout={arg('spawn_timeout')}",
        ],
    )

    def _on_spawn_exit(event, _context):
        if event.returncode == 0:
            return [LogInfo(msg=f"[{robot_name}] 스폰 확인 완료")]
        return [
            LogInfo(msg=f"[{robot_name}] ERROR: 스폰 실패 (gz_world.py 종료 코드 {event.returncode})"
                        " → 런치 종료"),
            EmitEvent(event=Shutdown(reason=f"{robot_name} 스폰 실패")),
        ]

    spawn_check = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=_on_spawn_exit))

    # 센서/제어 토픽 브리지 (이미지 제외)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        namespace=robot_name,
        output="screen",
        parameters=[{
            "config_file": bridge_config,
            "expand_gz_topic_names": True,
            "use_sim_time": use_sim_time,
        }],
    )

    # 이미지 브리지 (image_transport). gz 토픽 이름이 곧 ROS 토픽 이름 (절대 이름)
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="gz_image_bridge",
        namespace=robot_name,
        output="screen",
        arguments=[
            f"/{robot_name}/{sensors['rgb_camera']['topic']}",
            f"/{robot_name}/{sensors['depth_camera']['topic']}",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return actions + [spawn, spawn_check, bridge, image_bridge]


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("robot_name", default_value="amr_01",
                              description="로봇 네임스페이스 및 Gazebo 모델 이름"),
        DeclareLaunchArgument("world", default_value="warehouse",
                              description="스폰할 Gazebo 월드 이름"),
        DeclareLaunchArgument("x", default_value="0.0", description="스폰 x [m]"),
        DeclareLaunchArgument("y", default_value="0.0", description="스폰 y [m]"),
        DeclareLaunchArgument("z", default_value="0.02", description="스폰 z [m]"),
        DeclareLaunchArgument("yaw", default_value="0.0", description="스폰 yaw [rad]"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="시뮬레이션 시계 사용 여부"),
        DeclareLaunchArgument("config_dir", default_value=_default_config_dir(),
                              description="sensors.yaml 디렉토리 (브리지 토픽 이름의 출처)"),
        DeclareLaunchArgument("bridge_config", default_value="",
                              description="parameter_bridge YAML 직접 지정 (비우면 sensors.yaml 에서 생성)"),
        DeclareLaunchArgument("bridge_odom_tf", default_value="false",
                              description="DiffDrive odom TF 를 /tf 로 브리지 (EKF 이전 초기 매핑용)"),
        DeclareLaunchArgument("spawn_timeout", default_value="900",
                              description="월드 create 서비스 대기 한도 [s]"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
