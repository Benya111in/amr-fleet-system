"""
Gazebo Fortress 에 로봇 스폰 + 로봇별 ros_gz 브리지.

description.launch.py 가 먼저 떠서 /<robot_name>/robot_description 을 발행하고 있어야 한다.
보통은 amr_simulation/launch/warehouse.launch.py 가 두 런치를 함께 포함한다.

    ros2 launch amr_description spawn.launch.py x:=0.0 y:=0.0 yaw:=0.0
    # 두 번째 로봇 (description.launch.py 도 robot_name:=amr_02 prefix:=amr_02/ 로 별도 기동)
    ros2 launch amr_description spawn.launch.py robot_name:=amr_02 x:=2.0

인자
    robot_name    네임스페이스 + Gazebo 모델 이름 (기본 amr_01)
    world         스폰 대상 월드 이름 (기본 warehouse; SDF <world name=...>)
    x, y, z, yaw  스폰 자세 [m, rad]. z 기본 0.02: 바퀴가 지면에 살짝 떠서 내려앉도록 (초기 관통 방지)
    use_sim_time  (기본 true) — 브리지/스폰 노드에 전달
    config_dir    sensors.yaml 디렉토리 (기본 $ROS_WS/config, ROS_WS 미설정 시 /ros2_ws/config).
                  브리지 토픽 목록을 sensors.yaml 의 topic 키에서 만든다 (xacro 와 같은 단일 진실 공급원)
    bridge_config 직접 작성한 parameter_bridge YAML 로 자동 생성을 대체할 때 (기본 "" = 자동 생성)

토픽 (ROS 는 모두 /<robot_name>/ 아래 상대 이름, gz 는 /<robot_name>/<같은 이름>)
    GZ→ROS  joint_states, scan, imu/data_raw, camera/image_raw, camera/camera_info,
            camera/depth/image_raw, camera/depth/camera_info,
            camera/depth/points (gz: camera/depth/image_raw/points, lazy — 구독자가 있을 때만 브리지·생성),
            odom_gz (DiffDrive 자체 오도메트리, 비교용), ground_truth/odom (평가용, frame_id world)
    ROS→GZ  cmd_vel
    브리지하지 않음: DiffDrive/OdometryPublisher 의 tf — odom→base_footprint 는 EKF 가 발행

브리지 구성 (결정)
    - parameter_bridge 는 config_file(생성한 YAML, 상대 이름) + expand_gz_topic_names=true 로 네임스페이스
      /<robot_name> 에서 동작한다 (ros_gz_bridge 0.244.26 에서 확인). 이름은 sensors.yaml 에서 읽으므로 토픽을
      바꾸면 xacro(gz 센서 <topic>)와 브리지가 함께 따라온다.
    - 이미지 두 토픽은 ros_gz_image image_bridge 로 보낸다: image_transport 를 거치므로 compressed /
      compressedDepth / theora 파생 토픽이 함께 생겨 웹 대시보드·rosbag 대역폭에 유리하고, gz 토픽 이름을
      ROS 이름과 같게 잡아 두어 재매핑이 필요 없다. camera_info 는 parameter_bridge 가 넘긴다.
    - 640x480 점군은 메시지당 3.6 MB(15 Hz → 55 MB/s)라 lazy 로 둔다: ROS 구독자가 없으면 브리지가 gz
      구독을 열지 않고, 그러면 gz-sensors 깊이 카메라도 points 메시지를 채우지 않는다 (소스 기준).

월드 요건 (amr_simulation 담당) — 아래 시스템이 월드 SDF 에 있어야 한다
    ignition-gazebo-physics-system, ignition-gazebo-user-commands-system (create 서비스),
    ignition-gazebo-sensors-system (<render_engine>ogre2</render_engine>; 헤드리스는 EGL),
    ignition-gazebo-imu-system, (선택) ignition-gazebo-scene-broadcaster-system.
    /clock 브리지는 월드 런치가 전역으로 한 번만 띄운다 (여기서 띄우지 않는다).

다중 로봇
    로봇마다 description.launch.py(robot_name, prefix="<robot_name>/") + 이 런치(robot_name, x, y)를
    한 쌍씩 기동한다. 토픽은 네임스페이스로, TF 프레임/관절 이름은 prefix 로 분리되어 충돌하지 않는다
    (docs/architecture/multi_robot.md §2). Gazebo 모델 이름 = robot_name 이라 같은 이름으로 두 번 스폰할 수 없다.
    모든 로봇은 월드 기동 직후(sim 시간이 작을 때) 스폰한다: Fortress(ign-sensors6) 센서의 다음 갱신 시각은
    0 에서 시작해 한 주기씩만 늘어나므로, sim 시간 T 에 스폰된 로봇의 센서는 T × update_rate 번을 매 물리
    스텝마다 갱신하며 따라잡는다 (실측: T ≈ 92 s 에 스폰한 amr_02 의 scan/imu/camera 가 ~1 kHz(sim)로 발행,
    월드 RTF 0.12 → 0.004, 호스트 부하 상태). 따라잡는 동안(카메라 30 Hz 기준 T × 30 스텝) 시뮬레이션이
    크게 느려지고 센서 토픽이 1 kHz 로 쏟아지므로, 운용 중 재스폰은 피한다.
    ros_gz_sim create(0.244.26)는 /world/<world>/create 응답을 5 s 안에 못 받으면 "timed out" ERROR 를 찍고도
    "OK creation" 과 종료 코드 0 으로 끝난다(확인) — 고부하에서 스폰 실패가 조용히 지나가므로
    `ign model --list` 로 모델이 생겼는지 확인한다.

Fortress 점군 주의
    camera/depth/points 의 header.frame_id 는 camera_depth_optical_frame 이지만 데이터는 센서 본체 규약
    (x 전방, y 좌, z 상)이다. 소비 노드에서 frame_id 를 camera_link 로 바꿔 해석하거나 depth 이미지 +
    camera_info 에서 점군을 직접 만들 것 (urdf/amr_gazebo.xacro, config/sensors.yaml 주석 참조).
"""

import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
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


def _bridge_entries(sensors: dict) -> list:
    """parameter_bridge YAML 항목. 상대 이름 → 브리지 노드의 네임스페이스로 확장된다."""
    lidar, rgb, depth, imu = (sensors["lidar"], sensors["rgb_camera"],
                              sensors["depth_camera"], sensors["imu"])

    def gz_to_ros(ros, gz, ros_type, gz_type, lazy=False):
        return {"ros_topic_name": ros, "gz_topic_name": gz, "ros_type_name": ros_type,
                "gz_type_name": gz_type, "direction": "GZ_TO_ROS", "lazy": lazy}

    return [
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


def _setup(context, *args, **kwargs):
    """런치 시점에 sensors.yaml 을 읽어 브리지 설정을 만들고 노드를 구성한다."""
    robot_name = LaunchConfiguration("robot_name").perform(context)
    world = LaunchConfiguration("world").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in ("true", "1")
    config_dir = LaunchConfiguration("config_dir").perform(context)
    bridge_config = LaunchConfiguration("bridge_config").perform(context)

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
            yaml.safe_dump(_bridge_entries(sensors), f, sort_keys=False)
            bridge_config = f.name
        actions.append(LogInfo(msg=f"[{robot_name}] bridge config generated: {bridge_config}"))

    # 모델 스폰: robot_state_publisher 가 발행하는 robot_description 토픽에서 URDF 를 읽는다
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=robot_name,
        output="screen",
        arguments=[
            "-world", world,
            "-name", robot_name,
            "-topic", f"/{robot_name}/robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

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

    return actions + [spawn, bridge, image_bridge]


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
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
