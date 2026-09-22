"""
로봇 모델 로드 + robot_state_publisher (명세 4장 1절 로봇 모델링, 4장 2절 TF).

urdf/amr.urdf.xacro 를 전개해 /<robot_name>/robot_description 을 발행하고
robot_state_publisher 를 네임스페이스 /<robot_name> 에서 띄운다.
치수/질량/관성/동역학 제한/센서 사양/토픽·프레임 이름은 <config_dir>/robot_params.yaml, sensors.yaml
에서 읽는다 (xacro 가 런치 시점에 평가하므로 YAML 수정 후 재빌드 없이 재런치만 하면 된다).

    ros2 launch amr_description description.launch.py
    ros2 launch amr_description description.launch.py robot_name:=amr_02 prefix:=amr_02/
    ros2 launch amr_description description.launch.py payload:=medium   # 중형 박스 10 kg 적재

인자
    robot_name    네임스페이스 + Gazebo 모델 이름 (기본 amr_01)
    prefix        모든 링크/조인트/프레임 이름과 센서 header.frame_id 의 접두어 (기본 "").
                  다중 로봇은 "<robot_name>/" (docs/architecture/multi_robot.md §2) — TF 는 전역 /tf 하나를
                  공유하므로 접두어가 없으면 프레임이 충돌한다. 접두어는 xacro 한 곳에서 URDF 링크/관절,
                  Gazebo 센서 frame_id, DiffDrive/OdometryPublisher 프레임에 함께 들어가므로
                  robot_state_publisher 의 frame_prefix 는 쓰지 않는다 (같이 쓰면 "amr_02/amr_02/..." 이중 접두어).
                  joint_states 의 관절 이름도 "<prefix>left_wheel_joint" 로 와서 URDF 와 그대로 맞는다.
                  CLI 에서 빈 값(prefix:=)은 ros2 launch 가 거부하므로 접두어가 없으면 인자를 생략한다
    use_sim_time  Gazebo /clock 사용 (기본 true)
    config_dir    YAML 디렉토리 (기본 $ROS_WS/config, ROS_WS 미설정 시 /ros2_ws/config)
    payload       적재물 종류 none | small | medium | large (기본 none). 데크 위 실제 크기 박스 + 질량
                  (robot_params.yaml payload.<종류>.{size, mass})
    payload_mass  적재 질량 [kg] 덮어쓰기 (기본 -1 = 종류의 질량). payload 없이 질량만 주면 그 질량을 담는
                  가장 작은 종류의 박스로 모델링한다 (payload_mass:=25 → large). 0 이면 적재물 없음.
                  스폰 시점 질량만 바꾼다 — 주행 중 적재/하역은 runtime_payload
    runtime_payload
                  true(기본)면 주행 중 적재/하역용 DetachableJoint 를 모델에 넣는다 (urdf/amr_gazebo.xacro
                  "런타임 적재물"). 화물 모델은 amr_simulation payload_manager_node 가 만든다
                  (spawn.launch.py 가 기동)
    self_visibility_bit
                  자기 차체 가시성 비트 (기본 "" = robot_name 끝 숫자로 자동, urdf/amr.urdf.xacro). 동시에 뜨는
                  로봇끼리 달라야 한다 — 같으면 서로의 LiDAR 에 안 보인다
    use_joint_state_publisher
                  Gazebo 없이 모델만 볼 때 true (기본 false). 시뮬레이션에서는
                  joint_states 가 Gazebo JointStatePublisher → 브리지(spawn.launch.py)로 들어온다

발행
    /<robot_name>/robot_description   (latched)
    /tf_static                         base_footprint → base_link → 센서/캐스터/적재함 프레임 (URDF 고정 조인트)
    /tf                                base_link → 바퀴 프레임 (joint_states 기반). joint_states 는 물리
                                       스텝마다 1 kHz 로 오지만 /tf 는 robot_state_publisher 의
                                       publish_frequency(기본 20 Hz) 상한으로 나간다 (multi_robot.md §2)
    odom → base_footprint 는 여기서 발행하지 않는다 (EKF 담당, config/ekf.yaml)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _default_config_dir() -> str:
    """최상위 config 디렉토리 기본값: $ROS_WS/config, 없으면 컨테이너 규약 경로."""
    ros_ws = os.environ.get("ROS_WS", "")
    return os.path.join(ros_ws, "config") if ros_ws else "/ros2_ws/config"


def generate_launch_description() -> LaunchDescription:
    robot_name = LaunchConfiguration("robot_name")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_dir = LaunchConfiguration("config_dir")
    payload = LaunchConfiguration("payload")
    payload_mass = LaunchConfiguration("payload_mass")
    runtime_payload = LaunchConfiguration("runtime_payload")
    self_visibility_bit = LaunchConfiguration("self_visibility_bit")

    args = [
        DeclareLaunchArgument(
            "robot_name", default_value="amr_01",
            description="로봇 네임스페이스 및 Gazebo 모델 이름"),
        DeclareLaunchArgument(
            "prefix", default_value="",
            description="TF 프레임/관절 이름 접두어 (다중 로봇: '<robot_name>/')"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="시뮬레이션 시계 사용 여부"),
        DeclareLaunchArgument(
            "config_dir", default_value=_default_config_dir(),
            description="robot_params.yaml / sensors.yaml 디렉토리"),
        DeclareLaunchArgument(
            "payload", default_value="none",
            description="적재물 종류 none|small|medium|large — 데크 위 실제 크기 박스"),
        DeclareLaunchArgument(
            "payload_mass", default_value="-1",
            description="적재 질량 [kg] 덮어쓰기 (-1 = payload 종류의 질량, 0 = 적재물 없음)"),
        DeclareLaunchArgument(
            "runtime_payload", default_value="true",
            description="주행 중 적재/하역용 DetachableJoint (화물 모델 <robot_name>_cargo)"),
        DeclareLaunchArgument(
            "self_visibility_bit", default_value="",
            description="자기 차체 가시성 비트 (빈 값 = robot_name 끝 숫자로 자동)"),
        DeclareLaunchArgument(
            "use_joint_state_publisher", default_value="false",
            description="Gazebo 없이 모델만 볼 때 joint_state_publisher 기동"),
    ]

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("amr_description"), "urdf", "amr.urdf.xacro"])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", xacro_file,
            " robot_name:=", robot_name,
            " prefix:=", prefix,
            " config_dir:=", config_dir,
            " payload:=", payload,
            " payload_mass:=", payload_mass,
            " runtime_payload:=", runtime_payload,
            " self_visibility_bit:=", self_visibility_bit,
        ]),
        value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_name,
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace=robot_name,
        condition=IfCondition(LaunchConfiguration("use_joint_state_publisher")),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(args + [robot_state_publisher, joint_state_publisher])
