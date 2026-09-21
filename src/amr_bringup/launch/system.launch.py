"""
전체 시스템 통합 런치 (명세 10장).

단일 로봇 기준 전체 파이프라인을 기동한다.
다중 로봇(5대)은 amr_fleet/launch/multi_robot.launch.py 를 사용한다.

    ros2 launch amr_bringup system.launch.py

아직 각 하위 런치가 구현되지 않았으므로, 구현 전까지는
use_* 인자로 개별 스택을 꺼두고 부분 기동할 수 있게 해 두었다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_name = LaunchConfiguration("robot_name")

    args = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="시뮬레이션 시계 사용 여부"),
        DeclareLaunchArgument(
            "robot_name", default_value="amr_01",
            description="로봇 네임스페이스 (다중 로봇 시 충돌 방지)"),
        DeclareLaunchArgument("use_simulation", default_value="true"),
        DeclareLaunchArgument("use_localization", default_value="true"),
        DeclareLaunchArgument("use_navigation", default_value="true"),
        DeclareLaunchArgument("use_perception", default_value="true"),
        DeclareLaunchArgument("use_behavior", default_value="false"),
    ]

    def include(pkg: str, launch_file: str, condition_arg: str):
        """하위 패키지 런치를 조건부로 포함한다."""
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare(pkg), "launch", launch_file])),
            condition=IfCondition(LaunchConfiguration(condition_arg)),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "robot_name": robot_name,
            }.items(),
        )

    stacks = [
        include("amr_simulation", "warehouse.launch.py", "use_simulation"),
        include("amr_localization", "localization.launch.py", "use_localization"),
        include("amr_navigation", "navigation.launch.py", "use_navigation"),
        include("amr_perception", "perception.launch.py", "use_perception"),
        include("amr_behavior", "behavior.launch.py", "use_behavior"),
    ]

    return LaunchDescription(args + stacks)
