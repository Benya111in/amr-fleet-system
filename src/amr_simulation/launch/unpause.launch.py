"""
월드 일시정지 해제 — 지정한 로봇 모델이 모두 월드에 생긴 것을 확인한 뒤 /world/<world>/control 에 pause: false.

    ros2 launch amr_simulation unpause.launch.py robots:=amr_01,amr_02,amr_03,amr_04,amr_05

다중 로봇 기동 순서 (warehouse.launch.py 머리말 "일시정지 기동"):
    1) warehouse.launch.py spawn_robot:=false paused:=true   — 월드가 sim 0 에 멈춘 채 뜬다
    2) 로봇마다 amr_description description.launch.py + spawn.launch.py (각 스폰 런치가 자기 모델 생성을 확인)
    3) 이 런치 — 모든 로봇 모델이 보이면 해제. 하나라도 timeout 안에 안 보이면 ERROR 로 끝나고 월드는 멈춘 채 남는다
       (센서 따라잡기 폭주를 일으키는 "늦은 스폰"을 막기 위해 일부만으로 풀지 않는다).
구현은 amr_description scripts/gz_world.py unpause
(spawn 과 같은 도구, `ign service` 로 scene/info·control 호출).

인자
    world    월드 이름 (기본 warehouse)
    robots   기다릴 모델 이름, 쉼표 구분 (기본 "" = 기다리지 않고 바로 해제)
    timeout  모델 대기 한도 [s] (기본 300)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="warehouse", description="월드 이름"),
        DeclareLaunchArgument("robots", default_value="", description="기다릴 모델 이름 (쉼표 구분)"),
        DeclareLaunchArgument("timeout", default_value="300", description="모델 대기 한도 [s]"),
        Node(
            package="amr_description", executable="gz_world.py", name="unpause", output="screen",
            arguments=["unpause", "--world", LaunchConfiguration("world"),
                       ["--wait-models=", LaunchConfiguration("robots")],
                       ["--verify-timeout=", LaunchConfiguration("timeout")]],
        ),
    ])
