"""
로봇별 fleet_adapter_node 만 띄우는 런치 — 중앙 노드와 다른 프로세스·컨테이너에 두기 위해.

    ros2 launch amr_fleet fleet_manager.launch.py launch_adapters:=false       # 중앙 (fleet 컨테이너)
    ros2 launch amr_fleet fleet_adapters.launch.py robot_ids:=amr_01,amr_02   # 어댑터 (다른 곳)

fleet_manager.launch.py 를 launch_manager:=false · traffic_manager:=false 기본값으로 그대로 쓴다 — 인자 이름과
해석(comm_latency_ms · seed · comm_model_file · auto_complete_after_s …)이 한 곳에 있다. 인자로 다시 켜면
그대로 켜진다. 왜 나누나: 기본 구성은 중앙 노드 둘과 어댑터 다섯을 한 런치(docker-compose `fleet` 한
컨테이너)에 띄우므로 중앙 컨테이너를 멈추면 어댑터(와 그 프로세스에 들어갈 분산 에이전트)도 함께 죽어
컨테이너 단위 장애 실험이 성립하지 않는다 (보고서 「분산 플릿 관리 협상 할당과 폴백」 통신 절,
docs/research/fleet-distributed/p0.md §3).
"""

import importlib.util
import pathlib

from launch import LaunchDescription


def _fleet_manager_launch():
    """옆에 설치된 fleet_manager.launch.py 를 모듈로 읽는다 (소스 트리 · share 어느 쪽이든 같은 폴더)."""
    path = pathlib.Path(__file__).resolve().parent / 'fleet_manager.launch.py'
    spec = importlib.util.spec_from_file_location('amr_fleet_fleet_manager_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description() -> LaunchDescription:
    """어댑터만: launch_manager · traffic_manager 기본값 false."""
    return _fleet_manager_launch().generate_launch_description(
        defaults={'launch_manager': 'false', 'traffic_manager': 'false'})
