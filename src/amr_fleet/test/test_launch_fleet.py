"""
fleet_manager.launch.py · fleet_adapters.launch.py 조립 (보너스 4 P0 "어댑터 분리").

런치를 띄우지 않고 OpaqueFunction 이 만든 Node 액션을 센다. 기본값은 예전과 같다 (중앙 둘 + 어댑터 N).
launch_adapters:=false 는 중앙 둘만, fleet_adapters.launch.py 는 어댑터 N 만 — 같은 인자 해석을 쓴다.
"""

import importlib.util
from pathlib import Path

import pytest

try:
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters
    HAVE_LAUNCH = True
except ImportError:
    HAVE_LAUNCH = False

pytestmark = pytest.mark.skipif(not HAVE_LAUNCH, reason='launch/launch_ros 가 없는 환경')

LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'
CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'
ROBOTS = 'amr_01,amr_02,amr_03'


def _load(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _nodes(launch_name, **args):
    """런치 인자를 채우고 OpaqueFunction 이 만든 Node 들의 (실행 파일, 파라미터 사전 목록)."""
    ld = _load(launch_name).generate_launch_description()
    ctx = LaunchContext()
    ctx.launch_configurations.update({
        'robot_ids': ROBOTS, 'params_file': str(CONFIG_DIR / 'fleet.yaml'),
        'traffic_params_file': str(CONFIG_DIR / 'traffic.yaml')})
    ctx.launch_configurations.update({k: str(v) for k, v in args.items()})
    opaque = None
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            if entity.name not in ctx.launch_configurations:
                entity.visit(ctx)
        elif isinstance(entity, OpaqueFunction):
            opaque = entity
    actions = opaque.visit(ctx)
    return [a for a in actions if isinstance(a, Node)]


def _executables(nodes):
    return sorted(n.node_executable for n in nodes)


def _overrides(node):
    """Node 의 두 번째 파라미터 원소(런치가 만든 덮어쓰기 사전)를 평가한 값."""
    return evaluate_parameters(LaunchContext(), node._Node__parameters)[1]


def test_default_launch_is_unchanged():
    nodes = _nodes('fleet_manager.launch.py')
    assert _executables(nodes) == sorted(
        ['fleet_manager_node', 'traffic_manager_node'] + ['fleet_adapter_node'] * 3)


def test_launch_adapters_false_keeps_only_central_nodes():
    nodes = _nodes('fleet_manager.launch.py', launch_adapters='false')
    assert _executables(nodes) == ['fleet_manager_node', 'traffic_manager_node']


def test_adapters_only_launch_shares_argument_handling():
    nodes = _nodes('fleet_adapters.launch.py', comm_latency_ms='[5.0, 30.0]', seed='9',
                   simulate_latency='false')
    assert _executables(nodes) == ['fleet_adapter_node'] * 3
    params = [_overrides(n) for n in nodes]
    assert [p['robot_id'] for p in params] == ROBOTS.split(',')
    assert all(tuple(p['comm_latency_ms']) == (5.0, 30.0) and p['seed'] == 9
               and p['simulate_latency'] is False and p['use_sim_time'] is True for p in params)
    # 인자로 다시 켜면 켜진다 (기본값만 바뀐다)
    again = _nodes('fleet_adapters.launch.py', launch_manager='true', traffic_manager='true')
    assert len(again) == 5


@pytest.mark.parametrize('arg, value', [
    ('launch_adapters', 'maybe'), ('launch_manager', '1.0'), ('seed', '-3'), ('seed', 'true')])
def test_bad_launch_arguments_are_rejected(arg, value):
    with pytest.raises(ValueError):
        _nodes('fleet_manager.launch.py', **{arg: value})


def test_unknown_default_override_is_rejected():
    with pytest.raises(ValueError):
        _load('fleet_manager.launch.py').generate_launch_description(defaults={'robot_idz': 'x'})
