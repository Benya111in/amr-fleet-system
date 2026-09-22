"""fleet_spawn: 스폰 목록 해석·검증, 공통·로봇별 통신 지연 → fleet 파라미터 (명세 4.9)."""

import math
from pathlib import Path

from amr_bringup import fleet_spawn as fs
import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'fleet_spawn.yaml'
FLEET_YAML = {
    '/fleet/fleet_manager_node': {'ros__parameters': {'comm_latency_ms': [0.0, 100.0],
                                                      'robot_ids': ''}},
    '/**/fleet_adapter_node': {'ros__parameters': {'comm_latency_ms': [0.0, 100.0]}},
}


def robots(*specs):
    return [{'name': n, 'x': x, 'y': y, 'yaw': 0.0} for n, x, y in specs]


def test_default_config_has_five_robots():
    spawn = fs.load_fleet_spawn(str(CONFIG))
    assert fs.robot_ids(spawn.robots) == [f'amr_0{i}' for i in range(1, 6)]
    assert spawn.pause_during_spawn is True
    assert spawn.spawn_z == pytest.approx(0.02)
    assert spawn.comm_latency_ms is None
    for r in spawn.robots:
        assert r.yaw == pytest.approx(math.pi / 2, abs=1e-4)   # 북쪽(창고 안쪽)을 본다


@pytest.mark.parametrize('value, expected', [
    (None, None), (50, (0.0, 50.0)), (0, (0.0, 0.0)), ([10, 80], (10.0, 80.0)),
    ((0.0, 100.0), (0.0, 100.0)),
])
def test_parse_latency_ok(value, expected):
    assert fs.parse_latency(value, 't') == expected


@pytest.mark.parametrize('value', [150, [-1, 10], [50, 10], [0, 100.5], True, 'abc', [1, 2, 3],
                                   [True, 5]])
def test_parse_latency_rejects(value):
    with pytest.raises(ValueError):
        fs.parse_latency(value, 't')


@pytest.mark.parametrize('item, match', [
    ({'name': 'amr_01', 'x': 0, 'y': 0}, '빠진 키'),
    ({'name': 'amr_01', 'x': 0, 'y': 0, 'yaw': 0, 'z': 1}, '모르는 키'),
    ({'name': 'robot1', 'x': 0, 'y': 0, 'yaw': 0}, '두 자리'),
    ({'name': 'amr_001', 'x': 0, 'y': 0, 'yaw': 0}, '두 자리'),
    ({'name': 'amr_01', 'x': 'a', 'y': 0, 'yaw': 0}, '숫자'),
    ({'name': 'amr_01', 'x': float('nan'), 'y': 0, 'yaw': 0}, '숫자'),
    ({'name': 'amr_01', 'x': 0, 'y': 0, 'yaw': 0, 'comm_latency_ms': 101}, '100'),
    ('amr_01', '사전'),
])
def test_parse_robot_rejects(item, match):
    with pytest.raises(ValueError, match=match):
        fs.parse_robot(item, 0)


def test_parse_robot_latency_per_robot():
    item = {'name': 'amr_02', 'x': 1, 'y': 2, 'yaw': 0.5, 'comm_latency_ms': [20, 80]}
    r = fs.parse_robot(item, 1)
    assert r == fs.RobotSpec('amr_02', 1.0, 2.0, 0.5, (20.0, 80.0))


def test_separation_and_duplicates():
    with pytest.raises(ValueError, match='중복'):
        fs.parse_fleet_spawn({'robots': robots(('amr_01', 0, 0), ('amr_01', 5, 0))})
    with pytest.raises(ValueError, match='min_separation_m'):
        fs.parse_fleet_spawn({'robots': robots(('amr_01', 0, 0), ('amr_02', 0.5, 0))})
    ok = fs.parse_fleet_spawn({'robots': robots(('amr_01', 0, 0), ('amr_02', 0.5, 0)),
                               'min_separation_m': 0.4})
    assert len(ok.robots) == 2


@pytest.mark.parametrize('data, match', [
    ({}, 'robots'), ({'robots': []}, 'robots'), ([1, 2], 'robots'),
    ({'robots': robots(('amr_01', 0, 0)), 'speed': 1}, '모르는 공통 옵션'),
    ({'robots': robots(('amr_01', 0, 0)), 'pause_during_spawn': 'yes'}, 'true/false'),
    ({'robots': robots(('amr_01', 0, 0)), 'spawn_timeout_s': 0}, '양수'),
    ({'robots': robots(('amr_01', 0, 0)), 'comm_latency_ms': [0, 200]}, '100'),
])
def test_parse_fleet_spawn_rejects(data, match):
    with pytest.raises(ValueError, match=match):
        fs.parse_fleet_spawn(data)


def test_select_and_find():
    spawn = fs.load_fleet_spawn(str(CONFIG))
    assert fs.robot_ids(fs.select_robots(spawn, 1)) == ['amr_01']
    assert len(fs.select_robots(spawn, 5)) == 5
    for bad in (0, 6, -1):
        with pytest.raises(ValueError, match='num_robots'):
            fs.select_robots(spawn, bad)
    assert fs.find_robot(spawn, 'amr_03').x == pytest.approx(22.0)
    assert fs.find_robot(spawn, 'amr_09') is None


def test_fleet_params_untouched_without_latency():
    spawn = fs.parse_fleet_spawn({'robots': robots(('amr_01', 0, 0))})
    assert fs.fleet_params_with_latency(FLEET_YAML, spawn, spawn.robots) is None


def test_fleet_params_shared_and_per_robot():
    data = {'robots': robots(('amr_01', 0, 0), ('amr_02', 2, 0), ('amr_03', 4, 0)),
            'comm_latency_ms': [10, 60]}
    data['robots'][1]['comm_latency_ms'] = 30
    spawn = fs.parse_fleet_spawn(data)
    out = fs.fleet_params_with_latency(FLEET_YAML, spawn, spawn.robots[:2])
    assert out[fs.MANAGER_KEY]['ros__parameters']['comm_latency_ms'] == [10.0, 60.0]
    assert out[fs.MANAGER_KEY]['ros__parameters']['robot_ids'] == ''       # 나머지는 보존
    assert out[fs.ADAPTER_KEY]['ros__parameters']['comm_latency_ms'] == [10.0, 60.0]
    assert out['/amr_02/fleet_adapter_node']['ros__parameters']['comm_latency_ms'] == [0.0, 30.0]
    keys = list(out)
    assert keys.index('/amr_02/fleet_adapter_node') > keys.index(fs.ADAPTER_KEY)   # 뒤가 이긴다
    untouched = FLEET_YAML['/**/fleet_adapter_node']['ros__parameters']
    assert untouched['comm_latency_ms'] == [0.0, 100.0]


def test_per_robot_latency_wins_in_rclpy(tmp_path):
    """생성한 파라미터 파일을 실제 rclpy 노드가 읽을 때 로봇별 키가 와일드카드를 이긴다."""
    rclpy = pytest.importorskip('rclpy')
    from rclpy.node import Node

    data = {'robots': robots(('amr_01', 0, 0), ('amr_02', 2, 0))}
    data['robots'][1]['comm_latency_ms'] = [20, 80]
    spawn = fs.parse_fleet_spawn(data)
    path = tmp_path / 'fleet.yaml'
    path.write_text(yaml.safe_dump(fs.fleet_params_with_latency(FLEET_YAML, spawn, spawn.robots),
                                   sort_keys=False))
    context = rclpy.Context()
    rclpy.init(args=['--ros-args', '--params-file', str(path)], context=context)
    try:
        values = {}
        for ns in ('amr_01', 'amr_02'):
            node = Node('fleet_adapter_node', namespace=ns, context=context,
                        automatically_declare_parameters_from_overrides=True)
            values[ns] = list(node.get_parameter('comm_latency_ms').value)
            node.destroy_node()
    finally:
        rclpy.shutdown(context=context)
    assert values == {'amr_01': [0.0, 100.0], 'amr_02': [20.0, 80.0]}
