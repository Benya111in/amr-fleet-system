"""behavior.yaml 구조·문서화 규칙과 behavior.launch.py 인자 처리 시험."""

import importlib.util
import math
import os

from launch import LaunchContext
from launch_ros.actions import Node
import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(PKG, 'config', 'behavior.yaml')
LAUNCH = os.path.join(PKG, 'launch', 'behavior.launch.py')
TREES = os.path.join(PKG, 'behavior_trees')


def _load_launch_module():
    spec = importlib.util.spec_from_file_location('behavior_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _config():
    with open(CONFIG, encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_docks_have_staging_and_standoff():
    docks = _config()['/**']['ros__parameters']['docks']
    assert len(docks['ids']) >= 4
    for dock_id in docks['ids']:
        entry = docks[dock_id]
        assert len(entry['staging']) == 3
        assert 0.3 < entry['standoff'] < 1.5
        assert -math.pi - 1e-6 <= entry['staging'][2] <= math.pi + 1e-6


def test_executor_and_docking_parameters():
    cfg = _config()
    executor = cfg['/**/task_executor_node']['ros__parameters']
    docking = cfg['/**/docking_server_node']['ros__parameters']
    assert executor['dock_attempts'] == 3            # 명세: 최대 3회
    assert executor['dock_max_retries'] == 1         # 단일 재시도 카운터
    assert executor['charger_dock_id'] in cfg['/**']['ros__parameters']['docks']['ids']
    assert docking['position_tolerance'] <= 0.02     # 명세: 2 cm
    assert docking['angle_tolerance'] <= math.radians(1.0) + 1e-6
    assert docking['control_law'] in ('graceful', 'proportional')
    assert docking['max_linear_speed'] <= 0.2        # Critical 존 상한 이하


def test_every_parameter_line_is_documented():
    """모든 스칼라 파라미터 줄에 의미·단위 주석이 있어야 한다 (명세 4.10 문서화)."""
    undocumented = []
    with open(CONFIG, encoding='utf-8') as f:
        for number, line in enumerate(f, 1):
            text = line.strip()
            if not text or text.startswith('#') or text.startswith('/'):
                continue
            if text.endswith(':') or text.startswith('ros__parameters'):
                continue
            key = text.split(':', 1)[0]
            if key in ('ids',) or text.startswith(('dock_', 'charger_c')):
                continue    # 도크 표는 블록 위 주석으로 설명
            if '#' not in text:
                undocumented.append(f'{number}: {text}')
    assert not undocumented, undocumented


def test_launch_arguments_and_nodes():
    module = _load_launch_module()
    ld = module.generate_launch_description()
    names = {a.name for a in ld.entities if hasattr(a, 'name')}
    for arg in ('namespace', 'use_sim_time', 'params_file', 'groot_publisher_port',
                'waiting_pose', 'frame_prefix'):
        assert arg in names
    context = LaunchContext()
    context.launch_configurations.update({
        'namespace': '/amr_02', 'robot_name': '', 'use_sim_time': 'false',
        'params_file': CONFIG, 'robot_params_file': '', 'frame_prefix': 'auto',
        'groot_publisher_port': '1670', 'groot_server_port': '1671',
        'waiting_pose': '1.0, 2.0, 0.5', 'start_executor': 'true', 'start_docking': 'true',
        'log_level': 'debug'})
    nodes = module._launch_setup(context)
    assert len(nodes) == 2
    assert all(isinstance(n, Node) for n in nodes)
    context.launch_configurations.update({'start_docking': 'false'})
    assert len(module._launch_setup(context)) == 1


def test_parse_pose():
    module = _load_launch_module()
    assert module._parse_pose('') is None
    assert module._parse_pose('1, 2, 3') == [1.0, 2.0, 3.0]
    with pytest.raises(ValueError):
        module._parse_pose('1,2')
    assert module._as_bool('True') and not module._as_bool('0')


def test_tree_files_exist_and_are_included():
    main = os.path.join(TREES, 'task_executor.xml')
    with open(main, encoding='utf-8') as f:
        text = f.read()
    for sub in ('move_to', 'perceive', 'dock_at', 'payload', 'recovery', 'charge', 'yield'):
        assert f'subtrees/{sub}.xml' in text
        assert os.path.isfile(os.path.join(TREES, 'subtrees', f'{sub}.xml'))
