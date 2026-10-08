"""
에이전트 통신 모의 설정의 환경 변수 대체값 (보너스 4 P5).

로봇 스택 런치(behavior.launch.py)를 고치지 않고 다섯 에이전트에 같은 장애 일정(분할 · 지연 · 손실)을 주려고
AMR_FLEET_COMM_MODEL_FILE 을 읽는다. 파라미터가 우선이고, 변수가 없으면 예전과 같다 (모의 꺼짐).
"""

import os
import pathlib

import pytest

try:
    import rclpy
    from rclpy.parameter import Parameter
    from amr_fleet.agent import node as agent_node
    HAVE_ROS = True
except ImportError:  # pragma: no cover - rclpy 없는 환경
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason='rclpy/amr_msgs 가 없는 환경')

MODEL = """
seed: 11
links:
  - {src: "*", dst: "*", comm_latency_ms: [0.0, 0.0], drop_rate: 0.0}
partitions:
  - {start_s: 10.0, end_s: 20.0, groups: [[amr_e1, amr_e2]]}
"""


def test_parameter_wins_over_environment():
    env = {agent_node.COMM_MODEL_ENV: '/env/model.yaml'}
    assert agent_node.comm_settings(False, '/param/model.yaml', env) == \
        (False, '/param/model.yaml')
    assert agent_node.comm_settings(True, '/param/model.yaml', env) == (True, '/param/model.yaml')


def test_environment_fills_an_empty_parameter_and_turns_emulation_on():
    env = {agent_node.COMM_MODEL_ENV: ' /env/model.yaml '}
    assert agent_node.comm_settings(False, '', env) == (True, '/env/model.yaml')


def test_no_environment_keeps_the_old_behaviour():
    assert agent_node.comm_settings(False, '', {}) == (False, '')
    assert agent_node.comm_settings(True, '  ', {agent_node.COMM_MODEL_ENV: ''}) == (True, '')


def test_node_loads_the_environment_model(tmp_path, monkeypatch):
    """실제 노드가 변수의 파일로 모의를 켠다 (분할 1 · 링크 규칙 1 · 파일 시드)."""
    path = tmp_path / 'model.yaml'
    path.write_text(MODEL, encoding='utf-8')
    monkeypatch.setenv(agent_node.COMM_MODEL_ENV, str(path))
    rclpy.init()
    try:
        node = agent_node.FleetAgentNode(
            namespace='/amr_e1',
            parameter_overrides=[Parameter('members', value='amr_e1,amr_e2,amr_e3'),
                                 Parameter('store_dir', value=str(tmp_path / 'store'))])
        try:
            text = node._comm.describe()
            assert node._comm.simulate
            assert '분할 1' in text and '링크 규칙 1' in text and 'seed=11' in text, text
            assert node._comm.cut_reason('amr_e1', 'amr_e3', 15.0) is not None
            assert node._comm.cut_reason('amr_e1', 'amr_e2', 15.0) is None
        finally:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()
    assert os.path.isfile(path) and pathlib.Path(path).read_text(encoding='utf-8') == MODEL
