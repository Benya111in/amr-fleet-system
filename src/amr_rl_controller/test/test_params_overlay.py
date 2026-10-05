"""nav2 파라미터 덧씌우기 (amr_rl_controller.params) 시험 — 실제 저장소 nav2_params.yaml 로."""

import os
import sys

import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from amr_rl_controller.params import overlay_nav2_params, PLUGIN, write_overlay  # noqa: E402

NAV2 = os.path.normpath(os.path.join(PKG, '..', 'amr_navigation', 'config', 'nav2_params.yaml'))


@pytest.fixture(scope='module')
def params():
    """저장소 nav2_params.yaml (치환 전 원본)."""
    with open(NAV2, encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_new_controller_id_copies_dwa_block(params):
    """새 id 는 DWA 튜닝을 그대로 복사하고 plugin·rl 만 바꾼다. 원본 DWA 는 그대로다."""
    out, cid = overlay_nav2_params(params, '/m.onnx', 'C')
    cs = out['controller_server']['ros__parameters']
    assert cid == 'RL'
    assert cs['controller_plugins'][-1] == 'RL'
    assert cs['RL']['plugin'] == PLUGIN
    assert cs['RL']['rl'] == {'model_path': '/m.onnx', 'mode': 'C'}
    dwa = params['controller_server']['ros__parameters']['DWA']
    for k, v in dwa.items():
        if k != 'plugin':
            assert cs['RL'][k] == v, k
    assert cs['DWA'] == dwa
    assert params['controller_server']['ros__parameters']['controller_plugins'] != \
        cs['controller_plugins']   # 원본 dict 를 바꾸지 않는다


def test_replace_dwa_and_overrides(params):
    """replace_dwa 는 DWA 자리를 쓰고 목록은 그대로다."""
    out, cid = overlay_nav2_params(params, '/m.onnx', 'B', replace_dwa=True,
                                   rl_overrides={'max_inference_ms': 10.0})
    cs = out['controller_server']['ros__parameters']
    assert cid == 'DWA'
    assert cs['DWA']['plugin'] == PLUGIN
    assert cs['DWA']['rl']['max_inference_ms'] == 10.0
    assert cs['controller_plugins'] == \
        params['controller_server']['ros__parameters']['controller_plugins']


def test_rejects_bad_input(params):
    """모드·경로·중복 id 검사."""
    with pytest.raises(ValueError):
        overlay_nav2_params(params, '/m.onnx', 'D')
    with pytest.raises(ValueError):
        overlay_nav2_params(params, '', 'C')
    with pytest.raises(ValueError):
        overlay_nav2_params(params, '/m.onnx', 'C', controller_id='PurePursuit')


def test_write_overlay_roundtrip_keeps_placeholders():
    """임시 파일은 다시 읽히고 '<prefix>' 자리표시가 남는다 (navigation.launch.py 가 치환)."""
    path, cid = write_overlay(NAV2, '/m.onnx')
    try:
        with open(path, encoding='utf-8') as f:
            text = f.read()
        assert '<prefix>' in text
        again = yaml.safe_load(text)
        assert again['controller_server']['ros__parameters'][cid]['plugin'] == PLUGIN
    finally:
        os.unlink(path)
