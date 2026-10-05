"""
nav2 파라미터 덧씌우기 — 기존 nav2_params.yaml 을 고치지 않고 RLController 를 controller_server 에 싣는다.

두 방식:
  - 새 controller_id (기본 'RL'): DWA 블록을 그대로 복사해 plugin 만 바꾸고 controller_plugins 에 덧붙인다.
    내부 폴백 DWA 가 '<id>.*' 파라미터를 읽으므로 DWA 튜닝이 그대로 따라온다. BT 기본 제어기(DWA)는 그대로다.
  - replace_dwa: 'DWA' 블록의 plugin 을 RLController 로 바꾼다 — BT 의 ControllerSelector 기본값 'DWA' 가
    그대로 RL 을 쓰게 된다 (NavigateToPose 스모크용).
입력 YAML 의 '<prefix>'·'<robot_ns>' 자리표시는 navigation.launch.py 가 나중에 치환하므로 그대로 둔다.
"""

import copy
import tempfile

import yaml

PLUGIN = 'amr_rl_controller::RLController'
DWA_ID = 'DWA'


def overlay_nav2_params(params, model_path, mode='C', controller_id='RL', replace_dwa=False,
                        rl_overrides=None):
    """
    nav2 파라미터 dict 에 RLController 를 얹은 사본을 돌려준다.

    params: yaml.safe_load(nav2_params.yaml). model_path: ONNX 경로. mode: 'B' | 'C'.
    rl_overrides: '<id>.rl.*' 에 더할 dict (예: {'max_inference_ms': 25.0}).
    """
    if mode not in ('B', 'C'):
        raise ValueError(f'mode 는 B 또는 C: {mode!r}')
    if not model_path:
        raise ValueError('model_path 가 비어 있다')
    out = copy.deepcopy(params)
    cs = out['controller_server']['ros__parameters']
    if DWA_ID not in cs:
        raise KeyError('controller_server 에 DWA 블록이 없다 (폴백 DWA 파라미터의 출처)')
    rl = {'model_path': str(model_path), 'mode': mode}
    rl.update(rl_overrides or {})
    if replace_dwa:
        cid = DWA_ID
        block = cs[DWA_ID]
    else:
        cid = controller_id
        if cid in cs and cid != DWA_ID:
            raise ValueError(f'controller_id {cid!r} 가 이미 있다')
        block = copy.deepcopy(cs[DWA_ID])
        cs[cid] = block
        plugins = list(cs.get('controller_plugins', []))
        if cid not in plugins:
            plugins.append(cid)
        cs['controller_plugins'] = plugins
    block['plugin'] = PLUGIN
    block['rl'] = rl
    return out, cid


def write_overlay(src_yaml, model_path, mode='C', controller_id='RL', replace_dwa=False,
                  rl_overrides=None):
    """src_yaml 을 읽어 덧씌운 임시 파일 경로와 controller_id 를 돌려준다."""
    with open(src_yaml, encoding='utf-8') as f:
        params = yaml.safe_load(f)
    out, cid = overlay_nav2_params(params, model_path, mode, controller_id, replace_dwa,
                                   rl_overrides)
    with tempfile.NamedTemporaryFile('w', prefix='amr_rl_nav2_', suffix='.yaml', delete=False,
                                     encoding='utf-8') as f:
        yaml.safe_dump(out, f, sort_keys=False, allow_unicode=True)
        return f.name, cid
