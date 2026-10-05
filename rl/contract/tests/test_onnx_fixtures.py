"""
ONNX 입출력 계약 고정물 시험 (torch·onnx·onnxruntime 이 있을 때만).

`.gitignore` 가 `*.onnx` 를 무시하므로 고정물 모델은 커밋하지 않는다. 시험마다 `make_fixtures.export` 로
임시 디렉터리에 다시 만들고(시드 고정, 결정적), 커밋된 `fixture_expected.json` 과 출력을 비교한다.
"""

import json
import os

import numpy as np
import pytest

from amr_rl_contract import constants as C
from amr_rl_contract.observation import OBS_DIM

ort = pytest.importorskip('onnxruntime')
onnx = pytest.importorskip('onnx')
pytest.importorskip('torch')
import make_fixtures as mf  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
EXPECTED = os.path.join(os.path.dirname(HERE), 'onnx', 'fixtures', 'fixture_expected.json')


@pytest.fixture(scope='module')
def models(tmp_path_factory):
    """임시 디렉터리에 고정물 두 개를 만든다."""
    return mf.export(str(tmp_path_factory.mktemp('onnx')))


def _io(path):
    m = onnx.load(path)

    def dims(t):
        return [d.dim_param or d.dim_value for d in t.type.tensor_type.shape.dim]
    ins = [(i.name, i.type.tensor_type.elem_type, dims(i)) for i in m.graph.input]
    outs = [(o.name, o.type.tensor_type.elem_type, dims(o)) for o in m.graph.output]
    meta = {p.key: p.value for p in m.metadata_props}
    return m, ins, outs, meta


def test_b_signature(models):
    m, ins, outs, meta = _io(models[0])
    assert m.opset_import[0].version == 17
    assert ins == [('obs', onnx.TensorProto.FLOAT, ['batch', OBS_DIM])]
    assert [(n, t) for n, t, _ in outs] == [('action', 1), ('mu', 1), ('log_std', 1)]
    assert all(d[-1] == 2 for _, _, d in outs)
    assert meta['arm'] == 'B' and meta['contract_version'] == C.CONTRACT_VERSION
    assert meta['normalization'] == 'inside'


def test_c_signature(models):
    m, ins, outs, meta = _io(models[1])
    assert ins == [('obs', onnx.TensorProto.FLOAT, ['batch', OBS_DIM]),
                   ('mask', onnx.TensorProto.FLOAT, ['batch', C.N_ACTIONS_C])]
    assert outs == [('logits', onnx.TensorProto.FLOAT, ['batch', C.N_ACTIONS_C]),
                    ('action', onnx.TensorProto.INT64, ['batch'])]
    assert meta['fallback_index'] == str(C.FALLBACK_INDEX)


def test_expected_outputs_and_mask_respected(models):
    with open(EXPECTED, encoding='utf-8') as f:
        exp = json.load(f)
    rows = mf.run_ort(models[0], models[1], mf.golden_inputs())
    assert len(rows) == len(exp['rows']) >= 14
    for a, b in zip(rows, exp['rows']):
        assert a['name'] == b['name'] and a['c_action'] == b['c_action']
        assert np.allclose(a['b_action'], b['b_action'], atol=exp['tolerance'])
        assert all(-1.0 <= v <= 1.0 for v in a['b_action'])
        assert b['c_logits_valid'][b['c_action']] is not None     # 선택 = 유효 셀


def test_fallback_only_mask_selects_fallback(models):
    sess = ort.InferenceSession(models[1], providers=['CPUExecutionProvider'])
    rng = np.random.default_rng(0)
    obs = rng.normal(0, 1, (16, OBS_DIM)).astype(np.float32)
    mask = np.zeros((16, C.N_ACTIONS_C), dtype=np.float32)
    mask[:, C.FALLBACK_INDEX] = 1.0
    _, act = sess.run(None, {'obs': obs, 'mask': mask})
    assert act.dtype == np.int64 and np.all(act == C.FALLBACK_INDEX)
    mask = (rng.random((16, C.N_ACTIONS_C)) < 0.2).astype(np.float32)
    mask[:, C.FALLBACK_INDEX] = 1.0
    _, act = sess.run(None, {'obs': obs, 'mask': mask})
    assert np.all(mask[np.arange(16), act] == 1.0)
