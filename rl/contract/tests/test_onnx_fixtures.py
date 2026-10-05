"""ONNX 입출력 계약 고정물 시험 (torch·onnx·onnxruntime 이 있을 때만)."""

import json
import os

import numpy as np
import pytest

from amr_rl_contract import constants as C
from amr_rl_contract.observation import OBS_DIM

ort = pytest.importorskip('onnxruntime')
onnx = pytest.importorskip('onnx')
pytest.importorskip('torch')

HERE = os.path.dirname(os.path.abspath(__file__))
FIX = os.path.join(os.path.dirname(HERE), 'onnx', 'fixtures')


def _io(path):
    m = onnx.load(path)

    def dims(t):
        return [d.dim_param or d.dim_value for d in t.type.tensor_type.shape.dim]
    ins = [(i.name, i.type.tensor_type.elem_type, dims(i)) for i in m.graph.input]
    outs = [(o.name, o.type.tensor_type.elem_type, dims(o)) for o in m.graph.output]
    meta = {p.key: p.value for p in m.metadata_props}
    return m, ins, outs, meta


def test_b_signature():
    m, ins, outs, meta = _io(os.path.join(FIX, 'fixture_b.onnx'))
    assert m.opset_import[0].version == 17
    assert ins == [('obs', onnx.TensorProto.FLOAT, ['batch', OBS_DIM])]
    assert [(n, t) for n, t, _ in outs] == [('action', 1), ('mu', 1), ('log_std', 1)]
    assert all(d[-1] == 2 for _, _, d in outs)
    assert meta['arm'] == 'B' and meta['contract_version'] == C.CONTRACT_VERSION
    assert meta['normalization'] == 'inside'


def test_c_signature():
    m, ins, outs, meta = _io(os.path.join(FIX, 'fixture_c.onnx'))
    assert ins == [('obs', onnx.TensorProto.FLOAT, ['batch', OBS_DIM]),
                   ('mask', onnx.TensorProto.FLOAT, ['batch', C.N_ACTIONS_C])]
    assert outs == [('logits', onnx.TensorProto.FLOAT, ['batch', C.N_ACTIONS_C]),
                    ('action', onnx.TensorProto.INT64, ['batch'])]
    assert meta['fallback_index'] == str(C.FALLBACK_INDEX)


def test_expected_outputs_and_mask_respected():
    with open(os.path.join(FIX, 'fixture_expected.json'), encoding='utf-8') as f:
        exp = json.load(f)
    import make_fixtures as mf
    rows = mf.run_ort(os.path.join(FIX, 'fixture_b.onnx'), os.path.join(FIX, 'fixture_c.onnx'),
                      mf.golden_inputs())
    assert len(rows) == len(exp['rows']) >= 14
    for a, b in zip(rows, exp['rows']):
        assert a['name'] == b['name'] and a['c_action'] == b['c_action']
        assert np.allclose(a['b_action'], b['b_action'], atol=exp['tolerance'])
        assert all(-1.0 <= v <= 1.0 for v in a['b_action'])
        assert b['c_logits_valid'][b['c_action']] is not None     # 선택 = 유효 셀


def test_fallback_only_mask_selects_fallback():
    sess = ort.InferenceSession(os.path.join(FIX, 'fixture_c.onnx'),
                                providers=['CPUExecutionProvider'])
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


def test_regenerated_fixtures_match(tmp_path):
    import make_fixtures as mf
    b, c = mf.export(str(tmp_path))
    ins = mf.golden_inputs()
    new = mf.run_ort(b, c, ins)
    old = mf.run_ort(os.path.join(FIX, 'fixture_b.onnx'), os.path.join(FIX, 'fixture_c.onnx'), ins)
    for a, o in zip(new, old):
        assert a['c_action'] == o['c_action']
        assert np.allclose(a['b_mu'], o['b_mu'], atol=1e-6)
