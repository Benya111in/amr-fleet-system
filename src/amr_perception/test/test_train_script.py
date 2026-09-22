"""scripts/train.py 테스트 — ultralytics.YOLO 를 가짜로 바꿔 인자 전달·지표 JSON·가중치 복사를 확인한다."""

import importlib.util
import json
import os
import sys
import types

import pytest

SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts',
                      'train.py')


class _Box:
    map50 = 0.8
    map = 0.6
    mp = 0.9
    mr = 0.7
    ap_class_index = [0, 2]
    ap50 = [0.85, 0.75]


class _FakeYOLO:
    calls = []

    def __init__(self, weights):
        self.weights = weights
        self.names = {0: 'box', 1: 'person', 2: 'sign'}

    def train(self, **kw):
        _FakeYOLO.calls.append(('train', self.weights, kw))
        run = os.path.join(kw['project'], kw['name'], 'weights')
        os.makedirs(run, exist_ok=True)
        with open(os.path.join(run, 'best.pt'), 'wb') as f:
            f.write(b'weights')

    def val(self, **kw):
        _FakeYOLO.calls.append(('val', self.weights, kw))
        return types.SimpleNamespace(box=_Box())


@pytest.fixture
def train_module(monkeypatch):
    fake = types.ModuleType('ultralytics')
    fake.YOLO = _FakeYOLO
    monkeypatch.setitem(sys.modules, 'ultralytics', fake)
    spec = importlib.util.spec_from_file_location('train_script', SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_train_writes_metrics_and_copies_weights(train_module, tmp_path, capsys):
    _FakeYOLO.calls.clear()
    out = tmp_path / 'models' / 'yolov8n_warehouse.pt'
    rc = train_module.main(['--data', str(tmp_path / 'data.yaml'), '--epochs', '3',
                            '--device', 'cpu', '--project', str(tmp_path / 'runs'),
                            '--name', 'unit', '--out', str(out)])
    assert rc == 0
    kind, _, kw = _FakeYOLO.calls[0]
    assert kind == 'train' and kw['epochs'] == 3 and kw['device'] == 'cpu'
    assert kw['amp'] is False and kw['deterministic'] is True
    assert _FakeYOLO.calls[1][0] == 'val' and _FakeYOLO.calls[1][1].endswith('best.pt')
    with open(tmp_path / 'runs' / 'unit' / 'metrics.json', 'r', encoding='utf-8') as f:
        m = json.load(f)
    assert m['mAP50'] == pytest.approx(0.8) and m['mAP50_95'] == pytest.approx(0.6)
    assert m['ap50_per_class'] == {'box': 0.85, 'sign': 0.75}
    assert out.read_bytes() == b'weights'
    assert '"copied_to"' in capsys.readouterr().out


def test_resolve_model_falls_back_to_name(train_module):
    assert train_module.resolve_model('no_such_weights.pt') == 'no_such_weights.pt'
