"""scripts/yolo_fps_probe.py 순수 함수 테스트 — 입력 프레임 적재와 백분위."""

import importlib.util
import math
import os

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(scope='module')
def probe():
    spec = importlib.util.spec_from_file_location(
        'yolo_fps_probe', os.path.join(PKG, 'scripts', 'yolo_fps_probe.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_load_frames_from_heldout_and_synthetic(probe, tmp_path):
    frames = probe.load_frames(os.path.join(PKG, 'models', 'heldout', 'images'), 3)
    assert len(frames) == 3 and frames[0].shape == (480, 640, 3)
    synth = probe.load_frames('', 2)
    assert len(synth) == 2 and synth[0].shape == (480, 640, 3)
    with pytest.raises(FileNotFoundError):
        probe.load_frames(str(tmp_path), 2)


def test_percentile(probe):
    assert probe.percentile([1.0, 2.0, 3.0], 50) == 2.0
    assert math.isnan(probe.percentile([], 50))
