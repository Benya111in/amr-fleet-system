"""클래스 매핑(config/classes.yaml) 과 마커 형식 테스트."""

import os

from amr_perception.class_mapping import ClassMapper
from amr_perception.markers import format_label, marker_spec
import pytest
import yaml

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config',
                      'classes.yaml')

# ultralytics COCO 이름 일부 (id → 이름)
COCO = {0: 'person', 1: 'bicycle', 2: 'car', 11: 'stop sign', 24: 'backpack', 28: 'suitcase'}


def test_config_maps_coco_ids_to_spec_classes():
    m = ClassMapper.from_yaml(CONFIG)
    assert m.classes == ['box', 'person', 'sign']
    table = m.bind(COCO)
    assert table == {0: (1, 'person'), 11: (2, 'sign'), 28: (0, 'box')}
    assert m.model_ids() == [0, 11, 28]
    assert m.map(0, 0.9) == (1, 'person')
    assert m.map(2, 0.99) is None          # car → 버림
    assert m.map(28, 0.2) is None          # 클래스별 최소 점수 미만


def test_config_coco_ids_match_names():
    with open(CONFIG, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    for name, cid in cfg['coco_ids'].items():
        assert COCO[cid] == name
        assert name in cfg['model_class_map']
    ultra = pytest.importorskip('ultralytics')
    path = os.path.join(os.path.dirname(ultra.__file__), 'cfg', 'datasets', 'coco.yaml')
    with open(path, 'r', encoding='utf-8') as f:
        coco = yaml.safe_load(f)
    for name, cid in cfg['coco_ids'].items():
        assert coco['names'][cid] == name


def test_finetuned_names_pass_through():
    m = ClassMapper.from_yaml(CONFIG)
    assert m.bind({0: 'box', 1: 'person', 2: 'sign'}) == {
        0: (0, 'box'), 1: (1, 'person'), 2: (2, 'sign')}


def test_invalid_mapping_rejected():
    with pytest.raises(ValueError):
        ClassMapper.from_dict({'classes': ['box'], 'model_class_map': {'person': 'human'}})
    empty = ClassMapper.from_dict({})
    assert empty.bind(COCO) == {}
    assert empty.model_ids() == []


def test_marker_label_format_matches_spec_example():
    assert format_label('box', 0.92, 1.5) == 'Class: box, Conf: 0.92, Dist: 1.5m'
    assert format_label('box', 0.916, 1.46, capitalize=True) == \
        'Class: Box, Conf: 0.92, Dist: 1.5m'


def test_marker_spec_geometry():
    s = marker_spec('box', 0.9, 2.0, center_z=0.1)
    assert s.color[0] > 0.8 and s.color[1] < 0.2          # 빨간 육면체 (명세 8장)
    assert s.scale == (0.5, 0.4, 0.3)
    assert s.cube_z == pytest.approx(0.15)                # 바닥에 앉힘
    assert s.text_z > s.cube_z + 0.15
    high = marker_spec('box', 0.9, 2.0, center_z=1.2)     # 선반 위 박스
    assert high.cube_z == pytest.approx(1.2)
    other = marker_spec('forklift', 0.5, 3.0, center_z=0.0)
    assert other.scale == (0.3, 0.3, 0.3)
    assert 'Class: forklift' in other.text
