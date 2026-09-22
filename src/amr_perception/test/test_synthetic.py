"""데이터셋 도구 테스트: 합성 렌더러, YOLO 라벨 형식, 3D 박스 투영, 가림 비율, 월드 SDF 파싱."""

import importlib.util
import json
import math
import os

from amr_perception import synthetic, world_objects
from amr_perception.synthetic import Label, RendererConfig, SyntheticSceneRenderer, WorldObject
import numpy as np
import pytest
import yaml

SCRIPTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts')


def test_label_line_round_trip():
    lab = Label(2, (100.0, 50.0, 300.0, 250.0))
    line = synthetic.yolo_label_line(lab, 640, 480)
    assert line == '2 0.312500 0.312500 0.312500 0.416667'
    back = synthetic.parse_label_line(line, 640, 480)
    assert back.class_id == 2
    assert np.allclose(back.bbox, lab.bbox, atol=1e-3)


def test_renderer_labels_are_valid():
    r = SyntheticSceneRenderer(seed=3)
    counts = np.zeros(3, int)
    for _ in range(25):
        img, labels = r.render()
        assert img.shape == (480, 640, 3) and img.dtype == np.uint8
        for lab in labels:
            x1, y1, x2, y2 = lab.bbox
            assert 0 <= x1 < x2 <= 639 and 0 <= y1 < y2 <= 479
            assert x2 - x1 >= 10 and y2 - y1 >= 10
            counts[lab.class_id] += 1
    assert np.all(counts > 0)          # 세 클래스 모두 나온다
    # 같은 시드 → 같은 장면 (재현성)
    a, la = SyntheticSceneRenderer(seed=9).render()
    b, lb = SyntheticSceneRenderer(seed=9).render()
    assert np.array_equal(a, b) and [x.bbox for x in la] == [x.bbox for x in lb]


def test_heavily_occluded_objects_are_dropped():
    cfg = RendererConfig(min_objects=6, max_objects=6, min_visible=1.1)  # 불가능한 가시 기준
    _, labels = SyntheticSceneRenderer(seed=1, config=cfg).render()
    assert labels == []


def test_generate_dataset_writes_yolo_layout(tmp_path):
    stats = synthetic.generate_synthetic_dataset(str(tmp_path), 6, 3, seed=2)
    assert stats['train_images'] == 6 and stats['val_images'] == 3
    for split, n in (('train', 6), ('val', 3)):
        assert len(os.listdir(tmp_path / 'images' / split)) == n
        assert len(os.listdir(tmp_path / 'labels' / split)) == n
    with open(tmp_path / 'data.yaml', 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    assert cfg['names'] == {0: 'box', 1: 'person', 2: 'sign'}
    assert cfg['train'] == 'images/train'
    lines = (tmp_path / 'labels' / 'train' / '000000.txt').read_text().split('\n')
    for ln in filter(None, lines):
        vals = [float(v) for v in ln.split()]
        assert int(vals[0]) in (0, 1, 2) and all(0.0 <= v <= 1.0 for v in vals[1:])


def test_generate_dataset_cli(tmp_path):
    spec = importlib.util.spec_from_file_location(
        'generate_dataset', os.path.join(SCRIPTS, 'generate_dataset.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.main(['synthetic', '--out', str(tmp_path), '--train', '2', '--val', '1']) == 0
    with open(tmp_path / 'stats.json', 'r', encoding='utf-8') as f:
        assert json.load(f)['train_images'] == 2
    with pytest.raises(SystemExit):
        mod.main([])                       # 원천(synthetic|gazebo) 필수


def test_project_object_and_truncation():
    obj = WorldObject('box', np.array([0.0, 0.0, 3.0]), np.array([0.5, 0.4, 0.3]))
    corners = synthetic.box_corners(obj)
    assert corners.shape == (8, 3)
    assert np.allclose(corners.mean(axis=0), obj.center)
    fx = fy = 300.0
    res = synthetic.project_object(corners, fx, fy, 320.0, 240.0, 640, 480)
    # 가장 가까운 면 z = 2.85 에서 가로 반폭 0.25 → u = 320 ± 300·0.25/2.85
    assert res.bbox[0] == pytest.approx(320 - 300 * 0.25 / 2.85)
    assert res.truncated == pytest.approx(0.0)
    assert res.depth == pytest.approx(3.0)
    edge = synthetic.box_corners(WorldObject('box', np.array([3.0, 0.0, 3.0]),
                                             np.array([0.5, 0.4, 0.3])))
    cut = synthetic.project_object(edge, fx, fy, 320.0, 240.0, 640, 480)
    assert 0.0 < cut.truncated < 1.0 and cut.bbox[2] == pytest.approx(639.0)
    behind = synthetic.box_corners(WorldObject('box', np.array([0.0, 0.0, 0.1]),
                                               np.array([0.5, 0.4, 0.3])))
    assert synthetic.project_object(behind, fx, fy, 320.0, 240.0, 640, 480) is None
    outside = synthetic.box_corners(WorldObject('box', np.array([50.0, 0.0, 3.0]),
                                                np.array([0.5, 0.4, 0.3])))
    assert synthetic.project_object(outside, fx, fy, 320.0, 240.0, 640, 480) is None
    rot = synthetic.box_corners(WorldObject('box', np.zeros(3), np.array([2.0, 0.0, 0.0]),
                                            yaw=math.pi / 2))
    assert np.allclose(np.abs(rot[:, 1]).max(), 1.0) and np.allclose(rot[:, 0], 0.0)


def test_occlusion_ratio():
    depth = np.full((100, 100), 5.0)
    depth[40:60, 40:50] = 1.0            # 중앙 절반을 가리는 기둥
    depth[40:45, 55:60] = np.nan
    bbox = (30.0, 30.0, 70.0, 70.0)      # 중앙 50 % = [40, 60) × [40, 60)
    assert synthetic.occlusion_ratio(depth, bbox, 5.0, 0.3) == pytest.approx(0.5)
    assert synthetic.occlusion_ratio(depth, bbox, 1.0, 0.3) == pytest.approx(0.0)
    assert synthetic.occlusion_ratio(depth, (200.0, 200.0, 210.0, 210.0), 5.0, 0.3) == 1.0


BOX_SDF = """<sdf version="1.8"><model name="box_medium"><link name="link">
<pose>0 0 0.15 0 0 0</pose><visual name="v"><geometry><box><size>0.50 0.40 0.30</size></box>
</geometry></visual></link></model></sdf>"""

WORLD_SDF = """<sdf version="1.8"><world name="w">
<include><uri>model://box_medium</uri><name>b1</name><pose>1 2 0.5 0 0 0.3</pose></include>
<include><uri>model://rack</uri><name>r1</name><pose>0 0 0 0 0 0</pose></include>
<include><uri>model://box_missing</uri><name>m</name><pose>0 0 0 0 0 0</pose></include>
<include><uri>file://abs/thing</uri></include>
<actor name="worker_a"><script><loop>true</loop><delay_start>0.0</delay_start>
<trajectory id="0" type="walk">
<waypoint><time>0</time><pose>0 0 0 0 0 0</pose></waypoint>
<waypoint><time>2</time><pose>2 0 0 0 0 1.0</pose></waypoint>
<waypoint><time>4</time><pose>2 2 0 0 0 1.5</pose></waypoint>
</trajectory></script></actor>
<actor name="static_actor"><pose>0 0 0 0 0 0</pose></actor>
</world></sdf>"""


def test_world_parsing_and_actor_interpolation(tmp_path):
    mdir = tmp_path / 'models' / 'box_medium'
    mdir.mkdir(parents=True)
    (mdir / 'model.sdf').write_text(BOX_SDF)
    objs, actors = world_objects.parse_world(WORLD_SDF, [str(tmp_path / 'models')])
    assert len(objs) == 1
    b = objs[0]
    assert b.class_name == 'box' and b.name == 'b1'
    assert np.allclose(b.center, [1.0, 2.0, 0.65]) and b.yaw == pytest.approx(0.3)
    assert np.allclose(b.size, [0.5, 0.4, 0.3])
    assert [a.name for a in actors] == ['worker_a']
    a = actors[0]
    p, yaw = a.pose_at(1.0)
    assert np.allclose(p, [1.0, 0.0, 0.0]) and yaw == pytest.approx(0.5)
    p, _ = a.pose_at(5.0)                 # loop: 5 mod 4 = 1
    assert np.allclose(p, [1.0, 0.0, 0.0])
    a.loop = False
    p, yaw = a.pose_at(10.0)
    assert np.allclose(p, [2.0, 2.0, 0.0]) and yaw == pytest.approx(1.5)
    p, _ = a.pose_at(0.0)
    assert np.allclose(p, [0.0, 0.0, 0.0])
    persons = world_objects.actor_objects(actors, 3.0)
    assert persons[0].class_name == 'person'
    assert np.allclose(persons[0].center, [2.0, 1.0, 0.875])
    assert world_objects.classify('box_large', world_objects.DEFAULT_CLASS_RULES) == 'box'
    assert world_objects.classify('stop_sign_2', world_objects.DEFAULT_CLASS_RULES) == 'sign'
    assert world_objects.classify('pillar', world_objects.DEFAULT_CLASS_RULES) is None
    assert world_objects.model_box('<sdf><model name="x"><link name="l"/></model></sdf>') is None
