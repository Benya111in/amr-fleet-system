"""scripts/dataset_capture.py 순수 도구 테스트: 구역 분할(완충대 — 분할 간 근사 중복 방지), 자유 공간 표본, 요청·라벨 형식."""

import importlib.util
import itertools
import math
import os
import random
import types

import numpy as np
import pytest

SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scripts',
                      'dataset_capture.py')


@pytest.fixture(scope='module')
def cap():
    spec = importlib.util.spec_from_file_location('dataset_capture', SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _rect_gap(a, b) -> float:
    dx = max(a[0] - b[1], b[0] - a[1], 0.0)
    dy = max(a[2] - b[3], b[2] - a[3], 0.0)
    return math.hypot(dx, dy)


def test_split_regions_are_disjoint_with_buffer(cap):
    # 서로 다른 분할의 카메라 위치는 3 m 이상 떨어진다 (회귀: 이전 셋은 한 자리 회전 프레임을 번갈아 val 로 보냈다)
    for (sa, ra), (sb, rb) in itertools.combinations(cap.SPLIT_REGIONS.items(), 2):
        for a in ra:
            for b in rb:
                assert _rect_gap(a, b) >= 3.0 - 1e-9, (sa, a, sb, b)
    assert cap.region_of(20.0, 0.0) == 'test'
    assert cap.region_of(-22.0, -16.0) == 'val'            # 충전소
    assert cap.region_of(0.0, 0.0) == 'train'
    assert cap.region_of(-20.0, 0.0) == 'train'
    assert cap.region_of(13.5, 0.0) is None                # train–test 완충대
    assert cap.region_of(-12.5, -10.0) is None             # train–val 완충대
    assert cap.region_of(-20.0, -4.5) is None
    assert abs(sum(cap.SPLIT_WEIGHTS.values()) - 1.0) < 1e-9


def _free_space(cap, clearance=0.5):
    grid = np.ones((200, 200), dtype=bool)          # 10 × 10 m, 해상도 0.05, 원점 (0, 0)
    grid[:, 95:105] = False                         # x 4.75–5.25 m 벽
    return cap.FreeSpace(grid, 0.05, (0.0, 0.0), clearance)


def test_free_space_clearance_and_sampling(cap):
    fs = _free_space(cap)
    assert fs.is_free(2.0, 5.0) and fs.is_free(8.0, 1.0)
    assert not fs.is_free(5.0, 5.0) and not fs.is_free(4.5, 5.0)     # 벽과 벽 옆 clearance
    assert not fs.is_free(-1.0, 5.0) and not fs.is_free(5.0, 11.0)   # 지도 밖
    rng = random.Random(0)
    for _ in range(200):
        x, y = fs.sample(rng)
        assert fs.is_free(x, y)


def test_free_space_from_map_yaml(cap, tmp_path):
    cv2 = pytest.importorskip('cv2')
    img = np.full((40, 60), 254, np.uint8)
    img[:, 30:32] = 0
    cv2.imwrite(str(tmp_path / 'm.pgm'), img)
    (tmp_path / 'm.yaml').write_text('image: m.pgm\nresolution: 0.1\norigin: [-3.0, -2.0, 0]\n'
                                     'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    fs = cap.FreeSpace.from_yaml(str(tmp_path / 'm.yaml'), 0.3)
    assert fs.is_free(-2.0, 0.0) and not fs.is_free(0.1, 0.0)      # 열 30 = x 0.0–0.1


def test_sample_pose_respects_region_hazards_and_separation(cap, monkeypatch):
    fs = _free_space(cap, 0.3)
    regions = {'train': [(0.0, 4.5, 0.0, 10.0)], 'test': [(5.5, 10.0, 0.0, 10.0)]}
    monkeypatch.setattr(cap, 'SPLIT_REGIONS', regions)
    rng = random.Random(1)
    targets = [('w', 2.0, 5.0, 1.0)]
    hazards = [(0.0, 4.5, 4.0, 6.0, 0.0)]
    placed = [(1.0, 1.0)]
    for _ in range(50):
        pose = cap.sample_pose(rng, fs, 'train', targets, placed, hazards, target_frac=0.5)
        assert pose is not None
        x, y, yaw, how = pose
        assert 0.0 <= x <= 4.5 and fs.is_free(x, y) and not 4.0 <= y <= 6.0
        assert math.hypot(x - 1.0, y - 1.0) >= 1.2 and -math.pi <= yaw <= math.pi
        assert how in ('random', 'target:w')
    assert cap.sample_pose(rng, fs, 'val', targets, [], [], tries=20) is None


def test_vehicle_hazards_and_pose_request(cap):
    hz = cap.vehicle_hazards({'forklift_main': (0.0, 15.0, 0.0), 'shuttle_amr': (9.0, 2.0, 1.57)})
    assert not cap.pose_is_safe(5.0, 15.5, [], hz)          # 지게차 진행 경로 앞
    assert not cap.pose_is_safe(9.5, 5.0, [], hz)           # 셔틀 경로
    assert cap.pose_is_safe(12.0, 15.0, [], hz)
    req = cap.pose_vector_request({'amr_01': (1.0, -2.0, math.pi / 2)})
    assert req.startswith('pose: [{name: "amr_01", position: {x: 1.0000, y: -2.0000')
    assert 'z: 0.707107, w: 0.707107' in req


def test_yolo_lines_and_label_record(cap):
    lab = types.SimpleNamespace(class_name='person', name='w', bbox=(100.0, 50.0, 140.0, 250.0),
                                visible=0.9, truncated=0.0, distance=3.2, pixels=900, kept=True,
                                reason='')
    rej = types.SimpleNamespace(**dict(vars(lab), kept=False, reason='occluded',
                                       class_name='box'))
    lines = cap.yolo_lines([lab, rej], 640, 480, ['box', 'person', 'sign', 'forklift', 'amr'])
    assert lines == ['1 0.187500 0.312500 0.062500 0.416667']
    rec = cap.label_record(rej)
    assert rec['cls'] == 'box' and rec['kept'] is False and rec['reason'] == 'occluded'


def test_write_data_yaml(cap, tmp_path):
    yaml = pytest.importorskip('yaml')
    path = cap.write_data_yaml(str(tmp_path), ['box', 'person'])
    cfg = yaml.safe_load(open(path, encoding='utf-8'))
    assert cfg['test'] == 'images/test' and cfg['names'] == {0: 'box', 1: 'person'}
