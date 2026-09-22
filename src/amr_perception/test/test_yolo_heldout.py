"""
회귀: 저장소 가중치가 보류 구역(test, x ≥ 15 m)의 Gazebo 프레임에서 box / person / sign 을 모두 검출하는가.

리뷰 회귀 — 이전 미세조정 가중치는 현재 월드에서 사람 0/175, 표지판 0/269 였다. 프레임과 라벨은
scripts/dataset_capture.py (깊이 검증 지면 진실) 가 만든 test 분할에서 yolo_train_eval.py heldout 으로 뽑은 것이다
(models/heldout, 학습·검증에 쓰지 않은 구역). yolo_node 와 같은 경로(YoloDetector + classes.yaml 매핑)로 돌린다.
"""

import glob
import importlib.util
import os

from amr_perception.class_mapping import ClassMapper
from amr_perception.world_objects import match_detections
from amr_perception.yolo_backend import YoloDetector
import cv2
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEIGHTS = os.path.join(PKG, 'models', 'yolov8n_warehouse.pt')
HELDOUT = os.path.join(PKG, 'models', 'heldout')
OUT_CLASSES = ('box', 'person', 'sign')
DATASET_CLASSES = ('box', 'person', 'sign', 'forklift', 'amr')
# 최소 재현율 (보류 프레임, conf 0.35, IoU ≥ 0.5, GT 짧은 변 ≥ 16 px). 실측은 docs/algorithms/perception.md §8.3.
MIN_RECALL = {'cuda': 0.85, 'cpu': 0.8}


@pytest.fixture(scope='module')
def tool():
    spec = importlib.util.spec_from_file_location(
        'yolo_train_eval', os.path.join(PKG, 'scripts', 'yolo_train_eval.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _recall(device: str, tool) -> dict:
    mapper = ClassMapper.from_yaml(os.path.join(PKG, 'config', 'classes.yaml'))
    det = YoloDetector(WEIGHTS, device=device, conf=0.35, mapper=mapper)
    stats = {c: [0, 0, 0] for c in OUT_CLASSES}          # gt, tp, fp
    for path in sorted(glob.glob(os.path.join(HELDOUT, 'images', '*.jpg'))):
        img = cv2.imread(path)
        h, w = img.shape[:2]
        stem = os.path.splitext(os.path.basename(path))[0]
        gts = [(DATASET_CLASSES[c], b, tool.min_side_px(b) >= 16.0)
               for c, b in tool.read_labels(os.path.join(HELDOUT, 'labels', stem + '.txt'), w, h)
               if DATASET_CLASSES[c] in OUT_CLASSES]
        dets = [(d.class_name, (d.cx - d.width / 2, d.cy - d.height / 2,
                                d.cx + d.width / 2, d.cy + d.height / 2), d.score)
                for d in det.infer(img)]
        gm, dm = match_detections(gts, dets)
        for (cls, _, req), m in zip(gts, gm):
            if req:
                stats[cls][0] += 1
                stats[cls][1] += m >= 0
        for (cls, _, _), m in zip(dets, dm):
            stats[cls][2] += m < 0
    return stats


@pytest.mark.parametrize('device', ['cpu', 'cuda'])
def test_heldout_frames_detect_all_three_classes(device, tool):
    torch = pytest.importorskip('torch')
    if device == 'cuda' and not torch.cuda.is_available():
        pytest.skip('CUDA 없음 (CPU 경우는 항상 실행)')
    stats = _recall(device, tool)
    for cls, (gt, tp, fp) in stats.items():
        assert gt >= 3, (cls, stats)
        assert tp / gt >= MIN_RECALL[device], (device, cls, stats)
        assert fp <= max(2, gt // 2), (device, cls, stats)


def test_train_eval_label_tools(tool, tmp_path):
    (tmp_path / 'images' / 'test').mkdir(parents=True)
    (tmp_path / 'labels' / 'test').mkdir(parents=True)
    img = cv2.imread(sorted(glob.glob(os.path.join(HELDOUT, 'images', '*.jpg')))[0])
    for i, lines in enumerate(['1 0.5 0.5 0.1 0.4\n0 0.2 0.2 0.02 0.02\n', '']):
        cv2.imwrite(str(tmp_path / 'images' / 'test' / f'r_{i:05d}.jpg'), img)
        (tmp_path / 'labels' / 'test' / f'r_{i:05d}.txt').write_text(lines)
    data = tmp_path / 'data.yaml'
    data.write_text(f'path: {tmp_path}\ntrain: images/test\nval: images/test\ntest: images/test\n'
                    'names: {0: box, 1: person, 2: sign}\n')
    img_dir, lbl_dir, names = tool.split_dirs(str(data), 'test')
    assert names == ['box', 'person', 'sign'] and lbl_dir.endswith(os.path.join('labels', 'test'))
    labs = tool.read_labels(os.path.join(lbl_dir, 'r_00000.txt'), 640, 480)
    assert labs[0][0] == 1 and labs[0][1] == pytest.approx((288.0, 144.0, 352.0, 336.0))
    assert tool.read_labels(str(tmp_path / 'missing.txt'), 640, 480) == []
    counts = tool.count_labels(str(data))
    assert counts['test'] == {'images': 2, 'empty': 1, 'box': 1, 'person': 1, 'sign': 0}
    res = tool.cmd_heldout(type('A', (), {'data': str(data), 'out': str(tmp_path / 'held'),
                                          'per_class': 1, 'classes': 3, 'min_px': 16.0})())
    assert res['images'] == ['r_00000.jpg']          # box 는 16 px 미만, person 만 고른다
    op = tool.operating_point(WEIGHTS, str(data), 'test', 'cpu', 0.35, 320)
    assert op['per_class']['person']['gt'] == 1 and op['imgsz'] == 320
