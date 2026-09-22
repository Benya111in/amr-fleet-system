"""
YOLOv8 래퍼 테스트: 가중치 경로·장치 선택, 저장소의 미세조정 가중치로 CPU/GPU 실추론과 클래스 매핑.

가중치(models/yolov8n_warehouse.pt)는 저장소에 들어 있으므로 깨끗한 체크아웃에서도 건너뛰지 않는다 (리뷰 회귀:
이전에는 models/ 에 .gitkeep 만 있어 실추론 테스트 5 개가 모두 skip 이었다).
"""

import glob
import os

from amr_perception.class_mapping import ClassMapper
from amr_perception.yolo_backend import resolve_weights, select_device
import cv2
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEIGHTS = os.path.join(PKG, 'models', 'yolov8n_warehouse.pt')
CLASSES_YAML = os.path.join(PKG, 'config', 'classes.yaml')
HELDOUT = sorted(glob.glob(os.path.join(PKG, 'models', 'heldout', 'images', '*.jpg')))


def test_finetuned_weights_are_provisioned():
    assert os.path.getsize(WEIGHTS) < 20 * 1024 * 1024       # 저장소 허용 크기
    assert len(HELDOUT) >= 6


def _node(**params):
    from amr_perception.yolo_node import YoloNode
    from rclpy.parameter import Parameter
    return YoloNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])


def test_yolo_node_loads_provisioned_weights_by_default():
    import rclpy
    rclpy.init()
    try:
        # weights 를 주지 않는다 → 기본값 yolov8n_warehouse.pt 가 share/models 또는 $ROS_WS 에서 풀려야 한다
        node = _node(device='cpu', classes_file=CLASSES_YAML)
        assert os.path.basename(node.detector.weights) == 'yolov8n_warehouse.pt'
        assert os.path.exists(node.detector.weights)
        assert node.detector.device == 'cpu' and node.detector.imgsz == 320
        names = node.detector.model_names
        assert [names[i] for i in sorted(names)] == ['box', 'person', 'sign', 'forklift', 'amr']
        assert node.detector.class_filter == [0, 1, 2]     # classes.yaml 이 box/person/sign 만 출력
        node.stop()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_yolo_node_falls_back_when_weights_missing():
    import rclpy
    rclpy.init()
    try:
        node = _node(weights='no_such_finetuned_weights.pt', fallback_weights=WEIGHTS,
                     device='cpu', classes_file=CLASSES_YAML)
        assert node.detector.weights == WEIGHTS
        node.stop()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_resolve_weights(tmp_path):
    f = tmp_path / 'w.pt'
    f.write_bytes(b'x')
    assert resolve_weights(str(f)) == str(f)
    assert resolve_weights('w.pt', [str(tmp_path)]) == str(f)
    assert resolve_weights('does_not_exist.pt', [str(tmp_path)]) == 'does_not_exist.pt'


def test_select_device():
    assert select_device('cpu') == 'cpu'
    auto = select_device('auto')
    assert auto in ('cpu', 'cuda:0')
    assert select_device('cuda') in ('cpu', 'cuda:0')
    assert select_device('cuda:1') in ('cpu', 'cuda:1')
    assert select_device('mps') == 'mps'


def test_cpu_inference_with_class_mapping():
    from amr_perception.yolo_backend import YoloDetector
    mapper = ClassMapper.from_yaml(CLASSES_YAML)
    det = YoloDetector(WEIGHTS, device='cpu', cpu_imgsz=320, conf=0.25, mapper=mapper)
    assert det.device == 'cpu' and det.imgsz == 320 and not det.half
    assert det.class_filter == [0, 1, 2]
    assert det.warmup(runs=1) > 0.0
    seen = set()
    for path in HELDOUT:
        out = det.infer(cv2.imread(path))
        for d in out:
            assert d.class_name in ('box', 'person', 'sign')
            assert 0.0 <= d.score <= 1.0 and d.width > 0 and d.height > 0
            assert d.class_id == ('box', 'person', 'sign').index(d.class_name)
        assert [d.score for d in out] == sorted((d.score for d in out), reverse=True)
        seen |= {d.class_name for d in out}
    assert seen == {'box', 'person', 'sign'}
    raw = YoloDetector(WEIGHTS, device='cpu', cpu_imgsz=320, conf=0.25)
    assert raw.class_filter is None
    for d in raw.infer(cv2.imread(HELDOUT[0])):
        assert d.class_name == raw.model_names[d.model_class_id]


def test_letterbox_params():
    from amr_perception.yolo_backend import letterbox_params
    assert letterbox_params(480, 640, 640) == (1.0, 640, 480, 0, 0, 0, 0)     # 카메라: 항등
    r, nw, nh, left, top, right, bottom = letterbox_params(480, 640, 320)
    assert (r, nw, nh) == (0.5, 320, 240) and (top, bottom) == (8, 8) and left == right == 0
    r, nw, nh, left, top, right, bottom = letterbox_params(200, 300, 320)
    assert (nw + left + right) % 32 == 0 and (nh + top + bottom) % 32 == 0


@pytest.mark.parametrize('device', ['cpu', 'cuda'])
def test_direct_path_matches_predict(device):
    import torch
    if device == 'cuda' and not torch.cuda.is_available():
        pytest.skip('CUDA 없음 (CPU 경우는 항상 실행)')
    from amr_perception.yolo_backend import YoloDetector
    fast = YoloDetector(WEIGHTS, device=device, cpu_imgsz=320, conf=0.05, half=False)
    slow = YoloDetector(WEIGHTS, device=device, cpu_imgsz=320, conf=0.05, half=False,
                        fast_path=False)
    assert fast.imgsz == (640 if device == 'cuda' else 320)
    matched = 0
    for path in HELDOUT[:4]:
        img = cv2.imread(path)
        for crop in (img, img[:200, :300]):          # 항등 레터박스 / 축소·패딩 레터박스
            a, b = fast.infer(crop), slow.infer(crop)
            assert fast.last_path == 'fast' and slow.last_path == 'predict'
            for db in b:
                if db.score < 0.3:
                    continue
                assert any(da.class_name == db.class_name and abs(da.cx - db.cx) < 4.0 and
                           abs(da.cy - db.cy) < 4.0 and abs(da.score - db.score) < 0.05
                           for da in a), (device, path, db)
                matched += 1
    assert matched >= 5        # 보류 프레임의 상자·사람·표지판 — 실제로 비교가 일어났는지
