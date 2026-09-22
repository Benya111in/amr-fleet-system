"""YOLOv8 래퍼 테스트: 가중치 경로·장치 선택, (가중치가 있으면) CPU 실추론과 클래스 매핑."""

import os

from amr_perception.class_mapping import ClassMapper
from amr_perception.synthetic import SyntheticSceneRenderer
from amr_perception.yolo_backend import resolve_weights, select_device
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEIGHTS = os.path.join(PKG, 'models', 'yolov8n.pt')


@pytest.mark.skipif(not os.path.exists(WEIGHTS), reason='models/yolov8n.pt 없음')
def test_yolo_node_loads_real_model_on_cpu():
    pytest.importorskip('ultralytics')
    from amr_perception.yolo_node import YoloNode
    import rclpy
    from rclpy.parameter import Parameter
    rclpy.init()
    try:
        node = YoloNode(parameter_overrides=[
            Parameter('weights', value=WEIGHTS), Parameter('device', value='cpu'),
            Parameter('classes_file', value=os.path.join(PKG, 'config', 'classes.yaml'))])
        assert node.detector.device == 'cpu' and node.detector.imgsz == 320
        assert node.detector.class_filter == [0, 11, 28]
        node.stop()
        node.destroy_node()
    finally:
        rclpy.shutdown()


@pytest.mark.skipif(not os.path.exists(WEIGHTS), reason='models/yolov8n.pt 없음')
def test_yolo_node_falls_back_when_finetuned_weights_missing():
    pytest.importorskip('ultralytics')
    from amr_perception.yolo_node import YoloNode
    import rclpy
    from rclpy.parameter import Parameter
    rclpy.init()
    try:
        node = YoloNode(parameter_overrides=[
            Parameter('weights', value='no_such_finetuned_weights.pt'),
            Parameter('fallback_weights', value=WEIGHTS), Parameter('device', value='cpu'),
            Parameter('classes_file', value=os.path.join(PKG, 'config', 'classes.yaml'))])
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


@pytest.mark.skipif(not os.path.exists(WEIGHTS),
                    reason='models/yolov8n.pt 없음 (scripts/download_models.sh)')
def test_cpu_inference_with_class_mapping():
    pytest.importorskip('ultralytics')
    from amr_perception.yolo_backend import YoloDetector
    mapper = ClassMapper.from_yaml(os.path.join(PKG, 'config', 'classes.yaml'))
    det = YoloDetector(WEIGHTS, device='cpu', cpu_imgsz=320, conf=0.05, mapper=mapper)
    assert det.device == 'cpu' and det.imgsz == 320 and not det.half
    assert det.class_filter == [0, 11, 28]
    assert det.warmup(runs=1) > 0.0
    img, _ = SyntheticSceneRenderer(seed=4).render()
    out = det.infer(img)
    for d in out:
        assert d.class_name in ('box', 'person', 'sign')
        assert 0.0 <= d.score <= 1.0 and d.width > 0 and d.height > 0
    assert [d.score for d in out] == sorted((d.score for d in out), reverse=True)
    raw = YoloDetector(WEIGHTS, device='cpu', cpu_imgsz=320, conf=0.05)
    assert raw.class_filter is None
    for d in raw.infer(img):
        assert d.class_name == raw.model_names[d.model_class_id]


def test_letterbox_params():
    from amr_perception.yolo_backend import letterbox_params
    assert letterbox_params(480, 640, 640) == (1.0, 640, 480, 0, 0, 0, 0)     # 카메라: 항등
    r, nw, nh, left, top, right, bottom = letterbox_params(480, 640, 320)
    assert (r, nw, nh) == (0.5, 320, 240) and (top, bottom) == (8, 8) and left == right == 0
    r, nw, nh, left, top, right, bottom = letterbox_params(200, 300, 320)
    assert (nw + left + right) % 32 == 0 and (nh + top + bottom) % 32 == 0


@pytest.mark.skipif(not os.path.exists(WEIGHTS), reason='models/yolov8n.pt 없음')
@pytest.mark.parametrize('device', ['cpu', 'cuda'])
def test_direct_path_matches_predict(device):
    torch = pytest.importorskip('torch')
    if device == 'cuda' and not torch.cuda.is_available():
        pytest.skip('CUDA 없음')
    from amr_perception.yolo_backend import YoloDetector
    fast = YoloDetector(WEIGHTS, device=device, cpu_imgsz=320, conf=0.05, half=False)
    slow = YoloDetector(WEIGHTS, device=device, cpu_imgsz=320, conf=0.05, half=False,
                        fast_path=False)
    assert fast.imgsz == (640 if device == 'cuda' else 320)
    matched = 0
    for seed in (1, 4, 7):
        img, _ = SyntheticSceneRenderer(seed=seed).render()
        for crop in (img, img[:200, :300]):          # 항등 레터박스 / 축소·패딩 레터박스
            a, b = fast.infer(crop), slow.infer(crop)
            assert fast.last_path == 'fast' and slow.last_path == 'predict'
            for db in b:
                if db.score < 0.15:
                    continue
                assert any(da.class_name == db.class_name and abs(da.cx - db.cx) < 4.0 and
                           abs(da.cy - db.cy) < 4.0 and abs(da.score - db.score) < 0.05
                           for da in a), (device, seed, db)
                matched += 1
    assert matched >= 1        # 합성 장면의 사람 실루엣 등 — 실제로 비교가 일어났는지
