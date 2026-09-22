"""
yolo_node — YOLOv8 2D 객체 인식 (components.md §3.4 / §5.4, 명세 4.6).

Sub  camera/image_raw (sensor_msgs/Image, sensor QoS depth 1 — 추론이 밀리면 오래된 프레임은 버린다)
Pub  perception/detections_2d (vision_msgs/Detection2DArray, 스탬프 = 입력 이미지 스탬프)
     perception/detections_image (sensor_msgs/Image, publish_annotated 일 때 bbox 를 그린 디버그 영상)

클래스: config/classes.yaml 로 모델 클래스(COCO 또는 미세조정 가중치) → box / person / sign.
Detection2D.results[0].hypothesis.class_id = 출력 클래스 이름 (예 "box"), Detection2D.id = 출력 class_id.
처리율(FPS)·지연은 5 s 마다 로그로 남긴다 (scripts/yolo_fps_probe.py 가 ROS 쪽 FPS 를 따로 잰다).
"""

from __future__ import annotations

import os
import threading
import time
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .class_mapping import ClassMapper
from .yolo_backend import Detection, resolve_weights, YoloDetector


def share_dir() -> Optional[str]:
    try:
        return get_package_share_directory('amr_perception')
    except PackageNotFoundError:  # pragma: no cover - 설치 전 소스 실행
        return None


def default_path(*parts: str) -> str:
    base = share_dir()
    if base is None:
        base = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(base, *parts)


def to_detection_array(dets: List[Detection], header) -> Detection2DArray:
    """검출 목록 → vision_msgs/Detection2DArray."""
    out = Detection2DArray()
    out.header = header
    for d in dets:
        m = Detection2D()
        m.header = header
        m.id = str(d.class_id)
        m.bbox.center.position.x = d.cx
        m.bbox.center.position.y = d.cy
        m.bbox.center.theta = 0.0
        m.bbox.size_x = d.width
        m.bbox.size_y = d.height
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = d.class_name
        hyp.hypothesis.score = d.score
        m.results.append(hyp)
        out.detections.append(m)
    return out


def draw_detections(bgr: np.ndarray, dets: List[Detection]) -> np.ndarray:
    """디버그 영상: bbox + "class score"."""
    img = bgr.copy()
    colors = {'box': (40, 40, 230), 'person': (40, 200, 40), 'sign': (230, 80, 30)}
    for d in dets:
        x1, y1 = int(d.cx - d.width / 2), int(d.cy - d.height / 2)
        x2, y2 = int(d.cx + d.width / 2), int(d.cy + d.height / 2)
        c = colors.get(d.class_name, (0, 200, 200))
        cv2.rectangle(img, (x1, y1), (x2, y2), c, 2)
        cv2.putText(img, f'{d.class_name} {d.score:.2f}', (x1, max(y1 - 4, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv2.LINE_AA)
    return img


class YoloNode(Node):
    """camera/image_raw → YOLOv8 → perception/detections_2d."""

    def __init__(self, detector: Optional[YoloDetector] = None, **kwargs):
        super().__init__('yolo_node', **kwargs)
        p = self.declare_parameter
        p('weights', 'yolov8n_warehouse.pt')
        p('fallback_weights', 'yolov8n.pt')
        p('device', 'auto')
        p('imgsz', 640)
        p('cpu_imgsz', 320)
        p('conf_thresh', 0.35)
        p('iou', 0.5)
        p('half', True)
        p('max_det', 50)
        p('torch_threads', 2)
        p('cpu_threads', 2)
        p('fast_path', True)
        p('async_inference', False)
        p('classes_file', '')
        p('publish_annotated', False)
        p('log_detections', True)
        p('stats_period', 5.0)
        p('topics.image', 'camera/image_raw')
        p('topics.detections', 'perception/detections_2d')
        p('topics.annotated', 'perception/detections_image')
        gp = self.get_parameter

        self.bridge = CvBridge()
        if detector is None:
            classes_file = gp('classes_file').value or default_path('config', 'classes.yaml')
            mapper = ClassMapper.from_yaml(classes_file)
            search = [default_path('models'), os.getcwd()]
            weights = resolve_weights(str(gp('weights').value), search)
            if not os.path.exists(weights):
                fallback = resolve_weights(str(gp('fallback_weights').value), search)
                self.get_logger().warn(
                    f'가중치 {weights} 없음 → {fallback} 로 대체. COCO 가중치는 창고 월드의 상자·표지판을 '
                    '검출하지 못한다 (perception.md §8.3) — 저장소의 models/yolov8n_warehouse.pt 가 '
                    '설치됐는지 확인하고, 재생성은 models/README.md')
                weights = fallback
            detector = YoloDetector(
                weights, str(gp('device').value), int(gp('imgsz').value),
                int(gp('cpu_imgsz').value), float(gp('conf_thresh').value),
                float(gp('iou').value), bool(gp('half').value), int(gp('max_det').value),
                mapper, int(gp('torch_threads').value), int(gp('cpu_threads').value),
                bool(gp('fast_path').value))
            warm = detector.warmup()
            self.get_logger().info(
                f'YOLO 가중치 {weights}, 장치 {detector.device} (imgsz {detector.imgsz}, '
                f'half {detector.half}), 예열 추론 {warm * 1e3:.1f} ms, 모델 클래스 → 출력 '
                f'{mapper.bind(detector.model_names)}')
            if not detector.on_gpu:
                self.get_logger().warn('GPU 를 쓸 수 없어 CPU 백엔드로 동작한다 (명세 대안 10 FPS 목표)')
        self.detector = detector
        self.publish_annotated = bool(gp('publish_annotated').value)
        self.log_detections = bool(gp('log_detections').value)
        self.stats_period = float(gp('stats_period').value)

        sensor_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Detection2DArray, gp('topics.detections').value, 10)
        self.annotated_pub = (self.create_publisher(Image, gp('topics.annotated').value, 1)
                              if self.publish_annotated else None)
        self.sub = self.create_subscription(Image, gp('topics.image').value, self.on_image,
                                            sensor_qos)
        self._count = 0
        self._infer_sum = 0.0
        self._latency_sum = 0.0
        self._window_start = time.monotonic()
        # 2 단 파이프라인: 실행기 스레드는 수신(역직렬화)만, 작업 스레드가 최신 1 장을 추론한다.
        # torch 가 CUDA 연산 동안 GIL 을 놓으므로 수신과 추론이 겹친다 (오래된 프레임은 덮어써 버린다).
        self._slot: Optional[Image] = None
        self._cv = threading.Condition()
        self._running = True
        self._worker = None
        if bool(gp('async_inference').value):
            self._worker = threading.Thread(target=self._work, name='yolo_infer', daemon=True)
            self._worker.start()

    def process(self, msg: Image) -> Detection2DArray:
        """이미지 메시지 한 장 → 검출 메시지 (발행은 호출자가)."""
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        t0 = time.perf_counter()
        dets = self.detector.infer(bgr)
        self._infer_sum += time.perf_counter() - t0
        out = to_detection_array(dets, msg.header)
        if self.annotated_pub is not None:
            vis = self.bridge.cv2_to_imgmsg(draw_detections(bgr, dets), encoding='bgr8')
            vis.header = msg.header
            self.annotated_pub.publish(vis)
        if self.log_detections:
            for d in dets:
                self.get_logger().info(
                    f'Detected: {d.class_name.capitalize()} (id={d.class_id}), '
                    f'Confidence: {d.score:.2f}', throttle_duration_sec=2.0)
        return out

    def on_image(self, msg: Image) -> None:
        if self._worker is None:
            self.handle_image(msg)
            return
        with self._cv:
            self._slot = msg          # 추론 중 도착한 이전 프레임은 버린다 (최신 우선)
            self._cv.notify()

    def _work(self) -> None:
        while True:
            with self._cv:
                while self._slot is None and self._running:
                    self._cv.wait(timeout=0.5)
                if not self._running:
                    return
                msg, self._slot = self._slot, None
            try:
                self.handle_image(msg)
            except Exception as exc:  # noqa: BLE001 - 작업 스레드가 죽지 않게 기록만
                self.get_logger().error(f'추론 실패: {exc}', throttle_duration_sec=5.0)

    def stop(self) -> None:
        """작업 스레드 종료."""
        with self._cv:
            self._running = False
            self._cv.notify()
        if self._worker is not None:
            self._worker.join(timeout=2.0)

    def handle_image(self, msg: Image) -> None:
        """추론 + 발행 + 통계."""
        out = self.process(msg)
        self.pub.publish(out)
        now = self.get_clock().now()
        stamp = Time.from_msg(msg.header.stamp, clock_type=now.clock_type)
        self._latency_sum += max((now - stamp).nanoseconds * 1e-9, 0.0)
        self._count += 1
        elapsed = time.monotonic() - self._window_start
        if elapsed >= self.stats_period and self._count > 0:
            self.get_logger().info(
                f'처리 {self._count / elapsed:.1f} FPS, 추론 평균 '
                f'{self._infer_sum / self._count * 1e3:.1f} ms, 이미지→발행 지연 평균 '
                f'{self._latency_sum / self._count * 1e3:.1f} ms')
            self._count = 0
            self._infer_sum = 0.0
            self._latency_sum = 0.0
            self._window_start = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
