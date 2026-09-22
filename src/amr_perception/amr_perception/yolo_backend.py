"""
YOLOv8 추론 래퍼 (ultralytics). ROS 비의존 — yolo_node 와 벤치/학습 도구가 공유한다.

- 가중치 경로 해석: 절대 경로 → 패키지 share/models → $ROS_WS/src/amr_perception/models → 이름 그대로
- 장치 선택: auto(CUDA 가능하면 cuda:0, 아니면 cpu) / cuda / cpu. CPU 는 명세 대안(10 FPS) 경로로
  cpu_imgsz(기본 320, 과부하 호스트 폴백 256 — perception.yaml 근거) 를 쓴다.
- 결과는 원본 이미지 픽셀 좌표의 (중심, 크기) 와 출력 클래스(ClassMapper) 로 돌려준다.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
import time
from typing import List, Optional, Sequence

import cv2
import numpy as np

from .class_mapping import ClassMapper

# OpenMP 스레드가 일감을 기다리며 바쁜 대기(spin)하면 과부하 호스트에서 CPU 추론이 수십 배 느려진다
# (측정: YOLOv8n 320 CPU 중앙값 6.3 s → PASSIVE 158 ms, load avg ≈ 110/32). torch import 전에 정해야 한다.
os.environ.setdefault('OMP_WAIT_POLICY', 'PASSIVE')


@dataclass
class Detection:
    """원본 이미지 픽셀 좌표의 2D 검출."""

    class_id: int
    class_name: str
    score: float
    cx: float
    cy: float
    width: float
    height: float
    model_class_id: int = -1


def resolve_weights(path: str, search_dirs: Sequence[str] = ()) -> str:
    """가중치 파일 경로 해석. 찾지 못하면 입력을 그대로 돌려준다 (ultralytics 가 이름으로 처리)."""
    if os.path.isabs(path) and os.path.exists(path):
        return path
    candidates = [os.path.join(d, path) for d in search_dirs]
    candidates.append(os.path.join(os.environ.get('ROS_WS', '/ros2_ws'), 'src', 'amr_perception',
                                   'models', os.path.basename(path)))
    candidates.append(os.path.abspath(path))
    for c in candidates:
        if os.path.exists(c):
            return c
    return path


def select_device(requested: str) -> str:
    """'auto' | 'cuda' | 'cuda:N' | 'cpu' → 실제 장치 문자열."""
    req = (requested or 'auto').lower()
    if req == 'cpu':
        return 'cpu'
    try:
        import torch
        available = bool(torch.cuda.is_available())
    except ImportError:  # pragma: no cover - 이미지에는 torch 가 있다
        available = False
    if req == 'auto':
        return 'cuda:0' if available else 'cpu'
    if req.startswith('cuda'):
        if not available:
            return 'cpu'
        return req if ':' in req else 'cuda:0'
    return req


def letterbox_params(h: int, w: int, imgsz: int, stride: int = 32):
    """
    레터박스 기하: 긴 변을 imgsz 로 맞춘 배율 r, 새 크기, stride 배수가 되도록 가운데 패딩 (좌, 상, 우, 하).

    ultralytics LetterBox(auto=True) 와 같은 규칙 — 640×480 을 640 으로 하면 r = 1, 패딩 0 (항등).
    """
    r = imgsz / max(h, w)
    nw, nh = int(round(w * r)), int(round(h * r))
    pw, ph = (stride - nw % stride) % stride, (stride - nh % stride) % stride
    return r, nw, nh, pw // 2, ph // 2, pw - pw // 2, ph - ph // 2


def _nms_function():
    """버전(ultralytics 8.3 / 8.4)에 따라 위치가 다른 NMS 함수."""
    try:
        from ultralytics.utils.nms import non_max_suppression
    except ImportError:  # pragma: no cover - 8.3 이하
        from ultralytics.utils.ops import non_max_suppression
    return non_max_suppression


class YoloDetector:
    """
    ultralytics YOLO 모델 + 클래스 매핑.

    직접 경로(기본): ultralytics predict() 래퍼 대신 cv2 레터박스(긴 변 = imgsz, stride 32 배수로 가운데
    패딩 114) → 융합 모델 forward (GPU FP16 / CPU FP32) → NMS → 레터박스 역변환. 640×480 카메라를 GPU
    imgsz 640 으로 돌리면 레터박스가 항등이라 전처리가 텐서 변환뿐이다 (GPU 추론 중앙값 1.90 ms vs predict()
    2.12 ms). CPU 는 부하에 따라 우열이 바뀐다: 과부하(load ≈ 90/32) 종단 FPS 는 직접 경로가 높았고(8.0 vs 5.0),
    정상 부하 마이크로 벤치에서는 predict() 가 빨랐다(9.1 vs 6.3 ms, 둘 다 10 FPS 목표의 10 배 이상) — perception.md
    §8.2. torch 스레드 수를 제한해 과부하 호스트의 스레드 경합을 줄인다 (32 스레드 21 ms → 2 스레드 8 ms).
    fast_path=False 면 predict() 경로.
    """

    def __init__(self, weights: str, device: str = 'auto', imgsz: int = 640,
                 cpu_imgsz: int = 320, conf: float = 0.35, iou: float = 0.5,
                 half: bool = True, max_det: int = 50,
                 mapper: Optional[ClassMapper] = None, torch_threads: int = 2,
                 cpu_threads: int = 2, fast_path: bool = True):
        import torch
        from ultralytics import YOLO  # 무거운 의존은 생성 시점에만

        self.device = select_device(device)
        self.on_gpu = self.device.startswith('cuda')
        threads = torch_threads if self.on_gpu else cpu_threads
        if threads > 0:
            torch.set_num_threads(int(threads))
        self.imgsz = int(imgsz if self.on_gpu else cpu_imgsz)
        self.conf = float(conf)
        self.iou = float(iou)
        self.half = bool(half) and self.on_gpu
        self.max_det = int(max_det)
        self.weights = weights
        self.model = YOLO(weights)
        self.model_names = {int(k): str(v) for k, v in self.model.names.items()}
        self.mapper = mapper
        self.class_filter: Optional[List[int]] = None
        if mapper is not None:
            mapper.bind(self.model_names)
            self.class_filter = mapper.model_ids() or None
        self._torch = torch
        self._net = None
        if fast_path:
            net = self.model.model.to(self.device).eval()
            if hasattr(net, 'fuse'):
                net = net.fuse(verbose=False)      # Conv+BN 융합 (predict() 와 같은 최적화)
            self._net = net.half() if self.half else net.float()
            self._nms = _nms_function()
        self.last_path = ''

    def warmup(self, shape=(480, 640, 3), runs: int = 2) -> float:
        """첫 추론의 CUDA 초기화 비용을 미리 치른다. 반환: 마지막 추론 시간 [s]."""
        dummy = np.zeros(shape, dtype=np.uint8)
        dt = 0.0
        for _ in range(max(runs, 1)):
            t0 = time.perf_counter()
            self.infer(dummy)
            dt = time.perf_counter() - t0
        return dt

    def _infer_fast(self, bgr: np.ndarray) -> np.ndarray:
        """직접 경로 → (n, 6) [x1, y1, x2, y2, score, model_cls] (입력 픽셀 좌표)."""
        torch = self._torch
        h, w = bgr.shape[:2]
        r, nw, nh, left, top, right, bottom = letterbox_params(h, w, self.imgsz)
        img = bgr
        if (nw, nh) != (w, h):
            img = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        if left or top or right or bottom:
            img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT,
                                     value=(114, 114, 114))
        with torch.inference_mode():
            x = torch.from_numpy(np.ascontiguousarray(img)).to(self.device, non_blocking=True)
            # BGR HWC → RGB NCHW. permute 만 하면 채널 마지막(NHWC) 보폭이 남아 CPU(oneDNN) 합성곱이 느린 경로를
            # 탄다 → contiguous 로 NCHW 연속 메모리를 만든다 (perception.md §8 마이크로 벤치)
            x = x.permute(2, 0, 1).flip(0).unsqueeze(0).contiguous()
            x = (x.half() if self.half else x.float()).div_(255.0)
            y = self._net(x)
            y = y[0] if isinstance(y, (list, tuple)) else y
            det = self._nms(y, self.conf, self.iou, classes=self.class_filter,
                            max_det=self.max_det)[0]
            out = det[:, :6].float().cpu().numpy()
        out[:, [0, 2]] = (out[:, [0, 2]] - left) / r
        out[:, [1, 3]] = (out[:, [1, 3]] - top) / r
        return out

    def _infer_predict(self, bgr: np.ndarray) -> np.ndarray:
        kwargs = dict(imgsz=self.imgsz, conf=self.conf, iou=self.iou, device=self.device,
                      max_det=self.max_det, classes=self.class_filter, verbose=False)
        if self.half:
            kwargs['half'] = True
        results = self.model.predict(bgr, **kwargs)
        if not results or results[0].boxes is None or len(results[0].boxes) == 0:
            return np.zeros((0, 6))
        b = results[0].boxes
        return np.column_stack([b.xyxy.cpu().numpy(), b.conf.cpu().numpy(),
                                b.cls.cpu().numpy()])

    def infer(self, bgr: np.ndarray) -> List[Detection]:
        """BGR 이미지 한 장 → 출력 클래스로 매핑된 검출 목록 (점수 내림차순)."""
        if self._net is not None:
            self.last_path = 'fast'
            rows = self._infer_fast(bgr)
        else:
            self.last_path = 'predict'
            rows = self._infer_predict(bgr)
        out: List[Detection] = []
        for x1, y1, x2, y2, s, mid in rows:
            if self.mapper is not None:
                mapped = self.mapper.map(int(mid), float(s))
                if mapped is None:
                    continue
                cid, name = mapped
            else:
                cid, name = int(mid), self.model_names.get(int(mid), str(int(mid)))
            out.append(Detection(cid, name, float(s), 0.5 * float(x1 + x2), 0.5 * float(y1 + y2),
                                 float(x2 - x1), float(y2 - y1), int(mid)))
        out.sort(key=lambda d: -d.score)
        return out
