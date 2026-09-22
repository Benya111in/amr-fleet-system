"""
YOLO 미세조정용 데이터셋 도구 (ROS 비의존).

1) 합성 렌더러(SyntheticSceneRenderer): Gazebo 없이 cv2 도형으로 창고풍 장면을 그려 box / person /
   sign 라벨을 만든다 (파이프라인 검증·폴백용). 화가 알고리즘으로 그리고 id 마스크로 가시 비율을
   세어 많이 가려진 물체는 라벨에서 뺀다.
2) Gazebo 지면 진실 투영: 월드 SDF 의 모델 자세 + 크기(3D 박스 8 꼭짓점)를 카메라 자세로 투영해
   2D 박스를 만들고, 깊이 이미지로 가림 비율을 검사한다 (scripts/generate_dataset.py gazebo).
3) YOLO 형식 쓰기: images/{train,val}/*.jpg, labels/{train,val}/*.txt ("cls cx cy w h", 0~1 정규화),
   data.yaml.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

CLASSES = ['box', 'person', 'sign']

BBox = Tuple[float, float, float, float]  # x1, y1, x2, y2 [px]


@dataclass
class Label:
    """한 물체 라벨."""

    class_id: int
    bbox: BBox


def yolo_label_line(label: Label, width: int, height: int) -> str:
    """YOLO txt 한 줄: "cls cx cy w h" (이미지 크기로 정규화, 소수 6자리)."""
    x1, y1, x2, y2 = label.bbox
    cx = 0.5 * (x1 + x2) / width
    cy = 0.5 * (y1 + y2) / height
    w = (x2 - x1) / width
    h = (y2 - y1) / height
    return f'{label.class_id} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}'


def parse_label_line(line: str, width: int, height: int) -> Label:
    """yolo_label_line 의 역."""
    c, cx, cy, w, h = line.split()
    cx, cy, w, h = float(cx) * width, float(cy) * height, float(w) * width, float(h) * height
    return Label(int(c), (cx - 0.5 * w, cy - 0.5 * h, cx + 0.5 * w, cy + 0.5 * h))


# ------------------------------------------------------------------ 합성 렌더러

@dataclass
class RendererConfig:
    """합성 장면 파라미터."""

    width: int = 640
    height: int = 480
    min_objects: int = 1
    max_objects: int = 5
    class_weights: Tuple[float, float, float] = (0.45, 0.35, 0.20)   # box, person, sign
    min_visible: float = 0.4          # 가시 비율이 이보다 작으면 라벨 제외
    min_box_px: int = 10              # 라벨 최소 변 [px]
    noise_std: float = 4.0            # 가우시안 픽셀 잡음 σ (0~255)
    blur_prob: float = 0.3


class SyntheticSceneRenderer:
    """cv2 도형 기반 box / person / sign 장면 생성기."""

    def __init__(self, seed: int = 0, config: Optional[RendererConfig] = None):
        self.rng = np.random.default_rng(seed)
        self.cfg = config or RendererConfig()

    # --- 배경
    def _background(self) -> np.ndarray:
        c = self.cfg
        img = np.zeros((c.height, c.width, 3), np.float32)
        horizon = int(self.rng.uniform(0.35, 0.6) * c.height)
        wall = self.rng.uniform(120, 220, 3)
        floor = self.rng.uniform(60, 160, 3)
        img[:horizon] = wall
        ramp = np.linspace(0.8, 1.2, c.height - horizon).reshape(-1, 1, 1)
        img[horizon:] = floor * ramp
        # 선반(랙) 줄무늬: 주황/파랑 수평 빔 + 회색 기둥
        for _ in range(int(self.rng.integers(0, 4))):
            y = int(self.rng.uniform(0.05, 0.9) * horizon)
            color = self.rng.choice([(40, 110, 230), (180, 90, 30), (90, 90, 90)])
            cv2.rectangle(img, (0, y), (c.width, y + int(self.rng.integers(4, 12))),
                          tuple(float(v) for v in color), -1)
        for _ in range(int(self.rng.integers(0, 5))):
            x = int(self.rng.uniform(0, c.width))
            cv2.rectangle(img, (x, 0), (x + int(self.rng.integers(5, 20)), horizon),
                          (80.0, 80.0, 80.0), -1)
        return img

    # --- 물체 (그린 id 마스크와 bbox 반환)
    def _draw_box(self, img, mask, oid, cx, cy, s):
        w = s * self.rng.uniform(0.9, 1.6)
        h = s * self.rng.uniform(0.6, 1.0)
        top = h * self.rng.uniform(0.15, 0.35)
        skew = w * self.rng.uniform(-0.2, 0.2)
        base = np.array(self.rng.uniform([40, 90, 140], [90, 150, 210]))  # 골판지 (BGR)
        x1, x2 = cx - w / 2, cx + w / 2
        y1, y2 = cy - h / 2, cy + h / 2
        front = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], np.int32)
        topf = np.array([[x1, y1], [x2, y1], [x2 + skew, y1 - top], [x1 + skew, y1 - top]],
                        np.int32)
        for poly, shade in ((front, 1.0), (topf, 1.25)):
            cv2.fillPoly(img, [poly], tuple(float(v) for v in np.clip(base * shade, 0, 255)))
            cv2.fillPoly(mask, [poly], oid)
            cv2.polylines(img, [poly], True, tuple(float(v) for v in base * 0.5), 1)
        # 테이프 줄
        tx = int(cx + skew * 0.5)
        cv2.line(img, (tx, int(y1 - top)), (int(cx), int(y2)), (70.0, 140.0, 190.0), 2)
        pts = np.vstack([front, topf])
        return pts.min(axis=0), pts.max(axis=0)

    def _draw_person(self, img, mask, oid, cx, cy, s):
        h = s * self.rng.uniform(2.2, 3.2)
        w = h * self.rng.uniform(0.28, 0.38)
        top = cy - h / 2
        skin = self.rng.uniform([60, 100, 150], [120, 170, 230])
        shirt = self.rng.uniform(0, 255, 3)
        if self.rng.random() < 0.4:  # 형광 조끼
            shirt = np.array([40.0, 220.0, 240.0]) if self.rng.random() < 0.5 else \
                np.array([30.0, 120.0, 250.0])
        pants = self.rng.uniform(20, 90, 3)
        head_r = 0.09 * h
        hc = (int(cx), int(top + head_r))
        parts = []
        cv2.circle(img, hc, int(head_r), tuple(float(v) for v in skin), -1)
        cv2.circle(mask, hc, int(head_r), oid, -1)
        parts.append(np.array([[cx - head_r, top], [cx + head_r, top + 2 * head_r]]))
        torso = (int(cx - w / 2), int(top + 2 * head_r), int(cx + w / 2), int(top + 0.55 * h))
        cv2.rectangle(img, torso[:2], torso[2:], tuple(float(v) for v in shirt), -1)
        cv2.rectangle(mask, torso[:2], torso[2:], oid, -1)
        parts.append(np.array([torso[:2], torso[2:]]))
        stride = self.rng.uniform(-0.12, 0.12) * h
        for side in (-1, 1):
            x0 = cx + side * w * 0.22
            leg = np.array([[x0 - w * 0.13, top + 0.55 * h], [x0 + w * 0.13, top + 0.55 * h],
                            [x0 + w * 0.13 + side * stride, top + h],
                            [x0 - w * 0.13 + side * stride, top + h]], np.int32)
            cv2.fillPoly(img, [leg], tuple(float(v) for v in pants))
            cv2.fillPoly(mask, [leg], oid)
            parts.append(leg)
            arm = (int(cx + side * (w / 2 + w * 0.08)), int(top + 0.25 * h))
            cv2.line(img, (int(cx + side * w / 2), int(top + 2.2 * head_r)), arm,
                     tuple(float(v) for v in shirt), max(int(w * 0.14), 2))
        pts = np.vstack([p.reshape(-1, 2) for p in parts])
        return pts.min(axis=0), pts.max(axis=0)

    def _draw_sign(self, img, mask, oid, cx, cy, s):
        r = s * self.rng.uniform(0.35, 0.6)
        kind = int(self.rng.integers(0, 3))
        # 기둥 (라벨 제외, 가리개 역할)
        cv2.rectangle(img, (int(cx - r * 0.08), int(cy)), (int(cx + r * 0.08), int(cy + 3 * r)),
                      (120.0, 120.0, 120.0), -1)
        if kind == 0:  # 정지 표지 (빨간 팔각형 + STOP)
            ang = np.arange(8) * math.pi / 4 + math.pi / 8
            poly = np.stack([cx + r * np.cos(ang), cy + r * np.sin(ang)], axis=1).astype(np.int32)
            cv2.fillPoly(img, [poly], (30.0, 30.0, 210.0))
            cv2.polylines(img, [poly], True, (255.0, 255.0, 255.0), max(int(r * 0.08), 1))
            text, color = 'STOP', (255.0, 255.0, 255.0)
        elif kind == 1:  # 경고 삼각형 (노랑)
            poly = np.array([[cx, cy - r], [cx + r * 0.95, cy + r * 0.75],
                             [cx - r * 0.95, cy + r * 0.75]], np.int32)
            cv2.fillPoly(img, [poly], (20.0, 210.0, 240.0))
            cv2.polylines(img, [poly], True, (20.0, 20.0, 20.0), max(int(r * 0.08), 1))
            text, color = '!', (20.0, 20.0, 20.0)
        else:  # 안내 사각형 (파랑)
            poly = np.array([[cx - r, cy - r * 0.7], [cx + r, cy - r * 0.7],
                             [cx + r, cy + r * 0.7],
                             [cx - r, cy + r * 0.7]], np.int32)
            cv2.fillPoly(img, [poly], (180.0, 90.0, 20.0))
            text, color = 'EXIT', (255.0, 255.0, 255.0)
        cv2.fillPoly(mask, [poly], oid)
        scale = max(r / 45.0, 0.25)
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
        cv2.putText(img, text, (int(cx - tw / 2), int(cy + th / 2)), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, max(int(scale * 2), 1), cv2.LINE_AA)
        return poly.min(axis=0), poly.max(axis=0)

    def render(self) -> Tuple[np.ndarray, List[Label]]:
        """장면 한 장 → (BGR uint8 이미지, 라벨)."""
        c = self.cfg
        img = self._background()
        mask = np.zeros((c.height, c.width), np.int32)
        n = int(self.rng.integers(c.min_objects, c.max_objects + 1))
        classes = self.rng.choice(len(CLASSES), size=n, p=np.array(c.class_weights) /
                                  np.sum(c.class_weights))
        scales = np.sort(self.rng.uniform(25, 110, size=n))  # 작은 것(먼 것)부터 → 화가 알고리즘
        drawers = (self._draw_box, self._draw_person, self._draw_sign)
        drawn = []
        for i, (cls, s) in enumerate(zip(classes, scales), start=1):
            cx = self.rng.uniform(0.05, 0.95) * c.width
            cy = self.rng.uniform(0.3, 0.85) * c.height
            lo, hi = drawers[int(cls)](img, mask, i, cx, cy, s)
            # 그린 직후 id 픽셀 수 = 가림 전 전체 면적 (뒤에 그린 물체가 덮으면 줄어든다)
            drawn.append((i, int(cls), lo, hi, int(np.count_nonzero(mask == i))))
        labels: List[Label] = []
        for oid, cls, lo, hi, full in drawn:
            visible = int(np.count_nonzero(mask == oid))
            x1, y1 = max(float(lo[0]), 0.0), max(float(lo[1]), 0.0)
            x2, y2 = min(float(hi[0]), c.width - 1.0), min(float(hi[1]), c.height - 1.0)
            if x2 - x1 < c.min_box_px or y2 - y1 < c.min_box_px:
                continue
            if full <= 0 or visible / full < c.min_visible:
                continue
            labels.append(Label(cls, (x1, y1, x2, y2)))
        img = self._photometric(img)
        return img, labels

    def _photometric(self, img: np.ndarray) -> np.ndarray:
        """밝기·대비 지터, 블러, 센서 잡음."""
        c = self.cfg
        img = img * self.rng.uniform(0.7, 1.3) + self.rng.uniform(-25, 25)
        if self.rng.random() < c.blur_prob:
            img = cv2.GaussianBlur(img, (3, 3), 0)
        if c.noise_std > 0:
            img = img + self.rng.normal(0.0, c.noise_std, img.shape)
        return np.clip(img, 0, 255).astype(np.uint8)


# ------------------------------------------------------------------ 데이터셋 쓰기

def write_sample(out_dir: str, split: str, index: int, image: np.ndarray,
                 labels: Sequence[Label]) -> str:
    """images/<split>/NNNNNN.jpg + labels/<split>/NNNNNN.txt. 반환: 이미지 경로."""
    img_dir = os.path.join(out_dir, 'images', split)
    lbl_dir = os.path.join(out_dir, 'labels', split)
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(lbl_dir, exist_ok=True)
    stem = f'{index:06d}'
    path = os.path.join(img_dir, stem + '.jpg')
    cv2.imwrite(path, image, [cv2.IMWRITE_JPEG_QUALITY, 92])
    h, w = image.shape[:2]
    with open(os.path.join(lbl_dir, stem + '.txt'), 'w', encoding='utf-8') as f:
        for lab in labels:
            f.write(yolo_label_line(lab, w, h) + '\n')
    return path


def write_data_yaml(out_dir: str, classes: Sequence[str] = CLASSES) -> str:
    """Ultralytics 데이터셋 설정 data.yaml."""
    path = os.path.join(out_dir, 'data.yaml')
    cfg = {'path': os.path.abspath(out_dir), 'train': 'images/train', 'val': 'images/val',
           'names': {i: n for i, n in enumerate(classes)}}
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)
    return path


def generate_synthetic_dataset(out_dir: str, n_train: int, n_val: int, seed: int = 0,
                               config: Optional[RendererConfig] = None) -> Dict[str, int]:
    """합성 데이터셋 생성. 반환: 분할별 라벨 수와 클래스별 개수."""
    stats: Dict[str, int] = {'train_images': 0, 'val_images': 0}
    for split, n, s in (('train', n_train, seed), ('val', n_val, seed + 100003)):
        renderer = SyntheticSceneRenderer(s, config)
        for i in range(n):
            img, labels = renderer.render()
            write_sample(out_dir, split, i, img, labels)
            stats[f'{split}_images'] += 1
            for lab in labels:
                key = f'{split}_{CLASSES[lab.class_id]}'
                stats[key] = stats.get(key, 0) + 1
    write_data_yaml(out_dir)
    return stats


# ------------------------------------------------------------------ Gazebo 지면 진실 투영

@dataclass
class WorldObject:
    """월드 좌표의 라벨 대상 3D 박스."""

    class_name: str
    center: np.ndarray          # 박스 중심 (x, y, z) [m], world
    size: np.ndarray            # (길이 x, 폭 y, 높이 z) [m]
    yaw: float = 0.0
    name: str = ''


def box_corners(obj: WorldObject) -> np.ndarray:
    """3D 박스 8 꼭짓점 (world)."""
    lx, ly, lz = 0.5 * np.asarray(obj.size, dtype=float)
    c, s = math.cos(obj.yaw), math.sin(obj.yaw)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    pts = np.array([[sx * lx, sy * ly, sz * lz] for sx in (-1, 1) for sy in (-1, 1)
                    for sz in (-1, 1)])
    return pts @ rot.T + np.asarray(obj.center, dtype=float)


@dataclass
class ProjectionResult:
    """투영된 2D 박스."""

    bbox: BBox
    depth: float                # 카메라에서 박스 중심까지 Z [m]
    truncated: float            # 이미지 밖으로 잘린 면적 비율
    occluded: float = 0.0       # 깊이 검사 가림 비율
    extra: Dict[str, float] = field(default_factory=dict)


def project_object(corners_cam: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                   width: int, height: int, min_z: float = 0.1) -> Optional[ProjectionResult]:
    """
    광학 프레임 8 꼭짓점 → 이미지 2D bbox. 모든 꼭짓점이 카메라 앞(Z > min_z) 이어야 한다.

    이미지 안으로 자르고, 잘린 면적 비율(truncated)을 함께 돌려준다.
    """
    z = corners_cam[:, 2]
    if np.any(z <= min_z):
        return None
    u = fx * corners_cam[:, 0] / z + cx
    v = fy * corners_cam[:, 1] / z + cy
    x1, x2, y1, y2 = float(u.min()), float(u.max()), float(v.min()), float(v.max())
    full = (x2 - x1) * (y2 - y1)
    cx1, cx2 = max(x1, 0.0), min(x2, width - 1.0)
    cy1, cy2 = max(y1, 0.0), min(y2, height - 1.0)
    if cx2 <= cx1 or cy2 <= cy1 or full <= 0.0:
        return None
    clipped = (cx2 - cx1) * (cy2 - cy1)
    return ProjectionResult((cx1, cy1, cx2, cy2), float(np.mean(z)), 1.0 - clipped / full)


def occlusion_ratio(depth_image: np.ndarray, bbox: BBox, expected_depth: float,
                    tolerance: float) -> float:
    """
    바운딩 박스 중앙 50 % 영역에서 측정 깊이가 기대 깊이보다 tolerance 이상 가까운 픽셀 비율.

    무효(비유한) 픽셀은 가림이 아닌 것으로 본다 (먼 배경).
    """
    x1, y1, x2, y2 = bbox
    w, h = x2 - x1, y2 - y1
    u0, u1 = int(x1 + 0.25 * w), int(math.ceil(x2 - 0.25 * w))
    v0, v1 = int(y1 + 0.25 * h), int(math.ceil(y2 - 0.25 * h))
    roi = np.asarray(depth_image, dtype=np.float64)[v0:max(v1, v0 + 1), u0:max(u1, u0 + 1)]
    if roi.size == 0:
        return 1.0
    closer = np.isfinite(roi) & (roi < expected_depth - tolerance)
    return float(np.count_nonzero(closer)) / roi.size
