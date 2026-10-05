"""
월드 SDF 의 **시각체** 기하로 만든 정답 면과 점-면 거리 (rclpy 비의존 — 센서 오차 측정, 보고서 2단계).

gpu_lidar 는 렌더링 기반이라 충돌체가 아니라 시각체를 본다 (랙 충돌체는 통짜 박스지만 시각체는 데크·기둥·선반).
그래서 정답도 시각체로 만든다. 지원: box, plane(유한, 법선 방향 무관하게 양면). 그 밖(cylinder·sphere·mesh)은
건너뛰고 개수를 센다 — 그 형상에 맞은 점은 정답 면이 없어 큰 거리로 나오므로 측정 위치를 그런 물체가 없는 곳으로 고른다.

    boxes, planes, skipped = load_world(world_sdf, models_dir, exclude=('forklift', 'shuttle_amr'))
    d = surface_distance(points_world, boxes, planes)   # [N] 가장 가까운 정답 면까지 거리 [m]

자세: SDF <pose> x y z roll pitch yaw (고정축 X-Y-Z = R = Rz·Ry·Rx). 합성 = 모델(include) ∘ 링크 ∘ 시각체.
<include> 의 <pose> 는 모델 자체 <pose> 를 **대체**한다 (SDF 1.8 규칙). actor 는 모두 뺀다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple
import xml.etree.ElementTree as ET

import numpy as np


@dataclass(frozen=True)
class Box:
    """월드 좌표의 방향 있는 상자 (중심, 회전 행렬 — 열 = 상자 축, 반치수)."""

    name: str
    center: np.ndarray
    rot: np.ndarray
    half: np.ndarray


@dataclass(frozen=True)
class Plane:
    """유한 평면 (중심, 회전 행렬 — 3열 = 법선, 반치수 2개)."""

    name: str
    center: np.ndarray
    rot: np.ndarray
    half: np.ndarray


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """SDF rpy → 회전 행렬 (Rz(yaw)·Ry(pitch)·Rx(roll))."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


Pose = Tuple[np.ndarray, np.ndarray]     # (R, t)


def parse_pose(elem) -> Pose:
    """<pose> 자식 → (R, t). 없으면 항등."""
    node = elem.find('pose') if elem is not None else None
    if node is None or not (node.text or '').strip():
        return np.eye(3), np.zeros(3)
    v = [float(t) for t in node.text.split()]
    if len(v) != 6:
        raise ValueError(f'pose 는 6 값: {node.text!r}')
    return rpy_matrix(v[3], v[4], v[5]), np.array(v[:3])


def compose(a: Pose, b: Pose) -> Pose:
    """부모 a 안의 자세 b → a 의 부모 좌표."""
    return a[0] @ b[0], a[0] @ b[1] + a[1]


def _model_shapes(model, model_pose: Pose, name: str, boxes, planes, skipped) -> None:
    for link in model.findall('link'):
        link_pose = compose(model_pose, parse_pose(link))
        for vis in link.findall('visual'):
            pose = compose(link_pose, parse_pose(vis))
            geom = vis.find('geometry')
            if geom is None:
                continue
            box = geom.find('box/size')
            plane = geom.find('plane')
            label = f'{name}/{vis.get("name")}'
            if box is not None:
                size = np.array([float(t) for t in box.text.split()])
                boxes.append(Box(label, pose[1], pose[0], size / 2.0))
            elif plane is not None:
                normal = np.array([float(t) for t in plane.find('normal').text.split()])
                size = np.array([float(t) for t in plane.find('size').text.split()])
                rot = pose[0] @ _frame_with_z(normal)
                planes.append(Plane(label, pose[1], rot, size / 2.0))
            else:
                skipped.append(label)


def _frame_with_z(normal: np.ndarray) -> np.ndarray:
    """법선을 z 축으로 하는 회전 (SDF plane 의 size 는 법선에 수직인 x, y 크기 — ±z 법선만 정확)."""
    n = normal / np.linalg.norm(normal)
    if abs(abs(n[2]) - 1.0) < 1e-9:
        return np.diag([1.0, 1.0, 1.0]) if n[2] > 0 else np.diag([1.0, -1.0, -1.0])
    x = np.cross([0.0, 0.0, 1.0], n)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(n, x), n])


def load_world(world_sdf: str, models_dir: str, exclude: Iterable[str] = ()) \
        -> Tuple[List[Box], List[Plane], List[str]]:
    """월드 SDF → (상자, 평면, 건너뛴 시각체 이름). exclude 는 include uri 의 모델 이름(예 'forklift')."""
    excluded = set(exclude)
    world = ET.parse(world_sdf).getroot().find('world')
    boxes: List[Box] = []
    planes: List[Plane] = []
    skipped: List[str] = []
    for inc in world.findall('include'):
        model_name = inc.find('uri').text.strip().replace('model://', '')
        if model_name in excluded:
            continue
        name_elem = inc.find('name')
        name = name_elem.text.strip() if name_elem is not None else model_name
        model = ET.parse(Path(models_dir) / model_name / 'model.sdf').getroot().find('model')
        pose = parse_pose(inc) if inc.find('pose') is not None else parse_pose(model)
        _model_shapes(model, pose, name, boxes, planes, skipped)
    for model in world.findall('model'):
        _model_shapes(model, parse_pose(model), model.get('name'), boxes, planes, skipped)
    return boxes, planes, skipped


def _box_abs_sdf(points: np.ndarray, box: Box) -> np.ndarray:
    """|부호 거리| (표면까지 거리 — 안쪽 점도 가장 가까운 면까지)."""
    q = (points - box.center) @ box.rot          # 상자 좌표
    d = np.abs(q) - box.half
    outside = np.linalg.norm(np.maximum(d, 0.0), axis=1)
    inside = np.minimum(d.max(axis=1), 0.0)
    return np.abs(outside + inside)


def _plane_dist(points: np.ndarray, plane: Plane) -> np.ndarray:
    q = (points - plane.center) @ plane.rot
    dx = np.maximum(np.abs(q[:, :2]) - plane.half, 0.0)
    return np.sqrt(dx[:, 0] ** 2 + dx[:, 1] ** 2 + q[:, 2] ** 2)


def surface_distance(points: np.ndarray, boxes: Sequence[Box], planes: Sequence[Plane],
                     chunk: int = 4096) -> np.ndarray:
    """각 점에서 가장 가까운 정답 면까지 거리 [m]. points: [N, 3] 월드 좌표."""
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    out = np.full(len(pts), np.inf)
    for s in range(0, len(pts), chunk):
        p = pts[s:s + chunk]
        best = np.full(len(p), np.inf)
        for b in boxes:
            best = np.minimum(best, _box_abs_sdf(p, b))
        for pl in planes:
            best = np.minimum(best, _plane_dist(p, pl))
        out[s:s + chunk] = best
    return out


def summarize(values: np.ndarray) -> dict:
    """중앙값·p95·평균·최대·개수 (유한값만)."""
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        return {'n': 0}
    return {'n': int(v.size), 'median': float(np.median(v)), 'p95': float(np.percentile(v, 95)),
            'mean': float(v.mean()), 'max': float(v.max()), 'rms': float(np.sqrt(np.mean(v * v)))}
