r"""
월드 SDF → 스캔 평면 단면 점유격자 (맵 품질 평가의 지면 진실, rclpy 비의존).

    ros2 run amr_localization world_to_map --world src/amr_simulation/worlds/warehouse.sdf \
        --models src/amr_simulation/models --height 0.38 --resolution 0.05 --out maps/warehouse_gt

gpu_lidar 는 visual 지오메트리를 측정하므로 각 정적 모델의 <visual> 을 스캔 평면 높이 z = h 에서
자른 단면을 래스터화한다. 지원: box, cylinder, sphere (메시는 건너뛰고 목록을 보고).
포즈 합성: world ← (include/model pose) ← link pose ← visual pose (roll/pitch 포함 일반 3D 회전).
셀 중심 (x, y, h) 가 도형 내부인지 역변환으로 판정하므로 회전된 도형도 정확히 처리한다.
동적 모델(지게차 등)과 actor 는 --exclude 이름 접두어로 뺀다.
"""

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys
from typing import List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

import numpy as np

DEFAULT_EXCLUDE = ('ground_plane', 'forklift', 'worker', 'amr_')


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """SDF rpy (고정축 X-Y-Z, R = Rz Ry Rx)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def pose_matrix(text: Optional[str]) -> np.ndarray:
    """'x y z r p y' → 4x4 동차 변환 (없으면 항등)."""
    t = np.eye(4)
    if text is None or not text.strip():
        return t
    v = [float(x) for x in text.split()]
    v += [0.0] * (6 - len(v))
    t[:3, :3] = rpy_matrix(v[3], v[4], v[5])
    t[:3, 3] = v[:3]
    return t


@dataclass
class Shape:
    """월드 좌표 도형 1 개."""

    name: str
    kind: str                 # box | cylinder | sphere
    size: Tuple[float, ...]   # box (sx, sy, sz), cylinder (r, L), sphere (r,)
    world_from_shape: np.ndarray

    def contains(self, pts: np.ndarray) -> np.ndarray:
        """월드 점 (N, 3) 이 내부인지."""
        inv = np.linalg.inv(self.world_from_shape)
        local = pts @ inv[:3, :3].T + inv[:3, 3]
        if self.kind == 'box':
            half = np.array(self.size) / 2.0
            return np.all(np.abs(local) <= half + 1e-9, axis=1)
        if self.kind == 'cylinder':
            r, length = self.size
            return ((local[:, 0] ** 2 + local[:, 1] ** 2 <= r * r + 1e-12)
                    & (np.abs(local[:, 2]) <= length / 2.0 + 1e-9))
        r = self.size[0]
        return np.sum(local ** 2, axis=1) <= r * r + 1e-12

    def xy_bounds(self) -> Tuple[float, float, float, float]:
        """외접 AABB 의 xy 범위."""
        if self.kind == 'box':
            h = np.array(self.size) / 2.0
        elif self.kind == 'cylinder':
            h = np.array([self.size[0], self.size[0], self.size[1] / 2.0])
        else:
            h = np.array([self.size[0]] * 3)
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                            for sz in (-1, 1)]) * h
        w = corners @ self.world_from_shape[:3, :3].T + self.world_from_shape[:3, 3]
        return w[:, 0].min(), w[:, 0].max(), w[:, 1].min(), w[:, 1].max()


def _geometry(elem: ET.Element) -> Optional[Tuple[str, Tuple[float, ...]]]:
    geom = elem.find('geometry')
    if geom is None:
        return None
    box = geom.find('box')
    if box is not None:
        return 'box', tuple(float(v) for v in box.findtext('size', '1 1 1').split())
    cyl = geom.find('cylinder')
    if cyl is not None:
        return 'cylinder', (float(cyl.findtext('radius', '0.5')),
                            float(cyl.findtext('length', '1.0')))
    sph = geom.find('sphere')
    if sph is not None:
        return 'sphere', (float(sph.findtext('radius', '0.5')),)
    return None


class WorldLoader:
    """월드 SDF 의 정적 visual 도형을 모은다."""

    def __init__(self, model_paths: Sequence[Path], exclude: Sequence[str] = DEFAULT_EXCLUDE,
                 element: str = 'visual') -> None:
        """model_paths: model://<name> 을 찾을 디렉토리들."""
        self.model_paths = [Path(p) for p in model_paths]
        self.exclude = tuple(exclude)
        self.element = element
        self.shapes: List[Shape] = []
        self.skipped: List[str] = []

    def _excluded(self, name: str) -> bool:
        return any(name.startswith(p) for p in self.exclude)

    def _resolve(self, uri: str) -> Optional[Path]:
        name = uri.replace('model://', '').strip('/')
        for base in self.model_paths:
            candidate = base / name / 'model.sdf'
            if candidate.is_file():
                return candidate
        return None

    def load_world(self, path: Path) -> List[Shape]:
        """월드 파일 전체."""
        root = ET.parse(path).getroot()
        world = root.find('world')
        if world is None:
            raise ValueError(f'{path}: <world> 없음')
        for model in world.findall('model'):
            name = model.get('name', '')
            if not self._excluded(name):
                self._add_model(model, pose_matrix(model.findtext('pose')), name)
        for inc in world.findall('include'):
            name = inc.findtext('name') or inc.findtext('uri', '')
            if self._excluded(name):
                continue
            model_file = self._resolve(inc.findtext('uri', ''))
            if model_file is None:
                self.skipped.append(f'{name}: model not found')
                continue
            model = ET.parse(model_file).getroot().find('model')
            if model is None:
                self.skipped.append(f'{name}: no <model> in {model_file}')
                continue
            inc_pose = pose_matrix(inc.findtext('pose'))
            self._add_model(model, inc_pose @ pose_matrix(model.findtext('pose')), name)
        return self.shapes

    def _add_model(self, model: ET.Element, world_from_model: np.ndarray, name: str) -> None:
        for sub in model.findall('model'):  # 중첩 모델
            self._add_model(sub, world_from_model @ pose_matrix(sub.findtext('pose')),
                            f"{name}::{sub.get('name', '')}")
        for link in model.findall('link'):
            world_from_link = world_from_model @ pose_matrix(link.findtext('pose'))
            for vis in link.findall(self.element):
                geo = _geometry(vis)
                label = f"{name}::{link.get('name', '')}::{vis.get('name', '')}"
                if geo is None:
                    self.skipped.append(f'{label}: unsupported geometry')
                    continue
                self.shapes.append(Shape(label, geo[0], geo[1],
                                         world_from_link @ pose_matrix(vis.findtext('pose'))))


@dataclass
class Raster:
    """점유격자 (True = 점유). 셀 (row 0, col 0) 의 모서리 = origin."""

    occupied: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float


def rasterize(shapes: Sequence[Shape], height: float, resolution: float,
              bounds: Tuple[float, float, float, float]) -> Raster:
    """높이 height 단면을 bounds=(xmin, xmax, ymin, ymax) 범위에 래스터화."""
    xmin, xmax, ymin, ymax = bounds
    width = int(math.ceil((xmax - xmin) / resolution))
    rows = int(math.ceil((ymax - ymin) / resolution))
    occ = np.zeros((rows, width), dtype=bool)
    for shape in shapes:
        sx0, sx1, sy0, sy1 = shape.xy_bounds()
        c0 = max(int(math.floor((sx0 - xmin) / resolution)), 0)
        c1 = min(int(math.ceil((sx1 - xmin) / resolution)), width)
        r0 = max(int(math.floor((sy0 - ymin) / resolution)), 0)
        r1 = min(int(math.ceil((sy1 - ymin) / resolution)), rows)
        if c0 >= c1 or r0 >= r1:
            continue
        cc, rr = np.meshgrid(np.arange(c0, c1), np.arange(r0, r1))
        pts = np.stack([xmin + (cc.ravel() + 0.5) * resolution,
                        ymin + (rr.ravel() + 0.5) * resolution,
                        np.full(cc.size, height)], axis=1)
        inside = shape.contains(pts)
        occ[rr.ravel()[inside], cc.ravel()[inside]] = True
    return Raster(occ, resolution, xmin, ymin)


def write_map(raster: Raster, stem: Path) -> Tuple[Path, Path]:
    """map_server 형식 (P5 PGM + YAML). 점유 0(검정), 자유 254(흰색), 위쪽 행 = +y 끝."""
    stem.parent.mkdir(parents=True, exist_ok=True)
    pgm = stem.with_suffix('.pgm')
    img = np.where(raster.occupied, 0, 254).astype(np.uint8)[::-1]
    with pgm.open('wb') as f:
        f.write(f'P5\n{img.shape[1]} {img.shape[0]}\n255\n'.encode())
        f.write(img.tobytes())
    yml = stem.with_suffix('.yaml')
    yml.write_text(
        f'image: {pgm.name}\nmode: trinary\nresolution: {raster.resolution}\n'
        f'origin: [{raster.origin_x}, {raster.origin_y}, 0.0]\nnegate: 0\n'
        f'occupied_thresh: 0.65\nfree_thresh: 0.25\n', encoding='utf-8')
    return pgm, yml


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI: 월드 → GT 맵 파일."""
    ap = argparse.ArgumentParser(description='world SDF → ground-truth occupancy map')
    ap.add_argument('--world', required=True)
    ap.add_argument('--models', action='append', default=[],
                    help='model:// 검색 경로 (여러 번 가능, 기본: IGN_GAZEBO_RESOURCE_PATH)')
    ap.add_argument('--height', type=float, default=0.38, help='스캔 평면 높이 [m]')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--bounds', type=float, nargs=4, default=[-30.5, 30.5, -20.5, 20.5],
                    metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'))
    ap.add_argument('--exclude', action='append', default=None)
    ap.add_argument('--out', required=True, help='출력 경로 (확장자 제외)')
    args = ap.parse_args(argv)
    paths = args.models or [p for p in os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '').split(':')
                            if p]
    loader = WorldLoader([Path(p) for p in paths], args.exclude or DEFAULT_EXCLUDE)
    shapes = loader.load_world(Path(args.world))
    raster = rasterize(shapes, args.height, args.resolution, tuple(args.bounds))
    pgm, yml = write_map(raster, Path(args.out))
    print(f'{len(shapes)} shapes, {int(raster.occupied.sum())} occupied cells → {pgm}, {yml}')
    for s in loader.skipped:
        print(f'skipped: {s}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
