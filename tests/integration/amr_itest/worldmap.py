"""
월드 SDF 의 정적 구조물 → 2D 지면 진실 (rclpy 비의존 순수 모듈, 시나리오 03·06).

  footprints()      warehouse.sdf 의 정적 충돌체(박스·원기둥) 중 LiDAR 평면 높이를 지나는 것의
                    평면 도형 (model://<이름> include 는 models/<이름>/model.sdf 로 푼다)
  rasterize()       도형 → OccupancyGrid 와 같은 격자의 점유 불리언
  map_agreement()   SLAM 지도 vs 지면 진실: 보이는 구조물 가장자리 재현율 + 자유 공간 오점유율
  sample_free()     구조물에서 margin 이상 떨어진 자유 지점 (경로 계획 시작/목표 쌍)
자세는 yaw 만 반영한다 (정적 구조물은 수평 배치). actor(작업자)는 충돌체가 없어 제외된다.
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple, Union
import xml.etree.ElementTree as ET

import numpy as np

Pose2 = Tuple[float, float, float, float]     # (x, y, z, yaw)


@dataclass(frozen=True)
class Rect:
    """회전 사각형 (중심, yaw, 크기)."""

    cx: float
    cy: float
    yaw: float
    sx: float
    sy: float

    def contains(self, px: np.ndarray, py: np.ndarray, pad: float = 0.0) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = px - self.cx, py - self.cy
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        return (np.abs(lx) <= self.sx / 2 + pad) & (np.abs(ly) <= self.sy / 2 + pad)

    def perimeter(self, step: float = 0.05) -> np.ndarray:
        hx, hy = self.sx / 2, self.sy / 2
        corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy), (-hx, -hy)]
        pts = []
        for (x0, y0), (x1, y1) in zip(corners[:-1], corners[1:]):
            n = max(2, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / step)))
            t = np.linspace(0.0, 1.0, n, endpoint=False)
            pts.append(np.column_stack([x0 + t * (x1 - x0), y0 + t * (y1 - y0)]))
        local = np.vstack(pts)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return local @ np.array([[c, s], [-s, c]]) + np.array([self.cx, self.cy])


@dataclass(frozen=True)
class Circle:
    """원 (원기둥 단면)."""

    cx: float
    cy: float
    r: float

    def contains(self, px: np.ndarray, py: np.ndarray, pad: float = 0.0) -> np.ndarray:
        return np.hypot(px - self.cx, py - self.cy) <= self.r + pad

    def perimeter(self, step: float = 0.05) -> np.ndarray:
        n = max(8, int(math.ceil(2 * math.pi * self.r / step)))
        a = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
        return np.column_stack([self.cx + self.r * np.cos(a), self.cy + self.r * np.sin(a)])


Shape = Union[Rect, Circle]


def parse_pose(text: Optional[str]) -> Pose2:
    """SDF <pose> 'x y z r p y' → (x, y, z, yaw)."""
    if not text:
        return 0.0, 0.0, 0.0, 0.0
    v = [float(t) for t in text.split()] + [0.0] * 6
    return v[0], v[1], v[2], v[5]


def compose(a: Pose2, b: Pose2) -> Pose2:
    """두 자세의 합성 a ∘ b (평면 + z)."""
    c, s = math.cos(a[3]), math.sin(a[3])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2], a[3] + b[3])


def _model_shapes(model: ET.Element, pose: Pose2, plane_z: float) -> List[Shape]:
    out: List[Shape] = []
    for link in model.findall('link'):
        lp = compose(pose, parse_pose(link.findtext('pose')))
        for col in link.findall('collision'):
            cp = compose(lp, parse_pose(col.findtext('pose')))
            box = col.find('geometry/box/size')
            cyl = col.find('geometry/cylinder')
            if box is not None:
                sx, sy, sz = (float(t) for t in box.text.split())
                if cp[2] - sz / 2 <= plane_z <= cp[2] + sz / 2:
                    out.append(Rect(cp[0], cp[1], cp[3], sx, sy))
            elif cyl is not None:
                r = float(cyl.findtext('radius', '0'))
                h = float(cyl.findtext('length', '0'))
                if cp[2] - h / 2 <= plane_z <= cp[2] + h / 2:
                    out.append(Circle(cp[0], cp[1], r))
    return out


def footprints(world_sdf: Path, models_dir: Path, plane_z: float) -> List[Shape]:
    """월드의 정적 충돌 도형 중 높이 plane_z [m] 평면을 지나는 것."""
    root = ET.parse(str(world_sdf)).getroot()
    world = root.find('world')
    shapes: List[Shape] = []
    for model in world.findall('model'):
        if model.findtext('static', 'false').strip() not in ('true', '1'):
            continue
        shapes += _model_shapes(model, parse_pose(model.findtext('pose')), plane_z)
    for inc in world.findall('include'):
        uri = inc.findtext('uri', '')
        if not uri.startswith('model://'):
            continue
        path = Path(models_dir) / uri[len('model://'):] / 'model.sdf'
        if not path.is_file():
            continue
        mroot = ET.parse(str(path)).getroot()
        model = mroot.find('model')
        if model is None or model.findtext('static', 'false').strip() not in ('true', '1'):
            continue
        base = compose(parse_pose(inc.findtext('pose')), parse_pose(model.findtext('pose')))
        shapes += _model_shapes(model, base, plane_z)
    return shapes


def cell_centers(width: int, height: int, resolution: float,
                 origin: Tuple[float, float]) -> Tuple[np.ndarray, np.ndarray]:
    """격자(OccupancyGrid) 셀 중심 좌표 (행 = y, 열 = x; 원점 yaw 0 가정)."""
    xs = origin[0] + (np.arange(width) + 0.5) * resolution
    ys = origin[1] + (np.arange(height) + 0.5) * resolution
    return np.meshgrid(xs, ys)


def rasterize(shapes: Iterable[Shape], width: int, height: int, resolution: float,
              origin: Tuple[float, float], pad: float = 0.0) -> np.ndarray:
    """도형 내부(+pad) 셀 = True 인 (height, width) 격자."""
    px, py = cell_centers(width, height, resolution, origin)
    occ = np.zeros((height, width), dtype=bool)
    for s in shapes:
        occ |= s.contains(px, py, pad)
    return occ


@dataclass(frozen=True)
class Agreement:
    """SLAM 지도 vs 지면 진실."""

    recall: float            # 보이는 구조물 가장자리 점 중 tol 안에 점유 셀이 있는 비율
    false_occupied: float    # 점유 셀 중 구조물에서 tol_fp 보다 먼 비율
    edge_points: int
    occupied_cells: int


def map_agreement(grid: np.ndarray, resolution: float, origin: Tuple[float, float],
                  shapes: Sequence[Shape], tol: float = 0.10,
                  tol_fp: float = 0.15) -> Agreement:
    """
    grid: OccupancyGrid.data 를 (height, width) 로 바꾼 int 배열 (−1 미지, 0 자유, 100 점유).

    재현율은 "보이는" 가장자리만 센다: 점 주변 tol 안에 자유 셀(0)이 있어야 LiDAR 가 볼 수 있다.
    """
    from scipy import ndimage

    def dist_to(mask: np.ndarray) -> np.ndarray:
        """각 셀에서 mask 가 참인 가장 가까운 셀까지 거리 [m] (mask 가 비면 inf)."""
        if not mask.any():
            return np.full(mask.shape, np.inf)
        return ndimage.distance_transform_edt(~mask) * resolution

    h, w = grid.shape
    occupied = grid >= 50
    free = grid == 0
    dist_occ = dist_to(occupied)
    dist_free = dist_to(free)
    gt = rasterize(shapes, w, h, resolution, origin)
    dist_gt = dist_to(gt)
    seen = hit = 0
    for s in shapes:
        pts = s.perimeter(resolution)
        ix = np.floor((pts[:, 0] - origin[0]) / resolution).astype(int)
        iy = np.floor((pts[:, 1] - origin[1]) / resolution).astype(int)
        ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        ix, iy = ix[ok], iy[ok]
        visible = dist_free[iy, ix] <= tol
        seen += int(np.sum(visible))
        hit += int(np.sum(visible & (dist_occ[iy, ix] <= tol)))
    n_occ = int(np.sum(occupied))
    fp = int(np.sum(occupied & (dist_gt > tol_fp)))
    return Agreement(hit / seen if seen else math.nan, fp / n_occ if n_occ else math.nan,
                     seen, n_occ)


def sample_free(shapes: Sequence[Shape], bounds: Tuple[float, float, float, float],
                margin: float, n: int, rng: np.random.Generator,
                max_tries: int = 100000) -> np.ndarray:
    """영역 bounds (xmin, ymin, xmax, ymax) 안에서 모든 도형과 margin 이상 떨어진 점 n 개."""
    out = []
    tries = 0
    while len(out) < n and tries < max_tries:
        tries += 1
        x = rng.uniform(bounds[0] + margin, bounds[2] - margin)
        y = rng.uniform(bounds[1] + margin, bounds[3] - margin)
        px, py = np.array([x]), np.array([y])
        if not any(bool(s.contains(px, py, margin)[0]) for s in shapes):
            out.append((x, y))
    return np.array(out).reshape(-1, 2)


def polyline_distance(points: np.ndarray, px: float, py: float) -> float:
    """점 (px, py) ~ 폴리라인 (N, 2) 최단 거리 [m] (경로 이탈 판정)."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return math.inf
    if len(pts) == 1:
        return float(math.hypot(px - pts[0, 0], py - pts[0, 1]))
    a, b = pts[:-1], pts[1:]
    ab = b - a
    denom = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
    t = np.clip(((px - a[:, 0]) * ab[:, 0] + (py - a[:, 1]) * ab[:, 1]) / denom, 0.0, 1.0)
    cx, cy = a[:, 0] + t * ab[:, 0], a[:, 1] + t * ab[:, 1]
    return float(np.min(np.hypot(px - cx, py - cy)))


@dataclass(frozen=True)
class ActorTrack:
    """SDF actor 궤적 (웨이포인트 선형 보간, loop 면 주기 반복)."""

    name: str
    times: Tuple[float, ...]
    xy: Tuple[Tuple[float, float], ...]
    loop: bool = True
    delay: float = 0.0

    def position(self, t: float) -> Tuple[float, float]:
        """시각 t [s](sim time)의 위치 (Fortress actor 는 렌더링 쪽에서 이 궤적을 sim time 으로 재생)."""
        if not self.times:
            return math.nan, math.nan
        period = self.times[-1]
        u = max(0.0, t - self.delay)
        if self.loop and period > 0.0:
            u = u % period
        u = min(u, period)
        i = int(np.searchsorted(self.times, u, side='right'))
        if i == 0:
            return self.xy[0]
        if i >= len(self.times):
            return self.xy[-1]
        t0, t1 = self.times[i - 1], self.times[i]
        r = 0.0 if t1 <= t0 else (u - t0) / (t1 - t0)
        (x0, y0), (x1, y1) = self.xy[i - 1], self.xy[i]
        return x0 + r * (x1 - x0), y0 + r * (y1 - y0)


def actor_tracks(world_sdf: Path) -> List[ActorTrack]:
    """월드의 actor(작업자) 궤적 — 충돌체가 없어 충돌 판정은 이 궤적과의 거리로 한다."""
    world = ET.parse(str(world_sdf)).getroot().find('world')
    out = []
    for actor in world.findall('actor'):
        traj = actor.find('script/trajectory')
        if traj is None:
            continue
        times, xy = [], []
        for wp in traj.findall('waypoint'):
            p = parse_pose(wp.findtext('pose'))
            times.append(float(wp.findtext('time', '0')))
            xy.append((p[0], p[1]))
        script = actor.find('script')
        out.append(ActorTrack(actor.get('name', ''), tuple(times), tuple(xy),
                              script.findtext('loop', 'true').strip() in ('true', '1'),
                              float(script.findtext('delay_start', '0') or 0.0)))
    return out


def include_pose(world_sdf: Path, name: str) -> Optional[Pose2]:
    """<include><name>name</name> 또는 <model name=...> 의 월드 자세 (없으면 None)."""
    world = ET.parse(str(world_sdf)).getroot().find('world')
    for inc in world.findall('include'):
        if inc.findtext('name', '') == name:
            return parse_pose(inc.findtext('pose'))
    for model in world.findall('model'):
        if model.get('name') == name:
            return parse_pose(model.findtext('pose'))
    return None
