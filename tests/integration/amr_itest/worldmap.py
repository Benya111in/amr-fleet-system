"""
월드 SDF → 2D 지면 진실 (rclpy 비의존 순수 모듈, 시나리오 03·04~13 의 map↔월드 정합, 06 경로 판정).

  footprints()      월드의 구조물·물품 도형 중 높이 plane_z 평면을 지나는 것의 평면 단면
                    (geometry: visual = LiDAR(gpu_lidar)가 실제로 재는 면 — 지도 비교 기본값,
                    collision = 물리 충돌체, both = 둘의 합). 스스로 움직이는 모델(<plugin> 을 가진 지게차·셔틀)과
                    ground_plane · actor 는 뺀다. model://<이름> include 는 models/<이름>/model.sdf 로 푼다
  rasterize()       도형 → OccupancyGrid 와 같은 격자의 점유 불리언
  map_agreement()   SLAM 지도 vs 지면 진실: 보이는 구조물 가장자리 재현율 + 자유 공간 오점유율
  register()        지도 점유 셀 ↔ 지면 진실의 SE(2) 정합 (map 프레임 = 월드 프레임 가정을 명시적으로 확인)
  sample_free()     구조물에서 margin 이상 떨어진 자유 지점 (경로 계획 시작/목표 쌍)
자세는 yaw 만 반영한다 (구조물은 수평 배치). actor(작업자)는 충돌체·visual 이 없어 제외된다.
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


GEOMETRIES = ('visual', 'collision', 'both')
EXCLUDED_MODELS = ('ground_plane',)


def _tags(geometry: str) -> Tuple[str, ...]:
    if geometry not in GEOMETRIES:
        raise ValueError(f'geometry={geometry!r}: {GEOMETRIES} 중 하나')
    return ('visual', 'collision') if geometry == 'both' else (geometry,)


def _model_shapes(model: ET.Element, pose: Pose2, plane_z: float,
                  tags: Sequence[str] = ('collision',)) -> List[Shape]:
    out: List[Shape] = []
    for link in model.findall('link'):
        lp = compose(pose, parse_pose(link.findtext('pose')))
        for tag in tags:
            for col in link.findall(tag):
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


def _is_static(model: ET.Element) -> bool:
    return model.findtext('static', 'false').strip() in ('true', '1')


def footprints(world_sdf: Path, models_dir: Path, plane_z: float, geometry: str = 'visual',
               static_only: bool = False) -> List[Shape]:
    """
    월드 도형 중 높이 plane_z [m] 평면을 지나는 것의 단면.

    geometry: visual(기본 — gpu_lidar 가 재는 면, 랙은 기둥·선반만) | collision(랙은 외곽 상자 하나) | both.
    <plugin> 을 가진 model/include(지게차·셔틀 등 스스로 움직이는 것)와 ground_plane 은 뺀다.
    static_only 면 <static>true</static> 모델만 (물리 상자·팔레트 제외).
    """
    tags = _tags(geometry)
    root = ET.parse(str(world_sdf)).getroot()
    world = root.find('world')
    shapes: List[Shape] = []
    for model in world.findall('model'):
        if model.get('name', '').startswith(EXCLUDED_MODELS) or model.find('plugin') is not None:
            continue
        if static_only and not _is_static(model):
            continue
        shapes += _model_shapes(model, parse_pose(model.findtext('pose')), plane_z, tags)
    for inc in world.findall('include'):
        uri = inc.findtext('uri', '')
        if not uri.startswith('model://') or inc.find('plugin') is not None:
            continue
        if inc.findtext('name', '').startswith(EXCLUDED_MODELS):
            continue
        path = Path(models_dir) / uri[len('model://'):] / 'model.sdf'
        if not path.is_file():
            continue
        mroot = ET.parse(str(path)).getroot()
        model = mroot.find('model')
        if model is None or model.find('plugin') is not None:
            continue
        if static_only and not _is_static(model):
            continue
        base = compose(parse_pose(inc.findtext('pose')), parse_pose(model.findtext('pose')))
        shapes += _model_shapes(model, base, plane_z, tags)
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
    # 오점유가 몰린 1 m 칸 [(x, y, 셀 수)] 상위 10 — 동적 장애물 흔적·드리프트 위치 진단용
    hotspots: Tuple[Tuple[float, float, int], ...] = ()


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
    fp_mask = occupied & (dist_gt > tol_fp)
    fp = int(np.sum(fp_mask))
    iy, ix = np.nonzero(fp_mask)
    bins = {}
    for bx, by in zip(np.floor(origin[0] + (ix + 0.5) * resolution).astype(int),
                      np.floor(origin[1] + (iy + 0.5) * resolution).astype(int)):
        bins[(int(bx), int(by))] = bins.get((int(bx), int(by)), 0) + 1
    top = sorted(bins.items(), key=lambda kv: -kv[1])[:10]
    return Agreement(hit / seen if seen else math.nan, fp / n_occ if n_occ else math.nan,
                     seen, n_occ, tuple((bx + 0.5, by + 0.5, n) for (bx, by), n in top))


def write_map(grid: np.ndarray, resolution: float, origin: Tuple[float, float],
              stem: Path) -> Tuple[Path, Path]:
    """
    점유 격자 값 배열(OccupancyGrid, height × width; −1/0/100)을 map_server 형식 PGM + YAML 로 쓴다.

    시스템의 지도 저장(slam_toolbox save_map)과 별개로, 판정에 쓴 지도를 로그에 남겨 진단할 수 있게 한다.
    """
    g = np.asarray(grid)
    img = np.full(g.shape, 205, dtype=np.uint8)          # 미지
    img[g == 0] = 254
    img[g >= 50] = 0
    stem = Path(stem)
    pgm, yml = stem.with_suffix('.pgm'), stem.with_suffix('.yaml')
    with open(pgm, 'wb') as fh:
        fh.write(f'P5\n{g.shape[1]} {g.shape[0]}\n255\n'.encode())
        fh.write(img[::-1].tobytes())                    # PGM 첫 행 = 지도 위쪽(y 최대)
    yml.write_text(f'image: {pgm.name}\nmode: trinary\nresolution: {resolution}\n'
                   f'origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\n'
                   'occupied_thresh: 0.65\nfree_thresh: 0.196\n', encoding='utf-8')
    return pgm, yml


def occupied_points(grid: np.ndarray, resolution: float,
                    origin: Tuple[float, float]) -> np.ndarray:
    """점유 격자(OccupancyGrid, height × width)의 점유(≥ 50) 셀 중심 좌표 (N, 2) [map 프레임]."""
    iy, ix = np.nonzero(np.asarray(grid) >= 50)
    return np.column_stack([origin[0] + (ix + 0.5) * resolution,
                            origin[1] + (iy + 0.5) * resolution])


@dataclass(frozen=True)
class Registration:
    """
    지도 점유 점 → 월드 지면 진실 SE(2) 정합 결과 (world = R(dyaw)·map + (dx, dy)).

    map 프레임 = 월드 프레임이면 (dx, dy, dyaw) ≈ 0 이고 정합 전 거리(median_identity)도 작다.
    """

    dx: float
    dy: float
    dyaw: float                 # [rad]
    median_identity: float      # [m] 정합 전 점 → 최근접 구조물 셀 거리 중앙값
    mean_identity: float        # [m] 같은 거리의 평균 (이상치 포함)
    median_aligned: float       # [m] 정합 후 중앙값
    points: int

    @property
    def translation(self) -> float:
        return math.hypot(self.dx, self.dy)

    def identity_ok(self, t_tol: float = 0.05, yaw_tol_deg: float = 0.1,
                    median_tol: float = 0.05) -> bool:
        """항등 정합이 맞는가: 잔여 이동 ≤ t_tol, 잔여 회전 ≤ yaw_tol_deg, 정합 전 중앙값 ≤ median_tol."""
        return (self.points > 0 and self.translation <= t_tol
                and abs(math.degrees(self.dyaw)) <= yaw_tol_deg
                and self.median_identity <= median_tol)

    def as_dict(self) -> dict:
        return {'dx_m': round(self.dx, 4), 'dy_m': round(self.dy, 4),
                'dyaw_deg': round(math.degrees(self.dyaw), 4),
                'median_identity_m': round(self.median_identity, 4),
                'mean_identity_m': round(self.mean_identity, 4),
                'median_aligned_m': round(self.median_aligned, 4), 'points': self.points}


def register(points: np.ndarray, shapes: Sequence[Shape], resolution: float = 0.05,
             margin: float = 1.0, max_points: int = 20000) -> Registration:
    """
    지도 점유 점(map 프레임)을 지면 진실 거리장에 SE(2) 로 맞춘다 (soft-L1 최소제곱, 초기값 항등).

    항등에서 크게 어긋난 지도(예: SLAM 시작 자세 = map 원점인 16 m 어긋남)는 국소 최소에 걸리지만
    median_identity 가 커서 identity_ok() 가 거짓이 된다 — 판정 목적(항등 확인)에는 충분하다.
    """
    from scipy import ndimage, optimize

    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) == 0 or not shapes:
        return Registration(math.nan, math.nan, math.nan, math.inf, math.inf, math.inf, 0)
    if len(pts) > max_points:
        pts = pts[np.random.default_rng(0).choice(len(pts), max_points, replace=False)]
    x0, y0 = pts.min(axis=0) - margin
    x1, y1 = pts.max(axis=0) + margin
    w = int(math.ceil((x1 - x0) / resolution))
    h = int(math.ceil((y1 - y0) / resolution))
    occ = rasterize(shapes, w, h, resolution, (x0, y0))
    if not occ.any():
        return Registration(math.nan, math.nan, math.nan, math.inf, math.inf, math.inf,
                            len(pts))
    field = ndimage.distance_transform_edt(~occ) * resolution

    def dist(p: np.ndarray) -> np.ndarray:
        col = (p[:, 0] - x0) / resolution - 0.5
        row = (p[:, 1] - y0) / resolution - 0.5
        return ndimage.map_coordinates(field, [row, col], order=1, mode='nearest')

    def moved(params) -> np.ndarray:
        dx, dy, a = params
        c, s = math.cos(a), math.sin(a)
        return np.column_stack([dx + c * pts[:, 0] - s * pts[:, 1],
                                dy + s * pts[:, 0] + c * pts[:, 1]])

    d0 = dist(pts)
    res = optimize.least_squares(lambda p: dist(moved(p)), np.zeros(3), loss='soft_l1',
                                 f_scale=0.1, x_scale=[0.1, 0.1, 0.01])
    d1 = dist(moved(res.x))
    return Registration(float(res.x[0]), float(res.x[1]), float(res.x[2]),
                        float(np.median(d0)), float(np.mean(d0)), float(np.median(d1)),
                        len(pts))


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


def box_size(world_sdf: Path, name: str) -> Optional[Tuple[float, float, float]]:
    """월드 <model name=name> 의 첫 collision 상자 크기 (sx, sy, sz) — 도킹 마커 판 두께 등 (없으면 None)."""
    world = ET.parse(str(world_sdf)).getroot().find('world')
    for model in world.findall('model'):
        if model.get('name') == name:
            size = model.find('link/collision/geometry/box/size')
            if size is not None:
                sx, sy, sz = (float(t) for t in size.text.split())
                return sx, sy, sz
    return None


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
