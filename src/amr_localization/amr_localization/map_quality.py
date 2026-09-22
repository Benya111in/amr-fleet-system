r"""
맵 품질 평가 (명세 4.3 "맵 품질 평가 지표(맵 일관성, 구조물 정확도) 정의·측정", rclpy 비의존).

    ros2 run amr_localization map_quality --map maps/warehouse.yaml \
        --world src/amr_simulation/worlds/warehouse.sdf --models src/amr_simulation/models \
        --initial 0 0 0 --out logs/eval/map_quality

지면 진실 M_gt = 월드 SDF visual 의 스캔 평면(0.38 m) 단면 (world_geometry). SLAM 맵 M_est 는 map 프레임 =
로봇 시작 자세이므로 스폰 자세(--initial)로 초기 정렬한 뒤, 점유 셀 중심을 GT 거리장에 맞추는 SE(2)
최소제곱(soft-L1)으로 잔여 정렬을 구한다 (정렬량 자체도 보고 — 크면 맵 원점이 틀어진 것).
지표 (docs/algorithms/slam.md §3)
  구조물 정확도: ADNN (M_est 점유 셀 → 최근접 M_gt 점유 셀 평균 거리), p95, Chamfer(관측 영역),
                 허용오차 IoU_τ / precision / recall (τ = 1 셀), 기지 거리 오차 (마주 보는 외벽 간격),
                 랜드마크(기둥·랙) 중심 오차
  일관성      : 외벽 직선성 (벽 띠 안 점유 셀의 직선 적합 RMS 잔차, 각도 오차), 벽 두께 [셀],
                 점유/자유/미지 비율
"""

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
from typing import Dict, List, Optional, Sequence, Tuple

from amr_localization.world_geometry import DEFAULT_EXCLUDE, rasterize, Raster, WorldLoader
import numpy as np
from scipy import ndimage, optimize
import yaml


@dataclass
class MapImage:
    """map_server 형식 맵."""

    occupied: np.ndarray   # bool (row 0 = 최하단 y)
    free: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def cell_centers(self, mask: np.ndarray) -> np.ndarray:
        """마스크 셀 중심의 map 좌표 (N, 2)."""
        rows, cols = np.nonzero(mask)
        lx = (cols + 0.5) * self.resolution
        ly = (rows + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return np.stack([self.origin_x + c * lx - s * ly, self.origin_y + s * lx + c * ly], 1)


def read_pgm(path: Path) -> np.ndarray:
    """P5/P2 PGM → uint8/uint16 배열 (행 0 = 이미지 위쪽)."""
    data = path.read_bytes()
    tokens: List[bytes] = []
    pos = 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    pos += 1
    if magic == b'P5':
        dtype = np.uint8 if maxval < 256 else np.dtype('>u2')
        img = np.frombuffer(data, dtype=dtype, count=width * height, offset=pos)
    elif magic == b'P2':
        img = np.array(data[pos:].split()[:width * height], dtype=np.int64)
    else:
        raise ValueError(f'{path}: unsupported PGM magic {magic!r}')
    return img.reshape(height, width)


def load_map(yaml_path: Path) -> MapImage:
    """map_server YAML (+ PGM) → MapImage (map_server 의 trinary 규칙과 같게 판정)."""
    meta = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    img = read_pgm(yaml_path.parent / meta['image']).astype(float)
    maxval = 255.0 if img.max() <= 255 else 65535.0
    p = img / maxval if meta.get('negate', 0) else (maxval - img) / maxval
    occ = p > float(meta.get('occupied_thresh', 0.65))
    free = p < float(meta.get('free_thresh', 0.25))
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    return MapImage(occ[::-1].copy(), free[::-1].copy(), float(meta['resolution']),
                    float(origin[0]), float(origin[1]), float(origin[2]))


class GtField:
    """GT 점유격자의 거리장 (쌍선형 보간 조회)."""

    def __init__(self, raster: Raster) -> None:
        """raster: world_geometry.rasterize 결과."""
        self.raster = raster
        self.dist = ndimage.distance_transform_edt(~raster.occupied) * raster.resolution

    def distance(self, pts: np.ndarray) -> np.ndarray:
        """월드 좌표 점들의 최근접 GT 점유 셀 거리 [m] (격자 밖은 가장자리 값)."""
        r = self.raster
        col = (pts[:, 0] - r.origin_x) / r.resolution - 0.5
        row = (pts[:, 1] - r.origin_y) / r.resolution - 0.5
        return ndimage.map_coordinates(self.dist, [row, col], order=1, mode='nearest')


def se2_apply(pts: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """점 (N, 2) 에 SE(2) 적용."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]], 1)


def align(points: np.ndarray, field: GtField, initial: Tuple[float, float, float],
          f_scale: float = 0.1, max_points: int = 20000) -> Tuple[Tuple[float, float, float],
                                                                  float]:
    """
    SE(2) 정렬: min Σ ρ(d_gt(T p)²), ρ = soft-L1 (f_scale 이상 거리는 선형 벌점 → 이상치 둔감).

    반환: (x, y, yaw), 정렬 후 RMS 거리.
    """
    pts = points
    if len(pts) > max_points:
        pts = pts[np.random.default_rng(0).choice(len(pts), max_points, replace=False)]

    def residual(p):
        return field.distance(se2_apply(pts, *p))

    res = optimize.least_squares(residual, np.array(initial, dtype=float), loss='soft_l1',
                                 f_scale=f_scale, x_scale=[0.1, 0.1, 0.01])
    return tuple(float(v) for v in res.x), float(np.sqrt(np.mean(res.fun ** 2)))


def fit_line(pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """전최소제곱 직선: (중심, 방향 단위벡터, RMS 수직 잔차)."""
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    d = vt[0]
    n = np.array([-d[1], d[0]])
    return c, d, float(np.sqrt(np.mean(((pts - c) @ n) ** 2)))


@dataclass
class WallMetric:
    """외벽 1 개의 직선성."""

    name: str
    cells: int
    rms_residual: float        # [m]
    angle_error_deg: float
    offset_error: float        # [m] 적합 직선과 GT 면의 거리
    thickness_cells: float     # 벽 단위 길이당 점유 셀 수 / (1/res)


@dataclass
class MapQuality:
    """맵 품질 결과."""

    resolution: float
    alignment: Tuple[float, float, float]
    alignment_rms: float
    adnn: float
    adnn_median: float
    adnn_p95: float
    chamfer: float
    iou_tol: float
    precision_tol: float
    recall_tol: float
    occupied_ratio: float
    free_ratio: float
    unknown_ratio: float
    walls: List[WallMetric]
    known_distance: Dict[str, Dict[str, float]]
    landmark_error_mean: float
    landmark_error_max: float
    landmark_count: int


def wall_metrics(pts: np.ndarray, room: Tuple[float, float, float, float], band: float,
                 resolution: float) -> Tuple[List[WallMetric], Dict[str, Dict[str, float]]]:
    """
    외벽 4 개 (방 안쪽 면 xmin/xmax/ymin/ymax) 의 직선성과 마주 보는 벽 간격 오차.

    벽 띠 = GT 면에서 ±band, 벽 방향 양 끝 1 m 는 모서리 영향을 피해 제외. 벽을 따라 셀 폭 구간마다
    방 안쪽에 가장 가까운 점유 셀(= LiDAR 가 본 표면)만 골라 직선을 적합하고, 셀 중심→안쪽 모서리
    반 셀 보정을 한 위치를 표면 위치로 쓴다. 두께 = 띠 안 점유 셀 수 / 구간 수.
    """
    xmin, xmax, ymin, ymax = room
    # (법선 축, GT 면 위치, 벽 방향 범위, 방 안쪽 방향 부호)
    walls = {'west': (0, xmin, (ymin, ymax), 1.0), 'east': (0, xmax, (ymin, ymax), -1.0),
             'south': (1, ymin, (xmin, xmax), 1.0), 'north': (1, ymax, (xmin, xmax), -1.0)}
    out: List[WallMetric] = []
    fitted: Dict[str, float] = {}
    for name, (a, value, (lo, hi), inward) in walls.items():
        b = 1 - a
        sel = (np.abs(pts[:, a] - value) <= band) & (pts[:, b] > lo + 1.0) & (pts[:, b] < hi - 1.0)
        wp = pts[sel]
        if len(wp) < 10:
            continue
        bins = np.floor(wp[:, b] / resolution).astype(np.int64)
        order = np.lexsort((-inward * wp[:, a], bins))     # 구간별로 안쪽 끝이 먼저
        first = np.ones(len(order), dtype=bool)
        first[1:] = bins[order][1:] != bins[order][:-1]
        surface = wp[order][first]
        if len(surface) < 10:
            continue
        c, d, rms = fit_line(surface)
        gt_dir = np.array([0.0, 1.0]) if a == 0 else np.array([1.0, 0.0])
        ang = math.degrees(math.acos(min(1.0, abs(float(d @ gt_dir)))))
        position = float(c[a]) + inward * resolution / 2.0
        fitted[name] = position
        out.append(WallMetric(name, int(len(surface)), rms, ang, position - value,
                              float(len(wp)) / len(surface)))
    known: Dict[str, Dict[str, float]] = {}
    for pair, (w1, w2, gt) in {'width_x': ('west', 'east', xmax - xmin),
                               'width_y': ('south', 'north', ymax - ymin)}.items():
        if w1 in fitted and w2 in fitted:
            measured = fitted[w2] - fitted[w1]
            known[pair] = {'gt': gt, 'measured': measured, 'error': measured - gt,
                           'error_percent': 100.0 * (measured - gt) / gt}
    return out, known


def landmark_errors(gt: Raster, est_pts: np.ndarray, max_area: float = 4.0,
                    obs_dist: float = 0.15) -> List[float]:
    """
    작은 GT 연결 성분(기둥·랙, 면적 ≤ max_area m²) 의 관측된 부분 중심과 그 근처 M_est 점 중심의 거리.

    LiDAR 는 표면만 보므로 GT 성분 중 M_est 점에서 obs_dist 이내인 셀만 GT 쪽 중심에 쓴다.
    """
    labels, n = ndimage.label(gt.occupied)
    if n == 0 or len(est_pts) == 0:
        return []
    errors = []
    cell_area = gt.resolution ** 2
    tree_pts = est_pts
    for idx, slc in enumerate(ndimage.find_objects(labels), start=1):
        comp = labels[slc] == idx
        if comp.sum() * cell_area > max_area:
            continue
        rows, cols = np.nonzero(comp)
        gpts = np.stack([gt.origin_x + (cols + slc[1].start + 0.5) * gt.resolution,
                         gt.origin_y + (rows + slc[0].start + 0.5) * gt.resolution], 1)
        lo = gpts.min(axis=0) - 0.3
        hi = gpts.max(axis=0) + 0.3
        near = tree_pts[np.all((tree_pts >= lo) & (tree_pts <= hi), axis=1)]
        if len(near) < 5:
            continue
        d = np.min(np.linalg.norm(gpts[:, None, :] - near[None, :, :], axis=2), axis=1)
        observed = gpts[d <= obs_dist]
        if len(observed) < 3:
            continue
        errors.append(float(np.linalg.norm(observed.mean(axis=0) - near.mean(axis=0))))
    return errors


def evaluate(est: MapImage, gt: Raster, initial: Tuple[float, float, float],
             room: Tuple[float, float, float, float], band: float = 0.3,
             tolerance_cells: int = 1) -> MapQuality:
    """전체 지표 계산."""
    field = GtField(gt)
    occ_pts = est.cell_centers(est.occupied)
    if len(occ_pts) == 0:
        raise ValueError('estimated map has no occupied cells')
    pose, rms = align(occ_pts, field, initial)
    occ_w = se2_apply(occ_pts, *pose)
    d_est = field.distance(occ_w)

    # 관측 영역 안의 GT 점유 셀 (자유/점유로 판정된 M_est 셀에서 2 셀 이내) 만 재현율·Chamfer 대상
    known_pts = se2_apply(est.cell_centers(est.free | est.occupied), *pose)
    known_mask = np.zeros_like(gt.occupied)
    col = np.floor((known_pts[:, 0] - gt.origin_x) / gt.resolution).astype(int)
    row = np.floor((known_pts[:, 1] - gt.origin_y) / gt.resolution).astype(int)
    ok = (col >= 0) & (col < gt.occupied.shape[1]) & (row >= 0) & (row < gt.occupied.shape[0])
    known_mask[row[ok], col[ok]] = True
    known_mask = ndimage.binary_dilation(known_mask, iterations=2)
    gt_obs = gt.occupied & known_mask

    est_mask = np.zeros_like(gt.occupied)
    col = np.floor((occ_w[:, 0] - gt.origin_x) / gt.resolution).astype(int)
    row = np.floor((occ_w[:, 1] - gt.origin_y) / gt.resolution).astype(int)
    ok = (col >= 0) & (col < gt.occupied.shape[1]) & (row >= 0) & (row < gt.occupied.shape[0])
    est_mask[row[ok], col[ok]] = True
    est_dist = ndimage.distance_transform_edt(~est_mask) * gt.resolution
    tol = tolerance_cells * gt.resolution + 1e-9
    tp = int(np.count_nonzero(d_est <= tol))
    fp = int(len(d_est) - tp)
    fn = int(np.count_nonzero(gt_obs & (est_dist > tol)))
    gt_to_est = est_dist[gt_obs]

    walls, known = wall_metrics(occ_w, room, band, est.resolution)
    lm = landmark_errors(gt, occ_w)
    total = est.occupied.size
    return MapQuality(
        resolution=est.resolution, alignment=pose, alignment_rms=rms,
        adnn=float(d_est.mean()), adnn_median=float(np.median(d_est)),
        adnn_p95=float(np.percentile(d_est, 95)),
        chamfer=float(0.5 * (d_est.mean() + (gt_to_est.mean() if gt_to_est.size else 0.0))),
        iou_tol=tp / max(tp + fp + fn, 1), precision_tol=tp / max(tp + fp, 1),
        recall_tol=1.0 - fn / max(int(gt_obs.sum()), 1),
        occupied_ratio=float(est.occupied.sum()) / total,
        free_ratio=float(est.free.sum()) / total,
        unknown_ratio=float((~est.occupied & ~est.free).sum()) / total,
        walls=walls, known_distance=known,
        landmark_error_mean=float(np.mean(lm)) if lm else math.nan,
        landmark_error_max=float(np.max(lm)) if lm else math.nan,
        landmark_count=len(lm))


def to_markdown(q: MapQuality, name: str = '') -> str:
    """Markdown 리포트."""
    x, y, yaw = q.alignment
    lines = [f'# 맵 품질 평가 {name}'.rstrip(), '',
             f'- 해상도 {q.resolution:.3f} m (명세 ≤ 0.05 m)',
             f'- 정렬 (map→world) x={x:.3f} m, y={y:.3f} m, yaw={math.degrees(yaw):.3f} deg, '
             f'정렬 후 RMS {q.alignment_rms:.4f} m', '',
             '| 지표 | 값 |', '| --- | --- |',
             f'| ADNN 평균 / 중앙 / p95 [m] | {q.adnn:.4f} / {q.adnn_median:.4f} / '
             f'{q.adnn_p95:.4f} |',
             f'| Chamfer (관측 영역) [m] | {q.chamfer:.4f} |',
             f'| IoU_τ / precision / recall (τ = 1 셀) | {q.iou_tol:.3f} / '
             f'{q.precision_tol:.3f} / {q.recall_tol:.3f} |',
             f'| 랜드마크 중심 오차 평균 / 최대 [m] (n={q.landmark_count}) | '
             f'{q.landmark_error_mean:.4f} / {q.landmark_error_max:.4f} |',
             f'| 점유 / 자유 / 미지 비율 | {q.occupied_ratio:.4f} / {q.free_ratio:.4f} / '
             f'{q.unknown_ratio:.4f} |']
    for k, v in q.known_distance.items():
        lines.append(f"| 기지 거리 {k} (GT {v['gt']:.2f} m) | 측정 {v['measured']:.3f} m, "
                     f"오차 {v['error']:+.3f} m ({v['error_percent']:+.3f} %) |")
    if q.walls:
        lines += ['', '| 외벽 | 셀 | 직선 RMS [m] | 각도 오차 [deg] | 면 오프셋 [m] | 두께 [셀] |',
                  '| --- | --- | --- | --- | --- | --- |']
        for w in q.walls:
            lines.append(f'| {w.name} | {w.cells} | {w.rms_residual:.4f} | '
                         f'{w.angle_error_deg:.3f} | {w.offset_error:+.4f} | '
                         f'{w.thickness_cells:.2f} |')
    return '\n'.join(lines) + '\n'


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI 진입점."""
    ap = argparse.ArgumentParser(description='occupancy map quality vs world ground truth')
    ap.add_argument('--map', required=True, help='map_server YAML')
    ap.add_argument('--world', required=True)
    ap.add_argument('--models', action='append', default=[])
    ap.add_argument('--height', type=float, default=0.38, help='스캔 평면 높이 [m]')
    ap.add_argument('--initial', type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=('X', 'Y', 'YAW'), help='map 원점의 월드 자세 (스폰 자세)')
    ap.add_argument('--room', type=float, nargs=4, default=[-29.8, 29.8, -19.8, 19.8],
                    metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'), help='외벽 안쪽 면')
    ap.add_argument('--band', type=float, default=0.3)
    ap.add_argument('--out', default='', help='결과 디렉토리 (map_quality.json/.md)')
    args = ap.parse_args(argv)
    est = load_map(Path(args.map))
    loader = WorldLoader([Path(p) for p in args.models], DEFAULT_EXCLUDE)
    shapes = loader.load_world(Path(args.world))
    xmin, xmax, ymin, ymax = args.room
    gt = rasterize(shapes, args.height, est.resolution,
                   (xmin - 1.0, xmax + 1.0, ymin - 1.0, ymax + 1.0))
    q = evaluate(est, gt, tuple(args.initial), tuple(args.room), args.band)
    md = to_markdown(q, Path(args.map).stem)
    print(md)
    if args.out:
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / 'map_quality.md').write_text(md, encoding='utf-8')
        (out / 'map_quality.json').write_text(json.dumps(asdict(q), indent=2), encoding='utf-8')
    return 0


if __name__ == '__main__':
    sys.exit(main())
