r"""
맵 품질 평가 (명세 4.3 "맵 품질 평가 지표(맵 일관성, 구조물 정확도) 정의·측정", rclpy 비의존).

    ros2 run amr_localization map_quality --map maps/warehouse.yaml \
        --world src/amr_simulation/worlds/warehouse.sdf --models src/amr_simulation/models \
        --out logs/eval/map_quality
    # SLAM 원본 맵을 월드 프레임으로 등록(강체 정렬 → 영상 재표본화)해 저장
    ros2 run amr_localization map_quality --map logs/map_new/warehouse.yaml ... \
        --register-out maps/warehouse

지면 진실 M_gt = 월드 SDF visual 의 LiDAR 스캔 평면 높이 단면 (world_geometry; 높이 기본값은
config/robot_params.yaml base_link_height + config/sensors.yaml lidar.extrinsic.z = 0.20 m).
map 프레임 = 월드 프레임 (통합 계약) 이므로 먼저 그대로 비교(ADNN_0)하고, 점유 셀 중심을 GT 거리장에 맞추는
SE(2) 최소제곱(soft-L1)으로 잔여 정렬을 구한다 (정렬량 자체도 보고 — 크면 맵 원점·방향이 틀어진 것).
지표 (docs/algorithms/slam.md §3)
  구조물 정확도: ADNN (M_est 점유 셀 → 최근접 M_gt 점유 셀 평균 거리), p95, Chamfer(관측 영역),
                 허용오차 IoU_τ / precision / recall (τ = 1 셀), 기지 거리 오차 (마주 보는 외벽 간격),
                 랜드마크(기둥·랙) 중심 오차
  일관성      : 외벽 직선성 (벽 띠 점유 셀 중앙값의 직선 적합 RMS 잔차, 각도 오차), 벽 두께 [셀],
                 점유/자유/미지 비율 (map_server 와 같은 trinary 규칙 — YAML free_thresh 가 0.196 이상이면
                 미지(205)가 자유로 읽힌다는 경고를 낸다)
"""

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import sys
from typing import Dict, List, Optional, Sequence, Tuple

from amr_localization.world_geometry import (DEFAULT_EXCLUDE, rasterize, Raster,
                                             scan_plane_height, WorldLoader)
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


# map_saver_cli (nav2 map_io, trinary) 가 쓰는 픽셀 값: 점유 0, 자유 254, 미지 205
TRINARY_UNKNOWN = 205
# 미지(205) 를 미지로 읽는 free_thresh: p = (255 − 205)/255 = 0.196 보다 작아야 한다 (ROS1 규약 0.196).
# nav2 map_saver_cli 기본 YAML 의 0.25 는 미지를 자유로 읽게 한다 (map_server /map 에 −1 셀이 없음, 실측).
FREE_THRESH = 0.19


def unknown_loads_as_free(meta: dict) -> bool:
    """이 YAML 로 map_server 가 미지 픽셀(205)을 자유(0)로 읽는지 (trinary, negate 0 기준)."""
    p = (255.0 - TRINARY_UNKNOWN) / 255.0
    if meta.get('negate', 0):
        p = TRINARY_UNKNOWN / 255.0
    return p < float(meta.get('free_thresh', 0.25))


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
    adnn_initial: float        # [m] 정렬 전(초기 자세 그대로, map = 월드면 (0,0,0)) ADNN
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
    pillar_distance: Dict[str, Dict[str, float]]
    landmark_error_mean: float
    landmark_error_max: float
    landmark_count: int


def wall_metrics(pts: np.ndarray, room: Tuple[float, float, float, float], band: float,
                 resolution: float) -> Tuple[List[WallMetric], Dict[str, Dict[str, float]]]:
    """
    외벽 4 개 (방 안쪽 면 xmin/xmax/ymin/ymax) 의 직선성과 마주 보는 벽 간격 오차.

    벽 띠 = GT 면에서 ±band, 벽 방향 양 끝 1 m 는 모서리 영향을 피해 제외. 벽을 따라 셀 폭 구간마다 띠 안
    점유 셀 중심의 법선 좌표 **중앙값**을 그 구간의 면 위치로 두고 직선을 적합한다 (ADNN 과 같은 셀 중심 규약,
    반 셀 보정 없음). 가장 안쪽 셀을 고르던 이전 추정은 LiDAR 잡음(σ 0.03 m)이 띠를 2.8~3.2 셀로 두껍게
    만들면 면을 방 안쪽으로 4~9 cm 옮겨 맵이 작아 보이게 했다 (test_map_quality 의 띠 두께 불변 시험).
    SLAM(Karto) 격자는 적중/통과 비로 점유를 정하므로 띠가 면 뒤로 조금 더 두껍다: 1D 모델에서 σ_eff
    0.03~0.05 m 의 완전한 맵의 띠 중앙값이 면 뒤 +0.7~+3.4 cm (slam.md §3) — 간격은 그 두 배만큼 커 보인다.
    두께 = 띠 안 점유 셀 수 / 구간 수.
    """
    xmin, xmax, ymin, ymax = room
    # (법선 축, GT 면 위치, 벽 방향 범위)
    walls = {'west': (0, xmin, (ymin, ymax)), 'east': (0, xmax, (ymin, ymax)),
             'south': (1, ymin, (xmin, xmax)), 'north': (1, ymax, (xmin, xmax))}
    out: List[WallMetric] = []
    fitted: Dict[str, float] = {}
    for name, (a, value, (lo, hi)) in walls.items():
        b = 1 - a
        sel = (np.abs(pts[:, a] - value) <= band) & (pts[:, b] > lo + 1.0) & (pts[:, b] < hi - 1.0)
        wp = pts[sel]
        if len(wp) < 10:
            continue
        bins = np.floor(wp[:, b] / resolution).astype(np.int64)
        keys, inverse = np.unique(bins, return_inverse=True)
        if len(keys) < 10:
            continue
        surface = np.array([[np.median(wp[inverse == k, a]), np.mean(wp[inverse == k, b])]
                            for k in range(len(keys))])
        if a == 1:
            surface = surface[:, ::-1]          # (x, y) 순서로
        c, d, rms = fit_line(surface)
        gt_dir = np.array([0.0, 1.0]) if a == 0 else np.array([1.0, 0.0])
        ang = math.degrees(math.acos(min(1.0, abs(float(d @ gt_dir)))))
        position = float(c[a])
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


# 월드 생성기(gen_warehouse_world.py PILLARS) 의 독립 기둥 중심 — 사방에서 관측되므로 띠 규약 편향이 상쇄된다
DEFAULT_PILLARS = ((-10.0, 12.0), (10.0, 12.0), (-10.0, -12.0), (10.0, -12.0),
                   (-25.0, 6.0), (25.0, 6.0), (-25.0, -6.0), (25.0, -6.0))


def pillar_distances(est_pts: np.ndarray, pillars: Sequence[Tuple[float, float]],
                     half: float = 0.45) -> Dict[str, Dict[str, float]]:
    """
    기둥 중심 간 기지 거리 (규약 무관 축척 지표).

    기둥 중심 = GT 중심 ±half 상자 안 M_est 점유 셀 중심의 평균. 띠가 면 뒤로 치우치는 양은 네 면에서 서로
    상쇄되므로 벽 간격(띠 규약에 의존)과 달리 축척·국소 변형만 본다. 쌍: 같은 x·같은 y 인 모든 기둥 쌍.
    """
    centers = {}
    for px, py in pillars:
        sel = np.all(np.abs(est_pts - np.array([px, py])) <= half, axis=1)
        if np.count_nonzero(sel) >= 8:
            centers[(px, py)] = est_pts[sel].mean(axis=0)
    out: Dict[str, Dict[str, float]] = {}
    keys = sorted(centers)
    for i, a in enumerate(keys):
        for b in keys[i + 1:]:
            if a[0] != b[0] and a[1] != b[1]:
                continue
            gt = math.hypot(b[0] - a[0], b[1] - a[1])
            measured = float(np.linalg.norm(centers[b] - centers[a]))
            out[f'({a[0]:g},{a[1]:g})-({b[0]:g},{b[1]:g})'] = {
                'gt': gt, 'measured': measured, 'error': measured - gt,
                'error_percent': 100.0 * (measured - gt) / gt}
    return out


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
             tolerance_cells: int = 1,
             pillars: Sequence[Tuple[float, float]] = DEFAULT_PILLARS) -> MapQuality:
    """전체 지표 계산."""
    field = GtField(gt)
    occ_pts = est.cell_centers(est.occupied)
    if len(occ_pts) == 0:
        raise ValueError('estimated map has no occupied cells')
    adnn_initial = float(field.distance(se2_apply(occ_pts, *initial)).mean())
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
        resolution=est.resolution, alignment=pose, alignment_rms=rms, adnn_initial=adnn_initial,
        adnn=float(d_est.mean()), adnn_median=float(np.median(d_est)),
        adnn_p95=float(np.percentile(d_est, 95)),
        chamfer=float(0.5 * (d_est.mean() + (gt_to_est.mean() if gt_to_est.size else 0.0))),
        iou_tol=tp / max(tp + fp + fn, 1), precision_tol=tp / max(tp + fp, 1),
        recall_tol=1.0 - fn / max(int(gt_obs.sum()), 1),
        occupied_ratio=float(est.occupied.sum()) / total,
        free_ratio=float(est.free.sum()) / total,
        unknown_ratio=float((~est.occupied & ~est.free).sum()) / total,
        walls=walls, known_distance=known, pillar_distance=pillar_distances(occ_w, pillars),
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
             f'| ADNN 정렬 전 (map 프레임 그대로) [m] | {q.adnn_initial:.4f} |',
             f'| ADNN 평균 / 중앙 / p95 (정렬 후) [m] | {q.adnn:.4f} / {q.adnn_median:.4f} / '
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
    for k, v in q.pillar_distance.items():
        lines.append(f"| 기둥 중심 간 {k} (GT {v['gt']:.2f} m) | 측정 {v['measured']:.3f} m, "
                     f"오차 {v['error']:+.3f} m ({v['error_percent']:+.3f} %) |")
    if q.walls:
        lines += ['', '| 외벽 | 셀 | 직선 RMS [m] | 각도 오차 [deg] | 면 오프셋 [m] | 두께 [셀] |',
                  '| --- | --- | --- | --- | --- | --- |']
        for w in q.walls:
            lines.append(f'| {w.name} | {w.cells} | {w.rms_residual:.4f} | '
                         f'{w.angle_error_deg:.3f} | {w.offset_error:+.4f} | '
                         f'{w.thickness_cells:.2f} |')
    return '\n'.join(lines) + '\n'


def min_resample_rotation(shape: Tuple[int, int], resolution: float) -> float:
    """
    재표본화가 의미 있는 최소 회전 [rad]: 맵 반대각선 끝의 회전 변위가 반 셀.

    그보다 작은 회전은 최근접 재표본화가 셀 단위 계단(±반 셀)으로만 옮겨 오히려 국소 오차가 커진다.
    """
    half_diag = 0.5 * resolution * math.hypot(shape[0], shape[1])
    return 0.5 * resolution / max(half_diag, 1e-9)


def register_map(yaml_path: Path, pose: Tuple[float, float, float], out_stem: Path,
                 free_thresh: float = FREE_THRESH) -> Tuple[Path, Path]:
    """
    맵을 SE(2) pose (map → world) 로 월드 프레임에 등록해 PGM + YAML 로 쓴다 (원점 yaw 0).

    AMCL·costmap 은 OccupancyGrid 원점 yaw 를 무시하므로 회전은 YAML 이 아니라 영상 재표본화로 넣는다:
    출력 셀 중심을 역변환한 입력 좌표의 셀 값(최근접, trinary 픽셀 값 0/205/254 보존)을 쓰고 입력 밖은 미지.
    회전이 min_resample_rotation 보다 작으면 재표본화 없이 원점만 옮긴다 (픽셀 동일, 남은 회전은 반 셀 미만).
    free_thresh 는 미지를 미지로 읽는 값으로 쓴다.
    """
    meta = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    img = read_pgm(yaml_path.parent / meta['image'])[::-1]           # 행 0 = 최하단 y
    res = float(meta['resolution'])
    ox, oy, oyaw = (float(v) for v in meta.get('origin', [0.0, 0.0, 0.0]))
    x, y, yaw = pose
    total = oyaw + yaw
    if abs(total) < min_resample_rotation(img.shape, res):
        yaw, total = -oyaw, 0.0
    # 입력 원점(셀 (0,0) 모서리)의 월드 좌표
    c, s = math.cos(yaw), math.sin(yaw)
    wx0, wy0 = x + c * ox - s * oy, y + s * ox + c * oy
    if total == 0.0:
        out, origin = img, (wx0, wy0)
    else:
        h, w = img.shape
        corners = np.array([[0, 0], [w, 0], [0, h], [w, h]], dtype=float) * res
        ct, st = math.cos(total), math.sin(total)
        cw = np.stack([wx0 + ct * corners[:, 0] - st * corners[:, 1],
                       wy0 + st * corners[:, 0] + ct * corners[:, 1]], 1)
        origin = (float(cw[:, 0].min()), float(cw[:, 1].min()))
        width = int(math.ceil((cw[:, 0].max() - origin[0]) / res))
        height = int(math.ceil((cw[:, 1].max() - origin[1]) / res))
        rows, cols = np.mgrid[0:height, 0:width]
        px = origin[0] + (cols + 0.5) * res - wx0
        py = origin[1] + (rows + 0.5) * res - wy0
        lx, ly = ct * px + st * py, -st * px + ct * py                 # 입력 격자 좌표 [m]
        ic, ir = np.floor(lx / res).astype(int), np.floor(ly / res).astype(int)
        ok = (ic >= 0) & (ic < w) & (ir >= 0) & (ir < h)
        out = np.full((height, width), TRINARY_UNKNOWN, dtype=img.dtype)
        out[ok] = img[ir[ok], ic[ok]]
    out_stem.parent.mkdir(parents=True, exist_ok=True)
    pgm, yml = out_stem.with_suffix('.pgm'), out_stem.with_suffix('.yaml')
    flipped = np.ascontiguousarray(out[::-1]).astype(np.uint8)
    with pgm.open('wb') as f:
        f.write(f'P5\n{flipped.shape[1]} {flipped.shape[0]}\n255\n'.encode())
        f.write(flipped.tobytes())
    yml.write_text(
        f'image: {pgm.name}\nmode: trinary\nresolution: {res}\n'
        f'origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0]\nnegate: 0\n'
        f'occupied_thresh: {float(meta.get("occupied_thresh", 0.65))}\n'
        f'free_thresh: {free_thresh}\n', encoding='utf-8')
    return pgm, yml


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI 진입점."""
    ap = argparse.ArgumentParser(description='occupancy map quality vs world ground truth')
    ap.add_argument('--map', required=True, help='map_server YAML')
    ap.add_argument('--world', required=True)
    ap.add_argument('--models', action='append', default=[])
    ap.add_argument('--height', type=float, default=None,
                    help='스캔 평면 높이 [m] (기본: ROS_WS/config 의 base_link_height + lidar z)')
    ap.add_argument('--initial', type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=('X', 'Y', 'YAW'),
                    help='map 원점의 월드 자세 (map 프레임 = 월드면 0 0 0)')
    ap.add_argument('--room', type=float, nargs=4, default=[-30.0, 30.0, -20.0, 20.0],
                    metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'),
                    help='외벽 안쪽 면 (gen_warehouse_world.py HALF_X/HALF_Y)')
    ap.add_argument('--band', type=float, default=0.3)
    ap.add_argument('--out', default='', help='결과 디렉토리 (map_quality.json/.md)')
    ap.add_argument('--register-out', default='',
                    help='정렬 결과로 월드 프레임에 등록한 맵을 쓸 경로 (확장자 제외)')
    args = ap.parse_args(argv)
    height = scan_plane_height() if args.height is None else args.height
    meta = yaml.safe_load(Path(args.map).read_text(encoding='utf-8'))
    if unknown_loads_as_free(meta):
        print(f"WARNING: free_thresh {meta.get('free_thresh')} ≥ 0.196 — 미지 픽셀(205)이 자유로 읽힌다 "
              f'(map_server 와 같은 규칙으로 평가; 등록 출력은 free_thresh {FREE_THRESH})')
    est = load_map(Path(args.map))
    loader = WorldLoader([Path(p) for p in args.models], DEFAULT_EXCLUDE)
    shapes = loader.load_world(Path(args.world))
    xmin, xmax, ymin, ymax = args.room
    gt = rasterize(shapes, height, est.resolution,
                   (xmin - 1.0, xmax + 1.0, ymin - 1.0, ymax + 1.0))
    q = evaluate(est, gt, tuple(args.initial), tuple(args.room), args.band)
    md = to_markdown(q, Path(args.map).stem) + f'\n- 스캔 평면 높이 {height:.3f} m\n'
    print(md)
    if args.out:
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / 'map_quality.md').write_text(md, encoding='utf-8')
        (out / 'map_quality.json').write_text(json.dumps(asdict(q), indent=2), encoding='utf-8')
    if args.register_out:
        pgm, yml = register_map(Path(args.map), q.alignment, Path(args.register_out))
        print(f'registered map (map→world {q.alignment[0]:+.4f} m, {q.alignment[1]:+.4f} m, '
              f'{math.degrees(q.alignment[2]):+.4f} deg) → {pgm}, {yml}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
