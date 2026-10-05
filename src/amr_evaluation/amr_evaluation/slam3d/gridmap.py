"""
같은 격자 공간의 2D 지도 지표 — 2D SLAM 지도와 3D 지도의 높이 대역 투영을 같은 저울에 올린다.

GT 는 gen_warehouse_world.py --gt-dir 의 래스터 (기본 0.01 m: 단면 0.20 m, 대역 0.05–0.40 / 0.05–2.0 m).
추정 지도는 map_server 형식 (PGM + YAML, trinary). 기존 amr_localization/map_quality.py 와 같은 규약:
  - 점은 점유 셀 **중심**, 정렬 없음(ADNN₀) — map 프레임 = 월드 프레임 계약.
  - GT 거리장 = GT 점유 셀 중심까지의 유클리드 거리변환을 쌍선형 보간 (map_quality.GtField 와 같다).
  - 관측 영역 = 추정 지도의 알려진 셀(자유 ∪ 점유)을 observed_margin [m] 만큼 넓힌 곳. 재현율·Chamfer 의
    GT 쪽은 그 안의 GT 점유 셀만 센다 (map_quality 는 2 셀 = 0.10 m @0.05).
해상도가 다른 GT(0.01)와 추정(0.05)을 같이 쓰므로 거리 지표는 해상도 무관하게 정의하고, IoU 는 공통 격자로
옮긴 뒤(셀 중심 기준 any-occupied 풀링) 한 해상도 안에서 센다.
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

TRINARY_UNKNOWN = 205


@dataclass
class Grid:
    """점유 격자. occupied/known 은 bool (행 0 = 최하단 y). 셀 (0, 0) 모서리 = (origin_x, origin_y)."""

    occupied: np.ndarray
    known: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    @property
    def shape(self) -> Tuple[int, int]:
        """(행, 열)."""
        return self.occupied.shape

    def centers(self, mask: np.ndarray) -> np.ndarray:
        """마스크 셀 중심의 월드 좌표 (N, 2)."""
        rows, cols = np.nonzero(mask)
        lx = (cols + 0.5) * self.resolution
        ly = (rows + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return np.stack([self.origin_x + c * lx - s * ly, self.origin_y + s * lx + c * ly], 1)

    def lookup(self, pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """월드 점 → (행, 열, 격자 안 여부). 원점 yaw 는 0 이어야 한다."""
        if abs(self.origin_yaw) > 1e-12:
            raise ValueError('원점 yaw ≠ 0 인 격자는 조회 대상이 될 수 없다 (먼저 재표본화)')
        col = np.floor((pts[:, 0] - self.origin_x) / self.resolution).astype(np.int64)
        row = np.floor((pts[:, 1] - self.origin_y) / self.resolution).astype(np.int64)
        ok = (col >= 0) & (col < self.shape[1]) & (row >= 0) & (row < self.shape[0])
        return row, col, ok


def read_pgm(path: Path) -> np.ndarray:
    """P5 (8/16 bit) PGM → 배열 (행 0 = 영상 위쪽)."""
    data = Path(path).read_bytes()
    tokens, pos = [], 0
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
    if tokens[0] != b'P5':
        raise ValueError(f'{path}: P5 PGM 만 지원 ({tokens[0]!r})')
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    dtype = np.uint8 if maxval < 256 else np.dtype('>u2')
    return np.frombuffer(data, dtype=dtype, count=w * h, offset=pos + 1).reshape(h, w)


def read_map_yaml(path: str) -> Grid:
    """
    map_server YAML (+ PGM) → Grid.

    map_server trinary 규칙: p > occupied_thresh 점유, p < free_thresh 자유.
    """
    import yaml
    path = Path(path)
    meta = yaml.safe_load(path.read_text(encoding='utf-8'))
    img = read_pgm(path.parent / meta['image']).astype(float)
    maxval = 255.0 if img.max() <= 255 else 65535.0
    p = img / maxval if meta.get('negate', 0) else (maxval - img) / maxval
    occ = p > float(meta.get('occupied_thresh', 0.65))
    free = p < float(meta.get('free_thresh', 0.25))
    o = meta.get('origin', [0.0, 0.0, 0.0])
    return Grid(occ[::-1].copy(), (occ | free)[::-1].copy(), float(meta['resolution']),
                float(o[0]), float(o[1]), float(o[2]) if len(o) > 2 else 0.0)


def distance_field(grid: Grid) -> np.ndarray:
    """점유 셀 중심까지의 거리장 [m] (셀 중심 격자)."""
    from scipy import ndimage
    if not grid.occupied.any():
        return np.full(grid.shape, np.inf)
    return ndimage.distance_transform_edt(~grid.occupied) * grid.resolution


def sample_field(field: np.ndarray, grid: Grid, pts: np.ndarray) -> np.ndarray:
    """셀 중심 격자의 값을 월드 점에서 쌍선형 보간 (격자 밖은 가장자리 값; map_quality.GtField 와 같다)."""
    from scipy import ndimage
    col = (pts[:, 0] - grid.origin_x) / grid.resolution - 0.5
    row = (pts[:, 1] - grid.origin_y) / grid.resolution - 0.5
    return ndimage.map_coordinates(field, [row, col], order=1, mode='nearest')


def observed_mask(est: Grid, target: Grid, margin: float, free_only: bool = False) -> np.ndarray:
    """
    관측 영역: target 셀 중 중심이 est 의 알려진 셀 안인 것을 margin [m] 만큼 넓힌 마스크.

    free_only 면 알려진 셀 대신 **자유** 셀만 넓힌다 — 면(경계) 재현율용: 벽 뒷면처럼 자유 공간에 닿지 않은
    면은 관측 영역에 들지 않는다.
    """
    from scipy import ndimage
    rows, cols = np.mgrid[0:target.shape[0], 0:target.shape[1]]
    pts = np.stack([target.origin_x + (cols.ravel() + 0.5) * target.resolution,
                    target.origin_y + (rows.ravel() + 0.5) * target.resolution], 1)
    known = np.zeros(len(pts), dtype=bool)
    src = est.known & ~est.occupied if free_only else est.known
    if abs(est.origin_yaw) < 1e-12:
        r, c, ok = est.lookup(pts)
        known[ok] = src[r[ok], c[ok]]
    else:
        cs, sn = math.cos(-est.origin_yaw), math.sin(-est.origin_yaw)
        d = pts - [est.origin_x, est.origin_y]
        lx, ly = cs * d[:, 0] - sn * d[:, 1], sn * d[:, 0] + cs * d[:, 1]
        c = np.floor(lx / est.resolution).astype(np.int64)
        r = np.floor(ly / est.resolution).astype(np.int64)
        ok = (c >= 0) & (c < est.shape[1]) & (r >= 0) & (r < est.shape[0])
        known[ok] = src[r[ok], c[ok]]
    mask = known.reshape(target.shape)
    if margin > 0:
        dist = ndimage.distance_transform_edt(~mask) * target.resolution
        mask = dist <= margin + 1e-9
    return mask


def paint(points: np.ndarray, target: Grid) -> np.ndarray:
    """점들이 떨어지는 target 셀을 True 로."""
    out = np.zeros(target.shape, dtype=bool)
    r, c, ok = target.lookup(points)
    out[r[ok], c[ok]] = True
    return out


def surface(occ: np.ndarray) -> np.ndarray:
    """경계 셀: 점유 셀 중 8 이웃에 비점유가 있는 것 (격자 가장자리도 경계). LiDAR 가 볼 수 있는 면."""
    from scipy import ndimage
    inner = ndimage.binary_erosion(occ, structure=np.ones((3, 3), dtype=bool), border_value=0)
    return occ & ~inner


def grid_metrics(est: Grid, gt: Grid, tau: Optional[float] = None,
                 observed_margin: float = 0.10) -> Dict[str, float]:
    """
    추정 지도 대 GT 래스터 (정렬 없음). tau 기본 = 추정 해상도 (map_quality 의 τ = 1 셀).

    GT 기준이 둘이다.
      filled  : GT 점유 전체 (고체 단면 — map_quality 와 같다). 고체 **안쪽**에 떨어진 추정 셀은 거리 0 이다.
      surface : GT 경계 셀 (surface()) — 면 뒤로 밀린 추정 셀도 그만큼 거리를 받는다. 해상도가 다른 GT 에서
                재현율이 고체 내부 셀 수에 끌려가지 않는다 (관측 영역을 넓히면 내부 셀이 분모에 들어온다).
    adnn0[_median|_p95]          : 추정 점유 셀 중심 → filled 거리장 [m] (map_quality ADNN₀ 와 같은 값)
    adnn0_surface[_median|_p95]  : 추정 점유 셀 중심 → surface 거리장 [m]
    precision_tau                : adnn0 거리 ≤ τ 비율 (map_quality 정밀도와 같은 규칙)
    precision_surface_tau        : adnn0_surface 거리 ≤ τ 비율
    recall_tau                   : 관측된 surface 셀(추정 **자유** 셀에서 observed_margin 안) 중 최근접
                                   추정 점유 셀 중심이 τ 안인 비율
    recall_filled_tau            : filled 셀(추정 알려진 셀에서 observed_margin 안)로 (map_quality 규칙)
    f1_tau                       : precision_surface_tau 와 recall_tau 의 조화평균
    gt_to_est_mean[_filled]      : 관측 영역 안 surface(filled) 셀 → 최근접 추정 점유 셀 중심 [m]
                                   (추정 점을 GT 격자에 칠해 거리변환)
    chamfer0                     : ½(adnn0 + gt_to_est_mean_filled)  (map_quality 규약, 정렬 없음)
    chamfer0_surface             : ½(adnn0_surface + gt_to_est_mean)
    """
    from scipy import ndimage
    tau = est.resolution if tau is None else float(tau)
    est_pts = est.centers(est.occupied)
    if len(est_pts) == 0:
        raise ValueError('추정 지도에 점유 셀이 없다')
    surf = Grid(surface(gt.occupied), gt.known, gt.resolution, gt.origin_x, gt.origin_y)
    d_fill = sample_field(distance_field(gt), gt, est_pts)
    d_surf = sample_field(distance_field(surf), gt, est_pts)
    obs = observed_mask(est, gt, observed_margin)
    obs_free = observed_mask(est, gt, observed_margin, free_only=True)
    est_dist = ndimage.distance_transform_edt(~paint(est_pts, gt)) * gt.resolution
    g_fill = est_dist[gt.occupied & obs]
    g_surf = est_dist[surf.occupied & obs_free]
    eps = 1e-9

    def frac(d: np.ndarray) -> float:
        return float(np.mean(d <= tau + eps)) if d.size else math.nan

    def mean(d: np.ndarray) -> float:
        return float(d.mean()) if d.size else math.nan
    prec_s, rec = frac(d_surf), frac(g_surf)
    f1 = 2 * prec_s * rec / (prec_s + rec) if (prec_s + rec) > 0 else 0.0
    gs = mean(g_surf) if g_surf.size else 0.0
    gf = mean(g_fill) if g_fill.size else 0.0
    return {'adnn0': float(d_fill.mean()), 'adnn0_median': float(np.median(d_fill)),
            'adnn0_p95': float(np.percentile(d_fill, 95)),
            'adnn0_surface': float(d_surf.mean()),
            'adnn0_surface_median': float(np.median(d_surf)),
            'adnn0_surface_p95': float(np.percentile(d_surf, 95)),
            'precision_tau': frac(d_fill), 'precision_surface_tau': prec_s,
            'recall_tau': rec, 'recall_filled_tau': frac(g_fill), 'f1_tau': f1,
            'gt_to_est_mean': mean(g_surf), 'gt_to_est_mean_filled': mean(g_fill),
            'chamfer0': 0.5 * (float(d_fill.mean()) + gf),
            'chamfer0_surface': 0.5 * (float(d_surf.mean()) + gs),
            'tau': tau, 'n_est': int(len(est_pts)), 'n_gt_observed': int(g_surf.size),
            'n_gt_observed_filled': int(g_fill.size)}


def common_spec(resolution: float,
                bounds: Tuple[float, float, float, float]) -> Grid:
    """빈 공통 격자 (map_quality·GT 와 같은 크기 규칙: ceil(폭 / 해상도))."""
    xmin, xmax, ymin, ymax = bounds
    shape = (int(math.ceil((ymax - ymin) / resolution)),
             int(math.ceil((xmax - xmin) / resolution)))
    z = np.zeros(shape, dtype=bool)
    return Grid(z, z.copy(), resolution, xmin, ymin)


def pool(src: Grid, spec: Grid) -> Grid:
    """풀링: src 셀 중심을 spec 셀로 옮긴다 (하나라도 점유면 점유, 하나라도 알려지면 알려짐)."""
    return Grid(paint(src.centers(src.occupied), spec), paint(src.centers(src.known), spec),
                spec.resolution, spec.origin_x, spec.origin_y)


def iou_common(est: Grid, gt: Grid, spec: Grid, tolerance_cells: int = 1,
               observed_margin: float = 0.10) -> Dict[str, float]:
    """
    공통 격자 spec 위 IoU_τ / 정밀도 / 재현율 (τ = tolerance_cells 셀, map_quality.evaluate 와 같은 계수 규칙).

    TP = GT 점유(공통 격자)까지 τ 안인 추정 점유 셀, FP = 나머지 추정 점유 셀,
    FN = 관측 영역 안 GT 점유 셀 중 추정 점유까지 τ 밖인 것. IoU = TP / (TP + FP + FN).
    """
    from scipy import ndimage
    e, g = pool(est, spec), pool(gt, spec)
    tol = tolerance_cells * spec.resolution + 1e-9
    d_to_gt = (ndimage.distance_transform_edt(~g.occupied) * spec.resolution
               if g.occupied.any() else np.full(spec.shape, np.inf))
    d_to_est = (ndimage.distance_transform_edt(~e.occupied) * spec.resolution
                if e.occupied.any() else np.full(spec.shape, np.inf))
    obs = observed_mask(est, spec, observed_margin)
    tp = int(np.count_nonzero(e.occupied & (d_to_gt <= tol)))
    fp = int(np.count_nonzero(e.occupied)) - tp
    gt_obs = g.occupied & obs
    fn = int(np.count_nonzero(gt_obs & (d_to_est > tol)))
    return {'iou_tau': tp / max(tp + fp + fn, 1), 'precision_tau': tp / max(tp + fp, 1),
            'recall_tau': 1.0 - fn / max(int(gt_obs.sum()), 1), 'tp': tp, 'fp': fp, 'fn': fn,
            'resolution': spec.resolution}
