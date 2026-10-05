"""
3D 점군 지도 지표 (정렬 없음 — 추정 지도는 이미 월드 프레임에 있어야 한다).

정의 (docs/research/slam3d/evaluation.md §4 — 출처 정의를 그대로 옮기고 해석이 필요한 곳은 표시)
  d_e→g  : 추정 점마다 최근접 GT 점 거리.   d_g→e : GT 점마다 최근접 추정 점 거리.
  precision@τ = mean(d_e→g ≤ τ), recall@τ = mean(d_g→e ≤ τ), F@τ = 2PR/(P+R)
              (Oxford Spires: τ = 5, 10 cm)
  AC@τ   = mean(d_e→g | d_e→g ≤ τ)  — MapEval "임계 안 대응점의 평균 오차"
  COM@τ  = recall@τ                  — MapEval 의 |C_τ|/N_g 를 GT→추정 방향으로 읽었다 [해석]
  chamfer_sum  = mean(d_e→g) + mean(d_g→e)   (MapEval CD)
  chamfer_half = ½ chamfer_sum               (map_quality 규약)
  유령 점: 정적 GT 에서 τ 보다 먼 추정 점 중 동적 장애물이 지나간 영역 안에 있는 것의 비율
  SA / DA / AA (DynamicMap Benchmark): 라벨 붙은 원시 점 중 정적 점이 출력에 남은 비율 / 동적 점이 지워진
           비율 / √(SA·DA). "남았다" = match_dist 안에 출력 점이 있다.
GT 를 관측 가능 영역으로 깎는 것(GT 마스킹)과 추정 점 중 GT 범위 밖을 지우는 것(Oxford Spires 필터)은 서로 다른
조작이다 — 둘 다 이 모듈 밖(호출자)이 하고, 결과 CSV 의 param 에 이름을 남긴다.
"""

from typing import Callable, Dict, Optional, Sequence, Tuple

import numpy as np

PLY_TYPES = {'char': 'i1', 'int8': 'i1', 'uchar': 'u1', 'uint8': 'u1', 'short': 'i2',
             'int16': 'i2', 'ushort': 'u2', 'uint16': 'u2', 'int': 'i4', 'int32': 'i4',
             'uint': 'u4', 'uint32': 'u4', 'float': 'f4', 'float32': 'f4', 'double': 'f8',
             'float64': 'f8'}


def read_ply(path: str, mmap: bool = True) -> np.ndarray:
    """binary_little_endian PLY 의 vertex 원소 → 구조 배열 (스칼라 속성만; 면 원소는 무시)."""
    with open(path, 'rb') as f:
        header = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f'{path}: PLY 머리 끝(end_header)이 없다')
            line = line.decode('ascii').strip()
            header.append(line)
            if line == 'end_header':
                break
        offset = f.tell()
    if header[0] != 'ply' or 'format binary_little_endian 1.0' not in header:
        raise ValueError(f'{path}: binary_little_endian PLY 만 지원')
    fields, count, in_vertex = [], 0, False
    for line in header:
        parts = line.split()
        if parts[:1] == ['element']:
            in_vertex = parts[1] == 'vertex'
            if in_vertex:
                count = int(parts[2])
        elif parts[:1] == ['property'] and in_vertex:
            if parts[1] == 'list':
                raise ValueError(f'{path}: vertex 의 list 속성은 지원하지 않는다')
            fields.append((parts[2], '<' + PLY_TYPES[parts[1]]))
    dtype = np.dtype(fields)
    if mmap:
        return np.memmap(path, dtype=dtype, mode='r', offset=offset, shape=(count,))
    return np.fromfile(path, dtype=dtype, count=count, offset=offset)


def write_ply(path: str, xyz: np.ndarray, label: Optional[np.ndarray] = None) -> None:
    """float32 x y z (+ uchar label) binary_little_endian PLY."""
    xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
    fields = [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]
    if label is not None:
        fields.append(('label', 'u1'))
    arr = np.empty(len(xyz), dtype=fields)
    arr['x'], arr['y'], arr['z'] = xyz.T
    if label is not None:
        arr['label'] = label
    head = ['ply', 'format binary_little_endian 1.0', f'element vertex {len(arr)}',
            'property float x', 'property float y', 'property float z']
    head += ['property uchar label'] if label is not None else []
    with open(path, 'wb') as f:
        f.write(('\n'.join(head + ['end_header']) + '\n').encode('ascii'))
        arr.tofile(f)


def xyz_of(arr: np.ndarray) -> np.ndarray:
    """구조 배열 또는 (N, 3) → float64 (N, 3)."""
    if arr.dtype.names:
        return np.stack([np.asarray(arr['x'], dtype=float), np.asarray(arr['y'], dtype=float),
                         np.asarray(arr['z'], dtype=float)], 1)
    return np.asarray(arr, dtype=float).reshape(-1, 3)


def select(arr: np.ndarray, labels: Optional[Sequence[int]] = None,
           roi: Optional[Tuple[float, float, float, float]] = None,
           z: Optional[Tuple[float, float]] = None) -> np.ndarray:
    """GT 점 고르기: 라벨 집합, xy 범위 (xmin, xmax, ymin, ymax), z 범위. 반환 (N, 3)."""
    keep = np.ones(len(arr), dtype=bool)
    if labels is not None:
        keep &= np.isin(np.asarray(arr['label']), list(labels))
    if roi is not None:
        x, y = np.asarray(arr['x']), np.asarray(arr['y'])
        keep &= (x >= roi[0]) & (x <= roi[1]) & (y >= roi[2]) & (y <= roi[3])
    if z is not None:
        zz = np.asarray(arr['z'])
        keep &= (zz >= z[0]) & (zz <= z[1])
    return xyz_of(arr[np.nonzero(keep)[0]])


def voxel_downsample(xyz: np.ndarray, voxel: float) -> np.ndarray:
    """복셀마다 첫 점 하나 (순서 보존). 밀도가 다른 지도를 같은 밀도로 맞출 때."""
    if voxel <= 0 or len(xyz) == 0:
        return xyz
    key = np.floor(xyz / voxel).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return xyz[np.sort(idx)]


def nn_distance(src: np.ndarray, dst: np.ndarray, upper: float = np.inf) -> np.ndarray:
    """최근접 거리: src 점마다 dst 의 최근접 점까지 (upper 를 넘으면 inf; dst 가 비면 전부 inf)."""
    from scipy.spatial import cKDTree
    if len(dst) == 0:
        return np.full(len(src), np.inf)
    if len(src) == 0:
        return np.zeros(0)
    d, _ = cKDTree(dst).query(src, k=1, distance_upper_bound=upper, workers=-1)
    return d


def cloud_metrics(est: np.ndarray, gt: np.ndarray,
                  taus: Sequence[float] = (0.05, 0.10)) -> Dict[str, float]:
    """정확도·완전성·F-score·Chamfer (모듈 독스트링 정의). est, gt 는 (N, 3) 월드 좌표."""
    if len(est) == 0 or len(gt) == 0:
        raise ValueError('추정 또는 GT 점군이 비었다')
    d_eg = nn_distance(est, gt)
    d_ge = nn_distance(gt, est)
    out: Dict[str, float] = {
        'acc_mean': float(d_eg.mean()), 'acc_median': float(np.median(d_eg)),
        'acc_p95': float(np.percentile(d_eg, 95)), 'comp_mean': float(d_ge.mean()),
        'comp_median': float(np.median(d_ge)), 'comp_p95': float(np.percentile(d_ge, 95)),
        'chamfer_sum': float(d_eg.mean() + d_ge.mean()),
        'chamfer_half': float(0.5 * (d_eg.mean() + d_ge.mean())),
        'n_est': int(len(est)), 'n_gt': int(len(gt))}
    for tau in taus:
        key = f'{tau:g}'
        inl = d_eg <= tau
        p = float(inl.mean())
        r = float((d_ge <= tau).mean())
        out[f'precision@{key}'] = p
        out[f'recall@{key}'] = r
        out[f'fscore@{key}'] = 2 * p * r / (p + r) if p + r > 0 else 0.0
        out[f'ac@{key}'] = float(d_eg[inl].mean()) if inl.any() else float('nan')
        out[f'n_inlier@{key}'] = int(inl.sum())
        out[f'com@{key}'] = r
    return out


def ghost_ratio(est: np.ndarray, static_gt: np.ndarray, tau: float,
                dynamic_region: Optional[Callable[[np.ndarray], np.ndarray]] = None
                ) -> Dict[str, float]:
    """
    정적 GT 로 설명되지 않는 추정 점의 비율.

    unexplained = d_e→g > τ 인 비율. ghost = 그중 dynamic_region(xy (N, 2) → bool) 안에 있는 점의 비율
    (분모는 전체 추정 점). dynamic_region 이 없으면 ghost = unexplained.
    """
    if len(est) == 0:
        return {'unexplained': float('nan'), 'ghost': float('nan'), 'n_est': 0}
    far = nn_distance(est, static_gt, upper=tau * 1.0001) > tau
    ghost = far & dynamic_region(est[:, :2]) if dynamic_region is not None else far
    return {'unexplained': float(far.mean()), 'ghost': float(ghost.mean()),
            'n_est': int(len(est))}


def dynamic_scores(points: np.ndarray, is_dynamic: np.ndarray, output: np.ndarray,
                   match_dist: float) -> Dict[str, float]:
    """
    SA / DA / AA (DynamicMap Benchmark): points = 라벨 붙은 원시 누적 점 (N, 3), output = 방법의 출력 지도.

    SA = 정적 점 중 출력에 남은 비율, DA = 동적 점 중 출력에서 사라진 비율, AA = √(SA·DA).
    """
    is_dynamic = np.asarray(is_dynamic, dtype=bool)
    kept = nn_distance(points, output, upper=match_dist * 1.0001) <= match_dist
    n_s, n_d = int((~is_dynamic).sum()), int(is_dynamic.sum())
    sa = float(kept[~is_dynamic].mean()) if n_s else float('nan')
    da = float((~kept[is_dynamic]).mean()) if n_d else float('nan')
    return {'SA': sa, 'DA': da, 'AA': float(np.sqrt(sa * da)), 'n_static': n_s,
            'n_dynamic': n_d}


def region_from_raster(occupied: np.ndarray, resolution: float, origin_x: float,
                       origin_y: float) -> Callable[[np.ndarray], np.ndarray]:
    """래스터(행 0 = 최하단 y) → xy 점이 점유 셀 안인지 판정하는 함수 (격자 밖은 False)."""
    def inside(xy: np.ndarray) -> np.ndarray:
        c = np.floor((xy[:, 0] - origin_x) / resolution).astype(np.int64)
        r = np.floor((xy[:, 1] - origin_y) / resolution).astype(np.int64)
        ok = (c >= 0) & (c < occupied.shape[1]) & (r >= 0) & (r < occupied.shape[0])
        out = np.zeros(len(xy), dtype=bool)
        out[ok] = occupied[r[ok], c[ok]]
        return out
    return inside
