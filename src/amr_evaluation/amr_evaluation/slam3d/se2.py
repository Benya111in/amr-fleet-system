"""
SE(2) 궤적 오차 — 정렬 없는 ATE 가 주 결과 (우리 계약: map 프레임 = 월드 프레임).

정의 (docs/research/slam3d/evaluation.md §4)
  연관   : 추정 시각마다 GT 를 선형 보간한다 (x, y 선형, yaw 는 풀어서 선형). 추정 시각 양옆 GT 표본 간격이
           max_gap 을 넘거나 GT 범위 밖이면 그 추정 표본을 버린다 (외삽 없음).
  ATE    : e_i = ‖p̂_i − p_i‖ (병진), Δψ_i = wrap(ψ̂_i − ψ_i) (방위). 정렬 none(주) / origin(첫 자세를 맞춤,
           evo --align_origin) / se2(2D Umeyama, 축척 없음 — 2D 추정기의 비관측 자유도와 같은 정렬 클래스).
  RPE    : 거리 Δ 쌍 (i, j) — j 는 GT 경로 길이 s_j − s_i ≥ Δ 인 첫 표본. E = (Q_i⁻¹Q_j)⁻¹(P_i⁻¹P_j),
           병진 ‖t(E)‖, 회전 |∠E|. all_pairs=True 면 모든 i 에서 시작 (KITTI 식), False 면 겹치지 않는 연속 구간.
  RTE    : 시간 창 Δt 쌍 (t_j − t_i ≥ Δt 인 첫 j), 오차 정의는 RPE 와 같다.
3D 추정기의 z·roll·pitch 는 이 모듈의 주 지표에 넣지 않는다 (진단용 z_rp_errors 로 따로).
"""

from dataclasses import dataclass
import math
from typing import Dict, Optional, Sequence, Tuple

import numpy as np


@dataclass
class Traj2D:
    """시각 (N,) [s], 위치 (N, 2) [m], 방위 (N,) [rad]. 시각은 증가해야 한다."""

    t: np.ndarray
    xy: np.ndarray
    yaw: np.ndarray

    def __post_init__(self) -> None:
        """모양·단조성 검사."""
        self.t = np.asarray(self.t, dtype=float).reshape(-1)
        self.xy = np.asarray(self.xy, dtype=float).reshape(-1, 2)
        self.yaw = np.asarray(self.yaw, dtype=float).reshape(-1)
        if not (len(self.t) == len(self.xy) == len(self.yaw)):
            raise ValueError('t, xy, yaw 길이가 다르다')
        if len(self.t) > 1 and np.any(np.diff(self.t) <= 0):
            raise ValueError('시각이 엄격히 증가하지 않는다')

    def __len__(self) -> int:
        """표본 수."""
        return len(self.t)


def wrap(a: np.ndarray) -> np.ndarray:
    """각을 [-π, π) 로."""
    return (np.asarray(a, dtype=float) + math.pi) % (2 * math.pi) - math.pi


def quat_to_rpy(q: np.ndarray) -> np.ndarray:
    """쿼터니언 (N, 4) [x, y, z, w] → (N, 3) roll, pitch, yaw (ZYX)."""
    q = np.asarray(q, dtype=float).reshape(-1, 4)
    x, y, z, w = q.T
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], axis=1)


def read_tum(path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """TUM 형식 (t x y z qx qy qz qw, '#' 주석) → (t, xyz (N, 3), quat (N, 4))."""
    rows = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            rows.append([float(v) for v in line.replace(',', ' ').split()[:8]])
    a = np.array(rows, dtype=float).reshape(-1, 8)
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def tum_to_se2(path: str) -> Traj2D:
    """TUM 파일 → Traj2D (yaw 는 쿼터니언의 ZYX yaw)."""
    t, xyz, q = read_tum(path)
    return Traj2D(t, xyz[:, :2], quat_to_rpy(q)[:, 2])


def read_csv_se2(path: str) -> Traj2D:
    """머리줄 있는 CSV (t|stamp|time, x, y, yaw|theta) → Traj2D."""
    import csv
    with open(path, encoding='utf-8', newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return Traj2D(np.zeros(0), np.zeros((0, 2)), np.zeros(0))

    def col(*names: str) -> np.ndarray:
        for n in names:
            if n in rows[0]:
                return np.array([float(r[n]) for r in rows])
        raise KeyError(f'{path}: 열 {names} 이 없다')
    return Traj2D(col('t', 'stamp', 'time'), np.stack([col('x'), col('y')], 1),
                  col('yaw', 'theta'))


def associate(gt: Traj2D, est: Traj2D, max_gap: float = 0.1) -> Tuple[Traj2D, Traj2D]:
    """
    추정 시각에 GT 를 선형 보간한 짝 (gt_i, est_i). 반환 두 궤적은 같은 시각열을 가진다.

    GT 범위 밖 추정 표본과, 둘러싼 두 GT 표본 간격이 max_gap [s] 를 넘는 추정 표본은 버린다.
    """
    if len(gt) < 2 or len(est) == 0:
        empty = Traj2D(np.zeros(0), np.zeros((0, 2)), np.zeros(0))
        return empty, empty
    t = est.t
    k = np.searchsorted(gt.t, t, side='right')
    inside = (k >= 1) & (k <= len(gt) - 1)
    exact_end = t == gt.t[-1]
    k = np.where(exact_end, len(gt) - 1, k)
    inside |= exact_end
    k = np.clip(k, 1, len(gt) - 1)
    gap = gt.t[k] - gt.t[k - 1]
    ok = inside & (gap <= max_gap + 1e-12)
    k, t = k[ok], t[ok]
    w = (t - gt.t[k - 1]) / (gt.t[k] - gt.t[k - 1])
    xy = gt.xy[k - 1] + w[:, None] * (gt.xy[k] - gt.xy[k - 1])
    dyaw = wrap(gt.yaw[k] - gt.yaw[k - 1])
    yaw = wrap(gt.yaw[k - 1] + w * dyaw)
    return Traj2D(t, xy, yaw), Traj2D(t, est.xy[ok], est.yaw[ok])


def stats(e: np.ndarray) -> Dict[str, float]:
    """오차 표본 → rmse, mean, median, std, p95, max, n (비면 NaN, n = 0)."""
    e = np.asarray(e, dtype=float).reshape(-1)
    if e.size == 0:
        return {k: math.nan for k in ('rmse', 'mean', 'median', 'std', 'p95', 'max')} | {'n': 0}
    return {'rmse': float(np.sqrt(np.mean(e ** 2))), 'mean': float(np.mean(e)),
            'median': float(np.median(e)), 'std': float(np.std(e)),
            'p95': float(np.percentile(e, 95)), 'max': float(np.max(e)), 'n': int(e.size)}


def umeyama_se2(src: np.ndarray, dst: np.ndarray) -> Tuple[float, float, float]:
    """정렬 변환 (tx, ty, θ): dst ≈ R(θ)·src + t 의 최소제곱 해 (축척 없음, Umeyama 1991)."""
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    if len(src) < 2:
        raise ValueError('SE(2) 정렬에는 표본이 2 개 이상 필요')
    mu_s, mu_d = src.mean(0), dst.mean(0)
    cov = (dst - mu_d).T @ (src - mu_s) / len(src)
    u, d, vt = np.linalg.svd(cov)
    if np.linalg.matrix_rank(cov) < 1:
        raise ValueError('SE(2) 정렬: 공분산 랭크 부족 (점이 한 곳에 모였다)')
    s = np.diag([1.0, np.sign(np.linalg.det(u) * np.linalg.det(vt))])
    r = u @ s @ vt
    t = mu_d - r @ mu_s
    return float(t[0]), float(t[1]), float(math.atan2(r[1, 0], r[0, 0]))


def apply_se2(traj: Traj2D, tx: float, ty: float, th: float) -> Traj2D:
    """궤적 전체에 월드 쪽 SE(2) 를 곱한다: p' = R p + t, ψ' = ψ + θ."""
    c, s = math.cos(th), math.sin(th)
    xy = traj.xy @ np.array([[c, s], [-s, c]]) + np.array([tx, ty])
    return Traj2D(traj.t, xy, wrap(traj.yaw + th))


def alignment(gt: Traj2D, est: Traj2D, kind: str = 'none') -> Tuple[float, float, float]:
    """정렬 변환 (tx, ty, θ): none = 항등, origin = 첫 자세를 맞춤, se2 = Umeyama 2D (위치만)."""
    if kind == 'none':
        return 0.0, 0.0, 0.0
    if kind == 'origin':
        th = float(wrap(gt.yaw[0] - est.yaw[0]))
        c, s = math.cos(th), math.sin(th)
        p = est.xy[0]
        return (float(gt.xy[0, 0] - (c * p[0] - s * p[1])),
                float(gt.xy[0, 1] - (s * p[0] + c * p[1])), th)
    if kind == 'se2':
        return umeyama_se2(est.xy, gt.xy)
    raise ValueError(f'정렬 종류 {kind!r} (none | origin | se2)')


def ate(gt: Traj2D, est: Traj2D, align: str = 'none', max_gap: float = 0.1) -> Dict[str, object]:
    """
    ATE: 연관 → (정렬) → 병진·방위 오차 통계.

    반환 {'trans': stats [m], 'yaw_deg': stats [deg], 'align': (tx, ty, θ), 'n_est': 원래 추정 표본 수,
    'n_used': 연관된 표본 수}.
    """
    g, e = associate(gt, est, max_gap)
    if len(g) == 0:
        return {'trans': stats([]), 'yaw_deg': stats([]), 'align': (0.0, 0.0, 0.0),
                'n_est': len(est), 'n_used': 0}
    tf = alignment(g, e, align)
    e = apply_se2(e, *tf)
    dt = np.linalg.norm(e.xy - g.xy, axis=1)
    dyaw = np.degrees(np.abs(wrap(e.yaw - g.yaw)))
    return {'trans': stats(dt), 'yaw_deg': stats(dyaw), 'align': tf, 'n_est': len(est),
            'n_used': len(g), 'errors': dt}


def _relative(xy: np.ndarray, yaw: np.ndarray, i: np.ndarray,
              j: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """P_i⁻¹ P_j 의 (병진 (K, 2), 회전 (K,))."""
    d = xy[j] - xy[i]
    c, s = np.cos(yaw[i]), np.sin(yaw[i])
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], 1), \
        wrap(yaw[j] - yaw[i])


def _pair_errors(g: Traj2D, e: Traj2D, i: np.ndarray,
                 j: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """쌍 (i, j) 의 상대 자세 오차: E = (Q_i⁻¹Q_j)⁻¹ (P_i⁻¹P_j) → (‖t(E)‖, |∠E| [deg])."""
    tq, rq = _relative(g.xy, g.yaw, i, j)
    tp, rp = _relative(e.xy, e.yaw, i, j)
    d = tp - tq
    c, s = np.cos(rq), np.sin(rq)
    te = np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], 1)
    return np.linalg.norm(te, axis=1), np.degrees(np.abs(wrap(rp - rq)))


def _pairs(key: np.ndarray, delta: float, all_pairs: bool) -> Tuple[np.ndarray, np.ndarray]:
    """key(단조 증가: 경로 길이 또는 시각)에서 key_j − key_i ≥ delta 인 첫 j 와 짝지은 (i, j)."""
    j = np.searchsorted(key, key + delta - 1e-12, side='left')
    i = np.arange(len(key))
    ok = j < len(key)
    i, j = i[ok], j[ok]
    if all_pairs or len(i) == 0:
        return i, j
    out_i, out_j = [], []
    cur = 0
    lookup = dict(zip(i.tolist(), j.tolist()))
    while cur in lookup:
        out_i.append(cur)
        out_j.append(lookup[cur])
        cur = lookup[cur]
    return np.array(out_i, dtype=int), np.array(out_j, dtype=int)


def path_length(xy: np.ndarray) -> np.ndarray:
    """누적 경로 길이 (N,)."""
    if len(xy) == 0:
        return np.zeros(0)
    return np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])


def rpe(gt: Traj2D, est: Traj2D, deltas: Sequence[float] = (1.0, 2.0, 5.0, 10.0, 20.0),
        all_pairs: bool = True, max_gap: float = 0.1) -> Dict[float, Dict[str, object]]:
    """거리 RPE: Δ 마다 {'trans': stats [m], 'trans_pct': stats [%], 'rot_deg': stats}."""
    g, e = associate(gt, est, max_gap)
    s = path_length(g.xy)
    out: Dict[float, Dict[str, object]] = {}
    for d in deltas:
        i, j = _pairs(s, d, all_pairs) if len(s) else (np.zeros(0, int), np.zeros(0, int))
        te, re = _pair_errors(g, e, i, j) if len(i) else (np.zeros(0), np.zeros(0))
        dist = s[j] - s[i] if len(i) else np.zeros(0)
        pct = 100.0 * te / np.maximum(dist, 1e-9)
        out[float(d)] = {'trans': stats(te), 'trans_pct': stats(pct), 'rot_deg': stats(re)}
    return out


def rte(gt: Traj2D, est: Traj2D, window: float = 10.0, all_pairs: bool = True,
        max_gap: float = 0.1) -> Dict[str, object]:
    """시간 창 RTE (Potokar & Kaess 2025 의 10 s 창): {'trans': stats [m], 'rot_deg': stats}."""
    g, e = associate(gt, est, max_gap)
    i, j = _pairs(g.t, window, all_pairs) if len(g) else (np.zeros(0, int), np.zeros(0, int))
    te, re = _pair_errors(g, e, i, j) if len(i) else (np.zeros(0), np.zeros(0))
    return {'trans': stats(te), 'rot_deg': stats(re)}


def motion_labels(g: Traj2D, smooth: int = 1) -> np.ndarray:
    """
    GT 속도로 명세 구간 라벨 (정지/직선/회전) — amr_evaluation.segments.classify_motion 과 같은 임계.

    속도는 중앙 차분 (끝은 한쪽 차분). smooth 는 차분 간격 [표본].
    """
    from amr_evaluation.segments import classify_motion
    n = len(g)
    if n < 2:
        return np.array(['정지'] * n)
    k = max(1, int(smooth))
    i0 = np.clip(np.arange(n) - k, 0, n - 1)
    i1 = np.clip(np.arange(n) + k, 0, n - 1)
    dt = g.t[i1] - g.t[i0]
    v = np.linalg.norm(g.xy[i1] - g.xy[i0], axis=1) / dt
    w = wrap(g.yaw[i1] - g.yaw[i0]) / dt
    return np.array([classify_motion(float(a), float(b)) for a, b in zip(v, w)])


def ate_by_motion(gt: Traj2D, est: Traj2D, max_gap: float = 0.1,
                  thresholds: Optional[Dict[str, float]] = None) -> Dict[str, Dict[str, object]]:
    """
    정렬 없는 위치 오차를 명세 구간별로 (정지 3 / 직선 5 / 회전 8 cm 기본 임계와 RMSE·max 판정).

    판정은 RMSE 와 max 둘 다 기록한다 (어느 쪽을 쓸지는 보고서가 정한다 — evaluation.md §4).
    """
    th = thresholds or {'정지': 0.03, '직선': 0.05, '회전': 0.08}
    g, e = associate(gt, est, max_gap)
    if len(g) == 0:
        return {}
    err = np.linalg.norm(e.xy - g.xy, axis=1)
    lab = motion_labels(g)
    out: Dict[str, Dict[str, object]] = {}
    for name, limit in th.items():
        st = stats(err[lab == name])
        out[name] = {'trans': st, 'limit': limit,
                     'rmse_ok': bool(st['n'] and st['rmse'] <= limit),
                     'max_ok': bool(st['n'] and st['max'] <= limit)}
    return out


def z_rp_errors(gt_t: np.ndarray, gt_xyz: np.ndarray, gt_q: np.ndarray, est_t: np.ndarray,
                est_xyz: np.ndarray, est_q: np.ndarray,
                max_gap: float = 0.1) -> Dict[str, Dict[str, float]]:
    """3D 추정기 진단: z 오차 [m], roll·pitch 오차 [deg] (GT 최근접 이웃 시각, 간격 > max_gap 은 버림)."""
    gt_t = np.asarray(gt_t, dtype=float)
    k = np.clip(np.searchsorted(gt_t, est_t), 1, len(gt_t) - 1)
    k = np.where(np.abs(gt_t[k - 1] - est_t) < np.abs(gt_t[k] - est_t), k - 1, k)
    ok = np.abs(gt_t[k] - est_t) <= max_gap
    g_rpy, e_rpy = quat_to_rpy(gt_q[k[ok]]), quat_to_rpy(np.asarray(est_q)[ok])
    return {'z': stats(np.abs(np.asarray(est_xyz)[ok, 2] - np.asarray(gt_xyz)[k[ok], 2])),
            'roll_deg': stats(np.degrees(np.abs(wrap(e_rpy[:, 0] - g_rpy[:, 0])))),
            'pitch_deg': stats(np.degrees(np.abs(wrap(e_rpy[:, 1] - g_rpy[:, 1]))))}
