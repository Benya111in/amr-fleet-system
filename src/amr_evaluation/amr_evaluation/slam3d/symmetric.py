r"""
대칭 비교표 (rclpy 비의존) — 2D·3D 행, SE(2)·SE(3) 추정기를 **같은** SE(2) 지표와 **같은** 시간창으로 잰다.

    python3 -m amr_evaluation.slam3d.symmetric --manifest runs.json --out sym.json [--md sym.md]

근거: 리서치 노트 「실측 기반 후속 기술 조사」 planar_lio_z_drift.md Q4·R1 — 순위는 xy·yaw 지표로만 매기고,
z·roll·pitch·지도 바닥 평탄도는 3D 센서 행에만 있는 **별도 열**로 둔다 (순위에 넣지 않는다).

주 열 (정렬 없음이 주, `se2.ate`·`rpe`·`rte` 그대로 — evaluation.md §4.1):
  ate_xy_rmse_cm·ate_xy_max_cm (정렬 없음), ate_xy_se2_rmse_cm (SE(2) Umeyama 정렬),
  ate_yaw_rmse_deg·ate_yaw_max_deg (정렬 없음), rpe1_pct·rpe10_pct (거리 1·10 m RPE 병진 %, 모든 시작점,
  RMSE), rte10_rmse_cm (시간 10 s 창, RMSE).
3D 열 (dim = 3d 인 행만, 2D 행은 None = 해당 없음):
  z_abs_max_cm·z_rmse_cm, roll_rmse_deg·pitch_rmse_deg·rp_abs_max_deg (GT 최근접 시각 —
  `se2.z_rp_errors`), 지도가 있으면 below_floor_pct (z < −0.05 m 점 비율), floor_z_p01_cm
  (z < 0.15 m 점들의 z 1 백분위), n_points.

공통 시간창: manifest 의 모든 실행 궤적 구간의 교집합 [max 첫 스탬프, min 끝 스탬프]. 행마다 처리 끝이 다르면
(LIO-SAM 끝 2 장 캐시 등) 그만큼 모든 행을 같이 자른다. 창 밖 추정 표본은 버린다.

manifest (JSON):
  {"gt": "gt.tum", "max_gap": 0.1, "window": "common" | [lo, hi],
   "runs": [{"row": "rtabmap3d_noimu", "replay": 0, "tum": ".../traj_online.tum", "dim": "3d",
             "ply": ".../cloud_world.ply" (선택), "oracle": false, "label": "자유 설명" (선택)}, ...]}
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from typing import Dict, List, Optional, Sequence, Tuple

from amr_evaluation.slam3d import se2
import numpy as np

MAIN_COLS = ('ate_xy_rmse_cm', 'ate_xy_max_cm', 'ate_xy_se2_rmse_cm', 'ate_yaw_rmse_deg',
             'ate_yaw_max_deg', 'rpe1_pct', 'rpe10_pct', 'rte10_rmse_cm')
DIM3_COLS = ('z_abs_max_cm', 'z_rmse_cm', 'roll_rmse_deg', 'pitch_rmse_deg', 'rp_abs_max_deg')
MAP_COLS = ('below_floor_pct', 'floor_z_p01_cm', 'n_points')
#: 바닥 아래로 보는 높이 [m] (rtabmap.md §4.2 의 "바닥 아래(z < −0.05 m) 점" 과 같은 문턱)
BELOW_FLOOR_Z = -0.05
#: 바닥 대역 위 끝 [m] — 이 아래 점의 z 분포 꼬리로 지도가 바닥 밑으로 처졌는지 본다
FLOOR_BAND_Z = 0.15


def common_window(stamps: Sequence[Sequence[float]]) -> Tuple[float, float]:
    """여러 궤적 스탬프 열의 공통 구간 [max 첫, min 끝]. 겹치지 않으면 ValueError."""
    if not stamps:
        raise ValueError('궤적이 없다')
    for t in stamps:
        if len(t) == 0:
            raise ValueError('빈 궤적이 있다')
    lo = max(float(np.min(t)) for t in stamps)
    hi = min(float(np.max(t)) for t in stamps)
    if not lo < hi:
        raise ValueError(f'공통 구간이 없다: [{lo}, {hi}]')
    return lo, hi


def in_window(t: np.ndarray, window: Tuple[float, float]) -> np.ndarray:
    """창 [lo, hi] 안 표본 마스크 (양 끝 포함)."""
    t = np.asarray(t, dtype=float)
    return (t >= window[0]) & (t <= window[1])


def _sub(tr: se2.Traj2D, m: np.ndarray) -> se2.Traj2D:
    return se2.Traj2D(tr.t[m], tr.xy[m], tr.yaw[m])


def se2_columns(gt: se2.Traj2D, est: se2.Traj2D, window: Tuple[float, float],
                max_gap: float = 0.1) -> Dict[str, float]:
    """주 열 — 창 안 추정 표본만으로 정렬 없음·SE(2) 정렬 ATE, 거리 RPE, 시간 RTE."""
    e = _sub(est, in_window(est.t, window))
    a = se2.ate(gt, e, 'none', max_gap)
    s = se2.ate(gt, e, 'se2', max_gap) if a['n_used'] >= 2 else None
    rp = se2.rpe(gt, e, (1.0, 10.0), True, max_gap)
    rt = se2.rte(gt, e, 10.0, True, max_gap)
    return {'n': int(a['n_used']), 'n_window': int(len(e)),
            'ate_xy_rmse_cm': 100.0 * a['trans']['rmse'],
            'ate_xy_max_cm': 100.0 * a['trans']['max'],
            'ate_xy_se2_rmse_cm': 100.0 * s['trans']['rmse'] if s else float('nan'),
            'ate_yaw_rmse_deg': a['yaw_deg']['rmse'], 'ate_yaw_max_deg': a['yaw_deg']['max'],
            'rpe1_pct': rp[1.0]['trans_pct']['rmse'], 'rpe10_pct': rp[10.0]['trans_pct']['rmse'],
            'rte10_rmse_cm': 100.0 * rt['trans']['rmse']}


def dim3_columns(gt: Tuple[np.ndarray, np.ndarray, np.ndarray],
                 est: Tuple[np.ndarray, np.ndarray, np.ndarray], window: Tuple[float, float],
                 max_gap: float = 0.1) -> Dict[str, float]:
    """3D 열 — 창 안 추정 표본의 z |오차|, roll·pitch 오차 (GT 최근접 시각, `se2.z_rp_errors`)."""
    t, xyz, q = (np.asarray(a, dtype=float) for a in est)
    m = in_window(t, window)
    r = se2.z_rp_errors(gt[0], gt[1], gt[2], t[m], xyz[m], q[m], max_gap)
    return {'z_abs_max_cm': 100.0 * r['z']['max'], 'z_rmse_cm': 100.0 * r['z']['rmse'],
            'roll_rmse_deg': r['roll_deg']['rmse'], 'pitch_rmse_deg': r['pitch_deg']['rmse'],
            'rp_abs_max_deg': max(r['roll_deg']['max'], r['pitch_deg']['max'])}


def floor_columns(xyz: np.ndarray) -> Dict[str, float]:
    """지도 바닥 평탄도 — 바닥 아래 점 비율 [%], 바닥 대역 점 z 1 백분위 [cm], 점 수."""
    xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
    n = len(xyz)
    if n == 0:
        return {'below_floor_pct': float('nan'), 'floor_z_p01_cm': float('nan'), 'n_points': 0}
    z = xyz[:, 2]
    band = z[z < FLOOR_BAND_Z]
    return {'below_floor_pct': 100.0 * float(np.count_nonzero(z < BELOW_FLOOR_Z)) / n,
            'floor_z_p01_cm': 100.0 * float(np.percentile(band, 1)) if len(band) else float('nan'),
            'n_points': int(n)}


def spread(values: Sequence[Optional[float]]) -> Dict[str, float]:
    """재생 간 산포 — 유한값만 (None·NaN 은 n_missing). sd 는 표본 표준편차 (n − 1)."""
    v = np.array([x for x in values if x is not None and math.isfinite(x)], dtype=float)
    out = {'n': int(len(v)), 'n_missing': int(len(values) - len(v))}
    if len(v) == 0:
        out.update({k: float('nan') for k in ('mean', 'sd', 'min', 'max', 'range')})
        return out
    sd = float(v.std(ddof=1)) if len(v) > 1 else float('nan')
    out.update({'mean': float(v.mean()), 'sd': sd,
                'min': float(v.min()), 'max': float(v.max()), 'range': float(v.max() - v.min())})
    return out


def noise_threshold(sd_a: float, sd_b: float) -> float:
    """사전 등록 문턱 — 두 행 재생 간 표준편차 중 큰 쪽의 2 배."""
    return 2.0 * max(sd_a, sd_b)


def differs(a: Dict[str, float], b: Dict[str, float]) -> Optional[bool]:
    """
    사전 등록 판정: 재생 범위가 겹치지 않고 평균 차가 문턱보다 크면 True, 아니면 False.

    어느 쪽이든 표본이 2 개 미만이라 sd 가 없으면 None (판정 불가).
    """
    if a['n'] < 2 or b['n'] < 2:
        return None
    apart = a['max'] < b['min'] or b['max'] < a['min']
    return bool(apart and abs(a['mean'] - b['mean']) > noise_threshold(a['sd'], b['sd']))


def _read_ply_xyz(path: str) -> np.ndarray:
    from amr_evaluation.slam3d import cloud
    return cloud.xyz_of(cloud.read_ply(path))


def evaluate(manifest: Dict[str, object]) -> Dict[str, object]:
    """실행 목록(manifest) → 실행별 열과 행별 산포."""
    gt_t, gt_xyz, gt_q = se2.read_tum(str(manifest['gt']))
    gt2 = se2.Traj2D(gt_t, gt_xyz[:, :2], se2.quat_to_rpy(gt_q)[:, 2])
    max_gap = float(manifest.get('max_gap', 0.1))
    runs = list(manifest['runs'])
    trajs = [se2.read_tum(str(r['tum'])) for r in runs]
    win = manifest.get('window', 'common')
    window = common_window([t for t, _, _ in trajs]) if win == 'common' else \
        (float(win[0]), float(win[1]))
    out_runs: List[Dict[str, object]] = []
    for r, (t, xyz, q) in zip(runs, trajs):
        est2 = se2.Traj2D(t, xyz[:, :2], se2.quat_to_rpy(q)[:, 2])
        row: Dict[str, object] = {
            'row': r['row'], 'replay': r.get('replay', 0), 'dim': r.get('dim', '3d'),
            'oracle': bool(r.get('oracle', False)), 'label': r.get('label', ''), 'tum': r['tum'],
            'n_traj': int(len(t))}
        row.update(se2_columns(gt2, est2, window, max_gap))
        if row['dim'] == '3d':
            row.update(dim3_columns((gt_t, gt_xyz, gt_q), (t, xyz, q), window, max_gap))
        else:
            row.update({k: None for k in DIM3_COLS})
        if r.get('ply'):
            row.update(floor_columns(_read_ply_xyz(str(r['ply']))))
            row['ply'] = r['ply']
        else:
            row.update({k: None for k in MAP_COLS})
        out_runs.append(row)
    names: List[str] = []
    for r in out_runs:
        if r['row'] not in names:
            names.append(str(r['row']))
    rows = {}
    for name in names:
        mine = [r for r in out_runs if r['row'] == name]
        rows[name] = {'replays': [r['replay'] for r in mine], 'dim': mine[0]['dim'],
                      'oracle': mine[0]['oracle'],
                      'cols': {c: spread([r[c] for r in mine])
                               for c in MAIN_COLS + DIM3_COLS + MAP_COLS + ('n',)}}
    return {'window': list(window), 'max_gap': max_gap, 'gt': manifest['gt'], 'runs': out_runs,
            'rows': rows}


def _fmt(s: Dict[str, float], nd: int) -> str:
    if s['n'] == 0:
        return '해당 없음'
    if s['n'] == 1:
        return f"{s['mean']:.{nd}f}"
    return f"{s['mean']:.{nd}f} ± {s['sd']:.{nd}f} [{s['min']:.{nd}f}, {s['max']:.{nd}f}]"


def markdown(res: Dict[str, object], cols: Sequence[str] = MAIN_COLS, nd: int = 2) -> str:
    """행별 표 — 평균 ± 표본 sd [min, max] (n = 재생 수). 오라클 행은 이름에 표시한다."""
    head = '| 행 | n | ' + ' | '.join(cols) + ' |'
    lines = [head, '|' + '---|' * (len(cols) + 2)]
    for name, r in res['rows'].items():
        tag = f'{name} (오라클·상한 참고)' if r['oracle'] else name
        n = r['cols']['n']['n']
        lines.append(f'| {tag} | {n} | ' + ' | '.join(_fmt(r['cols'][c], nd) for c in cols) + ' |')
    return '\n'.join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--manifest', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--md', default='')
    ns = ap.parse_args(argv)
    with open(ns.manifest, encoding='utf-8') as f:
        man = json.load(f)
    res = evaluate(man)
    with open(ns.out, 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    if ns.md:
        with open(ns.md, 'w', encoding='utf-8') as f:
            f.write(f"공통 시간창 [{res['window'][0]:.3f}, {res['window'][1]:.3f}] s\n\n")
            f.write(markdown(res, MAIN_COLS) + '\n\n' + markdown(res, DIM3_COLS + MAP_COLS) + '\n')
    print(f"{len(res['runs'])} 실행, {len(res['rows'])} 행, 창 {res['window']}")
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
