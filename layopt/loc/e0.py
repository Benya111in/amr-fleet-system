"""
E0 지도 지표 평가기 — 체류 지점·경로 표본의 앨리어싱 마진과 이웃 가림 inlier (contract.md §3).

한 자세(pose)와 이웃 k 대에 대해:
  1. 레이캐스트 세계 = 기준 지도 + 이웃 로봇 상자(0.60 × 0.40, 이웃 자세). 기준 지도(점수용)에는 넣지 않는다.
  2. 720 빔, 25 m, σ 0.03 가상 스캔 (raycast.py — alias_margin.py 와 비트 같음).
  3. 운용 amr_localization.global_seed.search (coarse 0.5 m × 5°, 90 빔, top-50, NMS 1 m/20°,
     fine 0.1 m × 1°, 180 빔, 인라이어 순위) → 가설.
  4. margin = global_seed.alias_margin(현재 = 참 자세, 별칭 = 참 자세에서 1 m / 20° 밖 가설, 0.2 m)
     — kidnap_monitor_node.alias_margin 과 같은 정의 (양쪽 ±0.1 m / ±2° 국소 정밀화).
  5. inlier = scan_map_match.match_ratio(참 자세, 180 빔, 0.2 m) — 운용 LOST 판정(0.50)과 같은 정의.
     운용 정의는 "지도에 없는 물체에 막혀 일찍 끝난 빔"을 분자·분모에서 뺀다 (explain_unmapped). 그래서
     참 자세에서는 이웃 가림이 inlier 를 거의 낮추지 못한다 — 보조로 inlier_raw (빼지 않음) 와
     occluded_frac (이웃 상자에 막힌 빔 비율) 를 함께 낸다.

    # 묶음 하나
    python3 -m layopt.loc.e0 bundle --bundle <bundles>/<spec_id> --out <run_dir>
    # 여러 묶음 (재개 가능, 묶음마다 파일)
    python3 -m layopt.loc.e0 batch --bundles-root <bundles> --out <run_dir> --workers 8
    # 검증: SLAM 지도 대기 슬롯 k=0 (설문 spawn_alias.py 재현)
    python3 -m layopt.loc.e0 validate --out <file>
"""

import argparse
import datetime as dt
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from layopt.loc import mapio, raycast

sys.path.insert(0, str(mapio.REPO / 'src' / 'amr_localization'))
from amr_localization import global_seed  # noqa: E402
from amr_localization.scan_map_match import match_ratio  # noqa: E402

Pose = Tuple[float, float, float]
INLIER_DIST = 0.2        # kidnap_monitor.yaml inlier_dist
MAX_BEAMS = 180          # kidnap_monitor.yaml max_beams
K_LEVELS = (0, 2, 4)
ROUTE_N = 40             # 묶음당 경로(자유 공간) 표본 수
ROUTE_CLEARANCE = 0.8    # [m] 표본 자세의 점유까지 최소 거리
SCHEMA = 'layopt-e0/1'


def neighbor_cells(spec, neighbors: Sequence[Sequence[float]]):
    """이웃 자세들 → 덧씌울 점유 셀 목록 (로봇 상자 0.60 × 0.40)."""
    return [mapio.footprint_cells(spec, (float(n[0]), float(n[1]), float(n[2])),
                                  raycast.ROBOT_L, raycast.ROBOT_W) for n in neighbors]


def evaluate_pose(field, occ: np.ndarray, pose: Pose, neighbors: Sequence[Sequence[float]] = (),
                  seed: int = 0) -> Dict:
    """자세 하나 + 이웃 → 지표 dict (마진·inlier·둘째 가설·가설 목록)."""
    t0 = time.perf_counter()
    spec = field.spec
    cells = neighbor_cells(spec, neighbors)
    world = raycast.occupied_mask(occ, cells)
    ranges, amin, inc = raycast.raycast(world, spec, pose, seed=seed)
    # 이웃 상자에 막힌 빔: 잡음 없는 두 스캔(이웃 있음/없음)에서 거리가 다른 빔. 잡음을 넣고 비교하면
    # 이웃이 무한 빔 하나를 유한하게 바꿀 때 뒤 빔들의 잡음 수열이 밀려 전부 "다르다"가 된다
    if cells:
        clean, _, _ = raycast.raycast(world, spec, pose, noise=0.0)
        base, _, _ = raycast.raycast(raycast.occupied_mask(occ), spec, pose, noise=0.0)
        a, b = np.array(clean), np.array(base)
        occluded = float(np.mean((a != b) & ~(np.isinf(a) & np.isinf(b))))
    else:
        occluded = 0.0
    lidar = raycast.LIDAR
    hyps = global_seed.search(field, field.free, ranges, amin, inc, raycast.RANGE_MAX, lidar)
    beams = global_seed.base_beams(ranges, amin, inc, raycast.RANGE_MAX, MAX_BEAMS, lidar)
    alts = global_seed.alternative_poses(hyps, pose, (0.0, 0.0, 0.0))
    margin = global_seed.alias_margin(field, beams, pose, alts, INLIER_DIST)
    rho_true, _ = global_seed.refined_ratio(field, beams, pose, INLIER_DIST)
    second, second_rho, second_dist = None, None, None
    if alts:
        refined = [global_seed.refined_ratio(field, beams, p, INLIER_DIST) for p in alts]
        j = int(np.argmax([r[0] for r in refined]))
        second_rho, sp = refined[j]
        second = [round(sp[0], 3), round(sp[1], 3), round(sp[2], 4)]
        second_dist = round(math.hypot(sp[0] - pose[0], sp[1] - pose[1]), 3)
        assert abs((rho_true - second_rho) - margin) < 1e-12
    inlier, n_valid = match_ratio(field, pose, lidar, ranges, amin, inc, raycast.RANGE_MAX,
                                  INLIER_DIST, MAX_BEAMS)
    inlier_raw, _ = match_ratio(field, pose, lidar, ranges, amin, inc, raycast.RANGE_MAX,
                                INLIER_DIST, MAX_BEAMS, explain_unmapped=False)
    truth_rank = next((i + 1 for i, h in enumerate(hyps)
                       if global_seed.pose_close((h.x, h.y, h.yaw), pose, 1.0,
                                                 math.radians(20.0))), None)
    return {
        'margin': None if margin is None else round(float(margin), 4),
        'inlier': round(float(inlier), 4), 'inlier_n': int(n_valid),
        'inlier_raw': round(float(inlier_raw), 4), 'occluded_frac': round(occluded, 4),
        'rho_true': round(float(rho_true), 4),
        'second': second,
        'second_rho': None if second_rho is None else round(float(second_rho), 4),
        'second_dist': second_dist, 'truth_rank': truth_rank,
        'hyps': [[round(h.x, 3), round(h.y, 3), round(h.yaw, 4), round(h.ratio, 4)] for h in hyps],
        't_s': round(time.perf_counter() - t0, 3),
    }


def sample_free_poses(field, n: int, seed: int, clearance: float = ROUTE_CLEARANCE,
                      bounds: Optional[Sequence[float]] = None) -> List[Pose]:
    """
    자유 공간 자세 표본: 알려진 자유 셀 중 점유까지 거리 ≥ clearance, 헤딩 0/90/180/270°.

    셀 중심 좌표, 시드 고정 (같은 지도·시드 → 같은 표본).
    """
    spec = field.spec
    ok = field.free & (field.field >= clearance)
    rows, cols = np.nonzero(ok)
    xy = global_seed.cell_centers(spec, rows, cols)
    if bounds is not None:
        x0, x1, y0, y1 = bounds
        keep = (xy[:, 0] >= x0) & (xy[:, 0] <= x1) & (xy[:, 1] >= y0) & (xy[:, 1] <= y1)
        xy = xy[keep]
    rng = np.random.default_rng(seed)
    idx = rng.choice(len(xy), size=min(n, len(xy)), replace=False)
    yaws = rng.integers(0, 4, size=len(idx))
    return [(round(float(xy[i, 0]), 3), round(float(xy[i, 1]), 3),
             round(float(q) * math.pi / 2.0 - (2.0 * math.pi if q == 3 else 0.0), 6))
            for i, q in zip(idx, yaws)]


# 자유 공간 자세의 이웃 후보 (로봇 좌표계 오프셋, m): 뒤·앞 대기열 1.2 m, 옆 2 m(대기 줄과 같은 간격), 그 두 배
NEIGHBOR_OFFSETS = [(-1.2, 0.0), (1.2, 0.0), (0.0, 2.0), (0.0, -2.0),
                    (-2.4, 0.0), (2.4, 0.0), (0.0, 4.0), (0.0, -4.0)]


def footprint_free(m: mapio.MapData, pose: Pose, pad: float = 0.15) -> bool:
    """로봇 상자(+ pad)가 지도에서 전부 알려진 자유 셀인가."""
    rr, cc = mapio.footprint_cells(m.spec, pose, raycast.ROBOT_L + 2 * pad,
                                   raycast.ROBOT_W + 2 * pad)
    expect = math.floor((raycast.ROBOT_L + 2 * pad) / m.spec.resolution) * math.floor(
        (raycast.ROBOT_W + 2 * pad) / m.spec.resolution) * 0.8
    return len(rr) >= expect and bool(np.all(m.occ[rr, cc] == 0))


def free_space_neighbors(m: mapio.MapData, pose: Pose, k_max: int = 4) -> List[List[float]]:
    """자유 공간 자세의 이웃: NEIGHBOR_OFFSETS 순서로 지도상 자리가 비는 것 k_max 개 (같은 헤딩)."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for ox, oy in NEIGHBOR_OFFSETS:
        p = (x + c * ox - s * oy, y + s * ox + c * oy, yaw)
        if footprint_free(m, p):
            out.append([round(p[0], 3), round(p[1], 3), round(yaw, 6)])
        if len(out) >= k_max:
            break
    return out


def evaluate_bundle(bundle_dir: Path, route_n: int = ROUTE_N, route_seed: int = 0,
                    k_levels: Sequence[int] = K_LEVELS, layout: Optional[Dict] = None,
                    map_yaml: Optional[Path] = None) -> Dict:
    """묶음 하나 → contract §3 JSON (+ 보조 필드)."""
    t0 = time.perf_counter()
    bundle_dir = Path(bundle_dir)
    if layout is None:
        layout = json.loads((bundle_dir / 'layout.json').read_text(encoding='utf-8'))
    m = mapio.load_map(map_yaml or bundle_dir / 'map.yaml')
    field = mapio.distance_field(m)
    points = []
    for dp in layout.get('dwell_points', []):
        pose = (float(dp['x']), float(dp['y']), float(dp['yaw']))
        nbs = dp.get('neighbors', [])
        for k in k_levels:
            used = nbs[:k]
            r = evaluate_pose(field, m.occ, pose, used)
            points.append({'id': dp['id'], 'kind': dp['kind'], 'k': k, 'k_used': len(used),
                           'x': pose[0], 'y': pose[1], 'yaw': pose[2], **r})
    route = []
    if route_n > 0:
        for i, pose in enumerate(sample_free_poses(field, route_n, route_seed,
                                                   bounds=layout.get('bounds'))):
            r = evaluate_pose(field, m.occ, pose, ())
            route.append({'id': f'route:{i:03d}', 'x': pose[0], 'y': pose[1], 'yaw': pose[2],
                          'margin': r['margin'], 'inlier': r['inlier'],
                          'second_dist': r['second_dist'], 'truth_rank': r['truth_rank']})
    margins = np.array([p['margin'] for p in route if p['margin'] is not None])
    return {
        'schema': SCHEMA, 'spec_id': layout.get('spec_id', bundle_dir.name),
        'map_sha256': m.pgm_sha256, 'map_yaml_sha256': m.yaml_sha256, 'map': m.path,
        'points': points,
        'route_samples': {
            'n': int(margins.size),
            'margin_q05': _q(margins, 0.05),
            'margin_q50': _q(margins, 0.5),
            'seed': route_seed, 'clearance': ROUTE_CLEARANCE, 'samples': route},
        'params': {'beams': raycast.N_BEAMS, 'range_max': raycast.RANGE_MAX,
                   'noise': raycast.NOISE, 'scan_seed': 0, 'inlier_dist': INLIER_DIST,
                   'max_beams': MAX_BEAMS, 'robot_box': [raycast.ROBOT_L, raycast.ROBOT_W],
                   'seed_params': vars(global_seed.SeedParams())},
        'eval_s': round(time.perf_counter() - t0, 2),
        'created': dt.datetime.now(dt.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
    }


def _q(a: np.ndarray, q: float) -> Optional[float]:
    """분위수 (빈 배열이면 None)."""
    return None if not a.size else round(float(np.quantile(a, q)), 4)


def write_atomic(path: Path, obj) -> None:
    """임시 파일에 쓰고 이름을 바꾼다 (중간에 끊겨도 반쪽 파일이 남지 않는다)."""
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(obj, ensure_ascii=False, indent=1), encoding='utf-8')
    os.replace(tmp, path)


def _worker(args) -> Dict:
    bundle, out_dir, route_n = args
    os.environ.setdefault('OMP_NUM_THREADS', '1')
    t0 = time.perf_counter()
    try:
        res = evaluate_bundle(Path(bundle), route_n=route_n)
        out = Path(out_dir) / f'{res["spec_id"]}.json'
        if out.exists():          # 다른 실행이 먼저 썼다 — 덮어쓰지 않는다
            return {'bundle': bundle, 'status': 'exists', 's': time.perf_counter() - t0}
        write_atomic(out, res)
        return {'bundle': bundle, 'spec_id': res['spec_id'], 'status': 'ok',
                's': round(time.perf_counter() - t0, 2), 'n_points': len(res['points'])}
    except Exception as exc:     # noqa: BLE001 — 묶음 하나의 실패가 배치를 멈추지 않게 기록만
        return {'bundle': bundle, 'status': 'error', 'error': repr(exc),
                's': round(time.perf_counter() - t0, 2)}


def list_bundles(root: Path, require_sealed: bool = True) -> List[Path]:
    """layout.json·map.yaml 이 있는 묶음 (기본: SEALED 된 것만)."""
    out = []
    for d in sorted(Path(root).iterdir()):
        if not d.is_dir():
            continue
        if not (d / 'layout.json').exists() or not (d / 'map.yaml').exists():
            continue
        if require_sealed and not (d / 'SEALED').exists():
            continue
        out.append(d)
    return out


def run_batch(bundles: Sequence[Path], out_dir: Path, workers: int, route_n: int) -> Dict:
    """묶음들을 병렬 평가. 이미 결과가 있는 묶음은 건너뛴다 (재개). progress.jsonl 에 묶음마다 한 줄."""
    out_dir.mkdir(parents=True, exist_ok=True)
    done = {p.stem for p in out_dir.glob('*.json') if not p.name.startswith('_')}
    todo = [b for b in bundles if b.name not in done]
    t0 = time.time()
    stats = {'todo': len(todo), 'skipped': len(bundles) - len(todo), 'ok': 0, 'error': 0}
    prog = out_dir / 'progress.jsonl'
    with mp.get_context('fork').Pool(max(1, workers)) as pool, prog.open('a') as f:
        for r in pool.imap_unordered(_worker, [(str(b), str(out_dir), route_n) for b in todo]):
            r['t'] = dt.datetime.now(dt.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
            f.write(json.dumps(r, ensure_ascii=False) + '\n')
            f.flush()
            stats['ok' if r['status'] == 'ok' else 'error'] += 1
    stats['wall_s'] = round(time.time() - t0, 1)
    stats['s_per_bundle_wall'] = (round(stats['wall_s'] / max(stats['ok'], 1), 2))
    return stats


# ------------------------------------------------------------------- 검증: 설문 대기 슬롯 재현
SLAM_SLOTS = [(x, -16.0, math.pi / 2) for x in (18.0, 20.0, 22.0, 24.0, 26.0)]


def slam_waiting_layout(k_max: int = 4) -> Dict:
    """SLAM 지도(maps/warehouse.yaml)용 대기 슬롯 체류 지점 (fleet_spawn.yaml 5 슬롯, 이웃 = 다른 슬롯 가까운 순)."""
    pts = []
    for i, (x, y, yaw) in enumerate(SLAM_SLOTS):
        others = sorted((p for p in SLAM_SLOTS if p[0] != x), key=lambda p: (abs(p[0] - x), p[0]))
        pts.append({'id': f'wait:amr_{i + 1:02d}', 'kind': 'waiting', 'x': x, 'y': y, 'yaw': yaw,
                    'neighbors': [list(p) for p in others[:k_max]]})
    return {'spec_id': 'slam-warehouse', 'bounds': [-30.0, 30.0, -20.0, 20.0], 'dwell_points': pts}


def main(argv=None) -> int:
    """명령줄 진입점 (bundle | batch | validate)."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    sub = ap.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('bundle', help='묶음 하나')
    b.add_argument('--bundle', required=True)
    b.add_argument('--out', required=True, help='출력 디렉터리 (<spec_id>.json)')
    b.add_argument('--route-n', type=int, default=ROUTE_N)
    bt = sub.add_parser('batch', help='여러 묶음 (병렬, 재개 가능)')
    bt.add_argument('--bundles-root', default='/home/CAPYI_AI/runs/layopt/bundles')
    bt.add_argument('--spec-ids', default='', help='쉼표 구분 (빈 값 = 전부)')
    bt.add_argument('--out', required=True)
    bt.add_argument('--resume', action='store_true', help='있는 출력 디렉터리에 이어 쓴다')
    bt.add_argument('--workers', type=int, default=8)
    bt.add_argument('--route-n', type=int, default=ROUTE_N)
    bt.add_argument('--include-unsealed', action='store_true')
    v = sub.add_parser('validate', help='SLAM 지도 대기 슬롯 (k = 0, 2, 4)')
    v.add_argument('--out', required=True)
    v.add_argument('--route-n', type=int, default=0)
    a = ap.parse_args(argv)

    if a.cmd == 'bundle':
        out_dir = Path(a.out)
        out_dir.mkdir(parents=True, exist_ok=True)
        res = evaluate_bundle(Path(a.bundle), route_n=a.route_n)
        out = out_dir / f'{res["spec_id"]}.json'
        if out.exists():
            print(f'이미 있다 (덮어쓰지 않음): {out}', file=sys.stderr)
            return 3
        write_atomic(out, res)
        print(json.dumps({'out': str(out), 'eval_s': res['eval_s'],
                          'n_points': len(res['points'])}, ensure_ascii=False))
        return 0
    if a.cmd == 'batch':
        out_dir = Path(a.out)
        if out_dir.exists() and not a.resume:
            print(f'출력 디렉터리가 이미 있다 (--resume 으로 이어 쓰기): {out_dir}', file=sys.stderr)
            return 3
        bundles = list_bundles(Path(a.bundles_root), require_sealed=not a.include_unsealed)
        if a.spec_ids:
            want = set(a.spec_ids.split(','))
            bundles = [x for x in bundles if x.name in want]
        stats = run_batch(bundles, out_dir, a.workers, a.route_n)
        write_atomic(out_dir / f'_batch_{int(time.time())}.json',
                     {**stats, 'workers': a.workers, 'bundles_root': a.bundles_root})
        print(json.dumps(stats, ensure_ascii=False))
        return 0 if stats['error'] == 0 else 1
    if a.cmd == 'validate':
        res = evaluate_bundle(mapio.REPO / 'maps', route_n=a.route_n,
                              layout=slam_waiting_layout(),
                              map_yaml=mapio.REPO / 'maps' / 'warehouse.yaml')
        out = Path(a.out)
        if out.exists():
            print(f'이미 있다: {out}', file=sys.stderr)
            return 3
        write_atomic(out, res)
        for p in res['points']:
            print(f"{p['id']} k={p['k']} margin={p['margin']} inlier={p['inlier']} "
                  f"raw={p['inlier_raw']} occ={p['occluded_frac']} second={p['second']} "
                  f"d={p['second_dist']} rank={p['truth_rank']} t={p['t_s']}")
        return 0
    return 2


if __name__ == '__main__':
    sys.exit(main())
