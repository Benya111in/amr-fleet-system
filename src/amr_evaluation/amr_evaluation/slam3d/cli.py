r"""
보너스 2 평가 CLI — 결과를 results.py 스키마의 CSV 행으로 덧붙인다.

    ros2 run amr_evaluation slam3d_eval traj --gt gt.tum --est est.tum --out results.csv \
        --run-id S3-b01-rtabmap3d-r0 --scenario S3 --bag-id b01 --replay 0 --row rtabmap_3d \
        --stage slam --factor sensor=3d --factor wheel_prior=on --git-head $(git rev-parse HEAD)
    ros2 run amr_evaluation slam3d_eval grid --gt-map gt/slice_z0.200_r0.010.yaml \
        --est-map maps/warehouse.yaml --common-res 0.05 ... --out results.csv
    ros2 run amr_evaluation slam3d_eval cloud --gt-ply gt/surface_points.ply --est-ply map.ply \
        --gt-labels 0 2 3 4 5 6 7 8 9 --roi -30 30 -20 20 ... --out results.csv

궤적 형식: .tum(t x y z qx qy qz qw) 또는 .csv(t, x, y, yaw). 정렬은 none 이 주 결과이고 --align 으로 보조
정렬(origin, se2)을 더한다. 지도 GT 는 gen_warehouse_world.py --gt-dir 출력.
"""

import argparse
import sys
from typing import Dict, List, Optional, Sequence

from amr_evaluation.slam3d import cloud, gridmap, results, se2

TOOL = 'amr_evaluation.slam3d/1'


def _traj(path: str) -> se2.Traj2D:
    return se2.read_csv_se2(path) if path.endswith('.csv') else se2.tum_to_se2(path)


def _common(args: argparse.Namespace, alignment: str = 'n/a') -> Dict[str, object]:
    factors = {}
    for f in args.factor or []:
        k, _, v = f.partition('=')
        factors[k] = v
    extra = {'seed': args.seed or '', 'gt_sha256': args.gt_sha256 or '', 'notes': args.notes or ''}
    return results.common_fields(args.run_id, args.scenario, args.bag_id, args.replay, args.row,
                                 args.stage, args.git_head, TOOL, alignment, factors, **extra)


def cmd_traj(args: argparse.Namespace) -> List[Dict[str, object]]:
    """궤적: ATE(정렬별), 거리 RPE, 시간 창 RTE, (선택) 명세 구간별 위치 오차."""
    gt, est = _traj(args.gt), _traj(args.est)
    rows: List[Dict[str, object]] = []
    for align in ['none'] + [a for a in args.align if a != 'none']:
        c = _common(args, align)
        try:
            r = se2.ate(gt, est, align, args.max_gap)
        except ValueError as exc:
            # 정렬 실패 (예: se2 정렬에 연관 표본 < 2, 리뷰 N6) — CLI 를 멈추지 않고 failed=1 행을 남긴다
            # (results.py: 실패 실행도 행을 남긴다 — 평균에서 빼지 않기 위해서다)
            n_used = len(se2.associate(gt, est, args.max_gap)[0])
            nan = dict(se2.stats([]), n=n_used)
            note = f"{c.get('notes') or ''} 정렬 {align} 실패: {exc}".strip()
            cf = dict(c, failed=1, notes=note)
            rows += results.rows_from_stats(cf, 'ate_trans', nan, 'm', segment='all')
            rows += results.rows_from_stats(cf, 'ate_yaw', nan, 'deg', segment='all')
            continue
        rows += results.rows_from_stats(c, 'ate_trans', r['trans'], 'm', segment='all')
        rows += results.rows_from_stats(c, 'ate_yaw', r['yaw_deg'], 'deg', segment='all')
    c = _common(args, 'n/a')
    for d, r in se2.rpe(gt, est, args.rpe_deltas, not args.consecutive, args.max_gap).items():
        p = f'delta={d:g}m'
        rows += results.rows_from_stats(c, 'rpe_trans', r['trans'], 'm', param=p)
        rows += results.rows_from_stats(c, 'rpe_trans_pct', r['trans_pct'], '%', param=p)
        rows += results.rows_from_stats(c, 'rpe_rot', r['rot_deg'], 'deg', param=p)
    r = se2.rte(gt, est, args.rte_window, not args.consecutive, args.max_gap)
    p = f'window={args.rte_window:g}s'
    rows += results.rows_from_stats(c, 'rte_trans', r['trans'], 'm', param=p)
    rows += results.rows_from_stats(c, 'rte_rot', r['rot_deg'], 'deg', param=p)
    if args.by_motion:
        for seg, r in se2.ate_by_motion(gt, est, args.max_gap).items():
            rows += results.rows_from_stats(_common(args, 'none'), 'loc_trans', r['trans'], 'm',
                                            segment=seg, param=f"limit={r['limit']:g}m")
    return rows


def cmd_grid(args: argparse.Namespace) -> List[Dict[str, object]]:
    """2D 지도: GT 래스터 대비 ADNN₀·Chamfer₀·P/R/F1@τ (filled·surface 두 기준) + 공통 격자 IoU."""
    gt = gridmap.read_map_yaml(args.gt_map)
    est = gridmap.read_map_yaml(args.est_map)
    m = gridmap.grid_metrics(est, gt, args.tau, args.observed_margin)
    c = _common(args, 'none')
    base = f"gt={args.gt_tag or args.gt_map};tau={m['tau']:g}"
    ne, ns, nf = m['n_est'], m['n_gt_observed'], m['n_gt_observed_filled']
    rows: List[Dict[str, object]] = []
    for ref, sfx in (('filled', ''), ('surface', '_surface')):
        p = f'{base};ref={ref}'
        adnn = results.value_row(c, 'map_adnn0', m[f'adnn0{sfx}'], 'm', ne, param=p)
        rows += [adnn, dict(adnn, stat='median', value=m[f'adnn0{sfx}_median']),
                 dict(adnn, stat='p95', value=m[f'adnn0{sfx}_p95']),
                 results.value_row(c, 'map_chamfer0', m[f'chamfer0{sfx}'], 'm', ne, param=p),
                 results.value_row(c, 'map_precision',
                                   m['precision_tau' if ref == 'filled' else
                                     'precision_surface_tau'], 'ratio', ne, param=p),
                 results.value_row(c, 'map_recall',
                                   m['recall_filled_tau' if ref == 'filled' else 'recall_tau'],
                                   'ratio', nf if ref == 'filled' else ns, param=p)]
    rows.append(results.value_row(c, 'map_f1', m['f1_tau'], 'ratio', ne,
                                  param=f'{base};ref=surface'))
    if args.common_res:
        spec = gridmap.common_spec(args.common_res, (gt.origin_x,
                                                     gt.origin_x + gt.shape[1] * gt.resolution,
                                                     gt.origin_y,
                                                     gt.origin_y + gt.shape[0] * gt.resolution))
        i = gridmap.iou_common(est, gt, spec, 1, args.observed_margin)
        q = f"gt={args.gt_tag or args.gt_map};common={args.common_res:g};tau=1cell;ref=filled"
        rows.append(results.value_row(c, 'map_iou', i['iou_tau'], 'ratio',
                                      i['tp'] + i['fp'] + i['fn'], param=q))
    return rows


def cmd_cloud(args: argparse.Namespace) -> List[Dict[str, object]]:
    """3D 점군: F-score·AC/COM·Chamfer (+ 유령 점 비율)."""
    gt_arr = cloud.read_ply(args.gt_ply)
    roi = tuple(args.roi) if args.roi else None
    gt = cloud.select(gt_arr, args.gt_labels, roi, tuple(args.z) if args.z else None)
    est_arr = cloud.read_ply(args.est_ply)
    est = cloud.xyz_of(est_arr)
    if roi is not None or args.z:
        keep = cloud.select(est_arr, None, roi, tuple(args.z) if args.z else None)
        est = keep
    if args.voxel:
        est = cloud.voxel_downsample(est, args.voxel)
    m = cloud.cloud_metrics(est, gt, args.taus)
    c = _common(args, 'none')
    base = f"labels={','.join(map(str, args.gt_labels)) if args.gt_labels else 'all'};" \
           f"roi={'x'.join(map(str, args.roi)) if args.roi else 'all'};voxel={args.voxel:g}"
    rows = []
    for tau in args.taus:
        k = f'{tau:g}'
        p = f'{base};tau={k}'
        rows += [results.value_row(c, 'cloud_precision', m[f'precision@{k}'], 'ratio',
                                   m['n_est'], param=p),
                 results.value_row(c, 'cloud_recall', m[f'recall@{k}'], 'ratio', m['n_gt'],
                                   param=p),
                 results.value_row(c, 'cloud_fscore', m[f'fscore@{k}'], 'ratio', m['n_est'],
                                   param=p),
                 results.value_row(c, 'cloud_ac', m[f'ac@{k}'], 'm', m[f'n_inlier@{k}'],
                                   param=p),
                 results.value_row(c, 'cloud_com', m[f'com@{k}'], 'ratio', m['n_gt'], param=p)]
    rows += [results.value_row(c, 'cloud_chamfer_sum', m['chamfer_sum'], 'm', m['n_est'],
                               param=base),
             results.value_row(c, 'cloud_chamfer_half', m['chamfer_half'], 'm', m['n_est'],
                               param=base),
             dict(results.value_row(c, 'cloud_acc', m['acc_mean'], 'm', m['n_est'], param=base),
                  stat='mean'),
             dict(results.value_row(c, 'cloud_comp', m['comp_mean'], 'm', m['n_gt'], param=base),
                  stat='mean')]
    if args.dynamic_map:
        dyn = gridmap.read_map_yaml(args.dynamic_map)
        region = cloud.region_from_raster(dyn.occupied, dyn.resolution, dyn.origin_x,
                                          dyn.origin_y)
        g = cloud.ghost_ratio(est, gt, args.ghost_tau, region)
        p = f'{base};tau={args.ghost_tau:g};region={args.dynamic_map}'
        rows += [results.value_row(c, 'ghost_ratio', g['ghost'], 'ratio', g['n_est'], param=p),
                 results.value_row(c, 'unexplained_ratio', g['unexplained'], 'ratio',
                                   g['n_est'], param=p)]
    return rows


def build_parser() -> argparse.ArgumentParser:
    """인자 파서."""
    ap = argparse.ArgumentParser(description='보너스 2 (3D SLAM) 평가 → 결과 CSV')
    sub = ap.add_subparsers(dest='cmd', required=True)

    def common(p: argparse.ArgumentParser) -> None:
        p.add_argument('--out', required=True, help='결과 CSV (있으면 덧붙인다)')
        p.add_argument('--run-id', required=True)
        p.add_argument('--scenario', required=True, help='S1–S7 | MAP | SYN')
        p.add_argument('--bag-id', required=True)
        p.add_argument('--replay', type=int, default=0)
        p.add_argument('--row', required=True, help='비교 행 이름')
        p.add_argument('--stage', required=True,
                       choices=sorted(results.ENUMS['stage']))
        p.add_argument('--factor', action='append', default=None, metavar='KEY=VALUE',
                       help='sensor | wheel_prior | dof | wheel_quality | aisle_end')
        p.add_argument('--git-head', required=True)
        p.add_argument('--seed', default='')
        p.add_argument('--gt-sha256', default='')
        p.add_argument('--notes', default='')

    t = sub.add_parser('traj', help='궤적 ATE/RPE/RTE')
    common(t)
    t.add_argument('--gt', required=True)
    t.add_argument('--est', required=True)
    t.add_argument('--align', nargs='*', default=['origin', 'se2'],
                   choices=['none', 'origin', 'se2'], help='주 결과 none 에 더할 보조 정렬')
    t.add_argument('--rpe-deltas', type=float, nargs='+', default=[1.0, 2.0, 5.0, 10.0, 20.0])
    t.add_argument('--rte-window', type=float, default=10.0)
    t.add_argument('--consecutive', action='store_true',
                   help='RPE/RTE 쌍을 겹치지 않는 연속 구간으로 (기본: 모든 시작점)')
    t.add_argument('--max-gap', type=float, default=0.1)
    t.add_argument('--by-motion', action='store_true', help='명세 구간(정지/직선/회전)별 위치 오차')
    t.set_defaults(func=cmd_traj)

    g = sub.add_parser('grid', help='2D 지도 대 GT 래스터')
    common(g)
    g.add_argument('--gt-map', required=True)
    g.add_argument('--gt-tag', default='', help='결과 param 에 남길 GT 이름 (기본: 경로)')
    g.add_argument('--est-map', required=True)
    g.add_argument('--tau', type=float, default=None, help='기본: 추정 지도 해상도')
    g.add_argument('--observed-margin', type=float, default=0.10)
    g.add_argument('--common-res', type=float, default=0.05, help='0 이면 IoU 생략')
    g.set_defaults(func=cmd_grid)

    c = sub.add_parser('cloud', help='3D 점군 지도 대 GT 표면 점')
    common(c)
    c.add_argument('--gt-ply', required=True)
    c.add_argument('--est-ply', required=True)
    c.add_argument('--gt-labels', type=int, nargs='*', default=None,
                   help='GT 부류 번호 (gt_meta.json classes 순서; 기본 전부)')
    c.add_argument('--roi', type=float, nargs=4, default=None,
                   metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'))
    c.add_argument('--z', type=float, nargs=2, default=None, metavar=('ZMIN', 'ZMAX'))
    c.add_argument('--voxel', type=float, default=0.0, help='추정 점군 복셀 솎기 [m]')
    c.add_argument('--taus', type=float, nargs='+', default=[0.05, 0.10])
    c.add_argument('--dynamic-map', default='', help='dynamic_swept_*.yaml (유령 점 비율)')
    c.add_argument('--ghost-tau', type=float, default=0.10)
    c.set_defaults(func=cmd_cloud)
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI 진입점."""
    args = build_parser().parse_args(argv)
    rows = args.func(args)
    n = results.write_csv(args.out, rows)
    print(f'{n} rows → {args.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
