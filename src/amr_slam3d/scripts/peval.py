#!/usr/bin/env python3
"""
보너스 2 P 단계 분석 (사전 등록 스크립트) — 재생 출력 → 재생별 지표 → 사전 등록 짝 비교.

    peval.py cache   --bag <태그> --out <캐시>          # 컨테이너(rosbag2_py) 안
    peval.py replays --root R --stage e2 --cache C --out R/analysis/e2/replays.jsonl \
        [--window-stage e2]
    peval.py compare --replays R/analysis/e2/replays.jsonl --out R/analysis/e2
        # E2·E3(a)·E5(매핑)
    peval.py compare-loc --replays R/analysis/e3/replays.jsonl --out R/analysis/e3
        # E3(b)·E5(위치추정)
    peval.py compare-abl --replays R/analysis/e4/replays.jsonl R/analysis/e2/replays.jsonl \
        --out R/analysis/e4                                                            # E4

정의 (노트 항목 `2026-10-09_*_b2-eval_*` 의 사전 등록과 같다 — 바꾸면 "결과 뒤 변경"):
- 궤적: slam_probe 의 traj_online.tum (입력 스탬프마다 map → base_footprint, map = 월드) 대 bag 의 gt.tum.
  평가 창 = 그 bag 에서 창 단계(--window-stage)의 재생 중 GT 구간을 90 % 이상 덮는 것들의 공통 구간.
  주 = 정렬 없는 SE(2) xy ATE (RMSE·max·p95), 보조 = SE(2) Umeyama 정렬 RMSE, yaw,
  거리 RPE 1·5·10 m (모든 시작점),
  명세 구간(정지·직선·회전) RMSE·max·p95, 통로 상자 안 축(y)·횡(x) 오차 (S3·S6), 3D 행의 z·roll·pitch.
- 2D 지도: probe 의 map_world.yaml 대 GT 0.01 m 단면 (z 0.20, gen_warehouse_world --gt-dir),
  τ = 0.05 m, 관측 여유 0.10 m, 주 = ADNN₀ surface, 정밀도·재현율·F1 surface, ADNN₀ filled,
  IoU (공통 0.05 m). S6 대기열·벽은 블로커 발자국 + 0.10 m 를 양쪽에서 가린다.
- 3D 점 지도 (3D 행): bag 의 lidar3d/points 중 **GT 움직임 키프레임**(직전 키프레임에서 0.10 m 또는 0.10 rad)을
  행의 궤적(같은 스탬프의 6DoF 자세)으로 월드에 붙이고 (거리 0.2–20 m, 장마다 0.05 m 복셀), 전체 0.05 m 복셀.
  GT = surface_points.ply 를 0.05 m 복셀. 영역 = GT 궤적 xy 에서 10 m 안 × z ∈ [−0.10, 3.0] (양쪽 같은 자름 —
  GT 를 깎는 조작이므로 이름 `roi=traj10m_z3`). F-score·정밀도·재현율 @5·10 cm, AC@5 cm.
- 실패: rc ≠ 0, 처리 스캔 판정 누락 > 0 (RTAB-Map·LIO-SAM — replaystat),
  slam_toolbox 는 "Message Filter dropping" > 0
  (처리 수 근거가 없어 대용), 치명 패턴 > 0, 궤적이 GT 구간의 90 % 미만. 실패 재생도 지표를 남기고 평균에서 빼지 않는다.
"""

import argparse
import json
import math
import os
import re
import sys

import numpy as np

REC = '/home/CAPYI_AI/runs/b24/b2-rec/rec'
GT_DIR = '/home/CAPYI_AI/runs/s2-eval/gt_full'
GT_GRID = 'slice_z0.200_r0.010.yaml'
LIDAR_TOPIC = '/amr_01/lidar3d/points'
BASE = 'amr_01/base_footprint'
KEY_D, KEY_A = 0.10, 0.10
RANGE = (0.2, 20.0)
VOXEL = 0.05
ROI_R, ROI_Z = 10.0, (-0.10, 3.0)
TAUS = (0.05, 0.10)
COVER_MIN = 0.90
S6_VARIANT = {'s6_open_a3': 'open', 's6_queue_a1': 'queue', 's6_wall_a1': 'wall'}
AISLE_BAGS = ('s3_aisle_a1', 's6_open_a3', 's6_queue_a1', 's6_wall_a1')
DIM3 = ('rtabmap3d', 'liosam', 'rtabloc')


def _row_is3d(row):
    return row.startswith(DIM3)


# ======================================================================== cache (컨테이너 안)
def cmd_cache(ns):
    """한 bag 의 캐시: GT 키프레임 점군(센서 프레임), base←sensor, 영역 래스터, GT 점 (0.05 복셀)."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import bag_tools
    from amr_evaluation.slam3d import cloud, pmetrics, se2
    from amr_slam3d import bagstat, cloudio
    from rclpy.serialization import deserialize_message
    import rosbag2_py
    from sensor_msgs.msg import PointCloud2
    rec = os.path.join(ns.rec, ns.bag)
    bag = os.path.join(rec, 'bag')
    gt = se2.tum_to_se2(os.path.join(rec, 'gt.tum'))
    reader, _ = bag_tools._reader(bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[LIDAR_TOPIC]))
    stamps, chunks, last, frame, n_all = [], [], None, None, 0
    while reader.has_next():
        _, raw, _ = reader.read_next()
        n_all += 1
        st = bagstat.cdr_header_stamp(raw)
        if st < gt.t[0] or st > gt.t[-1]:
            continue
        k = int(np.clip(np.searchsorted(gt.t, st), 1, len(gt.t) - 1))
        w = (st - gt.t[k - 1]) / (gt.t[k] - gt.t[k - 1])
        p = gt.xy[k - 1] + w * (gt.xy[k] - gt.xy[k - 1])
        yaw = float(se2.wrap(gt.yaw[k - 1] + w * se2.wrap(gt.yaw[k] - gt.yaw[k - 1])))
        if last is not None and (np.linalg.norm(p - last[0]) < KEY_D
                                 and abs(float(se2.wrap(yaw - last[1]))) < KEY_A):
            continue
        last = (p, yaw)
        msg = deserialize_message(raw, PointCloud2)
        frame = msg.header.frame_id
        off = {f.name: f.offset for f in msg.fields if f.datatype == 7}
        arr = np.frombuffer(bytes(msg.data), dtype=np.dtype(
            {'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
             'offsets': [off['x'], off['y'], off['z']], 'itemsize': msg.point_step}),
            count=msg.width * msg.height)
        xyz = cloudio.finite(np.column_stack([arr['x'], arr['y'], arr['z']]).astype(float))
        r = np.linalg.norm(xyz, axis=1)
        xyz = cloudio.voxel_downsample(xyz[(r >= RANGE[0]) & (r <= RANGE[1])], VOXEL)
        stamps.append(st)
        chunks.append(xyz.astype(np.float32))
    base_t_sensor = bag_tools._static_tf(bag, BASE, frame)
    region = pmetrics.region_raster(gt.xy, ROI_R)
    arr = cloud.read_ply(os.path.join(GT_DIR, 'surface_points.ply'))
    keep = np.zeros(len(arr), dtype=bool)
    step = 5_000_000
    for i in range(0, len(arr), step):
        a = arr[i:i + step]
        z = np.asarray(a['z'])
        m = (z >= ROI_Z[0]) & (z <= ROI_Z[1])
        idx = np.nonzero(m)[0]
        xyz = np.column_stack([np.asarray(a['x'])[idx], np.asarray(a['y'])[idx], z[idx]])
        m2 = pmetrics.in_region(region, xyz)
        keep[i + idx[m2]] = True
    sel = np.nonzero(keep)[0]
    gt_xyz = np.column_stack([np.asarray(arr['x'][sel]), np.asarray(arr['y'][sel]),
                              np.asarray(arr['z'][sel])]).astype(float)
    gt_xyz = cloudio.voxel_downsample(gt_xyz, VOXEL)
    os.makedirs(ns.out, exist_ok=True)
    lens = np.array([len(c) for c in chunks])
    np.savez_compressed(os.path.join(ns.out, 'keyframes.npz'), stamps=np.array(stamps),
                        offsets=np.concatenate([[0], np.cumsum(lens)]),
                        pts=np.concatenate(chunks) if chunks else np.zeros((0, 3), np.float32),
                        base_t_sensor=base_t_sensor, frame=frame)
    np.savez_compressed(os.path.join(ns.out, 'region.npz'), mask=region['mask'],
                        x0=region['x0'], y0=region['y0'], cell=region['cell'],
                        radius=region['radius'])
    np.save(os.path.join(ns.out, 'gt_cloud.npy'), gt_xyz.astype(np.float32))
    info = {'bag': ns.bag, 'clouds_in_bag': n_all, 'keyframes': len(stamps),
            'keyframe_points': int(lens.sum()), 'gt_points_voxel': int(len(gt_xyz)),
            'frame': frame, 'rule': {'key_d': KEY_D, 'key_a': KEY_A, 'range': RANGE,
                                     'voxel': VOXEL, 'roi_r': ROI_R, 'roi_z': ROI_Z}}
    with open(os.path.join(ns.out, 'cache.json'), 'w', encoding='utf-8') as f:
        json.dump(info, f, ensure_ascii=False, indent=1)
    print(json.dumps(info, ensure_ascii=False))
    return 0


# ======================================================================== replays
def _json(path):
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def list_replays(root, stage):
    """R/<stage>/<bag>/<row>/r<k>/ (DONE 있는 것) → [(bag, row, k, dir)]."""
    out = []
    base = os.path.join(root, stage)
    if not os.path.isdir(base):
        return out
    for bag in sorted(os.listdir(base)):
        for row in sorted(os.listdir(os.path.join(base, bag)) if os.path.isdir(
                os.path.join(base, bag)) else []):
            d = os.path.join(base, bag, row)
            if not os.path.isdir(d):
                continue
            for rk in sorted(os.listdir(d)):
                m = re.fullmatch(r'r(\d+)', rk)
                if m and os.path.isfile(os.path.join(d, rk, 'DONE')):
                    out.append((bag, row, int(m.group(1)), os.path.join(d, rk)))
    return out


def _read_tum3(path):
    from amr_evaluation.slam3d import se2
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        return None
    t, xyz, q = se2.read_tum(path)
    if len(t) == 0:
        return None
    o = np.argsort(t)
    t, xyz, q = t[o], xyz[o], q[o]
    keep = np.concatenate([[True], np.diff(t) > 0])
    return t[keep], xyz[keep], q[keep]


def drops_of(d, row):
    """처리 스캔 판정 누락·치명·로그 (계열별)."""
    from amr_slam3d import replaystat
    scan = _json(os.path.join(d, 'log_scan.json')) or {}
    out = {'missing': None, 'fatal': None, 'msgfilter_drop': None, 'null_odom': None}
    try:
        fam = replaystat.family(row)
    except ValueError:
        fam = None
    if fam in ('rtabmap', 'liosam'):
        s = replaystat.summarize(d, row)
        out.update({k: s.get(k) for k in ('missing', 'missing_log', 'match_missing',
                                          'missing_disagree', 'missing_stamps', 'processed',
                                          'input', 'tail_missing', 'null_odom',
                                          'imu_wait_drops', 'fatal', 'launch_exit',
                                          'db_optimized_poses', 'load_play_start',
                                          'load_play_end')})
    elif scan:
        out['fatal'] = sum((scan.get('fatal') or {}).values())
    common = scan.get('common') or {}
    fatal = scan.get('fatal') or {}
    out['msgfilter_drop'] = int(common.get('Message Filter dropping', 0)
                                + fatal.get('Message Filter dropping', 0)) if scan else None
    return out


def window_for(gt_t, trajs):
    """덮음 ≥ COVER_MIN 인 궤적들의 공통 구간 (없으면 GT 구간)."""
    from amr_evaluation.slam3d import pmetrics
    ok = [t for t in trajs if t is not None and pmetrics.coverage(t, gt_t) >= COVER_MIN]
    if not ok:
        return float(gt_t[0]), float(gt_t[-1])
    return max(float(t[0]) for t in ok), min(float(t[-1]) for t in ok)


def traj_metrics(gt2, gt3, est3, window, bag, row):
    """궤적 지표 (cm·deg·%)."""
    from amr_evaluation.slam3d import pmetrics, se2, symmetric
    t, xyz, q = est3
    yaw = se2.quat_to_rpy(q)[:, 2]
    est = se2.Traj2D(t, xyz[:, :2], yaw)
    m = symmetric.in_window(est.t, window)
    e = se2.Traj2D(est.t[m], est.xy[m], est.yaw[m])
    out = {'n_window': int(len(e)), 'coverage': pmetrics.coverage(t, gt2.t)}
    a = se2.ate(gt2, e, 'none')
    out['n'] = int(a['n_used'])
    if a['n_used'] < 2:
        return out
    tr, yw = a['trans'], a['yaw_deg']
    out.update({'ate_rmse_cm': 100 * tr['rmse'], 'ate_max_cm': 100 * tr['max'],
                'ate_p95_cm': 100 * tr['p95'], 'ate_mean_cm': 100 * tr['mean'],
                'yaw_rmse_deg': yw['rmse'], 'yaw_max_deg': yw['max']})
    s = se2.ate(gt2, e, 'se2')
    out['ate_se2_rmse_cm'] = 100 * s['trans']['rmse']
    rp = se2.rpe(gt2, e, (1.0, 5.0, 10.0), True)
    for d, v in rp.items():
        k = f'{d:g}'
        out[f'rpe{k}_cm'] = 100 * v['trans']['rmse']
        out[f'rpe{k}_pct'] = v['trans_pct']['rmse']
        out[f'rpe{k}_rot_deg'] = v['rot_deg']['rmse']
    seg = se2.ate_by_motion(gt2, e)
    names = {'정지': 'still', '직선': 'straight', '회전': 'turn'}
    for name, v in seg.items():
        k = names[name]
        st = v['trans']
        out[f'seg_{k}_rmse_cm'] = 100 * st['rmse']
        out[f'seg_{k}_max_cm'] = 100 * st['max']
        out[f'seg_{k}_p95_cm'] = 100 * st['p95']
        out[f'seg_{k}_n'] = st['n']
        out[f'seg_{k}_rmse_ok'] = v['rmse_ok']
        out[f'seg_{k}_max_ok'] = v['max_ok']
    if bag in AISLE_BAGS:
        g, ee = se2.associate(gt2, e)
        ae = pmetrics.aisle_errors(g.xy, ee.xy)
        for part in ('axis', 'lateral'):
            for st in ('rmse', 'max', 'p95', 'mean'):
                out[f'aisle_{part}_{st}_cm'] = 100 * ae[part][st]
            out[f'aisle_{part}_n'] = ae[part]['n']
    if _row_is3d(row):
        tm = symmetric.in_window(t, window)
        r = se2.z_rp_errors(gt3[0], gt3[1], gt3[2], t[tm], xyz[tm], q[tm])
        out.update({'z_max_cm': 100 * r['z']['max'], 'z_rmse_cm': 100 * r['z']['rmse'],
                    'roll_rmse_deg': r['roll_deg']['rmse'],
                    'pitch_rmse_deg': r['pitch_deg']['rmse']})
    return out


class GridEval:
    """GT 0.01 단면 (한 번 읽음) → 지도 지표."""

    def __init__(self, gt_file=GT_GRID):
        from amr_evaluation.slam3d import gridmap
        self.gm = gridmap
        self.gt = gridmap.read_map_yaml(os.path.join(GT_DIR, gt_file))

    def metrics(self, yaml_path, bag):
        from amr_evaluation.slam3d import localizability, pmetrics
        est = self.gm.read_map_yaml(yaml_path)
        gt = self.gt
        rects = []
        if bag in S6_VARIANT:
            rects = pmetrics.blocker_rects(localizability.s6_blockers(S6_VARIANT[bag]))
            if rects:
                est, gt = pmetrics.mask_boxes(est, rects), pmetrics.mask_boxes(gt, rects)
        m = self.gm.grid_metrics(est, gt, 0.05, 0.10)
        spec = self.gm.common_spec(0.05, (gt.origin_x, gt.origin_x + gt.shape[1] * gt.resolution,
                                          gt.origin_y, gt.origin_y + gt.shape[0] * gt.resolution))
        iou = self.gm.iou_common(est, gt, spec, 1, 0.10)
        return {'map_adnn_surf_cm': 100 * m['adnn0_surface'],
                'map_adnn_surf_p95_cm': 100 * m['adnn0_surface_p95'],
                'map_adnn_fill_cm': 100 * m['adnn0'], 'map_prec_surf': m['precision_surface_tau'],
                'map_recall_surf': m['recall_tau'], 'map_f1': m['f1_tau'],
                'map_iou': iou['iou_tau'], 'map_n_est': m['n_est'],
                'map_n_gt_obs': m['n_gt_observed'], 'map_masked_rects': len(rects)}


class CloudEval:
    """bag 캐시 (키프레임·GT 점·영역) → 3D F-score."""

    def __init__(self, cache_dir):
        from scipy.spatial import cKDTree
        k = np.load(os.path.join(cache_dir, 'keyframes.npz'))
        self.stamps = k['stamps']
        self.off = k['offsets']
        self.pts = k['pts'].astype(float)
        self.b_t_s = k['base_t_sensor']
        r = np.load(os.path.join(cache_dir, 'region.npz'))
        self.region = {'mask': r['mask'], 'x0': float(r['x0']), 'y0': float(r['y0']),
                       'cell': float(r['cell'])}
        self.gt = np.load(os.path.join(cache_dir, 'gt_cloud.npy')).astype(float)
        self.gt_tree = cKDTree(self.gt)

    def metrics(self, est3):
        from amr_evaluation.slam3d import pmetrics
        from amr_slam3d import cloudio
        from scipy.spatial import cKDTree
        t, xyz, q = est3
        idx = {round(float(s), 4): i for i, s in enumerate(t)}
        chunks, used = [], 0
        for j, st in enumerate(self.stamps):
            i = idx.get(round(float(st), 4))
            if i is None:
                continue
            m = cloudio.pose_matrix(xyz[i], q[i]) @ self.b_t_s
            chunks.append(cloudio.apply(m, self.pts[self.off[j]:self.off[j + 1]]))
            used += 1
        out = {'cloud_keyframes_used': used, 'cloud_keyframes': int(len(self.stamps))}
        if not chunks:
            return out
        p = np.concatenate(chunks)
        p = p[(p[:, 2] >= ROI_Z[0]) & (p[:, 2] <= ROI_Z[1]) & pmetrics.in_region(self.region, p)]
        p = cloudio.voxel_downsample(p, VOXEL)
        if len(p) == 0:
            return out
        d_eg, _ = self.gt_tree.query(p, distance_upper_bound=0.2)
        d_ge, _ = cKDTree(p).query(self.gt, distance_upper_bound=0.2)
        out['cloud_n_est'] = int(len(p))
        out['cloud_n_gt'] = int(len(self.gt))
        for tau in TAUS:
            k = f'{int(round(tau * 100))}'
            pr = float(np.mean(d_eg <= tau))
            rc = float(np.mean(d_ge <= tau))
            out[f'cloud_prec{k}'] = pr
            out[f'cloud_recall{k}'] = rc
            out[f'cloud_f{k}'] = 2 * pr * rc / (pr + rc) if pr + rc > 0 else 0.0
        inl = d_eg <= 0.05
        out['cloud_ac5_cm'] = 100 * float(d_eg[inl].mean()) if inl.any() else math.nan
        return out


GT_BAND = 'band_z0.050-2.000_r0.010.yaml'
#: 위치추정 발산 (사전 등록, E3(b)): 평가 창 안 xy 오차 최대 > 0.5 m 또는 yaw 오차 최대 > 10°
DIVERGE_XY_CM, DIVERGE_YAW_DEG = 50.0, 10.0


def is_loc_row(row):
    """E3(b) 위치추정 행 (AMCL·RTAB-Map 위치추정)."""
    return row.startswith(('amcl_', 'rtabmap3d_loc_'))


def loc_fail_of(rec):
    """
    위치추정 행의 실패 사유 (E3(b) 사전 등록): 기동·처리 실패 + 발산·LOST.

    처리: rc ≠ 0, 덮음 < 90 %, RTAB-Map 은 처리 스캔 판정 누락 > 0, AMCL 은 "Message Filter dropping" > 0 (대용).
    발산: ate_max_cm > 50 또는 yaw_max_deg > 10.
    LOST: RTAB-Map icp_odometry null odom (첫 장 1 개 제외) > 0.
    """
    why = []
    if rec.get('rc') not in (0, None):
        why.append(f"rc={rec.get('rc')}")
    if (rec.get('coverage') or 0.0) < COVER_MIN:
        why.append(f"coverage={rec.get('coverage', 0):.2f}")
    if rec.get('row', '').startswith('amcl_'):
        if rec.get('msgfilter_drop'):
            why.append(f"msgfilter={rec['msgfilter_drop']}")
    else:
        if rec.get('missing') is None:
            why.append('missing=미측정')
        elif rec.get('missing'):
            why.append(f"missing={rec['missing']}")
        if (rec.get('null_odom') or 0) > 1:
            why.append(f"lost={rec['null_odom'] - 1}")
    if rec.get('fatal'):
        why.append(f"fatal={rec['fatal']}")
    mx, yw = rec.get('ate_max_cm'), rec.get('yaw_max_deg')
    if (mx is not None and mx > DIVERGE_XY_CM) or (yw is not None and yw > DIVERGE_YAW_DEG):
        why.append('diverged')
    return why


def fail_of(rec):
    """실패 사유 목록 (사전 등록 규칙)."""
    why = []
    if rec.get('rc') not in (0, None):
        why.append(f"rc={rec.get('rc')}")
    if rec.get('missing') not in (None, 0):
        why.append(f"missing={rec['missing']}")
    if rec.get('row', '').startswith('slam_toolbox') and rec.get('msgfilter_drop'):
        why.append(f"msgfilter={rec['msgfilter_drop']}")
    if rec.get('fatal'):
        why.append(f"fatal={rec['fatal']}")
    if (rec.get('coverage') or 0.0) < COVER_MIN:
        why.append(f"coverage={rec.get('coverage', 0):.2f}")
    if rec.get('missing') is None and rec.get('family') in ('rtabmap', 'liosam'):
        why.append('missing=미측정')
    return why


def cmd_replays(ns):
    """재생별 지표 → jsonl (bag 마다 창 계산)."""
    from amr_evaluation.slam3d import se2
    runs = list_replays(ns.root, ns.stage)
    if ns.bags:
        runs = [r for r in runs if r[0] in ns.bags]
    wstage = ns.window_stage or ns.stage
    wruns = list_replays(ns.root, wstage)
    grid = GridEval() if not ns.no_grid else None
    band = GridEval(GT_BAND) if ns.grid_band and not ns.no_grid else None
    os.makedirs(os.path.dirname(os.path.abspath(ns.out)), exist_ok=True)
    out = open(ns.out, 'w', encoding='utf-8')
    windows = {}
    for bag in sorted({r[0] for r in runs}):
        rec = os.path.join(ns.rec, bag)
        gt2 = se2.tum_to_se2(os.path.join(rec, 'gt.tum'))
        gt3 = se2.read_tum(os.path.join(rec, 'gt.tum'))
        wt = []
        for b, row, k, d in wruns:
            if b == bag and (not ns.window_rows or row in ns.window_rows):
                tr = _read_tum3(os.path.join(d, 'traj_online.tum'))
                wt.append(tr[0] if tr else None)
        window = window_for(gt2.t, wt)
        if ns.warmup:
            # E3(b) 위치추정: 시작 정지 구간의 처음 warmup 초는 평가에서 뺀다 (배포 map EKF 는 첫 amcl_pose 전 원점에
            # 있다 — 두 사슬에 같은 규칙, 모든 bag 은 처음 ≥ 10 s 정지)
            window = (window[0] + ns.warmup, window[1])
        windows[bag] = window
        ce = None
        if not ns.no_cloud and any(_row_is3d(r[1]) for r in runs if r[0] == bag):
            cdir = os.path.join(ns.cache, bag)
            if os.path.isfile(os.path.join(cdir, 'gt_cloud.npy')):
                ce = CloudEval(cdir)
        for b, row, k, d in runs:
            if b != bag:
                continue
            job = _json(os.path.join(d, 'job.json')) or {}
            done = _json(os.path.join(d, 'DONE')) or {}
            r = {'stage': ns.stage, 'bag': bag, 'row': row, 'replay': k, 'dir': d,
                 'rc': done.get('rc'), 'started': job.get('started'), 'ended': done.get('ended'),
                 'load_start': job.get('load_start'), 'load_end': done.get('load_end'),
                 'rate': job.get('rate'), 'head': job.get('head'), 'window': window,
                 'preregistered_bag': bag != 's1_seed2x_a1'}
            try:
                from amr_slam3d import replaystat
                r['family'] = replaystat.family(row)
            except ValueError:
                r['family'] = row.split('_')[0]
            r.update(drops_of(d, row))
            est3 = _read_tum3(os.path.join(d, 'traj_online.tum'))
            if est3 is not None:
                r.update(traj_metrics(gt2, gt3, est3, window, bag, row))
                if ce is not None and _row_is3d(row):
                    r.update(ce.metrics(est3))
            else:
                r['coverage'] = 0.0
            ymap = os.path.join(d, 'map_world.yaml')
            if grid is not None and os.path.isfile(ymap) and not is_loc_row(row):
                try:
                    r.update(grid.metrics(ymap, bag))
                    if band is not None:
                        r.update({k + '_band': v for k, v in band.metrics(ymap, bag).items()
                                  if k.startswith('map_')})
                except ValueError as exc:
                    r['map_error'] = str(exc)
            log = os.path.join(d, 'row.log')
            if row.startswith(('rtabmap', 'rtabloc')) and os.path.isfile(log):
                from amr_evaluation.slam3d import pmetrics
                txt = open(log, encoding='utf-8', errors='replace').read()
                r['lat_icp'] = pmetrics.icp_latency(txt)
                r['lat_rtabmap'] = pmetrics.rtabmap_latency(txt)
            r['fail'] = loc_fail_of(r) if is_loc_row(row) else fail_of(r)
            out.write(json.dumps(r, ensure_ascii=False, default=float) + '\n')
            out.flush()
            print(f"{bag} {row} r{k} fail={r['fail']} ate={r.get('ate_rmse_cm')}", flush=True)
    out.close()
    with open(os.path.splitext(ns.out)[0] + '_windows.json', 'w', encoding='utf-8') as f:
        json.dump(windows, f, indent=1)
    return 0


# ======================================================================== compare (사전 등록 판정)
BAGS_ALL = ('s3_aisle_a1', 's1_seed1_a1', 's1_seed3_a2', 's1_seed2x_a1', 's6_open_a3',
            's6_queue_a1', 's6_wall_a1', 'map_long_a1')
BAGS_PRE = tuple(b for b in BAGS_ALL if b != 's1_seed2x_a1')
S1_ALL = ('s1_seed1_a1', 's1_seed3_a2', 's1_seed2x_a1')
S1_PRE = ('s1_seed1_a1', 's1_seed3_a2')
#: 사전 등록 비교 가족 — (이름, A 행, B 행, 지표, δ). 차 = A − B (작을수록 좋은 지표: 음수면 A 가 낫다)
FAMILIES = {
    'T': [('C1', 'rtabmap3d_noimu', 'rtabmap2d_noimu', 'ate_rmse_cm', 0.2),
          ('C2', 'rtabmap3d_noimu', 'slam_toolbox', 'ate_rmse_cm', 0.2),
          ('C3', 'liosam_planar', 'slam_toolbox', 'ate_rmse_cm', 0.2)],
    'M': [('C4', 'rtabmap3d_noimu', 'rtabmap2d_noimu', 'map_adnn_surf_cm', 0.2),
          ('C5', 'rtabmap3d_noimu', 'slam_toolbox', 'map_adnn_surf_cm', 0.2)],
}
DESC_METRICS = ('ate_rmse_cm', 'ate_max_cm', 'ate_p95_cm', 'ate_se2_rmse_cm', 'yaw_rmse_deg',
                'rpe1_pct', 'rpe5_pct', 'rpe10_pct', 'map_adnn_surf_cm', 'map_prec_surf',
                'map_recall_surf', 'map_f1', 'map_iou', 'cloud_f5', 'cloud_f10',
                'cloud_prec5', 'cloud_recall5', 'aisle_axis_rmse_cm', 'aisle_lateral_rmse_cm',
                'z_max_cm')
E5_ROWS = ('slam_toolbox', 'rtabmap2d_noimu', 'rtabmap3d_noimu', 'liosam_planar')


def load_replays(paths):
    """여러 jsonl → 재생 목록."""
    out = []
    for p in paths:
        with open(p, encoding='utf-8') as f:
            out += [json.loads(line) for line in f if line.strip()]
    return out


def _val(r, m):
    v = r.get(m)
    return float(v) if isinstance(v, (int, float)) and v is not None else math.nan


def pairs_for(recs, a, b, metric, bags):
    """짝 목록 {bag: [(A_k, B_k)]} (같은 재생 번호)."""
    idx = {(r['bag'], r['row'], r['replay']): r for r in recs}
    out = {}
    for bag in bags:
        ps = []
        for k in range(10):
            ra, rb = idx.get((bag, a, k)), idx.get((bag, b, k))
            if ra is None or rb is None:
                continue
            ps.append((_val(ra, metric), _val(rb, metric)))
        if ps:
            out[bag] = ps
    return out


def family_tests(recs, bags, tag, families=None):
    """가족 T·M (또는 주어진 가족) — Wilcoxon 정확 + Holm + 두 단계 부트스트랩 + 판정."""
    from amr_evaluation.slam3d import pstats
    res = {}
    for fam, comps in (families or FAMILIES).items():
        rows = []
        for name, a, b, metric, delta in comps:
            pr = pairs_for(recs, a, b, metric, bags)
            bd = pstats.bag_diffs(pr)
            d = [v for v in bd.values() if math.isfinite(v)]
            w = pstats.wilcoxon_exact(d)
            bs = pstats.cluster_bootstrap(pr)
            rows.append({'name': name, 'A': a, 'B': b, 'metric': metric, 'delta': delta,
                         'bag_diffs': bd, 'n_bags': len(d), 'wilcoxon': w,
                         'min_p_possible': pstats.min_p_wilcoxon(len(d)), 'boot': bs,
                         'n_neg': sum(1 for v in d if v < 0), 'n_pos': sum(1 for v in d if v > 0)})
        adj = pstats.holm([r['wilcoxon']['p'] for r in rows])
        m = len(rows)
        for r, pa in zip(rows, adj):
            r['p_holm'] = pa
            r['rejectable'] = r['min_p_possible'] * m < 0.05
            r['verdict'] = pstats.verdict(pa, r['boot']['lo'], r['boot']['hi'], r['boot']['mean'],
                                          r['delta'])
        res[fam] = rows
    return {'bags': list(bags), 'tag': tag, 'families': res}


def per_row_tables(recs, stage_rows):
    """행 × bag 평균·sd, 행 전체 분산 성분, 실패율 Wilson."""
    from amr_evaluation.slam3d import pstats
    tab = {}
    for row in stage_rows:
        rr = [r for r in recs if r['row'] == row]
        if not rr:
            continue
        ent = {'n': len(rr), 'fail': sum(1 for r in rr if r['fail']),
               'fail_reasons': sorted({w for r in rr for w in r['fail']})}
        ent['fail_wilson'] = pstats.wilson(ent['fail'], ent['n'])
        ent['metrics'] = {}
        for m in DESC_METRICS:
            byb = {}
            for r in rr:
                v = _val(r, m)
                if math.isfinite(v):
                    byb.setdefault(r['bag'], []).append(v)
            if not byb:
                continue
            vc = pstats.variance_components(byb)
            ent['metrics'][m] = {
                'by_bag': {b: {'mean': float(np.mean(v)), 'sd': float(np.std(v, ddof=1))
                               if len(v) > 1 else 0.0, 'n': len(v), 'values': v}
                           for b, v in byb.items()},
                'mean_of_bags': float(np.mean([np.mean(v) for v in byb.values()])),
                'within_sd': vc['within_sd'], 'between_sd': vc['between_sd']}
        tab[row] = ent
    return tab


def e5(recs):
    """S6 가설: 열림 대비 대기열·벽의 통로 축 오차 (재생 짝), 2D·3D 차의 차."""
    from amr_evaluation.slam3d import pstats
    idx = {(r['bag'], r['row'], r['replay']): r for r in recs}
    out = {'per_row': {}, 'did': {}}
    met = 'aisle_axis_rmse_cm'
    for row in E5_ROWS:
        vals = {v: [_val(idx[(b, row, k)], met) for k in range(3) if (b, row, k) in idx]
                for b, v in S6_VARIANT.items()}
        ent = {'values': vals}
        for v in ('queue', 'wall'):
            a, o = vals.get(v, []), vals.get('open', [])
            ent[f'{v}_minus_open'] = (float(np.nanmean(a) - np.nanmean(o))
                                      if a and o else math.nan)
            ent[f'{v}_rule'] = pstats.replay_rule(a, o, 0.2)
        for m2 in ('aisle_lateral_rmse_cm', 'ate_rmse_cm', 'aisle_axis_max_cm'):
            ent[m2] = {v: [_val(idx[(b, row, k)], m2) for k in range(3) if (b, row, k) in idx]
                       for b, v in S6_VARIANT.items()}
        out['per_row'][row] = ent
    for d2, d3 in (('rtabmap2d_noimu', 'rtabmap3d_noimu'), ('slam_toolbox', 'rtabmap3d_noimu'),
                   ('slam_toolbox', 'liosam_planar')):
        for v in ('queue', 'wall'):
            x = out['per_row'].get(d2, {}).get(f'{v}_minus_open', math.nan)
            y = out['per_row'].get(d3, {}).get(f'{v}_minus_open', math.nan)
            out['did'][f'{d2}-vs-{d3}:{v}'] = x - y
    return out


def _f(v, nd=2):
    return '—' if v is None or (isinstance(v, float) and not math.isfinite(v)) else f'{v:.{nd}f}'


def write_md(path, res):
    """사람이 읽는 표."""
    lines = ['# P 단계 분석 결과 (peval.py compare)', '']
    for key in ('ALL8', 'PRE7', 'S1_3', 'S1_2'):
        ft = res['tests'].get(key)
        if not ft:
            continue
        lines += [f'## 가족 검정 — {key} (bag {len(ft["bags"])})', '',
                  '| 가족 | 비교 | 지표 | bag 차 평균 [95 % CI] | 음/양 bag | Wilcoxon p '
                  '| 가능 최소 p | Holm p | 기각 가능 설계 | 판정 |', '|' + '---|' * 10]
        for fam, rows in ft['families'].items():
            for r in rows:
                b = r['boot']
                lines.append(
                    f"| {fam} | {r['name']} {r['A']} − {r['B']} | {r['metric']} | "
                    f"{_f(b['mean'], 3)} "
                    f"[{_f(b['lo'], 3)}, {_f(b['hi'], 3)}] | {r['n_neg']}/{r['n_pos']} | "
                    f"{_f(r['wilcoxon']['p'], 4)} | {_f(r['min_p_possible'], 4)} | "
                    f"{_f(r['p_holm'], 4)} | {'예' if r['rejectable'] else '아니오'} | "
                    f"{r['verdict']} |")
        lines.append('')
    lines += ['## 행별 (bag 평균의 평균, 재생 간 sd / bag 간 sd, 실패)', '',
              '| 행 | n | 실패 (Wilson 95 %) | ATE RMSE cm | ATE max cm | SE(2) 정렬 cm | '
              'RPE1 % | RPE10 % | '
              'ADNN surf cm | F1 | IoU | F@5 | F@10 |', '|' + '---|' * 13]
    for row, e in res['rows'].items():
        mm = e['metrics']

        def g(m, nd=2):
            x = mm.get(m)
            if not x:
                return '—'
            return (f"{_f(x['mean_of_bags'], nd)} "
                    f"({_f(x['within_sd'], nd)}/{_f(x['between_sd'], nd)})")
        lo, hi = e['fail_wilson']
        lines.append(f"| {row} | {e['n']} | {e['fail']} [{lo:.2f}, {hi:.2f}] | "
                     f"{g('ate_rmse_cm')} | "
                     f"{g('ate_max_cm')} | {g('ate_se2_rmse_cm')} | {g('rpe1_pct')} | "
                     f"{g('rpe10_pct', 3)} | {g('map_adnn_surf_cm')} | {g('map_f1', 3)} | "
                     f"{g('map_iou', 3)} | {g('cloud_f5', 3)} | {g('cloud_f10', 3)} |")
    lines.append('')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


def cmd_compare(ns):
    """사전 등록 가족 검정 (ALL8·PRE7, S1 은 기술), 행별 표, E5."""
    recs = load_replays(ns.replays)
    rows = sorted({r['row'] for r in recs})
    res = {'tests': {'ALL8': family_tests(recs, BAGS_ALL, 'ALL8'),
                     'PRE7': family_tests(recs, BAGS_PRE, 'PRE7'),
                     'S1_3': family_tests(recs, S1_ALL, 'S1_3'),
                     'S1_2': family_tests(recs, S1_PRE, 'S1_2')},
           'rows': per_row_tables(recs, rows),
           'rows_pre7': per_row_tables([r for r in recs if r['bag'] in BAGS_PRE], rows),
           'e5': e5(recs)}
    os.makedirs(ns.out, exist_ok=True)
    with open(os.path.join(ns.out, 'compare.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=1, default=float)
    write_md(os.path.join(ns.out, 'compare.md'), res)
    print(os.path.join(ns.out, 'compare.md'))
    return 0


# ======================================================================== E3(b)·E5(위치추정)
LOC_BAGS_ALL = tuple(b for b in BAGS_ALL if b != 'map_long_a1')
LOC_BAGS_PRE = tuple(b for b in LOC_BAGS_ALL if b != 's1_seed2x_a1')
#: 가족 L1·L2 (각자 하나 — Holm 없음). 차 = 3D − 2D
LOC_FAMILIES = {
    'L1': [('L1', 'rtabmap3d_loc_gt', 'amcl_gt', 'ate_rmse_cm', 0.2)],
    'L2': [('L2', 'rtabmap3d_loc_slam', 'amcl_slam', 'ate_rmse_cm', 0.2)],
}
LOC_ROWS = ('amcl_gt', 'rtabmap3d_loc_gt', 'amcl_slam', 'rtabmap3d_loc_slam')
SEG_LIMIT = {'still': 3.0, 'straight': 5.0, 'turn': 8.0}


def loc_tests(recs, bags, tag):
    """가족 L1·L2 — family_tests 와 같은 계산 (가족마다 비교 하나)."""
    return family_tests(recs, bags, tag, LOC_FAMILIES)


def segment_table(recs):
    """행 × 구간: 재생별 RMSE·max·p95 (cm) 의 bag 평균의 평균과 팀 기준(3/5/8 cm) 통과 재생 수."""
    out = {}
    for row in LOC_ROWS + E5_ROWS:
        rr = [r for r in recs if r['row'] == row]
        if not rr:
            continue
        ent = {}
        for seg, lim in SEG_LIMIT.items():
            e = {}
            for st in ('rmse', 'max', 'p95'):
                byb = {}
                for r in rr:
                    v = _val(r, f'seg_{seg}_{st}_cm')
                    if math.isfinite(v):
                        byb.setdefault(r['bag'], []).append(v)
                e[st] = float(np.mean([np.mean(v) for v in byb.values()])) if byb else math.nan
                e[f'{st}_worst'] = max((max(v) for v in byb.values()), default=math.nan)
                e[f'{st}_pass'] = sum(1 for r in rr if _val(r, f'seg_{seg}_{st}_cm') <= lim)
            e['n'] = len(rr)
            e['limit_cm'] = lim
            ent[seg] = e
        out[row] = ent
    return out


def loc_failures(recs):
    """행별 위치추정 실패(발산·LOST 포함) 수와 Wilson 구간, 사유별 수."""
    from amr_evaluation.slam3d import pstats
    out = {}
    for row in LOC_ROWS:
        rr = [r for r in recs if r['row'] == row]
        if not rr:
            continue
        k = sum(1 for r in rr if r['fail'])
        reasons = {}
        for r in rr:
            for w in r['fail']:
                key = w.split('=')[0]
                reasons[key] = reasons.get(key, 0) + 1
        out[row] = {'n': len(rr), 'fail': k, 'wilson': pstats.wilson(k, len(rr)),
                    'diverged': sum(1 for r in rr if 'diverged' in r['fail']),
                    'reasons': reasons}
    return out


def map_effect(recs):
    """지도 품질이 위치추정에 주는 몫 (기술): (SLAM 지도 − 정답 지도) xy ATE RMSE, 같은 bag·재생 짝."""
    from amr_evaluation.slam3d import pstats
    out = {}
    for loc, (a, b) in {'2d_amcl': ('amcl_slam', 'amcl_gt'),
                        '3d_rtabmap': ('rtabmap3d_loc_slam', 'rtabmap3d_loc_gt')}.items():
        pr = pairs_for(recs, a, b, 'ate_rmse_cm', LOC_BAGS_ALL)
        out[loc] = {'bag_diffs': pstats.bag_diffs(pr), 'boot': pstats.cluster_bootstrap(pr)}
    return out


def e5_loc(recs):
    """S6 가설 (위치추정): 행마다 wall/queue − open 통로 축 RMSE, 2D 대 3D 차의 차 (정답 지도 = 주)."""
    from amr_evaluation.slam3d import pstats
    idx = {(r['bag'], r['row'], r['replay']): r for r in recs}
    out = {'per_row': {}, 'did': {}}
    for row in LOC_ROWS:
        vals = {v: [_val(idx[(b, row, k)], 'aisle_axis_rmse_cm') for k in range(3)
                    if (b, row, k) in idx] for b, v in S6_VARIANT.items()}
        ent = {'values': vals}
        for v in ('queue', 'wall'):
            a, o = vals.get(v, []), vals.get('open', [])
            ent[f'{v}_minus_open'] = (float(np.nanmean(a) - np.nanmean(o))
                                      if a and o else math.nan)
            ent[f'{v}_rule'] = pstats.replay_rule(a, o, 0.2)
        for m2 in ('aisle_lateral_rmse_cm', 'aisle_axis_max_cm', 'ate_rmse_cm'):
            ent[m2] = {v: [_val(idx[(b, row, k)], m2) for k in range(3) if (b, row, k) in idx]
                       for b, v in S6_VARIANT.items()}
        out['per_row'][row] = ent
    for d2, d3 in (('amcl_gt', 'rtabmap3d_loc_gt'), ('amcl_slam', 'rtabmap3d_loc_slam')):
        for v in ('queue', 'wall'):
            x = out['per_row'].get(d2, {}).get(f'{v}_minus_open', math.nan)
            y = out['per_row'].get(d3, {}).get(f'{v}_minus_open', math.nan)
            out['did'][f'{d2}-vs-{d3}:{v}'] = x - y
    return out


def cmd_compare_loc(ns):
    """E3(b): 가족 L1·L2 (7 bag·사전 등록 6 bag), 구간 표, 실패, 지도 몫, E5(위치추정)."""
    recs = load_replays(ns.replays)
    res = {'tests': {'ALL7': loc_tests(recs, LOC_BAGS_ALL, 'ALL7'),
                     'PRE6': loc_tests(recs, LOC_BAGS_PRE, 'PRE6'),
                     'S1_3': loc_tests(recs, S1_ALL, 'S1_3'),
                     'S1_2': loc_tests(recs, S1_PRE, 'S1_2')},
           'rows': per_row_tables(recs, sorted({r['row'] for r in recs})),
           'rows_pre6': per_row_tables([r for r in recs if r['bag'] in LOC_BAGS_PRE],
                                       sorted({r['row'] for r in recs})),
           'segments': segment_table(recs), 'failures': loc_failures(recs),
           'map_effect': map_effect(recs), 'e5_loc': e5_loc(recs)}
    os.makedirs(ns.out, exist_ok=True)
    with open(os.path.join(ns.out, 'compare_loc.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=1, default=float)
    print(os.path.join(ns.out, 'compare_loc.json'))
    return 0


# ======================================================================== E4 절제
ABL_BASE = 'rtabmap3d_noimu'
ABL_BAGS = ('s3_aisle_a1', 's1_seed1_a1', 's1_seed3_a2', 's1_seed2x_a1')
#: 지표 → δ (기술 판정 replay_rule). 지연은 공유 부하 하 예비값
ABL_METRICS = {'ate_rmse_cm': 0.2, 'ate_max_cm': 0.2, 'rpe1_pct': 0.05,
               'map_adnn_surf_cm': 0.2, 'map_f1': 0.01, 'map_iou': 0.01, 'map_recall_surf': 0.01,
               'map_adnn_surf_cm_band': 0.2, 'map_recall_surf_band': 0.01, 'map_iou_band': 0.01,
               'cloud_f5': 0.01, 'lat_icp_p50_ms': 5.0, 'lat_icp_p95_ms': 5.0,
               'lat_rtabmap_p50_ms': 10.0}


def _abl_val(r, m):
    if m.startswith('lat_'):
        part, st = ('lat_icp', m[len('lat_icp_'):]) if m.startswith('lat_icp_') else \
            ('lat_rtabmap', m[len('lat_rtabmap_'):])
        v = (r.get(part) or {}).get(st)
        return float(v) if v is not None else math.nan
    return _val(r, m)


def cmd_compare_abl(ns):
    """E4: 절제 행마다 기준(E2 rtabmap3d_noimu, 같은 bag) 대비 bag 별 평균 차와 replay_rule 판정 (기술)."""
    from amr_evaluation.slam3d import pstats
    recs = load_replays(ns.replays)
    rows = sorted({r['row'] for r in recs if r['row'].startswith('rtabmap3d_abl_')})
    res = {'base': ABL_BASE, 'bags': list(ABL_BAGS), 'rows': {}}
    for row in rows:
        ent = {'n': sum(1 for r in recs if r['row'] == row),
               'fail': sum(1 for r in recs if r['row'] == row and r['fail']),
               'fail_reasons': sorted({w for r in recs if r['row'] == row for w in r['fail']}),
               'metrics': {}}
        for m, delta in ABL_METRICS.items():
            per = {}
            for bag in ABL_BAGS:
                a = [_abl_val(r, m) for r in recs if r['row'] == row and r['bag'] == bag]
                b = [_abl_val(r, m) for r in recs if r['row'] == ABL_BASE and r['bag'] == bag]
                a = [v for v in a if math.isfinite(v)]
                b = [v for v in b if math.isfinite(v)]
                if not a or not b:
                    continue
                per[bag] = {'abl': a, 'base': b, 'diff': float(np.mean(a) - np.mean(b)),
                            'rule': pstats.replay_rule(a, b, delta)}
            if per:
                md = float(np.mean([v['diff'] for v in per.values()]))
                ent['metrics'][m] = {'per_bag': per, 'delta': delta, 'mean_diff': md,
                                     'n_differs': sum(1 for v in per.values()
                                                      if v['rule'] == 'differs')}
        res['rows'][row] = ent
    os.makedirs(ns.out, exist_ok=True)
    with open(os.path.join(ns.out, 'compare_abl.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=1, default=float)
    print(os.path.join(ns.out, 'compare_abl.json'))
    return 0


# ======================================================================== E3(b) 2D 정답 지도
def perfect_map(occ01, res01=0.01, res=0.05):
    """
    "완전한 지도" (evaluation.md §2): 0.01 m GT 단면의 **면이 지나는** 0.05 m 셀만 점유, 고체 안쪽은 미지, 나머지 자유.

    반환 int16 [행, 열] (−1 미지, 0 자유, 100 점유) — OccupancyGrid 규약, 행 0 = 최하단. 0.01 경계 셀(8 이웃에 비점유)을
    품은 0.05 셀이 점유다 — 셀 중심이 면에서 ±2.5 cm 안에 고르게 놓여 평균 편향이 없다 (SLAM 지도의 얇은 띠와 같은 모양).
    채운 단면 래스터를 그대로 주면 배포 scan_matcher 의 "띠 중심선" 면 점이 고체 안쪽으로 밀린다 (파일럿 정지 오차 7 cm).
    """
    from amr_evaluation.slam3d import gridmap
    k = int(round(res / res01))
    rows, cols = occ01.shape
    r2, c2 = rows // k, cols // k
    occ = occ01[:r2 * k, :c2 * k]
    surf = gridmap.surface(occ)
    any_surf = surf.reshape(r2, k, c2, k).any(axis=(1, 3))
    any_occ = occ.reshape(r2, k, c2, k).any(axis=(1, 3))
    out = np.zeros((r2, c2), dtype=np.int16)
    out[any_occ] = -1
    out[any_surf] = 100
    return out


def cmd_perfect_map(ns):
    """GT 0.01 단면 → 0.05 m 완전한 지도 (map_server 형식) — E3(b) 2D 행의 정답 지도."""
    from amr_evaluation.slam3d import gridmap
    from amr_slam3d import grid as ogrid
    g = gridmap.read_map_yaml(os.path.join(GT_DIR, GT_GRID))
    data = perfect_map(g.occupied, g.resolution, ns.res)
    og = ogrid.OccGrid(data, ns.res, (g.origin_x, g.origin_y, 0.0))
    os.makedirs(ns.out, exist_ok=True)
    pgm, yml = ogrid.write_map(os.path.join(ns.out, f'perfect_r{ns.res:.3f}'), og,
                               comment=f'완전한 지도: {GT_DIR}/{GT_GRID} 의 면이 지나는 {ns.res} m 셀')
    print(json.dumps({'pgm': pgm, 'yaml': yml, 'occupied': int((data == 100).sum()),
                      'unknown': int((data == -1).sum()), 'free': int((data == 0).sum())}))
    return 0


# ======================================================================== E3(b) 지도 선택
SELECT = {'sel_slam_toolbox': ('slam_toolbox', ('map_world.yaml', 'map_world.pgm')),
          'sel_rtabmap3d': ('rtabmap3d_noimu', ('rtabmap.db',))}


def select_rule(cands):
    """
    사전 등록 규칙: 후보 [(재생 번호, ATE, 깨끗한가)] → (고른 번호, 사유).

    깨끗한 후보 3 → ATE 중앙값, 2 → 번호가 작은 것, 1 → 그것, 0 → 누락이 가장 적은 재생(같으면 번호 작은 것,
    호출자가 깨끗하지 않은 후보를 (번호, 누락 수) 순으로 정렬해 준다) — "실패 재생의 지도" 표시.
    """
    clean = sorted((k, a) for k, a, ok in cands if ok)
    if len(clean) >= 3:
        k = sorted(clean, key=lambda x: (x[1], x[0]))[len(clean) // 2][0]
        return k, 'median_ate'
    if clean:
        return clean[0][0], 'lowest_index_clean' if len(clean) == 2 else 'only_clean'
    if cands:
        return cands[0][0], 'failed_replay_map'
    return None, 'none'


def cmd_select_maps(ns):
    """E2 map_long 재생에서 E3(b) 지도를 고르고 R/maps/sel_*/ 로 복사, 정답 자세 DB 도 R/maps/gtdb/ 로."""
    import shutil
    from amr_evaluation.slam3d import se2
    gt2 = se2.tum_to_se2(os.path.join(ns.rec, 'map_long_a1', 'gt.tum'))
    out = {}
    for dst, (row, files) in SELECT.items():
        base = os.path.join(ns.root, 'e2', 'map_long_a1', row)
        cands, info = [], []
        for k in range(3):
            d = os.path.join(base, f'r{k}')
            done = _json(os.path.join(d, 'DONE'))
            if done is None:
                continue
            dr = drops_of(d, row)
            est = _read_tum3(os.path.join(d, 'traj_online.tum'))
            ate = math.nan
            if est is not None:
                a = se2.ate(gt2, se2.Traj2D(est[0], est[1][:, :2], se2.quat_to_rpy(est[2])[:, 2]),
                            'none')
                ate = 100 * a['trans']['rmse'] if a['n_used'] else math.nan
            have = all(os.path.isfile(os.path.join(d, f)) for f in files)
            bad = dr.get('missing') if row.startswith('rtabmap') else dr.get('msgfilter_drop')
            ok = done.get('rc') == 0 and have and bad == 0 and math.isfinite(ate)
            info.append({'replay': k, 'rc': done.get('rc'), 'ate_full_cm': ate, 'files': have,
                         'missing_or_msgfilter': bad, 'clean': ok})
            cands.append((k, ate, ok, bad if bad is not None else 10 ** 9))
        dirty = sorted([c for c in cands if not c[2]], key=lambda c: (c[3], c[0]))
        k, why = select_rule([c[:3] for c in cands if c[2]] + [c[:3] for c in dirty])
        out[dst] = {'row': row, 'candidates': info, 'chosen': k, 'rule': why}
        if k is not None and not ns.dry_run:
            os.makedirs(os.path.join(ns.maps, dst), exist_ok=True)
            for f in files:
                shutil.copyfile(os.path.join(base, f'r{k}', f), os.path.join(ns.maps, dst, f))
    g = os.path.join(ns.root, 'e3', 'map_long_a1', 'rtabmap3d_gtdb', 'r0')
    gd = _json(os.path.join(g, 'DONE'))
    out['gtdb'] = {'dir': g, 'rc': gd.get('rc') if gd else None}
    if gd and gd.get('rc') == 0 and os.path.isfile(os.path.join(g, 'rtabmap.db')) \
            and not ns.dry_run:
        os.makedirs(os.path.join(ns.maps, 'gtdb'), exist_ok=True)
        shutil.copyfile(os.path.join(g, 'rtabmap.db'), os.path.join(ns.maps, 'gtdb', 'rtabmap.db'))
    with open(os.path.join(ns.maps, 'selection.json' if not ns.dry_run else 'selection_dry.json'),
              'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False, indent=1, default=float)
    print(json.dumps(out, ensure_ascii=False, default=float))
    return 0


def main(argv=None):
    """명령줄."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('cache')
    s.add_argument('--rec', default=REC)
    s.add_argument('--bag', required=True)
    s.add_argument('--out', required=True)
    s = sub.add_parser('replays')
    s.add_argument('--rec', default=REC)
    s.add_argument('--root', required=True)
    s.add_argument('--stage', required=True)
    s.add_argument('--cache', required=True)
    s.add_argument('--out', required=True)
    s.add_argument('--bags', nargs='*')
    s.add_argument('--window-stage', default=None)
    s.add_argument('--window-rows', nargs='*', default=None)
    s.add_argument('--no-grid', action='store_true')
    s.add_argument('--warmup', type=float, default=0.0,
                   help='[s, sim] 창 시작에서 더 뺄 시간 (E3(b) 위치추정 2.0)')
    s.add_argument('--grid-band', action='store_true',
                   help='대역 0.05–2.0 m GT 로도 지도 지표 (_band) — E4 높이 대역 행')
    s.add_argument('--no-cloud', action='store_true')
    s = sub.add_parser('compare')
    s.add_argument('--replays', nargs='+', required=True)
    s.add_argument('--out', required=True)
    s = sub.add_parser('perfect-map')
    s.add_argument('--out', required=True)
    s.add_argument('--res', type=float, default=0.05)
    s = sub.add_parser('select-maps')
    s.add_argument('--rec', default=REC)
    s.add_argument('--root', required=True)
    s.add_argument('--maps', required=True)
    s.add_argument('--dry-run', action='store_true')
    s = sub.add_parser('compare-loc')
    s.add_argument('--replays', nargs='+', required=True)
    s.add_argument('--out', required=True)
    s = sub.add_parser('compare-abl')
    s.add_argument('--replays', nargs='+', required=True)
    s.add_argument('--out', required=True)
    ns = ap.parse_args(argv)
    return {'cache': cmd_cache, 'replays': cmd_replays, 'compare': cmd_compare,
            'compare-loc': cmd_compare_loc, 'compare-abl': cmd_compare_abl,
            'select-maps': cmd_select_maps, 'perfect-map': cmd_perfect_map}[ns.cmd](ns)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
