"""
지도 변형 env 스모크 (정책 없음) — 레이캐스트·충돌·정본 φ 가 env 별 타일(변형 지도)을 쓰는지 Isaac 안에서 확인한다.

    rl/isaac/run_isaac.sh mgsmoke rl/isaac/scripts/smoke_mapgen.py --bundle <npz> --out <dir>

검사 (모두 통과하면 rc 0):
  R1 레이캐스트: env 의 실제 Isaac 스캔(잡음 전, env._read_scan) = 그 env 타일 사각형에 대한
     CPU 2D 광선 (mapgen.raycast_2d)
     — 유한 빔 최대 차 ≤ 2e-3 m, 유·무한 일치. 변형 타일에서는 명목 지도 광선과 > 0.05 m 다른 빔이 있어야 한다
     (타일 0·먼 사본은 0). LiDAR 원점이 상자 안인 env (R2 용 자세)는 뺀다.
  R2 충돌: 새 상자 중심·옮겨진 랙의 원래 자리에 풋프린트를 두고 한 결정 스텝 (행동 0) → env 의 충돌 판정이 그 env 타일의
     사각형을 따른다 (변형: 새 상자 = 충돌, 옛 랙 자리 = 비충돌 / 명목: 반대). 패치한 충돌 함수가 서브스텝마다 불렸는지도.
  R3 정본 φ: 앞 0.4–0.9 m 에 새 상자가 있는 자세에서 전진 명령 → φ 의 접근 거리 D 가 변형 지도의 직진 여유와 맞고
     (|D − 기하| ≤ 0.10 m), 같은 국소 자세의 명목 지도에서는 D 가 > 2 m 다.
  R4 관측 불변: 같은 국소 상태의 관측이 타일 이동과 무관 (최대 차 ≤ 1e-5).
"""

import argparse
import json
import math
import os
import sys

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument('--bundle', required=True)
ap.add_argument('--out', required=True)
ap.add_argument('--seed', type=int, default=4242)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

sys.path.insert(0, '/workspace/amr/rl/isaac')
sys.path.insert(0, '/workspace/amr/rl/eval')
import numpy as np  # noqa: E402
import torch  # noqa: E402

from amr_isaac import contract_torch as CT  # noqa: E402
from amr_isaac import eval_sets as ES  # noqa: E402
from amr_isaac import map_bundle as MB  # noqa: E402
from amr_isaac import mapgen as MG  # noqa: E402
from amr_isaac import warehouse_geom as W  # noqa: E402
from amr_isaac.env_cfg import AmrNavEvalEnvCfg  # noqa: E402
from amr_isaac.env_mapgen import AmrNavMapEnv  # noqa: E402
from amr_rl_contract import constants as C  # noqa: E402

PER_TILE = 8


def footprint_free(pose, obb):
    """풋프린트가 사각형들과 겹치지 않는가 (numpy 기준판)."""
    return not W.footprint_hits_obb(pose, obb, C.HALF_L, C.HALF_W)


def facing_pose(obb_var, obb_nom, box, rng):
    """새 상자 box 앞 0.4–0.9 m 에서 상자를 보는 자세 (변형만 막고 명목은 > 2 m 열림) 또는 None."""
    for _ in range(400):
        th = rng.uniform(-math.pi, math.pi)
        for dist in np.arange(0.9, 2.5, 0.05):
            p = np.array([box[0] - dist * math.cos(th), box[1] - dist * math.sin(th), th])
            if not (footprint_free(p, obb_var) and footprint_free(p, obb_nom)):
                continue
            dv = float(ES.straight_approach_batch(p[None], obb_var)[0])
            dn = float(ES.straight_approach_batch(p[None], obb_nom)[0])
            if 0.4 <= dv <= 0.9 and dn > 2.5:
                return p, dv, dn
    return None


def main():
    """스모크."""
    os.makedirs(args.out, exist_ok=True)
    bundle = MB.load_bundle(args.bundle)
    names = bundle['cond_names'].tolist()
    t_add = int(np.flatnonzero((bundle['tile_cond'] == names.index('add')))[0])
    t_jit = int(np.flatnonzero((bundle['tile_cond'] == names.index('jitter_0p60')))[0])
    t_rem = int(np.flatnonzero((bundle['tile_cond'] == names.index('remove')))[0])
    far = int(bundle['far_tile'])
    tiles = [0, t_add, t_jit, t_rem, far]
    offs = bundle['tile_offset']
    # 경로: 각 타일마다 그 타일의 유효 행 8 개 (명목·먼 사본은 add 타일 행의 명목 경로)
    cfg = AmrNavEvalEnvCfg()
    cfg.safety_filter = 'canonical'
    cfg.seed = args.seed
    from amr_isaac.assets import load_pool
    pool = load_pool(seed=1, out_dir=cfg.assets_dir, log=None)
    extra, env_tile = [], []
    add_rows = np.flatnonzero((bundle['row_tile'] == t_add) & (bundle['row_code'] == 0))[:PER_TILE]
    for t in tiles:
        if bundle['tile_cond'][t] < 0:
            for r in add_rows:
                pid = int(bundle['row_pid'][r])
                o = pool['offs']
                extra.append((t, pool['pts'][o[pid]:o[pid + 1]]))
                env_tile.append(t)
        else:
            rr = np.flatnonzero((bundle['row_tile'] == t) & (bundle['row_code'] == 0))[:PER_TILE]
            for r in rr:
                extra.append((t, MB.row_path(bundle, int(r))))
                env_tile.append(t)
    N = len(extra)
    cfg.scene.num_envs = N
    base = int(pool['length'].shape[0])
    env = AmrNavMapEnv(cfg, bundle, extra)
    dev = env.device
    env.forced_path_ids = torch.arange(base, base + N, device=dev)
    env.set_env_tiles(env_tile)
    with torch.inference_mode():
        env.reset(seed=args.seed)
    rep = {'tiles': tiles, 'N': N, 'mesh_triangles_per_tile': env.mesh_counts, 'checks': {}}
    world = [MG.shift_obb(MB.tile_rects(bundle, t)[0], offs[t]) for t in range(offs.shape[0])]
    zrs = [MB.tile_rects(bundle, t)[1] for t in range(offs.shape[0])]
    nom_local = MB.tile_rects(bundle, 0)[0]
    rng = np.random.default_rng(0)

    # ---------------- 시험 자세 정하기 (국소 좌표) ----------------
    add_obb, _zr, add_gid = MB.tile_rects(bundle, t_add)
    boxes = add_obb[add_gid == MG.ADDED]
    faces = []
    for b in boxes:
        fp = facing_pose(add_obb, nom_local, b, rng)
        if fp is not None:
            faces.append(fp)
        if len(faces) >= 3:
            break
    jit_obb, _z, jit_gid = MB.tile_rects(bundle, t_jit)
    nom_gid = MB.tile_rects(bundle, 0)[2]
    moved = None
    for g in np.unique(nom_gid[nom_gid >= 0]):
        c_nom = nom_local[(nom_gid == g)][np.argmax(nom_local[nom_gid == g][:, 2] *
                                                    nom_local[nom_gid == g][:, 3])]
        p = np.array([c_nom[0], c_nom[1], c_nom[4]])
        if footprint_free(p, jit_obb) and not footprint_free(p, nom_local):
            moved = p
            break
    rep['test_poses'] = {'faces': [[list(map(float, f[0])), f[1], f[2]] for f in faces],
                         'moved_rack_pose': None if moved is None else list(map(float, moved))}
    idx = {t: [i for i, x in enumerate(env_tile) if x == t] for t in tiles}

    def set_poses(assign):
        """{env: 국소 자세} → 세계 자세로 env 에 쓰고 센서를 갱신."""
        pose = env.pose.clone()
        for i, p in assign.items():
            t = env_tile[i]
            pose[i] = torch.tensor([p[0] + offs[t, 0], p[1] + offs[t, 1], p[2]], dtype=pose.dtype,
                                   device=dev)
        env.pose.copy_(pose)
        env._write_root_pose(env.robot._ALL_INDICES)
        env.sim.step(render=False)
        env.scene.update(dt=env.physics_dt)

    with torch.inference_mode():
        # ---------------- R1 레이캐스트 ----------------
        assign = {}
        if faces:
            for t in (t_add, 0):
                for k, f in enumerate(faces):
                    assign[idx[t][k]] = f[0]
        if moved is not None:
            assign[idx[t_jit][0]] = moved
            assign[idx[0][PER_TILE - 1]] = moved
        set_poses(assign)
        r = env._read_scan().cpu().numpy()
        pose = env.pose.cpu().numpy()
        org = env.sensor_origin().cpu().numpy()
        beam = CT.beam_angles('cpu', torch.float64).numpy()
        r1 = {'lidar_pose_err': float(env.lidar_pose_err), 'per_tile': {}}
        ok1 = float(env.lidar_pose_err) < 1e-3
        inside = []
        for t in tiles:
            mx, mism, diff_nom = 0.0, 0, 0
            for i in idx[t]:
                # LiDAR 원점이 상자 안이면 (R2 용 자세) Isaac 은 안쪽 면, CPU 광선은 0 을 낸다 — 대조에서 뺀다
                if MG.rect_point_distance(world[t], org[i]).min() == 0.0 or \
                        MG.rect_point_distance(nom_local, org[i] - offs[t]).min() == 0.0:
                    inside.append(int(i))
                    continue
                ang = pose[i, 2] + beam
                ref = MG.raycast_2d(org[i], ang, world[t], zrs[t], C.LIDAR_RANGE_MAX)
                nom = MG.raycast_2d(org[i] - offs[t], ang, nom_local, zrs[0], C.LIDAR_RANGE_MAX)
                fin = np.isfinite(r[i]) & np.isfinite(ref)
                mism += int((np.isfinite(r[i]) != np.isfinite(ref)).sum())
                if fin.any():
                    mx = max(mx, float(np.abs(r[i][fin] - ref[fin]).max()))
                both = np.isfinite(r[i]) & np.isfinite(nom)
                diff_nom += int((np.abs(r[i][both] - nom[both]) > 0.05).sum() +
                                (np.isfinite(r[i]) != np.isfinite(nom)).sum())
            r1['per_tile'][str(t)] = {'max_abs_diff_m': mx, 'finite_mismatch_beams': mism,
                                      'beams_differing_from_nominal': diff_nom}
            ok1 &= mx <= 2e-3 and mism <= 2
            if bundle['tile_cond'][t] < 0:
                ok1 &= diff_nom <= 2
            else:
                ok1 &= diff_nom > 0
        r1['skipped_origin_inside_box'] = inside
        r1['pass'] = bool(ok1)
        rep['checks']['R1_raycast'] = r1

        # ---------------- R3 정본 φ (스캔 → 게이트) ----------------
        r3 = {'cases': []}
        ok3 = bool(faces)
        if faces:
            ph = env.phi
            ph.reset(env.robot._ALL_INDICES)
            scan = torch.as_tensor(r, dtype=torch.float64, device=dev)
            v = torch.full((N,), 0.5, dtype=torch.float64, device=dev)
            w = torch.zeros(N, dtype=torch.float64, device=dev)
            t0 = 1000.0
            ph.set_scan(scan, t0)
            out = None
            for k in range(5):
                out = ph.step(v, w, v, w, t0 + 0.02 * k)
            D = out['min_distance'].cpu().numpy()
            for k, f in enumerate(faces):
                dv, dn = D[idx[t_add][k]], D[idx[0][k]]
                r3['cases'].append({'pose_local': list(map(float, f[0])), 'geom_variant': f[1],
                                    'geom_nominal': f[2], 'phi_D_variant_tile': float(dv),
                                    'phi_D_nominal_tile': float(dn)})
                ok3 &= abs(dv - f[1]) <= 0.10 and (not math.isfinite(dn) or dn > 2.0)
        r3['pass'] = bool(ok3)
        rep['checks']['R3_phi'] = r3

        # ---------------- R2 충돌 (한 결정 스텝, 행동 0) ----------------
        assign = {}
        if len(boxes):
            b = boxes[0]
            p_box = np.array([b[0], b[1], 0.0])
            assign[idx[t_add][0]] = p_box
            assign[idx[0][0]] = p_box
        if moved is not None:
            assign[idx[t_jit][1]] = moved
            assign[idx[0][1]] = moved
        set_poses(assign)
        env.v_cmd.zero_()
        env.w_cmd.zero_()
        calls0 = env.collide_calls
        env.step(torch.zeros(N, 2, device=dev))
        col = env.step_diag['collision'].cpu().numpy()
        exp = {}
        if len(boxes):
            exp[idx[t_add][0]] = True
            exp[idx[0][0]] = not footprint_free(p_box, nom_local)
        if moved is not None:
            exp[idx[t_jit][1]] = False
            exp[idx[0][1]] = True
        r2 = {'collide_calls_per_step': env.collide_calls - calls0,
              'cases': [{'env': int(i), 'tile': int(env_tile[i]), 'expected': bool(e),
                         'got': bool(col[i])} for i, e in exp.items()]}
        r2['pass'] = bool(exp) and all(c['expected'] == c['got'] for c in r2['cases']) and \
            r2['collide_calls_per_step'] == 5
        rep['checks']['R2_collision'] = r2

        # ---------------- R4 관측 불변 (같은 국소 상태, 다른 타일) ----------------
        st = env.contract_state()
        i0, i1 = idx[0][2], idx[far][2]
        sub = CT.BatchState(**{k: getattr(st, k)[[i0, i0]].clone()
                               for k in st.__dataclass_fields__})
        sub.pose[1, 0] += float(offs[far, 0])
        sub.pose[1, 1] += float(offs[far, 1])
        sub.path_pts[1, :, 0] += float(offs[far, 0])
        sub.path_pts[1, :, 1] += float(offs[far, 1])
        o = CT.build_observation(sub)
        r4 = {'max_abs_obs_diff': float((o[0] - o[1]).abs().max()), 'envs': [i0, i1]}
        r4['pass'] = r4['max_abs_obs_diff'] <= 1e-5
        rep['checks']['R4_obs_invariance'] = r4
    rep['pass'] = all(c['pass'] for c in rep['checks'].values())
    with open(os.path.join(args.out, 'smoke.json'), 'w') as f:
        json.dump(rep, f, indent=1, ensure_ascii=False)
    print('SMOKE_MAPGEN ' + json.dumps({k: v['pass'] for k, v in rep['checks'].items()}),
          flush=True)
    return 0 if rep['pass'] else 1


try:
    rc = main()
except Exception:
    import traceback
    traceback.print_exc()
    rc = 2
os._exit(rc)
