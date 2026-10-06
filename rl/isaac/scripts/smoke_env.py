"""
Isaac env 점검: 생성·reset·step, LiDAR 대조, 무작위·경로추종 정책 처리량 (스스로 종료).

    rl/isaac/run_isaac.sh smoke rl/isaac/scripts/smoke_env.py --num_envs 256 --steps 300

검사 (실패하면 SMOKE_FAIL 과 함께 rc 1):
  1. reset 관측 모양 [N, 291], 유한
  2. RayCaster 센서 원점 = 계약 자세로 계산한 LiDAR 원점 (리셋 직후·스텝 뒤)
  3. RayCaster 거리 = 창고 상자에 대한 해석적 2D 레이캐스트 (잡음·드롭아웃 끈 상태, 무작위 env 8 개)
  4. 무작위 행동 step: 관측·보상 유한, 처리량 [측정]
  5. 경로추종(순수 추종) 휴리스틱으로 성공이 실제로 나오는가 (env 가 풀 수 있는 과제인지) [측정]
결과 JSON 을 --out 에 쓴다.
"""

import argparse
import json
import math
import os
import sys
import time

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument('--num_envs', type=int, default=256)
ap.add_argument('--steps', type=int, default=300)
ap.add_argument('--heur_steps', type=int, default=1500)
ap.add_argument('--lidar_source', default='sensor')
ap.add_argument('--out', default=os.path.join(os.environ.get(
    'AMR_ISAAC_RUNS', '/home/CAPYI_AI/runs/r7-isaac12'), 'smoke', 'smoke.json'))
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
args.headless = True
app = AppLauncher(args).app

sys.path.insert(0, '/workspace/amr/rl/isaac')
import numpy as np  # noqa: E402
import torch  # noqa: E402

from amr_isaac import contract_torch as CT  # noqa: E402
from amr_isaac import warehouse_geom as W  # noqa: E402
from amr_isaac.env import AmrNavEnv  # noqa: E402
from amr_isaac.env_cfg import AmrNavEnvCfg  # noqa: E402

FAIL = []


def check(cond, msg):
    """조건이 거짓이면 실패 목록에 넣는다."""
    print(('OK   ' if cond else 'FAIL ') + msg, flush=True)
    if not cond:
        FAIL.append(msg)


def ray_box_2d(o, d, boxes):
    """2D 레이 o + t·d 와 상자들(스캔 평면 z=0.20 을 지나는 것)의 최소 t (없으면 inf)."""
    best = math.inf
    for b in boxes:
        cx, cy = b.center[0], b.center[1]
        yaw = math.atan2(b.rot[1, 0], b.rot[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy = o[0] - cx, o[1] - cy
        lo = (c * ox + s * oy, -s * ox + c * oy)
        ld = (c * d[0] + s * d[1], -s * d[0] + c * d[1])
        h = (0.5 * b.size[0], 0.5 * b.size[1])
        t0, t1 = -math.inf, math.inf
        ok = True
        for k in range(2):
            if abs(ld[k]) < 1e-12:
                if abs(lo[k]) > h[k]:
                    ok = False
                    break
            else:
                a = (-h[k] - lo[k]) / ld[k]
                bb = (h[k] - lo[k]) / ld[k]
                t0, t1 = max(t0, min(a, bb)), min(t1, max(a, bb))
        if ok and t0 <= t1 and t1 >= 0:
            best = min(best, t0 if t0 >= 0 else 0.0)
    return best


def heuristic(env):
    """경로 순수 추종 (1.0 m 앞 경로점) → raw 행동 (tanh 역)."""
    st = env.contract_state()
    s0 = CT.project_onto_path(st, env.pose[:, :2])[0]
    tgt = CT.interpolate_at(st, s0 + 1.0)
    q = CT.to_robot_frame(env.pose, tgt.unsqueeze(1)).squeeze(1)
    b = torch.atan2(q[:, 1], q[:, 0])
    gd = torch.hypot(*(CT.goal_point(st) - env.pose[:, :2]).unbind(1))
    v_t = torch.clamp(0.7 * torch.cos(b), min=0.0)
    v_t = torch.minimum(v_t, 0.6 * gd)
    w_t = torch.clamp(1.5 * b, -1.0, 1.0)
    a_v = ((v_t - env.v_cmd) / 0.1).clamp(-1.0, 1.0)
    a_w = ((w_t - env.w_cmd) / 0.1).clamp(-2.0, 2.0)
    u = torch.stack([a_v / 1.0, a_w / 2.0], 1).clamp(-0.999, 0.999)
    return torch.atanh(u).to(torch.float32)


def main():
    """점검 실행."""
    res = {'num_envs': args.num_envs}
    cfg = AmrNavEnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.lidar_source = args.lidar_source
    cfg.dr.enable = False
    cfg.dr.nominal_lidar_sigma_m = 0.0
    t0 = time.time()
    env = AmrNavEnv(cfg)
    res['create_s'] = round(time.time() - t0, 1)
    obs, _ = env.reset()
    o = obs['policy']
    check(tuple(o.shape) == (args.num_envs, 291), f'reset 관측 모양 {tuple(o.shape)}')
    check(bool(torch.isfinite(o).all()), 'reset 관측 유한')
    res['lidar_pose_err_reset'] = float(env.lidar_pose_err)
    check(res['lidar_pose_err_reset'] < 1e-3,
          f'리셋 직후 센서 원점 오차 {res["lidar_pose_err_reset"]:.2e} m')

    # 3. 해석적 레이캐스트 대조
    boxes = [b for b in W.load_boxes() if b.z_range()[0] <= 0.20 <= b.z_range()[1]]
    ids = torch.randperm(args.num_envs)[:8].tolist()
    errs = []
    bad = []
    ang = CT.beam_angles().numpy()
    for i in ids:
        org = env.sensor_origin()[i].cpu().numpy()
        th = float(env.pose[i, 2])
        for k in range(0, 720, 3):
            a = th + ang[k]
            t = ray_box_2d(org, (math.cos(a), math.sin(a)), boxes)
            exp = t if t <= 25.0 else math.nan
            got = float(env.scan[i, k])
            if math.isnan(exp) and math.isnan(got):
                continue
            e = abs(exp - got) if not (math.isnan(exp) or math.isnan(got)) else 99.0
            errs.append(e)
            if e > 2e-3 and len(bad) < 12:
                bad.append({'env': i, 'beam': k, 'exp': exp, 'got': got,
                            'origin': [float(org[0]), float(org[1])], 'yaw': th})
    errs = np.asarray(errs)
    res['lidar_analytic_max_err'] = float(errs.max())
    res['lidar_analytic_p99_err'] = float(np.quantile(errs, 0.99))
    res['lidar_analytic_n'] = int(errs.size)
    res['lidar_analytic_bad'] = bad
    check(res['lidar_analytic_p99_err'] < 2e-3,
          f'LiDAR vs 해석 레이캐스트 p99 {res["lidar_analytic_p99_err"]:.2e} m, '
          f'max {res["lidar_analytic_max_err"]:.2e} m (n={errs.size})')

    # 4. 무작위 행동
    torch.cuda.synchronize()
    t0 = time.time()
    resets = 0
    finite = True
    for _ in range(args.steps):
        a = torch.randn(args.num_envs, 2, device=env.device)
        obs, rew, term, trunc, _ = env.step(a)
        resets += int((term | trunc).sum())
        finite &= bool(torch.isfinite(obs['policy']).all()) and bool(torch.isfinite(rew).all())
    torch.cuda.synchronize()
    dt = time.time() - t0
    res['random_steps_per_s'] = round(args.steps * args.num_envs / dt, 1)
    res['random_resets'] = resets
    res['random_stats'] = env.pop_stats()
    res['lidar_pose_err_step'] = float(env.lidar_pose_err)
    check(finite, '무작위 step 관측·보상 유한')
    check(res['lidar_pose_err_step'] < 1e-3,
          f'스텝 뒤 센서 원점 오차 {res["lidar_pose_err_step"]:.2e} m')

    # 5. 휴리스틱 경로추종
    env.reset()
    env.pop_stats()
    t0 = time.time()
    rsum = torch.zeros(args.num_envs, device=env.device)
    for _ in range(args.heur_steps):
        obs, rew, term, trunc, _ = env.step(heuristic(env))
        rsum += rew
    torch.cuda.synchronize()
    res['heuristic_steps_per_s'] = round(args.heur_steps * args.num_envs / (time.time() - t0), 1)
    res['heuristic_stats'] = env.pop_stats()
    hs = res['heuristic_stats']
    check(hs['success'] > 0, f'휴리스틱 성공 {hs["success"]}/{hs["episodes"]}')
    res['gpu_max_alloc_MiB'] = round(torch.cuda.max_memory_allocated() / 2**20, 1)
    res['fail'] = FAIL
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w') as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print('SMOKE_RESULT ' + json.dumps(res, ensure_ascii=False), flush=True)


try:
    main()
except Exception:
    import traceback
    traceback.print_exc()
    FAIL.append('exception')
print('SMOKE_FAIL' if FAIL else 'SMOKE_PASS', flush=True)
# app.close() 가 멈춘 적이 있다 (verify_isaac 2026-10-06) — 검사가 끝났으므로 바로 끝낸다
os._exit(1 if FAIL else 0)
