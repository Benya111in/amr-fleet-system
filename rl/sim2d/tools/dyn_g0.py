#!/usr/bin/env python3
"""
2b G0 — 동적 장애물 진단 (학습 없음). docs/research/rl-navigation/dyn_obstacles.md §2–§3 의 원자료를 만든다.

    python3 rl/sim2d/tools/dyn_g0.py --out-dir /runs/g0 --code-commit $(git rev-parse HEAD)

(a)·(b) unavoid: 평가와 같은 분포(mixed: 무작위 0–4 + 20 % Gazebo 7 개, DR·φ 켬, 경로 3–15 m, 평가 seed 4242,
    env 2 048 = 첫 에피소드 2 048 개)에서 시작점 거르기 설정마다 탈출 정책 묶음(dyn_diag.ESCAPE)의 첫 에피소드 결말.
    서 있는 로봇(idle)이 맞는 비율·시각, 추종기(follow_0.8)의 결말, 묶음의 모든 정책이 맞는 비율(피할 수 없음의 상한 추정).
(c) signal: truncate=False 로 첫 종단까지의 할인 보상을 항별로 (정확한 무한 지평 가치) — 대기·회전·추종·PPO 초기 꼴
    확률 정책(μ_v, μ_ω 를 옮긴 것)을 정적·동적에서. μ 유한 차분으로 학습 초기 기울기 방향과 그것을 뒤집는 C 를 계산한다.
산출: <out-dir>/g0.json, g0.md, unavoid_<설정>.pt, signal.pt
"""

import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import torch  # noqa: E402

from amr_sim2d import dyn_diag as D  # noqa: E402
from amr_sim2d import scripted as SP  # noqa: E402
from amr_sim2d.env import EnvConfig, Sim2DEnv  # noqa: E402


def make_env(a, mode, window=0.0, radius=1.0, truncate=True, n=None):
    """평가 분포의 env (seed = 평가 seed — 경로 풀도 평가와 같다)."""
    return Sim2DEnv(EnvConfig(
        num_envs=n or a.envs, seed=a.seed, arm='B', obstacle_mode=mode,
        n_obstacles=tuple(a.n_obstacles), path_len=tuple(a.path_len), dr=True, safety=True,
        truncate=truncate, start_clear_window=window, start_clear_radius=radius,
        compile=not a.no_compile), log=lambda *x: None)


def parse_filters(items):
    """'이름=창:반경' 목록 → [(이름, 창, 반경)]."""
    out = []
    for it in items:
        name, wr = it.split('=')
        w, r = wr.split(':')
        out.append((name, float(w), float(r)))
    return out


def unavoid(a, out):
    """(a)·(b): 거르기 설정 × 탈출 정책 첫 에피소드."""
    res = {}
    for name, w, r in parse_filters(a.filters):
        t0 = time.time()
        env = make_env(a, 'mixed', w, r)
        env.clear_stats = {k: 0 for k in env.clear_stats}
        env.gen.manual_seed(a.seed)
        env.reset()
        cs = dict(env.clear_stats)
        n_obst0 = env.ep_nobs0.double()
        recs = {}
        for pn, pol in D.ESCAPE.items():
            recs[pn] = D.first_episodes(env, pol, a.seed)
        torch.save({k: {kk: (vv.cpu() if torch.is_tensor(vv) else vv) for kk, vv in v.items()}
                    for k, v in recs.items()}, os.path.join(out, f'unavoid_{name}.pt'))
        coll = torch.stack([recs[p]['outcome'] == 1 for p in recs])           # [P,N]
        tcol = torch.stack([torch.where(recs[p]['outcome'] == 1, recs[p]['time'],
                                        torch.full_like(recs[p]['time'], float('inf')))
                            for p in recs])
        idle = recs['idle']
        lvl = idle['level']
        gz = lvl == env.n_levels - 1
        it = torch.where(idle['outcome'] == 1, idle['time'], torch.full_like(idle['time'], 1e9))
        s = {'filter': {'window': w, 'radius': r}, 'clear_stats_first_reset': cs,
             'n_obst_mean': float(n_obst0.mean()),
             'n_obst_mean_random': float(n_obst0[~gz].mean()),
             'n_obst_mean_gazebo': float(n_obst0[gz].mean()) if bool(gz.any()) else None,
             'frac_gazebo': float(gz.double().mean()),
             'policies': {p: D.summarize_first(v) for p, v in recs.items()},
             'idle_hit_by_time': {str(T): float((it <= T).double().mean())
                                  for T in (1, 2, 3, 5, 10, 20)},
             'idle_hit_gazebo': float((idle['outcome'][gz] == 1).double().mean()),
             'idle_hit_random': float((idle['outcome'][~gz] == 1).double().mean()),
             'idle_hit_by_level': {str(k): float((idle['outcome'][lvl == k] == 1).double().mean())
                                   for k in torch.unique(lvl).tolist()},
             'all_escape_collide': float(coll.all(0).double().mean()),
             'all_escape_collide_by_time': {
                 str(T): float((tcol <= T).all(0).double().mean()) for T in (2, 3, 5)},
             'all_escape_collide_gazebo': float(coll.all(0)[gz].double().mean()),
             'all_escape_collide_random': float(coll.all(0)[~gz].double().mean()),
             'any_escape_success': float(torch.stack(
                 [recs[p]['outcome'] == 0 for p in recs]).any(0).double().mean()),
             'wall_s': round(time.time() - t0, 1)}
        res[name] = s
        print(f"[unavoid {name}] idle 충돌 {s['policies']['idle']['collision']:.4f} "
              f"(≤3 s {s['idle_hit_by_time']['3']:.4f}) follow_0.8 성공 "
              f"{s['policies']['follow_0.8']['success']:.3f} 충돌 "
              f"{s['policies']['follow_0.8']['collision']:.3f} | 모든 탈출 정책 충돌 "
              f"{s['all_escape_collide']:.4f} (≤3 s {s['all_escape_collide_by_time']['3']:.4f}) "
              f"| 활성 장애물 {s['n_obst_mean']:.2f} | {cs} | {s['wall_s']} s", flush=True)
        del env
        torch.cuda.empty_cache()
    return res


SIGNAL_POLICIES = {
    'idle': SP.idle, 'spin': SP.spin, 'follow_0.8': D.follow_v(0.8),
    'follow_0.3': D.follow_v(0.3),
    'g(-0.3,0)': D.gauss(-0.3, 0.0), 'g(0,0)': D.gauss(0.0, 0.0), 'g(+0.3,0)': D.gauss(0.3, 0.0),
    'g(0,+0.5)': D.gauss(0.0, 0.5),
}


def signal(a, out):
    """(c): 항별 무한 지평 가치와 μ 유한 차분."""
    w, r = parse_filters([a.signal_filter])[0][1:]
    envs = {'static': ('none', 0.0, 1.0), 'dyn': ('mixed', 0.0, 1.0), 'dyn_f': ('mixed', w, r)}
    res, raw = {}, {}
    C0 = None
    for en, (mode, ww, rr) in envs.items():
        t0 = time.time()
        env = make_env(a, mode, ww, rr, truncate=False, n=a.signal_envs)
        C0 = env.spec.collision_penalty
        raw[en] = {}
        res[en] = {'policies': {}}
        for pn, pol in SIGNAL_POLICIES.items():
            v = D.continuing_values(env, pol, a.seed, steps=a.signal_steps)
            raw[en][pn] = {k: x.cpu() for k, x in v.items()}
            s = {k: float(v[k].mean()) for k in D.TERMS + ('V',)}
            s.update({'coll_rate': float(v['coll'].double().mean()),
                      'succ_rate': float(v['succ'].double().mean()),
                      'open': float(v['open'].double().mean()),
                      'disc_coll': float((v['disc_end'] * v['coll'].double()).mean()),
                      'interv': float(v['interv'].mean()), 'spin': float(v['spin'].mean()),
                      'V_sd': float(v['V'].std())})
            # 수익 분산 중 충돌 항의 몫 Cov(G_coll, G)/Var(G)
            vc, vv = v['collision'], v['V']
            var = float(vv.var())
            s['coll_var_share'] = (float(((vc - vc.mean()) * (vv - vv.mean())).mean()) / var
                                   if var > 1e-12 else None)
            res[en]['policies'][pn] = s
        P = res[en]['policies']
        fd = {}
        for k in D.TERMS + ('V', 'disc_coll'):
            fd[k] = (P['g(+0.3,0)'][k] - P['g(-0.3,0)'][k]) / 0.6
        dD = fd['disc_coll']
        # V(C) = V + (C0 − C)·E[γ^T 1coll] → dV/dμ_v 가 0 이 되는 C
        fd['C_flip'] = (C0 + fd['V'] / dD) if abs(dD) > 1e-9 else None
        res[en]['dV_dmu_v'] = fd
        res[en]['dV_dmu_w'] = {k: (P['g(0,+0.5)'][k] - P['g(0,0)'][k]) / 0.5
                               for k in D.TERMS + ('V', 'disc_coll')}
        sp, idl = raw[en]['spin']['V'], raw[en]['idle']['V']
        res[en]['spin_minus_idle'] = {'mean': float((sp - idl).mean()),
                                      'frac_pos': float(((sp - idl) > 1e-9).double().mean())}
        fo = raw[en]['follow_0.8']['V']
        res[en]['follow_minus_idle'] = {'mean': float((fo - idl).mean()),
                                        'frac_pos': float(((fo - idl) > 1e-9).double().mean())}
        res[en]['wall_s'] = round(time.time() - t0, 1)
        print(f"[signal {en}] dV/dμ_v {fd['V']:+.3f} (충돌 항 {fd['collision']:+.3f}, 진행 "
              f"{fd['progress']:+.3f}, 목표 {fd['goal']:+.3f}) C_flip {fd['C_flip']} | dV/dμ_ω "
              f"{res[en]['dV_dmu_w']['V']:+.3f} | spin−idle "
              f"{res[en]['spin_minus_idle']['mean']:+.4f} | follow−idle "
              f"{res[en]['follow_minus_idle']['mean']:+.3f} | "
              f"{res[en]['wall_s']} s", flush=True)
        del env
        torch.cuda.empty_cache()
    torch.save(raw, os.path.join(out, 'signal.pt'))
    res['C0'] = C0
    return res


def render(res):
    """요약 → 마크다운."""
    L = []
    if 'unavoid' in res:
        L.append('## (a)·(b) 피할 수 없는 에피소드 — 첫 에피소드, 평가 분포\n')
        L.append('| 거르기 | 활성 장애물 | idle 충돌 | idle ≤1 s | ≤3 s | ≤5 s | idle Gazebo | idle 무작위 | '
                 'follow_0.8 성공 | follow_0.8 충돌 | 모든 탈출 충돌 | ≤3 s | ≤5 s | 하나라도 성공 |')
        L.append('|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|')
        for name, s in res['unavoid'].items():
            ih, ps = s['idle_hit_by_time'], s['policies']
            L.append(f"| {name} ({s['filter']['window']} s, {s['filter']['radius']} m) | "
                     f"{s['n_obst_mean']:.2f} | {ps['idle']['collision']:.4f} | {ih['1']:.4f} | "
                     f"{ih['3']:.4f} | {ih['5']:.4f} | {s['idle_hit_gazebo']:.4f} | "
                     f"{s['idle_hit_random']:.4f} | {ps['follow_0.8']['success']:.3f} | "
                     f"{ps['follow_0.8']['collision']:.3f} | {s['all_escape_collide']:.4f} | "
                     f"{s['all_escape_collide_by_time']['3']:.4f} | "
                     f"{s['all_escape_collide_by_time']['5']:.4f} | "
                     f"{s['any_escape_success']:.3f} |")
        L.append('\n| 거르기 | 정책 | 성공 | 충돌 | 주행 중 | 정지 중 | 정체 | 시간상한 | '
                 '최소 이격 중앙 [m] |')
        L.append('|---|---|---:|---:|---:|---:|---:|---:|---:|')
        for name, s in res['unavoid'].items():
            for p, v in s['policies'].items():
                L.append(f"| {name} | {p} | {v['success']:.3f} | {v['collision']:.3f} | "
                         f"{v['collision_moving']:.3f} | {v['collision_stationary']:.3f} | "
                         f"{v['stall']:.3f} | {v['timeout']:.3f} | "
                         f"{v.get('min_sep_median', float('nan')):.2f} |")
    if 'signal' in res:
        L.append('\n## (c) 무한 지평 가치의 항별 분해 (truncate 없음, 첫 종단까지)\n')
        L.append('| env | 정책 | V | 시간 | PBRS | 진행 | 평활 | 추종 | 힌지 | 목표 | 충돌 | 충돌률 | 성공률 | '
                 '회전 | 개입 합 | 충돌 항 분산 몫 |')
        L.append('|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|')
        for en, s in res['signal'].items():
            if en == 'C0':
                continue
            for p, v in s['policies'].items():
                cs = v['coll_var_share']
                L.append(f"| {en} | {p} | {v['V']:.2f} | {v['time']:.2f} | {v['shape']:.2f} | "
                         f"{v['progress']:.2f} | {v['smooth']:.3f} | {v['track']:.3f} | "
                         f"{v['dev']:.3f} | {v['goal']:.2f} | {v['collision']:.2f} | "
                         f"{v['coll_rate']:.3f} | {v['succ_rate']:.3f} | {v['spin']:.3f} | "
                         f"{v['interv']:.2f} | {'' if cs is None else f'{cs:.2f}'} |")
        L.append('\n| env | dV/dμ_v | 충돌 항 | 진행 항 | 목표 항 | PBRS 항 | 뒤집는 C | dV/dμ_ω | 회전 − 대기 | '
                 '추종 − 대기 |')
        L.append('|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|')
        for en, s in res['signal'].items():
            if en == 'C0':
                continue
            f = s['dV_dmu_v']
            cf = f['C_flip']
            L.append(f"| {en} | {f['V']:+.3f} | {f['collision']:+.3f} | {f['progress']:+.3f} | "
                     f"{f['goal']:+.3f} | {f['shape']:+.3f} | "
                     f"{'' if cf is None else f'{cf:.1f}'} | "
                     f"{s['dV_dmu_w']['V']:+.3f} | {s['spin_minus_idle']['mean']:+.4f} | "
                     f"{s['follow_minus_idle']['mean']:+.3f} |")
    return '\n'.join(L) + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--part', default='all', choices=['all', 'unavoid', 'signal'])
    ap.add_argument('--envs', type=int, default=2048)
    ap.add_argument('--signal-envs', type=int, default=1024)
    ap.add_argument('--signal-steps', type=int, default=1000)
    ap.add_argument('--seed', type=int, default=4242)
    ap.add_argument('--path-len', type=float, nargs=2, default=[3.0, 15.0])
    ap.add_argument('--n-obstacles', type=int, nargs=2, default=[0, 4])
    ap.add_argument('--filters', nargs='*', default=['orig=0:1', 'w2r1=2:1.0', 'w3r1=3:1.0',
                                                     'w4r1.5=4:1.5'])
    ap.add_argument('--signal-filter', default='w3r1=3:1.0')
    ap.add_argument('--no-compile', action='store_true')
    ap.add_argument('--code-commit', default='unknown')
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    t0 = time.time()
    res = {'meta': {'args': vars(a), 'code_commit': a.code_commit,
                    'loadavg_start': os.getloadavg(), 'gpu': torch.cuda.get_device_name(0)}}
    if a.part in ('all', 'unavoid'):
        res['unavoid'] = unavoid(a, a.out_dir)
    if a.part in ('all', 'signal'):
        res['signal'] = signal(a, a.out_dir)
    res['meta']['wall_s'] = round(time.time() - t0, 1)
    res['meta']['loadavg_end'] = os.getloadavg()
    res['meta']['gpu_max_reserved_gb'] = torch.cuda.max_memory_reserved() / 1e9
    tag = a.part
    with open(os.path.join(a.out_dir, f'g0_{tag}.json'), 'w') as f:
        json.dump(res, f, indent=1)
    with open(os.path.join(a.out_dir, f'g0_{tag}.md'), 'w') as f:
        f.write(render(res))
    print(f"[done] {res['meta']['wall_s']} s")


if __name__ == '__main__':
    main()
