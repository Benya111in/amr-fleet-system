#!/usr/bin/env python3
"""
B5 교차 평가 실행기 — 2D 시뮬레이터에서 Isaac 평가 집합을 RL(ONNX) 또는 2D-DWA 로 (env 마다 첫 에피소드).

  python3 rl/sim2d/tools/b5_crosseval.py --arm dwa --world <W1 yaml> --pool <npz> --out <dir>
  python3 rl/sim2d/tools/b5_crosseval.py --arm rl --policy <policy_final.onnx> --world <W1 yaml> \
      --pool <npz> --out <dir>

규칙 (노트 항목 2026-10-08_*_b1-b5_* 「설정」 에 사전 등록):
  - 행 = 집합 순서 main_3_15, stress_3_15, main_3_30, stress_3_30 의 경로 id (Isaac 풀 seed 1, 내용 해시 대조)
  - 시작 yaw = Isaac 규칙 (첫 점 → 0.5 m 앞 점 접선) + U[−0.5, 0.5]
    (numpy default_rng(평가 seed) — 두 팔·모든 시드 같은 값)
  - DR 끔(명목: T_m 0.08 · T_d 0.04 · 가속 배율 1 · LiDAR σ 0.03 · 관측 지연 0), 장애물 없음,
    safety φ = safety_lite 켬
  - RL: ONNX 출력 `action` = tanh(μ) (결정론), CPU onnxruntime. DWA: amr_dwa2d_py (팀 코어, 배포 설정, 20 Hz)
  - 끝: 계약 사건 (성공 0.10 m ∧ |v| ≤ 0.05 · 충돌 · 정체 20 s/0.3 m · 시간 상한 min(120, 20 + 3L)).
    unfinished 는 0 이어야 한다
출력: episodes.npz (행별), summary.json, registry.json (rl/tools/notebook.py register 용 초안), run.log
"""

import argparse
import datetime as dt
import hashlib
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM2D = os.path.dirname(HERE)
RL = os.path.dirname(SIM2D)
REPO = os.path.dirname(RL)
SETS = ('main_3_15', 'stress_3_15', 'main_3_30', 'stress_3_30')
OUTCOME = {0: 'success', 1: 'collision', 2: 'stall', 3: 'timeout', -1: 'unfinished'}


def parse():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm', choices=('dwa', 'rl'), required=True)
    ap.add_argument('--policy', default='', help='RL: policy_final.onnx')
    ap.add_argument('--world', required=True, help='2D 세계 map_server yaml (W1)')
    ap.add_argument('--eval-set', default=os.path.join(RL, 'isaac', 'eval_sets',
                                                       'b1_eval_sets_v1.json'))
    ap.add_argument('--pool', required=True, help='Isaac 풀 npz (pts, offs, length)')
    ap.add_argument('--paths-npz', default='',
                    help='b5_plan_paths.py 출력 — 주면 두 팔 모두 그 계획 경로 (빈 값 = Isaac 풀 경로)')
    ap.add_argument('--sets', default=','.join(SETS))
    ap.add_argument('--rows', default='', help='시험용: 앞에서 k 행만 (빈 값 = 전부)')
    ap.add_argument('--pool-rows', default='', help='스모크용: 집합 대신 풀 행 "lo:hi" (집합 무시)')
    ap.add_argument('--seed', type=int, default=4242)
    ap.add_argument('--threads', type=int, default=8)
    ap.add_argument('--max-steps', type=int, default=1300)
    ap.add_argument('--device', default='cuda')
    ap.add_argument('--out', required=True)
    ap.add_argument('--run-id', default='')
    ap.add_argument('--no-compile', action='store_true')
    return ap.parse_args()


def git(*a):
    return subprocess.run(['git', '-C', REPO, *a], capture_output=True, text=True).stdout.strip()


def load():
    return [f'{x:.2f}' for x in os.getloadavg()]


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest()


def build_rows(a):
    sys.path.insert(0, os.path.join(RL, 'isaac'))
    from amr_isaac import eval_sets as ES
    with np.load(a.pool) as z:
        pts, offs, length = z['pts'], z['offs'], z['length']
    if a.pool_rows:
        lo, hi = (int(x) for x in a.pool_rows.split(':'))
        rows = [('pool', i) for i in range(lo, hi)]
        digest = ES.pool_digest(pts, offs, length)
    else:
        doc = ES.load_sets(a.eval_set)
        digest = ES.check_pool(doc, pts, offs, length)
        names = [s for s in a.sets.split(',') if s]
        rows = [(s, int(p)) for s in names for p in doc['sets'][s]]
    if a.rows:
        rows = rows[:int(a.rows)]
    if not a.paths_npz:
        paths = [np.asarray(pts[offs[p]:offs[p + 1]], dtype=np.float64) for _, p in rows]
        return rows, paths, digest
    # NpzFile 은 키를 읽을 때마다 압축을 푼다 — 배열을 한 번만 꺼낸다
    with np.load(a.paths_npz) as q:
        flat, status, n_pts = q['flat'], q['status'], q['n_pts']
        qset, qpid = q['set'], q['pid']
    o = np.concatenate([[0], np.cumsum(n_pts)])
    key = {(str(s), int(p)): i for i, (s, p) in enumerate(zip(qset, qpid))}
    paths = []
    for s, p in rows:
        i = key[(s, p)]
        if status[i] != 'success':
            raise RuntimeError(f'계획 실패 행 ({s}, {p}) — 사전 등록: 계획 실패 행은 없어야 한다')
        paths.append(np.asarray(flat[o[i]:o[i + 1]], dtype=np.float64))
    return rows, paths, digest


def main():
    a = parse()
    os.makedirs(a.out, exist_ok=True)
    done_flag = os.path.join(a.out, 'summary.json')
    if os.path.exists(done_flag):
        with open(done_flag) as f:
            if json.load(f).get('complete'):
                print(f'건너뜀 (이미 끝남): {a.out}', flush=True)
                return 0
    os.environ.setdefault('SIM2D_CACHE', os.path.join(os.path.dirname(os.path.abspath(a.out)),
                                                      'cache'))
    sys.path.insert(0, SIM2D)
    import torch
    from amr_sim2d import crosseval as CE
    from amr_sim2d.env import EnvConfig
    started = dt.datetime.now().astimezone().isoformat(timespec='seconds')
    load0 = load()
    rows, paths, digest = build_rows(a)
    N = len(rows)
    rng = np.random.default_rng(a.seed)
    noise = rng.uniform(-0.5, 0.5, N)
    yaws = np.array([CE.start_yaw_isaac(p[:, :2], e) for p, e in zip(paths, noise)])
    cfg = EnvConfig(num_envs=N, device=a.device, seed=a.seed, arm='B', map_yaml=a.world,
                    n_paths=16, path_len=(3.0, 30.0), obstacle_mode='none', obstacle_slots=0,
                    dr=False, safety=True, compile=not a.no_compile, truncate=True)
    log = open(os.path.join(a.out, 'run.log'), 'a')

    def say(msg):
        line = f'{dt.datetime.now():%H:%M:%S} {msg}'
        print(line, flush=True)
        log.write(line + '\n')
        log.flush()
    say(f'arm={a.arm} rows={N} world={a.world} seed={a.seed} '
        f'commit={git("rev-parse", "--short", "HEAD")} load={load0}')
    env = CE.FixedEpisodeEnv(cfg, [p[:, :2] for p in paths], yaws, log=say)
    obs = env.reset()
    sess = dwa = None
    if a.arm == 'rl':
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.intra_op_num_threads = a.threads
        so.inter_op_num_threads = 1
        sess = ort.InferenceSession(a.policy, so, providers=['CPUExecutionProvider'])
        meta = sess.get_modelmeta().custom_metadata_map
        if meta.get('obs_dim', '291') != '291' or meta.get('arm', 'B') != 'B':
            raise RuntimeError(f'ONNX 메타데이터가 계약과 다르다: {meta}')
        say(f'onnx meta: {json.dumps(meta, ensure_ascii=False)[:400]}')
    else:
        from amr_sim2d import dwa2d
        dwa = dwa2d.DwaBatch(N, threads=a.threads)
        for i, p in enumerate(paths):
            dwa.reset(i, p)
        sp = CE.sensor_poses(env.pose)
        dwa.update_costmap(np.arange(N), sp.cpu().numpy(), env.pose.cpu().numpy(),
                           env.ranges.cpu().numpy())
    rec = {k: np.full(N, np.nan) for k in ('time_s', 'path_len', 'd_start', 'd_final',
                                           'd_goal_end', 'travel', 'max_e', 'goal_min',
                                           'return', 'v_end', 'w_end')}
    for k in ('steps', 'spin_steps', 'interv_sub', 'interv_events', 'stop_sub'):
        rec[k] = np.full(N, -1, dtype=np.int64)
    rec['pose_end'] = np.full((N, 3), np.nan)
    rec['cost'] = np.full((N, 5), np.nan)
    outcome = np.full(N, -1, dtype=np.int64)
    flags = {k: np.zeros(N, dtype=bool) for k in ('success', 'collision', 'collision_moving',
                                                  'collision_turning', 'stalled', 'timeout')}
    dwa_cnt = {k: np.zeros(N, dtype=np.int64) for k in ('no_valid_ticks', 'exceptions')}
    active = np.ones(N, dtype=bool)
    t0 = time.time()
    steps = 0
    infer_s = 0.0
    while active.any() and steps < a.max_steps:
        if a.arm == 'rl':
            idx = np.nonzero(active)[0]
            ti = time.time()
            x = obs[torch.as_tensor(idx, device=obs.device)].cpu().numpy().astype(np.float32)
            u = sess.run(['action'], {'obs': x})[0]
            infer_s += time.time() - ti
            act = torch.zeros(N, 2, dtype=torch.float32, device=env.dev)
            act[torch.as_tensor(idx, device=env.dev)] = torch.as_tensor(u, device=env.dev)
            obs, _r, _te, _tr, info = env.step_rl(act)
        else:
            ti = time.time()
            obs, _r, _te, _tr, info = env.step_dwa(dwa, active)
            infer_s += time.time() - ti
            for k in dwa_cnt:
                dwa_cnt[k][active] += info['dwa'][k][active]
        steps += 1
        done = info['done'].cpu().numpy() & active
        di = np.nonzero(done)[0]
        if len(di):
            snap = env.episode_snapshot(torch.as_tensor(di, device=env.dev))
            for k, v in snap.items():
                rec[k][di] = v
            f = {k: info[k].cpu().numpy()[di] for k in flags}
            for k in flags:
                flags[k][di] = f[k]
            oc = np.where(f['stalled'], 2, 3)
            oc = np.where(f['collision'], 1, oc)
            outcome[di] = np.where(f['success'], 0, oc)
            active[di] = False
        if a.arm == 'dwa':
            live = np.nonzero(active)[0]
            if len(live):
                sp = CE.sensor_poses(env.pose)
                li = torch.as_tensor(live, device=env.dev)
                dwa.update_costmap(live, sp[li].cpu().numpy(), env.pose[li].cpu().numpy(),
                                   env.ranges[li].cpu().numpy())
        if steps % 100 == 0:
            say(f'step {steps} active {int(active.sum())} wall {time.time() - t0:.0f}s '
                f'load {load()[0]}')
    wall = time.time() - t0
    unfinished = int(active.sum())
    nav2 = rec['goal_min'] <= 0.25
    set_names = np.array([s for s, _ in rows])
    pids = np.array([p for _, p in rows], dtype=np.int64)
    np.savez_compressed(os.path.join(a.out, 'episodes.npz'), set=set_names, pid=pids,
                        yaw0=yaws, outcome=outcome, nav2=nav2, **flags, **rec,
                        **{'dwa_' + k: v for k, v in dwa_cnt.items()})
    by = {}
    for s in dict.fromkeys(set_names.tolist()):
        m = set_names == s
        by[s] = {'n': int(m.sum()), 'strict_success': int((outcome[m] == 0).sum()),
                 'strict_rate': float((outcome[m] == 0).mean()),
                 'nav2_compat': int(nav2[m].sum()), 'collision': int((outcome[m] == 1).sum()),
                 'stall': int((outcome[m] == 2).sum()), 'timeout': int((outcome[m] == 3).sum()),
                 'unfinished': int((outcome[m] == -1).sum())}
    ended = dt.datetime.now().astimezone().isoformat(timespec='seconds')
    load1 = load()
    summ = {'complete': unfinished == 0, 'arm': a.arm, 'policy': a.policy or None,
            'policy_sha256': sha256(a.policy) if a.policy else None,
            'world': a.world, 'world_sha256': sha256(a.world.replace('.yaml', '.pgm')),
            'eval_set': a.eval_set if not a.pool_rows else None, 'pool': a.pool,
            'paths_npz': a.paths_npz or None,
            'paths_sha256': sha256(a.paths_npz) if a.paths_npz else None,
            'pool_digest': digest, 'pool_rows': a.pool_rows or None, 'rows': N,
            'seed': a.seed, 'steps': steps, 'unfinished': unfinished, 'wall_s': round(wall, 1),
            'infer_or_dwa_wall_s': round(infer_s, 1), 'threads': a.threads,
            'commit': git('rev-parse', '--short', 'HEAD'),
            'dirty': bool(git('status', '--porcelain', '--', 'rl/sim2d')),
            'started': started, 'ended': ended, 'load': {'start': load0, 'end': load1},
            'by': by, 'dwa_exceptions_total': int(dwa_cnt['exceptions'].sum()),
            'dwa_no_valid_ticks_total': int(dwa_cnt['no_valid_ticks'].sum())}
    if a.arm == 'dwa':
        from amr_sim2d import dwa2d
        conf = dwa2d.load_config()
        summ['dwa_config'] = conf['cfg']
        summ['dwa_wrapper'] = conf['wrapper']
        summ['dwa_costmap'] = conf['costmap']
        summ['dwa_sources'] = conf['sources']
    with open(os.path.join(a.out, 'summary.json'), 'w') as f:
        json.dump(summ, f, indent=1, ensure_ascii=False)
    result = {s: {'n': v['n'], 'strict': v['strict_success'], 'nav2': v['nav2_compat'],
                  'collision': v['collision']} for s, v in by.items()}
    result['unfinished'] = unfinished
    path_src = ('team A* ' + os.path.basename(a.paths_npz)) if a.paths_npz else 'isaac pool'
    reg = {'run_id': a.run_id or os.path.basename(os.path.abspath(a.out)), 'stage': 'B5',
           'agent': 'b1-b5', 'kind': 'eval', 'sim': '2d', 'commit': summ['commit'],
           'dirty': summ['dirty'],
           'config': (f'b5_crosseval.py --arm {a.arm} world={os.path.basename(a.world)} '
                      f'sets={a.sets if not a.pool_rows else "pool " + a.pool_rows}, '
                      f'paths={path_src}, '
                      f'DR off, obstacles none, safety_lite, first episode per env'
                      + (f', policy sha256 {summ["policy_sha256"][:12]}' if a.policy else
                         ', DWA nav2_params.yaml 20 Hz')),
           'seed': a.seed, 'started': started, 'ended': ended,
           'raw_dir': os.path.abspath(a.out), 'result': result,
           'label': '[측정]', 'load': f'start {" ".join(load0)} / end {" ".join(load1)}'}
    with open(os.path.join(a.out, 'registry.json'), 'w') as f:
        json.dump(reg, f, indent=1, ensure_ascii=False)
    say(f'끝 unfinished={unfinished} wall={wall:.0f}s by={json.dumps(by)}')
    return 0 if unfinished == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
