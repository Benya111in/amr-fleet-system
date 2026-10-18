#!/usr/bin/env python3
"""
chain_sim_x 두 빌드의 동치 시험 (prereg_r2.md §4): 기준 빌드(85e66cb) 대 새 빌드(새 스위치 끔).

    run_equiv_x.py run --bin <chain_sim_x> --label <ref|new|newlog> --out <dir>
        --cfg-dir <dir> [--paths /abl/paths] [--workers 12] [--log-resets]
    run_equiv_x.py compare --out <dir> --a ref --b new [--b newlog]

held-out A 60 경로 (scenes_heldout_{smooth,narrow,astar}.txt) x 팔 dwa_full·X03
(cfg-dir 의 dwa_full.cfg·dwa_x_X03.cfg) x 조건 잡음 없음·잡음 seed 1 (원본 NOISE 인자).
두 빌드는 둘 다 컨테이너의 /ros2_ws 에 올려야 하므로 (RPATH) label 마다 따로 돈다.
비교는 compare_chain_jsonl.py 규칙: 기준(a) 줄의 필드 전부를 출력 원문 그대로 비교하고
cycle_us_* 만 뺀다. b 가 끝에 더한 필드는 비교하지 않는다.
"""
import argparse
import concurrent.futures as cf
import hashlib
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FAMILIES = ('smooth', 'narrow', 'astar')
ARMS = {'dwa_full': 'dwa_full.cfg', 'X03': 'dwa_x_X03.cfg'}
# docs/research/dwa-ablation/scripts/orchestrate.py NOISE 와 같다
NOISE = ['odom_noise_v=0.02', 'odom_noise_w=0.02', 'pose_noise_xy=0.01', 'pose_noise_yaw=0.005',
         'pose_noise_tau=1.0', 'lidar=1']
CONDS = {'clean': [], 'noise_s1': NOISE + ['seed=1']}


def sha256(fn):
    h = hashlib.sha256()
    with open(os.path.realpath(fn), 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def run_one(cmd, log):
    t0 = time.time()
    with open(log, 'w', encoding='utf-8') as lf:
        rc = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT).returncode
    return {'cmd': cmd, 'rc': rc, 'sec': round(time.time() - t0, 2)}


def do_run(args):
    d = os.path.join(args.out, args.label)
    os.makedirs(os.path.join(d, 'logs'), exist_ok=False)
    if args.log_resets:
        os.makedirs(os.path.join(d, 'resets'), exist_ok=False)
    jobs = []
    for arm, cfg in ARMS.items():
        for cond, extra in CONDS.items():
            for fam in FAMILIES:
                sc = os.path.join(args.paths, f'scenes_heldout_{fam}.txt')
                out = os.path.join(d, f'{arm}__{cond}__{fam}.jsonl')
                cmd = [args.bin, sc, out, f'cfg={os.path.join(args.cfg_dir, cfg)}',
                       f'arm={arm}'] + extra
                if args.log_resets:
                    rl = os.path.join(d, 'resets', f'{arm}__{cond}__{fam}.csv')
                    cmd.append(f'reset_log={rl}')
                jobs.append((cmd, os.path.join(d, 'logs', f'{arm}__{cond}__{fam}.log')))
    t0 = time.time()
    load0 = os.getloadavg()
    with cf.ThreadPoolExecutor(max_workers=args.workers) as ex:
        results = list(ex.map(lambda j: run_one(*j), jobs))
    for arm in ARMS:
        for cond in CONDS:
            with open(os.path.join(d, f'{arm}__{cond}.jsonl'), 'w', encoding='utf-8') as f:
                for fam in FAMILIES:
                    with open(os.path.join(d, f'{arm}__{cond}__{fam}.jsonl'),
                              encoding='utf-8') as pf:
                        f.write(pf.read())
    manifest = {
        'label': args.label, 'bin': args.bin, 'bin_sha256': sha256(args.bin),
        'commit': args.commit, 'start': t0, 'end': time.time(),
        'loadavg': [load0, os.getloadavg()],
        'cfg': {arm: sha256(os.path.join(args.cfg_dir, cfg)) for arm, cfg in ARMS.items()},
        'scenes': {fam: sha256(os.path.join(args.paths, f'scenes_heldout_{fam}.txt'))
                   for fam in FAMILIES},
        'noise_args': NOISE, 'runs': results,
    }
    with open(os.path.join(d, 'manifest.json'), 'w', encoding='utf-8') as f:
        json.dump(manifest, f, ensure_ascii=False, indent=1)
    bad = [r for r in results if r['rc'] != 0]
    print(f'{args.label}: {len(results)} runs, rc != 0: {len(bad)}')
    return 1 if bad else 0


def do_compare(args):
    groups = []
    ok = True
    for b in args.b:
        for arm in ARMS:
            for cond in CONDS:
                fa = os.path.join(args.out, args.a, f'{arm}__{cond}.jsonl')
                fb = os.path.join(args.out, b, f'{arm}__{cond}.jsonl')
                sm = os.path.join(args.out, f'compare__{args.a}_vs_{b}__{arm}__{cond}.json')
                rc = subprocess.run(
                    [sys.executable, os.path.join(HERE, 'compare_chain_jsonl.py'), fa, fb,
                     '--out', sm], stdout=subprocess.PIPE, text=True).returncode
                with open(sm, encoding='utf-8') as f:
                    s = json.load(f)
                groups.append({'a': args.a, 'b': b, 'arm': arm, 'cond': cond, 'rc': rc,
                               'n_lines': s['n_lines'], 'n_identical': s['n_identical'],
                               'n_fields_compared': s['n_fields_compared']})
                ok = ok and rc == 0 and s['n_lines'] == 60
    summary = {'pass': ok, 'groups': groups,
               'totals': {b: [sum(g['n_identical'] for g in groups if g['b'] == b),
                              sum(g['n_lines'] for g in groups if g['b'] == b)]
                          for b in args.b},
               'compare_rule': 'compare_chain_jsonl.py: 기준 줄의 필드 전부 출력 원문 비교, '
                               'cycle_us_* 제외, b 가 끝에 더한 필드는 비교 안 함'}
    with open(os.path.join(args.out, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('--bin', required=True)
    r.add_argument('--label', required=True)
    r.add_argument('--out', required=True)
    r.add_argument('--cfg-dir', required=True)
    r.add_argument('--paths', default='/abl/paths')
    r.add_argument('--workers', type=int, default=12)
    r.add_argument('--commit', default='')
    r.add_argument('--log-resets', action='store_true')
    c = sub.add_parser('compare')
    c.add_argument('--out', required=True)
    c.add_argument('--a', required=True)
    c.add_argument('--b', required=True, action='append')
    args = ap.parse_args()
    return do_run(args) if args.cmd == 'run' else do_compare(args)


if __name__ == '__main__':
    sys.exit(main())
