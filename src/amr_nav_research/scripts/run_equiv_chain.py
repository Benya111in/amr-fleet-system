#!/usr/bin/env python3
"""
chain_sim 동치 시험을 돌린다 (prereg.md §3.3): 원본 chain_sim 과 chain_sim_x (스위치 끔).

    run_equiv_chain.py --out <dir> --commit <sha> --dirty <n>
        [--paths /abl/paths] [--cfg /abl/cfg] [--workers 12]

held-out A 60 경로 (scenes_heldout_{smooth,narrow,astar}.txt) x 팔 dwa_full·dwa_no_m3 x
조건 잡음 없음·잡음 seed 1 (원본 orchestrate.py 의 NOISE 인자 그대로).
chain_sim_x 는 두 번 돈다: 인자 없이 (x), 재설정 기록만 켜고 (xlog — 기록이 거동을 바꾸지
않음을 본다). 결과는 <dir>/{orig,x,xlog}/<arm>__<cond>.jsonl, 비교는 compare_chain_jsonl.py
규칙 (계산 시간 필드만 빼고 출력 원문 비교). 요약은 <dir>/summary.json, 실행 기록은
<dir>/manifest.json. 컨테이너 안에서 돈다 (/ros2_ws = 작업공간).
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

ORIG = '/ros2_ws/install/amr_navigation/lib/amr_navigation/chain_sim'
XBIN = '/ros2_ws/install/amr_nav_research/lib/amr_nav_research/chain_sim_x'
FAMILIES = ('smooth', 'narrow', 'astar')
ARMS = ('dwa_full', 'dwa_no_m3')
# docs/research/dwa-ablation/scripts/orchestrate.py NOISE 와 같다
NOISE = ['odom_noise_v=0.02', 'odom_noise_w=0.02', 'pose_noise_xy=0.01', 'pose_noise_yaw=0.005',
         'pose_noise_tau=1.0', 'lidar=1']
CONDS = {'clean': [], 'noise_s1': NOISE + ['seed=1']}


def sha256(fn):
    h = hashlib.sha256()
    with open(fn, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def run(cmd, log):
    t0 = time.time()
    with open(log, 'w', encoding='utf-8') as lf:
        rc = subprocess.run(cmd, stdout=lf, stderr=subprocess.STDOUT).returncode
    return {'cmd': cmd, 'rc': rc, 'sec': round(time.time() - t0, 2)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--out', required=True)
    ap.add_argument('--commit', required=True)
    ap.add_argument('--dirty', required=True)
    ap.add_argument('--paths', default='/abl/paths')
    ap.add_argument('--cfg', default='/abl/cfg')
    ap.add_argument('--workers', type=int, default=12)
    args = ap.parse_args()
    for sub in ('orig', 'x', 'xlog', 'logs', 'resets'):
        os.makedirs(os.path.join(args.out, sub), exist_ok=False)
    jobs = []
    for arm in ARMS:
        cfg = os.path.join(args.cfg, f'{arm}.cfg')
        for cond, extra in CONDS.items():
            for fam in FAMILIES:
                sc = os.path.join(args.paths, f'scenes_heldout_{fam}.txt')
                base = [sc, None, f'cfg={cfg}', f'arm={arm}'] + extra
                for kind, binary in (('orig', ORIG), ('x', XBIN), ('xlog', XBIN)):
                    out = os.path.join(args.out, kind, f'{arm}__{cond}__{fam}.jsonl')
                    cmd = [binary] + base
                    cmd[2] = out
                    if kind == 'xlog':
                        cmd.append('reset_log=' + os.path.join(
                            args.out, 'resets', f'{arm}__{cond}__{fam}.csv'))
                    log = os.path.join(args.out, 'logs', f'{kind}__{arm}__{cond}__{fam}.log')
                    jobs.append((cmd, log))
    t0 = time.time()
    load0 = os.getloadavg()
    with cf.ThreadPoolExecutor(max_workers=args.workers) as ex:
        results = list(ex.map(lambda j: run(*j), jobs))
    load1 = os.getloadavg()
    # 묶음(팔 x 조건)마다 세 가족을 이어 붙여 60 줄로 비교
    groups = []
    for arm in ARMS:
        for cond in CONDS:
            merged = {}
            for kind in ('orig', 'x', 'xlog'):
                fn = os.path.join(args.out, kind, f'{arm}__{cond}.jsonl')
                with open(fn, 'w', encoding='utf-8') as f:
                    for fam in FAMILIES:
                        part = os.path.join(args.out, kind, f'{arm}__{cond}__{fam}.jsonl')
                        with open(part, encoding='utf-8') as pf:
                            f.write(pf.read())
                merged[kind] = fn
            for kind in ('x', 'xlog'):
                sm = os.path.join(args.out, f'compare__{kind}__{arm}__{cond}.json')
                rc = subprocess.run(
                    [sys.executable, os.path.join(HERE, 'compare_chain_jsonl.py'),
                     merged['orig'], merged[kind], '--out', sm],
                    stdout=subprocess.PIPE, text=True)
                with open(sm, encoding='utf-8') as f:
                    s = json.load(f)
                groups.append({'arm': arm, 'cond': cond, 'kind': kind, 'rc': rc.returncode,
                               'n_lines': s['n_lines'], 'n_identical': s['n_identical'],
                               'n_fields_compared': s['n_fields_compared']})
    n = sum(g['n_lines'] for g in groups if g['kind'] == 'x')
    n_same = sum(g['n_identical'] for g in groups if g['kind'] == 'x')
    nl = sum(g['n_lines'] for g in groups if g['kind'] == 'xlog')
    nl_same = sum(g['n_identical'] for g in groups if g['kind'] == 'xlog')
    resets = 0
    for fn in os.listdir(os.path.join(args.out, 'resets')):
        with open(os.path.join(args.out, 'resets', fn), encoding='utf-8') as f:
            resets += max(0, sum(1 for _ in f) - 1)
    summary = {
        'pass': n_same == n and nl_same == nl and n == 240 and all(
            r['rc'] == 0 for r in results),
        'x_identical': n_same, 'x_lines': n, 'xlog_identical': nl_same, 'xlog_lines': nl,
        'reset_rows_logged': resets, 'groups': groups,
        'compare_rule': 'compare_chain_jsonl.py: 원본 필드 전부 출력 원문 비교, cycle_us_* 제외',
    }
    manifest = {
        'commit': args.commit, 'dirty': args.dirty,
        'start': t0, 'end': time.time(), 'loadavg': [load0, load1],
        'binaries': {'chain_sim': sha256(ORIG), 'chain_sim_x': sha256(XBIN)},
        'scenes': {fam: sha256(os.path.join(args.paths, f'scenes_heldout_{fam}.txt'))
                   for fam in FAMILIES},
        'cfg': {arm: sha256(os.path.join(args.cfg, f'{arm}.cfg')) for arm in ARMS},
        'noise_args': NOISE, 'runs': results,
    }
    with open(os.path.join(args.out, 'summary.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    with open(os.path.join(args.out, 'manifest.json'), 'w', encoding='utf-8') as f:
        json.dump(manifest, f, ensure_ascii=False, indent=1)
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    return 0 if summary['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
