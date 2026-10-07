#!/usr/bin/env python3
"""
단계 C 묶음(또는 파일럿 레인) 하나를 실행 목록에 등록할 레코드로 만든다 (표준 라이브러리 + analyze_c 의 분류).

    register_batch.py <묶음 디렉터리> --run-id C_b1gzc_dwa_b0 [--stage C2] [--print]
        → JSON 을 stdout 으로 (python3 rl/tools/notebook.py register --json "$(…)" 로 넘긴다)

결과 dict: 행 수, 제외 분류 E1–E4, 집합별 n·strict_gt·strict_est·nav2compat_gt·system_success 수, RTF 중앙,
에피소드 벽시계 합, 시도 수. 중간 기록용이다 — 주 결론(짝 비교)은 analyze_c.py 가 모든 묶음이 끝난 뒤에 낸다.
"""

import argparse
import csv
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import analyze_c as A  # noqa: E402


def first_line(path, default='n/a'):
    """파일 첫 줄 (없으면 default)."""
    try:
        with open(path, encoding='utf-8') as f:
            return f.readline().strip()
    except OSError:
        return default


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('batch_dir')
    ap.add_argument('--run-id', required=True)
    ap.add_argument('--stage', default='C2')
    a = ap.parse_args()
    bd = os.path.abspath(a.batch_dir)
    atts = A.attempts(bd)
    if not atts:
        print('시도 없음', file=sys.stderr)
        return 2
    rows = {}
    for at in atts:
        f = os.path.join(at, 'gzbench.csv')
        if os.path.exists(f):
            with open(f, encoding='utf-8', newline='') as fh:
                for r in csv.DictReader(fh):
                    rows.setdefault(int(r['exec_index']), r)
    rs = list(rows.values())
    head = {}
    for line in open(os.path.join(atts[0], 'HEAD.txt'), encoding='utf-8'):
        k, _, v = line.partition(' ')
        head.setdefault(k, v.strip())
    ctrl = head.get('lane', '').split()
    lane = ctrl[0] if ctrl else '?'
    res = {'n': len(rs), 'attempts': len(atts),
           'excl': {k: sum(1 for r in rs if A.excl(r) == k) for k in ('E1', 'E2', 'E3', 'E4')}}
    for st in ('main', 'stress'):
        s = [r for r in rs if r.get('difficulty') == st]
        res[st] = {'n': len(s), **{c: sum(1 for r in s if A.truthy(r.get(c)))
                                   for c in A.RATE_COLS}}
    rtf = [A.num(r['rtf']) for r in rs if A.num(r['rtf']) == A.num(r['rtf'])]
    res['rtf_median'] = round(statistics.median(rtf), 3) if rtf else None
    walls = [A.num(r['wall_episode_s']) for r in rs]
    res['wall_episode_sum_s'] = round(sum(w for w in walls if w == w), 1)
    pol = rs[0].get('policy_id') if rs else 'n/a'
    rec = {
        'run_id': a.run_id, 'stage': a.stage, 'agent': 'b1-gzc', 'kind': 'gazebo', 'sim': 'gazebo',
        'commit': head.get('snapshot', 'n/a').split()[0][:7], 'dirty': False,
        'config': (f"c_batch.sh 레인 {lane}: {head.get('lane', '')}; 고정 사본 "
                   f"{head.get('snapshot', '')}; "
                   f"model_sha256 {head.get('model_sha256', '')[:16]}; "
                   f"world {head.get('world_sha256', '')[:16]}; scenarios "
                   f"{head.get('scenarios_sha256', '')[:16]}; "
                   f"작업 트리 {head.get('worktree_head', '')[:7]}"),
        'seed': f'scenarios_v1 seed 20261006 / policy {pol}',
        'started': first_line(os.path.join(atts[0], 'started.txt')),
        'ended': first_line(os.path.join(atts[-1], 'ended.txt')),
        'raw_dir': bd, 'result': res, 'label': '[측정]',
        'load': (first_line(os.path.join(atts[0], 'host_load.txt')) + ' / '
                 + open(os.path.join(atts[-1], 'host_load.txt'), encoding='utf-8')
                 .read().strip().split('\n')[-1]),
    }
    print(json.dumps(rec, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
