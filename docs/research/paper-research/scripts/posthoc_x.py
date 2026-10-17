#!/usr/bin/env python3
"""
**사후 분석** (paper_research, 사전 등록 밖 — 조정자 요청 2026-10-18). held-out B 결과를 본 뒤 만든 분석이라
확증이 아니고 기술이다. held-out B 로 다시 튜닝·선택하지 않는다. 다음 라운드(held-out C) 설계 근거용.

H3 미확증(DWA-R − RPP_R00 = +1.55 % [+1.06, +2.10], ROS C1)의 시간 차가 경로의 어느 구간에서 생기는지 나눈다.

    python3 posthoc_x.py --runs /home/CAPYI_AI/paper_research/runs --out <dir>

방법 (C1 의 A2 궤적 20 Hz CSV, 참값 자세):
- 궤적 점마다 경로 호길이 s 를 구한다 (직전 투영 근처 창에서 최근접 선분 투영 — ros_bench 의 CTE 와 같은 방식).
- 수락(t = 0)부터 판정 통과(t_reach)까지 각 표본 간격(0.05 s)을 그 시점 s 가 속한 구간에 넣는다:
  - 출발 가속: s < 1.5 m (정지 출발 → 0.9 m/s 가 약 1 m 안에서 끝난다; 제어기 시동 지연 포함)
  - 목표 접근: s ≥ L − 1.5 m 이고 참값이 목표 0.10 m 안에 처음 들기 전 (감속·접근)
  - 판정 대기: 참값 0.10 m 첫 진입(t_first_entry) → 판정 통과(t_reach). xy 0.10 m 와 yaw 0.05 rad 를
    같은 순간에 만족할 때까지의 시간 (정렬·미세 접근)
  - 곡선: 그 밖에서 경로 곡률 |κ| > 0.1 1/m 정점 ± 0.5 m (ros_bench 곡선 표지와 같다)
  - 순항: 그 밖
- 경로마다 구간 시간 차 ΔT_z = T_a,z − T_b,z 를 b 의 전체 시간 T_b 로 나눈 몫 [%] 을 낸다. 구간 몫의 합 =
  (T_a − T_b)/T_b (H3 의 log 비와 거의 같다). 경로 묶음 부트스트랩 95 % (B 10 000, seed 12345).
- 비교 셋: dwa_R − RPP_R00, dwa_full − RPP_R00, dwa_R − dwa_full. 전체·매끈·통로·A* 별.
"""
import argparse
import collections
import csv
import glob
import json
import math
import os

import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', '..', 'src', 'amr_navigation'))
from amr_navigation import path_metrics  # noqa: E402

B = 10000
SEED = 12345
ZONES = ('start', 'cruise', 'curve', 'goal', 'judge')
ZNAME = {'start': '출발 가속 (s < 1.5 m)', 'cruise': '순항 (직선)', 'curve': '곡선 (|κ|>0.1 ±0.5 m)',
         'goal': '목표 접근 (s ≥ L − 1.5 m, 첫 진입 전)',
         'judge': '판정 대기 (참값 0.10 m 첫 진입 → 판정 통과)'}
PAIRS = (('dwa_R', 'RPP_R00'), ('dwa_full', 'RPP_R00'), ('dwa_R', 'dwa_full'))
FAMS = (('전체', ('smooth', 'narrow', 'astar')), ('매끈', ('smooth',)), ('통로', ('narrow',)),
        ('A*', ('astar',)))


def boot(v):
    v = np.asarray(v, dtype=float)
    if not len(v):
        return float('nan'), float('nan'), float('nan')
    rng = np.random.default_rng(SEED)
    bs = v[rng.integers(0, len(v), size=(B, len(v)))].mean(axis=1)
    return float(v.mean()), float(np.percentile(bs, 2.5)), float(np.percentile(bs, 97.5))


def fmt(m, lo, hi, nd=2):
    if math.isnan(m):
        return '–'
    return f'{m:+.{nd}f} [{lo:+.{nd}f}, {hi:+.{nd}f}]'


def labels(xy):
    """ros_bench curve_labels 와 같다 (path_metrics.discrete_curvature 0.25 m, |κ| > 0.1 ± 0.5 m)."""
    k = np.abs(path_metrics.discrete_curvature(xy, 0.25))
    s = path_metrics.cumulative_length(xy)
    curve = k > 0.1
    out = np.zeros(len(xy), dtype=bool)
    for i in range(len(xy)):
        out[i] = bool(np.any(curve & (np.abs(s - s[i]) <= 0.5)))
    return s, out


def project_s(xy, s_path, pts):
    """궤적 점 → 호길이 (직전 투영 근처 창)."""
    hint = 0
    n = len(xy)
    out = []
    for x, y in pts:
        b = max(0, hint - 10)
        e = min(n - 1, hint + 80)
        a = xy[b:e]
        d = xy[b + 1:e + 1] - a
        ll = np.maximum((d ** 2).sum(axis=1), 1e-12)
        t = np.clip(((x - a[:, 0]) * d[:, 0] + (y - a[:, 1]) * d[:, 1]) / ll, 0.0, 1.0)
        dist = np.hypot(x - (a[:, 0] + t * d[:, 0]), y - (a[:, 1] + t * d[:, 1]))
        j = int(np.argmin(dist))
        hint = b + j
        out.append(s_path[hint] + t[j] * (s_path[min(hint + 1, n - 1)] - s_path[hint]))
    return out


def zone_times(trace_fn, t_reach, xy, s_path, is_curve, t_first=-1.0):
    L = s_path[-1]
    rows = []
    with open(trace_fn, encoding='utf-8') as f:
        for r in csv.DictReader(f):
            if r['x'] == '' or float(r['t']) < 0.0 or float(r['t']) > t_reach + 1e-9:
                continue
            rows.append((float(r['t']), float(r['x']), float(r['y'])))
    if len(rows) < 2:
        return None
    ss = project_s(xy, s_path, [(x, y) for _, x, y in rows])
    zt = collections.Counter()
    for i in range(len(rows)):
        t0 = rows[i][0]
        t1 = rows[i + 1][0] if i + 1 < len(rows) else t_reach
        dt = max(0.0, min(t1, t_reach) - t0)
        s = ss[i]
        if 0.0 <= t_first <= t0:
            z = 'judge'
        elif s < 1.5:
            z = 'start'
        elif s >= L - 1.5:
            z = 'goal'
        else:
            j = int(np.clip(np.searchsorted(s_path, s), 0, len(s_path) - 1))
            z = 'curve' if is_curve[j] else 'cruise'
        zt[z] += dt
    zt['start'] += rows[0][0]       # 수락 직후 첫 표본 전 시간
    return zt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', required=True)
    ap.add_argument('--stage', default='c1')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    paths = {}
    for fn in ('heldout_smooth.json', 'heldout_narrow.json', 'heldout_astar.json'):
        with open(os.path.join(args.runs, 'paths', fn), encoding='utf-8') as f:
            for e in json.load(f):
                xy = np.array([[p[0], p[1]] for p in e['points']])
                s, lab = labels(xy)
                paths[e['id']] = (e['family'], xy, s, lab)
    recs = {}
    root = os.path.join(args.runs, args.stage)
    for fn in glob.glob(os.path.join(root, 'raw', '*.jsonl')):
        jid = os.path.basename(fn)[:-6]
        with open(fn, encoding='utf-8') as f:
            for line in f:
                r = json.loads(line)
                if r['success'] and r['arm'] in ('dwa_R', 'dwa_full', 'RPP_R00'):
                    tr = os.path.join(root, 'traces', jid,
                                      f"{r['arm']}__{r['scene']}__s{r['seed']}__r{r['rep']}.csv")
                    fam, xy, s, lab = paths[r['scene']]
                    zt = zone_times(tr, r['t_reach'], xy, s, lab, r.get('t_first_entry', -1.0))
                    if zt is not None:
                        recs[(r['arm'], r['scene'])] = (fam, r['t_reach'], zt)
    md = ['# 사후 분해 — H3 의 시간 차는 어느 구간에서 생기는가 (ROS C1, held-out B)', '',
          '**사후 분석, 확증 아님.** 방법은 이 파일(`scripts/posthoc_x.py`) 머리말. 값 = 경로마다 '
          '(구간 시간 차 ÷ 기준 팔 전체 시간) [%] 의 평균 [부트스트랩 95 %], 구간 몫의 합 = 전체 시간 차.', '']
    summary = {}
    for a, b in PAIRS:
        md += [f'## {a} − {b}', '',
               '| 경로 | n | 전체 [%] | ' + ' | '.join(ZNAME[z] for z in ZONES) + ' |',
               '| --- | --- | --- |' + ' --- |' * len(ZONES)]
        for fname, fams in FAMS:
            common = [sc for (arm, sc) in recs if arm == a and (b, sc) in recs and
                      recs[(a, sc)][0] in fams]
            tot = [100.0 * (recs[(a, sc)][1] - recs[(b, sc)][1]) / recs[(b, sc)][1] for sc in common]
            cells = []
            for z in ZONES:
                v = [100.0 * (recs[(a, sc)][2][z] - recs[(b, sc)][2][z]) / recs[(b, sc)][1]
                     for sc in common]
                cells.append(fmt(*boot(v)))
                summary[f'{a}-{b}|{fname}|{z}'] = boot(v)
            summary[f'{a}-{b}|{fname}|total'] = boot(tot)
            md.append(f'| {fname} | {len(common)} | {fmt(*boot(tot))} | ' + ' | '.join(cells) + ' |')
        # 구간 시간 자체 (초, 중앙)
        md += ['', f'구간 시간 중앙 [s] ({a} / {b}, 전체 경로):', '',
               '| 구간 | ' + a + ' | ' + b + ' |', '| --- | --- | --- |']
        for z in ZONES:
            va = [recs[k][2][z] for k in recs if k[0] == a]
            vb = [recs[k][2][z] for k in recs if k[0] == b]
            md.append(f'| {ZNAME[z]} | {np.median(va):.2f} | {np.median(vb):.2f} |')
        md.append('')
    with open(os.path.join(args.out, 'posthoc_h3_decomp.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(md) + '\n')
    with open(os.path.join(args.out, 'posthoc_h3_decomp.json'), 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=1)
    print('\n'.join(md))


if __name__ == '__main__':
    main()
