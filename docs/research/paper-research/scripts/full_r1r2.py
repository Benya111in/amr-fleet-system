#!/usr/bin/env python3
"""
1·2 라운드 원자료 전수 분석 (prereg_r2.md §6 — 3 라운드 진입 판단 자료). **기술·사후 분석이다.**
진입 결정과 3 라운드 사전 등록은 조정자가 한다. 이 스크립트는 근거 표만 만든다.

    python3 full_r1r2.py --runs /home/CAPYI_AI/paper_research/runs --out <dir>

내용:
A. 원자료 목록 (단계·실행 수·실시간 품질)
B. 진입 조건 (prereg_r2 §6): H4·H6 판정, C2·C3 에서 DWA-R2 − dwa_full
C. 원인 후보:
   C1 구간 분해 (출발 가속·순항·곡선·목표 접근·판정 대기) — 1 라운드 C1, 2 라운드 C1
   C2 목표 0.10 m 첫 진입 순간 yaw 오차 분포와 판정 대기
   C3 0→0.9 m/s 시간, 실행당 재설정 수 (chain, 국면별)
   C4 O 장면 실패 원인 (장애물 앞 정지·충돌·시간 초과), 끝 이격·진행률
   C5 잡음 느린 실행 (chain·ROS)
   C6 좁은 통로 최소 여유·실패
   C7 튜닝 순위가 held-out 순위를 예측했는가 (튜닝 J 대 held-out 시간)
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
sys.path.insert(0, HERE)
import analyze_x as ax  # noqa: E402
import posthoc_x as ph  # noqa: E402

ROUNDS = {
    'r1': {'stage': '', 'paths': 'paths', 'sel': 'tuning/selection.json', 'R': 'dwa_R',
           'tun': 'tuning/raw'},
    'r2': {'stage': 'r2/', 'paths': 'r2/paths_c', 'sel': 'r2/tuning/selection_r2.json',
           'R': 'dwa_R2', 'tun': 'r2/tuning/raw'},
}


def load_stage(R, rel):
    d = os.path.join(R, rel, 'raw')
    return ax.load_dir(d) if os.path.isdir(d) else []


def sel_of(R, rnd):
    fn = os.path.join(R, ROUNDS[rnd]['sel'])
    if not os.path.exists(fn):
        return {}
    with open(fn, encoding='utf-8') as f:
        return json.load(f)


def paths_of(R, rnd):
    out = {}
    for fn in ('heldout_smooth.json', 'heldout_narrow.json', 'heldout_astar.json'):
        p = os.path.join(R, ROUNDS[rnd]['paths'], fn)
        if os.path.exists(p):
            with open(p, encoding='utf-8') as f:
                for e in json.load(f):
                    xy = np.array([[q[0], q[1]] for q in e['points']])
                    s, lab = ph.labels(xy)
                    out[e['id']] = (e['family'], xy, s, lab, e['points'][-1])
    return out


def zones(R, rnd, stage, arms):
    """C1 궤적 → (팔, 경로) → (가족, t_reach, 구간 시간)."""
    P = paths_of(R, rnd)
    root = os.path.join(R, ROUNDS[rnd]['stage'] + stage)
    out = {}
    for fn in glob.glob(os.path.join(root, 'raw', '*.jsonl')):
        jid = os.path.basename(fn)[:-6]
        with open(fn, encoding='utf-8') as f:
            for line in f:
                r = json.loads(line)
                if not r['success'] or r['arm'] not in arms or r['scene'] not in P:
                    continue
                tr = os.path.join(root, 'traces', jid,
                                  f"{r['arm']}__{r['scene']}__s{r['seed']}__r{r['rep']}.csv")
                if not os.path.exists(tr):
                    continue
                fam, xy, s, lab, _ = P[r['scene']]
                zt = ph.zone_times(tr, r['t_reach'], xy, s, lab, r.get('t_first_entry', -1.0))
                if zt is not None:
                    out[(r['arm'], r['scene'])] = (fam, r['t_reach'], zt)
    return out


def decomp_table(Z, a, b, title):
    lines = [f'#### {title}: {a} − {b}', '',
             '| 경로 | n | 전체 [%] | ' + ' | '.join(ph.ZNAME[z] for z in ph.ZONES) + ' |',
             '| --- | --- | --- |' + ' --- |' * len(ph.ZONES)]
    for fname, fams in ph.FAMS:
        common = [sc for (arm, sc) in Z if arm == a and (b, sc) in Z and Z[(a, sc)][0] in fams]
        if not common:
            continue
        tot = [100 * (Z[(a, sc)][1] - Z[(b, sc)][1]) / Z[(b, sc)][1] for sc in common]
        cells = [ph.fmt(*ph.boot([100 * (Z[(a, sc)][2][z] - Z[(b, sc)][2][z]) / Z[(b, sc)][1]
                                  for sc in common])) for z in ph.ZONES]
        lines.append(f'| {fname} | {len(common)} | {ph.fmt(*ph.boot(tot))} | ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines) + '\n'


def entry_yaw(R, rnd, stage, arms):
    """첫 진입 순간 yaw 오차 (참값) 와 판정 대기."""
    P = paths_of(R, rnd)
    root = os.path.join(R, ROUNDS[rnd]['stage'] + stage)
    res = collections.defaultdict(list)
    for fn in glob.glob(os.path.join(root, 'raw', '*.jsonl')):
        jid = os.path.basename(fn)[:-6]
        with open(fn, encoding='utf-8') as f:
            for line in f:
                r = json.loads(line)
                if r['arm'] not in arms or not r['success'] or r.get('t_first_entry', -1) < 0 or \
                        r['scene'] not in P:
                    continue
                tr = os.path.join(root, 'traces', jid,
                                  f"{r['arm']}__{r['scene']}__s{r['seed']}__r{r['rep']}.csv")
                if not os.path.exists(tr):
                    continue
                gth = P[r['scene']][4][2]
                row = None
                with open(tr, encoding='utf-8') as tf:
                    for x in csv.DictReader(tf):
                        if x['x'] != '' and float(x['t']) >= r['t_first_entry'] - 1e-6:
                            row = x
                            break
                if row is None:
                    continue
                ye = abs((float(row['yaw']) - gth + math.pi) % (2 * math.pi) - math.pi)
                res[r['arm']].append((P[r['scene']][0], ye, r['t_reach'] - r['t_first_entry']))
    lines = [f'#### 첫 진입 yaw 오차 — {rnd} {stage}', '',
             '| 팔 | 성공 | yaw 오차 중앙 [rad] | > 0.05 rad (매끈/통로/A*) | 판정 대기 중앙 · p90 [s] | > 0.05 일 때 판정 대기 중앙 [s] |',
             '| --- | --- | --- | --- | --- | --- |']
    for arm in arms:
        v = res.get(arm, [])
        if not v:
            continue
        ye = np.array([x[1] for x in v])
        w = np.array([x[2] for x in v])
        by = collections.Counter(x[0] for x in v if x[1] > 0.05)
        lines.append(f'| {arm} | {len(v)} | {np.median(ye):.3f} | {int((ye > 0.05).sum())} '
                     f'({by["smooth"]}/{by["narrow"]}/{by["astar"]}) | {np.median(w):.2f} · '
                     f'{np.percentile(w, 90):.2f} | '
                     f'{(np.median(w[ye > 0.05]) if (ye > 0.05).any() else float("nan")):.2f} |')
    return '\n'.join(lines) + '\n'


def accel_table(recs_by_label, arms):
    lines = ['#### 0→0.9 m/s 시간 (ROS 잡음 없음, 경로 중앙 [p10, p90] s)', '',
             '| 팔 | ' + ' | '.join(recs_by_label) + ' |', '| --- |' + ' --- |' * len(recs_by_label)]
    for arm in arms:
        cells = []
        for lab, recs in recs_by_label.items():
            v = [r['t_v90'] for r in recs if r['arm'] == arm and r.get('t_v90', -1) > 0]
            cells.append(f'{np.median(v):.2f} [{np.percentile(v, 10):.2f}, {np.percentile(v, 90):.2f}]'
                         if v else '–')
        lines.append(f'| {arm} | ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines) + '\n'


def o_table(recs_by_label, arms):
    lines = ['#### O 장면 결과 원인 (실행 수)', '',
             '| 라운드 | 팔 | 성공 | 충돌 | 장애물 앞 정지 | 시간 초과 | 그 밖 | 끝 이격 중앙 [cm] | 진행률 중앙 | CTE 최대 중앙 [cm] |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for lab, recs in recs_by_label.items():
        for arm in arms:
            rs = [r for r in recs if r['arm'] == arm]
            if not rs:
                continue
            c = collections.Counter(ax.o_cause(r) for r in rs)
            lines.append(f'| {lab} | {arm} | {c["success"]}/{len(rs)} | {c["collision"]} | {c["blocked"]} | '
                         f'{c["timeout"]} | {c["aborted"] + c["other"] + c["rejected"]} | '
                         f'{100 * np.median([r["obs_end_clear"] for r in rs]):.1f} | '
                         f'{np.median([r["progress_frac"] for r in rs]):.2f} | '
                         f'{100 * np.median([r["cte_max"] for r in rs]):.1f} |')
    return '\n'.join(lines) + '\n'


def tuning_vs_heldout(R, rnd, c1, fams):
    """튜닝 순위(J' 또는 J'')가 held-out C1 시간을 예측했는가: 격자 안 설정의 순위와 held-out 시간 (선택 설정만 held-out 에 있음)."""
    sel = sel_of(R, rnd)
    lines = [f'#### 튜닝 선택과 held-out ({rnd})', '',
             '| 격자 | 선택 | 튜닝 1·2 위 점수 | held-out C1 선택 팔 T/T_ref 중앙 | held-out 성공 |',
             '| --- | --- | --- | --- | --- |']
    for fam in fams:
        if fam not in sel:
            continue
        rk = sel[fam]['ranking']
        key = 'J_prime' if 'J_prime' in rk[0] else 'J2'
        arm = sel[fam]['chosen']
        name = {'DWA_R': 'dwa_R', 'DWA_R2': 'dwa_R2', 'TB': 'dwa_textbook_tuned'}.get(fam, arm)
        tr = [r['t_reach'] / ax.t_lb(r['length']) for r in c1 if r['arm'] == name and r['success']]
        ok = sum(r['success'] for r in c1 if r['arm'] == name)
        n = sum(1 for r in c1 if r['arm'] == name)
        lines.append(f'| {fam} | {arm} | {rk[0][key]:.3f} / {rk[1][key]:.3f} | '
                     f'{np.median(tr) if tr else float("nan"):.3f} | {ok}/{n} |')
    return '\n'.join(lines) + '\n'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    R = args.runs
    md = ['# 1·2 라운드 원자료 전수 분석 (prereg_r2 §6 — 3 라운드 진입 판단 자료)', '',
          '**기술·사후 분석이다.** 판정은 prereg_r2 §5 의 H4·H5·H6 만이고, 여기 표는 진입 조건과 원인 후보를 보는 근거다.', '']
    # A. 목록
    md += ['## A. 원자료 목록', '', '| 단계 | 실행 | 명령 주기 최소 [Hz] | 18 Hz 미만 |', '| --- | --- | --- | --- |']
    for rel in ('tuning', 'c1', 'c2', 'c3', 'c4', 'o', 'm', 'm_clr', 'r2/tuning', 'r2/c1', 'r2/c2',
                'r2/c3', 'r2/o'):
        rs = load_stage(R, rel)
        if rs:
            md.append(f'| {rel} | {len(rs)} | {min(r["cmd_rate_hz"] for r in rs):.2f} | '
                      f'{sum(1 for r in rs if r["cmd_rate_hz"] < 18)} |')
    for rel in ('chain', 'r2/chain'):
        d = os.path.join(R, rel, 'raw')
        if os.path.isdir(d):
            md.append(f'| {rel} | {len(ax.load_chain(d))} | (결정적) | – |')
    md.append('')
    # B. 진입 조건
    md += ['## B. 진입 조건 (prereg_r2 §6)', '']
    hyp_fn = os.path.join(args.out, 'hypotheses_r2.json')
    if os.path.exists(hyp_fn):
        with open(hyp_fn, encoding='utf-8') as f:
            hyp = json.load(f)
        for k in ('H4', 'H5', 'H6'):
            if k in hyp:
                v = hyp[k]
                val = v.get('time_ratio_pct') or [v['mean'], *v['ci95']]
                md.append(f'- **{k}**: {v["decision"]} — {", ".join(f"{x:+.3f}" for x in val)} ({v["rule"]})')
    sel2 = sel_of(R, 'r2')
    c1r2 = ax.substitute_tuned(load_stage(R, 'r2/c1'), sel2) if sel2 else []
    c2r2 = ax.substitute_tuned(load_stage(R, 'r2/c2'), sel2) if sel2 else []
    c3r2 = ax.substitute_tuned(load_stage(R, 'r2/c3'), sel2) if sel2 else []
    for lab, recs in (('C1', c1r2), ('C2', c2r2), ('C3', c3r2)):
        if not recs:
            continue
        d_t = ax.paired(recs, 'dwa_R2', 'dwa_full', 't_reach', ax.logratio)
        d_s = ax.paired(recs, 'dwa_R2', 'dwa_full', 'success', only_success=False)
        d_e = ax.paired(recs, 'dwa_R2', 'dwa_full', 'stop_err')
        d_c = ax.paired(recs, 'dwa_R2', 'dwa_full', 'cte_curve_mean')
        md.append(f'- {lab} DWA-R2 − dwa_full: 시간 {ax.fmt_pct(*ax.boot(d_t.values()))} % (n {len(d_t)}), '
                  f'성공률 {ax.fmt_ci(*ax.boot(d_s.values()), 100)} %p, |정지 오차| '
                  f'{ax.fmt_ci(*ax.boot(d_e.values()), 100)} cm, 곡선 CTE {ax.fmt_ci(*ax.boot(d_c.values()), 100)} cm')
    md.append('')
    # C. 원인 후보
    md += ['## C. 원인 후보 (사후)', '', '### C1. 시간 차의 구간 분해 (ROS C1 궤적)', '']
    sel1 = sel_of(R, 'r1')
    Z1 = zones(R, 'r1', 'c1', ('dwa_R', 'dwa_full', 'RPP_R00', 'pp'))
    md.append(decomp_table(Z1, 'dwa_R', 'RPP_R00', '1 라운드 (held-out B)'))
    md.append(decomp_table(Z1, 'pp', 'RPP_R00', '1 라운드 (held-out B)'))
    if sel2:
        rpp2 = sel2['RPP']['chosen']
        Z2 = zones(R, 'r2', 'c1', ('dwa_R2', 'dwa_R', 'dwa_full', rpp2, 'pp', 'RPP_R00'))
        md.append(decomp_table(Z2, 'dwa_R2', rpp2, '2 라운드 (held-out C)'))
        md.append(decomp_table(Z2, 'dwa_R2', 'dwa_full', '2 라운드 (held-out C)'))
        md.append(decomp_table(Z2, 'dwa_R2', 'dwa_R', '2 라운드 (held-out C)'))
        md.append(decomp_table(Z2, 'dwa_full', rpp2, '2 라운드 (held-out C)'))
    md += ['### C2. 목표 첫 진입 yaw 오차와 판정 대기', '']
    md.append(entry_yaw(R, 'r1', 'c1', ('dwa_full', 'dwa_R', 'RPP_R00', 'pp')))
    if sel2:
        md.append(entry_yaw(R, 'r2', 'c1', ('dwa_full', 'dwa_R', 'dwa_R2', sel2['RPP']['chosen'], 'pp')))
    md += ['### C3. 출발 가속', '']
    c1r1 = ax.substitute_tuned(load_stage(R, 'c1'), sel1)
    lab = {'1R C1': c1r1}
    if c1r2:
        lab['2R C1'] = c1r2
    md.append(accel_table(lab, ['dwa_full', 'dwa_R', 'dwa_R2', 'RPP_R00', 'pp', 'DWB_tuned']))
    for rel, arms in (('chain', ['dwa_full', 'dwa_R']), ('r2/chain', ['dwa_full', 'dwa_R', 'dwa_R2'])):
        d = os.path.join(R, rel, 'raw')
        if os.path.isdir(d):
            md.append(f'{rel}:\n\n' + ax.reset_counts(ax.load_chain(d), os.path.join(R, rel), arms))
    md += ['### C4. 정적 장애물 O', '']
    ol = {'1R': ax.substitute_tuned(load_stage(R, 'o'), sel1)}
    if sel2:
        ol['2R'] = ax.substitute_tuned(load_stage(R, 'r2/o'), sel2)
    md.append(o_table(ol, ['dwa_full', 'dwa_R', 'dwa_R2', 'MPPI_tuned', 'RPP_tuned', 'DWB_tuned', 'pp',
                           'dwa_textbook_tuned']))
    md += ['### C5. 잡음 느린 실행', '']
    for rel, arms in (('chain', ['dwa_full', 'dwa_R']), ('r2/chain', ['dwa_full', 'dwa_R', 'dwa_R2'])):
        d = os.path.join(R, rel, 'raw')
        if os.path.isdir(d):
            md.append(f'{rel}:\n\n' + ax.slow_runs(ax.load_chain(d), arms, 0))
    for lab2, recs, ref in (('1R C3', ax.substitute_tuned(load_stage(R, 'c3'), sel1), 'c1'),
                            ('2R C3', c3r2, 'r2/c1')):
        if not recs:
            continue
        clean = {(r['arm'], r['scene']): r for r in load_stage(R, ref)}
        md.append(f'#### ROS 느린 실행 ({lab2}, 같은 팔 잡음 없음 대비 +30 % 초과 또는 실패)\n')
        md.append('| 팔 | 실행 | 느린 | 실패 |\n| --- | --- | --- | --- |')
        for arm in ('dwa_full', 'dwa_x_X00', 'dwa_R', 'dwa_R2', 'RPP_R00', 'pp'):
            rs = [r for r in recs if r['arm'] == arm]
            if not rs:
                continue
            refarm = 'dwa_full' if arm == 'dwa_x_X00' else arm
            slow = sum(1 for r in rs if r['success'] and (refarm, r['scene']) in clean and
                       clean[(refarm, r['scene'])]['success'] and
                       r['t_reach'] / clean[(refarm, r['scene'])]['t_reach'] > 1.30)
            md.append(f'| {arm} | {len(rs)} | {slow} | {sum(1 for r in rs if not r["success"])} |')
        md.append('')
    md += ['### C6. 좁은 통로', '', '| 라운드·조건 | 팔 | 성공 | 참값 최소 여유 최솟값 / 중앙 [cm] |', '| --- | --- | --- | --- |']
    for lab3, recs in (('1R C1', c1r1), ('1R C2', ax.substitute_tuned(load_stage(R, 'c2'), sel1)),
                       ('2R C1', c1r2), ('2R C2', c2r2)):
        for arm in ('dwa_full', 'dwa_R', 'dwa_R2', 'RPP_R00', 'MPPI_tuned', 'pp'):
            rs = [r for r in recs if r['arm'] == arm and r['family'] == 'narrow']
            if rs:
                mc = [r['min_clear'] for r in rs]
                md.append(f'| {lab3} | {arm} | {sum(r["success"] for r in rs)}/{len(rs)} | '
                          f'{100 * min(mc):.1f} / {100 * np.median(mc):.1f} |')
    md.append('')
    md += ['### C7. 튜닝 선택과 held-out', '']
    md.append(tuning_vs_heldout(R, 'r1', c1r1, ('DWB', 'MPPI', 'RPP', 'DWA_R', 'TB')))
    if sel2:
        md.append(tuning_vs_heldout(R, 'r2', c1r2, ('DWB', 'MPPI', 'RPP', 'DWA_R2', 'TB')))
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'r1r2_full_tables.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(md) + '\n')
    print('\n'.join(md))


if __name__ == '__main__':
    main()
