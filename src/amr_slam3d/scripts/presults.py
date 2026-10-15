#!/usr/bin/env python3
"""
P 단계 결과 표 그리기 (표준 라이브러리만) — peval.py 의 출력(json·jsonl)을 결과 문서용 마크다운 표로 옮긴다.

새 통계는 내지 않는다: 판정·구간·p 는 peval.py compare / compare-loc / compare-abl 이 낸 값을 그대로 옮기고, 이 파일이
더하는 것은 표 모양과 아래 두 가지 셈뿐이다.
- E4 "바꾼다" (사전 등록): 4 bag 중 ≥ 3 이 같은 방향 `differs`. 사전 등록 3 bag(s1_seed2x 제외) 표시는 3 bag 모두 같은
  방향 `differs` (비율 ≥ 3/4 를 3 bag 에 옮긴 것 — E4 결과 전 2026-10-13 노트 항목에 적은 해석).
- 보충(사전 등록 외) 누락 분해: 처리 스캔 판정 누락 스탬프가 그 bag 의 첫 점군 스탬프 + 1.0 s 안이면 "시작", 아니면 "주행 중".

    presults.py --ana R/analysis --rec <b2-rec rec 루트> --out R/analysis/tables
"""

import argparse
import json
import math
import os
import statistics

MAIN = ('slam_toolbox', 'rtabmap2d_noimu', 'rtabmap3d_noimu', 'liosam_planar')
REF = ('liosam_loop_re',)
AUX = ('liosam_oracle',)
BAGS = ('s3_aisle_a1', 's1_seed1_a1', 's1_seed3_a2', 's1_seed2x_a1', 's6_open_a3',
        's6_queue_a1', 's6_wall_a1', 'map_long_a1')
LOC_ROWS = ('amcl_gt', 'rtabmap3d_loc_gt', 'amcl_slam', 'rtabmap3d_loc_slam')
VERDICT_KO = {'A<B': 'A<B (A 가 낫다)', 'A>B': 'A>B (B 가 낫다)',
              'equiv': 'equiv (차이 δ 이하로 검출 안 됨)', 'undecided': 'undecided (가르지 못함)'}
STARTUP_S = 1.0


def f(v, nd=2):
    """숫자 → 문자열 (없거나 NaN 이면 —)."""
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return '—'
    if isinstance(v, bool):
        return '예' if v else '아니오'
    if isinstance(v, int):
        return str(v)
    return f'{v:.{nd}f}'


def load_jsonl(p):
    """줄마다 JSON 인 파일 → 목록 (없으면 빈 목록)."""
    if not os.path.isfile(p):
        return []
    with open(p, encoding='utf-8') as fh:
        return [json.loads(x) for x in fh if x.strip()]


def load_json(p):
    """JSON 파일 (없으면 None)."""
    if not os.path.isfile(p):
        return None
    with open(p, encoding='utf-8') as fh:
        return json.load(fh)


def val(r, k):
    """재생 기록의 수치 (없으면 NaN)."""
    v = r.get(k)
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else math.nan


def fam_table(ft):
    """가족 검정 표 (peval family_tests 출력)."""
    out = [f"bag {len(ft['bags'])} 개 ({', '.join(ft['bags'])})", '',
           '| 가족 | 비교 (A − B) | 지표 | bag 차 평균 [두 단계 부트스트랩 95 %] | 음/양 bag | Wilcoxon p | '
           '가능 최소 p | Holm p | 기각 가능 설계 | 판정 |', '|' + '---|' * 10]
    for fam, rows in ft['families'].items():
        for r in rows:
            b = r['boot']
            out.append(f"| {fam} | {r['name']} `{r['A']}` − `{r['B']}` | {r['metric']} | "
                       f"{f(b['mean'], 3)} [{f(b['lo'], 3)}, {f(b['hi'], 3)}] | "
                       f"{r['n_neg']}/{r['n_pos']} | {f(r['wilcoxon']['p'], 4)} | "
                       f"{f(r['min_p_possible'], 4)} | {f(r['p_holm'], 4)} | "
                       f"{'예' if r['rejectable'] else '아니오'} | {VERDICT_KO[r['verdict']]} |")
    out += ['', 'bag 별 차 (A − B):', '', '| 비교 | ' + ' | '.join(ft['bags']) + ' |',
            '|' + '---|' * (len(ft['bags']) + 1)]
    for fam, rows in ft['families'].items():
        for r in rows:
            cells = [f(r['bag_diffs'].get(b), 3) for b in ft['bags']]
            out.append(f"| {r['name']} | " + ' | '.join(cells) + ' |')
    return out


def row_table(rows, metrics, order):
    """행별 bag 평균의 평균 (재생 간 sd / bag 간 sd) 와 실패."""
    head = '| 행 | n | 실패 [Wilson 95 %] | ' + ' | '.join(m for m, _ in metrics) + ' |'
    out = [head, '|' + '---|' * (len(metrics) + 3)]
    for row in order:
        e = rows.get(row)
        if not e:
            continue
        cells = []
        for m, nd in metrics:
            x = e['metrics'].get(m)
            cells.append('—' if not x else
                         f"{f(x['mean_of_bags'], nd)} "
                         f"({f(x['within_sd'], nd)} / {f(x['between_sd'], nd)})")
        lo, hi = e['fail_wilson']
        out.append(f"| `{row}` | {e['n']} | {e['fail']} [{lo:.2f}, {hi:.2f}] | "
                   + ' | '.join(cells) + ' |')
    return out


def per_bag(recs, metric, order, nd=2):
    """행 × bag: 재생 값 (실패는 *)."""
    bags = [b for b in BAGS if any(r['bag'] == b for r in recs)]
    out = ['| 행 | ' + ' | '.join(bags) + ' |', '|' + '---|' * (len(bags) + 1)]
    for row in order:
        rr = [r for r in recs if r['row'] == row]
        if not rr:
            continue
        cells = []
        for b in bags:
            xs = sorted((r for r in rr if r['bag'] == b), key=lambda r: r['replay'])
            cells.append(' / '.join(f(val(r, metric), nd) + ('*' if r['fail'] else '') for r in xs)
                         or '—')
        out.append(f'| `{row}` | ' + ' | '.join(cells) + ' |')
    return out


def first_scan(rec_root, bag):
    """그 bag 의 첫 점군 스탬프 (gate.json, 없으면 None)."""
    g = load_json(os.path.join(rec_root, bag, 'gate.json'))
    if not g:
        return None
    t = g['topics']
    cands = [t[k]['first_s'] for k in t if k.endswith(('lidar3d/points', 'scan_filtered'))]
    return min(cands) if cands else None


def failures(recs, order, rec_root):
    """행별 실패 사유와 보충 분해 (시작 / 주행 중 누락)."""
    out = ['| 행 | n | 실패 | 사유 (재생 수) | 보충: 누락 재생 중 시작 장만 / 주행 중 포함 |', '|---|---|---|---|---|']
    firsts = {b: first_scan(rec_root, b) for b in BAGS}
    for row in order:
        rr = [r for r in recs if r['row'] == row]
        if not rr:
            continue
        reasons = {}
        for r in rr:
            for w in r['fail']:
                k = w.split('=')[0]
                reasons[k] = reasons.get(k, 0) + 1
        start_only = moving = 0
        for r in rr:
            st = r.get('missing_stamps') or []
            if not r.get('missing') or not st:
                continue
            f0 = firsts.get(r['bag'])
            if f0 is not None and all(s <= f0 + STARTUP_S for s in st):
                start_only += 1
            else:
                moving += 1
        out.append(f"| `{row}` | {len(rr)} | {sum(1 for r in rr if r['fail'])} | "
                   + (', '.join(f'{k} {v}' for k, v in sorted(reasons.items())) or '—')
                   + f' | {start_only} / {moving} |')
    return out


def e1_table(recs, rows3d):
    """E1: 3D 행이 bag 마다 끝까지 도는가 — rc 0 수, 덮음 최소, 누락 (재생별)."""
    out = ['| 행 | bag | 재생 | rc 0 | 덮음 최소 | 처리 판정 누락 (재생별) | 런치 종료 |',
           '|---|---|---|---|---|---|---|']
    for row in rows3d:
        for b in BAGS:
            rr = sorted((r for r in recs if r['row'] == row and r['bag'] == b),
                        key=lambda r: r['replay'])
            if not rr:
                continue
            cov = min(val(r, 'coverage') for r in rr)
            ex = sorted({str(r.get('launch_exit') or '—').split(' after')[0] for r in rr})
            out.append(f"| `{row}` | {b} | {len(rr)} | {sum(1 for r in rr if r.get('rc') == 0)} | "
                       f"{f(cov, 3)} | {' / '.join(f(r.get('missing'), 0) for r in rr)} | "
                       f"{', '.join(ex)} |")
    return out


def latency(recs, order):
    """RTAB-Map 행 스캔당 지연 (재생 p50·p95 의 평균, 최대의 최대) — 공유 부하 하 예비값."""
    out = ['| 행 | 재생 | icp p50 ms | icp p95 ms | icp max ms | rtabmap p50 ms | rtabmap p95 ms |',
           '|---|---|---|---|---|---|---|']
    for row in order:
        rr = [r for r in recs if r['row'] == row and r.get('lat_icp')]
        if not rr:
            continue

        def m(part, k):
            xs = [r[part].get(k) for r in rr if r.get(part) and r[part].get(k) is not None]
            xs = [x for x in xs if math.isfinite(x)]
            return statistics.mean(xs) if xs else math.nan
        mx = max((r['lat_icp'].get('max_ms') or 0.0) for r in rr)
        out.append(f"| `{row}` | {len(rr)} | {f(m('lat_icp', 'p50_ms'), 1)} | "
                   f"{f(m('lat_icp', 'p95_ms'), 1)} | {f(mx, 0)} | "
                   f"{f(m('lat_rtabmap', 'p50_ms'), 1)} | "
                   f"{f(m('lat_rtabmap', 'p95_ms'), 1)} |")
    return out


def e5_table(e5, rows, did_key='did'):
    """E5 표: 행마다 open/queue/wall 재생 값, 차, 판정, 차의 차."""
    out = ['| 행 | open (재생) | queue (재생) | wall (재생) | queue − open | 판정 | wall − open | 판정 |',
           '|---|---|---|---|---|---|---|---|']
    for row in rows:
        e = e5['per_row'].get(row)
        if not e:
            continue
        v = e['values']
        out.append(f"| `{row}` | {' / '.join(f(x) for x in v.get('open', []))} | "
                   f"{' / '.join(f(x) for x in v.get('queue', []))} | "
                   f"{' / '.join(f(x) for x in v.get('wall', []))} | "
                   f"{f(e['queue_minus_open'], 3)} | "
                   f"{e['queue_rule']} | {f(e['wall_minus_open'], 3)} | {e['wall_rule']} |")
    out += ['', '| 차의 차 (2D − 3D) | 값 [cm] |', '|---|---|']
    for k, x in e5[did_key].items():
        out.append(f'| {k} | {f(x, 3)} |')
    return out


def seg_table(seg):
    """E3(b) 명세 구간 표."""
    out = ['| 행 | 구간 (기준) | RMSE 평균 / 최악 | max 평균 / 최악 | p95 평균 / 최악 | 통과 재생 RMSE / max / p95 |',
           '|---|---|---|---|---|---|']
    for row, ent in seg.items():
        for s, e in ent.items():
            n = e['n']
            out.append(f"| `{row}` | {s} ({e['limit_cm']:g} cm) | "
                       f"{f(e['rmse'])} / {f(e['rmse_worst'])} | "
                       f"{f(e['max'])} / {f(e['max_worst'])} | "
                       f"{f(e['p95'])} / {f(e['p95_worst'])} | "
                       f"{e['rmse_pass']}/{n} · {e['max_pass']}/{n} · {e['p95_pass']}/{n} |")
    return out


def abl_tables(abl):
    """E4: 손잡이 × 지표 — 평균 차, bag 별 (차, 판정), 4 bag·3 bag 판정."""
    out = []
    pre = ('s3_aisle_a1', 's1_seed1_a1', 's1_seed3_a2')
    for row, ent in abl['rows'].items():
        out += [f"#### `{row}` (재생 {ent['n']}, 실패 {ent['fail']}"
                + (f" — {', '.join(ent['fail_reasons'])}" if ent['fail_reasons'] else '')
                + ')', '',
                '| 지표 (δ) | 평균 차 (절제 − 기준) | ' + ' | '.join(abl['bags'])
                + ' | 4 bag 판정 | 사전 등록 3 bag 판정 |', '|' + '---|' * (len(abl['bags']) + 4)]
        for m, e in ent['metrics'].items():
            per = e['per_bag']
            cells = []
            for b in abl['bags']:
                p = per.get(b)
                mark = ' ●' if p and p['rule'] == 'differs' else ''
                cells.append('—' if not p else f"{f(p['diff'], 3)}{mark}")

            def verdict(bags, need):
                ups = sum(1 for b in bags if b in per and per[b]['rule'] == 'differs'
                          and per[b]['diff'] > 0)
                dns = sum(1 for b in bags if b in per and per[b]['rule'] == 'differs'
                          and per[b]['diff'] < 0)
                if ups >= need:
                    return f'바꾼다 (+, {ups}/{len(bags)})'
                if dns >= need:
                    return f'바꾼다 (−, {dns}/{len(bags)})'
                return f'가르지 못함 (+{ups}/−{dns})'
            out.append(f"| {m} ({e['delta']:g}) | {f(e['mean_diff'], 3)} | " + ' | '.join(cells)
                       + f" | {verdict(abl['bags'], 3)} | {verdict(pre, 3)} |")
        out.append('')
    out.append('● = 그 bag 에서 `replay_rule` differs (평균 차 > max(2 × 합동 재생 sd, δ)).')
    return out


def main(argv=None):
    """명령줄."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--ana', required=True)
    ap.add_argument('--rec', default='/home/CAPYI_AI/runs/b24/b2-rec/rec')
    ap.add_argument('--out', required=True)
    ns = ap.parse_args(argv)
    os.makedirs(ns.out, exist_ok=True)
    order = MAIN + REF + AUX
    mtr = (('ate_rmse_cm', 2), ('ate_max_cm', 2), ('ate_p95_cm', 2), ('ate_se2_rmse_cm', 2),
           ('yaw_rmse_deg', 3), ('rpe1_pct', 2), ('rpe5_pct', 3), ('rpe10_pct', 3))
    mmap = (('map_adnn_surf_cm', 2), ('map_prec_surf', 3), ('map_recall_surf', 3), ('map_f1', 3),
            ('map_iou', 3), ('cloud_f5', 3), ('cloud_f10', 3), ('cloud_prec5', 3),
            ('cloud_recall5', 3), ('z_max_cm', 2))
    e2 = load_json(os.path.join(ns.ana, 'e2', 'compare.json'))
    r2 = load_jsonl(os.path.join(ns.ana, 'e2', 'replays.jsonl'))
    if e2:
        L = ['# E2·E3(a)·E5(매핑) 표 (presults.py)', '']
        for key in ('ALL8', 'PRE7', 'S1_3', 'S1_2'):
            L += [f'## 가족 검정 {key}', ''] + fam_table(e2['tests'][key]) + ['']
        for tag, rows in (('8 bag', e2['rows']), ('사전 등록 7 bag', e2['rows_pre7'])):
            L += [f'## 행별 궤적 ({tag}) — bag 평균의 평균 (재생 간 sd / bag 간 sd)', '']
            L += row_table(rows, mtr, order) + ['']
            L += [f'## 행별 지도 ({tag})', ''] + row_table(rows, mmap, order) + ['']
        for m, nd in (('ate_rmse_cm', 2), ('ate_max_cm', 2), ('map_adnn_surf_cm', 2),
                      ('map_iou', 3), ('cloud_f5', 3), ('rpe10_pct', 3)):
            L += [f'## bag × 행 — {m} (재생 r0 / r1 / r2, * = 실패 재생)', '']
            L += per_bag(r2, m, order, nd) + ['']
        L += ['## 실패', ''] + failures(r2, order, ns.rec) + ['']
        L += ['## E1 — 3D 행이 끝까지 도는가', ''] + e1_table(
            r2, ('rtabmap3d_noimu', 'liosam_planar', 'liosam_loop_re', 'liosam_oracle')) + ['']
        L += ['## 스캔당 지연 (공유 부하 하 예비값)', ''] + latency(r2, order) + ['']
        L += ['## E5 (매핑) — 통로 축 RMSE [cm]', ''] + e5_table(e2['e5'], MAIN) + ['']
        with open(os.path.join(ns.out, 'e2.md'), 'w', encoding='utf-8') as fh:
            fh.write('\n'.join(L) + '\n')
    e3 = load_json(os.path.join(ns.ana, 'e3', 'compare_loc.json'))
    r3 = load_jsonl(os.path.join(ns.ana, 'e3', 'replays.jsonl'))
    if e3:
        L = ['# E3(b)·E5(위치추정) 표 (presults.py)', '']
        for key in ('ALL7', 'PRE6', 'S1_3', 'S1_2'):
            L += [f'## 가족 검정 {key}', ''] + fam_table(e3['tests'][key]) + ['']
        for tag, rows in (('7 bag', e3['rows']), ('사전 등록 6 bag', e3['rows_pre6'])):
            L += [f'## 행별 궤적 ({tag})', ''] + row_table(rows, mtr, LOC_ROWS) + ['']
        for m in ('ate_rmse_cm', 'ate_max_cm'):
            L += [f'## bag × 행 — {m}', ''] + per_bag(r3, m, LOC_ROWS) + ['']
        L += ['## 명세 구간', ''] + seg_table(e3['segments']) + ['']
        L += ['## 위치추정 실패', '', '| 행 | n | 실패 [Wilson 95 %] | 발산 | 사유 |', '|---|---|---|---|---|']
        for row, e in e3['failures'].items():
            lo, hi = e['wilson']
            why = ', '.join(f'{k} {v}' for k, v in sorted(e['reasons'].items())) or '—'
            L.append(f"| `{row}` | {e['n']} | {e['fail']} [{lo:.2f}, {hi:.2f}] | "
                     f"{e['diverged']} | {why} |")
        lost = load_jsonl(os.path.join(ns.ana, 'e3', 'lost.jsonl'))
        if lost:
            L += ['', '배포 2D kidnap_monitor (`loc_lost.py`):', '',
                  '| 행 | 재생 | lost 메시지 | LOST 사건 합 | 사건 > 0 재생 | 마지막 상태 |',
                  '|---|---|---|---|---|---|']
            for row in ('amcl_gt', 'amcl_slam'):
                rr = [x for x in lost if x['row'] == row]
                if not rr:
                    continue
                ev = [x['lost_events'] for x in rr if x['lost_events'] is not None]
                st = sorted({str(x['status_last_state']) for x in rr})
                L.append(f"| `{row}` | {len(rr)} | {sum(x['lost_msgs'] or 0 for x in rr)} | "
                         f"{sum(ev) if ev else '미측정'} | {sum(1 for x in ev if x > 0)} | "
                         f"{', '.join(st)} |")
        L += ['', '## 지도의 몫 (SLAM 지도 − 정답 지도, xy ATE RMSE cm)', '', '| 사슬 | 평균 [95 %] | bag 별 |',
              '|---|---|---|']
        for k, e in e3['map_effect'].items():
            b = e['boot']
            L.append(f"| {k} | {f(b['mean'], 3)} [{f(b['lo'], 3)}, {f(b['hi'], 3)}] | "
                     + ', '.join(f'{bb} {f(v, 2)}' for bb, v in e['bag_diffs'].items()) + ' |')
        L += ['', '## E5 (위치추정) — 통로 축 RMSE [cm]', ''] + e5_table(e3['e5_loc'], LOC_ROWS)
        with open(os.path.join(ns.out, 'e3.md'), 'w', encoding='utf-8') as fh:
            fh.write('\n'.join(L) + '\n')
    e4 = load_json(os.path.join(ns.ana, 'e4', 'compare_abl.json'))
    r4 = load_jsonl(os.path.join(ns.ana, 'e4', 'replays.jsonl'))
    if e4:
        L = ['# E4 절제 표 (presults.py)', '',
             f"기준 `{e4['base']}` (E2 재생 3), bag {', '.join(e4['bags'])}", '']
        L += abl_tables(e4)
        abl_rows = sorted({r['row'] for r in r4})
        L += ['', '## 절제 행 실패', ''] + failures(r4, abl_rows, ns.rec)
        base = [r for r in r2 if r['row'] == 'rtabmap3d_noimu' and r['bag'] in e4['bags']]
        L += ['', '## 절제 행 지연 (공유 부하 하 예비값)', '']
        L += latency(r4 + base, ('rtabmap3d_noimu',) + tuple(abl_rows))
        with open(os.path.join(ns.out, 'e4.md'), 'w', encoding='utf-8') as fh:
            fh.write('\n'.join(L) + '\n')
    print(ns.out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
