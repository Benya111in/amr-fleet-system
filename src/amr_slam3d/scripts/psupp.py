#!/usr/bin/env python3
"""
P 단계 보충 분석 (사전 등록 외) — EKF 결함 bag 을 뺀 집합으로 같은 표를 낸다.

EKF 결함 bag = 녹화 때 `imu_filter_node` 가 "falling back to parameter bias" 를 낸 bag
(자이로 z 바이어스 0.01 rad/s 가 빠지지 않아 bag 의 EKF odom 이 yaw 표류) —
노트 항목 `2026-10-09_0000_b2-eval_mapping-compare-prereg.md` 「EKF 결함 bag」.
판정 규칙·지표·검정은 peval.py 의 사전 등록 함수를 그대로 부른다 (집합만 바꾼다). 결과는 **기술**이다:
E2 5 bag (가능 최소 p 0.0625), E3(b) 4 bag (0.125) 은 기각 불가능한 설계.

    psupp.py e2 --replays R/analysis/e2/replays.jsonl --out R/analysis/supp
    psupp.py e3 --replays R/analysis/e3/replays.jsonl --out R/analysis/supp
    psupp.py defect --rec <b2-rec rec 루트>          # bag 마다 녹화 로그의 바이어스 추정 줄
"""

import argparse
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

#: 녹화 launch.log 의 imu_filter_node 기동 바이어스 줄 (2026-10-14 확인)
DEFECT_BAGS = ('s1_seed3_a2', 's1_seed2x_a1', 's6_wall_a1')
FALLBACK = 'falling back to parameter bias'
ALL_BAGS = ('s3_aisle_a1', 's1_seed1_a1', 's1_seed3_a2', 's1_seed2x_a1', 's6_open_a3',
            's6_queue_a1', 's6_wall_a1', 'map_long_a1')
CLEAN_E2 = tuple(b for b in ALL_BAGS if b not in DEFECT_BAGS)
CLEAN_E3 = tuple(b for b in CLEAN_E2 if b != 'map_long_a1')


def bias_line(launch_log):
    """녹화 launch.log → (상태, 원문 줄). 상태 = fallback | estimated | partial | none."""
    try:
        text = open(launch_log, encoding='utf-8', errors='replace').read()
    except OSError:
        return 'none', ''
    for line in text.splitlines():
        if 'imu_filter_node' not in line:
            continue
        if FALLBACK in line:
            return 'fallback', line.strip()
        if 'startup bias estimated' in line:
            return 'estimated', line.strip()
        if 'samples collected before motion' in line:
            return 'partial', line.strip()
    return 'none', ''


def cmd_defect(ns):
    """Bag 마다 바이어스 추정 상태 (DEFECT_BAGS 와 맞는지 확인)."""
    out = {}
    for b in ALL_BAGS:
        st, line = bias_line(os.path.join(ns.rec, b, 'launch.log'))
        m = re.search(r'gyro_bias \[([^\]]*)\]', line)
        out[b] = {'state': st, 'gyro_bias': m.group(1) if m else None,
                  'defect_rule': st == 'fallback', 'in_defect_list': b in DEFECT_BAGS}
    print(json.dumps(out, ensure_ascii=False, indent=1))
    bad = [b for b, v in out.items() if v['defect_rule'] != v['in_defect_list']]
    return 1 if bad else 0


def cmd_e2(ns):
    """E2: 가족 T·M 과 행별 표를 결함 bag 제외 5 bag 으로."""
    import peval
    import presults
    recs = peval.load_replays(ns.replays)
    rows = sorted({r['row'] for r in recs})
    sub = [r for r in recs if r['bag'] in CLEAN_E2]
    res = {'bags': list(CLEAN_E2), 'excluded': list(DEFECT_BAGS),
           'tests': peval.family_tests(recs, CLEAN_E2, 'CLEAN5'),
           'rows': peval.per_row_tables(sub, rows)}
    os.makedirs(ns.out, exist_ok=True)
    with open(os.path.join(ns.out, 'e2_clean.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=1, default=float)
    order = presults.MAIN + presults.REF + presults.AUX
    lines = ['# 보충(사전 등록 외, 결과 뒤): E2 — EKF 결함 bag 제외 5 bag', '',
             f"제외: {', '.join(DEFECT_BAGS)}", ''] + presults.fam_table(res['tests'])
    lines += ['', '## 행별 (5 bag)', ''] + presults.row_table(
        res['rows'], (('ate_rmse_cm', 2), ('ate_max_cm', 2), ('ate_se2_rmse_cm', 2),
                      ('rpe1_pct', 2), ('rpe10_pct', 3), ('map_adnn_surf_cm', 2), ('map_iou', 3),
                      ('cloud_f5', 3)), order)
    with open(os.path.join(ns.out, 'e2_clean.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print(os.path.join(ns.out, 'e2_clean.md'))
    return 0


def cmd_e3(ns):
    """E3(b): 가족 L1·L2·행별·구간 표를 결함 bag 제외 4 bag 으로."""
    import peval
    import presults
    recs = peval.load_replays(ns.replays)
    sub = [r for r in recs if r['bag'] in CLEAN_E3]
    res = {'bags': list(CLEAN_E3), 'excluded': list(DEFECT_BAGS),
           'tests': peval.loc_tests(recs, CLEAN_E3, 'CLEAN4'),
           'rows': peval.per_row_tables(sub, sorted({r['row'] for r in sub})),
           'segments': peval.segment_table(sub), 'failures': peval.loc_failures(sub)}
    os.makedirs(ns.out, exist_ok=True)
    with open(os.path.join(ns.out, 'e3_clean.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, ensure_ascii=False, indent=1, default=float)
    lines = ['# 보충(사전 등록 외, E3(b) 결과 전 규칙): E3(b) — EKF 결함 bag 제외 4 bag', '',
             f"제외: {', '.join(DEFECT_BAGS)}", ''] + presults.fam_table(res['tests'])
    lines += ['', '## 행별 (4 bag)', ''] + presults.row_table(
        res['rows'], (('ate_rmse_cm', 2), ('ate_max_cm', 2), ('ate_p95_cm', 2),
                      ('ate_se2_rmse_cm', 2)), presults.LOC_ROWS)
    lines += ['', '## 명세 구간 (4 bag)', ''] + presults.seg_table(res['segments'])
    with open(os.path.join(ns.out, 'e3_clean.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print(os.path.join(ns.out, 'e3_clean.md'))
    return 0


def main(argv=None):
    """명령줄."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name in ('e2', 'e3'):
        s = sub.add_parser(name)
        s.add_argument('--replays', nargs='+', required=True)
        s.add_argument('--out', required=True)
    s = sub.add_parser('defect')
    s.add_argument('--rec', default='/home/CAPYI_AI/runs/b24/b2-rec/rec')
    ns = ap.parse_args(argv)
    return {'e2': cmd_e2, 'e3': cmd_e3, 'defect': cmd_defect}[ns.cmd](ns)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
