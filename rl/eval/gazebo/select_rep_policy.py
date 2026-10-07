#!/usr/bin/env python3
"""
단계 C 대표 정책을 b1-ppo10-eval 의 사전 등록 규칙으로 다시 고른다 (보너스 1 C0, 독립 재계산).

규칙 원문: 노트 `entries/2026-10-07_0915_b1-ppo10-eval_ppo10-sets-eval.md` 「C 단계 대표 정책 선택 규칙」
(브랜치 feature/b1-ppo10-eval e5b3c03). 요약:
  1. 후보 = 최종 정책 중 main_3_15 엄격 성공 ≥ 0.90 이고 onnx_check_policy_final.json 의 fails 가 빈 것
  2. |c_main15 − median(c_main15)| 최소 (c = 엄격 성공 수, median 은 후보 시드의 중앙값 — 짝수면 가운데 둘의 평균)
  3. 동률이면 |c_stress15 − median(c_stress15)|, 4. 그다음 |c_main30 − median(c_main30)|, 5. 시드 번호가 작은 쪽

    select_rep_policy.py --eval-root <b1-ppo10-eval/eval> --train-root <b1-ppo10/train> \
        [--selection <analysis/selection.json>] [--out <json>]

--selection 을 주면 그 파일의 선택과 대조해 'match' 를 적는다 (다르면 종료 코드 3).
"""

import argparse
import hashlib
import json
import os
import sys

SETS = ('main_3_15', 'stress_3_15', 'main_3_30')
N_SEEDS = 10


def median(xs):
    """중앙값 (짝수면 가운데 두 값의 평균)."""
    s = sorted(xs)
    n = len(s)
    if n == 0:
        raise ValueError('빈 목록')
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def sha256(path):
    """파일의 SHA-256 해시 — 없으면 None."""
    if not os.path.exists(path):
        return None
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def read_seed(eval_root, train_root, s):
    """시드 하나의 집합 평가 값과 ONNX 검사 — 없으면 None."""
    ej = os.path.join(eval_root, f'B1e_ppo10_s{s}_final_sets', 'eval.json')
    if not os.path.exists(ej):
        return None
    with open(ej, encoding='utf-8') as f:
        d = json.load(f)
    row = {'seed': s, 'eval_json': ej, 'unfinished': d.get('unfinished'),
           'policy': d.get('policy')}
    for k in SETS:
        b = d['by'][k]
        iv = b['intervals']
        row[k] = {'n': iv['n'], 'strict': iv['strict_successes'],
                  'rate': iv['strict_successes'] / iv['n'], 'unfinished': b.get('unfinished')}
    oc = os.path.join(train_root, f'B1_ppo10_s{s}', 'onnx_check_policy_final.json')
    fails = None
    if os.path.exists(oc):
        with open(oc, encoding='utf-8') as f:
            fails = json.load(f).get('fails')
    row['onnx_check'] = oc
    row['onnx_fails'] = fails
    onnx = os.path.join(train_root, f'B1_ppo10_s{s}', 'policy_final.onnx')
    row['onnx'] = onnx
    row['onnx_sha256'] = sha256(onnx)
    return row


def select(rows):
    """규칙 1–5 → (선택 시드, 후보 표, 중앙값들)."""
    cand = [r for r in rows
            if r['main_3_15']['rate'] >= 0.90 and r['onnx_fails'] == []]
    if not cand:
        raise ValueError('후보 없음')
    med = {k: median([r[k]['strict'] for r in cand]) for k in SETS}
    table = []
    for r in cand:
        key = tuple(abs(r[k]['strict'] - med[k]) for k in SETS) + (r['seed'],)
        table.append({'seed': r['seed'], 'key': list(key),
                      **{k: r[k]['strict'] for k in SETS}})
    table.sort(key=lambda t: tuple(t['key']))
    return table[0]['seed'], table, med


def main():
    """명령행."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--eval-root', default='/home/CAPYI_AI/runs/bonus1/b1-ppo10-eval/eval')
    ap.add_argument('--train-root', default='/home/CAPYI_AI/runs/bonus1/b1-ppo10/train')
    ap.add_argument('--selection', default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--allow-partial', action='store_true',
                    help='10 시드가 다 없어도 (중간 확인용 — 결과에 partial 표시)')
    a = ap.parse_args()
    rows = [r for r in (read_seed(a.eval_root, a.train_root, s) for s in range(N_SEEDS)) if r]
    missing = sorted(set(range(N_SEEDS)) - {r['seed'] for r in rows})
    bad = [r['seed'] for r in rows
           if r['unfinished'] != 0 or any(r[k]['unfinished'] != 0 for k in SETS)]
    if missing and not a.allow_partial:
        print(f'시드 {missing} 의 eval.json 이 없다 (--allow-partial 로 중간 확인)', file=sys.stderr)
        return 2
    seed, table, med = select(rows)
    res = {'rule': 'b1-ppo10-eval 0915 항목 「C 단계 대표 정책 선택 규칙」 (e5b3c03)',
           'partial': bool(missing), 'missing_seeds': missing, 'unfinished_bad': bad,
           'medians': med, 'ranking': table, 'selected_seed': seed,
           'selected_onnx': next(r['onnx'] for r in rows if r['seed'] == seed),
           'selected_onnx_sha256': next(r['onnx_sha256'] for r in rows if r['seed'] == seed),
           'seeds': rows}
    rc = 0
    if a.selection:
        with open(a.selection, encoding='utf-8') as f:
            other = json.load(f)
        theirs = other.get('selected_seed', other.get('seed'))
        res['their_selection_file'] = a.selection
        res['their_selected_seed'] = theirs
        res['match'] = theirs == seed
        rc = 0 if res['match'] else 3
    text = json.dumps(res, ensure_ascii=False, indent=1)
    if a.out:
        with open(a.out, 'w', encoding='utf-8') as f:
            f.write(text + '\n')
    print(json.dumps({k: res[k] for k in ('partial', 'missing_seeds', 'medians',
                                          'selected_seed', 'selected_onnx_sha256')
                      if k in res}, ensure_ascii=False))
    for t in table:
        print(t)
    if 'match' in res:
        print(f"대조: 그쪽 {res['their_selected_seed']} · 이쪽 {seed} → match={res['match']}")
    return rc


if __name__ == '__main__':
    sys.exit(main())
