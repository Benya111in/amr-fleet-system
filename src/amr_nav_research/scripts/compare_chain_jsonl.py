#!/usr/bin/env python3
"""
chain_sim 과 chain_sim_x 의 JSONL 지표를 필드별로 비교한다 (prereg.md §3.3 동치 시험).

    compare_chain_jsonl.py <orig.jsonl> <x.jsonl> [--out summary.json]

- 값은 출력 원문 그대로(문자열) 비교한다 — 비트 동일 판정이다 (printf 형식이 같으므로
  같은 double 은 같은 글자가 된다. -0.0000 과 0.0000 도 다르게 센다).
- 계산 시간 필드(cycle_us_*)만 뺀다.
- 원본 줄의 키는 X 줄의 앞부분에 같은 순서로 있어야 한다. X 가 덧붙인 필드는 비교하지 않는다.
- 줄 순서와 (scene, arm, seed) 도 같아야 한다.
종료 코드: 모두 같으면 0, 하나라도 다르면 1.
"""
import argparse
import json
import sys

TIMING = ('cycle_us_mean', 'cycle_us_p95', 'cycle_us_max')


def load(fn):
    rows = []
    with open(fn, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line, parse_float=str, parse_int=str))
    return rows


def compare_row(a, b):
    """Return a list of differences between original row a and X row b."""
    diffs = []
    keys_a = list(a.keys())
    keys_b = list(b.keys())
    if keys_b[:len(keys_a)] != keys_a:
        diffs.append({'field': '<key order>', 'orig': keys_a, 'x': keys_b[:len(keys_a)]})
    for k in keys_a:
        if k in TIMING:
            continue
        if k not in b:
            diffs.append({'field': k, 'orig': a[k], 'x': None})
        elif a[k] != b[k]:
            diffs.append({'field': k, 'orig': a[k], 'x': b[k]})
    return diffs


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('orig')
    ap.add_argument('x')
    ap.add_argument('--out', default='')
    args = ap.parse_args()
    ra = load(args.orig)
    rb = load(args.x)
    rows = []
    n_same = 0
    for i in range(max(len(ra), len(rb))):
        if i >= len(ra) or i >= len(rb):
            rows.append({'line': i, 'diffs': [{'field': '<missing line>'}]})
            continue
        d = compare_row(ra[i], rb[i])
        if d:
            rows.append({'line': i, 'scene': ra[i].get('scene'), 'diffs': d})
        else:
            n_same += 1
    n = max(len(ra), len(rb))
    summary = {
        'orig': args.orig, 'x': args.x, 'n_lines': n, 'n_identical': n_same,
        'n_fields_compared': len([k for k in (ra[0] if ra else {}) if k not in TIMING]),
        'excluded_fields': list(TIMING), 'mismatches': rows[:50],
    }
    text = json.dumps(summary, ensure_ascii=False, indent=1)
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            f.write(text + '\n')
    print(f'{n_same}/{n} identical ({args.orig} vs {args.x})')
    if rows:
        print(text)
    return 0 if n_same == n and n > 0 else 1


if __name__ == '__main__':
    sys.exit(main())
