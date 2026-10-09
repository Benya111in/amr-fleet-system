"""E1 2단계 필수 포함 목록 — 조정자 규칙(일지 2026-10-10 04:18)을 그대로 적용한다.

(a) 상식 수정안: V1≠0 이고 나머지 수준 0 인 배치 가운데 E0 대기 슬롯 최소 마진(k=0)이 가장 큰 것.
(b) 측위 후보 10: 유효 묶음 가운데 E0 대기 슬롯 최소 마진(k=2) 상위 10 (동률이면 평균 마진 k=2 큰 쪽).
정오표(queue)는 wait:* 만 쓰므로 무관하다.

사용: python3 must_include.py <e0 실행 디렉터리> <출력 json>
"""
import json
import os
import sys


def waiting_margins(path, k):
    d = json.load(open(path))
    return [p['margin'] for p in d['points'] if p['kind'] == 'waiting' and p['k'] == k]


def main():
    e0, out = sys.argv[1], sys.argv[2]
    rows = {}
    for f in sorted(os.listdir(e0)):
        if not (f.startswith('v1') and f.endswith('.json')):
            continue
        sid = f[:-5]
        m0, m2 = waiting_margins(os.path.join(e0, f), 0), waiting_margins(os.path.join(e0, f), 2)
        if not m0 or not m2:
            continue
        rows[sid] = {'min0': min(m0), 'min2': min(m2), 'mean2': sum(m2) / len(m2)}
    fix_cands = {s: r for s, r in rows.items() if s.endswith('-v20-v30-v40-v50') and not s.startswith('v10-')}
    fix = max(fix_cands, key=lambda s: fix_cands[s]['min0'])
    top = sorted(rows, key=lambda s: (-rows[s]['min2'], -rows[s]['mean2']))[:10]
    res = {'rule': 'journal 2026-10-10 04:18 (main)', 'e0_run': e0, 'n_bundles': len(rows),
           'human_fix': fix, 'human_fix_candidates': fix_cands, 'loc_top10': top,
           'loc_top10_values': {s: rows[s] for s in top},
           'baseline': rows.get('v10-v20-v30-v40-v50')}
    json.dump(res, open(out, 'x'), indent=1)
    print(fix, fix_cands[fix])
    for s in top:
        print(s, rows[s])
    print('baseline', res['baseline'])


if __name__ == '__main__':
    main()
