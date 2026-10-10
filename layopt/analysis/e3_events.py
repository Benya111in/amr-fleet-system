"""E3 실행에서 '대기 슬롯 정지 중 첫 LOST' 사건마다, 그 뒤 150 s 안에 오수렴(추정–실제 > 1 m 가 5 s 이상)이 났는지 센다.

사용: python3 e3_events.py <e3 루트> <캠페인 jsonl>
"""
import bisect
import csv
import json
import math
import os
import re
import sys

PAT = re.compile(r'\[(\d+\.\d+)\] \[(amr_\d+)\.kidnap_monitor_node\]: \[RECOVERING\] LOST: (.*)$')


def events(p):
    m = json.load(open(os.path.join(p, 'metrics.json')))
    if not m.get('valid'):
        return None
    tb = m['t_base']
    pairs = sorted((e['w'], e['t']) for e in map(json.loads, open(os.path.join(p, 'events.jsonl')))
                   if 'w' in e and 't' in e)
    ws = [a for a, _ in pairs]

    def w2t(w):
        i = min(max(bisect.bisect_left(ws, w), 1), len(pairs) - 1)
        (w0, t0), (w1, t1) = pairs[i - 1], pairs[i]
        return t0 if w1 == w0 else t0 + (t1 - t0) * (w - w0) / (w1 - w0)
    lost = {}
    for line in open(os.path.join(p, 'launch.log'), errors='replace'):
        mm = PAT.search(line)
        if mm:
            lost.setdefault(mm.group(2), []).append(w2t(float(mm.group(1))))
    gt, est = {}, {}
    for r in csv.DictReader(open(os.path.join(p, 'gt.csv'))):
        gt.setdefault(r['robot'], []).append((float(r['t']), float(r['x']), float(r['y']), float(r['v'])))
    for r in csv.DictReader(open(os.path.join(p, 'est.csv'))):
        est.setdefault(r['robot'], []).append((float(r['t']), float(r['x']), float(r['y'])))
    out = []
    for rb, g in gt.items():
        gts = [a[0] for a in g]
        sx, sy = g[0][1], g[0][2]
        parked = [t for t, x, y, v in g if t > tb + 30 and math.hypot(x - sx, y - sy) < 0.3 and abs(v) < 0.05]
        eps, c = [], None
        for t in parked:
            if c and t - c[1] <= 1.0:
                c[1] = t
            else:
                c = [t, t]
                eps.append(c)
        for a, b in eps:
            hits = [t for t in lost.get(rb, []) if a <= t <= b]
            if not hits:
                continue
            tl = min(hits)
            run, best = 0.0, 0.0
            prev = None
            for t, x, y in est.get(rb, []):
                if t < tl or t > tl + 150:
                    continue
                i = min(bisect.bisect_left(gts, t), len(g) - 1)
                e = math.hypot(x - g[i][1], y - g[i][2])
                dt = 0.0 if prev is None else t - prev
                prev = t
                run = run + dt if e > 1.0 else 0.0
                best = max(best, run)
            out.append({'robot': rb, 't_lost': round(tl, 1), 'mis': best >= 5.0})
    return out


def main():
    root, camp = sys.argv[1], sys.argv[2]
    agg = {}
    for line in open(camp):
        c = json.loads(line)
        ev = events(os.path.join(root, c['tag']))
        if ev is None:
            continue
        n, k = len(ev), sum(e['mis'] for e in ev)
        a = agg.setdefault(c['spec'], [0, 0, 0, 0])
        a[0] += 1
        a[1] += n
        a[2] += k
        a[3] += 1 if k else 0
        print(c['spec'], c['tag'][:16], 'parked_first_LOST', n, 'misconverged', k)
    print('--- 배치: 유효 실행, 정지 LOST 사건, 그중 오수렴, 오수렴이 1건 이상인 실행')
    for s, a in agg.items():
        print(s, a)


if __name__ == '__main__':
    main()
