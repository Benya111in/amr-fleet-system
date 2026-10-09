"""λ_LOST (prereg §7): P5 유효 실행에서 '대기 슬롯 정지 중 첫 LOST 선언 수 / 첫 LOST 전까지의 정지 로봇·초'.

- LOST 시각: launch.log 의 '[RECOVERING] LOST:' 줄 벽시계 → events.jsonl 의 (w, t) 쌍으로 선형 보간해 sim 시각.
- 정지: t > t_base + 30, 자기 슬롯(첫 gt 위치) 0.3 m 안, |v| < 0.05.
- 정지 구간(연속 정지)마다 첫 LOST 까지만 노출로 센다 (첫 사건 위험률).
사용: python3 lambda_lost.py <P5 raw 디렉터리> <출력 json>
"""
import bisect
import csv
import json
import math
import os
import re
import sys

PAT = re.compile(r'\[(\d+\.\d+)\] \[(amr_\d+)\.kidnap_monitor_node\]: \[RECOVERING\] LOST: (.*)$')


def main():
    raw, out = sys.argv[1], sys.argv[2]
    tot_exp, n_ev, rows = 0.0, 0, []
    for d in sorted(os.listdir(raw)):
        p = os.path.join(raw, d)
        try:
            m = json.load(open(os.path.join(p, 'metrics.json')))
        except Exception:
            continue
        if not m.get('valid'):
            continue
        tb = m['t_base']
        pairs = []
        for line in open(os.path.join(p, 'events.jsonl')):
            e = json.loads(line)
            if 'w' in e and 't' in e:
                pairs.append((e['w'], e['t']))
        pairs.sort()
        ws = [a for a, _ in pairs]

        def w2t(w):
            i = min(max(bisect.bisect_left(ws, w), 1), len(pairs) - 1)
            (w0, t0), (w1, t1) = pairs[i - 1], pairs[i]
            return t0 if w1 == w0 else t0 + (t1 - t0) * (w - w0) / (w1 - w0)
        lost = {}
        for line in open(os.path.join(p, 'launch.log'), errors='replace'):
            mm = PAT.search(line)
            if mm:
                lost.setdefault(mm.group(2), []).append((w2t(float(mm.group(1))), mm.group(3)[:60]))
        gt = {}
        for r in csv.DictReader(open(os.path.join(p, 'gt.csv'))):
            gt.setdefault(r['robot'], []).append((float(r['t']), float(r['x']), float(r['y']), float(r['v'])))
        for rb, g in gt.items():
            sx, sy = g[0][1], g[0][2]
            parked = [t for t, x, y, v in g
                      if t > tb + 30 and math.hypot(x - sx, y - sy) < 0.3 and abs(v) < 0.05]
            # 연속 정지 구간으로 묶기 (간격 > 1 s 면 끊김)
            eps, cur = [], None
            for t in parked:
                if cur and t - cur[1] <= 1.0:
                    cur[1] = t
                else:
                    cur = [t, t]
                    eps.append(cur)
            for a, b in eps:
                hits = [(t, why) for t, why in lost.get(rb, []) if a <= t <= b]
                if hits:
                    t_hit, why = min(hits)
                    tot_exp += t_hit - a
                    n_ev += 1
                    rows.append({'run': d, 'robot': rb, 'park_start': round(a, 1),
                                 't_lost': round(t_hit, 1), 'exposure_s': round(t_hit - a, 1),
                                 'why': why})
                else:
                    tot_exp += b - a
                    rows.append({'run': d, 'robot': rb, 'park_start': round(a, 1),
                                 't_lost': None, 'exposure_s': round(b - a, 1)})
    lam = n_ev / tot_exp if tot_exp else None
    res = {'n_events': n_ev, 'exposure_s': round(tot_exp, 1), 'lambda_per_s': lam,
           'lambda_per_h': lam * 3600 if lam else None, 'episodes': rows}
    json.dump(res, open(out, 'x'), indent=1, ensure_ascii=False)
    print(json.dumps({k: v for k, v in res.items() if k != 'episodes'}))
    for r in rows:
        if r['t_lost'] is not None:
            print(r)


if __name__ == '__main__':
    main()
