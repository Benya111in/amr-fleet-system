"""E3(5 대 Gazebo) 실행별 측위·교통 지표 (실험 0·4).

실행마다:
- 교통: valid, completed/n_tasks, makespan, mean_task_s, rtf, deadlocks, unresolved, 게이트 대기 합(events traffic)
- 측위: LOST 선언 수(launch.log), 대기 슬롯 정지 중 첫 LOST 수와 노출(λ 정의와 같은 규칙),
  오수렴 구간 수(t > t_ready+5 에서 추정–실제 오차 > 1 m 가 5 s 넘게 이어진 구간), 오차 > 1 m 누적 시간,
  그중 점대칭 거울(−x,−y) 2 m 안 구간 수
사용: python3 e3_runs.py <e3 루트> <캠페인 jsonl> <출력 json>
"""
import bisect
import csv
import json
import math
import os
import re
import sys

PAT = re.compile(r'\[(\d+\.\d+)\] \[(amr_\d+)\.kidnap_monitor_node\]: \[RECOVERING\] LOST: (.*)$')


def run_metrics(p):
    m = json.load(open(os.path.join(p, 'metrics.json')))
    out = {k: m.get(k) for k in ('valid', 'completed', 'n_tasks', 'makespan', 'mean_task_s', 'rtf',
                                 'deadlocks', 'unresolved', 't_ready', 't_base', 't_end')}
    pairs, gate = [], 0.0
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
    bt_fail = {}
    for line in open(os.path.join(p, 'launch.log'), errors='replace'):
        mm = PAT.search(line)
        if mm:
            lost.setdefault(mm.group(2), []).append(w2t(float(mm.group(1))))
        elif 'Exception when loading BT' in line:
            mb = re.search(r'\[(amr_\d+)\.bt_navigator\]', line)
            if mb:
                bt_fail[mb.group(1)] = bt_fail.get(mb.group(1), 0) + 1
    gt, est = {}, {}
    for r in csv.DictReader(open(os.path.join(p, 'gt.csv'))):
        gt.setdefault(r['robot'], []).append((float(r['t']), float(r['x']), float(r['y']), float(r['v'])))
    for r in csv.DictReader(open(os.path.join(p, 'est.csv'))):
        est.setdefault(r['robot'], []).append((float(r['t']), float(r['x']), float(r['y'])))
    t0 = (out['t_ready'] or 0) + 5
    tb = out['t_base'] or 60
    rob = {}
    n_lost = n_park_lost = 0
    park_exp = err_s = 0.0
    n_mis = n_mir = 0
    for rb, g in gt.items():
        gts = [a[0] for a in g]
        sx, sy = g[0][1], g[0][2]
        # 오수렴 구간
        segs, cur = [], None
        prev_t = None
        for t, x, y in est.get(rb, []):
            if t < t0:
                continue
            i = min(bisect.bisect_left(gts, t), len(g) - 1)
            _, gx, gy, _ = g[i]
            e = math.hypot(x - gx, y - gy)
            dt = 0.0 if prev_t is None else min(t - prev_t, 1.0)
            prev_t = t
            if e > 1.0:
                err_s += dt
                mir = math.hypot(x + gx, y + gy) < 2.0
                if cur and t - cur[1] <= 2.0:
                    cur[1] = t
                    cur[2] = cur[2] or mir
                else:
                    cur = [t, t, mir]
                    segs.append(cur)
        segs = [s for s in segs if s[1] - s[0] >= 5.0]
        n_mis += len(segs)
        n_mir += sum(1 for s in segs if s[2])
        # 정지 중 첫 LOST
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
            if hits:
                n_park_lost += 1
                park_exp += min(hits) - a
            else:
                park_exp += b - a
        n_lost += len(lost.get(rb, []))
        rob[rb] = {'lost': len(lost.get(rb, [])), 'mis_segments': [[round(s[0], 1), round(s[1], 1), s[2]]
                                                                   for s in segs]}
    # 사후 표시(사전 등록 외): Nav2 bt_navigator 가 BT 를 끝내 못 올린 로봇 = 그 로봇은 실행 내내 주행 불가
    out['nav_dead'] = sorted(r for r, c in bt_fail.items() if c >= 10)
    out.update({'lost_total': n_lost, 'parked_first_lost': n_park_lost, 'parked_exposure_s': round(park_exp, 1),
                'mis_segments': n_mis, 'mirror_segments': n_mir, 'err_gt1m_robot_s': round(err_s, 1),
                'robots': rob})
    return out


def main():
    root, camp, outp = sys.argv[1], sys.argv[2], sys.argv[3]
    res = []
    for line in open(camp):
        c = json.loads(line)
        p = os.path.join(root, c['tag'])
        r = {'spec': c['spec'], 'tag': c['tag'], 'rc': c['rc']}
        try:
            r.update(run_metrics(p))
        except Exception as ex:  # 무효 실행은 파일이 빠질 수 있다
            r['error'] = repr(ex)
        res.append(r)
    json.dump(res, open(outp, 'x'), indent=1)
    for r in res:
        print(r['spec'], r['tag'][:16], 'rc', r['rc'], 'valid', r.get('valid'), 'done', r.get('completed'),
              'mk', r.get('makespan'), 'rtf', r.get('rtf'), 'LOST', r.get('lost_total'),
              'parkLOST', r.get('parked_first_lost'), '/', r.get('parked_exposure_s'), 's',
              'mis', r.get('mis_segments'), 'mirror', r.get('mirror_segments'), 'err>1m', r.get('err_gt1m_robot_s'),
              'nav_dead', r.get('nav_dead'),
              r.get('error', ''))


if __name__ == '__main__':
    main()
