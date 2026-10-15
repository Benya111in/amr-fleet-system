#!/usr/bin/env python3
"""
P5 리뷰(2026-10-15) 차단 B1–B3 를 원자료로 다시 센다 [계산 — 결과 뒤 기록, 판정 지표 아님].

B1: 원판 동시 점유 사건마다 허가 없는 로봇이 그 구역을 요청 · 막힘(`gate` 사건)했는가, 사건 중 정지 비율,
    중심 사이 거리(최대 깊이 때 · 사건 중 최소). 허가 없는 진입 구간(분석기와 같은 규칙: 같은 로봇 · 구역,
    1.0 s 넘게 끊기면 새 구간)을 요청 없음 / 요청만 / 요청+막힘 으로 나눈다.
B2: 펜싱 검사 기회 — fence_reject 수, 그때 펜스 epoch 보다 낮은 epoch 로 온 assign_proxy 수.
B3: 문서 수치 (중심 기준 사건, `b696804` 분산 완료, C2 짝 7 허가 없는 진입 시간, RTF 범위).

쓰기: python3 p5_review_check.py [raw 디렉터리] > out.txt
"""
import collections
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', '..', 'tests', 'integration', 'fleet_p5'))
import p5_analyze as A  # noqa: E402

FINAL_EXPS = ('C2', 'P', 'S', 'X', 'C3')
PRE_FIX_ARM = 'p5_C2_p5_distributed'      # 에이전트 수정만 한 b696804 갈래 중 사건이 난 갈래


def metrics(raw, tag):
    """갈래 metrics.json."""
    with open(os.path.join(raw, tag, 'metrics.json'), encoding='utf-8') as f:
        return json.load(f)


def final_arms(raw):
    """판정에 쓴 짝(`.done`)의 갈래 → (분산, 중앙)."""
    dist, cent = [], []
    for fn in sorted(os.listdir(os.path.join(raw, 'pairs'))):
        if not fn.endswith('.done') or fn.split('_p')[0] not in FINAL_EXPS:
            continue
        with open(os.path.join(raw, 'pairs', fn), encoding='utf-8') as f:
            for tag in json.load(f)['tags'].values():
                (dist if metrics(raw, tag)['mode'] == 'distributed' else cent).append(tag)
    return dist, cent


def gate_marks(events, robot, zone, t0, t1):
    """[t0, t1] 안 `gate` 사건에서 그 구역을 요청한 시각들, 막힌 (시각, 중심 진입 거리)들."""
    req, blk = [], []
    key = f'zone:{zone}'
    for e in events:
        ev = e.get('ev') or {}
        if e.get('k') != 'agent_event' or e.get('r') != robot or ev.get('event') != 'gate':
            continue
        if not t0 <= e['t'] <= t1:
            continue
        if key in (ev.get('request') or []):
            req.append(round(e['t'], 1))
        if key in (ev.get('blocking') or []):
            blk.append((round(e['t'], 1), ev.get('first_entry_m')))
    return req, blk


def incidents(raw, tag, events, gt, held):
    """원판 동시 점유 사건마다 허가 없는 로봇의 기제 행."""
    rows = []
    for o in metrics(raw, tag).get('occupancy_disc') or []:
        zone, t0, t1 = o['zone'], o['t0'], o['t1']
        robots = o['robots']
        unauth = [r for r in robots
                  if f'zone:{zone}' not in (A.value_at(held.get(r, []), t0 + 0.05, []) or [])]
        poses = o.get('poses') or {}
        d_max = math.dist(*poses.values()) if len(poses) == 2 else None
        d_min = min(math.dist(cur[robots[0]], cur[robots[1]]) for _, cur in A.bins(gt, t0, t1)
                    if all(r in cur for r in robots))
        for r in unauth:
            req, blk = gate_marks(events, r, zone, t0 - 15.0, t1 + 1.0)
            samples = [s for s in gt[r] if t0 <= s[0] <= t1]
            stop = sum(1 for s in samples if abs(s[4]) < 0.05) / max(1, len(samples))
            rows.append({'tag': tag, 'zone': zone, 't0': t0, 'dur': round(t1 - t0, 2),
                         'robot': r, 'requested': req[:1], 'blocking': blk[:1],
                         'mech': '요청·막힘·구역 안 대기' if blk else ('요청만' if req else '요청 없음'),
                         'stop_frac': round(stop, 2), 'depth': o.get('depth_m'),
                         'd_center_at_max_depth': None if d_max is None else round(d_max, 2),
                         'd_center_min': round(d_min, 2)})
    return rows


def unauth_episodes(raw, tag, events, gt, held, zones):
    """허가 없는 진입 구간 (분석기 `occupancy` 와 같은 칸 · 같은 끊김 규칙) 과 기제 분류."""
    m = metrics(raw, tag)
    active, eps = {}, []
    for t, cur in A.bins(gt, m['t_base'], m['t_end']):
        for zid, poly in zones.items():
            for r, (x, y) in cur.items():
                if r not in held or not A.disc_overlaps(poly, x, y):
                    continue
                h = A.value_at(held[r], t, None)
                if h is None or f'zone:{zid}' in h:
                    continue
                k = (r, zid)
                if k in active and t - active[k][1] <= 1.0:
                    active[k][1] = t
                    active[k][2] += A.BIN
                else:
                    if k in active:
                        eps.append((k, *active[k]))
                    active[k] = [t, t, A.BIN]
    eps.extend((k, *v) for k, v in active.items())
    rows = []
    for (r, z), a, b, sec in eps:
        req, blk = gate_marks(events, r, z, a - 15.0, b + 1.0)
        rows.append({'tag': tag, 'robot': r, 'zone': z, 't0': round(a, 2), 'sec': round(sec, 2),
                     'mech': '요청+막힘' if blk else ('요청만' if req else '요청 없음')})
    return rows


def fencing(events):
    """(fence_reject 수, assign_proxy 수, 펜스 epoch 보다 낮은 epoch 도착 수, 그중 수용 수)."""
    fence = A.timeline(events, 'agent_state', 'fence_epoch')
    rejects = proxy = low = low_acc = 0
    for e in events:
        ev = e.get('ev') or {}
        if e.get('k') != 'agent_event':
            continue
        if ev.get('event') == 'fence_reject':
            rejects += 1
        if ev.get('event') == 'assign_proxy':
            proxy += 1
            fmax = A.value_at(fence.get(e['r'], []), e['t'] - 1e-6, 0) or 0
            if int(ev.get('epoch') or 0) < int(fmax):
                low += 1
                low_acc += bool(ev.get('accepted'))
    return rejects, proxy, low, low_acc


def main(argv):
    """원자료를 읽어 B1–B3 수치를 출력한다."""
    raw = argv[1] if len(argv) > 1 else '/home/CAPYI_AI/runs/b24/b4-p5/raw'
    dist, cent = final_arms(raw)
    print('최종 코드 분산 갈래', len(dist), '중앙 갈래', len(cent))
    inc, eps = [], []
    fence = collections.Counter()
    for tag in dist + [PRE_FIX_ARM]:
        events = A.read_jsonl(os.path.join(raw, tag, 'events.jsonl'))
        gt = A.read_gt(os.path.join(raw, tag, 'gt.csv'))
        held = A.timeline(events, 'agent_state', 'held')
        with open(os.path.join(raw, tag, 'meta.json'), encoding='utf-8') as f:
            zones = A.load_zones(json.load(f)['zones_text'])
        inc += incidents(raw, tag, events, gt, held)
        if tag in dist:
            eps += unauth_episodes(raw, tag, events, gt, held, zones)
            fence.update(dict(zip(('fence_reject', 'assign_proxy', 'low_epoch_arrived',
                                   'low_epoch_accepted'), fencing(events))))
    print('\n## B1 사건 (허가 없는 로봇마다)')
    for row in inc:
        print(json.dumps(row, ensure_ascii=False))
    by = collections.defaultdict(list)
    for row in inc:
        by[row['mech']].append(row['dur'])
    print('사건', len(inc), '시간 합', round(sum(r['dur'] for r in inc), 1),
          {k: (len(v), round(sum(v), 1)) for k, v in by.items()})
    print('최대 깊이 때 중심 거리', min(r['d_center_at_max_depth'] for r in inc), '–',
          max(r['d_center_at_max_depth'] for r in inc), '사건 중 최소',
          min(r['d_center_min'] for r in inc))
    print('\n## B1 허가 없는 진입 구간 (최종 코드 분산)')
    sec = collections.defaultdict(float)
    cnt = collections.Counter()
    for row in eps:
        sec[row['mech']] += row['sec']
        cnt[row['mech']] += 1
    print('구간', len(eps), dict(cnt), {k: round(v, 1) for k, v in sec.items()},
          '합', round(sum(sec.values()), 1))
    for row in eps:
        if row['mech'] != '요청 없음':
            print(' ', json.dumps(row, ensure_ascii=False))
    print('\n## B2 펜싱 검사 기회 (최종 코드 분산)', dict(fence))
    print('\n## B3')
    center = [(t, o) for t in dist + [PRE_FIX_ARM]
              for o in metrics(raw, t).get('occupancy_center') or []]
    print('분산 중심 기준 사건', len(center), center)
    b69 = []
    for fn in sorted(os.listdir(os.path.join(raw, 'pairs'))):
        if fn.startswith('C2_p') and fn.endswith('pre_mgrfix_b696804'):
            with open(os.path.join(raw, 'pairs', fn), encoding='utf-8') as f:
                for tag in json.load(f)['tags'].values():
                    m = metrics(raw, tag)
                    if m['mode'] == 'distributed':
                        b69.append(m['completed'])
    print('b696804 분산 완료', b69, '7/8 갈래', sum(1 for c in b69 if c == 7))
    c2 = {t: round(sum((metrics(raw, t).get('unauth_s') or {}).values()), 1)
          for t in dist if '_C2_' in t}
    print('C2 분산 허가 없는 진입 s', c2, '합', round(sum(c2.values()), 1))
    rtf = [metrics(raw, t).get('rtf') for t in dist + cent]
    print('RTF 판정 갈래', min(rtf), '–', max(rtf))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
