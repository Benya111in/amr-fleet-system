#!/usr/bin/env python3
"""
보너스 4 P5 분석 (사전 등록 — 결과 전에 커밋하고 sha256 을 노트 항목에 적는다). 표준 라이브러리 + PyYAML.

  python3 p5_analyze.py run <갈래 원자료 디렉터리>  → metrics.json (+ registry.json, --registry-info 가 있으면)
  python3 p5_analyze.py exp <실험> <원자료 루트> [--out DIR]    → <실험>.json · <실험>.md (짝 표 · 판정)
  시험: tests/integration/unit/test_fleet_p5.py (합성 자료)

정의 (노트 항목 「P5 사전 등록」과 같다)
- 실제 자세 기준 구역 점유: 정답 자세(ground_truth/odom) 원판 반경 BODY_R = 0.36 m 가 구역 다각형과 겹치면 그 구역 안.
  같은 구역 안에 두 대 이상이 동시에 있는 0.1 s 칸이 이어진 구간 = 동시 점유 사건. 중심 기준(중심점이 다각형 안)도 따로 센다.
  다각형은 그 실행이 쓴 traffic_zones.yaml(meta.json 에 원문) 과, 도크는 **고정 지표**(옛 fd17e49 의 큰 상자 ±1 m)로도 센다.
- 믿은 허가 겹침: 에이전트 상태 held 의 zone:* 를 두 로봇이 같은 시각에 가진 구간 (분산 모드).
- 허가 없는 진입: 분산 모드에서 원판이 구역 Z 와 겹치는데 그 로봇의 held 에 zone:Z 가 없는 0.1 s 칸 (plan 신선도 · 양보 이동 우회 지표).
- 완료: 실행기 task_status COMPLETED (로봇 · 시각). 완료 시간 = 완료 sim − (t_base + offset).
  makespan = 마지막 완료 − t_base.
- 픽업: 로봇이 LOADING 단계에 들어갈 때 그 로봇의 마지막 IN_PROGRESS 작업. 중복 = 같은 작업의 픽업이 두 번 이상 (다른 로봇이든 같은 로봇이든).
- 이동 거리: 정답 자세 경로 길이 (0.1 s 칸, 칸당 이동 < 0.02 m 는 잡음으로 버림).
"""

import argparse
import bisect
import collections
import csv
import json
import math
import os
import statistics
import sys

BODY_R = 0.36
BIN = 0.1
OLD_DOCK_BOXES = {   # fd17e49 traffic_zones.yaml (도크 구역 ±1 m) — F5 비교의 고정 지표 (리뷰 §10.8)
    'dock_1_approach': [[-29.8, 16.0], [-26.8, 16.0], [-26.8, 18.0], [-29.8, 18.0]],
    'dock_2_approach': [[-29.8, 12.0], [-26.8, 12.0], [-26.8, 14.0], [-29.8, 14.0]],
    'dock_a_approach': [[26.8, 16.0], [29.8, 16.0], [29.8, 18.0], [26.8, 18.0]],
    'dock_b_approach': [[26.8, 12.0], [29.8, 12.0], [29.8, 14.0], [26.8, 14.0]]}
MODE = {0: 'NORMAL', 1: 'HOLD_NEW', 2: 'DEGRADED', 3: 'ISOLATED', 4: 'RESYNC'}


# ---------------------------------------------------------------------- 기하
def _seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    u = 0.0 if L == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L))
    return math.hypot(px - (ax + u * dx), py - (ay + u * dy))


def inside(poly, x, y) -> bool:
    """점-다각형 (짝수-홀수 규칙)."""
    n, c = len(poly), False
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xi = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xi:
                c = not c
    return c


def signed_distance(poly, x, y) -> float:
    """점 → 다각형 경계 거리 (안이면 음수)."""
    d = min(_seg_dist(x, y, *poly[i], *poly[(i + 1) % len(poly)]) for i in range(len(poly)))
    return -d if inside(poly, x, y) else d


def disc_overlaps(poly, x, y, r=BODY_R) -> bool:
    return signed_distance(poly, x, y) < r


def load_zones(text: str) -> dict:
    import yaml
    doc = yaml.safe_load(text) or {}
    return {z['id']: [list(map(float, p)) for p in z['polygon']]
            for z in doc.get('zones', []) if z.get('polygon')}


# ---------------------------------------------------------------------- 읽기
def read_jsonl(path):
    out = []
    if not os.path.isfile(path):
        return out
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
    return out


def read_json(path, default=None):
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def read_gt(path):
    """{robot: [(t, x, y, yaw, v)]} 시각순."""
    out = collections.defaultdict(list)
    if not os.path.isfile(path):
        return out
    with open(path, encoding='utf-8') as f:
        for row in csv.DictReader(f):
            try:
                out[row['robot']].append((float(row['t']), float(row['x']), float(row['y']),
                                          float(row['yaw']), float(row.get('v') or 0.0)))
            except (KeyError, ValueError):
                pass
    for r in out:
        out[r].sort()
    return out


def bins(gt, t0, t1, stale=0.25):
    """0.1 s 칸마다 {robot: (x, y)} — 칸 시각 이전의 마지막 표본 (stale 초보다 오래되면 없음)."""
    times = {r: [s[0] for s in v] for r, v in gt.items()}
    k0, k1 = int(math.floor(t0 / BIN)), int(math.ceil(t1 / BIN))
    for k in range(k0, k1 + 1):
        t = k * BIN
        cur = {}
        for r, v in gt.items():
            i = bisect.bisect_right(times[r], t) - 1
            if i >= 0 and t - v[i][0] <= stale:
                cur[r] = (v[i][1], v[i][2])
        yield t, cur


# ---------------------------------------------------------------------- 시간선
def timeline(events, kind, key):
    """{robot: [(t, value)]} — events 중 k == kind 의 key 값."""
    out = collections.defaultdict(list)
    for e in events:
        if e.get('k') == kind and e.get('r'):
            out[e['r']].append((e['t'], e.get(key)))
    for r in out:
        out[r].sort(key=lambda p: p[0])
    return out


def value_at(series, t, default=None):
    i = bisect.bisect_right([p[0] for p in series], t) - 1
    return series[i][1] if i >= 0 else default


def intervals(flags):
    """[(t, bool)] (BIN 간격) → [(t_start, t_end)]."""
    out, start, last = [], None, None
    for t, on in flags:
        if on and start is None:
            start = t
        if not on and start is not None:
            out.append((start, t))
            start = None
        last = t
    if start is not None:
        out.append((start, (last or start) + BIN))
    return out


# ---------------------------------------------------------------------- 실행 지표
def occupancy(gt, zones, t0, t1, held_tl=None, mode_tl=None, phase_tl=None):
    """동시 점유 사건 (원판 · 중심), 허가 없는 진입 칸 (held_tl 이 있으면)."""
    ev_disc = collections.defaultdict(list)
    ev_center = collections.defaultdict(list)
    unauth = collections.defaultdict(float)
    unauth_ev = []
    active = {}
    held_cache = {}
    for t, cur in bins(gt, t0, t1):
        for zid, poly in zones.items():
            ins = sorted(r for r, (x, y) in cur.items() if disc_overlaps(poly, x, y))
            cen = sorted(r for r, (x, y) in cur.items() if inside(poly, x, y))
            ev_disc[zid].append((t, len(ins) >= 2, ins))
            ev_center[zid].append((t, len(cen) >= 2, cen))
            if held_tl is not None:
                for r in ins:
                    if r not in held_tl:
                        continue
                    h = value_at(held_tl[r], t, None)
                    if h is None:
                        continue
                    held_cache[r] = h
                    if f'zone:{zid}' not in h:
                        unauth[(r, zid)] += BIN
                        key = (r, zid)
                        if active.get(key) is None or t - active[key] > 1.0:
                            unauth_ev.append({'t': round(t, 2), 'robot': r, 'zone': zid,
                                              'pose': [round(v, 2) for v in cur[r]],
                                              'mode': None if mode_tl is None or r not in mode_tl
                                              else MODE.get(value_at(mode_tl[r], t), '?'),
                                              'phase': None if phase_tl is None
                                              or r not in phase_tl
                                              else value_at(phase_tl[r], t)})
                        active[key] = t

    def collect(series_by_zone):
        out = []
        for zid, series in series_by_zone.items():
            for a, b in intervals([(t, f) for t, f, _ in series]):
                who = sorted({r for t, f, rs in series if a <= t < b and f for r in rs})
                out.append({'zone': zid, 't0': round(a, 2), 't1': round(b, 2),
                            'dur': round(b - a, 2), 'robots': who})
        return sorted(out, key=lambda e: e['t0'])
    return {'disc': collect(ev_disc), 'center': collect(ev_center),
            'unauth_s': {f'{r}:{z}': round(v, 2) for (r, z), v in sorted(unauth.items())},
            'unauth_events': unauth_ev}


def depth_details(gt, zones, events):
    """사건마다 최대 침범 깊이 · 그때 자세 (원판 경계 기준: r − 부호 거리, 둘 중 작은 쪽)."""
    for e in events:
        poly = zones[e['zone']]
        best = None
        for t, cur in bins(gt, e['t0'], e['t1']):
            ds = []
            for r in e['robots']:
                if r in cur:
                    ds.append((BODY_R - signed_distance(poly, *cur[r]), r, cur[r]))
            if len(ds) >= 2:
                ds.sort()
                depth = ds[-2][0]     # 둘째로 깊은 로봇의 깊이 = 동시 점유 깊이
                if best is None or depth > best[0]:
                    best = (depth, t, {r: [round(p[0], 2), round(p[1], 2)] for _, r, p in ds})
        if best:
            e['depth_m'] = round(best[0], 3)
            e['t_max'] = round(best[1], 2)
            e['poses'] = best[2]
    return events


def held_overlaps(held_tl, t0, t1):
    """믿은 허가 겹침: 같은 zone:* 를 두 로봇이 동시에 held."""
    res = collections.defaultdict(list)
    times = sorted({t for v in held_tl.values() for t, _ in v} | {t0})
    for i, t in enumerate(times):
        if t < t0 or t > t1:
            continue
        by = collections.defaultdict(list)
        for r, ser in held_tl.items():
            for h in value_at(ser, t, []) or []:
                if h.startswith('zone:'):
                    by[h].append(r)
        for h, rs in by.items():
            if len(rs) >= 2:
                res[h].append({'t': round(t, 2), 'robots': sorted(rs)})
    return {h: v for h, v in res.items()}


def path_length(gt, t0, t1):
    out = {}
    for r, v in gt.items():
        d, last = 0.0, None
        for t, x, y, _, _ in v:
            if t < t0 or t > t1:
                continue
            if last is not None:
                step = math.hypot(x - last[0], y - last[1])
                if step >= 0.02:
                    d += step
                    last = (x, y)
            else:
                last = (x, y)
        out[r] = round(d, 2)
    return out


def min_separation(gt, t0, t1, thr=0.8):
    """로봇 쌍 최소 중심 거리와 thr 미만 구간 (차체 막힘 · 같은 픽업 자세 모임 지표)."""
    best = {}
    close = collections.defaultdict(list)
    for t, cur in bins(gt, t0, t1):
        rs = sorted(cur)
        for i in range(len(rs)):
            for j in range(i + 1, len(rs)):
                a, b = rs[i], rs[j]
                d = math.hypot(cur[a][0] - cur[b][0], cur[a][1] - cur[b][1])
                k = f'{a}-{b}'
                if k not in best or d < best[k][0]:
                    best[k] = (d, t, cur[a], cur[b])
                close[k].append((t, d < thr))
    ev = []
    for k, flags in close.items():
        for a, b in intervals(flags):
            ev.append({'pair': k, 't0': round(a, 2), 't1': round(b, 2)})
    return ({k: {'d': round(v[0], 3), 't': round(v[1], 2),
                 'at': [[round(c, 2) for c in v[2]], [round(c, 2) for c in v[3]]]}
             for k, v in best.items()}, sorted(ev, key=lambda e: e['t0']))


def pickups(events):
    """[(t, robot, task)] — LOADING 진입 때 그 로봇의 마지막 IN_PROGRESS 작업."""
    cur = {}
    out = []
    for e in sorted(events, key=lambda e: e['t']):
        if e.get('k') == 'task_status' and e.get('status') == 1:
            cur[e['r']] = e.get('task_id')
        elif e.get('k') == 'phase' and e.get('phase') == 'LOADING':
            out.append((e['t'], e['r'], cur.get(e['r'])))
    return out


def docking(events, gt, behavior_text):
    """DOCKING → LOADING · UNLOADING 전이 때 정답 자세 대 기대 도킹 자세 (marker_pose + standoff)."""
    import yaml
    doc = yaml.safe_load(behavior_text or '') or {}
    table = doc.get('/**', {}).get('ros__parameters', {}).get('docks', {})
    expect = {}
    for d in table.get('ids', []):
        e = table[d]
        if 'marker_pose' not in e:
            continue
        x, y, yaw = (float(v) for v in e['marker_pose'])
        k = 0.02 / 2.0 + float(e['standoff'])
        expect[d] = (x + k * math.cos(yaw), y + k * math.sin(yaw),
                     math.atan2(math.sin(yaw + math.pi), math.cos(yaw + math.pi)))
    phases = timeline(events, 'phase', 'phase')
    out = []
    for r, ser in phases.items():
        for (t, ph), (t2, ph2) in zip(ser, ser[1:]):
            if ph != 'DOCKING' or ph2 not in ('LOADING', 'UNLOADING'):
                continue
            v = gt.get(r, [])
            i = bisect.bisect_right([s[0] for s in v], t2) - 1
            if i < 0:
                continue
            _, x, y, yaw, _ = v[i]
            dock = min(expect, key=lambda d: math.hypot(expect[d][0] - x, expect[d][1] - y),
                       default=None)
            if dock is None:
                continue
            ex = expect[dock]
            dyaw = math.atan2(math.sin(yaw - ex[2]), math.cos(yaw - ex[2]))
            out.append({'robot': r, 't': round(t2, 2), 'dock': dock, 'next': ph2,
                        'pos_err_m': round(math.hypot(x - ex[0], y - ex[1]), 4),
                        'yaw_err_deg': round(abs(math.degrees(dyaw)), 3),
                        'dock_s': round(t2 - t, 2)})
    entries = sum(1 for ser in phases.values() for _, ph in ser if ph == 'DOCKING')
    return {'completed': sorted(out, key=lambda d: d['t']), 'docking_entries': entries}


def modes(events, t0, t1):
    tl = timeline(events, 'agent_state', 'mode')
    trans = []
    for r, ser in tl.items():
        last = None
        for t, m in ser:
            if m != last:
                trans.append({'t': round(t, 2), 'robot': r, 'mode': MODE.get(m, m),
                              'from': MODE.get(last, last)})
                last = m
    return sorted(trans, key=lambda d: (d['t'], d['robot'])), tl


def first_after(trans, robot, mode, t):
    for d in trans:
        if d['robot'] == robot and d['mode'] == mode and d['t'] >= t:
            return d['t']
    return None


def run_metrics(raw: str) -> dict:
    meta = read_json(os.path.join(raw, 'meta.json'), {}) or {}
    summ = read_json(os.path.join(raw, 'summary.json'), {}) or {}
    plan = meta.get('plan') or read_json(os.path.join(raw, 'plan.json'), {}) or {}
    events = read_jsonl(os.path.join(raw, 'events.jsonl'))
    faults = read_jsonl(os.path.join(raw, 'faults.jsonl'))
    gt = read_gt(os.path.join(raw, 'gt.csv'))
    out = {'raw': raw, 'exp': plan.get('exp'), 'pair': plan.get('pair'),
           'label': plan.get('label'), 'mode': plan.get('mode'), 'layout': plan.get('layout'),
           'why': summ.get('why'), 'notes': summ.get('notes', []),
           'aborted': os.path.isfile(os.path.join(raw, 'ABORTED.txt'))}
    t_base, t_end = summ.get('t_base'), summ.get('t_end')
    out.update({'t_ready': summ.get('t_ready'), 't_base': t_base, 't_end': t_end,
                'lifecycle_assumed': summ.get('lifecycle_assumed'),
                'hold_first_missing': sorted(set(plan.get('robots', [])) -
                                             set(summ.get('hold_first', []))),
                'hold_unmanaged': summ.get('hold_unmanaged', [])})
    out['valid'] = bool(summ.get('why') == 'DONE' and not out['aborted'] and t_base is not None)
    ps_path = os.path.join(raw, 'ps_stack.txt')
    if os.path.isfile(ps_path):
        with open(ps_path, encoding='utf-8') as f:
            ps = f.read()
        out['stack_processes'] = max(0, len(ps.strip().split('\n')) - 2)
        out['stack_has_server'] = any(n in ps for n in ('fleet_manager_node',
                                                        'traffic_manager_node'))
    if t_base is None or t_end is None:
        return out
    # 작업
    tasks = plan.get('tasks', [])
    completed = {}
    for e in events:
        if e.get('k') == 'task_status' and e.get('status') == 2 and e.get('task_id'):
            completed.setdefault(e['task_id'], []).append((e['t'], e['r']))
    tinfo = []
    for t in tasks:
        tid = t['task_id']
        c = sorted(completed.get(tid, []))
        rel = t_base + float(t['offset_s'])
        tinfo.append({'task_id': tid, 'posted': rel, 'completed_t': c[0][0] if c else None,
                      'robot': c[0][1] if c else None,
                      'dt': round(c[0][0] - rel, 2) if c else None,
                      'completions': len(c)})
    done = [d for d in tinfo if d['completed_t'] is not None]
    out['tasks'] = tinfo
    out['completed'] = len(done)
    out['n_tasks'] = len(tasks)
    out['makespan'] = round(max(d['completed_t'] for d in done) - t_base, 2) if done else None
    out['mean_task_s'] = (round(statistics.mean(d['dt'] for d in done), 2) if done else None)
    out['failed_reports'] = sum(1 for e in events if e.get('k') == 'task_status'
                                and e.get('status') == 3)
    pk = pickups(events)
    by_task = collections.defaultdict(list)
    for t, r, tid in pk:
        by_task[tid].append((round(t, 2), r))
    out['pickups'] = len(pk)
    out['duplicate_pickups'] = {k: v for k, v in by_task.items() if k and len(v) >= 2}
    out['completions_dup'] = {k: v for k, v in completed.items() if len(v) >= 2}
    # 거리 · 근접
    t_stop = t_end
    out['distance_m'] = path_length(gt, t_base, t_stop)
    out['distance_total_m'] = round(sum(out['distance_m'].values()), 2)
    if done:
        out['distance_to_makespan_m'] = round(sum(path_length(
            gt, t_base, t_base + out['makespan']).values()), 2)
    sep, close = min_separation(gt, t_base, t_stop)
    out['min_separation'] = sep
    out['close_events'] = close
    # 구역 점유
    zones = load_zones(meta.get('zones_text', '')) if meta.get('zones_text') else {}
    held_tl = timeline(events, 'agent_state', 'held') if out['mode'] == 'distributed' else None
    trans, mode_tl = modes(events, t_base, t_stop)
    if zones:
        occ = occupancy(gt, zones, t_base, t_stop, held_tl if held_tl else None, mode_tl,
                        timeline(events, 'phase', 'phase'))
        out['occupancy_disc'] = depth_details(gt, zones, occ['disc'])
        out['occupancy_center'] = occ['center']
        out['unauth_s'] = occ['unauth_s']
        out['unauth_events'] = occ['unauth_events'][:200]
        fixed = dict(zones)
        fixed.update(OLD_DOCK_BOXES)
        occ_fixed = occupancy(gt, OLD_DOCK_BOXES, t_base, t_stop)
        out['occupancy_dock_fixed_disc'] = depth_details(gt, fixed, occ_fixed['disc'])
        out['occupancy_dock_fixed_center'] = occ_fixed['center']
        # 노출: 로봇-구역 진입 수 (원판 기준)
        entries = 0
        for zid, poly in zones.items():
            for r in gt:
                flags = [(t, r in cur and disc_overlaps(poly, *cur[r]))
                         for t, cur in bins({r: gt[r]}, t_base, t_stop)]
                entries += len(intervals(flags))
        out['zone_entries'] = entries
    if held_tl:
        out['held_overlaps'] = held_overlaps(held_tl, t_base, t_stop)
    # 모드 · 펜스 · 장애
    out['mode_transitions'] = trans
    out['faults'] = faults
    kills = [f for f in faults if f.get('action') in ('kill', 'pause')]
    starts = [f for f in faults if f.get('action') in ('start', 'unpause')]
    out['fault_response'] = []
    for f in kills:
        t = f.get('sim')
        if t is None:
            continue
        resp = {'action': f['action'], 'sim': t}
        for r in plan.get('robots', []):
            a = first_after(trans, r, 'HOLD_NEW', t)
            b = first_after(trans, r, 'DEGRADED', t)
            resp[r] = {'hold_new_s': None if a is None else round(a - t, 2),
                       'degraded_s': None if b is None else round(b - t, 2)}
        out['fault_response'].append(resp)
    for f in starts:
        t = f.get('sim')
        if t is None:
            continue
        resp = {'action': f['action'], 'sim': t}
        for r in plan.get('robots', []):
            a = first_after(trans, r, 'RESYNC', t)
            b = first_after(trans, r, 'NORMAL', t)
            resp[r] = {'resync_s': None if a is None else round(a - t, 2),
                       'normal_s': None if b is None else round(b - t, 2)}
        out['fault_response'].append(resp)
    # 에이전트 재기동 (p3 §8-7): boot 사건 (incarnation · 디스크에서 남긴 구역 · 작업 잠금), 재기동 뒤 그 로봇의 구역 해제
    boots = []
    for e in events:
        ev = e.get('ev') or {}
        if e.get('k') == 'agent_event' and ev.get('event') == 'boot':
            rel = [x['t'] for x in events if x.get('k') == 'agent_event' and x.get('r') == e['r']
                   and (x.get('ev') or {}).get('event') == 'zone_release' and x['t'] >= e['t']]
            boots.append({'t': round(e['t'], 2), 'robot': e['r'],
                          'incarnation': ev.get('incarnation'),
                          'kept_zones': ev.get('kept_zones'), 'kept_tasks': ev.get('kept_tasks'),
                          'first_release_after_s': round(min(rel) - e['t'], 2) if rel else None})
    out['boots'] = boots
    for f in faults:
        if f.get('action') in ('agent_kill', 'agent_restart') and f.get('sim') is not None:
            later = [b for b in boots if b['robot'] == f.get('robot') and b['t'] >= f['sim']]
            out['fault_response'].append({'action': f['action'], 'sim': f['sim'],
                                          'robot': f.get('robot'),
                                          'boot_after_s': round(later[0]['t'] - f['sim'], 2)
                                          if later else None})
    # 하트비트 epoch
    hb = [(e['t'], e.get('epoch'), e.get('role'), e.get('leader')) for e in events
          if e.get('k') == 'hb']
    out['hb_epochs'] = sorted({(ep, ro) for _, ep, ro, _ in hb if ep is not None})
    # 낮은 epoch 수용: assign_proxy accepted 인데 그때 그 로봇 펜스 최고보다 낮은 epoch
    fence_tl = timeline(events, 'agent_state', 'fence_epoch')
    low_acc, rejects, proxy = [], 0, 0
    for e in events:
        if e.get('k') != 'agent_event':
            continue
        ev = e.get('ev') or {}
        if ev.get('event') == 'fence_reject':
            rejects += 1
        if ev.get('event') == 'assign_proxy':
            proxy += 1
            fmax = value_at(fence_tl.get(e['r'], []), e['t'] - 1e-6, 0) or 0
            if ev.get('accepted') and int(ev.get('epoch') or 0) < int(fmax):
                low_acc.append({'t': e['t'], 'robot': e['r'], 'epoch': ev.get('epoch'),
                                'fence': fmax, 'task_id': ev.get('task_id')})
    out['low_epoch_accepted'] = low_acc
    out['fence_rejects'] = rejects
    out['assign_proxy'] = proxy
    # 사건 수 (분산)
    cnt = collections.Counter((e.get('ev') or {}).get('event') for e in events
                              if e.get('k') == 'agent_event')
    out['agent_event_counts'] = dict(sorted((k or '?', v) for k, v in cnt.items()))
    # 경로 게이트 정지: gate 사건 (blocking 비지 않음) 뒤 첫 정지(v < 0.05) 때 원판 ↔ 막는 구역 거리
    gate_stops = []
    for e in events:
        ev = e.get('ev') or {}
        if e.get('k') != 'agent_event' or ev.get('event') != 'gate' or not ev.get('blocking'):
            continue
        r = e['r']
        v = gt.get(r, [])
        i = bisect.bisect_left([s[0] for s in v], e['t'])
        stop = next((s for s in v[i:i + 100] if s[4] < 0.05), None)
        if stop is None or not zones:
            continue
        zid = str(ev['blocking'][0]).replace('zone:', '')
        if zid in zones:
            gate_stops.append({'t': round(e['t'], 2), 'robot': r, 'zone': zid,
                               'stop_after_s': round(stop[0] - e['t'], 2),
                               'clear_m': round(signed_distance(zones[zid], stop[1], stop[2])
                                                - BODY_R, 3)})
    out['gate_stops'] = gate_stops
    # 해제 게이트: zone_release 사건 때 정답 원판이 아직 그 구역과 겹치는가 (AMCL 별칭 · 자세 튐 — p3 §8-3)
    rel_inside, rel_n = [], 0
    for e in events:
        ev = e.get('ev') or {}
        if e.get('k') != 'agent_event' or ev.get('event') != 'zone_release' or not zones:
            continue
        v = gt.get(e['r'], [])
        i = bisect.bisect_right([s[0] for s in v], e['t']) - 1
        if i < 0:
            continue
        for res_id in ev.get('resources') or []:
            zid = str(res_id).replace('zone:', '')
            if zid not in zones:
                continue
            rel_n += 1
            if disc_overlaps(zones[zid], v[i][1], v[i][2]):
                rel_inside.append({'t': round(e['t'], 2), 'robot': e['r'], 'zone': zid,
                                   'gt': [round(v[i][1], 2), round(v[i][2], 2)],
                                   'clear_m': round(signed_distance(zones[zid], v[i][1], v[i][2])
                                                    - BODY_R, 3)})
    out['zone_releases'] = rel_n
    out['release_inside'] = rel_inside
    # 작업 잠금 확정 (CommitTaskLock): 요청 뒤 35 s(sim) 안에 그 로봇 held 에 task:<id> 가 없으면 실패
    commits = []
    held_add = collections.defaultdict(list)
    for e in events:
        ev = e.get('ev') or {}
        if e.get('k') == 'agent_event' and ev.get('event') == 'held':
            for res_id in ev.get('added') or []:
                held_add[(e['r'], res_id)].append(e['t'])
    seen_req = set()
    for e in events:
        ev = e.get('ev') or {}
        if e.get('k') != 'agent_event' or ev.get('event') != 'task_commit_request':
            continue
        key = (e['r'], ev.get('task_id'))
        if key in seen_req:
            continue
        seen_req.add(key)
        ts = [t for t in held_add.get((e['r'], f'task:{ev.get("task_id")}'), [])
              if t >= e['t'] - 1]
        ok = bool(ts) and min(ts) - e['t'] <= 35.0
        commits.append({'t': round(e['t'], 2), 'robot': e['r'], 'task_id': ev.get('task_id'),
                        'held_after_s': round(min(ts) - e['t'], 2) if ts else None, 'ok': ok})
    out['commits'] = commits
    out['commit_failures'] = sum(1 for c in commits if not c['ok'])
    # 경합 공허 검사: 같은 작업에 확정 요청을 낸 로봇이 둘 이상
    req_by = collections.defaultdict(set)
    for c in commits:
        req_by[c['task_id']].add(c['robot'])
    out['contention_tasks'] = sorted(t for t, rs in req_by.items() if len(rs) >= 2)
    # 오탐 전환: 장애 주입이 없는 실행에서 NORMAL 밖으로 간 전환 수 (첫 NORMAL 뒤)
    out['false_transitions'] = (None if faults or out['mode'] != 'distributed' else sum(
        1 for d in trans if d['t'] >= t_base and d['mode'] != 'NORMAL'))
    # 영속 파일 크기 (p3 §8-9 · 10): store/<로봇>/{zone,task,fence}.json
    sizes = {}
    store = os.path.join(raw, 'store')
    if os.path.isdir(store):
        for r in sorted(os.listdir(store)):
            for name in ('zone.json', 'task.json', 'fence.json'):
                pth = os.path.join(store, r, name)
                if os.path.isfile(pth):
                    sizes[f'{r}/{name}'] = os.path.getsize(pth)
    out['store_bytes'] = sizes
    # 교통 사건
    tr = collections.Counter(e.get('name') for e in events if e.get('k') == 'traffic')
    out['traffic_counts'] = dict(tr)
    out['deadlocks'] = tr.get('traffic/DEADLOCK', 0)
    out['unresolved'] = tr.get('traffic/UNRESOLVED', 0)
    # plan 신선도: 움직이는 단계(MOVING · RETURNING) 시작 뒤 첫 plan 까지 시간, plan 없이 2 s 넘게 움직인 횟수
    ph_tl = timeline(events, 'phase', 'phase')
    plan_tl = collections.defaultdict(list)
    for e in events:
        if e.get('k') == 'plan' and e.get('r') and e.get('n', 0) > 0:
            plan_tl[e['r']].append(e['t'])
    first_plan = []
    for r, ser in ph_tl.items():
        for (t, ph), nxt in zip(ser, ser[1:] + [(t_end, None)]):
            if ph in ('MOVING', 'RETURNING') and t >= t_base:
                ps = [p for p in plan_tl.get(r, []) if t <= p < nxt[0]]
                first_plan.append({'robot': r, 't': round(t, 2), 'phase': ph,
                                   'first_plan_s': round(ps[0] - t, 2) if ps else None})
    out['first_plan'] = first_plan
    out['moving_without_plan'] = sum(1 for d in first_plan
                                     if d['first_plan_s'] is None or d['first_plan_s'] > 2.0)
    # 도킹
    if meta.get('behavior_text'):
        dk = docking(events, gt, meta['behavior_text'])
        out['docking'] = dk
        errs = [d['pos_err_m'] for d in dk['completed']]
        out['dock_n'] = len(errs)
        out['dock_pos_err_med'] = round(statistics.median(errs), 4) if errs else None
        out['dock_pos_err_max'] = round(max(errs), 4) if errs else None
        out['dock_within_2cm_1deg'] = sum(1 for d in dk['completed']
                                          if d['pos_err_m'] <= 0.02 and d['yaw_err_deg'] <= 1.0)
    # RTF · 메시지
    rows = [(e['t'], e['w']) for e in events if 't' in e and 'w' in e and e['t'] >= t_base]
    if len(rows) >= 2 and rows[-1][1] > rows[0][1]:
        out['rtf'] = round((rows[-1][0] - rows[0][0]) / (rows[-1][1] - rows[0][1]), 4)
    p2p = collections.defaultdict(lambda: [0, 0])
    p2p_path = os.path.join(raw, 'p2p.csv')
    if os.path.isfile(p2p_path):
        with open(p2p_path, encoding='utf-8') as f:
            for row in csv.DictReader(f):
                if float(row['t']) >= t_base:
                    p2p[row['topic']][0] += int(row['count'])
                    p2p[row['topic']][1] += int(row['bytes'])
    span = max(t_end - t_base, 1e-9)
    out['messages_per_sim_s'] = {k: round(v[0] / span, 2) for k, v in p2p.items()}
    out['bytes_per_sim_s'] = {k: round(v[1] / span, 1) for k, v in p2p.items()}
    # 부하
    for name in ('start.txt', 'end.txt'):
        p = os.path.join(raw, name)
        if os.path.isfile(p):
            out[name.split('.')[0] + '_load'] = open(p, encoding='utf-8').read().split('\n')[1:2]
    return out


# ---------------------------------------------------------------------- 통계 · 판정
def sign_test(diffs):
    """양측 정확 부호 검정 (0 은 뺀다). (p, 양수, 음수, 0)."""
    pos = sum(1 for d in diffs if d > 0)
    neg = sum(1 for d in diffs if d < 0)
    zero = len(diffs) - pos - neg
    n = pos + neg
    if n == 0:
        return 1.0, pos, neg, zero
    k = min(pos, neg)
    p = sum(math.comb(n, i) for i in range(0, k + 1)) / 2 ** n * 2
    return min(1.0, p), pos, neg, zero


T975 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306,
        9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145, 15: 2.131}


def holm(pvals: dict) -> dict:
    """Holm 단계 하향 보정 p (조정값, 1 로 자름)."""
    items = sorted(pvals.items(), key=lambda kv: kv[1])
    m, out, run = len(items), {}, 0.0
    for i, (k, p) in enumerate(items):
        run = max(run, min(1.0, (m - i) * p))
        out[k] = run
    return out


def mean_ci(diffs):
    n = len(diffs)
    if n == 0:
        return None, None, None
    m = statistics.mean(diffs)
    if n < 2:
        return m, None, None
    sd = statistics.stdev(diffs)
    h = T975.get(n - 1, 1.96) * sd / math.sqrt(n)
    return m, m - h, m + h


def pearson(a, b):
    if len(a) < 3:
        return None
    try:
        return statistics.correlation(a, b)
    except (statistics.StatisticsError, AttributeError, ZeroDivisionError):
        return None


def compare(pairs, metric, label_a, label_b):
    """짝 [(ma, mb)] 의 metric 차 (b − a)."""
    diffs, rows = [], []
    for ma, mb in pairs:
        va, vb = ma.get(metric), mb.get(metric)
        rows.append([ma.get('pair'), va, vb])
        if va is not None and vb is not None:
            diffs.append(vb - va)
    p, pos, neg, zero = sign_test(diffs)
    m, lo, hi = mean_ci(diffs)
    return {'metric': metric, 'a': label_a, 'b': label_b, 'n': len(diffs), 'diffs': diffs,
            'rows': rows, 'sign_p': p, 'pos': pos, 'neg': neg, 'zero': zero, 'mean': m,
            'ci95': [lo, hi]}


def decide(c, margin=None, direction=None):
    """사전 등록 판정 문구 (가르지 못함을 '같다' 로 쓰지 않는다)."""
    if c['n'] == 0:
        return '판정 불가 (짝 없음)'
    txt = []
    if c['sign_p'] < 0.05:
        txt.append(('b > a' if c['pos'] > c['neg'] else 'b < a')
                   + f' (부호 검정 p = {c["sign_p"]:.4f})')
    else:
        txt.append(f'가르지 못함 (부호 검정 p = {c["sign_p"]:.4f}, +{c["pos"]}/−{c["neg"]}/0 {c["zero"]})')
    if margin is not None and c['ci95'][0] is not None:
        lo = c['ci95'][0]
        txt.append(f'비열등(마진 {margin:g}) ' + ('충족' if lo > -abs(margin) else '확인 안 됨')
                   + f' (95 % CI 하한 {lo:.3g})')
    elif margin is not None and c['n'] >= 1 and all(d == c['diffs'][0] for d in c['diffs']):
        txt.append(f'모든 짝 차가 {c["diffs"][0]:g} (분산 0 — CI 정의 안 됨)')
    return '; '.join(txt)


def zero_events(ms, key):
    return sum(len(m.get(key) or []) if isinstance(m.get(key), list) else len(m.get(key) or {})
               for m in ms)


def exp_summary(exp: str, root: str) -> dict:
    """원자료 루트의 <실험>_p<k>_* 갈래 metrics.json 을 모아 짝 표와 판정."""
    arms = collections.defaultdict(dict)
    for name in sorted(os.listdir(root)):
        m = read_json(os.path.join(root, name, 'metrics.json'))
        if not m or m.get('exp') != exp:
            continue
        arms[m['pair']][m['label']] = m          # 같은 짝 · 갈래의 다시 돈 시도는 이름순 마지막(유효한 것)
    labels = None
    pairs = []
    for k in sorted(arms):
        d = arms[k]
        if len(d) != 2:
            continue
        a, b = sorted(d) if labels is None else labels
        labels = labels or (a, b)
        if not (d[a].get('valid') and d[b].get('valid')):
            continue
        pairs.append((d[a], d[b]))
    res = {'exp': exp, 'labels': labels, 'n_pairs_valid': len(pairs),
           'pairs_seen': sorted(arms), 'invalid': [
               {'pair': k, 'label': lab, 'why': m.get('why'), 'aborted': m.get('aborted')}
               for k, d in sorted(arms.items()) for lab, m in d.items() if not m.get('valid')]}
    if not pairs:
        return res
    la, lb = labels
    metrics = ['completed', 'makespan', 'mean_task_s', 'distance_total_m',
               'distance_to_makespan_m', 'deadlocks', 'failed_reports', 'pickups', 'rtf']
    res['compare'] = {mt: compare(pairs, mt, la, lb) for mt in metrics}
    allm = [m for p in pairs for m in p]
    res['safety'] = {
        'occupancy_disc_events': {m['label'] + f"#{m['pair']}": len(m.get('occupancy_disc', []))
                                  for m in allm},
        'occupancy_center_events': {m['label'] + f"#{m['pair']}":
                                    len(m.get('occupancy_center', [])) for m in allm},
        'dock_fixed_disc_events': {m['label'] + f"#{m['pair']}":
                                   len(m.get('occupancy_dock_fixed_disc', [])) for m in allm},
        'held_overlaps': {m['label'] + f"#{m['pair']}": len(m.get('held_overlaps') or {})
                          for m in allm if m.get('mode') == 'distributed'},
        'low_epoch_accepted': {m['label'] + f"#{m['pair']}": len(m.get('low_epoch_accepted', []))
                               for m in allm},
        'duplicate_pickups': {m['label'] + f"#{m['pair']}": len(m.get('duplicate_pickups', {}))
                              for m in allm},
        'zone_entries': {m['label'] + f"#{m['pair']}": m.get('zone_entries') for m in allm},
        'release_inside': {m['label'] + f"#{m['pair']}": len(m.get('release_inside') or [])
                           for m in allm},
        'unauth_s_total': {m['label'] + f"#{m['pair']}":
                           round(sum((m.get('unauth_s') or {}).values()), 2) for m in allm},
        'commit_failures': {m['label'] + f"#{m['pair']}": m.get('commit_failures')
                            for m in allm},
        'false_transitions': {m['label'] + f"#{m['pair']}": m.get('false_transitions')
                              for m in allm}}
    if exp == 'AA':
        aa = {}
        for mt in ('completed', 'makespan', 'mean_task_s', 'distance_total_m'):
            xa = [p[0].get(mt) for p in pairs]
            xb = [p[1].get(mt) for p in pairs]
            ok = [(x, y) for x, y in zip(xa, xb) if x is not None and y is not None]
            diffs = [y - x for x, y in ok]
            aa[mt] = {'rho': pearson([x for x, _ in ok], [y for _, y in ok]),
                      'diff_sd': statistics.stdev(diffs) if len(diffs) >= 2 else None,
                      'n': len(ok)}
        res['aa'] = aa
        res['divergence'] = [divergence(p[0], p[1]) for p in pairs]
    if exp == 'C1':
        res['decision'] = {'completed': decide(res['compare']['completed'], margin=1.0),
                           'makespan': decide(res['compare']['makespan'])}
    if exp == 'C2':
        res['outage'] = [outage_completions(p) for p in pairs]
        diffs = [o['b'] - o['a'] for o in res['outage']
                 if o['a'] is not None and o['b'] is not None]
        p, pos, neg, zero = sign_test(diffs)
        m, lo, hi = mean_ci(diffs)
        res['outage_compare'] = {'diffs': diffs, 'sign_p': p, 'pos': pos, 'neg': neg,
                                 'zero': zero, 'mean': m, 'ci95': [lo, hi]}
        res['decision'] = {'outage_completed': (
            ('분산 > 중앙' if pos > neg else '분산 < 중앙') + f' (부호 검정 p = {p:.4f})'
            if p < 0.05 else f'가르지 못함 (p = {p:.4f}, +{pos}/−{neg}/0 {zero})'),
            'completed': decide(res['compare']['completed'])}
    if exp == 'C3':
        res['decision'] = {k: decide(res['compare'][k])
                           for k in ('makespan', 'mean_task_s', 'distance_to_makespan_m',
                                     'completed')}
        res['holm'] = holm({k: res['compare'][k]['sign_p']
                            for k in ('makespan', 'mean_task_s', 'distance_to_makespan_m')})
    if exp == 'F5L':
        res['decision'] = {k: decide(res['compare'][k]) for k in ('completed', 'makespan')}
        res['docking'] = {m['label'] + f"#{m['pair']}": {
            'n': m.get('dock_n'), 'within': m.get('dock_within_2cm_1deg'),
            'med': m.get('dock_pos_err_med'), 'max': m.get('dock_pos_err_max'),
            'entries': (m.get('docking') or {}).get('docking_entries')} for m in allm}
    return res


def outage_completions(pair):
    """C2: kill ~ start 사이(sim) 완료 수 (갈래마다 자기 장애 시각)."""
    out = {}
    for key, m in zip(('a', 'b'), pair):
        f = m.get('faults') or []
        tk = next((x.get('sim') for x in f if x.get('action') == 'kill'), None)
        ts = next((x.get('sim') for x in f if x.get('action') == 'start'), None)
        if tk is None or ts is None:
            out[key] = None
            continue
        out[key] = sum(1 for t in m.get('tasks', []) if t.get('completed_t') is not None
                       and tk <= t['completed_t'] < ts)
    out['pair'] = pair[0].get('pair')
    return out


def divergence(ma, mb, thr=1.0):
    """A/A 사건열 발산: t_base 기준 같은 상대 시각에 어느 로봇의 정답 위치든 thr m 넘게 갈라진 첫 시각, 할당 첫 차이."""
    ga, gb = read_gt(os.path.join(ma['raw'], 'gt.csv')), read_gt(os.path.join(mb['raw'], 'gt.csv'))
    ta, tb = ma['t_base'], mb['t_base']
    first = None
    span = min(ma['t_end'] - ta, mb['t_end'] - tb)
    la = list(bins(ga, ta, ta + span))
    lb = list(bins(gb, tb, tb + span))
    for i in range(min(len(la), len(lb))):
        rel = round(i * BIN, 1)
        a, b = la[i][1], lb[i][1]
        for r in set(a) & set(b):
            if math.hypot(a[r][0] - b[r][0], a[r][1] - b[r][1]) > thr:
                first = rel
                break
        if first is not None:
            break
    assign_a = {t['task_id'].split('.')[-1]: t.get('robot') for t in ma.get('tasks', [])}
    assign_b = {t['task_id'].split('.')[-1]: t.get('robot') for t in mb.get('tasks', [])}
    diff = sorted(k for k in assign_a if assign_a.get(k) != assign_b.get(k))
    return {'pair': ma.get('pair'), 'gt_diverge_s': first, 'assign_diff': diff}


# ---------------------------------------------------------------------- 보고
def md_table(res) -> str:
    lines = [f"# P5 {res['exp']} — 짝 {res['n_pairs_valid']} (유효), 갈래 {res.get('labels')}", '']
    for mt, c in (res.get('compare') or {}).items():
        lines.append(f"- {mt}: n={c['n']} 평균차(b−a)={c['mean']} CI={c['ci95']} "
                     f"부호 +{c['pos']}/−{c['neg']}/0 {c['zero']} p={c['sign_p']:.4f} 짝={c['rows']}")
    for k, v in (res.get('decision') or {}).items():
        lines.append(f'- 판정 {k}: {v}')
    if res.get('invalid'):
        lines.append(f"- 무효 갈래: {res['invalid']}")
    return '\n'.join(lines) + '\n'


def registry(m: dict, info: dict) -> dict:
    keep = {k: m.get(k) for k in ('completed', 'n_tasks', 'makespan', 'mean_task_s',
                                  'distance_total_m', 'deadlocks', 'failed_reports', 'pickups',
                                  'rtf', 'valid', 'why', 'zone_entries', 'fence_rejects')}
    keep.update({'occupancy_disc_events': len(m.get('occupancy_disc', []) or []),
                 'occupancy_center_events': len(m.get('occupancy_center', []) or []),
                 'dock_fixed_disc_events': len(m.get('occupancy_dock_fixed_disc', []) or []),
                 'held_overlaps': len(m.get('held_overlaps') or {}),
                 'low_epoch_accepted': len(m.get('low_epoch_accepted') or []),
                 'duplicate_pickups': len(m.get('duplicate_pickups') or {}),
                 'release_inside': len(m.get('release_inside') or []),
                 'contention_tasks': len(m.get('contention_tasks') or []),
                 'commit_failures': m.get('commit_failures'),
                 'false_transitions': m.get('false_transitions'),
                 'unauth_s_total': round(sum((m.get('unauth_s') or {}).values()), 2),
                 'n': 1})
    rec = dict(info)
    rec['result'] = keep
    return rec


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='P5 분석')
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('raw')
    r.add_argument('--registry-info', default='', help='실행 목록 필드 JSON (driver 가 준다)')
    e = sub.add_parser('exp')
    e.add_argument('exp')
    e.add_argument('root')
    e.add_argument('--out', default='')
    a = ap.parse_args(argv)
    if a.cmd == 'run':
        m = run_metrics(a.raw)
        with open(os.path.join(a.raw, 'metrics.json'), 'w', encoding='utf-8') as f:
            json.dump(m, f, ensure_ascii=False, indent=1, default=str)
        if a.registry_info:
            info = json.loads(open(a.registry_info, encoding='utf-8').read())
            with open(os.path.join(a.raw, 'registry.json'), 'w', encoding='utf-8') as f:
                json.dump(registry(m, info), f, ensure_ascii=False, indent=1, default=str)
        print(json.dumps({k: m.get(k) for k in ('valid', 'why', 'completed', 'makespan', 'rtf')},
                         ensure_ascii=False))
        return 0
    if a.cmd == 'exp':
        res = exp_summary(a.exp, a.root)
        out = a.out or a.root
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, f'{a.exp}.json'), 'w', encoding='utf-8') as f:
            json.dump(res, f, ensure_ascii=False, indent=1, default=str)
        with open(os.path.join(out, f'{a.exp}.md'), 'w', encoding='utf-8') as f:
            f.write(md_table(res))
        print(md_table(res))
        return 0
    return 2


if __name__ == '__main__':
    sys.exit(main())
