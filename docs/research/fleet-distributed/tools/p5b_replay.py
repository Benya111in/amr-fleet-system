#!/usr/bin/env python3
"""
P5 원판 동시 점유 사건 15 건을 고친 경로 게이트로 다시 판정한다 [계산 — 오프라인, 근사].

P5 원자료에는 로봇 plan 전체가 없다 (`plan` 사건은 점 수 · 끝점만). 그래서 허가 없는 로봇의 **실제로 간 정답 궤적**(사건
30 s 전부터 끝 10 s 뒤까지)을 그 로봇의 plan 대신 넣고, 사건 전 0.2 s 마다 옛 판정(body_entry=False)과 몸체 판정
(body_entry=True)의 경로 게이트 결정을 낸다 (held 는 비움 — 그 로봇은 그 구역 허가가 없었다). 근사: 실제 plan 은 궤적과
다를 수 있고(재계획), 막혔다면 궤적 자체가 달라졌을 것이다 — 이 계산은 "그 궤적을 따라갔다면 언제 요청 · 막았나" 만 본다.

각 사건: 옛 판정이 그 구역을 처음 요청 · 막은 시각 (P5 의 실제 `gate` 기록과 대조 — 재현 충실도), 몸체 판정이 처음
요청 · 막은 시각과 그때 원판 진입까지 거리(first_entry_m), 막힌 자리에서 원판이 실제로 구역에 처음 닿기까지 궤적을 따라 간
거리(여유 — P5 에서 잰 막힌 뒤 정지 거리 0.80–0.81 m 보다 커야 막힌 채 몸체가 밖에 선다).

쓰기: python3 p5b_replay.py [P5 원자료] > out.txt   (src/amr_fleet 을 PYTHONPATH 에 — 컨테이너)
"""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', '..', 'tests', 'integration', 'fleet_p5'))
import p5_analyze as A  # noqa: E402
import p5_review_check as RC  # noqa: E402

from amr_fleet.agent.path_gate import PathGate, PathGateConfig  # noqa: E402
from amr_fleet.traffic_zones import approach_map, zones_from_config  # noqa: E402

OVERSHOOT_M = 0.81


def gates(zones_text):
    """그 실행의 배치로 옛 · 몸체 판정 경로 게이트 (traffic.yaml 기본 요청 거리 2.0 · 통로 2.5, 사전 요청 1.0)."""
    import yaml
    doc = yaml.safe_load(zones_text)
    zs = zones_from_config(doc['zones'], True)
    appr = approach_map(zs, 2.0, 2.5)
    return (PathGate(zs, appr, PathGateConfig(prerequest_m=1.0, body_entry=False)),
            PathGate(zs, appr, PathGateConfig(prerequest_m=1.0, body_entry=True)))


def track(gt, r, t0, t1):
    """[t0, t1] 정답 궤적 [(t, x, y, yaw)] (0.1 s 칸)."""
    return [(s[0], s[1], s[2], s[3]) for s in gt.get(r, []) if t0 <= s[0] <= t1]


def future_path(tr, k):
    """궤적의 k 번째 점부터 끝까지 점열 (2 cm 안 이웃 점은 뺀다)."""
    pts = [tr[k][1:3]]
    for s in tr[k + 1:]:
        if math.dist(pts[-1], s[1:3]) >= 0.02:
            pts.append(s[1:3])
    return np.array(pts, dtype=float)


def replay(raw, tag, inc):
    """사건 하나 → 옛 · 새 판정의 첫 요청 · 막힘과 여유."""
    with open(os.path.join(raw, tag, 'meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    old, new = gates(meta['zones_text'])
    zones = A.load_zones(meta['zones_text'])
    gt = A.read_gt(os.path.join(raw, tag, 'gt.csv'))
    r, z, t0 = inc['robot'], inc['zone'], inc['t0']
    tr = track(gt, r, t0 - 30.0, t0 + inc['dur'] + 10.0)
    key = f'zone:{z}'
    touch = next((s for s in tr if A.disc_overlaps(zones[z], s[1], s[2])), None)
    out = {'tag': tag, 'robot': r, 'zone': z, 't0': t0, 'mech_p5': inc['mech'],
           'p5_gate_req': inc['requested'], 'p5_gate_block': inc['blocking'],
           'touch_t': None if touch is None else round(touch[0], 2)}
    for name, g in (('old', old), ('new', new)):
        req = blk = None
        for k in range(0, len(tr) - 1, 2):
            s = tr[k]
            if s[0] > t0:
                break
            path = future_path(tr, k)
            if len(path) < 2:
                continue
            d = g.decide((s[1], s[2], s[3]), path, set())
            if req is None and key in d.request:
                fe = None if d.first_entry_m is None else round(d.first_entry_m, 2)
                req = (round(s[0], 1), fe)
            if blk is None and key in d.blocking:
                blk = (round(s[0], 1), round(d.first_entry_m, 2), (round(s[1], 2), round(s[2], 2)))
        out[f'{name}_req'] = req
        out[f'{name}_block'] = blk
        if blk is not None and touch is not None:
            # 막힌 자리 → 원판이 처음 닿은 자리까지 궤적 길이 (막힌 뒤 계속 갔다면)
            i0 = next(i for i, s in enumerate(tr) if s[0] >= blk[0])
            i1 = next(i for i, s in enumerate(tr) if s[0] >= touch[0])
            ln = sum(math.dist(tr[i][1:3], tr[i + 1][1:3]) for i in range(i0, max(i0, i1)))
            out[f'{name}_margin_m'] = round(ln, 2) if i1 > i0 else 0.0
    return out


def main(argv):
    raw = argv[1] if len(argv) > 1 else '/home/CAPYI_AI/runs/b24/b4-p5/raw'
    dist, _ = RC.final_arms(raw)
    rows = []
    for tag in dist + [RC.PRE_FIX_ARM]:
        events = A.read_jsonl(os.path.join(raw, tag, 'events.jsonl'))
        gt = A.read_gt(os.path.join(raw, tag, 'gt.csv'))
        held = A.timeline(events, 'agent_state', 'held')
        for inc in RC.incidents(raw, tag, events, gt, held):
            rows.append(replay(raw, tag, inc))
    for row in rows:
        print(json.dumps(row, ensure_ascii=False))
    n = len(rows)
    new_ok = sum(1 for x in rows if x.get('new_margin_m') is not None
                 and x['new_margin_m'] > OVERSHOOT_M)
    old_none = sum(1 for x in rows if x.get('old_req') is None)
    print(f'# 사건 {n}: 옛 판정 요청 없음 {old_none}, 몸체 판정이 원판 닿기 전 막음 '
          f'{sum(1 for x in rows if x.get("new_block"))}, 그중 막힌 자리 → 닿는 자리 > {OVERSHOOT_M} m '
          f'{new_ok}')
    margins = sorted(x['new_margin_m'] for x in rows if x.get('new_margin_m') is not None)
    print(f'# 몸체 판정 여유 [m] 최소 {margins[0] if margins else None} 중앙 '
          f'{margins[len(margins) // 2] if margins else None}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
