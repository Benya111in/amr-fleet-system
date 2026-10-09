#!/usr/bin/env python3
"""
봉인 묶음의 queue 체류 지점 오류 목록 (2026-10-10 dwell.approach_blocked 수정 전 생성분).

    python3 -m layopt.gen.errata_queue \
        --out /home/CAPYI_AI/runs/layopt/tmp/gen/errata/<run_id>.json

묶음은 덮어쓰지 않는다. 대신 묶음마다 layout.json 의 queue 점 중 수정된 규칙으로는 빠졌어야 할 id 를 적는다
(구역 쪽 접근 선분이 장애물 0.25 m 안 — 좁은 통로 옆면, 도크 화물 너머, 랙이 막은 T 자 교차로). 소비자(E0·E1)는
이 id 를 빼고 쓴다. 장애물은 layout.json 의 랙(좁은 통로 포함)·기둥·구조물·충전 스테이션과 도크 화물(생성기 규칙).
"""

import argparse
import glob
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from layopt.gen import levels as L  # noqa: E402
from layopt.gen.common import BUNDLES  # noqa: E402
from layopt.gen.dwell import approach_blocked  # noqa: E402
from layopt.gen.zones import APPROACH_M, CORRIDOR_APPROACH_M  # noqa: E402


def blocked_ids(lj):
    obs = [L.rack_aabb(r) for r in lj['racks']]
    obs += [L.pillar_aabb((p['x'], p['y']), p['half']) for p in lj['pillars']]
    obs += [L.struct_aabb(s) for s in lj['structures']]
    obs += [L.station_aabb({'x': c['pad'][0], 'y': c['pad'][1], 'yaw': c['yaw']})
            for c in lj['chargers']]
    obs += [b for _n, b in L.dock_boxes()]
    out = []
    for p in lj['dwell_points']:
        if p['kind'] != 'queue':
            continue
        zone = p['id'].split(':')[1]
        d = CORRIDOR_APPROACH_M if zone == 'narrow_aisle' else APPROACH_M
        if approach_blocked(p['x'], p['y'], p['yaw'], d, obs):
            out.append(p['id'])
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    res = {}
    for f in sorted(glob.glob(str(BUNDLES / 'v1*' / 'layout.json'))):
        lj = json.load(open(f, encoding='utf-8'))
        res[lj['spec_id']] = blocked_ids(lj)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, 'x', encoding='utf-8') as f:
        json.dump({'rule': 'dwell.approach_blocked (APPROACH_CLEAR 0.25 m)',
                   'drop_queue_ids': res}, f, ensure_ascii=False, indent=1)
    n = sum(len(v) for v in res.values())
    print(json.dumps({'bundles': len(res), 'drop_total': n,
                      'per_bundle': sorted({len(v) for v in res.values()})}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
