#!/usr/bin/env python3
"""
녹화 시나리오 경로 파일 생성·검사 (보너스 2 G 단계) — config/routes/*.yaml 과 config/spawn/*.yaml.

    make_routes.py write [--dir src/amr_slam3d/config]
        경로·스폰 파일을 쓴다 (결정적, 시험이 대조한다)
    make_routes.py check --gt-band <band_z0.050-2.000_r*.yaml> --out clearance.json
        정답 높이 대역 래스터로 장애물 여유 검사

시나리오 (사전 등록 — docs/research/slam3d/notebook/entries/*_b2-rec_*.md, recordings.md):
  s1_seed1·2·3 : S1 정상 (월드 --variant s1), routegen 씨드 걸음 60 m + 출발 마디로 귀환
  map_long     : 긴 매핑 경로 (월드 s1), 차선 지그재그 308.6 m
  s3_aisle     : S3 좁은 통로 왕복 (월드 s1) (0, −5.5) → 중앙 (0, −10) 15 s 정지 → (0, −14.5)
                 → (0, −5.5)
  s6_inside    : S6 통로 안 (월드 s6-open/s6-queue/s6-wall 공통) — 스폰 (0, −8.6, −π/2) 통로 안,
                 중앙 15 s 정지 → 남쪽 끝 (0, −11.4) → 후진으로 (0, −8.6). sim 60 s 에 출발 (변형 짝)
"""

import argparse
import json
import math
import os
import sys

from amr_slam3d import routegen
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
COMMON = {'hold_start_s': 10.0, 'hold_end_s': 3.0, 'dwell_s': 1.0,
          'limits': {'v_max': 0.5, 'w_max': 0.5}}
S6_SPAWN = (0.0, -8.6, -math.pi / 2)
S6_DEPART = 60.0


def _wp(p):
    return [round(float(p[0]), 3), round(float(p[1]), 3)]


def routes():
    """이름 → (머리말, route dict, 스폰 (x, y, yaw))."""
    sx, sy, syaw = routegen.SPAWN
    out = {}
    for seed in (1, 2, 3):
        w = routegen.s1_waypoints(seed)
        out[f's1_seed{seed}'] = (
            f'S1 정상 — routegen 씨드 {seed} 걸음 (60 m 뒤 출발 마디로 귀환), '
            f'길이 {routegen.path_length([(sx, sy)] + w):.1f} m',
            dict(start=[sx, sy], start_yaw=round(syaw, 4), **COMMON,
                 waypoints=[_wp(p) for p in w]), routegen.SPAWN)
    m = routegen.map_waypoints()
    out['map_long'] = (
        f'MAP 긴 매핑 — 차선 지그재그, 길이 {routegen.path_length([(sx, sy)] + m):.1f} m '
        '(slam.md §6.1 의 322 m 원 웨이포인트는 저장소에 없다 — 같은 차선으로 다시 만든 것)',
        dict(start=[sx, sy], start_yaw=round(syaw, 4), **COMMON,
             waypoints=[_wp(p) for p in m]), routegen.SPAWN)
    s3 = [[18.0, -5.5], [0.0, -5.5], {'at': [0.0, -10.0], 'dwell': 15.0},
          {'at': [0.0, -14.5], 'dwell': 2.0}, [0.0, -5.5]]
    out['s3_aisle'] = (
        'S3 좁은 통로 왕복 — (0, −5.5) → 중앙 (0, −10) 15 s 정지 → (0, −14.5) → (0, −5.5) '
        '(evaluation.md §6)',
        dict(start=[sx, sy], start_yaw=round(syaw, 4), **COMMON, waypoints=s3), routegen.SPAWN)
    s6 = [{'at': [0.0, -10.0], 'dwell': 15.0}, {'at': [0.0, -11.4], 'dwell': 2.0},
          {'at': [0.0, -8.6], 'dwell': 2.0, 'reverse': True}]
    out['s6_inside'] = (
        'S6 통로 안 — 스폰 (0, −8.6, −π/2), 중앙 15 s 정지 → (0, −11.4) → 후진 (0, −8.6). '
        f'sim {S6_DEPART:g} s 에 출발 (s6-open·queue·wall 짝)',
        dict(start=list(S6_SPAWN[:2]), start_yaw=round(S6_SPAWN[2], 4), **COMMON,
             depart_at_s=S6_DEPART, waypoints=s6), S6_SPAWN)
    return out


def spawn_doc(pose, note):
    """multi_robot.launch.py robots_file 형식 (amr_bringup fleet_spawn.py) — amr_01(3D) 1 대."""
    x, y, yaw = pose
    robot = {'name': 'amr_01', 'x': float(x), 'y': float(y), 'yaw': round(float(yaw), 4),
             'lidar3d': True}
    return {'robots': [robot],
            'spawn_z': 0.02, 'pause_during_spawn': True, 'spawn_timeout_s': 120.0,
            'comm_latency_ms': None, 'min_separation_m': 1.0}, note


def cmd_write(ns) -> int:
    rdir = os.path.join(ns.dir, 'routes')
    sdir = os.path.join(ns.dir, 'spawn')
    os.makedirs(rdir, exist_ok=True)
    os.makedirs(sdir, exist_ok=True)
    for name, (head, route, _) in routes().items():
        text = (f'# {head}\n# 생성: scripts/make_routes.py write '
                '(손으로 고치지 않는다 — test_route.py 가 대조)\n'
                + yaml.safe_dump({'route': route}, allow_unicode=True, sort_keys=False))
        with open(os.path.join(rdir, f'{name}.yaml'), 'w', encoding='utf-8') as f:
            f.write(text)
    doc, note = spawn_doc(S6_SPAWN, 'S6 통로 안 스폰 — amr_01(3D) 1 대, 좁은 통로 북쪽 안 (0, −8.6), 남쪽을 향함')
    with open(os.path.join(sdir, 's6_inside.yaml'), 'w', encoding='utf-8') as f:
        f.write(f'# {note}\n# 생성: scripts/make_routes.py write\n'
                + yaml.safe_dump(doc, allow_unicode=True, sort_keys=False))
    print(f'wrote {len(routes())} routes → {rdir}, spawn → {sdir}')
    return 0


def _read_band(path):
    meta = yaml.safe_load(open(path, encoding='utf-8'))
    pgm = os.path.join(os.path.dirname(path), meta['image'])
    data = open(pgm, 'rb').read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    w, h = int(tokens[1]), int(tokens[2])
    img = np.frombuffer(data, dtype=np.uint8, count=w * h, offset=pos + 1).reshape(h, w)
    p = (255.0 - img) / 255.0 if not meta.get('negate', 0) else img / 255.0
    occ = (p > float(meta.get('occupied_thresh', 0.65)))[::-1]
    return occ, float(meta['resolution']), (float(meta['origin'][0]), float(meta['origin'][1]))


def cmd_check(ns) -> int:
    occ, res, origin = _read_band(ns.gt_band)
    report = {'gt_band': os.path.abspath(ns.gt_band), 'routes': {}}
    ok = True
    for name, (_, route, spawn) in routes().items():
        wps = [p['at'] if isinstance(p, dict) else p for p in route['waypoints']]
        r = routegen.check_clearance(spawn[:2], wps, occ, res, origin)
        report['routes'][name] = r
        ok &= r['ok']
        worst = min(r['segments'], key=lambda s: s['worst_margin'])
        print(f"{name:10s} ok={r['ok']} 최악 선분 여유 {worst['worst_margin']:+.3f} m "
              f"({worst['worst_at']}, 요구 {worst['need_there']}) 회전 지점 "
              f"{min([t['clearance'] for t in r['turns']] or [math.nan]):.3f} m")
    report['ok'] = bool(ok)
    with open(ns.out, 'w', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=1)
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    w = sub.add_parser('write')
    w.add_argument('--dir', default=os.path.join(HERE, '..', 'config'))
    c = sub.add_parser('check')
    c.add_argument('--gt-band', required=True)
    c.add_argument('--out', required=True)
    ns = ap.parse_args(argv)
    return {'write': cmd_write, 'check': cmd_check}[ns.cmd](ns)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
