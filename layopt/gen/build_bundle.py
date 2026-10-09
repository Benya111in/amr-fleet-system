#!/usr/bin/env python3
"""
배치 묶음 하나 만들기 (contract §1).

spec.json, layout.json, world.sdf, map.pgm/map.yaml, config/behavior.yaml, config/fleet_spawn.yaml,
config/traffic_zones.yaml, valid.json (+ 봉인 SHA256SUMS, SEALED).

    # 컨테이너 안 (numpy·scipy·yaml) — 저장소를 /ros2_ws, 실행 결과 루트를 같은 경로로 마운트
    python3 -m layopt.gen.build_bundle --spec v10-v20-v30-v40-v50 \
        --out /home/CAPYI_AI/runs/layopt/bundles/v10-v20-v30-v40-v50 --commit <sha> [--dirty]
    # 호스트에서는 layopt/gen/run_build.sh 가 커밋을 재고 컨테이너로 부른다

--spec 은 spec_id 또는 {"levels": {"V1": i, …}} JSON 파일. 이미 있는 --out 은 거부한다 (덮어쓰지 않는다).
임시 디렉토리(<out>.tmp-<pid>)에 다 쓴 뒤 이름을 바꾸고 봉인한다. 무효 배치(valid=false)는 spec.json ·
layout.json(해석됐으면) · valid.json 만 쓰고 봉인한다 (월드·지도·설정 없음 — 실행 대상이 아니다).
"""

import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from layopt.gen import configs, dwell, levels as L, zones as Z  # noqa: E402
from layopt.gen.common import BEHAVIOR_SRC, BUNDLES, load_generator, r4  # noqa: E402

SEAL = '/home/CAPYI_AI/runs/layopt/bin/seal'


def _levels_from_arg(text):
    if os.path.isfile(text):
        with open(text, encoding='utf-8') as f:
            data = json.load(f)
        lv = data.get('levels', data)
        return {k: int(lv[k]) for k in ('V1', 'V2', 'V3', 'V4', 'V5')}
    return L.parse_spec_id(text)


def layout_json(res, gen, tz, dw, notes):
    """계약 §2 layout.json (+ 정보: level_keys, waiting_zone, dynamic_lanes, gen_spec, notes)."""
    lay = res['gen_spec']
    racks = [{'id': r['id'], 'row': r['row'], 'kind': 'main', 'x': r4(r['x']), 'y': r4(r['y']),
              'yaw': r4(r.get('yaw', 0.0)), 'l': gen.RACK_L, 'd': gen.RACK_D}
             for r in lay['racks']]
    racks += [{'id': r['id'], 'row': None, 'kind': 'narrow', 'x': r4(r['x']), 'y': r4(r['y']),
               'yaw': r4(r['yaw']), 'l': gen.RACK_L, 'd': gen.RACK_D} for r in L.narrow_racks()]
    docks = []
    for name, (x, y, kind, _mid) in gen.DOCKS.items():
        sx, sy = dwell_pose(dw, f'dock:{name}')
        docks.append({'id': name, 'kind': kind, 'pad': [x, y], 'staging': [sx[0], sx[1], sy]})
    chargers = []
    for c in res['chargers']:
        mk, st, dk = L.charger_geometry(c)
        chargers.append({'id': c['id'], 'pad': [r4(c['x']), r4(c['y'])], 'yaw': r4(c['yaw']),
                         'staging': [r4(v) for v in st], 'marker_pose': [r4(v) for v in mk],
                         'marker_id': c['marker_id'], 'docked': [r4(v) for v in dk]})
    markers = []
    for name, (x, y, kind, mid) in gen.DOCKS.items():
        mx = -gen.HALF_X + 0.01 if kind == 'inbound' else gen.HALF_X - 0.01
        yaw = 0.0 if kind == 'inbound' else r4(math.pi)
        markers.append({'id': mid, 'x': mx, 'y': y, 'yaw': yaw, 'kind': 'dock'})
    for c in chargers:
        markers.append({'id': c['marker_id'], 'x': c['marker_pose'][0], 'y': c['marker_pose'][1],
                        'yaw': c['marker_pose'][2], 'kind': 'charger'})
    for mid, (x, y, yaw) in sorted(gen.pillar_marker_poses().items()):
        markers.append({'id': mid, 'x': r4(x), 'y': r4(y), 'yaw': r4(yaw), 'kind': 'pillar'})
    for mid, (x, y, yaw) in sorted(gen.rack_marker_poses().items()):
        markers.append({'id': mid, 'x': r4(x), 'y': r4(y), 'yaw': r4(yaw), 'kind': 'rack'})
    return {
        'spec_id': res['spec_id'], 'levels': res['levels'], 'level_keys': res['level_keys'],
        'bounds': [-gen.HALF_X, gen.HALF_X, -gen.HALF_Y, gen.HALF_Y],
        'racks': racks,
        'pillars': [{'id': f'P{i}', 'x': float(p[0]), 'y': float(p[1]), 'half': gen.PILLAR_HALF}
                    for i, p in enumerate(lay['pillars'], 1)],
        'structures': [{k: (r4(v) if isinstance(v, float) else v) for k, v in s.items()}
                       for s in lay['structures']],
        'docks': docks, 'chargers': chargers,
        'waiting_slots': [{'robot': s['robot'], 'x': r4(s['x']), 'y': r4(s['y']),
                           'yaw': r4(s['yaw'])} for s in res['slots']],
        'waiting_zone': res['waiting'],
        'narrow_aisle': {'cx': gen.NARROW_CX, 'cy': gen.NARROW_CY, 'clear': gen.NARROW_CLEAR},
        'markers': markers,
        'dwell_points': dw,
        'traffic': {'n_zones': len(tz['zones']), 'n_pockets': len(tz['pockets']),
                    'zone_ids': [z['id'] for z in tz['zones']]},
        'dynamic_lanes': {'actors': {n: {k: v for k, v in lane.items() if k != 'edges'}
                                     for n, lane in lay['lanes'].items()},
                          'vehicles': {n: {'axis': row[0], 'lo': row[1], 'hi': row[2],
                                           'fixed': row[3]}
                                       for n, row in gen.vehicle_table().items()}},
        'notes': notes,
        'gen_spec': lay,
    }


def dwell_pose(dw, pid):
    for p in dw:
        if p['id'] == pid:
            return (p['x'], p['y']), p['yaw']
    raise KeyError(pid)


def build(levels, out, commit, dirty, seal=True):
    """묶음 하나 → valid.json 내용 (dict). out 이 이미 있으면 FileExistsError."""
    from layopt.gen import raster as R, validate as V
    out = Path(out)
    if out.exists():
        raise FileExistsError(f'이미 있다 (덮어쓰지 않는다): {out}')
    sid = L.spec_id(levels)
    tmp = out.parent / f'{out.name}.tmp-{os.getpid()}'
    if tmp.exists():
        shutil.rmtree(tmp)
    tmp.mkdir(parents=True)
    gen = load_generator()
    spec = {'spec_id': sid, 'levels': levels,
            'level_keys': {k: L.LEVELS[k][v]['key'] for k, v in levels.items()},
            'generator_commit': commit, 'generator_dirty': bool(dirty),
            'created': dt.datetime.now().astimezone().isoformat(timespec='seconds'),
            'builder': 'layopt/gen/build_bundle.py'}
    (tmp / 'spec.json').write_text(json.dumps(spec, ensure_ascii=False, indent=1) + '\n',
                                   encoding='utf-8')
    reasons, info = [], {}
    res = None
    try:
        res = L.resolve(levels)
    except L.LayoutError as exc:
        reasons.append(f'resolve:{exc}')
    if res is not None:
        notes = list(res['notes'])
        try:
            sdf, _info = gen.build_sdf(res['gen_spec'])
        except AssertionError as exc:
            reasons.append(f'generator:{exc}')
            sdf = None
        obstacles = L._static_obstacles(res['gen_spec'])
        tz, znotes = Z.derive(res, obstacles)
        notes += znotes
        dw = dwell.dwell_points(res, tz, obstacles)
        if sdf is not None:
            (tmp / 'world.sdf').write_text(sdf, encoding='utf-8')
            raster, unknown, skipped = R.rasterize_world(tmp / 'world.sdf')
            R.write_map(raster, unknown, tmp / 'map')
            info['raster_skipped'] = skipped
            zones_text = configs.traffic_zones_yaml(tz, sid, znotes)
            ok, vr, vinfo = V.check(res, gen, raster, unknown, tz, obstacles, zones_text)
            reasons += vr
            info.update(vinfo)
            info['occupied_cells'] = int(raster.occupied.sum())
        lj = layout_json(res, gen, tz, dw, notes)
        (tmp / 'layout.json').write_text(json.dumps(lj, ensure_ascii=False, indent=1) + '\n',
                                         encoding='utf-8')
        info['n_dwell'] = {k: sum(1 for p in dw if p['kind'] == k)
                           for k in ('waiting', 'dock', 'charger', 'queue')}
        if not reasons:
            cfg = tmp / 'config'
            cfg.mkdir()
            (cfg / 'behavior.yaml').write_text(
                configs.behavior_yaml(BEHAVIOR_SRC.read_text(encoding='utf-8'), res, sid),
                encoding='utf-8')
            (cfg / 'fleet_spawn.yaml').write_text(configs.fleet_spawn_yaml(res, sid),
                                                  encoding='utf-8')
            (cfg / 'traffic_zones.yaml').write_text(zones_text, encoding='utf-8')
        else:
            for name in ('world.sdf', 'map.pgm', 'map.yaml'):
                if (tmp / name).exists():
                    (tmp / name).unlink()
    valid = {'spec_id': sid, 'valid': not reasons, 'reasons': reasons,
             'reason_kinds': sorted({r.split(':', 1)[0] for r in reasons}), 'info': info}
    (tmp / 'valid.json').write_text(json.dumps(valid, ensure_ascii=False, indent=1) + '\n',
                                    encoding='utf-8')
    os.rename(tmp, out)
    if seal:
        subprocess.run(['bash', SEAL, str(out)], check=True, capture_output=True)
    return valid


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--spec', required=True, help='spec_id 또는 {"levels": …} JSON')
    ap.add_argument('--out', default='', help='묶음 디렉토리 (기본 runs/layopt/bundles/<spec_id>)')
    ap.add_argument('--commit', required=True, help='생성기 커밋 (호스트 git rev-parse HEAD)')
    ap.add_argument('--dirty', action='store_true', help='작업 트리에 커밋 안 한 변경이 있었다')
    ap.add_argument('--no-seal', action='store_true')
    a = ap.parse_args(argv)
    levels = _levels_from_arg(a.spec)
    out = a.out or str(BUNDLES / L.spec_id(levels))
    try:
        v = build(levels, out, a.commit, a.dirty, seal=not a.no_seal)
    except FileExistsError as exc:
        print(exc, file=sys.stderr)
        return 3
    except Exception:  # noqa: B902 — 실패도 원인을 남긴다
        traceback.print_exc()
        return 1
    print(json.dumps({k: v[k] for k in ('spec_id', 'valid', 'reasons')}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
