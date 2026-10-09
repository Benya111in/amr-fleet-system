"""
묶음 설정 파일 생성: config/behavior.yaml, config/fleet_spawn.yaml, config/traffic_zones.yaml.

- behavior.yaml: 저장소 src/amr_behavior/config/behavior.yaml 원문에서 배치가 정하는 줄만 바꾼다 (나머지 주석·튜닝값은
  그대로 — 같은 소프트웨어 설정으로 배치만 바꾸기 위해). 바꾸는 것: docks 의 charger_c1..c3 (staging, marker_pose,
  marker_id), waiting_pose (대기 패드 중심 [x, y, 패드 yaw] — 다중 로봇은 런치가 로봇별 슬롯으로 덮어쓴다),
  relocalization.landmark_ids / landmark_poses (생성기 pillar_marker_poses + rack_marker_poses).
  도크 4 개는 고정.
- fleet_spawn.yaml: 슬롯 5 개 + 저장소 원문의 공통 옵션 값.
- traffic_zones.yaml: zones.derive 결과.
표준 라이브러리만 쓴다 (yaml 대조 시험은 tests/ 에서 컨테이너로).
"""

import math
import re

from layopt.gen.common import load_generator
from layopt.gen.levels import charger_geometry

HEADER = ('# layopt 배치 묶음 {spec_id} 의 생성본 — layopt/gen/configs.py 가 {src} 에서 만들었다.\n'
          '# 직접 고치지 말 것 (docs/research/layout-opt/gen.md).\n')


def _f(v, nd=4):
    v = round(float(v), nd)
    return repr(0.0 if v == 0 else v)


def _yaw(v):
    return _f(math.atan2(math.sin(v), math.cos(v)), 8)


def landmarks():
    """(ids, poses 평탄 목록) — 기둥 → 랙 마커, id 순 (behavior.yaml 형식: x, y 3 자리, yaw 4 자리)."""
    gen = load_generator()
    want = {**gen.pillar_marker_poses(), **gen.rack_marker_poses()}
    ids = sorted(want)
    poses = []
    for i in ids:
        x, y, yaw = want[i]
        poses.append((_f(x, 3), _f(y, 3), _f(yaw, 4)))
    return ids, poses


def behavior_yaml(src_text, res, spec_id, src_name='src/amr_behavior/config/behavior.yaml'):
    """원문 behavior.yaml → 배치 반영본 (현재 생성기 LAY 가 res 의 배치여야 한다 — build 가 set_layout 한다)."""
    text = src_text
    for c in res['chargers']:
        mk, st, _dk = charger_geometry(c)
        line = (f"      charger_{c['id']}: {{staging: [{_f(st[0])}, {_f(st[1])}, {_yaw(st[2])}], "
                f"standoff: 0.65, marker_id: {c['marker_id']}, marker_pose: [{_f(mk[0])}, "
                f"{_f(mk[1])}, {_yaw(mk[2])}]}}")
        text, n = re.subn(rf"^      charger_{c['id']}: \{{.*\}}$", line, text, flags=re.M)
        if n != 1:
            raise ValueError(f"behavior.yaml 에 charger_{c['id']} 줄이 {n} 개")
    wx, wy, wyaw = res['gen_spec']['waiting_pad']
    text, n = re.subn(r'^(    waiting_pose: )\[[^\]]*\]',
                      lambda m: f'{m.group(1)}[{_f(wx)}, {_f(wy)}, {_f(wyaw)}]', text, flags=re.M)
    if n != 1:
        raise ValueError(f'behavior.yaml 에 waiting_pose 줄이 {n} 개')
    ids, poses = landmarks()
    gen = load_generator()
    n_p = 4 * len(gen.LAY['pillars'])
    body = []
    for i in range(0, n_p, 4):
        body.append('        ' + ', '.join(', '.join(p) for p in poses[i:i + 4])
                    + f',   # 기둥 {i // 4 + 1}')
    rest = poses[n_p:]
    k = 0
    for row, (y, xs) in sorted(gen.rows_of(gen.LAY['racks']).items(), key=lambda kv: -kv[1][0]):
        part = rest[k:k + 2 * len(xs)]
        k += len(part)
        body.append('        ' + ', '.join(', '.join(p) for p in part)
                    + (',' if k < len(rest) else '') + f'   # 랙 {row} 열 y={y:g} (서→동, 북면·남면)')
    assert k == len(rest), (k, len(rest))
    block = (f"      landmark_ids: [{', '.join(str(i) for i in ids)}]"
             f'   # 기둥 {n_p} + 랙 {len(rest)}\n'
             f'      landmark_poses: [   # [x, y, yaw] × {len(ids)}'
             ' — layopt 생성 (gen_warehouse_world.py 값)\n'
             + '\n'.join(body) + '\n      ]')
    text, n = re.subn(r'^      landmark_ids: \[.*?^      \][^\n]*', lambda m: block, text,
                      flags=re.M | re.S)
    if n != 1:
        raise ValueError(f'behavior.yaml 에 landmark 블록이 {n} 개')
    return HEADER.format(spec_id=spec_id, src=src_name) + text


def fleet_spawn_yaml(res, spec_id):
    src = 'src/amr_bringup/config/fleet_spawn.yaml'
    lines = [HEADER.format(spec_id=spec_id, src=src).rstrip(),
             f"# 대기 구역 {res['waiting']['key']} (중심 {res['waiting']['center']}), "
             f"배열 {res['level_keys']['V2']}. 형식: amr_bringup/fleet_spawn.py", '', 'robots:']
    for s in res['slots']:
        lines.append(f"  - {{name: {s['robot']}, x: {_f(s['x'])}, y: {_f(s['y'])}, "
                     f"yaw: {_f(math.atan2(math.sin(s['yaw']), math.cos(s['yaw'])), 4)}}}")
    lines += ['', 'spawn_z: 0.02', 'pause_during_spawn: true', 'spawn_timeout_s: 120.0',
              'comm_latency_ms: null', 'min_separation_m: 1.0', '']
    return '\n'.join(lines)


def traffic_zones_yaml(tz, spec_id, notes=()):
    lines = [HEADER.format(spec_id=spec_id, src='배치 기하 (layopt/gen/zones.py)').rstrip(),
             '# 규칙: zones.py 머리말 (현행 손 작성 traffic_zones.yaml 의 규칙을 모든 배치에 같게)']
    lines += [f'# {n}' for n in notes]
    lines += [f"frame_id: {tz['frame_id']}", 'zones:']
    for z in tz['zones']:
        lines.append(f"  - id: {z['id']}")
        lines.append(f"    kind: {z['kind']}")
        if 'same_direction' in z:
            lines.append(f"    same_direction: {'true' if z['same_direction'] else 'false'}")
        lines.append('    polygon: [' + ', '.join(f'[{_f(x)}, {_f(y)}]' for x, y in z['polygon'])
                     + ']')
    lines.append('pockets:')
    for p in tz['pockets']:
        lines.append(f"  - {{id: {p['id']}, x: {_f(p['x'])}, y: {_f(p['y'])}, "
                     f"yaw: {_f(p['yaw'])}}}")
    return '\n'.join(lines) + '\n'
