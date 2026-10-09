"""
설계 변수 V1–V5 의 수준 정의와 spec_id → 배치 사양(LayoutSpec) 해석.

수준 표와 근거는 docs/research/layout-opt/levels.md (사전 등록 항목 — 결과를 보기 전에 커밋한다).
spec_id 형식 (contract §1): v1<i>-v2<j>-v3<k>-v4<l>-v5<m>, 0 = 현행. 기준 배치 v10-v20-v30-v40-v50 의 사양은
월드 생성기 default_layout() 과 같다 (시험 test_levels.py).

해석 결과 resolve(levels) → dict:
  gen_spec      월드 생성기 --spec 사양 (default_layout() 과 같은 키)
  slots         대기 슬롯 [{robot, x, y, yaw}] (amr_01 … amr_05)
  waiting       대기 구역 {center, u, f, wall, key}
  chargers      [{id, x, y, yaw, marker_id}] (스테이션 중심, yaw = 판이 보는 방향)
  notes         해석 중 정한 것 (경로 재배치 등)
배치가 고정 조건을 못 지키면 LayoutError(이유) — build 가 valid.json 의 reasons 로 남긴다.
표준 라이브러리만 쓴다.
"""

import copy
import math
import re

from layopt.gen.common import load_generator, MARKER_OFF, STAGING_FROM_PLATE, STATION_WALL_OFF

HALF_X, HALF_Y = 30.0, 20.0
RACK_L, RACK_D = 2.0, 1.0
ROBOTS = ['amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05']
_S, _N, _E, _W = math.pi / 2, -math.pi / 2, math.pi, 0.0
PLATE_OFF = 0.012


class LayoutError(Exception):
    """고정 조건 위반 (이유 문자열 = 'kind:detail')."""


# ---------------------------------------------------------------------------- 수준 표
# V1 대기 구역: 중심, 슬롯 축 u, 로봇이 보는 방향 f(= 창고 안쪽), 뒤 벽.
V1 = [
    {'key': 'se_current', 'center': (22.0, -16.0), 'u': (1.0, 0.0), 'f': (0.0, 1.0), 'wall': 'S',
     'desc': '현행 남동 (22, -16), 남쪽 벽 앞, 북쪽을 본다'},
    {'key': 's_pillar', 'center': (8.0, -16.0), 'u': (1.0, 0.0), 'f': (0.0, 1.0), 'wall': 'S',
     'desc': '남쪽 벽 앞 (8, -16) — 기둥 (10, -12)·좁은 통로 남쪽 출구 근처'},
    {'key': 'e_wall', 'center': (27.0, -11.0), 'u': (0.0, 1.0), 'f': (-1.0, 0.0), 'wall': 'E',
     'desc': '동쪽 벽 앞 (27, -11), 슬롯이 y 방향으로 늘어서고 서쪽을 본다'},
    {'key': 's_west', 'center': (-6.0, -17.0), 'u': (1.0, 0.0), 'f': (0.0, 1.0), 'wall': 'S',
     'desc': '남쪽 벽 앞 (-6, -17) — 좁은 통로 서쪽, 기둥 (-10, -12) 근처'},
    {'key': 'n_center', 'center': (0.0, 17.5), 'u': (1.0, 0.0), 'f': (0.0, -1.0), 'wall': 'N',
     'desc': '북쪽 벽 앞 (0, 17.5) — 메인 통로(지게차 차선 y=15) 북쪽, 남쪽을 본다'},
]
# V2 대기 배열: 슬롯 (축 u 오프셋 s, 안쪽 f 오프셋 t), amr_01 … amr_05 순
V2 = [
    {'key': 'line2', 'offsets': [(-4, 0), (-2, 0), (0, 0), (2, 0), (4, 0)],
     'desc': '일렬 2 m 간격 (현행, 이웃 틈 1.6 m)'},
    {'key': 'line3', 'offsets': [(-6, 0), (-3, 0), (0, 0), (3, 0), (6, 0)],
     'desc': '일렬 3 m 간격 (이웃 틈 2.6 m)'},
    {'key': 'stagger', 'offsets': [(-4, 0), (-2, -1.5), (0, 0), (2, -1.5), (4, 0)],
     'desc': '엇갈린 2열: 앞 열 3대 4 m 간격, 뒤 열 2대 (벽 쪽 1.5 m, 앞 열 사이)'},
]
V3 = [
    {'key': 'sw_current', 'desc': '현행 남서: 남쪽 벽 앞 x = -26/-22/-18 (y -18.4)'},
    {'key': 'beside_waiting',
     'desc': '대기 구역 옆: 같은 벽 앞, 슬롯 줄 끝에서 3 m 떨어져 4 m 간격 3대 (+u 쪽, 안 들어가면 -u 쪽)'},
    {'key': 'split_docks',
     'desc': '도크 쪽 분산: 북쪽 벽 앞 x = -22 (입고 도크 쪽) · 22 · 18 (출고 도크 쪽)'},
]
# V4 랙 배열: 열마다 랙 중심 x (주 랙 21 개 고정)
_X6 = [-18.0, -12.0, -6.0, 0.0, 6.0, 12.0, 18.0]
V4 = [
    {'key': 'uniform6', 'rows': {'A': _X6, 'B': _X6, 'C': _X6},
     'desc': '6 m 균일 (현행)'},
    {'key': 'b_shift3', 'rows': {'A': _X6, 'B': [x + 3.0 for x in _X6], 'C': _X6},
     'desc': 'B 열 동쪽으로 3 m 이동 (x -15 … 21)'},
    {'key': 'pitch_mix',
     'rows': {'A': [-18.0, -13.0, -6.0, 0.0, 7.0, 12.0, 18.0],     # 간격 5 7 6 7 5 6
              'B': [-18.0, -12.0, -7.0, 0.0, 5.0, 12.0, 18.0],     # 간격 6 5 7 5 7 6
              'C': [-18.0, -11.0, -5.0, 0.0, 6.0, 13.0, 18.0]},    # 간격 7 6 5 6 7 5
     'desc': '열마다 다른 간격 (5/6/7 m 섞음, 6 m 주기·점대칭 깨기)'},
    {'key': 'end_moved', 'rows': {'A': _X6 + [24.0], 'B': _X6, 'C': _X6[1:]},
     'desc': 'C 열 서쪽 끝 랙 (-18, -3) 을 A 열 동쪽 끝 (24, 9) 으로'},
]
V5 = [
    {'key': 'none', 'desc': '없음 (현행)'},
    {'key': 'structures',
     'desc': '대기 구역 옆·충전소 옆 고정 구조물 (팔레트 더미 1.2 x 1.0 x 1.5 m) 각 1 개'},
    {'key': 'pillars_moved',
     'desc': '기둥 2 개 비대칭 이동: 4 번 (10, -12) → (13, -12), 8 번 (25, -6) → (25, -9.5)'},
]
LEVELS = {'V1': V1, 'V2': V2, 'V3': V3, 'V4': V4, 'V5': V5}
ROW_Y = {'A': 9.0, 'B': 3.0, 'C': -3.0}
STRUCT = {'l': 1.2, 'd': 1.0, 'h': 1.5}
PILLAR_MOVES = {3: (13.0, -12.0), 7: (25.0, -9.5)}      # 0 부터 센 기둥 번호 → 새 중심
SPLIT_CHARGERS = [(-22.0, 'N'), (22.0, 'N'), (18.0, 'N')]

SPEC_RE = re.compile(r'^v1(\d)-v2(\d)-v3(\d)-v4(\d)-v5(\d)$')
BASELINE_ID = 'v10-v20-v30-v40-v50'


def spec_id(levels):
    """{'V1': i, …} → 'v1i-v2j-…'."""
    return '-'.join(f'v{k}{levels[f"V{k}"]}' for k in range(1, 6))


def parse_spec_id(text):
    """'v1i-…' → {'V1': i, …} (범위 밖이면 ValueError)."""
    m = SPEC_RE.match(text.strip())
    if not m:
        raise ValueError(f'spec_id 형식이 아니다: {text!r}')
    levels = {f'V{k + 1}': int(v) for k, v in enumerate(m.groups())}
    for k, v in levels.items():
        if not 0 <= v < len(LEVELS[k]):
            raise ValueError(f'{k} 수준 {v} 없음 (0..{len(LEVELS[k]) - 1})')
    return levels


def all_spec_ids():
    """전 조합 (사전순) — 5 × 3 × 3 × 4 × 3 = 540."""
    out = []
    for a in range(len(V1)):
        for b in range(len(V2)):
            for c in range(len(V3)):
                for d in range(len(V4)):
                    for e in range(len(V5)):
                        out.append(spec_id({'V1': a, 'V2': b, 'V3': c, 'V4': d, 'V5': e}))
    return out


# ---------------------------------------------------------------------------- 기하 도구
def _wall_point(wall, along):
    """벽 안쪽 면 위의 점 (along = 벽을 따라가는 좌표) 과 안쪽 법선."""
    if wall == 'S':
        return (along, -HALF_Y), (0.0, 1.0)
    if wall == 'N':
        return (along, HALF_Y), (0.0, -1.0)
    if wall == 'E':
        return (HALF_X, along), (-1.0, 0.0)
    return (-HALF_X, along), (1.0, 0.0)


def _sign_on_wall(wall, along):
    """벽 표지 [x, y, yaw] (판 법선 = 안쪽)."""
    if wall == 'S':
        return [along, -HALF_Y + PLATE_OFF, _S]
    if wall == 'N':
        return [along, HALF_Y - PLATE_OFF, _N]
    if wall == 'E':
        return [HALF_X - PLATE_OFF, along, _E]
    return [-HALF_X + PLATE_OFF, along, _W]


def _along(wall, x, y):
    return x if wall in ('S', 'N') else y


def aabb_gap(a, b):
    """두 AABB (x0, x1, y0, y1) 사이 최단 거리 (겹치면 0)."""
    dx = max(a[0] - b[1], b[0] - a[1], 0.0)
    dy = max(a[2] - b[3], b[2] - a[3], 0.0)
    return math.hypot(dx, dy)


def point_aabb(px, py, b):
    dx = max(b[0] - px, px - b[1], 0.0)
    dy = max(b[2] - py, py - b[3], 0.0)
    return math.hypot(dx, dy)


def seg_point(ax, ay, bx, by, px, py):
    """선분 ab 와 점 p 의 거리."""
    vx, vy = bx - ax, by - ay
    n2 = vx * vx + vy * vy
    t = 0.0 if n2 == 0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / n2))
    return math.hypot(ax + t * vx - px, ay + t * vy - py)


def seg_aabb(ax, ay, bx, by, b, step=0.1):
    """선분과 AABB 의 (근사) 최단 거리 — 선분 위 표본."""
    n = max(1, int(math.hypot(bx - ax, by - ay) / step))
    return min(point_aabb(ax + (bx - ax) * k / n, ay + (by - ay) * k / n, b) for k in range(n + 1))


def rack_aabb(r):
    if abs(math.sin(r.get('yaw', 0.0))) < 0.5:
        return (r['x'] - RACK_L / 2, r['x'] + RACK_L / 2, r['y'] - RACK_D / 2, r['y'] + RACK_D / 2)
    return (r['x'] - RACK_D / 2, r['x'] + RACK_D / 2, r['y'] - RACK_L / 2, r['y'] + RACK_L / 2)


def station_aabb(c):
    hx, hy = (0.25, 0.2) if abs(math.sin(c['yaw'])) < 0.5 else (0.2, 0.25)
    return (c['x'] - hx, c['x'] + hx, c['y'] - hy, c['y'] + hy)


def struct_aabb(s):
    hx, hy = (s['l'] / 2, s['d'] / 2) if abs(math.sin(s.get('yaw', 0.0))) < 0.5 \
        else (s['d'] / 2, s['l'] / 2)
    return (s['x'] - hx, s['x'] + hx, s['y'] - hy, s['y'] + hy)


def pillar_aabb(p, half=0.25):
    return (p[0] - half, p[0] + half, p[1] - half, p[1] + half)


def narrow_racks():
    """좁은 통로 4베이 (생성기 rack_instances 와 같은 식)."""
    gen = load_generator()
    off = gen.NARROW_CLEAR / 2 + gen.RACK_D / 2
    out = []
    for side, sx in (('W', -off), ('E', off)):
        for j, dy in enumerate((-1.0, 1.0), 1):
            out.append({'id': f'rack_narrow_{side}{j}', 'x': gen.NARROW_CX + sx,
                        'y': gen.NARROW_CY + dy, 'yaw': math.pi / 2, 'kind': 'narrow'})
    return out


def dock_boxes():
    """도크 화물 상자 (생성기 docks_and_zones: 판에서 1.2 m 안쪽, y ± 1) — 대략 0.6 m 상자로 본다."""
    gen = load_generator()
    out = []
    for name, (x, y, kind, _mid) in gen.DOCKS.items():
        sx = 1.0 if kind == 'inbound' else -1.0
        bx = x - sx * 1.2
        for dy in (1.0, -1.0):
            out.append((f'{name}_box{"+" if dy > 0 else "-"}',
                        (bx - 0.35, bx + 0.35, y + dy - 0.35, y + dy + 0.35)))
    return out


def charger_geometry(c):
    """스테이션 → (판 중심 자세 [x, y, yaw], staging [x, y, yaw], 도킹 완료 자세 [x, y, yaw])."""
    nx, ny = math.cos(c['yaw']), math.sin(c['yaw'])
    mx, my = c['x'] + MARKER_OFF * nx, c['y'] + MARKER_OFF * ny
    face = math.atan2(-ny, -nx)
    return ([mx, my, c['yaw']], [mx + STAGING_FROM_PLATE * nx, my + STAGING_FROM_PLATE * ny, face],
            [mx + 0.65 * nx, my + 0.65 * ny, face])


def row_gaps(xs):
    """열 x 정렬 목록 → [(틈 중심, 틈 왼쪽 면, 틈 오른쪽 면)] (이웃 랙 사이)."""
    return [((a + b) / 2, a + RACK_L / 2, b - RACK_L / 2) for a, b in zip(xs, xs[1:])]


def junction_groups(centers):
    """통로 하나의 교차 후보 x (정렬) → 겹치는(2 m 미만) 것끼리 묶은 [[x…]]."""
    groups = []
    for c in sorted(set(centers)):
        if groups and c - groups[-1][-1] < 2.0 - 1e-9:
            groups[-1].append(c)
        else:
            groups.append([c])
    return groups


def rows_from_racks(racks):
    rows = {}
    for r in racks:
        if r.get('kind', 'main') != 'main':
            continue
        rows.setdefault(r['row'], {'y': r['y'], 'xs': []})['xs'].append(r['x'])
    for v in rows.values():
        v['xs'].sort()
    return dict(sorted(rows.items(), key=lambda kv: -kv[1]['y']))


def aisles(rows):
    """
    통로 목록 [{id, y, y0, y1, north, south, centers}].

    열 사이 통로(ab, bc): y 범위 = 남쪽 열 북면 ~ 북쪽 열 남면, 교차 후보 = 두 열의 틈 중심 중 두 열이 모두 있는
    x 범위 안 (끝 넘어 트인 곳 제외). 남쪽 홀 통로(s): 가장 남쪽 열 남면 2.5 m 아래, 그 열의 틈.
    """
    names = list(rows)
    out = []
    for n, s in zip(names, names[1:]):
        rn, rs = rows[n], rows[s]
        lo, hi = max(rn['xs'][0], rs['xs'][0]), min(rn['xs'][-1], rs['xs'][-1])
        cs = [g[0] for g in row_gaps(rn['xs']) + row_gaps(rs['xs'])
              if lo - 1e-9 <= g[0] <= hi + 1e-9]
        out.append({'id': f'{n.lower()}{s.lower()}', 'y': (rn['y'] + rs['y']) / 2,
                    'y0': rs['y'] + RACK_D / 2, 'y1': rn['y'] - RACK_D / 2,
                    'north': n, 'south': s, 'centers': cs})
    last = names[-1]
    out.append({'id': 's', 'y': rows[last]['y'] - 3.0, 'y0': None,
                'y1': rows[last]['y'] - RACK_D / 2, 'north': last, 'south': None,
                'centers': [g[0] for g in row_gaps(rows[last]['xs'])]})
    return out


# ---------------------------------------------------------------------------- 동적 장애물 경로
def walk_graph(rows):
    """
    무작위 보행 격자 (간선 목록, 통로별 노드).

    통로마다 교차 묶음 중심에 노드, 통로를 따라 이웃 노드 간선, 열의 같은 틈 안에 있는 위·아래
    통로 노드끼리 간선. 스폰 셀 (0, 0) 1 m 안을 지나는 노드·간선은 뺀다. 현행 배치에서 default_walk_edges() 와
    같은 간선을 같은 순서로 낸다 (시험 test_levels.py).
    """
    al = aisles(rows)
    nodes = []
    for a in al:
        nodes.append([(sum(g) / len(g), a['y']) for g in junction_groups(a['centers'])])

    def near_spawn(p, q=None):
        if q is None:
            return math.hypot(*p) < 1.0
        return seg_point(p[0], p[1], q[0], q[1], 0.0, 0.0) < 1.0

    edges = []
    for ns in nodes:
        for p, q in zip(ns, ns[1:]):
            if not (near_spawn(p) or near_spawn(q) or near_spawn(p, q)):
                edges.append((p, q))
    vert = []
    for k in range(len(al) - 1):
        row = rows[al[k]['south']]
        for p in nodes[k]:
            for q in nodes[k + 1]:
                if near_spawn(p) or near_spawn(q):
                    continue
                if any(g0 + 0.5 <= min(p[0], q[0]) and max(p[0], q[0]) <= g1 - 0.5
                       for _c, g0, g1 in row_gaps(row['xs'])):
                    vert.append(((p[0], q[0], k), (p, q)))
    edges += [e for _key, e in sorted(vert, key=lambda kv: kv[0])]
    return edges, nodes


def _lane_clear(ax, ay, bx, by, obstacles, margin):
    return all(seg_aabb(ax, ay, bx, by, b) > margin for _n, b in obstacles)


def resolve_lanes(lay, obstacles, notes):
    """사람 actor · 차량 경로: 현행 값을 지킬 수 있으면 그대로, 막히면 정한 규칙으로 옮긴다 (notes 에 기록)."""
    gen = load_generator()
    lanes = copy.deepcopy(gen.default_layout()['lanes'])
    rows = rows_from_racks(lay['racks'])
    margin = 0.40                    # 생성기 check_layout 0.35 + 표본 여유
    # 동측 통로 보행자 (현행 x = 22, y -10..10)
    fast = lanes['worker_straight_fast']
    if not _lane_clear(22.0, -10.0, 22.0, 10.0, obstacles, margin):
        best = None
        for k in range(0, 33):
            x = 19.5 + 0.25 * k
            m = min(seg_aabb(x, -10.0, x, 10.0, b) for _n, b in obstacles)
            key = (round(m, 3), -abs(x - 22.0))
            if best is None or key > best[0]:
                best = (key, x)
        if best[0][0] <= margin:
            raise LayoutError('lane:worker_straight_fast 동측 통로에 자리 없음')
        fast['a'], fast['b'] = [best[1], -10.0], [best[1], 10.0]
        notes.append(f'worker_straight_fast x 22 → {best[1]:g} (랙 끝과 기둥 사이 최대 여유)')
    # 서측 원 보행자 (현행 중심 (-24, 0) r 3.5)
    cv = lanes['worker_curve']
    for k in range(0, 4):
        cx = -24.0 - 0.5 * k
        pts = [(cx + 3.5 * math.cos(2 * math.pi * i / 72), 3.5 * math.sin(2 * math.pi * i / 72))
               for i in range(72)]
        if all(point_aabb(px, py, b) > margin for px, py in pts for _n, b in obstacles):
            if k:
                cv['c'] = [cx, 0.0]
                notes.append(f'worker_curve 중심 x -24 → {cx:g}')
            break
    else:
        raise LayoutError('lane:worker_curve 서측 공터에 자리 없음')
    # 무작위 보행 격자
    edges, nodes = walk_graph(rows)
    rw = lanes['worker_random']
    default_edges = [((float(a[0]), float(a[1])), (float(b[0]), float(b[1])))
                     for a, b in gen.default_walk_edges()]
    if edges != default_edges:
        rw['edges'] = [[list(a), list(b)] for a, b in edges]
        snodes = nodes[-1]
        if not snodes:
            raise LayoutError('lane:worker_random 남쪽 통로 노드 없음')
        start = min(snodes, key=lambda p: (math.hypot(p[0] + 9.0, p[1] + 6.0), p[0]))
        rw['start'] = list(start)
        notes.append(f'worker_random 격자 재생성 (간선 {len(edges)}, 시작 {start})')
    # 셔틀 AMR (현행 x = 9, y -6..11): 막히면 아래 끝을 통로 중심(0 → 6)으로 올린다
    veh = {}
    for lo in (-6.0, 0.0, 6.0):
        box = (9.0 - 0.55, 9.0 + 0.55, lo - 0.35, 11.0 + 0.35)
        if all(aabb_gap(box, b) > 0.0 for _n, b in obstacles):
            if lo != -6.0:
                veh['shuttle_amr'] = {'lo': lo}
                notes.append(f'shuttle_amr 차선 y -6..11 → {lo:g}..11 (x = 9 에서 열이 막혀서)')
            break
    else:
        raise LayoutError('lane:shuttle_amr x=9 교차 통로가 막힘')
    return lanes, veh


# ---------------------------------------------------------------------------- 해석
def _slots(w, arr):
    (cx, cy), (ux, uy), (fx, fy) = w['center'], w['u'], w['f']
    yaw = math.atan2(fy, fx)
    out = []
    for robot, (s, t) in zip(ROBOTS, arr['offsets']):
        out.append({'robot': robot, 'x': cx + s * ux + t * fx, 'y': cy + s * uy + t * fy,
                    'yaw': yaw})
    return out


def _static_obstacles(lay):
    """(이름, AABB) — 랙(좁은 통로 포함)·기둥·스테이션·구조물·도크 화물."""
    obs = [(r['id'], rack_aabb(r)) for r in lay['racks']]
    obs += [(r['id'], rack_aabb(r)) for r in narrow_racks()]
    obs += [(f'pillar_{i}', pillar_aabb(p)) for i, p in enumerate(lay['pillars'], 1)]
    obs += [(f"charging_station_{c['id']}", station_aabb(c)) for c in lay['chargers']]
    obs += [(f"structure_{s['id']}", struct_aabb(s)) for s in lay['structures']]
    obs += dock_boxes()
    return obs


def _inside_room(b, margin):
    return (b[0] >= -HALF_X + margin and b[1] <= HALF_X - margin
            and b[2] >= -HALF_Y + margin and b[3] <= HALF_Y - margin)


def _place_chargers_beside(w, slots, obstacles):
    """대기 구역 옆 충전소 3대: 같은 벽, 슬롯 줄 끝 + 3 m, 4 m 간격 (+u 쪽 먼저)."""
    (cx, cy), (ux, uy) = w['center'], w['u']
    s_list = [(sl['x'] - cx) * ux + (sl['y'] - cy) * uy for sl in slots]
    along0 = _along(w['wall'], cx, cy)
    for sign in (1.0, -1.0):
        edge = max(s_list) if sign > 0 else min(s_list)
        out = []
        for k, ds in enumerate((3.0, 7.0, 11.0)):
            along = along0 + edge + sign * ds
            (wx, wy), (nx, ny) = _wall_point(w['wall'], along)
            out.append({'id': f'c{k + 1}', 'x': wx + STATION_WALL_OFF * nx,
                        'y': wy + STATION_WALL_OFF * ny, 'yaw': math.atan2(ny, nx),
                        'marker_id': 4 + k})
        if all(_inside_room(station_aabb(c), 0.5) for c in out) and all(
                aabb_gap(station_aabb(c), b) > 1.0 for c in out for _n, b in obstacles):
            return out
    raise LayoutError('chargers:대기 구역 옆 양쪽에 충전소 3대 자리가 없음')


def _charging_pad_sign(chargers, group):
    """충전 패드 [x, y, yaw] (무리 중심 앞 2.4 m) 와 벽 표지 (무리 중심 벽면)."""
    g = [c for c in chargers if c['id'] in group]
    mx = sum(c['x'] for c in g) / len(g)
    my = sum(c['y'] for c in g) / len(g)
    nx, ny = math.cos(g[0]['yaw']), math.sin(g[0]['yaw'])
    wall = {(0, 1): 'S', (0, -1): 'N', (-1, 0): 'E', (1, 0): 'W'}[(round(nx), round(ny))]
    pad = [mx + 2.4 * nx, my + 2.4 * ny, math.pi / 2 if wall in ('E', 'W') else 0.0]
    return pad, _sign_on_wall(wall, _along(wall, mx, my)), wall


def _place_structure(sid, anchor_pts, u, wall, obstacles, keep_clear):
    """
    구조물 하나: 줄(anchor_pts — 같은 벽 앞에 늘어선 자세들) 끝에서 u 방향으로 2.0 m, 벽에서 0.8 m.

    +u 끝 먼저, 안 되면 -u 끝. keep_clear 점(슬롯·staging)과 1.2 m, 장애물과 1.0 m 를 지켜야 한다.
    """
    (ux, uy) = u
    s_list = [p[0] * ux + p[1] * uy for p in anchor_pts]
    for sign in (1.0, -1.0):
        edge = max(s_list) if sign > 0 else min(s_list)
        along = edge + sign * 2.0
        (wx, wy), (nx, ny) = _wall_point(wall, along)
        st = {'id': sid, 'kind': 'pallet_stack', 'x': wx + 0.8 * nx, 'y': wy + 0.8 * ny,
              'yaw': 0.0 if wall in ('S', 'N') else math.pi / 2, **STRUCT}
        b = struct_aabb(st)
        if not _inside_room(b, 0.2):
            continue
        if any(aabb_gap(b, ob) < 1.0 for _n, ob in obstacles):
            continue
        if any(point_aabb(px, py, b) < 1.2 for px, py in keep_clear):
            continue
        return st
    raise LayoutError(f'structure:{sid} 둘 자리가 없음')


def resolve(levels):
    """수준 → 해석 dict (모듈 설명). 고정 조건을 못 지키면 LayoutError."""
    gen = load_generator()
    lay = gen.default_layout()
    notes = []
    w, arr = V1[levels['V1']], V2[levels['V2']]
    ch, rk, st = V3[levels['V3']], V4[levels['V4']], V5[levels['V5']]
    # V4 랙
    racks = []
    for row, xs in rk['rows'].items():
        for i, x in enumerate(sorted(xs), 1):
            racks.append({'id': f'rack_{row}{i}', 'row': row, 'x': float(x), 'y': ROW_Y[row],
                          'yaw': 0.0})
    assert len(racks) == 21, len(racks)
    lay['racks'] = racks
    # V5 기둥
    if st['key'] == 'pillars_moved':
        for i, p in PILLAR_MOVES.items():
            lay['pillars'][i] = list(p)
    # V1·V2 대기 구역
    slots = _slots(w, arr)
    (cx, cy), (ux, uy) = w['center'], w['u']
    lay['waiting_pad'] = [cx, cy, math.atan2(uy, ux)]
    lay['waiting_sign'] = _sign_on_wall(w['wall'], _along(w['wall'], cx, cy))
    # V3 충전소
    if ch['key'] == 'beside_waiting':
        base = [(n, b) for n, b in _static_obstacles(dict(lay, chargers=[]))]
        lay['chargers'] = _place_chargers_beside(w, slots, base)
        pad, sign, _wall = _charging_pad_sign(lay['chargers'], ('c1', 'c2', 'c3'))
        lay['charging_pad'], lay['charging_sign'] = pad, sign
    elif ch['key'] == 'split_docks':
        out = []
        for k, (along, wall) in enumerate(SPLIT_CHARGERS):
            (wx, wy), (nx, ny) = _wall_point(wall, along)
            out.append({'id': f'c{k + 1}', 'x': wx + STATION_WALL_OFF * nx,
                        'y': wy + STATION_WALL_OFF * ny, 'yaw': math.atan2(ny, nx),
                        'marker_id': 4 + k})
        lay['chargers'] = out
        pad, sign, _wall = _charging_pad_sign(out, ('c2', 'c3'))
        lay['charging_pad'], lay['charging_sign'] = pad, sign
    # V5 구조물 (대기 구역 옆, 충전소 c1 무리 옆)
    if st['key'] == 'structures':
        obstacles = _static_obstacles(lay)
        keep = [(s['x'], s['y']) for s in slots]
        stagings = [charger_geometry(c)[1] for c in lay['chargers']]
        keep += [(p[0], p[1]) for p in stagings]
        s1 = _place_structure('S1', [(s['x'], s['y']) for s in slots], w['u'], w['wall'],
                              obstacles, keep)
        lay['structures'] = [s1]
        obstacles.append(('structure_S1', struct_aabb(s1)))
        c0 = lay['chargers'][0]
        nx, ny = math.cos(c0['yaw']), math.sin(c0['yaw'])
        cwall = {(0, 1): 'S', (0, -1): 'N', (-1, 0): 'E', (1, 0): 'W'}[(round(nx), round(ny))]
        cu = (abs(ny), abs(nx))                    # 벽을 따라가는 축
        group = [c for c in lay['chargers']
                 if abs(math.cos(c['yaw']) - nx) < 1e-6 and abs(math.sin(c['yaw']) - ny) < 1e-6]
        s2 = _place_structure('S2', [(c['x'], c['y']) for c in group], cu, cwall, obstacles,
                              keep)
        lay['structures'].append(s2)
    # 동적 장애물 경로
    obstacles = _static_obstacles(lay)
    lanes, veh = resolve_lanes(lay, obstacles, notes)
    lay['lanes'], lay['vehicles'] = lanes, veh
    return {'levels': dict(levels), 'spec_id': spec_id(levels), 'gen_spec': lay, 'slots': slots,
            'waiting': {'key': w['key'], 'center': list(w['center']), 'u': list(w['u']),
                        'f': list(w['f']), 'wall': w['wall']},
            'chargers': lay['chargers'], 'notes': notes,
            'level_keys': {'V1': w['key'], 'V2': arr['key'], 'V3': ch['key'], 'V4': rk['key'],
                           'V5': st['key']}}
