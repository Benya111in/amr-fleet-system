"""
지도 변형 묶음 (suite JSON → bundle npz) — 평가 전에 한 번 만들어 sha256 으로 고정한다. Isaac 비의존.

suite (rl/isaac/eval_sets/b1_map_suite_v1.json, 형식 mapgen.SUITE_FORMAT):
  sets            변형 평가에 쓰는 집합 이름 (평가 집합 파일 안, 예 main_3_15·stress_3_15)
  maps_per_condition  조건마다 지도 수 K
  assign_seed     목표 → 지도 배정 난수 (모든 조건이 같은 배정을 쓴다: 지도 k 에는 늘 같은 목표들)
  tile_spacing_m  타일 간격 (x, y)
  conditions      [{name, kind, seed, …kind 별 매개변수}] (mapgen 머리말)
bundle (형식 mapgen.BUNDLE_FORMAT) 의 배열:
  타일  tile_offset [T, 2] · tile_cond [T] (−1 = 명목) · tile_map [T] · tile_obb_offs [T+1] ·
        tile_obb [ΣM, 5] (타일 국소) ·
        tile_zr [ΣM, 2] · tile_gid [ΣM] — 타일 0 = 명목 지도(원래 자리), 마지막 타일 = 명목 지도의 먼 사본(부동소수 탐침)
  행    row_cond · row_set · row_j (집합 안 순번) · row_pid (풀 원래 번호) · row_tile ·
        row_code (mapgen.CODES) · row_len · row_len_nom · row_clear (변형 끝 직진 여유) ·
        row_main_variant (≥ 0.40) · row_changed (경로가 명목과 다름) ·
        row_near_added (명목 경로와 새 상자 사이 최소 거리, 없으면 inf) · row_pts_offs [R+1] ·
        row_pts [Σ, 2] (타일 국소, 유효 행만)
"""

import hashlib
import json
import math
import os

import numpy as np

from . import eval_sets as ES
from . import mapgen as MG


def load_suite(path):
    """지도 변형 suite JSON 읽기 → dict (형식·조건 확인)."""
    with open(path, encoding='utf-8') as f:
        doc = json.load(f)
    validate_suite(doc)
    return doc


def validate_suite(doc):
    """지도 변형 suite 의 형식·조건 이름 유일·종류·필수 매개변수 확인 (틀리면 ValueError)."""
    if doc.get('format') != MG.SUITE_FORMAT:
        raise ValueError(f'지도 변형 suite 형식이 아니다: {doc.get("format")!r}')
    need = {'jitter': ('sigma_xy', 'sigma_yaw_deg'), 'scale': ('lo', 'hi'), 'remove': ('p',),
            'add': ('n', 'half_lo', 'half_hi', 'arc_lo', 'arc_hi', 'lateral_sigma',
                    'keepout_m', 'height')}
    names = [c['name'] for c in doc['conditions']]
    if len(set(names)) != len(names):
        raise ValueError(f'조건 이름 중복 {names}')
    for c in doc['conditions']:
        if c['kind'] not in need:
            raise ValueError(f'모르는 변형 {c["kind"]!r}')
        miss = [k for k in need[c['kind']] + ('seed',) if k not in c]
        if miss:
            raise ValueError(f'조건 {c["name"]} 에 {miss} 없음')
    if int(doc['maps_per_condition']) < 1:
        raise ValueError('maps_per_condition ≥ 1')
    return doc


def suite_sha(doc):
    """지도 변형 suite 내용 해시 (키 정렬 JSON)."""
    return hashlib.sha256(json.dumps(doc, sort_keys=True, ensure_ascii=False)
                          .encode()).hexdigest()[:16]


def goal_assignment(goal_cells, K, seed):
    """서로 다른 목표 셀 → 지도 번호 (정렬 → 무작위 순열 → 차례로 K 묶음)."""
    goals = np.unique(np.asarray(goal_cells))
    perm = np.random.default_rng(seed).permutation(goals.shape[0])
    owner = np.empty(goals.shape[0], dtype=np.int64)
    owner[perm] = np.arange(goals.shape[0]) % K
    return dict(zip(goals.tolist(), owner.tolist()))


def _pool_path(pool, pid):
    o = pool['offs']
    return pool['pts'][o[pid]:o[pid + 1]]


_CTX = {}     # 작업 공통 문맥 (fork 로 물려받는다 — 큰 풀 배열을 작업마다 직렬화하지 않게)


def _task(args):
    """(조건, 지도) 하나 — 변형 지도를 만들고 배정된 행을 다시 계획한다."""
    ci, k, rows = args
    cond, nominal, gid, anchor, pool = (_CTX[x] for x in ('conds', 'nominal', 'gid', 'anchor',
                                                          'pool'))
    cond = cond[ci]
    host = [_pool_path(pool, r['pid']) for r in rows]
    keep_pts = np.array([p[0] for p in host] + [p[-1] for p in host]).reshape(-1, 2)
    var = MG.make_variant(nominal, gid, anchor, cond, k, host_paths=host, keep_pts=keep_pts)
    planner = MG.Planner(var['obb'])
    added = var['obb'][var['gid'] == MG.ADDED]
    order = sorted(range(len(rows)), key=lambda i: (rows[i]['goal'], i))   # 목표마다 dijkstra 한 번
    res = [None] * len(rows)
    for i in order:
        r = rows[i]
        code, pts, L = planner.plan(r['goal'], r['start'])
        nom = host[i]
        rec = {'code': code, 'len': L, 'len_nom': float(np.sum(np.hypot(
            np.diff(nom[:, 0]), np.diff(nom[:, 1])))), 'pts': pts,
            'near_added': float(MG.rect_point_distance(added, nom).min()) if added.shape[0]
            else math.inf}
        if code == MG.VALID:
            rec['clear'] = MG.end_clearance(pts, var['obb'])
            rec['changed'] = not (pts.shape == nom.shape and np.array_equal(pts, nom))
        else:
            rec['clear'] = math.nan
            rec['changed'] = True
        res[i] = rec
    return {'ci': ci, 'k': k, 'variant': var, 'rows': res}


def build_bundle(suite, eval_doc, pool, nominal=None, workers=1, log=print):
    """
    suite·평가 집합·held-out 풀 → (bundle 배열 dict, 요약 dict).

    nominal = mapgen.nominal_layout() (None 이면 warehouse.sdf 에서).
    workers > 1 이면 (조건, 지도) 작업을 프로세스로 나눈다.
    결과는 workers 와 무관하다 (작업마다 독립 난수).
    """
    validate_suite(suite)
    ES.check_pool(eval_doc, pool['pts'], pool['offs'], pool['length'])
    nominal = MG.nominal_layout() if nominal is None else nominal
    gid, groups, anchor = MG.object_groups(nominal['names'], nominal['obb'])
    K = int(suite['maps_per_condition'])
    conds = suite['conditions']
    sets = list(suite['sets'])
    meta = pool['meta']
    srows = []          # (set_index, j, pid, goal, start)
    for si, s in enumerate(sets):
        for j, pid in enumerate(eval_doc['sets'][s]):
            g, st = (int(x) for x in meta[int(pid)])
            srows.append((si, j, int(pid), g, st))
    owner = goal_assignment([r[3] for r in srows], K, int(suite['assign_seed']))
    tasks = []
    for ci in range(len(conds)):
        for k in range(K):
            rows = [{'si': si, 'j': j, 'pid': pid, 'goal': g, 'start': st}
                    for si, j, pid, g, st in srows if owner[g] == k]
            tasks.append((ci, k, rows))
    _CTX.update(conds=conds, nominal=nominal, gid=gid, anchor=anchor, pool=pool)
    if workers > 1:
        import multiprocessing as mp
        with mp.get_context('fork').Pool(workers) as p:
            results = p.map(_task, tasks, chunksize=1)
    else:
        results = [_task(t) for t in tasks]
    n_var_tiles = len(conds) * K
    T = 1 + n_var_tiles + 1
    offsets = MG.tile_offsets(T, suite['tile_spacing_m'])
    far = T - 1
    if MG.far_cell_index(offsets) != far:
        # 먼 사본은 가장 먼 칸에 둔다 — 마지막 칸과 바꾼다
        fi = MG.far_cell_index(offsets)
        offsets[[fi, far]] = offsets[[far, fi]]
    tile_cond = np.full(T, -1, dtype=np.int64)
    tile_map = np.full(T, -1, dtype=np.int64)
    tile_obbs = [nominal['obb']] + [None] * n_var_tiles + [nominal['obb']]
    tile_zrs = [nominal['zr']] + [None] * n_var_tiles + [nominal['zr']]
    tile_gids = [gid] + [None] * n_var_tiles + [gid]
    tile_info = [{'kind': 'nominal'}] + [None] * n_var_tiles + [{'kind': 'nominal_far_copy'}]
    by_task = {}
    for r in results:
        t = 1 + r['ci'] * K + r['k']
        tile_cond[t], tile_map[t] = r['ci'], r['k']
        v = r['variant']
        tile_obbs[t], tile_zrs[t], tile_gids[t], tile_info[t] = v['obb'], v['zr'], v['gid'], \
            v['info']
        by_task[(r['ci'], r['k'])] = r
    # 행: 조건 → 집합 → 집합 안 순번
    cols = {k: [] for k in ('cond', 'set', 'j', 'pid', 'tile', 'code', 'len', 'len_nom', 'clear',
                            'main_variant', 'changed', 'near_added')}
    pts_list = []
    for ci in range(len(conds)):
        recs = {}
        for k in range(K):
            r = by_task[(ci, k)]
            rows = [x for x in srows if owner[x[3]] == k]
            for (si, j, pid, g, st), rec in zip(rows, r['rows']):
                recs[(si, j)] = (pid, 1 + ci * K + k, rec)
        for si, s in enumerate(sets):
            for j in range(len(eval_doc['sets'][s])):
                pid, t, rec = recs[(si, j)]
                cols['cond'].append(ci)
                cols['set'].append(si)
                cols['j'].append(j)
                cols['pid'].append(pid)
                cols['tile'].append(t)
                cols['code'].append(rec['code'])
                cols['len'].append(rec['len'])
                cols['len_nom'].append(rec['len_nom'])
                cols['clear'].append(rec['clear'])
                cols['main_variant'].append(rec['code'] == MG.VALID and
                                            rec['clear'] >= ES.THRESHOLD)
                cols['changed'].append(rec['changed'])
                cols['near_added'].append(rec['near_added'])
                pts_list.append(rec['pts'] if rec['pts'] is not None else np.zeros((0, 2)))
    offs = np.concatenate([[0], np.cumsum([p.shape[0] for p in pts_list])]).astype(np.int64)
    toffs = np.concatenate([[0], np.cumsum([o.shape[0] for o in tile_obbs])]).astype(np.int64)
    arr = {
        'format': np.array(MG.BUNDLE_FORMAT),
        'suite_json': np.array(json.dumps(suite, ensure_ascii=False, sort_keys=True)),
        'suite_sha': np.array(suite_sha(suite)),
        'pool_digest': np.array(eval_doc['pool_digest']),
        'cond_names': np.array([c['name'] for c in conds]),
        'set_names': np.array(sets),
        'tile_offset': offsets, 'tile_cond': tile_cond, 'tile_map': tile_map,
        'tile_obb_offs': toffs, 'tile_obb': np.concatenate(tile_obbs),
        'tile_zr': np.concatenate(tile_zrs), 'tile_gid': np.concatenate(tile_gids),
        'tile_info_json': np.array(json.dumps(tile_info, ensure_ascii=False)),
        'far_tile': np.array(far),
        'row_cond': np.array(cols['cond'], np.int64), 'row_set': np.array(cols['set'], np.int64),
        'row_j': np.array(cols['j'], np.int64), 'row_pid': np.array(cols['pid'], np.int64),
        'row_tile': np.array(cols['tile'], np.int64), 'row_code': np.array(cols['code'], np.int64),
        'row_len': np.array(cols['len']), 'row_len_nom': np.array(cols['len_nom']),
        'row_clear': np.array(cols['clear']),
        'row_main_variant': np.array(cols['main_variant'], bool),
        'row_changed': np.array(cols['changed'], bool),
        'row_near_added': np.array(cols['near_added']),
        'row_pts_offs': offs, 'row_pts': np.concatenate(pts_list).reshape(-1, 2),
    }
    return arr, summarize(arr, tile_info, groups)


def summarize(arr, tile_info=None, groups=None):
    """묶음 배열 → 조건·집합별 유효·제외·재분류 수 (사전 등록에 적는 설계 값 — 정책 결과 없음)."""
    tile_info = json.loads(str(arr['tile_info_json'])) if tile_info is None else tile_info
    out = {'conditions': {}, 'tiles': int(arr['tile_offset'].shape[0]),
           'far_tile': int(arr['far_tile']),
           'far_offset': [float(x) for x in arr['tile_offset'][int(arr['far_tile'])]],
           'max_abs_offset': float(np.abs(arr['tile_offset']).max())}
    if groups is not None:
        out['objects'] = len(groups)
    for ci, cn in enumerate(arr['cond_names'].tolist()):
        blk = {}
        for si, sn in enumerate(arr['set_names'].tolist()):
            m = (arr['row_cond'] == ci) & (arr['row_set'] == si)
            code = arr['row_code'][m]
            valid = code == MG.VALID
            mv = arr['row_main_variant'][m]
            blk[sn] = {
                'rows': int(m.sum()), 'valid': int(valid.sum()),
                'excluded': {MG.CODES[c]: int((code == c).sum()) for c in range(1, len(MG.CODES))},
                'valid_variant_main': int(mv.sum()),
                'valid_variant_narrow': int((valid & ~mv).sum()),
                'changed_share_of_valid': float(arr['row_changed'][m][valid].mean())
                if valid.any() else None,
                'len_ratio_median': float(np.median(arr['row_len'][m][valid] /
                                                    arr['row_len_nom'][m][valid]))
                if valid.any() else None,
                'near_added_lt_0p5_share': float((arr['row_near_added'][m] < 0.5).mean()),
                'near_added_lt_1p0_share': float((arr['row_near_added'][m] < 1.0).mean()),
            }
        tiles = np.flatnonzero(arr['tile_cond'] == ci)
        blk['maps'] = [tile_info[int(t)] for t in tiles]
        out['conditions'][cn] = blk
    return out


def save_bundle(path, arr):
    """묶음 npz 저장 (압축) → sha256."""
    tmp = path + f'.{os.getpid()}.tmp.npz'
    np.savez_compressed(tmp, **arr)
    os.replace(tmp, path)
    return file_sha256(path)


def file_sha256(path):
    """파일 sha256 (16 진 전체)."""
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest()


def load_bundle(path, expect_sha=None):
    """묶음 npz 읽기 → dict (형식·선택적 sha256 확인)."""
    if expect_sha:
        got = file_sha256(path)
        if not got.startswith(expect_sha):
            raise ValueError(f'bundle sha256 {got[:16]} ≠ 기대 {expect_sha}')
    d = dict(np.load(path, allow_pickle=False))
    if str(d['format']) != MG.BUNDLE_FORMAT:
        raise ValueError(f'bundle 형식이 아니다: {d["format"]}')
    return d


def tile_rects(arr, t):
    """타일 t 의 (사각형 [M, 5] 국소, 연직 범위 [M, 2], 물체 번호 [M])."""
    o = arr['tile_obb_offs']
    return arr['tile_obb'][o[t]:o[t + 1]], arr['tile_zr'][o[t]:o[t + 1]], \
        arr['tile_gid'][o[t]:o[t + 1]]


def row_path(arr, r):
    """행 r 의 재계획 경로 (타일 국소, 무효 행이면 빈 배열)."""
    o = arr['row_pts_offs']
    return arr['row_pts'][o[r]:o[r + 1]]
