#!/usr/bin/env python3
"""
보너스 2 결함 bag 재녹화 (b2-rerec) — 재생 작업 목록과 분석 보기(view) 만들기 (호스트, 표준 라이브러리만).

    rerec_plan.py plan --root R [--force]
        R/state/<id>.ok 의 통과 태그마다 E2 (행 6 × 재생 3) · E3(b) (위치추정 행 4 × 재생 3) ·
        E4 (S1 둘만, 절제 9 × 재생 2) 작업을 eval_driver.job() 으로 만들어 R/plans/{e2,e3,e4}.json 에 쓴다
        (eval_driver.py run 이 그대로 돈다)
    rerec_plan.py view --root R --out R/view
        분석 보기: <out>/{e2,e3,e4,rec,cache}/<슬롯> 심볼릭 링크 — 결함 없는 기존 5 bag 은 b2-eval·b2-rec 원자료, 결함
        bag 슬롯 셋은 재녹화 bag 과 그 재생 (SLOTS). 사전 등록 분석(peval.py·presults.py — b2-eval 사전 등록 판 그대로)은
        bag 이름을 디렉터리 이름으로 읽으므로, 이 보기 위에서 고치지 않고 다시 돈다

조건은 원래 재생과 같게 (사전 등록 notebook/entries/*_b2-rerec_*.md):
- 같은 재생 사본 — E2 `ws_e2` (ebf0cea), E3(b)·E4 `ws_e3b` (a0850e1), 읽기 전용 (b2-eval 원자료 디렉터리)
- 같은 지도 — E3(b) 는 b2-eval `maps/` (map_long 은 다시 녹화하지 않는다)
- 같은 배속 규칙 — 유효 sim 속도 0.25 / 녹화 RTF (RTF = /clock sim 구간 / 기록 벽시계 구간, 소수 셋째 자리)
- 같은 DOMAIN·우선순위 (E2 45 · E3(b) 46 · E4 48; E2 주 20+k, 참고 30+k, 오라클 55+k, E3(b) 45+k, E4 60+k)
"""

import argparse
import importlib.util
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OLD_EVAL = '/home/CAPYI_AI/runs/b24/b2-eval'
OLD_REC = '/home/CAPYI_AI/runs/b24/b2-rec/rec'
WS_E2 = f'{OLD_EVAL}/ws_e2'
WS_E3 = f'{OLD_EVAL}/ws_e3b'
MAPS = f'{OLD_EVAL}/maps'
#: 재녹화 계획 id → 분석 슬롯 (기존 결함 bag 이름). s1_seed2 재녹화는 셋째 S1 슬롯(s1_seed2x_a1)을 쓴다 — 같은 경로(씨드 2)
SLOTS = {'rr_s1_seed3': 's1_seed3_a2', 'rr_s1_seed2': 's1_seed2x_a1', 'rr_s6_wall': 's6_wall_a1'}
#: 결함 없는 기존 bag (그대로 쓴다)
KEEP = ('s3_aisle_a1', 's1_seed1_a1', 's6_open_a3', 's6_queue_a1', 'map_long_a1')
#: E4 절제를 다시 도는 슬롯 (E4 bag 4 개 중 결함 bag 둘)
ABL_IDS = ('rr_s1_seed3', 'rr_s1_seed2')
E_EFF = 0.25


def _drv():
    spec = importlib.util.spec_from_file_location('eval_driver',
                                                  os.path.join(HERE, 'eval_driver.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _rep():
    spec = importlib.util.spec_from_file_location('rec_report',
                                                  os.path.join(HERE, 'rec_report.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def passed_tags(root):
    """R/state/<id>.ok → {id: 태그} (SLOTS 의 id 만)."""
    out = {}
    for i in SLOTS:
        p = os.path.join(root, 'state', f'{i}.ok')
        if os.path.isfile(p):
            with open(p, encoding='utf-8') as f:
                out[i] = f.read().strip()
    return out


def bag_rtf(rec_dir):
    """녹화 시도 디렉터리의 bag_meta.json → RTF (소수 셋째 자리, recordings.md 표와 같은 반올림)."""
    with open(os.path.join(rec_dir, 'bag_meta.json'), encoding='utf-8') as f:
        meta = json.load(f)
    r = _rep().rtf(meta)
    if r is None:
        raise SystemExit(f'RTF 를 못 구했다: {rec_dir}')
    return round(r, 3)


def rate_of(rtf, e_eff=E_EFF):
    """재생 배속 = E / RTF (소수 셋째 자리 — eval_driver.rate_for 와 같은 반올림)."""
    return round(e_eff / rtf, 3)


def build_plans(root, tags, rtfs):
    """{단계: 작업 목록} — eval_driver 의 job()·행 목록·우선순위 그대로, bag 디렉터리·사본·배속만 이 재녹화의 것."""
    drv = _drv()
    rec = os.path.join(root, 'rec')
    go = os.path.join(root, 'go')
    plans = {'e2': [], 'e3': [], 'e4': []}
    rows = drv.E2_ROWS_MAIN + drv.E2_ROWS_REF + drv.E2_ROWS_AUX
    for k in range(3):
        for row in rows:
            for i, tag in tags.items():
                pri = 20 + k if row in drv.E2_ROWS_MAIN else (
                    30 + k if row in drv.E2_ROWS_REF else 55 + k)
                j = drv.job('e2', tag, row, k, pri, domain=45, rate=rate_of(rtfs[i]))
                j['wt'] = WS_E2
                j['bagdir'] = os.path.join(rec, tag, j['kind'])
                plans['e2'].append(j)
    mnt = [f'{MAPS}:/data/map:ro']
    maps = {
        'amcl_gt': ({'LOC_MAP': '/data/map/perfect_r0.050.yaml'},
                    [f'{MAPS}/perfect_r0.050.yaml']),
        'amcl_slam': ({'LOC_MAP': '/data/map/sel_slam_toolbox/map_world.yaml'},
                      [f'{MAPS}/sel_slam_toolbox/map_world.yaml']),
        'rtabmap3d_loc_gt': ({'LOC_DB': '/data/map/gtdb/rtabmap.db',
                              'MAP_SPAWN': drv.MAP_LONG_SPAWN}, [f'{MAPS}/gtdb/rtabmap.db']),
        'rtabmap3d_loc_slam': ({'LOC_DB': '/data/map/sel_rtabmap3d/rtabmap.db',
                                'MAP_SPAWN': drv.MAP_LONG_SPAWN},
                               [f'{MAPS}/sel_rtabmap3d/rtabmap.db']),
    }
    for k in range(3):
        for row, (env, req) in maps.items():
            for i, tag in tags.items():
                j = drv.job('e3', tag, row, k, 45 + k, 46, image=drv.SLAM_IMG, kind='bag', env=env,
                            requires=req + [f'{go}/e3'], mounts=mnt, rate=rate_of(rtfs[i]))
                j['wt'] = WS_E3
                j['bagdir'] = os.path.join(rec, tag, 'bag')
                plans['e3'].append(j)
    for k in range(2):
        for row in drv.ABL_ROWS:
            for i, tag in tags.items():
                if i not in ABL_IDS:
                    continue
                j = drv.job('e4', tag, row, k, 60 + k, 48, image=drv.SLAM_IMG, kind='bag',
                            requires=[f'{go}/e4'], rate=rate_of(rtfs[i]))
                j['wt'] = WS_E3
                j['bagdir'] = os.path.join(rec, tag, 'bag')
                plans['e4'].append(j)
    return plans


def cmd_plan(ns):
    """통과 태그 → R/plans/{e2,e3,e4}.json."""
    tags = passed_tags(ns.root)
    if not tags:
        raise SystemExit('통과한 재녹화 bag 이 없다 (state/*.ok)')
    rtfs = {i: bag_rtf(os.path.join(ns.root, 'rec', t)) for i, t in tags.items()}
    plans = build_plans(ns.root, tags, rtfs)
    pdir = os.path.join(ns.root, 'plans')
    os.makedirs(pdir, exist_ok=True)
    for st, jobs in plans.items():
        path = os.path.join(pdir, f'{st}.json')
        if os.path.exists(path) and not ns.force:
            raise SystemExit(f'이미 있다: {path} (--force)')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(jobs, f, ensure_ascii=False, indent=1)
        print(f'{st}: {len(jobs)} 작업 → {path}')
    print(json.dumps({'tags': tags, 'rtf': rtfs, 'rate': {i: rate_of(r) for i, r in rtfs.items()}},
                     ensure_ascii=False))
    return 0


def view_links(root, tags, old_eval=OLD_EVAL, old_rec=OLD_REC):
    """{보기 안 상대 경로: 대상 절대 경로} — 단계 디렉터리·녹화·캐시마다 슬롯 이름으로."""
    links = {}
    for b in KEEP:
        for st in ('e2', 'e3', 'e4'):
            src = os.path.join(old_eval, st, b)
            if os.path.isdir(src):
                links[f'{st}/{b}'] = src
        links[f'rec/{b}'] = os.path.join(old_rec, b)
        links[f'cache/{b}'] = os.path.join(old_eval, 'cache', b)
    for i, tag in tags.items():
        slot = SLOTS[i]
        for st in ('e2', 'e3') + (('e4',) if i in ABL_IDS else ()):
            links[f'{st}/{slot}'] = os.path.join(root, st, tag)
        links[f'rec/{slot}'] = os.path.join(root, 'rec', tag)
        links[f'cache/{slot}'] = os.path.join(root, 'cache', slot)
    return links


def cmd_view(ns):
    """분석 보기 디렉터리 (심볼릭 링크) — 이미 있으면 같은 대상인지 확인만."""
    tags = passed_tags(ns.root)
    links = view_links(ns.root, tags)
    for rel, dst in sorted(links.items()):
        p = os.path.join(ns.out, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        if os.path.islink(p):
            if os.readlink(p) != dst:
                raise SystemExit(f'보기 링크가 다르다: {p} → {os.readlink(p)} (원하는 것 {dst})')
            continue
        os.symlink(dst, p)
    with open(os.path.join(ns.out, 'VIEW.json'), 'w', encoding='utf-8') as f:
        json.dump({'slots': {SLOTS[i]: t for i, t in tags.items()}, 'keep': list(KEEP),
                   'links': links}, f, ensure_ascii=False, indent=1)
    print(json.dumps({'slots': {SLOTS[i]: t for i, t in tags.items()}, 'n_links': len(links)},
                     ensure_ascii=False))
    return 0


def main(argv=None):
    """명령줄."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('plan')
    s.add_argument('--root', required=True)
    s.add_argument('--force', action='store_true')
    s = sub.add_parser('view')
    s.add_argument('--root', required=True)
    s.add_argument('--out', required=True)
    ns = ap.parse_args(argv)
    return {'plan': cmd_plan, 'view': cmd_view}[ns.cmd](ns)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
