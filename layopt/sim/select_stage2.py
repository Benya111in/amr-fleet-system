"""
E1 2 단계 배치 선정 (규칙은 일지에 먼저 적었다 — 2026-10-10 layopt-sim "2단계 선정 규칙").

    python3 -m layopt.sim.select_stage2 --stage1 <1단계 run_dir> --out <선정 json> \
        [--must /home/CAPYI_AI/runs/layopt/analysis/stage2_must_include.txt]

배치마다 1 단계(부하 1.0, 시드 0–9) 평균으로 두 목적을 만든다.
- T = 처리량 throughput_per_h (클수록 좋다)
- W = 대기 슬롯(wait:*) 체류 중 k ≥ 1 인 시간 합 [s] (작을수록 좋다 — 이웃이 있는 대기 = 측위 위험 대리)
선정:
 1. 비지배 정렬 층(1 층 = 파레토 전선)을 차례로 통째로 넣되 누적이 N_PARETO(45) 를 넘지 않을 때까지.
 2. 다음 층에서는 1 층까지의 정규화 거리 d = min_f max((T_f − T)/s_T, (W − W_f)/s_W) 가 작은 순으로 채운다
    (s_T, s_W = 배치 간 표준편차). 1–2 를 합쳐 "파레토 근처" N_PARETO 개.
 3. 기준 배치 v10-v20-v30-v40-v50 를 넣는다.
 4. must 파일(있으면 — 조정자가 E0 결과로 만든다)의 spec_id 를 모두 넣는다 (1 단계에 없으면 경고).
 5. 남은 배치에서 random.Random(20261010) 으로 정렬된 id 중 10 개를 넣는다.
시드 10 개 미만인 배치(1 단계 실패)는 후보에서 빼고 기록한다.
"""

from __future__ import annotations

import argparse
import collections
import json
import pathlib
import random
import statistics
import sys
from typing import Dict, List

BASELINE = 'v10-v20-v30-v40-v50'
N_PARETO = 45
N_RANDOM = 10
RANDOM_SEED = 20261010


def objectives(results: pathlib.Path, load: float = 1.0, min_seeds: int = 10):
    t = collections.defaultdict(list)
    w = collections.defaultdict(list)
    for line in results.read_text(encoding='utf-8').splitlines():
        if not line.strip():
            continue
        r = json.loads(line)
        if float(r['load']) != load:
            continue
        sid = r['spec_id']
        t[sid].append(float(r['throughput_per_h']))
        w[sid].append(sum(v for d in r['dwell'] if d['id'].startswith('wait:')
                          for k, v in d['k_hist_s'].items() if int(k) >= 1))
    out, short = {}, {}
    for sid in t:
        if len(t[sid]) < min_seeds:
            short[sid] = len(t[sid])
            continue
        out[sid] = (statistics.fmean(t[sid]), statistics.fmean(w[sid]))
    return out, short


def dominates(a, b) -> bool:
    """지배 판정: 배치 a 가 b 보다 T 크고 W 작음 (하나는 엄격)."""
    return a[0] >= b[0] and a[1] <= b[1] and (a[0] > b[0] or a[1] < b[1])


def layers(obj: Dict[str, tuple]) -> List[List[str]]:
    rest = set(obj)
    out = []
    while rest:
        front = sorted(s for s in rest
                       if not any(dominates(obj[o], obj[s]) for o in rest if o != s))
        out.append(front)
        rest -= set(front)
    return out


def select(obj: Dict[str, tuple], must: List[str]):
    ls = layers(obj)
    s_t = statistics.pstdev([v[0] for v in obj.values()]) or 1.0
    s_w = statistics.pstdev([v[1] for v in obj.values()]) or 1.0
    front = ls[0] if ls else []
    pareto: List[str] = []
    used_layers = 0
    for layer in ls:
        if len(pareto) + len(layer) <= N_PARETO:
            pareto += layer
            used_layers += 1
            continue

        def dist(s):
            t, w = obj[s]
            return min(max((obj[f][0] - t) / s_t, (w - obj[f][1]) / s_w) for f in front)
        need = N_PARETO - len(pareto)
        pareto += sorted(layer, key=lambda s: (dist(s), s))[:need]
        break
    chosen = list(pareto)
    reasons = {s: 'pareto' for s in pareto}
    if BASELINE not in reasons:
        chosen.append(BASELINE)
        reasons[BASELINE] = 'baseline'
    missing = []
    for s in must:
        if s not in obj:
            missing.append(s)
        if s not in reasons:
            chosen.append(s)
            reasons[s] = 'must'
    rest = sorted(s for s in obj if s not in reasons)
    rng = random.Random(RANDOM_SEED)
    for s in rng.sample(rest, min(N_RANDOM, len(rest))):
        chosen.append(s)
        reasons[s] = 'random'
    return chosen, reasons, {'layers_used': used_layers, 'n_layers': len(ls),
                             'front': front, 's_T': s_t, 's_W': s_w, 'must_missing': missing}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='E1 2 단계 배치 선정')
    ap.add_argument('--stage1', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--must',
                    default='/home/CAPYI_AI/runs/layopt/analysis/stage2_must_include.txt')
    a = ap.parse_args(argv)
    obj, short = objectives(pathlib.Path(a.stage1) / 'results.jsonl')
    mp = pathlib.Path(a.must)
    must = [ln.strip() for ln in mp.read_text(encoding='utf-8').splitlines()
            if ln.strip() and not ln.startswith('#')] if mp.is_file() else []
    chosen, reasons, info = select(obj, must)
    doc = {'rule': __doc__, 'stage1': str(a.stage1), 'n_candidates': len(obj),
           'short_seeds': short, 'must_file': str(mp) if mp.is_file() else None, 'must': must,
           'n_chosen': len(chosen), 'chosen': chosen, 'reasons': reasons,
           'objectives': {s: {'T': round(obj[s][0], 3), 'W': round(obj[s][1], 1)}
                          for s in chosen if s in obj}, 'info': info}
    out = pathlib.Path(a.out)
    if out.exists():
        print(f'이미 있다 (덮어쓰지 않는다): {out}', file=sys.stderr)
        return 3
    out.write_text(json.dumps(doc, ensure_ascii=False, indent=1), encoding='utf-8')
    cnt = collections.Counter(reasons.values())
    print(f'선정 {len(chosen)}: {dict(cnt)}, 층 {info["layers_used"]}/{info["n_layers"]}, '
          f'전선 {len(info["front"])}, must 누락 {info["must_missing"]}, 시드 부족 {len(short)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
