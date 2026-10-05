"""
협상 파라미터 스윕 [계산] — K·H·손실률·작업 수 → 중복·미할당·고착·라운드·비용 비.

사용
    python3 -m amr_fleet.negotiation.sweep --out-dir DIR [--cases 1000] [--workers 8]
        [--only commit,dropout,...] [--map maps/warehouse.yaml]

실험 (모두 로봇 5대, 완전연결, 링크별 파생 난수, 지연 U(0, 100) ms, 라운드 시한 150 ms)
- commit  : 이탈 없음. K × 전원 수신 × H × 생존 판정 × 손실 × 작업 수. 잠금은 낙관(협상만의 중복을 잰다)
- rule    : 병합 규칙 'max' 대 'cbba' (H 오탐 무효화가 있을 때 차이)
- dropout : 작업을 쥔 로봇이 3 라운드째 이탈 → 고착·미할당·재할당 라운드 (H × 생존 판정 × 손실)
- commitdie: 첫 확정 로봇이 잠금 방송 전에 이탈 → 낙관 모드 중복 대 배타 잠금
- health  : 보유 로봇의 위치추정 건강이 2 라운드째 나빠짐 → 'max' 대 'cbba' 재할당
- event   : 이벤트 구동 방송(부정 대조군) 대 주기 재방송
- burst   : Gilbert 버스트 손실(rho 0.8) 대 Bernoulli
- cost    : 유클리드 대 격자 A* 비용에서 CBAA·중앙 탐욕·Hungarian 비용 비 (스폰·흩어진 배치)

사례는 연구 계산과 같은 시드(사례 55·nt + k, 링크 시드 13·k + 1)를 써서 셀 사이가 쌍비교가 된다.
결과: DIR/<실험>.csv (셀마다 한 줄), 표준 출력에 요약.
"""

from __future__ import annotations

import argparse
import concurrent.futures as cf
import csv
import dataclasses
import math
import os
import random
import statistics
import sys
import time
from typing import Dict, Iterable, List, Optional, Sequence

from amr_fleet.negotiation import scenario
from amr_fleet.negotiation.agent import AgentConfig
from amr_fleet.negotiation.channel import PerLinkChannel
from amr_fleet.negotiation.cost import EuclideanCost, GridAstarCost
from amr_fleet.negotiation.sim import assignment_indices, HOLDER, run_case, SimConfig
from amr_fleet.negotiation.task_lock import ExclusiveTaskLock, OptimisticTaskLock

LOSSES = (0.0, 0.05, 0.1, 0.2, 0.3, 0.4)
TASKS = (3, 5, 8)
T_B_MS = 200.0                       # 보고서 54행 초기값: 방송 주기 sim 200 ms


@dataclasses.dataclass
class Cell:
    """스윕 셀 1개 (한 줄의 CSV)."""

    exp: str
    nt: int
    loss: float
    K: int = 3
    all_heard: bool = True
    H: Optional[int] = 5
    liveness: str = 'direct'
    rule: str = 'cbba'
    lock: str = 'optimistic'          # optimistic | exclusive
    mode: str = 'scatter'
    burst: float = 0.0
    event: bool = False
    fault: str = 'none'               # none | holder_drop | commit_die | holder_unhealthy
    cost: str = 'euclid'


def upper95(k: int, n: int) -> float:
    """비율의 단측 95 % 상한. k = 0 이면 정확값 1 − 0.05^(1/n) (≈ 3/n, rule of three), 아니면 Wilson."""
    if n <= 0:
        return float('nan')
    if k == 0:
        return 1.0 - 0.05 ** (1.0 / n)
    z = 1.6448536269514722
    p = k / n
    den = 1.0 + z * z / n
    centre = p + z * z / (2 * n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return min(1.0, (centre + half) / den)


def pct(xs: Sequence[float], q: float) -> float:
    """최근접 순위 백분위 (연구 계산 mrta_small.pct 와 같은 규칙)."""
    if not xs:
        return float('nan')
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * (len(s) - 1) + 0.5))]


_GRID = None


def _grid(map_yaml: str):
    global _GRID
    if _GRID is None:
        from amr_fleet.negotiation.grid import OccupancyGrid
        _GRID = OccupancyGrid.from_map_yaml(map_yaml, cell_m=0.5, inflate_m=0.25)
    return _GRID


def _case(cell: Cell, k: int, map_yaml: str = ''):
    if cell.cost == 'grid':
        grid = _grid(map_yaml)
        case = scenario.make_case(cell.nt, random.Random(55 * cell.nt + k), cell.mode,
                                  free_xy=grid.is_free_xy)
        model = GridAstarCost(grid, 1.0)
    else:
        case = scenario.make_case(cell.nt, random.Random(55 * cell.nt + k), cell.mode)
        model = EuclideanCost(1.0)
    return case, model.matrix(case.robots, case.tasks)


def _one(cell: Cell, case, cost, k: int) -> dict:
    cfg = AgentConfig(rule=cell.rule, stable_rounds=cell.K, require_all_heard=cell.all_heard,
                      heartbeat_timeout=cell.H, liveness=cell.liveness)
    ch = PerLinkChannel(seed=13 * k + 1, p_loss=cell.loss, burst_rho=cell.burst)
    lock = ExclusiveTaskLock() if cell.lock == 'exclusive' else OptimisticTaskLock()
    sim = SimConfig(max_rounds=120, event_driven=cell.event)
    drops, health = None, None
    if cell.fault == 'holder_drop':
        drops = {HOLDER: 3}
    elif cell.fault in ('commit_die', 'holder_unhealthy'):
        base = run_case(cost, case.robot_ids, case.task_ids, cfg,
                        PerLinkChannel(seed=13 * k + 1, p_loss=cell.loss, burst_rho=cell.burst),
                        sim, task_lock=ExclusiveTaskLock())
        if cell.fault == 'commit_die' and base.commit_round:
            first = min(base.commit_round, key=lambda r: (base.commit_round[r], r))
            drops = {first: base.commit_round[first] + 1}
            sim = SimConfig(max_rounds=60, termination='fixed', event_driven=cell.event)
        elif cell.fault == 'holder_unhealthy':
            victim = base.assignment.get(case.task_ids[0])
            if victim is not None:
                health = [(2, victim, False)]
    res = run_case(cost, case.robot_ids, case.task_ids, cfg, ch, sim, task_lock=lock,
                   dropouts=drops, health_events=health)
    out = {'dup': res.dup > 0, 'unassigned': res.unassigned > 0, 'stuck': res.stuck > 0,
           'rounds': res.rounds, 'broadcasts': res.broadcasts, 'wall': res.wall_ms,
           'inval': res.invalidations, 'denied': res.lock_denied,
           'last_commit': max(res.commit_round.values()) if res.commit_round else res.rounds}
    if cell.fault == 'none' and res.clean:
        a = assignment_indices(res, case.robot_ids, case.task_ids)
        g = scenario.central_greedy(cost)
        opt = scenario.total_cost(cost, scenario.optimal_assignment(_finite(cost)))
        tot = scenario.total_cost(cost, a)
        out['greedy_eq'] = a == g
        gtot = scenario.total_cost(cost, g)
        out['r_greedy'] = tot / gtot if gtot > 0 else 1.0
        out['r_opt'] = tot / opt if opt > 0 else 1.0
        out['g_opt'] = gtot / opt if opt > 0 else 1.0
    return out


def _finite(cost):
    big = 1e6
    return [[c if math.isfinite(c) else big for c in row] for row in cost]


def run_cell(args) -> dict:
    """셀 1개를 cases 사례로 돌려 요약 dict 를 돌려준다 (프로세스 풀 작업 단위)."""
    cell, cases, map_yaml = args
    t0 = time.time()
    rows = []
    for k in range(cases):
        case, cost = _case(cell, k, map_yaml)
        rows.append(_one(cell, case, cost, k))
    return summarize(cell, rows, time.time() - t0)


def summarize(cell: Cell, rows: List[dict], secs: float) -> dict:
    """사례별 결과 → 셀 요약."""
    n = len(rows)
    out = dataclasses.asdict(cell)
    for key in ('dup', 'unassigned', 'stuck'):
        k = sum(1 for r in rows if r[key])
        out[f'{key}_runs'] = k
        out[f'{key}_pct'] = 100.0 * k / n
        out[f'{key}_u95_pct'] = 100.0 * upper95(k, n)
    out['n'] = n
    rounds = [r['rounds'] for r in rows]
    out['rounds_mean'] = statistics.mean(rounds)
    out['rounds_p95'] = pct(rounds, 0.95)
    lc = [r['last_commit'] for r in rows]
    out['commit_mean'] = statistics.mean(lc)
    out['commit_p95'] = pct(lc, 0.95)
    out['commit_ms_mean'] = out['commit_mean'] * T_B_MS
    out['broadcasts_mean'] = statistics.mean(r['broadcasts'] for r in rows)
    out['wall_ms_mean'] = statistics.mean(r['wall'] for r in rows)
    out['invalidations_mean'] = statistics.mean(r['inval'] for r in rows)
    out['lock_denied_mean'] = statistics.mean(r['denied'] for r in rows)
    clean = [r for r in rows if 'greedy_eq' in r]
    out['clean_runs'] = len(clean)
    if clean:
        out['greedy_eq'] = sum(1 for r in clean if r['greedy_eq'])
        for key in ('r_greedy', 'r_opt', 'g_opt'):
            v = [r[key] for r in clean]
            out[f'{key}_mean'] = statistics.mean(v)
            out[f'{key}_p95'] = pct(v, 0.95)
            out[f'{key}_max'] = max(v)
    out['secs'] = secs
    return out


# ---------------------------------------------------------------------- 실험 정의
def cells_commit() -> Iterable[Cell]:
    for nt in TASKS:
        for loss in LOSSES:
            for K in (1, 2, 3, 5):
                for heard in (False, True):
                    yield Cell('commit', nt, loss, K, heard, None)
                    for H in (3, 5, 8):
                        for live in ('direct', 'indirect'):
                            yield Cell('commit', nt, loss, K, heard, H, live)


def cells_rule() -> Iterable[Cell]:
    for nt in (3, 5):
        for loss in LOSSES:
            for rule in ('max', 'cbba'):
                for K, heard in ((2, False), (3, True)):
                    for H in (None, 3, 5):
                        yield Cell('rule', nt, loss, K, heard, H, 'direct', rule)


def cells_dropout() -> Iterable[Cell]:
    for nt in (3, 5):
        for loss in (0.0, 0.1, 0.2, 0.4):
            for K, heard in ((2, False), (3, True)):
                for H in (None, 2, 3, 5, 8):
                    for live in (('direct',) if H is None else ('direct', 'indirect')):
                        for lock in ('optimistic', 'exclusive'):
                            yield Cell('dropout', nt, loss, K, heard, H, live, lock=lock,
                                       fault='holder_drop')


def cells_commitdie() -> Iterable[Cell]:
    for nt in (3, 5):
        for loss in (0.0, 0.2, 0.4):
            for H in (3, 5):
                for lock in ('optimistic', 'exclusive'):
                    yield Cell('commitdie', nt, loss, 3, True, H, 'direct', lock=lock,
                               fault='commit_die')


def cells_health() -> Iterable[Cell]:
    for nt in (3, 5):
        for loss in (0.0, 0.2, 0.4):
            for rule in ('max', 'cbba'):
                yield Cell('health', nt, loss, 3, True, 5, 'direct', rule, lock='exclusive',
                           fault='holder_unhealthy')


def cells_event() -> Iterable[Cell]:
    for nt in (3, 5):
        for loss in (0.05, 0.2, 0.4):
            for K, heard in ((2, False), (3, False), (3, True)):
                for ev in (False, True):
                    yield Cell('event', nt, loss, K, heard, None, event=ev)


def cells_burst() -> Iterable[Cell]:
    for nt in (3, 5):
        for loss in (0.05, 0.2, 0.4):
            for K, heard in ((2, False), (3, True)):
                for H, live in ((None, 'direct'), (5, 'direct'), (5, 'indirect')):
                    for burst in (0.0, 0.8):
                        yield Cell('burst', nt, loss, K, heard, H, live, burst=burst)


def cells_cost() -> Iterable[Cell]:
    for cost in ('euclid', 'grid'):
        for mode in ('spawn', 'scatter'):
            for nt in TASKS:
                for loss in (0.0, 0.2):
                    yield Cell('cost', nt, loss, 3, True, 5, 'indirect', mode=mode, cost=cost)


EXPERIMENTS = {
    'commit': cells_commit, 'rule': cells_rule, 'dropout': cells_dropout,
    'commitdie': cells_commitdie, 'health': cells_health, 'event': cells_event,
    'burst': cells_burst, 'cost': cells_cost,
}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--cases', type=int, default=1000)
    ap.add_argument('--cost-cases', type=int, default=300, help='cost 실험(격자 A*)의 사례 수')
    ap.add_argument('--workers', type=int, default=max(1, min(8, (os.cpu_count() or 2) // 4)))
    ap.add_argument('--only', default='', help='쉼표로 실험 이름 (기본: 전부)')
    ap.add_argument('--map', default='maps/warehouse.yaml')
    a = ap.parse_args(argv)
    names = [n for n in a.only.split(',') if n] or list(EXPERIMENTS)
    os.makedirs(a.out_dir, exist_ok=True)
    for name in names:
        cells = list(EXPERIMENTS[name]())
        cases = a.cost_cases if name == 'cost' else a.cases
        t0 = time.time()
        jobs = [(c, cases, a.map) for c in cells]
        with cf.ProcessPoolExecutor(max_workers=a.workers) as ex:
            results = list(ex.map(run_cell, jobs, chunksize=1))
        path = os.path.join(a.out_dir, f'{name}.csv')
        keys: List[str] = []
        for r in results:
            keys += [k for k in r if k not in keys]
        with open(path, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            w.writerows(results)
        print(f'[{name}] 셀 {len(cells)} × 사례 {cases} → {path} ({time.time() - t0:.0f} s)',
              flush=True)
    return 0


def read_csv(path: str) -> List[Dict[str, str]]:
    """스윕 CSV 를 읽는다 (문서 표 생성용)."""
    with open(path, newline='') as fh:
        return list(csv.DictReader(fh))


if __name__ == '__main__':
    sys.exit(main())
