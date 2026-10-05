#!/usr/bin/env python3
"""작업 할당 최적성 비교 (명세 4장 9절 "최적성 지표(총 이동 거리, 작업 완료 시간)를 측정하고 분석").

왜 오프라인인가: `allocation.py` 는 rclpy 비의존 순수 모듈이고 최적성 지표는 할당 **결정**의
함수다 (주행 결과가 아니다). 그래서 Gazebo 없이 세 전략을 **같은 입력**으로 돌려 비교할 수 있고,
그래야 공정 비교가 된다 — 통합 실행은 기본값 `nearest` 하나만 쓰므로
`logs/allocation_*.csv` 1,117 건 전부 단일 전략이고 비교가 불가능하다.

입력은 실제 배치다: 도크 네 곳(`amr_behavior/config/behavior.yaml` docks 의 staging),
로봇 다섯 대 스폰(`amr_bringup/config/fleet_spawn.yaml`).

사용: python3 scripts/allocation_compare.py [--repeat N] [--csv 경로]
"""
import argparse
import itertools
import math
import os
import random
import statistics as st
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'amr_fleet'))

from amr_fleet import allocation as al          # noqa: E402
from amr_fleet.task_schema import Pose2D, TaskSpec   # noqa: E402

# 실제 배치 — behavior.yaml docks.*.staging
DOCKS = {
    'dock_1': (-27.78, 17.0), 'dock_2': (-27.78, 13.0),
    'dock_a': (27.78, 17.0), 'dock_b': (27.78, 13.0),
}
# 실제 스폰 — fleet_spawn.yaml
SPAWNS = [('amr_01', 18.0, -16.0), ('amr_02', 20.0, -16.0), ('amr_03', 22.0, -16.0),
          ('amr_04', 24.0, -16.0), ('amr_05', 26.0, -16.0)]
NOMINAL_SPEED = 1.0       # fleet.yaml 의 nominal_speed 와 같게 둔다


def make_case(n_tasks, n_robots, rng):
    """집하·배송 도크를 무작위로 짝지은 작업 n_tasks 건 + 로봇 n_robots 대."""
    names = list(DOCKS)
    tasks = []
    for i in range(n_tasks):
        a, b = rng.sample(names, 2)
        tasks.append(TaskSpec(
            task_id=f't{i}', pickup=Pose2D(*DOCKS[a]), dropoff=Pose2D(*DOCKS[b]),
            item_type='box', item_mass=5.0))
    robots = [al.RobotInfo(robot_id=r, x=x, y=y) for r, x, y in SPAWNS[:n_robots]]
    return tasks, robots


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--repeat', type=int, default=200, help='규모 조합마다 무작위 사례 수')
    ap.add_argument('--csv', default='')
    a = ap.parse_args(argv)

    strategies = al.available_strategies()
    sizes = [(n, r) for r in (2, 3, 5) for n in (2, 3, 5, 8)]
    rows = []
    for n_tasks, n_robots in sizes:
        acc = {s: {'dist': [], 'mk': [], 'ms': [], 'n': []} for s in strategies}
        for k in range(a.repeat):
            rng = random.Random(hash((n_tasks, n_robots, k)) & 0xFFFFFFFF)
            tasks, robots = make_case(n_tasks, n_robots, rng)
            for s in strategies:
                res = al.allocate(al.create_strategy(s, nominal_speed=NOMINAL_SPEED),
                                  tasks, robots)
                m = res.metrics
                acc[s]['dist'].append(m['total_travel_distance_m'])
                acc[s]['mk'].append(m['makespan_s'])
                acc[s]['ms'].append(m['compute_time_ms'])
                acc[s]['n'].append(len(res.assignments))
        for s in strategies:
            d = acc[s]
            rows.append(dict(n_tasks=n_tasks, n_robots=n_robots, strategy=s,
                             assigned=st.mean(d['n']),
                             dist=st.mean(d['dist']), mk=st.mean(d['mk']),
                             ms=st.mean(d['ms'])))

    w = max(len(s) for s in strategies)
    print(f'사례 {a.repeat} 회/조합 · 전략 {strategies} · nominal_speed {NOMINAL_SPEED} m/s\n')
    print(f'{"작업":>4s} {"로봇":>4s} {"전략":<{w}s} {"배정":>5s} '
          f'{"총이동(m)":>10s} {"makespan(s)":>12s} {"계산(ms)":>9s}  {"기준대비":>8s}')
    for (n_tasks, n_robots), grp in itertools.groupby(rows, key=lambda r: (r['n_tasks'], r['n_robots'])):
        g = list(grp)
        base = next(x for x in g if x['strategy'] == 'nearest')
        for r in g:
            rel = 100.0 * (r['mk'] - base['mk']) / base['mk'] if base['mk'] else 0.0
            tag = '기준' if r['strategy'] == 'nearest' else f'{rel:+6.1f} %'
            print(f'{n_tasks:>4d} {n_robots:>4d} {r["strategy"]:<{w}s} {r["assigned"]:>5.2f} '
                  f'{r["dist"]:>10.2f} {r["mk"]:>12.2f} {r["ms"]:>9.3f}  {tag:>8s}')
        print()

    print('전략별 전체 평균:')
    for s in strategies:
        g = [r for r in rows if r['strategy'] == s]
        print(f'  {s:<{w}s} 총이동 {st.mean(r["dist"] for r in g):8.2f} m · '
              f'makespan {st.mean(r["mk"] for r in g):7.2f} s · '
              f'계산 {st.mean(r["ms"] for r in g):6.3f} ms')

    if a.csv:
        import csv as _csv
        with open(a.csv, 'w', newline='') as fh:
            wr = _csv.DictWriter(fh, fieldnames=list(rows[0]))
            wr.writeheader()
            wr.writerows(rows)
        print(f'\n기록: {a.csv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
