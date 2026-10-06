"""
시나리오 집합 읽기·검사·선택 (rclpy 비의존) — rl/eval/gazebo/scenarios_v1.json (make_scenarios.py 가 만든다).

선택: 'smoke' (사전 등록한 스모크 20 = 주 10 + 스트레스 10, 실행 순서대로), 'all' (exec_order 전체),
'main' · 'stress' (exec_order 에서 그 집합만), 또는 쉼표로 이은 id 목록. --start/--count 로 잘라 이어 돌 수 있다.
"""

import json
import math

REQUIRED = ('id', 'set', 'scenario_seed', 'start', 'goal', 'path_length_m', 'time_limit_s')


def load(path):
    """JSON → dict (검사 통과 시). 문제는 ValueError."""
    with open(path, encoding='utf-8') as f:
        d = json.load(f)
    problems = check(d)
    if problems:
        raise ValueError('; '.join(problems[:10]))
    return d


def check(d):
    """문제 문자열 목록 (없으면 빈 목록)."""
    out = []
    ids = set()
    for sc in d.get('scenarios', []):
        for k in REQUIRED:
            if k not in sc:
                out.append(f'{sc.get("id")}: {k} 없음')
        if sc.get('id') in ids:
            out.append(f'id 중복 {sc.get("id")}')
        ids.add(sc.get('id'))
        if sc.get('set') not in ('main', 'stress'):
            out.append(f'{sc.get("id")}: set {sc.get("set")}')
        st = sc.get('goal_straight_m')
        st = math.inf if st is None else st
        if sc.get('set') == 'main' and st < d.get('rules', {}).get('main_straight_min_m', 0.4):
            out.append(f'{sc.get("id")}: 주 집합인데 직진 여유 {st}')
        if sc.get('set') == 'stress' and st >= d.get('rules', {}).get('main_straight_min_m', 0.4):
            out.append(f'{sc.get("id")}: 스트레스인데 직진 여유 {st}')
        L = sc.get('path_length_m', 0)
        if not (3.0 <= L <= 30.0):
            out.append(f'{sc.get("id")}: 길이 {L}')
        if abs(sc.get('time_limit_s', 0) - min(120.0, 20.0 + 3.0 * L)) > 0.01:
            out.append(f'{sc.get("id")}: 시간 상한 {sc.get("time_limit_s")}')
    for i in d.get('exec_order', []):
        if i not in ids:
            out.append(f'exec_order 의 모르는 id {i}')
    if len(set(d.get('exec_order', []))) != len(d.get('exec_order', [])):
        out.append('exec_order 중복')
    for i in d.get('smoke', []):
        if i not in ids:
            out.append(f'smoke 의 모르는 id {i}')
    return out


def select(d, which='smoke', start=0, count=None):
    """선택 규칙대로 시나리오 dict 목록 (실행 순서)."""
    by_id = {sc['id']: sc for sc in d['scenarios']}
    if which == 'smoke':
        ids = list(d['smoke'])
    elif which == 'all':
        ids = list(d['exec_order'])
    elif which in ('main', 'stress'):
        ids = [i for i in d['exec_order'] if by_id[i]['set'] == which]
    else:
        ids = [x.strip() for x in which.split(',') if x.strip()]
        missing = [i for i in ids if i not in by_id]
        if missing:
            raise KeyError(f'모르는 시나리오 {missing}')
    ids = ids[start:] if count is None else ids[start:start + count]
    return [by_id[i] for i in ids]
