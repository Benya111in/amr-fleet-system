#!/usr/bin/env python3
"""
보너스 4 P5 실험 계획 (사전 등록의 기계 판 — docs/research/fleet-distributed/notebook 의 P5 항목과 같은 값).

표준 라이브러리만 쓴다 (호스트 · 컨테이너 둘 다). 실험 · 짝 · 갈래 → 실행 계획 JSON (기록기 p5_recorder.py 가 읽는다),
통신 모의 YAML (에이전트 · 관리자 · 어댑터가 읽는다), 장애 일정 (호스트 p5_driver.py 가 실행한다).

시각: 모든 offset 은 t_base (sim) 기준 초. t_base = max(T_START, 준비 시각 + lead_s). 통신 모의의 분할 · 단절은 노드가
기동 때 파일로 읽으므로 **절대 sim 시각**(T_START + offset)으로 쓴다 — 준비가 T_START 뒤로 밀리면 어긋나므로 기록기가
그 사실을 summary.notes 에 남기고 분석이 그 실행을 표시한다.

작업 묶음 (시나리오 12 의 PLAN 과 같은 순서 — 고른 근거는 노트 항목): 도크 넷(서쪽 dock_1 · dock_2, 동쪽 dock_a · dock_b)
사이 작업, 앞 넷은 같은 통로를 반대 방향으로 지나는 쌍.
"""

import argparse
import json
import os
import sys

PLAN_PAIRS = [('dock_1', 'dock_a'), ('dock_a', 'dock_1'), ('dock_2', 'dock_b'),
              ('dock_b', 'dock_2'), ('dock_1', 'dock_b'), ('dock_a', 'dock_2'),
              ('dock_2', 'dock_a'), ('dock_b', 'dock_1')]
# behavior.yaml docks.*.staging (파일에서 다시 읽어 대조한다 — load_docks)
DOCK_STAGING = {'dock_1': (-27.78, 17.0, 3.14159265), 'dock_2': (-27.78, 13.0, 3.14159265),
                'dock_a': (27.78, 17.0, 0.0), 'dock_b': (27.78, 13.0, 0.0)}
ROBOTS = ['amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05']
# 절대 sim 시각 기준점 [s]. 파일럿(사전 등록 보완 단계)에서 준비 시각을 재고 정한다 — 여기 값이 실행에 쓰인 값이다.
T_START = float(os.environ.get('P5_T_START', '240'))
HORIZON = 660.0          # 시나리오 12 와 같은 상한 (작업당 600 s × ceil(5/5) + 60)

IDEAL_LINK = {'src': '*', 'dst': '*', 'comm_latency_ms': [0.0, 0.0], 'drop_rate': 0.0}


def batch(first: int, last: int, offset_s: float, prefix: str):
    """PLAN_PAIRS[first:last] → 작업 목록 (task_id = <prefix>.<번호 2 자리>)."""
    out = []
    for i in range(first, last):
        pick, drop = PLAN_PAIRS[i]
        out.append({'task_id': f'{prefix}.{i:02d}', 'pickup_dock': pick, 'dropoff_dock': drop,
                    'pickup': list(DOCK_STAGING[pick]), 'dropoff': list(DOCK_STAGING[drop]),
                    'offset_s': float(offset_s), 'priority': 100 - i, 'item_type': 'small',
                    'item_mass': 2.0})
    return out


def comm_partition_contention(t_start: float = None):
    """P: F5 분할 {amr_01, amr_02} | 나머지+서버 [40, 100) s, K 경합: 01 ↔ 02 단절 [200, 400) s."""
    t = T_START if t_start is None else float(t_start)
    return {'seed': 1, 'links': [IDEAL_LINK],
            'partitions': [{'start_s': t + 40.0, 'end_s': t + 100.0,
                            'groups': [['amr_01', 'amr_02']]}],
            'outages': [{'start_s': t + 200.0, 'end_s': t + 400.0, 'src': 'amr_01',
                         'dst': 'amr_02'},
                        {'start_s': t + 200.0, 'end_s': t + 400.0, 'src': 'amr_02',
                         'dst': 'amr_01'}]}


def comm_stress(t_start: float = None):
    """X (보고서 F7): 모든 링크 지연 U(0, 1000) ms · 손실 0.10 · Gilbert–Elliott 버스트 (좋음 50 s / 나쁨 2.5 s)."""
    return {'seed': 1, 'links': [{'src': '*', 'dst': '*', 'comm_latency_ms': [0.0, 1000.0],
                                  'drop_rate': 0.10,
                                  'burst': {'mean_good_s': 50.0, 'mean_bad_s': 2.5,
                                            'loss_good': 0.0, 'loss_bad': 1.0}}]}


def arm(mode, layout='new', faults=(), comm=None, label=''):
    return {'mode': mode, 'layout': layout, 'faults': [list(f) for f in faults], 'comm': comm,
            'label': label or mode}


# 장애: (offset_s, 동작, 대상). 서버 동작 kill · pause · unpause · start 의 대상은 서버 번호 (서버 1 은 갈래 시작 때 뜬다),
# 에이전트 동작 agent_kill · agent_restart 의 대상은 로봇 id (스택 컨테이너 안 p5_agent_ctl.py).
EXPERIMENTS = {
    'pilot': {'pairs': 2, 'question': '준비 시각 · RTF (짝 1 차례로 = 단독, 짝 2 동시) — 셈 밖',
              'arms': [arm('central'), arm('distributed')],
              'tasks': batch(0, 5, 0.0, 'p5pl'), 'horizon_s': HORIZON, 'lead_s': 0.0},
    'AA': {'pairs': 6, 'question': 'A/A 잡음 바닥 (분산 대 분산, 무장애)',
           'arms': [arm('distributed', label='dist_a'), arm('distributed', label='dist_b')],
           'tasks': batch(0, 5, 0.0, 'p5aa'), 'horizon_s': HORIZON, 'lead_s': 0.0},
    'C1': {'pairs': 8, 'question': 'F1 회귀: 중앙 대 분산 (무장애)',
           'arms': [arm('central'), arm('distributed')],
           'tasks': batch(0, 5, 0.0, 'p5c1'), 'horizon_s': HORIZON, 'lead_s': 0.0},
    'C2': {'pairs': 8, 'question': 'F3: 서버 컨테이너 kill (폴백 끔 = 중앙 대 켬 = 분산), 재기동',
           'arms': [arm('central', faults=[(60.0, 'kill', 1), (300.0, 'start', 2)]),
                    arm('distributed', faults=[(60.0, 'kill', 1), (300.0, 'start', 2)])],
           'tasks': batch(0, 5, 0.0, 'p5c2') + batch(5, 8, 90.0, 'p5c2'),
           'horizon_s': 720.0, 'min_end_s': 360.0, 'lead_s': 0.0},
    'C3': {'pairs': 8, 'question': 'F2: 협상(서버 장애 중 CBAA) 대 중앙 할당 — 같은 묶음',
           'arms': [arm('distributed', faults=[(-30.0, 'kill', 1)], label='dist_dead'),
                    arm('distributed', label='dist_alive')],
           'tasks': batch(0, 5, 0.0, 'p5c3'), 'horizon_s': HORIZON, 'lead_s': 40.0},
    'F5L': {'pairs': 8, 'question': 'F5 도크 배치: 옛(fd17e49 ±1 m) 대 새(±0.75 m), 중앙 모드',
            'arms': [arm('central', layout='old', label='central_old'),
                     arm('central', layout='new', label='central_new')],
            'tasks': batch(0, 5, 0.0, 'p5f5'), 'horizon_s': HORIZON, 'lead_s': 0.0},
    'S': {'pairs': 2, 'question': '보고서 F3 정지 20 s 뒤 재개 + F3/F10 정지 중 새 서버 → 옛 서버 재개 (split-brain)',
          'arms': [arm('distributed', faults=[(60.0, 'pause', 1), (80.0, 'unpause', 1),
                                              (240.0, 'pause', 1), (250.0, 'start', 2),
                                              (270.0, 'unpause', 1), (330.0, 'kill', 1)],
                       label='dist_s_a'),
                   arm('distributed', faults=[(60.0, 'pause', 1), (80.0, 'unpause', 1),
                                              (240.0, 'pause', 1), (250.0, 'start', 2),
                                              (270.0, 'unpause', 1), (330.0, 'kill', 1)],
                       label='dist_s_b')],
          'tasks': batch(0, 5, 0.0, 'p5s') + batch(5, 8, 200.0, 'p5s'),
          'horizon_s': 720.0, 'min_end_s': 360.0, 'lead_s': 0.0},
    'P': {'pairs': 2, 'question': '보고서 F5 부분 분할 + 경합(K: 서버 장애 중 amr_01↔amr_02 단절) + 재기동',
          'arms': [arm('distributed', faults=[(180.0, 'kill', 1), (420.0, 'start', 2)],
                       comm='partition_contention', label='dist_p_a'),
                   arm('distributed', faults=[(180.0, 'kill', 1), (420.0, 'start', 2)],
                       comm='partition_contention', label='dist_p_b')],
          'tasks': batch(0, 5, 0.0, 'p5p') + batch(5, 8, 210.0, 'p5p'),
          'horizon_s': 780.0, 'min_end_s': 460.0, 'lead_s': 0.0},
    'R': {'pairs': 2, 'question': 'p3 §8-7: 에이전트 재기동 (amr_03 SIGKILL → 같은 명령줄 · 저장소로 15 s 뒤)',
          'arms': [arm('distributed', faults=[(60.0, 'agent_kill', 'amr_03'),
                                              (75.0, 'agent_restart', 'amr_03')],
                       label='dist_r_a'),
                   arm('distributed', faults=[(60.0, 'agent_kill', 'amr_03'),
                                              (75.0, 'agent_restart', 'amr_03')],
                       label='dist_r_b')],
          'tasks': batch(0, 5, 0.0, 'p5r'), 'horizon_s': HORIZON, 'lead_s': 0.0},
    'X': {'pairs': 2, 'question': '보고서 F7 무장애 스트레스: 지연 U(0,1000) ms · 손실 0.1 · 버스트',
          'arms': [arm('distributed', comm='stress', label='dist_x_a'),
                   arm('distributed', comm='stress', label='dist_x_b')],
          'tasks': batch(0, 5, 0.0, 'p5x'), 'horizon_s': HORIZON, 'lead_s': 0.0},
}
# layopt 배치 최적화 연구 E3 (docs/research/layout-opt/gen.md): 배치 묶음 하나에서 5 대 한 번 (짝 아님 — p5_driver.py
# --run-one, --bundle). 1 차 5 작업 t_base, 2 차 같은 5 쌍 t_base + 300 s, 지평 900 s 고정
# (end_when_done 끔 — 2 차 뒤에도 대기 슬롯 체류를 같은 길이로 남긴다). 갈래 하나: LO 중앙 모드,
# LOd 분산 모드. LOsmoke 는 기동 확인용 (1 차만, 지평 60 s).
LO_TASKS = batch(0, 5, 0.0, 'plo1') + batch(0, 5, 300.0, 'plo2')
EXPERIMENTS['LO'] = {'pairs': 1, 'question': 'layopt E3: 배치 묶음 5 대 운용 (중앙)',
                     'arms': [arm('central', label='lo_central')], 'tasks': LO_TASKS,
                     'horizon_s': 900.0, 'end_when_done': False, 'lead_s': 0.0}
EXPERIMENTS['LOd'] = {'pairs': 1, 'question': 'layopt E3: 배치 묶음 5 대 운용 (분산)',
                      'arms': [arm('distributed', label='lo_dist')], 'tasks': LO_TASKS,
                      'horizon_s': 900.0, 'end_when_done': False, 'lead_s': 0.0}
EXPERIMENTS['LOsmoke'] = {'pairs': 1, 'question': 'layopt E3 하네스 기동 확인 (셈 밖)',
                          'arms': [arm('central', label='lo_smoke')],
                          'tasks': batch(0, 5, 0.0, 'plos'), 'horizon_s': 60.0,
                          'end_when_done': False, 'lead_s': 0.0}
# 파이프라인 시험 전용 (Gazebo 없음, p5_driver.py --dryrun): 장애 주입 경로 전부를 짧게
EXPERIMENTS['dry'] = {
    'pairs': 1, 'question': '파이프라인 시험 (셈 밖)',
    'arms': [arm('central', faults=[(10.0, 'pause', 1), (20.0, 'unpause', 1), (30.0, 'kill', 1),
                                    (40.0, 'start', 2)], label='dry_c'),
             arm('distributed', faults=[(10.0, 'pause', 1), (20.0, 'unpause', 1),
                                        (30.0, 'kill', 1), (40.0, 'start', 2)],
                 comm='partition_contention', label='dry_d')],
    'tasks': batch(0, 2, 0.0, 'p5dry'), 'horizon_s': 70.0, 'lead_s': 0.0}
COMM_MODELS = {'partition_contention': comm_partition_contention, 'stress': comm_stress}
# 실행 순서 (우선순위): A/A 를 먼저 (보고서 — 잡음 바닥을 잰 뒤 비교), 그다음 사전 등록 비교, 기술(記述) 장애.
ORDER = ['AA', 'C1', 'C2', 'C3', 'F5L', 'S', 'P', 'X', 'R']
DOMAINS = [(21, 23), (25, 27)]       # 짝마다 번갈아 (COORDINATION §10.1 배정 21–29, 짝 안 간격 2)


def schedule(order=None):
    """[(실험, 짝 번호 1.., 먼저 띄울 갈래 색인, (도메인 0 번 갈래, 1 번 갈래))] — 짝마다 먼저 띄우는 갈래를 번갈아."""
    out = []
    for exp in (order or ORDER):
        n = EXPERIMENTS[exp]['pairs']
        for k in range(1, n + 1):
            first = (k - 1) % 2
            d = DOMAINS[(k - 1) % 2]
            out.append((exp, k, first, d))
    return out


def run_plan(exp: str, pair: int, arm_index: int, t_start: float = None) -> dict:
    """기록기 계획 JSON."""
    e = EXPERIMENTS[exp]
    a = e['arms'][arm_index]
    return {'exp': exp, 'pair': pair, 'arm': arm_index, 'label': a['label'], 'mode': a['mode'],
            'layout': a['layout'], 'robots': ROBOTS,
            't_start': float(T_START if t_start is None else t_start),
            'lead_s': float(e.get('lead_s', 0.0)), 'horizon_s': float(e['horizon_s']),
            'min_end_s': float(e.get('min_end_s', 0.0)),
            'end_when_done': bool(e.get('end_when_done', True)),
            'tasks': e['tasks'], 'faults': a['faults'], 'comm': a['comm']}


def comm_yaml(name: str, t_start: float = None) -> str:
    """통신 모의 설정 → YAML 문자열 (JSON 은 YAML 의 부분집합). 절대 시각은 그 실행의 t_start 기준."""
    return json.dumps(COMM_MODELS[name](t_start), indent=1)


def check_docks(behavior_yaml: str) -> list:
    """behavior.yaml 의 도크 staging 이 DOCK_STAGING 과 같은지 (다르면 문제 목록)."""
    import re
    text = open(behavior_yaml, encoding='utf-8').read()
    problems = []
    for dock, (x, y, yaw) in DOCK_STAGING.items():
        m = re.search(rf'^\s*{dock}:\s*\{{staging:\s*\[([^\]]+)\]', text, re.M)
        if not m:
            problems.append(f'{dock}: behavior.yaml 에 없음')
            continue
        vals = [float(v) for v in m.group(1).split(',')]
        if any(abs(a - b) > 1e-6 for a, b in zip(vals, (x, y, yaw))):
            problems.append(f'{dock}: {vals} != {(x, y, yaw)}')
    return problems


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='P5 실험 계획')
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('schedule')
    s.add_argument('--order', default=','.join(ORDER))
    w = sub.add_parser('write')
    w.add_argument('--exp', required=True)
    w.add_argument('--pair', type=int, required=True)
    w.add_argument('--arm', type=int, required=True)
    w.add_argument('--out', required=True, help='계획 JSON 경로')
    w.add_argument('--comm-out', default='', help='통신 모의 YAML 경로 (갈래에 comm 이 있을 때)')
    w.add_argument('--t-start', type=float, default=None)
    c = sub.add_parser('check-docks')
    c.add_argument('behavior_yaml')
    args = ap.parse_args(argv)
    if args.cmd == 'schedule':
        for exp, k, first, d in schedule(args.order.split(',')):
            print(exp, k, first, d[0], d[1])
        return 0
    if args.cmd == 'check-docks':
        probs = check_docks(args.behavior_yaml)
        print('\n'.join(probs) or 'ok')
        return 1 if probs else 0
    plan = run_plan(args.exp, args.pair, args.arm, args.t_start)
    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump(plan, f, ensure_ascii=False, indent=1)
    if plan['comm']:
        if not args.comm_out:
            print('갈래에 통신 모의가 있는데 --comm-out 이 없다', file=sys.stderr)
            return 2
        with open(args.comm_out, 'w', encoding='utf-8') as f:
            f.write(comm_yaml(plan['comm'], plan['t_start']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
