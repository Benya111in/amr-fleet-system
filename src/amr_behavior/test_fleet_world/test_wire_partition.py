"""
launch 시험 W2 — DEGRADED 협상 할당을 실제 실행기 5 대가 수행, 2 | 3 분할 중 작업 ID 잠금 (④a · ④c, b4-wire).

서버 없음 → 부팅 뒤 DEGRADED. /fleet/task_board 에 작업 6 개 (분할 쪽마다 3 개, 로봇에서 18 m 안팎 — 이동 중에
분할이 시작된다). 수신 측 통신 모의로 sim [112, 175) 동안 {amr_01, amr_02} | {amr_03, amr_04, amr_05}.
협상 낙찰은 에이전트가 실제 실행기 executor/assign_task 로 넘기고, 실행기 BT 의 PickupLocked 가 픽업 직전 확정한다.

판정 (사전 등록: notebook `*_b4-wire_wire-launch-tests.md`):
- 소수 쪽(정족수 3 미만) 분할 중 작업 잠금 확정(HELD = 픽업 허가) 0, 같은 작업 적재 2 대 이상 0,
  모든 적재는 그 로봇의 그 작업 확정(held 사건) 뒤.
- 공허 검사: 소수 쪽 분할 중 확정 요청 ≥ 1, 다수 쪽 분할 중 확정 ≥ 1. 안 맞으면 경합이 없었던 것 — 실패로 둔다.
- 활성(보고만): 끝까지 완료한 작업 수, task_lock_failed 수, 낡은 LOCKED 정리 수.
"""

import collections
import json
import os
import unittest

from amr_msgs.msg import AgentState, Task
import harness as H
import launch
import launch_testing.actions
import pytest
import rclpy

ROBOTS = [f'amr_0{i}' for i in range(1, 6)]
MINORITY = ('amr_01', 'amr_02')
MAJORITY = ('amr_03', 'amr_04', 'amr_05')
POSES = [(0.0, 12.0, 0.0), (0.0, 14.0, 0.0), (6.0, -15.0, 0.0), (8.0, -15.0, 0.0),
         (10.0, -15.0, 0.0)]
SEED = int(os.environ.get('AMR_WIRE_SEED', '1'))
LOSS = float(os.environ.get('AMR_WIRE_LOSS', '0.2'))
SIM_START = 100.0
# 분할은 낙찰(sim ≈ 107) 뒤 · 다수 쪽 첫 확정(이동 8–18 m) 전에 시작해, 소수 쪽 확정 시간 초과(30 s)가 분할 안에서
# 끝나게 한다 (개발 실행 w2a 에서 125 로 두었더니 다수 쪽 확정이 모두 분할 전이라 공허 검사가 실패했다)
PART_START, PART_END = 112.0, 175.0
TASKS = ([(f'w2.n{i}', (18.0 + 3 * i, 13.0, 0.0), (18.0 + 3 * i, 11.0, -1.5708))
          for i in range(3)]
         + [(f'w2.s{i}', (18.0 + 3 * i, -15.0, 0.0), (18.0 + 3 * i, -17.0, -1.5708))
            for i in range(3)])


@pytest.mark.launch_test
def generate_test_description():
    raw = H.raw_dir('AMR_WIRE_RAW', 'wire_part_')
    groups = f'[[{", ".join(MINORITY)}], [{", ".join(MAJORITY)}]]'
    comm = H.comm_yaml(raw, SEED, LOSS, partitions=(
        f'  - {{start_s: {PART_START}, end_s: {PART_END}, groups: {groups}}}\n'))
    procs = H.processes(raw, ROBOTS, POSES, comm, SEED, sim_start=SIM_START,
                        with_adapters=False)
    return launch.LaunchDescription(procs + [launch_testing.actions.ReadyToTest()]), {'raw': raw}


class TestWirePartition(unittest.TestCase):
    """④a · ④c."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def test_partition_minority_cannot_pick_up(self, raw):
        obs = H.Observer(raw, ROBOTS)
        summary = {'seed': SEED, 'loss': LOSS, 'raw': str(raw),
                   'partition': [PART_START, PART_END], 'load_start': os.getloadavg()}
        try:
            self._run(obs, raw, summary)
        finally:
            summary['load_end'] = os.getloadavg()
            (raw / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1,
                                                         default=str), encoding='utf-8')
            obs.close()

    def _run(self, obs, raw, summary):
        self.assertTrue(obs.spin_for(60.0, until=lambda: obs.all_mode(AgentState.MODE_DEGRADED)),
                        f'DEGRADED 안 됨: {obs.modes()}')
        summary['duplicate_nodes'] = obs.duplicates()
        self.assertEqual(summary['duplicate_nodes'], {}, '격리 실패 — 같은 이름 노드가 둘 이상')
        evs = H.file_events(raw)
        summary['posted_sim'] = max(e['t'] for e in evs)
        for task_id, pick, drop in TASKS:
            obs.post(task_id, pick, drop)

        def sim_now():
            es = H.file_events(raw)
            return max(e['t'] for e in es) if es else 0.0
        # 분할이 끝나고 실패한 작업이 다시 잡혀 끝날 때까지 (sim PART_END + 70 s 상한)
        ids = [t for t, _, _ in TASKS]
        obs.spin_for(400.0, until=lambda: all(
            any(s == Task.STATUS_COMPLETED for _, s in obs.statuses(t)) for t in ids)
            or sim_now() > PART_END + 70.0)
        obs.spin_for(2.0)
        evs = H.file_events(raw)
        loads = H.loads(obs, raw)

        def held_task(e):
            return [x for x in e['added'] if x.startswith('task:')] if e['event'] == 'held' else []
        in_part = [e for e in evs if PART_START <= e['t'] < PART_END]
        minority_held = [(e['robot'], x, e['t']) for e in in_part if e['robot'] in MINORITY
                         for x in held_task(e)]
        majority_held = [(e['robot'], x, e['t']) for e in in_part if e['robot'] in MAJORITY
                         for x in held_task(e)]
        minority_commits = [(e['robot'], e['task_id'], e['t']) for e in in_part
                            if e['robot'] in MINORITY and e['event'] == 'task_commit_request']
        by_task = collections.defaultdict(list)
        for r, t, _ in loads:
            by_task[t].append(r)
        dups = {t: rs for t, rs in by_task.items() if len(rs) > 1}
        # 적재는 확정 뒤에만: 적재 벽시계보다 앞선 그 로봇의 held(task:<id>) 사건이 있어야 한다
        first_held = {}
        for e in evs:
            for x in held_task(e):
                first_held.setdefault((e['robot'], x[len('task:'):]), e['wall'])
        unconfirmed = [(r, t) for r, t, w in loads if not (first_held.get((r, t), 1e18) <= w)]
        logs = {r: H.exec_log(raw, r) for r in ROBOTS}
        summary.update({
            'minority_held_in_partition': minority_held,
            'majority_held_in_partition': majority_held,
            'minority_commit_requests_in_partition': minority_commits,
            'loads': loads, 'duplicate_loads': dups, 'loads_without_prior_held': unconfirmed,
            'completed': sorted({t for t in ids if any(
                s == Task.STATUS_COMPLETED for _, s in obs.statuses(t))}),
            'lock_failed': {r: [x[1] for x in logs[r] if x[0] == 'failed'
                                and x[2] == 'task_lock_failed'] for r in ROBOTS},
            'nego_dispatch': [(e['robot'], e['task_id'], e['t']) for e in evs
                              if e['event'] == 'nego_dispatch'],
            'stale_lock_cleared': [(e['robot'], e['task_id'], e['holder'], e['t']) for e in evs
                                   if e['event'] == 'stale_lock_cleared'],
            'sim_end': max(e['t'] for e in evs)})
        self.assertEqual(minority_held, [])
        self.assertEqual(dups, {})
        self.assertEqual(unconfirmed, [])
        self.assertGreaterEqual(len(minority_commits), 1, '소수 쪽 분할 중 확정 요청 없음 (공허)')
        self.assertGreaterEqual(len(majority_held), 1, '다수 쪽 분할 중 확정 없음 (공허)')
