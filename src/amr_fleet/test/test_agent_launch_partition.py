"""
launch 시험 ④c: DEGRADED 협상 중 2 | 3 분할 — 작업 ID 잠금 경합을 ROS 로 (보너스 4 P3 리뷰 반영).

에이전트 5 개 + 가짜 /clock (sim 100 s 에서 시작, 1×), 서버 없음 → 부팅 뒤 unknown_dead_s 지나 DEGRADED.
작업 6 개를 /fleet/task_board 에 올리고, 수신 측 통신 모의로 sim [PART_START, PART_END) 동안
{amr_01, amr_02} | {amr_03, amr_04, amr_05} 를 가른다. 작업은 로봇에서 멀어(이동 ≥ 8 s, 소수 쪽 ≥ 17 s)
소수 쪽의 확정(commit)이 분할 안에서 일어난다. 두 쪽 CBAA 는 서로를 못 보고 같은 작업을 따로 고를 수 있다.

판정 (사전 등록: notebook `…_2348_b4-p3_review-fix-proxy-leak-contention.md`):
- 소수 쪽(정족수 3 미만) 분할 중 픽업 시작 0, 같은 task_id 픽업 시작 2 회 이상 0.
- 공허 검사: 소수 쪽 분할 중 commit 요청 ≥ 1 (= 확정 거부 ≥ 1), 다수 쪽 분할 중 픽업 시작 ≥ 1.
  공허 검사가 안 맞으면 경합이 생기지 않은 것 — 판정 불가로 실패시킨다 (요약에 시각을 남긴다).

환경 변수: AMR_AGENT_LAUNCH_SEED(1), AMR_AGENT_LAUNCH_LOSS(0.2), AMR_AGENT_LAUNCH_RAW(원자료 디렉터리).
"""

import collections
import json
import os
import pathlib
import sys
import tempfile
import time
import unittest

from amr_msgs.msg import AgentState, Task
import launch
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock as ClockMsg

ROBOTS = [f'amr_0{i}' for i in range(1, 6)]
MINORITY = ('amr_01', 'amr_02')
MAJORITY = ('amr_03', 'amr_04', 'amr_05')
SEED = int(os.environ.get('AMR_AGENT_LAUNCH_SEED', '1'))
LOSS = float(os.environ.get('AMR_AGENT_LAUNCH_LOSS', '0.2'))
RAW = os.environ.get('AMR_AGENT_LAUNCH_RAW', '')
SIM_START = 100.0                    # fake_clock start_s 기본값
PART_START, PART_END = 118.0, 160.0  # 분할 [sim s)
SIM_STOP = PART_END + 15.0
TASK_X = (20.0, 22.0, 24.0, 26.0, 28.0, 30.0)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
BOARD_QOS = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _raw_dir() -> pathlib.Path:
    d = pathlib.Path(RAW) if RAW else pathlib.Path(tempfile.mkdtemp(prefix='agent_part_'))
    d.mkdir(parents=True, exist_ok=True)
    return d


@pytest.mark.launch_test
def generate_test_description():
    raw = _raw_dir()
    (raw / 'store').mkdir(exist_ok=True)
    comm = raw / 'comm.yaml'
    groups = f'[[{", ".join(MINORITY)}], [{", ".join(MAJORITY)}]]'
    comm.write_text(
        f'seed: {SEED}\n'
        'links:\n'
        f'  - {{src: "amr_*", dst: "amr_*", comm_latency_ms: [0.0, 100.0], drop_rate: {LOSS}}}\n'
        'partitions:\n'
        f'  - {{start_s: {PART_START}, end_s: {PART_END}, groups: {groups}}}\n',
        encoding='utf-8')
    procs = [ExecuteProcess(cmd=[sys.executable, '-m', 'amr_fleet.agent.fake_clock', '--ros-args',
                                 '-p', f'start_s:={SIM_START}'],
                            name='fake_clock', output='log')]
    for i, r in enumerate(ROBOTS):
        procs.append(ExecuteProcess(
            cmd=[sys.executable, '-m', 'amr_fleet.agent.node', '--ros-args',
                 '-r', f'__ns:=/{r}', '-p', 'use_sim_time:=true',
                 '-p', f'store_dir:={raw / "store"}', '-p', f'event_log:={raw}/events_{r}.jsonl',
                 '-p', 'sim_execution:=true', '-p', 'sim.pickup_s:=1.0',
                 '-p', 'sim.deliver_s:=1.0',
                 '-p', 'sim.commit_timeout_s:=25.0', '-p', 'task_hold_s:=60.0',
                 '-p', 'vote.tentative_s:=10.0',
                 '-p', 'simulate_latency:=true', '-p', f'comm_model_file:={comm}',
                 '-p', f'seed:={SEED}',
                 '-p', f'initial_pose:=[{3.0 * i}, {float(i % 2)}]'],
            name=f'agent_{r}', output='log'))
    return launch.LaunchDescription(procs + [launch_testing.actions.ReadyToTest()]), {
        'raw': raw}


def _events(raw: pathlib.Path):
    out = []
    for f in sorted(raw.glob('events_*.jsonl')):
        for line in f.read_text(encoding='utf-8').splitlines():
            if line.strip():
                out.append(json.loads(line))
    return out


def _in_partition(t: float) -> bool:
    return PART_START <= t < PART_END


class TestAgentPartition(unittest.TestCase):
    """④c 한 시나리오."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def test_partition_minority_cannot_start_pickups(self, raw):
        node = rclpy.create_node('agent_partition_observer')
        state, sim = {}, [0.0]
        node.create_subscription(
            ClockMsg, '/clock',
            lambda m: sim.__setitem__(0, m.clock.sec + m.clock.nanosec * 1e-9),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        for r in ROBOTS:
            node.create_subscription(AgentState, f'/{r}/fleet_agent/state',
                                     lambda m, r=r: state.__setitem__(r, m), LATCHED)
        board = node.create_publisher(Task, '/fleet/task_board', BOARD_QOS)
        summary = {'seed': SEED, 'loss': LOSS, 'raw': str(raw),
                   'partition': [PART_START, PART_END], 'minority': MINORITY,
                   'load_start': os.getloadavg()}
        try:
            for i, x in enumerate(TASK_X):
                t = Task()
                t.task_id = f'b4p3.part.{SEED:02d}{i:02d}'
                t.status = Task.STATUS_PENDING
                t.pickup_pose.header.frame_id = 'map'
                t.pickup_pose.pose.position.x = x
                t.pickup_pose.pose.position.y = 0.0
                board.publish(t)
            end = time.monotonic() + 150.0
            while time.monotonic() < end and sim[0] < SIM_STOP:
                rclpy.spin_once(node, timeout_sec=0.05)
            summary['sim_reached'] = sim[0]
            summary['modes_final'] = {r: (state[r].mode if r in state else None) for r in ROBOTS}
            self._judge(raw, summary)
        finally:
            summary['load_end'] = os.getloadavg()
            (raw / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1,
                                                         default=str), encoding='utf-8')
            node.destroy_node()

    def _judge(self, raw, summary):
        evs = _events(raw)
        starts = collections.defaultdict(list)
        for e in evs:
            if e['event'] == 'pickup_start':
                starts[e['task_id']].append((e['robot'], round(e['t'], 2)))
        dups = {t: rs for t, rs in starts.items() if len(rs) > 1}
        minority_pickups = [(t, r, ts) for t, rs in starts.items() for r, ts in rs
                            if r in MINORITY and _in_partition(ts)]
        majority_pickups = [(t, r, ts) for t, rs in starts.items() for r, ts in rs
                            if r in MAJORITY and _in_partition(ts)]
        minority_commits = [(e['robot'], e['task_id'], round(e['t'], 2)) for e in evs
                            if e['event'] == 'commit_request' and e['robot'] in MINORITY
                            and _in_partition(e['t'])]
        granted = [(e['robot'], e['task_id'], round(e['t'], 2)) for e in evs
                   if e['event'] == 'task_granted']
        degraded_at = {r: min((e['t'] for e in evs if e['robot'] == r and e['event'] == 'mode'
                               and e.get('mode') == 'DEGRADED'), default=None) for r in ROBOTS}
        summary['④c'] = {
            'pickup_starts': sum(len(v) for v in starts.values()), 'starts': dict(starts),
            'duplicates': dups, 'minority_pickups_in_partition': minority_pickups,
            'majority_pickups_in_partition': majority_pickups,
            'minority_commits_in_partition': minority_commits,
            'failed': [(e['robot'], e['task_id'], e.get('reason'), round(e['t'], 2)) for e in evs
                       if e['event'] == 'failed'],
            'task_granted': granted, 'first_degraded': degraded_at,
            'done': sorted({e['task_id'] for e in evs if e['event'] == 'done'})}
        self.assertGreaterEqual(summary['sim_reached'], SIM_STOP, summary)
        # 공허 검사 (경합이 실제로 생겼나)
        self.assertGreaterEqual(len(minority_commits), 1, summary['④c'])
        self.assertGreaterEqual(len(majority_pickups), 1, summary['④c'])
        # 판정
        self.assertEqual(minority_pickups, [], summary['④c'])
        self.assertEqual(dups, {}, summary['④c'])
