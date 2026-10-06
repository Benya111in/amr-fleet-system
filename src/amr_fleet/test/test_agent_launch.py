"""
launch 시험: 에이전트 5 개 + 가짜 /clock 을 띄우고(시뮬레이터 없음) 서버 프로세스를 죽였다 살린다 (보너스 4 P3).

시나리오 하나에 네 판정 (사전 등록: docs/research/fleet-distributed/notebook/entries/*_b4-p3_ros-integration-*):
① 서버(fleet_manager_node + 하트비트를 한 프로세스에 묶은 manager_shim) SIGKILL 전후로 /fleet_p2p/* 의 에이전트
  발행자 · 구독자 5/5 유지, 비컨이 계속 흐름, 한 구역을 5 대가 번갈아 잡는 동안 로봇별 허가 구간이 겹치지 않음
  (각 로봇의 'held' 사건 sim 시각 — 모두 같은 /clock), kill 뒤에도 허가가 이어짐
② 모드: NORMAL → HOLD_NEW → DEGRADED → (관찰자 복귀 서버) RESYNC → NORMAL 을 5 대 모두 지남
③ 낮은 (epoch, leader_id) 의 assign_task 는 프록시가 거부하고 가짜 실행기까지 가지 않음, 낮은 epoch 하트비트는
  생존 신호가 아님 (옛 권위자가 살아 있어도 진짜 서버가 죽으면 DEGRADED)
④b (리뷰 반영, NORMAL) 중앙 프록시 경로의 작업 ID 잠금: 실행기가 거부한 작업을 다른 로봇에 다시 주면 수락 → HELD,
  같은 task_id 를 두 로봇 프록시에 주고 둘 다 COMMIT_TASK → 허가 구간 겹침 0, HELD 한 대, 확정 거부 ≥ 1
④a DEGRADED 에서 /fleet/task_board 작업을 CBAA + 작업 ID 잠금으로 나눠 가짜 실행: 같은 task_id 픽업 시작 2 회 이상 0.
  이 단계는 경합을 만들지 않는다 (시드 1–4 에서 경합 0 — 리뷰) — 잠금 효과의 증거는 ④b 와 분할 시험
  (test_agent_launch_partition.py, ④c) 이다. 경합 수는 요약에 함께 적는다.

통신 모의: 로봇↔로봇 링크 손실 AMR_AGENT_LAUNCH_LOSS(기본 0.2) · 지연 U(0, 100) ms, 서버→로봇은 지연만.
환경 변수: AMR_AGENT_LAUNCH_SEED(1), AMR_AGENT_LAUNCH_RAW(원자료 디렉터리, 기본 임시), AMR_AGENT_LAUNCH_TASKS(10).
"""

import collections
import json
import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from amr_msgs.msg import AgentBeacon, AgentState, Task
from amr_msgs.srv import AssignTask, ZoneCommand
import launch
from launch.actions import ExecuteProcess
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

ROBOTS = [f'amr_0{i}' for i in range(1, 6)]
SEED = int(os.environ.get('AMR_AGENT_LAUNCH_SEED', '1'))
LOSS = float(os.environ.get('AMR_AGENT_LAUNCH_LOSS', '0.2'))
N_TASKS = int(os.environ.get('AMR_AGENT_LAUNCH_TASKS', '10'))
RAW = os.environ.get('AMR_AGENT_LAUNCH_RAW', '')
ZONE = 'zone:x_ab_1'
RESYNC_S = 3.0
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
BOARD_QOS = QoSProfile(depth=256, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _raw_dir() -> pathlib.Path:
    d = pathlib.Path(RAW) if RAW else pathlib.Path(tempfile.mkdtemp(prefix='agent_launch_'))
    d.mkdir(parents=True, exist_ok=True)
    return d


@pytest.mark.launch_test
def generate_test_description():
    raw = _raw_dir()
    (raw / 'store').mkdir(exist_ok=True)
    comm = raw / 'comm.yaml'
    comm.write_text(
        f'seed: {SEED}\n'
        'links:\n'
        f'  - {{src: "amr_*", dst: "amr_*", comm_latency_ms: [0.0, 100.0], drop_rate: {LOSS}}}\n'
        '  - {src: "fleet", dst: "*", comm_latency_ms: [0.0, 100.0], drop_rate: 0.0}\n',
        encoding='utf-8')
    procs = [ExecuteProcess(cmd=[sys.executable, '-m', 'amr_fleet.agent.fake_clock'],
                            name='fake_clock', output='log')]
    for i, r in enumerate(ROBOTS):
        procs.append(ExecuteProcess(
            cmd=[sys.executable, '-m', 'amr_fleet.agent.node', '--ros-args',
                 '-r', f'__ns:=/{r}', '-p', 'use_sim_time:=true',
                 '-p', f'store_dir:={raw / "store"}', '-p', f'event_log:={raw}/events_{r}.jsonl',
                 '-p', f'modes.resync_s:={RESYNC_S}', '-p', 'sim_execution:=true',
                 '-p', 'sim.pickup_s:=1.0', '-p', 'sim.deliver_s:=1.0',
                 '-p', 'task_hold_s:=20.0', '-p', 'vote.tentative_s:=10.0',
                 '-p', 'simulate_latency:=true', '-p', f'comm_model_file:={comm}',
                 '-p', f'seed:={SEED}', '-p', 'proxy_assign_task:=true',
                 '-p', f'initial_pose:=[{3.0 * i}, {float(i % 2)}]'],
            name=f'agent_{r}', output='log'))
    return launch.LaunchDescription(procs + [launch_testing.actions.ReadyToTest()]), {
        'raw': raw}


class _Observer:
    """시험 프로세스의 관찰 · 조작 노드 (상태 · 사건 구독, 명령 서비스, 가짜 실행기)."""

    def __init__(self, raw: pathlib.Path):
        self.raw = raw
        self.node = rclpy.create_node('agent_launch_observer')
        self.state = {}
        self.events = []
        self.beacons = []
        self.exec_calls = collections.Counter()
        self.reject_exec = set()                   # 이 로봇들의 가짜 실행기는 거부한다 (④b)
        self.cmd = {}
        self.assign = {}
        for r in ROBOTS:
            self.node.create_subscription(AgentState, f'/{r}/fleet_agent/state',
                                          lambda m, r=r: self.state.__setitem__(r, m), LATCHED)
            self.node.create_subscription(String, f'/{r}/fleet_agent/events',
                                          self._on_event, 100)
            self.cmd[r] = self.node.create_client(ZoneCommand, f'/{r}/fleet_agent/zone_command')
            self.assign[r] = self.node.create_client(AssignTask, f'/{r}/assign_task')
            self.node.create_service(AssignTask, f'/{r}/executor/assign_task',
                                     lambda req, resp, r=r: self._fake_exec(r, req, resp))
        self.node.create_subscription(
            AgentBeacon, '/fleet_p2p/beacon',
            lambda m: self.beacons.append((time.monotonic(), m.robot_id)),
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.board = self.node.create_publisher(Task, '/fleet/task_board', BOARD_QOS)
        self.servers = {}

    def _on_event(self, msg: String) -> None:
        self.events.append(json.loads(msg.data))

    def file_events(self):
        """에이전트가 직접 쓴 사건 로그(JSONL) 전부 — 판정은 토픽이 아니라 이 원자료로 한다."""
        out = []
        for f in sorted(self.raw.glob('events_*.jsonl')):
            for line in f.read_text(encoding='utf-8').splitlines():
                if line.strip():
                    out.append(json.loads(line))
        return out

    def _fake_exec(self, r, req, resp):
        self.exec_calls[r] += 1
        if r in self.reject_exec:
            resp.success, resp.robot_id, resp.message = False, r, 'fake reject busy:other'
            return resp
        resp.success, resp.robot_id, resp.message = True, r, 'fake executor'
        return resp

    def assign_task(self, r, task_id, epoch, leader):
        """프록시 assign_task 1 회 — 결과와 실행기 도달 수."""
        req = AssignTask.Request()
        req.task.task_id = task_id
        req.task.epoch, req.task.leader_id = int(epoch), leader
        before = self.exec_calls[r]
        resp = self.call(self.assign[r], req)
        return {'success': None if resp is None else resp.success,
                'message': None if resp is None else resp.message,
                'reached_executor': self.exec_calls[r] - before}

    def holds(self, r, res) -> bool:
        st = self.state.get(r)
        return st is not None and res in st.held

    def denied(self, r, res) -> bool:
        st = self.state.get(r)
        return st is not None and res in st.denied

    # ------------------------------------------------------------------ 진행
    def spin_for(self, seconds: float, until=None, step=None) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if step is not None:
                step()
            if until is not None and until():
                return True
        return until is None

    def call(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return fut.result() if fut.done() else None

    def modes(self):
        return {r: (self.state[r].mode if r in self.state else None) for r in ROBOTS}

    def all_mode(self, code) -> bool:
        return all(m == code for m in self.modes().values())

    def server(self, name: str, *args: str) -> subprocess.Popen:
        log = open(self.raw / f'{name}.log', 'w')
        p = subprocess.Popen([sys.executable, '-m', 'amr_fleet.agent.manager_shim',
                              '--log-dir', str(self.raw / name), *args,
                              '--ros-args', '-p', 'use_sim_time:=true'],
                             stdout=log, stderr=subprocess.STDOUT)
        self.servers[name] = p
        return p

    def kill(self, name: str) -> float:
        """SIGKILL — 돌려주는 값은 벽시계 (사건 로그의 wall 과 같은 축)."""
        p = self.servers[name]
        t = time.time()
        os.kill(p.pid, signal.SIGKILL)
        p.wait(timeout=10)
        return t

    def endpoints(self):
        out = {}
        for topic in ('/fleet_p2p/vote', '/fleet_p2p/beacon', '/fleet_p2p/neg'):
            pubs = self.node.get_publishers_info_by_topic(topic)
            subs = self.node.get_subscriptions_info_by_topic(topic)
            out[topic] = {
                'agent_pub': sum(1 for e in pubs if e.node_name == 'fleet_agent'),
                'agent_sub': sum(1 for e in subs if e.node_name == 'fleet_agent'),
                'other_pub': sorted(f'{e.node_namespace}/{e.node_name}' for e in pubs
                                    if e.node_name != 'fleet_agent'),
                'other_sub': sorted(f'{e.node_namespace}/{e.node_name}' for e in subs
                                    if e.node_name != 'fleet_agent' and
                                    e.node_name != 'agent_launch_observer')}
        return out

    def close(self):
        for p in self.servers.values():
            if p.poll() is None:
                p.kill()
                p.wait(timeout=10)
        self.node.destroy_node()


def _zone_controller(obs, held_since, last_cmd):
    """구역 하나를 5 대가 번갈아: 잡은 로봇은 1 s 뒤 놓고, 아닌 로봇은 요청한다 (명령은 로봇마다 0.5 s 에 한 번)."""
    def step():
        now = time.monotonic()
        for r in ROBOTS:
            st = obs.state.get(r)
            if st is None or now - last_cmd.get(r, 0.0) < 0.5:
                continue
            holds = ZONE in st.held
            if holds:
                held_since.setdefault(r, now)
                if now - held_since[r] >= 1.0:
                    last_cmd[r] = now
                    obs.cmd[r].call_async(ZoneCommand.Request(op=ZoneCommand.Request.OP_RELEASE,
                                                              resources=[ZONE]))
            else:
                held_since.pop(r, None)
                if ZONE not in st.requested and st.mode != AgentState.MODE_ISOLATED:
                    last_cmd[r] = now
                    obs.cmd[r].call_async(ZoneCommand.Request(op=ZoneCommand.Request.OP_ACQUIRE,
                                                              resources=[ZONE], priority=0))
    return step


def _intervals(events, resource):
    """로봇별 허가 구간 [(t_start, t_end, robot)] — 'held' 사건의 sim 시각."""
    out, open_ = [], {}
    for e in sorted((e for e in events if e['event'] == 'held'), key=lambda e: e['t']):
        r = e['robot']
        if resource in e['added']:
            open_[r] = e['t']
        if resource in e['removed'] and r in open_:
            out.append((open_.pop(r), e['t'], r))
    for r, t in open_.items():
        out.append((t, float('inf'), r))
    return sorted(out)


def _overlaps(iv):
    """서로 다른 로봇의 구간 겹침 목록과, 연속한 두 허가 사이 최소 간격 [s] (음수면 겹침)."""
    bad, gaps = [], []
    for i, (s1, e1, r1) in enumerate(iv):
        for s2, e2, r2 in iv[i + 1:]:
            if r1 != r2 and s2 < e1:
                bad.append(((s1, e1, r1), (s2, e2, r2)))
        nxt = [s2 for s2, _, r2 in iv[i + 1:] if r2 != r1]
        if nxt and e1 != float('inf'):
            gaps.append(min(nxt) - e1)
    return bad, (min(gaps) if gaps else None)


class TestAgentFleet(unittest.TestCase):
    """①–④ 를 한 시나리오로."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def test_server_kill_modes_fence_and_task_lock(self, raw):
        obs = _Observer(raw)
        summary = {'seed': SEED, 'loss': LOSS, 'n_tasks': N_TASKS, 'raw': str(raw),
                   'load_start': os.getloadavg()}
        try:
            self._run(obs, summary)
        finally:
            summary['load_end'] = os.getloadavg()
            (raw / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1,
                                                         default=str), encoding='utf-8')
            obs.close()

    def _run(self, obs, summary):
        # --- 준비: 서버(epoch 3) → 5 대 NORMAL
        obs.server('server_a', '--epoch', '3')
        self.assertTrue(obs.spin_for(40.0, until=lambda: obs.all_mode(AgentState.MODE_NORMAL)
                                     and all(obs.state[r].fence_epoch == 3 for r in ROBOTS)),
                        f'NORMAL 이 안 됨: {obs.modes()}')

        # --- ① 구역 배타 + 서버 kill
        held_since, last_cmd = {}, {}
        ctl = _zone_controller(obs, held_since, last_cmd)
        obs.spin_for(20.0, step=ctl)
        ep_before = obs.endpoints()
        t_kill = obs.kill('server_a')
        m_kill = time.monotonic()
        summary['kill_wall'] = t_kill
        obs.spin_for(3.0, step=ctl)
        ep_after = obs.endpoints()
        beacons_after = collections.Counter(r for t, r in obs.beacons if t > m_kill + 0.5)
        obs.spin_for(25.0, step=ctl)
        # 구간 분석은 컨트롤러를 멈추고 진행 중 허가가 정리된 뒤에
        for r in ROBOTS:
            obs.cmd[r].call_async(ZoneCommand.Request(op=ZoneCommand.Request.OP_CANCEL))
        obs.spin_for(2.0)
        for r in ROBOTS:
            obs.cmd[r].call_async(ZoneCommand.Request(op=ZoneCommand.Request.OP_RELEASE,
                                                      resources=[ZONE]))
        obs.spin_for(2.0)
        evs = obs.file_events()
        iv = _intervals(evs, ZONE)
        bad, min_gap = _overlaps(iv)
        grants_after = sum(1 for e in evs if e['event'] == 'held' and ZONE in e['added']
                           and e['wall'] > t_kill)
        summary['①'] = {'endpoints_before': ep_before, 'endpoints_after_3s': ep_after,
                        'beacons_after_kill_by_robot': dict(beacons_after),
                        'intervals': len(iv), 'overlaps': bad, 'min_gap_s': min_gap,
                        'grants_after_kill': grants_after,
                        'grants_by_robot': dict(collections.Counter(r for _, _, r in iv))}
        for topic, ep in ep_after.items():
            self.assertEqual((ep['agent_pub'], ep['agent_sub']), (5, 5), (topic, ep_after))
        self.assertEqual(set(beacons_after), set(ROBOTS), beacons_after)
        self.assertEqual(bad, [])
        self.assertGreaterEqual(grants_after, 3)

        # --- ② DEGRADED 확인 → 관찰자 복귀 서버 → RESYNC → NORMAL
        self.assertTrue(obs.spin_for(20.0, until=lambda: obs.all_mode(AgentState.MODE_DEGRADED)),
                        f'DEGRADED 안 됨: {obs.modes()}')
        obs.server('server_b', '--observe-s', '2.0')
        self.assertTrue(obs.spin_for(30.0, until=lambda: obs.all_mode(AgentState.MODE_NORMAL)),
                        f'NORMAL 복귀 안 됨: {obs.modes()}')
        evs = obs.file_events()
        seqs = {r: [(round(e['t'], 2), e['previous'], e['mode'], e['reason']) for e in evs
                    if e['event'] == 'mode' and e['robot'] == r] for r in ROBOTS}
        t_kill_sim = {r: max((e['t'] for e in evs if e['robot'] == r and e['wall'] <= t_kill),
                             default=None) for r in ROBOTS}
        summary['②'] = {'transitions': seqs, 'last_event_sim_before_kill': t_kill_sim}
        want = ['NORMAL', 'HOLD_NEW', 'DEGRADED', 'RESYNC', 'NORMAL']
        for r, seq in seqs.items():
            path = ['NORMAL'] + [b for _, _, b, _ in seq]
            it = iter(path)
            self.assertTrue(all(w in it for w in want), (r, path))
        epoch_b = obs.state['amr_01'].fence_epoch
        summary['epoch_after_observer'] = epoch_b
        self.assertGreater(epoch_b, 3)

        # --- ③ 낮은 epoch 거부: 프록시 + 옛 권위자 하트비트
        leader = '/fleet/fleet_manager_node'
        res = {}
        # 서로 다른 거부 사례 셋: 낮은 epoch, 같은 epoch 의 작은 leader_id, 펜스가 생긴 뒤의 epoch 0 (옛 구성)
        # (리뷰 전의 'first_epoch'(3) 은 lower_epoch(epoch_b − 1 = 3) 과 같은 입력이었다)
        for name, ep, ld in (('current', epoch_b, leader), ('lower_epoch', epoch_b - 1, leader),
                             ('same_epoch_lower_leader', epoch_b, '/fleet/a_old_leader'),
                             ('unfenced_epoch0', 0, '')):
            res[name] = obs.assign_task('amr_02', f'fence_{name}', ep, ld)
        summary['③_proxy'] = res
        self.assertTrue(res['current']['success'] and res['current']['reached_executor'] == 1,
                        res)
        for k in ('lower_epoch', 'same_epoch_lower_leader', 'unfenced_epoch0'):
            self.assertFalse(res[k]['success'], res)
            self.assertEqual(res[k]['reached_executor'], 0, res)

        # --- ④b 중앙 프록시 경로의 작업 ID 잠금 (NORMAL, 서버 B 살아 있음)
        self._central_task_lock(obs, summary, epoch_b, leader)
        obs.server('stale', '--epoch', '2', '--no-manager', '--leader-id', leader)
        obs.spin_for(3.0)
        rej0 = {r: obs.state[r].low_epoch_rejected for r in ROBOTS}
        obs.kill('server_b')
        ok = obs.spin_for(20.0, until=lambda: obs.all_mode(AgentState.MODE_DEGRADED))
        rej1 = {r: obs.state[r].low_epoch_rejected for r in ROBOTS}
        summary['③_stale_heartbeat'] = {'stale_alive': obs.servers['stale'].poll() is None,
                                        'rejected_before_kill': rej0, 'rejected_after': rej1,
                                        'degraded_with_stale_alive': ok}
        self.assertTrue(ok, f'옛 권위자 하트비트로 DEGRADED 가 막힘: {obs.modes()}')
        self.assertTrue(all(rej1[r] > rej0[r] for r in ROBOTS), (rej0, rej1))
        obs.kill('stale')

        # --- ④a DEGRADED 협상 + 작업 ID 잠금: 픽업 시작 중복 0 (경합은 만들지 않는다 — 경합 수를 함께 적는다)
        t_post = time.time()
        for i in range(N_TASKS):
            t = Task()
            t.task_id = f'b4p3.{SEED:02d}{i:04d}'
            t.priority = i % 3
            t.status = Task.STATUS_PENDING
            t.pickup_pose.header.frame_id = 'map'
            t.pickup_pose.pose.position.x = float((i * 5) % 13)
            t.pickup_pose.pose.position.y = float((i * 3) % 4)
            obs.board.publish(t)
        done = set()

        def all_done():
            for e in obs.events:
                if e['event'] == 'done' and e['wall'] >= t_post:
                    done.add(e['task_id'])
            return len(done) >= N_TASKS

        finished = obs.spin_for(90.0, until=all_done)
        obs.spin_for(1.0)
        evs = [e for e in obs.file_events() if e['wall'] >= t_post]
        done = {e['task_id'] for e in evs if e['event'] == 'done'}
        starts = collections.defaultdict(list)
        for e in evs:
            if e['event'] == 'pickup_start':
                starts[e['task_id']].append(e['robot'])
        dups = {t: rs for t, rs in starts.items() if len(rs) > 1}
        failed = [e for e in evs if e['event'] == 'failed']
        granted = [(e['robot'], e['task_id']) for e in evs if e['event'] == 'task_granted']
        commits = {(e['robot'], e['task_id']) for e in evs if e['event'] == 'commit_request'}
        picked = {(e['robot'], e['task_id']) for e in evs if e['event'] == 'pickup_start'}
        summary['④'] = {'pickup_starts': sum(len(v) for v in starts.values()),
                        'tasks_started': len(starts), 'done': len(done),
                        'all_done': finished, 'duplicates': dups,
                        'failed': [(e['robot'], e['task_id'], e.get('reason')) for e in failed],
                        'starts': dict(starts),
                        # 경합 수 (리뷰: 이 단계는 경합 0 이었다 — 잠금 효과의 증거로 세지 않는다)
                        'contention': {'task_granted': len(granted),
                                       'granted_tasks_distinct': len({t for _, t in granted}),
                                       'commit_without_pickup': sorted(commits - picked),
                                       'failed': len(failed),
                                       'denied_final': {r: list(obs.state[r].denied)
                                                        for r in ROBOTS if r in obs.state}}}
        self.assertEqual(dups, {})
        self.assertGreaterEqual(len(starts), N_TASKS // 2, summary['④'])

    def _central_task_lock(self, obs, summary, epoch, leader):
        """
        ④b: 중앙 프록시 경로의 작업 ID 잠금을 ROS 로 (가짜 실행기는 응답만 — 확정은 ZoneCommand COMMIT_TASK).

        1) amr_04 의 실행기가 거부한 작업 T1 을 amr_02 에 다시 준다 → 수락 → COMMIT_TASK → HELD (리뷰 차단 1 의 회귀:
           고치기 전에는 amr_04 비컨의 current_task_id 가 단조 taken 에 남아 'task_taken' 으로 거부했다).
        2) 같은 작업 T2 를 amr_03 · amr_05 프록시에 주고 둘 다 COMMIT_TASK 를 되풀이 → 허가 구간 겹침 0, HELD 한 대,
           확정 거부(denied 또는 끝까지 HELD 아님) ≥ 1.
        """
        t1, t2 = f'b4p3.rej.{SEED:02d}', f'b4p3.race.{SEED:02d}'
        r1, r2 = f'task:{t1}', f'task:{t2}'
        out = {}
        obs.reject_exec = {'amr_04'}
        out['T1_amr_04'] = obs.assign_task('amr_04', t1, epoch, leader)
        obs.reject_exec = set()
        obs.spin_for(1.5)                      # 비컨이 돈다 (고치기 전이면 amr_04 비컨에 T1 이 실린다)
        out['T1_amr_02'] = obs.assign_task('amr_02', t1, epoch, leader)
        last = {}

        def committer(robots, task_id):
            def step():
                now = time.monotonic()
                for r in robots:
                    if now - last.get(r, 0.0) >= 0.5 and not obs.holds(r, f'task:{task_id}'):
                        last[r] = now
                        obs.cmd[r].call_async(ZoneCommand.Request(
                            op=ZoneCommand.Request.OP_COMMIT_TASK, resources=[task_id]))
            return step

        t0 = time.monotonic()
        ok1 = obs.spin_for(15.0, until=lambda: obs.holds('amr_02', r1),
                           step=committer(['amr_02'], t1))
        out['T1_amr_02_held_wall_s'] = round(time.monotonic() - t0, 2) if ok1 else None
        racers = ['amr_03', 'amr_05']
        out['T2'] = {r: obs.assign_task(r, t2, epoch, leader) for r in racers}
        last.clear()
        t0 = time.monotonic()

        def settled():
            held = [r for r in racers if obs.holds(r, r2)]
            return len(held) == 1 and all(obs.denied(r, r2) for r in racers if r not in held)

        ok2 = obs.spin_for(20.0, until=settled, step=committer(racers, t2))
        obs.spin_for(3.0, step=committer(racers, t2))   # 정한 뒤에도 진 쪽이 계속 확정을 원한다
        out['T2_settled_wall_s'] = round(time.monotonic() - t0, 2) if ok2 else None
        evs = obs.file_events()
        iv2 = _intervals(evs, r2)
        bad2, _ = _overlaps(iv2)
        holders2 = sorted({r for _, _, r in iv2})
        refused = [r for r in racers if r not in holders2]
        out.update({'T2_intervals': iv2, 'T2_overlaps': bad2, 'T2_holders': holders2,
                    'T2_refused': refused,
                    'T2_denied_final': {r: obs.denied(r, r2) for r in racers},
                    'T1_holders': sorted({r for _, _, r in _intervals(evs, r1)}),
                    'T1_amr_04_central_accept': sum(
                        1 for e in evs if e['robot'] == 'amr_04' and e['event'] == 'central_accept'
                        and e.get('task_id') == t1)})
        summary['④b'] = out
        self.assertFalse(out['T1_amr_04']['success'], out)
        self.assertEqual(out['T1_amr_04']['reached_executor'], 1, out)
        self.assertEqual(out['T1_amr_04_central_accept'], 0, out)
        self.assertTrue(out['T1_amr_02']['success'], out)
        self.assertTrue(ok1, out)
        self.assertEqual(out['T1_holders'], ['amr_02'], out)
        self.assertTrue(all(v['success'] for v in out['T2'].values()), out)
        self.assertEqual(bad2, [], out)
        self.assertEqual(len(holders2), 1, out)
        self.assertGreaterEqual(len(refused), 1, out)
