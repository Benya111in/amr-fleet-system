"""
launch 시험 W1 — 실제 task_executor_node 5 대로 P3 의 ①②③④b 를 다시 (보너스 4 b4-wire, Gazebo 없음).

구성 (harness.py): 가짜 /clock + 가짜 세계(직선 운동학 Nav2 대역 · 인식 · 실제 자세 기록) + 로봇마다 실제 실행기
(fleet.distributed, 분산 트리, assign_task → executor/assign_task) · 실제 에이전트(펜스 프록시 · 경로 게이트 · zone_map
해제) · 실제 어댑터, 서버 = 실제 fleet_manager_node + 하트비트 (manager_shim) 를 죽였다 살린다.
로봇↔로봇 링크 손실 AMR_WIRE_LOSS(0.2) · 지연 U(0, 100) ms, 서버→로봇 지연만.

판정 (사전 등록: docs/research/fleet-distributed/notebook/entries/*_b4-wire_wire-launch-tests.md):
① 고정 중앙 작업 4 개가 구역 x_bc_1 (· x_ab_1 사슬)을 서로 엇갈려 지나는 중 서버 SIGKILL: 실제 자세(가짜 세계) 원판
  기준 구역 동시 점유 0 · 에이전트가 믿은 허가 구간 겹침 0 (모든 구역) · kill 3 s 뒤 /fleet_p2p 끝점 5/5 · 작업 4/4 완료.
  공허 검사: 결정점에서 막힌 적(gate blocking) ≥ 1, kill 뒤 구역 허가 ≥ 1.
② 모드 NORMAL → HOLD_NEW → DEGRADED → (관찰자 복귀 서버) RESYNC → NORMAL 5/5, 관찰자 epoch > 3.
③ 낮은 (epoch, leader) 세 사례 프록시 거부 · 실제 실행기 도달 0 (실행기 로그에 그 작업이 없다). 현재 epoch 는 수락 · 완료.
  옛 권위자 하트비트가 살아 있어도 진짜 서버 kill 뒤 DEGRADED.
④b 실행기가 거부(busy)한 T1 을 다른 로봇에 → 그 로봇이 HELD 뒤 적재 · 완료 (거부한 로봇은 잠정 잠금 안 잡음).
   같은 T2 를 두 로봇 프록시에 (거의 같은 거리) → 적재 정확히 한 대, 다른 한 대는 task_lock_failed 로 실패 보고,
   task:T2 허가 구간 겹침 0.
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
POSES = [(-23.0, 0.5, 0.0), (-15.0, -9.0, 1.5708), (-7.0, -0.5, 3.1416),
         (-15.5, 11.5, -1.5708), (-23.0, -7.0, 0.0)]
SEED = int(os.environ.get('AMR_WIRE_SEED', '1'))
LOSS = float(os.environ.get('AMR_WIRE_LOSS', '0.2'))
LEADER = '/fleet/fleet_manager_node'
CROSSING = {          # ① 고정 중앙 작업: (픽업, 하역) — 가로 둘 · 세로 둘이 x_bc_1 을 두 번씩 지난다
    'amr_01': ((-8.0, 0.5, 0.0), (-22.0, 0.5, 3.1416)),
    'amr_02': ((-15.0, 11.0, 1.5708), (-15.0, -8.5, -1.5708)),
    'amr_03': ((-22.0, -0.5, 3.1416), (-8.0, -0.5, 0.0)),
    'amr_04': ((-15.5, -8.5, -1.5708), (-15.5, 11.0, 1.5708)),
}


@pytest.mark.launch_test
def generate_test_description():
    raw = H.raw_dir('AMR_WIRE_RAW', 'wire_launch_')
    comm = H.comm_yaml(raw, SEED, LOSS)
    procs = H.processes(raw, ROBOTS, POSES, comm, SEED)
    return launch.LaunchDescription(procs + [launch_testing.actions.ReadyToTest()]), {'raw': raw}


class TestWire(unittest.TestCase):
    """①②③④b 를 한 시나리오로."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def test_wired_fleet_with_real_executors(self, raw):
        obs = H.Observer(raw, ROBOTS)
        summary = {'seed': SEED, 'loss': LOSS, 'raw': str(raw), 'load_start': os.getloadavg()}
        try:
            self._run(obs, raw, summary)
        finally:
            summary['load_end'] = os.getloadavg()
            (raw / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1,
                                                         default=str), encoding='utf-8')
            obs.close()

    def _run(self, obs, raw, summary):
        obs.server('server_a', '--epoch', '3')
        self.assertTrue(obs.spin_for(60.0, until=lambda: obs.all_mode(AgentState.MODE_NORMAL)
                                     and all(obs.state[r].fence_epoch == 3 for r in ROBOTS)),
                        f'NORMAL 이 안 됨: {obs.modes()}')
        obs.spin_for(5.0)                       # 관리자가 robot_state 로 로봇을 찾는다
        summary['duplicate_nodes'] = obs.duplicates()
        self.assertEqual(summary['duplicate_nodes'], {}, '격리 실패 — 같은 이름 노드가 둘 이상')

        # --- ① 엇갈려 지나는 중앙 작업 + 서버 kill
        for r, (pick, drop) in CROSSING.items():
            obs.submit(f'w1.{r}', r, pick, drop)
        started = obs.spin_for(30.0, until=lambda: all(
            (r, Task.STATUS_IN_PROGRESS) in obs.statuses(f'w1.{r}') for r in CROSSING))
        self.assertTrue(started, {r: obs.statuses(f'w1.{r}') for r in CROSSING})
        obs.spin_for(18.0)
        t_kill = obs.kill('server_a')
        obs.spin_for(3.0)
        ep_after = obs.endpoints()
        done = obs.spin_for(150.0, until=lambda: all(obs.terminal(f'w1.{r}') for r in CROSSING))
        evs = H.file_events(raw)
        polys = H.zone_polygons()
        believed = {z: H.overlaps(H.intervals(evs, f'zone:{z}')) for z in polys}
        grants = collections.Counter(z for e in evs if e['event'] == 'held'
                                     for z in e['added'] if z.startswith('zone:'))
        grants_after = sum(1 for e in evs if e['event'] == 'held' and e['wall'] > t_kill
                           for z in e['added'] if z.startswith('zone:'))
        blocked = collections.Counter(e['robot'] for e in evs if e['event'] == 'gate'
                                      and e['blocking'])
        disc, center, occ = H.true_cooccupancy(raw, polys, ROBOTS)
        summary['①'] = {
            'endpoints_after_3s': ep_after, 'all_terminal': done,
            'terminal': {r: obs.terminal(f'w1.{r}') for r in CROSSING},
            'believed_overlaps': {z: v for z, v in believed.items() if v},
            'zone_grants': dict(grants), 'zone_grants_after_kill': grants_after,
            'gate_blocking_events_by_robot': dict(blocked),
            'true_cooccupancy_disc': disc, 'true_cooccupancy_center': center,
            'zone_occupancy_samples': occ,
            'exec_logs': {r: H.exec_log(raw, r) for r in ROBOTS}}
        for topic, ep in ep_after.items():
            self.assertEqual(ep, (5, 5), (topic, ep_after))
        self.assertTrue(done, summary['①']['terminal'])
        for r in CROSSING:
            self.assertEqual(obs.terminal(f'w1.{r}'), [(r, Task.STATUS_COMPLETED)], r)
        self.assertEqual({z: v for z, v in believed.items() if v}, {})
        self.assertEqual(disc, [])
        self.assertGreaterEqual(sum(blocked.values()), 1, '경합이 없었다 (공허)')
        self.assertGreaterEqual(grants_after, 1, 'kill 뒤 구역 허가가 없었다 (공허)')

        # --- ② DEGRADED → 관찰자 복귀 서버 → RESYNC → NORMAL
        self.assertTrue(obs.spin_for(30.0, until=lambda: obs.all_mode(AgentState.MODE_DEGRADED)),
                        f'DEGRADED 안 됨: {obs.modes()}')
        obs.server('server_b', '--observe-s', '2.0')
        self.assertTrue(obs.spin_for(45.0, until=lambda: obs.all_mode(AgentState.MODE_NORMAL)),
                        f'NORMAL 복귀 안 됨: {obs.modes()}')
        evs = H.file_events(raw)
        seqs = {r: [b for e in evs if e['event'] == 'mode' and e['robot'] == r
                    for b in [e['mode']]] for r in ROBOTS}
        epoch_b = obs.state['amr_01'].fence_epoch
        summary['②'] = {'transitions': seqs, 'epoch_after_observer': epoch_b}
        want = ['NORMAL', 'HOLD_NEW', 'DEGRADED', 'RESYNC', 'NORMAL']
        for r, seq in seqs.items():
            it = iter(['NORMAL'] + seq)
            self.assertTrue(all(w in it for w in want), (r, seq))
        self.assertGreater(epoch_b, 3)

        # --- ③ 펜스: 낮은 명령은 실제 실행기까지 가지 않는다
        safe = ((-23.0, -10.0, -1.5708), (-25.0, -7.0, 3.1416))   # amr_05 근처, 구역 없음
        res = {}
        for name, ep, ld in (('lower_epoch', epoch_b - 1, LEADER),
                             ('same_epoch_lower_leader', epoch_b, '/fleet/a_old_leader'),
                             ('unfenced_epoch0', 0, ''), ('current', epoch_b, LEADER)):
            res[name] = obs.proxy_assign('amr_05', f'w3.{name}', ep, ld, *safe)
        obs.spin_for(2.0)
        # 실행기 도달 = 실행기 로그의 수락 · 거절 줄 또는 그 작업의 task_status (현재 epoch 는 있어야 한다 — 공허 검사)
        reached = {name: [k for k, t, _ in H.exec_log(raw, 'amr_05') if t == f'w3.{name}']
                   + [s for _, s in obs.statuses(f'w3.{name}')] for name in res}
        done3 = obs.spin_for(60.0, until=lambda: obs.terminal('w3.current'))
        summary['③_proxy'] = {'responses': res, 'executor_log': reached,
                              'current_terminal': obs.terminal('w3.current')}
        self.assertEqual(res['current'][0], True, res)
        self.assertTrue(reached['current'], reached)
        self.assertTrue(done3 and obs.terminal('w3.current') == [
            ('amr_05', Task.STATUS_COMPLETED)], summary['③_proxy'])
        for k in ('lower_epoch', 'same_epoch_lower_leader', 'unfenced_epoch0'):
            self.assertEqual(res[k][0], False, res)
            self.assertEqual(reached[k], [], reached)

        # --- ④b 중앙 프록시 경로의 작업 ID 잠금 (실제 실행기 · 실제 BT PickupLocked)
        self._central_lock(obs, raw, summary, epoch_b)

        # --- ③ 옛 권위자 하트비트 + 진짜 서버 kill → DEGRADED
        obs.server('stale', '--epoch', str(epoch_b - 1), '--no-manager', '--leader-id', LEADER)
        obs.spin_for(3.0)
        rej0 = {r: obs.state[r].low_epoch_rejected for r in ROBOTS}
        obs.kill('server_b')
        ok = obs.spin_for(30.0, until=lambda: obs.all_mode(AgentState.MODE_DEGRADED))
        rej1 = {r: obs.state[r].low_epoch_rejected for r in ROBOTS}
        summary['③_stale_heartbeat'] = {'stale_alive': obs.servers['stale'].poll() is None,
                                        'rejected_before_kill': rej0, 'rejected_after': rej1,
                                        'degraded_with_stale_alive': ok}
        self.assertTrue(ok, f'옛 권위자 하트비트로 DEGRADED 가 막힘: {obs.modes()}')
        self.assertTrue(all(rej1[r] > rej0[r] for r in ROBOTS), (rej0, rej1))
        obs.kill('stale')

    def _central_lock(self, obs, raw, summary, epoch):
        out = {}
        # T1: amr_04 를 바쁘게 한 뒤 T1 → amr_04 (실행기가 busy 로 거부) → amr_02 (수락 · 확정 · 적재)
        busy = ((-15.5, -8.5, -1.5708), (-15.5, 11.0, 1.5708))
        out['busy'] = obs.proxy_assign('amr_04', 'w4.busy', epoch, LEADER, *busy)
        obs.spin_for(10.0, until=lambda: ('amr_04', Task.STATUS_IN_PROGRESS) in obs.statuses(
            'w4.busy'))
        t1 = ((-20.0, -8.5, 3.1416), (-20.0, -5.5, 1.5708))
        out['T1_amr_04'] = obs.proxy_assign('amr_04', 'w4.T1', epoch, LEADER, *t1)
        obs.spin_for(1.5)
        out['T1_amr_02'] = obs.proxy_assign('amr_02', 'w4.T1', epoch, LEADER, *t1)
        # T2: 같은 작업을 거의 같은 거리의 amr_01 · amr_03 에 (둘 다 수락 → 픽업 직전 확정 경합)
        t2 = ((-15.0, -5.0, -1.5708), (-19.0, -5.0, 3.1416))
        out['T2'] = {r: obs.proxy_assign(r, 'w4.T2', epoch, LEADER, *t2)
                     for r in ('amr_01', 'amr_03')}
        fin = obs.spin_for(120.0, until=lambda: obs.terminal('w4.T1') and len(
            obs.terminal('w4.T2')) >= 2 and obs.terminal('w4.busy'))
        obs.spin_for(2.0)
        evs = H.file_events(raw)
        loads = H.loads(obs, raw)
        logs = {r: H.exec_log(raw, r) for r in ROBOTS}
        iv2 = H.intervals(evs, 'task:w4.T2')
        out.update({
            'finished': fin,
            'terminal': {t: obs.terminal(t) for t in ('w4.busy', 'w4.T1', 'w4.T2')},
            'loads': [x for x in loads if x[1].startswith('w4.')],
            'T1_amr_04_central_accept': sum(
                1 for e in evs if e['robot'] == 'amr_04' and e['event'] == 'central_accept'
                and e.get('task_id') == 'w4.T1'),
            'T2_intervals': iv2, 'T2_overlaps': H.overlaps(iv2),
            'T2_failed_logs': {r: [x for x in logs[r] if x[1] == 'w4.T2' and x[0] == 'failed']
                               for r in ('amr_01', 'amr_03')}})
        summary['④b'] = out
        self.assertEqual(out['T1_amr_04'][0], False, out)
        self.assertIn('busy', out['T1_amr_04'][1] or '', out)
        self.assertEqual(out['T1_amr_04_central_accept'], 0, out)
        self.assertEqual(out['T1_amr_02'][0], True, out)
        self.assertTrue(fin, out['terminal'])
        self.assertEqual(obs.terminal('w4.T1'), [('amr_02', Task.STATUS_COMPLETED)], out)
        t1_loads = [r for r, t, _ in loads if t == 'w4.T1']
        t2_loads = [r for r, t, _ in loads if t == 'w4.T2']
        self.assertEqual(t1_loads, ['amr_02'], loads)
        self.assertTrue(all(v[0] for v in out['T2'].values()), out)
        self.assertEqual(len(t2_loads), 1, loads)             # 적재는 정확히 한 대
        loser = ({'amr_01', 'amr_03'} - set(t2_loads)).pop()
        self.assertEqual([x[2] for x in out['T2_failed_logs'][loser]], ['task_lock_failed'], out)
        self.assertEqual(out['T2_overlaps'], [], out)
