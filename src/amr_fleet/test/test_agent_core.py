"""
에이전트 코어 통합 시험 — ROS 없이 5 대를 한 프로세스에서 (agent/loopback.py).

launch 시험(test_agent_launch.py)과 같은 네 성질을 결정적으로, 손실 · 시드를 바꿔 가며 본다.
① 서버가 죽어도 구역 배타 유지 ② 모드 전환 ③ 낮은 epoch 거부 ④ 협상 할당에서 픽업 시작 중복 0.
"""

import collections
import os

import pytest

from amr_fleet.agent.core import (
    BoardTask, CoreConfig, OP_ACQUIRE, OP_RELEASE,
)
from amr_fleet.agent.loopback import LoopbackFleet
from amr_fleet.agent.modes import DEGRADED, HOLD_NEW, ISOLATED, ModeConfig, NORMAL, RESYNC
from amr_fleet.agent.sim_exec import SimExecConfig
from amr_fleet.comm_emulator import CommEmulator, LinkModel, Outage
from amr_fleet.distributed.vote_core import VoteConfig

ROBOTS = ('amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05')
POSES = {'amr_01': (0.0, 0.0), 'amr_02': (5.0, 0.0), 'amr_03': (10.0, 0.0),
         'amr_04': (0.0, 5.0), 'amr_05': (5.0, 5.0)}
VOTE = VoteConfig(ROBOTS, tentative_s=10.0, retry_s=0.5, backoff_min_s=0.3, backoff_max_s=1.5)


def _cfgs(**kw):
    base = dict(members=ROBOTS, zone_vote=VOTE, task_vote=VOTE,
                modes=ModeConfig(resync_s=3.0, isolate_s=1.0, boot_grace_s=2.0))
    base.update(kw)
    return [CoreConfig(robot_id=r, **base) for r in ROBOTS]


def _lossy(seed, loss=0.2):
    return CommEmulator(seed=seed, default=LinkModel(0.0, 100.0, loss))


def _transitions(fleet):
    return {r: [(round(t, 2), a, b) for t, a, b, _ in c.modes.transitions]
            for r, c in fleet.cores.items()}


# ---------------------------------------------------------------------- ② 모드 · ③ epoch
def test_server_loss_then_return_walks_all_modes():
    f = LoopbackFleet(_cfgs(), poses=POSES)
    f.run(3.0)
    assert set(f.modes().values()) == {NORMAL}
    t_kill = f.t
    f.server().alive = False
    f.run(9.0)
    assert set(f.modes().values()) == {DEGRADED}
    s = f.server()
    s.alive, s.epoch, s.seq = True, 2, 0          # 재기동한 서버는 더 높은 epoch
    f.run(1.0)
    assert set(f.modes().values()) == {RESYNC}
    f.run(3.5)
    assert set(f.modes().values()) == {NORMAL}
    for r, seq in _transitions(f).items():
        assert [(a, b) for _, a, b in seq] == [(NORMAL, HOLD_NEW), (HOLD_NEW, DEGRADED),
                                               (DEGRADED, RESYNC), (RESYNC, NORMAL)], r
        t_hold, t_deg = seq[0][0], seq[1][0]
        # 마지막 하트비트(≤ t_kill) 뒤 suspect 1.5 s, dead 5 s (+ 감시 주기 · 비컨 전파)
        assert 1.0 <= t_hold - t_kill <= 2.2, (r, t_hold - t_kill)
        assert 4.5 <= t_deg - t_kill <= 7.5, (r, t_deg - t_kill)


def test_low_epoch_heartbeat_and_command_are_rejected():
    f = LoopbackFleet(_cfgs(), poses=POSES)
    f.server().epoch = 3
    f.run(2.0)
    stale = f.add_server(epoch=2)                 # 옛 권위자 (예: SIGSTOP 에서 깨어난 서버)
    f.run(2.0)
    assert all(c.low_epoch_rejected > 0 for c in f.cores.values())
    f.server().alive = False                       # 진짜 권위자만 죽인다
    f.run(9.0)
    # 낮은 epoch 하트비트는 생존 신호가 아니다 → 옛 서버가 살아 있어도 DEGRADED
    assert set(f.modes().values()) == {DEGRADED}
    assert stale.alive
    c = f.cores['amr_01']
    ok, why = c.check_assignment('t_low', 2, stale.leader_id, f.t)
    assert not ok and why.startswith('stale_epoch')


def test_assignment_fence_accepts_only_in_normal_and_not_below_max():
    f = LoopbackFleet(_cfgs(), poses=POSES)
    f.server().epoch = 5
    f.run(2.0)
    c = f.cores['amr_02']
    assert c.check_assignment('t1', 5, '/fleet/fleet_manager_node', f.t)[0]
    n0 = c.low_epoch_rejected
    assert not c.check_assignment('t2', 4, '/fleet/fleet_manager_node', f.t)[0]
    assert not c.check_assignment('t3', 5, '/fleet/a_lower_name', f.t)[0]   # 같은 epoch, 작은 id
    assert c.low_epoch_rejected == n0 + 2
    assert c.check_assignment('t4', 6, '/fleet/new_leader', f.t)[0]          # 더 높으면 채택
    assert not c.check_assignment('t5', 5, '/fleet/fleet_manager_node', f.t)[0]
    f.server().alive = False
    f.run(9.0)
    ok, why = c.check_assignment('t6', 7, '/fleet/x', f.t)
    assert not ok and why == 'mode DEGRADED'


# ---------------------------------------------------------------------- ① 구역 배타
@pytest.mark.parametrize('seed', [1, 2, 3])
def test_zone_exclusive_under_loss_and_server_kill(seed):
    f = LoopbackFleet(_cfgs(), comm=_lossy(seed), poses=POSES)
    zone = 'zone:x_ab_1'
    held_since = {}
    grants = collections.Counter()
    worst = [0]

    def hook(fl):
        hs = fl.holders(zone)
        worst[0] = max(worst[0], len(hs))
        for r in ROBOTS:
            if r in hs:
                if r not in held_since:
                    held_since[r] = fl.t
                    grants[r] += 1
                elif fl.t - held_since[r] >= 1.0:
                    fl.command(r, OP_RELEASE, [zone])
            elif r in held_since:
                del held_since[r]
            core = fl.cores[r]
            if r not in hs and not core.zone.requested() and fl.t >= 2.0:
                fl.command(r, OP_ACQUIRE, [zone], priority=0)

    f.run(20.0, hook=hook)
    before_kill = sum(grants.values())
    f.server().alive = False
    f.run(30.0, hook=hook)
    assert worst[0] <= 1
    assert before_kill >= 3
    assert sum(grants.values()) - before_kill >= 5          # 서버가 죽은 뒤에도 허가가 이어진다
    assert len([r for r in ROBOTS if grants[r]]) >= 3        # 한 로봇만 독식하지 않는다 (진단)


# ---------------------------------------------------------------------- ④ 협상 + 작업 잠금
N_TASK_SEEDS = int(os.environ.get('AMR_AGENT_NEGO_SEEDS', '3'))


def _degraded_run(seed, loss, n_tasks=8, seconds=90.0, crash=None):
    cfgs = _cfgs(sim_exec=SimExecConfig(pickup_s=1.0, deliver_s=1.0, commit_timeout_s=20.0),
                 task_hold_s=20.0)
    f = LoopbackFleet(cfgs, comm=_lossy(seed, loss), poses=POSES)
    f.server().alive = False                     # 서버 없음 — 부팅 뒤 unknown_dead_s 지나 DEGRADED
    tasks = [BoardTask(f't{i:02d}', priority=i % 3, pickup_x=float((i * 7) % 15),
                       pickup_y=float((i * 3) % 7)) for i in range(n_tasks)]
    f.post(tasks)
    crashed = []

    def hook(fl):
        if crash and not crashed and fl.t >= crash[1]:
            fl.crash(crash[0])
            crashed.append(fl.t)
        if crashed and fl.cores.get(crash[0]) is None and fl.t >= crashed[0] + crash[2]:
            fl.boot(crash[0])
            fl.post(tasks)

    f.run(seconds, hook=hook)
    starts = collections.defaultdict(list)
    done = set()
    for ev in f.events:
        if ev['event'] == 'pickup_start':
            starts[ev['task_id']].append(ev['robot'])
        elif ev['event'] == 'done':
            done.add(ev['task_id'])
    return f, starts, done


@pytest.mark.parametrize('seed', range(1, N_TASK_SEEDS + 1))
@pytest.mark.parametrize('loss', [0.0, 0.2])
def test_degraded_negotiation_never_double_starts_a_pickup(seed, loss):
    f, starts, done = _degraded_run(seed, loss)
    assert set(f.modes().values()) == {DEGRADED}
    dups = {t: rs for t, rs in starts.items() if len(rs) > 1}
    assert dups == {}
    # 공허 검사 · 활성(진단): 작업이 실제로 픽업 · 완료된다
    assert len(starts) >= 6, dict(starts)
    assert len(done) >= 6


def test_crash_after_pickup_keeps_task_lock_across_restart():
    f, starts, _ = _degraded_run(seed=7, loss=0.1, crash=('amr_01', 25.0, 3.0), seconds=80.0)
    dups = {t: rs for t, rs in starts.items() if len(rs) > 1}
    assert dups == {}
    c = f.cores['amr_01']
    for t in [t for t, rs in starts.items() if rs == ['amr_01']]:
        assert c.task.may_enter(f'task:{t}')          # 재기동 뒤에도 확정 잠금을 놓지 않는다


# ---------------------------------------------------------------------- ISOLATED
def test_isolated_robot_refuses_new_entry_and_others_continue():
    comm = CommEmulator(seed=3, default=LinkModel(0.0, 50.0, 0.0),
                        outages=[Outage(5.0, 12.0, 'amr_05', '*'),
                                 Outage(5.0, 12.0, '*', 'amr_05')])
    f = LoopbackFleet(_cfgs(), comm=comm, poses=POSES)
    f.run(8.0)
    m = f.modes()
    assert m['amr_05'] == ISOLATED
    assert all(m[r] == NORMAL for r in ROBOTS if r != 'amr_05')
    ok, why = f.command('amr_05', OP_ACQUIRE, ['zone:x_ab_2'])
    assert not ok and 'ISOLATED' in why
    assert f.command('amr_01', OP_ACQUIRE, ['zone:x_ab_2'])[0]
    f.run(2.0)
    assert f.holders('zone:x_ab_2') == ['amr_01']           # 4 대로 과반 3 을 모은다
    f.run(6.0)
    assert f.modes()['amr_05'] == NORMAL


def test_partition_minority_negotiates_but_cannot_start_pickups():
    """
    협상 도중 {amr_01, amr_02} 대 나머지 셋으로 갈라진다: 두 쪽 CBAA 가 같은 작업을 따로 고를 수 있다.

    잠금이 없으면(낙관 모드) 두 쪽이 같은 작업을 픽업한다 — 작업 ID 잠금의 정족수 3 이 소수 쪽을 막아야 한다.
    """
    groups = (frozenset({'amr_01', 'amr_02'}), frozenset({'amr_03', 'amr_04', 'amr_05'}))
    from amr_fleet.comm_emulator import Partition
    comm = CommEmulator(seed=5, default=LinkModel(0.0, 50.0, 0.0),
                        partitions=[Partition(7.0, 40.0, groups)])
    cfgs = _cfgs(sim_exec=SimExecConfig(pickup_s=1.0, deliver_s=1.0, commit_timeout_s=60.0),
                 task_hold_s=20.0)
    f = LoopbackFleet(cfgs, comm=comm, poses=POSES)
    f.server().alive = False
    tasks = [BoardTask(f't{i:02d}', pickup_x=float(i), pickup_y=0.0) for i in range(6)]
    f.post(tasks)
    f.run(80.0)
    starts = collections.defaultdict(list)
    for ev in f.events:
        if ev['event'] == 'pickup_start':
            starts[ev['task_id']].append((ev['robot'], round(ev['t'], 1)))
    assert {t: rs for t, rs in starts.items() if len(rs) > 1} == {}
    minority_started_in_partition = [
        (t, r, ts) for t, rs in starts.items() for r, ts in rs
        if r in groups[0] and ts < 40.0]
    assert minority_started_in_partition == []
    # 공허 검사: 소수 쪽도 작업 잠금을 청했다 (협상은 했지만 과반을 못 모았다)
    assert all(f.cores[r].lock.n_requests > 0 for r in groups[0])
    assert len(starts) == len(tasks)                          # 분할이 풀린 뒤 모두 끝난다 (진단)


def test_observer_heartbeat_does_not_raise_the_fence():
    """관찰자 하트비트는 펜스를 올리지 않는다 (올리면 관찰자 epoch 가 되먹임으로 계속 오른다)."""
    from amr_fleet.agent.core import ROLE_OBSERVER
    f = LoopbackFleet(_cfgs(), poses=POSES)
    f.run(2.0)
    s = f.server()
    s.alive = False
    f.run(9.0)
    assert set(f.modes().values()) == {DEGRADED}
    s.alive, s.role, s.epoch, s.seq = True, ROLE_OBSERVER, 2, 0
    f.run(5.0)
    assert set(f.modes().values()) == {RESYNC}               # 관찰 중: 정상으로 돌아가지 않는다
    assert {c.fence.max[0] for c in f.cores.values()} == {1}  # 펜스는 그대로
    s.role = 0                                   # 관찰 끝 → ACTIVE (같은 epoch 2, seq 는 이어서)
    f.run(4.0)
    assert set(f.modes().values()) == {NORMAL}
    assert {c.fence.max[0] for c in f.cores.values()} == {2}


# ---------------------------------------------------------------------- 해제 게이트 · 재기동
SQUARE = {'zone:sq': [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]}


def _gate_fleet():
    cfgs = _cfgs(release_gate='zone_map')
    f = LoopbackFleet(cfgs, poses=POSES)
    for r in ROBOTS:
        f.cores[r].zone_polygons = dict(SQUARE)
    return f


def test_zone_map_gate_releases_only_when_body_is_outside_and_localized():
    f = _gate_fleet()
    f.run(1.0)
    c = f.cores['amr_01']
    c.set_pose(1.0, 1.0, 0.0)                         # 구역 안
    assert f.command('amr_01', OP_ACQUIRE, ['zone:sq'])[0]
    f.run(1.0)
    assert f.holders('zone:sq') == ['amr_01']
    f.command('amr_01', OP_RELEASE, ['zone:sq'])
    f.run(1.0)
    assert f.holders('zone:sq') == ['amr_01']         # 몸체가 안이라 RELEASE 보류
    c.set_pose(2.3, 1.0, 0.0)                         # 변에서 0.3 m < 반경 0.36 + 여유 0.1
    f.run(1.0)
    assert f.holders('zone:sq') == ['amr_01']
    c.set_pose(3.0, 1.0, 0.0, healthy=False)          # 밖이지만 위치추정 상실 → 판정 불가
    f.run(1.0)
    assert f.holders('zone:sq') == ['amr_01']
    c.set_pose(3.0, 1.0, 0.0, healthy=True)
    f.run(1.0)
    assert f.holders('zone:sq') == []


def test_restart_with_unknown_pose_keeps_zone_until_body_is_seen_outside():
    f = _gate_fleet()
    f.run(1.0)
    f.cores['amr_02'].set_pose(1.0, 1.0, 0.0)
    f.command('amr_02', OP_ACQUIRE, ['zone:sq'])
    f.run(1.0)
    assert f.holders('zone:sq') == ['amr_02']
    f.crash('amr_02')
    f.poses.pop('amr_02')                             # 재기동 직후 자세를 모른다
    core = f.boot('amr_02')
    core.zone_polygons = dict(SQUARE)
    f.run(2.0)
    assert f.holders('zone:sq') == ['amr_02']         # 모르면 놓지 않는다 (fail-closed)
    core.set_pose(1.5, 1.5, 0.0)
    f.run(1.0)
    assert f.holders('zone:sq') == ['amr_02']         # 안에 있으면 그대로
    core.set_pose(5.0, 5.0, 0.0)
    f.run(1.0)
    assert f.holders('zone:sq') == []                 # 밖을 확인하고 푼다
    assert f.command('amr_03', OP_ACQUIRE, ['zone:sq'])[0]
    f.run(1.0)
    assert f.holders('zone:sq') == ['amr_03']


def test_disc_clear_of_polygon_geometry():
    from amr_fleet.agent.core import disc_clear_of_polygon
    sq = SQUARE['zone:sq']
    assert not disc_clear_of_polygon(1.0, 1.0, 0.1, sq)
    assert not disc_clear_of_polygon(2.3, 1.0, 0.36, sq)
    assert disc_clear_of_polygon(2.5, 1.0, 0.36, sq)
    assert not disc_clear_of_polygon(2.2, 2.2, 0.36, sq)      # 꼭짓점 근처
    assert disc_clear_of_polygon(2.3, 2.3, 0.36, sq)
