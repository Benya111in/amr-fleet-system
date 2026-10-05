"""장애 감지기 시험 — sim-time 하트비트, SUSPECT/DEAD, 자기 지연 점검, 과반 의심, epoch 펜스."""

import pytest

from amr_fleet.distributed.failure_detector import (
    ALIVE, DEAD, DetectorConfig, EpochFence, HeartbeatMonitor, LocalHealth, MajoritySuspicion,
    SUSPECT, UNKNOWN,
)
from amr_fleet.distributed.store import MemoryStore

CFG = DetectorConfig()
MEMBERS = ('amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05')


def test_config_validation():
    with pytest.raises(ValueError):
        DetectorConfig(suspect_s=6.0, dead_s=5.0)
    with pytest.raises(ValueError):
        DetectorConfig(late_factor=1.0)


def test_suspect_then_dead_on_sim_time():
    hb = HeartbeatMonitor(CFG)
    assert hb.status(0.0) == UNKNOWN
    t = 0.0
    for seq in range(10):
        hb.observe(t, epoch=1, seq=seq)
        assert hb.status(t + 0.4) == ALIVE
        t += 0.5
    last = t - 0.5
    assert hb.status(last + 1.49) == ALIVE
    assert hb.status(last + 1.5) == SUSPECT          # 2 Hz 3 회 누락
    assert hb.status(last + 4.99) == SUSPECT
    assert hb.status(last + 5.0) == DEAD
    hb.observe(last + 6.0, epoch=1, seq=10)          # 돌아오면 되돌릴 수 있다
    assert hb.status(last + 6.1) == ALIVE


def test_only_sim_time_matters():
    """RTF 0.25: sim 0.5 s 마다 하트비트 = 벽시계 2 s 마다. 감지기는 sim 시각만 보므로 ALIVE."""
    hb = HeartbeatMonitor(CFG)
    for k in range(40):
        t_sim = 0.5 * k
        hb.observe(t_sim, 1, k)
        assert hb.status(t_sim + 0.45) == ALIVE


def test_stale_heartbeats_do_not_refresh():
    hb = HeartbeatMonitor(CFG)
    assert hb.observe(0.0, epoch=2, seq=5)
    assert not hb.observe(3.0, epoch=1, seq=99)      # 낮은 epoch (옛 서버)
    assert not hb.observe(3.0, epoch=2, seq=5)       # 중복
    assert not hb.observe(3.0, epoch=2, seq=4)       # 재정렬된 옛 것
    assert hb.ignored == 3
    assert hb.status(5.0) == DEAD
    assert hb.observe(5.5, epoch=3, seq=0)           # 새 epoch 의 서버는 seq 가 다시 0 이어도 받는다
    assert hb.status(5.6) == ALIVE


def test_time_jump_resets_baseline():
    hb = HeartbeatMonitor(CFG)
    hb.observe(100.0, 1, 1)
    assert hb.status(2.0) == ALIVE and hb.time_jumps == 1
    assert hb.status(7.0) == DEAD


def test_local_health_extends_timeouts_when_self_is_slow():
    health = LocalHealth(CFG)
    hb = HeartbeatMonitor(CFG, health)
    hb.observe(0.0, 1, 1)
    t = 0.0
    health.observe_tick(t)
    for _ in range(3):                    # 자기 감시가 1 s 간격으로만 돈다 (기대 0.1 s)
        t += 1.0
        health.observe_tick(t)
    assert health.score == 3 and health.multiplier() == 4.0
    assert hb.status(3.0) == ALIVE        # 1.5 × 4 = 6 s 전에는 의심하지 않는다
    for _ in range(40):                   # 다시 제때 돌면 점수가 내려간다
        t += 0.1
        health.observe_tick(t)
    assert health.score == 0


def test_no_dead_verdict_right_after_own_pause():
    health = LocalHealth(CFG)
    hb = HeartbeatMonitor(CFG, health)
    hb.observe(0.0, 1, 1)
    health.observe_tick(0.0)
    health.observe_tick(30.0)             # 자기 프로세스가 30 s 멈췄다 (kill -STOP)
    assert health.late
    assert hb.status(30.0) == SUSPECT     # 큐의 하트비트를 아직 못 읽었을 수 있다 — DEAD 보류
    for k in range(1, 200):               # 자기는 다시 제때 도는데 서버는 계속 조용
        health.observe_tick(30.0 + 0.1 * k)
    assert hb.status(50.0) == DEAD


def test_majority_suspicion_needs_three_of_five_fresh():
    ms = MajoritySuspicion(MEMBERS, beacon_ttl_s=2.0)
    assert ms.quorum == 3
    ms.observe('amr_01', 10.0, True, seq=1)
    ms.observe('amr_02', 10.0, True, seq=1)
    ms.observe('amr_03', 10.0, False, seq=1)
    assert not ms.degraded(10.5)          # 2/5 — 두 로봇의 링크 문제일 수 있다
    ms.observe('amr_04', 10.2, True, seq=1)
    assert ms.degraded(10.5)
    assert not ms.degraded(12.1)          # 01·02 비컨이 묵음 → 신선한 동의 1 (04)
    assert not ms.observe('amr_04', 12.0, False, seq=0)    # 옛 비컨은 무시
    assert not ms.observe('ghost', 12.0, True)
    with pytest.raises(ValueError):
        MajoritySuspicion(MEMBERS, quorum=2)


def test_connected_majority_for_isolation():
    ms = MajoritySuspicion(MEMBERS, beacon_ttl_s=2.0)
    ms.observe('amr_01', 0.0, False, seq=1)
    ms.observe('amr_02', 0.0, False, seq=1)
    assert not ms.connected_majority(1.0)
    ms.observe('amr_03', 0.5, False, seq=1)
    assert ms.connected_majority(1.0)
    assert not ms.connected_majority(2.6)


def test_epoch_fence_rejects_lower_and_persists():
    st = MemoryStore()
    f = EpochFence(st)
    assert f.accept(3, 'fm_a')
    assert not f.accept(2, 'fm_b')               # 낮은 epoch (복귀한 옛 서버)
    assert f.accept(3, 'fm_a')                   # 같은 리더 · 같은 epoch
    assert f.accept(3, 'fm_b')                   # 동률이면 leader_id 큰 쪽이 이긴다
    assert not f.accept(3, 'fm_a')
    assert f.rejected == 2
    f2 = EpochFence(st)                          # 재시작 뒤에도 최댓값 유지
    assert not f2.accept(3, 'fm_a') and f2.accept(4, 'fm_a')
