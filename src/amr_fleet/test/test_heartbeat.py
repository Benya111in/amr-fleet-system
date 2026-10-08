"""관리자 하트비트 상태기 (heartbeat.py, rclpy 비의존) — 관찰자 복귀 · epoch · seq."""

import pytest

from amr_fleet.heartbeat import ManagerHeartbeat, ROLE_ACTIVE, ROLE_OBSERVER
from amr_fleet.task_board import resolve_epoch, write_epoch_file


def test_without_observation_it_is_active_from_the_first_beat():
    hb = ManagerHeartbeat(3)
    assert hb.active
    b = hb.tick(10.0)
    assert (b.epoch, b.seq, b.role, b.activated) == (3, 1, ROLE_ACTIVE, False)
    assert hb.tick(10.5).seq == 2


def test_nothing_before_the_clock_starts():
    hb = ManagerHeartbeat(1, observe_s=2.0)
    assert hb.tick(0.0) is None and hb.seq == 0 and hb.started is None
    assert hb.tick(-1.0) is None
    b = hb.tick(100.0)                       # 관찰은 첫 sim 시각부터 잰다 (0 → 100 도약을 세지 않는다)
    assert b.role == ROLE_OBSERVER and not hb.active


def test_observer_takes_max_seen_plus_one_and_keeps_seq():
    hb = ManagerHeartbeat(1, observe_s=2.0)
    b1 = hb.tick(100.0)
    assert (b1.epoch, b1.role) == (1, ROLE_OBSERVER)      # 아직 비컨 없음 → 기본 epoch
    hb.on_beacon(3)
    hb.on_beacon(5)
    hb.on_beacon(4)                                       # 최댓값만
    b2 = hb.tick(100.5)
    assert (b2.epoch, b2.role, b2.seq) == (6, ROLE_OBSERVER, 2)
    assert not hb.active
    b3 = hb.tick(101.9)
    assert b3.role == ROLE_OBSERVER and not b3.activated
    b4 = hb.tick(102.0)                                   # observe_s 경과
    assert (b4.epoch, b4.role, b4.activated, b4.seq) == (6, ROLE_ACTIVE, True, 4)
    assert hb.active and hb.activated_at == 102.0
    b5 = hb.tick(102.5)
    assert (b5.epoch, b5.seq, b5.activated) == (6, 5, False)   # seq 를 1 로 되돌리지 않는다


def test_base_epoch_wins_when_robots_saw_lower():
    hb = ManagerHeartbeat(9, observe_s=1.0)
    hb.on_beacon(4)
    hb.tick(1.0)
    assert hb.tick(2.0).epoch == 9


def test_epoch_is_frozen_after_activation_and_superseded_is_reported():
    hb = ManagerHeartbeat(1, observe_s=1.0)
    hb.on_beacon(2)
    hb.tick(1.0)
    assert hb.tick(2.0).epoch == 3 and hb.active
    assert not hb.superseded()
    hb.on_beacon(3)                       # 내 epoch 와 같다 — 내 하트비트를 본 로봇
    assert not hb.superseded()
    hb.on_beacon(7)                       # 더 높은 권위자를 본 로봇이 있다 → 내 명령은 거부된다
    assert hb.superseded()
    assert hb.tick(2.5).epoch == 3        # 스스로 올리지 않는다 (관리자 둘의 epoch 경쟁 방지)


def test_observer_is_not_superseded_while_observing():
    hb = ManagerHeartbeat(1, observe_s=5.0)
    hb.tick(1.0)
    hb.on_beacon(10)
    assert not hb.superseded()


@pytest.mark.parametrize('epoch,observe', [(0, 0.0), (-1, 0.0), (1, -0.5), (1, float('nan'))])
def test_bad_arguments(epoch, observe):
    with pytest.raises(ValueError):
        ManagerHeartbeat(epoch, observe_s=observe)


def test_write_epoch_file_feeds_the_next_resolve(tmp_path):
    path = str(tmp_path / 'sub' / 'epoch')
    assert resolve_epoch(0, path) == 1
    write_epoch_file(path, 6)              # 관찰자 복귀로 올린 epoch 를 남긴다
    assert resolve_epoch(0, path) == 7     # 다음 기동은 그보다 높다
    with pytest.raises(ValueError):
        write_epoch_file(path, 0)
    assert not [p for p in (tmp_path / 'sub').iterdir() if p.name.startswith('.epoch_')]
