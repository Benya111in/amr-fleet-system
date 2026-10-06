"""amr_rl_bench.settle 시험 — 정착 대기 내내 추정 자세를 표집하는가 (A5 리뷰 차단 1, 가짜 시계)."""

from amr_rl_bench.settle import settle
import pytest


class FakeClock:
    """벽시계 w, sim = 100 + rtf·w (rtf 0 = sim 멈춤)."""

    def __init__(self, rtf=0.8):
        self.w = 0.0
        self.rtf = rtf

    def wall(self):
        return self.w

    def sleep(self, dt):
        self.w += dt

    def sim(self):
        return 100.0 + self.rtf * self.w


def run(rtf=0.8, stop_at=None, **kw):
    """stop_at (sim) 에 멈추는 로봇으로 settle → (결과, 표본 목록)."""
    c = FakeClock(rtf)
    rows = []

    def sample():
        row = [c.sim(), 1.0, 2.0, 0.0]
        rows.append(row)
        return row

    def stopped():
        return stop_at is not None and c.sim() >= stop_at
    res = settle(c.sim, stopped, sample, clock=c.wall, sleep=c.sleep, **kw)
    return res, rows


def test_samples_through_wait_and_tail():
    """멈추기 전 대기 구간과 멈춘 뒤 0.2 s 꼬리 내내 표집, 마지막 표본 = 최종."""
    res, rows = run(stop_at=100.5)
    t = [r[0] for r in rows]
    assert res['stopped']
    assert t[0] == pytest.approx(100.0)                 # 대기 시작부터
    assert max(b - a for a, b in zip(t[:-1], t[1:])) <= 0.02 * 0.8 + 1e-9
    k_stop = next(i for i, x in enumerate(t) if x >= 100.5)
    assert k_stop >= 25                                 # 0.5 s sim / (0.02 × 0.8) ≈ 31 표본이 정착 대기 중
    assert t[-1] - 100.5 >= 0.2 * 0.8 - 1e-9            # 꼬리 0.2 s 벽시계
    assert res['final'] is rows[-1] and res['t_final'] == pytest.approx(rows[-1][0])
    assert res['n_samples'] == len(rows)


def test_sim_timeout_without_stop():
    """멈추지 않으면 sim 3 s 뒤 끝 — 그 3 s 내내 표집."""
    res, rows = run(stop_at=None, sim_timeout=3.0)
    assert not res['stopped']
    span = rows[-1][0] - rows[0][0]
    # sim 상한 + 꼬리 (벽시계 상한 40 s 가 아니라)
    assert 3.0 <= span <= 3.0 + 0.2 * 0.8 + 2 * 0.02 * 0.8 + 1e-9
    assert len(rows) >= 3.0 / (0.02 * 0.8)


def test_frozen_sim_clock_hits_wall_cap():
    """시계(sim)가 멈추면 벽시계 상한에서 끝난다 (실행기가 영원히 기다리지 않게)."""
    res, rows = run(rtf=0.0, stop_at=None, wall_cap=1.0)
    assert not res['stopped']
    assert 1.0 <= len(rows) * 0.02 <= 1.0 + 0.2 + 0.1


def test_sample_failure_returns_none_final():
    """TF 가 없으면 (표본 None) 최종도 None — 실행기는 기록의 마지막 표본으로 대신한다."""
    c = FakeClock()
    res = settle(c.sim, lambda: True, lambda: None, clock=c.wall, sleep=c.sleep)
    assert res['final'] is None and res['stopped']
