"""
에피소드 끝 정착 대기 (rclpy 비의존) — 기다리는 동안에도 추정 자세를 표집한다.

A5 리뷰 차단 1: 실행기가 추정 자세(TF map→base)를 감시 루프에서만, 즉 Nav2 종료까지만 표집했다. 지표는 성공 사건을 정착
구간까지 보므로, 정착 구간의 추정 기준 판정이 "Nav2 가 끝난 순간의 낡은 추정" 으로 이뤄졌다. 이 함수는 정착 대기의 매 폴링에서
표집하고, 끝에 한 번 더 표집해 그것을 최종 추정 자세로 돌려준다. 시계·잠을 주입받아 가짜 시계로 시험한다.
"""

import time


def settle(now, stopped, sample, sim_timeout=3.0, tail_wall=0.2, poll=0.02, wall_cap=None,
           clock=time.monotonic, sleep=time.sleep):
    """
    멈출 때까지 (sim 상한) 기다린 뒤 tail_wall 벽시계를 더 기다린다 — 그동안 poll 마다 sample() 을 부른다.

    now() → sim 시각 [s], stopped() → bool, sample() → 표본 또는 None (부르는 쪽이 기록한다).
    wall_cap: 정착 대기의 벽시계 상한 (기본 sim_timeout / 0.1 + 10 — gzbench_runner.wait_until 과 같다).
    반환 dict: t_final (마지막 표본 시각, sim), final (마지막 표본), stopped (멈췄나), n_samples (sample 호출 수).
    """
    cap = wall_cap if wall_cap is not None else sim_timeout / 0.1 + 10.0
    t0, w0 = now(), clock()
    n, is_stopped = 0, False
    while clock() - w0 < cap:
        sample()
        n += 1
        if stopped():
            is_stopped = True
            break
        if now() - t0 >= sim_timeout:
            break
        sleep(poll)
    w1 = clock()
    while clock() - w1 < tail_wall:
        sleep(poll)
        sample()
        n += 1
    t_final = now()
    final = sample()
    n += 1
    return {'t_final': t_final, 'final': final, 'stopped': is_stopped, 'n_samples': n}
