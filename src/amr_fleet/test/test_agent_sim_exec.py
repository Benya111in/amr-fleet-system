"""가짜 실행기 단위 시험 (agent/sim_exec.py): 픽업은 잠금이 held 일 때만, denied 면 실패."""

from amr_fleet.agent.sim_exec import SimExecConfig, SimExecutor

CFG = SimExecConfig(pickup_s=1.0, deliver_s=1.0, commit_timeout_s=5.0, min_travel_s=0.5)


def _run(ex, phases, t0=0.0, t1=10.0, dt=0.1):
    """phases(t) → 잠금 단계. 사건 목록과 commit 호출 수."""
    evs, commits = [], []
    t = t0
    while t < t1 and ex.busy:
        t = round(t + dt, 6)
        evs += ex.step(t, lambda task: phases(t), lambda task: commits.append(t) or phases(t))
    return evs, commits


def test_pickup_only_after_held():
    ex = SimExecutor(CFG)
    ex.start('t1', 1.0, 0.0)
    evs, commits = _run(ex, lambda t: 'voted' if t < 3.0 else 'held')
    names = [(e['event'], round(e['t'], 1)) for e in evs]
    assert ('commit_request', 1.0) in names
    starts = [t for n, t in names if n == 'pickup_start']
    assert starts == [3.0]                       # 잠정 과반(voted)에서는 시작하지 않는다
    assert names[-1][0] == 'done' and commits


def test_denied_lock_fails_without_pickup():
    ex = SimExecutor(CFG)
    ex.start('t1', 0.5, 0.0)
    evs, _ = _run(ex, lambda t: 'pending' if t < 2.0 else 'denied')
    names = [e['event'] for e in evs]
    assert 'pickup_start' not in names
    assert evs[-1]['event'] == 'failed' and evs[-1]['reason'] == 'lock_denied'
    assert not ex.busy and not ex.picked_up


def test_commit_timeout_fails():
    ex = SimExecutor(CFG)
    ex.start('t1', 0.5, 0.0)
    evs, commits = _run(ex, lambda t: 'pending')
    assert evs[-1]['event'] == 'failed' and evs[-1]['reason'] == 'commit_timeout'
    assert len(commits) > 10                     # 확정 표시를 계속 다시 세운다 (멱등)


def test_start_twice_is_an_error():
    ex = SimExecutor(CFG)
    ex.start('t1', 1.0, 0.0)
    try:
        ex.start('t2', 1.0, 0.0)
    except RuntimeError:
        return
    raise AssertionError('두 번째 작업을 받았다')
