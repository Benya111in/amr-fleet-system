"""comm_emulator: 링크별 결정적 지연·손실·분할 모의 (보너스 4 P0, 같은 시드 → 같은 지연열)."""

import random
import statistics

import pytest

from amr_fleet.comm_emulator import (
    DELIVERED, DROP_BURST, DROP_LOSS, DROP_OUTAGE, DROP_PARTITION, CommEmulator,
    GilbertElliott, LinkModel, Outage, Partition, from_node_params, link_model, resolve_seed,
    summarize,
)

ROBOTS = ['amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05']


def _seq(em, src, dst, n, t0=0.0, dt=0.5):
    return [em.transmit(src, dst, t0 + i * dt) for i in range(n)]


def test_delay_inside_range_and_fills_it():
    em = CommEmulator(seed=11, default=LinkModel(5.0, 30.0))
    d = _seq(em, 'fleet', 'amr_01', 2000)
    assert all(0.005 <= x <= 0.030 for x in d)
    assert min(d) < 0.006 and max(d) > 0.029
    assert statistics.mean(d) == pytest.approx(0.0175, abs=0.001)


def test_same_seed_same_sequence_per_link():
    a = CommEmulator(seed=42)
    b = CommEmulator(seed=42)
    for rid in ROBOTS:
        assert _seq(a, 'fleet', rid, 50) == _seq(b, 'fleet', rid, 50)
        assert _seq(a, rid, 'fleet', 50) == _seq(b, rid, 'fleet', 50)
    assert _seq(CommEmulator(seed=43), 'fleet', 'amr_01', 50) != _seq(
        CommEmulator(seed=42), 'fleet', 'amr_01', 50)


def test_link_sequence_independent_of_other_links_traffic():
    """
    공통 난수: 다른 링크에 메시지를 몇 개, 어떤 순서로 끼워 넣어도 이 링크의 k 번째 지연은 같다.

    노드당 RNG 하나(옛 LatencyModel)면 정책이 호출 순서를 바꿀 때 모든 링크의 난수열이 밀린다.
    """
    ref = _seq(CommEmulator(seed=9), 'amr_01', 'amr_02', 40)
    em = CommEmulator(seed=9)
    rng = random.Random(1)
    got = []
    for i in range(40):
        for _ in range(rng.randrange(4)):        # 다른 링크 잡음 (로봇↔로봇, 중앙)
            src, dst = rng.choice(ROBOTS), rng.choice(ROBOTS + ['fleet'])
            if (src, dst) != ('amr_01', 'amr_02'):
                em.transmit(src, dst, i * 0.5)
        got.append(em.transmit('amr_01', 'amr_02', i * 0.5))
    assert got == ref


def test_direction_matters():
    em = CommEmulator(seed=5)
    assert _seq(em, 'amr_01', 'amr_02', 20) != _seq(em, 'amr_02', 'amr_01', 20)


def test_golden_values_pin_derivation():
    """파생 규칙(blake2b 키 형식)을 바꾸면 이전 실행의 시드로 재현할 수 없다 → 값을 고정해 둔다."""
    em = CommEmulator(seed=1, default=LinkModel(0.0, 100.0))
    got = [round(x * 1e3, 6) for x in _seq(em, 'fleet', 'amr_01', 3)]
    assert got == GOLDEN_FLEET_AMR01


# seed=1, (fleet → amr_01), k = 0, 1, 2 의 지연 [ms] (2026-10-06 이 구현으로 계산해 고정)
GOLDEN_FLEET_AMR01 = [4.030257, 88.551425, 30.899439]


def test_seed_zero_draws_nonzero_seed_and_is_reproducible():
    a = CommEmulator(seed=0)
    assert a.seed != 0 and a.requested_seed == 0
    assert str(a.seed) in a.describe() and '무작위' in a.describe()
    first = _seq(a, 'fleet', 'amr_03', 30)
    again = _seq(CommEmulator(seed=a.seed), 'fleet', 'amr_03', 30)   # 기록한 실효 시드로 재현
    assert first == again
    assert resolve_seed(7) == 7
    with pytest.raises(ValueError):
        resolve_seed(-1)


def test_bernoulli_drop_rate_and_counts():
    em = CommEmulator(seed=2, default=LinkModel(0.0, 10.0, drop_rate=0.25))
    d = _seq(em, 'amr_02', 'fleet', 4000)
    lost = sum(x is None for x in d)
    assert lost / len(d) == pytest.approx(0.25, abs=0.03)
    assert em.stats[DROP_LOSS] == lost and em.stats[DELIVERED] == len(d) - lost
    assert em.count('amr_02', 'fleet') == 4000           # 유실도 k 를 올린다


def test_simulate_false_is_transparent():
    em = CommEmulator(seed=3, default=LinkModel(0.0, 100.0, drop_rate=0.5),
                      partitions=[Partition(0.0, 1e9, (frozenset({'fleet'}),))], simulate=False)
    assert _seq(em, 'fleet', 'amr_01', 100) == [0.0] * 100
    assert not em.enabled('fleet', 'amr_01') and em.describe() == 'off'


def test_enabled_reflects_link_model_and_schedules():
    zero = LinkModel(0.0, 0.0)
    assert not CommEmulator(seed=1, default=zero).enabled('fleet', 'amr_01')
    assert CommEmulator(seed=1, default=LinkModel(0.0, 0.0, 0.1)).enabled('fleet', 'amr_01')
    out = CommEmulator(seed=1, default=zero, outages=[Outage(1.0, 2.0, 'amr_01', 'fleet')])
    assert out.enabled('amr_01', 'fleet') and not out.enabled('fleet', 'amr_01')
    part = CommEmulator(seed=1, default=zero,
                        partitions=[Partition(1.0, 2.0, (frozenset({'amr_01'}),))])
    assert part.enabled('fleet', 'amr_02')


def test_partition_drops_only_across_groups_inside_window():
    p = Partition(10.0, 20.0, (frozenset({'amr_01', 'amr_02'}),))   # {R1,R2} 대 나머지 (F5 형태)
    em = CommEmulator(seed=4, default=LinkModel(0.0, 50.0), partitions=[p])
    assert em.transmit('amr_01', 'amr_03', 15.0) is None
    assert em.transmit('fleet', 'amr_02', 15.0) is None
    assert em.transmit('amr_01', 'amr_02', 15.0) is not None        # 같은 그룹
    assert em.transmit('amr_03', 'fleet', 15.0) is not None         # 둘 다 나머지
    assert em.transmit('amr_01', 'amr_03', 9.999) is not None       # 구간 밖
    assert em.transmit('amr_01', 'amr_03', 20.0) is not None        # 끝은 열린 구간
    assert em.stats[DROP_PARTITION] == 2
    with pytest.raises(ValueError):
        Partition(5.0, 5.0, ())
    with pytest.raises(ValueError):
        Partition(0.0, 1.0, (frozenset({'a'}), frozenset({'a', 'b'})))


def test_outage_is_directional_with_patterns():
    em = CommEmulator(seed=4, default=LinkModel(0.0, 0.0),
                      outages=[Outage(0.0, 5.0, src='amr_0[12]', dst='fleet')])
    assert em.transmit('amr_01', 'fleet', 1.0) is None              # 상향만 끊김 (비대칭)
    assert em.transmit('amr_02', 'fleet', 1.0) is None
    assert em.transmit('fleet', 'amr_01', 1.0) == 0.0
    assert em.transmit('amr_03', 'fleet', 1.0) == 0.0
    assert em.transmit('amr_01', 'fleet', 5.0) == 0.0
    assert em.stats[DROP_OUTAGE] == 2


def test_burst_state_is_function_of_sim_time_only():
    ge = GilbertElliott(mean_good_s=8.0, mean_bad_s=2.0, loss_good=0.0, loss_bad=1.0)
    a = CommEmulator(seed=21, default=LinkModel(0.0, 0.0, burst=ge))
    b = CommEmulator(seed=21, default=LinkModel(0.0, 0.0, burst=ge))
    times = [i * 0.05 for i in range(20000)]                         # sim 1000 s
    for t in times[::7]:                                             # 메시지를 많이 보낸 쪽
        a.transmit('fleet', 'amr_01', t)
    states_a = [a.burst_bad('fleet', 'amr_01', t) for t in times]
    states_b = [b.burst_bad('fleet', 'amr_01', t) for t in reversed(times)][::-1]  # 역순 조회
    assert states_a == states_b
    frac = sum(states_a) / len(states_a)
    assert frac == pytest.approx(ge.mean_bad_s / (ge.mean_good_s + ge.mean_bad_s), abs=0.06)
    assert ge.mean_loss == pytest.approx(0.2)
    # 다른 링크는 다른 버스트열
    other = [b.burst_bad('fleet', 'amr_02', t) for t in times]
    assert other != states_a
    assert not a.burst_bad('fleet', 'amr_01', -1.0)


def test_burst_losses_fall_inside_bad_periods():
    ge = GilbertElliott(mean_good_s=5.0, mean_bad_s=1.5)            # loss_bad 1, loss_good 0
    em = CommEmulator(seed=8, default=LinkModel(0.0, 10.0, burst=ge))
    for i in range(4000):
        t = i * 0.5
        d = em.transmit('amr_04', 'fleet', t)
        assert (d is None) == em.burst_bad('amr_04', 'fleet', t)
    assert em.stats[DROP_BURST] > 0 and em.stats[DELIVERED] > 0
    with pytest.raises(ValueError):
        GilbertElliott(0.0, 1.0)
    with pytest.raises(ValueError):
        GilbertElliott(1.0, 1.0, loss_bad=1.5)


def test_link_rules_first_match_and_robot_to_robot():
    em = CommEmulator.from_config({
        'links': [
            {'src': 'amr_01', 'dst': 'amr_02', 'comm_latency_ms': [0, 0], 'drop_rate': 0.5},
            {'src': 'amr_*', 'dst': 'amr_*', 'comm_latency_ms': [200, 300]},
        ]}, seed=6, default=LinkModel(0.0, 100.0))
    assert em.model('amr_01', 'amr_02').drop_rate == 0.5
    assert em.model('amr_02', 'amr_01').max_ms == 300.0              # 로봇↔로봇 규칙
    assert em.model('fleet', 'amr_01').max_ms == 100.0               # default
    assert em.exceeds_spec() and '링크 규칙 2' in em.describe()
    d = [em.transmit('amr_03', 'amr_05', i) for i in range(200)]
    assert all(0.2 <= x <= 0.3 for x in d)


def test_from_config_partitions_outages_seed_and_validation():
    em = CommEmulator.from_config({
        'seed': 77,
        'partitions': [{'start_s': 1, 'end_s': 2, 'groups': [['fleet']]}],
        'outages': [{'start_s': 3, 'end_s': 4, 'src': 'amr_05'}],
    }, seed=5)
    assert em.seed == 77                                             # 파일 시드가 이긴다
    assert em.cut_reason('fleet', 'amr_01', 1.5) == DROP_PARTITION   # 서버 ↔ 전체 (F4)
    assert em.cut_reason('amr_05', 'amr_01', 3.5) == DROP_OUTAGE     # 로봇 1 대 송신 단절
    assert em.cut_reason('amr_01', 'amr_05', 3.5) is None
    with pytest.raises(ValueError):
        CommEmulator.from_config({'linkz': []})
    with pytest.raises(ValueError):
        CommEmulator.from_config({'links': [{'comm_latency_ms': [50, 10]}]})
    with pytest.raises(ValueError):
        link_model({'drop_rate': 1.0})


def test_from_node_params_matches_fleet_yaml_semantics(tmp_path):
    em = from_node_params(True, [5.0, 30.0], 0.1, 13)
    assert em.default == LinkModel(5.0, 30.0, 0.1) and em.seed == 13
    em2 = from_node_params(True, 80.0, 0.0, 13)                      # 숫자 하나 = [0, 값]
    assert (em2.default.min_ms, em2.default.max_ms) == (0.0, 80.0)
    f = tmp_path / 'comm.yaml'
    f.write_text('links:\n  - {src: "amr_*", dst: "amr_*", drop_rate: 0.2}\n', encoding='utf-8')
    em3 = from_node_params(True, [0.0, 100.0], 0.0, 13, str(f))
    assert em3.model('amr_01', 'amr_02').drop_rate == 0.2
    assert em3.model('amr_01', 'amr_02').max_ms == 100.0             # 빠진 값은 기본 링크에서
    f.write_text('- 1\n', encoding='utf-8')
    with pytest.raises(ValueError):
        from_node_params(True, [0.0, 100.0], 0.0, 13, str(f))


def test_history_and_summarize():
    em = CommEmulator(seed=3, default=LinkModel(0.0, 50.0, drop_rate=0.3), history=10)
    for i in range(30):
        em.transmit('fleet', 'amr_01', float(i))
    assert len(em.history) == 10 and em.history[-1].index == 29
    seqs = summarize(list(em.history))
    assert list(seqs) == [('fleet', 'amr_01')] and len(seqs[('fleet', 'amr_01')]) == 10


def test_example_config_file_loads():
    import pathlib
    path = pathlib.Path(__file__).resolve().parents[1] / 'config' / 'examples' / 'comm_model.yaml'
    em = from_node_params(True, [0.0, 100.0], 0.0, 1, str(path))
    assert em.model('amr_02', 'amr_05').drop_rate == 0.05
    assert em.model('fleet', 'amr_01').burst.mean_bad_s == 2.5
    assert em.model('fleet', 'amr_01').burst.mean_loss == pytest.approx(2.5 / 52.5)
    assert em.cut_reason('amr_01', 'fleet', 310.0) == DROP_PARTITION
    assert em.cut_reason('amr_03', 'fleet', 405.0) == DROP_OUTAGE
    assert em.cut_reason('fleet', 'amr_03', 405.0) is None
