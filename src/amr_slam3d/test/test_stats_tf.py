"""bag 스탬프 통계·누락 대응·TF 트리 판정 시험."""

import struct

from amr_slam3d import bagstat, tftree
import pytest


def _cdr(sec, nsec, tail=b'frame'):
    return b'\x00\x01\x00\x00' + struct.pack('<iI', sec, nsec) + tail


def test_cdr_header_stamp():
    assert bagstat.cdr_header_stamp(_cdr(12, 500000000)) == pytest.approx(12.5)
    with pytest.raises(ValueError):
        bagstat.cdr_header_stamp(b'\x00\x00\x00\x00' + struct.pack('<iI', 1, 0))   # 빅엔디언 표시
    with pytest.raises(ValueError):
        bagstat.cdr_header_stamp(b'\x00\x01\x00')


def test_interval_stats_regular_and_gap():
    st = [i * 0.1 for i in range(101)]
    s = bagstat.interval_stats(st, 10.0)
    assert s['count'] == 101 and s['rate_hz'] == pytest.approx(10.0)
    assert s['gaps'] == 0 and s['bursts'] == 0 and s['backwards'] == 0
    st2 = st[:50] + st[52:]                         # 두 장 빠짐 → 간격 0.3 s 하나
    s2 = bagstat.interval_stats(st2, 10.0)
    assert s2['gaps'] == 1 and s2['dt_max'] == pytest.approx(0.3)
    st3 = sorted(st + [5.0005])                     # 따라잡기 렌더 같은 짧은 간격
    assert bagstat.interval_stats(st3, 10.0)['bursts'] == 1


def test_interval_stats_counts_duplicates_and_backwards():
    s = bagstat.interval_stats([0.0, 0.1, 0.1, 0.05, 0.2])
    assert s['dup'] == 1 and s['backwards'] == 1


def test_matched_counts_missing():
    inp = [0.1 * i for i in range(10)]
    out = [s for i, s in enumerate(inp) if i != 3]
    m = bagstat.matched(inp, out)
    assert m['matched'] == 9 and m['missing'] == 1
    assert m['first_missing'] == [pytest.approx(0.3)]
    assert bagstat.matched(inp, inp)['missing'] == 0
    shifted = [s + 0.01 for s in inp]               # 스탬프가 다르면 대응이 아니다
    assert bagstat.matched(inp, shifted)['matched'] == 0


def test_tf_tree_valid_chain():
    edges = [('map', 'slam_map'), ('slam_map', 'amr_01/icp_odom'),
             ('amr_01/icp_odom', 'amr_01/odom'), ('amr_01/odom', 'amr_01/base_footprint'),
             ('amr_01/base_footprint', 'amr_01/base_link'),
             ('amr_01/base_link', 'amr_01/lidar3d_link')]
    r = tftree.check(edges, ['amr_01/base_footprint', 'amr_01/lidar3d_link'], root='map')
    assert r['valid'] and not r['multi_parent']
    assert r['chains']['amr_01/base_footprint']['path'][-1] == 'map'
    assert r['roots'] == ['map']


def test_tf_tree_two_parents_invalid():
    """LIO-SAM 행에 bag 의 EKF /tf 를 같이 틀었을 때: base_footprint 의 부모가 둘."""
    edges = [('map', 'slam_map'), ('slam_map', 'amr_01/liosam_odom'),
             ('amr_01/liosam_odom', 'amr_01/base_footprint'),
             ('amr_01/odom', 'amr_01/base_footprint')]
    r = tftree.check(edges, ['amr_01/base_footprint'], root='map')
    assert not r['valid']
    assert r['multi_parent'] == {'amr_01/base_footprint': ['amr_01/liosam_odom', 'amr_01/odom']}


def test_tf_tree_two_parents_off_the_checked_chain_still_invalid():
    """부모가 둘인 프레임이 조회 경로 밖에 있어도 트리는 무효다 (tf2 는 그 프레임 조회를 흔든다)."""
    edges = [('map', 'slam_map'), ('slam_map', 'base'), ('base', 'lidar_link'),
             ('liosam_odom', 'lidar_link')]
    r = tftree.check(edges, ['base'], root='map')
    assert r['chains']['base']['ok']
    assert not r['valid'] and 'lidar_link' in r['multi_parent']


def test_tf_tree_wrong_root_and_cycle():
    r = tftree.check([('odom', 'base')], ['base'], root='map')
    assert not r['valid'] and r['chains']['base']['root'] == 'odom'
    r2 = tftree.check([('a', 'b'), ('b', 'a')], ['a'])
    assert not r2['valid'] and r2['chains']['a']['status'] == 'cycle'


def test_tf_leading_slash_ignored():
    r = tftree.check([('/map', 'base')], ['/base'], root='map')
    assert r['valid']


def test_cdr_header_key_is_exact_integer():
    assert bagstat.cdr_header_key(_cdr(9, 480000000)) == (9, 480000000)
    with pytest.raises(ValueError):
        bagstat.cdr_header_key(b'\x00\x01\x00')


def test_matched_tail_exempt_separates_structural_tail():
    """LIO-SAM 2 장 캐시: 마지막 2 장 누락은 판정 누락에서 빼고 따로 센다. 원래 계수는 그대로 둔다."""
    inp = [0.1 * i for i in range(10)]
    out = [s for i, s in enumerate(inp) if i not in (3, 8, 9)]
    m = bagstat.matched(inp, out, tail_exempt=2)
    assert m['missing'] == 3 and m['input'] == 10                  # 옛 계수
    assert m['judged_input'] == 8 and m['judged_missing'] == 1
    assert m['tail_missing'] == 2 and m['tail_exempt'] == 2
    assert m['judged_missing_stamps'] == pytest.approx([0.3])
    assert m['missing_all'] == pytest.approx([0.3, 0.8, 0.9])
    # 끝 장을 처리했으면 tail_missing 0 — 끝 장 여부는 입력 순서가 아니라 스탬프로 정한다
    m2 = bagstat.matched(list(reversed(inp)), inp[:-2] + [inp[-1]], tail_exempt=2)
    assert m2['judged_missing'] == 0 and m2['tail_missing'] == 1
    m0 = bagstat.matched(inp, out)
    assert m0['judged_missing'] == 3 and m0['tail_missing'] == 0
    assert bagstat.matched(inp[:1], [], tail_exempt=5)['tail_exempt'] == 1
    with pytest.raises(ValueError):
        bagstat.matched(inp, out, tail_exempt=-1)
