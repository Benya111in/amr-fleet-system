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


def test_tf_tree_wrong_root_and_cycle():
    r = tftree.check([('odom', 'base')], ['base'], root='map')
    assert not r['valid'] and r['chains']['base']['root'] == 'odom'
    r2 = tftree.check([('a', 'b'), ('b', 'a')], ['a'])
    assert not r2['valid'] and r2['chains']['a']['status'] == 'cycle'


def test_tf_leading_slash_ignored():
    r = tftree.check([('/map', 'base')], ['/base'], root='map')
    assert r['valid']
