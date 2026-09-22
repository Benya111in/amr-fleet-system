"""tf_tree: 간선 그래프·계약 간선·판정."""

import math

from amr_itest import config, tf_tree
import pytest


def test_quaternion_helpers():
    q = tf_tree.quat_from_rpy(0.0, 0.0, math.pi / 2)
    assert q == pytest.approx((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))
    assert tf_tree.quat_angle(q, tuple(-v for v in q)) == pytest.approx(0.0, abs=1e-6)
    assert tf_tree.quat_angle((0, 0, 0, 1), q) == pytest.approx(math.pi / 2)


def _graph():
    g = tf_tree.TfGraph()
    for t in (0.0, 0.02, 0.04, 0.06):
        g.add('map', 'odom', t, False)
        g.add('/odom', 'base_footprint', t, False)
    g.add('base_footprint', 'base_link', 0.0, True, (0.0, 0.0, 0.18))
    g.add('base_link', 'lidar_link', 0.0, True, (0.15, 0.0, 0.2))
    return g


def test_graph_structure():
    g = _graph()
    assert g.edge('map', 'odom').rate() == pytest.approx(50.0)
    assert g.edge('base_footprint', 'base_link').rate() == 0.0     # 정적
    assert g.roots() == {'map'}
    assert g.chain('lidar_link') == ['lidar_link', 'base_link', 'base_footprint', 'odom', 'map']
    assert not g.duplicate_parents()
    assert not g.has_cycle()
    assert 'odom' in g.frames() and len(g.edges) == 4
    g.add('world', 'odom', 0.0, False)
    assert g.duplicate_parents() == {'odom': ['map', 'world']}
    g.add('lidar_link', 'map', 0.0, True)
    assert g.has_cycle()
    with pytest.raises(ValueError):
        g.chain('map')
    dot = g.to_dot(highlight=[('base_link', 'imu_link')])
    assert '"map" -> "odom"' in dot and 'missing' in dot
    single = tf_tree.ObservedEdge('a', 'b', stamps=[1.0, 1.0])
    assert single.rate() == 0.0


def test_depth_limit():
    g = tf_tree.TfGraph()
    for i in range(5):
        g.add(f'f{i + 1}', f'f{i}', 0.0, True)
    with pytest.raises(ValueError):
        g.chain('f0', max_depth=3)


def test_expected_edges_from_config():
    edges = tf_tree.expected_edges(config.sensors(), config.robot_params(), config.ekf_params())
    names = [(e.parent, e.child) for e in edges]
    assert names[:3] == [('map', 'odom'), ('odom', 'base_footprint'),
                         ('base_footprint', 'base_link')]
    lidar = next(e for e in edges if e.child == 'lidar_link')
    ext = config.sensors()['lidar']['extrinsic']
    assert lidar.translation == pytest.approx((ext['x'], ext['y'], ext['z']))
    prefixed = tf_tree.expected_edges(config.sensors(), config.robot_params(),
                                      config.ekf_params(), 'amr_02/')
    assert prefixed[0].parent == 'map' and prefixed[0].child == 'amr_02/odom'
    assert all(e.child.startswith('amr_02/') for e in prefixed)


def test_check_edges_reports_problems():
    expected = tf_tree.expected_edges({}, {}, {})
    g = _graph()
    g.add('base_link', 'imu_link', 0.0, True, (0.0, 0.0, 0.5))             # 값 틀림
    g.add('odom', 'camera_link', 0.0, True, (0.18, 0.0, 0.25))             # 부모 틀림
    g.add('world', 'base_footprint', 0.0, False)                            # 이중 부모
    checks = {c.expected.child: c for c in tf_tree.check_edges(
        g, expected, min_rate={'odom': 100.0})}
    assert checks['base_link'].ok
    assert not checks['odom'].ok and 'rate' in checks['odom'].problems[0]
    assert 'multiple parents' in checks['base_footprint'].problems[0]
    assert not checks['imu_link'].ok and checks['imu_link'].translation_error > 0.3
    assert 'wrong parent' in checks['camera_link'].problems[0]
    assert checks['left_wheel_link'].problems == ['missing']
    row = checks['imu_link'].as_row()
    assert len(row) == len(tf_tree.EDGE_CSV_COLUMNS) and row[8] == 0
    g2 = tf_tree.TfGraph()
    g2.add('base_link', 'lidar_link', 0.0, True, (0.0, 0.0, 0.0),
           tf_tree.quat_from_rpy(0.0, 0.0, 0.1))
    rot = tf_tree.check_edges(g2, [e for e in expected if e.child == 'lidar_link'])[0]
    assert rot.rotation_error == pytest.approx(0.1) and 'rotation' in rot.problems[0]


def test_detect_prefix():
    assert tf_tree.detect_prefix(['map', 'base_link']) == ''
    assert tf_tree.detect_prefix(['map', 'amr_01/base_link']) == 'amr_01/'
    assert tf_tree.detect_prefix(['amr_01/base_link', 'amr_02/base_link']) is None
    assert tf_tree.detect_prefix(['map']) is None
