"""launch_testing_ros 연동: 러너 진입점(launch_test_ros) · WaitForTopics 기동 게이트."""

import unittest

from amr_itest import cases, catalog, launch_test_ros
from amr_itest.probe import GraphProbe
from amr_itest.scenario import Context
import pytest
from std_msgs.msg import String

LAUNCH_TEST = '''
import unittest

import launch
import launch_testing.actions
import launch_testing.markers


@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])


class TestInside(unittest.TestCase):

    def test_value(self):
        self.assertEqual(1, {expected})
'''


def test_runner_entry_point(tmp_path, capsys):
    ok = tmp_path / 'test_ok.py'
    ok.write_text(LAUNCH_TEST.format(expected=1))
    xml = tmp_path / 'ok.xml'
    assert launch_test_ros.main([str(ok), '--junit-xml', str(xml)]) == 0
    assert '<testsuites' in xml.read_text()
    bad = tmp_path / 'test_bad.py'
    bad.write_text(LAUNCH_TEST.format(expected=2))
    assert launch_test_ros.main([str(bad)]) == 1
    broken = tmp_path / 'test_broken.py'
    broken.write_text('raise RuntimeError("boom")\n')
    assert launch_test_ros.main([str(broken)]) == 2
    assert 'boom' in capsys.readouterr().err
    with pytest.raises(SystemExit) as exc:
        launch_test_ros.main([str(tmp_path / 'missing.py')])
    assert exc.value.code == 2
    assert launch_test_ros.build_parser().parse_args(['f.py', 'a:=1']).launch_arguments == ['a:=1']


def test_wait_for_topics_and_ready_gate(itest_env):
    ctx = Context(catalog.get(13)).begin()
    talker = GraphProbe('unit_talker', namespace=ctx.settings.namespace)
    pub = talker.publisher('unit_ready', String)
    talker.node.create_timer(0.05, lambda: pub.publish(String(data='x')))
    try:
        full = talker.node.resolve_topic_name('unit_ready')
        assert cases.wait_for_topics([(full, String)], 10.0) == (True, [])
        assert cases.wait_for_topics([(full, String), ('/unit_nobody', String)], 0.3) == \
            (False, ['/unit_nobody'])

        class Case(cases.ProbeCase):
            CTX = ctx

            def test_gate(self):
                self.ready_gate([('unit_ready', String)], 10.0)
                with self.assertRaises(AssertionError):
                    self.ready_gate([('unit_silent', String)], 0.3)

        result = unittest.TextTestRunner(verbosity=0).run(
            unittest.defaultTestLoader.loadTestsFromTestCase(Case))
        assert result.wasSuccessful(), result.failures + result.errors
    finally:
        talker.close()
    gates = [c for c in ctx.record.data['checks'] if c['name'].startswith('ready')]
    assert [c['passed'] for c in gates] == [True, False]
    assert gates[1]['value'] == [f'{ctx.settings.namespace}/unit_silent']
