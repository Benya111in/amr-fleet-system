"""alerts: /fleet/alerts 이름 규약, /fleet/traffic_events 교착 판별."""

import pytest

from amr_fleet import alerts


@pytest.mark.parametrize('name, expected', [
    ('traffic/DEADLOCK', True), ('traffic/deadlock', True), ('DEADLOCK', True),
    (' traffic/DEADLOCK ', True), ('traffic/RESOLVED', False), ('traffic/HOLD', False),
    ('deadlock_resolved', False), ('', False),
])
def test_is_deadlock_event(name, expected):
    assert alerts.is_deadlock_event(name) is expected


def test_alert_names_follow_fleet_prefix():
    names = [v for k, v in vars(alerts).items() if k.startswith('ALERT_')]
    assert len(names) == 6 and all(n.startswith('fleet/') and n.split('/')[1].isupper()
                                   for n in names)
    assert alerts.TRAFFIC_DEADLOCK == 'traffic/DEADLOCK'
    assert alerts.TRAFFIC_RESOLVED == 'traffic/RESOLVED'


def test_alert_values_drop_empty_and_stringify():
    assert alerts.alert_values('t1', 'amr_01') == {'task_id': 't1', 'robots': 'amr_01'}
    assert alerts.alert_values(robot_id='amr_02') == {'robots': 'amr_02'}
    assert alerts.alert_values('t2', attempts=2, status=1, note='', extra=None) == {
        'task_id': 't2', 'attempts': '2', 'status': '1'}
    assert alerts.alert_values() == {}
