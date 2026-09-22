"""estop: E-stop 발행·리셋 결과 모음과 대기."""

import threading
from types import SimpleNamespace as NS

from amr_dashboard import estop as es


def test_release_ok_and_detail():
    out = es.EstopOutcome('all', False)
    out.add_published('/fleet/estop')
    out.set_reset('amr_01', es.PENDING)
    out.set_reset('amr_02', es.NOT_CONFIGURED)
    assert out.pending() == ['amr_01']
    threading.Timer(0.05, out.set_reset, args=('amr_01', es.OK, 'reset')).start()
    assert out.wait(2.0) is True
    assert out.ok() and out.still_latched() == [] and out.errors() == []
    d = out.detail()
    assert d['reset'] == {'amr_01': 'ok', 'amr_02': 'not_configured'}
    assert d['messages'] == {'amr_01': 'reset'} and d['published'] == ['/fleet/estop']


def test_release_failures_keep_robots_latched():
    out = es.EstopOutcome('all', False)
    out.set_reset('amr_01', es.REJECTED, 'obstacle still inside 0.3 m')
    out.set_reset('amr_02', es.NO_SERVER, 'service not available')
    out.set_reset('amr_03', es.PENDING)
    out.fail_publish('rv-bad', 'InvalidTopicNameException: bad')
    assert out.wait(0.05) is False                 # amr_03 은 응답 없음 → timeout
    assert out.reset['amr_03'] == es.TIMEOUT
    assert out.still_latched() == ['amr_01', 'amr_02', 'amr_03', 'rv-bad']
    assert not out.ok()
    errs = out.errors()
    assert errs[0].startswith('rv-bad: E-stop 값 발행 실패')
    assert any('amr_01: reset_estop rejected (obstacle still inside 0.3 m)' in e for e in errs)
    assert all('래치가 남아' in e for e in errs[1:])


def test_no_server_counts_as_released_without_ack_requirement():
    out = es.EstopOutcome('amr_01', False, require_ack=False)
    out.set_reset('amr_01', es.NO_SERVER)
    assert out.ok() and out.still_latched() == []
    strict = es.EstopOutcome('amr_01', False, require_ack=True)
    strict.set_reset('amr_01', es.NO_SERVER)
    assert strict.still_latched() == ['amr_01']


def test_activation_never_latched_but_publish_failure_reported():
    out = es.EstopOutcome('amr_01', True)
    assert out.ok() and out.still_latched() == []
    out.fail_publish('amr_01', 'boom')
    assert not out.ok() and out.errors() == ['amr_01: E-stop 값 발행 실패 (boom)']


def test_simple_outcome_and_result_mapping():
    assert es.simple_outcome('amr_01', False).reset == {'amr_01': es.NOT_CONFIGURED}
    assert es.simple_outcome('all', False).reset == {}
    assert es.simple_outcome('amr_01', True).ok()
    assert es.reset_status_from_result(NS(success=True, message='ok')) == (es.OK, 'ok')
    assert es.reset_status_from_result(NS(success=False, message='no')) == (es.REJECTED, 'no')
    assert es.reset_status_from_result(None, RuntimeError('x')) == (es.ERROR, 'x')
    assert es.reset_status_from_result(None) == (es.ERROR, 'no response')
