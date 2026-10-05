"""security: Host 허용 목록 · Origin · 조작 토큰."""

import pytest

from amr_dashboard import security as sec


@pytest.mark.parametrize('header, name', [
    ('127.0.0.1:8080', '127.0.0.1'), ('LOCALHOST', 'localhost'), ('[::1]:8080', '::1'),
    ('[::1]', '::1'), ('::1', '::1'), ('attacker.example:18077', 'attacker.example'),
    ('', ''), ('[broken', ''),
])
def test_split_host(header, name):
    assert sec.split_host(header) == name


def test_allowed_hosts_loopback_specific_and_wildcard(monkeypatch):
    assert sec.allowed_hosts('127.0.0.1') == ['localhost', '127.0.0.1', '::1']
    assert sec.allowed_hosts('192.168.0.7', ['Dash.Local']) == \
        ['localhost', '127.0.0.1', '::1', '192.168.0.7', 'dash.local']
    monkeypatch.setattr(sec, 'local_host_names', lambda: ['robo-srv', '10.0.0.5'])
    assert sec.allowed_hosts('0.0.0.0') == \
        ['localhost', '127.0.0.1', '::1', 'robo-srv', '10.0.0.5']
    assert sec.allowed_hosts('0.0.0.0', include_local=False) == ['localhost', '127.0.0.1', '::1']


def test_local_host_names_is_best_effort(monkeypatch):
    assert isinstance(sec.local_host_names(), list)

    def boom(*_a):
        raise OSError('no dns')
    monkeypatch.setattr(sec.socket, 'gethostname', boom)
    assert sec.local_host_names() == []


def test_host_allowed_blocks_dns_rebinding():
    allowed = sec.allowed_hosts('127.0.0.1')
    assert sec.host_allowed('127.0.0.1:8080', allowed)
    assert sec.host_allowed('localhost:8080', allowed)
    assert sec.host_allowed('[::1]:8080', allowed)
    assert not sec.host_allowed('attacker.example:18077', allowed)
    assert not sec.host_allowed('', allowed)
    assert sec.host_allowed('anything', ['*'])


@pytest.mark.parametrize('origin, host, ok', [
    (None, '127.0.0.1:8080', True),
    ('', '127.0.0.1:8080', True),
    ('http://127.0.0.1:8080', '127.0.0.1:8080', True),
    ('http://LOCALHOST:8080', 'localhost:8080', True),
    ('http://attacker.example:8080', '127.0.0.1:8080', False),
    ('http://127.0.0.1:9999', '127.0.0.1:8080', False),
    ('null', '127.0.0.1:8080', False),
    ('file://', '127.0.0.1:8080', False),
])
def test_origin_allowed(origin, host, ok):
    assert sec.origin_allowed(origin, host) is ok


def test_token_ok_and_resolve():
    assert sec.token_ok(None, '')
    assert sec.token_ok('abc', 'abc')
    assert not sec.token_ok('abd', 'abc')
    assert not sec.token_ok(None, 'abc')
    assert sec.resolve_token(' p ', {sec.TOKEN_ENV: 'e'}) == 'p'
    assert sec.resolve_token('', {sec.TOKEN_ENV: ' e '}) == 'e'
    assert sec.resolve_token('', {}) == ''
