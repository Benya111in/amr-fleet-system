"""
HTTP 요청 출처 검사 (순수 파이썬): Host 허용 목록 · Origin · 조작 토큰.

위협 (리뷰 r-dashboard):
- DNS rebinding: 공격 페이지의 도메인을 127.0.0.1 로 다시 풀면 브라우저는 그것을 같은 출처로 보고
  E-stop 해제·작업 투입 POST 를 보낸다. Host 헤더가 공격자 도메인이므로 Host 허용 목록으로 막는다.
- 교차 출처 POST: Origin 이 이 서버(Host 와 같은 scheme://host:port)가 아니면 거부한다.
- 공유 서버의 다른 사용자: 127.0.0.1 에는 누구나 붙을 수 있으므로 조작 API(POST /api/estop,
  /api/tasks)에 선택적 토큰(api_token 파라미터 또는 AMR_DASHBOARD_TOKEN 환경 변수)을 요구한다.
  토큰은 페이지에 심지 않는다 (페이지는 누구나 받을 수 있다) — 운영자가 처음 조작할 때 입력한다.
"""

import hmac
import socket
from typing import Iterable, List, Optional
from urllib.parse import urlsplit

LOOPBACK_NAMES = ('localhost', '127.0.0.1', '::1')
WILDCARD_BINDS = ('0.0.0.0', '::', '')
TOKEN_HEADER = 'X-Dashboard-Token'
TOKEN_ENV = 'AMR_DASHBOARD_TOKEN'


def split_host(host: str) -> str:
    """Host 헤더 값 → 소문자 호스트 이름 (포트·IPv6 괄호 제거). 형식이 틀리면 ''."""
    host = (host or '').strip().lower()
    if not host:
        return ''
    if host.startswith('['):                       # [::1]:8080
        end = host.find(']')
        return host[1:end] if end > 0 else ''
    if host.count(':') == 1:                       # name:port
        return host.split(':', 1)[0]
    return host                                    # name 또는 괄호 없는 IPv6


def local_host_names() -> List[str]:
    """이 컴퓨터의 호스트 이름·FQDN·IP (0.0.0.0 바인드일 때 LAN 에서 쓰는 이름)."""
    names = []
    try:
        hostname = socket.gethostname()
        names += [hostname, socket.getfqdn()]
        names += socket.gethostbyname_ex(hostname)[2]
    except OSError:
        pass
    return names


def allowed_hosts(bind_host: str, extra: Iterable[str] = (), include_local: bool = True
                  ) -> List[str]:
    """
    받아들일 Host 이름 목록: 루프백 + 특정 주소에 바인드했으면 그 주소 + extra.

    0.0.0.0/:: 에 바인드했으면(LAN 공개) 이 컴퓨터의 호스트 이름·IP 도 넣는다. extra 에 '*' 가 있으면
    검사를 끈다 (DNS rebinding 방어가 없어지므로 권하지 않는다).
    """
    hosts = list(LOOPBACK_NAMES)
    bind = (bind_host or '').strip().lower()
    if bind not in WILDCARD_BINDS:
        hosts.append(split_host(bind) or bind)
    elif include_local:
        hosts += local_host_names()
    hosts += [str(h) for h in extra]
    out = []
    for h in hosts:
        h = h.strip().lower()
        if h and h not in out:
            out.append(h)
    return out


def host_allowed(host_header: str, allowed: Iterable[str]) -> bool:
    """Host 헤더가 허용 목록에 있는가 ('*' 면 항상 참)."""
    allowed = list(allowed)
    if '*' in allowed:
        return True
    name = split_host(host_header)
    return bool(name) and name in allowed


def origin_allowed(origin: Optional[str], host_header: str) -> bool:
    """
    Origin 헤더 검사: 없으면(같은 출처 GET·curl) 통과, 있으면 Host 와 같은 호스트:포트여야 한다.

    'null' (파일·샌드박스 iframe) 은 거부한다.
    """
    if origin is None or origin == '':
        return True
    if origin.strip().lower() == 'null':
        return False
    parts = urlsplit(origin.strip())
    if parts.scheme not in ('http', 'https') or not parts.netloc:
        return False
    return parts.netloc.lower() == (host_header or '').strip().lower()


def token_ok(provided: Optional[str], expected: str) -> bool:
    """토큰 비교 (expected 가 비면 토큰 없음 = 항상 참). 시간 상수 비교."""
    if not expected:
        return True
    return hmac.compare_digest((provided or '').encode('utf-8'), expected.encode('utf-8'))


def resolve_token(param_value: str, environ) -> str:
    """토큰: 파라미터가 있으면 그것, 없으면 환경 변수 AMR_DASHBOARD_TOKEN, 둘 다 없으면 ''."""
    return (param_value or '').strip() or (environ.get(TOKEN_ENV, '') or '').strip()
