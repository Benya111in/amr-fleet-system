"""
Server-Sent Events 포맷터와 스트림 생성기 (순수 파이썬).

브라우저 EventSource 규격: `event:`/`data:`/`id:` 줄 + 빈 줄 하나로 이벤트 하나.
data 는 JSON 한 줄이며 프록시가 연결을 끊지 않도록 주기적으로 heartbeat 이벤트를 보낸다.
"""

import json
import queue
import time

# 브라우저 자동 재접속 대기 [ms] (첫 이벤트에 한 번 실어 보낸다)
RETRY_MS = 3000


def to_json(data) -> str:
    """이벤트 payload 를 한 줄 JSON 으로 만든다 (한글 그대로, NaN 금지)."""
    return json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False)


def format_sse(event: str, data, event_id=None, retry_ms=None) -> str:
    """SSE 이벤트 한 건의 와이어 표현. data 가 여러 줄이면 줄마다 `data:` 를 붙인다."""
    lines = []
    if retry_ms is not None:
        lines.append(f'retry: {int(retry_ms)}')
    if event_id is not None:
        lines.append(f'id: {event_id}')
    lines.append(f'event: {event}')
    payload = data if isinstance(data, str) else to_json(data)
    for line in payload.split('\n'):
        lines.append(f'data: {line}')
    return '\n'.join(lines) + '\n\n'


def heartbeat(ts=None) -> str:
    """SSE heartbeat 이벤트 (프록시 keep-alive 겸 클라이언트 연결 상태 표시)."""
    return format_sse('heartbeat', {'ts': time.time() if ts is None else ts})


def parse_sse(text: str) -> list:
    """
    SSE 와이어 텍스트 → [{'event', 'data', 'id'}] (테스트/검증 스크립트용 최소 파서).

    data 줄이 여러 개면 개행으로 이어 붙인다. 완결되지 않은(빈 줄로 끝나지 않은) 마지막 블록은 버린다.
    """
    events = []
    for block in text.split('\n\n'):
        if not block.strip():
            continue
        ev = {'event': 'message', 'data': [], 'id': None}
        for line in block.split('\n'):
            if line.startswith('event:'):
                ev['event'] = line[6:].strip()
            elif line.startswith('data:'):
                ev['data'].append(line[5:].lstrip(' '))
            elif line.startswith('id:'):
                ev['id'] = line[3:].strip()
        ev['data'] = '\n'.join(ev['data'])
        events.append(ev)
    if not text.endswith('\n\n') and events:
        events.pop()
    return events


def event_stream(store, heartbeat_period=5.0, initial_snapshot=True, max_events=None):
    """
    SSE 문자열을 내는 생성기. 구독자 하나 = 연결 하나.

    첫 이벤트로 전체 스냅샷을 보내 늦게 접속한 클라이언트도 상태를 갖추게 하고, 이후에는
    저장소가 배포하는 이벤트를 그대로 전달한다. heartbeat_period 동안 이벤트가 없으면
    heartbeat 를 보낸다. max_events 는 테스트용 (해당 개수 뒤 종료).
    """
    q = store.subscribe()
    sent = 0
    try:
        if initial_snapshot:
            yield format_sse('snapshot', store.snapshot(), retry_ms=RETRY_MS)
            sent += 1
        while max_events is None or sent < max_events:
            try:
                event, data, seq = q.get(timeout=heartbeat_period)
            except queue.Empty:
                yield heartbeat()
            else:
                yield format_sse(event, data, event_id=seq)
            sent += 1
    finally:
        store.unsubscribe(q)
