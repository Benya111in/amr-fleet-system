"""sse: 이벤트 포맷, 파서, 스트림 생성기 (스냅샷 → 이벤트 → heartbeat)."""

import json

import pytest

from amr_dashboard import sse
from amr_dashboard.state_store import StateStore


def test_format_sse_basic():
    wire = sse.format_sse('status', {'a': 1, '한글': 'ok'}, event_id=7)
    assert wire == 'id: 7\nevent: status\ndata: {"a":1,"한글":"ok"}\n\n'


def test_format_sse_multiline_and_retry():
    wire = sse.format_sse('note', 'line1\nline2', retry_ms=1500)
    assert wire == 'retry: 1500\nevent: note\ndata: line1\ndata: line2\n\n'
    assert sse.format_sse('empty', '') == 'event: empty\ndata: \n\n'


def test_to_json_rejects_nan():
    with pytest.raises(ValueError):
        sse.to_json({'x': float('nan')})


def test_heartbeat():
    events = sse.parse_sse(sse.heartbeat(ts=12.5))
    assert events == [{'event': 'heartbeat', 'data': '{"ts":12.5}', 'id': None}]


def test_parse_sse_drops_incomplete_tail():
    text = 'id: 1\nevent: a\ndata: x\n\nevent: b\ndata: y'
    assert sse.parse_sse(text) == [{'event': 'a', 'data': 'x', 'id': '1'}]
    assert sse.parse_sse('') == []
    assert sse.parse_sse('data: only\n\n') == [{'event': 'message', 'data': 'only', 'id': None}]


def test_event_stream_snapshot_then_events_then_heartbeat():
    store = StateStore()
    store.set_estop('all', True)  # 구독 전 이벤트는 스냅샷에만 반영된다
    gen = sse.event_stream(store, heartbeat_period=0.05, max_events=3)
    first = sse.parse_sse(next(gen))[0]
    assert first['event'] == 'snapshot' and first['id'] == '1'      # 스냅샷 id = 반영된 seq
    assert json.loads(first['data'])['estops'] == {'all': True}
    assert store.subscriber_count() == 1
    store.broadcast('status', {'k': 1})
    second = sse.parse_sse(next(gen))[0]
    assert second['event'] == 'status' and second['id'] is not None
    third = sse.parse_sse(next(gen))[0]
    assert third['event'] == 'heartbeat'
    with pytest.raises(StopIteration):
        next(gen)
    assert store.subscriber_count() == 0


def test_event_stream_without_snapshot_unsubscribes_on_close():
    store = StateStore()
    gen = sse.event_stream(store, heartbeat_period=0.01, initial_snapshot=False)
    assert sse.parse_sse(next(gen))[0]['event'] == 'heartbeat'
    assert store.subscriber_count() == 1
    gen.close()
    assert store.subscriber_count() == 0


def test_event_stream_skips_events_already_in_snapshot():
    """방어 경로: 스냅샷 seq 이하 이벤트가 큐에 있어도 내보내지 않는다."""
    store = StateStore()

    class Store:
        def subscribe_with_snapshot(self):
            q, snap = store.subscribe_with_snapshot()
            snap['seq'] = 5
            q.put(('old', {}, 5))
            q.put(('new', {'k': 1}, 6))
            return q, snap

        def unsubscribe(self, q):
            store.unsubscribe(q)

    gen = sse.event_stream(Store(), heartbeat_period=0.05, max_events=2)
    assert sse.parse_sse(next(gen))[0]['event'] == 'snapshot'
    ev = sse.parse_sse(next(gen))[0]
    assert ev['event'] == 'new' and ev['id'] == '6'
    gen.close()
    assert store.subscriber_count() == 0
