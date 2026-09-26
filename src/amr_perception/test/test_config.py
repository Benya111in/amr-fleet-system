"""config/perception.yaml 이 ROS 2 파라미터 파서 규약을 지키는지 (colcon test)."""

from pathlib import Path

import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'perception.yaml'


def _params(doc):
    """{노드 키: {파라미터: 값}} 로 펼친다 (중첩 사전 포함)."""
    for node, body in doc.items():
        if not isinstance(body, dict) or 'ros__parameters' not in body:
            continue
        stack = [(node, body['ros__parameters'])]
        while stack:
            prefix, d = stack.pop()
            for k, v in d.items():
                if isinstance(v, dict):
                    stack.append((f'{prefix}.{k}', v))
                else:
                    yield f'{prefix}.{k}', v


def test_sequences_are_homogeneous():
    """
    배열 파라미터는 원소 타입이 모두 같아야 한다.

    rcl 의 파서는 섞인 배열을 거부하고 **파일 전체**를 못 읽는다 — 그러면 그 파일을 쓰는 노드가
    전부 기동하지 못한다. 실제로 [10, 0.30, 11, 0.30, ...] 처럼 정수와 실수를 섞었다가
    "Sequence should be of same type. Value type 'double' do not belong" 로 인지 노드가 모두
    죽었다 (통합 08 b8: 검출기·마커 노드 사망, 시행 23/30, 목표 도달 11).
    """
    doc = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    mixed = []
    for name, value in _params(doc):
        if not isinstance(value, list) or not value:
            continue
        kinds = {'bool' if isinstance(v, bool) else type(v).__name__ for v in value}
        if len(kinds) > 1:
            mixed.append(f'{name}: {sorted(kinds)}')
    assert not mixed, f'배열 원소 타입이 섞였다 (rcl 이 파일 전체를 거부한다): {mixed}'


def test_marker_size_by_id_pairs_are_valid():
    """marker_size_by_id 는 [id, 크기] 쌍이고, 크기는 양수여야 한다."""
    doc = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    flat = dict(_params(doc)).get('/**/aruco_detector_node.marker_size_by_id')
    if flat is None:
        pytest.skip('marker_size_by_id 가 없다')
    assert len(flat) % 2 == 0, f'[id, 크기] 쌍이어야 한다 ({len(flat)} 개)'
    for i in range(0, len(flat), 2):
        mid, size = flat[i], flat[i + 1]
        assert float(mid).is_integer() and float(mid) >= 0, f'id 가 이상하다: {mid}'
        assert 0.0 < float(size) < 2.0, f'마커 크기가 이상하다: id {mid} → {size} m'
