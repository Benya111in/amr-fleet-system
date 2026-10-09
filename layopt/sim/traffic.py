"""
실제 TrafficManager(src/amr_fleet/amr_fleet/traffic_manager.py) 연결.

교통 관리 본체는 고치지 않고 그대로 부른다 (update(now, observations) → TickResult). 바꾼 것은 속도 하나다:
- FastZoneMap: 축 정렬 사각형 구역의 점 → 구역 조회를 벡터로 한다. 원본 ZoneMap.zones_at 의 ray casting 이
  축 정렬 사각형에서 정확히 반열린 상자 [xmin, xmax) × [ymin, ymax) 가 된다는 것을 이용한다 (사각형이 아닌 구역 ·
  지도 유도 라벨은 원본 경로). 결과가 원본과 같다는 것은 tests/test_traffic.py 가 무작위 · 경계 점으로 확인한다.
  프로파일: 5 대 주행 시 update 5.8 ms 중 4.5 ms 가 points_in_polygon 이었다.
- E1TrafficManager: TrafficManager 를 상속해 생성 · set_map 뒤 zone_map 만 FastZoneMap 으로 바꾼다.

설정은 src/amr_fleet/config/traffic.yaml 을 그대로 읽는다
(노드 전용 키 update_rate_hz · map_topic · frame_id 만 뺀다).
zones_file 은 배치의 traffic_zones.yaml.
"""

from __future__ import annotations

import pathlib
from typing import Dict, List, Optional, Sequence

import numpy as np
import yaml

from layopt.sim import REPO, ensure_paths

ensure_paths()
from amr_fleet.traffic_geometry import points_in_polygon  # noqa: E402
from amr_fleet.traffic_manager import TrafficConfig, TrafficManager  # noqa: E402
from amr_fleet.traffic_zones import Zone, ZoneMap  # noqa: E402

TRAFFIC_YAML = REPO / 'src' / 'amr_fleet' / 'config' / 'traffic.yaml'
UPDATE_RATE_HZ = 2.0            # traffic.yaml update_rate_hz
NODE_ONLY = ('update_rate_hz', 'map_topic', 'frame_id')


def _rect_of(poly: np.ndarray) -> Optional[tuple]:
    """꼭짓점 4 개 축 정렬 사각형이면 (xmin, xmax, ymin, ymax), 아니면 None."""
    if poly is None or len(poly) != 4:
        return None
    xs = sorted({float(v) for v in poly[:, 0]})
    ys = sorted({float(v) for v in poly[:, 1]})
    if len(xs) != 2 or len(ys) != 2:
        return None
    for i in range(4):
        a, b = poly[i], poly[(i + 1) % 4]
        if (a[0] != b[0]) == (a[1] != b[1]):        # 변은 축 하나만 바뀌어야 한다
            return None
    return xs[0], xs[1], ys[0], ys[1]


class FastZoneMap(ZoneMap):
    """축 정렬 사각형 구역의 zones_at 을 벡터로 (원본과 같은 결과)."""

    def __init__(self, polygon_zones: Sequence[Zone] = ()):
        super().__init__(polygon_zones)
        rects = [_rect_of(z.polygon) for z in self._polys]
        self._fast = all(r is not None for r in rects) and len(rects) > 0
        if self._fast:
            a = np.array(rects, dtype=float)
            self._xmin, self._xmax, self._ymin, self._ymax = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
            self._ids = [z.zone_id for z in self._polys]

    def zones_at(self, xy: np.ndarray) -> List[Optional[str]]:
        if not self._fast:
            return super().zones_at(xy)
        pts = np.asarray(xy, dtype=float).reshape(-1, 2)
        x = pts[:, 0:1]
        y = pts[:, 1:2]
        inside = (x >= self._xmin) & (x < self._xmax) & (y >= self._ymin) & (y < self._ymax)
        hit = inside.any(axis=1)
        first = inside.argmax(axis=1)                  # 먼저 정의한 구역이 이긴다
        ids = self._ids
        return [ids[f] if h else None for f, h in zip(first.tolist(), hit.tolist())]


def fast_zone_map(zm: ZoneMap) -> ZoneMap:
    """지도 유도 라벨이 없으면 FastZoneMap 으로 바꾼다."""
    if zm._labels is not None or isinstance(zm, FastZoneMap):
        return zm
    return FastZoneMap(zm._polys)


class E1TrafficManager(TrafficManager):
    """TrafficManager 그대로 + zone_map 만 FastZoneMap."""

    def __init__(self, config: Optional[TrafficConfig] = None, zones=None, pockets=None):
        super().__init__(config, zones, pockets)
        self.zone_map = fast_zone_map(self.zone_map)

    def set_map(self, occupied: np.ndarray, spec) -> None:
        super().set_map(occupied, spec)
        self.zone_map = fast_zone_map(self.zone_map)


def load_config(zones_file, traffic_yaml: pathlib.Path = TRAFFIC_YAML,
                overrides: Optional[Dict[str, object]] = None) -> TrafficConfig:
    """traffic.yaml → TrafficConfig (zones_file = 배치의 traffic_zones.yaml)."""
    doc = yaml.safe_load(pathlib.Path(traffic_yaml).read_text(encoding='utf-8'))
    params = doc['/fleet/traffic_manager_node']['ros__parameters']
    flat: Dict[str, object] = {}

    def walk(prefix: str, d: dict) -> None:
        for k, v in d.items():
            key = f'{prefix}.{k}' if prefix else k
            if isinstance(v, dict):
                walk(key, v)
            else:
                flat[key] = v

    walk('', params)
    for k in NODE_ONLY:
        flat.pop(k, None)
    flat['zones_file'] = str(zones_file)
    if overrides:
        flat.update(overrides)
    return TrafficConfig.from_flat(flat)


def points_in_polygon_ref(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """시험용: 원본 ray casting."""
    return points_in_polygon(points, polygon)


__all__ = ['E1TrafficManager', 'FastZoneMap', 'TRAFFIC_YAML', 'UPDATE_RATE_HZ', 'fast_zone_map',
           'load_config']
