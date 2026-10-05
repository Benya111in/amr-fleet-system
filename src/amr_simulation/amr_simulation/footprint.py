"""
2D 발자국(footprint) 기하 — 충돌 판정용 부호 거리.

부호 거리(signed distance): 두 도형이 떨어져 있으면 최단 거리(> 0), 겹치면 −(가장 얕은 침투 깊이)(< 0).
명세 4.7 "동적 장애물 회피 30회 충돌 0건" 판정은 이 값이 0 미만이 되는 순간을 충돌로 센다 (collision_monitor_node).
도형: 방향 있는 직사각형(로봇·지게차·셔틀, 몸체 원점 기준 [x_min, x_max, y_min, y_max]) 과 원(사람).
"""

import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Rect:
    """몸체 자세 (x, y, yaw) + 몸체 좌표 경계 [x_min, x_max, y_min, y_max]."""

    x: float
    y: float
    yaw: float
    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def corners(self) -> List[Tuple[float, float]]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        pts = [(self.x_min, self.y_min), (self.x_max, self.y_min),
               (self.x_max, self.y_max), (self.x_min, self.y_max)]
        return [(self.x + c * px - s * py, self.y + s * px + c * py) for px, py in pts]

    def to_local(self, px: float, py: float) -> Tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = px - self.x, py - self.y
        return c * dx + s * dy, -s * dx + c * dy


def point_rect(px: float, py: float, r: Rect) -> float:
    """점 → 직사각형 부호 거리 (안쪽 음수)."""
    lx, ly = r.to_local(px, py)
    dx = max(r.x_min - lx, 0.0, lx - r.x_max)
    dy = max(r.y_min - ly, 0.0, ly - r.y_max)
    if dx > 0.0 or dy > 0.0:
        return math.hypot(dx, dy)
    return -min(lx - r.x_min, r.x_max - lx, ly - r.y_min, r.y_max - ly)


def rect_circle(r: Rect, cx: float, cy: float, radius: float) -> float:
    return point_rect(cx, cy, r) - radius


def _seg_point(ax, ay, bx, by, px, py) -> float:
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 == 0.0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
    return math.hypot(ax + t * vx - px, ay + t * vy - py)


def _axes(r: Rect):
    c, s = math.cos(r.yaw), math.sin(r.yaw)
    return [(c, s), (-s, c)]


def rect_rect(a: Rect, b: Rect) -> float:
    """직사각형 ↔ 직사각형 부호 거리 (분리축 정리: 겹치면 최소 겹침 폭의 음수)."""
    ca, cb = a.corners(), b.corners()
    overlap = math.inf
    for ax, ay in _axes(a) + _axes(b):
        pa = [x * ax + y * ay for x, y in ca]
        pb = [x * ax + y * ay for x, y in cb]
        o = min(max(pa), max(pb)) - max(min(pa), min(pb))
        if o <= 0.0:
            overlap = None
            break
        overlap = min(overlap, o)
    if overlap is not None:
        return -overlap
    d = math.inf
    for poly, other in ((ca, cb), (cb, ca)):
        for i in range(4):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % 4]
            for px, py in other:
                d = min(d, _seg_point(x1, y1, x2, y2, px, py))
    return d
