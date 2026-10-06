"""
기하 도우미: 차체 사각형 · 구역 상자 겹침, 꺾은선 경로.

차체는 0.60 × 0.40 m 사각형(중심 = base_link, 진행 방향 = 긴 축)이다. 구역은 traffic_zones.yaml 의 축 정렬
사각형이다 (world 가 축 정렬인지 확인한다). 겹침은 분리축 정리(SAT)로 정확히 판정한다.
"""

from __future__ import annotations

import bisect
import dataclasses
import math
from typing import List, Sequence, Tuple

HALF_LENGTH = 0.30          # 차체 반길이 [m] (0.60 m)
HALF_WIDTH = 0.20           # 차체 반폭 [m] (0.40 m)
CIRCUMRADIUS = math.hypot(HALF_LENGTH, HALF_WIDTH)   # 0.3606 m (traffic.yaml robot_radius 0.36)


@dataclasses.dataclass(frozen=True)
class Box:
    """축 정렬 사각형 [xmin, xmax] × [ymin, ymax]."""

    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def contains(self, x: float, y: float) -> bool:
        """점이 상자 안(경계 포함)인가."""
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax

    def distance(self, x: float, y: float) -> float:
        """점에서 상자까지 거리 (안이면 0)."""
        dx = max(self.xmin - x, 0.0, x - self.xmax)
        dy = max(self.ymin - y, 0.0, y - self.ymax)
        return math.hypot(dx, dy)


def box_of_polygon(poly: Sequence[Tuple[float, float]], tol: float = 1e-9) -> Box:
    """축 정렬 사각형 다각형 → Box. 축 정렬 사각형이 아니면 ValueError."""
    xs = sorted({round(p[0], 9) for p in poly})
    ys = sorted({round(p[1], 9) for p in poly})
    if len(poly) != 4 or len(xs) != 2 or len(ys) != 2:
        raise ValueError(f'축 정렬 사각형이 아니다: {poly}')
    return Box(xs[0], ys[0], xs[1], ys[1])


def circle_box_overlap(cx: float, cy: float, r: float, box: Box) -> bool:
    """원(중심, 반경)과 상자가 겹치나 (경계 접촉 제외)."""
    return box.distance(cx, cy) < r


def rect_corners(cx: float, cy: float, th: float, hl: float = HALF_LENGTH,
                 hw: float = HALF_WIDTH) -> List[Tuple[float, float]]:
    """방향 사각형의 네 꼭짓점."""
    c, s = math.cos(th), math.sin(th)
    out = []
    for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)):
        out.append((cx + a * c - b * s, cy + a * s + b * c))
    return out


def rect_box_overlap(cx: float, cy: float, th: float, box: Box, hl: float = HALF_LENGTH,
                     hw: float = HALF_WIDTH) -> bool:
    """
    방향 사각형(차체)과 축 정렬 상자가 겹치나 — 분리축 정리. 경계 접촉은 겹침이 아니다.

    축 4 개: x, y (상자 축), 차체의 긴 축·짧은 축.
    """
    if box.distance(cx, cy) >= math.hypot(hl, hw):
        return False                                    # 빠른 기각
    corners = rect_corners(cx, cy, th, hl, hw)
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    if max(xs) <= box.xmin or min(xs) >= box.xmax or max(ys) <= box.ymin or min(ys) >= box.ymax:
        return False
    c, s = math.cos(th), math.sin(th)
    bc = ((box.xmin, box.ymin), (box.xmax, box.ymin), (box.xmax, box.ymax), (box.xmin, box.ymax))
    for ax, ay, half in ((c, s, hl), (-s, c, hw)):
        center = cx * ax + cy * ay
        proj = [px * ax + py * ay - center for px, py in bc]
        if max(proj) <= -half or min(proj) >= half:
            return False
    return True


class Path:
    """
    꺾은선 경로. s = 시작점에서 잰 호 길이 [m].

    heading(s) 는 s 가 속한 선분의 방향이다 (꼭짓점에서는 다음 선분). 꼭짓점 목록과 꺾임 각은 운동학이 쓴다.
    """

    def __init__(self, points: Sequence[Tuple[float, float]]):
        pts: List[Tuple[float, float]] = []
        for p in points:
            q = (float(p[0]), float(p[1]))
            if not pts or math.hypot(q[0] - pts[-1][0], q[1] - pts[-1][1]) > 1e-9:
                pts.append(q)
        if not pts:
            raise ValueError('빈 경로')
        self.points: Tuple[Tuple[float, float], ...] = tuple(pts)
        cum = [0.0]
        heads = []
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            cum.append(cum[-1] + math.hypot(x1 - x0, y1 - y0))
            heads.append(math.atan2(y1 - y0, x1 - x0))
        self.cum: Tuple[float, ...] = tuple(cum)
        self.headings: Tuple[float, ...] = tuple(heads)
        self.length = cum[-1]

    def segment(self, s: float) -> int:
        """호 길이 s 가 속한 선분 번호 (0 .. n-2). 점 하나짜리 경로는 -1."""
        if len(self.points) < 2:
            return -1
        i = bisect.bisect_right(self.cum, s) - 1
        return min(max(i, 0), len(self.points) - 2)

    def point(self, s: float) -> Tuple[float, float]:
        """호 길이 s 의 점 (경로 밖이면 끝점)."""
        if len(self.points) < 2 or s <= 0.0:
            return self.points[0]
        if s >= self.length:
            return self.points[-1]
        i = self.segment(s)
        seg = self.cum[i + 1] - self.cum[i]
        f = (s - self.cum[i]) / seg if seg > 0 else 0.0
        (x0, y0), (x1, y1) = self.points[i], self.points[i + 1]
        return x0 + f * (x1 - x0), y0 + f * (y1 - y0)

    def heading(self, s: float, default: float = 0.0) -> float:
        """호 길이 s 에서의 진행 방향 [rad] (점 하나짜리 경로는 default)."""
        i = self.segment(s)
        return default if i < 0 else self.headings[i]

    def turn_angles(self) -> List[float]:
        """꼭짓점 k(1 .. n-2) 에서의 꺾임 각 |Δθ| [rad] — 목록 번호 k-1."""
        return [abs(wrap(b - a)) for a, b in zip(self.headings, self.headings[1:])]


def wrap(a: float) -> float:
    """각을 (-π, π] 로."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


__all__ = ['Box', 'CIRCUMRADIUS', 'HALF_LENGTH', 'HALF_WIDTH', 'Path', 'box_of_polygon',
           'circle_box_overlap', 'rect_box_overlap', 'rect_corners', 'wrap']
