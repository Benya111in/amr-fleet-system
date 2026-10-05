"""
점유 격자와 8-이웃 A* — 입찰 비용용 경로 길이 추정 (Nav2 를 부르지 않는다, 보고서 54행).

지도(maps/warehouse.yaml + .pgm, 0.05 m)를 cell_m 칸으로 줄이고(칸 안에 점유 화소가 하나라도 있거나
미지 화소가 절반을 넘으면 막힘) inflate_m 만큼 팽창한다. 그 위에서 옥타일 휴리스틱 A* 로 칸 중심 사이
경로 길이를 구하고, 시작·목표 점이 막힌 칸이면 가장 가까운 빈 칸으로 옮기며 옮긴 거리를 더한다.
결과는 (시작 칸, 목표 칸) 쌍으로 기억한다.

이 값은 **추정**이다: 차체 방향·회전 반경·동적 장애물·교통 대기를 모르고, 0.60 m 좁은 통로처럼 칸 크기와
팽창보다 좁은 곳은 막힌 것으로 본다. 입찰 순서를 정하는 데만 쓴다.
"""

from __future__ import annotations

import heapq
import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

Cell = Tuple[int, int]
_SQRT2 = math.sqrt(2.0)


class OccupancyGrid:
    """막힘(True)/빈칸(False) 격자. (0, 0) 칸의 왼쪽 아래 모서리가 origin 이다 (+x 열, +y 행)."""

    def __init__(self, blocked: Sequence[Sequence[bool]], cell_m: float,
                 origin: Tuple[float, float] = (0.0, 0.0)):
        if cell_m <= 0.0:
            raise ValueError('cell_m 은 양수여야 한다')
        self.height = len(blocked)
        self.width = len(blocked[0]) if self.height else 0
        if any(len(row) != self.width for row in blocked):
            raise ValueError('격자 행 길이가 다르다')
        self.cell_m = float(cell_m)
        self.origin = (float(origin[0]), float(origin[1]))
        self._blocked = bytearray(1 if blocked[r][c] else 0
                                  for r in range(self.height) for c in range(self.width))
        self._cache: Dict[Tuple[Cell, Cell], float] = {}
        self.n_searches = 0

    # ------------------------------------------------------------------ 생성
    @classmethod
    def from_rows(cls, rows: Sequence[str], cell_m: float = 1.0,
                  origin: Tuple[float, float] = (0.0, 0.0)) -> 'OccupancyGrid':
        """문자열 행('#' = 막힘)으로 만든다. 첫 문자열이 **위쪽**(+y 끝) 행이다 (지도 그림과 같은 방향)."""
        return cls([[ch == '#' for ch in row] for row in reversed(rows)], cell_m, origin)

    @classmethod
    def from_map_yaml(cls, yaml_path: str, cell_m: float = 0.25,
                      inflate_m: float = 0.25) -> 'OccupancyGrid':
        """map_server 형식 YAML(+ P5 PGM)을 읽어 cell_m 칸으로 줄이고 inflate_m 만큼 팽창한다."""
        meta = _read_map_yaml(yaml_path)
        image = meta['image']
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
        width, height, pixels = read_pgm(image)
        res = float(meta['resolution'])
        origin = (float(meta['origin'][0]), float(meta['origin'][1]))
        negate = int(meta.get('negate', 0))
        occ_t = float(meta.get('occupied_thresh', 0.65))
        free_t = float(meta.get('free_thresh', 0.196))
        factor = max(1, int(round(cell_m / res)))
        cw, ch = width // factor, height // factor
        occ = [[0] * cw for _ in range(ch)]
        unk = [[0] * cw for _ in range(ch)]
        for row in range(height):                 # PGM 첫 행 = 지도 위쪽(+y 끝)
            gy = (height - 1 - row) // factor     # 맨 아래 행이 칸 행 0 (origin 이 왼쪽 아래)
            if gy >= ch:
                continue                          # 위쪽 나머지 행: 칸을 다 채우지 못해 격자 밖
            base = row * width
            o_row, u_row = occ[gy], unk[gy]
            for col in range(cw * factor):
                v = pixels[base + col]
                p = v / 255.0 if negate else (255 - v) / 255.0
                if p > occ_t:
                    o_row[col // factor] += 1
                elif p >= free_t:
                    u_row[col // factor] += 1
        half = factor * factor / 2.0
        blocked = [[occ[r][c] > 0 or unk[r][c] > half for c in range(cw)] for r in range(ch)]
        grid = cls(blocked, res * factor, origin)
        return grid.inflated(inflate_m)

    def inflated(self, radius_m: float) -> 'OccupancyGrid':
        """막힌 칸을 radius_m 원판으로 팽창한 새 격자."""
        r = int(math.ceil(radius_m / self.cell_m - 1e-9))
        if r <= 0:
            return self
        offsets = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
                   if dx * dx + dy * dy <= r * r]
        out = [[False] * self.width for _ in range(self.height)]
        for y in range(self.height):
            for x in range(self.width):
                if not self._blocked[y * self.width + x]:
                    continue
                for dx, dy in offsets:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        out[ny][nx] = True
        return OccupancyGrid(out, self.cell_m, self.origin)

    # ------------------------------------------------------------------ 조회
    def cell_of(self, x: float, y: float) -> Optional[Cell]:
        """월드 좌표 → 칸 (격자 밖이면 None)."""
        cx = int(math.floor((x - self.origin[0]) / self.cell_m))
        cy = int(math.floor((y - self.origin[1]) / self.cell_m))
        if 0 <= cx < self.width and 0 <= cy < self.height:
            return cx, cy
        return None

    def center(self, cell: Cell) -> Tuple[float, float]:
        """칸 중심의 월드 좌표."""
        return (self.origin[0] + (cell[0] + 0.5) * self.cell_m,
                self.origin[1] + (cell[1] + 0.5) * self.cell_m)

    def is_free(self, cell: Cell) -> bool:
        """격자 안의 빈칸인지."""
        cx, cy = cell
        return (0 <= cx < self.width and 0 <= cy < self.height
                and not self._blocked[cy * self.width + cx])

    def is_free_xy(self, x: float, y: float) -> bool:
        """월드 좌표가 빈칸 위인지."""
        cell = self.cell_of(x, y)
        return cell is not None and self.is_free(cell)

    def free_fraction(self) -> float:
        """빈칸 비율 (진단용)."""
        n = len(self._blocked)
        return (n - sum(self._blocked)) / n if n else 0.0

    def nearest_free(self, x: float, y: float, max_radius_m: float = 3.0) -> Optional[Cell]:
        """(x, y) 에서 가장 가까운 빈칸 (칸 중심 기준). max_radius_m 안에 없으면 None."""
        cx = int(math.floor((x - self.origin[0]) / self.cell_m))
        cy = int(math.floor((y - self.origin[1]) / self.cell_m))
        if self.is_free((cx, cy)):
            return cx, cy
        rmax = int(math.ceil(max_radius_m / self.cell_m))
        best, best_d = None, math.inf
        for r in range(1, rmax + 1):
            for dx in range(-r, r + 1):
                for dy in (-r, r) if abs(dx) != r else range(-r, r + 1):
                    cell = (cx + dx, cy + dy)
                    if self.is_free(cell):
                        px, py = self.center(cell)
                        d = math.hypot(px - x, py - y)
                        if d < best_d:
                            best, best_d = cell, d
            if best is not None and best_d <= r * self.cell_m:
                return best
        return best if best_d <= max_radius_m else None

    def path_length(self, start: Tuple[float, float], goal: Tuple[float, float]) -> float:
        """
        시작점 → 목표점 경로 길이 추정 [m] (도달 불가면 inf).

        = |start − 시작 칸 중심| + A*(칸 중심 사이, 8-이웃) + |목표 칸 중심 − goal|.
        """
        s = self.nearest_free(*start)
        g = self.nearest_free(*goal)
        if s is None or g is None:
            return math.inf
        sx, sy = self.center(s)
        gx, gy = self.center(g)
        snap = math.hypot(sx - start[0], sy - start[1]) + math.hypot(goal[0] - gx, goal[1] - gy)
        key = (s, g)
        d = self._cache.get(key)
        if d is None:
            d = self._astar(s, g)
            self._cache[key] = d
        return d + snap

    # ------------------------------------------------------------------ A*
    def _astar(self, s: Cell, g: Cell) -> float:
        self.n_searches += 1
        if s == g:
            return 0.0
        w, h = self.width, self.height
        blocked = self._blocked
        step = self.cell_m
        diag = step * _SQRT2
        gx, gy = g

        def heur(x: int, y: int) -> float:
            dx, dy = abs(x - gx), abs(y - gy)
            return step * (max(dx, dy) - min(dx, dy)) + diag * min(dx, dy)

        start = s[1] * w + s[0]
        goal = gy * w + gx
        best: Dict[int, float] = {start: 0.0}
        heap: List[Tuple[float, float, int]] = [(heur(*s), 0.0, start)]
        nbrs = ((1, 0, step), (-1, 0, step), (0, 1, step), (0, -1, step),
                (1, 1, diag), (1, -1, diag), (-1, 1, diag), (-1, -1, diag))
        while heap:
            _, cost, idx = heapq.heappop(heap)
            if idx == goal:
                return cost
            if cost > best.get(idx, math.inf):
                continue
            x, y = idx % w, idx // w
            for dx, dy, dc in nbrs:
                nx, ny = x + dx, y + dy
                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                n = ny * w + nx
                if blocked[n]:
                    continue
                if dx and dy and (blocked[y * w + nx] or blocked[ny * w + x]):
                    continue                       # 모서리 자르기 금지
                nc = cost + dc
                if nc < best.get(n, math.inf):
                    best[n] = nc
                    heapq.heappush(heap, (nc + heur(nx, ny), nc, n))
        return math.inf


def read_pgm(path: str) -> Tuple[int, int, bytes]:
    """이진 PGM(P5, maxval < 256)을 읽는다 → (너비, 높이, 화소 바이트)."""
    with open(path, 'rb') as fh:
        data = fh.read()
    tokens: List[bytes] = []
    pos = 0
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while pos < len(data) and data[pos:pos + 1] not in (b'\n', b'\r'):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    if tokens[0] != b'P5':
        raise ValueError(f'P5 PGM 이 아니다: {path}')
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval >= 256:
        raise ValueError('16 비트 PGM 은 지원하지 않는다')
    pos += 1                                       # 헤더 끝 공백 1 바이트
    pixels = data[pos:pos + width * height]
    if len(pixels) != width * height:
        raise ValueError(f'PGM 화소 수가 모자란다: {len(pixels)} < {width * height}')
    return width, height, pixels


def _read_map_yaml(path: str) -> dict:
    try:
        import yaml
        with open(path) as fh:
            return yaml.safe_load(fh)
    except ImportError:                            # pyyaml 이 없는 환경: 단순 key: value 만 읽는다
        out: dict = {}
        with open(path) as fh:
            for line in fh:
                line = line.split('#', 1)[0].strip()
                if ':' not in line:
                    continue
                key, val = (s.strip() for s in line.split(':', 1))
                if val.startswith('['):
                    out[key] = [float(v) for v in val.strip('[]').split(',')]
                else:
                    out[key] = val
        return out


__all__ = ['OccupancyGrid', 'read_pgm']
