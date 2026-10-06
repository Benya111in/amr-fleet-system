"""
월드 SDF 의 정적 충돌 상자 → 2D 사각형과, 로봇 풋프린트의 부호 거리·직진 접근 거리 (rclpy 비의존).

규칙은 `rl/isaac/amr_isaac/warehouse_geom.py` (Isaac 학습 환경이 쓰는 정적 충돌체) 와 같다 — 같은 월드에서 같은
사각형이 나와야 Isaac·Gazebo 의 충돌·여유 판정이 같은 기하를 본다 (`test/test_world_geom.py` 가 대조한다):
  - 대상: 월드 최상위 <model> 과 <include>(model://X → models/X/model.sdf) 의 <collision> 상자.
    자세 = include/모델 자세 ∘ 링크 자세 ∘ 충돌체 자세 (SDF x y z roll pitch yaw, 고정축 XYZ = Rz·Ry·Rx).
  - 제외: <plane>, <actor>, 움직이는 모델(이름이 forklift·shuttle_amr·worker 로 시작).
  - 상자가 아닌 충돌체는 실패한다 (조용히 빼지 않는다).
  - 2D 장애물 = 연직 범위가 로봇 몸체 띠 [0.02, 0.33] m 와 겹치는 상자의 바닥 사각형 (cx, cy, hx, hy, yaw).
풋프린트 = base_link 중심 0.60 × 0.40 m 사각형 (config/robot_params.yaml footprint, 계약 §2).
직진 접근 거리는 A3 재판정(`feature/b1-isaac-phi` 의 `rl/isaac/amr_isaac/eval_metrics.py` straight_approach)과
같은 정의다 — 완성 계획 §6 의 주 집합 규칙 "경로 끝 접선 자세에서 풋프린트 앞 끝 직진 여유 ≥ 0.40 m".
"""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np

MOVING_MODELS = ('forklift', 'shuttle_amr', 'worker')
Z_BAND_LO = 0.02
Z_BAND_HI = 0.33
HALF_L, HALF_W = 0.30, 0.20
CIRCUMRADIUS = math.hypot(HALF_L, HALF_W)


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y),
                              math.sin(y))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _pose(text):
    t = np.eye(4)
    if text:
        v = [float(x) for x in text.split()] + [0.0] * 6
        t[:3, :3] = _rpy(v[3], v[4], v[5])
        t[:3, 3] = v[:3]
    return t


def _boxes(model, base):
    out = []
    for link in model.findall('link'):
        lt = base @ _pose(link.findtext('pose'))
        for col in link.findall('collision'):
            geom = col.find('geometry')
            if geom is None or len(geom) == 0:
                continue
            shape = geom[0]
            if shape.tag == 'plane':
                continue
            if shape.tag != 'box':
                raise ValueError(f'상자가 아닌 충돌체 {shape.tag}: {model.get("name")}')
            ct = lt @ _pose(col.findtext('pose'))
            size = np.array([float(x) for x in shape.findtext('size').split()])
            out.append((ct, size))
    return out


def load_obb(sdf_path, models_dir, z_lo=Z_BAND_LO, z_hi=Z_BAND_HI):
    """월드 SDF → 몸체 띠와 겹치는 정적 상자의 2D 사각형 [M, 5] = (cx, cy, hx, hy, yaw)."""
    world = ET.parse(sdf_path).getroot().find('world')
    boxes = []
    for m in world.findall('model'):
        if any(m.get('name', '').startswith(k) for k in MOVING_MODELS):
            continue
        boxes += _boxes(m, _pose(m.findtext('pose')))
    for inc in world.findall('include'):
        uri = (inc.findtext('uri') or '').strip()
        if not uri.startswith('model://'):
            raise ValueError(f'지원하지 않는 uri {uri}')
        name = uri[len('model://'):]
        if name in MOVING_MODELS:
            continue
        base = _pose(inc.findtext('pose'))
        for m in ET.parse(os.path.join(models_dir, name, 'model.sdf')).getroot().iter('model'):
            boxes += _boxes(m, base)     # include 자세가 모델 자세를 대체한다 (SDF include 규칙)
    rows = []
    sx = np.array([-1, 1, 1, -1, -1, 1, 1, -1], dtype=float)
    sy = np.array([-1, -1, 1, 1, -1, -1, 1, 1], dtype=float)
    sz = np.array([-1, -1, -1, -1, 1, 1, 1, 1], dtype=float)
    for t, size in boxes:
        h = 0.5 * size
        corners = np.stack([sx * h[0], sy * h[1], sz * h[2]], 1) @ t[:3, :3].T + t[:3, 3]
        lo, hi = corners[:, 2].min(), corners[:, 2].max()
        if hi < z_lo or lo > z_hi:
            continue
        if abs(abs(t[2, 2]) - 1.0) > 1e-6:
            raise ValueError('기울어진 상자')
        rows.append([t[0, 3], t[1, 3], h[0], h[1], math.atan2(t[1, 0], t[0, 0])])
    return np.asarray(rows, dtype=np.float64).reshape(-1, 5)


def rect_corners(x, y, hx, hy, yaw):
    """사각형 꼭짓점 [4, 2] (반시계)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([(x + c * a - s * b, y + s * a + c * b)
                     for a, b in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))])


def _point_rect_signed(px, py, cx, cy, hx, hy, yaw):
    """점들 → 사각형 부호 거리 (안쪽 음수 = −가장 가까운 변까지)."""
    c, s = np.cos(yaw), np.sin(yaw)
    dx, dy = px - cx, py - cy
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    ex = np.abs(lx) - hx
    ey = np.abs(ly) - hy
    out = np.hypot(np.maximum(ex, 0.0), np.maximum(ey, 0.0))
    inside = (ex <= 0) & (ey <= 0)
    return np.where(inside, np.maximum(ex, ey), out)


def _sat_overlap(a, b):
    """두 볼록 사각형(꼭짓점 [4,2])의 최소 겹침 폭 (분리되면 0 이하)."""
    best = math.inf
    for poly in (a, b):
        for i in range(4):
            e = poly[(i + 1) % 4] - poly[i]
            n = np.array([-e[1], e[0]]) / max(math.hypot(e[0], e[1]), 1e-12)
            pa, pb = a @ n, b @ n
            o = min(pa.max(), pb.max()) - max(pa.min(), pb.min())
            if o <= 0.0:
                return o
            best = min(best, o)
    return best


class StaticWorld:
    """정적 사각형 묶음 + 근방 거르기 (자세마다 가까운 사각형만 정확히 잰다)."""

    def __init__(self, obb):
        """사각형 묶음 obb [M, 5] = (cx, cy, hx, hy, yaw)."""
        self.obb = np.asarray(obb, dtype=np.float64).reshape(-1, 5)
        self.rad = np.hypot(self.obb[:, 2], self.obb[:, 3])

    @classmethod
    def from_sdf(cls, sdf_path, models_dir):
        """SDF 에서 읽는다."""
        return cls(load_obb(sdf_path, models_dir))

    def _near(self, x, y, margin):
        d = np.hypot(self.obb[:, 0] - x, self.obb[:, 1] - y) - self.rad
        return np.nonzero(d <= margin)[0]

    def point_clearance(self, x, y, horizon=5.0):
        """점 → 가장 가까운 사각형 부호 거리 (horizon 안에 없으면 horizon)."""
        idx = self._near(x, y, horizon)
        if idx.size == 0:
            return horizon
        o = self.obb[idx]
        return float(np.min(_point_rect_signed(x, y, o[:, 0], o[:, 1], o[:, 2], o[:, 3], o[:, 4])))

    def footprint_clearance(self, x, y, yaw, horizon=3.0, half_l=HALF_L, half_w=HALF_W):
        """
        풋프린트(자세 x, y, yaw) ↔ 사각형들 최소 부호 거리 [m]: 떨어져 있으면 거리(> 0), 겹치면 −최소 겹침 폭.

        horizon 안에 사각형이 없으면 horizon. 떨어진 두 볼록 다각형의 최소 거리는 한쪽 꼭짓점과 다른 쪽 사이에서 난다.
        """
        idx = self._near(x, y, horizon + math.hypot(half_l, half_w))
        if idx.size == 0:
            return horizon
        fp = rect_corners(x, y, half_l, half_w, yaw)
        best = horizon
        for o in self.obb[idx]:
            oc = rect_corners(*o)
            ov = _sat_overlap(fp, oc)
            if ov > 0.0:
                best = min(best, -ov)
                continue
            d1 = _point_rect_signed(fp[:, 0], fp[:, 1], *o).min()
            d2 = _point_rect_signed(oc[:, 0], oc[:, 1], x, y, half_l, half_w, yaw).min()
            best = min(best, float(min(d1, d2)))
        return best

    def straight_approach(self, x, y, yaw, max_dist=5.0, half_l=HALF_L, half_w=HALF_W):
        """
        자세에서 풋프린트를 앞(+x)으로 직진시킬 때 사각형에 처음 닿기까지 거리 (없으면 inf, 이미 겹치면 0).

        A3 eval_metrics.straight_approach 와 같은 정의 (safety_gate 스윕 풋프린트 직진 접근 거리의 기하값).
        """
        c, s = math.cos(yaw), math.sin(yaw)
        own = rect_corners(0.0, 0.0, half_l, half_w, 0.0)
        best = math.inf
        for o in self.obb[self._near(x, y, max_dist + math.hypot(half_l, half_w))]:
            oc = rect_corners(*o)
            dx, dy = oc[:, 0] - x, oc[:, 1] - y
            loc = np.stack([c * dx + s * dy, -s * dx + c * dy], 1)
            if _sat_overlap(own, loc) > 0.0:
                return 0.0
            ok = (np.abs(loc[:, 1]) <= half_w) & (loc[:, 0] >= half_l)
            if ok.any():
                best = min(best, float((loc[ok, 0] - half_l).min()))
            for cy in (-half_w, half_w):
                for i in range(4):
                    p0, p1 = loc[i], loc[(i + 1) % 4]
                    if (p0[1] - cy) * (p1[1] - cy) > 0 or p0[1] == p1[1]:
                        continue
                    t = (cy - p0[1]) / (p1[1] - p0[1])
                    xi = p0[0] + t * (p1[0] - p0[0])
                    if xi >= half_l:
                        best = min(best, float(xi - half_l))
        return best if best <= max_dist else math.inf
