"""
창고 정적 충돌체 변환기 — warehouse.sdf (읽기 전용) → 상자 목록 → 삼각형 메시·2D 사각형·점유 격자.

Isaac Sim 에 의존하지 않는다 (표준 라이브러리 + numpy). USD 로 옮기는 일은 usd_scene.py 가 한다.

규칙
  - 대상: 월드 최상위 <model> 과 <include>(model://X → src/amr_simulation/models/X/model.sdf) 의
    <collision> 상자. 자세 = include/모델 자세 ∘ 링크 자세 ∘ 충돌체 자세 (SDF x y z roll pitch yaw,
    고정축 XYZ = Rz·Ry·Rx).
  - 제외: <plane>(바닥), <actor>(작업자), 움직이는 모델 MOVING_MODELS (지게차·셔틀 — 월드 안에서 왕복하는
    동적 장애물이다, gen_warehouse_world.py 머리말), 충돌체가 없는 모델(표지판·패드).
  - 상자 말고 다른 형상(cylinder·mesh·sphere)이 나오면 실패한다 (지금 창고에는 없다 — 조용히 빼지 않는다).
  - 2D 장애물 = 연직 범위가 로봇 몸체 높이 띠 [Z_BAND_LO, Z_BAND_HI] 와 겹치는 상자.
"""

from dataclasses import dataclass
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

MOVING_MODELS = ('forklift', 'shuttle_amr', 'worker')
Z_BAND_LO = 0.02        # [m] 차체 바닥 (바닥 마커·패드 같은 높이 0 판은 충돌체가 없다)
Z_BAND_HI = 0.33        # [m] 데크 상면 (config/sensors.yaml: 데크 지면 +0.33 m)
LIDAR_Z = 0.20          # [m] 스캔 평면 = 지면 +0.20 (sensors.yaml lidar: base_link_height 0.18 + z 0.02)


@dataclass
class Box:
    """세계 좌표 상자 하나."""

    name: str
    center: np.ndarray      # (3,)
    size: np.ndarray        # (3,) 전체 길이
    rot: np.ndarray         # (3, 3)

    def z_range(self):
        """연직 범위 (회전 포함 꼭짓점 기준)."""
        c = self.corners()
        return float(c[:, 2].min()), float(c[:, 2].max())

    def corners(self):
        """꼭짓점 8 개 (세계 좌표)."""
        h = 0.5 * self.size
        sx = np.array([-1, 1, 1, -1, -1, 1, 1, -1], dtype=np.float64)
        sy = np.array([-1, -1, 1, 1, -1, -1, 1, 1], dtype=np.float64)
        sz = np.array([-1, -1, -1, -1, 1, 1, 1, 1], dtype=np.float64)
        local = np.stack([sx * h[0], sy * h[1], sz * h[2]], axis=1)
        return local @ self.rot.T + self.center

    def is_planar(self, tol=1e-6):
        """roll·pitch 가 0 (z 축이 세계 z 와 평행) 인가."""
        return abs(abs(self.rot[2, 2]) - 1.0) < tol


def default_paths():
    """저장소 안의 기본 SDF·모델 경로 (rl/isaac/amr_isaac 기준)."""
    repo = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                         '..', '..', '..'))
    sim = os.path.join(repo, 'src', 'amr_simulation')
    return os.path.join(sim, 'worlds', 'warehouse.sdf'), os.path.join(sim, 'models')


def rpy_matrix(r, p, y):
    """고정축 XYZ 회전 (SDF/URDF 규약) = Rz(y)·Ry(p)·Rx(r)."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def pose_matrix(text):
    """'x y z r p y' → 4×4 (없으면 단위)."""
    t = np.eye(4)
    if text is None or not text.strip():
        return t
    v = [float(x) for x in text.split()]
    if len(v) != 6:
        raise ValueError(f'자세 형식 오류: {text!r}')
    t[:3, :3] = rpy_matrix(v[3], v[4], v[5])
    t[:3, 3] = v[:3]
    return t


def _collision_boxes(model_el, base, prefix):
    """<model> 요소의 충돌 상자들 (base = 모델의 세계 자세 4×4)."""
    out = []
    for link in model_el.findall('link'):
        lt = base @ pose_matrix(link.findtext('pose'))
        for col in link.findall('collision'):
            ct = lt @ pose_matrix(col.findtext('pose'))
            geom = col.find('geometry')
            if geom is None or len(geom) == 0:
                continue
            shape = geom[0]
            name = f'{prefix}/{link.get("name")}/{col.get("name")}'
            if shape.tag == 'plane':
                continue
            if shape.tag != 'box':
                raise ValueError(f'상자가 아닌 충돌체 {shape.tag}: {name}')
            size = np.array([float(x) for x in shape.findtext('size').split()])
            out.append(Box(name, ct[:3, 3].copy(), size, ct[:3, :3].copy()))
    return out


def load_boxes(sdf_path=None, models_dir=None):
    """warehouse.sdf → 정적 충돌 상자 목록 (제외 규칙은 모듈 머리말)."""
    d_sdf, d_models = default_paths()
    sdf_path = sdf_path or d_sdf
    models_dir = models_dir or d_models
    world = ET.parse(sdf_path).getroot().find('world')
    boxes = []
    for m in world.findall('model'):
        name = m.get('name')
        if any(name.startswith(k) for k in MOVING_MODELS):
            continue
        boxes += _collision_boxes(m, pose_matrix(m.findtext('pose')), name)
    for inc in world.findall('include'):
        uri = inc.findtext('uri').strip()
        if not uri.startswith('model://'):
            raise ValueError(f'지원하지 않는 uri {uri}')
        mname = uri[len('model://'):]
        if mname in MOVING_MODELS:
            continue
        inc_t = pose_matrix(inc.findtext('pose'))
        msdf = ET.parse(os.path.join(models_dir, mname, 'model.sdf')).getroot()
        for m in msdf.iter('model'):
            # include 자세가 모델 자세를 대체한다 (SDF include 규칙)
            boxes += _collision_boxes(m, inc_t, inc.findtext('name') or mname)
    return boxes


def triangle_mesh(boxes):
    """상자들 → (정점 [8n, 3] float32, 삼각형 [12n, 3] int32). 면은 바깥 법선."""
    faces = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                      [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]],
                     dtype=np.int32)
    verts, tris = [], []
    for i, b in enumerate(boxes):
        verts.append(b.corners())
        tris.append(faces + 8 * i)
    return np.concatenate(verts).astype(np.float32), np.concatenate(tris).astype(np.int32)


def obstacles_2d(boxes, z_lo=Z_BAND_LO, z_hi=Z_BAND_HI):
    """
    로봇 몸체 높이 띠와 겹치는 상자의 바닥 사각형 [n, 5] = (cx, cy, hx, hy, yaw).

    roll·pitch 가 0 이 아니면 실패한다 (바닥 사각형이 정의되지 않는다).
    """
    rows = []
    for b in boxes:
        lo, hi = b.z_range()
        if hi < z_lo or lo > z_hi:
            continue
        if not b.is_planar():
            raise ValueError(f'기울어진 상자 {b.name}')
        yaw = math.atan2(b.rot[1, 0], b.rot[0, 0])
        rows.append([b.center[0], b.center[1], 0.5 * b.size[0], 0.5 * b.size[1], yaw])
    return np.asarray(rows, dtype=np.float64).reshape(-1, 5)


def rasterize(obb, res=0.05, bounds=(-30.5, 30.5, -20.5, 20.5), pad=None):
    """
    2D 사각형들 → 점유 격자 (bool [ny, nx], 행 = y). 셀 중심이 사각형(+pad) 안이면 점유.

    pad 기본 = res/2 (얇은 판 0.02 m 도 최소 한 셀). 반환 (grid, origin(x0, y0), res).
    """
    if pad is None:
        pad = 0.5 * res
    x0, x1, y0, y1 = bounds
    nx = int(round((x1 - x0) / res))
    ny = int(round((y1 - y0) / res))
    xs = x0 + (np.arange(nx) + 0.5) * res
    ys = y0 + (np.arange(ny) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)
    grid = np.zeros((ny, nx), dtype=bool)
    for cx, cy, hx, hy, yaw in obb:
        r = math.hypot(hx, hy) + pad
        ix0 = max(0, int((cx - r - x0) / res))
        ix1 = min(nx, int((cx + r - x0) / res) + 2)
        iy0 = max(0, int((cy - r - y0) / res))
        iy1 = min(ny, int((cy + r - y0) / res) + 2)
        if ix0 >= ix1 or iy0 >= iy1:
            continue
        dx = gx[iy0:iy1, ix0:ix1] - cx
        dy = gy[iy0:iy1, ix0:ix1] - cy
        c, s = math.cos(yaw), math.sin(yaw)
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        grid[iy0:iy1, ix0:ix1] |= (np.abs(lx) <= hx + pad) & (np.abs(ly) <= hy + pad)
    return grid, (x0, y0), res


def footprint_hits_obb(pose, obb, half_l, half_w):
    """
    풋프린트 사각형(세계 자세 pose (x, y, θ)) 이 사각형들 중 하나와 겹치는가 (분리축 정리, 경계 포함).

    numpy 기준판 — env 는 같은 식의 torch 판을 쓴다 (시험에서 대조).
    """
    x, y, th = pose
    a_axes = np.array([[math.cos(th), math.sin(th)], [-math.sin(th), math.cos(th)]])
    for cx, cy, hx, hy, yaw in obb:
        b_axes = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
        d = np.array([cx - x, cy - y])
        sep = False
        for ax in np.concatenate([a_axes, b_axes]):
            ra = half_l * abs(ax @ a_axes[0]) + half_w * abs(ax @ a_axes[1])
            rb = hx * abs(ax @ b_axes[0]) + hy * abs(ax @ b_axes[1])
            if abs(ax @ d) > ra + rb:
                sep = True
                break
        if not sep:
            return True
    return False
