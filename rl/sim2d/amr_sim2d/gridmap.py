"""
점유격자 지도 — 팀 지도(maps/warehouse.pgm·yaml, 읽기 전용) 또는 생성 지도.

규약 (nav2 map_server / costmap 과 같게):
  - 셀 (mx, my) 의 my = 0 이 아래 (PGM 의 마지막 행). 셀 (0,0) 의 좌하단 모서리 = origin.
  - trinary: p = (255 − v)/255 (negate 0). p > occupied_thresh → 점유, p < free_thresh → 자유,
    그 밖 → 미지.
  - 시뮬레이터의 **물리 세계** = 점유 ∪ 미지 ∪ 지도 밖 (미지 셀은 SLAM 이 못 본 랙 내부·기둥·벽 뒤다.
    2026-10-06 분석: 미지 23 688 셀 중 주 자유 영역에 맞닿은 것은 374 셀 [계산]).
  - A* 비용 격자 = 점유·미지 → 254 (LETHAL), 자유 → 0, 그 뒤 전역 코스트맵의 정적 팽창
    (inflate(0.20, 1.2, 2.0): nav2_params.yaml global_costmap inflation_layer, 내접 반경 = 풋프린트 반폭).
"""

import math
import os

import numpy as np
import torch
import yaml

from ._contract import REPO_DIR

DEFAULT_MAP = os.path.join(REPO_DIR, 'maps', 'warehouse.yaml')
GLOBAL_INFLATION = {'inscribed_radius': 0.20, 'inflation_radius': 1.2, 'cost_scaling_factor': 2.0}


def read_pgm(path):
    """P5(8 비트) PGM → uint8 [rows, cols] (행 0 = 위)."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens = []
    i = 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b'P5':
        raise ValueError(f'P5 PGM 이 아니다: {tokens[0]!r}')
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError('16 비트 PGM 은 지원하지 않는다')
    i += 1   # 헤더 뒤 공백 한 바이트
    return np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)


class GridMap:
    """
    물리 세계 격자 + A* 비용 격자.

    solid: uint8 [H, W] (my 축 아래→위), 1 = 막힘. resolution, origin (x0, y0).
    """

    def __init__(self, solid, resolution, origin, name='map', free_known=None):
        """막힘 격자 solid [H,W] bool/uint8 (행 = my), resolution [m], origin (x, y)."""
        self.solid = np.ascontiguousarray(solid.astype(np.uint8))
        self.H, self.W = self.solid.shape
        self.res = float(resolution)
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.name = name
        self.free_known = free_known if free_known is not None else (self.solid == 0)
        self._edt = None
        self._cost = None

    # ------------------------------------------------------------ 읽기
    @staticmethod
    def from_yaml(path=DEFAULT_MAP):
        """map_server YAML (+PGM) → GridMap."""
        with open(path, encoding='utf-8') as f:
            meta = yaml.safe_load(f)
        img = read_pgm(os.path.join(os.path.dirname(path), meta['image']))
        v = img.astype(np.float64)
        p = v / 255.0 if int(meta.get('negate', 0)) else (255.0 - v) / 255.0
        occ = p > float(meta['occupied_thresh'])
        free = p < float(meta['free_thresh'])
        n_occ, n_unk = int(occ.sum()), int((~occ & ~free).sum())
        solid = (~free)[::-1]                  # 점유 ∪ 미지, 행 0 = my 0 (아래)
        gm = GridMap(solid, meta['resolution'], meta['origin'][:2],
                     name=os.path.splitext(os.path.basename(path))[0],
                     free_known=np.ascontiguousarray(free[::-1]))
        gm.n_occupied, gm.n_unknown = n_occ, n_unk
        return gm

    # ------------------------------------------------------------ 좌표
    def world_to_cell(self, x, y):
        """월드 [m] → 셀 (내림)."""
        return (np.floor((np.asarray(x) - self.ox) / self.res).astype(np.int64),
                np.floor((np.asarray(y) - self.oy) / self.res).astype(np.int64))

    def cell_center(self, mx, my):
        """셀 중심 월드 좌표."""
        return (self.ox + (np.asarray(mx) + 0.5) * self.res,
                self.oy + (np.asarray(my) + 0.5) * self.res)

    @property
    def extent(self):
        """(x0, x1, y0, y1) [m]."""
        return (self.ox, self.ox + self.W * self.res, self.oy, self.oy + self.H * self.res)

    # ------------------------------------------------------------ 파생 격자
    @property
    def edt(self):
        """자유 셀 중심에서 가장 가까운 막힌 셀 **경계**까지의 거리 하한 [m] (EDT − 반 셀)."""
        if self._edt is None:
            from scipy import ndimage
            pad = np.pad(self.solid == 0, 1, constant_values=False)   # 지도 밖 = 막힘
            d = ndimage.distance_transform_edt(pad)[1:-1, 1:-1] * self.res
            self._edt = np.where(self.solid == 0, np.maximum(d - 0.5 * self.res, 0.0), 0.0)
        return self._edt

    def cost_grid(self, inflation=GLOBAL_INFLATION):
        """경로 계획용 비용 격자 (LETHAL 254 + 정적 팽창). dwa_binding 모듈이 필요하다."""
        if self._cost is None:
            from .binding import core_module
            m = core_module()
            cells = np.where(self.solid != 0, m.LETHAL, 0).astype(np.uint8)
            self._cost = m.inflate(cells, self.res, inflation['inscribed_radius'],
                                   inflation['inflation_radius'], inflation['cost_scaling_factor'])
        return self._cost

    def to_torch(self, device):
        """막힘 격자 uint8 텐서 (raycast·충돌용)."""
        return torch.as_tensor(self.solid, device=device).contiguous()


def stack_maps(maps, device):
    """
    여러 지도를 [M, H, W] 텐서 하나로 (가장 큰 크기로 막힘 채움) + 지도별 메타 [M,4] (res, ox, oy, W·H).

    해상도는 모두 같아야 한다 (계약 §10: 0.05 m).
    """
    res = {m.res for m in maps}
    if len(res) != 1:
        raise ValueError(f'해상도가 다르다: {res}')
    H = max(m.H for m in maps)
    W = max(m.W for m in maps)
    t = torch.ones(len(maps), H, W, dtype=torch.uint8)
    meta = torch.zeros(len(maps), 4, dtype=torch.float64)
    for i, m in enumerate(maps):
        t[i, :m.H, :m.W] = torch.as_tensor(m.solid)
        meta[i] = torch.tensor([m.res, m.ox, m.oy, 0.0], dtype=torch.float64)
    return t.to(device).contiguous(), meta.to(device), (H, W), float(next(iter(res)))


def map_rect_world(m, x0, x1, y0, y1, value=1):
    """월드 좌표 축정렬 사각형의 셀(셀 중심 포함 기준)을 value 로 (생성기·시험용)."""
    mx0 = int(math.ceil((x0 - m.ox) / m.res - 0.5))
    mx1 = int(math.floor((x1 - m.ox) / m.res - 0.5))
    my0 = int(math.ceil((y0 - m.oy) / m.res - 0.5))
    my1 = int(math.floor((y1 - m.oy) / m.res - 0.5))
    mx0, my0 = max(mx0, 0), max(my0, 0)
    mx1, my1 = min(mx1, m.W - 1), min(my1, m.H - 1)
    if mx1 >= mx0 and my1 >= my0:
        m.solid[my0:my1 + 1, mx0:mx1 + 1] = value
        m._edt = None
        m._cost = None
