"""
SLAM 점유 격자 → 월드 정렬 격자, map_server 형식 쓰기·읽기, 점유 일치율 (rclpy 비의존).

SLAM 행의 map 프레임(`slam_map`)은 오도메트리 원점 = 스폰 자세에서 시작한다. 평가 계약은 "map = 월드"
(보고서 「3D LiDAR SLAM 도입 방안」 지표 절 — 정렬 없는 SE(2) ATE)이므로, 재생 때 `map → slam_map` 고정 TF 를
스폰 자세로 주고, 격자는 이 모듈로 월드 축에 맞춰 다시 표본화해 저장한다 (map_server 는 YAML origin 의 yaw 를
무시하므로 회전된 격자를 그대로 쓰면 Nav2·평가 도구가 틀린 곳에 놓는다).

격자 값 규약은 nav_msgs/OccupancyGrid 와 같다: −1 미지, 0..100 점유 확률. 배열 행 0 = 최하단 y.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, Tuple

from amr_slam3d.se2 import compose, Pose2
import numpy as np

UNKNOWN = -1


@dataclass
class OccGrid:
    """점유 격자 (OccupancyGrid 와 같은 배치). origin = 셀 (0, 0) 모서리의 자세 (자기 프레임)."""

    data: np.ndarray            # int16 [rows, cols], −1 / 0..100
    resolution: float
    origin: Pose2               # (x, y, yaw) — 격자 프레임에서 셀 (0,0) 모서리

    @property
    def shape(self) -> Tuple[int, int]:
        """(행, 열)."""
        return tuple(self.data.shape)


def from_msg_fields(width: int, height: int, resolution: float, origin: Pose2,
                    values) -> OccGrid:
    """메시지 필드(OccupancyGrid) → OccGrid (values 는 행 우선 int8 목록)."""
    arr = np.asarray(values, dtype=np.int16).reshape(int(height), int(width))
    return OccGrid(arr, float(resolution), origin)


def to_world(grid: OccGrid, world_T_frame: Pose2) -> OccGrid:
    """
    격자를 월드 축 정렬로 다시 표본화한다 (최근접 셀, 해상도 그대로).

    출력 셀 중심을 원래 격자로 역변환해 가장 가까운 셀 값을 가져온다. 회전이 π/2 의 정수배에 가까우면
    (스폰 yaw 1.5708 처럼) 셀 중심이 원래 셀 중심에 0.1 mm 안으로 떨어져 값이 그대로 옮겨진다.
    """
    res = grid.resolution
    w_T_g = compose(world_T_frame, grid.origin)            # 월드 ← 격자 모서리
    rows, cols = grid.shape
    corners = np.array([[0, 0], [cols * res, 0], [0, rows * res], [cols * res, rows * res]])
    c, s = math.cos(w_T_g[2]), math.sin(w_T_g[2])
    wx = w_T_g[0] + c * corners[:, 0] - s * corners[:, 1]
    wy = w_T_g[1] + s * corners[:, 0] + c * corners[:, 1]
    # 월드 격자 경계를 해상도 격자점에 맞춘다. 스폰 yaw 1.5708 은 π/2 와 3.7e-6 rad 다르므로 모서리가 격자점에서
    # 셀의 1e-4 쯤 벗어난다 — snap(셀 1/1000) 안이면 격자점으로 본다 (아니면 한 셀씩 늘어난다)
    snap = 1e-3
    x0 = math.floor(wx.min() / res + snap) * res
    y0 = math.floor(wy.min() / res + snap) * res
    ncols = int(math.ceil(wx.max() / res - snap)) - int(round(x0 / res))
    nrows = int(math.ceil(wy.max() / res - snap)) - int(round(y0 / res))
    out = np.full((nrows, ncols), UNKNOWN, dtype=np.int16)
    jj, ii = np.meshgrid(np.arange(ncols), np.arange(nrows))
    px = x0 + (jj + 0.5) * res - w_T_g[0]
    py = y0 + (ii + 0.5) * res - w_T_g[1]
    gx = c * px + s * py                                    # 격자 프레임 좌표 (모서리 기준)
    gy = -s * px + c * py
    src_c = np.floor(gx / res).astype(np.int64)
    src_r = np.floor(gy / res).astype(np.int64)
    ok = (src_c >= 0) & (src_c < cols) & (src_r >= 0) & (src_r < rows)
    out[ok] = grid.data[src_r[ok], src_c[ok]]
    return OccGrid(out, res, (x0, y0, 0.0))


def trinary_pixels(data: np.ndarray, free_thresh: float = 0.19,
                   occupied_thresh: float = 0.65) -> np.ndarray:
    """
    점유 값 → PGM 화소 (nav2 map_saver trinary 규약).

    값 < 0 또는 > 100 → 205(미지), ≤ free·100 → 254(자유), ≥ occ·100 → 0(점유), 그 사이 → 205.
    문턱은 저장소 map_saver_server 설정(free 0.19, occupied 0.65)과 같다.
    """
    free_i = int(round(free_thresh * 100))
    occ_i = int(round(occupied_thresh * 100))
    px = np.full(data.shape, 205, dtype=np.uint8)
    valid = (data >= 0) & (data <= 100)
    px[valid & (data <= free_i)] = 254
    px[valid & (data >= occ_i)] = 0
    return px


def write_map(stem: str, grid: OccGrid, free_thresh: float = 0.19,
              occupied_thresh: float = 0.65, comment: str = '') -> Tuple[str, str]:
    """map_server 형식 (<stem>.pgm + <stem>.yaml). 원점 yaw 는 0 이어야 한다."""
    if abs(grid.origin[2]) > 1e-9:
        raise ValueError('월드 정렬(to_world) 뒤에만 쓴다 — map_server 는 origin yaw 를 무시한다')
    px = trinary_pixels(grid.data, free_thresh, occupied_thresh)[::-1]   # 영상 행 0 = 최상단
    pgm = Path(stem + '.pgm')
    yml = Path(stem + '.yaml')
    rows, cols = px.shape
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{cols} {rows}\n255\n'.encode('ascii'))
        f.write(np.ascontiguousarray(px).tobytes())
    lines = [f'image: {pgm.name}', 'mode: trinary', f'resolution: {grid.resolution:g}',
             f'origin: [{grid.origin[0]:.6f}, {grid.origin[1]:.6f}, 0]', 'negate: 0',
             f'occupied_thresh: {occupied_thresh:g}', f'free_thresh: {free_thresh:g}']
    if comment:
        lines += [f'# {line}' for line in comment.splitlines()]
    yml.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return str(pgm), str(yml)


def read_map(yaml_path: str) -> OccGrid:
    """map_server YAML(+PGM) → OccGrid (점유 100 / 자유 0 / 미지 −1, trinary·negate 0 만)."""
    import yaml  # 지연 import: 순수 계산 함수만 쓰는 시험이 yaml 없이도 돈다
    meta = yaml.safe_load(Path(yaml_path).read_text(encoding='utf-8'))
    if int(meta.get('negate', 0)) != 0:
        raise ValueError('negate 1 은 지원하지 않는다')
    img = read_pgm(str(Path(yaml_path).parent / meta['image']))[::-1]   # 행 0 = 최하단
    occ_t = float(meta.get('occupied_thresh', 0.65))
    free_t = float(meta.get('free_thresh', 0.19))
    p = (255.0 - img.astype(float)) / 255.0
    data = np.full(img.shape, UNKNOWN, dtype=np.int16)
    data[p > occ_t] = 100
    data[p < free_t] = 0
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    oyaw = float(meta['origin'][2]) if len(meta['origin']) > 2 else 0.0
    return OccGrid(data, float(meta['resolution']), (ox, oy, oyaw))


def read_pgm(path: str) -> np.ndarray:
    """P5 8 bit PGM → uint8 배열 (행 0 = 영상 위)."""
    raw = Path(path).read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':
            pos = raw.index(b'\n', pos) + 1
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        tokens.append(raw[pos:end].decode('ascii'))
        pos = end
    if tokens[0] != 'P5' or int(tokens[3]) > 255:
        raise ValueError(f'{path}: 8 bit P5 PGM 만 지원')
    cols, rows = int(tokens[1]), int(tokens[2])
    pos += 1                                          # 최대값 뒤 공백 한 바이트
    return np.frombuffer(raw[pos:pos + cols * rows], dtype=np.uint8).reshape(rows, cols)


def _dilate(mask: np.ndarray, r: int) -> np.ndarray:
    """정사각 창 (2r+1)² 이진 팽창 (scipy 없이)."""
    if r <= 0:
        return mask.copy()
    rows, cols = mask.shape
    pad = np.pad(mask, r)
    out = np.zeros_like(mask)
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            out |= pad[r + di:r + di + rows, r + dj:r + dj + cols]
    return out


def occupied_agreement(est: OccGrid, ref: OccGrid, tol_cells: int = 2) -> Dict[str, float]:
    """
    점유 일치율 — 월드 정렬 확인용 거친 지표 (정식 지표는 amr_evaluation.slam3d.gridmap).

    precision: 추정 점유 셀 중 tol_cells 안에 참조 점유 셀이 있는 비율.
    recall_observed: 추정이 아는 셀(자유·점유)을 tol 만큼 넓힌 영역 안의 참조 점유 셀 중, tol 안에 추정 점유가
    있는 비율. 두 격자는 해상도가 같고 원점 yaw 0 이어야 한다 (원점 차이는 정수 셀로 맞춘다).
    """
    if abs(est.resolution - ref.resolution) > 1e-9 or est.origin[2] or ref.origin[2]:
        raise ValueError('해상도가 같고 원점 yaw 가 0 인 격자만 비교한다')
    res = est.resolution
    off_c = int(round((est.origin[0] - ref.origin[0]) / res))
    off_r = int(round((est.origin[1] - ref.origin[1]) / res))
    rows, cols = ref.shape
    e_occ = np.zeros((rows, cols), dtype=bool)
    e_known = np.zeros((rows, cols), dtype=bool)
    er, ec = np.nonzero(est.data >= 0)
    rr, cc = er + off_r, ec + off_c
    inside = (rr >= 0) & (rr < rows) & (cc >= 0) & (cc < cols)
    vals = est.data[er[inside], ec[inside]]
    e_known[rr[inside], cc[inside]] = True
    e_occ[rr[inside], cc[inside]] = vals >= 65
    outside_occ = int(np.count_nonzero(est.data[er[~inside], ec[~inside]] >= 65))
    r_occ = ref.data >= 65
    n_est = int(np.count_nonzero(e_occ)) + outside_occ
    hit = e_occ & _dilate(r_occ, tol_cells)
    observed = _dilate(e_known, tol_cells)
    r_obs = r_occ & observed
    found = r_obs & _dilate(e_occ, tol_cells)
    return {'n_est_occupied': float(n_est),
            'precision': float(np.count_nonzero(hit)) / n_est if n_est else float('nan'),
            'n_ref_observed': float(np.count_nonzero(r_obs)),
            'recall_observed': (float(np.count_nonzero(found)) / np.count_nonzero(r_obs)
                                if np.count_nonzero(r_obs) else float('nan')),
            'tol_m': tol_cells * res}
