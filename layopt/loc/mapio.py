"""
점유 지도 읽기 — nav2 map_server(map_io) 와 같은 규칙 (rclpy 비의존).

map_server trinary: p = (255 − v) / 255 (negate 0). p > occupied_thresh → 100, p < free_thresh → 0,
그 밖은 −1. pgm 행 0 = 최대 y 이므로 OccupancyGrid 행 0 = 최소 y 로 뒤집는다.
docs/research/state-estimation/checks/alias_margin.py load_grid 와 같은 결과를 낸다 (시험으로 고정).
"""

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import sys
from typing import Tuple

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'src' / 'amr_localization'))
from amr_localization.scan_map_match import DistanceField, GridSpec  # noqa: E402


@dataclass
class MapData:
    """읽은 지도: 점유 격자(−1/0/100, 행 0 = 최소 y), 메타, 파일 해시."""

    occ: np.ndarray
    spec: GridSpec
    pgm_sha256: str
    yaml_sha256: str
    path: str


def read_pgm(path: Path) -> np.ndarray:
    """P5 pgm → (h, w) uint8 (파일 행 순서 그대로)."""
    data = Path(path).read_bytes()
    if data[:2] != b'P5':
        raise ValueError(f'P5 pgm 이 아니다: {path}')
    tok, i = [], 2
    while len(tok) < 3:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tok.append(int(data[i:j]))
        i = j
    i += 1
    w, h, maxval = tok
    if maxval > 255:
        raise ValueError(f'16 비트 pgm 은 지원하지 않는다: {path}')
    return np.frombuffer(data[i:i + w * h], np.uint8).reshape(h, w)


def load_map(yaml_path) -> MapData:
    """map.yaml(+ image) → MapData (map_server trinary 규칙)."""
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    mode = str(meta.get('mode', 'trinary'))
    if mode != 'trinary':
        raise ValueError(f'mode {mode} 은 지원하지 않는다 (trinary 만): {yaml_path}')
    img_path = Path(meta['image'])
    if not img_path.is_absolute():
        img_path = yaml_path.parent / img_path
    img = read_pgm(img_path)[::-1]
    h, w = img.shape
    if int(meta.get('negate', 0)):
        p = img.astype(np.float64) / 255.0
    else:
        p = (255.0 - img.astype(np.float64)) / 255.0
    occ = np.full((h, w), -1, np.int16)
    occ[p > float(meta['occupied_thresh'])] = 100
    occ[p < float(meta['free_thresh'])] = 0
    ox, oy, oyaw = (float(v) for v in meta['origin'])
    spec = GridSpec(w, h, float(meta['resolution']), ox, oy, oyaw)
    return MapData(occ, spec, hashlib.sha256(img_path.read_bytes()).hexdigest(),
                   hashlib.sha256(yaml_path.read_bytes()).hexdigest(), str(yaml_path))


def distance_field(m: MapData, occupied_thresh: int = 65) -> DistanceField:
    """운용 kidnap_monitor_node.on_map 과 같은 거리장 (occupied_thresh 65)."""
    return DistanceField(m.occ.ravel().tolist(), m.spec, occupied_thresh)


def footprint_cells(spec: GridSpec, pose: Tuple[float, float, float], length: float,
                    width: float) -> Tuple[np.ndarray, np.ndarray]:
    """자세 pose 에 놓인 length × width 직사각형 안에 셀 중심이 드는 셀 (행, 열)."""
    if spec.origin_yaw != 0.0:
        raise ValueError('origin yaw ≠ 0 지도는 지원하지 않는다')
    x, y, yaw = pose
    half = 0.5 * math.hypot(length, width) + spec.resolution
    c0 = int(math.floor((x - half - spec.origin_x) / spec.resolution))
    c1 = int(math.floor((x + half - spec.origin_x) / spec.resolution))
    r0 = int(math.floor((y - half - spec.origin_y) / spec.resolution))
    r1 = int(math.floor((y + half - spec.origin_y) / spec.resolution))
    rr, cc = np.meshgrid(np.arange(max(r0, 0), min(r1, spec.height - 1) + 1),
                         np.arange(max(c0, 0), min(c1, spec.width - 1) + 1), indexing='ij')
    cx = spec.origin_x + (cc + 0.5) * spec.resolution - x
    cy = spec.origin_y + (rr + 0.5) * spec.resolution - y
    c, s = math.cos(yaw), math.sin(yaw)
    lx = c * cx + s * cy
    ly = -s * cx + c * cy
    inside = (np.abs(lx) <= 0.5 * length) & (np.abs(ly) <= 0.5 * width)
    return rr[inside], cc[inside]
