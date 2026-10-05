"""
점 지도 파일 읽기·쓰기·좌표 변환 (rclpy 비의존) — SLAM 행의 3D 지도를 월드 프레임 PLY 로 맞춘다.

- RTAB-Map: rtabmap-export --cloud 가 slam_map 프레임 PLY (binary_little_endian, PCL) 를 낸다.
- LIO-SAM: save_map 서비스가 liosam_odom 프레임 PCD (DATA binary, PointXYZI) 를 낸다.
둘 다 재생 때 기록한 world ← 지도 프레임 TF 로 옮겨 x y z float32 binary_little_endian PLY 로 쓴다
(amr_evaluation.slam3d.cloud.read_ply 가 읽는 형식).
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np

_PLY_TYPES = {'char': 'i1', 'int8': 'i1', 'uchar': 'u1', 'uint8': 'u1', 'short': 'i2',
              'int16': 'i2', 'ushort': 'u2', 'uint16': 'u2', 'int': 'i4', 'int32': 'i4',
              'uint': 'u4', 'uint32': 'u4', 'float': 'f4', 'float32': 'f4', 'double': 'f8',
              'float64': 'f8'}
_PCD_TYPES = {('F', 4): 'f4', ('F', 8): 'f8', ('U', 1): 'u1', ('U', 2): 'u2', ('U', 4): 'u4',
              ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}


def read_ply_xyz(path: str) -> np.ndarray:
    """binary_little_endian PLY 의 vertex x y z → (N, 3) float64 (다른 스칼라 속성은 건너뛴다)."""
    with open(path, 'rb') as f:
        header = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f'{path}: end_header 가 없다')
            text = line.decode('ascii').strip()
            header.append(text)
            if text == 'end_header':
                break
        body = f.read()
    if 'format binary_little_endian 1.0' not in header:
        raise ValueError(f'{path}: binary_little_endian PLY 만 지원')
    fields, count, in_vertex = [], 0, False
    for text in header:
        parts = text.split()
        if parts[:1] == ['element']:
            if in_vertex:
                break                                   # vertex 다음 원소(면 등)는 무시
            in_vertex = parts[1] == 'vertex'
            count = int(parts[2]) if in_vertex else count
        elif parts[:1] == ['property'] and in_vertex:
            if parts[1] == 'list':
                raise ValueError(f'{path}: vertex list 속성은 지원하지 않는다')
            fields.append((parts[2], '<' + _PLY_TYPES[parts[1]]))
    arr = np.frombuffer(body, dtype=np.dtype(fields), count=count)
    return np.column_stack([arr['x'], arr['y'], arr['z']]).astype(float)


def read_pcd_xyz(path: str) -> np.ndarray:
    """PCD (DATA ascii | binary) 의 x y z → (N, 3) float64. binary_compressed 는 지원하지 않는다."""
    raw = Path(path).read_bytes()
    head, pos = {}, 0
    while True:
        end = raw.index(b'\n', pos)
        line = raw[pos:end].decode('ascii').strip()
        pos = end + 1
        if not line or line.startswith('#'):
            continue
        key, _, rest = line.partition(' ')
        head[key.upper()] = rest.split()
        if key.upper() == 'DATA':
            break
    names = head['FIELDS']
    sizes = [int(s) for s in head['SIZE']]
    types = head['TYPE']
    counts = [int(c) for c in head.get('COUNT', ['1'] * len(names))]
    npts = int(head['POINTS'][0])
    mode = head['DATA'][0]
    if mode == 'ascii':
        vals = np.loadtxt(raw[pos:].decode('ascii').splitlines(), ndmin=2)
        idx = {n: sum(counts[:i]) for i, n in enumerate(names)}
        return vals[:, [idx['x'], idx['y'], idx['z']]].astype(float)
    if mode != 'binary':
        raise ValueError(f'{path}: DATA {mode} 는 지원하지 않는다')
    fields = []
    for n, s, t, c in zip(names, sizes, types, counts):
        base = '<' + _PCD_TYPES[(t, s)]
        fields.append((n if n != '_' else f'_pad{len(fields)}', base if c == 1 else (base, (c,))))
    arr = np.frombuffer(raw[pos:], dtype=np.dtype(fields), count=npts)
    return np.column_stack([arr['x'], arr['y'], arr['z']]).astype(float)


def write_ply_xyz(path: str, xyz: np.ndarray) -> None:
    """float32 x y z binary_little_endian PLY."""
    pts = np.asarray(xyz, dtype='<f4').reshape(-1, 3)
    head = ['ply', 'format binary_little_endian 1.0', f'element vertex {len(pts)}',
            'property float x', 'property float y', 'property float z', 'end_header']
    with open(path, 'wb') as f:
        f.write(('\n'.join(head) + '\n').encode('ascii'))
        f.write(np.ascontiguousarray(pts).tobytes())


def quat_matrix(q: Sequence[float]) -> np.ndarray:
    """쿼터니언 (x, y, z, w) → 3×3 회전 행렬 (정규화 포함)."""
    x, y, z, w = (float(v) for v in q)
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def transform(xyz: np.ndarray, t: Sequence[float], q: Sequence[float]) -> np.ndarray:
    """점 (N, 3) 에 변환 (t, q) 적용: p' = R·p + t (자식 프레임 점 → 부모 프레임)."""
    return np.asarray(xyz, dtype=float) @ quat_matrix(q).T + np.asarray(t, dtype=float)


def finite(xyz: np.ndarray) -> np.ndarray:
    """비유한 점 제거."""
    return xyz[np.all(np.isfinite(xyz), axis=1)]
