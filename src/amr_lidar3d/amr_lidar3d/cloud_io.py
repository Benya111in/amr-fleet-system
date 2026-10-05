"""
PointCloud2 ↔ numpy, CDR 머리 읽기, 자세 계산 (rclpy 비의존 — 측정 스크립트와 pytest 가 같이 쓴다).

cloud_fields(msg) / cloud_array(msg) 는 sensor_msgs/PointCloud2 와 같은 속성
(fields, point_step, row_step, width, height, data, is_bigendian)을 가진 아무 객체나 받는다.
"""

from __future__ import annotations

import math
import struct
from typing import Dict, List, Tuple

from amr_lidar3d.world_gt import rpy_matrix
import numpy as np

# sensor_msgs/PointField datatype → numpy
_DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
_NAMES = {1: 'INT8', 2: 'UINT8', 3: 'INT16', 4: 'UINT16', 5: 'INT32', 6: 'UINT32',
          7: 'FLOAT32', 8: 'FLOAT64'}


def cloud_fields(msg) -> List[Dict]:
    """필드 목록 [{name, offset, datatype(이름), count}]."""
    return [{'name': f.name, 'offset': int(f.offset), 'datatype': _NAMES.get(f.datatype, '?'),
             'count': int(f.count)} for f in msg.fields]


def cloud_array(msg) -> np.ndarray:
    """PointCloud2 → 구조화 배열 [height * width] (행 패딩은 건너뛴다)."""
    order = '>' if msg.is_bigendian else '<'
    dtype = np.dtype({'names': [f.name for f in msg.fields],
                      'formats': [order + _DTYPES[f.datatype] for f in msg.fields],
                      'offsets': [int(f.offset) for f in msg.fields],
                      'itemsize': int(msg.point_step)})
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    rows = []
    for r in range(int(msg.height)):
        start = r * int(msg.row_step)
        row = buf[start:start + int(msg.width) * int(msg.point_step)]
        rows.append(row.view(dtype))
    return np.concatenate(rows) if rows else np.zeros(0, dtype=dtype)


def xyz(arr: np.ndarray) -> np.ndarray:
    """구조화 배열 → [N, 3] float64."""
    return np.column_stack([arr['x'], arr['y'], arr['z']]).astype(float)


def cdr_stamp(raw: bytes) -> float:
    """
    직렬화된 메시지(첫 필드가 std_msgs/Header)의 header.stamp [s].

    CDR: 4 B 캡슐화 머리 + int32 sec + uint32 nanosec (리틀엔디언 캡슐화 0x0001 만).
    큰 점군을 역직렬화하지 않고 주기·지연만 잴 때 쓴다.
    """
    if len(raw) < 12 or raw[1] != 1:
        raise ValueError('CDR 리틀엔디언 Header 메시지가 아니다')
    sec, nsec = struct.unpack_from('<iI', raw, 4)
    return sec + nsec * 1e-9


def quat_yaw(x: float, y: float, z: float, w: float) -> float:
    """쿼터니언 → yaw [rad]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """쿼터니언 → 회전 행렬."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def wrap(angle: np.ndarray) -> np.ndarray:
    """각도를 [−π, π) 로."""
    return (np.asarray(angle) + math.pi) % (2.0 * math.pi) - math.pi


def interval_stats(times: List[float]) -> Dict:
    """시각 열 → 개수, 평균 주기로 본 주파수, 간격 중앙값·p95·최대 (2 개 미만이면 개수만)."""
    t = np.asarray(sorted(times), dtype=float)
    if t.size < 2:
        return {'n': int(t.size)}
    dt = np.diff(t)
    span = float(t[-1] - t[0])
    return {'n': int(t.size), 'span_s': span,
            'hz': float((t.size - 1) / span) if span > 0 else float('inf'),
            'dt_median': float(np.median(dt)), 'dt_p95': float(np.percentile(dt, 95)),
            'dt_max': float(dt.max())}


def transform(points: np.ndarray, rot: np.ndarray, trans: np.ndarray) -> np.ndarray:
    """점 [N, 3] 에 (R, t) 적용."""
    return np.asarray(points) @ np.asarray(rot).T + np.asarray(trans)


def pose_from_xyz_rpy(xyz_: Tuple[float, float, float], rpy: Tuple[float, float, float]) \
        -> Tuple[np.ndarray, np.ndarray]:
    """(x, y, z), (r, p, y) → (R, t) — URDF origin 규약 (Rz·Ry·Rx)."""
    return rpy_matrix(*rpy), np.asarray(xyz_, dtype=float)
