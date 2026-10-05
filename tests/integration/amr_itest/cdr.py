"""
직렬화된(raw) ROS 2 메시지에서 std_msgs/Header 를 읽는다 (rclpy 비의존 순수 모듈).

640×480 이미지(≈1 MB)를 30 Hz 로 역직렬화하면 부하가 커서 주기 측정이 왜곡된다. 그래서 큰
메시지는 raw=True 로 구독하고, 첫 필드가 Header 인 메시지(Image, CameraInfo, LaserScan, Imu,
Odometry, JointState …)의 스탬프와 frame_id 만 CDR 바이트에서 직접 읽는다.

CDR 배치 (XCDR1, DDS 표준)
  [0:2]  encapsulation id  0x0000 CDR_BE / 0x0001 CDR_LE
  [2:4]  options
  [4:8]  int32  stamp.sec
  [8:12] uint32 stamp.nanosec
  [12:16] uint32 frame_id 길이 (NUL 포함), 이어서 문자열
"""

import struct
from typing import Tuple

_ENCAP_BE = (0x00, 0x00)
_ENCAP_LE = (0x00, 0x01)


def _endian(data: bytes) -> str:
    if len(data) < 4:
        raise ValueError('CDR 버퍼가 너무 짧다')
    encap = (data[0], data[1])
    if encap == _ENCAP_LE:
        return '<'
    if encap == _ENCAP_BE:
        return '>'
    raise ValueError(f'지원하지 않는 encapsulation {encap}')


def header_stamp(data: bytes) -> float:
    """Header 로 시작하는 raw 메시지의 stamp [s]."""
    end = _endian(data)
    if len(data) < 12:
        raise ValueError('CDR 버퍼가 너무 짧다')
    sec, nsec = struct.unpack_from(end + 'iI', data, 4)
    return sec + nsec * 1e-9


def header(data: bytes) -> Tuple[float, str]:
    """Header 로 시작하는 raw 메시지의 (stamp [s], frame_id)."""
    end = _endian(data)
    stamp = header_stamp(data)
    if len(data) < 16:
        raise ValueError('CDR 버퍼가 너무 짧다')
    (length,) = struct.unpack_from(end + 'I', data, 12)
    if length == 0:
        return stamp, ''
    raw = data[16:16 + length]
    if len(raw) < length:
        raise ValueError('frame_id 가 버퍼를 넘는다')
    return stamp, raw.rstrip(b'\x00').decode('utf-8', errors='replace')


def encode_header(sec: int, nanosec: int, frame_id: str, little: bool = True) -> bytes:
    """테스트용: Header 만 담은 CDR 바이트를 만든다 (header() 의 역)."""
    end = '<' if little else '>'
    encap = bytes(_ENCAP_LE if little else _ENCAP_BE) + b'\x00\x00'
    fid = frame_id.encode('utf-8') + b'\x00'
    return encap + struct.pack(end + 'iII', sec, nanosec, len(fid)) + fid
