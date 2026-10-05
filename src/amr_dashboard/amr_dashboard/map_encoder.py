"""
nav_msgs/OccupancyGrid 셀 데이터 → PNG / 런렝스 JSON 인코더 (순수 파이썬, 표준 라이브러리만).

값 규약 (OccupancyGrid): -1 미지, 0 자유, 100 점유, 1~99 점유 확률.
PNG 는 8-bit 그레이스케일: 자유 255(흰색) → 점유 0(검정), 미지 128(회색).
ROS 격자는 행 0 이 원점(남쪽)이고 PNG 는 위에서 아래로 저장하므로 행을 뒤집어 넣는다.
따라서 클라이언트는 이미지를 (origin_x, origin_y + height*resolution) 을 좌상단으로 그대로 그리면 된다.
"""

import itertools
import struct
import zlib

PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'
UNKNOWN_GRAY = 128


def _build_gray_table() -> bytes:
    table = bytearray(256)
    for u in range(256):
        v = u - 256 if u >= 128 else u
        if v < 0:
            table[u] = UNKNOWN_GRAY
        elif v >= 100:
            table[u] = 0
        else:
            table[u] = 255 - int(round(v * 2.55))
    return bytes(table)


# int8 셀 값(부호 없는 바이트로 본 것) → 회색 밝기
OCCUPANCY_TO_GRAY = _build_gray_table()


def grid_bytes(data) -> bytes:
    """OccupancyGrid.data(int8 시퀀스: array('b'), numpy int8, list, bytes) → 부호 없는 바이트열."""
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    try:
        mv = memoryview(data)
    except TypeError:
        mv = None
    if mv is not None and mv.ndim == 1 and mv.format in ('b', 'B'):
        return mv.cast('B').tobytes()
    return bytes((int(v) & 0xFF) for v in data)


def png_chunk(tag: bytes, payload: bytes) -> bytes:
    """PNG 청크 하나 (길이 + 태그 + 데이터 + CRC32)."""
    crc = zlib.crc32(tag + payload) & 0xFFFFFFFF
    return struct.pack('>I', len(payload)) + tag + payload + struct.pack('>I', crc)


def grid_to_png(width: int, height: int, data, compress_level: int = 6) -> bytes:
    """nav_msgs/OccupancyGrid → 8-bit 그레이스케일 PNG bytes (행을 뒤집어 북쪽이 위)."""
    raw = grid_bytes(data)
    if width <= 0 or height <= 0 or len(raw) != width * height:
        raise ValueError(f'grid size mismatch: {width}x{height} vs {len(raw)} cells')
    gray = raw.translate(OCCUPANCY_TO_GRAY)
    scanlines = bytearray()
    for row in range(height - 1, -1, -1):
        scanlines.append(0)  # 필터 타입 None
        scanlines += gray[row * width:(row + 1) * width]
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    return (PNG_SIGNATURE
            + png_chunk(b'IHDR', ihdr)
            + png_chunk(b'IDAT', zlib.compress(bytes(scanlines), compress_level))
            + png_chunk(b'IEND', b''))


def grid_to_rle(data) -> list:
    """nav_msgs/OccupancyGrid → [[value, count], ...] 행 우선 런렝스 (value 는 -1..100 원래 값)."""
    raw = grid_bytes(data)
    runs = []
    for u, group in itertools.groupby(raw):
        v = u - 256 if u >= 128 else u
        runs.append([v, sum(1 for _ in group)])
    return runs


def decode_png_gray(png: bytes):
    """
    grid_to_png 가 만든 PNG 를 (width, height, rows[bytes]) 로 되돌린다 (검증용, 필터 None 전제).

    일반 PNG 디코더가 아니다: 이 모듈이 만든 8-bit 그레이스케일·필터 0 이미지만 다룬다.
    """
    if not png.startswith(PNG_SIGNATURE):
        raise ValueError('not a PNG')
    pos = len(PNG_SIGNATURE)
    width = height = 0
    idat = b''
    while pos < len(png):
        length, = struct.unpack('>I', png[pos:pos + 4])
        tag = png[pos + 4:pos + 8]
        payload = png[pos + 8:pos + 8 + length]
        pos += 12 + length
        if tag == b'IHDR':
            width, height = struct.unpack('>II', payload[:8])
        elif tag == b'IDAT':
            idat += payload
    scan = zlib.decompress(idat)
    stride = width + 1
    rows = [bytes(scan[r * stride + 1:(r + 1) * stride]) for r in range(height)]
    return width, height, rows
