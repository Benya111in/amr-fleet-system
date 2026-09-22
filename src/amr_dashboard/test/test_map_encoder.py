"""map_encoder: OccupancyGrid → PNG(그레이, 행 뒤집기) / 런렝스 JSON."""

import array
import struct
import zlib

import pytest

from amr_dashboard import map_encoder as me


def test_grid_bytes_from_various_containers():
    assert me.grid_bytes([0, 100, -1]) == b'\x00\x64\xff'
    assert me.grid_bytes(array.array('b', [0, 100, -1])) == b'\x00\x64\xff'
    assert me.grid_bytes(b'\x01\x02') == b'\x01\x02'
    assert me.grid_bytes(bytearray(b'\x03')) == b'\x03'
    assert me.grid_bytes((5, 6)) == b'\x05\x06'
    numpy = pytest.importorskip('numpy')
    assert me.grid_bytes(numpy.array([0, -1], dtype=numpy.int8)) == b'\x00\xff'


def test_gray_table_endpoints():
    assert me.OCCUPANCY_TO_GRAY[0] == 255          # 자유 → 흰색
    assert me.OCCUPANCY_TO_GRAY[100] == 0          # 점유 → 검정
    assert me.OCCUPANCY_TO_GRAY[0xFF] == 128       # -1 미지 → 회색
    assert me.OCCUPANCY_TO_GRAY[127] == 0          # 100 초과는 점유로 클램프
    assert 0 < me.OCCUPANCY_TO_GRAY[50] < 255
    assert me.OCCUPANCY_TO_GRAY[30] > me.OCCUPANCY_TO_GRAY[70]


def test_png_roundtrip_and_row_flip():
    width, height = 4, 3
    data = [0] * 12
    data[0] = 100      # 행 0 (원점, 남쪽) 첫 셀 점유
    data[11] = -1      # 행 2 (북쪽) 마지막 셀 미지
    data[5] = 50
    png = me.grid_to_png(width, height, data)
    assert png.startswith(me.PNG_SIGNATURE)
    w, h, rows = me.decode_png_gray(png)
    assert (w, h) == (width, height)
    assert len(rows) == height and all(len(r) == width for r in rows)
    # PNG 첫 행 = 격자 마지막 행 (북쪽)
    assert rows[0][3] == 128
    assert rows[2][0] == 0
    assert rows[1][1] == me.OCCUPANCY_TO_GRAY[50]
    assert rows[0][0] == 255


def test_png_chunks_have_valid_crc():
    png = me.grid_to_png(2, 2, [0, 0, 0, 0])
    pos = len(me.PNG_SIGNATURE)
    tags = []
    while pos < len(png):
        length, = struct.unpack('>I', png[pos:pos + 4])
        tag = png[pos + 4:pos + 8]
        payload = png[pos + 8:pos + 8 + length]
        crc, = struct.unpack('>I', png[pos + 8 + length:pos + 12 + length])
        assert crc == zlib.crc32(tag + payload) & 0xFFFFFFFF
        tags.append(tag)
        pos += 12 + length
    assert tags == [b'IHDR', b'IDAT', b'IEND']


def test_png_size_mismatch_rejected():
    with pytest.raises(ValueError):
        me.grid_to_png(3, 3, [0] * 8)
    with pytest.raises(ValueError):
        me.grid_to_png(0, 0, [])


def test_decode_rejects_non_png():
    with pytest.raises(ValueError):
        me.decode_png_gray(b'GIF89a')


def test_rle():
    assert me.grid_to_rle([0, 0, 0, 100, 100, -1]) == [[0, 3], [100, 2], [-1, 1]]
    assert me.grid_to_rle([]) == []
    assert me.grid_to_rle(array.array('b', [7])) == [[7, 1]]


def test_large_grid_encodes_quickly():
    width, height = 1200, 800   # 60 x 40 m @ 0.05 m
    data = bytes([0] * (width * height))
    png = me.grid_to_png(width, height, data)
    w, h, _ = me.decode_png_gray(png)
    assert (w, h) == (width, height)
    assert len(png) < 20_000  # 균일 지도는 잘 압축된다
