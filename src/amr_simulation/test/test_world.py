"""
월드 생성 재현성 + 월드 내용 검사 (colcon test).

- gen_warehouse_world.py 를 다시 돌린 결과가 커밋된 warehouse.sdf 와 바이트 단위로 같다 (생성기만 고치고 SDF 를
  안 고치거나, 반대로 SDF 를 손으로 고치는 것을 잡는다). 생성기의 check_layout(통로 폭·경로 충돌)도 함께 돈다.
- 벽 안쪽 치수 ≥ 60 x 40, 도킹 마커 높이 = 로봇 카메라 광학 중심 높이 (config/*.yaml), 표지판 텍스처 존재·비율,
  동적 장애물 8개(사람 6 + 차량 2), actor 속도 0.3~1.5 m/s (명세 4.1).
"""

import filecmp
import importlib.util
import os
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG.parents[1] / "config"
WORLD = PKG / "worlds" / "warehouse.sdf"
GEN = PKG / "worlds" / "gen_warehouse_world.py"
sys.path.insert(0, str(PKG))

from amr_simulation.actor_trajectory import load_actors  # noqa: E402


@pytest.fixture(scope="module")
def world():
    return ET.parse(WORLD).getroot().find("world")


def _floats(text):
    return [float(v) for v in text.split()]


def test_regenerated_world_is_identical(monkeypatch, capsys):
    """생성기를 같은 프로세스에서 다시 돌린다 (모듈을 새로 읽어 seed 난수 상태도 처음부터 — 커버리지에도 잡힌다)."""
    spec = importlib.util.spec_from_file_location("gen_warehouse_world_fresh", GEN)
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, "warehouse.sdf")
        monkeypatch.setattr(sys, "argv", [str(GEN), out])
        gen.main()
        assert filecmp.cmp(out, WORLD, shallow=False), \
            "커밋된 warehouse.sdf 가 생성기 출력과 다르다 — python3 worlds/gen_warehouse_world.py 로 다시 생성할 것"
    assert "actors=" in capsys.readouterr().out


def test_interior_at_least_60_by_40(world):
    walls = world.find("model[@name='warehouse_walls']/link")
    inner = {}
    for v in walls.findall("visual"):
        x, y = _floats(v.find("pose").text)[:2]
        sx, sy, _ = _floats(v.find("geometry/box/size").text)
        inner[v.get("name")] = (abs(x) - sx / 2) if sx < sy else (abs(y) - sy / 2)
    assert 2 * min(inner["wall_east"], inner["wall_west"]) >= 60.0 - 1e-9
    assert 2 * min(inner["wall_north"], inner["wall_south"]) >= 40.0 - 1e-9


def test_marker_height_matches_camera():
    with open(CONFIG / "robot_params.yaml", encoding="utf-8") as f:
        base_h = yaml.safe_load(f)["/**"]["ros__parameters"]["robot"]["base_link_height"]
    with open(CONFIG / "sensors.yaml", encoding="utf-8") as f:
        cam_z = yaml.safe_load(f)["/**"]["ros__parameters"]["camera_link"]["extrinsic"]["z"]
    root = ET.parse(WORLD).getroot().find("world")
    markers = [m for m in root.findall("model") if m.get("name").endswith("_marker")]
    assert len(markers) == 7
    for m in markers:
        z = _floats(m.find("link/visual/pose").text)[2]
        assert z == pytest.approx(base_h + cam_z), m.get("name")


def _png_size(path):
    with open(path, "rb") as f:
        head = f.read(24)
    return struct.unpack(">II", head[16:24])


def test_signs_have_textures(world):
    signs = [m for m in world.findall("model") if m.get("name").startswith("sign_")]
    assert len(signs) == 25
    for m in signs:
        vis = m.find("link/visual")
        uri = vis.find("material/pbr/metal/albedo_map").text
        assert uri.startswith("model://sign/materials/textures/")
        png = PKG / "models" / uri[len("model://"):]
        assert png.is_file(), png
        w_px, h_px = _png_size(png)
        _, w, h = _floats(vis.find("geometry/box/size").text)
        assert w_px / h_px == pytest.approx(w / h, rel=0.02), m.get("name")
        assert vis.find("pose") is not None and _floats(vis.find("pose").text)[2] > 0.4   # 스캔 평면 위


def test_dynamic_obstacles(world):
    actors = load_actors(str(WORLD))
    assert len(actors) == 6
    for a in actors:
        speeds = []
        n = int(a.duration / 0.05)
        for i in range(n):
            _, _, _, vx, vy = a.state(i * 0.05)
            speeds.append((vx * vx + vy * vy) ** 0.5)
        cruise = sorted(speeds)[len(speeds) // 2]
        assert 0.3 - 1e-6 <= cruise <= 1.5 + 1e-6, (a.name, cruise)
        assert max(speeds) <= 1.05 * 1.5, a.name
    names = {i.find("name").text for i in world.findall("include")}
    assert {"forklift_main", "shuttle_amr"} <= names


def test_dock_floor_boxes_reach_scan_plane(world):
    """도크 바닥 화물(중형 0.30, 대형 0.40 m)은 LiDAR 스캔 평면(지면 +0.20)보다 높다."""
    with open(CONFIG / "robot_params.yaml", encoding="utf-8") as f:
        base_h = yaml.safe_load(f)["/**"]["ros__parameters"]["robot"]["base_link_height"]
    with open(CONFIG / "sensors.yaml", encoding="utf-8") as f:
        lidar_z = yaml.safe_load(f)["/**"]["ros__parameters"]["lidar"]["extrinsic"]["z"]
    boxes = [i for i in world.findall("include") if i.find("name").text.startswith("dock_")
             and "_box_" in i.find("name").text]
    assert len(boxes) == 8
    for b in boxes:
        kind = b.find("uri").text.split("/")[-1]
        sdf = ET.parse(PKG / "models" / kind / "model.sdf").getroot()
        height = _floats(sdf.find(".//visual/geometry/box/size").text)[2]
        assert height > base_h + lidar_z, kind


def test_fleet_spawn_poses_are_free():
    """config/fleet_spawn_poses.yaml 자세가 벽·랙·기둥·사람 경로·차량 경로와 겹치지 않는다."""
    spec = importlib.util.spec_from_file_location("gen", GEN)
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    with open(PKG / "config" / "fleet_spawn_poses.yaml", encoding="utf-8") as f:
        poses = yaml.safe_load(f)["fleet_spawn_poses"]
    assert len(poses) >= 5
    r = 0.36 + 0.3                                   # 로봇 외접원 반지름 + 여유
    aabbs = gen._aabbs(gen.rack_instances())
    actors = load_actors(str(WORLD))
    pts = list(poses.values())
    for name, (x, y, _yaw) in poses.items():
        assert abs(x) < gen.HALF_X - r and abs(y) < gen.HALF_Y - r, name
        for oname, x0, x1, y0, y1 in aabbs:
            dx, dy = max(x0 - x, 0, x - x1), max(y0 - y, 0, y - y1)
            assert (dx * dx + dy * dy) ** 0.5 > r, (name, oname)
        for a in actors:
            dmin = min(((a.state(t)[0] - x) ** 2 + (a.state(t)[1] - y) ** 2) ** 0.5
                       for t in [i * 0.25 for i in range(int(a.duration * 4))])
            assert dmin > r + 0.3, (name, a.name, dmin)
        for vname, (axis, lo, hi, fixed, *_rest) in gen.VEHICLES.items():
            along, across = (x, y) if axis == "x" else (y, x)
            assert not (lo - 2.5 < along < hi + 2.5 and abs(across - fixed) < 1.5), (name, vname)
    for i, a in enumerate(pts):
        for b in pts[i + 1:]:
            assert ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5 >= 1.5
