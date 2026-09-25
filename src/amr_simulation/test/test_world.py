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


def test_location_marker_registry_matches_behavior_config():
    """위치 표지 마커의 지도 자세가 재위치추정 레지스트리(behavior.yaml)와 같아야 한다.

    통로가 랙 열 간격 6 m 로 주기적이라 통로 방향 6 m 순간 이동은 LiDAR + 지도만으로 구별할 수
    없다 (통합 11). 이 마커가 유일한 전역 기준점이므로 자세가 어긋나면 복구가 로봇을 엉뚱한 곳으로
    끌고 간다 — 둘이 따로 노는 것을 여기서 잡는다.
    """
    spec = importlib.util.spec_from_file_location("gen_warehouse_world_markers", GEN)
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    want = {**gen.pillar_marker_poses(), **gen.rack_marker_poses()}
    path = PKG.parents[0] / "amr_behavior" / "config" / "behavior.yaml"
    if not path.is_file():
        pytest.skip("amr_behavior 가 이 워크스페이스에 없다 (패키지 단독 브랜치)")
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    reloc = next((v["ros__parameters"]["relocalization"] for v in cfg.values()
                  if isinstance(v, dict) and "ros__parameters" in v
                  and "relocalization" in v["ros__parameters"]), None)
    if reloc is None or "landmark_ids" not in reloc:
        pytest.skip("behavior.yaml 에 위치 표지 레지스트리가 아직 없다")
    ids, poses = reloc["landmark_ids"], reloc["landmark_poses"]
    assert len(poses) == 3 * len(ids), (len(poses), len(ids))
    got = {int(i): tuple(poses[3 * k:3 * k + 3]) for k, i in enumerate(ids)}
    assert set(got) == set(want), (sorted(set(got) ^ set(want)))
    for mid, (x, y, yaw) in want.items():
        gx, gy, gyaw = got[mid]
        assert abs(gx - x) < 1e-3 and abs(gy - y) < 1e-3, (mid, got[mid])
        assert abs(gyaw - yaw) < 1e-3, (mid, got[mid])


def test_location_markers_are_in_the_world(world):
    """위치 표지 판이 실제로 월드에 있고 개수가 기둥 × 4 + 랙 열 × 행 × 2 와 같다."""
    names = [m.get("name") for m in world.iter("model")]
    pillar = [n for n in names if n and n.startswith("pillar_") and "_marker_" in n]
    rack = [n for n in names if n and n.startswith("rack_marker_")]
    assert len(pillar) == 32, len(pillar)
    assert len(rack) == 42, len(rack)


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
