"""scripts/gz_world.py 순수 함수 단위 테스트 (Gazebo 없이): scene/info 파싱, create 요청 문자열."""

import importlib.util
import math
from pathlib import Path

import pytest

_SPEC = importlib.util.spec_from_file_location(
    "gz_world", Path(__file__).resolve().parents[1] / "scripts" / "gz_world.py")
gz_world = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(gz_world)

SCENE = """name: "warehouse"
ambient {
  r: 0.25
}
model {
  name: "ground_plane"
  id: 8
  link {
    name: "link"
    visual {
      name: "visual"
    }
  }
}
model {
  name: "amr_01"
  link {
    name: "amr_01/base_footprint"
  }
}
light {
  name: "sun"
}
"""


def test_parse_scene_models_only_top_level_models():
    assert gz_world.parse_scene_models(SCENE) == {"ground_plane", "amr_01"}


def test_create_request_escapes_and_pose():
    req = gz_world.create_request("amr_02", '<robot name="a">\n  <x v="1\\2"/>\n</robot>',
                                  1.5, -2.0, 0.02, math.pi / 2)
    assert req.startswith('sdf: "<robot name=\\"a\\">\\n  <x v=\\"1\\\\2\\"/>\\n</robot>"')
    assert 'name: "amr_02" allow_renaming: false' in req
    qz = float(req.split("orientation { x: 0 y: 0 z: ")[1].split()[0])
    assert qz == pytest.approx(math.sin(math.pi / 4))
    assert "position { x: 1.5 y: -2.0 z: 0.02 }" in req


def test_strip_comments():
    out = gz_world.strip_comments('<robot><!-- 한글 주석 --><link name="a"/></robot>')
    assert "<!--" not in out and 'name="a"' in out and out.isascii()


def test_argv_ignores_ros_args(monkeypatch):
    seen = {}
    monkeypatch.setattr(gz_world, "cmd_exists", lambda a: seen.setdefault("name", a.name) and 0)
    rc = gz_world.main(["exists", "--world", "w", "--name", "amr_09",
                        "--ros-args", "-r", "__ns:=/x"])
    assert rc == 0 and seen["name"] == "amr_09"
