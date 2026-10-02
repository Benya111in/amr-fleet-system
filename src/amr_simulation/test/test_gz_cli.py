"""amr_simulation/gz_cli.py 단위 테스트 (Gazebo 없이): 요청 문자열, Pose_V 텍스트 파싱, CLI 실패 처리."""

import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from amr_simulation import gz_cli  # noqa: E402
from amr_simulation.gz_cli import (GzWorld, create_request, parse_pose_v,  # noqa: E402
                                   proto_string, remove_request)

POSE_V = """header {
  stamp {
    sec: 12
    nsec: 5000000
  }
}
pose {
  name: "amr_01"
  id: 42
  position {
    x: 18
    y: -16.5
  }
  orientation {
    z: 0.70710678
    w: 0.70710678
  }
}
pose {
  name: "link"
  id: 43
  position {
    z: 0.2
  }
  orientation {
    w: 1
  }
}
pose {
  name: "amr_01_cargo"
  id: 90
  position {
    x: 18
    y: -16.5
    z: 0.532
  }
  orientation {
    z: 0.70710678
    w: 0.70710678
  }
}
"""


def test_parse_pose_v_defaults_and_names():
    p = parse_pose_v(POSE_V)
    assert p["amr_01"] == pytest.approx((18.0, -16.5, 0.0, 0.0, 0.0, 0.70710678, 0.70710678))
    assert p["amr_01_cargo"][2] == pytest.approx(0.532)
    assert p["link"] == pytest.approx((0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0))
    assert parse_pose_v("") == {} and parse_pose_v("garbage\n}") == {}


def test_requests():
    assert proto_string('a"b\\c\nd\te\r') == '"a\\"b\\\\c\\nd\\te\\r"'
    req = create_request("amr_01_cargo", '<sdf v="1"/>',
                         (1.0, 2.0, 0.532, 0.0, 0.0, 0.5, 0.8660254))
    assert req.startswith('sdf: "<sdf v=\\"1\\"/>" name: "amr_01_cargo" allow_renaming: false')
    assert "position { x: 1.000000 y: 2.000000 z: 0.532000 }" in req
    assert "orientation { x: 0.000000000 y: 0.000000000 z: 0.500000000 w: 0.866025400 }" in req
    assert remove_request("amr_01_cargo") == 'name: "amr_01_cargo" type: MODEL'


class Runner:
    def __init__(self, rc=0, out="data: true\n"):
        self.rc, self.out, self.calls = rc, out, []

    def __call__(self, args, timeout):
        self.calls.append((args, timeout))
        return subprocess.CompletedProcess(args, self.rc, self.out, "")


def test_gz_world_calls():
    r = Runner()
    gz = GzWorld("warehouse", runner=r, timeout_ms=3000)
    assert gz.create("m", "<sdf/>", (0, 0, 0, 0, 0, 0, 1))
    args, timeout = r.calls[-1]
    assert args[:3] == ["service", "-s", "/world/warehouse/create"]
    assert "ignition.msgs.EntityFactory" in args and timeout == pytest.approx(13.0)
    assert gz.remove("m")
    assert r.calls[-1][0][2] == "/world/warehouse/remove"
    assert not GzWorld("w", runner=Runner(out="data: false")).remove("m")
    assert not GzWorld("w", runner=Runner(rc=-1, out="")).create("m", "", (0,) * 7)


def test_gz_world_poses():
    assert GzWorld("w", runner=Runner(out=POSE_V)).poses()["amr_01"][0] == 18.0
    r = Runner(rc=1, out=POSE_V)
    assert GzWorld("w", runner=r).poses() == {}
    assert r.calls[-1][0] == ["topic", "-e", "-n", "1", "-t", "/world/w/pose/info"]


def test_ign_failures_do_not_raise(monkeypatch):
    def timeout(*a, **kw):
        raise subprocess.TimeoutExpired(cmd=["ign"], timeout=1.0, output="partial")
    monkeypatch.setattr(gz_cli.subprocess, "run", timeout)
    r = gz_cli.ign(["service", "-l"], 1.0)
    assert r.returncode == -1 and r.stdout == "partial"

    def missing(*a, **kw):
        raise FileNotFoundError("ign")
    monkeypatch.setattr(gz_cli.subprocess, "run", missing)
    r = gz_cli.ign(["service", "-l"], 1.0)
    assert r.returncode == -1 and "ign" in r.stderr

    monkeypatch.setattr(gz_cli.subprocess, "run",
                        lambda *a, **kw: subprocess.CompletedProcess(a[0], 0, "ok", ""))
    assert gz_cli.ign(["x"], 1.0).stdout == "ok"
