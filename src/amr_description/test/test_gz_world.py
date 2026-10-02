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


# ---------------------------------------------------------------- 가짜 ign CLI 로 명령 흐름
class FakeClock:
    """time.monotonic / time.sleep 대체 — sleep 이 시간을 바로 흘린다."""

    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


class FakeIgn:
    """ign service 호출 흉내: 서비스 목록, scene/info(현재 모델), create(모델 추가), control."""

    def __init__(self, services=("/world/w/create", "/world/w/control"), models=("ground_plane",),
                 create_ok=True, create_adds=True, control_ok=True, scene_ok=True):
        self.services, self.models = list(services), set(models)
        self.create_ok, self.create_adds = create_ok, create_adds
        self.control_ok, self.scene_ok = control_ok, scene_ok
        self.calls = []

    def __call__(self, args, timeout):
        import subprocess
        self.calls.append(args)
        out = ""
        if args[:2] == ["service", "-l"]:
            out = "\n".join(self.services)
        elif args[:2] == ["service", "-s"] and args[2].endswith("/scene/info"):
            if self.scene_ok:
                out = "".join(f'model {{\n  name: "{m}"\n}}\n' for m in sorted(self.models))
        elif args[:2] == ["service", "-s"] and args[2].endswith("/create"):
            if self.create_adds:
                name = args[args.index("--req") + 1].split('name: "')[1].split('"')[0]
                self.models.add(name)
            out = "data: true" if self.create_ok else ""
        elif args[:2] == ["service", "-s"] and args[2].endswith("/control"):
            out = "data: true" if self.control_ok else "data: false"
        return subprocess.CompletedProcess(args, 0, out, "")


@pytest.fixture
def fake(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(gz_world, "time", clock)

    def install(**kw):
        ign = FakeIgn(**kw)
        monkeypatch.setattr(gz_world, "ign", ign)
        return ign
    monkeypatch.setattr(gz_world, "get_robot_description", lambda topic, timeout: "<robot/>")
    return install


def _run(*argv):
    return gz_world.main(list(argv))


def test_spawn_happy_path(fake):
    ign = fake()
    assert _run("spawn", "--world", "w", "--name", "amr_01", "--x", "1.5", "--yaw", "0.3") == 0
    create = next(c for c in ign.calls if c[1:3] == ["-s", "/world/w/create"])
    assert 'name: "amr_01" allow_renaming: false' in create[create.index("--req") + 1]
    assert "amr_01" in ign.models


def test_spawn_failures(fake, monkeypatch):
    fake(services=())                                             # 월드가 뜨지 않음
    assert _run("spawn", "--world", "w", "--name", "a", "--world-timeout", "3") == 1
    fake(models=("ground_plane", "a"))                            # 같은 이름이 이미 있음
    assert _run("spawn", "--world", "w", "--name", "a") == 1
    fake(scene_ok=False)                                          # scene/info 실패
    assert _run("spawn", "--world", "w", "--name", "a") == 1
    fake(create_ok=False, create_adds=False)                      # 거부 + 끝내 안 생김
    assert _run("spawn", "--world", "w", "--name", "a", "--verify-timeout", "3") == 1
    ign = fake(create_ok=False)                                   # 응답 없음이어도 실제로 생기면 성공
    assert _run("spawn", "--world", "w", "--name", "a") == 0 and "a" in ign.models
    monkeypatch.setattr(gz_world, "get_robot_description", lambda topic, timeout: "")
    fake()
    assert _run("spawn", "--world", "w", "--name", "a") == 1     # robot_description 없음


def test_unpause_and_exists(fake):
    ign = fake(models=("ground_plane", "amr_01", "amr_02"))
    assert _run("unpause", "--world", "w", "--wait-models", "amr_01,amr_02") == 0
    assert any(c[1:3] == ["-s", "/world/w/control"] and "pause: false" in c for c in ign.calls)
    fake()
    assert _run("unpause", "--world", "w", "--wait-models", "amr_09",
                "--verify-timeout", "2") == 1
    fake(control_ok=False)
    assert _run("unpause", "--world", "w") == 1                   # 5회 재시도 후 실패
    fake(services=())
    assert _run("unpause", "--world", "w", "--world-timeout", "2") == 1
    fake(models=("amr_01",))
    assert _run("exists", "--world", "w", "--name", "amr_01") == 0
    assert _run("exists", "--world", "w", "--name", "amr_02") == 1


def test_ign_timeout_and_logging(monkeypatch, capsys):
    import subprocess

    def boom(*a, **kw):
        raise subprocess.TimeoutExpired(cmd=["ign"], timeout=1.0)
    monkeypatch.setattr(gz_world.subprocess, "run", boom)
    r = gz_world.ign(["service", "-l"], 1.0)
    assert r.returncode == -1
    gz_world.log("hello")
    gz_world.err("bad")
    out = capsys.readouterr()
    assert "[gz_world] hello" in out.out and "ERROR: bad" in out.err


def test_wait_service_logs_progress(fake, capsys):
    fake(services=())
    assert not gz_world.wait_service("/world/w/create", 65.0)
    assert "대기 중" in capsys.readouterr().out


def test_get_robot_description_times_out():
    """아무도 발행하지 않는 토픽이면 빈 문자열 (rclpy 가 없으면 건너뛴다)."""
    pytest.importorskip("rclpy")
    pytest.importorskip("std_msgs.msg")
    assert gz_world.get_robot_description("/gz_world_test/none/robot_description", 0.3) == ""
