"""
spawn.launch.py / description.launch.py 구성 테스트 (Gazebo 없이, 런치 구성만 만든다).

- 브리지에 런타임 적재 토픽 3개(cargo/attach·detach ROS→GZ Empty, cargo/state GZ→ROS String)가 있고, gz 이름이
  xacro DetachableJoint 의 토픽(/<robot_name>/cargo/*)과 같다 — 다르면 요청이 플러그인에 닿지 않는다.
- 로봇마다 payload_manager_node 를 그 네임스페이스에 띄운다 (bringup 변경 없이 스폰 경로에서). amr_simulation 이
  없으면 로그만 남긴다. payload_manager:=false 면 띄우지 않는다.
- description.launch.py 가 runtime_payload 인자를 xacro 로 넘긴다.
- 3D LiDAR (보너스 2): lidar3d 가 true 거나 auto + 스폰 목록의 로봇별 키가 true 일 때만 점군 브리지(lazy)와
  amr_lidar3d 전처리 노드가 붙는다. 기본(키 없음)은 브리지·노드 구성이 그대로다. description.launch.py 의 xacro
  lidar3d 값도 같은 규칙으로 정해진다.
"""

import importlib.util
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

launch = pytest.importorskip("launch")
launch_ros = pytest.importorskip("launch_ros")
from launch import LaunchContext  # noqa: E402
from launch.actions import DeclareLaunchArgument, LogInfo  # noqa: E402

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG.parents[1] / "config"


def _load(name):
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), PKG / "launch" / name)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


spawn = _load("spawn.launch.py")
description = _load("description.launch.py")
SENSORS = yaml.safe_load((CONFIG / "sensors.yaml").read_text(encoding="utf-8"))["/**"][
    "ros__parameters"]


def _cargo_entries(entries):
    return {e["ros_topic_name"]: e for e in entries if e["ros_topic_name"].startswith("cargo/")}


def test_bridge_has_cargo_topics():
    cargo = _cargo_entries(spawn._bridge_entries(SENSORS))
    assert set(cargo) == {"cargo/attach", "cargo/detach", "cargo/state"}
    for req in ("attach", "detach"):
        e = cargo[f"cargo/{req}"]
        assert (e["direction"], e["ros_type_name"], e["gz_type_name"]) == (
            "ROS_TO_GZ", "std_msgs/msg/Empty", "ignition.msgs.Empty")
    e = cargo["cargo/state"]
    assert (e["direction"], e["ros_type_name"], e["gz_type_name"], e["lazy"]) == (
        "GZ_TO_ROS", "std_msgs/msg/String", "ignition.msgs.StringMsg", False)
    # odom TF 옵션과 무관하게 항상 있다
    assert set(_cargo_entries(spawn._bridge_entries(SENSORS, bridge_odom_tf=True))) == set(cargo)


def test_bridge_gz_names_match_xacro_plugin():
    """브리지 노드 네임스페이스(/<robot>) + expand_gz_topic_names → xacro 의 attach/detach/output 토픽."""
    r = subprocess.run(["xacro", str(PKG / "urdf" / "amr.urdf.xacro"), f"config_dir:={CONFIG}",
                        "robot_name:=amr_03", "prefix:=amr_03/"],
                       capture_output=True, text=True, check=False)
    if r.returncode != 0:
        pytest.skip(f"xacro 전개 불가: {r.stderr[-200:]}")
    dj = ET.fromstring(r.stdout).find(
        ".//plugin[@name='ignition::gazebo::systems::DetachableJoint']")
    cargo = _cargo_entries(spawn._bridge_entries(SENSORS))
    assert dj.find("attach_topic").text == "/amr_03/" + cargo["cargo/attach"]["gz_topic_name"]
    assert dj.find("detach_topic").text == "/amr_03/" + cargo["cargo/detach"]["gz_topic_name"]
    assert dj.find("output_topic").text == "/amr_03/" + cargo["cargo/state"]["gz_topic_name"]


def _context(**overrides):
    ctx = LaunchContext()
    values = {"robot_name": "amr_04", "world": "warehouse", "x": "1.0", "y": "2.0", "z": "0.02",
              "yaw": "0.5", "use_sim_time": "true", "config_dir": str(CONFIG), "bridge_config": "",
              "bridge_odom_tf": "false", "spawn_timeout": "60", "payload_manager": "true",
              "lidar3d": "auto", "lidar3d_config": "", "lidar3d_preprocessor": "true"}
    values.update(overrides)
    ctx.launch_configurations.update(values)
    return ctx


def _nodes(actions):
    return [a for a in actions if isinstance(a, launch_ros.actions.Node)]


def _generated_bridge_file(actions, ctx):
    for a in actions:
        if isinstance(a, LogInfo):
            text = "".join(s.perform(ctx) for s in a.msg)
            if "bridge config generated:" in text:
                return text.split("generated:")[1].strip()
    return None


def test_setup_starts_payload_manager_in_robot_namespace():
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory("amr_simulation")
    except Exception:
        pytest.skip("amr_simulation 이 설치 공간에 없다")
    ctx = _context()
    actions = spawn._setup(ctx)
    path = _generated_bridge_file(actions, ctx)
    try:
        assert path and os.path.isfile(path)
        entries = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        assert set(_cargo_entries(entries)) == {"cargo/attach", "cargo/detach", "cargo/state"}
    finally:
        if path and os.path.isfile(path):
            os.remove(path)
    pm = [n for n in _nodes(actions) if n.node_executable == "payload_manager_node.py"]
    assert len(pm) == 1 and pm[0].node_package == "amr_simulation"
    assert getattr(pm[0], "_Node__node_namespace") == "amr_04"      # launch_ros 내부 속성 (실행 전)
    execs = {n.node_executable for n in _nodes(actions)}
    assert {"gz_world.py", "parameter_bridge", "image_bridge"} <= execs


def test_payload_manager_can_be_disabled_and_missing_package_is_logged(monkeypatch):
    ctx = _context(payload_manager="false", bridge_config="/tmp/unused_bridge.yaml")
    actions = spawn._setup(ctx)
    assert not [n for n in _nodes(actions) if n.node_executable == "payload_manager_node.py"]

    def missing(pkg):
        raise spawn.PackageNotFoundError(pkg)
    monkeypatch.setattr(spawn, "get_package_share_directory", missing)
    act = spawn.payload_manager_action("amr_05", "warehouse", str(CONFIG), True)
    assert isinstance(act, LogInfo)
    assert "amr_simulation" in "".join(s.perform(LaunchContext()) for s in act.msg)


def test_setup_rejects_missing_sensors_yaml(tmp_path):
    with pytest.raises(RuntimeError):
        spawn._setup(_context(config_dir=str(tmp_path)))


def test_launch_arguments():
    names = {a.name: a for a in spawn.generate_launch_description().entities
             if isinstance(a, DeclareLaunchArgument)}
    assert "payload_manager" in names
    assert "".join(s.perform(LaunchContext()) for s in names["payload_manager"].default_value) \
        == "true"
    desc = {a.name: a for a in description.generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument)}
    assert "".join(s.perform(LaunchContext()) for s in desc["runtime_payload"].default_value) \
        == "true"
    assert spawn._gz_camera_info_topic("camera/image_raw") == "camera/camera_info"
    assert description._default_config_dir().endswith("config")


def _robots_file(tmp_path, flag_robot="amr_04"):
    path = tmp_path / "spawn.yaml"
    robots = [{"name": "amr_04", "x": 0.0, "y": 0.0, "yaw": 0.0},
              {"name": "amr_05", "x": 2.0, "y": 0.0, "yaw": 0.0}]
    for r in robots:
        if r["name"] == flag_robot:
            r["lidar3d"] = True
    path.write_text(yaml.safe_dump({"robots": robots}), encoding="utf-8")
    return str(path)


def _bridge_from_setup(ctx):
    actions = spawn._setup(ctx)
    path = _generated_bridge_file(actions, ctx)
    try:
        return actions, yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    finally:
        os.remove(path)


def _l3_nodes(actions):
    return [n for n in _nodes(actions) if n.node_package == "amr_lidar3d"]


def _l3_logged(actions, ctx):
    return any("lidar3d" in "".join(s.perform(ctx) for s in a.msg)
               for a in actions if isinstance(a, LogInfo))


def test_lidar3d_auto_without_key_keeps_bridge_and_nodes(tmp_path):
    """스폰 목록에 키가 없거나(기본) 다른 로봇에만 있으면 브리지·노드가 이전과 같다."""
    for robots_file in ("", _robots_file(tmp_path, flag_robot="amr_05")):
        ctx = _context(payload_manager="false", robots_file=robots_file)
        actions, entries = _bridge_from_setup(ctx)
        assert entries == spawn._bridge_entries(SENSORS)
        assert not [e for e in entries if "lidar3d" in e["ros_topic_name"]]
        assert not _l3_nodes(actions) and not _l3_logged(actions, ctx)
        assert {n.node_executable for n in _nodes(actions)} == {
            "gz_world.py", "parameter_bridge", "image_bridge"}


@pytest.mark.parametrize("how", ["auto_key", "explicit"])
def test_lidar3d_robot_gets_points_bridge_and_preprocessor(tmp_path, how):
    ctx = (_context(payload_manager="false", robots_file=_robots_file(tmp_path))
           if how == "auto_key" else _context(payload_manager="false", lidar3d="true"))
    actions, entries = _bridge_from_setup(ctx)
    base = spawn._bridge_entries(SENSORS)
    assert entries[:len(base)] == base                       # 기존 항목은 그대로, 뒤에 더한다
    extra = entries[len(base):]
    assert extra == [{"ros_topic_name": "lidar3d/points_raw", "gz_topic_name": "lidar3d/points",
                      "ros_type_name": "sensor_msgs/msg/PointCloud2",
                      "gz_type_name": "ignition.msgs.PointCloudPacked",
                      "direction": "GZ_TO_ROS", "lazy": True}]
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory("amr_lidar3d")
    except Exception:
        assert _l3_logged(actions, ctx)                      # 없으면 이유를 남긴다
        return
    nodes = _l3_nodes(actions)
    assert len(nodes) == 1 and nodes[0].node_executable == "cloud_preprocessor_node"
    assert getattr(nodes[0], "_Node__node_namespace") == "amr_04"


def test_lidar3d_explicit_false_wins_and_preprocessor_can_be_disabled(tmp_path):
    ctx = _context(payload_manager="false", robots_file=_robots_file(tmp_path), lidar3d="false")
    actions, entries = _bridge_from_setup(ctx)
    assert entries == spawn._bridge_entries(SENSORS)
    ctx = _context(payload_manager="false", lidar3d="true", lidar3d_preprocessor="false")
    actions, entries = _bridge_from_setup(ctx)
    assert entries[-1]["ros_topic_name"] == "lidar3d/points_raw"
    assert not _l3_nodes(actions)


def test_description_lidar3d_flag_substitution(tmp_path):
    from launch.substitutions import LaunchConfiguration
    sub = description.Lidar3dFlag(LaunchConfiguration("robot_name"),
                                  LaunchConfiguration("lidar3d"))
    robots_file = _robots_file(tmp_path)
    cases = [({"robot_name": "amr_04", "lidar3d": "auto", "robots_file": robots_file}, "true"),
             ({"robot_name": "amr_05", "lidar3d": "auto", "robots_file": robots_file}, "false"),
             ({"robot_name": "amr_04", "lidar3d": "auto"}, "false"),
             ({"robot_name": "amr_04", "lidar3d": "false", "robots_file": robots_file}, "false"),
             ({"robot_name": "amr_05", "lidar3d": "true"}, "true")]
    for values, expected in cases:
        ctx = LaunchContext()
        ctx.launch_configurations.update(values)
        assert sub.perform(ctx) == expected, values
    desc = {a.name: a for a in description.generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument)}
    assert "".join(s.perform(LaunchContext()) for s in desc["lidar3d"].default_value) == "auto"
    cfg = description.Lidar3dConfig(LaunchConfiguration("lidar3d_config"))
    ctx = LaunchContext()
    ctx.launch_configurations["lidar3d_config"] = ""
    assert cfg.perform(ctx).endswith("config/lidar3d.yaml")


def test_lidar3d_preprocessor_params_not_lazy():
    """늦게 켜진 Fortress 센서는 놓친 주기를 몰아 렌더한다 → 전처리는 기동 즉시 구독(lazy 끔)."""
    from amr_description import lidar3d as l3d
    params = l3d.load_config(str(Path(spawn.__file__).resolve().parents[1] / "config"
                                 / "lidar3d.yaml"))
    p = spawn.lidar3d_preprocessor_params(params, True)
    assert p == {"input_topic": "lidar3d/points_raw", "output_topic": "lidar3d/points",
                 "frame_id": "", "lazy": False, "use_sim_time": True}
