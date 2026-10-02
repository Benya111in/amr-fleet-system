"""
spawn.launch.py / description.launch.py 구성 테스트 (Gazebo 없이, 런치 구성만 만든다).

- 브리지에 런타임 적재 토픽 3개(cargo/attach·detach ROS→GZ Empty, cargo/state GZ→ROS String)가 있고, gz 이름이
  xacro DetachableJoint 의 토픽(/<robot_name>/cargo/*)과 같다 — 다르면 요청이 플러그인에 닿지 않는다.
- 로봇마다 payload_manager_node 를 그 네임스페이스에 띄운다 (bringup 변경 없이 스폰 경로에서). amr_simulation 이
  없으면 로그만 남긴다. payload_manager:=false 면 띄우지 않는다.
- description.launch.py 가 runtime_payload 인자를 xacro 로 넘긴다.
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
              "bridge_odom_tf": "false", "spawn_timeout": "60", "payload_manager": "true"}
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
