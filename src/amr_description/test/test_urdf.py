"""
xacro 전개 + URDF 파싱/정합 검사 (colcon test).

- xacro 가 여러 인자 조합(접두어, 적재물)에서 오류 없이 전개되는지
- URDF 가 단일 루트 트리이고, 센서 extrinsic·구동계 한계·비상 감속이 config/*.yaml 과 같은지
- 자기 차체 가시성 비트: 모든 링크 시각체에 같은 flags, LiDAR/카메라 mask 는 그 비트만 뺀 값, 로봇마다 다른 비트
- (도구가 있으면) check_urdf 통과, `ign sdf -p` 변환 후 병합된 모든 시각체에 visibility_flags 가 남는지
  (sdformat 이 <gazebo reference> 시각체 확장을 이름으로 짝짓는 동작에 기대므로 회귀를 잡는다)
"""

import math
import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG.parents[1] / "config"
XACRO = PKG / "urdf" / "amr.urdf.xacro"
FULL_MASK = 0xFFFFFFFF


def _params(name: str) -> dict:
    with open(CONFIG / name, encoding="utf-8") as f:
        return yaml.safe_load(f)["/**"]["ros__parameters"]


ROBOT = _params("robot_params.yaml")
SENSORS = _params("sensors.yaml")


def _xacro(**kw) -> str:
    args = ["xacro", str(XACRO), f"config_dir:={CONFIG}"] + [f"{k}:={v}" for k, v in kw.items()]
    r = subprocess.run(args, capture_output=True, text=True, check=False)
    assert r.returncode == 0, r.stderr
    return r.stdout


def _parse(text: str) -> ET.Element:
    root = ET.fromstring(text)
    assert root.tag == "robot"
    return root


def _joint(root: ET.Element, name: str) -> ET.Element:
    j = root.find(f"joint[@name='{name}']")
    assert j is not None, name
    return j


def _xyz(elem: ET.Element) -> list:
    return [float(v) for v in elem.find("origin").get("xyz").split()]


@pytest.fixture(scope="module")
def default_urdf() -> ET.Element:
    return _parse(_xacro())


@pytest.fixture(scope="module")
def fleet_urdf() -> ET.Element:
    return _parse(_xacro(robot_name="amr_02", prefix="amr_02/", payload="large"))


def test_tree_single_root(default_urdf, fleet_urdf):
    """모든 조인트의 부모/자식이 링크로 존재하고, 루트가 base_footprint 하나인 트리."""
    for root, pfx in ((default_urdf, ""), (fleet_urdf, "amr_02/")):
        links = {e.get("name") for e in root.findall("link")}
        children = set()
        for j in root.findall("joint"):
            parent, child = j.find("parent").get("link"), j.find("child").get("link")
            assert parent in links and child in links
            assert child not in children, f"링크 {child} 의 부모가 둘"
            children.add(child)
        assert links - children == {pfx + "base_footprint"}
        assert all(n.startswith(pfx) for n in links)


def test_extrinsics_match_sensors_yaml(default_urdf):
    """센서 링크 조인트 원점 == sensors.yaml extrinsic (URDF == YAML == TF 원칙)."""
    for joint, key in (("lidar_joint", "lidar"), ("camera_joint", "camera_link"),
                       ("imu_joint", "imu")):
        ext = SENSORS[key]["extrinsic"]
        assert _xyz(_joint(default_urdf, joint)) == pytest.approx([ext["x"], ext["y"], ext["z"]])
    base_h = ROBOT["robot"]["base_link_height"]
    # 결정된 장착 높이: LiDAR 스캔 평면 지면 +0.20 (차체 안), 카메라 +0.25 (데크 아래), 전면 1 cm 안쪽
    assert base_h + SENSORS["lidar"]["extrinsic"]["z"] == pytest.approx(0.20)
    assert base_h + SENSORS["camera_link"]["extrinsic"]["z"] == pytest.approx(0.25)
    front = ROBOT["robot"]["footprint_length"] / 2.0
    assert SENSORS["camera_link"]["extrinsic"]["x"] == pytest.approx(front - 0.01)
    deck = base_h + ROBOT["robot"]["height"] / 2.0
    assert base_h + SENSORS["lidar"]["extrinsic"]["z"] < deck
    assert base_h + SENSORS["camera_link"]["extrinsic"]["z"] < deck


def test_wheel_limits_and_diffdrive(default_urdf):
    """바퀴 조인트 <limit> = drive.*, DiffDrive 감속 한계 = -emergency_deceleration."""
    drive = ROBOT["drive"]
    for side in ("left", "right"):
        lim = _joint(default_urdf, f"{side}_wheel_joint").find("limit")
        assert float(lim.get("effort")) == pytest.approx(drive["wheel_effort_limit"])
        assert float(lim.get("velocity")) == pytest.approx(drive["wheel_velocity_limit"])
    dd = default_urdf.find(".//plugin[@name='ignition::gazebo::systems::DiffDrive']")
    assert float(dd.find("min_linear_acceleration").text) == pytest.approx(
        -ROBOT["limits"]["emergency_deceleration"])
    assert float(dd.find("max_linear_acceleration").text) == pytest.approx(
        ROBOT["limits"]["max_linear_acceleration"])
    # 바퀴 최고 속도가 최고 선속도 + 최고 각속도 동시 지령의 바깥 바퀴를 덮는다
    r, b = ROBOT["robot"]["wheel_radius"], ROBOT["robot"]["wheel_separation"]
    lim = ROBOT["limits"]
    need = (lim["max_linear_velocity"] + lim["max_angular_velocity"] * b / 2.0) / r
    assert drive["wheel_velocity_limit"] >= need


@pytest.mark.parametrize("kw,kind,mass", [
    ({}, None, 0.0),
    ({"payload": "small"}, "small", 2.0),
    ({"payload": "large"}, "large", 25.0),
    ({"payload_mass": "25"}, "large", 25.0),       # 질량만 → 담을 수 있는 가장 작은 박스
    ({"payload_mass": "5"}, "medium", 5.0),
    ({"payload": "medium", "payload_mass": "7.5"}, "medium", 7.5),
    ({"payload_mass": "0"}, None, 0.0),
])
def test_payload_box(kw, kind, mass):
    """적재물 = 명세 크기 박스, 데크 위 중심, 질량 반영. 없으면 cargo_link 는 빈 프레임."""
    root = _parse(_xacro(**kw))
    link = root.find("link[@name='cargo_link']")
    assert link is not None
    if kind is None:
        assert link.find("visual") is None and link.find("inertial") is None
        return
    size = [float(v) for v in link.find("visual/geometry/box").get("size").split()]
    assert size == pytest.approx(ROBOT["payload"][kind]["size"])
    assert float(link.find("inertial/mass").get("value")) == pytest.approx(mass)
    com_z = [float(v) for v in link.find("inertial/origin").get("xyz").split()][2]
    assert com_z == pytest.approx(size[2] / 2.0)
    deck = _xyz(_joint(root, "cargo_joint"))[2]
    assert deck == pytest.approx(ROBOT["robot"]["height"] / 2.0)


def _visibility(root: ET.Element):
    flags = {int(e.text) for e in root.iter("visibility_flags")}
    masks = {int(e.text) for e in root.iter("visibility_mask")}
    return flags, masks


def test_self_visibility_bits(default_urdf, fleet_urdf):
    """링크 시각체 flags 는 로봇 고유 비트 하나, 센서 mask 는 그 비트만 뺀 값, 로봇마다 다르다."""
    f1, m1 = _visibility(default_urdf)
    f2, m2 = _visibility(fleet_urdf)
    assert f1 == {1 << 4} and m1 == {FULL_MASK - (1 << 4)}      # amr_01 → 비트 4
    assert f2 == {1 << 5} and m2 == {FULL_MASK - (1 << 5)}      # amr_02 → 비트 5
    # 가시성 확장이 붙은 링크 = 시각체가 있는 링크 전부
    refs = {g.get("reference") for g in default_urdf.findall("gazebo")
            if g.find("visual/visibility_flags") is not None}
    with_visual = {e.get("name") for e in default_urdf.findall("link")
                   if e.find("visual") is not None}
    assert with_visual <= refs
    # 센서 3종 (gpu_lidar, camera, depth_camera) 모두 mask 를 가진다
    for s in default_urdf.iter("sensor"):
        if s.get("type") in ("gpu_lidar", "camera", "depth_camera"):
            assert s.find(".//visibility_mask") is not None, s.get("name")
    # 명시 인자가 우선한다
    f3, _ = _visibility(_parse(_xacro(robot_name="amr_03", self_visibility_bit="12")))
    assert f3 == {1 << 12}


def test_rgb_noise(default_urdf):
    """RGB 카메라에 sensors.yaml 의 가우시안 노이즈(셰이더 보정값)가 들어간다 (명세 7장)."""
    cam = default_urdf.find(".//sensor[@type='camera']")
    stddev = cam.find("camera/noise/stddev")
    assert stddev is not None
    rgb = SENSORS["rgb_camera"]
    assert float(stddev.text) == pytest.approx(rgb.get("gz_noise_stddev", rgb["noise_stddev"]))
    assert 0.0 < rgb["noise_stddev"] < 0.05


def test_check_urdf():
    """URDF 를 urdfdom check_urdf 로 파싱한다 (도구가 없으면 건너뛴다)."""
    exe = shutil.which("check_urdf")
    if exe is None:
        pytest.skip("check_urdf 없음")
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(_xacro(robot_name="amr_02", prefix="amr_02/", payload="medium"))
        path = f.name
    try:
        r = subprocess.run([exe, path], capture_output=True, text=True, check=False)
        assert r.returncode == 0, r.stdout + r.stderr
        assert "root Link: amr_02/base_footprint" in r.stdout
    finally:
        os.remove(path)


def test_sdf_conversion_keeps_visibility_flags():
    """`ign sdf -p` (sdformat URDF 변환) 뒤 병합된 모든 시각체에 자기 비트 flags 가 남는다."""
    exe = shutil.which("ign")
    if exe is None:
        pytest.skip("ign 없음")
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(_xacro(robot_name="amr_02", prefix="amr_02/", payload="large"))
        path = f.name
    try:
        r = subprocess.run([exe, "sdf", "-p", path], capture_output=True, text=True, check=False)
        assert r.returncode == 0, r.stderr
        sdf = ET.fromstring(r.stdout)
    finally:
        os.remove(path)
    visuals = list(sdf.iter("visual"))
    assert len(visuals) >= 9          # 차체 2 + 바퀴 2 + 캐스터 2 + 센서 3 + 적재물 1
    for v in visuals:
        flags = [int(e.text) for e in v.findall("visibility_flags")]
        assert flags and set(flags) == {1 << 5}, v.get("name")
    # 적재물 질량이 병합 링크에 합산된다 (47.6 − 바퀴 2.0 + 25)
    base = sdf.find(".//link[@name='amr_02/base_footprint']")
    lumped = float(base.find("inertial/mass").text)
    r0 = ROBOT["robot"]
    expected = r0["base_mass"] + 2 * r0["caster_mass"] + 25.0
    assert math.isclose(lumped, expected, rel_tol=1e-6)
    # 바퀴 조인트 한계가 SDF 로 넘어간다
    for j in sdf.iter("joint"):
        if j.get("name").endswith("wheel_joint"):
            assert float(j.find("axis/limit/effort").text) == pytest.approx(
                ROBOT["drive"]["wheel_effort_limit"])
