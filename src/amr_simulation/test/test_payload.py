"""
런타임 적재물 순수 로직 단위 테스트 (Gazebo·ROS 없이).

물품 표 해석, settle, 화물 SDF, 데크 자세, 그리고 DetachableJoint(Fortress 6.18 실측 동작)를 흉내 낸
가짜 Gazebo 로 적재/하역 절차.
xacro 출력과 이름·가시성 비트·데크 높이 규칙이 같은지도 대조한다 (어긋나면 화물이 영영 붙지 않는다).
"""

import math
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG.parents[1] / "config"
XACRO = PKG.parent / "amr_description" / "urdf" / "amr.urdf.xacro"
sys.path.insert(0, str(PKG))

from amr_simulation.payload import (CargoSpec, DesiredPayload, DriveModel,  # noqa: E402
                                    PayloadManager, PayloadTable, cargo_model_name, cargo_sdf,
                                    deck_pose, roll_pitch, rotate, self_visibility_bit,
                                    state_json, to_local)

TABLE = PayloadTable.from_robot_params(str(CONFIG / "robot_params.yaml"))


def _quat_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


# ---------------------------------------------------------------- 물품 표 / 요청 해석
def test_table_matches_spec_item_table():
    """명세 8장 물품 표: 소형 2 kg 30x20x15, 중형 10 kg 50x40x30, 대형 25 kg 60x50x40. 데크 = 지면 +0.33."""
    assert TABLE.items["small"] == (2.0, (0.30, 0.20, 0.15))
    assert TABLE.items["medium"] == (10.0, (0.50, 0.40, 0.30))
    assert TABLE.items["large"] == (25.0, (0.60, 0.50, 0.40))
    assert TABLE.deck_height == pytest.approx(0.33)


@pytest.mark.parametrize("item,mass,expect", [
    ("large", 25.0, ("large", 25.0)),
    ("LARGE ", 25.0, ("large", 25.0)),         # 대소문자·공백 (executor 는 소문자로 비교)
    ("medium", None, ("medium", 10.0)),        # payload/mass 미발행 → 표 질량
    ("medium", 0.0, ("medium", 10.0)),
    ("small", 3.5, ("small", 3.5)),            # item_mass 덮어쓰기
    ("", 25.0, ("large", 25.0)),               # 질량만 → 담는 가장 작은 박스 (xacro 규칙)
    ("none", 5.0, ("medium", 5.0)),
    ("", 40.0, ("large", 40.0)),               # 표보다 무거우면 가장 큰 박스
    ("pallet", 7.0, ("medium", 7.0)),          # 모르는 종류 + 질량
])
def test_resolve(item, mass, expect):
    spec = TABLE.resolve(item, mass)
    assert (spec.item, spec.mass) == expect
    assert spec.size == TABLE.items[expect[0]][1]


@pytest.mark.parametrize("item,mass", [("", None), ("", 0.0), ("none", -1.0), ("", math.nan)])
def test_resolve_none(item, mass):
    assert TABLE.resolve(item, mass) is None


def test_resolve_unknown_without_mass_raises():
    with pytest.raises(ValueError):
        TABLE.resolve("pallet", None)


def test_inertia_is_homogeneous_box():
    ixx, iyy, izz = CargoSpec("large", (0.60, 0.50, 0.40), 25.0).inertia()
    # models/box_large/model.sdf 의 해석값과 같다
    assert (ixx, iyy, izz) == pytest.approx((0.854167, 1.083333, 1.270833), abs=1e-6)


def test_drive_model_predictions():
    """토크 한계 모델: 무적재·소형·중형은 가속 1.0 한계, 대형은 한계 아래. 제동은 질량이 클수록 작다."""
    m = DriveModel.from_robot_params(str(CONFIG / "robot_params.yaml"))
    assert m.empty_mass == pytest.approx(47.6)
    assert m.force == pytest.approx(2 * 3.0 / 0.0825)
    assert m.wheel_inertia_mass == pytest.approx(2 * 0.0034 / 0.0825 ** 2)
    assert m.accel(0.0) == m.accel(2.0) == m.accel(10.0) == 1.0
    assert m.accel(25.0) == pytest.approx(0.830, abs=0.002)
    assert m.brake(0.0) == pytest.approx(1.653, abs=0.002)    # 보정점 (무적재 E-Stop 실측)
    assert m.brake(25.0) == pytest.approx(1.146, abs=0.002)
    brakes = [m.brake(x) for x in (0.0, 2.0, 10.0, 25.0)]
    assert brakes == sorted(brakes, reverse=True)
    assert m.brake(0.0) < m.brake_limit                       # 비상 감속 3.0 보다 토크가 먼저 걸린다


def test_desired_payload_settles_after_both_messages():
    d = DesiredPayload(settle=0.2)
    assert d.ready(0.0) is None                       # 아직 요청 없음
    d.on_attach("large", 10.0)
    assert d.ready(10.1) is None                      # settle 중
    d.on_mass(25.0, 10.15)                            # 두 번째 메시지가 settle 을 다시 연다
    assert d.ready(10.3) is None
    assert d.ready(10.36) == ("large", 25.0)
    d.on_mass(0.0, 11.0)                              # 하역: 질량이 먼저 와도
    d.on_attach("", 11.01)
    assert d.ready(11.25) == ("", 0.0)


# ---------------------------------------------------------------- SDF / 자세
def test_cargo_sdf():
    spec = TABLE.resolve("large", 25.0)
    root = ET.fromstring(cargo_sdf("amr_03_cargo", spec, 1 << 6))
    model = root.find("model")
    assert model.get("name") == "amr_03_cargo"
    link = model.find("link")
    assert link.get("name") == "link"                 # DetachableJoint child_link
    assert float(link.find("inertial/mass").text) == 25.0
    assert float(link.find("inertial/inertia/izz").text) == pytest.approx(1.270833, abs=1e-5)
    for tag in ("collision", "visual"):
        size = [float(v) for v in link.find(f"{tag}/geometry/box/size").text.split()]
        assert size == pytest.approx([0.60, 0.50, 0.40])
    assert int(link.find("visual/visibility_flags").text) == 1 << 6
    assert link.find("collision/surface/friction/ode/mu").text == "0.6"


def test_rotation_helpers():
    q = _quat_yaw(math.pi / 2)
    assert rotate(q, (1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)
    # 90° pitch: x → -z
    qp = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))
    assert rotate(qp, (1.0, 0.0, 0.0)) == pytest.approx((0.0, 0.0, -1.0), abs=1e-12)
    assert roll_pitch(qp) == pytest.approx((0.0, math.pi / 2), abs=1e-6)
    frame = (1.0, 2.0, 0.0) + q
    assert to_local(frame, (1.0, 3.0, 0.5)) == pytest.approx((1.0, 0.0, 0.5), abs=1e-12)


def test_deck_pose_on_level_and_tilted_robot():
    spec = TABLE.resolve("medium", None)
    robot = (5.0, -3.0, 0.001) + _quat_yaw(1.2)
    p = deck_pose(robot, TABLE.deck_height, spec, 0.002)
    assert p[:3] == pytest.approx((5.0, -3.0, 0.001 + 0.33 + 0.15 + 0.002))
    assert p[3:] == robot[3:]
    # 기울어진 로봇: 로봇 기준으로는 항상 데크 중앙 위
    tilt = (0.0, math.sin(0.05), 0.0, math.cos(0.05))
    p = deck_pose((0.0, 0.0, 0.0) + tilt, TABLE.deck_height, spec, 0.0)
    assert to_local((0.0, 0.0, 0.0) + tilt, p[:3]) == pytest.approx((0.0, 0.0, 0.48), abs=1e-12)


def test_state_json():
    import json
    s = json.loads(state_json("attached", CargoSpec("small", (0.3, 0.2, 0.15), 2.0), "m", "d"))
    assert s == {"state": "attached", "item": "small", "mass": 2.0, "model": "m", "detail": "d",
                 "offset_m": None}
    assert json.loads(state_json("none", None, "m"))["mass"] == 0.0


# ---------------------------------------------------------------- xacro 규칙과 대조
def _xacro(**kw):
    if shutil.which("xacro") is None:
        pytest.skip("xacro 없음")
    args = ["xacro", str(XACRO), f"config_dir:={CONFIG}"] + [f"{k}:={v}" for k, v in kw.items()]
    r = subprocess.run(args, capture_output=True, text=True, check=False)
    if r.returncode != 0 and "find" in r.stderr:
        pytest.skip("amr_description 이 설치 공간에 없다 ($(find) 해석 불가)")
    assert r.returncode == 0, r.stderr
    return ET.fromstring(r.stdout)


@pytest.mark.parametrize("name,prefix", [("amr_01", ""), ("amr_04", "amr_04/"),
                                         ("amr_17", "amr_17/")])
def test_names_and_bits_match_xacro(name, prefix):
    root = _xacro(robot_name=name, prefix=prefix)
    dj = root.find(".//plugin[@name='ignition::gazebo::systems::DetachableJoint']")
    assert dj is not None
    assert dj.find("child_model").text == cargo_model_name(name)
    assert dj.find("child_link").text == "link"
    flags = {int(e.text) for e in root.iter("visibility_flags")}
    assert flags == {1 << self_visibility_bit(name)}
    # 데크 높이 = base_footprint_joint z + cargo_joint z

    def z(joint):
        return float(root.find(f"joint[@name='{prefix}{joint}']/origin").get("xyz").split()[2])
    z0, z1 = z("base_footprint_joint"), z("cargo_joint")
    assert z0 + z1 == pytest.approx(TABLE.deck_height)


def test_visibility_bit_rule():
    assert self_visibility_bit("amr_01") == 4
    assert self_visibility_bit("amr_05") == 8
    assert self_visibility_bit("amr_21") == 4          # 20대 주기
    assert self_visibility_bit("robot") == 4
    assert self_visibility_bit("amr_02", 12) == 12


# ---------------------------------------------------------------- 적재/하역 절차 (가짜 Gazebo)
class FakeGazebo:
    """
    DetachableJoint + UserCommands 흉내 (Fortress 6.18 실측 동작, urdf/amr_gazebo.xacro 머리말).

      초기 상태 = 붙이기 요청, 자식 모델이 생기면 그 스텝에 붙는다, detach 뒤에는 attach 요청이 있어야 다시 붙는다,
      이미 붙어 있으면 attach 무시 / 떨어져 있으면 detach 무시 (state 메시지 없음), 같은 이름 create 는 접수되지만 실패한다.
    """

    def __init__(self, child="amr_01_cargo", plugin=True,
                 pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)):
        self.child = child
        self.plugin = plugin
        self.models = {}
        self.attach_requested = True
        self.attached = False
        self.joint_log = []
        self.calls = []
        self.pose = pose
        self.slept = 0.0
        self.offset = 0.0

    def _step(self):
        if (self.plugin and self.attach_requested and not self.attached
                and self.child in self.models):
            self.attached, self.attach_requested = True, False
            self.joint_log.append("attached")

    # PayloadBackend
    def robot_pose(self, timeout):
        return (self.pose, "") if self.pose else (None, "")

    def create(self, name, sdf, pose):
        self.calls.append(("create", name, pose))
        if name not in self.models:
            self.models[name] = sdf
        self._step()
        return True

    def remove(self, name):
        self.calls.append(("remove", name))
        self.models.pop(name, None)
        return True

    def joint_seq(self):
        return len(self.joint_log)

    def request(self, attach):
        self.calls.append(("attach" if attach else "detach",))
        if attach:
            if not self.attached:
                self.attach_requested = True
        elif self.attached:
            self.attached = False
            self.joint_log.append("detached")
        self._step()

    def wait_joint(self, state, since, timeout):
        return state in self.joint_log[since:]

    def sleep(self, seconds):
        self.slept += seconds

    def cargo_in_robot(self, model):
        """로봇 기준 화물 위치 = 마지막 생성 자세 (로봇은 원점) + offset (생성 중 이동 흉내). None = 조회 실패."""
        if self.offset is None or model not in self.models:
            return None
        pose = [c for c in self.calls if c[0] == "create"][-1][2]
        return (pose[0] + self.offset, pose[1], pose[2])


def _manager(gz, **kw):
    states, logs = [], []
    import json
    mgr = PayloadManager(gz, TABLE, "amr_01_cargo", 1 << 4,
                         publish=lambda s: states.append(json.loads(s)), log=logs.append, **kw)
    return mgr, states, logs


def test_load_unload_reload_cycle():
    gz = FakeGazebo()
    mgr, states, _ = _manager(gz)
    large = TABLE.resolve("large", 25.0)
    assert mgr.apply(large)
    assert gz.attached and mgr.current == large
    assert [s["state"] for s in states] == ["loading", "attached"]
    create = next(c for c in gz.calls if c[0] == "create")
    assert create[2][2] == pytest.approx(0.33 + 0.20 + 0.002)
    assert "<mass>25</mass>" in gz.models["amr_01_cargo"]
    assert states[-1]["offset_m"] == 0.0

    assert mgr.apply(None)
    assert not gz.attached and "amr_01_cargo" not in gz.models and mgr.current is None
    assert [s["state"] for s in states[2:]] == ["unloading", "none"]

    # detach 뒤 재적재: attach 요청을 create 전에 보내야 붙는다 (보내지 않으면 가짜도 실제도 안 붙는다)
    small = TABLE.resolve("small", None)
    assert mgr.apply(small)
    assert gz.attached and states[-1]["item"] == "small" and states[-1]["mass"] == 2.0
    calls = [c[0] for c in gz.calls]
    last_create = len(calls) - 1 - calls[::-1].index("create")
    assert "attach" in calls[calls.index("remove"):last_create]


def test_change_item_while_loaded_unloads_first():
    gz = FakeGazebo()
    mgr, states, _ = _manager(gz)
    mgr.apply(TABLE.resolve("small", None))
    assert mgr.apply(TABLE.resolve("large", None))
    assert [s["state"] for s in states] == [
        "loading", "attached", "unloading", "loading", "attached"]
    assert "<mass>25</mass>" in gz.models["amr_01_cargo"]
    assert mgr.apply(TABLE.resolve("large", None))           # 같은 요청은 아무것도 하지 않는다
    assert len(states) == 5


def test_stale_attached_model_is_replaced():
    """이전 실행이 남긴 같은 이름 모델이 붙어 있으면: 첫 시도 실패 → 떼고 치우고 재시도."""
    gz = FakeGazebo()
    gz.models["amr_01_cargo"] = "<old/>"
    gz._step()
    assert gz.attached
    mgr, states, logs = _manager(gz)
    assert mgr.apply(TABLE.resolve("medium", 10.0))
    assert states[-1]["state"] == "attached" and states[-1]["detail"] == "재시도 후 성공"
    assert "<mass>10</mass>" in gz.models["amr_01_cargo"] and gz.slept > 0
    assert any("재시도" in m for m in logs)


def test_never_attaches_reports_error_and_cleans_up():
    gz = FakeGazebo(plugin=False)          # 로봇에 DetachableJoint 가 없는 경우 (runtime_payload:=false)
    mgr, states, _ = _manager(gz)
    assert not mgr.apply(TABLE.resolve("large", None))
    assert states[-1]["state"] == "error" and "large" in states[-1]["detail"]
    assert gz.models == {} and mgr.current is None and mgr.errors == 1


def test_no_robot_pose_is_error():
    gz = FakeGazebo(pose=None)
    mgr, states, _ = _manager(gz)
    assert not mgr.apply(TABLE.resolve("small", None))
    assert states[-1]["state"] == "error" and gz.models == {}


def test_moving_robot_warning_is_reported():
    class Moving(FakeGazebo):
        def robot_pose(self, timeout):
            return self.pose, "멈추지 않음"
    gz = Moving()
    mgr, states, logs = _manager(gz)
    assert mgr.apply(TABLE.resolve("small", None))
    assert states[-1]["detail"] == "멈추지 않음" and logs == ["amr_01_cargo: 멈추지 않음"]


def test_attached_offset_is_checked():
    """생성 중 로봇이 움직여 데크 중앙에서 어긋나 붙으면 경고하고 offset 을 싣는다. 조회 실패면 null."""
    gz = FakeGazebo()
    gz.offset = 0.05
    mgr, states, logs = _manager(gz)
    assert mgr.apply(TABLE.resolve("large", None))
    assert states[-1]["offset_m"] == pytest.approx(0.05) and "어긋나" in states[-1]["detail"]
    assert any("어긋나" in m for m in logs)
    gz2 = FakeGazebo()
    gz2.offset = None
    mgr2, states2, _ = _manager(gz2)
    assert mgr2.apply(TABLE.resolve("large", None))
    assert states2[-1]["offset_m"] is None and states2[-1]["state"] == "attached"


def test_unload_without_detach_message_still_removes():
    gz = FakeGazebo()
    mgr, states, logs = _manager(gz)
    mgr.apply(TABLE.resolve("small", None))
    gz.attached = False                       # 이미 떨어짐 → detach 에 state 메시지가 안 온다
    assert mgr.apply(None)
    assert gz.models == {} and any("detached 확인 없음" in m for m in logs)


def test_apply_none_when_empty_publishes_none():
    gz = FakeGazebo()
    mgr, states, _ = _manager(gz)
    assert mgr.apply(None) and states == [] and gz.calls == []
    mgr.current = CargoSpec("x", (0.1, 0.1, 0.1), 1.0)
    mgr.apply(None)
    assert states[-1]["state"] == "none"
