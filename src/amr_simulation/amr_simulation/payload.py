"""
런타임 적재물 — payload_manager_node 의 ROS·Gazebo 와 무관한 부분 (명세 4.1 "적재 질량 변화가 동역학에 반영", 8장 물품 표).

    PayloadTable     robot_params.yaml 의 payload.<종류>.{mass, size} 와 데크 높이
    CargoSpec        실을 화물 박스 (종류, 크기, 질량, 균질 박스 관성)
    DesiredPayload   payload/attach·payload/mass 두 토픽을 묶어 "안정된 요청" 하나로 만든다 (settle)
    cargo_sdf        Gazebo 화물 모델 SDF (자기 센서에서 빼는 가시성 비트 포함)
    deck_pose        로봇 자세(지면 진실) → 데크 위 화물 중심 자세
    PayloadManager   적재/하역 절차 (화물 생성 → attach 확인 / detach 확인 → 제거). Gazebo·ROS 호출은 backend 로 주입
    DriveModel       바퀴 토크 한계 모델 — 적재 질량별 가속·제동 예측 (Gazebo 실측과 대조하는 기준)

이름 규칙 (urdf/amr.urdf.xacro, amr_gazebo.xacro "런타임 적재물" 과 같아야 한다 — test 가 xacro 출력과 대조)
    화물 모델 <robot_name>_cargo, 링크 link, 가시성 비트 4 + (이름 끝 숫자 − 1) mod 20
"""

import json
import math
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Sequence, Tuple

import yaml

CARGO_LINK = "link"
CARGO_MU = 0.6                   # 상자 바닥 마찰 (models/box_* 와 같음, robot_params.yaml limits 주석)
CARGO_RGBA = "0.72 0.55 0.30 1"  # urdf/amr_materials.xacro amr_cargo


def cargo_model_name(robot_name: str) -> str:
    """화물 모델 이름 = DetachableJoint 의 child_model (amr.urdf.xacro cargo_model)."""
    return f"{robot_name}_cargo"


def self_visibility_bit(robot_name: str, override: int = -1) -> int:
    """amr.urdf.xacro 의 자기 차체 가시성 비트 규칙 (override ≥ 0 이면 그 값)."""
    if override >= 0:
        return override
    digits = robot_name[len(robot_name.rstrip("0123456789")):]
    return 4 + (int(digits) - 1) % 20 if digits else 4


@dataclass(frozen=True)
class CargoSpec:
    """데크에 실을 균질 박스."""

    item: str
    size: Tuple[float, float, float]     # [m] x(전후), y(좌우), z
    mass: float                          # [kg]

    def inertia(self) -> Tuple[float, float, float]:
        """균질 박스 대각 관성 [kg m^2] (amr_inertial.xacro inertial_box 와 같은 식)."""
        x, y, z = self.size
        k = self.mass / 12.0
        return k * (y * y + z * z), k * (x * x + z * z), k * (x * x + y * y)


@dataclass(frozen=True)
class PayloadTable:
    """물품 표 (종류 → (질량, 크기))와 base_footprint 기준 데크 상면 높이."""

    items: Dict[str, Tuple[float, Tuple[float, float, float]]]
    deck_height: float

    @classmethod
    def from_robot_params(cls, path: str) -> "PayloadTable":
        with open(path, encoding="utf-8") as f:
            p = yaml.safe_load(f)["/**"]["ros__parameters"]
        items = {name: (float(v["mass"]), tuple(float(s) for s in v["size"]))
                 for name, v in p["payload"].items()}
        r = p["robot"]
        return cls(items, float(r["base_link_height"]) + float(r["height"]) / 2.0)

    def smallest_for(self, mass: float) -> str:
        """그 질량을 담는 가장 작은 종류 (표 질량 기준, 넘으면 가장 큰 종류) — xacro payload_mass 규칙."""
        order = sorted(self.items, key=lambda n: self.items[n][0])
        for name in order:
            if mass <= self.items[name][0]:
                return name
        return order[-1]

    def resolve(self, item: str, mass: Optional[float]) -> Optional[CargoSpec]:
        """
        (payload/attach, payload/mass) → 실을 화물. None = 적재물 없음.

        종류가 표에 있으면 그 크기, 질량은 payload/mass (없거나 0 이하면 표 질량 — publish_payload_mass:=false 대비).
        종류가 비었거나 표에 없고 질량만 양수면 그 질량을 담는 가장 작은 박스. 둘 다 없으면 없음 / ValueError.
        """
        name = (item or "").strip().lower()
        has_mass = mass is not None and math.isfinite(mass) and mass > 0.0
        if name in ("", "none"):
            if not has_mass:
                return None
            name = self.smallest_for(mass)
        elif name not in self.items:
            if not has_mass:
                raise ValueError(f"물품 표에 없는 종류 '{item}' (질량도 없음)")
            name = self.smallest_for(mass)
        table_mass, size = self.items[name]
        return CargoSpec(name, size, float(mass) if has_mass else table_mass)


GRAVITY = 9.8                    # SDF 월드 기본 중력 (9.80665 아님)
# 구름 저항 계수 (무차원, 저항력 = ROLLING · 총 질량 · g). 무적재 E-Stop 제동 실측 1.653 m/s² 에서 맞춘 값:
# (48.6 · 1.653 − 72.73) / (47.6 · 9.8) = 0.0163 ≈ 캐스터 μ 0.05 × 캐스터가 받는 하중 비율 약 1/3.
ROLLING_COEFF = 0.0163


@dataclass(frozen=True)
class DriveModel:
    """
    바퀴 토크 한계 모델 (robot_params.yaml drive 주석) — 적재 질량 → 가속·제동 상한 [m/s²].

        F = 2 · wheel_effort_limit / wheel_radius              (구동 2륜 최대 견인력, 72.7 N)
        m_eff = 공차 + 적재 + 2 · I_axis / r²                   (바퀴 회전 관성의 등가 질량 1.0 kg)
        가속 = min(max_linear_acceleration, (F − c·m·g) / m_eff)
        제동 = min(emergency_deceleration,  (F + c·m·g) / m_eff)   (구름 저항은 가속을 빼고 제동을 돕는다)
    """

    empty_mass: float
    force: float
    wheel_inertia_mass: float
    accel_limit: float
    brake_limit: float
    rolling: float = ROLLING_COEFF

    @classmethod
    def from_robot_params(cls, path: str) -> "DriveModel":
        with open(path, encoding="utf-8") as f:
            p = yaml.safe_load(f)["/**"]["ros__parameters"]
        r, d, lim = p["robot"], p["drive"], p["limits"]
        radius = float(r["wheel_radius"])
        empty = float(r["base_mass"]) + 2 * float(r["wheel_mass"]) + 2 * float(r["caster_mass"])
        return cls(empty, 2.0 * float(d["wheel_effort_limit"]) / radius,
                   2.0 * float(r["wheel_inertia"][1]) / radius ** 2,
                   float(lim["max_linear_acceleration"]), float(lim["emergency_deceleration"]))

    def _terms(self, payload_mass: float) -> Tuple[float, float]:
        m = self.empty_mass + payload_mass
        return self.rolling * m * GRAVITY, m + self.wheel_inertia_mass

    def accel(self, payload_mass: float) -> float:
        rr, m_eff = self._terms(payload_mass)
        return min(self.accel_limit, (self.force - rr) / m_eff)

    def brake(self, payload_mass: float) -> float:
        rr, m_eff = self._terms(payload_mass)
        return min(self.brake_limit, (self.force + rr) / m_eff)


class DesiredPayload:
    """
    payload/attach 와 payload/mass 두 토픽을 요청 하나로 묶는다.

    두 토픽은 따로 온다(BT 가 연달아 발행, 도착 순서 보장 없음). 마지막 메시지 후 settle 초 동안 조용하면
    그 쌍을 요청으로 확정한다.
    """

    def __init__(self, settle: float):
        self.settle = settle
        self.item: Optional[str] = None
        self.mass: Optional[float] = None
        self.stamp: Optional[float] = None   # 마지막 메시지 시각 (단조 시계)

    def on_attach(self, item: str, now: float) -> None:
        self.item, self.stamp = item, now

    def on_mass(self, mass: float, now: float) -> None:
        self.mass, self.stamp = mass, now

    def ready(self, now: float) -> Optional[Tuple[str, Optional[float]]]:
        """확정된 (item, mass). 아직 메시지가 없거나 settle 중이면 None."""
        if self.stamp is None or now - self.stamp < self.settle:
            return None
        return (self.item or "", self.mass)


def cargo_sdf(model_name: str, spec: CargoSpec, visibility_flags: int) -> str:
    """화물 모델 SDF. 링크 원점 = 박스 중심 (create 자세 = 박스 중심)."""
    sx, sy, sz = spec.size
    ixx, iyy, izz = spec.inertia()
    box = f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>"
    return (
        f'<sdf version="1.9"><model name="{model_name}"><link name="{CARGO_LINK}">'
        f"<inertial><mass>{spec.mass:.6g}</mass><inertia>"
        f"<ixx>{ixx:.6g}</ixx><iyy>{iyy:.6g}</iyy><izz>{izz:.6g}</izz>"
        f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
        f'<collision name="collision">{box}<surface><friction><ode>'
        f"<mu>{CARGO_MU}</mu><mu2>{CARGO_MU}</mu2></ode></friction></surface></collision>"
        f'<visual name="visual">{box}<material><ambient>{CARGO_RGBA}</ambient>'
        f"<diffuse>{CARGO_RGBA}</diffuse></material>"
        f"<visibility_flags>{visibility_flags}</visibility_flags></visual>"
        f"</link></model></sdf>")


Pose = Tuple[float, float, float, float, float, float, float]   # x y z qx qy qz qw


def rotate(q: Sequence[float], v: Sequence[float]) -> Tuple[float, float, float]:
    """쿼터니언 (x, y, z, w) 로 벡터를 돌린다."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # t = 2 q_v × v,  v' = v + w t + q_v × t
    tx, ty, tz = 2 * (qy * vz - qz * vy), 2 * (qz * vx - qx * vz), 2 * (qx * vy - qy * vx)
    return (vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx))


def to_local(frame: Pose, point: Sequence[float]) -> Tuple[float, float, float]:
    """월드 점 → frame 자세 좌표 (검증용: 데크 위 화물의 로봇 기준 위치)."""
    x, y, z, qx, qy, qz, qw = frame
    return rotate((-qx, -qy, -qz, qw), (point[0] - x, point[1] - y, point[2] - z))


def roll_pitch(q: Sequence[float]) -> Tuple[float, float]:
    """쿼터니언 (x, y, z, w) → (roll, pitch) [rad]."""
    qx, qy, qz, qw = q
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    return roll, pitch


def deck_pose(robot: Pose, deck_height: float, spec: CargoSpec, gap: float) -> Pose:
    """로봇 모델 원점(base_footprint) 자세 → 데크 중앙 위 gap 만큼 띄운 화물 중심 자세 (자세 회전은 로봇과 같다)."""
    x, y, z, qx, qy, qz, qw = robot
    dx, dy, dz = rotate((qx, qy, qz, qw), (0.0, 0.0, deck_height + spec.size[2] / 2.0 + gap))
    return (x + dx, y + dy, z + dz, qx, qy, qz, qw)


def state_json(state: str, spec: Optional[CargoSpec], model: str, detail: str = "",
               offset: Optional[float] = None) -> str:
    """payload/sim_state 메시지 본문. offset = 붙은 화물의 데크 중앙 기준 위치 오차 [m] (확인 못 하면 null)."""
    return json.dumps({"state": state, "item": spec.item if spec else "",
                       "mass": round(spec.mass, 3) if spec else 0.0,
                       "model": model, "detail": detail,
                       "offset_m": None if offset is None else round(offset, 4)},
                      ensure_ascii=False)


class PayloadBackend:
    """PayloadManager 가 쓰는 외부 호출 (payload_manager_node 가 Gazebo/ROS 로 구현, 테스트는 가짜)."""

    def robot_pose(self, timeout: float) -> Tuple[Optional[Pose], str]:   # pragma: no cover
        """정지한 로봇 자세 (timeout 안에 정지하지 않으면 마지막 자세 + 경고 문자열). 자세가 없으면 None."""
        raise NotImplementedError

    def create(self, name: str, sdf: str, pose: Pose) -> bool:   # pragma: no cover
        raise NotImplementedError

    def remove(self, name: str) -> bool:   # pragma: no cover
        raise NotImplementedError

    def joint_seq(self) -> int:   # pragma: no cover
        """지금까지 받은 DetachableJoint state 메시지 수 (wait_joint 의 기준점)."""
        raise NotImplementedError

    def request(self, attach: bool) -> None:   # pragma: no cover
        """cargo/attach 또는 cargo/detach 발행."""
        raise NotImplementedError

    def wait_joint(self, state: str, since: int, timeout: float) -> bool:   # pragma: no cover
        """순번 since 뒤에 state("attached"/"detached") 메시지가 오면 True."""
        raise NotImplementedError

    def sleep(self, seconds: float) -> None:   # pragma: no cover
        raise NotImplementedError

    def cargo_in_robot(self, model: str) -> Optional[Tuple[float, ...]]:   # pragma: no cover
        """Gazebo 가 보는 화물 위치 (로봇 모델 좌표). 조회 실패면 None."""
        raise NotImplementedError


class PayloadManager:
    """
    화물 모델 하나(<robot>_cargo)의 적재/하역 절차. 성공하면 current 가 바뀌고, 실패하면 화물을 치우고 None 으로 둔다.

    적재   attach 요청 → create(데크 위 +gap) → "attached" 대기. detach 뒤에는 attach 요청이 있어야 다시 붙고, 첫 적재는
           플러그인의 초기 붙이기 요청으로 create 만으로 붙는다 — 어느 경우든 요청을 먼저 보내 두면 모델이 생기는 스텝에 붙는다.
           시간 안에 안 붙으면(이전 실행이 남긴 같은 이름 모델 때문에 create 가 거부된 경우 등) 한 번 치우고 다시 한다.
    하역   detach 요청 → "detached" 대기(이미 떨어져 있으면 메시지가 안 오므로 시간 초과도 진행) → remove.
    """

    def __init__(self, backend: PayloadBackend, table: PayloadTable, model: str,
                 visibility_flags: int, publish: Callable[[str], None], log: Callable[[str], None],
                 gap: float = 0.002, joint_timeout: float = 5.0, stationary_timeout: float = 5.0,
                 retry_pause: float = 0.5, offset_tolerance: float = 0.02):
        self.b = backend
        self.table = table
        self.model = model
        self.flags = visibility_flags
        self.publish = publish
        self.log = log
        self.gap = gap
        self.joint_timeout = joint_timeout
        self.stationary_timeout = stationary_timeout
        self.retry_pause = retry_pause
        self.offset_tolerance = offset_tolerance
        self.current: Optional[CargoSpec] = None
        self.errors = 0

    def _state(self, state: str, spec: Optional[CargoSpec], detail: str = "",
               offset: Optional[float] = None) -> None:
        self.publish(state_json(state, spec, self.model, detail, offset))

    def _attached(self, spec: CargoSpec, detail: str) -> bool:
        """붙은 뒤 Gazebo 자세로 데크 위치를 확인한다 (생성 지연 동안 로봇이 움직였으면 그만큼 어긋나 고정된다)."""
        self.current = spec
        got = self.b.cargo_in_robot(self.model)
        offset = None
        if got is not None:
            want = (0.0, 0.0, self.table.deck_height + spec.size[2] / 2.0 + self.gap)
            offset = math.dist(got, want)
            if offset > self.offset_tolerance:
                msg = f"데크 중앙에서 {offset:.3f} m 어긋나 붙음 (생성 중 로봇 이동)"
                self.log(f"{self.model}: {msg}")
                detail = f"{detail}; {msg}" if detail else msg
        self._state("attached", spec, detail, offset)
        return True

    def apply(self, target: Optional[CargoSpec]) -> bool:
        """현재 적재(current)를 target 으로 바꾼다. 같으면 아무것도 하지 않는다. 성공하면 True."""
        if target == self.current:
            return True
        if self.current is not None:
            self.unload()
        if target is None:
            self._state("none", None)
            return True
        return self.load(target)

    def unload(self) -> None:
        spec = self.current
        self._state("unloading", spec)
        seq = self.b.joint_seq()
        self.b.request(attach=False)
        if not self.b.wait_joint("detached", seq, self.joint_timeout):
            self.log(f"{self.model}: detached 확인 없음 ({self.joint_timeout:.1f} s) — 제거 진행")
        self.b.remove(self.model)
        self.current = None

    def _attempt(self, spec: CargoSpec, pose: Pose) -> bool:
        seq = self.b.joint_seq()
        self.b.request(attach=True)
        if not self.b.create(self.model, cargo_sdf(self.model, spec, self.flags), pose):
            return False
        return self.b.wait_joint("attached", seq, self.joint_timeout)

    def load(self, spec: CargoSpec) -> bool:
        self._state("loading", spec)
        robot, warn = self.b.robot_pose(self.stationary_timeout)
        if robot is None:
            self._state("error", None, "로봇 자세(ground_truth/odom) 없음")
            self.errors += 1
            return False
        if warn:
            self.log(f"{self.model}: {warn}")
        pose = deck_pose(robot, self.table.deck_height, spec, self.gap)
        if self._attempt(spec, pose):
            return self._attached(spec, warn)
        # 같은 이름의 남은 모델(이전 실행) 등: 떼고 치운 뒤 한 번 더
        self.log(f"{self.model}: {self.joint_timeout:.1f} s 안에 붙지 않음 — 남은 모델을 치우고 재시도")
        self.b.request(attach=False)
        self.b.remove(self.model)
        self.b.sleep(self.retry_pause)
        robot, _ = self.b.robot_pose(self.stationary_timeout)
        if robot is not None:
            pose = deck_pose(robot, self.table.deck_height, spec, self.gap)
        if self._attempt(spec, pose):
            return self._attached(spec, "재시도 후 성공")
        self.b.request(attach=False)
        self.b.remove(self.model)
        self.current = None
        self.errors += 1
        self._state("error", None, f"{spec.item} {spec.mass:g} kg 화물을 붙이지 못함 (attached 없음)")
        return False
