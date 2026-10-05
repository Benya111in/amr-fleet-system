"""
Gazebo 회귀 시험 — 주행 중 적재/하역이 실제 동역학(질량)을 바꾸는가 (명세 4.1, 8장 "적재 시 질량 변화 반영").

물리 전용 월드(센서·렌더링 없음, 1 ms 스텝, 바닥 μ 0.8 = warehouse)에 실제 로봇 모델을 실제 런치로 띄운다:
  description.launch.py (robot_state_publisher)
  + spawn.launch.py (확인 스폰 + 브리지 + payload_manager_node)
그리고 amr_behavior SimulateLoad/Unload 와 같은 토픽·QoS 로 적재/하역을 요청하고, 같은 cmd_vel 스텝 프로파일
(0 → 2.0 m/s → 0: 가속 한계와 E-Stop 제동)을 무적재 → 대형 25 kg 적재 → 하역 순서로 준다. 지면 진실로 잰 가속·제동이
바퀴 토크 한계 모델(robot_params.yaml drive 표)대로 달라지고, 하역하면 되돌아와야 한다. 적재 주행 뒤 gz pose/info 로
화물이 데크 위 제자리(로봇 기준 (0, 0, 데크 + 높이/2 + 2 mm))에 붙어 있는지, 하역 뒤 화물 모델이 없어졌는지도 본다.
payload_manager_node 가 없거나 화물이 붙지 않으면(질량이 스폰 시점에만 정해지던 이전 동작) 실패한다.
(ground_truth/odom 의 z·roll·pitch 는 이 설정에서 0 으로 나와 자세 확인에 쓰지 않는다 — pose/info 를 쓴다.)

Gazebo(ign)·rclpy·설치된 amr_description 이 없으면 건너뛴다. 소요 ~1 분 (RTF 1 로 sim 약 40 s).
IGN_PARTITION 은 시험마다 새로 만들고, ROS_DOMAIN_ID 는 환경 값을 쓴다 (토픽은 amr_09 네임스페이스).
단독 실행: python3 test_payload_gazebo.py [결과.json] — 측정값을 출력한다.
"""

import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid
from pathlib import Path

import pytest

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32, String
    from geometry_msgs.msg import Twist
    from ament_index_python.packages import get_package_share_directory
    get_package_share_directory("amr_description")
    get_package_share_directory("amr_simulation")
    from amr_simulation.gz_cli import GzWorld
    from amr_simulation.payload import DriveModel, PayloadTable, roll_pitch, to_local
    HAVE_ROS = True
except Exception:   # rclpy·메시지·설치 공간이 없는 환경
    HAVE_ROS = False
    Node = object

pytestmark = pytest.mark.skipif(not HAVE_ROS or shutil.which("ign") is None
                                or shutil.which("ros2") is None,
                                reason="Gazebo(ign)/rclpy/설치된 amr_description 이 없다")

CONFIG = Path(__file__).resolve().parents[3] / "config"
ROBOT = "amr_09"
WORLD_NAME = "payload_test"
WORLD = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
  </world>
</sdf>
"""

V_CMD = 2.0            # [m/s] 스텝 지령 (최고 속도)
T_IDLE, T_RUN, T_STOP = 1.0, 3.5, 2.5   # [s, sim] 정지 → 2.0 지령 → 0 지령


def latched():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Probe(Node):
    """cmd_vel·적재 요청 발행, 지면 진실·payload/sim_state 기록."""

    def __init__(self):
        super().__init__("payload_gz_probe", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        ns = f"/{ROBOT}"
        self.cmd = self.create_publisher(Twist, f"{ns}/cmd_vel", 10)
        self.attach = self.create_publisher(String, f"{ns}/payload/attach", latched())
        self.mass = self.create_publisher(Float32, f"{ns}/payload/mass", latched())
        self.odom = []          # (t, 속력)
        self.state = None
        self.create_subscription(Odometry, f"{ns}/ground_truth/odom", self._on_odom, 50)
        self.create_subscription(String, f"{ns}/payload/sim_state",
                                 lambda m: setattr(self, "state", json.loads(m.data)), latched())

    def _on_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.odom.append((t, math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)))

    def spin_until(self, cond, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            if cond():
                return True
        return False

    def request(self, item, mass):
        self.attach.publish(String(data=item))
        self.mass.publish(Float32(data=float(mass)))

    def profile(self):
        """정지 T_IDLE → V_CMD 스텝 T_RUN → 0 스텝 T_STOP (sim 시간). (시작, 가속 시작, 정지 지령 시각, 기록)."""
        n0 = len(self.odom)
        assert self.spin_until(lambda: len(self.odom) > n0, 30.0), "ground_truth/odom 없음"
        t0 = self.odom[-1][0]
        phases = ((T_IDLE, 0.0), (T_IDLE + T_RUN, V_CMD), (T_IDLE + T_RUN + T_STOP, 0.0))
        last_pub, deadline = 0.0, time.monotonic() + 300.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.005)
            t = self.odom[-1][0] - t0
            if t >= phases[-1][0]:
                break
            v = next(v for end, v in phases if t < end)
            if time.monotonic() - last_pub > 0.02:
                msg = Twist()
                msg.linear.x = v
                self.cmd.publish(msg)
                last_pub = time.monotonic()
        self.cmd.publish(Twist())
        return t0 + T_IDLE, t0 + T_IDLE + T_RUN, self.odom[n0:]


def slope(samples):
    """최소제곱 기울기 dv/dt."""
    n = len(samples)
    mt = sum(t for t, _ in samples) / n
    mv = sum(v for _, v in samples) / n
    num = sum((t - mt) * (v - mv) for t, v in samples)
    den = sum((t - mt) ** 2 for t, _ in samples)
    return num / den


def measure(t_go, t_stop, rec, lo=0.2, hi=1.8):
    """가속(0.2 → 1.8 m/s 구간)·제동(1.8 → 0.2 m/s 구간) 최소제곱 기울기."""
    acc = [(t, v) for t, v in rec if t_go <= t < t_stop and lo <= v <= hi]
    brk = [(t, v) for t, v in rec if t >= t_stop and lo <= v <= hi]
    peak = max(v for t, v in rec if t < t_stop + 0.1)
    return {"accel": round(slope(acc), 4), "brake": round(-slope(brk), 4),
            "peak_v": round(peak, 3), "n_acc": len(acc), "n_brk": len(brk)}


def _popen(cmd, env, log):
    return subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)


def _signal(p, sig):
    try:
        os.killpg(p.pid, sig)
    except ProcessLookupError:
        pass


def _stop(procs):
    """SIGINT → 15 s 기다림 → 프로세스 그룹 SIGKILL (ros2 launch 의 자식까지 남기지 않는다)."""
    for p in procs:
        _signal(p, signal.SIGINT)
    deadline = time.monotonic() + 15.0
    for p in procs:
        try:
            p.wait(timeout=max(0.1, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            pass
    for p in procs:
        _signal(p, signal.SIGKILL)
        p.wait()


def _cargo_check(gz, model):
    """Gazebo pose/info 로 로봇 기준 화물 위치와 로봇 roll/pitch 를 잰다."""
    poses = gz.poses()
    robot, cargo = poses.get(ROBOT), poses.get(model)
    out = {"robot_roll_pitch": [round(a, 5) for a in roll_pitch(robot[3:])] if robot else None,
           "cargo_present": cargo is not None}
    if robot and cargo:
        out["cargo_in_robot"] = [round(c, 4) for c in to_local(robot, cargo[:3])]
        out["cargo_roll_pitch"] = [round(a, 5) for a in roll_pitch(cargo[3:])]
    return out


def run_scenario(logdir: Path) -> dict:
    """시험 본체. 측정값 dict (실패 조건은 호출 쪽이 판정)."""
    env = os.environ.copy()
    env["IGN_PARTITION"] = f"pay_test_{uuid.uuid4().hex[:8]}"
    world = logdir / f"{WORLD_NAME}.sdf"
    world.write_text(WORLD, encoding="utf-8")
    logs = {n: open(logdir / f"{n}.log", "w", encoding="utf-8")
            for n in ("gazebo", "clock", "description", "spawn")}
    procs = [
        _popen(["ign", "gazebo", "-s", "-r", str(world), "--force-version", "6"], env,
               logs["gazebo"]),
        _popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"], env, logs["clock"]),
        _popen(["ros2", "launch", "amr_description", "description.launch.py",
                f"robot_name:={ROBOT}", f"config_dir:={CONFIG}"], env, logs["description"]),
        _popen(["ros2", "launch", "amr_description", "spawn.launch.py", f"robot_name:={ROBOT}",
                f"world:={WORLD_NAME}", f"config_dir:={CONFIG}", "spawn_timeout:=120"], env,
               logs["spawn"]),
    ]
    old_partition = os.environ.get("IGN_PARTITION")
    os.environ["IGN_PARTITION"] = env["IGN_PARTITION"]      # gz pose/info 조회 (ign CLI)
    gz = GzWorld(WORLD_NAME)
    model = f"{ROBOT}_cargo"
    rclpy.init()
    probe = Probe()
    out = {"ign_partition": env["IGN_PARTITION"],
           "deck_height": PayloadTable.from_robot_params(str(CONFIG / "robot_params.yaml"))
           .deck_height}
    try:
        assert probe.spin_until(lambda: probe.state is not None and len(probe.odom) > 50, 180.0), \
            "로봇/payload_manager_node 가 뜨지 않음 (spawn.log)"
        assert probe.state["state"] == "none", probe.state
        probe.spin_until(lambda: False, 1.0)          # 스폰 착지
        out["unloaded"] = measure(*probe.profile())

        t0 = time.monotonic()
        probe.request("large", 25.0)
        assert probe.spin_until(lambda: probe.state["state"] == "attached", 60.0), probe.state
        out["attach_wall_s"] = round(time.monotonic() - t0, 2)
        out["attached_state"] = probe.state
        probe.spin_until(lambda: False, 1.0)
        out["after_attach"] = _cargo_check(gz, model)
        out["loaded_25kg"] = measure(*probe.profile())
        out["after_loaded_run"] = _cargo_check(gz, model)

        t0 = time.monotonic()
        probe.request("", 0.0)
        assert probe.spin_until(lambda: probe.state["state"] == "none", 60.0), probe.state
        out["detach_wall_s"] = round(time.monotonic() - t0, 2)
        probe.spin_until(lambda: False, 1.0)
        out["after_unload"] = _cargo_check(gz, model)
        out["unloaded_again"] = measure(*probe.profile())
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        _stop(procs[::-1])
        for f in logs.values():
            f.close()
        if old_partition is None:
            os.environ.pop("IGN_PARTITION", None)
        else:
            os.environ["IGN_PARTITION"] = old_partition
    return out


def test_runtime_payload_changes_dynamics():
    with tempfile.TemporaryDirectory(prefix="pay_gz_") as d:
        try:
            r = run_scenario(Path(d))
        except AssertionError:
            for name in ("spawn", "gazebo"):
                p = Path(d) / f"{name}.log"
                if p.exists():
                    print(f"---- {name}.log (끝)\n" + p.read_text(encoding="utf-8")[-4000:])
            raise
    print(json.dumps(r, indent=1, ensure_ascii=False))
    a, b, c = r["unloaded"], r["loaded_25kg"], r["unloaded_again"]
    for m in (a, b, c):
        assert m["n_acc"] >= 20 and m["n_brk"] >= 20, m
        assert m["peak_v"] > 1.9, m                     # 세 경우 모두 최고 속도에 닿는다
    # 바퀴 토크 한계 모델 (amr_simulation/payload.py DriveModel: 72.7 N, 총 질량 47.6 / 72.6 kg) 예측과 3 % 안
    #   무적재: 가속 1.0 (DiffDrive 한계), 제동 1.653 / 25 kg: 가속 0.830, 제동 1.146
    model = DriveModel.from_robot_params(str(CONFIG / "robot_params.yaml"))
    for m, kg in ((a, 0.0), (b, 25.0), (c, 0.0)):
        assert m["accel"] == pytest.approx(model.accel(kg), rel=0.03), (kg, m)
        assert m["brake"] == pytest.approx(model.brake(kg), rel=0.03), (kg, m)
    assert a["accel"] - b["accel"] > 0.1 and a["brake"] - b["brake"] > 0.3
    # 하역하면 무적재 동역학으로 돌아온다
    assert c["accel"] == pytest.approx(a["accel"], abs=0.03)
    assert c["brake"] == pytest.approx(a["brake"], abs=0.05)
    assert r["attached_state"]["item"] == "large" and r["attached_state"]["mass"] == 25.0
    assert r["attached_state"]["offset_m"] < 0.005           # 매니저의 pose/info 자체 확인
    # 화물은 데크 중앙 위 제자리에 강체로 붙어 있고 (가감속 뒤에도 1 mm 안), 하역 뒤에는 월드에서 사라진다
    expect = [0.0, 0.0, r["deck_height"] + 0.40 / 2.0 + 0.002]
    for key in ("after_attach", "after_loaded_run"):
        chk = r[key]
        assert chk["cargo_present"], chk
        assert chk["cargo_in_robot"] == pytest.approx(expect, abs=1e-3), chk
        assert max(map(abs, chk["robot_roll_pitch"])) < 0.01, chk
    assert not r["after_unload"]["cargo_present"]


if __name__ == "__main__":
    with tempfile.TemporaryDirectory(prefix="pay_gz_") as d:
        res = run_scenario(Path(d))
    print(json.dumps(res, indent=1, ensure_ascii=False))
    if len(sys.argv) > 1:
        Path(sys.argv[1]).write_text(json.dumps(res, indent=1, ensure_ascii=False),
                                     encoding="utf-8")
