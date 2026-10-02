"""
payload_manager_node — 가상 적재/하역의 질량 변화를 Gazebo 동역학에 반영한다 (명세 4.1, 8장 "적재 시 질량 변화 반영").

로봇마다 1개, 네임스페이스 /<robot>. 보통 amr_description spawn.launch.py 가 스폰과 함께 띄운다
(payload_manager:=true 기본, amr_simulation 이 설치돼 있을 때) — amr_bringup 은 바꿀 필요가 없다.

    ros2 run amr_simulation payload_manager_node.py --ros-args -r __ns:=/amr_01 \
        -p use_sim_time:=true

동작 (결정)
    amr_behavior SimulateLoad/SimulateUnload 가 연달아 발행하는 payload/attach(종류, '' = 하역)와
    payload/mass([kg])를 같은 QoS(reliable + transient_local, depth 1)로 받아, 마지막 메시지 후 settle 초
    동안 조용하면 요청으로 확정한다.
    요청 → 화물 박스(amr_simulation/payload.py PayloadTable.resolve: 크기 = 종류의 물품 표 크기, 질량 = payload/mass)
      적재  로봇이 멈추기를 기다려(ground_truth/odom) 데크 중앙 +2 mm 에 모델 <robot>_cargo 를 만들고(UserCommands create)
            DetachableJoint 가 붙인 것("attached")을 확인한다. 로봇 모델의 플러그인(urdf/amr_gazebo.xacro "런타임 적재물")이
            부모 링크 base_footprint 에 고정 조인트로 묶는다 → DART 가 화물 바디를 로봇 스켈레톤에 넣어 질량·관성이 합산된다.
      하역  detach → "detached" 확인 → 모델 제거. 하역한 화물을 도크에 남기지 않는다 (결정: 다음 도킹의 장애물이 되고
            월드에 모델이 쌓인다 — 명세의 하역은 가상 이벤트다).
    화물을 월드 밖에 미리 세워 두고 옮겨 붙이는 방식은 쓰지 않는다: DetachableJoint 는 초기 상태가 "붙이기 요청"이라
    스폰 즉시 먼 화물을 그 자리 오프셋 그대로 로봇에 묶는다 (실측, urdf/amr_gazebo.xacro 머리말).
    붙이지 못하면(시간 초과) 남은 같은 이름 모델을 치우고 한 번 더 하고, 그래도 안 되면 화물을 치우고 error 를 발행한다.
    같은 요청으로 다시 시도하지 않는다 (새 요청이 오면 다시 한다).
    붙은 뒤 gz pose/info 로 화물이 데크 중앙에 있는지 잰다 (offset_m). 생성 자세는 지면 진실에서 계산하므로 요청~생성
    사이(`ign service` 호출 ≈ 0.3 s 벽시계)에 로봇이 움직이면 그만큼 어긋난 채 고정된다 — 2 cm 를 넘으면 경고.
    Fortress UserCommands 는 EntityFactory.relative_to 를 무시한다 (실측: 로봇 기준 (1, 0, 0.5) 요청이
    월드 (1, 0, 0.5) 에 생김) → 로봇 기준 생성으로 이 지연을 없앨 수 없다.

입력
    payload/attach   std_msgs/String   (reliable, transient_local, depth 1)
    payload/mass     std_msgs/Float32  (reliable, transient_local, depth 1)
    ground_truth/odom  nav_msgs/Odometry (월드 절대 자세 = map, 화물 생성 자세와 정지 판정)
    cargo/state      std_msgs/String   ← gz /<robot>/cargo/state
                                        (DetachableJoint output_topic, spawn.launch.py 브리지)
출력
    cargo/attach, cargo/detach   std_msgs/Empty → gz /<robot>/cargo/{attach,detach} (브리지)
    payload/sim_state  std_msgs/String JSON (reliable, transient_local, depth 1 — 중간 상태는 놓칠 수 있다)
        {"state": none|loading|attached|unloading|error, "item", "mass", "model", "detail",
         "offset_m": 붙은 화물의 데크 중앙 기준 위치 오차 [m] (gz pose/info, 확인 못 하면 null)}
    Gazebo 서비스 (`ign service`, amr_simulation/gz_cli.py): /world/<world>/create, .../remove
파라미터
    robot_name (기본 = 네임스페이스), world (warehouse),
    config_dir ($ROS_WS/config — robot_params.yaml 의 물품 표·데크 높이),
    self_visibility_bit (-1 = 이름 규칙, xacro 와 같게), settle 0.1 s, joint_timeout 5.0 s (벽시계),
    stationary_timeout 5.0 s, stationary_speed 0.05 m/s, deck_gap 0.002 m

실측 (Fortress 6.18, amr-fleet-system:wf-final, 2026-09-22, 1 ms 스텝, 바닥 μ 0.8, 지면 진실 50 Hz,
      cmd_vel 스텝 0 → 2.0 m/s → 0 (E-Stop 과 같은 0 지령), 기울기 = 0.2~1.8 m/s 구간 최소제곱)
    예측 = amr_simulation/payload.py DriveModel (바퀴 토크 3.0 N·m, 구름 저항 계수 0.0163 은 무적재 제동에서 맞춤)
    warehouse 월드 5대 (amr_bringup multi_robot.launch.py, 스택 끔, 헤드리스 + GPU 센서 렌더링, 센서 노이즈 켬),
    5대 동시 적재 → 같은 프로파일. 2회 실행 (RTF 0.33~0.46 / 부하 평균 31~40, RTF 0.11~0.17 / 부하 68~160,
    32 코어 공유 호스트) 모두 같은 값 (sim 시간 기준이라 RTF 와 무관):
      적재            가속 [m/s²] 실측/예측     제동 [m/s²] 실측/예측
      없음 (5대)      1.000 / 1.000            1.653 / 1.653 (보정점)
      소형 2 kg       1.000 / 1.000            1.592 / 1.594
      중형 10 kg      1.000 / 1.000            1.389 / 1.398
      대형 25 kg (3대) 0.842 / 0.831           1.135 / 1.146
      하역 뒤 (5대)   1.000                    1.653  (무적재로 복원)
    물리 전용 월드 1대 (test/test_payload_gazebo.py, RTF 1): 무적재 1.000 / 1.653 → 대형 0.843 / 1.134
    → 하역 1.000 / 1.653. 스폰 시점 적재(xacro payload, robot_params.yaml drive 표 25 kg 0.843 / 1.13)와
    같은 값 — 같은 질량이 같은 동역학을 낸다.
    안정성: 적재 직후와 적재 주행 뒤 gz pose/info 에서 화물의 로봇 기준 위치 = (0, 0, 데크 + 높이/2 + 2 mm)
    (대형 0.532, 중형 0.482, 소형 0.407 m, 0.1 mm 단위까지 같음, offset_m 0.0), 로봇 roll/pitch 0.0000 rad.
    IMU(imu/data_raw) 롤·피치 각속도 최대 0.0105~0.0108 rad/s — 무적재 주행과 같은 노이즈 수준.
    센서: 같은 자세에서 적재 전/후 스캔 20개씩 (5대, 2회) — 0.5 m 안 빔 0개(전·후), 최소 거리 1.70~1.72 m (옆 로봇).
    빔별 중앙값이 0.1 m 넘게 바뀐 빔은 로봇당 0~3개이고 모두 1.70 m 밖이다 (걷는 actor). 화물의 어느 점도 LiDAR
    (base_link 전방 0.15 m)에서 수평 0.52 m 안이고 스캔 평면 위(데크 +0.33 > 스캔 +0.20)라, 맞았다면 0.52 m 안에 나온다.
    카메라 6 프레임 평균의 전/후 차이 0.56 / 0.67 (0~255) < 같은 조건 잡음 바닥 0.73 / 0.88 (amr_01 / amr_04)
    → 자기 화물은 LiDAR·카메라에 들어오지 않는다.
    요청 → attached: 1대 0.5~0.55 s, 5대 동시 1.0 s (벽시계, pose/info 확인 포함). 하역 → none 0.4~0.9 s.
    비용: warehouse 5대 RTF 는 런타임 적재 경로 유무로 구분되지 않는다. 40 s 창, 교대 2회: 기준선(runtime_payload·
    payload_manager 끔) 0.321~0.416 (6 창, 부하 34~53), 이 경로 0.275~0.397 (무적재·5대 적재·하역 뒤 6 창, 부하
    31~74). 2회차는 기준선 0.399~0.416 (부하 35~53) / 이 경로 0.390~0.397 (부하 31~32) — 약 4 % 차이로, 같은 조건
    창끼리의 변동(1회차 기준선 0.321~0.397)보다 작아 이 측정으로는 비용을 가를 수 없다.
    물리 전용 5대의 서버 CPU 초 / sim 초도 차이 없음 (urdf/amr_gazebo.xacro "런타임 적재물").
"""

import math
import os
import threading
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Float32, String

from amr_simulation.gz_cli import GzWorld
from amr_simulation.payload import (DesiredPayload, PayloadManager, PayloadTable, cargo_model_name,
                                    self_visibility_bit, state_json, to_local)


def _default_config_dir() -> str:
    ros_ws = os.environ.get("ROS_WS", "")
    return os.path.join(ros_ws, "config") if ros_ws else "/ros2_ws/config"


def latched_qos() -> QoSProfile:
    """amr_behavior task_executor_node latchedQos() 와 같은 QoS."""
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class PayloadManagerNode(Node):
    """ROS 토픽 ↔ PayloadManager. Gazebo 호출(블로킹)은 작업 스레드에서 한다 — 콜백은 상태만 갱신한다."""

    POSE_FRESH = 2.0            # [s, 벽시계] 이보다 오래된 지면 진실은 없는 것으로 본다 (저 RTF 에서도 50 Hz sim 이면 충분)

    def __init__(self, gz=None):
        super().__init__("payload_manager_node")
        ns = self.get_namespace().strip("/")
        self.robot = self.declare_parameter("robot_name", ns or "amr_01").value
        world = self.declare_parameter("world", "warehouse").value
        config_dir = self.declare_parameter("config_dir", _default_config_dir()).value
        bit = int(self.declare_parameter("self_visibility_bit", -1).value)
        settle = float(self.declare_parameter("settle", 0.1).value)
        self.stationary_speed = float(self.declare_parameter("stationary_speed", 0.05).value)
        table = PayloadTable.from_robot_params(os.path.join(config_dir, "robot_params.yaml"))
        self.table = table
        self.model = cargo_model_name(self.robot)
        self.gz = gz or GzWorld(world)

        self._cv = threading.Condition()
        self.desired = DesiredPayload(settle)
        self._odom = None                       # (pose, 선속도, 각속도, 받은 시각)
        self._joint = deque(maxlen=64)          # (순번, state)
        self._joint_count = 0
        self._stop = False
        self._attempted = None                  # 실패한 요청의 stamp (같은 요청은 다시 하지 않는다)
        self._bad = None                        # 해석할 수 없는 요청 (경고 1회)

        latched = latched_qos()
        self.create_subscription(String, "payload/attach", self._on_attach, latched)
        self.create_subscription(Float32, "payload/mass", self._on_mass, latched)
        self.create_subscription(Odometry, "ground_truth/odom", self._on_odom, 10)
        self.create_subscription(String, "cargo/state", self._on_joint, 10)
        self.pub_attach = self.create_publisher(Empty, "cargo/attach", 10)
        self.pub_detach = self.create_publisher(Empty, "cargo/detach", 10)
        self.pub_state = self.create_publisher(String, "payload/sim_state", latched)

        self.mgr = PayloadManager(
            self, table, self.model, 1 << self_visibility_bit(self.robot, bit),
            publish=lambda s: self.pub_state.publish(String(data=s)),
            log=lambda s: self.get_logger().warn(s),
            gap=float(self.declare_parameter("deck_gap", 0.002).value),
            joint_timeout=float(self.declare_parameter("joint_timeout", 5.0).value),
            stationary_timeout=float(self.declare_parameter("stationary_timeout", 5.0).value))
        self.pub_state.publish(String(data=state_json("none", None, self.model)))
        self._worker = threading.Thread(target=self._run, name="payload_worker", daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"화물 모델 {self.model} (월드 {world}), 데크 높이 {table.deck_height:.3f} m, "
            f"물품 {', '.join(f'{k} {v[0]:g} kg' for k, v in table.items.items())}")

    # ---- ROS 콜백 (상태만 갱신) ----
    def _on_attach(self, msg: String) -> None:
        with self._cv:
            self.desired.on_attach(msg.data, time.monotonic())
            self._cv.notify_all()

    def _on_mass(self, msg: Float32) -> None:
        with self._cv:
            self.desired.on_mass(float(msg.data), time.monotonic())
            self._cv.notify_all()

    def _on_odom(self, m: Odometry) -> None:
        p, q, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        v = math.hypot(tw.linear.x, tw.linear.y)
        with self._cv:
            self._odom = (pose, v, abs(tw.angular.z), time.monotonic())
            self._cv.notify_all()

    def _on_joint(self, msg: String) -> None:
        with self._cv:
            self._joint_count += 1
            self._joint.append((self._joint_count, msg.data))
            self._cv.notify_all()

    # ---- PayloadBackend (작업 스레드에서 호출) ----
    def _fresh_odom(self):
        o = self._odom
        return o if o is not None and time.monotonic() - o[3] < self.POSE_FRESH else None

    def robot_pose(self, timeout: float):
        deadline = time.monotonic() + timeout
        with self._cv:
            while True:
                o = self._fresh_odom()
                if o is not None and o[1] < self.stationary_speed and o[2] < self.stationary_speed:
                    return o[0], ""
                rem = deadline - time.monotonic()
                if rem <= 0:
                    if o is None:
                        return None, ""
                    return o[0], (f"{timeout:.1f} s 안에 멈추지 않아 주행 중 자세로 생성 "
                                  f"(v {o[1]:.2f} m/s, w {o[2]:.2f} rad/s)")
                self._cv.wait(min(rem, 0.1))

    def create(self, name: str, sdf: str, pose) -> bool:
        return self.gz.create(name, sdf, pose)

    def remove(self, name: str) -> bool:
        return self.gz.remove(name)

    def joint_seq(self) -> int:
        with self._cv:
            return self._joint_count

    def request(self, attach: bool) -> None:
        (self.pub_attach if attach else self.pub_detach).publish(Empty())

    def wait_joint(self, state: str, since: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        with self._cv:
            while True:
                if any(n > since and s == state for n, s in self._joint):
                    return True
                rem = deadline - time.monotonic()
                if rem <= 0:
                    return False
                self._cv.wait(rem)

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def cargo_in_robot(self, model: str):
        """Gazebo pose/info 한 메시지의 로봇·화물 자세 → 로봇 좌표의 화물 위치 (같은 스텝 값이라 지연 무관)."""
        poses = self.gz.poses()
        robot, cargo = poses.get(self.robot), poses.get(model)
        return to_local(robot, cargo[:3]) if robot and cargo else None

    # ---- 작업 스레드 ----
    def step(self) -> bool:
        """확정된 요청이 현재와 다르면 한 번 적용한다. 적용을 시도했으면 True (테스트가 직접 부른다)."""
        with self._cv:
            req = self.desired.ready(time.monotonic())
            stamp = self.desired.stamp
            has_pose = self._fresh_odom() is not None
        if req is None:
            return False
        try:
            target = self.table.resolve(*req)
        except ValueError as e:
            if self._bad != stamp:
                self._bad = stamp
                self.get_logger().error(f"적재 요청 무시: {e}")
                self.pub_state.publish(String(data=state_json(
                    "error", self.mgr.current, self.model, str(e))))
            return False
        if target == self.mgr.current or self._attempted == stamp:
            return False
        if target is not None and not has_pose:
            return False                          # 로봇이 아직 없다 (스폰 전·일시정지) — 기다린다
        ok = self.mgr.apply(target)
        self._attempted = None if ok else stamp
        if ok:
            self.get_logger().info(
                f"{self.model}: " + (f"{target.item} {target.mass:g} kg 적재" if target else "하역"))
        return True

    def _run(self) -> None:
        while True:
            with self._cv:
                if self._stop:
                    return
                self._cv.wait(0.05)
                if self._stop:
                    return
            try:
                self.step()
            except Exception as e:   # 작업 스레드가 죽으면 이후 요청을 영영 못 받는다
                self.get_logger().error(f"적재 처리 예외: {e!r}")

    def destroy_node(self):
        with self._cv:
            self._stop = True
            self._cv.notify_all()
        self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PayloadManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl-C / 런치 종료(SIGINT)
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
