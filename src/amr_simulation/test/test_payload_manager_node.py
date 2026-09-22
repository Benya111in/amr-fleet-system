"""
payload_manager_node ROS 계층 테스트 (Gazebo 없이, 한 프로세스 안).

실제 토픽으로 amr_behavior 와 같은 QoS(reliable + transient_local, depth 1)의 payload/attach·payload/mass 와
ground_truth/odom 을 보내고, Gazebo 호출은 DetachableJoint 처럼 반응하는 가짜(create → cargo/state "attached")로
바꿔 payload/sim_state 가 attached → none 으로 가는지, 요청 확정(settle)·잘못된 요청·로봇 없음 처리를 본다.
rclpy 가 없는 환경에서는 건너뛴다.
"""

import json
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Float32, String
    from amr_simulation import payload_manager_node as pmn
    HAVE_ROS = True
except Exception:   # rclpy·메시지 없는 환경
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason="rclpy/std_msgs/nav_msgs 없음")

CONFIG = Path(__file__).resolve().parents[3] / "config"
NS = "/pay_node_test"


class FakeGz:
    """create 가 오면 (attach 요청이 있었으면) 다음 순간 state "attached" 를 낸다 — DetachableJoint 흉내."""

    def __init__(self):
        self.node = None
        self.created, self.removed = [], []
        self.plugin = True

    def create(self, name, sdf, pose):
        self.created.append((name, sdf, pose))
        if self.plugin:
            self.node._on_joint(String(data="attached"))
        return True

    def remove(self, name):
        self.removed.append(name)
        return True

    def poses(self):
        """마지막으로 만든 화물이 데크 중앙에 있는 것처럼 (로봇 (3, -2), 회전 없음)."""
        if not self.created or len(self.removed) >= len(self.created):
            return {}
        _, _, pose = self.created[-1]
        return {"pay_node_test": (3.0, -2.0, 0.0, 0.0, 0.0, 0.0, 1.0), self.created[-1][0]: pose}


class EchoNode(pmn.PayloadManagerNode if HAVE_ROS else object):
    """detach 요청에 가짜 플러그인이 "detached" 로 답한다."""

    def request(self, attach):
        super().request(attach)
        if not attach and self.gz.created and len(self.gz.removed) < len(self.gz.created):
            self._on_joint(String(data="detached"))


@pytest.fixture
def env():
    rclpy.init(args=["--ros-args", "-r", f"__ns:={NS}", "-p", f"config_dir:={CONFIG}"])
    gz = FakeGz()
    node = EchoNode(gz=gz)
    gz.node = node
    helper = Node("pay_test_helper", namespace=NS)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(helper)
    states = []
    helper.create_subscription(String, "payload/sim_state",
                               lambda m: states.append(json.loads(m.data)), pmn.latched_qos())
    pubs = {"attach": helper.create_publisher(String, "payload/attach", pmn.latched_qos()),
            "mass": helper.create_publisher(Float32, "payload/mass", pmn.latched_qos()),
            "odom": helper.create_publisher(Odometry, "ground_truth/odom", 10)}

    def spin_until(cond, timeout=10.0, odom=True):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if odom:
                m = Odometry()
                m.pose.pose.position.x, m.pose.pose.position.y = 3.0, -2.0
                m.pose.pose.orientation.w = 1.0
                pubs["odom"].publish(m)
            ex.spin_once(timeout_sec=0.02)
            if cond():
                return True
        return False

    yield node, gz, pubs, states, spin_until
    ex.shutdown()
    node.destroy_node()
    helper.destroy_node()
    rclpy.shutdown()


def test_parameters_and_initial_state(env):
    node, gz, pubs, states, spin_until = env
    assert node.robot == "pay_node_test" and node.model == "pay_node_test_cargo"
    assert spin_until(lambda: states, odom=False)
    assert states[0]["state"] == "none" and states[0]["model"] == "pay_node_test_cargo"


def test_load_and_unload_through_topics(env):
    node, gz, pubs, states, spin_until = env
    pubs["attach"].publish(String(data="large"))
    pubs["mass"].publish(Float32(data=25.0))
    assert spin_until(lambda: states and states[-1]["state"] == "attached"), states
    assert states[-1]["item"] == "large" and states[-1]["mass"] == 25.0
    assert states[-1]["offset_m"] == pytest.approx(0.0, abs=1e-9)
    name, sdf, pose = gz.created[-1]
    assert name == "pay_node_test_cargo" and "<mass>25</mass>" in sdf
    assert pose[:3] == pytest.approx((3.0, -2.0, 0.33 + 0.20 + 0.002))
    n_created = len(gz.created)
    pubs["attach"].publish(String(data=""))
    pubs["mass"].publish(Float32(data=0.0))
    assert spin_until(lambda: states[-1]["state"] == "none"), states
    assert gz.removed == ["pay_node_test_cargo"] and len(gz.created) == n_created
    # 중간 상태(loading/unloading)는 depth 1 latched 라 구독자가 놓칠 수 있다 — 최종 상태만 본다


def test_invalid_request_and_failure_are_not_retried(env):
    node, gz, pubs, states, spin_until = env
    pubs["attach"].publish(String(data="pallet"))          # 모르는 종류 + 질량 없음
    assert spin_until(lambda: states and states[-1]["state"] == "error"), states
    assert "pallet" in states[-1]["detail"] and gz.created == []
    gz.plugin = False                                       # 붙지 않는 로봇
    node.mgr.joint_timeout = 0.2
    node.mgr.retry_pause = 0.0
    pubs["attach"].publish(String(data="small"))
    assert spin_until(lambda: len(gz.removed) >= 2 and states[-1]["state"] == "error",
                      timeout=15.0), states
    assert "small" in states[-1]["detail"]
    n = len(gz.created)
    assert n == 2 and gz.removed                            # 첫 시도 + 재시도, 화물은 치운다
    spin_until(lambda: False, timeout=1.0)
    assert len(gz.created) == n                             # 같은 요청은 다시 하지 않는다


def test_waits_for_robot_pose(env):
    node, gz, pubs, states, spin_until = env
    pubs["attach"].publish(String(data="small"))
    spin_until(lambda: False, timeout=0.6, odom=False)
    assert gz.created == [] and node.step() is False       # 지면 진실이 없으면 기다린다
    assert spin_until(lambda: states[-1]["state"] == "attached")


def test_robot_pose_backend(env):
    node, gz, pubs, states, spin_until = env
    assert node.robot_pose(0.05) == (None, "")
    m = Odometry()
    m.pose.pose.orientation.w = 1.0
    m.twist.twist.linear.x = 1.0
    node._on_odom(m)
    pose, warn = node.robot_pose(0.05)
    assert pose[6] == 1.0 and "멈추지 않아" in warn
    m.twist.twist.linear.x = 0.0
    node._on_odom(m)
    assert node.robot_pose(0.05) == ((0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0), "")
    seq = node.joint_seq()
    assert not node.wait_joint("attached", seq, 0.05)
    node._on_joint(String(data="attached"))
    assert node.wait_joint("attached", seq, 0.05)
    node.sleep(0.0)
    assert node.remove("x") and gz.removed == ["x"]
