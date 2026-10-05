"""
collision_monitor_node / obstacle_truth_node 단위 테스트 (Gazebo 없이, 콜백 직접 호출).

- 충돌 판정: 로봇 두 대가 겹치면 접촉 1건(같은 사건은 contact_release 이상 벌어질 때까지 다시 세지 않음),
  지게차 오도메트리를 받으면 장애물로 들어가고, 요약 JSON·리셋 서비스가 동작한다.
- 지면 진실: info JSON 에 actor 6 + 차량 2, tick 마다 장애물 수만큼 마커·트랙을 낸다 (차량 마커는 발자국 중심).
rclpy 가 없는 환경에서는 건너뛴다.
"""

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from std_srvs.srv import Trigger
    from amr_simulation.collision_monitor_node import CollisionMonitorNode
    from amr_simulation.obstacle_truth_node import ObstacleTruthNode, _stamp
    HAVE_ROS = True
except Exception:   # rclpy·메시지 없는 환경
    HAVE_ROS = False

pytestmark = pytest.mark.skipif(not HAVE_ROS, reason="rclpy/amr_msgs 없음")

PKG = Path(__file__).resolve().parents[1]
CONFIG = PKG.parents[1] / "config"
WORLD = PKG / "worlds" / "warehouse.sdf"
PARAMS = PKG / "config" / "dynamic_obstacles.yaml"


@pytest.fixture
def ros(tmp_path):
    # 노드별 키(collision_monitor_node: robots)가 전역 -p 보다 우선하므로 두 번째 파라미터 파일로 덮는다
    override = tmp_path / "override.yaml"
    override.write_text("collision_monitor_node:\n  ros__parameters:\n"
                        "    robots: [amr_01, amr_02]\n", encoding="utf-8")
    rclpy.init(args=["--ros-args", "--params-file", str(PARAMS), "--params-file", str(override),
                     "-p", f"world_file:={WORLD}", "-p", f"config_dir:={CONFIG}"])
    yield
    rclpy.shutdown()


def _odom(t, x, y, yaw=0.0, v=0.0):
    m = Odometry()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.twist.twist.linear.x = v
    return m


class Capture:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


def test_collision_monitor_contacts_and_summary(ros):
    node = CollisionMonitorNode()
    try:
        assert node.robots == ["amr_01", "amr_02"]
        assert node.bounds == pytest.approx((-0.30, 0.30, -0.20, 0.20))
        events, summary = Capture(), Capture()
        node.pub_events, node.pub_summary = events, summary
        # 남동 대기 구역 (사람 경로에서 ≥ 6 m): 두 로봇이 겹친다 → 접촉 1건
        node._on_robot("amr_01", _odom(10.0, 22.0, -16.0))
        node._on_robot("amr_02", _odom(10.0, 22.2, -16.0))
        assert node.contacts["amr_02"] == 1 and len(events.msgs) == 1
        ev = json.loads(events.msgs[0].data)
        assert ev["robot"] == "amr_02" and ev["obstacle"] == "amr_01" and ev["distance"] < 0
        node._on_robot("amr_02", _odom(10.02, 22.25, -16.0))      # 같은 사건
        assert node.contacts["amr_02"] == 1
        node._on_robot("amr_02", _odom(10.5, 26.0, -16.0))        # 떨어짐 → 해제
        node._on_robot("amr_02", _odom(11.0, 22.1, -16.0))        # 다시 겹침 → 새 사건
        assert node.contacts["amr_02"] == 2
        # 지게차 오도메트리 → 장애물로 들어온다
        node._on_model("forklift_main", _odom(11.0, 0.0, 15.0, v=1.5))
        node._on_robot("amr_01", _odom(11.0, 0.0, 17.5))
        assert "forklift_main" in node.per_obstacle_min["amr_01"]
        node._summary()
        s = json.loads(summary.msgs[-1].data)
        assert s["amr_02"]["contacts"] == 2
        assert s["amr_01"]["min_distance"] <= s["amr_01"]["current"]
        res = node._on_reset(Trigger.Request(), Trigger.Response())
        assert res.success and node.contacts == {"amr_01": 0, "amr_02": 0}
    finally:
        node.destroy_node()


def test_obstacle_truth_info_and_tick(ros):
    node = ObstacleTruthNode()
    try:
        info = node._info()
        kinds = [o["kind"] for o in info]
        assert kinds.count("person") == 6 and kinds.count("vehicle") == 2
        assert sorted(o["id"] for o in info) == list(range(8))
        markers, tracks = Capture(), Capture()
        node.pub_markers, node.pub_tracks = markers, tracks
        node._tick()
        assert len(markers.msgs[-1].markers) == 6            # 차량은 오도메트리 전이라 없다
        now = node.get_clock().now().nanoseconds * 1e-9
        node._on_odom("shuttle_amr", _odom(now, 9.0, 2.0, yaw=math.pi / 2, v=1.0))
        node._tick()
        mk = {m.ns: m for m in markers.msgs[-1].markers}
        tr = tracks.msgs[-1].obstacles
        assert len(mk) == 7 and len(tr) == 7
        shuttle = mk["shuttle_amr"]
        assert shuttle.scale.x == pytest.approx(0.60) and shuttle.scale.y == pytest.approx(0.40)
        st = next(t for t in tr if t.track_id == node.ids["shuttle_amr"])
        assert st.velocity.y == pytest.approx(1.0, abs=1e-6)
        assert _stamp(12.5).sec == 12 and _stamp(12.5).nanosec == 500000000
    finally:
        node.destroy_node()
