"""
충전 경로 통합 시험 (리뷰 회귀: battery_state 발행자가 없어 Charge 가 돌지 않았고, 모든 로봇이 charger_c1 을 썼다).

실제 task_executor_node 실행 파일 3 대(amr_01, amr_02, amr_04 — 번호대로면 amr_04 도 c1 이 첫 선택) +
프로세스 안 battery_model_node(잔량 15 %) + 모의 navigate_to_pose 서버. 기대: 셋 다 Charge 로 들어가 서로
다른 충전소 staging 으로 주행 goal 을 보내고, /…/charger_claims 로 점유를 나눈다.
"""

import json
import os
import subprocess
import sys
import threading
import time
import uuid

import pytest

rclpy = pytest.importorskip('rclpy')
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from amr_behavior.battery_model_node import BatteryModelNode  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402

EXE = os.environ.get('TASK_EXECUTOR_EXE', '')
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XML = os.path.join(PKG, 'behavior_trees', 'task_executor.xml')
CONFIG = os.path.join(PKG, 'config', 'behavior.yaml')
STAGING_X = {'charger_c1': -26.0, 'charger_c2': -22.0, 'charger_c3': -18.0}


class _MockNav:
    """navigate_to_pose: goal 을 기록하고 취소될 때까지 실행 중으로 둔다."""

    def __init__(self, node):
        self.goals = []
        self._stop = False
        group = ReentrantCallbackGroup()
        self.server = ActionServer(
            node, NavigateToPose, 'navigate_to_pose', execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)

    def _on_goal(self, goal):
        self.goals.append((goal.pose.pose.position.x, goal.pose.pose.position.y))
        return GoalResponse.ACCEPT

    def _execute(self, handle):
        while not self._stop and not handle.is_cancel_requested:
            time.sleep(0.05)
        handle.canceled()
        return NavigateToPose.Result()

    def stop(self):
        self._stop = True


@pytest.mark.skipif(not EXE or not os.path.exists(EXE), reason='TASK_EXECUTOR_EXE 가 없다')
def test_low_battery_robots_take_distinct_chargers(tmp_path):
    tag = uuid.uuid4().hex[:6]
    claims_topic = f'/c{tag}/charger_claims'
    robots = ['amr_01', 'amr_02', 'amr_04']
    rclpy.init()
    procs = []
    nodes = []
    executor = MultiThreadedExecutor(num_threads=6)
    claims = {}
    try:
        world = rclpy.create_node(f'charge_world_{tag}')
        nodes.append(world)

        def on_claim(msg):
            data = json.loads(msg.data)
            claims[data['robot']] = data['charger']
        world.create_subscription(String, claims_topic, on_claim, 10)
        executor.add_node(world)
        navs = {}
        for rid in robots:
            ns = f'/c{tag}/{rid}'
            nav_node = rclpy.create_node(f'mock_nav_{rid}', namespace=ns)
            navs[rid] = _MockNav(nav_node)
            battery = BatteryModelNode(namespace=ns, parameter_overrides=[
                Parameter('initial_percent', value=15.0)])
            for n in (nav_node, battery):
                executor.add_node(n)
                nodes.append(n)
            # behavior.yaml 의 노드별 키(/**/task_executor_node)는 -p 보다 우선하므로 덮어쓸 값도 파일로 준다
            override = tmp_path / f'{rid}.yaml'
            override.write_text(
                '/**/task_executor_node:\n  ros__parameters:\n'
                f'    robot_id: {rid}\n    bt_xml: {XML}\n    groot:\n      enabled: false\n'
                f'    charger_claims_topic: {claims_topic}\n', encoding='utf-8')
            procs.append(subprocess.Popen(
                [EXE, '--ros-args', '-r', f'__ns:={ns}', '--params-file', CONFIG,
                 '--params-file', str(override)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        spin = threading.Thread(target=executor.spin, daemon=True)
        spin.start()

        def latest():
            return {rid: navs[rid].goals[-1] for rid in robots if navs[rid].goals}

        end = time.monotonic() + 60.0
        while time.monotonic() < end:
            goals = latest()
            chosen = {rid: min(STAGING_X, key=lambda c: abs(STAGING_X[c] - g[0]))
                      for rid, g in goals.items()}
            if len(goals) == 3 and len(set(chosen.values())) == 3 and len(claims) == 3:
                break
            time.sleep(0.2)
        goals = latest()
        assert len(goals) == 3, {r: navs[r].goals for r in robots}
        for g in goals.values():
            assert abs(g[1] - (-16.64)) < 1e-6                 # 충전소 staging 줄
        chosen = {rid: min(STAGING_X, key=lambda c: abs(STAGING_X[c] - g[0]))
                  for rid, g in goals.items()}
        assert len(set(chosen.values())) == 3, chosen          # 서로 다른 충전소
        assert claims == chosen, (claims, chosen)              # 점유 공유와 주행 목표가 같다
        for nav in navs.values():
            nav.stop()
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:  # pragma: no cover
                p.kill()
        executor.shutdown()
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()
