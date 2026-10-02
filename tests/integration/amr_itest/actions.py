"""
시나리오 공통 동작: 주행 명령 발행, 정지 대기, 장애물 주입 (프로브 위에서 동작).

모든 시간 구간은 ROS 시각(probe.now())으로 잰다 — 운동학 백엔드에선 wall, Gazebo 에선 sim time.
wall 상한(wall_cap)은 sim time 이 멈추거나 매우 느릴 때 테스트가 끝없이 기다리지 않게 한다.
"""

import json
import math
import threading
import time
from typing import Iterable, List, Optional, Sequence, Tuple

from amr_itest import metrics
from amr_itest.probe import GraphProbe, TopicRecord
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def twist(v: float = 0.0, w: float = 0.0) -> Twist:
    msg = Twist()
    msg.linear.x = float(v)
    msg.angular.z = float(w)
    return msg


def pose_stamped(x: float, y: float, yaw: float = 0.0, frame: str = 'map'):
    """geometry_msgs/PoseStamped (평면 자세, 스탬프 0 = 최신 TF)."""
    from geometry_msgs.msg import PoseStamped
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y = float(x), float(y)
    msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return msg


EKF_SET_POSE = 'ekf_filter_node_map/set_pose'   # amr_localization localization.launch.py 리맵 이름
AMCL_NOMOTION = 'request_nomotion_update'        # nav2_amcl 서비스 (std_srvs/Empty)


def initial_pose(x: float, y: float, yaw: float, sigma_xy: float = 0.05,
                 sigma_yaw_deg: float = 3.0, frame: str = 'map'):
    """geometry_msgs/PoseWithCovarianceStamped (initialpose · EKF set_pose 공용)."""
    from geometry_msgs.msg import PoseWithCovarianceStamped
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame
    msg.pose.pose = pose_stamped(x, y, yaw, frame).pose
    msg.pose.covariance[0] = msg.pose.covariance[7] = sigma_xy ** 2
    msg.pose.covariance[35] = math.radians(sigma_yaw_deg) ** 2
    return msg


def seed_pose(probe: GraphProbe, ns: str, x: float, y: float, yaw: float,
              timeout: float = 5.0, repeats: int = 3) -> dict:
    """
    배치 단계: 알려진 자세를 AMCL(initialpose) 과 map EKF(set_pose) 에 함께 준다.

    kidnap_monitor 가 복구할 때 하는 것과 같은 묶음(initialpose + map EKF set_pose + AMCL 무이동 갱신)이다.
    initialpose 만 주면 정지한 로봇에서 odometry/filtered_map 이 옛 자리에 남을 수 있다 (12 스모크 실측:
    AMCL 은 (0.8, 5) 로 설정됐는데 filtered_map 오차가 240 s 동안 0.1 m 를 넘었고, scan_matcher 는 보정 과다로
    거절했다). 판정 대상이 아닌 준비 단계에서만 쓴다.
    ns: 로봇 이름공간 ('' = 프로브 기준 상대 이름). 반환 {'initialpose': 발행 수, 'ekf_set_pose': 성공,
    'amcl_nomotion_updates': 성공한 무이동 갱신 요청 수}.
    """
    prefix = f'/{ns.strip("/")}/' if ns else ''
    for _ in range(repeats):
        msg = initial_pose(x, y, yaw)
        msg.header.stamp = probe.node.get_clock().now().to_msg()
        probe.publish(f'{prefix}initialpose', msg)
        time.sleep(0.2)
    ok = False
    try:
        from robot_localization.srv import SetPose
    except ImportError:          # robot_localization 없는 환경
        SetPose = None
    if SetPose is not None:
        req = SetPose.Request()
        req.pose = initial_pose(x, y, yaw)
        req.pose.header.stamp = probe.node.get_clock().now().to_msg()
        ok = probe.call(SetPose, f'{prefix}{EKF_SET_POSE}', req, timeout) is not None
    # 정지한 로봇의 AMCL 은 움직임 문턱 전에는 갱신·발행하지 않는다 → 무이동 갱신 요청 (kidnap_monitor 와 같다)
    from std_srvs.srv import Empty
    nomotion = sum(1 for _ in range(2)
                   if probe.call(Empty, f'{prefix}{AMCL_NOMOTION}', Empty.Request(),
                                 min(timeout, 2.0)) is not None)
    return {'initialpose': repeats, 'ekf_set_pose': ok, 'amcl_nomotion_updates': nomotion}


def is_zero(msg: Twist, tol: float = 1e-3) -> bool:
    """정지 명령인가 (선속도·각속도 모두 tol 이하)."""
    return abs(msg.linear.x) <= tol and abs(msg.linear.y) <= tol and abs(msg.angular.z) <= tol


def odom_speed(msg) -> Tuple[float, float]:
    """nav_msgs/Odometry → (평면 속도 크기, yaw 속도)."""
    lin = msg.twist.twist.linear
    return math.hypot(lin.x, lin.y), msg.twist.twist.angular.z


class Commander:
    """
    주행 명령을 rate [Hz] 로 계속 발행하는 스레드 (Nav2 컨트롤러 자리).

    set(v, w) 로 명령을 바꾸고 stop() 으로 0 을 한 번 보낸 뒤 멈춘다.
    """

    def __init__(self, probe: GraphProbe, topic: str, rate: float = 20.0):
        self.probe = probe
        self.topic = topic
        self.period = 1.0 / rate
        self._cmd = twist()
        self._lock = threading.Lock()
        self._run = False
        self._thread: Optional[threading.Thread] = None

    def set(self, v: float, w: float = 0.0) -> float:
        """명령을 바꾸고 즉시 한 번 발행한다 (발행 wall 시각 반환)."""
        with self._lock:
            self._cmd = twist(v, w)
            cmd = self._cmd
        return self.probe.publish(self.topic, cmd)

    def start(self) -> 'Commander':
        if not self._run:
            self._run = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
        return self

    def _loop(self) -> None:
        while self._run:
            with self._lock:
                cmd = self._cmd
            self.probe.publish(self.topic, cmd)
            time.sleep(self.period)

    def stop(self) -> None:
        self._run = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.probe.publish(self.topic, twist())


def drive_for(probe: GraphProbe, topic: str, v: float, w: float, duration: float,
              wall_cap: float, rate: float = 20.0) -> bool:
    """(v, w) 를 ROS 시각 duration [s] 동안 rate 로 발행한다 (wall_cap 초과 시 False)."""
    t0 = probe.now()
    deadline = time.monotonic() + wall_cap
    msg = twist(v, w)
    while probe.now() - t0 < duration:
        if time.monotonic() > deadline:
            return False
        probe.publish(topic, msg)
        time.sleep(1.0 / rate)
    return True


def wait_rest(probe: GraphProbe, gt: TopicRecord, hold: float, timeout: float,
              v_tol: float = 0.01, w_tol: float = 0.01) -> bool:
    """GT 속도가 hold [s](ROS 시각) 동안 v_tol/w_tol 이하로 유지될 때까지 기다린다."""
    state = {'since': None}

    def rested() -> bool:
        msg = gt.last()
        if msg is None:
            return False
        v, w = odom_speed(msg)
        now = probe.now()
        if v > v_tol or abs(w) > w_tol:
            state['since'] = None
            return False
        if state['since'] is None:
            state['since'] = now
        return now - state['since'] >= hold

    return probe.wait_until(rested, timeout, period=0.02)


def first_after(rec: TopicRecord, since: float, predicate) -> Optional[Tuple[float, object]]:
    """since(wall) 이후 수신한 메시지 중 predicate 를 만족하는 첫 (수신 시각, 메시지)."""
    for t, msg in rec.messages(since):
        if predicate(msg):
            return t, msg
    return None


def wait_first(probe: GraphProbe, rec: TopicRecord, since: float, predicate,
               timeout: float) -> Optional[Tuple[float, object]]:
    """first_after 가 나올 때까지 기다린다 (없으면 None)."""
    box = {}

    def found() -> bool:
        hit = first_after(rec, since, predicate)
        if hit is not None:
            box['hit'] = hit
            return True
        return False

    probe.wait_until(found, timeout, period=0.002)
    return box.get('hit')


def obstacles_json(world_circles: Iterable = (), world_boxes: Iterable = (),
                   robot_circles: Iterable = (), robot_boxes: Iterable = ()) -> String:
    """kinematic_sim 의 itest/obstacles 메시지."""
    return String(data=json.dumps({
        'world': {'circles': [list(c) for c in world_circles],
                  'boxes': [list(b) for b in world_boxes]},
        'robot': {'circles': [list(c) for c in robot_circles],
                  'boxes': [list(b) for b in robot_boxes]},
    }))


def follow_waypoints(probe: GraphProbe, gt: TopicRecord, topic: str,
                     waypoints: Sequence[Tuple[float, float]], v: float = 0.5,
                     tol: float = 0.3, timeout_per_wp: float = 120.0, k_heading: float = 1.5,
                     w_max: float = 1.0, rate: float = 20.0,
                     wall_factor: float = 10.0) -> List[bool]:
    """
    GT 자세를 되먹임해 웨이포인트를 차례로 지난다 (Nav2 없이 지도 작성·장시간 주행용).

    헤딩 오차 e 에 대해 ω = clip(k·e, ±w_max), v_cmd = v·max(0, cos e) (제자리 회전 후 전진).
    timeout_per_wp 는 ROS 시각(Gazebo 면 sim time) — 부하로 RTF 가 떨어져도 구간이 잘리지 않게 (03 실측:
    loadavg 85, RTF ≈ 0.3 에서 wall 상한으로 19 m 구간이 끝나기 전에 넘어갔다). sim time 이 멈추면
    wall 상한 timeout_per_wp × wall_factor 에서 포기한다.
    반환: 웨이포인트별 도달 여부 (시간 초과면 False, 다음 점으로 넘어간다).
    """
    reached = []
    for wx, wy in waypoints:
        wall_deadline = time.monotonic() + timeout_per_wp * wall_factor
        t0 = probe.now()
        ok = False
        while probe.now() - t0 < timeout_per_wp and time.monotonic() < wall_deadline:
            msg = gt.last()
            if msg is None:
                time.sleep(0.05)
                continue
            s = metrics.sample_from_odom(msg)
            dx, dy = wx - s.x, wy - s.y
            if math.hypot(dx, dy) <= tol:
                ok = True
                break
            err = metrics.wrap(math.atan2(dy, dx) - s.yaw)
            w = max(-w_max, min(w_max, k_heading * err))
            probe.publish(topic, twist(v * max(0.0, math.cos(err)), w))
            time.sleep(1.0 / rate)
        reached.append(ok)
    probe.publish(topic, twist())
    return reached


class ActionCaller:
    """
    프로브 노드 위의 액션 클라이언트 (Nav2 compute_path_to_pose, navigate_to_pose, dock 등).

    프로브 executor 스레드가 응답을 처리하므로 테스트 스레드는 이벤트만 기다린다.
    """

    def __init__(self, probe: GraphProbe, action_type, name: str):
        from rclpy.action import ActionClient
        self.probe = probe
        self.client = ActionClient(probe.node, action_type, name)
        self.feedback: List = []

    def wait_server(self, timeout: float) -> bool:
        return self.client.wait_for_server(timeout_sec=timeout)

    def call(self, goal, timeout: float) -> Tuple[Optional[int], Optional[object]]:
        """
        목표 goal 을 보내고 결과까지 기다린다. 반환 (GoalStatus 코드, result) — 거절·시간 초과면 None.

        시간 초과 시 목표를 취소한다 (다음 시행에 영향이 없게).
        """
        self.feedback = []
        accepted = threading.Event()
        box = {}

        def on_goal(fut) -> None:
            box['handle'] = fut.result()
            accepted.set()

        deadline = time.monotonic() + timeout
        send = self.client.send_goal_async(goal, feedback_callback=self.feedback.append)
        send.add_done_callback(on_goal)
        if not accepted.wait(timeout) or not box['handle'].accepted:
            return None, None
        done = threading.Event()
        res_fut = box['handle'].get_result_async()
        res_fut.add_done_callback(lambda _f: done.set())
        if not done.wait(max(0.0, deadline - time.monotonic())):
            box['handle'].cancel_goal_async()
            return None, None
        res = res_fut.result()
        return res.status, res.result
