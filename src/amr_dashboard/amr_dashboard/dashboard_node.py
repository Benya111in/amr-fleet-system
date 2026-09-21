"""
dashboard_node: Flask + SSE 웹 모니터링 대시보드 (components.md §5.7, 명세 4.9).

구독  /fleet/status (amr_msgs/FleetStatus) · /fleet/alerts (diagnostic_msgs/DiagnosticArray)
      /fleet/task_events (amr_msgs/Task) · /map (nav_msgs/OccupancyGrid, transient_local)
발행  /fleet/task_request (std_msgs/String, JSON Task Description)
      /fleet/estop, /<robot_id>/estop (std_msgs/Bool, transient_local "latched")
HTTP  :8080 — 경로 목록은 web_app 모듈 docstring 참고.

스레드 모델: rclpy 실행기는 백그라운드 스레드에서 돌고, Flask 는 메인 스레드의 werkzeug 스레드
서버가 서비스한다. 둘은 StateStore(스레드 안전)로만 만난다. eventlet/flask-socketio 는 rclpy 를
막으므로 쓰지 않는다.
실행기는 기본 SingleThreadedExecutor (executor_threads: 1): 콜백이 모두 짧아(변환 + 큐 넣기) 병렬이
필요 없고, 측정에서 rclpy MultiThreadedExecutor 는 발행 → SSE 지연을 4~5배(평균 ~21 → ~100 ms),
CPU 를 8배 늘렸다. executor_threads >= 2 로 MultiThreadedExecutor 를 고를 수 있다.
"""

import logging
import os
import signal
import threading

from ament_index_python.packages import get_package_share_directory
from amr_msgs.msg import FleetStatus, Task
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from werkzeug.serving import make_server

from amr_dashboard.action_log import ActionLog, default_log_dir
from amr_dashboard.state_store import DEFAULT_WORLD, StateStore
from amr_dashboard.web_app import create_app

DEFAULT_ROBOT_IDS = ['amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05']


def qos_reliable(depth=10, transient_local=False) -> QoSProfile:
    """발행/구독 QoS: reliable volatile depth N. transient_local=True 면 latched."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if transient_local
                    else DurabilityPolicy.VOLATILE),
    )


class DashboardNode(Node):
    """ROS 쪽 얇은 래퍼: 구독 → StateStore, HTTP 조작 → 발행/서비스."""

    def __init__(self, store: StateStore = None, **node_kwargs):
        super().__init__('dashboard_node', **node_kwargs)
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8080)
        self.declare_parameter('web_dir', '')
        self.declare_parameter('log_dir', '')
        self.declare_parameter('heartbeat_period', 5.0)
        self.declare_parameter('executor_threads', 1)
        self.declare_parameter('robot_ids', DEFAULT_ROBOT_IDS)
        self.declare_parameter('task_events_transient_local', False)
        self.declare_parameter('max_alerts', 100)
        self.declare_parameter('max_task_events', 50)
        self.declare_parameter('topics.fleet_status', '/fleet/status')
        self.declare_parameter('topics.fleet_alerts', '/fleet/alerts')
        self.declare_parameter('topics.task_events', '/fleet/task_events')
        self.declare_parameter('topics.map', '/map')
        self.declare_parameter('topics.task_request', '/fleet/task_request')
        self.declare_parameter('topics.fleet_estop', '/fleet/estop')
        self.declare_parameter('topics.robot_estop', 'estop')
        self.declare_parameter('topics.reset_estop_service', 'safety/reset_estop')
        for key, value in DEFAULT_WORLD.items():
            self.declare_parameter(f'world.{key}', value)

        self.robot_ids = list(self.get_parameter('robot_ids').value)
        world = {key: self.get_parameter(f'world.{key}').value for key in DEFAULT_WORLD}
        self.store = store if store is not None else StateStore(
            max_alerts=self.get_parameter('max_alerts').value,
            max_task_events=self.get_parameter('max_task_events').value,
            world=world, robot_ids=self.robot_ids)

        topic = self._topic
        self.create_subscription(FleetStatus, topic('fleet_status'), self._on_status,
                                 qos_reliable(10))
        self.create_subscription(DiagnosticArray, topic('fleet_alerts'), self._on_alerts,
                                 qos_reliable(10))
        self.create_subscription(
            Task, topic('task_events'), self._on_task_event,
            qos_reliable(50, self.get_parameter('task_events_transient_local').value))
        self.create_subscription(OccupancyGrid, topic('map'), self._on_map,
                                 qos_reliable(1, transient_local=True))

        self._task_pub = self.create_publisher(String, topic('task_request'), qos_reliable(10))
        self._fleet_estop_pub = self.create_publisher(Bool, topic('fleet_estop'),
                                                      qos_reliable(1, transient_local=True))
        self._entity_lock = threading.Lock()
        self._estop_pubs = {}
        self._reset_clients = {}
        for rid in self.robot_ids:
            self._estop_publisher(rid)
            self._reset_client(rid)

        self.get_logger().info(
            f'구독 {topic("fleet_status")}, {topic("fleet_alerts")}, {topic("task_events")}, '
            f'{topic("map")} / 발행 {topic("task_request")}, {topic("fleet_estop")}, '
            f'/<robot_id>/{topic("robot_estop")} (robot_ids={self.robot_ids})')

    # ----- 파라미터 도우미 -----

    def _topic(self, key: str) -> str:
        return self.get_parameter(f'topics.{key}').value

    def robot_topic(self, robot_id: str, relative: str) -> str:
        """로봇 네임스페이스 아래 상대 이름 → 전역 이름 (/amr_01/estop)."""
        return f'/{robot_id}/{relative.lstrip("/")}'

    @property
    def web_dir(self) -> str:
        """web_dir 파라미터, 비어 있으면 share/amr_dashboard/web."""
        value = self.get_parameter('web_dir').value
        if value:
            return value
        return os.path.join(get_package_share_directory('amr_dashboard'), 'web')

    @property
    def log_dir(self) -> str:
        """log_dir 파라미터, 비어 있으면 $ROS_WS/logs."""
        return self.get_parameter('log_dir').value or default_log_dir()

    # ----- 구독 콜백 (실행기 스레드) -----

    def _on_status(self, msg: FleetStatus) -> None:
        self.store.update_status(msg)

    def _on_alerts(self, msg: DiagnosticArray) -> None:
        alerts = self.store.add_alerts(msg)
        for alert in alerts:
            self.get_logger().warn(
                f'알림 [{alert["level_name"]}] {alert["hardware_id"]} {alert["name"]}: '
                f'{alert["message"]}')

    def _on_task_event(self, msg: Task) -> None:
        event = self.store.add_task_event(msg)
        self.get_logger().info(
            f'작업 이벤트 {event["task_id"]} → {event["status_name"]} (robot={event["robot_id"]})')

    def _on_map(self, msg: OccupancyGrid) -> None:
        meta = self.store.update_map(msg)
        self.get_logger().info(
            f'지도 수신 {meta["width"]}x{meta["height"]} @ {meta["resolution"]} m')

    # ----- HTTP 조작 → ROS (Flask 스레드) -----

    def publish_task_request(self, json_str: str, task: dict) -> None:
        """/fleet/task_request 에 JSON Task Description 을 발행한다."""
        self._task_pub.publish(String(data=json_str))
        self.get_logger().info(
            f'작업 투입 {task["task_id"]} (priority={task["priority"]}, item={task["item_type"]})')

    def publish_estop(self, target: str, active: bool) -> None:
        """
        E-stop 발행. target 'all' → /fleet/estop, 아니면 /<robot_id>/estop (둘 다 latched).

        전체 해제는 로봇별 토픽에도 false 를 발행한다: 래치된 /<robot_id>/estop=true 가 남아
        있으면 safety_node 가 계속 정지하기 때문이다 (StateStore.set_estop 의 표시 규칙과 같다).
        해제(active=False)는 값 발행에 더해 safety/reset_estop(Trigger) 를 호출한다:
        safety_node 의 대시보드 E-stop 래치는 서비스로만 풀리기 때문이다 (sequences.md).
        """
        msg = Bool(data=bool(active))
        if target == 'all':
            self._fleet_estop_pub.publish(msg)
            targets = self.store.known_robot_ids()
            if not active:
                for rid in targets:
                    self._estop_publisher(rid).publish(msg)
        else:
            self._estop_publisher(target).publish(msg)
            targets = [target]
        self.get_logger().warn(f'E-stop {"활성" if active else "해제"} → {target}')
        if not active:
            for rid in targets:
                self._call_reset_estop(rid)

    def _estop_publisher(self, robot_id: str):
        with self._entity_lock:
            pub = self._estop_pubs.get(robot_id)
            if pub is None:
                pub = self.create_publisher(
                    Bool, self.robot_topic(robot_id, self._topic('robot_estop')),
                    qos_reliable(1, transient_local=True))
                self._estop_pubs[robot_id] = pub
            return pub

    def _reset_client(self, robot_id: str):
        service = self._topic('reset_estop_service')
        if not service:
            return None
        with self._entity_lock:
            client = self._reset_clients.get(robot_id)
            if client is None:
                client = self.create_client(Trigger, self.robot_topic(robot_id, service))
                self._reset_clients[robot_id] = client
            return client

    def _call_reset_estop(self, robot_id: str) -> bool:
        """safety/reset_estop 를 비동기로 호출한다 (서버가 없으면 건너뛴다)."""
        client = self._reset_client(robot_id)
        if client is None or not client.service_is_ready():
            self.get_logger().info(f'{robot_id}: reset_estop 서비스 없음 — 값 발행만 한다')
            return False
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f, rid=robot_id: self._on_reset_done(rid, f))
        return True

    def _on_reset_done(self, robot_id: str, future) -> None:
        try:
            result = future.result()
        except Exception as exc:  # 서비스 실패는 로그만 남긴다
            self.get_logger().error(f'{robot_id}: reset_estop 실패: {exc}')
            return
        level = self.get_logger().info if result.success else self.get_logger().warn
        level(f'{robot_id}: reset_estop → success={result.success} {result.message}')


def main(args=None) -> int:
    """실행기(rclpy, 백그라운드 스레드) + werkzeug 서버(메인 스레드). SIGINT/SIGTERM 으로 종료."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DashboardNode()
    action_log = ActionLog(node.log_dir, on_error=lambda m: node.get_logger().warn(m))
    app = create_app(
        node.store, web_dir=node.web_dir,
        on_task_request=node.publish_task_request, on_estop=node.publish_estop,
        heartbeat_period=node.get_parameter('heartbeat_period').value,
        action_log=action_log)
    logging.getLogger('werkzeug').setLevel(logging.WARNING)

    # 포트 바인드를 실행기 스레드보다 먼저: 실패하면(werkzeug 는 sys.exit(1)) 스핀 스레드가 살아 있는
    # 채로 인터프리터가 내려가 abort 되지 않고, 오류를 남기고 1 로 끝난다.
    host = node.get_parameter('host').value
    port = int(node.get_parameter('port').value)
    try:
        server = make_server(host, port, app, threaded=True)
    except (OSError, SystemExit) as exc:
        # werkzeug 는 원인(strerror)을 stderr 에 찍고 sys.exit(1) 한다
        detail = exc.strerror if isinstance(exc, OSError) else '원인은 바로 위 werkzeug 출력'
        node.get_logger().fatal(
            f'HTTP {host}:{port} 바인드 실패 ({detail}) — 포트를 쓰는 다른 프로세스나 host 값 확인')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # executor_threads 1 = SingleThreadedExecutor (근거는 모듈 docstring), 2 이상 = MultiThreaded
    threads = int(node.get_parameter('executor_threads').value)
    executor = (SingleThreadedExecutor() if threads <= 1
                else MultiThreadedExecutor(num_threads=threads))
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name='rclpy-spin', daemon=True)
    spin_thread.start()

    def request_shutdown(signum, frame):
        node.get_logger().info(f'시그널 {signum} 수신 — 서버를 내린다')
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    node.get_logger().info(
        f'대시보드 http://{host}:{port}/ (web={node.web_dir}, logs={node.log_dir})')
    try:
        server.serve_forever()
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
