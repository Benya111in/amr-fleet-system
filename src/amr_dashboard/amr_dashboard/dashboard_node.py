"""
dashboard_node: Flask + SSE 웹 모니터링 대시보드 (components.md §5.7, 명세 4.9).

구독  /fleet/status (amr_msgs/FleetStatus) · /fleet/alerts (diagnostic_msgs/DiagnosticArray)
      /fleet/task_events (amr_msgs/Task) · /map (nav_msgs/OccupancyGrid, transient_local)
발행  /fleet/task_request (std_msgs/String, JSON Task Description) — 구독자가 없으면 보내지 않고 503
      /fleet/estop, /<robot_id>/estop (std_msgs/Bool, transient_local "latched")
서비스 클라이언트  /<robot_id>/safety/reset_estop (std_srvs/Trigger) — E-stop 해제 때
HTTP  :8080 — 경로 목록·보안 규칙은 web_app / security 모듈 docstring 참고.

E-stop 해제 순서: false 발행 → 구독자 확인 응답(ack)을 reset_ack_timeout 까지 기다림 → reset_estop 호출.
reset_estop 응답(성공/거부/서버 없음/무응답)은 EstopOutcome 에 모여 HTTP 응답과 표시 상태가 된다 —
리셋이 확인되지 않은 로봇은 계속 정지로 표시한다 (require_reset_ack, estop 모듈).

스레드 모델: rclpy 실행기는 백그라운드 스레드에서 돌고, Flask 는 메인 스레드의 werkzeug 스레드
서버가 서비스한다. 둘은 StateStore(스레드 안전)로만 만난다. eventlet/flask-socketio 는 rclpy 를
막으므로 쓰지 않는다. 실행기 스레드는 spin_guarded 로 돈다 — 콜백 예외 하나에 스핀이 끝나 HTTP 만 살아
있는 "멈춘 대시보드" 가 되지 않게 예외를 기록하고 계속 돈다.
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
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, \
    ShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from werkzeug.serving import make_server

from amr_dashboard import estop as es
from amr_dashboard import security
from amr_dashboard.action_log import ActionLog, default_log_dir
from amr_dashboard.state_store import DEFAULT_WORLD, StateStore, valid_robot_id
from amr_dashboard.web_app import create_app

DEFAULT_ROBOT_IDS = ['amr_01', 'amr_02', 'amr_03', 'amr_04', 'amr_05']


def parse_robot_ids(value) -> list:
    """robot_ids: 문자열 배열 또는 쉼표 구분 문자열("amr_01,amr_02") — fleet_manager 와 같은 두 형식."""
    if isinstance(value, str):
        items = value.split(',')
    else:
        items = list(value or [])
    return [str(v).strip() for v in items if str(v).strip()]


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
        self.declare_parameter('robot_ids', DEFAULT_ROBOT_IDS,
                               ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('allowed_hosts', [''])       # Host 허용 추가분 ('*' 는 검사 끔)
        self.declare_parameter('api_token', '')             # '' → $AMR_DASHBOARD_TOKEN → 없음
        self.declare_parameter('require_reset_ack', True)   # 리셋 서버가 없으면 해제 실패로
        self.declare_parameter('reset_timeout', 2.0)        # [s] reset_estop 응답 대기
        self.declare_parameter('reset_ack_timeout', 0.2)    # [s] false 발행 ack 대기 (리셋 전)
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

        ids = parse_robot_ids(self.get_parameter('robot_ids').value)
        self.robot_ids = [r for r in ids if valid_robot_id(r)]
        for bad in sorted(set(ids) - set(self.robot_ids)):
            self.get_logger().error(f'robot_ids 의 "{bad}" 는 토픽 이름으로 쓸 수 없어 뺀다')
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

    @property
    def allowed_hosts(self) -> list:
        """HTTP Host 허용 목록: 루프백 + host 파라미터(특정 주소면) + allowed_hosts 파라미터."""
        extra = [h for h in self.get_parameter('allowed_hosts').value if h]
        return security.allowed_hosts(self.get_parameter('host').value, extra)

    @property
    def api_token(self) -> str:
        return security.resolve_token(self.get_parameter('api_token').value, os.environ)

    def publish_task_request(self, json_str: str, task: dict) -> int:
        """
        /fleet/task_request 에 JSON Task Description 을 발행하고 구독자 수를 돌려준다.

        구독자가 0 이면(fleet_manager 미기동·미발견) 발행하지 않고 0 — 휘발성 토픽이라 그대로 유실되기
        때문이다. web_app 이 503 으로 알린다.
        """
        count = self._task_pub.get_subscription_count()
        if count == 0:
            self.get_logger().warn(
                f'작업 {task["task_id"]} 거부: {self._topic("task_request")} 구독자 없음 (fleet_manager?)')
            return 0
        self._task_pub.publish(String(data=json_str))
        self.get_logger().info(
            f'작업 투입 {task["task_id"]} (priority={task["priority"]}, item={task["item_type"]}, '
            f'구독자 {count})')
        return count

    def publish_estop(self, target: str, active: bool) -> es.EstopOutcome:
        """
        E-stop 발행. target 'all' → /fleet/estop, 아니면 /<robot_id>/estop (둘 다 latched).

        전체 해제는 로봇별 토픽에도 false 를 발행한다: 래치된 /<robot_id>/estop=true 가 남아
        있으면 safety_node 가 계속 정지하기 때문이다 (StateStore.set_estop 의 표시 규칙과 같다).
        해제(active=False)는 false 발행 → ack 대기(reset_ack_timeout) → safety/reset_estop(Trigger)
        비동기 호출 순서다: safety_node 의 대시보드 E-stop 래치는 서비스로만 풀리고(sequences.md),
        false 보다 리셋이 먼저 닿으면 거절될 수 있기 때문이다. 로봇 하나의 실패(토픽 이름 불가 등)는
        그 로봇만 publish_failed 로 남기고 나머지는 계속한다. 리셋 응답은 결과 객체에 비동기로 채워진다.
        """
        outcome = es.EstopOutcome(target, active, self.get_parameter('require_reset_ack').value)
        msg = Bool(data=bool(active))
        pubs = []
        if target == 'all':
            self._fleet_estop_pub.publish(msg)
            outcome.add_published(self._topic('fleet_estop'))
            pubs.append(self._fleet_estop_pub)
            targets = self.store.known_robot_ids() if not active else []
        else:
            targets = [target]
        released = []
        for rid in targets:
            try:
                pub = self._estop_publisher(rid)
                pub.publish(msg)
            except Exception as exc:  # noqa: B902 — 잘못된 id 하나로 나머지 로봇을 멈추지 않는다
                outcome.fail_publish(rid, f'{type(exc).__name__}: {exc}')
                self.get_logger().error(f'{rid}: E-stop 발행 실패: {exc}')
                continue
            outcome.add_published(self.robot_topic(rid, self._topic('robot_estop')))
            pubs.append(pub)
            released.append(rid)
        self.get_logger().warn(f'E-stop {"활성" if active else "해제"} → {target}')
        if not active:
            self._wait_acked(pubs)
            for rid in released:
                self._call_reset_estop(rid, outcome)
        return outcome

    def _wait_acked(self, pubs) -> None:
        """E-stop 값을 reliable 구독자가 받았다는 확인(ack)을 짧게 기다린다 (리셋보다 값이 먼저 닿게)."""
        timeout = float(self.get_parameter('reset_ack_timeout').value)
        if timeout <= 0.0:
            return
        deadline = Duration(seconds=timeout)
        for pub in pubs:
            try:
                pub.wait_for_all_acked(deadline)
            except Exception as exc:  # noqa: B902 — ack 대기 실패는 순서 보장만 약해진다
                self.get_logger().debug(f'wait_for_all_acked 실패: {exc}')

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

    def _call_reset_estop(self, robot_id: str, outcome: es.EstopOutcome = None) -> str:
        """safety/reset_estop 를 비동기로 호출하고 결과를 outcome 에 채운다. 돌려주는 값은 즉시 상태."""
        outcome = outcome if outcome is not None else es.EstopOutcome(robot_id, False)
        try:
            client = self._reset_client(robot_id)
        except Exception as exc:  # noqa: B902 — 서비스 이름 불가 등
            outcome.set_reset(robot_id, es.ERROR, str(exc))
            return es.ERROR
        if client is None:
            outcome.set_reset(robot_id, es.NOT_CONFIGURED)
            return es.NOT_CONFIGURED
        if not client.service_is_ready():
            self.get_logger().warn(
                f'{robot_id}: reset_estop 서비스 없음 — false 만 발행했고 safety_node 래치는 그대로다')
            outcome.set_reset(robot_id, es.NO_SERVER, 'service not available')
            return es.NO_SERVER
        outcome.set_reset(robot_id, es.PENDING)
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f, rid=robot_id: self._on_reset_done(rid, f, outcome))
        return es.PENDING

    def _on_reset_done(self, robot_id: str, future, outcome: es.EstopOutcome = None) -> None:
        try:
            status, message = es.reset_status_from_result(future.result())
        except Exception as exc:  # noqa: B902 — 서비스 실패는 결과로 남긴다
            status, message = es.reset_status_from_result(None, exc)
            self.get_logger().error(f'{robot_id}: reset_estop 실패: {exc}')
        else:
            # rclpy 로거는 호출 위치마다 심각도가 고정이다 — 한 줄에서 info/warn 을 번갈아 부르면 ValueError 로
            # 실행기 스레드가 죽는다 (기능 실행에서 재현). 심각도마다 호출 위치를 나눈다.
            if status == es.OK:
                self.get_logger().info(f'{robot_id}: reset_estop → {status} {message}')
            else:
                self.get_logger().warn(f'{robot_id}: reset_estop → {status} {message}')
        if outcome is not None:
            outcome.set_reset(robot_id, status, message)


def spin_guarded(executor, logger, running) -> int:
    """
    실행기를 running() 이 참인 동안 spin_once 로 돌린다. 콜백 예외는 기록하고 계속 돈다 → 잡은 예외 수.

    executor.spin() 은 콜백 예외 하나에 스핀 스레드째 끝나고, 그 뒤로는 HTTP 만 살아 상태가 멈춘
    대시보드가 된다 (구독·reset_estop 응답이 끊겨 해제가 모두 timeout). 그래서 예외를 격리한다.
    """
    errors = 0
    while running():
        try:
            executor.spin_once(timeout_sec=0.2)
        except (ExternalShutdownException, ShutdownException):
            break
        except Exception as exc:  # noqa: B902 — 콜백 하나의 오류로 대시보드 전체를 멈추지 않는다
            errors += 1
            logger.error(f'콜백 예외 (실행기는 계속 돈다): {type(exc).__name__}: {exc}')
    return errors


def main(args=None) -> int:
    """실행기(rclpy, 백그라운드 스레드) + werkzeug 서버(메인 스레드). SIGINT/SIGTERM 으로 종료."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DashboardNode()
    action_log = ActionLog(node.log_dir, on_error=lambda m: node.get_logger().warn(m))
    app = create_app(
        node.store, web_dir=node.web_dir,
        on_task_request=node.publish_task_request, on_estop=node.publish_estop,
        heartbeat_period=node.get_parameter('heartbeat_period').value,
        action_log=action_log, allowed_hosts=node.allowed_hosts, api_token=node.api_token,
        reset_timeout=float(node.get_parameter('reset_timeout').value))
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
    stop = threading.Event()
    spin_thread = threading.Thread(
        target=spin_guarded, args=(executor, node.get_logger(),
                                   lambda: rclpy.ok() and not stop.is_set()),
        name='rclpy-spin', daemon=True)
    spin_thread.start()

    def request_shutdown(signum, frame):
        node.get_logger().info(f'시그널 {signum} 수신 — 서버를 내린다')
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    node.get_logger().info(
        f'대시보드 http://{host}:{port}/ (web={node.web_dir}, logs={node.log_dir}, '
        f'Host 허용 {node.allowed_hosts}, 조작 토큰 {"있음" if node.api_token else "없음"})')
    try:
        server.serve_forever()
    finally:
        stop.set()
        spin_thread.join(timeout=2.0)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
