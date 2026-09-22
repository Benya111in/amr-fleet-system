"""
ROS 그래프 프로브: 시나리오 테스트가 토픽을 구독·발행하고, 주기를 재고, TF 와 서비스를 쓰는 창구.

launch_testing 은 pre-shutdown 테스트를 백그라운드 스레드에서 돌리고 launch 는 메인 스레드에서
돈다. 프로브는 자기 rclpy Context 와 executor 스레드를 따로 가지므로 launch_ros 의 컨텍스트와
섞이지 않고, 콜백은 수신 즉시 (수신 시각, 스탬프, 메시지) 로 기록된다. 테스트 코드는 기록을
읽고 wait_until() 로 조건을 기다리기만 한다.

시각
  wall()  time.time() — 같은 호스트의 모든 노드와 비교 가능한 시스템 시각 (use_sim_time=false 의
          ROS 시각과 같다). 지연(latency) 측정은 이 시각으로 한다.
  now()   프로브 노드의 ROS 시각 (use_sim_time=true 면 /clock 기준 sim time).
"""

from collections import deque
import threading
import time
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

from amr_itest import cdr
from amr_itest.tf_tree import TfGraph
import rclpy
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import (ExternalShutdownException, ShutdownException,
                             SingleThreadedExecutor)
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

QOS = {
    'reliable': QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                           reliability=ReliabilityPolicy.RELIABLE),
    'sensor': QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT),
    'latched': QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL),
    'static_tf': QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL),
}


def qos(name_or_profile) -> QoSProfile:
    if isinstance(name_or_profile, QoSProfile):
        return name_or_profile
    return QOS[name_or_profile]


def stamp_seconds(msg: Any) -> Optional[float]:
    """메시지의 header.stamp [s] (헤더가 없으면 None, Clock 이면 clock)."""
    header = getattr(msg, 'header', None)
    if header is not None:
        return header.stamp.sec + header.stamp.nanosec * 1e-9
    clock = getattr(msg, 'clock', None)
    if clock is not None and hasattr(clock, 'sec'):
        return clock.sec + clock.nanosec * 1e-9
    return None


class TopicRecord:
    """
    한 토픽의 수신 기록 (스레드 안전).

    entries: (수신 wall 시각, header 스탬프 또는 None, 메시지 또는 None(raw)) 최근 keep 개.
    """

    def __init__(self, topic: str, keep: int = 20000, keep_messages: int = 200):
        self.topic = topic
        self._lock = threading.Lock()
        self._times: Deque[Tuple[float, Optional[float]]] = deque(maxlen=keep)
        self._msgs: Deque[Tuple[float, Any]] = deque(maxlen=keep_messages)
        self.count = 0
        self.frame_id: Optional[str] = None

    def add(self, recv: float, stamp: Optional[float], msg: Any = None,
            frame_id: Optional[str] = None) -> None:
        with self._lock:
            self.count += 1
            self._times.append((recv, stamp))
            if msg is not None:
                self._msgs.append((recv, msg))
            if frame_id is not None:
                self.frame_id = frame_id

    def last(self) -> Optional[Any]:
        with self._lock:
            return self._msgs[-1][1] if self._msgs else None

    def last_recv(self) -> Optional[float]:
        with self._lock:
            return self._times[-1][0] if self._times else None

    def messages(self, since: float = -1.0) -> List[Tuple[float, Any]]:
        """(수신 시각, 메시지) 목록 (since 이후 수신분)."""
        with self._lock:
            return [(t, m) for t, m in self._msgs if t >= since]

    def stamps(self, since: float = -1.0) -> List[float]:
        """메시지 header 스탬프 목록 (since 이후 수신분, 스탬프 없는 메시지 제외)."""
        with self._lock:
            return [s for t, s in self._times if t >= since and s is not None]

    def recv_times(self, since: float = -1.0) -> List[float]:
        with self._lock:
            return [t for t, _ in self._times if t >= since]

    def count_since(self, since: float) -> int:
        with self._lock:
            return sum(1 for t, _ in self._times if t >= since)

    def clear(self) -> None:
        with self._lock:
            self._times.clear()
            self._msgs.clear()


class GraphProbe:
    """시나리오 테스트용 rclpy 노드 + 전용 executor 스레드."""

    def __init__(self, name: str = 'itest_probe', use_sim_time: bool = False,
                 namespace: str = ''):
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            name, namespace=namespace or None, context=self.context,
            parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)])
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self._records: Dict[str, TopicRecord] = {}
        self._pubs: Dict[Tuple[str, type], Any] = {}
        self._clients: Dict[Tuple[str, type], Any] = {}
        self._subs: List[Any] = []
        self._tf_buffer = None
        self._tf_listener = None
        self.tf_graph: Optional[TfGraph] = None
        self._tf_lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._spin, name=f'{name}_spin', daemon=True)
        self._thread.start()

    # --- 수명 ---
    def _spin(self) -> None:
        while self._running and self.context.ok():
            try:
                self.executor.spin_once(timeout_sec=0.05)
            except (ExternalShutdownException, ShutdownException):
                break
            except Exception as exc:     # noqa: B902 — 콜백 예외로 스레드가 죽지 않게 기록만
                self.node.get_logger().error(f'probe callback error: {exc!r}')

    def close(self) -> None:
        """프로브 executor 스레드를 멈추고 노드·컨텍스트를 정리한다."""
        self._running = False
        self._thread.join(timeout=5.0)
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        finally:
            if self.context.ok():
                rclpy.shutdown(context=self.context)

    # --- 시각 ---
    @staticmethod
    def wall() -> float:
        return time.time()

    def now(self) -> float:
        """ROS 시각 [s] (use_sim_time 이면 /clock)."""
        return self.node.get_clock().now().nanoseconds * 1e-9

    def sleep_ros(self, seconds: float, wall_cap: float) -> bool:
        """ROS 시각으로 seconds 만큼 기다린다 (sim time 이 멈추면 wall_cap 에서 포기, False)."""
        t0 = self.now()
        return self.wait_until(lambda: self.now() - t0 >= seconds, wall_cap, period=0.01)

    @staticmethod
    def wait_until(predicate: Callable[[], bool], timeout: float, period: float = 0.02) -> bool:
        """조건 predicate 가 참이 될 때까지 (최대 timeout [s]) 기다린다."""
        deadline = time.monotonic() + timeout
        while True:
            if predicate():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(period)

    # --- 토픽 ---
    def subscribe(self, topic: str, msg_type: type, qos_profile='reliable', raw: bool = False,
                  keep: int = 20000, keep_messages: int = 200) -> TopicRecord:
        """토픽을 구독하고 기록기를 돌려준다 (같은 토픽 재구독이면 기존 기록기)."""
        if topic in self._records:
            return self._records[topic]
        rec = TopicRecord(topic, keep, 0 if raw else keep_messages)
        if raw:
            def on_raw(data: bytes, rec=rec) -> None:
                recv = time.time()
                try:
                    stamp, frame = cdr.header(data)
                except ValueError:
                    stamp, frame = None, None
                rec.add(recv, stamp, None, frame)
            cb = on_raw
        else:
            def on_msg(msg: Any, rec=rec) -> None:
                recv = time.time()
                header = getattr(msg, 'header', None)
                rec.add(recv, stamp_seconds(msg), msg,
                        header.frame_id if header is not None else None)
            cb = on_msg
        self._subs.append(self.node.create_subscription(msg_type, topic, cb, qos(qos_profile),
                                                        raw=raw))
        self._records[topic] = rec
        return rec

    def record(self, topic: str) -> TopicRecord:
        return self._records[topic]

    def publisher(self, topic: str, msg_type: type, qos_profile='reliable'):
        key = (topic, msg_type)
        if key not in self._pubs:
            self._pubs[key] = self.node.create_publisher(msg_type, topic, qos(qos_profile))
        return self._pubs[key]

    def publish(self, topic: str, msg: Any, qos_profile='reliable') -> float:
        """발행하고 발행 직전 wall 시각을 돌려준다."""
        pub = self.publisher(topic, type(msg), qos_profile)
        t = time.time()
        pub.publish(msg)
        return t

    def wait_for_messages(self, rec: TopicRecord, n: int, timeout: float,
                          since: float = -1.0) -> bool:
        return self.wait_until(lambda: rec.count_since(since) >= n, timeout)

    def publisher_count(self, topic: str) -> int:
        return self.node.count_publishers(topic)

    def wait_for_publisher(self, topic: str, timeout: float) -> bool:
        return self.wait_until(lambda: self.node.count_publishers(topic) > 0, timeout, 0.1)

    def matching_qos(self, topic: str) -> str:
        """
        발행자 QoS 에 맞는 구독 프로필 이름.

        발행자가 모두 reliable 이면 'reliable' (드롭 없이 주기 측정), 하나라도 best-effort 면
        'sensor' (reliable 구독은 best-effort 발행자와 연결되지 않는다). 발행자가 없으면 'sensor'.
        """
        infos = self.node.get_publishers_info_by_topic(topic)
        if infos and all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                         for i in infos):
            return 'reliable'
        return 'sensor'

    # --- 그래프 ---
    def node_names(self) -> List[str]:
        """'/ns/name' 형태의 전체 노드 이름."""
        out = []
        for name, ns in self.node.get_node_names_and_namespaces():
            out.append((ns.rstrip('/') + '/' + name) if ns != '/' else '/' + name)
        return out

    def wait_for_node(self, full_name: str, timeout: float) -> bool:
        return self.wait_until(lambda: full_name in self.node_names(), timeout, 0.2)

    # --- 서비스 ---
    def call(self, srv_type: type, name: str, request: Any, timeout: float) -> Optional[Any]:
        """서비스를 동기 호출한다 (서버가 없거나 시간 초과면 None)."""
        key = (name, srv_type)
        cli = self._clients.get(key)
        if cli is None:
            cli = self.node.create_client(srv_type, name)
            self._clients[key] = cli
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        done = threading.Event()
        fut = cli.call_async(request)
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            fut.cancel()
            return None
        return fut.result()

    def service_available(self, srv_type: type, name: str, timeout: float) -> bool:
        key = (name, srv_type)
        cli = self._clients.get(key)
        if cli is None:
            cli = self.node.create_client(srv_type, name)
            self._clients[key] = cli
        return cli.wait_for_service(timeout_sec=timeout)

    # --- TF ---
    def tf_buffer(self, cache_seconds: float = 60.0):
        """tf2 Buffer + TransformListener (이 프로브의 executor 로 돈다)."""
        if self._tf_buffer is None:
            from tf2_ros.buffer import Buffer
            from tf2_ros.transform_listener import TransformListener
            self._tf_buffer = Buffer(cache_time=Duration(seconds=cache_seconds))
            self._tf_listener = TransformListener(self._tf_buffer, self.node, spin_thread=False)
        return self._tf_buffer

    def capture_tf(self) -> TfGraph:
        """
        /tf, /tf_static 를 직접 구독해 간선 그래프를 쌓는다.

        tf2 Buffer 는 자식마다 마지막 부모만 기억하므로 이중 부모 검출에는 원시 메시지가 필요하다.
        """
        if self.tf_graph is not None:
            return self.tf_graph
        from tf2_msgs.msg import TFMessage
        self.tf_graph = TfGraph()

        def make_cb(static: bool):
            def cb(msg: TFMessage) -> None:
                with self._tf_lock:
                    for tr in msg.transforms:
                        t = tr.transform.translation
                        r = tr.transform.rotation
                        self.tf_graph.add(
                            tr.header.frame_id, tr.child_frame_id,
                            tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9, static,
                            (t.x, t.y, t.z), (r.x, r.y, r.z, r.w))
            return cb

        self._subs.append(self.node.create_subscription(
            TFMessage, '/tf', make_cb(False),
            QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                       reliability=ReliabilityPolicy.RELIABLE)))
        self._subs.append(self.node.create_subscription(
            TFMessage, '/tf_static', make_cb(True), QOS['static_tf']))
        return self.tf_graph

    def tf_snapshot(self) -> TfGraph:
        """현재까지의 TF 그래프 복사본 (콜백과 경쟁하지 않게)."""
        import copy
        with self._tf_lock:
            return copy.deepcopy(self.tf_graph) if self.tf_graph else TfGraph()
