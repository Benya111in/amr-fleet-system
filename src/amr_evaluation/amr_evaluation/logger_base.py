"""측정 로거 노드 공통부: 런 디렉토리 파라미터, QoS, CSV 기록기, 시계 영역 검사, 종료 시 요약 로그."""

from typing import Dict, Optional, Sequence

from amr_evaluation import clocks
from amr_evaluation import io
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class EvalLoggerNode(Node):
    """
    로거 노드 베이스.

    공통 파라미터
      output_dir: 로그 루트 (기본 '' → $ROS_WS/logs/eval)
      run_name:   런 이름 (기본 '' → run_YYYYmmdd_HHMMSS; launch 가 모든 로거에 같은 값을 준다)
      best_effort: 구독 QoS 를 best-effort 로 (센서 QoS 발행자와 맞출 때)
      report_period: 진행 상황 로그 주기 [s] (0 이면 끔)
    CSV 이름에는 노드 네임스페이스를 붙인다 (/amr_01 → pose_error_amr_01.csv): 로봇마다 같은 run_name
    으로 띄워도 서로 덮어쓰지 않는다. 같은 이름의 파일이 이미 있으면 _1, _2 … 를 붙여 새로 만든다.
    """

    def __init__(self, name: str, csv_name: str, columns: Sequence[str], **node_kwargs):
        super().__init__(name, **node_kwargs)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('run_name', '')
        self.declare_parameter('best_effort', False)
        self.declare_parameter('report_period', 5.0)
        self.run_dir = io.resolve_run_dir(self.p_str('output_dir'), self.p_str('run_name'))
        self.writer: Optional[io.CsvWriter] = None
        self._csv_name = io.namespaced_name(csv_name, self.get_namespace())
        self._columns = list(columns)
        self.clock_mismatch: Dict[str, int] = {}
        period = float(self.get_parameter('report_period').value)
        if period > 0.0:
            self.create_timer(period, self._report_progress)

    # --- 파라미터 헬퍼 ---
    def p_str(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def p_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def p_bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def sub_qos(self, depth: int = 50) -> QoSProfile:
        rel = (ReliabilityPolicy.BEST_EFFORT if self.p_bool('best_effort')
               else ReliabilityPolicy.RELIABLE)
        return QoSProfile(reliability=rel, history=HistoryPolicy.KEEP_LAST, depth=depth)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- 시계 영역 ---
    @property
    def expected_domain(self) -> str:
        """이 노드의 use_sim_time 으로 기대하는 스탬프 영역 (sim | wall)."""
        return clocks.expected_domain(self.p_bool('use_sim_time'))

    def check_clock(self, stream: str, stamp: float) -> bool:
        """
        입력 스탬프가 노드의 시계 영역과 같은지 본다. 다르면 스트림마다 처음 한 번 경고하고 센다.

        한쪽만 use_sim_time 이면 시뮬 시간(0 부터)과 벽시계 에포크(~1.8e9)가 섞여 짝짓기가 조용히
        실패한다 — 발행 노드와 로거의 use_sim_time 을 맞춘다 (launch 기본값은 모두 true).
        """
        if stamp <= 0.0:
            return True
        domain = clocks.clock_domain(stamp)
        if domain == self.expected_domain:
            return True
        count = self.clock_mismatch.get(stream, 0)
        if count == 0:
            self.get_logger().warn(
                f'{stream} 스탬프 {stamp:.3f} s 는 {domain} 시계인데 이 노드는 '
                f'use_sim_time={self.p_bool("use_sim_time")} ({self.expected_domain}) — '
                '발행 쪽과 로거의 use_sim_time 을 맞춘다')
        self.clock_mismatch[stream] = count + 1
        return False

    def clock_text(self) -> str:
        if not self.clock_mismatch:
            return ''
        return ', 시계 불일치 ' + ' / '.join(f'{k} {v}' for k, v in self.clock_mismatch.items())

    # --- 기록 ---
    def open_writer(self, columns: Optional[Sequence[str]] = None) -> io.CsvWriter:
        """CSV 를 연다 (열이 실행 시점에 정해지는 노드는 columns 를 넘긴다)."""
        if columns is not None:
            self._columns = list(columns)
        path = io.unique_path(self.run_dir / self._csv_name)
        if path.name != self._csv_name:
            self.get_logger().warn(f'{self._csv_name} 가 이미 있어 {path.name} 로 기록한다')
        self.writer = io.CsvWriter(path, self._columns)
        self.get_logger().info(f'기록 시작: {self.writer.path}')
        return self.writer

    def write_row(self, values: Sequence) -> None:
        if self.writer is None:
            self.open_writer()
        self.writer.write(values)

    @property
    def rows(self) -> int:
        return self.writer.rows if self.writer else 0

    def progress_text(self) -> str:
        """진행 로그 본문 (하위 클래스가 덧붙인다)."""
        return f'{self.rows} 행 기록'

    def _report_progress(self) -> None:
        self.get_logger().info(self.progress_text() + self.clock_text())

    def finish(self) -> None:
        """종료 처리: 남은 데이터 flush, CSV 닫기, 요약 로그."""
        self.on_finish()
        if self.writer is not None:
            self.writer.close()
            self.get_logger().info(
                f'종료: {self.progress_text()}{self.clock_text()} → {self.writer.path}')

    def on_finish(self) -> None:
        """하위 클래스 훅 (기본 없음)."""
