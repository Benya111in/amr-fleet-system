"""측정 로거 노드 공통부: 런 디렉토리 파라미터, QoS, CSV 기록기, 종료 시 요약 로그."""

from typing import Optional, Sequence

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
    """

    def __init__(self, name: str, csv_name: str, columns: Sequence[str], **node_kwargs):
        super().__init__(name, **node_kwargs)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('run_name', '')
        self.declare_parameter('best_effort', False)
        self.declare_parameter('report_period', 5.0)
        self.run_dir = io.resolve_run_dir(self.p_str('output_dir'), self.p_str('run_name'))
        self.writer: Optional[io.CsvWriter] = None
        self._csv_name = csv_name
        self._columns = list(columns)
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

    # --- 기록 ---
    def open_writer(self, columns: Optional[Sequence[str]] = None) -> io.CsvWriter:
        """CSV 를 연다 (열이 실행 시점에 정해지는 노드는 columns 를 넘긴다)."""
        if columns is not None:
            self._columns = list(columns)
        self.writer = io.CsvWriter(self.run_dir / self._csv_name, self._columns)
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
        self.get_logger().info(self.progress_text())

    def finish(self) -> None:
        """종료 처리: 남은 데이터 flush, CSV 닫기, 요약 로그."""
        self.on_finish()
        if self.writer is not None:
            self.writer.close()
            self.get_logger().info(f'종료: {self.progress_text()} → {self.writer.path}')

    def on_finish(self) -> None:
        """하위 클래스 훅 (기본 없음)."""
