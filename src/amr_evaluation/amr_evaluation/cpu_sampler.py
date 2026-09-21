"""
cpu_sampler: /proc/stat 기반 CPU 사용률 샘플러 (명세 4.10 "5대 운용 시 CPU 80 % 이하").

이미지에 mpstat(sysstat) 이 없어 /proc/stat 을 직접 읽는다. sample_rate (기본 1 Hz) 마다
두 스냅샷의 jiffies 차분으로 전체·코어별 사용률을 구해 cpu.csv 에 쓴다:
    [timestamp, cpu_total_percent, cpu0, cpu1, ...]
컨테이너 안에서도 /proc/stat 은 호스트 전체 CPU 를 보여 준다 (5대 동시 운용 부하 = 호스트 부하).
"""

from pathlib import Path
from typing import Dict, List, Optional

from amr_evaluation import io
from amr_evaluation.logger_base import EvalLoggerNode
from amr_evaluation.metrics import cpu_percent, parse_proc_stat
import rclpy
from rclpy.executors import ExternalShutdownException


def cpu_columns(stat: Dict[str, List[int]]) -> List[str]:
    """/proc/stat 스냅샷의 코어 키(cpu0, cpu1, ...) 를 번호순으로."""
    cores = sorted((k for k in stat if k != 'cpu' and k.startswith('cpu')),
                   key=lambda k: int(k[3:]))
    return io.CPU_BASE_COLUMNS + cores


class CpuSampler(EvalLoggerNode):
    """/proc/stat 차분으로 CPU 사용률을 기록하는 노드."""

    def __init__(self, **node_kwargs):
        super().__init__('cpu_sampler', io.CPU_FILE, io.CPU_BASE_COLUMNS, **node_kwargs)
        self.declare_parameter('sample_rate', 1.0)          # [Hz]
        self.declare_parameter('proc_stat_path', '/proc/stat')
        self.stat_path = Path(self.p_str('proc_stat_path'))
        self._prev: Optional[Dict[str, List[int]]] = None
        self._cores: List[str] = []
        self.create_timer(1.0 / max(self.p_float('sample_rate'), 1e-3), self.sample)
        self.get_logger().info(f'{self.stat_path} 를 {self.p_float("sample_rate")} Hz 로 샘플링')

    def read_stat(self) -> Dict[str, List[int]]:
        return parse_proc_stat(self.stat_path.read_text())

    def sample(self) -> None:
        cur = self.read_stat()
        if self._prev is None or 'cpu' not in cur:
            self._prev = cur
            return
        if self.writer is None:
            columns = cpu_columns(cur)
            self._cores = columns[len(io.CPU_BASE_COLUMNS):]
            self.open_writer(columns)
        now = self.get_clock().now().nanoseconds * 1e-9
        row = [now, cpu_percent(self._prev['cpu'], cur['cpu'])]
        for core in self._cores:
            if core in cur and core in self._prev:
                row.append(cpu_percent(self._prev[core], cur[core]))
            else:
                row.append(float('nan'))
        self.write_row(row)
        self._prev = cur

    def progress_text(self) -> str:
        return f'{self.rows} 샘플 ({len(self._cores)} 코어)'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CpuSampler()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
