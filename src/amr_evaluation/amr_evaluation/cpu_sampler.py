"""
cpu_sampler: CPU 사용률 샘플러 (명세 4.10 "5대 운용 시 CPU 80 % 이하", multi_robot.md §7).

이미지에 mpstat(sysstat) 이 없어 /proc 을 직접 읽는다. sample_rate (기본 1 Hz, 단조 시계 타이머 —
시뮬 시간이 멈춰도 돈다) 마다 cpu.csv 에 한 행:
    [timestamp, cpu_total_percent, cpu_<그룹>_percent..., procs_<그룹>..., cpu_cgroup_percent,
     rtf, load1, cpu0, cpu1, ...]
- cpu_total_percent: /proc/stat 호스트 전체 (참고값 — 컨테이너 안에서도 호스트 전체라서 공유 서버에서는
  다른 사용자 작업이 섞인다)
- cpu_<그룹>_percent / procs_<그룹>: process_groups 정규식에 맞는 이 PID 네임스페이스 프로세스의 사용률과
  개수 (기본 ros / gazebo / nav — cpu_accounting 모듈). 시스템 귀속 값이다
- cpu_cgroup_percent: 이 컨테이너 cgroup 전체 (cgroup_root 가 '' 이면 열 없음)
- rtf: 직전 행 이후 Δ(ROS 시각)/Δ(단조 시계) — use_sim_time 이면 시뮬레이션 실시간 계수
- load1: 호스트 1 분 부하 평균 (외부 부하 기록용)
사용률은 모두 호스트 CPU 수(num_cpus, 0 이면 자동) × 경과 시간 대비 [%] 다. analyze 는 기본으로
cpu_total_percent 를 판정하고(보수적), --cpu-column cpu_ros_percent 로 시스템 귀속 값을 판정한다.
"""

import os
from pathlib import Path
import time
from typing import Dict, List, Optional, Tuple

from amr_evaluation import cpu_accounting as acct
from amr_evaluation import io
from amr_evaluation.logger_base import EvalLoggerNode
from amr_evaluation.metrics import cpu_percent, parse_proc_stat
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException


def cpu_columns(stat: Dict[str, List[int]], groups: List[str] = (), cgroup: bool = False
                ) -> List[str]:
    """cpu.csv 열: 기본 열 + 그룹 열 + cgroup + rtf + load1 + 코어(cpu0, cpu1, ... 번호순)."""
    cores = sorted((k for k in stat if k != 'cpu' and k.startswith('cpu')),
                   key=lambda k: int(k[3:]))
    cols = list(io.CPU_BASE_COLUMNS)
    cols += [f'cpu_{g}_percent' for g in groups]
    cols += [f'procs_{g}' for g in groups]
    if cgroup:
        cols.append('cpu_cgroup_percent')
    cols += ['rtf', 'load1']
    return cols + cores


class CpuSampler(EvalLoggerNode):
    """/proc 차분으로 CPU 사용률(호스트 전체 + 귀속)을 기록하는 노드."""

    def __init__(self, **node_kwargs):
        super().__init__('cpu_sampler', io.CPU_FILE, io.CPU_BASE_COLUMNS, **node_kwargs)
        self.declare_parameter('sample_rate', 1.0)          # [Hz]
        self.declare_parameter('proc_stat_path', '/proc/stat')
        self.declare_parameter('proc_root', '/proc')        # 프로세스 그룹 계산 ('' 이면 끔)
        self.declare_parameter('process_groups', list(acct.DEFAULT_GROUPS))
        self.declare_parameter('cgroup_root', '/sys/fs/cgroup')   # '' 이면 cgroup 열 없음
        self.declare_parameter('num_cpus', 0)               # 0 → 호스트 온라인 CPU 수
        self.stat_path = Path(self.p_str('proc_stat_path'))
        self.proc_root = self.p_str('proc_root')
        self.cgroup_root = self.p_str('cgroup_root')
        groups = acct.parse_groups(self.get_parameter('process_groups').value)
        self.groups = acct.ProcessGroupAccounting(
            groups if self.proc_root else [], os.sysconf('SC_CLK_TCK'))
        n = int(self.get_parameter('num_cpus').value)
        self.n_cpu = n if n > 0 else acct.host_cpu_count()
        self._cmd_cache: Dict[Tuple[int, int], str] = {}
        self._prev: Optional[Dict[str, List[int]]] = None
        self._prev_cgroup: Optional[float] = None
        self._prev_times: Optional[Tuple[float, float]] = None   # (ROS 시각, 단조 시각)
        self._cores: List[str] = []
        self._last_groups: Dict[str, Tuple[float, int]] = {}
        self.create_timer(1.0 / max(self.p_float('sample_rate'), 1e-3), self.sample,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'{self.stat_path} 를 {self.p_float("sample_rate")} Hz 로 샘플링 (호스트 CPU {self.n_cpu}, '
            f'그룹 {self.groups.names or "-"}, cgroup {self.cgroup_root or "끔"})')

    def read_stat(self) -> Dict[str, List[int]]:
        return parse_proc_stat(self.stat_path.read_text())

    def sample(self) -> None:
        cur = self.read_stat()
        mono = time.monotonic()
        ros_now = self.now_sec()
        procs = (acct.read_processes(self.proc_root, self._cmd_cache)
                 if self.groups.names else [])
        cgroup = acct.read_cgroup_usage(self.cgroup_root) if self.cgroup_root else None
        prev_times = self._prev_times
        wall_dt = mono - prev_times[1] if prev_times else 0.0
        groups = self.groups.update(procs, wall_dt, self.n_cpu)
        self._last_groups = groups
        if len(self._cmd_cache) > 4096:
            self._cmd_cache.clear()
        if self._prev is None or 'cpu' not in cur or prev_times is None:
            self._prev, self._prev_cgroup, self._prev_times = cur, cgroup, (ros_now, mono)
            return
        if self.writer is None:
            columns = cpu_columns(cur, self.groups.names, bool(self.cgroup_root))
            self._cores = [c for c in columns if c.startswith('cpu') and c[3:].isdigit()]
            self.open_writer(columns)
        row = [ros_now, cpu_percent(self._prev['cpu'], cur['cpu'])]
        row += [groups[g][0] for g in self.groups.names]
        row += [groups[g][1] for g in self.groups.names]
        if self.cgroup_root:
            row.append(acct.usage_percent(self._prev_cgroup, cgroup, wall_dt, self.n_cpu))
        rtf = (ros_now - prev_times[0]) / wall_dt if wall_dt > 0.0 else float('nan')
        row += [rtf, os.getloadavg()[0]]
        for core in self._cores:
            if core in cur and core in self._prev:
                row.append(cpu_percent(self._prev[core], cur[core]))
            else:
                row.append(float('nan'))
        self.write_row(row)
        self._prev, self._prev_cgroup, self._prev_times = cur, cgroup, (ros_now, mono)

    def progress_text(self) -> str:
        grp = ', '.join(f'{g} {pct:.1f} % ({n})' for g, (pct, n) in self._last_groups.items())
        return f'{self.rows} 샘플 ({len(self._cores)} 코어' + (f', {grp})' if grp else ')')


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
