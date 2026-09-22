"""
프로세스·CPU 감시 (rclpy 비의존 순수 모듈, 시나리오 12 CPU · 14 장시간 안정성).

  ros_processes()   --ros-args 를 가진 프로세스(= ROS 2 노드) 목록 {pid: 이름}
  rss_mb()          /proc/<pid>/status VmRSS [MB]
  CpuSampler        /proc/stat 두 시점 차이로 전체 CPU 사용률 [%]
  slope_per_hour()  최소제곱 기울기 (메모리 누수 판정: MB/h)
/proc 경로는 인자로 받아 단위 테스트에서 가짜 트리를 쓸 수 있게 했다.
"""

from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

PROC = Path('/proc')


def _cmdline(pid_dir: Path) -> Sequence[str]:
    try:
        raw = (pid_dir / 'cmdline').read_bytes()
    except OSError:
        return []
    return [a.decode('utf-8', errors='replace') for a in raw.split(b'\x00') if a]


def node_name(args: Sequence[str]) -> str:
    """명령줄에서 노드 이름 (__node:=X 또는 -r __node:=X), 없으면 실행 파일 이름."""
    for a in args:
        if a.startswith('__node:='):
            return a.split(':=', 1)[1]
    return Path(args[0]).name if args else ''


def ros_processes(proc: Path = PROC) -> Dict[int, str]:
    """--ros-args 를 명령줄에 가진 프로세스 {pid: 노드 이름}."""
    out: Dict[int, str] = {}
    for d in proc.iterdir():
        if not d.name.isdigit():
            continue
        args = _cmdline(d)
        if '--ros-args' in args:
            out[int(d.name)] = node_name(args)
    return out


def rss_mb(pid: int, proc: Path = PROC) -> Optional[float]:
    """프로세스 VmRSS [MB] (프로세스가 없으면 None)."""
    try:
        text = (proc / str(pid) / 'status').read_text()
    except OSError:
        return None
    for line in text.splitlines():
        if line.startswith('VmRSS:'):
            return float(line.split()[1]) / 1024.0
    return None


def read_cpu_times(proc: Path = PROC) -> Tuple[float, float]:
    """/proc/stat 첫 줄 → (전체 jiffies, idle+iowait jiffies)."""
    first = (proc / 'stat').read_text().splitlines()[0].split()
    vals = [float(v) for v in first[1:]]
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0.0)
    return sum(vals[:8]), idle


class CpuSampler:
    """두 번의 sample() 사이 전체 CPU 사용률 [%]."""

    def __init__(self, proc: Path = PROC):
        self.proc = proc
        self._last = read_cpu_times(proc)

    def sample(self) -> float:
        total, idle = read_cpu_times(self.proc)
        dt, di = total - self._last[0], idle - self._last[1]
        self._last = (total, idle)
        return 100.0 * (1.0 - di / dt) if dt > 0 else 0.0


def process_jiffies(pid: int, proc: Path = PROC) -> Optional[float]:
    """/proc/<pid>/stat 의 utime + stime [jiffies] (없으면 None)."""
    try:
        text = (proc / str(pid) / 'stat').read_text()
    except OSError:
        return None
    fields = text.rsplit(')', 1)[1].split()     # comm 에 공백이 있을 수 있어 ')' 뒤부터
    return float(fields[11]) + float(fields[12])


class ProcessCpuSampler:
    """
    지정한 프로세스들의 CPU 사용률 합 [% of 전체 코어] — 공유 호스트에서 외부 부하를 뺀 값.

    /proc/stat 전체 사용률(amr_evaluation cpu_sampler)은 같은 호스트의 다른 작업까지 포함하므로,
    시스템 자신의 몫은 프로세스별 utime+stime 증분 / 전체 jiffies 증분으로 따로 잰다.
    """

    def __init__(self, pids: Sequence[int], proc: Path = PROC):
        self.proc = proc
        self.pids = list(pids)
        self._last_total = read_cpu_times(proc)[0]
        self._last = {p: process_jiffies(p, proc) for p in self.pids}

    def sample(self) -> float:
        total = read_cpu_times(self.proc)[0]
        used = 0.0
        for p in self.pids:
            now = process_jiffies(p, self.proc)
            prev = self._last.get(p)
            if now is not None and prev is not None:
                used += now - prev
            self._last[p] = now
        dt = total - self._last_total
        self._last_total = total
        return 100.0 * used / dt if dt > 0 else 0.0


def slope_per_hour(times_s: Sequence[float], values: Sequence[float]) -> float:
    """최소제곱 직선 기울기 [단위/h] (표본 < 3 이면 NaN)."""
    t = np.asarray(times_s, dtype=float)
    v = np.asarray(values, dtype=float)
    ok = np.isfinite(t) & np.isfinite(v)
    if np.sum(ok) < 3:
        return float('nan')
    slope, _ = np.polyfit(t[ok], v[ok], 1)
    return float(slope * 3600.0)
