"""
프로세스·CPU 감시 (rclpy 비의존 순수 모듈, 시나리오 12 CPU · 14 장시간 안정성).

  ros_processes()   --ros-args 를 가진 프로세스(= ROS 2 노드) 목록 {pid: 이름}
  rss_mb()          /proc/<pid>/status VmRSS [MB]
  CpuSampler        /proc/stat 두 시점 차이로 전체 CPU 사용률 [%]
  slope_per_hour()  최소제곱 기울기 [단위/h]
  leak_verdict()    누수 판정 (기울기 + 요동 폭 초과 증가량, 표본 부족은 판정 불가)
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


def is_simulator(args: Sequence[str]) -> bool:
    """Gazebo 서버 명령줄인가 (Fortress: ruby /usr/bin/ign gazebo … — 서버가 ruby 프로세스 안에서 돈다)."""
    words = [Path(a).name for a in args[:3]]
    return ('gazebo' in args[:4] and ('ign' in words or 'gz' in words)) \
        or any(w.startswith(('ign-gazebo', 'gz-sim')) for w in words)


def system_processes(proc: Path = PROC) -> Dict[int, str]:
    """
    시스템 프로세스 = ROS 노드(--ros-args) + 시뮬레이터 서버 {pid: 이름}.

    명세 4.10 "5대 운용 시 CPU 80 %" 의 분자: 시뮬레이터도 시스템의 일부로 센다 (시뮬레이터를 빼면
    Gazebo 가 쓰는 몇 코어가 사라져 기준이 헐거워진다). 하네스 러너(launch_testing)는 뺀다.
    """
    out = ros_processes(proc)
    for d in proc.iterdir():
        if not d.name.isdigit() or int(d.name) in out:
            continue
        args = _cmdline(d)
        if args and is_simulator(args):
            out[int(d.name)] = 'gazebo_server'
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


def cpu_count(proc: Path = PROC) -> int:
    """/proc/stat 의 cpuN 줄 수 (호스트 코어 = CPU 사용률 분모의 코어 수)."""
    lines = (proc / 'stat').read_text().splitlines()
    return max(1, sum(1 for ln in lines if ln.startswith('cpu') and ln[3:4].isdigit()))


class ProcessCpuSampler:
    """
    지정한 프로세스들의 CPU 사용률 합 [% of 전체 코어] — 공유 호스트에서 외부 부하를 뺀 값.

    /proc/stat 전체 사용률(amr_evaluation cpu_sampler)은 같은 호스트의 다른 작업까지 포함하므로,
    시스템 자신의 몫은 프로세스별 utime+stime 증분 / 전체 jiffies 증분으로 따로 잰다 (분모 = 호스트 전체 코어).
    cores_used() 는 같은 값을 "코어 몇 개 분" 으로 (% × 코어 수 / 100). update_pids() 로 새로 뜬 노드를 더한다.
    """

    def __init__(self, pids: Sequence[int], proc: Path = PROC):
        self.proc = proc
        self.pids = list(pids)
        self.cores = cpu_count(proc)
        self._last_total = read_cpu_times(proc)[0]
        self._last = {p: process_jiffies(p, proc) for p in self.pids}
        self.last_percent = 0.0

    def update_pids(self, pids: Sequence[int]) -> None:
        """감시 대상을 바꾼다 (새 pid 는 다음 sample 부터 증분을 센다)."""
        for p in pids:
            if p not in self._last:
                self._last[p] = process_jiffies(p, self.proc)
        self.pids = list(pids)

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
        self.last_percent = 100.0 * used / dt if dt > 0 else 0.0
        return self.last_percent

    def cores_used(self) -> float:
        """마지막 sample 의 사용량 [코어 수]."""
        return self.last_percent * self.cores / 100.0


def slope_per_hour(times_s: Sequence[float], values: Sequence[float]) -> float:
    """최소제곱 직선 기울기 [단위/h] (표본 < 3 이면 NaN)."""
    t = np.asarray(times_s, dtype=float)
    v = np.asarray(values, dtype=float)
    ok = np.isfinite(t) & np.isfinite(v)
    if np.sum(ok) < 3:
        return float('nan')
    slope, _ = np.polyfit(t[ok], v[ok], 1)
    return float(slope * 3600.0)


RSS_NOISE_MB = 2.0          # 할당기 청크 단위 RSS 요동 (14 실측: gz_image_bridge 142.6↔144.1 MB 왕복)
LEAK_MIN_POINTS = 5


def leak_verdict(times_s: Sequence[float], values: Sequence[float], max_mb_h: float,
                 noise_mb: float = RSS_NOISE_MB,
                 min_points: int = LEAK_MIN_POINTS) -> Tuple[str, float, float]:
    """
    메모리 누수 판정 → (상태, 기울기 MB/h, 창 동안 적합 증가량 MB).

    상태: 'leak' = 기울기 > max_mb_h 이고 적합 증가량(기울기 × 창 길이) > noise_mb,
    'ok', 'insufficient' = 유효 표본 < min_points (판정 불가 — 통과로 세지 않는다).
    짧은 창에서 1~2 MB 왕복 요동을 h 당으로 외삽하면 수십 MB/h 가 되므로 (0.25 h 스모크, 창 4 분에서
    gz_image_bridge 18 MB/h) 증가량이 요동 폭을 넘을 때만 누수로 본다. 4 h 캠페인에서는 5 MB/h × 3.8 h
    = 19 MB ≫ noise_mb 라 기준이 느슨해지지 않는다.
    """
    t = np.asarray(times_s, dtype=float)
    v = np.asarray(values, dtype=float)
    ok = np.isfinite(t) & np.isfinite(v)
    if np.sum(ok) < max(min_points, 3):
        return 'insufficient', float('nan'), float('nan')
    slope = slope_per_hour(t[ok], v[ok])
    growth = slope * float(np.max(t[ok]) - np.min(t[ok])) / 3600.0
    if slope > max_mb_h and growth > noise_mb:
        return 'leak', slope, growth
    return 'ok', slope, growth
