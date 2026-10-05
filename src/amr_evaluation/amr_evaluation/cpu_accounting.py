"""
CPU 사용률의 귀속 계산 (rclpy 비의존 순수 모듈).

/proc/stat 은 네임스페이스가 없어 컨테이너 안에서도 호스트 전체를 보여 준다 — 서버를 여러 사람이 쓰면
다른 사용자 작업이 값을 좌우한다. 그래서 cpu_sampler 는 호스트 전체(참고)와 함께 다음을 기록한다:

- 프로세스 그룹: 이 PID 네임스페이스에서 보이는 프로세스의 /proc/<pid>/stat utime+stime 을
  cmdline 정규식 그룹별로 합산한다. 기본 그룹: ros (우리 ROS 2 프로세스 전체 — --ros-args,
  /opt/ros, /ros2_ws, ros2 CLI, Gazebo), gazebo (ign/gz gazebo), nav (Nav2 서버·EKF·SLAM) —
  multi_robot.md §7 의 [cpu_total_pct, cpu_gazebo_pct, cpu_nav_pct] 열에 해당한다.
  다른 컨테이너의 프로세스는 보이지 않으므로 시스템 전체를 재려면 같은 컨테이너(또는 pid: host)에서 띄운다.
- cgroup: 이 컨테이너(cgroup) 전체의 누적 CPU 시간 (v2 cpu.stat usage_usec, v1 cpuacct.usage).
백분율은 모두 "호스트 CPU 수 × 경과 벽시계 시간" 대비라서 호스트 전체 값과 같은 척도다
(multi_robot.md §7: 32 스레드 × 80 % = 25.6 코어).
"""

import os
from pathlib import Path
import re
from typing import Dict, Iterable, List, Optional, Pattern, Sequence, Tuple

NAV_PROCESSES = ('nav2', 'component_container', 'controller_server', 'planner_server',
                 'bt_navigator', 'behavior_server', 'smoother_server', 'velocity_smoother',
                 'waypoint_follower', 'amcl', 'map_server', 'ekf_node', 'slam_toolbox')
GAZEBO_RE = r'(^|[ /])(ign|gz) (gazebo|sim)\b|gzserver|ign-gazebo|gz-sim'
DEFAULT_GROUPS = (
    'ros=--ros-args|/opt/ros/|/ros2_ws/|(^|/)ros2( |$)|' + GAZEBO_RE,
    'gazebo=' + GAZEBO_RE,
    'nav=' + '|'.join(NAV_PROCESSES),
)


def parse_groups(specs: Sequence[str]) -> List[Tuple[str, Pattern]]:
    """['name=regex', ...] → [(name, 컴파일된 정규식)]. 이름은 [a-z0-9_] 만."""
    out: List[Tuple[str, Pattern]] = []
    for spec in specs:
        name, sep, pattern = str(spec).partition('=')
        name = name.strip()
        if not sep or not re.fullmatch(r'[a-z0-9_]+', name) or not pattern:
            raise ValueError(f"process group must be 'name=regex' (name [a-z0-9_]+), got {spec!r}")
        out.append((name, re.compile(pattern)))
    return out


def parse_pid_stat(text: str) -> Optional[Tuple[int, int]]:
    """/proc/<pid>/stat → (utime + stime [tick], starttime [tick]). 형식이 다르면 None."""
    try:
        rest = text[text.rindex(')') + 2:].split()
        return int(rest[11]) + int(rest[12]), int(rest[19])
    except (ValueError, IndexError):
        return None


def read_processes(proc_root: str = '/proc',
                   cmdline_cache: Optional[Dict[Tuple[int, int], str]] = None
                   ) -> List[Tuple[int, int, int, str]]:
    """
    보이는 프로세스 목록 [(pid, starttime, ticks, cmdline)]. 커널 스레드(cmdline 빈 것)는 뺀다.

    cmdline_cache 를 넘기면 (pid, starttime) 별로 cmdline 을 한 번만 읽는다.
    """
    root = Path(proc_root)
    out = []
    try:
        entries = [p for p in root.iterdir() if p.name.isdigit()]
    except OSError:
        return out
    for entry in entries:
        try:
            parsed = parse_pid_stat((entry / 'stat').read_text(errors='replace'))
            if parsed is None:
                continue
            ticks, start = parsed
            key = (int(entry.name), start)
            cmd = cmdline_cache.get(key) if cmdline_cache is not None else None
            if cmd is None:
                raw = (entry / 'cmdline').read_bytes()
                cmd = raw.replace(b'\0', b' ').decode('utf-8', 'replace').strip()
                if cmdline_cache is not None:
                    cmdline_cache[key] = cmd
        except OSError:          # 읽는 사이에 끝난 프로세스
            continue
        if cmd:
            out.append((int(entry.name), start, ticks, cmd))
    return out


class ProcessGroupAccounting:
    """프로세스 그룹별 CPU 사용률 [%] — 두 스냅샷 사이의 utime+stime 증가분."""

    def __init__(self, groups: Sequence[Tuple[str, Pattern]], clk_tck: int = 100):
        self.groups = list(groups)
        self.clk_tck = clk_tck
        self._prev: Optional[Dict[Tuple[int, int], int]] = None

    @property
    def names(self) -> List[str]:
        return [name for name, _ in self.groups]

    def update(self, procs: Iterable[Tuple[int, int, int, str]], wall_dt: float, n_cpu: int
               ) -> Dict[str, Tuple[float, int]]:
        """
        스냅샷 입력 → {그룹: (사용률 %, 일치 프로세스 수)}. 첫 호출은 기준점이라 사용률 NaN.

        새로 보인 프로세스는 시작 이후 누적 전부를 이번 구간에 넣는다 (직전 스냅샷 뒤에 시작했으므로).
        두 스냅샷 사이에 끝난 프로세스의 마지막 구간은 빠진다 (1 Hz 에서 무시할 만하다).
        """
        cur: Dict[Tuple[int, int], int] = {}
        used = {name: 0 for name in self.names}
        count = {name: 0 for name in self.names}
        for pid, start, ticks, cmd in procs:
            key = (pid, start)
            cur[key] = ticks
            delta = ticks - self._prev.get(key, 0) if self._prev is not None else 0
            for name, pattern in self.groups:
                if pattern.search(cmd):
                    count[name] += 1
                    used[name] += max(delta, 0)
        first = self._prev is None
        self._prev = cur
        out: Dict[str, Tuple[float, int]] = {}
        for name in self.names:
            if first or wall_dt <= 0.0 or n_cpu <= 0:
                out[name] = (float('nan'), count[name])
            else:
                pct = 100.0 * used[name] / self.clk_tck / (wall_dt * n_cpu)
                out[name] = (pct, count[name])
        return out


def parse_cgroup_v2_usage(text: str) -> Optional[float]:
    """Cgroup v2 cpu.stat 본문 → 누적 CPU 시간 [s] (usage_usec)."""
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0] == 'usage_usec':
            return int(parts[1]) * 1e-6
    return None


CGROUP_V1_FILES = ('cpuacct/cpuacct.usage', 'cpu,cpuacct/cpuacct.usage', 'cpuacct.usage')


def read_cgroup_usage(root: str = '/sys/fs/cgroup') -> Optional[float]:
    """이 프로세스가 속한 cgroup(컨테이너)의 누적 CPU 시간 [s]. 못 읽으면 None."""
    base = Path(root)
    try:
        usage = parse_cgroup_v2_usage((base / 'cpu.stat').read_text())
        if usage is not None:
            return usage
    except OSError:
        pass
    for rel in CGROUP_V1_FILES:
        try:
            return int((base / rel).read_text().strip()) * 1e-9
        except (OSError, ValueError):
            continue
    return None


def usage_percent(prev: Optional[float], cur: Optional[float], wall_dt: float,
                  n_cpu: int) -> float:
    """누적 CPU 시간 두 값 → 호스트 CPU 대비 사용률 [%] (못 구하면 NaN)."""
    if prev is None or cur is None or wall_dt <= 0.0 or n_cpu <= 0 or cur < prev:
        return float('nan')
    return 100.0 * (cur - prev) / (wall_dt * n_cpu)


def host_cpu_count() -> int:
    """호스트 온라인 CPU 수 (컨테이너에서도 호스트 값)."""
    return os.cpu_count() or 1
