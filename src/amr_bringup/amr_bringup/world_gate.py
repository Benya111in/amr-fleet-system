"""
Gazebo 월드 게이트: 월드 로드 대기 → (선택) 일시 정지 → 스폰 확인 → 재개.

system.launch.py / multi_robot.launch.py 가 ExecuteProcess 로 두 번 부른다 (launch_utils.spawn_sequence).

    python3 -m amr_bringup.world_gate hold --world warehouse --pause
    python3 -m amr_bringup.world_gate release --world warehouse --models amr_01,amr_02 --unpause
    ros2 run amr_bringup world_gate release --world warehouse --models amr_01 --unpause   # 수동 재개

hold     /world/<w>/create 서비스(UserCommands)가 보일 때까지 기다린다 = 월드 로드 완료 (warehouse.launch.py 의
         스폰 대기와 같은 기준). --pause 면 월드가 뜨기 전부터 /world/<w>/control 에 pause 요청을 걸어 둔다 —
         ign service -s 는 서비스가 나타날 때까지(--timeout) 기다렸다가 곧바로 보내므로 로드 직후 멈춘다
         (목록을 폴링한 뒤 멈추면 그 사이 sim 이 최대 3 s 흘렀다, 실측). 멈춘 sim 시각을 알린다.
release  `ign model --list` 에 --models 가 모두 보일 때까지 기다린 뒤 --unpause 면 재개한다. 시간이 넘어도
         재개한다 (멈춘 월드를 남기지 않는다) — 빠진 모델은 ERROR 로 알린다. ros_gz_sim create 는 응답을
         5 s 안에 못 받으면 실패하고도 종료 코드 0 이라(spawn.launch.py 주석) 모델 목록으로 확인한다.
종료 코드: 0 정상, 2 일부 모델 누락(재개는 함), 3 월드 없음(시간 초과), 4 일시 정지/재개 요청 실패.

gz CLI(ign service/model/topic)는 IGN_PARTITION 을 서버와 같게 물려받아야 한다 (multi_robot.md §5).

왜 멈춘 채로 스폰하나 (결정; 수치는 5대, Fortress 6.18 실측)
    Fortress(ign-sensors6) 센서의 다음 갱신 시각은 0 에서 시작해 한 주기씩만 늘어나서, sim 시각 T 에 생긴 센서는
    T × update_rate 번을 매 물리 스텝마다 따라잡는다 (spawn.launch.py 주석). warehouse.launch.py 는 -r 로 곧바로
    달리므로 xacro·create·센서 생성이 끝날 때까지 T 가 자란다 — 고부하에서는 수십 초.
    - 멈추지 않으면: 센서가 T ≈ 3.2 s 에 생겨 로봇마다 scan 32, camera 97, depth 48, imu 350 개가 1 ms 간격
      stamp 로 3~5 s(벽시계) 동안 쏟아졌다 (월드는 계속 달림).
    - 멈추면: 렌더링 센서(scan/camera/depth)의 따라잡기는 멈춘 동안 끝난다 (같은 stamp T 로 발행, 스택은 아직
      없음). 재개 뒤에는 짧은 간격이 없다. IMU(비렌더링, Imu 시스템)만 재개 직후 0.8 s 동안 따라잡는다.
    - 멈춘 동안에도 create(UserCommands)와 ign model --list 는 동작한다 (5대 확인 6~8 s).
    그래서 T 는 스폰에 걸린 시간과 무관하게 멈춘 시각(로드 직후)에 묶인다.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from typing import Callable, List, Optional, Sequence

EXIT_OK, EXIT_MISSING, EXIT_NO_WORLD, EXIT_CONTROL = 0, 2, 3, 4
CLI_TIMEOUT_S = 30.0                   # ign CLI 한 번 (ruby 기동 + 디스커버리, 고부하에서 수 초)

Runner = Callable[[Sequence[str], float], str]


def run_cli(cmd: Sequence[str], timeout: float) -> str:
    """명령의 표준 출력 (실패·시간 초과면 빈 문자열)."""
    try:
        done = subprocess.run(list(cmd), capture_output=True, text=True, timeout=timeout,
                              check=False)
    except (OSError, subprocess.TimeoutExpired):
        return ''
    return done.stdout or ''


def service_listed(text: str, service: str) -> bool:
    """`ign service -l` 출력에 service 가 한 줄로 있는지."""
    return any(line.strip() == service for line in text.splitlines())


def parse_model_list(text: str) -> List[str]:
    """`ign model --list` 출력 ("    - amr_01") → 모델 이름 목록."""
    return [m.group(1) for m in re.finditer(r'^\s*-\s+(\S+)\s*$', text, re.MULTILINE)]


def reply_ok(text: str) -> bool:
    """ignition.msgs.Boolean 응답 텍스트가 data: true 인지."""
    return re.search(r'\bdata:\s*true\b', text) is not None


def parse_sim_time(text: str) -> Optional[float]:
    """월드 통계(WorldStatistics) 텍스트의 sim_time [s]. 없으면 None (값 0 인 필드는 생략된다)."""
    block = re.search(r'\bsim_time\s*\{([^}]*)\}', text)
    if block is None:
        return None
    sec = re.search(r'\bsec:\s*(\d+)', block.group(1))
    nsec = re.search(r'\bnsec:\s*(\d+)', block.group(1))
    return (int(sec.group(1)) if sec else 0) + (int(nsec.group(1)) if nsec else 0) * 1e-9


def parse_paused(text: str) -> Optional[bool]:
    """월드 통계(WorldStatistics) 텍스트의 paused. 메시지 없음 None, 필드 없음(기본값) False."""
    if not text.strip():
        return None
    m = re.search(r'\bpaused:\s*(true|false)\b', text)
    return bool(m and m.group(1) == 'true')


class WorldGate:
    """Ign CLI 로 한 월드를 조회·제어한다 (runner/clock/sleep 은 시험용 주입)."""

    def __init__(self, world: str, runner: Runner = run_cli,
                 clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep,
                 log: Callable[[str], None] = None):
        self.world = world
        self._run = runner
        self._clock = clock
        self._sleep = sleep
        self._log = log or (lambda msg: print(f'[world_gate] {msg}', flush=True))

    def world_ready(self) -> bool:
        return service_listed(self._run(['ign', 'service', '-l'], CLI_TIMEOUT_S),
                              f'/world/{self.world}/create')

    def models(self) -> List[str]:
        return parse_model_list(self._run(['ign', 'model', '--list'], CLI_TIMEOUT_S))

    def stats(self) -> str:
        return self._run(['ign', 'topic', '-e', '-t', f'/world/{self.world}/stats', '-n', '1'],
                         CLI_TIMEOUT_S)

    def request_pause(self, paused: bool, timeout_ms: int = 5000) -> bool:
        """월드 제어(WorldControl) 요청 한 번. 서비스가 없으면 timeout_ms 동안 나타나기를 기다린다."""
        req = f'pause: {"true" if paused else "false"}'
        return reply_ok(self._run(['ign', 'service', '-s', f'/world/{self.world}/control',
                                   '--reqtype', 'ignition.msgs.WorldControl',
                                   '--reptype', 'ignition.msgs.Boolean',
                                   '--timeout', str(timeout_ms), '--req', req], CLI_TIMEOUT_S))

    def set_paused(self, paused: bool, attempts: int = 5) -> bool:
        """응답 data: true 를 받을 때까지 attempts 번 요청한다."""
        for _ in range(attempts):
            if self.request_pause(paused):
                return True
            self._sleep(1.0)
        return False

    def pause_when_up(self, timeout: float) -> bool:
        """월드가 뜨기 전부터 pause 를 요청해 로드 직후 멈춘다 (한 번에 3 s 씩 서비스 등장을 기다림)."""
        deadline = self._clock() + timeout
        while True:
            if self.request_pause(True, timeout_ms=3000):
                return True
            if self._clock() >= deadline:
                return False
            self._sleep(0.2)

    def wait_world(self, timeout: float, period: float = 1.0) -> bool:
        deadline = self._clock() + timeout
        while True:
            if self.world_ready():
                return True
            if self._clock() >= deadline:
                return False
            self._sleep(period)

    def wait_models(self, names: Sequence[str], timeout: float, period: float = 1.0) -> List[str]:
        """모델이 모두 보이거나 시간이 넘을 때까지 기다리고 빠진 이름 목록을 돌려준다."""
        deadline = self._clock() + timeout
        while True:
            present = set(self.models())
            missing = [n for n in names if n not in present]
            if not missing or self._clock() >= deadline:
                return missing
            self._sleep(period)

    def describe_time(self) -> str:
        text = self.stats()
        t, paused = parse_sim_time(text), parse_paused(text)
        if t is None and paused is None:
            return 'sim 시각 미확인'
        return f'sim t={t or 0.0:.2f} s, paused={paused}'

    def hold(self, pause: bool, timeout: float) -> int:
        t0 = self._clock()
        self._log(f'/world/{self.world} 대기 (최대 {timeout:.0f} s'
                  + (', 뜨면 곧바로 일시 정지)' if pause else ')'))
        paused = pause and self.pause_when_up(timeout)
        if not self.wait_world(max(timeout - (self._clock() - t0), 0.0)):
            self._log(f'ERROR: {timeout:.0f} s 안에 월드 {self.world} 가 뜨지 않았다 → 스폰 안 함')
            return EXIT_NO_WORLD
        if pause and not paused:
            self._log('ERROR: 월드 일시 정지 요청 실패 → 멈추지 않고 스폰한다')
            return EXIT_CONTROL
        self._log(f'월드 준비 {self._clock() - t0:.1f} s'
                  + (f', 일시 정지 ({self.describe_time()})' if pause else ''))
        return EXIT_OK

    def release(self, names: Sequence[str], unpause: bool, timeout: float) -> int:
        t0 = self._clock()
        missing = self.wait_models(names, timeout)
        if missing:
            self._log(f'ERROR: {timeout:.0f} s 안에 스폰되지 않은 모델: {", ".join(missing)} '
                      '(ros_gz_sim create 로그 확인)')
        else:
            # 멈추지 않았다면 이 sim 시각이 곧 늦은 스폰의 센서 따라잡기 크기 (T × update_rate)
            self._log(f'모델 {len(names)}개 확인 ({self._clock() - t0:.1f} s, '
                      f'{self.describe_time()})')
        if unpause:
            if not self.set_paused(False):
                self._log('ERROR: 월드 재개 요청 실패 — '
                          f'ign service -s /world/{self.world}/control ... pause: false 로 재개')
                return EXIT_CONTROL
            self._log(f'월드 재개 ({self.describe_time()})')
        return EXIT_MISSING if missing else EXIT_OK


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog='world_gate', description=__doc__.split('\n')[1])
    sub = parser.add_subparsers(dest='cmd', required=True)
    hold = sub.add_parser('hold', help='월드 로드 대기 (+ 일시 정지)')
    hold.add_argument('--world', required=True)
    hold.add_argument('--pause', action='store_true')
    hold.add_argument('--timeout', type=float, default=600.0, help='[s] 월드 로드 대기 상한')
    rel = sub.add_parser('release', help='모델 스폰 확인 (+ 재개)')
    rel.add_argument('--world', required=True)
    rel.add_argument('--models', required=True, help='쉼표 구분 모델 이름')
    rel.add_argument('--unpause', action='store_true')
    rel.add_argument('--timeout', type=float, default=120.0, help='[s] 스폰 확인 대기 상한')
    args = parser.parse_args(argv)
    gate = WorldGate(args.world)
    if args.cmd == 'hold':
        return gate.hold(args.pause, args.timeout)
    names = [n.strip() for n in args.models.split(',') if n.strip()]
    return gate.release(names, args.unpause, args.timeout)


if __name__ == '__main__':
    sys.exit(main())
