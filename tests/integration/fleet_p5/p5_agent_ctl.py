#!/usr/bin/env python3
"""
P5 에이전트 재기동 도구 (스택 컨테이너 안에서 `docker exec` 로 — `p3.md` §8-7 · §10.6-9).

  python3 p5_agent_ctl.py kill <로봇> <저장 접두어>
      fleet_agent_node(네임스페이스 /<로봇>)를 SIGKILL 하고 그 명령줄을 <접두어>.args 에 남긴다 — 런치가 준
      파라미터 파일(/tmp/launch_params_*)까지 그대로 다시 쓰려고.
  python3 p5_agent_ctl.py restart <저장 접두어> <환경 파일>
      환경 파일(p5_container.sh 가 남긴 export -p)을 읽은 셸에서 남긴 명령줄을 그대로 다시 띄운다
      (출력은 <접두어>_restart.log). 같은 store_dir 이라 투표 · 펜스 · 작업 잠금이 이어진다.

런치는 에이전트를 respawn 하지 않는다 (behavior.launch.py) — 이 도구가 "같은 파라미터로 다시 띄우는" 운영자 역할을 한다.
다른 프로세스는 건드리지 않는다 (실행 파일 — argv[0], 또는 python 이면 argv[1] — 이 fleet_agent_node 이고 정확히
__ns:=/<로봇> 인 인자가 있는 것만. 명령줄 문자열에 그 낱말이 들어 있는 셸은 고르지 않는다).
명령줄 인자에 공백이 없다는 가정을 쓴다 (경로 · ROS 인자 — 남긴 .args 를 결과에 함께 둔다).
"""

import os
import shlex
import signal
import subprocess
import sys


def find_agent(robot: str):
    """(pid, argv) — ps 로 찾는다 (자기 자신의 argv 에는 __ns:=/<로봇> 원소가 없다)."""
    out = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True).stdout
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 2 or not parts[0].isdigit():
            continue
        pid, argv = int(parts[0]), parts[1:]
        if pid == os.getpid():
            continue
        exe = argv[0] if not os.path.basename(argv[0]).startswith('python') else (
            argv[1] if len(argv) > 1 else '')
        if exe.endswith('fleet_agent_node') and f'__ns:=/{robot}' in argv:
            return pid, argv
    return None, None


def cmd_kill(robot: str, prefix: str) -> int:
    pid, argv = find_agent(robot)
    if pid is None:
        print(f'{robot}: fleet_agent_node 없음', file=sys.stderr)
        return 3
    with open(prefix + '.args', 'w', encoding='utf-8') as f:
        f.write(shlex.join(argv) + '\n')
    os.kill(pid, signal.SIGKILL)
    print(f'{robot}: pid {pid} SIGKILL — {shlex.join(argv)[:300]}')
    return 0


def cmd_restart(prefix: str, env_file: str) -> int:
    with open(prefix + '.args', encoding='utf-8') as f:
        cmd = f.read().strip()
    script = (f'source {shlex.quote(env_file)} && exec {cmd} '
              f'>> {shlex.quote(prefix + "_restart.log")} 2>&1')
    os.execvp('bash', ['bash', '-c', script])
    return 0                     # 오지 않는다


def main(argv=None) -> int:
    a = list(sys.argv[1:] if argv is None else argv)
    if len(a) == 3 and a[0] == 'kill':
        return cmd_kill(a[1], a[2])
    if len(a) == 3 and a[0] == 'restart':
        return cmd_restart(a[1], a[2])
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main())
