"""
E2 캠페인 드라이버 (호스트, 표준 라이브러리만) — 세션을 이어 붙여 시행 목록을 끝까지 돈다.

    python3 layopt/loc/e2_campaign.py --trials layopt/loc/exp1_trials.json --tag exp1
    python3 layopt/loc/e2_campaign.py --trials ... --run-dir <runs>/e2/<run_id> --resume

세션 하나 = `gz_slot.sh layopt-loc docker run --rm amr-ai/rl:0.3 … e2_container.sh` (Gazebo + 스택 + 러너,
per_session 시행 뒤 또는 이상 시 끝남). 세션마다 Gazebo 를 새로 띄우므로 이상 상태가 다음 세션으로 넘어가지 않는다.
재개: 러너가 attempts.jsonl 을 보고 done 시행을 건너뛴다. 실행 디렉터리는 새로 만들 때만 생성(이미 있으면 거부),
--resume 이면 있는 디렉터리를 이어 쓴다. 시행 목록 사본(trials.json)·HEAD·git status 를 남긴다.
"""

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

WS = Path(__file__).resolve().parents[2]
RUNS = Path('/home/CAPYI_AI/runs/layopt')
GZ_SLOT = '/home/CAPYI_AI/env/gz_slot.sh'
IMAGE = 'amr-ai/rl:0.3'
EXIT_ANOMALY = 10


def utc_id() -> str:
    """run_id 시각 부분."""
    return dt.datetime.now(dt.timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def cpath(p) -> str:
    """호스트 경로 → 컨테이너 경로 (/ros2_ws, /runs). 이미 컨테이너 경로면 그대로."""
    if str(p).startswith(('/ros2_ws/', '/runs/')):
        return str(p)
    p = str(Path(p).resolve())
    if p.startswith(str(WS)):
        return '/ros2_ws' + p[len(str(WS)):]
    if p.startswith(str(RUNS)):
        return '/runs' + p[len(str(RUNS)):]
    raise ValueError(f'컨테이너에 마운트되지 않는 경로: {p}')


def log(run_dir: Path, msg: str) -> None:
    """campaign.log 에 한 줄 (부하 포함)."""
    line = (f'{dt.datetime.now().isoformat(timespec="seconds")} '
            f'load={" ".join(f"{v:.2f}" for v in os.getloadavg())} {msg}')
    print(line, flush=True)
    with (run_dir / 'campaign.log').open('a', encoding='utf-8') as f:
        f.write(line + '\n')


def pending(run_dir: Path, spec: dict, max_attempts: int, only: str = '', lane: int = 0,
            lanes: int = 1):
    """러너와 같은 규칙의 남은 시행 (order 순)."""
    hist = {}
    att = run_dir / 'attempts.jsonl'
    if att.exists():
        for line in att.read_text(encoding='utf-8').splitlines():
            if line.strip():
                r = json.loads(line)
                hist.setdefault(r['trial_id'], []).append(r)
    out = []
    for tr in sorted(spec['trials'], key=lambda r: r['order']):
        h = hist.get(tr['trial_id'], [])
        if any(r['status'] == 'done' for r in h):
            continue
        if sum(1 for r in h if r['status'] == 'invalid') >= max_attempts:
            continue
        out.append(tr)
    if only:
        want = set(only.split(','))
        out = [t for t in out if t['trial_id'] in want]
    if lanes > 1:
        out = [t for t in out if t['order'] % lanes == lane]
    return out


def main(argv=None) -> int:
    """캠페인."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', required=True)
    ap.add_argument('--tag', default='')
    ap.add_argument('--run-dir', default='')
    ap.add_argument('--resume', action='store_true')
    ap.add_argument('--per-session', type=int, default=20)
    ap.add_argument('--max-sessions', type=int, default=40)
    ap.add_argument('--max-attempts', type=int, default=2)
    ap.add_argument('--max-consecutive-anomalies', type=int, default=5)
    ap.add_argument('--anomaly-pause-s', type=float, default=120.0)
    ap.add_argument('--dummy-mode', default='move')
    ap.add_argument('--only', default='')
    ap.add_argument('--session-timeout-per-trial', type=float, default=480.0)
    # 병렬 세션 (조정자 05:00 허용): 레인마다 드라이버 하나, 시행은 order % lanes 로 나눈다.
    # 레인 0 = 도메인 57 / amr_ai_layopt_e2 / 세션 s###, 레인 1 = 58 / amr_ai_layopt_e2b / b###
    ap.add_argument('--lane', type=int, default=0)
    ap.add_argument('--lanes', type=int, default=1)
    ap.add_argument('--module', default='layopt.loc.e2_runner',
                    help='컨테이너 안 러너 (탐침: layopt.loc.probe_stationary, --max-sessions 1)')
    a = ap.parse_args(argv)

    trials_path = Path(a.trials).resolve()
    spec = json.loads(trials_path.read_text(encoding='utf-8'))
    if a.run_dir:
        run_dir = Path(a.run_dir)
        if not run_dir.exists() or not a.resume:
            print('--run-dir 는 --resume 과 함께 기존 디렉터리에만', file=sys.stderr)
            return 2
    else:
        run_dir = RUNS / 'e2' / f'{utc_id()}_{a.tag or "e2"}'
        if run_dir.exists():
            print(f'이미 있다: {run_dir}', file=sys.stderr)
            return 2
        run_dir.mkdir(parents=True)
        shutil.copyfile(trials_path, run_dir / 'trials.json')
        (run_dir / 'HEAD.txt').write_text(subprocess.run(
            ['git', '-C', str(WS), 'rev-parse', 'HEAD'], capture_output=True,
            text=True).stdout, encoding='utf-8')
        (run_dir / 'git_status.txt').write_text(subprocess.run(
            ['git', '-C', str(WS), 'status', '--porcelain'], capture_output=True,
            text=True).stdout, encoding='utf-8')
        (run_dir / 'trials_source.txt').write_text(str(trials_path) + '\n', encoding='utf-8')
    frozen = run_dir / 'trials.json'
    if json.loads(frozen.read_text(encoding='utf-8'))['trials'] != spec['trials']:
        print('시행 목록이 실행 디렉터리 사본과 다르다 — 거부', file=sys.stderr)
        return 2
    log(run_dir, f'start run_dir={run_dir} trials={trials_path} n={len(spec["trials"])} '
                 f'per_session={a.per_session}')
    world, mapy = spec['world'], spec['map']
    prefix = 'sb'[a.lane] if a.lane < 2 else f'l{a.lane}'
    domain = {0: 57, 1: 58}[a.lane]
    partition = {0: 'amr_ai_layopt_e2', 1: 'amr_ai_layopt_e2b'}[a.lane]
    sessions_done = len(list((run_dir / 'sessions').glob(f'{prefix}[0-9]*'))) if (
        run_dir / 'sessions').exists() else 0
    consecutive = 0
    for s in range(sessions_done, sessions_done + a.max_sessions):
        todo = pending(run_dir, spec, a.max_attempts, a.only, a.lane, a.lanes)
        if not todo:
            log(run_dir, 'all trials finished')
            break
        sess = f'{prefix}{s:03d}'
        first = todo[0]
        name = f'amr-ai-layopt-e2-{run_dir.name[-24:].replace("_", "-").lower()}-{sess}'
        envs = ['-e', f'E2_RUN_DIR={cpath(run_dir)}', '-e', f'E2_TRIALS={cpath(frozen)}',
                '-e', f'E2_SESSION={sess}', '-e', f'E2_WORLD={cpath(world)}',
                '-e', f'E2_MAP={cpath(mapy)}',
                '-e', f'E2_SPAWN={first["x"]},{first["y"]},{first["yaw"]}',
                '-e', f'E2_PER_SESSION={a.per_session}', '-e', f'E2_DUMMY_MODE={a.dummy_mode}',
                '-e', f'E2_ONLY={a.only}', '-e', f'E2_MAX_ATTEMPTS={a.max_attempts}',
                '-e', f'E2_DOMAIN={domain}', '-e', f'E2_PARTITION={partition}',
                '-e', f'E2_LANE={a.lane}', '-e', f'E2_LANES={a.lanes}',
                '-e', f'E2_MODULE={a.module}']
        cmd = [GZ_SLOT, 'layopt-loc', 'docker', 'run', '--rm', '--name', name, '--gpus', 'all',
               '--shm-size=8g', '-v', f'{WS}:/ros2_ws', '-v', f'{RUNS}:/runs'] + envs + [
               IMAGE, 'bash', '/ros2_ws/layopt/loc/e2_container.sh']
        log(run_dir, f'session {sess} start: pending {len(todo)}, first {first["trial_id"]}')
        t0 = time.time()
        timeout = 900.0 + a.session_timeout_per_trial * min(a.per_session, len(todo))
        try:
            rc = subprocess.run(cmd, timeout=timeout, check=False).returncode
        except subprocess.TimeoutExpired:
            log(run_dir, f'session {sess} timeout {timeout:.0f} s → docker stop {name}')
            subprocess.run(['docker', 'stop', '-t', '90', name], check=False,
                           capture_output=True)
            rc = -1
        log(run_dir, f'session {sess} rc={rc} wall={time.time() - t0:.0f} s')
        if rc == 0:
            consecutive = 0
        elif rc == EXIT_ANOMALY or rc == -1:
            consecutive += 1
            if consecutive >= a.max_consecutive_anomalies:
                log(run_dir, f'{consecutive} consecutive anomalies → stop')
                return 1
            time.sleep(a.anomaly_pause_s)      # 부하 급등 뒤 바로 다시 띄우지 않는다
        else:
            log(run_dir, f'unexpected rc {rc} → stop')
            return 1
    left = pending(run_dir, spec, a.max_attempts, a.only, a.lane, a.lanes)
    log(run_dir, f'end: pending {len(left)}')
    return 0 if not left else 1


if __name__ == '__main__':
    sys.exit(main())
