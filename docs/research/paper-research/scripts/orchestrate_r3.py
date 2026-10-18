#!/usr/bin/env python3
"""
3 라운드 실행 조율 (prereg_r3.md). 2 라운드 orchestrate_r2.py 를 그대로 쓰고 경로 집합 held-out D 와
3 라운드 단계·팔만 더한다. 원자료 /home/CAPYI_AI/paper_research/runs/r3/<단계>/ (컨테이너 /runs/r3/<단계>/).

    python3 orchestrate_r3.py paths --commit <sha> --dirty 0 --ws /home/CAPYI_AI/wt/dwa-abl   # held-out D + O 20

경로 집합 (prereg_r3 §4): held-out D /runs/r3/paths_d, seed 20261021, 매끈 32 · 통로 8 (시드 의존) · A* 20, O 20.
튜닝은 2 라운드 튜닝-20 (/runs/r2/paths_tune) 을 다시 쓴다.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import orchestrate_r2 as o2  # noqa: E402

o2.SETS['d'] = {'dir': '/runs/r3/paths_d', 'seed': 20261021, 'n': (32, 8, 20), 'o': 20}
o2.PATHS_LOGS = 'r3/paths_logs'


def main():
    if len(sys.argv) > 1 and sys.argv[1] == 'paths':
        # orchestrate_r2.paths_stage 를 held-out D 로 (로그 디렉터리는 r2/paths_logs 대신 r3/paths_logs)
        sys.argv += ['--sets', 'd']
        os.makedirs(os.path.join(o2.RUNS_HOST, 'r3', 'paths_logs'), exist_ok=True)
        return o2.main()
    raise SystemExit('알 수 없는 명령')


if __name__ == '__main__':
    sys.exit(main())
