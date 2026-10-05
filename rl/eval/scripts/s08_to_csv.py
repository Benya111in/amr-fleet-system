#!/usr/bin/env python3
"""
시나리오 08 실행 로그 → 결과 CSV (`gz_s08_actor42`) — r3 플러그인 실행을 DWA 기준선과 같은 열로 옮긴다.

사용 (한 줄):
  python3 rl/eval/scripts/s08_to_csv.py --log-root <logs> --arm C_hybrid
      --policy-id C_hybrid/s3/best --train-seed 3 --out c_s08.csv <태그1> <태그2> ...

태그는 실행 순서대로 준다. 각 태그 디렉터리에 HEAD.txt 가 없으면 commit 이 unknown 이 되고 판정표에 드러난다.
safety node 개입·저크는 지금 하네스가 시행별로 남기지 않아 빈 칸이다 (PROTOCOL.md §9).
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from amr_rl_eval import baseline as BL  # noqa: E402
from amr_rl_eval import schema  # noqa: E402


def main(argv=None):
    """명령행 진입점."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('tags', nargs='+')
    ap.add_argument('--log-root', required=True)
    ap.add_argument('--arm', required=True, choices=schema.ARMS)
    ap.add_argument('--policy-id', required=True)
    ap.add_argument('--train-seed', type=int, default=None)
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)
    rows, provs = BL.s08_runs_to_episodes(a.log_root, a.tags, a.arm, a.policy_id, a.train_seed)
    schema.write_csv(a.out, rows)
    schema.read_csv(a.out)
    for p in provs:
        print(f'{p["tag"]}: HEAD {p["head_status"]} {p["head"]}, 하네스 판정 '
              f'{"통과" if p["harness_passed"] else "실패 " + str(p["failed_checks"])}')
    print(a.out, len(rows), '행')
    return 0


if __name__ == '__main__':
    sys.exit(main())
