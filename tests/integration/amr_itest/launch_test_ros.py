"""
launch_test CLI 의 ROS 판: launch_testing_ros.LaunchTestRunner 로 시나리오 파일 하나를 돌린다.

ros2test 패키지의 `ros2 test` 와 같은 경로다 (launch_testing.launch_test.run 에 ROS 러너를 넘긴다).
Humble 이미지에는 ros2test 가 없어서 scripts/run_integration.sh 가 이 진입점을 쓴다.
인자는 launch_test 와 같다.

    python3 -m amr_itest.launch_test_ros tests/integration/test_09_emergency_stop.py \
        --junit-xml logs/itest/09_emergency_stop/junit.xml

종료 코드: 0 전부 통과(skip 포함) / 1 실패 / 2 사용법·로드 오류 (argparse, launch_test 와 같다).
pytest 로 돌릴 때는 launch_testing_ros 의 pytest 플러그인(entry point 'launch_ros')이 같은 러너를
쓴다.
"""

import argparse
import logging
import sys
from typing import List, Optional

from launch_testing import launch_test
from launch_testing_ros import LaunchTestRunner


def build_parser() -> argparse.ArgumentParser:
    """launch_test 와 같은 인자 (launch_test_file, --junit-xml, launch 인자 name:=value …)."""
    parser = argparse.ArgumentParser(
        prog='amr_itest.launch_test_ros',
        description='launch_testing_ros 러너로 통합 테스트 시나리오를 실행한다.')
    launch_test.add_arguments(parser)
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    """시나리오 파일 하나를 실행하고 종료 코드를 돌려준다."""
    logging.basicConfig()
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return launch_test.run(parser, args, test_runner_cls=LaunchTestRunner)
    except Exception as exc:     # noqa: B902 — launch_test.main 과 같이 사용법 오류(2)로 보고
        parser.print_usage(sys.stderr)
        print(f'{parser.prog}: error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
