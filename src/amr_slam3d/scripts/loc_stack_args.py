#!/usr/bin/env python3
"""
multi_robot.launch.py 가 로봇 하나의 localization.launch.py 에 주는 인자를 그대로 낸다 (b2-rerec 2 단계 기동).

    loc_stack_args.py <robots_file> [--mode odom]      → 'k:=v k:=v …' 한 줄 (ros2 launch 인자)

record_bag.sh 는 위치추정 스택을 뺀 채(with_localization:=false) 월드·로봇을 띄우고, 로봇이 정착한 뒤(settle_wait.py)
위치추정 스택을 따로 띄운다. 그 스택이 bringup 이 띄웠을 때와 같아야 하므로 인자를 손으로 적지 않고
amr_bringup.launch_utils 의 stack_arguments·stack_extra_arguments·frame_prefix 를 같은 값으로 부른다
(multi_robot.launch.py: 목록 앞 1 대, prefixed=True, use_sim_time true, localization_mode = --mode,
map_yaml '').
"""

import argparse
import shlex
import sys
from typing import Dict


def loc_arguments(robots_file: str, mode: str = 'odom') -> Dict[str, str]:
    """스폰 목록 파일 → bringup 이 첫 로봇의 localization 스택에 주는 인자 dict."""
    from amr_bringup import fleet_spawn
    from amr_bringup import launch_utils as lu
    spawn = fleet_spawn.load_fleet_spawn(robots_file)
    robot = fleet_spawn.select_robots(spawn, 1)[0]
    opts = lu.Options(use_sim_time=True, localization_mode=mode)
    return {**lu.stack_arguments(robot, lu.frame_prefix(robot, True), opts, 0),
            **lu.stack_extra_arguments('localization', opts)}


def main(argv=None) -> int:
    """명령줄."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('robots_file')
    ap.add_argument('--mode', default='odom')
    ns = ap.parse_args(argv)
    args = loc_arguments(ns.robots_file, ns.mode)
    print(' '.join(shlex.quote(f'{k}:={v}') for k, v in args.items()))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
