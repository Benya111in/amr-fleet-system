"""
로봇마다 fleet_agent 하나를 띄운다 (보너스 4 P3, docs/research/fleet-distributed/p3.md).

    ros2 launch amr_fleet fleet_agents.launch.py robot_ids:=amr_01,amr_02,amr_03,amr_04,amr_05

에이전트는 python3 -m amr_fleet.agent.node 로 뜬다 — amr_fleet 의 install(PROGRAMS) 는 공유 파일이라 이 과제에서
진입점을 더하지 않았다 (p3.md §7 요청). 보고서는 에이전트를 fleet_adapter 프로세스의 두 번째 노드로 넣기를 권한다
(participant · 포트 절약) — 그 결합도 같은 요청에 있다. 지금은 로봇마다 프로세스 하나다.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

BOOL_ARGS = ('use_sim_time', 'sim_execution', 'proxy_assign_task', 'simulate_latency')


def _bool(name: str, value: str) -> str:
    v = value.strip().lower()
    if v not in ('true', 'false'):
        raise ValueError(f'{name} 는 true/false: {value!r}')
    return v


def _agents(context, *args, **kwargs):
    get = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    robots = [r.strip() for r in get('robot_ids').split(',') if r.strip()]
    if not robots:
        raise ValueError('robot_ids 가 비었다')
    members = get('members') or ','.join(robots)
    flags = {n: _bool(n, get(n)) for n in BOOL_ARGS}
    seed = int(get('seed'))
    if seed < 0:
        raise ValueError(f'seed 는 0 이상: {seed}')
    actions = []
    for r in robots:
        cmd = [sys.executable, '-m', 'amr_fleet.agent.node', '--ros-args',
               '-r', f'__ns:=/{r}', '-r', '__node:=fleet_agent',
               '--params-file', get('params_file'),
               '-p', f'use_sim_time:={flags["use_sim_time"]}', '-p', f'members:={members}',
               '-p', f'sim_execution:={flags["sim_execution"]}',
               '-p', f'proxy_assign_task:={flags["proxy_assign_task"]}',
               '-p', f'simulate_latency:={flags["simulate_latency"]}', '-p', f'seed:={seed}']
        for name in ('store_dir', 'zones_file', 'comm_model_file', 'release_gate'):
            if get(name):
                cmd += ['-p', f'{name}:={get(name)}']
        actions.append(ExecuteProcess(cmd=cmd, name=f'fleet_agent_{r}', output='screen'))
    return actions


def generate_launch_description() -> LaunchDescription:
    """런치 설명."""
    share = get_package_share_directory('amr_fleet')
    return LaunchDescription([
        DeclareLaunchArgument('robot_ids', default_value='amr_01,amr_02,amr_03,amr_04,amr_05'),
        DeclareLaunchArgument('members', default_value='',
                              description="정적 멤버 ('' = robot_ids). 정족수는 그 과반"),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'fleet_agent.yaml')),
        DeclareLaunchArgument('zones_file',
                              default_value=os.path.join(share, 'config', 'traffic_zones.yaml')),
        DeclareLaunchArgument('release_gate', default_value=''),
        DeclareLaunchArgument('store_dir', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('sim_execution', default_value='false'),
        DeclareLaunchArgument('proxy_assign_task', default_value='false'),
        DeclareLaunchArgument('simulate_latency', default_value='false'),
        DeclareLaunchArgument('comm_model_file', default_value=''),
        DeclareLaunchArgument('seed', default_value='1'),
        OpaqueFunction(function=_agents),
    ])
