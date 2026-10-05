"""
3D LiDAR(보너스 2) 런치 보조 — 로봇별 장착 여부 결정, 설정 읽기, 브리지 항목 (launch 비의존 순수 모듈).

description.launch.py 와 spawn.launch.py 가 같은 규칙으로 "이 로봇에 3D LiDAR 가 있는가" 를 정한다.
두 런치가 서로 다르게 판단하면 URDF 에는 센서가 없는데 브리지만 생기거나 그 반대가 된다.

lidar3d 인자 값
    true / false   그대로 (CLI·상위 런치가 명시)
    auto (기본)    런치 범위에 robots_file 이 있으면 그 파일의 robots[] 중 robot_name 항목의 lidar3d 키
                   (없으면 false), robots_file 이 없으면 false.
                   multi_robot.launch.py·system.launch.py 는 robots_file 을 최상위 인자로 선언하므로 그 아래에서
                   포함된 description/spawn 런치가 같은 값을 본다 (amr_bringup launch_utils 를 고치지 않고 로봇별
                   값을 넘기는 경로). 키의 형식 검증은 amr_bringup fleet_spawn.py 가 같은 파일을 읽을 때 한다.
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

import yaml

AUTO = 'auto'
_TRUE = ('true', '1', 'yes')
_FALSE = ('false', '0', 'no')
CONFIG_NAME = 'lidar3d.yaml'
#: Fortress gpu_lidar 가 <topic> 옆에 점군을 내는 하위 이름 (ign-sensors 6 GpuLidarSensor)
GZ_POINTS_SUFFIX = '/points'


def parse_flag(text: str) -> Optional[bool]:
    """Lidar3d 인자 문자열 → True/False, auto 는 None. 그 밖은 ValueError."""
    value = str(text).strip().lower()
    if value in _TRUE:
        return True
    if value in _FALSE:
        return False
    if value in (AUTO, ''):
        return None
    raise ValueError(f'lidar3d 는 true | false | auto: {text!r}')


def flag_from_robots_file(path: str, robot_name: str) -> bool:
    """스폰 목록 YAML 의 robot_name 항목의 lidar3d (키가 없거나 항목이 없으면 False)."""
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    for item in data.get('robots') or []:
        if isinstance(item, dict) and str(item.get('name')) == robot_name:
            value = item.get('lidar3d', False)
            if not isinstance(value, bool):
                raise ValueError(f'{path}: {robot_name}.lidar3d 는 true/false: {value!r}')
            return value
    return False


def resolve(value: str, robot_name: str, robots_file: Optional[str]) -> bool:
    """인자 값과 (있으면) 스폰 목록에서 이 로봇의 3D LiDAR 장착 여부."""
    flag = parse_flag(value)
    if flag is not None:
        return flag
    if not robots_file:
        return False
    return flag_from_robots_file(robots_file, robot_name)


def load_config(path: str) -> Dict[str, Any]:
    """config/lidar3d.yaml → ros__parameters 사전 (lidar3d 블록 + imu_update_rate)."""
    with open(path, encoding='utf-8') as f:
        params = yaml.safe_load(f)['/**']['ros__parameters']
    if 'lidar3d' not in params:
        raise ValueError(f'{path}: lidar3d 블록이 없다')
    return params


def default_config_path(share_dir: str) -> str:
    """패키지 share 디렉터리 아래 기본 설정 경로."""
    return os.path.join(share_dir, 'config', CONFIG_NAME)


def bridge_entries(params: Dict[str, Any]) -> List[Dict[str, Any]]:
    """
    parameter_bridge 항목 (상대 이름 — 브리지 노드 네임스페이스 /<robot> 로 확장된다).

    점군만 넘긴다. 다층 LaserScan(gz <topic>)은 ros_gz_bridge 가 가운데 행만 넘기므로 넘기지 않는다.
    lazy: ROS 구독자가 없으면 브리지가 gz 구독을 열지 않고, 그러면 Sensors 시스템이 이 센서를 렌더하지 않는다
    (렌더 비용은 구독이 생길 때만 든다 — 보고서 "구독이 있을 때만 렌더한다").
    """
    l3 = params['lidar3d']
    return [{
        'ros_topic_name': l3['points_raw_topic'],
        'gz_topic_name': l3['topic'] + GZ_POINTS_SUFFIX,
        'ros_type_name': 'sensor_msgs/msg/PointCloud2',
        'gz_type_name': 'ignition.msgs.PointCloudPacked',
        'direction': 'GZ_TO_ROS',
        'lazy': True,
    }]
