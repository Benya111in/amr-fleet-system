"""
도크 표 (amr_behavior behavior.yaml docks.*) 와 월드 마커로 기대 도킹 자세 계산 (rclpy 비의존 순수 모듈).

  load_docks()    behavior.yaml → {dock_id: Dock(staging, standoff, marker_id, perceive_yaw)}
  docked_pose()   월드 마커 자세 + 판 두께 + standoff → 도킹 완료 시 base_link 기대 자세 (월드)
작업 자세는 도크 staging 이어야 한다: task_executor_node 는 staging 에서 물품을 인식(Perceive)하고 도킹하며,
도크와 맞지 않는 자세는 거절한다 (allow_undocked_tasks=false). map = 월드 이므로 staging 을 그대로 쓴다.
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Dict, Optional, Tuple

import yaml

Pose = Tuple[float, float, float]


@dataclass(frozen=True)
class Dock:
    """도크 하나 (behavior.yaml docks.<id>)."""

    dock_id: str
    staging: Pose               # [x, y, yaw] 마커를 바라보는 접근 자세 (map = 월드)
    standoff: float             # [m] 도킹 완료 시 판 면 ~ base_link
    marker_id: int = -1
    perceive_yaw: Optional[float] = None


def load_docks(path: Path) -> Dict[str, Dock]:
    """behavior.yaml 의 '/**' ros__parameters.docks 표."""
    doc = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
    params = doc.get('/**', {}).get('ros__parameters', {})
    table = params.get('docks', {})
    out = {}
    for dock_id in table.get('ids', []):
        d = table[dock_id]
        st = [float(v) for v in d['staging']]
        out[dock_id] = Dock(dock_id, (st[0], st[1], st[2]), float(d['standoff']),
                            int(d.get('marker_id', -1)),
                            float(d['perceive_yaw']) if 'perceive_yaw' in d else None)
    return out


def docked_pose(marker: Pose, standoff: float, plate_thickness: float = 0.02) -> Pose:
    """
    도킹 완료 시 base_link 의 기대 월드 자세.

    marker = 마커 모델 월드 자세 (x, y, yaw). 계약 C3: 마커 모델 +x = 판 바깥 법선. 판 면 = 중심 + 두께/2 ·
    법선, base_link = 판 면 + standoff · 법선, 방위 = 법선 반대 (판을 바라본다).
    """
    x, y, yaw = marker
    nx, ny = math.cos(yaw), math.sin(yaw)
    d = plate_thickness / 2.0 + standoff
    return x + d * nx, y + d * ny, math.atan2(math.sin(yaw + math.pi), math.cos(yaw + math.pi))


def pose_error(actual: Pose, expected: Pose) -> Tuple[float, float]:
    """(위치 오차 [m], 방위 오차 [rad] 절댓값)."""
    dyaw = math.atan2(math.sin(actual[2] - expected[2]), math.cos(actual[2] - expected[2]))
    return math.hypot(actual[0] - expected[0], actual[1] - expected[1]), abs(dyaw)
