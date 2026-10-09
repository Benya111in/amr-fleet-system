"""
layopt/gen 공용: 저장소 경로, 월드 생성기 모듈 적재, 기하 상수.

표준 라이브러리만 쓴다 (호스트 python3 에서도 import 된다). numpy·yaml 이 필요한 모듈(raster, validate,
build_bundle)은 amr-ai/rl:0.3 컨테이너에서 돈다.
"""

import importlib.util
import math
import os
from pathlib import Path

REPO = Path(os.environ.get('LAYOPT_REPO', Path(__file__).resolve().parents[2]))
GEN_PATH = REPO / 'src' / 'amr_simulation' / 'worlds' / 'gen_warehouse_world.py'
MODELS_DIR = REPO / 'src' / 'amr_simulation' / 'models'
BEHAVIOR_SRC = REPO / 'src' / 'amr_behavior' / 'config' / 'behavior.yaml'
FLEET_SPAWN_SRC = REPO / 'src' / 'amr_bringup' / 'config' / 'fleet_spawn.yaml'
ZONES_SRC = REPO / 'src' / 'amr_fleet' / 'config' / 'traffic_zones.yaml'
TRAFFIC_SRC = REPO / 'src' / 'amr_fleet' / 'config' / 'traffic.yaml'
SLAM_MAP = REPO / 'maps' / 'warehouse.yaml'
BUNDLES = Path('/home/CAPYI_AI/runs/layopt/bundles')

# 로봇 (config/robot_params.yaml footprint 0.60 x 0.40, traffic.yaml robot_radius)
ROBOT_LENGTH, ROBOT_WIDTH = 0.60, 0.40
ROBOT_RADIUS = 0.36
# 충전소 기하 (gen_warehouse_world.py: 스테이션 중심 = 벽 안쪽 면에서 1.6 m, 판 = 중심 + 0.26 m)
STATION_WALL_OFF = 1.6
MARKER_OFF = 0.26                    # STATION_FRONT 0.25 + 판 두께 반 0.01
STAGING_FROM_PLATE = 1.50            # behavior.yaml 충전소 staging: 판에서 1.50 m (-18.14 → -16.64)
DOCK_STANDOFF = 0.65                 # behavior.yaml standoff (도킹 완료 시 판–base_link)


_GEN = None


def load_generator():
    """월드 생성기 모듈 (gen_warehouse_world.py) — 한 번만 적재한다."""
    global _GEN
    if _GEN is None:
        spec = importlib.util.spec_from_file_location('gen_warehouse_world_layopt', GEN_PATH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _GEN = mod
    return _GEN


def yaw_of(vx, vy):
    """방향 벡터 → yaw [rad]."""
    return math.atan2(vy, vx)


def wrap(a):
    """각도를 (-π, π] 로."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def r4(v):
    """좌표 반올림 (출력 형식): 소수 4 자리, -0.0 은 0.0."""
    v = round(float(v), 4)
    return 0.0 if v == 0 else v
