"""
하네스 설정: 저장소 경로, config/*.yaml 로딩, 환경 변수 노브 (rclpy 비의존).

환경 변수 (scripts/run_integration.sh 옵션이 같은 이름으로 내보낸다)
  ITEST_ROBOT          로봇 이름 = 네임스페이스 (기본 amr_01)
  ITEST_FRAME_PREFIX   TF 프레임 접두어 (기본 '' — 단일 로봇 규약. 다중 로봇 규약은 'amr_01/')
  ITEST_SIM            auto | kinematic | gazebo — 시뮬레이터 백엔드 (auto: 시나리오 기본값)
  ITEST_PROFILE        auto | component | system — 스택 구성
                         component  하네스가 노드 단위로 조립 (실제 노드가 있으면 실제, 없으면 대역)
                         system     amr_bringup/system.launch.py 전체 (모든 하위 런치가 있어야 함)
                         auto       시나리오 catalog profiles 의 첫 번째 (04·13: system, 02·09: component)
  ITEST_STANDINS       auto | always | never — 대역 노드 정책 (component 프로필)
                         auto   실제 노드가 설치돼 있으면 실제, 없으면 대역
                         always 항상 대역 (하네스 자체 검증용)
                         never  대역 금지: 실제 노드가 없으면 시나리오를 건너뛴다 (최종 통합 CI)
  ITEST_LOG_DIR        결과 루트 (기본 <repo>/logs/itest)
  ITEST_TIMEOUT_SCALE  모든 대기 상한에 곱하는 배율 (호스트 부하가 크면 키운다, 기본 1.0)
  ITEST_WORLD          Gazebo 월드 파일 (기본 warehouse.sdf, amr_simulation/worlds 기준 또는 절대 경로)
  ITEST_SEED           대역 노드 난수 시드 (기본 7)
  ITEST_SCENARIO_TIMEOUT  러너가 이 시나리오에 준 wall 상한 [s] (run_integration.sh 가 시나리오마다 내보낸다,
                       없으면 무제한). 시행을 반복하는 시나리오는 남은 시간(ProbeCase.time_left)으로 새 시행을
                       시작할지 정한다 — 러너에 잘려 측정값 없이 끝나지 않게
"""

from dataclasses import dataclass
import functools
import math
import os
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

# tests/integration (이 패키지의 부모)
HARNESS_DIR = Path(__file__).resolve().parent.parent

SIM_MODES = ('auto', 'kinematic', 'gazebo')
PROFILES = ('auto', 'component', 'system')
STANDIN_POLICIES = ('auto', 'always', 'never')


def repo_root() -> Path:
    """
    저장소 루트.

    ITEST_REPO_ROOT → ROS_WS (컨테이너 /ros2_ws, config/ 가 있을 때) → tests/integration 의 두 단계 위.
    """
    for var in ('ITEST_REPO_ROOT', 'ROS_WS'):
        val = os.environ.get(var, '').strip()
        if val and (Path(val) / 'config').is_dir():
            return Path(val)
    return HARNESS_DIR.parent.parent


def config_dir() -> Path:
    """최상위 config/ 디렉토리 (robot_params.yaml, sensors.yaml, ekf.yaml)."""
    return repo_root() / 'config'


def load_ros_params(path: Path, node_key: Optional[str] = None) -> Dict[str, Any]:
    """
    ROS 파라미터 YAML 의 ros__parameters 블록을 dict 로 읽는다.

    node_key 가 없으면 '/**' 를, 그것도 없으면 첫 키를 쓴다.
    """
    with open(path, encoding='utf-8') as fh:
        doc = yaml.safe_load(fh) or {}
    if node_key is None:
        node_key = '/**' if '/**' in doc else next(iter(doc), None)
    if node_key is None or node_key not in doc:
        raise KeyError(f'{path}: 키 {node_key!r} 없음')
    return dict(doc[node_key].get('ros__parameters', {}))


@functools.lru_cache(maxsize=None)
def robot_params() -> Dict[str, Any]:
    """config/robot_params.yaml 의 ros__parameters."""
    return load_ros_params(config_dir() / 'robot_params.yaml')


@functools.lru_cache(maxsize=None)
def sensors() -> Dict[str, Any]:
    """config/sensors.yaml 의 ros__parameters."""
    return load_ros_params(config_dir() / 'sensors.yaml')


@functools.lru_cache(maxsize=None)
def ekf_params(node: str = 'ekf_filter_node_map') -> Dict[str, Any]:
    """config/ekf.yaml 의 노드별 ros__parameters (키 '/**/<node>')."""
    return load_ros_params(config_dir() / 'ekf.yaml', f'/**/{node}')


_REQUIRED = object()


def get(d: Dict[str, Any], dotted: str, default: Any = _REQUIRED) -> Any:
    """
    중첩 dict 를 'a.b.c' 경로로 읽는다.

    default 를 주지 않으면 필수 키: 없으면 KeyError (설정이 바뀌었는데 하네스가 옛 기본값으로 조용히 재는
    일이 없게 — 예: LiDAR 를 차체 안 0.20 m 로 옮긴 뒤에도 0.38 m 평면으로 판정하던 기본값).
    """
    cur: Any = d
    for key in dotted.split('.'):
        if not isinstance(cur, dict) or key not in cur:
            if default is _REQUIRED:
                raise KeyError(f'설정 키 {dotted!r} 없음')
            return default
        cur = cur[key]
    return cur


def scan_plane_height() -> float:
    """스캔 평면(LiDAR)의 지면 높이 [m] = robot.base_link_height + lidar.extrinsic.z (필수 키)."""
    return (float(get(robot_params(), 'robot.base_link_height'))
            + float(get(sensors(), 'lidar.extrinsic.z')))


def _env(name: str, default: str) -> str:
    return os.environ.get(name, default).strip() or default


@dataclass(frozen=True)
class Settings:
    """실행 설정 (환경 변수에서 만든다, 시나리오 모듈이 import 시점에 읽는다)."""

    robot: str = 'amr_01'
    frame_prefix: str = ''
    sim: str = 'auto'
    profile: str = 'auto'
    standins: str = 'auto'
    log_root: Path = Path('logs/itest')
    timeout_scale: float = 1.0
    world: str = 'warehouse.sdf'
    seed: int = 7
    scenario_timeout: float = math.inf

    @classmethod
    def from_env(cls) -> 'Settings':
        """ITEST_* 환경 변수로 설정을 만든다 (잘못된 값은 ValueError)."""
        sim = _env('ITEST_SIM', 'auto')
        if sim not in SIM_MODES:
            raise ValueError(f'ITEST_SIM={sim!r}: {SIM_MODES} 중 하나')
        profile = _env('ITEST_PROFILE', 'auto')
        if profile not in PROFILES:
            raise ValueError(f'ITEST_PROFILE={profile!r}: {PROFILES} 중 하나')
        standins = _env('ITEST_STANDINS', 'auto')
        if standins not in STANDIN_POLICIES:
            raise ValueError(f'ITEST_STANDINS={standins!r}: {STANDIN_POLICIES} 중 하나')
        scale = float(_env('ITEST_TIMEOUT_SCALE', '1.0'))
        if not math.isfinite(scale) or scale <= 0.0:
            raise ValueError(f'ITEST_TIMEOUT_SCALE={scale}: 양수여야 한다')
        log_root = os.environ.get('ITEST_LOG_DIR', '').strip()
        return cls(
            robot=_env('ITEST_ROBOT', 'amr_01').strip('/'),
            frame_prefix=os.environ.get('ITEST_FRAME_PREFIX', '').strip(),
            sim=sim,
            profile=profile,
            standins=standins,
            log_root=Path(log_root) if log_root else repo_root() / 'logs' / 'itest',
            timeout_scale=scale,
            world=_env('ITEST_WORLD', 'warehouse.sdf'),
            seed=int(_env('ITEST_SEED', '7')),
            scenario_timeout=float(_env('ITEST_SCENARIO_TIMEOUT', 'inf')),
        )

    @property
    def namespace(self) -> str:
        """로봇 네임스페이스 (앞 '/' 포함)."""
        return '/' + self.robot

    def frame(self, name: str) -> str:
        """프레임 이름에 접두어를 붙인다 ('map' 은 전역이라 붙이지 않는다)."""
        if name == 'map' or not self.frame_prefix:
            return name
        return self.frame_prefix + name

    def timeout(self, seconds: float) -> float:
        """대기 상한 [s] × ITEST_TIMEOUT_SCALE."""
        return seconds * self.timeout_scale
