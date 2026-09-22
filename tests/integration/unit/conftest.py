"""하네스 단위 테스트 공통: amr_itest import 경로 + rclpy 컨텍스트 픽스처."""

from pathlib import Path
import sys

import pytest

HARNESS = Path(__file__).resolve().parents[1]
if str(HARNESS) not in sys.path:
    sys.path.insert(0, str(HARNESS))


@pytest.fixture
def ros():
    """테스트 하나 동안 rclpy 기본 컨텍스트를 연다 (대역 노드 메서드 직접 호출용)."""
    import rclpy
    rclpy.init()
    yield rclpy
    rclpy.try_shutdown()


@pytest.fixture
def itest_env(tmp_path, monkeypatch):
    """ITEST_* 환경을 초기화하고 로그 루트를 임시 디렉토리로."""
    for var in ('ITEST_SIM', 'ITEST_PROFILE', 'ITEST_STANDINS', 'ITEST_ROBOT',
                'ITEST_FRAME_PREFIX', 'ITEST_TIMEOUT_SCALE', 'ITEST_WORLD', 'ITEST_SEED'):
        monkeypatch.delenv(var, raising=False)
    monkeypatch.setenv('ITEST_LOG_DIR', str(tmp_path / 'itest'))
    return tmp_path / 'itest'
