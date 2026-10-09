"""layopt.sim 시험 공용: 저장소 뿌리를 import 경로에 두고, 지금 배치 World 를 한 번만 만든다."""

import pathlib
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from layopt.sim.layout import load_current  # noqa: E402
from layopt.sim.navmap import core_available  # noqa: E402

needs_core = pytest.mark.skipif(not core_available(), reason='amr_core_py 미빌드 (native/build.sh)')


@pytest.fixture(scope='session')
def layout():
    return load_current()


@pytest.fixture(scope='session')
def world(layout):
    if not core_available():
        pytest.skip('amr_core_py 미빌드')
    from layopt.sim.sim import World
    return World(layout)
