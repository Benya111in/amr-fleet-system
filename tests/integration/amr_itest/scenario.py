"""
시나리오 베이스: 메타데이터(명세 4.10 표의 지표·기준·로그 포맷), 백엔드 선택, 건너뛰기, 결과 기록.

시나리오 파일 하나(tests/integration/test_NN_<slug>.py) = launch_testing 런 하나.

    SCENARIO = Scenario(number=9, slug='emergency_stop', title=..., spec=..., metric=...,
                        threshold=..., log_format=..., backends=('kinematic', 'gazebo'))
    CTX = Context(SCENARIO)

    @pytest.mark.launch_test
    def generate_test_description():
        CTX.begin()                         # 로그 디렉토리 정리 + result.json
        stack = Stack(CTX, CTX.select_backend())   # 불가능하면 여기서 SkipTest
        ...
        return stack.launch_description(), {'ctx': CTX, 'stack': stack}

건너뛰기는 generate_test_description 안에서 unittest.SkipTest 를 던진다. launch_testing 은 이를
"skip decorator" 경로로 처리해 모든 테스트 케이스를 skipped(사유 포함)로 JUnit 에 남긴다.
(launch_testing 의 unittest 러너 안에서는 pytest.skip 의 Skipped 예외가 통하지 않는다.
pytest 로 직접 돌릴 때는 conftest.py 가 CTX.skip_reason 을 보고 SKIPPED 로 바꾼다.)
"""

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence, Tuple
import unittest

from amr_itest import requirements as req
from amr_itest.config import Settings
from amr_itest.results import ScenarioRecord

KINEMATIC = 'kinematic'
GAZEBO = 'gazebo'
COMPONENT = 'component'
SYSTEM = 'system'


def gazebo_requirements() -> Tuple[req.Requirement, ...]:
    """Gazebo 백엔드 요구사항 (amr_simulation 월드 + amr_description 스폰 + GPU)."""
    return (
        req.launch('amr_simulation', 'warehouse.launch.py', 'Gazebo 물류센터 월드'),
        req.launch('amr_description', 'description.launch.py', '로봇 URDF / robot_state_publisher'),
        req.launch('amr_description', 'spawn.launch.py', '로봇 스폰 + ros_gz 브리지'),
        req.package('ros_gz_sim'),
        req.gpu('Gazebo 헤드리스 렌더링 센서 (docker run --gpus all)'),
    )


@dataclass(frozen=True)
class Scenario:
    """시나리오 메타데이터 (result.json · README 표 · --list 출력의 출처)."""

    number: int
    slug: str
    title: str
    spec: str                   # 명세 조항
    metric: str                 # 측정 지표 (산출 방법)
    threshold: str              # 합격 기준
    log_format: str             # 산출 로그 (파일 + 열)
    backends: Tuple[str, ...] = (KINEMATIC,)   # 지원 백엔드, auto 는 앞에서부터 가능한 것
    profiles: Tuple[str, ...] = (COMPONENT,)   # 지원 스택 구성, auto 는 첫 번째
    implemented: bool = True    # False = 스켈레톤 (필요 노드가 머지되면 켜짐)
    long_running: bool = False  # True = 기본 실행에서 제외 (명시적으로 골라야 실행)

    @property
    def id(self) -> str:
        return f'{self.number:02d}_{self.slug}'

    def meta(self) -> dict:
        d = asdict(self)
        d['id'] = self.id
        d['backends'] = list(self.backends)
        d['profiles'] = list(self.profiles)
        return d


class Context:
    """시나리오 한 번의 실행 문맥: 설정, 결과 기록, 백엔드, 건너뛰기 사유."""

    def __init__(self, scenario: Scenario, settings: Optional[Settings] = None):
        self.scenario = scenario
        self.settings = settings or Settings.from_env()
        self.log_dir: Path = self.settings.log_root / scenario.id
        self.record = ScenarioRecord(scenario.meta(), self.log_dir, {
            'robot': self.settings.robot, 'frame_prefix': self.settings.frame_prefix,
            'sim': self.settings.sim, 'profile': self.settings.profile,
            'standins': self.settings.standins,
            'timeout_scale': self.settings.timeout_scale, 'world': self.settings.world,
            'seed': self.settings.seed})
        self.backend: Optional[str] = None
        self.profile: Optional[str] = None
        self.skip_reason: Optional[str] = None

    # --- 수명 ---
    def begin(self) -> 'Context':
        self.record.begin(clean=True)
        return self

    def finish(self) -> None:
        self.record.finish()

    # --- 건너뛰기 ---
    def skip(self, reason: str) -> None:
        """사유를 기록하고 unittest.SkipTest 를 던진다 (launch 는 뜨지 않는다)."""
        self.skip_reason = reason
        self.record.skipped(reason)
        raise unittest.SkipTest(reason)

    def require(self, reqs: Iterable[req.Requirement], context: str = '') -> None:
        """요구사항이 하나라도 없으면 모든 사유를 모아 건너뛴다."""
        missing = req.missing(reqs)
        if missing:
            head = f'{context}: ' if context else ''
            self.skip(head + '; '.join(missing))

    # --- 백엔드 ---
    def backend_missing(self, backend: str) -> Sequence[str]:
        if backend == GAZEBO:
            return req.missing(gazebo_requirements())
        return []

    def select_backend(self) -> str:
        """
        ITEST_SIM 과 시나리오 지원 목록으로 백엔드를 고른다.

        auto: backends 를 앞에서부터 보며 요구사항이 충족되는 첫 번째.
        명시(kinematic/gazebo): 지원하지 않거나 요구사항이 없으면 건너뛴다.
        """
        want = self.settings.sim
        if want != 'auto':
            if want not in self.scenario.backends:
                self.skip(f'backend {want!r} not supported by scenario {self.scenario.id} '
                          f'(supported: {", ".join(self.scenario.backends)})')
            miss = self.backend_missing(want)
            if miss:
                self.skip(f'backend {want}: ' + '; '.join(miss))
            self.backend = want
        else:
            reasons = []
            for cand in self.scenario.backends:
                miss = self.backend_missing(cand)
                if not miss:
                    self.backend = cand
                    break
                reasons.append(f'{cand}: ' + '; '.join(miss))
            if self.backend is None:
                self.skip('no usable backend — ' + ' | '.join(reasons))
        self.record.set_backend(self.backend)
        return self.backend

    def select_profile(self) -> str:
        """ITEST_PROFILE 과 시나리오 지원 목록으로 스택 구성을 고른다 (auto: 첫 번째)."""
        want = self.settings.profile
        if want == 'auto':
            want = self.scenario.profiles[0]
        elif want not in self.scenario.profiles:
            self.skip(f'profile {want!r} not supported by scenario {self.scenario.id} '
                      f'(supported: {", ".join(self.scenario.profiles)})')
        self.profile = want
        self.record.measure('profile', want)
        return want

    # --- 편의 ---
    def timeout(self, seconds: float) -> float:
        return self.settings.timeout(seconds)

    def path(self, name: str) -> Path:
        return self.log_dir / name

    @property
    def use_sim_time(self) -> bool:
        return self.backend == GAZEBO
