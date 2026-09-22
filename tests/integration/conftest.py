"""
pytest 설정 (tests/integration): 하네스 경로 + launch_testing 시나리오의 skip 보고.

기본 실행기는 scripts/run_integration.sh (amr_itest.launch_test_ros = launch_testing_ros 러너,
시나리오 파일마다 JUnit) 이다.
pytest 로 직접 돌릴 때도 같은 결과가 나오게 한다:
    pytest tests/integration/unit                       하네스 순수 로직 단위 테스트
    pytest tests/integration/test_09_emergency_stop.py  launch_testing pytest 플러그인 경유

launch_testing 의 pytest 플러그인은 generate_test_description 안의 unittest.SkipTest 를
"성공" 으로 보고한다 (SkipResult.wasSuccessful() 가 True). 시나리오 모듈의 CTX.skip_reason 이
있으면 그 항목을 SKIPPED(사유 포함)로 바꿔, 필요한 노드가 없어 건너뛴 시나리오가 통과로 잘못
세어지지 않게 한다.
"""

from pathlib import Path
import sys

import pytest

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    """launch_test 항목이 CTX.skip() 으로 끝났으면 SKIPPED 로 보고한다."""
    outcome = yield
    report = outcome.get_result()
    if call.when != 'call' or not report.passed:
        return
    module = sys.modules.get(getattr(item, 'name', ''))
    ctx = getattr(module, 'CTX', None)
    reason = getattr(ctx, 'skip_reason', None)
    if reason:
        report.outcome = 'skipped'
        report.longrepr = (str(getattr(item, 'path', item.fspath)), 0, f'Skipped: {reason}')
