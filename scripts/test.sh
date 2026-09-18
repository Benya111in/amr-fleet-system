#!/usr/bin/env bash
# 단위 테스트 + 커버리지 측정 (명세 10장: 주요 모듈 커버리지 70% 이상)
set -euo pipefail

source /opt/ros/"${ROS_DISTRO}"/setup.bash
cd "${ROS_WS}"

if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

echo "=== colcon test ==="
colcon test \
    --event-handlers console_direct+ \
    --return-code-on-test-failure \
    "$@" || TEST_FAILED=1

echo
echo "=== 테스트 결과 요약 ==="
colcon test-result --verbose || true

echo
echo "=== Python 커버리지 (pytest-cov) ==="
# 각 파이썬 패키지의 test 디렉토리를 대상으로 커버리지 측정
python3 -m pytest \
    src/*/test \
    --cov=src \
    --cov-report=term-missing \
    --cov-report=html:"${ROS_WS}/logs/htmlcov" \
    --cov-report=xml:"${ROS_WS}/logs/coverage.xml" \
    -q || true

echo
echo "커버리지 HTML 리포트: logs/htmlcov/index.html"
exit "${TEST_FAILED:-0}"
