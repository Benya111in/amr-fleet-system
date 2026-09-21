#!/usr/bin/env bash
# 단위 테스트 + 커버리지 측정 (명세 4.10: 주요 모듈 커버리지 70% 이상)
#
#   ./scripts/test.sh                               전체 패키지
#   ./scripts/test.sh --packages-select amr_fleet   일부만 (인자는 colcon test 로 그대로 전달)
#
# 커버리지는 패키지별 ament_add_pytest_test 가 pytest-cov 로 측정하고
# (package.xml 의 <test_depend>python3-pytest-cov</test_depend> 가 스위치),
# colcon coveragepy-result 가 패키지별 → 전체 순으로 합산한다.
set -euo pipefail

cd "${ROS_WS}"

# ROS setup.bash 는 미정의 변수(AMENT_TRACE_SETUP_FILES)를 참조하므로 소싱 동안만 -u 를 끈다.
set +u
source /opt/ros/"${ROS_DISTRO}"/setup.bash
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo "install/setup.bash 가 없다. 먼저 ./scripts/build.sh 를 실행한다." >&2
    exit 1
fi
set -u

TEST_FAILED=0

# 이전 실행의 결과 파일(build/*/test_results 등)을 먼저 지운다 — colcon test-result 에는 패키지 선택
# 옵션이 없어 build/ 전체를 읽으므로, 지우지 않으면 --packages-select 로 돌린 이번 실행의 요약과
# 종료 코드에 다른 패키지의 지난 실패가 섞인다. (.coverage 는 지우지 않는다)
colcon test-result --delete-yes >/dev/null 2>&1 || true

echo "=== colcon test ==="
colcon test \
    --event-handlers console_direct+ \
    --return-code-on-test-failure \
    "$@" || TEST_FAILED=1

echo
echo "=== 테스트 결과 요약 ==="
# 실패/에러가 있으면 상세를 출력하고 0 이 아닌 코드를 돌려준다.
colcon test-result --verbose || TEST_FAILED=1

echo
echo "=== Python 커버리지 (패키지별 → 전체) ==="
# build/<pkg>/pytest_cov/*/.coverage 를 패키지별로 합산해 모듈 단위로 출력하고,
# 전체 합산본과 HTML 은 logs/coverage/ 에 남긴다. 테스트 파일 자체는 집계에서 뺀다.
# (옵션 값 앞의 공백은 colcon 이 자기 옵션과 구분하기 위해 요구하는 표기)
COVERAGE_DIR="${ROS_WS}/logs/coverage"
mkdir -p "${COVERAGE_DIR}"
colcon coveragepy-result \
    --verbose \
    --coveragepy-base "${COVERAGE_DIR}" \
    --coverage-report-args ' --show-missing' ' --omit=*/test/*' \
    --coverage-html-args ' --omit=*/test/*' \
    || echo "커버리지 집계 실패 (측정된 테스트가 없거나 pytest-cov 미설치)" >&2

echo
echo "커버리지 HTML 리포트: logs/coverage/htmlcov/index.html"
exit "${TEST_FAILED}"
