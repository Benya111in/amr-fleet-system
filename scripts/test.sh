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
# 패키지를 한 번에 하나씩 (--executor sequential): 여러 패키지의 ROS 통신 시험이 같은 ROS_DOMAIN_ID 에서 동시에
# 돌면 서로의 /tf·토픽을 받아 실패한다 (실측: amr_simulation 의 Gazebo 적재 시험과 amr_perception safety_node
# 시험을 같은 도메인에서 동시에 돌리면 실패, 다른 도메인이면 통과). 패키지 안의 시험은 ctest 가 원래 순차다.
colcon test \
    --executor sequential \
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

# 출처를 남긴다 — 이게 없어서 logs/coverage 가 어느 커밋 것인지 못 따졌다 (전수 감사 지적).
# 통합 시나리오는 run_integration.sh 가 HEAD.txt·uncommitted.txt 를 남기는데 커버리지는 빠져 있었다.
_snap_provenance() {
    local out="$1"
    mkdir -p "$out"
    # git 워크트리를 컨테이너에 마운트하면 .git 이 마운트 **밖**(/home/<user>/.git/worktrees/...)을
    # 가리켜 컨테이너 안에서는 못 읽는다 (확인: "fatal: not a git repository").
    # 그래서 호스트가 AMR_HEAD 로 넘겨주면 그것을 쓰고, 없으면 컨테이너 안에서 시도하고,
    # 그마저 안 되면 **모른다고 적는다** — 빈 값이나 거짓 값을 남기지 않는다.
    if [ -n "${AMR_HEAD:-}" ]; then
        printf '%s\n' "${AMR_HEAD}" > "$out/HEAD.txt"
        printf '%s\n' "${AMR_UNCOMMITTED:-}" > "$out/uncommitted.txt"
    elif git rev-parse --short HEAD > "$out/HEAD.txt" 2>/dev/null; then
        git status --short > "$out/uncommitted.txt" 2>/dev/null || true
    else
        printf 'UNKNOWN (컨테이너에서 git 접근 불가 — 호스트에서 AMR_HEAD 로 넘겨라)\n' \
            > "$out/HEAD.txt"
        : > "$out/uncommitted.txt"
    fi
    date -Is > "$out/measured_at.txt"
}
_snap_provenance "${COVERAGE_DIR}"
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
