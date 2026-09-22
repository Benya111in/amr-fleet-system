#!/usr/bin/env bash
# 통합 테스트 시나리오 실행 (명세 4.10: 통합 테스트 시나리오 최소 10개 작성·자동화)
#
#   docker compose exec dev ./scripts/run_integration.sh              기본 세트 (장시간 14 제외)
#   docker compose exec dev ./scripts/run_integration.sh 4 9 13       번호 / id / slug 로 선택
#   docker compose exec dev ./scripts/run_integration.sh --list       시나리오 목록
#
# 먼저 ./scripts/build.sh 로 빌드한다 (install/setup.bash 필요). 시나리오 정의·결과 형식은
# tests/README.md, tests/integration/README.md 참고.
#
# 옵션
#   --sim auto|kinematic|gazebo      시뮬레이터 백엔드 (기본 auto: 시나리오 기본값 → ITEST_SIM)
#   --profile auto|component|system  스택 구성 (ITEST_PROFILE)
#   --standins auto|always|never     대역 노드 정책 (ITEST_STANDINS; never = 실제 노드만, 최종 CI)
#   --robot NAME                     로봇 네임스페이스 (기본 amr_01)
#   --frame-prefix P                 TF 프레임 접두어 (다중 로봇 규약이면 amr_01/)
#   --timeout-scale X                모든 대기 상한 배율 (고부하 호스트에서 2~4)
#   --scenario-timeout SEC           시나리오 하나의 wall 상한 (기본 1800, 장시간은 5 h)
#   --log-dir DIR                    결과 루트 (기본 logs/itest)
#   --domain-id N                    ROS_DOMAIN_ID (기본: 이 실행 전용 100~199 — dev 스택과 섞이지 않게,
#                                    'env' 면 현재 값 유지)
#   --include-long                   기본 세트에 장시간 시나리오(14) 포함
#   --fail-on-skip                   skip 도 실패로 셈 (모든 패키지가 머지된 최종 CI)
#   --no-unit                        하네스 단위 테스트(tests/integration/unit) 생략
#   --unit-only                      하네스 단위 테스트만
#   --coverage                       하네스 단위 테스트 커버리지 (pytest-cov → logs/itest/unit/)
#   -v, --verbose                    러너 출력을 화면에도 (항상 <id>/launch.log 에 남는다)
#
# 결과  logs/itest/<id>/{result.json, junit.xml, launch.log, *.csv}
#       logs/itest/junit.xml (전체 합본), summary.json, summary.md
# 종료 코드  0 전부 통과 또는 skip / 1 실패·에러·시간 초과 / 2 사용법·환경 오류 / 130 중단
#
# Gazebo 시나리오(01·02 의 gazebo 백엔드, 03·05~08·10~12·14)는 헤드리스 렌더링 센서에 GPU 가 필요하다.
# dev 컨테이너(docker-compose.yml 의 nvidia 장치 예약)나 `docker run --gpus all` 에서 돌린다.
# GPU 가 없으면 해당 시나리오는 사유와 함께 skip 되고, 04·09·13 은 운동학 대역으로 돈다.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROS_WS:-$(dirname "${SCRIPT_DIR}")}"
cd "${WS}"

usage() { sed -n '2,38p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

SELECT=()
LOG_DIR="${WS}/logs/itest"
SCENARIO_TIMEOUT=""
DOMAIN_ID="auto"
INCLUDE_LONG=0
FAIL_ON_SKIP=0
RUN_UNIT=1
UNIT_ONLY=0
COVERAGE=0
VERBOSE=0
LIST=0
while [ $# -gt 0 ]; do
    case "$1" in
        --sim) export ITEST_SIM="$2"; shift 2 ;;
        --profile) export ITEST_PROFILE="$2"; shift 2 ;;
        --standins) export ITEST_STANDINS="$2"; shift 2 ;;
        --robot) export ITEST_ROBOT="$2"; shift 2 ;;
        --frame-prefix) export ITEST_FRAME_PREFIX="$2"; shift 2 ;;
        --timeout-scale) export ITEST_TIMEOUT_SCALE="$2"; shift 2 ;;
        --scenario-timeout) SCENARIO_TIMEOUT="$2"; shift 2 ;;
        --log-dir) LOG_DIR="$(realpath -m "$2")"; shift 2 ;;
        --domain-id) DOMAIN_ID="$2"; shift 2 ;;
        --include-long) INCLUDE_LONG=1; shift ;;
        --fail-on-skip) FAIL_ON_SKIP=1; shift ;;
        --no-unit) RUN_UNIT=0; shift ;;
        --unit-only) UNIT_ONLY=1; shift ;;
        --coverage) COVERAGE=1; shift ;;
        -v|--verbose) VERBOSE=1; shift ;;
        --list) LIST=1; shift ;;
        -h|--help) usage; exit 0 ;;
        -*) echo "알 수 없는 옵션: $1" >&2; usage >&2; exit 2 ;;
        *) SELECT+=("$1"); shift ;;
    esac
done

# ROS setup.bash 는 미정의 변수를 참조하므로 소싱 동안만 -u 를 끈다.
set +u
source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash
if [ -f "${WS}/install/setup.bash" ]; then
    source "${WS}/install/setup.bash"
else
    echo "install/setup.bash 가 없다. 먼저 ./scripts/build.sh 를 실행한다." >&2
    exit 2
fi
set -u

HARNESS="${WS}/tests/integration"
export PYTHONPATH="${HARNESS}${PYTHONPATH:+:${PYTHONPATH}}"
export ITEST_LOG_DIR="${LOG_DIR}"
export ITEST_REPO_ROOT="${WS}"
report() { python3 -m amr_itest.report --log-dir "${LOG_DIR}" "$@"; }

if [ "${LIST}" = "1" ]; then
    report --list
    exit 0
fi

# 이 실행 전용 DDS 도메인·Gazebo 파티션 (같은 컨테이너의 dev 스택/다른 실행과 섞이지 않게)
if [ "${DOMAIN_ID}" = "auto" ]; then
    export ROS_DOMAIN_ID=$(( 100 + $$ % 100 ))
elif [ "${DOMAIN_ID}" != "env" ]; then
    export ROS_DOMAIN_ID="${DOMAIN_ID}"
fi
export IGN_PARTITION="${ITEST_IGN_PARTITION:-itest_$$}"

# 실행할 시나리오 id
IDS=()
if [ "${UNIT_ONLY}" = "0" ]; then
    if [ ${#SELECT[@]} -gt 0 ]; then
        mapfile -t IDS < <(report --resolve "${SELECT[@]}") || exit 2
        [ ${#IDS[@]} -eq ${#SELECT[@]} ] || exit 2
    else
        mapfile -t IDS < <(report --default-set)
        if [ "${INCLUDE_LONG}" = "1" ]; then
            mapfile -t IDS < <(report --resolve $(report --default-set) 14)
        fi
    fi
fi

mkdir -p "${LOG_DIR}"
T_START=$(date +%s)
echo "=== 통합 테스트: ${#IDS[@]} 시나리오 (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}, IGN_PARTITION=${IGN_PARTITION})"
echo "    로그: ${LOG_DIR}   부하: $(cut -d' ' -f1-3 /proc/loadavg) / $(nproc) CPU"

UNIT_RC=0
if [ "${RUN_UNIT}" = "0" ]; then
    rm -rf "${LOG_DIR}/unit"      # 지난 실행의 단위 테스트 결과가 이번 집계에 섞이지 않게
else
    echo "--- 하네스 단위 테스트 (tests/integration/unit)"
    mkdir -p "${LOG_DIR}/unit"
    COV_ARGS=()
    if [ "${COVERAGE}" = "1" ]; then
        COV_ARGS=(--cov=amr_itest --cov-report=term-missing
                  "--cov-report=html:${LOG_DIR}/unit/htmlcov")
    fi
    # rootdir 를 unit/ 으로 고정한다: 상위(tests/integration)를 수집하면 launch_testing pytest
    # 플러그인이 시나리오 파일(test_NN_*.py)까지 import 한다.
    (cd "${HARNESS}/unit" && python3 -m pytest . --rootdir . -q -p no:cacheprovider \
        --junitxml="${LOG_DIR}/unit/junit.xml" "${COV_ARGS[@]}") || UNIT_RC=$?
fi

# 시나리오 러너: launch_testing_ros.LaunchTestRunner (ros2test 의 `ros2 test` 와 같은 경로).
# Humble 이미지에는 ros2test 가 없어 amr_itest.launch_test_ros 진입점을 쓴다 (인자는 launch_test 와 같다).
RUNNER=(python3 -m amr_itest.launch_test_ros)

CURRENT_PID=""
cleanup_session() {
    # 러너를 새 세션(setsid)으로 띄웠으므로 세션 전체(노드·Gazebo)를 정리한다
    local sid="$1"
    [ -n "${sid}" ] || return 0
    pkill -INT -s "${sid}" 2>/dev/null || true
    sleep 3
    pkill -KILL -s "${sid}" 2>/dev/null || true
}
on_interrupt() {
    echo "중단됨 — 실행 중인 시나리오를 정리한다" >&2
    cleanup_session "${CURRENT_PID}"
    report --scenarios "${IDS[@]}" >/dev/null 2>&1 || true
    exit 130
}
trap on_interrupt INT TERM

for id in "${IDS[@]}"; do
    file="${HARNESS}/test_${id}.py"
    dir="${LOG_DIR}/${id}"
    mkdir -p "${dir}"
    rm -f "${dir}/junit.xml" "${dir}/launch.log"
    limit="${SCENARIO_TIMEOUT:-1800}"
    [ "${id}" = "14_soak" ] && [ -z "${SCENARIO_TIMEOUT}" ] && limit=18000
    t0=$(date +%s)
    if [ "${VERBOSE}" = "1" ]; then
        setsid timeout --signal=INT --kill-after=60 "${limit}" \
            "${RUNNER[@]}" "${file}" --junit-xml "${dir}/junit.xml" \
            > >(tee "${dir}/launch.log") 2>&1 &
    else
        setsid timeout --signal=INT --kill-after=60 "${limit}" \
            "${RUNNER[@]}" "${file}" --junit-xml "${dir}/junit.xml" > "${dir}/launch.log" 2>&1 &
    fi
    CURRENT_PID=$!
    rc=0
    wait "${CURRENT_PID}" || rc=$?
    cleanup_session "${CURRENT_PID}"
    CURRENT_PID=""
    dt=$(( $(date +%s) - t0 ))
    if [ "${rc}" = "124" ] || [ "${rc}" = "137" ]; then
        report --synthetic "${id}" error "timeout after ${limit} s (runner rc=${rc})"
    elif [ ! -s "${dir}/junit.xml" ]; then
        report --synthetic "${id}" error "runner exited rc=${rc} without JUnit (see launch.log)"
    fi
    status=$(python3 -c 'import sys; from amr_itest import junit; print(junit.status_of(sys.argv[1]))' \
        "${dir}/junit.xml")
    printf '    %-26s %-8s %5d s  (rc=%s)\n' "${id}" "${status}" "${dt}" "${rc}"
done

echo
echo "--- 집계"
REPORT_ARGS=()
[ "${FAIL_ON_SKIP}" = "1" ] && REPORT_ARGS+=(--fail-on-skip)
RC=0
if [ ${#IDS[@]} -gt 0 ]; then
    report --scenarios "${IDS[@]}" "${REPORT_ARGS[@]}" || RC=$?
else
    report --scenarios "${REPORT_ARGS[@]}" || RC=$?
fi
[ "${UNIT_RC}" = "0" ] || RC=1

echo "총 소요: $(( $(date +%s) - T_START )) s, 종료 시 부하: $(cut -d' ' -f1-3 /proc/loadavg)"
exit "${RC}"
