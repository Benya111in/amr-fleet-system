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
#   --scenario-timeout SEC           시나리오 하나의 wall 상한 (기본 1800, 14 는 운용 시간 + 30 분)
#   --log-dir DIR                    결과 루트 (기본 logs/itest)
#   --domain-id N|auto|env           ROS_DOMAIN_ID (기본 auto: 이 실행 전용 1~101 난수 — dev 스택·다른 실행과
#                                    섞이지 않게. 'env' 면 현재 값 유지. 병렬 CI 는 N 을 명시)
#   --ign-partition NAME|auto|env    IGN_PARTITION (기본 auto: ITEST_IGN_PARTITION, 없으면 itest_<난수> — 바꿀 때
#                                    원래 값을 출력한다. 'env' 면 호출한 셸의 IGN_PARTITION 유지)
#   --include-long                   기본 세트에 장시간 시나리오(14) 포함
#   --soak-hours H                   14 의 운용 시간 (ITEST_SOAK_HOURS, 기본 4.0)
#   --fail-on-skip                   모든 skip 을 실패로 셈 (needs 패키지 설치 여부 무관)
#   --allow-skip                     skip 을 실패로 세지 않음 (기본: needs 패키지가 모두 설치된 시나리오의 skip·
#                                    일부 skip 은 실패 — GPU 없음 등 환경 때문에 못 돈 것도 통과가 아니다)
#   --no-unit                        하네스 단위 테스트(tests/integration/unit) 생략
#   --unit-only                      하네스 단위 테스트만
#   --coverage                       하네스 단위 테스트 커버리지 (pytest-cov → logs/itest/unit/)
#   -v, --verbose                    러너 출력을 화면에도 (항상 <id>/launch.log 에 남는다)
#
# 시행 수 (명세 캠페인 기본값, 스모크에서 줄일 때 — 줄이면 시행 수 판정이 실패한다)
#   ITEST_TRIALS (08, 30) · ITEST_RT_SAMPLES (13, 50) · ITEST_MR_TASKS (12, 5) · ITEST_SOAK_HOURS (14, 4.0)
#
# 결과  logs/itest/<id>/{result.json, junit.xml, launch.log, *.csv}
#       logs/itest/junit.xml (전체 합본), summary.json, summary.md
# 종료 코드  0 전부 통과 / 1 실패·에러·시간 초과·실패로 세는 skip / 2 사용법·환경 오류 / 130 중단
#
# Gazebo 시나리오(01, 03~08·10~14 의 system 구성, 02·09 의 gazebo 백엔드)는 헤드리스 렌더링 센서에 GPU 가
# 필요하다. dev 컨테이너(docker-compose.yml 의 nvidia 장치 예약)나 `docker run --gpus all --shm-size=8g` 에서
# 돌린다. GPU 없는 CI 는 운동학 구성만 고른다: --sim kinematic --profile component 2 4 9 13
# (04·13 의 component 구성은 AMCL·실행기 대역이라 명세 판정이 아니다 — result.json components 참고).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROS_WS:-$(dirname "${SCRIPT_DIR}")}"
cd "${WS}"

usage() { sed -n '2,48p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

SELECT=()
LOG_DIR="${WS}/logs/itest"
SCENARIO_TIMEOUT=""
DOMAIN_ID="auto"
PARTITION="auto"
INCLUDE_LONG=0
SKIP_POLICY=()
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
        --ign-partition) PARTITION="$2"; shift 2 ;;
        --include-long) INCLUDE_LONG=1; shift ;;
        --soak-hours) export ITEST_SOAK_HOURS="$2"; shift 2 ;;
        --fail-on-skip) SKIP_POLICY=(--fail-on-skip); shift ;;
        --allow-skip) SKIP_POLICY=(--allow-skip); shift ;;
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

# Fast DDS 프로파일: bringup 런치(amr_bringup launch_utils.dds_environment)와 같은 파일을 하네스 프로브·
# component 구성 노드에도 준다 — 참가자 포트 시도 한도(5대 ≈ 150 참가자), 영상 SHM 세그먼트, /dev/shm 가
# 작으면 SHM 확대 없는 변형(docker 기본 64 MB: --shm-size=8g 권장), ROS_LOCALHOST_ONLY=1 이면 127.0.0.1 변형.
# 선택 규칙은 launch_utils.dds_profile_name 하나를 쓴다 (이미 설정돼 있으면 그대로)
if [ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    DDS_PROFILE="$(python3 -c 'import os
from ament_index_python.packages import get_package_share_directory as share
from amr_bringup.launch_utils import dds_profile_name
print(os.path.join(share("amr_bringup"), "config", dds_profile_name()))' 2>/dev/null || true)"
    if [ -n "${DDS_PROFILE}" ] && [ -f "${DDS_PROFILE}" ]; then
        export FASTRTPS_DEFAULT_PROFILES_FILE="${DDS_PROFILE}"
    fi
fi

HARNESS="${WS}/tests/integration"
export PYTHONPATH="${HARNESS}${PYTHONPATH:+:${PYTHONPATH}}"
export ITEST_LOG_DIR="${LOG_DIR}"
export ITEST_REPO_ROOT="${WS}"
report() { python3 -m amr_itest.report --log-dir "${LOG_DIR}" "$@"; }

if [ "${LIST}" = "1" ]; then
    report --list
    exit 0
fi

# 이 실행 전용 DDS 도메인·Gazebo 파티션 (같은 컨테이너의 dev 스택/다른 실행과 섞이지 않게).
# 난수는 /dev/urandom — 새 컨테이너마다 거의 같은 $$ 로 만들면 병렬 실행이 같은 값을 받는다.
# 도메인은 1~101 (ROS 2 Linux 권장 상한 101: 그 위는 포트가 임시 포트 범위와 겹친다), 호출한 셸의 값은 피한다.
rand16() { od -An -N2 -tu2 /dev/urandom | tr -d ' '; }
if [ "${DOMAIN_ID}" = "auto" ]; then
    d=$(( 1 + $(rand16) % 101 ))
    [ "${d}" = "${ROS_DOMAIN_ID:-0}" ] && d=$(( d % 101 + 1 ))
    export ROS_DOMAIN_ID="${d}"
elif [ "${DOMAIN_ID}" != "env" ]; then
    case "${DOMAIN_ID}" in
        ''|*[!0-9]*) echo "--domain-id: 0~232 정수, auto, env 중 하나: ${DOMAIN_ID}" >&2; exit 2 ;;
    esac
    [ "${DOMAIN_ID}" -le 101 ] || echo "경고: ROS_DOMAIN_ID ${DOMAIN_ID} > 101 (Linux 임시 포트 범위와 겹칠 수 있다)" >&2
    export ROS_DOMAIN_ID="${DOMAIN_ID}"
fi
CALLER_PARTITION="${IGN_PARTITION:-}"
if [ "${PARTITION}" = "env" ]; then
    export IGN_PARTITION="${CALLER_PARTITION}"
elif [ "${PARTITION}" = "auto" ]; then
    export IGN_PARTITION="${ITEST_IGN_PARTITION:-itest_$(printf '%04x' "$(rand16)")}"
else
    export IGN_PARTITION="${PARTITION}"
fi
if [ -n "${CALLER_PARTITION}" ] && [ "${CALLER_PARTITION}" != "${IGN_PARTITION}" ]; then
    echo "    IGN_PARTITION: 호출 셸의 ${CALLER_PARTITION} 대신 ${IGN_PARTITION} (유지하려면 --ign-partition env)"
fi

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
echo "    로그: ${LOG_DIR}   부하: $(cut -d' ' -f1-3 /proc/loadavg) / $(nproc) CPU, uptime $(cut -d' ' -f1 /proc/uptime) s"
echo "    DDS: ${FASTRTPS_DEFAULT_PROFILES_FILE:-(기본)}   /dev/shm: $(df -h /dev/shm | awk 'NR==2 {print $2}')"

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
    if [ "${id}" = "14_soak" ] && [ -z "${SCENARIO_TIMEOUT}" ]; then
        # 운용 시간 + 30 분 (기동·준비·종료)
        limit=$(python3 -c 'import math, os; print(int(math.ceil(float(os.environ.get("ITEST_SOAK_HOURS", "4.0")) * 3600)) + 1800)')
    fi
    t0=$(date +%s)
    export ITEST_SCENARIO_TIMEOUT="${limit}"     # 반복 시나리오가 남은 시간으로 새 시행을 정한다
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
    printf '    %-26s %-8s %5d s  (rc=%s, 부하 %s)\n' "${id}" "${status}" "${dt}" "${rc}" \
        "$(cut -d' ' -f1 /proc/loadavg)"
done

echo
echo "--- 집계"
REPORT_ARGS=("${SKIP_POLICY[@]}")
RC=0
if [ ${#IDS[@]} -gt 0 ]; then
    report --scenarios "${IDS[@]}" "${REPORT_ARGS[@]}" || RC=$?
else
    report --scenarios "${REPORT_ARGS[@]}" || RC=$?
fi
[ "${UNIT_RC}" = "0" ] || RC=1

echo "총 소요: $(( $(date +%s) - T_START )) s, 종료 시 부하: $(cut -d' ' -f1-3 /proc/loadavg), uptime $(cut -d' ' -f1 /proc/uptime) s"
exit "${RC}"
