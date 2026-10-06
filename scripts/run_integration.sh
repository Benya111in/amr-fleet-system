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
#                                    섞이지 않게. 'env' 면 현재 값 유지. 병렬 CI 는 N 을 명시). N(과 env 값)은 0~101 ·
#                                    215~232 만 받는다: 102~214 는 참가자 포트가 Linux 임시 포트 범위(32768~60999)와
#                                    겹쳐 거부한다 (종료 코드 2 — ROS 2 문서 "The ROS_DOMAIN_ID", hold_failsafe.md §7)
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
#   --self-test                      도메인 규칙 자체 시험만 돌리고 끝낸다 (ROS·빌드 불필요)
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

usage() { awk 'NR == 1 {next} /^#/ {sub(/^# ?/, ""); print; next} {exit}' "${BASH_SOURCE[0]}"; }

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
SELF_TEST=0
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
        --self-test) SELF_TEST=1; shift ;;
        -h|--help) usage; exit 0 ;;
        -*) echo "알 수 없는 옵션: $1" >&2; usage >&2; exit 2 ;;
        *) SELECT+=("$1"); shift ;;
    esac
done

# 이 실행 전용 DDS 도메인 규칙. 참가자 p 의 수신 포트는 7400 + 250·d + {10, 11} + 2·p 다 (Fast DDS 기본 포트 공식).
# 도메인 102~214 는 그 포트가 Linux 임시 포트 범위(32768~60999)와 겹친다 — 다른 프로세스의 송신 소켓이 참가자 포트를
# 먼저 잡으면 그 참가자의 UDP 포트만 다음 번호로 밀려 UDP·SHM 포트 번호가 어긋난다 (리서치 「실측 기반 후속 기술 조사」).
# 시나리오 12 에서 늦게 뜬 실행기가 매칭된 채 traffic/hold 를 못 받은 실행들이 도메인 210 이었다. 그래서 경고가 아니라
# 거부한다. 233 이상은 포트가 65535 를 넘는다.
rand16() { od -An -N2 -tu2 /dev/urandom | tr -d ' '; }
# 검사한 도메인(10 진수로 정규화)을 출력하고 0, 거부면 이유를 stderr 로 내고 2.
check_domain_id() {
    local v="$1"
    case "${v}" in
        ''|*[!0-9]*) echo "--domain-id: 0~101 · 215~232 정수, auto, env 중 하나: '${v}'" >&2; return 2 ;;
    esac
    v="${v#"${v%%[!0]*}"}"          # 앞자리 0 을 지운다 (bash 산술은 0 으로 시작하는 수를 8 진수로 읽는다)
    [ -n "${v}" ] || v=0
    if [ "${#v}" -gt 3 ] || [ "${v}" -gt 232 ]; then
        echo "--domain-id ${1} 거부: 232 를 넘으면 참가자 포트가 65535 를 넘는다" >&2
        return 2
    fi
    if [ "${v}" -ge 102 ] && [ "${v}" -le 214 ]; then
        echo "--domain-id ${v} 거부: 참가자 포트가 Linux 임시 포트 범위(32768~60999)와 겹친다 (ROS 2 문서 안전 범위" \
             "0~101 · 215~232, hold_failsafe.md §7). auto(1~101) 또는 215~229 를 쓴다" >&2
        return 2
    fi
    echo "${v}"
}
# auto: 1~101 난수 (/dev/urandom — 새 컨테이너마다 거의 같은 $$ 로 만들면 병렬 실행이 같은 값을 받는다).
# 호출한 셸의 값(인자 1)은 피한다.
pick_auto_domain() {
    local d=$(( 1 + $(rand16) % 101 ))
    [ "${d}" = "${1:-0}" ] && d=$(( d % 101 + 1 ))
    echo "${d}"
}
# 사용자 유니캐스트 포트(7411 + 250·d + 2·p)가 임시 포트 범위(도메인 ≤ 101) 또는 65535 에 닿기 전까지의 참가자 수.
# 232 → 63 (ROS 2 문서의 "도메인 232 는 63 프로세스" 와 같다). 5대 시나리오 12 는 참가자 ≈ 150~170.
domain_port_ceiling() {
    local d="$1" limit=65535
    [ "${d}" -le 101 ] && limit=32767
    echo $(( (limit - 7411 - 250 * d) / 2 + 1 ))
}
self_test_domain() {
    local fail=0 d out rc exp item arg want_rc want_out i
    for d in $(seq 0 240); do
        exp=2
        if [ "${d}" -le 101 ] || { [ "${d}" -ge 215 ] && [ "${d}" -le 232 ]; }; then exp=0; fi
        rc=0; out=$(check_domain_id "${d}" 2>/dev/null) || rc=$?
        if [ "${rc}" != "${exp}" ] || { [ "${exp}" = 0 ] && [ "${out}" != "${d}" ]; }; then
            echo "FAIL 도메인 ${d}: rc=${rc} 출력='${out}' (기대 rc=${exp})"; fail=1
        fi
    done
    # 인자:기대 rc:기대 출력
    for item in 'abc:2:' ':2:' '-1:2:' '1.5:2:' '1 :2:' '0101:0:101' '0210:2:' '007:0:7' '000:0:0' \
                '0000000000000000000001:0:1' '99999999999999999999:2:' '1000:2:'; do
        arg="${item%%:*}"; want_rc="${item#*:}"; want_out="${want_rc#*:}"; want_rc="${want_rc%%:*}"
        rc=0; out=$(check_domain_id "${arg}" 2>/dev/null) || rc=$?
        if [ "${rc}" != "${want_rc}" ] || [ "${out}" != "${want_out}" ]; then
            echo "FAIL 인자 '${arg}': rc=${rc} 출력='${out}' (기대 rc=${want_rc} 출력='${want_out}')"; fail=1
        fi
    done
    for i in $(seq 1 2000); do
        d=$(pick_auto_domain 50)
        if [ "${d}" -lt 1 ] || [ "${d}" -gt 101 ] || [ "${d}" = 50 ]; then
            echo "FAIL auto: ${d} (1~101 이고 호출 셸 값 50 이 아니어야 한다)"; fail=1; break
        fi
    done
    [ "$(pick_auto_domain 101)" != 101 ] || { echo "FAIL auto: 101 을 피하지 못했다"; fail=1; }
    for item in 0:12679 99:304 100:179 101:54 215:2188 229:438 232:63; do
        d="${item%%:*}"
        if [ "$(domain_port_ceiling "${d}")" != "${item#*:}" ]; then
            echo "FAIL 참가자 상한 ${d}: $(domain_port_ceiling "${d}") (기대 ${item#*:})"; fail=1
        fi
    done
    if [ "${fail}" = 0 ]; then echo "self-test: 통과"; else echo "self-test: 실패"; fi
    return "${fail}"
}
if [ "${SELF_TEST}" = "1" ]; then
    rc=0; self_test_domain || rc=$?
    exit "${rc}"
fi

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

# 이 실행 전용 DDS 도메인·Gazebo 파티션 (같은 컨테이너의 dev 스택/다른 실행과 섞이지 않게). 규칙은 위 check_domain_id.
if [ "${DOMAIN_ID}" = "auto" ]; then
    export ROS_DOMAIN_ID="$(pick_auto_domain "${ROS_DOMAIN_ID:-0}")"
else
    [ "${DOMAIN_ID}" = "env" ] && DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    d=$(check_domain_id "${DOMAIN_ID}") || exit 2
    export ROS_DOMAIN_ID="${d}"
fi
ceiling=$(domain_port_ceiling "${ROS_DOMAIN_ID}")
if [ "${ceiling}" -lt 200 ]; then
    echo "    참고: 도메인 ${ROS_DOMAIN_ID} 에서는 참가자가 ${ceiling} 개를 넘으면 수신 포트가 임시 포트 범위·65535 에 닿는다" \
         "(5대 시나리오 12 는 ≈ 150~170 개)"
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
    # 시나리오별 기본 상한. 1800 일괄값은 sim 시간을 모르는 값이라, RTF 가 낮은 시나리오를
    # 구조적으로 실패시킨다 — 12 는 5대 동거로 RTF 0.21~0.32 이고 5 작업 완주에 sim 280 s
    # (= wall 900~1300 s) 가 필요한데 기동·wedge 우회까지 합치면 1800 s 로는 못 끝낸다.
    # 13 은 시행 간격이 sim 120 s 로 고정이라 50 표본에 wall 6800~7500 s 가 필요하다
    # (실측: 1800 s 에서 9~12 표본). 실제로 통과한 실행들이 쓴 값을 기본값으로 올린다.
    case "${id}" in
        12_*) default_limit=7200 ;;
        13_*) default_limit=9000 ;;
        *)    default_limit=1800 ;;
    esac
    limit="${SCENARIO_TIMEOUT:-${default_limit}}"
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
