#!/usr/bin/env bash
# layopt E2 — 컨테이너 진입: Gazebo + 단일 로봇 운용 스택을 띄우고 시행 러너(e2_runner.py)를 포그라운드로 돈다.
# 호스트 e2_campaign.py 가 gz_slot.sh 아래에서 `docker run ... bash /ros2_ws/layopt/loc/e2_container.sh` 로 부른다.
#
# 스택 = amr_bringup system.launch.py (multi_robot.launch.py 와 같은 launch_utils.bringup_actions, 같은 패키지 YAML).
#   robot_name amr_01, prefix amr_01/ (다중 로봇과 같은 프레임 이름), localization_mode localization.
#   켬: localization(amcl + 이중 EKF + kidnap_monitor + scan_matcher), navigation(behavior_server spin),
#       perception(safety_node — cmd_vel 의 유일 발행자, 빼면 복구 회전이 끊긴다: test_05 머리말).
#   끔: behavior(작업 실행기·도킹 서버 — 단일 로봇에 작업이 없고, 도킹 서버의 marker_fix 는 lost 중 무시된다),
#       fleet·dashboard·evaluation (단일 로봇 시행과 무관).
# 환경: E2_RUN_DIR(컨테이너 경로) E2_TRIALS E2_SESSION E2_WORLD(월드 sdf) E2_MAP(map.yaml)
#       E2_SPAWN("x,y,yaw") E2_PER_SESSION E2_DUMMY_MODE(move|recreate) E2_ONLY(선택) E2_MAX_ATTEMPTS
#       E2_DOMAIN E2_PARTITION E2_LANE E2_LANES (병렬 세션) E2_MODULE (기본 e2_runner, 탐침은 probe_stationary)
set -u
: "${E2_RUN_DIR:?}" "${E2_TRIALS:?}" "${E2_SESSION:?}" "${E2_WORLD:?}" "${E2_MAP:?}" "${E2_SPAWN:?}"
set +u
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
set -u
# 레인 0 = 57 / amr_ai_layopt_e2 (contract §5), 레인 1 = 58 / amr_ai_layopt_e2b (조정자 2026-10-10 05:00 허용)
export ROS_DOMAIN_ID="${E2_DOMAIN:-57}"
export IGN_PARTITION="${E2_PARTITION:-amr_ai_layopt_e2}"
export PYTHONHASHSEED=0
export PYTHONPATH="/ros2_ws:${PYTHONPATH:-}"
unset ROS_LOCALHOST_ONLY
CFG=/ros2_ws/install/amr_bringup/share/amr_bringup/config
export FASTRTPS_DEFAULT_PROFILES_FILE="${CFG}/fastdds_multi_robot.xml"
SD="${E2_RUN_DIR}/sessions/${E2_SESSION}"
mkdir -p "${SD}"
env | grep -E '^(ROS_|RMW_|FASTRTPS_|IGN_|GZ_|E2_)' | sort > "${SD}/env.txt"

# 월드: launch 는 파일 이름(확장자 제외) = <world name> 을 요구한다 → 이름을 맞춘 사본
WNAME=$(python3 -c "import re,sys; t=open(sys.argv[1],encoding='utf-8').read(); print(re.search(r'<world\s+name=\"([^\"]+)\"', t).group(1))" "${E2_WORLD}")
WDIR=$(mktemp -d /tmp/e2world.XXXXXX)
cp "${E2_WORLD}" "${WDIR}/${WNAME}.sdf"
IFS=, read -r SX SY SYAW <<< "${E2_SPAWN}"
ARGS=(robot_name:=amr_01 prefix:=amr_01/ "x:=${SX}" "y:=${SY}" "yaw:=${SYAW}"
      "world:=${WDIR}/${WNAME}.sdf" "map_yaml:=${E2_MAP}" headless:=true use_sim_time:=true
      localization_mode:=localization with_localization:=true with_navigation:=true
      with_perception:=true with_behavior:=false with_fleet:=false with_dashboard:=false
      with_evaluation:=false)
echo "$(date -Iseconds) e2_container: world ${WNAME} (${E2_WORLD}) map ${E2_MAP} spawn ${E2_SPAWN}" > "${SD}/container.log"
echo "ros2 launch amr_bringup system.launch.py ${ARGS[*]}" >> "${SD}/container.log"
setsid ros2 launch amr_bringup system.launch.py "${ARGS[@]}" > "${SD}/launch.log" 2>&1 &
LPID=$!

stop_stack() {
    if kill -0 "${LPID}" 2> /dev/null; then
        kill -INT -- "-${LPID}" 2> /dev/null
        for _ in $(seq 1 60); do kill -0 "${LPID}" 2> /dev/null || break; sleep 1; done
        kill -KILL -- "-${LPID}" 2> /dev/null
    fi
    echo "$(date -Iseconds) 스택 정지" >> "${SD}/container.log"
}
trap 'stop_stack; exit 143' TERM INT

python3 -m "${E2_MODULE:-layopt.loc.e2_runner}" --trials "${E2_TRIALS}" --run-dir "${E2_RUN_DIR}" \
    --world-name "${WNAME}" --session "${E2_SESSION}" --per-session "${E2_PER_SESSION:-20}" \
    --dummy-mode "${E2_DUMMY_MODE:-move}" --only "${E2_ONLY:-}" \
    --max-attempts "${E2_MAX_ATTEMPTS:-2}" --lane "${E2_LANE:-0}" --lanes "${E2_LANES:-1}" \
    > "${SD}/runner.log" 2>&1
RC=$?
echo "$(date -Iseconds) runner rc=${RC}" >> "${SD}/container.log"
stop_stack
exit "${RC}"
