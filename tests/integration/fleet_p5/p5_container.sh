#!/usr/bin/env bash
# 보너스 4 P5 — 스택 컨테이너 진입 (Gazebo 5 대 + 로봇 스택 + 어댑터; 서버는 따로 p5_server.sh 컨테이너).
# 호스트 p5_driver.py 가 `docker run ... bash /ros2_ws/tests/integration/fleet_p5/p5_container.sh` 로 부른다.
# 환경: P5_TAG P5_MODE(central|distributed) P5_DOMAIN P5_OUT(/raw/<태그>) P5_PLAN P5_COMM(선택, 통신 모의 YAML)
# 1) multi_robot.launch.py 를 launch_manager:=false traffic_manager:=false 로 (관리자 · 교통 관리자는 서버 컨테이너가),
#    fleet_mode 와 실행마다 새 에이전트 저장소를 준다. 통신 모의가 있으면 에이전트(환경 변수)와 어댑터(launch 인자)에 같은 파일.
# 2) 기록기(p5_recorder.py)를 UDP 전용 DDS 프로파일로 포그라운드 실행 — 끝나면 스택을 SIGINT → 60 s 뒤 SIGKILL.
set -u
: "${P5_TAG:?}" "${P5_MODE:?}" "${P5_DOMAIN:?}" "${P5_OUT:?}" "${P5_PLAN:?}"
set +u
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID="${P5_DOMAIN}"
export IGN_PARTITION="b4p5_${P5_TAG}"
export PYTHONHASHSEED=0
unset ROS_LOCALHOST_ONLY
CFG=/ros2_ws/install/amr_bringup/share/amr_bringup/config
export FASTRTPS_DEFAULT_PROFILES_FILE="${CFG}/fastdds_multi_robot.xml"
mkdir -p "${P5_OUT}"
env | grep -E '^(ROS_|RMW_|FASTRTPS_|IGN_|GZ_|P5_|AMR_)' | sort > "${P5_OUT}/env.txt"

ARGS=(num_robots:=5 use_sim_time:=true headless:=true localization_mode:=localization
      map_yaml:=/ros2_ws/maps/warehouse.yaml with_fleet:=true with_dashboard:=false
      with_evaluation:=false launch_manager:=false traffic_manager:=false
      "fleet_mode:=${P5_MODE}")
if [ "${P5_MODE}" = distributed ]; then
    ARGS+=("fleet_store_dir:=${P5_OUT}/store")
fi
if [ -n "${P5_COMM:-}" ]; then
    export AMR_FLEET_COMM_MODEL_FILE="${P5_COMM}"
    ARGS+=(simulate_latency:=true "comm_model_file:=${P5_COMM}")
fi
echo "$(date -Iseconds) p5_container: ros2 launch amr_bringup multi_robot.launch.py ${ARGS[*]}" \
    > "${P5_OUT}/container.log"

if [ "${P5_DRYRUN:-0}" = 1 ]; then
    # 파이프라인 시험 전용 (Gazebo 없음): 스택 대신 가짜 /clock 만 — 기록기는 준비를 못 해 READY_TIMEOUT 으로 끝난다
    echo "P5_DRYRUN: 스택 대신 amr_fleet.agent.fake_clock" >> "${P5_OUT}/container.log"
    setsid python3 -m amr_fleet.agent.fake_clock --ros-args -p start_s:=1.0 \
        > "${P5_OUT}/launch.log" 2>&1 &
else
    setsid ros2 launch amr_bringup multi_robot.launch.py "${ARGS[@]}" > "${P5_OUT}/launch.log" 2>&1 &
fi
LPID=$!
echo "launch pid ${LPID}" >> "${P5_OUT}/container.log"

stop_stack() {
    if kill -0 "${LPID}" 2> /dev/null; then
        kill -INT -- "-${LPID}" 2> /dev/null
        for _ in $(seq 1 60); do kill -0 "${LPID}" 2> /dev/null || break; sleep 1; done
        kill -KILL -- "-${LPID}" 2> /dev/null
    fi
    echo "$(date -Iseconds) 스택 정지, 남은 프로세스 $(ps -e --no-headers | wc -l)" >> "${P5_OUT}/container.log"
}
trap 'stop_stack; exit 143' TERM INT

SHARE_FLEET=/ros2_ws/install/amr_fleet/share/amr_fleet/config
SHARE_BEH=/ros2_ws/install/amr_behavior/share/amr_behavior/config
REC_EXTRA=()
if [ "${P5_DRYRUN:-0}" = 1 ]; then
    REC_EXTRA+=(--skip-ready)
fi
FASTRTPS_DEFAULT_PROFILES_FILE="${CFG}/fastdds_multi_robot_udponly.xml" \
    python3 /ros2_ws/tests/integration/fleet_p5/p5_recorder.py --out "${P5_OUT}" --plan "${P5_PLAN}" \
    "${REC_EXTRA[@]}" \
    --launch-log "${P5_OUT}/launch.log" --zones "${SHARE_FLEET}/traffic_zones.yaml" \
    --traffic "${SHARE_FLEET}/traffic.yaml" --behavior "${SHARE_BEH}/behavior.yaml" \
    > "${P5_OUT}/recorder.log" 2>&1
RC=$?
echo "$(date -Iseconds) recorder rc=${RC}" >> "${P5_OUT}/container.log"
stop_stack
exit "${RC}"
