#!/usr/bin/env bash
# 보너스 4 P5 — 스택 컨테이너 진입 (Gazebo 5 대 + 로봇 스택 + 어댑터; 서버는 따로 p5_server.sh 컨테이너).
# 호스트 p5_driver.py 가 `docker run ... bash /ros2_ws/tests/integration/fleet_p5/p5_container.sh` 로 부른다.
# 환경: P5_TAG P5_MODE(central|distributed) P5_DOMAIN P5_OUT(/raw/<태그>) P5_PLAN P5_COMM(선택, 통신 모의 YAML)
#       P5_PARTITION(선택, 기본 b4p5_<태그>) P5_BUNDLE(선택, layopt 배치 묶음 마운트 /bundle — 아래 0))
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
export IGN_PARTITION="${P5_PARTITION:-b4p5_${P5_TAG}}"
export PYTHONHASHSEED=0
unset ROS_LOCALHOST_ONLY
CFG=/ros2_ws/install/amr_bringup/share/amr_bringup/config
# DDS: 기본은 bringup 과 같은 SHM + UDP 프로파일. 01:25 에 UDP 전용으로 바꿨다가(SHM 에서 /clock 정지 7 중 3) 04:00 에 되돌렸다 —
# UDP 전용은 부하 아래 영상 처리율 0.1 FPS · scan costmap 지연 · 제어 루프 누락으로 주행이 무너졌다 (노트 04:00). /clock 정지는
# 기록기의 180 s 감시로 빨리 무효화하고 짝을 다시 돈다. P5_DDS=udp 면 UDP 전용 (진단용). bringup 은 이미 준 프로파일을 유지한다.
if [ "${P5_DDS:-shm}" = udp ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${CFG}/fastdds_multi_robot_udponly.xml"
else
    export FASTRTPS_DEFAULT_PROFILES_FILE="${CFG}/fastdds_multi_robot.xml"
fi
mkdir -p "${P5_OUT}"
env | grep -E '^(ROS_|RMW_|FASTRTPS_|IGN_|GZ_|P5_|AMR_)' | sort > "${P5_OUT}/env.txt"

# 0) 배치 묶음 (p5_driver.py --bundle): 드라이버가 묶음 파일을 소스 트리 위에 겹쳐 마운트했다. 노드가 실제로 여는
#    경로(install share 의 symlink 를 따라간 곳, traffic_manager 의 소스 트리 기본값)가 묶음 파일과 같은지 sha256 으로 확인하고,
#    하나라도 다르면 기동하지 않는다 (종료 5). 지도·스폰 목록은 런치 인자로도 묶음 경로를 준다.
SHARE_FLEET=/ros2_ws/install/amr_fleet/share/amr_fleet/config
SHARE_BEH=/ros2_ws/install/amr_behavior/share/amr_behavior/config
ZONES_FILE="${SHARE_FLEET}/traffic_zones.yaml"
BEHAVIOR_FILE="${SHARE_BEH}/behavior.yaml"
MAP_YAML=/ros2_ws/maps/warehouse.yaml
BUNDLE_ARGS=()
if [ -n "${P5_BUNDLE:-}" ]; then
    BAD=0
    {
        echo "묶음 ${P5_BUNDLE} $(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["spec_id"])' "${P5_BUNDLE}/spec.json")"
        for pair in "world.sdf:/ros2_ws/install/amr_simulation/share/amr_simulation/worlds/warehouse.sdf" \
                    "config/behavior.yaml:${SHARE_BEH}/behavior.yaml" \
                    "config/fleet_spawn.yaml:/ros2_ws/install/amr_bringup/share/amr_bringup/config/fleet_spawn.yaml" \
                    "config/traffic_zones.yaml:${SHARE_FLEET}/traffic_zones.yaml" \
                    "config/traffic_zones.yaml:/ros2_ws/src/amr_fleet/config/traffic_zones.yaml"; do
            want=$(sha256sum "${P5_BUNDLE}/${pair%%:*}" | cut -d' ' -f1)
            got=$(sha256sum "$(readlink -f "${pair#*:}")" | cut -d' ' -f1)
            [ "$want" = "$got" ] && ok=ok || { ok=MISMATCH; BAD=1; }
            echo "${ok} ${pair%%:*} -> ${pair#*:} ($(readlink -f "${pair#*:}"))"
        done
    } > "${P5_OUT}/bundle_check.txt" 2>&1
    if [ "${BAD}" != 0 ]; then
        echo "$(date -Iseconds) 묶음 겹침 확인 실패 — 기동하지 않는다 (bundle_check.txt)" >> "${P5_OUT}/container.log"
        exit 5
    fi
    ZONES_FILE="${P5_BUNDLE}/config/traffic_zones.yaml"
    BEHAVIOR_FILE="${P5_BUNDLE}/config/behavior.yaml"
    MAP_YAML="${P5_BUNDLE}/map.yaml"
    BUNDLE_ARGS=("robots_file:=${P5_BUNDLE}/config/fleet_spawn.yaml")
fi

ARGS=(num_robots:=5 use_sim_time:=true headless:=true localization_mode:=localization
      "map_yaml:=${MAP_YAML}" with_fleet:=true with_dashboard:=false
      with_evaluation:=false launch_manager:=false traffic_manager:=false
      "fleet_mode:=${P5_MODE}" "${BUNDLE_ARGS[@]}")
if [ "${P5_MODE}" = distributed ]; then
    ARGS+=("fleet_store_dir:=${P5_OUT}/store")
fi
if [ -n "${P5_COMM:-}" ]; then
    export AMR_FLEET_COMM_MODEL_FILE="${P5_COMM}"
    ARGS+=(simulate_latency:=true "comm_model_file:=${P5_COMM}")
fi
echo "$(date -Iseconds) p5_container: ros2 launch amr_bringup multi_robot.launch.py ${ARGS[*]}" \
    > "${P5_OUT}/container.log"
export -p > "${P5_OUT}/stack_env.sh"      # 에이전트 재기동(p5_agent_ctl.py restart)이 같은 환경을 쓰게

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

REC_EXTRA=()
if [ "${P5_DRYRUN:-0}" = 1 ]; then
    REC_EXTRA+=(--skip-ready)
fi
FASTRTPS_DEFAULT_PROFILES_FILE="${CFG}/fastdds_multi_robot_udponly.xml" \
    python3 /ros2_ws/tests/integration/fleet_p5/p5_recorder.py --out "${P5_OUT}" --plan "${P5_PLAN}" \
    "${REC_EXTRA[@]}" \
    --launch-log "${P5_OUT}/launch.log" --zones "${ZONES_FILE}" \
    --traffic "${SHARE_FLEET}/traffic.yaml" --behavior "${BEHAVIOR_FILE}" \
    > "${P5_OUT}/recorder.log" 2>&1
RC=$?
echo "$(date -Iseconds) recorder rc=${RC}" >> "${P5_OUT}/container.log"
stop_stack
exit "${RC}"
