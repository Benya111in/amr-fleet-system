#!/usr/bin/env bash
# 보너스 4 P5 — 서버 컨테이너 진입 (fleet_manager_node + traffic_manager_node, 어댑터 없음).
# 스택 컨테이너와 네트워크 · IPC 이름공간을 함께 쓴다 (docker run --network container:<스택> --ipc container:<스택>).
# 장애 주입은 이 컨테이너 단위다: docker kill (보고서 F1 충돌) · docker pause/unpause (F3 정지 뒤 재개) · 새 컨테이너 (재기동 —
# 분산 모드면 관찰자 기동: 로봇 펜스 최고 + 1). epoch_file 은 갈래마다 하나 (/raw/<태그>/epoch.txt) — 재기동이 이어 쓴다.
# 환경: P5_TAG P5_MODE P5_DOMAIN P5_OUT P5_SERVER(번호) P5_COMM(선택)
set -u
: "${P5_TAG:?}" "${P5_MODE:?}" "${P5_DOMAIN:?}" "${P5_OUT:?}" "${P5_SERVER:?}"
set +u
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID="${P5_DOMAIN}"
export PYTHONHASHSEED=0
unset ROS_LOCALHOST_ONLY
CFG=/ros2_ws/install/amr_bringup/share/amr_bringup/config
if [ "${P5_DDS:-shm}" = udp ]; then        # 스택과 같은 전송 (p5_container.sh 주석)
    export FASTRTPS_DEFAULT_PROFILES_FILE="${CFG}/fastdds_multi_robot_udponly.xml"
else
    export FASTRTPS_DEFAULT_PROFILES_FILE="${CFG}/fastdds_multi_robot.xml"
fi
mkdir -p "${P5_OUT}/srv${P5_SERVER}"
if [ -n "${P5_BUNDLE:-}" ]; then        # 묶음 구역 파일이 traffic_manager 가 여는 소스 트리 경로에 겹쳤는지 (p5_container.sh 0))
    want=$(sha256sum "${P5_BUNDLE}/config/traffic_zones.yaml" | cut -d' ' -f1)
    got=$(sha256sum "$(readlink -f /ros2_ws/src/amr_fleet/config/traffic_zones.yaml)" | cut -d' ' -f1)
    echo "$(date -Iseconds) 묶음 구역 파일 ${want} / 서버가 보는 파일 ${got}" >> "${P5_OUT}/srv${P5_SERVER}/server.log"
    [ "$want" = "$got" ] || { echo "묶음 구역 파일 불일치 — 서버 기동 안 함" >> "${P5_OUT}/srv${P5_SERVER}/server.log"; exit 5; }
fi
ARGS=(launch_adapters:=false robot_ids:=amr_01,amr_02,amr_03,amr_04,amr_05 use_sim_time:=true
      "fleet_mode:=${P5_MODE}" "epoch_file:=${P5_OUT}/epoch.txt" "log_dir:=${P5_OUT}/srv${P5_SERVER}")
if [ -n "${P5_COMM:-}" ]; then
    ARGS+=(simulate_latency:=true "comm_model_file:=${P5_COMM}")
fi
echo "$(date -Iseconds) p5_server ${P5_SERVER}: ${ARGS[*]}" >> "${P5_OUT}/srv${P5_SERVER}/server.log"
exec ros2 launch amr_fleet fleet_manager.launch.py "${ARGS[@]}"
