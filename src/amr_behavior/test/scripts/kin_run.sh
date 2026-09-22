#!/usr/bin/env bash
# 운동학 폐루프 도킹 조건 1개: kin_run.sh <이름> <law> <filter_coef> <heading_stop> <noise_pos> <noise_yaw_deg> [trials]
HERE="$(cd "$(dirname "$0")" && pwd)"
source "${HERE}/env.sh"
export RCUTILS_LOGGING_BUFFERED_STREAM=0
name=$1; law=$2; fc=$3; hs=$4; np=$5; ny=$6; n=${7:-15}
out="${OUT}"
printf "/**/docking_server_node:\n  ros__parameters:\n    control_law: %s\n    filter_coef: %s\n    heading_stop_tolerance: %s\n" "$law" "$fc" "$hs" > $out/$name.yaml
/ros2_ws/install/amr_behavior/lib/amr_behavior/docking_server_node --ros-args -r __ns:=/kin_$name --params-file /ros2_ws/install/amr_behavior/share/amr_behavior/config/behavior.yaml --params-file $out/$name.yaml > $out/$name.server.log 2>&1 &
sp=$!
sleep 3
echo "uptime_start: $(uptime)" > $out/$name.meta
timeout 3000 python3 "${HERE}/dock_trials_kin.py" --mode kinematic --ns kin_$name --trials $n --seed 1 --noise-pos $np --noise-yaw-deg $ny --out $out/$name.json > $out/$name.log 2>&1
echo "rc=$? uptime_end: $(uptime)" >> $out/$name.meta
kill -INT $sp
wait $sp
