#!/usr/bin/env bash
# 도킹 측정용 단일 로봇 시스템 기동 (dock_system.launch.py). 인자: 로그 이름, 이후 ros2 launch 인자
HERE="$(cd "$(dirname "$0")" && pwd)"
source "${HERE}/env.sh"
name="$1"; shift
echo "[bhv] $(date +%F_%T) uptime: $(uptime)" > "${OUT}/${name}.log"
exec setsid ros2 launch "${HERE}/dock_system.launch.py" "$@" >> "${OUT}/${name}.log" 2>&1
