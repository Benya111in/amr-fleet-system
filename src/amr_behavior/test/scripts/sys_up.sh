#!/usr/bin/env bash
# 종단 작업 시험용 전체 시스템 기동 (amr_bringup system.launch.py). 인자: 로그 이름, 이후 ros2 launch 인자
HERE="$(cd "$(dirname "$0")" && pwd)"
source "${HERE}/env.sh"
name="$1"; shift
echo "[bhv] $(date +%F_%T) uptime: $(uptime)" > "${OUT}/${name}.log"
exec setsid ros2 launch amr_bringup system.launch.py "$@" >> "${OUT}/${name}.log" 2>&1
