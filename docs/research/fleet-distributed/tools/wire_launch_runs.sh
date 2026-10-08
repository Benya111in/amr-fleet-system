#!/bin/bash
# 판정 실행 (b4-wire, 사전 등록 notebook *_b4-wire_wire-launch-tests.md): 실제 실행기 launch 시험 W1 · W2 를 시드 1–3.
# 컨테이너 안 (저장소 스냅숏을 /ros2_ws 읽기 전용, 산출 /out): 새 build/install(/out/ws) 을 Release 로 만들고
# 시드 · 시험마다 /out/<시험>_s<시드>/ 에 원자료 · pytest 출력 · 시작/끝 loadavg. 끝난 칸(rc 파일 있음)은 건너뛴다 (재개).
source /opt/ros/humble/setup.bash
cd /ros2_ws
B=/out/ws/build; I=/out/ws/install; L=/out/ws/log
export ROS_LOCALHOST_ONLY=1 PYTHONHASHSEED=0 PYTHONDONTWRITEBYTECODE=1
if [ ! -f $I/setup.bash ]; then
  MAKEFLAGS=-j4 colcon --log-base $L build --build-base $B --install-base $I --parallel-workers 4 \
    --packages-up-to amr_msgs amr_fleet amr_behavior \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-Werror > /out/build.log 2>&1 || exit 1
fi
source $I/setup.bash
export TASK_EXECUTOR_EXE=$B/amr_behavior/task_executor_node
for seed in 1 2 3; do
  for t in test_wire_launch test_wire_partition; do
    d=/out/${t}_s${seed}
    [ -f $d/rc ] && continue
    rm -rf $d; mkdir -p $d
    echo "start $(date -Iseconds) load $(cut -d' ' -f1-3 /proc/loadavg)" > $d/meta
    if [ $t = test_wire_launch ]; then dom=56; else dom=55; fi
    (cd /ros2_ws/src/amr_behavior/test_fleet_world && ROS_DOMAIN_ID=$dom AMR_WIRE_SEED=$seed \
      AMR_WIRE_RAW=$d/raw timeout 1500 python3 -m pytest -q -p no:cacheprovider $t.py \
      > $d/pytest.log 2>&1)
    rc=$?
    echo "end $(date -Iseconds) load $(cut -d' ' -f1-3 /proc/loadavg) rc=$rc" >> $d/meta
    echo $rc > $d/rc
  done
done
