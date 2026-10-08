#!/bin/bash
# 판정 실행 (b4-wire, 사전 등록 notebook *_b4-wire_wire-launch-tests.md): 실제 실행기 launch 시험 W1 · W2 를 시드 1–3.
# 컨테이너 안 (저장소 스냅숏을 /ros2_ws 읽기 전용, 산출 /out): 새 build/install(/out/ws) 을 Release 로 만들고
# 시드 · 시험마다 /out/<시험>_s<시드>/ 에 원자료 · pytest 출력 · 시작/끝 loadavg. 끝난 칸(rc 파일 있음)은 건너뛴다 (재개).
# SEEDS 환경변수로 시드 목록을 바꿀 수 있다 (기본 "1 2 3").
source /opt/ros/humble/setup.bash
cd /ros2_ws
B=/out/ws/build; I=/out/ws/install; L=/out/ws/log
export ROS_LOCALHOST_ONLY=1 PYTHONHASHSEED=0 PYTHONDONTWRITEBYTECODE=1
la() { local a b c d; read -r a b c d < /proc/loadavg; echo "$a $b $c"; }
if [ ! -f $I/setup.bash ]; then
  MAKEFLAGS=-j4 colcon --log-base $L build --build-base $B --install-base $I --parallel-workers 4 \
    --packages-up-to amr_msgs amr_fleet amr_behavior \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-Werror > /out/build.log 2>&1 || exit 1
fi
source $I/setup.bash
export TASK_EXECUTOR_EXE=$B/amr_behavior/task_executor_node
# 남은 프로세스 (앞 실행의 고아). W-launch 1 회차에서 harness 의 bash -c 에 exec 가 없어 에이전트가 launch 종료 뒤에도
# 남았고, 같은 도메인의 다음 시드에 섞여 시드 2 를 망쳤다 (결과 뒤 변경, 노트에 적음). 시험 앞뒤로 세어 meta 에 적고
# (판정 원자료 — strays_after 는 0 이어야 한다) 남은 것은 죽인다. 격리는 이 스크립트가 보장한다.
PAT='amr_fleet[.]|task_executor_node|fake_fleet_world'
kill_strays() {  # pgrep 으로 PID 를 확인한 뒤 그 PID 만 (자기 셸 제외)
  local p; p=$(pgrep -f "$PAT" | grep -vx "$$")
  [ -n "$p" ] && kill -9 $p 2>/dev/null
  return 0
}
for seed in ${SEEDS:-1 2 3}; do
  for t in test_wire_launch test_wire_partition; do
    d=/out/${t}_s${seed}
    [ -f $d/rc ] && continue
    rm -rf $d; mkdir -p $d
    pgrep -af "$PAT" > $d/strays_before.txt
    echo "strays_before $(wc -l < $d/strays_before.txt)" > $d/meta
    kill_strays
    echo "start $(date -Iseconds) load $(la)" >> $d/meta
    if [ $t = test_wire_launch ]; then dom=56; else dom=55; fi
    (cd /ros2_ws/src/amr_behavior/test_fleet_world && ROS_DOMAIN_ID=$dom AMR_WIRE_SEED=$seed \
      AMR_WIRE_RAW=$d/raw timeout 1500 python3 -m pytest -q -p no:cacheprovider $t.py \
      > $d/pytest.log 2>&1)
    rc=$?
    echo "end $(date -Iseconds) load $(la) rc=$rc" >> $d/meta
    pgrep -af "$PAT" > $d/strays_after.txt
    echo "strays_after $(wc -l < $d/strays_after.txt)" >> $d/meta
    kill_strays
    echo $rc > $d/rc
  done
done
