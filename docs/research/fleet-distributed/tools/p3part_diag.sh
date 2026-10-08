#!/bin/bash
# 회귀 진단 (b4-wire, 사전 등록 notebook *_b4-wire_p3-partition-regression-diagnosis.md):
# P3 launch 시험 ④c (src/amr_fleet/test/test_agent_launch_partition.py) 를 한 팔(소스 트리)에서 N 번 되풀이한다.
# 컨테이너 안: 소스 /src (읽기 전용), 산출 /out. amr_msgs · amr_fleet 만 Release 로 /out/ws 에 빌드하고
# /out/run_<k>/ 에 원자료(events_*.jsonl · summary.json) · pytest.log · meta(시작/끝 loadavg · rc · 남은 프로세스).
# 끝난 칸(rc 파일 있음)은 건너뛴다 (재개). N 환경변수 (기본 4).
source /opt/ros/humble/setup.bash
B=/out/ws/build; I=/out/ws/install; L=/out/ws/log
export ROS_LOCALHOST_ONLY=1 PYTHONHASHSEED=0 PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-54}
la() { local a b c d; read -r a b c d < /proc/loadavg; echo "$a $b $c"; }
if [ ! -f $I/setup.bash ]; then
  (cd /src && MAKEFLAGS=-j4 colcon --log-base $L build --build-base $B --install-base $I \
    --packages-up-to amr_msgs amr_fleet --cmake-args -DCMAKE_BUILD_TYPE=Release) > /out/build.log 2>&1 || exit 1
fi
source $I/setup.bash
PAT='amr_fleet[.]agent'
kill_strays() {  # pgrep 으로 PID 를 확인한 뒤 그 PID 만 (자기 셸 제외)
  local p; p=$(pgrep -f "$PAT" | grep -vx "$$")
  [ -n "$p" ] && kill -9 $p 2>/dev/null
  return 0
}
for k in $(seq 1 ${N:-4}); do
  d=/out/run_$k
  [ -f $d/rc ] && continue
  rm -rf $d; mkdir -p $d
  pgrep -af "$PAT" > $d/strays_before.txt
  echo "strays_before $(wc -l < $d/strays_before.txt)" > $d/meta
  kill_strays
  echo "start $(date -Iseconds) load $(la)" >> $d/meta
  (cd /src/src/amr_fleet && AMR_AGENT_LAUNCH_RAW=$d/raw timeout 900 \
    python3 -m pytest -q -p no:cacheprovider test/test_agent_launch_partition.py > $d/pytest.log 2>&1)
  rc=$?
  echo "end $(date -Iseconds) load $(la) rc=$rc" >> $d/meta
  pgrep -af "$PAT" > $d/strays_after.txt
  echo "strays_after $(wc -l < $d/strays_after.txt)" >> $d/meta
  kill_strays
  echo $rc > $d/rc
done
