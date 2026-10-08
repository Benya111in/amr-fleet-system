#!/bin/bash
# 회귀 묶음 (b4-wire): 새 build/install(/out/ws) 에서 Release -Werror 빌드 + colcon test, xunit XML 을 남긴다.
# 컨테이너 안에서 돈다 (저장소를 /ros2_ws 로, 산출 디렉터리를 /out 으로 마운트).
#   BUILD_PKGS="amr_msgs amr_fleet amr_behavior amr_bringup" TEST_PKGS="amr_fleet amr_behavior amr_bringup" \
#     bash /ros2_ws/docs/research/fleet-distributed/tools/regress.sh
# 결과: /out/build.log · build_warnings.txt · test.log · <pkg>_test_results/ · status.txt
# 판정은 tools/xunit_summary.py 가 XML 로 한다 (colcon test-result 의 로그 경로 문제를 피한다).
# (set -u 는 ROS setup.bash 와 맞지 않는다)
source /opt/ros/humble/setup.bash
cd /ros2_ws
OUT=/out
B=/out/ws/build; I=/out/ws/install; L=/out/ws/log
export ROS_LOCALHOST_ONLY=1 PYTHONHASHSEED=0 PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-51}
rm -rf /out/ws
echo "start $(date -Iseconds) load $(cut -d' ' -f1-3 /proc/loadavg)" > $OUT/status.txt
MAKEFLAGS=-j4 colcon --log-base $L build --build-base $B --install-base $I --parallel-workers 4 \
  --packages-up-to ${BUILD_PKGS} \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-Werror > $OUT/build.log 2>&1
brc=$?
echo "build rc=$brc $(date -Iseconds)" >> $OUT/status.txt
grep -iE "warning:" $OUT/build.log > $OUT/build_warnings.txt
if [ $brc -ne 0 ]; then echo "빌드 실패" >> $OUT/status.txt; exit $brc; fi
source $I/setup.bash
colcon --log-base $L test --build-base $B --install-base $I --packages-select ${TEST_PKGS} \
  --event-handlers console_direct- > $OUT/test.log 2>&1
echo "test rc=$? $(date -Iseconds) load $(cut -d' ' -f1-3 /proc/loadavg)" >> $OUT/status.txt
for p in ${TEST_PKGS}; do
  rm -rf $OUT/${p}_test_results
  cp -r $B/$p/test_results $OUT/${p}_test_results 2>/dev/null
done
rm -rf /out/ws
