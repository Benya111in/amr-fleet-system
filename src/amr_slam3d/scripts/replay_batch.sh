#!/usr/bin/env bash
# 매핑 행 재생 여러 건을 컨테이너로 동시에 돌린다 (호스트에서, 보너스 2 재생 보강 — Gazebo 없음).
#
#   replay_batch.sh <worktree> <bag 디렉터리> <출력 루트> <동시 수> <행:재생번호>...
#     예) replay_batch.sh /home/CAPYI_AI/wt/b2-offline /runs/derive/bag_madg /runs/replay 4 liosam_planar:0 rtabmap3d_noimu:0
#
# 행마다 rows.py 의 이미지로 `replay_row.sh <행> /data/bag /out 0.5` 를 돈다. 출력 = <출력 루트>/<행>/r<번호>/.
# 컨테이너는 기본 브리지 네트워크 + ROS_LOCALHOST_ONLY=1 이라 컨테이너끼리 DDS 가 섞이지 않는다 (같은 DOMAIN 이어도).
# 실행마다 출력 디렉터리에 HEAD.txt·status.txt·host.log 를 남긴다. 공유 서버 규칙: 이 스크립트는 cpu_slot.sh 안에서 부른다.
# 환경: DOMAIN(기본 45), RATE(기본 0.5), AGENT(기본 b2-offline — 컨테이너 이름 접두어 amr-ai-<AGENT>-).
set -uo pipefail
WT=${1:?worktree}; BAG=${2:?bag}; ROOT=${3:?출력 루트}; PAR=${4:?동시 수}; shift 4
DOMAIN=${DOMAIN:-45}; RATE=${RATE:-0.5}; AGENT=${AGENT:-b2-offline}
HEAD=$(git -C "$WT" rev-parse HEAD)
DIRTY=$(git -C "$WT" status --porcelain -- src | wc -l)

one() {
  local spec=$1 row k img out name rc
  row=${spec%%:*}; k=${spec##*:}
  img=$(PYTHONPATH="$WT/src/amr_slam3d" python3 -c "from amr_slam3d import rows; print(rows.row('$row').image)") || return 2
  out="$ROOT/$row/r$k"
  if [ -e "$out/env.txt" ]; then echo "이미 있다: $out (건너뜀)"; return 0; fi
  mkdir -p "$out"
  echo "$HEAD" > "$out/HEAD.txt"
  git -C "$WT" status --porcelain > "$out/status.txt"
  name="amr-ai-$AGENT-${row//_/-}-r$k"
  echo "$(date -Iseconds) START $row r$k image=$img load=$(cut -d' ' -f1-3 /proc/loadavg)" | tee -a "$ROOT/batch.log"
  docker run --rm --name "$name" --shm-size=2g \
    -e ROS_DOMAIN_ID="$DOMAIN" -e ROS_LOCALHOST_ONLY=1 -e GIT_HEAD="$HEAD" -e IMAGE="$img" \
    -e REPLAY_ID="B2off-$row-r$k" -v "$WT":/ros2_ws:ro -v "$BAG":/data/bag:ro -v "$out":/out -w /ros2_ws \
    "$img" bash -lc "/ros2_ws/src/amr_slam3d/scripts/replay_row.sh $row /data/bag /out $RATE" \
    > "$out/host.log" 2>&1
  rc=$?
  echo "RC=$rc" >> "$out/host.log"
  echo "$(date -Iseconds) END $row r$k rc=$rc load=$(cut -d' ' -f1-3 /proc/loadavg)" | tee -a "$ROOT/batch.log"
  return $rc
}

mkdir -p "$ROOT"
echo "$(date -Iseconds) BATCH head=$HEAD dirty_src=$DIRTY par=$PAR rate=$RATE specs=$*" >> "$ROOT/batch.log"
running=0
for spec in "$@"; do
  one "$spec" &
  running=$((running + 1))
  if [ "$running" -ge "$PAR" ]; then wait -n; running=$((running - 1)); fi
  sleep 5                                  # 기동 겹침을 조금 푼다
done
wait
echo "$(date -Iseconds) BATCH-END load=$(cut -d' ' -f1-3 /proc/loadavg)" >> "$ROOT/batch.log"
