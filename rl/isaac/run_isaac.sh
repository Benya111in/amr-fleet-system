#!/usr/bin/env bash
# Isaac Lab 컨테이너에서 rl/isaac 스크립트를 돈다 (마운트·환경변수는 /home/CAPYI_AI/env/verify/verify_isaac.sh 그대로).
#   rl/isaac/run_isaac.sh <태그> <worktree 기준 스크립트 경로> [인자…]
# 예: rl/isaac/run_isaac.sh smoke rl/isaac/scripts/smoke_env.py --num_envs 256
# 에이전트 id AMR_AI_ID (기본 r7-isaac12): 컨테이너 이름 amr-ai-<id>-<태그> (--rm), 산출 루트
# /home/CAPYI_AI/runs/<id> 를 같은 경로로 마운트하고 컨테이너 안 AMR_ISAAC_RUNS 로 알린다 (자산 캐시 <루트>/assets).
# 시작 전 GPU 여유 ≥ MIN_FREE_MIB (기본 16000) 를 확인한다 (COORDINATION §3: Isaac ≤ 14 GB). 같은 에이전트의
# Isaac 프로세스를 하나 더 띄울 때는 호출하는 쪽이 자기 합계 ≤ 14 GB 를 확인하고 MIN_FREE_MIB 를 낮춘다.
set -uo pipefail
A=/home/CAPYI_AI
C=$A/cache/isaac
ID=${AMR_AI_ID:-r7-isaac12}
RUNS=$A/runs/$ID
WT=$(cd "$(dirname "$0")/../.." && pwd)
TAG=$1
SCRIPT=$2
shift 2
mkdir -p $C/{kit,ov,pip,glcache,computecache,logs,data,docs} "$RUNS"
FREE=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1)
MIN=${MIN_FREE_MIB:-16000}
echo "GPU free MiB: $FREE (need $MIN)"
[ "$FREE" -ge "$MIN" ] || { echo "SKIP: GPU 여유 부족 ($FREE MiB < $MIN)"; exit 3; }
exec docker run --rm --name amr-ai-$ID-$TAG --gpus all --shm-size=8g \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=N -e OMNI_KIT_ACCEPT_EULA=YES \
  -e ROS_LOCALHOST_ONLY=1 -e PYTHONUNBUFFERED=1 -e AMR_ISAAC_RUNS="$RUNS" ${EXTRA_DOCKER_ARGS:-} \
  -v $C/kit:/isaac-sim/kit/cache -v $C/ov:/root/.cache/ov -v $C/pip:/root/.cache/pip \
  -v $C/glcache:/root/.cache/nvidia/GLCache -v $C/computecache:/root/.nv/ComputeCache \
  -v $C/logs:/root/.nvidia-omniverse/logs -v $C/data:/root/.local/share/ov/data -v $C/docs:/root/Documents \
  -v "$WT":/workspace/amr -v "$RUNS":"$RUNS" \
  --entrypoint bash amr-ai/isaaclab:2.3.2 -lc \
  "cd /workspace/isaaclab && ./isaaclab.sh -p /workspace/amr/$SCRIPT $*"
