#!/usr/bin/env bash
# 컨테이너 안에서 실행하여 개발 환경이 실제로 동작하는지 검증한다.
#
#   docker compose exec dev ./scripts/verify_env.sh
#
# 명세가 요구하는 핵심 구성요소가 모두 import/실행 가능한지 확인한다.

PASS=0
FAIL=0

check() {
    local label="$1"; shift
    if "$@" >/dev/null 2>&1; then
        printf "  \033[1;32m[OK]\033[0m   %s\n" "${label}"
        PASS=$((PASS + 1))
    else
        printf "  \033[1;31m[FAIL]\033[0m %s\n" "${label}"
        FAIL=$((FAIL + 1))
    fi
}

source /opt/ros/"${ROS_DISTRO}"/setup.bash

echo "=== ROS2 코어 ==="
check "ROS2 CLI (${ROS_DISTRO})" ros2 --help
check "rclpy import" python3 -c "import rclpy"
check "colcon" colcon --help

echo
echo "=== 시뮬레이터 ==="
check "Gazebo Fortress (ign gazebo)" ign gazebo --versions
check "ros_gz_bridge" ros2 pkg prefix ros_gz_bridge

echo
echo "=== 내비게이션 / 위치추정 ==="
for p in navigation2 nav2_bringup slam_toolbox robot_localization \
         nav2_costmap_2d nav2_amcl; do
    check "${p}" ros2 pkg prefix "${p}"
done

echo
echo "=== Behavior Tree ==="
check "behaviortree_cpp" ros2 pkg prefix behaviortree_cpp
check "nav2_behavior_tree" ros2 pkg prefix nav2_behavior_tree

echo
echo "=== 인지 (AI) ==="
check "PyTorch import" python3 -c "import torch"
check "CUDA 사용 가능" python3 -c "import torch; assert torch.cuda.is_available()"
check "ultralytics (YOLOv8)" python3 -c "import ultralytics"
check "cv_bridge" python3 -c "import cv_bridge"
check "OpenCV + ArUco" python3 -c "import cv2; cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)"

echo
echo "=== 테스트 도구 ==="
check "pytest" python3 -m pytest --version
check "pytest-cov" python3 -c "import pytest_cov"

echo
echo "=== GPU 상세 ==="
python3 - <<'PY' 2>/dev/null || echo "  GPU 정보 조회 실패"
import torch
if torch.cuda.is_available():
    print(f"  device       : {torch.cuda.get_device_name(0)}")
    print(f"  capability   : sm_{''.join(map(str, torch.cuda.get_device_capability(0)))}")
    print(f"  torch        : {torch.__version__}")
    print(f"  torch cuda   : {torch.version.cuda}")
    x = torch.randn(1000, 1000, device='cuda')
    print(f"  matmul 검증  : {(x @ x).shape} OK")
else:
    print("  CUDA 사용 불가")
PY

echo
echo "============================================"
printf "결과: \033[1;32m%d개 통과\033[0m / \033[1;31m%d개 실패\033[0m\n" "${PASS}" "${FAIL}"
echo "============================================"
exit $((FAIL > 0 ? 1 : 0))
