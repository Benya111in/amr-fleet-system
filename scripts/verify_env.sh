#!/usr/bin/env bash
# 컨테이너 안에서 실행하여 개발 환경이 실제로 동작하는지 검증한다.
#
#   docker compose exec dev ./scripts/verify_env.sh
#
# 명세가 요구하는 핵심 구성요소가 모두 import/실행 가능한지 확인한다.
# 단순 import 만으로는 "import 는 되지만 호출하면 죽는" 부류(numpy 2 ABI 붕괴,
# pytest 9 + launch_testing 플러그인 충돌 등)를 못 잡으므로 실제 호출까지 해 본다.
# 모든 검사는 자식 프로세스로 실행되므로 segfault(rc 139)도 FAIL 로 집계된다.

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

# 라벨에 붙일 버전 문자열 (조회 실패 시 "?")
pyver() {
    python3 -c "$1" 2>/dev/null || echo "?"
}

# 임시 디렉토리에 트리비얼 테스트를 만들어 pytest 로 실제 수집+실행한다.
# ROS 를 소싱한 상태이므로 launch_testing 의 pytest11 플러그인이 함께 로드된다 —
# pytest 9.x 는 이 플러그인(hookimpl 'path' 인자)에서 PluginValidationError 로 죽는다.
pytest_smoke() {
    local dir rc
    dir=$(mktemp -d) || return 1
    printf 'def test_smoke():\n    assert 1 + 1 == 2\n' > "${dir}/test_smoke.py"
    (cd "${dir}" && python3 -m pytest -q -p no:cacheprovider "$@" test_smoke.py)
    rc=$?
    rm -rf "${dir}"
    return "${rc}"
}

# ROS_DOMAIN_ID 가 0~101 정수인지 (미설정이면 ROS 기본값 0 으로 본다).
# .env 를 잘못 편집해도(자리표시자 그대로 등) compose 는 그 값을 그대로 넘기고, ros2 CLI/rmw 는
# 정수가 아닌 도메인에서 죽는다 — 여기서 먼저 잡는다.
domain_id_ok() {
    local v="${ROS_DOMAIN_ID:-0}"
    [[ "${v}" =~ ^[0-9]+$ ]] && [ "${v}" -le 101 ]
}

source /opt/ros/"${ROS_DISTRO:-humble}"/setup.bash

echo "=== ROS2 코어 ==="
check "ROS2 CLI (${ROS_DISTRO})" ros2 --help
check "rclpy import" python3 -c "import rclpy"
check "colcon" colcon --help

echo
echo "=== 컨테이너 환경변수 (.env → compose environment) ==="
check "ROS_DOMAIN_ID 는 0~101 정수 (현재: ${ROS_DOMAIN_ID:-미설정=0})" domain_id_ok

echo
echo "=== 시뮬레이터 ==="
check "Gazebo Fortress (ign gazebo)" ign gazebo --versions
check "ros_gz_bridge" ros2 pkg prefix ros_gz_bridge

echo
echo "=== 내비게이션 / 위치추정 ==="
for p in navigation2 nav2_bringup slam_toolbox robot_localization \
         nav2_costmap_2d nav2_amcl \
         nav2_navfn_planner nav2_smac_planner nav2_dwb_controller \
         nav2_regulated_pure_pursuit_controller nav2_collision_monitor; do
    check "${p}" ros2 pkg prefix "${p}"
done

echo
echo "=== Behavior Tree ==="
check "behaviortree_cpp" ros2 pkg prefix behaviortree_cpp
check "nav2_behavior_tree" ros2 pkg prefix nav2_behavior_tree

echo
echo "=== 모니터링 (명세 4.9) ==="
check "foxglove_bridge" ros2 pkg prefix foxglove_bridge

echo
echo "=== Python 수치 스택 (numpy 1.x ABI 정합) ==="
NUMPY_VER=$(pyver "import numpy; print(numpy.__version__)")
check "numpy < 2 (설치: ${NUMPY_VER})" python3 -c "
import numpy
assert int(numpy.__version__.split('.')[0]) < 2, numpy.__version__
"
# scipy 는 지원 범위 밖 numpy 에서 UserWarning 을 낸다 — 그 경고도 실패로 본다.
check "scipy.optimize.linear_sum_assignment" python3 -c "
import warnings
with warnings.catch_warnings(record=True) as w:
    warnings.simplefilter('always')
    from scipy.optimize import linear_sum_assignment
bad = [str(x.message) for x in w if 'NumPy version' in str(x.message)]
assert not bad, bad
r, c = linear_sum_assignment([[4, 1, 3], [2, 0, 5], [3, 2, 2]])
assert list(c) == [1, 0, 2], c
"
check "matplotlib (Agg 렌더링)" python3 -c "
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, ax = plt.subplots()
ax.plot([0, 1], [0, 1])
fig.canvas.draw()
"
check "pandas DataFrame" python3 -c "
import numpy as np, pandas as pd
df = pd.DataFrame({'a': np.arange(4)})
assert int(df['a'].sum()) == 6
"
T3D_VER=$(pyver "import transforms3d; print(transforms3d.__version__)")
check "tf_transformations (transforms3d ${T3D_VER})" python3 -c "
import tf_transformations as t
q = t.quaternion_from_euler(0.1, 0.2, 0.3)
assert abs(sum(x * x for x in q) - 1.0) < 1e-9, q
"
check "pip check (의존성 정합)" python3 -m pip check

echo
echo "=== 인지 (AI) ==="
TORCH_VER=$(pyver "import torch; print(torch.__version__)")
check "PyTorch import (${TORCH_VER})" python3 -c "import torch"
check "CUDA 사용 가능 + GPU matmul" python3 -c "
import torch
assert torch.cuda.is_available()
x = torch.randn(512, 512, device='cuda')
y = (x @ x).sum().item()
assert y == y  # NaN 이 아님
"
check "ultralytics (YOLOv8)" python3 -c "from ultralytics import YOLO"
CV2_VER=$(pyver "import cv2; print(cv2.__version__)")
check "OpenCV import (${CV2_VER})" python3 -c "import cv2"
# 합성 마커를 만들어 실제로 검출한다 (contrib 모듈 aruco 가 진짜 동작하는지).
check "cv2.aruco.ArucoDetector 검출" python3 -c "
import cv2, numpy as np
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
img = cv2.copyMakeBorder(cv2.aruco.generateImageMarker(d, 7, 200),
                         50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
corners, ids, _ = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters()).detectMarkers(img)
assert ids is not None and np.asarray(ids).ravel().tolist() == [7], ids
"
check "cv2.aruco.estimatePoseSingleMarkers" python3 -c "
import cv2, numpy as np
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
img = cv2.copyMakeBorder(cv2.aruco.generateImageMarker(d, 7, 200),
                         50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
corners, ids, _ = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters()).detectMarkers(img)
K = np.array([[300.0, 0, 150], [0, 300.0, 150], [0, 0, 1]])
rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, K, np.zeros(5))
assert rvecs.shape == (1, 1, 3) and tvecs.shape == (1, 1, 3) and np.isfinite(tvecs).all()
"
# 인코딩 변환(rgb8→bgr8)은 boost.python C 확장을 타므로 numpy ABI 불일치가 여기서 드러난다.
check "cv_bridge 왕복 (bgr8 / rgb8→bgr8 / 32FC1)" python3 -c "
import numpy as np
from cv_bridge import CvBridge
b = CvBridge()
img = np.random.default_rng(0).integers(0, 256, (48, 64, 3), dtype=np.uint8)
out = b.imgmsg_to_cv2(b.cv2_to_imgmsg(img, encoding='bgr8'), desired_encoding='bgr8')
assert out.dtype == np.uint8 and np.array_equal(out, img)
out = b.imgmsg_to_cv2(b.cv2_to_imgmsg(img, encoding='rgb8'), desired_encoding='bgr8')
assert np.array_equal(out, img[:, :, ::-1])
depth = np.random.default_rng(1).random((48, 64), dtype=np.float32)
out = b.imgmsg_to_cv2(b.cv2_to_imgmsg(depth, encoding='32FC1'), desired_encoding='32FC1')
assert out.dtype == np.float32 and np.array_equal(out, depth)
"

echo
echo "=== 테스트 도구 ==="
PYTEST_VER=$(pyver "import pytest; print(pytest.__version__)")
check "pytest-cov import" python3 -c "import pytest_cov"
check "pytest-timeout import" python3 -c "import pytest_timeout"
check "pytest ${PYTEST_VER} 수집+실행 (launch_testing 플러그인 포함)" pytest_smoke -p launch_testing
check "pytest --cov / --timeout 옵션 동작" pytest_smoke --cov=. --cov-report=term --timeout=60

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
