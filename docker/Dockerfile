FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=amr
ARG UID=1000
ARG GID=1000

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# --- 기본 개발 도구 ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git curl wget gnupg2 lsb-release sudo \
        python3-pip python3-dev python3-venv \
        vim nano tmux htop iputils-ping net-tools \
        libeigen3-dev libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# --- Gazebo Fortress (명세 6장: Gazebo Fortress 또는 Isaac Sim) --------------
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ignition-fortress \
        ros-humble-ros-gz \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-image \
    && rm -rf /var/lib/apt/lists/*

# --- ROS2 스택: Nav2, SLAM, 로컬라이제이션, TF, 시각화 ----------------------
# foxglove-bridge: 명세 4.9 모니터링 — Foxglove Studio 를 실시간(WebSocket)으로 연결한다.
# (rosbag 재생만으로는 라이브 모니터링이 안 된다.)
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-cartographer-ros \
        ros-humble-robot-localization \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-xacro \
        ros-humble-urdf \
        ros-humble-tf2-tools \
        ros-humble-tf-transformations \
        ros-humble-rviz2 \
        ros-humble-rqt-common-plugins \
        ros-humble-rosbag2 \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-vision-opencv \
        ros-humble-laser-filters \
        ros-humble-pointcloud-to-laserscan \
        ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# --- BehaviorTree.CPP + Groot2 연동 (명세 4.8) ------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-behaviortree-cpp \
        ros-humble-behaviortree-cpp-v3 \
        ros-humble-nav2-behavior-tree \
        libzmq3-dev \
    && rm -rf /var/lib/apt/lists/*

# --- 테스트/커버리지 도구 (명세 4.10: 커버리지 70% 이상) --------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ament-cmake-gtest \
        ros-humble-ament-lint-auto \
        ros-humble-ament-lint-common \
        python3-colcon-common-extensions \
        python3-colcon-coveragepy-result \
        python3-colcon-lcov-result \
        lcov gcovr \
    && rm -rf /var/lib/apt/lists/*

# --- Python: YOLOv8 + PyTorch (CUDA 12.8 / RTX 5090 Blackwell sm_120) -------
# RTX 5090은 compute capability 12.0 이므로 cu128 이상 휠이 필요하다.
# ros:humble 이미지의 sympy/mpmath 는 distutils 로 설치되어 있어 pip 이 제거하지 못한다.
# --ignore-installed 로 해당 패키지만 덮어쓴다.
# 버전은 고정한다 — 미고정 시 빌드 시점마다 다른 torch 가 잡혀 이미지가 재현되지 않는다.
RUN pip3 install --no-cache-dir --upgrade pip wheel \
    && pip3 install --no-cache-dir --ignore-installed sympy==1.14.0 mpmath==1.3.0 \
    && pip3 install --no-cache-dir \
        torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu128

# --- Python 과학/테스트/웹 스택 (전부 버전 고정 — 재현 가능한 이미지) ---------
# * numpy 1.x 필수: cv_bridge(boost.python), apt matplotlib 등 ROS 측 C 확장이 numpy 1.x
#   ABI 로 빌드되어 있어 numpy 2 에서는 import/변환이 깨진다 (_ARRAY_API not found).
# * opencv-python: ultralytics 의 하드 의존성이라 제거할 수 없다. 실제 cv2 바이너리는
#   아래 마지막 단계에서 같은 버전의 contrib-headless 로 덮어쓴다 (동일 버전 → 설정 파일 동일).
#   opencv-python-headless 는 셋이 같은 cv2/ 를 두고 겹치므로 설치하지 않는다.
# * scipy: apt 1.8.0 은 numpy>=1.25 에서 "unsupported NumPy" 경고를 내므로 pip 휠로 올린다.
#   matplotlib/pyyaml 은 apt 버전(3.5.1 / 5.4.1)이 numpy 1.26 과 호환되어 그대로 쓴다
#   (pip 은 동일 버전이 이미 있으면 건너뛴다).
# * pytest 8.x: pytest 9 는 launch_testing 의 pytest 플러그인(hookimpl 'path' 인자)과
#   충돌해 ROS 를 소싱한 상태에서는 pytest 자체가 뜨지 않는다.
# * transforms3d 0.4.2: apt 0.3.1 은 numpy>=1.24 에서 import 실패(np.float 제거) →
#   tf_transformations 가 깨진다. apt 판은 sympy 처럼 distutils egg-info 라 pip 이 제거하지
#   못하므로 --ignore-installed --no-deps 로 먼저 덮어쓴다 (/usr/local 이 sys.path 에서 앞선다).
RUN pip3 install --no-cache-dir --no-deps --ignore-installed transforms3d==0.4.2 \
    && pip3 install --no-cache-dir \
        numpy==1.26.4 \
        opencv-python==4.11.0.86 \
        ultralytics==8.4.155 \
        transforms3d==0.4.2 \
        scipy==1.15.3 \
        matplotlib==3.5.1 \
        pandas==2.3.3 \
        pytest==8.3.5 pytest-cov==7.1.0 pytest-timeout==2.4.0 \
        flask==3.1.3 flask-socketio==5.6.1 eventlet==0.41.2 \
        pyyaml==5.4.1 jsonschema==4.26.0 \
    && pip3 install --no-cache-dir --no-deps --force-reinstall \
        opencv-contrib-python-headless==4.11.0.86
# 마지막 줄 = 도킹용 ArUco (명세 4.8). contrib 모듈(aruco) + headless(rviz/rqt 의 시스템 Qt 와
# 번들 Qt 충돌 방지). opencv-python 과 같은 cv2/ 디렉토리에 풀리므로 반드시 마지막에
# --force-reinstall 로 설치해 contrib-headless 의 cv2.abi3.so 가 결정적으로 이기게 한다.
# --no-deps: 위에서 고정한 numpy 등을 건드리지 않는다.

# --- 비루트 사용자 ----------------------------------------------------------
RUN groupadd -g ${GID} ${USERNAME} 2>/dev/null || true \
    && useradd -u ${UID} -g ${GID} -m -s /bin/bash ${USERNAME} 2>/dev/null || true \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# --- 워크스페이스 -----------------------------------------------------------
ENV ROS_WS=/ros2_ws
RUN mkdir -p ${ROS_WS}/src && chown -R ${UID}:${GID} ${ROS_WS}
WORKDIR ${ROS_WS}

# Gazebo Fortress 리소스 경로 — 프로젝트 월드/모델. ENV 라서 엔트리포인트를 거치지 않는
# `docker compose exec dev bash` 셸에서도 적용된다. (런치 파일은 install/share 경로를 덧붙인다)
ENV IGN_GAZEBO_RESOURCE_PATH=${ROS_WS}/src/amr_simulation/worlds:${ROS_WS}/src/amr_simulation/models
ENV GZ_SIM_RESOURCE_PATH=${IGN_GAZEBO_RESOURCE_PATH}

# 대화형 셸(docker compose exec dev bash)은 엔트리포인트를 타지 않으므로 여기서 ROS 를 소싱한다.
# /etc/bash.bashrc 는 모든 사용자의 대화형 bash 가 읽는다. install/ 은 빌드 후에만 존재한다.
RUN printf '%s\n' \
        '' \
        '# ROS2 + 워크스페이스 오버레이 (amr-fleet-system)' \
        'source /opt/ros/${ROS_DISTRO}/setup.bash' \
        'if [ -f ${ROS_WS}/install/setup.bash ]; then source ${ROS_WS}/install/setup.bash;' \
        'else echo "[amr] ${ROS_WS}/install 이 아직 없다 — builder 가 끝난 뒤 새 셸을 열거나 source install/setup.bash"; fi' \
        >> /etc/bash.bashrc

# --- GPU 렌더링 (headless EGL) ----------------------------------------------
# nvidia-container-toolkit 은 libEGL_nvidia.so 는 주입하지만 이를 가리키는
# EGL ICD 파일(10_nvidia.json)은 주입하지 않는다. 이게 없으면 libglvnd 가
# Mesa 로 폴백하여 Gazebo(OGRE2)가 /dev/dri/card0(내장 GPU)을 잡고 segfault 한다.
RUN mkdir -p /usr/share/glvnd/egl_vendor.d \
    && printf '%s\n' \
        '{' \
        '    "file_format_version" : "1.0.0",' \
        '    "ICD" : {' \
        '        "library_path" : "libEGL_nvidia.so.0"' \
        '    }' \
        '}' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia
ENV QT_X11_NO_MITSHM=1
# OGRE2 가 NVIDIA EGL 디바이스를 선택하도록 강제한다.
ENV __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
ENV MESA_LOADER_DRIVER_OVERRIDE=""

# --- 엔트리포인트 -----------------------------------------------------------
COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
