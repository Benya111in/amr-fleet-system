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
    && rm -rf /var/lib/apt/lists/*

# --- BehaviorTree.CPP + Groot2 연동 (명세 8장) ------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-behaviortree-cpp \
        ros-humble-behaviortree-cpp-v3 \
        ros-humble-nav2-behavior-tree \
        libzmq3-dev \
    && rm -rf /var/lib/apt/lists/*

# --- 테스트/커버리지 도구 (명세 10장: 커버리지 70% 이상) --------------------
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
RUN pip3 install --no-cache-dir --upgrade pip wheel \
    && pip3 install --no-cache-dir --ignore-installed sympy mpmath \
    && pip3 install --no-cache-dir \
        torch torchvision --index-url https://download.pytorch.org/whl/cu128

RUN pip3 install --no-cache-dir \
        ultralytics \
        opencv-python-headless \
        numpy'<2' \
        scipy \
        matplotlib \
        pandas \
        transforms3d \
        pytest pytest-cov pytest-timeout \
        flask flask-socketio eventlet \
        pyyaml jsonschema

# --- 도킹용 ArUco (명세 8장 도킹 시스템) ------------------------------------
RUN pip3 install --no-cache-dir opencv-contrib-python-headless

# --- 비루트 사용자 ----------------------------------------------------------
RUN groupadd -g ${GID} ${USERNAME} 2>/dev/null || true \
    && useradd -u ${UID} -g ${GID} -m -s /bin/bash ${USERNAME} 2>/dev/null || true \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# --- 워크스페이스 -----------------------------------------------------------
ENV ROS_WS=/ros2_ws
RUN mkdir -p ${ROS_WS}/src && chown -R ${UID}:${GID} ${ROS_WS}
WORKDIR ${ROS_WS}

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
