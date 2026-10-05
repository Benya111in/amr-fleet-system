cmake_minimum_required(VERSION 3.8)
project(amr_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# --- 핵심 알고리즘 라이브러리 (ROS 비의존, gtest 대상) ---
# 클러스터링·칼만 필터·Hungarian 연관·TTC·안전 게이트·깊이 점군. 노드는 얇은 래퍼다.
add_library(${PROJECT_NAME}_core STATIC
  src/geometry2d.cpp
  src/hungarian.cpp
  src/kalman_filter.cpp
  src/scan_clustering.cpp
  src/obstacle_tracker.cpp
  src/ttc.cpp
  src/safety_gate.cpp
  src/depth_cloud.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Eigen3::Eigen 은 IMPORTED 타깃이라 헤더가 SYSTEM 으로 포함된다 (-Wpedantic 경고 격리)
target_link_libraries(${PROJECT_NAME}_core PUBLIC Eigen3::Eigen)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- C++ 노드 (라이브러리 + 얇은 main) ---
# 노드 클래스는 팩토리(include/amr_perception/nodes.hpp)로 노출해 실행 파일과 rclcpp 통합 테스트가 공유한다.
set(_node_deps
  rclcpp amr_msgs diagnostic_msgs geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs tf2 tf2_ros
  visualization_msgs)
add_library(${PROJECT_NAME}_nodes STATIC
  src/obstacle_tracker_node.cpp
  src/safety_node.cpp
  src/pointcloud_filter_node.cpp
)
target_link_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_nodes ${_node_deps})

foreach(_node obstacle_tracker_node safety_node pointcloud_filter_node)
  add_executable(${_node} src/main_${_node}.cpp)
  target_link_libraries(${_node} ${PROJECT_NAME}_nodes)
  ament_target_dependencies(${_node} rclcpp)
endforeach()

install(TARGETS
  obstacle_tracker_node
  safety_node
  pointcloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

# --- Python 모듈 설치 ---
ament_python_install_package(${PROJECT_NAME})

# --- Python 노드/도구 실행 파일 (ros2 run amr_perception <name>) ---
install(PROGRAMS
  scripts/yolo_node
  scripts/object_localizer_node
  scripts/detection_marker_node
  scripts/aruco_detector_node
  scripts/generate_dataset.py
  scripts/train.py
  scripts/yolo_fps_probe.py
  scripts/tracker_scenario.py
  scripts/safety_scenario.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- 리소스 디렉토리 설치 ---
install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

# --- 테스트 ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 라이선스는 package.xml 에 한 번 선언한다. 파일별 저작권 헤더 검사(ament_copyright)는 생략.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  # CONFIGURE_DEPENDS: 테스트 파일을 추가하면 재구성 없이 다음 빌드에 반영된다.
  file(GLOB _pytests CONFIGURE_DEPENDS test/test_*.py)
  if(_pytests)
    # ament 는 --cov=<소스 디렉토리> 만 준다. 테스트가 import 하는 것은 설치된 패키지라서
    # --symlink-install 이 아니면 모듈이 0 % 로 잡힌다 → import 한 패키지도 측정한다.
    ament_add_pytest_test(amr_perception_pytest test
      ENV "PYTEST_ADDOPTS=--cov=amr_perception")
  endif()

  # C++ 핵심 알고리즘 단위 테스트 (gtest). 커버리지: --cmake-args -DCMAKE_CXX_FLAGS="--coverage"
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(amr_perception_gtest
    test/test_geometry.cpp
    test/test_hungarian.cpp
    test/test_kalman_filter.cpp
    test/test_scan_clustering.cpp
    test/test_obstacle_tracker.cpp
    test/test_ttc.cpp
    test/test_safety_gate.cpp
    test/test_depth_cloud.cpp
    TIMEOUT 300
  )
  if(TARGET amr_perception_gtest)
    target_link_libraries(amr_perception_gtest ${PROJECT_NAME}_core)
  endif()

  # C++ 노드 rclcpp 통합 테스트: 실제 토픽/QoS/TF/서비스로 노드를 구동해 출력 확인
  ament_add_gtest(amr_perception_ros_gtest test/test_ros_nodes.cpp TIMEOUT 300)
  if(TARGET amr_perception_ros_gtest)
    target_link_libraries(amr_perception_ros_gtest ${PROJECT_NAME}_nodes)
    ament_target_dependencies(amr_perception_ros_gtest ${_node_deps})
  endif()
endif()

ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_package()
