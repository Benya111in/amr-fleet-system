cmake_minimum_required(VERSION 3.8)
project(amr_localization)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# --- 순수 알고리즘 라이브러리 (ROS 비의존 → gtest 로 단위 검증) ---
# 순기구학·원호 적분, 인코더 모델, 공분산 전파, IMU LPF·바이어스, 스캔 필터
add_library(${PROJECT_NAME}_core SHARED
  src/diff_drive_kinematics.cpp
  src/encoder_model.cpp
  src/imu_bias_estimator.cpp
  src/imu_filter.cpp
  src/low_pass_filter.cpp
  src/odometry_covariance.cpp
  src/scan_filter.cpp
  src/wheel_odometry.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)

# --- 노드 (얇은 ROS 래퍼) ---
set(_node_deps rclcpp geometry_msgs nav_msgs sensor_msgs std_srvs tf2_ros)
foreach(_node wheel_odometry_node imu_filter_node scan_filter_node)
  add_executable(${_node} src/${_node}.cpp)
  target_link_libraries(${_node} ${PROJECT_NAME}_core)
  ament_target_dependencies(${_node} ${_node_deps})
endforeach()

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS wheel_odometry_node imu_filter_node scan_filter_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# --- Python 모듈 설치 ---
ament_python_install_package(${PROJECT_NAME})

# --- Python 노드/CLI 실행 파일 (ros2 run amr_localization <name>) ---
install(PROGRAMS
  scripts/amcl_map_adapter
  scripts/kidnap_monitor_node
  scripts/odom_drift_experiment
  scripts/drift_report
  scripts/map_quality
  scripts/world_to_map
  DESTINATION lib/${PROJECT_NAME}
)

# --- 리소스 디렉토리 설치 ---
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# --- 테스트 ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 라이선스는 package.xml 에 한 번 선언한다. 파일별 저작권 헤더 검사(ament_copyright)는 생략.
  set(ament_cmake_copyright_FOUND TRUE)
  # cpplint 도 자체적으로 파일 머리 저작권 줄(legal/copyright)을 검사한다 → 같은 정책으로 그 규칙만 끄고
  # 나머지 Google 스타일 검사는 그대로 돌린다.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-legal/copyright")

  find_package(ament_cmake_pytest REQUIRED)
  # CONFIGURE_DEPENDS: 테스트 파일을 추가하면 재구성 없이 다음 빌드에 반영된다.
  file(GLOB _pytests CONFIGURE_DEPENDS test/test_*.py)
  if(_pytests)
    # 광선 투사·전역 가설 탐색·Monte-Carlo 검증이 있어 기본 60 s 보다 길게 (고부하 호스트 여유)
    ament_add_pytest_test(amr_localization_pytest test TIMEOUT 900)
  endif()

  # C++ 단위 테스트 (알고리즘 라이브러리)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(_test
      test_diff_drive_kinematics
      test_encoder_model
      test_odometry_covariance
      test_wheel_odometry
      test_low_pass_filter
      test_imu_filter
      test_scan_filter)
    ament_add_gtest(${_test} test/${_test}.cpp TIMEOUT 120)
    target_link_libraries(${_test} ${PROJECT_NAME}_core)
  endforeach()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_package()
