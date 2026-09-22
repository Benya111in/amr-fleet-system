cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(amr_msgs REQUIRED)

set(ROS_DEPS
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  amr_msgs
)

# --- Python 모듈 설치 ---
ament_python_install_package(${PROJECT_NAME})

# --- 코어 알고리즘 라이브러리 (ROS 비의존: A*, 평활화, DWA, VO, Pure Pursuit, 프로파일러, PID) ---
add_library(amr_navigation_core SHARED
  src/core/grid.cpp
  src/core/geometry.cpp
  src/core/astar.cpp
  src/core/path_smoother.cpp
  src/core/footprint.cpp
  src/core/velocity_obstacle.cpp
  src/core/dwa.cpp
  src/core/speed_profile.cpp
  src/core/pure_pursuit.cpp
  src/core/jerk_limiter.cpp
  src/core/pid.cpp
  src/core/velocity_profiler.cpp
)
target_include_directories(amr_navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Nav2 플러그인 (AStarPlanner, DWAController, PurePursuitController) ---
add_library(amr_navigation_plugins SHARED
  src/ros/ros_utils.cpp
  src/ros/astar_planner.cpp
  src/ros/dwa_controller.cpp
  src/ros/pure_pursuit_controller.cpp
)
target_link_libraries(amr_navigation_plugins amr_navigation_core)
ament_target_dependencies(amr_navigation_plugins ${ROS_DEPS})
pluginlib_export_plugin_description_file(nav2_core plugins/nav2_plugins.xml)

# --- velocity_profiler_node ---
add_library(amr_velocity_profiler SHARED src/nodes/velocity_profiler_node.cpp)
target_link_libraries(amr_velocity_profiler amr_navigation_core)
ament_target_dependencies(amr_velocity_profiler rclcpp nav_msgs geometry_msgs std_msgs)
add_executable(velocity_profiler_node src/nodes/velocity_profiler_main.cpp)
target_link_libraries(velocity_profiler_node amr_velocity_profiler)
ament_target_dependencies(velocity_profiler_node rclcpp)

# --- 도구 (ROS 비의존): PID 게인 격자 탐색, A* 벤치마크, 폐루프 추종 시뮬레이터 ---
add_executable(pid_tuning tools/pid_tuning.cpp)
target_link_libraries(pid_tuning amr_navigation_core)
add_executable(astar_benchmark tools/astar_benchmark.cpp)
target_link_libraries(astar_benchmark amr_navigation_core)
add_executable(tracking_sim tools/tracking_sim.cpp)
target_link_libraries(tracking_sim amr_navigation_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS amr_navigation_core amr_navigation_plugins amr_velocity_profiler
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS velocity_profiler_node pid_tuning astar_benchmark tracking_sim
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/bench_planners.py
  scripts/kinematic_sim.py
  scripts/closed_loop_eval.py
  scripts/make_warehouse_map.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- 리소스 디렉토리 설치 ---
install(DIRECTORY
  launch
  config
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# --- 테스트 ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 라이선스는 package.xml 에 한 번 선언한다. 파일별 저작권 헤더 검사(ament_copyright)는 생략.
  set(ament_cmake_copyright_FOUND TRUE)
  # cpplint 도 자체적으로 파일 머리 저작권 줄(legal/copyright)을 검사한다 → 같은 정책으로 그 규칙만 끄고
  # 나머지 Google 스타일 검사(줄 길이는 한글 폭 2 로 계산)는 그대로 돌린다. (amr_localization 과 같은 방식)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-legal/copyright")

  find_package(ament_cmake_pytest REQUIRED)
  # CONFIGURE_DEPENDS: 테스트 파일을 추가하면 재구성 없이 다음 빌드에 반영된다.
  file(GLOB _pytests CONFIGURE_DEPENDS test/test_*.py)
  if(_pytests)
    ament_add_pytest_test(amr_navigation_pytest test)
  endif()

  # C++ 단위 테스트 (gtest): 코어 모듈별 1 바이너리 + 플러그인/노드 통합 1 바이너리
  find_package(ament_cmake_gtest REQUIRED)
  foreach(_t grid astar geometry path_smoother collision dwa pure_pursuit profiler)
    ament_add_gtest(test_${_t} test/test_${_t}.cpp TIMEOUT 120)
    target_link_libraries(test_${_t} amr_navigation_core)
  endforeach()
  ament_add_gtest(test_ros_plugins test/test_ros_plugins.cpp TIMEOUT 120
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_ros_plugins amr_navigation_plugins amr_velocity_profiler)
  ament_target_dependencies(test_ros_plugins ${ROS_DEPS})
endif()

ament_export_include_directories(include)
ament_export_libraries(amr_navigation_core amr_navigation_plugins)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${ROS_DEPS})
ament_package()
