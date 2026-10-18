cmake_minimum_required(VERSION 3.8)
project(amr_nav_research)

# DWA 논문 보강 실험용 연구 사본 (docs/research/paper-research/contract.md §1).
# 원본 amr_navigation 은 고치지 않는다 — 코어 라이브러리(amr_navigation_core)를 링크해서 쓰고,
# DwaPlanner·DWAController·chain_sim 만 사본(…X)으로 둔다.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(amr_navigation REQUIRED)
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

# --- 연구 코어 (ROS 비의존): DwaPlannerX + 재설정 기록·목표 판정 도구 ---
# 원본 코어는 imported target 하나만 링크한다. ament_target_dependencies(amr_navigation) 는
# amr_navigation_plugins 까지 링크하므로 쓰지 않는다 (ros_utils_x.hpp 참고).
add_library(amr_nav_research_core SHARED
  src/dwa_x.cpp
  src/research_io.cpp
  src/chain_model.cpp
)
target_include_directories(amr_nav_research_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(amr_nav_research_core amr_navigation::amr_navigation_core)

# --- Nav2 플러그인 amr_nav_research::DwaControllerX ---
add_library(amr_nav_research_plugins SHARED
  src/ros_utils_x.cpp
  src/dwa_controller_x.cpp
)
target_link_libraries(amr_nav_research_plugins amr_nav_research_core)
ament_target_dependencies(amr_nav_research_plugins ${ROS_DEPS})
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# --- 도구: chain_sim 사본 ---
add_executable(chain_sim_x tools/chain_sim_x.cpp)
target_link_libraries(chain_sim_x amr_nav_research_core)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS amr_nav_research_core amr_nav_research_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS chain_sim_x DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/compare_chain_jsonl.py scripts/run_equiv_chain.py scripts/run_equiv_x.py
  DESTINATION lib/${PROJECT_NAME})

# --- 테스트 ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 원본 amr_navigation 과 같은 정책: 파일별 저작권 머리 검사는 빼고 나머지 린터는 그대로
  set(ament_cmake_copyright_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-legal/copyright")

  find_package(ament_cmake_gtest REQUIRED)
  # 1) 기본값 = 원본 코어 (무작위·대표 입력 결과 전체 비교 + 원본 golden 해시)
  ament_add_gtest(test_dwa_x_equivalence test/test_dwa_x_equivalence.cpp TIMEOUT 300)
  target_link_libraries(test_dwa_x_equivalence amr_nav_research_core)
  # 2) 연구 스위치 단위 시험 (R1 지속, R2 저크 띠, R3 후보, stateful 잠금, 기록)
  ament_add_gtest(test_dwa_x_research test/test_dwa_x_research.cpp TIMEOUT 120)
  target_link_libraries(test_dwa_x_research amr_nav_research_core)
  # 2b) 2 라운드 스위치 (F1 사슬 모형 재설정, F2 목표 연장, F3 막힌 경로 우회)
  ament_add_gtest(test_dwa_x_round2 test/test_dwa_x_round2.cpp TIMEOUT 300)
  target_link_libraries(test_dwa_x_round2 amr_nav_research_core)
  # 2c) 3 라운드 스위치 (G1 가속 국면 재설정 유예)
  ament_add_gtest(test_dwa_x_round3 test/test_dwa_x_round3.cpp TIMEOUT 300)
  target_link_libraries(test_dwa_x_round3 amr_nav_research_core)
  # 3) 플러그인: 원본 DWAController 와 DwaControllerX 를 **pluginlib 으로 한 프로세스에** 올려
  #    (시험대의 controller_server 와 같은 상황) 같은 입력에 같은 명령을 내는지 본다.
  #    플러그인 라이브러리는 링크하지 않는다 — 링크하면 class_loader 등록이 엉킨다.
  ament_add_gtest(test_dwa_controller_x test/test_dwa_controller_x.cpp TIMEOUT 120
    APPEND_ENV AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX})
  target_link_libraries(test_dwa_controller_x amr_navigation::amr_navigation_core)
  ament_target_dependencies(test_dwa_controller_x ${ROS_DEPS})
endif()

ament_export_include_directories(include)
ament_export_libraries(amr_nav_research_core amr_nav_research_plugins)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(amr_navigation ${ROS_DEPS})
ament_package()
