cmake_minimum_required(VERSION 3.8)
project(amr_behavior)

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
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Python 모듈 설치 ---
ament_python_install_package(${PROJECT_NAME})

# --- C++ 라이브러리 ---
set(_bt_deps rclcpp rclcpp_action behaviortree_cpp_v3 nav2_msgs amr_msgs geometry_msgs)

# 순수 알고리즘 (ROS 비의존): 도킹 제어기, TTC 트리거
add_library(amr_behavior_algorithms SHARED
  src/docking/docking_controller.cpp
  src/ttc/ttc_trigger.cpp
)
target_include_directories(amr_behavior_algorithms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 작업 BT 코어: 실행기 컨텍스트, 노드 등록 표, 트리 생성
add_library(amr_behavior_bt SHARED
  src/executor_context.cpp
  src/bt_registry.cpp
  src/task_tree.cpp
)
target_include_directories(amr_behavior_bt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amr_behavior_bt ${_bt_deps})

# 노드 구현 (실행 파일과 통합 테스트가 공유)
add_library(amr_task_executor SHARED src/task_executor_node.cpp)
target_link_libraries(amr_task_executor amr_behavior_bt)
ament_target_dependencies(amr_task_executor ${_bt_deps} ament_index_cpp sensor_msgs std_msgs std_srvs)

add_library(amr_docking_server SHARED src/docking/docking_server_node.cpp)
target_link_libraries(amr_docking_server amr_behavior_algorithms)
ament_target_dependencies(amr_docking_server
  rclcpp rclcpp_action amr_msgs geometry_msgs std_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs)

# Nav2 BT 조건 플러그인 (bt_navigator 의 plugin_lib_names 로 로드, BT_REGISTER_NODES)
add_library(amr_is_ttc_below_threshold_condition_bt_node SHARED
  src/plugins/is_ttc_below_threshold_condition.cpp)
target_link_libraries(amr_is_ttc_below_threshold_condition_bt_node amr_behavior_algorithms)
ament_target_dependencies(amr_is_ttc_below_threshold_condition_bt_node
  rclcpp behaviortree_cpp_v3 amr_msgs)
target_compile_definitions(amr_is_ttc_below_threshold_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)

# --- 실행 파일 ---
add_executable(task_executor_node src/task_executor_main.cpp)
target_link_libraries(task_executor_node amr_task_executor)

add_executable(docking_server_node src/docking/docking_server_main.cpp)
target_link_libraries(docking_server_node amr_docking_server)

add_executable(bt_tool src/bt_tool.cpp)
target_link_libraries(bt_tool amr_behavior_bt)
ament_target_dependencies(bt_tool ament_index_cpp)

install(TARGETS
  amr_behavior_algorithms
  amr_behavior_bt
  amr_task_executor
  amr_docking_server
  amr_is_ttc_below_threshold_condition_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS task_executor_node docking_server_node bt_tool
  DESTINATION lib/${PROJECT_NAME})
# 시뮬레이션 배터리 (battery_state 발행, amr_behavior/battery_model_node.py)
install(PROGRAMS scripts/battery_model_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

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
  # 같은 정책으로 cpplint 의 legal/copyright 항목만 끄고 나머지 검사는 그대로 돌린다
  # (자동 등록에서 빼고 필터를 붙여 직접 등록).
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-legal/copyright")

  find_package(ament_cmake_pytest REQUIRED)
  # CONFIGURE_DEPENDS: 테스트 파일을 추가하면 재구성 없이 다음 빌드에 반영된다.
  file(GLOB _pytests CONFIGURE_DEPENDS test/test_*.py)
  if(_pytests)
    # 교차 패키지·다중 프로세스 시험이 빌드된 실행 파일을 직접 띄운다 (설치 전)
    ament_add_pytest_test(amr_behavior_pytest test TIMEOUT 300
      ENV
      "DOCKING_SERVER_EXE=$<TARGET_FILE:docking_server_node>"
      "TASK_EXECUTOR_EXE=$<TARGET_FILE:task_executor_node>")
  endif()

  find_package(ament_cmake_gtest REQUIRED)
  # 트리 XML 은 소스 트리에서 읽는다 (설치 전 테스트)
  set(_src_def "AMR_BEHAVIOR_SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"")

  ament_add_gtest(test_docking_controller test/test_docking_controller.cpp)
  target_link_libraries(test_docking_controller amr_behavior_algorithms)

  ament_add_gtest(test_marker_relocalization test/test_marker_relocalization.cpp)
  target_link_libraries(test_marker_relocalization amr_behavior_algorithms)

  ament_add_gtest(test_ttc_trigger test/test_ttc_trigger.cpp)
  target_link_libraries(test_ttc_trigger amr_behavior_algorithms
    amr_is_ttc_below_threshold_condition_bt_node)
  ament_target_dependencies(test_ttc_trigger rclcpp behaviortree_cpp_v3 amr_msgs)
  target_compile_definitions(test_ttc_trigger PRIVATE
    AMR_TTC_PLUGIN_PATH="$<TARGET_FILE:amr_is_ttc_below_threshold_condition_bt_node>")

  ament_add_gtest(test_executor_context test/test_executor_context.cpp)
  target_link_libraries(test_executor_context amr_behavior_bt)

  ament_add_gtest(test_bt_nodes test/test_bt_nodes.cpp)
  target_link_libraries(test_bt_nodes amr_behavior_bt)

  ament_add_gtest(test_task_tree test/test_task_tree.cpp TIMEOUT 120)
  target_link_libraries(test_task_tree amr_behavior_bt)
  target_compile_definitions(test_task_tree PRIVATE ${_src_def})

  ament_add_gtest(test_ros_bt_nodes test/test_ros_bt_nodes.cpp TIMEOUT 120)
  target_link_libraries(test_ros_bt_nodes amr_behavior_bt)

  ament_add_gtest(test_docking_server test/test_docking_server.cpp TIMEOUT 120)
  target_link_libraries(test_docking_server amr_docking_server)

  ament_add_gtest(test_task_executor_node test/test_task_executor_node.cpp TIMEOUT 120)
  target_link_libraries(test_task_executor_node amr_task_executor)
  target_compile_definitions(test_task_executor_node PRIVATE ${_src_def})
endif()

ament_export_include_directories(include)
ament_export_libraries(amr_behavior_algorithms amr_behavior_bt)
ament_export_dependencies(${_bt_deps})
ament_package()
