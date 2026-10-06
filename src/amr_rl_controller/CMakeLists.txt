cmake_minimum_required(VERSION 3.8)
project(amr_rl_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(amr_navigation REQUIRED)

set(ROS_DEPS
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  amr_msgs
  amr_navigation
)

# 저장소 루트의 RL 계약 (golden 벡터·기준 구현). rl/COLCON_IGNORE 라 패키지가 아니다.
get_filename_component(RL_CONTRACT_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../rl/contract" ABSOLUTE)

# 파이썬 도우미 (런치의 nav2 파라미터 덧씌우기)
ament_python_install_package(${PROJECT_NAME})

# --- ROS·ORT 비의존 코어: 계약 관측·마스크·적분·결정 엔진 ---
add_library(amr_rl_core SHARED
  src/core/geometry.cpp
  src/core/observation.cpp
  src/core/kinematics.cpp
  src/core/mask.cpp
  src/core/decision_engine.cpp
)
target_include_directories(amr_rl_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(amr_rl_core amr_navigation::amr_navigation_core)

# --- ONNX Runtime C++ (rosdep 키 없음 → ONNXRUNTIME_ROOT, amr-ai/rl:0.3 은 /opt/onnxruntime) ---
if(NOT DEFINED ONNXRUNTIME_ROOT)
  set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}")
endif()
find_path(ORT_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS "${ONNXRUNTIME_ROOT}/include" /opt/onnxruntime/include
  PATH_SUFFIXES onnxruntime)
find_library(ORT_LIBRARY onnxruntime
  HINTS "${ONNXRUNTIME_ROOT}/lib" /opt/onnxruntime/lib)
if(ORT_INCLUDE_DIR AND ORT_LIBRARY)
  set(HAVE_ORT TRUE)
  get_filename_component(ORT_LIB_DIR "${ORT_LIBRARY}" DIRECTORY)
  message(STATUS "amr_rl_controller: ONNX Runtime ${ORT_LIBRARY}")
else()
  set(HAVE_ORT FALSE)
  message(STATUS "amr_rl_controller: ONNX Runtime 없음 → 코어만 빌드 (플러그인 생략)")
endif()

if(HAVE_ORT)
  add_library(amr_rl_policy SHARED src/policy/onnx_policy.cpp)
  target_include_directories(amr_rl_policy PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  # 경고 검사 대상이 아니도록 SYSTEM (CI 는 -Werror)
  target_include_directories(amr_rl_policy SYSTEM PUBLIC ${ORT_INCLUDE_DIR})
  target_link_libraries(amr_rl_policy ${ORT_LIBRARY})
  set_target_properties(amr_rl_policy PROPERTIES
    BUILD_RPATH "${ORT_LIB_DIR}" INSTALL_RPATH "${ORT_LIB_DIR}")

  add_library(amr_rl_controller SHARED src/ros/rl_controller.cpp)
  target_link_libraries(amr_rl_controller amr_rl_core amr_rl_policy
    amr_navigation::amr_navigation_plugins)
  ament_target_dependencies(amr_rl_controller ${ROS_DEPS})
  set_target_properties(amr_rl_controller PROPERTIES
    BUILD_RPATH "${ORT_LIB_DIR}" INSTALL_RPATH "${ORT_LIB_DIR}")
  pluginlib_export_plugin_description_file(nav2_core plugins.xml)

  install(TARGETS amr_rl_policy amr_rl_controller
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  # 추론 지연 측정 (ROS 비의존): golden 상태로 관측·마스크·ORT 를 반복 실행
  find_package(nlohmann_json 3 QUIET)
  if(nlohmann_json_FOUND)
    add_executable(latency_bench tools/latency_bench.cpp)
    target_link_libraries(latency_bench amr_rl_core amr_rl_policy nlohmann_json::nlohmann_json)
    target_include_directories(latency_bench PRIVATE test)
    target_compile_definitions(latency_bench PRIVATE RL_CONTRACT_DIR="${RL_CONTRACT_DIR}")
    set_target_properties(latency_bench PROPERTIES
      BUILD_RPATH "${ORT_LIB_DIR}" INSTALL_RPATH "${ORT_LIB_DIR}")
    install(TARGETS latency_bench DESTINATION lib/${PROJECT_NAME})
  endif()
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS amr_rl_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/rl_smoke.py
  test/make_test_models.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 저작권 헤더 검사는 저장소 관례대로 생략 (package.xml 에 라이선스 한 번)
  set(ament_cmake_copyright_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-legal/copyright")

  find_package(ament_cmake_gtest REQUIRED)
  find_package(nlohmann_json 3 REQUIRED)
  # golden 벡터 대조 (관측 1e-5, 마스크·셀 판정 정확 일치, VO 시각 1e-4, 적분 1e-9)
  ament_add_gtest(test_contract_golden test/test_contract_golden.cpp TIMEOUT 120)
  target_link_libraries(test_contract_golden amr_rl_core nlohmann_json::nlohmann_json)
  target_compile_definitions(test_contract_golden PRIVATE RL_CONTRACT_DIR="${RL_CONTRACT_DIR}")
  ament_add_gtest(test_decision_engine test/test_decision_engine.cpp TIMEOUT 120)
  target_link_libraries(test_decision_engine amr_rl_core nlohmann_json::nlohmann_json)
  target_compile_definitions(test_decision_engine PRIVATE RL_CONTRACT_DIR="${RL_CONTRACT_DIR}")

  find_package(ament_cmake_pytest REQUIRED)
  # 계약 상수 대조·nav2 파라미터 덧씌우기 (test/test_*.py 자동 수집)
  ament_add_pytest_test(amr_rl_controller_pytest test
    ENV RL_CONTRACT_DIR=${RL_CONTRACT_DIR} PKG_DIR=${CMAKE_CURRENT_SOURCE_DIR})

  if(HAVE_ORT)
    # 고정물 ONNX 는 커밋하지 않는다 (.gitignore *.onnx) → 시험 시점에 torch 로 만든다
    set(FIXTURE_DIR "${CMAKE_CURRENT_BINARY_DIR}/onnx_fixtures")
    ament_add_gtest(test_onnx_policy test/test_onnx_policy.cpp TIMEOUT 300)
    target_link_libraries(test_onnx_policy amr_rl_core amr_rl_policy
      nlohmann_json::nlohmann_json)
    target_include_directories(test_onnx_policy SYSTEM PRIVATE ${ORT_INCLUDE_DIR})
    target_compile_definitions(test_onnx_policy PRIVATE
      RL_CONTRACT_DIR="${RL_CONTRACT_DIR}"
      FIXTURE_DIR="${FIXTURE_DIR}"
      MAKE_MODELS="${CMAKE_CURRENT_SOURCE_DIR}/test/make_test_models.py")
    set_target_properties(test_onnx_policy PROPERTIES BUILD_RPATH "${ORT_LIB_DIR}")
    ament_add_gtest(test_rl_controller_plugin test/test_rl_controller_plugin.cpp TIMEOUT 300)
    target_link_libraries(test_rl_controller_plugin amr_rl_controller)
    ament_target_dependencies(test_rl_controller_plugin ${ROS_DEPS})
    target_compile_definitions(test_rl_controller_plugin PRIVATE
      FIXTURE_DIR="${FIXTURE_DIR}"
      MAKE_MODELS="${CMAKE_CURRENT_SOURCE_DIR}/test/make_test_models.py"
      RL_CONTRACT_DIR="${RL_CONTRACT_DIR}")
    set_target_properties(test_rl_controller_plugin PROPERTIES BUILD_RPATH "${ORT_LIB_DIR}")
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(amr_rl_core)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(amr_navigation)
ament_package()
