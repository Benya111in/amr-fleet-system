cmake_minimum_required(VERSION 3.8)
project(amr_lidar3d)

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
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- 순수 로직 (rclcpp 비의존, gtest 대상): 비유한 점 제거 ---
add_library(${PROJECT_NAME}_core STATIC src/cloud_filter.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core sensor_msgs)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- rclcpp 컴포넌트 amr_lidar3d::CloudPreprocessor + 실행 파일 cloud_preprocessor_node ---
# 컴포넌트로 SLAM 노드와 같은 컨테이너에 올리면 intra-process 로 넘긴다 (docs/research/slam3d/sensor.md)
add_library(${PROJECT_NAME}_component SHARED src/cloud_preprocessor.cpp)
target_link_libraries(${PROJECT_NAME}_component ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "amr_lidar3d::CloudPreprocessor"
  EXECUTABLE cloud_preprocessor_node
)

install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# --- Python 순수 모듈 (점군 해석, 정답 면 거리) + 측정 스크립트 ---
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/lidar3d_probe.py
  scripts/lidar3d_sensor_check.py
  scripts/make_static_world.py
  scripts/run_lidar3d_bench.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 라이선스는 package.xml 에 한 번 선언한다. 파일별 저작권 헤더 검사(ament_copyright)는 생략.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  file(GLOB _pytests CONFIGURE_DEPENDS test/test_*.py)
  if(_pytests)
    ament_add_pytest_test(amr_lidar3d_pytest test TIMEOUT 300)
  endif()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(amr_lidar3d_gtest test/test_cloud_filter.cpp TIMEOUT 120)
  if(TARGET amr_lidar3d_gtest)
    target_link_libraries(amr_lidar3d_gtest ${PROJECT_NAME}_core)
  endif()

  # rclcpp 통합: lazy 구독·실제 토픽 출력
  ament_add_gtest(amr_lidar3d_ros_gtest test/test_preprocessor_ros.cpp TIMEOUT 120)
  if(TARGET amr_lidar3d_ros_gtest)
    target_link_libraries(amr_lidar3d_ros_gtest ${PROJECT_NAME}_component)
    ament_target_dependencies(amr_lidar3d_ros_gtest rclcpp sensor_msgs)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs)
ament_package()
