cmake_minimum_required(VERSION 3.8)
project(amr_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

file(GLOB _msgs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
file(GLOB _srvs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*.srv)
file(GLOB _acts RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*.action)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${_msgs}
  ${_srvs}
  ${_acts}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
