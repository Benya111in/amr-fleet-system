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

# CONFIGURE_DEPENDS: 인터페이스 파일을 추가하면 재구성 없이 다음 빌드에 반영된다.
file(GLOB _msgs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS msg/*.msg)
file(GLOB _srvs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS srv/*.srv)
file(GLOB _acts RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS action/*.action)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${_msgs}
  ${_srvs}
  ${_acts}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 라이선스는 package.xml 에 한 번 선언한다. 파일별 저작권 헤더 검사(ament_copyright)는 생략.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
