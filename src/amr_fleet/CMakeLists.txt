cmake_minimum_required(VERSION 3.8)
project(amr_fleet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# --- Python 모듈 설치 ---
ament_python_install_package(${PROJECT_NAME})

# --- 리소스 디렉토리 설치 ---
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# --- 테스트 ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  file(GLOB _pytests test/test_*.py)
  if(_pytests)
    ament_add_pytest_test(amr_fleet_pytest test)
  endif()
endif()

ament_package()
