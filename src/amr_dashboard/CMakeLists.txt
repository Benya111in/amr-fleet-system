cmake_minimum_required(VERSION 3.8)
project(amr_dashboard)

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
  web
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
    ament_add_pytest_test(amr_dashboard_pytest test)
  endif()
endif()

ament_package()
