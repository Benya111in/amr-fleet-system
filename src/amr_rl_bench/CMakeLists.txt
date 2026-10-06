cmake_minimum_required(VERSION 3.8)
project(amr_rl_bench)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# 순수 모듈 (world_geom · scenarios · metrics · records) — rclpy 비의존
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/gzbench_runner.py
  scripts/run_bench.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 라이선스는 package.xml 에 한 번 (저장소 관례: 파일별 저작권 헤더 검사 생략)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  get_filename_component(REPO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
  file(GLOB _pytests CONFIGURE_DEPENDS test/test_*.py)
  if(_pytests)
    ament_add_pytest_test(amr_rl_bench_pytest test
      ENV REPO_DIR=${REPO_DIR})
  endif()
endif()

ament_package()
