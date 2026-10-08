cmake_minimum_required(VERSION 3.8)
project(amr_slam3d)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# --- Python 순수 모듈 (경로 추종·격자 변환·bag 통계·TF 트리 검사 — rclpy 비의존) ---
ament_python_install_package(${PROJECT_NAME})

# --- 실행 파일: 경로 주행·재생 측정·bag 도구·컨테이너 안 구동기 ---
install(PROGRAMS
  scripts/route_driver.py
  scripts/slam_probe.py
  scripts/anchor_tf.py
  scripts/bag_tools.py
  scripts/record_pilot.sh
  scripts/record_bag.sh
  scripts/make_routes.py
  scripts/replay_row.sh
  scripts/madg_lockstep.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 라이선스는 package.xml 에 한 번 선언한다. 파일별 저작권 헤더 검사(ament_copyright)는 생략.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  file(GLOB _pytests CONFIGURE_DEPENDS test/test_*.py)
  if(_pytests)
    ament_add_pytest_test(amr_slam3d_pytest test TIMEOUT 300)
  endif()
endif()

ament_package()
