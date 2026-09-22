cmake_minimum_required(VERSION 3.8)
project(amr_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# --- Python 모듈 설치 (actor 궤적 보간, 발자국 기하, 지면 진실/충돌 판정 노드, 런타임 적재물) ---
ament_python_install_package(${PROJECT_NAME})

# --- 실행 스크립트 (ros2 run amr_simulation <이름>) ---
install(PROGRAMS
  scripts/obstacle_truth_node.py
  scripts/collision_monitor_node.py
  scripts/payload_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- 리소스 디렉토리 설치 ---
install(DIRECTORY
  launch
  config
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# --- 테스트 ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 라이선스는 package.xml 에 한 번 선언한다. 파일별 저작권 헤더 검사(ament_copyright)는 생략.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # 월드 재생성 == 커밋된 SDF, actor 궤적 보간, 발자국 부호 거리, 런타임 적재물 (test/test_*.py).
  # test_payload_gazebo.py 는 Gazebo(ign) 가 있을 때 물리 전용 월드에서 실제 적재·가감속을 확인한다 (~1 분)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(amr_simulation_pytest test TIMEOUT 600)
endif()

ament_package()
