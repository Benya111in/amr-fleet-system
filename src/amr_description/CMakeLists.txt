cmake_minimum_required(VERSION 3.8)
project(amr_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 이 패키지는 launch/config/urdf 리소스와 스폰 도구 스크립트(scripts/gz_world.py)를 제공한다 (파이썬 모듈 없음).

# --- 리소스 디렉토리 설치 ---
install(DIRECTORY
  launch
  config
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# --- 실행 스크립트 (ros2 run amr_description gz_world.py ...) ---
install(PROGRAMS
  scripts/gz_world.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- 테스트 ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 라이선스는 package.xml 에 한 번 선언한다. 파일별 저작권 헤더 검사(ament_copyright)는 생략.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # xacro 전개 + URDF 파싱/정합 검사 (test/test_urdf.py). colcon test 가 install 공간을 소싱한 상태에서 돈다
  # ($(find amr_description) 해석). 최상위 config/ 는 테스트 파일 위치에서 찾는다.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(amr_description_pytest test TIMEOUT 300)
endif()

ament_package()
