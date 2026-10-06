"""
보너스 1 Gazebo 다회 에피소드 평가 하네스 (A5, 완료 정의 D4·D5).

rclpy 에 의존하지 않는 순수 모듈 (world_geom · scenarios · metrics · records) 과, 그것을 쓰는 ROS 실행기
(scripts/gzbench_runner.py) 로 나뉜다. 순수 모듈은 pytest 로 바로 시험한다 (kidnap_detector.py 와 같은 구조).
"""
