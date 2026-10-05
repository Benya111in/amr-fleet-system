std_msgs/Header header
TrackedObstacle[] obstacles

# 이 스캔의 TTC 가 로봇의 **계획 경로**를 따라가는 모형으로 계산됐는가.
# false 면 직선 외삽(속도 방향) 또는 정지 모형이라, time_to_collision 은
# "내 경로 위에서 만나는가" 가 아니라 다른 양이다 — 상위가 그 구분 없이 쓰면 안 된다.
# (obstacle_tracker_node 가 RobotMotionModel::followsPath() 를 그대로 싣는다.)
bool ttc_path_based
