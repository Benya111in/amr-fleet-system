# 시스템 전체 KPI (명세 9장 모니터링)
std_msgs/Header header

RobotState[] robots
uint32 tasks_pending
uint32 tasks_in_progress
uint32 tasks_completed
uint32 tasks_failed

float32 throughput                  # 작업 처리량 [tasks/hour]
float32 avg_task_duration           # 평균 작업 시간 [s]
float32 robot_utilization           # 로봇 가동률 0~1
uint32 deadlock_count
