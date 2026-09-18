# Fleet 대시보드용 로봇 상태 (명세 9장)
std_msgs/Header header

string robot_id
geometry_msgs/PoseStamped pose
float32 battery_level               # 0~100 [%]
string current_task_id
uint8 status                        # 아래 상수 참조

uint8 STATUS_IDLE=0
uint8 STATUS_MOVING=1
uint8 STATUS_DOCKING=2
uint8 STATUS_LOADING=3
uint8 STATUS_CHARGING=4
uint8 STATUS_ERROR=5
uint8 STATUS_ESTOP=6
