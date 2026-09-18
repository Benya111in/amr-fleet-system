# JSON Task Description 에 대응하는 작업 정의 (명세 8장)
std_msgs/Header header

string task_id
string robot_id                     # 미할당 시 빈 문자열
uint8 priority                      # 0(낮음) ~ 255(높음)
builtin_interfaces/Time deadline

geometry_msgs/PoseStamped pickup_pose
geometry_msgs/PoseStamped dropoff_pose

string item_type                    # "small", "medium", "large"
float32 item_mass                   # [kg] — 적재 시 동역학 반영

uint8 status

uint8 STATUS_PENDING=0
uint8 STATUS_IN_PROGRESS=1
uint8 STATUS_COMPLETED=2
uint8 STATUS_FAILED=3
