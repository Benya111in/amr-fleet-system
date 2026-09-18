# YOLOv8 인식 결과 + Pinhole 모델로 변환한 3D 위치 (명세 6장)
std_msgs/Header header

string class_name          # "box", "person", "sign" 등 3종 이상
uint32 class_id
float32 confidence

# 2D 바운딩 박스 (픽셀)
uint32 bbox_x
uint32 bbox_y
uint32 bbox_width
uint32 bbox_height

geometry_msgs/PoseStamped pose_3d   # map frame 3D 좌표
float32 distance                    # 로봇 기준 거리 [m]
