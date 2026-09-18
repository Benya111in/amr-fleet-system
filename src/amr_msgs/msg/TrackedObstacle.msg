# 칼만 필터 기반 동적 장애물 추적 결과 (명세 7장)
std_msgs/Header header

uint32 track_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity      # 추정 속도 [m/s]
float32 heading                     # 이동 방향 [rad]
float32 confidence                  # 추적 신뢰도 0~1
bool is_dynamic                     # 동적/정적 분류 결과
float32 time_to_collision           # TTC [s], 충돌 없으면 inf
