#!/usr/bin/env python3
"""
안전 게이트 Gazebo 시험 월드 생성기.

warehouse.sdf 에서 동적 장애물(actor 6, 지게차·셔틀)을 빼고 시험용 장애물을 더한다 (tracking.md §9.4).
정적 구조(랙, 0.60 m 좁은 통로, 도크 마커, 벽)는 그대로다.

    python3 safety_test_world.py <share>/amr_simulation/worlds/warehouse.sdf /tmp/warehouse.sdf

추가 모델 (world 좌표 = map 좌표, 계약 C4)
  obs_person     원기둥 r 0.20, 높이 1.70 — LiDAR 평면(지면 +0.20)에 보이는 사람 대역
  obs_box_low    상자 0.40 × 0.40 × 0.15 — LiDAR 평면 아래 (깊이 카메라만 본다)
  obs_forklift   model://forklift 정적 (포크 z 0.05~0.10, 끝 = 모델 x +2.0) — 포크 끝이 lane 쪽
두 이동체는 중력 없는 운동학 물체다: VelocityControl(/model/<이름>/cmd_vel) 로 움직이고
OdometryPublisher(/model/<이름>/odometry, 50 Hz) 가 지면 진실을 낸다. 자세는 set_pose 로 옮긴다.
출력 파일 이름의 월드 이름은 warehouse 로 둔다 (스폰 런치가 /world/warehouse/create 를 부른다).
"""

import re
import sys

MOVERS = {
    # 이름: (기하 SDF, 중심 높이 z, 주차 위치 x, y)
    'obs_person': ('<cylinder><radius>0.20</radius><length>1.70</length></cylinder>', 0.852,
                   -6.0, 19.0),
    'obs_box_low': ('<box><size>0.40 0.40 0.15</size></box>', 0.077, -7.0, 19.0),
}
FORKLIFT_POSE = (8.0, 17.5, 3.14159265)   # 포크 끝 x = 8.0 - 2.0 = 6.0, 포크 중심 y = 17.5 ± 0.30


def mover_sdf(name: str, geometry: str, z: float, x: float, y: float) -> str:
    return f"""    <model name="{name}">
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <gravity>false</gravity>
        <inertial><mass>20.0</mass>
          <inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="collision"><geometry>{geometry}</geometry></collision>
        <visual name="visual"><geometry>{geometry}</geometry>
          <material><ambient>0.8 0.4 0.1 1</ambient><diffuse>0.8 0.4 0.1 1</diffuse></material>
        </visual>
      </link>
      <plugin filename="ignition-gazebo-velocity-control-system"
              name="ignition::gazebo::systems::VelocityControl">
        <topic>/model/{name}/cmd_vel</topic>
      </plugin>
      <plugin filename="ignition-gazebo-odometry-publisher-system"
              name="ignition::gazebo::systems::OdometryPublisher">
        <odom_frame>world</odom_frame>
        <robot_base_frame>{name}</robot_base_frame>
        <odom_topic>/model/{name}/odometry</odom_topic>
        <tf_topic>/model/{name}/odometry/tf</tf_topic>
        <odom_publish_frequency>50</odom_publish_frequency>
        <dimensions>2</dimensions>
      </plugin>
    </model>
"""


def build(text: str) -> str:
    """warehouse.sdf 문자열 → 시험 월드 문자열."""
    text = re.sub(r'\s*<actor name="[^"]*">.*?</actor>', '', text, flags=re.S)
    # 지게차·셔틀 (include 안에 플러그인이 있다): 주석 줄부터 그 include 끝까지
    for name in ('forklift_main', 'shuttle_amr'):
        text = re.sub(r'\s*<!-- ' + name + r':.*?</include>', '', text, flags=re.S)
    extra = '    <!-- ===== 안전 게이트 시험 장애물 (safety_test_world.py) ===== -->\n'
    for name, (geom, z, x, y) in MOVERS.items():
        extra += mover_sdf(name, geom, z, x, y)
    fx, fy, fyaw = FORKLIFT_POSE
    extra += f"""    <include>
      <uri>model://forklift</uri>
      <name>obs_forklift</name>
      <static>true</static>
      <pose>{fx} {fy} 0 0 0 {fyaw}</pose>
    </include>
"""
    return text.replace('  </world>', extra + '  </world>', 1)


def main(argv=None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 2:
        print(__doc__)
        return 2
    with open(args[0], encoding='utf-8') as f:
        text = build(f.read())
    with open(args[1], 'w', encoding='utf-8') as f:
        f.write(text)
    print(f'시험 월드: {args[1]} (actor {text.count("<actor ")} 개 남음)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
