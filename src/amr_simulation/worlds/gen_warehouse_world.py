#!/usr/bin/env python3
"""
warehouse.sdf 생성기 — 60 m x 40 m 물류센터 (명세 4.1 시뮬레이터 환경, 8장 레이아웃 예시).

    python3 gen_warehouse_world.py              # 같은 디렉토리의 warehouse.sdf 를 다시 쓴다
    python3 gen_warehouse_world.py out.sdf

생성물(warehouse.sdf)을 커밋하므로 빌드 시 실행할 필요는 없다. 레이아웃을 바꾸면 이 파일을 고치고 다시 생성한다.
colcon test 가 "다시 생성한 결과 == 커밋된 warehouse.sdf" 를 검사한다 (test/test_world.py). 의존성: 표준 라이브러리만.

좌표계
  원점 = 창고 중심, x = 60 m 축(+x 동쪽 = 출고구역), y = 40 m 축(+y 북쪽 = 도크/메인 통로), 바닥 z = 0.
  기본 스폰(0, 0, yaw 0)은 랙 B열-C열 사이 5 m 통로 한가운데다.

레이아웃 (명세 8장 예시를 따름; 단위 m)
  외벽 안쪽 면 x = ±30, y = ±20 (내부 60 x 40), 높이 8 m, 천장(시각체만) 8 m, 바닥판 64 x 44
  북쪽 y≈+15 : 메인 통로 (지게차 순찰 x -20..20)
  북서 (-27.5, 17/13): 입고 Dock-1/2,  북동 (27.5, 17/13): 출고 Dock-A/B  (마커는 벽면)
  y=+9 / +3 / -3 : 랙 A/B/C 열, 각 7베이 (x = -18, -12, ..., 18; 베이 2.0 x 1.0 x 2.5)
  (0, -10) : 좁은 통로 테스트 구간 — 랙 4베이를 90도 돌려 세워 순폭 0.60 m(로봇 폭 0.40 + 0.20), 길이 4 m, y 방향
  남서 (-22, -16) : 충전 구역 C1~C3 (남쪽 벽 앞, 스테이션마다 마커),  남동 (22, -16): 대기 구역
  기둥 8개: (±10, ±12), (±25, ±6)
  ArUco 마커 (DICT_4X4_50, 판 0.30 m, 흑백 영역 0.18 m, 중심 높이 MARKER_Z = 로봇 카메라 광학 중심 0.25 m):
    Dock-1/2/A/B = id 0~3, C1~C3 = id 4~6
  표지판 25개 (models/sign, SIGN_POSES): 도크·입출고 구역·충전·대기·랙 열(A/B/C) 끝면·기둥 주의·좁은 통로 주의·비상구

동적 장애물 8개 (0.3~1.5 m/s, 직선/곡선/무작위)
  worker_straight_slow  0.5 m/s  A-B 통로(y=6) x -16..16 왕복            직선
  worker_straight_fast  1.2 m/s  동측 통로(x=22) y -10..10 왕복          직선
  worker_curve          0.8 m/s  서측 공터 (-24, 0) 반지름 3.5 원 순환    곡선
  worker_random         0.6 m/s  통로 격자 위 무작위 보행 (seed 42)      무작위
  worker_crossing       1.0 m/s  좁은 통로 입구 앞(y=-7) x -6..6 횡단     직선 (명세 4.7 테스트)
  worker_slow_south     0.3 m/s  좁은 통로 남쪽 출구 앞(y=-14.5) x -6..6  직선 (명세 하한 속도)
  forklift_main         1.5 m/s  메인 통로(y=15) x -20..20 왕복, 끝에서 1.0 m/s² 로 1.5 s 감속·1.5 s 가속 (물리 모델)
  shuttle_amr           1.0 m/s  교차 통로(x=9) y -6..11 왕복, 0.8 m/s² 램프 — "다른 로봇" (물리 모델, AMR 크기)

사람 actor 궤적 (결정)
  경로 노드를 직선 구간으로 잇고 구간마다 정지→가속(WALK_ACCEL)→등속→감속→정지 사다리꼴 속도, 꺾이는 노드에서는 멈춰서
  TURN_RATE 로 제자리 회전한다 (급회전 전에 감속). 웨이포인트는 이 운동을 ACTOR_DT 간격으로 샘플한 것이다: Fortress 는
  웨이포인트를 Catmull-Rom Hermite(tension 0, 구간 매개변수 = 구간 안 시간 비율)로 잇는데, 등시간 간격이면 접선 =
  속도 × 간격 이라 구간 경계에서도 속도가 이어지고 등속 구간은 정확히 등속이다. 정지 직전 간격이 짧아 제자리 회전 시
  위치가 부푸는 양은 수 mm 이하다. 원 궤적은 등속 순환이다.
  "무작위" 패턴은 고정 seed 의 난수 보행으로 미리 만든 긴 비반복 웨이포인트 목록이다 (실행 중 난수 아님).
  Fortress 의 actor 는 서버(물리)에서 움직이지 않는다: 궤적은 렌더링 쪽(Sensors 시스템)이 sim time 으로 계산하므로
  로봇의 LiDAR/카메라에는 보이지만 /world/warehouse/pose/info 에는 actor 포즈가 없고 충돌체도 없다(로봇이 통과한다).
  → 지면 진실은 amr_simulation/obstacle_truth_node(같은 스플라인으로 SDF 궤적 보간)가 /sim/dynamic_obstacles 로
    발행하고, 충돌은 collision_monitor_node 가 발자국 기하로 판정한다.

지게차·셔틀 (결정: 물리 모델 + DiffDrive + TriggeredPublisher, ROS 노드 없이 월드만으로 동작)
  DiffDrive 의 가감속 제한이 속도를 램프로 바꾼다: 끝 지점 전 창(window)에 들어오면 TriggeredPublisher 가 반대 방향
  속도를 한 번 내고, 차량은 v²/(2a) 만큼 더 가서 멈춘 뒤 반대로 가속한다 → 반전 지점 = 창 시작 + v²/(2a).
  같은 창을 다시 지나며 같은 지령이 또 나가도 목표가 같아 영향이 없다. 처음 위치를 한쪽 끝 창 안에 두어 출발시킨다.
  몸체를 돌리지 않고 전진·후진으로 왕복한다(회전 누적 오차가 없어 수 시간 반복해도 경로가 유지된다).
  지면 진실: OdometryPublisher → gz /model/<이름>/odometry (월드 절대 자세 + 속도) → ROS /sim/<이름>/odom
  (warehouse.launch.py 가 브리지). PosePublisher(/model/<이름>/pose)는 트리거 입력용이다.
"""
import math
import os
import random
from collections import deque

SEED = 42                       # 무작위 보행/상자 배치 난수 seed
ACTOR_DT = 0.5                  # [s] actor 웨이포인트 시간 간격 (등시간 샘플, 머리말 참고)
WALK_ACCEL = 0.6                # [m/s^2] 보행 출발·정지 가감속 (성인 보행 0.5~1.0)
TURN_RATE = 2.0                 # [rad/s] 제자리 방향 전환 (90도 ≈ 0.8 s)

# ---- 치수 -------------------------------------------------------------------
HALF_X, HALF_Y = 30.0, 20.0     # 외벽 안쪽 면 = ±HALF_X, ±HALF_Y → 내부 60 x 40 m (명세 최소 크기)
WALL_T, WALL_H = 0.2, 8.0       # 벽 두께, 높이 (= 천장 높이. 실제 물류센터 8~10 m)
FLOOR_MARGIN = 2.0              # 바닥판을 벽 밖으로 더 까는 폭 (벽 아래 틈 방지)
# 바닥 마찰: 건조 콘크리트 vs 우레탄/고무 바퀴 μ≈0.6~0.85 (Engineering ToolBox, rubber-concrete dry)
FLOOR_MU = 0.8
# 바닥 알베도: 콘크리트 0.25~0.45 → 0.35. 이전 0.55 + 강한 태양광·천장 없음은 로봇 카메라 영상 하단 바닥이
# 241/255 (리뷰 실측)로 하얗게 떴다. 천장 + 약한 태양광 + 0.40 에서 184/255, 0.35 에서 173/255 (포화 픽셀 0.1 %)
# (실측 조건은 docs/architecture/components.md §3.1)
FLOOR_ALBEDO = 0.35
RACK_L, RACK_D, RACK_H = 2.0, 1.0, 2.5
ROWS = {"A": 9.0, "B": 3.0, "C": -3.0}
RACK_X = [-18.0, -12.0, -6.0, 0.0, 6.0, 12.0, 18.0]
NARROW_CLEAR = 0.60             # 로봇 폭 0.40 + 0.20 (명세 4.4)
NARROW_CX, NARROW_CY = 0.0, -10.0
DOCKS = {                       # 이름: (패드 중심 x, y, 종류, 마커 id)
    "dock_1": (-27.5, 17.0, "inbound", 0),
    "dock_2": (-27.5, 13.0, "inbound", 1),
    "dock_a": (27.5, 17.0, "outbound", 2),
    "dock_b": (27.5, 13.0, "outbound", 3),
}
# 이름: (x, 마커 id); y 는 CHARGER_Y
CHARGERS = {"c1": (-26.0, 4), "c2": (-22.0, 5), "c3": (-18.0, 6)}
CHARGER_Y = -18.4
STATION_FRONT = 0.25            # 충전 스테이션 본체 전면까지 (models/charging_station 0.5 x 0.4 박스의 반)
PILLARS = [(-10, 12), (10, 12), (-10, -12), (10, -12), (-25, 6), (25, 6), (-25, -6), (25, -6)]
PILLAR_HALF = 0.25
# 마커 판 중심 높이 = AMR 카메라 광학 중심의 지면 높이
#   = config/robot_params.yaml base_link_height 0.18 + config/sensors.yaml camera_link z 0.07
MARKER_Z = 0.25
DECK_H = 0.50                   # 랙 하단 적재 블록 상면 (models/rack, LiDAR 스캔 평면 지면 +0.20 m 를 덮음)
SHELF1_Z = 1.275                # 랙 1단 선반 상면 (선반 중심 1.25 + 두께/2)
PLATE_OFF = 0.012               # 벽/면에 붙이는 판(두께 0.02) 중심의 면 앞 거리

rng = random.Random(SEED)


def pose(x, y, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return f"{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}"


def include(uri, name, x, y, z=0.0, yaw=0.0, static=None, extra=""):
    st = f"\n      <static>{'true' if static else 'false'}</static>" if static is not None else ""
    return f"""    <include>
      <uri>model://{uri}</uri>
      <name>{name}</name>
      <pose>{pose(x, y, z, 0, 0, yaw)}</pose>{st}{extra}
    </include>
"""


# ---- 정적 구조물 ------------------------------------------------------------
def ground_and_walls():
    hx, hy = HALF_X + WALL_T / 2, HALF_Y + WALL_T / 2           # 벽 중심선 (안쪽 면 = ±HALF)
    lx, ly = 2 * (HALF_X + WALL_T), 2 * (HALF_Y + WALL_T)       # 모서리를 닫는 길이
    size = f"{2 * (HALF_X + FLOOR_MARGIN):.0f} {2 * (HALF_Y + FLOOR_MARGIN):.0f}"
    plane = f"<plane><normal>0 0 1</normal><size>{size}</size></plane>"
    a = FLOOR_ALBEDO
    return f"""    <!-- 바닥 {size.replace(' ', ' x ')} m (벽 밖까지). μ={FLOOR_MU}:
         건조 콘크리트-고무(우레탄) 바퀴 0.6~0.85 범위의 중간값 (Engineering ToolBox). 알베도 {a} (콘크리트) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>{plane}</geometry>
          <surface><friction><ode>
            <mu>{FLOOR_MU}</mu><mu2>{FLOOR_MU}</mu2>
          </ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry>{plane}</geometry>
          <material>
            <ambient>{a} {a} {a - 0.02:.2f} 1</ambient><diffuse>{a} {a} {a - 0.02:.2f} 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- 외벽 (두께 {WALL_T} m, 높이 {WALL_H} m),
         안쪽 면 x=±{HALF_X:.1f}, y=±{HALF_Y:.1f} → 내부 60 x 40 m -->
    <model name="warehouse_walls">
      <static>true</static>
      <link name="link">
""" + "".join(
        f"""        <collision name="{n}_collision">
          <pose>{pose(cx, cy, WALL_H / 2)}</pose>
          <geometry><box><size>{sx:.1f} {sy:.1f} {WALL_H}</size></box></geometry>
        </collision>
        <visual name="{n}">
          <pose>{pose(cx, cy, WALL_H / 2)}</pose>
          <geometry><box><size>{sx:.1f} {sy:.1f} {WALL_H}</size></box></geometry>
          <material>
            <ambient>0.72 0.72 0.70 1</ambient><diffuse>0.72 0.72 0.70 1</diffuse>
          </material>
        </visual>
"""
        for n, cx, cy, sx, sy in [
            ("wall_north", 0, hy, lx, WALL_T),
            ("wall_south", 0, -hy, lx, WALL_T),
            ("wall_east", hx, 0, WALL_T, ly),
            ("wall_west", -hx, 0, WALL_T, ly),
        ]
    ) + """      </link>
    </model>
"""


HIGHBAYS = [(x, y) for y in (15.0, 6.0, -6.0, -15.0) for x in (-18.0, -6.0, 6.0, 18.0)
            if not (y in (15.0, -15.0) and x in (-6.0, 6.0))]
HIGHBAY_Z = 7.0


def roof():
    """
    천장: 아래를 향한 평면 시각체(충돌 없음, 그림자 없음) + 하이베이 등기구.

    평면은 한쪽 면만 그려지므로 로봇 카메라(아래)에는 천장이 보이고, 위에서 내려다보는 GUI 카메라에는 보이지 않아
    창고 안이 계속 보인다. 하이베이 등기구(발광 판)를 그 아래에 단다.
    """
    lum = "".join(f"""        <visual name="luminaire_{i}">
          <pose>{pose(x, y, HIGHBAY_Z + 0.25)}</pose>
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>1.2 0.4 0.08</size></box></geometry>
          <material>
            <ambient>1 1 0.95 1</ambient><diffuse>1 1 0.95 1</diffuse>
            <emissive>0.9 0.9 0.85 1</emissive>
          </material>
        </visual>
""" for i, (x, y) in enumerate(HIGHBAYS))
    size = f"{2 * (HALF_X + WALL_T):.1f} {2 * (HALF_Y + WALL_T):.1f}"
    return f"""    <!-- 천장 (시각체만, 지면 +{WALL_H} m): 아래 방향 평면이라 위에서는 보이지 않는다 + 하이베이 등기구 -->
    <model name="roof">
      <static>true</static>
      <link name="link">
        <visual name="ceiling">
          <pose>{pose(0, 0, WALL_H)}</pose>
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 -1</normal><size>{size}</size></plane></geometry>
          <material>
            <ambient>0.45 0.46 0.48 1</ambient><diffuse>0.45 0.46 0.48 1</diffuse>
          </material>
        </visual>
{lum}      </link>
    </model>
"""


def lights():
    """
    약한 방향광(천창 채광 대용) + 통로 위 고천장 LED 포인트 조명.

    실제 물류센터는 7~10 m 천장에 LED 하이베이를 통로 축을 따라 배치한다(약 300 lx). 포인트 조명을 통로 위 z=7 m 에
    12개 두어 카메라 영상에 통로별 밝기 차이가 생기게 한다(마커 검출·도메인 랜덤화의 현실적 조건). 천장이 있으므로
    방향광은 천창 채광 정도로 낮춘다 (이전 diffuse 0.7 + ambient 0.35 는 바닥이 포화됐다).
    """
    out = """    <!-- 방향광(sun): 천창 채광 대용(약하게), 그림자 켬. 천장은 그림자를 드리우지 않아 빛을 가리지 않는다.
         highbay_*: 통로 축 위 z=7 m LED 하이베이 12개(실제 7~10 m 천장, 약 300 lx), 그림자 끔.
         visualize=false: Fortress 는 광원 표시 기호(녹색 십자/사각형)를 카메라 센서 영상에도 그린다
         (실측) → 로봇 카메라에 인공물이 찍히지 않게 끈다. -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <visualize>false</visualize>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.35 0.35 0.35 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
"""
    for n, (x, y) in enumerate(HIGHBAYS):
        out += f"""    <light type="point" name="highbay_{n}">
      <cast_shadows>false</cast_shadows>
      <visualize>false</visualize>
      <pose>{pose(x, y, HIGHBAY_Z)}</pose>
      <diffuse>0.75 0.75 0.70 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>25</range><constant>0.3</constant><linear>0.02</linear><quadratic>0.004</quadratic>
      </attenuation>
    </light>
"""
    return out


def rack_instances():
    """(이름, x, y, yaw) 목록: 열 A/B/C 7베이씩 + 좁은 통로 4베이."""
    racks = []
    for row, y in ROWS.items():
        for i, x in enumerate(RACK_X, 1):
            racks.append((f"rack_{row}{i}", x, y, 0.0))
    # 좁은 통로: 랙을 90도 돌려(길이 2.0 m 가 y 방향) 좌우 2베이씩 세운다. 안쪽 면 x = ±NARROW_CLEAR/2
    off = NARROW_CLEAR / 2 + RACK_D / 2
    for side, sx in (("W", -off), ("E", off)):
        for j, dy in enumerate((-1.0, 1.0), 1):
            racks.append((f"rack_narrow_{side}{j}", NARROW_CX + sx, NARROW_CY + dy, math.pi / 2))
    return racks


def rack_boxes(racks):
    """랙마다 하단 블록 위(DECK_H)에 대형/중형 1개씩, 1단 선반(SHELF1_Z)에 소형 2개 (정적)."""
    out = ""
    for name, rx, ry, yaw in racks:
        slots = [("box_large", -0.5, 0.0, DECK_H), ("box_medium", 0.5, 0.0, DECK_H),
                 ("box_small", -0.5, 0.0, SHELF1_Z), ("box_small", 0.5, 0.0, SHELF1_Z)]
        for k, (kind, lx, ly, lz) in enumerate(slots):
            lx += rng.uniform(-0.05, 0.05)
            byaw = yaw + rng.uniform(-0.15, 0.15)
            wx = rx + lx * math.cos(yaw) - ly * math.sin(yaw)
            wy = ry + lx * math.sin(yaw) + ly * math.cos(yaw)
            out += include(kind, f"{name}_{kind}_{k}", wx, wy, lz, byaw, static=True)
    return out


MARKER_TEX = "model://dock_marker/materials/textures/aruco_4x4_100_"
SIGN_TEX = "model://sign/materials/textures/sign_"


def plate_model(name, tex, x, y, z, yaw, w, h, collide=False):
    """두께 0.02 m 판 (텍스처 = albedo_map), 법선 +x 가 yaw 방향 = 보는 쪽. 마커·표지판 공용."""
    col = f"""        <collision name="plate_collision">
          <pose>0 0 {z:.3f} 0 0 0</pose>
          <geometry><box><size>0.02 {w:.2f} {h:.2f}</size></box></geometry>
        </collision>
""" if collide else ""
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{pose(x, y, 0, 0, 0, yaw)}</pose>
      <link name="link">
{col}        <visual name="plate">
          <pose>0 0 {z:.3f} 0 0 0</pose>
          <geometry><box><size>0.02 {w:.2f} {h:.2f}</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
            <pbr><metal>
              <albedo_map>{tex}</albedo_map>
              <metalness>0.0</metalness><roughness>0.9</roughness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>
"""


# 위치 표지 판은 도크 마커(0.30 m 판 / 0.18 m 마커)보다 크다. 통합 11 실측: 납치 지점에서
# 통로를 따라 보이는 랙 표지까지 4~5 m 인데, 0.18 m 마커는 그 거리에서 약 12 px 로
# min_side_px 한계라 **검출 자체가 0 건**이었다 (모호성 거부도 0 — 아예 안 보였다).
# 0.50 m 판 / 0.30 m 마커면 같은 거리에서 20 px 로 여유가 생긴다. 기둥 면(0.50 m)에 맞춘 상한.
LOCATION_PLATE = 0.50


def marker_model(name, mid, x, y, yaw, plate=0.30):
    """models/dock_marker/model.sdf 와 같은 판을 id 별 텍스처로 인라인 생성. +x 법선이 로봇 쪽."""
    return plate_model(name, f"{MARKER_TEX}{mid}.png", x, y, MARKER_Z, yaw, plate, plate,
                       collide=True)


# 표지판 크기 = models/sign/gen_sign_textures.py SIGNS 의 size 와 같아야 한다 (텍스처 비율)
SIGN_SIZE = {
    "dock_1": (0.90, 0.45), "dock_2": (0.90, 0.45), "dock_a": (0.90, 0.45), "dock_b": (0.90, 0.45),
    "zone_inbound": (1.60, 0.40), "zone_outbound": (1.60, 0.40), "charging": (1.60, 0.50),
    "c1": (0.30, 0.20), "c2": (0.30, 0.20), "c3": (0.30, 0.20), "waiting": (1.60, 0.50),
    "zone_a": (0.80, 0.60), "zone_b": (0.80, 0.60), "zone_c": (0.80, 0.60),
    "caution_forklift": (0.60, 0.60), "caution_narrow": (0.60, 0.60), "exit": (0.80, 0.30),
}
_W, _E, _S, _N = 0.0, math.pi, math.pi / 2, -math.pi / 2     # 판이 보는 방향: 서벽→동쪽, 동벽→서쪽, 남벽→북쪽, 북벽→남쪽
_RACK_END = RACK_X[-1] + RACK_L / 2                          # 랙 열 끝면 |x| = 19
_NARROW_OFF = NARROW_CLEAR / 2 + RACK_D / 2
SIGN_POSES = [   # (이름, 텍스처, x, y, 판 중심 높이, yaw)
    ("sign_dock_1", "dock_1", -HALF_X + PLATE_OFF, 17.0, 1.6, _W),
    ("sign_dock_2", "dock_2", -HALF_X + PLATE_OFF, 13.0, 1.6, _W),
    ("sign_dock_a", "dock_a", HALF_X - PLATE_OFF, 17.0, 1.6, _E),
    ("sign_dock_b", "dock_b", HALF_X - PLATE_OFF, 13.0, 1.6, _E),
    ("sign_zone_inbound", "zone_inbound", -HALF_X + PLATE_OFF, 15.0, 2.6, _W),
    ("sign_zone_outbound", "zone_outbound", HALF_X - PLATE_OFF, 15.0, 2.6, _E),
    ("sign_charging", "charging", -22.0, -HALF_Y + PLATE_OFF, 1.8, _S),
    ("sign_waiting", "waiting", 22.0, -HALF_Y + PLATE_OFF, 1.8, _S),
] + [
    (f"sign_{n}", n, x, CHARGER_Y + STATION_FRONT + PLATE_OFF, 0.53, _S)
    for n, (x, _) in CHARGERS.items()
] + [
    (f"sign_zone_{r.lower()}_{side}", f"zone_{r.lower()}", sx * (_RACK_END + PLATE_OFF), y, 1.7,
     _W if sx > 0 else _E)
    for r, y in ROWS.items() for side, sx in (("west", -1), ("east", 1))
] + [
    (f"sign_caution_forklift_{i}", "caution_forklift", x, 12 - PILLAR_HALF - PLATE_OFF, 1.6, _N)
    for i, x in enumerate((-10.0, 10.0), 1)
] + [
    ("sign_caution_narrow_north", "caution_narrow", NARROW_CX - _NARROW_OFF,
     NARROW_CY + RACK_L + PLATE_OFF, 1.6, _S),
    ("sign_caution_narrow_south", "caution_narrow", NARROW_CX + _NARROW_OFF,
     NARROW_CY - RACK_L - PLATE_OFF, 1.6, _N),
    ("sign_exit_north", "exit", 0.0, HALF_Y - PLATE_OFF, 2.4, _N),
    ("sign_exit_south", "exit", 0.0, -HALF_Y + PLATE_OFF, 2.4, _S),
    ("sign_exit_west", "exit", -HALF_X + PLATE_OFF, 0.0, 2.4, _W),
    ("sign_exit_east", "exit", HALF_X - PLATE_OFF, 0.0, 2.4, _E),
]


def signs():
    out = ""
    for name, tex, x, y, z, yaw in SIGN_POSES:
        w, h = SIGN_SIZE[tex]
        out += plate_model(name, f"{SIGN_TEX}{tex}.png", x, y, z, yaw, w, h)
    # 비상구 문 (벽면 도색 판, 1.0 x 2.1 m) — 비상구 표지 아래
    for name, tex, x, y, z, yaw in SIGN_POSES:
        if tex != "exit":
            continue
        out += f"""    <model name="{name.replace('sign_', 'door_')}">
      <static>true</static>
      <pose>{pose(x, y, 0, 0, 0, yaw)}</pose>
      <link name="link">
        <visual name="door">
          <pose>-0.002 0 1.05 0 0 0</pose>
          <geometry><box><size>0.02 1.0 2.1</size></box></geometry>
          <material>
            <ambient>0.30 0.36 0.32 1</ambient><diffuse>0.30 0.36 0.32 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""
    return out


def docks_and_zones():
    out = ""
    for name, (x, y, kind, mid) in DOCKS.items():
        out += include(f"dock_pad_{kind}", f"{name}_pad", x, y)
        # 마커: 벽 안쪽 면에 밀착, 로봇이 있는 창고 안쪽을 향한다
        if kind == "inbound":
            out += marker_model(f"{name}_marker", mid, -HALF_X + 0.01, y, 0.0)
        else:
            out += marker_model(f"{name}_marker", mid, HALF_X - 0.01, y, math.pi)
        # 도크 위 화물 2개 (동적 — 밀리는 물체). 높이 0.30/0.40 m 라 스캔 평면(0.20 m)에 걸려 LiDAR 로 보인다
        sx = 1.0 if kind == "inbound" else -1.0
        bx = x - sx * 1.2
        out += include("box_medium", f"{name}_box_medium", bx, y + 1.0, 0.0,
                       rng.uniform(-0.3, 0.3))
        out += include("box_large", f"{name}_box_large", bx, y - 1.0, 0.0,
                       rng.uniform(-0.3, 0.3))
    # 충전 구역: 패드 + 스테이션 3개(남쪽 벽 앞, +x 진입 방향이 +y 가 되도록 yaw=π/2) + 스테이션 전면 마커
    out += include("charging_pad", "charging_zone_pad", -22.0, -16.0)
    for name, (x, mid) in CHARGERS.items():
        cs = f"charging_station_{name}"
        out += include("charging_station", cs, x, CHARGER_Y, 0.0, math.pi / 2)
        out += marker_model(f"{cs}_marker", mid, x, CHARGER_Y + STATION_FRONT + 0.01, math.pi / 2)
    out += include("waiting_zone", "waiting_zone_pad", 22.0, -16.0)
    return out


# 기둥 위치 표지 마커 (명세 4.5 Kidnapped Robot 복구). 창고 통로는 랙 열 간격 6 m 로 주기적이라
# 통로 방향 6 m 순간 이동은 LiDAR + 지도만으로 구별할 수 없다 — 스캔이 옆 통로에 그대로 맞아
# AMCL 추정이 움직이지 않으므로 "점프 > 1 m"·"인라이어 < 0.5" 가 모두 걸리지 않는다 (통합 11 실측:
# [-9.98, 0.23] → [-9.98, 6.23] 주입에 LOST 미선언, 마커도 시야에 없어 복구 경로 없음).
# 실제 물류센터가 기둥에 구역 표지를 붙이듯, 기둥 네 면에 고유 id 마커를 붙여 전역 기준점을 만든다.
# 통로를 따라 달리면 기둥이 정면에 들어오므로 전방 카메라로 잡힌다.
PILLAR_MARKER_ID0 = 10          # 도크 0~3, 충전 4~6, 예비 7~9 다음
PILLAR_FACES = [(1, 0, 0.0), (-1, 0, math.pi), (0, 1, math.pi / 2), (0, -1, -math.pi / 2)]


def pillar_markers():
    """기둥 i 의 면 j → id = PILLAR_MARKER_ID0 + 4*(i-1) + j (면 순서 +x, -x, +y, -y)."""
    off = PILLAR_HALF + PLATE_OFF
    out = ""
    for i, (px, py) in enumerate(PILLARS, 1):
        for j, (nx, ny, yaw) in enumerate(PILLAR_FACES):
            mid = PILLAR_MARKER_ID0 + 4 * (i - 1) + j
            out += marker_model(f"pillar_{i}_marker_{j}", mid, px + nx * off, py + ny * off,
                                yaw, LOCATION_PLATE)
    return out


def pillar_marker_poses():
    """{id: (x, y, yaw)} — 재위치추정 레지스트리(config/behavior.yaml)와 같은 값이어야 한다."""
    off = PILLAR_HALF + PLATE_OFF
    out = {}
    for i, (px, py) in enumerate(PILLARS, 1):
        for j, (nx, ny, yaw) in enumerate(PILLAR_FACES):
            out[PILLAR_MARKER_ID0 + 4 * (i - 1) + j] = (px + nx * off, py + ny * off, yaw)
    return out


RACK_MARKER_ID0 = 42            # 기둥 10~41 다음


def rack_marker_poses():
    """{id: (x, y, yaw)} — 랙 열마다 통로를 보는 양면에 하나씩 (복구 회전에서 바로 잡히도록)."""
    off = RACK_D / 2 + PLATE_OFF
    out = {}
    k = 0
    for row_y in sorted(ROWS.values(), reverse=True):
        for cx in RACK_X:
            out[RACK_MARKER_ID0 + k] = (cx, row_y + off, math.pi / 2)      # 북쪽 통로를 본다
            out[RACK_MARKER_ID0 + k + 1] = (cx, row_y - off, -math.pi / 2)  # 남쪽 통로를 본다
            k += 2
    return out


def rack_markers():
    out = ""
    for mid, (x, y, yaw) in sorted(rack_marker_poses().items()):
        out += marker_model(f"rack_marker_{mid}", mid, x, y, yaw, LOCATION_PLATE)
    return out


def pillars():
    return "".join(include("pillar", f"pillar_{i}", x, y) for i, (x, y) in enumerate(PILLARS, 1))


# ---- 동적 장애물: 사람 actor ---------------------------------------------------
def _heading(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def merge_collinear(nodes):
    """같은 방향으로 이어지는 중간 노드를 없앤다 (거기서는 멈추지 않는다)."""
    out = [nodes[0]]
    for i in range(1, len(nodes) - 1):
        if abs(_wrap(_heading(nodes[i], nodes[i + 1]) - _heading(out[-1], nodes[i]))) < 1e-9:
            continue
        out.append(nodes[i])
    out.append(nodes[-1])
    return out


def trapezoid(length, v, a):
    """정지→정지 사다리꼴(짧으면 삼각형) 속도 프로파일. (s(t) 함수, 총 시간)."""
    ta = v / a
    if a * ta * ta > length:                 # 등속 구간 없음
        ta = math.sqrt(length / a)
        v = a * ta
    tc = (length - a * ta * ta) / v
    total = 2 * ta + tc

    def s(t):
        if t <= ta:
            return 0.5 * a * t * t
        if t <= ta + tc:
            return 0.5 * a * ta * ta + v * (t - ta)
        r = total - t
        return length - 0.5 * a * r * r
    return s, total


def path_waypoints(nodes, speed, accel=WALK_ACCEL, turn_rate=TURN_RATE, dt=ACTOR_DT):
    """
    닫힌 경로(마지막 노드 = 첫 노드)를 걷는 (t, x, y, yaw) 웨이포인트.

    구간마다 정지→가속→등속→감속→정지, 꺾이는 노드에서 제자리 회전. 각 구간을 ACTOR_DT 에 가까운 등시간 간격으로
    샘플한다 (구간 경계 시각은 정확히 포함).
    """
    nodes = merge_collinear(nodes)
    assert math.hypot(nodes[-1][0] - nodes[0][0], nodes[-1][1] - nodes[0][1]) < 1e-9, \
        "loop must return to start"
    yaw0 = _heading(nodes[0], nodes[1])
    wps = [(0.0, nodes[0][0], nodes[0][1], yaw0)]
    t, yaw = 0.0, yaw0

    def turn_to(x, y, target):
        nonlocal t, yaw
        d = _wrap(target - yaw)
        if abs(d) < 1e-9:
            return
        tt = abs(d) / turn_rate
        n = max(1, math.ceil(tt / dt - 1e-9))
        for j in range(1, n + 1):
            wps.append((t + tt * j / n, x, y, yaw + d * j / n))
        t += tt
        yaw = target

    for k in range(len(nodes) - 1):
        (x0, y0), (x1, y1) = nodes[k], nodes[k + 1]
        turn_to(x0, y0, _heading(nodes[k], nodes[k + 1]))
        length = math.hypot(x1 - x0, y1 - y0)
        s_of, total = trapezoid(length, speed, accel)
        n = max(1, math.ceil(total / dt - 1e-9))
        for j in range(1, n + 1):
            tj = total * j / n
            f = s_of(tj) / length
            wps.append((t + tj, x0 + f * (x1 - x0), y0 + f * (y1 - y0), yaw))
        t += total
    turn_to(nodes[0][0], nodes[0][1], yaw0)
    return wps


def circle_waypoints(cx, cy, r, speed, dt=ACTOR_DT):
    """반시계 원 등속 순환. 등시간 샘플이라 닫힌 Catmull-Rom 이 된다 (마지막 점 = 첫 점)."""
    total = 2 * math.pi * r / speed
    n = max(8, math.ceil(total / dt - 1e-9))
    wps = []
    for i in range(n + 1):
        th = 2 * math.pi * (i % n) / n
        wps.append((total * i / n, cx + r * math.cos(th), cy + r * math.sin(th),
                    _wrap(th + math.pi / 2)))
    return wps


def random_walk_nodes(seed=SEED, steps=36):
    """
    통로 격자(x∈{±3,±9,±15}, y∈{6,0,-6}, 간격 6 m) 위 무작위 보행.

    스폰 셀을 지나는 간선 (-3,0)-(3,0) 은 제외한다.

    seed 고정 → 항상 같은 경로. steps 걸음 후 BFS 최단 경로로 출발점에 돌아와 루프를 닫는다.
    """
    xs, ys = [-15, -9, -3, 3, 9, 15], [6, 0, -6]
    nodes = [(x, y) for x in xs for y in ys]
    adj = {n: [] for n in nodes}

    def link(a, b):
        adj[a].append(b)
        adj[b].append(a)

    for y in ys:
        for i in range(len(xs) - 1):
            if y == 0 and xs[i] == -3:
                continue                     # 스폰 셀 (0,0) 통과 간선 제외
            link((xs[i], y), (xs[i + 1], y))
    for x in xs:
        for j in range(len(ys) - 1):
            link((x, ys[j]), (x, ys[j + 1]))
    r = random.Random(seed)
    start = (-9, -6)
    path = [start]
    prev = None
    for _ in range(steps):
        cand = [n for n in adj[path[-1]] if n != prev] or adj[path[-1]]
        prev = path[-1]
        path.append(r.choice(cand))
    # 출발점으로 복귀 (BFS)
    par = {path[-1]: None}
    q = deque([path[-1]])
    while q:
        u = q.popleft()
        if u == start:
            break
        for v in adj[u]:
            if v not in par:
                par[v] = u
                q.append(v)
    back = []
    u = start
    while u != path[-1]:
        back.append(u)
        u = par[u]
    path.extend(reversed(back))
    return [(float(x), float(y)) for x, y in path]


def actor_xml(name, wps):
    body = "".join(f"          <waypoint><time>{t:.3f}</time>"
                   f"<pose>{pose(x, y, 0, 0, 0, _wrap(yaw))}</pose></waypoint>\n"
                   for t, x, y, yaw in wps)
    return f"""    <actor name="{name}">
      <pose>0 0 0 0 0 0</pose>
      <skin><filename>model://worker/meshes/worker.dae</filename><scale>1.0</scale></skin>
      <animation name="walk"><filename>model://worker/meshes/worker.dae</filename></animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="0">
{body}        </trajectory>
      </script>
    </actor>
"""


def actor_specs():
    """(이름, 속도, 웨이포인트, 설명)."""
    def shuttle(a, b, v):
        return path_waypoints([a, b, a], v)
    return [
        ("worker_straight_slow", 0.5, shuttle((-16.0, 6.0), (16.0, 6.0), 0.5), "직선 왕복, A-B 통로"),
        ("worker_straight_fast", 1.2, shuttle((22.0, -10.0), (22.0, 10.0), 1.2), "직선 왕복, 동측 통로"),
        ("worker_curve", 0.8, circle_waypoints(-24.0, 0.0, 3.5, 0.8), "곡선(원) 순환, 서측 공터"),
        ("worker_random", 0.6, path_waypoints(random_walk_nodes(), 0.6),
         f"무작위 보행 (seed {SEED}, gen_warehouse_world.py random_walk_nodes)"),
        ("worker_crossing", 1.0, shuttle((-6.0, -7.0), (6.0, -7.0), 1.0),
         "좁은 통로 입구 횡단 (명세 4.7: 1.0 m/s 동적 장애물)"),
        ("worker_slow_south", 0.3, shuttle((-6.0, -14.5), (6.0, -14.5), 0.3),
         "좁은 통로 남쪽 출구 앞 느린 보행 (명세 하한 0.3 m/s)"),
    ]


# ---- 동적 장애물: 물리 모델 (지게차, 셔틀 AMR) --------------------------------------
SYSM = "ignition::gazebo::systems::"
LIB = "ignition-gazebo-"


def shuttle_plugins(name, axis, lo, hi, speed, accel, wheel_sep, wheel_r, left_joint, right_joint):
    """
    DiffDrive(가감속 램프) + PosePublisher(트리거 입력) + OdometryPublisher(지면 진실) + TriggeredPublisher 2개.

    axis 방향(몸체 전방)의 월드 좌표 성분 position.<axis> 가 [lo, hi] 사이를 왕복한다. 반전 창 시작 = 끝 − v²/(2a).
    """
    topic = f"/model/{name}"
    over = speed * speed / (2 * accel)
    tol = 0.25
    hi_trig, lo_trig = hi - over + tol, lo + over - tol       # 창 = 값 ± tol, 첫 일치 = 창 시작
    return f"""
      <plugin filename="{LIB}diff-drive-system" name="{SYSM}DiffDrive">
        <left_joint>{left_joint}</left_joint>
        <right_joint>{right_joint}</right_joint>
        <wheel_separation>{wheel_sep}</wheel_separation>
        <wheel_radius>{wheel_r}</wheel_radius>
        <topic>{topic}/cmd_vel</topic>
        <odom_topic>{topic}/diff_drive/odom</odom_topic>
        <tf_topic>{topic}/diff_drive/tf</tf_topic>
        <max_linear_velocity>{speed}</max_linear_velocity>
        <min_linear_velocity>{-speed}</min_linear_velocity>
        <max_linear_acceleration>{accel}</max_linear_acceleration>
        <min_linear_acceleration>{-accel}</min_linear_acceleration>
      </plugin>
      <plugin filename="{LIB}pose-publisher-system" name="{SYSM}PosePublisher">
        <!-- Fortress PosePublisher 는 모델 자신의 포즈를
             publish_nested_model_pose && publish_model_pose 일 때만 낸다 (하위 호환 코드) -->
        <publish_model_pose>true</publish_model_pose>
        <publish_nested_model_pose>true</publish_nested_model_pose>
        <publish_link_pose>false</publish_link_pose>
        <publish_collision_pose>false</publish_collision_pose>
        <publish_visual_pose>false</publish_visual_pose>
        <publish_sensor_pose>false</publish_sensor_pose>
        <use_pose_vector_msg>false</use_pose_vector_msg>
        <static_publisher>false</static_publisher>
        <update_frequency>50</update_frequency>
      </plugin>
      <plugin filename="{LIB}odometry-publisher-system" name="{SYSM}OdometryPublisher">
        <odom_frame>world</odom_frame>
        <robot_base_frame>{name}</robot_base_frame>
        <odom_topic>{topic}/odometry</odom_topic>
        <tf_topic>{topic}/odometry/tf</tf_topic>
        <odom_publish_frequency>50</odom_publish_frequency>
        <dimensions>2</dimensions>
      </plugin>
      <plugin filename="{LIB}triggered-publisher-system" name="{SYSM}TriggeredPublisher">
        <input type="ignition.msgs.Pose" topic="{topic}/pose">
          <match field="position.{axis}" tol="{tol}">{hi_trig:.3f}</match>
        </input>
        <output type="ignition.msgs.Twist" topic="{topic}/cmd_vel">linear: {{x: {-speed}}}</output>
      </plugin>
      <plugin filename="{LIB}triggered-publisher-system" name="{SYSM}TriggeredPublisher">
        <input type="ignition.msgs.Pose" topic="{topic}/pose">
          <match field="position.{axis}" tol="{tol}">{lo_trig:.3f}</match>
        </input>
        <output type="ignition.msgs.Twist" topic="{topic}/cmd_vel">linear: {{x: {speed}}}</output>
      </plugin>"""


# 이름: (축, 끝 lo, 끝 hi, 고정 좌표, 속도, 가속, yaw, 모델, 바퀴 간격, 바퀴 반지름)
VEHICLES = {
    "forklift_main": ("x", -20.0, 20.0, 15.0, 1.5, 1.0, 0.0, "forklift", 1.1, 0.25),
    "shuttle_amr": ("y", -6.0, 11.0, 9.0, 1.0, 0.8, math.pi / 2, "shuttle_amr", 0.36, 0.0825),
}


def vehicle_start(axis, lo, fixed, speed, accel):
    """출발 위치: lo 쪽 반전 창 안 (첫 포즈 메시지가 전진 지령을 낸다)."""
    s = lo + speed * speed / (2 * accel) - 0.1
    return (s, fixed) if axis == "x" else (fixed, s)


def vehicles():
    out = ""
    for name, (axis, lo, hi, fixed, v, a, yaw, model, sep, r) in VEHICLES.items():
        x, y = vehicle_start(axis, lo, fixed, v, a)
        desc = {"forklift_main": "지게차 (2.5 t, 전진·후진 왕복)",
                "shuttle_amr": "다른 회사 AMR 셔틀 (0.6 x 0.4 m, 전진·후진 왕복)"}[name]
        out += (f"    <!-- {name}: {desc}, {axis} {lo:g}..{hi:g}, {v} m/s, 가감속 {a} m/s² "
                f"(반전 {v / a:.1f} s 감속 + {v / a:.1f} s 가속) -->\n")
        out += include(model, name, x, y, 0.0, yaw, extra=shuttle_plugins(
            name, axis, lo, hi, v, a, sep, r, "wheel_left_joint", "wheel_right_joint"))
    return out


def dynamic_obstacles():
    out = "    <!-- ===== 동적 장애물 (사람 actor 6 + 물리 모델 2: 지게차, 셔틀 AMR) ===== -->\n"
    specs = actor_specs()
    for name, v, wps, desc in specs:
        out += f"    <!-- {name}: {v} m/s, {desc}, 루프 {wps[-1][0]:.1f} s, 웨이포인트 {len(wps)}개 -->\n"
        out += actor_xml(name, wps)
    out += vehicles()
    return out, specs


def world_plugins():
    """월드 레벨 시스템 플러그인 (헤더 주석 참고)."""
    out = ""
    for lib, cls in [("physics", "Physics"), ("user-commands", "UserCommands"),
                     ("scene-broadcaster", "SceneBroadcaster"), ("sensors", "Sensors"),
                     ("imu", "Imu"), ("contact", "Contact")]:
        tag = f'    <plugin filename="ignition-gazebo-{lib}-system" ' \
              f'name="ignition::gazebo::systems::{cls}"'
        if cls == "Sensors":
            out += tag + ">\n      <render_engine>ogre2</render_engine>\n    </plugin>\n"
        else:
            out += tag + "/>\n"
    return out


# ---- 검증 (생성 시점) --------------------------------------------------------
def _aabbs(racks):
    aabbs = []
    for name, x, y, yaw in racks:
        hx, hy = (RACK_L / 2, RACK_D / 2) if abs(math.sin(yaw)) < 0.5 else (RACK_D / 2, RACK_L / 2)
        aabbs.append((name, x - hx, x + hx, y - hy, y + hy))
    for i, (x, y) in enumerate(PILLARS, 1):
        aabbs.append((f"pillar_{i}", x - PILLAR_HALF, x + PILLAR_HALF,
                      y - PILLAR_HALF, y + PILLAR_HALF))
    return aabbs


def check_layout(racks, specs):
    aabbs = _aabbs(racks)
    margin = 0.35
    for name, x0, x1, y0, y1 in aabbs:
        assert not (x0 - 1.0 < 0 < x1 + 1.0 and y0 - 1.0 < 0 < y1 + 1.0), \
            f"스폰 셀 (0,0) 이 {name} 과 겹침"
    for aname, _, wps, _ in specs:
        for t, x, y, _ in wps:
            assert abs(x) < HALF_X - 0.5 and abs(y) < HALF_Y - 0.5, f"{aname} 웨이포인트 벽 밖: {x},{y}"
            for name, x0, x1, y0, y1 in aabbs:
                assert not (x0 - margin < x < x1 + margin and y0 - margin < y < y1 + margin), \
                    f"{aname} 웨이포인트 ({x},{y}) 가 {name} 안"
        dts = [b[0] - a[0] for a, b in zip(wps, wps[1:])]
        assert min(dts) > 1e-3 and max(dts) <= ACTOR_DT + 1e-6, (aname, min(dts), max(dts))
    # 차량 경로(반전 지점까지 + 차체 반폭 + 여유)가 랙·기둥과 겹치지 않는다
    half_w = {"forklift_main": 0.7, "shuttle_amr": 0.25}
    reach = {"forklift_main": (1.3, 2.1), "shuttle_amr": (0.35, 0.35)}   # 후방, 전방(포크) 길이
    for name, (axis, lo, hi, fixed, *_rest) in VEHICLES.items():
        w, (back, front) = half_w[name] + 0.3, reach[name]
        a0, a1 = lo - back, hi + front
        box = (a0, a1, fixed - w, fixed + w) if axis == "x" else (fixed - w, fixed + w, a0, a1)
        for oname, x0, x1, y0, y1 in aabbs:
            assert box[1] < x0 or box[0] > x1 or box[3] < y0 or box[2] > y1, \
                f"{name} 경로가 {oname} 과 겹침"
        assert max(abs(box[0]), abs(box[1])) < HALF_X and max(abs(box[2]), abs(box[3])) < HALF_Y
    # 좁은 통로 순폭
    w = [r for r in racks if r[0].startswith("rack_narrow_W")][0]
    e = [r for r in racks if r[0].startswith("rack_narrow_E")][0]
    clear = (e[1] - RACK_D / 2) - (w[1] + RACK_D / 2)
    assert abs(clear - NARROW_CLEAR) < 1e-6, clear


# ---- 지면 진실(GT) 내보내기 — --gt-dir 플래그로만 켠다 --------------------------------------------
# 기본 실행(warehouse.sdf 생성)은 이 절을 거치지 않는다 (표준 라이브러리만, 출력 바이트 불변).
# GT 는 build_sdf() 의 SDF 문자열 + models/<이름>/model.sdf 의 **시각체**에서 뽑는다. gpu_lidar·깊이 카메라는
# 렌더링 기반이라 충돌체가 아니라 시각체를 본다 (랙: 충돌체는 통짜 2.0 x 1.0 x 2.5, 시각체는 데크·기둥·선반).
# 포즈 합성은 amr_localization/world_geometry.py 와 같다: world ← include/model ← link ← visual.
# numpy 가 필요하다 (GT 함수 안에서만 import).
GT_SPACING = 0.01               # [m] 3D 표면 점 간격 (면마다 셀 중심 표본, 실제 간격 ≤ 이 값)
GT_RES = 0.01                   # [m] 2D 래스터 해상도 — 0.05 GT 의 셀 양자화(±2.5 cm)가 ADNN(≈2.6 cm)과 같은 크기라서
GT_SLICE_Z = (0.20,)            # [m] 2D 스캔 평면 = base_link_height 0.18 + lidar z 0.02 (config)
GT_BANDS = ((0.05, 0.40), (0.05, 2.0))      # [m] 비교용(로봇 높이 0.30 + 여유) / 운용용(랙 선반까지)
# 래스터 범위 = map_quality 의 --room ±1 m 와 같다
GT_BOUNDS = (-HALF_X - 1.0, HALF_X + 1.0, -HALF_Y - 1.0, HALF_Y + 1.0)
# 움직이는 모델은 뺀다 (actor 는 <model> 이 아니라 처음부터 없다). world_geometry.DEFAULT_EXCLUDE 는
# ('ground_plane', 'forklift', 'worker', 'amr_') 라 shuttle_amr 가 스폰 자세 그대로 GT 에 들어간다 — 여기서는 뺀다.
GT_EXCLUDE = ("forklift", "shuttle_amr", "worker", "amr_")
GT_LEGACY_EXCLUDE = ("ground_plane", "forklift", "worker", "amr_")   # world_geometry 와 같은 집합 (검증용)
GT_CLASSES = ("floor", "ceiling", "wall", "rack", "cargo", "pillar", "plate", "fixture",
              "floor_paint", "movable_cargo", "vehicle", "other")
GT_OFFSET = 1e-4                # [m] 가림 판정: 면 점을 바깥 법선으로 이만큼 민 점이 다른 고체 안이면 버린다
GT_PLY_DTYPE = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("label", "u1")]


def gt_label(model, visual):
    """모델·시각체 이름 → GT 부류 (GT_CLASSES)."""
    if model == "ground_plane":
        return "floor"
    if model == "roof":
        return "ceiling" if visual == "ceiling" else "fixture"
    if model == "warehouse_walls":
        return "wall"
    if "_marker" in model or model.startswith(("sign_", "door_")):
        return "plate"
    if model.startswith(("forklift", "shuttle_amr")):
        return "vehicle"
    if model.startswith("dock_") and "_box_" in model:
        return "movable_cargo"              # 도크 화물: 정적이 아니다(로봇이 밀 수 있다) — 시작 자세 기준
    if model.startswith("rack_") and "_box_" in model:
        return "cargo"
    if model.startswith("rack_"):
        return "rack"
    if model.startswith("pillar_"):
        return "pillar"
    if model.endswith("_pad") or visual in ("pad", "border"):
        return "floor_paint"
    if model.startswith("charging_station_"):
        return "fixture"
    return "other"


def _pose_matrix(text):
    """'x y z r p y' → 4x4 (SDF rpy 고정축 X-Y-Z, R = Rz Ry Rx)."""
    import numpy as np
    t = np.eye(4)
    if text is None or not text.strip():
        return t
    v = [float(x) for x in text.split()] + [0.0] * 6
    (cr, sr), (cp, sp), (cy, sy) = [(math.cos(a), math.sin(a)) for a in v[3:6]]
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    t[:3, :3] = rz @ ry @ rx
    t[:3, 3] = v[:3]
    return t


class GtPrim:
    """월드 좌표 시각체 도형 1 개 (box | cylinder | sphere | plane)."""

    def __init__(self, name, model, label, kind, size, world_from_shape):
        import numpy as np
        self.name, self.model, self.label, self.kind = name, model, label, kind
        self.size = tuple(float(v) for v in size)
        self.T = np.asarray(world_from_shape, dtype=float)
        self.R, self.t = self.T[:3, :3], self.T[:3, 3]

    def half(self):
        """국소 반 크기 (x, y, z). 평면은 두께 0."""
        import numpy as np
        if self.kind == "box":
            return np.array(self.size) / 2.0
        if self.kind == "cylinder":
            return np.array([self.size[0], self.size[0], self.size[1] / 2.0])
        if self.kind == "sphere":
            return np.array([self.size[0]] * 3)
        return np.array([self.size[0] / 2.0, self.size[1] / 2.0, 0.0])

    def aabb(self):
        """월드 AABB (lo, hi)."""
        import numpy as np
        ext = np.abs(self.R) @ self.half()
        return self.t - ext, self.t + ext

    def contains(self, pts, tol=1e-9):
        """월드 점 (N, 3) 이 닫힌 도형 안인지 (평면은 아래 반공간 = 법선 반대쪽, 판 범위 안)."""
        import numpy as np
        local = (pts - self.t) @ self.R
        h = self.half()
        if self.kind == "box":
            return np.all(np.abs(local) <= h + tol, axis=1)
        if self.kind == "cylinder":
            return ((local[:, 0] ** 2 + local[:, 1] ** 2 <= h[0] ** 2 + tol)
                    & (np.abs(local[:, 2]) <= h[2] + tol))
        if self.kind == "sphere":
            return np.sum(local ** 2, axis=1) <= h[0] ** 2 + tol
        return ((np.abs(local[:, 0]) <= h[0] + tol) & (np.abs(local[:, 1]) <= h[1] + tol)
                & (local[:, 2] <= tol))

    def yaw_only(self):
        """국소 z 축이 월드 z 축과 같은지 (높이 대역 래스터는 이 경우만 정확)."""
        return abs(self.R[2, 2] - 1.0) < 1e-9


def _plane_frame(geom):
    """SDF <plane> → (크기, 국소 z = 법선 회전). 법선은 ±z 만 지원 (바닥·천장)."""
    import numpy as np
    normal = [float(v) for v in geom.findtext("normal", "0 0 1").split()]
    size = [float(v) for v in geom.findtext("size", "1 1").split()]
    rot = np.eye(4)
    if normal[2] < 0:
        rot[:3, :3] = np.diag([1.0, -1.0, -1.0])            # x 축 둘레 π
    if abs(abs(normal[2]) - 1.0) > 1e-9:
        raise ValueError(f"plane normal {normal} 은 지원하지 않는다 (±z 만)")
    return size, rot


def gt_primitives(sdf_text, model_dirs, exclude=GT_EXCLUDE):
    """
    SDF 문자열의 정적 시각체 → (GtPrim 목록, 건너뛴 것 목록).

    월드 <model>(인라인)과 <include>(model_dirs 의 <uri>/model.sdf) 를 모두 읽는다. 이름이 exclude 접두어로
    시작하는 모델은 뺀다. 메시 시각체는 건너뛰고 목록에 남긴다 (이 월드의 정적 모델에는 없다).
    """
    import xml.etree.ElementTree as ET
    import numpy as np
    world = ET.fromstring(sdf_text).find("world")
    prims, skipped = [], []

    def add_model(model, world_from_model, name):
        for sub in model.findall("model"):
            add_model(sub, world_from_model @ _pose_matrix(sub.findtext("pose")),
                      f"{name}::{sub.get('name', '')}")
        for link in model.findall("link"):
            world_from_link = world_from_model @ _pose_matrix(link.findtext("pose"))
            for vis in link.findall("visual"):
                vname = vis.get("name", "")
                label = f"{name}::{link.get('name', '')}::{vname}"
                geom = vis.find("geometry")
                tf = world_from_link @ _pose_matrix(vis.findtext("pose"))
                kind = None if geom is None else next(
                    (g.tag for g in geom if g.tag in ("box", "cylinder", "sphere", "plane")), None)
                if kind == "box":
                    size = [float(v) for v in geom.findtext("box/size", "1 1 1").split()]
                elif kind == "cylinder":
                    size = [float(geom.findtext("cylinder/radius", "0.5")),
                            float(geom.findtext("cylinder/length", "1.0"))]
                elif kind == "sphere":
                    size = [float(geom.findtext("sphere/radius", "0.5"))]
                elif kind == "plane":
                    size, rot = _plane_frame(geom.find("plane"))
                    tf = tf @ rot
                else:
                    skipped.append(f"{label}: unsupported geometry")
                    continue
                top = name.split("::")[0]
                prims.append(GtPrim(label, top, gt_label(top, vname), kind, size, tf))

    def excluded(name):
        return any(name.startswith(p) for p in exclude)

    for model in world.findall("model"):
        name = model.get("name", "")
        if not excluded(name):
            add_model(model, _pose_matrix(model.findtext("pose")), name)
    for inc in world.findall("include"):
        name = inc.findtext("name") or inc.findtext("uri", "")
        if excluded(name):
            continue
        uri = inc.findtext("uri", "").replace("model://", "").strip("/")
        path = next((os.path.join(d, uri, "model.sdf") for d in model_dirs
                     if os.path.isfile(os.path.join(d, uri, "model.sdf"))), None)
        if path is None:
            skipped.append(f"{name}: model {uri} not found")
            continue
        model = ET.parse(path).getroot().find("model")
        add_model(model, _pose_matrix(inc.findtext("pose")) @ _pose_matrix(model.findtext("pose")),
                  name)
    assert all(np.isfinite(p.T).all() for p in prims)
    return prims, skipped


def gt_raster(prims, z0, z1, res=GT_RES, bounds=GT_BOUNDS):
    """
    점유 래스터 (행 0 = 최하단 y, 셀 (0,0) 모서리 = (xmin, ymin)). 평면(바닥·천장)은 넣지 않는다.

    z0 == z1: 높이 z0 단면 — 셀 중심 (x, y, z0) 가 도형 안이면 점유 (world_geometry.rasterize 와 같은 규칙).
    z0 < z1: 높이 대역 — 도형의 z 범위가 [z0, z1] 과 겹치고 셀 중심 (x, y) 가 그 도형의 xy 발자국 안이면 점유.
    대역은 국소 z = 월드 z 인 상자만 정확하다 (아니면 ValueError — 이 월드의 정적 시각체는 전부 yaw 회전뿐).
    """
    import numpy as np
    xmin, xmax, ymin, ymax = bounds
    width = int(math.ceil((xmax - xmin) / res))
    rows = int(math.ceil((ymax - ymin) / res))
    occ = np.zeros((rows, width), dtype=bool)
    for p in prims:
        if p.kind == "plane":
            continue
        lo, hi = p.aabb()
        if hi[2] < z0 - 1e-9 or lo[2] > z1 + 1e-9:
            continue
        if z1 > z0 and not (p.kind == "box" and p.yaw_only()):
            raise ValueError(f"{p.name}: 높이 대역 래스터는 yaw 만 회전한 상자만 지원")
        c0 = max(int(math.floor((lo[0] - xmin) / res)), 0)
        c1 = min(int(math.ceil((hi[0] - xmin) / res)), width)
        r0 = max(int(math.floor((lo[1] - ymin) / res)), 0)
        r1 = min(int(math.ceil((hi[1] - ymin) / res)), rows)
        if c0 >= c1 or r0 >= r1:
            continue
        cc, rr = np.meshgrid(np.arange(c0, c1), np.arange(r0, r1))
        pts = np.stack([xmin + (cc.ravel() + 0.5) * res, ymin + (rr.ravel() + 0.5) * res,
                        np.full(cc.size, p.t[2] if z1 > z0 else z0)], axis=1)
        inside = p.contains(pts)
        occ[rr.ravel()[inside], cc.ravel()[inside]] = True
    return occ


GT_DYN_MARGIN = 0.5            # [m] 동적 장애물 경로 둘레 여유 (사람 반폭 + 팔 흔들림, 차량은 차체 반폭에 더한다)
GT_DYN_RES = 0.05


def gt_dynamic_raster(specs, res=GT_DYN_RES, margin=GT_DYN_MARGIN, bounds=GT_BOUNDS):
    """
    동적 장애물이 지나가는 영역 래스터 (유령 점 판정용).

    actor 웨이포인트 꺾은선에서 margin 안 + 차량 왕복 차선(check_layout 의 차체 반폭·전후 길이 + margin).
    """
    import numpy as np
    xmin, xmax, ymin, ymax = bounds
    width = int(math.ceil((xmax - xmin) / res))
    rows = int(math.ceil((ymax - ymin) / res))
    occ = np.zeros((rows, width), dtype=bool)

    def window(x0, x1, y0, y1):
        c0 = max(int(math.floor((x0 - xmin) / res)), 0)
        c1 = min(int(math.ceil((x1 - xmin) / res)), width)
        r0 = max(int(math.floor((y0 - ymin) / res)), 0)
        r1 = min(int(math.ceil((y1 - ymin) / res)), rows)
        cc, rr = np.meshgrid(np.arange(c0, max(c0, c1)), np.arange(r0, max(r0, r1)))
        cc, rr = cc.ravel(), rr.ravel()
        return cc, rr, xmin + (cc + 0.5) * res, ymin + (rr + 0.5) * res

    for _, _, wps, _ in specs:
        pts = [(x, y) for _, x, y, _ in wps]
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            cc, rr, px, py = window(min(ax, bx) - margin, max(ax, bx) + margin,
                                    min(ay, by) - margin, max(ay, by) + margin)
            dx, dy = bx - ax, by - ay
            ll = dx * dx + dy * dy
            u = np.clip(((px - ax) * dx + (py - ay) * dy) / ll, 0.0, 1.0) if ll > 0 else 0.0 * px
            hit = np.hypot(px - (ax + u * dx), py - (ay + u * dy)) <= margin
            occ[rr[hit], cc[hit]] = True
    half_w = {"forklift_main": 0.7, "shuttle_amr": 0.25}        # check_layout 와 같은 값
    reach = {"forklift_main": (1.3, 2.1), "shuttle_amr": (0.35, 0.35)}
    for name, (axis, lo, hi, fixed, *_rest) in VEHICLES.items():
        w, (back, front) = half_w[name] + margin, reach[name]
        a0, a1 = lo - back - margin, hi + front + margin
        box = (a0, a1, fixed - w, fixed + w) if axis == "x" else (fixed - w, fixed + w, a0, a1)
        cc, rr, _, _ = window(*box)
        occ[rr, cc] = True
    return occ


def write_pgm_map(occ, res, bounds, stem):
    """map_server 형식 (P5 PGM 점유 0 / 자유 254 + YAML, 위쪽 행 = +y 끝). world_geometry.write_map 과 같다."""
    import numpy as np
    img = np.where(occ, 0, 254).astype(np.uint8)[::-1]
    with open(stem + ".pgm", "wb") as f:
        f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
        f.write(img.tobytes())
    with open(stem + ".yaml", "w", encoding="utf-8") as f:
        f.write(f"image: {os.path.basename(stem)}.pgm\nmode: trinary\nresolution: {res}\n"
                f"origin: [{bounds[0]}, {bounds[2]}, 0.0]\nnegate: 0\n"
                f"occupied_thresh: 0.65\nfree_thresh: 0.19\n")


def _faces(p, room):
    """도형의 면 (c0, eu, ev, n, Lu, Lv): 점 = c0 + a·eu + b·ev (a∈[0,Lu], b∈[0,Lv]), n = 바깥 법선."""
    import numpy as np
    if p.kind == "plane":
        hx, hy = p.size[0] / 2.0, p.size[1] / 2.0
        x0, x1 = max(p.t[0] - hx, room[0]), min(p.t[0] + hx, room[1])     # 방 안쪽으로 자른다
        y0, y1 = max(p.t[1] - hy, room[2]), min(p.t[1] + hy, room[3])
        if x0 >= x1 or y0 >= y1:
            return []
        return [(np.array([x0, y0, p.t[2]]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]),
                 p.R[:, 2].copy(), x1 - x0, y1 - y0)]
    if p.kind != "box":
        raise ValueError(f"{p.name}: 3D 표면 표본은 상자·평면만 지원 ({p.kind})")
    h = p.half()
    out = []
    for a in range(3):
        b, c = [k for k in range(3) if k != a]
        for sgn in (-1.0, 1.0):
            local = np.zeros(3)
            local[a] = sgn * h[a]
            local[b], local[c] = -h[b], -h[c]
            out.append((p.R @ local + p.t, p.R[:, b].copy(), p.R[:, c].copy(), sgn * p.R[:, a],
                        2 * h[b], 2 * h[c]))
    return out


def _solid(p):
    """가림 판정용 고체 (lo, hi, contains). 바닥 평면 = 아래 반공간, 천장 평면(법선 −z) = 위 반공간."""
    lo, hi = p.aabb()
    if p.kind == "plane":
        lo, hi = lo.copy(), hi.copy()
        if p.R[2, 2] > 0:
            lo[2] = -1e3
        else:
            hi[2] = 1e3
    return lo, hi, p.contains


def gt_surface_points(prims, spacing=GT_SPACING, roi=None, sink=None):
    """
    시각체 합집합의 표면 점 (부류 라벨 포함) — 가려진 면(다른 고체 안)과 방 밖 면은 뺀다.

    면마다 간격 ≤ spacing 의 셀 중심 격자로 표본하고, 점을 바깥 법선으로 GT_OFFSET 민 위치가 다른 고체 안이면
    버린다 (상자 바닥이 데크 위에 놓인 면, 데크 윗면 중 상자 아래, 기둥 안쪽에 묻힌 랙 기둥 등).
    방 밖(외벽 바깥면·모서리) 점은 실내 센서가 볼 수 없으므로 뺀다. 같은 평면에 겹친 두 면(랙 기둥 바깥면과 데크
    옆면 등)은 둘 다 남아 그 자리 밀도가 두 배다. roi=(xmin, xmax, ymin, ymax) 이면 그 안만.
    sink(array) 가 있으면 면마다 구조 배열(GT_PLY_DTYPE)을 넘기고, 없으면 모아서 돌려준다.
    """
    import numpy as np
    room = (-HALF_X, HALF_X, -HALF_Y, HALF_Y)
    tol = 1e-6
    solids = [_solid(p) for p in prims]
    lo_all = np.array([s[0] for s in solids])
    hi_all = np.array([s[1] for s in solids])
    chunks = []
    counts = {}
    for k, p in enumerate(prims):
        lab = GT_CLASSES.index(p.label)
        for c0, eu, ev, n, lu, lv in _faces(p, room):
            nu = max(1, math.ceil(lu / spacing - 1e-9))
            nv = max(1, math.ceil(lv / spacing - 1e-9))
            du, dv = lu / nu, lv / nv
            corners = np.array([c0, c0 + lu * eu, c0 + lv * ev, c0 + lu * eu + lv * ev])
            flo, fhi = corners.min(0) - 2 * GT_OFFSET, corners.max(0) + 2 * GT_OFFSET
            if roi is not None and (fhi[0] < roi[0] or flo[0] > roi[1] or fhi[1] < roi[2]
                                    or flo[1] > roi[3]):
                continue
            if (fhi[0] < room[0] - tol or flo[0] > room[1] + tol or fhi[1] < room[2] - tol
                    or flo[1] > room[3] + tol):
                continue
            keep = np.ones((nv, nu), dtype=bool)
            near = np.nonzero(np.all(lo_all <= fhi, axis=1) & np.all(hi_all >= flo, axis=1))[0]
            for j in near:
                if j == k:
                    continue
                slo, shi, inside = solids[j]
                box = np.array([[x, y, z] for x in (slo[0], shi[0]) for y in (slo[1], shi[1])
                                for z in (slo[2], shi[2])]) - c0
                u, v = box @ eu, box @ ev
                i0 = max(0, math.ceil(u.min() / du - 0.5 - 1e-9))
                i1 = min(nu - 1, math.floor(u.max() / du - 0.5 + 1e-9))
                j0 = max(0, math.ceil(v.min() / dv - 0.5 - 1e-9))
                j1 = min(nv - 1, math.floor(v.max() / dv - 0.5 + 1e-9))
                if i0 > i1 or j0 > j1:
                    continue
                ii, jj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1))
                pts = (c0 + GT_OFFSET * n + np.outer((ii.ravel() + 0.5) * du, eu)
                       + np.outer((jj.ravel() + 0.5) * dv, ev))
                hit = inside(pts)
                keep[jj.ravel()[hit], ii.ravel()[hit]] = False
            jj, ii = np.nonzero(keep)
            for s in range(0, len(ii), 4_000_000):
                a, b = ii[s:s + 4_000_000], jj[s:s + 4_000_000]
                pts = c0 + np.outer((a + 0.5) * du, eu) + np.outer((b + 0.5) * dv, ev)
                ok = ((pts[:, 0] >= room[0] - tol) & (pts[:, 0] <= room[1] + tol)
                      & (pts[:, 1] >= room[2] - tol) & (pts[:, 1] <= room[3] + tol)
                      & (pts[:, 2] >= -tol) & (pts[:, 2] <= WALL_H + tol))
                if roi is not None:
                    ok &= ((pts[:, 0] >= roi[0]) & (pts[:, 0] <= roi[1])
                           & (pts[:, 1] >= roi[2]) & (pts[:, 1] <= roi[3]))
                pts = pts[ok]
                arr = np.empty(len(pts), dtype=GT_PLY_DTYPE)
                arr["x"], arr["y"], arr["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
                arr["label"] = lab
                counts[p.label] = counts.get(p.label, 0) + len(arr)
                if sink is not None:
                    sink(arr)
                else:
                    chunks.append(arr)
    if sink is not None:
        return None, counts
    return (np.concatenate(chunks) if chunks else np.empty(0, dtype=GT_PLY_DTYPE)), counts


def ply_header(n, comments=()):
    """binary_little_endian PLY 머리 (x, y, z float32 + label uchar)."""
    lines = ["ply", "format binary_little_endian 1.0"]
    lines += [f"comment {c}" for c in comments]
    lines += [f"element vertex {n}", "property float x", "property float y", "property float z",
              "property uchar label", "end_header"]
    return ("\n".join(lines) + "\n").encode("ascii")


def write_obj(prims, path):
    """
    시각체 메시 (OBJ): 상자 = 꼭짓점 8 + 사각면 6, 평면 = 꼭짓점 4 + 사각면 1. g = 부류, o = 시각체 이름.

    원기둥·구(차량 바퀴 등 — 기본 제외 집합에서는 없다)는 쓰지 않고 그 이름 목록을 돌려준다.
    """
    import numpy as np
    lines = [f"# gen_warehouse_world.py --gt-dir: 정적 시각체 {len(prims)} 개 (월드 좌표, m)"]
    skipped = []
    base = 1
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    for p in prims:
        h = p.half()
        if p.kind == "plane":
            local = np.array([[-h[0], -h[1], 0], [h[0], -h[1], 0], [h[0], h[1], 0],
                              [-h[0], h[1], 0]])
            faces = [(0, 1, 2, 3)]
        elif p.kind == "box":
            local = np.array([[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1, 1)
                              for sy in (-1, 1) for sz in (-1, 1)])
            faces = quads
        else:
            skipped.append(p.name)
            continue
        lines.append(f"g {p.label}")
        lines.append(f"o {p.name}")
        for q in local @ p.R.T + p.t:
            lines.append(f"v {q[0]:.6f} {q[1]:.6f} {q[2]:.6f}")
        lines += ["f " + " ".join(str(base + i) for i in f) for f in faces]
        base += len(local)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return skipped


def export_gt(out_dir, model_dirs, spacing=GT_SPACING, res=GT_RES, slices=GT_SLICE_Z,
              bands=GT_BANDS, exclude=GT_EXCLUDE, points=True, roi=None, committed=None):
    """
    GT 파일 세트를 out_dir 에 쓴다. 반환: 메타데이터 dict (gt_meta.json 과 같은 내용).

      visual_mesh.obj        정적 시각체 메시
      primitives.json        시각체 도형 목록 (부류, 종류, 크기, 월드 4x4 변환) — 해석적 거리·광선 추적용
      surface_points.ply     합집합 표면 점 (간격 ≤ spacing, float32 x y z + uchar label)
      slice_z<h>_r<res>.{pgm,yaml}       높이 h 단면 래스터 (map_server 형식)
      band_z<z0>-<z1>_r<res>.{pgm,yaml}  높이 대역 래스터
      dynamic_swept_r0.050.{pgm,yaml}    동적 장애물 경로 영역 (유령 점 판정용)
      gt_meta.json           생성 조건·SDF sha256·부류별 점 수
    """
    import hashlib
    import json
    import shutil
    import tempfile
    sdf, _, specs = build_sdf()
    os.makedirs(out_dir, exist_ok=True)
    prims, skipped = gt_primitives(sdf, model_dirs, exclude)
    meta = {"generator": "gen_warehouse_world.py --gt-dir", "seed": SEED,
            "sdf_sha256": hashlib.sha256(sdf.encode()).hexdigest(),
            "committed_sdf_sha256": None, "matches_committed_sdf": None,
            "exclude_prefixes": list(exclude), "classes": list(GT_CLASSES),
            "spacing": spacing, "raster_resolution": res, "raster_bounds": list(GT_BOUNDS),
            "slices_z": list(slices), "bands_z": [list(b) for b in bands],
            "roi": list(roi) if roi else None, "primitives": len(prims),
            "primitives_by_class": {c: sum(p.label == c for p in prims) for c in GT_CLASSES},
            "skipped": skipped, "files": []}
    if committed and os.path.isfile(committed):
        with open(committed, "rb") as f:
            meta["committed_sdf_sha256"] = hashlib.sha256(f.read()).hexdigest()
        meta["matches_committed_sdf"] = meta["committed_sdf_sha256"] == meta["sdf_sha256"]

    def name(*parts):
        meta["files"].append(parts[-1])
        return os.path.join(out_dir, *parts)

    meta["not_in_mesh_or_points"] = write_obj(prims, name("visual_mesh.obj"))
    with open(name("primitives.json"), "w", encoding="utf-8") as f:
        json.dump([{"name": p.name, "model": p.model, "label": p.label, "kind": p.kind,
                    "size": list(p.size), "T": [round(float(v), 9) for v in p.T[:3].ravel()]}
                   for p in prims], f, ensure_ascii=False, indent=0)
    for z in slices:
        stem = f"slice_z{z:.3f}_r{res:.3f}"
        write_pgm_map(gt_raster(prims, z, z, res), res, GT_BOUNDS, os.path.join(out_dir, stem))
        meta["files"] += [stem + ".pgm", stem + ".yaml"]
    for z0, z1 in bands:
        stem = f"band_z{z0:.3f}-{z1:.3f}_r{res:.3f}"
        write_pgm_map(gt_raster(prims, z0, z1, res), res, GT_BOUNDS, os.path.join(out_dir, stem))
        meta["files"] += [stem + ".pgm", stem + ".yaml"]
    stem = f"dynamic_swept_r{GT_DYN_RES:.3f}"
    write_pgm_map(gt_dynamic_raster(specs), GT_DYN_RES, GT_BOUNDS, os.path.join(out_dir, stem))
    meta["files"] += [stem + ".pgm", stem + ".yaml"]
    meta["dynamic_margin"] = GT_DYN_MARGIN
    if points:
        with tempfile.TemporaryFile(dir=out_dir) as body:
            solid = [p for p in prims if p.kind in ("box", "plane")]
            _, counts = gt_surface_points(solid, spacing, roi, sink=lambda a: a.tofile(body))
            n = sum(counts.values())
            body.seek(0)
            with open(name("surface_points.ply"), "wb") as f:
                f.write(ply_header(n, [f"gen_warehouse_world.py --gt-dir spacing {spacing} m",
                                       "label " + " ".join(f"{i}={c}" for i, c in
                                                           enumerate(GT_CLASSES))]))
                shutil.copyfileobj(body, f, 16 << 20)
        meta["points"] = n
        meta["points_by_class"] = {c: counts.get(c, 0) for c in GT_CLASSES}
    with open(name("gt_meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    return meta


def check_blockers(racks, blockers):
    """블로커가 랙·기둥과 겹치지 않고 실내에 있는지 (블로커 AABB 대 랙·기둥 AABB, 닿는 것은 허용)."""
    aabbs = _aabbs(racks)
    for kind, x, y, yaw in blockers:
        sx, sy = AMR_BLOCKER[:2] if kind == "amr" else WALL_BLOCKER[:2]
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        hx, hy = (c * sx + s * sy) / 2, (s * sx + c * sy) / 2
        assert abs(x) + hx < HALF_X and abs(y) + hy < HALF_Y, (kind, x, y)
        for name, x0, x1, y0, y1 in aabbs:
            assert x + hx <= x0 or x - hx >= x1 or y + hy <= y0 or y - hy >= y1, \
                f"블로커 ({x}, {y}) 가 {name} 과 겹침"


# ---- 평가 시나리오용 월드 변형 — --variant 플래그로만 (docs/research/slam3d/evaluation.md §6) ---------
# 기본 월드(변형 없음)는 위 build_sdf 출력 그대로다. 변형은 커밋하지 않고 기록할 때 만들어 쓴다.
#   s1        : 동적 장애물(사람 actor 6 + 지게차·셔틀) 없음 — S1 정상
#   s6-queue  : 좁은 통로 북쪽 토큰 대기 위치 (0, -5.5) 에 AMR 크기 정적 블로커 (traffic.yaml corridor_approach_m 2.5)
#   s6-queue2 : 북 (0, -5.5) + 남 (0, -14.5) 양끝 대기
#   s6-wall   : 양끝 입구 바로 앞 (0, -7.75)·(0, -12.25) 를 0.8 x 0.3 x 0.5 m 화물로 완전 차단 (최악)
# 블로커는 정적 모델이라 GT(--gt-dir)에는 넣지 않는다 — 사전 지도에 없는 물체로 다룬다.
# 같은 블로커 기하가 amr_evaluation/slam3d/localizability.py s6_blockers 에 있다 (시험이 둘을 대조한다).
AMR_BLOCKER = (0.6, 0.4, 0.30, 0.03)        # 길이, 폭, 높이, 바닥 틈 [m] — config/robot_params.yaml 차체
WALL_BLOCKER = (0.8, 0.3, 0.5)
S6_BLOCKERS = {
    "s6-queue": [("amr", 0.0, NARROW_CY + 4.5, -math.pi / 2)],
    "s6-queue2": [("amr", 0.0, NARROW_CY + 4.5, -math.pi / 2),
                  ("amr", 0.0, NARROW_CY - 4.5, math.pi / 2)],
    "s6-wall": [("wall", 0.0, NARROW_CY + RACK_L + 0.25, 0.0),
                ("wall", 0.0, NARROW_CY - RACK_L - 0.25, 0.0)],
}
VARIANTS = ("s1",) + tuple(S6_BLOCKERS)


def blocker_xml(variant):
    """S6 정적 블로커 모델들 (시각체 + 충돌체 상자)."""
    out = f"    <!-- ===== 평가 변형 {variant}: 정적 블로커 (evaluation.md §6) ===== -->\n"
    for i, (kind, x, y, yaw) in enumerate(S6_BLOCKERS[variant]):
        if kind == "amr":
            sx, sy, sz, gap = AMR_BLOCKER
        else:
            (sx, sy, sz), gap = WALL_BLOCKER, 0.0
        geom = f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
        out += f"""    <model name="s6_blocker_{i}">
      <static>true</static>
      <pose>{pose(x, y, 0, 0, 0, yaw)}</pose>
      <link name="link">
        <collision name="collision"><pose>0 0 {gap + sz / 2:.3f} 0 0 0</pose>{geom}</collision>
        <visual name="visual"><pose>0 0 {gap + sz / 2:.3f} 0 0 0</pose>{geom}
          <material><ambient>0.2 0.3 0.6 1</ambient><diffuse>0.2 0.3 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>
"""
    return out


def build_sdf(variant=None):
    """
    월드 SDF 문자열과 (racks, specs). 호출마다 같은 결과 (seed 난수를 처음부터 다시 쓴다).

    variant 가 None 이면 커밋된 warehouse.sdf 와 바이트 단위로 같다. VARIANTS 의 값이면 평가용 변형.
    """
    if variant is not None and variant not in VARIANTS:
        raise ValueError(f"variant {variant!r} (가능: {VARIANTS})")
    rng.seed(SEED)
    racks = rack_instances()
    dyn_xml, specs = dynamic_obstacles()
    check_layout(racks, specs)
    if variant == "s1":
        dyn_xml = "    <!-- ===== 평가 변형 s1: 동적 장애물 없음 (evaluation.md §6) ===== -->\n"
    elif variant is not None:
        check_blockers(racks, S6_BLOCKERS[variant])
        dyn_xml = dyn_xml + blocker_xml(variant)

    n_loc_markers = 4 * len(PILLARS) + len(rack_marker_poses())

    sdf = f"""<?xml version="1.0" ?>
<!--
  물류센터 월드 (내부 60 x 40 m) — gen_warehouse_world.py 가 생성 (seed {SEED}). 직접 수정하지 말고 생성기를 고칠 것.
  레이아웃/좌표계/동적 장애물 설명은 생성기 docstring 참고. 기동은 launch/warehouse.launch.py.

  월드 플러그인
    Physics          : 1 ms 고정 스텝, RTF 1.0.
                       (gz-sim Fortress 는 <physics type> 과 무관하게 ign-physics DART 를 쓴다)
    UserCommands     : create(스폰, amr_description scripts/gz_world.py)/set_pose 서비스
    SceneBroadcaster : pose/info, scene/info (GUI, 스폰 확인)
    Sensors(ogre2)   : gpu_lidar / 카메라 / depth 렌더링
                       헤드리스는 ign gazebo -s 에 headless-rendering 옵션 (EGL)
                       (XML 주석 안에는 이중 하이픈을 쓸 수 없어 옵션 이름을 풀어 적음)
    Imu              : IMU 센서는 월드 레벨 Imu 시스템이 있어야 데이터를 낸다
                       (Fortress: imu/magnetometer/air_pressure 는 월드 시스템)
    Contact          : 접촉 센서(범퍼/E-Stop 시험용) — 선택
-->
<sdf version="1.8">
  <world name="warehouse">
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
{world_plugins()}
    <scene>
      <ambient>0.25 0.25 0.25 1</ambient>
      <background>0.35 0.37 0.40 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- ===== 조명 ===== -->
{lights()}
    <!-- ===== 바닥 / 외벽 / 천장 ===== -->
{ground_and_walls()}
{roof()}
    <!-- ===== 기둥 {len(PILLARS)}개 + 위치 표지 마커 {n_loc_markers}개 ===== -->
{pillars()}{pillar_markers()}{rack_markers()}
    <!-- ===== 랙: A/B/C 열 7베이씩 (21) + 좁은 통로 4베이 (rack_narrow_*) =====
         좁은 통로: 중심 ({NARROW_CX}, {NARROW_CY}), y 방향 4 m,
         순폭 {NARROW_CLEAR} m = 로봇 폭 0.40 + 0.20 (명세 4.4).
         북쪽 입구 y={NARROW_CY + 2}, 남쪽 출구 y={NARROW_CY - 2}.
         worker_crossing 이 입구 앞 y=-7 을 1.0 m/s 로,
         worker_slow_south 가 출구 앞 y=-14.5 를 0.3 m/s 로 지난다. -->
{"".join(include("rack", n, x, y, 0.0, yaw) for n, x, y, yaw in racks)}
    <!-- ===== 랙 위 화물 (정적) ===== -->
{rack_boxes(racks)}
    <!-- ===== 도크 / 충전 / 대기 구역 (마커: DICT_4X4_50, dock 0~3, charger 4~6) ===== -->
{docks_and_zones()}
    <!-- ===== 표지판 {len(SIGN_POSES)}개 (models/sign 텍스처, 판 법선 = 보는 쪽) + 비상구 문 ===== -->
{signs()}
{dyn_xml}  </world>
</sdf>
"""
    return sdf, racks, specs


def _parse_args(argv):
    import argparse
    ap = argparse.ArgumentParser(
        description="warehouse.sdf 생성 (기본). --gt-dir 를 주면 지면 진실(GT) 파일 세트를 쓴다.")
    ap.add_argument("out", nargs="?", default=None,
                    help="출력 SDF (기본: 같은 디렉토리 warehouse.sdf; --gt-dir 만 주면 SDF 를 쓰지 않는다)")
    ap.add_argument("--variant", default=None, choices=VARIANTS,
                    help="평가용 월드 변형 (out 경로를 반드시 줘야 한다 — 커밋된 warehouse.sdf 보호)")
    ap.add_argument("--gt-dir", default=None, help="GT 출력 디렉토리 (export_gt 독스트링 참고)")
    ap.add_argument("--gt-models", action="append", default=None,
                    help="model:// 검색 경로 (기본: 이 파일의 ../models)")
    ap.add_argument("--gt-spacing", type=float, default=GT_SPACING, help="3D 표면 점 간격 [m]")
    ap.add_argument("--gt-res", type=float, default=GT_RES, help="2D 래스터 해상도 [m]")
    ap.add_argument("--gt-slice-z", type=float, action="append", default=None,
                    help=f"단면 높이 [m] (여러 번, 기본 {GT_SLICE_Z})")
    ap.add_argument("--gt-band", type=float, nargs=2, action="append", default=None,
                    metavar=("Z0", "Z1"), help=f"높이 대역 [m] (여러 번, 기본 {GT_BANDS})")
    ap.add_argument("--gt-exclude", action="append", default=None,
                    help=f"뺄 모델 이름 접두어 (여러 번, 기본 {GT_EXCLUDE})")
    ap.add_argument("--gt-legacy-exclude", action="store_true",
                    help=f"world_geometry 와 같은 제외 집합 {GT_LEGACY_EXCLUDE} — 0.20 m 단면 "
                         "검증 전용 (셔틀의 원기둥·구가 들어와 대역·점·메시에서는 빠진다; 대역 기본값 없음)")
    ap.add_argument("--gt-roi", type=float, nargs=4, default=None,
                    metavar=("XMIN", "XMAX", "YMIN", "YMAX"), help="3D 점을 이 xy 범위로 자른다")
    ap.add_argument("--gt-no-points", action="store_true", help="3D 표면 점 파일을 쓰지 않는다")
    return ap.parse_args(argv)


def main(argv=None):
    args = _parse_args(argv)
    here = os.path.dirname(os.path.abspath(__file__))
    if args.variant is not None and args.out is None:
        raise SystemExit("--variant 에는 출력 경로(out)를 줘야 한다 (커밋된 warehouse.sdf 를 덮어쓰지 않게)")
    if args.gt_dir is None or args.out is not None:
        out_path = args.out or os.path.join(here, "warehouse.sdf")
        sdf, racks, specs = build_sdf(args.variant)
        with open(out_path, "w") as f:
            f.write(sdf)
        n_act, n_veh = (0, 0) if args.variant == "s1" else (len(specs), len(VEHICLES))
        print(f"wrote {out_path}: racks={len(racks)} actors={n_act} vehicles={n_veh} "
              f"signs={len(SIGN_POSES)}" + (f" variant={args.variant}" if args.variant else ""))
        for name, v, wps, _ in specs:
            print(f"  {name:22s} {v} m/s loop={wps[-1][0]:.1f}s waypoints={len(wps)}")
    if args.gt_dir is not None:
        exclude = GT_LEGACY_EXCLUDE if args.gt_legacy_exclude else tuple(
            args.gt_exclude or GT_EXCLUDE)
        meta = export_gt(args.gt_dir, args.gt_models or [os.path.join(here, os.pardir, "models")],
                         spacing=args.gt_spacing, res=args.gt_res,
                         slices=tuple(args.gt_slice_z or GT_SLICE_Z),
                         bands=tuple(tuple(b) for b in (args.gt_band or (
                             () if args.gt_legacy_exclude else GT_BANDS))),
                         exclude=exclude, points=not args.gt_no_points, roi=args.gt_roi,
                         committed=os.path.join(here, "warehouse.sdf"))
        print(f"wrote GT to {args.gt_dir}: primitives={meta['primitives']} "
              f"points={meta.get('points', 0)} "
              f"matches_committed_sdf={meta['matches_committed_sdf']}")
        for s in meta["skipped"]:
            print(f"  skipped: {s}")


if __name__ == "__main__":
    main()
