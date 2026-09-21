#!/usr/bin/env python3
"""
warehouse.sdf 생성기 — 60 m x 40 m 물류센터 (명세 4.1 시뮬레이터 환경, 8장 레이아웃 예시).

    python3 gen_warehouse_world.py              # 같은 디렉토리의 warehouse.sdf 를 다시 쓴다
    python3 gen_warehouse_world.py out.sdf

생성물(warehouse.sdf)을 커밋하므로 빌드 시 실행할 필요는 없다. 레이아웃을 바꾸면 이 파일을 고치고 다시 생성한다.
의존성: 표준 라이브러리만.

좌표계
  원점 = 창고 중심, x = 60 m 축(+x 동쪽 = 출고구역), y = 40 m 축(+y 북쪽 = 도크/메인 통로), 바닥 z = 0.
  기본 스폰(0, 0, yaw 0)은 랙 B열-C열 사이 5 m 통로 한가운데다.

레이아웃 (명세 8장 예시를 따름; 단위 m)
  북쪽 y≈+15 : 메인 통로 (지게차 순찰 x -20..20)
  북서 (-27.5, 17/13): 입고 Dock-1/2,  북동 (27.5, 17/13): 출고 Dock-A/B  (마커는 벽면)
  y=+9 / +3 / -3 : 랙 A/B/C 열, 각 7베이 (x = -18, -12, ..., 18; 베이 2.0 x 1.0 x 2.5)
  (0, -10) : 좁은 통로 테스트 구간 — 랙 4베이를 90도 돌려 세워 순폭 0.60 m(로봇 폭 0.40 + 0.20), 길이 4 m, y 방향
  남서 (-22, -16) : 충전 구역 C1~C3 (남쪽 벽 앞, 스테이션마다 마커),  남동 (22, -16): 대기 구역
  기둥 8개: (±10, ±12), (±25, ±6)
  ArUco 마커 (DICT_4X4_50, 판 0.30 m, 흑백 영역 0.18 m, 중심 높이 MARKER_Z):
    Dock-1/2/A/B = id 0~3, C1~C3 = id 4~6

동적 장애물 6개 (0.3~1.5 m/s, 직선/곡선/무작위)
  worker_straight_slow  0.5 m/s  A-B 통로(y=6) x -16..16 왕복            직선
  worker_straight_fast  1.2 m/s  동측 통로(x=22) y -10..10 왕복          직선
  worker_curve          0.8 m/s  서측 공터 (-24, 0) 반지름 3.5 원 순환    곡선
  worker_random         0.6 m/s  통로 격자 위 무작위 보행 (seed 42)      무작위
  forklift_main         1.5 m/s  메인 통로(y=15) x -20..20 왕복 (물리 모델) 직선
  worker_crossing       1.0 m/s  좁은 통로 입구 앞(y=-7) x -6..6 횡단     직선 (명세 4.7 테스트)
  actor 의 웨이포인트 시간은 구간 길이/속도로 계산하고, 방향 전환 시 TURN_TIME 만큼 제자리 회전한다.
  직선 구간은 LEG_STEP 간격으로 나누고 trajectory tension 0 을 써서 구간 안 속도를 일정하게 만든다 (leg_waypoints).
  "무작위" 패턴은 고정 seed 의 난수 보행으로 미리 만든 긴 비반복 웨이포인트 목록이다 (실행 중 난수 아님).

Fortress 의 actor 는 서버(물리)에서 움직이지 않는다: 궤적은 렌더링 쪽(Sensors 시스템의 RenderUtil)이 sim time 으로
계산하므로 로봇의 LiDAR/카메라에는 움직임이 보이지만 /world/warehouse/pose/info 에는 actor 포즈가 없다.
actor 의 지면 진실 포즈가 필요하면 SDF 의 <trajectory> 웨이포인트를 sim time 으로 보간한다 (tension 0, 등간격 직선 →
선형 보간과 거의 같다). 물리 모델인 지게차는 pose/info 와 /model/forklift_main/pose 에 나온다.
actor 에는 충돌체가 없다(로봇이 물리적으로 통과함) → 충돌 판정은 위 보간 궤적과의 거리로 평가한다.
"""
import math
import os
import random
import sys
from collections import deque

SEED = 42                       # 무작위 보행/상자 배치 난수 seed
TURN_TIME = 0.5                 # [s] actor 제자리 방향 전환 시간
LEG_STEP = 1.0                  # [m] 직선 구간 웨이포인트 간격 (등속 보장, leg_waypoints 참고)

# ---- 치수 -------------------------------------------------------------------
HALF_X, HALF_Y = 30.0, 20.0     # 창고 반폭: 60 x 40 m
WALL_T, WALL_H = 0.2, 4.0
# 바닥 마찰: 건조 콘크리트 vs 우레탄/고무 바퀴 μ≈0.6~0.85 (Engineering ToolBox, rubber-concrete dry)
FLOOR_MU = 0.8
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
PILLARS = [(-10, 12), (10, 12), (-10, -12), (10, -12), (-25, 6), (25, 6), (-25, -6), (25, -6)]
# 마커 판 중심 높이 = AMR 카메라 광학 중심의 지면 높이
#   = config/robot_params.yaml base_link_height 0.18 + config/sensors.yaml camera_link z 0.25
MARKER_Z = 0.43
DECK_H = 0.50                   # 랙 하단 적재 블록 상면 (models/rack, LiDAR 평면 지면 +0.38 m 를 덮음)
SHELF1_Z = 1.275                # 랙 1단 선반 상면 (선반 중심 1.25 + 두께/2)

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
    hx, hy = HALF_X - WALL_T / 2, HALF_Y - WALL_T / 2
    size = f"{2 * HALF_X:.0f} {2 * HALF_Y:.0f}"
    plane = f"<plane><normal>0 0 1</normal><size>{size}</size></plane>"
    return f"""    <!-- 바닥 60 x 40 m. μ={FLOOR_MU}:
         건조 콘크리트-고무(우레탄) 바퀴 0.6~0.85 범위의 중간값 (Engineering ToolBox) -->
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
            <ambient>0.55 0.55 0.53 1</ambient><diffuse>0.55 0.55 0.53 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- 외벽 (두께 {WALL_T} m, 높이 {WALL_H} m),
         안쪽 면 x=±{HALF_X - WALL_T:.1f}, y=±{HALF_Y - WALL_T:.1f} -->
    <model name="warehouse_walls">
      <static>true</static>
      <link name="link">
""" + "".join(
        f"""        <collision name="{n}_collision">
          <pose>{pose(cx, cy, WALL_H / 2)}</pose>
          <geometry><box><size>{sx} {sy} {WALL_H}</size></box></geometry>
        </collision>
        <visual name="{n}">
          <pose>{pose(cx, cy, WALL_H / 2)}</pose>
          <geometry><box><size>{sx} {sy} {WALL_H}</size></box></geometry>
          <material>
            <ambient>0.75 0.75 0.72 1</ambient><diffuse>0.75 0.75 0.72 1</diffuse>
          </material>
        </visual>
"""
        for n, cx, cy, sx, sy in [
            ("wall_north", 0, hy, 2 * HALF_X, WALL_T),
            ("wall_south", 0, -hy, 2 * HALF_X, WALL_T),
            ("wall_east", hx, 0, WALL_T, 2 * HALF_Y),
            ("wall_west", -hx, 0, WALL_T, 2 * HALF_Y),
        ]
    ) + """      </link>
    </model>
"""


def lights():
    """
    태양광(천창/개구부 대용 방향광) + 통로 위 고천장 LED 포인트 조명.

    실제 물류센터는 7~10 m 천장에 LED 하이베이를 통로 축을 따라 배치한다(약 300 lx). 포인트 조명을 통로 위 z=7 m 에
    12개 두어 카메라 영상에 통로별 밝기 차이·그림자가 생기게 한다(도메인 랜덤화/마커 검출의 현실적 조건).
    ogre2 는 forward-clustered 조명이라 이 정도 광원 수는 성능에 문제없다.
    """
    out = """    <!-- 방향광(sun): 천창/개구부 채광 대용, 그림자 켬.
         highbay_*: 통로 축 위 z=7 m LED 하이베이(실제 7~10 m 천장, 약 300 lx) 12개, 그림자 끔.
         통로마다 밝기가 달라 카메라/마커 검출이 균일 조명보다 현실적인 조건을 겪는다.
         visualize=false: Fortress 는 광원 표시 기호(녹색 십자/사각형)를 카메라 센서 영상에도 그린다
         (실측) → 로봇 카메라에 인공물이 찍히지 않게 끈다. -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <visualize>false</visualize>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.7 0.7 0.7 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
"""
    n = 0
    for y in (15.0, 6.0, -6.0, -15.0):
        for x in (-18.0, -6.0, 6.0, 18.0):
            if y in (15.0, -15.0) and x in (-6.0, 6.0):
                continue
            out += f"""    <light type="point" name="highbay_{n}">
      <cast_shadows>false</cast_shadows>
      <visualize>false</visualize>
      <pose>{pose(x, y, 7.0)}</pose>
      <diffuse>0.8 0.8 0.75 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>25</range><constant>0.3</constant><linear>0.02</linear><quadratic>0.004</quadratic>
      </attenuation>
    </light>
"""
            n += 1
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


MARKER_TEX = "model://dock_marker/materials/textures/aruco_4x4_50_"


def marker_model(name, mid, x, y, yaw):
    """models/dock_marker/model.sdf 와 같은 판을 id 별 텍스처로 인라인 생성. +x 법선이 로봇 쪽."""
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{pose(x, y, 0, 0, 0, yaw)}</pose>
      <link name="link">
        <collision name="plate_collision">
          <pose>0 0 {MARKER_Z} 0 0 0</pose>
          <geometry><box><size>0.02 0.30 0.30</size></box></geometry>
        </collision>
        <visual name="plate">
          <pose>0 0 {MARKER_Z} 0 0 0</pose>
          <geometry><box><size>0.02 0.30 0.30</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
            <pbr><metal>
              <albedo_map>{MARKER_TEX}{mid}.png</albedo_map>
              <metalness>0.0</metalness><roughness>0.9</roughness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>
"""


def docks_and_zones():
    out = ""
    wall_in = HALF_X - WALL_T
    for name, (x, y, kind, mid) in DOCKS.items():
        out += include(f"dock_pad_{kind}", f"{name}_pad", x, y)
        # 마커: 벽 안쪽 면에 밀착, 로봇이 있는 창고 안쪽을 향한다
        if kind == "inbound":
            out += marker_model(f"{name}_marker", mid, -wall_in + 0.01, y, 0.0)
        else:
            out += marker_model(f"{name}_marker", mid, wall_in - 0.01, y, math.pi)
        # 도크 위 화물 2개 (동적 — 밀리는 물체)
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
        out += marker_model(f"{cs}_marker", mid, x, CHARGER_Y + 0.26, math.pi / 2)
    out += include("waiting_zone", "waiting_zone_pad", 22.0, -16.0)
    return out


def pillars():
    return "".join(include("pillar", f"pillar_{i}", x, y) for i, (x, y) in enumerate(PILLARS, 1))


# ---- 동적 장애물 ------------------------------------------------------------
def leg_waypoints(nodes, speed, turn=TURN_TIME, close_loop=True, step=LEG_STEP):
    """
    노드 목록을 따라 (t, x, y, yaw) 웨이포인트를 만든다.

    구간 시간 = 길이/속도, 방향 전환은 제자리 회전.

    직선 구간은 step 간격으로 잘게 나눈다: Fortress 는 웨이포인트를 Hermite 스플라인으로 잇는데 접선이
    0.5·(p[i+1]-p[i-1])·(1-tension) 이라, 등간격 일직선 점 + tension 0 이면 접선 = 현(chord) 이 되어 구간 안 속도가
    정확히 일정하다. (긴 구간 하나 + tension 1 은 접선 0 → smoothstep 완급으로 중앙 속도가 평균의 1.5 배가 된다.)
    방향 전환 지점의 같은 위치 두 점(yaw 만 다름) 사이에서는 스플라인이 현/8 (≈0.125 m) 만큼 살짝 부푼다.
    """
    wps = []
    t = 0.0
    yaw_prev = None
    for k in range(len(nodes) - 1):
        (x0, y0), (x1, y1) = nodes[k], nodes[k + 1]
        d = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        if yaw_prev is None:
            wps.append((t, x0, y0, yaw))
        elif abs((yaw - yaw_prev + math.pi) % (2 * math.pi) - math.pi) > 1e-6:
            t += turn
            wps.append((t, x0, y0, yaw))
        n = max(1, int(round(d / step)))
        for j in range(1, n + 1):
            s = j / n
            t += (d / n) / speed
            wps.append((t, x0 + s * (x1 - x0), y0 + s * (y1 - y0), yaw))
        yaw_prev = yaw
    if close_loop:
        x0, y0, yaw0 = wps[0][1], wps[0][2], wps[0][3]
        assert math.hypot(nodes[-1][0] - x0, nodes[-1][1] - y0) < 1e-9, "loop must return to start"
        if abs((yaw0 - yaw_prev + math.pi) % (2 * math.pi) - math.pi) > 1e-6:
            t += turn
            wps.append((t, x0, y0, yaw0))
    return wps


def circle_waypoints(cx, cy, r, speed, n=36):
    """반시계 원 순환. 구간 시간 = 호 길이/속도 (36분할이라 현/호 차이 0.04 %)."""
    wps = []
    seg_t = (2 * math.pi * r / n) / speed
    for i in range(n + 1):
        th = 2 * math.pi * i / n
        wps.append((i * seg_t, cx + r * math.cos(th), cy + r * math.sin(th), th + math.pi / 2))
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


def actor_xml(name, wps, tension):
    body = "".join(f"          <waypoint><time>{t:.3f}</time>"
                   f"<pose>{pose(x, y, 0, 0, 0, yaw)}</pose></waypoint>\n"
                   for t, x, y, yaw in wps)
    return f"""    <actor name="{name}">
      <pose>0 0 0 0 0 0</pose>
      <skin><filename>model://worker/meshes/worker.dae</filename><scale>1.0</scale></skin>
      <animation name="walk"><filename>model://worker/meshes/worker.dae</filename></animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk" tension="{tension}">
{body}        </trajectory>
      </script>
    </actor>
"""


def forklift_xml(x0, x1, y, speed):
    """
    물리 모델 지게차: VelocityControl + PosePublisher(50 Hz) + TriggeredPublisher 2개(양 끝 반전).

    - VelocityControl 은 매 스텝 모델 속도를 (v, 0, 0)/각속도 0 으로 다시 쓰므로 로봇이 부딪혀도 진로가 유지된다
      (TrajectoryFollower 는 힘 기반이라 속도가 정확히 안 나와 쓰지 않는다). 바퀴 충돌체가 바닥에 닿아 있어야
      스텝 사이에 적분되는 중력 낙하가 접촉으로 상쇄된다 (models/forklift 참고).
    - TriggeredPublisher 는 /model/forklift_main/pose 의 position.x 가 목표 ±tol 안이면 반대 방향 Twist 를 낸다.
      x 만 비교하므로 y 가 조금 밀려도 반전은 계속된다. 첫 일치는 목표에 tol 만큼 못 미친 곳이므로 목표를 tol 만큼
      바깥에 둔다 → 실제 반전 위치 ≈ x0, x1.
    - 지게차 지면 진실 포즈는 /model/forklift_main/pose (ignition.msgs.Pose) 로도 얻을 수 있다 (추적기 평가용).
    """
    tol = 0.25
    sysm = "ignition::gazebo::systems::"
    lib = "ignition-gazebo-"
    topic = "/model/forklift_main"
    plugins = f"""
      <plugin filename="{lib}velocity-control-system" name="{sysm}VelocityControl">
        <initial_linear>{speed} 0 0</initial_linear>
      </plugin>
      <plugin filename="{lib}pose-publisher-system" name="{sysm}PosePublisher">
        <!-- Fortress PosePublisher 는 모델 자신의 포즈를
             publish_nested_model_pose && publish_model_pose 일 때만 낸다 (하위 호환 코드).
             중첩 모델은 없으므로 둘 다 true. 링크/시각/충돌 포즈는 끈다 -->
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
      <plugin filename="{lib}triggered-publisher-system" name="{sysm}TriggeredPublisher">
        <input type="ignition.msgs.Pose" topic="{topic}/pose">
          <match field="position.x" tol="{tol}">{x1 + tol:.2f}</match>
        </input>
        <output type="ignition.msgs.Twist" topic="{topic}/cmd_vel">linear: {{x: {-speed}}}</output>
      </plugin>
      <plugin filename="{lib}triggered-publisher-system" name="{sysm}TriggeredPublisher">
        <input type="ignition.msgs.Pose" topic="{topic}/pose">
          <match field="position.x" tol="{tol}">{x0 - tol:.2f}</match>
        </input>
        <output type="ignition.msgs.Twist" topic="{topic}/cmd_vel">linear: {{x: {speed}}}</output>
      </plugin>"""
    return include("forklift", "forklift_main", x0, y, 0.0, 0.0, extra=plugins)


def dynamic_obstacles():
    out = "    <!-- ===== 동적 장애물 (스크립트 actor 5 + 물리 모델 지게차 1) ===== -->\n"
    # tension 은 모두 0 (Catmull-Rom): 등간격 웨이포인트와 함께 써야 구간 안 속도가 일정하다 (leg_waypoints 참고)
    specs = [
        ("worker_straight_slow", 0.5, "0",
         leg_waypoints([(-16.0, 6.0), (16.0, 6.0), (-16.0, 6.0)], 0.5),
         "직선 왕복, A-B 통로"),
        ("worker_straight_fast", 1.2, "0",
         leg_waypoints([(22.0, -10.0), (22.0, 10.0), (22.0, -10.0)], 1.2),
         "직선 왕복, 동측 통로"),
        ("worker_curve", 0.8, "0", circle_waypoints(-24.0, 0.0, 3.5, 0.8),
         "곡선(원) 순환, 서측 공터"),
        ("worker_random", 0.6, "0", leg_waypoints(random_walk_nodes(), 0.6),
         f"무작위 보행 (seed {SEED}, gen_warehouse_world.py random_walk_nodes)"),
        ("worker_crossing", 1.0, "0",
         leg_waypoints([(-6.0, -7.0), (6.0, -7.0), (-6.0, -7.0)], 1.0),
         "좁은 통로 입구 횡단 (명세 4.7: 1.0 m/s 동적 장애물)"),
    ]
    for name, v, tension, wps, desc in specs:
        out += f"    <!-- {name}: {v} m/s, {desc}, 루프 {wps[-1][0]:.1f} s, 웨이포인트 {len(wps)}개 -->\n"
        out += actor_xml(name, wps, tension)
    out += "    <!-- forklift_main: 1.5 m/s, 메인 통로(y=15) x -20..20 직선 왕복, 물리 모델(충돌 있음) -->\n"
    out += forklift_xml(-20.0, 20.0, 15.0, 1.5)
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
def check_layout(racks, specs):
    aabbs = []
    for name, x, y, yaw in racks:
        hx, hy = (RACK_L / 2, RACK_D / 2) if abs(math.sin(yaw)) < 0.5 else (RACK_D / 2, RACK_L / 2)
        aabbs.append((name, x - hx, x + hx, y - hy, y + hy))
    for i, (x, y) in enumerate(PILLARS, 1):
        aabbs.append((f"pillar_{i}", x - 0.25, x + 0.25, y - 0.25, y + 0.25))
    margin = 0.35
    for name, x0, x1, y0, y1 in aabbs:
        assert not (x0 - 1.0 < 0 < x1 + 1.0 and y0 - 1.0 < 0 < y1 + 1.0), \
            f"스폰 셀 (0,0) 이 {name} 과 겹침"
    for aname, _, _, wps, _ in specs:
        for t, x, y, _ in wps:
            assert abs(x) < HALF_X - WALL_T - 0.5 and abs(y) < HALF_Y - WALL_T - 0.5, \
                f"{aname} 웨이포인트 벽 밖: {x},{y}"
            for name, x0, x1, y0, y1 in aabbs:
                assert not (x0 - margin < x < x1 + margin and y0 - margin < y < y1 + margin), \
                    f"{aname} 웨이포인트 ({x},{y}) 가 {name} 안"
    # 좁은 통로 순폭
    w = [r for r in racks if r[0].startswith("rack_narrow_W")][0]
    e = [r for r in racks if r[0].startswith("rack_narrow_E")][0]
    clear = (e[1] - RACK_D / 2) - (w[1] + RACK_D / 2)
    assert abs(clear - NARROW_CLEAR) < 1e-6, clear


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "warehouse.sdf")
    racks = rack_instances()
    dyn_xml, specs = dynamic_obstacles()
    check_layout(racks, specs)

    sdf = f"""<?xml version="1.0" ?>
<!--
  물류센터 월드 60 x 40 m — gen_warehouse_world.py 가 생성 (seed {SEED}). 직접 수정하지 말고 생성기를 고칠 것.
  레이아웃/좌표계/동적 장애물 설명은 생성기 docstring 참고. 기동은 launch/warehouse.launch.py.

  월드 플러그인
    Physics          : 1 ms 고정 스텝, RTF 1.0.
                       (gz-sim Fortress 는 <physics type> 과 무관하게 ign-physics DART 를 쓴다)
    UserCommands     : ros_gz_sim create(스폰)/set_pose 서비스
    SceneBroadcaster : pose/info, scene/info (GUI, ign model 도구)
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
      <ambient>0.35 0.35 0.35 1</ambient>
      <background>0.6 0.65 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- ===== 조명 ===== -->
{lights()}
    <!-- ===== 바닥 / 외벽 ===== -->
{ground_and_walls()}
    <!-- ===== 기둥 {len(PILLARS)}개 ===== -->
{pillars()}
    <!-- ===== 랙: A/B/C 열 7베이씩 (21) + 좁은 통로 4베이 (rack_narrow_*) =====
         좁은 통로: 중심 ({NARROW_CX}, {NARROW_CY}), y 방향 4 m,
         순폭 {NARROW_CLEAR} m = 로봇 폭 0.40 + 0.20 (명세 4.4).
         북쪽 입구 y={NARROW_CY + 2}, 남쪽 출구 y={NARROW_CY - 2}.
         worker_crossing 이 입구 앞 y=-7 을 1.0 m/s 로 횡단한다. -->
{"".join(include("rack", n, x, y, 0.0, yaw) for n, x, y, yaw in racks)}
    <!-- ===== 랙 위 화물 (정적) ===== -->
{rack_boxes(racks)}
    <!-- ===== 도크 / 충전 / 대기 구역 (마커: DICT_4X4_50, dock 0~3, charger 4~6) ===== -->
{docks_and_zones()}
{dyn_xml}  </world>
</sdf>
"""
    with open(out_path, "w") as f:
        f.write(sdf)
    print(f"wrote {out_path}: racks={len(racks)} actors={len(specs)}")
    for name, v, _, wps, _ in specs:
        print(f"  {name:22s} {v} m/s loop={wps[-1][0]:.1f}s waypoints={len(wps)}")


if __name__ == "__main__":
    main()
