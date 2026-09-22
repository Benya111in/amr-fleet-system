"""
config/fleet_spawn.yaml 의 스폰 자세가 월드(warehouse.sdf)의 빈 바닥인지 SDF 와 대조한다.

- 정적 충돌체: 월드의 <include>(models/<uri>/model.sdf)와 인라인 <model> 의 box 충돌체를 바닥 평면에 투영한
  회전 사각형. 로봇 외접원(0.60 x 0.40 → r 0.36 m) + 여유 STATIC_CLEARANCE 이상 떨어져야 한다.
- 동적 장애물: actor <trajectory> 웨이포인트 꺾은선과 TriggeredPublisher 로 x 왕복하는 물리 모델(지게차)의
  주행 구간. DYNAMIC_CLEARANCE 이상 떨어져야 한다 (actor 에는 충돌체가 없어 로봇을 뚫고 지나간다).
"""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from amr_bringup import fleet_spawn as fs
import pytest

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'fleet_spawn.yaml'
ROBOT_RADIUS = math.hypot(0.60 / 2, 0.40 / 2)   # config/robot_params.yaml footprint
STATIC_CLEARANCE = 0.5                           # [m] 외접원 바깥 여유
DYNAMIC_CLEARANCE = 3.0                          # [m] 로봇 중심 ↔ 동적 장애물 경로


def _simulation_share():
    """설치된 amr_simulation share, 없으면 소스 트리 (colcon test 는 소스의 test/ 를 돌린다)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('amr_simulation'))
        if (share / 'worlds' / 'warehouse.sdf').is_file():
            return share
    except (ImportError, LookupError):   # 패키지·색인 없음 → 소스 트리
        pass
    src = Path(__file__).resolve().parents[2] / 'amr_simulation'
    return src if (src / 'worlds' / 'warehouse.sdf').is_file() else None


def _pose(elem):
    """<pose> 텍스트 → (x, y, yaw). 없으면 원점."""
    node = elem.find('pose') if elem is not None else None
    if node is None or not (node.text or '').strip():
        return 0.0, 0.0, 0.0
    v = [float(t) for t in node.text.split()]
    return v[0], v[1], v[5]


def _compose(a, b):
    """평면 자세 합성: a 좌표계 안의 자세 b 를 a 의 부모 좌표계로."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return ax + c * bx - s * by, ay + s * bx + c * by, ath + bth


def _boxes(model_elem, model_pose):
    """모델의 box 충돌체 → [(cx, cy, yaw, sx, sy)] (월드 평면)."""
    out = []
    for link in model_elem.findall('link'):
        link_pose = _compose(model_pose, _pose(link))
        for col in link.findall('collision'):
            size = col.find('geometry/box/size')
            if size is None:
                continue
            sx, sy, _ = (float(t) for t in size.text.split())
            cx, cy, cyaw = _compose(link_pose, _pose(col))
            out.append((cx, cy, cyaw, sx, sy))
    return out


def _point_box_distance(px, py, box):
    """점과 회전 사각형 사이 거리 (안이면 0)."""
    cx, cy, yaw, sx, sy = box
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * (px - cx) - s * (py - cy), s * (px - cx) + c * (py - cy)
    dx, dy = max(abs(lx) - sx / 2, 0.0), max(abs(ly) - sy / 2, 0.0)
    return math.hypot(dx, dy)


def _point_segment_distance(px, py, a, b):
    (ax, ay), (bx, by) = a, b
    vx, vy = bx - ax, by - ay
    norm2 = vx * vx + vy * vy
    t = 0.0 if norm2 == 0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / norm2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def _world(share):
    root = ET.parse(share / 'worlds' / 'warehouse.sdf').getroot().find('world')
    static, paths = [], {}
    for inc in root.findall('include'):
        uri = inc.find('uri').text.strip()
        name = inc.find('name').text.strip()
        model_sdf = share / 'models' / uri.replace('model://', '') / 'model.sdf'
        model = ET.parse(model_sdf).getroot().find('model')
        pose = _pose(inc)
        matches = [float(m.text) for m in inc.iter('match') if m.get('field') == 'position.x']
        if matches:                                     # x 왕복 물리 모델 (지게차)
            paths[name] = [(min(matches), pose[1]), (max(matches), pose[1])]
        else:
            static += [(name, box) for box in _boxes(model, pose)]
    for model in root.findall('model'):
        static += [(model.get('name'), box) for box in _boxes(model, _pose(model))]
    for actor in root.findall('actor'):
        pts = [_pose(wp)[:2] for wp in actor.iter('waypoint')]
        paths[actor.get('name')] = pts
    return static, paths


@pytest.fixture(scope='module')
def world():
    share = _simulation_share()
    if share is None:
        pytest.skip('amr_simulation warehouse.sdf 를 찾을 수 없다')
    return _world(share)


@pytest.fixture(scope='module')
def spawn():
    return fs.load_fleet_spawn(str(CONFIG))


def test_world_parsed(world):
    static, paths = world
    names = {n for n, _ in static}
    assert {'warehouse_walls', 'rack_A1', 'pillar_1', 'charging_station_c1'} <= names
    assert {'forklift_main', 'worker_straight_fast', 'worker_random'} <= set(paths)


def test_spawn_poses_clear_of_static_collisions(world, spawn):
    static, _ = world
    for r in spawn.robots:
        name, box = min(static, key=lambda nb: _point_box_distance(r.x, r.y, nb[1]))
        gap = _point_box_distance(r.x, r.y, box) - ROBOT_RADIUS
        assert gap >= STATIC_CLEARANCE, f'{r.name} ↔ {name}: 여유 {gap:.2f} m'


def test_spawn_poses_clear_of_dynamic_paths(world, spawn):
    _, paths = world
    for r in spawn.robots:
        for name, pts in paths.items():
            d = min(_point_segment_distance(r.x, r.y, a, b) for a, b in zip(pts, pts[1:]))
            assert d >= DYNAMIC_CLEARANCE, f'{r.name} ↔ {name} 경로: {d:.2f} m'


def test_spawn_poses_inside_warehouse(spawn):
    for r in spawn.robots:
        assert abs(r.x) < 30.0 - 0.2 - ROBOT_RADIUS and abs(r.y) < 20.0 - 0.2 - ROBOT_RADIUS


def test_check_detects_blocked_pose(world):
    """검사 자체의 음성 대조: 랙 A1 중심과 worker_straight_fast 경로 위는 거절된다."""
    static, paths = world
    rack = next(box for n, box in static if n == 'rack_A1')
    assert _point_box_distance(rack[0], rack[1], rack) == 0.0
    pts = paths['worker_straight_fast']
    assert min(_point_segment_distance(22.0, 0.0, a, b) for a, b in zip(pts, pts[1:])) < 0.1
