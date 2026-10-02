"""behavior.yaml 구조·문서화 규칙, 도크 기하(월드·센서와 일치), behavior.launch.py 인자 처리 시험."""

import importlib.util
import math
import os
import sys

from launch import LaunchContext
from launch_ros.actions import Node
import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(PKG))
CONFIG = os.path.join(PKG, 'config', 'behavior.yaml')
WORLD_GEN = os.path.join(REPO, 'src', 'amr_simulation', 'worlds', 'gen_warehouse_world.py')
SENSORS = os.path.join(REPO, 'config', 'sensors.yaml')
ROBOT = os.path.join(REPO, 'config', 'robot_params.yaml')
LAUNCH = os.path.join(PKG, 'launch', 'behavior.launch.py')
TREES = os.path.join(PKG, 'behavior_trees')


def _load_launch_module():
    spec = importlib.util.spec_from_file_location('behavior_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _config():
    with open(CONFIG, encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_docks_have_staging_and_standoff():
    docks = _config()['/**']['ros__parameters']['docks']
    assert len(docks['ids']) >= 4
    for dock_id in docks['ids']:
        entry = docks[dock_id]
        assert len(entry['staging']) == 3
        assert 0.3 < entry['standoff'] < 1.5
        assert -math.pi - 1e-6 <= entry['staging'][2] <= math.pi + 1e-6


def test_executor_and_docking_parameters():
    cfg = _config()
    executor = cfg['/**/task_executor_node']['ros__parameters']
    docking = cfg['/**/docking_server_node']['ros__parameters']
    assert executor['dock_attempts'] == 3            # 명세: 최대 3회
    assert executor['dock_max_retries'] == 1         # 단일 재시도 카운터
    ids = cfg['/**']['ros__parameters']['docks']['ids']
    assert executor['charger_dock_ids'] and set(executor['charger_dock_ids']) <= set(ids)
    assert executor['allow_undocked_tasks'] is False   # 도크 없는 작업은 명시적 선택
    assert executor['error_hold_ms'] > 600           # fleet_adapter 2 Hz 샘플 + 지연 100 ms
    assert docking['position_tolerance'] <= 0.02     # 명세: 2 cm
    assert docking['angle_tolerance'] <= math.radians(1.0) + 1e-6
    assert docking['heading_stop_tolerance'] < docking['angle_tolerance']   # 1° 경계에서 멈추지 않는다
    assert docking['control_law'] in ('graceful', 'proportional')
    assert docking['max_linear_speed'] <= 0.2        # Critical 존 상한 이하
    # 계약 C3: aruco_detector_node 는 마커 모델 프레임(+x = 판 법선)으로 낸다
    assert docking['marker_normal_axis'] == 'x'
    # 계약 C2: 예외 사각형이 LiDAR 노이즈(σ 3 cm)의 3배 이상을 덮고, 범퍼(판 앞 0.35 m)는 덮지 않는다
    exclusion = docking['exclusion']
    assert exclusion['enabled'] is True
    assert 0.09 <= exclusion['front_margin'] < 0.35 - 0.10


def test_every_parameter_line_is_documented():
    """모든 스칼라 파라미터 줄에 의미·단위 주석이 있어야 한다 (명세 4.10 문서화)."""
    undocumented = []
    with open(CONFIG, encoding='utf-8') as f:
        for number, line in enumerate(f, 1):
            text = line.strip()
            if not text or text.startswith('#') or text.startswith('/'):
                continue
            if text.endswith(':') or text.startswith('ros__parameters'):
                continue
            key = text.split(':', 1)[0]
            if key in ('ids',) or text.startswith(('dock_', 'charger_c')):
                continue    # 도크 표는 블록 위 주석으로 설명
            if '#' not in text:
                undocumented.append(f'{number}: {text}')
    assert not undocumented, undocumented


def test_launch_arguments_and_nodes():
    module = _load_launch_module()
    ld = module.generate_launch_description()
    names = {a.name for a in ld.entities if hasattr(a, 'name')}
    for arg in ('namespace', 'use_sim_time', 'params_file', 'groot_publisher_port',
                'groot_server_port', 'waiting_pose', 'frame_prefix', 'start_battery_model',
                'battery_initial_percent'):
        assert arg in names
    context = LaunchContext()
    context.launch_configurations.update({
        'namespace': '/amr_02', 'robot_name': '', 'use_sim_time': 'false',
        'params_file': CONFIG, 'robot_params_file': '', 'frame_prefix': 'auto',
        'groot_publisher_port': '1668', 'groot_server_port': '1669',
        'waiting_pose': '1.0, 2.0, 0.5', 'start_executor': 'true', 'start_docking': 'true',
        'start_battery_model': 'true', 'battery_initial_percent': '25', 'log_level': 'debug'})
    nodes = module._launch_setup(context)
    assert len(nodes) == 3
    assert all(isinstance(n, Node) for n in nodes)
    assert {n.node_executable for n in nodes} == {
        'task_executor_node', 'docking_server_node', 'battery_model_node'}
    # amr_bringup 이 로봇 i 에 주는 Groot 포트(1666 + 2i / 1667 + 2i)가 실행기 파라미터로 들어간다
    values = dict(context.launch_configurations)
    executor, docking, battery = module._overrides(lambda name: values[name], 'amr_02/')
    assert executor['groot.publisher_port'] == 1668 and executor['groot.server_port'] == 1669
    assert executor['waiting_pose'] == [1.0, 2.0, 0.5]
    assert docking['base_frame'] == 'amr_02/base_link'
    assert battery['initial_percent'] == 25.0 and battery['use_sim_time'] is False
    context.launch_configurations.update({'start_docking': 'false', 'start_battery_model': 'false',
                                          'battery_initial_percent': ''})
    assert len(module._launch_setup(context)) == 1


def test_parse_pose():
    module = _load_launch_module()
    assert module._parse_pose('') is None
    assert module._parse_pose('1, 2, 3') == [1.0, 2.0, 3.0]
    with pytest.raises(ValueError):
        module._parse_pose('1,2')
    assert module._as_bool('True') and not module._as_bool('0')


def test_tree_files_exist_and_are_included():
    main = os.path.join(TREES, 'task_executor.xml')
    with open(main, encoding='utf-8') as f:
        text = f.read()
    for sub in ('move_to', 'perceive', 'dock_at', 'payload', 'recovery', 'charge', 'yield'):
        assert f'subtrees/{sub}.xml' in text
        assert os.path.isfile(os.path.join(TREES, 'subtrees', f'{sub}.xml'))


def _world():
    """amr_simulation 월드 생성기의 레이아웃 상수 (도크·충전소·마커)."""
    if not os.path.isfile(WORLD_GEN):
        pytest.skip('월드 생성기가 없다 (소스 트리 밖에서 실행)')
    spec = importlib.util.spec_from_file_location('gen_warehouse_world', WORLD_GEN)
    module = importlib.util.module_from_spec(spec)
    saved = sys.argv
    try:
        sys.argv = ['gen_warehouse_world.py']
        spec.loader.exec_module(module)
    finally:
        sys.argv = saved
    return module


def _extrinsics():
    with open(SENSORS, encoding='utf-8') as f:
        sensors = yaml.safe_load(f)['/**']['ros__parameters']
    with open(ROBOT, encoding='utf-8') as f:
        robot = yaml.safe_load(f)['/**']['ros__parameters']
    cam = sensors['camera_link']['extrinsic']
    return cam, sensors['rgb_camera'], robot


def _dock_geometry():
    """도크 id → (판 면 중심 x, y, 바깥 법선 방위) — 월드 생성기 값에서 유도."""
    w = _world()
    faces = {}
    for name, (_x, y, kind, _mid) in w.DOCKS.items():
        if kind == 'inbound':
            faces[name] = (-w.HALF_X + 0.02, y, 0.0)      # 판 중심 −HALF_X + 0.01, 두께 0.02
        else:
            faces[name] = (w.HALF_X - 0.02, y, math.pi)
    for name, (x, _mid) in w.CHARGERS.items():
        faces[f'charger_{name}'] = (x, w.CHARGER_Y + w.STATION_FRONT + 0.02, math.pi / 2)
    return w, faces


def test_dock_table_matches_world_and_camera_geometry():
    """
    도크 표가 월드(마커 판)·센서(카메라) 배치와 맞는다 — 레이아웃이 바뀌면 이 시험이 먼저 깨진다.

    staging 은 판 법선 위에서 판을 마주 보고, 카메라에 마커가 검출 하한의 2배 이상으로 보이며, 판 중심
    높이는 카메라 광학 중심 높이와 같다. standoff 에서 범퍼–판은 safety 정지 거리(0.30 m)보다 크다.
    """
    w, faces = _dock_geometry()
    cam, rgb, robot = _extrinsics()
    cfg = _config()
    docks = cfg['/**']['ros__parameters']['docks']
    aruco_min_px = 12.0                                   # perception.yaml aruco min_side_px
    f_px = 0.5 * rgb['width'] / math.tan(0.5 * rgb['hfov'])
    assert abs(robot['robot']['base_link_height'] + cam['z'] - w.MARKER_Z) < 1e-6
    assert set(faces) == set(docks['ids'])                # 월드의 도크·충전소 = 도크 표
    mids = {name: mid for name, (_x, _y, _k, mid) in w.DOCKS.items()}
    mids.update({f'charger_{c}': mid for c, (_x, mid) in w.CHARGERS.items()})
    assert {d: docks[d]['marker_id'] for d in docks['ids']} == mids   # ArUco id = 월드 텍스처
    for dock_id, (fx, fy, normal) in faces.items():
        sx, sy, syaw = docks[dock_id]['staging']
        dx, dy = sx - fx, sy - fy
        along = dx * math.cos(normal) + dy * math.sin(normal)
        across = -dx * math.sin(normal) + dy * math.cos(normal)
        assert abs(across) < 0.02, dock_id                # 판 법선 위
        assert abs(math.remainder(syaw - (normal + math.pi), 2 * math.pi)) < 1e-3, dock_id
        cam_dist = along - cam['x']
        assert f_px * 0.18 / cam_dist >= 2 * aruco_min_px, dock_id
        assert 1.3 <= along <= 2.3, dock_id               # docking.md §2.3 표의 값
        bumper_gap = docks[dock_id]['standoff'] - 0.5 * robot['robot']['footprint_length']
        assert bumper_gap > robot['safety']['emergency_stop_distance'], dock_id


def test_perceive_yaw_points_the_camera_at_the_dock_items():
    """
    리뷰 회귀: staging 방위에서는 도크 박스가 카메라 시야(±43.5°) 밖 → perceive_yaw 로 돌아보면 들어온다.

    staging 에서 제자리 회전(외접원)이 도크 박스와 safety 정지 거리 0.30 m + LiDAR 3σ 이상 떨어져 있다.
    """
    w, _ = _dock_geometry()
    cam, rgb, _ = _extrinsics()
    cfg = _config()
    docks = cfg['/**']['ros__parameters']['docks']
    max_distance = cfg['/**/task_executor_node']['ros__parameters']['perception_max_distance']
    half_fov = 0.5 * rgb['hfov']
    for name, (x, y, kind, _mid) in w.DOCKS.items():
        entry = docks[name]
        sx, sy, syaw = entry['staging']
        bx = x - (1.0 if kind == 'inbound' else -1.0) * 1.2     # 월드 생성기의 도크 박스 x
        boxes = [(bx, y + 1.0), (bx, y - 1.0)]

        def bearing(yaw, box):
            cx, cy = sx + cam['x'] * math.cos(yaw), sy + cam['x'] * math.sin(yaw)
            return abs(math.remainder(math.atan2(box[1] - cy, box[0] - cx) - yaw, 2 * math.pi))
        # staging 방위로는 두 박스 모두 시야 밖 (돌아보기가 필요한 이유)
        assert all(bearing(syaw, b) > half_fov for b in boxes), name
        # perceive_yaw 로 돌면 적어도 한 박스가 시야 중앙 부근·인식 거리 안
        yaw = entry['perceive_yaw']
        seen = [b for b in boxes if bearing(yaw, b) < math.radians(15.0)]
        assert seen, name
        assert all(math.hypot(b[0] - sx, b[1] - sy) <= max_distance for b in seen), name
        # 제자리 회전 여유: 박스(최대 0.6 × 0.5, 무작위 방위)의 가장 먼 모서리 반경 0.39 m 를 빼고
        circ = math.hypot(0.30, 0.20)
        for b in boxes:
            clearance = math.hypot(b[0] - sx, b[1] - sy) - math.hypot(0.30, 0.25) - circ
            assert clearance > 0.30 + 3 * 0.03, (name, round(clearance, 3))


def test_chargers_in_world_are_all_candidates():
    w, _ = _dock_geometry()
    executor = _config()['/**/task_executor_node']['ros__parameters']
    assert sorted(executor['charger_dock_ids']) == sorted(f'charger_{c}' for c in w.CHARGERS)
