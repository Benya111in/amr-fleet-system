"""
배포 nav2_params.yaml 의 코스트맵 관측 소스 높이 창 ↔ 센서 평면.

리뷰: Humble 소스별 max_obstacle_height 기본값 0.0 때문에 LiDAR 점이 전부 버려졌는데 어떤 시험도
실제 설정을 읽지 않았다.

navigation.launch.py 와 같은 템플릿 경로(ReplaceString → RewrittenYaml(root_key))로 파일을 만든 뒤, 센서 높이는
config/robot_params.yaml (base_link_height) + config/sensors.yaml (extrinsic) 에서 계산한다:
  - LaserScan 소스: 창이 LiDAR 평면(지면 +0.20)을 ±0.1 m 이상 여유로 담는다
  - PointCloud2 마킹 소스: 하한이 바닥 잡음(실측 p99.9 < 0.01 m)보다 위, LiDAR 평면 아래 물체(소형 박스 0.15 m,
    지게차 포크 0.05~0.10 m)를 담고, 상한이 카메라 높이(0.25 m)보다 위
  - 모든 소스가 min/max 를 명시한다 (기본값에 기대지 않는다)
실제 코스트맵 마킹 동작은 test_costmap_marking.cpp 가 같은 파일로 확인한다.
"""
import os
from pathlib import Path

from launch import LaunchContext
from nav2_common.launch import ReplaceString, RewrittenYaml
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
REPO_CONFIG = PKG.parents[1] / 'config'
# NAV2_PARAMS_OVERRIDE: 다른 파일로 같은 검사 (예: 리뷰 전 설정이 여기 걸리는지 확인)
PARAMS = Path(os.environ.get('NAV2_PARAMS_OVERRIDE', PKG / 'config' / 'nav2_params.yaml'))


def _root(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)['/**']['ros__parameters']


@pytest.fixture(scope='module')
def sensors():
    robot = _root(REPO_CONFIG / 'robot_params.yaml')['robot']
    sens = _root(REPO_CONFIG / 'sensors.yaml')
    base = robot['base_link_height']
    return {
        'lidar_z': base + sens['lidar']['extrinsic']['z'],
        'camera_z': base + sens['camera_link']['extrinsic']['z'],
        'lidar_frame': sens['lidar']['frame_id'],
    }


@pytest.fixture(scope='module')
def params():
    """navigation.launch.py 와 같은 치환 (다중 로봇 amr_01, 접두어 amr_01/)."""
    ctx = LaunchContext()
    templated = ReplaceString(source_file=str(PARAMS),
                              replacements={'<prefix>': 'amr_01/', '<robot_ns>': '/amr_01'})
    rewritten = RewrittenYaml(source_file=templated, root_key='amr_01',
                              param_rewrites={'use_sim_time': 'true', 'autostart': 'true'},
                              convert_types=True)
    with open(rewritten.perform(ctx), encoding='utf-8') as f:
        return yaml.safe_load(f)['amr_01']


def _sources(p):
    """(코스트맵, 레이어, 소스 이름, 레이어 dict, 소스 dict) — observation_sources 를 가진 모든 레이어."""
    out = []
    for cm in ('local_costmap', 'global_costmap'):
        rp = p[cm][cm]['ros__parameters']
        for layer in rp['plugins']:
            lp = rp[layer]
            for src in lp.get('observation_sources', '').split():
                out.append((cm, layer, src, lp, lp[src]))
    return out


def test_every_source_sets_explicit_height_window(params):
    srcs = _sources(params)
    assert len(srcs) >= 6   # 두 코스트맵 × (LiDAR + 깊이 마킹 + 깊이 소거)
    for cm, layer, src, _, sp in srcs:
        assert 'min_obstacle_height' in sp, f'{cm}.{layer}.{src}: Humble 기본 0.0 에 기대면 안 된다'
        assert 'max_obstacle_height' in sp, f'{cm}.{layer}.{src}'
        assert sp['max_obstacle_height'] > sp['min_obstacle_height']


def test_scan_sources_contain_lidar_plane(params, sensors):
    z = sensors['lidar_z']
    assert z == pytest.approx(0.20)   # 재배치 후 LiDAR 평면 (sensors.yaml 주석과 같은 값)
    scans = [s for s in _sources(params) if s[4]['data_type'] == 'LaserScan']
    assert {s[0] for s in scans} == {'local_costmap', 'global_costmap'}
    for cm, layer, src, lp, sp in scans:
        assert sp['min_obstacle_height'] <= z - 0.1, f'{cm}.{layer}.{src}'
        assert sp['max_obstacle_height'] >= z + 0.1, f'{cm}.{layer}.{src}'
        assert lp.get('max_obstacle_height', 2.0) >= sp['max_obstacle_height']
        assert sp['marking'] and sp['clearing']
        assert sp['observation_persistence'] == 0.0     # 동적 장애물: 최신 스캔만
        # 게이트 중앙값 필터 출력 (절대 이름). 전역은 동적 트랙 제외본 (costmap.md §5.2)
        want = '/amr_01/scan_costmap' if cm == 'local_costmap' else '/amr_01/scan_costmap_static'
        assert sp['topic'] == want, f'{cm}.{layer}.{src}'


def test_depth_sources_mark_below_lidar_plane(params, sensors):
    # 깊이 레이어의 소스만 본다. 전역에는 예측 점유(predicted_layer)도 PointCloud2 인데
    # 그것은 깊이 센서가 아니라 추적기 예측이라 규약이 다르다 (아래 별도 계약).
    depth = [s for s in _sources(params)
             if s[4]['data_type'] == 'PointCloud2' and 'depth' in s[1]]
    marking = [s for s in depth if s[4]['marking']]
    clearing = [s for s in depth if s[4]['clearing']]
    assert {s[0] for s in marking} == {'local_costmap', 'global_costmap'}
    assert {s[0] for s in clearing} == {'local_costmap', 'global_costmap'}
    for cm, layer, src, lp, sp in depth:
        want = ('/amr_01/camera/depth/points_filtered' if cm == 'local_costmap'
                else '/amr_01/camera/depth/points_static')
        assert sp['topic'] == want, f'{cm}.{layer}.{src}'
    for cm, layer, src, lp, sp in marking:
        # 바닥 잡음 위 (실측 p99.9 < 0.01 m) 이고 포크(0.05~0.10)·소형 박스(0.15) 를 담는다
        assert 0.02 <= sp['min_obstacle_height'] <= 0.05, f'{cm}.{layer}.{src}'
        assert sp['min_obstacle_height'] < sensors['lidar_z']
        assert sp['max_obstacle_height'] >= sensors['camera_z'] + 0.5
        assert lp['max_obstacle_height'] >= sp['max_obstacle_height']
        assert sp['observation_persistence'] == 0.0
    for cm, layer, src, lp, sp in clearing:
        # 소거 소스는 바닥 점을 담아야 지나간 물체 셀을 지운다 (VoxelLayer 잔상 실측, costmap.md §5)
        assert sp['min_obstacle_height'] < 0.0, f'{cm}.{layer}.{src}'
        assert not sp['marking']


def test_scan_filter_node_feeds_costmaps(params):
    f = params['costmap_scan_filter_node']['ros__parameters']
    assert f['input_topic'] == 'scan_filtered'
    assert f['output_topic'] == 'scan_costmap'
    assert f['half_window'] >= 3 and 0.05 <= f['range_gate'] <= 0.3
    assert f['static_output_topic'] == 'scan_costmap_static'
    # amr_perception pointcloud_filter_node 출력
    assert f['cloud_input_topic'] == 'camera/depth/points_filtered'
    assert f['cloud_output_topic'] == 'camera/depth/points_static'
    assert f['exclude_dynamic'] and f['dynamic_radius'] >= 0.30       # 사람 발자국 반경 이상


def test_local_costmap_resolution_resolves_aisle_margin(params):
    # 0.60 m 통로 측면 여유 0.10 m: 격자 양자화 손실이 셀 하나 ≤ 0.025 m 여야 한다 (costmap.md §3)
    rp = params['local_costmap']['local_costmap']['ros__parameters']
    assert rp['resolution'] <= 0.025
    assert rp['footprint_padding'] == 0.0
    half = min(rp['width'], rp['height']) / 2.0
    dwa = params['controller_server']['ros__parameters']['DWA']
    assert dwa['path_horizon'] <= half


def test_global_sensor_marks_get_narrow_inflation(params):
    # 정적 지도는 넓게(A* 가 넓은 통로 선호), 센서 장애물은 좁게 — 지나가는 사람 주위로 1 m 넘게 돌지 않게
    rp = params['global_costmap']['global_costmap']['ros__parameters']
    plugins = rp['plugins']
    infl = [p for p in plugins if rp[p]['plugin'] == 'nav2_costmap_2d::InflationLayer']
    assert len(infl) == 2
    first, last = infl
    assert plugins.index(first) < plugins.index('obstacle_layer') < plugins.index(last)
    assert plugins.index('depth_layer') < plugins.index(last) and plugins[-1] == last
    assert rp[last]['inflation_radius'] < rp[first]['inflation_radius']
    assert rp[last]['inflation_radius'] >= 0.361 + 0.1   # 외접 반경 + 여유
    # 지역은 하나 (모든 장애물에 같은 여유 비용: DWA J_clear)
    lp = params['local_costmap']['local_costmap']['ros__parameters']
    assert [p for p in lp['plugins'] if lp[p]['plugin'] == 'nav2_costmap_2d::InflationLayer'] == \
        [lp['plugins'][-1]]


def test_predicted_layer_is_global_only_and_marks_forward(params):
    """
    명세 4.7 "경로를 재계획(Replanning)하여 회피" / 9장 "예측 기반 회피".

    계획 충돌 트랙의 **예측** 점유를 전역 코스트맵에만 넣는다. 지역은 종전대로 DWA 의
    VO/TTC 가 맡는다 — 지역이 이 토픽을 구독하면 두 계층이 같은 것을 두 번 피한다.

    현재 위치가 아니라 예측을 쓰는 이유(기하): 마킹이 경로를 막는 것은 보행자가 경로에서
    (robot_radius 0.361 + r_obs 0.25) = 0.611 m 안일 때이고 v = 1.0 이면 TTC 0.61 s 인데,
    재계획에는 코스트맵 0.2 s + BT 1.0 s + A* 0.013 s = 1.21 s 가 든다. 0.61 < 1.21 이라
    현재 위치 마킹으로는 원리적으로 늦는다.
    """
    pred = [s for s in _sources(params) if 'predicted' in s[1]]
    assert pred, 'predicted_layer 가 없다'
    assert {s[0] for s in pred} == {'global_costmap'}, '지역 코스트맵이 예측을 구독하면 안 된다'
    for cm, layer, src, lp, sp in pred:
        assert sp['data_type'] == 'PointCloud2'
        assert sp['marking'] is True
        assert sp['clearing'] is False, '예측은 소거하지 않는다 (발행이 멈추면 만료된다)'
        assert sp['observation_persistence'] == 0.0, '옛 예측이 남으면 안 된다'


def test_admit_ttc_leaves_time_to_replan(params):
    """
    재진입 임계가 재계획 파이프라인 지연보다 커야 의미가 있다.

    지연 = 전역 코스트맵 갱신(1/update_frequency) + BT 재계획 주기 + A* 최대 계획 시간.
    이보다 늦게 재진입하면 예측을 넣어도 새 경로가 나올 시간이 없다.
    """
    f = params['costmap_scan_filter_node']['ros__parameters']
    admit = f['admit_ttc']
    if admit <= 0.0:
        return                                  # 꺼져 있으면 검사하지 않는다
    g = params['global_costmap']['global_costmap']['ros__parameters']
    lag = 1.0 / g['update_frequency'] + 1.0 + 0.4   # 코스트맵 + BT 1 Hz + A* max_planning_time
    assert admit > lag, f'admit_ttc {admit} 이 재계획 지연 {lag:.2f} s 보다 작다'
    # 예측 구간이 차단 반경을 충분히 쓸고 지나가야 경로를 실제로 가로막는다
    assert f['predict_horizon'] * 1.0 >= 3 * (0.361 + 0.25)
