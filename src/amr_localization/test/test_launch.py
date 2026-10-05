"""localization.launch.py 구성 테스트: 공유 설정 읽기와 모드별 노드 구성."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitution import Substitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest
import yaml

HERE = Path(__file__).resolve().parent
WS_CONFIG = HERE.parents[2] / 'config'


def load_module():
    spec = importlib.util.spec_from_file_location(
        'localization_launch', HERE.parent / 'launch' / 'localization.launch.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def context_for(mod, **overrides):
    ctx = LaunchContext()
    for entity in mod.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            ctx.launch_configurations[entity.name] = perform_substitutions(
                ctx, entity.default_value)
    ctx.launch_configurations['config_dir'] = str(WS_CONFIG)
    ctx.launch_configurations.update(overrides)
    return ctx


def executables(actions):
    return sorted(a.node_executable for a in actions if isinstance(a, Node))


def test_shared_parameters_from_workspace_config():
    mod = load_module()
    shared = mod.shared_parameters(str(WS_CONFIG))
    assert shared['wheel']['wheel_radius'] == pytest.approx(0.0825)
    assert shared['wheel']['wheel_separation'] == pytest.approx(0.36)
    assert shared['wheel']['ticks_per_revolution'] == 4096
    assert shared['wheel']['publish_rate'] == pytest.approx(50.0)
    assert shared['imu']['gyro_noise_stddev'] == pytest.approx(0.0002)
    assert shared['scan']['range_max'] == pytest.approx(25.0)
    # 섀도우 단계의 잡음 조건은 LiDAR 잡음 모델(sensors.yaml noise_stddev)을 따른다
    assert shared['scan']['shadow_range_noise_stddev'] == pytest.approx(0.03)
    assert shared['lidar_offset'] == pytest.approx([0.15, 0.0, 0.0])
    assert shared['range_noise'] == pytest.approx(0.03)
    assert mod.shared_parameters('/nonexistent') == {
        'wheel': {}, 'imu': {}, 'scan': {}, 'lidar_offset': None, 'range_noise': None}


@pytest.mark.parametrize('mode, expected', [
    ('odom', ['ekf_node', 'imu_filter_node', 'scan_filter_node', 'wheel_odometry_node']),
    # slam 모드는 지도 저장용 map_saver_server 와 그 lifecycle_manager 를 함께 띄운다
    # (slam_toolbox 내부 saver 는 이름공간 아래에서 쓸 수 없다 — launch 주석)
    ('slam', ['async_slam_toolbox_node', 'ekf_node', 'imu_filter_node', 'lifecycle_manager',
              'map_saver_server', 'scan_filter_node', 'wheel_odometry_node']),
    ('localization', ['amcl', 'amcl_map_adapter', 'ekf_node', 'ekf_node', 'imu_filter_node',
                      'kidnap_monitor_node', 'lifecycle_manager', 'lifecycle_manager',
                      'map_server', 'scan_filter_node', 'scan_matcher_node',
                      'wheel_odometry_node']),
])
def test_modes_build_expected_nodes(mode, expected):
    mod = load_module()
    actions = mod._setup(context_for(mod, mode=mode, bias_estimation_time='5.0'))
    assert executables(actions) == expected


def test_localization_without_map_server_or_monitor():
    mod = load_module()
    actions = mod._setup(context_for(mod, start_map_server='false', use_kidnap_monitor='false',
                                     robot_name=''))
    names = executables(actions)
    assert 'map_server' not in names and 'kidnap_monitor_node' not in names
    off = executables(mod._setup(context_for(mod, use_scan_matcher='false')))
    assert 'scan_matcher_node' not in off and 'amcl' in off


def plain(value):
    # launch_ros 가 정규화한 값(Substitution 튜플)을 평범한 값으로
    if isinstance(value, tuple) and value and all(isinstance(v, Substitution) for v in value):
        return perform_substitutions(LaunchContext(), list(value))
    return value


def amcl_params(actions):
    amcl = [a for a in actions if isinstance(a, Node) and a.node_executable == 'amcl'][0]
    merged = {}
    for p in amcl._Node__parameters:   # launch_ros 내부 속성 (정규화된 파라미터 목록)
        if isinstance(p, dict):
            # 문자열 값은 launch_ros 가 YAML 로 직렬화해 둔다 ('map_amcl\n...\n')
            merged.update({plain(k): yaml.safe_load(plain(v)) if isinstance(plain(v), str)
                           else plain(v) for k, v in p.items()})
    return merged


def test_amcl_half_cell_fix_switch():
    # 기본: AMCL 은 반 셀 보정 맵(map_amcl, 상대 이름)을 구독; 끄면 amcl.yaml 의 /map 그대로
    mod = load_module()
    on = mod._setup(context_for(mod))
    assert amcl_params(on).get('map_topic') == 'map_amcl'
    off = mod._setup(context_for(mod, amcl_half_cell_fix='false'))
    assert 'amcl_map_adapter' not in executables(off)
    assert 'map_topic' not in amcl_params(off)


def test_invalid_mode_and_missing_ekf(tmp_path):
    mod = load_module()
    with pytest.raises(RuntimeError):
        mod._setup(context_for(mod, mode='mapping'))
    with pytest.raises(RuntimeError):
        mod._setup(context_for(mod, config_dir=str(tmp_path)))
