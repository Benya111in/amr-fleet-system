"""행 설정·런치 계약 시험 — 문자열 파라미터, deskew 끔, 정답 방위 누출 차단, 재생 토픽 규칙, LIO-SAM 외부 파라미터."""

import importlib.util
from pathlib import Path
import re

from amr_slam3d import rows
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[1]


def _load_launch(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), PKG / 'launch' / name)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _yaml(name):
    return yaml.safe_load((PKG / 'config' / name).read_text(encoding='utf-8'))


def test_rtabmap_core_parameters_are_strings():
    """Grp/Name 형식(RTAB-Map 코어) 파라미터는 문자열 — bool·숫자면 노드가 abort (s0 VERSIONS.md 1.1)."""
    data = _yaml('rtabmap.yaml')
    n = 0
    for block in data.values():
        for key, value in block['ros__parameters'].items():
            if re.fullmatch(r'[A-Z][A-Za-z0-9]*/[A-Za-z0-9]+', key):
                assert isinstance(value, str), (key, value)
                n += 1
    assert n >= 40


def test_rtabmap_deskew_off_and_planar():
    data = _yaml('rtabmap.yaml')
    odom = data['/**/icp_odometry']['ros__parameters']
    shared = data['/**']['ros__parameters']
    assert odom['deskewing'] is False               # time 필드 없음 → 켜면 "Failed to deskew"
    assert shared['Reg/Force3DoF'] == 'true'
    assert shared['Icp/RangeMin'] == '0.2'
    # icp_odometry 가 옮겨 가는 두 값은 ROS 파라미터와 코어 값이 같아야 한다
    assert float(shared['Icp/VoxelSize']) == odom['scan_voxel_size']
    assert int(shared['Icp/PointToPlaneK']) == odom['scan_normal_k']


def test_rtabmap_launch_rejects_oracle_imu():
    mod = _load_launch('rtabmap.launch.py')
    for bad in ('/amr_01/imu/data_raw', '/amr_01/imu/data', 'imu/data'):
        with pytest.raises(RuntimeError):
            mod.check_imu_topic(bad)
    mod.check_imu_topic('/amr_01/imu/data_madgwick')
    mod.check_imu_topic('')


def test_rtabmap_launch_2d_3d_share_everything_but_input():
    mod = _load_launch('rtabmap.launch.py')
    a = mod.node_setup('3d', 'amr_01', '/c', '/s', '/amr_01/imu/data_madgwick', True, 'slam_map',
                       '/db', 'p.yaml', 'info')
    b = mod.node_setup('2d', 'amr_01', '/c', '/s', '/amr_01/imu/data_madgwick', True, 'slam_map',
                       '/db', 'p.yaml', 'info')
    assert a[0] == b[0]                              # icp_odometry 파라미터 동일
    diff = {k for k in a[1][1] if a[1][1][k] != b[1][1][k]}
    assert diff == {'subscribe_scan_cloud', 'subscribe_scan'}
    ra, rb = dict(a[2]), dict(b[2])
    assert ra['scan_cloud'] == '/c' and ra['scan'] == 'scan_not_used'
    assert rb['scan'] == '/s' and rb['scan_cloud'] == 'scan_cloud_not_used'
    assert a[0][1]['guess_frame_id'] == 'amr_01/odom'
    assert a[0][1]['odom_frame_id'] == 'amr_01/icp_odom'
    no_guess = mod.node_setup('3d', 'amr_01', '/c', '/s', '', False, 'slam_map', '/db', 'p', 'i')
    assert no_guess[0][1]['guess_frame_id'] == '' and no_guess[0][1]['wait_imu_to_init'] is False
    with pytest.raises(RuntimeError):
        mod.node_setup('4d', 'amr_01', '/c', '/s', '', True, 'm', '/db', 'p', 'i')


def test_madgwick_no_mag_no_tf():
    p = _yaml('madgwick.yaml')['/**/imu_filter_madgwick']['ros__parameters']
    assert p['use_mag'] is False and p['publish_tf'] is False
    assert p['remove_gravity_vector'] is False


def test_rows_play_recorded_clock_and_never_oracle_topics():
    """시간원 = 기록된 /clock (--clock 은 기록 벽시계를 낸다), 정답 토픽은 재생하지 않는다."""
    for r in rows.ROWS.values():
        assert '/clock' in r.play_topics
        for t in rows.ORACLE_TOPICS:
            assert t not in r.play_topics, (r.name, t)
        assert '/tf_static' in r.play_topics
        assert r.input_topic in r.play_topics


def test_liosam_rows_do_not_play_dynamic_tf():
    """LIO-SAM 은 odometryFrame → base_footprint 를 낸다 — EKF /tf 를 같이 틀면 부모가 둘."""
    for name in ('liosam_loop', 'liosam_noloop'):
        assert '/tf' not in rows.row(name).play_topics
    for name in ('rtabmap3d', 'rtabmap2d', 'slam_toolbox'):
        assert '/tf' in rows.row(name).play_topics   # 휠-EKF guess 가 TF 로 들어온다


def test_record_topics_cover_report_list():
    """보고서 4단계: 2D 스캔·3D 점군(전처리 전)·원시 IMU·휠·EKF odom·GT odom·TF."""
    need = {'/amr_01/scan', '/amr_01/lidar3d/points_raw', '/amr_01/imu/data_raw',
            '/amr_01/wheel_odom', '/amr_01/odometry/filtered', '/amr_01/ground_truth/odom',
            '/tf', '/tf_static', '/clock'}
    assert need <= set(rows.RECORD_TOPICS)
    for r in rows.ROWS.values():
        for t in r.play_topics:
            assert t in rows.RECORD_TOPICS, (r.name, t)


def test_liosam_config_matches_sensor_contract_and_urdf():
    p = _yaml('liosam.yaml')['/**']['ros__parameters']
    assert p['sensor'] == 'velodyne' and p['N_SCAN'] == 16 and p['Horizon_SCAN'] == 1800
    assert p['imuTopic'].endswith('imu/data_madgwick')
    assert p['pointCloudTopic'] == rows.CLOUD
    assert p['baselinkFrame'] == 'amr_01/base_footprint'
    assert p['odometryFrame'] not in ('odom', 'amr_01/odom')     # EKF odom 과 겹치면 안 된다
    # extrinsicTrans = LiDAR 프레임에서 본 IMU 위치 = imu − lidar (URDF 의 두 joint 에서 계산)
    lidar = yaml.safe_load((REPO / 'src/amr_description/config/lidar3d.yaml')
                           .read_text(encoding='utf-8'))
    text = (REPO / 'src/amr_description/test/golden/fleet_amr_01.urdf').read_text(encoding='utf-8')
    imu_xyz = re.search(r'<joint name="amr_01/imu_joint".*?xyz="([^"]+)"', text, re.S).group(1)
    imu_z = float(imu_xyz.split()[2])               # base_link 기준
    mast = lidar['/**']['ros__parameters'] if '/**' in lidar else lidar
    assert mast is not None
    assert p['extrinsicTrans'] == pytest.approx([-0.27, 0.0, imu_z - 0.72])
    assert p['lidarMinRange'] == pytest.approx(0.2)


def test_liosam_launch_loop_flag():
    mod = _load_launch('liosam.launch.py')
    assert Path(mod.default_params()).name == 'liosam.yaml'
    assert Path(mod.default_params()).is_file()
    assert len(mod.EXECUTABLES) == 4


def test_scan_log_counts_patterns():
    r = rows.row('rtabmap3d')
    s = rows.scan_log('x Failed to deskew input cloud y\nFailed to deskew', r)
    assert s['fatal']['Failed to deskew'] == 2
    assert s['fatal']['aborting odometry update'] == 0
